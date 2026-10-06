// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2011 Liteon-semi Corporation
 *
 * Contact: YC Hou <yc_hou@liteon-semi.com>
 *
 * AL3201 ambient light sensor driver.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 06/28/11 YC       Original Creation (Test version:1.0)
 * 06/28/11 YC       Change dev name to dyna for demo purpose (ver 1.5).
 * 08/29/11 YC       Add engineer mode. Change version to 1.6.
 * 09/26/11 YC       Add calibration compensation function and add not power up
 *                   prompt. Change version to 1.7.
 * 03/06/17 KP       Rework to build on kernel version 4.4 and change version to 1.8
 * 05/26/21 MJ       update for 5.10, error and warning cleanup, version bump to 1.9
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/math64.h>

#define AL3201_DRV_NAME		"al3201"
#define DRIVER_VERSION		"1.9"

#define AL3201_NUM_CACHABLE_REGS	5

#define AL3201_RAN_COMMAND	0x01
#define AL3201_RAN_MASK		0x01
#define AL3201_RAN_SHIFT	(0)

#define AL3201_RT_COMMAND	0x02
#define AL3201_RT_MASK		0x03
#define AL3201_RT_SHIFT		(0)

#define AL3201_POW_COMMAND	0x00
#define AL3201_POW_MASK		0x03
#define AL3201_POW_UP		0x03
#define AL3201_POW_DOWN		0x00
#define AL3201_POW_SHIFT	(0)

#define	AL3201_ADC_LSB		0x0c
#define	AL3201_ADC_MSB		0x0d

/* 0: 0 to 32768 lux, 1: 0 to 8192 lux */
#define AL3201_RANGE_HIGH_RES	1
/* 200ms low pass filter */
#define AL3201_RT_200MS		0x02
/* time until the first sample is valid after power up */
#define AL3201_POWER_UP_MS	180
/* counts at or below this are noise */
#define AL3201_DARK_COUNT	5

static u8 al3201_reg[AL3201_NUM_CACHABLE_REGS] = {
	0x00, 0x01, 0x02, 0x0c, 0x0d
};

/* micro lux per count, indexed by range */
static const int al3201_ulux_per_count[2] = { 500000, 125000 };

struct al3201_data {
	struct i2c_client *client;
	struct mutex lock; /* protects register access and power state */
	u8 reg_cache[AL3201_NUM_CACHABLE_REGS];
};

static int cali = 100;

static int __al3201_read_reg(struct al3201_data *data,
			     u32 reg, u8 mask, u8 shift)
{
	return (data->reg_cache[reg] & mask) >> shift;
}

/* must be called with data->lock held */
static int __al3201_write_reg(struct al3201_data *data,
			      u32 reg, u8 mask, u8 shift, u8 val)
{
	int ret = 0;
	u8 tmp;

	if (reg >= AL3201_NUM_CACHABLE_REGS)
		return -EINVAL;

	tmp = data->reg_cache[reg];
	tmp &= ~mask;
	tmp |= val << shift;

	ret = i2c_smbus_write_byte_data(data->client, reg, tmp);
	if (!ret)
		data->reg_cache[reg] = tmp;

	return ret;
}

static int al3201_get_range(struct al3201_data *data)
{
	return __al3201_read_reg(data, AL3201_RAN_COMMAND,
				 AL3201_RAN_MASK, AL3201_RAN_SHIFT);
}

static int al3201_set_range(struct al3201_data *data, int range)
{
	return __al3201_write_reg(data, AL3201_RAN_COMMAND,
				  AL3201_RAN_MASK, AL3201_RAN_SHIFT, range);
}

static int al3201_set_response_time(struct al3201_data *data, int time)
{
	return __al3201_write_reg(data, AL3201_RT_COMMAND,
				  AL3201_RT_MASK, AL3201_RT_SHIFT, time);
}

static int al3201_set_power_state(struct al3201_data *data, int state)
{
	return __al3201_write_reg(data, AL3201_POW_COMMAND,
				  AL3201_POW_MASK, AL3201_POW_SHIFT,
				  state ? AL3201_POW_UP : AL3201_POW_DOWN);
}

/*
 * The chip is kept powered down and only woken up for a measurement. The
 * configuration is written every time, the registers may have been reset
 * while the chip was not in use.
 */
static int al3201_get_adc_value(struct al3201_data *data)
{
	int lsb, msb, ret;

	mutex_lock(&data->lock);

	ret = al3201_set_range(data, AL3201_RANGE_HIGH_RES);
	if (ret < 0)
		goto out_unlock;

	ret = al3201_set_response_time(data, AL3201_RT_200MS);
	if (ret < 0)
		goto out_unlock;

	ret = al3201_set_power_state(data, 1);
	if (ret < 0)
		goto out_unlock;

	msleep(AL3201_POWER_UP_MS);

	lsb = i2c_smbus_read_byte_data(data->client, AL3201_ADC_LSB);
	msb = i2c_smbus_read_byte_data(data->client, AL3201_ADC_MSB);

	al3201_set_power_state(data, 0);

	if (lsb < 0)
		ret = lsb;
	else if (msb < 0)
		ret = msb;
	else
		ret = msb << 8 | lsb;

out_unlock:
	mutex_unlock(&data->lock);
	return ret;
}

static const struct iio_chan_spec al3201_channels[] = {
	{
		.type   = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
				| BIT(IIO_CHAN_INFO_PROCESSED),
	}
};

static ssize_t in_illuminance_calibration_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", cali);
}

static ssize_t in_illuminance_calibration_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct al3201_data *data = iio_priv(indio_dev);
	int stdls, adc;
	u64 ulux;
	char tmp[10] = {0};

	int n = sscanf(buf, "%d %9s", &stdls, tmp);

	if (n < 1 || n > 2)
		return -EINVAL;

	if (!strncmp(tmp, "-setcv", 6)) {
		cali = stdls;
		return count;
	}

	if (stdls < 0) {
		dev_err(dev, "Std light source: [%d] < 0\n", stdls);
		return -EINVAL;
	}

	adc = al3201_get_adc_value(data);
	if (adc < 0)
		return adc;
	if (adc == 0)
		return -ENODATA;

	ulux = (u64)adc * al3201_ulux_per_count[al3201_get_range(data)];
	cali = div64_u64((u64)stdls * 100 * 1000000, ulux);

	return count;
}

static DEVICE_ATTR_RW(in_illuminance_calibration);

static struct attribute *al3201_attributes[] = {
	&dev_attr_in_illuminance_calibration.attr,
	NULL,
};

static const struct attribute_group al3201_attribute_group = {
	.attrs = al3201_attributes,
};

static int al3201_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	struct al3201_data *data = iio_priv(indio_dev);
	u64 ulux;
	u32 rem;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		ret = al3201_get_adc_value(data);
		if (ret < 0)
			return ret;
		if (ret <= AL3201_DARK_COUNT)
			ret = 0;

		ulux = (u64)ret * al3201_ulux_per_count[al3201_get_range(data)];
		ulux = div_u64(ulux * cali, 100);
		*val = div_u64_rem(ulux, 1000000, &rem);
		*val2 = rem;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_RAW:
		ret = al3201_get_adc_value(data);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int al3201_init(struct al3201_data *data)
{
	int i, ret;

	/*
	 * read all the registers once to fill the cache.
	 * if one of the reads fails, we consider the init failed
	 */
	for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++) {
		int v = i2c_smbus_read_byte_data(data->client, al3201_reg[i]);

		if (v < 0)
			return -ENODEV;

		data->reg_cache[i] = v;
	}

	/* set defaults, the chip is powered up on demand */
	mutex_lock(&data->lock);
	ret = al3201_set_range(data, AL3201_RANGE_HIGH_RES);
	if (!ret)
		ret = al3201_set_response_time(data, AL3201_RT_200MS);
	if (!ret)
		ret = al3201_set_power_state(data, 0);
	mutex_unlock(&data->lock);

	return ret;
}

static const struct iio_info al3201_info = {
	.read_raw       = al3201_read_raw,
	.attrs          = &al3201_attribute_group,
};

static int al3201_probe(struct i2c_client *client)
{
	struct al3201_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	mutex_init(&data->lock);

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &al3201_info;
	indio_dev->name = AL3201_DRV_NAME;
	indio_dev->channels = al3201_channels;
	indio_dev->num_channels = ARRAY_SIZE(al3201_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = al3201_init(data);
	if (ret < 0) {
		dev_err(&client->dev, "al3201 chip init failed\n");
		return ret;
	}
	return devm_iio_device_register(&client->dev, indio_dev);
}

static void al3201_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct al3201_data *data = iio_priv(indio_dev);

	mutex_lock(&data->lock);
	al3201_set_power_state(data, 0);
	mutex_unlock(&data->lock);
}

static const struct i2c_device_id al3201_id[] = {
	{ "al3201", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, al3201_id);

static const struct of_device_id al3201_of_match[] = {
	{ .compatible = "dynaimage,al3201", .data = NULL, },
	{ }
};
MODULE_DEVICE_TABLE(of, al3201_of_match);

static struct i2c_driver al3201_driver = {
	.driver = {
		.name = AL3201_DRV_NAME,
		.of_match_table = of_match_ptr(al3201_of_match),
	},
	.probe = al3201_probe,
	.remove = al3201_remove,
	.id_table = al3201_id,
};

module_i2c_driver(al3201_driver);

MODULE_AUTHOR("YC Hou, LiteOn-semi corporation.");
MODULE_DESCRIPTION("AL3201 light sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);
