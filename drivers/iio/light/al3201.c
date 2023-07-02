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
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#define AL3201_DRV_NAME			"al3201"
#define DRIVER_VERSION			"1.9"

#define AL3201_POW_COMMAND		0x00
#define AL3201_POW_MASK			0x03
#define AL3201_POW_UP			0x03
#define AL3201_POW_DOWN			0x00
#define AL3201_POW_SHIFT		(0)

#define AL3201_RAN_COMMAND		0x01
#define AL3201_RAN_MASK			0x01
#define AL3201_RAN_LOW_RES		0x00 /* 0 to 32768 lux */
#define AL3201_RAN_HIGH_RES		0x01 /* 0 to 8192 lux */
#define AL3201_RAN_SHIFT		(0)

#define	AL3201_ADC_LSB			0x0c
#define	AL3201_ADC_MSB			0x0d
#define AL3201_MAX_REGISTER		0x0e

#define AL3201_SCALE_AVAILABLE		"0.032768 0.008192"

struct al3201_data {
	struct i2c_client *client;
	struct regmap *regmap;
};

static const unsigned int al3201_scales[][2] = {
	{0, 32768},
	{0, 8192}
};

static int al3201_read_reg(struct al3201_data *data,
			   u8 reg, u8 mask, u8 shift, unsigned int *result)
{
	unsigned int tmp;
	int ret;

	ret = regmap_read(data->regmap, reg, &tmp);
	if (ret) {
		dev_err(&data->client->dev, "error reading regmap, ret=%d\n", ret);
		return ret;
	}
	*result = (tmp & mask) >> shift;
	return ret;
}

static int al3201_write_reg(struct al3201_data *data,
			    u8 reg, u8 mask, u8 shift, unsigned int val)
{
	return regmap_write(data->regmap, reg, (val << shift) & mask);
}

static int al3201_get_range(struct al3201_data *data, unsigned int *val)
{
	unsigned int tmp;
	int ret;

	ret = al3201_read_reg(data, AL3201_RAN_COMMAND,
			      AL3201_RAN_MASK, AL3201_RAN_SHIFT, &tmp);

	if (!ret)
		*val = al3201_scales[tmp][1];

	return ret;
}

static int al3201_set_range(struct al3201_data *data, unsigned int range)
{
	return al3201_write_reg(data, AL3201_RAN_COMMAND,
				  AL3201_RAN_MASK, AL3201_RAN_SHIFT,
				  range);
}

static int al3201_get_power_state(struct al3201_data *data, int *val)
{
	unsigned int tmp;
	int ret;

	ret = al3201_read_reg(data, AL3201_POW_COMMAND,
			      AL3201_POW_MASK, AL3201_POW_SHIFT, &tmp);
	if (!ret)
		*val = tmp > 0 ? 1 : 0;

	return ret;
}

static int al3201_set_power_state(struct al3201_data *data, int state)
{
	return al3201_write_reg(data, AL3201_POW_COMMAND,
				AL3201_POW_MASK, AL3201_POW_SHIFT,
				state ? AL3201_POW_UP : AL3201_POW_DOWN);
}

static const struct iio_chan_spec al3201_channels[] = {
	{
		.type   = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
				| BIT(IIO_CHAN_INFO_SCALE)
				| BIT(IIO_CHAN_INFO_ENABLE),
	}
};

static IIO_CONST_ATTR(in_illuminance_scale_available, AL3201_SCALE_AVAILABLE);

static struct attribute *al3201_attributes[] = {
	&iio_const_attr_in_illuminance_scale_available.dev_attr.attr,
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
	int ret;
	u16 tmp;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		/*
		 * ALS ADC value is stored in two adjacent registers:
		 * - low byte of output is stored at AL3201_ADC_LSB
		 * - high byte of output is stored at AL3201_ADC_LSB + 1
		 */
		ret = regmap_bulk_read(data->regmap, AL3201_ADC_LSB, &tmp, 2);
		if (ret)
			return ret;
		*val = le16_to_cpu(tmp);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		ret = al3201_get_range(data, val2);
		if (ret)
			return ret;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_ENABLE:
		ret = al3201_get_power_state(data, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int al3201_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val,
			    int val2, long mask)
{
	struct al3201_data *data = iio_priv(indio_dev);
	unsigned int i;
	int err;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		for (i = 0; i < ARRAY_SIZE(al3201_scales); i++) {
			if (val == al3201_scales[i][0] &&
			    val2 == al3201_scales[i][1]) {
				err = al3201_set_range(data, i);
				break;
			}
		}
		break;
	case IIO_CHAN_INFO_ENABLE:
		err = al3201_set_power_state(data, val);
		break;
	default:
		err = -EINVAL;
		break;
	}

	return err;
}

static int al3201_init(struct al3201_data *data)
{
	int ret;

	ret = al3201_set_range(data, AL3201_RAN_LOW_RES);
	if (ret)
		return ret;

	ret = al3201_set_power_state(data, 1);
	return ret;
}

static const struct iio_info al3201_info = {
	.read_raw       = al3201_read_raw,
	.write_raw      = al3201_write_raw,
	.attrs          = &al3201_attribute_group,
};

static bool al3201_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AL3201_ADC_LSB:
	case AL3201_ADC_MSB:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config al3201_regmap_config = {
	.name = "al3201_regmap",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AL3201_MAX_REGISTER,
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = al3201_is_volatile_reg,
};

static int al3201_probe(struct i2c_client *client)
{
	struct al3201_data *data;
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	regmap = devm_regmap_init_i2c(client, &al3201_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "regmap init failed.\n");
		return PTR_ERR(regmap);
	}
	data->regmap = regmap;

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

static int al3201_runtime_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct al3201_data *data = iio_priv(indio_dev);
	int ret = 0;

	ret = al3201_set_power_state(data, 1);
	return ret;
}

static int al3201_runtime_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct al3201_data *data = iio_priv(indio_dev);
	int ret = 0;

	ret = al3201_set_power_state(data, 0);
	return ret;
}

static DEFINE_RUNTIME_DEV_PM_OPS(al3201_pm_ops, al3201_runtime_suspend,
				 al3201_runtime_resume, NULL);

static const struct i2c_device_id al3201_id[] = {
	{ "al3201", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, al3201_id);

static const struct of_device_id al3201_of_match[] = {
	{ .compatible = "liteon,al3201", .data = NULL, },
	{ }
};
MODULE_DEVICE_TABLE(of, al3201_of_match);

static struct i2c_driver al3201_driver = {
	.driver = {
		.name = AL3201_DRV_NAME,
		.pm = pm_ptr(&al3201_pm_ops),
		.of_match_table = of_match_ptr(al3201_of_match),
	},
	.probe_new = al3201_probe,
	.id_table = al3201_id,
};

module_i2c_driver(al3201_driver);

MODULE_AUTHOR("YC Hou, LiteOn-semi corporation.");
MODULE_AUTHOR("Martin Jücker <martin.juecker@gmail.com>");
MODULE_DESCRIPTION("AL3201 light sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
