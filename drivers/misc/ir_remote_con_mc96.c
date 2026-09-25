// SPDX-License-Identifier: GPL-2.0
/*
 * ir_remote_con_mc96.c - ABOV MC96FR IR blaster driver
 *
 * Copyright (C) 2012 Samsung Electronics
 *
 * Ported from the Samsung p4note (GT-N8000 family) vendor kernel
 * driver drivers/irda/ir_remote_con_mc96.c to the mainline GPIO
 * descriptor / regulator / devicetree APIs. The chip sits on a
 * bit-banged (i2c-gpio) I2C bus and is controlled through a wake
 * GPIO, a power regulator and an ack/status GPIO that the driver
 * polls after every transfer -- none of that is a real interrupt
 * line, so it is modelled as a plain input GPIO rather than an IRQ.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include "mc96_fw.h"

#define MC96_MAX_SIZE		2048
#define MC96_READ_LENGTH	8
#define MC96_DUMMY		0xffff

/* keep the FW resident and the chip in standby between transfers */
#define MC96_USE_STOP_MODE

static struct class *sec_class;

struct mc96_ir_data {
	struct i2c_client	*client;
	struct gpio_desc	*wake_gpio;
	struct gpio_desc	*ack_gpio;
	struct regulator	*vdd;
	struct device		*sec_dev;
	struct mutex		mutex;

	bool	vdd_on;
	char	signal[MC96_MAX_SIZE];
	int	count;
	int	dev_id;
	int	ir_freq;
	int	ir_sum;
	bool	on_off;

	int	count_number;
	int	ack_number;
};

static void mc96_wake_en(struct mc96_ir_data *data, bool onoff)
{
	gpiod_set_value_cansleep(data->wake_gpio, onoff);
}

static void mc96_vdd_onoff(struct mc96_ir_data *data, bool onoff)
{
	int ret;

	if (onoff == data->vdd_on)
		return;

	if (onoff) {
		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"failed to enable vdd: %d\n", ret);
			return;
		}
	} else {
		regulator_disable(data->vdd);
	}
	data->vdd_on = onoff;
}

/*
 * Read the 8 byte status frame. Returns 0 only if all 8 bytes arrived,
 * so callers never look at a stale or uninitialised buffer.
 */
static int mc96_read_status(struct mc96_ir_data *data, u8 *buf)
{
	struct i2c_client *client = data->client;
	int ret;

	ret = i2c_master_recv(client, buf, MC96_READ_LENGTH);
	if (ret < 0) {
		dev_err(&client->dev, "status read failed: %d\n", ret);
		return ret;
	}
	if (ret != MC96_READ_LENGTH) {
		dev_err(&client->dev, "short status read: %d\n", ret);
		return -EIO;
	}
	return 0;
}

/* boot mode frames carry a 16 bit sum of bytes 0..5 in bytes 6..7 */
static bool mc96_boot_checksum_ok(const u8 *buf)
{
	int k, checksum = 0;

	for (k = 0; k < 6; k++)
		checksum += buf[k];

	return checksum == (buf[6] << 8 | buf[7]);
}

static int mc96_fw_update(struct mc96_ir_data *data)
{
	struct i2c_client *client = data->client;
	int i, ret;
	bool ok;
	u8 buf[8];

	msleep(20);
	mc96_vdd_onoff(data, 0);
	msleep(20);
	mc96_vdd_onoff(data, 1);
	mc96_wake_en(data, 1);
	msleep(100);

	ret = mc96_read_status(data, buf);
	if (ret)
		ret = mc96_read_status(data, buf);
	if (ret) {
		dev_info(&client->dev, "%s: broken FW!\n", __func__);
		ret = MC96_DUMMY;
	} else {
		ret = buf[2] << 8 | buf[3];
	}

	if (ret != MC96_FW_VERSION) {
		dev_info(&client->dev,
			 "%s: chip: %04x, bin: %04x, need update!\n",
			 __func__, ret, MC96_FW_VERSION);
		mc96_vdd_onoff(data, 0);
		mc96_wake_en(data, 0);
		msleep(20);
		mc96_vdd_onoff(data, 1);
		msleep(100);

		if (mc96_read_status(data, buf) ||
		    !mc96_boot_checksum_ok(buf)) {
			dev_err(&client->dev, "ABOV IC bootcode broken\n");
			goto err_bootmode;
		}
		dev_info(&client->dev,
			 "%s: boot mode, FW download start! ret=%04x\n",
			 __func__, buf[6] << 8 | buf[7]);

		msleep(30);

		for (i = 0; i < MC96_FRAME_COUNT; i++) {
			int len = (i == MC96_FRAME_COUNT - 1) ? 6 :
							MC96_PACKET_SIZE;

			ret = i2c_master_send(client,
					&mc96_fw_binary[i * 70], len);
			if (ret < 0) {
				dev_err(&client->dev,
					"%s: update fail! frame %d, ret = %d\n",
					__func__, i, ret);
				goto err_bootmode;
			}
			msleep(30);
		}

		ok = !mc96_read_status(data, buf) &&
		     mc96_boot_checksum_ok(buf);

		msleep(20);

		if (!mc96_read_status(data, buf) &&
		    mc96_boot_checksum_ok(buf))
			ok = true;

		if (!ok) {
			dev_err(&client->dev, "FW checksum fail\n");
			goto err_bootmode;
		}
		dev_info(&client->dev, "%s: boot down complete\n", __func__);

		mc96_vdd_onoff(data, 0);
		msleep(20);
		mc96_vdd_onoff(data, 1);
		mc96_wake_en(data, 1);
		msleep(60);
		if (mc96_read_status(data, buf))
			ret = MC96_DUMMY;
		else
			ret = buf[2] << 8 | buf[3];
		dev_info(&client->dev,
			 "%s: user mode: upgrade FW version: %04x\n",
			 __func__, ret);
		mc96_wake_en(data, 0);
		mc96_vdd_onoff(data, 0);
		data->on_off = false;
		msleep(100);

		return 0;
	}

	dev_info(&client->dev, "%s: chip: %04x, bin: %04x, latest FW\n",
		 __func__, ret, MC96_FW_VERSION);
	mc96_wake_en(data, 0);
	mc96_vdd_onoff(data, 0);
	data->on_off = false;
	msleep(100);

	return 0;

err_bootmode:
	mc96_wake_en(data, 0);
	mc96_vdd_onoff(data, 0);
	data->on_off = false;
	return -EIO;
}

/* caller holds data->mutex */
static int mc96_read_device_info(struct mc96_ir_data *data)
{
	u8 buf[8];
	int ret;

	mc96_vdd_onoff(data, 1);
	mc96_wake_en(data, 1);
	msleep(60);
	ret = mc96_read_status(data, buf);
	if (!ret) {
		data->dev_id = buf[2] << 8 | buf[3];
		ret = data->dev_id;
	}

	mc96_wake_en(data, 0);
	mc96_vdd_onoff(data, 0);
	data->on_off = false;
	return ret;
}

static void mc96_add_checksum_length(struct mc96_ir_data *data, int count)
{
	int i, csum = 0;

	data->signal[0] = count >> 8;
	data->signal[1] = count & 0xff;

	for (i = 0; i < count; i++)
		csum += data->signal[i];

	data->signal[count] = csum >> 8;
	data->signal[count + 1] = csum & 0xff;
}

/* caller holds data->mutex; data->signal holds a validated frame */
static void mc96_send_signal(struct mc96_ir_data *data, int count)
{
	struct i2c_client *client = data->client;
	int buf_size = count + 2;
	int ret, sleep_timing, end_data, emission_time;
	int ack;

	if (data->count_number >= 100)
		data->count_number = 0;
	data->count_number++;

	if (data->on_off) {
		mc96_wake_en(data, 0);
		udelay(200);
		mc96_wake_en(data, 1);
		msleep(30);
	} else {
		mc96_vdd_onoff(data, 1);
		mc96_wake_en(data, 1);
		msleep(60);
		data->on_off = true;
	}

	mc96_add_checksum_length(data, count);

	ret = i2c_master_send(client, data->signal, buf_size);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err1 %d\n", __func__, ret);
		ret = i2c_master_send(client, data->signal, buf_size);
		if (ret < 0)
			dev_err(&client->dev, "%s: err2 %d\n", __func__, ret);
	}

	mdelay(10);

	if (gpiod_get_value_cansleep(data->ack_gpio)) {
		dev_info(&client->dev, "%s: %d checksum NG!\n", __func__,
			 data->count_number);
		ack = 1;
	} else {
		dev_info(&client->dev, "%s: %d checksum OK!\n", __func__,
			 data->count_number);
		ack = 2;
	}
	data->ack_number = ack;

	/*
	 * ir_sum is at most ~1000 pulses of 0xffff carrier cycles, so the
	 * scaled sum needs 64 bits.
	 */
	end_data = data->signal[count - 2] << 8 | data->signal[count - 1];
	emission_time = div_u64(1000ULL * (data->ir_sum - end_data),
				data->ir_freq) + 10;
	sleep_timing = emission_time - 130;
	if (sleep_timing > 0)
		msleep(sleep_timing);

	emission_time = div_u64(1000ULL * data->ir_sum, data->ir_freq) + 50;
	if (emission_time > 0)
		msleep(emission_time);

	if (gpiod_get_value_cansleep(data->ack_gpio)) {
		dev_info(&client->dev, "%s: %d sending IR OK!\n", __func__,
			 data->count_number);
		ack = 4;
	} else {
		dev_info(&client->dev, "%s: %d sending IR NG!\n", __func__,
			 data->count_number);
		ack = 2;
	}
	data->ack_number += ack;

#ifndef MC96_USE_STOP_MODE
	mc96_vdd_onoff(data, 0);
	data->on_off = false;
	mc96_wake_en(data, 0);
#endif
}

/*
 * Input is "freq,pulse,pulse,..." (commas or spaces), terminated by the
 * end of the string or a 0. The frequency takes 3 bytes of the frame,
 * each pulse 2, and 2 more are reserved for the trailing checksum.
 */
static ssize_t ir_send_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct mc96_ir_data *data = dev_get_drvdata(dev);
	unsigned int val;
	ssize_t ret = size;

	mutex_lock(&data->mutex);

	data->count = 2;
	data->ir_freq = 0;
	data->ir_sum = 0;

	while (*buf) {
		if (sscanf(buf, "%u", &val) != 1 || val == 0)
			break;

		if (data->count == 2) {
			if (val > 0xffffff) {
				ret = -EINVAL;
				goto out;
			}
			data->ir_freq = val;
			data->signal[2] = val >> 16;
			data->signal[3] = (val >> 8) & 0xff;
			data->signal[4] = val & 0xff;
			data->count += 3;
		} else {
			if (val > 0xffff ||
			    data->count + 4 > MC96_MAX_SIZE) {
				ret = -EINVAL;
				goto out;
			}
			data->ir_sum += val;
			data->signal[data->count] = val >> 8;
			data->signal[data->count + 1] = val & 0xff;
			data->count += 2;
		}

		while (*buf && *buf != ',' && *buf != ' ')
			buf++;
		while (*buf == ',' || *buf == ' ')
			buf++;
	}

	/* need a carrier frequency and at least one pulse */
	if (!data->ir_freq || data->count < 7) {
		ret = -EINVAL;
		goto out;
	}

	mc96_send_signal(data, data->count);

out:
	data->count = 2;
	data->ir_freq = 0;
	data->ir_sum = 0;
	mutex_unlock(&data->mutex);
	return ret;
}

static ssize_t ir_send_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct mc96_ir_data *data = dev_get_drvdata(dev);
	int i, len = 0;

	mutex_lock(&data->mutex);
	for (i = 5; i < MC96_MAX_SIZE - 1; i++) {
		if (data->signal[i] == 0 && data->signal[i + 1] == 0)
			break;
		len += sysfs_emit_at(buf, len, "%u,", data->signal[i]);
	}
	mutex_unlock(&data->mutex);

	return len;
}
static DEVICE_ATTR_RW(ir_send);

static ssize_t ir_send_result_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mc96_ir_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->ack_number == 6);
}
static DEVICE_ATTR_RO(ir_send_result);

static ssize_t check_ir_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct mc96_ir_data *data = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&data->mutex);
	ret = mc96_read_device_info(data);
	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", ret);
}
static DEVICE_ATTR_RO(check_ir);

static struct attribute *mc96_attrs[] = {
	&dev_attr_ir_send.attr,
	&dev_attr_ir_send_result.attr,
	&dev_attr_check_ir.attr,
	NULL,
};
ATTRIBUTE_GROUPS(mc96);

static int mc96_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mc96_ir_data *data;
	int i, ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EIO;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	mutex_init(&data->mutex);
	data->count = 2;

	data->wake_gpio = devm_gpiod_get(&client->dev, "wake", GPIOD_OUT_LOW);
	if (IS_ERR(data->wake_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(data->wake_gpio),
				      "failed to get wake gpio\n");

	data->ack_gpio = devm_gpiod_get(&client->dev, "ack", GPIOD_IN);
	if (IS_ERR(data->ack_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(data->ack_gpio),
				      "failed to get ack gpio\n");

	data->vdd = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(data->vdd))
		return dev_err_probe(&client->dev, PTR_ERR(data->vdd),
				      "failed to get vdd regulator\n");

	i2c_set_clientdata(client, data);

	for (i = 0; i < 6; i++) {
		ret = mc96_fw_update(data);
		if (!ret)
			break;
	}

	sec_class = class_create(THIS_MODULE, "sec");
	if (IS_ERR(sec_class)) {
		ret = PTR_ERR(sec_class);
		dev_err(&client->dev, "failed to create sec class: %d\n", ret);
		return ret;
	}

	data->sec_dev = device_create_with_groups(sec_class, NULL, 0, data,
						   mc96_groups, "sec_ir");
	if (IS_ERR(data->sec_dev)) {
		ret = PTR_ERR(data->sec_dev);
		dev_err(&client->dev, "failed to create sec_ir device: %d\n",
			ret);
		class_destroy(sec_class);
		return ret;
	}

	return 0;
}

static int mc96_remove(struct i2c_client *client)
{
	struct mc96_ir_data *data = i2c_get_clientdata(client);

	device_unregister(data->sec_dev);
	class_destroy(sec_class);
	mutex_destroy(&data->mutex);
	return 0;
}

static int __maybe_unused mc96_suspend(struct device *dev)
{
	struct mc96_ir_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	mc96_vdd_onoff(data, 0);
	data->on_off = false;
	mc96_wake_en(data, 0);
	mutex_unlock(&data->mutex);
	return 0;
}

static int __maybe_unused mc96_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(mc96_pm_ops, mc96_suspend, mc96_resume);

static const struct i2c_device_id mc96_id[] = {
	{ "mc96", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mc96_id);

static const struct of_device_id mc96_of_match[] = {
	{ .compatible = "abov,mc96" },
	{ }
};
MODULE_DEVICE_TABLE(of, mc96_of_match);

static struct i2c_driver mc96_i2c_driver = {
	.driver = {
		.name = "mc96",
		.of_match_table = mc96_of_match,
		.pm = &mc96_pm_ops,
	},
	.probe = mc96_probe,
	.remove = mc96_remove,
	.id_table = mc96_id,
};
module_i2c_driver(mc96_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ABOV MC96 IR remote controller driver");
