// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Wacom G5SP Digitizer Controller (I2C bus)
 *
 * Ported from the Samsung GPL kernel driver drivers/input/touchscreen/wacom
 * (wacom_i2c.c, wacom_i2c_func.c), as shipped on the GT-N8000, reduced to
 * the CONFIG_MACH_P4NOTE code paths. Board setup from p4-input.c moved to
 * the devicetree: PEN_LDO_EN is a fixed regulator ("vdd"), PEN_PDCT and the
 * S-Pen silo switch are GPIOs.
 *
 * Not ported: firmware flashing (the digitizer keeps the firmware flashed
 * by the stock ROM), the factory checksum/coil tests, the Samsung DVFS and
 * bus frequency locks and the sec_class "sec_epen" sysfs device. The
 * Samsung specific KEY_PEN_PDCT is not reported, as 0x230 is KEY_ALS_TOGGLE
 * in mainline, and the silo switch is reported as SW_PEN_INSERTED instead
 * of Samsung's inverted SW_PEN_INSERT.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <asm/unaligned.h>

#define COM_COORD_NUM		7
#define COM_QUERY_NUM		9

#define COM_SAMPLERATE_STOP	0x30
#define COM_SAMPLERATE_40	0x33
#define COM_SAMPLERATE_80	0x32
#define COM_SAMPLERATE_133	0x31
#define COM_QUERY		0x2A

/* CONFIG_MACH_P4NOTE values from wacom_i2c.h */
#define WACOM_POSX_MAX		21866
#define WACOM_POSY_MAX		13730
#define WACOM_POSX_OFFSET	170
#define WACOM_POSY_OFFSET	170
#define WACOM_MAX_PRESSURE	0x3FF

#define EPEN_RESUME_DELAY	180

struct wacom_g5sp {
	struct i2c_client *client;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct regulator *vdd;
	struct gpio_desc *pdct_gpio;
	struct gpio_desc *insert_gpio;
	int irq;
	int irq_pdct;
	int irq_insert;
	bool irq_pdct_requested;

	/* power state, protected by lock */
	struct mutex lock;
	bool powered;
	bool irqs_enabled;
	bool suspended;
	bool saving_mode;
	struct delayed_work resume_work;

	/* pen state and input reports, protected by report_lock */
	struct mutex report_lock;
	int tool;
	bool pen_prox;
	bool pen_pressed;
	bool side_pressed;
	u16 last_x;
	u16 last_y;
	struct delayed_work pendct_work;

	struct delayed_work insert_work;
	bool pen_inserted;

	bool query_ok;
	u16 fw_version;
};

/* Caller holds report_lock */
static void wacom_g5sp_forced_release(struct wacom_g5sp *wac)
{
	struct input_dev *input = wac->input;

	input_report_abs(input, ABS_X, wac->last_x);
	input_report_abs(input, ABS_Y, wac->last_y);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_report_key(input, BTN_STYLUS, 0);
	input_report_key(input, BTN_TOUCH, 0);
	input_report_key(input, BTN_TOOL_RUBBER, 0);
	input_report_key(input, BTN_TOOL_PEN, 0);
	input_sync(input);

	wac->last_x = 0;
	wac->last_y = 0;
	wac->pen_prox = false;
	wac->pen_pressed = false;
	wac->side_pressed = false;
}

/* PEN_PDCT is pulled low while the pen is near the digitizer */
static bool wacom_g5sp_pen_near(struct wacom_g5sp *wac)
{
	return gpiod_get_value_cansleep(wac->pdct_gpio) > 0;
}

static void wacom_g5sp_pendct_work(struct work_struct *work)
{
	struct wacom_g5sp *wac = container_of(work, struct wacom_g5sp,
					      pendct_work.work);

	mutex_lock(&wac->report_lock);
	if (!wacom_g5sp_pen_near(wac))
		wacom_g5sp_forced_release(wac);
	mutex_unlock(&wac->report_lock);
}

static irqreturn_t wacom_g5sp_irq(int irq, void *dev_id)
{
	struct wacom_g5sp *wac = dev_id;
	struct input_dev *input = wac->input;
	u8 data[COM_COORD_NUM];
	bool prox, stylus;
	u16 x, y, pressure;
	int ret;

	ret = i2c_master_recv(wac->client, data, sizeof(data));
	if (ret != sizeof(data)) {
		dev_err_ratelimited(&wac->client->dev,
				    "failed to read coordinates: %d\n", ret);
		return IRQ_HANDLED;
	}

	mutex_lock(&wac->report_lock);
	cancel_delayed_work(&wac->pendct_work);

	if (!(data[0] & 0x80)) {
		/* pen left the sensing range */
		if (wacom_g5sp_pen_near(wac)) {
			wac->tool = (data[0] & 0x40) ?
				BTN_TOOL_RUBBER : BTN_TOOL_PEN;
			input_report_abs(input, ABS_PRESSURE, 0);
			input_report_key(input, BTN_STYLUS, 0);
			input_report_key(input, BTN_TOUCH, 0);
			input_report_key(input, wac->tool, 0);
			input_sync(input);
		}
		schedule_delayed_work(&wac->pendct_work, HZ / 10);
		goto out;
	}

	if (!wac->pen_prox) {
		wac->pen_prox = true;
		wac->tool = (data[0] & 0x40) ? BTN_TOOL_RUBBER : BTN_TOOL_PEN;
	}

	prox = data[0] & 0x10;
	stylus = data[0] & 0x20;
	x = get_unaligned_be16(&data[1]);
	y = get_unaligned_be16(&data[3]);
	pressure = get_unaligned_be16(&data[5]);

	/* outside of the active area */
	if (x <= WACOM_POSX_OFFSET || y <= WACOM_POSY_OFFSET) {
		dev_dbg(&wac->client->dev, "raw data x=%u, y=%u\n", x, y);
		goto out;
	}

	touchscreen_report_pos(input, &wac->prop, x, y, false);
	input_report_abs(input, ABS_PRESSURE, pressure);
	input_report_key(input, BTN_STYLUS, stylus);
	input_report_key(input, BTN_TOUCH, prox);
	input_report_key(input, wac->tool, 1);
	input_sync(input);

	wac->last_x = x;
	wac->last_y = y;

	if (prox != wac->pen_pressed)
		dev_dbg(&wac->client->dev, "%s (%u,%u,%u) tool %d\n",
			prox ? "pressed" : "released", x, y, pressure,
			wac->tool);

	wac->pen_pressed = prox;
	wac->side_pressed = stylus;
out:
	mutex_unlock(&wac->report_lock);
	return IRQ_HANDLED;
}

static irqreturn_t wacom_g5sp_irq_pdct(int irq, void *dev_id)
{
	struct wacom_g5sp *wac = dev_id;
	bool near;

	if (!wac->query_ok)
		return IRQ_HANDLED;

	near = wacom_g5sp_pen_near(wac);

	mutex_lock(&wac->report_lock);
	dev_dbg(&wac->client->dev, "pdct %d (%d)\n", near, wac->pen_prox);

	/* release a stale hover position; the pen is still working if prox */
	if (!wac->pen_prox && (!near || wac->last_x || wac->last_y))
		wacom_g5sp_forced_release(wac);
	mutex_unlock(&wac->report_lock);

	return IRQ_HANDLED;
}

/* Caller holds lock */
static void wacom_g5sp_enable_irqs(struct wacom_g5sp *wac)
{
	int ret;

	if (wac->irqs_enabled)
		return;

	/*
	 * PEN_PDCT is driven as an output during power up, which gpiolib
	 * refuses while the line is locked as an interrupt, so the PDCT
	 * interrupt is only requested while the digitizer is powered.
	 */
	if (wac->irq_pdct > 0) {
		ret = request_threaded_irq(wac->irq_pdct, NULL,
					   wacom_g5sp_irq_pdct,
					   IRQF_TRIGGER_RISING |
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   "wacom_g5sp_pdct", wac);
		if (ret)
			dev_err(&wac->client->dev,
				"failed to request pdct irq: %d\n", ret);
		else
			wac->irq_pdct_requested = true;
	}

	enable_irq(wac->irq);
	wac->irqs_enabled = true;
}

/* Caller holds lock */
static void wacom_g5sp_disable_irqs(struct wacom_g5sp *wac)
{
	if (!wac->irqs_enabled)
		return;

	disable_irq(wac->irq);
	if (wac->irq_pdct_requested) {
		free_irq(wac->irq_pdct, wac);
		wac->irq_pdct_requested = false;
	}
	wac->irqs_enabled = false;

	cancel_delayed_work_sync(&wac->pendct_work);
}

static int wacom_g5sp_hw_on(struct wacom_g5sp *wac)
{
	int ret;

	/* p4-input.c wacom_late_resume_hw() */
	gpiod_direction_output_raw(wac->pdct_gpio, 1);

	ret = regulator_enable(wac->vdd);
	if (ret) {
		dev_err(&wac->client->dev,
			"failed to enable vdd: %d\n", ret);
		gpiod_direction_input(wac->pdct_gpio);
		return ret;
	}

	msleep(20);
	gpiod_direction_input(wac->pdct_gpio);

	return 0;
}

static void wacom_g5sp_hw_off(struct wacom_g5sp *wac)
{
	regulator_disable(wac->vdd);
}

static void wacom_g5sp_power_on(struct wacom_g5sp *wac)
{
	mutex_lock(&wac->lock);

	if (wac->powered || wac->suspended)
		goto out;

	if (wac->saving_mode && wac->pen_inserted)
		goto out;

	if (wacom_g5sp_hw_on(wac))
		goto out;

	wac->powered = true;
	schedule_delayed_work(&wac->resume_work,
			      msecs_to_jiffies(EPEN_RESUME_DELAY));
	dev_dbg(&wac->client->dev, "power on\n");
out:
	mutex_unlock(&wac->lock);
}

static void wacom_g5sp_power_off(struct wacom_g5sp *wac)
{
	cancel_delayed_work_sync(&wac->resume_work);

	mutex_lock(&wac->lock);

	if (!wac->powered)
		goto out;

	wacom_g5sp_disable_irqs(wac);

	/* release pen, if it is pressed */
	mutex_lock(&wac->report_lock);
	wacom_g5sp_forced_release(wac);
	mutex_unlock(&wac->report_lock);

	wacom_g5sp_hw_off(wac);
	wac->powered = false;
	dev_dbg(&wac->client->dev, "power off\n");
out:
	mutex_unlock(&wac->lock);
}

static void wacom_g5sp_resume_work(struct work_struct *work)
{
	struct wacom_g5sp *wac = container_of(work, struct wacom_g5sp,
					      resume_work.work);

	mutex_lock(&wac->lock);
	if (wac->powered)
		wacom_g5sp_enable_irqs(wac);
	mutex_unlock(&wac->lock);
}

static void wacom_g5sp_insert_work(struct work_struct *work)
{
	struct wacom_g5sp *wac = container_of(work, struct wacom_g5sp,
					      insert_work.work);
	int inserted;

	inserted = gpiod_get_value_cansleep(wac->insert_gpio);
	if (inserted < 0)
		return;

	dev_dbg(&wac->client->dev, "pen %s\n",
		inserted ? "inserted" : "removed");

	mutex_lock(&wac->report_lock);
	wac->pen_inserted = inserted;
	input_report_switch(wac->input, SW_PEN_INSERTED, inserted);
	input_sync(wac->input);
	mutex_unlock(&wac->report_lock);

	if (inserted && wac->saving_mode)
		wacom_g5sp_power_off(wac);
	else
		wacom_g5sp_power_on(wac);
}

static irqreturn_t wacom_g5sp_irq_insert(int irq, void *dev_id)
{
	struct wacom_g5sp *wac = dev_id;

	mod_delayed_work(system_wq, &wac->insert_work, HZ / 20);

	return IRQ_HANDLED;
}

static int wacom_g5sp_query(struct wacom_g5sp *wac)
{
	struct device *dev = &wac->client->dev;
	u8 cmd = COM_QUERY;
	u8 data[COM_QUERY_NUM];
	int ret = -EIO;
	int i;

	for (i = 0; i < 10; i++) {
		ret = i2c_master_send(wac->client, &cmd, 1);
		if (ret < 0) {
			dev_err(dev, "query send failed: %d\n", ret);
			continue;
		}

		msleep(100);

		ret = i2c_master_recv(wac->client, data, sizeof(data));
		if (ret < 0) {
			dev_err(dev, "query recv failed: %d\n", ret);
			continue;
		}

		if (ret == sizeof(data) && data[0] == 0x0f) {
			wac->fw_version = get_unaligned_be16(&data[7]);
			dev_info(dev, "fw version 0x%x, x %u, y %u, pressure %u\n",
				 wac->fw_version,
				 get_unaligned_be16(&data[1]),
				 get_unaligned_be16(&data[3]),
				 get_unaligned_be16(&data[5]));
			wac->query_ok = true;
			return 0;
		}

		dev_notice(dev, "unexpected query reply %*ph\n",
			   ret, data);
	}

	wac->query_ok = false;
	return ret < 0 ? ret : -EIO;
}

static ssize_t fw_version_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct wacom_g5sp *wac = dev_get_drvdata(dev);

	return sprintf(buf, "%04X\n", wac->fw_version);
}
static DEVICE_ATTR_RO(fw_version);

static ssize_t sampling_rate_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct wacom_g5sp *wac = dev_get_drvdata(dev);
	unsigned int value;
	u8 mode;
	int ret;

	ret = kstrtouint(buf, 0, &value);
	if (ret)
		return ret;

	switch (value) {
	case 0:
		mode = COM_SAMPLERATE_STOP;
		break;
	case 40:
		mode = COM_SAMPLERATE_40;
		break;
	case 80:
		mode = COM_SAMPLERATE_80;
		break;
	case 133:
		mode = COM_SAMPLERATE_133;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&wac->lock);

	if (!wac->powered) {
		ret = -ENODEV;
		goto out;
	}

	if (wac->irqs_enabled)
		disable_irq(wac->irq);

	ret = i2c_master_send(wac->client, &mode, 1);
	if (ret == 1) {
		dev_dbg(dev, "sampling rate %u\n", value);
		msleep(100);
		ret = count;
	} else if (ret >= 0) {
		ret = -EIO;
	}

	if (wac->irqs_enabled)
		enable_irq(wac->irq);
out:
	mutex_unlock(&wac->lock);
	return ret;
}
static DEVICE_ATTR_WO(sampling_rate);

static ssize_t saving_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct wacom_g5sp *wac = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", wac->saving_mode);
}

/* Power the digitizer off while the pen is in its silo */
static ssize_t saving_mode_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct wacom_g5sp *wac = dev_get_drvdata(dev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	wac->saving_mode = val;

	if (wac->saving_mode && wac->pen_inserted)
		wacom_g5sp_power_off(wac);
	else
		wacom_g5sp_power_on(wac);

	return count;
}
static DEVICE_ATTR_RW(saving_mode);

static struct attribute *wacom_g5sp_attrs[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_sampling_rate.attr,
	&dev_attr_saving_mode.attr,
	NULL
};
ATTRIBUTE_GROUPS(wacom_g5sp);

static int wacom_g5sp_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct wacom_g5sp *wac;
	struct input_dev *input;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	wac = devm_kzalloc(dev, sizeof(*wac), GFP_KERNEL);
	if (!wac)
		return -ENOMEM;

	wac->client = client;
	wac->irq = client->irq;
	mutex_init(&wac->lock);
	mutex_init(&wac->report_lock);
	INIT_DELAYED_WORK(&wac->resume_work, wacom_g5sp_resume_work);
	INIT_DELAYED_WORK(&wac->pendct_work, wacom_g5sp_pendct_work);
	INIT_DELAYED_WORK(&wac->insert_work, wacom_g5sp_insert_work);
	i2c_set_clientdata(client, wac);

	wac->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(wac->vdd))
		return dev_err_probe(dev, PTR_ERR(wac->vdd),
				     "failed to get vdd regulator\n");

	wac->pdct_gpio = devm_gpiod_get_optional(dev, "pdct", GPIOD_IN);
	if (IS_ERR(wac->pdct_gpio))
		return dev_err_probe(dev, PTR_ERR(wac->pdct_gpio),
				     "failed to get pdct gpio\n");

	wac->insert_gpio = devm_gpiod_get_optional(dev, "pen-insert",
						   GPIOD_IN);
	if (IS_ERR(wac->insert_gpio))
		return dev_err_probe(dev, PTR_ERR(wac->insert_gpio),
				     "failed to get pen-insert gpio\n");

	if (wac->pdct_gpio) {
		wac->irq_pdct = gpiod_to_irq(wac->pdct_gpio);
		if (wac->irq_pdct < 0)
			return dev_err_probe(dev, wac->irq_pdct,
					     "failed to get pdct irq\n");
	}

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	wac->input = input;
	input->name = "sec_e-pen";
	input->id.bustype = BUS_I2C;
	input_set_drvdata(input, wac);

	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	input_set_capability(input, EV_KEY, BTN_TOUCH);
	input_set_capability(input, EV_KEY, BTN_TOOL_PEN);
	input_set_capability(input, EV_KEY, BTN_TOOL_RUBBER);
	input_set_capability(input, EV_KEY, BTN_STYLUS);
	if (wac->insert_gpio)
		input_set_capability(input, EV_SW, SW_PEN_INSERTED);

	input_set_abs_params(input, ABS_X, WACOM_POSX_OFFSET,
			     WACOM_POSX_MAX, 4, 0);
	input_set_abs_params(input, ABS_Y, WACOM_POSY_OFFSET,
			     WACOM_POSY_MAX, 4, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, WACOM_MAX_PRESSURE, 0, 0);
	touchscreen_parse_properties(input, false, &wac->prop);

	/* Power on */
	ret = wacom_g5sp_hw_on(wac);
	if (ret)
		return ret;
	msleep(200);

	ret = wacom_g5sp_query(wac);
	if (ret) {
		dev_err(dev, "digitizer does not respond: %d\n", ret);
		goto err_power_off;
	}

	wac->powered = true;

	irq_set_status_flags(wac->irq, IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, wac->irq, NULL, wacom_g5sp_irq,
					IRQF_ONESHOT, "wacom_g5sp", wac);
	if (ret) {
		dev_err(dev, "failed to request irq: %d\n", ret);
		goto err_power_off;
	}

	ret = input_register_device(input);
	if (ret) {
		dev_err(dev, "failed to register input device: %d\n", ret);
		goto err_power_off;
	}

	if (wac->insert_gpio) {
		wac->irq_insert = gpiod_to_irq(wac->insert_gpio);
		if (wac->irq_insert < 0) {
			ret = wac->irq_insert;
			dev_err(dev, "failed to get pen-insert irq: %d\n", ret);
			goto err_power_off;
		}

		ret = devm_request_irq(dev, wac->irq_insert,
				       wacom_g5sp_irq_insert,
				       IRQF_TRIGGER_RISING |
				       IRQF_TRIGGER_FALLING,
				       "wacom_g5sp_insert", wac);
		if (ret) {
			dev_err(dev, "failed to request pen-insert irq: %d\n",
				ret);
			goto err_power_off;
		}

		device_init_wakeup(dev,
				   device_property_read_bool(dev,
							     "wakeup-source"));

		/* update the current status */
		schedule_delayed_work(&wac->insert_work, HZ / 2);
	}

	mutex_lock(&wac->lock);
	wacom_g5sp_enable_irqs(wac);
	mutex_unlock(&wac->lock);

	return 0;

err_power_off:
	wac->powered = false;
	wacom_g5sp_hw_off(wac);
	return ret;
}

static void wacom_g5sp_remove(struct i2c_client *client)
{
	struct wacom_g5sp *wac = i2c_get_clientdata(client);

	if (wac->irq_insert > 0)
		disable_irq(wac->irq_insert);
	cancel_delayed_work_sync(&wac->insert_work);

	wacom_g5sp_power_off(wac);
}

static int __maybe_unused wacom_g5sp_suspend(struct device *dev)
{
	struct wacom_g5sp *wac = dev_get_drvdata(dev);

	mutex_lock(&wac->lock);
	wac->suspended = true;
	mutex_unlock(&wac->lock);

	wacom_g5sp_power_off(wac);

	if (wac->irq_insert > 0 && device_may_wakeup(dev))
		enable_irq_wake(wac->irq_insert);

	return 0;
}

static int __maybe_unused wacom_g5sp_resume(struct device *dev)
{
	struct wacom_g5sp *wac = dev_get_drvdata(dev);

	if (wac->irq_insert > 0 && device_may_wakeup(dev))
		disable_irq_wake(wac->irq_insert);

	mutex_lock(&wac->lock);
	wac->suspended = false;
	mutex_unlock(&wac->lock);

	wacom_g5sp_power_on(wac);

	/* the pen may have been taken out while asleep */
	if (wac->insert_gpio)
		mod_delayed_work(system_wq, &wac->insert_work, HZ / 20);

	return 0;
}

static SIMPLE_DEV_PM_OPS(wacom_g5sp_pm, wacom_g5sp_suspend, wacom_g5sp_resume);

static const struct i2c_device_id wacom_g5sp_id[] = {
	{ "wacom_g5sp_i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wacom_g5sp_id);

static const struct of_device_id wacom_g5sp_of_match[] = {
	{ .compatible = "wacom,g5sp" },
	{ }
};
MODULE_DEVICE_TABLE(of, wacom_g5sp_of_match);

static struct i2c_driver wacom_g5sp_driver = {
	.driver = {
		.name = "wacom_g5sp_i2c",
		.of_match_table = wacom_g5sp_of_match,
		.dev_groups = wacom_g5sp_groups,
		.pm = &wacom_g5sp_pm,
	},
	.probe = wacom_g5sp_probe,
	.remove = wacom_g5sp_remove,
	.id_table = wacom_g5sp_id,
};
module_i2c_driver(wacom_g5sp_driver);

MODULE_AUTHOR("Samsung");
MODULE_DESCRIPTION("Driver for Wacom G5SP Digitizer Controller");
MODULE_LICENSE("GPL");
