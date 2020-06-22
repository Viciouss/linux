// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2011 Samsung Electronics Co. Ltd. All Rights Reserved.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/pwm.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#define HAPTIC_CONTROL_REG0		0x30
#define HAPTIC_CONTROL_REG1		0x31
#define HAPTIC_CONTROL_REG2		0x32
#define HAPTIC_PLL_REG				0x33
#define HAPTIC_CONTROL_REG4		0x34
#define HAPTIC_PWM_DUTY_REG		0x35
#define HAPTIC_PWM_PERIOD_REG	0x36
#define HAPTIC_AMPLITUDE_REG		0x37

/* HAPTIC_CONTROL_REG0 */
#define CTL0_DIVIDER128				0
#define CTL0_DIVIDER256				1
#define CTL0_DIVIDER512				2
#define CTL0_DIVIDER1024			3
#define CTL0_13MHZ					1 << 2
#define CTL0_PWM_INPUT			1 << 3
#define CTL0_PWM_GEN				2 << 3
#define CTL0_WAVE_GEN				3 << 3
#define CTL0_HIGH_DRIVE			1 << 5
#define CTL0_OVER_DR_EN			1 << 6
#define CTL0_NORMAL_OP			1 << 7

/* HAPTIC_CONTROL_REG1 */
#define CTL1_HAPTICOFF_16U			0
#define CTL1_HAPTICOFF_32U			1
#define CTL1_HAPTICOFF_64U			2
#define CTL1_HAPTICOFF_100U		3
#define CTL1_HAPTICON_1U			1 << 2
#define CTL1_SMART_EN				1 << 3
#define CTL1_PLL_EN					1 << 4
#define CTL1_ERM_TYPE				1 << 5
#define CTL1_DEFAULT				1 << 6
#define CTL1_EXT_CLOCK				1 << 7

/* HAPTIC_CONTROL_REG2 */
#define CTL2_EFFECT_EN				1
#define CTL2_START_EFF_EN			1 << 2
#define CTL2_SOFT_RESET_EN			1 << 7

struct isa1200_vibrator_drvdata {
	struct i2c_client *client;
	struct pwm_device *pwm_dev;
	struct input_dev *input_device;

	struct gpio_desc *enable_gpio;

	struct workqueue_struct *wq;
	struct delayed_work work;

	struct hrtimer timer;
	spinlock_t lock;
	int timeout;
	int max_timeout;

	bool running;
	u8 ctrl0;
	u8 ctrl1;
	u8 ctrl2;
	u8 ctrl4;
	u8 pll;
	u8 duty;
	u8 period;
	int pwm_period;

	// 1000=100%, 500=50%, 0=0% Default is 999
	int current_pwm_duty;

};

static int isa1200_vibrator_i2c_write(struct i2c_client *client, u8 addr, u8 val)
{
	int error = 0;
	error = i2c_smbus_write_byte_data(client, addr, val);
	if (error)
		dev_err(&client->dev, "[VIB] Failed to write addr=[0x%x], val=[0x%x]\n", addr, val);

	return error;
}

static void isa1200_vibrator_hw_init(struct isa1200_vibrator_drvdata *vib)
{
	msleep(20);
	isa1200_vibrator_i2c_write(vib->client,
		HAPTIC_CONTROL_REG0, vib->ctrl0);
	isa1200_vibrator_i2c_write(vib->client,
		HAPTIC_CONTROL_REG1, vib->ctrl1);
	isa1200_vibrator_i2c_write(vib->client,
		HAPTIC_CONTROL_REG2, vib->ctrl2);
	isa1200_vibrator_i2c_write(vib->client,
		HAPTIC_PLL_REG, vib->pll);
	isa1200_vibrator_i2c_write(vib->client,
		HAPTIC_CONTROL_REG4, vib->ctrl4);
	isa1200_vibrator_i2c_write(vib->client,
		HAPTIC_PWM_DUTY_REG, vib->period/2);
	isa1200_vibrator_i2c_write(vib->client,
		HAPTIC_PWM_PERIOD_REG, vib->period);

	dev_dbg(&vib->client->dev, "[VIB] ctrl0 = 0x%x\n", vib->ctrl0);
	dev_dbg(&vib->client->dev, "[VIB] ctrl1 = 0x%x\n", vib->ctrl1);
	dev_dbg(&vib->client->dev, "[VIB] ctrl2 = 0x%x\n", vib->ctrl2);
	dev_dbg(&vib->client->dev, "[VIB] pll = 0x%x\n", vib->pll);
	dev_dbg(&vib->client->dev, "[VIB] ctrl4 = 0x%x\n", vib->ctrl4);
	dev_dbg(&vib->client->dev, "[VIB] duty = 0x%x\n", vib->period/2);
	dev_dbg(&vib->client->dev, "[VIB] period = 0x%x\n", vib->period);

}

static void isa1200_vibrator_on(struct isa1200_vibrator_drvdata *vib)
{
	int duty = vib->duty;

	dev_dbg(&vib->client->dev, "entering %s\n", __func__);

	isa1200_vibrator_i2c_write(vib->client,
		HAPTIC_CONTROL_REG0, vib->ctrl0 | CTL0_NORMAL_OP);
	isa1200_vibrator_i2c_write(vib->client,
		HAPTIC_PWM_DUTY_REG, duty);

	dev_dbg(&vib->client->dev, "[VIB] ctrl0 = 0x%x\n", vib->ctrl0 | CTL0_NORMAL_OP);
	dev_dbg(&vib->client->dev, "[VIB] duty = 0x%x\n", duty);

}

static void isa1200_vibrator_off(struct isa1200_vibrator_drvdata *vib)
{
	dev_dbg(&vib->client->dev, "entering %s\n", __func__);
	isa1200_vibrator_i2c_write(vib->client,
		HAPTIC_PWM_DUTY_REG, vib->period/2);
	isa1200_vibrator_i2c_write(vib->client,
		HAPTIC_CONTROL_REG0, vib->ctrl0);
}

static void isa1200_set_duty(struct isa1200_vibrator_drvdata *data, int duty)
{
	int duty_ns = 0;
	int period = data->pwm_period;
	duty_ns = (period * duty) / 1000;
	pwm_config(data->pwm_dev, duty_ns, period);
}

static void isa1200_pwm_enable(struct isa1200_vibrator_drvdata *data, bool en)
{
	if (en) {
		pwm_enable(data->pwm_dev);
	} else {
		pwm_disable(data->pwm_dev);
	}
}

static void isa1200_vibrator_work(struct work_struct *work)
{
	struct isa1200_vibrator_drvdata *vib =
		container_of(to_delayed_work(work), struct isa1200_vibrator_drvdata, work);

	dev_dbg(&vib->client->dev, "entering %s\n", __func__);

	if (vib->timeout == 0) {
		if (!vib->running)
			return;

		vib->running = false;
		isa1200_vibrator_off(vib);
		isa1200_set_duty(vib, 500);
		isa1200_pwm_enable(vib, true);
	} else {
		if (vib->running)
			return;

		vib->running = true;
		isa1200_set_duty(vib, vib->current_pwm_duty);
		isa1200_pwm_enable(vib, true);
		mdelay(1);
		isa1200_vibrator_on(vib);
	}
}

static enum hrtimer_restart isa1200_vibrator_timer_func(struct hrtimer *_timer)
{
	struct isa1200_vibrator_drvdata *vib =
		container_of(_timer, struct isa1200_vibrator_drvdata, timer);

	vib->timeout = 0;

	queue_delayed_work(vib->wq, &vib->work, 0);
	return HRTIMER_NORESTART;
}

static ssize_t enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isa1200_vibrator_drvdata *vib = i2c_get_clientdata(client);

	unsigned long	flags;
	int value;

	sscanf(buf, "%d", &value);
	dev_dbg(dev, "[VIB] time = %d ms\n", value);

	cancel_delayed_work(&vib->work);
	hrtimer_cancel(&vib->timer);
	vib->timeout = value;
	queue_delayed_work(vib->wq, &vib->work, 0);
	spin_lock_irqsave(&vib->lock, flags);
	if (value > 0) {
		if (value > vib->max_timeout)
			value = vib->max_timeout;

		hrtimer_start(&vib->timer,
			ns_to_ktime((u64)value * NSEC_PER_MSEC),
			HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&vib->lock, flags);

	return size;
}

static ssize_t pwm_value_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
		int count;
		struct isa1200_vibrator_drvdata *data = dev_get_drvdata(dev);

		unsigned long pwm_val = ((data->current_pwm_duty - 500) * 100) / 500;

		count = sprintf(buf, "%lu\n", pwm_val);
		dev_dbg(dev, "[VIB] pwm_value: %lu\n", pwm_val);

		return count;
}

ssize_t pwm_value_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct isa1200_vibrator_drvdata *data = dev_get_drvdata(dev);
	unsigned long pwm_val = 0;

	if (kstrtoul(buf, 0, &pwm_val))
		dev_err(dev, "[VIB] error on storing pwm_value\n");

	dev_dbg(dev, "[VIB] pwm_value=%lu\n", pwm_val);

	data->current_pwm_duty = (pwm_val * 500) / 100 + 500;

	/* make sure new pwm duty is in range */
	if(data->current_pwm_duty > 1000) {
		data->current_pwm_duty = 1000;
	} else if (data->current_pwm_duty < 500) {
		data->current_pwm_duty = 500;
	}

	dev_dbg(dev, "[VIB] current_pwm_duty = %d\n", data->current_pwm_duty);

	return size;
}

static struct device_attribute isa1200_device_attrs[] = {
	__ATTR(enable, S_IWUSR, NULL, enable_store),
	__ATTR(pwm_value, S_IRUGO | S_IWUSR, pwm_value_show, pwm_value_store),
};

static int isa1200_parse_dt(struct i2c_client *client,
		struct isa1200_vibrator_drvdata *drvdata)
{
	struct device_node *np = client->dev.of_node;
	int val, error;

	drvdata->enable_gpio = devm_gpiod_get_optional(&client->dev,
		"enable", GPIOD_OUT_HIGH);
	if (IS_ERR(drvdata->enable_gpio)) {
		error = PTR_ERR(drvdata->enable_gpio);
		dev_err(&client->dev, "Failed to get enable gpio: %d\n", error);
		return error;
	}

	drvdata->pwm_dev = devm_pwm_get(&client->dev, NULL);
	if (IS_ERR(drvdata->pwm_dev)) {
		error = PTR_ERR(drvdata->pwm_dev);
		dev_err(&client->dev, "Could not get pwm link\n");
	}

	if (!of_property_read_u32(np, "max-timeout", &val))
		drvdata->max_timeout = val;
	if (!of_property_read_u32(np, "ctrl0", &val))
		drvdata->ctrl0 = val;
	if (!of_property_read_u32(np, "ctrl1", &val))
		drvdata->ctrl1 = val;
	if (!of_property_read_u32(np, "ctrl2", &val))
		drvdata->ctrl2 = val;
	if (!of_property_read_u32(np, "ctrl4", &val))
		drvdata->ctrl4 = val;
	if (!of_property_read_u32(np, "pll", &val))
		drvdata->pll = val;
	if (!of_property_read_u32(np, "duty", &val))
		drvdata->duty = val;
	if (!of_property_read_u32(np, "period", &val))
		drvdata->period = val;
	if (!of_property_read_u32(np, "pwm-period", &val))
		drvdata->pwm_period = val;

	drvdata->current_pwm_duty = 999;
	
	return 0;
}

static int isa1200_play_effect(struct input_dev *dev, void *data,
				struct ff_effect *effect)
{
	struct isa1200_vibrator_drvdata *ddata = input_get_drvdata(dev);

	/* support basic vibration */
	ddata->running = effect->u.rumble.strong_magnitude >> 8;
	if (!ddata->running)
		ddata->running = effect->u.rumble.weak_magnitude >> 9;

	schedule_delayed_work(&ddata->work, 0);
	return 0;
}

static int isa1200_vibrator_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct isa1200_vibrator_drvdata *ddata;
	int i, ret = 0;

	dev_dbg(&client->dev, "[VIB] %s\n", __func__);

	ddata = devm_kzalloc(&client->dev, sizeof(struct isa1200_vibrator_drvdata), GFP_KERNEL);
	if (NULL == ddata) {
		dev_err(&client->dev, "[VIB] Failed to alloc memory\n");
		ret = -ENOMEM;
		return ret;
	}

	ret = isa1200_parse_dt(client, ddata);
	if (ret) {
		dev_err(&client->dev, "error parsing device tree\n");
		return ret;
	}

	ddata->client = client;
	i2c_set_clientdata(client, ddata);
	dev_set_drvdata(&client->dev, ddata);
	
	gpiod_direction_output(ddata->enable_gpio, 1);
	isa1200_vibrator_hw_init(ddata);

	hrtimer_init(&ddata->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ddata->timer.function = isa1200_vibrator_timer_func;

	ddata->input_device = devm_input_allocate_device(&client->dev);
	if (!ddata->input_device) {
		dev_err(&client->dev, "input device alloc failed\n");
		ret = -ENOMEM;
		goto err_disable_gpio;
	}

	input_set_drvdata(ddata->input_device, ddata);
	ddata->input_device->name = "isa1200-haptic-device";
	ddata->input_device->dev.parent = &client->dev;
	ddata->input_device->id.bustype = BUS_I2C;

	input_set_capability(ddata->input_device, EV_FF, FF_RUMBLE);

	ret = input_ff_create_memless(ddata->input_device, NULL, isa1200_play_effect);
	if (ret) {
		dev_err(&client->dev, "error calling input_ff_create_memless\n");
		goto err_disable_gpio;
	}

	ret = input_register_device(ddata->input_device);
	if (ret) {
		dev_err(&client->dev, "could not register input device\n");
		goto err_disable_gpio;
	}

	ddata->wq = create_singlethread_workqueue("isa1200");
	INIT_DELAYED_WORK(&ddata->work, isa1200_vibrator_work);

	for (i = 0; i < ARRAY_SIZE(isa1200_device_attrs); i++) {
		ret = device_create_file(&client->dev, &isa1200_device_attrs[i]);
		if (ret < 0) {
			dev_err(&client->dev, "failed to create sysfs attributes\n");
			goto err_disable_gpio;
		}
	}

	return 0;

err_disable_gpio:
	gpiod_set_value(ddata->enable_gpio, 0);
	return ret;

}

static int isa1200_vibrator_i2c_remove(struct i2c_client *client)
{
	struct isa1200_vibrator_drvdata *data  = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < ARRAY_SIZE(isa1200_device_attrs); i++) {
		 device_remove_file(&data->client->dev, &isa1200_device_attrs[i]);
	}

	flush_workqueue(data->wq);
	destroy_workqueue(data->wq);
	data->wq = NULL;

	kfree(data);

	return 0;
}

static int isa1200_vibrator_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isa1200_vibrator_drvdata *vib  = i2c_get_clientdata(client);
	
	gpiod_set_value(vib->enable_gpio, 0);
	
	return 0;
}

static int isa1200_vibrator_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isa1200_vibrator_drvdata *data  = i2c_get_clientdata(client);
	
	gpiod_set_value(data->enable_gpio, 1);
	isa1200_vibrator_hw_init(data);

	return 0;
}

static SIMPLE_DEV_PM_OPS(isa1200_pm,
	isa1200_vibrator_suspend, isa1200_vibrator_resume);

static const struct i2c_device_id isa1200_vibrator_device_id[] = {
	{ "isa1200_vibrator", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, isa1200_vibrator_device_id);

#ifdef CONFIG_OF
static const struct of_device_id isa1200_dt_match[] = {
	{ .compatible = "samsung_p3,isa1200_vibrator" },
	{ },
};
MODULE_DEVICE_TABLE(of, isa1200_dt_match);
#endif

static struct i2c_driver isa1200_vibrator_i2c_driver = {
	.driver = {
		.name = "isa1200-vibrator",
		.pm = &isa1200_pm,
		.of_match_table = of_match_ptr(isa1200_dt_match),
		.owner = THIS_MODULE,
	},
	.probe     = isa1200_vibrator_i2c_probe,
	.remove    = isa1200_vibrator_i2c_remove,
	.id_table  = isa1200_vibrator_device_id,
};

module_i2c_driver(isa1200_vibrator_i2c_driver);

MODULE_LICENSE("GPL");