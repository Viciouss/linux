// SPDX-License-Identifier: GPL-2.0+
/*
 * Immersion Corporation ISA1200 haptic feedback unit
 * This chip controls two force-feedback motors
 *
 * This chip can use either an external clock or an external PWM
 * to drive the PWM. If a clock is used, the chip will generate the
 * PWM for the motor. If a PWM input is used, the chip will act as
 * an amplifier for the PWM input.
 *
 * No datasheet exists, registers and values extracted from GPL
 * code drops from various Samsung mobile phones:
 *
 * - Galaxy S Advance GT-I9070
 * - Galaxy Beam GT-I8350
 * - Galaxy Note 10.1 (all variants)
 *
 * Copyright (C) 2022 Linus Walleij <linus.walleij@linaro.org>
 */
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/property.h>
#include <linux/pwm.h>

/* System control (LDO regulator) */
#define ISA1200_SCTRL			0x00

#define ISA1200_LDO_VOLTAGE_23V		0x08
#define ISA1200_LDO_VOLTAGE_24V		0x09
#define ISA1200_LDO_VOLTAGE_25V		0x0A
#define ISA1200_LDO_VOLTAGE_26V		0x0B
#define ISA1200_LDO_VOLTAGE_27V		0x0C
#define ISA1200_LDO_VOLTAGE_28V		0x0D
#define ISA1200_LDO_VOLTAGE_29V		0x0E
#define ISA1200_LDO_VOLTAGE_30V		0x0F
#define ISA1200_LDO_VOLTAGE_31V		0x00
#define ISA1200_LDO_VOLTAGE_32V		0x01
#define ISA1200_LDO_VOLTAGE_33V		0x02
#define ISA1200_LDO_VOLTAGE_34V		0x03
#define ISA1200_LDO_VOLTAGE_35V		0x04
#define ISA1200_LDO_VOLTAGE_36V		0x05
#define ISA1200_LDO_VOLTAGE_37V		0x06
#define ISA1200_LDO_VOLTAGE_38V		0x07

/*
 * The vendor source code for the GT-I9070 states that the output
 * frequency is calculated like this:
 *
 *                 base clock frequency
 * fout = -----------------------------------------
 *        (128 - PWM_FREQ) * 2 * PLLDIV * PWM_PERIOD
 *
 * The base clock frequency is the clock frequency provided on the
 * clock input to the chip, divided by the value in HCTRL0
 *
 * PWM_FREQ is configured in register HCTRL4, it is common to set this
 * to 0 to get only two variables to calculate.
 *
 * PLLDIV is configured in register HCTRL3 (bits 7..4, so 0..15)
 * PWM_PERIOD is configured in register HCTRL6
 * Further the duty cycle can be configured in HCTRL5
 */

/*
 * HCTRL0 configures clock or PWM input and selects the divider for
 * the clock input.
 *
 * Code comments says that bits [1..0] gives the division factor.
 * If the input clock is 44.8 kHz, writing 0x01 into this register
 * sets the div_factor to 256 and the resulting frequency will be
 * 44800/256 = 175Hz.
 *
 * If the PWM input mode is set the input on the clock/PWM pin is
 * assumed to be a PWM already rather than a regular clock pulse.
 */
#define ISA1200_HCTRL0			0x30
#define ISA1200_HCTRL0_PWM_GEN_ENABLE	BIT(7)
#define ISA1200_HCTRL0_OVER_DRIVE	BIT(6)
#define ISA1200_HCTRL0_HIGH_DRIVE	BIT(5)
#define ISA1200_HCTRL0_PWM_WAVE_MODE	(BIT(3) + BIT(4))
#define ISA1200_HCTRL0_PWM_GEN_MODE	BIT(4)
#define ISA1200_HCTRL0_PWM_INPUT_MODE	BIT(3)
#define ISA1200_HCTRL0_13MHZ		BIT(2)
#define ISA1200_HCTRL0_DIV_128		0x00
#define ISA1200_HCTRL0_DIV_256		0x01
#define ISA1200_HCTRL0_DIV_512		0x02
#define ISA1200_HCTRL0_DIV_1024		0x03

/*
 * HCTRL1 configures the motor type(s)
 * ERM = Eccentric rotating mass
 * LRA = Linear Resonant Actuator
 */
#define ISA1200_HCTRL1					0x31
#define ISA1200_HCTRL1_EXT_CLOCK	BIT(7) /* Else PWM in is assumed */
#define ISA1200_HCTRL1_DAC_INVERT	BIT(6)
#define ISA1200_HCTRL1_ERM		BIT(5) /* Else LRA is assumed */
#define ISA1200_HCTRL1_PLL_EN		BIT(4)
#define ISA1200_HCTRL1_SMART_EN		BIT(3)
#define ISA1200_HCTRL1_HAPTICON_1U	BIT(2)
#define ISA1200_HCTRL1_HAPTICOFF_16U	0x0
#define ISA1200_HCTRL1_HAPTICOFF_32U	0x1
#define ISA1200_HCTRL1_HAPTICOFF_64U	0x2
#define ISA1200_HCTRL1_HAPTICOFF_100U	0x3

/* HCTRL2 controls software reset of the chip */
#define ISA1200_HCTRL2			0x32
#define ISA1200_HCTRL2_SW_RESET		BIT(0)

/*
 * HCTRL3 controls the PLL divisor
 *
 * Bits [0,1] are always set to 1 (we don't know what they are
 * used for) and bit 4 and upward control the PLL divisor.
 */
#define ISA1200_HCTRL3			0x33
#define ISA1200_HCTRL3_DEFAULT		0x03
#define ISA1200_HCTRL3_PLLDIV_SHIFT	4

/*
 * HCTRL4 controls the PWM frequency
 *
 * Set this to 0 and ignore the duty cycle and period registers if
 * PWM input mode is used.
 */
#define ISA1200_HCTRL4			0x34

/* HCTRL5 controls the PWM high duty cycle */
#define ISA1200_HCTRL5			0x35

/* HCTRL6 controls the PWM period */
#define ISA1200_HCTRL6			0x36

/* The use for these registers is unknown but they exist */
#define ISA1200_HCTRL7			0x37	/* haptic amplitude reg */
#define ISA1200_HCTRL8			0x38
#define ISA1200_HCTRL9			0x39
#define ISA1200_HCTRLA			0x3A
#define ISA1200_HCTRLB			0x3B
#define ISA1200_HCTRLC			0x3C
#define ISA1200_HCTRLD			0x3D

struct isa1200_config {
	u8 ldo_voltage;
	u8 clkdiv;
	u8 plldiv;
	u8 freq;
	u8 duty;
	u8 period;
};

struct isa1200 {
	struct isa1200_config *conf;
	struct input_dev *input;
	struct device *dev;
	struct regmap *map;
	struct clk *clk;
	struct pwm_device *pwm;
	struct gpio_desc *hen;
	struct gpio_desc *len;
	struct work_struct play_work;
	int level;
};

struct isa1200_ldo_voltage {
	u8 voltage;
	u8 reg_value;
};

static struct isa1200_ldo_voltage isa1200_ldo_voltages[] = {
	{ 23, ISA1200_LDO_VOLTAGE_23V },
	{ 24, ISA1200_LDO_VOLTAGE_24V },
	{ 25, ISA1200_LDO_VOLTAGE_25V },
	{ 26, ISA1200_LDO_VOLTAGE_26V },
	{ 27, ISA1200_LDO_VOLTAGE_27V },
	{ 28, ISA1200_LDO_VOLTAGE_28V },
	{ 29, ISA1200_LDO_VOLTAGE_29V },
	{ 30, ISA1200_LDO_VOLTAGE_30V },
	{ 31, ISA1200_LDO_VOLTAGE_31V },
	{ 32, ISA1200_LDO_VOLTAGE_32V },
	{ 33, ISA1200_LDO_VOLTAGE_33V },
	{ 34, ISA1200_LDO_VOLTAGE_34V },
	{ 35, ISA1200_LDO_VOLTAGE_35V },
	{ 36, ISA1200_LDO_VOLTAGE_36V },
	{ 37, ISA1200_LDO_VOLTAGE_37V },
	{ 38, ISA1200_LDO_VOLTAGE_38V }
};

static const struct regmap_config isa1200_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x3d,
};

static void isa1200_drive(struct isa1200 *isa, bool enable)
{
	struct pwm_state state;
	int err;

	if (enable) {
		if (isa->clk) {
			clk_prepare_enable(isa->clk);
		} else {
			pwm_get_state(isa->pwm, &state);
			pwm_set_relative_duty_cycle(&state, isa->level, 0xffff);
			state.enabled = true;

			err = pwm_apply_might_sleep(isa->pwm, &state);
			if (err)
				dev_err(isa->dev,
					"failed to apply pwm state: %d\n", err);
		}
	} else {
		if (isa->clk) {
			clk_disable_unprepare(isa->clk);
		} else {
			pwm_get_state(isa->pwm, &state);
			// vendor tree sets duty to 50% when device not active
			pwm_set_relative_duty_cycle(&state, 50, 100);
			state.enabled = false;

			err = pwm_apply_might_sleep(isa->pwm, &state);
			if (err)
				dev_err(isa->dev,
					"failed to apply pwm state: %d\n", err);
		}
	}
};

static void isa1200_start(struct isa1200 *isa)
{
	const struct isa1200_config *cfg = isa->conf;
	u8 hctrl0;
	u8 hctrl1;
	u8 hctrl3;

	gpiod_set_value(isa->hen, 1);
	gpiod_set_value(isa->len, 1);
	isa1200_drive(isa, true);

	usleep_range(200, 300);

	regmap_write(isa->map, ISA1200_SCTRL, cfg->ldo_voltage);

	if (isa->pwm) {
		hctrl0 = ISA1200_HCTRL0_PWM_INPUT_MODE;
		hctrl1 = 0;
	} else {
		hctrl0 = ISA1200_HCTRL0_PWM_GEN_MODE;
		hctrl1 = ISA1200_HCTRL1_EXT_CLOCK;
	}
	hctrl0 |= cfg->clkdiv;
	hctrl1 |= ISA1200_HCTRL1_DAC_INVERT;

	regmap_write(isa->map, ISA1200_HCTRL0, hctrl0);
	regmap_write(isa->map, ISA1200_HCTRL1, hctrl1);

	/* Make sure to de-assert software reset */
	regmap_write(isa->map, ISA1200_HCTRL2, 0x00);

	/* PLL divisor */
	hctrl3 = ISA1200_HCTRL3_DEFAULT;
	hctrl3 |= (cfg->plldiv << ISA1200_HCTRL3_PLLDIV_SHIFT);
	regmap_write(isa->map, ISA1200_HCTRL3, hctrl3);

	/* Frequency */
	regmap_write(isa->map, ISA1200_HCTRL4, cfg->freq);
	/* Duty cycle */
	regmap_write(isa->map, ISA1200_HCTRL5, cfg->duty);
	/* Period */
	regmap_write(isa->map, ISA1200_HCTRL6, cfg->period);

	/* Turn on PWM generation in BIT(7) */
	hctrl0 |= ISA1200_HCTRL0_PWM_GEN_ENABLE;
	regmap_write(isa->map, ISA1200_HCTRL0, hctrl0);

	/*
	 * This is done in the vendor tree with the comment
	 * "Duty 0x64 == nForce 90", and no force feedback happens
	 * unless we do this.
	 */
	regmap_write(isa->map, ISA1200_HCTRL5, 0x64);
}

static void isa1200_stop(struct isa1200 *isa)
{
	regmap_write(isa->map, ISA1200_HCTRL0, 0);
	isa1200_drive(isa, false);
	gpiod_set_value(isa->len, 0);
	gpiod_set_value(isa->hen, 0);
}

static void isa1200_play_work(struct work_struct *work)
{
	struct isa1200 *isa =
		container_of(work, struct isa1200, play_work);

	if (isa->level)
		isa1200_start(isa);
	else
		isa1200_stop(isa);
}

static int isa1200_vibrator_play_effect(struct input_dev *input, void *data,
					struct ff_effect *effect)
{
	struct isa1200 *isa = input_get_drvdata(input);
	int level;

	/*
	 * TODO: we currently only support rumble.
	 * The ISA1200 can control two motors and some devices
	 * also have two motors mounted.
	 */
	level = effect->u.rumble.strong_magnitude;
	if (!level)
		level = effect->u.rumble.weak_magnitude;

	dev_dbg(&input->dev, "FF effect type %d level %d\n",
		effect->type, level);

	if (isa->level != level) {
		isa->level = level;
		schedule_work(&isa->play_work);
	}

	return 0;
}

static void isa1200_vibrator_close(struct input_dev *input)
{
	struct isa1200 *isa = input_get_drvdata(input);

	cancel_work_sync(&isa->play_work);
	if (isa->level)
		isa1200_stop(isa);
	isa->level = 0;
}

static int isa1200_parse_device_tree(struct device *dev,
				     struct isa1200_config *cfg)
{
	struct device_node *np = dev->of_node;
	u16 clock_divider;
	u8 dt_voltage;
	int i;

	if (of_property_read_u8(np, "isa1200,ldo-voltage", &dt_voltage))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(isa1200_ldo_voltages); i++) {
		if (isa1200_ldo_voltages[i].voltage == dt_voltage) {
			cfg->ldo_voltage = isa1200_ldo_voltages[i].reg_value;
			break;
		}
	}

	if (i == ARRAY_SIZE(isa1200_ldo_voltages)) {
		dev_err(dev, "LDO voltage is not in range (23-38): %d\n",
			dt_voltage);
		return -EINVAL;
	}

	of_property_read_u16(np, "isa1200,clock-divider", &clock_divider);

	switch (clock_divider) {
	case 128:
		cfg->clkdiv = ISA1200_HCTRL0_DIV_128;
		break;
	case 256:
		cfg->clkdiv = ISA1200_HCTRL0_DIV_256;
		break;
	case 512:
		cfg->clkdiv = ISA1200_HCTRL0_DIV_512;
		break;
	case 1024:
		cfg->clkdiv = ISA1200_HCTRL0_DIV_1024;
		break;
	default:
		cfg->clkdiv = ISA1200_HCTRL0_DIV_128;
		break;
	}

	of_property_read_u8(np, "isa1200,pll-divider", &cfg->plldiv);
	of_property_read_u8(np, "isa1200,pwm-frequency", &cfg->freq);
	of_property_read_u8(np, "isa1200,pwm-duty", &cfg->duty);
	of_property_read_u8(np, "isa1200,pwm-period", &cfg->period);

	return 0;
};

static int isa1200_probe(struct i2c_client *client)
{
	struct isa1200 *isa;
	struct device *dev = &client->dev;
	u32 val;
	int error;
	struct pwm_state state;

	isa = devm_kzalloc(dev, sizeof(*isa), GFP_KERNEL);
	if (!isa)
		return -ENOMEM;

	isa->input = devm_input_allocate_device(dev);
	if (!isa->input)
		return -ENOMEM;

	i2c_set_clientdata(client, isa);
	isa->dev = dev;

	isa->conf = devm_kzalloc(dev, sizeof(*isa->conf), GFP_KERNEL);
	if (!isa->conf)
		return -ENOMEM;

	error = isa1200_parse_device_tree(dev, isa->conf);
	if (error) {
		dev_err(dev, "failed to parse device tree: %d\n", error);
		return error;
	}

	isa->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(isa->clk)) {
		error = PTR_ERR(isa->clk);
		return dev_err_probe(dev, error, "failed to get clock\n");
	}

	isa->pwm = devm_pwm_get(dev, NULL);
	if (IS_ERR(isa->pwm)) {
		error = PTR_ERR(isa->pwm);
		// this is optional
		if (error == -ENODEV) {
			isa->pwm = NULL;
		} else {
			error = dev_err_probe(dev, error,
					      "unable to request PWM\n");
			return error;
		}
	} else {
		pwm_init_state(isa->pwm, &state);
		state.enabled = false;

		error = pwm_apply_might_sleep(isa->pwm, &state);
		if (error) {
			dev_err(dev, "failed to apply initial PWM state: %d\n",
				error);
			return error;
		}
	}

	isa->map = devm_regmap_init_i2c(client, &isa1200_regmap_config);
	if (IS_ERR(isa->map)) {
		error = PTR_ERR(isa->map);
		return dev_err_probe(dev, error,
				     "failed to initialize register map\n");
	}

	isa->hen = devm_gpiod_get_optional(dev, "hen", GPIOD_OUT_LOW);
	if (IS_ERR(isa->hen)) {
		error = PTR_ERR(isa->hen);
		return dev_err_probe(dev, error, "failed to get HEN GPIO\n");
	}
	isa->len = devm_gpiod_get(dev, "len", GPIOD_OUT_LOW);
	if (IS_ERR(isa->len)) {
		error = PTR_ERR(isa->len);
		return dev_err_probe(dev, error, "failed to get LEN GPIO\n");
	}

	/* Read a register so we know that regmap and I2C transport works */
	gpiod_set_value(isa->len, 1);
	usleep_range(200, 300);
	error = regmap_read(isa->map, ISA1200_SCTRL, &val);
	if (error) {
		dev_info(dev, "failed to read SCTRL: %d\n", error);
		return error;
	}
	gpiod_set_value(isa->len, 0);

	INIT_WORK(&isa->play_work, isa1200_play_work);

	isa->input->name = "isa1200-haptic";
	isa->input->id.bustype = BUS_HOST;
	isa->input->dev.parent = dev;
	isa->input->close = isa1200_vibrator_close;

	input_set_drvdata(isa->input, isa);
	/* TODO: this hardware can likely support more than rumble */
	input_set_capability(isa->input, EV_FF, FF_RUMBLE);

	error = input_ff_create_memless(isa->input, NULL,
					isa1200_vibrator_play_effect);
	if (error)
		return dev_err_probe(dev, error, "couldn't create FF dev\n");

	error = input_register_device(isa->input);
	if (error)
		return dev_err_probe(dev, error,
				     "couldn't register input dev\n");

	return error;
}

static int __maybe_unused isa1200_suspend(struct device *dev)
{
	struct isa1200 *isa = dev_get_drvdata(dev);

	if (isa->level)
		isa1200_stop(isa);

	return 0;
}

static int __maybe_unused isa1200_resume(struct device *dev)
{
	struct isa1200 *isa = dev_get_drvdata(dev);

	if (isa->level)
		isa1200_start(isa);

	return 0;
}

static SIMPLE_DEV_PM_OPS(isa1200_pm, isa1200_suspend, isa1200_resume);

static const struct of_device_id isa1200_of_match[] = {
	{
		.compatible = "immersion,isa1200",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, isa1200_of_match);

static struct i2c_driver isa1200_i2c_driver = {
	.driver = {
		.name = "isa1200",
		.of_match_table = isa1200_of_match,
		.pm = &isa1200_pm,
	},
	.probe = isa1200_probe,
};
module_i2c_driver(isa1200_i2c_driver);

MODULE_AUTHOR("Linus Walleij <linus.walleij@linaro.org>");
MODULE_DESCRIPTION("Immersion ISA1200 haptic feedback unit");
MODULE_LICENSE("GPL");
