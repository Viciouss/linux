/* linux/drivers/video/samsung/s3cfb_s6c1372.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S6F1202A : 7" WSVGA Landscape LCD module driver
 * S6C1372 : 7.3" WXGA Landscape LCD module driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/lcd.h>


struct s6c1372_lcd {
	unsigned int			power;
	struct device			*dev;
	struct lcd_device		*ld;
	int gpio_reset;
	int gpio_lcds_nshdn;
};

static int lvds_lcd_set_power(struct lcd_device *ld, int power)
{
	struct s6c1372_lcd *lcd = lcd_get_data(ld);
	if (power != FB_BLANK_UNBLANK && power != FB_BLANK_POWERDOWN &&
		power != FB_BLANK_NORMAL) {
		dev_err(&lcd->ld->dev, "power value should be 0, 1 or 4.\n");
		return -EINVAL;
	}

	if (power)
	{
		/* LVDS_N_SHDN to high*/
		mdelay(1);
		gpio_set_value(lcd->gpio_lcds_nshdn, 1);
		msleep(300);

		gpio_set_value(lcd->gpio_reset, 1);
		mdelay(2);
	}
	else
	{
		gpio_set_value(lcd->gpio_reset, 0);
		msleep(200);

		/* LVDS_nSHDN low*/
		gpio_set_value(lcd->gpio_lcds_nshdn, 0);
		msleep(40);
	}
	lcd->power = power;
	return 0;
}

static int lvds_lcd_get_power(struct lcd_device *ld)
{
	struct s6c1372_lcd *lcd = lcd_get_data(ld);
	return lcd->power;
}

static struct lcd_ops s6c1372_ops = {
	.set_power = lvds_lcd_set_power,
	.get_power = lvds_lcd_get_power,
};

static struct s6c1372_lcd *s6c1372_parse_dt(struct device *dev)
{
	struct device_node *np = of_node_get(dev->of_node);
	struct s6c1372_lcd *pdata;

	if (!np) {
		dev_err(dev, "s6c1372 node not found.\n");
		return ERR_PTR(-ENODATA);
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		of_node_put(np);
		return ERR_PTR(-ENOMEM);
	}

	pdata->gpio_reset = of_get_gpio(dev->of_node, 0); // GPIO_LED_BACKLIGHT_RESET
	pdata->gpio_lcds_nshdn = of_get_gpio(dev->of_node, 1); // GPIO_LVDS_NSHDN

	return pdata;
}


static int s6c1372_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s6c1372_lcd *lcd;
	int ret = 0;

	lcd = s6c1372_parse_dt(dev);
		if (IS_ERR(lcd))
			return PTR_ERR(lcd);

	lcd->ld = devm_lcd_device_register(dev, "panel", &pdev->dev, lcd, &s6c1372_ops);

	if (IS_ERR(lcd->ld)) {
		pr_err("failed to register lcd device\n");
		return PTR_ERR(lcd->ld);
	}

	lcd->power = 1;

	dev_set_drvdata(&pdev->dev, lcd);

	dev_info(&lcd->ld->dev, "lcd panel driver has been probed.\n");

	return  0;
}

static int s6c1372_remove(struct platform_device *pdev)
{
	struct s6c1372_lcd *lcd = dev_get_drvdata(&pdev->dev);

	lcd_device_unregister(lcd->ld);

	return 0;
}

static void s6c1372_shutdown(struct platform_device *pdev)
{
	struct s6c1372_lcd *lcd = dev_get_drvdata(&pdev->dev);
	
	gpio_set_value(lcd->gpio_reset, 0);
	msleep(200);

}

static const struct of_device_id s6c1372_lcd_of_match[] = {
	{ .compatible = "samsung,s6c1372", },
	{ }
};
MODULE_DEVICE_TABLE(of, s6c1372_lcd_of_match);

static struct platform_driver s6c1372_driver = {
	.driver = {
		.name	= "s6c1372",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(s6c1372_lcd_of_match),
	},
	.probe		= s6c1372_probe,
	.remove		= s6c1372_remove,
	.shutdown		= s6c1372_shutdown,
};

module_platform_driver(s6c1372_driver);

MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("S6C1372 LCD driver");
MODULE_LICENSE("GPL");

