// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>


struct irq_test_config {
	struct gpio_desc *acc;
	int acc_irq;
	struct gpio_desc *dock;
	int dock_irq;
	struct gpio_desc *charger;
	int charger_irq;
};

static void printgpio(struct irq_test_config *data) {
	int ret = gpiod_get_value(data->acc);
	pr_info("ACC is: %d\n", ret);
	ret = gpiod_get_value(data->dock);
	pr_info("DOCK is: %d\n", ret);
	ret = gpiod_get_value(data->charger);
	pr_info("CHARGER is: %d\n", ret);
}

static irqreturn_t acc_handler(int irq, void *arg) {
	struct irq_test_config *data = arg;
	pr_info("%s: IRQ %d fired", __FUNCTION__, irq);
	printgpio(data);
	return IRQ_HANDLED;
}

static irqreturn_t dock_handler(int irq, void *arg) {
	struct irq_test_config *data = arg;
	pr_info("%s: IRQ %d fired\n", __FUNCTION__, irq);
	printgpio(data);
	return IRQ_HANDLED;
}

static irqreturn_t charger_handler(int irq, void *arg) {
	struct irq_test_config *data = arg;
	pr_info("%s: IRQ %d fired\n", __FUNCTION__, irq);
	printgpio(data);
	return IRQ_HANDLED;
}

static ssize_t gpio_values_show(struct device *dev, struct device_attribute *attr, char *buf) {
	struct irq_test_config *data = dev_get_drvdata(dev);
	buf = "1";
	printgpio(data);
	return sizeof(*buf);
}

static DEVICE_ATTR(irq_values, S_IRUGO, gpio_values_show, NULL);

static struct attribute *fun_attrs[] = {
	&dev_attr_irq_values.attr,
	NULL
};

static const struct attribute_group fun_attr_group = {
	.attrs = fun_attrs,
};

static int irq_test_probe(struct platform_device *pdev) {
	struct irq_test_config *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(struct irq_test_config), GFP_KERNEL);
	if(!data)
		return -ENOMEM;

	// accessory interrupt
	data->acc = devm_gpiod_get(&pdev->dev, "acc", GPIOD_IN);
	if (IS_ERR(data->acc)) {
		ret = PTR_ERR(data->acc);
		goto fail;
	}
	data->acc_irq = gpiod_to_irq(data->acc);

	ret = devm_request_threaded_irq(&pdev->dev, data->acc_irq, NULL, acc_handler, 
			IRQF_ONESHOT | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"acc_detect", data);
	if(ret) {
		pr_info("Failed to register acc interrupt: -%d\n", ret);
	}

	// dock interrupt
	data->dock = devm_gpiod_get(&pdev->dev, "dock", GPIOD_IN);
	if (IS_ERR(data->dock)) {
		ret = PTR_ERR(data->dock);
		goto fail;
	}
	data->dock_irq = gpiod_to_irq(data->dock);

	ret = devm_request_threaded_irq(&pdev->dev, data->dock_irq, NULL, dock_handler,
		IRQF_ONESHOT | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"dock_detect", data);
	if(ret) {
		pr_info("Failed to register dock interrupt: -%d\n", ret);
	}
	
	// charger interrupt
	data->charger = devm_gpiod_get(&pdev->dev, "charger", GPIOD_IN);
	if (IS_ERR(data->charger)) {
		ret = PTR_ERR(data->charger);
		goto fail;
	}
	data->charger_irq = gpiod_to_irq(data->charger);

	ret = devm_request_threaded_irq(&pdev->dev, data->charger_irq, NULL, charger_handler,
		IRQF_ONESHOT | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "charger intr", data);
	if(ret) {
		pr_info("Failed to register charger interrupt: -%d\n", ret);
	}
	
	ret = sysfs_create_group(&pdev->dev.kobj, &fun_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "Failure %d creating sysfs group\n",
			ret);
		return ret;
	}

	dev_set_drvdata(&pdev->dev, data);

fail:
	return ret;

}

static int irq_test_suspend(struct device *dev)
{
	int error;

	struct irq_test_config *data = dev_get_drvdata(dev);

	error = enable_irq_wake(data->acc_irq);
	if (error) {
		dev_err(dev->parent,
			"failed to configure IRQ %d as wakeup source: %d\n",
			data->acc_irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ added as wakeup source: %d", data->acc_irq);

	error = enable_irq_wake(data->charger_irq);
	if (error) {
		dev_err(dev->parent,
			"failed to configure IRQ %d as wakeup source: %d\n",
			data->charger_irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ added as wakeup source: %d", data->charger_irq);
	
	error = enable_irq_wake(data->dock_irq);
	if (error) {
		dev_err(dev->parent,
			"failed to configure IRQ %d as wakeup source: %d\n",
			data->dock_irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ added as wakeup source: %d", data->dock_irq);

	return 0;
}

static int irq_test_resume(struct device *dev)
{
	int error;

	struct irq_test_config *data = dev_get_drvdata(dev);

	error = disable_irq_wake(data->acc_irq);
	if (error) {
		dev_err(dev->parent,
			"failed to remove IRQ %d as wakeup source: %d\n",
			data->acc_irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ removed as wakeup source: %d", data->acc_irq);

	error = disable_irq_wake(data->charger_irq);
	if (error) {
		dev_err(dev->parent,
			"failed to remove IRQ %d as wakeup source: %d\n",
			data->charger_irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ removed as wakeup source: %d", data->charger_irq);

	error = disable_irq_wake(data->dock_irq);
	if (error) {
		dev_err(dev->parent,
			"failed to remove IRQ %d as wakeup source: %d\n",
			data->dock_irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ removed as wakeup source: %d", data->dock_irq);

	return 0;
}


static SIMPLE_DEV_PM_OPS(irq_test_pm_ops, irq_test_suspend, irq_test_resume);

static const struct of_device_id irq_test_of_match[] = {
	{ .compatible = "p4note,irq-test", },
	{},
};
MODULE_DEVICE_TABLE(of, irq_test_of_match);

static struct platform_driver irq_test_driver = {
	.probe			= irq_test_probe,
	.driver			= {
		.name			= "irq_test",
		.pm				= &irq_test_pm_ops,
		.of_match_table	= irq_test_of_match,
	}
};

module_platform_driver(irq_test_driver);

MODULE_DESCRIPTION("IRQ test driver for Samsung P4NOTE");
MODULE_LICENSE("GPL");
