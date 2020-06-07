// SPDX-License-Identifier: GPL-2.0-only

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/notifier.h>
#include <linux/extcon.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

/**
 * what the driver should do:
 * 
 * -> detect cable changes
 * --> check battery full and either activate charging or start full charge worker
 *
 * -> detect battery full irq
 * --> start full charge worker
 *
 * -> detect battery low irq
 * -->
 */

struct p4note_manager_data {

	struct gpio_desc *battery_full;
	int battery_full_irq;
	// struct gpio_desc *battery_low;
	// int battery_low_irq;

	struct power_supply *charger_usb;
	struct power_supply *battery;

	struct notifier_block cable_notifier;
	
	struct device *dev;
	struct extcon_dev *edev;

};

static void extcon_cable_worker(struct p4note_manager_data *data)
{
	union power_supply_propval val;
	struct extcon_dev *edev = data->edev;

	bool is_usb_connected = extcon_get_state(edev, EXTCON_CHG_USB_SDP) > 0;
	bool is_dedicated_charger_connected = extcon_get_state(edev, EXTCON_CHG_USB_DCP) > 0;

	if (is_usb_connected) {
		dev_info(data->dev, "low power charging detected");
		val.intval = POWER_SUPPLY_USB_TYPE_SDP;
		power_supply_set_property(data->charger_usb, POWER_SUPPLY_PROP_USB_TYPE, &val);
	} else if(is_dedicated_charger_connected) {
		if (extcon_get_state(edev, EXTCON_DOCK) > 0) {
			dev_info(data->dev, "high power charging with dock detected");
		} else {
			dev_info(data->dev, "high power charging detected");
		}
		val.intval = POWER_SUPPLY_USB_TYPE_DCP;
		power_supply_set_property(data->charger_usb, POWER_SUPPLY_PROP_USB_TYPE, &val);
	}

	if(is_usb_connected || is_dedicated_charger_connected) {
		if(gpiod_get_value(data->battery_full)) {
			dev_info(data->dev, "battery full, enable full charge worker");
			val.intval = 0;
			power_supply_set_property(data->charger_usb, POWER_SUPPLY_PROP_ONLINE, &val);
		} else {
			dev_info(data->dev, "battery not full, enable charging");
			val.intval = 1;
			power_supply_set_property(data->charger_usb, POWER_SUPPLY_PROP_ONLINE, &val);
		}
	} else {
		val.intval = 0;
		power_supply_set_property(data->charger_usb, POWER_SUPPLY_PROP_ONLINE, &val);
		dev_info(data->dev, "no charger connected, disabled charging");
	}
}

static int extcon_cable_event(struct notifier_block *nb,
					   unsigned long event, void *param)
{
	struct p4note_manager_data *data =
		container_of(nb, struct p4note_manager_data, cable_notifier);
	dev_info(data->dev, "received extcon cable event, triggering scheduled work...");
	extcon_cable_worker(data);
	return NOTIFY_OK;
}

static irqreturn_t battery_full_handler(int irq, void *arg)
{
	struct p4note_manager_data *data = arg;
	dev_info(data->dev, "received full charge interrupt, starting full charge worker, current value is %d (FIXME!)", gpiod_get_value(data->battery_full));
	return IRQ_HANDLED;
}

/*
static irqreturn_t battery_low_handler(int irq, void *arg)
{
	struct p4note_manager_data *data = arg;
	dev_info(data->dev, "received low charge interrupt, starting full charge worker (FIXME!)");
	return IRQ_HANDLED;
}
*/

static int p4note_charger_manager_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct p4note_manager_data *data;
	int err = 0;

	data = devm_kzalloc(dev, sizeof(struct p4note_manager_data), GFP_KERNEL);
	if (err < 0) 
		return -ENOMEM;

	data->dev = dev;

	// extcon
	data->edev = extcon_get_edev_by_phandle(dev, 0);
	if (IS_ERR(data->edev)) {
		if (PTR_ERR(data->edev) != -EPROBE_DEFER)
			dev_err(dev, "missing extcon connection\n");
		return PTR_ERR(data->edev);
	}

	data->cable_notifier.notifier_call = extcon_cable_event;
	devm_extcon_register_notifier_all(dev, data->edev, &data->cable_notifier);

	// gpios and interrupts
	data->battery_full = devm_gpiod_get(dev, "battery-full", GPIOD_IN);
	if (IS_ERR(data->battery_full)) {
		return PTR_ERR(data->battery_full);
	}
	data->battery_full_irq = gpiod_to_irq(data->battery_full);

	err = devm_request_threaded_irq(dev, data->battery_full_irq, NULL, battery_full_handler,
		IRQF_ONESHOT | IRQF_TRIGGER_RISING, "battery full intr", data);
	if (err)
		return err;

	/*
	data->battery_low = devm_gpiod_get(dev, "battery-low", GPIOD_IN);
	if (IS_ERR(data->battery_low)) {
		return PTR_ERR(data->battery_low);
	}
	data->battery_low_irq = gpiod_to_irq(data->battery_low);

	err = devm_request_threaded_irq(dev, data->battery_low_irq, NULL, battery_low_handler,
		IRQF_ONESHOT | IRQF_TRIGGER_FALLING, "battery low intr", data);
	if (err)
		return err;
	*/

	dev_set_drvdata(dev, data);

	// power supplies
	data->charger_usb = devm_power_supply_get_by_phandle(dev, "usb-supply");
	data->battery = devm_power_supply_get_by_phandle(dev, "battery-supply");

	return 0;
}

static int p4note_charger_manager_suspend(struct device *dev)
{
	int error;
	struct p4note_manager_data *data = dev_get_drvdata(dev);

	error = enable_irq_wake(data->battery_full_irq);
	if (error) {
		dev_err(dev->parent,
			"failed to configure IRQ %d as wakeup source: %d\n",
			data->battery_full_irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ added as wakeup source: %d", data->battery_full_irq);

	/*
	error = enable_irq_wake(data->battery_low_irq);
	if (error) {
		dev_err(dev->parent,
			"failed to configure IRQ %d as wakeup source: %d\n",
			data->battery_low_irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ added as wakeup source: %d", data->battery_low_irq);
	*/

	return 0;
}

static int p4note_charger_manager_resume(struct device *dev)
{
	int error;
	struct p4note_manager_data *data = dev_get_drvdata(dev);

	error = disable_irq_wake(data->battery_full_irq);
	if (error) {
		dev_err(dev->parent,
			"failed to remove IRQ %d as wakeup source: %d\n",
			data->battery_full_irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ removed as wakeup source: %d", data->battery_full_irq);

	/*
	error = disable_irq_wake(data->battery_low_irq);
	if (error) {
		dev_err(dev->parent,
			"failed to remove IRQ %d as wakeup source: %d\n",
			data->battery_low_irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ removed as wakeup source: %d", data->battery_low_irq);
	*/

	return 0;
}

static SIMPLE_DEV_PM_OPS(p4note_charger_manager_pm_ops, p4note_charger_manager_suspend, p4note_charger_manager_resume);

static const struct of_device_id p4note_match_table[] = {
	{
		.compatible = "p4note,charger-manager",
	},
	{}
};
MODULE_DEVICE_TABLE(of, p4note_match_table);

static struct platform_driver p4note_charger_manager = {
	.driver = {
		.name = "p4note-charger-manager",
		.pm = &p4note_charger_manager_pm_ops,
		.of_match_table = p4note_match_table,
	},
	.probe = p4note_charger_manager_probe,
};

module_platform_driver(p4note_charger_manager);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Samsung P4NOTE device charging manager.");