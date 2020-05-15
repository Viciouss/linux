// SPDX-License-Identifier: GPL-2.0-or-later

/**
 * Here is some info extracted from the old 3.0 kernel driver mess:
 * 
 * UART selection GPIOs
 * --------------------
 * 
 * UART		val=0	val=1
 * SEL_1	CP		AP
 * SEL_2	FAC?	DOCK
 * 
 * UART is used in context of the keyboard dock only, if the keyboard is
 * connected the values are set to 1/1 and if it is disconnected to 0/0.
 * This is only done after the USB driver actually recognizes the device.
 * 
 * There is more code in the px-switch which is used to manually set the
 * switches via sysfs to either 1/0 in case of "AP", 0/- for "CP" and 1/1 for
 * "DOCK".
 * 
 * 
 * USB selection GPIOs
 * -------------------
 * 
 * The table is sorted by priority, as the states can be set in parallel.
 * Usually it's either AP or CP, the ADC check is set only briefly, the
 * TA path is set in case there is a cable attached but it's not a USB
 * connection.
 * 
 * USB path		SEL_0	SEL_1	SEL_CP (SAFEOUT)	Comment
 * ADC Check	1		0		no change			used to determine cable type for charger, short activation only
 * TA	 		0		0		no change			set if there is no USB connection, 
 * CP			0		1		set to 1			initial value if GPIOs are set up like this (from bootloader probably)
 * AP			1		1		set to 0			set for OTG power, remains set
 * Default		1		1		no change			probably unused
 * 
 * The px-switch has a reference to these, too. Writing "MODEM" to the sysfs
 * file activates the CP path, "PDA" clears the CP path.
 * 
 * 
 * IIO adc channels
 * ----------------
 * 
 * Channel 4: headphone jack on the top of the tablet
 * 
 * Channel 6: cable type (TA, USB, etc.) -> activate ADC check USB path first
 * so that the USB config is in the right state before reading the value
 * 
 * Channel 7: accessory type, this is shared between accessories like USB, the
 * card reader and the dock headphone connector
 * 
 *
 * Charging currents
 * -----------------
 * 
 * This is for the smb347 control
 * 
 * - wall charger => 1800mA, HC mode
 * - dock + wall charger => 1500mA, HC mode
 * - usb via dock/hdmi or direct usb => "USB5" charging mode
 * - fallback => "USB1" charging mode
 *   ->(sound station, does not exist for p4note so it would never be active?)
 * 
 * Dock
 * ----
 * 
 * The dock can be used to connect other hardware as it has a 30pin connector
 * on it's right side as well as a head phone connector on the back.
 * 
 * 
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/iio/consumer.h>
#include <linux/extcon-provider.h>
#include <linux/gpio/consumer.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/delay.h>

static const unsigned int p4note_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_DOCK,
	EXTCON_JACK_LINE_OUT,
	EXTCON_NONE,
};

struct p4note_charger_cond {
	unsigned int type;
	u32 min_adc;
	u32 max_adc;
};

struct p4note_accessory_cond {
	unsigned int type1;
	unsigned int type2;
	u32 min_adc;
	u32 max_adc;
};

enum gpio_state {
	GPIO_OFF,
	GPIO_ON
};

enum usb_path {
	USB_PATH_NONE = 0,
	USB_PATH_ADCCHECK = (1 << 28),
	USB_PATH_TA = (1 << 24),
	USB_PATH_CP = (1 << 20),
	USB_PATH_AP = (1 << 16)
};

struct p4note_gpio {
	struct gpio_desc *desc;
	int irq;
	int last_state;
};

struct p4note_extcon_data {
	struct device *dev;
	struct extcon_dev *edev;

	struct p4note_gpio accessory;
	struct p4note_gpio dock;
	struct p4note_gpio charger;

	struct gpio_desc *accessory_enable;
	struct gpio_desc *accessory_5v;

	struct gpio_desc *uart_sel_1;
	struct gpio_desc *uart_sel_2;

	struct gpio_desc *usb_sel_0;
	struct gpio_desc *usb_sel_1;
	struct gpio_desc *usb_sel_cp;

	struct mutex irq_mutex;

	struct iio_channel *headphone_iio_chan;
	struct iio_channel *charger_iio_chan;
	struct iio_channel *acc_iio_chan;
	
	struct delayed_work acc_adc_work; 
	struct delayed_work charger_adc_work;

	enum usb_path current_path;

	u8 acc_en_token;

	struct p4note_accessory_cond *current_type;

};

#define CHARGER_BATTERY		0
#define CHARGER_USB			1
#define CHARGER_AC			2
#define CHARGER_DOCK		3
#define CHARGER_MISC		4
#define CHARGER_DISCHARGE	5

/**
 * this is the charger connector, these values should be valid for direct
 * connections as well as connections to the dock
 * 
 * adc channel: 6
 */
static struct p4note_charger_cond charger_adc_conditions[] = {
	{
		.type = CHARGER_AC,
		.min_adc = 800,
		.max_adc = 1800
	},
	{
		// is there a sound station for the p4note? I don't think so
		.type = CHARGER_MISC,
		.min_adc = 550,
		.max_adc = 700
	},
	{}
};

/**
 * this is the head phone connector on the back of the dock
 * 
 * adc channel: 4
 * 
 * here is a list of tested values for the different devices
 * 
 * device	adc value		connected1		connected2
 * n8000	860				3 pole hp		OTG
 * n8000	986				3 pole hp		-
 * n8000	1475			4 pole hp		OTG
 * n8000	1937			4 pole hp		-
 * n8000	2193			OTG				-
 * 
 */
static struct p4note_accessory_cond acc_adc_conditions[] = {
	{
		// 3 pole hp + OTG
		.type1 = EXTCON_JACK_LINE_OUT,
		.type2 = EXTCON_USB_HOST,
		.min_adc = 800,
		.max_adc = 924
	},
	{
		// 3 pole hp
		.type1 = EXTCON_JACK_LINE_OUT,
		.type2 = -1,
		.min_adc = 925,
		.max_adc = 1050
	},
	{
		// 4 pole hp + OTG
		.type1 = EXTCON_JACK_LINE_OUT,
		.type2 = EXTCON_USB_HOST,
		.min_adc = 1350,
		.max_adc = 1600
	},
	{
		// 4 pole head phone
		.type1 = EXTCON_JACK_LINE_OUT,
		.type2 = -1,
		.min_adc = 1800,
		.max_adc = 2059
	},
	{
		// OTG
		.type1 = EXTCON_USB_HOST,
		.type2 = -1,
		.min_adc = 2060,
		.max_adc = 2350
	},
	{}
};

#define LOG_STATE pr_info("accessory(%d) + dock(%d) + charger(%d)\n", data->accessory.last_state, data->dock.last_state, data->charger.last_state)

void smdk_accessory_power(struct p4note_extcon_data *data, u8 token, bool active)
{
	int try_cnt = 0;

	/*
		token info
		0 : power off,
		1 : Keyboard dock
		2 : USB
	*/

	if (active) {
		if (data->acc_en_token) {
			pr_info("Board: Keyboard dock is connected.\n");
			gpiod_set_value(data->accessory_enable, 0);
			msleep(100);
		}

		data->acc_en_token |= (1 << token);
		gpiod_set_value(data->accessory_enable, 1);
		msleep(20);

		/* prevent the overcurrent */
		while (!gpiod_get_value(data->accessory_5v)) {
			gpiod_set_value(data->accessory_enable, 0);
			msleep(20);
			gpiod_set_value(data->accessory_enable, 1);
			if (try_cnt > 10) {
				pr_err("[acc] failed to enable the accessory_en");
				gpiod_set_value(data->accessory_enable, 0);
				break;
			} else
				try_cnt++;
		}

	} else {
		if (0 == token) {
			gpiod_set_value(data->accessory_enable, 0);
		} else {
			data->acc_en_token &= ~(1 << token);
			if (0 == data->acc_en_token) {
				gpiod_set_value(data->accessory_enable, 0);
			}
		}
	}

	pr_info("Board : %s (%d,%d) %s\n", __func__,
		token, active, data->acc_en_token ? "on" : "off");
}

static void pmic_safeout2(struct p4note_extcon_data *data, int onoff)
{
	struct gpio_desc *cp = data->usb_sel_cp;
	if (onoff) {
		if (!gpiod_get_value(cp)) {
			gpiod_set_value(cp, onoff);
			pr_info("changed usb cp to ON\n");
		} else {
			pr_info("%s: onoff:%d No change in safeout2\n",
				   __func__, onoff);
		}
	} else {
		if (gpiod_get_value(cp)) {
			gpiod_set_value(cp, onoff);
			pr_info("changed usb cp to OFF\n");
		} else {
			pr_info("%s: onoff:%d No change in safeout2\n",
				   __func__, onoff);
		}
	}
}

static void usb_apply_path(struct p4note_extcon_data *data)
{
	enum usb_path path = data->current_path;
	struct gpio_desc *sel0 = data->usb_sel_0;
	struct gpio_desc *sel1 = data->usb_sel_1;
	struct gpio_desc *cp = data->usb_sel_cp;
	
	pr_info("%s: current gpio before changing : sel0:%d sel1:%d sel_cp:%d\n",
		   __func__, gpiod_get_value(sel0),
		   gpiod_get_value(sel1), cp ? gpiod_get_value(cp) : -1);
	pr_info("%s: target path %x\n", __func__, path);

	/* following checks are ordered according to priority */
	if (path & USB_PATH_ADCCHECK) { // charger type detection via stmpe channel 6
		pr_info("set USB path to ADCCHECK\n");
		gpiod_set_value(sel0, 1);
		gpiod_set_value(sel1, 0);
		goto out_nochange;
	}

	if (path & USB_PATH_TA) {
		pr_info("set USB path to TA\n");
		gpiod_set_value(sel0, 0);
		gpiod_set_value(sel1, 0);
		goto out_nochange;
	}

	if (path & USB_PATH_CP) {
		pr_info("set USB path to CP\n");
		gpiod_set_value(sel0, 0);
		gpiod_set_value(sel1, 1);
		mdelay(3);
		goto out_cp;
	}

	if (path & USB_PATH_AP) {
		pr_info("set USB path to AP\n");
		gpiod_set_value(sel0, 1);
		gpiod_set_value(sel1, 1);
		goto out_ap;
	}

	/* default */
	pr_info("set USB path to DEFAULT\n");
	gpiod_set_value(sel0, 1);
	gpiod_set_value(sel1, 1);

out_ap:
	pr_info("%s: %x safeout2 off\n", __func__, path);
	pmic_safeout2(data, 0);

out_cp:
	pr_info("%s: %x safeout2 on\n", __func__, path);
	pmic_safeout2(data, 1);

out_nochange:
	pr_info("%s: %x safeout2 no change\n", __func__, path);
	return;

}

static void usb_switch_set_path(struct p4note_extcon_data *data, enum usb_path path)
{
	// avoid gpio updates if not necessary
	if((data->current_path | path) != data->current_path) {
		pr_info("%s: current_path before changing -> %x\n", __func__, data->current_path);
		data->current_path |= path;
		usb_apply_path(data);
	} else {
		pr_info("%s: setting path unnecessary, skipping", __func__);
	}
}

static void usb_switch_clr_path(struct p4note_extcon_data *data, enum usb_path path)
{
	// avoid gpio updates if not necessary
	if((data->current_path & ~path) != data->current_path) {
		pr_info("%s: current_path before changing -> %x\n", __func__, data->current_path);
		data->current_path &= ~path;
		usb_apply_path(data);
	} else {
		pr_info("%s: clearing path unnecessary, skipping", __func__);
	}
}

static void enable_usb(struct p4note_extcon_data *data) {
	mutex_lock(&data->irq_mutex);
	usb_switch_set_path(data, USB_PATH_AP);
	smdk_accessory_power(data, 2, true);
	mutex_unlock(&data->irq_mutex);
}

static void disable_usb(struct p4note_extcon_data *data) {
	mutex_lock(&data->irq_mutex);
	usb_switch_clr_path(data, USB_PATH_AP);
	smdk_accessory_power(data, 2, false);
	mutex_unlock(&data->irq_mutex);
}

static void start_acc_worker(struct p4note_extcon_data *data, int timeout) {
	cancel_delayed_work(&data->acc_adc_work);
	schedule_delayed_work(&data->acc_adc_work, msecs_to_jiffies(timeout));
	pr_info("new work added, starting in %d millis\n", timeout);
}

static irqreturn_t irq_handler_accessory(int irq, void *arg) {
	struct p4note_extcon_data *data = arg;
	
	msleep(200);
	
	if(gpiod_get_value(data->accessory.desc)) {
		if(data->accessory.last_state == GPIO_OFF) {
			pr_info("%s: connecting accessory...\n", __FUNCTION__);
			data->accessory.last_state = GPIO_ON;
			start_acc_worker(data, 0);
		} else {
			pr_info("%s: accessory already connected...\n", __FUNCTION__);
		}
	} else {
		if(data->accessory.last_state == GPIO_ON) {
			pr_info("%s: disconnecting accessory...\n", __FUNCTION__);
			data->accessory.last_state = GPIO_OFF;
			data->current_type = NULL;
			cancel_delayed_work(&data->acc_adc_work);
			disable_usb(data);
		} else {
			pr_info("%s: accessory already disconnected...\n", __FUNCTION__);
		}
	}
	
	LOG_STATE;
	return IRQ_HANDLED;
}

static irqreturn_t irq_handler_dock(int irq, void *arg) {
	struct p4note_extcon_data *data = arg;
	
	if(gpiod_get_value(data->dock.desc)) {
		if(data->dock.last_state == GPIO_OFF) {
			pr_info("%s: connecting dock...\n", __FUNCTION__);
			data->dock.last_state = GPIO_ON;
		} else {
			pr_info("%s: dock already connected...\n", __FUNCTION__);
		}
	} else {
		if(data->dock.last_state == GPIO_ON) {
			pr_info("%s: disconnecting dock...\n", __FUNCTION__);
			data->dock.last_state = GPIO_OFF;
		} else {
			pr_info("%s: dock already disconnected...\n", __FUNCTION__);
		}
	}

	LOG_STATE;
	// FIXMEs: 
	// * determine type of dock (desk dock = MHL connection - or keyboard dock)
	// * enable corresponding driver
	// * make sure that usb is working correctly if there is a dock connected
	return IRQ_HANDLED;
}

static irqreturn_t irq_handler_charger(int irq, void *arg) {
	struct p4note_extcon_data *data = arg;

	if(gpiod_get_value(data->charger.desc)) {
		if(data->charger.last_state == GPIO_OFF) {
			pr_info("%s: connecting charger...\n", __FUNCTION__);
			data->charger.last_state = GPIO_ON;

			cancel_delayed_work(&data->charger_adc_work);
			schedule_delayed_work(&data->charger_adc_work, msecs_to_jiffies(200));
		} else {
			pr_info("%s: charger already connected...\n", __FUNCTION__);
		}
	} else {
		if(data->charger.last_state == GPIO_ON) {
			pr_info("%s: disconnecting charger...\n", __FUNCTION__);
			data->charger.last_state = GPIO_OFF;
			cancel_delayed_work(&data->charger_adc_work);
		} else {
			pr_info("%s: charger already disconnected...\n", __FUNCTION__);
		}
	}

	LOG_STATE;
	// FIXMEs:
	// * turn on charging with the right settings
	return IRQ_HANDLED;
}

static void read_accessory_adc_worker(struct work_struct *work) {

	struct delayed_work *dwork = to_delayed_work(work);
	struct p4note_extcon_data *data =
		container_of(dwork, struct p4note_extcon_data, acc_adc_work);

	int err = 0;
	int adc_val = 0;
	int condition_counter = 0;

	err = iio_read_channel_processed(data->acc_iio_chan, &adc_val);

	if (err) {
		pr_info("there was an error reading the adc value for the accessory: %d\n", err);
		return;
	}

	pr_info("read accessory adc value = %d\n", adc_val);
	
	while(acc_adc_conditions[condition_counter].type1) {
		int min = acc_adc_conditions[condition_counter].min_adc;
		int max = acc_adc_conditions[condition_counter].max_adc;
		if(min <= adc_val && adc_val <= max) {
			break;
		}
		condition_counter++;
	}

	if(data->current_type != &acc_adc_conditions[condition_counter]) {
		
		pr_info("%s: something was changed in the current setup, setting up new type", __FUNCTION__);

		if(data->current_type) {
			extcon_set_state_sync(data->edev, data->current_type->type1, false);
			if(data->current_type->type2 >= 0) {
				extcon_set_state_sync(data->edev, data->current_type->type2, false);
			}
			data->current_type = NULL;
		}

		if(acc_adc_conditions[condition_counter].type1) {

			data->current_type = &acc_adc_conditions[condition_counter];
			pr_info("device attached with type1=%d and type2=%d\n", data->current_type->type1, data->current_type->type2);
			
			if(data->current_type->type1 == EXTCON_USB_HOST || data->current_type->type2 == EXTCON_USB_HOST) {
				enable_usb(data);
			} else {
				disable_usb(data);
			}

			extcon_set_state_sync(data->edev, data->current_type->type1, true);
			if(data->current_type->type2 >= 0) {
				extcon_set_state_sync(data->edev, data->current_type->type2, true);
			}

		} else {
			pr_info("unknown device attached with adc value %d\n", adc_val);
			disable_usb(data);
		}
	} else {
		pr_info("%s: the cable did not change, do nothing", __FUNCTION__);
	}

	if(data->accessory.last_state == GPIO_ON && data->dock.last_state == GPIO_ON) {
		// both the dock and the accessory are connected, poll the adc value to get any updates
		pr_info("rescheduling adc task as both accessory and dock line are set\n");
		start_acc_worker(data, 3000);
	} else {
		if(data->dock.last_state == GPIO_ON) {
			pr_info("acc removed or not present, stop polling\n");
		} else {
			pr_info("dock removed or not present, stop polling\n");
		}
		
	}

}

static void read_charger_adc_worker(struct work_struct *work) {

	struct delayed_work *dwork = to_delayed_work(work);
	struct p4note_extcon_data *data =
		container_of(dwork, struct p4note_extcon_data, charger_adc_work);

	int err = 0;
	int adc_val = 0;
	int condition_counter = 0;

	mutex_lock(&data->irq_mutex);
	usb_switch_set_path(data, USB_PATH_ADCCHECK);
	err = iio_read_channel_processed(data->charger_iio_chan, &adc_val);
	usb_switch_clr_path(data, USB_PATH_ADCCHECK);
	mutex_unlock(&data->irq_mutex);

	if (err) {
		pr_info("there was an error reading the adc value for the charger: %d\n", err);
		return;
	}

	pr_info("read charger adc value = %d\n", adc_val);

	while(charger_adc_conditions[condition_counter].type) {
		int min = charger_adc_conditions[condition_counter].min_adc;
		int max = charger_adc_conditions[condition_counter].max_adc;
		if(min <= adc_val && adc_val <= max) {
			break;
		}
		condition_counter++;
	}

	if(charger_adc_conditions[condition_counter].type) {
		struct p4note_charger_cond hit = charger_adc_conditions[condition_counter];
		pr_info("the connected charger is of type %d\n", hit.type);
		// FIXME: set charger type and enable charging for the correct cable
	} else {
		pr_info("could not determine charger, default is USB\n");
		// FIXME: set charger type
	}

}

static int p4note_extcon_probe(struct platform_device *pdev) {

	int ret = 0;
	struct device *dev = &pdev->dev;
	struct p4note_extcon_data *data;
	
	pr_info("%s: p4note probe...\n", __FUNCTION__);

	data = devm_kzalloc(dev, sizeof(struct p4note_extcon_data), GFP_KERNEL);
	if(!data)
		return -ENOMEM;
	
	pr_info("%s: got some memory\n", __FUNCTION__);

	// accessory interrupt
	data->accessory.desc = devm_gpiod_get(dev, "acc", GPIOD_IN);
	if (IS_ERR(data->accessory.desc)) {
		return PTR_ERR(data->accessory.desc);
	}
	data->accessory.irq = gpiod_to_irq(data->accessory.desc);

	ret = devm_request_threaded_irq(dev, data->accessory.irq, NULL, irq_handler_accessory, 
			IRQF_ONESHOT | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"acc_detect", data);
	if(ret)
		return ret;
	
	data->accessory.last_state = gpiod_get_value(data->accessory.desc);
	// FIXME: fire acc detection
	pr_info("%s: acc registerd with initial value %d\n", __FUNCTION__, data->accessory.last_state);

	// (keyboard) dock interrupt
	data->dock.desc = devm_gpiod_get(dev, "dock", GPIOD_IN);
	if (IS_ERR(data->dock.desc)) {
		return PTR_ERR(data->dock.desc);
	}
	data->dock.irq = gpiod_to_irq(data->dock.desc);

	ret = devm_request_threaded_irq(dev, data->dock.irq, NULL, irq_handler_dock,
		IRQF_ONESHOT | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"dock_detect", data);
	if(ret)
		return ret;

	data->dock.last_state = gpiod_get_value(data->dock.desc);
	// FIXME: fire dock detection
	pr_info("%s: dock registerd with initial value %d\n", __FUNCTION__, data->dock.last_state);

	// charger interrupt
	data->charger.desc = devm_gpiod_get(dev, "charger", GPIOD_IN);
	if (IS_ERR(data->charger.desc)) {
		return PTR_ERR(data->charger.desc);
	}
	data->charger.irq = gpiod_to_irq(data->charger.desc);

	ret = devm_request_threaded_irq(dev, data->charger.irq, NULL, irq_handler_charger,
		IRQF_ONESHOT | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "charger intr", data);
	if(ret)
		return ret;
	
	data->charger.last_state = gpiod_get_value(data->charger.desc);
	// FIXME: fire charger detection
	pr_info("%s: charger registerd with initial value %d\n", __FUNCTION__, data->charger.last_state);

	// accessory enable + 5v 
	data->accessory_enable = devm_gpiod_get(&pdev->dev, "acc_en", GPIOD_OUT_HIGH);
	if (IS_ERR(data->accessory_enable)) {
		dev_err(&pdev->dev, "failed to get accessory_enable GPIO\n");
		return PTR_ERR(data->accessory_enable);
	}

	data->accessory_5v = devm_gpiod_get(&pdev->dev, "acc_5v", GPIOD_IN);
	if (IS_ERR(data->accessory_5v)) {
		dev_err(&pdev->dev, "failed to get accessory_5v GPIO\n");
		return PTR_ERR(data->accessory_5v);
	}

	// adc channel setup
	data->headphone_iio_chan = devm_iio_channel_get(&pdev->dev, "headphone");
	if (IS_ERR(data->headphone_iio_chan)) {
		if (PTR_ERR(data->headphone_iio_chan) == -ENODEV) {
			pr_info("%s: headphone adc is not ready, deferring\n", __FUNCTION__);
			return -EPROBE_DEFER;
		}
		pr_info("%s: headphone error %ld\n", __FUNCTION__, PTR_ERR(data->headphone_iio_chan));
		return PTR_ERR(data->headphone_iio_chan);
	}
	pr_info("%s: headphone adc found\n", __FUNCTION__);

	data->charger_iio_chan = devm_iio_channel_get(&pdev->dev, "charger");
	if (IS_ERR(data->charger_iio_chan)) {
		if (PTR_ERR(data->charger_iio_chan) == -ENODEV) {
			pr_info("%s: charger adc is not ready, deferring\n", __FUNCTION__);
			return -EPROBE_DEFER;
		}
		pr_info("%s: charger error %ld\n", __FUNCTION__, PTR_ERR(data->headphone_iio_chan));
		return PTR_ERR(data->charger_iio_chan);
	}
	pr_info("%s: charger adc found\n", __FUNCTION__);

	data->acc_iio_chan = devm_iio_channel_get(&pdev->dev, "accessory");
	if (IS_ERR(data->acc_iio_chan)) {
		if (PTR_ERR(data->acc_iio_chan) == -ENODEV) {
			pr_info("%s: accessory adc is not ready, deferring\n", __FUNCTION__);
			return -EPROBE_DEFER;
		}
		pr_info("%s: accessory error %ld\n", __FUNCTION__, PTR_ERR(data->headphone_iio_chan));
		return PTR_ERR(data->acc_iio_chan);
	}
	pr_info("%s: accessory adc found\n", __FUNCTION__);

	INIT_DELAYED_WORK(&data->acc_adc_work, read_accessory_adc_worker);
	INIT_DELAYED_WORK(&data->charger_adc_work, read_charger_adc_worker);
	pr_info("%s: delayed work initalized\n", __FUNCTION__);

	// usb path setup
	data->usb_sel_0 = devm_gpiod_get(&pdev->dev, "usb0", GPIOD_OUT_HIGH);
	if (IS_ERR(data->usb_sel_0)) {
		dev_err(&pdev->dev, "failed to get usb_sel_0 GPIO\n");
		return PTR_ERR(data->usb_sel_0);
	}

	data->usb_sel_1 = devm_gpiod_get(&pdev->dev, "usb1", GPIOD_OUT_HIGH);
	if (IS_ERR(data->usb_sel_1)) {
		dev_err(&pdev->dev, "failed to get usb_sel_1 GPIO\n");
		return PTR_ERR(data->usb_sel_1);
	}

	data->usb_sel_cp = devm_gpiod_get(&pdev->dev, "cp", GPIOD_OUT_HIGH);
	if (IS_ERR(data->usb_sel_cp)) {
		dev_err(&pdev->dev, "failed to get usb_sel_cp GPIO\n");
		return PTR_ERR(data->usb_sel_cp);
	}

	/**
	ret = gpiod_export(data->usb_sel_0, 1);
	if (ret)
		return ret;

	ret = gpiod_export(data->usb_sel_1, 1);
	if (ret)
		return ret;

	ret = gpiod_export(data->usb_sel_cp, 1);
	if (ret)
		return ret;
	*/

	data->current_path = USB_PATH_NONE;

	if ((!gpiod_get_value(data->usb_sel_0)) && (gpiod_get_value(data->usb_sel_1))) {
		usb_switch_set_path(data, USB_PATH_CP);
	}


	// uart path setup
	data->uart_sel_1 = devm_gpiod_get(&pdev->dev, "uart1", GPIOD_OUT_HIGH);
	if (IS_ERR(data->uart_sel_1)) {
		dev_err(&pdev->dev, "failed to get uart_sel_1 GPIO\n");
		return PTR_ERR(data->uart_sel_1);
	}

	data->uart_sel_2 = devm_gpiod_get_optional(&pdev->dev, "uart2", GPIOD_OUT_HIGH);
	if (IS_ERR(data->uart_sel_2)) {
		dev_err(&pdev->dev, "failed to get uart_sel_2 GPIO\n");
		return PTR_ERR(data->uart_sel_2);
	}

	/**
	ret = gpiod_export(data->uart_sel_1, 1);
	if (ret)
		return ret;

	if(data->uart_sel_2) {
		ret = gpiod_export(data->uart_sel_2, 1);
		if (ret)
			return ret;
	}
	*/

	mutex_init(&data->irq_mutex);
	pr_info("%s: mutex init done\n", __FUNCTION__);

	dev_set_drvdata(dev, data);

	// register extcon device
	data->edev = devm_extcon_dev_allocate(&pdev->dev,
			p4note_extcon_cable);
	if (IS_ERR(data->edev)) {
		dev_err(&pdev->dev, "Failed to allocate memory for extcon\n");
		return -ENODEV;
	}

	ret = devm_extcon_dev_register(&pdev->dev, data->edev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register extcon device\n");
		return ret;
	}

	return 0;
}

static int extcon_p4note_suspend(struct device *dev)
{
	int error;
	struct p4note_extcon_data *data = dev_get_drvdata(dev);

	error = enable_irq_wake(data->accessory.irq);
	if (error) {
		dev_err(dev->parent,
			"failed to configure IRQ %d as wakeup source: %d\n",
			data->accessory.irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ added as wakeup source: %d", data->accessory.irq);

	error = enable_irq_wake(data->charger.irq);
	if (error) {
		dev_err(dev->parent,
			"failed to configure IRQ %d as wakeup source: %d\n",
			data->charger.irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ added as wakeup source: %d", data->charger.irq);
	
	error = enable_irq_wake(data->dock.irq);
	if (error) {
		dev_err(dev->parent,
			"failed to configure IRQ %d as wakeup source: %d\n",
			data->dock.irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ added as wakeup source: %d", data->dock.irq);

	return 0;
}

static int extcon_p4note_resume(struct device *dev)
{
	int error;

	struct p4note_extcon_data *data = dev_get_drvdata(dev);

	error = disable_irq_wake(data->accessory.irq);
	if (error) {
		dev_err(dev->parent,
			"failed to remove IRQ %d as wakeup source: %d\n",
			data->accessory.irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ removed as wakeup source: %d", data->accessory.irq);

	error = disable_irq_wake(data->charger.irq);
	if (error) {
		dev_err(dev->parent,
			"failed to remove IRQ %d as wakeup source: %d\n",
			data->charger.irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ removed as wakeup source: %d", data->charger.irq);

	error = disable_irq_wake(data->dock.irq);
	if (error) {
		dev_err(dev->parent,
			"failed to remove IRQ %d as wakeup source: %d\n",
			data->dock.irq, error);
		return error;
	}
	dev_info(dev->parent, "IRQ removed as wakeup source: %d", data->dock.irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(extcon_p4note_pm_ops, extcon_p4note_suspend, extcon_p4note_resume);

static const struct of_device_id p4note_extcon_match[] = {
	{ .compatible = "extcon,p4note", },
	{},
};
MODULE_DEVICE_TABLE(of, p4note_extcon_match);

static struct platform_driver p4note_extcon_driver = {
	.driver		= {
		.name	= "extcon_p4note",
		.pm				= &extcon_p4note_pm_ops,
		.of_match_table	= p4note_extcon_match,
	},
	.probe		= p4note_extcon_probe,
};

module_platform_driver(p4note_extcon_driver);

MODULE_DESCRIPTION("Samsung P4Note external connector");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:extcon-p4note");

