// SPDX-License-Identifier: GPL-2.0-only

#include <linux/input.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/serio.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/mod_devicetable.h>
#include <linux/gpio/consumer.h>

#define KEYBOARD_SIZE   128
#define US_KEYBOARD     0xeb
#define UK_KEYBOARD     0xec

#define KEYBOARD_MIN   0x4
#define KEYBOARD_MAX   0x80

static struct serio_device_id sec_serio_ids[] = {
	{
		.type	= SERIO_RS232,
		.proto	= 0x3c,
		.id		= SERIO_ANY,
		.extra	= SERIO_ANY,
	},
	{ 0 }
};

MODULE_DEVICE_TABLE(serio, sec_serio_ids);

enum KEY_LAYOUT {
	UNKOWN_KEYLAYOUT = 0,
	US_KEYLAYOUT,
	UK_KEYLAYOUT,
};

struct ekd_k14_data {

	struct input_dev *input_dev;
	struct device *keyboard_dev;
	struct delayed_work remap_dwork;
	struct delayed_work power_dwork;
	struct delayed_work ack_dwork;
	struct serio *serio;
	struct serio_driver serio_driver;

	bool dockconnected;
	bool pre_connected;
	bool pressed[KEYBOARD_SIZE];
	bool tx_ready;
	bool univ_kbd_dock;

	unsigned int remap_key;
	unsigned int kl;
	unsigned int pre_kl;
	unsigned int ack_code;
	unsigned short keycode[KEYBOARD_SIZE];

};

static const unsigned short sec_keycodes[KEYBOARD_SIZE] = {
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_A, KEY_B, KEY_C, KEY_D,
	KEY_E, KEY_F, KEY_G, KEY_H, KEY_I, KEY_J, KEY_K, KEY_L, KEY_M, KEY_N, KEY_O, KEY_P,
	KEY_Q, KEY_R, KEY_S, KEY_T, KEY_U, KEY_V, KEY_W, KEY_X, KEY_Y, KEY_Z, KEY_1, KEY_2,
	KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9, KEY_0, KEY_ENTER, KEY_BACK,
	KEY_BACKSPACE, KEY_TAB, KEY_SPACE, KEY_MINUS, KEY_EQUAL, KEY_LEFTBRACE,
	KEY_RIGHTBRACE, KEY_HOME, KEY_RESERVED, KEY_SEMICOLON, KEY_APOSTROPHE, KEY_GRAVE,
	KEY_COMMA, KEY_DOT, KEY_SLASH, KEY_CAPSLOCK, KEY_TIME, KEY_F3, KEY_WWW, KEY_EMAIL,
	KEY_SCREENLOCK, KEY_BRIGHTNESSDOWN, KEY_BRIGHTNESSUP, KEY_MUTE, KEY_VOLUMEDOWN,
	KEY_VOLUMEUP, KEY_PLAY, KEY_REWIND, KEY_F15, KEY_RESERVED, KEY_FASTFORWARD, KEY_MENU,
	KEY_RESERVED, KEY_RESERVED, KEY_DELETE, KEY_RESERVED, KEY_RESERVED, KEY_RIGHT,
	KEY_LEFT, KEY_DOWN, KEY_UP, KEY_NUMLOCK, KEY_KPSLASH, KEY_APOSTROPHE, KEY_KPMINUS,
	KEY_KPPLUS, KEY_KPENTER, KEY_KP1, KEY_KP2, KEY_KP3, KEY_KP4, KEY_KP5, KEY_KP6,
	KEY_KP7, KEY_KP8, KEY_KP9, KEY_KPDOT, KEY_RESERVED, KEY_BACKSLASH, KEY_F22,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_HANGEUL, KEY_HANJA,
	KEY_LEFTCTRL, KEY_LEFTSHIFT, KEY_F20, KEY_SEARCH, KEY_RIGHTALT, KEY_RIGHTSHIFT,
	KEY_F21, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_F17,
};

static void sec_keyboard_tx(struct ekd_k14_data *data, u8 cmd)
{
	if (data->pre_connected && data->tx_ready)
		serio_write(data->serio, cmd);
}

static void sec_keyboard_power(struct work_struct *work)
{
	struct ekd_k14_data *data = container_of(work,
			struct ekd_k14_data, power_dwork.work);

	pr_info("Keyboard layout %d\n", data->kl);

	if (UNKOWN_KEYLAYOUT == data->kl) {
		//acc_power(data, 1, false);
		data->pre_connected = false;

		//check_uart_path(data, false);
	}
}

static void sec_keyboard_remapkey(struct work_struct *work)
{
	unsigned int keycode = 0;
	struct ekd_k14_data *data = container_of(work,
			struct ekd_k14_data, remap_dwork.work);

	pr_info("%s ", __func__);

	if (data->pressed[0x45] || data->pressed[0x48]) {
		pr_info("%s inside", __func__);
		keycode = data->keycode[data->remap_key];
		input_report_key(data->input_dev, keycode, 1);
		input_sync(data->input_dev);
	}
	data->remap_key = 0;
}

static void sec_keyboard_ack(struct work_struct *work)
{
	unsigned int ackcode = 0;
	struct ekd_k14_data *data = container_of(work,
			struct ekd_k14_data, ack_dwork.work);

	pr_info("%s\n", __func__);

	if (data->ack_code) {
		pr_info("%s: there is an ack code\n", __func__);
		ackcode = data->ack_code;
		sec_keyboard_tx(data, ackcode);
	}

	if (ackcode == 0x68) {
		data->univ_kbd_dock = true;
		pr_info("%s: univ_kbd_dock\n", __func__);
	}
		

	pr_info("Ack code to KBD 0x%x\n", ackcode);

	// FIXME: data->noti_univ_kbd_dock(data->ack_code);
}

static void release_all_keys(struct ekd_k14_data *data)
{
	int i;
	pr_info("Release the pressed keys.\n");
	for (i = 0; i < KEYBOARD_MAX; i++) {
		if (data->pressed[i]) {
			input_report_key(data->input_dev, data->keycode[i], 0);
			data->pressed[i] = false;
		}
		input_sync(data->input_dev);
	}
}

static void sec_keyboard_process_data(
	struct ekd_k14_data *data, u8 scan_code)
{
	bool press;
	unsigned int keycode;

	pr_info("%s: scan_code %x\n", __func__, scan_code);

	/* keyboard driver need the contry code*/
	if (data->kl == UNKOWN_KEYLAYOUT) {
		switch (scan_code) {
		case US_KEYBOARD:
			data->kl = US_KEYLAYOUT;
			data->keycode[49] = KEY_BACKSLASH;
			/* for the wakeup state*/
			data->pre_kl = data->kl;
			pr_info("US keyboard is attacted.\n");
			break;

		case UK_KEYBOARD:
			data->kl = UK_KEYLAYOUT;
			data->keycode[49] = KEY_NUMERIC_POUND;
			/* for the wakeup state*/
			data->pre_kl = data->kl;
			pr_info("UK keyboard is attacted.\n");
			break;

		default:
			pr_info("Unkown layout : %x\n",
				scan_code);
			break;
		}
	} else {
		switch (scan_code) {
		case 0x0:
			release_all_keys(data);
			break;

		case 0xca: /* Caps lock on */
		case 0xcb: /* Caps lock off */
		case 0xeb: /* US keyboard */
		case 0xec: /* UK keyboard */
			break; /* Ignore */

		case 0x45:
		case 0x48:
			data->remap_key = scan_code;
			data->pressed[scan_code] = true;
			schedule_delayed_work(&data->remap_dwork, HZ/3);
			break;
		case 0x68:
		case 0x69:
		case 0x6a:
		case 0x6b:
		case 0x6c:
			data->ack_code = scan_code;
			schedule_delayed_work(&data->ack_dwork, HZ/200);
			pr_info("scan_code %d Received.\n",
				scan_code);
			break;
		case 0xc5:
		case 0xc8:
			keycode = (scan_code & 0x7f);
			data->pressed[keycode] = false;
			if (0 == data->remap_key) {
				input_report_key(data->input_dev,
					data->keycode[keycode], 0);
				input_sync(data->input_dev);
			} else {
				cancel_delayed_work_sync(&data->remap_dwork);
				if (0x48 == keycode)
					keycode = KEY_NEXTSONG;
				else
					keycode = KEY_PREVIOUSSONG;

				input_report_key(data->input_dev,
					keycode, 1);
				input_report_key(data->input_dev,
					keycode, 0);
				input_sync(data->input_dev);
			}
			break;

		default:
			keycode = (scan_code & 0x7f);
			press = ((scan_code & 0x80) != 0x80);

			if (keycode >= KEYBOARD_MIN
				|| keycode <= KEYBOARD_MAX) {
				data->pressed[keycode] = press;
				input_report_key(data->input_dev,
					data->keycode[keycode], press);
				input_sync(data->input_dev);
			}
			break;
		}
	}
}

static int sec_keyboard_event(struct input_dev *dev,
			unsigned int type, unsigned int code, int value)
{
	struct ekd_k14_data *data = input_get_drvdata(dev);

	switch (type) {
	case EV_LED:
		if (value)
			sec_keyboard_tx(data, 0xca);
		else
			sec_keyboard_tx(data, 0xcb);

	pr_info("%s, capslock on led value=%d\n",\
		 __func__, value);
		return 0;
	}
	return -1;
}

static ssize_t check_keyboard_connection(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct ekd_k14_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->dockconnected);
}

static DEVICE_ATTR(attached, S_IRUGO, check_keyboard_connection, NULL);

static struct attribute *sec_keyboard_attributes[] = {
	&dev_attr_attached.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = sec_keyboard_attributes,
};

static irqreturn_t sec_keyboard_interrupt(struct serio *serio,
		unsigned char data, unsigned int flags)
{
	struct ekd_k14_data *kb_data = serio_get_drvdata(serio);
	if (kb_data->pre_connected)
		sec_keyboard_process_data(kb_data, data);
	return IRQ_HANDLED;
}

static int sec_keyboard_connect(struct serio *serio, struct serio_driver *drv)
{
	struct ekd_k14_data *data = container_of(drv,
			struct ekd_k14_data, serio_driver);
	pr_info("%s\n", __func__);
	data->serio = serio;
	serio_set_drvdata(serio, data);
	if (serio_open(serio, drv))
		pr_err("failed to open serial port\n");
	else
		data->tx_ready = true;
	return 0;
}

static void sec_keyboard_disconnect(struct serio *serio)
{
	struct ekd_k14_data *data = serio_get_drvdata(serio);
	pr_info("%s", __func__);
	data->tx_ready = false;
	serio_close(serio);
}

static int sec_keyboard_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ekd_k14_data *data;
	struct input_dev *input;
	int i, error;

	data = devm_kzalloc(dev, sizeof(struct ekd_k14_data), GFP_KERNEL);
	if(!data)
		return -ENOMEM;

	input = devm_input_allocate_device(dev);
	if (!input) {
		pr_err("failed to allocate input device.\n");
		return -ENOMEM;
	}

	data->input_dev = input;
	data->dockconnected = false;
	data->pre_connected = false;
	data->univ_kbd_dock = false;
	data->remap_key = 0;
	data->kl = UNKOWN_KEYLAYOUT;
	
	memcpy(data->keycode, sec_keycodes, sizeof(sec_keycodes));

	INIT_DELAYED_WORK(&data->remap_dwork, sec_keyboard_remapkey);
	INIT_DELAYED_WORK(&data->power_dwork, sec_keyboard_power);
	INIT_DELAYED_WORK(&data->ack_dwork, sec_keyboard_ack);

	platform_set_drvdata(pdev, data);
	input_set_drvdata(input, data);

	input->name = pdev->name;
	input->dev.parent = &pdev->dev;
	input->id.bustype = BUS_RS232;
	input->event = sec_keyboard_event;

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_LED, input->evbit);
	__set_bit(LED_CAPSL, input->ledbit);
	/* framework doesn't use repeat event */
	/* __set_bit(EV_REP, input->evbit); */

	for (i = 0; i < KEYBOARD_SIZE; i++) {
		if (KEY_RESERVED != data->keycode[i])
			input_set_capability(input, EV_KEY, data->keycode[i]);
	}

	/* for the UK keyboard */
	input_set_capability(input, EV_KEY, KEY_NUMERIC_POUND);

	/* for the remaped keys */
	input_set_capability(input, EV_KEY, KEY_NEXTSONG);
	input_set_capability(input, EV_KEY, KEY_PREVIOUSSONG);

	/* for the wakeup key */
	input_set_capability(input, EV_KEY, KEY_WAKEUP);

	error = input_register_device(input);
	if (error < 0) {
		pr_err("failed to register input device.\n");
		return -ENOMEM;
	}

	data->serio_driver.driver.name = pdev->name;
	data->serio_driver.id_table = sec_serio_ids;
	data->serio_driver.interrupt = sec_keyboard_interrupt,
	data->serio_driver.connect = sec_keyboard_connect,
	data->serio_driver.disconnect = sec_keyboard_disconnect,

	error = serio_register_driver(&data->serio_driver);
	if (error < 0) {
		pr_err("failed to register serio\n");
		return -ENOMEM;
	}

	error = devm_device_add_group(dev, &attr_group);
	if (error) {
		pr_err("failed to create sysfs group\n");
		goto err_sysfs_create_group;
	}

	return 0;

err_sysfs_create_group:
	serio_unregister_driver(&data->serio_driver);
	return error;

}

static int sec_keyboard_remove(struct platform_device *pdev)
{
	struct ekd_k14_data *data = platform_get_drvdata(pdev);
	serio_unregister_driver(&data->serio_driver);
	return 0;
}

static int sec_keyboard_suspend(struct platform_device *pdev,
			pm_message_t state)
{
	struct ekd_k14_data *data = platform_get_drvdata(pdev);

	if (data->kl != UNKOWN_KEYLAYOUT)
		sec_keyboard_tx(data, 0x10);

	return 0;
}

static const struct of_device_id samsung_ekd_k14_dt_match[] = {
	{ .compatible = "samsung,ekd-k14", },
	{},
};
MODULE_DEVICE_TABLE(of, samsung_ekd_k14_dt_match);

static struct platform_driver samsung_ekd_k14_driver = {
	.driver = {
		.name	= "samsung-ekd-k14",
		.owner	= THIS_MODULE,
		.of_match_table = samsung_ekd_k14_dt_match,
	},
	.probe = sec_keyboard_probe,
	.remove = sec_keyboard_remove,
	.suspend = sec_keyboard_suspend,
};

module_platform_driver(samsung_ekd_k14_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Samsung EKD-K14 Keyboard Dock Driver");
