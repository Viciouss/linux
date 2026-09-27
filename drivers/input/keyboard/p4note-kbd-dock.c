// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Keyboard dock driver for the Samsung Galaxy Note 10.1 (p4note).
 *
 * Based on sec_keyboard.c from the Samsung p4note GPL kernel.
 *
 * The keyboard dock talks to the tablet over UART2 through the 30-pin
 * connector, 9600 8N1. The dock connector driver (extcon-p4note) detects the
 * dock, power cycles the accessory 5V rail and routes the UART to the dock,
 * then reports EXTCON_DOCK. This driver reports the handshake result back.
 *
 * Right after power-up the keyboard sends its layout byte (0xeb US, 0xec UK).
 * Scancodes are USB HID usage ids, bit 7 set for a release, 0x00 releases all
 * keys. The tablet can send 0xca/0xcb (caps lock LED on/off) and 0x10 (idle).
 *
 * Like the vendor kernel, a dock that does not answer within 700ms is not a
 * keyboard. extcon-p4note then switches the dock over to the desk dock (MHL).
 */

#include <linux/bitmap.h>
#include <linux/completion.h>
#include <linux/extcon.h>
#include <linux/extcon/extcon-p4note.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/workqueue.h>

#define KBD_BAUDRATE		9600
#define KBD_NUM_KEYS		128
#define KBD_HANDSHAKE_MS	700
#define KBD_REMAP_MS		333

/* keyboard -> tablet */
#define KBD_ALL_RELEASED	0x00
#define KBD_LAYOUT_US		0xeb
#define KBD_LAYOUT_UK		0xec
#define KBD_CAPS_ON_ECHO	0xca
#define KBD_CAPS_OFF_ECHO	0xcb
#define KBD_UNIV_FIRST		0x68
#define KBD_UNIV_LAST		0x6c
#define KBD_RELEASE		BIT(7)
#define KBD_CODE_MASK		0x7f

/* tablet -> keyboard */
#define KBD_CMD_CAPS_ON		0xca
#define KBD_CMD_CAPS_OFF	0xcb
#define KBD_CMD_IDLE		0x10

/* keys that send a track skip on a short tap */
#define KBD_SCAN_REWIND		0x45
#define KBD_SCAN_FASTFORWARD	0x48
/* key whose meaning depends on the layout */
#define KBD_SCAN_LAYOUT_KEY	0x31

enum kbd_layout {
	LAYOUT_UNKNOWN,
	LAYOUT_US,
	LAYOUT_UK,
};

static const unsigned short p4note_kbd_keycodes[KBD_NUM_KEYS] = {
	[0x04] = KEY_A,
	[0x05] = KEY_B,
	[0x06] = KEY_C,
	[0x07] = KEY_D,
	[0x08] = KEY_E,
	[0x09] = KEY_F,
	[0x0a] = KEY_G,
	[0x0b] = KEY_H,
	[0x0c] = KEY_I,
	[0x0d] = KEY_J,
	[0x0e] = KEY_K,
	[0x0f] = KEY_L,
	[0x10] = KEY_M,
	[0x11] = KEY_N,
	[0x12] = KEY_O,
	[0x13] = KEY_P,
	[0x14] = KEY_Q,
	[0x15] = KEY_R,
	[0x16] = KEY_S,
	[0x17] = KEY_T,
	[0x18] = KEY_U,
	[0x19] = KEY_V,
	[0x1a] = KEY_W,
	[0x1b] = KEY_X,
	[0x1c] = KEY_Y,
	[0x1d] = KEY_Z,
	[0x1e] = KEY_1,
	[0x1f] = KEY_2,
	[0x20] = KEY_3,
	[0x21] = KEY_4,
	[0x22] = KEY_5,
	[0x23] = KEY_6,
	[0x24] = KEY_7,
	[0x25] = KEY_8,
	[0x26] = KEY_9,
	[0x27] = KEY_0,
	[0x28] = KEY_ENTER,
	[0x29] = KEY_BACK,
	[0x2a] = KEY_BACKSPACE,
	[0x2b] = KEY_TAB,
	[0x2c] = KEY_SPACE,
	[0x2d] = KEY_MINUS,
	[0x2e] = KEY_EQUAL,
	[0x2f] = KEY_LEFTBRACE,
	[0x30] = KEY_RIGHTBRACE,
	[0x31] = KEY_HOME,	/* replaced depending on the layout */
	[0x33] = KEY_SEMICOLON,
	[0x34] = KEY_APOSTROPHE,
	[0x35] = KEY_GRAVE,
	[0x36] = KEY_COMMA,
	[0x37] = KEY_DOT,
	[0x38] = KEY_SLASH,
	[0x39] = KEY_CAPSLOCK,
	[0x3a] = KEY_TIME,
	[0x3b] = KEY_F3,
	[0x3c] = KEY_WWW,
	[0x3d] = KEY_EMAIL,
	[0x3e] = KEY_SCREENLOCK,
	[0x3f] = KEY_BRIGHTNESSDOWN,
	[0x40] = KEY_BRIGHTNESSUP,
	[0x41] = KEY_MUTE,
	[0x42] = KEY_VOLUMEDOWN,
	[0x43] = KEY_VOLUMEUP,
	[0x44] = KEY_PLAY,
	[0x45] = KEY_REWIND,
	[0x46] = KEY_F15,
	[0x48] = KEY_FASTFORWARD,
	[0x49] = KEY_MENU,
	[0x4c] = KEY_DELETE,
	[0x4f] = KEY_RIGHT,
	[0x50] = KEY_LEFT,
	[0x51] = KEY_DOWN,
	[0x52] = KEY_UP,
	[0x53] = KEY_NUMLOCK,
	[0x54] = KEY_KPSLASH,
	[0x55] = KEY_APOSTROPHE,
	[0x56] = KEY_KPMINUS,
	[0x57] = KEY_KPPLUS,
	[0x58] = KEY_KPENTER,
	[0x59] = KEY_KP1,
	[0x5a] = KEY_KP2,
	[0x5b] = KEY_KP3,
	[0x5c] = KEY_KP4,
	[0x5d] = KEY_KP5,
	[0x5e] = KEY_KP6,
	[0x5f] = KEY_KP7,
	[0x60] = KEY_KP8,
	[0x61] = KEY_KP9,
	[0x62] = KEY_KPDOT,
	[0x64] = KEY_BACKSLASH,
	[0x65] = KEY_F22,
	[0x6f] = KEY_HANGEUL,
	[0x70] = KEY_HANJA,
	[0x71] = KEY_LEFTCTRL,
	[0x72] = KEY_LEFTSHIFT,
	[0x73] = KEY_F20,
	[0x74] = KEY_SEARCH,
	[0x75] = KEY_RIGHTALT,
	[0x76] = KEY_RIGHTSHIFT,
	[0x77] = KEY_F21,
	[0x7f] = KEY_F17,
};

struct p4note_kbd {
	struct device *dev;
	struct serdev_device *serdev;

	struct extcon_dev *dock_edev;
	struct notifier_block dock_nb;

	struct work_struct dock_work;
	struct work_struct feedback_work;
	struct work_struct led_work;
	struct delayed_work remap_work;
	struct completion layout_rcvd;

	/* protects everything below */
	struct mutex lock;
	struct input_dev *input;
	bool docked;
	enum kbd_layout layout;
	u8 remap_scan;
	unsigned short keycode[KBD_NUM_KEYS];
	DECLARE_BITMAP(pressed, KBD_NUM_KEYS);
};

static void p4note_kbd_send(struct p4note_kbd *kbd, u8 cmd)
{
	int ret;

	ret = serdev_device_write_buf(kbd->serdev, &cmd, 1);
	if (ret != 1)
		dev_warn(kbd->dev, "failed to send 0x%02x: %d\n", cmd, ret);
}

static void p4note_kbd_release_all(struct p4note_kbd *kbd)
{
	unsigned int i;

	if (!kbd->input)
		return;

	for_each_set_bit(i, kbd->pressed, KBD_NUM_KEYS)
		input_report_key(kbd->input, kbd->keycode[i], 0);
	input_sync(kbd->input);
	bitmap_zero(kbd->pressed, KBD_NUM_KEYS);
}

static int p4note_kbd_input_event(struct input_dev *input, unsigned int type,
				  unsigned int code, int value)
{
	struct p4note_kbd *kbd = input_get_drvdata(input);

	if (type != EV_LED || code != LED_CAPSL)
		return -EINVAL;

	/* called with the input event lock held, send from process context */
	schedule_work(&kbd->led_work);
	return 0;
}

static void p4note_kbd_led_work(struct work_struct *work)
{
	struct p4note_kbd *kbd = container_of(work, struct p4note_kbd,
					      led_work);

	mutex_lock(&kbd->lock);
	if (kbd->input)
		p4note_kbd_send(kbd, test_bit(LED_CAPSL, kbd->input->led) ?
				KBD_CMD_CAPS_ON : KBD_CMD_CAPS_OFF);
	mutex_unlock(&kbd->lock);
}

static int p4note_kbd_connect(struct p4note_kbd *kbd)
{
	struct input_dev *input;
	unsigned int i;
	int ret;

	lockdep_assert_held(&kbd->lock);

	if (kbd->input)
		return 0;

	memcpy(kbd->keycode, p4note_kbd_keycodes, sizeof(kbd->keycode));
	kbd->keycode[KBD_SCAN_LAYOUT_KEY] = kbd->layout == LAYOUT_UK ?
					    KEY_NUMERIC_POUND : KEY_BACKSLASH;
	bitmap_zero(kbd->pressed, KBD_NUM_KEYS);
	kbd->remap_scan = 0;

	input = input_allocate_device();
	if (!input)
		return -ENOMEM;

	/* same name as the vendor driver, keeps old keylayout files working */
	input->name = "sec_keyboard";
	input->phys = "p4note-kbd-dock/input0";
	input->id.bustype = BUS_RS232;
	input->dev.parent = kbd->dev;
	input->event = p4note_kbd_input_event;

	input->keycode = kbd->keycode;
	input->keycodesize = sizeof(kbd->keycode[0]);
	input->keycodemax = ARRAY_SIZE(kbd->keycode);

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_LED, input->evbit);
	__set_bit(LED_CAPSL, input->ledbit);

	for (i = 0; i < KBD_NUM_KEYS; i++)
		if (kbd->keycode[i] != KEY_RESERVED)
			__set_bit(kbd->keycode[i], input->keybit);
	__set_bit(KEY_PREVIOUSSONG, input->keybit);
	__set_bit(KEY_NEXTSONG, input->keybit);
	__clear_bit(KEY_RESERVED, input->keybit);

	input_set_drvdata(input, kbd);

	ret = input_register_device(input);
	if (ret) {
		input_free_device(input);
		return ret;
	}

	kbd->input = input;
	/* the feedback can sleep, and we may be in receive_buf */
	schedule_work(&kbd->feedback_work);

	dev_info(kbd->dev, "%s keyboard dock connected\n",
		 kbd->layout == LAYOUT_UK ? "UK" : "US");

	return 0;
}

/* called with kbd->lock held, drops it to wait for the remap work */
static void p4note_kbd_disconnect(struct p4note_kbd *kbd)
{
	struct input_dev *input = kbd->input;

	lockdep_assert_held(&kbd->lock);

	if (!input)
		return;

	p4note_kbd_release_all(kbd);
	kbd->input = NULL;

	mutex_unlock(&kbd->lock);
	cancel_delayed_work_sync(&kbd->remap_work);
	cancel_work_sync(&kbd->led_work);
	input_unregister_device(input);
	mutex_lock(&kbd->lock);

	dev_info(kbd->dev, "keyboard dock disconnected\n");
}

static void p4note_kbd_remap_work(struct work_struct *work)
{
	struct p4note_kbd *kbd = container_of(to_delayed_work(work),
					      struct p4note_kbd, remap_work);

	mutex_lock(&kbd->lock);
	/* key is still held: report the key itself instead of a track skip */
	if (kbd->input && kbd->remap_scan &&
	    test_bit(kbd->remap_scan, kbd->pressed)) {
		input_report_key(kbd->input, kbd->keycode[kbd->remap_scan], 1);
		input_sync(kbd->input);
	}
	kbd->remap_scan = 0;
	mutex_unlock(&kbd->lock);
}

static void p4note_kbd_report_key(struct p4note_kbd *kbd, u8 scan)
{
	u8 code = scan & KBD_CODE_MASK;
	bool press = !(scan & KBD_RELEASE);
	unsigned int keycode = kbd->keycode[code];

	if (code == KBD_SCAN_REWIND || code == KBD_SCAN_FASTFORWARD) {
		if (press) {
			__set_bit(code, kbd->pressed);
			kbd->remap_scan = code;
			schedule_delayed_work(&kbd->remap_work,
					      msecs_to_jiffies(KBD_REMAP_MS));
			return;
		}

		__clear_bit(code, kbd->pressed);
		if (kbd->remap_scan == code) {
			/* short tap, the remap work has not fired yet */
			cancel_delayed_work(&kbd->remap_work);
			kbd->remap_scan = 0;
			keycode = code == KBD_SCAN_FASTFORWARD ?
				  KEY_NEXTSONG : KEY_PREVIOUSSONG;
			input_report_key(kbd->input, keycode, 1);
			input_sync(kbd->input);
			input_report_key(kbd->input, keycode, 0);
			input_sync(kbd->input);
			return;
		}

		input_report_key(kbd->input, keycode, 0);
		input_sync(kbd->input);
		return;
	}

	if (keycode == KEY_RESERVED) {
		dev_dbg(kbd->dev, "unmapped scancode 0x%02x\n", scan);
		return;
	}

	if (press)
		__set_bit(code, kbd->pressed);
	else
		__clear_bit(code, kbd->pressed);

	input_report_key(kbd->input, keycode, press);
	input_sync(kbd->input);
}

static void p4note_kbd_process_byte(struct p4note_kbd *kbd, u8 byte)
{
	lockdep_assert_held(&kbd->lock);

	if (byte == KBD_LAYOUT_US || byte == KBD_LAYOUT_UK) {
		/*
		 * The layout byte can arrive before the dock notification
		 * reached us, so keep it even if we are not docked yet.
		 */
		if (kbd->layout == LAYOUT_UNKNOWN) {
			kbd->layout = byte == KBD_LAYOUT_UK ? LAYOUT_UK :
							      LAYOUT_US;
			complete(&kbd->layout_rcvd);
		}
		return;
	}

	/* with the dock detached the UART is routed away from the connector */
	if (!kbd->docked)
		return;

	if (byte >= KBD_UNIV_FIRST && byte <= KBD_UNIV_LAST) {
		/*
		 * Universal keyboard dock (USB host/charging) events. The
		 * keyboard expects them to be echoed back. The OTG and charger
		 * handling of the vendor kernel is not implemented.
		 */
		dev_dbg(kbd->dev, "universal dock event 0x%02x\n", byte);
		p4note_kbd_send(kbd, byte);
		return;
	}

	if (!kbd->input) {
		/*
		 * The dock was already powered when we started listening
		 * (e.g. docked at boot), so the layout byte was missed. Any
		 * real key press means a keyboard is there.
		 */
		if (byte == KBD_ALL_RELEASED || (byte & KBD_RELEASE) ||
		    p4note_kbd_keycodes[byte] == KEY_RESERVED)
			return;

		if (kbd->layout == LAYOUT_UNKNOWN) {
			dev_info(kbd->dev, "key press without handshake, assuming US layout\n");
			kbd->layout = LAYOUT_US;
		}
		if (p4note_kbd_connect(kbd))
			return;
	}

	switch (byte) {
	case KBD_ALL_RELEASED:
		p4note_kbd_release_all(kbd);
		break;
	case KBD_CAPS_ON_ECHO:
	case KBD_CAPS_OFF_ECHO:
		break;
	default:
		p4note_kbd_report_key(kbd, byte);
		break;
	}
}

static int p4note_kbd_receive_buf(struct serdev_device *serdev,
				  const unsigned char *buf, size_t count)
{
	struct p4note_kbd *kbd = serdev_device_get_drvdata(serdev);
	size_t i;

	mutex_lock(&kbd->lock);
	for (i = 0; i < count; i++)
		p4note_kbd_process_byte(kbd, buf[i]);
	mutex_unlock(&kbd->lock);

	return count;
}

static const struct serdev_device_ops p4note_kbd_serdev_ops = {
	.receive_buf = p4note_kbd_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

static void p4note_kbd_feedback_work(struct work_struct *work)
{
	struct p4note_kbd *kbd = container_of(work, struct p4note_kbd,
					      feedback_work);

	p4note_extcon_keyboard_feedback(kbd->dock_edev, true);
}

static void p4note_kbd_dock_work(struct work_struct *work)
{
	struct p4note_kbd *kbd = container_of(work, struct p4note_kbd,
					      dock_work);
	bool docked = extcon_get_state(kbd->dock_edev, EXTCON_DOCK) > 0;
	unsigned long timeout = msecs_to_jiffies(KBD_HANDSHAKE_MS);
	bool no_keyboard = false;

	mutex_lock(&kbd->lock);

	if (docked == kbd->docked) {
		mutex_unlock(&kbd->lock);
		return;
	}
	kbd->docked = docked;

	if (!docked) {
		p4note_kbd_disconnect(kbd);
		kbd->layout = LAYOUT_UNKNOWN;
		reinit_completion(&kbd->layout_rcvd);
		mutex_unlock(&kbd->lock);
		return;
	}

	mutex_unlock(&kbd->lock);

	/* extcon-p4note has powered the dock up, wait for the layout byte */
	wait_for_completion_timeout(&kbd->layout_rcvd, timeout);

	mutex_lock(&kbd->lock);
	if (kbd->docked && !kbd->input) {
		if (kbd->layout != LAYOUT_UNKNOWN) {
			int ret = p4note_kbd_connect(kbd);

			if (ret)
				dev_err(kbd->dev,
					"failed to register input device: %d\n",
					ret);
		} else {
			dev_info(kbd->dev, "no keyboard handshake\n");
			no_keyboard = true;
		}
	}
	mutex_unlock(&kbd->lock);

	/* the feedback takes the locks of extcon-p4note, drop ours first */
	if (no_keyboard)
		p4note_extcon_keyboard_feedback(kbd->dock_edev, false);
}

static int p4note_kbd_dock_notifier(struct notifier_block *nb,
				    unsigned long event, void *ptr)
{
	struct p4note_kbd *kbd = container_of(nb, struct p4note_kbd, dock_nb);

	schedule_work(&kbd->dock_work);
	return NOTIFY_DONE;
}

static int p4note_kbd_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct p4note_kbd *kbd;
	int ret;

	kbd = devm_kzalloc(dev, sizeof(*kbd), GFP_KERNEL);
	if (!kbd)
		return -ENOMEM;

	kbd->dev = dev;
	kbd->serdev = serdev;
	mutex_init(&kbd->lock);
	init_completion(&kbd->layout_rcvd);
	INIT_WORK(&kbd->dock_work, p4note_kbd_dock_work);
	INIT_WORK(&kbd->feedback_work, p4note_kbd_feedback_work);
	INIT_WORK(&kbd->led_work, p4note_kbd_led_work);
	INIT_DELAYED_WORK(&kbd->remap_work, p4note_kbd_remap_work);

	kbd->dock_edev = extcon_get_edev_by_phandle(dev, 0);
	if (IS_ERR(kbd->dock_edev))
		return dev_err_probe(dev, PTR_ERR(kbd->dock_edev),
				     "failed to get dock extcon\n");

	serdev_device_set_drvdata(serdev, kbd);
	serdev_device_set_client_ops(serdev, &p4note_kbd_serdev_ops);

	ret = serdev_device_open(serdev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to open serial port\n");

	serdev_device_set_baudrate(serdev, KBD_BAUDRATE);
	serdev_device_set_flow_control(serdev, false);
	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret) {
		dev_err(dev, "failed to set parity: %d\n", ret);
		goto err_close;
	}

	kbd->dock_nb.notifier_call = p4note_kbd_dock_notifier;
	ret = extcon_register_notifier(kbd->dock_edev, EXTCON_DOCK,
				       &kbd->dock_nb);
	if (ret) {
		dev_err(dev, "failed to register dock notifier: %d\n", ret);
		goto err_close;
	}

	/*
	 * A dock attached before we were probed sent its handshake to nobody,
	 * have extcon-p4note power it up again.
	 */
	ret = p4note_extcon_keyboard_ready(kbd->dock_edev);
	if (ret) {
		dev_err(dev, "dock extcon is not extcon-p4note: %d\n", ret);
		goto err_unregister;
	}

	/* pick up a dock that was attached before we were probed */
	schedule_work(&kbd->dock_work);

	return 0;

err_unregister:
	extcon_unregister_notifier(kbd->dock_edev, EXTCON_DOCK, &kbd->dock_nb);
	cancel_work_sync(&kbd->dock_work);
err_close:
	serdev_device_close(serdev);
	return ret;
}

static void p4note_kbd_remove(struct serdev_device *serdev)
{
	struct p4note_kbd *kbd = serdev_device_get_drvdata(serdev);

	extcon_unregister_notifier(kbd->dock_edev, EXTCON_DOCK, &kbd->dock_nb);
	cancel_work_sync(&kbd->dock_work);

	mutex_lock(&kbd->lock);
	/* stop receive_buf from registering the input device again */
	kbd->docked = false;
	p4note_kbd_disconnect(kbd);
	mutex_unlock(&kbd->lock);
	cancel_work_sync(&kbd->feedback_work);

	serdev_device_close(serdev);
}

static int __maybe_unused p4note_kbd_suspend(struct device *dev)
{
	struct p4note_kbd *kbd = dev_get_drvdata(dev);

	mutex_lock(&kbd->lock);
	if (kbd->input) {
		p4note_kbd_release_all(kbd);
		p4note_kbd_send(kbd, KBD_CMD_IDLE);
		serdev_device_wait_until_sent(kbd->serdev,
					      msecs_to_jiffies(20));
	}
	mutex_unlock(&kbd->lock);

	return 0;
}

static SIMPLE_DEV_PM_OPS(p4note_kbd_pm_ops, p4note_kbd_suspend, NULL);

static const struct of_device_id p4note_kbd_of_match[] = {
	{ .compatible = "samsung,p4note-keyboard-dock" },
	{ }
};
MODULE_DEVICE_TABLE(of, p4note_kbd_of_match);

static struct serdev_device_driver p4note_kbd_driver = {
	.probe = p4note_kbd_probe,
	.remove = p4note_kbd_remove,
	.driver = {
		.name = "p4note-kbd-dock",
		.of_match_table = p4note_kbd_of_match,
		.pm = &p4note_kbd_pm_ops,
	},
};
module_serdev_device_driver(p4note_kbd_driver);

MODULE_DESCRIPTION("Samsung p4note keyboard dock driver");
MODULE_LICENSE("GPL");
