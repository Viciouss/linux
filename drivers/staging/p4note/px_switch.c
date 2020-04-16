#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/semaphore.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <usb_switch.h>

struct px_switch_config {
	struct gpio_desc *usb_select0;
	struct gpio_desc *usb_select1;
	struct gpio_desc *usb_select_cp;
	struct gpio_desc *uart_select;
	struct gpio_desc *uart_select2;

	struct semaphore usb_switch_sem;
	bool usb_connected;
	enum usb_path_t current_path;
	struct device *sec_switch_dev;
};

static struct px_switch_config *px_switch_config;

static ssize_t show_usb_sel(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct px_switch_config *state = platform_get_drvdata(pdev);

	 char *mode;

	if (state->current_path & USB_PATH_CP) {
		/* CP */
		mode = "MODEM";
	} else {
		/* AP */
		mode = "PDA";
	}

	pr_info("%s: %s\n", __func__, mode);

	return sprintf(buf, "%s\n", mode);
}

static ssize_t store_usb_sel(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	pr_info("%s: %s\n", __func__, buf);

	if (!strncasecmp(buf, "PDA", 3)) {
		usb_switch_lock();
		usb_switch_clr_path(USB_PATH_CP);
		usb_switch_unlock();
	} else if (!strncasecmp(buf, "MODEM", 5)) {
		usb_switch_lock();
		usb_switch_set_path(USB_PATH_CP);
		usb_switch_unlock();
	} else {
		pr_err("%s: wrong usb_sel value(%s)!!\n", __func__, buf);
		return -EINVAL;
	}

	return count;
}

static ssize_t show_uart_sel(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct px_switch_config *state = platform_get_drvdata(pdev);

	int val_sel;
	int val_sel2;
	const char *mode;

	val_sel = gpiod_get_value(state->uart_select);
	if(state->uart_select2) {
		val_sel2 = gpiod_get_value(state->uart_select2);
	}

	if (val_sel == 0) {
		/* CP */
		mode = "CP";
	} else {
		if(state->uart_select2 && val_sel2 != 0) {
			mode = "DOCK";
		} else {
			mode = "AP";
		}
	}

	pr_info("%s: %s\n", __func__, mode);

	return sprintf(buf, "%s\n", mode);
}

static ssize_t store_uart_sel(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct px_switch_config *state = platform_get_drvdata(pdev);

	int uart_sel = -1;
	int uart_sel2 = -1;

	pr_info("%s: %s\n", __func__, buf);

	uart_sel = gpiod_get_value(state->uart_select);
	if(state->uart_select2) {
		uart_sel2 = gpio_get_value(state->uart_select2);
	}

	if (!strncasecmp(buf, "AP", 2)) {
		uart_sel = 1;
		if(state->uart_select2) {
			uart_sel2 = 0;
		}
	} else if (!strncasecmp(buf, "CP", 2)) {
		uart_sel = 0;
	} else if (state->uart_select2 && !strncasecmp(buf, "DOCK", 4)) {
		uart_sel = 1;
		uart_sel2 = 1;
	} else {
		pr_err("%s: wrong uart_sel value(%s)!!\n", __func__, buf);
		return -EINVAL;
	}

	/* 1 for AP, 0 for CP */
	gpiod_set_value(state->uart_select, uart_sel);
	pr_info("%s: uart_sel(%d)\n", __func__, uart_sel);

	if(state->uart_select2) {
		/* 1 for (AP)DOCK, 0 for (AP)FAC */
		gpio_set_value(state->uart_select2, uart_sel2);
		pr_info("%s: uart_sel2(%d)\n", __func__, uart_sel2);
	}

	return count;
}

static ssize_t show_usb_state(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct px_switch_config *state = platform_get_drvdata(pdev);

	const char *state;

	if (state->usb_connected)
		state = "USB_STATE_CONFIGURED";
	else
		state = "USB_STATE_NOTCONFIGURED";

	pr_info("%s: %s\n", __func__, state);

	return sprintf(buf, "%s\n", state);
}

static DEVICE_ATTR(usb_sel, 0664, show_usb_sel, store_usb_sel);
static DEVICE_ATTR(uart_sel, 0664, show_uart_sel, store_uart_sel);
static DEVICE_ATTR(usb_state, S_IRUGO, show_usb_state, NULL);

static struct attribute *px_switch_attributes[] = {
	&dev_attr_usb_sel.attr,
	&dev_attr_uart_sel.attr,
	&dev_attr_usb_state.attr,
	NULL
};

static const struct attribute_group px_switch_group = {
	.attrs = px_switch_attributes,
};

void set_usb_connection_state(bool connected)
{
	
	pr_info("%s: set %s\n", __func__, (connected ? "True" : "False"));

	if (px_switch_config->usb_connected != connected) {
		px_switch_config->usb_connected = connected;

		pr_info("%s: send \"usb_state\" sysfs_notify\n", __func__);
		sysfs_notify(&px_switch_config->sec_switch_dev->kobj, NULL, "usb_state");
	}
}

static void pmic_safeout2(int onoff)
{
	struct gpio_desc *cp = px_switch_config->usb_select_cp;
	if (onoff) {
		if (!gpiod_get_value(cp)) {
			gpiod_set_value(cp, onoff);
		} else {
			pr_info("%s: onoff:%d No change in safeout2\n",
				   __func__, onoff);
		}
	} else {
		if (gpiod_get_value(cp)) {
			gpiod_set_value(cp, onoff);
		} else {
			pr_info("%s: onoff:%d No change in safeout2\n",
				   __func__, onoff);
		}
	}
}

static void usb_apply_path(enum usb_path_t path)
{
	struct gpio_desc *sel0 = px_switch_config->usb_select0;
	struct gpio_desc *sel1 = px_switch_config->usb_select1;
	struct gpio_desc *cp = px_switch_config->usb_select_cp;
	
	pr_info("%s: current gpio before changing : sel0:%d sel1:%d sel_cp:%d\n",
		   __func__, gpiod_get_value(sel0),
		   gpio_get_value(sel1), cp ? gpio_get_value(cp) : "-");
	pr_info("%s: target path %x\n", __func__, path);

	/* following checks are ordered according to priority */
	if (path & USB_PATH_ADCCHECK) { // charger type detection via stmpe channel 6
		gpiod_set_value(sel0, 1);
		gpiod_set_value(sel1, 0);
		goto out_nochange;
	}

	if (path & USB_PATH_TA) {
		gpiod_set_value(sel0, 0);
		gpiod_set_value(sel1, 0);
		goto out_nochange;
	}

	if (path & USB_PATH_CP) {
		pr_info("DEBUG: set USB path to CP\n");
		gpiod_set_value(sel0, 0);
		gpiod_set_value(sel1, 1);
		mdelay(3);
		goto out_cp;
	}

	if (path & USB_PATH_AP) {
		gpiod_set_value(sel0, 1);
		gpiod_set_value(sel1, 1);
		goto out_ap;
	}

	/* default */
	gpiod_set_value(sel0, 1);
	gpiod_set_value(sel1, 1);

out_ap:
	pr_info("%s: %x safeout2 off\n", __func__, path);
	pmic_safeout2(0);
	goto sysfs_noti;

out_cp:
	pr_info("%s: %x safeout2 on\n", __func__, path);
	pmic_safeout2(1);
	goto sysfs_noti;

out_nochange:
	pr_info("%s: %x safeout2 no change\n", __func__, path);
	return;

sysfs_noti:
	pr_info("%s: send \"usb_sel\" sysfs_notify\n", __func__);
	sysfs_notify(&px_switch_config->sec_switch_dev->kobj, NULL, "usb_sel");
	return;
}

/*
  Typical usage of usb switch:

  usb_switch_lock();  (alternatively from hard/soft irq context)
  ( or usb_switch_trylock() )
  ...
  usb_switch_set_path(USB_PATH_ADCCHECK);
  ...
  usb_switch_set_path(USB_PATH_TA);
  ...
  usb_switch_unlock(); (this restores previous usb switch settings)
*/
enum usb_path_t usb_switch_get_path(void)
{
	pr_info("%s: current path(%d)\n", __func__, px_switch_config->current_path);

	return px_switch_config->current_path;
}

void usb_switch_set_path(enum usb_path_t path)
{
	pr_info("%s: %x current_path before changing\n",
		__func__, px_switch_config->current_path);

	px_switch_config->current_path |= path;
	usb_apply_path(px_switch_config->current_path);
}

void usb_switch_clr_path(enum usb_path_t path)
{
	pr_info("%s: %x current_path before changing\n",
		__func__, px_switch_config->current_path);

	px_switch_config->current_path &= ~path;
	usb_apply_path(px_switch_config->current_path);
}

int usb_switch_lock(void)
{
	return down_interruptible(&px_switch_config->usb_switch_sem);
}

int usb_switch_trylock(void)
{
	return down_trylock(&px_switch_config->usb_switch_sem);
}

void usb_switch_unlock(void)
{
	up(&px_switch_config->usb_switch_sem);
}

static int px_switch_probe(struct platform_device *pdev)
{
	struct px_switch_config *data;
	struct gpio_desc *gpiod;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(struct px_switch_config), GFP_KERNEL);

	if(!data)
		return -ENOMEM;

	data->usb_select0 = devm_gpiod_get(&pdev->dev, "usb0", GPIOD_OUT_HIGH);
	if (IS_ERR(data->usb_select0)) {
		dev_err(&pdev->dev, "failed to get select0 GPIO\n");
		return PTR_ERR(data->usb_select0);
	}

	data->usb_select1 = devm_gpiod_get(&pdev->dev, "usb1", GPIOD_OUT_HIGH);
	if (IS_ERR(data->usb_select1)) {
		dev_err(&pdev->dev, "failed to get select0 GPIO\n");
		return PTR_ERR(data->usb_select1);
	}

	data->usb_select_cp = devm_gpiod_get_optional(&pdev->dev, "cp", GPIOD_OUT_HIGH);
	if (IS_ERR(data->usb_select_cp)) {
		dev_err(&pdev->dev, "failed to get select_cp GPIO\n");
		return PTR_ERR(data->usb_select_cp);
	}

	data->uart_select = devm_gpiod_get(&pdev->dev, "uart1", GPIOD_OUT_HIGH);
	if (IS_ERR(data->uart_select)) {
		dev_err(&pdev->dev, "failed to get select0 GPIO\n");
		return PTR_ERR(data->uart_select);
	}

	data->uart_select2 = devm_gpiod_get_optional(&pdev->dev, "uart2", GPIOD_OUT_HIGH);
	if (IS_ERR(data->uart_select2)) {
		dev_err(&pdev->dev, "failed to get select0 GPIO\n");
		return PTR_ERR(data->uart_select2);
	}

	ret = gpiod_export(data->usb_select0, 1);
	if (ret)
		return ret;

	ret = gpiod_export(data->usb_select1, 1);
	if (ret)
		return ret;

	if(data->uart_select2) {
		ret = gpiod_export(data->usb_select_cp, 1);
		if (ret)
			return ret;
	}

	ret = gpiod_export(data->uart_select, 1);
	if (ret)
		return ret;

	if(data->uart_select2) {
		ret = gpiod_export(data->uart_select2, 1);
		if (ret)
			return ret;
	}

	BUG_ON(!sec_class);
	data->sec_switch_dev = device_create(sec_class, NULL, 0, NULL, "switch");
	BUG_ON(!data->sec_switch_dev);

	ret = gpiod_export_link(data->sec_switch_dev, "GPIO_USB_SEL0", data->usb_select0);
	if (ret)
		return ret;

	ret = gpiod_export_link(data->sec_switch_dev, "GPIO_USB_SEL1", data->usb_select1);
	if (ret)
		return ret;

	if(data->usb_select_cp) {
		ret = gpiod_export_link(data->sec_switch_dev, "GPIO_USB_SEL_CP", data->usb_select_cp);
		if (ret)
			return ret;
	}

	ret = gpiod_export_link(data->sec_switch_dev, "GPIO_UART_SEL", data->uart_select);
	if (ret)
		return ret;

	if(data->uart_select2) {
		ret = gpiod_export_link(data->sec_switch_dev, "GPIO_UART_SEL2", data->uart_select2);
		if (ret)
			return ret;
	}

	/*init_MUTEX(&usb_switch_sem);*/
	sema_init(&data->usb_switch_sem, 1);

	if ((!gpiod_get_value(data->usb_select0)) && (gpio_get_value(data->usb_select1))) {
		usb_switch_lock();
		usb_switch_set_path(USB_PATH_CP);
		usb_switch_unlock();
	}

	/* create sysfs group */
	ret = sysfs_create_group(&data->sec_switch_dev->kobj, &px_switch_group);
	if (ret) {
		pr_err("failed to create px switch attribute group\n");
		return ret;
	}

	px_switch_config = data;

	return ret;
}

static const struct of_device_id px_switch_of_match[] = {
	{ .compatible = "p4note,px-switch", },
	{ },
};
MODULE_DEVICE_TABLE(of, px_switch_of_match);

static struct platform_driver px_switch_driver = {
	.probe          = px_switch_probe,
	.driver         = {
		.name   = "px-switch",
		.of_match_table = px_switch_of_match,
	},
};

module_platform_driver(px_switch_driver);

MODULE_DESCRIPTION("Samsung Galaxy Note 10.1 USB switch driver");
MODULE_LICENSE("GPL");