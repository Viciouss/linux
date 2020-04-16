

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/pm_wakeup.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>

#include <extcon/extcon.h>
#include <linux/power_supply.h>

#include <asm/irq.h>

#include "30pin_con.h"

#ifdef CONFIG_REGULATOR
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#endif

#ifdef CONFIG_DRM_SII9234
#include "sii9234.h"
#endif

#define SUBJECT "ACCESSORY"

#define ACC_CONDEV_DBG(format, ...) \
	pr_info("[ "SUBJECT " (%s,%d) ] " format "\n", \
		__func__, __LINE__, ## __VA_ARGS__);

#define DETECTION_INTR_DELAY	(get_jiffies_64() + (HZ*15)) /* 20s */

enum accessory_type {
	ACCESSORY_NONE = 0,
	ACCESSORY_OTG,
	ACCESSORY_LINEOUT,
	ACCESSORY_CARMOUNT,
	ACCESSORY_UNKNOWN,
};

enum dock_type {
	DOCK_NONE = 0,
	DOCK_DESK,
	DOCK_KEYBOARD,
};

enum uevent_dock_type {
	UEVENT_DOCK_NONE = 0,
	UEVENT_DOCK_DESK,
	UEVENT_DOCK_CAR,
	UEVENT_DOCK_KEYBOARD = 9,
};

struct acc_con_info {
	struct device *acc_dev;
	struct acc_con_platform_data *pdata;
	struct delayed_work acc_dwork;
	struct delayed_work acc_id_dwork;
	struct extcon_dev dock_switch;
	struct extcon_dev ear_jack_switch;
	struct sec_30pin_callbacks callbacks;
	enum accessory_type current_accessory;
	enum accessory_type univ_kdb_accessory;
	enum dock_type current_dock;
	int accessory_irq;
	int dock_irq;
	int cable_type;
	int cable_sub_type;
	int cable_pwr_type;
#if defined(CONFIG_DRM_SII9234)
	int mhl_irq;
	bool mhl_pwr_state;
#endif
	struct delayed_work acc_con_work;
	struct mutex lock;
};

#if defined(CONFIG_STMPE811_ADC)
#if defined(CONFIG_MACH_P4NOTE) || defined(CONFIG_MACH_KONA)
#define ACCESSORY_ID_ADC_CH 7
#else
#define ACCESSORY_ID_ADC_CH 0
#endif
#else
#define ACCESSORY_ID_ADC_CH 4
#endif

#ifdef CONFIG_SAMSUNG_MHL_9290
static BLOCKING_NOTIFIER_HEAD(acc_notifier);

int acc_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&acc_notifier, nb);
}

int acc_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&acc_notifier, nb);
}

static int acc_notify(int event)
{
	pr_info("notifier: mhl callback\n");
	return blocking_notifier_call_chain(&acc_notifier, event, NULL);
}
#endif
static int acc_get_adc_accessroy_id()
{
	int adc_data = stmpe811_get_adc_data(ACCESSORY_ID_ADC_CH);
/*	ACC_CONDEV_DBG("[ACC] adc_data = %d..\n",adc_data); */
	return adc_data;
}

static int acc_get_accessory_id(struct acc_con_info *acc)
{
	int i;
	u32 adc = 0, adc_sum = 0;
	u32 adc_buff[5] = {0};
	u32 adc_val = 0;
	u32 adc_min = 0;
	u32 adc_max = 0;

	if (!acc) {
		pr_err("adc client is not registered!\n");
		return -1;
	}

	for (i = 0; i < 5; i++) {
		adc_val = acc_get_adc_accessroy_id();
		ACC_CONDEV_DBG("ACCESSORY_ID adc_val[%d] value = %d",
			i, adc_val);
		adc_buff[i] = adc_val;
		adc_sum += adc_buff[i];
		if (i == 0) {
			adc_min = adc_buff[0];
			adc_max = adc_buff[0];
		} else {
			if (adc_max < adc_buff[i])
				adc_max = adc_buff[i];
			else if (adc_min > adc_buff[i])
				adc_min = adc_buff[i];
		}
		msleep(20);
	}
	/* adc = (adc_sum - adc_max - adc_min)/3; */
	adc = adc_buff[4];
	ACC_CONDEV_DBG("ACCESSORY_ID ADC value = %d", adc);
	return (int)adc;
}

static ssize_t acc_read_acc_id(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct acc_con_info *acc  = dev_get_drvdata(dev);
	int adc_val = 0 ;
	int jig_uart_off = 0 ;
	int count = 0 ;
	adc_val = acc_get_accessory_id(acc);

	if ((3540 < adc_val) && (adc_val < 3920))
		jig_uart_off = 28 ;
	else
		jig_uart_off = 0 ;

	ACC_CONDEV_DBG("jig_uart_off : %d", jig_uart_off);

	count = sprintf(buf, "%x\n", jig_uart_off);

	return count;
}

static DEVICE_ATTR(adc, S_IRUGO, acc_read_acc_id, NULL);


void acc_accessory_uevent(struct acc_con_info *acc, int acc_adc)
{
	char *env_ptr;
	char *stat_ptr;
	char *envp[3];

	/* value is changed for PxLTE
	3 pole earjack  1.00 V ( 0.90~1.10V)   adc: 797~1002
	Car mount        1.38 V (1.24~1.45V)   adc: 1134~1352
	4 pole earjack   just bundles is supported . adc :1360~1449
	OTG                 2.2 V  (2.00~2.35V)    adc: 1903~2248 */

	if (acc_adc != false) {
		if ((1100 < acc_adc) && (1400 > acc_adc)) {
			/* 3 pole earjack 1220 */
			env_ptr = "ACCESSORY=lineout";
			acc->current_accessory = ACCESSORY_LINEOUT;
			switch_set_state(&acc->ear_jack_switch, 1);
#if 0
		} else if ((1134 < acc_adc) && (1352 > acc_adc)) {
			/* car mount */
			env_ptr = "ACCESSORY=carmount";
			acc->current_accessory = ACCESSORY_CARMOUNT;
#endif
		} else if ((1800 < acc_adc) && (2350 > acc_adc)) {
			/* 4 pole earjack, No warranty 2000 */
			env_ptr = "ACCESSORY=lineout";
			acc->current_accessory = ACCESSORY_LINEOUT;
			switch_set_state(&acc->ear_jack_switch, 1);
		} else if ((2450 < acc_adc) && (2850 > acc_adc)) {
			/* otg 2730 */
			env_ptr = "ACCESSORY=OTG";
			acc->current_accessory = ACCESSORY_OTG;
		} else {
			env_ptr = "ACCESSORY=unknown";
			acc->current_accessory = ACCESSORY_UNKNOWN;
		}

		stat_ptr = "STATE=online";
		envp[0] = env_ptr;
		envp[1] = stat_ptr;
		envp[2] = NULL;
		if (acc->current_accessory == ACCESSORY_OTG) {
			if (acc->pdata->usb_ldo_en)
				acc->pdata->usb_ldo_en(1);
			if (acc->pdata->otg_en)
				acc->pdata->otg_en(1);
		}
		kobject_uevent_env(&acc->acc_dev->kobj, KOBJ_CHANGE, envp);
		ACC_CONDEV_DBG("%s : %s", env_ptr, stat_ptr);
	} else {
		if (acc->current_accessory == ACCESSORY_OTG)
			env_ptr = "ACCESSORY=OTG";
		else if (acc->current_accessory == ACCESSORY_LINEOUT) {
			env_ptr = "ACCESSORY=lineout";
			switch_set_state(&acc->ear_jack_switch,
				UEVENT_DOCK_NONE);
		} else if (acc->current_accessory == ACCESSORY_CARMOUNT)
			env_ptr = "ACCESSORY=carmount";
		else
			env_ptr = "ACCESSORY=unknown";

		stat_ptr = "STATE=offline";
		envp[0] = env_ptr;
		envp[1] = stat_ptr;
		envp[2] = NULL;
		kobject_uevent_env(&acc->acc_dev->kobj, KOBJ_CHANGE, envp);
		if ((acc->current_accessory == ACCESSORY_OTG) &&
			acc->pdata->otg_en)
			acc->pdata->otg_en(0);

		acc->current_accessory = ACCESSORY_NONE;
		ACC_CONDEV_DBG("%s : %s", env_ptr, stat_ptr);
	}
}

static void acc_dock_uevent(struct acc_con_info *acc, bool connected)
{
	char *env_ptr;
	char *stat_ptr;
	char *envp[3];

	if (acc->current_dock == DOCK_KEYBOARD)
		env_ptr = "DOCK=keyboard";
	else if (acc->current_dock == DOCK_DESK)
		env_ptr = "DOCK=desk";
	else
		env_ptr = "DOCK=unknown";

	if (!connected) {
		stat_ptr = "STATE=offline";
		acc->current_dock = DOCK_NONE;
	} else {
		stat_ptr = "STATE=online";
	}

	envp[0] = env_ptr;
	envp[1] = stat_ptr;
	envp[2] = NULL;
	kobject_uevent_env(&acc->acc_dev->kobj, KOBJ_CHANGE, envp);
	ACC_CONDEV_DBG("%s : %s", env_ptr, stat_ptr);
}

/* power supply name for set state */
#define PSY_NAME	"battery"
static void acc_dock_psy(struct acc_con_info *acc)
{
	struct power_supply *psy = power_supply_get_by_name(PSY_NAME);
	union power_supply_propval value;

/* only support p4note(high current charging) */
#if !defined(CONFIG_MACH_P4NOTE) && !defined(CONFIG_MACH_KONA)
	return;
#endif

	if (!psy || !psy->set_property) {
		pr_err("%s: fail to get %s psy\n", __func__, PSY_NAME);
		return;
	}

	value.intval = 0;
	value.intval = (acc->cable_type << 16) + (acc->cable_sub_type << 8) +
			(acc->cable_pwr_type << 0);
	pr_info("[BATT]30 cx(%d), sub(%d), pwr(%d)\n",
		acc->cable_type, acc->cable_sub_type, acc->cable_pwr_type);

	psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
}

void acc_otg_enable_by_univkbd(struct acc_con_info *acc, bool val)
{
	char *env_ptr;
	char *stat_ptr;
	char *envp[3];

	if (val == true) {
		if (acc->univ_kdb_accessory == ACCESSORY_NONE) {
			env_ptr = "ACCESSORY=OTG";
			stat_ptr = "STATE=online";
			acc->univ_kdb_accessory = ACCESSORY_OTG;

			if (acc->pdata->usb_ldo_en)
				acc->pdata->usb_ldo_en(1);
			if (acc->pdata->otg_en)
				acc->pdata->otg_en(1);

			envp[0] = env_ptr;
			envp[1] = stat_ptr;
			envp[2] = NULL;
			kobject_uevent_env(&acc->acc_dev->kobj,
					KOBJ_CHANGE, envp);
			ACC_CONDEV_DBG("%s : %s", env_ptr, stat_ptr);
		}
	} else {
		if (acc->univ_kdb_accessory == ACCESSORY_OTG) {
			env_ptr = "ACCESSORY=OTG";
			stat_ptr = "STATE=offline";

			envp[0] = env_ptr;
			envp[1] = stat_ptr;
			envp[2] = NULL;
			kobject_uevent_env(&acc->acc_dev->kobj,
					KOBJ_CHANGE, envp);
			ACC_CONDEV_DBG("%s : %s", env_ptr, stat_ptr);

			if (acc->pdata->otg_en)
				acc->pdata->otg_en(0);

			acc->univ_kdb_accessory = ACCESSORY_NONE;
		}
	}
}

static void acc_check_dock_detection(struct acc_con_info *acc)
{
	if (NULL == acc->pdata->get_dock_state) {
		ACC_CONDEV_DBG("[30PIN] failed to get acc state!!!");
		return;
	}
	if (!acc->pdata->get_dock_state()) {

		if (acc->pdata->check_keyboard &&
			acc->pdata->check_keyboard(true)) {
			
			if (DOCK_KEYBOARD == acc->current_dock) {
				switch_set_state(&acc->dock_switch,
					UEVENT_DOCK_NONE);
				acc_dock_uevent(acc, false);
			}

			acc->current_dock = DOCK_KEYBOARD;
			ACC_CONDEV_DBG
			("The dock proves to be a keyboard dock..!");
			switch_set_state(&acc->dock_switch,
				UEVENT_DOCK_KEYBOARD);
			acc->cable_type = POWER_SUPPLY_TYPE_DOCK;
			acc->cable_sub_type = ONLINE_SUB_TYPE_DESK;
		} else {
			ACC_CONDEV_DBG
			("The dock proves to be a desktop dock..!");
			switch_set_state(&acc->dock_switch, UEVENT_DOCK_DESK);
			acc->current_dock = DOCK_DESK;
			acc->cable_type = POWER_SUPPLY_TYPE_DOCK;
			acc->cable_sub_type = ONLINE_SUB_TYPE_DESK;

			mutex_lock(&acc->lock);
			if (!acc->mhl_pwr_state) {

				acc_notify(1);

				acc->mhl_pwr_state = true;
			}
			mutex_unlock(&acc->lock);

		}
		acc_dock_uevent(acc, true);
	} else {

		ACC_CONDEV_DBG("dock station detached.. !");

		switch_set_state(&acc->dock_switch, UEVENT_DOCK_NONE);
		acc->current_dock = DOCK_NONE;
		acc->cable_type = POWER_SUPPLY_TYPE_BATTERY;
		acc->cable_sub_type = ONLINE_SUB_TYPE_UNKNOWN;

		if (acc->pdata->check_keyboard)
			acc->pdata->check_keyboard(false);
		if (acc->univ_kdb_accessory == ACCESSORY_OTG) {
			acc_otg_enable_by_univkbd(acc, false);
		}

#if defined(CONFIG_DRM_SII9234)
		/*call MHL deinit */
		if (acc->mhl_pwr_state) {
			acc_notify(0);
			acc->mhl_pwr_state = false;
		}
#endif
		acc_dock_uevent(acc, false);

	}
	acc_dock_psy(acc);
}

static irqreturn_t acc_dock_isr(int irq, void *ptr)
{
	struct acc_con_info *acc = ptr;
	pm_stay_awake(acc->acc_dev);
	ACC_CONDEV_DBG
		("A dock station attached or detached..");
	acc_check_dock_detection(acc);
	pm_relax(acc->acc_dev);
	return IRQ_HANDLED;
}

#define DETECTION_DELAY_MS	200

static irqreturn_t acc_accessory_isr(int irq, void *dev_id)
{
	struct acc_con_info *acc = (struct acc_con_info *)dev_id;
	ACC_CONDEV_DBG("");
	cancel_delayed_work_sync(&acc->acc_id_dwork);
	schedule_delayed_work(&acc->acc_id_dwork,
			msecs_to_jiffies(DETECTION_DELAY_MS));
	return IRQ_HANDLED;
}

static int acc_init_dock_int(struct acc_con_info *acc)
{
	int ret = 0;
	acc->accessory_irq = gpio_to_irq(acc->pdata->accessory_irq_gpio);
	ret = request_threaded_irq(acc->accessory_irq, NULL, acc_dock_isr,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
			| IRQF_NO_SUSPEND,
			"accessory_detect", acc);
	if (ret)
		ACC_CONDEV_DBG("request_irq(accessory_irq) return : %d\n", ret);

	ret = enable_irq_wake(acc->accessory_irq);
	if (ret)
		ACC_CONDEV_DBG("enable_irq_wake(accessory_irq) return : %d\n",
			ret);

	return ret;
}

static int acc_init_accessory_int(struct acc_con_info *acc)
{
	int ret = 0;
	acc->dock_irq = gpio_to_irq(acc->pdata->dock_irq_gpio);
	ret = request_threaded_irq(acc->dock_irq, NULL, acc_accessory_isr,
			IRQF_ONESHOT |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"dock_detect", acc);
	if (ret)
		ACC_CONDEV_DBG("request_irq(dock_irq) return : %d\n", ret);

	ret = enable_irq_wake(acc->dock_irq);
	if (ret)
		ACC_CONDEV_DBG("enable_irq_wake(dock_irq) return : %d\n", ret);

	return ret;
}

static void acc_dwork_int_init(struct work_struct *work)
{
	struct acc_con_info *acc = container_of(work,
		struct acc_con_info, acc_dwork.work);
	int retval;

	ACC_CONDEV_DBG("");

	retval = acc_init_dock_int(acc);
	if (retval) {
		ACC_CONDEV_DBG("failed to initialize dock_int irq");
		goto err_irq_dock;
	}

	retval = acc_init_accessory_int(acc);
	if (retval) {
		ACC_CONDEV_DBG(" failed to initialize accessory_int irq");
		goto err_irq_acc;
	}

	if (acc->pdata->get_dock_state) {
		if (!acc->pdata->get_dock_state())
			acc_check_dock_detection(acc);
	}

	if (acc->pdata->get_acc_state) {
		if (!acc->pdata->get_acc_state())
			schedule_delayed_work(&acc->acc_id_dwork,
				msecs_to_jiffies(DETECTION_DELAY_MS));
	}

	return ;

err_irq_acc:
	free_irq(acc->accessory_irq, acc);
err_irq_dock:
	switch_dev_unregister(&acc->ear_jack_switch);
	return ;
}

static int acc_noti_univkbd_dock(struct sec_30pin_callbacks *cb,
	unsigned int code)
{
	struct acc_con_info *acc =
		container_of(cb, struct acc_con_info, callbacks);

	char *env_ptr;
	char *stat_ptr;
	char *envp[3];

	ACC_CONDEV_DBG("universal keyboard noti. callback 0x%x", code);

	switch (code) {
	case 0x68: /*dock is con*/
		acc_otg_enable_by_univkbd(acc, true);
		acc->cable_type = POWER_SUPPLY_TYPE_DOCK;
		acc->cable_sub_type = ONLINE_SUB_TYPE_KBD;
		acc_dock_psy(acc);
		break;
	case 0x69: /*usb charging*/
		acc->cable_pwr_type = ONLINE_POWER_TYPE_USB;
		acc_dock_psy(acc);
		break;
	case 0x6a: /*USB cable attached */
		acc_otg_enable_by_univkbd(acc, false);
		acc->cable_pwr_type = ONLINE_POWER_TYPE_USB;
		acc_dock_psy(acc);
		break;
	case 0x6b: /*TA connection*/
		acc->cable_pwr_type = ONLINE_POWER_TYPE_TA;
		acc_dock_psy(acc);
		break;
	case 0x6c: /* USB cable detached */
		acc_otg_enable_by_univkbd(acc, true);
		acc->cable_pwr_type = ONLINE_POWER_TYPE_BATTERY;
		acc_dock_psy(acc);
		break;
	}

	return 0;
}

static void acc_dwork_accessory_detect(struct work_struct *work)
{
	struct acc_con_info *acc = container_of(work,
			struct acc_con_info, acc_id_dwork.work);

	int adc_val = 0;
	int acc_state = 0;
	int acc_state2 = 0;

	acc_state = acc->pdata->get_acc_state();

	if (acc_state) {
		ACC_CONDEV_DBG("Accessory detached");
		acc_accessory_uevent(acc, false);
	} else {
		adc_val = acc_get_accessory_id(acc);

		acc_state2 = acc->pdata->get_acc_state();
		if (acc_state2) {
			ACC_CONDEV_DBG("Accessory detached2.");
			acc_accessory_uevent(acc, false);
		} else {
		ACC_CONDEV_DBG("Accessory attached");
		acc_accessory_uevent(acc, adc_val);
		}
	}
}

static int acc_con_probe(struct platform_device *pdev)
{
	struct acc_con_info *acc;
	struct acc_con_platform_data *pdata;

	int	retval;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	acc = devm_kzalloc(&pdev->dev, sizeof(*acc), GFP_KERNEL);
	if (!acc)
		return -ENOMEM;

	acc->accessory_irq = of_get_named_gpio(&pdev->dev.of_node, "accessory-irq", 0);
	acc->acc_en = of_get_named_gpio(&pdev->dev.of_node, "accessory-en", 0);
	acc->acc_5v = of_get_named_gpio(&pdev->dev.of_node, "accessory-5v", 0);
	acc->dock_irq = of_get_named_gpio(&pdev->dev.of_node, "dock-irq", 0);

	acc->pdata = pdata;
	acc->current_dock = DOCK_NONE;
	acc->current_accessory = ACCESSORY_NONE;
	acc->univ_kdb_accessory = ACCESSORY_NONE;
#if defined(CONFIG_DRM_SII9234)
	acc->mhl_irq = gpio_to_irq(pdata->mhl_irq_gpio);
	acc->mhl_pwr_state = false;
#endif

	mutex_init(&acc->lock);
	dev_set_drvdata(&pdev->dev, acc);

	acc->acc_dev = &pdev->dev;

	acc->callbacks.noti_univ_kdb_dock = acc_noti_univkbd_dock;
	if (pdata->register_cb)
		pdata->register_cb(&acc->callbacks);

	acc->dock_switch.name = "dock";
	retval = switch_dev_register(&acc->dock_switch);
	if (retval < 0)
		goto err_sw_dock;

	acc->ear_jack_switch.name = "usb_audio";
	retval = switch_dev_register(&acc->ear_jack_switch);
	if (retval < 0)
		goto err_sw_jack;

	device_init_wakeup(&pdev->dev, true);

	INIT_DELAYED_WORK(&acc->acc_dwork, acc_dwork_int_init);
	schedule_delayed_work(&acc->acc_dwork, msecs_to_jiffies(10000));
	INIT_DELAYED_WORK(&acc->acc_id_dwork, acc_dwork_accessory_detect);

	if (device_create_file(sec_switch_dev, &dev_attr_adc) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_adc  .attr.name);
	dev_set_drvdata(sec_switch_dev, acc);

	return 0;

err_sw_jack:
	switch_dev_unregister(&acc->dock_switch);
err_sw_dock:

	kfree(acc);

	return retval;
}

static int acc_con_remove(struct platform_device *pdev)
{
	struct acc_con_info *acc = platform_get_drvdata(pdev);
	ACC_CONDEV_DBG("");

	free_irq(acc->accessory_irq, acc);
	free_irq(acc->dock_irq, acc);

	switch_dev_unregister(&acc->dock_switch);
	switch_dev_unregister(&acc->ear_jack_switch);
	kfree(acc);
	return 0;
}

static int acc_con_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct acc_con_info *acc = platform_get_drvdata(pdev);
	ACC_CONDEV_DBG("");
#if defined(CONFIG_DRM_SII9234)
	if (acc->mhl_pwr_state) {
		pr_err("%s::MHL off\n", __func__);
		acc_notify(0);
		acc->mhl_pwr_state = false;
	}
#endif
	return 0;
}

static int acc_con_resume(struct platform_device *pdev)
{
	struct acc_con_info *acc = platform_get_drvdata(pdev);
	ACC_CONDEV_DBG("");

	mutex_lock(&acc->lock);
#if defined(CONFIG_DRM_SII9234)
	if (acc->current_dock == DOCK_DESK && !acc->mhl_pwr_state) {
		pr_err("%s::MHL init\n", __func__);
		acc_notify(1);
		acc->mhl_pwr_state = true;
	}
#endif
	mutex_unlock(&acc->lock);

	return 0;
}

static const struct of_device_id acc_con_of_match[] = {
	{ .compatible = "p4note,30pin-con", },
	{ },
};
MODULE_DEVICE_TABLE(of, acc_con_of_match);

static struct platform_driver acc_con_driver = {
	.probe		= acc_con_probe,
	.remove		= acc_con_remove,
	.suspend	= acc_con_suspend,
	.resume		= acc_con_resume,
	.driver		= {
		.name		= "acc_con",
		.owner		= THIS_MODULE,
		.of_match_table = acc_con_of_match,
	},
};

module_platform_driver(acc_con_driver);

MODULE_AUTHOR("Kyungrok Min <gyoungrok.min@samsung.com>");
MODULE_DESCRIPTION("acc connector driver");
MODULE_LICENSE("GPL");
