// SPDX-License-Identifier: GPL-2.0
/*
 * drd.c - DesignWare USB2 DRD Controller Dual-role support
 *
 * Copyright (C) 2020 STMicroelectronics
 *
 * Author(s): Amelie Delaunay <amelie.delaunay@st.com>
 */

#include <linux/clk.h>
#include <linux/extcon.h>
#include <linux/iopoll.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/usb/role.h>
#include "core.h"

static void dwc2_ovr_init(struct dwc2_hsotg *hsotg)
{
	unsigned long flags;
	u32 gotgctl;

	spin_lock_irqsave(&hsotg->lock, flags);

	gotgctl = dwc2_readl(hsotg, GOTGCTL);
	gotgctl |= GOTGCTL_BVALOEN | GOTGCTL_AVALOEN | GOTGCTL_VBVALOEN;
	gotgctl |= GOTGCTL_DBNCE_FLTR_BYPASS;
	gotgctl &= ~(GOTGCTL_BVALOVAL | GOTGCTL_AVALOVAL | GOTGCTL_VBVALOVAL);
	dwc2_writel(hsotg, gotgctl, GOTGCTL);

	spin_unlock_irqrestore(&hsotg->lock, flags);

	dwc2_force_mode(hsotg, (hsotg->dr_mode == USB_DR_MODE_HOST));
}

static int dwc2_ovr_avalid(struct dwc2_hsotg *hsotg, bool valid)
{
	u32 gotgctl = dwc2_readl(hsotg, GOTGCTL);

	/* Check if A-Session is already in the right state */
	if ((valid && (gotgctl & GOTGCTL_ASESVLD)) ||
	    (!valid && !(gotgctl & GOTGCTL_ASESVLD)))
		return -EALREADY;

	gotgctl &= ~GOTGCTL_BVALOVAL;
	if (valid)
		gotgctl |= GOTGCTL_AVALOVAL | GOTGCTL_VBVALOVAL;
	else
		gotgctl &= ~(GOTGCTL_AVALOVAL | GOTGCTL_VBVALOVAL);
	dwc2_writel(hsotg, gotgctl, GOTGCTL);

	return 0;
}

static int dwc2_ovr_bvalid(struct dwc2_hsotg *hsotg, bool valid)
{
	u32 gotgctl = dwc2_readl(hsotg, GOTGCTL);

	/* Check if B-Session is already in the right state */
	if ((valid && (gotgctl & GOTGCTL_BSESVLD)) ||
	    (!valid && !(gotgctl & GOTGCTL_BSESVLD)))
		return -EALREADY;

	gotgctl &= ~GOTGCTL_AVALOVAL;
	if (valid)
		gotgctl |= GOTGCTL_BVALOVAL | GOTGCTL_VBVALOVAL;
	else
		gotgctl &= ~(GOTGCTL_BVALOVAL | GOTGCTL_VBVALOVAL);
	dwc2_writel(hsotg, gotgctl, GOTGCTL);

	return 0;
}

static int dwc2_drd_set_role(struct dwc2_hsotg *hsotg, enum usb_role role)
{
	unsigned long flags;
	int already = 0;

	/* Skip session not in line with dr_mode */
	if ((role == USB_ROLE_DEVICE && hsotg->dr_mode == USB_DR_MODE_HOST) ||
	    (role == USB_ROLE_HOST && hsotg->dr_mode == USB_DR_MODE_PERIPHERAL))
		return -EINVAL;

#if IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL) || \
	IS_ENABLED(CONFIG_USB_DWC2_DUAL_ROLE)
	/* Skip session if core is in test mode */
	if (role == USB_ROLE_NONE && hsotg->test_mode) {
		dev_dbg(hsotg->dev, "Core is in test mode\n");
		return -EBUSY;
	}
#endif

	/*
	 * In case of USB_DR_MODE_PERIPHERAL, clock is disabled at the end of
	 * the probe and enabled on udc_start.
	 * If role-switch set is called before the udc_start, we need to enable
	 * the clock to read/write GOTGCTL and GUSBCFG registers to override
	 * mode and sessions. It is the case if cable is plugged at boot.
	 */
	if (!hsotg->ll_hw_enabled && hsotg->clk) {
		int ret = clk_prepare_enable(hsotg->clk);

		if (ret)
			return ret;
	}

	spin_lock_irqsave(&hsotg->lock, flags);

	if (role == USB_ROLE_HOST) {
		already = dwc2_ovr_avalid(hsotg, true);
	} else if (role == USB_ROLE_DEVICE) {
		already = dwc2_ovr_bvalid(hsotg, true);
		/* This clear DCTL.SFTDISCON bit */
		dwc2_hsotg_core_connect(hsotg);
	} else {
		if (dwc2_is_device_mode(hsotg)) {
			if (!dwc2_ovr_bvalid(hsotg, false))
				/* This set DCTL.SFTDISCON bit */
				dwc2_hsotg_core_disconnect(hsotg);
		} else {
			dwc2_ovr_avalid(hsotg, false);
		}
	}

	spin_unlock_irqrestore(&hsotg->lock, flags);

	if (!already && hsotg->dr_mode == USB_DR_MODE_OTG)
		/* This will raise a Connector ID Status Change Interrupt */
		dwc2_force_mode(hsotg, role == USB_ROLE_HOST);

	if (!hsotg->ll_hw_enabled && hsotg->clk)
		clk_disable_unprepare(hsotg->clk);

	dev_dbg(hsotg->dev, "%s-session valid\n",
		role == USB_ROLE_NONE ? "No" :
		role == USB_ROLE_HOST ? "A" : "B");

	return 0;
}

static int dwc2_drd_role_sw_set(struct usb_role_switch *sw, enum usb_role role)
{
	struct dwc2_hsotg *hsotg = usb_role_switch_get_drvdata(sw);

	return dwc2_drd_set_role(hsotg, role);
}

static void dwc2_set_role_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct dwc2_hsotg *hsotg =
		container_of(dwork, struct dwc2_hsotg, edev_work);

	dev_info(hsotg->dev, "Updating current state: %s",
		 hsotg->new_role == USB_ROLE_HOST ? "Host" : "Device");
	dwc2_drd_set_role(hsotg, hsotg->new_role);
}

static int dwc2_drd_notifier(struct notifier_block *nb,
			     unsigned long event, void *ptr)
{
	struct dwc2_hsotg *hsotg = container_of(nb, struct dwc2_hsotg, edev_nb);
	enum usb_role role = event ?
		      USB_ROLE_HOST :
		      USB_ROLE_DEVICE;

	hsotg->new_role = role;

	cancel_delayed_work(&hsotg->edev_work);
	schedule_delayed_work(&hsotg->edev_work, msecs_to_jiffies(10));

	return NOTIFY_DONE;
}

static void dwc2_drd_update(struct dwc2_hsotg *hsotg)
{
	int id;
	enum usb_role role;

	if (hsotg->edev) {
		id = extcon_get_state(hsotg->edev, EXTCON_USB_HOST);
		if (id < 0)
			id = 0;
		role = id ?
			USB_ROLE_HOST :
			USB_ROLE_DEVICE;

		hsotg->new_role = role;
		schedule_delayed_work(&hsotg->edev_work, msecs_to_jiffies(10));
	}
}

static struct extcon_dev *dwc2_get_extcon(struct dwc2_hsotg *hsotg)
{
	struct device *dev = hsotg->dev;
	struct device_node *np_phy, *np_conn;
	struct extcon_dev *edev;
	const char *name;

	if (device_property_read_bool(dev, "extcon"))
		return extcon_get_edev_by_phandle(dev, 0);

	/*
	 * Device tree platforms should get extcon via phandle.
	 * On ACPI platforms, we get the name from a device property.
	 * This device property is for kernel internal use only and
	 * is expected to be set by the glue code.
	 */
	if (device_property_read_string(dev, "linux,extcon-name", &name) == 0) {
		edev = extcon_get_extcon_dev(name);
		if (!edev)
			return ERR_PTR(-EPROBE_DEFER);

		return edev;
	}

	np_phy = of_parse_phandle(dev->of_node, "phys", 0);
	np_conn = of_graph_get_remote_node(np_phy, -1, -1);

	if (np_conn)
		edev = extcon_find_edev_by_node(np_conn);
	else
		edev = NULL;

	of_node_put(np_conn);
	of_node_put(np_phy);

	return edev;
}

int dwc2_drd_init(struct dwc2_hsotg *hsotg)
{
	struct usb_role_switch_desc role_sw_desc = {0};
	struct usb_role_switch *role_sw;
	int ret;

	hsotg->edev = dwc2_get_extcon(hsotg);
	if (IS_ERR(hsotg->edev))
		return PTR_ERR(hsotg->edev);

	if (device_property_read_bool(hsotg->dev, "usb-role-switch")) {
		role_sw_desc.driver_data = hsotg;
		role_sw_desc.fwnode = dev_fwnode(hsotg->dev);
		role_sw_desc.set = dwc2_drd_role_sw_set;
		role_sw_desc.allow_userspace_control = true;

		role_sw = usb_role_switch_register(hsotg->dev, &role_sw_desc);
		if (IS_ERR(role_sw)) {
			ret = PTR_ERR(role_sw);
			dev_err(hsotg->dev,
				"failed to register role switch: %d\n", ret);
			return ret;
		}

		hsotg->role_sw = role_sw;
	} else if (hsotg->edev) {
		hsotg->edev_nb.notifier_call = dwc2_drd_notifier;
		ret = devm_extcon_register_notifier(hsotg->dev, hsotg->edev, EXTCON_USB_HOST,
						    &hsotg->edev_nb);
		if (ret < 0) {
			dev_err(hsotg->dev, "couldn't register cable notifier\n");
			return ret;
		}

		INIT_DELAYED_WORK(&hsotg->edev_work, dwc2_set_role_worker);
		dwc2_drd_update(hsotg);

		dev_info(hsotg->dev, "Successfully registered extcon device");
	} else {
		return 0;
	}

	/* Enable override and initialize values */
	dwc2_ovr_init(hsotg);

	return 0;
}

void dwc2_drd_suspend(struct dwc2_hsotg *hsotg)
{
	u32 gintsts, gintmsk;

	if (hsotg->role_sw && !hsotg->params.external_id_pin_ctl) {
		gintmsk = dwc2_readl(hsotg, GINTMSK);
		gintmsk &= ~GINTSTS_CONIDSTSCHNG;
		dwc2_writel(hsotg, gintmsk, GINTMSK);
		gintsts = dwc2_readl(hsotg, GINTSTS);
		dwc2_writel(hsotg, gintsts | GINTSTS_CONIDSTSCHNG, GINTSTS);
	}
}

void dwc2_drd_resume(struct dwc2_hsotg *hsotg)
{
	u32 gintsts, gintmsk;

	if (hsotg->role_sw && !hsotg->params.external_id_pin_ctl) {
		gintsts = dwc2_readl(hsotg, GINTSTS);
		dwc2_writel(hsotg, gintsts | GINTSTS_CONIDSTSCHNG, GINTSTS);
		gintmsk = dwc2_readl(hsotg, GINTMSK);
		gintmsk |= GINTSTS_CONIDSTSCHNG;
		dwc2_writel(hsotg, gintmsk, GINTMSK);
	}
}

void dwc2_drd_exit(struct dwc2_hsotg *hsotg)
{
	if (hsotg->role_sw)
		usb_role_switch_unregister(hsotg->role_sw);
}
