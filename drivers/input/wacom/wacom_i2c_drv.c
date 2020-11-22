/*
 *  wacom_i2c.c - Wacom G5 Digitizer Controller (I2C bus)
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include "wacom_i2c.h"
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include "wacom_i2c_func.h"
#include "wacom_i2c_flash.h"
#include "wacom_i2c_firm_p4.h"

#define WACOM_UMS_UPDATE
#define WACOM_FW_PATH "/sdcard/firmware/wacom_firm.bin"

static struct class *epen_class;

unsigned char *Binary;
bool ums_binary;

void wacom_i2c_set_firm_data(unsigned char *Binary_new)
{
	if (Binary_new == NULL) {
		Binary = (unsigned char *)Binary_48;
		ums_binary = false;
		return;
	}

	Binary = (unsigned char *)Binary_new;
	ums_binary = true;
}

void wacom_i2c_init_firm_data(void)
{
	Binary = (unsigned char *)Binary_48;
}

static inline int wake_lock_active(struct wakeup_source *lock)
{
	return lock->active;
}

unsigned char screen_rotate;
unsigned char user_hand = 1;
static bool epen_reset_result;

static bool firmware_updating_state;

static void wacom_i2c_enable_irq(struct wacom_i2c *wac_i2c, bool enable)
{
	static int depth;

	mutex_lock(&wac_i2c->irq_lock);
	if (enable) {
		if (depth) {
			--depth;
			enable_irq(wac_i2c->irq);
		}
	} else {
		if (!depth) {
			++depth;
			disable_irq(wac_i2c->irq);
		}
	}
	mutex_unlock(&wac_i2c->irq_lock);

	dev_dbg(&wac_i2c->client->dev, "%s: Enable %d, depth %d\n",
			__func__, (int)enable, depth);
}

static void wacom_i2c_enable_pdct_irq(struct wacom_i2c *wac_i2c, bool enable)
{
	static int depth;

	mutex_lock(&wac_i2c->irq_lock);
	if (enable) {
		if (depth) {
			--depth;
			enable_irq(wac_i2c->irq_pdct);
		}
	} else {
		if (!depth) {
			++depth;
			disable_irq(wac_i2c->irq_pdct);
		}
	}
	mutex_unlock(&wac_i2c->irq_lock);


	dev_dbg(&wac_i2c->client->dev, "%s: Enable %d, depth %d\n",
			__func__, (int)enable, depth);
}

int wacom_power(struct wacom_i2c *wac_i2c, bool on)
{
	struct i2c_client *client = wac_i2c->client;

	int ret = 0;
	static struct regulator *vdd;
	
	dev_info(&client->dev, "%s: power %s\n",
		   __func__, on ? "enabled" : "disabled");

	if (wac_i2c->power_enable == on) {
		dev_info(&client->dev, "%s: pwr already %s\n",
			   __func__, on ? "enabled" : "disabled");
		return 0;
	}

	if (!vdd) {
		vdd = devm_regulator_get(&client->dev, "vdd");

		if (IS_ERR(vdd)) {
			dev_err(&client->dev,
				  "%s: could not get vdd, rc = %ld\n",
				  __func__, PTR_ERR(vdd));
			vdd = NULL;
			return -ENODEV;
		}

		dev_err(&client->dev, "%s: vdd is enabled %s\n",
			  __func__,
			  regulator_is_enabled(vdd) ? "TRUE" : "FALSE");
	}
	
	if (on) {
		ret = regulator_enable(vdd);
		if (ret) {
			dev_err(&client->dev,
				  "%s: Fail to enable regulator vdd[%d]\n",
				  __func__, ret);
		} else {
			dev_err(&client->dev, "%s: vdd is enabled[OK]\n",
				  __func__);
		}
	} else {
		if (regulator_is_enabled(vdd)) {
			ret = regulator_disable(vdd);
			if (ret) {
				dev_err(&client->dev,
					  "%s: Fail to disable regulator vdd[%d]\n",
					  __func__, ret);

			} else {
				dev_err(&client->dev,
					  "%s: vdd is disabled[OK]\n", __func__);
			}
		} else {
			dev_err(&client->dev,
				  "%s: vdd is already disabled\n", __func__);
		}
	}

	wac_i2c->power_enable = on;

	return 0;
}

static int wacom_reset_hw(struct wacom_i2c *wac_i2c)
{
	wacom_power(wac_i2c, false);
	/* recommended delay in spec */
	msleep(100);
	wacom_power(wac_i2c, true);
	msleep(200);

	return 0;
}

void wacom_wakeup_sequence(struct wacom_i2c *wac_i2c)
{
	struct i2c_client *client = wac_i2c->client;

	mutex_lock(&wac_i2c->lock);

	if (wac_i2c->screen_on) {
		dev_info(&client->dev,
			   "already enabled. pass %s\n", __func__);
		goto out_power_on;
	}

	cancel_delayed_work_sync(&wac_i2c->resume_work);
	schedule_delayed_work(&wac_i2c->resume_work,
			      msecs_to_jiffies(180));

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(wac_i2c->irq_pen_insert);

	wac_i2c->screen_on = true;

out_power_on:
	mutex_unlock(&wac_i2c->lock);

	dev_info(&client->dev, "%s: end\n", __func__);
}

void wacom_sleep_sequence(struct wacom_i2c *wac_i2c)
{
	struct i2c_client *client = wac_i2c->client;
	int retry = 1;

	mutex_lock(&wac_i2c->lock);

	if (!wac_i2c->screen_on) {
		dev_info(&client->dev,
			   "already disabled. pass %s\n", __func__);
		goto out_power_off;
	}
	cancel_delayed_work_sync(&wac_i2c->resume_work);

reset:
	if (wac_i2c->reset_flag) {
		dev_info(&client->dev,
			   "%s: IC reset start\n", __func__);

		wac_i2c->reset_flag = false;

		wacom_i2c_enable_irq(wac_i2c, false);
		wacom_i2c_enable_pdct_irq(wac_i2c, false);

		wacom_reset_hw(wac_i2c);

		wac_i2c->pen_pdct =
		    gpio_get_value(wac_i2c->wac_dt_data->gpio_pendct);

		dev_info(&client->dev,
			   "%s: IC reset end, pdct(%d)\n", __func__,
			   wac_i2c->pen_pdct);


		wacom_i2c_enable_irq(wac_i2c, true);
		wacom_i2c_enable_pdct_irq(wac_i2c, true);
	}
	
	/* release pen, if it is pressed */
	if (wac_i2c->pen_pdct == PDCT_DETECT_PEN)
		forced_release(wac_i2c);

	if (wac_i2c->reset_flag && retry--)
		goto reset;

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(wac_i2c->irq_pen_insert);

	wac_i2c->screen_on = false;

out_power_off:
	mutex_unlock(&wac_i2c->lock);

	dev_info(&client->dev, "%s: end\n", __func__);
}

static void wacom_i2c_enable(struct wacom_i2c *wac_i2c)
{
	bool en = true;

	if (wac_i2c->battery_saving_mode
		&& wac_i2c->pen_insert)
		en = false;

	if (en) {
		if (!wac_i2c->power_enable)
			wacom_power(wac_i2c, true);

		cancel_delayed_work_sync(&wac_i2c->resume_work);
		schedule_delayed_work(&wac_i2c->resume_work, HZ / 5);
	}
}

static void wacom_i2c_disable(struct wacom_i2c *wac_i2c)
{
	if (wac_i2c->power_enable) {
		wacom_i2c_enable_irq(wac_i2c, false);
		wacom_i2c_enable_pdct_irq(wac_i2c, false);

		/* release pen, if it is pressed */
		if (wac_i2c->pen_pdct == PDCT_DETECT_PEN)
			forced_release(wac_i2c);

		if (!wake_lock_active(wac_i2c->wakelock)) {
			wac_i2c->power_enable = false;
			wacom_power(wac_i2c, false);
		}
	}
}

int wacom_i2c_get_ums_data(struct wacom_i2c *wac_i2c, u8 **ums_data)
{
	struct file *fp;
	mm_segment_t old_fs;
	long fsize, nread;
	int ret = 0;
	unsigned int nSize;

	nSize = MAX_ADDR_514 + 1;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(WACOM_FW_PATH, O_RDONLY, S_IRUSR);

	if (IS_ERR(fp)) {
		dev_err(&wac_i2c->client->dev, "%s: failed to open %s.\n",
				__func__, WACOM_FW_PATH);
		ret = -ENOENT;
		set_fs(old_fs);
		return ret;
	}

	fsize = fp->f_path.dentry->d_inode->i_size;
	pr_notice("%s: start, file path %s, size %ld Bytes\n",
		__func__, WACOM_FW_PATH, fsize);

	if (fsize != nSize) {
		dev_err(&wac_i2c->client->dev, "%s: UMS firmware size is different\n", __func__);
		ret = -EFBIG;
		goto size_error;
	}

	*ums_data = kmalloc(fsize, GFP_KERNEL);
	if (IS_ERR(*ums_data)) {
		dev_err(&wac_i2c->client->dev, "%s: kmalloc failed\n", __func__);
		ret = -EFAULT;
		goto malloc_error;
	}

	nread = kernel_read(fp, (char __user *)*ums_data,
		fsize, &fp->f_pos);
	pr_notice("%s: nread %ld Bytes\n", __func__, nread);
	if (nread != fsize) {
		dev_err(&wac_i2c->client->dev, "%s: failed to read firmware file, nread %ld Bytes\n",
				__func__, nread);
		ret = -EIO;
		kfree(*ums_data);
		goto read_err;
	}

	filp_close(fp, current->files);
	set_fs(old_fs);

	return 0;

 read_err:
 malloc_error:
 size_error:
	filp_close(fp, current->files);
	set_fs(old_fs);
	return ret;
}

int wacom_i2c_fw_update_UMS(struct wacom_i2c *wac_i2c)
{
	int ret = 0;
	u8 *ums_data = NULL;

	dev_err(&wac_i2c->client->dev, "%s: Start firmware flashing (UMS).\n", __func__);

	/*read firmware data*/
	ret = wacom_i2c_get_ums_data(wac_i2c, &ums_data);
	if (ret < 0) {
		dev_dbg(&wac_i2c->client->dev, "%s: file op is failed\n", __func__);
		return 0;
	}

	/*start firm update*/
	wacom_i2c_set_firm_data(ums_data);

	ret = wacom_i2c_flash(wac_i2c);
	if (ret < 0) {
		dev_err(&wac_i2c->client->dev, "%s: failed to write firmware(%d)\n",
				__func__, ret);
		kfree(ums_data);
		wacom_i2c_set_firm_data(NULL);
		return ret;
	}

	wacom_i2c_set_firm_data(NULL);
	kfree(ums_data);

	return 0;
}

static bool epen_check_factory_mode(void)
{
	struct file *fp;
	mm_segment_t old_fs;
	long fsize;
	int err = 0;
	int nread = 0;
	char *buf;
	bool ret = false;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open("/efs/FactoryApp/factorymode",
			O_RDONLY, S_IRUSR);

	if (IS_ERR(fp)) {
		err = PTR_ERR(fp);
		if (err == -ENOENT)
			pr_err("[E-PEN] There is no file\n");
		else
			pr_err("[E-PEN] File open error(%d)\n", err);

		ret = true;
		goto out;
	}

	fsize = fp->f_path.dentry->d_inode->i_size;

	if (!fsize) {
		pr_err("[E-PEN] File size is zero\n");
		ret = true;
		goto err_filesize;
	}

	buf = kzalloc(fsize, GFP_KERNEL);
	if (!buf) {
		pr_err("[E-PEN] Memory allocate failed\n");
		ret = false;
		goto err_filesize;
	}

	nread = kernel_read(fp, (char __user *)buf, fsize, &fp->f_pos);
	if (nread != fsize) {
		pr_err("[E-PEN] File size[%ld] not same with read one[%d]\n",
				fsize, nread);
		ret = false;
		goto err_readfail;
	}

	/*
	* if the factory mode is disable,
	* do not update the firmware.
	*	factory mode : ON -> disable
	*	factory mode : OFF -> enable
	*/
	if (strncmp("ON", buf, 2))
		ret = true;

	pr_err("[E-PEN] Factorymode is %s\n", ret ? "ENG" : "USER");

err_readfail:
	kfree(buf);
err_filesize:
	filp_close(fp, current->files);
out:
	set_fs(old_fs);
	return ret;
}

static void update_fw_p4(struct wacom_i2c *wac_i2c)
{
	int ret = 0;
	int retry = 2;

	/* the firmware should be updated in factory mode durring the boot */
	if (!epen_check_factory_mode())
		retry = 0;

	while (retry--) {
		dev_dbg(&wac_i2c->client->dev, "%s: INIT_FIRMWARE_FLASH is enabled.\n", __func__);
		ret = wacom_i2c_flash(wac_i2c);
		if (ret == 0)
			break;

		dev_dbg(&wac_i2c->client->dev, "%s: update failed. retry %d, ret %d\n",
		       __func__, retry, ret);

		/* Reset IC */
		wacom_reset_hw(wac_i2c);		
	}

	dev_dbg(&wac_i2c->client->dev, "%s: flashed.(%d)\n", __func__, ret);
}


static irqreturn_t wacom_interrupt(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;
	wacom_i2c_coord(wac_i2c);
	return IRQ_HANDLED;
}

static irqreturn_t wacom_interrupt_pdct(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;

	if (wac_i2c->query_status == false)
		return IRQ_HANDLED;

	wac_i2c->pen_pdct = gpio_get_value(wac_i2c->wac_dt_data->gpio_pendct);
	
	dev_dbg(&wac_i2c->client->dev, "%s: pdct %d(%d)\n", 
			__func__, wac_i2c->pen_pdct, wac_i2c->pen_prox);

	if (wac_i2c->pen_pdct == PDCT_NOSIGNAL) {
		/* If rdy is 1, pen is still working*/
		if (wac_i2c->pen_prox == 0)
			forced_release(wac_i2c);
	} else if (wac_i2c->pen_prox == 0)
		forced_hover(wac_i2c);

	return IRQ_HANDLED;
}

static void pen_insert_work(struct work_struct *work)
{
	struct wacom_i2c *wac_i2c =
		container_of(work, struct wacom_i2c, pen_insert_dwork.work);
	
	dev_dbg(&wac_i2c->client->dev, "%s: %d\n", 
			__func__, wac_i2c->pen_insert);

	input_report_switch(wac_i2c->input_dev,
		SW_PEN_INSERT, !wac_i2c->pen_insert);
	input_sync(wac_i2c->input_dev);

	if (wac_i2c->pen_insert) {
		if (wac_i2c->battery_saving_mode)
			wacom_i2c_disable(wac_i2c);
	} else
		wacom_i2c_enable(wac_i2c);
}
static irqreturn_t wacom_pen_detect(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;

	cancel_delayed_work_sync(&wac_i2c->pen_insert_dwork);
	schedule_delayed_work(&wac_i2c->pen_insert_dwork, HZ / 20);
	return IRQ_HANDLED;
}

static int wacom_i2c_input_open(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);

	dev_info(&wac_i2c->client->dev,
			"%s\n", __func__);

	wacom_wakeup_sequence(wac_i2c);
	
	return 0;
}

static void wacom_i2c_input_close(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);

	if (wake_lock_active(wac_i2c->wakelock)) {
		dev_err(&wac_i2c->client->dev, "%s: wakelock active\n",
					__func__);
		return;
	}

	dev_info(&wac_i2c->client->dev,
			"%s\n", __func__);

	wacom_sleep_sequence(wac_i2c);
}

static void wacom_i2c_set_input_values(struct i2c_client *client,
				       struct wacom_i2c *wac_i2c,
				       struct input_dev *input_dev)
{
	struct wacom_features *wac_feature = wac_i2c->wac_feature;
	/*Set input values before registering input device */
	
	input_dev->name = "Wacom I2C Digitizer";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x56a;
	input_dev->dev.parent = &client->dev;
	input_dev->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	input_dev->evbit[0] |= BIT_MASK(EV_SW);
	input_set_capability(input_dev, EV_SW, SW_PEN_INSERT);

	input_dev->open = wacom_i2c_input_open;
	input_dev->close = wacom_i2c_input_close;

	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_PRESSURE, input_dev->absbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(BTN_TOOL_PEN, input_dev->keybit);
	__set_bit(BTN_TOOL_RUBBER, input_dev->keybit);
	__set_bit(BTN_STYLUS, input_dev->keybit);
	__set_bit(KEY_UNKNOWN, input_dev->keybit);
	__set_bit(KEY_PEN_PDCT, input_dev->keybit);

	/*  __set_bit(BTN_STYLUS2, input_dev->keybit); */
	/*  __set_bit(ABS_MISC, input_dev->absbit); */
	
	input_set_abs_params(input_dev, ABS_PRESSURE, 0,
			     wac_feature->pressure_max, 0, 0);	
	
	if (wac_i2c->wac_dt_data->xy_switch) {
		input_set_abs_params(input_dev, ABS_X, WACOM_POSY_OFFSET,
			wac_feature->y_max, 4, 0);
		input_set_abs_params(input_dev, ABS_Y, WACOM_POSX_OFFSET,
			wac_feature->x_max, 4, 0);
	} else {
		input_set_abs_params(input_dev, ABS_X, WACOM_POSX_OFFSET,
			wac_feature->x_max, 4, 0);
		input_set_abs_params(input_dev, ABS_Y, WACOM_POSY_OFFSET,
			wac_feature->y_max, 4, 0);
	}

	input_abs_set_res(input_dev, ABS_X, WACOM_PIXELS_PER_MM);
	input_abs_set_res(input_dev, ABS_Y, WACOM_PIXELS_PER_MM);
	
	input_set_drvdata(input_dev, wac_i2c);
}

static void wacom_i2c_resume_work(struct work_struct *work)
{
	struct wacom_i2c *wac_i2c =
	    container_of(work, struct wacom_i2c, resume_work.work);

	irq_set_irq_type(wac_i2c->irq_pdct, IRQ_TYPE_EDGE_BOTH);

	irq_set_irq_type(wac_i2c->client->irq, IRQ_TYPE_EDGE_RISING);

	wac_i2c->power_enable = true;
	wacom_i2c_enable_irq(wac_i2c, true);
	wacom_i2c_enable_pdct_irq(wac_i2c, true);

	dev_dbg(&wac_i2c->client->dev, "%s\n", __func__);
}

static ssize_t epen_firm_update_status_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);

	dev_err(&wac_i2c->client->dev, "%s:  (%d)\n", 
			__func__, wac_i2c->wac_feature->firm_update_status);

	if (wac_i2c->wac_feature->firm_update_status == 2)
		return sprintf(buf, "PASS\n");
	else if (wac_i2c->wac_feature->firm_update_status == 1)
		return sprintf(buf, "DOWNLOADING\n");
	else if (wac_i2c->wac_feature->firm_update_status == -1)
		return sprintf(buf, "FAIL\n");
	else
		return 0;
}

static ssize_t epen_firm_version_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	dev_err(&wac_i2c->client->dev, "%s: 0x%x|0x%X\n", 
			__func__, wac_i2c->wac_feature->fw_version, wac_i2c->firmware_version_of_file);

	return sprintf(buf, "%04X\t%04X\n",
			wac_i2c->wac_feature->fw_version,
			wac_i2c->firmware_version_of_file);
}

static bool check_update_condition(struct wacom_i2c *wac_i2c, const char buf)
{
	u32 fw_ic_ver = wac_i2c->wac_feature->fw_version;
	bool bUpdate = false;

	switch (buf) {
	case 'I':
	case 'K':
		bUpdate = true;
		break;
	case 'R':
	case 'W':
		if (fw_ic_ver <
			wac_i2c->firmware_version_of_file)
			bUpdate = true;
		break;
	default:
		dev_err(&wac_i2c->client->dev, "%s: wrong parameter\n", __func__);
		bUpdate = false;
		break;
	}

	return bUpdate;
}

static ssize_t epen_firmware_update_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	int ret = 1;
	u32 fw_ic_ver = wac_i2c->wac_feature->fw_version;
	bool need_update = false;

	dev_dbg(&wac_i2c->client->dev, "%s\n", __func__);

	need_update = check_update_condition(wac_i2c, *buf);
	if (need_update == false) {
		dev_dbg(&wac_i2c->client->dev, "%s: Pass Update."
			"Cmd %c, IC ver %04x, Ker ver %04x\n",
			__func__, *buf, fw_ic_ver, wac_i2c->firmware_version_of_file);
		dev_dbg(&wac_i2c->client->dev, "%s: Pass Update."
				"Cmd %c, IC ver %04x, Ker ver %04x\n", 
					__func__, *buf, fw_ic_ver, wac_i2c->firmware_version_of_file);
		return count;
	}

	/*start firm update*/
	mutex_lock(&wac_i2c->lock);
	wacom_i2c_enable_irq(wac_i2c, false);
	wacom_i2c_enable_pdct_irq(wac_i2c, false);
	wac_i2c->wac_feature->firm_update_status = 1;

	switch (*buf) {
	/*ums*/
	case 'I':
		ret = wacom_i2c_fw_update_UMS(wac_i2c);
		break;
	/*kernel*/
	case 'K':
		dev_err(&wac_i2c->client->dev, "%s: Start firmware flashing (kernel image).\n", __func__);
		ret = wacom_i2c_flash(wac_i2c);
		break;

	/*booting*/
	case 'R':
		update_fw_p4(wac_i2c);
		break;
	default:
		/*There's no default case*/
		break;
	}

	if (ret < 0) {
		dev_err(&wac_i2c->client->dev, "%s: failed to flash firmware.\n", __func__);
		goto failure;
	}

	dev_err(&wac_i2c->client->dev, "%s: Finish firmware flashing.\n", __func__);

	wacom_i2c_query(wac_i2c);

	wac_i2c->wac_feature->firm_update_status = 2;
	wacom_i2c_enable_irq(wac_i2c, true);
	wacom_i2c_enable_pdct_irq(wac_i2c, true);
	mutex_unlock(&wac_i2c->lock);

	return count;

 failure:
	wac_i2c->wac_feature->firm_update_status = -1;
	wacom_i2c_enable_irq(wac_i2c, true);
	wacom_i2c_enable_pdct_irq(wac_i2c, true);
	mutex_unlock(&wac_i2c->lock);
	return -1;
}

static ssize_t epen_sampling_rate_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	int value;
	char mode;

	if (sscanf(buf, "%d", &value) == 1) {
		switch (value) {
		case 0:
			mode = COM_SAMPLERATE_STOP;
			break;
		case 40:
			mode = COM_SAMPLERATE_40;
			break;
		case 80:
			mode = COM_SAMPLERATE_80;
			break;
		case 133:
			mode = COM_SAMPLERATE_133;
			break;
		default:
			pr_err("[E-PEN] Invalid sampling rate value\n");
			count = -1;
			goto fail;
		}

		wacom_i2c_enable_irq(wac_i2c, false);
		wacom_i2c_enable_pdct_irq(wac_i2c, false);
		if (1 == wacom_i2c_send(wac_i2c, &mode, 1, false)) {
			dev_dbg(&wac_i2c->client->dev, "%s: sampling rate %d\n", 
					__func__, value);
			msleep(100);
		} else {
			pr_err("[E-PEN] I2C write error\n");
			wacom_i2c_enable_irq(wac_i2c, true);
			wacom_i2c_enable_pdct_irq(wac_i2c, true);		
			count = -1;
		}
		wacom_i2c_enable_irq(wac_i2c, true);
		wacom_i2c_enable_pdct_irq(wac_i2c, true);

	} else {
		pr_err("[E-PEN] can't get sampling rate data\n");
		count = -1;
	}
 fail:
	return count;
}

static ssize_t epen_reset_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	int ret;
	int val;

	sscanf(buf, "%d", &val);

	if (val == 1) {
		wacom_i2c_enable_irq(wac_i2c, false);
		wacom_i2c_enable_pdct_irq(wac_i2c, false);
		
		/* Reset IC */
		wacom_reset_hw(wac_i2c);
		
		/* I2C Test */
		ret = wacom_i2c_query(wac_i2c);

		wacom_i2c_enable_irq(wac_i2c, true);
		wacom_i2c_enable_pdct_irq(wac_i2c, true);

		if (ret < 0)
			epen_reset_result = false;
		else
			epen_reset_result = true;

		dev_dbg(&wac_i2c->client->dev, "%s: result %d\n",
				__func__, epen_reset_result);
	}

	return count;
}

static ssize_t epen_reset_result_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	
	if (epen_reset_result) {
		dev_dbg(&wac_i2c->client->dev, "%s: PASS\n", __func__);
		return sprintf(buf, "PASS\n");
	} else {
		dev_dbg(&wac_i2c->client->dev, "%s: FAIL\n", __func__);
		return sprintf(buf, "FAIL\n");
	}
}

static ssize_t epen_checksum_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	bool check_version = false;
	int val;

	sscanf(buf, "%d", &val);

	if (wac_i2c->checksum_result)
		return count;
	else
		check_version = true;

	if (val == 1 && check_version) {
		wacom_i2c_enable_irq(wac_i2c, false);
		wacom_i2c_enable_pdct_irq(wac_i2c, false);

		wacom_checksum(wac_i2c);

		wacom_i2c_enable_irq(wac_i2c, true);
		wacom_i2c_enable_pdct_irq(wac_i2c, true);
	}
	dev_dbg(&wac_i2c->client->dev, "%s: check %d, result %d\n",
			__func__, check_version, wac_i2c->checksum_result);

	return count;
}

static ssize_t epen_checksum_result_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);

	if (wac_i2c->checksum_result) {
		dev_dbg(&wac_i2c->client->dev, "%s: checksum, PASS\n", __func__);
		return sprintf(buf, "PASS\n");
	} else {
		dev_dbg(&wac_i2c->client->dev, "%s: checksum, FAIL\n", __func__);
		return sprintf(buf, "FAIL\n");
	}
}

static ssize_t epen_connection_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);

		dev_dbg(&wac_i2c->client->dev,
				"%s: connection_check : %d\n",
				__func__, wac_i2c->connection_check);
	return sprintf(buf, "%s\n",
		wac_i2c->connection_check ?
		"OK" : "NG");
}

static ssize_t epen_type_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", wac_i2c->pen_type);
}

static ssize_t epen_type_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	int val;

	sscanf(buf, "%d", &val);

	wac_i2c->pen_type = !!val;

	return count;
}

static ssize_t epen_saving_mode_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	int val;

	if (sscanf(buf, "%u", &val) == 1)
		wac_i2c->battery_saving_mode = !!val;

	if (wac_i2c->battery_saving_mode) {
		if (wac_i2c->pen_insert)
			wacom_i2c_disable(wac_i2c);
	} else
		wacom_i2c_enable(wac_i2c);
	return count;
}

/* firmware update */
static DEVICE_ATTR(epen_firm_update,
		   S_IWUSR | S_IWGRP, NULL, epen_firmware_update_store);
/* return firmware update status */
static DEVICE_ATTR(epen_firm_update_status,
		   S_IRUGO, epen_firm_update_status_show, NULL);
/* return firmware version */
static DEVICE_ATTR(epen_firm_version, S_IRUGO, epen_firm_version_show, NULL);

static DEVICE_ATTR(epen_sampling_rate,
		   S_IWUSR | S_IWGRP, NULL, epen_sampling_rate_store);
static DEVICE_ATTR(epen_type,
		S_IRUGO | S_IWUSR | S_IWGRP, epen_type_show, epen_type_store);

/* For SMD Test */
static DEVICE_ATTR(epen_reset, S_IWUSR | S_IWGRP, NULL, epen_reset_store);
static DEVICE_ATTR(epen_reset_result,
		   S_IRUSR | S_IRGRP, epen_reset_result_show, NULL);

/* For SMD Test. Check checksum */
static DEVICE_ATTR(epen_checksum, S_IWUSR | S_IWGRP, NULL, epen_checksum_store);
static DEVICE_ATTR(epen_checksum_result, S_IRUSR | S_IRGRP,
		   epen_checksum_result_show, NULL);

static DEVICE_ATTR(epen_connection, S_IRUGO, epen_connection_show, NULL);

static DEVICE_ATTR(epen_saving_mode,
		   S_IWUSR | S_IWGRP, NULL, epen_saving_mode_store);

static struct attribute *epen_attributes[] = {
	&dev_attr_epen_firm_update.attr,
	&dev_attr_epen_firm_update_status.attr,
	&dev_attr_epen_firm_version.attr,
	&dev_attr_epen_sampling_rate.attr,
	&dev_attr_epen_type.attr,
	&dev_attr_epen_reset.attr,
	&dev_attr_epen_reset_result.attr,
	&dev_attr_epen_checksum.attr,
	&dev_attr_epen_checksum_result.attr,
	&dev_attr_epen_connection.attr,
	&dev_attr_epen_saving_mode.attr,
	NULL,
};

static struct attribute_group epen_attr_group = {
	.attrs = epen_attributes,
};

static int wacom_request_gpio(struct i2c_client *client,
							   struct wacom_devicetree_data *wac_dt_data)
{
	int ret;
	pr_info("%s: request gpio\n", __func__);

	ret = devm_gpio_request(&client->dev, wac_dt_data->gpio_pen_pdct, "pen_pdct-gpio");
	if (ret) {
		pr_err("%s: unable to request pen_pdct-gpio [%d]\n",
				__func__, wac_dt_data->gpio_pen_pdct);
		return ret;
	}
	
	ret = devm_gpio_request(&client->dev, wac_dt_data->gpio_int, "wacom_irq");
	if (ret) {
		pr_err("%s: unable to request wacom_irq [%d]\n",
				__func__, wac_dt_data->gpio_int);
		return ret;
	}

	ret = devm_gpio_request(&client->dev, wac_dt_data->gpio_pen_insert, "wacom_pen_insert");
	if (ret) {
		pr_err("%s: unable to request wacom_pen_insert [%d]\n",
				__func__, wac_dt_data->gpio_pen_insert);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_OF
static int wacom_get_dt_coords(struct device *dev, char *name,
				struct wacom_devicetree_data *wac_dt_data)
{
	u32 coords[WACOM_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != WACOM_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "%s: Unable to read %s\n", __func__, name);
		return rc;
	}

	if (strncmp(name, "wacom,panel-coords",
			sizeof("wacom,panel-coords")) == 0) {
		wac_dt_data->x_invert = coords[0];
		wac_dt_data->y_invert = coords[1];
/*
 * below max_x, max_y, min_x, min_y, min_pressure, max_pressure values will be removed.
 * using received value from wacom query data.
 */
		wac_dt_data->min_x = coords[2];
		wac_dt_data->max_x = coords[3];
		wac_dt_data->min_y = coords[4];
		wac_dt_data->max_y = coords[5];
		wac_dt_data->xy_switch = coords[6];
		wac_dt_data->min_pressure = coords[7];
		wac_dt_data->max_pressure = coords[8];

		pr_err("%s: x_invert = %d, y_invert = %d, xy_switch = %d\n",
				__func__, wac_dt_data->x_invert,
				wac_dt_data->y_invert, wac_dt_data->xy_switch);
	} else {
		dev_err(dev, "%s: nsupported property %s\n", __func__, name);
		return -EINVAL;
	}

	return 0;
}

static int wacom_parse_dt(struct device *dev,
			struct wacom_devicetree_data *wac_dt_data)
{
	int rc;
	struct device_node *np = dev->of_node;

	rc = wacom_get_dt_coords(dev, "wacom,panel-coords", wac_dt_data);
	if (rc)
		return rc;

	/* reset, irq gpio info */
	wac_dt_data->gpio_int = of_get_named_gpio_flags(np, "wacom,irq-gpio",
				0, &wac_dt_data->irq_gpio_flags);
	
	wac_dt_data->gpio_pen_pdct = of_get_named_gpio_flags(np,
		"wacom,pen_pdct-gpio", 0, &wac_dt_data->pen_pdct_gpio_flags);
	
	wac_dt_data->gpio_pen_insert = of_get_named_gpio(np, "wacom,sense-gpio", 0);

	rc = of_property_read_u32(np, "wacom,ic_mpu_ver", &wac_dt_data->ic_mpu_ver);
	if (rc < 0)
		dev_info(dev, "%s: Unable to read wacom,ic_mpu_ver\n", __func__);

	rc = of_property_read_u32(np, "wacom,irq_flags", &wac_dt_data->irq_flags);
	if (rc < 0)
		dev_info(dev, "%s: Unable to read wacom,irq_flags\n", __func__);
	
	rc = of_property_read_u32(np, "wacom,irq_type", &wac_dt_data->irq_type);
	if (rc < 0)
		dev_info(dev, "%s: Unable to read wacom,irq_type\n", __func__);
	
	rc = of_property_read_u32(np, "wacom,firmware_version_of_file", &wac_dt_data->firmware_version_of_file);
	if (rc < 0)
		dev_info(dev, "%s: Unable to read wacom,firmware_version_of_file\n", __func__);
	
	rc = of_property_read_u32_array(np, "wacom,firmware_checksum", wac_dt_data->firmware_checksum,
									ARRAY_SIZE(wac_dt_data->firmware_checksum));
	if (rc < 0)
		dev_info(dev, "%s: Unable to read wacom,firmware_checksum\n", __func__);

	pr_err("%s: pdct: %d, insert: %d, mpu: %x, irq_f=%x\n",
			__func__, wac_dt_data->gpio_pen_pdct, wac_dt_data->gpio_pen_insert, 
			wac_dt_data->ic_mpu_ver, wac_dt_data->irq_flags);
	
	return 0;
}
#else
static int wacom_parse_dt(struct device *dev,
			struct wacom_devicetree_data *wac_dt_data)
{
	return -ENODEV;
}
#endif

static int wacom_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct wacom_devicetree_data *wac_dt_data;
	struct wacom_i2c *wac_i2c;
	struct input_dev *input;
	unsigned long i;
	int ret = 0;
	int error;

	firmware_updating_state = false;
	
	/*Check I2C functionality */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: No I2C functionality found\n", __func__);
		ret = -ENODEV;
		goto err_i2c_fail;
	}

	/*Obtain kernel memory space for wacom i2c */
	wac_i2c = devm_kzalloc(&client->dev,
		sizeof(struct wacom_i2c), GFP_KERNEL);
	if (NULL == wac_i2c) {
		dev_err(&client->dev, "%s: Failed to allocate wac_i2c.\n", __func__);
		return -ENOMEM;
	}
	
	if (client->dev.of_node) {
		wac_dt_data = devm_kzalloc(&client->dev,
			sizeof(struct wacom_devicetree_data), GFP_KERNEL);
		if (!wac_dt_data) {
			dev_err(&client->dev, "%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}
		error = wacom_parse_dt(&client->dev, wac_dt_data);
		if (error)
			return error;
	} else {
		wac_dt_data = client->dev.platform_data;
		if (!wac_dt_data) {
			dev_err(&client->dev, "%s: No wac_dt_data\n", __func__);
			ret = -ENODEV;
			goto err_i2c_fail;
		}
	}	
	
	ret = wacom_request_gpio(client, wac_dt_data);
	if (ret) {
		dev_err(&client->dev, "%s: Failed to request gpio\n", __func__);
		return ret;
	}

	/*
	wac_i2c->supplies[0].supply = "vdd";
	ret = devm_regulator_bulk_get(&client->dev, ARRAY_SIZE(wac_i2c->supplies),
				      wac_i2c->supplies);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_info(&client->dev, "%s: failed to get regulators: %d\n", __func__, ret);
		return ret;
	}
	*/
	
	wac_i2c->client_boot = i2c_new_dummy_device(client->adapter,
		WACOM_I2C_BOOT);
	if (!wac_i2c->client_boot) {
		dev_err(&client->dev, "%s: Fail to register sub client[0x%x]\n", __func__, WACOM_I2C_BOOT);
	}

	wac_i2c->wac_feature = devm_kzalloc(&client->dev, sizeof(struct wacom_features), GFP_KERNEL);
	if (!wac_i2c->wac_feature) {
		dev_err(&client->dev,
				"%s: failed to allocate wacom_features.\n", __func__);
		return -ENOMEM;
	}

	/* Set default command state to QUERY */
	wac_i2c->wac_feature->comstat = COM_QUERY;

	wac_i2c->wac_dt_data = wac_dt_data;	
	wac_i2c->client = client;
	
	wac_i2c->ic_mpu_ver = wac_dt_data->ic_mpu_ver;
	wac_i2c->firmware_version_of_file = wac_dt_data->firmware_version_of_file;
	
	for (i = 0; i < 5; i++)
		wac_i2c->firmware_checksum[i] = wac_dt_data->firmware_checksum[i];
	
	wac_i2c->irq_pdct = gpio_to_irq(wac_dt_data->gpio_pendct);

	wac_i2c->gpio_pen_insert = wac_dt_data->gpio_pen_insert;
	wac_i2c->irq_pen_insert = gpio_to_irq(wac_i2c->gpio_pen_insert);

	/* Firmware Feature */
	wacom_i2c_init_firm_data();

	wac_i2c->pen_pdct = PDCT_NOSIGNAL;

	wacom_power(wac_i2c, true);
	wac_i2c->screen_on = true;
	msleep(200);

	wac_i2c->power_enable = true;

	ret = wacom_i2c_query(wac_i2c);
	
	if (ret < 0)
		epen_reset_result = false;
	else
		epen_reset_result = true;

	input = input_allocate_device();
	if (NULL == input) {
		dev_err(&client->dev, "%s: Failed to allocate input device.\n", __func__);
		ret = -ENOMEM;
		goto err_input_allocate_device;
	} else
	wacom_i2c_set_input_values(client, wac_i2c, input);
	
	wac_i2c->input_dev = input;
	
	/*Before registering input device, data in each input_dev must be set */
	ret = input_register_device(input);
	if (ret) {
		dev_err(&client->dev, "%s: failed to register input device.\n", __func__);
		goto err_register_device;
	}

	client->irq = gpio_to_irq(wac_i2c->wac_dt_data->gpio_int);
	dev_info(&wac_i2c->client->dev, "%s: gpio_to_irq : %d\n", __func__, client->irq);
	wac_i2c->irq = client->irq;

	
	/*Set client data */
	i2c_set_clientdata(client, wac_i2c);
	i2c_set_clientdata(wac_i2c->client_boot, wac_i2c);
	
	/*Initializing for semaphor */
	mutex_init(&wac_i2c->lock);
	mutex_init(&wac_i2c->irq_lock);
	
    if((wac_i2c->wakelock = wakeup_source_create("wacom")))
		wakeup_source_add(wac_i2c->wakelock);
	
	INIT_DELAYED_WORK(&wac_i2c->resume_work, wacom_i2c_resume_work);
	INIT_DELAYED_WORK(&wac_i2c->pendct_dwork, wacom_i2c_pendct_work);
	INIT_DELAYED_WORK(&wac_i2c->pen_insert_dwork, pen_insert_work);

	wac_i2c->dev = device_create(epen_class, NULL, wac_i2c->devepen, NULL, "epen");
	if (IS_ERR(wac_i2c->dev)) {
		dev_err(&client->dev, "%s: Failed to create device(wac_i2c->dev)!\n", __func__);
		goto err_sysfs_create_group;
	} else {
		dev_set_drvdata(wac_i2c->dev, wac_i2c);
		ret = sysfs_create_group(&wac_i2c->dev->kobj, &epen_attr_group);
		if (ret) {
			dev_err(&client->dev, "%s: failed to create sysfs group\n", __func__);
			goto err_sysfs_create_group;
		}
	}


	ret = sysfs_create_link(&wac_i2c->dev->kobj, &wac_i2c->input_dev->dev.kobj, "input");
	if (ret < 0)
		dev_err(&wac_i2c->client->dev, "%s: Failed to create input symbolic link[%d]\n",
				__func__, ret);

	/* firmware info */
	dev_info(&wac_i2c->client->dev, "%s: wacom fw ver : 0x%x, new fw ver : 0x%x\n", 
			 __func__, wac_i2c->wac_feature->fw_version, wac_i2c->firmware_version_of_file);
	
	/*Request IRQ */
	if (wac_dt_data->irq_flags) {
		ret = devm_request_threaded_irq(&client->dev, wac_i2c->irq, NULL,
					wacom_interrupt, IRQF_ONESHOT |
					wac_dt_data->irq_type, WACOM_INTERRUPT_NAME, wac_i2c);
		if (ret < 0) {
			dev_err(&client->dev, "%s: failed to request irq(%d) - %d\n",
					__func__,  wac_i2c->irq, ret);
			goto err_request_irq;
		}

		ret = devm_request_threaded_irq(&client->dev, wac_i2c->irq_pdct, NULL,
					wacom_interrupt_pdct, IRQF_ONESHOT |
					wac_dt_data->irq_type, WACOM_PDCT_NAME, wac_i2c);
		if (ret < 0) {
			dev_err(&client->dev, "%s: failed to request irq(%d) - %d\n",
					__func__, wac_i2c->irq_pdct, ret);
			goto err_request_irq;
		}

		ret = devm_request_threaded_irq(&client->dev, wac_i2c->irq_pen_insert, NULL,
					wacom_pen_detect, IRQF_ONESHOT |
					wac_dt_data->irq_type, WACOM_PEN_INSERT_NAME, wac_i2c);
		if (ret < 0)
			dev_err(&client->dev, "%s: failed to request pen insert irq\n", __func__);

		enable_irq_wake(wac_i2c->irq_pen_insert);

		/* update the current status */
		schedule_delayed_work(&wac_i2c->pen_insert_dwork, HZ / 2);

	}
	
	device_init_wakeup(&client->dev, true);

	return 0;

 err_request_irq:
 err_sysfs_create_group:
 err_register_device:
	input_unregister_device(input);
 err_input_allocate_device:
	input_free_device(input);
 err_i2c_fail:
	return ret;
}

static void wacom_i2c_shutdown(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

	if (!wac_i2c)
		return;

	dev_info(&wac_i2c->client->dev, "%s: called!\n", __func__);

	cancel_delayed_work_sync(&wac_i2c->pen_insert_dwork);
	cancel_delayed_work_sync(&wac_i2c->pendct_dwork);
	cancel_delayed_work_sync(&wac_i2c->resume_work);

	wacom_i2c_enable_irq(wac_i2c, false);
	wacom_i2c_enable_pdct_irq(wac_i2c, false);

	wacom_power(wac_i2c, false);

	dev_info(&wac_i2c->client->dev, "%s: Done\n", __func__);
}

static int wacom_i2c_remove(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

	dev_info(&wac_i2c->client->dev, "%s: called!\n", __func__);
	
	sysfs_remove_link(&wac_i2c->dev->kobj, "input");
	
	cancel_delayed_work_sync(&wac_i2c->pen_insert_dwork);
	cancel_delayed_work_sync(&wac_i2c->pendct_dwork);
	cancel_delayed_work_sync(&wac_i2c->resume_work);

	device_init_wakeup(&client->dev, false);
	
	wacom_i2c_enable_irq(wac_i2c, false);
	wacom_i2c_enable_pdct_irq(wac_i2c, false);

	wacom_power(wac_i2c, false);

	wakeup_source_remove(wac_i2c->wakelock);
	mutex_destroy(&wac_i2c->lock);
	mutex_destroy(&wac_i2c->irq_lock);

	dev_info(&wac_i2c->client->dev, "%s: Done\n", __func__);
		
	input_unregister_device(wac_i2c->input_dev);
	wac_i2c->input_dev = NULL;
	
	sysfs_remove_group(&wac_i2c->dev->kobj, &epen_attr_group);
	device_destroy(epen_class, wac_i2c->devepen);		
	
    class_destroy(epen_class);
	
	i2c_unregister_device(wac_i2c->client_boot);
	
	return 0;
}

#ifdef CONFIG_PM
static int wacom_i2c_suspend(struct device *dev)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	
	wac_i2c->pm_suspend = true;
	reinit_completion(&wac_i2c->resume_done);
	if (wac_i2c->input_dev->users)
		wacom_sleep_sequence(wac_i2c);
	
	return 0;
}

static int wacom_i2c_resume(struct device *dev)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	
	wac_i2c->pm_suspend = false;
	complete_all(&wac_i2c->resume_done);
	if (wac_i2c->input_dev->users)
		wacom_wakeup_sequence(wac_i2c);
	
	return 0;
}

static SIMPLE_DEV_PM_OPS(wacom_pm_ops, wacom_i2c_suspend, wacom_i2c_resume);
#endif

static const struct i2c_device_id wacom_i2c_id[] = {
	{"wacom_g5sp_i2c", 0},
	{},
};

#ifdef CONFIG_OF
static struct of_device_id wacom_match_table[] = {
	{ .compatible = "wacom,wacom-g5sp-i2c",},
	{ },
};
#else
#define wacom_match_table	NULL
#endif

/*Create handler for wacom_i2c_driver*/
static struct i2c_driver wacom_i2c_driver = {
	.driver = {
		   .name = "wacom_g5sp_i2c",
#ifdef CONFIG_PM
		   .pm = &wacom_pm_ops,
#endif
		   .of_match_table = wacom_match_table,
		   },
	.probe = wacom_i2c_probe,
	.remove = wacom_i2c_remove,
	.shutdown = wacom_i2c_shutdown,
	.id_table = wacom_i2c_id,
};

static int __init wacom_i2c_init(void)
{
	int ret = 0;

	epen_class = class_create(THIS_MODULE, "epen");
	if (IS_ERR(epen_class)) {
		pr_err("%s: failed to allocate epen class\n", __func__);
		return -ENODEV;
	}
	
	pr_info("%s: Sleep type-PEN_LDO_EN pin\n", __func__);

	ret = i2c_add_driver(&wacom_i2c_driver);
	if (ret)
		pr_err("%s: Failed to add i2c driver\n", __func__);
	return ret;
}

static void __exit wacom_i2c_exit(void)
{
	i2c_del_driver(&wacom_i2c_driver);
}

module_init(wacom_i2c_init);
module_exit(wacom_i2c_exit);

MODULE_AUTHOR("Samsung");
MODULE_DESCRIPTION("Driver for Wacom G5SP Digitizer Controller");
MODULE_LICENSE("GPL");
