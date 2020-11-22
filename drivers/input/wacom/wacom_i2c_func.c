/*
 *  wacom_i2c_func.c - Wacom G5 Digitizer Controller (I2C bus)
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

#include "wacom_i2c.h"
#include "wacom_i2c_flash.h"

void forced_release(struct wacom_i2c *wac_i2c)
{
	dev_dbg(&wac_i2c->client->dev, "%s:\n", __func__);
	input_report_abs(wac_i2c->input_dev, ABS_X, wac_i2c->last_x);
	input_report_abs(wac_i2c->input_dev, ABS_Y, wac_i2c->last_y);
	input_report_abs(wac_i2c->input_dev, ABS_PRESSURE, 0);
	input_report_key(wac_i2c->input_dev, BTN_STYLUS, 0);
	input_report_key(wac_i2c->input_dev, BTN_TOUCH, 0);
	input_report_key(wac_i2c->input_dev, BTN_TOOL_RUBBER, 0);
	input_report_key(wac_i2c->input_dev, BTN_TOOL_PEN, 0);
	input_report_key(wac_i2c->input_dev, KEY_PEN_PDCT, 0);
	input_sync(wac_i2c->input_dev);

	wac_i2c->last_x = 0;
	wac_i2c->last_y = 0;
	wac_i2c->pen_prox = 0;
	wac_i2c->pen_pressed = 0;
	wac_i2c->side_pressed = 0;
	wac_i2c->pen_pdct = PDCT_NOSIGNAL;

}

void forced_hover(struct wacom_i2c *wac_i2c)
{
	/* To distinguish hover and pdct area, release */
	if (wac_i2c->last_x != 0 || wac_i2c->last_y != 0) {
		dev_dbg(&wac_i2c->client->dev, "%s: release hover\n", __func__);
		forced_release(wac_i2c);
	}
	wac_i2c->rdy_pdct = true;
	dev_dbg(&wac_i2c->client->dev, "%s:\n", __func__);
	input_report_key(wac_i2c->input_dev, KEY_PEN_PDCT, 1);
	input_sync(wac_i2c->input_dev);

}

void wacom_i2c_pendct_work(struct work_struct *work)
{
	struct wacom_i2c *wac_i2c =
	    container_of(work, struct wacom_i2c, pendct_dwork.work);

	dev_dbg(&wac_i2c->client->dev, "%s: , %d\n",
			__func__, gpio_get_value(wac_i2c->wac_dt_data->gpio_pendct));

	if (gpio_get_value(wac_i2c->wac_dt_data->gpio_pendct))
		forced_release(wac_i2c);
}

int wacom_i2c_send(struct wacom_i2c *wac_i2c,
			  const char *buf, int count, bool mode)
{
	struct i2c_client *client = mode ?
		wac_i2c->client_boot : wac_i2c->client;

	if (wac_i2c->boot_mode && !mode) {
		dev_dbg(&wac_i2c->client->dev, "%s: failed to send\n", __func__);
		return 0;
	}

	return i2c_master_send(client, buf, count);
}

int wacom_i2c_recv(struct wacom_i2c *wac_i2c,
			char *buf, int count, bool mode)
{
	struct i2c_client *client = mode ?
		wac_i2c->client_boot : wac_i2c->client;

	if (wac_i2c->boot_mode && !mode) {
		dev_dbg(&wac_i2c->client->dev, "%s: failed to send\n", __func__);
		return 0;
	}

	return i2c_master_recv(client, buf, count);
}

int wacom_i2c_test(struct wacom_i2c *wac_i2c)
{
	int ret, i;
	char buf, test[10];
	buf = COM_QUERY;

	ret = wacom_i2c_send(wac_i2c, &buf, sizeof(buf), false);
	if (ret > 0)
		dev_info(&wac_i2c->client->dev, "%s: buf:%d, sent:%d\n", __func__, buf, ret);
	else {
		dev_err(&wac_i2c->client->dev, "%s: Digitizer is not active\n", __func__);
		return -1;
	}

	ret = wacom_i2c_recv(wac_i2c, test, sizeof(test), false);
	if (ret >= 0) {
		for (i = 0; i < 8; i++)
			dev_info(&wac_i2c->client->dev, "%s: %d\n", __func__, test[i]);
	} else {
		dev_err(&wac_i2c->client->dev, "%s: Digitizer does not reply\n", __func__);
		return -1;
	}

	return 0;
}

static void wacom_open_test(struct wacom_i2c *wac_i2c)
{
	u8 cmd = 0;
	u8 buf[2] = {0,};
	int ret = 0, cnt = 30;

	cmd = WACOM_I2C_STOP;
	ret = wacom_i2c_send(wac_i2c, &cmd, 1, false);
	if (ret <= 0) {
		dev_err(&wac_i2c->client->dev, "%s: failed to send stop command\n", __func__);
		return ;
	}

	cmd = WACOM_I2C_GRID_CHECK;
	ret = wacom_i2c_send(wac_i2c, &cmd, 1, false);
	if (ret <= 0) {
		dev_err(&wac_i2c->client->dev, "%s: failed to send stop command\n", __func__);		
		goto grid_check_error;
	}

	cmd = WACOM_STATUS;
	do {
		msleep(50);
		if (1 == wacom_i2c_send(wac_i2c, &cmd, 1, false)) {
			if (2 == wacom_i2c_recv(wac_i2c,
						buf, 2, false)) {
				switch (buf[0]) {
				/*
				*	status value
				*	0 : data is not ready
				*	1 : PASS
				*	2 : Fail (coil function error)
				*	3 : Fail (All coil function error)
				*/
				case 1:
				case 2:
				case 3:
					cnt = 0;
					break;

				default:
					break;
				}
			}
		}
	} while (cnt--);

	wac_i2c->connection_check = (1 == buf[0]);
	dev_dbg(&wac_i2c->client->dev, "%s: epen_connection : %s %d\n",
			__func__, (1 == buf[0]) ? "Pass" : "Fail", buf[1]);

grid_check_error:
	cmd = WACOM_I2C_STOP;
	wacom_i2c_send(wac_i2c, &cmd, 1, false);

	cmd = WACOM_I2C_START;
	wacom_i2c_send(wac_i2c, &cmd, 1, false);

}

int wacom_checksum(struct wacom_i2c *wac_i2c)
{
	int ret = 0, retry = 10;
	int i = 0;
	u8 buf[5] = {0, };

	buf[0] = COM_CHECKSUM;

	while (retry--) {
		ret = wacom_i2c_send(wac_i2c, &buf[0], 1, false);
		if (ret < 0) {
			dev_dbg(&wac_i2c->client->dev, "%s: i2c fail, retry, %d\n",
					__func__, __LINE__);
			continue;
		}

		msleep(200);
		ret = wacom_i2c_recv(wac_i2c, buf, 5, false);
		if (ret < 0) {
			dev_dbg(&wac_i2c->client->dev, "%s: i2c fail, retry, %d\n",
					__func__, __LINE__);
			continue;
		} else if (buf[0] == 0x1f)
			break;
		dev_dbg(&wac_i2c->client->dev, "%s: checksum retry\n", __func__);
	}

	if (ret >= 0) {
		dev_dbg(&wac_i2c->client->dev, "%s: received checksum %x, %x, %x, %x, %x\n",
				__func__, buf[0], buf[1], buf[2], buf[3], buf[4]);
	}

	for (i = 0; i < 5; ++i) {
		if (buf[i] != wac_i2c->firmware_checksum[i]) {
			dev_dbg(&wac_i2c->client->dev, "%s: checksum fail %dth %x %x\n",
				__func__, i, buf[i], wac_i2c->firmware_checksum[i]);
			break;
		}
	}

	wac_i2c->checksum_result = (5 == i);

	if (!wac_i2c->connection_check)
		wacom_open_test(wac_i2c);

	return ret;
}

int wacom_i2c_query(struct wacom_i2c *wac_i2c)
{
	struct wacom_features *wac_feature = wac_i2c->wac_feature;
	int ret;
	u8 buf;
	u8 data[9] = {0, };
	int i = 0;
	int query_limit = 10;

	buf = COM_QUERY;

	for (i = 0; i < query_limit; i++) {
		ret = wacom_i2c_send(wac_i2c, &buf, 1, false);
		if (ret < 0) {
			dev_err(&wac_i2c->client->dev, "%s: I2C send failed(%d)\n", __func__, ret);
			continue;
		}
		msleep(100);
		ret = wacom_i2c_recv(wac_i2c, data, COM_QUERY_NUM, false);
		if (ret < 0) {
			dev_err(&wac_i2c->client->dev, "%s: I2C recv failed(%d)\n", __func__, ret);
			continue;
		}
		dev_info(&wac_i2c->client->dev, "%s: %dth ret of wacom query=%d\n",
				 __func__, i, ret);
		if (COM_QUERY_NUM == ret) {
			if (0x0f == data[0]) {
				wac_feature->fw_version =
					((u16) data[7] << 8) + (u16) data[8];
				break;
			} else {
				dev_notice(&wac_i2c->client->dev, "%s: %X, %X, %X, %X, %X, %X, %X, fw=0x%x\n", 
						   __func__, data[0], data[1], data[2], data[3],
						   data[4], data[5], data[6], wac_feature->fw_version);
			}
		}
	}
	
	wac_feature->x_max = (u16) wac_i2c->wac_dt_data->max_x;
	wac_feature->y_max = (u16) wac_i2c->wac_dt_data->max_y;

	
	wac_feature->pressure_max = (u16) data[6] + ((u16) data[5] << 8);

	dev_notice(&wac_i2c->client->dev, "%s: x_max=0x%X\n", 
			   __func__, wac_feature->x_max);
	dev_notice(&wac_i2c->client->dev, "%s: y_max=0x%X\n", 
			   __func__, wac_feature->y_max);
	dev_notice(&wac_i2c->client->dev, "%s: pressure_max=0x%X\n", 
			   __func__, wac_feature->pressure_max);
	dev_notice(&wac_i2c->client->dev, "%s: fw_version=0x%X (d7:0x%X,d8:0x%X)\n", 
			   __func__, wac_feature->fw_version, data[7], data[8]);
	dev_notice(&wac_i2c->client->dev, "%s: %X, %X, %X, %X, %X, %X, %X, %X, %X\n", 
			   __func__, data[0], data[1], data[2], data[3], data[4], data[5], data[6], 
			   data[7], data[8]);

	if ((i == 10) && (ret < 0)) {
		dev_dbg(&wac_i2c->client->dev, "%s: failed\n", __func__);
		wac_i2c->query_status = false;
		return ret;
	}
	wac_i2c->query_status = true;

	wacom_checksum(wac_i2c);

	return 0;
}

static bool wacom_i2c_coord_range(s16 *x, s16 *y, struct wacom_i2c *wac_i2c)
{
	if ((*x <= WACOM_POSX_OFFSET) || (*y <= WACOM_POSY_OFFSET))
		return false;
	if ((*x <= wac_i2c->wac_feature->x_max) && (*y <= wac_i2c->wac_feature->y_max))
		return true;

	return false;
}

int wacom_i2c_coord(struct wacom_i2c *wac_i2c)
{
	bool prox = false;
	int ret = 0;
	u8 *data;
	int rubber, stylus;
	static s16 x, y, pressure;
	static s16 tmp;
	int rdy = 0;

	cancel_delayed_work(&wac_i2c->pendct_dwork);

	data = wac_i2c->wac_feature->data;
	ret = wacom_i2c_recv(wac_i2c, data, COM_COORD_NUM, false);

	if (ret < 0) {
		dev_err(&wac_i2c->client->dev, "%s: failed to read i2c.L%d\n",
				__func__, __LINE__);		
		return -1;
	}
	dev_dbg(&wac_i2c->client->dev, "%s: %x, %x, %x, %x, %x, %x, %x\n", 
			__func__, data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
	if (data[0] & 0x80) {
		/* enable emr device */
		if (!wac_i2c->pen_prox) {

			wac_i2c->pen_prox = 1;

			if (data[0] & 0x40)
				wac_i2c->tool = BTN_TOOL_RUBBER;
			else
				wac_i2c->tool = BTN_TOOL_PEN;
			dev_dbg(&wac_i2c->client->dev, "%s: is in(%d)\n", 
					__func__, wac_i2c->tool);
		}

		prox = !!(data[0] & 0x10);
		stylus = !!(data[0] & 0x20);
		rubber = !!(data[0] & 0x40);
		rdy = !!(data[0] & 0x80);

		x = ((u16) data[1] << 8) + (u16) data[2];
		y = ((u16) data[3] << 8) + (u16) data[4];
		pressure = ((u16) data[5] << 8) + (u16) data[6];

		if (wac_i2c->wac_dt_data->x_invert)
			x = wac_i2c->wac_feature->x_max - x;
		if (wac_i2c->wac_dt_data->y_invert)
			y = wac_i2c->wac_feature->y_max - y;

		if (wac_i2c->wac_dt_data->xy_switch) {
			tmp = x;
			x = y;
			y = tmp;
		}

		if (wacom_i2c_coord_range(&x, &y, wac_i2c)) {
			input_report_abs(wac_i2c->input_dev, ABS_X, x);
			input_report_abs(wac_i2c->input_dev, ABS_Y, y);
			input_report_abs(wac_i2c->input_dev,
					 ABS_PRESSURE, pressure);
			input_report_key(wac_i2c->input_dev,
					 BTN_STYLUS, stylus);
			input_report_key(wac_i2c->input_dev, BTN_TOUCH, prox);
			input_report_key(wac_i2c->input_dev, wac_i2c->tool, 1);
			if (wac_i2c->rdy_pdct) {
				wac_i2c->rdy_pdct = false;
				input_report_key(wac_i2c->input_dev,
					KEY_PEN_PDCT, 0);
			}
			input_sync(wac_i2c->input_dev);
			wac_i2c->last_x = x;
			wac_i2c->last_y = y;

			if (prox && !wac_i2c->pen_pressed) {
				dev_dbg(&wac_i2c->client->dev, 
						"%s: is pressed(%d,%d,%d)(%d)\n",
						__func__, x, y, pressure, wac_i2c->tool);

			} else if (!prox && wac_i2c->pen_pressed) {
				dev_dbg(&wac_i2c->client->dev, 
						"%s: is released(%d,%d,%d)(%d)\n",
						__func__, x, y, pressure, wac_i2c->tool);
			}

			wac_i2c->pen_pressed = prox;

			if (stylus && !wac_i2c->side_pressed)
				dev_dbg(&wac_i2c->client->dev, "%s: side on\n", __func__);
			else if (!stylus && wac_i2c->side_pressed)
				dev_dbg(&wac_i2c->client->dev, "%s: side off\n", __func__);

			wac_i2c->side_pressed = stylus;
		}
		else
			dev_dbg(&wac_i2c->client->dev, "%s: raw data x=%d, y=%d\n",
					__func__, x, y);
	} else {

		if (!gpio_get_value(wac_i2c->wac_dt_data->gpio_pendct)) {
			x = ((u16) data[1] << 8) + (u16) data[2];
			y = ((u16) data[3] << 8) + (u16) data[4];

			if (data[0] & 0x40)
				wac_i2c->tool = BTN_TOOL_RUBBER;
			else
				wac_i2c->tool = BTN_TOOL_PEN;

			input_report_abs(wac_i2c->input_dev, ABS_PRESSURE, 0);
			input_report_key(wac_i2c->input_dev, BTN_STYLUS, 0);
			input_report_key(wac_i2c->input_dev, BTN_TOUCH, 0);
			input_report_key(wac_i2c->input_dev, wac_i2c->tool, 0);
			input_sync(wac_i2c->input_dev);
		}
		schedule_delayed_work(&wac_i2c->pendct_dwork, HZ / 10);

		return 0;
	}
	return 0;
}
