// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sony ISX012 5MP camera sensor driver
 *
 * Copyright (c) 2010, Samsung Electronics. All rights reserved
 * Author: dongseong.lim
 *
 * Ported to the mainline v4l2-subdev/media-controller API from the
 * Samsung GPL kernel driver drivers/media/video/isx012.c, as shipped
 * on the GT-N8000. That driver targets the old v4l2_int_device /
 * "sec_camera" style API and carries a large amount of AF/flash/EXIF
 * logic tied to Samsung's out-of-tree camera HAL, none of which
 * applies to the mainline media-controller pipeline (exynos4-is +
 * s5p-mipi-csis + s5p-fimc). This driver only re-implements what is
 * needed to bring the sensor up and stream YUV422 preview frames:
 * power sequencing, the PLL/init/shading-calibration register
 * sequence, resolution changes and a handful of standard V4L2
 * controls (exposure bias, exposure metering, ISO, colorfx) backed
 * by the original tuning tables in isx012-regs.h.
 *
 * Deliberately NOT ported (left as future work, and not required to
 * get a working viewfinder): autofocus (the sensor also drives a VCM
 * lens), flash strobe control, still-capture mode transition/high
 * resolution capture, scene modes, ESD recovery and the I2C burst
 * write optimisation used by the original driver during init.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <asm/unaligned.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#include "isx012-regs.h"

#define ISX012_DRV_NAME			"ISX012"
#define ISX012_CLK_NAME			"extclk"
#define ISX012_DEFAULT_CLK_FREQ	24000000U

#define ISX012_DEFAULT_WIDTH			640
#define ISX012_DEFAULT_HEIGHT			480

/* Registers used directly by this driver; the bulk of the tuning
 * data lives in isx012-regs.h and is only ever written wholesale.
 */
#define REG_VERSION			0x0000
#define REG_INTSTS			0x000E
#define REG_INTCLR			0x0012
#define REG_INTBIT_OM			0x01
#define REG_INTBIT_CM			0x02
#define REG_HSIZE_MONI			0x0090
#define REG_VSIZE_MONI			0x0096
#define REG_STREAM			0x00BF
#define REG_STREAM_STS			0x00DC
#define REG_STREAM_STS_ACTIVE		0x04
#define REG_ENDIAN_SEL			0x5008

/* OTP lens-shading calibration registers, see isx012_load_shading() */
#define REG_OTP1_FLAG			0x004F
#define REG_OTP1_TABLE_SEL		0x005C
#define REG_OTP1_NORR			0x0054
#define REG_OTP1_NORB			0x0056
#define REG_OTP1_PRER			0x005A
#define REG_OTP1_PREB			0x005B
#define REG_OTP0_FLAG			0x0040
#define REG_OTP0_TABLE_SEL		0x004D
#define REG_OTP0_NORR			0x0045
#define REG_OTP0_NORB			0x0047
#define REG_OTP0_PRER			0x004B
#define REG_OTP0_PREB			0x004C
#define REG_SHADING_NORR		0x6804
#define REG_SHADING_NORB		0x6806
#define REG_SHADING_PRER		0x6808
#define REG_SHADING_PREB		0x680A

#define ISX012_OM_CHANGE_RETRIES	30
#define ISX012_CM_CHANGE_RETRIES	280
#define ISX012_STREAMOFF_RETRIES	300
/*
 * The sensor NACKs while its PLL relocks after a mode change (seen right
 * after ISX012_Pll_Setting_4). Retry reads and writes like the vendor
 * driver does.
 */
#define ISX012_I2C_RETRIES		5
#define ISX012_I2C_RETRY_MS		10

enum {
	ISX012_SUPP_CORE,	/* 3MP_CORE_1.2V */
	ISX012_SUPP_IO,		/* CAM_IO_1.8V */
	ISX012_SUPP_A,		/* CAM_SENSOR_A 2.8V, GPIO-switched */
	ISX012_SUPP_AF,		/* 3MP_AF_2.8V, VCM lens driver */
	ISX012_NUM_SUPPLIES,
};

static const char * const isx012_supply_names[] = {
	[ISX012_SUPP_CORE]	= "vddcore",
	[ISX012_SUPP_IO]	= "vddio",
	[ISX012_SUPP_A]		= "vdda",
	[ISX012_SUPP_AF]	= "vddaf",
};

struct isx012_framesize {
	u32 width;
	u32 height;
};

/* Preview/monitor resolutions the original driver exposed. The sensor
 * takes the raw pixel width/height in REG_HSIZE_MONI/REG_VSIZE_MONI,
 * there is no per-resolution register table to select between them.
 */
static const struct isx012_framesize isx012_framesizes[] = {
	{ 176, 144 },
	{ 320, 240 },
	{ 352, 288 },
	{ 528, 432 },
	{ 640, 480 },
	{ 720, 480 },
	{ 880, 720 },
	{ 1024, 576 },
	{ 1024, 768 },
	{ 1280, 720 },
};

struct isx012 {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct mutex lock;

	struct regulator_bulk_data supplies[ISX012_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *standby_gpio;
	struct clk *clock;
	u32 clock_frequency;

	struct v4l2_mbus_framefmt format;
	int power_count;
	bool streaming;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *exposure_bias;
	struct v4l2_ctrl *exposure_metering;
	struct v4l2_ctrl *auto_iso;
	struct v4l2_ctrl *iso;
	struct v4l2_ctrl *colorfx;
};

static inline struct isx012 *sd_to_isx012(struct v4l2_subdev *sd)
{
	return container_of(sd, struct isx012, sd);
}

/* -------------------------------------------------------------------
 * I2C register access. The sensor uses a 16-bit big-endian register
 * address followed by 1/2/4 bytes of little-endian data.
 * -------------------------------------------------------------------
 */
static int isx012_read(struct isx012 *sensor, u16 addr, u32 *val, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	u8 buf[4] = { 0, };
	__be16 wbuf = cpu_to_be16(addr);
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(wbuf),
			.buf = (u8 *)&wbuf,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	int ret, retry;

	if (len != 1 && len != 2 && len != 4)
		return -EINVAL;

	for (retry = 0; retry < ISX012_I2C_RETRIES; retry++) {
		ret = i2c_transfer(client->adapter, msg, 2);
		if (ret == 2)
			break;
		msleep(ISX012_I2C_RETRY_MS);
	}
	if (ret != 2) {
		dev_err(sensor->dev, "failed to read 0x%04x: %d\n", addr, ret);
		/*
		 * Not the NACK's -ENXIO: fimc_pipeline_s_power() reads that
		 * as "no such subdev" and would open the pipeline with the
		 * sensor unpowered.
		 */
		return -EIO;
	}

	/* Register addresses are big-endian, register data little-endian. */
	switch (len) {
	case 1:
		*val = buf[0];
		break;
	case 2:
		*val = get_unaligned_le16(buf);
		break;
	default:
		*val = get_unaligned_le32(buf);
		break;
	}
	return 0;
}

static int isx012_write(struct isx012 *sensor, u16 addr, u32 val, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	u8 buf[6];
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = len + 2,
		.buf = buf,
	};
	int ret, retry;

	if (len != 1 && len != 2 && len != 4)
		return -EINVAL;

	buf[0] = addr >> 8;
	buf[1] = addr & 0xff;

	switch (len) {
	case 1:
		buf[2] = val & 0xff;
		break;
	case 2:
		put_unaligned_le16(val, &buf[2]);
		break;
	default:
		put_unaligned_le32(val, &buf[2]);
		break;
	}

	for (retry = 0; retry < ISX012_I2C_RETRIES; retry++) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			return 0;
		msleep(ISX012_I2C_RETRY_MS);
	}

	dev_err(sensor->dev, "failed to write 0x%04x: %d\n", addr, ret);
	/* -EIO rather than the NACK's -ENXIO, see isx012_read(). */
	return -EIO;
}

static int isx012_write_regs(struct isx012 *sensor,
			      const struct isx012_reg *regs, unsigned int count)
{
	unsigned int i;
	int ret;

	for (i = 0; i < count; i++) {
		/* 0xFFFF is not a real register: it means "sleep val ms",
		 * used by the original tables between mode transitions.
		 */
		if (unlikely(regs[i].addr == 0xFFFF)) {
			msleep(regs[i].val);
			continue;
		}

		ret = isx012_write(sensor, regs[i].addr, regs[i].val,
				    regs[i].len);
		if (ret)
			return ret;
	}
	return 0;
}

#define isx012_write_table(sensor, table) \
	isx012_write_regs(sensor, table, ARRAY_SIZE(table))

/*
 * Wait for the sensor to raise, then clear, the operating-mode (OM)
 * or capture-mode (CM) change interrupt bit. This is how the sensor
 * signals that it has finished applying a mode transition; the
 * retry/delay counts match the original driver.
 */
static int isx012_wait_mode_change(struct isx012 *sensor, u32 bit,
				    unsigned int retries)
{
	unsigned int i;
	u32 val;
	int ret;

	for (i = 0; i < retries; i++) {
		ret = isx012_read(sensor, REG_INTSTS, &val, 1);
		if (ret)
			return ret;
		if (val & bit)
			break;
		msleep(5);
	}
	if (i == retries) {
		dev_err(sensor->dev, "mode change 0x%x not signalled\n", bit);
		return -ETIMEDOUT;
	}

	for (i = 0; i < retries; i++) {
		ret = isx012_write(sensor, REG_INTCLR, bit, 1);
		if (ret)
			return ret;
		ret = isx012_read(sensor, REG_INTSTS, &val, 1);
		if (ret)
			return ret;
		if (!(val & bit))
			return 0;
		msleep(5);
	}
	return -ETIMEDOUT;
}

/*
 * Per-unit lens-shading correction is stored in the sensor's OTP and
 * selects one of four calibration tables; read it back and load the
 * matching table plus the R/B colour normalisation values, exactly
 * as the original isx012_Sensor_Calibration() does.
 */
struct isx012_otp_bank {
	u16 flag;
	u16 table_sel;
	u16 norr;
	u16 norb;
	u16 prer;
	u16 preb;
};

static const struct isx012_otp_bank isx012_otp_banks[] = {
	{
		.flag = REG_OTP1_FLAG, .table_sel = REG_OTP1_TABLE_SEL,
		.norr = REG_OTP1_NORR, .norb = REG_OTP1_NORB,
		.prer = REG_OTP1_PRER, .preb = REG_OTP1_PREB,
	}, {
		.flag = REG_OTP0_FLAG, .table_sel = REG_OTP0_TABLE_SEL,
		.norr = REG_OTP0_NORR, .norb = REG_OTP0_NORB,
		.prer = REG_OTP0_PRER, .preb = REG_OTP0_PREB,
	},
};

/* Copy one OTP calibration value into its shading register. */
static int isx012_copy_otp(struct isx012 *sensor, u16 src, u16 dst,
			   u32 mask, unsigned int shift)
{
	u32 val;
	int ret;

	ret = isx012_read(sensor, src, &val, 2);
	if (ret)
		return ret;

	return isx012_write(sensor, dst, (val & mask) >> shift, 2);
}

static int isx012_load_shading(struct isx012 *sensor)
{
	const struct isx012_otp_bank *bank;
	unsigned int i;
	u32 flag, sel;
	int ret;

	for (i = 0; i < ARRAY_SIZE(isx012_otp_banks); i++) {
		bank = &isx012_otp_banks[i];

		ret = isx012_read(sensor, bank->flag, &flag, 2);
		if (ret)
			return ret;
		if (flag & 0x10)
			break;
	}

	if (i == ARRAY_SIZE(isx012_otp_banks))
		return isx012_write_table(sensor, ISX012_Shading_Nocal);

	ret = isx012_read(sensor, bank->table_sel, &sel, 2);
	if (ret)
		return ret;
	sel = (sel & 0x03C0) >> 6;

	if (sel == 0)
		ret = isx012_write_table(sensor, ISX012_Shading_0);
	else if (sel == 1)
		ret = isx012_write_table(sensor, ISX012_Shading_1);
	else if (sel == 2)
		ret = isx012_write_table(sensor, ISX012_Shading_2);
	if (ret)
		return ret;

	ret = isx012_copy_otp(sensor, bank->norr, REG_SHADING_NORR, 0x3FFF, 0);
	if (ret)
		return ret;
	ret = isx012_copy_otp(sensor, bank->norb, REG_SHADING_NORB, 0x3FFF, 0);
	if (ret)
		return ret;
	ret = isx012_copy_otp(sensor, bank->prer, REG_SHADING_PRER, 0x0FFC, 2);
	if (ret)
		return ret;
	return isx012_copy_otp(sensor, bank->preb, REG_SHADING_PREB, 0x3FF0, 4);
}

/*
 * Full sensor bring-up sequence: PLL/clock setup, releasing the
 * hardware standby pin, per-unit shading calibration and finally the
 * large tuning-table write. Mirrors isx012_post_poweron() +
 * isx012_init() from the original driver, minus the AF/flash/EXIF
 * bits that don't apply here.
 */
static int isx012_hw_init(struct isx012 *sensor)
{
	u32 val;
	int ret;

	ret = isx012_read(sensor, REG_VERSION, &val, 2);
	if (ret) {
		dev_err(sensor->dev, "sensor not responding on I2C: %d\n", ret);
		return ret;
	}
	dev_dbg(sensor->dev, "chip version 0x%04x\n", val);

	/* Settle delays before each OM poll match the vendor driver. */
	msleep(10);
	ret = isx012_wait_mode_change(sensor, REG_INTBIT_OM,
				       ISX012_OM_CHANGE_RETRIES);
	if (ret)
		return ret;

	ret = isx012_write_table(sensor, ISX012_Pll_Setting_4);
	if (ret)
		return ret;

	msleep(10);

	ret = isx012_wait_mode_change(sensor, REG_INTBIT_OM,
				       ISX012_OM_CHANGE_RETRIES);
	if (ret)
		return ret;

	/* Halt internal streaming while we release hardware standby */
	ret = isx012_write(sensor, REG_STREAM, 0x01, 1);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(sensor->standby_gpio, 0);
	msleep(50);

	ret = isx012_wait_mode_change(sensor, REG_INTBIT_OM,
				       ISX012_OM_CHANGE_RETRIES);
	if (ret)
		return ret;

	ret = isx012_wait_mode_change(sensor, REG_INTBIT_CM,
				       ISX012_CM_CHANGE_RETRIES);
	if (ret)
		return ret;

	ret = isx012_write(sensor, REG_ENDIAN_SEL, 0x00, 1);
	if (ret)
		return ret;

	ret = isx012_load_shading(sensor);
	if (ret)
		dev_warn(sensor->dev, "shading calibration failed: %d\n", ret);

	return isx012_write_table(sensor, ISX012_Init_Reg);
}

/* -------------------------------------------------------------------
 * Power sequencing, mirrors isx012_power_on()/isx012_power_down() in
 * arch/arm/mach-exynos/midas-camera.c (P4NOTE variant).
 * -------------------------------------------------------------------
 */
static int isx012_power_on(struct isx012 *sensor)
{
	int ret;

	ret = clk_set_rate(sensor->clock, sensor->clock_frequency);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(sensor->standby_gpio, 1);
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);

	ret = regulator_enable(sensor->supplies[ISX012_SUPP_CORE].consumer);
	if (ret)
		return ret;
	udelay(10);

	ret = regulator_enable(sensor->supplies[ISX012_SUPP_IO].consumer);
	if (ret)
		goto err_core;
	udelay(10);

	ret = regulator_enable(sensor->supplies[ISX012_SUPP_A].consumer);
	if (ret)
		goto err_io;
	udelay(200);

	ret = clk_prepare_enable(sensor->clock);
	if (ret)
		goto err_a;
	udelay(10);

	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	udelay(10);

	ret = regulator_enable(sensor->supplies[ISX012_SUPP_AF].consumer);
	if (ret)
		goto err_clk;
	usleep_range(6000, 6500);

	ret = isx012_hw_init(sensor);
	if (ret) {
		dev_err(sensor->dev, "sensor init failed: %d\n", ret);
		goto err_af;
	}

	return 0;

err_af:
	regulator_disable(sensor->supplies[ISX012_SUPP_AF].consumer);
err_clk:
	clk_disable_unprepare(sensor->clock);
err_a:
	regulator_disable(sensor->supplies[ISX012_SUPP_A].consumer);
err_io:
	regulator_disable(sensor->supplies[ISX012_SUPP_IO].consumer);
err_core:
	regulator_disable(sensor->supplies[ISX012_SUPP_CORE].consumer);
	gpiod_set_value_cansleep(sensor->standby_gpio, 1);
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	return ret;
}

static void isx012_power_off(struct isx012 *sensor)
{
	regulator_disable(sensor->supplies[ISX012_SUPP_AF].consumer);
	udelay(10);

	gpiod_set_value_cansleep(sensor->standby_gpio, 1);
	udelay(10);

	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	udelay(50);

	clk_disable_unprepare(sensor->clock);
	udelay(10);

	regulator_disable(sensor->supplies[ISX012_SUPP_A].consumer);
	udelay(10);

	regulator_disable(sensor->supplies[ISX012_SUPP_IO].consumer);
	udelay(10);

	regulator_disable(sensor->supplies[ISX012_SUPP_CORE].consumer);

	/* The sensor comes back up halted, see isx012_hw_init(). */
	sensor->streaming = false;
}

static int isx012_set_power(struct v4l2_subdev *sd, int on)
{
	struct isx012 *sensor = sd_to_isx012(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);

	if (sensor->power_count == !on) {
		if (on) {
			ret = isx012_power_on(sensor);
			if (!ret) {
				/* isx012_s_ctrl() only touches a powered sensor */
				sensor->power_count = 1;
				ret = __v4l2_ctrl_handler_setup(&sensor->ctrl_handler);
				if (ret) {
					dev_err(sensor->dev,
						"failed to apply controls: %d\n", ret);
					isx012_power_off(sensor);
					sensor->power_count = 0;
				}
			}
		} else {
			isx012_power_off(sensor);
			sensor->power_count = 0;
		}
	}

	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops isx012_core_ops = {
	.s_power = isx012_set_power,
};

/* ------------------------------------------------------------------- */

static const struct isx012_framesize *isx012_find_framesize(u32 width,
							      u32 height)
{
	const struct isx012_framesize *fs, *best = &isx012_framesizes[0];
	long best_err = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(isx012_framesizes); i++) {
		long err;

		fs = &isx012_framesizes[i];
		err = abs((long)fs->width * fs->height -
			  (long)width * height);
		if (best_err < 0 || err < best_err) {
			best_err = err;
			best = fs;
		}
	}
	return best;
}

static void isx012_try_format(struct v4l2_mbus_framefmt *mf)
{
	const struct isx012_framesize *fs;

	fs = isx012_find_framesize(mf->width, mf->height);
	mf->width = fs->width;
	mf->height = fs->height;
	mf->code = MEDIA_BUS_FMT_VYUY8_2X8;
	mf->field = V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_JPEG;
}

static int isx012_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_VYUY8_2X8;
	return 0;
}

static int isx012_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(isx012_framesizes))
		return -EINVAL;
	if (fse->code != MEDIA_BUS_FMT_VYUY8_2X8)
		return -EINVAL;

	fse->min_width = fse->max_width = isx012_framesizes[fse->index].width;
	fse->min_height = fse->max_height = isx012_framesizes[fse->index].height;
	return 0;
}

static struct v4l2_mbus_framefmt *isx012_get_pad_format(
		struct isx012 *sensor, struct v4l2_subdev_pad_config *cfg,
		u32 pad, enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return cfg ? v4l2_subdev_get_try_format(&sensor->sd, cfg, pad) : NULL;

	return &sensor->format;
}

static int isx012_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct isx012 *sensor = sd_to_isx012(sd);
	struct v4l2_mbus_framefmt *mf;
	int ret = 0;

	isx012_try_format(&fmt->format);

	mf = isx012_get_pad_format(sensor, cfg, fmt->pad, fmt->which);
	if (!mf)
		return 0;

	mutex_lock(&sensor->lock);
	/* The size is only programmed in isx012_set_stream() */
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && sensor->streaming)
		ret = -EBUSY;
	else
		*mf = fmt->format;
	mutex_unlock(&sensor->lock);
	return ret;
}

static int isx012_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct isx012 *sensor = sd_to_isx012(sd);
	struct v4l2_mbus_framefmt *mf;

	mf = isx012_get_pad_format(sensor, cfg, fmt->pad, fmt->which);
	if (!mf)
		return -EINVAL;

	mutex_lock(&sensor->lock);
	fmt->format = *mf;
	mutex_unlock(&sensor->lock);
	return 0;
}

static const struct v4l2_subdev_pad_ops isx012_pad_ops = {
	.enum_mbus_code	= isx012_enum_mbus_code,
	.enum_frame_size = isx012_enum_frame_size,
	.get_fmt	= isx012_get_fmt,
	.set_fmt	= isx012_set_fmt,
};

/*
 * After REG_STREAM is set the sensor finishes the current frame before
 * its MIPI output goes quiet. Wait for that, as the vendor driver's
 * isx012_do_wait_steamoff() does, so CSIS/FIMC aren't stopped (or the
 * sensor powered off) mid-frame. A timeout is only logged, like there.
 */
static int isx012_wait_stream_off(struct isx012 *sensor)
{
	unsigned int i;
	u32 val;
	int ret;

	for (i = 0; i < ISX012_STREAMOFF_RETRIES; i++) {
		ret = isx012_read(sensor, REG_STREAM_STS, &val, 1);
		if (ret)
			return ret;
		if (!(val & REG_STREAM_STS_ACTIVE))
			return 0;
		usleep_range(2000, 2500);
	}

	dev_warn(sensor->dev, "stream off not signalled\n");
	return 0;
}

static int isx012_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct isx012 *sensor = sd_to_isx012(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);

	if (sensor->streaming == !!enable)
		goto done;

	if (enable) {
		ret = isx012_write(sensor, REG_HSIZE_MONI,
				    sensor->format.width, 2);
		if (ret)
			goto done;
		ret = isx012_write(sensor, REG_VSIZE_MONI,
				    sensor->format.height, 2);
		if (ret)
			goto done;

		ret = isx012_write_table(sensor, ISX012_Preview_Mode);
		if (ret)
			goto done;

		ret = isx012_wait_mode_change(sensor, REG_INTBIT_CM,
					       ISX012_CM_CHANGE_RETRIES);
		if (ret)
			goto done;

		ret = isx012_write(sensor, REG_STREAM, 0x00, 1);
	} else {
		ret = isx012_write(sensor, REG_STREAM, 0x01, 1);
		if (!ret)
			ret = isx012_wait_stream_off(sensor);
	}

	if (!ret)
		sensor->streaming = !!enable;

done:
	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_video_ops isx012_video_ops = {
	.s_stream = isx012_set_stream,
};

static int isx012_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *format =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);

	format->code = MEDIA_BUS_FMT_VYUY8_2X8;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_JPEG;
	format->width = ISX012_DEFAULT_WIDTH;
	format->height = ISX012_DEFAULT_HEIGHT;
	return 0;
}

static const struct v4l2_subdev_internal_ops isx012_internal_ops = {
	.open = isx012_open,
};

/* -------------------------------------------------------------------
 * V4L2 controls. Each of these just selects one of the sensor's
 * canned register tables (see isx012-regs.h), same as the original
 * driver's isx012_s_ctrl().
 * -------------------------------------------------------------------
 */
static const s64 isx012_ev_bias_qmenu[] = {
	-2000, -1500, -1000, -500, 0, 500, 1000, 1500, 2000
};

static const s64 isx012_iso_qmenu[] = { 50, 100, 200, 400 };

static int isx012_apply_ev_bias(struct isx012 *sensor, s32 idx)
{
	switch (idx) {
	case 0: return isx012_write_table(sensor, ISX012_ExpSetting_M4Step);
	case 1: return isx012_write_table(sensor, ISX012_ExpSetting_M3Step);
	case 2: return isx012_write_table(sensor, ISX012_ExpSetting_M2Step);
	case 3: return isx012_write_table(sensor, ISX012_ExpSetting_M1Step);
	case 4: return isx012_write_table(sensor, ISX012_ExpSetting_Default);
	case 5: return isx012_write_table(sensor, ISX012_ExpSetting_P1Step);
	case 6: return isx012_write_table(sensor, ISX012_ExpSetting_P2Step);
	case 7: return isx012_write_table(sensor, ISX012_ExpSetting_P3Step);
	case 8: return isx012_write_table(sensor, ISX012_ExpSetting_P4Step);
	default: return -EINVAL;
	}
}

static int isx012_apply_metering(struct isx012 *sensor, s32 val)
{
	switch (val) {
	case V4L2_EXPOSURE_METERING_AVERAGE:
		return isx012_write_table(sensor, isx012_Metering_Matrix);
	case V4L2_EXPOSURE_METERING_CENTER_WEIGHTED:
		return isx012_write_table(sensor, isx012_Metering_Center);
	case V4L2_EXPOSURE_METERING_SPOT:
		return isx012_write_table(sensor, isx012_Metering_Spot);
	default:
		return -EINVAL;
	}
}

static int isx012_apply_iso(struct isx012 *sensor, bool is_auto, s32 idx)
{
	if (is_auto)
		return isx012_write_table(sensor, isx012_ISO_Auto);

	switch (idx) {
	case 0: return isx012_write_table(sensor, isx012_ISO_50);
	case 1: return isx012_write_table(sensor, isx012_ISO_100);
	case 2: return isx012_write_table(sensor, isx012_ISO_200);
	case 3: return isx012_write_table(sensor, isx012_ISO_400);
	default: return -EINVAL;
	}
}

static int isx012_apply_colorfx(struct isx012 *sensor, s32 val)
{
	switch (val) {
	case V4L2_COLORFX_NONE:
		return isx012_write_table(sensor, isx012_Effect_Normal);
	case V4L2_COLORFX_BW:
		return isx012_write_table(sensor, isx012_Effect_Black_White);
	case V4L2_COLORFX_SEPIA:
		return isx012_write_table(sensor, isx012_Effect_Sepia);
	case V4L2_COLORFX_NEGATIVE:
		return isx012_write_table(sensor, ISX012_Effect_Negative);
	case V4L2_COLORFX_SKETCH:
		return isx012_write_table(sensor, isx012_Effect_Sketch);
	case V4L2_COLORFX_SOLARIZATION:
		return isx012_write_table(sensor, isx012_Effect_Solar);
	default:
		return -EINVAL;
	}
}

static int isx012_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct isx012 *sensor = container_of(ctrl->handler, struct isx012,
					      ctrl_handler);

	/* Called with sensor->lock held (it is the handler's lock), so
	 * this can't race power sequencing or streaming. Controls are
	 * only meaningful once the sensor is powered; if it isn't, the
	 * cached ctrl->val is pushed to hardware from isx012_set_power().
	 */
	if (!sensor->power_count)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_AUTO_EXPOSURE_BIAS:
		return isx012_apply_ev_bias(sensor, ctrl->val);
	case V4L2_CID_EXPOSURE_METERING:
		return isx012_apply_metering(sensor, ctrl->val);
	case V4L2_CID_ISO_SENSITIVITY_AUTO:
	case V4L2_CID_ISO_SENSITIVITY:
		return isx012_apply_iso(sensor,
				sensor->auto_iso->val == V4L2_ISO_SENSITIVITY_AUTO,
				sensor->iso->val);
	case V4L2_CID_COLORFX:
		return isx012_apply_colorfx(sensor, ctrl->val);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops isx012_ctrl_ops = {
	.s_ctrl = isx012_s_ctrl,
};

static int isx012_init_controls(struct isx012 *sensor)
{
	const struct v4l2_ctrl_ops *ops = &isx012_ctrl_ops;
	struct v4l2_ctrl_handler *hdl = &sensor->ctrl_handler;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 5);
	if (ret)
		return ret;
	hdl->lock = &sensor->lock;

	sensor->exposure_bias = v4l2_ctrl_new_int_menu(hdl, ops,
			V4L2_CID_AUTO_EXPOSURE_BIAS,
			ARRAY_SIZE(isx012_ev_bias_qmenu) - 1,
			ARRAY_SIZE(isx012_ev_bias_qmenu) / 2,
			isx012_ev_bias_qmenu);

	sensor->exposure_metering = v4l2_ctrl_new_std_menu(hdl, ops,
			V4L2_CID_EXPOSURE_METERING, 2, ~0x7,
			V4L2_EXPOSURE_METERING_AVERAGE);

	sensor->auto_iso = v4l2_ctrl_new_std_menu(hdl, ops,
			V4L2_CID_ISO_SENSITIVITY_AUTO, 1, 0,
			V4L2_ISO_SENSITIVITY_AUTO);

	sensor->iso = v4l2_ctrl_new_int_menu(hdl, ops,
			V4L2_CID_ISO_SENSITIVITY,
			ARRAY_SIZE(isx012_iso_qmenu) - 1, 1,
			isx012_iso_qmenu);

	sensor->colorfx = v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_COLORFX,
			V4L2_COLORFX_SOLARIZATION, ~0x202f,
			V4L2_COLORFX_NONE);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	v4l2_ctrl_auto_cluster(2, &sensor->auto_iso, 0, false);
	sensor->sd.ctrl_handler = hdl;
	return 0;
}

static const struct v4l2_subdev_ops isx012_subdev_ops = {
	.core	= &isx012_core_ops,
	.pad	= &isx012_pad_ops,
	.video	= &isx012_video_ops,
};

/* ------------------------------------------------------------------- */

static int isx012_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct isx012 *sensor;
	struct v4l2_subdev *sd;
	unsigned int i;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = dev;
	mutex_init(&sensor->lock);

	sensor->clock = devm_clk_get(dev, ISX012_CLK_NAME);
	if (IS_ERR(sensor->clock))
		return PTR_ERR(sensor->clock);

	if (of_property_read_u32(dev->of_node, "clock-frequency",
				  &sensor->clock_frequency))
		sensor->clock_frequency = ISX012_DEFAULT_CLK_FREQ;

	sensor->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return PTR_ERR(sensor->reset_gpio);

	sensor->standby_gpio = devm_gpiod_get(dev, "standby", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->standby_gpio))
		return PTR_ERR(sensor->standby_gpio);

	for (i = 0; i < ISX012_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = isx012_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ISX012_NUM_SUPPLIES,
				       sensor->supplies);
	if (ret)
		return ret;

	sensor->format.code = MEDIA_BUS_FMT_VYUY8_2X8;
	sensor->format.field = V4L2_FIELD_NONE;
	sensor->format.colorspace = V4L2_COLORSPACE_JPEG;
	sensor->format.width = ISX012_DEFAULT_WIDTH;
	sensor->format.height = ISX012_DEFAULT_HEIGHT;

	sd = &sensor->sd;
	v4l2_i2c_subdev_init(sd, client, &isx012_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->internal_ops = &isx012_internal_ops;

	ret = isx012_init_controls(sensor);
	if (ret)
		return ret;

	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (ret)
		goto err_ctrl_free;

	ret = v4l2_async_register_subdev(sd);
	if (ret)
		goto err_entity_cleanup;

	return 0;

err_entity_cleanup:
	media_entity_cleanup(&sd->entity);
err_ctrl_free:
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
	return ret;
}

static int isx012_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct isx012 *sensor = sd_to_isx012(sd);

	v4l2_async_unregister_subdev(sd);

	mutex_lock(&sensor->lock);
	if (sensor->power_count) {
		isx012_power_off(sensor);
		sensor->power_count = 0;
	}
	mutex_unlock(&sensor->lock);

	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&sensor->lock);
	return 0;
}

static const struct i2c_device_id isx012_ids[] = {
	{ ISX012_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, isx012_ids);

#ifdef CONFIG_OF
static const struct of_device_id isx012_of_match[] = {
	{ .compatible = "sony,isx012" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, isx012_of_match);
#endif

static struct i2c_driver isx012_driver = {
	.driver = {
		.of_match_table	= of_match_ptr(isx012_of_match),
		.name		= ISX012_DRV_NAME,
	},
	.probe_new	= isx012_probe,
	.remove		= isx012_remove,
	.id_table	= isx012_ids,
};

module_i2c_driver(isx012_driver);

MODULE_DESCRIPTION("Sony ISX012 image sensor subdev driver");
MODULE_LICENSE("GPL v2");
