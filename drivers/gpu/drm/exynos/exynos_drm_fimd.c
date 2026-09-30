// SPDX-License-Identifier: GPL-2.0-or-later
/* exynos_drm_fimd.c
 *
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Authors:
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Inki Dae <inki.dae@samsung.com>
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#include <video/of_display_timing.h>
#include <video/of_videomode.h>
#include <video/samsung_fimd.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_vblank.h>
#include <drm/exynos_drm.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_plane.h"

/*
 * FIMD stands for Fully Interactive Mobile Display and
 * as a display controller, it transfers contents drawn on memory
 * to a LCD Panel through Display Interfaces such as RGB or
 * CPU Interface.
 */

#define MIN_FB_WIDTH_FOR_16WORD_BURST 128

/* position control register for hardware window 0, 2 ~ 4.*/
#define VIDOSD_A(win)		(VIDOSD_BASE + 0x00 + (win) * 16)
#define VIDOSD_B(win)		(VIDOSD_BASE + 0x04 + (win) * 16)
/*
 * size control register for hardware windows 0 and alpha control register
 * for hardware windows 1 ~ 4
 */
#define VIDOSD_C(win)		(VIDOSD_BASE + 0x08 + (win) * 16)
/* size control register for hardware windows 1 ~ 2. */
#define VIDOSD_D(win)		(VIDOSD_BASE + 0x0C + (win) * 16)

#define VIDWnALPHA0(win)	(VIDW_ALPHA + 0x00 + (win) * 8)
#define VIDWnALPHA1(win)	(VIDW_ALPHA + 0x04 + (win) * 8)

#define VIDWx_BUF_START(win, buf)	(VIDW_BUF_START(buf) + (win) * 8)
#define VIDWx_BUF_START_S(win, buf)	(VIDW_BUF_START_S(buf) + (win) * 8)
#define VIDWx_BUF_END(win, buf)		(VIDW_BUF_END(buf) + (win) * 8)
#define VIDWx_BUF_SIZE(win, buf)	(VIDW_BUF_SIZE(buf) + (win) * 4)

/* color key control register for hardware window 1 ~ 4. */
#define WKEYCON0_BASE(x)		((WKEYCON0 + 0x140) + ((x - 1) * 8))
/* color key value register for hardware window 1 ~ 4. */
#define WKEYCON1_BASE(x)		((WKEYCON1 + 0x140) + ((x - 1) * 8))

/* I80 trigger control register */
#define TRIGCON				0x1A4
#define TRGMODE_ENABLE			(1 << 0)
#define SWTRGCMD_ENABLE			(1 << 1)
/* Exynos3250, 3472, 5260 5410, 5420 and 5422 only supported. */
#define HWTRGEN_ENABLE			(1 << 3)
#define HWTRGMASK_ENABLE		(1 << 4)
/* Exynos3250, 3472, 5260, 5420 and 5422 only supported. */
#define HWTRIGEN_PER_ENABLE		(1 << 31)

/* display mode change control register except exynos4 */
#define VIDOUT_CON			0x000
#define VIDOUT_CON_F_I80_LDI0		(0x2 << 8)

/* I80 interface control for main LDI register */
#define I80IFCONFAx(x)			(0x1B0 + (x) * 4)
#define I80IFCONFBx(x)			(0x1B8 + (x) * 4)
#define LCD_CS_SETUP(x)			((x) << 16)
#define LCD_WR_SETUP(x)			((x) << 12)
#define LCD_WR_ACTIVE(x)		((x) << 8)
#define LCD_WR_HOLD(x)			((x) << 4)
#define I80IFEN_ENABLE			(1 << 0)

/* FIMD has totally five hardware windows. */
#define WINDOWS_NR	5

/*
 * WINCONx.ENWIN and the SHADOWCON channel enables take effect immediately,
 * not at vsync like the rest of the window setup. Turning a window on
 * mid-frame therefore scans it out with its stale latched registers (old
 * position, size and buffer address) until the next vsync, and turning it
 * off tears the current frame. While the display is on, windows 1-4 are
 * never switched off: a disabled plane is parked instead, i.e. pointed at a
 * small fully transparent buffer through the shadowed registers. 128 pixels
 * keeps the parked window on the 16-word DMA burst. Window 0 has no alpha,
 * so it is still switched off.
 */
#define PARK_WIDTH	128
#define PARK_HEIGHT	32
#define PARK_PITCH	(PARK_WIDTH * 4)
#define PARK_SIZE	(PARK_PITCH * PARK_HEIGHT)

/*
 * FIMD can stop scanning out mid-frame (seen with a window whose pitch is
 * not a multiple of 8 bytes, and when a DMA channel is enabled mid-frame):
 * the line counter stops, no frame interrupt comes and every later commit
 * times out. Toggling ENVID restarts it. While the display is on, VIDCON1
 * is checked every FIMD_HANG_CHECK_MS; unchanged over 3 ms means a hang,
 * since vertical blanking only lasts a fraction of a millisecond.
 */
#define FIMD_HANG_CHECK_MS	50
#define FIMD_HANG_SAMPLES	3

/* HW trigger flag on i80 panel. */
#define I80_HW_TRG     (1 << 1)

struct fimd_driver_data {
	unsigned int timing_base;
	unsigned int lcdblk_offset;
	unsigned int lcdblk_vt_shift;
	unsigned int lcdblk_bypass_shift;
	unsigned int lcdblk_mic_bypass_shift;
	unsigned int trg_type;

	unsigned int has_shadowcon:1;
	unsigned int has_clksel:1;
	unsigned int has_limited_fmt:1;
	unsigned int has_vidoutcon:1;
	unsigned int has_vtsel:1;
	unsigned int has_mic_bypass:1;
	unsigned int has_dp_clk:1;
	unsigned int has_hw_trigger:1;
	unsigned int has_trigger_per_te:1;
	unsigned int has_bgr_support:1;
};

static struct fimd_driver_data s3c64xx_fimd_driver_data = {
	.timing_base = 0x0,
	.has_clksel = 1,
	.has_limited_fmt = 1,
};

static struct fimd_driver_data s5pv210_fimd_driver_data = {
	.timing_base = 0x0,
	.has_shadowcon = 1,
	.has_clksel = 1,
};

static struct fimd_driver_data exynos3_fimd_driver_data = {
	.timing_base = 0x20000,
	.lcdblk_offset = 0x210,
	.lcdblk_bypass_shift = 1,
	.has_shadowcon = 1,
	.has_vidoutcon = 1,
};

static struct fimd_driver_data exynos4_fimd_driver_data = {
	.timing_base = 0x0,
	.lcdblk_offset = 0x210,
	.lcdblk_vt_shift = 10,
	.lcdblk_bypass_shift = 1,
	.has_shadowcon = 1,
	.has_vtsel = 1,
	.has_bgr_support = 1,
};

static struct fimd_driver_data exynos5_fimd_driver_data = {
	.timing_base = 0x20000,
	.lcdblk_offset = 0x214,
	.lcdblk_vt_shift = 24,
	.lcdblk_bypass_shift = 15,
	.has_shadowcon = 1,
	.has_vidoutcon = 1,
	.has_vtsel = 1,
	.has_dp_clk = 1,
	.has_bgr_support = 1,
};

static struct fimd_driver_data exynos5420_fimd_driver_data = {
	.timing_base = 0x20000,
	.lcdblk_offset = 0x214,
	.lcdblk_vt_shift = 24,
	.lcdblk_bypass_shift = 15,
	.lcdblk_mic_bypass_shift = 11,
	.has_shadowcon = 1,
	.has_vidoutcon = 1,
	.has_vtsel = 1,
	.has_mic_bypass = 1,
	.has_dp_clk = 1,
	.has_bgr_support = 1,
};

struct fimd_context {
	struct device			*dev;
	struct drm_device		*drm_dev;
	void				*dma_priv;
	struct exynos_drm_crtc		*crtc;
	struct exynos_drm_plane		planes[WINDOWS_NR];
	struct exynos_drm_plane_config	configs[WINDOWS_NR];
	struct clk			*bus_clk;
	struct clk			*lcd_clk;
	void __iomem			*regs;
	struct regmap			*sysreg;
	unsigned long			irq_flags;
	u32				vidcon0;
	u32				vidcon1;
	u32				vidout_con;
	u32				i80ifcon;
	bool				i80_if;
	bool				suspended;
	bool				dp_clk_enabled;
	wait_queue_head_t		wait_vsync_queue;
	atomic_t			wait_vsync_event;
	atomic_t			win_updated;
	atomic_t			triggering;
	atomic_t			fifo_underruns;
	atomic_t			hang_recoveries;
	struct delayed_work		hang_work;
	u32				clkdiv;
	void				*park_vaddr;
	dma_addr_t			park_dma;
	resource_size_t			regs_size;
	struct mutex			testbuf_lock;
	void				*testbuf_vaddr;
	dma_addr_t			testbuf_dma;
	size_t				testbuf_size;
	void __iomem			*sysmmu_regs;
	struct clk			*sysmmu_clk;

	const struct fimd_driver_data *driver_data;
	struct drm_encoder *encoder;
	struct exynos_drm_clk		dp_clk;
};

static const struct of_device_id fimd_driver_dt_match[] = {
	{ .compatible = "samsung,s3c6400-fimd",
	  .data = &s3c64xx_fimd_driver_data },
	{ .compatible = "samsung,s5pv210-fimd",
	  .data = &s5pv210_fimd_driver_data },
	{ .compatible = "samsung,exynos3250-fimd",
	  .data = &exynos3_fimd_driver_data },
	{ .compatible = "samsung,exynos4210-fimd",
	  .data = &exynos4_fimd_driver_data },
	{ .compatible = "samsung,exynos5250-fimd",
	  .data = &exynos5_fimd_driver_data },
	{ .compatible = "samsung,exynos5420-fimd",
	  .data = &exynos5420_fimd_driver_data },
	{},
};
MODULE_DEVICE_TABLE(of, fimd_driver_dt_match);

static const enum drm_plane_type fimd_win_types[WINDOWS_NR] = {
	DRM_PLANE_TYPE_PRIMARY,
	DRM_PLANE_TYPE_OVERLAY,
	DRM_PLANE_TYPE_OVERLAY,
	DRM_PLANE_TYPE_OVERLAY,
	DRM_PLANE_TYPE_CURSOR,
};

static const uint32_t fimd_formats[] = {
	DRM_FORMAT_C8,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static const uint32_t fimd_extended_formats[] = {
	DRM_FORMAT_C8,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_XBGR1555,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
};

static const unsigned int capabilities[WINDOWS_NR] = {
	0,
	EXYNOS_DRM_PLANE_CAP_WIN_BLEND | EXYNOS_DRM_PLANE_CAP_PIX_BLEND,
	EXYNOS_DRM_PLANE_CAP_WIN_BLEND | EXYNOS_DRM_PLANE_CAP_PIX_BLEND,
	EXYNOS_DRM_PLANE_CAP_WIN_BLEND | EXYNOS_DRM_PLANE_CAP_PIX_BLEND,
	EXYNOS_DRM_PLANE_CAP_WIN_BLEND | EXYNOS_DRM_PLANE_CAP_PIX_BLEND,
};

static inline void fimd_set_bits(struct fimd_context *ctx, u32 reg, u32 mask,
				 u32 val)
{
	val = (val & mask) | (readl(ctx->regs + reg) & ~mask);
	writel(val, ctx->regs + reg);
}

static int fimd_enable_vblank(struct exynos_drm_crtc *crtc)
{
	struct fimd_context *ctx = crtc->ctx;
	u32 val;

	if (ctx->suspended)
		return -EPERM;

	if (!test_and_set_bit(0, &ctx->irq_flags)) {
		val = readl(ctx->regs + VIDINTCON0);

		val |= VIDINTCON0_INT_ENABLE;

		if (ctx->i80_if) {
			val |= VIDINTCON0_INT_I80IFDONE;
			val |= VIDINTCON0_INT_SYSMAINCON;
			val &= ~VIDINTCON0_INT_SYSSUBCON;
		} else {
			val |= VIDINTCON0_INT_FRAME;

			val &= ~VIDINTCON0_FRAMESEL0_MASK;
			val |= VIDINTCON0_FRAMESEL0_FRONTPORCH;
			val &= ~VIDINTCON0_FRAMESEL1_MASK;
			val |= VIDINTCON0_FRAMESEL1_NONE;
		}

		writel(val, ctx->regs + VIDINTCON0);
	}

	return 0;
}

static void fimd_disable_vblank(struct exynos_drm_crtc *crtc)
{
	struct fimd_context *ctx = crtc->ctx;
	u32 val;

	if (ctx->suspended)
		return;

	if (test_and_clear_bit(0, &ctx->irq_flags)) {
		val = readl(ctx->regs + VIDINTCON0);

		/* The FIFO underrun interrupt needs the global enable too. */
		if (!(val & VIDINTCON0_INT_FIFO_EN))
			val &= ~VIDINTCON0_INT_ENABLE;

		if (ctx->i80_if) {
			val &= ~VIDINTCON0_INT_I80IFDONE;
			val &= ~VIDINTCON0_INT_SYSMAINCON;
			val &= ~VIDINTCON0_INT_SYSSUBCON;
		} else
			val &= ~VIDINTCON0_INT_FRAME;

		writel(val, ctx->regs + VIDINTCON0);
	}
}

/*
 * Arm the FIFO-empty (underrun) interrupt for all windows. The handler
 * disarms it on the first underrun, and the next atomic flush re-arms it,
 * so fifo_underruns counts commits with at least one underrun instead of
 * flooding the CPU with one interrupt per starved line.
 */
static void fimd_arm_fifo_irq(struct fimd_context *ctx)
{
	u32 val;

	if (ctx->suspended || ctx->i80_if)
		return;

	val = readl(ctx->regs + VIDINTCON0);
	if (val & VIDINTCON0_INT_FIFO_EN)
		return;

	val &= ~(VIDINTCON0_FIFIOSEL_MASK | VIDINTCON0_FIFOLEVEL_MASK);
	val |= VIDINTCON0_FIFIOSEL_ALL | VIDINTCON0_FIFOLEVEL_EMPTY |
	       VIDINTCON0_INT_FIFO_EN | VIDINTCON0_INT_ENABLE;
	writel(val, ctx->regs + VIDINTCON0);
}

static void fimd_disarm_fifo_irq(struct fimd_context *ctx)
{
	u32 val;

	val = readl(ctx->regs + VIDINTCON0);
	val &= ~VIDINTCON0_INT_FIFO_EN;
	if (!test_bit(0, &ctx->irq_flags))
		val &= ~VIDINTCON0_INT_ENABLE;
	writel(val, ctx->regs + VIDINTCON0);
}

static void fimd_wait_for_vblank(struct exynos_drm_crtc *crtc)
{
	struct fimd_context *ctx = crtc->ctx;

	if (ctx->suspended)
		return;

	atomic_set(&ctx->wait_vsync_event, 1);

	/*
	 * wait for FIMD to signal VSYNC interrupt or return after
	 * timeout which is set to 50ms (refresh rate of 20).
	 */
	if (!wait_event_timeout(ctx->wait_vsync_queue,
				!atomic_read(&ctx->wait_vsync_event),
				HZ/20))
		DRM_DEV_DEBUG_KMS(ctx->dev, "vblank wait timed out.\n");
}

static void fimd_enable_video_output(struct fimd_context *ctx, unsigned int win,
					bool enable)
{
	u32 val = readl(ctx->regs + WINCON(win));

	if (enable)
		val |= WINCONx_ENWIN;
	else
		val &= ~WINCONx_ENWIN;

	writel(val, ctx->regs + WINCON(win));
}

static void fimd_enable_shadow_channel_path(struct fimd_context *ctx,
						unsigned int win,
						bool enable)
{
	u32 val = readl(ctx->regs + SHADOWCON);

	if (enable)
		val |= SHADOWCON_CHx_ENABLE(win);
	else
		val &= ~SHADOWCON_CHx_ENABLE(win);

	writel(val, ctx->regs + SHADOWCON);
}

static int fimd_clear_channels(struct exynos_drm_crtc *crtc)
{
	struct fimd_context *ctx = crtc->ctx;
	unsigned int win, ch_enabled = 0;
	int ret;

	/* Hardware is in unknown state, so ensure it gets enabled properly */
	ret = pm_runtime_resume_and_get(ctx->dev);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to enable FIMD device.\n");
		return ret;
	}

	clk_prepare_enable(ctx->bus_clk);
	clk_prepare_enable(ctx->lcd_clk);

	/* Check if any channel is enabled. */
	for (win = 0; win < WINDOWS_NR; win++) {
		u32 val = readl(ctx->regs + WINCON(win));

		if (val & WINCONx_ENWIN) {
			fimd_enable_video_output(ctx, win, false);

			if (ctx->driver_data->has_shadowcon)
				fimd_enable_shadow_channel_path(ctx, win,
								false);

			ch_enabled = 1;
		}
	}

	/* Wait for vsync, as disable channel takes effect at next vsync */
	if (ch_enabled) {
		ctx->suspended = false;

		fimd_enable_vblank(ctx->crtc);
		fimd_wait_for_vblank(ctx->crtc);
		fimd_disable_vblank(ctx->crtc);

		ctx->suspended = true;
	}

	clk_disable_unprepare(ctx->lcd_clk);
	clk_disable_unprepare(ctx->bus_clk);

	pm_runtime_put(ctx->dev);

	return 0;
}


static int fimd_atomic_check(struct exynos_drm_crtc *crtc,
		struct drm_crtc_state *state)
{
	struct drm_display_mode *mode = &state->adjusted_mode;
	struct fimd_context *ctx = crtc->ctx;
	unsigned long ideal_clk, lcd_rate;
	u32 clkdiv;
	const struct drm_plane_state *plane_state;
	struct drm_plane *plane;

	/*
	 * VIDWxxADD2: PAGEWIDTH + OFFSIZE, i.e. the pitch, must be a multiple
	 * of 8 bytes. Scanout stops at the end of such a window otherwise.
	 */
	drm_atomic_crtc_state_for_each_plane_state(plane, plane_state, state) {
		if (plane_state->fb && plane_state->fb->pitches[0] % 8) {
			DRM_DEV_DEBUG_KMS(ctx->dev,
					  "[PLANE:%d] pitch %u not a multiple of 8\n",
					  plane->base.id,
					  plane_state->fb->pitches[0]);
			return -EINVAL;
		}
	}

	if (mode->clock == 0) {
		DRM_DEV_ERROR(ctx->dev, "Mode has zero clock value.\n");
		return -EINVAL;
	}

	ideal_clk = mode->clock * 1000;

	if (ctx->i80_if) {
		/*
		 * The frame done interrupt should be occurred prior to the
		 * next TE signal.
		 */
		ideal_clk *= 2;
	}

	lcd_rate = clk_get_rate(ctx->lcd_clk);
	if (2 * lcd_rate < ideal_clk) {
		DRM_DEV_ERROR(ctx->dev,
			      "sclk_fimd clock too low(%lu) for requested pixel clock(%lu)\n",
			      lcd_rate, ideal_clk);
		return -EINVAL;
	}

	/* Find the clock divider value that gets us closest to ideal_clk */
	clkdiv = DIV_ROUND_CLOSEST(lcd_rate, ideal_clk);
	if (clkdiv >= 0x200) {
		DRM_DEV_ERROR(ctx->dev, "requested pixel clock(%lu) too low\n",
			      ideal_clk);
		return -EINVAL;
	}

	ctx->clkdiv = (clkdiv < 0x100) ? clkdiv : 0xff;

	return 0;
}

static void fimd_setup_trigger(struct fimd_context *ctx)
{
	void __iomem *timing_base = ctx->regs + ctx->driver_data->timing_base;
	u32 trg_type = ctx->driver_data->trg_type;
	u32 val = readl(timing_base + TRIGCON);

	val &= ~(TRGMODE_ENABLE);

	if (trg_type == I80_HW_TRG) {
		if (ctx->driver_data->has_hw_trigger)
			val |= HWTRGEN_ENABLE | HWTRGMASK_ENABLE;
		if (ctx->driver_data->has_trigger_per_te)
			val |= HWTRIGEN_PER_ENABLE;
	} else {
		val |= TRGMODE_ENABLE;
	}

	writel(val, timing_base + TRIGCON);
}

static void fimd_commit(struct exynos_drm_crtc *crtc)
{
	struct fimd_context *ctx = crtc->ctx;
	struct drm_display_mode *mode = &crtc->base.state->adjusted_mode;
	const struct fimd_driver_data *driver_data = ctx->driver_data;
	void *timing_base = ctx->regs + driver_data->timing_base;
	u32 val;

	if (ctx->suspended)
		return;

	/* nothing to do if we haven't set the mode yet */
	if (mode->htotal == 0 || mode->vtotal == 0)
		return;

	if (ctx->i80_if) {
		val = ctx->i80ifcon | I80IFEN_ENABLE;
		writel(val, timing_base + I80IFCONFAx(0));

		/* disable auto frame rate */
		writel(0, timing_base + I80IFCONFBx(0));

		/* set video type selection to I80 interface */
		if (driver_data->has_vtsel && ctx->sysreg &&
				regmap_update_bits(ctx->sysreg,
					driver_data->lcdblk_offset,
					0x3 << driver_data->lcdblk_vt_shift,
					0x1 << driver_data->lcdblk_vt_shift)) {
			DRM_DEV_ERROR(ctx->dev,
				      "Failed to update sysreg for I80 i/f.\n");
			return;
		}
	} else {
		int vsync_len, vbpd, vfpd, hsync_len, hbpd, hfpd;
		u32 vidcon1;

		/* setup polarity values */
		vidcon1 = ctx->vidcon1;
		if (mode->flags & DRM_MODE_FLAG_NVSYNC)
			vidcon1 |= VIDCON1_INV_VSYNC;
		if (mode->flags & DRM_MODE_FLAG_NHSYNC)
			vidcon1 |= VIDCON1_INV_HSYNC;
		writel(vidcon1, ctx->regs + driver_data->timing_base + VIDCON1);

		/* setup vertical timing values. */
		vsync_len = mode->crtc_vsync_end - mode->crtc_vsync_start;
		vbpd = mode->crtc_vtotal - mode->crtc_vsync_end;
		vfpd = mode->crtc_vsync_start - mode->crtc_vdisplay;

		val = VIDTCON0_VBPD(vbpd - 1) |
			VIDTCON0_VFPD(vfpd - 1) |
			VIDTCON0_VSPW(vsync_len - 1);
		writel(val, ctx->regs + driver_data->timing_base + VIDTCON0);

		/* setup horizontal timing values.  */
		hsync_len = mode->crtc_hsync_end - mode->crtc_hsync_start;
		hbpd = mode->crtc_htotal - mode->crtc_hsync_end;
		hfpd = mode->crtc_hsync_start - mode->crtc_hdisplay;

		val = VIDTCON1_HBPD(hbpd - 1) |
			VIDTCON1_HFPD(hfpd - 1) |
			VIDTCON1_HSPW(hsync_len - 1);
		writel(val, ctx->regs + driver_data->timing_base + VIDTCON1);
	}

	if (driver_data->has_vidoutcon)
		writel(ctx->vidout_con, timing_base + VIDOUT_CON);

	/* set bypass selection */
	if (ctx->sysreg && regmap_update_bits(ctx->sysreg,
				driver_data->lcdblk_offset,
				0x1 << driver_data->lcdblk_bypass_shift,
				0x1 << driver_data->lcdblk_bypass_shift)) {
		DRM_DEV_ERROR(ctx->dev,
			      "Failed to update sysreg for bypass setting.\n");
		return;
	}

	/* TODO: When MIC is enabled for display path, the lcdblk_mic_bypass
	 * bit should be cleared.
	 */
	if (driver_data->has_mic_bypass && ctx->sysreg &&
	    regmap_update_bits(ctx->sysreg,
				driver_data->lcdblk_offset,
				0x1 << driver_data->lcdblk_mic_bypass_shift,
				0x1 << driver_data->lcdblk_mic_bypass_shift)) {
		DRM_DEV_ERROR(ctx->dev,
			      "Failed to update sysreg for bypass mic.\n");
		return;
	}

	/* setup horizontal and vertical display size. */
	val = VIDTCON2_LINEVAL(mode->vdisplay - 1) |
	       VIDTCON2_HOZVAL(mode->hdisplay - 1) |
	       VIDTCON2_LINEVAL_E(mode->vdisplay - 1) |
	       VIDTCON2_HOZVAL_E(mode->hdisplay - 1);
	writel(val, ctx->regs + driver_data->timing_base + VIDTCON2);

	fimd_setup_trigger(ctx);

	/*
	 * fields of register with prefix '_F' would be updated
	 * at vsync(same as dma start)
	 */
	val = ctx->vidcon0;
	val |= VIDCON0_ENVID | VIDCON0_ENVID_F;

	if (ctx->driver_data->has_clksel)
		val |= VIDCON0_CLKSEL_LCD;

	if (ctx->clkdiv > 1)
		val |= VIDCON0_CLKVAL_F(ctx->clkdiv - 1) | VIDCON0_CLKDIR;

	writel(val, ctx->regs + VIDCON0);
}

static void fimd_win_set_bldeq(struct fimd_context *ctx, unsigned int win,
			       unsigned int alpha, unsigned int pixel_alpha)
{
	u32 mask = BLENDEQ_A_FUNC_F(0xf) | BLENDEQ_B_FUNC_F(0xf);
	u32 val = 0;

	switch (pixel_alpha) {
	case DRM_MODE_BLEND_PIXEL_NONE:
	case DRM_MODE_BLEND_COVERAGE:
		val |= BLENDEQ_A_FUNC_F(BLENDEQ_ALPHA_A);
		val |= BLENDEQ_B_FUNC_F(BLENDEQ_ONE_MINUS_ALPHA_A);
		break;
	case DRM_MODE_BLEND_PREMULTI:
	default:
		if (alpha != DRM_BLEND_ALPHA_OPAQUE) {
			val |= BLENDEQ_A_FUNC_F(BLENDEQ_ALPHA0);
			val |= BLENDEQ_B_FUNC_F(BLENDEQ_ONE_MINUS_ALPHA_A);
		} else {
			val |= BLENDEQ_A_FUNC_F(BLENDEQ_ONE);
			val |= BLENDEQ_B_FUNC_F(BLENDEQ_ONE_MINUS_ALPHA_A);
		}
		break;
	}
	fimd_set_bits(ctx, BLENDEQx(win), mask, val);
}

static void fimd_win_set_bldmod(struct fimd_context *ctx, unsigned int win,
				unsigned int alpha, unsigned int pixel_alpha)
{
	u32 win_alpha_l = (alpha >> 8) & 0xf;
	u32 win_alpha_h = alpha >> 12;
	u32 val = 0;

	switch (pixel_alpha) {
	case DRM_MODE_BLEND_PIXEL_NONE:
		break;
	case DRM_MODE_BLEND_COVERAGE:
	case DRM_MODE_BLEND_PREMULTI:
	default:
		val |= WINCON1_ALPHA_SEL;
		val |= WINCON1_BLD_PIX;
		val |= WINCON1_ALPHA_MUL;
		break;
	}
	fimd_set_bits(ctx, WINCON(win), WINCONx_BLEND_MODE_MASK, val);

	/* OSD alpha */
	val = VIDISD14C_ALPHA0_R(win_alpha_h) |
		VIDISD14C_ALPHA0_G(win_alpha_h) |
		VIDISD14C_ALPHA0_B(win_alpha_h) |
		VIDISD14C_ALPHA1_R(0x0) |
		VIDISD14C_ALPHA1_G(0x0) |
		VIDISD14C_ALPHA1_B(0x0);
	writel(val, ctx->regs + VIDOSD_C(win));

	val = VIDW_ALPHA_R(win_alpha_l) | VIDW_ALPHA_G(win_alpha_l) |
		VIDW_ALPHA_B(win_alpha_l);
	writel(val, ctx->regs + VIDWnALPHA0(win));

	val = VIDW_ALPHA_R(0x0) | VIDW_ALPHA_G(0x0) |
		VIDW_ALPHA_B(0x0);
	writel(val, ctx->regs + VIDWnALPHA1(win));

	fimd_set_bits(ctx, BLENDCON, BLENDCON_NEW_MASK,
			BLENDCON_NEW_8BIT_ALPHA_VALUE);
}

static void fimd_win_set_pixfmt(struct fimd_context *ctx, unsigned int win,
				struct drm_framebuffer *fb, int width)
{
	struct exynos_drm_plane *plane = &ctx->planes[win];
	struct exynos_drm_plane_state *state =
		to_exynos_plane_state(plane->base.state);
	uint32_t pixel_format = fb->format->format;
	unsigned int alpha = state->base.alpha;
	u32 val = WINCONx_ENWIN;
	unsigned int pixel_alpha;

	if (fb->format->has_alpha)
		pixel_alpha = state->base.pixel_blend_mode;
	else
		pixel_alpha = DRM_MODE_BLEND_PIXEL_NONE;

	/*
	 * In case of s3c64xx, window 0 doesn't support alpha channel.
	 * So the request format is ARGB8888 then change it to XRGB8888.
	 */
	if (ctx->driver_data->has_limited_fmt && !win) {
		if (pixel_format == DRM_FORMAT_ARGB8888)
			pixel_format = DRM_FORMAT_XRGB8888;
	}

	switch (pixel_format) {
	case DRM_FORMAT_C8:
		val |= WINCON0_BPPMODE_8BPP_PALETTE;
		val |= WINCONx_BURSTLEN_8WORD;
		val |= WINCONx_BYTSWP;
		break;
	case DRM_FORMAT_XRGB1555:
	case DRM_FORMAT_XBGR1555:
		val |= WINCON0_BPPMODE_16BPP_1555;
		val |= WINCONx_HAWSWP;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_BGR565:
		val |= WINCON0_BPPMODE_16BPP_565;
		val |= WINCONx_HAWSWP;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_XBGR8888:
		val |= WINCON0_BPPMODE_24BPP_888;
		val |= WINCONx_WSWP;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_ABGR8888:
	default:
		val |= WINCON1_BPPMODE_25BPP_A1888;
		val |= WINCONx_WSWP;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	}

	switch (pixel_format) {
	case DRM_FORMAT_XBGR1555:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_BGR565:
		writel(WIN_RGB_ORDER_REVERSE, ctx->regs + WIN_RGB_ORDER(win));
		break;
	default:
		writel(WIN_RGB_ORDER_FORWARD, ctx->regs + WIN_RGB_ORDER(win));
		break;
	}

	/*
	 * Setting dma-burst to 16Word causes permanent tearing for very small
	 * buffers, e.g. cursor buffer. Burst Mode switching which based on
	 * plane size is not recommended as plane size varies alot towards the
	 * end of the screen and rapid movement causes unstable DMA, but it is
	 * still better to change dma-burst than displaying garbage.
	 */

	if (width < MIN_FB_WIDTH_FOR_16WORD_BURST) {
		val &= ~WINCONx_BURSTLEN_MASK;
		val |= WINCONx_BURSTLEN_4WORD;
	}
	fimd_set_bits(ctx, WINCON(win), ~WINCONx_BLEND_MODE_MASK, val);

	/* hardware window 0 doesn't support alpha channel. */
	if (win != 0) {
		fimd_win_set_bldmod(ctx, win, alpha, pixel_alpha);
		fimd_win_set_bldeq(ctx, win, alpha, pixel_alpha);
	}
}

static void fimd_win_set_colkey(struct fimd_context *ctx, unsigned int win)
{
	unsigned int keycon0 = 0, keycon1 = 0;

	keycon0 = ~(WxKEYCON0_KEYBL_EN | WxKEYCON0_KEYEN_F |
			WxKEYCON0_DIRCON) | WxKEYCON0_COMPKEY(0);

	keycon1 = WxKEYCON1_COLVAL(0xffffffff);

	writel(keycon0, ctx->regs + WKEYCON0_BASE(win));
	writel(keycon1, ctx->regs + WKEYCON1_BASE(win));
}

/**
 * fimd_shadow_protect_win() - disable updating values from shadow registers at vsync
 *
 * @ctx: local driver data
 * @win: window to protect registers for
 * @protect: 1 to protect (disable updates)
 */
static void fimd_shadow_protect_win(struct fimd_context *ctx,
				    unsigned int win, bool protect)
{
	u32 reg, bits, val;

	/*
	 * SHADOWCON/PRTCON register is used for enabling timing.
	 *
	 * for example, once only width value of a register is set,
	 * if the dma is started then fimd hardware could malfunction so
	 * with protect window setting, the register fields with prefix '_F'
	 * wouldn't be updated at vsync also but updated once unprotect window
	 * is set.
	 */

	if (ctx->driver_data->has_shadowcon) {
		reg = SHADOWCON;
		bits = SHADOWCON_WINx_PROTECT(win);
	} else {
		reg = PRTCON;
		bits = PRTCON_PROTECT;
	}

	val = readl(ctx->regs + reg);
	if (protect)
		val |= bits;
	else
		val &= ~bits;
	writel(val, ctx->regs + reg);
}

static void fimd_atomic_begin(struct exynos_drm_crtc *crtc)
{
	struct fimd_context *ctx = crtc->ctx;
	int i;

	if (ctx->suspended)
		return;

	for (i = 0; i < WINDOWS_NR; i++)
		fimd_shadow_protect_win(ctx, i, true);
}

static void fimd_atomic_flush(struct exynos_drm_crtc *crtc)
{
	struct fimd_context *ctx = crtc->ctx;
	int i;

	if (ctx->suspended)
		return;

	for (i = 0; i < WINDOWS_NR; i++)
		fimd_shadow_protect_win(ctx, i, false);

	fimd_arm_fifo_irq(ctx);

	exynos_crtc_handle_event(crtc);
}

static void fimd_update_plane(struct exynos_drm_crtc *crtc,
			      struct exynos_drm_plane *plane)
{
	struct exynos_drm_plane_state *state =
				to_exynos_plane_state(plane->base.state);
	struct fimd_context *ctx = crtc->ctx;
	struct drm_framebuffer *fb = state->base.fb;
	dma_addr_t dma_addr;
	unsigned long val, size, offset;
	unsigned int last_x, last_y, buf_offsize, line_size;
	unsigned int win = plane->index;
	unsigned int cpp = fb->format->cpp[0];
	unsigned int pitch = fb->pitches[0];

	if (ctx->suspended)
		return;

	offset = state->src.x * cpp;
	offset += state->src.y * pitch;

	/* buffer start address */
	dma_addr = exynos_drm_fb_dma_addr(fb, 0) + offset;
	val = (unsigned long)dma_addr;
	writel(val, ctx->regs + VIDWx_BUF_START(win, 0));

	/* buffer end address */
	size = pitch * state->crtc.h;
	val = (unsigned long)(dma_addr + size);
	writel(val, ctx->regs + VIDWx_BUF_END(win, 0));

	DRM_DEV_DEBUG_KMS(ctx->dev,
			  "start addr = 0x%lx, end addr = 0x%lx, size = 0x%lx\n",
			  (unsigned long)dma_addr, val, size);
	DRM_DEV_DEBUG_KMS(ctx->dev, "ovl_width = %d, ovl_height = %d\n",
			  state->crtc.w, state->crtc.h);

	/* buffer size */
	buf_offsize = pitch - (state->crtc.w * cpp);
	line_size = state->crtc.w * cpp;
	val = VIDW_BUF_SIZE_OFFSET(buf_offsize) |
		VIDW_BUF_SIZE_PAGEWIDTH(line_size) |
		VIDW_BUF_SIZE_OFFSET_E(buf_offsize) |
		VIDW_BUF_SIZE_PAGEWIDTH_E(line_size);
	writel(val, ctx->regs + VIDWx_BUF_SIZE(win, 0));

	/* OSD position */
	val = VIDOSDxA_TOPLEFT_X(state->crtc.x) |
		VIDOSDxA_TOPLEFT_Y(state->crtc.y) |
		VIDOSDxA_TOPLEFT_X_E(state->crtc.x) |
		VIDOSDxA_TOPLEFT_Y_E(state->crtc.y);
	writel(val, ctx->regs + VIDOSD_A(win));

	last_x = state->crtc.x + state->crtc.w;
	if (last_x)
		last_x--;
	last_y = state->crtc.y + state->crtc.h;
	if (last_y)
		last_y--;

	val = VIDOSDxB_BOTRIGHT_X(last_x) | VIDOSDxB_BOTRIGHT_Y(last_y) |
		VIDOSDxB_BOTRIGHT_X_E(last_x) | VIDOSDxB_BOTRIGHT_Y_E(last_y);

	writel(val, ctx->regs + VIDOSD_B(win));

	DRM_DEV_DEBUG_KMS(ctx->dev,
			  "osd pos: tx = %d, ty = %d, bx = %d, by = %d\n",
			  state->crtc.x, state->crtc.y, last_x, last_y);

	/* OSD size */
	if (win != 3 && win != 4) {
		u32 offset = VIDOSD_D(win);
		if (win == 0)
			offset = VIDOSD_C(win);
		val = state->crtc.w * state->crtc.h;
		writel(val, ctx->regs + offset);

		DRM_DEV_DEBUG_KMS(ctx->dev, "osd size = 0x%x\n",
				  (unsigned int)val);
	}

	fimd_win_set_pixfmt(ctx, win, fb, state->src.w);

	/* hardware window 0 doesn't support color key. */
	if (win != 0)
		fimd_win_set_colkey(ctx, win);

	fimd_enable_video_output(ctx, win, true);

	if (ctx->driver_data->has_shadowcon)
		fimd_enable_shadow_channel_path(ctx, win, true);

	if (ctx->i80_if)
		atomic_set(&ctx->win_updated, 1);
}

static void fimd_disable_win(struct fimd_context *ctx, unsigned int win)
{
	fimd_enable_video_output(ctx, win, false);

	if (ctx->driver_data->has_shadowcon)
		fimd_enable_shadow_channel_path(ctx, win, false);
}

static bool fimd_can_park_win(struct fimd_context *ctx, unsigned int win)
{
	return win != 0 && ctx->park_vaddr;
}

/*
 * Point @win at the transparent parking buffer and keep it enabled. All
 * registers written here are shadowed, so within a commit (shadow protect
 * held) the window switches from its old content to the parked state at
 * the next vsync, like any buffer change.
 */
static void fimd_park_win(struct fimd_context *ctx, unsigned int win)
{
	unsigned int last_x = PARK_WIDTH - 1, last_y = PARK_HEIGHT - 1;
	u32 val;

	writel(ctx->park_dma, ctx->regs + VIDWx_BUF_START(win, 0));
	writel(ctx->park_dma + PARK_SIZE, ctx->regs + VIDWx_BUF_END(win, 0));

	val = VIDW_BUF_SIZE_OFFSET(0) | VIDW_BUF_SIZE_PAGEWIDTH(PARK_PITCH) |
		VIDW_BUF_SIZE_OFFSET_E(0) | VIDW_BUF_SIZE_PAGEWIDTH_E(PARK_PITCH);
	writel(val, ctx->regs + VIDWx_BUF_SIZE(win, 0));

	writel(0, ctx->regs + VIDOSD_A(win));
	val = VIDOSDxB_BOTRIGHT_X(last_x) | VIDOSDxB_BOTRIGHT_Y(last_y) |
		VIDOSDxB_BOTRIGHT_X_E(last_x) | VIDOSDxB_BOTRIGHT_Y_E(last_y);
	writel(val, ctx->regs + VIDOSD_B(win));

	/* Window 0 is never parked, so windows 1 and 2 have VIDOSD_D. */
	if (win != 3 && win != 4)
		writel(PARK_WIDTH * PARK_HEIGHT, ctx->regs + VIDOSD_D(win));

	/* ARGB8888, all zero: premultiplied alpha blending leaves dst as is. */
	val = WINCONx_ENWIN | WINCON1_BPPMODE_25BPP_A1888 | WINCONx_WSWP |
		WINCONx_BURSTLEN_16WORD;
	fimd_set_bits(ctx, WINCON(win), ~WINCONx_BLEND_MODE_MASK, val);
	fimd_win_set_bldmod(ctx, win, DRM_BLEND_ALPHA_OPAQUE,
			    DRM_MODE_BLEND_PREMULTI);
	fimd_win_set_bldeq(ctx, win, DRM_BLEND_ALPHA_OPAQUE,
			   DRM_MODE_BLEND_PREMULTI);

	if (ctx->driver_data->has_shadowcon)
		fimd_enable_shadow_channel_path(ctx, win, true);
}

static void fimd_disable_plane(struct exynos_drm_crtc *crtc,
			       struct exynos_drm_plane *plane)
{
	struct fimd_context *ctx = crtc->ctx;
	unsigned int win = plane->index;

	if (ctx->suspended)
		return;

	if (fimd_can_park_win(ctx, win))
		fimd_park_win(ctx, win);
	else
		fimd_disable_win(ctx, win);
}

static void fimd_restart_scanout(struct fimd_context *ctx, u32 vidcon1)
{
	u32 vidcon0 = readl(ctx->regs + VIDCON0);

	writel(vidcon0 & ~(VIDCON0_ENVID | VIDCON0_ENVID_F),
	       ctx->regs + VIDCON0);
	writel(vidcon0 | VIDCON0_ENVID | VIDCON0_ENVID_F, ctx->regs + VIDCON0);

	/* A hang leaves the FIFO underrun flag pending. */
	writel(VIDINTCON1_INT_FIFO, ctx->regs + VIDINTCON1);

	dev_warn(ctx->dev,
		 "scanout hung at line %u (vstatus %u), restarted (#%d)\n",
		 VIDCON1_LINECNT_GET(vidcon1),
		 (vidcon1 & VIDCON1_VSTATUS_MASK) >> VIDCON1_VSTATUS_SHIFT,
		 atomic_inc_return(&ctx->hang_recoveries));
}

static void fimd_hang_check(struct work_struct *work)
{
	struct fimd_context *ctx = container_of(to_delayed_work(work),
						struct fimd_context, hang_work);
	void __iomem *vidcon1 = ctx->regs + ctx->driver_data->timing_base +
				VIDCON1;
	u32 first;
	int i;

	/* Not active means the display is off; atomic_enable requeues. */
	if (pm_runtime_get_if_active(ctx->dev, true) <= 0)
		return;

	first = readl(vidcon1);
	for (i = 0; i < FIMD_HANG_SAMPLES; i++) {
		usleep_range(1000, 1200);
		if (readl(vidcon1) != first)
			break;
	}

	if (i == FIMD_HANG_SAMPLES)
		fimd_restart_scanout(ctx, first);

	pm_runtime_put(ctx->dev);

	queue_delayed_work(system_wq, &ctx->hang_work,
			   msecs_to_jiffies(FIMD_HANG_CHECK_MS));
}

static void fimd_atomic_enable(struct exynos_drm_crtc *crtc)
{
	struct fimd_context *ctx = crtc->ctx;
	unsigned int win;

	if (!ctx->suspended)
		return;

	ctx->suspended = false;

	if (pm_runtime_resume_and_get(ctx->dev) < 0) {
		dev_warn(ctx->dev, "failed to enable FIMD device.\n");
		return;
	}

	/* if vblank was enabled status, enable it again. */
	if (test_and_clear_bit(0, &ctx->irq_flags))
		fimd_enable_vblank(ctx->crtc);

	/*
	 * Park windows 1-4 before video output starts, so they are enabled
	 * with valid registers from the first frame on and plane updates
	 * never have to switch ENWIN on mid-frame.
	 */
	for (win = 0; win < WINDOWS_NR; win++)
		if (fimd_can_park_win(ctx, win))
			fimd_park_win(ctx, win);

	fimd_commit(ctx->crtc);

	if (!ctx->i80_if)
		queue_delayed_work(system_wq, &ctx->hang_work,
				   msecs_to_jiffies(FIMD_HANG_CHECK_MS));
}

static void fimd_atomic_disable(struct exynos_drm_crtc *crtc)
{
	struct fimd_context *ctx = crtc->ctx;
	int i;

	if (ctx->suspended)
		return;

	/* Before anything is switched off; the work requeues itself. */
	cancel_delayed_work_sync(&ctx->hang_work);

	fimd_disarm_fifo_irq(ctx);

	/*
	 * We need to make sure that all windows are disabled before we
	 * suspend that connector. Otherwise we might try to scan from
	 * a destroyed buffer later.
	 */
	for (i = 0; i < WINDOWS_NR; i++)
		fimd_disable_win(ctx, i);

	fimd_enable_vblank(crtc);
	fimd_wait_for_vblank(crtc);
	fimd_disable_vblank(crtc);

	writel(0, ctx->regs + VIDCON0);

	pm_runtime_put_sync(ctx->dev);
	ctx->suspended = true;
}

static void fimd_trigger(struct device *dev)
{
	struct fimd_context *ctx = dev_get_drvdata(dev);
	const struct fimd_driver_data *driver_data = ctx->driver_data;
	void *timing_base = ctx->regs + driver_data->timing_base;
	u32 reg;

	 /*
	  * Skips triggering if in triggering state, because multiple triggering
	  * requests can cause panel reset.
	  */
	if (atomic_read(&ctx->triggering))
		return;

	/* Enters triggering mode */
	atomic_set(&ctx->triggering, 1);

	reg = readl(timing_base + TRIGCON);
	reg |= (TRGMODE_ENABLE | SWTRGCMD_ENABLE);
	writel(reg, timing_base + TRIGCON);

	/*
	 * Exits triggering mode if vblank is not enabled yet, because when the
	 * VIDINTCON0 register is not set, it can not exit from triggering mode.
	 */
	if (!test_bit(0, &ctx->irq_flags))
		atomic_set(&ctx->triggering, 0);
}

static void fimd_te_handler(struct exynos_drm_crtc *crtc)
{
	struct fimd_context *ctx = crtc->ctx;
	u32 trg_type = ctx->driver_data->trg_type;

	/* Checks the crtc is detached already from encoder */
	if (!ctx->drm_dev)
		return;

	if (trg_type == I80_HW_TRG)
		goto out;

	/*
	 * If there is a page flip request, triggers and handles the page flip
	 * event so that current fb can be updated into panel GRAM.
	 */
	if (atomic_add_unless(&ctx->win_updated, -1, 0))
		fimd_trigger(ctx->dev);

out:
	/* Wakes up vsync event queue */
	if (atomic_read(&ctx->wait_vsync_event)) {
		atomic_set(&ctx->wait_vsync_event, 0);
		wake_up(&ctx->wait_vsync_queue);
	}

	if (test_bit(0, &ctx->irq_flags))
		drm_crtc_handle_vblank(&ctx->crtc->base);
}

static void fimd_dp_clock_enable(struct exynos_drm_clk *clk, bool enable)
{
	struct fimd_context *ctx = container_of(clk, struct fimd_context,
						dp_clk);
	u32 val = enable ? DP_MIE_CLK_DP_ENABLE : DP_MIE_CLK_DISABLE;

	if (enable == ctx->dp_clk_enabled)
		return;

	if (enable)
		pm_runtime_resume_and_get(ctx->dev);

	ctx->dp_clk_enabled = enable;
	writel(val, ctx->regs + DP_MIE_CLKCON);

	if (!enable)
		pm_runtime_put(ctx->dev);
}

static const struct exynos_drm_crtc_ops fimd_crtc_ops = {
	.atomic_enable = fimd_atomic_enable,
	.atomic_disable = fimd_atomic_disable,
	.enable_vblank = fimd_enable_vblank,
	.disable_vblank = fimd_disable_vblank,
	.atomic_begin = fimd_atomic_begin,
	.update_plane = fimd_update_plane,
	.disable_plane = fimd_disable_plane,
	.atomic_flush = fimd_atomic_flush,
	.atomic_check = fimd_atomic_check,
	.te_handler = fimd_te_handler,
};

static irqreturn_t fimd_irq_handler(int irq, void *dev_id)
{
	struct fimd_context *ctx = (struct fimd_context *)dev_id;
	u32 val, clear_bit;

	val = readl(ctx->regs + VIDINTCON1);

	if (val & VIDINTCON1_INT_FIFO) {
		u32 enwin = 0;
		int i;

		fimd_disarm_fifo_irq(ctx);
		writel(VIDINTCON1_INT_FIFO, ctx->regs + VIDINTCON1);

		for (i = 0; i < WINDOWS_NR; i++)
			if (readl(ctx->regs + WINCON(i)) & WINCONx_ENWIN)
				enwin |= BIT(i);

		dev_warn_ratelimited(ctx->dev,
				     "FIFO underrun #%d, enabled windows 0x%02x\n",
				     atomic_inc_return(&ctx->fifo_underruns),
				     enwin);
	}

	clear_bit = ctx->i80_if ? VIDINTCON1_INT_I80 : VIDINTCON1_INT_FRAME;
	if (val & clear_bit)
		writel(clear_bit, ctx->regs + VIDINTCON1);

	/* check the crtc is detached already from encoder */
	if (!ctx->drm_dev)
		goto out;

	/* A FIFO-only interrupt is not a vblank. */
	if (!ctx->i80_if && !(val & VIDINTCON1_INT_FRAME))
		goto out;

	if (!ctx->i80_if)
		drm_crtc_handle_vblank(&ctx->crtc->base);

	if (ctx->i80_if) {
		/* Exits triggering mode */
		atomic_set(&ctx->triggering, 0);
	} else {
		/* set wait vsync event to zero and wake up queue. */
		if (atomic_read(&ctx->wait_vsync_event)) {
			atomic_set(&ctx->wait_vsync_event, 0);
			wake_up(&ctx->wait_vsync_queue);
		}
	}

out:
	return IRQ_HANDLED;
}

/* Free the debug test buffer; caller holds testbuf_lock. */
static void fimd_testbuf_free(struct fimd_context *ctx)
{
	if (!ctx->testbuf_vaddr)
		return;

	dma_free_attrs(to_dma_dev(ctx->drm_dev), ctx->testbuf_size,
		       ctx->testbuf_vaddr, ctx->testbuf_dma,
		       DMA_ATTR_WRITE_COMBINE);
	ctx->testbuf_vaddr = NULL;
	ctx->testbuf_size = 0;
}

static int fimd_bind(struct device *dev, struct device *master, void *data)
{
	struct fimd_context *ctx = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct exynos_drm_plane *exynos_plane;
	unsigned int i;
	int ret;

	ctx->drm_dev = drm_dev;

	for (i = 0; i < WINDOWS_NR; i++) {
		if (ctx->driver_data->has_bgr_support) {
			ctx->configs[i].pixel_formats = fimd_extended_formats;
			ctx->configs[i].num_pixel_formats = ARRAY_SIZE(fimd_extended_formats);
		} else {
			ctx->configs[i].pixel_formats = fimd_formats;
			ctx->configs[i].num_pixel_formats = ARRAY_SIZE(fimd_formats);
		}

		ctx->configs[i].zpos = i;
		ctx->configs[i].type = fimd_win_types[i];
		ctx->configs[i].capabilities = capabilities[i];
		ret = exynos_plane_init(drm_dev, &ctx->planes[i], i,
					&ctx->configs[i]);
		if (ret)
			return ret;
	}

	exynos_plane = &ctx->planes[DEFAULT_WIN];
	ctx->crtc = exynos_drm_crtc_create(drm_dev, &exynos_plane->base,
			EXYNOS_DISPLAY_TYPE_LCD, &fimd_crtc_ops, ctx);
	if (IS_ERR(ctx->crtc))
		return PTR_ERR(ctx->crtc);

	if (ctx->driver_data->has_dp_clk) {
		ctx->dp_clk.enable = fimd_dp_clock_enable;
		ctx->crtc->pipe_clk = &ctx->dp_clk;
	}

	if (ctx->encoder)
		exynos_dpi_bind(drm_dev, ctx->encoder);

	if (is_drm_iommu_supported(drm_dev)) {
		int ret;

		ret = fimd_clear_channels(ctx->crtc);
		if (ret < 0)
			return ret;
	}

	ret = exynos_drm_register_dma(drm_dev, dev, &ctx->dma_priv);
	if (ret)
		return ret;

	/* Without the parking buffer, disabled windows are switched off. */
	ctx->park_vaddr = dma_alloc_attrs(to_dma_dev(drm_dev), PARK_SIZE,
					  &ctx->park_dma, GFP_KERNEL,
					  DMA_ATTR_WRITE_COMBINE);
	if (ctx->park_vaddr)
		memset(ctx->park_vaddr, 0, PARK_SIZE);
	else
		dev_warn(dev, "no window parking buffer, windows will toggle\n");

	return 0;
}

static void fimd_unbind(struct device *dev, struct device *master,
			void *data)
{
	struct fimd_context *ctx = dev_get_drvdata(dev);

	fimd_atomic_disable(ctx->crtc);

	mutex_lock(&ctx->testbuf_lock);
	fimd_testbuf_free(ctx);
	mutex_unlock(&ctx->testbuf_lock);

	if (ctx->park_vaddr) {
		dma_free_attrs(to_dma_dev(ctx->drm_dev), PARK_SIZE,
			       ctx->park_vaddr, ctx->park_dma,
			       DMA_ATTR_WRITE_COMBINE);
		ctx->park_vaddr = NULL;
	}

	exynos_drm_unregister_dma(ctx->drm_dev, ctx->dev, &ctx->dma_priv);

	if (ctx->encoder)
		exynos_dpi_remove(ctx->encoder);
}

static const struct component_ops fimd_component_ops = {
	.bind	= fimd_bind,
	.unbind = fimd_unbind,
};

/* Map FIMD's System MMU for the regs debug dump; optional. */
static void fimd_map_sysmmu(struct fimd_context *ctx)
{
	struct device_node *np;
	struct resource res;

	np = of_parse_phandle(ctx->dev->of_node, "iommus", 0);
	if (!np)
		return;

	if (!of_address_to_resource(np, 0, &res))
		ctx->sysmmu_regs = devm_ioremap(ctx->dev, res.start,
						resource_size(&res));

	ctx->sysmmu_clk = of_clk_get_by_name(np, "sysmmu");
	if (IS_ERR(ctx->sysmmu_clk))
		ctx->sysmmu_clk = NULL;

	of_node_put(np);
}

static int fimd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fimd_context *ctx;
	struct device_node *i80_if_timings;
	struct resource *res;
	int ret;

	if (!dev->of_node)
		return -ENODEV;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	ctx->suspended = true;
	ctx->driver_data = of_device_get_match_data(dev);

	if (of_property_read_bool(dev->of_node, "samsung,invert-vden"))
		ctx->vidcon1 |= VIDCON1_INV_VDEN;
	if (of_property_read_bool(dev->of_node, "samsung,invert-vclk"))
		ctx->vidcon1 |= VIDCON1_INV_VCLK;

	i80_if_timings = of_get_child_by_name(dev->of_node, "i80-if-timings");
	if (i80_if_timings) {
		u32 val;

		ctx->i80_if = true;

		if (ctx->driver_data->has_vidoutcon)
			ctx->vidout_con |= VIDOUT_CON_F_I80_LDI0;
		else
			ctx->vidcon0 |= VIDCON0_VIDOUT_I80_LDI0;
		/*
		 * The user manual describes that this "DSI_EN" bit is required
		 * to enable I80 24-bit data interface.
		 */
		ctx->vidcon0 |= VIDCON0_DSI_EN;

		if (of_property_read_u32(i80_if_timings, "cs-setup", &val))
			val = 0;
		ctx->i80ifcon = LCD_CS_SETUP(val);
		if (of_property_read_u32(i80_if_timings, "wr-setup", &val))
			val = 0;
		ctx->i80ifcon |= LCD_WR_SETUP(val);
		if (of_property_read_u32(i80_if_timings, "wr-active", &val))
			val = 1;
		ctx->i80ifcon |= LCD_WR_ACTIVE(val);
		if (of_property_read_u32(i80_if_timings, "wr-hold", &val))
			val = 0;
		ctx->i80ifcon |= LCD_WR_HOLD(val);
	}
	of_node_put(i80_if_timings);

	ctx->sysreg = syscon_regmap_lookup_by_phandle(dev->of_node,
							"samsung,sysreg");
	if (IS_ERR(ctx->sysreg)) {
		dev_warn(dev, "failed to get system register.\n");
		ctx->sysreg = NULL;
	}

	ctx->bus_clk = devm_clk_get(dev, "fimd");
	if (IS_ERR(ctx->bus_clk)) {
		dev_err(dev, "failed to get bus clock\n");
		return PTR_ERR(ctx->bus_clk);
	}

	ctx->lcd_clk = devm_clk_get(dev, "sclk_fimd");
	if (IS_ERR(ctx->lcd_clk)) {
		dev_err(dev, "failed to get lcd clock\n");
		return PTR_ERR(ctx->lcd_clk);
	}

	ctx->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(ctx->regs))
		return PTR_ERR(ctx->regs);
	ctx->regs_size = resource_size(res);

	fimd_map_sysmmu(ctx);

	ret = platform_get_irq_byname(pdev, ctx->i80_if ? "lcd_sys" : "vsync");
	if (ret < 0)
		return ret;

	ret = devm_request_irq(dev, ret, fimd_irq_handler, 0, "drm_fimd", ctx);
	if (ret) {
		dev_err(dev, "irq request failed.\n");
		return ret;
	}

	init_waitqueue_head(&ctx->wait_vsync_queue);
	mutex_init(&ctx->testbuf_lock);
	INIT_DELAYED_WORK(&ctx->hang_work, fimd_hang_check);
	atomic_set(&ctx->wait_vsync_event, 0);

	platform_set_drvdata(pdev, ctx);

	ctx->encoder = exynos_dpi_probe(dev);
	if (IS_ERR(ctx->encoder))
		return PTR_ERR(ctx->encoder);

	pm_runtime_enable(dev);

	ret = component_add(dev, &fimd_component_ops);
	if (ret)
		goto err_disable_pm_runtime;

	return ret;

err_disable_pm_runtime:
	pm_runtime_disable(dev);

	return ret;
}

static int fimd_remove(struct platform_device *pdev)
{
	struct fimd_context *ctx = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);

	if (ctx->sysmmu_clk)
		clk_put(ctx->sysmmu_clk);

	component_del(&pdev->dev, &fimd_component_ops);

	return 0;
}

#ifdef CONFIG_PM
static int exynos_fimd_suspend(struct device *dev)
{
	struct fimd_context *ctx = dev_get_drvdata(dev);

	clk_disable_unprepare(ctx->lcd_clk);
	clk_disable_unprepare(ctx->bus_clk);

	return 0;
}

static int exynos_fimd_resume(struct device *dev)
{
	struct fimd_context *ctx = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(ctx->bus_clk);
	if (ret < 0) {
		DRM_DEV_ERROR(dev,
			      "Failed to prepare_enable the bus clk [%d]\n",
			      ret);
		return ret;
	}

	ret = clk_prepare_enable(ctx->lcd_clk);
	if  (ret < 0) {
		DRM_DEV_ERROR(dev,
			      "Failed to prepare_enable the lcd clk [%d]\n",
			      ret);
		return ret;
	}

	return 0;
}
#endif

static const struct dev_pm_ops exynos_fimd_pm_ops = {
	SET_RUNTIME_PM_OPS(exynos_fimd_suspend, exynos_fimd_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static ssize_t fifo_underruns_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct fimd_context *ctx = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", atomic_read(&ctx->fifo_underruns));
}
static DEVICE_ATTR_RO(fifo_underruns);

static ssize_t hang_recoveries_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct fimd_context *ctx = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", atomic_read(&ctx->hang_recoveries));
}
static DEVICE_ATTR_RO(hang_recoveries);

/* System MMU v1-v3 registers, see drivers/iommu/exynos-iommu.c */
#define SYSMMU_CTRL		0x000
#define SYSMMU_CFG		0x004
#define SYSMMU_STATUS		0x008
#define SYSMMU_PT_BASE		0x014
#define SYSMMU_INT_STATUS	0x018
#define SYSMMU_PAGE_FAULT	0x024
#define SYSMMU_AW_FAULT		0x028
#define SYSMMU_AR_FAULT		0x02c
#define SYSMMU_DEFAULT_SLAVE	0x030
#define SYSMMU_VERSION		0x034

/* SHD_VIDWxxADD0/1/2: the buffer registers' latched values, at +0x4000. */
#define FIMD_ACTIVE(reg)	((reg) + 0x4000)

/*
 * Debug dump of the FIMD and FIMD System MMU state, readable while the
 * display is stuck: whether scanout still runs (line counter), pending
 * interrupts, and per window the programmed vs. latched setup.
 */
static ssize_t regs_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct fimd_context *ctx = dev_get_drvdata(dev);
	void __iomem *regs = ctx->regs;
	void __iomem *timing = regs + ctx->driver_data->timing_base;
	unsigned int win;
	u32 vidcon1, vidcon1_later;
	int len = 0;

	if (pm_runtime_get_if_active(dev, true) <= 0)
		return sysfs_emit(buf, "suspended\n");

	vidcon1 = readl(timing + VIDCON1);
	udelay(500);
	vidcon1_later = readl(timing + VIDCON1);

	len += sysfs_emit_at(buf, len, "VIDCON0    %08x\n",
			     readl(regs + VIDCON0));
	len += sysfs_emit_at(buf, len,
			     "VIDCON1    %08x line %u vstatus %u, 500us later %08x line %u vstatus %u\n",
			     vidcon1, VIDCON1_LINECNT_GET(vidcon1),
			     (vidcon1 & VIDCON1_VSTATUS_MASK) >> VIDCON1_VSTATUS_SHIFT,
			     vidcon1_later, VIDCON1_LINECNT_GET(vidcon1_later),
			     (vidcon1_later & VIDCON1_VSTATUS_MASK) >>
			     VIDCON1_VSTATUS_SHIFT);
	len += sysfs_emit_at(buf, len, "VIDINTCON0 %08x VIDINTCON1 %08x\n",
			     readl(regs + VIDINTCON0),
			     readl(regs + VIDINTCON1));
	len += sysfs_emit_at(buf, len, "SHADOWCON  %08x BLENDCON %08x\n",
			     readl(regs + SHADOWCON), readl(regs + BLENDCON));

	len += sysfs_emit_at(buf, len,
			     "win WINCON OSD_A OSD_B, programmed/latched: START END SIZE\n");
	for (win = 0; win < WINDOWS_NR; win++) {
		u32 reg[] = {
			VIDWx_BUF_START(win, 0), VIDWx_BUF_END(win, 0),
			VIDWx_BUF_SIZE(win, 0),
		};
		unsigned int i;

		len += sysfs_emit_at(buf, len, "win%u %08x %08x %08x", win,
				     readl(regs + WINCON(win)),
				     readl(regs + VIDOSD_A(win)),
				     readl(regs + VIDOSD_B(win)));
		for (i = 0; i < ARRAY_SIZE(reg); i++)
			len += sysfs_emit_at(buf, len, " %08x/%08x",
					     readl(regs + reg[i]),
					     readl(regs + FIMD_ACTIVE(reg[i])));
		len += sysfs_emit_at(buf, len, "\n");
	}

	if (ctx->sysmmu_regs &&
	    (!ctx->sysmmu_clk || !clk_prepare_enable(ctx->sysmmu_clk))) {
		void __iomem *mmu = ctx->sysmmu_regs;

		len += sysfs_emit_at(buf, len,
				     "SYSMMU CTRL %08x CFG %08x STATUS %08x PT_BASE %08x INT_STATUS %08x VERSION %08x\n",
				     readl(mmu + SYSMMU_CTRL),
				     readl(mmu + SYSMMU_CFG),
				     readl(mmu + SYSMMU_STATUS),
				     readl(mmu + SYSMMU_PT_BASE),
				     readl(mmu + SYSMMU_INT_STATUS),
				     readl(mmu + SYSMMU_VERSION));
		len += sysfs_emit_at(buf, len,
				     "SYSMMU PAGE_FAULT %08x AW_FAULT %08x AR_FAULT %08x DEFAULT_SLAVE %08x\n",
				     readl(mmu + SYSMMU_PAGE_FAULT),
				     readl(mmu + SYSMMU_AW_FAULT),
				     readl(mmu + SYSMMU_AR_FAULT),
				     readl(mmu + SYSMMU_DEFAULT_SLAVE));
		if (ctx->sysmmu_clk)
			clk_disable_unprepare(ctx->sysmmu_clk);
	}

	pm_runtime_put(dev);

	return len;
}
/*
 * Debug: "<offset> <value>" (hex) writes one FIMD register, for simulating
 * hangs and testing recovery. Bypasses DRM, so only touch parked windows.
 */
static ssize_t regs_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct fimd_context *ctx = dev_get_drvdata(dev);
	u32 offset, val;

	if (sscanf(buf, "%x %x", &offset, &val) != 2)
		return -EINVAL;

	if (offset % 4 || offset >= ctx->regs_size)
		return -EINVAL;

	if (pm_runtime_get_if_active(dev, true) <= 0)
		return -EAGAIN;

	writel(val, ctx->regs + offset);
	dev_info(dev, "debug write %05x = %08x\n", offset, val);

	pm_runtime_put(dev);

	return count;
}
static DEVICE_ATTR_RW(regs);

/*
 * Debug: a zeroed (transparent) buffer to point a parked window at with
 * the regs file, for reproducing scanout hangs with real-sized windows.
 * Write "<size>" (hex bytes) to allocate, "0" to free; read for its address.
 */
static ssize_t testbuf_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct fimd_context *ctx = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&ctx->testbuf_lock);
	if (ctx->testbuf_vaddr)
		len = sysfs_emit(buf, "%08llx %zx\n",
				 (unsigned long long)ctx->testbuf_dma,
				 ctx->testbuf_size);
	else
		len = sysfs_emit(buf, "none\n");
	mutex_unlock(&ctx->testbuf_lock);

	return len;
}

/* True if any window's programmed or latched buffer lies in the test buffer. */
static bool fimd_testbuf_in_use(struct fimd_context *ctx)
{
	dma_addr_t start = ctx->testbuf_dma;
	dma_addr_t end = start + ctx->testbuf_size;
	unsigned int win;
	bool busy = false;

	if (pm_runtime_get_if_active(ctx->dev, true) <= 0)
		return false;

	for (win = 0; win < WINDOWS_NR && !busy; win++) {
		u32 prog = readl(ctx->regs + VIDWx_BUF_START(win, 0));
		u32 latched = readl(ctx->regs +
				    FIMD_ACTIVE(VIDWx_BUF_START(win, 0)));

		busy = (prog >= start && prog < end) ||
		       (latched >= start && latched < end);
	}

	pm_runtime_put(ctx->dev);

	return busy;
}

static ssize_t testbuf_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct fimd_context *ctx = dev_get_drvdata(dev);
	unsigned long size;
	int ret = 0;

	if (kstrtoul(buf, 16, &size))
		return -EINVAL;

	if (!ctx->drm_dev)
		return -ENODEV;

	mutex_lock(&ctx->testbuf_lock);

	if (ctx->testbuf_vaddr && fimd_testbuf_in_use(ctx)) {
		ret = -EBUSY;
		goto out;
	}
	fimd_testbuf_free(ctx);

	if (size) {
		size = PAGE_ALIGN(size);
		ctx->testbuf_vaddr = dma_alloc_attrs(to_dma_dev(ctx->drm_dev),
						     size, &ctx->testbuf_dma,
						     GFP_KERNEL,
						     DMA_ATTR_WRITE_COMBINE);
		if (!ctx->testbuf_vaddr) {
			ret = -ENOMEM;
			goto out;
		}
		memset(ctx->testbuf_vaddr, 0, size);
		ctx->testbuf_size = size;
		dev_info(dev, "debug test buffer %08llx size %lx\n",
			 (unsigned long long)ctx->testbuf_dma, size);
	}

out:
	mutex_unlock(&ctx->testbuf_lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(testbuf);

static struct attribute *fimd_attrs[] = {
	&dev_attr_fifo_underruns.attr,
	&dev_attr_hang_recoveries.attr,
	&dev_attr_regs.attr,
	&dev_attr_testbuf.attr,
	NULL,
};
ATTRIBUTE_GROUPS(fimd);

struct platform_driver fimd_driver = {
	.probe		= fimd_probe,
	.remove		= fimd_remove,
	.driver		= {
		.name	= "exynos4-fb",
		.owner	= THIS_MODULE,
		.dev_groups = fimd_groups,
		.pm	= &exynos_fimd_pm_ops,
		.of_match_table = fimd_driver_dt_match,
	},
};
