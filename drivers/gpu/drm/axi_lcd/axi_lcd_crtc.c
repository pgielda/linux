/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/slab.h>
#include <linux/delay.h>


#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "axi_lcd_crtc.h"
#include "axi_lcd_drv.h"
#include "axi_lcd_encoder.h"

struct axi_lcd_crtc {
	struct drm_crtc drm_crtc;
	int mode;
};

static inline struct axi_lcd_crtc *to_axi_lcd_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct axi_lcd_crtc, drm_crtc);
}

#define DMA_BASE		0x10000
#define DMA_CONTROL 		(DMA_BASE + 0)
#define DMA_STATUS 		(DMA_BASE + 0x04)
#define DMA_PARK_POINTER 	(DMA_BASE + 0x28)
#define DMA_VSIZE 		(DMA_BASE + 0x50)
#define DMA_HSIZE 		(DMA_BASE + 0x54)
#define DMA_STRIDE 		(DMA_BASE + 0x58)
#define DMA_BUF_ADDR0 		(DMA_BASE + 0x5C)
#define DMA_BUF_ADDR1 		(DMA_BASE + 0x60)

int first = 0;

static int axi_lcd_crtc_update(struct drm_crtc *crtc)
{
	struct axi_lcd_crtc *axi_lcd_crtc = to_axi_lcd_crtc(crtc);
	struct drm_display_mode *mode = &crtc->mode;
	struct drm_framebuffer *fb = crtc->fb;
	struct drm_gem_cma_object *obj;
	unsigned int reg;
	unsigned int bufno;
	unsigned int bufaddr[2];

	struct axi_lcd_private *private  = (struct axi_lcd_private *) crtc->dev->dev_private;

	if (!mode || !fb)
		return -EINVAL;

	if (axi_lcd_crtc->mode == DRM_MODE_DPMS_ON) {
		obj = drm_fb_cma_get_gem_obj(fb, 0);
		if (!obj)
			return -EINVAL;

		if (first < 2) {
			first++;

			bufaddr[0] = obj->paddr;
			bufaddr[1] = obj->paddr + (crtc->x * fb->bits_per_pixel / 8 + mode->vdisplay * fb->pitches[0]);

			//halt dma (just in case)
			reg = readl(private->base + DMA_CONTROL);
			reg &= ~(0x01);
			//disable circular mode
			reg &= ~(0x02);
			writel(reg, private->base + DMA_CONTROL);

			//set first buffer as the one which is displayed
			reg = readl(private->base + DMA_PARK_POINTER);
			reg &= ~(0x1f);
			reg |= 0x00; // buffer0
			writel(reg, private->base + DMA_PARK_POINTER);

			//set the buffer pointers
			writel(bufaddr[0], private->base + DMA_BUF_ADDR0);
			writel(bufaddr[1], private->base + DMA_BUF_ADDR1);

			//start dma
			reg = readl(private->base + DMA_CONTROL);
			reg |= 0x01;
			writel(reg, private->base + DMA_CONTROL);

			writel(fb->pitches[0], private->base + DMA_STRIDE);
			writel(mode->hdisplay * fb->bits_per_pixel / 8, private->base + DMA_HSIZE);
			//set vsize (this starts the transfer)
			writel(mode->vdisplay, private->base + DMA_VSIZE);
			return 0;
		}

		bufno = (crtc->y == 0) ? 0 : 1;

		reg = readl(private->base + DMA_PARK_POINTER);
                reg &= ~(0x1f);
                reg |= bufno;
		writel(reg, private->base + DMA_PARK_POINTER);
		//wait till the frame is being operated
		reg = readl(private->base + DMA_PARK_POINTER);
		while( ((reg & 0x001f0000) >> 16) != bufno) {
			usleep_range(1,2);
			reg = readl(private->base + DMA_PARK_POINTER);
		}
	}
	return 0;
}

static void axi_lcd_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct axi_lcd_crtc *axi_lcd_crtc = to_axi_lcd_crtc(crtc);

	if (axi_lcd_crtc->mode != mode) {
		axi_lcd_crtc->mode = mode;
		//printk(KERN_ERR"%s: calling crtc_update \n", __func__);
		axi_lcd_crtc_update(crtc);
	}
}

static void axi_lcd_crtc_prepare(struct drm_crtc *crtc)
{
	struct axi_lcd_crtc *axi_lcd_crtc = to_axi_lcd_crtc(crtc);
	//printk(KERN_ERR"%s: calling crtc_update \n", __func__);
}

static void axi_lcd_crtc_commit(struct drm_crtc *crtc)
{
	struct axi_lcd_crtc *axi_lcd_crtc = to_axi_lcd_crtc(crtc);

	axi_lcd_crtc->mode = DRM_MODE_DPMS_ON;
	//printk(KERN_ERR"%s: calling crtc_update \n", __func__);
	axi_lcd_crtc_update(crtc);
}

static bool axi_lcd_crtc_mode_fixup(struct drm_crtc *crtc,
	const struct drm_display_mode *mode,
	struct drm_display_mode *adjusted_mode)
{
	return true;
}

static int axi_lcd_crtc_mode_set(struct drm_crtc *crtc,
	struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode,
	int x, int y, struct drm_framebuffer *old_fb)
{
	/* We do everything in commit() */
	return 0;
}

static int axi_lcd_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
	struct drm_framebuffer *old_fb)
{
	//printk(KERN_ERR"%s: calling crtc_update x= %d, y=%d \n", __func__, x, y);
	return axi_lcd_crtc_update(crtc);
}

static void axi_lcd_crtc_load_lut(struct drm_crtc *crtc)
{
}

static struct drm_crtc_helper_funcs axi_lcd_crtc_helper_funcs = {
	.dpms		= axi_lcd_crtc_dpms,
	.prepare	= axi_lcd_crtc_prepare,
	.commit		= axi_lcd_crtc_commit,
	.mode_fixup	= axi_lcd_crtc_mode_fixup,
	.mode_set	= axi_lcd_crtc_mode_set,
	.mode_set_base	= axi_lcd_crtc_mode_set_base,
	.load_lut	= axi_lcd_crtc_load_lut,
};

static void axi_lcd_crtc_destroy(struct drm_crtc *crtc)
{
	struct axi_lcd_crtc *axi_lcd_crtc = to_axi_lcd_crtc(crtc);

	drm_crtc_cleanup(crtc);
	kfree(axi_lcd_crtc);
}

static struct drm_crtc_funcs axi_lcd_crtc_funcs = {
	.set_config	= drm_crtc_helper_set_config,
	.destroy	= axi_lcd_crtc_destroy,
};

struct drm_crtc *axi_lcd_crtc_create(struct drm_device *dev)
{
	struct axi_lcd_private *p = dev->dev_private;
	struct axi_lcd_crtc *axi_lcd_crtc;
	struct drm_crtc *crtc;

	axi_lcd_crtc = kzalloc(sizeof(*axi_lcd_crtc), GFP_KERNEL);
	if (!axi_lcd_crtc) {
		DRM_ERROR("failed to allocate axi_lcd crtc\n");
		return ERR_PTR(-ENOMEM);
	}

	crtc = &axi_lcd_crtc->drm_crtc;

	drm_crtc_init(dev, crtc, &axi_lcd_crtc_funcs);
	drm_crtc_helper_add(crtc, &axi_lcd_crtc_helper_funcs);

	return crtc;
}
