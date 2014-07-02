/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#ifndef _AXI_LCD_DRV_H_
#define _AXI_LCD_DRV_H_

#include <drm/drm.h>
#include <drm/drm_fb_cma_helper.h>
#include <linux/of.h>
#include <linux/clk.h>

struct xlnx_pcm_dma_params {
	struct device_node *of_node;
	int chan_id;
};

struct axi_lcd_encoder;

struct axi_lcd_private {
	struct drm_device *drm_dev;
	struct drm_fbdev_cma *fbdev;
	struct drm_crtc *crtc;
	struct axi_lcd_encoder *encoder;
	struct i2c_client *encoder_slave;

	void __iomem *base;

	struct clk *lcd_clock;
	bool clk_enabled;

	struct dma_chan *dma;

	bool is_rgb;
};

#endif
