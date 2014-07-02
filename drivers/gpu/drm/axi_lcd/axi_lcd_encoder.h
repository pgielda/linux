/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#ifndef _AXI_LCD_ENCODER_H_
#define _AXI_LCD_ENCODER_H_

struct drm_encoder *axi_lcd_encoder_create(struct drm_device *dev);

#endif
