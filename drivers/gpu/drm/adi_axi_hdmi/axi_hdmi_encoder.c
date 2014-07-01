/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/clk.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_edid.h>

#include "axi_hdmi_drv.h"

#include "../i2c/adv7511.h"

#define AXI_HDMI_LEGACY_REG_CTRL		0x04
#define AXI_HDMI_LEGACY_REG_HTIMING1		0x08
#define AXI_HDMI_LEGACY_REG_HTIMING2		0x0C
#define AXI_HDMI_LEGACY_REG_VTIMING1		0x10
#define AXI_HDMI_LEGACY_REG_VTIMING2		0x14
#define AXI_HDMI_LEGACY_REG_STATUS		0x10

#define AXI_HDMI_LEGACY_ES_REG_HTIMING		0x08
#define AXI_HDMI_LEGACY_ES_REG_VTIMING		0x0c

#define AXI_HDMI_LEGACY_CTRL_ENABLE		BIT(0)

#define AXI_HDMI_STATUS_VMDA_UNDERFLOW	BIT(4)
#define AXI_HDMI_STATUS_VMDA_OVERFLOW	BIT(3)
#define AXI_HDMI_STATUS_VMDA_BE_ERROR	BIT(2)
#define AXI_HDMI_STATUS_VMDA_TPM_OOS	BIT(1)
#define AXI_HDMI_STATUS_HDMI_TPM_OOS	BIT(0)

#define AXI_HDMI_COLOR_PATTERN_ENABLE	BIT(24)

#define AXI_HDMI_REG_RESET		0x040
#define AXI_HDMI_REG_CTRL		0x044
#define AXI_HDMI_REG_SOURCE_SEL		0x048
#define AXI_HDMI_REG_COLORPATTERN	0x04c
#define AXI_HDMI_REG_STATUS		0x05c
#define AXI_HDMI_REG_VDMA_STATUS	0x060
#define AXI_HDMI_REG_TPM_STATUS		0x064
#define AXI_HDMI_REG_HTIMING1		0x400
#define AXI_HDMI_REG_HTIMING2		0x404
#define AXI_HDMI_REG_HTIMING3		0x408
#define AXI_HDMI_REG_VTIMING1		0x440
#define AXI_HDMI_REG_VTIMING2		0x444
#define AXI_HDMI_REG_VTIMING3		0x448

#define AXI_HDMI_RESET_ENABLE		BIT(0)

#define AXI_HDMI_CTRL_FULL_RANGE	BIT(1)
#define AXI_HDMI_CTRL_CSC_BYPASS	BIT(0)

#define AXI_HDMI_SOURCE_SEL_COLORPATTERN	0x3
#define AXI_HDMI_SOURCE_SEL_TESTPATTERN		0x2
#define AXI_HDMI_SOURCE_SEL_NORMAL		0x1
#define AXI_HDMI_SOURCE_SEL_NONE		0x0

static const struct debugfs_reg32 axi_hdmi_encoder_debugfs_regs[] = {
	{ "Reset", AXI_HDMI_REG_RESET },
	{ "Control", AXI_HDMI_REG_CTRL },
	{ "Source select", AXI_HDMI_REG_SOURCE_SEL },
	{ "Colorpattern", AXI_HDMI_REG_COLORPATTERN },
	{ "Status", AXI_HDMI_REG_STATUS },
	{ "VDMA status", AXI_HDMI_REG_VDMA_STATUS },
	{ "TPM status", AXI_HDMI_REG_TPM_STATUS },
	{ "HTiming1", AXI_HDMI_REG_HTIMING1 },
	{ "HTiming2", AXI_HDMI_REG_HTIMING2 },
	{ "HTiming3", AXI_HDMI_REG_HTIMING3 },
	{ "VTiming1", AXI_HDMI_REG_VTIMING1 },
	{ "VTiming2", AXI_HDMI_REG_VTIMING2 },
	{ "VTiming3", AXI_HDMI_REG_VTIMING3 },
};

static const uint16_t adv7511_csc_ycbcr_to_rgb[] = {
	0x0734, 0x04ad, 0x0000, 0x1c1b,
	0x1ddc, 0x04ad, 0x1f24, 0x0135,
	0x0000, 0x04ad, 0x087c, 0x1b77,
};

struct axi_hdmi_encoder {
	struct drm_encoder_slave encoder;
	struct drm_connector connector;

#ifdef CONFIG_DEBUG_FS
	struct debugfs_regset32 regset;
#endif
};

static inline struct axi_hdmi_encoder *to_axi_hdmi_encoder(struct drm_encoder *enc)
{
	return container_of(enc, struct axi_hdmi_encoder, encoder.base);
}

static inline struct drm_encoder *connector_to_encoder(struct drm_connector *connector)
{
	struct axi_hdmi_encoder *enc = container_of(connector, struct axi_hdmi_encoder, connector);
	return &enc->encoder.base;
}

static int axi_hdmi_connector_init(struct drm_device *dev,
	struct drm_connector *connector, struct drm_encoder *encoder);

static inline struct drm_encoder_slave_funcs *
get_slave_funcs(struct drm_encoder *enc)
{
	if (!to_encoder_slave(enc))
		return NULL;

	return to_encoder_slave(enc)->slave_funcs;
}

#ifdef CONFIG_DEBUG_FS

static int axi_hdmi_debugfs_cp_get(void *data, u64 *val)
{
	struct axi_hdmi_private *private = data;
	*val = readl(private->base + AXI_HDMI_REG_COLORPATTERN);
	return 0;
}

static int axi_hdmi_debugfs_cp_set(void *data, u64 val)
{
	struct axi_hdmi_private *private = data;

	writel(val, private->base + AXI_HDMI_REG_COLORPATTERN);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(axi_hdmi_cp_fops, axi_hdmi_debugfs_cp_get,
	axi_hdmi_debugfs_cp_set, "0x%06llx\n");


static const char * const axi_hdmi_mode_text[] = {
	[AXI_HDMI_SOURCE_SEL_NONE] = "none",
	[AXI_HDMI_SOURCE_SEL_NORMAL] = "normal",
	[AXI_HDMI_SOURCE_SEL_TESTPATTERN] = "testpattern",
	[AXI_HDMI_SOURCE_SEL_COLORPATTERN] = "colorpattern",
};

static ssize_t axi_hdmi_read_mode(struct file *file, char __user *userbuf,
	size_t count, loff_t *ppos)
{
	struct axi_hdmi_private *private = file->private_data;
	uint32_t src;
	const char *fmt;
	size_t len = 0;
	char buf[50];
	int i;
	src = readl(private->base + AXI_HDMI_REG_SOURCE_SEL);

	for (i = 0; i < ARRAY_SIZE(axi_hdmi_mode_text); i++) {
		if (src == i)
			fmt = "[%s] ";
		else
			fmt = "%s ";
		len += scnprintf(buf + len, sizeof(buf) - len, fmt,
				axi_hdmi_mode_text[i]);
	}

	buf[len - 1] = '\n';

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t axi_hdmi_set_mode(struct file *file, const char __user *userbuf,
	size_t count, loff_t *ppos)
{
	struct axi_hdmi_private *private = file->private_data;
	char buf[20];
	unsigned int i;
	count = min_t(size_t, count, sizeof(buf) - 1);
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = '\0';

	for (i = 0; i < ARRAY_SIZE(axi_hdmi_mode_text); i++) {
		if (sysfs_streq(axi_hdmi_mode_text[i], buf))
			break;
	}

	if (i == ARRAY_SIZE(axi_hdmi_mode_text))
		return -EINVAL;

	writel(i, private->base + AXI_HDMI_REG_SOURCE_SEL);

	return count;
}

static const struct file_operations axi_hdmi_mode_fops = {
	.open = simple_open,
	.read = axi_hdmi_read_mode,
	.write = axi_hdmi_set_mode,
};

static void axi_hdmi_debugfs_init(struct axi_hdmi_encoder *encoder)
{
	struct axi_hdmi_private *priv = encoder->encoder.base.dev->dev_private;

	if (priv->version != AXI_HDMI)
		return;

	encoder->regset.base = priv->base;
	encoder->regset.regs = axi_hdmi_encoder_debugfs_regs;
	encoder->regset.nregs = ARRAY_SIZE(axi_hdmi_encoder_debugfs_regs);

	debugfs_create_regset32(dev_name(encoder->encoder.base.dev->dev), S_IRUGO, NULL, &encoder->regset);
	debugfs_create_file("color_pattern", 0600, NULL, priv, &axi_hdmi_cp_fops);

	debugfs_create_file("mode", 0600, NULL, priv, &axi_hdmi_mode_fops);
}

#else

static inline void axi_hdmi_debugfs_init(struct axi_hdmi_encoder *enc)
{
}

#endif

// 800x480

static const char fake_edid_info[] = {
0x00,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x31,0xd8,0x00,0x00,
0x00,0x00,0x00,0x00,0x05,0x16,0x01,0x03,0x6d,0x1b,0x10,0x78,
0xea,0x5e,0xc0,0xa4,0x59,0x4a,0x98,0x25,0x20,0x50,0x54,0x00,
0x08,0x00,0x45,0x40,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0x01,0x01,0x01,0x01,0x01,0x64,0x19,0x20,0x40,0x31,0xe0,
0x26,0x10,0x08,0x90,0x36,0x00,0x15,0xa6,0x10,0x00,0x00,0x18,
0x00,0x00,0x00,0xff,0x00,0x4c,0x69,0x6e,0x75,0x78,0x20,0x23,
0x30,0x0a,0x20,0x20,0x20,0x20,0x00,0x00,0x00,0xfd,0x00,0x3b,
0x3d,0x39,0x3b,0x07,0x00,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,
0x00,0x00,0x00,0xfc,0x00,0x4c,0x69,0x6e,0x75,0x78,0x20,0x58,
0x47,0x41,0x0a,0x20,0x20,0x20,0x00,0x52,
};

static int get_edid_block(void *data, unsigned char *buf, int block,
                                  int len)
{
        if (len > 128)
                return -EINVAL;
        memcpy(buf, fake_edid_info, len);
        return 0;
}

static void axi_hdmi_encoder_dpms(struct drm_encoder *encoder, int mode)
{
        printk(KERN_ERR "Entered DPMS - turning %s\n", mode == DRM_MODE_DPMS_ON ? "on" : "off");
	struct axi_hdmi_encoder *axi_hdmi_encoder = to_axi_hdmi_encoder(encoder);
	struct drm_connector *connector = &axi_hdmi_encoder->connector;
	struct axi_hdmi_private *private = encoder->dev->dev_private;
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	struct adv7511_video_config config;
	struct edid *edid = NULL;
	switch (mode) {
	case DRM_MODE_DPMS_ON:
		if (!private->clk_enabled) {
			clk_prepare_enable(private->hdmi_clock);
			private->clk_enabled = true;
		}
		if (private->version == AXI_HDMI)
			writel(AXI_HDMI_RESET_ENABLE, private->base + AXI_HDMI_REG_RESET);
		else
			writel(AXI_HDMI_LEGACY_CTRL_ENABLE, private->base + AXI_HDMI_LEGACY_REG_CTRL);
                #ifdef CONFIG_DRM_ENCODER_ADV7511
		edid = adv7511_get_edid(encoder);
                #else

                // inject 800x480 edid
                edid = drm_do_get_edid(connector, get_edid_block, encoder);
                drm_mode_connector_update_edid_property(connector, edid);
                drm_add_edid_modes(connector, edid);

                #endif
		if (edid) {
			config.hdmi_mode = drm_detect_hdmi_monitor(edid);
			kfree(edid);
		} else {
			config.hdmi_mode = false;
		}
		hdmi_avi_infoframe_init(&config.avi_infoframe);

		config.avi_infoframe.scan_mode = HDMI_SCAN_MODE_UNDERSCAN;
		if (private->is_rgb) {
                                printk(KERN_ERR "Forcing RGB mode.\n");
				config.csc_enable = false;
				config.avi_infoframe.colorspace = HDMI_COLORSPACE_RGB;
		} else {
			config.csc_scaling_factor = ADV7511_CSC_SCALING_4;
			config.csc_coefficents = adv7511_csc_ycbcr_to_rgb;

			if ((connector->display_info.color_formats & DRM_COLOR_FORMAT_YCRCB422) &&
				config.hdmi_mode) {
				config.csc_enable = false;
				config.avi_infoframe.colorspace = HDMI_COLORSPACE_YUV422;
			} else {
				config.csc_enable = true;
				config.avi_infoframe.colorspace = HDMI_COLORSPACE_RGB;
			}
		}
                if (sfuncs) 
        		sfuncs->set_config(encoder, &config);

		break;
	default:
		if (private->version == AXI_HDMI)
			writel(0, private->base + AXI_HDMI_REG_RESET);
		else
			writel(0, private->base + AXI_HDMI_LEGACY_REG_CTRL);
		if (private->clk_enabled) {
			clk_disable_unprepare(private->hdmi_clock);
			private->clk_enabled = false;
		}
		break;
	}

	if (sfuncs && sfuncs->dpms)
		sfuncs->dpms(encoder, mode);
}

static bool axi_hdmi_encoder_mode_fixup(struct drm_encoder *encoder,
	const struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode)
{
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	if (sfuncs && sfuncs->mode_fixup)
		return sfuncs->mode_fixup(encoder, mode, adjusted_mode);

	return true;
}

static const struct drm_display_mode edt_etm0700g0dh6_mode = {
       .clock = 33260,
       .hdisplay = 800,
       .hsync_start = 800 + 40,
       .hsync_end = 800 + 40 + 128,
       .htotal = 800 + 40 + 128 + 88,
       .vdisplay = 480,
       .vsync_start = 480 + 10,
       .vsync_end = 480 + 10 + 2,
       .vtotal = 480 + 10 + 2 + 33,
       .vrefresh = 60,
       .flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

static void axi_hdmi_encoder_mode_set(struct drm_encoder *encoder,
	struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode)
{
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	struct axi_hdmi_private *private = encoder->dev->dev_private;
	unsigned int h_de_min, h_de_max;
	unsigned int v_de_min, v_de_max;
	unsigned int val;

	if (sfuncs && sfuncs->mode_set)
	sfuncs->mode_set(encoder, mode, adjusted_mode);

        memcpy((void*)mode,(void*)&edt_etm0700g0dh6_mode, sizeof(struct drm_display_mode));

	h_de_min = mode->htotal - mode->hsync_start;
	h_de_max = h_de_min + mode->hdisplay;
	v_de_min = mode->vtotal - mode->vsync_start;
	v_de_max = v_de_min + mode->vdisplay;
	
	switch (private->version) {
	case AXI_HDMI:
		val = (mode->hdisplay << 16) | mode->htotal;
		writel(val,  private->base + AXI_HDMI_REG_HTIMING1);
		val = mode->hsync_end - mode->hsync_start;
		writel(val,  private->base + AXI_HDMI_REG_HTIMING2);
		val = (h_de_max << 16) | h_de_min;
		writel(val,  private->base + AXI_HDMI_REG_HTIMING3);

		val = (mode->vdisplay << 16) | mode->vtotal;
		writel(val,  private->base + AXI_HDMI_REG_VTIMING1);
		val = mode->vsync_end - mode->vsync_start;
		writel(val,  private->base + AXI_HDMI_REG_VTIMING2);
		val = (v_de_max << 16) | v_de_min;
		writel(val,  private->base + AXI_HDMI_REG_VTIMING3);
		break;
	case AXI_HDMI_LEGACY_ES:
		val = (mode->hdisplay << 16) | mode->htotal;
		writel(val, private->base + AXI_HDMI_LEGACY_ES_REG_HTIMING);
		val = (mode->vdisplay << 16) | mode->vtotal;
		writel(val, private->base + AXI_HDMI_LEGACY_ES_REG_VTIMING);
		break;
	case AXI_HDMI_LEGACY:
		val = (mode->hsync_end - mode->hsync_start) << 16 | mode->htotal;
		writel(val, private->base + AXI_HDMI_LEGACY_REG_HTIMING1);
		val = (h_de_min << 16) | h_de_max;
		writel(val, private->base + AXI_HDMI_LEGACY_REG_HTIMING2);
		val = (mode->vsync_end - mode->vsync_start) << 16 | mode->vtotal;
		writel(val, private->base + AXI_HDMI_LEGACY_REG_VTIMING1);
		val = (v_de_min << 16) | v_de_max;
		writel(val, private->base + AXI_HDMI_LEGACY_REG_VTIMING2);
		break;
	default:
		break;
	}

        printk(KERN_ERR "clock = %d\n", mode->clock);
	clk_set_rate(private->hdmi_clock, mode->clock * 1000);
}

static void axi_hdmi_encoder_commit(struct drm_encoder *encoder)
{
	axi_hdmi_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static void axi_hdmi_encoder_prepare(struct drm_encoder *encoder)
{
	axi_hdmi_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static struct drm_crtc *axi_hdmi_encoder_get_crtc(struct drm_encoder *encoder)
{
	return encoder->crtc;
}

static struct drm_encoder_helper_funcs axi_hdmi_encoder_helper_funcs = {
	.dpms		= axi_hdmi_encoder_dpms,
	.mode_fixup	= axi_hdmi_encoder_mode_fixup,
	.mode_set	= axi_hdmi_encoder_mode_set,
	.prepare	= axi_hdmi_encoder_prepare,
	.commit		= axi_hdmi_encoder_commit,
	.get_crtc	= axi_hdmi_encoder_get_crtc,
};

static void axi_hdmi_encoder_destroy(struct drm_encoder *encoder)
{
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	struct axi_hdmi_encoder *axi_hdmi_encoder =
		to_axi_hdmi_encoder(encoder);

	if (sfuncs && sfuncs->destroy)
		sfuncs->destroy(encoder);

	drm_encoder_cleanup(encoder);
	encoder->dev->mode_config.num_encoder--;
	kfree(axi_hdmi_encoder);
}

static struct drm_encoder_funcs axi_hdmi_encoder_funcs = {
	.destroy = axi_hdmi_encoder_destroy,
};

struct drm_encoder *axi_hdmi_encoder_create(struct drm_device *dev)
{
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct axi_hdmi_encoder *axi_hdmi_encoder;
	struct drm_i2c_encoder_driver *encoder_drv;
	struct axi_hdmi_private *priv = dev->dev_private;

	axi_hdmi_encoder = kzalloc(sizeof(*axi_hdmi_encoder), GFP_KERNEL);
	if (!axi_hdmi_encoder) {
		DRM_ERROR("failed to allocate encoder\n");
		return NULL;
	}

	encoder = &axi_hdmi_encoder->encoder.base;
	encoder->possible_crtcs = 1;
	drm_encoder_init(dev, encoder, &axi_hdmi_encoder_funcs,
			DRM_MODE_ENCODER_TMDS);
	drm_encoder_helper_add(encoder, &axi_hdmi_encoder_helper_funcs);

#ifdef CONFIG_DRM_ENCODER_ADV7511
	encoder_drv =
	to_drm_i2c_encoder_driver(to_i2c_driver(priv->encoder_slave->dev.driver));
	encoder_drv->encoder_init(priv->encoder_slave, dev,
		&axi_hdmi_encoder->encoder);
#endif
	connector = &axi_hdmi_encoder->connector;

	axi_hdmi_connector_init(dev, connector, encoder);
	
        axi_hdmi_debugfs_init(axi_hdmi_encoder);

	if (priv->version == AXI_HDMI) {
		writel(AXI_HDMI_SOURCE_SEL_NORMAL, priv->base + AXI_HDMI_REG_SOURCE_SEL);
                if (priv->is_rgb) {
                        printk(KERN_ERR "RGB - forcing CSC_BYPASS\n");
			writel(AXI_HDMI_CTRL_CSC_BYPASS, priv->base + AXI_HDMI_REG_CTRL);
                }
	}
	return encoder;
}

static int axi_hdmi_connector_get_modes(struct drm_connector *connector)
{
	struct drm_encoder *encoder = connector_to_encoder(connector);
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	int count = 0;
#ifndef CONFIG_DRM_ENCODER_ADV7511
        printk(KERN_ERR "Get modes, forcing 1 mode, issuing dpms\n"); // TODO: do we have to do that ?
        axi_hdmi_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
        return 1;
#endif
	if (sfuncs && sfuncs->get_modes)
		count += sfuncs->get_modes(encoder, connector);

	return count;
}

static int axi_hdmi_connector_mode_valid(struct drm_connector *connector,
	struct drm_display_mode *mode)
{
	if (mode->clock > 165000)
		return MODE_CLOCK_HIGH;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;
	return MODE_OK;
}

static struct drm_encoder *axi_hdmi_best_encoder(struct drm_connector *connector)
{
	return connector_to_encoder(connector);
}

static struct drm_connector_helper_funcs axi_hdmi_connector_helper_funcs = {
	.get_modes	= axi_hdmi_connector_get_modes,
	.mode_valid	= axi_hdmi_connector_mode_valid,
	.best_encoder	= axi_hdmi_best_encoder,
};

static enum drm_connector_status axi_hdmi_connector_detect(
	struct drm_connector *connector, bool force)
{
#ifdef CONFIG_DRM_ENCODER_ADV7511
	enum drm_connector_status status = connector_status_unknown;
	struct drm_encoder *encoder = connector_to_encoder(connector);
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);

	if (sfuncs && sfuncs->detect)
		status = sfuncs->detect(encoder, connector);

	return status;
#else
        return connector_status_connected;
#endif
}

static void axi_hdmi_connector_destroy(struct drm_connector *connector)
{
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs axi_hdmi_connector_funcs = {
	.dpms		= drm_helper_connector_dpms,
	.fill_modes	= drm_helper_probe_single_connector_modes,
	.detect		= axi_hdmi_connector_detect,
	.destroy	= axi_hdmi_connector_destroy,
};

static int axi_hdmi_connector_init(struct drm_device *dev,
	struct drm_connector *connector, struct drm_encoder *encoder)
{
	int type;
	int err;
	type = DRM_MODE_CONNECTOR_HDMIA;
	connector->polled = DRM_CONNECTOR_POLL_CONNECT |
				DRM_CONNECTOR_POLL_DISCONNECT;

	drm_connector_init(dev, connector, &axi_hdmi_connector_funcs, type);
	drm_connector_helper_add(connector, &axi_hdmi_connector_helper_funcs);
	err = drm_sysfs_connector_add(connector);
	if (err)
		goto err_connector;
	connector->encoder = encoder;

	err = drm_mode_connector_attach_encoder(connector, encoder);
	if (err) {
		DRM_ERROR("failed to attach a connector to a encoder\n");
		goto err_sysfs;
	}
	return 0;

err_sysfs:
	drm_sysfs_connector_remove(connector);
err_connector:
	drm_connector_cleanup(connector);
	return err;
}
