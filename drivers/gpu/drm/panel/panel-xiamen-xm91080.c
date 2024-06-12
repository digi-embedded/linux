// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024, Digi International Inc.
 *
 * Author: Gonzalo Ruiz <gonzalo.ruiz@digi.com>
 */

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <linux/bitfield.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#define DSI_CMD_SETMIPI_1LANE		0
#define DSI_CMD_SETMIPI_2LANE		1
#define DSI_CMD_SETMIPI_3LANE		2
#define DSI_CMD_SETMIPI_4LANE		3

struct xm91080;

struct xm91080_panel_desc {
	const struct drm_display_mode *mode;
	unsigned int lanes;
	enum mipi_dsi_pixel_format format;
	unsigned int panel_sleep_delay;
	void (*gip_sequence)(struct xm91080 *xm91080);
};

struct xm91080 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	const struct xm91080_panel_desc *desc;
	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset;
	unsigned int sleep_delay;
};

static inline struct xm91080 *panel_to_xm91080(struct drm_panel *panel)
{
	return container_of(panel, struct xm91080, panel);
}

static inline int xm91080_dsi_write(struct xm91080 *xm91080, const void *seq,
				   size_t len)
{
	return mipi_dsi_dcs_write_buffer(xm91080->dsi, seq, len);
}

#define XM91080_DSI(xm91080, seq...)				\
	{							\
		const u8 d[] = { seq };				\
		xm91080_dsi_write(xm91080, d, ARRAY_SIZE(d));	\
	}

static int xm91080_read_id(struct xm91080 *xm91080)
{
	u8 id1, id2, id3;
	int ret;

	ret = mipi_dsi_dcs_read(xm91080->dsi, 0xDA, &id1, 1);
	if (ret < 0) {
		dev_err(&xm91080->dsi->dev, "could not read ID1\n");
		return ret;
	}

	ret = mipi_dsi_dcs_read(xm91080->dsi, 0xDB, &id2, 1);
	if (ret < 0) {
		dev_err(&xm91080->dsi->dev, "could not read ID2\n");
		return ret;
	}

	ret = mipi_dsi_dcs_read(xm91080->dsi, 0xDC, &id3, 1);
	if (ret < 0) {
		dev_err(&xm91080->dsi->dev, "could not read ID3\n");
		return ret;
	}

	dev_info(&xm91080->dsi->dev,
		     "manufacturer: %02x version: %02x driver: %02x\n",
		     id1, id2, id3);

	return 0;
}

/* General initialization Packet (GIP) sequence */
static void e55rb_i_mw346_c_gip_sequence(struct xm91080 *xm91080)
{
	XM91080_DSI(xm91080, MIPI_DCS_SOFT_RESET);

	/* We need to wait 5ms before sending new commands */
	msleep(5);

	/* Set XM Command Password 1 */
	XM91080_DSI(xm91080, 0x00,0x00);
	XM91080_DSI(xm91080, 0xFF,0x10,0x80,0x01);

	/* Set XM Command Password 2 */
	XM91080_DSI(xm91080, 0x00,0x80);
	XM91080_DSI(xm91080, 0xFF,0x10,0x80);

	/* boe 5.5  */
	/* tcon setting ok */
	XM91080_DSI(xm91080, 0x00,0x81);
	XM91080_DSI(xm91080, 0xb2,0x74);

	XM91080_DSI(xm91080, 0x00,0x86);
	/*XM91080_DSI(xm91080, 0xb2,0x01,0x01,0x01,0x01,0x22,0x02);*/
	XM91080_DSI(xm91080, 0xb2,0x01,0x01,0x01,0x01,0x1B,0x08); /* 20190905 */

	/* ckv setting */
	XM91080_DSI(xm91080, 0x00,0x80);
	XM91080_DSI(xm91080, 0xb4,0x18,0x03,0x07,0x80,0x02,0x00,0x00,0x02,0x00,0x00);
	XM91080_DSI(xm91080, 0x00,0x90);
	XM91080_DSI(xm91080, 0xb4,0x18,0x02,0x07,0x82,0x02,0x00,0x00,0x02,0x00,0x00);
	XM91080_DSI(xm91080, 0x00,0xa0);
	XM91080_DSI(xm91080, 0xb4,0x18,0x01,0x07,0x82,0x02,0x00,0x00,0x02,0x00,0x00);
	XM91080_DSI(xm91080, 0x00,0xb0);
	XM91080_DSI(xm91080, 0xb4,0x18,0x00,0x07,0x83,0x02,0x00,0x00,0x02,0x00,0x00);

	/* vst */
	XM91080_DSI(xm91080, 0x00,0x80);
	XM91080_DSI(xm91080, 0xb6,0x83,0x02,0x00,0x60,0x82,0x02,0x00,0x60); 

	/* u2d ok */
	XM91080_DSI(xm91080, 0x00,0x80);
	XM91080_DSI(xm91080, 0xbc,0x00,0x0e,0x25,0x26,0x00,0x06,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x21,0x20,0x1f); 
	XM91080_DSI(xm91080, 0x00,0x90);
	XM91080_DSI(xm91080, 0xbc,0x00,0x0d,0x25,0x26,0x00,0x05,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x21,0x20,0x1f); 

	/* d2u ok */
	XM91080_DSI(xm91080, 0x00,0xa0);
	XM91080_DSI(xm91080, 0xbc,0x00,0x0d,0x26,0x25,0x00,0x07,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x21,0x20,0x1f); 
	XM91080_DSI(xm91080, 0x00,0xb0);
	XM91080_DSI(xm91080, 0xbc,0x00,0x0e,0x26,0x25,0x00,0x08,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x21,0x20,0x1f);

	XM91080_DSI(xm91080, 0x00,0xa0);
	XM91080_DSI(xm91080, 0xb9,0xff,0xd4,0xd7,0xd7,0xff,0xd4,0xe0,0xd4,0xff,0xff,0xff,0xff,0xff,0xe4,0xe4,0xe4);

	XM91080_DSI(xm91080, 0x00,0xb0);
	XM91080_DSI(xm91080, 0xb9,0xff,0xd4,0xd7,0xd7,0xff,0xd4,0xe0,0xd4,0xff,0xff,0xff,0xff,0xff,0xe4,0xe4,0xe4);

	XM91080_DSI(xm91080, 0x00,0x80);
	XM91080_DSI(xm91080, 0xb9,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff);

	XM91080_DSI(xm91080, 0x00,0x90);
	XM91080_DSI(xm91080, 0xb9,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff);

	XM91080_DSI(xm91080, 0x00,0x80);
	XM91080_DSI(xm91080, 0xba,0xea,0xea,0xff,0xc0,0xea,0xea,0xff,0xc0);

	XM91080_DSI(xm91080, 0x00,0xd0);
	XM91080_DSI(xm91080, 0xb6,0x81,0x00,0x02,0x02);
	XM91080_DSI(xm91080, 0x00,0xe0);
	XM91080_DSI(xm91080, 0xb6,0x00,0x0c,0x22,0x02,0x20,0x11,0x01,0x01,0x04,0x00);

	/* G-swap */
	XM91080_DSI(xm91080, 0x00,0xA5);
	XM91080_DSI(xm91080, 0xC0,0x20);

	/* mirror X2 */
	XM91080_DSI(xm91080, 0x00,0xA0);
	XM91080_DSI(xm91080, 0xA5,0x20); /* 20-0  ˢ  10-  ˢ */

	/* VGH=VGHO/VGL=VGLO */
	XM91080_DSI(xm91080, 0x00,0xF0);
	XM91080_DSI(xm91080, 0xA4,0x00);

	/* VGL/VGH = -8 / 9 */
	XM91080_DSI(xm91080, 0x00,0x90);
	XM91080_DSI(xm91080, 0xAB,0xA8,0x9E);

	/* VCOM */
	XM91080_DSI(xm91080, 0x00,0xB1);
	XM91080_DSI(xm91080, 0xA4,0xB8,0xB8);  /* -0.2 */

	/* GVDDP/GVDDN */
	XM91080_DSI(xm91080, 0x00,0xA0);
	XM91080_DSI(xm91080, 0xA4,0x23,0x23);  /* 4.6 */

	XM91080_DSI(xm91080, 0x00,0x80);                                                                                                                              //
	XM91080_DSI(xm91080, 0xD4,0x03,0x05,0x07,0x0C,0x12,0x16,0x19,0x1F,0x23,0x32,0x3C,0x4D,0x59,0x6D,0x7C,0x7C,0x8C,0x9E,0xAB,0xBB,0xC6,0xD4,0xD8,0xDD,0xE2,0xE7,0xEC,0xF3,0xFB,0xFF,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00);
	XM91080_DSI(xm91080, 0x00,0x80);
	XM91080_DSI(xm91080, 0xD5,0x03,0x05,0x07,0x0C,0x12,0x16,0x19,0x1F,0x23,0x32,0x3C,0x4D,0x59,0x6D,0x7C,0x7C,0x8C,0x9E,0xAB,0xBB,0xC6,0xD4,0xD8,0xDD,0xE2,0xE7,0xEC,0xF3,0xFB,0xFF,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00);
	XM91080_DSI(xm91080, 0x00,0x80);
	XM91080_DSI(xm91080, 0xD6,0x03,0x05,0x07,0x0C,0x12,0x16,0x19,0x1F,0x23,0x32,0x3C,0x4D,0x59,0x6D,0x7C,0x7C,0x8C,0x9E,0xAB,0xBB,0xC6,0xD4,0xD8,0xDD,0xE2,0xE7,0xEC,0xF3,0xFB,0xFF,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00);
	XM91080_DSI(xm91080, 0x00,0x80);
	XM91080_DSI(xm91080, 0xD7,0x03,0x05,0x07,0x0C,0x12,0x16,0x19,0x1F,0x23,0x32,0x3C,0x4D,0x59,0x6D,0x7C,0x7C,0x8C,0x9E,0xAB,0xBB,0xC6,0xD4,0xD8,0xDD,0xE2,0xE7,0xEC,0xF3,0xFB,0xFF,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00);
	XM91080_DSI(xm91080, 0x00,0x80);
	XM91080_DSI(xm91080, 0xD8,0x03,0x05,0x07,0x0C,0x12,0x16,0x19,0x1F,0x23,0x32,0x3C,0x4D,0x59,0x6D,0x7C,0x7C,0x8C,0x9E,0xAB,0xBB,0xC6,0xD4,0xD8,0xDD,0xE2,0xE7,0xEC,0xF3,0xFB,0xFF,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00);
	XM91080_DSI(xm91080, 0x00,0x80);
	XM91080_DSI(xm91080, 0xD9,0x03,0x05,0x07,0x0C,0x12,0x16,0x19,0x1F,0x23,0x32,0x3C,0x4D,0x59,0x6D,0x7C,0x7C,0x8C,0x9E,0xAB,0xBB,0xC6,0xD4,0xD8,0xDD,0xE2,0xE7,0xEC,0xF3,0xFB,0xFF,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00);

	/* enmode_sdpch */
	XM91080_DSI(xm91080, 0x00,0xC2);
	XM91080_DSI(xm91080, 0xA6,0x08);

	/* sd_prc */
	XM91080_DSI(xm91080, 0x00,0x86);
	XM91080_DSI(xm91080, 0xA5,0x19);

	XM91080_DSI(xm91080, 0x00,0x83);
	XM91080_DSI(xm91080, 0xA7,0x88); /* 0x88=1.325 */

	/* mipi skew */
	XM91080_DSI(xm91080, 0x00,0x90);
	XM91080_DSI(xm91080, 0xA3,0x04,0x04,0x01,0x05,0x06,0x00);

	XM91080_DSI(xm91080, 0x00,0xC0);
	XM91080_DSI(xm91080, 0xA4,0x01); /* 20190905 */

	/* Sleep Out */
	XM91080_DSI(xm91080, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(250);
}

static int xm91080_prepare(struct drm_panel *panel)
{
	struct xm91080 *xm91080 = panel_to_xm91080(panel);
	int ret;

	gpiod_set_value(xm91080->reset, 0);

	ret = regulator_bulk_enable(ARRAY_SIZE(xm91080->supplies),
				    xm91080->supplies);
	if (ret < 0)
		return ret;
	msleep(20);

	gpiod_set_value(xm91080->reset, 1);
	msleep(150);

	xm91080_read_id(xm91080);

	if (xm91080->desc->gip_sequence)
		xm91080->desc->gip_sequence(xm91080);

	return 0;
}

static int xm91080_enable(struct drm_panel *panel)
{
	struct xm91080 *xm91080 = panel_to_xm91080(panel);

	XM91080_DSI(xm91080, MIPI_DCS_SET_DISPLAY_ON);
	msleep(50);

	return 0;
}

static int xm91080_disable(struct drm_panel *panel)
{
	struct xm91080 *xm91080 = panel_to_xm91080(panel);

	XM91080_DSI(xm91080, MIPI_DCS_SET_DISPLAY_OFF);
	msleep(50);

	return 0;
}

static int xm91080_unprepare(struct drm_panel *panel)
{
	struct xm91080 *xm91080 = panel_to_xm91080(panel);

	XM91080_DSI(xm91080, MIPI_DCS_ENTER_SLEEP_MODE);

	msleep(xm91080->sleep_delay);

	gpiod_set_value(xm91080->reset, 0);

	msleep(xm91080->sleep_delay);

	regulator_bulk_disable(ARRAY_SIZE(xm91080->supplies), xm91080->supplies);

	return 0;
}

static int xm91080_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	struct xm91080 *xm91080 = panel_to_xm91080(panel);
	const struct drm_display_mode *desc_mode = xm91080->desc->mode;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, desc_mode);
	if (!mode) {
		dev_err(&xm91080->dsi->dev, "failed to add mode %ux%u@%u\n",
			desc_mode->hdisplay, desc_mode->vdisplay,
			drm_mode_vrefresh(desc_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = desc_mode->width_mm;
	connector->display_info.height_mm = desc_mode->height_mm;

	return 1;
}

static const struct drm_panel_funcs xm91080_funcs = {
	.disable	= xm91080_disable,
	.unprepare	= xm91080_unprepare,
	.prepare	= xm91080_prepare,
	.enable		= xm91080_enable,
	.get_modes	= xm91080_get_modes,
};

static const struct drm_display_mode e55rb_i_mw346_c_mode = {
	.clock		= 130000,

	.hdisplay	= 1080,
	.hsync_start	= 1080 + 10,		/* hdisplay + hfront */
	.hsync_end	= 1080 + 10 + 8,	/* hdisplay + hfront + hsync */
	.htotal		= 1080 + 10 + 8 + 20,	/* hdisplay + hfront + hsync + hback */

	.vdisplay	= 1920,
	.vsync_start	= 1920 + 10,		/* vdisplay + vfront */
	.vsync_end	= 1920 + 10 + 8,	/* vdisplay + vfront + vsync */
	.vtotal		= 1920 + 10 + 8 + 20,	/* vdisplay + vfront + vsync + vback */

	.width_mm	= 56,
	.height_mm	= 78,

	.flags		= DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,

	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static const struct xm91080_panel_desc e55rb_i_mw346_c_desc = {
	.mode = &e55rb_i_mw346_c_mode,
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.panel_sleep_delay = 0,
	.gip_sequence = e55rb_i_mw346_c_gip_sequence,
};

static int xm91080_dsi_probe(struct mipi_dsi_device *dsi)
{
	const struct xm91080_panel_desc *desc;
	struct xm91080 *xm91080;
	int ret;

	xm91080 = devm_kzalloc(&dsi->dev, sizeof(*xm91080), GFP_KERNEL);
	if (!xm91080)
		return -ENOMEM;

	desc = of_device_get_match_data(&dsi->dev);
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	dsi->format = desc->format;
	dsi->lanes = desc->lanes;

	xm91080->supplies[0].supply = "VCC";
	xm91080->supplies[1].supply = "IOVCC";

	ret = devm_regulator_bulk_get(&dsi->dev, ARRAY_SIZE(xm91080->supplies),
				      xm91080->supplies);
	if (ret < 0)
		return ret;

	xm91080->reset = devm_gpiod_get(&dsi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(xm91080->reset)) {
		dev_err(&dsi->dev, "Couldn't get our reset GPIO\n");
		return PTR_ERR(xm91080->reset);
	}

	drm_panel_init(&xm91080->panel, &dsi->dev, &xm91080_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	/**
	 * Once sleep out has been issued, XM91080 IC required to wait 120ms
	 * before initiating new commands.
	 */
	xm91080->sleep_delay = 120 + desc->panel_sleep_delay;

	ret = drm_panel_of_backlight(&xm91080->panel);
	if (ret)
		return ret;

	drm_panel_add(&xm91080->panel);

	mipi_dsi_set_drvdata(dsi, xm91080);
	xm91080->dsi = dsi;
	xm91080->desc = desc;

	ret = mipi_dsi_attach(dsi);
	if (ret)
		goto err_attach;

	return 0;

err_attach:
	drm_panel_remove(&xm91080->panel);
	return ret;
}

static void xm91080_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct xm91080 *xm91080 = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&xm91080->panel);
}

static const struct of_device_id xm91080_of_match[] = {
	{ .compatible = "focus,e55rb-i-mw346-c", .data = &e55rb_i_mw346_c_desc },
	{ }
};
MODULE_DEVICE_TABLE(of, xm91080_of_match);

static struct mipi_dsi_driver xm91080_dsi_driver = {
	.probe		= xm91080_dsi_probe,
	.remove		= xm91080_dsi_remove,
	.driver = {
		.name		= "xm91080",
		.of_match_table	= xm91080_of_match,
	},
};
module_mipi_dsi_driver(xm91080_dsi_driver);

MODULE_AUTHOR("Gonzalo Ruiz <gonzalo.ruiz@digi.com>");
MODULE_DESCRIPTION("Xiamen XM91080 TFT LCD Panel Driver");
MODULE_LICENSE("GPL");
