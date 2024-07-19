// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 Rockchip Electronics Co. Ltd.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <video/of_display_timing.h>

#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_file.h>
#include <drm/drm_ioctl.h>

#include <drm/drm_of.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_panel.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_probe_helper.h>

struct lt8912 {
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct drm_display_mode mode;
	struct device *dev;
	struct mipi_dsi_device *dsi;
	struct device_node *host_node;
	u8 num_dsi_lanes;
	u8 channel_id;
	struct i2c_client *i2c;
	u8 sink_is_hdmi;
	struct regmap *regmap[3];
	struct gpio_desc *reset_n;
	struct gpio_desc *gpiod_int;
	struct regulator *vdd1;
	struct regulator *vdd2;
	int hpd_irq;
	bool is_suspended;
	bool no_hpd;
	bool no_edid;
};

static int lt8912_attach_dsi(struct lt8912 *lt);

static inline struct lt8912 *bridge_to_lt8912(struct drm_bridge *b)
{
	return container_of(b, struct lt8912, bridge);
}

static inline struct lt8912 *connector_to_lt8912(struct drm_connector *c)
{
	return container_of(c, struct lt8912, connector);
}

/* LT8912 MIPI to HDMI & LVDS REG setting - 20180115.txt */
static void lt8912_init(struct lt8912 *lt)
{
	u8 lanes = lt->dsi->lanes;
	const struct drm_display_mode *mode = &lt->mode;
	u32 hactive, hfp, hsync, hbp, vfp, vsync, vbp, htotal, vtotal;
	unsigned int hsync_activehigh, vsync_activehigh, reg;
	unsigned int version[2];

	dev_info(lt->dev, DRM_MODE_FMT "\n", DRM_MODE_ARG(mode));
	/* TODO: lvds output init */

	hactive = mode->hdisplay;
	hfp = mode->hsync_start - mode->hdisplay;
	hsync = mode->hsync_end - mode->hsync_start;
	hsync_activehigh = !!(mode->flags & DRM_MODE_FLAG_PHSYNC);
	hbp = mode->htotal - mode->hsync_end;
	vfp = mode->vsync_start - mode->vdisplay;
	vsync = mode->vsync_end - mode->vsync_start;
	vsync_activehigh = !!(mode->flags & DRM_MODE_FLAG_PVSYNC);
	vbp = mode->vtotal - mode->vsync_end;
	htotal = mode->htotal;
	vtotal = mode->vtotal;

	regmap_read(lt->regmap[0], 0x00, &version[0]);
	regmap_read(lt->regmap[0], 0x01, &version[1]);

	dev_info(lt->dev, "LT8912 ID: %02x, %02x\n",
		 version[0], version[1]);

	/* DigitalClockEn */
	regmap_write(lt->regmap[0], 0x08, 0xff);
	regmap_write(lt->regmap[0], 0x09, 0xff);
	regmap_write(lt->regmap[0], 0x0a, 0xff);
	regmap_write(lt->regmap[0], 0x0b, 0x7c);
	regmap_write(lt->regmap[0], 0x0c, 0xff);

	/* TxAnalog */
	regmap_write(lt->regmap[0], 0x31, 0xa1);
	regmap_write(lt->regmap[0], 0x32, 0xa1);
	regmap_write(lt->regmap[0], 0x33, 0x03);
	regmap_write(lt->regmap[0], 0x37, 0x00);
	regmap_write(lt->regmap[0], 0x38, 0x22);
	regmap_write(lt->regmap[0], 0x60, 0x82);

	/* CbusAnalog */
	regmap_write(lt->regmap[0], 0x39, 0x45);
	regmap_write(lt->regmap[0], 0x3a, 0x00);
	regmap_write(lt->regmap[0], 0x3b, 0x00);

	/* HDMIPllAnalog */
	regmap_write(lt->regmap[0], 0x44, 0x31);
	regmap_write(lt->regmap[0], 0x55, 0x44);
	regmap_write(lt->regmap[0], 0x57, 0x01);
	regmap_write(lt->regmap[0], 0x5a, 0x02);

	/* MIPIAnalog */
	regmap_write(lt->regmap[0], 0x3e, 0xce);
	regmap_write(lt->regmap[0], 0x3f, 0xd4);
	regmap_write(lt->regmap[0], 0x41, 0x3c);

	/* MipiBasicSet */
	regmap_write(lt->regmap[1], 0x12, 0x04);
	regmap_write(lt->regmap[1], 0x13, lanes % 4);
	regmap_write(lt->regmap[1], 0x14, 0x00);

	regmap_write(lt->regmap[1], 0x15, 0x00);
	regmap_write(lt->regmap[1], 0x1a, 0x03);
	regmap_write(lt->regmap[1], 0x1b, 0x03);

	/* MIPIDig */
	regmap_write(lt->regmap[1], 0x10, 0x01);
	regmap_write(lt->regmap[1], 0x11, 0x0a);
	regmap_write(lt->regmap[1], 0x18, hsync);
	regmap_write(lt->regmap[1], 0x19, vsync);
	regmap_write(lt->regmap[1], 0x1c, hactive % 0x100);
	regmap_write(lt->regmap[1], 0x1d, hactive >> 8);

	regmap_write(lt->regmap[1], 0x2f, 0x0c);

	regmap_write(lt->regmap[1], 0x34, htotal % 0x100);
	regmap_write(lt->regmap[1], 0x35, htotal >> 8);
	regmap_write(lt->regmap[1], 0x36, vtotal % 0x100);
	regmap_write(lt->regmap[1], 0x37, vtotal >> 8);
	regmap_write(lt->regmap[1], 0x38, vbp % 0x100);
	regmap_write(lt->regmap[1], 0x39, vbp >> 8);
	regmap_write(lt->regmap[1], 0x3a, vfp % 0x100);
	regmap_write(lt->regmap[1], 0x3b, vfp >> 8);
	regmap_write(lt->regmap[1], 0x3c, hbp % 0x100);
	regmap_write(lt->regmap[1], 0x3d, hbp >> 8);
	regmap_write(lt->regmap[1], 0x3e, hfp % 0x100);
	regmap_write(lt->regmap[1], 0x3f, hfp >> 8);
	regmap_read(lt->regmap[0], 0xab, &reg);
	reg &= 0xfc;
	reg |= (hsync_activehigh < 1) | vsync_activehigh;
	regmap_write(lt->regmap[0], 0xab, reg);

	/* DDSConfig */
	regmap_write(lt->regmap[1], 0x4e, 0x6a);
	regmap_write(lt->regmap[1], 0x4f, 0xad);
	regmap_write(lt->regmap[1], 0x50, 0xf3);
	regmap_write(lt->regmap[1], 0x51, 0x80);

	regmap_write(lt->regmap[1], 0x1f, 0x5e);
	regmap_write(lt->regmap[1], 0x20, 0x01);
	regmap_write(lt->regmap[1], 0x21, 0x2c);
	regmap_write(lt->regmap[1], 0x22, 0x01);
	regmap_write(lt->regmap[1], 0x23, 0xfa);
	regmap_write(lt->regmap[1], 0x24, 0x00);
	regmap_write(lt->regmap[1], 0x25, 0xc8);
	regmap_write(lt->regmap[1], 0x26, 0x00);
	regmap_write(lt->regmap[1], 0x27, 0x5e);
	regmap_write(lt->regmap[1], 0x28, 0x01);
	regmap_write(lt->regmap[1], 0x29, 0x2c);
	regmap_write(lt->regmap[1], 0x2a, 0x01);
	regmap_write(lt->regmap[1], 0x2b, 0xfa);
	regmap_write(lt->regmap[1], 0x2c, 0x00);
	regmap_write(lt->regmap[1], 0x2d, 0xc8);
	regmap_write(lt->regmap[1], 0x2e, 0x00);
	regmap_write(lt->regmap[1], 0x42, 0x64);
	regmap_write(lt->regmap[1], 0x43, 0x00);
	regmap_write(lt->regmap[1], 0x44, 0x04);
	regmap_write(lt->regmap[1], 0x45, 0x00);
	regmap_write(lt->regmap[1], 0x46, 0x59);
	regmap_write(lt->regmap[1], 0x47, 0x00);
	regmap_write(lt->regmap[1], 0x48, 0xf2);
	regmap_write(lt->regmap[1], 0x49, 0x06);
	regmap_write(lt->regmap[1], 0x4a, 0x00);
	regmap_write(lt->regmap[1], 0x4b, 0x72);
	regmap_write(lt->regmap[1], 0x4c, 0x45);
	regmap_write(lt->regmap[1], 0x4d, 0x00);
	regmap_write(lt->regmap[1], 0x52, 0x08);
	regmap_write(lt->regmap[1], 0x53, 0x00);
	regmap_write(lt->regmap[1], 0x54, 0xb2);
	regmap_write(lt->regmap[1], 0x55, 0x00);
	regmap_write(lt->regmap[1], 0x56, 0xe4);
	regmap_write(lt->regmap[1], 0x57, 0x0d);
	regmap_write(lt->regmap[1], 0x58, 0x00);
	regmap_write(lt->regmap[1], 0x59, 0xe4);
	regmap_write(lt->regmap[1], 0x5a, 0x8a);
	regmap_write(lt->regmap[1], 0x5b, 0x00);
	regmap_write(lt->regmap[1], 0x5c, 0x34);
	regmap_write(lt->regmap[1], 0x1e, 0x4f);
	regmap_write(lt->regmap[1], 0x51, 0x00);

	regmap_write(lt->regmap[0], 0xb2, lt->sink_is_hdmi);

	/* Audio Disable */
	regmap_write(lt->regmap[2], 0x06, 0x00);
	regmap_write(lt->regmap[2], 0x07, 0x00);

	regmap_write(lt->regmap[2], 0x34, 0xd2);

	regmap_write(lt->regmap[2], 0x3c, 0x41);

	/* MIPIRxLogicRes */
	regmap_write(lt->regmap[0], 0x03, 0x7f);
	usleep_range(10000, 20000);
	regmap_write(lt->regmap[0], 0x03, 0xff);

	regmap_write(lt->regmap[1], 0x51, 0x80);
	usleep_range(10000, 20000);
	regmap_write(lt->regmap[1], 0x51, 0x00);

}

int lt8912_parse_dt(struct device_node *np, struct lt8912 *lt)
{
	struct device *dev = &lt->i2c->dev;
	u32 num_lanes = 0;

	lt->no_hpd = of_property_read_bool(np, "no-hpd");
	lt->no_edid = of_property_read_bool(np, "no-edid");

	of_property_read_u32(np, "digi,dsi-lanes", &num_lanes);

	if (num_lanes < 1 || num_lanes > 4) {
		dev_err(dev, "Invalid dsi-lanes: %d\n", num_lanes);
		return -EINVAL;
	}

	lt->num_dsi_lanes = num_lanes;

	return 0;
}

static void lt8912_wakeup(struct lt8912 *lt)
{
	int ret;

	if (lt->vdd1 && lt->is_suspended) {
		ret = regulator_enable(lt->vdd1);
		if (ret)
			dev_err(lt->dev,
				"failed to enable regulator vdd1 (%d)\n",
				ret);
	}

	if (lt->vdd2 && lt->is_suspended) {
		ret = regulator_enable(lt->vdd2);
		if (ret)
			dev_err(lt->dev,
				"failed to enable regulator vdd2 (%d)\n",
				ret);
	}

	gpiod_direction_output(lt->reset_n, 1);
	msleep(120);
	gpiod_direction_output(lt->reset_n, 0);

	regmap_write(lt->regmap[0], 0x08,0xff); /* enable clk gating */
	regmap_write(lt->regmap[0], 0x41,0x3c); /* MIPI Rx Power On */
	regmap_write(lt->regmap[0], 0x05,0xfb); /* DDS logical reset */
	regmap_write(lt->regmap[0], 0x05,0xff);
	regmap_write(lt->regmap[0], 0x03,0x7f); /* MIPI RX logical reset */
	usleep_range(10000, 20000);
	regmap_write(lt->regmap[0], 0x03,0xff);
	regmap_write(lt->regmap[0], 0x32,0xa1);
	regmap_write(lt->regmap[0], 0x33,0x03);

	lt->is_suspended = false;
}

static void lt8912_sleep(struct lt8912 *lt)
{
	int ret;

	regmap_write(lt->regmap[0], 0x32,0xa0);
	regmap_write(lt->regmap[0], 0x33,0x00); /* Disable HDMI output. */
	regmap_write(lt->regmap[0], 0x41,0x3d); /* MIPI Rx Power Down. */
	regmap_write(lt->regmap[0], 0x08,0x00); /* disable DDS clk. */

	gpiod_direction_output(lt->reset_n, 1);

	if (lt->vdd2 && !lt->is_suspended) {
		ret = regulator_disable(lt->vdd2);
		if (ret)
			dev_err(lt->dev,
				"failed to disable regulator vdd2 (%d)\n",
				ret);
	}

	if (lt->vdd1 && !lt->is_suspended) {
		ret = regulator_disable(lt->vdd1);
		if (ret)
			dev_err(lt->dev,
				"failed to disable regulator vdd1 (%d)\n",
				ret);
	}

	lt->is_suspended = true;
}

static irqreturn_t lt8912_hpd_threaded_handler(int unused, void *data)
{
	struct lt8912 *lt = data;
	struct drm_connector *connector = &lt->connector;

	if (connector->dev)
		drm_helper_hpd_irq_event(connector->dev);

	return IRQ_HANDLED;
}

static enum drm_connector_status
lt8912_connector_detect(struct drm_connector *connector, bool force)
{
	struct lt8912 *lt = connector_to_lt8912(connector);
	int i = 0;

	if (lt->no_hpd) {
		return connector_status_connected;
	} else {
		/* TODO: HPD handing (reg[0xc1] - bit[7]) */
		do {
			if (gpiod_get_value(lt->gpiod_int))
				return connector_status_connected;
			if (force)
				usleep_range(5000, 10000);
		} while (i++ < 100 && force);

		return connector_status_disconnected;
	}
}

static const struct drm_connector_funcs lt8912_connector_funcs = {
	.detect = lt8912_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct drm_encoder *
lt8912_connector_best_encoder(struct drm_connector *connector)
{
	struct lt8912 *lt = connector_to_lt8912(connector);

	return lt->bridge.encoder;
}

static int lt8912_connector_get_modes(struct drm_connector *connector)
{
	struct lt8912 *lt = connector_to_lt8912(connector);
	struct drm_display_mode *mode;
	struct edid *edid;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	u32 bus_flags = 0;
	int ret, num = 0;

	if (lt->no_edid) {
		mode = drm_mode_create(connector->dev);
		if (!mode)
			return -EINVAL;

		ret = of_get_drm_display_mode(lt->dev->of_node, mode,
					      &bus_flags, OF_USE_NATIVE_MODE);
		if (ret) {
			dev_err(lt->dev, "failed to get display timings\n");
			drm_mode_destroy(connector->dev, mode);
			return ret;
		}

		mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);

		num = 1;
	} else {
		edid = drm_get_edid(connector, lt->i2c->adapter);
		if (edid) {
			drm_connector_update_edid_property(connector, edid);
			num = drm_add_edid_modes(connector, edid);
			lt->sink_is_hdmi = !!drm_detect_hdmi_monitor(edid);
		} else {
			dev_err(lt->dev, "failed to get display EDID data\n");
		}
	}

	ret = drm_display_info_set_bus_formats(&connector->display_info,
					       &bus_format, 1);
	if (ret)
		return ret;

	return num;
}

static enum drm_mode_status lt8912_connector_mode_valid(struct drm_connector *connector,
			     struct drm_display_mode *mode)
{
	if (mode->clock > 150000)
		return MODE_CLOCK_HIGH;

	if (mode->hdisplay > 1920)
		return MODE_BAD_HVALUE;

	if (mode->vdisplay > 1080)
		return MODE_BAD_VVALUE;

	return MODE_OK;
}

static const struct drm_connector_helper_funcs lt8912_connector_helper_funcs = {
	.get_modes = lt8912_connector_get_modes,
	.best_encoder = lt8912_connector_best_encoder,
	.mode_valid = lt8912_connector_mode_valid,
};

static void lt8912_bridge_post_disable(struct drm_bridge *bridge)
{
	struct lt8912 *lt = bridge_to_lt8912(bridge);
	lt8912_sleep(lt);
}

static void lt8912_bridge_enable(struct drm_bridge *bridge)
{
	struct lt8912 *lt = bridge_to_lt8912(bridge);
	lt8912_init(lt);
}

static void lt8912_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct lt8912 *lt = bridge_to_lt8912(bridge);
	lt8912_wakeup(lt);
}

static void lt8912_bridge_mode_set(struct drm_bridge *bridge,
				   const struct drm_display_mode *mode,
				   const struct drm_display_mode *adj)
{
	struct lt8912 *lt = bridge_to_lt8912(bridge);

	drm_mode_copy(&lt->mode, adj);
}

static int lt8912_bridge_attach(struct drm_bridge *bridge,
				enum drm_bridge_attach_flags flags)
{
	struct lt8912 *lt = bridge_to_lt8912(bridge);
	struct drm_connector *connector = &lt->connector;
	int ret;

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(bridge->dev, connector,
				 &lt8912_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret) {
		dev_err(lt->dev, "failed to initialize connector\n");
		return ret;
	}

	drm_connector_helper_add(connector, &lt8912_connector_helper_funcs);
	drm_connector_attach_encoder(connector, bridge->encoder);

	if (!lt->no_hpd)
		enable_irq(lt->hpd_irq);

	return ret;
}

static const struct drm_bridge_funcs lt8912_bridge_funcs = {
	.attach = lt8912_bridge_attach,
	.mode_set = lt8912_bridge_mode_set,
	.pre_enable = lt8912_bridge_pre_enable,
	.enable = lt8912_bridge_enable,
	.post_disable = lt8912_bridge_post_disable,
};

static const struct regmap_config lt8912_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static int lt8912_i2c_init(struct lt8912 *lt,
			   struct i2c_client *client)
{
	struct i2c_board_info info[] = {
		{ I2C_BOARD_INFO("lt8912p0", 0x48), },
		{ I2C_BOARD_INFO("lt8912p1", 0x49), },
		{ I2C_BOARD_INFO("lt8912p2", 0x4a), }
	};
	struct regmap *regmap;
	unsigned int i;
	int ret;

	if (!lt || !client)
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(info); i++) {
		if (i > 0 ) {
			client = i2c_new_client_device(client->adapter, &info[i]);
			if (!client)
				return -ENODEV;
		}
		regmap = devm_regmap_init_i2c(client, &lt8912_regmap_config);
		if (IS_ERR(regmap)) {
			ret = PTR_ERR(regmap);
			dev_err(lt->dev,
				"Failed to initialize regmap: %d\n", ret);
			return ret;
		}

		lt->regmap[i] = regmap;
	}

	return 0;
}

int lt8912_attach_dsi(struct lt8912 *lt)
{
	struct device *dev = lt->dev;
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	int ret = 0;
	const struct mipi_dsi_device_info info = { .type = "lt8912",
						   .channel = lt->channel_id,
						   .node = NULL,
						 };

	host = of_find_mipi_dsi_host_by_node(lt->host_node);
	if (!host) {
		dev_err(dev, "failed to find dsi host\n");
		return -EPROBE_DEFER;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		dev_err(dev, "failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		goto err_dsi_device;
	}

	lt->dsi = dsi;

	dsi->lanes = lt->num_dsi_lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "failed to attach dsi to host\n");
		goto err_dsi_attach;
	}

	return 0;

err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
err_dsi_device:
	return ret;
}

void lt8912_detach_dsi(struct lt8912 *lt)
{
	mipi_dsi_detach(lt->dsi);
	mipi_dsi_device_unregister(lt->dsi);
}


static int lt8912_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct device *dev = &i2c->dev;
	struct lt8912 *lt;
	struct device_node *endpoint;
	int ret = 0;

	static int initialize_it = 1;

	if(!initialize_it) {
		initialize_it = 1;
		return -EPROBE_DEFER;
	}

	lt = devm_kzalloc(dev, sizeof(*lt), GFP_KERNEL);
	if (!lt)
		return -ENOMEM;

	lt->dev = dev;
	lt->is_suspended = false;

	/* get DT configuration */
	lt8912_parse_dt(dev->of_node, lt);

	lt->reset_n = devm_gpiod_get_optional(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(lt->reset_n)) {
		ret = PTR_ERR(lt->reset_n);
		dev_err(dev, "failed to request reset GPIO: %d\n", ret);
		return ret;
	}

	lt->vdd1 = devm_regulator_get_optional(dev, "vdd1");
	if (IS_ERR(lt->vdd1)) {
		ret = PTR_ERR(lt->vdd1);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		lt->vdd1 = NULL;
		dev_dbg(dev, "No vdd1 regulator found (%d)\n", ret);
	}

	if (lt->vdd1) {
		ret = regulator_enable(lt->vdd1);
		if (ret) {
			dev_err(dev, "Failed to enable vdd1 regulator\n");
			return ret;
		}
	}

	lt->vdd2 = devm_regulator_get_optional(dev, "vdd2");
	if (IS_ERR(lt->vdd2)) {
		ret = PTR_ERR(lt->vdd2);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		lt->vdd2 = NULL;
		dev_dbg(dev, "No vdd2 regulator found (%d)\n", ret);
	}

	if (lt->vdd2) {
		ret = regulator_enable(lt->vdd2);
		if (ret) {
			dev_err(dev, "Failed to enable vdd2 regulator\n");
			return ret;
		}
	}

	if (!lt->no_hpd) {
		lt->gpiod_int = devm_gpiod_get_optional(dev, "hpd", GPIOD_IN);
		if (IS_ERR(lt->gpiod_int)) {
			ret = PTR_ERR(lt->gpiod_int);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get hpd GPIO: %d\n",
					ret);
			return ret;
		}

		lt->hpd_irq = gpiod_to_irq(lt->gpiod_int);
		if (lt->hpd_irq < 0) {
			dev_err(dev, "Failed to get HPD IRQ: %d\n", lt->hpd_irq);
			return -ENODEV;
		}

		ret = devm_request_threaded_irq(dev, lt->hpd_irq, NULL,
						lt8912_hpd_threaded_handler,
						IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"lt8912-irq", lt);
		if (ret) {
			dev_err(dev,
				"Failed to request CABLE_DET threaded IRQ: %d\n",
				ret);
			return ret;
		}
		disable_irq(lt->hpd_irq);
	}

	lt->i2c = i2c;
	ret = lt8912_i2c_init(lt, lt->i2c);
	if (ret)
		return ret;

	lt->channel_id = 1;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint)
		return -ENODEV;

	lt->host_node = of_graph_get_remote_port_parent(endpoint);
	if (!lt->host_node) {
		of_node_put(endpoint);
		return -ENODEV;
	}

	of_node_put(endpoint);
	of_node_put(lt->host_node);

	lt->bridge.funcs = &lt8912_bridge_funcs;
	lt->bridge.of_node = dev->of_node;
	drm_bridge_add(&lt->bridge);
	ret = lt8912_attach_dsi(lt);
	if (ret)
		goto err_attach;

	return 0;

err_attach:
	return ret;
}

static void lt8912_remove(struct i2c_client *i2c)
{
	struct lt8912 *lt = i2c_get_clientdata(i2c);

	lt8912_sleep(lt);
	mipi_dsi_detach(lt->dsi);
	drm_bridge_remove(&lt->bridge);
}

static const struct i2c_device_id lt8912_i2c_ids[] = {
	{ "lt8912", 0 },
	{ }
};

static const struct of_device_id lt8912_of_match[] = {
	{ .compatible = "lontium,lt8912" },
	{}
};
MODULE_DEVICE_TABLE(of, lt8912_of_match);

static struct mipi_dsi_driver lt8912_driver = {
	.driver.name = "lt8912",
};

static struct i2c_driver lt8912_i2c_driver = {
	.driver = {
		.name = "lt8912",
		.of_match_table = lt8912_of_match,
	},
	.id_table = lt8912_i2c_ids,
	.probe = lt8912_probe,
	.remove = lt8912_remove,
};

static int __init lt8912_i2c_drv_init(void)
{
	mipi_dsi_driver_register(&lt8912_driver);

	return i2c_add_driver(&lt8912_i2c_driver);
}
module_init(lt8912_i2c_drv_init);

static void __exit lt8912_i2c_exit(void)
{
	i2c_del_driver(&lt8912_i2c_driver);

	mipi_dsi_driver_unregister(&lt8912_driver);
}
module_exit(lt8912_i2c_exit);

MODULE_AUTHOR("Wyon Bi <bivvy.bi@rock-chips.com>");
MODULE_DESCRIPTION("Lontium LT8912 MIPI-DSI to LVDS and HDMI/MHL bridge");
MODULE_LICENSE("GPL v2");
