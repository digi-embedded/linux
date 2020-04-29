/*
 * Licensed under the GPL-2.
 */

#include <linux/of.h>
#include <linux/of_graph.h>

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <video/of_display_timing.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include "dummy_dsi_bridge_timing.h"

#define DRVNAME			"dummy_dsi_bridge"
#define DEFAULT_WIDTH		149
#define DEFAULT_HEIGHT		93
#define DEFAULT_NUM_LANES	4
#define DEFAULT_VIDEO_MODE	1

struct dummy_dsi_bridge {
	struct device		*dev;
	/* Bridge dummy panel parameters */
	struct videomode	vm;
	u8			num_dsi_lanes;
	u8			video_mode;
	/* MIPI DSI parameters*/
	struct mipi_dsi_device	*dsi;
	/* DRM parameters */
	struct drm_display_mode	curr_mode;
	struct drm_bridge	bridge;
	struct drm_connector	connector;
};

/* Connector functions */
static struct dummy_dsi_bridge *connector_to_dsi_bridge(struct drm_connector *connector)
{
	return container_of(connector, struct dummy_dsi_bridge, connector);
}

static int dummy_dsi_bridge_connector_get_modes(struct drm_connector *connector)
{
	struct dummy_dsi_bridge *dsi_bridge = connector_to_dsi_bridge(connector);
	struct device *dev = dsi_bridge->dev;
	struct drm_display_mode *mode;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	u32 *bus_flags = &connector->display_info.bus_flags;
	int ret;

	dev_dbg(dev, "%s\n",__func__);

	/* Create and initialize default mode */
	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_DEV_ERROR(dev, "Failed to create display mode!\n");
		return 0;
	}

	drm_display_mode_from_videomode(&dsi_bridge->vm, mode);
	mode->width_mm = DEFAULT_WIDTH;
	mode->height_mm = DEFAULT_HEIGHT;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	drm_mode_probed_add(connector, mode);
	drm_connector_list_update(connector);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	if (dsi_bridge->vm.flags & DISPLAY_FLAGS_DE_HIGH)
		*bus_flags |= DRM_BUS_FLAG_DE_HIGH;
	if (dsi_bridge->vm.flags & DISPLAY_FLAGS_DE_LOW)
		*bus_flags |= DRM_BUS_FLAG_DE_LOW;
	if (dsi_bridge->vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_NEGEDGE;
	if (dsi_bridge->vm.flags & DISPLAY_FLAGS_PIXDATA_POSEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_POSEDGE;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
					       &bus_format, 1);
	if (ret)
	return ret;

	return 1;
}

static enum drm_connector_status
dummy_dsi_bridge_connector_detect(struct drm_connector *connector, bool force)
{
	struct dummy_dsi_bridge *dsi_bridge = connector_to_dsi_bridge(connector);
	struct device *dev = dsi_bridge->dev;

	dev_dbg(dev, "%s\n",__func__);
	return connector_status_connected;
}

int drm_helper_probe_single_connector_modes(struct drm_connector *connector,
                        uint32_t maxX, uint32_t maxY);

static struct drm_connector_funcs dummy_dsi_bridge_connector_funcs = {
	.detect = dummy_dsi_bridge_connector_detect,
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct drm_connector_helper_funcs
	dummy_dsi_bridge_connector_helper_funcs = {
	.get_modes = dummy_dsi_bridge_connector_get_modes,
};

/* Bridge functions */
static void dummy_dsi_bridge_mode_set(struct drm_bridge *bridge,
				      const struct drm_display_mode *mode,
				      const struct drm_display_mode *adjusted)
{
	struct dummy_dsi_bridge *dsi_bridge = bridge->driver_private;
	struct device *dev = dsi_bridge->dev;

	dev_dbg(dev, "%s: mode: %d*%d@%d\n",__func__,
		mode->hdisplay,mode->vdisplay,mode->clock);
	drm_mode_copy(&dsi_bridge->curr_mode, adjusted);
}

static int dummy_dsi_bridge_attach_host_dsi(struct dummy_dsi_bridge *dsi_bridge)
{
	struct device *dev = dsi_bridge->dev;
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	struct device_node *np = dev->of_node;
	struct device_node *endpoint,*remote_endpoint;
	int ret = 0;
	const struct mipi_dsi_device_info info = { .type = "dummy_dsi_bridge",
						   .channel = 0,
						   .node = NULL,
						 };

	dev_dbg(dev, "%s\n",__func__);
	endpoint = of_graph_get_next_endpoint(np, NULL);
	if (!endpoint) {
		dev_err(dev, "no endpoint found!\n");
		return -ENODEV;
	}

	remote_endpoint = of_graph_get_remote_port_parent(endpoint);
	if (!remote_endpoint) {
		of_node_put(endpoint);
		dev_err(dev, "no remote endpoint found!\n");
		return -ENODEV;
	}
	of_node_put(endpoint);
	of_node_put(remote_endpoint);

	host = of_find_mipi_dsi_host_by_node(remote_endpoint);
	if (!host) {
		dev_err(dev, "failed to find dsi host\n");
		return -EPROBE_DEFER;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		dev_err(dev, "failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		return -ENODEV;
	}

	dsi_bridge->dsi = dsi;
	dsi->lanes = dsi_bridge->num_dsi_lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | dsi_bridge->video_mode;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "failed to attach dsi to host\n");
		mipi_dsi_device_unregister(dsi);
	}

	return ret;
}

static int dummy_dsi_bridge_attach(struct drm_bridge *bridge)
{
	struct dummy_dsi_bridge *dsi_bridge = bridge->driver_private;
	struct device *dev = dsi_bridge->dev;
	int ret;

	dev_dbg(dev,"%s\n",__func__);
	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found");
		return -ENODEV;
	}

	/* Create the connector */
	dsi_bridge->connector.polled = DRM_CONNECTOR_POLL_CONNECT;

	ret = drm_connector_init(bridge->dev, &dsi_bridge->connector,
				 &dummy_dsi_bridge_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}
	drm_connector_helper_add(&dsi_bridge->connector,
				 &dummy_dsi_bridge_connector_helper_funcs);
	drm_connector_attach_encoder(&dsi_bridge->connector, bridge->encoder);

	/* Attach the Host MIPI DSI interface */
	ret = dummy_dsi_bridge_attach_host_dsi(dsi_bridge);

	return ret;
}

static void dummy_dsi_bridge_detach(struct drm_bridge *bridge)
{
	struct dummy_dsi_bridge *dsi_bridge = bridge->driver_private;
	struct device *dev = dsi_bridge->dev;

	dev_dbg(dev, "%s\n",__func__);
	mipi_dsi_detach(dsi_bridge->dsi);
	mipi_dsi_device_unregister(dsi_bridge->dsi);
}

static struct drm_bridge_funcs dummy_dsi_bridge_funcs = {
	.mode_set = dummy_dsi_bridge_mode_set,
	.attach = dummy_dsi_bridge_attach,
	.detach = dummy_dsi_bridge_detach,
};

/* Driver functions */
static int dummy_dsi_bridge_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dummy_dsi_bridge *dsi_bridge;
	u32 num_lanes = DEFAULT_NUM_LANES;
	u32 video_mode = DEFAULT_VIDEO_MODE;
	int ret = 0;

	dev_dbg(dev,"%s\n",__func__);
	if (!dev->of_node)
		return -EINVAL;

	dsi_bridge = devm_kzalloc(dev, sizeof(*dsi_bridge), GFP_KERNEL);
	if (!dsi_bridge)
		return -ENOMEM;

	/* Parse DT node */
	of_property_read_u32(dev->of_node, "dsi-lanes", &num_lanes);
	if (num_lanes < 1 || num_lanes > 4) {
		dev_err(dev, "invalid dsi-lanes: %d\n", num_lanes);
		return -EINVAL;
	}
	dsi_bridge->num_dsi_lanes = num_lanes;

	of_property_read_u32(dev->of_node, "video-mode", &video_mode);
	switch (video_mode) {
	case 0:
		/* Video burst mode */
		dsi_bridge->video_mode = MIPI_DSI_MODE_VIDEO_BURST;
		break;
	case 1:
		/* Video pulse mode */
		dsi_bridge->video_mode = MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
		break;
	default:
		dev_err(dev, "invalid video-mode %d\n", video_mode);
		return -EINVAL;
	}

	/* Read default timing if device tree node is not found */
	if ((of_get_videomode(dev->of_node, &dsi_bridge->vm, OF_USE_NATIVE_MODE)) < 0)
		videomode_from_timing(&panel_default_timing, &dsi_bridge->vm);

	dsi_bridge->dev = dev;
	platform_set_drvdata(pdev, dsi_bridge);

	dsi_bridge->bridge.driver_private = dsi_bridge;
	dsi_bridge->bridge.funcs = &dummy_dsi_bridge_funcs;
	dsi_bridge->bridge.of_node = dev->of_node;
	drm_bridge_add(&dsi_bridge->bridge);

	return ret;
}

static int dummy_dsi_bridge_remove(struct platform_device *pdev)
{
	struct dummy_dsi_bridge *dsi_bridge = platform_get_drvdata(pdev);
	struct device *dev = dsi_bridge->dev;

	dev_dbg(dev, "%s\n",__func__);
	drm_bridge_remove(&dsi_bridge->bridge);
	return 0;
}

static const struct of_device_id dummy_dsi_bridge_of_ids[] = {
	{ .compatible = "digi,dummy_dsi_bridge" },
	{ }
};
MODULE_DEVICE_TABLE(of, dummy_dsi_bridge_of_ids);

static struct mipi_dsi_driver dummy_dsi_bridge_dsi_driver = {
	.driver.name = DRVNAME,
};

static struct platform_driver dummy_dsi_bridge_driver = {
	.probe = dummy_dsi_bridge_probe,
	.remove = dummy_dsi_bridge_remove,
	.driver = {
		.name = DRVNAME,
		.owner = THIS_MODULE,
		.of_match_table = dummy_dsi_bridge_of_ids,
	},
};

static int __init dummy_dsi_bridge_init(void)
{
	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_register(&dummy_dsi_bridge_dsi_driver);

	return platform_driver_register(&dummy_dsi_bridge_driver);
}
module_init(dummy_dsi_bridge_init);

static void __exit dummy_dsi_bridge_exit(void)
{
	platform_driver_unregister(&dummy_dsi_bridge_driver);

	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_unregister(&dummy_dsi_bridge_dsi_driver);
}
module_exit(dummy_dsi_bridge_exit);

/* Module information */
MODULE_AUTHOR("Digi International Inc.");
MODULE_DESCRIPTION("Dummy DSI bridge driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
MODULE_ALIAS(DRVNAME);
