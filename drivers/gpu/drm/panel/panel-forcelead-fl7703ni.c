// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024, Digi International Inc.
 *
 * Author: Arturo Buzarra <arturo.buzarra@digi.com>
 *         Gonzalo Ruiz <gonzalo.ruiz@digi.com>
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

struct fl7703ni;

struct fl7703ni_panel_desc {
	const struct drm_display_mode *mode;
	enum mipi_dsi_pixel_format format;
	unsigned int panel_sleep_delay;
	void (*gip_sequence)(struct fl7703ni *fl7703ni);
};

struct fl7703ni {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	const struct fl7703ni_panel_desc *desc;
	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset;
	unsigned int sleep_delay;
};

static inline struct fl7703ni *panel_to_fl7703ni(struct drm_panel *panel)
{
	return container_of(panel, struct fl7703ni, panel);
}

static inline int fl7703ni_dsi_write(struct fl7703ni *fl7703ni, const void *seq,
				   size_t len)
{
	return mipi_dsi_dcs_write_buffer(fl7703ni->dsi, seq, len);
}

#define FL7703NI_DSI(fl7703ni, seq...)				\
	{							\
		const u8 d[] = { seq };				\
		fl7703ni_dsi_write(fl7703ni, d, ARRAY_SIZE(d));	\
	}

static int fl7703ni_read_id(struct fl7703ni *fl7703ni)
{
	u8 id1, id2, id3;
	int ret;

	ret = mipi_dsi_dcs_read(fl7703ni->dsi, 0xDA, &id1, 1);
	if (ret < 0) {
		dev_err(&fl7703ni->dsi->dev, "could not read ID1\n");
		return ret;
	}

	ret = mipi_dsi_dcs_read(fl7703ni->dsi, 0xDB, &id2, 1);
	if (ret < 0) {
		dev_err(&fl7703ni->dsi->dev, "could not read ID2\n");
		return ret;
	}

	ret = mipi_dsi_dcs_read(fl7703ni->dsi, 0xDC, &id3, 1);
	if (ret < 0) {
		dev_err(&fl7703ni->dsi->dev, "could not read ID3\n");
		return ret;
	}

	dev_info(&fl7703ni->dsi->dev,
		     "manufacturer: %02x version: %02x driver: %02x\n",
		     id1, id2, id3);

	return 0;
}

/* General initialization Packet (GIP) sequence */
static void nhd_35_640480ef_gip_sequence(struct fl7703ni *fl7703ni)
{
	const unsigned int lanes = fl7703ni->dsi->lanes;
	int lane_config;

	FL7703NI_DSI(fl7703ni, MIPI_DCS_SOFT_RESET);

	/* We need to wait 5ms before sending new commands */
	msleep(5);

	/* Enable USER command */
	FL7703NI_DSI(fl7703ni, 0xB9,  /* SETEXTC command */
			       0xF1, 0x12, 0x87);

	/* Set Display resolution */
	FL7703NI_DSI(fl7703ni, 0xB2,  /* SETDISP command */
			       0x78,  /* 480Gate(120*4+0) */
			       0x14,  /* 640RGB */
			       0x70);

	/* Set RGB */
	FL7703NI_DSI(fl7703ni, 0xB3,  /* SETRGBIF command */
			       0x10,  /* VBP_RGB_GEN */
			       0x10,  /* VFP_RGB_GEN */
			       0x28,  /* DE_BP_RGB_GEN */
			       0x28,  /* DE_FP_RGB_GEN */
			       0x03, 0xFF, 0x00, 0x00, 0x00, 0x00);

	/* Set Panel Inversion */
	FL7703NI_DSI(fl7703ni, 0xB4,  /* SETCYC command */
			       0x80); /* Zig-zag inversion type D */

	/* Set VREF/NVREF voltage */
	FL7703NI_DSI(fl7703ni, 0xB5,  /* SETBGP command */
			       0x0A,  /* VREF */
			       0x0A); /* NVREF */

	/* Set VCOM voltage */
	FL7703NI_DSI(fl7703ni, 0xB6,  /* SETVCOM command */
			       0x70,  /* F_VCOM */
			       0x70); /* B_VCOM */

	/* Set ECP */
	FL7703NI_DSI(fl7703ni, 0xB8,  /* SETPOWER_EXT command */
			       0x26);

	switch (lanes) {
	case 1:
		lane_config = DSI_CMD_SETMIPI_1LANE;
		break;
	case 2:
		lane_config = DSI_CMD_SETMIPI_2LANE;
		break;
	case 3:
		lane_config = DSI_CMD_SETMIPI_3LANE;
		break;
	case 4:
		lane_config = DSI_CMD_SETMIPI_4LANE;
	default:
		break;
	}

	/* Set MIPI configurations */
	FL7703NI_DSI(fl7703ni, 0xBA,  /* SETMIPI command */
			       0x30|lane_config, /* Lane_Number[1:0] */
			       0x81, 0x05, 0xF9, 0x0E, 0x0E, 0x20, 0x00, 0x00,
			       0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x25, 0x00,
			       0x91, 0x0A, 0x00, 0x00, 0x01, 0x4F, 0x01, 0x00,
			       0x00, 0x37);

	/* Set NVDDD/VDDD voltage */
	FL7703NI_DSI(fl7703ni, 0xBC,  /* SETVDC command */
			       0x47);

	/* Set PCR settings */
	FL7703NI_DSI(fl7703ni, 0xBF,
			       0x02, 0x11, 0x00);

	/* Set Source driving settings */
	FL7703NI_DSI(fl7703ni, 0xC0,  /* SETSCR command */
			       0x73, 0x73, 0x50, 0x50, 0x00, 0x00, 0x12, 0x73,
			       0x00);

	/* Set POWER settings */
	FL7703NI_DSI(fl7703ni, 0xC1,  /* SETPOWER command */
			       0x54,  /* VBTHS VBTLS */
			       0x00,
			       0x32,  /* VSPR */
			       0x32,  /* VSNR */
			       0x77,  /* VSP VSN */
			       0xF4,  /* APS */
			       0x77,  /* VGH1_L VGL1_L */
			       0x77,  /* VGH1_R VGL1_R */
			       0xCC,  /* VGH2_L VGL2_L */
			       0xCC,  /* VGH2_R VGL2_R */
			       0xFF,  /* VGH3_L VGL3_L */
			       0xFF,  /* VGH3_R VGL3_R */
			       0x11,  /* VGH4_L VGL4_L */
			       0x11,  /* VGH4_R VGL4_R */
			       0x00,  /* VGH5_L VGL5_L */
			       0x00,  /* VGH5_R VGL5_R */
			       0x31); /* VSP VSN DET EN */

	/* Set I/O settings */
	FL7703NI_DSI(fl7703ni, 0xC7,  /* SETIO command */
			       0x10,  /* Enable VOUT output*/
			       0x00, 0x0A, 0x00, 0x00,
			       0x00,  /* Set VOUT at 3.3V */
			       0x00,  /* D6-5:HOUT_SEL | D1-0:VOUT_SEL */
			       0x00,  /* D1-0:PWM_SEL */
			       0xED,  /* D7:MIPI_ERR_DIS_TE
					 D6:VGL_DET_DIS_TE
			                 D5:VGH_SET_DIS_TE
					 D4:DBV_ZERO_DIS_TE
					 D3:LVPUR_DIS_TE
					 D2:TE_ONLY_AT_NORMAL
					 D1:CRC_MATC
					 D0:REF_CRC_DIS_TE */
			       0xC7,  /* D7:OLED_CMD
					 D6:N_CMD_EN
					 D5:HS_STOP_CMD_EN
					 D4:D6:MIPI_STOP_09_PAR3_EN
					 D3:CHECK_VSYNC
					 D2:MIPI_STOP_09_EN
					 D1:TE_STOP_RD_EN
					 D0:STATE_ERR_READ_EN */
			       0x00,
			       0xA5);

	/* Set CABC settings */
	FL7703NI_DSI(fl7703ni, 0xC8,  /* SETCABC command */
			       0x10, 0x40, 0x1E, 0x03);

	/* Set Panel settings */
	FL7703NI_DSI(fl7703ni, 0xCC,  /* SETPANEL command */
			       0x0B);

	/* Set Asymmetric gamma2.2 */
	FL7703NI_DSI(fl7703ni, 0xE0,  /* SETGAMMA command */
			       0x00, 0x05, 0x09, 0x29, 0x3C, 0x3F, 0x3B, 0x37,
			       0x05, 0x0A, 0x0C, 0x10, 0x13, 0x10, 0x13, 0x12,
			       0x1A, 0x00, 0x05, 0x09, 0x29, 0x3C, 0x3F, 0x3B,
			       0x37, 0x05, 0x0A, 0x0C, 0x10, 0x13, 0x10, 0x13,
			       0x12, 0x1A);

	FL7703NI_DSI(fl7703ni, 0xE1,
			       0x11, 0x11, 0x91,
			       0x00,  /* D7-5:VGH_SET_SEL | D4-0:VGL_DET_SEL */
			       0x00,  /* D5-0:VSN_DET_SE */
			       0x00,  /* D5-0:VSP_SET_SE */
			       0x00); /* D7:PUREN_IOVCC
					 D6-4:IOVCC_PUR_SEL
					 D3-2:DCHG1
					 D1-0:DCHG2 */

	/* Set EQ settings */
	FL7703NI_DSI(fl7703ni, 0xE3,  /* SETEQ command */
			       0x07, 0x07, 0x0B, 0x0B, 0x03, 0x0B, 0x00, 0x00,
			       0x00, 0x00, 0xFF, 0x04, 0xC0, 0x10);

	/* Set GIP */
	FL7703NI_DSI(fl7703ni, 0xE9,  /* SETGIP1 command */
			       0x01,  /* PANEL_SEL */
			       0x00, 0x0E, /* SHR_0 */
			       0x00, 0x00, /* SHR_1 */
			       0xB0, 0xB1, /* SPON SPOFF */
			       0x11, 0x31, /* SHR0_1 SHR0_2 SHR0_3 SHR1_1 */
			       0x23,  /* SHR1_2 SHR1_3 */
			       0x28,  /* SHP SCP */
			       0x10,  /* CHR */
			       0xB0,  /* CON */
			       0xB1,  /* COFF */
			       0x27,  /* CHP CCP */
			       0x08,  /* USER_GIP_GATE */
			       0x00, 0x04, 0x02, /* CGTS_L (Gout2 & 11 = STV) */
			       0x00, 0x00, 0x00, /* CGTS_INV_L */
			       0x00, 0x04, 0x02, /* CGTS_R (Gout2 & 11 = STV) */
			       0x00, 0x00, 0x00, /* CGTS_INV_R */
			       0x88, 0x88, 0xBA, 0x60, /* COS1_L  - COS8_L */
			       0x24, 0x08, 0x88, 0x88, /* COS9_L  - COS16_L */
			       0x88, 0x88, 0x88,       /* COS17_L - COS22_L */
			       0x88, 0x88, 0xBA, 0x71, /* COS1_R  - COS8_R */
			       0x35, 0x18, 0x88, 0x88, /* COS9_R  - COS16_R */
			       0x88, 0x88, 0x88,       /* COS17_R - COS22_R */
			       0x00,  /* TCON_OPT */
			       0x00, 0x00, 0x01, /* GIP_OPT */
			       0x00,  /* CHR2 */
			       0x00,  /* CON2 */
			       0x00,  /* COFF2 */
			       0x00,  /* CHP2 CCP2 */
			       0x00, 0x00, 0x00, /* CKS */
			       0x00,  /* COFF CON SPOFF SPON */
			       0x00); /* COFF2 CON2 PANEL_SEL_INI */

	/* Set GIP2 */
	FL7703NI_DSI(fl7703ni, 0xEA,  /* SETGIP2 command */
			       0x97,  /* YS2_SEL YS1_SEL */
			       0x0A,  /* USER_GIP_GATE1 */
			       0x82,  /* CK_ALL_ON_WIDTH1 */
			       0x02,  /* CK_ALL_ON_WIDTH2 */
			       0x03,  /* CK_ALL_ON_WIDTH3 */
			       0x07,  /* YS_FLAG_PERIOD */
			       0x00,  /* YS2_SEL_2 YS1_SEL_2 */
			       0x00,  /* USER_GIP_GATE1_2 */
			       0x00,  /* CK_ALL_ON_WIDTH1_2 */
			       0x00,  /* CK_ALL_ON_WIDTH2_2 */
			       0x00,  /* CK_ALL_ON_WIDTH3_2 */
			       0x00,  /* YS_FLAG_PERIOD_2 */
			       0x81, 0x88, 0xBA, 0x17, /* COS1_L  - COS8_L */
			       0x53, 0x88, 0x88, 0x88, /* COS9_L  - COS16_L */
			       0x88, 0x88, 0x88,       /* COS17_L - COS22_L */
			       0x80, 0x88, 0xBA, 0x06, /* COS1_R  - COS8_R */
			       0x42, 0x88, 0x88, 0x88, /* COS9_R  - COS16_R */
			       0x88, 0x88, 0x88,       /* COS17_R - COS22_R */
			       0x23,  /* EQOPT EQSEL */
			       0x00,  /* EQ_DELAY */
			       0x00,  /* EQ2_DELAY EQ_DELAY_HSYNC */
			       0x02, 0x80, /* HSYNC_TO_CL1_CNT10 */
			       0x00,  /* HIZ_L */
			       0x00,  /* HIZ_R */
			       0x00, 0x00, 0x00, /* CKS_GS */
			       0x00, 0x00, 0x00, /* CK_MSB_EN */
			       0x00, 0x00, 0x00, /* CK_MSB_EN_GS */
			       0x00, 0x00, /* SHR2 */
			       0x00, 0x00, /* SHR2_1 SHR2_2 SHR2_3 */
			       0x00, 0x00, 0x00, /* SHP1 SPON1 SPOFF1 */
			       0x00, 0x00, 0x00, /* SHP2 SPON2 SPOFF2 */
			       0x00);  /* SPOFF2 SPON2 SPOFF1 SPON1 */

	FL7703NI_DSI(fl7703ni, 0xEF,
			       0xFF, 0xFF, 0x01);

	/* Sleep Out */
	FL7703NI_DSI(fl7703ni, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(250);
}


static int fl7703ni_prepare(struct drm_panel *panel)
{
	struct fl7703ni *fl7703ni = panel_to_fl7703ni(panel);
	int ret;

	gpiod_set_value(fl7703ni->reset, 0);

	ret = regulator_bulk_enable(ARRAY_SIZE(fl7703ni->supplies),
				    fl7703ni->supplies);
	if (ret < 0)
		return ret;
	msleep(20);

	gpiod_set_value(fl7703ni->reset, 1);
	msleep(150);

	fl7703ni_read_id(fl7703ni);

	if (fl7703ni->desc->gip_sequence)
		fl7703ni->desc->gip_sequence(fl7703ni);

	return 0;
}

static int fl7703ni_enable(struct drm_panel *panel)
{
	struct fl7703ni *fl7703ni = panel_to_fl7703ni(panel);

	FL7703NI_DSI(fl7703ni, MIPI_DCS_SET_DISPLAY_ON);
	msleep(50);

	return 0;
}

static int fl7703ni_disable(struct drm_panel *panel)
{
	struct fl7703ni *fl7703ni = panel_to_fl7703ni(panel);

	FL7703NI_DSI(fl7703ni, MIPI_DCS_SET_DISPLAY_OFF);
	msleep(50);

	return 0;
}

static int fl7703ni_unprepare(struct drm_panel *panel)
{
	struct fl7703ni *fl7703ni = panel_to_fl7703ni(panel);

	FL7703NI_DSI(fl7703ni, MIPI_DCS_ENTER_SLEEP_MODE);

	msleep(fl7703ni->sleep_delay);

	gpiod_set_value(fl7703ni->reset, 0);

	msleep(fl7703ni->sleep_delay);

	regulator_bulk_disable(ARRAY_SIZE(fl7703ni->supplies), fl7703ni->supplies);

	return 0;
}

static int fl7703ni_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	struct fl7703ni *fl7703ni = panel_to_fl7703ni(panel);
	const struct drm_display_mode *desc_mode = fl7703ni->desc->mode;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, desc_mode);
	if (!mode) {
		dev_err(&fl7703ni->dsi->dev, "failed to add mode %ux%u@%u\n",
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

static const struct drm_panel_funcs fl7703ni_funcs = {
	.disable	= fl7703ni_disable,
	.unprepare	= fl7703ni_unprepare,
	.prepare	= fl7703ni_prepare,
	.enable		= fl7703ni_enable,
	.get_modes	= fl7703ni_get_modes,
};

static const struct drm_display_mode nhd_35_640480ef_mode = {
	.clock          = 31200,

	.hdisplay       = 640,
	.hsync_start    = 640 + 120,             /* hdisplay + hfront */
	.hsync_end      = 640 + 120 + 120,       /* hdisplay + hfront + hsync */
	.htotal         = 640 + 120 + 120 + 120, /* hdisplay + hfront + hsync + hback */

	.vdisplay       = 480,
	.vsync_start    = 480 + 25,              /* vdisplay + vfront */
	.vsync_end      = 480 + 25 + 5,          /* vdisplay + vfront + vsync */
	.vtotal         = 480 + 25 + 5 + 12,     /* vdisplay + vfront + vsync + vback */

	.width_mm       = 56,
	.height_mm      = 78,

	.flags          = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,

	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static const struct fl7703ni_panel_desc nhd_35_640480ef_desc = {
	.mode = &nhd_35_640480ef_mode,
	.format = MIPI_DSI_FMT_RGB888,
	.panel_sleep_delay = 0,
	.gip_sequence = nhd_35_640480ef_gip_sequence,
};

static int fl7703ni_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *np = dev->of_node;
	const struct fl7703ni_panel_desc *desc;
	struct fl7703ni *fl7703ni;
	int ret;

	fl7703ni = devm_kzalloc(&dsi->dev, sizeof(*fl7703ni), GFP_KERNEL);
	if (!fl7703ni)
		return -ENOMEM;

	desc = of_device_get_match_data(&dsi->dev);
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	dsi->format = desc->format;
	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret) {
		dev_err(&dsi->dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	fl7703ni->supplies[0].supply = "VCC";
	fl7703ni->supplies[1].supply = "IOVCC";

	ret = devm_regulator_bulk_get(&dsi->dev, ARRAY_SIZE(fl7703ni->supplies),
				      fl7703ni->supplies);
	if (ret < 0)
		return ret;

	fl7703ni->reset = devm_gpiod_get(&dsi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(fl7703ni->reset)) {
		dev_err(&dsi->dev, "Couldn't get our reset GPIO\n");
		return PTR_ERR(fl7703ni->reset);
	}

	drm_panel_init(&fl7703ni->panel, &dsi->dev, &fl7703ni_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	/**
	 * Once sleep out has been issued, FL7703NI IC required to wait 120ms
	 * before initiating new commands.
	 */
	fl7703ni->sleep_delay = 120 + desc->panel_sleep_delay;

	ret = drm_panel_of_backlight(&fl7703ni->panel);
	if (ret)
		return ret;

	drm_panel_add(&fl7703ni->panel);

	mipi_dsi_set_drvdata(dsi, fl7703ni);
	fl7703ni->dsi = dsi;
	fl7703ni->desc = desc;

	ret = mipi_dsi_attach(dsi);
	if (ret)
		goto err_attach;

	return 0;

err_attach:
	drm_panel_remove(&fl7703ni->panel);
	return ret;
}

static void fl7703ni_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct fl7703ni *fl7703ni = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&fl7703ni->panel);
}

static const struct of_device_id fl7703ni_of_match[] = {
	{ .compatible = "newhaven,nhd-3.5-640480ef-msxp", .data = &nhd_35_640480ef_desc },
	{ }
};
MODULE_DEVICE_TABLE(of, fl7703ni_of_match);

static struct mipi_dsi_driver fl7703ni_dsi_driver = {
	.probe		= fl7703ni_dsi_probe,
	.remove		= fl7703ni_dsi_remove,
	.driver = {
		.name		= "fl7703ni",
		.of_match_table	= fl7703ni_of_match,
	},
};
module_mipi_dsi_driver(fl7703ni_dsi_driver);

MODULE_AUTHOR("Arturo Buzarra <arturo.buzarra@digi.com>, Gonzalo Ruiz <gonzalo.ruiz@digi.com>");
MODULE_DESCRIPTION("ForceLEAD FL7703NI TFT LCD Panel Driver");
MODULE_LICENSE("GPL");
