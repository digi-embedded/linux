// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for STM32 Digital Camera Memory Interface Pixel Processor
 *
 * Copyright (C) STMicroelectronics SA 2023
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          Alain Volmat <alain.volmat@foss.st.com>
 *          for STMicroelectronics.
 */

#include <linux/v4l2-mediabus.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>

#include "dcmipp-common.h"

#define DCMIPP_PRCR	0x104
#define DCMIPP_PRCR_FORMAT_SHIFT	16
#define DCMIPP_PRCR_FORMAT_YUV422	0x1e
#define DCMIPP_PRCR_FORMAT_RGB565	0x22
#define DCMIPP_PRCR_FORMAT_RGB888	0x24
#define DCMIPP_PRCR_FORMAT_RAW8		0x2a
#define DCMIPP_PRCR_FORMAT_RAW10	0x2b
#define DCMIPP_PRCR_FORMAT_RAW12	0x2c
#define DCMIPP_PRCR_FORMAT_RAW14	0x2d
#define DCMIPP_PRCR_FORMAT_G8		0x4a
#define DCMIPP_PRCR_FORMAT_G10		0x4b
#define DCMIPP_PRCR_FORMAT_G12		0x4c
#define DCMIPP_PRCR_FORMAT_G14		0x4d
#define DCMIPP_PRCR_FORMAT_BYTE_STREAM	0x5a
#define DCMIPP_PRCR_ESS			BIT(4)
#define DCMIPP_PRCR_PCKPOL		BIT(5)
#define DCMIPP_PRCR_HSPOL		BIT(6)
#define DCMIPP_PRCR_VSPOL		BIT(7)
#define DCMIPP_PRCR_ENABLE		BIT(14)
#define DCMIPP_PRCR_SWAPCYCLES		BIT(25)

#define DCMIPP_PRESCR	0x108
#define DCMIPP_PRESUR	0x10c

#define DCMIPP_CMCR	0x204
#define DCMIPP_CMCR_INSEL	BIT(0)

#define DCMIPP_P0FSCR	0x404
#define DCMIPP_P1FSCR	0x804
#define DCMIPP_P2FSCR	0xC04
#define DCMIPP_P1FSCR_PIPEDIFF		BIT(18)
#define DCMIPP_PxFSCR_DTMODE_MASK	GENMASK(17, 16)
#define DCMIPP_PxFSCR_DTMODE_SHIFT	16
#define DCMIPP_PxFSCR_DTMODE_DTIDA	0x00
#define DCMIPP_P0FSCR_DTMODE_ALLDT	0x03
#define DCMIPP_PxFSCR_DTIDA_MASK	GENMASK(5, 0)
#define DCMIPP_PxFSCR_DTIDA_SHIFT	0

#define DCMIPP_PxFSCR(a) (((a) == 0) ? DCMIPP_P0FSCR :\
			  ((a) == 1) ? DCMIPP_P1FSCR :\
			   DCMIPP_P2FSCR)

#define IS_SINK(pad) (!(pad))
#define IS_SRC(pad)  ((pad))

struct dcmipp_inp_pix_map {
	unsigned int code_sink;
	unsigned int code_src;
	/* Parallel related information */
	u8 prcr_format;
	u8 prcr_swapcycles;
	/* CSI related information */
	unsigned int dt;
};

#define PIXMAP_SINK_SRC_PRCR_SWAP(sink, src, prcr, swap, data_type)	\
	{							\
		.code_sink = MEDIA_BUS_FMT_##sink,		\
		.code_src = MEDIA_BUS_FMT_##src,		\
		.prcr_format = DCMIPP_PRCR_FORMAT_##prcr,	\
		.prcr_swapcycles = swap,			\
		.dt = data_type,				\
	}
static const struct dcmipp_inp_pix_map dcmipp_inp_pix_map_list[] = {
	/* RGB565 */
	PIXMAP_SINK_SRC_PRCR_SWAP(RGB565_2X8_LE, RGB565_2X8_LE, RGB565, 1, MIPI_CSI2_DT_RGB565),
	PIXMAP_SINK_SRC_PRCR_SWAP(RGB565_2X8_BE, RGB565_2X8_LE, RGB565, 0, MIPI_CSI2_DT_RGB565),
	PIXMAP_SINK_SRC_PRCR_SWAP(RGB565_1X16, RGB565_1X16, RGB565, 0, MIPI_CSI2_DT_RGB565),
	/* RGB888 */
	PIXMAP_SINK_SRC_PRCR_SWAP(RGB888_3X8, RGB888_3X8, RGB888, 0, MIPI_CSI2_DT_RGB888),
	PIXMAP_SINK_SRC_PRCR_SWAP(RGB888_1X24, RGB888_1X24, RGB888, 0, MIPI_CSI2_DT_RGB888),
	/* YUV422 */
	PIXMAP_SINK_SRC_PRCR_SWAP(YUYV8_2X8, YUYV8_2X8, YUV422, 0, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_SINK_SRC_PRCR_SWAP(YUYV8_1X16, YUYV8_1X16, YUV422, 0, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_SINK_SRC_PRCR_SWAP(YUYV8_2X8, UYVY8_2X8, YUV422, 1, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_SINK_SRC_PRCR_SWAP(UYVY8_2X8, UYVY8_2X8, YUV422, 0, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_SINK_SRC_PRCR_SWAP(UYVY8_1X16, UYVY8_1X16, YUV422, 0, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_SINK_SRC_PRCR_SWAP(UYVY8_2X8, YUYV8_2X8, YUV422, 1, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_SINK_SRC_PRCR_SWAP(YVYU8_2X8, YVYU8_2X8, YUV422, 0, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_SINK_SRC_PRCR_SWAP(YVYU8_1X16, YVYU8_1X16, YUV422, 0, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_SINK_SRC_PRCR_SWAP(VYUY8_2X8, VYUY8_2X8, YUV422, 0, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_SINK_SRC_PRCR_SWAP(VYUY8_1X16, VYUY8_1X16, YUV422, 0, MIPI_CSI2_DT_YUV422_8B),
	/* GREY */
	PIXMAP_SINK_SRC_PRCR_SWAP(Y8_1X8, Y8_1X8, G8, 0, MIPI_CSI2_DT_RAW8),
	PIXMAP_SINK_SRC_PRCR_SWAP(Y10_1X10, Y10_1X10, G10, 0, MIPI_CSI2_DT_RAW10),
	PIXMAP_SINK_SRC_PRCR_SWAP(Y12_1X12, Y12_1X12, G12, 0, MIPI_CSI2_DT_RAW12),
	PIXMAP_SINK_SRC_PRCR_SWAP(Y14_1X14, Y14_1X14, G14, 0, MIPI_CSI2_DT_RAW14),
	/* Raw Bayer */
	PIXMAP_SINK_SRC_PRCR_SWAP(SBGGR8_1X8, SBGGR8_1X8, RAW8, 0, MIPI_CSI2_DT_RAW8),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGBRG8_1X8, SGBRG8_1X8, RAW8, 0, MIPI_CSI2_DT_RAW8),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGRBG8_1X8, SGRBG8_1X8, RAW8, 0, MIPI_CSI2_DT_RAW8),
	PIXMAP_SINK_SRC_PRCR_SWAP(SRGGB8_1X8, SRGGB8_1X8, RAW8, 0, MIPI_CSI2_DT_RAW8),
	PIXMAP_SINK_SRC_PRCR_SWAP(SBGGR10_1X10, SBGGR10_1X10, RAW10, 0, MIPI_CSI2_DT_RAW10),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGBRG10_1X10, SGBRG10_1X10, RAW10, 0, MIPI_CSI2_DT_RAW10),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGRBG10_1X10, SGRBG10_1X10, RAW10, 0, MIPI_CSI2_DT_RAW10),
	PIXMAP_SINK_SRC_PRCR_SWAP(SRGGB10_1X10, SRGGB10_1X10, RAW10, 0, MIPI_CSI2_DT_RAW10),
	PIXMAP_SINK_SRC_PRCR_SWAP(SBGGR12_1X12, SBGGR12_1X12, RAW12, 0, MIPI_CSI2_DT_RAW12),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGBRG12_1X12, SGBRG12_1X12, RAW12, 0, MIPI_CSI2_DT_RAW12),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGRBG12_1X12, SGRBG12_1X12, RAW12, 0, MIPI_CSI2_DT_RAW12),
	PIXMAP_SINK_SRC_PRCR_SWAP(SRGGB12_1X12, SRGGB12_1X12, RAW12, 0, MIPI_CSI2_DT_RAW12),
	PIXMAP_SINK_SRC_PRCR_SWAP(SBGGR14_1X14, SBGGR14_1X14, RAW14, 0, MIPI_CSI2_DT_RAW14),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGBRG14_1X14, SGBRG14_1X14, RAW14, 0, MIPI_CSI2_DT_RAW14),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGRBG14_1X14, SGRBG14_1X14, RAW14, 0, MIPI_CSI2_DT_RAW14),
	PIXMAP_SINK_SRC_PRCR_SWAP(SRGGB14_1X14, SRGGB14_1X14, RAW14, 0, MIPI_CSI2_DT_RAW14),
	/* JPEG */
	PIXMAP_SINK_SRC_PRCR_SWAP(JPEG_1X8, JPEG_1X8, BYTE_STREAM, 0, 0),
};

/*
 * Search through the pix_map table, skipping two consecutive entry with the
 * same code
 */
static inline const struct dcmipp_inp_pix_map *dcmipp_inp_pix_map_by_index
						(unsigned int index,
						 unsigned int pad)
{
	unsigned int i = 0;
	u32 prev_code = 0, cur_code;

	while (i < ARRAY_SIZE(dcmipp_inp_pix_map_list)) {
		if (IS_SRC(pad))
			cur_code = dcmipp_inp_pix_map_list[i].code_src;
		else
			cur_code = dcmipp_inp_pix_map_list[i].code_sink;

		if (cur_code == prev_code) {
			i++;
			continue;
		}
		prev_code = cur_code;

		if (index == 0)
			break;
		i++;
		index--;
	}

	if (i >= ARRAY_SIZE(dcmipp_inp_pix_map_list))
		return NULL;

	return &dcmipp_inp_pix_map_list[i];
}

static inline const struct dcmipp_inp_pix_map *dcmipp_inp_pix_map_by_code
					(u32 code_sink, u32 code_src)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dcmipp_inp_pix_map_list); i++) {
		if ((dcmipp_inp_pix_map_list[i].code_sink == code_sink &&
		     dcmipp_inp_pix_map_list[i].code_src == code_src) ||
		    (dcmipp_inp_pix_map_list[i].code_sink == code_src &&
		     dcmipp_inp_pix_map_list[i].code_src == code_sink) ||
		    (dcmipp_inp_pix_map_list[i].code_sink == code_sink &&
		     code_src == 0) ||
		    (code_sink == 0 &&
		     dcmipp_inp_pix_map_list[i].code_src == code_src))
			return &dcmipp_inp_pix_map_list[i];
	}
	return NULL;
}

struct dcmipp_inp_device {
	struct dcmipp_ent_device ved;
	struct v4l2_subdev sd;
	struct device *dev;
	void __iomem *regs;

	/* Protect concurrent access to s_stream */
	struct mutex lock;
	u32 usecnt;
};

static const struct v4l2_mbus_framefmt fmt_default = {
	.width = DCMIPP_FMT_WIDTH_DEFAULT,
	.height = DCMIPP_FMT_HEIGHT_DEFAULT,
	.code = MEDIA_BUS_FMT_RGB565_2X8_LE,
	.field = V4L2_FIELD_NONE,
	.colorspace = DCMIPP_COLORSPACE_DEFAULT,
	.ycbcr_enc = DCMIPP_YCBCR_ENC_DEFAULT,
	.quantization = DCMIPP_QUANTIZATION_DEFAULT,
	.xfer_func = DCMIPP_XFER_FUNC_DEFAULT,
};

static int dcmipp_inp_init_cfg(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *sd_state)
{
	unsigned int i;

	for (i = 0; i < sd->entity.num_pads; i++) {
		struct v4l2_mbus_framefmt *mf;

		mf = v4l2_subdev_state_get_format(sd_state, i);
		*mf = fmt_default;
	}

	return 0;
}

static int dcmipp_inp_enum_mbus_code(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_mbus_code_enum *code)
{
	const struct dcmipp_inp_pix_map *vpix =
		dcmipp_inp_pix_map_by_index(code->index, code->pad);

	if (!vpix)
		return -EINVAL;

	code->code = IS_SRC(code->pad) ? vpix->code_src : vpix->code_sink;

	return 0;
}

static int dcmipp_inp_enum_frame_size(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *sd_state,
				      struct v4l2_subdev_frame_size_enum *fse)
{
	const struct dcmipp_inp_pix_map *vpix;

	if (fse->index)
		return -EINVAL;

	/* Only accept code in the pix map table */
	vpix = dcmipp_inp_pix_map_by_code(IS_SINK(fse->pad) ? fse->code : 0,
					  IS_SRC(fse->pad) ? fse->code : 0);
	if (!vpix)
		return -EINVAL;

	fse->min_width = DCMIPP_FRAME_MIN_WIDTH;
	fse->max_width = DCMIPP_FRAME_MAX_WIDTH;
	fse->min_height = DCMIPP_FRAME_MIN_HEIGHT;
	fse->max_height = DCMIPP_FRAME_MAX_HEIGHT;

	return 0;
}

static void dcmipp_inp_adjust_fmt(struct dcmipp_inp_device *inp,
				  struct v4l2_mbus_framefmt *fmt, __u32 pad)
{
	const struct dcmipp_inp_pix_map *vpix;

	/* Only accept code in the pix map table */
	vpix = dcmipp_inp_pix_map_by_code(IS_SINK(pad) ? fmt->code : 0,
					  IS_SRC(pad) ? fmt->code : 0);
	if (!vpix)
		fmt->code = fmt_default.code;

	/* Exclude JPEG if BT656 bus is selected */
	if (vpix && vpix->code_sink == MEDIA_BUS_FMT_JPEG_1X8 &&
	    inp->ved.bus_type == V4L2_MBUS_BT656)
		fmt->code = fmt_default.code;

	fmt->width = clamp_t(u32, fmt->width, DCMIPP_FRAME_MIN_WIDTH,
			     DCMIPP_FRAME_MAX_WIDTH);
	fmt->height = clamp_t(u32, fmt->height, DCMIPP_FRAME_MIN_HEIGHT,
			      DCMIPP_FRAME_MAX_HEIGHT);

	if (fmt->field == V4L2_FIELD_ANY || fmt->field == V4L2_FIELD_ALTERNATE)
		fmt->field = fmt_default.field;

	dcmipp_colorimetry_clamp(fmt);
}

static int dcmipp_inp_set_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct dcmipp_inp_device *inp = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *mf;
	int i;

	mutex_lock(&inp->lock);

	if (inp->usecnt) {
		mutex_unlock(&inp->lock);
		return -EBUSY;
	}

	mf = v4l2_subdev_state_get_format(sd_state, fmt->pad);

	/* Set the new format */
	dcmipp_inp_adjust_fmt(inp, &fmt->format, fmt->pad);

	dev_dbg(inp->dev, "%s: format update: old:%dx%d (0x%x, %d, %d, %d, %d) new:%dx%d (0x%x, %d, %d, %d, %d)\n",
		inp->sd.name,
		/* old */
		mf->width, mf->height, mf->code,
		mf->colorspace,	mf->quantization,
		mf->xfer_func, mf->ycbcr_enc,
		/* new */
		fmt->format.width, fmt->format.height, fmt->format.code,
		fmt->format.colorspace, fmt->format.quantization,
		fmt->format.xfer_func, fmt->format.ycbcr_enc);

	*mf = fmt->format;

	/* When setting the sink format, report that format on the src pad */
	if (IS_SINK(fmt->pad)) {
		for (i = 1; i < sd->entity.num_pads; i++) {
			mf = v4l2_subdev_state_get_format(sd_state, i);
			*mf = fmt->format;
			dcmipp_inp_adjust_fmt(inp, mf, 1);
		}
	}

	mutex_unlock(&inp->lock);

	return 0;
}

static const struct v4l2_subdev_pad_ops dcmipp_inp_pad_ops = {
	.init_cfg		= dcmipp_inp_init_cfg,
	.enum_mbus_code		= dcmipp_inp_enum_mbus_code,
	.enum_frame_size	= dcmipp_inp_enum_frame_size,
	.get_fmt		= v4l2_subdev_get_fmt,
	.set_fmt		= dcmipp_inp_set_fmt,
};

static int dcmipp_inp_configure_parallel(struct dcmipp_inp_device *inp,
					 int enable)
{
	u32 val = 0;
	const struct dcmipp_inp_pix_map *vpix;
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_mbus_framefmt *src_fmt;

	if (!enable) {
		/* Disable parallel interface */
		reg_clear(inp, DCMIPP_PRCR, DCMIPP_PRCR_ENABLE);

		return 0;
	}

	/* Set vertical synchronization polarity */
	if (inp->ved.bus.flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH)
		val |= DCMIPP_PRCR_VSPOL;

	/* Set horizontal synchronization polarity */
	if (inp->ved.bus.flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH)
		val |= DCMIPP_PRCR_HSPOL;

	/* Set pixel clock polarity */
	if (inp->ved.bus.flags & V4L2_MBUS_PCLK_SAMPLE_RISING)
		val |= DCMIPP_PRCR_PCKPOL;

	/*
	 * BT656 embedded synchronisation bus mode.
	 *
	 * Default SAV/EAV mode is supported here with default codes
	 * SAV=0xff000080 & EAV=0xff00009d.
	 * With DCMIPP this means LSC=SAV=0x80 & LEC=EAV=0x9d.
	 */
	if (inp->ved.bus_type == V4L2_MBUS_BT656) {
		val |= DCMIPP_PRCR_ESS;

		/* Unmask all codes */
		reg_write(inp, DCMIPP_PRESUR, 0xffffffff);/* FEC:LEC:LSC:FSC */

		/* Trig on LSC=0x80 & LEC=0x9d codes, ignore FSC and FEC */
		reg_write(inp, DCMIPP_PRESCR, 0xff9d80ff);/* FEC:LEC:LSC:FSC */
	}

	/* Set format */
	state = v4l2_subdev_lock_and_get_active_state(&inp->sd);
	sink_fmt = v4l2_subdev_state_get_format(state, 0);
	src_fmt = v4l2_subdev_state_get_format(state, 1);
	v4l2_subdev_unlock_state(state);

	vpix = dcmipp_inp_pix_map_by_code(sink_fmt->code, src_fmt->code);
	if (!vpix) {
		dev_err(inp->dev, "Invalid sink/src format configuration\n");
		return -EINVAL;
	}

	val |= vpix->prcr_format << DCMIPP_PRCR_FORMAT_SHIFT;

	/* swap cycles */
	/*
	 * PPCR SWAPYUV is not available on STM32MP13 so behavior regarding
	 * to SWAPCYCLES should be reversed compared to other platforms for
	 * YUV422 format
	 */
	if (of_device_is_compatible(inp->dev->of_node, "st,stm32mp13-dcmipp") &&
	    (src_fmt->code == MEDIA_BUS_FMT_YUYV8_2X8 ||
	     src_fmt->code == MEDIA_BUS_FMT_YVYU8_2X8 ||
	     src_fmt->code == MEDIA_BUS_FMT_UYVY8_2X8 ||
	     src_fmt->code == MEDIA_BUS_FMT_VYUY8_2X8)) {
		if (!vpix->prcr_swapcycles)
			val |= DCMIPP_PRCR_SWAPCYCLES;
	} else {
		if (vpix->prcr_swapcycles)
			val |= DCMIPP_PRCR_SWAPCYCLES;
	}

	reg_write(inp, DCMIPP_PRCR, val);

	/* Select the DCMIPP parallel interface */
	reg_write(inp, DCMIPP_CMCR, 0);

	/* Enable parallel interface */
	reg_set(inp, DCMIPP_PRCR, DCMIPP_PRCR_ENABLE);

	return 0;
}

static int dcmipp_inp_configure_csi_dt(struct dcmipp_inp_device *inp,
				       u32 pipe_id)
{
	const struct dcmipp_inp_pix_map *vpix;
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_mbus_framefmt *src_fmt;

	/* Get format information */
	state = v4l2_subdev_lock_and_get_active_state(&inp->sd);
	sink_fmt = v4l2_subdev_state_get_format(state, 0);
	src_fmt = v4l2_subdev_state_get_format(state, 1 + pipe_id);
	v4l2_subdev_unlock_state(state);

	/* Only configure Pipe #2 input if is enabled */
	if (pipe_id == 2 && !media_pad_remote_pad_first(&inp->ved.pads[3])) {
		dev_dbg(inp->dev, "Skip disabled pipe %d\n", pipe_id);
		return 0;
	}

	vpix = dcmipp_inp_pix_map_by_code(sink_fmt->code, src_fmt->code);
	if (!vpix) {
		dev_err(inp->dev, "Invalid sink/src format configuration\n");
		return -EINVAL;
	}

	/* We cannot handle JPEG data on main - aux pipes */
	if (pipe_id && !vpix->dt) {
		dev_dbg(inp->dev, "Skip null DT config on pipe %d\n", pipe_id);
		return 0;
	}

	reg_clear(inp, DCMIPP_PxFSCR(pipe_id),
		  DCMIPP_PxFSCR_DTMODE_MASK | DCMIPP_PxFSCR_DTIDA_MASK);

	/* In case of JPEG we don't know the DT so we allow all data */
	/*
	 * TODO - check instead dt == 0 for the time being to allow other
	 * unknown data-type
	 */
	if (!vpix->dt) {
		reg_set(inp, DCMIPP_P0FSCR,
			DCMIPP_P0FSCR_DTMODE_ALLDT << DCMIPP_PxFSCR_DTMODE_SHIFT);
	} else {
		reg_set(inp, DCMIPP_PxFSCR(pipe_id),
			vpix->dt << DCMIPP_PxFSCR_DTIDA_SHIFT |
			DCMIPP_PxFSCR_DTMODE_DTIDA);
	}

	return 0;
}

static int dcmipp_inp_configure_csi(struct dcmipp_inp_device *inp)
{
	int i, ret;

	for (i = 0; i < inp->ved.dcmipp->pipe_cfg->pipe_nb; i++) {
		ret = dcmipp_inp_configure_csi_dt(inp, i);
		if (ret)
			return ret;
	}

	/* Select the DCMIPP CSI interface */
	reg_write(inp, DCMIPP_CMCR, DCMIPP_CMCR_INSEL);

	return 0;
}

static int dcmipp_inp_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dcmipp_inp_device *inp =
				container_of(sd, struct dcmipp_inp_device, sd);
	struct v4l2_subdev *s_subdev;
	struct media_pad *pad;
	int ret = 0;

	/* Get source subdev */
	pad = media_pad_remote_pad_first(&sd->entity.pads[0]);
	if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
		return -EINVAL;
	s_subdev = media_entity_to_v4l2_subdev(pad->entity);

	mutex_lock(&inp->lock);

	if (enable) {
		/* Nothing to do if already enabled by someone */
		if (inp->usecnt)
			goto out;

		if (inp->ved.bus_type == V4L2_MBUS_PARALLEL ||
		    inp->ved.bus_type == V4L2_MBUS_BT656)
			ret = dcmipp_inp_configure_parallel(inp, enable);
		else if (inp->ved.bus_type == V4L2_MBUS_CSI2_DPHY)
			ret = dcmipp_inp_configure_csi(inp);
		if (ret)
			goto error_s_stream;

		/*
		 * Check if the Aux pipe source pad is connected / enabled
		 * or not.  If enabled, it means that Aux pipe works alone
		 * and not connected to Main pipe ISP
		 */
		if (inp->ved.ent->num_pads >= 3 &&
		    !media_pad_remote_pad_first(&inp->ved.pads[3]))
			reg_clear(inp, DCMIPP_P1FSCR, DCMIPP_P1FSCR_PIPEDIFF);
		else
			reg_set(inp, DCMIPP_P1FSCR, DCMIPP_P1FSCR_PIPEDIFF);

		ret = v4l2_subdev_call(s_subdev, video, s_stream, enable);
		if (ret < 0) {
			dev_err(inp->dev,
				"failed to start source subdev streaming (%d)\n",
				ret);
			goto error_s_stream;
		}
	} else {
		if (inp->usecnt > 1)
			goto out;

		ret = v4l2_subdev_call(s_subdev, video, s_stream, enable);
		if (ret < 0) {
			dev_err(inp->dev,
				"failed to stop source subdev streaming (%d)\n",
				ret);
			goto error_s_stream;
		}

		if (inp->ved.bus_type == V4L2_MBUS_PARALLEL ||
		    inp->ved.bus_type == V4L2_MBUS_BT656) {
			ret = dcmipp_inp_configure_parallel(inp, enable);
			if (ret)
				goto error_s_stream;
		}
	}

out:
	inp->usecnt += enable ? 1 : -1;

error_s_stream:
	mutex_unlock(&inp->lock);

	return ret;
}

static const struct v4l2_subdev_video_ops dcmipp_inp_video_ops = {
	.s_stream = dcmipp_inp_s_stream,
};

static const struct v4l2_subdev_ops dcmipp_inp_ops = {
	.pad = &dcmipp_inp_pad_ops,
	.video = &dcmipp_inp_video_ops,
};

static void dcmipp_inp_release(struct v4l2_subdev *sd)
{
	struct dcmipp_inp_device *inp =
				container_of(sd, struct dcmipp_inp_device, sd);

	kfree(inp);
}

static const struct v4l2_subdev_internal_ops dcmipp_inp_int_ops = {
	.release = dcmipp_inp_release,
};

void dcmipp_inp_ent_release(struct dcmipp_ent_device *ved)
{
	struct dcmipp_inp_device *inp =
			container_of(ved, struct dcmipp_inp_device, ved);

	dcmipp_ent_sd_unregister(ved, &inp->sd);
	mutex_destroy(&inp->lock);
}

struct dcmipp_ent_device *dcmipp_inp_ent_init(const char *entity_name,
					      struct dcmipp_device *dcmipp)
{
	struct dcmipp_inp_device *inp;
	const unsigned long pads_flag_stm32mp25[] = {
		MEDIA_PAD_FL_SINK, MEDIA_PAD_FL_SOURCE,
		MEDIA_PAD_FL_SOURCE, MEDIA_PAD_FL_SOURCE,
	};
	struct device *dev = dcmipp->dev;
	u16 pads_nb = dcmipp->pipe_cfg->pipe_nb + 1;
	int ret;


	/* Allocate the inp struct */
	inp = kzalloc(sizeof(*inp), GFP_KERNEL);
	if (!inp)
		return ERR_PTR(-ENOMEM);

	inp->regs = dcmipp->regs;

	/* Initialize the lock */
	mutex_init(&inp->lock);

	/* Initialize ved and sd */
	ret = dcmipp_ent_sd_register(&inp->ved, &inp->sd, &dcmipp->v4l2_dev,
				     entity_name, MEDIA_ENT_F_VID_IF_BRIDGE,
				     pads_nb, pads_flag_stm32mp25,
				     &dcmipp_inp_int_ops, &dcmipp_inp_ops,
				     NULL, NULL);
	if (ret) {
		mutex_destroy(&inp->lock);
		kfree(inp);
		return ERR_PTR(ret);
	}
	inp->ved.dcmipp = dcmipp;

	inp->dev = dev;

	return &inp->ved;
}
