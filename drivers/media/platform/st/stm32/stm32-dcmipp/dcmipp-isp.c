// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for STM32 Digital Camera Memory Interface Pixel Processor
 *
 * Copyright (C) STMicroelectronics SA 2021
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          Alain Volmat <alain.volmat@foss.st.com>
 *          for STMicroelectronics.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/vmalloc.h>
#include <linux/v4l2-mediabus.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-event.h>
#include <media/v4l2-rect.h>
#include <media/v4l2-subdev.h>

#include "dcmipp-common.h"

#define DCMIPP_ISP_DRV_NAME "dcmipp-isp"

#define DCMIPP_CMSR2_P1VSYNCF BIT(18)

#define DCMIPP_P1SRCR	0x820
#define DCMIPP_P1SRCR_LASTLINE_SHIFT	0
#define DCMIPP_P1SRCR_LASTLINE_MASK	GENMASK(11, 0)
#define DCMIPP_P1SRCR_FIRSTLINEDEL_SHIFT	12
#define DCMIPP_P1SRCR_FIRSTLINEDEL_MASK	GENMASK(14, 12)
#define DCMIPP_P1SRCR_CROPEN		BIT(15)

#define DCMIPP_P1DECR	0x830
#define DCMIPP_P1DECR_ENABLE		BIT(0)
#define DCMIPP_P1DECR_HDEC_SHIFT	1
#define DCMIPP_P1DECR_HDEC_MASK		GENMASK(2, 1)
#define DCMIPP_P1DECR_VDEC_SHIFT	3
#define DCMIPP_P1DECR_VDEC_MASK		GENMASK(4, 3)

#define DCMIPP_P1DMCR	0x870
#define DCMIPP_P1DMCR_ENABLE		BIT(0)
#define DCMIPP_P1DMCR_TYPE_SHIFT	1
#define DCMIPP_P1DMCR_TYPE_MASK		GENMASK(2, 1)
#define DCMIPP_P1DMCR_TYPE_RGGB		0x0
#define DCMIPP_P1DMCR_TYPE_GRBG		0x1
#define DCMIPP_P1DMCR_TYPE_GBRG		0x2
#define DCMIPP_P1DMCR_TYPE_BGGR		0x3

#define IS_SINK(pad) (!(pad))
#define IS_SRC(pad)  ((pad))
#define PAD_STR(pad) (IS_SRC((pad))) ? "src" : "sink"

#define ISP_MEDIA_BUS_SINK_FMT_DEFAULT MEDIA_BUS_FMT_RGB565_2X8_LE
#define ISP_MEDIA_BUS_SRC_FMT_DEFAULT MEDIA_BUS_FMT_RGB888_1X24

static const unsigned int dcmipp_isp_sink_pix_map_list[] = {
	/* RGB565 */
	MEDIA_BUS_FMT_RGB565_2X8_LE,
	MEDIA_BUS_FMT_RGB565_1X16,
	/* RGB888 */
	MEDIA_BUS_FMT_RGB888_3X8,
	MEDIA_BUS_FMT_RGB888_1X24,
	/* YUV422 */
	MEDIA_BUS_FMT_YUYV8_2X8,
	MEDIA_BUS_FMT_UYVY8_1X16,
	/* GREY */
	MEDIA_BUS_FMT_Y8_1X8,
	MEDIA_BUS_FMT_Y10_1X10,
	MEDIA_BUS_FMT_Y12_1X12,
	MEDIA_BUS_FMT_Y14_1X14,
	/* Raw Bayer */
	/* Raw 8 */
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	/* Raw 10 */
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	/* Raw 12 */
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	/* Raw 14 */
	MEDIA_BUS_FMT_SBGGR14_1X14,
	MEDIA_BUS_FMT_SGBRG14_1X14,
	MEDIA_BUS_FMT_SGRBG14_1X14,
	MEDIA_BUS_FMT_SRGGB14_1X14,
	/* Raw 16 */
	MEDIA_BUS_FMT_SBGGR16_1X16,
	MEDIA_BUS_FMT_SGBRG16_1X16,
	MEDIA_BUS_FMT_SGRBG16_1X16,
	MEDIA_BUS_FMT_SRGGB16_1X16,
};

static const unsigned int dcmipp_isp_src_pix_map_list[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_YUV8_1X24,
};

static unsigned int
dcmipp_isp_pix_map_by_code(u32 code, unsigned int pad)
{
	const unsigned int *l;
	unsigned int size;
	unsigned int i;

	if (IS_SRC(pad)) {
		l = dcmipp_isp_src_pix_map_list;
		size = ARRAY_SIZE(dcmipp_isp_src_pix_map_list);
	} else {
		l = dcmipp_isp_sink_pix_map_list;
		size = ARRAY_SIZE(dcmipp_isp_sink_pix_map_list);
	}

	for (i = 0; i < size; i++) {
		if (l[i] == code)
			return code;
	}

	return 0;
}

struct dcmipp_isp_device {
	struct dcmipp_ent_device ved;
	struct v4l2_subdev sd;
	struct device *dev;

	/* Protect concurrent access to s_stream */
	struct mutex lock;
	u32 usecnt;

	void __iomem *regs;
};

static const struct v4l2_mbus_framefmt fmt_default = {
	.width = DCMIPP_FMT_WIDTH_DEFAULT,
	.height = DCMIPP_FMT_HEIGHT_DEFAULT,
	.code = ISP_MEDIA_BUS_SINK_FMT_DEFAULT,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_REC709,
	.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
	.quantization = V4L2_QUANTIZATION_DEFAULT,
	.xfer_func = V4L2_XFER_FUNC_DEFAULT,
};

static inline unsigned int dcmipp_isp_adjust_decimation(unsigned int size,
							unsigned int req)
{
	unsigned int i = 0;

	/* Maximum decimation factor is 8 */
	while (size > req && ++i < 4)
		size /= 2;

	return size;
}

static void dcmipp_isp_adjust_fmt(struct v4l2_mbus_framefmt *fmt, u32 pad)
{
	/* Only accept code in the pix map table */
	if (!dcmipp_isp_pix_map_by_code(fmt->code, pad))
		fmt->code = IS_SRC(pad) ? ISP_MEDIA_BUS_SRC_FMT_DEFAULT :
					  ISP_MEDIA_BUS_SINK_FMT_DEFAULT;

	fmt->width = clamp_t(u32, fmt->width, DCMIPP_FRAME_MIN_WIDTH,
			     DCMIPP_PIXEL_FRAME_MAX_WIDTH) & ~1;
	fmt->height = clamp_t(u32, fmt->height, DCMIPP_FRAME_MIN_HEIGHT,
			      DCMIPP_PIXEL_FRAME_MAX_HEIGHT);

	if (fmt->field == V4L2_FIELD_ANY || fmt->field == V4L2_FIELD_ALTERNATE)
		fmt->field = V4L2_FIELD_NONE;

	dcmipp_colorimetry_clamp(fmt);
}

static int dcmipp_isp_init_cfg(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *mf;
	unsigned int i;

	/* Initialize the sink & source pads of data */
	for (i = 0; i < 2; i++) {

		mf = v4l2_subdev_state_get_format(state, i);
		*mf = fmt_default;
		mf->code = IS_SRC(i) ? ISP_MEDIA_BUS_SRC_FMT_DEFAULT :
				       ISP_MEDIA_BUS_SINK_FMT_DEFAULT;
	}

	/* Initialize isp params & isp stats pads */
	mf = v4l2_subdev_state_get_format(state, 2);
	memset(mf, 0, sizeof(struct v4l2_mbus_framefmt));
	mf->code = MEDIA_BUS_FMT_METADATA_FIXED;

	mf = v4l2_subdev_state_get_format(state, 3);
	memset(mf, 0, sizeof(struct v4l2_mbus_framefmt));
	mf->code = MEDIA_BUS_FMT_METADATA_FIXED;

	return 0;
}

static int dcmipp_isp_enum_mbus_code(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_mbus_code_enum *code)
{
	const unsigned int *l;
	unsigned int size;

	if (IS_SRC(code->pad)) {
		l = dcmipp_isp_src_pix_map_list;
		size = ARRAY_SIZE(dcmipp_isp_src_pix_map_list);
	} else {
		l = dcmipp_isp_sink_pix_map_list;
		size = ARRAY_SIZE(dcmipp_isp_sink_pix_map_list);
	}

	if (code->index >= size)
		return -EINVAL;

	code->code = l[code->index];

	return 0;
}

static int dcmipp_isp_enum_frame_size(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index)
		return -EINVAL;

	/* Only accept code in the pix map table */
	if (!dcmipp_isp_pix_map_by_code(fse->code, fse->pad))
		return -EINVAL;

	fse->min_width = DCMIPP_FRAME_MIN_WIDTH;
	fse->max_width = DCMIPP_PIXEL_FRAME_MAX_WIDTH;
	fse->min_height = DCMIPP_FRAME_MIN_HEIGHT;
	fse->max_height = DCMIPP_PIXEL_FRAME_MAX_HEIGHT;

	return 0;
}

static int dcmipp_isp_set_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *fmt)
{
	struct dcmipp_isp_device *isp = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *pad_fmt, *opp_pad_fmt;

	mutex_lock(&isp->lock);

	if (isp->usecnt) {
		mutex_unlock(&isp->lock);
		return -EBUSY;
	}

	pad_fmt = v4l2_subdev_state_get_format(state, fmt->pad);

	dcmipp_isp_adjust_fmt(&fmt->format, fmt->pad);

	if (IS_SINK(fmt->pad)) {
		struct v4l2_rect *crop, *compose;

		opp_pad_fmt = v4l2_subdev_state_get_format(state, 1);
		crop = v4l2_subdev_state_get_crop(state, 0);
		compose = v4l2_subdev_state_get_compose(state, 0);

		*opp_pad_fmt = fmt->format;
		if (fmt->format.code >= MEDIA_BUS_FMT_Y8_1X8 &&
		    fmt->format.code < MEDIA_BUS_FMT_SBGGR8_1X8)
			opp_pad_fmt->code = MEDIA_BUS_FMT_YUV8_1X24;
		else
			opp_pad_fmt->code = MEDIA_BUS_FMT_RGB888_1X24;

		crop->top = 0;
		crop->left = 0;
		crop->width = fmt->format.width;
		crop->height = fmt->format.height;
		*compose = *crop;

		dev_dbg(isp->dev, "%s: source format update: new:%dx%d (0x%x, %d, %d, %d, %d)\n",
			isp->sd.name,
			opp_pad_fmt->width, opp_pad_fmt->height,
			opp_pad_fmt->code, opp_pad_fmt->colorspace,
			opp_pad_fmt->quantization,
			opp_pad_fmt->xfer_func, opp_pad_fmt->ycbcr_enc);
	} else {
		struct v4l2_rect *compose;

		opp_pad_fmt = v4l2_subdev_state_get_format(state, 0);
		compose = v4l2_subdev_state_get_compose(state, 0);

		fmt->format = *opp_pad_fmt;
		if (opp_pad_fmt->code >= MEDIA_BUS_FMT_Y8_1X8 &&
		    opp_pad_fmt->code < MEDIA_BUS_FMT_SBGGR8_1X8)
			fmt->format.code = MEDIA_BUS_FMT_YUV8_1X24;
		else
			fmt->format.code = MEDIA_BUS_FMT_RGB888_1X24;
		if (compose->width && compose->height) {
			fmt->format.width = compose->width;
			fmt->format.height = compose->height;
		}
	}

	dev_dbg(isp->dev, "%s: %s format update: old:%dx%d (0x%x, %d, %d, %d, %d) new:%dx%d (0x%x, %d, %d, %d, %d)\n",
		isp->sd.name,
		PAD_STR(fmt->pad),
		/* old */
		pad_fmt->width, pad_fmt->height, pad_fmt->code,
		pad_fmt->colorspace, pad_fmt->quantization,
		pad_fmt->xfer_func, pad_fmt->ycbcr_enc,
		/* new */
		fmt->format.width, fmt->format.height, fmt->format.code,
		fmt->format.colorspace, fmt->format.quantization,
		fmt->format.xfer_func, fmt->format.ycbcr_enc);

	*pad_fmt = fmt->format;

	mutex_unlock(&isp->lock);

	return 0;
}

static void dcmipp_isp_adjust_crop(struct v4l2_rect *r,
				   const struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_rect src_rect = {
		.top = 0,
		.left = 0,
		.width = fmt->width,
		.height = fmt->height,
	};
	struct v4l2_rect crop_min = {
		.top = 8,
		.left = 0,
		.width = fmt->width,
		.height = 1,
	};

	/* Disallow rectangles smaller than the minimal one. */
	v4l2_rect_set_min_size(r, &crop_min);
	v4l2_rect_map_inside(r, &src_rect);
}

static int dcmipp_isp_get_selection(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_selection *s)
{
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_rect *crop, *compose;

	if (IS_SRC(s->pad))
		return -EINVAL;

	sink_fmt = v4l2_subdev_state_get_format(state, s->pad);
	crop = v4l2_subdev_state_get_crop(state, s->pad);
	compose = v4l2_subdev_state_get_compose(state, s->pad);

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		s->r = *crop;
		break;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r.top = 0;
		s->r.left = 0;
		s->r.width = sink_fmt->width;
		s->r.height = sink_fmt->height;
		break;
	case V4L2_SEL_TGT_COMPOSE:
		s->r = *compose;
		break;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		s->r = *crop;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int dcmipp_isp_set_selection(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_selection *s)
{
	struct dcmipp_isp_device *isp = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink_fmt, *src_fmt;
	struct v4l2_rect *crop, *compose;

	if (IS_SRC(s->pad))
		return -EINVAL;

	sink_fmt = v4l2_subdev_state_get_format(state, s->pad);
	src_fmt = v4l2_subdev_state_get_format(state, 1);
	crop = v4l2_subdev_state_get_crop(state, s->pad);
	compose = v4l2_subdev_state_get_compose(state, s->pad);

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		dcmipp_isp_adjust_crop(&s->r, sink_fmt);

		*crop = s->r;
		*compose = s->r;

		dev_dbg(isp->dev, "s_selection: crop %ux%u@(%u,%u)\n",
			crop->width, crop->height, crop->left, crop->top);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		s->r.top = 0;
		s->r.left = 0;
		if (s->r.width > crop->width)
			s->r.width = crop->width;
		else
			s->r.width = dcmipp_isp_adjust_decimation(crop->width,
								  s->r.width);
		if (s->r.height > crop->height)
			s->r.height = crop->height;
		else
			s->r.height = dcmipp_isp_adjust_decimation(crop->height,
								   s->r.height);
		*compose = s->r;

		dev_dbg(isp->dev, "s_selection: compose %ux%u@(%u,%u)\n",
			s->r.width, s->r.height, s->r.left, s->r.top);
		break;
	default:
		return -EINVAL;
	}

	/* Update the source pad size */
	src_fmt->width = s->r.width;
	src_fmt->height = s->r.height;

	return 0;
}

static const struct v4l2_subdev_pad_ops dcmipp_isp_pad_ops = {
	.init_cfg		= dcmipp_isp_init_cfg,
	.enum_mbus_code		= dcmipp_isp_enum_mbus_code,
	.enum_frame_size	= dcmipp_isp_enum_frame_size,
	.get_fmt		= v4l2_subdev_get_fmt,
	.set_fmt		= dcmipp_isp_set_fmt,
	.get_selection		= dcmipp_isp_get_selection,
	.set_selection		= dcmipp_isp_set_selection,
};

static void dcmipp_isp_config_demosaicing(struct dcmipp_isp_device *isp,
					  struct v4l2_mbus_framefmt *sink_fmt)
{
	unsigned int pix_code = sink_fmt->code;
	unsigned int val = 0;

	/* Disable demosaicing */
	reg_clear(isp, DCMIPP_P1DMCR,
		  DCMIPP_P1DMCR_ENABLE | DCMIPP_P1DMCR_TYPE_MASK);

	if (pix_code < 0x3000 || pix_code >= 0x4000)
		return;

	dev_dbg(isp->dev, "Input is RawBayer, enable Demosaicing\n");

	if (pix_code == MEDIA_BUS_FMT_SBGGR8_1X8 ||
	    pix_code == MEDIA_BUS_FMT_SBGGR10_1X10 ||
	    pix_code == MEDIA_BUS_FMT_SBGGR12_1X12 ||
	    pix_code == MEDIA_BUS_FMT_SBGGR14_1X14 ||
	    pix_code == MEDIA_BUS_FMT_SBGGR16_1X16)
		val = DCMIPP_P1DMCR_TYPE_BGGR << DCMIPP_P1DMCR_TYPE_SHIFT;
	else if (pix_code == MEDIA_BUS_FMT_SGBRG8_1X8 ||
		 pix_code == MEDIA_BUS_FMT_SGBRG10_1X10 ||
		 pix_code == MEDIA_BUS_FMT_SGBRG12_1X12 ||
		 pix_code == MEDIA_BUS_FMT_SGBRG14_1X14 ||
		 pix_code == MEDIA_BUS_FMT_SGBRG16_1X16)
		val = DCMIPP_P1DMCR_TYPE_GBRG << DCMIPP_P1DMCR_TYPE_SHIFT;
	else if (pix_code == MEDIA_BUS_FMT_SGRBG8_1X8 ||
		 pix_code == MEDIA_BUS_FMT_SGRBG10_1X10 ||
		 pix_code == MEDIA_BUS_FMT_SGRBG12_1X12 ||
		 pix_code == MEDIA_BUS_FMT_SGRBG14_1X14 ||
		 pix_code == MEDIA_BUS_FMT_SGRBG16_1X16)
		val = DCMIPP_P1DMCR_TYPE_GRBG << DCMIPP_P1DMCR_TYPE_SHIFT;
	else if (pix_code == MEDIA_BUS_FMT_SRGGB8_1X8 ||
		 pix_code == MEDIA_BUS_FMT_SRGGB10_1X10 ||
		 pix_code == MEDIA_BUS_FMT_SRGGB12_1X12 ||
		 pix_code == MEDIA_BUS_FMT_SRGGB14_1X14 ||
		 pix_code == MEDIA_BUS_FMT_SRGGB16_1X16)
		val = DCMIPP_P1DMCR_TYPE_RGGB << DCMIPP_P1DMCR_TYPE_SHIFT;

	val |= DCMIPP_P1DMCR_ENABLE;

	reg_set(isp, DCMIPP_P1DMCR, val);
}

static void dcmipp_isp_config_decimation(struct dcmipp_isp_device *isp,
					 struct v4l2_rect *crop,
					 struct v4l2_rect *compose)
{
	u32 decr;

	decr = (fls(crop->width / compose->width) - 1) << DCMIPP_P1DECR_HDEC_SHIFT |
	       (fls(crop->height / compose->height) - 1) << DCMIPP_P1DECR_VDEC_SHIFT;
	if (decr)
		decr |= DCMIPP_P1DECR_ENABLE;

	dev_dbg(isp->dev, "%s: config decr: 0x%x\n", __func__, decr);

	reg_write(isp, DCMIPP_P1DECR, decr);
}

/* Histogram block - only available starting from stm32mp21 */
#define DCMIPP_P1HSCR			0x8b0
/* 4 Comp / 64 bins per comp / 1 region / decimated by 2*/
#define DCMIPP_P1HSCR_DEFAULT		0x08411000

#define DCMIPP_P1HSSTR			0x8b4
#define DCMIPP_P1HSSTR_START(x, y)	((x) | ((y) << 16))

#define DCMIPP_P1HSSZR			0x8b8
#define DCMIPP_P1HSSZR_SIZE(w, h)	((w) | ((h) << 16))
static void dcmipp_isp_config_histo(struct dcmipp_isp_device *isp,
				    struct v4l2_rect *compose)
{
	if (!isp->ved.dcmipp->pipe_cfg->has_histo)
		return;

	/*
	 * Configure a default setting for histogram
	 * In order to be able to capture statistics even if histogram settings
	 * aren't yet done, perform initial settings of histogram with always
	 * valid settings
	 */
	reg_write(isp, DCMIPP_P1HSCR, DCMIPP_P1HSCR_DEFAULT);
	reg_write(isp, DCMIPP_P1HSSTR,
		  DCMIPP_P1HSSTR_START(compose->width / 4, compose->height / 4));
	reg_write(isp, DCMIPP_P1HSSZR,
		  DCMIPP_P1HSSZR_SIZE(compose->width / 2, compose->height / 2));
}

static int dcmipp_isp_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dcmipp_isp_device *isp = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_rect *compose, *crop;
	struct v4l2_subdev_state *state;
	struct v4l2_subdev *s_subdev;
	struct media_pad *pad;
	int ret = 0;

	/* Get source subdev */
	pad = media_pad_remote_pad_first(&sd->entity.pads[0]);
	if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
		return -EINVAL;
	s_subdev = media_entity_to_v4l2_subdev(pad->entity);

	mutex_lock(&isp->lock);

	if (enable) {
		/* Nothing to do if already enabled by someone */
		if (isp->usecnt)
			goto out;

		state = v4l2_subdev_lock_and_get_active_state(&isp->sd);
		sink_fmt = v4l2_subdev_state_get_format(state, 0);
		crop = v4l2_subdev_state_get_crop(state, 0);
		compose = v4l2_subdev_state_get_compose(state, 0);
		v4l2_subdev_unlock_state(state);

		/* Configure Statistic Removal */
		reg_write(isp, DCMIPP_P1SRCR,
			  ((crop->top << DCMIPP_P1SRCR_FIRSTLINEDEL_SHIFT) |
			   (crop->height << DCMIPP_P1SRCR_LASTLINE_SHIFT) |
			   DCMIPP_P1SRCR_CROPEN));

		/* Configure Decimation */
		dcmipp_isp_config_decimation(isp, crop, compose);

		/* Configure Demosaicing */
		dcmipp_isp_config_demosaicing(isp, sink_fmt);

		/* Configure default ISP Histo area */
		dcmipp_isp_config_histo(isp, compose);
	} else {
		if (isp->usecnt > 1)
			goto out;

		/* Disable all blocks */
		reg_write(isp, DCMIPP_P1SRCR, 0);
		reg_write(isp, DCMIPP_P1DECR, 0);
		reg_write(isp, DCMIPP_P1DMCR, 0);
	}

	ret = v4l2_subdev_call(s_subdev, video, s_stream, enable);
	if (ret < 0) {
		dev_err(isp->dev,
			"failed to start source subdev streaming (%d)\n", ret);
		goto error_s_stream;
	}

out:
	isp->usecnt += enable ? 1 : -1;
error_s_stream:
	mutex_unlock(&isp->lock);

	return 0;
}

static const struct v4l2_subdev_video_ops dcmipp_isp_video_ops = {
	.s_stream = dcmipp_isp_s_stream,
};

static const struct v4l2_subdev_ops dcmipp_isp_ops = {
	.pad = &dcmipp_isp_pad_ops,
	.video = &dcmipp_isp_video_ops,
};

void dcmipp_isp_ent_release(struct dcmipp_ent_device *ved)
{
	struct dcmipp_isp_device *isp =
			container_of(ved, struct dcmipp_isp_device, ved);

	dcmipp_ent_sd_unregister(ved, &isp->sd);
	mutex_destroy(&isp->lock);
	kfree(isp);
}

static int dcmipp_isp_link_validate(struct media_link *link)
{
	/*
	 * We only need to check link coming to the sink pad #0 since
	 * sink pad #2 is connected to the output video device
	 */
	if (link->sink->index != 0)
		return 0;

	return v4l2_subdev_link_validate(link);
}

static const struct media_entity_operations dcmipp_isp_mops = {
	.link_validate		= dcmipp_isp_link_validate,
};

struct dcmipp_ent_device *dcmipp_isp_ent_init(const char *entity_name,
					      struct dcmipp_device *dcmipp)
{
	struct dcmipp_isp_device *isp;
	const unsigned long pads_flag[] = {
		MEDIA_PAD_FL_SINK, MEDIA_PAD_FL_SOURCE,
		MEDIA_PAD_FL_SINK, MEDIA_PAD_FL_SOURCE,
	};
	int ret;

	/* Allocate the isp struct */
	isp = kzalloc(sizeof(*isp), GFP_KERNEL);
	if (!isp)
		return ERR_PTR(-ENOMEM);

	isp->regs = dcmipp->regs;
	isp->dev = dcmipp->dev;

	/* Initialize the lock */
	mutex_init(&isp->lock);

	/* Initialize ved and sd */
	ret = dcmipp_ent_sd_register(&isp->ved, &isp->sd,
				     &dcmipp->v4l2_dev, entity_name,
				     MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER,
				     ARRAY_SIZE(pads_flag), pads_flag,
				     NULL, &dcmipp_isp_ops,
				     NULL, NULL);
	if (ret) {
		mutex_destroy(&isp->lock);
		kfree(isp);
		return ERR_PTR(ret);
	}
	isp->sd.entity.ops = &dcmipp_isp_mops;
	isp->ved.dcmipp = dcmipp;

	return &isp->ved;
}
