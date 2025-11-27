// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for STM32 Digital Camera Memory Interface Pixel Processor
 *
 * Copyright (C) STMicroelectronics SA 2023
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          Alain Volmat <alain.volmat@foss.st.com>
 *          for STMicroelectronics.
 */

#include <linux/vmalloc.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-rect.h>
#include <media/v4l2-subdev.h>

#include "dcmipp-common.h"

#define DCMIPP_P0FCTCR	0x500
#define DCMIPP_P0FCTCR_FRATE_MASK	GENMASK(1, 0)
#define DCMIPP_P0SCSTR	0x504
#define DCMIPP_P0SCSTR_HSTART_SHIFT	0
#define DCMIPP_P0SCSTR_VSTART_SHIFT	16
#define DCMIPP_P0SCSZR	0x508
#define DCMIPP_P0SCSZR_ENABLE		BIT(31)
#define DCMIPP_P0SCSZR_HSIZE_SHIFT	0
#define DCMIPP_P0SCSZR_VSIZE_SHIFT	16
#define DCMIPP_P0PPCR	0x5c0
#define DCMIPP_P0PPCR_SWAPYUV		BIT(0)
#define DCMIPP_P0PPCR_BSM_1_2		0x1
#define DCMIPP_P0PPCR_BSM_1_4		0x2
#define DCMIPP_P0PPCR_BSM_2_4		0x3
#define DCMIPP_P0PPCR_BSM_MASK		GENMASK(8, 7)
#define DCMIPP_P0PPCR_BSM_SHIFT		0x7
#define DCMIPP_P0PPCR_LSM		BIT(10)
#define DCMIPP_P0PPCR_OELS		BIT(11)

#define IS_SINK(pad) (!(pad))
#define IS_SRC(pad)  ((pad))

#define DCMIPP_P0_CROP_MAX	4094

struct dcmipp_byteproc_pix_map {
	unsigned int code;
	unsigned int bpp;
};

#define PIXMAP_MBUS_BPP(mbus, byteperpixel)		\
	{						\
		.code = MEDIA_BUS_FMT_##mbus,		\
		.bpp = byteperpixel,			\
	}
static const struct dcmipp_byteproc_pix_map dcmipp_byteproc_pix_map_list[] = {
	PIXMAP_MBUS_BPP(RGB565_2X8_LE, 2),
	PIXMAP_MBUS_BPP(RGB565_1X16, 2),
	PIXMAP_MBUS_BPP(RGB888_3X8, 3),
	PIXMAP_MBUS_BPP(RGB888_1X24, 3),
	PIXMAP_MBUS_BPP(YUYV8_2X8, 2),
	PIXMAP_MBUS_BPP(YUYV8_1X16, 2),
	PIXMAP_MBUS_BPP(YVYU8_2X8, 2),
	PIXMAP_MBUS_BPP(YVYU8_1X16, 2),
	PIXMAP_MBUS_BPP(UYVY8_2X8, 2),
	PIXMAP_MBUS_BPP(UYVY8_1X16, 2),
	PIXMAP_MBUS_BPP(VYUY8_2X8, 2),
	PIXMAP_MBUS_BPP(VYUY8_1X16, 2),
	PIXMAP_MBUS_BPP(Y8_1X8, 1),
	PIXMAP_MBUS_BPP(Y10_1X10, 2),
	PIXMAP_MBUS_BPP(Y12_1X12, 2),
	PIXMAP_MBUS_BPP(Y14_1X14, 2),
	PIXMAP_MBUS_BPP(SBGGR8_1X8, 1),
	PIXMAP_MBUS_BPP(SGBRG8_1X8, 1),
	PIXMAP_MBUS_BPP(SGRBG8_1X8, 1),
	PIXMAP_MBUS_BPP(SRGGB8_1X8, 1),
	PIXMAP_MBUS_BPP(SBGGR10_1X10, 2),
	PIXMAP_MBUS_BPP(SGBRG10_1X10, 2),
	PIXMAP_MBUS_BPP(SGRBG10_1X10, 2),
	PIXMAP_MBUS_BPP(SRGGB10_1X10, 2),
	PIXMAP_MBUS_BPP(SBGGR12_1X12, 2),
	PIXMAP_MBUS_BPP(SGBRG12_1X12, 2),
	PIXMAP_MBUS_BPP(SGRBG12_1X12, 2),
	PIXMAP_MBUS_BPP(SRGGB12_1X12, 2),
	PIXMAP_MBUS_BPP(SBGGR14_1X14, 2),
	PIXMAP_MBUS_BPP(SGBRG14_1X14, 2),
	PIXMAP_MBUS_BPP(SGRBG14_1X14, 2),
	PIXMAP_MBUS_BPP(SRGGB14_1X14, 2),
	PIXMAP_MBUS_BPP(JPEG_1X8, 1),
};

static const struct dcmipp_byteproc_pix_map *
dcmipp_byteproc_pix_map_by_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dcmipp_byteproc_pix_map_list); i++) {
		if (dcmipp_byteproc_pix_map_list[i].code == code)
			return &dcmipp_byteproc_pix_map_list[i];
	}

	return NULL;
}

struct dcmipp_byteproc_device {
	struct dcmipp_ent_device ved;
	struct v4l2_subdev sd;
	struct device *dev;
	void __iomem *regs;
	bool streaming;

	struct v4l2_fract sink_interval;
	struct v4l2_fract src_interval;
	unsigned int frate;
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

static const struct v4l2_rect crop_min = {
	.width = DCMIPP_FRAME_MIN_WIDTH,
	.height = DCMIPP_FRAME_MIN_HEIGHT,
	.top = 0,
	.left = 0,
};

static void dcmipp_byteproc_adjust_crop(struct dcmipp_byteproc_device *byteproc,
					struct v4l2_rect *r,
					struct v4l2_rect *compose,
					unsigned int bytesperpixel)
{
	u32 alignment = 1;

	/* The pipe outputs in words hence the line byte length needs to be multiple of 4 */
	if (bytesperpixel == 1 || bytesperpixel == 3)
		alignment = 4;
	else if (bytesperpixel == 2)
		alignment = 2;

	/* Disallow rectangles smaller than the minimal one. */
	v4l2_rect_set_min_size(r, &crop_min);
	v4l2_rect_map_inside(r, compose);

	/* Ensure that frame output match HW requirements */
	r->left = round_down(r->left, alignment);
	r->width = round_down(r->width, alignment);

	/* Check if crop can be programmed within the registers */
	if (r->left * bytesperpixel / 4 > DCMIPP_P0_CROP_MAX ||
	    r->width * bytesperpixel / 4 > DCMIPP_P0_CROP_MAX) {
		r->left = 0;
		r->width = compose->width;
		dev_dbg(byteproc->dev, "Frame horizontal crop disabled\n");
	}
	if (r->top > DCMIPP_P0_CROP_MAX || r->height > DCMIPP_P0_CROP_MAX) {
		r->top = 0;
		r->height = compose->height;
		dev_dbg(byteproc->dev, "Frame vertical crop disabled\n");
	}
	if (r->width != compose->width || r->height != compose->height)
		dev_dbg(byteproc->dev, "Frame cropped as (%d,%d)%dx%d\n",
			r->left, r->top, r->width, r->height);
}

static void dcmipp_byteproc_adjust_compose(struct v4l2_rect *r,
					   const struct v4l2_mbus_framefmt *fmt)
{
	const struct dcmipp_byteproc_pix_map *vpix;

	r->top = 0;
	r->left = 0;

	/* Compose is not possible for JPEG or Bayer formats */
	if (fmt->code >= MEDIA_BUS_FMT_SBGGR8_1X8 &&
	    fmt->code <= MEDIA_BUS_FMT_JPEG_1X8) {
		r->width = fmt->width;
		r->height = fmt->height;
		return;
	}

	/* Prevent compose on formats which are not 1 or 2 bytes per pixel */
	vpix = dcmipp_byteproc_pix_map_by_code(fmt->code);
	if (vpix->bpp != 1 && vpix->bpp != 2) {
		r->width = fmt->width;
		r->height = fmt->height;
		return;
	}

	/* Adjust height - we can only perform 1/2 decimation */
	if (r->height <= (fmt->height / 2))
		r->height = fmt->height / 2;
	else
		r->height = fmt->height;

	/* Adjust width /2 or /4 for 8bits formats and /2 for 16bits formats */
	if (vpix->bpp == 1 && r->width <= (fmt->width / 4))
		r->width = fmt->width / 4;
	else if (r->width <= (fmt->width / 2))
		r->width = fmt->width / 2;
	else
		r->width = fmt->width;
}

static void dcmipp_byteproc_adjust_fmt_crop(struct dcmipp_byteproc_device *byteproc,
					    struct v4l2_mbus_framefmt *fmt,
					    struct v4l2_rect *crop)
{
	const struct dcmipp_byteproc_pix_map *vpix;

	/* Only accept code in the pix map table */
	vpix = dcmipp_byteproc_pix_map_by_code(fmt->code);
	if (!vpix) {
		fmt->code = fmt_default.code;
		vpix = dcmipp_byteproc_pix_map_by_code(fmt->code);
	}

	fmt->width = clamp_t(u32, fmt->width, DCMIPP_FRAME_MIN_WIDTH,
			     DCMIPP_FRAME_MAX_WIDTH);
	fmt->height = clamp_t(u32, fmt->height, DCMIPP_FRAME_MIN_HEIGHT,
			      DCMIPP_FRAME_MAX_HEIGHT);

	if (fmt->field == V4L2_FIELD_ANY || fmt->field == V4L2_FIELD_ALTERNATE)
		fmt->field = fmt_default.field;

	dcmipp_colorimetry_clamp(fmt);

	crop->top = 0;
	crop->left = 0;
	crop->width = fmt->width;
	crop->height = fmt->height;

	/* At this stage the output of compose is equivalent to crop */
	dcmipp_byteproc_adjust_crop(byteproc, crop, crop, vpix->bpp);
}

static int dcmipp_byteproc_init_cfg(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state)
{
	unsigned int i;

	for (i = 0; i < sd->entity.num_pads; i++) {
		struct v4l2_mbus_framefmt *mf;
		struct v4l2_rect *r;

		mf = v4l2_subdev_state_get_format(sd_state, i);
		*mf = fmt_default;

		if (IS_SINK(i))
			r = v4l2_subdev_state_get_compose(sd_state, i);
		else
			r = v4l2_subdev_state_get_crop(sd_state, i);

		r->top = 0;
		r->left = 0;
		r->width = DCMIPP_FMT_WIDTH_DEFAULT;
		r->height = DCMIPP_FMT_HEIGHT_DEFAULT;
	}

	return 0;
}

static int
dcmipp_byteproc_enum_mbus_code(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *sd_state,
			       struct v4l2_subdev_mbus_code_enum *code)
{
	const struct dcmipp_byteproc_pix_map *vpix;
	struct v4l2_mbus_framefmt *sink_fmt;

	if (IS_SINK(code->pad)) {
		if (code->index >= ARRAY_SIZE(dcmipp_byteproc_pix_map_list))
			return -EINVAL;
		vpix = &dcmipp_byteproc_pix_map_list[code->index];
		code->code = vpix->code;
	} else {
		/* byteproc doesn't support transformation on format */
		if (code->index > 0)
			return -EINVAL;

		sink_fmt = v4l2_subdev_state_get_format(sd_state, 0);
		code->code = sink_fmt->code;
	}

	return 0;
}

static int
dcmipp_byteproc_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_size_enum *fse)
{
	struct v4l2_rect *compose;

	if (fse->index)
		return -EINVAL;

	fse->min_width = DCMIPP_FRAME_MIN_WIDTH;
	fse->min_height = DCMIPP_FRAME_MIN_HEIGHT;

	if (IS_SINK(fse->pad)) {
		fse->max_width = DCMIPP_FRAME_MAX_WIDTH;
		fse->max_height = DCMIPP_FRAME_MAX_HEIGHT;
	} else {
		compose = v4l2_subdev_state_get_compose(sd_state, 0);
		fse->max_width = compose->width;
		fse->max_height = compose->height;
	}

	return 0;
}

static int dcmipp_byteproc_set_fmt(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *mf, *src_fmt;
	struct v4l2_rect *crop, *compose;

	if (byteproc->streaming)
		return -EBUSY;

	mf = v4l2_subdev_state_get_format(sd_state, fmt->pad);

	crop = v4l2_subdev_state_get_crop(sd_state, 1);
	compose = v4l2_subdev_state_get_compose(sd_state, 0);

	if (IS_SRC(fmt->pad)) {
		fmt->format = *v4l2_subdev_state_get_format(sd_state, 0);
		fmt->format.width = crop->width;
		fmt->format.height = crop->height;
	} else {
		dcmipp_byteproc_adjust_fmt_crop(byteproc, &fmt->format,
						crop);
		compose->top = 0;
		compose->left = 0;
		compose->width = fmt->format.width;
		compose->height = fmt->format.height;
		/* Set the same format on SOURCE pad with crop constraints */
		src_fmt = v4l2_subdev_state_get_format(sd_state, 1);
		*src_fmt = fmt->format;
		src_fmt->width = crop->width;
		src_fmt->height = crop->height;
	}

	*mf = fmt->format;

	return 0;
}

static int dcmipp_byteproc_get_selection(struct v4l2_subdev *sd,
					 struct v4l2_subdev_state *sd_state,
					 struct v4l2_subdev_selection *s)
{
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_rect *crop, *compose;

	/*
	 * In the HW, the decimation block is located prior to the crop hence:
	 * Compose is done on the sink pad
	 * Crop is done on the src pad
	 */
	if (IS_SINK(s->pad) &&
	    (s->target == V4L2_SEL_TGT_CROP ||
	     s->target == V4L2_SEL_TGT_CROP_BOUNDS ||
	     s->target == V4L2_SEL_TGT_CROP_DEFAULT))
		return -EINVAL;

	if (IS_SRC(s->pad) &&
	    (s->target == V4L2_SEL_TGT_COMPOSE ||
	     s->target == V4L2_SEL_TGT_COMPOSE_BOUNDS ||
	     s->target == V4L2_SEL_TGT_COMPOSE_DEFAULT))
		return -EINVAL;

	sink_fmt = v4l2_subdev_state_get_format(sd_state, 0);
	crop = v4l2_subdev_state_get_crop(sd_state, 1);
	compose = v4l2_subdev_state_get_compose(sd_state, 0);

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		s->r = *crop;
		break;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r = *compose;
		break;
	case V4L2_SEL_TGT_COMPOSE:
		s->r = *compose;
		break;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		s->r.top = 0;
		s->r.left = 0;
		s->r.width = sink_fmt->width;
		s->r.height = sink_fmt->height;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int dcmipp_byteproc_set_selection(struct v4l2_subdev *sd,
					 struct v4l2_subdev_state *sd_state,
					 struct v4l2_subdev_selection *s)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);
	const struct dcmipp_byteproc_pix_map *vpix;
	struct v4l2_mbus_framefmt *mf;
	struct v4l2_rect *crop, *compose;

	/*
	 * In the HW, the decimation block is located prior to the crop hence:
	 * Compose is done on the sink pad
	 * Crop is done on the src pad
	 */
	if ((s->target == V4L2_SEL_TGT_CROP ||
	     s->target == V4L2_SEL_TGT_CROP_BOUNDS ||
	     s->target == V4L2_SEL_TGT_CROP_DEFAULT) && IS_SINK(s->pad))
		return -EINVAL;

	if ((s->target == V4L2_SEL_TGT_COMPOSE ||
	     s->target == V4L2_SEL_TGT_COMPOSE_BOUNDS ||
	     s->target == V4L2_SEL_TGT_COMPOSE_DEFAULT) && IS_SRC(s->pad))
		return -EINVAL;

	crop = v4l2_subdev_state_get_crop(sd_state, 1);
	compose = v4l2_subdev_state_get_compose(sd_state, 0);

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		mf = v4l2_subdev_state_get_format(sd_state, 1);
		vpix = dcmipp_byteproc_pix_map_by_code(mf->code);
		dcmipp_byteproc_adjust_crop(byteproc, &s->r, compose, vpix->bpp);

		*crop = s->r;
		mf->width = s->r.width;
		mf->height = s->r.height;

		dev_dbg(byteproc->dev, "s_selection: crop %ux%u@(%u,%u)\n",
			crop->width, crop->height, crop->left, crop->top);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		mf = v4l2_subdev_state_get_format(sd_state, 0);
		dcmipp_byteproc_adjust_compose(&s->r, mf);
		*compose = s->r;

		mf = v4l2_subdev_state_get_format(sd_state, 1);
		vpix = dcmipp_byteproc_pix_map_by_code(mf->code);
		dcmipp_byteproc_adjust_crop(byteproc, &s->r, compose, vpix->bpp);
		*crop = s->r;

		mf->width = s->r.width;
		mf->height = s->r.height;

		dev_dbg(byteproc->dev, "s_selection: compose %ux%u@(%u,%u)\n",
			compose->width, compose->height,
			compose->left, compose->top);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const unsigned int dcmipp_frates[] = {1, 2, 4, 8};

static int dcmipp_byteproc_enum_frame_interval
				(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_frame_interval_enum *fie)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);
	struct v4l2_fract *sink_interval = &byteproc->sink_interval;
	unsigned int ratio;

	if (fie->pad > 1 ||
	    fie->index >= (IS_SRC(fie->pad) ? ARRAY_SIZE(dcmipp_frates) : 1) ||
	    fie->width > DCMIPP_FRAME_MAX_WIDTH ||
	    fie->height > DCMIPP_FRAME_MAX_HEIGHT)
		return -EINVAL;

	if (IS_SINK(fie->pad)) {
		fie->interval = *sink_interval;
		return 0;
	}

	ratio = dcmipp_frates[fie->index];

	fie->interval.numerator = sink_interval->numerator * ratio;
	fie->interval.denominator = sink_interval->denominator;

	return 0;
}

static const struct v4l2_subdev_pad_ops dcmipp_byteproc_pad_ops = {
	.init_cfg		= dcmipp_byteproc_init_cfg,
	.enum_mbus_code		= dcmipp_byteproc_enum_mbus_code,
	.enum_frame_size	= dcmipp_byteproc_enum_frame_size,
	.enum_frame_interval	= dcmipp_byteproc_enum_frame_interval,
	.get_fmt		= v4l2_subdev_get_fmt,
	.set_fmt		= dcmipp_byteproc_set_fmt,
	.get_selection		= dcmipp_byteproc_get_selection,
	.set_selection		= dcmipp_byteproc_set_selection,
};

static int dcmipp_byteproc_configure_scale_crop
			(struct dcmipp_byteproc_device *byteproc)
{
	const struct dcmipp_byteproc_pix_map *vpix;
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *sink_fmt;
	u32 hprediv, vprediv;
	struct v4l2_rect *compose, *crop;
	u32 cstr = 0, cszr = 0;
	u32 val = 0;

	state = v4l2_subdev_lock_and_get_active_state(&byteproc->sd);
	sink_fmt = v4l2_subdev_state_get_format(state, 0);
	compose = v4l2_subdev_state_get_compose(state, 0);
	crop = v4l2_subdev_state_get_crop(state, 1);
	v4l2_subdev_unlock_state(state);

	/* find output format bpp */
	vpix = dcmipp_byteproc_pix_map_by_code(sink_fmt->code);
	if (!vpix)
		return -EINVAL;

	/* clear decimation/crop/swap yuv */
	reg_clear(byteproc, DCMIPP_P0PPCR, DCMIPP_P0PPCR_BSM_MASK);
	reg_clear(byteproc, DCMIPP_P0PPCR, DCMIPP_P0PPCR_LSM);
	reg_clear(byteproc, DCMIPP_P0PPCR, DCMIPP_P0PPCR_SWAPYUV);
	reg_write(byteproc, DCMIPP_P0SCSTR, 0);
	reg_write(byteproc, DCMIPP_P0SCSZR, 0);

	/* Ignore decimation/crop with JPEG */
	if (vpix->code == MEDIA_BUS_FMT_JPEG_1X8)
		return 0;

	/* decimation */
	hprediv = sink_fmt->width / compose->width;
	if (hprediv == 4)
		val |= DCMIPP_P0PPCR_BSM_1_4 << DCMIPP_P0PPCR_BSM_SHIFT;
	else if ((vpix->code == MEDIA_BUS_FMT_Y8_1X8) && (hprediv == 2))
		val |= DCMIPP_P0PPCR_BSM_1_2 << DCMIPP_P0PPCR_BSM_SHIFT;
	else if (hprediv == 2)
		val |= DCMIPP_P0PPCR_BSM_2_4 << DCMIPP_P0PPCR_BSM_SHIFT;

	vprediv = sink_fmt->height / compose->height;
	if (vprediv == 2)
		val |= DCMIPP_P0PPCR_LSM | DCMIPP_P0PPCR_OELS;

	/*
	 * Perform a SWAP YUV if input is parallel since in this mode
	 * the DCMIPP will swap YUV by default
	 */
	if (sink_fmt->code == MEDIA_BUS_FMT_YUYV8_2X8 ||
	    sink_fmt->code == MEDIA_BUS_FMT_YVYU8_2X8 ||
	    sink_fmt->code == MEDIA_BUS_FMT_UYVY8_2X8 ||
	    sink_fmt->code == MEDIA_BUS_FMT_VYUY8_2X8)
		val |= DCMIPP_P0PPCR_SWAPYUV;

	/* decimate using bytes and lines skipping */
	if (val) {
		reg_set(byteproc, DCMIPP_P0PPCR, val);

		dev_dbg(byteproc->dev, "decimate to %dx%d [prediv=%dx%d]\n",
			compose->width, compose->height,
			hprediv, vprediv);
	}

	/* Configure crop if necessary */
	if (crop->width != compose->width) {
		cstr |= (((crop->left * vpix->bpp) / 4) <<
			DCMIPP_P0SCSTR_HSTART_SHIFT);
		cszr |= (((crop->width * vpix->bpp) / 4) <<
			DCMIPP_P0SCSZR_HSIZE_SHIFT);
		cszr |= DCMIPP_P0SCSZR_ENABLE;
	}
	if (crop->height != compose->height) {
		cstr |= crop->top << DCMIPP_P0SCSTR_VSTART_SHIFT;
		cszr |= crop->height << DCMIPP_P0SCSZR_VSIZE_SHIFT;
		cszr |= DCMIPP_P0SCSZR_ENABLE;
	}
	if (cstr && cszr) {
		dev_dbg(byteproc->dev, "crop to (%d,%d)%dx%d\n",
			crop->left, crop->top, crop->width, crop->height);
		reg_write(byteproc, DCMIPP_P0SCSTR, cstr);
		reg_write(byteproc, DCMIPP_P0SCSZR, cszr);
	}

	return 0;
}

static void dcmipp_byteproc_configure_framerate
			(struct dcmipp_byteproc_device *byteproc)
{
	/* Frame skipping */
	reg_clear(byteproc, DCMIPP_P0FCTCR, DCMIPP_P0FCTCR_FRATE_MASK);
	reg_set(byteproc, DCMIPP_P0FCTCR, byteproc->frate);
}

static int dcmipp_byteproc_g_frame_interval(struct v4l2_subdev *sd,
					    struct v4l2_subdev_frame_interval *fi)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);

	if (IS_SINK(fi->pad))
		fi->interval = byteproc->sink_interval;
	else
		fi->interval = byteproc->src_interval;

	return 0;
}

static int dcmipp_byteproc_s_frame_interval(struct v4l2_subdev *sd,
					    struct v4l2_subdev_frame_interval *fi)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);

	if (byteproc->streaming)
		return -EBUSY;

	if (fi->interval.numerator == 0 || fi->interval.denominator == 0)
		fi->interval = byteproc->sink_interval;

	if (IS_SINK(fi->pad)) {
		/*
		 * Setting sink frame interval resets frame skipping.
		 * Sink frame interval is propagated to src.
		 */
		byteproc->frate = 0;
		byteproc->sink_interval = fi->interval;
		byteproc->src_interval = byteproc->sink_interval;
	} else {
		unsigned int ratio;

		/* Normalize ratio */
		ratio = (byteproc->sink_interval.denominator *
			 fi->interval.numerator) /
			(byteproc->sink_interval.numerator *
			 fi->interval.denominator);

		/* Hardware can skip 1 frame over 2, 4 or 8 */
		byteproc->frate = ratio >= 8 ? 3 :
				  ratio >= 4 ? 2 :
				  ratio >= 2 ? 1 : 0;

		ratio = dcmipp_frates[byteproc->frate];

		/* Adjust src frame interval to what hardware can really do */
		byteproc->src_interval.numerator =
			byteproc->sink_interval.numerator * ratio;
		byteproc->src_interval.denominator =
			byteproc->sink_interval.denominator;
	}

	return 0;
}

static int dcmipp_byteproc_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *s_subdev;
	struct media_pad *pad;
	int ret = 0;

	/* Get source subdev */
	pad = media_pad_remote_pad_first(&sd->entity.pads[0]);
	if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
		return -EINVAL;
	s_subdev = media_entity_to_v4l2_subdev(pad->entity);

	if (enable) {
		dcmipp_byteproc_configure_framerate(byteproc);

		ret = dcmipp_byteproc_configure_scale_crop(byteproc);
		if (ret)
			return ret;

		ret = v4l2_subdev_call(s_subdev, video, s_stream, enable);
		if (ret < 0) {
			dev_err(byteproc->dev,
				"failed to start source subdev streaming (%d)\n",
				ret);
			return ret;
		}
	} else {
		ret = v4l2_subdev_call(s_subdev, video, s_stream, enable);
		if (ret < 0) {
			dev_err(byteproc->dev,
				"failed to stop source subdev streaming (%d)\n",
				ret);
			return ret;
		}
	}

	byteproc->streaming = enable;

	return 0;
}

static const struct v4l2_subdev_video_ops dcmipp_byteproc_video_ops = {
	.g_frame_interval = dcmipp_byteproc_g_frame_interval,
	.s_frame_interval = dcmipp_byteproc_s_frame_interval,
	.s_stream = dcmipp_byteproc_s_stream,
};

static const struct v4l2_subdev_ops dcmipp_byteproc_ops = {
	.pad = &dcmipp_byteproc_pad_ops,
	.video = &dcmipp_byteproc_video_ops,
};

void dcmipp_byteproc_ent_release(struct dcmipp_ent_device *ved)
{
	struct dcmipp_byteproc_device *byteproc =
			container_of(ved, struct dcmipp_byteproc_device, ved);

	dcmipp_ent_sd_unregister(ved, &byteproc->sd);
	kfree(byteproc);
}

struct dcmipp_ent_device *
dcmipp_byteproc_ent_init(const char *entity_name,
			 struct dcmipp_device *dcmipp)
{
	struct dcmipp_byteproc_device *byteproc;
	const unsigned long pads_flag[] = {
		MEDIA_PAD_FL_SINK, MEDIA_PAD_FL_SOURCE,
	};
	int ret;

	/* Allocate the byteproc struct */
	byteproc = kzalloc(sizeof(*byteproc), GFP_KERNEL);
	if (!byteproc)
		return ERR_PTR(-ENOMEM);

	byteproc->regs = dcmipp->regs;

	/* Initialize ved and sd */
	ret = dcmipp_ent_sd_register(&byteproc->ved, &byteproc->sd,
				     &dcmipp->v4l2_dev, entity_name,
				     MEDIA_ENT_F_PROC_VIDEO_SCALER,
				     ARRAY_SIZE(pads_flag), pads_flag,
				     NULL,
				     &dcmipp_byteproc_ops,
				     NULL, NULL);
	if (ret) {
		kfree(byteproc);
		return ERR_PTR(ret);
	}

	byteproc->ved.dcmipp = dcmipp;
	byteproc->dev = dcmipp->dev;

	return &byteproc->ved;
}
