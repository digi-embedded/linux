// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for STM32 Digital Camera Memory Interface Pixel Processor
 *
 * Copyright (C) STMicroelectronics SA 2021
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          Alain Volmat <alain.volmat@foss.st.com>
 *          for STMicroelectronics.
 */

#include <linux/component.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-rect.h>
#include <media/v4l2-subdev.h>

#include "dcmipp-common.h"
#include "dcmipp-colorconv.h"

#define DCMIPP_PIXELPROC_DRV_NAME "dcmipp-pixelproc"

#define DCMIPP_FMT_WIDTH_DEFAULT  640
#define DCMIPP_FMT_HEIGHT_DEFAULT 480

#define DCMIPP_P1FCTCR (0x900)
#define DCMIPP_P2FCTCR (0xD00)
#define DCMIPP_PxFCTCR(id) (((id) == 1) ? DCMIPP_P1FCTCR :\
			   DCMIPP_P2FCTCR)
#define DCMIPP_PxFCTCR_FRATE_MASK GENMASK(1, 0)
#define DCMIPP_P1CRSTR (0x904)
#define DCMIPP_P2CRSTR (0xD04)
#define DCMIPP_PxCRSTR(id) (((id) == 1) ? DCMIPP_P1CRSTR :\
			   DCMIPP_P2CRSTR)
#define DCMIPP_PxCRSTR_HSTART_SHIFT	0
#define DCMIPP_PxCRSTR_VSTART_SHIFT	16
#define DCMIPP_P1CRSZR (0x908)
#define DCMIPP_P2CRSZR (0xD08)
#define DCMIPP_PxCRSZR(id) (((id) == 1) ? DCMIPP_P1CRSZR :\
			   DCMIPP_P2CRSZR)
#define DCMIPP_PxCRSZR_ENABLE BIT(31)
#define DCMIPP_PxCRSZR_HSIZE_SHIFT	0
#define DCMIPP_PxCRSZR_VSIZE_SHIFT	16

#define DCMIPP_P1DSCR (0x910)
#define DCMIPP_P2DSCR (0xD10)
#define DCMIPP_PxDSCR(id) (((id) == 1) ? DCMIPP_P1DSCR :\
			   DCMIPP_P2DSCR)
#define DCMIPP_PxDSCR_HDIV_SHIFT 0
#define DCMIPP_PxDSCR_HDIV_MASK GENMASK(9, 0)
#define DCMIPP_PxDSCR_VDIV_SHIFT 16
#define DCMIPP_PxDSCR_VDIV_MASK GENMASK(25, 16)
#define DCMIPP_PxDSCR_ENABLE BIT(31)

#define DCMIPP_P1DSRTIOR (0x914)
#define DCMIPP_P2DSRTIOR (0xD14)
#define DCMIPP_PxDSRTIOR(id) (((id) == 1) ? DCMIPP_P1DSRTIOR :\
			   DCMIPP_P2DSRTIOR)
#define DCMIPP_PxDSRTIOR_HRATIO_SHIFT 0
#define DCMIPP_PxDSRTIOR_HRATIO_MASK GENMASK(15, 0)
#define DCMIPP_PxDSRTIOR_VRATIO_SHIFT 16
#define DCMIPP_PxDSRTIOR_VRATIO_MASK GENMASK(31, 16)

#define DCMIPP_P1DSSZR (0x918)
#define DCMIPP_P2DSSZR (0xD18)
#define DCMIPP_PxDSSZR(id) (((id) == 1) ? DCMIPP_P1DSSZR :\
			   DCMIPP_P2DSSZR)
#define DCMIPP_PxDSSZR_HSIZE_SHIFT 0
#define DCMIPP_PxDSSZR_HSIZE_MASK GENMASK(11, 0)
#define DCMIPP_PxDSSZR_VSIZE_SHIFT 16
#define DCMIPP_PxDSSZR_VSIZE_MASK GENMASK(27, 16)

#define DCMIPP_P1GMCR (0x970)
#define DCMIPP_P2GMCR (0x970)
#define DCMIPP_PxGMCR(id) (((id) == 1) ? DCMIPP_P1GMCR :\
			   DCMIPP_P2GMCR)
#define DCMIPP_PxGMCR_ENABLE BIT(0)

#define DCMIPP_P1YUVCR (0x980)
#define DCMIPP_P1YUVCR_ENABLE BIT(0)
#define DCMIPP_P1YUVCR_TYPE_YUV 0
#define DCMIPP_P1YUVCR_TYPE_RGB BIT(1)
#define DCMIPP_P1YUVCR_CLAMP BIT(2)
#define DCMIPP_P1YUVRR1 (0x984)
#define DCMIPP_P1YUVRR2 (0x988)
#define DCMIPP_P1YUVGR1 (0x98C)
#define DCMIPP_P1YUVGR2 (0x990)
#define DCMIPP_P1YUVBR1 (0x994)
#define DCMIPP_P1YUVBR2 (0x998)

#define DCMIPP_P1PPCR (0x9C0)
#define DCMIPP_P2PPCR (0xDC0)
#define DCMIPP_PxPPCR(id) (((id) == 1) ? DCMIPP_P1PPCR :\
			   DCMIPP_P2PPCR)
#define DCMIPP_PxPPCR_FORMAT_SHIFT 0
#define DCMIPP_PxPPCR_FORMAT_MASK GENMASK(3, 0)
#define DCMIPP_PxPPCR_FORMAT_RGB888_OR_YUV444_1BUFFER 0x0
#define DCMIPP_PxPPCR_FORMAT_RGB565 0x1
#define DCMIPP_PxPPCR_FORMAT_ARGB8888 0x2
#define DCMIPP_PxPPCR_FORMAT_RGBA8888 0x3
#define DCMIPP_PxPPCR_FORMAT_Y8 0x4
#define DCMIPP_PxPPCR_FORMAT_YUV444 0x5
#define DCMIPP_PxPPCR_FORMAT_YUYV 0x6
#define DCMIPP_P1PPCR_FORMAT_NV61 0x7
#define DCMIPP_P1PPCR_FORMAT_NV21 0x8
#define DCMIPP_P1PPCR_FORMAT_YV12 0x9
#define DCMIPP_PxPPCR_FORMAT_UYVY 0xa

#define DCMIPP_PxPPCR_SWAPRB BIT(4)

#define IS_SINK(pad) (!(pad))
#define IS_SRC(pad)  ((pad))
#define PAD_STR(pad) (IS_SRC((pad))) ? "src" : "sink"

#define PIXELPROC_MEDIA_BUS_SRC_FMT_DEFAULT MEDIA_BUS_FMT_RGB565_2X8_LE
#define PIXELPROC_MEDIA_BUS_SINK_FMT_DEFAULT MEDIA_BUS_FMT_RGB888_1X24

struct dcmipp_pixelproc_pix_map {
	unsigned int code;
	unsigned int ppcr_fmt;
	unsigned int swap_uv;
};

#define PIXMAP_MBUS_PPCR_SWAPUV(mbus, pp_code, swap)	\
		{						\
			.code = MEDIA_BUS_FMT_##mbus,		\
			.ppcr_fmt = pp_code,	\
			.swap_uv = swap,	\
		}
static const struct dcmipp_pixelproc_pix_map dcmipp_pixelproc_sink_pix_map_list[] = {
	PIXMAP_MBUS_PPCR_SWAPUV(RGB888_1X24, 0, 0),
	PIXMAP_MBUS_PPCR_SWAPUV(YUV8_1X24, 0, 0),
};

static const struct dcmipp_pixelproc_pix_map dcmipp_pixelproc_src_pix_map_list[] = {
	PIXMAP_MBUS_PPCR_SWAPUV(RGB888_1X24, DCMIPP_PxPPCR_FORMAT_RGB888_OR_YUV444_1BUFFER, 0),
	PIXMAP_MBUS_PPCR_SWAPUV(RGB565_2X8_LE, DCMIPP_PxPPCR_FORMAT_RGB565, 0),
	PIXMAP_MBUS_PPCR_SWAPUV(YUYV8_2X8, DCMIPP_PxPPCR_FORMAT_YUYV, 0),
	PIXMAP_MBUS_PPCR_SWAPUV(YVYU8_2X8, DCMIPP_PxPPCR_FORMAT_YUYV, 1),
	PIXMAP_MBUS_PPCR_SWAPUV(UYVY8_2X8, DCMIPP_PxPPCR_FORMAT_UYVY, 0),
	PIXMAP_MBUS_PPCR_SWAPUV(VYUY8_2X8, DCMIPP_PxPPCR_FORMAT_UYVY, 1),
	PIXMAP_MBUS_PPCR_SWAPUV(Y8_1X8, DCMIPP_PxPPCR_FORMAT_Y8, 0),
	PIXMAP_MBUS_PPCR_SWAPUV(YUYV8_1_5X8, DCMIPP_P1PPCR_FORMAT_NV21, 0), /* FIXME no mbus code for semiplanar (NV12) */
	PIXMAP_MBUS_PPCR_SWAPUV(YVYU8_1_5X8, DCMIPP_P1PPCR_FORMAT_NV21, 1), /* FIXME no mbus code for semiplanar (NV21) */
	PIXMAP_MBUS_PPCR_SWAPUV(YUYV8_1X16,  DCMIPP_P1PPCR_FORMAT_NV61, 0), /* FIXME no mbus code for semiplanar (NV16)*/
	PIXMAP_MBUS_PPCR_SWAPUV(YVYU8_1X16,  DCMIPP_P1PPCR_FORMAT_NV61, 1), /* FIXME no mbus code for semiplanar (NV61)*/
	PIXMAP_MBUS_PPCR_SWAPUV(UYVY8_1_5X8, DCMIPP_P1PPCR_FORMAT_YV12, 0), /* FIXME no mbus code for planar (I420/YU12)*/
	PIXMAP_MBUS_PPCR_SWAPUV(VYUY8_1_5X8, DCMIPP_P1PPCR_FORMAT_YV12, 1), /* FIXME no mbus code for planar (YV12)*/
};

static const struct dcmipp_pixelproc_pix_map *
dcmipp_pixelproc_pix_map_by_index(unsigned int i, unsigned int pad)
{
	const struct dcmipp_pixelproc_pix_map *l;
	unsigned int size;

	if (IS_SRC(pad)) {
		l = dcmipp_pixelproc_src_pix_map_list;
		size = ARRAY_SIZE(dcmipp_pixelproc_src_pix_map_list);
	} else {
		l = dcmipp_pixelproc_sink_pix_map_list;
		size = ARRAY_SIZE(dcmipp_pixelproc_sink_pix_map_list);
	}

	if (i >= size)
		return NULL;

	return &l[i];
}

static const struct dcmipp_pixelproc_pix_map *
dcmipp_pixelproc_pix_map_by_code(u32 code, unsigned int pad)
{
	const struct dcmipp_pixelproc_pix_map *l;
	unsigned int size;
	unsigned int i;

	if (IS_SRC(pad)) {
		l = dcmipp_pixelproc_src_pix_map_list;
		size = ARRAY_SIZE(dcmipp_pixelproc_src_pix_map_list);
	} else {
		l = dcmipp_pixelproc_sink_pix_map_list;
		size = ARRAY_SIZE(dcmipp_pixelproc_sink_pix_map_list);
	}

	for (i = 0; i < size; i++) {
		if (l[i].code == code)
			return &l[i];
	}

	return NULL;
}

struct dcmipp_pixelproc_device {
	struct dcmipp_ent_device ved;
	struct v4l2_subdev sd;
	struct device *dev;
	struct v4l2_mbus_framefmt sink_fmt;
	struct v4l2_mbus_framefmt src_fmt;
	bool streaming;
	/* Protect this data structure */
	struct mutex lock;

	void __iomem *regs;
	struct v4l2_ctrl_handler ctrls;

	u32 pipe_id;

	struct v4l2_fract sink_interval;
	unsigned int frate;
	u32 src_code;
	struct v4l2_rect crop;
	struct v4l2_rect compose;
};

static const struct v4l2_mbus_framefmt fmt_sink_default = {
	.width = DCMIPP_FMT_WIDTH_DEFAULT,
	.height = DCMIPP_FMT_HEIGHT_DEFAULT,
	.code = PIXELPROC_MEDIA_BUS_SINK_FMT_DEFAULT,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_DEFAULT,
};

static const struct v4l2_mbus_framefmt fmt_src_default = {
	.width = DCMIPP_FMT_WIDTH_DEFAULT,
	.height = DCMIPP_FMT_HEIGHT_DEFAULT,
	.code = PIXELPROC_MEDIA_BUS_SRC_FMT_DEFAULT,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_DEFAULT,
};

static const struct v4l2_rect crop_min = {
	.width = DCMIPP_FRAME_MIN_WIDTH,
	.height = DCMIPP_FRAME_MIN_HEIGHT,
	.top = 0,
	.left = 0,
};

#define DCMIPP_MAX_DOWNSIZE_RATIO 8

/*
 * Functions handling controls
 */
#define V4L2_CID_PIXELPROC_GAMMA_CORRECTION	(V4L2_CID_USER_BASE | 0x1001)

static int dcmipp_pixelproc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct dcmipp_pixelproc_device *pixelproc =
		container_of(ctrl->handler,
			     struct dcmipp_pixelproc_device, ctrls);
	int ret = 0;

	mutex_lock(&pixelproc->lock);

	if (pixelproc->streaming) {
		ret = -EBUSY;
		goto out;
	}

	switch (ctrl->id) {
	case V4L2_CID_PIXELPROC_GAMMA_CORRECTION:
		reg_write(pixelproc, DCMIPP_PxGMCR(pixelproc->pipe_id),
			  (ctrl->val ? DCMIPP_PxGMCR_ENABLE : 0));
		break;
	}

out:
	mutex_unlock(&pixelproc->lock);
	return ret;
};

static const struct v4l2_ctrl_ops dcmipp_pixelproc_ctrl_ops = {
	.s_ctrl = dcmipp_pixelproc_s_ctrl,
};

static const struct v4l2_ctrl_config dcmipp_pixelproc_ctrls[] = {
	{
		.ops		= &dcmipp_pixelproc_ctrl_ops,
		.id		= V4L2_CID_PIXELPROC_GAMMA_CORRECTION,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Gamma correction",
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
	}
};

static struct v4l2_rect
dcmipp_pixelproc_get_crop_bound(const struct v4l2_mbus_framefmt *fmt)
{
	/* Get the crop bounds to clamp the crop rectangle correctly */
	struct v4l2_rect r = {
		.left = 0,
		.top = 0,
		.width = fmt->width,
		.height = fmt->height,
	};
	return r;
}

static void dcmipp_pixelproc_adjust_crop(struct v4l2_rect *r,
					 const struct v4l2_mbus_framefmt *fmt)
{
	const struct v4l2_rect src_rect =
		dcmipp_pixelproc_get_crop_bound(fmt);

	/* Disallow rectangles smaller than the minimal one. */
	v4l2_rect_set_min_size(r, &crop_min);
	v4l2_rect_map_inside(r, &src_rect);
}

static void dcmipp_pixelproc_adjust_fmt(struct v4l2_mbus_framefmt *fmt, u32 pad)
{
	const struct dcmipp_pixelproc_pix_map *vpix;

	/* Only accept code in the pix map table */
	vpix = dcmipp_pixelproc_pix_map_by_code(fmt->code, pad);
	if (!vpix)
		fmt->code = IS_SRC(pad) ? fmt_src_default.code :
					  fmt_sink_default.code;

	fmt->width = clamp_t(u32, fmt->width, DCMIPP_FRAME_MIN_WIDTH,
			     DCMIPP_FRAME_MAX_WIDTH) & ~1;
	fmt->height = clamp_t(u32, fmt->height, DCMIPP_FRAME_MIN_HEIGHT,
			      DCMIPP_FRAME_MAX_HEIGHT);

	if (fmt->field == V4L2_FIELD_ANY || fmt->field == V4L2_FIELD_ALTERNATE)
		fmt->field = IS_SRC(pad) ? fmt_src_default.field :
					   fmt_sink_default.field;

	dcmipp_colorimetry_clamp(fmt);
}

static int dcmipp_pixelproc_init_cfg(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state)
{
	unsigned int i;

	for (i = 0; i < sd->entity.num_pads; i++) {
		struct v4l2_mbus_framefmt *mf;

		mf = v4l2_subdev_get_try_format(sd, state, i);
		*mf = IS_SRC(i) ? fmt_src_default : fmt_sink_default;
	}

	return 0;
}

static int dcmipp_pixelproc_enum_mbus_code(struct v4l2_subdev *sd,
					   struct v4l2_subdev_state *state,
					   struct v4l2_subdev_mbus_code_enum *code)
{
	const struct dcmipp_pixelproc_pix_map *vpix;

	vpix = dcmipp_pixelproc_pix_map_by_index(code->index, code->pad);
	if (!vpix)
		return -EINVAL;

	code->code = vpix->code;

	return 0;
}

static int dcmipp_pixelproc_enum_frame_size(struct v4l2_subdev *sd,
					    struct v4l2_subdev_state *state,
					    struct v4l2_subdev_frame_size_enum *fse)
{
	const struct dcmipp_pixelproc_pix_map *vpix;

	if (fse->index)
		return -EINVAL;

	/* Only accept code in the pix map table */
	vpix = dcmipp_pixelproc_pix_map_by_code(fse->code, fse->pad);
	if (!vpix)
		return -EINVAL;

	fse->min_width = DCMIPP_FRAME_MIN_WIDTH;
	fse->max_width = DCMIPP_FRAME_MAX_WIDTH;
	fse->min_height = DCMIPP_FRAME_MIN_HEIGHT;
	fse->max_height = DCMIPP_FRAME_MAX_HEIGHT;

	return 0;
}

static int dcmipp_pixelproc_get_fmt(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_format *fmt)
{
	struct dcmipp_pixelproc_device *pixelproc = v4l2_get_subdevdata(sd);

	mutex_lock(&pixelproc->lock);

	if (IS_SINK(fmt->pad))
		fmt->format = fmt->which == V4L2_SUBDEV_FORMAT_TRY ?
			      *v4l2_subdev_get_try_format(sd, state, 0) :
			      pixelproc->sink_fmt;
	else
		fmt->format = fmt->which == V4L2_SUBDEV_FORMAT_TRY ?
			      *v4l2_subdev_get_try_format(sd, state, 0) :
			      pixelproc->src_fmt;

	mutex_unlock(&pixelproc->lock);

	return 0;
}

static int dcmipp_pixelproc_set_fmt(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_format *fmt)
{
	struct dcmipp_pixelproc_device *pixelproc = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *pad_fmt;
	int ret = 0;

	mutex_lock(&pixelproc->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (pixelproc->streaming) {
			ret = -EBUSY;
			goto out;
		}

		if (IS_SINK(fmt->pad))
			pad_fmt = &pixelproc->sink_fmt;
		else
			pad_fmt = &pixelproc->src_fmt;

	} else {
		pad_fmt = v4l2_subdev_get_try_format(sd, state, 0);
	}

	dcmipp_pixelproc_adjust_fmt(&fmt->format, fmt->pad);

	dev_dbg(pixelproc->dev, "%s: %s format update: old:%dx%d (0x%x, %d, %d, %d, %d) new:%dx%d (0x%x, %d, %d, %d, %d)\n",
		pixelproc->sd.name,
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

	/* Update sink pad crop - compose */
	if (IS_SINK(fmt->pad) && fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		/* Update sink pad crop */
		pixelproc->crop.top = 0;
		pixelproc->crop.left = 0;
		pixelproc->crop.width = fmt->format.width;
		pixelproc->crop.height = fmt->format.height;

		pixelproc->compose.top = 0;
		pixelproc->compose.left = 0;
		pixelproc->compose.width = fmt->format.width;
		pixelproc->compose.height = fmt->format.height;

		/* Update src pad format & size */
		pixelproc->src_fmt = fmt_src_default;
		if (fmt->format.code >= MEDIA_BUS_FMT_Y8_1X8 &&
		    fmt->format.code < MEDIA_BUS_FMT_SBGGR8_1X8)
			pixelproc->src_fmt.code = MEDIA_BUS_FMT_YUYV8_2X8;
		else
			pixelproc->src_fmt.code = MEDIA_BUS_FMT_RGB565_2X8_LE;
		pixelproc->src_fmt.width = fmt->format.width;
		pixelproc->src_fmt.height = fmt->format.height;
	}

out:
	mutex_unlock(&pixelproc->lock);

	return ret;
}

static int dcmipp_pixelproc_get_selection(struct v4l2_subdev *sd,
					  struct v4l2_subdev_state *state,
					  struct v4l2_subdev_selection *s)
{
	struct dcmipp_pixelproc_device *pixelproc = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_rect *crop, *compose;

	if (IS_SRC(s->pad))
		return -EINVAL;

	if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sink_fmt = &pixelproc->sink_fmt;
		crop = &pixelproc->crop;
		compose = &pixelproc->compose;
	} else {
		sink_fmt = v4l2_subdev_get_try_format(sd, state, s->pad);
		crop = v4l2_subdev_get_try_crop(sd, state, s->pad);
		compose = v4l2_subdev_get_try_compose(sd, state, s->pad);
	}

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		s->r = *crop;
		break;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r = dcmipp_pixelproc_get_crop_bound(sink_fmt);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		s->r = *compose;
		break;
	/* TODO - not sure how to define BOUND here */
	default:
		return -EINVAL;
	}

	return 0;
}

static int dcmipp_pixelproc_set_selection(struct v4l2_subdev *sd,
					  struct v4l2_subdev_state *state,
					  struct v4l2_subdev_selection *s)
{
	struct dcmipp_pixelproc_device *pixelproc = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_rect *crop, *compose;

	if (IS_SRC(s->pad))
		return -EINVAL;

	if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sink_fmt = &pixelproc->sink_fmt;
		crop = &pixelproc->crop;
		compose = &pixelproc->compose;
	} else {
		sink_fmt = v4l2_subdev_get_try_format(sd, state, s->pad);
		crop = v4l2_subdev_get_try_crop(sd, state, s->pad);
		compose = v4l2_subdev_get_try_compose(sd, state, s->pad);
	}

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		dcmipp_pixelproc_adjust_crop(&s->r, sink_fmt);

		*crop = s->r;

		/* Setting the crop also set the compose identically */
		*compose = *crop;

		/*
		 * In case of setting the crop with ACTIVE set, we need to
		 * update the source pad size
		 */
		if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
			pixelproc->src_fmt.width = s->r.width;
			pixelproc->src_fmt.height = s->r.height;
		}
		/* TODO - when not in format active, we should also update the
		 * try src pad format
		 */

		dev_dbg(pixelproc->dev, "s_selection: crop %ux%u@(%u,%u)\n",
			crop->width, crop->height, crop->left, crop->top);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		if (s->r.width > crop->width)
			s->r.width = crop->width;
		else if (s->r.width < (crop->width / DCMIPP_MAX_DOWNSIZE_RATIO))
			s->r.width = crop->width / DCMIPP_MAX_DOWNSIZE_RATIO;

		if (s->r.height > crop->height)
			s->r.height = crop->height;
		else if (s->r.height < (crop->height / DCMIPP_MAX_DOWNSIZE_RATIO))
			s->r.height = crop->width / DCMIPP_MAX_DOWNSIZE_RATIO;
		s->r.top = 0;
		s->r.left = 0;

		*compose = s->r;

		/*
		 * In case of setting the compose with ACTIVE set, we need to
		 * update the source pad size
		 */
		if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
			pixelproc->src_fmt.width = s->r.width;
			pixelproc->src_fmt.height = s->r.height;
		}
		/* TODO - when not in format active, we should also update the
		 * try src pad format
		 */

		dev_dbg(pixelproc->dev, "s_selection: compose %ux%u@(%u,%u)\n",
			s->r.width, s->r.height, s->r.left, s->r.top);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_subdev_pad_ops dcmipp_pixelproc_pad_ops = {
	.init_cfg		= dcmipp_pixelproc_init_cfg,
	.enum_mbus_code		= dcmipp_pixelproc_enum_mbus_code,
	.enum_frame_size	= dcmipp_pixelproc_enum_frame_size,
	.get_fmt		= dcmipp_pixelproc_get_fmt,
	.set_fmt		= dcmipp_pixelproc_set_fmt,
	.get_selection		= dcmipp_pixelproc_get_selection,
	.set_selection		= dcmipp_pixelproc_set_selection,
};

static int
dcmipp_pixelproc_colorconv_config(struct dcmipp_pixelproc_device *pixelproc)
{
	struct dcmipp_colorconv_config ccconf = { 0 };
	int i, ret = 0;
	unsigned int val = 0;

	ret = dcmipp_colorconv_configure(pixelproc->dev, &pixelproc->sink_fmt,
					 &pixelproc->src_fmt, &ccconf);
	if (ret)
		return ret;

	for (i = 0; i < 6; i++)
		reg_write(pixelproc, DCMIPP_P1YUVRR1 + (4 * i),
			  ccconf.conv_matrix[i]);

	if (ccconf.clamping)
		val |= DCMIPP_P1YUVCR_CLAMP;
	if (ccconf.clamping_as_rgb)
		val |= DCMIPP_P1YUVCR_TYPE_RGB;
	if (ccconf.enable)
		val |= DCMIPP_P1YUVCR_ENABLE;

	reg_write(pixelproc, DCMIPP_P1YUVCR, val);

	return 0;
}

#define DCMIPP_PIXELPROC_HVRATIO_CONS	8192
#define DCMIPP_PIXELPROC_HVRATIO_MAX	65535
#define DCMIPP_PIXELPROC_HVDIV_CONS	1024
#define DCMIPP_PIXELPROC_HVDIV_MAX	1023
static void
dcmipp_pixelproc_set_downsize(struct dcmipp_pixelproc_device *pixelproc)
{
	unsigned int hratio, vratio, hdiv, vdiv;

	hratio = (pixelproc->crop.width - 1) * DCMIPP_PIXELPROC_HVRATIO_CONS /
		 (pixelproc->compose.width - 1);
	if (hratio > DCMIPP_PIXELPROC_HVRATIO_MAX)
		hratio = DCMIPP_PIXELPROC_HVRATIO_MAX;
	vratio = (pixelproc->crop.height - 1) * DCMIPP_PIXELPROC_HVRATIO_CONS /
		 (pixelproc->compose.height - 1);
	if (vratio > DCMIPP_PIXELPROC_HVRATIO_MAX)
		vratio = DCMIPP_PIXELPROC_HVRATIO_MAX;
	hdiv = (DCMIPP_PIXELPROC_HVDIV_CONS * pixelproc->compose.width) /
		pixelproc->crop.width;
	if (hdiv > DCMIPP_PIXELPROC_HVDIV_MAX)
		hdiv = DCMIPP_PIXELPROC_HVDIV_MAX;
	vdiv = (DCMIPP_PIXELPROC_HVDIV_CONS * pixelproc->compose.height) /
		pixelproc->crop.height;
	if (vdiv > DCMIPP_PIXELPROC_HVDIV_MAX)
		vdiv = DCMIPP_PIXELPROC_HVDIV_MAX;

	dev_dbg(pixelproc->dev, "%s: downsize config: hratio: 0x%x, vratio: 0x%x, hdiv: 0x%x, vdiv: 0x%x\n",
		pixelproc->sd.name,
		hratio, vratio,
		hdiv, vdiv);

	reg_clear(pixelproc, DCMIPP_PxDSCR(pixelproc->pipe_id),
		  DCMIPP_PxDSCR_ENABLE);
	reg_write(pixelproc, DCMIPP_PxDSRTIOR(pixelproc->pipe_id),
		  (hratio << DCMIPP_PxDSRTIOR_HRATIO_SHIFT) |
		  (vratio << DCMIPP_PxDSRTIOR_VRATIO_SHIFT));
	reg_write(pixelproc, DCMIPP_PxDSSZR(pixelproc->pipe_id),
		  (pixelproc->compose.width << DCMIPP_PxDSSZR_HSIZE_SHIFT) |
		  (pixelproc->compose.height << DCMIPP_PxDSSZR_VSIZE_SHIFT));
	reg_write(pixelproc, DCMIPP_PxDSCR(pixelproc->pipe_id),
		  (hdiv << DCMIPP_PxDSCR_HDIV_SHIFT) |
		  (vdiv << DCMIPP_PxDSCR_VDIV_SHIFT) |
		  DCMIPP_PxDSCR_ENABLE);
}

static int dcmipp_pixelproc_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dcmipp_pixelproc_device *pixelproc = v4l2_get_subdevdata(sd);
	const struct dcmipp_pixelproc_pix_map *vpix;
	int ret = 0;
	unsigned int val;

	mutex_lock(&pixelproc->lock);
	if (enable) {
		/* TODO - need to add framerate control */

		/* Configure cropping */
		reg_write(pixelproc, DCMIPP_PxCRSTR(pixelproc->pipe_id),
			  (pixelproc->crop.top << DCMIPP_PxCRSTR_VSTART_SHIFT) |
			  (pixelproc->crop.left << DCMIPP_PxCRSTR_HSTART_SHIFT));
		reg_write(pixelproc, DCMIPP_PxCRSZR(pixelproc->pipe_id),
			  (pixelproc->crop.width << DCMIPP_PxCRSZR_HSIZE_SHIFT) |
			  (pixelproc->crop.height << DCMIPP_PxCRSZR_VSIZE_SHIFT) |
			  DCMIPP_PxCRSZR_ENABLE);

		/* Configure downsize */
		dcmipp_pixelproc_set_downsize(pixelproc);

		/* Configure YUV Conversion (if applicable) */
		if (pixelproc->pipe_id == 1) {
			ret = dcmipp_pixelproc_colorconv_config(pixelproc);
			if (ret)
				goto out;
		}

		/* Setup the PixelPacker based on the src pad format */
		vpix = dcmipp_pixelproc_pix_map_by_code(pixelproc->src_fmt.code, 1);
		if (!vpix) {
			ret = -EINVAL;
			goto out;
		}

		val = vpix->ppcr_fmt;
		if (vpix->swap_uv)
			val |= DCMIPP_PxPPCR_SWAPRB;

		reg_write(pixelproc, DCMIPP_PxPPCR(pixelproc->pipe_id), val);
	}

out:
	mutex_unlock(&pixelproc->lock);

	return ret;
}

static const struct v4l2_subdev_video_ops dcmipp_pixelproc_video_ops = {
	.s_stream = dcmipp_pixelproc_s_stream,
};

static const struct v4l2_subdev_ops dcmipp_pixelproc_ops = {
	.pad = &dcmipp_pixelproc_pad_ops,
	.video = &dcmipp_pixelproc_video_ops,
};

/* FIXME */
static void dcmipp_pixelproc_release(struct v4l2_subdev *sd)
{
	struct dcmipp_pixelproc_device *pixelproc = v4l2_get_subdevdata(sd);

	kfree(pixelproc);
}

static const struct v4l2_subdev_internal_ops dcmipp_pixelproc_int_ops = {
	.release = dcmipp_pixelproc_release,
};

static void
dcmipp_pixelproc_comp_unbind(struct device *comp, struct device *master,
			     void *master_data)
{
	struct dcmipp_ent_device *ved = dev_get_drvdata(comp);
	struct dcmipp_pixelproc_device *pixelproc =
			container_of(ved, struct dcmipp_pixelproc_device, ved);

	dcmipp_ent_sd_unregister(ved, &pixelproc->sd);
}

static int dcmipp_name_to_pipe_id(const char *name)
{
	if (strstr(name, "main"))
		return 1;
	else if (strstr(name, "aux"))
		return 2;
	else
		return -EINVAL;
}

static int
dcmipp_pixelproc_comp_bind(struct device *comp, struct device *master,
			   void *master_data)
{
	struct dcmipp_bind_data *bind_data = master_data;
	struct dcmipp_platform_data *pdata = comp->platform_data;
	struct dcmipp_pixelproc_device *pixelproc;
	int ret, i;

	/* Allocate the pixelproc struct */
	pixelproc = kzalloc(sizeof(*pixelproc), GFP_KERNEL);
	if (!pixelproc)
		return -ENOMEM;

	pixelproc->regs = bind_data->regs;

	/* Initialize the lock */
	mutex_init(&pixelproc->lock);

	/* Initialize ved and sd */
	ret = dcmipp_ent_sd_register(&pixelproc->ved, &pixelproc->sd,
				     bind_data->v4l2_dev,
				     pdata->entity_name,
				     MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER, 2,
				     (const unsigned long[2]) {
				     MEDIA_PAD_FL_SINK,
				     MEDIA_PAD_FL_SOURCE,
				     },
				     &dcmipp_pixelproc_int_ops, &dcmipp_pixelproc_ops,
				     NULL, NULL);
	if (ret) {
		kfree(pixelproc);
		return ret;
	}

	dev_set_drvdata(comp, &pixelproc->ved);
	pixelproc->dev = comp;

	/* Pipe identifier */
	pixelproc->pipe_id = dcmipp_name_to_pipe_id(pdata->entity_name);
	if (pixelproc->pipe_id != 1 && pixelproc->pipe_id != 2) {
		dev_err(comp, "failed to retrieve pipe_id\n");
		/* TODO - fix error handling */
		return -1;
	}

	/* Initialize the frame format */
	pixelproc->sink_fmt = fmt_sink_default;
	pixelproc->src_fmt = fmt_src_default;
	pixelproc->crop.top = 0;
	pixelproc->crop.left = 0;
	pixelproc->crop.width = DCMIPP_FMT_WIDTH_DEFAULT;
	pixelproc->crop.height = DCMIPP_FMT_HEIGHT_DEFAULT;
	pixelproc->compose.top = 0;
	pixelproc->compose.left = 0;
	pixelproc->compose.width = DCMIPP_FMT_WIDTH_DEFAULT;
	pixelproc->compose.height = DCMIPP_FMT_HEIGHT_DEFAULT;

	/* Initialize controls */
	v4l2_ctrl_handler_init(&pixelproc->ctrls,
			       ARRAY_SIZE(dcmipp_pixelproc_ctrls));

	for (i = 0; i < ARRAY_SIZE(dcmipp_pixelproc_ctrls); i++)
		v4l2_ctrl_new_custom(&pixelproc->ctrls,
				     &dcmipp_pixelproc_ctrls[i], NULL);

	pixelproc->sd.ctrl_handler = &pixelproc->ctrls;
	if (pixelproc->ctrls.error) {
		dev_err(pixelproc->dev, "control initialization error %d\n",
			pixelproc->ctrls.error);
		/* TODO - error handling */
	}

	ret = v4l2_ctrl_handler_setup(&pixelproc->ctrls);
	if (ret < 0) {
		dev_err(pixelproc->dev, "Failed to set up control handlers\n");
		/* TODO - error handling */
	}

	return 0;
}

static const struct component_ops dcmipp_pixelproc_comp_ops = {
	.bind = dcmipp_pixelproc_comp_bind,
	.unbind = dcmipp_pixelproc_comp_unbind,
};

static int dcmipp_pixelproc_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &dcmipp_pixelproc_comp_ops);
}

static int dcmipp_pixelproc_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dcmipp_pixelproc_comp_ops);

	return 0;
}

static const struct platform_device_id dcmipp_pixelproc_driver_ids[] = {
	{
		.name           = DCMIPP_PIXELPROC_DRV_NAME,
	},
	{ }
};

static struct platform_driver dcmipp_pixelproc_pdrv = {
	.probe		= dcmipp_pixelproc_probe,
	.remove		= dcmipp_pixelproc_remove,
	.id_table	= dcmipp_pixelproc_driver_ids,
	.driver		= {
		.name	= DCMIPP_PIXELPROC_DRV_NAME,
	},
};

module_platform_driver(dcmipp_pixelproc_pdrv);

MODULE_DEVICE_TABLE(platform, dcmipp_pixelproc_driver_ids);

MODULE_AUTHOR("Hugues Fruchet <hugues.fruchet@foss.st.com>");
MODULE_AUTHOR("Alain Volmat <alain.volmat@foss.st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 Digital Camera Memory Interface with Pixel Processor driver");
MODULE_LICENSE("GPL");
