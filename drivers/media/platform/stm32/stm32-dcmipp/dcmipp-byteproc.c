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
#include <media/v4l2-rect.h>
#include <media/v4l2-subdev.h>

#include "dcmipp-common.h"

#define DCMIPP_BYTEPROC_DRV_NAME "dcmipp-byteproc"

#define DCMIPP_FMT_WIDTH_DEFAULT  640
#define DCMIPP_FMT_HEIGHT_DEFAULT 480

#define DCMIPP_P0FCTCR (0x500)
#define DCMIPP_P0FCTCR_FRATE_MASK GENMASK(1, 0)
#define DCMIPP_P0SCSTR (0x504)
#define DCMIPP_P0SCSTR_HSTART_SHIFT	0
#define DCMIPP_P0SCSTR_VSTART_SHIFT	16
#define DCMIPP_P0SCSZR (0x508)
#define DCMIPP_P0SCSZR_ENABLE BIT(31)
#define DCMIPP_P0SCSZR_HSIZE_SHIFT	0
#define DCMIPP_P0SCSZR_VSIZE_SHIFT	16
#define DCMIPP_P0PPCR (0x5C0)
#define DCMIPP_P0PPCR_BSM_2_4 0x3
#define DCMIPP_P0PPCR_BSM_MASK GENMASK(8, 7)
#define DCMIPP_P0PPCR_BSM_SHIFT 0x7
#define DCMIPP_P0PPCR_LSM BIT(10)

#define IS_SINK(pad) (!(pad))
#define IS_SRC(pad)  ((pad))
#define PAD_STR(pad) (IS_SRC((pad))) ? "src" : "sink"

#define BYTEPROC_MEDIA_BUS_FMT_DEFAULT MEDIA_BUS_FMT_RGB565_2X8_LE

struct dcmipp_byteproc_pix_map {
	unsigned int code;
	unsigned int bpp;
};

#define PIXMAP_MBUS_BPP(mbus, byteperpixel)	\
		{						\
			.code = MEDIA_BUS_FMT_##mbus,		\
			.bpp = byteperpixel,	\
		}
static const struct dcmipp_byteproc_pix_map dcmipp_byteproc_pix_map_list[] = {
	PIXMAP_MBUS_BPP(RGB565_2X8_LE, 2),
	PIXMAP_MBUS_BPP(YUYV8_2X8, 2),
	PIXMAP_MBUS_BPP(YVYU8_2X8, 2),
	PIXMAP_MBUS_BPP(UYVY8_2X8, 2),
	PIXMAP_MBUS_BPP(VYUY8_2X8, 2),
	PIXMAP_MBUS_BPP(Y8_1X8, 1),
	PIXMAP_MBUS_BPP(SBGGR8_1X8, 1),
	PIXMAP_MBUS_BPP(SGBRG8_1X8, 1),
	PIXMAP_MBUS_BPP(SGRBG8_1X8, 1),
	PIXMAP_MBUS_BPP(SRGGB8_1X8, 1),
	PIXMAP_MBUS_BPP(JPEG_1X8, 1),
};

static const struct dcmipp_byteproc_pix_map *dcmipp_byteproc_pix_map_by_index(unsigned int i)
{
	const struct dcmipp_byteproc_pix_map *l = dcmipp_byteproc_pix_map_list;
	unsigned int size = ARRAY_SIZE(dcmipp_byteproc_pix_map_list);

	if (i >= size)
		return NULL;

	return &l[i];
}

static const struct dcmipp_byteproc_pix_map *dcmipp_byteproc_pix_map_by_code(u32 code)
{
	const struct dcmipp_byteproc_pix_map *l = dcmipp_byteproc_pix_map_list;
	unsigned int size = ARRAY_SIZE(dcmipp_byteproc_pix_map_list);
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (l[i].code == code)
			return &l[i];
	}

	return NULL;
}

struct dcmipp_byteproc_device {
	struct dcmipp_ent_device ved;
	struct v4l2_subdev sd;
	struct device *dev;
	struct v4l2_mbus_framefmt sink_fmt;
	struct v4l2_mbus_framefmt src_fmt;
	bool streaming;
	/* Protect this data structure */
	struct mutex lock;

	void __iomem *regs;

	struct v4l2_fract sink_interval;
	unsigned int frate;
	u32 src_code;
	struct v4l2_rect crop;
	bool do_crop;
};

static const struct v4l2_mbus_framefmt fmt_default = {
	.width = DCMIPP_FMT_WIDTH_DEFAULT,
	.height = DCMIPP_FMT_HEIGHT_DEFAULT,
	.code = BYTEPROC_MEDIA_BUS_FMT_DEFAULT,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_DEFAULT,
};

static const struct v4l2_rect crop_min = {
	.width = DCMIPP_FRAME_MIN_WIDTH,
	.height = DCMIPP_FRAME_MIN_HEIGHT,
	.top = 0,
	.left = 0,
};

static struct v4l2_rect
dcmipp_byteproc_get_crop_bound(const struct v4l2_mbus_framefmt *fmt)
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

static void dcmipp_byteproc_adjust_crop(struct v4l2_rect *r,
					const struct v4l2_mbus_framefmt *fmt)
{
	const struct v4l2_rect src_rect =
		dcmipp_byteproc_get_crop_bound(fmt);

	/* Disallow rectangles smaller than the minimal one. */
	v4l2_rect_set_min_size(r, &crop_min);
	v4l2_rect_map_inside(r, &src_rect);
}

static void dcmipp_byteproc_adjust_src_fmt(struct v4l2_mbus_framefmt *src_fmt,
					   struct v4l2_mbus_framefmt *sink_fmt)
{
	/* Can only divide by 2 in width or height */
	if (src_fmt->width <= sink_fmt->width / 2)
		src_fmt->width = sink_fmt->width / 2;
	else
		src_fmt->width = sink_fmt->width;

	if (src_fmt->height <= sink_fmt->height / 2)
		src_fmt->height = sink_fmt->height / 2;
	else
		src_fmt->height = sink_fmt->height;
}

static void dcmipp_byteproc_adjust_fmt(struct v4l2_mbus_framefmt *fmt, u32 pad)
{
	const struct dcmipp_byteproc_pix_map *vpix;

	/* Only accept code in the pix map table */
	vpix = dcmipp_byteproc_pix_map_by_code(fmt->code);
	if (!vpix)
		fmt->code = fmt_default.code;

	fmt->width = clamp_t(u32, fmt->width, DCMIPP_FRAME_MIN_WIDTH,
			     DCMIPP_FRAME_MAX_WIDTH) & ~1;
	fmt->height = clamp_t(u32, fmt->height, DCMIPP_FRAME_MIN_HEIGHT,
			      DCMIPP_FRAME_MAX_HEIGHT) & ~1;

	if (fmt->field == V4L2_FIELD_ANY || fmt->field == V4L2_FIELD_ALTERNATE)
		fmt->field = fmt_default.field;

	dcmipp_colorimetry_clamp(fmt);
}

static int dcmipp_byteproc_init_cfg(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state)
{
	unsigned int i;

	for (i = 0; i < sd->entity.num_pads; i++) {
		struct v4l2_mbus_framefmt *mf;

		mf = v4l2_subdev_get_try_format(sd, sd_state, i);
		*mf = fmt_default;
	}

	return 0;
}

static int dcmipp_byteproc_enum_mbus_code(struct v4l2_subdev *sd,
					  struct v4l2_subdev_state *sd_state,
					  struct v4l2_subdev_mbus_code_enum *code)
{
	const struct dcmipp_byteproc_pix_map *vpix;

	vpix = dcmipp_byteproc_pix_map_by_index(code->index);
	if (!vpix)
		return -EINVAL;

	code->code = vpix->code;

	return 0;
}

static int dcmipp_byteproc_enum_frame_size(struct v4l2_subdev *sd,
					   struct v4l2_subdev_state *sd_state,
					   struct v4l2_subdev_frame_size_enum *fse)
{
	const struct dcmipp_byteproc_pix_map *vpix;

	if (fse->index)
		return -EINVAL;

	/* Only accept code in the pix map table */
	vpix = dcmipp_byteproc_pix_map_by_code(fse->code);
	if (!vpix)
		return -EINVAL;

	fse->min_width = DCMIPP_FRAME_MIN_WIDTH;
	fse->max_width = DCMIPP_FRAME_MAX_WIDTH;
	fse->min_height = DCMIPP_FRAME_MIN_HEIGHT;
	fse->max_height = DCMIPP_FRAME_MAX_HEIGHT;

	return 0;
}

static int dcmipp_byteproc_get_fmt(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);

	mutex_lock(&byteproc->lock);

	if (IS_SINK(fmt->pad))
		fmt->format = fmt->which == V4L2_SUBDEV_FORMAT_TRY ?
			      *v4l2_subdev_get_try_format(sd, sd_state, 0) :
			      byteproc->sink_fmt;
	else
		fmt->format = fmt->which == V4L2_SUBDEV_FORMAT_TRY ?
			      *v4l2_subdev_get_try_format(sd, sd_state, 0) :
			      byteproc->src_fmt;

	mutex_unlock(&byteproc->lock);

	return 0;
}

static int dcmipp_byteproc_set_fmt(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *pad_fmt;
	int ret = 0;

	mutex_lock(&byteproc->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (byteproc->streaming) {
			ret = -EBUSY;
			goto out;
		}

		if (IS_SINK(fmt->pad))
			pad_fmt = &byteproc->sink_fmt;
		else
			pad_fmt = &byteproc->src_fmt;

	} else {
		pad_fmt = v4l2_subdev_get_try_format(sd, sd_state, 0);
	}

	dcmipp_byteproc_adjust_fmt(&fmt->format, fmt->pad);

	if (IS_SRC(fmt->pad))
		dcmipp_byteproc_adjust_src_fmt(&fmt->format,
					       &byteproc->sink_fmt);

	dev_dbg(byteproc->dev, "%s: %s format update: old:%dx%d (0x%x, %d, %d, %d, %d) new:%dx%d (0x%x, %d, %d, %d, %d)\n",
		byteproc->sd.name,
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

out:
	mutex_unlock(&byteproc->lock);

	return ret;
}

static int dcmipp_byteproc_get_selection(struct v4l2_subdev *sd,
					 struct v4l2_subdev_state *sd_state,
					 struct v4l2_subdev_selection *s)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_rect *crop;

	if (IS_SINK(s->pad))
		return -EINVAL;

	if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sink_fmt = &byteproc->sink_fmt;
		crop = &byteproc->crop;
	} else {
		sink_fmt = v4l2_subdev_get_try_format(sd, sd_state, 0);
		crop = v4l2_subdev_get_try_crop(sd, sd_state, 0);
	}

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		s->r = *crop;
		break;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r = dcmipp_byteproc_get_crop_bound(sink_fmt);
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
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_rect *crop;
	bool _do_crop;
	bool *do_crop;

	if (IS_SINK(s->pad))
		return -EINVAL;

	if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sink_fmt = &byteproc->sink_fmt;
		crop = &byteproc->crop;
		do_crop = &byteproc->do_crop;
	} else {
		sink_fmt = v4l2_subdev_get_try_format(sd, sd_state, 0);
		crop = v4l2_subdev_get_try_crop(sd, sd_state, 0);
		do_crop = &_do_crop;
	}

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		dcmipp_byteproc_adjust_crop(&s->r, sink_fmt);

		*crop = s->r;
		*do_crop = true;

		dev_dbg(byteproc->dev, "s_selection: crop %ux%u@(%u,%u)\n",
			crop->width, crop->height, crop->left, crop->top);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_subdev_pad_ops dcmipp_byteproc_pad_ops = {
	.init_cfg		= dcmipp_byteproc_init_cfg,
	.enum_mbus_code		= dcmipp_byteproc_enum_mbus_code,
	.enum_frame_size	= dcmipp_byteproc_enum_frame_size,
	.get_fmt		= dcmipp_byteproc_get_fmt,
	.set_fmt		= dcmipp_byteproc_set_fmt,
	.get_selection		= dcmipp_byteproc_get_selection,
	.set_selection		= dcmipp_byteproc_set_selection,
};

static int dcmipp_byteproc_configure_scale_crop
			(struct dcmipp_byteproc_device *byteproc)
{
	const struct dcmipp_byteproc_pix_map *vpix;
	u32 hprediv, vprediv;
	struct v4l2_rect crop;
	bool do_crop = false;
	u32 val = 0;

	/* find output format bpp */
	vpix = dcmipp_byteproc_pix_map_by_code(byteproc->src_fmt.code);
	if (!vpix)
		return -EINVAL;

	/* clear decimation/crop */
	reg_clear(byteproc, DCMIPP_P0PPCR, DCMIPP_P0PPCR_BSM_MASK);
	reg_clear(byteproc, DCMIPP_P0PPCR, DCMIPP_P0PPCR_LSM);
	reg_write(byteproc, DCMIPP_P0SCSTR, 0);
	reg_write(byteproc, DCMIPP_P0SCSZR, 0);

	if (vpix->code == MEDIA_BUS_FMT_JPEG_1X8)
		/* Ignore scale/crop with JPEG */
		return 0;

	/* decimation */
	hprediv = byteproc->sink_fmt.width /
		  byteproc->src_fmt.width;
	if (hprediv == 2)
		val |= DCMIPP_P0PPCR_BSM_2_4 << DCMIPP_P0PPCR_BSM_SHIFT;

	vprediv = byteproc->sink_fmt.height /
		  byteproc->src_fmt.height;
	if (vprediv == 2)
		val |= DCMIPP_P0PPCR_LSM;/* one line out of two */

	if (val) {
		/* decimate using bytes and lines skipping */
		reg_set(byteproc, DCMIPP_P0PPCR, val);

		/* crop to decimated resolution */
		crop.top = 0;
		crop.left = 0;
		crop.width = byteproc->src_fmt.width;
		crop.height = byteproc->src_fmt.height;
		do_crop = true;

		dev_dbg(byteproc->dev, "decimate to %dx%d [prediv=%dx%d]\n",
			crop.width, crop.height, hprediv, vprediv);
	}

	if (byteproc->do_crop) {
		/* explicit crop superseed default crop */
		crop = byteproc->crop;
		do_crop = true;
	}

	if (do_crop) {
		dev_dbg(byteproc->dev, "crop to %dx%d\n",
			crop.width, crop.height);

		/* expressed in 32-bits words on X axis, lines on Y axis */
		reg_write(byteproc, DCMIPP_P0SCSTR,
			  (((crop.left * vpix->bpp) / 4) << DCMIPP_P0SCSTR_HSTART_SHIFT) |
			  (crop.top << DCMIPP_P0SCSTR_VSTART_SHIFT));
		reg_write(byteproc, DCMIPP_P0SCSZR,
			  DCMIPP_P0SCSZR_ENABLE |
			  (((crop.width * vpix->bpp) / 4) << DCMIPP_P0SCSZR_HSIZE_SHIFT) |
			  (crop.height << DCMIPP_P0SCSZR_VSIZE_SHIFT));
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

	if (IS_SINK(fi->pad)) {
		fi->interval = byteproc->sink_interval;
	} else {
		fi->interval.numerator = byteproc->sink_interval.numerator;
		fi->interval.denominator =
			byteproc->sink_interval.denominator /
			(1 << byteproc->frate);
	}

	return 0;
}

static int dcmipp_byteproc_s_frame_interval(struct v4l2_subdev *sd,
					    struct v4l2_subdev_frame_interval *fi)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);

	mutex_lock(&byteproc->lock);

	if (byteproc->streaming) {
		mutex_unlock(&byteproc->lock);
		return -EBUSY;
	}

	if (IS_SINK(fi->pad)) {
		byteproc->sink_interval = fi->interval;
	} else {
		if (fi->interval.denominator >=
		    byteproc->sink_interval.denominator) {
			byteproc->frate = 0;
		} else {
			if (fi->interval.denominator <=
			   (byteproc->sink_interval.denominator / 8))
				byteproc->frate = 3;
			else if (fi->interval.denominator <=
			   (byteproc->sink_interval.denominator / 4))
				byteproc->frate = 2;
			else if (fi->interval.denominator <=
			   (byteproc->sink_interval.denominator / 2))
				byteproc->frate = 1;
		}
	}

	mutex_unlock(&byteproc->lock);

	return 0;
}

#define STOP_TIMEOUT_US 1000
#define POLL_INTERVAL_US  50
static int dcmipp_byteproc_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&byteproc->lock);
	if (enable) {
		/* Byteproc subdev do not support different format at sink/src */
		if (byteproc->sink_fmt.code != byteproc->src_fmt.code) {
			dev_err(byteproc->dev, "Sink & Src format differ\n");
			ret = -EPIPE;
			goto err;
		}

		dcmipp_byteproc_configure_framerate(byteproc);

		ret = dcmipp_byteproc_configure_scale_crop(byteproc);
		if (ret)
			goto err;
	}

err:
	mutex_unlock(&byteproc->lock);

	return ret;
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

/* FIXME */
static void dcmipp_byteproc_release(struct v4l2_subdev *sd)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);

	kfree(byteproc);
}

static const struct v4l2_subdev_internal_ops dcmipp_byteproc_int_ops = {
	.release = dcmipp_byteproc_release,
};

static void dcmipp_byteproc_comp_unbind(struct device *comp,
					struct device *master,
					void *master_data)
{
	struct dcmipp_ent_device *ved = dev_get_drvdata(comp);
	struct dcmipp_byteproc_device *byteproc =
			container_of(ved, struct dcmipp_byteproc_device, ved);

	dcmipp_ent_sd_unregister(ved, &byteproc->sd);
}

static int dcmipp_byteproc_comp_bind(struct device *comp, struct device *master,
				     void *master_data)
{
	struct dcmipp_bind_data *bind_data = master_data;
	struct dcmipp_platform_data *pdata = comp->platform_data;
	struct dcmipp_byteproc_device *byteproc;
	int ret;

	/* Allocate the byteproc struct */
	byteproc = kzalloc(sizeof(*byteproc), GFP_KERNEL);
	if (!byteproc)
		return -ENOMEM;

	byteproc->regs = bind_data->regs;

	/* Initialize the lock */
	mutex_init(&byteproc->lock);

	/* Initialize ved and sd */
	ret = dcmipp_ent_sd_register(&byteproc->ved, &byteproc->sd,
				     bind_data->v4l2_dev,
				     pdata->entity_name,
				     MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER, 2,
				     (const unsigned long[2]) {
				     MEDIA_PAD_FL_SINK,
				     MEDIA_PAD_FL_SOURCE,
				     },
				     &dcmipp_byteproc_int_ops,
				     &dcmipp_byteproc_ops,
				     NULL, NULL);
	if (ret) {
		kfree(byteproc);
		return ret;
	}

	dev_set_drvdata(comp, &byteproc->ved);
	byteproc->dev = comp;

	/* Initialize the frame format */
	byteproc->sink_fmt = fmt_default;
	byteproc->src_fmt = fmt_default;

	return 0;
}

static const struct component_ops dcmipp_byteproc_comp_ops = {
	.bind = dcmipp_byteproc_comp_bind,
	.unbind = dcmipp_byteproc_comp_unbind,
};

static int dcmipp_byteproc_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &dcmipp_byteproc_comp_ops);
}

static int dcmipp_byteproc_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dcmipp_byteproc_comp_ops);

	return 0;
}

static const struct platform_device_id dcmipp_byteproc_driver_ids[] = {
	{
		.name           = DCMIPP_BYTEPROC_DRV_NAME,
	},
	{ }
};

static struct platform_driver dcmipp_byteproc_pdrv = {
	.probe		= dcmipp_byteproc_probe,
	.remove		= dcmipp_byteproc_remove,
	.id_table	= dcmipp_byteproc_driver_ids,
	.driver		= {
		.name	= DCMIPP_BYTEPROC_DRV_NAME,
	},
};

module_platform_driver(dcmipp_byteproc_pdrv);

MODULE_DEVICE_TABLE(platform, dcmipp_byteproc_driver_ids);

MODULE_AUTHOR("Hugues Fruchet <hugues.fruchet@foss.st.com>");
MODULE_AUTHOR("Alain Volmat <alain.volmat@foss.st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 Digital Camera Memory Interface with Pixel Processor driver");
MODULE_LICENSE("GPL");
