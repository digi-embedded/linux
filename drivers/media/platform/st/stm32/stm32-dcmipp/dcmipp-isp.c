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
#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-rect.h>
#include <media/v4l2-subdev.h>

#include "dcmipp-common.h"
#include "dcmipp-colorconv.h"

#define DCMIPP_ISP_DRV_NAME "dcmipp-isp"

#define DCMIPP_FMT_WIDTH_DEFAULT  640
#define DCMIPP_FMT_HEIGHT_DEFAULT 480

#define DCMIPP_P1FSCR (0x804)
#define DCMIPP_P1FSCR_DTIDA_MASK GENMASK(5, 0)
#define DCMIPP_P1FSCR_DTIDA_SHIFT 0
#define DCMIPP_P1FSCR_PIPEDIFF BIT(18)
#define DCMIPP_P1SRCR (0x820)
#define DCMIPP_P1SRCR_LASTLINE_SHIFT 0
#define DCMIPP_P1SRCR_LASTLINE_MASK GENMASK(11, 0)
#define DCMIPP_P1SRCR_FIRSTLINEDEL_SHIFT 12
#define DCMIPP_P1SRCR_FIRSTLINEDEL_MASK GENMASK(14, 12)
#define DCMIPP_P1SRCR_CROPEN BIT(15)

#define DCMIPP_P1BPRCR (0x824)
#define DCMIPP_P1BPRCR_ENABLE BIT(0)
#define DCMIPP_P1BPRCR_STRENGTH_SHIFT 1
#define DCMIPP_P1BPRCR_STRENGTH_MASK GENMASK(3, 1)

#define DCMIPP_P1BPRSR (0x828)
#define DCMIPP_P1BPRSR_BADCNT_MASK GENMASK(11, 0)

#define DCMIPP_P1DECR (0x830)
#define DCMIPP_P1DECR_ENABLE BIT(0)
#define DCMIPP_P1DECR_HDEC_SHIFT 1
#define DCMIPP_P1DECR_HDEC_MASK GENMASK(2, 1)
#define DCMIPP_P1DECR_VDEC_SHIFT 3
#define DCMIPP_P1DECR_VDEC_MASK GENMASK(4, 3)

#define DCMIPP_P1BLCCR (0x840)
#define DCMIPP_P1BLCCR_ENABLE BIT(0)
#define DCMIPP_P1BLCCR_BLCB_SHIFT 8
#define DCMIPP_P1BLCCR_BLCB_MASK GENMASK(15, 8)
#define DCMIPP_P1BLCCR_BLCG_SHIFT 16
#define DCMIPP_P1BLCCR_BLCG_MASK GENMASK(23, 16)
#define DCMIPP_P1BLCCR_BLCR_SHIFT 24
#define DCMIPP_P1BLCCR_BLCR_MASK GENMASK(31, 24)

#define DCMIPP_P1DMCR (0x870)
#define DCMIPP_P1DMCR_ENABLE BIT(0)
#define DCMIPP_P1DMCR_TYPE_SHIFT 1
#define DCMIPP_P1DMCR_TYPE_MASK GENMASK(2, 1)
#define DCMIPP_P1DMCR_TYPE_RGGB 0x0
#define DCMIPP_P1DMCR_TYPE_GRBG 0x1
#define DCMIPP_P1DMCR_TYPE_GBRG 0x2
#define DCMIPP_P1DMCR_TYPE_BGGR 0x3
#define DCMIPP_P1DMCR_PEAK_SHIFT 16
#define DCMIPP_P1DMCR_PEAK_MASK GENMASK(19, 16)
#define DCMIPP_P1DMCR_LINEV_SHIFT 20
#define DCMIPP_P1DMCR_LINEV_MASK GENMASK(23, 20)
#define DCMIPP_P1DMCR_LINEH_SHIFT 24
#define DCMIPP_P1DMCR_LINEH_MASK GENMASK(27, 24)
#define DCMIPP_P1DMCR_EDGE_SHIFT 28
#define DCMIPP_P1DMCR_EDGE_MASK GENMASK(31, 28)

#define DCMIPP_P1CCCR (0x880)
#define DCMIPP_P1CCCR_ENABLE BIT(0)
#define DCMIPP_P1CCCR_TYPE_YUV 0
#define DCMIPP_P1CCCR_TYPE_RGB BIT(1)
#define DCMIPP_P1CCCR_CLAMP BIT(2)
#define DCMIPP_P1CCRR1 (0x884)
#define DCMIPP_P1CCRR2 (0x888)
#define DCMIPP_P1CCGR1 (0x88C)
#define DCMIPP_P1CCGR2 (0x890)
#define DCMIPP_P1CCBR1 (0x894)
#define DCMIPP_P1CCBR2 (0x898)

#define DCMIPP_P1CRSZR (0x908)
#define DCMIPP_P1CRSZR_HSIZE_SHIFT 0
#define DCMIPP_P1CRSZR_HSIZE_MASK GENMASK(11, 0)
#define DCMIPP_P1CRSZR_VSIZE_SHIFT 16
#define DCMIPP_P1CRSZR_VSIZE_MASK GENMASK(27, 16)
#define DCMIPP_P1CRSZR_ENABLE BIT(31)

#define DCMIPP_P1DSCR (0x910)
#define DCMIPP_P1DSCR_HDIV_SHIFT 0
#define DCMIPP_P1DSCR_HDIV_MASK GENMASK(9, 0)
#define DCMIPP_P1DSCR_VDIV_SHIFT 16
#define DCMIPP_P1DSCR_VDIV_MASK GENMASK(25, 16)
#define DCMIPP_P1DSCR_ENABLE BIT(31)

#define IS_SINK(pad) (!(pad))
#define IS_SRC(pad)  ((pad))
#define PAD_STR(pad) (IS_SRC((pad))) ? "src" : "sink"

#define ISP_MEDIA_BUS_SINK_FMT_DEFAULT MEDIA_BUS_FMT_RGB565_2X8_LE
#define ISP_MEDIA_BUS_SRC_FMT_DEFAULT MEDIA_BUS_FMT_RGB888_1X24

struct dcmipp_isp_pix_map {
	unsigned int code;
	unsigned int dt;
};

#define PIXMAP_MBUS(mbus, datatype) \
		{						\
			.code = MEDIA_BUS_FMT_##mbus,		\
			.dt = datatype,		\
		}
static const struct dcmipp_isp_pix_map dcmipp_isp_sink_pix_map_list[] = {
	/* RGB565 */
	PIXMAP_MBUS(RGB565_2X8_LE, MIPI_CSI2_DT_RGB565),
	/* YUV422 */
	PIXMAP_MBUS(YUYV8_2X8, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_MBUS(UYVY8_2X8, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_MBUS(YVYU8_2X8, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_MBUS(VYUY8_2X8, MIPI_CSI2_DT_YUV422_8B),
	/* GREY */
	PIXMAP_MBUS(Y8_1X8, 0x00), /* TODO - DT value to be fixed */
	/* Raw Bayer */
	/* Raw 8 */
	PIXMAP_MBUS(SBGGR8_1X8, MIPI_CSI2_DT_RAW8),
	PIXMAP_MBUS(SGBRG8_1X8, MIPI_CSI2_DT_RAW8),
	PIXMAP_MBUS(SGRBG8_1X8, MIPI_CSI2_DT_RAW8),
	PIXMAP_MBUS(SRGGB8_1X8, MIPI_CSI2_DT_RAW8),
	/* Raw 10 */
	PIXMAP_MBUS(SBGGR10_1X10, MIPI_CSI2_DT_RAW10),
	PIXMAP_MBUS(SGBRG10_1X10, MIPI_CSI2_DT_RAW10),
	PIXMAP_MBUS(SGRBG10_1X10, MIPI_CSI2_DT_RAW10),
	PIXMAP_MBUS(SRGGB10_1X10, MIPI_CSI2_DT_RAW10),
	/* Raw 12 */
	PIXMAP_MBUS(SBGGR12_1X12, MIPI_CSI2_DT_RAW12),
	PIXMAP_MBUS(SGBRG12_1X12, MIPI_CSI2_DT_RAW12),
	PIXMAP_MBUS(SGRBG12_1X12, MIPI_CSI2_DT_RAW12),
	PIXMAP_MBUS(SRGGB12_1X12, MIPI_CSI2_DT_RAW12),
	/* Raw 14 */
	PIXMAP_MBUS(SBGGR14_1X14, MIPI_CSI2_DT_RAW14),
	PIXMAP_MBUS(SGBRG14_1X14, MIPI_CSI2_DT_RAW14),
	PIXMAP_MBUS(SGRBG14_1X14, MIPI_CSI2_DT_RAW14),
	PIXMAP_MBUS(SRGGB14_1X14, MIPI_CSI2_DT_RAW14),
};

static const struct dcmipp_isp_pix_map dcmipp_isp_src_pix_map_list[] = {
	PIXMAP_MBUS(RGB888_1X24, 0),
	PIXMAP_MBUS(YUV8_1X24, 0),
};

static const struct dcmipp_isp_pix_map *
dcmipp_isp_pix_map_by_index(unsigned int i, unsigned int pad)
{
	const struct dcmipp_isp_pix_map *l;
	unsigned int size;

	if (IS_SRC(pad)) {
		l = dcmipp_isp_src_pix_map_list;
		size = ARRAY_SIZE(dcmipp_isp_src_pix_map_list);
	} else {
		l = dcmipp_isp_sink_pix_map_list;
		size = ARRAY_SIZE(dcmipp_isp_sink_pix_map_list);
	}

	if (i >= size)
		return NULL;

	return &l[i];
}

static const struct dcmipp_isp_pix_map *
dcmipp_isp_pix_map_by_code(u32 code, unsigned int pad)
{
	const struct dcmipp_isp_pix_map *l;
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
		if (l[i].code == code)
			return &l[i];
	}

	return NULL;
}

struct dcmipp_isp_device {
	struct dcmipp_ent_device ved;
	struct v4l2_subdev sd;
	struct device *dev;
	struct v4l2_mbus_framefmt sink_fmt;
	struct v4l2_mbus_framefmt src_fmt;
	unsigned int decimation;
	struct v4l2_rect crop;
	struct v4l2_rect compose;
	bool streaming;
	/* Protect this data structure */
	struct mutex lock;

	void __iomem *regs;

	struct v4l2_ctrl_handler ctrls;
};

static const struct v4l2_mbus_framefmt fmt_sink_default = {
	.width = DCMIPP_FMT_WIDTH_DEFAULT,
	.height = DCMIPP_FMT_HEIGHT_DEFAULT,
	.code = ISP_MEDIA_BUS_SINK_FMT_DEFAULT,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_DEFAULT,
};

static const struct v4l2_mbus_framefmt fmt_src_default = {
	.width = DCMIPP_FMT_WIDTH_DEFAULT,
	.height = DCMIPP_FMT_HEIGHT_DEFAULT,
	.code = ISP_MEDIA_BUS_SRC_FMT_DEFAULT,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_DEFAULT,
};

/*
 * Functions handling controls
 */
#define V4L2_CID_ISP_DEMOSAICING_PEAK	(V4L2_CID_USER_BASE | 0x1001)
#define V4L2_CID_ISP_DEMOSAICING_LINEH	(V4L2_CID_USER_BASE | 0x1002)
#define V4L2_CID_ISP_DEMOSAICING_LINEV	(V4L2_CID_USER_BASE | 0x1003)
#define V4L2_CID_ISP_DEMOSAICING_EDGE	(V4L2_CID_USER_BASE | 0x1004)
#define V4L2_CID_ISP_BLACKLEVEL		(V4L2_CID_USER_BASE | 0x1005)
#define V4L2_CID_ISP_BAD_PIXEL		(V4L2_CID_USER_BASE | 0x1006)
#define V4L2_CID_ISP_BAD_PIXEL_COUNT	(V4L2_CID_USER_BASE | 0x1007)

static int dcmipp_isp_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct dcmipp_isp_device *isp =
			container_of(ctrl->handler, struct dcmipp_isp_device, ctrls);

	if (!isp->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ISP_DEMOSAICING_PEAK:
		reg_clear(isp, DCMIPP_P1DMCR, DCMIPP_P1DMCR_PEAK_MASK);
		reg_set(isp, DCMIPP_P1DMCR, ctrl->val << DCMIPP_P1DMCR_PEAK_SHIFT);
		break;
	case V4L2_CID_ISP_DEMOSAICING_LINEH:
		reg_clear(isp, DCMIPP_P1DMCR, DCMIPP_P1DMCR_LINEH_MASK);
		reg_set(isp, DCMIPP_P1DMCR, ctrl->val << DCMIPP_P1DMCR_LINEH_SHIFT);
		break;
	case V4L2_CID_ISP_DEMOSAICING_LINEV:
		reg_clear(isp, DCMIPP_P1DMCR, DCMIPP_P1DMCR_LINEV_MASK);
		reg_set(isp, DCMIPP_P1DMCR, ctrl->val << DCMIPP_P1DMCR_LINEV_SHIFT);
		break;
	case V4L2_CID_ISP_DEMOSAICING_EDGE:
		reg_clear(isp, DCMIPP_P1DMCR, DCMIPP_P1DMCR_EDGE_MASK);
		reg_set(isp, DCMIPP_P1DMCR, ctrl->val << DCMIPP_P1DMCR_EDGE_SHIFT);
		break;
	case V4L2_CID_ISP_BLACKLEVEL:
		reg_write(isp, DCMIPP_P1BLCCR, ctrl->val ?
			  ((ctrl->val << DCMIPP_P1BLCCR_BLCB_SHIFT) |
			   (ctrl->val << DCMIPP_P1BLCCR_BLCG_SHIFT) |
			   (ctrl->val << DCMIPP_P1BLCCR_BLCR_SHIFT) |
			   DCMIPP_P1BLCCR_ENABLE) : 0);
		break;
	case V4L2_CID_ISP_BAD_PIXEL:
		reg_write(isp, DCMIPP_P1BPRCR, ctrl->val ?
			   ((ctrl->val - 1) << DCMIPP_P1BPRCR_STRENGTH_SHIFT) |
			   DCMIPP_P1BPRCR_ENABLE : 0);
		break;
	}

	return 0;
};

static int dcmipp_isp_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct dcmipp_isp_device *isp =
			container_of(ctrl->handler, struct dcmipp_isp_device, ctrls);
	int ret = 0;

	if (!isp->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ISP_BAD_PIXEL_COUNT:
		ctrl->val = reg_read(isp, DCMIPP_P1BPRSR) &
				     DCMIPP_P1BPRSR_BADCNT_MASK;
		break;
	}

	return ret;
};

static const struct v4l2_ctrl_ops dcmipp_isp_ctrl_ops = {
	.s_ctrl = dcmipp_isp_s_ctrl,
};

static const struct v4l2_ctrl_ops dcmipp_isp_get_ctrl_ops = {
	.g_volatile_ctrl = dcmipp_isp_g_ctrl,
};

static const struct v4l2_ctrl_config dcmipp_isp_ctrls[] = {
	{
		.ops		= &dcmipp_isp_ctrl_ops,
		.id		= V4L2_CID_ISP_DEMOSAICING_PEAK,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Demosaicing Peak",
		.min		= 0,
		.max		= 7,
		.step		= 1,
		.def		= 0,
		.flags		= 0,
	}, {
		.ops		= &dcmipp_isp_ctrl_ops,
		.id		= V4L2_CID_ISP_DEMOSAICING_LINEH,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Demosaicing Horizontal Line",
		.min		= 0,
		.max		= 7,
		.step		= 1,
		.def		= 0,
		.flags		= 0,
	}, {
		.ops		= &dcmipp_isp_ctrl_ops,
		.id		= V4L2_CID_ISP_DEMOSAICING_LINEV,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Demosaicing Vertical Line",
		.min		= 0,
		.max		= 7,
		.step		= 1,
		.def		= 0,
		.flags		= 0,
	}, {
		.ops		= &dcmipp_isp_ctrl_ops,
		.id		= V4L2_CID_ISP_DEMOSAICING_EDGE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Demosaicing Edge",
		.min		= 0,
		.max		= 7,
		.step		= 1,
		.def		= 0,
		.flags		= 0,
	}, {
		.ops		= &dcmipp_isp_ctrl_ops,
		.id		= V4L2_CID_ISP_BLACKLEVEL,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "BlackLevel (RGB)",
		.min		= 0,
		.max		= 255,
		.step		= 1,
		.def		= 0,
		.flags		= 0,
	}, {
		.ops		= &dcmipp_isp_ctrl_ops,
		.id		= V4L2_CID_ISP_BAD_PIXEL,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Bad Pixel Control",
		.min		= 0,
		.max		= 8,
		.step		= 1,
		.def		= 0,
		.flags		= 0,
	}, {
		.ops		= &dcmipp_isp_get_ctrl_ops,
		.id		= V4L2_CID_ISP_BAD_PIXEL_COUNT,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Bad Pixel Count",
		.min		= 0,
		.max		= 4095,
		.step		= 1,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	}
};

static inline unsigned int dcmipp_isp_compute_decimation(unsigned int orig,
							 unsigned int req)
{
	unsigned int i;

	for (i = 0; i < 4; i++) {
		if (req >= (orig / (1 << i)))
			return i;
	}

	return (i - 1);
}

static void dcmipp_isp_adjust_fmt(struct v4l2_mbus_framefmt *fmt, u32 pad)
{
	const struct dcmipp_isp_pix_map *vpix;

	/* Only accept code in the pix map table */
	vpix = dcmipp_isp_pix_map_by_code(fmt->code, pad);
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

static int dcmipp_isp_init_cfg(struct v4l2_subdev *sd,
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

static int dcmipp_isp_enum_mbus_code(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_mbus_code_enum *code)
{
	const struct dcmipp_isp_pix_map *vpix;

	vpix = dcmipp_isp_pix_map_by_index(code->index, code->pad);
	if (!vpix)
		return -EINVAL;

	code->code = vpix->code;

	return 0;
}

static int dcmipp_isp_enum_frame_size(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_frame_size_enum *fse)
{
	const struct dcmipp_isp_pix_map *vpix;

	if (fse->index)
		return -EINVAL;

	/* Only accept code in the pix map table */
	vpix = dcmipp_isp_pix_map_by_code(fse->code, fse->pad);
	if (!vpix)
		return -EINVAL;

	fse->min_width = DCMIPP_FRAME_MIN_WIDTH;
	fse->max_width = DCMIPP_FRAME_MAX_WIDTH;
	fse->min_height = DCMIPP_FRAME_MIN_HEIGHT;
	fse->max_height = DCMIPP_FRAME_MAX_HEIGHT;

	return 0;
}

static int dcmipp_isp_get_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *fmt)
{
	struct dcmipp_isp_device *isp = v4l2_get_subdevdata(sd);

	mutex_lock(&isp->lock);

	if (IS_SINK(fmt->pad))
		fmt->format = fmt->which == V4L2_SUBDEV_FORMAT_TRY ?
			      *v4l2_subdev_get_try_format(sd, state, 0) :
			      isp->sink_fmt;
	else
		fmt->format = fmt->which == V4L2_SUBDEV_FORMAT_TRY ?
			      *v4l2_subdev_get_try_format(sd, state, 0) :
			      isp->src_fmt;

	mutex_unlock(&isp->lock);

	return 0;
}

#define dcmipp_isp_is_yuv_fmt(a) ({					      \
	typeof(a) __a = (a);						      \
	((__a) >= MEDIA_BUS_FMT_Y8_1X8 && (__a) < MEDIA_BUS_FMT_SBGGR8_1X8) ? \
	 true : false; })
static int dcmipp_isp_set_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *fmt)
{
	struct dcmipp_isp_device *isp = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *pad_fmt;
	int ret = 0;

	mutex_lock(&isp->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (isp->streaming) {
			ret = -EBUSY;
			goto out;
		}

		if (IS_SINK(fmt->pad))
			pad_fmt = &isp->sink_fmt;
		else
			pad_fmt = &isp->src_fmt;

	} else {
		pad_fmt = v4l2_subdev_get_try_format(sd, state, 0);
	}

	dcmipp_isp_adjust_fmt(&fmt->format, fmt->pad);

	if (IS_SINK(fmt->pad) && fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		/* When setting sink format, we have to update the src format */
		isp->src_fmt = fmt->format;
		if (dcmipp_isp_is_yuv_fmt(fmt->format.code))
			isp->src_fmt.code = MEDIA_BUS_FMT_YUV8_1X24;
		else
			isp->src_fmt.code = MEDIA_BUS_FMT_RGB888_1X24;
	}
	/* TODO - we need to update the try src format as well */

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

	/* Update sink pad crop */
	if (IS_SINK(fmt->pad) && fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		isp->crop.top = 0;
		isp->crop.left = 0;
		isp->crop.width = fmt->format.width;
		isp->crop.height = fmt->format.height;

		isp->compose.top = 0;
		isp->compose.left = 0;
		isp->compose.width = fmt->format.width;
		isp->compose.height = fmt->format.height;
		isp->decimation = 0;
	}

out:
	mutex_unlock(&isp->lock);

	return ret;
}

static struct v4l2_rect
dcmipp_isp_get_crop_bound(const struct v4l2_mbus_framefmt *fmt)
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

static void dcmipp_isp_adjust_crop(struct v4l2_rect *r,
				   const struct v4l2_mbus_framefmt *fmt,
				   unsigned int fmt_width)
{
	const struct v4l2_rect src_rect =
		dcmipp_isp_get_crop_bound(fmt);
	static struct v4l2_rect crop_min = {
		.width = DCMIPP_FMT_WIDTH_DEFAULT,
		.height = 1,
		.top = 8,
		.left = 0,
	};

	/* Disallow rectangles smaller than the minimal one. */
	crop_min.width = fmt_width;
	v4l2_rect_set_min_size(r, &crop_min);
	v4l2_rect_map_inside(r, &src_rect);
}

static int dcmipp_isp_get_selection(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_selection *s)
{
	struct dcmipp_isp_device *isp = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_rect *crop, *compose;

	if (IS_SRC(s->pad))
		return -EINVAL;

	if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sink_fmt = &isp->sink_fmt;
		crop = &isp->crop;
		compose = &isp->compose;
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
		s->r = dcmipp_isp_get_crop_bound(sink_fmt);
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

static int dcmipp_isp_set_selection(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_selection *s)
{
	struct dcmipp_isp_device *isp = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_rect *crop, *compose;
	unsigned int dec;

	if (IS_SRC(s->pad))
		return -EINVAL;

	if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sink_fmt = &isp->sink_fmt;
		crop = &isp->crop;
		compose = &isp->compose;
	} else {
		sink_fmt = v4l2_subdev_get_try_format(sd, state, s->pad);
		crop = v4l2_subdev_get_try_crop(sd, state, s->pad);
		compose = v4l2_subdev_get_try_compose(sd, state, s->pad);
	}

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		dcmipp_isp_adjust_crop(&s->r, sink_fmt, isp->sink_fmt.width);

		*crop = s->r;

		/* When we set the crop, this impact as well the compose */
		*compose = s->r;
		isp->decimation = 0;

		/*
		 * In case of setting the crop with ACTIVE set, we need to
		 * update the source pad size
		 */
		if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
			isp->src_fmt.width = s->r.width;
			isp->src_fmt.height = s->r.height;
		}

		/* TODO - when not in format active, we should also update the
		 * try src pad format
		 */

		dev_dbg(isp->dev, "s_selection: crop %ux%u@(%u,%u)\n",
			crop->width, crop->height, crop->left, crop->top);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		s->r.top = 0;
		s->r.left = 0;
		if (s->r.width > crop->width)
			s->r.width = crop->width;
		if (s->r.height > crop->height)
			s->r.height = crop->height;
		dec = dcmipp_isp_compute_decimation(crop->width, s->r.width);
		s->r.width = crop->width / (1 << dec);
		if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
			isp->decimation = 0;
			isp->decimation |= (dec << DCMIPP_P1DECR_HDEC_SHIFT);
		}
		dec = dcmipp_isp_compute_decimation(crop->height, s->r.height);
		s->r.height = crop->height / (1 << dec);
		if (crop->height % (1 << dec))
			s->r.height += 1;
		if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
			isp->decimation |= (dec << DCMIPP_P1DECR_VDEC_SHIFT);
			if (isp->decimation)
				isp->decimation |= DCMIPP_P1DECR_ENABLE;
		}

		*compose = s->r;

		/*
		 * In case of setting the compose with ACTIVE set, we need to
		 * update the source pad size
		 */
		if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
			isp->src_fmt.width = s->r.width;
			isp->src_fmt.height = s->r.height;
		}
		/* TODO - when not in format active, we should also update the
		 * try src pad format
		 */

		dev_dbg(isp->dev, "s_selection: compose %ux%u@(%u,%u)\n",
			s->r.width, s->r.height, s->r.left, s->r.top);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_subdev_pad_ops dcmipp_isp_pad_ops = {
	.init_cfg		= dcmipp_isp_init_cfg,
	.enum_mbus_code		= dcmipp_isp_enum_mbus_code,
	.enum_frame_size	= dcmipp_isp_enum_frame_size,
	.get_fmt		= dcmipp_isp_get_fmt,
	.set_fmt		= dcmipp_isp_set_fmt,
	.get_selection		= dcmipp_isp_get_selection,
	.set_selection		= dcmipp_isp_set_selection,
};

static void dcmipp_isp_config_demosaicing(struct dcmipp_isp_device *isp)
{
	const struct dcmipp_isp_pix_map *vpix =
		dcmipp_isp_pix_map_by_code(isp->sink_fmt.code, 0);
	unsigned int val = 0;

	/* Disable demosaicing */
	reg_clear(isp, DCMIPP_P1DMCR, DCMIPP_P1DMCR_ENABLE | DCMIPP_P1DMCR_TYPE_MASK);

	if (vpix->code >= 0x3000 && vpix->code < 0x4000) {
		dev_info(isp->dev, "Input is RawBayer, enable Demosaicing\n");

		if (vpix->code == MEDIA_BUS_FMT_SBGGR8_1X8 ||
		    vpix->code == MEDIA_BUS_FMT_SBGGR10_1X10 ||
		    vpix->code == MEDIA_BUS_FMT_SBGGR12_1X12 ||
		    vpix->code == MEDIA_BUS_FMT_SBGGR14_1X14)
			val = DCMIPP_P1DMCR_TYPE_BGGR << DCMIPP_P1DMCR_TYPE_SHIFT;
		else if (vpix->code == MEDIA_BUS_FMT_SGBRG8_1X8 ||
			 vpix->code == MEDIA_BUS_FMT_SGBRG10_1X10 ||
			 vpix->code == MEDIA_BUS_FMT_SGBRG12_1X12 ||
			 vpix->code == MEDIA_BUS_FMT_SGBRG14_1X14)
			val = DCMIPP_P1DMCR_TYPE_GBRG << DCMIPP_P1DMCR_TYPE_SHIFT;
		else if (vpix->code == MEDIA_BUS_FMT_SGRBG8_1X8 ||
			 vpix->code == MEDIA_BUS_FMT_SGRBG10_1X10 ||
			 vpix->code == MEDIA_BUS_FMT_SGRBG12_1X12 ||
			 vpix->code == MEDIA_BUS_FMT_SGRBG14_1X14)
			val = DCMIPP_P1DMCR_TYPE_GRBG << DCMIPP_P1DMCR_TYPE_SHIFT;
		else if (vpix->code == MEDIA_BUS_FMT_SRGGB8_1X8 ||
			 vpix->code == MEDIA_BUS_FMT_SRGGB10_1X10 ||
			 vpix->code == MEDIA_BUS_FMT_SRGGB12_1X12 ||
			 vpix->code == MEDIA_BUS_FMT_SRGGB14_1X14)
			val = DCMIPP_P1DMCR_TYPE_RGGB << DCMIPP_P1DMCR_TYPE_SHIFT;

		val |= DCMIPP_P1DMCR_ENABLE;
	}

	if (val)
		reg_set(isp, DCMIPP_P1DMCR, val);
}

static int dcmipp_isp_colorconv_config(struct dcmipp_isp_device *isp)
{
	struct dcmipp_colorconv_config ccconf = { 0 };
	int i, ret = 0;
	unsigned int val = 0;

	ret = dcmipp_colorconv_configure(isp->dev, &isp->sink_fmt,
					 &isp->src_fmt, &ccconf);
	if (ret)
		return ret;

	for (i = 0; i < 6; i++)
		reg_write(isp, DCMIPP_P1CCRR1 + (4 * i), ccconf.conv_matrix[i]);

	if (ccconf.clamping)
		val |= DCMIPP_P1CCCR_CLAMP;
	if (!dcmipp_isp_is_yuv_fmt(isp->src_fmt.code))
		val |= DCMIPP_P1CCCR_TYPE_RGB;
	if (ccconf.enable)
		val |= DCMIPP_P1CCCR_ENABLE;

	reg_write(isp, DCMIPP_P1CCCR, val);

	return 0;
}

static int dcmipp_isp_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dcmipp_isp_device *isp = v4l2_get_subdevdata(sd);
	const struct dcmipp_isp_pix_map *vpix =
		dcmipp_isp_pix_map_by_code(isp->sink_fmt.code, 0);
	int ret = 0;

	mutex_lock(&isp->lock);
	if (enable) {
		/* Configure CSI DataType */
		reg_clear(isp, DCMIPP_P1FSCR, DCMIPP_P1FSCR_DTIDA_MASK);
		reg_set(isp, DCMIPP_P1FSCR,
			vpix->dt << DCMIPP_P1FSCR_DTIDA_SHIFT);

		/* Check if link between ISP & Pipe2 postproc is enabled */
		if (media_pad_remote_pad_first(&sd->entity.pads[2]))
			reg_clear(isp, DCMIPP_P1FSCR, DCMIPP_P1FSCR_PIPEDIFF);
		else
			reg_set(isp, DCMIPP_P1FSCR, DCMIPP_P1FSCR_PIPEDIFF);

		/* Configure Statistic Removal */
		reg_write(isp, DCMIPP_P1SRCR,
			  ((isp->crop.top << DCMIPP_P1SRCR_FIRSTLINEDEL_SHIFT) |
			   (isp->crop.height << DCMIPP_P1SRCR_LASTLINE_SHIFT) |
			   DCMIPP_P1SRCR_CROPEN));

		/* Configure Decimation */
		reg_write(isp, DCMIPP_P1DECR, isp->decimation);

		/* Configure Exposure */
		/* Configure Demosaicing */
		dcmipp_isp_config_demosaicing(isp);

		/* Configure ColorConversion */
		ret = dcmipp_isp_colorconv_config(isp);
		if (ret)
			goto out;
	}

	isp->streaming = enable;

	mutex_unlock(&isp->lock);

	return v4l2_ctrl_handler_setup(&isp->ctrls);

out:
	mutex_unlock(&isp->lock);

	return ret;
}

static const struct v4l2_subdev_video_ops dcmipp_isp_video_ops = {
	.s_stream = dcmipp_isp_s_stream,
};

static const struct v4l2_subdev_ops dcmipp_isp_ops = {
	.pad = &dcmipp_isp_pad_ops,
	.video = &dcmipp_isp_video_ops,
};

/* FIXME */
static void dcmipp_isp_release(struct v4l2_subdev *sd)
{
	struct dcmipp_isp_device *isp = v4l2_get_subdevdata(sd);

	kfree(isp);
}

static const struct v4l2_subdev_internal_ops dcmipp_isp_int_ops = {
	.release = dcmipp_isp_release,
};

static void dcmipp_isp_comp_unbind(struct device *comp, struct device *master,
				   void *master_data)
{
	struct dcmipp_ent_device *ved = dev_get_drvdata(comp);
	struct dcmipp_isp_device *isp =
			container_of(ved, struct dcmipp_isp_device, ved);

	dcmipp_ent_sd_unregister(ved, &isp->sd);
}

static int dcmipp_isp_comp_bind(struct device *comp, struct device *master,
				void *master_data)
{
	struct dcmipp_bind_data *bind_data = master_data;
	struct dcmipp_platform_data *pdata = comp->platform_data;
	struct dcmipp_isp_device *isp;
	int ret, i;

	/* Allocate the isp struct */
	isp = kzalloc(sizeof(*isp), GFP_KERNEL);
	if (!isp)
		return -ENOMEM;

	isp->regs = bind_data->regs;

	/* Initialize the lock */
	mutex_init(&isp->lock);

	/* Initialize ved and sd */
	ret = dcmipp_ent_sd_register(&isp->ved, &isp->sd,
				     bind_data->v4l2_dev,
				     pdata->entity_name,
				     MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER, 4,
				     (const unsigned long[4]) {
				     MEDIA_PAD_FL_SINK,
				     MEDIA_PAD_FL_SOURCE,
				     MEDIA_PAD_FL_SOURCE,
				     MEDIA_PAD_FL_SOURCE,
				     },
				     &dcmipp_isp_int_ops, &dcmipp_isp_ops,
				     NULL, NULL);
	if (ret) {
		kfree(isp);
		return ret;
	}

	dev_set_drvdata(comp, &isp->ved);
	isp->dev = comp;

	/* Initialize the frame format */
	isp->sink_fmt = fmt_sink_default;
	isp->src_fmt = fmt_src_default;

	/* Initialize controls */
	v4l2_ctrl_handler_init(&isp->ctrls, ARRAY_SIZE(dcmipp_isp_ctrls));
	isp->ctrls.lock = &isp->lock;

	for (i = 0; i < ARRAY_SIZE(dcmipp_isp_ctrls); i++)
		v4l2_ctrl_new_custom(&isp->ctrls, &dcmipp_isp_ctrls[i], NULL);

	isp->sd.ctrl_handler = &isp->ctrls;
	if (isp->ctrls.error) {
		dev_err(isp->dev, "control initialization error %d\n",
			isp->ctrls.error);
		/* TODO - error handling */
	}

	ret = v4l2_ctrl_handler_setup(&isp->ctrls);
	if (ret < 0) {
		dev_err(isp->dev, "Failed to set up control handlers\n");
		/* TODO - error handling */
	}

	return 0;
}

static const struct component_ops dcmipp_isp_comp_ops = {
	.bind = dcmipp_isp_comp_bind,
	.unbind = dcmipp_isp_comp_unbind,
};

static int dcmipp_isp_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &dcmipp_isp_comp_ops);
}

static int dcmipp_isp_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dcmipp_isp_comp_ops);

	return 0;
}

static const struct platform_device_id dcmipp_isp_driver_ids[] = {
	{
		.name           = DCMIPP_ISP_DRV_NAME,
	},
	{ }
};

static struct platform_driver dcmipp_isp_pdrv = {
	.probe		= dcmipp_isp_probe,
	.remove		= dcmipp_isp_remove,
	.id_table	= dcmipp_isp_driver_ids,
	.driver		= {
		.name	= DCMIPP_ISP_DRV_NAME,
	},
};

module_platform_driver(dcmipp_isp_pdrv);

MODULE_DEVICE_TABLE(platform, dcmipp_isp_driver_ids);

MODULE_AUTHOR("Hugues Fruchet <hugues.fruchet@foss.st.com>");
MODULE_AUTHOR("Alain Volmat <alain.volmat@foss.st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 Digital Camera Memory Interface with Pixel Processor driver");
MODULE_LICENSE("GPL");
