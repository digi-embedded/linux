// SPDX-License-Identifier: GPL-2.0
/*
 * STM32MP25 VENC video encoder driver
 *
 * Copyright (C) STMicroelectronics SA 2022
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          for STMicroelectronics.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/reset.h>

#include "hantro.h"
#include "hantro_jpeg.h"
#include "hantro_h1_regs.h"

/*
 * Supported formats.
 */

static const struct hantro_fmt stm32mp25_venc_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_YUV420M,
		.codec_mode = HANTRO_MODE_NONE,
		.enc_fmt = ROCKCHIP_VPU_ENC_FMT_YUV420P,
		.frmsize = {
			.min_width = 96,
			.max_width = 8176,
			.step_width = MB_DIM,
			.min_height = 32,
			.max_height = 8176,
			.step_height = MB_DIM,
		},
	},
	{
		.fourcc = V4L2_PIX_FMT_NV12M,
		.codec_mode = HANTRO_MODE_NONE,
		.enc_fmt = ROCKCHIP_VPU_ENC_FMT_YUV420SP,
		.frmsize = {
			.min_width = 96,
			.max_width = 8176,
			.step_width = MB_DIM,
			.min_height = 32,
			.max_height = 8176,
			.step_height = MB_DIM,
		},
	},
	{
		.fourcc = V4L2_PIX_FMT_YUYV,
		.codec_mode = HANTRO_MODE_NONE,
		.enc_fmt = ROCKCHIP_VPU_ENC_FMT_YUYV422,
		.frmsize = {
			.min_width = 96,
			.max_width = 8176,
			.step_width = MB_DIM,
			.min_height = 32,
			.max_height = 8176,
			.step_height = MB_DIM,
		},
	},
	{
		.fourcc = V4L2_PIX_FMT_UYVY,
		.codec_mode = HANTRO_MODE_NONE,
		.enc_fmt = ROCKCHIP_VPU_ENC_FMT_UYVY422,
		.frmsize = {
			.min_width = 96,
			.max_width = 8176,
			.step_width = MB_DIM,
			.min_height = 32,
			.max_height = 8176,
			.step_height = MB_DIM,
		},
	},
	{
		.fourcc = V4L2_PIX_FMT_JPEG,
		.codec_mode = HANTRO_MODE_JPEG_ENC,
		.max_depth = 2,
		.header_size = JPEG_HEADER_SIZE,
		.frmsize = {
			.min_width = 96,
			.max_width = 8176,
			.step_width = MB_DIM,
			.min_height = 32,
			.max_height = 8176,
			.step_height = MB_DIM,
		},
	},
};

static irqreturn_t stm32mp25_venc_irq(int irq, void *dev_id)
{
	struct hantro_dev *vpu = dev_id;
	enum vb2_buffer_state state;
	u32 status;

	status = vepu_read(vpu, H1_REG_INTERRUPT);
	state = (status & H1_REG_INTERRUPT_FRAME_RDY) ?
		VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR;

	vepu_write(vpu, H1_REG_INTERRUPT_BIT, H1_REG_INTERRUPT);

	hantro_irq_done(vpu, state);

	return IRQ_HANDLED;
}

static void stm32mp25_venc_reset(struct hantro_ctx *ctx)
{
}

/*
 * Supported codec ops.
 */

static const struct hantro_codec_ops stm32mp25_venc_codec_ops[] = {
	[HANTRO_MODE_JPEG_ENC] = {
		.run = hantro_h1_jpeg_enc_run,
		.reset = stm32mp25_venc_reset,
		.done = hantro_h1_jpeg_enc_done,
	},
};

/*
 * Variants.
 */

static const struct hantro_irq stm32mp25_venc_irqs[] = {
	{ "venc", stm32mp25_venc_irq },
};

static const char * const stm32mp25_venc_clk_names[] = {
	"venc-clk"
};

const struct hantro_variant stm32mp25_venc_variant = {
	.enc_fmts = stm32mp25_venc_fmts,
	.num_enc_fmts = ARRAY_SIZE(stm32mp25_venc_fmts),
	.codec = HANTRO_JPEG_ENCODER,
	.codec_ops = stm32mp25_venc_codec_ops,
	.irqs = stm32mp25_venc_irqs,
	.num_irqs = ARRAY_SIZE(stm32mp25_venc_irqs),
	.clk_names = stm32mp25_venc_clk_names,
	.num_clocks = ARRAY_SIZE(stm32mp25_venc_clk_names)
};

