// SPDX-License-Identifier: GPL-2.0
/*
 * STM32 Factory-programmed memory read access driver
 *
 * Copyright (C) 2017-2021, STMicroelectronics - All Rights Reserved
 * Author: Fabrice Gasnier <fabrice.gasnier@st.com> for STMicroelectronics.
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/tee_drv.h>

/* BSEC secure service access from non-secure */
#define STM32_SMC_BSEC			0x82001003
#define STM32_SMC_READ_SHADOW		0x01
#define STM32_SMC_PROG_OTP		0x02
#define STM32_SMC_WRITE_SHADOW		0x03
#define STM32_SMC_READ_OTP		0x04

/* shadow registers offest */
#define STM32MP15_BSEC_DATA0		0x200

/*
 * BSEC OTP regions: 4096 OTP bits (with 3072 effective bits)
 * - Lower: 1K bits, 2:1 redundancy, incremental bit programming
 *   => 32 (x 32-bits) lower shadow registers = words 0 to 31
 * - Upper: 2K bits, ECC protection, word programming only
 *   => 64 (x 32-bits) = words 32 to 95
 */
#define STM32MP15_BSEC_NUM_LOWER	32

#define STM32_ROMEM_AUTOSUSPEND_DELAY_MS	50

struct stm32_romem_cfg {
	int size;
	bool ta;
};

struct stm32_romem_priv {
	void __iomem *base;
	struct nvmem_config cfg;
	struct clk *clk;
	struct device *ta;
};

struct device *stm32_bsec_pta_find(void);
static int stm32_bsec_pta_read(void *context, unsigned int offset, void *buf,
			       size_t bytes);

static int stm32_romem_read(void *context, unsigned int offset, void *buf,
			    size_t bytes)
{
	struct stm32_romem_priv *priv = context;
	struct device *dev = priv->cfg.dev;
	u8 *buf8 = buf;
	int i, ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	for (i = offset; i < offset + bytes; i++)
		*buf8++ = readb_relaxed(priv->base + i);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;
}

static int stm32_bsec_smc(u8 op, u32 otp, u32 data, u32 *result)
{
#if IS_ENABLED(CONFIG_HAVE_ARM_SMCCC)
	struct arm_smccc_res res;

	arm_smccc_smc(STM32_SMC_BSEC, op, otp, data, 0, 0, 0, 0, &res);
	if (res.a0)
		return -EIO;

	if (result)
		*result = (u32)res.a1;

	return 0;
#else
	return -ENXIO;
#endif
}

static int stm32_bsec_read(void *context, unsigned int offset, void *buf,
			   size_t bytes)
{
	struct stm32_romem_priv *priv = context;
	struct device *dev = priv->cfg.dev;
	u32 roffset, rbytes, val;
	u8 *buf8 = buf, *val8 = (u8 *)&val;
	int i, j = 0, ret, skip_bytes, size;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	/* Round unaligned access to 32-bits */
	roffset = rounddown(offset, 4);
	skip_bytes = offset & 0x3;
	rbytes = roundup(bytes + skip_bytes, 4);

	if (roffset + rbytes > priv->cfg.size) {
		ret = -EINVAL;
		goto end_read;
	}

	for (i = roffset; (i < roffset + rbytes); i += 4) {
		u32 otp = i >> 2;

		if (otp < STM32MP15_BSEC_NUM_LOWER) {
			/* read lower data from shadow registers */
			val = readl_relaxed(
				priv->base + STM32MP15_BSEC_DATA0 + i);
		} else {
			ret = stm32_bsec_smc(STM32_SMC_READ_SHADOW, otp, 0,
					     &val);
			if (ret) {
				dev_err(dev, "Can't read data%d (%d)\n", otp,
					ret);
				goto end_read;
			}
		}
		/* skip first bytes in case of unaligned read */
		if (skip_bytes)
			size = min(bytes, (size_t)(4 - skip_bytes));
		else
			size = min(bytes, (size_t)4);
		memcpy(&buf8[j], &val8[skip_bytes], size);
		bytes -= size;
		j += size;
		skip_bytes = 0;
	}

end_read:
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

static int stm32_bsec_write(void *context, unsigned int offset, void *buf,
			    size_t bytes)
{
	struct stm32_romem_priv *priv = context;
	struct device *dev = priv->cfg.dev;
	u32 *buf32 = buf;
	int ret, i;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	/* Allow only writing complete 32-bits aligned words */
	if ((bytes % 4) || (offset % 4)) {
		ret = -EINVAL;
		goto end_write;
	}

	for (i = offset; i < offset + bytes; i += 4) {
		ret = stm32_bsec_smc(STM32_SMC_PROG_OTP, i >> 2, *buf32++,
				     NULL);
		if (ret) {
			dev_err(dev, "Can't write data%d (%d)\n", i >> 2, ret);
			goto end_write;
		}
	}

	if (offset + bytes >= STM32MP15_BSEC_NUM_LOWER * 4)
		dev_warn(dev, "Update of upper OTPs with ECC protection (word programming, only once)\n");

end_write:
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

static int stm32_romem_probe(struct platform_device *pdev)
{
	const struct stm32_romem_cfg *cfg;
	struct device *dev = &pdev->dev;
	struct stm32_romem_priv *priv;
	struct resource *res;
	struct nvmem_device *nvmem;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->cfg.name = "stm32-romem";
	priv->cfg.word_size = 1;
	priv->cfg.stride = 1;
	priv->cfg.dev = dev;
	priv->cfg.priv = priv;
	priv->cfg.owner = THIS_MODULE;
	priv->cfg.type = NVMEM_TYPE_OTP;

	priv->clk = devm_clk_get_optional(&pdev->dev, NULL);
	if (IS_ERR(priv->clk))
		return dev_err_probe(dev, PTR_ERR(priv->clk),
				     "failed to get clock\n");

	if (priv->clk) {
		ret = clk_prepare_enable(priv->clk);
		if (ret) {
			dev_err(dev, "failed to enable clock (%d)\n", ret);
			return ret;
		}
	}

	pm_runtime_set_autosuspend_delay(dev,
					 STM32_ROMEM_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);

	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	cfg = (const struct stm32_romem_cfg *)
		of_match_device(dev->driver->of_match_table, dev)->data;
	if (!cfg) {
		priv->cfg.read_only = true;
		priv->cfg.size = resource_size(res);
		priv->cfg.reg_read = stm32_romem_read;
	} else {
		priv->cfg.size = cfg->size;
		if (cfg->ta) {
			priv->ta = stm32_bsec_pta_find();
			/* wait for OP-TEE client driver to be up and ready */
			if (!priv->ta) {
				ret = -EPROBE_DEFER;
				goto err_pm_stop;
			}
			priv->cfg.read_only = true;
			priv->cfg.reg_read = stm32_bsec_pta_read;
		} else {
			priv->cfg.reg_read = stm32_bsec_read;
			priv->cfg.reg_write = stm32_bsec_write;
		}
	}

	platform_set_drvdata(pdev, priv);

	nvmem = nvmem_register(&priv->cfg);
	if (IS_ERR(nvmem)) {
		ret = PTR_ERR(nvmem);
		goto err_pm_stop;
	}

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

err_pm_stop:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_put_noidle(dev);

	if (priv->clk)
		clk_disable_unprepare(priv->clk);

	return ret;
}

static int stm32_romem_remove(struct platform_device *pdev)
{
	struct stm32_romem_priv *priv;
	int ret;

	priv = dev_get_drvdata(&pdev->dev);
	if (!priv)
		return -ENODEV;

	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0)
		return ret;

	nvmem_unregister((struct nvmem_device *)&priv->cfg);

	put_device(priv->ta);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);

	if (priv->clk)
		clk_disable_unprepare(priv->clk);

	return 0;
}

static int __maybe_unused stm32_romem_runtime_suspend(struct device *dev)
{
	struct stm32_romem_priv *priv;

	priv = dev_get_drvdata(dev);
	if (!priv)
		return -ENODEV;

	if (priv->clk)
		clk_disable_unprepare(priv->clk);

	return 0;
}

static int __maybe_unused stm32_romem_runtime_resume(struct device *dev)
{
	struct stm32_romem_priv *priv;
	int ret;

	priv = dev_get_drvdata(dev);
	if (!priv)
		return -ENODEV;

	if (priv->clk) {
		ret = clk_prepare_enable(priv->clk);
		if (ret) {
			dev_err(priv->cfg.dev,
				"Failed to prepare_enable clock (%d)\n", ret);
			return ret;
		}
	}

	return 0;
}

static const struct dev_pm_ops stm32_romem_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(stm32_romem_runtime_suspend,
			   stm32_romem_runtime_resume, NULL)
};

static const struct stm32_romem_cfg stm32mp15_bsec_cfg = {
	.size = 384, /* 96 x 32-bits data words */
	.ta = false,
};

static const struct stm32_romem_cfg stm32mp13_bsec_cfg = {
	.size = 384,     /* 96 x 32-bits data words */
	.ta = true,
};

static const struct of_device_id stm32_romem_of_match[] = {
	{ .compatible = "st,stm32f4-otp", }, {
		.compatible = "st,stm32mp15-bsec",
		.data = (void *)&stm32mp15_bsec_cfg,
	}, {
		.compatible = "st,stm32mp13-bsec",
		.data = (void *)&stm32mp13_bsec_cfg,
	},
};
MODULE_DEVICE_TABLE(of, stm32_romem_of_match);

static struct platform_driver stm32_romem_driver = {
	.probe = stm32_romem_probe,
	.remove = stm32_romem_remove,
	.driver = {
		.name = "stm32-romem",
		.pm = &stm32_romem_pm_ops,
		.of_match_table = of_match_ptr(stm32_romem_of_match),
	},
};

#if IS_ENABLED(CONFIG_OPTEE)
/*************************************************************************
 * BSEC PTA : OP-TEE client driver to pseudo trusted application
 *************************************************************************/
/*
 * Read OTP memory
 *
 * [in]	     value            a: OTP start offset in byte
 *                            b: access type
 *                               0 to read from shadow
 *                               1 to read from fuse
 *                               2 to read lock status
 * [out]     memref           buffer: Output buffer to store read values
 *                            size: Size of OTP to be read
 *
 * Return codes:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_BAD_PARAMETERS - Incorrect input param
 */
#define PTA_BSEC_READ_MEM		0x0 /* Read OTP */

/* value of PTA_BSEC access type = value[in] b */
#define SHADOW_ACCESS	0
#define FUSE_ACCESS	1
#define LOCK_ACCESS	2

/**
 * struct stm32_bsec_pta_priv - OP-TEE BSEC TA private data
 * @ctx:		OP-TEE context handler.
 * @session_id:		TA session identifier.
 */
struct stm32_bsec_pta_priv {
	struct tee_context *ctx;
	u32 session_id;
};

/*
 * Check whether this driver supports the BSEC TA in the TEE instance
 * represented by the params (ver/data) to this function.
 */
static int stm32_bsec_pta_match(struct tee_ioctl_version_data *ver, const void *data)
{
	/*
	 * Currently this driver only supports GP compliant, OP-TEE based TA
	 */
	if ((ver->impl_id == TEE_IMPL_ID_OPTEE) &&
		(ver->gen_caps & TEE_GEN_CAP_GP))
		return 1;
	else
		return 0;
}

/**
 * stm32_bsec_pta_probe() - initialize the PTA BSEC
 * @dev: the platform_device description.
 *
 * Return:
 *	On success, 0. On failure, -errno.
 */
static int stm32_bsec_pta_probe(struct device *dev)
{
	int rc;
	struct tee_ioctl_open_session_arg sess_arg;
	struct tee_client_device *tee_device = to_tee_client_device(dev);
	struct stm32_bsec_pta_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Open context with TEE driver */
	priv->ctx = tee_client_open_context(NULL, stm32_bsec_pta_match, NULL, NULL);
	if (IS_ERR(priv->ctx)) {
		if (PTR_ERR(priv->ctx) == -ENOENT)
			return -EPROBE_DEFER;
		dev_err(dev, "%s: tee_client_open_context failed\n", __func__);
		return PTR_ERR(priv->ctx);
	}

	/* Open a session with BSEC TA */
	memset(&sess_arg, 0, sizeof(sess_arg));
	export_uuid(sess_arg.uuid, &tee_device->id.uuid);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_REE_KERNEL;
	sess_arg.num_params = 0;

	rc = tee_client_open_session(priv->ctx, &sess_arg, NULL);
	if ((rc < 0) || (sess_arg.ret != 0)) {
		dev_err(dev, "%s: tee_client_open_session failed, err=%x\n",
			__func__, sess_arg.ret);
		rc = -EINVAL;
		goto out_tee_session;
	}
	priv->session_id = sess_arg.session;
	dev_set_drvdata(dev, priv);

	return 0;

out_tee_session:
	tee_client_close_context(priv->ctx);
	priv->ctx = NULL;

	return rc;
}

/**
 * stm32_bsec_pta_remove() - remove the BSEC TEE device
 * @dev: the platform_device description.
 *
 * Return:
 *	0 always.
 */
static int stm32_bsec_pta_remove(struct device *dev)
{
	struct stm32_bsec_pta_priv *priv = dev_get_drvdata(dev);

	if (!IS_ERR_OR_NULL(priv->ctx)) {
		tee_client_close_session(priv->ctx, priv->session_id);
		tee_client_close_context(priv->ctx);
	}

	return 0;
}

/**
 * stm32_bsec_pta_read() - nvmem read access using PTA client driver
 * @context: nvmem context => romem privdate data
 * @offset: nvmem offset
 * @buf: buffer to fill with nvem values
 * @bytes: number of bytes to read
 *
 * Return:
 *	On success, 0. On failure, -errno.
 */
static int stm32_bsec_pta_read(void *context, unsigned int offset, void *buf,
			       size_t bytes)
{
	struct stm32_romem_priv *romem_priv = context;
	struct device *dev;
	struct stm32_bsec_pta_priv *priv;
	struct tee_shm *shm;
	struct tee_ioctl_invoke_arg arg;
	struct tee_param param[2];
	u8 *shm_buf;
	u32 start, num_bytes;
	int ret;

	dev = romem_priv->ta;
	if (!dev) {
		pr_err("TA_BSEC invoke with driver\n");
		return -ENXIO;
	}

	priv = dev_get_drvdata(dev);

	memset(&arg, 0, sizeof(arg));
	memset(&param, 0, sizeof(param));

	arg.func = PTA_BSEC_READ_MEM;
	arg.session = priv->session_id;
	arg.num_params = 2;

	/* align access on 32bits */
	start = ALIGN_DOWN(offset, 4);
	num_bytes = round_up(offset + bytes - start, 4);
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = start;
	param[0].u.value.b = SHADOW_ACCESS;

	shm = tee_shm_alloc(priv->ctx, num_bytes, TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm))
		return PTR_ERR(shm);

	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT;
	param[1].u.memref.shm = shm;
	param[1].u.memref.size = num_bytes;

	ret = tee_client_invoke_func(priv->ctx, &arg, param);
	if (ret < 0 || arg.ret != 0) {
		dev_err(dev, "TA_BSEC invoke failed TEE err: %x, ret:%x\n",
			arg.ret, ret);
		if (!ret)
			ret = -EIO;
	}
	if (!ret) {
		shm_buf = tee_shm_get_va(shm, 0);
		if (IS_ERR(shm_buf)) {
			dev_err(dev, "tee_shm_get_va failed for transmit\n");
			ret = PTR_ERR(shm_buf);
		} else {
			ret = 0;
			/* read data from 32 bits aligned buffer */
			memcpy(buf, &shm_buf[offset % 4], bytes);
		}
	}

	tee_shm_free(shm);

	return ret;
}

static const struct tee_client_device_id stm32_bsec_id_table[] = {
	{
		UUID_INIT(0x94cf71ad, 0x80e6, 0x40b5,
			  0xa7, 0xc6, 0x3d, 0xc5, 0x01, 0xeb, 0x28, 0x03)
	},
	{ }
};

MODULE_DEVICE_TABLE(tee, stm32_bsec_id_table);

static struct tee_client_driver stm32_bsec_pta_driver = {
	.id_table	= stm32_bsec_id_table,
	.driver		= {
		.name = "stm32-bsec-pta",
		.bus = &tee_bus_type,
		.probe = stm32_bsec_pta_probe,
		.remove = stm32_bsec_pta_remove,
	},
};

struct device *stm32_bsec_pta_find(void)
{
	return driver_find_next_device(&stm32_bsec_pta_driver.driver, NULL);
}

#else
static int stm32_bsec_pta_read(void *context, unsigned int offset, void *buf,
			       size_t bytes)
{
	pr_debug("%s: TA BSEC request without OPTEE support\n", __func__);

	return -ENXIO;
}
struct device *stm32_bsec_pta_find(void)
{
	pr_debug("%s: TA BSEC request without OPTEE support\n", __func__);

	return NULL;
}
#endif

static int __init stm32_romem_init(void)
{
	int rc;

	rc = platform_driver_register(&stm32_romem_driver);
	if (rc)
		return rc;

#if IS_ENABLED(CONFIG_OPTEE)
	rc = driver_register(&stm32_bsec_pta_driver.driver);
#endif

	return rc;
}

static void __exit stm32_romem_exit(void)
{
	platform_driver_unregister(&stm32_romem_driver);
#if IS_ENABLED(CONFIG_OPTEE)
	driver_unregister(&stm32_bsec_pta_driver.driver);
#endif
}

module_init(stm32_romem_init);
module_exit(stm32_romem_exit);

MODULE_AUTHOR("Fabrice Gasnier <fabrice.gasnier@st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 RO-MEM");
MODULE_ALIAS("platform:nvmem-stm32-romem");
MODULE_LICENSE("GPL v2");
