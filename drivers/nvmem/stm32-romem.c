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
};

struct stm32_romem_priv {
	void __iomem *base;
	struct nvmem_config cfg;
	struct clk *clk;
};

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
		priv->cfg.reg_read = stm32_bsec_read;
		priv->cfg.reg_write = stm32_bsec_write;
	}

	platform_set_drvdata(pdev, priv);

	nvmem = nvmem_register(&priv->cfg);
	if (IS_ERR(nvmem))
		goto err_pm_stop;

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

err_pm_stop:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_put_noidle(dev);

	if (priv->clk)
		clk_disable_unprepare(priv->clk);

	return PTR_ERR(nvmem);
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
};

static const struct of_device_id stm32_romem_of_match[] = {
	{ .compatible = "st,stm32f4-otp", }, {
		.compatible = "st,stm32mp15-bsec",
		.data = (void *)&stm32mp15_bsec_cfg,
	}, {
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
module_platform_driver(stm32_romem_driver);

MODULE_AUTHOR("Fabrice Gasnier <fabrice.gasnier@st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 RO-MEM");
MODULE_ALIAS("platform:nvmem-stm32-romem");
MODULE_LICENSE("GPL v2");
