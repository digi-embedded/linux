// SPDX-License-Identifier: GPL
/*
 * Copyright (C) 2023, STMicroelectronics - All Rights Reserved
 * Author(s): Patrice Chotard <patrice.chotard@st.com> for STMicroelectronics.
 */

#include <linux/mtd/cfi.h>
#include <memory/stm32-omi.h>

struct stm32_hyperbus {
	struct device *dev;
	struct hyperbus_ctlr ctlr;
	struct hyperbus_device hbdev;
	struct stm32_omi omi;
	u32 flash_freq;		/* flash max supported frequency */
	u32 real_flash_freq;	/* real flash freq = bus_freq x prescaler */
	u32 tacc;
	u32 cs;
	bool wzl;
};

static void stm32_hyperbus_copy_from(struct hyperbus_device *hbdev, void *to,
				     unsigned long from, ssize_t len)
{
	struct stm32_hyperbus *hyperbus =
		container_of(hbdev, struct stm32_hyperbus, hbdev);
	void __iomem *mm_base = hyperbus->omi.mm_base;

	memcpy_fromio(to, omi->mm_base + from, len);
}

static void stm32_hyperbus_set_mode(struct stm32_hyperbus *hyperbus, u8 mode)
{
	struct stm32_omi *omi = &hyperbus->omi;
	void __iomem *regs_base = omi->regs_base;
	u32 cr;

	if (omi->fmode == mode)
		return;

	if (omi->fmode == CR_FMODE_MM)
		stm32_omi_abort(omi);

	cr = readl_relaxed(regs_base + OSPI_CR);
	cr &= ~CR_FMODE_MASK;
	cr |= FIELD_PREP(CR_FMODE_MASK, mode);
	writel_relaxed(cr, regs_base + OSPI_CR);
	omi->fmode = mode;
}

static u16 stm32_hyperbus_read16(struct hyperbus_device *hbdev, unsigned long addr)
{
	struct stm32_hyperbus *hyperbus =
		container_of(hbdev, struct stm32_hyperbus, hbdev);
	void __iomem *regs_base = hyperbus->omi.regs_base;
	u16 data;

	stm32_hyperbus_set_mode(hyperbus, CR_FMODE_INDR);
	writel(addr, regs_base + OSPI_AR);
	stm32_omi_tx_poll(&hyperbus->omi, (u8 *)&data, 2, true);

	/* Wait end of tx in indirect mode */
	stm32_omi_wait_cmd(&hyperbus->omi);

	dev_dbg(hyperbus->dev, "%s: read 0x%x @ 0x%lx\n",
		__func__, data, addr >> 1);

	stm32_hyperbus_set_mode(hyperbus, CR_FMODE_MM);

	return data;
}

static void stm32_hyperbus_write16(struct hyperbus_device *hbdev,
				  unsigned long addr, u16 data)
{
	struct stm32_hyperbus *hyperbus =
		container_of(hbdev, struct stm32_hyperbus, hbdev);
	void __iomem *regs_base = hyperbus->omi.regs_base;

	stm32_hyperbus_set_mode(hyperbus, CR_FMODE_INDW);
	writel(addr, regs_base + OSPI_AR);
	stm32_omi_tx_poll(&hyperbus->omi, (u8 *)&data, 2, false);

	/* Wait end of tx in indirect mode */
	stm32_omi_wait_cmd(&hyperbus->omi);

	dev_dbg(hyperbus->dev, "%s: write 0x%x @ 0x%lx\n",
		__func__, data, addr >> 1);

	stm32_hyperbus_set_mode(hyperbus, CR_FMODE_MM);
}

static int stm32_hyperbus_check_transfert(struct stm32_omi *omi)
{
	struct stm32_hyperbus *hyperbus =
		container_of(omi, struct stm32_hyperbus, omi);
	struct map_info *map = &hyperbus->hbdev.map;
	struct cfi_private cfi;
	int ret;

	cfi.interleave = 1;
	cfi.device_type = CFI_DEVICETYPE_X16;

	ret = cfi_qry_mode_on(0, map, &cfi);
	cfi_qry_mode_off(0, map, &cfi);

	return !ret;
}

static int stm32_hyperbus_calibrate(struct hyperbus_device *hbdev)
{
	struct stm32_hyperbus *hyperbus =
		container_of(hbdev, struct stm32_hyperbus, hbdev);
	struct stm32_omi *omi = &hyperbus->omi;
	void __iomem *regs_base = omi->regs_base;
	u32 prescaler;
	u16 period_ps = 0;
	int ret;
	bool bypass_mode = false;

	prescaler = FIELD_GET(DCR2_PRESC_MASK,
			      readl(regs_base + OSPI_DCR2));
	if (prescaler)
		writel_relaxed(TCR_DHQC, regs_base + OSPI_TCR);

	if (hyperbus->real_flash_freq <= STM32_DLYB_FREQ_THRESHOLD) {
		bypass_mode = true;
		period_ps = NSEC_PER_SEC / (hyperbus->real_flash_freq / 1000);
	}

	ret = stm32_omi_dlyb_init(omi, bypass_mode, period_ps);
	if (ret) {
		dev_err(hyperbus->dev, "Can't enable delay block\n");
		return 0;
	}

	if (bypass_mode || prescaler)
		/* perform only RX TAP selection */
		ret = stm32_omi_dlyb_find_tap(omi, true);
	else
		/* perform RX/TX TAP selection */
		ret = stm32_omi_dlyb_find_tap(omi, false);

	if (ret) {
		dev_err(omi->dev, "Calibration failed\n");
		if (!bypass_mode)
			/* stop delay block when configured in lock mode */
			stm32_omi_dlyb_stop(omi);
	}

	return ret ? 0 : 1;
}

static const struct hyperbus_ops stm32_hyperbus_ops = {
	.read16 = stm32_hyperbus_read16,
	.write16 = stm32_hyperbus_write16,
	.copy_from = stm32_hyperbus_copy_from,
	.calibrate = stm32_hyperbus_calibrate,
};

static void stm32_hyperbus_init(struct stm32_hyperbus *hyperbus)
{
	struct stm32_omi *omi = &hyperbus->omi;
	void __iomem *regs_base = omi->regs_base;
	unsigned long period;
	u32 cr, dcr1, hlcr, ccr;
	u32 prescaler;

	cr = CR_EN;
	cr |= FIELD_PREP(CR_CSSEL, hyperbus->cs);
	writel_relaxed(cr, regs_base + OSPI_CR);

	/* set MTYP to HyperBus memory-map mode */
	dcr1 = FIELD_PREP(DCR1_MTYP_MASK, DCR1_MTYP_HP_MEMMODE);
	/* set DEVSIZE to memory map size */
	dcr1 |= FIELD_PREP(DCR1_DEVSIZE_MASK, ffs(omi->mm_size) - 1);
	writel_relaxed(dcr1, regs_base + OSPI_DCR1);

	prescaler = DIV_ROUND_UP(omi->clk_rate, hyperbus->flash_freq) - 1;
	if (prescaler > 255)
		prescaler = 255;

	writel_relaxed(FIELD_PREP(DCR2_PRESC_MASK, prescaler), regs_base + OSPI_DCR2);
	hyperbus->real_flash_freq = omi->clk_rate / (prescaler + 1);

	writel_relaxed(1, regs_base + OSPI_DLR);

	/* set access time latency */
	period = NSEC_PER_SEC / hyperbus->real_flash_freq;
	hlcr = FIELD_PREP(HLCR_TACC_MASK, DIV_ROUND_UP(hyperbus->tacc, period));
	if (hyperbus->wzl)
		hlcr |= HLCR_WZL;

	writel_relaxed(hlcr, regs_base + OSPI_HLCR);

	ccr = CCR_DQSE | CCR_DDTR | CCR_ADDTR;
	ccr |= FIELD_PREP(CCR_DMODE_MASK, CCR_DMODE_8LINES);
	ccr |= FIELD_PREP(CCR_ADSIZE_MASK, CCR_ADSIZE_32BITS);
	ccr |= FIELD_PREP(CCR_ADMODE_MASK, CCR_ADMODE_8LINES);
	writel(ccr, regs_base + OSPI_CCR);

	stm32_hyperbus_set_mode(hyperbus, CR_FMODE_MM);
}

static int __maybe_unused stm32_hyperbus_suspend(struct device *dev)
{
	struct stm32_hyperbus *hyperbus = dev_get_drvdata(dev);
	struct stm32_omi *omi = &hyperbus->omi;
	void __iomem *regs_base = omi->regs_base;
	u32 cr;

	cr = readl_relaxed(regs_base + OSPI_CR);
	cr &= ~CR_EN;
	writel_relaxed(cr, regs_base + OSPI_CR);

	clk_disable_unprepare(omi->clk);

	return pinctrl_pm_select_sleep_state(dev);
}

static int __maybe_unused stm32_hyperbus_resume(struct device *dev)
{
	struct stm32_hyperbus *hyperbus = dev_get_drvdata(dev);
	struct stm32_omi *omi = &hyperbus->omi;
	int ret;

	ret = clk_prepare_enable(omi->clk);
	if (ret)
		return ret;

	pinctrl_pm_select_default_state(dev);
	stm32_hyperbus_init(hyperbus);

	return 0;
}

static const struct dev_pm_ops stm32_hyperbus_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stm32_hyperbus_suspend, stm32_hyperbus_resume)
};

static int stm32_hyperbus_probe(struct platform_device *pdev)
{
	struct device *dev = pdev->dev.parent;
	struct stm32_hyperbus *hyperbus;
	struct stm32_omi *omi;
	struct device_node *flash;
	u32 value;
	int ret;

	hyperbus = devm_kzalloc(&pdev->dev, sizeof(*hyperbus), GFP_KERNEL);
	if (!hyperbus)
		return -ENOMEM;

	omi = &hyperbus->omi;
	omi->dev = &pdev->dev;
	omi->calibration = false;
	hyperbus->dev = dev;

	ret = stm32_omi_get_resources(omi, dev);
	if (ret)
		return ret;

	/* mandatory for HyperFlash */
	if (!omi->mm_size) {
		dev_err(dev, "Memory-map region not found\n");
		return -EINVAL;
	}

	/* mandatory for HyperFlash */
	if (!omi->dlyb_base) {
		dev_err(dev, "Incorrect delay block base address\n");
		return -EINVAL;
	}

	omi->clk_rate = clk_get_rate(omi->clk);
	if (!omi->clk_rate) {
		dev_err(dev, "Invalid clock rate\n");
		return -EINVAL;
	}

	ret = clk_prepare_enable(omi->clk);
	if (ret) {
		dev_err(dev, "Can not enable hyperbus clock\n");
		return ret;
	}

	if (omi->rstc) {
		reset_control_assert(omi->rstc);
		udelay(2);
		reset_control_deassert(omi->rstc);
	}

	ret = devm_request_irq(dev, omi->irq, stm32_omi_irq, 0,
			       dev_name(dev), omi);
	if (ret) {
		dev_err(dev, "Failed to request irq\n");
		goto err_clk_disable;
	}

	flash = of_get_next_child(dev->of_node, NULL);
	if (!flash) {
		dev_warn(&pdev->dev, "No flash node found\n");
		goto err_clk_disable;
	}

	ret = of_property_read_u32(flash, "reg", &hyperbus->cs);
	if (ret) {
		dev_err(&pdev->dev, "Can't find reg property\n");
		goto err_clk_disable;
	}

	ret = of_property_read_u32(flash, "st,max-frequency", &value);
	if (ret) {
		dev_err(&pdev->dev, "Can't find st,max-frequency property\n");
		goto err_clk_disable;
	}
	hyperbus->flash_freq = value;

	/* optional */
	ret = of_property_read_u32(flash, "st,tacc-ns", &hyperbus->tacc);
	if (ret)
		hyperbus->tacc = 0;

	hyperbus->wzl = of_property_read_bool(flash, "st,wzl");

	init_completion(&omi->data_completion);
	omi->check_transfer = stm32_hyperbus_check_transfert;

	stm32_hyperbus_init(hyperbus);

	platform_set_drvdata(pdev, hyperbus);

	hyperbus->hbdev.map.size = omi->mm_size;
	hyperbus->hbdev.map.virt = omi->mm_base;

	hyperbus->dev = &pdev->dev;
	hyperbus->ctlr.dev = &pdev->dev;
	hyperbus->ctlr.ops = &stm32_hyperbus_ops;
	hyperbus->hbdev.ctlr = &hyperbus->ctlr;
	hyperbus->hbdev.np = of_get_next_child(dev->of_node, NULL);

	return hyperbus_register_device(&hyperbus->hbdev);

err_clk_disable:
	clk_disable_unprepare(omi->clk);

	return ret;
}

static int stm32_hyperbus_remove(struct platform_device *pdev)
{
	struct stm32_hyperbus *hyperbus = platform_get_drvdata(pdev);

	hyperbus_unregister_device(&hyperbus->hbdev);
	stm32_omi_dma_free(&hyperbus->omi);
	clk_disable_unprepare(hyperbus->omi.clk);

	return 0;
}

static struct platform_driver stm32_hyperbus_driver = {
	.probe	= stm32_hyperbus_probe,
	.remove	= stm32_hyperbus_remove,
	.driver	= {
		.name = "stm32-hyperflash",
		.pm = &stm32_hyperbus_pm_ops,
	},
};
module_platform_driver(stm32_hyperbus_driver);

MODULE_DESCRIPTION("STMicroelectronics STM32 HyperBus driver");
MODULE_LICENSE("GPL");
