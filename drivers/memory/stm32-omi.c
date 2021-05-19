// SPDX-License-Identifier: GPL
/*
 * Copyright (C) 2022, STMicroelectronics - All Rights Reserved
 * Author(s): Patrice Chotard <patrice.chotard@foss.st.com> for STMicroelectronics.
 */

#include <memory/stm32-omi.h>

static void stm32_omi_read_fifo(u8 *val, void __iomem *addr)
{
	*val = readb_relaxed(addr);
}

static void stm32_omi_write_fifo(u8 *val, void __iomem *addr)
{
	writeb_relaxed(*val, addr);
}

int stm32_omi_abort(struct stm32_omi *omi)
{
	void __iomem *regs_base = omi->regs_base;
	u32 cr;
	int timeout;

	cr = readl_relaxed(regs_base + OSPI_CR) | CR_ABORT;
	writel_relaxed(cr, regs_base + OSPI_CR);

	/* wait clear of abort bit by hw */
	timeout = readl_relaxed_poll_timeout_atomic(regs_base + OSPI_CR,
						    cr, !(cr & CR_ABORT), 1,
						    STM32_ABT_TIMEOUT_US);

	if (timeout)
		dev_err(omi->dev, "%s abort timeout:%d\n", __func__, timeout);

	return timeout;
}
EXPORT_SYMBOL(stm32_omi_abort);

int stm32_omi_tx_poll(struct stm32_omi *omi, u8 *buf, u32 len, bool read)
{
	void __iomem *regs_base = omi->regs_base;
	void (*tx_fifo)(u8 *val, void __iomem *addr);
	u32 sr;
	int ret;

	if (read)
		tx_fifo = stm32_omi_read_fifo;
	else
		tx_fifo = stm32_omi_write_fifo;

	while (len--) {
		ret = readl_relaxed_poll_timeout_atomic(regs_base + OSPI_SR,
							sr, sr & SR_FTF, 1,
							STM32_FIFO_TIMEOUT_US);
		if (ret) {
			dev_err(omi->dev, "fifo timeout (len:%d stat:%#x)\n",
				len, sr);
			return ret;
		}
		tx_fifo(buf++, regs_base + OSPI_DR);
	}

	return 0;
}
EXPORT_SYMBOL(stm32_omi_tx_poll);

int stm32_omi_wait_nobusy(struct stm32_omi *omi)
{
	u32 sr;

	return readl_relaxed_poll_timeout_atomic(omi->regs_base + OSPI_SR,
						 sr, !(sr & SR_BUSY), 1,
						 STM32_BUSY_TIMEOUT_US);
}
EXPORT_SYMBOL(stm32_omi_wait_nobusy);

int stm32_omi_wait_cmd(struct stm32_omi *omi)
{
	void __iomem *regs_base = omi->regs_base;
	u32 cr, sr;
	int err = 0;

	if ((readl_relaxed(regs_base + OSPI_SR) & SR_TCF) ||
	    omi->fmode == CR_FMODE_APM)
		goto out;

	reinit_completion(&omi->data_completion);
	cr = readl_relaxed(regs_base + OSPI_CR);
	writel_relaxed(cr | CR_TCIE | CR_TEIE, regs_base + OSPI_CR);

	if (!wait_for_completion_timeout(&omi->data_completion,
				msecs_to_jiffies(STM32_COMP_TIMEOUT_MS))) {
		err = -ETIMEDOUT;
	} else {
		sr = readl_relaxed(regs_base + OSPI_SR);
		if (sr & SR_TEF)
			err = -EIO;
	}

out:
	/* clear flags */
	writel_relaxed(FCR_CTCF | FCR_CTEF, regs_base + OSPI_FCR);

	if (!err)
		err = stm32_omi_wait_nobusy(omi);

	return err;
}
EXPORT_SYMBOL(stm32_omi_wait_cmd);

irqreturn_t stm32_omi_irq(int irq, void *dev_id)
{
	struct stm32_omi *omi = (struct stm32_omi *)dev_id;
	void __iomem *regs_base = omi->regs_base;
	u32 cr, sr;

	cr = readl_relaxed(regs_base + OSPI_CR);
	sr = readl_relaxed(regs_base + OSPI_SR);

	if (cr & CR_SMIE && sr & SR_SMF) {
		/* disable irq */
		cr &= ~CR_SMIE;
		writel_relaxed(cr, regs_base + OSPI_CR);
		complete(&omi->match_completion);

		return IRQ_HANDLED;
	}

	if (sr & (SR_TEF | SR_TCF)) {
		/* disable irq */
		cr = readl_relaxed(regs_base + OSPI_CR);
		cr &= ~CR_TCIE & ~CR_TEIE;
		writel_relaxed(cr, regs_base + OSPI_CR);
		complete(&omi->data_completion);
	}

	return IRQ_HANDLED;
}
EXPORT_SYMBOL(stm32_omi_irq);

void stm32_omi_dma_callback(void *arg)
{
	struct completion *dma_completion = arg;

	complete(dma_completion);
}
EXPORT_SYMBOL(stm32_omi_dma_callback);

void stm32_omi_dma_free(struct stm32_omi *omi)
{
	if (omi->dma_chtx)
		dma_release_channel(omi->dma_chtx);
	if (omi->dma_chrx)
		dma_release_channel(omi->dma_chrx);
}
EXPORT_SYMBOL(stm32_omi_dma_free);

int stm32_omi_dma_setup(struct stm32_omi *omi, struct device *dev,
			       struct dma_slave_config *dma_cfg)
{
	int ret = 0;

	omi->dma_chrx = dma_request_chan(dev, "rx");
	if (IS_ERR(omi->dma_chrx)) {
		ret = PTR_ERR(omi->dma_chrx);
		omi->dma_chrx = NULL;
		if (ret == -EPROBE_DEFER)
			goto out;
	} else if (dma_cfg) {
		if (dmaengine_slave_config(omi->dma_chrx, dma_cfg)) {
			dev_err(dev, "dma rx config failed\n");
			dma_release_channel(omi->dma_chrx);
			omi->dma_chrx = NULL;
		}
	}

	omi->dma_chtx = dma_request_chan(dev, "tx");
	if (IS_ERR(omi->dma_chtx)) {
		ret = PTR_ERR(omi->dma_chtx);
		omi->dma_chtx = NULL;
	} else if (dma_cfg) {
		if (dmaengine_slave_config(omi->dma_chtx, dma_cfg)) {
			dev_err(dev, "dma tx config failed\n");
			dma_release_channel(omi->dma_chtx);
			omi->dma_chtx = NULL;
		}
	}

	init_completion(&omi->dma_completion);
out:
	if (ret != -EPROBE_DEFER)
		ret = 0;

	return ret;
}
EXPORT_SYMBOL(stm32_omi_dma_setup);

int stm32_omi_get_resources(struct stm32_omi *omi, struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *np = dev->of_node;
	struct resource res;
	struct reserved_mem *rmem = NULL;
	struct device_node *node;
	int ret = 0;

	omi->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(omi->clk))
		return dev_err_probe(dev, PTR_ERR(omi->clk),
				     "Can't get clock\n");

	omi->irq = platform_get_irq(pdev, 0);
	if (omi->irq < 0) {
		dev_err(dev, "Can't get irq %d\n", omi->irq);
		return omi->irq;
	}

	omi->rstc = devm_reset_control_array_get(dev, false, true);
	if (IS_ERR(omi->rstc))
		return dev_err_probe(dev, PTR_ERR(omi->rstc),
				     "Can't get reset\n");

	ret = of_address_to_resource(np, 0, &res);
	if (ret)
		return ret;

	omi->regs_base = ioremap(res.start, resource_size(&res));
	if (IS_ERR(omi->regs_base))
		return PTR_ERR(omi->regs_base);

	omi->regs_phys_base = res.start;

	node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (node)
		rmem = of_reserved_mem_lookup(node);

	of_node_put(node);
	if (rmem) {
		omi->mm_phys_base = rmem->base;
		omi->mm_size = rmem->size;
		omi->mm_base = devm_ioremap(dev, rmem->base, rmem->size);
		if (IS_ERR(omi->mm_base)) {
			dev_err(dev, "unable to map memory region: %pa+%pa\n",
				&rmem->base, &rmem->size);
			ret = PTR_ERR(omi->mm_base);
		}

		if (omi->mm_size > STM32_OMI_MAX_MMAP_SZ) {
			dev_err(dev, "Memory map size outsize bounds\n");
			ret = -EINVAL;
		}
	} else {
		dev_info(dev, "No memory-map region found\n");
	}

	return ret;
}
EXPORT_SYMBOL(stm32_omi_get_resources);

static int stm32_omi_probe(struct platform_device *pdev)
{
	struct platform_device *vdev;
	struct device *dev = &pdev->dev;
	struct device_node *child;
	const char *name;
	u8 hyperflash_count = 0;
	u8 spi_flash_count = 0;
	u8 child_count = 0;

	/*
	 * Flash subnodes sanity check:
	 *        2 spi-nand/spi-nor flashes			=> supported
	 *        1 HyperFlash					=> supported
	 *	  All other flash node configuration		=> not supported
	 */
	for_each_available_child_of_node(dev->of_node, child) {
		if (of_device_is_compatible(child, "cfi-flash"))
			hyperflash_count++;

		if (of_device_is_compatible(child, "jedec,spi-nor") ||
		    of_device_is_compatible(child, "spi-nand"))
			spi_flash_count++;

		child_count++;
	}

	if (!child_count) {
		dev_err(dev, "Missing flash node\n");
		return -ENODEV;
	}

	if ((!hyperflash_count && !spi_flash_count) ||
	    child_count != (hyperflash_count + spi_flash_count)) {
		dev_err(dev, "Unknown flash type\n");
		return -ENODEV;
	}

	if ((hyperflash_count && spi_flash_count) ||
	     hyperflash_count > 1) {
		dev_err(dev, "Flash node configuration not supported\n");
		return -EINVAL;
	}

	if (spi_flash_count)
		name = "stm32-ospi";
	else
		name = "stm32-hyperflash";

	vdev = platform_device_alloc(name, PLATFORM_DEVID_AUTO);
	if (!vdev)
		return -ENOMEM;

	vdev->dev.parent = dev;
	platform_set_drvdata(pdev, vdev);

	return platform_device_add(vdev);
}

static int stm32_omi_remove(struct platform_device *pdev)
{
	struct platform_device *vdev = platform_get_drvdata(pdev);

	platform_device_unregister(vdev);

	return 0;
}

static const struct of_device_id stm32_omi_of_match[] = {
	{ .compatible = "st,stm32mp25-omi", },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_omi_of_match);

static struct platform_driver stm32_omi_driver = {
	.probe	= stm32_omi_probe,
	.remove	= stm32_omi_remove,
	.driver = {
		.name =	"stm32-omi",
		.of_match_table = stm32_omi_of_match,
	},
};
module_platform_driver(stm32_omi_driver);

MODULE_DESCRIPTION("STMicroelectronics STM32 OSPI Memory Interface driver");
MODULE_LICENSE("GPL");
