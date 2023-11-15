// SPDX-License-Identifier: GPL-2.0-only
/*
 * ST PCIe driver for STM32-MP25 SoC
 *
 * Copyright (C) 2023 ST Microelectronics - All Rights Reserved
 * Author: Christian Bruel <christian.bruel@foss.st.com>
 */

#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/phy/phy.h>
#include <linux/msi.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include "pcie-designware.h"
#include "pcie-stm32.h"

struct stm32_pcie {
	struct dw_pcie *pci;
	struct regmap *regmap;
	struct reset_control *rst;
	struct phy *phy;
	struct clk *clk;
	struct gpio_desc *reset_gpio;
	int aer_irq;
	int pme_irq;
	u32     max_payload;
	u32     max_readreq;
};

static const struct of_device_id stm32_pcie_of_match[] = {
	{ .compatible = "st,stm32mp25-pcie-rc" },
	{},
};

static int stm32_pcie_set_max_payload(struct dw_pcie *pci, u32 size)
{
	u32 val;
	u16 exp_cap_off = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	int max_payload_size = fls(size) - 8;
	struct device *dev = pci->dev;

	if (size != 128 && size != 256) {
		dev_err(dev, "Unexpected payload size %d\n", size);
		return -EINVAL;
	}

	val = dw_pcie_readl_dbi(pci, exp_cap_off + PCI_EXP_DEVCTL);

	val &= ~PCI_EXP_DEVCTL_PAYLOAD;
	val |= PCIE_CAP_MAX_PAYLOAD_SIZE(max_payload_size);
	dw_pcie_writel_dbi(pci, exp_cap_off + PCI_EXP_DEVCTL, val);

	return 0;
}

static int stm32_pcie_set_max_rd_req_size(struct dw_pcie *pci, u32 size)
{
	u32 val;
	u16 exp_cap_off = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	int max_readreq_size = fls(size) - 8;
	struct device *dev = pci->dev;

	if (!is_power_of_2(size) || size < 128 || size > 1024) {
		dev_err(dev, "Unexpected read request size %d\n", size);
		return -EINVAL;
	}

	val = dw_pcie_readl_dbi(pci, exp_cap_off + PCI_EXP_DEVCTL);

	val &= ~PCI_EXP_DEVCTL_READRQ;
	val |= PCIE_CAP_MAX_READ_REQ_SIZE(max_readreq_size);
	dw_pcie_writel_dbi(pci, exp_cap_off + PCI_EXP_DEVCTL, val);

	return 0;
}

static int stm32_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct stm32_pcie *stm32_pcie = to_stm32_pcie(pci);
	int ret = 0;

	if (stm32_pcie->max_payload) {
		ret = stm32_pcie_set_max_payload(pci, stm32_pcie->max_payload);
		if (ret)
			return ret;
	}

	if (stm32_pcie->max_readreq)
		ret = stm32_pcie_set_max_rd_req_size(pci,
						     stm32_pcie->max_readreq);

	return ret;
}

static int stm32_pcie_start_link(struct dw_pcie *pci)
{
	struct stm32_pcie *stm32_pcie = to_stm32_pcie(pci);
	u32 ret;

	if (stm32_pcie->reset_gpio) {
		/* Make sure PERST# is asserted. */
		gpiod_set_value(stm32_pcie->reset_gpio, 1);

		/* Deassert PERST# after 100us */
		usleep_range(100, 200);
		gpiod_set_value(stm32_pcie->reset_gpio, 0);
	}

	ret = regmap_update_bits(stm32_pcie->regmap, SYSCFG_PCIECR,
				 STM32MP25_PCIECR_LTSSM_EN,
				 STM32MP25_PCIECR_LTSSM_EN);

	/*
	 * PCIe specification states that you should not issue any config
	 * requests until 100ms after asserting reset, so we enforce that here
	 */
	if (stm32_pcie->reset_gpio)
		msleep(100);

	return ret;
}

static void stm32_pcie_stop_link(struct dw_pcie *pci)
{
	struct stm32_pcie *stm32_pcie = to_stm32_pcie(pci);

	regmap_update_bits(stm32_pcie->regmap, SYSCFG_PCIECR, STM32MP25_PCIECR_LTSSM_EN, 0);

	/* Assert PERST# */
	if (stm32_pcie->reset_gpio)
		gpiod_set_value(stm32_pcie->reset_gpio, 1);
}

static const struct dw_pcie_host_ops stm32_pcie_host_ops = {
	.host_init = stm32_pcie_host_init
};

static const struct dw_pcie_ops dw_pcie_ops = {
	.start_link = stm32_pcie_start_link,
	.stop_link = stm32_pcie_stop_link
};

/*
 * PME/AER platform irq in this driver only re-initialises the platform IRQ glue logic.
 * The handling is done by the generic pci drivers (trough platform IRQ hook), through shared IRQ
 */
static irqreturn_t stm32_pcie_pme_msi_irq_handler(int irq, void *priv)
{
	struct stm32_pcie *stm32_pcie = priv;

	regmap_write(stm32_pcie->regmap, SYSCFG_PCIEPMEMSICR, 1);
	regmap_write(stm32_pcie->regmap, SYSCFG_PCIEPMEMSICR, 0);

	return IRQ_HANDLED;
}

static irqreturn_t stm32_pcie_aer_msi_irq_handler(int irq, void *priv)
{
	struct stm32_pcie *stm32_pcie = priv;

	regmap_write(stm32_pcie->regmap, SYSCFG_PCIEAERRCMSICR, 1);
	regmap_write(stm32_pcie->regmap, SYSCFG_PCIEAERRCMSICR, 0);

	return IRQ_HANDLED;
}

static int stm32_pcie_port_irqs(struct pci_dev *pci_dev, u32 *pme, u32 *aer, u32 *dpc)
{
	struct dw_pcie_rp *pp = pci_dev->bus->sysdata;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct stm32_pcie *stm32_pcie = to_stm32_pcie(pci);
	int nirqs = 0;

	if (aer && stm32_pcie->aer_irq)
		nirqs++;
	if (pme && stm32_pcie->pme_irq)
		nirqs++;

	nirqs = pci_alloc_irq_vectors(pci_dev, nirqs, nirqs, PCI_IRQ_MSI);
	if (nirqs < 0)
		return nirqs;

	if (aer && stm32_pcie->aer_irq)
		*aer = stm32_pcie->aer_irq;
	if (pme && stm32_pcie->pme_irq)
		*pme = stm32_pcie->pme_irq;

	return 0;
}

static int stm32_add_pcie_port(struct stm32_pcie *stm32_pcie,
			       struct platform_device *pdev)
{
	struct dw_pcie *pci = stm32_pcie->pci;
	struct device *dev = stm32_pcie->pci->dev;
	struct dw_pcie_rp *pp = &pci->pp;
	int ret;

	/* Use generic AER driver if we can register the port_drv */
	if (pci_aer_available()) {
		stm32_pcie->aer_irq = platform_get_irq_byname_optional(pdev, "aer_msi");
		if (stm32_pcie->aer_irq < 0) {
			if (stm32_pcie->aer_irq != -ENXIO)
				return dev_err_probe(dev, stm32_pcie->aer_irq,
						     "failed to get AER IRQ\n");
			stm32_pcie->aer_irq = 0;
		}

		if (stm32_pcie->aer_irq) {
			ret = devm_request_irq(dev, stm32_pcie->aer_irq,
					       stm32_pcie_aer_msi_irq_handler,
					       IRQF_SHARED, "stm32-aer-msi", stm32_pcie);
			if (ret < 0) {
				dev_err(dev, "failed to request AER MSI IRQ %d\n",
					stm32_pcie->aer_irq);
				return ret;
			}
		}
	}

	stm32_pcie->pme_irq = platform_get_irq_byname_optional(pdev, "pme_msi");
	if (stm32_pcie->pme_irq < 0) {
		if (stm32_pcie->pme_irq != -ENXIO)
			return dev_err_probe(dev, stm32_pcie->pme_irq,
					     "failed to get PME MSI IRQ\n");
		stm32_pcie->pme_irq = 0;
	}

	if (stm32_pcie->pme_irq) {
		ret = devm_request_irq(dev, stm32_pcie->pme_irq,
				       stm32_pcie_pme_msi_irq_handler,
				       IRQF_SHARED, "stm32-pme-msi", stm32_pcie);
		if (ret < 0) {
			dev_err(dev, "failed to request PME MSI IRQ %d\n",
				stm32_pcie->pme_irq);
			return ret;
		}
	}

	/* Register AER/PME pordrv hook */
	if (stm32_pcie->pme_irq || stm32_pcie->aer_irq)
		pcie_port_irqs_hook = stm32_pcie_port_irqs;

	ret = regmap_update_bits(stm32_pcie->regmap, SYSCFG_PCIECR,
				 STM32MP25_PCIECR_TYPE_MASK, STM32MP25_PCIECR_RC);
	if (ret)
		return ret;

	ret = reset_control_reset(stm32_pcie->rst);
	if (ret) {
		dev_err(&pdev->dev, "reset_control_reset failed %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(stm32_pcie->clk);
	if (ret)
		return ret;

	pp->ops = &stm32_pcie_host_ops;
	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize host: %d\n", ret);
		clk_disable_unprepare(stm32_pcie->clk);
		return ret;
	}

	return 0;
}

static int stm32_pcie_probe(struct platform_device *pdev)
{
	struct stm32_pcie *stm32_pcie;
	struct dw_pcie *dw;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	stm32_pcie = devm_kzalloc(dev, sizeof(*stm32_pcie), GFP_KERNEL);
	if (!stm32_pcie)
		return -ENOMEM;

	dw = devm_kzalloc(dev, sizeof(*dw), GFP_KERNEL);
	if (!dw)
		return -ENOMEM;
	stm32_pcie->pci = dw;

	dw->dev = dev;
	dw->ops = &dw_pcie_ops;

	/* regmap registers for PCIe IP configuration */
	stm32_pcie->regmap = syscon_regmap_lookup_by_phandle(np, "st,syscfg");
	if (IS_ERR(stm32_pcie->regmap))
		return dev_err_probe(dev, PTR_ERR(stm32_pcie->regmap),
				     "No syscfg phandle specified\n");

	stm32_pcie->phy = devm_phy_get(dev, "pcie-phy");
	if (IS_ERR(stm32_pcie->phy))
		return dev_err_probe(dev, PTR_ERR(stm32_pcie->phy),
				     "failed to get pcie-phy\n");

	stm32_pcie->clk = devm_clk_get(dev, "core");
	if (IS_ERR(stm32_pcie->clk))
		return dev_err_probe(dev, PTR_ERR(stm32_pcie->clk),
				     "Failed to get PCIe clock source\n");

	stm32_pcie->rst = devm_reset_control_get(dev, "pcie");
	if (IS_ERR(stm32_pcie->rst))
		return dev_err_probe(dev, PTR_ERR(stm32_pcie->rst),
				     "Failed to get PCIe reset\n");

	/* Optionally limit payload */
	ret = of_property_read_u32(np, "max-payload-size", &stm32_pcie->max_payload);
	if (ret && ret != -EINVAL)
		return dev_err_probe(dev, ret, "Error reading max-payload value\n");

	/* Optionally limit readreq */
	ret = of_property_read_u32(np, "max-readreq-size", &stm32_pcie->max_readreq);
	if (ret && ret != -EINVAL)
		return dev_err_probe(dev, ret, "Error reading max-readreq value\n");

	stm32_pcie->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(stm32_pcie->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(stm32_pcie->reset_gpio),
				     "Failed to get reset GPIO\n");

	ret = phy_set_mode(stm32_pcie->phy, PHY_MODE_PCIE);
	if (ret)
		return ret;

	ret = phy_init(stm32_pcie->phy);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, stm32_pcie);

	ret = stm32_add_pcie_port(stm32_pcie, pdev);
	if (ret)
		goto phy_disable;

	return 0;

phy_disable:
	phy_exit(stm32_pcie->phy);

	return ret;
}

static int stm32_pcie_remove(struct platform_device *pdev)
{
	struct stm32_pcie *stm32_pcie = platform_get_drvdata(pdev);
	struct dw_pcie_rp *pp = &stm32_pcie->pci->pp;

	dw_pcie_host_deinit(pp);
	clk_disable_unprepare(stm32_pcie->clk);
	pcie_port_irqs_hook = NULL;

	phy_exit(stm32_pcie->phy);

	return 0;
}

static struct platform_driver stm32_pcie_driver = {
	.probe = stm32_pcie_probe,
	.remove	= stm32_pcie_remove,
	.driver = {
		.name = "stm32-pcie",
		.of_match_table = stm32_pcie_of_match,
	},
};

static bool is_stm32_pcie_driver(struct device *dev)
{
	/* PCI bridge */
	dev = get_device(dev);

	/* Platform driver */
	dev = get_device(dev->parent);

	return (dev->driver == &stm32_pcie_driver.driver);
}

static void quirk_stm32_pcie_limit_mrrs(struct pci_dev *pci)
{
	struct pci_dev *root_port;
	struct pci_bus *bus = pci->bus;
	int readrq;
	int mps;

	if (pci_is_root_bus(bus))
		return;

	root_port = pcie_find_root_port(pci);

	if (WARN_ON(!root_port) || !is_stm32_pcie_driver(root_port->dev.parent))
		return;

	mps = pcie_get_mps(root_port);

	/*
	 * STM32 PCI controller has a h/w performance limitation on the AXI DDR requests.
	 * Limit the maximum read request size to 256B on all downstream devices.
	 */
	readrq = pcie_get_readrq(pci);
	if (readrq > 256) {
		int mrrs = min(mps, 256);

		pcie_set_readrq(pci, mrrs);

		pci_info(pci, "Max Read Rq set to %4d (was %4d)\n", mrrs, readrq);
	}
}

DECLARE_PCI_FIXUP_HEADER(PCI_ANY_ID, PCI_ANY_ID,
			 quirk_stm32_pcie_limit_mrrs);

static int stm32_dma_limit(struct pci_dev *pdev, void *data)
{
	dev_info(&pdev->dev, "set bus_dma_limit");

	pdev->dev.bus_dma_limit = DMA_BIT_MASK(32);

	return 0;
}

static void quirk_stm32_dma_mask(struct pci_dev *pci)
{
	struct pci_dev *root_port;

	root_port = pcie_find_root_port(pci);

	if (root_port && is_stm32_pcie_driver(root_port->dev.parent))
		pci_walk_bus(pci->bus, stm32_dma_limit, NULL);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_SYNOPSYS, 0x0550, quirk_stm32_dma_mask);

module_platform_driver(stm32_pcie_driver);
MODULE_DESCRIPTION("STM32MP25 PCIe Controller driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, stm32_pcie_of_match);
