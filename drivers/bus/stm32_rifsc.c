// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023, STMicroelectronics - All Rights Reserved
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include <linux/bus/stm32_firewall.h>

/*
 * RIFSC offset register
 */
#define RIFSC_RISC_SECCFGR0		0x10
#define RIFSC_RISC_PRIVCFGR0		0x30
#define RIFSC_RISC_PER0_CIDCFGR		0x100
#define RIFSC_RISC_PER0_SEMCR		0x104
#define RIFSC_RISC_REG0_ACFGR		0x900
#define RIFSC_RISC_REG3_AADDR		0x924
#define RIFSC_RISC_HWCFGR2		0xFEC

/*
 * SEMCR register
 */
#define SEMCR_MUTEX			BIT(0)

/*
 * HWCFGR2 register
 */
#define HWCFGR2_CONF1_MASK		GENMASK(15, 0)
#define HWCFGR2_CONF2_MASK		GENMASK(23, 16)
#define HWCFGR2_CONF3_MASK		GENMASK(31, 24)

/*
 * RIFSC miscellaneous
 */
#define RIFSC_RISC_CFEN_MASK		BIT(0)
#define RIFSC_RISC_SEM_EN_MASK		BIT(1)
#define RIFSC_RISC_SCID_MASK		GENMASK(6, 4)
#define RIFSC_RISC_SEML_SHIFT		16
#define RIFSC_RISC_SEMWL_MASK		GENMASK(23, 16)
#define RIFSC_RISC_PER_ID_MASK		GENMASK(31, 24)

#define RIFSC_RISC_PERx_CID_MASK	(RIFSC_RISC_CFEN_MASK | \
					 RIFSC_RISC_SEM_EN_MASK | \
					 RIFSC_RISC_SCID_MASK | \
					 RIFSC_RISC_SEMWL_MASK)

#define IDS_PER_RISC_SEC_PRIV_REGS	32

/* RIF miscellaneous */
/*
 * CIDCFGR register fields
 */
#define CIDCFGR_CFEN			BIT(0)
#define CIDCFGR_SEMEN			BIT(1)
#define CIDCFGR_SEMWL(x)		BIT(RIFSC_RISC_SEML_SHIFT + (x))

#define SEMWL_SHIFT			16

/* Compartiment IDs */
#define RIF_CID0			0x0
#define RIF_CID1			0x1

#ifdef CONFIG_DEBUG_FS
#define STM32MP25_RIFSC_DEVICE_ENTRIES		128
#define STM32MP25_RIFSC_MASTER_ENTRIES	16
#define STM32MP25_RIFSC_RISAL_SUBREGIONS	2
#define STM32MP25_RIFSC_RISAL_GRANULARITY	8

#define RIFSC_RIMC_ATTR0		0xC10

#define RIFSC_RIMC_CIDSEL		BIT(2)
#define RIFSC_RIMC_MCID_MASK		GENMASK(6, 4)
#define RIFSC_RIMC_MSEC			BIT(8)
#define RIFSC_RIMC_MPRIV		BIT(9)

#define RIFSC_RISC_SRCID_MASK	GENMASK(6, 4)
#define RIFSC_RISC_SRPRIV	BIT(9)
#define RIFSC_RISC_SRSEC	BIT(8)
#define RIFSC_RISC_SRRLOCK	BIT(1)
#define RIFSC_RISC_SREN	BIT(0)
#define RIFSC_RISC_SRLENGTH_MASK	GENMASK(27, 16)
#define RIFSC_RISC_SRSTART_MASK	GENMASK(10, 0)

static const char *stm32mp25_rifsc_master_names[STM32MP25_RIFSC_MASTER_ENTRIES] = {
	"ETR",
	"SDMMC1",
	"SDMMC2",
	"SDMMC3",
	"USB3DR",
	"USBH",
	"ETH1",
	"ETH2",
	"PCIE",
	"GPU",
	"DMCIPP",
	"LTDC_L0/L1",
	"LTDC_L2",
	"LTDC_ROT",
	"VDEC",
	"VENC"
};

static const char *stm32mp25_rifsc_dev_names[STM32MP25_RIFSC_DEVICE_ENTRIES] = {
	"TIM1",
	"TIM2",
	"TIM3",
	"TIM4",
	"TIM5",
	"TIM6",
	"TIM7",
	"TIM8",
	"TIM10",
	"TIM11",
	"TIM12",
	"TIM13",
	"TIM14",
	"TIM15",
	"TIM16",
	"TIM17",
	"TIM20",
	"LPTIM1",
	"LPTIM2",
	"LPTIM3",
	"LPTIM4",
	"LPTIM5",
	"SPI1",
	"SPI2",
	"SPI3",
	"SPI4",
	"SPI5",
	"SPI6",
	"SPI7",
	"SPI8",
	"SPDIFRX",
	"USART1",
	"USART2",
	"USART3",
	"UART4",
	"UART5",
	"USART6",
	"UART7",
	"UART8",
	"UART9",
	"LPUART1",
	"I2C1",
	"I2C2",
	"I2C3",
	"I2C4",
	"I2C5",
	"I2C6",
	"I2C7",
	"I2C8",
	"SAI1",
	"SAI2",
	"SAI3",
	"SAI4",
	"RESERVED",
	"MDF1",
	"ADF1",
	"FDCAN",
	"HDP",
	"ADC12",
	"ADC3",
	"ETH1",
	"ETH2",
	"RESERVED",
	"USBH",
	"RESERVED",
	"RESERVED",
	"USB3DR",
	"COMBOPHY",
	"PCIE",
	"UCPD1",
	"ETHSW_DEIP",
	"ETHSW_ACM_CF",
	"ETHSW_ACM_MSGBU",
	"STGEN",
	"OCTOSPI1",
	"OCTOSPI2",
	"SDMMC1",
	"SDMMC2",
	"SDMMC3",
	"GPU",
	"LTDC_CMN",
	"DSI_CMN",
	"RESERVED",
	"RESERVED",
	"LVDS",
	"RESERVED",
	"CSI",
	"DCMIPP",
	"DCMI_PSSI",
	"VDEC",
	"VENC",
	"RESERVED",
	"RNG",
	"PKA",
	"SAES",
	"HASH",
	"CRYP1",
	"CRYP2",
	"IWDG1",
	"IWDG2",
	"IWDG3",
	"IWDG4",
	"IWDG5",
	"WWDG1",
	"WWDG2",
	"RESERVED",
	"VREFBUF",
	"DTS",
	"RAMCFG",
	"CRC",
	"SERC",
	"OCTOSPIM",
	"GICV2M",
	"RESERVED",
	"I3C1",
	"I3C2",
	"I3C3",
	"I3C4",
	"ICACHE_DCACHE",
	"LTDC_L0L1",
	"LTDC_L2",
	"LTDC_ROT",
	"DSI_TRIG",
	"DSI_RDFIFO",
	"RESERVED",
	"OTFDEC1",
	"OTFDEC2",
	"IAC",
};

struct rifsc_dev_debug_data {
	char dev_name[15];
	u8 dev_cid;
	u8 dev_sem_cids;
	u8 dev_id;
	bool dev_cid_filt_en;
	bool dev_sem_en;
	bool dev_priv;
	bool dev_sec;
};

struct rifsc_master_debug_data {
	char m_name[11];
	u8 m_cid;
	bool cidsel;
	bool m_sec;
	bool m_priv;
};

struct rifsc_subreg_debug_data {
	bool sr_sec;
	bool sr_priv;
	u8 sr_cid;
	bool sr_rlock;
	bool sr_enable;
	u16 sr_start;
	u16 sr_length;
};

static void stm32_rifsc_fill_master_dbg_entry(struct stm32_firewall_controller *rifsc,
					      struct rifsc_master_debug_data *dbg_entry, int i)
{
	u32 rimc_attr = readl_relaxed(rifsc->mmio + RIFSC_RIMC_ATTR0 + 0x4 * i);

	snprintf(dbg_entry->m_name, sizeof(dbg_entry->m_name), "%s",
		 stm32mp25_rifsc_master_names[i]);
	dbg_entry->m_cid = FIELD_GET(RIFSC_RIMC_MCID_MASK, rimc_attr);
	dbg_entry->cidsel = rimc_attr & RIFSC_RIMC_CIDSEL;
	dbg_entry->m_sec = rimc_attr & RIFSC_RIMC_MSEC;
	dbg_entry->m_priv = rimc_attr & RIFSC_RIMC_MPRIV;
}

static void stm32_rifsc_fill_dev_dbg_entry(struct stm32_firewall_controller *rifsc,
					   struct rifsc_dev_debug_data *dbg_entry, int i)
{
	u32 cid_cfgr, sec_cfgr, priv_cfgr;
	u8 reg_id = i / IDS_PER_RISC_SEC_PRIV_REGS;
	u8 reg_offset = i % IDS_PER_RISC_SEC_PRIV_REGS;

	cid_cfgr = readl_relaxed(rifsc->mmio + RIFSC_RISC_PER0_CIDCFGR + 0x8 * i);
	sec_cfgr = readl_relaxed(rifsc->mmio + RIFSC_RISC_SECCFGR0 + 0x4 * reg_id);
	priv_cfgr = readl_relaxed(rifsc->mmio + RIFSC_RISC_PRIVCFGR0 + 0x4 * reg_id);

	snprintf(dbg_entry->dev_name, sizeof(dbg_entry->dev_name), "%s",
		 stm32mp25_rifsc_dev_names[i]);
	dbg_entry->dev_id = i;
	dbg_entry->dev_cid_filt_en = cid_cfgr & CIDCFGR_CFEN;
	dbg_entry->dev_sem_en = cid_cfgr & CIDCFGR_SEMEN;
	dbg_entry->dev_cid = FIELD_GET(RIFSC_RISC_SCID_MASK, cid_cfgr);
	dbg_entry->dev_sem_cids = FIELD_GET(RIFSC_RISC_SEMWL_MASK, cid_cfgr);
	dbg_entry->dev_sec = sec_cfgr & BIT(reg_offset) ?  true : false;
	dbg_entry->dev_priv = priv_cfgr & BIT(reg_offset) ?  true : false;
}

static void stm32_rifsc_fill_subreg_dbg_entry(struct stm32_firewall_controller *rifsc,
					      struct rifsc_subreg_debug_data *dbg_entry, int i,
						  int j)
{
	u32 risc_xcfgr = readl_relaxed(rifsc->mmio + RIFSC_RISC_REG0_ACFGR + 0x10 * i + 0x8 * j);
	u32 risc_xaddr;

	dbg_entry->sr_sec = risc_xcfgr & RIFSC_RISC_SRSEC;
	dbg_entry->sr_priv = risc_xcfgr & RIFSC_RISC_SRPRIV;
	dbg_entry->sr_cid = FIELD_GET(RIFSC_RISC_SRCID_MASK, risc_xcfgr);
	dbg_entry->sr_rlock = risc_xcfgr & RIFSC_RISC_SRRLOCK;
	dbg_entry->sr_enable = risc_xcfgr & RIFSC_RISC_SREN;
	if (i == 2) {
		risc_xaddr = readl_relaxed(rifsc->mmio + RIFSC_RISC_REG3_AADDR + 0x8 * j);
		dbg_entry->sr_length = FIELD_GET(RIFSC_RISC_SRLENGTH_MASK, risc_xaddr);
		dbg_entry->sr_start = FIELD_GET(RIFSC_RISC_SRSTART_MASK, risc_xaddr);
	} else {
		dbg_entry->sr_start = 0;
		dbg_entry->sr_length = U16_MAX;
	}
}

static int stm32_rifsc_conf_dump_show(struct seq_file *s, void *data)
{
	struct stm32_firewall_controller *rifsc = (struct stm32_firewall_controller *)s->private;
	u32 subregion_start, subregion_end;
	int i, j;

	seq_puts(s, "\n=============================================\n");
	seq_puts(s, "                 RIFSC dump\n");
	seq_puts(s, "=============================================\n\n");

	seq_puts(s, "\n=============================================\n");
	seq_puts(s, "                 RISUP dump\n");
	seq_puts(s, "=============================================\n");

	seq_printf(s, "\n| %-15s |", "Peripheral name");
	seq_puts(s, "| Firewall ID |");
	seq_puts(s, "| N/SECURE |");
	seq_puts(s, "| N/PRIVILEGED |");
	seq_puts(s, "| CID filtering |");
	seq_puts(s, "| Semaphore mode |");
	seq_puts(s, "| SCID |");
	seq_printf(s, "| %7s |\n", "SEMWL");

	for (i = 0; i < STM32MP25_RIFSC_DEVICE_ENTRIES; i++) {
		struct rifsc_dev_debug_data d_dbg_entry;

		stm32_rifsc_fill_dev_dbg_entry(rifsc, &d_dbg_entry, i);

		seq_printf(s, "| %-15s |", d_dbg_entry.dev_name);
		seq_printf(s, "| %-11d |", d_dbg_entry.dev_id);
		seq_printf(s, "| %-8s |", d_dbg_entry.dev_sec ? "SEC" : "NSEC");
		seq_printf(s, "| %-12s |", d_dbg_entry.dev_priv ? "PRIV" : "NPRIV");
		seq_printf(s, "| %-13s |",
			   d_dbg_entry.dev_cid_filt_en ? "enabled" : "disabled");
		seq_printf(s, "| %-14s |",
			   d_dbg_entry.dev_sem_en ? "enabled" : "disabled");
		seq_printf(s, "| %-4d |", d_dbg_entry.dev_cid);
		seq_printf(s, "| %#-7x |\n", d_dbg_entry.dev_sem_cids);
	}

	seq_puts(s, "\n=============================================\n");
	seq_puts(s, "                  RIMU dump\n");
	seq_puts(s, "=============================================\n");

	seq_puts(s, "| Master name |");
	seq_puts(s, "| CIDSEL |");
	seq_puts(s, "| MCID |");
	seq_puts(s, "| N/SECURE |");
	seq_puts(s, "| N/PRIVILEGED |\n");

	for (i = 0; i < STM32MP25_RIFSC_MASTER_ENTRIES; i++) {
		struct rifsc_master_debug_data m_dbg_entry;

		stm32_rifsc_fill_master_dbg_entry(rifsc, &m_dbg_entry, i);

		seq_printf(s, "| %-11s |", m_dbg_entry.m_name);
		seq_printf(s, "| %-6s |", m_dbg_entry.cidsel ? "CIDSEL" : "");
		seq_printf(s, "| %-4d |", m_dbg_entry.m_cid);
		seq_printf(s, "| %-8s |", m_dbg_entry.m_sec ? "SEC" : "NSEC");
		seq_printf(s, "| %-12s |\n", m_dbg_entry.m_priv ? "PRIV" : "NPRIV");
	}

	if (rifsc->nb_risal > 0) {
		seq_puts(s, "\n=============================================\n");
		seq_puts(s, "                  RISAL dump\n");
		seq_puts(s, "=============================================\n");

		seq_puts(s, "| Memory  |");
		seq_puts(s, "| Subreg. |");
		seq_puts(s, "| N/SECURE |");
		seq_puts(s, "| N/PRIVILEGED |");
		seq_puts(s, "| Subreg. CID |");
		seq_puts(s, "| Resource lock |");
		seq_puts(s, "| Subreg. enable |");
		seq_puts(s, "| Subreg. start |");
		seq_puts(s, "|  Subreg. end  |\n");

		for (i = 0; i < rifsc->nb_risal; i++) {
			for (j = 0; j < STM32MP25_RIFSC_RISAL_SUBREGIONS; j++) {
				struct rifsc_subreg_debug_data sr_dbg_entry;

				stm32_rifsc_fill_subreg_dbg_entry(rifsc, &sr_dbg_entry, i, j);

				seq_printf(s, "| LPSRAM%1d |", i + 1);
				seq_printf(s, "|    %1s    |", (j == 0) ? "A" : "B");
				seq_printf(s, "| %-8s |", sr_dbg_entry.sr_sec ? "SEC" : "NSEC");
				seq_printf(s, "| %-12s |", sr_dbg_entry.sr_priv ? "PRIV" : "NPRIV");
				seq_printf(s, "| 0x%-9x |", sr_dbg_entry.sr_cid);
				seq_printf(s, "| %-13s |",
					   sr_dbg_entry.sr_rlock ? "locked (1)" : "unlocked (0)");
				seq_printf(s, "| %-14s |",
					   sr_dbg_entry.sr_enable ? "enabled" : "disabled");

				subregion_start = rifsc->risal_map_bases[2 * i] +
					sr_dbg_entry.sr_start * STM32MP25_RIFSC_RISAL_GRANULARITY;
				subregion_end = rifsc->risal_map_bases[2 * i] +
					rifsc->risal_map_bases[2 * i + 1] - 1;
				subregion_end = min(subregion_start + sr_dbg_entry.sr_length *
						    STM32MP25_RIFSC_RISAL_GRANULARITY - 1,
						    subregion_end);

				seq_printf(s, "| 0x%-11x |", subregion_start);
				seq_printf(s, "| 0x%-11x |\n", subregion_end);
			}
		}
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(stm32_rifsc_conf_dump);

static int stm32_rifsc_register_debugfs(struct stm32_firewall_controller *controller)
{
	struct dentry *root = NULL;

	root = debugfs_lookup("stm32_firewall", NULL);
	if (!root)
		root = debugfs_create_dir("stm32_firewall", NULL);

	if (IS_ERR(root))
		return PTR_ERR(root);

	debugfs_create_file("rifsc", 0444, root, controller, &stm32_rifsc_conf_dump_fops);

	return 0;
}
#endif /* CONFIG_DEBUG_FS */

static bool stm32_rifsc_is_semaphore_available(void __iomem *addr)
{
	return !(readl(addr) & SEMCR_MUTEX);
}

static int stm32_rif_acquire_semaphore(struct stm32_firewall_controller *stm32_firewall_controller,
				       int id)
{
	void __iomem *addr = stm32_firewall_controller->mmio + RIFSC_RISC_PER0_SEMCR + 0x8 * id;

	writel(SEMCR_MUTEX, addr);

	/* Check that CID1 has the semaphore */
	if (stm32_rifsc_is_semaphore_available(addr) ||
	    FIELD_GET(RIFSC_RISC_SCID_MASK, readl(addr)) != RIF_CID1)
		return -EACCES;

	return 0;
}

static void stm32_rif_release_semaphore(struct stm32_firewall_controller *stm32_firewall_controller,
					int id)
{
	void __iomem *addr = stm32_firewall_controller->mmio + RIFSC_RISC_PER0_SEMCR + 0x8 * id;

	if (stm32_rifsc_is_semaphore_available(addr))
		return;

	writel(SEMCR_MUTEX, addr);

	/* Ok if another compartment takes the semaphore before the check */
	WARN_ON(!stm32_rifsc_is_semaphore_available(addr) &&
		FIELD_GET(RIFSC_RISC_SCID_MASK, readl(addr)) == RIF_CID1);
}

static int stm32_rifsc_grant_access(struct stm32_firewall_controller *ctrl, u32 firewall_id)
{
	struct stm32_firewall_controller *rifsc_controller = ctrl;
	u32 reg_offset, reg_id, sec_reg_value, cid_reg_value;
	int rc;

	if (firewall_id >= rifsc_controller->max_entries) {
		dev_err(rifsc_controller->dev, "Invalid sys bus ID %u", firewall_id);
		return -EINVAL;
	}

	/*
	 * RIFSC_RISC_PRIVCFGRx and RIFSC_RISC_SECCFGRx both handle configuration access for
	 * 32 peripherals. On the other hand, there is one _RIFSC_RISC_PERx_CIDCFGR register
	 * per peripheral
	 */
	reg_id = firewall_id / IDS_PER_RISC_SEC_PRIV_REGS;
	reg_offset = firewall_id % IDS_PER_RISC_SEC_PRIV_REGS;
	sec_reg_value = readl(rifsc_controller->mmio + RIFSC_RISC_SECCFGR0 + 0x4 * reg_id);
	cid_reg_value = readl(rifsc_controller->mmio + RIFSC_RISC_PER0_CIDCFGR + 0x8 * firewall_id);

	/* First check conditions for semaphore mode, which doesn't take into account static CID. */
	if ((cid_reg_value & CIDCFGR_SEMEN) && (cid_reg_value & CIDCFGR_CFEN)) {
		if (cid_reg_value & BIT(RIF_CID1 + SEMWL_SHIFT)) {
			/* Static CID is irrelevant if semaphore mode */
			goto skip_cid_check;
		} else {
			dev_dbg(rifsc_controller->dev,
				"Invalid bus semaphore configuration: index %d\n", firewall_id);
			return -EACCES;
		}
	}

	/*
	 * Skip CID check if CID filtering isn't enabled or filtering is enabled on CID0, which
	 * corresponds to whatever CID.
	 */
	if (!(cid_reg_value & CIDCFGR_CFEN) ||
	    FIELD_GET(RIFSC_RISC_SCID_MASK, cid_reg_value) == RIF_CID0)
		goto skip_cid_check;

	/* Coherency check with the CID configuration */
	if (FIELD_GET(RIFSC_RISC_SCID_MASK, cid_reg_value) != RIF_CID1) {
		dev_dbg(rifsc_controller->dev, "Invalid CID configuration for peripheral: %d\n",
			firewall_id);
		return -EACCES;
	}

skip_cid_check:
	/* Check security configuration */
	if (sec_reg_value & BIT(reg_offset)) {
		dev_dbg(rifsc_controller->dev,
			"Invalid security configuration for peripheral: %d\n", firewall_id);
		return -EACCES;
	}

	/*
	 * If the peripheral is in semaphore mode, take the semaphore so that
	 * the CID1 has the ownership.
	 */
	if ((cid_reg_value & CIDCFGR_SEMEN) && (cid_reg_value & CIDCFGR_CFEN)) {
		rc = stm32_rif_acquire_semaphore(rifsc_controller, firewall_id);
		if (rc) {
			dev_err(rifsc_controller->dev,
				"Couldn't acquire semaphore for peripheral: %d\n", firewall_id);
			return rc;
		}
	}

	return 0;
}

static void stm32_rifsc_release_access(struct stm32_firewall_controller *ctrl, u32 firewall_id)
{
	stm32_rif_release_semaphore(ctrl, firewall_id);
}

static int stm32_rifsc_probe(struct platform_device *pdev)
{
	struct stm32_firewall_controller *rifsc_controller;
	struct device_node *np = pdev->dev.of_node;
	u32 nb_risup, nb_rimu, nb_risal;
	struct resource *res;
	void __iomem *mmio;
	int rc, err, nb_risal_map_bases;

	rifsc_controller = devm_kzalloc(&pdev->dev, sizeof(*rifsc_controller), GFP_KERNEL);
	if (!rifsc_controller)
		return -ENOMEM;

	mmio = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(mmio))
		return PTR_ERR(mmio);

	rifsc_controller->dev = &pdev->dev;
	rifsc_controller->mmio = mmio;
	rifsc_controller->name = dev_driver_string(rifsc_controller->dev);
	rifsc_controller->type = STM32_PERIPHERAL_FIREWALL | STM32_MEMORY_FIREWALL;
	rifsc_controller->grant_access = stm32_rifsc_grant_access;
	rifsc_controller->release_access = stm32_rifsc_release_access;

	/* Get number of RIFSC entries*/
	nb_risup = FIELD_GET(HWCFGR2_CONF1_MASK,
			     readl(rifsc_controller->mmio + RIFSC_RISC_HWCFGR2));
	nb_rimu = FIELD_GET(HWCFGR2_CONF2_MASK,
			    readl(rifsc_controller->mmio + RIFSC_RISC_HWCFGR2));
	nb_risal = FIELD_GET(HWCFGR2_CONF3_MASK,
			     readl(rifsc_controller->mmio + RIFSC_RISC_HWCFGR2));
	rifsc_controller->max_entries = nb_risup + nb_rimu + nb_risal;
	rifsc_controller->nb_risal = nb_risal;

	/*
	 * In STM32MP21, RIFSC_RISC_HWCFGR2 shows an incorrect number of RISAL (NUM_RISAL is 3
	 * instead of 0). A software workaround is implemented using the st,mem-map property in the
	 * device tree. This property is absent or left empty if there is no RISAL.
	 */
	if (of_property_present(np, "st,mem-map")) {
		nb_risal_map_bases = of_property_count_u32_elems(np, "st,mem-map");
		if (nb_risal_map_bases < 0) {
			pr_err("Couldn't read st,mem-map property");
			return nb_risal_map_bases;
		}
	} else {
		nb_risal_map_bases = 0;
	}

	if (nb_risal_map_bases == 0)
		rifsc_controller->nb_risal = 0;

	/* Get RISAL map bases */
	if (rifsc_controller->nb_risal > 0) {
		if (nb_risal_map_bases != 2 * rifsc_controller->nb_risal) {
			pr_err("RISAL count in HW configuration register and device tree mismatch");
			return -EINVAL;
		}

		rifsc_controller->risal_map_bases = devm_kzalloc(&pdev->dev, nb_risal_map_bases *
								sizeof(u32), GFP_KERNEL);
		if (!rifsc_controller->risal_map_bases)
			return -ENOMEM;

		err = of_property_read_u32_array(np, "st,mem-map",
						 rifsc_controller->risal_map_bases,
						 nb_risal_map_bases);
		if (err) {
			pr_err("Couldn't read st,mem-map property");
			return err;
		}
	}

	platform_set_drvdata(pdev, rifsc_controller);

	rc = stm32_firewall_controller_register(rifsc_controller);
	if (rc) {
		dev_err(rifsc_controller->dev, "Couldn't register as a firewall controller: %d",
			rc);
		return rc;
	}

	rc = stm32_firewall_populate_bus(rifsc_controller);
	if (rc) {
		dev_err(rifsc_controller->dev, "Couldn't populate RIFSC bus: %d",
			rc);
		return rc;
	}

	stm32_rifsc_register_debugfs(rifsc_controller);

	/* Populate all allowed nodes */
	return of_platform_populate(np, NULL, NULL, &pdev->dev);
}

static const struct of_device_id stm32_rifsc_of_match[] = {
	{ .compatible = "st,stm32mp25-rifsc" },
	{}
};
MODULE_DEVICE_TABLE(of, stm32_rifsc_of_match);

static struct platform_driver stm32_rifsc_driver = {
	.probe  = stm32_rifsc_probe,
	.driver = {
		.name = "stm32-rifsc",
		.of_match_table = stm32_rifsc_of_match,
	},
};
module_platform_driver(stm32_rifsc_driver);

MODULE_AUTHOR("Gatien Chevallier <gatien.chevallier@foss.st.com>");
MODULE_DESCRIPTION("STMicroelectronics RIFSC driver");
MODULE_LICENSE("GPL");
