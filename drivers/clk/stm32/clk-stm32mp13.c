// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2022 - All Rights Reserved
 * Author: Gabriel Fernandez <gabriel.fernandez@foss.st.com> for STMicroelectronics.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/stm32mp13-clks.h>

#include "clk-stm32-core.h"
#include "stm32mp13_rcc.h"

#define RCC_CLR		0x4

static const char * const adc12_src[] = {
	"pll4_r", "ck_per", "pll3_q"
};

static const char * const dcmipp_src[] = {
	"ck_axi", "pll2_q", "pll4_p", "ck_per",
};

static const char * const eth12_src[] = {
	"pll4_p", "pll3_q"
};

static const char * const fdcan_src[] = {
	"ck_hse", "pll3_q", "pll4_q", "pll4_r"
};

static const char * const fmc_src[] = {
	"ck_axi", "pll3_r", "pll4_p", "ck_per"
};

static const char * const i2c12_src[] = {
	"pclk1", "pll4_r", "ck_hsi", "ck_csi"
};

static const char * const i2c345_src[] = {
	"pclk6", "pll4_r", "ck_hsi", "ck_csi"
};

static const char * const lptim1_src[] = {
	"pclk1", "pll4_p", "pll3_q", "ck_lse", "ck_lsi", "ck_per"
};

static const char * const lptim23_src[] = {
	"pclk3", "pll4_q", "ck_per", "ck_lse", "ck_lsi"
};

static const char * const lptim45_src[] = {
	"pclk3", "pll4_p", "pll3_q", "ck_lse", "ck_lsi", "ck_per"
};

static const char * const mco1_src[] = {
	"ck_hsi", "ck_hse", "ck_csi", "ck_lsi", "ck_lse"
};

static const char * const mco2_src[] = {
	"ck_mpu", "ck_axi", "ck_mlahb", "pll4_p", "ck_hse", "ck_hsi"
};

static const char * const qspi_src[] = {
	"ck_axi", "pll3_r", "pll4_p", "ck_per"
};

static const char * const rng1_src[] = {
	"ck_csi", "pll4_r", "reserved", "ck_lsi"
};

static const char * const saes_src[] = {
	"ck_axi", "ck_per", "pll4_r", "ck_lsi"
};

static const char * const sai1_src[] = {
	"pll4_q", "pll3_q", "i2s_ckin", "ck_per", "pll3_r"
};

static const char * const sai2_src[] = {
	"pll4_q", "pll3_q", "i2s_ckin", "ck_per", "spdif_ck_symb", "pll3_r"
};

static const char * const sdmmc12_src[] = {
	"ck_axi", "pll3_r", "pll4_p", "ck_hsi"
};

static const char * const spdif_src[] = {
	"pll4_p", "pll3_q", "ck_hsi"
};

static const char * const spi123_src[] = {
	"pll4_p", "pll3_q", "i2s_ckin", "ck_per", "pll3_r"
};

static const char * const spi4_src[] = {
	"pclk6", "pll4_q", "ck_hsi", "ck_csi", "ck_hse", "i2s_ckin"
};

static const char * const spi5_src[] = {
	"pclk6", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const stgen_src[] = {
	"ck_hsi", "ck_hse"
};

static const char * const usart12_src[] = {
	"pclk6", "pll3_q", "ck_hsi", "ck_csi", "pll4_q", "ck_hse"
};

static const char * const usart34578_src[] = {
	"pclk1", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const usart6_src[] = {
	"pclk2", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const usbo_src[] = {
	"pll4_r", "ck_usbo_48m"
};

static const char * const usbphy_src[] = {
	"ck_hse", "pll4_r", "clk-hse-div2"
};

enum enum_gate_cfg {
	GATE_MCO1,
	GATE_MCO2,
	GATE_DBGCK,
	GATE_TRACECK,
	GATE_DDRC1,
	GATE_DDRC1LP,
	GATE_DDRPHYC,
	GATE_DDRPHYCLP,
	GATE_DDRCAPB,
	GATE_DDRCAPBLP,
	GATE_AXIDCG,
	GATE_DDRPHYCAPB,
	GATE_DDRPHYCAPBLP,
	GATE_TIM2,
	GATE_TIM3,
	GATE_TIM4,
	GATE_TIM5,
	GATE_TIM6,
	GATE_TIM7,
	GATE_LPTIM1,
	GATE_SPI2,
	GATE_SPI3,
	GATE_USART3,
	GATE_UART4,
	GATE_UART5,
	GATE_UART7,
	GATE_UART8,
	GATE_I2C1,
	GATE_I2C2,
	GATE_SPDIF,
	GATE_TIM1,
	GATE_TIM8,
	GATE_SPI1,
	GATE_USART6,
	GATE_SAI1,
	GATE_SAI2,
	GATE_DFSDM,
	GATE_ADFSDM,
	GATE_FDCAN,
	GATE_LPTIM2,
	GATE_LPTIM3,
	GATE_LPTIM4,
	GATE_LPTIM5,
	GATE_VREF,
	GATE_DTS,
	GATE_PMBCTRL,
	GATE_HDP,
	GATE_SYSCFG,
	GATE_DCMIPP,
	GATE_DDRPERFM,
	GATE_IWDG2APB,
	GATE_USBPHY,
	GATE_STGENRO,
	GATE_LTDC,
	GATE_TZC,
	GATE_ETZPC,
	GATE_IWDG1APB,
	GATE_BSEC,
	GATE_STGENC,
	GATE_USART1,
	GATE_USART2,
	GATE_SPI4,
	GATE_SPI5,
	GATE_I2C3,
	GATE_I2C4,
	GATE_I2C5,
	GATE_TIM12,
	GATE_TIM13,
	GATE_TIM14,
	GATE_TIM15,
	GATE_TIM16,
	GATE_TIM17,
	GATE_DMA1,
	GATE_DMA2,
	GATE_DMAMUX1,
	GATE_DMA3,
	GATE_DMAMUX2,
	GATE_ADC1,
	GATE_ADC2,
	GATE_USBO,
	GATE_TSC,
	GATE_GPIOA,
	GATE_GPIOB,
	GATE_GPIOC,
	GATE_GPIOD,
	GATE_GPIOE,
	GATE_GPIOF,
	GATE_GPIOG,
	GATE_GPIOH,
	GATE_GPIOI,
	GATE_PKA,
	GATE_SAES,
	GATE_CRYP1,
	GATE_HASH1,
	GATE_RNG1,
	GATE_BKPSRAM,
	GATE_AXIMC,
	GATE_MCE,
	GATE_ETH1CK,
	GATE_ETH1TX,
	GATE_ETH1RX,
	GATE_ETH1MAC,
	GATE_FMC,
	GATE_QSPI,
	GATE_SDMMC1,
	GATE_SDMMC2,
	GATE_CRC1,
	GATE_USBH,
	GATE_ETH2CK,
	GATE_ETH2TX,
	GATE_ETH2RX,
	GATE_ETH2MAC,
	GATE_ETH1STP,
	GATE_ETH2STP,
	GATE_MDMA,
	GATE_NB
};

#define _CFG_GATE(_id, _offset, _bit_idx, _offset_clr)\
	[(_id)] = {\
		.offset		= (_offset),\
		.bit_idx	= (_bit_idx),\
		.set_clr	= (_offset_clr),\
	}

#define CFG_GATE(_id, _offset, _bit_idx)\
	_CFG_GATE(_id, _offset, _bit_idx, 0)

#define CFG_GATE_SETCLR(_id, _offset, _bit_idx)\
	_CFG_GATE(_id, _offset, _bit_idx, RCC_CLR)

static struct stm32_gate_cfg stm32mp13_gates[] = {
	CFG_GATE(GATE_MCO1,		RCC_MCO1CFGR,	12),
	CFG_GATE(GATE_MCO2,		RCC_MCO2CFGR,	12),
	CFG_GATE(GATE_DBGCK,		RCC_DBGCFGR,	8),
	CFG_GATE(GATE_TRACECK,		RCC_DBGCFGR,	9),
	CFG_GATE(GATE_DDRC1,		RCC_DDRITFCR,	0),
	CFG_GATE(GATE_DDRC1LP,		RCC_DDRITFCR,	1),
	CFG_GATE(GATE_DDRPHYC,		RCC_DDRITFCR,	4),
	CFG_GATE(GATE_DDRPHYCLP,	RCC_DDRITFCR,	5),
	CFG_GATE(GATE_DDRCAPB,		RCC_DDRITFCR,	6),
	CFG_GATE(GATE_DDRCAPBLP,	RCC_DDRITFCR,	7),
	CFG_GATE(GATE_AXIDCG,		RCC_DDRITFCR,	8),
	CFG_GATE(GATE_DDRPHYCAPB,	RCC_DDRITFCR,	9),
	CFG_GATE(GATE_DDRPHYCAPBLP,	RCC_DDRITFCR,	10),
	CFG_GATE_SETCLR(GATE_TIM2,	RCC_MP_APB1ENSETR,	0),
	CFG_GATE_SETCLR(GATE_TIM3,	RCC_MP_APB1ENSETR,	1),
	CFG_GATE_SETCLR(GATE_TIM4,	RCC_MP_APB1ENSETR,	2),
	CFG_GATE_SETCLR(GATE_TIM5,	RCC_MP_APB1ENSETR,	3),
	CFG_GATE_SETCLR(GATE_TIM6,	RCC_MP_APB1ENSETR,	4),
	CFG_GATE_SETCLR(GATE_TIM7,	RCC_MP_APB1ENSETR,	5),
	CFG_GATE_SETCLR(GATE_LPTIM1,	RCC_MP_APB1ENSETR,	9),
	CFG_GATE_SETCLR(GATE_SPI2,	RCC_MP_APB1ENSETR,	11),
	CFG_GATE_SETCLR(GATE_SPI3,	RCC_MP_APB1ENSETR,	12),
	CFG_GATE_SETCLR(GATE_USART3,	RCC_MP_APB1ENSETR,	15),
	CFG_GATE_SETCLR(GATE_UART4,	RCC_MP_APB1ENSETR,	16),
	CFG_GATE_SETCLR(GATE_UART5,	RCC_MP_APB1ENSETR,	17),
	CFG_GATE_SETCLR(GATE_UART7,	RCC_MP_APB1ENSETR,	18),
	CFG_GATE_SETCLR(GATE_UART8,	RCC_MP_APB1ENSETR,	19),
	CFG_GATE_SETCLR(GATE_I2C1,	RCC_MP_APB1ENSETR,	21),
	CFG_GATE_SETCLR(GATE_I2C2,	RCC_MP_APB1ENSETR,	22),
	CFG_GATE_SETCLR(GATE_SPDIF,	RCC_MP_APB1ENSETR,	26),
	CFG_GATE_SETCLR(GATE_TIM1,	RCC_MP_APB2ENSETR,	0),
	CFG_GATE_SETCLR(GATE_TIM8,	RCC_MP_APB2ENSETR,	1),
	CFG_GATE_SETCLR(GATE_SPI1,	RCC_MP_APB2ENSETR,	8),
	CFG_GATE_SETCLR(GATE_USART6,	RCC_MP_APB2ENSETR,	13),
	CFG_GATE_SETCLR(GATE_SAI1,	RCC_MP_APB2ENSETR,	16),
	CFG_GATE_SETCLR(GATE_SAI2,	RCC_MP_APB2ENSETR,	17),
	CFG_GATE_SETCLR(GATE_DFSDM,	RCC_MP_APB2ENSETR,	20),
	CFG_GATE_SETCLR(GATE_ADFSDM,	RCC_MP_APB2ENSETR,	21),
	CFG_GATE_SETCLR(GATE_FDCAN,	RCC_MP_APB2ENSETR,	24),
	CFG_GATE_SETCLR(GATE_LPTIM2,	RCC_MP_APB3ENSETR,	0),
	CFG_GATE_SETCLR(GATE_LPTIM3,	RCC_MP_APB3ENSETR,	1),
	CFG_GATE_SETCLR(GATE_LPTIM4,	RCC_MP_APB3ENSETR,	2),
	CFG_GATE_SETCLR(GATE_LPTIM5,	RCC_MP_APB3ENSETR,	3),
	CFG_GATE_SETCLR(GATE_VREF,	RCC_MP_APB3ENSETR,	13),
	CFG_GATE_SETCLR(GATE_DTS,	RCC_MP_APB3ENSETR,	16),
	CFG_GATE_SETCLR(GATE_PMBCTRL,	RCC_MP_APB3ENSETR,	17),
	CFG_GATE_SETCLR(GATE_HDP,	RCC_MP_APB3ENSETR,	20),
	CFG_GATE_SETCLR(GATE_SYSCFG,	RCC_MP_NS_APB3ENSETR,	0),
	CFG_GATE_SETCLR(GATE_DCMIPP,	RCC_MP_APB4ENSETR,	1),
	CFG_GATE_SETCLR(GATE_DDRPERFM,	RCC_MP_APB4ENSETR,	8),
	CFG_GATE_SETCLR(GATE_IWDG2APB,	RCC_MP_APB4ENSETR,	15),
	CFG_GATE_SETCLR(GATE_USBPHY,	RCC_MP_APB4ENSETR,	16),
	CFG_GATE_SETCLR(GATE_STGENRO,	RCC_MP_APB4ENSETR,	20),
	CFG_GATE_SETCLR(GATE_LTDC,	RCC_MP_NS_APB4ENSETR,	0),
	CFG_GATE_SETCLR(GATE_TZC,	RCC_MP_APB5ENSETR,	11),
	CFG_GATE_SETCLR(GATE_ETZPC,	RCC_MP_APB5ENSETR,	13),
	CFG_GATE_SETCLR(GATE_IWDG1APB,	RCC_MP_APB5ENSETR,	15),
	CFG_GATE_SETCLR(GATE_BSEC,	RCC_MP_APB5ENSETR,	16),
	CFG_GATE_SETCLR(GATE_STGENC,	RCC_MP_APB5ENSETR,	20),
	CFG_GATE_SETCLR(GATE_USART1,	RCC_MP_APB6ENSETR,	0),
	CFG_GATE_SETCLR(GATE_USART2,	RCC_MP_APB6ENSETR,	1),
	CFG_GATE_SETCLR(GATE_SPI4,	RCC_MP_APB6ENSETR,	2),
	CFG_GATE_SETCLR(GATE_SPI5,	RCC_MP_APB6ENSETR,	3),
	CFG_GATE_SETCLR(GATE_I2C3,	RCC_MP_APB6ENSETR,	4),
	CFG_GATE_SETCLR(GATE_I2C4,	RCC_MP_APB6ENSETR,	5),
	CFG_GATE_SETCLR(GATE_I2C5,	RCC_MP_APB6ENSETR,	6),
	CFG_GATE_SETCLR(GATE_TIM12,	RCC_MP_APB6ENSETR,	7),
	CFG_GATE_SETCLR(GATE_TIM13,	RCC_MP_APB6ENSETR,	8),
	CFG_GATE_SETCLR(GATE_TIM14,	RCC_MP_APB6ENSETR,	9),
	CFG_GATE_SETCLR(GATE_TIM15,	RCC_MP_APB6ENSETR,	10),
	CFG_GATE_SETCLR(GATE_TIM16,	RCC_MP_APB6ENSETR,	11),
	CFG_GATE_SETCLR(GATE_TIM17,	RCC_MP_APB6ENSETR,	12),
	CFG_GATE_SETCLR(GATE_DMA1,	RCC_MP_AHB2ENSETR,	0),
	CFG_GATE_SETCLR(GATE_DMA2,	RCC_MP_AHB2ENSETR,	1),
	CFG_GATE_SETCLR(GATE_DMAMUX1,	RCC_MP_AHB2ENSETR,	2),
	CFG_GATE_SETCLR(GATE_DMA3,	RCC_MP_AHB2ENSETR,	3),
	CFG_GATE_SETCLR(GATE_DMAMUX2,	RCC_MP_AHB2ENSETR,	4),
	CFG_GATE_SETCLR(GATE_ADC1,	RCC_MP_AHB2ENSETR,	5),
	CFG_GATE_SETCLR(GATE_ADC2,	RCC_MP_AHB2ENSETR,	6),
	CFG_GATE_SETCLR(GATE_USBO,	RCC_MP_AHB2ENSETR,	8),
	CFG_GATE_SETCLR(GATE_TSC,	RCC_MP_AHB4ENSETR,	15),
	CFG_GATE_SETCLR(GATE_GPIOA,	RCC_MP_NS_AHB4ENSETR,	0),
	CFG_GATE_SETCLR(GATE_GPIOB,	RCC_MP_NS_AHB4ENSETR,	1),
	CFG_GATE_SETCLR(GATE_GPIOC,	RCC_MP_NS_AHB4ENSETR,	2),
	CFG_GATE_SETCLR(GATE_GPIOD,	RCC_MP_NS_AHB4ENSETR,	3),
	CFG_GATE_SETCLR(GATE_GPIOE,	RCC_MP_NS_AHB4ENSETR,	4),
	CFG_GATE_SETCLR(GATE_GPIOF,	RCC_MP_NS_AHB4ENSETR,	5),
	CFG_GATE_SETCLR(GATE_GPIOG,	RCC_MP_NS_AHB4ENSETR,	6),
	CFG_GATE_SETCLR(GATE_GPIOH,	RCC_MP_NS_AHB4ENSETR,	7),
	CFG_GATE_SETCLR(GATE_GPIOI,	RCC_MP_NS_AHB4ENSETR,	8),
	CFG_GATE_SETCLR(GATE_PKA,	RCC_MP_AHB5ENSETR,	2),
	CFG_GATE_SETCLR(GATE_SAES,	RCC_MP_AHB5ENSETR,	3),
	CFG_GATE_SETCLR(GATE_CRYP1,	RCC_MP_AHB5ENSETR,	4),
	CFG_GATE_SETCLR(GATE_HASH1,	RCC_MP_AHB5ENSETR,	5),
	CFG_GATE_SETCLR(GATE_RNG1,	RCC_MP_AHB5ENSETR,	6),
	CFG_GATE_SETCLR(GATE_BKPSRAM,	RCC_MP_AHB5ENSETR,	8),
	CFG_GATE_SETCLR(GATE_AXIMC,	RCC_MP_AHB5ENSETR,	16),
	CFG_GATE_SETCLR(GATE_MCE,	RCC_MP_AHB6ENSETR,	1),
	CFG_GATE_SETCLR(GATE_ETH1CK,	RCC_MP_AHB6ENSETR,	7),
	CFG_GATE_SETCLR(GATE_ETH1TX,	RCC_MP_AHB6ENSETR,	8),
	CFG_GATE_SETCLR(GATE_ETH1RX,	RCC_MP_AHB6ENSETR,	9),
	CFG_GATE_SETCLR(GATE_ETH1MAC,	RCC_MP_AHB6ENSETR,	10),
	CFG_GATE_SETCLR(GATE_FMC,	RCC_MP_AHB6ENSETR,	12),
	CFG_GATE_SETCLR(GATE_QSPI,	RCC_MP_AHB6ENSETR,	14),
	CFG_GATE_SETCLR(GATE_SDMMC1,	RCC_MP_AHB6ENSETR,	16),
	CFG_GATE_SETCLR(GATE_SDMMC2,	RCC_MP_AHB6ENSETR,	17),
	CFG_GATE_SETCLR(GATE_CRC1,	RCC_MP_AHB6ENSETR,	20),
	CFG_GATE_SETCLR(GATE_USBH,	RCC_MP_AHB6ENSETR,	24),
	CFG_GATE_SETCLR(GATE_ETH2CK,	RCC_MP_AHB6ENSETR,	27),
	CFG_GATE_SETCLR(GATE_ETH2TX,	RCC_MP_AHB6ENSETR,	28),
	CFG_GATE_SETCLR(GATE_ETH2RX,	RCC_MP_AHB6ENSETR,	29),
	CFG_GATE_SETCLR(GATE_ETH2MAC,	RCC_MP_AHB6ENSETR,	30),
	CFG_GATE_SETCLR(GATE_ETH1STP,	RCC_MP_AHB6LPENSETR,	11),
	CFG_GATE_SETCLR(GATE_ETH2STP,	RCC_MP_AHB6LPENSETR,	31),
	CFG_GATE_SETCLR(GATE_MDMA,	RCC_MP_NS_AHB6ENSETR,	0),
};

enum enum_mux_cfg {
	MUX_I2C12,
	MUX_LPTIM45,
	MUX_SPI23,
	MUX_UART35,
	MUX_UART78,
	MUX_ADC1,
	MUX_ADC2,
	MUX_DCMIPP,
	MUX_ETH1,
	MUX_ETH2,
	MUX_FDCAN,
	MUX_FMC,
	MUX_I2C3,
	MUX_I2C4,
	MUX_I2C5,
	MUX_LPTIM1,
	MUX_LPTIM2,
	MUX_LPTIM3,
	MUX_QSPI,
	MUX_RNG1,
	MUX_SAES,
	MUX_SAI1,
	MUX_SAI2,
	MUX_SDMMC1,
	MUX_SDMMC2,
	MUX_SPDIF,
	MUX_SPI1,
	MUX_SPI4,
	MUX_SPI5,
	MUX_STGEN,
	MUX_UART1,
	MUX_UART2,
	MUX_UART4,
	MUX_UART6,
	MUX_USBO,
	MUX_USBPHY,
	MUX_MCO1,
	MUX_MCO2,
	MUX_NB
};

#define _CFG_MUX(_id, _offset, _shift, _witdh, _ready, _flags)\
	[_id] = {\
		.offset		= (_offset),\
		.shift		= (_shift),\
		.width		= (_witdh),\
		.ready		= (_ready),\
		.flags		= (_flags),\
	}

#define CFG_MUX(_id, _offset, _shift, _witdh)\
	_CFG_MUX(_id, _offset, _shift, _witdh, MUX_NO_RDY, 0)

#define CFG_MUX_RDY(_id, _offset, _shift, _witdh, _ready)\
	_CFG_MUX(_id, _offset, _shift, _witdh, _ready, 0)

#define CFG_MUX_SAFE(_id, _offset, _shift, _witdh)\
	_CFG_MUX(_id, _offset, _shift, _witdh, MUX_NO_RDY, MUX_SAFE)

static const struct stm32_mux_cfg stm32mp13_muxes[] = {
	CFG_MUX(MUX_I2C12,	RCC_I2C12CKSELR,	0, 3),
	CFG_MUX(MUX_LPTIM45,	RCC_LPTIM45CKSELR,	0, 3),
	CFG_MUX(MUX_SPI23,	RCC_SPI2S23CKSELR,	0, 3),
	CFG_MUX(MUX_UART35,	RCC_UART35CKSELR,	0, 3),
	CFG_MUX(MUX_UART78,	RCC_UART78CKSELR,	0, 3),
	CFG_MUX(MUX_ADC1,	RCC_ADC12CKSELR,	0, 2),
	CFG_MUX(MUX_ADC2,	RCC_ADC12CKSELR,	2, 2),
	CFG_MUX(MUX_DCMIPP,	RCC_DCMIPPCKSELR,	0, 2),
	CFG_MUX(MUX_ETH1,	RCC_ETH12CKSELR,	0, 2),
	CFG_MUX(MUX_ETH2,	RCC_ETH12CKSELR,	8, 2),
	CFG_MUX(MUX_FDCAN,	RCC_FDCANCKSELR,	0, 2),
	CFG_MUX(MUX_I2C3,	RCC_I2C345CKSELR,	0, 3),
	CFG_MUX(MUX_I2C4,	RCC_I2C345CKSELR,	3, 3),
	CFG_MUX(MUX_I2C5,	RCC_I2C345CKSELR,	6, 3),
	CFG_MUX(MUX_LPTIM1,	RCC_LPTIM1CKSELR,	0, 3),
	CFG_MUX(MUX_LPTIM2,	RCC_LPTIM23CKSELR,	0, 3),
	CFG_MUX(MUX_LPTIM3,	RCC_LPTIM23CKSELR,	3, 3),
	CFG_MUX(MUX_MCO1,	RCC_MCO1CFGR,		0, 3),
	CFG_MUX(MUX_MCO2,	RCC_MCO2CFGR,		0, 3),
	CFG_MUX(MUX_RNG1,	RCC_RNG1CKSELR,		0, 2),
	CFG_MUX(MUX_SAES,	RCC_SAESCKSELR,		0, 2),
	CFG_MUX(MUX_SAI1,	RCC_SAI1CKSELR,		0, 3),
	CFG_MUX(MUX_SAI2,	RCC_SAI2CKSELR,		0, 3),
	CFG_MUX(MUX_SPDIF,	RCC_SPDIFCKSELR,	0, 2),
	CFG_MUX(MUX_SPI1,	RCC_SPI2S1CKSELR,	0, 3),
	CFG_MUX(MUX_SPI4,	RCC_SPI45CKSELR,	0, 3),
	CFG_MUX(MUX_SPI5,	RCC_SPI45CKSELR,	3, 3),
	CFG_MUX(MUX_STGEN,	RCC_STGENCKSELR,	0, 2),
	CFG_MUX(MUX_UART1,	RCC_UART12CKSELR,	0, 3),
	CFG_MUX(MUX_UART2,	RCC_UART12CKSELR,	3, 3),
	CFG_MUX(MUX_UART4,	RCC_UART4CKSELR,	0, 3),
	CFG_MUX(MUX_UART6,	RCC_UART6CKSELR,	0, 3),
	CFG_MUX(MUX_USBO,	RCC_USBCKSELR,		4, 1),
	CFG_MUX(MUX_USBPHY,	RCC_USBCKSELR,		0, 2),

	CFG_MUX_SAFE(MUX_FMC,	RCC_FMCCKSELR,		0, 2),
	CFG_MUX_SAFE(MUX_QSPI,	RCC_QSPICKSELR,		0, 2),
	CFG_MUX_SAFE(MUX_SDMMC1, RCC_SDMMC12CKSELR,	0, 3),
	CFG_MUX_SAFE(MUX_SDMMC2, RCC_SDMMC12CKSELR,	3, 3),
};

enum enum_div_cfg {
	DIV_MCO1,
	DIV_MCO2,
	DIV_TRACE,
	DIV_ETH1PTP,
	DIV_ETH2PTP,
	DIV_NB
};

static const struct clk_div_table ck_trace_div_table[] = {
	{ 0, 1 }, { 1, 2 }, { 2, 4 }, { 3, 8 },
	{ 4, 16 }, { 5, 16 }, { 6, 16 }, { 7, 16 },
	{ 0 },
};

#define CFG_DIV(_id, _offset, _shift, _width, _flags, _table, _ready)\
	[(_id)] = {\
		.offset	= (_offset),\
		.shift	= (_shift),\
		.width	= (_width),\
		.flags	= (_flags),\
		.table	= (_table),\
		.ready	= (_ready),\
	}

static const struct stm32_div_cfg stm32mp13_dividers[DIV_NB] = {
	CFG_DIV(DIV_MCO1, RCC_MCO1CFGR, 4, 4, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_MCO2, RCC_MCO2CFGR, 4, 4, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_TRACE, RCC_DBGCFGR, 0, 3, 0, ck_trace_div_table, DIV_NO_RDY),
	CFG_DIV(DIV_ETH1PTP, RCC_ETH12CKSELR, 4, 4, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_ETH2PTP, RCC_ETH12CKSELR, 12, 4, 0, NULL, DIV_NO_RDY),
};

struct clk_stm32_securiy {
	u16	offset;
	u8	bit_idx;
};

enum securit_clk {
	SECF_NONE,
	SECF_LPTIM2,
	SECF_LPTIM3,
	SECF_VREF,
	SECF_DCMIPP,
	SECF_USBPHY,
	SECF_RTC,
	SECF_TZC,
	SECF_ETZPC,
	SECF_IWDG1,
	SECF_BSEC,
	SECF_STGENC,
	SECF_STGENRO,
	SECF_USART1,
	SECF_USART2,
	SECF_SPI4,
	SECF_SPI5,
	SECF_I2C3,
	SECF_I2C4,
	SECF_I2C5,
	SECF_TIM12,
	SECF_TIM13,
	SECF_TIM14,
	SECF_TIM15,
	SECF_TIM16,
	SECF_TIM17,
	SECF_DMA3,
	SECF_DMAMUX2,
	SECF_ADC1,
	SECF_ADC2,
	SECF_USBO,
	SECF_TSC,
	SECF_PKA,
	SECF_SAES,
	SECF_CRYP1,
	SECF_HASH1,
	SECF_RNG1,
	SECF_BKPSRAM,
	SECF_MCE,
	SECF_FMC,
	SECF_QSPI,
	SECF_SDMMC1,
	SECF_SDMMC2,
	SECF_ETH1CK,
	SECF_ETH1TX,
	SECF_ETH1RX,
	SECF_ETH1MAC,
	SECF_ETH1STP,
	SECF_ETH2CK,
	SECF_ETH2TX,
	SECF_ETH2RX,
	SECF_ETH2MAC,
	SECF_ETH2STP,
	SECF_MCO1,
	SECF_MCO2
};

#define SECF(_sec_id, _offset, _bit_idx)[_sec_id] = {\
	.offset	= _offset,\
	.bit_idx	= _bit_idx,\
}

static const struct clk_stm32_securiy stm32mp13_security[] = {
	SECF(SECF_LPTIM2, RCC_APB3SECSR, RCC_APB3SECSR_LPTIM2SECF),
	SECF(SECF_LPTIM3, RCC_APB3SECSR, RCC_APB3SECSR_LPTIM3SECF),
	SECF(SECF_VREF, RCC_APB3SECSR, RCC_APB3SECSR_VREFSECF),
	SECF(SECF_DCMIPP, RCC_APB4SECSR, RCC_APB4SECSR_DCMIPPSECF),
	SECF(SECF_USBPHY, RCC_APB4SECSR, RCC_APB4SECSR_USBPHYSECF),
	SECF(SECF_RTC, RCC_APB5SECSR, RCC_APB5SECSR_RTCSECF),
	SECF(SECF_TZC, RCC_APB5SECSR, RCC_APB5SECSR_TZCSECF),
	SECF(SECF_ETZPC, RCC_APB5SECSR, RCC_APB5SECSR_ETZPCSECF),
	SECF(SECF_IWDG1, RCC_APB5SECSR, RCC_APB5SECSR_IWDG1SECF),
	SECF(SECF_BSEC, RCC_APB5SECSR, RCC_APB5SECSR_BSECSECF),
	SECF(SECF_STGENC, RCC_APB5SECSR, RCC_APB5SECSR_STGENCSECF),
	SECF(SECF_STGENRO, RCC_APB5SECSR, RCC_APB5SECSR_STGENROSECF),
	SECF(SECF_USART1, RCC_APB6SECSR, RCC_APB6SECSR_USART1SECF),
	SECF(SECF_USART2, RCC_APB6SECSR, RCC_APB6SECSR_USART2SECF),
	SECF(SECF_SPI4, RCC_APB6SECSR, RCC_APB6SECSR_SPI4SECF),
	SECF(SECF_SPI5, RCC_APB6SECSR, RCC_APB6SECSR_SPI5SECF),
	SECF(SECF_I2C3, RCC_APB6SECSR, RCC_APB6SECSR_I2C3SECF),
	SECF(SECF_I2C4, RCC_APB6SECSR, RCC_APB6SECSR_I2C4SECF),
	SECF(SECF_I2C5, RCC_APB6SECSR, RCC_APB6SECSR_I2C5SECF),
	SECF(SECF_TIM12, RCC_APB6SECSR, RCC_APB6SECSR_TIM12SECF),
	SECF(SECF_TIM13, RCC_APB6SECSR, RCC_APB6SECSR_TIM13SECF),
	SECF(SECF_TIM14, RCC_APB6SECSR, RCC_APB6SECSR_TIM14SECF),
	SECF(SECF_TIM15, RCC_APB6SECSR, RCC_APB6SECSR_TIM15SECF),
	SECF(SECF_TIM16, RCC_APB6SECSR, RCC_APB6SECSR_TIM16SECF),
	SECF(SECF_TIM17, RCC_APB6SECSR, RCC_APB6SECSR_TIM17SECF),
	SECF(SECF_DMA3, RCC_AHB2SECSR, RCC_AHB2SECSR_DMA3SECF),
	SECF(SECF_DMAMUX2, RCC_AHB2SECSR, RCC_AHB2SECSR_DMAMUX2SECF),
	SECF(SECF_ADC1, RCC_AHB2SECSR, RCC_AHB2SECSR_ADC1SECF),
	SECF(SECF_ADC2, RCC_AHB2SECSR, RCC_AHB2SECSR_ADC2SECF),
	SECF(SECF_USBO, RCC_AHB2SECSR, RCC_AHB2SECSR_USBOSECF),
	SECF(SECF_TSC, RCC_AHB4SECSR, RCC_AHB4SECSR_TSCSECF),
	SECF(SECF_PKA, RCC_AHB5SECSR, RCC_AHB5SECSR_PKASECF),
	SECF(SECF_SAES, RCC_AHB5SECSR, RCC_AHB5SECSR_SAESSECF),
	SECF(SECF_CRYP1, RCC_AHB5SECSR, RCC_AHB5SECSR_CRYP1SECF),
	SECF(SECF_HASH1, RCC_AHB5SECSR, RCC_AHB5SECSR_HASH1SECF),
	SECF(SECF_RNG1, RCC_AHB5SECSR, RCC_AHB5SECSR_RNG1SECF),
	SECF(SECF_BKPSRAM, RCC_AHB5SECSR, RCC_AHB5SECSR_BKPSRAMSECF),
	SECF(SECF_MCE, RCC_AHB6SECSR, RCC_AHB6SECSR_MCESECF),
	SECF(SECF_FMC, RCC_AHB6SECSR, RCC_AHB6SECSR_FMCSECF),
	SECF(SECF_QSPI, RCC_AHB6SECSR, RCC_AHB6SECSR_QSPISECF),
	SECF(SECF_SDMMC1, RCC_AHB6SECSR, RCC_AHB6SECSR_SDMMC1SECF),
	SECF(SECF_SDMMC2, RCC_AHB6SECSR, RCC_AHB6SECSR_SDMMC2SECF),
	SECF(SECF_ETH1CK, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH1CKSECF),
	SECF(SECF_ETH1TX, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH1TXSECF),
	SECF(SECF_ETH1RX, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH1RXSECF),
	SECF(SECF_ETH1MAC, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH1MACSECF),
	SECF(SECF_ETH1STP, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH1STPSECF),
	SECF(SECF_ETH2CK, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH2CKSECF),
	SECF(SECF_ETH2TX, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH2TXSECF),
	SECF(SECF_ETH2RX, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH2RXSECF),
	SECF(SECF_ETH2MAC, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH2MACSECF),
	SECF(SECF_ETH2STP, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH2STPSECF),
	SECF(SECF_MCO1, RCC_SECCFGR, RCC_SECCFGR_MCO1SECF),
	SECF(SECF_MCO2, RCC_SECCFGR, RCC_SECCFGR_MCO2SECF),
};

#define CLK_KER(_name, _parents, _flags, _gate_id, _mux_id)\
	CLK_STM32_COMPOSITE(_name, _parents, ((_flags) | CLK_OPS_PARENT_ENABLE |\
			    CLK_SET_RATE_NO_REPARENT), _gate_id, _mux_id, NO_STM32_DIV)

static CLK_STM32_GATE(tim2_k, "timg1_ck", CLK_SET_RATE_PARENT, GATE_TIM2);
static CLK_STM32_GATE(tim3_k, "timg1_ck", CLK_SET_RATE_PARENT, GATE_TIM3);
static CLK_STM32_GATE(tim4_k, "timg1_ck", CLK_SET_RATE_PARENT, GATE_TIM4);
static CLK_STM32_GATE(tim5_k, "timg1_ck", CLK_SET_RATE_PARENT, GATE_TIM5);
static CLK_STM32_GATE(tim6_k, "timg1_ck", CLK_SET_RATE_PARENT, GATE_TIM6);
static CLK_STM32_GATE(tim7_k, "timg1_ck", CLK_SET_RATE_PARENT, GATE_TIM7);
static CLK_STM32_GATE(tim1_k, "timg2_ck", CLK_SET_RATE_PARENT, GATE_TIM1);
static CLK_STM32_GATE(tim8_k, "timg2_ck", CLK_SET_RATE_PARENT, GATE_TIM8);
static CLK_STM32_GATE(tim12_k, "timg3_ck", CLK_SET_RATE_PARENT, GATE_TIM12);
static CLK_STM32_GATE(tim13_k, "timg3_ck", CLK_SET_RATE_PARENT, GATE_TIM13);
static CLK_STM32_GATE(tim14_k, "timg3_ck", CLK_SET_RATE_PARENT, GATE_TIM14);
static CLK_STM32_GATE(tim15_k, "timg3_ck", CLK_SET_RATE_PARENT, GATE_TIM15);
static CLK_STM32_GATE(tim16_k, "timg3_ck", CLK_SET_RATE_PARENT, GATE_TIM16);
static CLK_STM32_GATE(tim17_k, "timg3_ck", CLK_SET_RATE_PARENT, GATE_TIM17);

static CLK_STM32_GATE(sai1, "pclk2", 0, GATE_SAI1);
static CLK_STM32_GATE(sai2, "pclk2", 0, GATE_SAI2);

static CLK_STM32_GATE(syscfg, "pclk3", 0, GATE_SYSCFG);
static CLK_STM32_GATE(vref, "pclk3", 0, GATE_VREF);
static CLK_STM32_GATE(dts, "pclk3", 0, GATE_DTS);
static CLK_STM32_GATE(pmbctrl, "pclk3", 0, GATE_PMBCTRL);
static CLK_STM32_GATE(hdp, "pclk3", 0, GATE_HDP);

static CLK_STM32_GATE(iwdg2, "pclk4", 0, GATE_IWDG2APB);
static CLK_STM32_GATE(stgenro, "pclk4", 0, GATE_STGENRO);
static CLK_STM32_GATE(gpioa, "pclk4", 0, GATE_GPIOA);
static CLK_STM32_GATE(gpiob, "pclk4", 0, GATE_GPIOB);
static CLK_STM32_GATE(gpioc, "pclk4", 0, GATE_GPIOC);
static CLK_STM32_GATE(gpiod, "pclk4", 0, GATE_GPIOD);
static CLK_STM32_GATE(gpioe, "pclk4", 0, GATE_GPIOE);
static CLK_STM32_GATE(gpiof, "pclk4", 0, GATE_GPIOF);
static CLK_STM32_GATE(gpiog, "pclk4", 0, GATE_GPIOG);
static CLK_STM32_GATE(gpioh, "pclk4", 0, GATE_GPIOH);
static CLK_STM32_GATE(gpioi, "pclk4", 0, GATE_GPIOI);
static CLK_STM32_GATE(tsc, "pclk4", 0, GATE_TSC);
static CLK_STM32_GATE(ddrperfm, "pclk4", 0, GATE_DDRPERFM);

static CLK_STM32_GATE(tzpc, "pclk5", 0, GATE_TZC);
static CLK_STM32_GATE(iwdg1, "pclk5", 0, GATE_IWDG1APB);
static CLK_STM32_GATE(bsec, "pclk5", 0, GATE_BSEC);

static CLK_STM32_GATE(dma1, "ck_mlahb", 0, GATE_DMA1);
static CLK_STM32_GATE(dma2, "ck_mlahb",  0, GATE_DMA2);
static CLK_STM32_GATE(dmamux1, "ck_mlahb", 0, GATE_DMAMUX1);
static CLK_STM32_GATE(dma3, "ck_mlahb", 0, GATE_DMA3);
static CLK_STM32_GATE(dmamux2, "ck_mlahb", 0, GATE_DMAMUX2);
static CLK_STM32_GATE(adc1, "ck_mlahb", 0, GATE_ADC1);
static CLK_STM32_GATE(adc2, "ck_mlahb", 0, GATE_ADC2);

static CLK_STM32_GATE(pka, "ck_axi", 0, GATE_PKA);
static CLK_STM32_GATE(cryp1, "ck_axi", 0, GATE_CRYP1);
static CLK_STM32_GATE(hash1, "ck_axi", 0, GATE_HASH1);
static CLK_STM32_GATE(bkpsram, "ck_axi", 0, GATE_BKPSRAM);
static CLK_STM32_GATE(mdma, "ck_axi", 0, GATE_MDMA);
static CLK_STM32_GATE(eth1tx, "ck_axi", 0, GATE_ETH1TX);
static CLK_STM32_GATE(eth1rx, "ck_axi", 0, GATE_ETH1RX);
static CLK_STM32_GATE(eth1mac, "ck_axi", 0, GATE_ETH1MAC);
static CLK_STM32_GATE(eth2tx, "ck_axi", 0, GATE_ETH2TX);
static CLK_STM32_GATE(eth2rx, "ck_axi", 0, GATE_ETH2RX);
static CLK_STM32_GATE(eth2mac, "ck_axi", 0, GATE_ETH2MAC);
static CLK_STM32_GATE(crc1, "ck_axi", 0, GATE_CRC1);
static CLK_STM32_GATE(usbh, "ck_axi", 0, GATE_USBH);
static CLK_STM32_GATE(eth1stp, "ck_axi", 0, GATE_ETH1STP);
static CLK_STM32_GATE(eth2stp, "ck_axi", 0, GATE_ETH2STP);

static CLK_KER(sdmmc1_k, sdmmc12_src, 0, GATE_SDMMC1, MUX_SDMMC1);
static CLK_KER(sdmmc2_k, sdmmc12_src, 0, GATE_SDMMC2, MUX_SDMMC2);
static CLK_KER(fmc_k, fmc_src, 0, GATE_FMC, MUX_FMC);
static CLK_KER(qspi_k, qspi_src, 0, GATE_QSPI, MUX_QSPI);
static CLK_KER(spi2_k, spi123_src, 0, GATE_SPI2, MUX_SPI23);
static CLK_KER(spi3_k, spi123_src, 0, GATE_SPI3, MUX_SPI23);
static CLK_KER(i2c1_k, i2c12_src, 0, GATE_I2C1, MUX_I2C12);
static CLK_KER(i2c2_k, i2c12_src, 0, GATE_I2C2, MUX_I2C12);
static CLK_KER(lptim4_k, lptim45_src, 0, GATE_LPTIM4, MUX_LPTIM45);
static CLK_KER(lptim5_k, lptim45_src, 0, GATE_LPTIM5, MUX_LPTIM45);
static CLK_KER(usart3_k, usart34578_src, 0, GATE_USART3, MUX_UART35);
static CLK_KER(uart5_k, usart34578_src, 0, GATE_UART5, MUX_UART35);
static CLK_KER(uart7_k, usart34578_src, 0, GATE_UART7, MUX_UART78);
static CLK_KER(uart8_k, usart34578_src, 0, GATE_UART8, MUX_UART78);
static CLK_KER(sai1_k, sai1_src, 0, GATE_SAI1, MUX_SAI1);
static CLK_KER(adfsdm_k, sai1_src, 0, GATE_ADFSDM, MUX_SAI1);
static CLK_KER(sai2_k, sai2_src, 0, GATE_SAI2, MUX_SAI2);
static CLK_KER(adc1_k, adc12_src, 0, GATE_ADC1, MUX_ADC1);
static CLK_KER(adc2_k, adc12_src, 0, GATE_ADC2, MUX_ADC2);
static CLK_KER(rng1_k, rng1_src, 0, GATE_RNG1, MUX_RNG1);
static CLK_KER(usbphy_k, usbphy_src, 0, GATE_USBPHY, MUX_USBPHY);
static CLK_KER(stgen_k, stgen_src, 0, GATE_STGENC, MUX_STGEN);
static CLK_KER(spdif_k, spdif_src, 0, GATE_SPDIF, MUX_SPDIF);
static CLK_KER(spi1_k, spi123_src, 0, GATE_SPI1, MUX_SPI1);
static CLK_KER(spi4_k, spi4_src, 0, GATE_SPI4, MUX_SPI4);
static CLK_KER(spi5_k, spi5_src, 0, GATE_SPI5, MUX_SPI5);
static CLK_KER(i2c3_k, i2c345_src, 0, GATE_I2C3, MUX_I2C3);
static CLK_KER(i2c4_k, i2c345_src, 0, GATE_I2C4, MUX_I2C4);
static CLK_KER(i2c5_k, i2c345_src, 0, GATE_I2C5, MUX_I2C5);
static CLK_KER(lptim1_k, lptim1_src, 0, GATE_LPTIM1, MUX_LPTIM1);
static CLK_KER(lptim2_k, lptim23_src, 0, GATE_LPTIM2, MUX_LPTIM2);
static CLK_KER(lptim3_k, lptim23_src, 0, GATE_LPTIM3, MUX_LPTIM3);
static CLK_KER(usart1_k, usart12_src, 0, GATE_USART1, MUX_UART1);
static CLK_KER(usart2_k, usart12_src, 0, GATE_USART2, MUX_UART2);
static CLK_KER(uart4_k, usart34578_src, 0, GATE_UART4, MUX_UART4);
static CLK_KER(uart6_k, usart6_src, 0, GATE_USART6, MUX_UART6);
static CLK_KER(fdcan_k, fdcan_src, 0, GATE_FDCAN, MUX_FDCAN);
static CLK_KER(dcmipp_k, dcmipp_src, 0, GATE_DCMIPP, MUX_DCMIPP);
static CLK_KER(usbo_k, usbo_src, 0, GATE_USBO, MUX_USBO);
static CLK_KER(eth1ck_k, eth12_src, 0, GATE_ETH1CK, MUX_ETH1);
static CLK_KER(eth2ck_k, eth12_src, 0, GATE_ETH2CK, MUX_ETH2);
static CLK_KER(saes_k, saes_src, 0, GATE_SAES, MUX_SAES);

static CLK_STM32_GATE(dfsdm_k, "ck_mlahb", 0, GATE_DFSDM);
static CLK_STM32_GATE(ltdc_px, "pll4_q", CLK_SET_RATE_PARENT, GATE_LTDC);

static CLK_STM32_COMPOSITE(eth1ptp_k, eth12_src, CLK_OPS_PARENT_ENABLE |
			   CLK_SET_RATE_NO_REPARENT,
			   NO_STM32_GATE, MUX_ETH1, DIV_ETH1PTP);

static CLK_STM32_COMPOSITE(eth2ptp_k, eth12_src, CLK_OPS_PARENT_ENABLE |
			   CLK_SET_RATE_NO_REPARENT,
			   NO_STM32_GATE, MUX_ETH2, DIV_ETH2PTP);

/* MCO clocks */
static CLK_STM32_COMPOSITE(ck_mco1, mco1_src, CLK_OPS_PARENT_ENABLE |
			   CLK_SET_RATE_NO_REPARENT | CLK_IGNORE_UNUSED,
			   GATE_MCO1, MUX_MCO1, DIV_MCO1);

static CLK_STM32_COMPOSITE(ck_mco2, mco2_src, CLK_OPS_PARENT_ENABLE |
			   CLK_SET_RATE_NO_REPARENT | CLK_IGNORE_UNUSED,
			   GATE_MCO2, MUX_MCO2, DIV_MCO2);

/* Debug clocks */
static CLK_STM32_GATE(ck_sys_dbg, "ck_axi", CLK_IS_CRITICAL, GATE_DBGCK);

static CLK_STM32_COMPOSITE(ck_trace, PARENT("ck_axi"), CLK_IGNORE_UNUSED,
			   GATE_TRACECK, NO_STM32_MUX, DIV_TRACE);

static const struct clock_config stm32mp13_clock_cfg[] = {
	/* Timer clocks */
	STM32_GATE_CFG(TIM2_K, tim2_k, SECF_NONE),
	STM32_GATE_CFG(TIM3_K, tim3_k, SECF_NONE),
	STM32_GATE_CFG(TIM4_K, tim4_k, SECF_NONE),
	STM32_GATE_CFG(TIM5_K, tim5_k, SECF_NONE),
	STM32_GATE_CFG(TIM6_K, tim6_k, SECF_NONE),
	STM32_GATE_CFG(TIM7_K, tim7_k, SECF_NONE),
	STM32_GATE_CFG(TIM1_K, tim1_k, SECF_NONE),
	STM32_GATE_CFG(TIM8_K, tim8_k, SECF_NONE),
	STM32_GATE_CFG(TIM12_K, tim12_k, SECF_TIM12),
	STM32_GATE_CFG(TIM13_K, tim13_k, SECF_TIM13),
	STM32_GATE_CFG(TIM14_K, tim14_k, SECF_TIM14),
	STM32_GATE_CFG(TIM15_K, tim15_k, SECF_TIM15),
	STM32_GATE_CFG(TIM16_K, tim16_k, SECF_TIM16),
	STM32_GATE_CFG(TIM17_K, tim17_k, SECF_TIM17),

	/* Peripheral clocks */
	STM32_GATE_CFG(SAI1, sai1, SECF_NONE),
	STM32_GATE_CFG(SAI2, sai2, SECF_NONE),
	STM32_GATE_CFG(SYSCFG, syscfg, SECF_NONE),
	STM32_GATE_CFG(VREF, vref, SECF_VREF),
	STM32_GATE_CFG(DTS, dts, SECF_NONE),
	STM32_GATE_CFG(PMBCTRL, pmbctrl, SECF_NONE),
	STM32_GATE_CFG(HDP, hdp, SECF_NONE),
	STM32_GATE_CFG(IWDG2, iwdg2, SECF_NONE),
	STM32_GATE_CFG(STGENRO, stgenro, SECF_STGENRO),
	STM32_GATE_CFG(TZPC, tzpc, SECF_TZC),
	STM32_GATE_CFG(IWDG1, iwdg1, SECF_IWDG1),
	STM32_GATE_CFG(BSEC, bsec, SECF_BSEC),
	STM32_GATE_CFG(DMA1, dma1, SECF_NONE),
	STM32_GATE_CFG(DMA2, dma2, SECF_NONE),
	STM32_GATE_CFG(DMAMUX1, dmamux1, SECF_NONE),
	STM32_GATE_CFG(DMA3, dma3, SECF_DMA3),
	STM32_GATE_CFG(DMAMUX2, dmamux2, SECF_DMAMUX2),
	STM32_GATE_CFG(ADC1, adc1, SECF_ADC1),
	STM32_GATE_CFG(ADC2, adc2, SECF_ADC2),
	STM32_GATE_CFG(GPIOA, gpioa, SECF_NONE),
	STM32_GATE_CFG(GPIOB, gpiob, SECF_NONE),
	STM32_GATE_CFG(GPIOC, gpioc, SECF_NONE),
	STM32_GATE_CFG(GPIOD, gpiod, SECF_NONE),
	STM32_GATE_CFG(GPIOE, gpioe, SECF_NONE),
	STM32_GATE_CFG(GPIOF, gpiof, SECF_NONE),
	STM32_GATE_CFG(GPIOG, gpiog, SECF_NONE),
	STM32_GATE_CFG(GPIOH, gpioh, SECF_NONE),
	STM32_GATE_CFG(GPIOI, gpioi, SECF_NONE),
	STM32_GATE_CFG(TSC, tsc, SECF_TZC),
	STM32_GATE_CFG(PKA, pka, SECF_PKA),
	STM32_GATE_CFG(CRYP1, cryp1, SECF_CRYP1),
	STM32_GATE_CFG(HASH1, hash1, SECF_HASH1),
	STM32_GATE_CFG(BKPSRAM, bkpsram, SECF_BKPSRAM),
	STM32_GATE_CFG(MDMA, mdma, SECF_NONE),
	STM32_GATE_CFG(ETH1TX, eth1tx, SECF_ETH1TX),
	STM32_GATE_CFG(ETH1RX, eth1rx, SECF_ETH1RX),
	STM32_GATE_CFG(ETH1MAC, eth1mac, SECF_ETH1MAC),
	STM32_GATE_CFG(ETH2TX, eth2tx, SECF_ETH2TX),
	STM32_GATE_CFG(ETH2RX, eth2rx, SECF_ETH2RX),
	STM32_GATE_CFG(ETH2MAC, eth2mac, SECF_ETH2MAC),
	STM32_GATE_CFG(CRC1, crc1, SECF_NONE),
	STM32_GATE_CFG(USBH, usbh, SECF_NONE),
	STM32_GATE_CFG(DDRPERFM, ddrperfm, SECF_NONE),
	STM32_GATE_CFG(ETH1STP, eth1stp, SECF_ETH1STP),
	STM32_GATE_CFG(ETH2STP, eth2stp, SECF_ETH2STP),

	/* Kernel clocks */
	STM32_COMPOSITE_CFG(SDMMC1_K, sdmmc1_k, SECF_SDMMC1),
	STM32_COMPOSITE_CFG(SDMMC2_K, sdmmc2_k, SECF_SDMMC2),
	STM32_COMPOSITE_CFG(FMC_K, fmc_k, SECF_FMC),
	STM32_COMPOSITE_CFG(QSPI_K, qspi_k, SECF_QSPI),
	STM32_COMPOSITE_CFG(SPI2_K, spi2_k, SECF_NONE),
	STM32_COMPOSITE_CFG(SPI3_K, spi3_k, SECF_NONE),
	STM32_COMPOSITE_CFG(I2C1_K, i2c1_k, SECF_NONE),
	STM32_COMPOSITE_CFG(I2C2_K, i2c2_k, SECF_NONE),
	STM32_COMPOSITE_CFG(LPTIM4_K, lptim4_k, SECF_NONE),
	STM32_COMPOSITE_CFG(LPTIM5_K, lptim5_k, SECF_NONE),
	STM32_COMPOSITE_CFG(USART3_K, usart3_k, SECF_NONE),
	STM32_COMPOSITE_CFG(UART5_K, uart5_k, SECF_NONE),
	STM32_COMPOSITE_CFG(UART7_K, uart7_k, SECF_NONE),
	STM32_COMPOSITE_CFG(UART8_K, uart8_k, SECF_NONE),
	STM32_COMPOSITE_CFG(SAI1_K, sai1_k, SECF_NONE),
	STM32_COMPOSITE_CFG(SAI2_K, sai2_k, SECF_NONE),
	STM32_COMPOSITE_CFG(ADFSDM_K, adfsdm_k, SECF_NONE),
	STM32_COMPOSITE_CFG(ADC1_K, adc1_k, SECF_ADC1),
	STM32_COMPOSITE_CFG(ADC2_K, adc2_k, SECF_ADC2),
	STM32_COMPOSITE_CFG(RNG1_K, rng1_k, SECF_RNG1),
	STM32_COMPOSITE_CFG(USBPHY_K, usbphy_k, SECF_USBPHY),
	STM32_COMPOSITE_CFG(STGEN_K, stgen_k, SECF_STGENC),
	STM32_COMPOSITE_CFG(SPDIF_K, spdif_k, SECF_NONE),
	STM32_COMPOSITE_CFG(SPI1_K, spi1_k, SECF_NONE),
	STM32_COMPOSITE_CFG(SPI4_K, spi4_k, SECF_SPI4),
	STM32_COMPOSITE_CFG(SPI5_K, spi5_k, SECF_SPI5),
	STM32_COMPOSITE_CFG(I2C3_K, i2c3_k, SECF_I2C3),
	STM32_COMPOSITE_CFG(I2C4_K, i2c4_k, SECF_I2C4),
	STM32_COMPOSITE_CFG(I2C5_K, i2c5_k, SECF_I2C5),
	STM32_COMPOSITE_CFG(LPTIM1_K, lptim1_k, SECF_NONE),
	STM32_COMPOSITE_CFG(LPTIM2_K, lptim2_k, SECF_LPTIM2),
	STM32_COMPOSITE_CFG(LPTIM3_K, lptim3_k, SECF_LPTIM3),
	STM32_COMPOSITE_CFG(USART1_K, usart1_k, SECF_USART1),
	STM32_COMPOSITE_CFG(USART2_K, usart2_k, SECF_USART2),
	STM32_COMPOSITE_CFG(UART4_K, uart4_k, SECF_NONE),
	STM32_COMPOSITE_CFG(USART6_K, uart6_k, SECF_NONE),
	STM32_COMPOSITE_CFG(FDCAN_K, fdcan_k, SECF_NONE),
	STM32_COMPOSITE_CFG(DCMIPP_K, dcmipp_k, SECF_DCMIPP),
	STM32_COMPOSITE_CFG(USBO_K, usbo_k, SECF_USBO),
	STM32_COMPOSITE_CFG(ETH1CK_K, eth1ck_k, SECF_ETH1CK),
	STM32_COMPOSITE_CFG(ETH2CK_K, eth2ck_k, SECF_ETH2CK),
	STM32_COMPOSITE_CFG(SAES_K, saes_k, SECF_SAES),

	STM32_GATE_CFG(DFSDM_K, dfsdm_k, SECF_NONE),
	STM32_GATE_CFG(LTDC_PX, ltdc_px, SECF_NONE),

	STM32_COMPOSITE_CFG(ETH1PTP_K, eth1ptp_k, SECF_ETH1CK),
	STM32_COMPOSITE_CFG(ETH2PTP_K, eth2ptp_k, SECF_ETH2CK),
	STM32_COMPOSITE_CFG(CK_MCO1, ck_mco1, SECF_MCO1),
	STM32_COMPOSITE_CFG(CK_MCO2, ck_mco2, SECF_MCO2),

	STM32_GATE_CFG(CK_DBG, ck_sys_dbg, SECF_NONE),

	STM32_COMPOSITE_CFG(CK_TRACE, ck_trace, SECF_NONE),
};

static int stm32mp13_check_security(void __iomem *base,
				    const struct clock_config *cfg)
{
	int sec_id = cfg->sec_id;
	int secured = 0;

	if (sec_id != SECF_NONE) {
		const struct clk_stm32_securiy *secf;

		secf = &stm32mp13_security[sec_id];
		secured = !!(readl(base + secf->offset) & BIT(secf->bit_idx));
	}

	return secured;
}

struct multi_mux {
	struct clk_hw *hw1;
	struct clk_hw *hw2;
};

static struct multi_mux *stm32_mp13_multi_mux[MUX_NB] = {
	[MUX_SPI23]	= &(struct multi_mux){ &spi2_k.hw,	&spi3_k.hw },
	[MUX_I2C12]	= &(struct multi_mux){ &i2c1_k.hw,	&i2c2_k.hw },
	[MUX_LPTIM45]	= &(struct multi_mux){ &lptim4_k.hw,	&lptim5_k.hw },
	[MUX_UART35]	= &(struct multi_mux){ &usart3_k.hw,	&uart5_k.hw },
	[MUX_UART78]	= &(struct multi_mux){ &uart7_k.hw,	&uart8_k.hw },
	[MUX_SAI1]	= &(struct multi_mux){ &sai1_k.hw,	&adfsdm_k.hw },
};

static struct clk_hw *clk_stm32_is_multi_mux(struct clk_hw *hw)
{
	struct clk_stm32_composite *composite = to_clk_stm32_composite(hw);
	struct multi_mux *mmux = stm32_mp13_multi_mux[composite->mux_id];

	if (mmux) {
		if (!(mmux->hw1 == hw))
			return mmux->hw1;
		else
			return mmux->hw2;
	}

	return NULL;
}

u16 stm32mp13_cpt_gate[GATE_NB];

struct clk_stm32_clock_data stm32mp13_clock_data = {
	.gate_cpt	= stm32mp13_cpt_gate,
	.gates		= stm32mp13_gates,
	.muxes		= stm32mp13_muxes,
	.dividers	= stm32mp13_dividers,
	.is_multi_mux	= clk_stm32_is_multi_mux,
};

static const struct stm32_rcc_match_data stm32mp13_data = {
	.tab_clocks	= stm32mp13_clock_cfg,
	.num_clocks	= ARRAY_SIZE(stm32mp13_clock_cfg),
	.clock_data	= &stm32mp13_clock_data,
	.check_security = &stm32mp13_check_security,
	.maxbinding	= STM32MP1_LAST_CLK,
	.clear_offset	= RCC_CLR,
};

static const struct of_device_id stm32mp13_match_data[] = {
	{
		.compatible = "st,stm32mp13-rcc",
		.data = &stm32mp13_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, stm32mp13_match_data);

static int stm32mp1_rcc_init(struct device *dev)
{
	void __iomem *rcc_base;
	int ret = -ENOMEM;

	rcc_base = of_iomap(dev_of_node(dev), 0);
	if (!rcc_base) {
		dev_err(dev, "%pOFn: unable to map resource", dev_of_node(dev));
		goto out;
	}

	ret = stm32_rcc_init(dev, stm32mp13_match_data, rcc_base);
out:
	if (ret) {
		if (rcc_base)
			iounmap(rcc_base);

		of_node_put(dev_of_node(dev));
	}

	return ret;
}

static int get_clock_deps(struct device *dev)
{
	static const char * const clock_deps_name[] = {
		"hsi", "hse", "csi", "lsi", "lse",
	};
	size_t deps_size = sizeof(struct clk *) * ARRAY_SIZE(clock_deps_name);
	struct clk **clk_deps;
	int i;

	clk_deps = devm_kzalloc(dev, deps_size, GFP_KERNEL);
	if (!clk_deps)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(clock_deps_name); i++) {
		struct clk *clk = of_clk_get_by_name(dev_of_node(dev),
						     clock_deps_name[i]);

		if (IS_ERR(clk)) {
			if (PTR_ERR(clk) != -EINVAL && PTR_ERR(clk) != -ENOENT)
				return PTR_ERR(clk);
		} else {
			/* Device gets a reference count on the clock */
			clk_deps[i] = devm_clk_get(dev, __clk_get_name(clk));
			clk_put(clk);
		}
	}

	return 0;
}

static int stm32mp1_rcc_clocks_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = get_clock_deps(dev);

	if (!ret)
		ret = stm32mp1_rcc_init(dev);

	return ret;
}

static int stm32mp1_rcc_clocks_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *child, *np = dev_of_node(dev);

	for_each_available_child_of_node(np, child)
		of_clk_del_provider(child);

	return 0;
}

static struct platform_driver stm32mp13_rcc_clocks_driver = {
	.driver	= {
		.name = "stm32mp13_rcc",
		.of_match_table = stm32mp13_match_data,
	},
	.probe = stm32mp1_rcc_clocks_probe,
	.remove = stm32mp1_rcc_clocks_remove,
};

static int __init stm32mp13_clocks_init(void)
{
	return platform_driver_register(&stm32mp13_rcc_clocks_driver);
}
core_initcall(stm32mp13_clocks_init);
