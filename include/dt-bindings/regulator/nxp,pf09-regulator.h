/* SPDX-License-Identifier: (GPL-2.0 OR BSD-2-Clause) */
/*
 * Copyright (C) 2025, Digi International Inc.
 */

#ifndef __DT_BINDINGS_REGULATOR_NXP_PF09_REGULATOR_H
#define __DT_BINDINGS_REGULATOR_NXP_PF09_REGULATOR_H

/* SCMI voltage domains identifiers */

/* Number og SoC regulators (VDD_ARM and VDD_SOC) */
#define DEV_SM_NUM_VOLT			2

/* PF09 regulators */
#define VOLTD_SCMI_PF09_SW1	(DEV_SM_NUM_VOLT + 0U)
#define VOLTD_SCMI_PF09_SW2	(DEV_SM_NUM_VOLT + 1U)
#define VOLTD_SCMI_PF09_SW3	(DEV_SM_NUM_VOLT + 2U)
#define VOLTD_SCMI_PF09_SW4	(DEV_SM_NUM_VOLT + 3U)
#define VOLTD_SCMI_PF09_SW5	(DEV_SM_NUM_VOLT + 4U)
#define VOLTD_SCMI_PF09_LDO1	(DEV_SM_NUM_VOLT + 5U)
#define VOLTD_SCMI_PF09_LDO2	(DEV_SM_NUM_VOLT + 6U)
#define VOLTD_SCMI_PF09_LDO3	(DEV_SM_NUM_VOLT + 7U)

#endif /*__DT_BINDINGS_REGULATOR_NXP_PF09_REGULATOR_H */
