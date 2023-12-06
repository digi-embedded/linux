/* SPDX-License-Identifier: GPL-2.0  */
/*
 * Copyright (C) STMicroelectronics 2022 - All Rights Reserved
 * Author: Gabriel Fernandez <gabriel.fernandez@foss.st.com> for STMicroelectronics.
 */

#include <linux/clk-provider.h>

struct stm32_rcc_match_data;

struct stm32_mux_cfg {
	u16	offset;
	u8	shift;
	u8	width;
	u8	flags;
	u32	*table;
	u8	ready;
};

struct stm32_gate_cfg {
	u16	offset;
	u8	bit_idx;
	u8	set_clr;
};

struct stm32_div_cfg {
	u16	offset;
	u8	shift;
	u8	width;
	u8	flags;
	u8	ready;
	const struct clk_div_table *table;
};

struct stm32_composite_cfg {
	int	mux;
	int	gate;
	int	div;
};

#define NO_ID 0xFFFFFFFF

#define NO_STM32_MUX		0xFFFF
#define NO_STM32_DIV		0xFFFF
#define NO_STM32_GATE		0xFFFF

struct clock_config {
	unsigned long	id;
	int		sec_id;
	void		*clock_cfg;

	struct clk_hw *(*func)(struct device *dev,
			       const struct stm32_rcc_match_data *data,
			       void __iomem *base,
			       spinlock_t *lock,
			       const struct clock_config *cfg);
};

struct clk_stm32_clock_data {
	void __iomem			*base;
	u16 *gate_cpt;
	const struct stm32_gate_cfg	*gates;
	const struct stm32_mux_cfg	*muxes;
	const struct stm32_div_cfg	*dividers;
	struct clk_hw *(*is_multi_mux)(struct clk_hw *hw);
};

struct clock_summary {
	struct clk_summary *clocks;
	int nb_clocks;
};

struct clk_summary {
	const char *name;
	unsigned long rate;
	int enabled;
	int nb_parents;
	int gate_id;
	int mux_id;
	int div_id;
	void *data;

	bool (*is_enabled)(struct clk_stm32_clock_data *data,
			   struct clk_summary *c);
	u8 (*get_parent)(struct clk_stm32_clock_data *data,
			 struct clk_summary *c);
	unsigned long (*get_rate)(struct clk_stm32_clock_data *data,
				  struct clk_summary *c,
				  unsigned long parent_rate);
	const char * const *parent_names;
};

struct stm32_rcc_match_data {
	struct clk_hw_onecell_data	*hw_clks;
	unsigned int			num_clocks;
	const struct clock_config	*tab_clocks;
	unsigned int			maxbinding;
	struct clk_stm32_clock_data	*clock_data;
	u32				clear_offset;

	int (*check_security)(void __iomem *base,
			      const struct clock_config *cfg);
	int (*multi_mux)(void __iomem *base, const struct clock_config *cfg);
	u32				reset_us;
	struct clock_summary		*clock_summary;
};

int stm32_rcc_reset_init(struct device *dev, const struct of_device_id *match,
			 void __iomem *base);

int stm32_rcc_init(struct device *dev, const struct of_device_id *match_data,
		   void __iomem *base);

unsigned long stm32_divider_get_rate(void __iomem *base,
				     struct clk_stm32_clock_data *data,
				     u16 div_id,
				     unsigned long parent_rate);

u8 stm32_mux_get_parent(void __iomem *base,
			struct clk_stm32_clock_data *data,
			u16 mux_id);

/* MUX define */
#define MUX_NO_RDY		0xFF
#define MUX_SAFE		BIT(7)

/* DIV define */
#define DIV_NO_RDY		0xFF

/* Definition of clock structure */
struct clk_stm32_mux {
	u16 mux_id;
	struct clk_hw hw;
	void __iomem *base;
	struct clk_stm32_clock_data *clock_data;
	spinlock_t *lock; /* spin lock */
};

#define to_clk_stm32_mux(_hw) container_of(_hw, struct clk_stm32_mux, hw)

struct clk_stm32_gate {
	u16 gate_id;
	struct clk_hw hw;
	void __iomem *base;
	struct clk_stm32_clock_data *clock_data;
	spinlock_t *lock; /* spin lock */
};

#define to_clk_stm32_gate(_hw) container_of(_hw, struct clk_stm32_gate, hw)

struct clk_stm32_div {
	u16 div_id;
	struct clk_hw hw;
	void __iomem *base;
	struct clk_stm32_clock_data *clock_data;
	spinlock_t *lock; /* spin lock */
};

#define to_clk_stm32_divider(_hw) container_of(_hw, struct clk_stm32_div, hw)

struct clk_stm32_composite {
	u16 gate_id;
	u16 mux_id;
	u16 div_id;
	struct clk_hw hw;
	void __iomem *base;
	struct clk_stm32_clock_data *clock_data;
	spinlock_t *lock; /* spin lock */
};

#define to_clk_stm32_composite(_hw) container_of(_hw, struct clk_stm32_composite, hw)

/* Clock operators */
extern const struct clk_ops clk_stm32_mux_ops;
extern const struct clk_ops clk_stm32_gate_ops;
extern const struct clk_ops clk_stm32_divider_ops;
extern const struct clk_ops clk_stm32_composite_ops;

/* Clock registering */
struct clk_hw *clk_stm32_mux_register(struct device *dev,
				      const struct stm32_rcc_match_data *data,
				      void __iomem *base,
				      spinlock_t *lock,
				      const struct clock_config *cfg);

struct clk_hw *clk_stm32_gate_register(struct device *dev,
				       const struct stm32_rcc_match_data *data,
				       void __iomem *base,
				       spinlock_t *lock,
				       const struct clock_config *cfg);

struct clk_hw *clk_stm32_div_register(struct device *dev,
				      const struct stm32_rcc_match_data *data,
				      void __iomem *base,
				      spinlock_t *lock,
				      const struct clock_config *cfg);

struct clk_hw *clk_stm32_composite_register(struct device *dev,
					    const struct stm32_rcc_match_data *data,
					    void __iomem *base,
					    spinlock_t *lock,
					    const struct clock_config *cfg);

#define STM32_CLOCK_CFG(_binding, _clk, _sec_id, _struct, _register)\
{\
	.id		= (_binding),\
	.sec_id		= (_sec_id),\
	.clock_cfg	= (_struct) {_clk},\
	.func		= (_register),\
}

#define STM32_MUX_CFG(_binding, _clk, _sec_id)\
	STM32_CLOCK_CFG(_binding, &(_clk), _sec_id, struct clk_stm32_mux *,\
			&clk_stm32_mux_register)

#define STM32_GATE_CFG(_binding, _clk, _sec_id)\
	STM32_CLOCK_CFG(_binding, &(_clk), _sec_id, struct clk_stm32_gate *,\
			&clk_stm32_gate_register)

#define STM32_DIV_CFG(_binding, _clk, _sec_id)\
	STM32_CLOCK_CFG(_binding, &(_clk), _sec_id, struct clk_stm32_div *,\
			&clk_stm32_div_register)

#define STM32_COMPOSITE_CFG(_binding, _clk, _sec_id)\
	STM32_CLOCK_CFG(_binding, &(_clk), _sec_id, struct clk_stm32_composite *,\
			&clk_stm32_composite_register)
