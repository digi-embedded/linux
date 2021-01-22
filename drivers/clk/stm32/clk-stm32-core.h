/* SPDX-License-Identifier: GPL-2.0  */
/*
 * Copyright (C) STMicroelectronics 2020 - All Rights Reserved
 * Author: Gabriel Fernandez <gabriel.fernandez@st.com> for STMicroelectronics.
 */

#include <linux/clk-provider.h>

struct stm32_rcc_match_data;

struct stm32_mux_cfg {
	const char * const *parent_names;
	u8	num_parents;
	u32	reg_off;
	u8	shift;
	u8	width;
	u8	mux_flags;
	u32	*table;
};

struct stm32_gate_cfg {
	u32	reg_off;
	u8	bit_idx;
	u8	gate_flags;
	u8	set_clr;
};

struct stm32_div_cfg {
	u32	reg_off;
	u8	shift;
	u8	width;
	u8	div_flags;
	const struct clk_div_table *table;
};

struct stm32_composite_cfg {
	int mux;
	int gate;
	int div;
};

struct clock_config {
	unsigned long	id;
	const char	*name;
	const char	*parent_name;
	unsigned long	flags;
	int		sec_id;
	void		*clock_cfg;

	struct clk_hw *(*func)(struct device *dev,
			       const struct stm32_rcc_match_data *data,
			       void __iomem *base,
			       spinlock_t *lock,
			       const struct clock_config *cfg);
};

struct stm32_rcc_match_data {
	unsigned int			num_clocks;
	const struct clock_config	*tab_clocks;
	unsigned int			maxbinding;
	const struct stm32_gate_cfg	*gates;
	const struct stm32_mux_cfg	*muxes;
	const struct stm32_div_cfg	*dividers;

	int (*check_security)(void __iomem *base,
			      const struct clock_config *cfg);
	u32 clear_offset;
};

int stm32_rcc_init(struct device *dev, const struct of_device_id *match_data,
		   void __iomem *base);

#define NO_ID 0xFFFF0000

#define NO_STM32_MUX	-1
#define NO_STM32_DIV	-1
#define NO_STM32_GATE	-1

struct clk_hw *clk_stm32_gate_ops_register(struct device *dev,
					   const char *name, const char *parent_name,
					   unsigned long flags, void __iomem *reg,
					   const struct stm32_gate_cfg *cfg,
					   const struct clk_ops *ops, spinlock_t *lock);

struct clk_hw *clk_stm32_gate_register(struct device *dev,
				       const char *name,
				       const char *parent_name,
				       unsigned long flags,
				       void __iomem *base,
				       const struct stm32_gate_cfg *cfg,
				       spinlock_t *lock); /* spinlock*/

struct clk_hw *
clk_stm32_register_composite(struct device *dev,
			     const char *name,
			     const char * const *parent_names,
			     int num_parents, void __iomem *base,
			     const struct stm32_mux_cfg *mux_cfg,
			     const struct stm32_div_cfg *div_cfg,
			     const struct stm32_gate_cfg *gate_cfg,
			     unsigned long flags, spinlock_t *lock);

struct clk_hw *
_clk_hw_register_gate(struct device *dev,
		      const struct stm32_rcc_match_data *data,
		      void __iomem *base, spinlock_t *lock,
		      const struct clock_config *cfg);

struct clk_hw *
_clk_stm32_gate_register(struct device *dev,
			 const struct stm32_rcc_match_data *data,
			 void __iomem *base, spinlock_t *lock,
			 const struct clock_config *cfg);
struct clk_hw *
_clk_stm32_mux_register(struct device *dev,
			const struct stm32_rcc_match_data *data,
			void __iomem *base, spinlock_t *lock,
			const struct clock_config *cfg);

struct clk_hw *
_clk_stm32_register_composite(struct device *dev,
			      const struct stm32_rcc_match_data *data,
			      void __iomem *base, spinlock_t *lock,
			      const struct clock_config *cfg);

int mp1_gate_clk_is_enabled(struct clk_hw *hw);
int mp1_gate_clk_enable(struct clk_hw *hw);
void mp1_gate_clk_disable(struct clk_hw *hw);
void mp1_gate_clk_endisable(struct clk_hw *hw, int enable);

struct clk_hw *stm32_get_mux_cfg(void __iomem *base,
				 const struct stm32_mux_cfg *cfg, spinlock_t *lock);

struct clk_hw *stm32_get_div_cfg(void __iomem *base,
				 const struct stm32_div_cfg *cfg, spinlock_t *lock);

const struct clk_ops *stm32_get_div_ops(const struct stm32_div_cfg *cfg);

struct clk_hw *stm32_get_gate_cfg(void __iomem *base,
				  const struct stm32_gate_cfg *cfg, spinlock_t *lock);

const struct stm32_mux_cfg *stm32_get_composite_mux_cfg(const struct stm32_rcc_match_data *data,
							const struct clock_config *cfg);

struct clk_hw *stm32_get_composite_mux_hw(struct device *dev,
					  const struct stm32_rcc_match_data *data,
					  void __iomem *base, spinlock_t *lock,
					  const struct clock_config *cfg);

struct clk_hw *stm32_get_composite_rate_hw(struct device *dev,
					   const struct stm32_rcc_match_data *data,
					   void __iomem *base, spinlock_t *lock,
					   const struct clock_config *cfg);

struct clk_hw *stm32_get_composite_gate_hw(struct device *dev,
					   const struct stm32_rcc_match_data *data,
					   void __iomem *base, spinlock_t *lock,
					   const struct clock_config *cfg);

const struct clk_ops *stm32_get_composite_gate_ops(const struct stm32_rcc_match_data *data,
						   const struct clock_config *cfg);
struct gate_cfg {
	u32 reg_off;
	u8 bit_idx;
	u8 gate_flags;
};

#define GATE(_id, _name, _parent, _flags, _offset, _bit_idx, _gate_flags)\
{\
	.id		= _id,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.clock_cfg	=  &(struct gate_cfg) {\
		.reg_off	= _offset,\
		.bit_idx	= _bit_idx,\
		.gate_flags	= _gate_flags,\
	},\
	.func		= _clk_hw_register_gate,\
}

struct stm32_clk_gate_cfg {
	int gate_id;
};

#define STM32_GATE(_id, _name, _parent, _flags, _gate_id, _sec_id)\
{\
	.id		= _id,\
	.sec_id		= _sec_id,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.clock_cfg	= &(struct stm32_clk_gate_cfg) {\
		.gate_id	= _gate_id,\
	},\
	.func		= _clk_stm32_gate_register,\
}

struct stm32_clk_mux_cfg {
	int mux_id;
};

#define STM32_MUX(_id, _name, _flags, _mux_id, _sec_id)\
{\
	.id		= _id,\
	.sec_id		= _sec_id,\
	.name		= _name,\
	.flags		= _flags,\
	.clock_cfg	= &(struct stm32_clk_mux_cfg) {\
		.mux_id	= _mux_id,\
	},\
	.func		= _clk_stm32_mux_register,\
}

struct stm32_clk_composite_cfg {
	int	gate_id;
	int	mux_id;
	int	div_id;
};

#define STM32_COMPOSITE(_id, _name, _flags, _sec_id,\
			_gate_id, _mux_id, _div_id)\
{\
	.id		= _id,\
	.name		= _name,\
	.sec_id		= _sec_id,\
	.flags		= _flags,\
	.clock_cfg	= &(struct stm32_clk_composite_cfg) {\
		.gate_id	= _gate_id,\
		.mux_id		= _mux_id,\
		.div_id		= _div_id,\
	},\
	.func		= _clk_stm32_register_composite,\
}

#define STM32_COMPOSITE_NOMUX(_id, _name, _parent, _flags, _sec_id, _gate_id, _div_id)\
{\
	.id		= _id,\
	.name		= _name,\
	.parent_name	= _parent,\
	.sec_id		= _sec_id,\
	.flags		= _flags,\
	.clock_cfg	= &(struct stm32_clk_composite_cfg) {\
		.gate_id	= _gate_id,\
		.mux_id		= NO_STM32_MUX,\
		.div_id		= _div_id,\
	},\
	.func		= _clk_stm32_register_composite,\
}
