/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>

#define HW_POWER_STS             0x000000c0
#define BM_POWER_STS_DC_OK       0x00000200

struct mxs_regulator {
        const char *name;
        void __iomem * control_reg;
        void __iomem * power_reg;
        struct regulator_desc rdesc;
        struct regulator_init_data *initdata;

        int mode;
        int type;
        int cur_current;
        struct mxs_regulator *parent;
        spinlock_t lock;
        wait_queue_head_t wait_q;
};

static int mxs_set_voltage(struct regulator_dev *reg, int min_uV, int uv,
        unsigned *selector)
{
        struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);
        struct regulation_constraints *constraints =
                &mxs_reg->initdata->constraints;
        u32 val, rg, i;

        pr_debug("%s: uv %d, min %d, max %d\n", __func__,
                uv, constraints->min_uV, constraints->max_uV);

        if (uv < constraints->min_uV || uv > constraints->max_uV)
                return -EINVAL;

        if( !strcmp(mxs_reg->name, "vddio") ){
                val = (uv - constraints->min_uV) / 50000;
        }
        else
                val = (uv - constraints->min_uV) * 0x1f /
                        (constraints->max_uV - constraints->min_uV);
        rg = (__raw_readl(mxs_reg->control_reg) & ~0x1f);
        pr_debug("%s: calculated val %d\n", __func__, val);
        __raw_writel(val | rg, mxs_reg->control_reg);

        for (i = 20; i; i--) {
                if (__raw_readl(mxs_reg->power_reg + HW_POWER_STS) &
                        BM_POWER_STS_DC_OK)
                        break;
                udelay(1);
        }

        if (i)
                goto out;

        __raw_writel(val | rg, mxs_reg->control_reg);
        for (i = 40000; i; i--) {
                if (__raw_readl(mxs_reg->power_reg + HW_POWER_STS) &
                        BM_POWER_STS_DC_OK)
                        break;
                udelay(1);
        }

        if (i)
                goto out;

        for (i = 40000; i; i--) {
                if (__raw_readl(mxs_reg->power_reg + HW_POWER_STS) &
                        BM_POWER_STS_DC_OK)
                        break;
                udelay(1);
        }

out:
        return !i;
}


static int mxs_get_voltage(struct regulator_dev *reg)
{
        struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);
        struct regulation_constraints * constraints =
                &mxs_reg->initdata->constraints;
        int uv;

        u32 val = __raw_readl(mxs_reg->control_reg) & 0x1f;
        if( !strcmp( mxs_reg->name, "vddio") ){
                if (val > 0x10)
                        val = 0x10;
                uv = constraints->min_uV + val * 50000;
                pr_debug("vddio = %d, val=%u\n", uv, val);
        } else
                uv = constraints->min_uV + val *
                        (constraints->max_uV - constraints->min_uV) / 0x1f;
        return uv;
}

static int mxs_get_bo_voltage(struct regulator_dev *reg)
{
        struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);
        int uv;
        int offs;

        uv = mxs_get_voltage(reg);
        offs = (__raw_readl(mxs_reg->control_reg) & ~0x700) >> 8;
        return uv - 25000*offs;
}

static int mxs_set_bo_voltage(struct regulator_dev *reg, int min_uV, int bo_uv,
        unsigned *selector)
{
        struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);
        int uv;
        int offs;
        u32 rg;
        int i;

        uv = mxs_get_voltage(reg);
        offs = (uv - bo_uv) / 25000;
        if (offs < 0 || offs > 7)
                return -EINVAL;

        rg = (__raw_readl(mxs_reg->control_reg) & ~0x700);
        pr_debug("%s: calculated offs %d\n", __func__, offs);
        __raw_writel((offs << 8) | rg, mxs_reg->control_reg);

        for (i = 10000; i; i--) {
                if (__raw_readl(mxs_reg->power_reg + HW_POWER_STS) &
                        BM_POWER_STS_DC_OK)
                        break;
                udelay(1);
        }

        if (i)
                goto out;

        for (i = 10000; i; i--) {
                if (__raw_readl(mxs_reg->power_reg + HW_POWER_STS) &
                        BM_POWER_STS_DC_OK)
                        break;
                udelay(1);
        }

out:
        return !i;
}


static int mxs_set_mode(struct regulator_dev *reg, unsigned int mode)
{
        struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);
        int ret = 0;
        u32 val;

        switch (mode) {
        case REGULATOR_MODE_FAST:
                if(mxs_reg->type == REGULATOR_VOLTAGE){
                        val = __raw_readl(mxs_reg->control_reg);
                        __raw_writel(val | (1 << 17), mxs_reg->control_reg);
                }
                mxs_reg->mode = REGULATOR_MODE_FAST;
                break;

        case REGULATOR_MODE_NORMAL:
                if(mxs_reg->type == REGULATOR_VOLTAGE){
                        val = __raw_readl(mxs_reg->control_reg);
                        __raw_writel(val & ~(1<<17), mxs_reg->control_reg);
                }
                mxs_reg->mode = REGULATOR_MODE_NORMAL;
                break;

        default:
                ret = -EINVAL;
                break;
        }
        return ret;
}

static unsigned int mxs_get_mode(struct regulator_dev *reg)
{
        struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);
        u32 val;

        if(mxs_reg->type == REGULATOR_VOLTAGE){
                val = __raw_readl(mxs_reg->control_reg) & (1 << 17);
                return val ? REGULATOR_MODE_FAST : REGULATOR_MODE_NORMAL;
        }

        return mxs_reg->mode;
}

int mxs_regulator_setup(void * driver_data)
{
        struct mxs_regulator *mxs_reg = driver_data;
        u32 vddio_reg;

        if(!strcmp(mxs_reg->name, "vddio")){
                /* Set voltage level to 3.3v */
                vddio_reg = __raw_readl(mxs_reg->control_reg) & ~0x1f;
                __raw_writel(vddio_reg | 0xA, mxs_reg->control_reg);
        }
        return 0;
}

static int main_add_current(struct mxs_regulator *mxs_reg, int uA)
{
        struct regulation_constraints * constraints =
                &mxs_reg->initdata->constraints;
        pr_debug("%s: enter reg %s, uA=%d\n",
                 __func__, mxs_reg->name, uA);
        if (uA > 0 && (mxs_reg->cur_current + uA > constraints->max_uA))
                return -EINVAL;
        else
                mxs_reg->cur_current += uA;
        return 0;
}

static int mxs_set_current_limit(struct regulator_dev *reg, int minuA, int uA)
{
        int ret = 0;
        unsigned long flags;
        struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);
        struct regulation_constraints * parent_constraints =
                &mxs_reg->parent->initdata->constraints;

        pr_debug("%s: enter reg %s, uA=%d\n",
                 __func__, mxs_reg->name, uA);

        if(!mxs_reg->parent)
                goto out;

        spin_lock_irqsave(&mxs_reg->parent->lock, flags);
        ret = main_add_current(mxs_reg->parent, uA - mxs_reg->cur_current);
        spin_unlock_irqrestore(&mxs_reg->parent->lock, flags);

        if (!ret)
                goto out;

        if (mxs_reg->mode == REGULATOR_MODE_FAST)
                return ret;

        while (ret) {
                wait_event(mxs_reg->parent->wait_q ,
                           (uA - mxs_reg->cur_current <
                            parent_constraints->max_uA -
                            mxs_reg->parent->cur_current));
                spin_lock_irqsave(&mxs_reg->parent->lock, flags);
                ret = main_add_current(mxs_reg->parent, uA -
                        mxs_reg->cur_current);
                spin_unlock_irqrestore(&mxs_reg->parent->lock, flags);
        }
out:
        if (mxs_reg->parent && (uA - mxs_reg->cur_current < 0))
                wake_up_all(&mxs_reg->parent->wait_q);
        mxs_reg->cur_current = uA;
        return 0;
}

static int mxs_get_current_limit(struct regulator_dev *reg)
{
        struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);
        return mxs_reg->cur_current;
}

static struct regulator_ops mxs_rops_voltage = {
        .set_voltage            = mxs_set_voltage,
        .get_voltage            = mxs_get_voltage,
        .set_mode               = mxs_set_mode,
        .get_mode               = mxs_get_mode,
};

static struct regulator_ops mxs_rops_current = {
        .set_current_limit      = mxs_set_current_limit,
        .get_current_limit      = mxs_get_current_limit,
        .set_mode               = mxs_set_mode,
        .get_mode               = mxs_get_mode,
};

static struct regulator_ops mxs_rops_bo = {
        .set_voltage            = mxs_set_bo_voltage,
        .get_voltage            = mxs_get_bo_voltage,
        .set_mode               = mxs_set_mode,
        .get_mode               = mxs_get_mode,
};

static int mxs_regulator_probe(struct platform_device *pdev)
{
        struct device *dev = &pdev->dev;
        struct device_node *np = dev->of_node;
        struct device_node *mxs_np;
        struct regulator_desc *rdesc;
        struct regulator_dev *rdev;
        struct mxs_regulator *sreg;
        struct regulator_init_data *initdata;
        struct regulator_config config = { };
        const char *regtype = "voltage";
        struct regulator * parent;
        int ret = 0;

        initdata = of_get_regulator_init_data(dev, np);
        sreg = devm_kzalloc(dev, sizeof(*sreg), GFP_KERNEL);
        if (!sreg)
                return -ENOMEM;
        sreg->initdata = initdata;
        sreg->initdata->regulator_init = mxs_regulator_setup;
        sreg->name = kstrdup(of_get_property(np, "regulator-name", NULL),
                             GFP_KERNEL);

        rdesc = &sreg->rdesc;
        memset(rdesc, 0, sizeof(*rdesc));
        rdesc->name = sreg->name;
        rdesc->owner = THIS_MODULE;
        rdesc->ops = &mxs_rops_voltage;

        of_property_read_string(np, "regulator-type", &regtype);
        if (!strncmp("voltage", regtype, 7)){
                rdesc->type = REGULATOR_VOLTAGE;
                sreg->control_reg = of_iomap(np, 0);
                WARN_ON(!sreg->control_reg);
                mxs_np = of_find_node_by_name(NULL,"power");
                sreg->power_reg = of_iomap(mxs_np, 0);
                WARN_ON(!sreg->power_reg);
                of_node_put(mxs_np);
        }
        else if (!strncmp("current", regtype, 7)){
                rdesc->type = REGULATOR_CURRENT;
                rdesc->ops = &mxs_rops_current;
                sreg->cur_current = 0;
                init_waitqueue_head(&sreg->wait_q);
                /* The overall_current regulator is parent to all others */
                if(strncmp(sreg->name,"overall_current",15)){
                        parent = regulator_get(dev, "overall_current");
                        if (!parent){
                                ret = -ENODEV;
                                goto mxs_probe_end;
                        }
                        sreg->parent = regulator_get_drvdata(parent);
                }
        }

        if (of_find_property(np, "mxs-regulator-brown-out", NULL))
                rdesc->ops = &mxs_rops_bo;

        config.dev = &pdev->dev;
        config.init_data = initdata;
        config.driver_data = sreg;
        config.of_node = pdev->dev.of_node;

        /* register regulator */
        rdev = regulator_register(rdesc, &config);
        if (IS_ERR(rdev)) {
                dev_err(dev, "failed to register %s\n",
                        rdesc->name);
                ret = PTR_ERR(rdev);
                goto mxs_probe_end;
        }

        platform_set_drvdata(pdev, rdev);

mxs_probe_end:
        if (ret)
                kfree(sreg->name);

        return ret;
}

static int mxs_regulator_remove(struct platform_device *pdev)
{
        struct regulator_dev *rdev = platform_get_drvdata(pdev);
        struct mxs_regulator *sreg = rdev_get_drvdata(rdev);
        const char *name = sreg->name;

        regulator_unregister(rdev);
        kfree(name);

        return 0;
}

static struct of_device_id of_mxs_regulator_match_tbl[] = {
        { .compatible = "digi,mxs-regulator", },
        { /* end */ }
};

static struct platform_driver mxs_regulator_driver = {
        .driver = {
                .name   = "mxs-regulator",
                .owner  = THIS_MODULE,
                .of_match_table = of_mxs_regulator_match_tbl,
        },
        .probe  = mxs_regulator_probe,
        .remove = mxs_regulator_remove,
};

static int __init mxs_regulator_init(void)
{
        return platform_driver_register(&mxs_regulator_driver);
}
postcore_initcall(mxs_regulator_init);

static void __exit mxs_regulator_exit(void)
{
        platform_driver_unregister(&mxs_regulator_driver);
}
module_exit(mxs_regulator_exit);

MODULE_DESCRIPTION("MXS Regulator driver");
MODULE_LICENSE("GPL v2");
