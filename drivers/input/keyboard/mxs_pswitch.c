/*
 * Copyright 2013 - Digi International, Inc. All Rights Reserved.
 *
 * PSWITCH driver for Freescale i.MX28 boards
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>

#define KEY_PRESSED		1
#define KEY_RELEASED		0
#define KEY_POLLING_PERIOD	(HZ/10)

#define HW_POWER_STS                    (0x000000c0)
#define HW_POWER_CTRL                   (0x00000000)
#define HW_POWER_CTRL_SET               (0x00000004)
#define HW_POWER_CTRL_CLR               (0x00000008)
#define BM_POWER_CTRL_POLARITY_PSWITCH  0x00040000

#define BM_POWER_CTRL_ENIRQ_PSWITCH     0x00020000
#define BM_POWER_CTRL_PSWITCH_IRQ       0x00100000
#define BM_POWER_STS_PSWITCH            0x00300000
#define BF_POWER_STS_PSWITCH(v)         (((v) << 20) & BM_POWER_STS_PSWITCH)

struct mxs_pswitch_data {
	struct input_dev	*input;
	int			irq;
	unsigned int    	input_code;
	struct delayed_work	poll_key;
        void __iomem            *power_base_addr;
};

static void mxs_pswitch_work_func(struct work_struct *work)
{
	struct mxs_pswitch_data *info =
		container_of(work, struct mxs_pswitch_data, poll_key.work);
	int pin_value;

	pin_value = __raw_readl(info->power_base_addr + HW_POWER_STS) &
			BF_POWER_STS_PSWITCH(0x1);

	if (!pin_value) {
		input_report_key(info->input, info->input_code, KEY_RELEASED);
		input_sync(info->input);

                /* Notify the PM core the end of a wakeup event */
                pm_relax(info->input->dev.parent);
	} else {
		schedule_delayed_work(&info->poll_key, KEY_POLLING_PERIOD);
	}
}

static irqreturn_t mxs_pswitch_irq_handler(int irq, void *dev_id)
{
	struct mxs_pswitch_data *info = dev_id;

	/* check if irq by power key */
	if (!(__raw_readl(info->power_base_addr + HW_POWER_CTRL) &
		BM_POWER_CTRL_PSWITCH_IRQ))
		return IRQ_HANDLED;

	/* Ack the irq */
	__raw_writel(BM_POWER_CTRL_PSWITCH_IRQ,
		info->power_base_addr + HW_POWER_CTRL_CLR);

        /* Notify the PM core of a wakeup event */
        pm_wakeup_event(info->input->dev.parent, 0);

	input_report_key(info->input, info->input_code, KEY_PRESSED);
	input_sync(info->input);

	/* schedule the work to poll the key for key-release event */
	schedule_delayed_work(&info->poll_key, KEY_POLLING_PERIOD);

	return IRQ_HANDLED;
}


static void mxs_pswitch_hwinit(struct platform_device *pdev)
{
	struct mxs_pswitch_data *info = platform_get_drvdata(pdev);

	__raw_writel(BM_POWER_CTRL_PSWITCH_IRQ,
		info->power_base_addr + HW_POWER_CTRL_CLR);
	__raw_writel(BM_POWER_CTRL_POLARITY_PSWITCH |
		BM_POWER_CTRL_ENIRQ_PSWITCH,
		info->power_base_addr + HW_POWER_CTRL_SET);
	__raw_writel(BM_POWER_CTRL_PSWITCH_IRQ,
		info->power_base_addr + HW_POWER_CTRL_CLR);

	/* enable interrupt as wakeup source */
	enable_irq_wake(info->irq);
}

static int mxs_pswitch_probe(struct platform_device *pdev)
{
	struct mxs_pswitch_data *info;
        struct device_node *np;
	int ret = 0;

	/* Create and register the input driver. */
	info = kzalloc(sizeof(struct mxs_pswitch_data), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

        if (pdev->dev.of_node) {
                np = pdev->dev.of_node;
                of_property_read_u32(np, "linux,code", &info->input_code);
                info->irq = platform_get_irq(pdev, 0);
                np = of_find_node_by_name(NULL, "power");
                info->power_base_addr = of_iomap(np, 0);
                WARN_ON(!info->power_base_addr);
                of_node_put(np);
                dev_dbg(pdev->dev.parent,"Power base address is %p\n",
                        info->power_base_addr);
        }
        else {
		dev_err(pdev->dev.parent, "No device tree data\n");
		goto out_input;
        }

	info->input = input_allocate_device();
	if (!info->input) {
		dev_err(pdev->dev.parent, "Failed to allocate input dev\n");
		ret = -ENOMEM;
		goto out_input;
	}

	info->input->name = "mxs-pswitch";
	info->input->phys = "mxs_pswitch/input0";
	info->input->id.bustype = BUS_HOST;
	info->input->dev.parent = &pdev->dev;
	info->input->evbit[0] = BIT_MASK(EV_KEY);
	info->input->keybit[BIT_WORD(info->input_code)] =
                BIT_MASK(info->input_code);

	platform_set_drvdata(pdev, info);

	INIT_DELAYED_WORK(&info->poll_key, mxs_pswitch_work_func);

	mxs_pswitch_hwinit(pdev);

	ret = request_any_context_irq(info->irq, mxs_pswitch_irq_handler,
				     IRQF_SHARED, "mxs-pswitch", info);
	if (ret < 0) {
		dev_err(pdev->dev.parent, "Failed to request IRQ: %d (%d)\n",
			info->irq, ret);
		goto out_irq;
	}

	ret = input_register_device(info->input);
	if (ret) {
		dev_err(pdev->dev.parent, "Can't register input device (%d)\n",
                         ret);
		goto out;
	}

        device_init_wakeup(&pdev->dev, 1);

	return 0;

out:
	free_irq(info->irq, info);
out_irq:
	input_free_device(info->input);
out_input:
	kfree(info);
return ret;
}

static int mxs_pswitch_remove(struct platform_device *pdev)
{
	struct mxs_pswitch_data *info = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&info->poll_key);
	free_irq(info->irq, info);
	input_unregister_device(info->input);
	kfree(info);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct of_device_id mxs_pswitch_of_match[] = {
        { .compatible = "digi,mxs-pswitch", },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxs_pswitch_of_match);

static struct platform_driver mxs_pswitch_driver = {
        .driver         = {
                .name   = "mxs-pswitch",
                .owner  = THIS_MODULE,
                .of_match_table = mxs_pswitch_of_match,
        },
        .probe          = mxs_pswitch_probe,
        .remove         = mxs_pswitch_remove,
};
module_platform_driver(mxs_pswitch_driver);

MODULE_DESCRIPTION("i.MX28 Power Switch Key driver");
MODULE_AUTHOR("Digi International Inc");
MODULE_LICENSE("GPL");
