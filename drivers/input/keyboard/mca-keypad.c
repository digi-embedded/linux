/* mca-keypad.c - Keypad driver for MCA devices.
 *
 * Copyright (C) 2022  Digi International Inc
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mfd/mca-common/core.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/stddef.h>
#include <linux/types.h>

#define MCA_KP_MAX_ROWS		8
#define MCA_KP_MAX_COLS		8
#define MCA_KP_MAX_N_KEYS	(MCA_KP_MAX_ROWS * MCA_KP_MAX_COLS)
#define IS_KEY_PRESSED(c)	((c) & MCA_KP_FIFO_EV_KPRESS)
#define GET_SCAN_CODE(c)	((c) & MCA_KP_FIFO_SC_MASK)

#define MCA_KP_MAX_DEB		250
#define MCA_KP_DEF_DEB		10

struct mca_kp {
	struct mca_drv		*mca;
	struct device_node	*np;
	struct input_dev	*inpdev;
	int			irq;
	unsigned int		n_rows;
	unsigned int		n_cols;
	unsigned short		keycodes[MCA_KP_MAX_N_KEYS];
	u8			rows_io_list[MCA_KP_MAX_ROWS];
	u8			cols_io_list[MCA_KP_MAX_COLS];
	bool			pwroff_wakeup;
	u32			debounce;
};

static irqreturn_t mca_kp_irq_handler(int irq, void *data)
{
	struct mca_kp *kp = data;
	int key_code, key_press;
	u8 buf[2];
	int ret;
	bool done = false;

	while (!done) {
		ret = regmap_bulk_read(kp->mca->regmap, MCA_REG_KP_FIFO,
				       buf, sizeof(buf));
		if (ret) {
			dev_warn(&kp->inpdev->dev,
				 "Failed to read MCA_REG_KP_FIFO (%d)\n", ret);
			return IRQ_HANDLED;
		}

		dev_dbg(&kp->inpdev->dev, "buf[0]: %x - buf[1]: %x\n", buf[0], buf[1]);

		if (buf[0] != 0) {
			key_press = IS_KEY_PRESSED(buf[1]);
			key_code = GET_SCAN_CODE(buf[1]);

			dev_dbg(&kp->inpdev->dev, "Event: %d - Key: %d - %s",
				key_code, kp->keycodes[key_code],
				key_press ? "pressed" : "released");

			input_event(kp->inpdev, EV_MSC, MSC_SCAN, key_code);
			input_report_key(kp->inpdev, kp->keycodes[key_code], key_press);
			input_sync(kp->inpdev);
		} else {
			done = true;
		}
	}

	return IRQ_HANDLED;
}

static int mca_kp_start(struct mca_kp *kp)
{
	return regmap_update_bits(kp->mca->regmap, MCA_REG_KP_CTRL0,
				  MCA_KP_EN, MCA_KP_EN);
}

static int mca_kp_stop(const struct mca_kp *kp)
{
	return regmap_update_bits(kp->mca->regmap, MCA_REG_KP_CTRL0,
				  MCA_KP_EN, 0);
}

static int mca_kp_open(struct input_dev *dev)
{
	struct mca_kp *kp = input_get_drvdata(dev);

	return mca_kp_start(kp);
}

static void mca_kp_close(struct input_dev *dev)
{
	struct mca_kp *kp = input_get_drvdata(dev);

	mca_kp_stop(kp);
}

static int mca_kp_init(struct mca_kp *kp)
{
	int ret;
	unsigned int val = 0;

	/* Set row and column width */
	ret = regmap_write(kp->mca->regmap, MCA_REG_KP_NCOLS, kp->n_cols);
	if (ret < 0) {
		dev_err(&kp->inpdev->dev,
			"failed to set number of cols: %d (%d)\n",
			kp->n_cols, ret);
		return ret;
	}

	ret = regmap_write(kp->mca->regmap, MCA_REG_KP_NROWS, kp->n_rows);
	if (ret < 0) {
		dev_err(&kp->inpdev->dev,
			"failed to set number of rows: %d (%d)\n",
			kp->n_rows, ret);
		return ret;
	}

	/* Set row and column ios */
	ret = regmap_bulk_write(kp->mca->regmap, MCA_REG_KP_COL0_IO,
				kp->cols_io_list, kp->n_cols);
	if (ret) {
		dev_err(&kp->inpdev->dev,
			"Error writing MCA_REG_KP_COL0_IO (%d)\n",
			ret);
		return ret;
	}

	ret = regmap_bulk_write(kp->mca->regmap, MCA_REG_KP_ROW0_IO,
				kp->rows_io_list, kp->n_rows);
	if (ret) {
		dev_err(&kp->inpdev->dev,
			"Error writing MCA_REG_KP_ROW0_IO (%d)\n",
			ret);
		return ret;
	}

	/* Configure debounce filter */
	ret = regmap_write(kp->mca->regmap, MCA_REG_KP_DEBOUNCE, kp->debounce);
	if (ret < 0) {
		dev_err(&kp->inpdev->dev,
			"failed to set debounce period filter: %d (%d)\n",
			kp->debounce, ret);
		return ret;
	}

	/* Configure power-off wakeup feature */
	val = kp->pwroff_wakeup ? MCA_KP_PWROFF_WAKE_EN : 0;
	ret = regmap_update_bits(kp->mca->regmap, MCA_REG_KP_CTRL0,
				 MCA_KP_PWROFF_WAKE_EN, val);
	if (ret) {
		dev_err(&kp->inpdev->dev,
			"Error writing MCA_REG_KP_CTRL0 (%d)\n",
			ret);
		return ret;
	}

	return 0;
}

static int mca_kp_parse_dt(struct mca_kp *kp)
{
	struct device *dev = kp->inpdev->dev.parent;
	struct device_node *np = dev->of_node;
	struct property *prop;
	const __be32 *cur;
	u32 io, num_ios;

	num_ios = 0;
	of_property_for_each_u32(np, "cols-io-list", prop, cur, io) {
		if (io >= MCA_MAX_IOS)
			return -EINVAL;

		kp->cols_io_list[num_ios] = (u8)io;
		num_ios++;
	}

	kp->n_cols = num_ios;

	if (kp->n_cols > MCA_KP_MAX_COLS) {
		dev_err(dev,
			"number columns exceeds max value (%d - %d)\n",
			kp->n_cols, MCA_KP_MAX_COLS);
		return -EINVAL;
	}

	num_ios = 0;
	of_property_for_each_u32(np, "rows-io-list", prop, cur, io) {
		if (io >= MCA_MAX_IOS)
			return -EINVAL;

		kp->rows_io_list[num_ios] = (u8)io;
		num_ios++;
	}

	kp->n_rows = num_ios;

	if (kp->n_rows > MCA_KP_MAX_ROWS) {
		dev_err(dev,
			"number rows exceeds max value (%d - %d)\n",
			kp->n_rows, MCA_KP_MAX_ROWS);
		return -EINVAL;
	}

	kp->pwroff_wakeup = of_property_read_bool(np, "pwr-off-wakeup");

	kp->debounce = MCA_KP_DEF_DEB;
	if (!of_property_read_u32(np, "debounce-ms", &kp->debounce)) {
		if (kp->debounce > MCA_KP_MAX_DEB)
			kp->debounce = MCA_KP_MAX_DEB;
	}

	dev_info(dev, "%d rows, %d cols, %d ms debounce%s\n", kp->n_rows,
		 kp->n_cols, kp->debounce,
		 kp->pwroff_wakeup ? ", pwroff_wakeup" : "");

	return 0;
}

static int mca_kp_probe(struct platform_device *pdev)
{
	struct mca_drv *mca = dev_get_drvdata(pdev->dev.parent);
	struct mca_kp *kp;
	struct device_node *np = NULL;
	int ret = 0;

	if (!mca || !mca->dev || !mca->dev->parent ||
	    !mca->dev->parent->of_node)
		return -EPROBE_DEFER;

	if (mca->dev->of_node) {
		const char * compatible = pdev->dev.driver->
				    of_match_table[0].compatible;

		/* Return if keypad node does not exist or if it is disabled */
		np = of_find_compatible_node(mca->dev->of_node, NULL, compatible);
		if (!np || !of_device_is_available(np))
			return -ENODEV;
	}

	kp = devm_kzalloc(&pdev->dev, sizeof(struct mca_kp), GFP_KERNEL);
	if (!kp) {
		dev_err(&pdev->dev, "Failed to allocate memory.\n");
		return -ENOMEM;
	}

	kp->inpdev = devm_input_allocate_device(&pdev->dev);
	if (!kp->inpdev) {
		dev_err(&pdev->dev, "failed to allocate the input device\n");
		ret = -ENOMEM;
		goto err_free_mem;
	}

	kp->mca = mca;
	kp->irq = platform_get_irq_byname(pdev, MCA_IRQ_KEYPAD_NAME);
	if (kp->irq < 0)
		goto err_free_mem2;

	__set_bit(EV_KEY, kp->inpdev->evbit);

	/* Enable auto repeat feature of Linux input subsystem */
	if (of_property_read_bool(pdev->dev.of_node, "autorepeat"))
		__set_bit(EV_REP, kp->inpdev->evbit);

	kp->inpdev->name = pdev->name;
	kp->inpdev->phys = "keypad/input0";
	kp->inpdev->dev.parent = &pdev->dev;
	kp->inpdev->open = mca_kp_open;
	kp->inpdev->close = mca_kp_close;
	kp->inpdev->id.bustype = BUS_I2C;

	input_set_capability(kp->inpdev, EV_MSC, MSC_SCAN);
	input_set_drvdata(kp->inpdev, kp);

	ret = mca_kp_parse_dt(kp);
	if (ret)
		goto err_free_mem2;

	ret = mca_kp_init(kp);
	if (ret)
		goto err_free_mem2;

	ret = matrix_keypad_build_keymap(NULL, NULL,
					 kp->n_rows, kp->n_cols,
					 kp->keycodes, kp->inpdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to build keymap\n");
		goto err_free_mem2;
	}

	ret = devm_request_threaded_irq(&pdev->dev, kp->irq, NULL,
					mca_kp_irq_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					MCA_IRQ_KEYPAD_NAME, kp);
	if (ret) {
		dev_err(&pdev->dev, "failed to request %s IRQ (%d)\n",
			MCA_IRQ_KEYPAD_NAME, kp->irq);
		goto err_free_mem2;
	}

	ret = input_register_device(kp->inpdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto err_irq;
	}

	platform_set_drvdata(pdev, kp);
	device_init_wakeup(&pdev->dev, 1);

	return 0;

err_irq:
	if (kp->irq)
		free_irq(kp->irq, kp);
err_free_mem2:
	input_free_device(kp->inpdev);
err_free_mem:
	kfree(kp);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int __maybe_unused mca_kp_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mca_kp *kp = platform_get_drvdata(pdev);
	unsigned int val = 0;

	if (device_may_wakeup(&pdev->dev))
		val = MCA_KP_SLEEP_WAKE_EN;

	return regmap_update_bits(kp->mca->regmap, MCA_REG_KP_CTRL0,
				  MCA_KP_SLEEP_WAKE_EN, val);
}

static int __maybe_unused mca_kp_resume(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(mca_kp_pm, mca_kp_suspend, mca_kp_resume);

static const struct of_device_id mca_kp_of_match[] = {
	{ .compatible = "digi,mca-keypad" },
	{ },
};
MODULE_DEVICE_TABLE(of, mca_kp_of_match);

static struct platform_driver mca_kp_device_driver = {
	.probe		= mca_kp_probe,
	.driver		= {
		.name	= "mca-keypad",
		.pm  	= &mca_kp_pm,
		.of_match_table = of_match_ptr(mca_kp_of_match),
	}
};

module_platform_driver(mca_kp_device_driver);

MODULE_AUTHOR("Digi International Inc");
MODULE_DESCRIPTION("MCA Keypad Driver");
MODULE_LICENSE("GPL v2");
