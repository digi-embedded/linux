/*
 *  Copyright 2018 Digi International Inc
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <linux/mfd/mca-common/core.h>
#include <linux/mfd/mca-common/registers.h>

#define MCA_TPM_PWM_NR_CH	8
#define MAX_NUM_TPMS		16	/* Just a safe max limit */
#define MCA_TPM_CLKIN_HZ	48000000U

struct mca_tpm_mod {
	struct mca_pwm_drv	*parent;
	struct device_node	*np;
	struct pwm_chip		chip;
	u32			tpm_idx;
	u8			cfg[MCA_TPM_PWM_NR_CH];
	u8			duty[MCA_TPM_PWM_NR_CH];
	u32			period;
};

struct mca_pwm_drv {
	int			num_tpms;
	u32			tpm_clk_hz;
	struct device		*dev;
	struct mca_drv		*mca;
	struct device_node	*np;
	struct mca_tpm_mod	*tpms;
};

#define to_mca_tpm_mod(_chip)	container_of(_chip, struct mca_tpm_mod, chip)

#define TPM_CH_CFG_REG(t,c)	(MCA_REG_TPM0_CH0_CFG + MCA_PWM_CH_LEN * (c) + MCA_PWM_TPM_LEN * (t))
#define TPM_CH_DUTY_REG(t,c)	(MCA_REG_TPM0_CH0_DUTY + MCA_PWM_CH_LEN * (c) + MCA_PWM_TPM_LEN * (t))
#define TPM_CFG_REG(t)		(MCA_REG_TPM0_CFG0 + MCA_PWM_TPM_LEN * (t))
#define TPM_CFG_FREQ(t)		(MCA_REG_TPM0_FREQ_0 + MCA_PWM_TPM_LEN * (t))
#define TPM_CFG_PRESCALER(t)	(MCA_REG_TPM0_PRESCALER + MCA_PWM_TPM_LEN * (t))

#define TPM_MOD_MAX_VAL		0xffff

static inline struct regmap *mca_tpm_mod_to_regmap(struct mca_tpm_mod *mca_tpm)
{
	return mca_tpm->parent->mca->regmap;
}

static int mca_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			  int duty_ns, int period_ns)
{
	struct mca_tpm_mod *mca_tpm = to_mca_tpm_mod(chip);
	struct regmap *map = mca_tpm_mod_to_regmap(mca_tpm);
	const unsigned int prescaler[] = { 1, 2, 4, 8, 16, 32, 64, 128 };
	u64 freq_hz, duty_percent, tpm_mod_val;
	int ret, prescaler_idx = 0;
	u8 buf[2];

	if (pwm_is_enabled(pwm) && (period_ns != pwm_get_period(pwm))) {
		dev_err(chip->dev, "cannot change PWM period while enabled\n");
		return -EBUSY;
	}

	/* compute the prescaler */
	tpm_mod_val = (u64)mca_tpm->parent->tpm_clk_hz * (u64)period_ns;
	do_div(tpm_mod_val, NSEC_PER_SEC);

	while (tpm_mod_val > TPM_MOD_MAX_VAL) {
		prescaler_idx++;
		tpm_mod_val >>= 1;
		if (prescaler_idx == ARRAY_SIZE(prescaler)) {
			dev_err(chip->dev,
				"prescaler required exceeds max value\n");
			return -EINVAL;
		}
	}

	/* compute the freqency and duty cycle values */
	freq_hz = NSEC_PER_SEC;
	do_div(freq_hz, period_ns);

	duty_percent = duty_ns * 100;
	do_div(duty_percent, period_ns);

	dev_dbg(chip->dev,
		"duty_ns = %d, period_ns = %d, freq = %llu, duty = %llu,"
		"prescaler = %d/%d\n",
		duty_ns, period_ns, freq_hz, duty_percent, prescaler_idx,
		prescaler[prescaler_idx]);

	/* Program the prescaler, the frequency and the duty cycle */
	ret = regmap_write(map, TPM_CFG_PRESCALER(mca_tpm->tpm_idx),
			   prescaler_idx);
	if (ret) {
		dev_err(chip->dev, "failed writting prescaler register\n");
		return ret;
	}

	dev_dbg(chip->dev, "[%04x] <- %x\n",
		TPM_CFG_PRESCALER(mca_tpm->tpm_idx), prescaler_idx);

	/*
	 * The frequency value is stored in a 64bit variable, to avoid losing
	 * precission in the period to frequency conversion, but the value fits
	 * in a 32 bit variable and the mca freq register has only 4 bytes.
	 * Therefore, we program only the four less significative bytes.
	 */
	ret = regmap_bulk_write(map, TPM_CFG_FREQ(mca_tpm->tpm_idx),
				(u8 *)&freq_hz, sizeof(u32));
	if (ret) {
		dev_err(chip->dev, "failed writting frequency register\n");
		return ret;
	}

	dev_dbg(chip->dev, "[%04x] <- %x\n", TPM_CFG_FREQ(mca_tpm->tpm_idx),
		(u32)freq_hz);

	buf[0] = mca_tpm->cfg[pwm->hwpwm];
	buf[1] = (u8)duty_percent;

	ret = regmap_bulk_write(map, TPM_CH_CFG_REG(mca_tpm->tpm_idx, pwm->hwpwm),
				buf, sizeof(buf));
	if (!ret) {
		mca_tpm->duty[pwm->hwpwm] = buf[1];

		dev_dbg(chip->dev, "[%04x] <- %02x.%02x\n",
			TPM_CH_CFG_REG(mca_tpm->tpm_idx, pwm->hwpwm),
			buf[0], buf[1]);
	}

	return ret;
}

static int mca_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct mca_tpm_mod *mca_tpm = to_mca_tpm_mod(chip);
	struct regmap *map = mca_tpm_mod_to_regmap(mca_tpm);
	u8 buf[2];
	int ret;

	buf[0] = mca_tpm->cfg[pwm->hwpwm] | MCA_TPM_CH_EN;
	buf[1] = mca_tpm->duty[pwm->hwpwm];

	ret = regmap_bulk_write(map, TPM_CH_CFG_REG(mca_tpm->tpm_idx, pwm->hwpwm),
				buf, sizeof(buf));
	if (!ret) {
		mca_tpm->cfg[pwm->hwpwm] = buf[0];

		dev_dbg(chip->dev, "[%04x] <- %02x.%02x\n",
			TPM_CH_CFG_REG(mca_tpm->tpm_idx, pwm->hwpwm),
			buf[0], buf[1]);
	}

	return ret;
}

static void mca_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct mca_tpm_mod *mca_tpm = to_mca_tpm_mod(chip);
	struct regmap *map = mca_tpm_mod_to_regmap(mca_tpm);
	u8 buf[2];
	int ret;

	buf[0] = mca_tpm->cfg[pwm->hwpwm] & ~MCA_TPM_CH_EN;

	ret = regmap_bulk_write(map, TPM_CH_CFG_REG(mca_tpm->tpm_idx, pwm->hwpwm),
				buf, sizeof(buf));
	if (!ret) {
		mca_tpm->cfg[pwm->hwpwm] = buf[0];

		dev_dbg(chip->dev, "[%04x] <- %02x.%02x\n",
			TPM_CH_CFG_REG(mca_tpm->tpm_idx, pwm->hwpwm),
			buf[0], buf[1]);
	}
}

static int mca_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
				enum pwm_polarity polarity)
{
	struct mca_tpm_mod *mca_tpm = to_mca_tpm_mod(chip);
	struct regmap *map = mca_tpm_mod_to_regmap(mca_tpm);
	u8 buf[2];
	int ret;

	if (polarity == PWM_POLARITY_NORMAL)
		buf[0] = mca_tpm->cfg[pwm->hwpwm] | MCA_TPM_CH_POL_HIGH;
	else
		buf[0] = mca_tpm->cfg[pwm->hwpwm] & ~MCA_TPM_CH_POL_HIGH;

	buf[1] = mca_tpm->duty[pwm->hwpwm];

	ret = regmap_bulk_write(map, TPM_CH_CFG_REG(mca_tpm->tpm_idx, pwm->hwpwm),
				buf, sizeof(buf));
	if (!ret) {
		mca_tpm->cfg[pwm->hwpwm] = buf[0];

		dev_dbg(chip->dev, "[%04x] <- %02x.%02x\n",
			TPM_CH_CFG_REG(mca_tpm->tpm_idx, pwm->hwpwm),
			buf[0], buf[1]);
	}

	return ret;
}

static struct pwm_device *
mca_pwm_of_xlate(struct pwm_chip *chip, const struct of_phandle_args *args)
{
	struct mca_tpm_mod *tpm = to_mca_tpm_mod(chip);

	if (tpm->np != args->np)
		return NULL;

	return of_pwm_xlate_with_flags(chip, args);
}

static struct pwm_ops mca_pwm_ops = {
	.config		= mca_pwm_config,
	.enable		= mca_pwm_enable,
	.disable	= mca_pwm_disable,
	.set_polarity	= mca_pwm_set_polarity,
	.owner		= THIS_MODULE,
};

static const struct of_device_id mca_pwm_dt_ids[] = {
	{
		.compatible = "digi,mca-pwm",
	},
};
MODULE_DEVICE_TABLE(of, mca_pwm_dt_ids);

#ifdef CONFIG_OF
static int mca_pwm_get_tpm_config_of(struct mca_pwm_drv *pwm_drv)
{
	struct device_node *node;
	u32 num_tpms;

	if (!pwm_drv->np || !of_get_next_child(pwm_drv->np, NULL))
		return -EINVAL;

	num_tpms = of_get_child_count(pwm_drv->np);
	if (!num_tpms || num_tpms > MAX_NUM_TPMS) {
		dev_err(pwm_drv->dev, "invalid number of tpm child nodes");
		return -ENODEV;
	}

	pwm_drv->tpms = devm_kzalloc(pwm_drv->dev,
				     num_tpms * sizeof(struct mca_tpm_mod),
				     GFP_KERNEL);
	if (!pwm_drv->tpms) {
		dev_err(pwm_drv->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	pwm_drv->num_tpms = 0;
	pwm_drv->tpm_clk_hz = MCA_TPM_CLKIN_HZ;

	for_each_child_of_node(pwm_drv->np, node) {
		struct mca_tpm_mod *tpm;
		u32 tpm_idx, num_ch, ch;

		tpm = &pwm_drv->tpms[pwm_drv->num_tpms];

		if (of_property_read_u32(node, "reg", &tpm_idx)) {
			dev_err(pwm_drv->dev,
				"invalid/missing reg entry in devicetree\n");
			continue;
		}

		if (of_property_read_u32(node, "pwm-channels", &num_ch) ||
		    num_ch >= MCA_TPM_PWM_NR_CH) {
			dev_err(pwm_drv->dev,
				"invalid/missing pwm-channels entry in devicetree\n");
			continue;
		}

		tpm->tpm_idx = tpm_idx;
		tpm->chip.npwm = num_ch;
		tpm->np = node;

		/* Init channels: disabled, active high polarity */
		for (ch = 0; ch < num_ch; ch++)
			tpm->cfg[ch] = MCA_TPM_CH_POL_HIGH;

		pwm_drv->num_tpms++;
	}

	return 0;
}
#endif

static int mca_pwm_probe(struct platform_device *pdev)
{
	struct mca_drv *mca = dev_get_drvdata(pdev->dev.parent);
	struct mca_pwm_drv *pwm_drv;
	int ret, i;

	pwm_drv = devm_kzalloc(&pdev->dev, sizeof(*pwm_drv), GFP_KERNEL);
	if (!pwm_drv) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, pwm_drv);
	pwm_drv->np = pdev->dev.of_node;
	pwm_drv->mca = mca;
	pwm_drv->dev = &pdev->dev;

	ret = mca_pwm_get_tpm_config_of(pwm_drv);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get DT driver configuration (%d)\n", ret);
		goto err_free;
	}

	for (i = 0; i < pwm_drv->num_tpms; i++) {
		struct mca_tpm_mod *tpm = &pwm_drv->tpms[i];

		tpm->parent = pwm_drv;
		tpm->chip.dev = &pdev->dev;
		tpm->chip.ops = &mca_pwm_ops;
		tpm->chip.of_pwm_n_cells = 3;
		tpm->chip.base = -1;
		tpm->chip.of_xlate = mca_pwm_of_xlate;

		ret = pwmchip_add(&tpm->chip);
		if (ret < 0) {
			dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
			goto err_free;
		}

		dev_info(&pdev->dev,
			 "registered PWM chip (tpm: %d, channels: %d)\n",
			 tpm->tpm_idx, tpm->chip.npwm);
	}

	return 0;

err_free:
	kfree(pwm_drv->tpms);
	kfree(pwm_drv);

	return ret;
}

static int mca_pwm_remove(struct platform_device *pdev)
{
	struct mca_pwm_drv *pwm_drv = platform_get_drvdata(pdev);
	int ret, i;

	for (i = 0; i < pwm_drv->num_tpms; i++) {
		struct mca_tpm_mod *tpm = &pwm_drv->tpms[i];

		ret = pwmchip_remove(&tpm->chip);
		if (ret) {
			dev_warn(&pdev->dev,
				 "error removing PWM chip (tpm: %d)\n",
				 tpm->tpm_idx);
		}
	}

	kfree(pwm_drv->tpms);
	kfree(pwm_drv);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mca_pwm_suspend(struct device *dev)
{
	return 0;
}

static int mca_pwm_resume(struct device *dev)
{
	return 0;
};
static SIMPLE_DEV_PM_OPS(mca_pwm_pm, mca_pwm_suspend, mca_pwm_resume);
#endif


static struct platform_driver mca_pwm_driver = {
	.driver = {
		.name = "mca-pwm",
		.of_match_table = of_match_ptr(mca_pwm_dt_ids),
#ifdef CONFIG_PM_SLEEP
		.pm = &mca_pwm_pm,
#endif
	},
	.probe = mca_pwm_probe,
	.remove = mca_pwm_remove,
};

module_platform_driver(mca_pwm_driver);

MODULE_ALIAS("platform:mca-pwm");
MODULE_AUTHOR("Digi International Inc");
MODULE_DESCRIPTION("MCA PWM driver");
MODULE_LICENSE("GPL v2");
