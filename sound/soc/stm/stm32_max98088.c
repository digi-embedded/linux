/*
 * Copyright (C) 2022 Digi International, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mfd/syscon.h>
#include "../codecs/max98088.h"

struct stm32_priv {
	struct snd_soc_component *component;
	struct platform_device *pdev;
};

struct stm32_max98088_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	struct clk *mclk;
	bool is_codec_master;
};

static struct stm32_priv card_priv;

static const struct snd_soc_dapm_widget stm32_max98088_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_MIC("Mic1", NULL),
	SND_SOC_DAPM_MIC("Mic2", NULL),
	SND_SOC_DAPM_LINE("LineInA", NULL),
	SND_SOC_DAPM_LINE("LineInB", NULL),
	SND_SOC_DAPM_LINE("LineOut", NULL),
};

static int stm32_hifi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct stm32_max98088_data *data = snd_soc_card_get_drvdata(card);
	struct device *dev = card->dev;
	unsigned int fmt;
	int ret = 0;

	/* Codec always slave, master is not supported */
	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF;

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set codec dai fmt: %d\n", ret);
		return ret;
	}

	/* Select I2S Bus clock to set RCLK, BCLK and I2S mclk */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, clk_get_rate(data->mclk), SND_SOC_CLOCK_OUT);
	if (ret) {
		dev_err(dev, "failed to set cpu sysclk: %d\n", ret);
		return ret;
	}

	/* Set the default MCLK rate for the codec */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
				     clk_get_rate(data->mclk),
				     SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(dev, "failed to set codec sysclock: %d\n", ret);
		return ret;
	}

	return ret;
}

static int stm32_hifi_hw_free(struct snd_pcm_substream *substream)
{
	return 0;
}

/* Exact audio frequencies for the selected master clock */
static u32 stm32_max98088_rates[] = {
	8000, 12000, 16000, 24000, 32000, 48000, 64000, 96000
};
static struct snd_pcm_hw_constraint_list stm32_max98088_rate_constraints = {
	.count = ARRAY_SIZE(stm32_max98088_rates),
	.list = stm32_max98088_rates,
};

static int stm32_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct stm32_max98088_data *data = snd_soc_card_get_drvdata(card);
	int ret = 0;

	ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
					 SNDRV_PCM_HW_PARAM_RATE,
					 &stm32_max98088_rate_constraints);
	if (ret)
		return ret;

	ret = clk_prepare_enable(data->mclk);
	if (ret) {
		dev_err(card->dev, "Failed to enable MCLK: %d\n", ret);
		return ret;
	}

	return ret;
}

static void stm32_hifi_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct stm32_max98088_data *data = snd_soc_card_get_drvdata(card);

	clk_disable_unprepare(data->mclk);
}

static struct snd_soc_ops stm32_hifi_ops = {
	.hw_params = stm32_hifi_hw_params,
	.hw_free   = stm32_hifi_hw_free,
	.startup   = stm32_hifi_startup,
	.shutdown  = stm32_hifi_shutdown,
};

static int stm32_max98088_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *codec_np = NULL;
	struct platform_device *cpu_pdev;
	struct stm32_priv *priv = &card_priv;
	struct i2c_client *codec_dev;
	struct stm32_max98088_data *data;
	struct snd_soc_dai_link_component *dlc;
	int ret;

	priv->pdev = pdev;

	dlc = devm_kzalloc(&pdev->dev, 3 * sizeof(*dlc), GFP_KERNEL);
	if (!dlc)
		return -ENOMEM;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI/I2S platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev || !codec_dev->dev.driver) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EPROBE_DEFER;
		goto fail;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	/* Only SAI as master is supported */
	data->is_codec_master = false;

	data->mclk = devm_clk_get(&codec_dev->dev, "mclk");
	if (IS_ERR(data->mclk)) {
		ret = PTR_ERR(data->mclk);
		dev_err(&pdev->dev, "failed to get codec clk: %d\n", ret);
		goto fail;
	}

	data->dai.cpus = &dlc[0];
	data->dai.num_cpus = 1;
	data->dai.platforms = &dlc[1];
	data->dai.num_platforms = 1;
	data->dai.codecs = &dlc[2];
	data->dai.num_codecs = 1;

	data->dai.name = "HiFi";
	data->dai.stream_name = "HiFi";
	data->dai.codecs->dai_name = "HiFi";
	data->dai.codecs->of_node = codec_np;
	data->dai.cpus->dai_name = dev_name(&cpu_pdev->dev);
	data->dai.platforms->of_node = cpu_np;
	data->dai.ops = &stm32_hifi_ops;
	data->dai.dai_fmt |= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF;
	data->card.owner = THIS_MODULE;

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;

	data->card.num_links = 1;
	data->card.dai_link = &data->dai;
	data->card.dapm_widgets = stm32_max98088_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(stm32_max98088_dapm_widgets);

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail;

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);
	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static int stm32_max98088_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct stm32_max98088_data *data = snd_soc_card_get_drvdata(card);

	snd_soc_unregister_card(card);
	clk_put(data->mclk);
	kfree(data);

	return 0;
}

static const struct of_device_id stm32_max98088_dt_ids[] = {
	{ .compatible = "st,stm-audio-max98088", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, stm32_max98088_dt_ids);

static struct platform_driver stm32_max98088_driver = {
	.driver = {
		.name = "stm32-max98088",
		.pm = &snd_soc_pm_ops,
		.of_match_table = stm32_max98088_dt_ids,
	},
	.probe = stm32_max98088_probe,
	.remove = stm32_max98088_remove,
};
module_platform_driver(stm32_max98088_driver);

MODULE_AUTHOR("Digi International, Inc.");
MODULE_DESCRIPTION("STM32 max98088 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:stm32-max98088");
