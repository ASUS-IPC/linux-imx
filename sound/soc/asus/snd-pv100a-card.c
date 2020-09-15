/*
 * ASUS machine ASoC Driver for the PV100A dev board using a RT5616 CODEC.
 *
 * Copyright (c) 2020, ASUSTeK COMPUTER INC.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/clk.h>

#include "../codecs/rt5616.h"
#include "../fsl/fsl_sai.h"

#define DRV_NAME "pv100a-audio-card"
#define RT5616_STEREO_RATES SNDRV_PCM_RATE_8000_192000
#define RT5616_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S8)

static unsigned long codec_clock = 2457600;

static const struct snd_soc_dapm_widget pv100a_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_SPK("Lineout", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
};

static const struct snd_soc_dapm_route pv100a_audio_map[] = {
	{"Headphones", NULL, "HPOR"},
	{"Headphones", NULL, "HPOL"},
	{"Lineout", NULL, "LOUTL"},
	{"Lineout", NULL, "LOUTR"},
	{"Int Mic", NULL, "micbias1"},
	{"IN1P", NULL, "Int Mic"},
};

static const struct snd_kcontrol_new pv100a_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphones"),
	SOC_DAPM_PIN_SWITCH("Lineout"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
};

static int pv100a_aif1_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int mclk, ret;

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		return -EINVAL;
	}

	/* Configure codec DAI PLL, freq_in=24.576MHz */
	ret = snd_soc_dai_set_pll(codec_dai, 0, RT5616_PLL1_S_MCLK, codec_clock, mclk);
	if (ret < 0) {
		dev_err(rtd->dev, "Can't set codec PLL: %d\n", ret);
		return ret;
	}
	/* Configure codec DAI system clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, RT5616_SCLK_S_PLL1, mclk, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(rtd->dev, "Can't set codec_dai sysclk in %d\n", ret);
		return ret;
	}
	ret = snd_soc_dai_set_sysclk(codec_dai, RT5616_SCLK_S_PLL1, mclk, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(rtd->dev, "Can't set codec_dai sysclk out %d\n", ret);
		return ret;
	}
	/* Configure cpu DAI system clock */
	ret = snd_soc_dai_set_sysclk(cpu_dai, FSL_SAI_CLK_MAST1, mclk, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(rtd->dev, "Can't set cpu_dai sysclk %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct snd_soc_ops pv100a_aif1_ops = {
	.hw_params = pv100a_aif1_hw_params,
};

static struct snd_soc_dai_link pv100a_dailink[] = {

	{
		.name = "rt5616",
		.stream_name = "RT5616 PCM",
		.codec_dai_name = "rt5616-aif1",
		.ops = &pv100a_aif1_ops,
		/* set rt5616 as slave */
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
	},
};

static struct snd_soc_card snd_soc_card_pv100a = {
	.name = "snd-pv100a-card",
	.owner = THIS_MODULE,
	.dai_link = pv100a_dailink,
	.num_links = ARRAY_SIZE(pv100a_dailink),
	.dapm_widgets = pv100a_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(pv100a_dapm_widgets),
	.dapm_routes = pv100a_audio_map,
	.num_dapm_routes = ARRAY_SIZE(pv100a_audio_map),
	.controls = pv100a_mc_controls,
	.num_controls = ARRAY_SIZE(pv100a_mc_controls),
};

static int snd_pv100a_mc_probe(struct platform_device *pdev)
{
	struct snd_soc_dai_link *dai = &pv100a_dailink[0];
	struct snd_soc_card *card = &snd_soc_card_pv100a;
	struct device *dev = &pdev->dev;
	struct i2c_client *codec_dev;
	struct device_node *cpu_node;
	struct clk *codec_clk;
	int ret;

	dev_info(&pdev->dev, "%s\n", __func__);

	if (!dev) {
		dev_err(dev, "pv100-audio-card: No device for this platform_device!\n");
		return -EINVAL;
	}

	card->dev = dev;

	if (!dev->of_node) {
		dev_err(dev, "This device requires a devicetree node!\n");
		return -EINVAL;
	}

	cpu_node = of_parse_phandle(dev->of_node, "audio-cpu", 0);

	/* Parse audio-cpu name */
	if (!cpu_node) {
		dev_err(dev, "Parseing 'audio-cpu' failed!\n");
		return -EINVAL;
	}


	/* DAI Links */
	dai->platform_of_node = cpu_node;
	dai->cpu_of_node = cpu_node;
	/* Parse audio-codec nam e*/
	dai->codec_of_node = of_parse_phandle(dev->of_node, "audio-codec", 0);
	if (!dai->codec_of_node) {
		dev_err(dev, "Parsing 'audio-codec' failed!\n");
		return -EINVAL;
	}

	/* Find codec node */
	codec_dev = of_find_i2c_device_by_node(dai->codec_of_node);
	if (!codec_dev) {
		dev_err(dev, "Can't find codec device!\n");
		return -EINVAL;
	}
	/* Lookup and obtain reference to codec clock, and then set codec clock source */
	codec_clk = clk_get(&codec_dev->dev, NULL);
	if (IS_ERR(codec_clk)) {
		dev_warn(dev, "Can't obtain clock!\n");
	} else {
		codec_clock = clk_get_rate(codec_clk);
		clk_put(codec_clk);
		dev_info(dev, "Set codec clock source to %ld\n", codec_clock);
	}

	//platform_set_drvdata(pdev, card);
	ret = devm_snd_soc_register_card(dev, card);
	if (ret)
		dev_err(dev, "%s register card failed %d\n",	__func__, ret);
	else
		dev_info(&pdev->dev, "snd_soc_register_card successful\n");
	return ret;
}

static const struct of_device_id pv100a_rt5616_of_match[] = {
	{ .compatible = "asus,pv100a-audio-card", },
	{},
};

MODULE_DEVICE_TABLE(of, pv100a_rt5616_of_match);

static struct platform_driver snd_pv100a_mc_driver = {
	.probe = snd_pv100a_mc_probe,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = pv100a_rt5616_of_match,
#ifdef CONFIG_PM
		.pm = &snd_soc_pm_ops,
#endif
	},
};


static int __init snd_pv100a_mc_driver_init(void)
{
	return platform_driver_register(&snd_pv100a_mc_driver);
}

static void __exit snd_pv100a_mc_driver_exit(void)
{
	platform_driver_unregister(&snd_pv100a_mc_driver);
}

module_init(snd_pv100a_mc_driver_init);
module_exit(snd_pv100a_mc_driver_exit);

MODULE_AUTHOR("kengyen chen <kengyen_chen@asus.com>");
MODULE_DESCRIPTION("ASoC Driver for PV100A");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
