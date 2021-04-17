// SPDX-License-Identifier: GPL-2.0+
/*
 * p4note device family audio support
 *
 * This file is based on midas_wm1811.c
 * 
 * Original author:
 * Simon Shields <simon@lineageos.org>
 *
 * Additional work by:
 * Martin Jücker <martin.juecker@gmail.com>
 *
 */

#include <linux/clk.h>
#include <linux/mfd/wm8994/registers.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>

#include "i2s.h"
#include "i2s-regs.h"
#include "../codecs/wm8994.h"

#define XTAL_24MHZ_AP 24000000
#define CODEC_CLK32K 32768
#define CODEC_DEFAULT_SYNC_CLK 11289600

struct p4note_machine_priv {
	struct device *dev;
	struct snd_soc_codec *codec;
	struct clk *codec_mclk1;
	struct regulator *reg_mic_bias;
	struct gpio_desc *gpio_lineout_sel;
	unsigned int fll1_rate;
	int aif2_mode;
	int kpcs_mode;
	int input_clamp;
	int lineout_mode;
	int aif2_digital_mute;
	struct wm8958_micd_rate *rates;
	int num_rates;
};

const char *aif2_mode_text[] = {
	"Slave", "Master"
};

const char *kpcs_mode_text[] = {
	"Off", "On"
};

const char *input_clamp_text[] = {
	"Off", "On"
};

const char *lineout_mode_text[] = {
	"Off", "On"
};

const char *switch_mode_text[] = {
	"Off", "On"
};

static const struct soc_enum aif2_mode_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(aif2_mode_text),
			    aif2_mode_text),
};

static const struct soc_enum kpcs_mode_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(kpcs_mode_text),
			    kpcs_mode_text),
};

static const struct soc_enum input_clamp_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(input_clamp_text),
			    input_clamp_text),
};

static const struct soc_enum lineout_mode_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(lineout_mode_text),
			    lineout_mode_text),
};

static const struct soc_enum switch_mode_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(switch_mode_text),
			    switch_mode_text),
};

static int p4note_start_fll1(struct snd_soc_pcm_runtime *rtd, unsigned int new_rate)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dai *aif1_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct p4note_machine_priv *priv = snd_soc_card_get_drvdata(card);
	int ret;

	ret = snd_soc_dai_set_pll(aif1_dai, WM8994_FLL1,
				  WM8994_FLL_SRC_MCLK1, XTAL_24MHZ_AP, new_rate);
	if (ret < 0) {
		dev_err(card->dev, "Failed to set FLL1 rate: %d\n", ret);
		return ret;
	}
	priv->fll1_rate = new_rate;

	ret = snd_soc_dai_set_sysclk(aif1_dai, WM8994_SYSCLK_FLL1,
				     priv->fll1_rate, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(card->dev, "Failed to set SYSCLK source: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_OPCLK,
				     0, MOD_OPCLK_PCLK);
	if (ret < 0) {
		dev_err(card->dev, "Failed to set OPCLK src: %d\n", ret);
		return ret;
	}

	dev_dbg(card->dev, "Started FLL1\n");
	return 0;
}

static int p4note_aif1_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd	= substream->private_data;
	struct snd_soc_dai *aif1_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	unsigned int pll_out;
	int ret;

	dev_info(aif1_dai->dev,
			"AIF1 DAI %s params ch %d, rate %d as i2s slave\n",
			((substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
			 "playback" : "capture"),
			params_channels(params),
			params_rate(params));

	/* AIF1CLK should be at least 3MHz for "optimal performance" */
	if (params_rate(params) == 8000 || params_rate(params) == 11025)
		pll_out = params_rate(params) * 512;
	else
		pll_out = params_rate(params) * 256;

	ret = snd_soc_dai_set_fmt(aif1_dai, SND_SOC_DAIFMT_I2S
					| SND_SOC_DAIFMT_NB_NF
					| SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* Set the cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S
					| SND_SOC_DAIFMT_NB_NF
					| SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	ret = p4note_start_fll1(rtd, pll_out);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops p4note_aif1_ops = {
	.hw_params = p4note_aif1_hw_params,
};

static int p4note_aif2_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd	= substream->private_data;
	struct snd_soc_dai *aif2_dai = asoc_rtd_to_codec(rtd, 1);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 1);
	struct snd_soc_card *card = cpu_dai->component->card;
	struct p4note_machine_priv *priv = snd_soc_card_get_drvdata(card);
	int ret;
	int prate;
	int bclk;

	dev_info(aif2_dai->dev, "%s ++\n", __func__);
	prate = params_rate(params);
	switch (params_rate(params)) {
	case 8000:
	case 16000:
		break;
	default:
		dev_warn(aif2_dai->dev, "Unsupported LRCLK %d, falling back to 8000Hz\n",
				(int)params_rate(params));
		prate = 8000;
	}

	/* Set the codec DAI configuration, aif2_mode:0 is slave */
	if (priv->aif2_mode == 0)
		ret = snd_soc_dai_set_fmt(aif2_dai, SND_SOC_DAIFMT_I2S
					| SND_SOC_DAIFMT_NB_NF
					| SND_SOC_DAIFMT_CBS_CFS);
	else
		ret = snd_soc_dai_set_fmt(aif2_dai, SND_SOC_DAIFMT_I2S
					| SND_SOC_DAIFMT_NB_NF
					| SND_SOC_DAIFMT_CBM_CFM);

	if (ret < 0)
		return ret;

	switch (prate) {
	case 8000:
		bclk = 256000;
		break;
	case 16000:
		bclk = 512000;
		break;
	default:
		return -EINVAL;
	}

	if (priv->aif2_mode == 0) {
		ret = snd_soc_dai_set_pll(aif2_dai, WM8994_FLL2,
					WM8994_FLL_SRC_BCLK,
					bclk, prate * 256);
	} else {
		ret = snd_soc_dai_set_pll(aif2_dai, WM8994_FLL2,
					  WM8994_FLL_SRC_MCLK1,
					  XTAL_24MHZ_AP, prate * 256);
	}

	if (ret < 0)
		dev_err(aif2_dai->dev, "Unable to configure FLL2: %d\n", ret);

	ret = snd_soc_dai_set_sysclk(aif2_dai, WM8994_SYSCLK_FLL2,
				     prate * 256, SND_SOC_CLOCK_IN);
	if (ret < 0)
		dev_err(aif2_dai->dev, "Unable to switch to FLL2: %d\n", ret);

	if (!(snd_soc_component_read(aif2_dai->component, WM8994_INTERRUPT_RAW_STATUS_2)
		& WM8994_FLL2_LOCK_STS)) {
		dev_info(aif2_dai->dev, "%s: use mclk1 for FLL2\n", __func__);
		ret = snd_soc_dai_set_pll(aif2_dai, WM8994_FLL2,
			WM8994_FLL_SRC_MCLK1,
			XTAL_24MHZ_AP, prate * 256);
	}

	dev_info(aif2_dai->dev, "%s --\n", __func__);
	return 0;
}

static struct snd_soc_ops p4note_aif2_ops = {
	.hw_params = p4note_aif2_hw_params,
};

static int get_aif2_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct p4note_machine_priv *priv = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = priv->aif2_mode;
	return 0;
}

static int set_aif2_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct p4note_machine_priv *priv = snd_soc_component_get_drvdata(component);

	if (priv->aif2_mode == ucontrol->value.integer.value[0])
		return 0;

	priv->aif2_mode = ucontrol->value.integer.value[0];

	dev_info(priv->dev, "set aif2 mode : %s\n", aif2_mode_text[priv->aif2_mode]);

	return 0;
}

static int get_kpcs_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct p4note_machine_priv *priv = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = priv->kpcs_mode;
	return 0;
}

static int set_kpcs_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct p4note_machine_priv *priv = snd_soc_component_get_drvdata(component);

	priv->kpcs_mode = ucontrol->value.integer.value[0];

	dev_info(priv->dev, "set kpcs mode : %d\n", priv->kpcs_mode);

	return 0;
}

static int get_input_clamp(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct p4note_machine_priv *priv = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = priv->input_clamp;
	return 0;
}

static int set_input_clamp(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct p4note_machine_priv *priv = snd_soc_component_get_drvdata(component);

	priv->input_clamp = ucontrol->value.integer.value[0];

	if (priv->input_clamp) {
		snd_soc_component_update_bits(component, WM8994_INPUT_MIXER_1,
				WM8994_INPUTS_CLAMP, WM8994_INPUTS_CLAMP);
		msleep(100);
	} else {
		snd_soc_component_update_bits(component, WM8994_INPUT_MIXER_1,
				WM8994_INPUTS_CLAMP, 0);
	}
	dev_info(priv->dev, "set fm input_clamp : %s\n", input_clamp_text[priv->input_clamp]);

	return 0;
}

static int get_lineout_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct p4note_machine_priv *priv = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = priv->lineout_mode;
	return 0;
}

static int set_lineout_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct p4note_machine_priv *priv = snd_soc_component_get_drvdata(component);

	priv->lineout_mode = ucontrol->value.integer.value[0];
	dev_dbg(priv->dev, "set lineout mode : %s\n",
		lineout_mode_text[priv->lineout_mode]);
	return 0;

}

static int get_aif2_mute_status(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct p4note_machine_priv *priv = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = priv->aif2_digital_mute;
	return 0;
}

static int set_aif2_mute_status(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct p4note_machine_priv *priv = snd_soc_component_get_drvdata(component);
	int reg;

	priv->aif2_digital_mute = ucontrol->value.integer.value[0];

	if (snd_soc_component_read(component, WM8994_POWER_MANAGEMENT_6)
		& WM8994_AIF2_DACDAT_SRC)
		priv->aif2_digital_mute = 0;

	if (priv->aif2_digital_mute)
		reg = WM8994_AIF1DAC1_MUTE;
	else
		reg = 0;

	snd_soc_component_update_bits(component, WM8994_AIF2_DAC_FILTERS_1,
				WM8994_AIF1DAC1_MUTE, reg);

	dev_info(priv->dev, "set aif2_digital_mute : %s\n",
			switch_mode_text[priv->aif2_digital_mute]);

	return 0;
}

static int p4note_mic_bias(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = w->dapm->card;
	struct p4note_machine_priv *priv = snd_soc_card_get_drvdata(card);

	int err = 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		dev_info(priv->dev, "mic bias pre pmu");
		err = regulator_enable(priv->reg_mic_bias);
		msleep(150);
		break;
	case SND_SOC_DAPM_POST_PMD:
		dev_info(priv->dev, "mic bias post pmd");
		return regulator_disable(priv->reg_mic_bias);
	}

	return err;
}

static int p4note_line_set(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = w->dapm->card;
	struct p4note_machine_priv *priv = snd_soc_card_get_drvdata(card);

	if (!priv->gpio_lineout_sel)
		return 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		gpiod_set_value_cansleep(priv->gpio_lineout_sel, 1);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		gpiod_set_value_cansleep(priv->gpio_lineout_sel, 0);
		break;
	}

	return 0;
}

static const struct snd_kcontrol_new p4note_controls[] = {
	SOC_DAPM_PIN_SWITCH("HP"),

	SOC_DAPM_PIN_SWITCH("SPK"),
	SOC_DAPM_PIN_SWITCH("RCV"),

	SOC_DAPM_PIN_SWITCH("LINE"),
	SOC_DAPM_PIN_SWITCH("HDMI"),

	SOC_DAPM_PIN_SWITCH("Main Mic"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),

	SOC_ENUM_EXT("AIF2 Mode", aif2_mode_enum[0],
		get_aif2_mode, set_aif2_mode),

	SOC_ENUM_EXT("KPCS Mode", kpcs_mode_enum[0],
		get_kpcs_mode, set_kpcs_mode),

	SOC_ENUM_EXT("Input Clamp", input_clamp_enum[0],
		get_input_clamp, set_input_clamp),

	SOC_ENUM_EXT("LineoutSwitch Mode", lineout_mode_enum[0],
		get_lineout_mode, set_lineout_mode),

	SOC_ENUM_EXT("AIF2 digital mute", switch_mode_enum[0],
		get_aif2_mute_status, set_aif2_mute_status),
};

static const struct snd_soc_dapm_widget p4note_dapm_widgets[] = {
	SND_SOC_DAPM_HP("HP", NULL),

	SND_SOC_DAPM_SPK("SPK", NULL),
	SND_SOC_DAPM_SPK("RCV", NULL),

	SND_SOC_DAPM_LINE("LINE", p4note_line_set),
	SND_SOC_DAPM_LINE("HDMI", NULL),

	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Main Mic", p4note_mic_bias),

	SND_SOC_DAPM_INPUT("S5P RP"),
};

static int p4note_late_probe(struct snd_soc_card *card) {
	struct snd_soc_pcm_runtime *rtd = snd_soc_get_pcm_runtime(card,
			&card->dai_link[0]);
	struct snd_soc_dai *aif1_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_component *component = aif1_dai->component;
	struct p4note_machine_priv *priv = snd_soc_card_get_drvdata(card);
	int ret;

	ret = clk_prepare_enable(priv->codec_mclk1);
	if (ret < 0) {
		dev_err(component->dev, "Failed to enable mclk1: %d\n", ret);
		return ret;
	}

	/* Switch the FLL */
	ret = snd_soc_dai_set_pll(aif1_dai,
					WM8994_FLL1,
					WM8994_FLL_SRC_MCLK1,
					XTAL_24MHZ_AP,
					CODEC_DEFAULT_SYNC_CLK);

	if (ret < 0)
		dev_err(aif1_dai->dev, "Unable to start FLL1: %d\n", ret);

	/* Use FLL1 as SYSCLK for boot */
	ret = snd_soc_dai_set_sysclk(aif1_dai,
					WM8994_SYSCLK_FLL1,
					CODEC_DEFAULT_SYNC_CLK,
					SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(aif1_dai->dev, "Failed to set FLL1: %d\n", ret);
		return ret;
	}

	snd_soc_dapm_force_enable_pin(&card->dapm, "AIF1CLK");

	return 0;
}

static struct snd_soc_dai_driver p4note_ext_dai[] = {
	{
		.name = "Voice call",
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 16000,
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 16000,
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
	{
		.name = "Bluetooth",
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 16000,
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 16000,
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
};

static const struct snd_soc_component_driver p4note_component = {
	.name	= "p4note-audio",
};

SND_SOC_DAILINK_DEFS(aif1,
	DAILINK_COMP_ARRAY(COMP_CPU(SAMSUNG_I2S_DAI)),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "wm8994-aif1")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(aif2,
	DAILINK_COMP_ARRAY(COMP_CPU(SAMSUNG_I2S_DAI)),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "wm8994-aif2")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(aif3,
	DAILINK_COMP_ARRAY(COMP_CPU(SAMSUNG_I2S_DAI)),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "wm8994-aif3")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link p4note_dai[] = {
	{
		.name = "WM8994 AIF1",
		.stream_name = "HiFi Primary",
		.ops = &p4note_aif1_ops,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM,
		SND_SOC_DAILINK_REG(aif1),
	},
	{
		.name = "WM1811 Voice",
		.stream_name = "Voice call",
		.ops = &p4note_aif2_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(aif2),
	},
	{
		.name = "WM1811 BT",
		.stream_name = "Bluetooth",
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(aif3),
	},
};

static struct snd_soc_card p4note_card = {
	.name = "P4Note WM1811",
	.owner = THIS_MODULE,

	.dai_link = p4note_dai,
	.num_links = ARRAY_SIZE(p4note_dai),
	.controls = p4note_controls,
	.num_controls = ARRAY_SIZE(p4note_controls),
	.dapm_widgets = p4note_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(p4note_dapm_widgets),

	.late_probe = p4note_late_probe,
};

static int p4note_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card = &p4note_card;
	struct p4note_machine_priv *priv;
	static struct snd_soc_dai_link *dai_link;
	struct device_node *cpu_dai_node, *codec_dai_node;
	int ret, i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	snd_soc_card_set_drvdata(card, priv);
	card->dev = dev;
	priv->dev = dev;

	priv->reg_mic_bias = devm_regulator_get(dev, "mic-bias");
	if (IS_ERR(priv->reg_mic_bias)) {
		dev_err(dev, "Failed to get mic bias regulator\n");
		return PTR_ERR(priv->reg_mic_bias);
	}

	priv->gpio_lineout_sel = devm_gpiod_get_optional(dev, "lineout-sel", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->gpio_lineout_sel)) {
		dev_err(dev, "Failed to get line out selection GPIO\n");
		return PTR_ERR(priv->gpio_lineout_sel);
	}

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret < 0) {
		dev_err(dev, "Card name is not specified\n");
		return ret;
	}

	ret = snd_soc_of_parse_audio_routing(card, "samsung,audio-routing");
	if (ret < 0) {
		dev_err(dev, "Audio routing invalid/unspecified\n");
		return ret;
	}

	cpu_dai_node = of_parse_phandle(dev->of_node, "i2s-controller", 0);
	if (!cpu_dai_node) {
		dev_err(dev, "i2s-controllers property invalid/missing\n");
		return -EINVAL;
	}

	codec_dai_node = of_parse_phandle(dev->of_node, "audio-codec", 0);
	if (!codec_dai_node) {
		dev_err(dev, "audio-codec property invalid/missing\n");
		ret = -EINVAL;
		goto put_cpu_dai_node;
	}

	for_each_card_prelinks(card, i, dai_link) {
		dai_link->codecs->of_node = codec_dai_node;
		dai_link->cpus->of_node = cpu_dai_node;
		dai_link->platforms->of_node = cpu_dai_node;
	}

	priv->codec_mclk1 = of_clk_get_by_name(codec_dai_node, "MCLK1");
	if (IS_ERR(priv->codec_mclk1)) {
		ret = PTR_ERR(priv->codec_mclk1);
		dev_err(dev, "Failed to get MCLK1: %d\n", ret);
		goto put_codec_dai_node;
	}

	ret = devm_snd_soc_register_component(dev, &p4note_component,
			p4note_ext_dai, ARRAY_SIZE(p4note_ext_dai));
	if (ret < 0) {
		dev_err(dev, "Failed to register component: %d\n", ret);
		goto put_codec_mclk1;
	}

	ret = devm_snd_soc_register_card(dev, card);
	if (ret < 0) {
		dev_err(dev, "Failed to register card: %d\n", ret);
		goto put_codec_mclk1;
	}

	priv->kpcs_mode = 2;

	return 0;

put_codec_mclk1:
	clk_put(priv->codec_mclk1);
put_codec_dai_node:
	of_node_put(codec_dai_node);
put_cpu_dai_node:
	of_node_put(cpu_dai_node);
	return ret;
}

static const struct of_device_id p4note_of_match[] = {
	{ .compatible = "samsung,p4note-audio" },
	{ },
};
MODULE_DEVICE_TABLE(of, p4note_of_match);

static struct platform_driver p4note_driver = {
	.driver = {
		.name = "p4note-audio",
		.of_match_table = p4note_of_match,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = p4note_probe,
};
module_platform_driver(p4note_driver);

MODULE_DESCRIPTION("ASoC support for p4note device family");
MODULE_LICENSE("GPL v2");