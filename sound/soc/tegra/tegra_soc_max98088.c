/*
 * tegra_soc_max98088.c  --  SoC audio for tegra
 *
 * Copyright  2010-2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define DEBUG

#include <linux/gpio.h>
#include "tegra_soc.h"
#include "../codecs/max98088.h"

static struct platform_device *tegra_snd_device;

extern struct snd_soc_dai tegra_i2s_dai[];
extern struct snd_soc_dai tegra_spdif_dai;
extern struct snd_soc_dai tegra_generic_codec_dai[];
extern struct snd_soc_platform tegra_soc_platform;
extern struct wired_jack_conf tegra_wired_jack_conf;

void speaker_settings(struct snd_soc_codec *codec, int value)
{
}

static int tegra_hifi_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int sys_clk;
	int err;

#if 0
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	int dai_flag = 0;
	enum dac_dap_data_format data_fmt;

	if (tegra_das_is_port_master(tegra_audio_codec_type_hifi))
		dai_flag |= SND_SOC_DAIFMT_CBM_CFM;
	else
#endif
		dai_flag |= SND_SOC_DAIFMT_CBS_CFS;

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	data_fmt = tegra_das_get_codec_data_fmt(tegra_audio_codec_type_hifi);

	/* We are supporting DSP and I2s format for now */
	if (data_fmt & dac_dap_data_format_dsp)
		dai_flag |= SND_SOC_DAIFMT_DSP_A;
	else
#endif
		dai_flag |= SND_SOC_DAIFMT_I2S;

	err = snd_soc_dai_set_fmt(codec_dai, dai_flag);
	if (err < 0) {
		pr_err("codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, dai_flag);
	if (err < 0) {
		pr_err("cpu_dai fmt not set\n");
		return err;
	}
#endif

	err = snd_soc_dai_set_fmt(codec_dai,
					SND_SOC_DAIFMT_I2S | \
					SND_SOC_DAIFMT_NB_NF | \
					SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		pr_err("codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai,
					SND_SOC_DAIFMT_I2S | \
					SND_SOC_DAIFMT_NB_NF | \
					SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		pr_err("cpu_dai fmt not set\n");
		return err;
	}

	/*FIXME: not sure this is the right way.
	This should be samplerate times 256 or 128 based on codec need*/
	sys_clk = tegra_das_get_mclk_rate();
	err = snd_soc_dai_set_sysclk(codec_dai, 0, sys_clk, SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("codec_dai clock not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(cpu_dai, 0, sys_clk, SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("cpu_dai clock not set\n");
		return err;
	}

	return 0;
}

static int tegra_voice_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int dai_flag = 0, sys_clk;
	int err;

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	enum dac_dap_data_format data_fmt;

	if (tegra_das_is_port_master(tegra_audio_codec_type_bluetooth))
		dai_flag |= SND_SOC_DAIFMT_CBM_CFM;
	else
#endif
		dai_flag |= SND_SOC_DAIFMT_CBS_CFS;

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	data_fmt = tegra_das_get_codec_data_fmt(tegra_audio_codec_type_bluetooth);

	/* We are supporting DSP and I2s format for now */
	if (data_fmt & dac_dap_data_format_i2s)
		dai_flag |= SND_SOC_DAIFMT_I2S;
	else
#endif
		dai_flag |= SND_SOC_DAIFMT_DSP_A;

	err = snd_soc_dai_set_fmt(codec_dai, dai_flag);
	if (err < 0) {
		pr_err("codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, dai_flag);
	if (err < 0) {
		pr_err("cpu_dai fmt not set\n");
		return err;
	}

	sys_clk = tegra_das_get_mclk_rate();
	err = snd_soc_dai_set_sysclk(codec_dai, 0, sys_clk, SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("cpu_dai clock not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(cpu_dai, 0, sys_clk, SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("cpu_dai clock not set\n");
		return err;
	}

	return 0;
}

static int tegra_spdif_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	return 0;
}

int tegra_codec_startup(struct snd_pcm_substream *substream)
{
	tegra_das_power_mode(true);

	return 0;
}

void tegra_codec_shutdown(struct snd_pcm_substream *substream)
{
	tegra_das_power_mode(false);
}

int tegra_soc_suspend_pre(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

int tegra_soc_suspend_post(struct platform_device *pdev, pm_message_t state)
{
	tegra_das_disable_mclk();

	return 0;
}

int tegra_soc_resume_pre(struct platform_device *pdev)
{
	tegra_das_enable_mclk();

	return 0;
}

int tegra_soc_resume_post(struct platform_device *pdev)
{
	return 0;
}

static struct snd_soc_ops tegra_hifi_ops = {
	.hw_params = tegra_hifi_hw_params,
	.startup = tegra_codec_startup,
	.shutdown = tegra_codec_shutdown,
};

static struct snd_soc_ops tegra_voice_ops = {
	.hw_params = tegra_voice_hw_params,
	.startup = tegra_codec_startup,
	.shutdown = tegra_codec_shutdown,
};

static struct snd_soc_ops tegra_spdif_ops = {
	.hw_params = tegra_spdif_hw_params,
};

void tegra_ext_control(struct snd_soc_codec *codec, int new_con)
{
	struct tegra_audio_data *audio_data = codec->socdev->codec_data;

	pr_info("%s: new_con 0x%X #####################\n", __func__, new_con);

	/* Disconnect old codec routes and connect new routes*/
	if (new_con & TEGRA_HEADPHONE)
		snd_soc_dapm_enable_pin(codec, "Headphone");
	else
		snd_soc_dapm_disable_pin(codec, "Headphone");

	if (new_con & (TEGRA_LINEOUT | TEGRA_EAR_SPK))
		snd_soc_dapm_enable_pin(codec, "Lineout");
	else
		snd_soc_dapm_disable_pin(codec, "Lineout");

	if (new_con & TEGRA_SPK)
		snd_soc_dapm_enable_pin(codec, "Int Spk");
	else
		snd_soc_dapm_disable_pin(codec, "Int Spk");

	if (new_con & TEGRA_INT_MIC)
		snd_soc_dapm_enable_pin(codec, "Int Mic");
	else
		snd_soc_dapm_disable_pin(codec, "Int Mic");

	if (new_con & TEGRA_EXT_MIC)
		snd_soc_dapm_enable_pin(codec, "Ext Mic");
	else
		snd_soc_dapm_disable_pin(codec, "Ext Mic");

	if (new_con & TEGRA_LINEIN)
		snd_soc_dapm_enable_pin(codec, "Linein");
	else
		snd_soc_dapm_disable_pin(codec, "Linein");

	if (new_con & TEGRA_HEADSET)
		snd_soc_dapm_enable_pin(codec, "Headset");
	else
		snd_soc_dapm_disable_pin(codec, "Headset");

	audio_data->codec_con = new_con;

	snd_soc_dapm_enable_pin(codec, "Headphone");

	/* signal a DAPM event */
	snd_soc_dapm_sync(codec);

}

static int tegra_dapm_event_int_spk(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *k, int event)
{
	if (tegra_wired_jack_conf.en_spkr != -1) {
		if (tegra_wired_jack_conf.amp_reg) {
			if (SND_SOC_DAPM_EVENT_ON(event) &&
				!tegra_wired_jack_conf.amp_reg_enabled) {
				regulator_enable(tegra_wired_jack_conf.amp_reg);
				tegra_wired_jack_conf.amp_reg_enabled = 1;
			} else if (!SND_SOC_DAPM_EVENT_ON(event) &&
				tegra_wired_jack_conf.amp_reg_enabled) {
				regulator_disable(tegra_wired_jack_conf.amp_reg);
				tegra_wired_jack_conf.amp_reg_enabled = 0;
			}
		}

		gpio_set_value_cansleep(tegra_wired_jack_conf.en_spkr,
			SND_SOC_DAPM_EVENT_ON(event) ? 1 : 0);
	}

	return 0;
}

static int tegra_dapm_event_int_mic(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *k, int event)
{
	if (tegra_wired_jack_conf.en_mic_int != -1)
		gpio_set_value_cansleep(tegra_wired_jack_conf.en_mic_int,
			SND_SOC_DAPM_EVENT_ON(event) ? 1 : 0);

	if (tegra_wired_jack_conf.en_mic_ext != -1)
		gpio_set_value_cansleep(tegra_wired_jack_conf.en_mic_ext,
			SND_SOC_DAPM_EVENT_ON(event) ? 0 : 1);

	return 0;
}

static int tegra_dapm_event_ext_mic(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *k, int event)
{
	if (tegra_wired_jack_conf.en_mic_ext != -1)
		gpio_set_value_cansleep(tegra_wired_jack_conf.en_mic_ext,
			SND_SOC_DAPM_EVENT_ON(event) ? 1 : 0);

	if (tegra_wired_jack_conf.en_mic_int != -1)
		gpio_set_value_cansleep(tegra_wired_jack_conf.en_mic_int,
			SND_SOC_DAPM_EVENT_ON(event) ? 0 : 1);

	return 0;
}

/*tegra machine dapm widgets */
static const struct snd_soc_dapm_widget tegra_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_HP("Headset", NULL),
	SND_SOC_DAPM_SPK("Lineout", NULL),
	SND_SOC_DAPM_SPK("Int Spk", tegra_dapm_event_int_spk),
	SND_SOC_DAPM_MIC("Ext Mic", tegra_dapm_event_ext_mic),
	SND_SOC_DAPM_MIC("Int Mic", tegra_dapm_event_int_mic),
	SND_SOC_DAPM_LINE("Linein", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Headphone connected to HPL and HPR */
	{"Headphone", NULL, "HPL"},
	{"Headphone", NULL, "HPR"},

	/* Headset connected to HPL and HPR */
	{"Headset", NULL, "HPL"},
	{"Headset", NULL, "HPR"},

	/* Speaker connected to SPKL and SPKR */
	{"Int Spk", NULL, "SPKL"},
	{"Int Spk", NULL, "SPKR"},

	/* Receiver connected to RECL and RECR */
	{"Lineout", NULL, "RECL"},
	{"Lineout", NULL, "RECR"},

	/*
	 * To be complete, add remained in/out devices such as built-in mic
	 * and headset mic.
	 */
};
#if 0
/* Tegra machine audio map (connections to the codec pins) */
static const struct snd_soc_dapm_route audio_map[] = {

	/* headphone connected to LHPOUT1, RHPOUT1 */
	{"Headphone", NULL, "HPOUTR"},
	{"Headphone", NULL, "HPOUTL"},

	/* headset Jack  - in = micin, out = HPOUT*/
	{"Headset", NULL, "HPOUTR"},
	{"Headset", NULL, "HPOUTL"},
	{"IN1L", NULL, "Headset"},
	{"IN1R", NULL, "Headset"},

	/* lineout connected to LINEOUTR and LINEOUTL */
	{"Lineout", NULL, "LINEOUTR"},
	{"Lineout", NULL, "LINEOUTL"},

	/* build-in speaker connected to LON/P RON/P */
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "LON"},
	{"Int Spk", NULL, "LOP"},

	/* internal mic is mono */
	{"IN1R", NULL, "Int Mic"},

	/* external mic is stero */
	{"IN1L", NULL, "Ext Mic"},
	{"IN1R", NULL, "Ext Mic"},

	/* Line In */
	{"IN3L", NULL, "Linein"},
	{"IN3R", NULL, "Linein"},
};
#endif


static int tegra_codec_init(struct snd_soc_codec *codec)
{
	struct tegra_audio_data *audio_data = codec->socdev->codec_data;
	int err = 0;

	if (!audio_data->init_done) {

		err = tegra_das_open();
		if (err) {
			pr_err(" Failed get dap mclk\n");
			err = -ENODEV;
			goto wm8903_init_fail;
		}

		err = tegra_das_enable_mclk();
		if (err) {
			pr_err(" Failed to enable dap mclk\n");
			err = -ENODEV;
			goto wm8903_init_fail;
		}

		/* Add tegra specific widgets */
		snd_soc_dapm_new_controls(codec, tegra_dapm_widgets,
					ARRAY_SIZE(tegra_dapm_widgets));

		/* Set up tegra specific audio path audio_map */
		snd_soc_dapm_add_routes(codec, audio_map,
					ARRAY_SIZE(audio_map));

		/* Add jack detection */
		err = tegra_jack_init(codec);
		if (err < 0) {
			pr_err("Failed in jack init\n");
			goto wm8903_init_fail;
		}

		/* Default to OFF */
		tegra_ext_control(codec, TEGRA_SPK);/* TEGRA_AUDIO_OFF); */

		err = tegra_controls_init(codec);
		if (err < 0) {
			pr_err("Failed in controls init\n");
			goto wm8903_init_fail;
		}

		audio_data->codec = codec;
		audio_data->init_done = 1;
		/* snd_soc_dapm_enable_pin(codec, "Headphone"); */

	}

	return err;

wm8903_init_fail:

	tegra_das_disable_mclk();
	tegra_das_close();
	return err;
}


#define TEGRA_CREATE_SOC_DAI_LINK(xname, xstreamname,	\
			xcpudai, xcodecdai, xops)	\
{							\
	.name = xname,					\
	.stream_name = xstreamname,			\
	.cpu_dai = xcpudai,				\
	.codec_dai = xcodecdai,				\
	.init = tegra_codec_init,			\
	.ops = xops,					\
}

static struct snd_soc_dai_link tegra_soc_dai[] = {
	TEGRA_CREATE_SOC_DAI_LINK("MAX98088", "MAX98088 HiFi",
				  &tegra_i2s_dai[0], &max98088_dai[0],
				  &tegra_hifi_ops),

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
	TEGRA_CREATE_SOC_DAI_LINK("Tegra-generic", "Tegra Generic Voice",
		&tegra_i2s_dai[1], &tegra_generic_codec_dai[1],
		&tegra_voice_ops),
#else
	/*
	TEGRA_CREATE_SOC_DAI_LINK("Tegra-generic-0", "Tegra BB Voice",
		&tegra_i2s_dai[1], &tegra_generic_codec_dai[1],
		&tegra_voice_ops),
	TEGRA_CREATE_SOC_DAI_LINK("Tegra-generic-1", "Tegra BT Voice",
		&tegra_i2s_dai[2], &tegra_generic_codec_dai[2],
		&tegra_voice_ops),
	*/

#endif

	TEGRA_CREATE_SOC_DAI_LINK("Tegra-spdif", "Tegra Spdif",
		&tegra_spdif_dai, &tegra_generic_codec_dai[0],
		&tegra_spdif_ops),
};

static struct tegra_audio_data audio_data = {
	.init_done = 0,
	.play_device = TEGRA_AUDIO_DEVICE_NONE,
	.capture_device = TEGRA_AUDIO_DEVICE_NONE,
	.is_call_mode = false,
	.codec_con = TEGRA_AUDIO_OFF,
};

static struct snd_soc_card tegra_snd_soc = {
	.name = "tegra",
	.platform = &tegra_soc_platform,
	.dai_link = tegra_soc_dai,
	.num_links = ARRAY_SIZE(tegra_soc_dai),
	.suspend_pre = tegra_soc_suspend_pre,
	.suspend_post = tegra_soc_suspend_post,
	.resume_pre = tegra_soc_resume_pre,
	.resume_post = tegra_soc_resume_post,
};

static struct snd_soc_device tegra_snd_devdata = {
	.card = &tegra_snd_soc,
	.codec_dev = &soc_codec_dev_max98088,
	.codec_data = &audio_data,
};

static int __init tegra_init(void)
{
	int ret = 0;

	tegra_snd_device = platform_device_alloc("soc-audio", -1);
	if (!tegra_snd_device) {
		pr_err("failed to allocate soc-audio\n");
		return -ENOMEM;
	}

	platform_set_drvdata(tegra_snd_device, &tegra_snd_devdata);
	tegra_snd_devdata.dev = &tegra_snd_device->dev;

	ret = platform_device_add(tegra_snd_device);
	if (ret) {
		pr_err("audio device could not be added\n");
		goto fail;
	}

	return 0;

fail:
	if (tegra_snd_device) {
		platform_device_put(tegra_snd_device);
		tegra_snd_device = 0;
	}

	return ret;
}

static void __exit tegra_exit(void)
{
	tegra_jack_exit();
	platform_device_unregister(tegra_snd_device);
}

module_init(tegra_init);
module_exit(tegra_exit);

/* Module information */
MODULE_DESCRIPTION("Tegra ALSA SoC for MAX98088");
MODULE_LICENSE("GPL");
