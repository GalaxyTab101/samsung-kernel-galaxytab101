/*
 * tegra_i2s.c  --  ALSA Soc Audio Layer
 *
 * Copyright (c) 2009-2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include "tegra_soc.h"

/* i2s controller */
struct tegra_i2s_info {
	struct platform_device *pdev;
	struct tegra_audio_platform_data *pdata;

	unsigned int bit_format;
	bool i2s_master;
	int ref_count;
	aud_dev_info  i2sdev_info;
	struct das_regs_cache das_regs;
};

extern int tegra_i2sloopback_func;

void free_i2s_dma_request(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct tegra_i2s_info *info = cpu_dai->private_data;

	info->i2sdev_info.fifo_mode = AUDIO_RX_MODE;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		info->i2sdev_info.fifo_mode = AUDIO_TX_MODE;

	am_free_dma_requestor(&info->i2sdev_info);
}

void setup_i2s_dma_request(struct snd_pcm_substream *substream,
			struct tegra_dma_req *req,
			void (*dma_callback)(struct tegra_dma_req *req),
			void *dma_data)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct tegra_i2s_info *info = cpu_dai->private_data;

	info->i2sdev_info.fifo_mode = AUDIO_RX_MODE;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		info->i2sdev_info.fifo_mode = AUDIO_TX_MODE;

	req->req_sel = am_get_dma_requestor(&info->i2sdev_info);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		req->to_memory = false;
		req->dest_addr =
			am_get_fifo_phy_base(&info->i2sdev_info);
		req->dest_wrap = 4;
		req->source_wrap = 0;
		if (info->bit_format == AUDIO_FRAME_FORMAT_DSP)
			req->dest_bus_width = info->pdata->dsp_bus_width;
		else
			req->dest_bus_width = info->pdata->i2s_bus_width;
		req->source_bus_width = 32;
	} else {
		req->to_memory = true;
		req->source_addr =
			am_get_fifo_phy_base(&info->i2sdev_info);
		req->dest_wrap = 0;
		req->source_wrap = 4;
		if (info->bit_format == AUDIO_FRAME_FORMAT_DSP)
			req->source_bus_width = info->pdata->dsp_bus_width;
		else
			req->source_bus_width = info->pdata->i2s_bus_width;
		req->dest_bus_width = 32;
	}

	req->complete = dma_callback;
	req->dev = dma_data;

	return;
}

/* playback */
static inline void start_i2s_playback(struct snd_soc_dai *cpu_dai)
{
	struct tegra_i2s_info *info = cpu_dai->private_data;

	info->i2sdev_info.fifo_mode = AUDIO_TX_MODE;
	am_set_stream_state(&info->i2sdev_info, true);
}

static inline void stop_i2s_playback(struct snd_soc_dai *cpu_dai)
{
	struct tegra_i2s_info *info = cpu_dai->private_data;

	info->i2sdev_info.fifo_mode = AUDIO_TX_MODE;
	am_set_stream_state(&info->i2sdev_info, false);
	while (am_get_status(&info->i2sdev_info));
}

/* recording */
static inline void start_i2s_capture(struct snd_soc_dai *cpu_dai)
{
	struct tegra_i2s_info *info = cpu_dai->private_data;

	info->i2sdev_info.fifo_mode = AUDIO_RX_MODE;
	am_set_stream_state(&info->i2sdev_info, true);
}

static inline void stop_i2s_capture(struct snd_soc_dai *cpu_dai)
{
	struct tegra_i2s_info *info = cpu_dai->private_data;

	info->i2sdev_info.fifo_mode = AUDIO_RX_MODE;
	am_set_stream_state(&info->i2sdev_info, false);
	while (am_get_status(&info->i2sdev_info));
}


static int tegra_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	int val;
	am_stream_format_info fmt;
	struct tegra_i2s_info *info = dai->private_data;

	info->i2sdev_info.fifo_mode = AUDIO_RX_MODE;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		info->i2sdev_info.fifo_mode = AUDIO_TX_MODE;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		val = AUDIO_BIT_SIZE_16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		val = AUDIO_BIT_SIZE_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		val = AUDIO_BIT_SIZE_32;
		break;
	default:
		return -EINVAL;
	}

	fmt.bitsize = val;

	switch (params_rate(params)) {
	case 8000:
	case 32000:
	case 44100:
	case 48000:
	case 88200:
	case 96000:
		val = params_rate(params);
		break;
	default:
		return -EINVAL;
	}

	fmt.samplerate = val;

	switch (params_channels(params)) {
	case 1: val = AUDIO_CHANNEL_1; break;
	case 2: val = AUDIO_CHANNEL_2; break;
	case 3: val = AUDIO_CHANNEL_3; break;
	case 4: val = AUDIO_CHANNEL_4; break;
	case 5: val = AUDIO_CHANNEL_5; break;
	case 6: val = AUDIO_CHANNEL_6; break;
	case 7: val = AUDIO_CHANNEL_7; break;
	case 8: val = AUDIO_CHANNEL_8; break;
	default:
		return -EINVAL;
	}

	fmt.channels = val;
	fmt.buffersize = params_period_bytes(params);

	am_set_stream_format(&info->i2sdev_info, &fmt);

	return 0;
}


static int tegra_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai,
					unsigned int fmt)
{
	int val1, val2;
	am_dev_format_info devfmt;
	struct tegra_i2s_info *info = cpu_dai->private_data;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		info->i2s_master = 1;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		info->i2s_master = 0;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
	case SND_SOC_DAIFMT_CBM_CFS:
		/* Tegra does not support different combinations of
		 * master and slave for FSYNC and BCLK */
	default:
		return -EINVAL;
	}

	devfmt.mastermode = info->i2s_master;

	val2 = AUDIO_LRCK_LEFT_LOW;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		val1 = AUDIO_FRAME_FORMAT_DSP;
		val2 = AUDIO_LRCK_RIGHT_LOW;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		val1 = AUDIO_FRAME_FORMAT_DSP;
		val2 = AUDIO_LRCK_RIGHT_LOW;
		break;
	case SND_SOC_DAIFMT_I2S:
		val1 = AUDIO_FRAME_FORMAT_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		val1 = AUDIO_FRAME_FORMAT_RJM;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val1 = AUDIO_FRAME_FORMAT_LJM;
		break;
	default:
		return -EINVAL;
	}

	devfmt.audiomode = val1;
	devfmt.polarity = val2;
	devfmt.loopmode =
		(tegra_i2sloopback_func == TEGRA_INT_I2SLOOPBACK_ON)? 1 : 0;

	am_set_device_format(&info->i2sdev_info, &devfmt);
	return 0;
}

static int tegra_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
					int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int tegra_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			start_i2s_playback(dai);
		else
			start_i2s_capture(dai);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			stop_i2s_playback(dai);
		else
			stop_i2s_capture(dai);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int i2s_configure(struct tegra_i2s_info *info )
{
	struct platform_device *pdev = info->pdev;
	struct tegra_audio_platform_data *pdata = pdev->dev.platform_data;

	am_stream_format_info strm_fmt;
	am_dev_format_info dev_fmt;

	memset(&dev_fmt, 0, sizeof(dev_fmt));
	dev_fmt.mastermode = pdata->i2s_master;
	dev_fmt.audiomode = pdata->mode;
	dev_fmt.clkrate = pdata->dev_clk_rate;
	dev_fmt.fifofmt = pdata->fifo_fmt;

	memset(&strm_fmt, 0, sizeof(strm_fmt));
	strm_fmt.bitsize = pdata->bit_size;
	strm_fmt.samplerate = pdata->i2s_master_clk;

	am_device_init(&info->i2sdev_info, (void*)&dev_fmt, (void*)&strm_fmt);

	return 0;
}

#ifdef CONFIG_PM
int tegra_i2s_suspend(struct snd_soc_dai *cpu_dai)
{
	struct tegra_i2s_info *info = cpu_dai->private_data;

	am_suspend(&info->i2sdev_info);
	tegra_das_get_all_regs(&info->das_regs);


	return 0;
}

int tegra_i2s_resume(struct snd_soc_dai *cpu_dai)
{
	struct tegra_i2s_info *info = cpu_dai->private_data;


	tegra_das_set_all_regs(&info->das_regs);
	am_resume(&info->i2sdev_info);
	tegra_jack_resume();


	/* disabled clock as it is being enabled back on startup */
	am_clock_disable(&info->i2sdev_info);
	return 0;
}

#else
#define tegra_i2s_suspend	NULL
#define tegra_i2s_resume	NULL
#endif

static int tegra_i2s_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct tegra_i2s_info *info = dai->private_data;

	if (!info->ref_count)
		am_clock_enable(&info->i2sdev_info);

	info->ref_count++;
	return 0;
}

static void tegra_i2s_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct tegra_i2s_info *info = dai->private_data;

	if (info->ref_count > 0)
	    info->ref_count--;

	if (!info->ref_count)
		am_clock_disable(&info->i2sdev_info);

	return;
}

static int tegra_i2s_probe(struct platform_device *pdev,
				struct snd_soc_dai *cpu_dai)
{
	return 0;
}

static struct snd_soc_dai_ops tegra_i2s_dai_ops = {
	.startup	= tegra_i2s_startup,
	.shutdown	= tegra_i2s_shutdown,
	.trigger	= tegra_i2s_trigger,
	.hw_params	= tegra_i2s_hw_params,
	.set_fmt	= tegra_i2s_set_dai_fmt,
	.set_sysclk	= tegra_i2s_set_dai_sysclk,
};

#define TEGRA_I2S_CREATE_DAI(link_id, ch_min, ch_max, sample_rates)	\
{							\
	.name = "tegra-i2s-"#link_id,			\
	.id = (link_id),				\
	.probe = tegra_i2s_probe,			\
	.suspend = tegra_i2s_suspend,			\
	.resume = tegra_i2s_resume,			\
	.playback = {					\
		.channels_min = (ch_min),		\
		.channels_max = (ch_max),		\
		.rates = (sample_rates),		\
		.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
	},						\
	.capture = {					\
		.channels_min = (ch_min),		\
		.channels_max = (ch_max),		\
		.rates = (sample_rates),		\
		.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
	},						\
	.ops = &tegra_i2s_dai_ops,			\
}

struct snd_soc_dai tegra_i2s_dai[] = {
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
	TEGRA_I2S_CREATE_DAI(0, 1, 2, TEGRA_SAMPLE_RATES),
	TEGRA_I2S_CREATE_DAI(1, 1, 2, TEGRA_SAMPLE_RATES),
#else
	TEGRA_I2S_CREATE_DAI(1, 2, 2, TEGRA_SAMPLE_RATES),
	TEGRA_I2S_CREATE_DAI(2, 1, 2, TEGRA_VOICE_SAMPLE_RATES),
	TEGRA_I2S_CREATE_DAI(3, 1, 2, TEGRA_VOICE_SAMPLE_RATES),
#endif
};

EXPORT_SYMBOL_GPL(tegra_i2s_dai);

static int tegra_i2s_driver_probe(struct platform_device *pdev)
{
	int err = 0;
	struct tegra_i2s_info *info;
	int i = 0;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->pdev = pdev;
	info->pdata = pdev->dev.platform_data;
	info->pdata->driver_data = info;
	BUG_ON(!info->pdata);

	info->i2sdev_info.dev_type = AUDIO_I2S_DEVICE;
	info->i2sdev_info.dev_id = pdev->id;

	err = i2s_configure(info);
	if (err) {
		goto fail_clock;
	}

	info->bit_format = info->pdata->mode;


	for (i = 0; i < ARRAY_SIZE(tegra_i2s_dai); i++) {
		if (tegra_i2s_dai[i].id == pdev->id) {
			tegra_i2s_dai[i].dev = &pdev->dev;
			tegra_i2s_dai[i].private_data = info;
			err = snd_soc_register_dai(&tegra_i2s_dai[i]);
			if (err)
				goto fail_clock;
		}
	}

	return 0;

fail_clock:
	am_device_deinit(&info->i2sdev_info);
	kfree(info);
	return err;
}


static int __devexit tegra_i2s_driver_remove(struct platform_device *pdev)
{
	struct tegra_i2s_info *info = tegra_i2s_dai[pdev->id].private_data;

	if (info)
		kfree(info);

	snd_soc_unregister_dai(&tegra_i2s_dai[pdev->id]);
	return 0;
}

static struct platform_driver tegra_i2s_driver = {
	.probe = tegra_i2s_driver_probe,
	.remove = __devexit_p(tegra_i2s_driver_remove),
	.driver = {
		.name = "i2s",
		.owner = THIS_MODULE,
	},
};

static int __init tegra_i2s_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&tegra_i2s_driver);
	return ret;
}
module_init(tegra_i2s_init);

static void __exit tegra_i2s_exit(void)
{
	platform_driver_unregister(&tegra_i2s_driver);
}
module_exit(tegra_i2s_exit);

/* Module information */
MODULE_DESCRIPTION("Tegra I2S SoC interface");
MODULE_LICENSE("GPL");
