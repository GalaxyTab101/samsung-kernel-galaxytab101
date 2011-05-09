/*
 * tegra_soc.h  --  SoC audio for tegra
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
 * */

#ifndef __TEGRA_AUDIO__
#define __TEGRA_AUDIO__

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/tegra_audio.h>
#include <linux/regulator/consumer.h>
#include <mach/iomap.h>
#include <mach/audio_manager.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/audio.h>
#include <mach/tegra_das.h>
#include <mach/dma.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc-dapm.h>
#include <sound/soc-dai.h>
#include <sound/tlv.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/hardware/scoop.h>

#define STATE_INIT	0
#define STATE_ABORT	1
#define STATE_EXIT	2
#define STATE_EXITED	3
#define STATE_INVALID	4

#define I2S1_CLK 		11289600
#define I2S2_CLK 		2000000
#define TEGRA_DEFAULT_SR	44100
#define TEGRA_INT_I2SLOOPBACK_ON	1
#define TEGRA_INT_I2SLOOPBACK_OFF	0

#define TEGRA_SAMPLE_RATES (SNDRV_PCM_RATE_8000_96000)
#define TEGRA_VOICE_SAMPLE_RATES SNDRV_PCM_RATE_8000

#define DMA_STEP_SIZE_MIN 8
#define DMA_REQ_QCOUNT 2

#define TEGRA_AUDIO_OFF		0x0
#define TEGRA_HEADPHONE		0x1
#define TEGRA_LINEOUT		0x2
#define TEGRA_SPK		0x4
#define TEGRA_EAR_SPK		0x8
#define TEGRA_INT_MIC		0x10
#define TEGRA_EXT_MIC		0x20
#define TEGRA_LINEIN		0x40
#define TEGRA_HEADSET		0x80

struct tegra_dma_channel;

struct tegra_runtime_data {
	struct snd_pcm_substream *substream;
	int size;
	int dma_pos;
	int dma_tail_idx;
	int dma_head_idx;
	int period_index;
	int dma_state;
	struct tegra_dma_req dma_req[DMA_REQ_QCOUNT];
	struct tegra_dma_channel *dma_chan;
};

struct tegra_audio_data {
	struct snd_soc_codec *codec;
	struct clk *dap_mclk;
	bool init_done;

	int play_device;
	int capture_device;
	bool is_call_mode;

	int codec_con;
};

void tegra_ext_control(struct snd_soc_codec *codec, int new_con);
int tegra_controls_init(struct snd_soc_codec *codec);

int tegra_jack_init(struct snd_soc_codec *codec);
void tegra_jack_exit(void);
void tegra_jack_resume(void);
void tegra_switch_set_state(int state);

void setup_i2s_dma_request(struct snd_pcm_substream *substream,
			struct tegra_dma_req *req,
			void (*dma_callback)(struct tegra_dma_req *req),
			void *dma_data);
void free_i2s_dma_request(struct snd_pcm_substream *substream);

void setup_spdif_dma_request(struct snd_pcm_substream *substream,
			struct tegra_dma_req *req,
			void (*dma_callback)(struct tegra_dma_req *req),
			void *dma_data);
void free_spdif_dma_request(struct snd_pcm_substream *substream);
#endif
