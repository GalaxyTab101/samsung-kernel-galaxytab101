/*
 * arch/arm/mach-tegra/include/mach/audio.h
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Iliyan Malchev <malchev@google.com>
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ARCH_ARM_MACH_TEGRA_AUDIO_H
#define __ARCH_ARM_MACH_TEGRA_AUDIO_H

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>

#define AUDIO_TX_MODE		0
#define AUDIO_RX_MODE		1

#define TEGRA_AUDIO_ENABLE_TX	1
#define TEGRA_AUDIO_ENABLE_RX	2
#define AUDIO_FIFO_CNT		2

/*
* Audio format modes
*/
#define AUDIO_FRAME_FORMAT_I2S		0
#define AUDIO_FRAME_FORMAT_RJM		1
#define AUDIO_FRAME_FORMAT_LJM		2
#define AUDIO_FRAME_FORMAT_DSP		3
#define AUDIO_FRAME_FORMAT_PCM		4
#define AUDIO_FRAME_FORMAT_NW		5
#define AUDIO_FRAME_FORMAT_TDM		6
#define AUDIO_FRAME_FORMAT_UNKNOWN	7

/*
*
*/
#define AUDIO_LRCK_LEFT_LOW	0
#define AUDIO_LRCK_RIGHT_LOW	1

/*
* Audio Bit code - based on compression/decompression
*/
#define AUDIO_BIT_CODE_LINEAR	0
#define AUDIO_BIT_CODE_ULAW	1
#define AUDIO_BIT_CODE_ALAW	2
#define AUDIO_BIT_CODE_RSVD	3

/*
* Audio channels
*/
#define AUDIO_CHANNEL_1		0
#define AUDIO_CHANNEL_2		1
#define AUDIO_CHANNEL_3		2
#define AUDIO_CHANNEL_4		3
#define AUDIO_CHANNEL_5		4
#define AUDIO_CHANNEL_6		5
#define AUDIO_CHANNEL_7		6
#define AUDIO_CHANNEL_8		7

/*
* Audio bit size
*/
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)

#define AUDIO_BIT_SIZE_16	0
#define AUDIO_BIT_SIZE_20	1
#define AUDIO_BIT_SIZE_24	2
#define AUDIO_BIT_SIZE_32	3

#define AUDIO_FIFO_16_LSB 0
#define AUDIO_FIFO_20_LSB 1
#define AUDIO_FIFO_24_LSB 2
#define AUDIO_FIFO_32     3
#define AUDIO_FIFO_PACKED 7

#elif defined(CONFIG_ARCH_TEGRA_3x_SOC)

#define AUDIO_BIT_SIZE_4		0
#define AUDIO_BIT_SIZE_8		1
#define AUDIO_BIT_SIZE_12		2
#define AUDIO_BIT_SIZE_16		3
#define AUDIO_BIT_SIZE_20		4
#define AUDIO_BIT_SIZE_24		5
#define AUDIO_BIT_SIZE_28		6
#define AUDIO_BIT_SIZE_32		7

#define AUDIO_FIFO_NOP			0
#define AUDIO_FIFO_RSVD			1
#define AUDIO_FIFO_PACK_8_4		2
#define AUDIO_FIFO_PACK_16		3

#endif

/*
* Audio Bit Order
*/
#define AUDIO_BIT_ORDER_MSB_FIRST	0
#define AUDIO_BIT_ORDER_LSB_FIRST	1

/*
* Audio Mask Bits
*/
#define AUDIO_MASK_BITS_ZERO		0
#define AUDIO_MASK_BITS_ONE		1
#define AUDIO_MASK_BITS_TWO		2
#define AUDIO_MASK_BITS_THREE		3
#define AUDIO_MASK_BITS_FOUR		4
#define AUDIO_MASK_BITS_FIVE		5
#define AUDIO_MASK_BITS_SIX		6
#define AUDIO_MASK_BITS_SEVEN		7

/*
* Audio Highz control
*/
#define AUDIO_NO_HIGHZ			0
#define AUDIO_HIGHZ			1
#define AUDIO_HIGHZ_HALF_BIT_CLK	2

/*
* Audio SamplemEdge control
*/
#define AUDIO_SAMPLE_POS_EDGE		0
#define AUDIO_SAMPLE_NEG_EDGE		1

/*
* Audio Master/Slave mode
*/
#define AUDIO_SLAVE_MODE		0
#define AUDIO_MASTER_MODE		1

struct tegra_audio_platform_data {
	bool i2s_master;
	bool dsp_master;
	int i2s_master_clk; /* When I2S mode and master, the framesync rate. */
	int dsp_master_clk; /* When DSP mode and master, the framesync rate. */
	bool dma_on;
	unsigned long i2s_clk_rate;
	unsigned long spdif_clk_rate;
	const char *dap_clk;
	const char *audio_sync_clk;

	int mode; /* I2S, LJM, RJM, etc. */
	int fifo_fmt;
	int bit_size;
	int i2s_bus_width; /* 32-bit for 16-bit packed I2S */
	int dsp_bus_width; /* 16-bit for DSP data format */
	int mask; /* enable tx and rx? */
	bool stereo_capture; /* True if hardware supports stereo */
	void *driver_data;
};

struct tegra_wired_jack_conf {
	int hp_det_n;   /* headphone jack detection gpio pin */
	int en_mic_ext; /* external mic enable gpio pin */
	int en_mic_int; /* internal mic enable gpio pin */
	int cdc_irq;    /* mic jack detection pin(IRQ-mode or generic gpio) */
	int en_spkr;    /* gpio pin to drive amplifier */
	const char *spkr_amp_reg;    /* regulator name for speaker amp */
	struct regulator *amp_reg;   /* regulator for speaker amp */
};

int audio_wired_jack_init(void);

#endif /* __ARCH_ARM_MACH_TEGRA_AUDIO_H */
