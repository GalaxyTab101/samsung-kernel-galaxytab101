/*
 * arch/arm/mach-tegra/audio_switch.c
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
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

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include "clock.h"
#include <asm/io.h>
#include <mach/iomap.h>
#include <mach/audio.h>
#include <mach/audio_switch.h>

#define TEGRA_AUDIO_CLUSTER_OFFSET  0x0000
#define TEGRA_AUDIO_SWITCH_OFFSET   0x0200

#define NR_APBIF_CHANNELS 4

#define check_apbif_ifc(n, ...) if ((n) > NR_APBIF_CHANNELS) {	\
	pr_err("%s: invalid interface %d\n", __func__, (n));	\
	return __VA_ARGS__;						\
}

/*
* Audiocif Ctrl
*/
#define AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT	28
#define AUDIOCIF_CTRL_FIFO_THRESHOLD_MASK\
		(0xf << AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT)

#define AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT	24
#define AUDIOCIF_CTRL_AUDIO_CHANNELS_MASK\
		(7 << AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT)

#define AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT	16
#define AUDIOCIF_CTRL_CLIENT_CHANNELS_MASK\
		(7 << AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT)

#define AUDIOCIF_CTRL_AUDIO_BITS_SHIFT		12
#define AUDIOCIF_CTRL_AUDIO_BITS_MASK\
		(7 << AUDIOCIF_CTRL_AUDIO_BITS_SHIFT)

#define AUDIOCIF_CTRL_CLIENT_BITS_SHIFT		8
#define AUDIOCIF_CTRL_CLIENT_BITS_MASK\
		(7 << AUDIOCIF_CTRL_CLIENT_BITS_SHIFT)


#define AUDIOCIF_CTRL_EXPAND_SHIFT	6
#define AUDIOCIF_CTRL_EXPAND_MASK\
		(3 << AUDIOCIF_CTRL_EXPAND_SHIFT)


#define AUDIOCIF_CTRL_STEREO_CONV_SHIFT		4
#define AUDIOCIF_CTRL_STEREO_CONV_MASK\
		(3 << AUDIOCIF_CTRL_STEREO_CONV_SHIFT)


#define AUDIOCIF_CTRL_REPLICATE_SHIFT		3
#define AUDIOCIF_CTRL_REPLICATE_MASK\
		(1 << AUDIOCIF_CTRL_REPLICATE_SHIFT)

#define AUDIOCIF_CTRL_DIRECTION_SHIFT		2
#define AUDIOCIF_CTRL_DIRECTION_TXCIF\
		(0 << AUDIOCIF_CTRL_DIRECTION_SHIFT)
#define AUDIOCIF_CTRL_DIRECTION_RXCIF\
		(1 << AUDIOCIF_CTRL_DIRECTION_SHIFT)

#define AUDIOCIF_CTRL_TRUNCATE_SHIFT		1
#define AUDIOCIF_CTRL_TRUNCATE_MASK\
		(1 << AUDIOCIF_CTRL_TRUNCATE_SHIFT)

#define AUDIOCIF_CTRL_MONO_CONV_SHIFT		0
#define AUDIOCIF_CTRL_MONO_CONV_MASK\
		(1 << AUDIOCIF_CTRL_MONO_CONV_SHIFT)

/* Offsets from AudioSwitch base */
#define AUDIO_APBIF_RX_OFFSET	0x4

/*
 * APBIF REGSITER OFFSETS
 */
#define APBIF_CHANNEL0_CTRL_0			0x0
#define APBIF_CHANNEL0_CLEAR_0			0x4
#define APBIF_CHANNEL0_STATUS_0			0x08
#define APBIF_CHANNEL0_TXFIFO_0			0x0c
#define APBIF_CHANNEL0_RXFIFO_0			0x10
#define APBIF_AUDIOCIF_TX0_CTRL_0		0x14
#define APBIF_AUDIOCIF_RX0_CTRL_0		0x18
#define APBIF_CHANNEL1_CTRL_0			0x20
#define APBIF_CHANNEL1_CLEAR_0			0x24
#define APBIF_CHANNEL1_STATUS_0			0x28
#define APBIF_CHANNEL1_TXFIFO_0			0x2c
#define APBIF_CHANNEL1_RXFIFO_0			0x30
#define APBIF_AUDIOCIF_TX1_CTRL_0		0x34
#define APBIF_AUDIOCIF_RX1_CTRL_0		0x38
#define APBIF_CHANNEL2_CTRL_0			0x40
#define APBIF_CHANNEL2_CLEAR_0			0x44
#define APBIF_CHANNEL2_STATUS_0			0x48
#define APBIF_CHANNEL2_TXFIFO_0			0x4c
#define APBIF_CHANNEL2_RXFIFO_0			0x50
#define APBIF_AUDIOCIF_TX2_CTRL_0		0x54
#define APBIF_AUDIOCIF_RX2_CTRL_0		0x58
#define APBIF_CHANNEL3_CTRL_0			0x60
#define APBIF_CHANNEL3_CLEAR_0			0x64
#define APBIF_CHANNEL3_STATUS_0			0x68
#define APBIF_CHANNEL3_TXFIFO_0			0x6c
#define APBIF_CHANNEL3_RXFIFO_0			0x70
#define APBIF_AUDIOCIF_TX3_CTRL_0		0x74
#define APBIF_AUDIOCIF_RX3_CTRL_0		0x78
#define APBIF_CONFIG_LINK_CTRL_0		0x80
#define APBIF_MISC_CTRL_0			0x84
#define APBIF_APBDMA_LIVE_STATUS_0		0x88
#define APBIF_I2S_LIVE_STATUS_0			0x8c
#define APBIF_DAM0_LIVE_STATUS_0		0x90
#define APBIF_DAM1_LIVE_STATUS_0		0x98
#define APBIF_DAM2_LIVE_STATUS_0		0xa0
#define APBIF_SPDIF_LIVE_STATUS_0		0xa8
#define APBIF_I2S_INT_MASK_0			0xb0
#define APBIF_DAM_INT_MASK_0			0xb4
#define APBIF_SPDIF_INT_MASK_0			0xbc
#define APBIF_APBIF_INT_MASK_0			0xc0
#define APBIF_I2S_INT_STATUS_0			0xc8
#define APBIF_DAM_INT_STATUS_0			0xcc
#define APBIF_SPDIF_INT_STATUS_0		0xd4
#define APBIF_APBIF_INT_STATUS_0		0xd8
#define APBIF_I2S_INT_SOURCE_0			0xe0
#define APBIF_DAM_INT_SOURCE_0			0xe4
#define APBIF_SPDIF_INT_SOURCE_0		0xec
#define APBIF_APBIF_INT_SOURCE_0		0xf0
#define APBIF_I2S_INT_SET_0			0xf8
#define APBIF_DAM_INT_SET_0			0xfc
#define APBIF_SPDIF_INT_SET_0			0x100
#define APBIF_APBIF_INT_SET_0			0x104

/*
* APBIF Channel Control
* Generic for all 4 apbif channels
*/

#define APBIF_CH_CTRL_TX_ENABLE		(1<<31)
#define APBIF_CH_CTRL_RX_ENABLE		(1<<30)
#define APBIF_CH_CTRL_LOOPBACK		(1<<29)

#define APBIF_CH_CTRL_THRESHOLD_LIMIT		0xff
#define APBIF_CH_CTRL_TX_THRESHOLD_SHIFT	16
#define APBIF_CH_CTRL_TX_THRESHOLD_MASK\
	(APBIF_CH_CTRL_THRESHOLD_LIMIT<<APBIF_CH_CTRL_TX_THRESHOLD_SHIFT)

#define APBIF_CH_CTRL_RX_THRESHOLD_SHIFT	8
#define APBIF_CH_CTRL_RX_THRESHOLD_MASK\
	(APBIF_CH_CTRL_THRESHOLD_LIMIT<<APBIF_CH_CTRL_RX_THRESHOLD_SHIFT)

#define APBIF_CH_CTRL_TX_PACK_EN		(1<<6)
#define APBIF_CH_CTRL_TX_PACK_SHIFT		4
#define APBIF_CH_CTRL_TX_PACK_MASK\
		(0x3<<APBIF_CH_CTRL_TX_PACK_SHIFT)


#define APBIF_CH_CTRL_RX_PACK_EN		(1<<2)
#define APBIF_CH_CTRL_RX_PACK_SHIFT		0
#define APBIF_CH_CTRL_RX_PACK_MASK\
		(0x3<<APBIF_CH_CTRL_RX_PACK_SHIFT)

/*
* APBIF Channel clear register
*/
#define APBIF_CH_CLEAR_TX_SOFT_RESET_EN		(1<<31)
#define APBIF_CH_CLEAR_RX_SOFT_RESET_EN		(1<<30)

/*
* APBIF Channel Status
*/
#define APBIF_CH_STATUS_TX_FREE_COUNT_SHIFT		24
#define APBIF_CH_STATUS_TX_FREE_COUNT_MASK\
		(0xff<<APBIF_CH_STATUS_TX_FREE_COUNT_SHIFT)
#define APBIF_CH_STATUS_RX_FREE_COUNT_SHIFT		16
#define APBIF_CH_STATUS_RX_FREE_COUNT_MASK\
		(0x00ff<<APBIF_CH_STATUS_RX_FREE_COUNT_SHIFT)

#define APBIF_CH_STATUS_TX_TRIG		(1<<1)
#define APBIF_CH_STATUS_RX_TRIG		(1<<0)

#define APBIF_CH_STATUS_FIFO_SHIFT	0
#define APBIF_CH_STATUS_FIFO_MASK\
		(0xffff<<APBIF_CH_STATUS_FIFO_SHIFT)

#define  ENABLE_AHUB_DEBUG_PRINT	0

#if ENABLE_AHUB_DEBUG_PRINT
#define AHUB_DEBUG_PRINT(fmt, arg...) printk(fmt, ## arg)
#else
#define AHUB_DEBUG_PRINT(fmt, arg...) do {} while (0)
#endif


/*
*  Internal functions
*/

struct apbif_channel_info {
	void __iomem	*virt_base;
	phys_addr_t	phy_base;
	int		dma_index;
	bool		fifo_inuse[AUDIO_FIFO_CNT];
	int		fifo_req[AUDIO_FIFO_CNT];
	int		fifo_refcnt[AUDIO_FIFO_CNT];
};

/* audio switch controller */
struct tegra_audiocont_info {
	struct clk *apbif_clk;
	struct clk *audiohub_clk;
	int  refcnt;
};

static struct tegra_audiocont_info *acinfo = NULL;
static int enable_audioswitch = 0;

static struct apbif_channel_info apbif_channels[NR_APBIF_CHANNELS];

static void *ahub_reg_base[ahubrx_maxnum];

static inline void audio_switch_writel(u32 reg, u32 val)
{
	writel(val, reg);
	AHUB_DEBUG_PRINT("ahub write offset 0x%x: %08x\n", reg, val);
}

static inline u32 audio_switch_readl(u32 reg)
{
	u32 val = readl(reg);
	AHUB_DEBUG_PRINT("ahub read offset 0x%x: %08x\n", reg, val);
	return val;
}

void audio_switch_dump_registers(int ifc)
{
	int i = 0;
	check_apbif_ifc(ifc);
	pr_info("%s: \n",__func__);
	for (i = 0; i < ahubrx_maxnum; i++)
		audio_switch_readl((u32)ahub_reg_base[i]);
}

void audio_switch_set_rx_port(int rxport, int txport)
{
	/*Get audioswitch base address*/
	audio_switch_writel((u32)ahub_reg_base[rxport], (1 << txport));
}

int audio_switch_get_rx_port(int rxport)
{
	/*Get audioswitch base address*/
	return audio_switch_readl(rxport);
}

/* audiocif control */
void audio_switch_set_acif(int addr, struct audio_cif *cifInfo)
{
	u32 val;

	val = __raw_readl(addr);
	AHUB_DEBUG_PRINT("acif value read 0x%x: %08x\n", addr, val);
	/* set threshold */
	val &= ~AUDIOCIF_CTRL_FIFO_THRESHOLD_MASK;
	val |= (cifInfo->threshold << AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT);
	/* set audio channels */
	val &= ~AUDIOCIF_CTRL_AUDIO_CHANNELS_MASK;
	val |= (cifInfo->audio_channels << AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT);
	/* client channels */
	val &= ~AUDIOCIF_CTRL_CLIENT_CHANNELS_MASK;
	val |=
	(cifInfo->client_channels << AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT);
	/* audio bits */
	val &= ~AUDIOCIF_CTRL_AUDIO_BITS_MASK;
	val |= (cifInfo->audio_bits << AUDIOCIF_CTRL_AUDIO_BITS_SHIFT);
	/* channel bits */
	val &= ~AUDIOCIF_CTRL_CLIENT_BITS_MASK;
	val |= (cifInfo->client_bits << AUDIOCIF_CTRL_CLIENT_BITS_SHIFT);
	/* expand */
	val &= ~AUDIOCIF_CTRL_EXPAND_MASK;
	val |= (cifInfo->expand << AUDIOCIF_CTRL_EXPAND_SHIFT);
	/* stereo convert */
	val &= ~AUDIOCIF_CTRL_STEREO_CONV_MASK;
	val |= (cifInfo->stereo_conv << AUDIOCIF_CTRL_STEREO_CONV_SHIFT);
	/* replicate */
	val &= ~AUDIOCIF_CTRL_REPLICATE_MASK;
	val |= (cifInfo->replicate << AUDIOCIF_CTRL_REPLICATE_SHIFT);
	/* truncate */
	val &= ~AUDIOCIF_CTRL_TRUNCATE_MASK;
	val |= (cifInfo->truncate << AUDIOCIF_CTRL_TRUNCATE_SHIFT);
	/* mono convert */
	val &= ~AUDIOCIF_CTRL_MONO_CONV_MASK;
	val |= (cifInfo->mono_conv << AUDIOCIF_CTRL_MONO_CONV_SHIFT);

	__raw_writel(val, addr);

	AHUB_DEBUG_PRINT("acif value written 0x%x: %08x\n", addr, val);
}


static inline void apbif_writel(int ifc, u32 val, u32 reg)
{
	struct apbif_channel_info *ch = &apbif_channels[ifc];

	AHUB_DEBUG_PRINT("apbif Write 0x%x : %08x\n",
		(unsigned int)ch->virt_base + reg, val);

	__raw_writel(val, ch->virt_base + reg);
}

static inline u32 apbif_readl(int ifc, u32 reg)
{
	struct apbif_channel_info *ch = &apbif_channels[ifc];
	u32 val = __raw_readl(ch->virt_base + reg);
	AHUB_DEBUG_PRINT("apbif Read 0x%x : %08x\n",
		(unsigned int)ch->virt_base + reg, val);
	return val;
}

void apbif_dump_registers(int ifc)
{
	check_apbif_ifc(ifc);
	pr_info("%s: \n",__func__);
	apbif_readl(ifc, APBIF_CHANNEL0_CTRL_0);
	apbif_readl(ifc, APBIF_CHANNEL0_CLEAR_0);
	apbif_readl(ifc, APBIF_CHANNEL0_STATUS_0);
	apbif_readl(ifc, APBIF_CHANNEL0_TXFIFO_0);
	apbif_readl(ifc, APBIF_CHANNEL0_RXFIFO_0);
	apbif_readl(ifc, APBIF_AUDIOCIF_TX0_CTRL_0);
	apbif_readl(ifc, APBIF_AUDIOCIF_RX0_CTRL_0);
	apbif_readl(0, APBIF_CONFIG_LINK_CTRL_0);
	apbif_readl(0, APBIF_MISC_CTRL_0);
	apbif_readl(0, APBIF_APBDMA_LIVE_STATUS_0);
	apbif_readl(0, APBIF_I2S_LIVE_STATUS_0);
	apbif_readl(0, APBIF_DAM0_LIVE_STATUS_0);
	apbif_readl(0, APBIF_DAM1_LIVE_STATUS_0);
	apbif_readl(0, APBIF_DAM2_LIVE_STATUS_0);
	apbif_readl(0, APBIF_SPDIF_LIVE_STATUS_0);
	apbif_readl(0, APBIF_I2S_INT_MASK_0);
	apbif_readl(0, APBIF_DAM_INT_MASK_0);
	apbif_readl(0, APBIF_SPDIF_INT_MASK_0);
	apbif_readl(0, APBIF_APBIF_INT_MASK_0);
	apbif_readl(0, APBIF_I2S_INT_STATUS_0);
	apbif_readl(0, APBIF_DAM_INT_STATUS_0);
	apbif_readl(0, APBIF_SPDIF_INT_STATUS_0);
	apbif_readl(0, APBIF_APBIF_INT_STATUS_0);
	apbif_readl(0, APBIF_I2S_INT_SOURCE_0);
	apbif_readl(0, APBIF_DAM_INT_SOURCE_0);
	apbif_readl(0, APBIF_SPDIF_INT_SOURCE_0);
	apbif_readl(0, APBIF_APBIF_INT_SOURCE_0);
	apbif_readl(0, APBIF_I2S_INT_SET_0);
	apbif_readl(0, APBIF_DAM_INT_SET_0);
	apbif_readl(0, APBIF_SPDIF_INT_SET_0);
	apbif_readl(0, APBIF_APBIF_INT_SET_0);
}

int audio_apbif_free_channel(int ifc, int fifo_mode)
{
	struct apbif_channel_info *ch = 0;
	ch = &apbif_channels[ifc];

	if (ch->fifo_inuse[fifo_mode] == true) {
		ch->fifo_refcnt[fifo_mode] -= 1;
		if (ch->fifo_refcnt[fifo_mode] <= 0) {
			ch->fifo_inuse[fifo_mode] = false;
			ch->fifo_req[fifo_mode]	= 0;
			ch->fifo_refcnt[fifo_mode] = 0;
			AHUB_DEBUG_PRINT("freed channel %d mode %d\n",
						ifc, fifo_mode);
		}
	}

	return 0;
}

/* FIXME : Stream with same index has to be taken care of
   need to figure out the stream->number is differential
   variable to make use of.
*/
static int get_apbif_channel(int fifo_req, int fifo_mode)
{
	struct apbif_channel_info *ch = 0;
	int i = 0;

	AHUB_DEBUG_PRINT(" %s ++ fifo_req 0x%x mode %d \n",
			__func__, (unsigned int)fifo_req, fifo_mode);

	for (i = 0; i < NR_APBIF_CHANNELS; i++) {
		ch = &apbif_channels[i];
		if (ch->fifo_inuse[fifo_mode] == true) {
			if (ch->fifo_req[fifo_mode] != fifo_req)
				continue;
			else
				goto found;
		}
	}

	for (i = 0; i < NR_APBIF_CHANNELS; i++) {
		ch = &apbif_channels[i];

		if (ch->fifo_inuse[fifo_mode] == true)
			continue;

		goto found;
	}

	return -ENOENT;

found:
	ch->fifo_inuse[fifo_mode] = true;
	ch->fifo_req[fifo_mode] = fifo_req;
	ch->fifo_refcnt[fifo_mode] += 1;

	AHUB_DEBUG_PRINT(" %s -- dmaindex 0x%x ch 0x%x refcnt 0x%x\n",
			__func__, ch->dma_index, i, ch->fifo_refcnt[fifo_mode]);

	/*FIXME: move the connection based on request for apbif if needed */
	if (fifo_mode == AUDIO_TX_MODE)
		audio_switch_set_rx_port(fifo_req, i);
	else
		audio_switch_set_rx_port(i, fifo_req);

	return ch->dma_index;
}

/*
*   Set the fifo mode as Tx or Rx or both
*/
void apbif_channel_enable(int ifc, int tx, int enable)
{
	u32 val;

	check_apbif_ifc(ifc);

	val = apbif_readl(ifc, APBIF_CHANNEL0_CTRL_0);
	if (tx != AUDIO_TX_MODE) {
		set_reg_mode(val, APBIF_CH_CTRL_RX_ENABLE, enable);
	} else {
		set_reg_mode(val, APBIF_CH_CTRL_TX_ENABLE, enable);
	}
	apbif_writel(ifc, val, APBIF_CHANNEL0_CTRL_0);
}

/*
*  apbif loopback mode
*/
void apbif_channel_set_loopback(int ifc, int on)
{
	u32 val;

	check_apbif_ifc(ifc);

	val = apbif_readl(ifc, APBIF_CHANNEL0_CTRL_0);
	set_reg_mode(val, APBIF_CH_CTRL_LOOPBACK, on);
	apbif_writel(ifc, val, APBIF_CHANNEL0_CTRL_0);
}

/*
*  Apbif set pack mode
*/
void apbif_set_pack_mode(int ifc, int tx, int pack_mode)
{
	u32 val;

	check_apbif_ifc(ifc);

	val = apbif_readl(ifc, APBIF_CHANNEL0_CTRL_0);
	if (tx == AUDIO_TX_MODE) {
		val &= ~APBIF_CH_CTRL_TX_PACK_MASK;
		val |= pack_mode << APBIF_CH_CTRL_TX_PACK_SHIFT;
		val &= ~APBIF_CH_CTRL_TX_PACK_EN;
		if ((pack_mode != AUDIO_PACK_NOP) &&
			(pack_mode != AUDIO_PACK_RSVD)) {
			val |= APBIF_CH_CTRL_TX_PACK_EN;
		}
	} else {
		val &= ~APBIF_CH_CTRL_RX_PACK_MASK;
		val |= pack_mode << APBIF_CH_CTRL_RX_PACK_SHIFT;
		val &= ~APBIF_CH_CTRL_RX_PACK_EN;
		if ((pack_mode != AUDIO_PACK_NOP) &&
			(pack_mode != AUDIO_PACK_RSVD))	{
			val |= APBIF_CH_CTRL_RX_PACK_EN;
		}
	}

	apbif_writel(ifc, val, APBIF_CHANNEL0_CTRL_0);
}

/*
*  Apbif set threshold
*/
int apbif_fifo_set_attention_level(int ifc, int tx, unsigned level)
{
	u32 val;

	check_apbif_ifc(ifc, -EINVAL);

	if (level > APBIF_CH_CTRL_THRESHOLD_LIMIT) {
		pr_err("%s: invalid fifo level selector %d\n", __func__,
			level);
		return -EINVAL;
	}

	val = apbif_readl(ifc, APBIF_CHANNEL0_CTRL_0);

	if (tx != AUDIO_TX_MODE) {
		val &= ~APBIF_CH_CTRL_RX_THRESHOLD_MASK;
		val |= level << APBIF_CH_CTRL_RX_THRESHOLD_SHIFT;
	} else {
		val &= ~APBIF_CH_CTRL_TX_THRESHOLD_MASK;
		val |= level << APBIF_CH_CTRL_TX_THRESHOLD_SHIFT;
	}

	apbif_writel(ifc, val, APBIF_CHANNEL0_CTRL_0);
	return 0;
}

void apbif_fifo_write(int ifc, int fifo_mode, u32 data)
{
	check_apbif_ifc(ifc);

	if (fifo_mode == AUDIO_TX_MODE) {
		apbif_writel(ifc, data, APBIF_CHANNEL0_TXFIFO_0);
	} else {
		apbif_writel(ifc, data, APBIF_CHANNEL0_RXFIFO_0);
	}
}

u32 apbif_fifo_read(int ifc, int fifo_mode)
{
	u32 val;

	check_apbif_ifc(ifc, 0);

	if (fifo_mode == AUDIO_TX_MODE) {
		val = apbif_readl(ifc, APBIF_CHANNEL0_TXFIFO_0);
	} else {
		val = apbif_readl(ifc, APBIF_CHANNEL0_RXFIFO_0);
	}

	return val;
}

/*
* Apbif soft channel reset
*/
void apbif_soft_reset(int ifc, int fifo_mode, int enable)
{
	u32 val;

	check_apbif_ifc(ifc);

	val = apbif_readl(ifc, APBIF_CHANNEL0_CTRL_0);

	if (fifo_mode == AUDIO_TX_MODE) {
		set_reg_mode(val, APBIF_CH_CLEAR_TX_SOFT_RESET_EN, enable);
	} else {
		set_reg_mode(val, APBIF_CH_CLEAR_RX_SOFT_RESET_EN, enable);
	}

	apbif_writel(ifc, val, APBIF_CHANNEL0_CLEAR_0);
}

/*
* Apbif channel get fifo free count
*/
int apbif_get_fifo_freecount(int ifc, int tx)
{
	u32 val;

	check_apbif_ifc(ifc, 0);

	val = apbif_readl(ifc, APBIF_CHANNEL0_STATUS_0);

	if (tx != AUDIO_TX_MODE) {
		val = val >> APBIF_CH_STATUS_RX_FREE_COUNT_SHIFT;
		return val & APBIF_CH_STATUS_RX_FREE_COUNT_MASK;
	} else {
		val = val >> APBIF_CH_STATUS_TX_FREE_COUNT_SHIFT;
		return val & APBIF_CH_STATUS_TX_FREE_COUNT_MASK;
	}
}

/*
* Apbif channel get fifo mode
*/
int apbif_get_fifo_mode(int ifc, int tx)
{
	u32 val;

	check_apbif_ifc(ifc, 0);

	val = apbif_readl(ifc, APBIF_CHANNEL0_STATUS_0);

	if (tx != AUDIO_TX_MODE) {
		val = val & APBIF_CH_STATUS_RX_TRIG;
		return val >> APBIF_CH_STATUS_RX_TRIG;
	} else {
		val = val & APBIF_CH_STATUS_TX_TRIG;
		return val >> APBIF_CH_STATUS_TX_TRIG;
	}
}

/*
* Apbif get fifo physical address
*/
phys_addr_t apbif_get_fifo_phy_base(int ifc, int tx)
{
	struct apbif_channel_info *ch = &apbif_channels[ifc];

	check_apbif_ifc(ifc, 0);
	return (ch->phy_base + ((tx == AUDIO_TX_MODE)?
		APBIF_CHANNEL0_TXFIFO_0:APBIF_CHANNEL0_RXFIFO_0));
}

int apbif_get_channel(int regindex, int fifo_mode)
{
	return get_apbif_channel(regindex, fifo_mode);
}

static void apbif_disable_clock(void)
{
	if (!acinfo) return;

	if (acinfo->audiohub_clk)
		clk_disable(acinfo->audiohub_clk);

	if (acinfo->apbif_clk)
		clk_disable(acinfo->apbif_clk);

}

static int apbif_enable_clock(void)
{

	int err = 0;

	if (!acinfo)
		return -EIO;

	/*apbif clocks */
	if (clk_enable(acinfo->apbif_clk)) {
		err = PTR_ERR(acinfo->apbif_clk);
		goto fail_audio_clock;
	}

	/* audio hub */
	if (clk_enable(acinfo->audiohub_clk)) {
		err = PTR_ERR(acinfo->audiohub_clk);
		goto fail_audio_clock;
	}

	return err;

fail_audio_clock:

	apbif_disable_clock();
	return err;
}

int audio_apbif_set_acif(int ifc, int fifo_mode, struct audio_cif *cifInfo)
{
	struct apbif_channel_info *ch;
	ch =  &apbif_channels[ifc];

	if (fifo_mode == AUDIO_TX_MODE) {
		audio_switch_set_acif((unsigned int)ch->virt_base +
		 APBIF_AUDIOCIF_TX0_CTRL_0, cifInfo);

		/* FIXME: packed mode as default */
		apbif_set_pack_mode(ifc, AUDIO_TX_MODE, AUDIO_PACK_16);

	} else {
		audio_switch_set_acif((unsigned int)ch->virt_base +
		 APBIF_AUDIOCIF_RX0_CTRL_0, cifInfo);

		/* FIXME: packed mode as default */
		apbif_set_pack_mode(ifc, AUDIO_RX_MODE, AUDIO_PACK_16);
	}
	return 0;
}

int apbif_initialize(int ifc, struct audio_cif *cifInfo)
{
	return 0;
}

int audio_switch_open(void)
{
	int err = 0, i = 0;

	AHUB_DEBUG_PRINT(" audio_switch_open  acinfo 0x%x enable %d ++ \n",
			(unsigned int)acinfo, enable_audioswitch);

	if (!acinfo && !enable_audioswitch) {
		struct apbif_channel_info *ch;

		acinfo =
		kzalloc(sizeof(struct tegra_audiocont_info), GFP_KERNEL);

		if (!acinfo)
			return -ENOMEM;

		memset(apbif_channels, 0, sizeof(apbif_channels));

		for (i = 0; i < NR_APBIF_CHANNELS; i++) {
			ch = &apbif_channels[i];
			ch->phy_base  = TEGRA_APBIF0_BASE +
					(TEGRA_APBIF0_SIZE * i);
			ch->virt_base = IO_ADDRESS(TEGRA_APBIF0_BASE) +
					(TEGRA_APBIF0_SIZE * i);
			ch->dma_index = i + 1;
		}

		acinfo->apbif_clk = clk_get_sys("apbif", NULL);
		if (IS_ERR_OR_NULL(acinfo->apbif_clk)) {
			err = -ENOENT;
			acinfo->apbif_clk = 0;
			goto fail_audio_open;
		}

		acinfo->audiohub_clk = clk_get_sys("d_audio", NULL);
		if (IS_ERR_OR_NULL(acinfo->audiohub_clk)) {
			err = -ENOENT;
			acinfo->audiohub_clk = 0;
			goto fail_audio_open;
		}

		/*FIXME: add interface to set the audiohub rate and parent */
		for (i = 0; i < ahubrx_maxnum; i++) {
			ahub_reg_base[i] = IO_ADDRESS(TEGRA_AHUB_BASE) +
					(i * AUDIO_APBIF_RX_OFFSET);
		}

		err = apbif_enable_clock();

		if (err)
			goto fail_audio_open;

		enable_audioswitch = 1;
	}

	acinfo->refcnt += 1;

	AHUB_DEBUG_PRINT(" audio_switch_open -- acinfo 0x%x refcnt %d \n",
			(unsigned int)acinfo, acinfo->refcnt);

	return 0;

fail_audio_open:
	if (acinfo) {
		apbif_disable_clock();
		kfree(acinfo);
	}

	return err;
}

int audio_switch_close(void)
{
	if (acinfo && enable_audioswitch) {
		acinfo->refcnt -= 1;

		if (!acinfo->refcnt) {
			apbif_disable_clock();
			kfree(acinfo);
			enable_audioswitch = 0;
		}
	}
	return 0;
}
