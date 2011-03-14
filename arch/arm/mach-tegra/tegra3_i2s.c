/*
 * arch/arm/mach-tegra/tegra3_i2s.c
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
#include "clock.h"
#include <asm/io.h>
#include <mach/iomap.h>
#include <mach/tegra3_i2s.h>

#define check_i2s_ifc(n, ...) if ((n) > NR_I2S_IFC) {			\
	pr_err("%s: invalid audio interface %d\n", __func__, (n));	\
	return __VA_ARGS__;						\
}

#define ENABLE_I2S_DEBUG_PRINT	0
#if  ENABLE_I2S_DEBUG_PRINT
#define I2S_DEBUG_PRINT(fmt, arg...)  printk(fmt, ## arg)
#else
#define I2S_DEBUG_PRINT(fmt, arg...) do {} while (0)
#endif

static void *i2s_base[NR_I2S_IFC] = {
	IO_ADDRESS(TEGRA_I2S0_BASE),
	IO_ADDRESS(TEGRA_I2S1_BASE),
	IO_ADDRESS(TEGRA_I2S2_BASE),
	IO_ADDRESS(TEGRA_I2S3_BASE),
	IO_ADDRESS(TEGRA_I2S4_BASE),
};

static inline void i2s_writel(int ifc, u32 val, u32 reg)
{
	I2S_DEBUG_PRINT("i2s Write 0x%x : %08x\n",
		(unsigned int)i2s_base[ifc] + reg, val);
	__raw_writel(val, i2s_base[ifc] + reg);
}

static inline u32 i2s_readl(int ifc, u32 reg)
{
	u32 val = __raw_readl(i2s_base[ifc] + reg);
	I2S_DEBUG_PRINT("i2s Read 0x%x : %08x\n",
		(unsigned int) i2s_base[ifc] + reg, val);
	return val;
}

void i2s_dump_registers(int ifc)
{
	check_i2s_ifc(ifc);
	pr_info("%s: \n",__func__);
	i2s_readl(ifc, I2S_CTRL_0);
	i2s_readl(ifc, I2S_TIMING_0);
	i2s_readl(ifc, I2S_OFFSET_0);
	i2s_readl(ifc, I2S_CH_CTRL_0);
	i2s_readl(ifc, I2S_AUDIOCIF_I2STX_CTRL_0);
	i2s_readl(ifc, I2S_AUDIOCIF_I2SRX_CTRL_0);
	i2s_readl(ifc, I2S_FLOWCTL_0);
	i2s_readl(ifc, I2S_TX_STEP_0);
	i2s_readl(ifc, I2S_FLOW_STATUS_0);
	i2s_readl(ifc, I2S_FLOW_TOTAL_0);
	i2s_readl(ifc, I2S_FLOW_OVER_0);
}

void i2s_get_all_regs(int ifc, struct i2s_runtime_data* ird)
{}
void i2s_set_all_regs(int ifc, struct i2s_runtime_data* ird)
{}
/*
*   Set the fifo mode as Tx or Rx or both
*/
void i2s_fifo_enable(int ifc, int tx, int enable)
{
	u32 val;

	check_i2s_ifc(ifc);

	/* FIXME : this is to moved to a common
	 place in the audio_switch call
	*/
	apbif_channel_enable(ifc, tx, enable);

	val = i2s_readl(ifc, I2S_CTRL_0);
	if (tx != I2S_FIFO_TX) {   /* receive */
		set_reg_mode(val, I2S_CTRL_XFER_EN_RX, enable);
	}
	else { /* transmit */
		set_reg_mode(val, I2S_CTRL_XFER_EN_TX, enable);
	}
	i2s_writel(ifc, val, I2S_CTRL_0);
}

/*
*   Set the 2nd level clock gating
*/
void i2s_set_clock_gating(int ifc, int enable)
{
	u32 val;

	check_i2s_ifc(ifc);

	val = i2s_readl(ifc, I2S_CTRL_0);
	set_reg_mode(val, I2S_CTRL_CG_EN, enable);
	i2s_writel(ifc, val, I2S_CTRL_0);
}

/*
*   Set i2s soft reset
*/
void i2s_set_soft_reset(int ifc, int enable)
{
	u32 val;

	check_i2s_ifc(ifc);

	val = i2s_readl(ifc, I2S_CTRL_0);
	set_reg_mode(val, I2S_CTRL_SOFT_RESET, enable);
	i2s_writel(ifc, val, I2S_CTRL_0);
}

/*
*  I2s loopback mode
*/
void i2s_set_loopback(int ifc, int on)
{
	u32 val;

	check_i2s_ifc(ifc);

	val = i2s_readl(ifc, I2S_CTRL_0);
	set_reg_mode(val, I2S_CTRL_LPBK_ENABLE, on);
	i2s_writel(ifc, val, I2S_CTRL_0);
}

/*
*  I2s master/Slave mode
*/
void i2s_set_master(int ifc, int master)
{
	u32 val;
	check_i2s_ifc(ifc);
	val = i2s_readl(ifc, I2S_CTRL_0);
	set_reg_mode(val, I2S_CTRL_MASTER_ENABLE, master);
	i2s_writel(ifc, val, I2S_CTRL_0);
}

/*
*  I2s lrck polarity
*/
void i2s_set_left_right_control_polarity(int ifc, int left_low)
{
	u32 val;

	check_i2s_ifc(ifc);

	val = i2s_readl(ifc, I2S_CTRL_0);
	set_reg_mode(val, I2S_CTRL_LRCK_L_LOW,left_low);
	i2s_writel(ifc, val, I2S_CTRL_0);
}

/*
*   Set i2s bit code
*/
int i2s_set_bit_code(int ifc, unsigned int bitcode)
{
	u32 val;

	check_i2s_ifc(ifc, -EINVAL);

	if (bitcode >= AUDIO_BIT_CODE_RSVD) {
		pr_err("%s: invalid bit-code selector %d\n", __func__, bitcode);
		return -EINVAL;
	}

	val = i2s_readl(ifc, I2S_CTRL_0);
	val &= ~I2S_CTRL_BIT_CODE_MASK;
	val |= (bitcode << I2S_CTRL_BIT_CODE_SHIFT);

	i2s_writel(ifc, val, I2S_CTRL_0);
	return 0;
}

/*
*   Set i2s frame format
*/
int i2s_set_bit_format(int ifc, unsigned fmt)
{
	u32 val;

	check_i2s_ifc(ifc, -EINVAL);

	if (fmt >= AUDIO_FRAME_FORMAT_UNKNOWN) {
		pr_err("%s: invalid bit-format selector %d\n", __func__, fmt);
		return -EINVAL;
	}

	val = i2s_readl(ifc, I2S_CTRL_0);
	val &= ~I2S_CTRL_FRAME_FORMAT_MASK;

	if ((fmt == AUDIO_FRAME_FORMAT_I2S) ||
		(fmt == AUDIO_FRAME_FORMAT_RJM) ||
		(fmt == AUDIO_FRAME_FORMAT_LJM))
	{
		val |= I2S_CTRL_FRAME_FORMAT_LRCK;
	}
	else /*Dsp,Pcm,Tdm,Nw*/
	{
		val |= I2S_CTRL_FRAME_FORMAT_FSYNC;
	}

	i2s_writel(ifc, val, I2S_CTRL_0);
	return 0;
}

/*
* Set i2s bit size
*/
int i2s_set_bit_size(int ifc, unsigned bit_size)
{
	u32 val;

	check_i2s_ifc(ifc, -EINVAL);

	val = i2s_readl(ifc, I2S_CTRL_0);
	val &= ~I2S_CTRL_BIT_SIZE_MASK;

	if (bit_size > AUDIO_BIT_SIZE_32) {
		pr_err("%s: invalid bit_size selector %d\n", __func__,
			bit_size);
		return -EINVAL;
	}

	val |= bit_size << I2S_CTRL_BIT_SIZE_SHIFT;

	i2s_writel(ifc, val, I2S_CTRL_0);
	return 0;
}
/*
*  Channel bit count - used to set the timing register
*/
int i2s_set_channel_bit_count(int ifc, int sampling, int bitclk)
{
	u32 val;
	int bitcnt;

	check_i2s_ifc(ifc, -EINVAL);

	bitcnt = bitclk / (2 * sampling) - 1;

	if (bitcnt < 0 || bitcnt >= 1<<11) {
		pr_err("%s: bit count %d is out of bounds\n", __func__,
			bitcnt);
		return -EINVAL;
	}

	val = bitcnt;
	if (bitclk % (2 * sampling)) {
		pr_info("%s: enabling non-symmetric mode\n", __func__);
		val |= I2S_TIMING_NON_SYM_ENABLE;
	}

	/* FIXME: I2S/LJM/RJM - No of bit clocks in left or right channel
			DSP - No of bit clocks in left+right channel
			TDM/NW/PCM - No of bit clocks in the frame
	*/

	i2s_writel(ifc, val, I2S_TIMING_0);
	return 0;
}

/*
* I2s data offset
*/
int i2s_set_data_offset(int ifc, int tx, int dataoffset)
{
	u32 val;

	check_i2s_ifc(ifc, -EINVAL);

	val = i2s_readl(ifc, I2S_OFFSET_0);

	if (tx != AUDIO_TX_MODE)
	{
		val &= ~I2S_OFFSET_RX_DATA_OFFSET_MASK;
		val |= (dataoffset << I2S_OFFSET_RX_DATA_OFFSET_SHIFT);
	}
	else
	{
		val &= ~I2S_OFFSET_TX_DATA_OFFSET_MASK;
		val |= (dataoffset << I2S_OFFSET_TX_DATA_OFFSET_SHIFT);
	}

	i2s_writel(ifc, val, I2S_OFFSET_0);
	return 0;
}

/*
*	I2s edge control
*/
int i2s_set_edge_control(int ifc, int edgectrl)
{
	u32 val;

	check_i2s_ifc(ifc, -EINVAL);

	val = i2s_readl(ifc, I2S_CH_CTRL_0);

	set_reg_mode(val, I2S_CH_CTRL_EGDE_CTRL_NEG_EDGE, edgectrl);

	i2s_writel(ifc, val, I2S_CH_CTRL_0);
	return 0;

}

/*
*   I2s highz control
*/
int i2s_set_highz_control(int ifc, int highzvalue)
{
	u32 val;

	check_i2s_ifc(ifc, -EINVAL);

	val = i2s_readl(ifc, I2S_CH_CTRL_0);

	val &= ~I2S_CH_CTRL_HIGHZ_CTRL_MASK;
	val |= (highzvalue << I2S_CH_CTRL_HIGHZ_CTRL_SHIFT);

	i2s_writel(ifc, val, I2S_CH_CTRL_0);
	return 0;
}

/*
*   I2s Fsync width
*/
int i2s_set_fsync_width(int ifc, int fifo, int fsyncwidth)
{
	u32 val;

	check_i2s_ifc(ifc, -EINVAL);
	val = i2s_readl(ifc, I2S_CH_CTRL_0);

	val &= ~I2S_CH_CTRL_FSYNC_WIDTH_MASK;
	val |= (fsyncwidth << I2S_CH_CTRL_FSYNC_WIDTH_SHIFT);

	i2s_writel(ifc, val, I2S_CH_CTRL_0);
	return 0;
}

/*
*   I2s slot control
*/
int i2s_set_slot_control(int ifc, int tx, int totalslot, int numslots)
{
	u32 val;

	check_i2s_ifc(ifc, -EINVAL);
	val = i2s_readl(ifc, I2S_CH_CTRL_0);

	val &= ~I2S_SLOT_CTRL_TOTAL_SLOT_MASK;
	val |= (totalslot << I2S_CH_CTRL_FSYNC_WIDTH_SHIFT);

	if (tx != AUDIO_TX_MODE)
	{
		val &= ~I2S_SLOT_CTRL_RX_SLOT_MASK;
		val |= (numslots << I2S_SLOT_CTRL_RX_SLOT_SHIFT);
	}
	else
	{
		val &= ~I2S_SLOT_CTRL_TX_SLOT_MASK;
		val |= (numslots << I2S_SLOT_CTRL_TX_SLOT_SHIFT);
	}

	i2s_writel(ifc, val, I2S_CH_CTRL_0);
	return 0;
}

/*
*   I2s bit order
*/
int i2s_set_bit_order(int ifc, int tx, int bitorder)
{
	u32 val;

	check_i2s_ifc(ifc, -EINVAL);

	val = i2s_readl(ifc, I2S_CH_CTRL_0);

	if (tx != AUDIO_TX_MODE)
	{
		set_reg_mode(val,I2S_CH_CTRL_RX_BIT_LSB_FIRST, bitorder);
	}
	else
	{
		set_reg_mode(val, I2S_CH_CTRL_TX_BIT_LSB_FIRST, bitorder);
	}

	i2s_writel(ifc, val, I2S_CH_CTRL_0);
	return 0;
}


/*
*   I2s mask bit
*   Used with Pcm Mode to get exact bit size
*/
int i2s_set_bit_mask(int ifc, int tx, int maskbit)
{
	u32 val;

	check_i2s_ifc(ifc, -EINVAL);

	val = i2s_readl(ifc, I2S_CH_CTRL_0);

	if (tx != AUDIO_TX_MODE)
	{
		val &= ~I2S_CH_CTRL_RX_MASK_BITS_MASK;
		val |= (maskbit << I2S_CH_CTRL_RX_MASK_BITS_SHIFT);
	}
	else
	{
		val &= ~I2S_CH_CTRL_TX_MASK_BITS_MASK;
		val |= (maskbit << I2S_CH_CTRL_TX_MASK_BITS_SHIFT);
	}

	i2s_writel(ifc, val, I2S_CH_CTRL_0);
	return 0;
}

/*
*   I2s flow control
*/
int i2s_set_flow_control(int ifc, int enable, int filtertype, int stepsize)
{
	u32 val;

	check_i2s_ifc(ifc, -EINVAL);

	val = i2s_readl(ifc, I2S_FLOWCTL_0);

	set_reg_mode(val, I2S_FLOWCTL_FILTER_QUAD, filtertype);

	i2s_writel(ifc, val, I2S_FLOWCTL_0);

	val = i2s_readl(ifc, I2S_TX_STEP_0);

	val &= ~I2S_TX_STEP_MASK;
	val |= (stepsize << I2S_TX_STEP_SHIFT);

	i2s_writel(ifc, val, I2S_TX_STEP_0);

	val = i2s_readl(ifc, I2S_CTRL_0);
	set_reg_mode(val, I2S_CTRL_TX_FLOWCTL_EN, enable);
	i2s_writel(ifc, val, I2S_CTRL_0);
	return 0;
}

int i2s_fifo_set_attention_level(int ifc, int fifo, unsigned level)
{
	return apbif_fifo_set_attention_level(ifc, fifo, (level - 1));
}

void i2s_fifo_clear(int ifc, int fifo)
{
/* FIXME: fifo are part of apbif channel, so pass call to apbif
*  or provide generic call to apbif to handle this
*/
}


void i2s_set_fifo_irq_on_err(int ifc, int fifo, int on)
{
/* FIXME: fifo are part of apbif channel, so pass call to apbif
*  or provide generic call to apbif to handle this
*/
}


void i2s_set_fifo_irq_on_qe(int ifc, int fifo, int on)
{
/* FIXME: fifo are part of apbif channel, so pass call to apbif
*  or provide generic call to apbif to handle this
*/
}

void i2s_enable_fifos(int ifc, int on)
{
/* no support needed */
}

void i2s_fifo_write(int ifc, int fifo, u32 data)
{
/* FIXME: fifo are part of apbif channel, so pass call to apbif
*  or provide generic call to apbif to handle this
*/
}

u32 i2s_fifo_read(int ifc, int fifo)
{
/* FIXME: fifo are part of apbif channel, so pass call to apbif
*  or provide generic call to apbif to handle this
*/
	return 0;
}

u32 i2s_get_status(int ifc)
{
/* FIXME: fifo are part of apbif channel, so pass call to apbif
*  or provide generic call to apbif to handle this
*/
	return apbif_get_fifo_mode(ifc, AUDIO_TX_MODE);
}

u32 i2s_get_control(int ifc)
{
	check_i2s_ifc(ifc, 0);
	return i2s_readl(ifc, I2S_CTRL_0);
}

void i2s_ack_status(int ifc)
{
/* FIXME: fifo are part of apbif channel, so pass call to apbif
*  or provide generic call to apbif to handle this
*/
}

u32 i2s_get_fifo_scr(int ifc)
{
/* FIXME: fifo are part of apbif channel, so pass call to apbif
*  or provide generic call to apbif to handle this
*/
    return 0;
}

phys_addr_t i2s_get_fifo_phy_base(int ifc, int fifo)
{
	return apbif_get_fifo_phy_base(ifc, fifo);
}

u32 i2s_get_fifo_full_empty_count(int ifc, int fifo)
{
/* FIXME: fifo are part of apbif channel, so pass call to apbif
*  or provide generic call to apbif to handle this
*/
	return 0;
}

int i2s_get_dma_requestor(int ifc)
{
	return apbif_get_channel(ifc);
}

struct clk *i2s_get_clock_by_name(const char *name)
{
	return tegra_get_clock_by_name(name);
}

static	struct audio_cif  audiocif;
int i2s_initialize(int ifc)
{
	int err = 0;
	struct audio_cif  *tx_audio_cif = &audiocif;

	/* open audio_switch first */
	err = audio_switch_open();
	if (err)
		return err;

	i2s_enable_fifos(ifc, 0);
	i2s_set_left_right_control_polarity(ifc, AUDIO_LRCK_LEFT_LOW); /* low */
	i2s_set_master(ifc, AUDIO_MASTER_MODE); /* set as master */
	i2s_set_bit_format(ifc, AUDIO_FRAME_FORMAT_I2S);
	i2s_set_bit_size(ifc, AUDIO_BIT_SIZE_16);
	i2s_set_bit_code(ifc, AUDIO_BIT_CODE_LINEAR);
	i2s_set_data_offset(ifc, AUDIO_TX_MODE, 1);
	i2s_set_data_offset(ifc, AUDIO_RX_MODE, 1);

	/* FIXME: move all the apbif call to a generic function
	    inside audio_switch code - temporarily added here to
	    get minimum audio function working
	*/

	/* set i2s audiocif */
	/* setting base value for acif */
	memset(tx_audio_cif, 0 , sizeof(struct audio_cif));
	tx_audio_cif->audio_channels  = AUDIO_CHANNEL_2;
	tx_audio_cif->client_channels = AUDIO_CHANNEL_2;
	tx_audio_cif->audio_bits	 = AUDIO_BIT_SIZE_16;
	tx_audio_cif->client_bits	 = AUDIO_BIT_SIZE_16;
	audio_switch_set_acif((unsigned int)i2s_base[ifc] +
		 I2S_AUDIOCIF_I2STX_CTRL_0,	tx_audio_cif);
	audio_switch_set_acif((unsigned int)i2s_base[ifc] +
		 I2S_AUDIOCIF_I2SRX_CTRL_0,	tx_audio_cif);

	apbif_initialize(ifc, tx_audio_cif);
	return 0;
}

