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
#include <linux/clk.h>
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

struct i2s_controller_info {
	struct tegra_i2s_channel_property i2s_ch_prop[AUDIO_FIFO_CNT];
	struct tegra_i2s_property i2sprop;
	int	clk_refs;
	int	i2s_regcache[I2S_REG_MAXINDEX];
};

static struct i2s_controller_info i2s_cont_info[NR_I2S_IFC];

static void *i2s_base[NR_I2S_IFC] = {
	IO_ADDRESS(TEGRA_I2S0_BASE),
	IO_ADDRESS(TEGRA_I2S1_BASE),
	IO_ADDRESS(TEGRA_I2S2_BASE),
	IO_ADDRESS(TEGRA_I2S3_BASE),
	IO_ADDRESS(TEGRA_I2S4_BASE),
};

struct i2s_clk_info {
	char *clk_name;
	char *sync_clk_name;
	char *audio_clk_name;
	char *audio2x_clk_name;
};

static struct i2s_clk_info i2sclk_info[NR_I2S_IFC] = {
	{"i2s0", "i2s0_sync", "audio0", "audio0_2x"},
	{"i2s1", "i2s1_sync", "audio1", "audio1_2x"},
	{"i2s2", "i2s2_sync", "audio2", "audio2_2x"},
	{"i2s3", "i2s3_sync", "audio3", "audio3_2x"},
	{"i2s4", "i2s4_sync", "audio4", "audio4_2x"}
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

static int i2s_get_apbif_channel(int ifc, int fifo_mode)
{
	return i2s_cont_info[ifc].i2s_ch_prop[fifo_mode].dma_ch;
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

struct i2s_controller_info * i2s_get_cont_info(int ifc)
{
	return &i2s_cont_info[ifc];
}

struct tegra_i2s_channel_property* i2s_get_channel_prop(int ifc, int mode)
{
	return &i2s_cont_info[ifc].i2s_ch_prop[mode];
}

struct tegra_i2s_property* i2s_get_prop(int ifc)
{
	return &i2s_cont_info[ifc].i2sprop;
}

static void i2s_save_registers(int ifc)
{
	int i = 0;
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	for (i = 0; i <= I2S_REG_MAXINDEX; i++)
		info->i2s_regcache[i] = i2s_readl(ifc, (i << 2));

}

static void i2s_restore_registers(int ifc)
{
	int i = 0;
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	for (i = 0; i <= I2S_REG_MAXINDEX; i++)
		i2s_writel(ifc, info->i2s_regcache[i], (i << 2));

}

void i2s_suspend(int ifc)
{
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	if (info->clk_refs == 0)
		i2s_clock_enable(ifc);

	i2s_save_registers(ifc);
	audio_switch_suspend();

	i2s_clock_disable(ifc);
}

void i2s_resume(int ifc)
{
	i2s_clock_enable(ifc);
	audio_switch_resume();
	i2s_restore_registers(ifc);
}

/*
*   Set the fifo mode as Tx or Rx or both
*/
void i2s_fifo_enable(int ifc, int tx, int enable)
{
	u32 val;
	struct i2s_controller_info *info = &i2s_cont_info[ifc];
	int apbif_ifc = 0;

	check_i2s_ifc(ifc);

	apbif_ifc = i2s_get_apbif_channel(ifc, tx);

	apbif_channel_enable(apbif_ifc, tx, enable);

	val = i2s_readl(ifc, I2S_CTRL_0);

	if (tx != AUDIO_TX_MODE) {
		set_reg_mode(val, I2S_CTRL_XFER_EN_RX, enable);
	} else  {
		set_reg_mode(val, I2S_CTRL_XFER_EN_TX, enable);
	}

	if (enable)
		i2s_fifo_set_attention_level(ifc, tx,
				info->i2s_ch_prop[tx].fifo_attn);

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
	set_reg_mode(val, I2S_CTRL_LRCK_R_LOW, left_low);
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
		(fmt == AUDIO_FRAME_FORMAT_LJM)) {
		val |= I2S_CTRL_FRAME_FORMAT_LRCK;
	} else { /*Dsp,Pcm,Tdm,Nw*/
		val |= I2S_CTRL_FRAME_FORMAT_FSYNC;

		i2s_set_fsync_width(ifc, 0);
		i2s_set_edge_control(ifc, 1);
		i2s_set_slot_control(ifc, AUDIO_TX_MODE, 0, 0x1);
		i2s_set_slot_control(ifc, AUDIO_RX_MODE, 0, 0x1);
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

int i2s_set_samplerate(int ifc, int samplerate)
{
	int rate = 0;
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	if (info->i2sprop.master_mode && info->i2sprop.i2s_clk)
	{
		rate = clk_get_rate(info->i2sprop.i2s_clk);

		if (info->i2sprop.audio_mode == AUDIO_FRAME_FORMAT_DSP)
			rate *= 2;

		i2s_set_channel_bit_count(ifc, samplerate, rate);
	}

	return 0;
}

int i2s_set_channels(int ifc, int channels)
{
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	info->i2sprop.channels = channels;

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

	if (tx != AUDIO_TX_MODE) {
		val &= ~I2S_OFFSET_RX_DATA_OFFSET_MASK;
		val |= (dataoffset << I2S_OFFSET_RX_DATA_OFFSET_SHIFT);
	} else {
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
int i2s_set_fsync_width(int ifc, int fsyncwidth)
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
	val = i2s_readl(ifc, I2S_SLOT_CTRL_0);

	val &= ~I2S_SLOT_CTRL_TOTAL_SLOT_MASK;
	val |= (totalslot << I2S_SLOT_CTRL_TOTAL_SLOT_SHIFT);

	if (tx != AUDIO_TX_MODE) {
		val &= ~I2S_SLOT_CTRL_RX_SLOT_MASK;
		val |= (numslots << I2S_SLOT_CTRL_RX_SLOT_SHIFT);
	} else {
		val &= ~I2S_SLOT_CTRL_TX_SLOT_MASK;
		val |= (numslots << I2S_SLOT_CTRL_TX_SLOT_SHIFT);
	}

	i2s_writel(ifc, val, I2S_SLOT_CTRL_0);
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

	if (tx != AUDIO_TX_MODE) {
		set_reg_mode(val,I2S_CH_CTRL_RX_BIT_LSB_FIRST, bitorder);
	} else {
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

	if (tx != AUDIO_TX_MODE) {
		val &= ~I2S_CH_CTRL_RX_MASK_BITS_MASK;
		val |= (maskbit << I2S_CH_CTRL_RX_MASK_BITS_SHIFT);
	} else {
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
	int apbif_ifc = i2s_get_apbif_channel(ifc, fifo);

	if (apbif_ifc != -ENOENT)
		return apbif_fifo_set_attention_level(apbif_ifc,
							 fifo, (level - 1));
	return 0;
}

void i2s_fifo_clear(int ifc, int fifo)
{
	int apbif_ifc = i2s_get_apbif_channel(ifc, fifo);

	if (apbif_ifc != -ENOENT)
		apbif_soft_reset(apbif_ifc, fifo, 1);
}

void i2s_set_fifo_attention(int ifc, int buffersize, int fifo_mode)
{
	int fifoattn = I2S_FIFO_ATN_LVL_FOUR_SLOTS;
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	if (buffersize & 0xF)
		fifoattn = I2S_FIFO_ATN_LVL_ONE_SLOT;
	else if ((buffersize >> 4) & 0x1)
		fifoattn = I2S_FIFO_ATN_LVL_FOUR_SLOTS;
	else
		fifoattn = I2S_FIFO_ATN_LVL_EIGHT_SLOTS;

	info->i2s_ch_prop[fifo_mode].fifo_attn = fifoattn;
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

void i2s_fifo_write(int ifc, int fifo, u32 data)
{
	int apbif_ifc = i2s_get_apbif_channel(ifc, fifo);

	if (apbif_ifc != -ENOENT)
		apbif_fifo_write(apbif_ifc, fifo, data);
}

u32 i2s_fifo_read(int ifc, int fifo)
{
	int apbif_ifc = i2s_get_apbif_channel(ifc, fifo);

	if (apbif_ifc != -ENOENT)
		return apbif_fifo_read(apbif_ifc, fifo);
	return 0;
}

u32 i2s_get_status(int ifc, int fifo)
{
	int apbif_ifc = i2s_get_apbif_channel(ifc, fifo);
	int regval = 0;

	if (apbif_ifc != -ENOENT)
		regval = apbif_get_fifo_mode(apbif_ifc, fifo);

	if (fifo == AUDIO_TX_MODE)
		regval &= I2S_FIFO_TX_BUSY;
	else
		regval &= I2S_FIFO_RX_BUSY;

	return regval;
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

phys_addr_t i2s_get_fifo_phy_base(int ifc, int fifo_mode)
{
	int apbif_ifc = i2s_get_apbif_channel(ifc, fifo_mode);

	if (apbif_ifc == -ENOENT) {
		pr_err("%s: dma not assigned \n", __func__);
		return -EINVAL;
	}

	return apbif_get_fifo_phy_base(apbif_ifc, fifo_mode);
}

u32 i2s_get_fifo_full_empty_count(int ifc, int fifo)
{
	int apbif_ifc = i2s_get_apbif_channel(ifc, fifo);

	if (apbif_ifc != -ENOENT)
		return apbif_get_fifo_freecount(apbif_ifc, fifo);

	return 0;
}

int i2s_get_dma_requestor(int ifc, int fifo_mode)
{
	int dma_index = 0;
	int regIndex  = ifc + ahubrx_i2s0;

	if (fifo_mode == AUDIO_TX_MODE)
		regIndex = ifc + ahubtx_i2s0;

	dma_index = apbif_get_channel(regIndex, fifo_mode);

	if (dma_index != -ENOENT) {
		i2s_cont_info[ifc].i2s_ch_prop[fifo_mode].dma_ch = dma_index-1;
		/* FIXME : this need to be called on connection request	*/
		i2s_set_acif(ifc, fifo_mode, 0);
	}

	return dma_index;
}

int i2s_free_dma_requestor(int ifc, int fifo_mode)
{
	int apbif_ifc = i2s_get_apbif_channel(ifc, fifo_mode);

	if (apbif_ifc != -ENOENT)
		audio_apbif_free_channel(apbif_ifc, fifo_mode);

	return 0;
}

struct clk *i2s_get_clock_by_name(const char *name)
{
	return tegra_get_clock_by_name(name);
}

static	struct audio_cif  audiocif;

int i2s_set_acif(int ifc, int fifo_mode, struct audio_cif *cifInfo)
{
	struct audio_cif  *tx_audio_cif = &audiocif;
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	/* set i2s audiocif */
	/* setting base value for acif */
	memset(tx_audio_cif, 0 , sizeof(struct audio_cif));
	tx_audio_cif->audio_channels	= info->i2sprop.channels;
	tx_audio_cif->client_channels	= info->i2sprop.channels;
	tx_audio_cif->audio_bits	= info->i2sprop.bit_size;
	tx_audio_cif->client_bits	= info->i2sprop.bit_size;

	if (fifo_mode == AUDIO_TX_MODE)
		audio_switch_set_acif((unsigned int)i2s_base[ifc] +
			 I2S_AUDIOCIF_I2STX_CTRL_0, tx_audio_cif);
	else
		audio_switch_set_acif((unsigned int)i2s_base[ifc] +
			 I2S_AUDIOCIF_I2SRX_CTRL_0, tx_audio_cif);

	apbif_set_pack_mode(i2s_get_apbif_channel(ifc, fifo_mode),
		fifo_mode, info->i2sprop.fifo_fmt);

	audio_apbif_set_acif(i2s_get_apbif_channel(ifc, fifo_mode),
		fifo_mode, tx_audio_cif);

	return 0;
}

int i2s_close(int ifc)
{
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	if (info->i2sprop.i2s_clk)
		clk_put(info->i2sprop.i2s_clk);

	if (info->i2sprop.i2s_sync_clk)
		clk_put(info->i2sprop.i2s_sync_clk);

	if (info->i2sprop.audio_clk)
		clk_put(info->i2sprop.audio_clk);

	if (info->i2sprop.audio2x_clk)
		clk_put(info->i2sprop.audio2x_clk);

	return 0;
}

static int i2s_open(int ifc)
{
	int err = 0;
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	info->i2sprop.i2s_clk = tegra_get_clock_by_name(
					i2sclk_info[ifc].clk_name);
	if (!info->i2sprop.i2s_clk) {

		pr_err("can't get i2s clock\n");
		err = -ENODEV;
		goto clk_err;
	}

	info->i2sprop.i2s_sync_clk = tegra_get_clock_by_name(
				i2sclk_info[ifc].sync_clk_name);
	if (!info->i2sprop.i2s_sync_clk) {

		pr_err("can't get i2s sync clock\n");
		err = -ENODEV;
		goto clk_err;
	}

	info->i2sprop.audio_clk = tegra_get_clock_by_name(
				i2sclk_info[ifc].audio_clk_name);
	if (!info->i2sprop.audio_clk) {

		pr_err("can't get audio clock\n");
		err = -ENODEV;
		goto clk_err;
	}

	info->i2sprop.audio2x_clk = tegra_get_clock_by_name(
				i2sclk_info[ifc].audio2x_clk_name);
	if (!info->i2sprop.audio2x_clk) {

		pr_err("can't get audio2x clock\n");
		err = -ENODEV;
		goto clk_err;
	}

	return err;

clk_err:

	i2s_close(ifc);

	return err;
}

int i2s_init(int ifc,  struct tegra_i2s_property* pi2sprop)
{
	int err = 0;
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	memset(info, 0 , sizeof(struct i2s_controller_info));

	i2s_cont_info[ifc].i2s_ch_prop[0].dma_ch = -ENOENT;
	i2s_cont_info[ifc].i2s_ch_prop[1].dma_ch = -ENOENT;

	memcpy(&info->i2sprop,pi2sprop,sizeof(struct tegra_i2s_property));

	if (i2s_open(ifc))
		return err;

	/* open audio_switch first */
	err = audio_switch_open();
	if (err) {
		i2s_close(ifc);
		return err;
	}

	i2s_clock_rate(ifc, pi2sprop->clk_rate);
	err = i2s_clock_enable(ifc);

	if (err) {
		audio_switch_close();
		i2s_close(ifc);
		return err;
	}

	i2s_set_left_right_control_polarity(ifc, AUDIO_LRCK_LEFT_LOW);
	i2s_set_master(ifc, pi2sprop->master_mode);

	i2s_set_bit_format(ifc, pi2sprop->audio_mode);
	i2s_set_bit_size(ifc, pi2sprop->bit_size);
	i2s_set_bit_code(ifc, AUDIO_BIT_CODE_LINEAR);

	i2s_set_data_offset(ifc, AUDIO_TX_MODE, 1);
	i2s_set_data_offset(ifc, AUDIO_RX_MODE, 1);

	i2s_set_samplerate(ifc, pi2sprop->sample_rate);

	i2s_clock_disable(ifc);

	return 0;
}


int i2s_clock_enable(int ifc)
{
	int err = 0;
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	err = audio_switch_enable_clock();
	if (err)
		return err;

	if (info->i2sprop.i2s_clk && (info->i2sprop.master_mode == true)) {
		clk_set_rate(info->i2sprop.i2s_clk, info->i2sprop.clk_rate);

		if (!info->clk_refs) {
			if (clk_enable(info->i2sprop.i2s_clk))
			{
				err = PTR_ERR(info->i2sprop.i2s_clk);
				return err;
			}
		}
		info->clk_refs++;
	} else {

		info->clk_refs++;
	}

	I2S_DEBUG_PRINT(" i2s enable clock count 0x%x \n", info->clk_refs);
	return err;
}

int i2s_clock_disable(int ifc)
{

	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	if (info->i2sprop.i2s_clk &&
		(info->i2sprop.master_mode == true)) {

		if(info->clk_refs > 0) {
			info->clk_refs--;
			if (info->clk_refs == 0) {
				clk_disable(info->i2sprop.i2s_clk);
				info->clk_refs = 0;
			}
		}
	} else {

		if (info->clk_refs > 0) {
			info->clk_refs--;

			if (info->clk_refs == 0) {

				if (info->i2sprop.i2s_sync_clk) {
					clk_disable(info->i2sprop.i2s_sync_clk);
				}

				if (info->i2sprop.audio_clk) {
					clk_disable(info->i2sprop.audio_clk);
				}

				if (info->i2sprop.audio2x_clk) {
					clk_disable(info->i2sprop.audio2x_clk);
				}
			}
		}
	}

	audio_switch_disable_clock();
	I2S_DEBUG_PRINT(" i2s disable clock count 0x%x \n", info->clk_refs);
	return 0;
}

int i2s_clock_rate(int ifc, int rate)
{
	struct i2s_controller_info *info = &i2s_cont_info[ifc];
	info->i2sprop.clk_rate = rate;
	return 0;
}

