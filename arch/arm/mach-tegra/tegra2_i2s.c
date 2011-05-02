/*
 * arch/arm/mach-tegra/tegra2_i2s.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *      Iliyan Malchev <malchev@google.com>
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

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include "clock.h"
#include <linux/clk.h>
#include <asm/io.h>
#include <mach/iomap.h>
#include <mach/audio.h>
#include <mach/tegra2_i2s.h>


#define NR_I2S_IFC	2

#define check_ifc(n, ...) if ((n) > NR_I2S_IFC) {			\
	pr_err("%s: invalid i2s interface %d\n", __func__, (n));	\
	return __VA_ARGS__;						\
}

struct i2s_runtime_data {
	int i2s_ctrl_0;
	int i2s_status_0;
	int i2s_timing_0;
	int i2s__fifo_scr_0;
	int i2s_pcm_ctrl_0;
	int i2s_nw_ctrl_0;
	int i2s_tdm_ctrl_0;
	int i2s_tdm_tx_rx_ctrl_0;
	int i2s_fifo1_0;
	int i2s_fifo2_0;
};

struct i2s_controller_info {
	struct tegra_i2s_channel_property i2s_ch_prop[AUDIO_FIFO_CNT];
	struct tegra_i2s_property i2sprop;
	struct i2s_runtime_data i2s_reg_data;
	int	clk_refs;
};

static phys_addr_t i2s_phy_base[NR_I2S_IFC] = {
	TEGRA_I2S1_BASE,
	TEGRA_I2S2_BASE,
};

static void *i2s_base[NR_I2S_IFC] = {
	IO_ADDRESS(TEGRA_I2S1_BASE),
	IO_ADDRESS(TEGRA_I2S2_BASE),
};

struct i2s_clk_info {
	char *clk_name;
};

static struct i2s_clk_info i2sclk_info[NR_I2S_IFC] = {
	{"i2s1"},
	{"i2s2"},
};

static struct i2s_controller_info i2s_cont_info[NR_I2S_IFC];

static inline void i2s_writel(int ifc, u32 val, u32 reg)
{
	__raw_writel(val, i2s_base[ifc] + reg);
}

static inline u32 i2s_readl(int ifc, u32 reg)
{
	return __raw_readl(i2s_base[ifc] + reg);
}

void i2s_dump_registers(int ifc)
{
	check_ifc(ifc);

	pr_info("%s: CTRL   %08x\n", __func__,
			i2s_readl(ifc, I2S_I2S_CTRL_0));
	pr_info("%s: STATUS %08x\n", __func__,
			i2s_readl(ifc, I2S_I2S_STATUS_0));
	pr_info("%s: TIMING %08x\n", __func__,
			i2s_readl(ifc, I2S_I2S_TIMING_0));
	pr_info("%s: SCR    %08x\n", __func__,
			i2s_readl(ifc, I2S_I2S_FIFO_SCR_0));
	pr_info("%s: FIFO1  %08x\n", __func__,
			i2s_readl(ifc, I2S_I2S_FIFO1_0));
	pr_info("%s: FIFO2  %08x\n", __func__,
			i2s_readl(ifc, I2S_I2S_FIFO1_0));
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

void i2s_suspend(int ifc)
{
	struct i2s_controller_info *info = &i2s_cont_info[ifc];
	struct i2s_runtime_data* ird = &info->i2s_reg_data;

	check_ifc(ifc);

	if (info->clk_refs == 0)
		i2s_clock_enable(ifc);

	ird->i2s_ctrl_0 = i2s_readl(ifc, I2S_I2S_CTRL_0);
	ird->i2s_status_0 = i2s_readl(ifc, I2S_I2S_STATUS_0);
	ird->i2s_timing_0 = i2s_readl(ifc, I2S_I2S_TIMING_0);
	ird->i2s__fifo_scr_0 = i2s_readl(ifc, I2S_I2S_FIFO_SCR_0);
	ird->i2s_pcm_ctrl_0 = i2s_readl(ifc, I2S_I2S_PCM_CTRL_0);
	ird->i2s_nw_ctrl_0 = i2s_readl(ifc, I2S_I2S_NW_CTRL_0);
	ird->i2s_tdm_ctrl_0 = i2s_readl(ifc, I2S_I2S_TDM_CTRL_0);
	ird->i2s_tdm_tx_rx_ctrl_0 = i2s_readl(ifc, I2S_I2S_TDM_TX_RX_CTRL_0);
	ird->i2s_fifo1_0 = i2s_readl(ifc, I2S_I2S_FIFO1_0);
	ird->i2s_fifo2_0 = i2s_readl(ifc, I2S_I2S_FIFO2_0);

	i2s_clock_disable(ifc);
}

void i2s_resume(int ifc)
{
	struct i2s_runtime_data* ird = &i2s_cont_info[ifc].i2s_reg_data;

	check_ifc(ifc);

	i2s_clock_enable(ifc);
	i2s_writel(ifc, ird->i2s_ctrl_0, I2S_I2S_CTRL_0);
	i2s_writel(ifc, ird->i2s_status_0, I2S_I2S_STATUS_0);
	i2s_writel(ifc, ird->i2s_timing_0, I2S_I2S_TIMING_0);
	i2s_writel(ifc, ird->i2s__fifo_scr_0, I2S_I2S_FIFO_SCR_0);
	i2s_writel(ifc, ird->i2s_pcm_ctrl_0, I2S_I2S_PCM_CTRL_0);
	i2s_writel(ifc, ird->i2s_nw_ctrl_0, I2S_I2S_NW_CTRL_0);
	i2s_writel(ifc, ird->i2s_tdm_ctrl_0, I2S_I2S_TDM_CTRL_0);
	i2s_writel(ifc, ird->i2s_tdm_tx_rx_ctrl_0, I2S_I2S_TDM_TX_RX_CTRL_0);
	i2s_writel(ifc, ird->i2s_fifo1_0, I2S_I2S_FIFO1_0);
	i2s_writel(ifc, ird->i2s_fifo2_0, I2S_I2S_FIFO2_0);
}

int i2s_set_channel_bit_count(int ifc, int sampling, int bitclk)
{
	u32 val;
	int bitcnt;

	check_ifc(ifc, -EINVAL);

	bitcnt = bitclk / (2 * sampling) - 1;

	if (bitcnt < 0 || bitcnt >= 1<<11) {
		pr_err("%s: bit count %d is out of bounds\n", __func__,
			bitcnt);
		return -EINVAL;
	}

	val = bitcnt;
	if (bitclk % (2 * sampling)) {
		pr_info("%s: enabling non-symmetric mode\n", __func__);
		val |= I2S_I2S_TIMING_NON_SYM_ENABLE;
	}

	i2s_writel(ifc, val, I2S_I2S_TIMING_0);
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

void i2s_set_fifo_mode(int ifc, int fifo, int tx)
{
	u32 val;

	check_ifc(ifc);

	val = i2s_readl(ifc, I2S_I2S_CTRL_0);
	if (fifo == 0) {
		val &= ~I2S_I2S_CTRL_FIFO1_RX_ENABLE;
		val |= (!tx) ? I2S_I2S_CTRL_FIFO1_RX_ENABLE : 0;
	}
	else {
		val &= ~I2S_I2S_CTRL_FIFO2_TX_ENABLE;
		val |= tx ? I2S_I2S_CTRL_FIFO2_TX_ENABLE : 0;
	}
	i2s_writel(ifc, val, I2S_I2S_CTRL_0);
}

void i2s_set_loopback(int ifc, int on)
{
	u32 val;

	check_ifc(ifc);

	val = i2s_readl(ifc, I2S_I2S_CTRL_0);
	val &= ~I2S_I2S_CTRL_FIFO_LPBK_ENABLE;
	val |= on ? I2S_I2S_CTRL_FIFO_LPBK_ENABLE : 0;

	i2s_writel(ifc, val, I2S_I2S_CTRL_0);
}

int i2s_fifo_set_attention_level(int ifc, int fifo, unsigned level)
{
	u32 val;

	check_ifc(ifc, -EINVAL);

	if (level > I2S_FIFO_ATN_LVL_TWELVE_SLOTS) {
		pr_err("%s: invalid fifo level selector %d\n", __func__,
			level);
		return -EINVAL;
	}

	val = i2s_readl(ifc, I2S_I2S_FIFO_SCR_0);

	if (!fifo) {
		val &= ~I2S_I2S_FIFO_SCR_FIFO1_ATN_LVL_MASK;
		val |= level << I2S_FIFO1_ATN_LVL_SHIFT;
	}
	else {
		val &= ~I2S_I2S_FIFO_SCR_FIFO2_ATN_LVL_MASK;
		val |= level << I2S_FIFO2_ATN_LVL_SHIFT;
	}

	i2s_writel(ifc, val, I2S_I2S_FIFO_SCR_0);
	return 0;
}

void i2s_fifo_enable(int ifc, int fifo, int on)
{
	u32 val;
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	check_ifc(ifc);

	val = i2s_readl(ifc, I2S_I2S_CTRL_0);
	if (!fifo) {
		val &= ~I2S_I2S_CTRL_FIFO1_ENABLE;
		val |= on ? I2S_I2S_CTRL_FIFO1_ENABLE : 0;
	}
	else {
		val &= ~I2S_I2S_CTRL_FIFO2_ENABLE;
		val |= on ? I2S_I2S_CTRL_FIFO2_ENABLE : 0;
	}

	if (on)
		i2s_fifo_set_attention_level(ifc, fifo,
				info->i2s_ch_prop[fifo].fifo_attn);

	i2s_writel(ifc, val, I2S_I2S_CTRL_0);
}

void i2s_fifo_clear(int ifc, int fifo)
{
	u32 val;

	check_ifc(ifc);

	val = i2s_readl(ifc, I2S_I2S_FIFO_SCR_0);
	if (!fifo) {
		val &= ~I2S_I2S_FIFO_SCR_FIFO1_CLR;
		val |= I2S_I2S_FIFO_SCR_FIFO1_CLR;
	}
	else {
		val &= ~I2S_I2S_FIFO_SCR_FIFO2_CLR;
		val |= I2S_I2S_FIFO_SCR_FIFO2_CLR;
	}

	i2s_writel(ifc, val, I2S_I2S_FIFO_SCR_0);
}

void i2s_set_master(int ifc, int master)
{
	u32 val;
	check_ifc(ifc);
	val = i2s_readl(ifc, I2S_I2S_CTRL_0);
	val &= ~I2S_I2S_CTRL_MASTER_ENABLE;
	val |= master ? I2S_I2S_CTRL_MASTER_ENABLE : 0;
	i2s_writel(ifc, val, I2S_I2S_CTRL_0);
}

int i2s_set_bit_format(int ifc, unsigned fmt)
{
	u32 val;

	check_ifc(ifc, -EINVAL);

	if (fmt > AUDIO_FRAME_FORMAT_DSP) {
		pr_err("%s: invalid bit-format selector %d\n", __func__, fmt);
		return -EINVAL;
	}

	val = i2s_readl(ifc, I2S_I2S_CTRL_0);
	val &= ~I2S_I2S_CTRL_BIT_FORMAT_MASK;
	val |= fmt << I2S_BIT_FORMAT_SHIFT;

	i2s_writel(ifc, val, I2S_I2S_CTRL_0);

	if (fmt == AUDIO_FRAME_FORMAT_DSP) {
		i2s_enable_pcm_mode(ifc, 1);
	}
	else {
		i2s_enable_pcm_mode(ifc, 0);
	}
	return 0;
}

int i2s_set_bit_size(int ifc, unsigned bit_size)
{
	u32 val;

	check_ifc(ifc, -EINVAL);

	val = i2s_readl(ifc, I2S_I2S_CTRL_0);
	val &= ~I2S_I2S_CTRL_BIT_SIZE_MASK;

	if (bit_size > I2S_BIT_SIZE_32) {
		pr_err("%s: invalid bit_size selector %d\n", __func__,
			bit_size);
		return -EINVAL;
	}

	val |= bit_size << I2S_BIT_SIZE_SHIFT;

	i2s_writel(ifc, val, I2S_I2S_CTRL_0);
	return 0;
}

int i2s_set_fifo_format(int ifc, unsigned fmt)
{
	u32 val;

	check_ifc(ifc, -EINVAL);

	val = i2s_readl(ifc, I2S_I2S_CTRL_0);
	val &= ~I2S_I2S_CTRL_FIFO_FORMAT_MASK;

	if (fmt > I2S_FIFO_32 && fmt != I2S_FIFO_PACKED) {
		pr_err("%s: invalid fmt selector %d\n", __func__, fmt);
		return -EINVAL;
	}

	val |= fmt << I2S_FIFO_SHIFT;

	i2s_writel(ifc, val, I2S_I2S_CTRL_0);
	return 0;
}

void i2s_set_left_right_control_polarity(int ifc, int high_low)
{
	u32 val;

	check_ifc(ifc);

	val = i2s_readl(ifc, I2S_I2S_CTRL_0);
	val &= ~I2S_I2S_CTRL_L_R_CTRL;
	val |= high_low ? I2S_I2S_CTRL_L_R_CTRL : 0;
	i2s_writel(ifc, val, I2S_I2S_CTRL_0);
}

void i2s_enable_pcm_mode(int ifc, int on)
{
	u32 val;

	check_ifc(ifc);

	val = i2s_readl(ifc, I2S_I2S_PCM_CTRL_0);
	val &= ~(I2S_I2S_PCM_CTRL_TRM_MODE | I2S_I2S_PCM_CTRL_RCV_MODE);
	val |= on ? (I2S_I2S_PCM_CTRL_TRM_MODE | I2S_I2S_PCM_CTRL_RCV_MODE) : 0;
	i2s_writel(ifc, val, I2S_I2S_PCM_CTRL_0);
}

int i2s_set_pcm_edge_mode(int ifc, unsigned edge_mode)
{
	u32 val;

	check_ifc(ifc, -EINVAL);

	if (edge_mode > I2S_I2S_PCM_TRM_EDGE_NEG_EDGE_HIGHZ) {
		pr_err("%s: invalid dsp edge mode \n", __func__);
		return -EINVAL;
	}

	val = i2s_readl(ifc, I2S_I2S_PCM_CTRL_0);
	val &= ~I2S_I2S_PCM_TRM_EDGE_CTRL_MASK;
	val |= edge_mode << I2S_PCM_TRM_EDGE_CTRL_SHIFT;

	i2s_writel(ifc, val, I2S_I2S_PCM_CTRL_0);
	return 0;
}

int i2s_set_pcm_mask_bits(int ifc, unsigned mask_bits, int tx)
{
	u32 val;

	check_ifc(ifc, -EINVAL);

	val = i2s_readl(ifc, I2S_I2S_PCM_CTRL_0);
	if (tx) {
		if (mask_bits > I2S_I2S_PCM_TRM_MASK_BITS_SEVEN) {
			pr_err("%s: invalid dsp mask bits \n", __func__);
			return -EINVAL;
		}
		val &= ~I2S_I2S_PCM_TRM_MASK_BITS_MASK;
		val |= mask_bits << I2S_PCM_TRM_MASK_BITS_SHIFT;
	}
	else {
		if (mask_bits > I2S_I2S_PCM_RCV_MASK_BITS_SEVEN) {
			pr_err("%s: invalid dsp mask bits \n", __func__);
			return -EINVAL;
		}
		val &= ~I2S_I2S_PCM_RCV_MASK_BITS_MASK;
		val |= mask_bits << I2S_PCM_RCV_MASK_BITS_SHIFT;
	}
	i2s_writel(ifc, val, I2S_I2S_PCM_CTRL_0);
	return 0;
}

void i2s_set_pcm_fsync_width(int ifc, int fsync_long)
{
	u32 val;

	check_ifc(ifc);

	val = i2s_readl(ifc, I2S_I2S_PCM_CTRL_0);
	val &= ~I2S_I2S_PCM_CTRL_FSYNC_PCM_CTRL;
	val |= fsync_long ? I2S_I2S_PCM_CTRL_FSYNC_PCM_CTRL : 0;

	i2s_writel(ifc, val, I2S_I2S_PCM_CTRL_0);
}

void i2s_set_fifo_irq_on_err(int ifc, int fifo, int on)
{
	u32 val;

	check_ifc(ifc);

	val = i2s_readl(ifc, I2S_I2S_CTRL_0);
	if (!fifo) {
		val &= ~I2S_I2S_IE_FIFO1_ERR;
		val |= on ? I2S_I2S_IE_FIFO1_ERR : 0;
	}
	else {
		val &= ~I2S_I2S_IE_FIFO2_ERR;
		val |= on ? I2S_I2S_IE_FIFO2_ERR : 0;
	}
	i2s_writel(ifc, val, I2S_I2S_CTRL_0);
}

void i2s_set_fifo_irq_on_qe(int ifc, int fifo, int on)
{
	u32 val;

	check_ifc(ifc);

	val = i2s_readl(ifc, I2S_I2S_CTRL_0);
	if (!fifo) {
		val &= ~I2S_I2S_QE_FIFO1;
		val |= on ? I2S_I2S_QE_FIFO1 : 0;
	}
	else {
		val &= ~I2S_I2S_QE_FIFO2;
		val |= on ? I2S_I2S_QE_FIFO2 : 0;
	}
	i2s_writel(ifc, val, I2S_I2S_CTRL_0);
}

void i2s_set_fifo_attention(int ifc, int buffersize, int fifo_mode)
{
	int fifoattn = I2S_FIFO_ATN_LVL_FOUR_SLOTS;
	struct i2s_controller_info *info = &i2s_cont_info[ifc];
	info->i2s_ch_prop[fifo_mode].fifo_attn = fifoattn;
}

void i2s_enable_fifos(int ifc, int on)
{
	u32 val;

	check_ifc(ifc);

	val = i2s_readl(ifc, I2S_I2S_CTRL_0);
	if (on)
		val |= I2S_I2S_QE_FIFO1 | I2S_I2S_QE_FIFO2 |
		       I2S_I2S_IE_FIFO1_ERR | I2S_I2S_IE_FIFO2_ERR;
	else
		val &= ~(I2S_I2S_QE_FIFO1 | I2S_I2S_QE_FIFO2 |
			 I2S_I2S_IE_FIFO1_ERR | I2S_I2S_IE_FIFO2_ERR);

	i2s_writel(ifc, val, I2S_I2S_CTRL_0);
}

void i2s_fifo_write(int ifc, int fifo, u32 data)
{
	check_ifc(ifc);
	i2s_writel(ifc, data, fifo ? I2S_I2S_FIFO2_0 : I2S_I2S_FIFO1_0);
}

u32 i2s_fifo_read(int ifc, int fifo)
{
	check_ifc(ifc, 0);
	return i2s_readl(ifc, fifo ? I2S_I2S_FIFO2_0 : I2S_I2S_FIFO1_0);
}

u32 i2s_get_status(int ifc, int fifo)
{
	int regval = 0;
	check_ifc(ifc, 0);
	regval = i2s_readl(ifc, I2S_I2S_STATUS_0);

	if (fifo == AUDIO_TX_MODE)
		regval &= I2S_I2S_FIFO_TX_BUSY;
	else
		regval &= I2S_I2S_FIFO_RX_BUSY;

	return regval;
}

u32 i2s_get_control(int ifc)
{
	check_ifc(ifc, 0);
	return i2s_readl(ifc, I2S_I2S_CTRL_0);
}

void i2s_ack_status(int ifc)
{
	check_ifc(ifc);
	return i2s_writel(ifc, i2s_readl(ifc, I2S_I2S_STATUS_0),
						 I2S_I2S_STATUS_0);
}

u32 i2s_get_fifo_scr(int ifc)
{
	check_ifc(ifc, 0);
	return i2s_readl(ifc, I2S_I2S_FIFO_SCR_0);
}

phys_addr_t i2s_get_fifo_phy_base(int ifc, int fifo)
{
	check_ifc(ifc, 0);
	return i2s_phy_base[ifc] + (fifo ? I2S_I2S_FIFO2_0 : I2S_I2S_FIFO1_0);
}

u32 i2s_get_fifo_full_empty_count(int ifc, int fifo)
{
	u32 val;

	check_ifc(ifc, 0);

	val = i2s_readl(ifc, I2S_I2S_FIFO_SCR_0);

	if (!fifo)
		val = val >> I2S_I2S_FIFO_SCR_FIFO1_FULL_EMPTY_COUNT_SHIFT;
	else
		val = val >> I2S_I2S_FIFO_SCR_FIFO2_FULL_EMPTY_COUNT_SHIFT;

	return val & I2S_I2S_FIFO_SCR_FIFO_FULL_EMPTY_COUNT_MASK;
}


struct clk *i2s_get_clock_by_name(const char *name)
{
    return tegra_get_clock_by_name(name);
}

int i2s_free_dma_requestor(int ifc, int  fifo)
{
	/* NULL function */
	return 0;
}

int i2s_get_dma_requestor(int ifc, int  fifo)
{
	return ((ifc)? 1 : 2); /* 1 = I2S2, 2 = I2S1 */
}

int i2s_close(int ifc)
{
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	if (info->i2sprop.i2s_clk)
		clk_put(info->i2sprop.i2s_clk);

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

	memcpy(&info->i2sprop,pi2sprop,sizeof(struct tegra_i2s_property));

	if (i2s_open(ifc))
		return err;

	i2s_clock_rate(ifc, pi2sprop->clk_rate);
	i2s_clock_set_parent(ifc, 0);

	err = i2s_clock_enable(ifc);

	if (err) {
		i2s_close(ifc);
		return err;
	}

	i2s_enable_fifos(ifc, 0);
	i2s_fifo_clear(ifc, AUDIO_TX_MODE);
	i2s_fifo_clear(ifc, AUDIO_RX_MODE);
	i2s_set_left_right_control_polarity(ifc, AUDIO_LRCK_LEFT_LOW);
	i2s_set_master(ifc, pi2sprop->master_mode);

	i2s_set_fifo_mode(ifc, AUDIO_TX_MODE, 1);
	i2s_set_fifo_mode(ifc, AUDIO_RX_MODE, 0);

	i2s_set_bit_format(ifc, pi2sprop->audio_mode);
	i2s_set_bit_size(ifc, pi2sprop->bit_size);
	i2s_set_fifo_format(ifc, pi2sprop->fifo_fmt);

	i2s_clock_disable(ifc);

	return 0;
}


int i2s_clock_enable(int ifc)
{
	int err = 0;
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

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
	}

	return err;
}

int i2s_clock_disable(int ifc)
{
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	if (info->i2sprop.i2s_clk &&
		 (info->i2sprop.master_mode == true) &&
		 (info->clk_refs > 0)) {
		info->clk_refs--;
		if (info->clk_refs <= 0) {
			clk_disable(info->i2sprop.i2s_clk);
			info->clk_refs = 0;
		}
	}

	return 0;
}

int i2s_clock_set_parent(int ifc, int parent)
{
	/* Fix set the parent properly */
	struct clk *pll_a_out0_clk = clk_get_sys(NULL, "pll_a_out0");
	struct i2s_controller_info *info = &i2s_cont_info[ifc];

	if (info->i2sprop.i2s_clk)
		clk_set_parent(info->i2sprop.i2s_clk, pll_a_out0_clk);

	return 0;
}

int i2s_clock_rate(int ifc, int rate)
{
	struct i2s_controller_info *info = &i2s_cont_info[ifc];
	info->i2sprop.clk_rate = rate;
	return 0;
}

