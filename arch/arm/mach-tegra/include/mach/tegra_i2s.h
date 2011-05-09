/*
 * arch/arm/mach-tegra/include/mach/tegra_i2s.h
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

#ifndef __ARCH_ARM_MACH_TEGRA_I2S_H
#define __ARCH_ARM_MACH_TEGRA_I2S_H

#include <linux/kernel.h>
#include <linux/types.h>

/*
* Struct defined to set all the needed i2s format informations
*/
struct tegra_i2s_channel_property {
	int channletype;	/* tx or rx */
	int dataoffset;		/* Data offset to Fsync */
	int bit_order;		/* MSB/LSB first */
	int bit_mask;		/* mask bits to get exact bit size in PCM */
	int num_slot;		/* no of slots in the channel in TDM mode */
	int fifo_attn;		/* fifo attention */
	int dma_ch;		/* dma channel being used */
	int stream_index;	/* stream index being used */
};

struct tegra_i2s_property {

	int audio_mode;		/* I2S,LJM,RJM,DSP,PCM,NW,TDM*/
	bool master_mode;	/* master/slave */
	int lrck_polarity;	/* low on left/ high on left */
	int bit_code;		/* linear/uLaw/alaw */
	int bit_size;		/* 8/12/16/20/24/28/32 */
	int fifo_fmt;		/* fifo format */

	int channels;		/* channels */
	int sample_rate;	/* Sample rate */
	unsigned long clk_rate; /* clock rate */

	int total_slots;	/* no of slots per fsync */
	int fsync_width;	/* Fsync width in terms of bit clocks */
	int highz_control;	/* Highz control */
	int edge_control;	/* sample data on edge */

	struct clk *i2s_clk;
	struct clk *i2s_sync_clk;
	struct clk *audio_clk;
	struct clk *audio2x_clk;
};

/* APIs */
void i2s_dump_registers(int ifc);
int i2s_suspend(int ifc);
int i2s_resume(int ifc);
int i2s_fifo_enable(int ifc, int fifo, int on);
int i2s_fifo_clear(int ifc, int fifo);
int i2s_fifo_set_attention_level(int ifc, int fifo, unsigned level);
int i2s_get_dma_requestor(int ifc, int fifo_mode);
int i2s_free_dma_requestor(int ifc, int fifo_mode);
phys_addr_t i2s_get_fifo_phy_base(int ifc, int fifo);
u32 i2s_get_status(int ifc, int fifo);

int i2s_set_loopback(int ifc, int on);
int i2s_set_master(int ifc, int master);
int i2s_set_left_right_control_polarity(int ifc, int left_low);
int i2s_set_channel_bit_count(int ifc, int sampling, int bitclk);
int i2s_init(int ifc, struct tegra_i2s_property* pi2sprop);
int i2s_set_bit_format(int ifc, unsigned format);
int i2s_set_bit_size(int ifc, unsigned bit_size);

int i2s_set_fifo_irq_on_err(int ifc, int fifo, int on);
int i2s_set_fifo_irq_on_qe(int ifc, int fifo, int on);

int i2s_fifo_write(int ifc, int fifo, u32 data);
u32 i2s_fifo_read(int ifc, int fifo);

u32 i2s_get_control(int ifc);
int i2s_ack_status(int ifc);
u32 i2s_get_fifo_scr(int ifc);
u32 i2s_get_fifo_full_empty_count(int ifc, int fifo);
struct clk *i2s_get_clock_by_name(const char *name);

int i2s_set_fifo_attention(int ifc, int fifo_mode, int buffersize);
int i2s_set_samplerate(int ifc, int samplerate);
int i2s_set_channels(int ifc, int channels);

int i2s_clock_disable(int ifc, int fifo_mode);
int i2s_clock_enable(int ifc, int fifo_mode);
int i2s_close(int ifc);
int i2s_clock_set_parent(int ifc, int mode, int parent);
int i2s_clock_set_rate(int ifc, int mode, int rate);
#endif /* __ARCH_ARM_MACH_TEGRA_I2S_H */
