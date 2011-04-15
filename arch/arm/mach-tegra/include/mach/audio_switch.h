 /*
 *
 * arch/arm/mach-tegra/include/mach/audio_switch.h
 *
 * Header and functions for audio switch and apbif module
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

#ifndef __ARCH_ARM_MACH_AUDIO_SWITCH_H
#define __ARCH_ARM_MACH_AUDIO_SWITCH_H

#include <linux/kernel.h>
#include <linux/types.h>

#define AUDIOCIF_EXPAND_ZERO	0
#define AUDIOCIF_EXPAND_ONE	1
#define AUDIOCIF_EXPAND_LFSR	2

/* generic macro to set modes */
#define set_reg_mode(r,m,v) {	\
	((r) &= (~(m)));	\
	((r) |= (v)?(m):(0));	\
}

typedef enum ahubtx_
{
	ahubtx0_apbif = 0,
	ahubtx1_apbif,
	ahubtx2_apbif,
	ahubtx3_apbif,
	ahubtx_i2s0,
	ahubtx_i2s1,
	ahubtx_i2s2,
	ahubtx_i2s3,
	ahubtx_i2s4,
	ahubtx_dam0,
	ahubtx_dam1,
	ahubtx_dam2,
	ahubtx0_spdif,
	ahubtx1_spdif,
	ahubtx_maxnum
} ahubtx;

typedef enum ahubrx_
{
	ahubrx0_apbif = 0x0,
	ahubrx1_apbif,
	ahubrx2_apbif,
	ahubrx3_apbif,
	ahubrx_i2s0,
	ahubrx_i2s1,
	ahubrx_i2s2,
	ahubrx_i2s3,
	ahubrx_i2s4,
	ahubrx0_dam0,
	ahubrx1_dam0,
	ahubrx0_dam1,
	ahubrx1_dam1,
	ahubrx0_dam2,
	ahubrx1_dam2,
	ahubrx0_spdif,
	ahubrx1_spdif,
	ahubrx_maxnum
} ahubrx;

typedef enum dam_chtype_
{
	dam_ch_in0 = 0x0,
	dam_ch_in1,
	dam_ch_out,
	dam_ch_maxnum
}dam_chtype;

struct audio_cif
{
	int threshold;
	int audio_channels;
	int client_channels;
	int audio_bits;
	int client_bits;
	int expand;
	int stereo_conv;
	int replicate;
	int direction;
	int truncate;
	int mono_conv;
};

/*
 * API
 */

void apbif_dump_registers(int ifc);
void audio_switch_dump_registers(int ifc);
int  apbif_get_fifo_mode(int ifc, int tx);
int  apbif_get_fifo_freecount(int ifc, int tx);
phys_addr_t apbif_get_fifo_phy_base(int ifc, int tx);
int  apbif_fifo_set_attention_level(int ifc, int tx, unsigned level);
void apbif_set_pack_mode(int ifc, int tx, int pack_mode);
void apbif_channel_set_loopback(int ifc, int on);
void apbif_channel_enable(int ifc, int tx, int enable);
int  apbif_get_channel(int regindex, int fifo_mode);

void apbif_soft_reset(int ifc, int fifo_mode, int enable);
void apbif_fifo_write(int ifc, int fifo_mode, u32 data);
u32 apbif_fifo_read(int ifc, int fifo_mode);
void audio_switch_set_acif(int addr, struct audio_cif *cifInfo);
int  audio_switch_get_rx_port(int rxport);
void audio_switch_set_rx_port(int rxport, int txport);

int audio_switch_close(void);
int audio_switch_open(void);
int audio_apbif_free_channel(int ifc, int fifo_mode);
int audio_apbif_set_acif(int ifc, int fifo_mode, struct audio_cif *cifInfo);
int audio_switch_suspend(void);
int audio_switch_resume(void);
int audio_switch_enable_clock(void);
void audio_switch_disable_clock(void);

void dam_enable(int ifc, int on, int chtype);
void dam_set_samplerate(int ifc, int chtype, int samplerate);

/* FIXME: move to dam.h if needed */
int dam_open(void);
int dam_close(void);
int dam_set_acif(int ifc, int chtype, struct audio_cif *cifInfo);
int dam_get_controller(void);
int dam_free_controller(int ifc);
int dam_suspend(int ifc);
int dam_resume(int ifc);
int dam_set_clock_rate(int rate);
int dam_set_clock_parent(int ifc, int parent);
int dam_enable_clock(int ifc);
void dam_disable_clock(int ifc);

#endif /* __ARCH_ARM_MACH_AUDIO_SWITCH_H */
