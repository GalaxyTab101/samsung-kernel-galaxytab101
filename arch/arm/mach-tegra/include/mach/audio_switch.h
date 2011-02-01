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
#define AUDIOCIF_EXPAND_ONE		1
#define AUDIOCIF_EXPAND_LFSR	2

#define AUDIO_PACK_NOP		0
#define AUDIO_PACK_RSVD		1
#define AUDIO_PACK_8_4		2
#define AUDIO_PACK_16		3

/* generic macro to set modes */
#define set_reg_mode(r,m,v) {	\
	((r) &= (~(m)));			\
	((r) |= (v)?(m):(0));		\
}

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

//struct tegra_audio_sw
//{
//	struct tegra_audio_module[];
//}

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
int  apbif_get_channel(int ifc);
int  apbif_initialize(int ifc, struct audio_cif *cifInfo);
int  apbif_enable_clock(void);
void audio_switch_set_acif(int addr, struct audio_cif *cifInfo);
int  audio_switch_get_rx_port(int rxport);
void audio_switch_set_rx_port(int rxport, int txport);

#endif /* __ARCH_ARM_MACH_AUDIO_SWITCH_H */
