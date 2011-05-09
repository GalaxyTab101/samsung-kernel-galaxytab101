/*
 * arch/arm/mach-tegra/include/mach/audio_manager.h
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

#ifndef __ARCH_ARM_MACH_TEGRA_AUDIO_MANAGER_H
#define __ARCH_ARM_MACH_TEGRA_AUDIO_MANAGER_H

#include <linux/kernel.h>
#include <linux/types.h>

#define AUDIO_I2S_DEVICE	0
#define AUDIO_SPDIF_DEVICE	1
#define AUDIO_MAX_DEVICE	2

typedef struct aud_dev_info_ {
	int dev_type;
	int dev_id;
	int fifo_mode;
	unsigned long base;
	phys_addr_t phy_base;
}aud_dev_info;

typedef struct am_stream_format_info_{
	int samplerate;
	int channels;
	int bitsize;
	int buffersize;
}am_stream_format_info;

typedef struct am_dev_format_info_{
	int mastermode;
	int audiomode;
	int polarity;
	int clkrate;
	int fifofmt;
	int loopmode;
}am_dev_format_info;

struct am_dev_fns {
	int (*aud_dev_suspend)(int);
	int (*aud_dev_resume)(int);
	int (*aud_dev_set_stream_state)(int, int, int);
	int (*aud_dev_get_dma_requestor)(int, int);
	int (*aud_dev_free_dma_requestor)(int, int);
	phys_addr_t (*aud_dev_get_fifo_phy_base)(int, int);
	int (*aud_dev_set_fifo_attention)(int, int, int);
	u32 (*aud_dev_get_status)(int, int);
	int (*aud_dev_clock_disable)(int, int);
	int (*aud_dev_clock_enable)(int, int);
	int (*aud_dev_clock_set_parent)(int, int, int);
	int (*aud_dev_clock_set_rate)(int, int, int);
	int (*aud_dev_deinit)(int);
};

int am_suspend(aud_dev_info* devinfo);
int am_resume(aud_dev_info* devinfo);
int am_set_stream_state(aud_dev_info* devinfo, bool enable);
int am_get_dma_requestor(aud_dev_info* devinfo);
int am_free_dma_requestor(aud_dev_info* devinfo);
phys_addr_t am_get_fifo_phy_base(aud_dev_info* devinfo);
int am_set_fifo_attention(aud_dev_info* devinfo, int buffersize);
u32 am_get_status(aud_dev_info* devinfo);
int am_clock_disable(aud_dev_info* devinfo);
int am_clock_enable(aud_dev_info* devinfo);
int am_clock_set_parent(aud_dev_info* devinfo, int parent);
int am_clock_set_rate(aud_dev_info* devinfo, int rate);

int am_device_init(aud_dev_info* devinfo, void *dev_fmt, void  *strm_fmt);
int am_device_deinit(aud_dev_info* devinfo);
int am_set_stream_format(aud_dev_info* devinfo, am_stream_format_info *format);
int am_set_device_format(aud_dev_info* devinfo, am_dev_format_info *format);

#endif /* __ARCH_ARM_MACH_TEGRA_AUDIO_MANAGER_H */
