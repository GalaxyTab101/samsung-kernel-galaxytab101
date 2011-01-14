/*
 * arch/arm/mach-tegra/dam.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *      Iliyan Malchev <malchev@google.com>
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

#include "clock.h"
#include <asm/io.h>
#include <mach/iomap.h>

#define NR_DAM_IFC	3

#define check_dam_ifc(n, ...) if ((n) > NR_DAM_IFC) {			\
	pr_err("%s: invalid dam interface %d\n", __func__, (n));	\
	return __VA_ARGS__;						\
}

/* Offsets from TEGRA_DAM1_BASE, TEGRA_DAM2_BASE and TEGRA_DAM3_BASE */
#define DAM_CTRL_0					0
#define DAM_CLIP_0					4
#define DAM_CLIP_THRESHOLD_0		8
#define DAM_AUDIOCIF_OUT_CTRL_0		0x0C
#define DAM_CH0_CTRL_0				0x10
#define DAM_CH0_CONV_0				0x14
#define DAM_AUDIOCIF_CH0_CTRL_0		0x1C
#define DAM_CH1_CTRL_0				0x20
#define DAM_CH1_CONV_0				0x24
#define DAM_AUDIOCIF_CH1_CTRL_0		0x2C

#define DAM_NUM_INPUT_CHANNELS		2
#define DAM_FS_8KHZ  				0
#define DAM_FS_16KHZ 				1
#define DAM_FS_44KHZ 				2
#define DAM_FS_48KHZ 				3

/*
DAM_CTRL_0
*/
#define DAM_CTRL_0_SOFT_RESET_ENABLE	(1<<31)

#define DAM_CTRL_0_FSOUT_SHIFT	4
#define DAM_CTRL_0_FSOUT_MASK	(0xf<<define DAM_CTRL_0_FSOUT_SHIFT)
#define DAM_CTRL_0_FSOUT_FS8	(DAM_FS_8KHZ<<DAM_CTRL_0_FSOUT_SHIFT)
#define DAM_CTRL_0_FSOUT_FS16	(DAM_FS_16KHZ<<DAM_CTRL_0_FSOUT_SHIFT)
#define DAM_CTRL_0_FSOUT_FS44	(DAM_FS_44KHZ<<DAM_CTRL_0_FSOUT_SHIFT)
#define DAM_CTRL_0_FSOUT_FS48	(DAM_FS_48KHZ<<DAM_CTRL_0_FSOUT_SHIFT)
#define DAM_CTRL_0_CG_EN		(1<<1)
#define DAM_CTRL_0_DAM_EN		(1<<0)

/*
DAM_CLIP_0
*/

#define DAM_CLIP_0_COUNTER_ENABLE	(1<<31)
#define DAM_CLIP_0_COUNT_MASK		0x7fffffff

/*
DAM_CLIP_THRESHOLD_0
*/
#define DAM_CLIP_THRESHOLD_0_VALUE_SHIFT	8
#define DAM_CLIP_THRESHOLD_0_VALUE_MASK		\
		(0x7fffff<<DAM_CLIP_THRESHOLD_0_VALUE_SHIFT)
#define DAM_CLIP_THRESHOLD_0_VALUE			(1<<31)
#define DAM_CLIP_THRESHOLD_0_COUNT_SHIFT	0


#define STEP_RESET		1
#define DATA_SYNC		1

/*
DAM_CH0_CTRL_0
*/
#define DAM_CH0_CTRL_0_FSIN_SHIFT	8
#define DAM_CH0_CTRL_0_STEP_SHIFT	16
#define DAM_CH0_CTRL_0_STEP_MASK	(0xffff<<16)
#define DAM_CH0_CTRL_0_STEP_RESET	(STEP_RESET<<16)
#define DAM_CH0_CTRL_0_FSIN_MASK	(0xf<<8)
#define DAM_CH0_CTRL_0_FSIN_FS8		(DAM_FS_8KHZ<<8)
#define DAM_CH0_CTRL_0_FSIN_FS16	(DAM_FS_16KHZ<<8)
#define DAM_CH0_CTRL_0_FSIN_FS44	(DAM_FS_44KHZ<<8)
#define DAM_CH0_CTRL_0_FSIN_FS48	(DAM_FS_48KHZ<<8)
#define DAM_CH0_CTRL_0_DATA_SYNC	(DATA_SYNC<<4)
#define DAM_CH0_CTRL_0_EN			(1<<0)

#define		GAIN		1

/*
DAM_CH0_CONV_0
*/
#define DAM_CH0_CONV_0_GAIN		GAIN<<0

/*
DAM_CH1_CTRL_0
*/

#define DAM_CH1_CTRL_0_DATA_SYNC		(DATA_SYNC<<4)
#define DAM_CH1_CTRL_0_EN				(1<<0)

/*
DAM_CH1_CONV_0
*/
#define DAM_CH1_CONV_0_GAIN		GAIN<<0

/*
*	Internal calls
*	FIXME: Move the needed call to a common location to be call
*	to call from audio_switch
*/
void dam_dump_registers(int ifc);
void dam_control_dam_enable(int ifc,int on);
void dam_control_cg_enable(int ifc,int on);
void dam_control_set_fsout(int ifc,int fsout);
void dam_ch0_control_enable(int ifc,int on);
void dam_ch0_control_set_fsin(int ifc,int fsin);
void dam_ch0_control_set_step(int ifc,int step);
void dam_ch0_control_set_datasync(int ifc,int datasync);
void dam_ch0_control_set_gain(int ifc,int gain);
void dam_ch1_control_enable(int ifc,int on);
void dam_ch1_control_set_datasync(int ifc,int datasync);
void dam_ch1_control_set_gain(int ifc,int gain);


/*static phys_addr_t dam_phy_base[NR_DAM_IFC] = {
	TEGRA_DAM0_BASE,
	TEGRA_DAM1_BASE,
	TEGRA_DAM2_BASE,
};
*/

static void *dam_base[NR_DAM_IFC] = {
	IO_ADDRESS(TEGRA_DAM0_BASE),
	IO_ADDRESS(TEGRA_DAM1_BASE),
	IO_ADDRESS(TEGRA_DAM2_BASE),
};

static inline void dam_writel(int ifc, u32 val, u32 reg)
{
	__raw_writel(val, dam_base[ifc] + reg);
	pr_info("dam write offset 0x%x: %08x\n", reg, val);
}

static inline u32 dam_readl(int ifc, u32 reg)
{
	u32 val = __raw_readl(dam_base[ifc] + reg);
	pr_info("dam read offset 0x%x: %08x\n", reg, val);
	return val;
}

void dam_dump_registers(int ifc)
{
	check_dam_ifc(ifc);

	pr_info("%s: \n",__func__);

	dam_readl(ifc, DAM_CTRL_0);
	dam_readl(ifc, DAM_CLIP_0);
	dam_readl(ifc, DAM_CLIP_THRESHOLD_0);
	dam_readl(ifc, DAM_AUDIOCIF_OUT_CTRL_0);
	dam_readl(ifc, DAM_CH0_CTRL_0);
	dam_readl(ifc, DAM_CH0_CONV_0);
	dam_readl(ifc, DAM_AUDIOCIF_CH0_CTRL_0);
	dam_readl(ifc, DAM_CH1_CTRL_0);
	dam_readl(ifc, DAM_CH1_CONV_0);
	dam_readl(ifc, DAM_AUDIOCIF_CH1_CTRL_0);
}

void dam_control_dam_enable(int ifc,int on)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CTRL_0);

	val &= ~DAM_CTRL_0_DAM_EN;
	val |= on ? DAM_CTRL_0_DAM_EN : 0;

	dam_writel(ifc, val, DAM_CTRL_0);
}

void dam_control_set_fsout(int ifc,int fsout)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CTRL_0);

	switch (fsout)
	{
		case DAM_FS_8KHZ:
			val |= DAM_CTRL_0_FSOUT_FS8;
			break;
		case DAM_FS_16KHZ:
			val |= DAM_CTRL_0_FSOUT_FS16;
			break;
		case DAM_FS_44KHZ:
			val |= DAM_CTRL_0_FSOUT_FS44;
			break;
		case DAM_FS_48KHZ:
			val |= DAM_CTRL_0_FSOUT_FS48;
			break;
		default:
			break;
	}

	dam_writel(ifc, val, DAM_CTRL_0);
}

void dam_ch0_control_enable(int ifc,int on)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CH0_CTRL_0);

	val &= ~DAM_CH0_CTRL_0_EN;
	val |= on ? DAM_CH0_CTRL_0_EN : 0;

	dam_writel(ifc, val, DAM_CH0_CTRL_0);
}

void dam_ch0_control_set_fsin(int ifc,int fsin)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CH0_CTRL_0);

	switch (fsin)
	{
		case DAM_FS_8KHZ:
			val |= DAM_CTRL_0_FSOUT_FS8;
			break;
		case DAM_FS_16KHZ:
			val |= DAM_CTRL_0_FSOUT_FS16;
			break;
		case DAM_FS_44KHZ:
			val |= DAM_CTRL_0_FSOUT_FS44;
			break;
		case DAM_FS_48KHZ:
			val |= DAM_CTRL_0_FSOUT_FS48;
			break;
		default:
		break;
	}

	dam_writel(ifc, val, DAM_CH0_CTRL_0);
}

void dam_ch0_control_set_step(int ifc,int step)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CH0_CTRL_0);

	val &= ~DAM_CH0_CTRL_0_STEP_MASK;
	val |= step << DAM_CH0_CTRL_0_STEP_SHIFT;

	dam_writel(ifc, val, DAM_CH0_CTRL_0);
}

void dam_ch0_control_set_datasync(int ifc,int datasync)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CH0_CTRL_0);

	val |= datasync<<DAM_CH0_CTRL_0_DATA_SYNC;

	dam_writel(ifc, val, DAM_CH0_CTRL_0);
}

void dam_ch0_control_set_gain(int ifc,int gain)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CH0_CONV_0);

	val |= gain<<DAM_CH0_CONV_0_GAIN;

	dam_writel(ifc, val, DAM_CH0_CONV_0);
}

void dam_ch1_control_enable(int ifc,int on)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CH1_CTRL_0);

	val &= ~DAM_CH1_CTRL_0_EN;
	val |= on ? DAM_CH1_CTRL_0_EN : 0;


	dam_writel(ifc, val, DAM_CH1_CTRL_0);
}

void dam_ch1_control_set_datasync(int ifc,int datasync)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CH1_CTRL_0);

	val |= datasync<<DAM_CH1_CTRL_0_DATA_SYNC;

	dam_writel(ifc, val, DAM_CH1_CTRL_0);
}

void dam_ch1_control_set_gain(int ifc,int gain)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CH1_CONV_0);

	val |= gain<<DAM_CH1_CONV_0_GAIN;

	dam_writel(ifc, val, DAM_CH1_CONV_0);
}

/*
u32 dam_get_acifChannelDirection(int ifc,int channelIndex)
{
   check_dam_ifc(ifc);

   return get_acifDirection(DAM_AUDIOCIF_CH_CTRL_0);
}

void dam_acifChannelConversion(int ifc,int channlConvType)
{
   check_dam_ifc(ifc);

   acifChannelConversion(DAM_AUDIOCIF_CH_CTRL_0_STEREO_CONV);

}

void dam_set_acifChannelFifoThreshold(int ifc,u32 FifoThreshold)
{
   check_dam_ifc(ifc);

   set_acifChannelFifoThreshold(DAM_AUDIOCIF_CH_CTRL_0_FIFO_THRESHOLD);
}

void dam_set_acifNumChannels(int ifc,int channel,int sourcetype)
{
   check_dam_ifc(ifc);

   set_acifNumChannels(DAM_AUDIOCIF_CH_CTRL_0_AUDIO_CHANNELS);
}

void dam_set_acifChannelBits(int ifc,int channelBits,int sourceType)
{
   check_dam_ifc(ifc);

   set_acifChannelBits(DAM_AUDIOCIF_CH_CTRL_0_AUDIO_BITS);
}

u32 dam_get_acifOutDirection(int ifc)
{
	return get_acifDirection(DAM_AUDIOCIF_OUT_CTRL_0);
}

void dam_acifOutChannelConversion(int ifc,int channlConvType)
{
   check_dam_ifc(ifc);

   acifChannelConversion(DAM_AUDIOCIF_OUT_CTRL_0_STEREO_CONV);
}

void dam_set_acifOutChannelFifoThreshold(int ifc,u32 FifoThreshold)
{
   check_dam_ifc(ifc);

   set_acifChannelFifoThreshold(DAM_AUDIOCIF_OUT_CTRL_0_FIFO_THRESHOLD);
}

void dam_set_acifOutNumChannels(int ifc,int channel,int sourcetype)
{
   check_dam_ifc(ifc);

   set_acifNumChannels(DAM_AUDIOCIF_OUT_CTRL_0_AUDIO_CHANNELS);
}

void dam_set_acifOutChannelBits(int ifc,int channelBits,int sourceType)
{
   check_dam_ifc(ifc);

   set_acifChannelBits(DAM_AUDIOCIF_OUT_CTRL_0_AUDIO_BITS);
}
*/
