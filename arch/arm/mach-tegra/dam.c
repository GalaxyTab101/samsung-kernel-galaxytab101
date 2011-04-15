/*
 * arch/arm/mach-tegra/dam.c
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
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
#include <linux/clk.h>
#include <linux/slab.h>

#include "clock.h"
#include <asm/io.h>
#include <mach/iomap.h>
#include <mach/audio.h>
#include <mach/audio_switch.h>

#define NR_DAM_IFC	3

#define check_dam_ifc(n, ...) if ((n) > NR_DAM_IFC) {			\
	pr_err("%s: invalid dam interface %d\n", __func__, (n));	\
	return __VA_ARGS__;						\
}

/* Offsets from TEGRA_DAM1_BASE, TEGRA_DAM2_BASE and TEGRA_DAM3_BASE */
#define DAM_CTRL_0			0
#define DAM_CLIP_0			4
#define DAM_CLIP_THRESHOLD_0		8
#define DAM_AUDIOCIF_OUT_CTRL_0		0x0C
#define DAM_CH0_CTRL_0			0x10
#define DAM_CH0_CONV_0			0x14
#define DAM_AUDIOCIF_CH0_CTRL_0		0x1C
#define DAM_CH1_CTRL_0			0x20
#define DAM_CH1_CONV_0			0x24
#define DAM_AUDIOCIF_CH1_CTRL_0		0x2C

#define DAM_CTRL_REGINDEX 	(DAM_AUDIOCIF_CH1_CTRL_0 >> 2)
#define DAM_CTRL_RSVD_6		6
#define DAM_CTRL_RSVD_10	10

#define DAM_NUM_INPUT_CHANNELS		2
#define DAM_FS_8KHZ  			0
#define DAM_FS_16KHZ 			1
#define DAM_FS_44KHZ 			2
#define DAM_FS_48KHZ 			3

/* DAM_CTRL_0 */

#define DAM_CTRL_0_SOFT_RESET_ENABLE	(1 << 31)

#define DAM_CTRL_0_FSOUT_SHIFT	4
#define DAM_CTRL_0_FSOUT_MASK	(0xf << DAM_CTRL_0_FSOUT_SHIFT)
#define DAM_CTRL_0_FSOUT_FS8	(DAM_FS_8KHZ << DAM_CTRL_0_FSOUT_SHIFT)
#define DAM_CTRL_0_FSOUT_FS16	(DAM_FS_16KHZ << DAM_CTRL_0_FSOUT_SHIFT)
#define DAM_CTRL_0_FSOUT_FS44	(DAM_FS_44KHZ << DAM_CTRL_0_FSOUT_SHIFT)
#define DAM_CTRL_0_FSOUT_FS48	(DAM_FS_48KHZ << DAM_CTRL_0_FSOUT_SHIFT)
#define DAM_CTRL_0_CG_EN	(1 << 1)
#define DAM_CTRL_0_DAM_EN	(1 << 0)

/* DAM_CLIP_0 */

#define DAM_CLIP_0_COUNTER_ENABLE	(1 << 31)
#define DAM_CLIP_0_COUNT_MASK		0x7fffffff

/* DAM_CLIP_THRESHOLD_0 */
#define DAM_CLIP_THRESHOLD_0_VALUE_SHIFT	8
#define DAM_CLIP_THRESHOLD_0_VALUE_MASK		\
		(0x7fffff << DAM_CLIP_THRESHOLD_0_VALUE_SHIFT)
#define DAM_CLIP_THRESHOLD_0_VALUE		(1 << 31)
#define DAM_CLIP_THRESHOLD_0_COUNT_SHIFT	0


#define STEP_RESET		1
#define DAM_DATA_SYNC		1
#define DAM_DATA_SYNC_SHIFT	4
#define DAM_GAIN		1
#define DAM_GAIN_SHIFT		0

/* DAM_CH0_CTRL_0 */
#define DAM_CH0_CTRL_0_FSIN_SHIFT	8
#define DAM_CH0_CTRL_0_STEP_SHIFT	16
#define DAM_CH0_CTRL_0_STEP_MASK	(0xffff << 16)
#define DAM_CH0_CTRL_0_STEP_RESET	(STEP_RESET << 16)
#define DAM_CH0_CTRL_0_FSIN_MASK	(0xf << 8)
#define DAM_CH0_CTRL_0_FSIN_FS8		(DAM_FS_8KHZ << 8)
#define DAM_CH0_CTRL_0_FSIN_FS16	(DAM_FS_16KHZ << 8)
#define DAM_CH0_CTRL_0_FSIN_FS44	(DAM_FS_44KHZ << 8)
#define DAM_CH0_CTRL_0_FSIN_FS48	(DAM_FS_48KHZ << 8)
#define DAM_CH0_CTRL_0_DATA_SYNC_MASK	(0xf << DAM_DATA_SYNC_SHIFT)
#define DAM_CH0_CTRL_0_DATA_SYNC	(DAM_DATA_SYNC << DAM_DATA_SYNC_SHIFT)
#define DAM_CH0_CTRL_0_EN		(1 << 0)


/* DAM_CH0_CONV_0 */
#define DAM_CH0_CONV_0_GAIN		(DAM_GAIN << DAM_GAIN_SHIFT)

/* DAM_CH1_CTRL_0 */
#define DAM_CH1_CTRL_0_DATA_SYNC_MASK	(0xf << DAM_DATA_SYNC_SHIFT)
#define DAM_CH1_CTRL_0_DATA_SYNC	(DAM_DATA_SYNC << DAM_DATA_SYNC_SHIFT)
#define DAM_CH1_CTRL_0_EN		(1 << 0)

/* DAM_CH1_CONV_0 */
#define DAM_CH1_CONV_0_GAIN		(DAM_GAIN << DAM_GAIN_SHIFT)

#define DAM_OUT_CHANNEL		0
#define DAM_IN_CHANNEL_0	1
#define DAM_IN_CHANNEL_1	2

#define  ENABLE_DAM_DEBUG_PRINT	0

#if ENABLE_DAM_DEBUG_PRINT
#define DAM_DEBUG_PRINT(fmt, arg...) printk(fmt, ## arg)
#else
#define DAM_DEBUG_PRINT(fmt, arg...) do {} while (0)
#endif

/* FIXME: move this control to audio_manager later if needed */
struct dam_context {
	int		outsamplerate;
	bool		ch_inuse[DAM_NUM_INPUT_CHANNELS];
	int		ch_insamplerate[DAM_NUM_INPUT_CHANNELS];
	int		ctrlreg_cache[DAM_CTRL_REGINDEX];
	struct clk 	*dam_clk;
	int  		clk_refcnt;
};

struct dam_context dam_cont_info[NR_DAM_IFC];

static char* damclk_info[NR_DAM_IFC] = {
	"dam0",
	"dam1",
	"dam2"
};

struct dam_module_context {
	int refcnt;
	bool inuse[NR_DAM_IFC];
};

static struct dam_module_context *dam_info = NULL;

static void *dam_base[NR_DAM_IFC] = {
	IO_ADDRESS(TEGRA_DAM0_BASE),
	IO_ADDRESS(TEGRA_DAM1_BASE),
	IO_ADDRESS(TEGRA_DAM2_BASE),
};

/* Internal calls */
void dam_dump_registers(int ifc);
void dam_ch0_enable(int ifc,int on);
void dam_ch1_enable(int ifc,int on);
void dam_set_input_samplerate(int ifc,int fsin);
void dam_set_output_samplerate(int ifc,int fsout);
void dam_ch0_set_step(int ifc,int step);
void dam_ch0_set_datasync(int ifc,int datasync);
void dam_ch0_set_gain(int ifc,int gain);
void dam_ch1_set_datasync(int ifc,int datasync);
void dam_ch1_set_gain(int ifc,int gain);

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

void dam_enable(int ifc, int on, int chtype)
{
	u32 val;
	struct dam_context *ch = &dam_cont_info[ifc];

	check_dam_ifc(ifc);

	if (chtype == dam_ch_in0) {
		dam_ch0_enable(ifc, on);
		ch->ch_inuse[dam_ch_in0] = (on)? true : false;
	} else if (chtype == dam_ch_in1) {
		dam_ch1_enable(ifc, on);
		ch->ch_inuse[dam_ch_in1] = (on)? true : false;
	}

	val = dam_readl(ifc, DAM_CTRL_0);

	val &= ~DAM_CTRL_0_DAM_EN;
	val |= on ? DAM_CTRL_0_DAM_EN : 0;

	dam_writel(ifc, val, DAM_CTRL_0);
}

void dam_enable_clip_counter(int ifc, int on)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CLIP_0);

	val &= ~ DAM_CLIP_0_COUNTER_ENABLE;
	val |= on ?  DAM_CLIP_0_COUNTER_ENABLE : 0;

	dam_writel(ifc, val, DAM_CLIP_0);
}

void dam_set_samplerate(int ifc, int chtype, int samplerate)
{
	struct dam_context *ch = &dam_cont_info[ifc];

	switch (chtype) {
	case dam_ch_in0:
		dam_set_input_samplerate(ifc, samplerate);
		ch->ch_insamplerate[dam_ch_in0] = samplerate;
		break;
	case dam_ch_in1:
		ch->ch_insamplerate[dam_ch_in1] = samplerate;
		break;
	case dam_ch_out:
		dam_set_output_samplerate(ifc, samplerate);
		ch->outsamplerate = samplerate;
		break;
	default:
		break;
	}
}

void dam_set_output_samplerate(int ifc,int fsout)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CTRL_0);
	val &=~DAM_CTRL_0_FSOUT_MASK;

	switch (fsout){

	case AUDIO_SAMPLERATE_8000:
		val |= DAM_CTRL_0_FSOUT_FS8;
		break;
	case AUDIO_SAMPLERATE_16000:
		val |= DAM_CTRL_0_FSOUT_FS16;
		break;
	case AUDIO_SAMPLERATE_44100:
		val |= DAM_CTRL_0_FSOUT_FS44;
		break;
	case AUDIO_SAMPLERATE_48000:
		val |= DAM_CTRL_0_FSOUT_FS48;
		break;
	default:
		break;
	}

	dam_writel(ifc, val, DAM_CTRL_0);
}

void dam_ch0_enable(int ifc,int on)
{
	u32 val;

	val = dam_readl(ifc, DAM_CH0_CTRL_0);

	val &= ~DAM_CH0_CTRL_0_EN;
	val |= on ? DAM_CH0_CTRL_0_EN : 0;

	dam_writel(ifc, val, DAM_CH0_CTRL_0);
}

void dam_set_input_samplerate(int ifc,int fsin)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CH0_CTRL_0);
	val &=~DAM_CH0_CTRL_0_FSIN_MASK;

	switch (fsin) {

	case AUDIO_SAMPLERATE_8000:
		val |= DAM_CH0_CTRL_0_FSIN_FS8;
		break;
	case AUDIO_SAMPLERATE_16000:
		val |= DAM_CH0_CTRL_0_FSIN_FS16;
		break;
	case AUDIO_SAMPLERATE_44100:
		val |= DAM_CH0_CTRL_0_FSIN_FS44;
		break;
	case AUDIO_SAMPLERATE_48000:
		val |= DAM_CH0_CTRL_0_FSIN_FS48;
		break;
	default:
		break;
	}

	dam_writel(ifc, val, DAM_CH0_CTRL_0);
}

void dam_ch0_set_step(int ifc,int step)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CH0_CTRL_0);

	val &= ~DAM_CH0_CTRL_0_STEP_MASK;
	val |= step << DAM_CH0_CTRL_0_STEP_SHIFT;

	dam_writel(ifc, val, DAM_CH0_CTRL_0);
}

void dam_ch0_set_datasync(int ifc,int datasync)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CH0_CTRL_0);
	val &= ~DAM_CH0_CTRL_0_DATA_SYNC_MASK;

	val |= datasync << DAM_DATA_SYNC_SHIFT;

	dam_writel(ifc, val, DAM_CH0_CTRL_0);
}

void dam_ch0_control_set_gain(int ifc,int gain)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CH0_CONV_0);

	val |= gain << DAM_GAIN_SHIFT;

	dam_writel(ifc, val, DAM_CH0_CONV_0);
}

void dam_ch1_enable(int ifc,int on)
{
	u32 val;

	val = dam_readl(ifc, DAM_CH1_CTRL_0);

	val &= ~DAM_CH1_CTRL_0_EN;
	val |= on ? DAM_CH1_CTRL_0_EN : 0;

	dam_writel(ifc, val, DAM_CH1_CTRL_0);
}

void dam_ch1_set_datasync(int ifc,int datasync)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CH1_CTRL_0);
	val &= ~DAM_CH1_CTRL_0_DATA_SYNC_MASK;

	val |= datasync << DAM_DATA_SYNC_SHIFT;

	dam_writel(ifc, val, DAM_CH1_CTRL_0);
}

void dam_ch1_set_gain(int ifc,int gain)
{
	u32 val;

	check_dam_ifc(ifc);

	val = dam_readl(ifc, DAM_CH1_CONV_0);

	val |= gain << DAM_GAIN_SHIFT;

	dam_writel(ifc, val, DAM_CH1_CONV_0);
}

void dam_save_ctrl_registers(int ifc)
{
	int i = 0;
	struct dam_context *ch;
	ch = &dam_cont_info[ifc];

	for (i = 0; i <= DAM_CTRL_REGINDEX; i++) {
		if ((i == DAM_CTRL_RSVD_6) || (i == DAM_CTRL_RSVD_10))
			continue;

		ch->ctrlreg_cache[i] = dam_readl(ifc, (i << 2));
	}
}

void dam_restore_ctrl_registers(int ifc)
{
	int i = 0;
	struct dam_context *ch;
	ch = &dam_cont_info[ifc];

	for (i = 0; i <= DAM_CTRL_REGINDEX; i++) {
		if ((i == DAM_CTRL_RSVD_6) || (i == DAM_CTRL_RSVD_10))
			continue;

		dam_writel(ifc, ch->ctrlreg_cache[i], (i << 2));
	}
}

int dam_suspend(int ifc)
{
	if (dam_info->inuse[ifc] == true) {
		dam_save_ctrl_registers(ifc);
	}

	dam_disable_clock(ifc);

	return 0;
}

int dam_resume(int ifc)
{

	dam_enable_clock(ifc);

	if (dam_info->inuse[ifc] == true) {
		dam_restore_ctrl_registers(ifc);
	}

	return 0;
}

int dam_set_clock_rate(int rate)
{
	/* FIXME: to complete */
	return 0;
}

int dam_set_clock_parent(int ifc, int parent)
{
	/* FIXME ; set parent based on need */
	struct dam_context *ch;
	struct clk *mclk_source = clk_get_sys(NULL, "pll_a_out0");

	ch =  &dam_cont_info[ifc];
	clk_set_parent(ch->dam_clk, mclk_source);
	return 0;
}

void dam_disable_clock(int ifc)
{
	struct dam_context *ch;

	ch =  &dam_cont_info[ifc];

	if (ch->clk_refcnt > 0) {
		ch->clk_refcnt--;
		if ((ch->clk_refcnt == 0)  &&
			(ch->dam_clk)) {
			clk_disable(ch->dam_clk);
		}
	}

	DAM_DEBUG_PRINT(" %s clk cnt %d \n",__func__,  ch->clk_refcnt);
}

int dam_enable_clock(int ifc)
{
	int err = 0;
	struct dam_context *ch;

	ch =  &dam_cont_info[ifc];

	if (!ch->clk_refcnt) {
		if (clk_enable(ch->dam_clk)) {
			err = PTR_ERR(ch->dam_clk);
			goto fail_dam_clock;
		}
	}

	ch->clk_refcnt++;

	DAM_DEBUG_PRINT(" %s clk cnt %d \n",__func__,  ch->clk_refcnt);
	return err;

fail_dam_clock:

	dam_disable_clock(ifc);
	return err;
}

int dam_set_acif(int ifc, int chtype, struct audio_cif *cifInfo)
{
	struct dam_context *ch;
	unsigned int reg_addr = 0;
	ch =  &dam_cont_info[ifc];

	switch (chtype) {
	case dam_ch_out:
		reg_addr = (unsigned int)dam_base + DAM_AUDIOCIF_OUT_CTRL_0;
		break;
	case dam_ch_in0:
		reg_addr = (unsigned int)dam_base + DAM_AUDIOCIF_CH0_CTRL_0;
		break;
	case dam_ch_in1:
		reg_addr = (unsigned int)dam_base + DAM_AUDIOCIF_CH1_CTRL_0;
		break;
	default:
		break;
	}

	if (reg_addr) {
		audio_switch_set_acif(reg_addr, cifInfo);
	}

	return 0;
}

int dam_get_controller(void)
{
	int i = 0;

	if (!dam_info)
		return -ENOENT;

	for (i = 0; i < NR_DAM_IFC; i++) {
		if (dam_info->inuse[i] == false) {
			dam_info->inuse[i] = true;
			return i;
		}
	}
	return -ENOENT;
}

int dam_free_controller(int ifc)
{
	if (!dam_info)
		return -ENOENT;

	/* FIXME: make sure the caller is done with the channels */
	dam_info->inuse[ifc] = false;
	return 0;
}

int dam_open(void)
{
	int err = 0, i = 0;

	DAM_DEBUG_PRINT("%s ++ \n", __func__);

	if (!dam_info) {
		struct dam_context *ch;

		dam_info =
		kzalloc(sizeof(struct dam_module_context), GFP_KERNEL);

		if (!dam_info)
			return -ENOMEM;

		memset(dam_cont_info, 0, sizeof(dam_cont_info));

		for (i = 0; i < NR_DAM_IFC; i++) {
			ch = &dam_cont_info[i];
			ch->dam_clk = tegra_get_clock_by_name(damclk_info[i]);
			if (!ch->dam_clk) {
				err = -ENOENT;
				goto fail_dam_open;
			}
			dam_set_clock_parent(i, 0);
		}
		dam_info->refcnt++;
	} else {
		dam_info->refcnt++;
	}

	DAM_DEBUG_PRINT(" %s -- \n", __func__);

	return 0;

fail_dam_open:

	dam_close();
	return err;
}

int dam_close(void)
{
	struct dam_context *ch;
	int i = 0;

	DAM_DEBUG_PRINT("%s \n", __func__);
	if (dam_info) {
		dam_info->refcnt--;

		if (dam_info->refcnt == 0) {
			for (i = 0; i < NR_DAM_IFC; i++) {
				ch = &dam_cont_info[i];
				dam_disable_clock(i);

				if (ch->dam_clk)
					clk_put(ch->dam_clk);
			}
			kfree(dam_info);
			dam_info = NULL;
		}
	}

	return 0;
}
