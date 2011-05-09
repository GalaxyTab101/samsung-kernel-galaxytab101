/*
 * linux/arch/arm/mach-tegra/audio_manager.c
 *
 * Audio Manager for tegra soc
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation
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


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/err.h>

#include "clock.h"
#include <mach/iomap.h>
#include <mach/pinmux.h>
#include <mach/tegra_das.h>
#include <mach/tegra_i2s.h>
#include <mach/spdif.h>
#include <mach/audio_manager.h>


static struct am_dev_fns init_am_dev_fns[] = {
[AUDIO_I2S_DEVICE] = {
	.aud_dev_suspend = i2s_suspend,
	.aud_dev_resume = i2s_resume,
	.aud_dev_set_stream_state = i2s_fifo_enable,
	.aud_dev_get_dma_requestor = i2s_get_dma_requestor,
	.aud_dev_free_dma_requestor = i2s_free_dma_requestor,
	.aud_dev_get_fifo_phy_base = i2s_get_fifo_phy_base,
	.aud_dev_set_fifo_attention = i2s_set_fifo_attention,
	.aud_dev_get_status = i2s_get_status,
	.aud_dev_clock_disable = i2s_clock_disable,
	.aud_dev_clock_enable = i2s_clock_enable,
	.aud_dev_clock_set_parent = i2s_clock_set_parent,
	.aud_dev_clock_set_rate = i2s_clock_set_rate,
	.aud_dev_deinit = i2s_close,
	},

[AUDIO_SPDIF_DEVICE] = {
	.aud_dev_suspend = spdif_suspend,
	.aud_dev_resume = spdif_resume,
	.aud_dev_set_stream_state = spdif_fifo_enable,
	.aud_dev_get_dma_requestor = spdif_get_dma_requestor,
	.aud_dev_free_dma_requestor = spdif_free_dma_requestor,
	.aud_dev_get_fifo_phy_base = spdif_get_fifo_phy_base,
	.aud_dev_set_fifo_attention = spdif_set_fifo_attention,
	.aud_dev_get_status = spdif_get_status,
	.aud_dev_clock_disable = spdif_clock_disable,
	.aud_dev_clock_enable = spdif_clock_enable,
	.aud_dev_clock_set_parent = spdif_clock_set_parent,
	.aud_dev_clock_set_rate = spdif_clock_set_rate,
	.aud_dev_deinit = spdif_close,
	},

};

#ifdef CONFIG_ARCH_TEGRA_2x_SOC

int am_suspend(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_suspend(devinfo->dev_id);
}

int am_resume(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_resume(devinfo->dev_id);
}

int am_set_stream_state(aud_dev_info* devinfo, bool enable)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_set_stream_state(
				devinfo->dev_id,
				devinfo->fifo_mode,
				((enable)? 1 : 0));
}

int am_get_dma_requestor(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_get_dma_requestor(
				devinfo->dev_id,
				devinfo->fifo_mode);
}

int am_free_dma_requestor(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_free_dma_requestor(
				devinfo->dev_id,
				devinfo->fifo_mode);
}

phys_addr_t am_get_fifo_phy_base(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_get_fifo_phy_base(
				devinfo->dev_id,
				devinfo->fifo_mode);
}

int am_set_fifo_attention(aud_dev_info* devinfo, int buffersize)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_set_fifo_attention(
				devinfo->dev_id,
				buffersize,
				devinfo->fifo_mode);
}

u32 am_get_status(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_get_status(
				devinfo->dev_id,
				devinfo->fifo_mode);
}

int am_clock_disable(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_clock_disable(
				devinfo->dev_id,
				devinfo->fifo_mode);
}

int am_clock_enable(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_clock_enable(
				devinfo->dev_id,
				devinfo->fifo_mode);
}

int am_clock_set_parent(aud_dev_info* devinfo, int parent)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_clock_set_parent(
				devinfo->dev_id,
				devinfo->fifo_mode,
				parent);
}

int am_clock_set_rate(aud_dev_info* devinfo, int rate)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_clock_set_rate(
				devinfo->dev_id,
				devinfo->fifo_mode,
				rate);
}

int am_device_deinit(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_deinit(devinfo->dev_id);
}

int am_set_stream_format(aud_dev_info* devinfo, am_stream_format_info *format)
{
	if (devinfo->dev_type == AUDIO_I2S_DEVICE) {
		i2s_set_bit_size(devinfo->dev_id, format->bitsize);
		i2s_set_samplerate(devinfo->dev_id, format->samplerate);
		i2s_set_channels(devinfo->dev_id, format->channels);
		i2s_set_fifo_attention(devinfo->dev_id,
			devinfo->fifo_mode, format->buffersize);

	} else if (devinfo->dev_type == AUDIO_SPDIF_DEVICE) {
		spdif_set_bit_mode(devinfo->dev_id, format->bitsize);
		/* fixme - move to appropriate locn later */
		spdif_set_fifo_packed(devinfo->dev_id, 1);

		spdif_set_sample_rate(devinfo->dev_id,
			devinfo->fifo_mode, format->samplerate);
		spdif_set_fifo_attention(devinfo->dev_id,
			devinfo->fifo_mode, format->buffersize);
	}
	return 0;
}

int am_set_device_format(aud_dev_info* devinfo, am_dev_format_info *format)
{
	if (devinfo->dev_type == AUDIO_I2S_DEVICE) {
		i2s_set_loopback(devinfo->dev_id, format->loopmode);
		i2s_set_master(devinfo->dev_id, format->mastermode);
		i2s_set_bit_format(devinfo->dev_id, format->audiomode);
		i2s_set_left_right_control_polarity(
				devinfo->dev_id,
				format->polarity);

	} else if (devinfo->dev_type == AUDIO_SPDIF_DEVICE) {

	}
	return 0;
}

int am_device_init(aud_dev_info* devinfo, void *dev_fmt, void  *strm_fmt)
{
	am_stream_format_info  *sfmt = (am_stream_format_info*)strm_fmt;
	am_dev_format_info *dfmt = (am_dev_format_info*)dev_fmt;

	if (devinfo->dev_type == AUDIO_I2S_DEVICE) {
		struct tegra_i2s_property i2sprop;


		memset(&i2sprop, 0, sizeof(i2sprop));

		if (sfmt) {
			i2sprop.bit_size = sfmt->bitsize;
			i2sprop.sample_rate = sfmt->samplerate;
		}

		if (dfmt) {
			i2sprop.master_mode = dfmt->mastermode;
			i2sprop.audio_mode = dfmt->audiomode;
			i2sprop.clk_rate = dfmt->clkrate;
			i2sprop.fifo_fmt = dfmt->fifofmt;
		}

		return i2s_init(devinfo->dev_id, &i2sprop);

	} else if (devinfo->dev_type == AUDIO_SPDIF_DEVICE) {

		struct tegra_spdif_property spdifprop;
		memset(&spdifprop, 0, sizeof(spdifprop));

		if (dfmt) {
			spdifprop.clk_rate = dfmt->clkrate;

			return spdif_init(
				devinfo->base,
				devinfo->phy_base,
				devinfo->fifo_mode,
				&spdifprop);
		}
	}
	return 0;
}

#else

struct audio_manager_context {
	struct clk *mclk;
	struct clk *pmc_clk;
	int mclk_refcnt;
	int mclk_rate;
	int mclk_parent;
};

struct audio_manager_context *aud_manager;

int am_suspend(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_suspend(devinfo->dev_id);
}

int am_resume(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_resume(devinfo->dev_id);
}

int am_set_stream_state(aud_dev_info* devinfo, bool enable)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_set_stream_state(
				devinfo->dev_id,
				devinfo->fifo_mode,
				((enable)? 1 : 0));
}

int am_get_dma_requestor(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_get_dma_requestor(
				devinfo->dev_id,
				devinfo->fifo_mode);
}

int am_free_dma_requestor(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_free_dma_requestor(
				devinfo->dev_id,
				devinfo->fifo_mode);
}

phys_addr_t am_get_fifo_phy_base(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_get_fifo_phy_base(
				devinfo->dev_id,
				devinfo->fifo_mode);
}

int am_set_fifo_attention(aud_dev_info* devinfo, int buffersize)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_set_fifo_attention(
				devinfo->dev_id,
				buffersize,
				devinfo->fifo_mode);
}

u32 am_get_status(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_get_status(
				devinfo->dev_id,
				devinfo->fifo_mode);
}

int am_clock_disable(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_clock_disable(
				devinfo->dev_id,
				devinfo->fifo_mode);
}

int am_clock_enable(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_clock_enable(
				devinfo->dev_id,
				devinfo->fifo_mode);
}

int am_clock_set_parent(aud_dev_info* devinfo, int parent)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_clock_set_parent(
				devinfo->dev_id,
				devinfo->fifo_mode,
				parent);
}

int am_clock_set_rate(aud_dev_info* devinfo, int rate)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_clock_set_rate(
				devinfo->dev_id,
				devinfo->fifo_mode,
				rate);
}

int am_device_deinit(aud_dev_info* devinfo)
{
	struct am_dev_fns* am_fn = &init_am_dev_fns[devinfo->dev_type];
	return am_fn->aud_dev_deinit(devinfo->dev_id);
}

int am_set_stream_format(aud_dev_info* devinfo, am_stream_format_info *format)
{
	if (devinfo->dev_type == AUDIO_I2S_DEVICE) {
		i2s_set_bit_size(devinfo->dev_id, format->bitsize);
		i2s_set_samplerate(devinfo->dev_id, format->samplerate);
		i2s_set_channels(devinfo->dev_id, format->channels);
		i2s_set_fifo_attention(devinfo->dev_id,
			devinfo->fifo_mode, format->buffersize);

	} else if (devinfo->dev_type == AUDIO_SPDIF_DEVICE) {
		spdif_set_bit_mode(devinfo->dev_id, format->bitsize);
		spdif_set_sample_rate(
			devinfo->dev_id,
			devinfo->fifo_mode,
			format->samplerate);
		spdif_set_fifo_attention(devinfo->dev_id,
			devinfo->fifo_mode, format->buffersize);
	}
	return 0;
}

int am_set_device_format(aud_dev_info* devinfo, am_dev_format_info *format)
{
	if (devinfo->dev_type == AUDIO_I2S_DEVICE) {
		i2s_set_loopback(devinfo->dev_id, format->loopmode);
		i2s_set_master(devinfo->dev_id, format->mastermode);
		i2s_set_bit_format(devinfo->dev_id, format->audiomode);
		i2s_set_left_right_control_polarity(
				devinfo->dev_id,
				format->polarity);

	} else if (devinfo->dev_type == AUDIO_SPDIF_DEVICE) {

	}
	return 0;
}

int am_device_init(aud_dev_info* devinfo, void *dev_fmt, void  *strm_fmt)
{
	am_stream_format_info  *sfmt = (am_stream_format_info*)strm_fmt;
	am_dev_format_info *dfmt = (am_dev_format_info*)dev_fmt;

	if (devinfo->dev_type == AUDIO_I2S_DEVICE) {

		struct tegra_i2s_property i2sprop;
		memset(&i2sprop, 0, sizeof(i2sprop));

		if (sfmt) {
			i2sprop.bit_size = sfmt->bitsize;
			i2sprop.sample_rate = sfmt->samplerate;
		}

		if (dfmt) {
			i2sprop.master_mode = dfmt->mastermode;
			i2sprop.audio_mode = dfmt->audiomode;
			i2sprop.clk_rate = dfmt->clkrate;
			i2sprop.fifo_fmt = dfmt->fifofmt;
		}

		return i2s_init(devinfo->dev_id, &i2sprop);

	} else if (devinfo->dev_type == AUDIO_SPDIF_DEVICE) {

		struct tegra_spdif_property spdifprop;
		memset(&spdifprop, 0, sizeof(spdifprop));

		if (dfmt) {
			spdifprop.clk_rate = dfmt->clkrate;

			return spdif_init(
				devinfo->base,
				devinfo->phy_base,
				devinfo->fifo_mode,
				&spdifprop);
		}
	}

	return 0;
}

int tegra_das_set_connection(enum tegra_das_port_con_id new_con_id)
{
	return 0;
}
EXPORT_SYMBOL_GPL(tegra_das_set_connection);

int tegra_das_get_connection(void)
{
	return 0;
}
EXPORT_SYMBOL_GPL(tegra_das_get_connection);

/* if is_normal is true then power mode is normal else tristated */
int tegra_das_power_mode(bool is_normal)
{
	int err = 0;
	/*
	if (is_normal == true) {
		err = tegra_das_enable_mclk();
	} else {
		err = tegra_das_disable_mclk();
	}*/

	return err;
}
EXPORT_SYMBOL_GPL(tegra_das_power_mode);

int tegra_das_open(void)
{
	int err = 0;

	aud_manager = kzalloc(sizeof(struct audio_manager_context), GFP_KERNEL);
	if (!aud_manager)
		return -ENOMEM;

	aud_manager->mclk = tegra_get_clock_by_name("extern1");

	if (!aud_manager->mclk) {
		pr_err(" err in getting mclk \n");
		err = -ENODEV;
		goto fail_clock;
	}

	aud_manager->pmc_clk  = clk_get_sys("clk_out_1", "extern1");
	if (IS_ERR_OR_NULL(aud_manager->pmc_clk))
	{
		pr_err("%s can't get pmc clock\n", __func__);
		err = -ENODEV;
		aud_manager->pmc_clk = 0;
		goto fail_clock;
	}

	return err;

fail_clock:

	tegra_das_close();
	return err;
}
EXPORT_SYMBOL_GPL(tegra_das_open);

int tegra_das_close(void)
{
	if (aud_manager->mclk)
		clk_put(aud_manager->mclk);

	if (aud_manager->pmc_clk)
		clk_put(aud_manager->pmc_clk);

	kfree(aud_manager);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra_das_close);

void tegra_das_get_all_regs(struct das_regs_cache* regs)
{

}
EXPORT_SYMBOL_GPL(tegra_das_get_all_regs);

void tegra_das_set_all_regs(struct das_regs_cache* regs)
{

}
EXPORT_SYMBOL_GPL(tegra_das_set_all_regs);

int tegra_das_set_mclk_parent(int parent)
{
	/* FIXME ; set parent based on need */
	struct clk *mclk_source = clk_get_sys(NULL, "pll_a_out0");

	clk_set_parent(aud_manager->mclk, mclk_source);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra_das_set_mclk_parent);

int tegra_das_enable_mclk(void)
{
	int err = 0;

	if (!aud_manager->mclk_refcnt) {
		if (aud_manager->mclk && aud_manager->pmc_clk) {
			tegra_das_set_mclk_parent(0);
			if (clk_enable(aud_manager->mclk)) {
				err = PTR_ERR(aud_manager->mclk);
				return err;
			}

			if (clk_enable(aud_manager->pmc_clk)) {
				clk_disable(aud_manager->mclk);
				err = PTR_ERR(aud_manager->pmc_clk);
				return err;
			}
		}
	}

	aud_manager->mclk_refcnt++;
	return err;
}
EXPORT_SYMBOL_GPL(tegra_das_enable_mclk);

int tegra_das_disable_mclk(void)
{
	int err = 0;

	if (aud_manager->mclk_refcnt > 0) {
		aud_manager->mclk_refcnt--;
		if (aud_manager->mclk_refcnt == 0) {
			if (aud_manager->mclk)
				clk_disable(aud_manager->mclk);

			if (aud_manager->pmc_clk)
				clk_disable(aud_manager->pmc_clk);
		}
	}
	return err;
}
EXPORT_SYMBOL_GPL(tegra_das_disable_mclk);

int tegra_das_set_mclk_rate(int rate)
{
	/* FIXME: change the clock after disabling it if needed */
	aud_manager->mclk_rate = rate;
	clk_set_rate(aud_manager->mclk, rate);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra_das_set_mclk_rate);

int tegra_das_get_mclk_rate(void)
{
	return clk_get_rate(aud_manager->mclk);
}
EXPORT_SYMBOL_GPL(tegra_das_get_mclk_rate);

MODULE_LICENSE("GPL");
#endif
