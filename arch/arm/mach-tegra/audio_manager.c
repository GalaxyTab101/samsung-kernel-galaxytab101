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

struct audio_manager_context {
	struct clk *mclk;
	struct clk *pmc_clk;
	int mclk_refcnt;
	int mclk_rate;
	int mclk_parent;
};

struct audio_manager_context *aud_manager;

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

MODULE_LICENSE("GPL");
