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

#include <mach/iomap.h>
#include <mach/pinmux.h>
#include <mach/tegra_das.h>

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
	return 0;
}
EXPORT_SYMBOL_GPL(tegra_das_power_mode);

int tegra_das_open(void)
{
	return 0;
}
EXPORT_SYMBOL_GPL(tegra_das_open);

int tegra_das_close(void)
{
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
MODULE_LICENSE("GPL");
