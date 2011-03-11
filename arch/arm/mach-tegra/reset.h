/*
 * arch/arm/mach-tegra/reset.h
 *
 * Declarations for reset dispatcher.
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#ifndef __MACH_TEGRA_RESET_H
#define __MACH_TEGRA_RESET_H

#define TEGRA_RESET_MASK_PRESENT_PTR	0
#define TEGRA_RESET_MASK_ONLINE_PTR	1
#define TEGRA_RESET_MASK_INIT		2
#define TEGRA_RESET_MASK_LP1		3
#define TEGRA_RESET_MASK_LP2		4
#define TEGRA_RESET_STARTUP_SECONDARY	5
#define TEGRA_RESET_STARTUP_HOTPLUG	6
#define TEGRA_RESET_STARTUP_LP2		7
#define TEGRA_RESET_STARTUP_LP1		8
#define TEGRA_RESET_DATA_SIZE		9

#ifndef __ASSEMBLY__

#include <asm/localtimer.h>

extern unsigned long __tegra_cpu_reset_handler_data[TEGRA_RESET_DATA_SIZE];
extern unsigned long tegra_wfi_fail_count[CONFIG_NR_CPUS];
extern void __tegra_cpu_reset_handler_start(void);

#ifdef CONFIG_SMP
#define tegra_cpu_init_map (*(unsigned long *)(IO_ADDRESS(TEGRA_RESET_HANDLER_BASE + \
		((unsigned long)&__tegra_cpu_reset_handler_data[TEGRA_RESET_MASK_INIT] - \
		 (unsigned long)__tegra_cpu_reset_handler_start))))
#endif
#ifdef CONFIG_PM
#define tegra_cpu_lp1_map (*(unsigned long *)(IO_ADDRESS(TEGRA_RESET_HANDLER_BASE + \
		((unsigned long)&__tegra_cpu_reset_handler_data[TEGRA_RESET_MASK_LP1] - \
		 (unsigned long)__tegra_cpu_reset_handler_start))))
#endif

#define tegra_cpu_lp2_map (*(unsigned long *)(IO_ADDRESS(TEGRA_RESET_HANDLER_BASE + \
		((unsigned long)&__tegra_cpu_reset_handler_data[TEGRA_RESET_MASK_LP2] - \
		 (unsigned long)__tegra_cpu_reset_handler_start))))

extern spinlock_t lp2_map_lock;

static inline bool suspend_wfi_failed(void)
{
#if defined(CONFIG_HAVE_ARM_TWD) /* !!!FIXME!!! && defined(DEBUG) */
	u32 twd_load = readl(twd_base);
	unsigned int cpu = hard_smp_processor_id();

	/* Did the WFI fail? */
	if (twd_load == (0xBAD00000 | cpu))
		return true;
#endif
	return false;
}

void tegra_cpu_reset_handler_enable(void);
void tegra_cpu_reset_handler_flush(bool l1cache);
void __init tegra_cpu_reset_handler_init(void);

#endif

#endif
