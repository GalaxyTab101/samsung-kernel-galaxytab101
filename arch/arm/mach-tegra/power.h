/*
 * arch/arm/mach-tegra/power.h
 *
 * Declarations for power state transition code
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#ifndef __MACH_TEGRA_POWER_H
#define __MACH_TEGRA_POWER_H

#include <asm/page.h>

#define TEGRA_POWER_PWRREQ_POLARITY	0x1   /* core power request polarity */
#define TEGRA_POWER_PWRREQ_OE		0x2   /* core power request enable */
#define TEGRA_POWER_SYSCLK_POLARITY	0x4   /* sys clk polarity */
#define TEGRA_POWER_SYSCLK_OE		0x8   /* system clock enable */
#define TEGRA_POWER_PWRGATE_DIS		0x10  /* power gate disabled */
#define TEGRA_POWER_EFFECT_LP0		0x40  /* enter LP0 when CPU pwr gated */
#define TEGRA_POWER_CPU_PWRREQ_POLARITY 0x80  /* CPU power request polarity */
#define TEGRA_POWER_CPU_PWRREQ_OE	0x100 /* CPU power request enable */
#define TEGRA_POWER_PMC_SHIFT		8
#define TEGRA_POWER_PMC_MASK		0x1ff
#define TEGRA_POWER_SDRAM_SELFREFRESH	0x400 /* SDRAM is in self-refresh */

#define TEGRA_POWER_CLUSTER_G		0x1000	/* G CPU */
#define TEGRA_POWER_CLUSTER_LP		0x2000	/* LP CPU */
#define TEGRA_POWER_CLUSTER_MASK	0x3000
#define TEGRA_POWER_CLUSTER_IMMEDIATE	0x4000	/* Immediate wake */
#define TEGRA_POWER_CLUSTER_FORCE	0x8000	/* Force switch */

/* CPU Context area (1KB per CPU) */
#define CONTEXT_SIZE_BYTES_SHIFT	10
#define CONTEXT_SIZE_BYTES		(1<<CONTEXT_SIZE_BYTES_SHIFT)

/* CPU Context area (1KB per CPU) */
#define CONTEXT_SIZE_BYTES_SHIFT	10
#define CONTEXT_SIZE_BYTES		(1<<CONTEXT_SIZE_BYTES_SHIFT)

/* layout of IRAM used for LP1 save & restore */
#define TEGRA_IRAM_CODE_AREA		TEGRA_IRAM_BASE + SZ_4K
#define TEGRA_IRAM_CODE_SIZE		SZ_4K

#define CLK_RESET_CLK_MASK_ARM		0x44

#define FLOW_CTRL_WAITEVENT		(2<<29)
#define FLOW_CTRL_WAIT_FOR_INTERRUPT	(4<<29)
#define FLOW_CTRL_JTAG_RESUME		(1<<28)
#define FLOW_CTRL_CSR_INTR_FLAG	(1<<15)
#define FLOW_CTRL_CST_EVENT_FLAG	(1<<14)

#define EVP_CPU_RESET_VECTOR \
	(IO_ADDRESS(TEGRA_EXCEPTION_VECTORS_BASE) + 0x100)
#define CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x340)
#define CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x344)
#define CLK_RST_CONTROLLER_CLK_CPU_CMPLX \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x4c)
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#define CLK_RST_CONTROLLER_CPU_CMPLX_STATUS \
	CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET
#else
#define CLK_RST_CONTROLLER_CPU_CMPLX_STATUS \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x470)
#endif

#ifndef __ASSEMBLY__

#define FLOW_CTRL_HALT_CPUx_EVENTS(cpu)	((cpu)?(((cpu)-1)*0x8 + 0x14) : 0x0)
#define FLOW_CTRL_CPUx_CSR(cpu)		((cpu)?(((cpu)-1)*0x8 + 0x18) : 0x8)

static inline void flowctrl_writel(unsigned long val, unsigned int offs)
{
	__raw_writel(val, IO_ADDRESS(TEGRA_FLOW_CTRL_BASE) + offs);
	(void)__raw_readl(IO_ADDRESS(TEGRA_FLOW_CTRL_BASE) + offs);
}

extern void *tegra_context_area;

struct cpuidle_device;
struct cpuidle_state;

u64 tegra_rtc_read_ms(void);
void tegra_lp2_set_trigger(unsigned long cycles);
void tegra_lp2_in_idle(bool enable);
unsigned long tegra_lp2_timer_remain(void);
void __cortex_a9_save(unsigned int mode);
void __cortex_a9_restore(void);
void __shut_off_mmu(void);
void tegra_lp2_startup(void);
void tegra_hotplug_startup(void);
void tegra_flow_wfi(struct cpuidle_device *dev);
unsigned int tegra_suspend_lp2(unsigned int us, unsigned int flags);
void tegra_idle_stats_lp2_ready(unsigned int cpu);
void tegra_idle_stats_lp2_time(unsigned int cpu, s64 us);
void tegra_idle_enter_lp2_cpu_0(struct cpuidle_device *dev,
	struct cpuidle_state *state);
void tegra_idle_enter_lp2_cpu_n(struct cpuidle_device *dev,
	struct cpuidle_state *state);
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
static inline int tegra_cluster_control(unsigned int us, unsigned int flags)
{ return -EPERM; }
#define tegra_cluster_switch_prolog(flags) do {} while (0)
#define tegra_cluster_switch_epilog(flags) do {} while (0)
static inline unsigned int is_lp_cluster(void)
{ return 0; }
static inline unsigned long tegra_get_lpcpu_max_rate(void)
{ return 0; }
int tegra_cpudile_init_soc(void);
static inline bool tegra_lp2_is_allowed(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{ return true; }
#else
int tegra_cluster_control(unsigned int us, unsigned int flags);
void tegra_cluster_switch_prolog(unsigned int flags);
void tegra_cluster_switch_epilog(unsigned int flags);
unsigned int is_lp_cluster(void);
unsigned long tegra_get_lpcpu_max_rate(void);
static inline int tegra_cpudile_init_soc(void)
{ return 0; }
bool tegra_lp2_is_allowed(struct cpuidle_device *dev,
	struct cpuidle_state *state);
#endif
#endif

#endif
