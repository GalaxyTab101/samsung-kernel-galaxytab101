/*
 * arch/arm/mach-tegra/power.h
 *
 * Declarations for power state transition code
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

#ifndef __MACH_TEGRA_POWER_H
#define __MACH_TEGRA_POWER_H

/* Setting this disable per-CPU LP2 wake on interrupt. This must be set to 1
   until bug 790458 is fixed after which the code associated with this can
   be removed. */
#define WAR_790458	1

#include <mach/iomap.h>
#include <asm/page.h>

#define TEGRA_POWER_PWRREQ_POLARITY	0x1	/* core power request polarity */
#define TEGRA_POWER_PWRREQ_OE		0x2	/* core power request enable */
#define TEGRA_POWER_SYSCLK_POLARITY	0x4	/* sys clk polarity */
#define TEGRA_POWER_SYSCLK_OE		0x8	/* system clock enable */
#define TEGRA_POWER_PWRGATE_DIS		0x10	/* power gate disabled */
#define TEGRA_POWER_EFFECT_LP0		0x40	/* enter LP0 when CPU pwr gated */
#define TEGRA_POWER_CPU_PWRREQ_POLARITY 0x80	/* CPU power request polarity */
#define TEGRA_POWER_CPU_PWRREQ_OE	0x100	/* CPU power request enable */
#define TEGRA_POWER_PMC_SHIFT		8
#define TEGRA_POWER_PMC_MASK		0x1ff
#define TEGRA_POWER_SDRAM_SELFREFRESH	0x400	/* SDRAM is in self-refresh */
#define TEGRA_POWER_HOTPLUG_SHUTDOWN	0x800	/* Hotplug shutdown */

#define TEGRA_POWER_CLUSTER_G		0x1000	/* G CPU */
#define TEGRA_POWER_CLUSTER_LP		0x2000	/* LP CPU */
#define TEGRA_POWER_CLUSTER_MASK	0x3000
#define TEGRA_POWER_CLUSTER_IMMEDIATE	0x4000	/* Immediate wake */
#define TEGRA_POWER_CLUSTER_FORCE	0x8000	/* Force switch */

/* CPU Context area (1KB per CPU) */
#define CONTEXT_SIZE_BYTES_SHIFT	10
#define CONTEXT_SIZE_BYTES		(1<<CONTEXT_SIZE_BYTES_SHIFT)

/* layout of IRAM used for LP1 save & restore */
#define TEGRA_IRAM_CODE_AREA		TEGRA_IRAM_BASE + SZ_4K
#define TEGRA_IRAM_CODE_SIZE		SZ_4K

#define FLOW_CTRL_HALT_CPU_EVENTS	0x0
#define   FLOW_CTRL_WAITEVENT		(2<<29)
#define   FLOW_CTRL_WAIT_FOR_INTERRUPT	(4<<29)
#define   FLOW_CTRL_JTAG_RESUME		(1<<28)
#define   FLOW_CTRL_HALT_CPU_FIQ	(1<<10)
#define   FLOW_CTRL_HALT_CPU_IRQ	(1<<8)
#define FLOW_CTLR_CPU_CSR		0x8
#define   FLOW_CTRL_CSR_INTR_FLAG	(1<<15)
#define   FLOW_CTRL_CSR_EVENT_FLAG	(1<<14)
#define   FLOW_CTRL_CSR_ENABLE		(1<<0)
#define FLOW_CTLR_HALT_CPU1_EVENTS	0x14
#define FLOW_CTLR_CPU1_CSR		0x18

#ifndef __ASSEMBLY__

#define EVP_CPU_RSVD_VECTOR \
	(IO_ADDRESS(TEGRA_EXCEPTION_VECTORS_BASE) + 0x114)
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

#define CPU_CLOCK(cpu)	(0x1<<(8+cpu))
#define CPU_RESET(cpu)	(0x1111ul<<(cpu))

#define FLOW_CTRL_HALT_CPUx_EVENTS(cpu)	(IO_ADDRESS(TEGRA_FLOW_CTRL_BASE) + \
	((cpu)?(((cpu)-1)*0x8 + 0x14) : 0x0))
#define FLOW_CTRL_CPUx_CSR(cpu)		(IO_ADDRESS(TEGRA_FLOW_CTRL_BASE) + \
	((cpu)?(((cpu)-1)*0x8 + 0x18) : 0x8))

#define FLOW_CTRL_CLUSTER_CONTROL \
	(IO_ADDRESS(TEGRA_FLOW_CTRL_BASE) + 0x2c)
#define FLOW_CTRL_CPU_CSR_IMMEDIATE_WAKE	(1<<3)
#define FLOW_CTRL_CPU_CSR_SWITCH_CLUSTER	(1<<2)

#define FUSE_SKU_DIRECT_CONFIG \
	(IO_ADDRESS(TEGRA_FUSE_BASE) + 0x1F4)
#define FUSE_SKU_DISABLE_ALL_CPUS	(1<<5)
#define FUSE_SKU_NUM_DISABLED_CPUS(x)	(((x) >> 3) & 3)

static inline void flowctrl_writel(unsigned long val, void __iomem *addr)
{
	writel(val, addr);
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	wmb();
#endif
	(void)__raw_readl(addr);
}

extern void *tegra_context_area;

#ifdef CONFIG_SMP
extern bool tegra_all_cpus_booted __read_mostly;
#else
#define tegra_all_cpus_booted (true)
#endif

struct cpuidle_device;
struct cpuidle_state;

u64 tegra_rtc_read_ms(void);
void tegra_lp2_set_trigger(unsigned long cycles);
void tegra_lp2_in_idle(bool enable);
unsigned long tegra_lp2_timer_remain(void);
void __cortex_a9_save(unsigned int mode);
void __cortex_a9_restore(void);
void __shut_off_mmu(void);
void tegra_secondary_startup(void);
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
void tegra_cpu_dynamic_power_init(void);

#if defined(CONFIG_TEGRA_AUTO_HOTPLUG) && !defined(CONFIG_ARCH_TEGRA_2x_SOC)
int tegra_auto_hotplug_init(struct mutex *cpu_lock);
void tegra_auto_hotplug_exit(void);
void tegra_auto_hotplug_governor(unsigned int cpu_freq);
#else
static inline int tegra_auto_hotplug_init(struct mutex *cpu_lock)
{ return 0; }
static inline void tegra_auto_hotplug_exit(void)
{ }
static inline void tegra_auto_hotplug_governor(unsigned int cpu_freq)
{ }
#endif

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#define INSTRUMENT_CLUSTER_SWITCH 0	/* Must be zero for ARCH_TEGRA_2x_SOC */
#define DEBUG_CLUSTER_SWITCH 0		/* Must be zero for ARCH_TEGRA_2x_SOC */
#define PARAMETERIZE_CLUSTER_SWITCH 0	/* Must be zero for ARCH_TEGRA_2x_SOC */
static inline int tegra_cluster_control(unsigned int us, unsigned int flags)
{ return -EPERM; }
#define tegra_cluster_switch_prolog(flags) do {} while (0)
#define tegra_cluster_switch_epilog(flags) do {} while (0)
static inline bool is_g_cluster_present(void)
{ return true; }
static inline unsigned int is_lp_cluster(void)
{ return 0; }
int tegra_cpudile_init_soc(void);
static inline bool tegra_lp2_is_allowed(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{ return true; }
#define tegra_lp0_suspend_mc() do {} while (0)
#define tegra_lp0_resume_mc() do {} while (0)
#else
#define INSTRUMENT_CLUSTER_SWITCH 1	/* Should be zero for shipping code */
#define DEBUG_CLUSTER_SWITCH 1		/* Should be zero for shipping code */
#define PARAMETERIZE_CLUSTER_SWITCH 1	/* Should be zero for shipping code */
int tegra_cluster_control(unsigned int us, unsigned int flags);
void tegra_cluster_switch_prolog(unsigned int flags);
void tegra_cluster_switch_epilog(unsigned int flags);
static inline bool is_g_cluster_present(void)
{
	u32 fuse_sku = readl(FUSE_SKU_DIRECT_CONFIG);
	if (fuse_sku & FUSE_SKU_DISABLE_ALL_CPUS)
		return false;
	return true;
}
static inline unsigned int is_lp_cluster(void)
{
	unsigned int reg;
	reg = readl(FLOW_CTRL_CLUSTER_CONTROL);
	return (reg & 1); /* 0 == G, 1 == LP*/
}
int tegra_cpudile_init_soc(void);
bool tegra_lp2_is_allowed(struct cpuidle_device *dev,
	struct cpuidle_state *state);
void tegra_lp0_suspend_mc(void);
void tegra_lp0_resume_mc(void);
#endif
#if DEBUG_CLUSTER_SWITCH
extern unsigned int tegra_cluster_debug;
#define DEBUG_CLUSTER(x) do { if (tegra_cluster_debug) printk x; } while (0)
#else
#define DEBUG_CLUSTER(x) do { } while (0)
#endif
#if PARAMETERIZE_CLUSTER_SWITCH
void tegra_cluster_switch_set_parameters(unsigned int us, unsigned int flags);
#else
static inline void tegra_cluster_switch_set_parameters(
	unsigned int us, unsigned int flags)
{ }
#endif

#endif

#endif
