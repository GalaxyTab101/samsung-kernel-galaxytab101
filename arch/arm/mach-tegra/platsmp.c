/*
 *  linux/arch/arm/mach-tegra/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 *  Copyright (C) 2009 Palm
 *  All Rights Reserved
 *
 *  Copyright (C) 2010-2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/completion.h>
#include <linux/sched.h>
#include <linux/cpu.h>
#include <linux/slab.h>
#include <linux/clk.h>

#include <asm/cacheflush.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/localtimer.h>
#include <asm/tlbflush.h>
#include <asm/smp_scu.h>
#include <asm/cpu.h>
#include <asm/mmu_context.h>

#include <mach/iomap.h>
#include <mach/powergate.h>
#include <mach/suspend.h>

#include "power.h"
#include "reset.h"
#include "clock.h"

bool tegra_all_cpus_booted = false;

static DEFINE_SPINLOCK(boot_lock);
static void __iomem *scu_base = IO_ADDRESS(TEGRA_ARM_PERIF_BASE);

#ifdef CONFIG_HOTPLUG_CPU
static DEFINE_PER_CPU(struct completion, cpu_killed);
extern void tegra_hotplug_startup(void);
#endif

static unsigned int available_cpus(void);
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
static inline int is_g_cluster_available(unsigned int cpu)
{ return -EPERM; }
static inline bool is_cpu_powered(unsigned int cpu)
{ return true; }
static inline int power_up_cpu(unsigned int cpu)
{ return 0; }

#else
static int is_g_cluster_available(unsigned int cpu);
static bool is_cpu_powered(unsigned int cpu);
static int power_up_cpu(unsigned int cpu);

#define CAR_BOND_OUT_V \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x390)
#define CAR_BOND_OUT_V_CPU_G	(1<<0)
#define CLK_RST_CONTROLLER_CPU_CMPLX_STATUS \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x470)

#endif

void __cpuinit platform_secondary_init(unsigned int cpu)
{
	trace_hardirqs_off();
	gic_cpu_init(0, IO_ADDRESS(TEGRA_ARM_PERIF_BASE) + 0x100);

	/* Initialize CPU0 dynamic power gating (n > 0). */
	tegra_cpu_dynamic_power_init();

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
#ifdef CONFIG_HOTPLUG_CPU
	INIT_COMPLETION(per_cpu(cpu_killed, cpu));
#endif
	tegra_cpu_init_map |= (1 << cpu);

	if (!tegra_all_cpus_booted)
		if (cpus_equal(*(cpumask_t*)&tegra_cpu_init_map, cpu_present_map))
			tegra_all_cpus_booted = true;

	spin_unlock(&boot_lock);
}
#ifdef CONFIG_TRUSTED_FOUNDATIONS
void callGenericSMC(u32 param0, u32 param1, u32 param2);
#endif

int boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;
	int status;

	if (is_lp_cluster()) {
		struct clk *cpu_clk, *cpu_g_clk;

		/* The G CPU may not be available for a
		   variety of reasons. */
		status = is_g_cluster_available(cpu);
		if (status)
			return status;

		cpu_clk = tegra_get_clock_by_name("cpu");
		cpu_g_clk = tegra_get_clock_by_name("cpu_g");

		/* Switch to G CPU before continuing. */
		if (!cpu_clk || !cpu_g_clk) {
			/* Early boot, clock infrastructure is not initialized
			   - CPU mode switch is not allowed */
			status = -EINVAL;
		} else
			status = clk_set_parent(cpu_clk, cpu_g_clk);

		if (status)
			return status;
	}

	/*
	 * set synchronization state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/* WARNING:
		The compiler just loves to reorder the following code.
		This code is very sensitive to the register write sequence.
		DO NOT remove the barrier() calls. */

#if CONFIG_TRUSTED_FOUNDATIONS
#error TrustedLogic change required
	callGenericSMC(0xFFFFFFFC, 0xFFFFFFE5, boot_vector);
#else
	barrier();
	writel(~0, EVP_CPU_RSVD_VECTOR);
	barrier();

	/* Force the CPU into reset. The CPU must remain in reset when the
	   flow controller state is cleared (which will cause the flow
	   controller to stop driving reset if the CPU has been power-gated
	   via the flow controller). This will have no effect on first boot
	   of the CPU since it should already be in reset. */
	writel(CPU_RESET(cpu), CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET);
	barrier();

	/* Unhalt the CPU. If the flow controller was used to power-gate the
	   CPU this will cause the flow controller to stop driving reset.
	   The CPU will remain in reset because the clock and reset block
	   is now driving reset. */
	wmb();
	flowctrl_writel(0, FLOW_CTRL_HALT_CPUx_EVENTS(cpu));
	barrier();

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
	flowctrl_writel(0, FLOW_CTRL_CPUx_CSR(cpu));
	barrier();
	{
		/* enable cpu clock on cpu */
		u32 reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
		writel(reg & ~CPU_CLOCK(cpu), CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
		barrier();
	}
#else
	/* On Tegra3 secondary CPU was power gated (not just halted). Clearing
	   CSR may abort power on state machine transition - do it only after
	   CPU is powered up */
	status = power_up_cpu(cpu);
	if (status)
		goto done;
	flowctrl_writel(0, FLOW_CTRL_CPUx_CSR(cpu));
	barrier();
#endif

	udelay(10);	/* power up delay */
	writel(CPU_RESET(cpu), CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR);
	wmb();
	barrier();

	timeout = jiffies + HZ;
	while (time_before(jiffies, timeout)) {
		if (readl(EVP_CPU_RSVD_VECTOR) != ~0) {
			status = 0;
			goto done;
		}
		udelay(10);
	}
	status = -ETIMEDOUT;

done:
#endif

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);
	return status;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init smp_init_cpus(void)
{
	unsigned int i, ncores = available_cpus();

	for (i = 0; i < ncores; i++)
		cpu_set(i, cpu_possible_map);

	/* Initialize the reset dispatcher. */
	tegra_cpu_reset_handler_init();

	/* Initialize CPU0 dynamic power gating. */
	tegra_cpu_dynamic_power_init();
}

void __init smp_prepare_cpus(unsigned int max_cpus)
{
	unsigned int ncores = available_cpus();
	unsigned int cpu = smp_processor_id();
	int i;

	smp_store_cpu_info(cpu);

	/*
	 * are we trying to boot more cores than exist?
	 */
	if (max_cpus > ncores)
		max_cpus = ncores;

	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);

	if (max_cpus == 1)
		tegra_all_cpus_booted = true;

#ifdef CONFIG_HOTPLUG_CPU
	for_each_present_cpu(i) {
		init_completion(&per_cpu(cpu_killed, i));
	}
#endif
	/* Switch to the CPU local timer whether there is more than one
	   CPU present or not. */
	percpu_timer_setup();

	/* Initialise the SCU if there are more than one CPU. */
	if (max_cpus > 1) {
		scu_enable(scu_base);
	}
}

#ifdef CONFIG_HOTPLUG_CPU

extern void vfp_sync_state(struct thread_info *thread);

void __cpuinit secondary_start_kernel(void);

int platform_cpu_kill(unsigned int cpu)
{
	unsigned int reg;
	int e;

	e = wait_for_completion_timeout(&per_cpu(cpu_killed, cpu), 100);
	printk(KERN_NOTICE "CPU%u: %s shutdown\n", cpu, (e) ? "clean":"forced");

	if (e) {
		do {
			reg = readl(CLK_RST_CONTROLLER_CPU_CMPLX_STATUS);
			cpu_relax();
		} while (!(reg & (1<<cpu)));
	} else {
		writel(CPU_RESET(cpu), CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET);
		/* put flow controller in WAIT_EVENT mode */
		flowctrl_writel(2<<29, FLOW_CTRL_HALT_CPUx_EVENTS(cpu));
	}

	spin_lock(&boot_lock);
	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	writel(reg | CPU_CLOCK(cpu), CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	spin_unlock(&boot_lock);
	return e;
}

void platform_cpu_die(unsigned int cpu)
{
#ifdef DEBUG
	unsigned int this_cpu = hard_smp_processor_id();

	if (cpu != this_cpu) {
		printk(KERN_CRIT "Eek! platform_cpu_die running on %u, should be %u\n",
			   this_cpu, cpu);
		BUG();
	}
#endif

	gic_cpu_exit(0);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* Tegra3 enters lpx states via WFI - do not propagate legacy IRQs
	   to CPU core to avoid fall through WFI; then gic output will be
	   enabled, however at this time - CPU is dying - no interrupt should
	   have afiinity to this CPU */
	tegra_irq_pass_through_disable();
#endif
	barrier();
	complete(&per_cpu(cpu_killed, cpu));
	flush_cache_all();
	tegra_cpu_reset_handler_flush(false);
	barrier();
	__cortex_a9_save(TEGRA_POWER_HOTPLUG_SHUTDOWN);

	/* return happens from __cortex_a9_restore */
	barrier();
	writel(smp_processor_id(), EVP_CPU_RSVD_VECTOR);
}

int platform_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	if (unlikely(!tegra_context_area))
		return -ENXIO;

	return cpu == 0 ? -EPERM : 0;
}
#endif

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)

static bool is_cpu_powered(unsigned int cpu)
{
	if (is_lp_cluster())
		return true;
	else
		return tegra_powergate_is_powered(TEGRA_CPU_POWERGATE_ID(cpu));
}

static int power_up_cpu(unsigned int cpu)
{
	int ret;
	u32 reg;
	unsigned long timeout;

	BUG_ON(cpu == smp_processor_id());
	BUG_ON(is_lp_cluster());

	/* This function is entered after CPU has been already un-gated by
	   flow controller. Wait for confirmation that cpu is powered and
	   remove clamps. */
	timeout = jiffies + HZ;
	do {
		if (is_cpu_powered(cpu))
			goto remove_clamps;
		udelay(10);
	} while (time_before(jiffies, timeout));

	/* Flow controller did not work as expected - try directly toggle
	   power gates. Bail out if direct power on also failed */
	if (!is_cpu_powered(cpu))
	{
		ret = tegra_powergate_power_on(TEGRA_CPU_POWERGATE_ID(cpu));
		if (ret)
			goto fail;

		/* Wait for the power to come up. */
		timeout = jiffies + 10*HZ;

		do {
			if (is_cpu_powered(cpu))
				goto remove_clamps;
			udelay(10);
		} while (time_before(jiffies, timeout));
		ret = -ETIMEDOUT;
		goto fail;
	}

remove_clamps:
	/* now CPU is up: enable clock, propagate reset, and remove clamps */
	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	writel(reg & ~CPU_CLOCK(cpu), CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	barrier();
	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);

	udelay(10);
	ret = tegra_powergate_remove_clamping(TEGRA_CPU_POWERGATE_ID(cpu));
fail:
	return ret;
}

static int is_g_cluster_available(unsigned int cpu)
{
	u32 fuse_sku = readl(FUSE_SKU_DIRECT_CONFIG);
	u32 bond_out = readl(CAR_BOND_OUT_V);

	/* Does the G CPU complex exist at all? */
	if ((fuse_sku & FUSE_SKU_DISABLE_ALL_CPUS) ||
	    (bond_out & CAR_BOND_OUT_V_CPU_G))
		return -EPERM;

	if (cpu >= available_cpus())
		return -EPERM;

	/* FIXME: The G CPU can be unavailable for a number of reasons
	 *	  (e.g., low battery, over temperature, etc.). Add checks for
	 *	  these conditions. */

	return 0;
}
#endif

static unsigned int available_cpus(void)
{
	static unsigned int ncores = 0;

	if (ncores == 0) {
		ncores = scu_get_core_count(scu_base);
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
		if (ncores > 1) {
			u32 fuse_sku = readl(FUSE_SKU_DIRECT_CONFIG);
			ncores -= FUSE_SKU_NUM_DISABLED_CPUS(fuse_sku);
			BUG_ON((int)ncores <= 0);
		}
#endif
	}
	return ncores;
}
