/*
 * arch/arm/mach-tegra/cpuidle.c
 *
 * Common CPU idle driver for Tegra CPUs
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

#include <linux/kernel.h>
#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/suspend.h>
#include <linux/tick.h>

#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <asm/localtimer.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/legacy_irq.h>
#include <mach/suspend.h>

#include "power.h"

static bool lp2_in_idle __read_mostly = true;
static bool lp2_disabled_by_suspend;
module_param(lp2_in_idle, bool, 0644);

int tegra_lp2_exit_latency;
static int tegra_lp2_power_off_time;

struct cpuidle_driver tegra_idle = {
	.name = "tegra_idle",
	.owner = THIS_MODULE,
};

static DEFINE_PER_CPU(struct cpuidle_device *, idle_devices);

void tegra_lp2_in_idle(bool enable)
{
	lp2_in_idle = enable;
}

void tegra_flow_wfi(struct cpuidle_device *dev)
{
	u32 halt = FLOW_CTRL_WAIT_FOR_INTERRUPT | FLOW_CTRL_JTAG_RESUME;
	u32 csr = FLOW_CTRL_CSR_INTR_FLAG | FLOW_CTRL_CSR_EVENT_FLAG;

	stop_critical_timings();
	dsb();
	flowctrl_writel(csr, FLOW_CTRL_CPUx_CSR(dev->cpu));
	flowctrl_writel(halt, FLOW_CTRL_HALT_CPUx_EVENTS(dev->cpu));
	__asm__ volatile ("wfi");
	flowctrl_writel(0, FLOW_CTRL_HALT_CPUx_EVENTS(dev->cpu));
	flowctrl_writel(csr, FLOW_CTRL_CPUx_CSR(dev->cpu));
	start_critical_timings();
}

static int tegra_idle_enter_lp3(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{
	ktime_t enter, exit;
	s64 us;

	local_irq_disable();
	local_fiq_disable();

	enter = ktime_get();
	if (!need_resched())
		tegra_flow_wfi(dev);
	exit = ktime_sub(ktime_get(), enter);
	us = ktime_to_us(exit);

	local_fiq_enable();
	local_irq_enable();
	return (int)us;
}

static int tegra_idle_enter_lp2(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{
	ktime_t enter, exit;
	s64 us;

	if (!lp2_in_idle || lp2_disabled_by_suspend ||
	    !tegra_lp2_is_allowed(dev, state))
		return tegra_idle_enter_lp3(dev, state);

	local_irq_disable();
	local_fiq_disable();
	enter = ktime_get();

	tegra_idle_stats_lp2_ready(dev->cpu);

#ifdef CONFIG_SMP
	if (dev->cpu != 0)
		tegra_idle_enter_lp2_cpu_n(dev, state);
	else
#endif
		tegra_idle_enter_lp2_cpu_0(dev, state);

	exit = ktime_sub(ktime_get(), enter);
	us = ktime_to_us(exit);

	local_fiq_enable();
	local_irq_enable();

	/* cpu clockevents may have been reset by powerdown */
	hrtimer_peek_ahead_timers();

	smp_rmb();
	state->exit_latency = tegra_lp2_exit_latency;
	state->target_residency = tegra_lp2_exit_latency +
		tegra_lp2_power_off_time;

	tegra_idle_stats_lp2_time(dev->cpu, us);

	return (int)us;
}

static int tegra_cpuidle_register_device(unsigned int cpu)
{
	struct cpuidle_device *dev;
	struct cpuidle_state *state;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->state_count = 0;
	dev->cpu = cpu;

	tegra_lp2_power_off_time = tegra_cpu_power_off_time();

	state = &dev->states[0];
	snprintf(state->name, CPUIDLE_NAME_LEN, "LP3");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU flow-controlled");
	state->exit_latency = 10;
	state->target_residency = 10;
	state->power_usage = 600;
	state->flags = CPUIDLE_FLAG_SHALLOW | CPUIDLE_FLAG_TIME_VALID;
	state->enter = tegra_idle_enter_lp3;
	dev->safe_state = state;
	dev->state_count++;

	state = &dev->states[1];
	snprintf(state->name, CPUIDLE_NAME_LEN, "LP2");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU power-gate");
	state->exit_latency = tegra_cpu_power_good_time();

	state->target_residency = tegra_cpu_power_off_time() +
		tegra_cpu_power_good_time();
	state->power_usage = 0;
	state->flags = CPUIDLE_FLAG_BALANCED | CPUIDLE_FLAG_TIME_VALID;
	state->enter = tegra_idle_enter_lp2;

	dev->power_specified = 1;
	dev->safe_state = state;
	dev->state_count++;

	if (cpuidle_register_device(dev)) {
		pr_err("CPU%u: failed to register idle device\n", cpu);
		kfree(dev);
		return -EIO;
	}
	per_cpu(idle_devices, cpu) = dev;
	return 0;
}

static int tegra_cpuidle_pm_notify(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
	if (event == PM_SUSPEND_PREPARE)
		lp2_disabled_by_suspend = true;
	else if (event == PM_POST_SUSPEND)
		lp2_disabled_by_suspend = false;

	return NOTIFY_OK;
}

static struct notifier_block tegra_cpuidle_pm_notifier = {
	.notifier_call = tegra_cpuidle_pm_notify,
};

static int __init tegra_cpuidle_init(void)
{
	unsigned int cpu;
	int ret;

	ret = tegra_cpudile_init_soc();
	if (ret)
		return ret;

	ret = cpuidle_register_driver(&tegra_idle);

	if (ret)
		return ret;

	for_each_possible_cpu(cpu) {
		if (tegra_cpuidle_register_device(cpu))
			pr_err("CPU%u: error initializing idle loop\n", cpu);
	}

	tegra_lp2_exit_latency = tegra_cpu_power_good_time();

	register_pm_notifier(&tegra_cpuidle_pm_notifier);

	return 0;
}

static void __exit tegra_cpuidle_exit(void)
{
	cpuidle_unregister_driver(&tegra_idle);
}

module_init(tegra_cpuidle_init);
module_exit(tegra_cpuidle_exit);

