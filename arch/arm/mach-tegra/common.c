/*
 * arch/arm/mach-tegra/common.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation.
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

#include <linux/console.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/memblock.h>

#include <asm/cacheflush.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/system.h>

#include <mach/iomap.h>
#include <mach/dma.h>
#include <mach/powergate.h>
#include <mach/system.h>

#include "apbio.h"
#include "board.h"
#include "clock.h"
#include "fuse.h"
#include "power.h"
#include "reset.h"

#define MC_SECURITY_CFG2	0x7c

unsigned long tegra_bootloader_fb_start;
unsigned long tegra_bootloader_fb_size;
unsigned long tegra_fb_start;
unsigned long tegra_fb_size;
unsigned long tegra_fb2_start;
unsigned long tegra_fb2_size;
unsigned long tegra_carveout_start;
unsigned long tegra_carveout_size;
unsigned long tegra_lp0_vec_start;
unsigned long tegra_lp0_vec_size;
unsigned long tegra_grhost_aperture = ~0ul;
static   bool is_tegra_debug_uart_hsport;

void (*tegra_reset)(char mode, const char *cmd);

static __initdata struct tegra_clk_init_table common_clk_init_table[] = {
	/* set up clocks that should always be on */
	/* name		parent		rate		enabled */
	{ "clk_m",	NULL,		0,		true },
	{ "pll_p",	NULL,		216000000,	true },
	{ "pll_p_out1",	"pll_p",	28800000,	true },
	{ "pll_p_out2",	"pll_p",	48000000,	true },
	{ "pll_p_out3",	"pll_p",	72000000,	true },
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	{ "pll_m_out1",	"pll_m",	275000000,	true },
	{ "sclk",	"pll_m_out1",	275000000,	true },
	{ "hclk",	"sclk",		275000000,	true },
	{ "pclk",	"hclk",		70000000,	true },
#else
	{ "pll_m_out1",	"pll_m",	120000000,	true },
	{ "sclk",	"pll_m_out1",	120000000,	true },
	{ "hclk",	"sclk",		120000000,	true },
	{ "pclk",	"hclk",		60000000,	true },
#endif
	{ "pll_x",	NULL,		0,		true },
	{ "cpu",	NULL,		0,		true },
	{ "emc",	NULL,		0,		true },
	{ "csite",	NULL,		0,		true },
	{ "timer", 	NULL,		0,		true },
	{ "kfuse",	NULL,		0,		true },
	{ "rtc",	NULL,		0,		true },

	/* set frequencies of some device clocks */
	{ "pll_u",	NULL,		480000000,	false },
	{ "sdmmc1",	"pll_c",	48000000,	true},
	{ "sdmmc3",	"pll_p",	48000000,	true},
	{ "sdmmc4",	"pll_p",	48000000,	true},
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	{ "vde",	"pll_c",	300000000,	false },
#endif
	{ NULL,		NULL,		0,		0},
};

void __init tegra_init_cache(void)
{
#ifdef CONFIG_CACHE_L2X0
	void __iomem *p = IO_ADDRESS(TEGRA_ARM_PERIF_BASE) + 0x3000;
	u32 aux_ctrl;

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
#ifndef CONFIG_TRUSTED_FOUNDATIONS
   /*
   ISSUE : Some registers of PL310 controler must be called from Secure context!
            When called form Normal we obtain an abort.
            Instructions that must be called in Secure :
               - Tag and Data RAM Latency Control Registers (0x108 & 0x10C) must be written in Secure.
        
   The following section of code has been regrouped in the implementation of "l2x0_init".
   The "l2x0_init" will in fact call an SMC intruction to switch from Normal context to Secure context.
   The configuration and activation will be done in Secure.
   */
	writel(0x331, p + L2X0_TAG_LATENCY_CTRL);
	writel(0x441, p + L2X0_DATA_LATENCY_CTRL);
	writel(2, p + L2X0_PWR_CTRL);
#endif

#elif defined(CONFIG_ARCH_TEGRA_3x_SOC)
#ifdef CONFIG_TEGRA_FPGA_PLATFORM
	writel(0x770, p + L2X0_TAG_LATENCY_CTRL);
	writel(0x770, p + L2X0_DATA_LATENCY_CTRL);
#else
	/* PL310 RAM latency is CPU dependent. NOTE: Changes here
	   must also be reflected in __cortex_a9_l2x0_restart */

	if (is_lp_cluster()) {
		writel(0x221, p + L2X0_TAG_LATENCY_CTRL);
		writel(0x221, p + L2X0_DATA_LATENCY_CTRL);
	} else {
		writel(0x331, p + L2X0_TAG_LATENCY_CTRL);
		writel(0x441, p + L2X0_DATA_LATENCY_CTRL);
	}
#endif

	/* Enable PL310 double line fill feature. */
	writel(((1<<30) | 7), p + L2X0_PREFETCH_OFFSET);
#endif
	aux_ctrl = readl(p + L2X0_CACHE_TYPE);
	aux_ctrl = (aux_ctrl & 0x700) << (17-8);
	aux_ctrl |= 0x6C000001;
	l2x0_init(p, aux_ctrl, 0x8200c3fe);
#endif
}

static void __init tegra_init_power(void)
{
	tegra_powergate_power_off(TEGRA_POWERGATE_MPE);
	tegra_powergate_power_off(TEGRA_POWERGATE_3D);
	tegra_powergate_power_off(TEGRA_POWERGATE_PCIE);
}

static bool console_flushed;

static void tegra_pm_flush_console(void)
{
	if (console_flushed)
		return;
	console_flushed = true;

	printk("\n");
	pr_emerg("Restarting %s\n", linux_banner);
	if (!try_acquire_console_sem()) {
		release_console_sem();
		return;
	}

	mdelay(50);

	local_irq_disable();
	if (try_acquire_console_sem())
		pr_emerg("tegra_restart: Console was locked! Busting\n");
	else
		pr_emerg("tegra_restart: Console was locked!\n");
	release_console_sem();
}

static void tegra_pm_restart(char mode, const char *cmd)
{
	tegra_pm_flush_console();
	arm_machine_restart(mode, cmd);
}

void tegra_cpu_reset_handler_enable(void)
{
	extern void __tegra_cpu_reset_handler(void);
	void __iomem *evp_cpu_reset =
		IO_ADDRESS(TEGRA_EXCEPTION_VECTORS_BASE + 0x100);
	unsigned long cpu_reset_handler;

	/* NOTE: This must be the one and only write to the CPU reset
		 vector in the entire system. */
	cpu_reset_handler = virt_to_phys(__tegra_cpu_reset_handler);
	writel(cpu_reset_handler, evp_cpu_reset);
	wmb();
}

void tegra_cpu_reset_handler_flush(bool l1cache)
{
	unsigned long first = (unsigned long)cpu_online_mask;
	unsigned long last  = (unsigned long)cpu_present_mask;

	if (first > last)
		swap(first,last);
	last += sizeof(struct cpumask);

	/* Push all of this data out to the L3 memory system. */
	if (l1cache) {
		__cpuc_coherent_kern_range(first, last);
		__cpuc_coherent_kern_range(
			(unsigned long)&__tegra_cpu_reset_handler_data[0],
			(unsigned long)&__tegra_cpu_reset_handler_data[TEGRA_RESET_DATA_SIZE]);
	}

	outer_clean_range(__pa(first), __pa(last));
	outer_clean_range(__pa(&__tegra_cpu_reset_handler_data[0]),
			  __pa(&__tegra_cpu_reset_handler_data[TEGRA_RESET_DATA_SIZE]));
}

void __init tegra_cpu_reset_handler_init(void)
{
#ifdef CONFIG_SMP
	/* Mark the initial boot CPU as initialized. */
	cpu_set(0, tegra_cpu_init_map);

	__tegra_cpu_reset_handler_data[TEGRA_RESET_MASK_PRESENT_PTR] =
		virt_to_phys((void*)cpu_present_mask);
	__tegra_cpu_reset_handler_data[TEGRA_RESET_STARTUP_SECONDARY] =
		virt_to_phys((void*)tegra_secondary_startup);
#ifdef CONFIG_HOTPLUG
	__tegra_cpu_reset_handler_data[TEGRA_RESET_MASK_ONLINE_PTR] =
		virt_to_phys((void*)cpu_online_mask);
	__tegra_cpu_reset_handler_data[TEGRA_RESET_STARTUP_HOTPLUG] =
		virt_to_phys((void*)tegra_hotplug_startup);
#endif
#endif
#ifdef CONFIG_PM
	__tegra_cpu_reset_handler_data[TEGRA_RESET_STARTUP_LP1] =
		TEGRA_IRAM_CODE_AREA;
#endif
	__tegra_cpu_reset_handler_data[TEGRA_RESET_STARTUP_LP2] =
		virt_to_phys((void*)tegra_lp2_startup);

	tegra_cpu_reset_handler_flush(true);
	tegra_cpu_reset_handler_enable();
}

void __init tegra_common_init(void)
{
	arm_pm_restart = tegra_pm_restart;
#ifndef CONFIG_SMP
	/* For SMP system, initializing the reset dispatcher here is too
	   late. For non-SMP systems, the function that initializes the
	   reset dispatcher is not called, so do it here for non-SMP. */
	tegra_cpu_reset_handler_init();
#endif
	tegra_init_fuse();
	tegra_init_clock();
	tegra_clk_init_from_table(common_clk_init_table);
	tegra_init_power();
	tegra_init_cache();
	tegra_dma_init();
	tegra_init_apb_dma();
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	tegra_mc_init(); /* !!!FIXME!!! Change Tegra3 behavior to match Tegra2 */
#endif
}

static int __init tegra_bootloader_fb_arg(char *options)
{
	char *p = options;

	tegra_bootloader_fb_size = memparse(p, &p);
	if (*p == '@')
		tegra_bootloader_fb_start = memparse(p+1, &p);

	pr_info("Found tegra_fbmem: %08lx@%08lx\n",
		tegra_bootloader_fb_size, tegra_bootloader_fb_start);

	return 0;
}
early_param("tegra_fbmem", tegra_bootloader_fb_arg);

static int __init tegra_lp0_vec_arg(char *options)
{
	char *p = options;
	unsigned long start;
	unsigned long size;

	size = memparse(p, &p);
	if (*p == '@') {
		start = memparse(p+1, &p);

		if (size && start) {
			tegra_lp0_vec_size  = size;
			tegra_lp0_vec_start = start;
		}
	}

	return 0;
}
early_param("lp0_vec", tegra_lp0_vec_arg);

static int __init tegra_debug_uartport(char *info)
{
	if (!strcmp(info, "hsport"))
		is_tegra_debug_uart_hsport = true;
	else if (!strcmp(info, "lsport"))
		is_tegra_debug_uart_hsport = false;

	return 1;
}

bool is_tegra_debug_uartport_hs(void)
{
	return is_tegra_debug_uart_hsport;
}

__setup("debug_uartport=", tegra_debug_uartport);

void tegra_get_board_info(struct board_info *bi)
{
	bi->board_id = (system_serial_high >> 16) & 0xFFFF;
	bi->sku = (system_serial_high) & 0xFFFF;
	bi->fab = (system_serial_low >> 24) & 0xFF;
	bi->major_revision = (system_serial_low >> 16) & 0xFF;
	bi->minor_revision = (system_serial_low >> 8) & 0xFF;
}

/*
 * Tegra has a protected aperture that prevents access by most non-CPU
 * memory masters to addresses above the aperture value.  Enabling it
 * secures the CPU's memory from the GPU, except through the GART.
 */
void __init tegra_protected_aperture_init(unsigned long aperture)
{
#ifndef CONFIG_NVMAP_ALLOW_SYSMEM
	void __iomem *mc_base = IO_ADDRESS(TEGRA_MC_BASE);
	pr_info("Enabling Tegra protected aperture at 0x%08lx\n", aperture);
	writel(aperture, mc_base + MC_SECURITY_CFG2);
#else
	pr_err("Tegra protected aperture disabled because nvmap is using "
		"system memory\n");
#endif
}

/*
 * Due to conflicting restrictions on the placement of the framebuffer,
 * the bootloader is likely to leave the framebuffer pointed at a location
 * in memory that is outside the grhost aperture.  This function will move
 * the framebuffer contents from a physical address that is anywher (lowmem,
 * highmem, or outside the memory map) to a physical address that is outside
 * the memory map.
 */
void tegra_move_framebuffer(unsigned long to, unsigned long from,
	unsigned long size)
{
	struct page *page;
	void __iomem *to_io;
	void *from_virt;
	unsigned long i;

	BUG_ON(PAGE_ALIGN((unsigned long)to) != (unsigned long)to);
	BUG_ON(PAGE_ALIGN(from) != from);
	BUG_ON(PAGE_ALIGN(size) != size);

	to_io = ioremap(to, size);
	if (!to_io) {
		pr_err("%s: Failed to map target framebuffer\n", __func__);
		return;
	}

	if (pfn_valid(page_to_pfn(phys_to_page(from)))) {
		for (i = 0 ; i < size; i += PAGE_SIZE) {
			page = phys_to_page(from + i);
			from_virt = kmap(page);
			memcpy_toio(to_io + i, from_virt, PAGE_SIZE);
			kunmap(page);
		}
	} else {
		void __iomem *from_io = ioremap(from, size);
		if (!from_io) {
			pr_err("%s: Failed to map source framebuffer\n",
				__func__);
			goto out;
		}

		for (i = 0; i < size; i+= 4)
			writel(readl(from_io + i), to_io + i);

		iounmap(from_io);
	}
out:
	iounmap(to_io);
}

void __init tegra_reserve(unsigned long carveout_size, unsigned long fb_size,
	unsigned long fb2_size)
{
	if (tegra_lp0_vec_size)
		if (memblock_reserve(tegra_lp0_vec_start, tegra_lp0_vec_size)) {
			pr_err("Failed to reserve lp0_vec %08lx@%08lx\n",
				tegra_lp0_vec_size, tegra_lp0_vec_start);
			tegra_lp0_vec_start = 0;
			tegra_lp0_vec_size = 0;
		}

	if (carveout_size) {
		tegra_carveout_start = memblock_end_of_DRAM() - carveout_size;
		if (memblock_remove(tegra_carveout_start, carveout_size)) {
			pr_err("Failed to remove carveout %08lx@%08lx "
				"from memory map\n",
				tegra_carveout_start, carveout_size);
			tegra_carveout_start = 0;
			tegra_carveout_size = 0;
		}
		else
			tegra_carveout_size = carveout_size;
	}

	if (fb2_size) {
		tegra_fb2_start = memblock_end_of_DRAM() - fb2_size;
		if (memblock_remove(tegra_fb2_start, fb2_size)) {
			pr_err("Failed to remove second framebuffer %08lx@%08lx "
				"from memory map\n",
				tegra_fb2_start, fb2_size);
			tegra_fb2_start = 0;
			tegra_fb2_size = 0;
		} else
			tegra_fb2_size = fb2_size;
	}

	if (fb_size) {
		tegra_fb_start = memblock_end_of_DRAM() - fb_size;
		if (memblock_remove(tegra_fb_start, fb_size)) {
			pr_err("Failed to remove framebuffer %08lx@%08lx "
				"from memory map\n",
				tegra_fb_start, fb_size);
			tegra_fb_start = 0;
			tegra_fb_size = 0;
		} else
			tegra_fb_size = fb_size;
	}

	if (tegra_fb_size)
		tegra_grhost_aperture = tegra_fb_start;

	if (tegra_fb2_size && tegra_fb2_start < tegra_grhost_aperture)
		tegra_grhost_aperture = tegra_fb2_start;

	if (tegra_carveout_size && tegra_carveout_start < tegra_grhost_aperture)
		tegra_grhost_aperture = tegra_carveout_start;

#ifdef CONFIG_TEGRA_IOVMM_SMMU
	if (memblock_reserve(TEGRA_SMMU_BASE, TEGRA_SMMU_SIZE)) {
		pr_err("Failed to reserve SMMU I/O VA window %08x@%08x\n",
			TEGRA_SMMU_BASE, TEGRA_SMMU_SIZE);
	}
#endif

	/*
	 * TODO: We should copy the bootloader's framebuffer to the framebuffer
	 * allocated above, and then free this one.
	 */
	if (tegra_bootloader_fb_size)
		if (memblock_reserve(tegra_bootloader_fb_start,
				tegra_bootloader_fb_size)) {
			pr_err("Failed to reserve bootloader frame buffer %08lx@%08lx\n",
				tegra_bootloader_fb_size, tegra_bootloader_fb_start);
			tegra_bootloader_fb_start = 0;
			tegra_bootloader_fb_size = 0;
		}

	pr_info("Tegra reserved memory:\n"
		"LP0:                    %08lx - %08lx\n"
		"Bootloader framebuffer: %08lx - %08lx\n"
		"Framebuffer:            %08lx - %08lx\n"
		"2nd Framebuffer:        %08lx - %08lx\n"
		"Carveout:               %08lx - %08lx\n",
		tegra_lp0_vec_start,
		tegra_lp0_vec_size ?
			tegra_lp0_vec_start + tegra_lp0_vec_size - 1 : 0,
		tegra_bootloader_fb_start,
		tegra_bootloader_fb_size ?
			tegra_bootloader_fb_start + tegra_bootloader_fb_size - 1 : 0,
		tegra_fb_start,
		tegra_fb_size ?
			tegra_fb_start + tegra_fb_size - 1 : 0,
		tegra_fb2_start,
		tegra_fb2_size ?
			tegra_fb2_start + tegra_fb2_size - 1 : 0,
		tegra_carveout_start,
		tegra_carveout_size ?
			tegra_carveout_start + tegra_carveout_size - 1 : 0);

#ifdef CONFIG_TEGRA_IOVMM_SMMU
	pr_info("SMMU:                   %08x - %08x\n",
		TEGRA_SMMU_BASE, TEGRA_SMMU_BASE + TEGRA_SMMU_SIZE - 1);
#endif
}
