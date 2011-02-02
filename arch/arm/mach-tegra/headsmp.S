/*
 * arch/arm/mach-tegra/headsmp.S
 *
 * SMP initialization routines for Tegra SoCs
 *
 * Copyright (c) 2009-2011, NVIDIA Corporation.
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

#include <linux/linkage.h>
#include <linux/init.h>

#include <asm/assembler.h>
#include <asm/cache.h>

#include <mach/iomap.h>
#include <mach/io.h>

#include "power.h"
#include "power-macros.S"
#include "reset.h"

#define DEBUG_CPU_RESET_HANDLER	0	/* Non-zero enables debug code */

#define RST_DEVICES_U		0xc
#define PMC_SCRATCH41		0x140
#define TWD_BASE		(TEGRA_ARM_PERIF_BASE + 0x600)

#define RESET_DATA(x)		((TEGRA_RESET_##x)*4)


/*        .section ".cpuinit.text", "ax"*/

.macro	poke_ev, val, tmp
	mov32	\tmp, (TEGRA_EXCEPTION_VECTORS_BASE + 0x114)
	str	\val, [\tmp]
.endm

/*
 * __tegra_cpu_reset_handler_halt_failed:
 *
 * Alternate entry point for reset handler for cases where the
 * WFI halt failed to take effect.
 *
 */
	.align PAGE_SHIFT
ENTRY(__tegra_cpu_reset_handler_start)
ENTRY(__tegra_cpu_reset_handler_halt_failed)

#if DEBUG_CPU_RESET_HANDLER
	b	.				@ Debug spin
#endif
#ifdef CONFIG_HAVE_ARM_TWD
	cpsid	aif, 0x13			@ SVC mode, interrupts disabled
	mrc	p15, 0, r10, c0, c0, 5		@ MPIDR
	and	r10, r10, #0x3			@ R10 = CPU number
	mov32	r1, 0xBAD00000
	orr	r1, r1, r10
	mov	r0, #0
	mov32	r2, TWD_BASE			@ Local timer base
	str	r0, [r2, #8]			@ Disable timer in case it's running
	str	r1, [r2, #0]			@ Save halt fail status
	b	__tegra_cpu_reset_handler
#endif
ENDPROC(__tegra_cpu_reset_handler_halt_failed)

/*
 * __tegra_cpu_reset_handler:
 *
 * Common handler for all CPU reset events.
 *
 * Register usage within the reset handler:
 *
 *	R5  = CPU present (to the OS) mask
 *	R6  = CPU initialized mask
 *	R7  = CPU online mask
 *	R8  = CPU in LP1 state mask
 *	R9  = CPU in LP2 state mask
 *	R10 = CPU number
 *	R11 = CPU mask
 *	R12 = pointer to reset handler data
 */

	.align L1_CACHE_SHIFT
	b	__tegra_cpu_reset_handler_halt_failed
ENTRY(__tegra_cpu_reset_handler)

#if DEBUG_CPU_RESET_HANDLER
	b	.
#endif
	cpsid	aif, 0x13			@ SVC mode, interrupts disabled
	mrc	p15, 0, r10, c0, c0, 5		@ MPIDR
	and	r10, r10, #0x3			@ R10 = CPU number
	mov	r11, #1
	mov	r11, r11, lsl r10  		@ R11 = CPU mask
	add	r12, pc, #__tegra_cpu_reset_handler_data-(.+8)

#ifdef CONFIG_SMP
	ldr	r5, [r12, #RESET_DATA(MASK_PRESENT_PTR)]
	cmp	r5, #0
	bleq	__die				@ present mask not initialized
	ldr	r5, [r5]

	ldr	r6, [r12, #RESET_DATA(MASK_INIT)]

	ldr	r7, [r12, #RESET_DATA(MASK_ONLINE_PTR)]
	cmp	r7, #0
	bleq	__die				@ online mask not initialized
	ldr	r7, [r7]
#else
	mov	r5, #0
	mov	r6, #0
	mov	r7, #0
#endif

#ifdef CONFIG_PM
	ldr	r8, [r12, #RESET_DATA(MASK_LP1)]
#else
	mov	r8, #0
#endif
	ldr	r9, [r12, #RESET_DATA(MASK_LP2)]

#ifdef CONFIG_SMP
	/* Does the OS know about this CPU? */
	tst	r5, r11 			@ if !present
	bleq	__die				@ CPU not present (to OS)

	/* Has this CPU ever been booted (and not CPU0)? */
	tst	r6, r11				@ if initialized
	bne	__is_initialized
	cmp	r10, #0
	bleq	__die				@ CPU0 cannot be here
	ldr	lr, [r12, #RESET_DATA(STARTUP_SECONDARY)]
	cmp	lr, #0
	bleq	__die				@ no secondary startup handler
	bx	lr
__is_initialized:

#ifdef CONFIG_HOTPLUG
	/* Is this CPU currently online to to the OS? */
	tst 	r7, r11				@ if online
	bne	__is_online
	cmp	r10, #0
	bleq	__die				@ CPU0 cannot be here
	ldr	lr, [r12, #RESET_DATA(STARTUP_HOTPLUG)]
	cmp	lr, #0
	bleq	__die				@ no hotplug startup handler
	bx	lr
__is_online:
#endif
#endif
#ifdef CONFIG_PM
	/* Waking up from LP1? */
	tst	r8, r11				@ if in_lp1
	beq	__is_not_lp1
	cmp	r10, #0
	bne	__die				@ only CPU0 can be here
	ldr	lr, [r12, #RESET_DATA(STARTUP_LP1)]
	cmp	lr, #0
	bleq	__die				@ no LP1 startup handler
	bx	lr
__is_not_lp1:
#endif

	/* Waking up from LP2? */
	tst	r9, r11				@ if in_lp2
	beq	__is_not_lp2
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	cmp	r10, #1
	bne	__is_not_cpu1
	/* Tegra2 CPU1 LP2 wakeup uses the hotplug startup handler */
	ldr	lr, [r12, #RESET_DATA(STARTUP_HOTPLUG)]
	cmp	lr, #0
	bleq	__die				@ no hotplug startup handler
	bx	lr
__is_not_cpu1:
#endif
	ldr	lr, [r12, #RESET_DATA(STARTUP_LP2)]
	cmp	lr, #0
	bleq	__die				@ no LP2 startup handler
	bx	lr

__is_not_lp2:

	blne	__die				@ bad news
/*
 * We don't know why the CPU reset. Just kill it.
 * The LR register will contain the address we died at + 4.
 */

__die:
	sub	lr, lr, #4
	mov32	r5, TEGRA_PMC_BASE
	str	lr, [r5, #PMC_SCRATCH41]

	mov32	r5, TEGRA_CLK_RESET_BASE

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	mov32	r0, 0x1111
	mov	r1, r0, lsl r10
	str	r1, [r5, #0x340]		@ CLK_RST_CPU_CMPLX_SET
#else
	mov32	r6, TEGRA_FLOW_CTRL_BASE

	cmp	r10, #0
	moveq	r1, #FLOW_CTRL_HALT_CPU_EVENTS
	moveq	r2, #FLOW_CTLR_CPU_CSR
	movne	r1, r10, lsl #3
	addne	r2, r1, #(FLOW_CTLR_CPU1_CSR-8)
	addne	r1, r1, #(FLOW_CTLR_HALT_CPU1_EVENTS-8)

	/* Clear CPU "event" and "interrupt" flags and power gate
	   it when halting but not before it is in the "WFI" state. */
	ldr	r0, [r6, +r2]
	orr	r0, r0, #FLOW_CTRL_CSR_INTR_FLAG | FLOW_CTRL_CSR_EVENT_FLAG
	orr	r0, r0, #FLOW_CTRL_CSR_ENABLE
	str	r0, [r6, +r2]

	/* Unconditionally halt this CPU */
	mov	r0, #FLOW_CTRL_WAITEVENT
	str	r0, [r6, +r1]
	ldr	r0, [r6, +r1]			@ memory barrier

	dsb
	isb
	wfi					@ CPU should be power gated here

	/* If the CPU didn't power gate above just kill it's clock. */

	mov	r0, r11, lsl #8
	str	r0, [r5, #348]			@ CLK_CPU_CMPLX_SET
#endif
	/* If the CPU still isn't dead, just spin here. */
	b	.

ENDPROC(__tegra_cpu_reset_handler)
	.align L1_CACHE_SHIFT
	.type	__tegra_cpu_reset_handler_data, %object
	.globl	__tegra_cpu_reset_handler_data
__tegra_cpu_reset_handler_data:
	.rept	TEGRA_RESET_DATA_SIZE
	.long	0
	.endr
	.size	__tegra_cpu_reset_handler_data, .-tegra_cpu_reset_handler_data
	.align L1_CACHE_SHIFT
ENTRY(__tegra_cpu_reset_handler_end)


#ifdef CONFIG_SMP
/*
 *	tegra_secondary_startup
 *
 *	 Initial secondary processor boot vector; jumps to kernel's
 *	 secondary_startup routine
 */
	.align L1_CACHE_SHIFT
ENTRY(tegra_secondary_startup)
	msr	cpsr_fsxc, #0xd3
	bl	__invalidate_cpu_state
	cpu_id	r0
	enable_coresite r1
	poke_ev	r0, r1
	b	secondary_startup
ENDPROC(tegra_secondary_startup)
#endif

/*
 *	__enable_coresite_access
 *
 *	  Called only on CPU0 to take the CoreSight debug interface out of
 *	  reset. Called with MMU disabled.
 */
	.align L1_CACHE_SHIFT
ENTRY(__enable_coresite_access)
	mov32	r0, (TEGRA_CLK_RESET_BASE + RST_DEVICES_U)
	mov32	r2, (TEGRA_TMRUS_BASE)

	/* assert reset for 2usec */
	ldr	r1, [r0]
#ifndef CONFIG_TEGRA_FPGA_PLATFORM
	orr	r1, #(1<<9)
	str	r1, [r0]
#endif
	wait_for_us r3, r2, r4
	add	r3, r3, #2
	bic	r1, r1, #(1<<9)
	wait_until r3, r2, r4
	str	r1, [r0]
	/* Enable CoreSight */
	enable_coresite r3
	bx	lr
ENDPROC(__enable_coresite_access)
