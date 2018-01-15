/*
 * Copyright (c) 2014-2017 Nuvoton Technology corporation.
 *
 * Released under the GPLv2 only.
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

#define WDOG_REG_WTCR	0x1C		/* Watchdog register offset */
#define WDOG_BIT_WTIF	(0x01 << 3)	/* WTCR interrupt flag*/
#define WDOG_BIT_WTIE	(0x01 << 6)	/* WTCR enable interrupt*/

/*
 * Register usage

 * r12 - virt. base addr
 * r13 - reg_val
 */

	.text
	.global npcm7xx_wdt_fiq_end

ENTRY(npcm7xx_wdt_fiq_start)
	@ FIQ intrrupt handler
	MRC p15,0,r0,c0,c0,5@ read Multiprocessor ID register
	BIC r0, #0xFFFFFFF0
	CMP r0,#0x0
	BEQ THIS_IS_CPU0
	b .     @ loop - all other cpus stuck here

	THIS_IS_CPU0:

        ldr r13, [r12, #WDOG_REG_WTCR]
        tst r13, #WDOG_BIT_WTIF
	beq exit				@ none - spurious FIQ? exit

	ldr r13, [r12, #WDOG_REG_WTCR]
	bic r13, r13, #WDOG_BIT_WTIE            @ Disable watchdog interrupt
	str r13, [r12, #WDOG_REG_WTCR]

@@@@@@@@@@@@@@@@@@@@@@@@@@
@
@ FIQ - Kernel Dump
@ Add Assembler Code here ...


@
@
@@@@@@@@@@@@@@@@@@@@@@@@@@

	b .	@ CPU0 - loop until HW watchdog reset


exit:
	subs	pc, lr, #4			@ return from FIQ


npcm7xx_wdt_fiq_end:

/*
 * Check the size of the FIQ,
 * it cannot go beyond 0xffff0200, and is copied to 0xffff001c
 */
.if (npcm7xx_wdt_fiq_end - npcm7xx_wdt_fiq_start) > (0x200 - 0x1c)
	.err
.endif
