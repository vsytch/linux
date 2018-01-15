/*
 * Copyright (c) 2017 Nuvoton Technology corporation.
 *
 * Debugging macro include header
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#define NPCM7XX_DBGU_SR			(0x14)		/* Status Register */
#define NPCM7XX_DBGU_THR		(0x00)		/* Transmitter Holding Register */
#define NPCM7XX_DBGU_TXRDY		(1 << 6)	/* Transmitter Ready */
#define NPCM7XX_DBGU_TXEMPTY	(1 << 6)	/* Transmitter Empty */

	.macro	addruart, rp, rv, tmp
	ldr	\rp, =CONFIG_DEBUG_UART_PHYS		@ System peripherals (phys address)
	ldr	\rv, =CONFIG_DEBUG_UART_VIRT		@ System peripherals (virt address)
	.endm

	.macro	senduart,rd,rx
	strb	\rd, [\rx, #(NPCM7XX_DBGU_THR)]		@ Write to Transmitter Holding Register
	.endm

	.macro	waituart,rd,rx
1001:	ldr	\rd, [\rx, #(NPCM7XX_DBGU_SR)]		@ Read Status Register
	tst	\rd, #NPCM7XX_DBGU_TXRDY			@ DBGU_TXRDY = 1 when ready to transmit
	beq	1001b
	.endm

	.macro	busyuart,rd,rx
1001:	ldr	\rd, [\rx, #(NPCM7XX_DBGU_SR)]		@ Read Status Register
	tst	\rd, #NPCM7XX_DBGU_TXEMPTY			@ DBGU_TXEMPTY = 1 when transmission complete
	beq	1001b
	.endm

