/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2008 by Nuvoton Technology Corporation                                                   */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   uncompress.h                                                                                          */
/*            This file contains functions to handle uncompress msg                                        */
/*            The file included in standalone program that runs in real-mode without MMU                   */
/*            (before loading the Linux Kernel)                                                            */
/*                                                                                                         */
/*            We assume that the UART driver was already initialized by the booter, otherwise              */
/*            no uncompress messages will be seen (maybe wanted behavior)                                  */
/*  Project:                                                                                               */
/*            Linux Kernel                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

#include <mach/hal.h>
#include <mach/map.h>
#include <linux/serial_reg.h>

#define TX_DONE	(UART_LSR_TEMT | UART_LSR_THRE)
static volatile u32 * const uart_base = (u32 *)NPCMX50_UART_BASE_ADDR(0);

/*---------------------------------------------------------------------------------------------------------*/
/* putc implementation                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void putc(int ch)
{
	/* Check THRE and TEMT bits before we transmit the character.
	 */
	while ((uart_base[UART_LSR] & TX_DONE) != TX_DONE)
		barrier();

	*uart_base = ch;
}


/*---------------------------------------------------------------------------------------------------------*/
/* Unused functions                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define arch_decomp_setup()
#define flush()
#define arch_decomp_wdog()


#endif //__ASM_ARCH_UNCOMPRESS_H
