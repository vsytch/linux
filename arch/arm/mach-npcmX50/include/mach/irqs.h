/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2008 by Nuvoton Technology Corporation                                                   */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   irqs.h                                                                                                */
/*            This file contains IRQ definitions for Linux                                                 */
/*  Project:                                                                                               */
/*            Linux Kernel                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H __FILE__

#include "hal.h"
//--------------------------#include "map.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Defining interrupt number                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define FIQ_START 0

#if defined(CONFIG_PCI)
	#define IRQ_NPCMX50_MSI_0	(NPCMX50_PCIE_RC_INTERRUPT+1)
	#define NR_IRQS     (IRQ_NPCMX50_MSI_0 + NPCMX50_PCIERC_NUM_OF_MSI)
#else
	#define NR_IRQS     NPCMX50_GIC_INTERRUPT_NUM
#endif

#endif // __ASM_ARCH_IRQ_H

