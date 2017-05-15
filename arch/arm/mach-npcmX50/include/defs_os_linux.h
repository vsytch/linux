/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation Confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2009-2016 by Nuvoton Technology Corporation                                              */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   defs_os_linux.h                                                                                       */
/*            This file contains definitions for Linux kernel API                                          */
/* Project:                                                                                                */
/*            SWC DEFS                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef _DEFS_LINUX_KERNEL_H_
#define _DEFS_LINUX_KERNEL_H_

#include <asm/io.h>

/*---------------------------------------------------------------------------------------------------------*/
/* OS dependent types                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
typedef unsigned int        UINT;                       /* Native type of the core that fits the core's    */
typedef int                 INT;                        /* internal registers                              */
typedef UINT                BOOLEAN;

/*---------------------------------------------------------------------------------------------------------*/
/* OS dependent PTR definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define REG8                volatile  UINT8
#define REG16               volatile  UINT16
#define REG32               volatile  UINT32

#define PTR8                REG8 *
#define PTR16               REG16 *
#define PTR32               REG32 *

/*---------------------------------------------------------------------------------------------------------*/
/* OS dependent MEM definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define MEMR8(a)            ioread8((UINT8*)a)
#define MEMR16(a)           ioread16((UINT16*)a)
#define MEMR32(a)           ioread32((UINT32*)a)

#define MEMW8(a,v)          iowrite8( (v), ((UINT8*)a))
#define MEMW16(a,v)         iowrite16((v), ((UINT16*)a))
#define MEMW32(a,v)         iowrite32((v), ((UINT32*)a))

/*---------------------------------------------------------------------------------------------------------*/
/* OS dependent IO definitions                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define IOR8(a)             inb(a)
#define IOR16(a)            inw(a)
#define IOR32(a)            inl(a)

#define IOW8(a,v)           outb((v), (a))
#define IOW16(a,v)          outw((v), (a))
#define IOW32(a,v)          outl((v), (a))

/*---------------------------------------------------------------------------------------------------------*/
/* Interrupts macros                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

/* Not Supported on ALL architectures                                                                      */
#define ENABLE_INTERRUPTS()                 local_irq_enable()
#define DISABLE_INTERRUPTS()                local_irq_disable()

/* Supported on ALL architectures                                                                          */
#define INTERRUPTS_SAVE_DISABLE(var)        local_irq_save(var)
#define INTERRUPTS_RESTORE(var)             local_irq_restore(var)

/*---------------------------------------------------------------------------------------------------------*/
/* Cache macros                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define ICACHE_SAVE_DISABLE(var)            var=0
#define ICACHE_SAVE_ENABLE(var)             var=0
#define ICACHE_RESTORE(var)

/*---------------------------------------------------------------------------------------------------------*/
/* Assertion macros                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef ASSERT
#ifdef DEBUG
#define ASSERT(cond)  {if (!(cond)) for(;;) ;}           /* infinite loop                                  */
#else
#define ASSERT(cond)
#endif
#endif

#endif /* _DEFS_LINUX_KERNEL_H_ */

