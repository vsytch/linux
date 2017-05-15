/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation Confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2009-2016 by Nuvoton Technology Corporation                                              */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   defs_core_cop8.h                                                                                      */
/*            This file contains definitions for COP8 core and its compilers                               */
/* Project:                                                                                                */
/*            SWC DEFS                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef _DEFS_COP8_H_
#define _DEFS_COP8_H_

#define __COP8_IAR__

#include <ioc8sdr.h>
#include <incop8.h>

/*---------------------------------------------------------------------------------------------------------*/
/* Core dependent types                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
typedef UINT8               UINT;                       /* Native type of the core that fits the core's    */
typedef INT8                INT;                        /* internal registers                              */
typedef UINT                BOOLEAN;

/*---------------------------------------------------------------------------------------------------------*/
/* Core dependent PTR definitions                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define FAR                 __far

#define REG8                volatile __nonbanked UINT8
#define REG16               volatile __nonbanked UINT16
#define REG32               volatile __nonbanked UINT32

#define PTR8                REG8 *
#define PTR16               REG16 *
#define PTR32               REG32 *

/*---------------------------------------------------------------------------------------------------------*/
/* Core dependent MEM definitions                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define MEMR8(a)            (*(PTR8) (a))
#define MEMR16(a)           (*(PTR16)(a))
#define MEMR32(a)           (*(PTR32)(a))

#define MEMW8(a,v)          (*((PTR8)  (a))) = ((UINT8)(v))
#define MEMW16(a,v)         (*((PTR16) (a))) = ((UINT16)(v))
#define MEMW32(a,v)         (*((PTR32) (a))) = ((UINT32)(v))

/*---------------------------------------------------------------------------------------------------------*/
/* Core dependent IO definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define IOR8(a)             MEMR8(a)
#define IOR16(a)            MEMR16(a)
#define IOR32(a)            MEMR32(a)

#define IOW8(a,v)           MEMW8(a, v)
#define IOW16(a,v)          MEMW16(a, v)
#define IOW32(a,v)          MEMW32(a, v)

/*---------------------------------------------------------------------------------------------------------*/
/* Interrupts macros                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define ENABLE_INTERRUPTS()     __enable_interrupt()             /* enable  interrupts                     */
#define DISABLE_INTERRUPTS()    __disable_interrupt()            /* disable interrupts                     */

#define INTERRUPTS_SAVE_DISABLE(var)   DISABLE_INTERRUPTS()      /* Preserve interrupt state and disable   */
#define INTERRUPTS_RESTORE(var)        ENABLE_INTERRUPTS()       /* Restore saved interrupt state          */

#define RST_PND_INTERRUPTS()    __reset_pending_interrupt()      /* reset pending interrupts               */

/*---------------------------------------------------------------------------------------------------------*/
/* Assertion macros                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#ifdef DEBUG
#define ASSERT(cond)  {if (!(cond)) asm("INTR"); }
#else
#define ASSERT(cond)
#endif


#endif //DEFS_COP8_H_
