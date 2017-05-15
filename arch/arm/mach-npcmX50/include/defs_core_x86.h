/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation Confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2009-2016 by Nuvoton Technology Corporation                                              */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   defs_core_x86.h                                                                                       */
/*            This file contains definitions for x86 core and its compilers                                */
/* Project:                                                                                                */
/*            SWC DEFS                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef _DEFS_X86_H_
#define _DEFS_X86_H_

#include <conio.h>
#include <dos.h>

/*---------------------------------------------------------------------------------------------------------*/
/* Core dependent types                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
typedef UINT32              UINT;                       /* Native type of the core that fits the core's    */
typedef INT32               INT;                        /* internal registers                              */
typedef UINT                BOOLEAN;

/*---------------------------------------------------------------------------------------------------------*/
/* Core dependent PTR definitions                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define FAR              _far

#define REG8                volatile UINT8  FAR
#define REG16               volatile UINT16 FAR
#define REG32               volatile UINT32 FAR

#define PTR8                REG8 *
#define PTR16               REG16 *
#define PTR32               REG32 *

/*---------------------------------------------------------------------------------------------------------*/
/* Core dependent MEM definitions                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define MEMMAP(offset, len)

#define MEMR8(a)         (*(PTR8)(a))
#define MEMR16(a)        (*(PTR16)(a))
#define MEMR32(a)        (*(PTR32)(a))

#define MEMW8(a,v)       (*((PTR8) (a))) = ((UINT8)(v))
#define MEMW16(a,v)      (*((PTR16)(a))) = ((UINT16)(v))
#define MEMW32(a,v)      (*((PTR32)(a))) = ((UINT32)(v))

/*---------------------------------------------------------------------------------------------------------*/
/* Core dependent IO definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define IOR8(a)         (inp(a))
#define IOR16(a)        (inpw(a))
#define IOR32(a)        (inpd(a))

#define IOW8(a,v)       (outp( (a),(v)))
#define IOW16(a,v)      (outpw((a),(v)))
#define IOW32(a,v)      (outpd((a),(v)))

/*---------------------------------------------------------------------------------------------------------*/
/* Interrupts macros                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define ENABLE_INTERRUPTS()            _enable()                 /* enable  interrupts                     */
#define DISABLE_INTERRUPTS()           _disable()                /* disable interrupts                     */

#define INTERRUPTS_SAVE_DISABLE(var)   DISABLE_INTERRUPTS()      /* Preserve interrupt state and disable   */
#define INTERRUPTS_RESTORE(var)        ENABLE_INTERRUPTS()       /* Restore saved interrupt state          */

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
#endif /* _DEFS_X86_H_ */
