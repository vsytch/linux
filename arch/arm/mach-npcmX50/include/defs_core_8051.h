/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation Confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2009-2016 by Nuvoton Technology Corporation                                              */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   defs_core_8051.h                                                                                      */
/*            This file contains definitions for 8051 microprocessor core and its compilers                */
/* Project:                                                                                                */
/*            SWC DEFS                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef _DEFS_8051_H_
#define _DEFS_8051_H_

/*---------------------------------------------------------------------------------------------------------*/
/* Core dependent types                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
typedef UINT8           UINT;                           /* Native type of the core that fits the core's    */
typedef INT8            INT;                            /* internal registers                              */
typedef bit             BOOLEAN;

/*---------------------------------------------------------------------------------------------------------*/
/* Core dependent PTR definitions                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define REG8            volatile UINT8  _MEM_TYPE_SLOW_
#define REG16           volatile UINT16 _MEM_TYPE_SLOW_
#define REG32           volatile UINT32 _MEM_TYPE_SLOW_

#define PTR8            REG8 *
#define PTR16           REG16 *
#define PTR32           REG32 *

/*---------------------------------------------------------------------------------------------------------*/
/* Core dependent MEM definitions                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define MEMR8(a)        (*(PTR8) (a))
#define MEMR16(a)       (*(PTR16)(a))
#define MEMR32(a)       (*(PTR32)(a))

#define MEMW8(a,v)      (*((PTR8) (a))) = ((UINT8)(v))
#define MEMW16(a,v)     (*((PTR16)(a))) = ((UINT16)(v))
#define MEMW32(a,v)     (*((PTR32)(a))) = ((UINT32)(v))

/*---------------------------------------------------------------------------------------------------------*/
/* Core dependent IO definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define IOR8(a)         MEMR8(a)
#define IOR16(a)        MEMR16(a)
#define IOR32(a)        MEMR32(a)

#define IOW8(a,v)       MEMW8(a, v)
#define IOW16(a,v)      MEMW16(a, v)
#define IOW32(a,v)      MEMW32(a, v)

/*---------------------------------------------------------------------------------------------------------*/
/* Interrupts macros                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define ENABLE_INTERRUPTS()             EA = 1                   /* enable  interrupts                     */
#define DISABLE_INTERRUPTS()            EA = 0                   /* disable interrupts                     */

#define INTERRUPTS_SAVE_DISABLE(var)    {var = EA; EA=0;}        /* Preserve interrupt state and disable   */
#define INTERRUPTS_RESTORE(var)         {EA = var;}              /* Restore saved interrupt state          */

/*---------------------------------------------------------------------------------------------------------*/
/* Call a function at addr                                                                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define FUNC_CALL(addr)                          (((void (_CONST_TYPE_*)(void))(addr))())

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

/*---------------------------------------------------------------------------------------------------------*/
/* Keil specific                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#if (defined (__C51__) && defined (__KEIL__)) /* 8051 family,KEIL tools                                    */

#define _MEM_TYPE_BIT_        bdata           /* Bit-addressable internal data memory                      */
#define _MEM_TYPE_FAST_       data            /* Directly addressable internal data memory                 */
#define _MEM_TYPE_MEDFAST_    idata           /* Indirectly addressable internal data memory               */
#define _MEM_TYPE_MEDSLOW_    pdata           /* Paged external data memory                                */
#define _MEM_TYPE_SLOW_       xdata           /* External data memory                                      */
#define _CONST_TYPE_          code            /* Program memory                                            */

#endif

#endif  /* _DEFS_8051_H_ */
