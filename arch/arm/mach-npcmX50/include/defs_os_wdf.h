/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation Confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2009-2016 by Nuvoton Technology Corporation                                              */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   defs_os_wdf.h                                                                                         */
/*            This file contains definitions for Windows Driver Foundation API                             */
/* Project:                                                                                                */
/*            SWC DEFS                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef _DEFS_WDF_H_
#define _DEFS_WDF_H_

/*---------------------------------------------------------------------------------------------------------*/
/* OS dependent types                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
typedef unsigned int        UINT;                       /* Native type of the core that fits the core's    */
typedef int                 INT;                        /* internal registers                              */

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
#define MEMR8(a)            (READ_REGISTER_UCHAR(a))
#define MEMR16(a)           (READ_REGISTER_USHORT(a))
#define MEMR32(a)           (READ_REGISTER_ULONG(a))

#define MEMW8(a,v)          WRITE_REGISTER_UCHAR( (a), (v))
#define MEMW16(a,v)         WRITE_REGISTER_USHORT((a), (v))
#define MEMW32(a,v)         WRITE_REGISTER_ULONG( (a), (v))

/*---------------------------------------------------------------------------------------------------------*/
/* OS dependent IO definitions                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define IOR8(a)             (READ_PORT_UCHAR((PUCHAR)(a)))
#define IOR16(a)            (READ_PORT_USHORT(a))
#define IOR32(a)            (READ_PORT_ULONG(a))

#define IOW8(a,v)           WRITE_PORT_UCHAR((PUCHAR)(a),(v))
#define IOW16(a,v)          WRITE_PORT_USHORT((a),(v))
#define IOW32(a,v)          WRITE_PORT_ULONG((a),(v))

/*---------------------------------------------------------------------------------------------------------*/
/* Interrupts macros                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define ENABLE_INTERRUPTS()             __UNDEFINED__
#define DISABLE_INTERRUPTS()            __UNDEFINED__

#define INTERRUPTS_SAVE_DISABLE(var)    __UNDEFINED__
#define INTERRUPTS_RESTORE(var)         __UNDEFINED__


#endif /* _DEFS_WDF_H_ */
