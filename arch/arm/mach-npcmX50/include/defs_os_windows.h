/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation Confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2009-2016 by Nuvoton Technology Corporation                                              */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   defs_os_windows.h                                                                                     */
/*            This file contains definitions for Windows OS Applications                                   */
/* Project:                                                                                                */
/*            SWC DEFS                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef _DEFS_WINDOWS_H_
#define _DEFS_WINDOWS_H_

#include <windows.h>

/*---------------------------------------------------------------------------------------------------------*/
/* OS dependent types                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
typedef unsigned int        UINT;                       /* Native type of the core that fits the core's    */
typedef int                 INT;                        /* internal registers                              */

/* Windows already define BOOLEAN basic type - no need to define it here                                   */

/*---------------------------------------------------------------------------------------------------------*/
/* OS dependent PTR definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define REG8                volatile  UINT8  *
#define REG16               volatile  UINT16 *
#define REG32               volatile  UINT32 *

#define PTR8                REG8 *
#define PTR16               REG16 *
#define PTR32               REG32 *


/*---------------------------------------------------------------------------------------------------------*/
/* Variables alignment                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define _ALIGN_(x, decl)                        __declspec(align(x))  decl

/*---------------------------------------------------------------------------------------------------------*/
/* Variables packing                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define _PACK_(decl)                            decl

#endif /* _DEFS_WINDOWS_H_ */
