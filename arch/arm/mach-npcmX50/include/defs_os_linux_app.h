/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation Confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2010-2016 by Nuvoton Technology Corporation                                              */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   defs_os_linux_app.h                                                                                   */
/*            This file contains definitions for Linux OS Applications                                     */
/* Project:                                                                                                */
/*            SWC DEFS                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef _DEFS_LINUX_APP_H_
#define _DEFS_LINUX_APP_H_

/*---------------------------------------------------------------------------------------------------------*/
/* OS dependent types                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
typedef unsigned int        UINT;                       /* Native type of the core that fits the core's    */
typedef int                 INT;                        /* internal registers                              */
typedef UINT                BOOLEAN;

/*---------------------------------------------------------------------------------------------------------*/
/* OS dependent PTR definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define PTR8                (volatile  UINT8  *)
#define PTR16               (volatile  UINT16 *)
#define PTR32               (volatile  UINT32 *)


#endif /* _DEFS_LINUX_APP_H_ */
