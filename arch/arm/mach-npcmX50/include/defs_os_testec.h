/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation Confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2009-2016 by Nuvoton Technology Corporation                                              */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   defs_os_testec.h                                                                                      */
/*            This file contains definitions for TestEC system for (non x86) cores                         */
/* Project:                                                                                                */
/*            SWC DEFS                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef __TESTEC_CORE_H
#define __TESTEC_CORE_H

#include <windows.h>
#include <Test_if.h>

/*---------------------------------------------------------------------------------------------------------*/
/* PSR register fields                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define PSR_E                   9,   1
#define PSR_I                   11,  1

/*---------------------------------------------------------------------------------------------------------*/
/* CFG register fields                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define CFG_DC                  2,  1                       /* Data Cache bit                              */
#define CFG_IC                  4,  1                       /* Instruction Cache bit                       */


/*---------------------------------------------------------------------------------------------------------*/
/* OS dependent types                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
typedef unsigned int            UINT;                       /* Native type of the core that fits the core's*/
typedef int                     INT;                        /* internal registers                          */

/*---------------------------------------------------------------------------------------------------------*/
/* OS dependent PTR definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define REG8                volatile  UINT8
#define REG16               volatile  UINT16
#define REG32               volatile  UINT32

#define PTR8                REG8 *
#define PTR16               REG16 *
#define PTR32               REG32 *

#if defined __TESTEC_PC_DEFAULT__
/*---------------------------------------------------------------------------------------------------------*/
/* Core Memory interface                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define COREMEMR8(a)            CoreReadB((UINT32)(a))
#define COREMEMR16(a)           CoreReadW((UINT32)(a))
#define COREMEMR32(a)           CoreReadD((UINT32)(a))

#define COREMEMW8(a,v)          CoreWriteB((UINT32)(a), (UINT8)(v))
#define COREMEMW16(a,v)         CoreWriteW((UINT32)(a), (UINT16)(v))
#define COREMEMW32(a,v)         CoreWriteD((UINT32)(a), (UINT32)(v))


/*---------------------------------------------------------------------------------------------------------*/
/* PC Physical Memory interface                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define MEMMAP(offset, len)
#define MEMR8(a)                PcMemReadB((UINT32)(a))
#define MEMR16(a)               PcMemReadW((UINT32)(a))
#define MEMR32(a)               PcMemReadD((UINT32)(a))

#define MEMW8(a,v)              PcMemWriteB((UINT32)(a),(UINT8) (v))
#define MEMW16(a,v)             PcMemWriteW((UINT32)(a),(UINT16)(v))
#define MEMW32(a,v)             PcMemWriteD((UINT32)(a),(UINT32)(v))

#elif defined __TESTEC_CORE_DEFAULT__
/*---------------------------------------------------------------------------------------------------------*/
/* Core Memory interface                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define MEMR8(a)                CoreReadB((UINT32)(a))
#define MEMR16(a)               CoreReadW((UINT32)(a))
#define MEMR32(a)               CoreReadD((UINT32)(a))

#define MEMW8(a,v)              CoreWriteB((UINT32)(a), (UINT8)(v))
#define MEMW16(a,v)             CoreWriteW((UINT32)(a), (UINT16)(v))
#define MEMW32(a,v)             CoreWriteD((UINT32)(a), (UINT32)(v))

/*---------------------------------------------------------------------------------------------------------*/
/* PC Physical Memory interface                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define PCMEMMAP(offset, len)
#define PCMEMR8(a)              PcMemReadB((UINT32)(a))
#define PCMEMR16(a)             PcMemReadW((UINT32)(a))
#define PCMEMR32(a)             PcMemReadD((UINT32)(a))

#define PCMEMW8(a,v)            PcMemWriteB((UINT32)(a),(UINT8) (v))
#define PCMEMW16(a,v)           PcMemWriteW((UINT32)(a),(UINT16)(v))
#define PCMEMW32(a,v)           PcMemWriteD((UINT32)(a),(UINT32)(v))

#else

#error "Default TestEC memory access is not specified"

#endif


/*---------------------------------------------------------------------------------------------------------*/
/* PC Virtual memory interface                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define VMEMR8(a)               (*PTR8(a))
#define VMEMR16(a)              (*PTR16(a))
#define VMEMR32(a)              (*PTR32(a))

#define VMEMW8(a,v)             (*(PTR8 (a))) = ((UINT8)(v))
#define VMEMW16(a,v)            (*(PTR16(a))) = ((UINT16)(v))
#define VMEMW32(a,v)            (*(PTR32(a))) = ((UINT32)(v))


/*---------------------------------------------------------------------------------------------------------*/
/* PC IO interface                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define IOR8(a)                 PcIoReadB((UINT32)(a))
#define IOR16(a)                PcIoReadW((UINT32)(a))
#define IOR32(a)                PcIoReadD((UINT32)(a))

#define IOW8(a,v)               PcIoWriteB((UINT32)(a),(UINT8)(v))
#define IOW16(a,v)              PcIoWriteW((UINT32)(a),(UINT16)(v))
#define IOW32(a,v)              PcIoWriteD((UINT32)(a),(UINT32)(v))


/*---------------------------------------------------------------------------------------------------------*/
/* Interrupts macros                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/* Enable interrupts - E and I bits                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define ENABLE_INTERRUPTS()                             \
{                                                       \
    UINT16 tpsr = (UINT16)CoreCpuRegisterRead(CPU_PSR); \
    SET_VAR_FIELD(tpsr, PSR_E, 1);                      \
    SET_VAR_FIELD(tpsr, PSR_I, 1);                      \
    CoreCpuRegisterWrite(CPU_PSR, tpsr);                \
}

/*---------------------------------------------------------------------------------------------------------*/
/* Disable interrupts - E and I bits                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define DISABLE_INTERRUPTS()                            \
{                                                       \
    UINT16 tpsr = (UINT16)CoreCpuRegisterRead(CPU_PSR); \
    SET_VAR_FIELD(tpsr, PSR_E, 0);                      \
    SET_VAR_FIELD(tpsr, PSR_I, 0);                      \
    CoreCpuRegisterWrite(CPU_PSR, tpsr);                \
}

/*---------------------------------------------------------------------------------------------------------*/
/* Preserve interrupt state and disable                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define INTERRUPTS_SAVE_DISABLE(var)                    \
{                                                       \
    var = (UINT16)CoreCpuRegisterRead(CPU_PSR);         \
    CoreDisableInterrupts();                            \
}

/*---------------------------------------------------------------------------------------------------------*/
/* Restore saved interrupt state - E bit only                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define INTERRUPTS_RESTORE(var)                         \
{                                                       \
    if (READ_VAR_FIELD(var, PSR_E))                     \
    {                                                   \
        CoreEnableInterrupts();                         \
    }                                                   \
}

/*---------------------------------------------------------------------------------------------------------*/
/* Cache macros utilities                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define __CACHE_SAVE_DISABLE(var, type)                 \
{                                                       \
    UINT16 tmp = (UINT16)CoreCpuRegisterRead(CPU_CFG);  \
    var = READ_VAR_FIELD(tmp, CFG_##type);              \
    SET_VAR_FIELD(tmp, CFG_##type, 0);                  \
    CoreCpuRegisterWrite(CPU_CFG, tmp);                 \
}

#define __CACHE_SAVE_ENABLE(var, type)                  \
{                                                       \
    UINT16 tmp = (UINT16)CoreCpuRegisterRead(CPU_CFG);  \
    var = READ_VAR_FIELD(tmp, CFG_##type);              \
    SET_VAR_FIELD(tmp, CFG_##type, 1);                  \
    CoreCpuRegisterWrite(CPU_CFG, tmp);                 \
}

#define __CACHE_RESTORE(var, type)                      \
{                                                       \
    UINT16 tmp = (UINT16)CoreCpuRegisterRead(CPU_CFG);  \
    SET_VAR_FIELD(tmp, CFG_##type, var);                \
    CoreCpuRegisterWrite(CPU_CFG, tmp);                 \
}

/*---------------------------------------------------------------------------------------------------------*/
/* Cache macros                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define ICACHE_SAVE_DISABLE(var)    __CACHE_SAVE_DISABLE(var, IC)
#define ICACHE_SAVE_ENABLE(var)     __CACHE_SAVE_ENABLE(var, IC)
#define ICACHE_RESTORE(var)         __CACHE_RESTORE(var, IC)

#define DCACHE_SAVE_DISABLE(var)    __CACHE_SAVE_DISABLE(var, DC)
#define DCACHE_SAVE_ENABLE(var)     __CACHE_SAVE_ENABLE(var, DC)
#define DCACHE_RESTORE(var)         __CACHE_RESTORE(var, DC)

/*---------------------------------------------------------------------------------------------------------*/
/* CPU power management                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define CPU_IDLE()                  CoreWait()

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

#endif /* __TESTEC_CORE_H */

