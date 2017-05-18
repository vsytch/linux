/**
 *<center>
 *               Avocent Corporation. Proprietary Information.
 * \n<em>
 *      This software is supplied under the terms of a license agreement or
 *      nondisclosure agreement with Avocent Corporation, or its subsidiary, and
 *      may not be copied, disseminated, or distributed except in accordance
 *      with the terms of that agreement.
 *
 *      2001 Gateway Place, Suite 520W, San Jose, California, 95110 U.S.A.
 *\n
 *                  US phone: 408.436.6333
 *
 *        ©2009-2015 Avocent Corporation. All rights reserved.
 *</em> </center>
 *----------------------------------------------------------------------------\n
 *  MODULES     Fan tachometer driver
 *----------------------------------------------------------------------------\n
 *  @file   aess_fansensordrv.h
 *  @brief  The header file of onboard fan tachometer device driver.
 *
 *  @internal
 *----------------------------------------------------------------------------*/
/*
 * NPCM750 On chip fan tach sensor driver.
 *
 * ©2009-2015 Avocent Corporation. All rights reserved.
 *
 * This file is subject to the terms and conditions of the GNU
 * General Public License. This program is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 */

#if !defined (AESSFANSENSORDRV_H)
#define AESSFANSENSORDRV_H

#include "aess_fansensordrv_platform.h"

/* for kernel drivers only */
#ifdef AESSFANSENSORDRV_C

#include <asm/ioctl.h>
//#include <asm/arch/map.h>
#include <asm/types.h>

#define STATUS_OK  0
#define STATUS_FAIL  1

#define NPCM750_MFT_CLKPS   255

/* PLL input */
#define PLL_INPUT_CLOCK  25000000

/* Get Fan Tach Timeout (base on clock 214843.75Hz, 1 cnt = 4.654us)
   Timeout 94ms ~= 0x5000
   (The minimum FAN speed could to support ~640RPM/pulse 1, 320RPM/pulse 2, ...-- 10.6Hz)
 */
 #define FAN_TACH_TIMEOUT   ((UINT16) 0x5000)

/* enable a background timer to poll fan tach value, (200ms * 4) to polling all fan) */
#define FAN_TACH_POLLING_INTERVAL 20  /* 1 = 1 jiffies = 10 ms */

/* MFT General Defintion */
#define NPCM750_MFT_MAX_MODULE	8
#define NPCM750_CMPA	0
#define NPCM750_CMPB	1

#define NPCM750_MFT_MODE_1	0      // PWM and Counter Mode (Not support)
#define NPCM750_MFT_MODE_2	1     // Dual Input Capture Mode (Not support)
#define NPCM750_MFT_MODE_3	2     // Dual Independent Timer Mode (Not support)
#define NPCM750_MFT_MODE_4	3     // Input Capture and Timer Mode (Not support)
#define NPCM750_MFT_MODE_5	4     // Dual Independent Input Capture (support)

#define NPCM750_MFT_TCNT    ((UINT16) 0xFFFF)
#define NPCM750_MFT_TCPA    ((UINT16) (NPCM750_MFT_TCNT - FAN_TACH_TIMEOUT))
#define NPCM750_MFT_TCPB    ((UINT16) (NPCM750_MFT_TCNT - FAN_TACH_TIMEOUT))

#define NPCM750_MFT_NO_CLOCK_MODE		0
#define NPCM750_MFT_APB_CLOCK_MODE		1

#define DEFAULT_PULSE_PER_REVOLUTION	2

//#define MFT_PHY_BASE_ADDR        0xF0180000
//#define MFT_REG_BASE_ADDR        IOMEMORY(MFT_PHY_BASE_ADDR)

/* Fantach MFT base address */
//#define MFT_REGS_BASE(n)    ((n * 0x1000) + MFT_REG_BASE_ADDR)
#define MFT_REGS_BASE(n)    NPCMX50_MFT_BASE_ADDR(n)
#define MFT_MFSEL2_FLSEL(n) (1<<n)

/* Fantach MFT registers */
#define MFT_REG_TCNT1(n)    ((void *) (MFT_REGS_BASE(n) + 0x00))
#define MFT_REG_TCRA(n)     ((void *) (MFT_REGS_BASE(n) + 0x02))
#define MFT_REG_TCRB(n)     ((void *) (MFT_REGS_BASE(n) + 0x04))
#define MFT_REG_TCNT2(n)    ((void *) (MFT_REGS_BASE(n) + 0x06))
#define MFT_REG_TPRSC(n)    ((void *) (MFT_REGS_BASE(n) + 0x08))
#define MFT_REG_TCKC(n)     ((void *) (MFT_REGS_BASE(n) + 0x0A))
#define MFT_REG_TMCTRL(n)   ((void *) (MFT_REGS_BASE(n) + 0x0C))
#define MFT_REG_TICTRL(n)   ((void *) (MFT_REGS_BASE(n) + 0x0E))
#define MFT_REG_TICLR(n)    ((void *) (MFT_REGS_BASE(n) + 0x10))
#define MFT_REG_TIEN(n)     ((void *) (MFT_REGS_BASE(n) + 0x12))
#define MFT_REG_TCPA(n)     ((void *) (MFT_REGS_BASE(n) + 0x14))
#define MFT_REG_TCPB(n)     ((void *) (MFT_REGS_BASE(n) + 0x16))
#define MFT_REG_TCPCFG(n)   ((void *) (MFT_REGS_BASE(n) + 0x18))
#define MFT_REG_TINASEL(n)  ((void *) (MFT_REGS_BASE(n) + 0x1A))
#define MFT_REG_TINBSEL(n)  ((void *) (MFT_REGS_BASE(n) + 0x1C))

#define NPCM750_TCKC_C2CSEL(mode)	    (((mode) & 0x7) << 3)
#define NPCM750_TCKC_C1CSEL(mode)	    ((mode) & 0x7)

#define NPCM750_TMCTRL_TBEN	            (1<<6)
#define NPCM750_TMCTRL_TAEN	            (1<<5)
#define NPCM750_TMCTRL_TBEDG	        (1<<4)
#define NPCM750_TMCTRL_TAEDG	        (1<<3)
#define NPCM750_TMCTRL_MDSEL(mode)	    ((mode) & 0x7)

#define NPCM750_TICLR_CLEAR_ALL         (0x3F)
#define NPCM750_TICLR_TFCLR             (1<<5)
#define NPCM750_TICLR_TECLR             (1<<4)
#define NPCM750_TICLR_TDCLR             (1<<3)
#define NPCM750_TICLR_TCCLR             (1<<2)
#define NPCM750_TICLR_TBCLR             (1<<1)
#define NPCM750_TICLR_TACLR             (1<<0)

#define NPCM750_TIEN_ENABLE_ALL         (0x3F)
#define NPCM750_TIEN_TFIEN              (1<<5)       /* Compare Match for TCNT2 or TCRB */
#define NPCM750_TIEN_TEIEN              (1<<4)       /* Compare Match for TCNT1 or TCRA */
#define NPCM750_TIEN_TDIEN              (1<<3)       /* TCNT2 underflow */
#define NPCM750_TIEN_TCIEN              (1<<2)       /* TCNT1 underflow */
#define NPCM750_TIEN_TBIEN              (1<<1)       /* Input Capture on TBn Transition */
#define NPCM750_TIEN_TAIEN              (1<<0)       /* Input Capture on TAn Transition */

#define NPCM750_TICTRL_TFPND            (1<<5)
#define NPCM750_TICTRL_TEPND            (1<<4)
#define NPCM750_TICTRL_TDPND            (1<<3)
#define NPCM750_TICTRL_TCPND            (1<<2)
#define NPCM750_TICTRL_TBPND            (1<<1)
#define NPCM750_TICTRL_TAPND            (1<<0)

#define NPCM750_TCPCFG_HIBEN            (1<<7)
#define NPCM750_TCPCFG_EQBEN            (1<<6)
#define NPCM750_TCPCFG_LOBEN            (1<<5)
#define NPCM750_TCPCFG_CPBSEL           (1<<4)
#define NPCM750_TCPCFG_HIAEN            (1<<3)
#define NPCM750_TCPCFG_EQAEN            (1<<2)
#define NPCM750_TCPCFG_LOAEN            (1<<1)
#define NPCM750_TCPCFG_CPASEL           (1<<0)

#define NPCM750_TINASEL_FANIN_DEFAULT   (0x0)

#define FAN_TACH_DISABLE                            0xFF
#define FAN_TACH_INIT                               0x00
#define FAN_TACH_PREPARE_TO_GET_FIRST_CAPTURE       0x01
#define FAN_TACH_ENOUGH_SAMPLE                      0x02


/* maximum fan tach input support */
#define NPCM750_MAX_FAN_TACH	 16

/* Obtain the fan number */
#define NPCM750_FAN_TACH_INPUT(mft, cmp)   \
    ((mft << 1) + (cmp))

/*-----------------------------------------------------------------------------*/
/*                                      MFT module                             */
/*-----------------------------------------------------------------------------*/
#define NPCMX50_GLOBAL_CTRL_REG         NPCMX50_GCR_BASE_ADDR

#define GLOBAL_REG_PIN_SELECT2_ADDR     (NPCMX50_GLOBAL_CTRL_REG + 0x10)

/*-----------------------------------------------------------------------------*/
/*                                      AIC module                             */
/*-----------------------------------------------------------------------------*/
/* AIC */
#define AIC_REG_GEN_ADDR    NPCMX50_AIC_BASE_ADDR

/*-----------------------------------------------------------------------------*/
/*                                      CLK_BA module                          */
/*-----------------------------------------------------------------------------*/
/* Clock control (CLK_BA) */
#ifndef CONFIG_OF
#define CLK_BA_REG_BASE_ADDR            NPCMX50_CLK_BASE_ADDR

#define CLK_BA_REG_CLKSEL_ADDR          (CLK_BA_REG_BASE_ADDR + 0x4)
#define CLK_BA_REG_CLKDIV1_ADDR         (CLK_BA_REG_BASE_ADDR + 0x8)
#define CLK_BA_REG_PLLCON0_ADDR         (CLK_BA_REG_BASE_ADDR + 0xc)
#define CLK_BA_REG_PLLCON1_ADDR         (CLK_BA_REG_BASE_ADDR + 0x10)
#define CLK_BA_REG_CLKDIV2_ADDR         (CLK_BA_REG_BASE_ADDR + 0x2C)
#endif

/******************************************************************************
*   STRUCT      :   sFanTachDev
******************************************************************************/
/**
 *  @brief   Structure to FanSensor driver internal usage
 *
 *****************************************************************************/
typedef struct
{
    /** FAN_TACH_DISABLE    0xFF
       FAN_TACH_INIT        0x00
       FAN_TACH_PREPARE_TO_GET_FIRST_CAPTURE  0x01
       ... */
    UINT8 u8FanStatusFlag;

    /** Pulse Revolution */
    UINT8 u8FanPulsePerRev;

    /** Fan Tach Count
       (unit: Base on the APB freq. [default: 55Mhz]
       and prescaler TPRSC   [default: 256]) */
    UINT16 u16FanTachCnt;

    /** Fan Tach Count Temp */
    UINT32 u32FanTachCntTemp;
} sFanTachDev;


/******************************************************************************
*   STRUCT      :   Function Prototype
******************************************************************************/
/**
 *  @brief   Prototype for each private function.
 *
 *****************************************************************************/

static int aess_fansensor_read(sFanTachData *pFanTachData);
static int aess_fansensor_config(sFanTachData *pFanTachData);
static int npcm750_fantach_init(void);

#endif   /* AESSFANSENSORDRV_C */

#endif   /* AESSFANSENSORDRV_H */

/* End of code */
