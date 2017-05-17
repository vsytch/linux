/*
 * $RCSfile$
 * $Revision$
 * $Date$
 * $Author$
 *
 * NPCMX50 On chip ADC driver.
 *
 * (C) 2009-2016 Avocent Corp.
 *
 * This file is subject to the terms and conditions of the GNU
 * General Public License. This program is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 */
#include <asm/ioctl.h>

#if !defined (AESSADCSENSORDRV_H)
#define AESSADCSENSORDRV_H

#ifdef AESSADCSENSORDRV_C

//#include "aess_debugfs.h"
#define ADC_MAX_CHNL_NUM    0xFF /**< define ADC_MAX_CHNL_NUM is 0xff */

#define ADC_DRIVER_NAME "aess_adcdrv"  /**< Name of device node of adc sensor driver. */
/* ioctl definitions */
#define AESS_ADCDRV_IOC_MAGIC       0xBB    /**< I/O magic number of adc driver. */
#define AESS_ADCDRV_R               _IOR(AESS_ADCDRV_IOC_MAGIC, 0, int)  /**< I/O control command: Read value from ADC. */
#define AESS_TEMPDRV_R              _IOR(AESS_ADCDRV_IOC_MAGIC, 1, int)  /**< I/O control command: Read temperature reading from ADC sensor. */

/******************************************************************************
*   STRUCT      :   sADCSensorData
******************************************************************************/
/**
 *  @brief   Structure used by application to pass data parameter to adc sensor driver.
 *
 *****************************************************************************/
typedef struct
{
    /** ADC reading value will be stored in this field. */
    UINT32 u32ADCReading;     

    /** ADC channel number that application want to read. */
    UINT8  u8ADCChannelNum;     

    /** Reserved. */
    UINT8  u8RSV[3];
} sADCSensorData;

/* init flag */
#define AESSADCSENSOR_NOT_INIT    0
#define AESSADCSENSOR_INIT_OK     1

/* ADC base address */
#define ADC_REGS_BASE   NPCMX50_ADC_BASE_ADDR

/* ADC registers */
#define NPCMX50_ADCCON	(void __iomem *) (ADC_REGS_BASE + 0x00)
#define NPCMX50_ADCDATA	(void __iomem *) (ADC_REGS_BASE + 0x04)

/* ADCCON Register Bits */
#define NPCMX50_ADCCON_ADCMUX(x)	(((x) & 0x0F)<<24)
#define NPCMX50_ADCCON_MUXMASK		(0x0F<<24)

#define NPCMX50_ADCCON_ADC_INT_EN	(1<<21)
#define NPCMX50_ADCCON_REFSEL		(1<<19)
#define NPCMX50_ADCCON_ADC_INT		(1<<18)
#define NPCMX50_ADCCON_ADC_EN		(1<<17)
#define NPCMX50_ADCCON_ADC_RST		(1<<16)
#define NPCMX50_ADCCON_ADC_CONV		(1<<13)
#define NPCMX50_ADCCON_ADC_DIV(x)	(((x) & 0xFF)<<24)

#define NPCMX50_ADCCON_ADC_DATA_MASK(x) ((x) & 0x3FF)

/* ADC General Defintion */
#define NPCMX50_ADC_INPUT_CLK_DIV	0
#define NPCMX50_ADC_CONVERT_MAX_RETRY_CNT	1000

#define NPCMX50_ADC_MAX_CHNL_NUM	8

#define NPCMX50_ADC_CHNL0_ADCI0	    0
#define NPCMX50_ADC_CHNL1_ADCI1	    1
#define NPCMX50_ADC_CHNL2_ADCI2	    2
#define NPCMX50_ADC_CHNL3_ADCI3	    3
#define NPCMX50_ADC_CHNL4_ADCI4	    4
#define NPCMX50_ADC_CHNL5_ADCI5	    5
#define NPCMX50_ADC_CHNL6_ADCI6	    6
#define NPCMX50_ADC_CHNL7_ADCI7	    7

#define NPCMX50_IPSRST1  (void __iomem *)  (NPCMX50_CLK_BASE_ADDR + 0x20)

/******************************************************************************
*   STRUCT      :   Function Prototype
******************************************************************************/
/**
 *  @brief   Prototype for each private function.
 *
 *****************************************************************************/

static int aess_adcsensor_read(sADCSensorData *pADCSensorStruct);

#endif   /* AESSADCSENSORDRV_C */

#endif   /* AESSADCSENSORDRV_H */
