/*
 *
 * Copyright (C) 2009,2010 Avocent Corporation
 *
 * This file is subject to the terms and conditions of the GNU
 * General Public License Version 2. This program is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License Version 2 for more details.
 *

 *----------------------------------------------------------------------------\n
 *  MODULES     aess pwm driver\n
 *----------------------------------------------------------------------------\n
 * @file    aess_pwmdrv.h
 * @brief   PWM driver header file
 *----------------------------------------------------------------------------
 */

#include <asm/ioctl.h>

#if !defined (AESSPWMDRV_H)
#define AESSPWMDRV_H

#ifdef AESSPWMDRV_C

/* type define */
typedef unsigned char       UINT8;
typedef unsigned short      UINT16;
typedef unsigned int        UINT32;

/* TBD: For temporary use, because no type.h now */
#define STATUS_OK     0
#define STATUS_FAIL   1

/* ioctl definitions */
#define AESS_PWMDRV_IOC_MAGIC           0xBE

#define AESS_PWM_CONFIG_INIT             _IOWR(AESS_PWMDRV_IOC_MAGIC, 0, sPWMDevConfig)
#define AESS_PWM_CONFIG_SET              _IOWR(AESS_PWMDRV_IOC_MAGIC, 1, sPWMDevConfig)
#define AESS_PWM_CONFIG_INFO             _IOWR(AESS_PWMDRV_IOC_MAGIC, 2, sPWMDevConfig)
#define AESS_PWM_CONFIG_DEBUG            _IOWR(AESS_PWMDRV_IOC_MAGIC, 3, sPWMDevConfig)

/* init flag */
#define AESSPWM_NOT_INIT 0
#define AESSPWM_INIT_OK  1

/* PWM ABP clock */
#define NPCMX50_APB_CLOCK			50000 //50kHz
/* PDID */

/* PWM port base address */  
#define NPCMX50_GLOBAL_CTRL_REG    NPCMX50_GCR_BASE_ADDR

#define GLOBAL_REG_PDID_REG          (NPCMX50_GLOBAL_CTRL_REG + 0x0)
#define GLOBAL_REG_PIN_SELECT2_ADDR  (NPCMX50_GLOBAL_CTRL_REG + 0x10)

#define PWM_REG_PRESCALE_ADDR(n)    	            (pwm_base[n] + 0)
#define PWM_REG_CLOCK_SELECTOR_ADDR(n) 	            (pwm_base[n] + 0x4)
#define PWM_REG_CONTROL_ADDR(n)        	            (pwm_base[n] + 0x8)
#define PWM_REG_COUNTER_ADDR(n, PORT)               (pwm_base[n] + 0xc + (12 * PORT))
#define PWM_REG_COMPARATOR_ADDR(n, PORT)            (pwm_base[n] + 0x10 + (12 * PORT))
#define PWM_REG_DATA_ADDR(n, PORT)    	            (pwm_base[n] + 0x14 + (12 * PORT))
#define PWM_REG_TIMER_INT_ENABLE_ADDR(n)            (pwm_base[n] + 0x3c)
#define PWM_REG_TIMER_INT_IDENTIFICATION_ADDR(n)    (pwm_base[n] + 0x40)

#define PWM_CTRL_CH0_MODE_BIT 		3
#define PWM_CTRL_CH1_MODE_BIT 		11
#define PWM_CTRL_CH2_MODE_BIT 		15
#define PWM_CTRL_CH3_MODE_BIT 		19

#define PWM_CTRL_CH0_INV_BIT 		2
#define PWM_CTRL_CH1_INV_BIT 		10
#define PWM_CTRL_CH2_INV_BIT 		14
#define PWM_CTRL_CH3_INV_BIT 		18

#define PWM_CTRL_CH0_ENABLE_BIT 	0
#define PWM_CTRL_CH1_ENABLE_BIT 	8
#define PWM_CTRL_CH2_ENABLE_BIT 	12
#define PWM_CTRL_CH3_ENABLE_BIT 	16

#define PWM_CLOCK_SELECTOR_MASK 	0x7
#define PWM_CLOCK_CH0_SELECTOR_BIT 	0
#define PWM_CLOCK_CH1_SELECTOR_BIT 	4
#define PWM_CLOCK_CH2_SELECTOR_BIT 	8
#define PWM_CLOCK_CH3_SELECTOR_BIT 	12

#define PWM_PRESCALE_MASK 			0xff
#define PWM_PRESCALE_CH01_BIT 		0
#define PWM_PRESCALE_CH23_BIT 		8

#define PWM_PIN_SELECT_CH0_BIT 		16
#define PWM_PIN_SELECT_CH0_GPIO_NUM 80
#define PWM_PIN_ENABLE_VALUE 		1

/* GPIO command type */
#define GPIO_CONFIG    0
#define GPIO_WRITE     1
#define GPIO_READ      2

#define GPIO_SELECTED_OUTPUT     	0x9

/* Define the maximum PWM channel number */
#define PWM_MAX_CHN_NUM  			8
#define PWM_MAX_CHN_NUM_IN_A_MODULE	4

/* Define the Counter Register, value = 100 for match 100% */
#define PWM_COUNTER_DEFALUT0_NUM  	63
#define PWM_COUNTER_DEFALUT1_NUM  	255
#define PWM_COMPARATOR_DEFALUT_NUM  50
#define PWM_CLOCK_SELE_DEFALUT_NUM  4
#define PWM_PRESCALE_DEFALUT_NUM  	1

/******************************************************************************
*   STRUCT      :   sPWMDevConfig
******************************************************************************/
/**
 *  @brief   Structure to PWM driver config.
 *
 *****************************************************************************/
typedef struct
{    
	/* PWM Channel number */
	UINT8 u8PWMChannelNum;
	
	/* PWM Base Cycle Frequency */
	UINT8 u8PWMBaseCycleFrequency;
	
	/* PWM Frequency Divider */
	UINT8 u8PWMFrequencyDivider; 

	/* PWM Duty Cycle */
	UINT8 u8PWMDutyCycle; 
	
} sPWMDevConfig;

static int aess_pwm_config_init(sPWMDevConfig *PWMDevConfig);
static int aess_pwm_config_set(sPWMDevConfig *PWMDevConfig);
static int aess_pwm_config_info(sPWMDevConfig *PWMDevConfig);
static int aess_pwm_config_debug(sPWMDevConfig *PWMDevConfig);

#endif   /* AESSPWMDRV_C */
#endif   /* AESSPWMDRV_H */
