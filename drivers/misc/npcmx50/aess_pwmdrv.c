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
 * @file    aess_pwmdrv.c
 * @brief   PWM driver
 *----------------------------------------------------------------------------
 */
/******************************************************************************
 * Content
 * ----------------
 *	  show_value()	- Used for sysfs reading.
 *    set_value()	- Used for sysfs writing.
 *    aess_pwm_config_init() -	PWMX ioctl config initializtion function
 *    aess_pwm_config_set() -   PWMX ioctl config set function
 *    aess_pwm_config_info -	PWM config info
 *    aess_pwm_inverter() -  	PWM inverter configuration
 *    aess_pwm_config_debug() - PWM configuration debug
 *    aess_pwm_open() -		pwm driver open node function
 *    aess_pwm_release() -	pwm driver realase function
 *    aess_pwm_ioctl() -	pwm driver ioctl function
 *    aess_pwm_init() -		pwm init function (called when insert module)
 *    aess_pwm_exit() -		pwm exit function (called when remove module)
*******************************************************************************/

#define AESSPWMDRV_C

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#ifdef CONFIG_OF
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif


#include <asm/uaccess.h>
#include <asm/io.h>
#include <mach/hal.h>

#include "aess_pwmdrv.h"

//#define PWM_DEBUG

#define DRIVER_NAME "aess_pwmdrv"


// Uncomment one of the following to determine whether PWMs go to maximum
// speed or are turned off upon driver "release."
// Comment out both for PWMs to maintain previous programmed speed.
//#define PWM_MAX_SPEED_ON_RELEASE      // Takes precedence
//#define PWM_OFF_ON_RELEASE

/* Debugging Message */
#ifdef PWM_DEBUG
#define DEBUG_MSG(fmt, args...)  printk("PWM: %s() " fmt, __func__ , ##args)
#else
#define DEBUG_MSG(fmt, args...)
#endif
#define PERROR(fmt, args...)  printk("PWM: %s() " fmt, __func__ , ##args)

static int majornum = 0;           /* default to dynamic major */
module_param(majornum, int, 0);
MODULE_PARM_DESC(majornum, "Major device number");

/* driver name will passed by insmod command */
static char *driver_name = "aess_pwmdrv"; 

/* The init flag for aess_pwm_open */
static UINT8 S_u8init_flag = AESSPWM_NOT_INIT;

/* Linux device data structure */
dev_t pwm_dev;
struct cdev *pwm_cdev;
struct clk *pwm_clk;
struct platform_device *pwm_pdev = NULL;


static const struct of_device_id pwm_dt_id[];
static struct platform_driver npcm750_pwm_driver;

void __iomem *pwm_base[NPCMX50_PWM_MAX_MODULES];

static int npcmx50_pwm_probe(struct platform_device *pdev);




UINT8 u8InitialPWM[PWM_MAX_CHN_NUM];

/******************************************************************************
 *   FUNCTION        :   aess_pwm_config_init
 ******************************************************************************/
/** @brief 	pass configuration to PWMX driver to do initialization
 * @return	STATUS_OK when success, otherwise when fail
 * @dependency     none
 * @limitation     none
 * @warning        none
 * @note           none
 * @internal       Function type: Local function \n
 ******************************************************************************/
static int aess_pwm_config_init(sPWMDevConfig *PWMDevConfig)
{
	UINT8  u8PWMChannel = (PWMDevConfig->u8PWMChannelNum % PWM_MAX_CHN_NUM_IN_A_MODULE);
	UINT32 u32TmpBuf = 0, u32TmpBuf2 = 0, u32TmpBuf3 = 0;
	UINT32 u32DestAddr = 0, u32CtrlAddr = 0, u32PrescaleAddr = 0, u32CSelectorAddr = 0;
	UINT32 n_module = PWMDevConfig->u8PWMChannelNum/PWM_MAX_CHN_NUM_IN_A_MODULE;

	//Set initial PWM value
	// DF286034 	IDRAC6 racadm reset causes a pnp removal of the perc6i card. Event ID
	u8InitialPWM[PWMDevConfig->u8PWMChannelNum] = (PWMDevConfig->u8PWMDutyCycle & 0xFF);

	/**********************************************************************
	 * Config PWMD register for setting frequency divider during initialize
	 **********************************************************************/

	/* get PWM register address */
	u32PrescaleAddr = (UINT32)PWM_REG_PRESCALE_ADDR(n_module);
	u32CSelectorAddr = (UINT32)PWM_REG_CLOCK_SELECTOR_ADDR(n_module);
	u32CtrlAddr = (UINT32)PWM_REG_CONTROL_ADDR(n_module);

	/* read PWMD value */
	u32TmpBuf = ioread32((void *) u32PrescaleAddr);
	u32TmpBuf2 = ioread32((void *) u32CSelectorAddr);
	u32TmpBuf3 = ioread32((void *) u32CtrlAddr);

	switch (u8PWMChannel)
	{
		case 0:
			/* set prescale bit[7:0]
					so far, default value is 127 */
			u32TmpBuf &= ~(PWM_PRESCALE_MASK << PWM_PRESCALE_CH01_BIT);
			u32TmpBuf |= ((PWM_PRESCALE_DEFALUT_NUM & PWM_PRESCALE_MASK) << PWM_PRESCALE_CH01_BIT);

			/* set clock selector bit [2:0] */
			u32TmpBuf2 &= ~(PWM_CLOCK_SELECTOR_MASK << PWM_CLOCK_CH0_SELECTOR_BIT);
			u32TmpBuf2 |=
				((PWMDevConfig->u8PWMBaseCycleFrequency & PWM_CLOCK_SELECTOR_MASK) << PWM_CLOCK_CH0_SELECTOR_BIT);

			/* set Toggle mode */
			u32TmpBuf3 |= (1 << PWM_CTRL_CH0_MODE_BIT);
			break;
		case 1:
			/* set prescale bit[7:0]
					so far, default value is 127 */
			u32TmpBuf &= ~(PWM_PRESCALE_MASK << PWM_PRESCALE_CH01_BIT);
			u32TmpBuf |= ((PWM_PRESCALE_DEFALUT_NUM & PWM_PRESCALE_MASK) << PWM_PRESCALE_CH01_BIT);

			/* set clock selector bit [5:4] */
			u32TmpBuf2 &= ~(PWM_CLOCK_SELECTOR_MASK << PWM_CLOCK_CH1_SELECTOR_BIT);
			u32TmpBuf2 |=
				((PWMDevConfig->u8PWMBaseCycleFrequency & PWM_CLOCK_SELECTOR_MASK) << PWM_CLOCK_CH1_SELECTOR_BIT);

			/* set Toggle mode */
			u32TmpBuf3 |= (1 << PWM_CTRL_CH1_MODE_BIT);
			break;
		case 2:
			/* set prescale bit[7:0]
					so far, default value is 127 */
			u32TmpBuf &= ~(PWM_PRESCALE_MASK << PWM_PRESCALE_CH23_BIT);
			u32TmpBuf |= ((PWM_PRESCALE_DEFALUT_NUM & PWM_PRESCALE_MASK) << PWM_PRESCALE_CH23_BIT);

			/* set clock selector bit [5:4] */
			u32TmpBuf2 &= ~(PWM_CLOCK_SELECTOR_MASK << PWM_CLOCK_CH2_SELECTOR_BIT);
			u32TmpBuf2 |=
				((PWMDevConfig->u8PWMBaseCycleFrequency & PWM_CLOCK_SELECTOR_MASK) << PWM_CLOCK_CH2_SELECTOR_BIT);

			/* set Toggle mode */
			u32TmpBuf3 |= (1 << PWM_CTRL_CH2_MODE_BIT);
			break;
		case 3:
			/* set prescale bit[7:0]
					so far, default value is 127 */
			u32TmpBuf &= ~(PWM_PRESCALE_MASK << PWM_PRESCALE_CH23_BIT);
			u32TmpBuf |= ((PWM_PRESCALE_DEFALUT_NUM & PWM_PRESCALE_MASK) << PWM_PRESCALE_CH23_BIT);

			/* set clock selector bit [5:4] */
			u32TmpBuf2 &= ~(PWM_CLOCK_SELECTOR_MASK << PWM_CLOCK_CH3_SELECTOR_BIT);
			u32TmpBuf2 |=
				((PWMDevConfig->u8PWMBaseCycleFrequency & PWM_CLOCK_SELECTOR_MASK) << PWM_CLOCK_CH3_SELECTOR_BIT);

			/* set Toggle mode */
			u32TmpBuf3 |= (1 << PWM_CTRL_CH3_MODE_BIT);
			break;
		default:
			return -ENODEV;
			break;
	}

	DEBUG_MSG("u8PWMChannel=%d, u32CSelectorAddr=0x%x, u8PWMBaseCycleFrequency=%d, PWMDTmpBuf=0x%x \n",
		PWMDevConfig->u8PWMChannelNum, u32CSelectorAddr, PWMDevConfig->u8PWMBaseCycleFrequency, u32TmpBuf2);

	/* write new PWM register value  */
	iowrite32(u32TmpBuf, (void *) u32PrescaleAddr);
	iowrite32(u32TmpBuf2, (void *) u32CSelectorAddr);
	iowrite32(u32TmpBuf3, (void *) u32CtrlAddr);

	/****************************************************************
	 * Config PWM Counter register for setting resolution
	 ****************************************************************/
	u32DestAddr = (UINT32)PWM_REG_COUNTER_ADDR(n_module, u8PWMChannel);

	u32TmpBuf = (UINT32)(PWMDevConfig->u8PWMFrequencyDivider & 0xFF);

	DEBUG_MSG("u8PWMChannel=%d, u32CSelectorAddr=0x%x, u8PWMFrequencyDivider=%d, PWMDTmpBuf=%x \n",
		PWMDevConfig->u8PWMChannelNum, u32DestAddr, PWMDevConfig->u8PWMFrequencyDivider, (unsigned int)u32TmpBuf);

	/* write new PWMC value  */
	iowrite32(u32TmpBuf, (void *) u32DestAddr);

	/****************************************************************
	 * Config PWM Comparator register for setting duty cycle
	 ****************************************************************/
	u32DestAddr = (UINT32)PWM_REG_COMPARATOR_ADDR(n_module, u8PWMChannel);

	u32TmpBuf = (UINT32)(PWMDevConfig->u8PWMDutyCycle & 0xFF);

	DEBUG_MSG("u8PWMChannel=%d, u32CSelectorAddr=0x%x, u8PWMDutyCycle=%d, PWMDTmpBuf=%x \n",
		PWMDevConfig->u8PWMChannelNum, u32DestAddr, PWMDevConfig->u8PWMDutyCycle, (unsigned int)u32TmpBuf);

	/* write new PWMC value  */
	iowrite32(u32TmpBuf, (void *) u32DestAddr);

	/****************************************************************
	 * Enable PWM
	 ****************************************************************/
	u32TmpBuf3 = ioread32((void *) u32CtrlAddr);

	switch (u8PWMChannel)
	{
		case 0:
			//Enable channel
			u32TmpBuf3 |= (1 << PWM_CTRL_CH0_ENABLE_BIT);
			break;
		case 1:
			//Enable channel
			u32TmpBuf3 |= (1 << PWM_CTRL_CH1_ENABLE_BIT);
			break;
		case 2:
			//Enable channel
			u32TmpBuf3 |= (1 << PWM_CTRL_CH2_ENABLE_BIT);
			break;
		case 3:
			//Enable channel
			u32TmpBuf3 |= (1 << PWM_CTRL_CH3_ENABLE_BIT);
			break;
		default:
			return -ENODEV;
			break;
	}

	/* write new PWM value  */
	iowrite32(u32TmpBuf3, (void *) u32CtrlAddr);

	return (STATUS_OK);
}


/******************************************************************************
 *   FUNCTION        :   aess_pwm_config_set
 ******************************************************************************/
/** @brief  	set PWMX duty cycle
 * @return	STATUS_OK when success, otherwise when fail
 * @dependency     none
 * @limitation     none
 * @warning        none
 * @note           none
 * @internal       Function type: Local function \n
 ******************************************************************************/
static int aess_pwm_config_set(sPWMDevConfig *PWMDevConfig)
{
	UINT8  u8PWMChannel = (PWMDevConfig->u8PWMChannelNum % PWM_MAX_CHN_NUM_IN_A_MODULE);
	UINT32 u32TmpBuf = 0;
	UINT32 u32DestAddr = 0;
	UINT32 n_module = PWMDevConfig->u8PWMChannelNum/PWM_MAX_CHN_NUM_IN_A_MODULE;

	/****************************************************************
	 * Config PWM Comparator register for setting duty cycle
	 ****************************************************************/
	u32DestAddr = (UINT32)PWM_REG_COMPARATOR_ADDR(n_module, u8PWMChannel);

	u32TmpBuf = (UINT32)(PWMDevConfig->u8PWMDutyCycle & 0xFF);

	DEBUG_MSG("u8PWMChannel=%d, u32CSelectorAddr=0x%x, u8PWMDutyCycle=%d, PWMDTmpBuf=%x \n",
			  PWMDevConfig->u8PWMChannelNum, u32DestAddr, PWMDevConfig->u8PWMDutyCycle, (unsigned int)u32TmpBuf);

	/* write new PWMC value  */
	iowrite32(u32TmpBuf, (void *) u32DestAddr);

// Added by DIJIC to diable output drive when 100% PWM set

	u32DestAddr = (UINT32)GLOBAL_REG_PIN_SELECT2_ADDR;
	u32TmpBuf = ioread32((void *) u32DestAddr);

	if (0 == (PWMDevConfig->u8PWMDutyCycle & 0xFF))
	{
		/****************************************************************
		 * Disable PWM
		 ****************************************************************/
		u32TmpBuf &= ~(1 << (PWM_PIN_SELECT_CH0_BIT + PWMDevConfig->u8PWMChannelNum));

	}
	else
	{
		/****************************************************************
		 * Enable PWM
		 ****************************************************************/
		u32TmpBuf |= 1 << (PWM_PIN_SELECT_CH0_BIT + PWMDevConfig->u8PWMChannelNum);

	};

	DEBUG_MSG("u8PWMChannel=%d, u32CSelectorAddr=0x%x, (Output Enable) u32TmpBuf=%x \n",
	PWMDevConfig->u8PWMChannelNum, u32DestAddr, (unsigned int)u32TmpBuf);

	/****************************************************************
	 * Enable PWM PIN
	 ****************************************************************/
	iowrite32(u32TmpBuf, (void *) u32DestAddr);

	return (STATUS_OK);
}


/******************************************************************************
 *   FUNCTION        :   aess_pwm_config_info
 ******************************************************************************/
/** @brief  	get PWMX config info
 * @return	STATUS_OK when success, otherwise when fail
 * @dependency     none
 * @limitation     none
 * @warning        none
 * @note           none
 * @internal       Function type: Local function \n
 ******************************************************************************/
static int aess_pwm_config_info(sPWMDevConfig *PWMDevConfig)
{
	UINT8  u8PWMChannel = (PWMDevConfig->u8PWMChannelNum % PWM_MAX_CHN_NUM_IN_A_MODULE);
	UINT32 n_module = PWMDevConfig->u8PWMChannelNum/PWM_MAX_CHN_NUM_IN_A_MODULE;
	UINT32 u32TmpBuf = 0, u32TmpBuf2 = 0, u32TmpBuf3 = 0;
	UINT32 u32DestAddr = 0, u32CtrlAddr = 0, u32PrescaleAddr = 0, u32CSelectorAddr = 0;
	UINT32 shitNum = 0;

	/****************************************************************
	 * Debug using
	 ****************************************************************/

	/* get PWM register address */
	u32PrescaleAddr = (UINT32)PWM_REG_PRESCALE_ADDR(n_module);
	u32CSelectorAddr = (UINT32)PWM_REG_CLOCK_SELECTOR_ADDR(n_module);
	u32CtrlAddr = (UINT32)PWM_REG_CONTROL_ADDR(n_module);

	/* read PWMD value */
	u32TmpBuf = ioread32((void *) u32PrescaleAddr);
	u32TmpBuf2 = ioread32((void *) u32CSelectorAddr);
	u32TmpBuf3 = ioread32((void *) u32CtrlAddr);

	printk("*** u8PWMChannel=%d, u32PrescaleAddr[0x%x]=0x%x \n",
		   PWMDevConfig->u8PWMChannelNum, (unsigned int)u32PrescaleAddr, (unsigned int)u32TmpBuf);

	if ((PWMDevConfig->u8PWMChannelNum % 4) < 2)
		printk("=== PrescaleAddr=0x%x \n\n", ((unsigned int)u32TmpBuf & 0xFF));
	else
		printk("=== PrescaleAddr=0x%x \n\n",(((unsigned int)u32TmpBuf >> 8) & 0xFF));

	printk("*** u32CSelectorAddr[0x%x]=0x%x \n", (unsigned int)u32CSelectorAddr, (unsigned int)u32TmpBuf2);

	shitNum = ((PWMDevConfig->u8PWMChannelNum % 4) * 4) & 0x7;
	printk("=== CSR =0x%x \n\n", ((unsigned int)u32TmpBuf2 >> shitNum));

	printk("*** u32CtrlAddr[0x%x]=0x%x \n", (unsigned int)u32CtrlAddr, (unsigned int)u32TmpBuf3);

	u32DestAddr = (UINT32)PWM_REG_COUNTER_ADDR(n_module, u8PWMChannel);
	u32TmpBuf = ioread32((void *) u32DestAddr);
	printk("*** COUNTER_ADDR[0x%x]=0x%x \n", (unsigned int)u32DestAddr, (unsigned int)u32TmpBuf);

	u32DestAddr = (UINT32)PWM_REG_COMPARATOR_ADDR(n_module, u8PWMChannel);
	u32TmpBuf = ioread32((void *) u32DestAddr);
	printk("*** Duty Cycle - CMR[0x%x]=0x%x \n", (unsigned int)u32DestAddr, (unsigned int)u32TmpBuf);

	u32DestAddr = (UINT32)GLOBAL_REG_PIN_SELECT2_ADDR;
	u32TmpBuf = ioread32((void *) u32DestAddr);
	printk("*** SELECT2_ADDR[0x%x]=0x%x \n", (unsigned int)u32DestAddr, (unsigned int)u32TmpBuf);

	return (STATUS_OK);
}

/******************************************************************************
 *   FUNCTION        :   aess_pwm_config_debug
 ******************************************************************************/
/** @brief  	debug PWM
 * @return	STATUS_OK when success, otherwise when fail
 * @dependency     none
 * @limitation     none
 * @warning        none
 * @note           none
 * @internal       Function type: Local function \n
 ******************************************************************************/
static int aess_pwm_config_debug(sPWMDevConfig *PWMDevConfig)
{
	UINT8  u8PWMChannel = (PWMDevConfig->u8PWMChannelNum % PWM_MAX_CHN_NUM_IN_A_MODULE);
	UINT32 u32TmpBuf = 0, u32TmpBuf2 = 0, u32TmpBuf3 = 0;
	UINT32 u32DestAddr = 0, u32CtrlAddr = 0, u32PrescaleAddr = 0, u32CSelectorAddr = 0;
	UINT32 n_module = PWMDevConfig->u8PWMChannelNum/PWM_MAX_CHN_NUM_IN_A_MODULE;

	/* get PWM register address */
	u32PrescaleAddr = (UINT32)PWM_REG_PRESCALE_ADDR(n_module);
	u32CSelectorAddr = (UINT32)PWM_REG_CLOCK_SELECTOR_ADDR(n_module);
	u32CtrlAddr = (UINT32)PWM_REG_CONTROL_ADDR(n_module);

	/* read PWMD value */
	u32TmpBuf = ioread32((void *) u32PrescaleAddr);
	u32TmpBuf2 = ioread32((void *) u32CSelectorAddr);
	u32TmpBuf3 = ioread32((void *) u32CtrlAddr);

	switch (u8PWMChannel)
	{
		case 0:
			/* set prescale bit[7:0]
					so far, default value is 127 */
			u32TmpBuf &= ~(PWM_PRESCALE_MASK << PWM_PRESCALE_CH01_BIT);
			u32TmpBuf |= ((PWMDevConfig->u8PWMDutyCycle & PWM_PRESCALE_MASK) << PWM_PRESCALE_CH01_BIT);

			/* set clock selector bit [2:0] */
			u32TmpBuf2 &= ~(PWM_CLOCK_SELECTOR_MASK << PWM_CLOCK_CH0_SELECTOR_BIT);
			u32TmpBuf2 |=
				((PWMDevConfig->u8PWMBaseCycleFrequency & PWM_CLOCK_SELECTOR_MASK) << PWM_CLOCK_CH0_SELECTOR_BIT);

			/* set Toggle mode */
			u32TmpBuf3 |= (1 << PWM_CTRL_CH0_MODE_BIT);
			break;
		case 1:
			/* set prescale bit[7:0]
					so far, default value is 127 */
			u32TmpBuf &= ~(PWM_PRESCALE_MASK << PWM_PRESCALE_CH01_BIT);
			u32TmpBuf |=
				((PWMDevConfig->u8PWMDutyCycle & PWM_PRESCALE_MASK) << PWM_PRESCALE_CH01_BIT);

			/* set clock selector bit [5:4] */
			u32TmpBuf2 &= ~(PWM_CLOCK_SELECTOR_MASK << PWM_CLOCK_CH1_SELECTOR_BIT);
			u32TmpBuf2 |=
				((PWMDevConfig->u8PWMBaseCycleFrequency & PWM_CLOCK_SELECTOR_MASK) << PWM_CLOCK_CH1_SELECTOR_BIT);

			/* set Toggle mode */
			u32TmpBuf3 |= (1 << PWM_CTRL_CH1_MODE_BIT);
			break;
		case 2:
			/* set prescale bit[7:0]
					so far, default value is 127 */
			u32TmpBuf &= ~(PWM_PRESCALE_MASK << PWM_PRESCALE_CH23_BIT);
			u32TmpBuf |= ((PWMDevConfig->u8PWMDutyCycle & PWM_PRESCALE_MASK) << PWM_PRESCALE_CH23_BIT);

			/* set clock selector bit [5:4] */
			u32TmpBuf2 &= ~(PWM_CLOCK_SELECTOR_MASK << PWM_CLOCK_CH2_SELECTOR_BIT);
			u32TmpBuf2 |=
				((PWMDevConfig->u8PWMBaseCycleFrequency & PWM_CLOCK_SELECTOR_MASK) << PWM_CLOCK_CH2_SELECTOR_BIT);

			/* set Toggle mode */
			u32TmpBuf3 |= (1 << PWM_CTRL_CH2_MODE_BIT);
			break;
		case 3:
			/* set prescale bit[7:0]
					so far, default value is 127 */
			u32TmpBuf &= ~(PWM_PRESCALE_MASK << PWM_PRESCALE_CH23_BIT);
			u32TmpBuf |= ((PWMDevConfig->u8PWMDutyCycle & PWM_PRESCALE_MASK) << PWM_PRESCALE_CH23_BIT);

			/* set clock selector bit [5:4] */
			u32TmpBuf2 &= ~(PWM_CLOCK_SELECTOR_MASK << PWM_CLOCK_CH3_SELECTOR_BIT);
			u32TmpBuf2 |=
				((PWMDevConfig->u8PWMBaseCycleFrequency & PWM_CLOCK_SELECTOR_MASK) << PWM_CLOCK_CH3_SELECTOR_BIT);

			/* set Toggle mode */
			u32TmpBuf3 |= (1 << PWM_CTRL_CH3_MODE_BIT);
			break;
		default:
			return -ENODEV;
			break;
	}

	/* write new PWM register value  */
	iowrite32(u32TmpBuf, (void *) u32PrescaleAddr);
	iowrite32(u32TmpBuf2, (void *) u32CSelectorAddr);
	iowrite32(u32TmpBuf3, (void *) u32CtrlAddr);

	/****************************************************************
	 * Config PWM Counter register for setting resolution
	 ****************************************************************/
	u32DestAddr = (UINT32)PWM_REG_COUNTER_ADDR(n_module, u8PWMChannel);

	u32TmpBuf = (UINT32)(PWMDevConfig->u8PWMFrequencyDivider & 0xFF);

	/* write new PWMC value  */
	iowrite32(u32TmpBuf, (void *) u32DestAddr);

	/****************************************************************
	 * Config PWM Comparator register for setting duty cycle
	 ****************************************************************/
	u32DestAddr = (UINT32)PWM_REG_COMPARATOR_ADDR(n_module, u8PWMChannel);

	u32TmpBuf = (UINT32)((PWMDevConfig->u8PWMFrequencyDivider & 0xFF) / 2);

	/* write new PWMC value  */
	iowrite32(u32TmpBuf, (void *) u32DestAddr);

	/****************************************************************
	 * Enable PWM
	 ****************************************************************/
	u32TmpBuf3 = ioread32((void *) u32CtrlAddr);

	switch (u8PWMChannel)
	{
		case 0:
			//Enable channel
			u32TmpBuf3 |= (1 << PWM_CTRL_CH0_ENABLE_BIT);
			break;
		case 1:
			//Enable channel
			u32TmpBuf3 |= (1 << PWM_CTRL_CH1_ENABLE_BIT);
			break;
		case 2:
			//Enable channel
			u32TmpBuf3 |= (1 << PWM_CTRL_CH2_ENABLE_BIT);
			break;
		case 3:
			//Enable channel
			u32TmpBuf3 |= (1 << PWM_CTRL_CH3_ENABLE_BIT);
			break;
		default:
			return -ENODEV;
			break;
	}

	/* write new PWM value  */
	iowrite32(u32TmpBuf3, (void *) u32CtrlAddr);

  /****************************************************************
	* Enable PWM PIN
	****************************************************************/

	u32DestAddr = (UINT32)GLOBAL_REG_PIN_SELECT2_ADDR;
	u32TmpBuf = ioread32((void *) u32DestAddr);
	u32TmpBuf |= 1 << (PWM_PIN_SELECT_CH0_BIT + PWMDevConfig->u8PWMChannelNum);
	iowrite32(u32TmpBuf, (void *) u32DestAddr);

  /****************************************************************
	* Debug using
	****************************************************************/
	/* get PWM register address */
	u32PrescaleAddr = (UINT32)PWM_REG_PRESCALE_ADDR(n_module);
	u32CSelectorAddr = (UINT32)PWM_REG_CLOCK_SELECTOR_ADDR(n_module);
	u32CtrlAddr = (UINT32)PWM_REG_CONTROL_ADDR(n_module);

	/* read PWMD value */
	u32TmpBuf = ioread32((void *) u32PrescaleAddr);
	u32TmpBuf2 = ioread32((void *) u32CSelectorAddr);
	u32TmpBuf3 = ioread32((void *) u32CtrlAddr);

	printk("*** u8PWMChannel=%d, u32PrescaleAddr[0x%x]=0x%x \n",
		PWMDevConfig->u8PWMChannelNum, (unsigned int)u32PrescaleAddr, (unsigned int)u32TmpBuf);

	printk("*** u32CSelectorAddr[0x%x]=0x%x \n",
		(unsigned int)u32CSelectorAddr, (unsigned int)u32TmpBuf2);

	printk("*** u32CtrlAddr[0x%x]=0x%x \n",
		(unsigned int)u32CtrlAddr, (unsigned int)u32TmpBuf3);

	u32DestAddr = (UINT32)PWM_REG_COUNTER_ADDR(n_module, u8PWMChannel);
	u32TmpBuf = ioread32((void *) u32DestAddr);
	printk("*** COUNTER_ADDR[0x%x]=0x%x \n",
		(unsigned int)u32DestAddr, (unsigned int)u32TmpBuf);

	u32DestAddr = (UINT32)PWM_REG_COMPARATOR_ADDR(n_module, u8PWMChannel);
	u32TmpBuf = ioread32((void *) u32DestAddr);
	printk("*** COMPARATOR_ADDR[0x%x]=0x%x \n",
		(unsigned int)u32DestAddr, (unsigned int)u32TmpBuf);

	u32DestAddr = (UINT32)GLOBAL_REG_PIN_SELECT2_ADDR;
	u32TmpBuf = ioread32((void *) u32DestAddr);
	printk("*** SELECT2_ADDR[0x%x]=0x%x \n",
		(unsigned int)u32DestAddr, (unsigned int)u32TmpBuf);

	return (STATUS_OK);
}


/******************************************************************************
 *   FUNCTION        :   aess_pwm_open
 ******************************************************************************/
/** @brief	   function for open node
 * @return	   always STATUS_OK
 * @dependency     none
 * @limitation     none
 * @warning        none
 * @note           none
 * @internal       Function type: Local function \n
 ******************************************************************************/
static int aess_pwm_open(struct inode *inode, struct file *filp)
{
	if (S_u8init_flag == AESSPWM_NOT_INIT)
	{
		DEBUG_MSG("KN:start init aess_pwm_driver\n");

		S_u8init_flag = AESSPWM_INIT_OK;

		DEBUG_MSG("KN:finish init aess_pwm_driver!\n");

	}
	return (STATUS_OK);
}


/******************************************************************************
 *   FUNCTION        :   aess_pwm_release
 ******************************************************************************/
/** @brief         release resource
 * @return	   always 0
 * @dependency     none
 * @limitation     none
 * @warning        none
 * @note           none
 * @internal       Function type: Local function \n
 ******************************************************************************/
static int aess_pwm_release(struct inode* inode, struct file *flip)
{
#ifdef PWM_MAX_SPEED_ON_RELEASE
  UINT8 u8PwmChannel;
  sPWMDevConfig PWMDevConfig;

  // PWMDevConfig.u8PWMDutyCycle = 0;      // 0 is max fan speed
  for (u8PwmChannel = 0; u8PwmChannel < PWM_MAX_CHN_NUM; u8PwmChannel++)
  {
	//Set PWM value to initial value
	// DF286034 	IDRAC6 racadm reset causes a pnp removal of the perc6i card. Event ID
	PWMDevConfig.u8PWMDutyCycle =   u8InitialPWM[u8PwmChannel];
	PWMDevConfig.u8PWMChannelNum = u8PwmChannel;
	(void) aess_pwm_config_set(&PWMDevConfig);
  }
#else
#ifdef PWM_OFF_ON_RELEASE
    int n_module;
	UINT32 u32TmpBuf = 0;
	UINT32 u32DestAddr = 0;
    
    for (n_module = 0; n_module < NPCMX50_PWM_MAX_MODULES; n_module++)
    {
    	/* disable PWM channel */
    	u32DestAddr = (UINT32)PWM_REG_CONTROL_ADDR(n_module);
    	u32TmpBuf = ioread32((void *) u32DestAddr);
    	u32TmpBuf &= ~((1 << PWM_CTRL_CH0_MODE_BIT) |
    					(1 << PWM_CTRL_CH1_MODE_BIT) |
    					(1 << PWM_CTRL_CH2_MODE_BIT) | (
    					1 << PWM_CTRL_CH3_MODE_BIT));

    	iowrite32(u32TmpBuf, (void *) u32DestAddr);
    }
	}
#endif
#endif

	DEBUG_MSG("\n aess_pwm_release exit!\n");

	return 0;
}


/******************************************************************************
 *   FUNCTION        :   aess_pwm_ioctl
 ******************************************************************************/
/** @brief         ioctl function
 * @return	   STATUS_OK when success, otherwise when fail
 * @dependency     none
 * @limitation     none
 * @warning        none
 * @note           none
 * @internal       Function type: Local function \n
 ******************************************************************************/
static long aess_pwm_ioctl(struct file *flip, unsigned int cmd, unsigned long arg)
{
	int err_check;
    
	sPWMDevConfig PWMDevConfig_L;
	sPWMDevConfig *PWMDevConfig = &PWMDevConfig_L;

	DEBUG_MSG("KN:start \n");

	if (copy_from_user(&PWMDevConfig_L, (void __user *) arg, sizeof(PWMDevConfig_L)))
	{
		PERROR("copy_from_user error!!\n");
	    return -EFAULT;
	}
    
	switch(cmd)
	{
		case AESS_PWM_CONFIG_INIT:
			DEBUG_MSG("KN:Start PWM Init config process!!\n");
			DEBUG_MSG("KN:Channel[%d]\n", PWMDevConfig->u8PWMChannelNum);

			err_check = aess_pwm_config_init(PWMDevConfig);
            if (err_check == STATUS_OK)
                err_check = aess_pwm_config_set(PWMDevConfig);

			DEBUG_MSG("KN:Finish PWM Init config process!!\n");
			break;

		case AESS_PWM_CONFIG_SET:

			DEBUG_MSG("KN:Start PWM Set config process!!\n");
			DEBUG_MSG("KN:Channel[%d]\n", PWMDevConfig->u8PWMChannelNum);

			err_check = aess_pwm_config_set(PWMDevConfig);

			DEBUG_MSG("KN:Finish PWM Set config process!!\n");
			break;
			
		case AESS_PWM_CONFIG_INFO:

			DEBUG_MSG("KN:Start PWM read config process!!\n");
			DEBUG_MSG("KN:Channel[%d]\n", PWMDevConfig->u8PWMChannelNum);

			err_check = aess_pwm_config_info(PWMDevConfig);

			DEBUG_MSG("KN:Finish PWM Set config process!!\n");
			break;

		case AESS_PWM_CONFIG_DEBUG:

			DEBUG_MSG("KN:Start PWM read config process!!\n");
			DEBUG_MSG("KN:Channel[%d]\n", PWMDevConfig->u8PWMChannelNum);

			err_check = aess_pwm_config_debug(PWMDevConfig);

			DEBUG_MSG("KN:Finish PWM Set config process!!\n");
			break;

		default:
			printk(KERN_ERR "KN:aess_pwm_ioctl, NEWCMD=%x command error\n", cmd);
			err_check = -EINVAL;
			break;
	}

	/* 0->ok, minus->fail */
	return err_check;
}


static struct file_operations aess_pwm_fops = {
	.open = aess_pwm_open,
	.unlocked_ioctl = aess_pwm_ioctl,
	.release = aess_pwm_release,
};




static int npcmx50_pwm_probe(struct platform_device *pdev)
{
    int ret = 0;
    
    printk(KERN_INFO "\t\t\t* pwm probe start %s\n", pdev->name);
    /*
     * A bit ugly, and it will never actually happen but there can
     * be only one pwm and this catches any bork
     */
    if (pwm_pdev)
    {
        pr_err("\t\t\t* pwm probe failed: can't instantiate more then one device.\n");
        return -EBUSY;
    }
    

    pwm_pdev = pdev;

#ifdef CONFIG_OF
    pwm_clk = devm_clk_get(&pdev->dev, NULL);
    
    if (IS_ERR(pwm_clk))    
    {
        pr_err(" pwm probe failed: can't read clk.\n");         
        return -EPROBE_DEFER; // this error will cause the probing to run again after clk is ready.

    }
    
    //clk_prepare_enable(pwm_clk);    
    
    printk("\tpwm clock is %ld\n" , clk_get_rate(pwm_clk)); 
#endif //  CONFIG_OF

    pwm_pdev = pdev;

   return ret;
}




/* work with hotplug and coldplug */
MODULE_ALIAS("platform:npcm750-pwm");

static const struct of_device_id pwm_dt_id[] = {
	{ .compatible = "nuvoton,npcm750-pwm",  },
	{},
};
MODULE_DEVICE_TABLE(of, pwm_dt_id);

static struct platform_driver npcm750_pwm_driver = {
	.probe		= npcmx50_pwm_probe,
	//.remove		= npcmx50_pwm_remove,
	.shutdown	= NULL,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(pwm_dt_id),
	}
};




/******************************************************************************
 *   FUNCTION        :   aess_pwm_init
 ******************************************************************************/
/** @brief         initial function for insert module
 * @return	   STATUS_OK when success, otherwise when fail
 * @dependency     none
 * @limitation     none
 * @warning        none
 * @note           none
 * @internal       Function type: Global function \n
 ******************************************************************************/
int __init aess_pwm_init(void)
{
	int result;
	int i;
    int ret = 0;
    int res_cnt = 0;
    int cnt2 = 0;

#ifdef CONFIG_OF
    struct resource res[NPCMX50_PWM_MAX_MODULES];
	struct device_node *np=NULL;
#endif
    

	//Initialize PWM values for max speed
	// DF286034 	IDRAC6 racadm reset causes a pnp removal of the perc6i card. Event ID
	for (i = 0; i < PWM_MAX_CHN_NUM; i++)
	{
		u8InitialPWM[i] = 0;
	}

	for (cnt2 = 0 ; cnt2 < NPCMX50_PWM_MAX_MODULES; cnt2++){   
        pwm_base[cnt2] = NULL;
        res[cnt2].start = 0;
	}

	DEBUG_MSG("KN:init_aess_pwm_module\n");
	DEBUG_MSG("KN:GCR Version ID 0x%x \n",ioread32((void __iomem *)GLOBAL_REG_PDID_REG));

	/** - Register PWM driver to the kernel. */
	pwm_cdev = cdev_alloc();
	pwm_cdev->ops = &aess_pwm_fops;
	
	if (majornum <= 0)
	{
		/* allocate a device number */
		result = alloc_chrdev_region(&pwm_dev, 0, 1, driver_name);
		majornum = MAJOR(pwm_dev);
	}
	else
	{
		pwm_dev = MKDEV(majornum, 0);
		result = register_chrdev_region(pwm_dev, 1, driver_name);
	}
	
	printk("mknod /dev/aess_pwmdrv c %d 0\n", MAJOR(pwm_dev));

	if (result < 0) {
		DEBUG_MSG (KERN_ERR "KN:Registering the PWM character device failed with %d\n", MAJOR(pwm_dev));
		return result;
	}

	cdev_add(pwm_cdev, pwm_dev, 1);

    pwm_pdev = NULL;

#ifdef CONFIG_OF	
    /*-------------------------------------------------------------------------------------------*/
    /* Register the platform driver                                                              */
    /*-------------------------------------------------------------------------------------------*/
    DEBUG_MSG("KN:register platform driver.\n");
    result = platform_driver_register(&npcm750_pwm_driver);
    if (result) {
        printk("PWM: unable to register a platform driver\n");
        return -ENXIO;
    }

	np = of_find_matching_node(NULL, pwm_dt_id);
	if (np == NULL){
		 	pr_info("Failed to find of_find_matching_node\n");
		 	return -EINVAL;
     }

   for (res_cnt = 0; res_cnt < NPCMX50_PWM_MAX_MODULES  ; res_cnt++)
   {
       	ret = of_address_to_resource(np, res_cnt, &res[res_cnt]);
    	if (ret) {
    		pr_info("PWM of_address_to_resource fail ret %d \n",ret);
    		return -EINVAL;
    	}

       if (!request_mem_region(res[res_cnt].start, resource_size(&(res[res_cnt])), np->name)) {
           pr_err(" pwm probe failed: can't request_mem_region for resource %d.\n", res_cnt);
           ret = -EBUSY;
           goto err_ioremap;
       }

       pwm_base[res_cnt] = ioremap(res[res_cnt].start, resource_size(&(res[res_cnt])));
       printk(KERN_INFO "pwm%d base is 0x%08X, res.start 0x%08X , size 0x%08X \n",res_cnt, (u32)pwm_base[res_cnt], 
                            res[res_cnt].start, resource_size(&(res[res_cnt])));
       
       if (!pwm_base[res_cnt]) {
           pr_err(" pwm probe failed: can't read pwm base address for resource %d.\n", res_cnt);            
           return -ENOMEM;
       }
   }
   

   return ret;
    
#else
    for ( i = 0; i < NPCMX50_PWM_MAX_MODULES; i++)    
        pwm_base[i] = NPCMX50_PWM_BASE_ADDR(i);
#endif

    printk("PWM: init done.\n");

	return (STATUS_OK);

err_ioremap:
    printk(KERN_INFO "NPCMx50: pwm module load fail .  Error %d\n", ret);
    
    // release resources already allocated.
    for (cnt2 = 0 ; cnt2 < NPCMX50_PWM_MAX_MODULES; cnt2++){   
    	if (pwm_base[cnt2] != NULL)
    	{
    		iounmap(pwm_base[cnt2]);
    		pwm_base[cnt2] = NULL;
    	}
    	if (res[cnt2].start != 0)
    	{
    	    release_mem_region(res[cnt2].start, resource_size(&res[cnt2]));
    	}
	}

	return ret;
}


/******************************************************************************
 *   FUNCTION        :   aess_pwm_exit
 ******************************************************************************/
/** @brief         exit function for remove module
 * @return	   STATUS_OK when success, otherwise when fail
 * @dependency     none
 * @limitation     none
 * @warning        none
 * @note           none
 * @internal       Function type: Local function \n
 ******************************************************************************/
static void __exit aess_pwm_exit(void)
{
	DEBUG_MSG("aess_pwmdrv_exit\n");
	cdev_del(pwm_cdev);
	unregister_chrdev_region(pwm_dev, 1);
}


MODULE_DESCRIPTION("AESS PWM Driver");
MODULE_AUTHOR("Justin Lee <justin.lee@emerson.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.0.0.1");
module_init(aess_pwm_init);
module_exit(aess_pwm_exit);
module_param(driver_name, charp, S_IRUGO);
