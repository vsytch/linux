/*
* Copyright (c) 2013-2017 by Nuvoton Technology Corporation. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify it under the terms of the
* GNU General Public License Version 2 (or later) as published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/sysfs.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>


#include <asm/uaccess.h>
#include <asm/io.h>
#include <mach/hal.h>


/* TBD: For temporary use, because no type.h now */
#define STATUS_OK     0
#define STATUS_FAIL   1

/* PWM ABP clock */
#define NPCMX50_APB_CLOCK			50000 //50kHz
/* PDID */

/* PWM port base address */  
#define NPCMX50_GLOBAL_CTRL_REG    NPCMX50_GCR_BASE_ADDR

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
	u8 u8PWMChannelNum;
	
	/* PWM Base Cycle Frequency */
	u8 u8PWMBaseCycleFrequency;
	
	/* PWM Frequency Divider */
	u8 u8PWMFrequencyDivider; 

	/* PWM Duty Cycle */
	u16 u8PWMDutyCycle; 
	
} sPWMDevConfig;

static int npcm750_pwm_config_init(sPWMDevConfig *PWMDevConfig);
static int npcm750_pwm_config_set(sPWMDevConfig *PWMDevConfig);
/*static int npcm750_pwm_config_info(sPWMDevConfig *PWMDevConfig);
static int npcm750_pwm_config_debug(sPWMDevConfig *PWMDevConfig);*/
static int npcm750_pwm_get_dutycycle(sPWMDevConfig *PWMDevConfig);

//#define PWM_DEBUG

#define DRIVER_NAME "npcm750_pwm"
#define PWM_MAX 255

struct npcm750_pwm_data {
	unsigned long clk_freq;
	bool pwm_present[8];
	const struct attribute_group *groups[2];
};

/******************************************************************************
*   STRUCT      :   sPWMDevConfig
******************************************************************************/
/**
 *  @brief   Structure to PWM driver config.
 *
 *****************************************************************************/

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

static const struct of_device_id pwm_dt_id[];
static struct platform_driver npcm750_pwm_driver;

void __iomem *pwm_base[NPCMX50_PWM_MAX_MODULES];

u8 u8InitialPWM[PWM_MAX_CHN_NUM];

static ssize_t set_pwm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
	int ret;
	sPWMDevConfig PWMDevConfig;
	long fan_ctrl;

	ret = kstrtol(buf, 10, &fan_ctrl);
	if (ret != 0)
    {
		return ret;
    }

	if (fan_ctrl < 0 || fan_ctrl > PWM_MAX)
		return -EINVAL;

	PWMDevConfig.u8PWMDutyCycle = fan_ctrl;
	PWMDevConfig.u8PWMChannelNum = index;

	npcm750_pwm_config_set(&PWMDevConfig);

	return count;
}

static ssize_t show_pwm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
	sPWMDevConfig PWMDevConfig;

	PWMDevConfig.u8PWMChannelNum = index;
	npcm750_pwm_get_dutycycle(&PWMDevConfig);

	return sprintf(buf, "%u\n", PWMDevConfig.u8PWMDutyCycle);
}

static umode_t pwm_is_visible(struct kobject *kobj,
			      struct attribute *a, int index)
{
	//struct device *dev = container_of(kobj, struct device, kobj);

	return a->mode;
}

static SENSOR_DEVICE_ATTR(pwm1, 0644,
			show_pwm, set_pwm, 0);
static SENSOR_DEVICE_ATTR(pwm2, 0644,
			show_pwm, set_pwm, 1);
static SENSOR_DEVICE_ATTR(pwm3, 0644,
			show_pwm, set_pwm, 2);
static SENSOR_DEVICE_ATTR(pwm4, 0644,
			show_pwm, set_pwm, 3);
static SENSOR_DEVICE_ATTR(pwm5, 0644,
			show_pwm, set_pwm, 4);
static SENSOR_DEVICE_ATTR(pwm6, 0644,
			show_pwm, set_pwm, 5);
static SENSOR_DEVICE_ATTR(pwm7, 0644,
			show_pwm, set_pwm, 6);
static SENSOR_DEVICE_ATTR(pwm8, 0644,
			show_pwm, set_pwm, 7);

static struct attribute *pwm_dev_attrs[] = {
	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_pwm2.dev_attr.attr,
	&sensor_dev_attr_pwm3.dev_attr.attr,
	&sensor_dev_attr_pwm4.dev_attr.attr,
	&sensor_dev_attr_pwm5.dev_attr.attr,
	&sensor_dev_attr_pwm6.dev_attr.attr,
	&sensor_dev_attr_pwm7.dev_attr.attr,
	&sensor_dev_attr_pwm8.dev_attr.attr,
	NULL,
};

static const struct attribute_group pwm_dev_group = {
	.attrs = pwm_dev_attrs,
	.is_visible = pwm_is_visible,
};

/******************************************************************************
 *   FUNCTION        :   npcm750_pwm_config_init
 ******************************************************************************/
/** @brief 	pass configuration to PWMX driver to do initialization
 * @return	STATUS_OK when success, otherwise when fail
 * @dependency     none
 * @limitation     none
 * @warning        none
 * @note           none
 * @internal       Function type: Local function \n
 ******************************************************************************/
static int npcm750_pwm_config_init(sPWMDevConfig *PWMDevConfig)
{
	u8  u8PWMChannel = (PWMDevConfig->u8PWMChannelNum % PWM_MAX_CHN_NUM_IN_A_MODULE);
	u32 u32TmpBuf = 0, u32TmpBuf2 = 0, u32TmpBuf3 = 0;
	u32 u32DestAddr = 0, u32CtrlAddr = 0, u32PrescaleAddr = 0, u32CSelectorAddr = 0;
	u32 n_module = PWMDevConfig->u8PWMChannelNum/PWM_MAX_CHN_NUM_IN_A_MODULE;

	//Set initial PWM value
	// DF286034 	IDRAC6 racadm reset causes a pnp removal of the perc6i card. Event ID
	u8InitialPWM[PWMDevConfig->u8PWMChannelNum] = (PWMDevConfig->u8PWMDutyCycle & 0xFF);

	/**********************************************************************
	 * Config PWMD register for setting frequency divider during initialize
	 **********************************************************************/

	/* get PWM register address */
	u32PrescaleAddr = (u32)PWM_REG_PRESCALE_ADDR(n_module);
	u32CSelectorAddr = (u32)PWM_REG_CLOCK_SELECTOR_ADDR(n_module);
	u32CtrlAddr = (u32)PWM_REG_CONTROL_ADDR(n_module);

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
	u32DestAddr = (u32)PWM_REG_COUNTER_ADDR(n_module, u8PWMChannel);

	u32TmpBuf = (u32)(PWMDevConfig->u8PWMFrequencyDivider & 0xFF);

	DEBUG_MSG("u8PWMChannel=%d, u32CSelectorAddr=0x%x, u8PWMFrequencyDivider=%d, PWMDTmpBuf=%x \n",
		PWMDevConfig->u8PWMChannelNum, u32DestAddr, PWMDevConfig->u8PWMFrequencyDivider, (unsigned int)u32TmpBuf);

	/* write new PWMC value  */
	iowrite32(u32TmpBuf, (void *) u32DestAddr);

	/****************************************************************
	 * Config PWM Comparator register for setting duty cycle
	 ****************************************************************/
	u32DestAddr = (u32)PWM_REG_COMPARATOR_ADDR(n_module, u8PWMChannel);

	u32TmpBuf = (u32)(PWMDevConfig->u8PWMDutyCycle & 0xFF);

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
 *   FUNCTION        :   npcm750_pwm_config_set
 ******************************************************************************/
/** @brief  	set PWMX duty cycle
 * @return	STATUS_OK when success, otherwise when fail
 * @dependency     none
 * @limitation     none
 * @warning        none
 * @note           none
 * @internal       Function type: Local function \n
 ******************************************************************************/
static int npcm750_pwm_config_set(sPWMDevConfig *PWMDevConfig)
{
	u8  u8PWMChannel = (PWMDevConfig->u8PWMChannelNum % PWM_MAX_CHN_NUM_IN_A_MODULE);
	u32 u32TmpBuf = 0;
	u32 u32DestAddr = 0;
	u32 n_module = PWMDevConfig->u8PWMChannelNum/PWM_MAX_CHN_NUM_IN_A_MODULE;

	/****************************************************************
	 * Config PWM Comparator register for setting duty cycle
	 ****************************************************************/
	u32DestAddr = (u32)PWM_REG_COMPARATOR_ADDR(n_module, u8PWMChannel);

	u32TmpBuf = (u32)(PWMDevConfig->u8PWMDutyCycle & 0xFF);

	DEBUG_MSG("u8PWMChannel=%d, u32CSelectorAddr=0x%x, u8PWMDutyCycle=%d, PWMDTmpBuf=%x \n",
			  PWMDevConfig->u8PWMChannelNum, u32DestAddr, PWMDevConfig->u8PWMDutyCycle, (unsigned int)u32TmpBuf);

	/* write new PWMC value  */
	iowrite32(u32TmpBuf, (void *) u32DestAddr);

// Added by DIJIC to diable output drive when 100% PWM set

	u32DestAddr = (u32)GLOBAL_REG_PIN_SELECT2_ADDR;
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
 *   FUNCTION        :   npcm750_pwm_config_info
 ******************************************************************************/
/** @brief  	get PWMX config info
 * @return	STATUS_OK when success, otherwise when fail
 * @dependency     none
 * @limitation     none
 * @warning        none
 * @note           none
 * @internal       Function type: Local function \n
 ******************************************************************************/
static int npcm750_pwm_get_dutycycle(sPWMDevConfig *PWMDevConfig)
{
	u8  u8PWMChannel = (PWMDevConfig->u8PWMChannelNum % PWM_MAX_CHN_NUM_IN_A_MODULE);
	u32 n_module = PWMDevConfig->u8PWMChannelNum/PWM_MAX_CHN_NUM_IN_A_MODULE;
	u32 u32TmpBuf = 0;
	u32 u32DestAddr = 0;

	/****************************************************************
	 * Debug using
	 ****************************************************************/

	u32DestAddr = (u32)PWM_REG_COMPARATOR_ADDR(n_module, u8PWMChannel);
	u32TmpBuf = ioread32((void *) u32DestAddr);
	//printk("*** Duty Cycle - CMR[0x%x]=0x%x \n", (unsigned int)u32DestAddr, (unsigned int)u32TmpBuf);

	PWMDevConfig->u8PWMDutyCycle = (u16)u32TmpBuf;

	return (STATUS_OK);
}

/******************************************************************************
 *   FUNCTION        :   npcm750_pwm_init
 ******************************************************************************/
/** @brief         initial function for insert module
 * @return	   STATUS_OK when success, otherwise when fail
 * @dependency     none
 * @limitation     none
 * @warning        none
 * @note           none
 * @internal       Function type: Global function \n
 ******************************************************************************/
static int npcm750_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct npcm750_pwm_data *priv;
    struct resource res[NPCMX50_PWM_MAX_MODULES];
	struct device *hwmon;
	struct clk *clk;
	int ret;
	int i,res_cnt;
	sPWMDevConfig PWMDevConfig;
	int err_check;

	np = dev->of_node;

	//Initialize PWM values for max speed
	// DF286034 	IDRAC6 racadm reset causes a pnp removal of the perc6i card. Event ID
	for (i = 0; i < PWM_MAX_CHN_NUM; i++)
	{
		u8InitialPWM[i] = 0;
	}

	for (res_cnt = 0; res_cnt < NPCMX50_PWM_MAX_MODULES  ; res_cnt++)
	{
		 ret = of_address_to_resource(np, res_cnt, &res[res_cnt]);
		 if (ret) {
			 pr_err("PWM of_address_to_resource fail ret %d \n",ret);
			 return -EINVAL;
		 }

		pwm_base[res_cnt] =  devm_ioremap_resource(dev, &(res[res_cnt]));
		DEBUG_MSG("pwm%d base is 0x%08X, res.start 0x%08X , size 0x%08X \n",res_cnt, (u32)pwm_base[res_cnt], 
							 res[res_cnt].start, resource_size(&(res[res_cnt])));

		if (!pwm_base[res_cnt]) {
			pr_err(" pwm probe failed: can't read pwm base address for resource %d.\n", res_cnt);            
			return -ENOMEM;
		}
	}

	/*if (IS_ERR(regs))
		return PTR_ERR(regs);*/
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk))
		return -ENODEV;

	priv->clk_freq = clk_get_rate(clk);


	for (i = 0; i < PWM_MAX_CHN_NUM; i++)
	{
		/* setting to PWM of 25Khz */
		PWMDevConfig.u8PWMChannelNum = i;
		PWMDevConfig.u8PWMBaseCycleFrequency = PWM_PRESCALE_DEFALUT_NUM;
		PWMDevConfig.u8PWMFrequencyDivider = PWM_COUNTER_DEFALUT1_NUM;
		PWMDevConfig.u8PWMDutyCycle = PWM_COUNTER_DEFALUT1_NUM / 2;
		err_check = npcm750_pwm_config_init(&PWMDevConfig);
		if (err_check != STATUS_OK)
		{
			pr_err("PWM init fail channel %d \n",i);
			return -EINVAL;
		}
	}

	/*aspeed_set_clock_enable(priv->regmap, true);
	aspeed_set_clock_source(priv->regmap, 0);*/

	priv->groups[0] = &pwm_dev_group;
	priv->groups[1] = NULL;

	hwmon = devm_hwmon_device_register_with_groups(dev,"npcm750_pwm",priv, priv->groups);
	if (IS_ERR(hwmon))
	{
		pr_err("PWM Driver failed - devm_hwmon_device_register_with_groups failed \n");
		return PTR_ERR(hwmon);
	}

	pr_info("NPCM750 PWM Driver probed\n");    
	        
	return 0;
}

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:npcm750-pwm");

static const struct of_device_id of_pwm_match_table[] = {
	{ .compatible = "nuvoton,npcm750-pwm", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_match_table);

static struct platform_driver npcm750_pwm_driver = {
	.probe		= npcm750_pwm_probe,
	.driver		= {
		.name	= "npcm750_pwm",
		.of_match_table = of_pwm_match_table,
	},
};

module_platform_driver(npcm750_pwm_driver);

MODULE_DESCRIPTION("Nuvoton NPCM750 PWM Driver");
MODULE_AUTHOR("Tomer Maimon <tomer.maimon@nuvoton.com>");
MODULE_LICENSE("GPL v2");
