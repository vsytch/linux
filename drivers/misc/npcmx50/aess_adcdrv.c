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

#define AESSADCSENSORDRV_C

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/signal.h>
#include <linux/spinlock.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <mach/hal.h>
#include <linux/delay.h>

#include "aess_adcdrv.h"

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/clk.h>


static void *adc_virt_addr;
#undef ADC_REGS_BASE
#define ADC_REGS_BASE		(adc_virt_addr)

static const struct of_device_id adc_dt_npcm750_match[] = {
       { .compatible = "nuvoton,npcm750-adc" },
       { /*sentinel*/ },
};
#endif

#define ADC_MAX_CLOCK 12500000
#define VREF_MVOLT 2048		//vref = 2.000v

//#define ADC_DEBUG

#ifdef ADC_DEBUG
static char *S_ADCChnlString[] =
{
	"ADCI0", "ADCI1", "ADCI2", "ADCI3", "ADCI4", "ADCI5", "ADCI6", "ADCI7"
};
#define PDEBUG(fmt, args...)	printk("aess_adcdrv %s() " fmt, __func__, ##args)
#else
#define PDEBUG(fmt, args...)
#endif
#define PERROR(fmt, args...)	printk("aess_adcdrv %s(): " fmt, __func__, ##args)

static int majornum = 0;           /* default to dynamic major */
module_param(majornum, int, 0);
MODULE_PARM_DESC(majornum, "Major device number");

static struct clk* adc_clk;

/* driver name will passed by insmod command */
static char *driver_name = ADC_DRIVER_NAME;

/**< ADC device */
static dev_t adcsensor_dev;

/**< declare adcsensor_cdev */
struct cdev *adcsensor_cdev;

/** Sysfs class structure. */
static struct class *sys_class;

struct aess_adc_data {    
	struct semaphore sem;
};

static UINT32 adc_clk_rate = ADC_MAX_CLOCK;

static struct platform_driver aess_adc_device_driver = {
	.driver = {
	 .name	 = ADC_DRIVER_NAME,
	 .owner	 = THIS_MODULE,
	}
};

static struct aess_adc_data *aess_adc;

static int aess_adcsensor_open(struct inode *inode, struct file *filp)
{
	PERROR("Open ADC sensor !!! \n");

	return (0);
}

static int aess_adcsensor_read(sADCSensorData *pADCSensorStruct)
{
	UINT8  u8ChannelNum = 0;
	UINT32 regtemp = 0;
	volatile int cnt = 0;

	u8ChannelNum = pADCSensorStruct->u8ADCChannelNum;

	/* Select ADC channal */
	regtemp = ioread32((void *) NPCMX50_ADCCON);
	regtemp &= ~NPCMX50_ADCCON_MUXMASK;

	iowrite32((UINT32) (regtemp | NPCMX50_ADCCON_ADCMUX(u8ChannelNum) | NPCMX50_ADCCON_ADC_EN | NPCMX50_ADCCON_REFSEL), (void *) NPCMX50_ADCCON);

	/* Activate convert the ADC input */
	regtemp = ioread32((void *) NPCMX50_ADCCON);
	iowrite32((UINT32) (regtemp | NPCMX50_ADCCON_ADC_CONV), (void *) NPCMX50_ADCCON);

	/* Wait value */
	while(((regtemp=ioread32((void *) NPCMX50_ADCCON)) & NPCMX50_ADCCON_ADC_CONV) != 0)
	{
		if (cnt < NPCMX50_ADC_CONVERT_MAX_RETRY_CNT)
			cnt++;
		else
		{
			PERROR("ADC CONVERT FAIL - Timeout\n");     
			PERROR("NPCMX50_ADCCON=0x%08X, ADC_MUX=%d u8ChannelNum=%d!!\n", regtemp, (regtemp>>24)&0xF, u8ChannelNum);
			if (((regtemp>>24)&0xF) != u8ChannelNum)        
				PERROR("ADC_MUX != u8ChannelNum, I suspect that 2 threads are trying to access this read and it is not protected by mutex\n");

			/* if convertion failed - reset ADC module */
			iowrite32((UINT32) 0x08000000, (void *) NPCMX50_IPSRST1);
			msleep(100);	 
			iowrite32((UINT32) 0x00000000, (void *) NPCMX50_IPSRST1);	  
			msleep(100);	
			PERROR("RESET ADC Complete\n");	
			return (-EAGAIN);
		}
	}
	
	/*When an ADC conversion operation finished, a delay must be added before the next conversion operation.
	The delay depend on the ADC clock: 
	When ADC clock is 0.5 MHz: delay is 4 us.
	When ADC clock is 12.5 MHz: delay is 160 ns.

	In the current driver the ADC clock is 12.5MHz, so delay is not needed. (already the R/W register take more than 160ns)
	If the ADC clock will be lower than 12.5MHz please add delay according the details above*/
	//udelay(conv_delay);
	
	/* finish to convert */
	pADCSensorStruct->u32ADCReading
	= NPCMX50_ADCCON_ADC_DATA_MASK(ioread32(NPCMX50_ADCDATA));

	PDEBUG("[%d_%s] ADCReading=%ld [%ldmV]\n",
		   u8ChannelNum, S_ADCChnlString[u8ChannelNum], (long int)pADCSensorStruct->u32ADCReading,
		   (long int)(pADCSensorStruct->u32ADCReading * VREF_MVOLT / 1024));

	return 0;
}


static long aess_adcsensor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err_check;
	sADCSensorData ADCSensorStruct;
	sADCSensorData *pADCSensorStruct = &ADCSensorStruct;

	if (copy_from_user(&ADCSensorStruct, (void __user *) arg, sizeof(ADCSensorStruct)))
	{
		PERROR("copy_from_user error!!\n");
	    return -EFAULT;
	}

	switch(cmd)
	{
		case AESS_ADCDRV_R:

			switch (pADCSensorStruct->u8ADCChannelNum)
			{
				case NPCMX50_ADC_CHNL0_ADCI0:
				case NPCMX50_ADC_CHNL1_ADCI1:
				case NPCMX50_ADC_CHNL2_ADCI2:
				case NPCMX50_ADC_CHNL3_ADCI3:
				case NPCMX50_ADC_CHNL4_ADCI4:
				case NPCMX50_ADC_CHNL5_ADCI5:
				case NPCMX50_ADC_CHNL6_ADCI6:
				case NPCMX50_ADC_CHNL7_ADCI7:
					down(&aess_adc->sem);
					err_check = aess_adcsensor_read(pADCSensorStruct);
    		        PDEBUG("%d = aess_adcsensor_read()\n", err_check);
					up(&aess_adc->sem);
					break;

				default:
					PERROR("aess_adcsensor_ioctl, Unsupport channel number [%d]!\n", pADCSensorStruct->u8ADCChannelNum);
					err_check = -ENODEV;
					break;
			}

			 break;

		default:
			PERROR("aess_adcsensor_ioctl, command error!!! \n");
			err_check = -EINVAL;
	}

    if (err_check == 0) {
		if (copy_to_user((void __user *)arg, &ADCSensorStruct, sizeof(ADCSensorStruct)))
    	{
    		PERROR("copy_to_user error!!\n");
    	    return -EFAULT;
    	}
    }

	/* 0->ok, minus->fail */
	return err_check;
}


static int aess_adcsensor_release(struct inode *inode, struct file *filp)
{
	PDEBUG("Release ADC sensor !!! \n");

	return (0);
}


struct file_operations aess_adcsensor_fops = {
	.open = aess_adcsensor_open,
	.unlocked_ioctl = aess_adcsensor_ioctl,
	.release = aess_adcsensor_release,
};


int __init aess_adcsensor_init(void)
{
	int result = 0;
	UINT32 regtemp = 0;
	
#ifdef CONFIG_OF
	struct device_node *np;
	struct resource res;

	np = of_find_compatible_node(NULL, NULL, "nuvoton,npcm750-adc");
	if (np == NULL)
	{
		PERROR("Failed to find of_find_matching_node\n");
		return -ENODEV;
	}

	result = of_address_to_resource(np, 0, &res);
	if (result)
	{
		PERROR("of_address_to_resource fail ret %d \n", result);
		return -EINVAL;
	}

	adc_virt_addr = ioremap_nocache(res.start, resource_size(&res));

	if (!adc_virt_addr)
	{
		PERROR("adc_virt_addr fail \n");
		return -ENOMEM;
	}
	printk(KERN_INFO "KN: ADC base is 0x%08X ,res.start 0x%08X \n", (u32) adc_virt_addr, res.start);
	
	adc_clk = of_clk_get(np, 0);    
    if (IS_ERR(adc_clk))	
    {
        PERROR("ADC clock failed: can't read clk. Assuming ADC clock Rate 12.5MHz\n");			
    }

	/* calculate ADC clock divider */
	regtemp = ioread32((void *) NPCMX50_ADCCON);
	regtemp = regtemp >> 1;
	regtemp &= 0xff;
	
	adc_clk_rate = clk_get_rate(adc_clk) / ((regtemp+1)*2);
		
#endif

	/** - allocate a cedv for adcsensor_cdev  */
	adcsensor_cdev = cdev_alloc();
	PDEBUG("Init_aess_adcsensor_module Sucess!!! \n");

	/** - dispatch aess_adcsensor_fops to adcsensor_cdev->ops */
	adcsensor_cdev->ops = &aess_adcsensor_fops;

	if (majornum <= 0)
	{
		/* allocate a device number */
		result = alloc_chrdev_region(&adcsensor_dev, 0, 1, driver_name);
		majornum = MAJOR(adcsensor_dev);
	}
	else
	{
		adcsensor_dev = MKDEV(majornum, 0);
		result = register_chrdev_region(adcsensor_dev, 1, driver_name);
	}

	/** - register the adc dev */
	/*result = alloc_chrdev_region(&adcsensor_dev, 0, 1, driver_name);*/

	if (result < 0) {
		PDEBUG ("Registering the character device failed with %d\n", MAJOR(adcsensor_dev));
		return result;
	}
	printk("mknod /dev/aess_adcdrv c %d 0\n", MAJOR(adcsensor_dev));

	/** - add adcsensor_cdev to system */
	cdev_add(adcsensor_cdev, adcsensor_dev, 1);

	/** - Register driver with sysfs class. */
	sys_class = class_create(THIS_MODULE, driver_name);
	device_create(sys_class, NULL, adcsensor_dev, NULL, driver_name);

	result = platform_driver_register(&aess_adc_device_driver);
	if (result)
	{
		PERROR("can't register sysfs.\n");
		return result;
	}

	/** create single semaphore **/
	aess_adc = kmalloc(sizeof(struct aess_adc_data), GFP_KERNEL);
	if (NULL == aess_adc)
	{	  
		return (-ENOMEM);
	}
	
	sema_init(&aess_adc->sem, 1);
	
	PERROR("ADC clock Rate %d \n",adc_clk_rate);		
	PDEBUG("Open the ADC Sensor Device!!!\n");

	/** Enable the ADC Module **/
	iowrite32((UINT32) NPCMX50_ADCCON_ADC_EN, (void *) NPCMX50_ADCCON);

#if 0 //ADC_DEBUG
	/* TEST CODE */
	{
		int i = 0;
		UINT32 regtemp = 0;
		sADCData pADCStruct;

		regtemp = ioread32((void *) NPCMX50_ADCCON);
		PDEBUG("NPCMX50_ADCCON=0x%lx\n", regtemp);

		pADCStruct.u32ADCReading = 0;

		aess_adc_open(NULL,NULL);

		for (i = 0; i < NPCMX50_ADC_MAX_CHNL_NUM; i++)
		{
			pADCSensorStruct.u8ADCChannelNum = i;
			aess_adcsensor_read(&pADCSensorStruct);
		}
	}
#endif

	return result;
}


void __exit aess_adcsensor_exit(void)
{
	UINT32 regtemp = 0;

	/* release semaphore */
	kfree(aess_adc);
		
	regtemp = ioread32((void *) NPCMX50_ADCCON);
		
	/* Disable the ADC Module */
	iowrite32((UINT32) (regtemp & ~NPCMX50_ADCCON_ADC_EN), (void *) NPCMX50_ADCCON);

	/** - Destory sysfs class. */
	device_destroy(sys_class, adcsensor_dev);
	class_destroy(sys_class);

	/** - delete adcsensor_cdev from sytem */
	cdev_del(adcsensor_cdev);

	/** - unregister adcsensor_cdev */
	unregister_chrdev_region(adcsensor_dev,1);
	PDEBUG ("ess_adcsensor_exit \n");

#if 0
	/** - Remove sysfs files. */
	driver_remove_file(&(aess_adc_device_driver.driver), &drv_attr_adcinfo);
#endif

	/** - Unregister sysfs. */
	platform_driver_unregister(&(aess_adc_device_driver));

	return;
}


MODULE_DESCRIPTION("AESS ADC Sensor Driver");
MODULE_AUTHOR("Justin Lee <justin.lee@emersion.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.0.0.9");
module_param(driver_name, charp, S_IRUGO);
module_init(aess_adcsensor_init);
module_exit(aess_adcsensor_exit);

