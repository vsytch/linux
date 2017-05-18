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
 *  @file   aess_fansensordrv.c
 *  @brief  This is onboard fan tachometer device driver for NPCM750
 *
 *  @internal
 *----------------------------------------------------------------------------*/
/******************************************************************************
* Content
* ----------------
*   npcm750_fantach_start_capture() - Start the input capture.
*   npcm750_fantach_polling()       - Polling to change channel and input capture.
*   aess_fansensor_open()           - Open fan tach device.
*   aess_fansensor_config()         - Configure fan tach channel.
*   aess_fansensor_read()           - Read the fan rotational speed.
*   aess_fansensor_ioctl()          - The I/O control function.
*   aess_fansensor_release()        - Close fan tach device.
*   npcm750_fantach_compute()       - Compute the fan rotational speed.
*   npcm750_check_cmp()             - Check different timers when IRQ occurs.
*   npcm750_mft0_isr()              - The ISR for MFT0.
*   npcm750_fantach_init()          - Initialized H/W registers.
*   aess_fansensor_init()           - The module initialized function.
*   aess_fansensor_exit()           - The module exited function.
******************************************************************************/
/*
 * NPCM750 fan tachometer driver.
 *
 * ©2009-2015 Avocent Corporation. All rights reserved.
 *
 * This file is subject to the terms and conditions of the GNU
 * General Public License. This program is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 */

#define AESSFANSENSORDRV_C
//#define CONFIG_DEBUG_FS

#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/signal.h>
#include <linux/spinlock.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/types.h>
#include <asm/uaccess.h>

#include <mach/hal.h>

#include "aess_fansensordrv.h"
#include "aess_debugfs.h"

static int majornum = 0;           /* default to dynamic major */
module_param(majornum, int, 0);
MODULE_PARM_DESC(majornum, "Major device number");

static int mft_virt_addr;

#ifdef CONFIG_OF
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>


#undef MFT_REGS_BASE

#define MFT_REGS_BASE(n)    (mft_virt_addr + ((n) * 0x1000L))

#undef MFT_INTERRUPT_0
#undef MFT_INTERRUPT_1
#undef MFT_INTERRUPT_2
#undef MFT_INTERRUPT_3
#undef MFT_INTERRUPT_4
#undef MFT_INTERRUPT_5
#undef MFT_INTERRUPT_6
#undef MFT_INTERRUPT_7
#define MFT_INTERRUPT_0 mft_irq[0]
#define MFT_INTERRUPT_1 mft_irq[1]
#define MFT_INTERRUPT_2 mft_irq[2]
#define MFT_INTERRUPT_3 mft_irq[3]
#define MFT_INTERRUPT_4 mft_irq[4]
#define MFT_INTERRUPT_5 mft_irq[5]
#define MFT_INTERRUPT_6 mft_irq[6]
#define MFT_INTERRUPT_7 mft_irq[7]
int mft_irq[8];

static const struct of_device_id mft_dt_npcm750_match[] = {
       { .compatible = "nuvoton,npcm750-mft" },
       { /*sentinel*/ },
};
#endif

/* driver name will be passed by insmod command */
static char *driver_name = FANSENSOR_DRIVER_NAME;

static dev_t dev;
struct cdev *fansensor_cdev;

/* for request irq use */
static UINT8 u8dummy = 0;


#ifdef CONFIG_DEBUG_FS
/* for debugfs */
static u32 u32tmpcounter = 0;
#endif

/* Input clock */
static UINT32 u32InputClock = 0;

static volatile sFanTachDev S_npcm750_fantach[NPCM750_MAX_FAN_TACH];
static UINT8 S_npcm750_fantach_select;

static struct timer_list npcm750_fantach_timer;

/** Sysfs class structure. */
static struct class *sys_class;

static struct clk* mft_clk;

/** FAN device driver for sysfs. */
static struct platform_driver aess_fan_device_driver = {
	.driver = {
		.name  = FANSENSOR_DRIVER_NAME,
		.owner = THIS_MODULE,
	}
};

/******************************************************************************
 *   FUNCTION        :   show_value
 ******************************************************************************/
/** @brief 	Used for sysfs, do nothing right now.
 * @return	Size of buffer.
 * @dependency     none
 * @limitation     none
 * @warning        none
 * @note           none
 * @internal       Function type: Local function \n
 ******************************************************************************/
static ssize_t show_value(

		/** Linux device driver. */
		struct device_driver *drv,

		/** Buffer to user space. */
		char *buf
	)
{
	/** @scope */
	/** - Do nothing. */

	return (strlen(buf));
}


/******************************************************************************
 *   FUNCTION        :   set_value
 ******************************************************************************/
/** @brief 	Used for sysfs, do nothing right now.
 * @return	Size of buffer.
 * @dependency     none
 * @limitation     none
 * @warning        none
 * @note           none
 * @internal       Function type: Local function \n
 ******************************************************************************/
static ssize_t set_value(
		/** Linux device driver. */
		struct device_driver *drv,

		/** User space buffer. */
		const char *buf,

		/** Size. */
		size_t count
						)
{
	/** @scope */
	/** - Do nothing. */

	return (strlen(buf));
}

/******************************************************************************
*   STRUCT      :   fan_flag
******************************************************************************/
/**
 *  @brief   Flag for sysfs
 *
 *****************************************************************************/
struct driver_attribute fan_flag = {
	/*Dell: .owner no longer part of driver_attribute struct */
	.attr = {
		.name  = "fan_info" ,
		/* .owner = THIS_MODULE,*/
		.mode   = S_IWUSR | S_IRUGO
	},
	.show   = show_value,
	.store  = set_value,
};

/******************************************************************************
*   FUNCTION        :   aess_fansensor_get_input_clock
******************************************************************************/
/**
 *  @brief      Get fan tach module input clock
 *
 *  @return     None
 *
 *  @dependency None
 *
 *  @limitation None
 *
 *  @warning    None
 *
 *  @note       None
 *
 *  @internal   Function Type: Private inline function\n
 *
 *****************************************************************************/
static void aess_fansensor_get_input_clock(void)
{
    unsigned int apb_clk_src = 0;
	
	//
	// Warning: this entire function is redundant. clk frequency can be read from device tree.
	//
	
#ifdef CONFIG_OF
	apb_clk_src = clk_get_rate(mft_clk);
#else
    unsigned long reg_value = 0;
    unsigned long pll_value = 0;
    unsigned int indv = 0;
    unsigned int otdv1 = 0, otdv2 = 0;
    unsigned int fbdv = 0;
    unsigned int apbdiv = 0;
    unsigned int ahbdiv = 0;
    unsigned int axidiv = 0;

    /* get CPUCKSEL, check clock source is PLL0, PLL1, or 25M. */
    reg_value = ioread32((void *)CLK_BA_REG_CLKSEL_ADDR) & 0x3;
    DRV_INFO("[FAN]: CPUCKSEL: 0x%lx\n", reg_value);

    switch (reg_value)
    {
        case 0:
            /* PLL0 */
            pll_value = ioread32((void *)CLK_BA_REG_PLLCON0_ADDR);
            break;

        case 1:
            /* PLL1 */
            pll_value = ioread32((void *)CLK_BA_REG_PLLCON1_ADDR);
            break;

        case 2:
            /* 25M */
            u32InputClock = PLL_INPUT_CLOCK;
            return;

        default:
            DRV_ERROR("Error CPUCKSEL!!\n");
            u32InputClock = -1;
            return;
    }

    /* get PLL Input Clock Divider. [5:0] */
    indv = pll_value & 0x1F;

    /* get PLL Output Clock Divider 1. [10:8] */
    otdv1 = (pll_value & 0x700) >> 8;
    /* get PLL Output Clock Divider 2. [15:13] */
    otdv2 = (pll_value & 0xE000) >> 13;

    /* get PLL VCO Output Clock Feedback Divider. [27:16] */
    fbdv = (pll_value & 0xFFF0000) >> 16;

    DRV_INFO("[FAN]: INDV : 0x%x\n", indv);
    DRV_INFO("[FAN]: OTDV1: 0x%x\n", otdv1);
    DRV_INFO("[FAN]: OTDV2: 0x%x\n", otdv2);
    DRV_INFO("[FAN]: FBDV : 0x%x\n", fbdv);

    apb_clk_src = PLL_INPUT_CLOCK * fbdv / indv / (otdv1 * otdv2);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Warning: previous code had these 3 lines missing, so APB4 was mincalculeted in the first place..    */
    /*-----------------------------------------------------------------------------------------------------*/
    if ((ioread32((void *)CLK_BA_REG_CLKSEL_ADDR) & 0x3)  == 1)    {
        apb_clk_src = apb_clk_src >> 1; 
    }

    DRV_INFO("[FAN] PLL: %d\n", (int) apb_clk_src);

    /* AXI clock divider */
    reg_value = ioread32((void *)CLK_BA_REG_CLKDIV1_ADDR);
    DRV_INFO("[FAN]: CLKDIV1: 0x%lx\n", reg_value);

    axidiv = (reg_value & 0x1);
    apb_clk_src = apb_clk_src >> axidiv;
    
    /* AHB clock divider */
    ahbdiv = (reg_value & 0x0C000000) >> 26;
    apb_clk_src = apb_clk_src >> ahbdiv;

    /* APB clock divider */
    reg_value = ioread32((void *)CLK_BA_REG_CLKDIV2_ADDR);
    DRV_INFO("[FAN]: CLKDIV2: 0x%lx\n", reg_value);

    apbdiv = (reg_value & 0xC0000000) >> 30;
    apb_clk_src = apb_clk_src >> apbdiv;

#endif

    DRV_INFO("[FAN] APB4: %d\n", (int)apb_clk_src);
    /* Fan tach input clock = APB clock / prescalar, default is 255. */
    u32InputClock = apb_clk_src / (NPCM750_MFT_CLKPS + 1);

    DRV_INFO("[FAN] PWM: %d\n", (int) u32InputClock);
    DRV_INFO("[FAN] InputClock: %d\n", (int) u32InputClock);

}


/******************************************************************************
*   FUNCTION        :   npcm750_fantach_start_capture
******************************************************************************/
/**
 *  @brief      Start input capture for calculate fan rotational speed.
 *
 *  @return     None
 *
 *  @dependency None
 *
 *  @limitation None
 *
 *  @warning    None
 *
 *  @note       None
 *
 *  @internal   Function Type: Private inline function\n
 *
 *****************************************************************************/
static inline void npcm750_fantach_start_capture(

                /** the MFT module */
                UINT8 mft,

                /** the timer of the specifc MFT */
                UINT8 cmp
                                                )
{
    UINT8 fan_id = 0;
    UINT8 reg_mode = 0;
    UINT8 reg_int = 0;

    fan_id = NPCM750_FAN_TACH_INPUT(mft, cmp);

    /** to check whether any fan tach is enable */
    if (FAN_TACH_DISABLE != S_npcm750_fantach[fan_id].u8FanStatusFlag)
    {
        DRV_MSG("npcm750_fantach_start_capture: %d\n", (int) fan_id);
        /** reset status */
        S_npcm750_fantach[fan_id].u8FanStatusFlag = FAN_TACH_INIT;

        reg_int = ioread8((void *)MFT_REG_TIEN(mft));

        if (NPCM750_CMPA == cmp)
        {
            /** enable interrupt */
            iowrite8((UINT8) (reg_int | (NPCM750_TIEN_TAIEN | NPCM750_TIEN_TEIEN)),
                    (void *)MFT_REG_TIEN(mft));

            reg_mode =  NPCM750_TCKC_C1CSEL(NPCM750_MFT_APB_CLOCK_MODE)
                | ioread8((void *)MFT_REG_TCKC(mft));

            /** start to Capture */
            iowrite8(reg_mode, (void *)MFT_REG_TCKC(mft));
        }
        else
        {
            /** enable interrupt */
            iowrite8((UINT8) (reg_int | (NPCM750_TIEN_TBIEN | NPCM750_TIEN_TFIEN)),
                    (void *)MFT_REG_TIEN(mft));

            reg_mode =  NPCM750_TCKC_C2CSEL(NPCM750_MFT_APB_CLOCK_MODE)
                | ioread8((void *)MFT_REG_TCKC(mft));

            /** start to Capture */
            iowrite8(reg_mode, (void *)MFT_REG_TCKC(mft));
        }
    }
}


/******************************************************************************
*   FUNCTION        :   npcm750_fantach_polling
******************************************************************************/
/**
 *  @brief      The polling function is used to
 *              1. change current fan of each channel.
 *              2. Start to capture the fans.
 *
 *  @return     None
 *
 *  @dependency None
 *
 *  @limitation None
 *
 *  @warning    None
 *
 *  @note       None
 *
 *  @internal   Function Type: Private function\n
 *
 *****************************************************************************/
static void npcm750_fantach_polling(

                /** reserved for future used. */
                unsigned long data
                                   )
{
    int i;

    DRV_MSG("npcm750_fantach_polling: %d\n", (int) S_npcm750_fantach_select);
    /* Polling two module per one round, MFT0 & MFT4 / MFT1 & MFT5 / MFT2 & MFT6 / MFT3 & MFT7 */
    for (i = S_npcm750_fantach_select; i < NPCM750_MFT_MAX_MODULE; i=i+4)
    {
        /** clear the flag and reset the counter (TCNT) */
        iowrite8((UINT8) NPCM750_TICLR_CLEAR_ALL, (void *) MFT_REG_TICLR(i));

        iowrite16(NPCM750_MFT_TCNT, (void *)MFT_REG_TCNT1(i));
        iowrite16(NPCM750_MFT_TCNT, (void *)MFT_REG_TCNT2(i));

        npcm750_fantach_start_capture(i, NPCM750_CMPA);
        npcm750_fantach_start_capture(i, NPCM750_CMPB);
    }

    S_npcm750_fantach_select++;
    S_npcm750_fantach_select &= 0x3;

    /** reset the timer interval */
    npcm750_fantach_timer.expires = jiffies + FAN_TACH_POLLING_INTERVAL;
    add_timer(&npcm750_fantach_timer);
}


/******************************************************************************
*   FUNCTION        :   aess_fansensor_open
******************************************************************************/
/**
 *  @brief      Do nothing.
 *
 *  @return     STATUS_OK
 *
 *  @dependency None
 *
 *  @limitation None
 *
 *  @warning    None
 *
 *  @note       None
 *
 *  @internal   Function Type: Private function\n
 *
 *****************************************************************************/
static int aess_fansensor_open(

                /** The inode pointer from the kernel. */
                struct inode *inode,

                /** The file pointer points to this file. */
                struct file *filp

                              )
{
    DRV_INFO("%s is opened.\n", driver_name);

    aess_fansensor_get_input_clock();

    return (STATUS_OK);
}


/******************************************************************************
*   FUNCTION        :   aess_fansensor_config
******************************************************************************/
/**
 *  @brief      Set the fan pulse per revolution.
 *
 *  @return     0
 *
 *  @dependency None
 *
 *  @limitation None
 *
 *  @warning    None
 *
 *  @note       None
 *
 *  @internal   Function Type: Private function\n
 *
 *****************************************************************************/
static int aess_fansensor_config(

                /** the fan tach device information */
                sFanTachData *pFanTachData

                                )
{
    UINT8 fantach_chnl = 0;
    UINT32 tempreg = 0;

    fantach_chnl = pFanTachData->u8ChannelNum;
    DRV_INFO("aess_fansensor_config for channel %d.\n", fantach_chnl);

    /** enable fantach input pin */
    tempreg = ioread32((void *)GLOBAL_REG_PIN_SELECT2_ADDR);
    iowrite32((UINT32) (tempreg | MFT_MFSEL2_FLSEL(fantach_chnl)), (void *)GLOBAL_REG_PIN_SELECT2_ADDR);

    S_npcm750_fantach[fantach_chnl].u8FanStatusFlag = FAN_TACH_INIT;
    S_npcm750_fantach[fantach_chnl].u8FanPulsePerRev = pFanTachData->u8FanPulsePerRev;

    return 0;
}


/******************************************************************************
*   FUNCTION        :   aess_fansensor_read
******************************************************************************/
/**
 *  @brief      Read the fan rotational speed in RPM.
 *
 *  @return     0
 *
 *  @dependency None
 *
 *  @limitation None
 *
 *  @warning    None
 *
 *  @note       None
 *
 *  @internal   Function Type: Private function\n
 *
 *****************************************************************************/
static int aess_fansensor_read(

                /** the fan tach device information */
                sFanTachData *pFanTachData

                              )

{
    UINT8 fan_id = 0;

    fan_id = pFanTachData->u8ChannelNum;

    if (S_npcm750_fantach[fan_id].u16FanTachCnt != 0)
    {
        pFanTachData->u16FanSpeedReading = S_npcm750_fantach[fan_id].u16FanTachCnt;
    }
    else
    {
        pFanTachData->u16FanSpeedReading = 0;
    }

    return  0;
}


/******************************************************************************
*   FUNCTION        :   aess_fansensor_ioctl
******************************************************************************/
/**
 *  @brief      The I/O control function.
 *
 *  @return     0 is ok, otherwise minus value is failed.
 *
 *  @dependency None
 *
 *  @limitation None
 *
 *  @warning    None
 *
 *  @note       None
 *
 *  @internal   Function Type: Private function\n
 *
 *****************************************************************************/
static long aess_fansensor_ioctl(

                /** The file pointer points to the kernel. */
                struct file *flip,

                /** The ioctl command passed from user space. */
                unsigned int cmd,

                /** The argument passed from user space. */
                unsigned long arg

                               )

{
    UINT8 fantach_chnl = 0;
    int err_check = 0;

    sFanTachData FanTachData;
    sFanTachData *pFanTachData = &FanTachData;

	if (copy_from_user(&FanTachData, (void __user *) arg, sizeof(FanTachData)))
	{
		DRV_ERROR("aess fansensor copy_from_user error!!\n");
	    return -EFAULT;
	}

    fantach_chnl = pFanTachData->u8ChannelNum;

    if (fantach_chnl >= NPCM750_MAX_FAN_TACH)
    {
        return (-ENODEV);
    }

    pFanTachData->u32InputClock = u32InputClock;

    switch(cmd)
    {
        case AESS_FANTACH_CONFIG:
            err_check = aess_fansensor_config(pFanTachData);
            break;

        case AESS_FANTACH_READ:
            err_check = aess_fansensor_read(pFanTachData);
            break;

        default:
            DRV_ERROR("aess fansensor ioctl command error\n");

            return (-ENOIOCTLCMD);
    }
    
    if (err_check == 0) {
		if (copy_to_user((void __user *)arg, &FanTachData, sizeof(FanTachData)))
    	{
    		DRV_ERROR("aess fansensor copy_to_user error!!\n");
    	    return -EFAULT;
    	}
    }

    return (err_check);
}


/******************************************************************************
*   FUNCTION        :   aess_fansensor_release
******************************************************************************/
/**
 *  @brief      Release the fan tach device.
 *
 *  @return     STATUS_OK
 *
 *  @dependency None
 *
 *  @limitation None
 *
 *  @warning    None
 *
 *  @note       None
 *
 *  @internal   Function Type: Private function\n
 *
 *****************************************************************************/
static int aess_fansensor_release(

                /** The inode pointer from the kernel. */
                struct inode *inode,

                /** The file pointer points to the kernel. */
                struct file *flip

                                 )

{
    int i;

    for (i = 0; i < NPCM750_MFT_MAX_MODULE; i++)
    {
        /** Disable all interrupts */
        iowrite8((UINT8) 0x00, (void *)MFT_REG_TIEN(i));
    }

    return (STATUS_OK);
}


static struct file_operations aess_fansensor_fops =
{
    .open = aess_fansensor_open,
    .unlocked_ioctl = aess_fansensor_ioctl,
    .release = aess_fansensor_release,
};


/******************************************************************************
*   FUNCTION        :   npcm750_fantach_compute
******************************************************************************/
/**
 *  @brief      Compute the fan rotational speed.
 *              This function is invoked when TnAPND or TnBPND is set.
 *
 *  @return     None
 *
 *  @dependency None
 *
 *  @limitation None
 *
 *  @warning    None
 *
 *  @note       None
 *
 *  @internal   Function Type: Private function\n
 *
 *****************************************************************************/
static inline void npcm750_fantach_compute(

                /** the MFT module */
                UINT8 mft,

                /** the timer of the specifc MFT */
                UINT8 cmp,

                /** the selected fan tach number */
                UINT8 fan_id,

                /** the interrupt flag from the specific fan */
                UINT8 flag_int,

                /** the clock mode flag */
                UINT8 flag_mode,

                /** the interrupt clear flag */
                UINT8 flag_clear

                                          )
{
    volatile UINT8  reg_int  = 0;
    volatile UINT8  reg_mode = 0;
    volatile UINT16 fan_cap  = 0;

    if (NPCM750_CMPA == cmp)
    {
        fan_cap = ioread16((void *) MFT_REG_TCRA(mft));
    }
    else
    {
        fan_cap = ioread16((void *) MFT_REG_TCRB(mft));
    }

    /** clear capature flag, H/W will auto reset the NPCM750_TCNTx */
    iowrite8((UINT8) flag_clear, (void *) MFT_REG_TICLR(mft));

    if (FAN_TACH_INIT == S_npcm750_fantach[fan_id].u8FanStatusFlag)
    {
        /** First capture, drop it */
        S_npcm750_fantach[fan_id].u8FanStatusFlag
            = FAN_TACH_PREPARE_TO_GET_FIRST_CAPTURE;

        /** reset counter */
        S_npcm750_fantach[fan_id].u32FanTachCntTemp = 0;
    }
    else if (S_npcm750_fantach[fan_id].u8FanStatusFlag < FAN_TACH_ENOUGH_SAMPLE)
    {
        /** collect the enough sampe, (ex: 2 pulse fan need to get 2 sample) */
        S_npcm750_fantach[fan_id].u32FanTachCntTemp += (NPCM750_MFT_TCNT - fan_cap);

        DRV_MSG("step 1, fan %d cnt %d total %x \n",
                fan_id, (NPCM750_MFT_TCNT - fan_cap),
                (unsigned int) S_npcm750_fantach[fan_id].u32FanTachCntTemp);

        S_npcm750_fantach[fan_id].u8FanStatusFlag++;


    }
    else
    {
        /** get enough sample or fan disable (u8FanStatusFlag = FAN_TACH_DISABLE) */
        if (S_npcm750_fantach[fan_id].u8FanStatusFlag == FAN_TACH_ENOUGH_SAMPLE)
        {
            S_npcm750_fantach[fan_id].u32FanTachCntTemp += (NPCM750_MFT_TCNT - fan_cap);

            DRV_MSG("step 2, fan %d cnt %d total %x \n",
                    fan_id, (NPCM750_MFT_TCNT - fan_cap),
                    (unsigned int) S_npcm750_fantach[fan_id].u32FanTachCntTemp);

            /** compute finial average cnt per pulse */
            S_npcm750_fantach[fan_id].u16FanTachCnt
                = S_npcm750_fantach[fan_id].u32FanTachCntTemp / FAN_TACH_ENOUGH_SAMPLE;

            DRV_MSG("step 3 fan %d avg %d\n\n",
                    fan_id, S_npcm750_fantach[fan_id].u16FanTachCnt);

            S_npcm750_fantach[fan_id].u8FanStatusFlag = FAN_TACH_INIT;
        }

        reg_int =  ioread8((void *)MFT_REG_TIEN(mft));

        /** disable interrupt */
        iowrite8((UINT8) (reg_int & ~flag_int), (void *)MFT_REG_TIEN(mft));

        reg_mode =  ioread8((void *)MFT_REG_TCKC(mft));

        /** stop capturing */
        iowrite8((UINT8) (reg_mode & ~flag_mode), (void *) MFT_REG_TCKC(mft));
    }
}


/******************************************************************************
*   FUNCTION        :   npcm750_check_cmp
******************************************************************************/
/**
 *  @brief      Check different timers when IRQ occurs.
 *
 *  @return     None
 *
 *  @dependency None
 *
 *  @limitation None
 *
 *  @warning    None
 *
 *  @note       None
 *
 *  @internal   Function Type: Private function\n
 *
 *****************************************************************************/
static inline void npcm750_check_cmp(

                /** the MFT module */
                UINT8 mft,

                /** the timer of the specifc MFT */
                UINT8 cmp,

                /** the interrupt flag */
                UINT8 flag

                                    )

{
    volatile UINT8 reg_int = 0;
    volatile UINT8 reg_mode = 0;
    volatile UINT8 flag_timeout;
    volatile UINT8 flag_cap;
    volatile UINT8 flag_clear;
    volatile UINT8 flag_int;
    volatile UINT8 flag_mode;
    volatile UINT8 fan_id;

    fan_id = NPCM750_FAN_TACH_INPUT(mft, cmp);

    if (NPCM750_CMPA == cmp)
    {
        flag_cap = NPCM750_TICTRL_TAPND;
        flag_timeout = NPCM750_TICTRL_TEPND;
        flag_int = (NPCM750_TIEN_TAIEN | NPCM750_TIEN_TEIEN);
        flag_mode = NPCM750_TCKC_C1CSEL(NPCM750_MFT_APB_CLOCK_MODE);
        flag_clear = NPCM750_TICLR_TACLR | NPCM750_TICLR_TECLR;
    }
    else
    {
        flag_cap = NPCM750_TICTRL_TBPND;
        flag_timeout = NPCM750_TICTRL_TFPND;
        flag_int = (NPCM750_TIEN_TBIEN | NPCM750_TIEN_TFIEN);
        flag_mode = NPCM750_TCKC_C2CSEL(NPCM750_MFT_APB_CLOCK_MODE);
        flag_clear = NPCM750_TICLR_TBCLR | NPCM750_TICLR_TFCLR;
    }

    if (flag & flag_timeout)
    {
        reg_int =  ioread8((void *)MFT_REG_TIEN(mft));

        /** disable interrupt */
        iowrite8((UINT8) (reg_int & ~flag_int), (void *)MFT_REG_TIEN(mft));

        /** clear interrup flag */
        iowrite8((UINT8) flag_clear, (void *) MFT_REG_TICLR(mft));

        reg_mode =  ioread8((void *)MFT_REG_TCKC(mft));

        /** stop capturing */
        iowrite8((UINT8) (reg_mode & ~flag_mode), (void *) MFT_REG_TCKC(mft));

        /**
         *  If timeout occurs (FAN_TACH_TIMEOUT), the fan doesn't connect
         *  or speed is lower than 10.6Hz (320RPM/pulse2).
         *  In these situation, the RPM output should be zero.
         */
        S_npcm750_fantach[fan_id].u16FanTachCnt = 0;

        DRV_WARN("npcm750_check_cmp : it is timeout fan_id %d \n", fan_id);
    }
    else
    {
        /** input capture is occurred */
        if (flag & flag_cap)
        {
            npcm750_fantach_compute(mft, cmp, fan_id, flag_int, flag_mode, flag_clear);
        }
    }
}


/******************************************************************************
*   FUNCTION        :   npcm750_mft0_isr
******************************************************************************/
/**
 *  @brief      The ISR for MFT0.
 *
 *  @return     IRQ_HANDLED
 *
 *  @dependency None
 *
 *  @limitation None
 *
 *  @warning    None
 *
 *  @note       None
 *
 *  @internal   Function Type: Private function\n
 *
 *****************************************************************************/
static irqreturn_t npcm750_mft0_isr(

                /** the irq number */
                int irq,

                /** the client data */
                void *dev_id

                                   )
{
    volatile UINT8 flag = 0;
    volatile int module;

    module = irq - MFT_INTERRUPT_0;
    flag = ioread8((void *)(void *) MFT_REG_TICTRL(module));
    if (flag > 0)
    {
        npcm750_check_cmp(module, NPCM750_CMPA, flag);
        npcm750_check_cmp(module, NPCM750_CMPB, flag);
        return (IRQ_HANDLED);
    }

    return (IRQ_NONE);
}

/******************************************************************************
*   FUNCTION        :   npcm750_fantach_init
******************************************************************************/
/**
 *  @brief      Initialize hardware registers during initialized phase.
 *
 *  @return     0
 *
 *  @dependency None
 *
 *  @limitation None
 *
 *  @warning    None
 *
 *  @note       None
 *
 *  @internal   Function Type: Private function\n
 *
 *****************************************************************************/
static int npcm750_fantach_init(void)
{
    int i;
    UINT32 reg_value;
	int ret = 0;
 
#ifdef CONFIG_OF
		struct device_node *np=NULL;
		struct resource res;
		struct platform_device *pdev = NULL; 
		
		np = of_find_compatible_node(NULL, NULL, "nuvoton,npcm750-mft");
		if (np==NULL){
			pr_info("Failed to find of_find_matching_node\n");
			return -ENODEV;				
		}
		ret = of_address_to_resource(np, 0, &res);
		if (ret) {
			pr_info("\t\t\t of_address_to_resource fail ret %d \n",ret);
			return -EINVAL;
		}
		
		mft_virt_addr = (int)ioremap(res.start, resource_size(&res));
		
		if (!mft_virt_addr) {
			pr_info("\t\t\t mft_virt_addr fail \n");
			return -ENOMEM;
		}
		printk(KERN_INFO "\t\t\t* MFT base is 0x%08X ,res.start 0x%08X \n", (u32)mft_virt_addr,res.start);
		
		
		pdev = of_find_device_by_node(np);
		 
		if (IS_ERR(pdev))	
		{
            pr_err(" MFT (FAN) probe failed: can't get platform device.\n");           
            return -ENODEV;
		}

		mft_clk = devm_clk_get(&pdev->dev, NULL);
		
		if (IS_ERR(mft_clk))	
		{
			pr_err(" MFT (FAN) probe failed: can't read clk.\n");			
			return -ENODEV;
		}

		clk_prepare_enable(mft_clk);    
#endif

    for (i = 0; i < NPCM750_MFT_MAX_MODULE; i++)
    {
        /** stop MFT0~7 clock */
        iowrite8((UINT8) NPCM750_MFT_NO_CLOCK_MODE, (void *)MFT_REG_TCKC(i));

        /** disable all interrupt */
        iowrite8((UINT8) 0x00, (void *)MFT_REG_TIEN(i));

        /** clear all interrupt */
        iowrite8((UINT8) NPCM750_TICLR_CLEAR_ALL, (void *)MFT_REG_TICLR(i));

        /** set MFT0~7 clock prescaler */
        iowrite8((UINT8) NPCM750_MFT_CLKPS, (void *)MFT_REG_TPRSC(i));

        /** set MFT0~7 mode (high-to-low transition) */
        iowrite8(
                    (UINT8) (
                        NPCM750_TMCTRL_MDSEL(NPCM750_MFT_MODE_5) |
                        NPCM750_TMCTRL_TBEN |
                        NPCM750_TMCTRL_TAEN
                            ),
                    (void *) MFT_REG_TMCTRL(i)
                );

        /** set MFT0~7 Initial Count/Cap */
        iowrite16(NPCM750_MFT_TCNT, (void *)MFT_REG_TCNT1(i));
        iowrite16(NPCM750_MFT_TCNT, (void *)MFT_REG_TCNT2(i));

        /** set MFT0~7 compare (equal to count) */
        iowrite8( (UINT8) (NPCM750_TCPCFG_EQAEN | NPCM750_TCPCFG_EQBEN),
                  (void *)MFT_REG_TCPCFG(i));

        /** set MFT0~7 compare value */
        iowrite16(NPCM750_MFT_TCPA, (void *)MFT_REG_TCPA(i));
        iowrite16(NPCM750_MFT_TCPB, (void *)MFT_REG_TCPB(i));

        /** set MFT0~7 fan input FANIN 0~15 */
        iowrite8((UINT8) NPCM750_TINASEL_FANIN_DEFAULT, (void *)MFT_REG_TINASEL(i));
        iowrite8((UINT8) NPCM750_TINASEL_FANIN_DEFAULT, (void *)MFT_REG_TINBSEL(i));

    }

    /** fan tach structure initialization */
    S_npcm750_fantach_select = 0;
    for (i = 0; i < NPCM750_MAX_FAN_TACH; i++)
    {
        S_npcm750_fantach[i].u8FanStatusFlag = FAN_TACH_DISABLE;
        S_npcm750_fantach[i].u8FanPulsePerRev = DEFAULT_PULSE_PER_REVOLUTION;
        S_npcm750_fantach[i].u16FanTachCnt = 0;
    }

    /** request IRQ */
    reg_value = ioread32((void *)AIC_REG_GEN_ADDR);
    reg_value |= 0xff;
    iowrite32(reg_value, (void *)AIC_REG_GEN_ADDR);

#ifdef CONFIG_OF
    for (i = 0; i < 8; i++) {
        mft_irq[i] = irq_of_parse_and_map(np, i);
        if (!mft_irq[i]) {
            printk(KERN_ERR "%s - failed to map irq %d\n", __FUNCTION__, i);
            return (-EAGAIN);
        }
    }
#endif

    if (request_irq(MFT_INTERRUPT_0, (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT0", (void *) &u8dummy))
    {
        DRV_ERROR("NPCM750: register irq MFT0 failed\n");
        return (-EAGAIN);
    }

    if (request_irq(MFT_INTERRUPT_1, (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT1", (void *) &u8dummy))
    {
        DRV_ERROR("NPCM750: register irq MFT1 failed\n");
        free_irq(MFT_INTERRUPT_0, (void *) &u8dummy);
        return (-EAGAIN);
    }

    if (request_irq(MFT_INTERRUPT_2, (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT2", (void *) &u8dummy))
    {
        DRV_ERROR("NPCM750: register irq MFT2 failed\n");
        free_irq(MFT_INTERRUPT_0, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_1, (void *) &u8dummy);
        return (-EAGAIN);
    }

    if (request_irq(MFT_INTERRUPT_3, (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT3", (void *) &u8dummy))
    {
        DRV_ERROR("NPCM750: register irq MFT3 failed\n");
        free_irq(MFT_INTERRUPT_0, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_1, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_2, (void *) &u8dummy);
        return (-EAGAIN);
    }

    if (request_irq(MFT_INTERRUPT_4, (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT4", (void *) &u8dummy))
    {
        DRV_ERROR("NPCM750: register irq MFT4 failed\n");
        free_irq(MFT_INTERRUPT_0, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_1, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_2, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_3, (void *) &u8dummy);
        return (-EAGAIN);
    }

    if (request_irq(MFT_INTERRUPT_5, (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT5", (void *) &u8dummy))
    {
        DRV_ERROR("NPCM750: register irq MFT5 failed\n");
        free_irq(MFT_INTERRUPT_0, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_1, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_2, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_3, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_4, (void *) &u8dummy);
        return (-EAGAIN);
    }

    if (request_irq(MFT_INTERRUPT_6, (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT6", (void *) &u8dummy))
    {
        DRV_ERROR("NPCM750: register irq MFT6 failed\n");
        free_irq(MFT_INTERRUPT_0, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_1, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_2, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_3, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_4, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_5, (void *) &u8dummy);
        return (-EAGAIN);
    }

    if (request_irq(MFT_INTERRUPT_7, (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT7", (void *) &u8dummy))
    {
        DRV_ERROR("NPCM750: register irq MFT7 failed\n");
        free_irq(MFT_INTERRUPT_0, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_1, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_2, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_3, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_4, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_5, (void *) &u8dummy);
        free_irq(MFT_INTERRUPT_6, (void *) &u8dummy);
        return (-EAGAIN);
    }

    /** initialize fan tach polling timer */
    npcm750_fantach_timer.data = 0;
    npcm750_fantach_timer.function = &npcm750_fantach_polling;

    /** set timer interval */
    npcm750_fantach_timer.expires = jiffies + FAN_TACH_POLLING_INTERVAL;

    init_timer(&npcm750_fantach_timer);
    add_timer(&npcm750_fantach_timer);

    return ret;
}


/******************************************************************************
*   FUNCTION        :   aess_fansensor_init
******************************************************************************/
/**
 *  @brief      The module initializaed function
 *
 *  @return     STATUS_OK
 *
 *  @dependency None
 *
 *  @limitation None
 *
 *  @warning    None
 *
 *  @note       None
 *
 *  @internal   Function Type: Private function\n
 *
 *****************************************************************************/
int __init aess_fansensor_init(void)
{
    int result;
		
    DRV_INFO("Init aess fansensor module success!\n");
    printk("%s: Enter \n",__func__);

    fansensor_cdev = cdev_alloc();

    fansensor_cdev->ops = &aess_fansensor_fops;
	
	if (majornum <= 0)
	{
		/* allocate a device number */
		result = alloc_chrdev_region(&dev, 0, 1, driver_name);
		majornum = MAJOR(dev);
	}
	else
	{
		dev = MKDEV(majornum, 0);
		result = register_chrdev_region(dev, 1, driver_name);
	}

    printk("mknod /dev/aess_fansensordrv c %d 0\n", MAJOR(dev));

    if (result < 0)
    {
        DRV_ERROR("Register the char device failed with %d\n", MAJOR(dev));
        return (result);
    }

    cdev_add(fansensor_cdev, dev, 1);

    result = npcm750_fantach_init();

    if (result < 0)
    {
        printk("ERROR: %s failed\n", __func__);
        return (result);
    }

#ifdef CONFIG_DEBUG_FS

    /* Create Debugfs */
    aess_debugfs_default_create(driver_name, NULL, 0, 0);
    aess_debugfs_create_file(driver_name, "u32", (u32 *) &u32tmpcounter, DBG_TYPE32, 0);

#endif

    /** - Register driver with sysfs class. */
    sys_class = class_create(THIS_MODULE, driver_name);
    device_create(sys_class, NULL, dev, NULL, driver_name);

    /** - Register sysfs. */
    result = platform_driver_register(&aess_fan_device_driver);

    if (result)
    {
    	printk(KERN_ERR "%s(): can't register sysfs.\n", __FUNCTION__);
    	return result;
    }

    result = driver_create_file(&aess_fan_device_driver.driver, &fan_flag);

    if (result)
    {
    	printk(KERN_ERR "%s(): Fail to create sysfs attrb.\n", __FUNCTION__);
    	return result;
    }
    printk("%s is success.\n", __func__);
    return (STATUS_OK);
}


/******************************************************************************
*   FUNCTION        :   aess_fansensor_exit
******************************************************************************/
/**
 *  @brief      The module exited function
 *
 *  @return     None
 *
 *  @dependency None
 *
 *  @limitation None
 *
 *  @warning    None
 *
 *  @note       None
 *
 *  @internal   Function Type: Private function\n
 *
 *****************************************************************************/
void __exit aess_fansensor_exit(void)
{
    int i;

    DRV_INFO("aess fansensor exit\n");

    for (i = 0; i < NPCM750_MFT_MAX_MODULE; i++)
    {
        /** stop MFT0/MFT1 clock */
        iowrite8((UINT8) NPCM750_MFT_NO_CLOCK_MODE, (void *)MFT_REG_TCKC(i));

        /** disable all interrupts */
        iowrite8((UINT8) 0x00, (void *)MFT_REG_TIEN(i));
    }

    /** free IRQs **/
    free_irq(MFT_INTERRUPT_0, (void *) &u8dummy);
    free_irq(MFT_INTERRUPT_1, (void *) &u8dummy);
    free_irq(MFT_INTERRUPT_2, (void *) &u8dummy);
    free_irq(MFT_INTERRUPT_3, (void *) &u8dummy);
    free_irq(MFT_INTERRUPT_4, (void *) &u8dummy);
    free_irq(MFT_INTERRUPT_5, (void *) &u8dummy);
    free_irq(MFT_INTERRUPT_6, (void *) &u8dummy);
    free_irq(MFT_INTERRUPT_7, (void *) &u8dummy);

    /** delete fan tach polling timer */
    del_timer(&npcm750_fantach_timer);

#ifdef CONFIG_DEBUG_FS
    /** remove debugfs */
    aess_debugfs_remove(driver_name, NULL, 0, 0);
#endif

    /** - Destory sysfs class. */
    device_destroy(sys_class, dev);
    class_destroy(sys_class);

    /** - Unregister FAN sensor driver from the kernel. */
    cdev_del(fansensor_cdev);
    unregister_chrdev_region(dev,1);

    /** - Remove sysfs. */
    driver_remove_file(&aess_fan_device_driver.driver, &fan_flag);
    platform_driver_unregister(&aess_fan_device_driver);
}

MODULE_DESCRIPTION("AESS Fan Sensor Driver");
MODULE_AUTHOR("Spencer Tsai <Spencer.Tsai@Emerson.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.0.0.4");
module_param(driver_name, charp, S_IRUGO);
module_init(aess_fansensor_init);
module_exit(aess_fansensor_exit);

/* End of code */
