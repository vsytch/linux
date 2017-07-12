/*
* Copyright (c) 2013-2017 by Nuvoton Technology Corporation. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify it under the terms of the
* GNU General Public License Version 2 (or later) as published by the Free Software Foundation.
*/

#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/signal.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/types.h>
#include <asm/uaccess.h>

#include <mach/hal.h>

typedef struct
{
    /** Set Fan Channel no. (zero-based) */
    u8 u8ChannelNum;

    /** Pulse Revolution */
    u8 u8FanPulsePerRev;

    /** Fan Speed Reading (unit: ticks) */
    u16 u16FanSpeedReading;

    /** Input Clock */
    u32 u32InputClock;
} sFanTachData;

#define STATUS_OK  0
#define STATUS_FAIL  1

#define NPCM750_MFT_CLKPS   255

/* PLL input */
#define PLL_INPUT_CLOCK  25000000

/* Get Fan Tach Timeout (base on clock 214843.75Hz, 1 cnt = 4.654us)
   Timeout 94ms ~= 0x5000
   (The minimum FAN speed could to support ~640RPM/pulse 1, 320RPM/pulse 2, ...-- 10.6Hz)
 */
 #define FAN_TACH_TIMEOUT   ((u16) 0x5000)

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

#define NPCM750_MFT_TCNT    ((u16) 0xFFFF)
#define NPCM750_MFT_TCPA    ((u16) (NPCM750_MFT_TCNT - FAN_TACH_TIMEOUT))
#define NPCM750_MFT_TCPB    ((u16) (NPCM750_MFT_TCNT - FAN_TACH_TIMEOUT))

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
#define CLK_BA_REG_BASE_ADDR            NPCMX50_CLK_BASE_ADDR

#define CLK_BA_REG_CLKSEL_ADDR          (CLK_BA_REG_BASE_ADDR + 0x4)
#define CLK_BA_REG_CLKDIV1_ADDR         (CLK_BA_REG_BASE_ADDR + 0x8)
#define CLK_BA_REG_PLLCON0_ADDR         (CLK_BA_REG_BASE_ADDR + 0xc)
#define CLK_BA_REG_PLLCON1_ADDR         (CLK_BA_REG_BASE_ADDR + 0x10)
#define CLK_BA_REG_CLKDIV2_ADDR         (CLK_BA_REG_BASE_ADDR + 0x2C)

/* Debugging Message */
#ifdef FAN_DEBUG
#define DEBUG_MSG(fmt, args...)  printk("PWM: %s() " fmt, __func__ , ##args)
#else
#define DEBUG_MSG(fmt, args...)
#endif
#define PERROR(fmt, args...)  printk("PWM: %s() " fmt, __func__ , ##args)

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
    u8 u8FanStatusFlag;

    /** Pulse Revolution */
    u8 u8FanPulsePerRev;

    /** Fan Tach Count
       (unit: Base on the APB freq. [default: 55Mhz]
       and prescaler TPRSC   [default: 256]) */
    u16 u16FanTachCnt;

    /** Fan Tach Count Temp */
    u32 u32FanTachCntTemp;
} sFanTachDev;


/******************************************************************************
*   STRUCT      :   Function Prototype
******************************************************************************/
/**
 *  @brief   Prototype for each private function.
 *
 *****************************************************************************/

static int npcm750_fan_read(sFanTachData *pFanTachData);

static int mft_virt_addr;

#undef MFT_REGS_BASE
#define MFT_REGS_BASE(n)    (mft_virt_addr + ((n) * 0x1000L))

/* for request irq use */
static u8 u8dummy = 0;
int mft_irq[8];

/* Input clock */
static u32 u32InputClock = 0;

static volatile sFanTachDev S_npcm750_fantach[NPCM750_MAX_FAN_TACH];
static u8 S_npcm750_fantach_select;

static struct timer_list npcm750_fantach_timer;

static struct clk* mft_clk;

struct npcm750_fan_data {
	unsigned long clk_freq;
	const struct attribute_group *groups[2];
};

static ssize_t show_rpm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
    sFanTachData FanTachData;

    FanTachData.u8ChannelNum = index;
    npcm750_fan_read(&FanTachData);
	if (FanTachData.u16FanSpeedReading < 0)
		return FanTachData.u16FanSpeedReading;

	return sprintf(buf, "%d\n", (int)FanTachData.u16FanSpeedReading);
}

static umode_t fan_dev_is_visible(struct kobject *kobj,
				  struct attribute *a, int index)
{
	/*struct device *dev = container_of(kobj, struct device, kobj);*/

	return a->mode;
}

static SENSOR_DEVICE_ATTR(fan1_input, 0444,
		show_rpm, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_input, 0444,
		show_rpm, NULL, 1);
static SENSOR_DEVICE_ATTR(fan3_input, 0444,
		show_rpm, NULL, 2);
static SENSOR_DEVICE_ATTR(fan4_input, 0444,
		show_rpm, NULL, 3);
static SENSOR_DEVICE_ATTR(fan5_input, 0444,
		show_rpm, NULL, 4);
static SENSOR_DEVICE_ATTR(fan6_input, 0444,
		show_rpm, NULL, 5);
static SENSOR_DEVICE_ATTR(fan7_input, 0444,
		show_rpm, NULL, 6);
static SENSOR_DEVICE_ATTR(fan8_input, 0444,
		show_rpm, NULL, 7);
static SENSOR_DEVICE_ATTR(fan9_input, 0444,
		show_rpm, NULL, 8);
static SENSOR_DEVICE_ATTR(fan10_input, 0444,
		show_rpm, NULL, 9);
static SENSOR_DEVICE_ATTR(fan11_input, 0444,
		show_rpm, NULL, 10);
static SENSOR_DEVICE_ATTR(fan12_input, 0444,
		show_rpm, NULL, 11);
static SENSOR_DEVICE_ATTR(fan13_input, 0444,
		show_rpm, NULL, 12);
static SENSOR_DEVICE_ATTR(fan14_input, 0444,
		show_rpm, NULL, 13);
static SENSOR_DEVICE_ATTR(fan15_input, 0444,
		show_rpm, NULL, 14);
static SENSOR_DEVICE_ATTR(fan16_input, 0444,
		show_rpm, NULL, 15);
static struct attribute *fan_dev_attrs[] = {
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan3_input.dev_attr.attr,
	&sensor_dev_attr_fan4_input.dev_attr.attr,
	&sensor_dev_attr_fan5_input.dev_attr.attr,
	&sensor_dev_attr_fan6_input.dev_attr.attr,
	&sensor_dev_attr_fan7_input.dev_attr.attr,
	&sensor_dev_attr_fan8_input.dev_attr.attr,
	&sensor_dev_attr_fan9_input.dev_attr.attr,
	&sensor_dev_attr_fan10_input.dev_attr.attr,
	&sensor_dev_attr_fan11_input.dev_attr.attr,
	&sensor_dev_attr_fan12_input.dev_attr.attr,
	&sensor_dev_attr_fan13_input.dev_attr.attr,
	&sensor_dev_attr_fan14_input.dev_attr.attr,
	&sensor_dev_attr_fan15_input.dev_attr.attr,
	&sensor_dev_attr_fan16_input.dev_attr.attr,
	NULL
};

static const struct attribute_group fan_dev_group = {
	.attrs = fan_dev_attrs,
	.is_visible = fan_dev_is_visible,
};

/******************************************************************************
*   FUNCTION        :   npcm750_fan_get_input_clock
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
static void npcm750_fan_get_input_clock(void)
{
    unsigned int apb_clk_src = clk_get_rate(mft_clk);
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
    pr_info("[FAN]: CPUCKSEL: 0x%lx\n", reg_value);

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
            pr_err("Error CPUCKSEL!!\n");
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

    pr_info("[FAN]: INDV : 0x%x\n", indv);
    pr_info("[FAN]: OTDV1: 0x%x\n", otdv1);
    pr_info("[FAN]: OTDV2: 0x%x\n", otdv2);
    pr_info("[FAN]: FBDV : 0x%x\n", fbdv);

    apb_clk_src = PLL_INPUT_CLOCK * fbdv / indv / (otdv1 * otdv2);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Warning: previous code had these 3 lines missing, so APB4 was mincalculeted in the first place..    */
    /*-----------------------------------------------------------------------------------------------------*/
    if ((ioread32((void *)CLK_BA_REG_CLKSEL_ADDR) & 0x3)  == 1)    {
        apb_clk_src = apb_clk_src >> 1; 
    }

    pr_info("[FAN] PLL: %d\n", (int) apb_clk_src);

    /* AXI clock divider */
    reg_value = ioread32((void *)CLK_BA_REG_CLKDIV1_ADDR);
    pr_info("[FAN]: CLKDIV1: 0x%lx\n", reg_value);

    axidiv = (reg_value & 0x1);
    apb_clk_src = apb_clk_src >> axidiv;
    
    /* AHB clock divider */
    ahbdiv = (reg_value & 0x0C000000) >> 26;
    apb_clk_src = apb_clk_src >> ahbdiv;

    /* APB clock divider */
    reg_value = ioread32((void *)CLK_BA_REG_CLKDIV2_ADDR);
    pr_info("[FAN]: CLKDIV2: 0x%lx\n", reg_value);

    apbdiv = (reg_value & 0xC0000000) >> 30;
    apb_clk_src = apb_clk_src >> apbdiv;

    pr_info("[FAN] APB4: %d\n", (int)apb_clk_src);
    /* Fan tach input clock = APB clock / prescalar, default is 255. */
    u32InputClock = apb_clk_src / (NPCM750_MFT_CLKPS + 1);

    pr_info("[FAN] PWM: %d\n", (int) u32InputClock);
    pr_info("[FAN] InputClock: %d\n", (int) u32InputClock);

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
                u8 mft,

                /** the timer of the specifc MFT */
                u8 cmp
                                                )
{
    u8 fan_id = 0;
    u8 reg_mode = 0;
    u8 reg_int = 0;

    fan_id = NPCM750_FAN_TACH_INPUT(mft, cmp);

    /** to check whether any fan tach is enable */
    if (FAN_TACH_DISABLE != S_npcm750_fantach[fan_id].u8FanStatusFlag)
    {
        //DRV_MSG("npcm750_fantach_start_capture: %d\n", (int) fan_id);
        /** reset status */
        S_npcm750_fantach[fan_id].u8FanStatusFlag = FAN_TACH_INIT;

        reg_int = ioread8((void *)MFT_REG_TIEN(mft));

        if (NPCM750_CMPA == cmp)
        {
            /** enable interrupt */
            iowrite8((u8) (reg_int | (NPCM750_TIEN_TAIEN | NPCM750_TIEN_TEIEN)),
                    (void *)MFT_REG_TIEN(mft));

            reg_mode =  NPCM750_TCKC_C1CSEL(NPCM750_MFT_APB_CLOCK_MODE)
                | ioread8((void *)MFT_REG_TCKC(mft));

            /** start to Capture */
            iowrite8(reg_mode, (void *)MFT_REG_TCKC(mft));
        }
        else
        {
            /** enable interrupt */
            iowrite8((u8) (reg_int | (NPCM750_TIEN_TBIEN | NPCM750_TIEN_TFIEN)),
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

    //DRV_MSG("npcm750_fantach_polling: %d\n", (int) S_npcm750_fantach_select);
    /* Polling two module per one round, MFT0 & MFT4 / MFT1 & MFT5 / MFT2 & MFT6 / MFT3 & MFT7 */
    for (i = S_npcm750_fantach_select; i < NPCM750_MFT_MAX_MODULE; i=i+4)
    {
        /** clear the flag and reset the counter (TCNT) */
        iowrite8((u8) NPCM750_TICLR_CLEAR_ALL, (void *) MFT_REG_TICLR(i));

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
*   FUNCTION        :   npcm750_fan_read
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
static int npcm750_fan_read(

                /** the fan tach device information */
                sFanTachData *pFanTachData

                              )

{
    u8 fan_id = 0;

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
                u8 mft,

                /** the timer of the specifc MFT */
                u8 cmp,

                /** the selected fan tach number */
                u8 fan_id,

                /** the interrupt flag from the specific fan */
                u8 flag_int,

                /** the clock mode flag */
                u8 flag_mode,

                /** the interrupt clear flag */
                u8 flag_clear

                                          )
{
    volatile u8  reg_int  = 0;
    volatile u8  reg_mode = 0;
    volatile u16 fan_cap  = 0;

    if (NPCM750_CMPA == cmp)
    {
        fan_cap = ioread16((void *) MFT_REG_TCRA(mft));
    }
    else
    {
        fan_cap = ioread16((void *) MFT_REG_TCRB(mft));
    }

    /** clear capature flag, H/W will auto reset the NPCM750_TCNTx */
    iowrite8((u8) flag_clear, (void *) MFT_REG_TICLR(mft));

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

        /*DRV_MSG("step 1, fan %d cnt %d total %x \n",
                fan_id, (NPCM750_MFT_TCNT - fan_cap),
                (unsigned int) S_npcm750_fantach[fan_id].u32FanTachCntTemp);*/

        S_npcm750_fantach[fan_id].u8FanStatusFlag++;


    }
    else
    {
        /** get enough sample or fan disable (u8FanStatusFlag = FAN_TACH_DISABLE) */
        if (S_npcm750_fantach[fan_id].u8FanStatusFlag == FAN_TACH_ENOUGH_SAMPLE)
        {
            S_npcm750_fantach[fan_id].u32FanTachCntTemp += (NPCM750_MFT_TCNT - fan_cap);

            /*DRV_MSG("step 2, fan %d cnt %d total %x \n",
                    fan_id, (NPCM750_MFT_TCNT - fan_cap),
                    (unsigned int) S_npcm750_fantach[fan_id].u32FanTachCntTemp);*/

            /** compute finial average cnt per pulse */
            S_npcm750_fantach[fan_id].u16FanTachCnt
                = S_npcm750_fantach[fan_id].u32FanTachCntTemp / FAN_TACH_ENOUGH_SAMPLE;

            /*DRV_MSG("step 3 fan %d avg %d\n\n",
                    fan_id, S_npcm750_fantach[fan_id].u16FanTachCnt);*/

            S_npcm750_fantach[fan_id].u8FanStatusFlag = FAN_TACH_INIT;
        }

        reg_int =  ioread8((void *)MFT_REG_TIEN(mft));

        /** disable interrupt */
        iowrite8((u8) (reg_int & ~flag_int), (void *)MFT_REG_TIEN(mft));

        reg_mode =  ioread8((void *)MFT_REG_TCKC(mft));

        /** stop capturing */
        iowrite8((u8) (reg_mode & ~flag_mode), (void *) MFT_REG_TCKC(mft));
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
                u8 mft,

                /** the timer of the specifc MFT */
                u8 cmp,

                /** the interrupt flag */
                u8 flag

                                    )

{
    volatile u8 reg_int = 0;
    volatile u8 reg_mode = 0;
    volatile u8 flag_timeout;
    volatile u8 flag_cap;
    volatile u8 flag_clear;
    volatile u8 flag_int;
    volatile u8 flag_mode;
    volatile u8 fan_id;

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
        iowrite8((u8) (reg_int & ~flag_int), (void *)MFT_REG_TIEN(mft));

        /** clear interrup flag */
        iowrite8((u8) flag_clear, (void *) MFT_REG_TICLR(mft));

        reg_mode =  ioread8((void *)MFT_REG_TCKC(mft));

        /** stop capturing */
        iowrite8((u8) (reg_mode & ~flag_mode), (void *) MFT_REG_TCKC(mft));

        /**
         *  If timeout occurs (FAN_TACH_TIMEOUT), the fan doesn't connect
         *  or speed is lower than 10.6Hz (320RPM/pulse2).
         *  In these situation, the RPM output should be zero.
         */
        S_npcm750_fantach[fan_id].u16FanTachCnt = 0;
        DEBUG_MSG("npcm750_check_cmp : it is timeout fan_id %d \n", fan_id);
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
    volatile u8 flag = 0;
    volatile int module;

    module = irq - mft_irq[0];
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
static int npcm750_fan_probe(struct platform_device *pdev)
{
    u32 reg_value;
	int ret = 0;
    struct device *dev = &pdev->dev;
    struct device_node *np;
    struct npcm750_fan_data *priv;
    struct resource res;
    struct device *hwmon;
    int i;

    priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    np = dev->of_node;
	
	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		pr_err("\t\t\t of_address_to_resource fail ret %d \n",ret);
		return -EINVAL;
	}
	
	mft_virt_addr = (int)ioremap(res.start, resource_size(&res));
	
	if (!mft_virt_addr) {
		pr_err("\t\t\t mft_virt_addr fail \n");
		return -ENOMEM;
	}

    DEBUG_MSG("MFT base is 0x%08X ,res.start 0x%08X \n", (u32)mft_virt_addr,res.start);
	
	mft_clk = devm_clk_get(&pdev->dev, NULL);
	
	if (IS_ERR(mft_clk))	
	{
		pr_err(" MFT (FAN) probe failed: can't read clk.\n");			
		return -ENODEV;
	}

	clk_prepare_enable(mft_clk);    

    for (i = 0; i < NPCM750_MFT_MAX_MODULE; i++)
    {
        /** stop MFT0~7 clock */
        iowrite8((u8) NPCM750_MFT_NO_CLOCK_MODE, (void *)MFT_REG_TCKC(i));

        /** disable all interrupt */
        iowrite8((u8) 0x00, (void *)MFT_REG_TIEN(i));

        /** clear all interrupt */
        iowrite8((u8) NPCM750_TICLR_CLEAR_ALL, (void *)MFT_REG_TICLR(i));

        /** set MFT0~7 clock prescaler */
        iowrite8((u8) NPCM750_MFT_CLKPS, (void *)MFT_REG_TPRSC(i));

        /** set MFT0~7 mode (high-to-low transition) */
        iowrite8(
                    (u8) (
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
        iowrite8( (u8) (NPCM750_TCPCFG_EQAEN | NPCM750_TCPCFG_EQBEN),
                  (void *)MFT_REG_TCPCFG(i));

        /** set MFT0~7 compare value */
        iowrite16(NPCM750_MFT_TCPA, (void *)MFT_REG_TCPA(i));
        iowrite16(NPCM750_MFT_TCPB, (void *)MFT_REG_TCPB(i));

        /** set MFT0~7 fan input FANIN 0~15 */
        iowrite8((u8) NPCM750_TINASEL_FANIN_DEFAULT, (void *)MFT_REG_TINASEL(i));
        iowrite8((u8) NPCM750_TINASEL_FANIN_DEFAULT, (void *)MFT_REG_TINBSEL(i));

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

    for (i = 0; i < 8; i++) {
        mft_irq[i] = platform_get_irq(pdev, i);
        if (!mft_irq[i]) {
            printk(KERN_ERR "%s - failed to map irq %d\n", __FUNCTION__, i);
            return (-EAGAIN);
        }
    }

    if (request_irq(mft_irq[0], (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT0", (void *) &u8dummy))
    {
        pr_err("NPCM750: register irq MFT0 failed\n");
        return (-EAGAIN);
    }

    if (request_irq(mft_irq[1], (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT1", (void *) &u8dummy))
    {
        pr_err("NPCM750: register irq MFT1 failed\n");
        free_irq(mft_irq[0], (void *) &u8dummy);
        return (-EAGAIN);
    }

    if (request_irq(mft_irq[2], (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT2", (void *) &u8dummy))
    {
        pr_err("NPCM750: register irq MFT2 failed\n");
        free_irq(mft_irq[0], (void *) &u8dummy);
        free_irq(mft_irq[1], (void *) &u8dummy);
        return (-EAGAIN);
    }

    if (request_irq(mft_irq[3], (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT3", (void *) &u8dummy))
    {
        pr_err("NPCM750: register irq MFT3 failed\n");
        free_irq(mft_irq[0], (void *) &u8dummy);
        free_irq(mft_irq[1], (void *) &u8dummy);
        free_irq(mft_irq[2], (void *) &u8dummy);
        return (-EAGAIN);
    }

    if (request_irq(mft_irq[4], (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT4", (void *) &u8dummy))
    {
        pr_err("NPCM750: register irq MFT4 failed\n");
        free_irq(mft_irq[0], (void *) &u8dummy);
        free_irq(mft_irq[1], (void *) &u8dummy);
        free_irq(mft_irq[2], (void *) &u8dummy);
        free_irq(mft_irq[3], (void *) &u8dummy);
        return (-EAGAIN);
    }

    if (request_irq(mft_irq[5], (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT5", (void *) &u8dummy))
    {
        pr_err("NPCM750: register irq MFT5 failed\n");
        free_irq(mft_irq[0], (void *) &u8dummy);
        free_irq(mft_irq[1], (void *) &u8dummy);
        free_irq(mft_irq[2], (void *) &u8dummy);
        free_irq(mft_irq[3], (void *) &u8dummy);
        free_irq(mft_irq[4], (void *) &u8dummy);
        return (-EAGAIN);
    }

    if (request_irq(mft_irq[6], (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT6", (void *) &u8dummy))
    {
        pr_err("NPCM750: register irq MFT6 failed\n");
        free_irq(mft_irq[0], (void *) &u8dummy);
        free_irq(mft_irq[1], (void *) &u8dummy);
        free_irq(mft_irq[2], (void *) &u8dummy);
        free_irq(mft_irq[3], (void *) &u8dummy);
        free_irq(mft_irq[4], (void *) &u8dummy);
        free_irq(mft_irq[5], (void *) &u8dummy);
        return (-EAGAIN);
    }

    if (request_irq(mft_irq[7], (irq_handler_t) npcm750_mft0_isr, 0,
                "NPCM750-MFT7", (void *) &u8dummy))
    {
        pr_err("NPCM750: register irq MFT7 failed\n");
        free_irq(mft_irq[0], (void *) &u8dummy);
        free_irq(mft_irq[1], (void *) &u8dummy);
        free_irq(mft_irq[2], (void *) &u8dummy);
        free_irq(mft_irq[3], (void *) &u8dummy);
        free_irq(mft_irq[4], (void *) &u8dummy);
        free_irq(mft_irq[5], (void *) &u8dummy);
        free_irq(mft_irq[6], (void *) &u8dummy);
        return (-EAGAIN);
    }

    /** initialize fan tach polling timer */
    npcm750_fantach_timer.data = 0;
    npcm750_fantach_timer.function = &npcm750_fantach_polling;

    /** set timer interval */
    npcm750_fantach_timer.expires = jiffies + FAN_TACH_POLLING_INTERVAL;

    init_timer(&npcm750_fantach_timer);
    add_timer(&npcm750_fantach_timer);

    npcm750_fan_get_input_clock();

	priv->groups[0] = &fan_dev_group;
	priv->groups[1] = NULL;

	hwmon = devm_hwmon_device_register_with_groups(dev,"npcm750_fan",priv, priv->groups);
    if (IS_ERR(hwmon))
    {
        pr_err("FAN Driver failed - devm_hwmon_device_register_with_groups failed \n");
        return PTR_ERR(hwmon);
    }

    pr_info("NPCM750 FAN Driver probed\n");    

    for (i = 0; i < NPCM750_MAX_FAN_TACH; i++)
    {
        S_npcm750_fantach[i].u8FanStatusFlag = FAN_TACH_INIT;
    }

	return 0;
}

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:npcm750-fan");

static const struct of_device_id of_fan_match_table[] = {
	{ .compatible = "nuvoton,npcm750-fan", },
	{},
};
MODULE_DEVICE_TABLE(of, of_fan_match_table);

static struct platform_driver npcm750_fan_driver = {
	.probe		= npcm750_fan_probe,
	.driver		= {
		.name	= "npcm750_fan",
		.of_match_table = of_fan_match_table,
	},
};

module_platform_driver(npcm750_fan_driver);

MODULE_DESCRIPTION("Nuvoton NPCM750 FAN Driver");
MODULE_AUTHOR("Tomer Maimon <tomer.maimon@nuvoton.com>");
MODULE_LICENSE("GPL v2");
