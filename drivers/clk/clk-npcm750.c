/*
 * drivers/clk/clk-npcm750.c
 *
 * Provides clock implementations for clk drivers. 
 * all the clocks are set on boot, so this driver allow only reading of current clk settings directly from the module.
 *
 * Copyright (C) 2016 Nuvoton Technologies tali.perry@nuvoton.com
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk/nuvoton.h>



#include <dt-bindings/clock/nuvoton,npcmx50-clks.h>
#include "clk-npcm750.h"


#include <asm/io.h>

#include <mach/hal.h>
#include <mach/regs_npcm750_clk.h>
#include <defs.h>



// #define CLK_DEBUG_MODULE 1
#ifdef CLK_DEBUG_MODULE
    #define CLOCK_DEBUG(fmt,args...)   printk(fmt ,##args)
#else
    #define CLOCK_DEBUG(fmt,args...)
#endif

#undef  CLK_VIRT_BASE_ADDR
#define CLK_VIRT_BASE_ADDR                  clk_base

extern void npcmx50_spin_lock_irqsave(unsigned long *flags);
extern void npcmx50_spin_unlock_irqrestore(unsigned long flags);
extern void npcmx50_atomic_io_modify(void __iomem *reg, u32 mask, u32 set);



/*---------------------------------------------------------------------------------------------------------*/
/* Spec predefined values                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* PLLCON 0 possible values:                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define npcm750_clk_333MHZ_PLLCON0_REG_CFG    0x00A02403
#define npcm750_clk_500MHZ_PLLCON0_REG_CFG    0x00282201
#define npcm750_clk_600MHZ_PLLCON0_REG_CFG    0x00302201
#define npcm750_clk_666MHZ_PLLCON0_REG_CFG    0x00A02203
#define npcm750_clk_700MHZ_PLLCON0_REG_CFG    0x001C2101
#define npcm750_clk_720MHZ_PLLCON0_REG_CFG    0x00902105
#define npcm750_clk_750MHZ_PLLCON0_REG_CFG    0x001E2101
#define npcm750_clk_800MHZ_PLLCON0_REG_CFG    0x00202101  /* NPCM750_PLL1 setting for 800 MHz in Z2 and later will have to be 0040_2101h (instead of 0040_2201h for Z1). */
#define npcm750_clk_825MHZ_PLLCON0_REG_CFG    0x00212101
#define npcm750_clk_850MHZ_PLLCON0_REG_CFG    0x00222101
#define npcm750_clk_888MHZ_PLLCON0_REG_CFG    0x03782119
#define npcm750_clk_900MHZ_PLLCON0_REG_CFG    0x00242101
#define npcm750_clk_950MHZ_PLLCON0_REG_CFG    0x00262101
#define npcm750_clk_1000MHZ_PLLCON0_REG_CFG   0x00282101
#define npcm750_clk_1066MHZ_PLLCON0_REG_CFG   0x00802103

/*---------------------------------------------------------------------------------------------------------*/
/* PLLCON 1 possible values (notice that NPCM750_PLL1 in Z2 has a divider /2, so OTDV1 is smaller in half          */
/*---------------------------------------------------------------------------------------------------------*/
#define npcm750_clk_333MHZ_PLLCON1_REG_CFG    0x00A02203


#define npcm750_clk_444MHZ_PLLCON1_REG_CFG       0x00A02303
#define npcm750_clk_500MHZ_PLLCON1_REG_CFG       0x00282101
#define npcm750_clk_600MHZ_PLLCON1_REG_CFG       0x00302101
#define npcm750_clk_666MHZ_PLLCON1_REG_CFG_Z1    0x00A02203 /* for Z1 */
#define npcm750_clk_666MHZ_PLLCON1_REG_CFG       0x00A02103 /* for Z2: change OTDV1 */

#define npcm750_clk_700MHZ_PLLCON1_REG_CFG    0x00382101
#define npcm750_clk_720MHZ_PLLCON1_REG_CFG    0x01202105
#define npcm750_clk_750MHZ_PLLCON1_REG_CFG    0x003C2101 /* change FBDV */
#define npcm750_clk_800MHZ_PLLCON1_REG_CFG    0x00402101 /* change OTDV1 : NPCM750_PLL1 setting for 800 MHz in Z2 and later will have to be 0040_2101h (instead of 0040_2201h for Z1). */
#define npcm750_clk_825MHZ_PLLCON1_REG_CFG    0x00422101 /* change FBDV */
#define npcm750_clk_850MHZ_PLLCON1_REG_CFG    0x00442101 /* change FBDV */
#define npcm750_clk_900MHZ_PLLCON1_REG_CFG    0x00482101 /* change FBDV */
#define npcm750_clk_950MHZ_PLLCON1_REG_CFG    0x004C2101 /* change FBDV */
#define npcm750_clk_1000MHZ_PLLCON1_REG_CFG   0x00502101 /* change FBDV */
#define npcm750_clk_1066MHZ_PLLCON1_REG_CFG   0x01002103

#define npcm750_clk_800MHZ_PLLCON0_REG_CFG_BB  0x00402201
/*---------------------------------------------------------------------------------------------------------*/
/* PLLCON 2 possible values                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define npcm750_clk_960MHZ_PLLCON2_REG_CFG    0x00C02105


#define LOK_TIMEOUT  100000  /* 4ms if 25 MHz */


/*---------------------------------------------------------------------------------------------------------*/
/* Local definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
/* SD Clock Target frequency */
#define SU60_DESIRED_FREQUENCY      60  // MHz (dont use _1MHz_)
#define SU_DESIRED_FREQUENCY        30  // MHz (dont use _1MHz_)


static void __iomem *clk_base;
static const struct of_device_id npcm750clk_match_table[];
struct clk_core;




/*---------------------------------------------------------------------------------------------------------*/
/* Local Functions                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static  void npcm750_clk_EnableGMACClock(u32 ethNum, bool bEN);
static  u32 npcm750_clk_EnableSDClock(u32 sdNum, bool bEN);
static u32 npcm750_clk_CalculatePLLFrequency(u32 pllVal);
static u32 npcm750_clk_GetPll0Freq (void);
static u32 npcm750_clk_GetPll1Freq(void);
static u32 npcm750_clk_GetPll2Freq(void);
static u32 npcm750_clk_GetMemoryFreq (void);
static u32 npcm750_clk_GetCPUFreq (void);
static u32 npcm750_clk_GetSPIFreq (u32 spi);
static u32 npcm750_clk_GetAPBFreq (NPCM750_APB_CLK apb);
static u32 npcm750_clk_Get_AHB_Freq (void);
// int npcm750_clk_Set_AHB_Freq (u32  cpFreq);
static u32 npcm750_clk_Get_AXI_Freq (void);
// int npcm750_clk_Set_AXI_Freq (u32  cpFreq);
static u32 npcm750_clk_GetSDFreq (u32 sdNum);
static u32 npcm750_clk_GetUartFreq(void);
static u32 npcm750_clk_GetPLL0toAPBdivisor(NPCM750_APB_CLK apb);
static u32 npcm750_clk_Delay_Cycles(u32 cycles);
// void npcm750_clk_ConfigurePCIClock(void);
// int npcm750_clk_ConfigureFIUClock(u8  fiu, u8 clkDiv);
// u8 npcm750_clk_GetFIUClockDiv(u8  fiu);
static int  npcm750_clk_GetUSBClock(void);
static u32  npcm750_clk_GetTimerFreq (void);
static unsigned long npcm750_clk_recalc(struct clk_hw *hw, unsigned long parent_rate);
// int npcm750_clk_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate);
static int	npcm750_clk_enable_disable(struct clk_hw *hw, bool bEN);
static int	npcm750_clk_enable(struct clk_hw *hw);
static void	npcm750_clk_disable(struct clk_hw *hw);
static const struct clk_ops npcm750_clk_ops;
static struct npcm750_clk *npcm750clk_clocks[];
static void npcm750clk_setup(struct device_node *np);


/*---------------------------------------------------------------------------------------------------------*/
/* Functions Implementation                                                                                */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_ConfigureUartClock                                                         */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine configures the Uart clock source to be closest to 24MHz by                */
/*                  modifying the UART divider.                                                            */
/*                  In _PALLADIUM_ bypass mode the UART input frequency is set to be highest possible -    */
/*                  same as APB frequency                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
u32 npcm750_clk_ConfigureUartClock(void)
{
    u32 uart_clk; //Hz
	unsigned long flags = 0;

    /*-------------------------------------------------------------------------------------------------*/
    /* Set UART to 24MHz. Source is NPCM750_PLL2 (960Mhz), which is divided by 2, so we get 24Mhz = 960/2/20   */
    /*-------------------------------------------------------------------------------------------------*/
    u32 uartDesiredFreq  = 24*_1MHz_; //Hz

    /*-----------------------------------------------------------------------------------------------------*/
    /* Normal configuration - UART from PLL0 with divider calculated from PLL0 configuration to get 24MHz  */
    /*-----------------------------------------------------------------------------------------------------*/

    /*-------------------------------------------------------------------------------------------------*/
    /* Calculate the divider given NPCM750_PLL2 output and desired frequency:                          */
    /*-------------------------------------------------------------------------------------------------*/
    u32 pllFreq = npcm750_clk_GetPll2Freq();
    u32 uartDiv = pllFreq/uartDesiredFreq;
    uart_clk = pllFreq / uartDiv;

    npcmx50_spin_lock_irqsave(&flags);

    /*-------------------------------------------------------------------------------------------------*/
    /* Set divider:                                                                                    */
    /*-------------------------------------------------------------------------------------------------*/
    SET_REG_FIELD(CLKDIV1, CLKDIV1_UARTDIV, CLKDIV1_UART_DIV(uartDiv));

    /*-------------------------------------------------------------------------------------------------*/
    /* Choose PLL0 as a source:                                                                        */
    /*-------------------------------------------------------------------------------------------------*/
    SET_REG_FIELD(CLKSEL, CLKSEL_UARTCKSEL, CLKSEL_UARTCKSEL_PLL2);

    npcmx50_spin_unlock_irqrestore(flags);
 
    /*-----------------------------------------------------------------------------------------------------*/
    /* Wait for 200 clock cycles between clkDiv change and clkSel change:                                  */
    /*-----------------------------------------------------------------------------------------------------*/
    npcm750_clk_Delay_Cycles(200);

    CLOCK_DEBUG("\t\tuartfreq is %d\n", uart_clk);

    return uart_clk;

}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_GetPLL0toAPBdivisor                                                        */
/*                                                                                                         */
/* Parameters:      apb: number of APB                                                                     */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine returns the value achieved by dividing PLL0 frequency to APB frequency    */
/*---------------------------------------------------------------------------------------------------------*/
static  u32  npcm750_clk_GetPLL0toAPBdivisor (NPCM750_APB_CLK apb)
{
    volatile u32 apb_divisor = 1;

    apb_divisor = apb_divisor * (READ_REG_FIELD(CLKDIV1, CLKDIV1_CLK2DIV) + 1);       // AXI divider ( div by 1\2)
    apb_divisor = apb_divisor * (READ_REG_FIELD(CLKDIV1, CLKDIV1_CLK4DIV) + 1);       // AHBn divider (div by 1\2\3\4)

    switch (apb)
    {
        case NPCM750_APB1:
            apb_divisor = apb_divisor * (1 << READ_REG_FIELD(CLKDIV2, CLKDIV2_APB1CKDIV));     // APB divider
            break;
        case NPCM750_APB2:
            apb_divisor = apb_divisor * (1 << READ_REG_FIELD(CLKDIV2, CLKDIV2_APB2CKDIV));     // APB divider
            break;
        case NPCM750_APB3:
            apb_divisor = apb_divisor * (1 << READ_REG_FIELD(CLKDIV2, CLKDIV2_APB3CKDIV));     // APB divider
            break;
        case NPCM750_APB4:
            apb_divisor = apb_divisor * (1 << READ_REG_FIELD(CLKDIV2, CLKDIV2_APB4CKDIV));     // APB divider
            break;
        case NPCM750_APB5:
            apb_divisor = apb_divisor * (1 << READ_REG_FIELD(CLKDIV2, CLKDIV2_APB5CKDIV));     // APB divider
            break;
        case NPCM750_SPI0:
            apb_divisor = apb_divisor * (1 + READ_REG_FIELD(CLKDIV3, CLKDIV3_SPI0CKDV));       // NPCM750_SPI0 divider
            break;
        case NPCM750_SPI3:
            apb_divisor = apb_divisor * (1 + READ_REG_FIELD(CLKDIV1, CLKDIV1_AHB3CKDIV));       // NPCM750_SPI3 divider
            break;
        case NPCM750_SPIX:
            apb_divisor = apb_divisor * (1 + READ_REG_FIELD(CLKDIV1, CLKDIV3_SPIXCKDV));       // NPCM750_SPIX divider
            break;    
        default:
            apb_divisor = 0xFFFFFFFF;
            break;
    }

    return apb_divisor;
}






/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_Delay_Cycles                                                               */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  cycles -  num of cycles to delay                                                       */
/*                                                                                                         */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs delay in number of cycles (delay in C code).                     */
/*                  For a more accurate delay, use : npcm750_clk_Delay_MicroSec                            */
/*---------------------------------------------------------------------------------------------------------*/
static u32 npcm750_clk_Delay_Cycles (u32 cycles)
{
    UINT            cacheState = 0;
    volatile UINT   i          = 0;
    volatile u32 iterations = 0;


    /*-----------------------------------------------------------------------------------------------------*/
    /* The measurements were done on PD over 50 cycles, fetches from ROM:                                  */
    /*-----------------------------------------------------------------------------------------------------*/
    const UINT CYCLES_IN_ONE_ITERATION_CACHE_DISABLED  =   145;
	// const UINT CYCLES_IN_ONE_ITERATION_CACHE_ENABLED      6

    /*-----------------------------------------------------------------------------------------------------*/
    /* Calculate number of iterations                                                                      */
    /*-----------------------------------------------------------------------------------------------------*/
    iterations = cycles/CYCLES_IN_ONE_ITERATION_CACHE_DISABLED + 1;

    /*-----------------------------------------------------------------------------------------------------*/
    /* The actual wait loop:                                                                               */
    /*-----------------------------------------------------------------------------------------------------*/
    ICACHE_SAVE_DISABLE(cacheState);
    for (i = 0; i < iterations; i++);
    ICACHE_RESTORE(cacheState);

    return iterations;
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_GetTimeStamp                                                               */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:         Current time stamp                                                                     */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void  npcm750_clk_GetTimeStamp (u32 time_quad[2])
{
    u32 Seconds;
    u32 RefClocks;

    do
    {
        Seconds = REG_READ(SECCNT);
        RefClocks = REG_READ(CNTR25M);
    } while (REG_READ(SECCNT) != Seconds);

    time_quad[0] = RefClocks;
    time_quad[1] = Seconds;
}
EXPORT_SYMBOL(npcm750_clk_GetTimeStamp);

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_Delay_MicroSec                                                                     */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  microSec -  number of microseconds to delay                                            */
/*                                                                                                         */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs a busy delay (empty loop)                                        */
/*                  the number of iterations is clk_based on current CPU clock calculation and cache           */
/*                  enabled/disabled                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
static u32 npcm750_clk_Delay_MicroSec(u32 microSec)
{

/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* This register is reset by only VSB power-up reset. The value of this register                           */
/* represents a counter with a 25 MHz clock, used to update the SECCNT register. This field is updated every*/
/* 640ns. The 4 LSB of this field are always 0. When this field reaches a value of 25,000,000 it goes to 0 */
/* and SEC_CNT field is updated.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
    /* not testing that microSec < 33 sec (2^25bit) us */

    u32 iUsCnt1[2], iUsCnt2[2];
    u32 delay;  // Acctual delay generated by FW
    u32 minimum_delay = (microSec * NPCMX50_EXT_CLOCK_FREQUENCY_MHZ) + CNTR25M_ACCURECY; /* this is equivalent to microSec/0.64 + minimal tic length.*/

    npcm750_clk_GetTimeStamp(iUsCnt1);

    do
    {
        npcm750_clk_GetTimeStamp(iUsCnt2);
        delay =  ((NPCMX50_EXT_CLOCK_FREQUENCY_MHZ * _1MHz_) * (iUsCnt2[1] - iUsCnt1[1])) + (iUsCnt2[0] - iUsCnt1[0]);
    }
    while(delay < minimum_delay);


    return (u32)(delay / NPCMX50_EXT_CLOCK_FREQUENCY_MHZ);

}



/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_Delay_Since                                                                */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  microSecDelay -  number of microseconds to delay since t0_time. if zero: no delay.     */
/*                  t0_time       - start time , to measure time from.                                     */
/*                                                                                                         */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  get a time stamp, delay microSecDelay from it. If microSecDelay has already passed     */
/*                  since the time stamp , then no delay is executed. returns the time that elapsed since  */
/*                  t0_time .                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
u32 npcm750_clk_Delay_Since (u32 microSecDelay, u32 t0_time[2])
{

    u32 iUsCnt2[2];
    u32 timeElapsedSince;  // Acctual delay generated by FW
    u32 minimum_delay = (microSecDelay * NPCMX50_EXT_CLOCK_FREQUENCY_MHZ) + CNTR25M_ACCURECY; /* this is equivalent to microSec/0.64 + minimal tic length.*/


    do
    {
        npcm750_clk_GetTimeStamp(iUsCnt2);
        timeElapsedSince =  ((NPCMX50_EXT_CLOCK_FREQUENCY_MHZ * _1MHz_) * (iUsCnt2[1] - t0_time[1])) + (iUsCnt2[0] - t0_time[0]);
    }
    while(timeElapsedSince < minimum_delay);

    /*-----------------------------------------------------------------------------------------------------*/
    /* return elapsed time                                                                                 */
    /*-----------------------------------------------------------------------------------------------------*/
    return (u32)(timeElapsedSince / NPCMX50_EXT_CLOCK_FREQUENCY_MHZ);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_EnableEMCClock                                                             */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  ethNum -  ethernet module number                                                       */
/*                                                                                                         */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine configures EMC clocks                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static  void npcm750_clk_EnableEMCClock(u32 ethNum, bool bEN)
{
    unsigned long flags = 0;
    npcmx50_spin_lock_irqsave(&flags);

    if (ethNum == 0)                          // ETH0 - EMC1
    {
       SET_REG_FIELD(CLKEN1, CLKEN1_EMC1, bEN);
    }
    else if (ethNum == 1)                     // ETH1 - EMC2
    {
       SET_REG_FIELD(CLKEN1, CLKEN1_EMC2, bEN);
    }
    npcmx50_spin_unlock_irqrestore(flags);
}



/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_EnableGMACClock                                                            */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  ethNum -  ethernet module number                                                       */
/*                                                                                                         */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine configures GMAC clocks                                                    */
/*---------------------------------------------------------------------------------------------------------*/
static  void npcm750_clk_EnableGMACClock(u32 ethNum, bool bEN)
{
    unsigned long flags = 0;
    npcmx50_spin_lock_irqsave(&flags);
    if (ethNum == 2)                          // ETH2 - GMAC1
    {
        SET_REG_FIELD(CLKEN2, CLKEN2_GMAC1, bEN);
    }
    else if (ethNum == 3)                     // ETH3 - GMAC2
    {
        SET_REG_FIELD(CLKEN2, CLKEN2_GMAC2, bEN);
    }
     npcmx50_spin_unlock_irqrestore(flags);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_EnableSDClock                                                              */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  sdNum -  SD module number                                                              */
/*                                                                                                         */
/* Returns:         SD clock frequency                                                                     */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  Enable the SD\MMC clock                                                                */
/*---------------------------------------------------------------------------------------------------------*/
static  u32 npcm750_clk_EnableSDClock(u32 sdNum, bool bEN)
{
	unsigned long flags = 0;
	
    /*-----------------------------------------------------------------------------------------------------*/
    /* Parameters check                                                                                    */
    /*-----------------------------------------------------------------------------------------------------*/
    if (sdNum >= NPCMX50_SD_NUM_OF_MODULES)
    {
        return -EINVAL;
    }
    npcmx50_spin_lock_irqsave(&flags);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Configure Clock Enable Register                                                                     */
    /*-----------------------------------------------------------------------------------------------------*/
    if (sdNum == NPCM750_CLK_SD)
    {
        SET_REG_FIELD(CLKEN2, CLKEN2_SDHC, bEN);
    }
    else if (sdNum == NPCM750_CLK_MMC)
    {
        SET_REG_FIELD(CLKEN2, CLKEN2_MMC, bEN);
    }
    npcmx50_spin_unlock_irqrestore(flags);
    return 0;

}

#ifdef _RESET_DEVICE_TREE_SUPPORT_TODO_ 

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_ResetEth                                                                   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  deviceNum -                                                                            */
/*                                                                                                         */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs SW reset of EMC module                                           */
/*---------------------------------------------------------------------------------------------------------*/
static void npcm750_clk_ResetEMC(u32 deviceNum)
{
    unsigned long flags = 0;
    npcmx50_spin_lock_irqsave(&flags);
    if (deviceNum == 0)
    {
        SET_REG_FIELD(IPSRST1, IPSRST1_EMC1, 1);
        SET_REG_FIELD(IPSRST1, IPSRST1_EMC1, 0);
    }
    else if (deviceNum == 1)
    {
        SET_REG_FIELD(IPSRST1, IPSRST1_EMC2, 1);
        SET_REG_FIELD(IPSRST1, IPSRST1_EMC2, 0);
    }
    npcmx50_spin_unlock_irqrestore(flags);
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_ResetFIU                                                                   */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs SW reset of FIU                                                  */
/*---------------------------------------------------------------------------------------------------------*/
static void npcm750_clk_ResetFIU (void)
{
    unsigned long flags = 0;
    npcmx50_spin_lock_irqsave(&flags);
    SET_REG_FIELD(IPSRST1, IPSRST1_SPI0, 1);
    SET_REG_FIELD(IPSRST1, IPSRST1_SPI0, 0);

    SET_REG_FIELD(IPSRST1, IPSRST1_SPI3, 1);
    SET_REG_FIELD(IPSRST1, IPSRST1_SPI3, 0);

    SET_REG_FIELD(IPSRST3, IPSRST3_SPIX, 1);
    SET_REG_FIELD(IPSRST3, IPSRST3_SPIX, 0);
    npcmx50_spin_unlock_irqrestore(flags);
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_ResetUART                                                                  */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  deviceNum -                                                                            */
/*                                                                                                         */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs SW reset of all UARTs. 0 and 1 have shared reset. 2 and 3 too.   */
/*---------------------------------------------------------------------------------------------------------*/
static void npcm750_clk_ResetUART(u32 deviceNum)
{
    unsigned long flags = 0;
    npcmx50_spin_lock_irqsave(&flags);
    SET_REG_FIELD(IPSRST1, IPSRST1_UART01, 1);
    SET_REG_FIELD(IPSRST1, IPSRST1_UART01, 0);

    SET_REG_FIELD(IPSRST1, IPSRST1_UART23, 1);
    SET_REG_FIELD(IPSRST1, IPSRST1_UART23, 0);
    npcmx50_spin_unlock_irqrestore(flags);
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_ResetAES                                                                   */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs SW reset of AES                                                  */
/*---------------------------------------------------------------------------------------------------------*/
static void npcm750_clk_ResetAES (void)
{
    unsigned long flags = 0;
    npcmx50_spin_lock_irqsave(&flags);
    SET_REG_FIELD(IPSRST1, IPSRST1_AES, 1);
    SET_REG_FIELD(IPSRST1, IPSRST1_AES, 0);
    npcmx50_spin_unlock_irqrestore(flags);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_ResetMC                                                                    */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs SW reset of MC                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static void npcm750_clk_ResetMC (void)
{
    unsigned long flags = 0;
    npcmx50_spin_lock_irqsave(&flags);
    SET_REG_FIELD(IPSRST1, IPSRST1_MC, 1);
    npcm750_clk_Delay_MicroSec(100);
    SET_REG_FIELD(IPSRST1, IPSRST1_MC, 0);
    npcm750_clk_Delay_MicroSec(100);

    
    /* Force re-training of DDR (because DDR module is reinitialized*/
    SET_REG_FIELD(INTCR2, INTCR2_MC_INIT, 0);
    npcmx50_spin_unlock_irqrestore(flags);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_ResetTIMER                                                                 */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  deviceNum -                                                                            */
/*                                                                                                         */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs SW reset of Timer                                                */
/*---------------------------------------------------------------------------------------------------------*/
static void npcm750_clk_ResetTIMER (u32 deviceNum)
{
    unsigned long flags = 0;
    npcmx50_spin_lock_irqsave(&flags);
    
    if (deviceNum <= 4)
    {
        SET_REG_FIELD(IPSRST1, IPSRST1_TIM0_4, 1);
        SET_REG_FIELD(IPSRST1, IPSRST1_TIM0_4, 0);
    }
    else if (deviceNum <= 9)
    {
        SET_REG_FIELD(IPSRST1, IPSRST1_TIM5_9, 1);
        SET_REG_FIELD(IPSRST1, IPSRST1_TIM5_9, 0);
    }
    else
    {
        SET_REG_FIELD(IPSRST3, IPSRST3_TIMER10_14, 1);
        SET_REG_FIELD(IPSRST3, IPSRST3_TIMER10_14, 0);
    }
    npcmx50_spin_unlock_irqrestore(flags);

}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_ResetSD                                                                    */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  sdNum -  SD module number                                                              */
/*                                                                                                         */
/* Returns:         BMC HAL Error code                                                                     */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs SW reset of SD                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static int npcm750_clk_ResetSD(u32 sdNum)
{
    unsigned long flags = 0;
    /*-----------------------------------------------------------------------------------------------------*/
    /* Parameters check                                                                                    */
    /*-----------------------------------------------------------------------------------------------------*/
    if (sdNum >= NPCMX50_SD_NUM_OF_MODULES)
    {
        return -2;
    }
    npcmx50_spin_lock_irqsave(&flags);

    if (sdNum == NPCM750_CLK_SD)
    {
        SET_REG_FIELD(IPSRST2, IPSRST2_SDHC, 1);
        SET_REG_FIELD(IPSRST2, IPSRST2_SDHC, 0);
    }
    else if (sdNum == NPCM750_CLK_MMC)
    {
        SET_REG_FIELD(IPSRST2, IPSRST2_MMC, 1);
        SET_REG_FIELD(IPSRST2, IPSRST2_MMC, 0);
    }
    npcmx50_spin_unlock_irqrestore(flags);

    return 0;
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_ResetGMAC                                                                   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  deviceNum -                                                                            */
/*                                                                                                         */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs SW reset of GMAC                                                 */
/*---------------------------------------------------------------------------------------------------------*/
static void npcm750_clk_ResetGMAC(u32 deviceNum)
{
    unsigned long flags = 0;
    npcmx50_spin_lock_irqsave(&flags);
    if (deviceNum == 2)
	{
        SET_REG_FIELD(IPSRST2, IPSRST2_GMAC1, 1);
        SET_REG_FIELD(IPSRST2, IPSRST2_GMAC1, 0);
	}
	else if (deviceNum == 3)
	{
        SET_REG_FIELD(IPSRST2, IPSRST2_GMAC2, 1);
        SET_REG_FIELD(IPSRST2, IPSRST2_GMAC2, 0);
	}
	npcmx50_spin_unlock_irqrestore(flags);

}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_ResetPSPI                                                                          */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  deviceNum -                                                                            */
/*                                                                                                         */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs SW reset of PSPI                                                 */
/*---------------------------------------------------------------------------------------------------------*/
static int npcm750_clk_ResetPSPI(u32 deviceNum)
{
    unsigned long flags = 0;
    /*-----------------------------------------------------------------------------------------------------*/
    /* Parameters check                                                                                    */
    /*-----------------------------------------------------------------------------------------------------*/
    if (deviceNum >= PSPI_NUM_OF_MODULES)
    {
        return -2;
    }
    npcmx50_spin_lock_irqsave(&flags);

    if (deviceNum == PSPI1_DEV)
    {
        SET_REG_FIELD(IPSRST2, IPSRST2_PSPI1, 1);
        SET_REG_FIELD(IPSRST2, IPSRST2_PSPI1, 0);
    }
    else if (deviceNum == PSPI2_DEV)
    {
        SET_REG_FIELD(IPSRST2, IPSRST2_PSPI2, 1);
        SET_REG_FIELD(IPSRST2, IPSRST2_PSPI2, 0);
    }
    npcmx50_spin_unlock_irqrestore(flags);

    return 0;
}
#endif // _RESET_DEVICE_TREE_SUPPORT_TODO_ 

//Calculates the PLL frequency in Hz given PLL register value
/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_CalculatePLLFrequency                                                      */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  pllVal    -                                                                            */
/*                                                                                                         */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine returns the PLL frequency in Hz                                           */
/*---------------------------------------------------------------------------------------------------------*/
static u32  npcm750_clk_CalculatePLLFrequency (u32 pllVal)
{
    u32  FIN         = NPCMX50_EXT_CLOCK_FREQUENCY_KHZ; // 25MHz in KHz units
    u32  FOUT        = 0;
    u32  NR          = 0;
    u32  NF          = 0;
    u32  NO          = 0;

    /*-----------------------------------------------------------------------------------------------------*/
    /* Extract PLL fields:                                                                                 */
    /*-----------------------------------------------------------------------------------------------------*/
    NR = READ_VAR_FIELD(pllVal, PLLCONn_INDV);   /* PLL Input Clock Divider */
    NF = READ_VAR_FIELD(pllVal, PLLCONn_FBDV);   /* PLL VCO Output Clock Feedback Divider). */
    NO = (READ_VAR_FIELD(pllVal, PLLCONn_OTDV1)) * (READ_VAR_FIELD(pllVal, PLLCONn_OTDV2));   /* PLL Output Clock Divider 1 */

    /*-----------------------------------------------------------------------------------------------------*/
    /* Calculate PLL frequency in Hz:                                                                     */
    /*-----------------------------------------------------------------------------------------------------*/
    FOUT = ((10*FIN*NF)/(NO*NR));

    /*-----------------------------------------------------------------------------------------------------*/
    /* Notice: for better accurecy we multiply the "MONE" in 10, and later in 100 to get to Hz units.      */
    /*-----------------------------------------------------------------------------------------------------*/

    /*-----------------------------------------------------------------------------------------------------*/
    /* Returning value in Hertz:                                                                           */
    /*-----------------------------------------------------------------------------------------------------*/
    return  FOUT*100 ;

}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_GetPll0Freq                                                                */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  Returns the frequency of PLL0 in Hz                                                    */
/*---------------------------------------------------------------------------------------------------------*/
static u32  npcm750_clk_GetPll0Freq (void)
{
    u32  pllVal      = 0;

    pllVal = REG_READ(PLLCON0);
    return npcm750_clk_CalculatePLLFrequency(pllVal);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_GetPll1Freq                                                                */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  Returns the frequency of NPCM750_PLL1 in Hz                                            */
/*---------------------------------------------------------------------------------------------------------*/
static u32  npcm750_clk_GetPll1Freq(void)
{
    u32  pllVal      = 0;

    pllVal = REG_READ(PLLCON1);
    
    return (npcm750_clk_CalculatePLLFrequency(pllVal)/2);

}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_GetPll2Freq                                                                */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  Returns the frequency of NPCM750_PLL2 in Hz                                            */
/*---------------------------------------------------------------------------------------------------------*/
static u32  npcm750_clk_GetPll2Freq(void)
{
    u32  pllVal      = 0;

    pllVal = REG_READ(PLLCON2);
    return (npcm750_clk_CalculatePLLFrequency(pllVal)/2);
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_GetMemoryFreq                                                              */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine calculates Memory frequency in Hz                                         */
/*---------------------------------------------------------------------------------------------------------*/
static u32  npcm750_clk_GetMemoryFreq (void)
{
    u32  FOUT        = 0;

    /*-----------------------------------------------------------------------------------------------------*/
    /* Calculate CPU clock:                                                                                */
    /*-----------------------------------------------------------------------------------------------------*/
    if (READ_REG_FIELD(CLKSEL, CLKSEL_MCCKSEL) == CLKSEL_MCCKSEL_PLL1)
    {
        FOUT = npcm750_clk_GetPll1Freq();
    }
    else if (READ_REG_FIELD(CLKSEL, CLKSEL_MCCKSEL) == CLKSEL_MCCKSEL_CLKREF)
    {
        /*-------------------------------------------------------------------------------------------------*/
        /* Reference clock 25MHz:                                                                          */
        /*-------------------------------------------------------------------------------------------------*/
        FOUT = NPCMX50_EXT_CLOCK_FREQUENCY_MHZ; //FOUT is specified in MHz
    }
    else
    {
        /*-------------------------------------------------------------------------------------------------*/
        /* External clock, assume low freq ref clock (25MHz):                                              */
        /*-------------------------------------------------------------------------------------------------*/
        FOUT = NPCMX50_EXT_CLOCK_FREQUENCY_MHZ; //FOUT is specified in MHz
    }


    /*-----------------------------------------------------------------------------------------------------*/
    /* Returing value in Hertz                                                                             */
    /*-----------------------------------------------------------------------------------------------------*/
    return FOUT;
}



/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_GetCPUFreq                                                                 */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine calculates CPU frequency in Hz                                            */
/*---------------------------------------------------------------------------------------------------------*/
static u32  npcm750_clk_GetCPUFreq (void)
{
    u32  FOUT        = 0;

    /*-----------------------------------------------------------------------------------------------------*/
    /* Calculate CPU clock:                                                                                */
    /*-----------------------------------------------------------------------------------------------------*/
    if (READ_REG_FIELD(CLKSEL, CLKSEL_CPUCKSEL) == CLKSEL_CPUCKSEL_PLL0)
    {
        FOUT = npcm750_clk_GetPll0Freq();
    }
    else if (READ_REG_FIELD(CLKSEL, CLKSEL_CPUCKSEL) == CLKSEL_CPUCKSEL_PLL1)
    {
        FOUT = npcm750_clk_GetPll1Freq();
    }
    else if (READ_REG_FIELD(CLKSEL, CLKSEL_CPUCKSEL) == CLKSEL_CPUCKSEL_CLKREF)
    {
        /*-------------------------------------------------------------------------------------------------*/
        /* Reference clock 25MHz:                                                                          */
        /*-------------------------------------------------------------------------------------------------*/
        FOUT = NPCMX50_EXT_CLOCK_FREQUENCY_MHZ; //FOUT is specified in MHz
    }
    else
    {
        /*-------------------------------------------------------------------------------------------------*/
        /* External clock, assume low freq ref clock (25MHz):                                              */
        /*-------------------------------------------------------------------------------------------------*/
        FOUT = NPCMX50_EXT_CLOCK_FREQUENCY_MHZ; //FOUT is specified in MHz
    }


    /*-----------------------------------------------------------------------------------------------------*/
    /* Returing value in Hertz                                                                             */
    /*-----------------------------------------------------------------------------------------------------*/
    return FOUT;
}



/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_GetSPIFreq                                                                 */
/*                                                                                                         */
/* Parameters:      apb number,1 to 5                                                                      */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine returns SPI frequency  in Hz                                              */
/*---------------------------------------------------------------------------------------------------------*/
static u32 npcm750_clk_GetSPIFreq (u32 spi)
{
    return  npcm750_clk_GetCPUFreq()  / npcm750_clk_GetPLL0toAPBdivisor(spi);  // u32  npcm750_clk_GetPLL0toAPBdivisor (NPCM750_APB_CLK apb)

}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_GetAPBFreq                                                                 */
/*                                                                                                         */
/* Parameters:      apb number,1 to 5                                                                      */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine returns APB frequency  in Hz                                              */
/*---------------------------------------------------------------------------------------------------------*/
static u32 npcm750_clk_GetAPBFreq (NPCM750_APB_CLK apb)
{
    /*-----------------------------------------------------------------------------------------------------*/
    /* Avalilable APBs between 1 to 5                                                                      */
    /*-----------------------------------------------------------------------------------------------------*/
     if ((apb > NPCM750_APB5) && (apb < NPCM750_APB1))
        return -1;

    /*-----------------------------------------------------------------------------------------------------*/
    /* In Yarkon APB frequency is CPU frequency divided by AHB0 Clock dividor, AHB1 Clock dividor and APB  */
    /* Clock divider                                                                                       */
    /*-----------------------------------------------------------------------------------------------------*/

    return  npcm750_clk_GetCPUFreq()  / npcm750_clk_GetPLL0toAPBdivisor(apb);  // u32  npcm750_clk_GetPLL0toAPBdivisor (NPCM750_APB_CLK apb)
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_Get_AHB_Freq                                                               */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine returns CP frequency in Hz  (this actually returns CLK4:  )               */
/*---------------------------------------------------------------------------------------------------------*/
static u32 npcm750_clk_Get_AHB_Freq (void)
{
    u32  clk2Div = 0;

    u32  clk4Div = 0;

    clk2Div = (READ_REG_FIELD(CLKDIV1, CLKDIV1_CLK2DIV) + 1);

    clk4Div = (READ_REG_FIELD(CLKDIV1, CLKDIV1_CLK4DIV) + 1);


    /*-----------------------------------------------------------------------------------------------------*/
    /* In Poleg APB frequency is CPU frequency divided by AHB0 Clock dividor, AHB1 Clock dividor and APB   */
    /* Clock divider                                                                                       */
    /*-----------------------------------------------------------------------------------------------------*/
    return  (npcm750_clk_GetCPUFreq()  / (clk2Div * clk4Div)) ; 

}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_Set_AHB_Freq                                                               */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine sets CP frequency in Hz                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int npcm750_clk_Set_AHB_Freq (u32  cpFreq)
{
    unsigned long flags = 0;
    u32  clkDiv = npcm750_clk_GetCPUFreq() / cpFreq;

    npcmx50_spin_lock_irqsave(&flags);
    switch (clkDiv)
    {
        case 1:
            SET_REG_FIELD(CLKDIV1, CLKDIV1_CLK4DIV, CLKDIV1_CLK4DIV1);
            SET_REG_FIELD(CLKDIV1, CLKDIV1_CLK2DIV, CLKDIV1_CLK2DIV1);
            break;
        case 2:
            SET_REG_FIELD(CLKDIV1, CLKDIV1_CLK4DIV, CLKDIV1_CLK4DIV2);
            SET_REG_FIELD(CLKDIV1, CLKDIV1_CLK2DIV, CLKDIV1_CLK2DIV1);
            break;
        case 3:
            SET_REG_FIELD(CLKDIV1, CLKDIV1_CLK4DIV, CLKDIV1_CLK4DIV3);
            SET_REG_FIELD(CLKDIV1, CLKDIV1_CLK2DIV, CLKDIV1_CLK2DIV1);
            break;
        case 4:
            SET_REG_FIELD(CLKDIV1, CLKDIV1_CLK4DIV, CLKDIV1_CLK4DIV2);
            SET_REG_FIELD(CLKDIV1, CLKDIV1_CLK2DIV, CLKDIV1_CLK2DIV2);
            break;
        case 6:
            SET_REG_FIELD(CLKDIV1, CLKDIV1_CLK4DIV, CLKDIV1_CLK4DIV3);
            SET_REG_FIELD(CLKDIV1, CLKDIV1_CLK2DIV, CLKDIV1_CLK2DIV2);
            break;
        case 8:
            SET_REG_FIELD(CLKDIV1, CLKDIV1_CLK4DIV, CLKDIV1_CLK4DIV4);
            SET_REG_FIELD(CLKDIV1, CLKDIV1_CLK2DIV, CLKDIV1_CLK2DIV2);
            break;
        default:
		{
		    npcmx50_spin_unlock_irqrestore(flags);
            return -1;
		}
    }

    npcmx50_spin_unlock_irqrestore(flags);

    npcm750_clk_Delay_MicroSec(20);

    return  0;

    
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_Get AHB_Freq                                                               */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine returns AXI frequency (l2cache etc., CLK2)                                */
/*---------------------------------------------------------------------------------------------------------*/
static u32 npcm750_clk_Get_AXI_Freq (void)
{
    u32  clk2Div = (READ_REG_FIELD(CLKDIV1, CLKDIV1_CLK2DIV) + 1);
    
    /*-----------------------------------------------------------------------------------------------------*/
    /* In Poleg APB frequency is CPU frequency divided by AHB0 Clock dividor, AHB1 Clock dividor and APB   */
    /* Clock divider                                                                                       */
    /*-----------------------------------------------------------------------------------------------------*/
    return  (npcm750_clk_GetCPUFreq()  / (clk2Div)) ; 
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_GetSDFreq                                                                  */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  sdNum -  SD module number                                                              */
/*                                                                                                         */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  Returns the SD clk_base clock frequency in Hz                                          */
/*---------------------------------------------------------------------------------------------------------*/
static u32 npcm750_clk_GetSDFreq (u32 sdNum)
{
    u32  divider;
    u32  pll0Freq;   //In Hz

    pll0Freq = npcm750_clk_GetPll0Freq();
    if (sdNum == NPCM750_CLK_SD)
    {
        divider = 1 + READ_REG_FIELD(CLKDIV2, CLKDIV2_SD1CKDIV);
    }
    else if (sdNum == NPCM750_CLK_MMC)
    {
        divider = 1 + READ_REG_FIELD(CLKDIV1, CLKDIV1_MMCCKDIV);
    }
    else
    {
	    return -2;
    }

    return (pll0Freq/divider);
}




/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_GetUartFreq                                                                */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine gets the UART clk                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static u32 npcm750_clk_GetUartFreq(void)
{
    return (npcm750_clk_GetPll2Freq() /  (u32)(READ_REG_FIELD(CLKDIV1, CLKDIV1_UARTDIV) + 1));
}




/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_ConfigurePCIClock                                                          */
/*                                                                                                         */
/* Parameters:      none  .                                                                                 */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs configuration of PCI clock depending on                          */
/*                  presence of VGA BIOS as specified by STRAP13                                           */
/*---------------------------------------------------------------------------------------------------------*/
void npcm750_clk_ConfigurePCIClock(void)
{
    u32 PLLCON1_l = 0;
    unsigned long flags = 0;

    // Need to pgm the PCI clock to 96 MHz.

    // Done in following steps:
    // 1.  Delay 2us
    // 2.  Change CLKSEL1.GFXCKSEL ( bits 17-16)  from 1h to 3h (select NPCM750_PLL2)
    // 3.  Delay 2 us
    // 4.  Change CLKDIV1.PCICKDIV (bits 5-2) from 02h to 04h (divide by 5)
    // 5.  Delay 2us

    PLLCON1_l  = npcm750_clk_333MHZ_PLLCON1_REG_CFG;

    SET_VAR_FIELD(PLLCON1_l, PLLCONn_PWDEN, PLLCONn_PWDEN_NORMAL);

    REG_WRITE(PLLCON1, PLLCON1_l);

    npcm750_clk_Delay_MicroSec(2);
	
	npcmx50_spin_lock_irqsave(&flags);

    SET_REG_FIELD(CLKSEL, CLKSEL_GFXCKSEL,  CLKSEL_GFXCKSEL_PLL2); // changed to workaroung PCI issue

    npcm750_clk_Delay_MicroSec(2);

    SET_REG_FIELD(CLKDIV1, CLKDIV1_PCICKDIV,  CLKDIV1_PCICK_DIV(5));  /* PCI clock div = 5  */

    npcmx50_spin_unlock_irqrestore(flags);

    npcm750_clk_Delay_MicroSec(2);


//    1.  NPCM750_PLL1 change to 33MHz -> PLLCON1 = A02403
//    2.  PCI Clock = 96 MHz ->  CLKSEL.GFXCKSEL selects NPCM750_PLL2/2
//                                              CLKDIV1.PCICKDIV = 4   (480/5 = 96)
//
//
//    3.  DRAM reference (in MR6: bits 5-0 = 0Ah)
//    4.  Poleg VREF = 0Ah   F05F01c8h bits 9-4
//    5.  Reftersh period. MC CTL_51 value should be divide by 2
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_ConfigureFIUClock                                                          */
/*                                                                                                         */
/* Parameters:      u8  fiu, u8 clkDiv                                                                     */
/* Returns:         int                                                                                    */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine config the FIU clock (according to the header )                           */
/*---------------------------------------------------------------------------------------------------------*/
int npcm750_clk_ConfigureFIUClock(u8  fiu, u8 clkDiv)
{

    unsigned long flags = 0;
     /*----------------------------------------------------------------------------------------------------*/
     /* Defines the clock divide ratio from AHB to FIU0 clock.                                             */
     /*----------------------------------------------------------------------------------------------------*/
     u32  ratio = 0;

     /*----------------------------------------------------------------------------------------------------*/
     /* Ignored if FIU_Clk_Divider is either 0 or 0FFh.                                                    */
     /*----------------------------------------------------------------------------------------------------*/
     if ((clkDiv == 0) || (clkDiv == 0xFF))
        return -1;

     npcmx50_spin_lock_irqsave(&flags);   

     /* set SPIn clk div */
     switch (fiu)
     {
        case FIU_MODULE_0:
            SET_REG_FIELD(CLKDIV3, CLKDIV3_SPI0CKDV,  (CLKDIV3_SPI0CKDV_DIV(clkDiv) & 0x1F));
            break;
        case  FIU_MODULE_3:
            SET_REG_FIELD(CLKDIV1, CLKDIV1_AHB3CKDIV, (CLKDIV1_AHB3CK_DIV(clkDiv)   & 0x1F));
            break;
        case  FIU_MODULE_X:
            SET_REG_FIELD(CLKDIV3, CLKDIV3_SPIXCKDV,  (CLKDIV3_SPIXCKDV_DIV(clkDiv) & 0x1F));
            break;
        default:
            npcmx50_spin_unlock_irqrestore(flags);
            return -2;


     }

     npcmx50_spin_unlock_irqrestore(flags);

     /*----------------------------------------------------------------------------------------------------*/
     /* After changing this field, ensure a delay of 25 NPCM750_SPI0 clock cycles before changing CPUCKSEL field in*/
     /* CLKSEL register or accessing the AHB18 bus.                                                        */
     /*----------------------------------------------------------------------------------------------------*/
     ratio = READ_REG_FIELD(CLKDIV1, CLKDIV1_CLK2DIV) * READ_REG_FIELD(CLKDIV1, CLKDIV1_CLK4DIV) * clkDiv;

     /* delay is according to ratio. Take some buffer too */
     npcm750_clk_Delay_Cycles(50 * ratio);

     return 0;
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_GetFIUClockDiv                                                             */
/*                                                                                                         */
/* Parameters:      u8  fiu                                                                                */
/* Returns:         CLKDIV                                                                                 */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine config the FIU clock (according to the header )                           */
/*---------------------------------------------------------------------------------------------------------*/
u8 npcm750_clk_GetFIUClockDiv(u8  fiu)
{

     /*----------------------------------------------------------------------------------------------------*/
     /* Defines the clock divide ratio from AHB to FIU0 clock.1                                            */
     /*----------------------------------------------------------------------------------------------------*/
     switch (fiu)
     {
        case FIU_MODULE_0:
            return READ_REG_FIELD(CLKDIV3, CLKDIV3_SPI0CKDV) + 1;
        case  FIU_MODULE_3:
            return READ_REG_FIELD(CLKDIV1, CLKDIV1_AHB3CKDIV) + 1;
        case  FIU_MODULE_X:
            return READ_REG_FIELD(CLKDIV3, CLKDIV3_SPIXCKDV) + 1;
        default:
            return 0xFF;
     }


}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_GetUSBClock                                                                */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  Get the USB (should be 60MHz)                                                          */
/*---------------------------------------------------------------------------------------------------------*/
static int  npcm750_clk_GetUSBClock(void)
{

    u32  choosenPllFreq;

    if (READ_REG_FIELD(CLKSEL, CLKSEL_SUCKSEL) == CLKSEL_SUCKSEL_PLL0)
    { 
        choosenPllFreq = npcm750_clk_GetPll0Freq();
    }
    else if (READ_REG_FIELD(CLKSEL, CLKSEL_SUCKSEL) == CLKSEL_SUCKSEL_PLL1)
    {
        choosenPllFreq = npcm750_clk_GetPll1Freq();
    }
    else if (READ_REG_FIELD(CLKSEL, CLKSEL_SUCKSEL) == CLKSEL_SUCKSEL_PLL2)
    {
        choosenPllFreq = npcm750_clk_GetPll2Freq();
    }
    else
    {
        return -EINVAL;
    }

    return (choosenPllFreq /  (READ_REG_FIELD(CLKDIV2, CLKDIV2_SU48CKDIV) + 1)) ;

    //SET_REG_FIELD(CLKDIV2, CLKDIV2_SUCKDIV, suDivider - 1);

}



/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_clk_GetTimerFreq                                                               */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine gets timer clock                                                          */
/*---------------------------------------------------------------------------------------------------------*/
static u32  npcm750_clk_GetTimerFreq (void)
{

    u32  choosenPllFreq;

    if (READ_REG_FIELD(CLKSEL, CLKSEL_TIMCKSEL) == CLKSEL_TIMCKSEL_PLL0)
    { 
        choosenPllFreq = npcm750_clk_GetPll0Freq();
    }
    else if (READ_REG_FIELD(CLKSEL, CLKSEL_TIMCKSEL) == CLKSEL_TIMCKSEL_PLL1)
    {
        choosenPllFreq = npcm750_clk_GetPll1Freq();
    }
    else if (READ_REG_FIELD(CLKSEL, CLKSEL_TIMCKSEL) == CLKSEL_TIMCKSEL_PLL2)
    {
        choosenPllFreq = npcm750_clk_GetPll2Freq();
    }
    else if (READ_REG_FIELD(CLKSEL, CLKSEL_TIMCKSEL) == CLKSEL_TIMCKSEL_CLKREF)
    {
        choosenPllFreq = NPCMX50_CLKREF;
    }
    else
    {
        return -EINVAL;
    }

    return (choosenPllFreq /  (READ_REG_FIELD(CLKDIV1, CLKDIV1_TIMCKDIV) + 1)) ;

}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * struct npcm750_clk - Common struct to all npcm750 clocks.
 * @hw: clk_hw for the common clk framework
 * @regmap: Regmap for the clock control registers
 */
struct npcm750_clk {
	struct clk_hw hw;
};
#define to_npcm750_clk(_hw) container_of(_hw, struct npcm750_clk, hw)


/**
 * npcm750_clk_recalc - Calculate the PLL generated clock rate given the
 * parent clock rate.
 */
static unsigned long npcm750_clk_recalc(struct clk_hw *hw, unsigned long parent_rate)
{
	unsigned long ret_val = -ENOKEY;

	/* setting ADC clock divider to 2*/
	SET_REG_FIELD(CLKDIV1, CLKDIV1_ADCCKDIV, 0);

    if(0 == strcmp(__clk_get_name(hw->clk), "clk_pll0"))               ret_val =  npcm750_clk_GetPll0Freq();
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_pll1"))               ret_val =  npcm750_clk_GetPll1Freq();
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_pll2"))               ret_val =  npcm750_clk_GetPll2Freq();
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_gfx"))                ret_val =  2*npcm750_clk_GetPll2Freq();
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_apb1"))               ret_val =  npcm750_clk_GetAPBFreq(NPCM750_APB1);
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_apb2"))               ret_val =  npcm750_clk_GetAPBFreq(NPCM750_APB2);
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_apb3"))               ret_val =  npcm750_clk_GetAPBFreq(NPCM750_APB3);
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_apb4"))               ret_val =  npcm750_clk_GetAPBFreq(NPCM750_APB4);
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_apb5"))               ret_val =  npcm750_clk_GetAPBFreq(NPCM750_APB5);
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_mc"))                 ret_val =  npcm750_clk_GetMemoryFreq();
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_cpu"))                ret_val =  npcm750_clk_GetCPUFreq();
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_spi0"))               ret_val =  npcm750_clk_GetSPIFreq(NPCM750_SPI0);
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_spi3"))               ret_val =  npcm750_clk_GetSPIFreq(NPCM750_SPI3);
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_spix"))               ret_val =  npcm750_clk_GetSPIFreq(NPCM750_SPIX);
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_uart_core"))          ret_val =  npcm750_clk_GetUartFreq(); // guarentied by ROM, should not be chagned.
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_timer"))              ret_val =  npcm750_clk_GetTimerFreq();
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_host_uart"))          ret_val =  npcm750_clk_GetUartFreq();
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_mmc"))                ret_val =  npcm750_clk_GetSDFreq(NPCM750_CLK_SD);
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_sdhc"))               ret_val =  npcm750_clk_GetSDFreq(NPCM750_CLK_MMC);
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_adc"))                ret_val =  npcm750_clk_GetTimerFreq() /  (1 << READ_REG_FIELD(CLKDIV1, CLKDIV1_ADCCKDIV));
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_gfx_mem"))            ret_val =  npcm750_clk_GetPll0Freq();
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_usb_bridge"))         ret_val =  npcm750_clk_GetUSBClock();
	if(0 == strcmp(__clk_get_name(hw->clk), "clk_axi"))                ret_val =  npcm750_clk_Get_AXI_Freq(); // CLK2: l2cache etc.
	if( (0 == strcmp(__clk_get_name(hw->clk), "clk_ahb"))  || 
	    (0 == strcmp(__clk_get_name(hw->clk), "clk_emc"))  || 
	    (0 == strcmp(__clk_get_name(hw->clk), "clk_gmac"))  ) {
	    ret_val =  npcm750_clk_Get_AHB_Freq(); // CLK4: CP etc.
	}


	CLOCK_DEBUG("\t npcm750_clk_recalc for clock %s => %ld\n", hw->init->name, ret_val);

    return ret_val;	
}


int npcm750_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{
    CLOCK_DEBUG("\t npcm750_clk_set_rate for clock %s => %ld\n", hw->init->name, rate);
    return 0;
}

static int		npcm750_clk_enable_disable(struct clk_hw *hw, bool bEN)
{
	unsigned long ret_val = 0;

	if(0 == strcmp(__clk_get_name(hw->clk) , "clk_emc")) 
	{
        npcm750_clk_EnableEMCClock(0, bEN);
        npcm750_clk_EnableEMCClock(1, bEN);
        ret_val = 0;
    }
    if(0 == strcmp(__clk_get_name(hw->clk) , "clk_gmac")) 
	{
        npcm750_clk_EnableGMACClock(0, bEN);
        npcm750_clk_EnableGMACClock(1, bEN);
        ret_val = 0;
    }
    if(0 == strcmp(__clk_get_name(hw->clk) , "clk_sdhc")) 
	{
        npcm750_clk_EnableSDClock(NPCM750_CLK_SD, bEN);
        ret_val = 0;
    }
    if(0 == strcmp(__clk_get_name(hw->clk) , "clk_mmc")) 
	{
        npcm750_clk_EnableSDClock(NPCM750_CLK_MMC, bEN);
        ret_val = 0;
    }
    
   
    return ret_val;
}

static int		npcm750_clk_enable(struct clk_hw *hw)
{
    CLOCK_DEBUG("\tnpcm750_clk_enable for clock %s\n", __clk_get_name(hw->clk));
    return npcm750_clk_enable_disable(hw, true);
}


static void		npcm750_clk_disable(struct clk_hw *hw)
{
    CLOCK_DEBUG("\tnpcm750_clk_disable for clock %s\n", __clk_get_name(hw->clk));
    npcm750_clk_enable_disable(hw, false);
    return;
}


static const struct clk_ops npcm750_clk_ops = {
	.recalc_rate = npcm750_clk_recalc,
	//.set_rate    = npcm750_clk_set_rate, // not needed. setting is done by BB.
	.enable      = npcm750_clk_enable,
	.disable     = npcm750_clk_disable,
};

/*
 * CLK definitions
 */


#define DECLARE_NUVOTON_CLK(_clk_name, _clk_name_str)  static struct npcm750_clk _clk_name = { \
	.hw.init = &(struct clk_init_data){ \
		.name = _clk_name_str, \
		.parent_names = (const char *[]){}, \
		.num_parents = 0, \
		.ops = &npcm750_clk_ops,	},};

DECLARE_NUVOTON_CLK(clk_pll0,         "clk_pll0");
DECLARE_NUVOTON_CLK(clk_pll1,         "clk_pll1");
DECLARE_NUVOTON_CLK(clk_pll2,         "clk_pll2");
DECLARE_NUVOTON_CLK(clk_gfx ,         "clk_gfx");
DECLARE_NUVOTON_CLK(clk_apb1,         "clk_apb1");
DECLARE_NUVOTON_CLK(clk_apb2,         "clk_apb2");
DECLARE_NUVOTON_CLK(clk_apb3,         "clk_apb3");
DECLARE_NUVOTON_CLK(clk_apb4,         "clk_apb4");
DECLARE_NUVOTON_CLK(clk_apb5,         "clk_apb5");
DECLARE_NUVOTON_CLK(clk_mc  ,         "clk_mc");
DECLARE_NUVOTON_CLK(clk_cpu ,         "clk_cpu");
DECLARE_NUVOTON_CLK(clk_spi0,         "clk_spi0");
DECLARE_NUVOTON_CLK(clk_spi3,	      "clk_spi3");
DECLARE_NUVOTON_CLK(clk_spix,	      "clk_spix");
DECLARE_NUVOTON_CLK(clk_uart_core,	  "clk_uart_core");
DECLARE_NUVOTON_CLK(clk_timer,		  "clk_timer");
DECLARE_NUVOTON_CLK(clk_host_uart,	  "clk_host_uart");
DECLARE_NUVOTON_CLK(clk_mmc,		  "clk_mmc");
DECLARE_NUVOTON_CLK(clk_sdhc,		  "clk_sdhc");
DECLARE_NUVOTON_CLK(clk_adc, 	      "clk_adc");
DECLARE_NUVOTON_CLK(clk_gfx_mem,	  "clk_gfx_mem");
DECLARE_NUVOTON_CLK(clk_usb_bridge,   "clk_usb_bridge");
DECLARE_NUVOTON_CLK(clk_axi,	      "clk_axi");
DECLARE_NUVOTON_CLK(clk_ahb,	      "clk_ahb");
DECLARE_NUVOTON_CLK(clk_emc,	      "clk_emc");
DECLARE_NUVOTON_CLK(clk_gmac,	      "clk_gmac");



/* Table of all supported clocks indexed by the clock identifiers from the
 * device tree binding
 */
static struct npcm750_clk *npcm750clk_clocks[] = {
	/*[npcm750_CLK_FAB_PLL]  = &clk_pll0.sclk,*/
	
	[NPCMX50_CLK_PLL0	    ] = &clk_pll0,
	[NPCMX50_CLK_PLL1	    ] = &clk_pll1,
	[NPCMX50_CLK_PLL2	    ] = &clk_pll2,
	[NPCMX50_CLK_GFX  	    ] = &clk_gfx,
	[NPCMX50_CLK_APB1	    ] = &clk_apb1,
	[NPCMX50_CLK_APB2	    ] = &clk_apb2,
	[NPCMX50_CLK_APB3	    ] = &clk_apb3,
	[NPCMX50_CLK_APB4	    ] = &clk_apb4,
	[NPCMX50_CLK_APB5	    ] = &clk_apb5,
	[NPCMX50_CLK_MC	        ] = &clk_mc,
	[NPCMX50_CLK_CPU	    ] = &clk_cpu,	   
	[NPCMX50_CLK_SPI0	    ] = &clk_spi0,   
	[NPCMX50_CLK_SPI3	    ] = &clk_spi3,	   
	[NPCMX50_CLK_SPIX	    ] = &clk_spix,	   
	[NPCMX50_CLK_UART_CORE	] = &clk_uart_core,	
	[NPCMX50_CLK_TIMER		] = &clk_timer,		
	[NPCMX50_CLK_HOST_UART	] = &clk_host_uart,			
	[NPCMX50_CLK_MMC		] = &clk_mmc,		
	[NPCMX50_CLK_SDHC		] = &clk_sdhc,		
	[NPCMX50_CLK_ADC 	    ] = &clk_adc, 	   
	[NPCMX50_CLK_GFX_MEM 	] = &clk_gfx_mem,	
	[NPCMX50_CLK_USB_BRIDGE ] = &clk_usb_bridge	,
	[NPCMX50_CLK_AXI	    ] = &clk_axi,	   
	[NPCMX50_CLK_AHB	    ] = &clk_ahb,
	[NPCMX50_CLK_EMC	    ] = &clk_emc,	   
	[NPCMX50_CLK_GMAC	    ] = &clk_gmac,
};


struct npcm750clk_priv {
	struct clk_onecell_data onecell;
	struct clk *clks[];
};

static void __init npcm750clk_setup(struct device_node *np)
{	
	int i, ret;
	struct clk *clk;
	size_t num_clks;
	struct npcm750clk_priv *priv;

	CLOCK_DEBUG("\tnpcm750clk_setup\n");

	num_clks = ARRAY_SIZE(npcm750clk_clocks);
	pr_info("\tnpcm750 clk: supporting %u clocks\n", num_clks);	
	priv = kzalloc(sizeof(*priv) + sizeof(*priv->clks) * num_clks,
			    GFP_KERNEL);
	if (!priv)
	{
	    printk(" npcm750clk_setup: alloc error\n");
		ret = -ENOMEM;
		goto setup_err;
	}

	priv->onecell.clks = priv->clks;
	priv->onecell.clk_num = num_clks;

    /* Update each entry with the allocated regmap and register the clock
    * with the common clock framework
    */
    for (i = 0; i < num_clks; i++) {
       	CLOCK_DEBUG("\t register clk%d..(%s)\t", i, npcm750clk_clocks[i]->hw.init->name);
       	clk = clk_register(NULL, &npcm750clk_clocks[i]->hw);
       	if (IS_ERR(clk))
       	{
       	    printk("\tnpcm750clk_setup: register clk error\n");
       		ret = PTR_ERR(clk);
       		goto setup_err;
       	}
       	priv->clks[i] = clk;
    }

	ret = of_clk_add_provider(np, of_clk_src_onecell_get, &priv->onecell);

	printk("npcm750 clk setup done\n");	

	return;

setup_err:
    printk("npcm750clk_setup: failed to start. errno %d\n", ret);
	of_node_put(np);
	BUG();
}

CLK_OF_DECLARE(npcm750_clk, "nuvoton,npcm750-clk", npcm750clk_setup);





static const struct of_device_id npcm750clk_match_table[] = {
	{ .compatible = "nuvoton,npcm750-clk" },
	{ }
};
MODULE_DEVICE_TABLE(of, npcm750clk_match_table);





void __init nuvoton_npcm750_clock_init(void)
{
    struct device_node *np;
	struct resource res;
	
    printk("NPCM750: clock init\n");

	np = of_find_compatible_node(NULL, NULL, "nuvoton,npcm750-clk");
	if (!np) {
		pr_err("\t%s: node not found\n", __func__);
		goto np_err;
	}

	if (of_address_to_resource(np, 0, &res)) {
		pr_err("\t%s: failed to get resource\n", np->name);
		goto np_err;
	}

	clk_base = ioremap(res.start, resource_size(&res));
	if (IS_ERR(clk_base))
	{
	    printk("\tnuvoton_npcm750_clock_init: resource error\n");
		goto np_err;
	}
	
	of_node_put(np);

	return;
	
np_err:
	of_node_put(np);
	BUG();
}


MODULE_DESCRIPTION("npcm750 clock driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:npcm750-clk");
