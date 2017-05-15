/*
 * npcmX50_module_init.c
 *
 * Provides module initialization at the general and mux registers. 
 *
 * Copyright (C) 2016 Nuvoton Technologies tomer.maimon@nuvoton.com
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/types.h> 
#include <linux/io.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <asm/delay.h>

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/spinlock.h>

#include <defs.h>
#include <mach/hal.h>
#include <mach/map.h>

#include <mach/regs_npcm750_gcr.h>
#include <mach/regs_npcm750_clk.h>


/*---------------------------------------------------------------------------------------------------------*/
/* gcr and mux spinlock                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
static DEFINE_SPINLOCK(npcmx50_gcr_lock);

/**********************************************************************************/
/********************			Service Routines 			***********************/
/**********************************************************************************/

void npcmx50_spin_lock_irqsave(unsigned long *flags)
{
	unsigned long local_flags;
	
    spin_lock_irqsave(&npcmx50_gcr_lock,local_flags);
	
	*flags = local_flags;
}
EXPORT_SYMBOL(npcmx50_spin_lock_irqsave);

void npcmx50_spin_unlock_irqrestore(unsigned long flags)
{
    spin_unlock_irqrestore(&npcmx50_gcr_lock,flags);
}
EXPORT_SYMBOL(npcmx50_spin_unlock_irqrestore);

void npcmx50_atomic_io_modify(void __iomem *reg, u32 mask, u32 set)
{
	unsigned long flags;
	u32 value;
	
    spin_lock_irqsave(&npcmx50_gcr_lock,flags);
	value = readl_relaxed(reg) & ~mask;
	value |= (set & mask);
	writel(value, reg);
	spin_unlock_irqrestore(&npcmx50_gcr_lock,flags);
}
EXPORT_SYMBOL(npcmx50_atomic_io_modify);

/**********************************************************************************/
/********************			   Init Routines 			***********************/
/**********************************************************************************/

void npcmx50_udc_replace_usb9(void)
{
	unsigned long flags;

	spin_lock_irqsave(&npcmx50_gcr_lock,flags);
	
	SET_REG_FIELD(INTCR3, INTCR3_USBPHYSW, 3);
	
	spin_unlock_irqrestore(&npcmx50_gcr_lock,flags);
	
}
EXPORT_SYMBOL(npcmx50_udc_replace_usb9);

static void npcmx50_ehci_init_reset(void)
{
	/********* phy init  ******/
	// reset usb host

	SET_REG_FIELD(IPSRST2, IPSRST2_USBHOST, 1);



	SET_REG_FIELD(IPSRST3, IPSRST3_USBPHY2, 1);// set  USBPHY2	to reset
		
	SET_REG_FIELD(USB2PHYCTL, USBPHY2_RS, 0);

	udelay(1);

	// enable phy
	SET_REG_FIELD(IPSRST3, IPSRST3_USBPHY2, 0);// release USBPHY2  from reset
	
		
	udelay(50); // enable phy


	SET_REG_FIELD(USB2PHYCTL, USBPHY2_RS, 1);// set RS (Reset Sequence) bit

	// enable host
	SET_REG_FIELD(IPSRST2, IPSRST2_USBHOST, 0);

}

static void npcmx50_udc_init_reset(void)
{

	SET_REG_FIELD(IPSRST1, IPSRST1_USBDEV6, 1);
	SET_REG_FIELD(IPSRST1, IPSRST1_USBDEV5, 1);
	SET_REG_FIELD(IPSRST1, IPSRST1_USBDEV4, 1);
	SET_REG_FIELD(IPSRST1, IPSRST1_USBDEV3, 1);
	SET_REG_FIELD(IPSRST1, IPSRST1_USBDEV2, 1);
	SET_REG_FIELD(IPSRST1, IPSRST1_USBDEV1, 1);
	
	
	SET_REG_FIELD(IPSRST3, IPSRST3_USBHUB, 1);
	SET_REG_FIELD(IPSRST3, IPSRST3_USBDEV9, 1);
	SET_REG_FIELD(IPSRST3, IPSRST3_USBDEV8, 1);
	SET_REG_FIELD(IPSRST3, IPSRST3_USBDEV7, 1);
	SET_REG_FIELD(IPSRST3, IPSRST3_USBDEV0, 1);
	
	
	SET_REG_FIELD(IPSRST3, IPSRST3_USBPHY1, 1);// set  USBPHY1 to reset
	
	SET_REG_FIELD(USB1PHYCTL, USBPHY1_RS, 0);
	
	SET_REG_FIELD(IPSRST3, IPSRST3_USBPHY1, 0);// release  USBPHY1 from reset
	
	udelay(50); // enable phy
	
	SET_REG_FIELD(USB1PHYCTL, USBPHY1_RS, 1);// set RS (Reset Sequence) bit
	
	 // enable hub
	
	SET_REG_FIELD(IPSRST3, IPSRST3_USBHUB, 0);

}


int npcmx50_udc_enable_devices(int udc_id)
{
	int ret=0;
	unsigned long flags;

	spin_lock_irqsave(&npcmx50_gcr_lock,flags);

	// enable devices
	switch(udc_id)
	{
		case 0:
			SET_REG_FIELD(IPSRST3, IPSRST3_USBDEV0, 0);
			break;
		case 1:
			SET_REG_FIELD(IPSRST1, IPSRST1_USBDEV1, 0);
			break;
		case 2:
			SET_REG_FIELD(IPSRST1, IPSRST1_USBDEV2, 0);
			break;
		case 3:
			SET_REG_FIELD(IPSRST1, IPSRST1_USBDEV3, 0);
			break;
		case 4:
			SET_REG_FIELD(IPSRST1, IPSRST1_USBDEV4, 0);
			break;
		case 5:
			SET_REG_FIELD(IPSRST1, IPSRST1_USBDEV5, 0);
			break;
		case 6:
			SET_REG_FIELD(IPSRST1, IPSRST1_USBDEV6, 0);
			break;
		case 7:
			SET_REG_FIELD(IPSRST3, IPSRST3_USBDEV7, 0);
			break;
		case 8:
			SET_REG_FIELD(IPSRST3, IPSRST3_USBDEV8, 0);
			break;
		case 9:
			SET_REG_FIELD(IPSRST3, IPSRST3_USBDEV9, 0);
			break;
		default :
			ret = -ENODEV;
	}
	
	spin_unlock_irqrestore(&npcmx50_gcr_lock,flags);

	return ret;
}
EXPORT_SYMBOL(npcmx50_udc_enable_devices);

static void npcmx50_emc_probe_mux_rst(int emc_id)
{

    /*-----------------------------------------------------------------------------------------------------*/
    /* Muxing RMII MDIO                                                                                    */
    /*-----------------------------------------------------------------------------------------------------*/
    if(emc_id == 0)            // ETH0 - EMC1
    {
        SET_REG_FIELD(MFSEL3, MFSEL3_RMII1SEL, 1);
        SET_REG_FIELD(MFSEL1, MFSEL1_R1MDSEL, 1);
        SET_REG_FIELD(MFSEL1, MFSEL1_R1ERRSEL, 1);
        SET_REG_FIELD(INTCR,  INTCR_R1EN, 1);
    }
    else if(emc_id == 1)       // ETH1 - EMC2
    {
        SET_REG_FIELD(MFSEL1, MFSEL1_RMII2SEL, 1);
        SET_REG_FIELD(MFSEL1, MFSEL1_R2MDSEL, 1);
        SET_REG_FIELD(MFSEL1, MFSEL1_R2ERRSEL, 1);
    }

    /*-----------------------------------------------------------------------------------------------------*/
    /* Reset EMC module                                                                                     */
    /*-----------------------------------------------------------------------------------------------------*/
    if (emc_id == 0)
    {
        SET_REG_FIELD(IPSRST1, IPSRST1_EMC1, 1);
        SET_REG_FIELD(IPSRST1, IPSRST1_EMC1, 0);
    }
    else if (emc_id == 1)
    {
        SET_REG_FIELD(IPSRST1, IPSRST1_EMC2, 1);
        SET_REG_FIELD(IPSRST1, IPSRST1_EMC2, 0);
    }
	
}

#if 0
static void npcmx50_gmac_probe_mux_rst(int gmac_id)
{

	/*-----------------------------------------------------------------------------------------------------*/
	/* Setting ETH muxing																				 */
	/*-----------------------------------------------------------------------------------------------------*/
    if (gmac_id == 0)            // ETH2 - GMAC1
    {
        SET_REG_FIELD(MFSEL4, MFSEL4_RG1SEL, 1);
        SET_REG_FIELD(MFSEL4, MFSEL4_RG1MSEL, 1);
    }
    else if (gmac_id == 1)       // ETH3 - GMAC2
    {
        SET_REG_FIELD(MFSEL4, MFSEL4_RG2SEL, 1);
        SET_REG_FIELD(MFSEL4, MFSEL4_RG2MSEL, 1);
    }
	
	/*-----------------------------------------------------------------------------------------------------*/
	/* Reseting the device																				 */
	/*-----------------------------------------------------------------------------------------------------*/
    if (gmac_id == 0)
	{
        SET_REG_FIELD(IPSRST2, IPSRST2_GMAC1, 1);
        SET_REG_FIELD(IPSRST2, IPSRST2_GMAC1, 0);
	}
	else if (gmac_id == 1)
	{
        SET_REG_FIELD(IPSRST2, IPSRST2_GMAC2, 1);
        SET_REG_FIELD(IPSRST2, IPSRST2_GMAC2, 0);
	}

}

void npcmx50_sdhci_probe_mux(int sdhci_id)
{
	unsigned long flags;

	spin_lock_irqsave(&npcmx50_gcr_lock,flags); 
	
    if (sdhci_id == SD1_DEV)  //     SD Card -  SDHC1
    {
	    SET_REG_FIELD(MFSEL3, MFSEL3_SD1SEL, 1);      
    }
    else if (sdhci_id == SD2_DEV)	//     eMMC  -   SDHC2
    {
        SET_REG_FIELD(MFSEL3, MFSEL3_MMCSEL, 1);
        SET_REG_FIELD(MFSEL3, MFSEL3_MMCCDSEL, 0); 
        SET_REG_FIELD(MFSEL4, MFSEL4_MMCRSEL, 1);
        SET_REG_FIELD(MFSEL3, MFSEL3_MMC8SEL, 1);
    }
	
	spin_unlock_irqrestore(&npcmx50_gcr_lock,flags);	
}
EXPORT_SYMBOL(npcmx50_sdhci_probe_mux);

void npcmx50_sdhci_probe_rst(int sdhci_id)
{
	unsigned long flags=0;

	spin_lock_irqsave(&npcmx50_gcr_lock,flags);

    if (sdhci_id == SD1_DEV)
    {
        SET_REG_FIELD(IPSRST2, IPSRST2_SDHC, 1);
        SET_REG_FIELD(IPSRST2, IPSRST2_SDHC, 0);
    }
    else if (sdhci_id == SD2_DEV)
    {
        SET_REG_FIELD(IPSRST2, IPSRST2_MMC, 1);
        SET_REG_FIELD(IPSRST2, IPSRST2_MMC, 0);
    }
	
	spin_unlock_irqrestore(&npcmx50_gcr_lock,flags);
}
EXPORT_SYMBOL(npcmx50_sdhci_probe_rst);
#endif 

void npcmx50_gcr_mux_uart(UINT redirection_mode, BOOLEAN CoreSP, BOOLEAN sp1, BOOLEAN sp2)
{
	//unsigned long flags;

	/* 111 combination is reserved: */
    if (redirection_mode >= 7)
	{
        return;
	}
	
	//spin_lock_irqsave(&npcmx50_gcr_lock,flags);

    SET_REG_FIELD(SPSWC, SPSWC_SPMOD, redirection_mode & 0x7); /* redirection mode number in enum == value at register */

    if (CoreSP)
	{
        SET_REG_FIELD(MFSEL1, MFSEL1_BSPSEL,  1);
        SET_REG_FIELD(MFSEL4, MFSEL4_BSPASEL, 1); /* use BSPRXD + BSPTXD */
	}
    if (sp1)
	{
        SET_REG_FIELD(MFSEL1, MFSEL1_HSI1SEL, 1);
        SET_REG_FIELD(MFSEL4, MFSEL4_BSPASEL, 0); /* Select TXD2+RXD2 */
	}
    if (sp2)
	{
        SET_REG_FIELD(MFSEL1, MFSEL1_HSI2SEL, 1);
        SET_REG_FIELD(MFSEL4, MFSEL4_BSPASEL, 0); /* Select TXD2+RXD2 */
	}

    // spin_unlock_irqrestore(&npcmx50_gcr_lock,flags);
	return;
}
EXPORT_SYMBOL(npcmx50_gcr_mux_uart);


void npcmx50_module_init(void)
{
	#ifdef CONFIG_NPCM750_EMC_ETH
	  	npcmx50_emc_probe_mux_rst(0);
	   	npcmx50_emc_probe_mux_rst(1);
	#endif
	
	#ifdef CONFIG_USB_EHCI_HCD
		npcmx50_ehci_init_reset();
	#endif
	
	#ifdef CONFIG_USB_NPCMX50_USB2
		npcmx50_udc_init_reset();
	#endif
	
	#ifdef CONFIG_STMMAC_ETH
	/* not need to be done */
		/*npcmx50_gmac_probe_mux_rst(0);
		npcmx50_gmac_probe_mux_rst(1);*/
	#endif
	
}
EXPORT_SYMBOL(npcmx50_module_init);






