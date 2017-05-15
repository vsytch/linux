/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2008 by Nuvoton Technology Corporation                                                   */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   mach-npcmX50.c                                                                                        */
/*            This file contains main NPCMX50 machine initialize                                           */
/*  Project:                                                                                               */
/*            Linux Kernel                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/types.h> 
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <mach/map.h>
#include <mach/hal.h>
#include <asm/system_misc.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/irqchip/arm-gic.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/cp15.h>
#include <linux/of_fdt.h>
#include <linux/platform_device.h>

#include "npcmX50_timer.h"

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach/irq.h>

#include <linux/irqchip.h>

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/mman.h>
#include <linux/nodemask.h>

#include <asm/mach/map.h>

#include <defs.h>
#include <mach/hal.h>
#include <mach/module_init.h>

#include "../mm/mm.h"


extern struct smp_operations platform_smp_ops;
void __init npcm750_timer_init(void);

#define NPCMX50_TIMER_PORTS_PER_MODULE          5
#define NPCMX50_TIMER_PORT_TO_MODULE(port)      ((port) / NPCMX50_TIMER_PORTS_PER_MODULE)
#define TIMER_WTCR(port)    (TIMER_VIRT_BASE_ADDR(NPCMX50_TIMER_PORT_TO_MODULE(port))+ 0x1C), NPCMX50_TIMER_ACCESS, 32     // R/W Watchdog Timer Control Register 0000_0400h


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        arch_reset                                                                             */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  cmd -                                                                                  */
/*                  mode -                                                                                 */
/*                                                                                                         */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs HW reset                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static inline void arch_reset(enum reboot_mode mode, const char *cmd)
{ 
    //TIMER_WatchdogReset(TIMER5_DEV);
    u32  timerNum = 5;
    volatile INT i = 1;
    REG_WRITE(TIMER_WTCR(timerNum), 0x83);
    while(i);
    
}

typedef struct 
{
    UINT32  phys_base;
    UINT32  virt_base;
    UINT32  size;
}iomemory_block_t;

/*---------------------------------------------------------------------------------------------------------*/
/* Array of IO memory blocks from the Chip definition in HAL                                               */
/*---------------------------------------------------------------------------------------------------------*/
static iomemory_block_t iomem_blocks[] __initdata = NPCMX50_IOMEMORY_BLOCKS;

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcmX50_map_io                                                                         */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs IO mapping for the given chip                                    */
/*---------------------------------------------------------------------------------------------------------*/
void __init npcmX50_map_io(void)
{
    int i = 0;
    struct map_desc desc[ARRAY_SIZE(iomem_blocks)];

    printk(KERN_NOTICE "NPCMX50: Mapping IO Blocks:\n");

    for (i=0; i<ARRAY_SIZE(iomem_blocks); ++i)
    {
        desc[i].virtual    = iomem_blocks[i].virt_base;
        desc[i].pfn        = __phys_to_pfn(iomem_blocks[i].phys_base);
        desc[i].length     = iomem_blocks[i].size;
		desc[i].type	   = MT_DEVICE;
  
        printk(KERN_NOTICE "Virtual [0x%lX], Physical [0x%x], Size [0x%lX]\n", \
                            desc[i].virtual, iomem_blocks[i].phys_base, desc[i].length);
    }   
	
	iotable_init(desc,ARRAY_SIZE(iomem_blocks));	 
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcmX50_irq_init                                                                       */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs IRQ initialization                                               */
/*---------------------------------------------------------------------------------------------------------*/
void __init npcmX50_irq_init(void)
{
   //gic_set_irqchip_flags(IRQCHIP_SKIP_SET_WAKE | IRQCHIP_MASK_ON_SUSPEND);
   irqchip_init();
}

void __init npcmX50_init_cache(void)
{
#if CONFIG_CACHE_L2X0

   /* PL310 Set TAG/DATA latency's */  
   writel_relaxed(0x000,(void __iomem *)(NPCMX50_L2_CACHE_BASE_ADDR + L310_TAG_LATENCY_CTRL));
   writel_relaxed(0x000,(void __iomem *)(NPCMX50_L2_CACHE_BASE_ADDR + L310_DATA_LATENCY_CTRL));
  /* PL310 Set D/I prefetch enable and cache replacement policy */ 
  
   l2x0_init(IOMEM(NPCMX50_L2_CACHE_BASE_ADDR), 0x36400001, ~0x36400001);

   /* A9 D/I prefetch enable bits*/
   set_auxcr(get_auxcr() | BIT(3) | BIT(2) | BIT(1));

#endif
}

void __init npcmX50_init_mach(void)
{    
	#ifndef CONFIG_SMP
	npcmX50_init_cache();
	#endif
    arm_pm_restart = arch_reset;
	npcmx50_module_init();
    printk(KERN_NOTICE "NPCMX50: Architecture initialized\n");
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);      
}

static const char * const npcmX50_dt_match[] = {
	"nuvoton,npcmx50",
	NULL
};

/*---------------------------------------------------------------------------------------------------------*/
/* In future we should move to NPCMX50 upon mach-types will be updated in both in Kernel and U-Boot        */
/* releases                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
DT_MACHINE_START(WPCM450, "NPCMX50 Chip family") 
#ifdef CONFIG_SMP
  .smp    = smp_ops(platform_smp_ops),
#endif
  .map_io         = npcmX50_map_io,
  .init_irq       = npcmX50_irq_init,
  .atag_offset 	  = 0x100,
  .init_machine   = npcmX50_init_mach,
  .init_time      = npcm750_timer_init,
  .dt_compat	  = npcmX50_dt_match,
MACHINE_END
