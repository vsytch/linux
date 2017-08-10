/*
 * Copyright (c) 2014 Nuvoton Technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

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
#include <linux/irqchip.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/mman.h>
#include <linux/nodemask.h>
#include <asm/mach/map.h>

//#include <defs.h>
#include <mach/hal.h>
#include <mach/module_init.h>

extern struct smp_operations platform_smp_ops;

typedef struct
{
	u32  phys_base;
	u32  virt_base;
	u32  size;
}iomemory_block_t;

/* Array of IO memory blocks from the Chip definition in HAL*/
static iomemory_block_t iomem_blocks[]__initdata = NPCMX50_IOMEMORY_BLOCKS;

void __init npcmX50_map_io(void)
{
	int i = 0;
	struct map_desc desc[ARRAY_SIZE(iomem_blocks)];

	printk(KERN_NOTICE "NPCMX50: Mapping IO Blocks:\n");

	for (i = 0; i < ARRAY_SIZE(iomem_blocks); ++i) {
		desc[i].virtual = iomem_blocks[i].virt_base;
		desc[i].pfn     = __phys_to_pfn(iomem_blocks[i].phys_base);
		desc[i].length  = iomem_blocks[i].size;
		desc[i].type	= MT_DEVICE;

		printk(KERN_NOTICE "Virtual [0x%lX], Physical [0x%x], Size [0x%lX]\n",\
		       desc[i].virtual, iomem_blocks[i].phys_base, desc[i].length);
	}

	iotable_init(desc, ARRAY_SIZE(iomem_blocks));
}

void __init npcmX50_init_cache(void)
{
	if (!IS_ENABLED(CONFIG_CACHE_L2X0))
		return;

	/* PL310 Set TAG/DATA latency's */
	writel_relaxed(0x000, (void __iomem *)(NPCMX50_L2_CACHE_BASE_ADDR
					       + L310_TAG_LATENCY_CTRL));
	writel_relaxed(0x000, (void __iomem *)(NPCMX50_L2_CACHE_BASE_ADDR
					       + L310_DATA_LATENCY_CTRL));
	/* PL310 Set D/I prefetch enable and cache replacement policy */
	l2x0_init(IOMEM(NPCMX50_L2_CACHE_BASE_ADDR), 0x36400001, ~0x36400001);

	/* A9 D/I prefetch enable bits*/
	set_auxcr(get_auxcr() | BIT(3) | BIT(2) | BIT(1));
}

void __init npcmX50_init_mach(void)
{
	#ifndef CONFIG_SMP
	npcmX50_init_cache();
	#endif

	//arm_pm_restart = arch_reset;
	printk(KERN_NOTICE "NPCMX50: Architecture initialized\n");
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static const char *const npcmX50_dt_match[] = {
	"nuvoton,npcmx50",
	NULL
};

/*---------------------------------------------------------------------------------------------------------*/
/* In future we should move to NPCMX50 upon mach-types will be updated in both in Kernel and U-Boot        */
/* releases                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
DT_MACHINE_START(NPCMX50_DT, "NPCMX50 Chip family")
#ifdef CONFIG_SMP
.smp    = smp_ops(platform_smp_ops),
#endif
.map_io         = npcmX50_map_io,
.atag_offset 	  = 0x100,
.init_machine   = npcmX50_init_mach,
.dt_compat	  = npcmX50_dt_match,
MACHINE_END
