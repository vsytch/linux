/*
 * Copyright (C) 2002 ARM Ltd.
 * Copyright (C) 2008 STMicroelctronics.
 * Copyright (C) 2009 ST-Ericsson.
 * Author: Srinidhi Kasagar <srinidhi.kasagar@stericsson.com>
 *
 * This file is based on arm realview platform
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/smp.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <asm/smp.h>
#include <linux/irqchip/arm-gic.h>
#include <asm/smp_plat.h>
#include <asm/smp_scu.h>
//#include <mach/hardware.h>

#include <mach/hal.h>
//--------------#include <mach/map.h>
#include <mach/regs_npcm750_gcr.h>

//#include <mach/setup.h>


extern void gic_raise_softirq(const struct cpumask *mask, unsigned int irq);
extern void __init npcmX50_init_cache(void);


struct smp_operations platform_smp_ops;

/* This is called from headsmp.S to wakeup the secondary core */
extern void npcmX50_secondary_startup(void);
extern void npcmX50_wakeup_z1(void);

/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen"
 */
//volatile int pen_release = -1;

/*
 * Write pen_release in a way that is guaranteed to be visible to all
 * observers, irrespective of whether they're taking part in coherency
 * or not.  This is necessary for the hotplug code to work reliably.
 */
static void write_pen_release(int val)
{
	pen_release = val;
	smp_wmb();
	__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
	outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
}



static DEFINE_SPINLOCK(boot_lock);

void platform_secondary_init(unsigned int cpu)
{
	/*
	 * if any interrupts are already enabled for the primary
	 * core (e.g. timer irq), then they will not have been enabled
	 * for us: do so
	 */
	//gic_secondary_init(0);

	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	write_pen_release(-1);

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}


int platform_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;

	/*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/*
	 * The secondary processor is waiting to be released from
	 * the holding pen - release it, then wait for it to flag
	 * that it has been released by resetting pen_release.
	 */
	write_pen_release(cpu_logical_map(cpu));

	REG_WRITE(SCRPAD,  virt_to_phys(npcmX50_secondary_startup) );
	smp_rmb();

	printk(KERN_NOTICE "NPCMX50: npcmX50_secondary_startup = %p ,"
			"virt_to_phys(npcmX50_secondary_startup) = 0x%x \n",npcmX50_secondary_startup,
			virt_to_phys(npcmX50_secondary_startup));

	//Changed by AMI to wake up the secondary core(Core 1)
	arch_send_wakeup_ipi_mask(cpumask_of(cpu));
	
	//npcmX50_wakeup_z1();
	printk(KERN_NOTICE "NPCMX50: after npcmX50_wakeup_z1 \n");

//	smp_send_reschedule(cpu);

	timeout  = jiffies + (HZ*1);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;
			
		udelay(10);
	}
#if 1

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	printk(KERN_NOTICE "========= NPCMX50: boot_secondary after wait  \n");

	return pen_release != -1 ? -ENOSYS : 0;
#else
	return 0;
#endif
}

static void __init wakeup_secondary(void)
{
//	void __iomem *backupram;
//
//	if (cpu_is_u5500())
//		backupram = __io_address(U5500_BACKUPRAM0_BASE);
//	else if (cpu_is_u8500())
//		backupram = __io_address(U8500_BACKUPRAM0_BASE);
//	else
//		ux500_unknown_soc();
//
//	/*
//	 * write the address of secondary startup into the backup ram register
//	 * at offset 0x1FF4, then write the magic number 0xA1FEED01 to the
//	 * backup ram register at offset 0x1FF0, which is what boot rom code
//	 * is waiting for. This would wake up the secondary core from WFE
//	 */
//#define UX500_CPU1_JUMPADDR_OFFSET 0x1FF4
	REG_WRITE(SCRPAD,  virt_to_phys(npcmX50_secondary_startup) );


//#define UX500_CPU1_WAKEMAGIC_OFFSET 0x1FF0
//	__raw_writel(0xA1FEED01,
//		     backupram + UX500_CPU1_WAKEMAGIC_OFFSET);
	smp_wmb();
	dsb_sev();
	/* make sure write buffer is drained */
	mb();
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init platform_smp_init_cpus(void)
{
	void __iomem *scu_base = IOMEM(NPCMX50_SCU_BASE_ADDR);
	unsigned int i, ncores;

	ncores = scu_get_core_count(scu_base);

	printk(KERN_NOTICE "NPCMX50: smp_init_cpus , ncores = %d \n" , ncores);

	/* sanity check */
	if (ncores > nr_cpu_ids) {
		pr_warn("SMP: %u cores greater than maximum (%u), clipping\n",
			ncores, nr_cpu_ids);
		ncores = nr_cpu_ids;
	}

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);

//	set_smp_cross_call(gic_raise_softirq);
}

void __init platform_smp_prepare_cpus(unsigned int max_cpus)
{
	
#if CONFIG_CACHE_L2X0
	npcmX50_init_cache();
#endif

	scu_enable((void __iomem *)NPCMX50_SCU_BASE_ADDR);
	wakeup_secondary();

	printk(KERN_NOTICE "NPCMX50: platform_smp_prepare_cpus \n");

}

struct smp_operations platform_smp_ops __initdata = {
  .smp_init_cpus    = platform_smp_init_cpus,
  .smp_prepare_cpus = platform_smp_prepare_cpus,
  .smp_boot_secondary = platform_boot_secondary,
  .smp_secondary_init = platform_secondary_init,
};

