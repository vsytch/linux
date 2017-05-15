/*
 *  Copyright (C) 2016 Dell EMC Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/mach/arch.h>
#include <linux/of.h>
#include <linux/io.h>
#include <mach/map.h>

#ifdef __iDRAC__
#define CPLD_BASE 0xf8000000
#endif

int planar_type=1;
EXPORT_SYMBOL(planar_type);

#ifdef __iDRAC__
static int __init gpio_read(int pin)
{
	int bank;
	u8 *base;
	u32 mask, val;
	const void __iomem *vaddr;

	bank = pin / (NPCMX50_GPIO_NUM_OF_GPIOS/NPCMX50_GPIO_NUM_OF_PORTS);
	base = NPCMX50_GPIO_PHYS_BASE_ADDR(bank);
	mask = (1L << (pin % (NPCMX50_GPIO_NUM_OF_GPIOS/NPCMX50_GPIO_NUM_OF_PORTS)));

	vaddr = ioremap(base, 0x1000);
	if (!vaddr) {
		printk(KERN_CRIT "%s: can't ioremap registers\n",
			__FUNCTION__);
		iounmap(vaddr);
		return -1;
	}
	val = ioread32(vaddr+0x4);
	iounmap(vaddr);
	val &= mask;
	return !!val;
}
#endif 

static int __init planar_init(void)
{
	struct device_node *np;
	static const char *model = "unknown";
	
#ifdef __iDRAC__
	void __iomem *vaddr;
#endif

	char buf[80];
	int id;

	np = of_find_node_by_path("/");
	if (!np) {
		printk(KERN_CRIT "%s: can't find node \"/\"\n", __FUNCTION__);
		return -1;
	}

	of_property_read_string(np, "model", &model);
	of_node_put(np);
	
#ifdef __iDRAC__
	/* ec code */
	if (!strcmp(model, "npcm750-ec")) {
		printk(KERN_INFO "%s: device tree model=%s\n",
			__FUNCTION__, model);
		return 0;
	}

	/* idrac code */
	vaddr = ioremap(CPLD_BASE, 64);
	if (!vaddr) {
		printk(KERN_CRIT "%s: can't ioremap registers\n",
			__FUNCTION__);
		iounmap(vaddr);
		return -1;
	}
	planar_type = ioread8(vaddr+2);
	iounmap(vaddr);
	printk(KERN_INFO "%s: model=%s planar type 0x%02x\n",
		__FUNCTION__, model, planar_type);
#endif

	snprintf(buf, sizeof(buf), "/planar@%02x", planar_type);
	buf[sizeof(buf)-1] = 0;
	np = of_find_node_by_path(buf);
	if (!np) {
		printk(KERN_CRIT "%s: can't find node for %s\n",
			__FUNCTION__, buf);
		return -1;
	}
	id = of_overlay_create(np);
	of_node_put(np);
	if (id < 0) {
		printk(KERN_CRIT "%s: could not create %s overlay id %d\n",
			__FUNCTION__, buf,id);
		return -1;
	}

	return 0;
}

arch_initcall(planar_init);
