/*
 * Copyright (c) 2014-2017 Nuvoton Technology corporation.
 *
 * Released under the GPLv2 only.
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/watchdog.h>
#include <asm/fiq.h>
#include <linux/of_irq.h>

/* Wdog0-2 are connected with OR gate directly to nFIQ. */
#define WATCHDOG_FIQ

/* #define DEBUG */
#ifdef DEBUG
	#undef WDOG_DEBUG
	#define WDOG_DEBUG(f, x...)	pr_info("NPCM7XX-WDOG: %s():"
					f, __func__, ## x)
#else
	#define WDOG_DEBUG(f, x...)
#endif

#define REG_WTCR	0x1C	/* WTCR Register Offset */

#define WTCLK		(0x03 << 10)
#define WTE		(0x01 << 7)	/* WTCR enable*/
#define WTIE		(0x01 << 6)	/* WTCR enable interrupt*/
#define WTIS		(0x03 << 4)	/* WTCR interval selection */
#define WTIF		(0x01 << 3)	/* WTCR interrupt flag*/
#define WTRF		(0x01 << 2)	/* WTCR reset flag */
#define WTRE		(0x01 << 1)	/* WTCR reset enable */
#define WTR		(0x01 << 0)	/* WTCR reset counter */

/*
 *Watchdog timeouts
 *
 *170     msec:    WTCLK=01 WTIS=00     VAL= 0x400
 *670     msec:    WTCLK=01 WTIS=01     VAL= 0x410
 *1360    msec:    WTCLK=10 WTIS=00     VAL= 0x800
 *2700    msec:    WTCLK=01 WTIS=10     VAL= 0x420
 *5360    msec:    WTCLK=10 WTIS=01     VAL= 0x810
 *10700   msec:    WTCLK=01 WTIS=11     VAL= 0x430
 *21600   msec:    WTCLK=10 WTIS=10     VAL= 0x820
 *43000   msec:    WTCLK=11 WTIS=00     VAL= 0xC00
 *85600   msec:    WTCLK=10 WTIS=11     VAL= 0x830
 *172000  msec:    WTCLK=11 WTIS=01     VAL= 0xC10
 *687000  msec:    WTCLK=11 WTIS=10     VAL= 0xC20
 *2750000 msec:    WTCLK=11 WTIS=11     VAL= 0xC30
 */

/* Default is 86 seconds */
#define NPCM7XX_WDOG_TIMEOUT	86

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0000);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct npcm7xx_wdt {
	struct resource		*res;
	struct platform_device	*pdev;
	void __iomem		*wdt_base;
	int irq;
};

static struct npcm7xx_wdt *npcm7xx_wdt;

static int npcm7xx_wdt_ping(struct watchdog_device *wdd)
{
	unsigned int val;

	WDOG_DEBUG("Ping\n");
	val = __raw_readl(npcm7xx_wdt->wdt_base + REG_WTCR);
	val |= WTR;
	WDOG_DEBUG("Ping REG_WTCR = 0x%x\n", val);
	__raw_writel(val, npcm7xx_wdt->wdt_base + REG_WTCR);
	return 0;
}

static int npcm7xx_wdt_start(struct watchdog_device *wdd)
{
	unsigned int val = 0;

	WDOG_DEBUG("Start timeout = %d\n", wdd->timeout);
	val |= (WTRE | WTE | WTR | WTIE);

	if (wdd->timeout < 2)
		val |= 0x800;
	else if (wdd->timeout < 3)
		val |= 0x420;
	else if (wdd->timeout < 6)
		val |= 0x810;
	else if (wdd->timeout < 11)
		val |= 0x430;
	else if (wdd->timeout < 22)
		val |= 0x820;
	else if (wdd->timeout < 44)
		val |= 0xC00;
	else if (wdd->timeout < 87)
		val |= 0x830;
	else if (wdd->timeout < 173)
		val |= 0xC10;
	else if (wdd->timeout < 688)
		val |= 0xC20;
	else if (wdd->timeout < 2751)
		val |= 0xC30;
	else
		val |= 0x830;

	WDOG_DEBUG("Start REG_WTCR = 0x%x\n", val);
	__raw_writel(val, npcm7xx_wdt->wdt_base + REG_WTCR);
	return 0;
}

static int npcm7xx_wdt_stop(struct watchdog_device *wdd)
{
	WDOG_DEBUG("Stop\n");
	__raw_writel(0, npcm7xx_wdt->wdt_base + REG_WTCR);
	return 0;
}


static int npcm7xx_wdt_set_timeout(struct watchdog_device *wdd,
				  unsigned int timeout)
{
	unsigned int val;

	WDOG_DEBUG("Timeout = %d\n", timeout);

	wdd->timeout = timeout;     /* New timeout */

	val = __raw_readl(npcm7xx_wdt->wdt_base + REG_WTCR);
	val &= ~(WTCLK | WTIS);

	if (wdd->timeout < 2)
		val |= 0x800;
	else if (wdd->timeout < 3)
		val |= 0x420;
	else if (wdd->timeout < 6)
		val |= 0x810;
	else if (wdd->timeout < 11)
		val |= 0x430;
	else if (wdd->timeout < 22)
		val |= 0x820;
	else if (wdd->timeout < 44)
		val |= 0xC00;
	else if (wdd->timeout < 87)
		val |= 0x830;
	else if (wdd->timeout < 173)
		val |= 0xC10;
	else if (wdd->timeout < 688)
		val |= 0xC20;
	else if (wdd->timeout < 2751)
		val |= 0xC30;
	else
		val |= 0x830;

	WDOG_DEBUG("Set Timeout REG_WTCR = 0x%x\n", val);
	__raw_writel(val, npcm7xx_wdt->wdt_base + REG_WTCR);

	return 0;
}

static int npcm7xx_wdt_restart(struct watchdog_device *wdd,
			      unsigned long action, void *data)
{

	__raw_writel(0x83, npcm7xx_wdt->wdt_base + REG_WTCR);
	udelay(1000);

	return 0;
}

#ifndef WATCHDOG_FIQ
/*
 * This interrupt occurs 10 ms before the watchdog WILL bark.
 */
static irqreturn_t npcm7xx_wdt_interrupt(int irq, void *data)
{
	unsigned int val;

	val = __raw_readl(npcm7xx_wdt->wdt_base + REG_WTCR);
	WDOG_DEBUG("%s() val = 0x%x\n", __func__, val);

	if (val & WTIF) {
		__raw_writel((val & ~WTIE) | WTIF,
			     npcm7xx_wdt->wdt_base + REG_WTCR);
		pr_info("NPCM7XX - Watchdog is barking!!!\n");

		/*
		 * Just disable and clear interrupt and await the imminent end.
		 * If you at some point need a host of callbacks to be called
		 * when the system is about to watchdog-reset, add them here!
		 *
		 * NOTE: on future versions of this IP-block, it will be
		 * possible to prevent a watchdog reset by feeding the
		 * watchdog at this point.
		 */
	}
	return IRQ_HANDLED;
}
#endif

static const struct watchdog_info npcm7xx_wdt_info = {
	.identity	= "npcm7xx watchdog",
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING
				| WDIOF_MAGICCLOSE,
};

static struct watchdog_ops npcm7xx_wdt_ops = {
	.owner = THIS_MODULE,
	.start = npcm7xx_wdt_start,
	.stop = npcm7xx_wdt_stop,
	.ping = npcm7xx_wdt_ping,
	.restart = npcm7xx_wdt_restart,
	.set_timeout = npcm7xx_wdt_set_timeout,
};

static struct watchdog_device npcm7xx_wdd = {
	.status = WATCHDOG_NOWAYOUT_INIT_STATUS,
	.info = &npcm7xx_wdt_info,
	.ops = &npcm7xx_wdt_ops,
	.min_timeout = 1,
	.max_timeout = 2751,
};

#ifdef WATCHDOG_FIQ
extern unsigned char npcm7xx_wdt_fiq_start, npcm7xx_wdt_fiq_end;
#endif

static int npcm7xx_wdt_probe(struct platform_device *pdev)
{
	int ret = 0;
#ifdef WATCHDOG_FIQ
	struct pt_regs regs;
#endif

	WDOG_DEBUG("Probing...\n");

	npcm7xx_wdt = devm_kzalloc(&pdev->dev, sizeof(struct npcm7xx_wdt),
				   GFP_KERNEL);
	if (!npcm7xx_wdt)
		return -ENOMEM;

	npcm7xx_wdt->pdev = pdev;

	npcm7xx_wdt->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (npcm7xx_wdt->res == NULL) {
		dev_err(&pdev->dev, "no memory resource specified\n");
		ret = -ENOENT;
		goto err_alloc;
	}

	if (!devm_request_mem_region(&pdev->dev, npcm7xx_wdt->res->start,
				resource_size(npcm7xx_wdt->res), pdev->name)) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		return -ENOENT;
	}

	npcm7xx_wdt->wdt_base = ioremap(npcm7xx_wdt->res->start,
					resource_size(npcm7xx_wdt->res));
	if (npcm7xx_wdt->wdt_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() region\n");
		ret = -EINVAL;
		goto err_mem;
	}

	WDOG_DEBUG("PA_ADDR = 0x%x\n", npcm7xx_wdt->res->start);
	WDOG_DEBUG("VA_ADDR = 0x%x\n", npcm7xx_wdt->wdt_base);

	npcm7xx_wdd.timeout = NPCM7XX_WDOG_TIMEOUT;

#ifdef WATCHDOG_FIQ

	set_fiq_handler(&npcm7xx_wdt_fiq_start,
			&npcm7xx_wdt_fiq_end - &npcm7xx_wdt_fiq_start);

//    regs.ARM_r10 = (long)DUMP_SRC;
//    regs.ARM_fp = (long)DUMP_DST;          // r11
	regs.ARM_ip = (long)npcm7xx_wdt->wdt_base;     // r12
	set_fiq_regs(&regs);
#else

	npcm7xx_wdt->irq = platform_get_irq(pdev, 0);
	WDOG_DEBUG("npcm7xx_wdt->irq = %d\n", npcm7xx_wdt->irq);

	if (request_irq(npcm7xx_wdt->irq, npcm7xx_wdt_interrupt,
			0, pdev->name, pdev)) {
		ret = -EIO;
		goto err_map;
	}
#endif

	watchdog_set_nowayout(&npcm7xx_wdd, nowayout);
	ret = watchdog_register_device(&npcm7xx_wdd);
	if (ret) {
		dev_err(&pdev->dev, "Error register watchdog device\n");
		goto err_irq;
	}

	WDOG_DEBUG("Probed\n");
	return 0;
err_irq:
	free_irq(npcm7xx_wdt->irq, pdev);
#ifndef WATCHDOG_FIQ
err_map:
#endif
	iounmap(npcm7xx_wdt->wdt_base);
err_mem:
	release_mem_region(npcm7xx_wdt->res->start,
			   resource_size(npcm7xx_wdt->res));
err_alloc:
	kfree(npcm7xx_wdt);
	return ret;
}

static int npcm7xx_wdt_remove(struct platform_device *pdev)
{
	WDOG_DEBUG("%s()\n", __func__);

	watchdog_unregister_device(&npcm7xx_wdd);
	iounmap(npcm7xx_wdt->wdt_base);
	release_mem_region(npcm7xx_wdt->res->start,
			   resource_size(npcm7xx_wdt->res));
	kfree(npcm7xx_wdt);

	return 0;
}

static void npcm7xx_wdt_shutdown(struct platform_device *pdev)
{
	WDOG_DEBUG("%s()\n", __func__);
	npcm7xx_wdt_stop(&npcm7xx_wdd);
}

#define npcm7xx_wdt_suspend	NULL
#define npcm7xx_wdt_resume	NULL

static const struct of_device_id wd_dt_id[] = {
	{.compatible = "nuvoton,npcm750-wdt"},
	{},
};
MODULE_DEVICE_TABLE(of, wd_dt_id);

static struct platform_driver npcm7xx_wdt_driver = {
	.probe		= npcm7xx_wdt_probe,
	.remove		= npcm7xx_wdt_remove,
	.shutdown	= npcm7xx_wdt_shutdown,
	.suspend	= npcm7xx_wdt_suspend,
	.resume		= npcm7xx_wdt_resume,
	.driver		= {
		.name	= "npcm7xx-wdt",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(wd_dt_id),
	},
};

module_platform_driver(npcm7xx_wdt_driver);

MODULE_DESCRIPTION("Watchdog driver for NPCM7XX");
MODULE_LICENSE("GPL");

