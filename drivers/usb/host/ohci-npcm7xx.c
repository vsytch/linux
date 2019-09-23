// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton NPCM7xx driver for OHCI HCD
 *
 * Copyright (C) 2019 Nuvoton Technologies,
 * Tomer Maimon <tomer.maimon@nuvoton.com> <tmaimon77@gmail.com>
 *
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "ohci.h"

#define DRIVER_DESC "OHCI NPCM7XX driver"

static const char hcd_name[] = "ohci-npcm7xx";
static struct hc_driver __read_mostly ohci_npcmx50_driver;

static int ohci_hcd_npcmx50_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	struct usb_hcd *hcd = NULL;
	int irq;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("platform_get_resource error.");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		pr_err("platform_get_irq error.");
		return -ENODEV;
	}

#ifdef CONFIG_OF
	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we have dma capability bindings this can go away.
	 */
	ret = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return -ENODEV;
#endif

	/* initialize hcd */
	hcd = usb_create_hcd(&ohci_npcmx50_driver, &pdev->dev, (char *)hcd_name);
	if (!hcd) {
		pr_err("Failed to create hcd");
		return -ENOMEM;
	}

	hcd->regs = (void __iomem *)res->start;
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		dev_dbg(&pdev->dev, "controller already in use\n");
		ret = -EBUSY;
		goto err_put_hcd;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (hcd->regs == NULL) {
		dev_dbg(&pdev->dev, "error mapping memory\n");
		ret = -EFAULT;
		goto err_release_region;
	}

	ret = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (ret != 0) {
		pr_err("Failed to add hcd");
		usb_put_hcd(hcd);
		return ret;
	}

	return ret;

	err_release_region:
		release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	err_put_hcd:
		usb_put_hcd(hcd);
	return ret;

}

static int ohci_hcd_npcmx50_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

	return 0;
}

#ifdef CONFIG_PM
static int npcm7xx_ohci_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	bool do_wakeup = device_may_wakeup(dev);
	int rc = ohci_suspend(hcd, do_wakeup);

	if (rc)
		return rc;

	return 0;
}

static int npcm7xx_ohci_resume(struct device *dev)
{
	struct usb_hcd *hcd			= dev_get_drvdata(dev);

	ohci_resume(hcd, false);

	return 0;
}
#else
#define npcm7xx_ohci_suspend	NULL
#define npcm7xx_ohci_resume	NULL
#endif


#ifdef CONFIG_OF
static const struct of_device_id npcm750_ohci_match[] = {
	{ .compatible = "nuvoton,npcm750-ohci" },
	{},
};
MODULE_DEVICE_TABLE(of, npcm750_ohci_match);
#endif

static const struct dev_pm_ops npcm7xx_ohci_pm_ops = {
	.suspend	= npcm7xx_ohci_suspend,
	.resume		= npcm7xx_ohci_resume,
};

static struct platform_driver ohci_hcd_npcmx50_driver = {
	.probe		= ohci_hcd_npcmx50_probe,
	.remove		= ohci_hcd_npcmx50_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.name	= "npcmx50-ohci",
		.pm	= &npcm7xx_ohci_pm_ops,
		.of_match_table = of_match_ptr(npcm750_ohci_match),
	},
};

static int __init ohci_npcm7xx_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("%s: " DRIVER_DESC "\n", hcd_name);
	ohci_init_driver(&ohci_npcmx50_driver, NULL);
	return platform_driver_register(&ohci_hcd_npcmx50_driver);
}
module_init(ohci_npcm7xx_init);

static void __exit ohci_npcm7xx_cleanup(void)
{
	platform_driver_unregister(&ohci_hcd_npcmx50_driver);
}
module_exit(ohci_npcm7xx_cleanup);

MODULE_ALIAS("platform:npcmx50_ohci");
MODULE_AUTHOR("Tomer Maimon <tomer.maimon@nuvoton.com>");
MODULE_LICENSE("GPL v2");
