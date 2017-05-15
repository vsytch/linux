/*
 *  Bus Glue for  npcmx50 built-in EHCI controller based on Loongson(ls1x)
 *
 *   
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */


#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>

#if 0
/* done in the npcmx50_module_init file */
#include <mach/module_init.h>
#endif

static int ehci_npcmx50_reset(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int     port = HCS_N_PORTS(ehci->hcs_params);
	int ret;

	ehci->caps = hcd->regs;

	ret = ehci_setup(hcd);
	if (ret)
		return ret;

	ehci_port_power(ehci, port, 0);

	return 0;
}

static const struct hc_driver ehci_npcmx50_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "POLEG EHCI",
	.hcd_priv_size		= sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq			= ehci_irq,
	.flags			= HCD_MEMORY | HCD_USB2 | HCD_BH,

	/*
	 * basic lifecycle operations
	 */
	.reset			= ehci_npcmx50_reset,
	.start			= ehci_run,
	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset		= ehci_endpoint_reset,

	/*
	 * scheduling support
	 */
	.get_frame_number	= ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,

	.clear_tt_buffer_complete	= ehci_clear_tt_buffer_complete,
};

static int ehci_hcd_npcmx50_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct resource *res;
	int irq;
	int ret;

	pr_debug("initializing npcmx50 ehci USB Controller\n");

#if 0
	/* done in the npcmx50_module_init file */
	npcmx50_ehci_init_reset();
#endif

	if (usb_disabled())
		return -ENODEV;

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

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}
	irq = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}

	hcd = usb_create_hcd(&ehci_npcmx50_hc_driver, &pdev->dev,
				dev_name(&pdev->dev));
	if (!hcd)
		return -ENOMEM;
	hcd->rsrc_start	= res->start;
	hcd->rsrc_len	= resource_size(res);

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
	if (ret)
		goto err_iounmap;

	return ret;

err_iounmap:
	iounmap(hcd->regs);
err_release_region:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err_put_hcd:
	usb_put_hcd(hcd);
	return ret;
}

static int ehci_hcd_npcmx50_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

	return 0;
}

static const struct of_device_id npcm750_ehci_match[] = {
	{ .compatible = "nuvoton,npcm750-ehci" },
	{},
};
MODULE_DEVICE_TABLE(of, npcm750_ehci_match);

static struct platform_driver ehci_npcmx50_driver = {
	.probe = ehci_hcd_npcmx50_probe,
	.remove = ehci_hcd_npcmx50_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.driver = {
		.name = "npcmx50-ehci",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(npcm750_ehci_match),
	},
};

MODULE_ALIAS("npcmx50-ehci");
