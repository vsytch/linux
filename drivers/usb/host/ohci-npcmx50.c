/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 *
 */

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>

static int ohci_npcmx50_start(struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);
	int ret;

	ohci_hcd_init(ohci);
	ret = ohci_init(ohci);
	if (ret < 0)
	{
		pr_err("ohci_init failed\n");
		return ret;
	}
			
	ret = ohci_run(ohci);
	if (ret < 0) 
	{		
		dev_err(hcd->self.controller, "can't start %s\n", hcd->self.bus_name);
		ohci_stop(hcd);
		return ret;
	}
	
	return 0;
}

static const struct hc_driver ohci_npcmx50_driver = {
	.description =		hcd_name,
	.product_desc =		"npcmx50 OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY | HCD_BH,

	/*
	 * basic lifecycle operations
	 */
	.start =		ohci_npcmx50_start,
	.stop =			ohci_stop,
	.shutdown =		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

/*-------------------------------------------------------------------------*/

static int ohci_hcd_npcmx50_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	struct usb_hcd *hcd = NULL;
	int irq;
	int ret;

	if (usb_disabled())
	{
		return -ENODEV;
	}

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

static const struct of_device_id npcm750_ohci_match[] = {
	{ .compatible = "nuvoton,npcm750-ohci" },
	{},
};
MODULE_DEVICE_TABLE(of, npcm750_ohci_match);

static struct platform_driver ohci_hcd_npcmx50_driver = {
	.probe		= ohci_hcd_npcmx50_probe,
	.remove		= ohci_hcd_npcmx50_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.name	= "npcmx50-ohci",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(npcm750_ohci_match),
	},
};

MODULE_ALIAS("platform:npcmx50_ohci");
