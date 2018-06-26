// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2014-2018 Nuvoton Technology corporation.

#include <linux/fs.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>

#define DEVICE_NAME	"npcm7xx-lpc-bpc"

#define NUM_BPC_CHANNELS 		2

/* BIOS POST Code FIFO Registers */
#define NPCM7XX_BPCFA2L_REG	0x2 //BIOS POST Code FIFO Address 2 LSB
#define NPCM7XX_BPCFA2M_REG	0x4 //BIOS POST Code FIFO Address 2 MSB
#define NPCM7XX_BPCFEN_REG	0x6 //BIOS POST Code FIFO Enable	
#define NPCM7XX_BPCFSTAT_REG	0x8 //BIOS POST Code FIFO Status
#define NPCM7XX_BPCFDATA_REG	0xA //BIOS POST Code FIFO Data 
#define NPCM7XX_BPCFMSTAT_REG	0xC //BIOS POST Code FIFO Miscellaneous Status
#define NPCM7XX_BPCFA1L_REG	0x10 //BIOS POST Code FIFO Address 1 LSB
#define NPCM7XX_BPCFA1M_REG	0x12 //BIOS POST Code FIFO Address 1 MSB

/*BIOS regiser data*/
#define FIFO_IOADDR1_ENABLE      		0x80
#define FIFO_IOADDR2_ENABLE      		0x40

/* BPC interface package and structure definition */
#define BPC_KFIFO_SIZE     			0x400

/*BPC regiser data*/
#define FIFO_READY_INT_ENABLE   		0x08
#define FIFO_DATA_VALID         		0x80
#define FIFO_OVERFLOW           		0x20
#define FIFO_ADDR_DECODE        		0x01

struct npcm7xx_bpc_channel {
	struct kfifo		fifo;
	wait_queue_head_t	wq;
	struct miscdevice	miscdev;
};

struct npcm7xx_bpc {
	void __iomem			*bpc_base;
	int				irq;
	struct npcm7xx_bpc_channel	ch[NUM_BPC_CHANNELS];
};

static struct npcm7xx_bpc_channel *npcm7xx_file_to_ch(struct file *file)
{
	return container_of(file->private_data,
			    struct npcm7xx_bpc_channel,
			    miscdev);
}

static ssize_t npcm7xx_bpc_read(struct file *file, char __user *buffer,
			       size_t count, loff_t *ppos)
{
	struct npcm7xx_bpc_channel *chan = npcm7xx_file_to_ch(file);
	unsigned int copied;
	int ret = 0;

	if (kfifo_is_empty(&chan->fifo)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(chan->wq,
				!kfifo_is_empty(&chan->fifo));
		if (ret == -ERESTARTSYS)
			return -EINTR;
	}

	ret = kfifo_to_user(&chan->fifo, buffer, count, &copied);

	return ret ? ret : copied;
}

static unsigned int npcm7xx_bpc_poll(struct file *file,
				    struct poll_table_struct *pt)
{
	struct npcm7xx_bpc_channel *chan = npcm7xx_file_to_ch(file);

	poll_wait(file, &chan->wq, pt);
	return !kfifo_is_empty(&chan->fifo) ? POLLIN : 0;
}

static const struct file_operations npcm7xx_bpc_fops = {
	.owner		= THIS_MODULE,
	.read		= npcm7xx_bpc_read,
	.poll		= npcm7xx_bpc_poll,
	.llseek		= noop_llseek,
};

static irqreturn_t npcm7xx_bpc_irq(int irq, void *arg)
{
	struct npcm7xx_bpc *lpc_bpc = arg;
	u8 fifo_st;
	u8 addr_index;
	u8 Data;
	bool ISRFlag = false;

	fifo_st = ioread8(lpc_bpc->bpc_base + NPCM7XX_BPCFSTAT_REG);
	while(FIFO_DATA_VALID & fifo_st)
	{
		addr_index = fifo_st & FIFO_ADDR_DECODE;

		/*Read data from FIFO to clear interrupt*/
		Data = ioread8(lpc_bpc->bpc_base + NPCM7XX_BPCFDATA_REG);

		if (kfifo_is_full(&lpc_bpc->ch[addr_index].fifo))
			kfifo_skip(&lpc_bpc->ch[addr_index].fifo);
		kfifo_put(&lpc_bpc->ch[addr_index].fifo, Data);
		wake_up_interruptible(&lpc_bpc->ch[addr_index].wq);

		if(fifo_st & FIFO_OVERFLOW)
			pr_info("BIOS Post Codes FIFO Overflow!!!\n");

		fifo_st = ioread8(lpc_bpc->bpc_base + NPCM7XX_BPCFSTAT_REG);
		ISRFlag = true;
	}

	if(ISRFlag)
		return IRQ_HANDLED;

	return IRQ_NONE;
}

static int npcm7xx_bpc_config_irq(struct npcm7xx_bpc *lpc_bpc,
				       struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc;

	lpc_bpc->irq = platform_get_irq(pdev, 0);
	if (!lpc_bpc->irq)
		return -ENODEV;

	rc = devm_request_irq(dev, lpc_bpc->irq,
			      npcm7xx_bpc_irq, IRQF_SHARED,
			      DEVICE_NAME, lpc_bpc);
	if (rc < 0) {
		dev_warn(dev, "Unable to request IRQ %d\n", lpc_bpc->irq);
		lpc_bpc->irq = 0;
		return rc;
	}

	return 0;
}

static int npcm7xx_enable_bpc(struct npcm7xx_bpc *lpc_bpc, struct device *dev, 
			      int channel, u16 lpc_port)
{
	int rc = 0;
	u8 Addr_en, reg_en;

	init_waitqueue_head(&lpc_bpc->ch[channel].wq);

	rc = kfifo_alloc(&lpc_bpc->ch[channel].fifo,
			 BPC_KFIFO_SIZE, GFP_KERNEL);
	if (rc)
		return rc;

	lpc_bpc->ch[channel].miscdev.minor = MISC_DYNAMIC_MINOR;
	lpc_bpc->ch[channel].miscdev.name =
		devm_kasprintf(dev, GFP_KERNEL, "%s%d", DEVICE_NAME, channel);
	lpc_bpc->ch[channel].miscdev.fops = &npcm7xx_bpc_fops;
	lpc_bpc->ch[channel].miscdev.parent = dev;
	rc = misc_register(&lpc_bpc->ch[channel].miscdev);
	if (rc)
		return rc;

	/* Enable LPC snoop channel at requested port */
	switch (channel) {
	case 0:
		Addr_en = FIFO_IOADDR1_ENABLE;
		iowrite8((u8)lpc_port & 0xFF,
			 lpc_bpc->bpc_base + NPCM7XX_BPCFA1L_REG);
		iowrite8((u8)lpc_port >> 8,
			 lpc_bpc->bpc_base + NPCM7XX_BPCFA1M_REG);
		break;
	case 1:
		Addr_en = FIFO_IOADDR2_ENABLE;
		iowrite8((u8)lpc_port & 0xFF,
			 lpc_bpc->bpc_base + NPCM7XX_BPCFA2L_REG);
		iowrite8((u8)lpc_port >> 8,
			 lpc_bpc->bpc_base + NPCM7XX_BPCFA2M_REG);
		break;
	default:
		return -EINVAL;
	}

	/* Enable FIFO Ready Interrupt and FIFO Capture of I/O address */
	reg_en = ioread8(lpc_bpc->bpc_base + NPCM7XX_BPCFEN_REG);
	iowrite8(reg_en | Addr_en | FIFO_READY_INT_ENABLE, 
		 lpc_bpc->bpc_base + NPCM7XX_BPCFEN_REG);

	return rc;
}

static void npcm7xx_disable_bpc(struct npcm7xx_bpc *lpc_bpc, int channel)
{
	u8 reg_en;

	switch (channel) {
	case 0:
		reg_en = ioread8(lpc_bpc->bpc_base + NPCM7XX_BPCFEN_REG);
		iowrite8(reg_en & ~FIFO_IOADDR1_ENABLE, 
			 lpc_bpc->bpc_base + NPCM7XX_BPCFEN_REG);
		break;
	case 1:
		reg_en = ioread8(lpc_bpc->bpc_base + NPCM7XX_BPCFEN_REG);
		iowrite8(reg_en & ~FIFO_IOADDR2_ENABLE, 
			 lpc_bpc->bpc_base + NPCM7XX_BPCFEN_REG);
		break;
	default:
		return;
	}

	kfifo_free(&lpc_bpc->ch[channel].fifo);
	misc_deregister(&lpc_bpc->ch[channel].miscdev);
}

static int npcm7xx_bpc_probe(struct platform_device *pdev)
{
	struct npcm7xx_bpc *lpc_bpc;
	struct resource *res;
	struct device *dev;
	u32 port;
	int rc;

	dev = &pdev->dev;

	lpc_bpc = devm_kzalloc(dev, sizeof(*lpc_bpc), GFP_KERNEL);
	if (!lpc_bpc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "BIOS Post Code of_address_to_resource fail\n\n\n\n\n\n");
		return -ENODEV;
	}

	dev_dbg(dev, "BIOS Post Code base resource is %pR\n", res);

	lpc_bpc->bpc_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(lpc_bpc->bpc_base)) {
		dev_err(dev, "BIOS Post Code probe failed: can't read pwm base address\n");
		return PTR_ERR(lpc_bpc->bpc_base);
	}

	dev_set_drvdata(&pdev->dev, lpc_bpc);

	rc = of_property_read_u32_index(dev->of_node, "snoop-ports", 0, &port);
	if (rc) {
		dev_err(dev, "no snoop ports configured\n");
		return -ENODEV;
	}

	rc = npcm7xx_bpc_config_irq(lpc_bpc, pdev);
	if (rc)
		return rc;

	rc = npcm7xx_enable_bpc(lpc_bpc, dev, 0, port);
	if (rc)
		return rc;

	/* Configuration of 2nd snoop channel port is optional */
	if (of_property_read_u32_index(dev->of_node, "snoop-ports",
				       1, &port) == 0) {
		rc = npcm7xx_enable_bpc(lpc_bpc, dev, 1, port);
		if (rc)
			npcm7xx_disable_bpc(lpc_bpc, 0);
	}

	pr_info("npcm7xx BIOS post code probe\n");

	return rc;
}

static int npcm7xx_bpc_remove(struct platform_device *pdev)
{
	struct npcm7xx_bpc *lpc_bpc = dev_get_drvdata(&pdev->dev);

	/* Disable both snoop channels */
	npcm7xx_disable_bpc(lpc_bpc, 0);
	npcm7xx_disable_bpc(lpc_bpc, 1);

	return 0;
}

static const struct of_device_id npcm7xx_bpc_match[] = {
	{ .compatible = "nuvoton,npcm7xx-lpc-bpc" },
	{ },
};

static struct platform_driver npcm7xx_bpc_driver = {
	.driver = {
		.name		= DEVICE_NAME,
		.of_match_table = npcm7xx_bpc_match,
	},
	.probe = npcm7xx_bpc_probe,
	.remove = npcm7xx_bpc_remove,
};

module_platform_driver(npcm7xx_bpc_driver);

MODULE_DEVICE_TABLE(of, npcm7xx_bpc_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomer Maimon <tomer.maimon@nuvoton.com>");
MODULE_DESCRIPTION("Linux driver to control NPCM7XX LPC BIOS post code snooping");
