/*
 * Copyright (c) 2015 Nuvoton Technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */
 
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <asm/signal.h>
#include <asm/mach/pci.h>
#include <linux/platform_device.h>

#include <linux/msi.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>
#include <asm/cputype.h>
#include <mach/irqs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_address.h>


#undef NPCMX50_PCIE_RC_INTERRUPT
#define NPCMX50_PCIE_RC_INTERRUPT pcie_rc_irq
int pcie_rc_irq;
#endif

//#define PCIE_DEBUG

#define KERN_DEBUG_PCIE  KERN_INFO

#ifdef PCIE_DEBUG
#define PCIE_DBG(fmt,args...)   printk(fmt ,##args)
#else
#define PCIE_DBG(fmt,args...)
#endif

#define DRIVER_NAME "npcmX50-pcie-rc"


/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                      PCIe Root Complex Register                                         */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
#define PCIERC_BASE  npcmx50_pcie_rc_reg_base_virtual_address

#define LINKSTAT    (PCIERC_BASE + 0x92)                  /*16bit: Link Status Register  */
#define RCCFGNUM    (PCIERC_BASE + 0x140)                 /*32bit: Configuration Number Register */

#define IMSI_ADDR					(0x190)
#define PCIERC_IMSI_ADDR			(PCIERC_BASE + IMSI_ADDR) , NPCMX50_PCIERC_ACCESS, 32
#define PCIERC_IMASK_LOCAL_ADDR		(PCIERC_BASE + 0x180), NPCMX50_PCIERC_ACCESS, 32
#define PCIERC_ISTATUS_LOCAL_ADDR	(PCIERC_BASE + 0x184), NPCMX50_PCIERC_ACCESS, 32
#define PCIERC_ISTATUS_LOCAL_MSI_BIT	(1 << 28)
#define PCIERC_ISTATUS_LOCAL_INTA_BIT	(1 << 24)
#define PCIERC_ISTATUS_MSI_ADDR		(PCIERC_BASE + 0x194), NPCMX50_PCIERC_ACCESS, 32
#define NPCMX50_CORE_SELECT			(15)

/* ----------------------------------- *
 * PCIe-to-AXI Window 0 Registers      *
 * ----------------------------------- */
#define RCPA0SAL    (PCIERC_BASE + 0x600)                 /*32bit: WINx Source Address Low Register  */
#define RCPA0SAH    (PCIERC_BASE + 0x604)                 /*32bit: WINx Source Address High Register */
#define RCPA0TAL    (PCIERC_BASE + 0x608)                 /*32bit: WINx Translation Address Low Register */
#define RCPA0TAH    (PCIERC_BASE + 0x60C)                 /*32bit: WINx Translation Address High Register */
#define RCPA0TP     (PCIERC_BASE + 0x610)                 /*32bit: WINx Translations Parameters Register  */
#define RCPA0TM		(PCIERC_BASE + 0x618)

/* ----------------------------------- *
 * AXI-to-PCIe Window 1 to 4 Registers *
 * ----------------------------------- */
#define RCAPnSAL(n) (PCIERC_BASE + 0x800 + (0x20 * (n)))  /*32bit: WINx Source Address Low Register  */
#define RCAPnSAH(n) (PCIERC_BASE + 0x804 + (0x20 * (n)))  /*32bit: WINx Source Address High Register */
#define RCAPnTAL(n) (PCIERC_BASE + 0x808 + (0x20 * (n)))  /*32bit: WINx Translation Address Low Register */
#define RCAPnTAH(n) (PCIERC_BASE + 0x80C + (0x20 * (n)))  /*32bit: WINx Translation Address High Register */
#define RCAPnTP(n)  (PCIERC_BASE + 0x810 + (0x20 * (n)))  /*32bit: WINx Translations Parameters Register  */


/* ------------------------- *
 * RCAPnSAL register fields  *
 * ------------------------- */
#define ATR_SIZE_4K     11
#define ATR_SIZE_8K     12
#define ATR_SIZE_16K    13
#define ATR_SIZE_32K    14
#define ATR_SIZE_64K    15
#define ATR_SIZE_128K   16
#define ATR_SIZE_256K   17
#define ATR_SIZE_512K   18
#define ATR_SIZE_1M     19
#define ATR_SIZE_2M     20
#define ATR_SIZE_4M     21
#define ATR_SIZE_8M     22
#define ATR_SIZE_16M    23
#define ATR_SIZE_32M    24
#define ATR_SIZE_64M    25
#define ATR_SIZE_128M   26
#define ATR_SIZE_256M   27
#define ATR_SIZE_512M   28
#define ATR_SIZE_1G     29
#define ATR_SIZE_2G     30

/* ------------------------- *
 * RCAPnTP register fields   *
 * ------------------------- */
#define TRSF_PARAM_MEMORY    (0L << 16)
#define TRSF_PARAM_CONFIG    (1L << 16)
#define TRSF_PARAM_IO        (2L << 16)
#define TRSL_ID_PCIE_TX_RX   0 
#define TRSL_ID_PCIE_CONFIG  1 


#define PLDA_XPRESS_RICH_MEMORY_WINDOW 	0
#define PLDA_XPRESS_RICH_CONFIG_WINDOW 	1
#define PLDA_XPRESS_RICH_IO_WINDOW 		2
//#define PLDA_XPRESS_RICH_IO_WINDOW 		1
#define PLDA_XPRESS_RICH_MESSAGE_WINDOW	4

#define PLDA_XPRESS_RICH_TARGET_PCI_TX_RX	0
#define PLDA_XPRESS_RICH_TARGET_PCI_CONFIG	1 // also for io
#define PLDA_XPRESS_RICH_TARGET_AXI_MASTER	4

#define PCI_RC_ATTR_WIN_EN_POS			0
#define PCI_RC_ATTR_WIN_SIZE_POS		1
#define PCI_RC_ATTR_AP_ADDR_L_POS		12

#define PCI_RC_ATTR_TRSF_PARAM_POS		16
#define PCI_RC_ATTR_TRSL_ID_POS			0

#define IRQ_REQUEST
#define PCI_BAR_REG 0x10
#define PCI_CMD_STATUS_REG  0x4

#define  PCI_COMMAND_IO		0x1	/* Enable response in I/O space */
#define  PCI_COMMAND_MEMORY	0x2	/* Enable response in Memory space */
#define  PCI_COMMAND_MASTER	0x4	/* Enable bus mastering */

/* ---------------------------------- *
 * Windows definitions                *
 * !! important note !! - Window base *
 *  must be align to window size.     *
 * ---------------------------------- */
#define CONFIG_WIN_BASE    (0xE8000000 + 0)
#define IO_WIN_BASE        (0xE8000000 + SZ_1M)
#define MEM_WIN_BASE       (0xE8000000 + SZ_32M)  

#define CONFIG_WIN_SIZE    SZ_4K
#define IO_WIN_SIZE        SZ_1M
#define MEM_WIN_SIZE       SZ_32M

#define CONFIG_WIN_V_BASE  npcmx50_pcie_rc_config_area_virtual_address

/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                      Other rgister for initialzing root complex                         */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/

//#define DBG_DELAY

#define GPIO6_BASE_ADDR      0xF0016000
#define GPIO0_BASE_ADDR      0xF0010000

#define GCR_BASE  npcmx50_gcr_reg_base_virtual_address
#define CLK_BASE  npcmx50_clk_reg_base_virtual_address
#define GPIO_PCI_BASE  npcmx50_gpio_pci_reg_base_virtual_address


#define MFSEL1      (GCR_BASE + 0x0C)

#define MFSEL3      (GCR_BASE + 0x64)
#define MFSEL3_PCIEPUSE_BIT (1 << 17)

#define INTCR3      (GCR_BASE + 0x9c)
#define INTCR3_RCCORER_BIT (1 << 22)

#define I2CSEGSEL   (GCR_BASE + 0xE0)

#define IPSRST3		(CLK_BASE + 0x34)
#define IPSRST3_PCIERC_BIT (1 << 15)

#define AHBCKFI     (CLK_BASE + 0x64)

#define GPIO_PCI_DOS  (GPIO_PCI_BASE + 0x68)
#define GPIO_PCI_DOC  (GPIO_PCI_BASE + 0x6C)
#define GPIO_PCI_OES  (GPIO_PCI_BASE + 0x70)

#define LINK_UP_FIELD (0x3F << 20)               // We have only one link in Negotiated Link Width       
#define LINK_RETRAIN_BIT (1 << 27)

#define RC_TO_EP_DELAY    15

#ifdef POLEG_DRB_HW
#define GPIO_PCI_BASE_ADDR   GPIO6_BASE_ADDR
#define GPIO_PCI_RESET_RC    (1 << (196%32))                      // GPIO196
#define GPIO_PCI_RESET_EP    (1 << (197%32))                      // GPIO197
#define GPIO_PCI_MUX_REG I2CSEGSEL
#define GPIO_PCI_MUX_BITS (1<<22 | 1<<1)                          // bits 22 & 1
#else
#define GPIO_PCI_BASE_ADDR   GPIO0_BASE_ADDR
#define GPIO_PCI_RESET_RC    (1 << 29)                            // GPIO29
#define GPIO_PCI_RESET_EP    (1 << 29)                            // GPIO29
#define GPIO_PCI_MUX_REG MFSEL1
#define GPIO_PCI_MUX_BITS (1<<1)                                  // bit  1
#endif

/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                      Types & Variable                                                   */  
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/

void __iomem *npcmx50_pcie_rc_reg_base_virtual_address;
void __iomem *npcmx50_pcie_rc_config_area_virtual_address;
void __iomem *npcmx50_gcr_reg_base_virtual_address;
void __iomem *npcmx50_clk_reg_base_virtual_address;
void __iomem *npcmx50_gpio_pci_reg_base_virtual_address;

static struct resource pcie_rc_io;

#ifdef CONFIG_PCI_MSI 

/* Number of MSI IRQs */
#define NPCM750_NUM_MSI_IRQS		32

#define NPCM750_MAX_NUM_RESOURCES		1

struct npcm750_pcie_port {
	void __iomem *reg_base;
	u32 irq;
	unsigned long msi_pages;
	u8 root_busno;
	struct device *dev;
	struct irq_domain *irq_domain;
	struct resource bus_range;
	struct list_head resources;
};

static DECLARE_BITMAP(msi_irq_in_use, NPCM750_NUM_MSI_IRQS);

static inline struct npcm750_pcie_port *sys_to_pcie(struct pci_sys_data *sys)
{
	return sys->private_data;
}

static uint32_t bar0;

/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                     extern functions                                                    */  
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/

extern void npcmx50_spin_lock_irqsave(unsigned long *flags);
extern void npcmx50_spin_unlock_irqrestore(unsigned long flags);

/**
 * npcm750_pcie_destroy_msi - Free MSI number
 * @irq: IRQ to be freed
 */
static void npcm750_pcie_destroy_msi(unsigned int irq)
{
	struct irq_desc *desc;
	struct msi_desc *msi;
	struct npcm750_pcie_port *port;

	desc = irq_to_desc(irq);
	if (desc == NULL)
	{
		printk("Trying to free unknown desc according irq#%d\n", irq);
		return;
	}
		
	msi = irq_desc_get_msi_desc(desc);
	port = sys_to_pcie(msi->dev->bus->sysdata);

	if (!test_bit(irq, msi_irq_in_use))
		dev_err(port->dev, "Trying to free unused MSI#%d\n", irq);
	else
		clear_bit(irq, msi_irq_in_use);
}

/**
 * npcm750_pcie_assign_msi - Allocate MSI number
 * @port: PCIe port structure
 *
 * Return: A valid IRQ on success and error value on failure.
 */
static int npcm750_pcie_assign_msi(struct npcm750_pcie_port *port)
{
	int pos;

	pos = find_first_zero_bit(msi_irq_in_use, NPCM750_NUM_MSI_IRQS);
	if (pos < NPCM750_NUM_MSI_IRQS)
		set_bit(pos, msi_irq_in_use);
	else
		return -ENOSPC;

	return pos;
}

static irqreturn_t npcmX50_msi_handler(int irq, void* dev_id)
{
	struct npcm750_pcie_port *port = (struct npcm750_pcie_port *)dev_id;
	unsigned int dt_irq;
	unsigned int index;
	unsigned long status;
	
	uint32_t global_pcie_status = REG_READ( PCIERC_ISTATUS_LOCAL_ADDR );
//	printk( "<1>  pcie rc interrupt\n");
	if( global_pcie_status & PCIERC_ISTATUS_LOCAL_MSI_BIT )
	{

		status = REG_READ( PCIERC_ISTATUS_MSI_ADDR );
		//printk( "<1>  pcie rc msi interrupt , PCIERC_ISTATUS_MSI_ADDR = 0x%x\n",status);
		if (!status)
		{
			return IRQ_HANDLED;
		}

		do {
			index = find_first_bit(&status, 32);
			REG_WRITE(PCIERC_ISTATUS_MSI_ADDR  , 1 << index  );/* write back to clear bit */
	
			dt_irq = irq_find_mapping(port->irq_domain, index);
			if (test_bit(index, msi_irq_in_use))
			{
				generic_handle_irq(dt_irq);
			}
			else
				dev_info(port->dev, "unhandled MSI\n");
			
			//printk( "<1>  pcie rc msi interrupt , PCIERC_ISTATUS_MSI_ADDR = 0x%x irq %d dt_irq %d index=%d\n",status,irq,dt_irq,index);
			
			status = REG_READ( PCIERC_ISTATUS_MSI_ADDR );
			
		} while (status);
	}
	REG_WRITE(PCIERC_ISTATUS_LOCAL_ADDR  , global_pcie_status );/* write back to clear   */

	return IRQ_HANDLED;
}


/* function : set_translation_window
 *  type - 0-memory
 *  size - should be 2^n bytes
 */
static int set_translation_window(void __iomem * config_address_base,dma_addr_t *source_addr,
		u32 size,dma_addr_t *dest_addr,u8 win_type,u8 target)
{
	u8 win_size=11 ;
	u32 val;

	if (size < 4096)
	{
		printk("%s : ERROR : window size should be greater then 4KB\n " , __FUNCTION__);
	}

	size=(size>>(win_size+2));
	while(size)
	{
		size= (size >> 1);
		win_size++;
	}

#ifdef __LP64__
	writel( ((uint64_t)source_addr & 0xffffffff) +
			(win_size << PCI_RC_ATTR_WIN_SIZE_POS)+( 1<< PCI_RC_ATTR_WIN_EN_POS) , config_address_base);

	writel( ((uint64_t)source_addr >> 32 ) & 0xffffffff ,config_address_base + 0x4 );
	writel(((uint64_t)dest_addr & 0xffffffff) , config_address_base + 0x8);
	writel( ((uint64_t)dest_addr >> 32 ) & 0xffffffff , config_address_base + 0xc );
#else
	writel( ((uint32_t)source_addr  ) +
			(win_size << PCI_RC_ATTR_WIN_SIZE_POS)+( 1<< PCI_RC_ATTR_WIN_EN_POS) , config_address_base);


	writel( 0 ,config_address_base + 0x4 );
	writel( (uint32_t)dest_addr  , config_address_base + 0x8);
	writel( 0, config_address_base + 0xc );
#endif

	val=(win_type << PCI_RC_ATTR_TRSF_PARAM_POS) + (target << PCI_RC_ATTR_TRSL_ID_POS);
	writel( val , config_address_base + 0x10);
	return 0;
}

/*static void npcmX50_msi_nop(struct irq_data *d)
{
	return;
}*/

static struct irq_chip npcmX50_msi_chip = {
	.name = "NPCM750 PCI-MSI",
//	.irq_ack = npcmX50_msi_nop,
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

int npcm750_pcie_msi_setup_irq(struct msi_controller *chip,
				     struct pci_dev *pdev,
				     struct msi_desc *desc)
{
	struct npcm750_pcie_port *port = sys_to_pcie(pdev->bus->sysdata);
	unsigned int irq,id;
	int hwirq;
	struct msi_msg msg;

	hwirq = npcm750_pcie_assign_msi(port);
	if (hwirq < 0)
		return hwirq;
	
	irq = irq_create_mapping(port->irq_domain, hwirq);
	if (!irq)
		return -EINVAL;

	irq_set_msi_desc(irq, desc);
	
	pr_info("npcm750_pcie_msi_setup_irq hwirq %d irq %d \n",(int)hwirq,irq);

	msg.address_hi = 0x0;
	msg.address_lo = (bar0 + IMSI_ADDR) & 0xfffffff0;

	id = read_cpuid_id();
	msg.data = (id << NPCMX50_CORE_SELECT) | hwirq;

	pci_write_msi_msg(irq, &msg);

	irq_set_chip_and_handler(irq, &npcmX50_msi_chip, handle_simple_irq);
	set_irq_flags(irq,  IRQF_VALID | IRQF_PROBE  );

	return 0;
}

void npcm750_msi_teardown_irq(struct msi_controller *chip,
				    unsigned int irq)
{
	pr_debug("%s : irq = %d \n " , __FUNCTION__ , irq);
	npcm750_pcie_destroy_msi(irq);
}

/* MSI Chip Descriptor */
static struct msi_controller npcm750_pcie_msi_chip = {
	.setup_irq = npcm750_pcie_msi_setup_irq,
	.teardown_irq = npcm750_msi_teardown_irq,
};

/**
 * npcm750_pcie_msi_map - Set the handler for the MSI and mark IRQ as valid
 * @domain: IRQ domain
 * @irq: Virtual IRQ number
 * @hwirq: HW interrupt number
 *
 * Return: Always returns 0.
 */
static int npcm750_pcie_msi_map(struct irq_domain *domain, unsigned int irq,
			       irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &npcmX50_msi_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);
	set_irq_flags(irq, IRQF_VALID);

	return 0;
}

/* IRQ Domain operations */
static const struct irq_domain_ops msi_domain_ops = {
	.map = npcm750_pcie_msi_map,
};

/**
 * npcm750_pcie_free_irq_domain - Free IRQ domain
 * @port: PCIe port information
 */
static void npcm750_pcie_free_irq_domain(struct npcm750_pcie_port *port)
{
	int i;
	u32 irq, num_irqs;

	/* Free IRQ Domain */
	if (IS_ENABLED(CONFIG_PCI_MSI)) {

		num_irqs = NPCM750_NUM_MSI_IRQS;
	} 

	for (i = 0; i < num_irqs; i++) {
		irq = irq_find_mapping(port->irq_domain, i);
		if (irq > 0)
			irq_dispose_mapping(irq);
	}

	irq_domain_remove(port->irq_domain);
}

/**
 * npcm750_pcie_init_irq_domain - Initialize IRQ domain
 * @port: PCIe port information
 *
 * Return: '0' on success and error value on failure
 */
static int npcm750_pcie_init_irq_domain(struct npcm750_pcie_port *port)
{
	struct device *dev = port->dev;
	struct device_node *node = dev->of_node;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		port->irq_domain = irq_domain_add_linear(node,
							 NPCM750_NUM_MSI_IRQS,
							 &msi_domain_ops,
							 &npcm750_pcie_msi_chip);
		if (!port->irq_domain) {
			dev_err(dev, "Failed to get a MSI IRQ domain\n");
			return PTR_ERR(port->irq_domain);
		}
	}

	return 0;
}

static int npcmX50_msi_init(struct npcm750_pcie_port *port)
{
	struct device *dev = port->dev;
	int ret = 0;
	struct device_node *node = dev->of_node;
	int err;

	REG_WRITE(PCIERC_IMASK_LOCAL_ADDR  , PCIERC_ISTATUS_LOCAL_MSI_BIT );/* enable interrupts  */

	
	set_translation_window((void __iomem *)RCPA0SAL ,  0, SZ_512M ,
	                     0 , PLDA_XPRESS_RICH_MEMORY_WINDOW , PLDA_XPRESS_RICH_TARGET_AXI_MASTER) ;

	pcie_rc_irq = irq_of_parse_and_map(node, 0);
	if (!pcie_rc_irq) {
		printk(KERN_ERR "%s - failed to map irq\n", __FUNCTION__);
		ret= -1;
		goto done;
	}

	port->irq = pcie_rc_irq;
	err = devm_request_irq(dev, port->irq, npcmX50_msi_handler,
				   IRQF_SHARED, "npcm750-pcie", port);
	if (err) {
		dev_err(dev, "unable to request irq %d\n", port->irq);
		return err;
	}
	
	err = npcm750_pcie_init_irq_domain(port);
	if (err) {
		dev_err(dev, "Failed creating IRQ Domain\n");
		return err;
	}

done:
	return ret;
}

static void npcmX50_msi_update_rc_bar0(uint32_t bar )
{
	uint32_t val;
	bar0 = bar;

	__raw_writel(0x1F0000 , (void __iomem *)  RCCFGNUM);
	val = __raw_readl(CONFIG_WIN_V_BASE + PCI_CMD_STATUS_REG);
	val |= (PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);
	__raw_writel(val , CONFIG_WIN_V_BASE + PCI_CMD_STATUS_REG);
	pr_debug( "%s , bar = 0x%x   \n",__FUNCTION__,bar);

}


#endif // CONFIG_PCI_MSI


/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                      Functions                                                            */  
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/

static void npcmx50_gpio_reset(int reset_clear)
{
    writel(readl(GPIO_PCI_MUX_REG) & (~GPIO_PCI_MUX_BITS), GPIO_PCI_MUX_REG);   /* Clear GPIO PCI mux bits */
    if (reset_clear)
    {  
        writel(GPIO_PCI_RESET_RC, GPIO_PCI_DOS);      /* Set GPIO to release PCIe RC reset */
        writel(GPIO_PCI_RESET_RC, GPIO_PCI_OES);      /* Output Enable the GPIO */
        
        // There is a timeout after which the LTSSM is moving from DETECT.QUIET to DETECT.ACTIVE
        msleep(RC_TO_EP_DELAY);
        
        writel(GPIO_PCI_RESET_EP, GPIO_PCI_DOS);      /* Set GPIO to release PCI EP reset */
        writel(GPIO_PCI_RESET_EP, GPIO_PCI_OES);      /* Output Enable the GPIO */
    }
    else // reset disable_TX
    {
        writel(GPIO_PCI_RESET_EP, GPIO_PCI_DOC);      /* clear GPIO to put PCIe EP in reset */
        writel(GPIO_PCI_RESET_EP, GPIO_PCI_OES);      /* Output Enable the GPIO */
        
        writel(GPIO_PCI_RESET_RC, GPIO_PCI_DOC);      /* clear GPIO to put PCIe RC in reset */
        writel(GPIO_PCI_RESET_RC, GPIO_PCI_OES);      /* Output Enable the GPIO */
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcmx50_pcie_rc_device_connected.                                                      */
/*                                                                                                         */
/* Description:                                                                                            */
/*                  Check is there is a device on the other side of the bridge.                            */
/* Returns:         1 - device exist (link connection)                                                     */
/*                  0 - no device    (no connection)                                                       */
/*---------------------------------------------------------------------------------------------------------*/
static int npcmx50_pcie_rc_device_connected(void)
{
    u32 val;

    /* ----------------------------------------------- *
     * Check the Link status register on the bridge 
     * configuration space at:
     * Bus 0, Device 0, function 0 offset 0x92 bit 4
     * ----------------------------------------------- */   
    __raw_writel(0x1F0000 , RCCFGNUM);
    val = __raw_readl(CONFIG_WIN_V_BASE + 0x90);
    return  ((val & LINK_UP_FIELD) >> 20); 
    
}
/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcmx50_pcie_rc_config_read                                                                */
/*                                                                                                         */
/* Description:                                                                                            */
/*                  ?????????????????????????????????????????????????????????????????                      */
/*---------------------------------------------------------------------------------------------------------*/
static int npcmx50_pcie_rc_config_read(struct pci_bus *bus, 
                                     unsigned int devfn, 
                                     int where, 
                                     int size, 
                                     u32 *value)
{

    /* ----------------------------------------------- *
     * Bypass HW issue.
     * There is currently no device on the other side 
     * of the bridge. Trying to access any register on
     * bus > 0 will stuck the cpu (HW issue). 
     * Ignore this request and return 0xFFFFFFFF
     * ----------------------------------------------- */
    if ((bus->number > 0) && (npcmx50_pcie_rc_device_connected() == 0))
    {
         printk(KERN_INFO "npcmx50_pcie_rc_config_read - NO LINK\n");        
         *value = 0xFFFFFFFF;
         return PCIBIOS_SUCCESSFUL;
    }
    
    /* ----------------------------------------------- * 
     * Bypass HW issue. 
     * The HW returns 0x6000001 on a read from:
     * Bus 0, Device 0, function 0 offset 8. 
     * but it should be 0x6040001 which means that 
     * this is a PCI-to-PCI bridge 
     * ----------------------------------------------- */
    if ((bus->number == 0) && (devfn == 0) && ((where & 0xFFFFFFFC) == 8))
    {
        *value = 0x6040001;
    }
    else
    {
        __raw_writel(0x1F0000 | (((unsigned int)(bus->number)) << 8) | (devfn & 0xFF), RCCFGNUM);
        *value = __raw_readl(CONFIG_WIN_V_BASE + (where & 0xFFFFFFFC));
    }
     
    /* ----------------------------------------------- * 
     * Return the correct value according to the size 
     * and offset.
     * ----------------------------------------------- */
    if (size == 1) {
        *value = (*value >> (8 * (where & 3))) & 0xff;
    }
    else if (size == 2) {
        *value = (*value >> (8 * (where & 3))) & 0xffff;
    }
    PCIE_DBG(KERN_DEBUG_PCIE "npcmx50_pcie_rc_config_read (b:%d,df:%d,w:%x, v:%x)\n",(int)(bus->number),devfn,where, *value);        
    
    return PCIBIOS_SUCCESSFUL;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcmx50_pcie_rc_config_write                                                               */
/*                                                                                                         */
/* Description:                                                                                            */
/*                  ?????????????????????????????????????????????????????????????????                      */
/*---------------------------------------------------------------------------------------------------------*/
static int npcmx50_pcie_rc_config_write(struct pci_bus *bus, 
                                      unsigned int devfn,
                                      int where,
                                      int size,
                                      u32 value)
{
    u32 org_val, new_val;    
    int ret = PCIBIOS_SUCCESSFUL;

    PCIE_DBG(KERN_DEBUG_PCIE "npcmx50_pcie_rc_config_write (b:%d,df:%d,w:%x,s:%d v:%x) - Start.\n", (int)(bus->number),devfn,where, size, value);

    /* ----------------------------------------------- *
     * Bypass HW issue.
     * There is currently no device on the other side 
     * of the bridge. Trying to access any register on
     * bus > 0 will stuck the cpu (HW issue). 
     * Ignore this write request
     * ----------------------------------------------- */
    if ((bus->number > 0) && (npcmx50_pcie_rc_device_connected() == 0))
    {
        printk(KERN_DEBUG_PCIE "npcmx50_pcie_rc_config_write - NO LINK\n");        
        return ret;
    }

#ifdef CONFIG_PCI_MSI
    if((bus->number == 0) && (0 == devfn) && (0x10 == where))
    {
    	npcmX50_msi_update_rc_bar0(value);
    	//printk(KERN_DEBUG_PCIE "change rc BAR0 to 0x%x\n",value);
    }
#endif    
    
    /* ----------------------------------------------- * 
     * Select the bus device and function 
     * ----------------------------------------------- */
    __raw_writel(0x1F0000 | (((unsigned int)(bus->number)) << 8) | (devfn & 0xFF), RCCFGNUM);
    
    /* ----------------------------------------------- * 
     * Write the data. 
     * ----------------------------------------------- */
    if (size == 4) 
    {
        __raw_writel(value, CONFIG_WIN_V_BASE + (where & 0xFFFFFFFC));
    }
    else if (size == 2)
    {
        org_val = __raw_readl(CONFIG_WIN_V_BASE + (where & 0xFFFFFFFC));
        value   = (value & 0x0000FFFF) << ((where & 0x3) * 8);
        new_val = (org_val & ~(0x0000FFFF << ((where & 0x3) * 8))) | value;
        __raw_writel(new_val, CONFIG_WIN_V_BASE + (where & 0xFFFFFFFC));
        //printk(KERN_DEBUG_PCIE "config_write 2 n:%x, o:%x\n",new_val, org_val);        
    }
    else if (size == 1)
    {
        org_val = __raw_readl(CONFIG_WIN_V_BASE + (where & 0xFFFFFFFC));
        value   = (value & 0x000000FF) << ((where & 0x3) * 8);
        new_val = (org_val & ~(0x000000FF << ((where & 0x3) * 8))) | value;
        __raw_writel(new_val, CONFIG_WIN_V_BASE + (where & 0xFFFFFFFC));
        //printk(KERN_DEBUG_PCIE "config_write 1 n:%x, o:%x\n",new_val, org_val);        
    }
    else 
    {
        ret = PCIBIOS_BAD_REGISTER_NUMBER;
    }

    return ret;
}

static struct pci_ops npcmx50_pcie_rc_ops = {
    .read	= npcmx50_pcie_rc_config_read,
    .write	= npcmx50_pcie_rc_config_write,
};



static void wait_for_L0_state(u32 timeout)
{
    __raw_writel(0x1F0000 , RCCFGNUM);
    /* Wait for link up and link retrain done as L0 State */         
    while ( ((LINK_UP_FIELD & __raw_readl(CONFIG_WIN_V_BASE + 0x90)) == 0) || ((LINK_RETRAIN_BIT & __raw_readl(CONFIG_WIN_V_BASE + 0x90)) != 0) )
    {
		if (timeout == 0) 
        {
		    printk(KERN_ERR DRIVER_NAME "Not in L0 State !!!- Link Control Register and Status (Offset 10h) = 0x%x \n",__raw_readl(CONFIG_WIN_V_BASE + 0x90));     
			break;
		}
		timeout--;
		udelay(100);
    }
}

static struct pci_bus* __init npcmx50_pcie_rc_scan_bus(int nr, struct pci_sys_data *sys)
{
    struct pci_bus* res;
    u32 val = 0;

    // wait 100 ms for EP to be ready
    msleep(100);

    PCIE_DBG(KERN_DEBUG_PCIE "npcmx50_pcie_rc_scan_bus - Start.\n");

    __raw_writel(0x1F0000 , RCCFGNUM);

    PCIE_DBG(KERN_DEBUG_PCIE "Link Capabilities Register (Offset 0Ch) = 0x%x \n", __raw_readl(CONFIG_WIN_V_BASE + 0x8C));        
    PCIE_DBG(KERN_DEBUG_PCIE "Link Control and Status Register (Offset 10h) = 0x%x \n", __raw_readl(CONFIG_WIN_V_BASE + 0x90));        
         
    wait_for_L0_state(400);       // timeout ~400*100usec 
#ifdef DBG_DELAY
	{		
	    int i;
		printk("before retrain:\n");
		for(i=0; i<30; i++)
		{
			printk(" %d", i);
			msleep(1000);
		}
	}
#endif
    /* Set link Retraining bit */
    val = __raw_readw(CONFIG_WIN_V_BASE + 0x90);
    val |= 0x20;
    __raw_writew(val, CONFIG_WIN_V_BASE + 0x90);

    wait_for_L0_state(400);       // timeout ~400*100usec

    PCIE_DBG(KERN_DEBUG_PCIE "Link Capabilities Register (Offset 0Ch) = 0x%x \n", __raw_readl(CONFIG_WIN_V_BASE + 0x8C));        
    PCIE_DBG(KERN_DEBUG_PCIE "Link Control and Status Register(Offset 10h) = 0x%x \n", __raw_readl(CONFIG_WIN_V_BASE + 0x90));        
    
    res = pci_scan_root_bus(NULL, sys->busnr, &npcmx50_pcie_rc_ops, sys, &sys->resources);
    PCIE_DBG(KERN_DEBUG_PCIE "npcmx50_pcie_rc_scan_bus - End.\n");
    return res;
}

static void npcmx50_pcie_rc_init_config_window(void)
{
    /* --------------- *
     * Config window
     * --------------- */
     
    writel(0, RCAPnTAL(1));    /* Translation address of 0,  */
    writel(0, RCAPnTAH(1));
    
    writel(TRSF_PARAM_CONFIG | TRSL_ID_PCIE_CONFIG, RCAPnTP(1));   /* TRSF_PARAM=1, TRSL_ID=1 */
    
    writel(0, RCAPnSAH(1));                   /* Src addr high to 0 */
    writel((CONFIG_WIN_BASE & 0xFFFFF000) |   /* Src addr low within the allocated 128 MB */
           (ATR_SIZE_4K << 1) |               /* Window size of 4 KB */
           0x01,                              /* Enable window */
           RCAPnSAL(1));
    
}


static void npcmx50_initialize_as_root_complex(void)
{ 
    unsigned long flags = 0;

    PCIE_DBG(KERN_DEBUG_PCIE "npcmx50_initialize_as_root_complex - Start\n");  

    npcmx50_spin_lock_irqsave(&flags);

    /* ----------------------------------- *
     * defining AHB clock frequency for RC
     * ----------------------------------- */
 /*
    unsigned int val;
    val = readl(AHBCKFI);       
    val &= (~ (0xff));
    val |= (0xfa << 0);
    writel(val, AHBCKFI);
*/
	/* ----------------------- *
     * put RC core to reset	(write 0 to enter reset and 1 to release)
     * ------------------------*/
    writel(readl(INTCR3) & (~INTCR3_RCCORER_BIT), INTCR3);
	
	/* ----------------------- *
     * put RC to reset (write 1 to enter reset and 0 to enable module)
     * ------------------------*/
    writel(readl(IPSRST3) | IPSRST3_PCIERC_BIT, IPSRST3);
	
    /* ----------------------- *
     * enable rootport access
     * ------------------------*/
    writel(readl(MFSEL3) | MFSEL3_PCIEPUSE_BIT, MFSEL3);
    
	/* ----------------------- *
     * release RC from reset
     * ------------------------*/
    writel(readl(INTCR3) | INTCR3_RCCORER_BIT, INTCR3);
	
	/* ----------------------- *
     * enable RC 	
     * ------------------------*/
    writel(readl(IPSRST3) & (~IPSRST3_PCIERC_BIT), IPSRST3);

    npcmx50_spin_unlock_irqrestore(flags);

    PCIE_DBG(KERN_DEBUG_PCIE "npcmx50_initialize_as_root_complex - End\n");    

}

static int __init npcmx50_pcie_rc_setup(int nr, struct pci_sys_data *sys)
{
	struct npcm750_pcie_port *port = sys_to_pcie(sys);
	struct device *dev = port->dev;
	struct device_node *node = dev->of_node;
	struct of_pci_range range;
	struct of_pci_range_parser parser;
	struct resource *mem;
	int err = 1, mem_resno = 0;

    PCIE_DBG(KERN_DEBUG_PCIE "npcmx50_pcie_rc_setup - Start (nr = %d).\n",nr);    

    if (nr > 0)
        return 0;

#ifdef DBG_DELAY
	{
	    int i;		
		printk("before RC enable:\n");
		for(i=0; i<30; i++)
		{
			printk(" %d", i);
			msleep(1000);
		}
	}
#endif

    /* 
       Set GPIO for Endpoint reset release on DRB/SVB, Done before everything.
    */
    npcmx50_gpio_reset(0);

    npcmx50_initialize_as_root_complex(); /* This function might not be needed
                                           * in case this system initialzations 
                                           * will be done in a different place */
#ifdef DBG_DELAY		
	{
	    int i;	    
		printk("After RC enable before window config:\n");
		for(i=0; i<30; i++)
		{
			printk(" %d", i);
		
		}
	}
#endif

	npcmx50_pcie_rc_init_config_window();

    pcie_rc_io.name   = "PCI IO space";
    pcie_rc_io.start  = IO_WIN_BASE;
    pcie_rc_io.end    = pcie_rc_io.start + (IO_WIN_SIZE - 1);
    pcie_rc_io.flags  = IORESOURCE_IO;

	request_resource(&ioport_resource, &pcie_rc_io);
	pci_add_resource_offset(&sys->resources, &pcie_rc_io, sys->io_offset);
	
	/* --------------------- *
	 * Config I/O window
	 * --------------------- */
		writel(pcie_rc_io.start, RCAPnTAL(3));	 /* Translation address */
		writel(0, RCAPnTAH(3));
		
		writel(TRSF_PARAM_IO| TRSL_ID_PCIE_CONFIG, RCAPnTP(3));   /* TRSF_PARAM=2, TRSL_ID=1 */
		
		writel(0, RCAPnSAH(3)); 							  /* Src addr high to 0 */
		writel((pcie_rc_io.start & 0xFFFFF000) |	 /* Src addr low within the allocated 128 MB */
			   (ATR_SIZE_1M << 1) | 			   /* Window size 1MB*/
			   0x01,							   /* Enable window */
			   RCAPnSAL(3));

	if (of_pci_range_parser_init(&parser, node)) {
		dev_err(dev, "missing \"ranges\" property\n");
		return -EINVAL;
	}
	
	/* Parse the ranges and add the resources found to the list */
	for_each_of_pci_range(&parser, &range) {

		if (mem_resno >= NPCM750_MAX_NUM_RESOURCES) {
			dev_err(dev, "Maximum memory resources exceeded\n");
			return -EINVAL;
		}

		mem = devm_kmalloc(dev, sizeof(*mem), GFP_KERNEL);
		if (!mem) {
			err = -ENOMEM;
			//goto free_resources;
		}

		of_pci_range_to_resource(&range, node, mem);

		switch (mem->flags & IORESOURCE_TYPE_BITS) {
			case IORESOURCE_MEM:
				request_resource(&iomem_resource, mem);
				pci_add_resource_offset(&sys->resources, mem, sys->mem_offset);
				
				/* --------------------- *
				 * Config memory window
				 * --------------------- */
					writel(mem->start, RCAPnTAL(2));	  /* Translation address */
					writel(0, RCAPnTAH(2));
					
					writel(TRSF_PARAM_MEMORY| TRSL_ID_PCIE_TX_RX, RCAPnTP(2));	 /* TRSF_PARAM=0, TRSL_ID=0 */
					
					writel(0, RCAPnSAH(2)); 							  /* Src addr high to 0 */
					writel((mem->start & 0xFFFFF000) |	/* Src addr low within the allocated 128 MB */
						   (ATR_SIZE_32M << 1) |				/* Window size 32MB */
						   0x01,								/* Enable window */
						   RCAPnSAL(2));
					
				mem_resno++;
				break;
		default:
			err = -EINVAL;
			break;
		}

		if (err < 0) {
			dev_warn(dev, "Invalid resource found %pR\n", mem);
			continue;
		}

	}

    /* 
       Set Endpoint reset release on DRB/SVB, Done right after PCI_RC enable on GCR.
    */
    npcmx50_gpio_reset(1);

#ifdef PCIE_DEBUG
    __raw_writel(0x1F0000 , RCCFGNUM);
#endif
 
#ifdef CONFIG_PCI_MSI
   	npcmX50_msi_init(port);
#endif

    PCIE_DBG(KERN_DEBUG_PCIE "Link Capabilities Register (Offset 0Ch) = 0x%x \n", __raw_readl(CONFIG_WIN_V_BASE + 0x8C));        
    PCIE_DBG(KERN_DEBUG_PCIE "Link Control and Status Register(Offset 10h) = 0x%x \n", __raw_readl(CONFIG_WIN_V_BASE + 0x90));        

    PCIE_DBG(KERN_DEBUG_PCIE "npcmx50_pcie_rc_setup - End.\n");    

    return err;
}

/*static struct hw_pci npcmx50_pcie_rc __initdata = {
    .nr_controllers   = 1,
    .setup            = npcmx50_pcie_rc_setup,
    .scan             = npcmx50_pcie_rc_scan_bus,
	.map_irq		  = of_irq_parse_and_map_pci,
    
};*/

static int npcm750_pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
 {
 		 struct npcm750_pcie_port *port = sys_to_pcie(dev->bus->sysdata);
         int irq;
 
         irq = of_irq_parse_and_map_pci(dev, slot, pin);
         if (!irq)
                 irq = port->irq;
 
         return irq;
 }

static int npcmx50_pcie_rc_probe(struct platform_device *pdev)
{    
	struct npcm750_pcie_port *port;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct device_node *np = dev->of_node;
    int error = 0;
	struct hw_pci hw;
	const char *type;
	
    PCIE_DBG(KERN_DEBUG_PCIE "npcmx50_pcie_rc_probe - Start.\n");    

	port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;
	
	port->dev = dev;
	
	type = of_get_property(np, "device_type", NULL);
	 if (!type || strcmp(type, "pci")) {
			 dev_err(dev, "invalid \"device_type\" %s\n", type);
			 return -EINVAL;
	 }

    /* ------------------------------------------------ *
     * Get the virtual address for the module registers 
     * ------------------------------------------------ */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (res == NULL) {
        printk(KERN_ERR DRIVER_NAME " failed to get io memory region resouce.\n");
        error = -ENXIO;
        goto failed;
    }

    if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
        printk(KERN_ERR DRIVER_NAME "failed to request I/O memory\n");
        error = -EBUSY;
        goto failed;
    }

    npcmx50_pcie_rc_reg_base_virtual_address = ioremap(res->start, resource_size(res));
    
    if (npcmx50_pcie_rc_reg_base_virtual_address == NULL) {
        printk(KERN_ERR DRIVER_NAME "failed to remap I/O memory\n");
        error = -ENXIO;
        goto failed1;
    }
    printk(KERN_INFO "npcmx50_pcie_rc_probe: registers base ph:%x, v:%x\n",res->start,(unsigned int)npcmx50_pcie_rc_reg_base_virtual_address);

    /* ------------------------------------------------ *
     * Get the virtual address for the Config window 
     * ------------------------------------------------ */
    if (!request_mem_region(CONFIG_WIN_BASE, CONFIG_WIN_SIZE , pdev->name)) {
        printk(KERN_ERR DRIVER_NAME "failed to request config window area\n");
        error = -EBUSY;
        goto failed2;
    }

    npcmx50_pcie_rc_config_area_virtual_address = ioremap(CONFIG_WIN_BASE, CONFIG_WIN_SIZE);  
    if (npcmx50_pcie_rc_config_area_virtual_address == NULL) {
        printk(KERN_ERR DRIVER_NAME "failed to map config window area\n");
        error = -ENXIO;
        goto failed3;
    }

    /* ------------------------------------------------ *
     * Get the virtual address for the GCR_BASE
     * ------------------------------------------------ */
    if (!request_mem_region(NPCMX50_GCR_PHYS_BASE_ADDR, 0x1000, pdev->name)) {
        printk(KERN_ERR DRIVER_NAME "failed to request area for GCR_BASE \n");
        error = -EBUSY;
        goto failed4;
    }

    npcmx50_gcr_reg_base_virtual_address = ioremap(NPCMX50_GCR_PHYS_BASE_ADDR, 0x1000);  
    if (npcmx50_gcr_reg_base_virtual_address == NULL) {
        printk(KERN_ERR DRIVER_NAME "failed to map I/O window area\n");
        error = -ENXIO;
        goto failed5;
    }
    
    /* ------------------------------------------------ *
     * Get the virtual address for the CLK_BASE 
     * ------------------------------------------------ */
    if (!request_mem_region(NPCMX50_CLK_PHYS_BASE_ADDR, 0x1000, pdev->name)) {
        printk(KERN_ERR DRIVER_NAME "failed to request area for CLK_BASE \n");
        error = -EBUSY;
        goto failed6;
    }

    npcmx50_clk_reg_base_virtual_address = ioremap(NPCMX50_CLK_PHYS_BASE_ADDR, 0x1000);  
    if (npcmx50_clk_reg_base_virtual_address == NULL) {
        printk(KERN_ERR DRIVER_NAME "failed to map I/O window area\n");
        error = -ENXIO;
        goto failed7;
    }

    /* ------------------------------------------------ *
     * Get the virtual address for the GPIO_PCI_BASE_ADDR 
     * ------------------------------------------------ */
    if (!request_mem_region(GPIO_PCI_BASE_ADDR, 0x1000, pdev->name)) {
        printk(KERN_ERR DRIVER_NAME "failed to request area for 0x%08x\n", GPIO_PCI_BASE_ADDR);
        error = -EBUSY;
        goto failed8;
    }

    npcmx50_gpio_pci_reg_base_virtual_address = ioremap(GPIO_PCI_BASE_ADDR, 0x1000);  
    if (npcmx50_gpio_pci_reg_base_virtual_address == NULL) {
        printk(KERN_ERR DRIVER_NAME "failed to map I/O window area\n");
        error = -ENXIO;
        goto failed9;
    }

	/* Register the device */
	memset(&hw, 0, sizeof(hw));
	hw = (struct hw_pci) {
		.nr_controllers	= 1,
		.private_data	= (void **)&port,
		.setup		= npcmx50_pcie_rc_setup,
		.map_irq	= npcm750_pcie_map_irq,
		.scan		= npcmx50_pcie_rc_scan_bus,
		.ops		= &npcmx50_pcie_rc_ops,
	};

#ifdef CONFIG_PCI_MSI
		npcm750_pcie_msi_chip.dev = port->dev;
		hw.msi_ctrl = &npcm750_pcie_msi_chip;
#endif

    /* ------------------------------------------------ *
     * Init the device 
     * ------------------------------------------------ */
    pci_common_init_dev(dev,&hw);

	release_mem_region(IO_WIN_BASE, IO_WIN_SIZE);

    PCIE_DBG(KERN_DEBUG_PCIE "npcmx50_pcie_rc_probe - End.\n");  

    return 0;


failed9:
    release_mem_region(GPIO_PCI_BASE_ADDR, 0x1000);
failed8:
    iounmap(npcmx50_clk_reg_base_virtual_address);
failed7:
    release_mem_region(NPCMX50_CLK_PHYS_BASE_ADDR, 0x1000);
failed6:
    iounmap(npcmx50_gcr_reg_base_virtual_address);
failed5:
    release_mem_region(NPCMX50_GCR_PHYS_BASE_ADDR, 0x1000);
failed4:
    iounmap(npcmx50_pcie_rc_config_area_virtual_address);
failed3:    
    release_mem_region(CONFIG_WIN_BASE, CONFIG_WIN_SIZE);
failed2:
    iounmap(npcmx50_pcie_rc_reg_base_virtual_address);
failed1:
    release_mem_region(res->start, resource_size(res));
failed:
    return error;

}


static int npcmx50_pcie_rc_remove(struct platform_device *pdev)
{
    struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct npcm750_pcie_port *port = platform_get_drvdata(pdev);

    PCIE_DBG(KERN_DEBUG_PCIE "npcmx50_pcie_rc_remove - Start.\n");        

    iounmap(npcmx50_pcie_rc_config_area_virtual_address);
    release_mem_region(CONFIG_WIN_BASE, CONFIG_WIN_SIZE);

    iounmap(npcmx50_pcie_rc_reg_base_virtual_address);
    if (res) {
        release_mem_region(res->start, resource_size(res));
    }

	npcm750_pcie_free_irq_domain(port);

    PCIE_DBG(KERN_DEBUG_PCIE "npcmx50_pcie_rc_remove - End.\n");        
 
    return 0;
}

#if 0
static struct resource npcmx50_pcie_rc_resources[] = {
    [0] = {
        .start  = PCIERC_PHYS_BASE_ADDR,
        .end    = PCIERC_PHYS_BASE_ADDR + SZ_4K - 1,
        .flags  = IORESOURCE_MEM,
    },
};

struct platform_device npcmx50_pcie_rc_device = {
         .name           = DRIVER_NAME,
         .id             = 0,
         .resource       = npcmx50_pcie_rc_resources,
         .num_resources  = ARRAY_SIZE(npcmx50_pcie_rc_resources),
};
#endif 

static struct of_device_id npcm750_pcie_of_match[] = {
	{ .compatible = "nuvoton,npcm750-pcirc", },
	{}
};

static struct platform_driver npcmx50_pcie_rc_driver = {
    .driver = {
        .name	= DRIVER_NAME,
		.of_match_table = npcm750_pcie_of_match,
        .owner	= THIS_MODULE,
    },
    .probe		= npcmx50_pcie_rc_probe,
    .remove		= npcmx50_pcie_rc_remove,
};


/*static int __init npcmx50_pcie_rc_init(void)
{
    int status;

    if ((status=platform_driver_register(&npcmx50_pcie_rc_driver)))
    {
        printk(KERN_ERR DRIVER_NAME " platform_driver_register error, status=%d\n", status);
        return status;
    }
        
    if ((status=platform_device_register(&npcmx50_pcie_rc_device)))
    {
        printk(KERN_ERR DRIVER_NAME " platform_device_register error, status=%d\n", status);
        platform_device_put(&npcmx50_pcie_rc_device);
        platform_driver_unregister(&npcmx50_pcie_rc_driver);
        return status;
    }    
    return status;
}
module_init(npcmx50_pcie_rc_init);*/

/*static void __exit npcmx50_pcie_rc_cleanup(void)
{
    PCIE_DBG(KERN_DEBUG_PCIE "npcmx50_pcie_rc_cleanup - Start.\n");
    platform_device_unregister(&npcmx50_pcie_rc_device);
    platform_driver_unregister(&npcmx50_pcie_rc_driver);
    PCIE_DBG(KERN_DEBUG_PCIE "npcmx50_pcie_rc_cleanup - End.\n");
}
module_exit(npcmx50_pcie_rc_cleanup);*/

module_platform_driver(npcmx50_pcie_rc_driver);

MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_DESCRIPTION("NPCM750 PCIE Roor Complex driver");
MODULE_LICENSE("GPL");

