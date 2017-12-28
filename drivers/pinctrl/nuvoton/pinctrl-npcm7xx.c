/*
 * Copyright (c) 2016, Dell Inc
 * Copyright (c) 2016-2017 Nuvoton Technology corporation.
 *
 * Released under the GPLv2 only.
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/machine.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/sysfs.h>
#include <linux/irq.h>

#include <linux/slab.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <linux/device.h>

#include "npcmreg.h"

#define DRV_DATE    "2016-10-15"
#define DRV_VERSION "1.0.0"

#undef DRV_MSG2
#define DRV_MSG2(x...)
#define GPIO_DRIVER


//#define irq_set_handler_locked(x, y) __irq_set_handler_locked(x->irq, y)

#define GCR_BANK  8
static struct npcm_reg npcm_bank[9];

struct npcm_dev {
	struct pinctrl_dev *pctl;
	struct platform_device *pdev;
	struct gpio_chip gc;
	struct irq_domain *domain;
};

static struct npcm_dev npcm;

#define gpiochip_to_data(_gc) container_of(_gc, struct npcm_dev, gc)

#define gcrreg(reg, shift, bits) ((GCR_ ## reg) << 8 | (bits << 5) | (shift))
enum {
	opSET,
	opGETBIT,
	opSETBIT,
	opCLRBIT,
};

/* Helper function to setup IRQ, ioremap, spinlocks per register block */
static int npcm_init_bank(int id, u32 base, size_t sz, int irqid,
			  const char *dts)
{
	int irq = 0;

#ifdef CONFIG_OF
	struct device_node *np;
	struct of_device_id dt_id[2] = { };
	struct resource res;
	int ret;

	if (dts != NULL) {
		snprintf(dt_id[0].compatible, sizeof(dt_id[0].compatible),
			 "%s", dts);
		np = of_find_matching_node(NULL, dt_id);
		if (np != NULL) {
			ret = of_address_to_resource(np, 0, &res);
			if (ret) {
				pr_err("of_address_to_resource pinctrl failed\n");
				return -1;
			}
			base = res.start+(id*sz);
			irq = irq_of_parse_and_map(np, irqid);
		}
	}
#endif
	if (id >= ARRAY_SIZE(npcm_bank))
		return -1;
	npcm_bank[id].base = ioremap(base, sz);
	pr_debug("[%d] iomap: %.8x, %.8x = %p irq:%d\n",
		id, base, sz, npcm_bank[id].base, irq);
	if (!npcm_bank[id].base)
		return -1;
	spin_lock_init(&npcm_bank[id].lock);
	npcm_bank[id].irq = irq;
	return 0;
}

/* Perform locked bit operations on GPIO registers */
static int gpio_bitop(int op, int offset, int reg)
{
	struct npcm_reg *bank;
	unsigned long flags;
	u32 mask;

	bank = &npcm_bank[offset / GPIO_PER_BANK];
	mask = (1L << (offset % GPIO_PER_BANK));
	spin_lock_irqsave(&bank->lock, flags);
	switch (op) {
	case opSET:
		npcm_write32(bank->base + reg, mask);
		break;
	case opGETBIT:
		mask &= npcm_read32(bank->base + reg);
		break;
	case opSETBIT:
		npcm_setbit32(bank->base + reg, mask);
		break;
	case opCLRBIT:
		npcm_clrbit32(bank->base + reg, mask);
		break;
	}
	spin_unlock_irqrestore(&bank->lock, flags);
	return !!mask;
}

/*
 * GPIO code
 */

/* Helper function to get GPIO direction */
static int _npcmgpio_get_direction(unsigned int gpio)
{
	u32 oe, ie;

	/* Get Input & Output state */
	ie = gpio_bitop(opGETBIT, gpio, GPnIEM);
	oe = gpio_bitop(opGETBIT, gpio, GPnOE);
	if (ie && !oe)
		return GPIOF_DIR_IN;
	else if (oe && !ie)
		return GPIOF_DIR_OUT;
	return -EINVAL;
}

/* Dump GPIO and GCR registers */
static void npcmgpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	int module;
	void *base;

	for (module = 0; module < 8; module++) {
		base = npcm_bank[module].base;
		seq_printf(s, "-- module %d [gpio%d - %d]\n", module,
			   module * GPIO_PER_BANK,
			   (module * GPIO_PER_BANK) + GPIO_PER_BANK - 1);
		seq_printf(s, "DIN :%.8x DOUT:%.8x IE  :%.8x OE	 :%.8x\n",
			   npcm_read32(base + GPnDIN),
			   npcm_read32(base + GPnDOUT),
			   npcm_read32(base + GPnIEM),
			   npcm_read32(base + GPnOE));
		seq_printf(s, "PU  :%.8x PD  :%.8x DB  :%.8x POL :%.8x\n",
			   npcm_read32(base + GPnPU),
			   npcm_read32(base + GPnPD),
			   npcm_read32(base + GPnDBNC),
			   npcm_read32(base + GPnPOL));
		seq_printf(s, "ETYP:%.8x EVBE:%.8x EVEN:%.8x EVST:%.8x\n",
			   npcm_read32(base + GPnEVTYP),
			   npcm_read32(base + GPnEVBE),
			   npcm_read32(base + GPnEVEN),
			   npcm_read32(base + GPnEVST));
		seq_printf(s, "OTYP:%.8x OSRC:%.8x ODSC:%.8x\n",
			   npcm_read32(base + GPnOTYP),
			   npcm_read32(base + GPnOSRC),
			   npcm_read32(base + GPnODSC));
		seq_printf(s, "OBL0:%.8x OBL1:%.8x OBL2:%.8x OBL3:%.8x\n",
			   npcm_read32(base + GPnOBL0),
			   npcm_read32(base + GPnOBL1),
			   npcm_read32(base + GPnOBL2),
			   npcm_read32(base + GPnOBL3));
		seq_printf(s, "SLCK:%.8x MLCK:%.8x\n",
			   npcm_read32(base + GPnSPLCK),
			   npcm_read32(base + GPnMPLCK));
	}
	seq_puts(s, "-- GCR\n");
	base = npcm_bank[GCR_BANK].base;
	seq_printf(s, "MFSEL1:%.8x MFSEL2:%.8x MFSEL3:%.8x MFSEL4:%.8x I2C:%.8x FLOCKR1:%.8x\n",
		   npcm_read32(base + GCR_MFSEL1),
		   npcm_read32(base + GCR_MFSEL2),
		   npcm_read32(base + GCR_MFSEL3),
		   npcm_read32(base + GCR_MFSEL4),
		   npcm_read32(base + GCR_I2CSEGSEL),
		   npcm_read32(base + GCR_FLOCKR1));
}

/* Get direction of GPIO pin */
static int npcmgpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	unsigned int gpio = chip->base + offset;

	DRV_MSG2("gpio_get_direction: gpio%d\n", gpio);
	return _npcmgpio_get_direction(gpio);
}

/* Set GPIO to Input */
static int npcmgpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	unsigned int gpio = chip->base + offset;

	DRV_MSG2("%s: %d\n", __func__, gpio);
	return pinctrl_gpio_direction_input(gpio);
}

/* Set GPIO to Output with initial value */
static int npcmgpio_direction_output(struct gpio_chip *chip,
				     unsigned int offset, int value)
{
	unsigned int gpio = chip->base + offset;

	DRV_MSG2("gpio_direction_output: gpio%d = %x\n", gpio, value);
	/* Check if we're enabled as an interrupt.. */
	if (gpio_bitop(opGETBIT, gpio, GPnEVEN) &&
	    gpio_bitop(opGETBIT, gpio, GPnIEM)) {
		pr_info("gpio_direction_output: IRQ enabled on gpio%d\n",
			gpio);
		return -EINVAL;
	}
	gpio_bitop(opSETBIT, gpio, value ? GPnDOS : GPnDOC);
	return pinctrl_gpio_direction_output(gpio);
}

/* Retrieve value of GPIO */
static int npcmgpio_get_value(struct gpio_chip *chip, unsigned int offset)
{
	unsigned int gpio = chip->base + offset;
	int dir;

	DRV_MSG2("gpio_get: gpio%d\n", gpio);
	dir = _npcmgpio_get_direction(gpio);
	return gpio_bitop(opGETBIT, gpio, dir == GPIOF_DIR_OUT ?
			  GPnDOUT : GPnDIN);
}

/* Set value of Output GPIO */
static void npcmgpio_set_value(struct gpio_chip *chip, unsigned int offset,
			       int value)
{
	unsigned int gpio = chip->base + offset;

	DRV_MSG2("gpio_set: gpio%d = %x\n", gpio, value);
	if (_npcmgpio_get_direction(gpio) == GPIOF_DIR_OUT)
		gpio_bitop(opSETBIT, gpio, value ? GPnDOS : GPnDOC);
}

#if (KERNEL_VERSION(4, 10, 17) >= LINUX_VERSION_CODE)
/* Set GPIO Debounce mode */
static int npcmgpio_set_debounce(struct gpio_chip *chip, unsigned int offset,
				 unsigned int debounce)
{
	unsigned int gpio = chip->base + offset;

	DRV_MSG2("gpio_set_debounce: gpio%d = %d\n", gpio, debounce);
	gpio_bitop(debounce ? opSETBIT : opCLRBIT, gpio, GPnDBNC);
	return 0;
}
#endif

#if ((KERNEL_VERSION(4, 7, 0) <= LINUX_VERSION_CODE) && (KERNEL_VERSION(4, 10, 17) >= LINUX_VERSION_CODE))
/* Set open-drain or push-pull mode */
static int npcmgpio_set_single_ended(struct gpio_chip *chip,
				     unsigned int offset,
				     enum single_ended_mode mode)
{
	unsigned int gpio = chip->base + offset;

	DRV_MSG2("gpio_set_single_ended: gpio%d mode:%d\n", gpio, mode);
	if (mode == LINE_MODE_PUSH_PULL)
		gpio_bitop(opCLRBIT, gpio, GPnOTYP);
	else if (mode == LINE_MODE_OPEN_DRAIN)
		gpio_bitop(opSETBIT, gpio, GPnOTYP);
	else
		return -EINVAL;
	return 0;
}
#endif

/* Dynamically allocate IRQ for GPIO */
static int npcmgpio_to_irq(struct gpio_chip *chip, unsigned int offset)
{
	struct npcm_dev *npcm = gpiochip_to_data(chip);
	unsigned int gpio = chip->base + offset;
	int irq;

	/* Allocate interrupt on demand */
	irq = irq_create_mapping(npcm->domain, gpio);
	DRV_MSG2("gpio_to_irq: gpio%d = %d\n", gpio, irq);
	if (!irq)
		return -EINVAL;
	return irq;
}

/* Request GPIO */
static int npcmgpio_gpio_request(struct gpio_chip *chip, unsigned int offset)
{
	unsigned int gpio = chip->base + offset;

	DRV_MSG2("gpio_request: gpio%d\n", gpio);
	return pinctrl_request_gpio(gpio);
}

/* Release GPIO */
static void npcmgpio_gpio_free(struct gpio_chip *chip, unsigned int offset)
{
	unsigned int gpio = chip->base + offset;

	DRV_MSG2("gpio_free: gpio%d\n", gpio);
	pinctrl_free_gpio(gpio);
}

/*
 * IRQ code
 */
#ifdef GPIO_DRIVER
static void npcmgpio_irq_handler(struct irq_desc *desc)
{
	struct irq_chip *chip;
	struct npcm_reg *bank;
	u32 sts, en, bit, virq;

	chip = irq_desc_get_chip(desc);
	bank = irq_desc_get_handler_data(desc);

	chained_irq_enter(chip, desc);
	sts = npcm_read32(bank->base + GPnEVST);
	en  = npcm_read32(bank->base + GPnEVEN);
	DRV_MSG2("==> got irq sts %.8x %.8x\n", sts, en);

	sts &= en;
	for_each_set_bit(bit, (const void *)&sts, GPIO_PER_BANK) {
		virq = irq_find_mapping(npcm.domain, bank->irqbase + bit);
		pr_info(" gpio:%u irq:%u\n", bank->irqbase + bit, virq);
		if (virq) {
			/* Pass off to virtual interrupt handler */
			generic_handle_irq(virq);
		} else {
			DRV_MSG2("gpio_irq_handler: spurious interrupt...\n");
			gpio_bitop(opCLRBIT, bank->irqbase + bit, GPnEVEN);
			gpio_bitop(opSETBIT, bank->irqbase + bit, GPnEVST);
		}
	}
	chained_irq_exit(chip, desc);
}
#endif

/* Set trigger type of GPIO interrupt */
static int npcmgpio_set_irq_type(struct irq_data *d, unsigned int type)
{
	unsigned int gpio = d->hwirq;
	struct npcm_reg *bank;

	DRV_MSG2("setirqtype: %u.%u = %u\n", gpio, d->irq, type);
	bank = irq_get_handler_data(d->irq);
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		/* EVTYP=1, POL=0, EVBE=0 */
		DRV_MSG2("	edge.rising\n");
		gpio_bitop(opCLRBIT, gpio, GPnEVBE);
		gpio_bitop(opCLRBIT, gpio, GPnPOL);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		/* EVTYP=1, POL=1, EVBE=1 */
		DRV_MSG2("	edge.falling\n");
		gpio_bitop(opCLRBIT, gpio, GPnEVBE);
		gpio_bitop(opSETBIT, gpio, GPnPOL);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		/* EVTYP=1, POL=0, EVBE=1 */
		DRV_MSG2("	edge.both\n");
		gpio_bitop(opSETBIT, gpio, GPnEVBE);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		/* EVTYP=0, POL=1 */
		DRV_MSG2("	level.low\n");
		gpio_bitop(opSETBIT, gpio, GPnPOL);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		/* EVTYP=0, POL=0 */
		DRV_MSG2("	level.high\n");
		gpio_bitop(opCLRBIT, gpio, GPnPOL);
		break;
	default:
		DRV_MSG2("	invalid irq type\n");
		return -EINVAL;
	}
	if (type & (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW)) {
		gpio_bitop(opCLRBIT, gpio, GPnEVTYP);
		irq_set_handler_locked(d, handle_level_irq);
	} else if (type & (IRQ_TYPE_EDGE_BOTH | IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING)) {
		gpio_bitop(opSETBIT, gpio, GPnEVTYP);
		irq_set_handler_locked(d, handle_edge_irq);
	}
	return 0;
}

/* ACK GPIO interrupt */
static void npcmgpio_irq_ack(struct irq_data *d)
{
	unsigned int gpio = d->hwirq;

	DRV_MSG2("irq_ack: %u.%u\n", gpio, d->irq);
	gpio_bitop(opSET, gpio, GPnEVST);
}

/* Disable GPIO interrupt */
static void npcmgpio_irq_mask(struct irq_data *d)
{
	unsigned int gpio = d->hwirq;

	/* Clear events */
	DRV_MSG2("irq_mask: %u.%u\n", gpio, d->irq);
	gpio_bitop(opSET, gpio, GPnEVENC);
}

/* Enable GPIO interrupt */
static void npcmgpio_irq_unmask(struct irq_data *d)
{
	unsigned int gpio = d->hwirq;

	/* Enable events */
	DRV_MSG2("irq_unmask: %u.%u\n", gpio, d->irq);
	gpio_bitop(opSET, gpio, GPnEVENS);
}

/* Initialize GPIO interrupt */
static unsigned int npcmgpio_irq_startup(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	unsigned int gpio = d->hwirq;

	/* active-high, input, clear interrupt, enable interrupt */
	DRV_MSG2("startup: %u.%u\n", gpio, d->irq);
	npcmgpio_direction_output(gc, gpio, 1);
	npcmgpio_direction_input(gc, gpio);
	npcmgpio_irq_ack(d);
	npcmgpio_irq_unmask(d);
	return 0;
}

static struct irq_chip npcmgpio_irqchip = {
	.name = "npcm",
	.irq_ack = npcmgpio_irq_ack,
	.irq_unmask = npcmgpio_irq_unmask,
	.irq_mask = npcmgpio_irq_mask,
	.irq_set_type = npcmgpio_set_irq_type,
	.irq_startup = npcmgpio_irq_startup,
};

static int npcmgpio_irqdomain_map(struct irq_domain *dom,
				  unsigned int irq,
				  irq_hw_number_t hwirq)
{
	DRV_MSG2("irqdomainmap: %lu -> %u %p %p\n", hwirq, irq, &npcm,
		 dom->host_data);
	irq_set_chip_and_handler(irq, &npcmgpio_irqchip,
				 handle_simple_irq);
	irq_set_chip_data(irq, dom->host_data);
	return 0;
}

static const struct irq_domain_ops npcmgpio_domain_ops = {
	.map = npcmgpio_irqdomain_map,
};

/*
 * PINCTRL code
 */

static const int smb0_pins[]  = { 115, 114 };
static const int smb0b_pins[] = { 195, 194 };
static const int smb0c_pins[] = { 202, 196 };
static const int smb0d_pins[] = { 198, 199 };
static const int smb0den_pins[] = { 197 };

static const int smb1_pins[]  = { 117, 116 };
static const int smb1b_pins[] = { 126, 127 };
static const int smb1c_pins[] = { 124, 125 };
static const int smb1d_pins[] = { 4, 5 };

static const int smb2_pins[]  = { 119, 118 };
static const int smb2b_pins[] = { 122, 123 };
static const int smb2c_pins[] = { 120, 121 };
static const int smb2d_pins[] = { 6, 7 };

static const int smb3_pins[]  = { 30, 31 };
static const int smb3b_pins[] = { 39, 40 };
static const int smb3c_pins[] = { 37, 38 };
static const int smb3d_pins[] = { 59, 60 };

static const int smb4_pins[]  = { 28, 29 };
static const int smb4b_pins[] = { 18, 19 };
static const int smb4c_pins[] = { 20, 21 };
static const int smb4d_pins[] = { 22, 23 };
static const int smb4den_pins[] = { 17 };

static const int smb5_pins[]  = { 26, 27 };
static const int smb5b_pins[] = { 13, 12 };
static const int smb5c_pins[] = { 15, 14 };
static const int smb5d_pins[] = { 94, 93 };
static const int ga20kbc_pins[] = { 94, 93 };

static const int smb6_pins[]  = { 172, 171 };
static const int smb7_pins[]  = { 174, 173 };
static const int smb8_pins[]  = { 129, 128 };
static const int smb9_pins[]  = { 131, 130 };
static const int smb10_pins[] = { 133, 132 };
static const int smb11_pins[] = { 135, 134 };
static const int smb12_pins[] = { 221, 220 };
static const int smb13_pins[] = { 223, 222 };
static const int smb14_pins[] = { 22, 23 };
static const int smb15_pins[] = { 20, 21 };

static const int fanin0_pins[] = { 64 };
static const int fanin1_pins[] = { 65 };
static const int fanin2_pins[] = { 66 };
static const int fanin3_pins[] = { 67 };
static const int fanin4_pins[] = { 68 };
static const int fanin5_pins[] = { 69 };
static const int fanin6_pins[] = { 70 };
static const int fanin7_pins[] = { 71 };
static const int fanin8_pins[] = { 72 };
static const int fanin9_pins[] = { 73 };
static const int fanin10_pins[] = { 74 };
static const int fanin11_pins[] = { 75 };
static const int fanin12_pins[] = { 76 };
static const int fanin13_pins[] = { 77 };
static const int fanin14_pins[] = { 78 };
static const int fanin15_pins[] = { 79 };
static const int faninx_pins[] = { 175, 176, 177, 203 };

static const int pwm0_pins[] = { 80 };
static const int pwm1_pins[] = { 81 };
static const int pwm2_pins[] = { 82 };
static const int pwm3_pins[] = { 83 };
static const int pwm4_pins[] = { 144 };
static const int pwm5_pins[] = { 145 };
static const int pwm6_pins[] = { 146 };
static const int pwm7_pins[] = { 147 };

static const int uart1_pins[] = { 43, 44, 45, 46, 47, 61, 62, 63 };
static const int uart2_pins[] = { 48, 49, 50, 51, 52, 53, 54, 55 };

static const int rg1_pins[] = { 96, 97, 98, 99, 100, 101, 102, 103, 104, 105,
	106, 107 };
static const int rg1mdio_pins[] = { 108, 109 };

static const int rg2_pins[] = { 110, 111, 112, 113, 208, 209, 210, 211, 212,
	213, 214, 215 };
static const int rg2mdio_pins[] = { 216, 217 };
static const int ddr_pins[] = { 110, 111, 112, 113, 208, 209, 210, 211, 212,
	213, 214, 215, 216, 217 };

static const int iox1_pins[] = { 0, 1, 2, 3 };
static const int iox2_pins[] = { 4, 5, 6, 7 };
static const int ioxh_pins[] = { 10, 11, 24, 25 };

static const int mmc_pins[] = { 152, 154, 156, 157, 158, 159 };
static const int mmcwp_pins[] = { 153 };
static const int mmccd_pins[] = { 155 };
static const int mmcrst_pins[] = { 155 };
static const int mmc8_pins[] = { 148, 149, 150, 151 };

static const int r1_pins[] = { 178, 179, 180, 181, 182, 193, 201 };
static const int r1err_pins[] = { 56 };
static const int r1md_pins[] = { 57, 58 };

static const int r2_pins[] = { 84, 85, 86, 87, 88, 89, 200 };
static const int r2err_pins[] = { 90 };
static const int r2md_pins[] = { 91, 92 };

static const int sd1_pins[] = { 136, 137, 138, 139, 140, 141, 142, 143 };
static const int sd1pwr_pins[] = { 143 };

static const int wdog1_pins[] = { 218 };
static const int wdog2_pins[] = { 219 };

static const int bmcuart0a_pins[] = { 41, 42 };
static const int bmcuart0b_pins[] = { 48, 49 };

static const int bmcuart1_pins[] = { 43, 44, 62, 63 };

static const int scipme_pins[] = { 169 };
static const int sci_pins[] = { 170 };
static const int serirq_pins[] = { 162 };

static const int clkout_pins[] = { 160 };
static const int clkreq_pins[] = { 231 };

static const int jtag2_pins[] = { 43, 44, 45, 46, 47 };

static const int gspi_pins[] = { 12, 13, 14, 15 };

static const int spix_pins[] = { 224, 225, 226, 227, 229, 230 };
static const int spixcs1_pins[] = { 228 };

static const int pspi1_pins[] = { 175, 176, 177 };
static const int pspi2_pins[] = { 17, 18, 19 };

static const int spi0cs1_pins[] = { 32 };

static const int spi3_pins[] = { 183, 184, 185, 186 };
static const int spi3cs1_pins[] = { 187 };
static const int spi3quad_pins[] = { 188, 189 };
static const int spi3cs2_pins[] = { 188 };
static const int spi3cs3_pins[] = { 189 };

static const int ddc_pins[] = { 204, 205, 206, 207 };

static const int lpc_pins[] = { 95, 161, 163, 164, 165, 166, 167 };
static const int lpcclk_pins[] = { 168 };
static const int espi_pins[] = { 95, 161, 163, 164, 165, 166, 167, 168 };

static const int lkgpo0_pins[] = { 16 };
static const int lkgpo1_pins[] = { 8 };
static const int lkgpo2_pins[] = { 9 };

static const int nprd_smi_pins[] = { 190 };

/*
 * pin:	     name, number
 * group:    name, npins,   pins
 * function: name, ngroups, groups
 */
struct npcm_group {
	const char *name;
	const unsigned int *pins;
	int npins;
};

#define NPCM_GRPS \
	GRP(smb0), \
	GRP(smb0b), \
	GRP(smb0c), \
	GRP(smb0d), \
	GRP(smb0den), \
	GRP(smb1), \
	GRP(smb1b), \
	GRP(smb1c), \
	GRP(smb1d), \
	GRP(smb2), \
	GRP(smb2b), \
	GRP(smb2c), \
	GRP(smb2d), \
	GRP(smb3), \
	GRP(smb3b), \
	GRP(smb3c), \
	GRP(smb3d), \
	GRP(smb4), \
	GRP(smb4b), \
	GRP(smb4c), \
	GRP(smb4d), \
	GRP(smb4den), \
	GRP(smb5), \
	GRP(smb5b), \
	GRP(smb5c), \
	GRP(smb5d), \
	GRP(ga20kbc), \
	GRP(smb6), \
	GRP(smb7), \
	GRP(smb8), \
	GRP(smb9), \
	GRP(smb10), \
	GRP(smb11), \
	GRP(smb12), \
	GRP(smb13), \
	GRP(smb14), \
	GRP(smb15), \
	GRP(fanin0), \
	GRP(fanin1), \
	GRP(fanin2), \
	GRP(fanin3), \
	GRP(fanin4), \
	GRP(fanin5), \
	GRP(fanin6), \
	GRP(fanin7), \
	GRP(fanin8), \
	GRP(fanin9), \
	GRP(fanin10), \
	GRP(fanin11), \
	GRP(fanin12), \
	GRP(fanin13), \
	GRP(fanin14), \
	GRP(fanin15), \
	GRP(faninx), \
	GRP(pwm0), \
	GRP(pwm1), \
	GRP(pwm2), \
	GRP(pwm3), \
	GRP(pwm4), \
	GRP(pwm5), \
	GRP(pwm6), \
	GRP(pwm7), \
	GRP(rg1), \
	GRP(rg1mdio), \
	GRP(rg2), \
	GRP(rg2mdio), \
	GRP(ddr), \
	GRP(uart1), \
	GRP(uart2), \
	GRP(bmcuart0a), \
	GRP(bmcuart0b), \
	GRP(bmcuart1), \
	GRP(iox1), \
	GRP(iox2), \
	GRP(ioxh), \
	GRP(gspi), \
	GRP(mmc), \
	GRP(mmcwp), \
	GRP(mmccd), \
	GRP(mmcrst), \
	GRP(mmc8), \
	GRP(r1), \
	GRP(r1err), \
	GRP(r1md), \
	GRP(r2), \
	GRP(r2err), \
	GRP(r2md), \
	GRP(sd1), \
	GRP(sd1pwr), \
	GRP(wdog1), \
	GRP(wdog2), \
	GRP(scipme), \
	GRP(sci), \
	GRP(serirq), \
	GRP(jtag2), \
	GRP(spix), \
	GRP(spixcs1), \
	GRP(pspi1), \
	GRP(pspi2), \
	GRP(ddc), \
	GRP(clkreq), \
	GRP(clkout), \
	GRP(spi3), \
	GRP(spi3cs1), \
	GRP(spi3quad), \
	GRP(spi3cs2), \
	GRP(spi3cs3), \
	GRP(spi0cs1), \
	GRP(lpc), \
	GRP(lpcclk), \
	GRP(espi), \
	GRP(lkgpo0), \
	GRP(lkgpo1), \
	GRP(lkgpo2), \
	GRP(nprd_smi), \
	\

/* Group enums */
enum {
#define GRP(x) fn_ ## x
	NPCM_GRPS
	/* add placeholder for none/gpio */
	GRP(none),
	GRP(gpio),
#undef GRP
};

/* Group names/pins */
static struct npcm_group npcm_groups[] = {
#define GRP(x) { .name = #x, .pins = x ## _pins, .npins = ARRAY_SIZE(x ## _pins) }
	NPCM_GRPS
#undef GRP
};

#define NPCM_SFUNC(a) NPCM_FUNC(a, #a)
#define NPCM_FUNC(a, b...) static const char *a ## _grp[] = { b }
#define NPCM_MKFUNC(nm) { .name = #nm, .ngroups = ARRAY_SIZE(nm ## _grp), .groups = nm ## _grp }
struct npcm_func {
	const char *name;
	const unsigned int ngroups;
	const char *const *groups;
};

NPCM_SFUNC(smb0);
NPCM_SFUNC(smb0b);
NPCM_SFUNC(smb0c);
NPCM_SFUNC(smb0d);
NPCM_SFUNC(smb0den);
NPCM_SFUNC(smb1);
NPCM_SFUNC(smb1b);
NPCM_SFUNC(smb1c);
NPCM_SFUNC(smb1d);
NPCM_SFUNC(smb2);
NPCM_SFUNC(smb2b);
NPCM_SFUNC(smb2c);
NPCM_SFUNC(smb2d);
NPCM_SFUNC(smb3);
NPCM_SFUNC(smb3b);
NPCM_SFUNC(smb3c);
NPCM_SFUNC(smb3d);
NPCM_SFUNC(smb4);
NPCM_SFUNC(smb4b);
NPCM_SFUNC(smb4c);
NPCM_SFUNC(smb4d);
NPCM_SFUNC(smb4den);
NPCM_SFUNC(smb5);
NPCM_SFUNC(smb5b);
NPCM_SFUNC(smb5c);
NPCM_SFUNC(smb5d);
NPCM_SFUNC(ga20kbc);
NPCM_SFUNC(smb6);
NPCM_SFUNC(smb7);
NPCM_SFUNC(smb8);
NPCM_SFUNC(smb9);
NPCM_SFUNC(smb10);
NPCM_SFUNC(smb11);
NPCM_SFUNC(smb12);
NPCM_SFUNC(smb13);
NPCM_SFUNC(smb14);
NPCM_SFUNC(smb15);
NPCM_SFUNC(fanin0);
NPCM_SFUNC(fanin1);
NPCM_SFUNC(fanin2);
NPCM_SFUNC(fanin3);
NPCM_SFUNC(fanin4);
NPCM_SFUNC(fanin5);
NPCM_SFUNC(fanin6);
NPCM_SFUNC(fanin7);
NPCM_SFUNC(fanin8);
NPCM_SFUNC(fanin9);
NPCM_SFUNC(fanin10);
NPCM_SFUNC(fanin11);
NPCM_SFUNC(fanin12);
NPCM_SFUNC(fanin13);
NPCM_SFUNC(fanin14);
NPCM_SFUNC(fanin15);
NPCM_SFUNC(faninx);
NPCM_SFUNC(pwm0);
NPCM_SFUNC(pwm1);
NPCM_SFUNC(pwm2);
NPCM_SFUNC(pwm3);
NPCM_SFUNC(pwm4);
NPCM_SFUNC(pwm5);
NPCM_SFUNC(pwm6);
NPCM_SFUNC(pwm7);
NPCM_SFUNC(rg1);
NPCM_SFUNC(rg1mdio);
NPCM_SFUNC(rg2);
NPCM_SFUNC(rg2mdio);
NPCM_SFUNC(ddr);
NPCM_SFUNC(uart1);
NPCM_SFUNC(uart2);
NPCM_SFUNC(bmcuart0a);
NPCM_SFUNC(bmcuart0b);
NPCM_SFUNC(bmcuart1);
NPCM_SFUNC(iox1);
NPCM_SFUNC(iox2);
NPCM_SFUNC(ioxh);
NPCM_SFUNC(gspi);
NPCM_SFUNC(mmc);
NPCM_SFUNC(mmcwp);
NPCM_SFUNC(mmccd);
NPCM_SFUNC(mmcrst);
NPCM_SFUNC(mmc8);
NPCM_SFUNC(r1);
NPCM_SFUNC(r1err);
NPCM_SFUNC(r1md);
NPCM_SFUNC(r2);
NPCM_SFUNC(r2err);
NPCM_SFUNC(r2md);
NPCM_SFUNC(sd1);
NPCM_SFUNC(sd1pwr);
NPCM_SFUNC(wdog1);
NPCM_SFUNC(wdog2);
NPCM_SFUNC(scipme);
NPCM_SFUNC(sci);
NPCM_SFUNC(serirq);
NPCM_SFUNC(jtag2);
NPCM_SFUNC(spix);
NPCM_SFUNC(spixcs1);
NPCM_SFUNC(pspi1);
NPCM_SFUNC(pspi2);
NPCM_SFUNC(ddc);
NPCM_SFUNC(clkreq);
NPCM_SFUNC(clkout);
NPCM_SFUNC(spi3);
NPCM_SFUNC(spi3cs1);
NPCM_SFUNC(spi3quad);
NPCM_SFUNC(spi3cs2);
NPCM_SFUNC(spi3cs3);
NPCM_SFUNC(spi0cs1);
NPCM_SFUNC(lpc);
NPCM_SFUNC(lpcclk);
NPCM_SFUNC(espi);
NPCM_SFUNC(lkgpo0);
NPCM_SFUNC(lkgpo1);
NPCM_SFUNC(lkgpo2);
NPCM_SFUNC(nprd_smi);

/* Function names */
static struct npcm_func npcm_funcs[] = {
	NPCM_MKFUNC(smb0),
	NPCM_MKFUNC(smb0b),
	NPCM_MKFUNC(smb0c),
	NPCM_MKFUNC(smb0d),
	NPCM_MKFUNC(smb0den),
	NPCM_MKFUNC(smb1),
	NPCM_MKFUNC(smb1b),
	NPCM_MKFUNC(smb1c),
	NPCM_MKFUNC(smb1d),
	NPCM_MKFUNC(smb2),
	NPCM_MKFUNC(smb2b),
	NPCM_MKFUNC(smb2c),
	NPCM_MKFUNC(smb2d),
	NPCM_MKFUNC(smb3),
	NPCM_MKFUNC(smb3b),
	NPCM_MKFUNC(smb3c),
	NPCM_MKFUNC(smb3d),
	NPCM_MKFUNC(smb4),
	NPCM_MKFUNC(smb4b),
	NPCM_MKFUNC(smb4c),
	NPCM_MKFUNC(smb4d),
	NPCM_MKFUNC(smb4den),
	NPCM_MKFUNC(smb5),
	NPCM_MKFUNC(smb5b),
	NPCM_MKFUNC(smb5c),
	NPCM_MKFUNC(smb5d),
	NPCM_MKFUNC(ga20kbc),
	NPCM_MKFUNC(smb6),
	NPCM_MKFUNC(smb7),
	NPCM_MKFUNC(smb8),
	NPCM_MKFUNC(smb9),
	NPCM_MKFUNC(smb10),
	NPCM_MKFUNC(smb11),
	NPCM_MKFUNC(smb12),
	NPCM_MKFUNC(smb13),
	NPCM_MKFUNC(smb14),
	NPCM_MKFUNC(smb15),
	NPCM_MKFUNC(fanin0),
	NPCM_MKFUNC(fanin1),
	NPCM_MKFUNC(fanin2),
	NPCM_MKFUNC(fanin3),
	NPCM_MKFUNC(fanin4),
	NPCM_MKFUNC(fanin5),
	NPCM_MKFUNC(fanin6),
	NPCM_MKFUNC(fanin7),
	NPCM_MKFUNC(fanin8),
	NPCM_MKFUNC(fanin9),
	NPCM_MKFUNC(fanin10),
	NPCM_MKFUNC(fanin11),
	NPCM_MKFUNC(fanin12),
	NPCM_MKFUNC(fanin13),
	NPCM_MKFUNC(fanin14),
	NPCM_MKFUNC(fanin15),
	NPCM_MKFUNC(faninx),
	NPCM_MKFUNC(pwm0),
	NPCM_MKFUNC(pwm1),
	NPCM_MKFUNC(pwm2),
	NPCM_MKFUNC(pwm3),
	NPCM_MKFUNC(pwm4),
	NPCM_MKFUNC(pwm5),
	NPCM_MKFUNC(pwm6),
	NPCM_MKFUNC(pwm7),
	NPCM_MKFUNC(rg1),
	NPCM_MKFUNC(rg1mdio),
	NPCM_MKFUNC(rg2),
	NPCM_MKFUNC(rg2mdio),
	NPCM_MKFUNC(ddr),
	NPCM_MKFUNC(uart1),
	NPCM_MKFUNC(uart2),
	NPCM_MKFUNC(bmcuart0a),
	NPCM_MKFUNC(bmcuart0b),
	NPCM_MKFUNC(bmcuart1),
	NPCM_MKFUNC(iox1),
	NPCM_MKFUNC(iox2),
	NPCM_MKFUNC(ioxh),
	NPCM_MKFUNC(gspi),
	NPCM_MKFUNC(mmc),
	NPCM_MKFUNC(mmcwp),
	NPCM_MKFUNC(mmccd),
	NPCM_MKFUNC(mmcrst),
	NPCM_MKFUNC(mmc8),
	NPCM_MKFUNC(r1),
	NPCM_MKFUNC(r1err),
	NPCM_MKFUNC(r1md),
	NPCM_MKFUNC(r2),
	NPCM_MKFUNC(r2err),
	NPCM_MKFUNC(r2md),
	NPCM_MKFUNC(sd1),
	NPCM_MKFUNC(sd1pwr),
	NPCM_MKFUNC(wdog1),
	NPCM_MKFUNC(wdog2),
	NPCM_MKFUNC(scipme),
	NPCM_MKFUNC(sci),
	NPCM_MKFUNC(serirq),
	NPCM_MKFUNC(jtag2),
	NPCM_MKFUNC(spix),
	NPCM_MKFUNC(spixcs1),
	NPCM_MKFUNC(pspi1),
	NPCM_MKFUNC(pspi2),
	NPCM_MKFUNC(ddc),
	NPCM_MKFUNC(clkreq),
	NPCM_MKFUNC(clkout),
	NPCM_MKFUNC(spi3),
	NPCM_MKFUNC(spi3cs1),
	NPCM_MKFUNC(spi3quad),
	NPCM_MKFUNC(spi3cs2),
	NPCM_MKFUNC(spi3cs3),
	NPCM_MKFUNC(spi0cs1),
	NPCM_MKFUNC(lpc),
	NPCM_MKFUNC(lpcclk),
	NPCM_MKFUNC(espi),
	NPCM_MKFUNC(lkgpo0),
	NPCM_MKFUNC(lkgpo1),
	NPCM_MKFUNC(lkgpo2),
	NPCM_MKFUNC(nprd_smi),
};

#define GCR_NONE 0

#define PINCFG(a, b, c, d, e, f, g, h, i, j, k) \
	[a] { .fn0 = fn_ ## b, .reg0 = GCR_ ## c, .bit0 = d, \
			.fn1 = fn_ ## e, .reg1 = GCR_ ## f, .bit1 = g, \
			.fn2 = fn_ ## h, .reg2 = GCR_ ## i, .bit2 = j, \
			.flag = k }

/* Drive strength controlled by GPnODSC */
#define DRIVE_STRENGTH_LO_SHIFT		8
#define DRIVE_STRENGTH_HI_SHIFT		12
#define DRIVE_STRENGTH_MASK		0x0000FF00

#define DS(lo, hi)	(((lo) << DRIVE_STRENGTH_LO_SHIFT) | \
			 ((hi) << DRIVE_STRENGTH_HI_SHIFT))
#define DSLO(x)		(((x) >> DRIVE_STRENGTH_LO_SHIFT) & 0xF)
#define DSHI(x)		(((x) >> DRIVE_STRENGTH_HI_SHIFT) & 0xF)

#define GPI		0x1 /* Not GPO */
#define GPO		0x2 /* Not GPI */
#define SLEW		0x4 /* Has Slew Control, GPnOSRC */
#define SLEWLPC		0x8 /* Has Slew Control, SRCNT.3 */

struct npcm_pincfg {
	int flag;
	int fn0, reg0, bit0;
	int fn1, reg1, bit1;
	int fn2, reg2, bit2;
};

static const struct npcm_pincfg pincfg[] = {
	/*	PIN	  FUNCTION 1		   FUNCTION 2		  FUNCTION 3	    FLAGS */
	PINCFG(0,	 iox1, MFSEL1, 30,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(1,	 iox1, MFSEL1, 30,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12)),
	PINCFG(2,	 iox1, MFSEL1, 30,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12)),
	PINCFG(3,	 iox1, MFSEL1, 30,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(4,	 iox2, MFSEL3, 14,	 smb1d, I2CSEGSEL, 7,	none, NONE, 0,	     SLEW),
	PINCFG(5,	 iox2, MFSEL3, 14,	 smb1d, I2CSEGSEL, 7,	none, NONE, 0,	     SLEW),
	PINCFG(6,	 iox2, MFSEL3, 14,	 smb2d, I2CSEGSEL, 10,  none, NONE, 0,       SLEW),
	PINCFG(7,	 iox2, MFSEL3, 14,	 smb2d, I2CSEGSEL, 10,  none, NONE, 0,       SLEW),
	PINCFG(8,      lkgpo1, FLOCKR1, 4,        none, NONE, 0,	none, NONE, 0,	     DS(8, 12)),
	PINCFG(9,      lkgpo2, FLOCKR1, 8,        none, NONE, 0,	none, NONE, 0,	     DS(8, 12)),
	PINCFG(10,	 ioxh, MFSEL3, 18,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12)),
	PINCFG(11,	 ioxh, MFSEL3, 18,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12)),
	PINCFG(12,	 gspi, MFSEL1, 24,	 smb5b, I2CSEGSEL, 19,  none, NONE, 0,	     SLEW),
	PINCFG(13,	 gspi, MFSEL1, 24,	 smb5b, I2CSEGSEL, 19,  none, NONE, 0,	     SLEW),
	PINCFG(14,	 gspi, MFSEL1, 24,	 smb5c, I2CSEGSEL, 20,	none, NONE, 0,	     SLEW),
	PINCFG(15,	 gspi, MFSEL1, 24,	 smb5c, I2CSEGSEL, 20,	none, NONE, 0,	     SLEW),
	PINCFG(16,     lkgpo0, FLOCKR1, 0,        none, NONE, 0,	none, NONE, 0,	     DS(8, 12)),
	PINCFG(17,      pspi2, MFSEL3, 13,     smb4den, I2CSEGSEL, 23,  none, NONE, 0,       DS(8, 12)),
	PINCFG(18,      pspi2, MFSEL3, 13,	 smb4b, I2CSEGSEL, 14,  none, NONE, 0,	     DS(8, 12)),
	PINCFG(19,      pspi2, MFSEL3, 13,	 smb4b, I2CSEGSEL, 14,  none, NONE, 0,	     DS(8, 12)),
	PINCFG(20,	smb4c, I2CSEGSEL, 15,    smb15, MFSEL3, 8,      none, NONE, 0,	     0),
	PINCFG(21,	smb4c, I2CSEGSEL, 15,    smb15, MFSEL3, 8,      none, NONE, 0,	     0),
	PINCFG(22,      smb4d, I2CSEGSEL, 16,	 smb14, MFSEL3, 7,      none, NONE, 0,	     0),
	PINCFG(23,      smb4d, I2CSEGSEL, 16,	 smb14, MFSEL3, 7,      none, NONE, 0,	     0),
	PINCFG(24,	 ioxh, MFSEL3, 18,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12)),
	PINCFG(25,	 ioxh, MFSEL3, 18,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12)),
	PINCFG(26,	 smb5, MFSEL1, 2,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(27,	 smb5, MFSEL1, 2,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(28,	 smb4, MFSEL1, 1,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(29,	 smb4, MFSEL1, 1,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(30,	 smb3, MFSEL1, 0,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(31,	 smb3, MFSEL1, 0,	  none, NONE, 0,	none, NONE, 0,	     0),

	PINCFG(32,    spi0cs1, MFSEL1, 3,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(33,   none, NONE, 0,     none, NONE, 0,	none, NONE, 0,	     SLEW),
	PINCFG(34,   none, NONE, 0,     none, NONE, 0,	none, NONE, 0,	     SLEW),
	PINCFG(37,	smb3c, I2CSEGSEL, 12,	  none, NONE, 0,	none, NONE, 0,	     SLEW),
	PINCFG(38,	smb3c, I2CSEGSEL, 12,	  none, NONE, 0,	none, NONE, 0,	     SLEW),
	PINCFG(39,	smb3b, I2CSEGSEL, 11,	  none, NONE, 0,	none, NONE, 0,	     SLEW),
	PINCFG(40,	smb3b, I2CSEGSEL, 11,	  none, NONE, 0,	none, NONE, 0,	     SLEW),
	PINCFG(41,  bmcuart0a, MFSEL1, 9,         none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(42,  bmcuart0a, MFSEL1, 9,         none, NONE, 0,	none, NONE, 0,	     DS(2, 4) | GPO),
	PINCFG(43,      uart1, MFSEL1, 10,	 jtag2, MFSEL4, 0,  bmcuart1, MFSEL3, 24,    0),
	PINCFG(44,      uart1, MFSEL1, 10,	 jtag2, MFSEL4, 0,  bmcuart1, MFSEL3, 24,    0),
	PINCFG(45,      uart1, MFSEL1, 10,	 jtag2, MFSEL4, 0,	none, NONE, 0,	     0),
	PINCFG(46,      uart1, MFSEL1, 10,	 jtag2, MFSEL4, 0,	none, NONE, 0,	     DS(2, 8)),
	PINCFG(47,      uart1, MFSEL1, 10,	 jtag2, MFSEL4, 0,	none, NONE, 0,	     DS(2, 8)),
	PINCFG(48,	uart2, MFSEL1, 11,   bmcuart0b, MFSEL4, 1,      none, NONE, 0,	     GPO),
	PINCFG(49,	uart2, MFSEL1, 11,   bmcuart0b, MFSEL4, 1,      none, NONE, 0,	     0),
	PINCFG(50,	uart2, MFSEL1, 11,	  none, NONE, 0,        none, NONE, 0,	     0),
	PINCFG(51,	uart2, MFSEL1, 11,	  none, NONE, 0,        none, NONE, 0,	     GPO),
	PINCFG(52,	uart2, MFSEL1, 11,	  none, NONE, 0,        none, NONE, 0,	     0),
	PINCFG(53,	uart2, MFSEL1, 11,	  none, NONE, 0,        none, NONE, 0,	     GPO),
	PINCFG(54,	uart2, MFSEL1, 11,	  none, NONE, 0,        none, NONE, 0,	     0),
	PINCFG(55,	uart2, MFSEL1, 11,	  none, NONE, 0,        none, NONE, 0,	     0),
	PINCFG(56,	r1err, MFSEL1, 12,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(57,       r1md, MFSEL1, 13,        none, NONE, 0,        none, NONE, 0,       DS(2, 4)),
	PINCFG(58,       r1md, MFSEL1, 13,        none, NONE, 0,	none, NONE, 0,	     DS(2, 4)),
	PINCFG(59,	smb3d, I2CSEGSEL, 13,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(60,	smb3d, I2CSEGSEL, 13,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(61,      uart1, MFSEL1, 10,	  none, NONE, 0,	none, NONE, 0,     GPO),
	PINCFG(62,      uart1, MFSEL1, 10,    bmcuart1, MFSEL3, 24,	none, NONE, 0,     GPO),
	PINCFG(63,      uart1, MFSEL1, 10,    bmcuart1, MFSEL3, 24,	none, NONE, 0,     GPO),

	PINCFG(64,    fanin0, MFSEL2, 0,          none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(65,    fanin1, MFSEL2, 1,          none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(66,    fanin2, MFSEL2, 2,          none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(67,    fanin3, MFSEL2, 3,          none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(68,    fanin4, MFSEL2, 4,          none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(69,    fanin5, MFSEL2, 5,          none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(70,    fanin6, MFSEL2, 6,          none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(71,    fanin7, MFSEL2, 7,          none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(72,    fanin8, MFSEL2, 8,          none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(73,    fanin9, MFSEL2, 9,          none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(74,    fanin10, MFSEL2, 10,        none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(75,    fanin11, MFSEL2, 11,        none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(76,    fanin12, MFSEL2, 12,        none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(77,    fanin13, MFSEL2, 13,        none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(78,    fanin14, MFSEL2, 14,        none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(79,    fanin15, MFSEL2, 15,        none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(80,	 pwm0, MFSEL2, 16,        none, NONE, 0,	none, NONE, 0,	     DS(4, 8)),
	PINCFG(81,	 pwm1, MFSEL2, 17,        none, NONE, 0,	none, NONE, 0,	     DS(4, 8)),
	PINCFG(82,	 pwm2, MFSEL2, 18,        none, NONE, 0,	none, NONE, 0,	     DS(4, 8)),
	PINCFG(83,	 pwm3, MFSEL2, 19,        none, NONE, 0,	none, NONE, 0,	     DS(4, 8)),
	PINCFG(84,         r2, MFSEL1, 14,        none, NONE, 0,        none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(85,         r2, MFSEL1, 14,        none, NONE, 0,        none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(86,         r2, MFSEL1, 14,        none, NONE, 0,        none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(87,         r2, MFSEL1, 14,        none, NONE, 0,        none, NONE, 0,	     0),
	PINCFG(88,         r2, MFSEL1, 14,        none, NONE, 0,        none, NONE, 0,	     0),
	PINCFG(89,         r2, MFSEL1, 14,        none, NONE, 0,        none, NONE, 0,	     0),
	PINCFG(90,      r2err, MFSEL1, 15,        none, NONE, 0,        none, NONE, 0,       0),
	PINCFG(91,       r2md, MFSEL1, 16,	  none, NONE, 0,        none, NONE, 0,	     DS(2, 4)),
	PINCFG(92,       r2md, MFSEL1, 16,	  none, NONE, 0,        none, NONE, 0,	     DS(2, 4)),
	PINCFG(93,    ga20kbc, MFSEL1, 17,	 smb5d, I2CSEGSEL, 21,  none, NONE, 0,	     0),
	PINCFG(94,    ga20kbc, MFSEL1, 17,	 smb5d, I2CSEGSEL, 21,  none, NONE, 0,	     0),
	PINCFG(95,	  lpc, NONE, 0,		  espi, MFSEL4, 8,      gpio, MFSEL1, 26,    0),

	PINCFG(96,	  rg1, MFSEL4, 22,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(97,	  rg1, MFSEL4, 22,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(98,	  rg1, MFSEL4, 22,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(99,	  rg1, MFSEL4, 22,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(100,	  rg1, MFSEL4, 22,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(101,	  rg1, MFSEL4, 22,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(102,	  rg1, MFSEL4, 22,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(103,	  rg1, MFSEL4, 22,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(104,	  rg1, MFSEL4, 22,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(105,	  rg1, MFSEL4, 22,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(106,	  rg1, MFSEL4, 22,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(107,	  rg1, MFSEL4, 22,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(108,   rg1mdio, MFSEL4, 21,        none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(109,   rg1mdio, MFSEL4, 21,        none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(110,       rg2, MFSEL4, 24,         ddr, MFSEL3, 26,     none, NONE, 0,       0),
	PINCFG(111,       rg2, MFSEL4, 24,         ddr, MFSEL3, 26,     none, NONE, 0,       0),
	PINCFG(112,       rg2, MFSEL4, 24,         ddr, MFSEL3, 26,     none, NONE, 0,       0),
	PINCFG(113,       rg2, MFSEL4, 24,         ddr, MFSEL3, 26,     none, NONE, 0,       0),
	PINCFG(114,	 smb0, MFSEL1, 6,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(115,	 smb0, MFSEL1, 6,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(116,	 smb1, MFSEL1, 7,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(117,	 smb1, MFSEL1, 7,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(118,	 smb2, MFSEL1, 8,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(119,	 smb2, MFSEL1, 8,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(120,	smb2c, I2CSEGSEL, 9,	  none, NONE, 0,	none, NONE, 0,	     SLEW),
	PINCFG(121,	smb2c, I2CSEGSEL, 9,      none, NONE, 0,	none, NONE, 0,	     SLEW),
	PINCFG(122,	smb2b, I2CSEGSEL, 8,	  none, NONE, 0,	none, NONE, 0,	     SLEW),
	PINCFG(123,	smb2b, I2CSEGSEL, 8,	  none, NONE, 0,	none, NONE, 0,	     SLEW),
	PINCFG(124,	smb1c, I2CSEGSEL, 6,	  none, NONE, 0,	none, NONE, 0,	     SLEW),
	PINCFG(125,	smb1c, I2CSEGSEL, 6,	  none, NONE, 0,	none, NONE, 0,	     SLEW),
	PINCFG(126,	smb1b, I2CSEGSEL, 5,	  none, NONE, 0,	none, NONE, 0,	     SLEW),
	PINCFG(127,	smb1b, I2CSEGSEL, 5,	  none, NONE, 0,	none, NONE, 0,	     SLEW),

	PINCFG(128,	 smb8, MFSEL4, 11,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(129,	 smb8, MFSEL4, 11,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(130,	 smb9, MFSEL4, 12,        none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(131,	 smb9, MFSEL4, 12,        none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(132,	smb10, MFSEL4, 13,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(133,	smb10, MFSEL4, 13,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(134,	smb11, MFSEL4, 14,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(135,	smb11, MFSEL4, 14,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(136,	  sd1, MFSEL3, 12,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(137,	  sd1, MFSEL3, 12,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(138,	  sd1, MFSEL3, 12,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(139,	  sd1, MFSEL3, 12,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(140,	  sd1, MFSEL3, 12,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(141,	  sd1, MFSEL3, 12,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(142,	  sd1, MFSEL3, 12,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(143,       sd1, MFSEL3, 12,      sd1pwr, MFSEL4, 5,      none, NONE, 0,       0),
	PINCFG(144,	 pwm4, MFSEL2, 20,	  none, NONE, 0,	none, NONE, 0,	     DS(4, 8)),
	PINCFG(145,	 pwm5, MFSEL2, 21,	  none, NONE, 0,	none, NONE, 0,	     DS(4, 8)),
	PINCFG(146,	 pwm6, MFSEL2, 22,	  none, NONE, 0,	none, NONE, 0,	     DS(4, 8)),
	PINCFG(147,	 pwm7, MFSEL2, 23,	  none, NONE, 0,	none, NONE, 0,	     DS(4, 8)),
	PINCFG(148,	 mmc8, MFSEL3, 11,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(149,	 mmc8, MFSEL3, 11,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(150,	 mmc8, MFSEL3, 11,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(151,	 mmc8, MFSEL3, 11,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(152,	  mmc, MFSEL3, 10,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(153,     mmcwp, FLOCKR1, 24,       none, NONE, 0,	none, NONE, 0,	     0),  /* Z1/A1 */
	PINCFG(154,	  mmc, MFSEL3, 10,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(155,     mmccd, MFSEL3, 25,      mmcrst, MFSEL4, 6,      none, NONE, 0,       0),  /* Z1/A1 */
	PINCFG(156,	  mmc, MFSEL3, 10,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(157,	  mmc, MFSEL3, 10,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(158,	  mmc, MFSEL3, 10,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(159,	  mmc, MFSEL3, 10,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),

	PINCFG(160,    clkout, MFSEL1, 21,        none, NONE, 0,        none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(161,	  lpc, NONE, 0,		  espi, MFSEL4, 8,      gpio, MFSEL1, 26,    DS(8, 12)),
	PINCFG(162,    serirq, NONE, 0,           gpio, MFSEL1, 31,	none, NONE, 0,	     DS(8, 12)),
	PINCFG(163,	  lpc, NONE, 0,		  espi, MFSEL4, 8,      gpio, MFSEL1, 26,    0),
	PINCFG(164,	  lpc, NONE, 0,		  espi, MFSEL4, 8,      gpio, MFSEL1, 26,    SLEWLPC),
	PINCFG(165,	  lpc, NONE, 0,		  espi, MFSEL4, 8,      gpio, MFSEL1, 26,    SLEWLPC),
	PINCFG(166,	  lpc, NONE, 0,		  espi, MFSEL4, 8,      gpio, MFSEL1, 26,    SLEWLPC),
	PINCFG(167,	  lpc, NONE, 0,		  espi, MFSEL4, 8,      gpio, MFSEL1, 26,    SLEWLPC),
	PINCFG(168,    lpcclk, NONE, 0,           espi, MFSEL4, 8,      gpio, MFSEL3, 16,    0),
	PINCFG(169,    scipme, MFSEL3, 0,         none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(170,	  sci, MFSEL1, 22,        none, NONE, 0,        none, NONE, 0,	     0),
	PINCFG(171,	 smb6, MFSEL3, 1,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(172,	 smb6, MFSEL3, 1,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(173,	 smb7, MFSEL3, 2,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(174,	 smb7, MFSEL3, 2,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(175,	pspi1, MFSEL3, 4,       faninx, MFSEL3, 3,      none, NONE, 0,	     DS(8, 12)),
	PINCFG(176,     pspi1, MFSEL3, 4,       faninx, MFSEL3, 3,      none, NONE, 0,	     DS(8, 12)),
	PINCFG(177,     pspi1, MFSEL3, 4,       faninx, MFSEL3, 3,      none, NONE, 0,	     DS(8, 12)),
	PINCFG(178,	   r1, MFSEL3, 9,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(179,	   r1, MFSEL3, 9,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(180,	   r1, MFSEL3, 9,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(181,	   r1, MFSEL3, 9,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(182,	   r1, MFSEL3, 9,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(183,     spi3, MFSEL4, 16,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(184,     spi3, MFSEL4, 16,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW | GPO),
	PINCFG(185,     spi3, MFSEL4, 16,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW | GPO),
	PINCFG(186,     spi3, MFSEL4, 16,	  none, NONE, 0,	none, NONE, 0,	     DS(8, 12)),
	PINCFG(187,   spi3cs1, MFSEL4, 17,        none, NONE, 0,	none, NONE, 0,	     DS(8, 12)),
	PINCFG(188,  spi3quad, MFSEL4, 20,     spi3cs2, MFSEL4, 18,     none, NONE, 0,    DS(8, 12) | SLEW),
	PINCFG(189,  spi3quad, MFSEL4, 20,     spi3cs3, MFSEL4, 19,     none, NONE, 0,    DS(8, 12) | SLEW),
	PINCFG(190,      gpio, FLOCKR1, 20,   nprd_smi, NONE, 0,	none, NONE, 0,	     DS(2, 4)),
	PINCFG(191,	 none, NONE, 0,		  none, NONE, 0,	none, NONE, 0,	     DS(8, 12)),  /* XX */

	PINCFG(192,	 none, NONE, 0,		  none, NONE, 0,	none, NONE, 0,	     DS(8, 12)),  /* XX */
	PINCFG(193,	   r1, MFSEL3, 9,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(194,	smb0b, I2CSEGSEL, 0,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(195,	smb0b, I2CSEGSEL, 0,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(196,	smb0c, I2CSEGSEL, 1,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(197,   smb0den, I2CSEGSEL, 22,     none, NONE, 0,	none, NONE, 0,	     SLEW),
	PINCFG(198,	smb0d, I2CSEGSEL, 2,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(199,	smb0d, I2CSEGSEL, 2,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(200,        r2, MFSEL1, 14,        none, NONE, 0,        none, NONE, 0,       0),
	PINCFG(201,	   r1, MFSEL3, 9,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(202,	smb0c, I2CSEGSEL, 1,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(203,    faninx, MFSEL3, 3,         none, NONE, 0,	none, NONE, 0,	     DS(8, 12)),
	PINCFG(204,	  ddc, NONE, 0,           gpio, MFSEL3, 22,	none, NONE, 0,	     SLEW),
	PINCFG(205,	  ddc, NONE, 0,           gpio, MFSEL3, 22,	none, NONE, 0,	     SLEW),
	PINCFG(206,	  ddc, NONE, 0,           gpio, MFSEL3, 22,	none, NONE, 0,	     DS(4, 8)),
	PINCFG(207,	  ddc, NONE, 0,           gpio, MFSEL3, 22,	none, NONE, 0,	     DS(4, 8)),
	PINCFG(208,       rg2, MFSEL4, 24,         ddr, MFSEL3, 26,     none, NONE, 0,       0),
	PINCFG(209,       rg2, MFSEL4, 24,         ddr, MFSEL3, 26,     none, NONE, 0,       0),
	PINCFG(210,       rg2, MFSEL4, 24,         ddr, MFSEL3, 26,     none, NONE, 0,       0),
	PINCFG(211,       rg2, MFSEL4, 24,         ddr, MFSEL3, 26,     none, NONE, 0,       0),
	PINCFG(212,       rg2, MFSEL4, 24,         ddr, MFSEL3, 26,     none, NONE, 0,       0),
	PINCFG(213,       rg2, MFSEL4, 24,         ddr, MFSEL3, 26,     none, NONE, 0,       0),
	PINCFG(214,       rg2, MFSEL4, 24,         ddr, MFSEL3, 26,     none, NONE, 0,       0),
	PINCFG(215,       rg2, MFSEL4, 24,         ddr, MFSEL3, 26,     none, NONE, 0,       0),
	PINCFG(216,   rg2mdio, MFSEL4, 23,         ddr, MFSEL3, 26,     none, NONE, 0,       0),
	PINCFG(217,   rg2mdio, MFSEL4, 23,         ddr, MFSEL3, 26,     none, NONE, 0,       0),
	PINCFG(218,     wdog1, MFSEL3, 19,        none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(219,     wdog2, MFSEL3, 20,        none, NONE, 0,	none, NONE, 0,	     DS(4, 8)),
	PINCFG(220,	smb12, MFSEL3, 5,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(221,	smb12, MFSEL3, 5,	  none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(222,     smb13, MFSEL3, 6,         none, NONE, 0,	none, NONE, 0,	     0),
	PINCFG(223,     smb13, MFSEL3, 6,         none, NONE, 0,	none, NONE, 0,	     0),

	PINCFG(224,	 spix, MFSEL4, 27,        none, NONE, 0,	none, NONE, 0,	     SLEW),
	PINCFG(225,	 spix, MFSEL4, 27,        none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW | GPO),
	PINCFG(226,	 spix, MFSEL4, 27,        none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW | GPO),
	PINCFG(227,	 spix, MFSEL4, 27,        none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(228,   spixcs1, MFSEL4, 28,        none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(229,	 spix, MFSEL4, 27,        none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(230,	 spix, MFSEL4, 27,        none, NONE, 0,	none, NONE, 0,	     DS(8, 12) | SLEW),
	PINCFG(231,    clkreq, MFSEL4, 9,         none, NONE, 0,        none, NONE, 0,	     DS(8, 12)),
	PINCFG(253,	 none, NONE, 0,		  none, NONE, 0,	none, NONE, 0,	     GPI), /* SDHC1 power */
	PINCFG(254,	 none, NONE, 0,		  none, NONE, 0,	none, NONE, 0,	     GPI), /* SDHC2 power */
	PINCFG(255,	 none, NONE, 0,		  none, NONE, 0,	none, NONE, 0,	     GPI), /* DACOSEL */
};

/* number, name, drv_data */
static const struct pinctrl_pin_desc npcm_pins[] = {
	PINCTRL_PIN(0,	"GPIO0/IOX1DI"),
	PINCTRL_PIN(1,	"GPIO1/IOX1LD"),
	PINCTRL_PIN(2,	"GPIO2/IOX1CK"),
	PINCTRL_PIN(3,	"GPIO3/IOX1D0"),
	PINCTRL_PIN(4,	"GPIO4/IOX2DI/SMB1DSDA"),
	PINCTRL_PIN(5,	"GPIO5/IOX2LD/SMB1DSCL"),
	PINCTRL_PIN(6,	"GPIO6/IOX2CK/SMB2DSDA"),
	PINCTRL_PIN(7,	"GPIO7/IOX2D0/SMB2DSCL"),
	PINCTRL_PIN(8,	"GPIO8/LKGPO1"),
	PINCTRL_PIN(9,	"GPIO9/LKGPO2"),
	PINCTRL_PIN(10, "GPIO10/IOXHLD"),
	PINCTRL_PIN(11, "GPIO11/IOXHCK"),
	PINCTRL_PIN(12, "GPIO12/GSPICK/SMB5BSCL"),
	PINCTRL_PIN(13, "GPIO13/GSPIDO/SMB5BSDA"),
	PINCTRL_PIN(14, "GPIO14/GSPIDI/SMB5CSCL"),
	PINCTRL_PIN(15, "GPIO15/GSPICS/SMB5CSDA"),
	PINCTRL_PIN(16, "GPIO16/LKGPO0"),
	PINCTRL_PIN(17, "GPIO17/PSPI2DI/SMB4DEN"),
	PINCTRL_PIN(18, "GPIO18/PSPI2D0/SMB4BSDA"),
	PINCTRL_PIN(19, "GPIO19/PSPI2CK/SMB4BSCL"),
	PINCTRL_PIN(20, "GPIO20/SMB4CSDA/SMB15SDA"),
	PINCTRL_PIN(21, "GPIO21/SMB4CSCL/SMB15SCL"),
	PINCTRL_PIN(22, "GPIO22/SMB4DSDA/SMB14SDA"),
	PINCTRL_PIN(23, "GPIO23/SMB4DSCL/SMB14SCL"),
	PINCTRL_PIN(24, "GPIO24/IOXHDO"),
	PINCTRL_PIN(25, "GPIO25/IOXHDI"),
	PINCTRL_PIN(26, "GPIO26/SMB5SDA"),
	PINCTRL_PIN(27, "GPIO27/SMB5SCL"),
	PINCTRL_PIN(28, "GPIO28/SMB4SDA"),
	PINCTRL_PIN(29, "GPIO29/SMB4SCL"),
	PINCTRL_PIN(30, "GPIO30/SMB3SDA"),
	PINCTRL_PIN(31, "GPIO31/SMB3SCL"),

	PINCTRL_PIN(32, "GPIO32/nSPI0CS1"),
	PINCTRL_PIN(33, "SPI0D2"),
	PINCTRL_PIN(34, "SPI0D3"),
	PINCTRL_PIN(37, "GPIO37/SMB3CSDA"),
	PINCTRL_PIN(38, "GPIO38/SMB3CSCL"),
	PINCTRL_PIN(39, "GPIO39/SMB3BSDA"),
	PINCTRL_PIN(40, "GPIO40/SMB3BSCL"),
	PINCTRL_PIN(41, "GPIO41/BSPRXD"),
	PINCTRL_PIN(42, "GPO42/BSPTXD/STRAP11"),
	PINCTRL_PIN(43, "GPIO43/RXD1/JTMS2/BU1RXD"),
	PINCTRL_PIN(44, "GPIO44/nCTS1/JTDI2/BU1CTS"),
	PINCTRL_PIN(45, "GPIO45/nDCD1/JTDO2"),
	PINCTRL_PIN(46, "GPIO46/nDSR1/JTCK2"),
	PINCTRL_PIN(47, "GPIO47/nRI1/JCP_RDY2"),
	PINCTRL_PIN(48, "GPIO48/TXD2/BSPTXD"),
	PINCTRL_PIN(49, "GPIO49/RXD2/BSPRXD"),
	PINCTRL_PIN(50, "GPIO50/nCTS2"),
	PINCTRL_PIN(51, "GPO51/nRTS2/STRAP2"),
	PINCTRL_PIN(52, "GPIO52/nDCD2"),
	PINCTRL_PIN(53, "GPIO53/nDTR2_BOUT2/STRAP1"),
	PINCTRL_PIN(54, "GPIO54/nDSR2"),
	PINCTRL_PIN(55, "GPIO55/nRI2"),
	PINCTRL_PIN(56, "GPIO56/R1RXERR"),
	PINCTRL_PIN(57, "GPIO57/R1MDC"),
	PINCTRL_PIN(58, "GPIO58/R1MDIO"),
	PINCTRL_PIN(59, "GPIO59/SMB3DSDA"),
	PINCTRL_PIN(60, "GPIO60/SMB3DSCL"),
	PINCTRL_PIN(61, "GPO61/nDTR1_BOUT1/STRAP6"),
	PINCTRL_PIN(62, "GPO62/nRTST1/STRAP5"),
	PINCTRL_PIN(63, "GPO63/TXD1/STRAP4"),

	PINCTRL_PIN(64, "GPIO64/FANIN0"),
	PINCTRL_PIN(65, "GPIO65/FANIN1"),
	PINCTRL_PIN(66, "GPIO66/FANIN2"),
	PINCTRL_PIN(67, "GPIO67/FANIN3"),
	PINCTRL_PIN(68, "GPIO68/FANIN4"),
	PINCTRL_PIN(69, "GPIO69/FANIN5"),
	PINCTRL_PIN(70, "GPIO70/FANIN6"),
	PINCTRL_PIN(71, "GPIO71/FANIN7"),
	PINCTRL_PIN(72, "GPIO72/FANIN8"),
	PINCTRL_PIN(73, "GPIO73/FANIN9"),
	PINCTRL_PIN(74, "GPIO74/FANIN10"),
	PINCTRL_PIN(75, "GPIO75/FANIN11"),
	PINCTRL_PIN(76, "GPIO76/FANIN12"),
	PINCTRL_PIN(77, "GPIO77/FANIN13"),
	PINCTRL_PIN(78, "GPIO78/FANIN14"),
	PINCTRL_PIN(79, "GPIO79/FANIN15"),
	PINCTRL_PIN(80, "GPIO80/PWM0"),
	PINCTRL_PIN(81, "GPIO81/PWM1"),
	PINCTRL_PIN(82, "GPIO82/PWM2"),
	PINCTRL_PIN(83, "GPIO83/PWM3"),
	PINCTRL_PIN(84, "GPIO84/R2TXD0"),
	PINCTRL_PIN(85, "GPIO85/R2TXD1"),
	PINCTRL_PIN(86, "GPIO86/R2TXEN"),
	PINCTRL_PIN(87, "GPIO87/R2RXD0"),
	PINCTRL_PIN(88, "GPIO88/R2RXD1"),
	PINCTRL_PIN(89, "GPIO89/R2CRSDV"),
	PINCTRL_PIN(90, "GPIO90/R2RXERR"),
	PINCTRL_PIN(91, "GPIO91/R2MDC"),
	PINCTRL_PIN(92, "GPIO92/R2MDIO"),
	PINCTRL_PIN(93, "GPIO93/GA20/SMB5DSCL"),
	PINCTRL_PIN(94, "GPIO94/nKBRST/SMB5DSDA"),
	PINCTRL_PIN(95, "GPIO95/nLRESET/nESPIRST"),

	PINCTRL_PIN(96, "GPIO96/RG1TXD0"),
	PINCTRL_PIN(97, "GPIO97/RG1TXD1"),
	PINCTRL_PIN(98, "GPIO98/RG1TXD2"),
	PINCTRL_PIN(99, "GPIO99/RG1TXD3"),
	PINCTRL_PIN(100, "GPIO100/RG1TXC"),
	PINCTRL_PIN(101, "GPIO101/RG1TXCTL"),
	PINCTRL_PIN(102, "GPIO102/RG1RXD0"),
	PINCTRL_PIN(103, "GPIO103/RG1RXD1"),
	PINCTRL_PIN(104, "GPIO104/RG1RXD2"),
	PINCTRL_PIN(105, "GPIO105/RG1RXD3"),
	PINCTRL_PIN(106, "GPIO106/RG1RXC"),
	PINCTRL_PIN(107, "GPIO107/RG1RXCTL"),
	PINCTRL_PIN(108, "GPIO108/RG1MDC"),
	PINCTRL_PIN(109, "GPIO109/RG1MDIO"),
	PINCTRL_PIN(110, "GPIO110/RG2TXD0/DDRV0"),
	PINCTRL_PIN(111, "GPIO111/RG2TXD1/DDRV1"),
	PINCTRL_PIN(112, "GPIO112/RG2TXD2/DDRV2"),
	PINCTRL_PIN(113, "GPIO113/RG2TXD3/DDRV3"),
	PINCTRL_PIN(114, "GPIO114/SMB0SCL"),
	PINCTRL_PIN(115, "GPIO115/SMB0SDA"),
	PINCTRL_PIN(116, "GPIO116/SMB1SCL"),
	PINCTRL_PIN(117, "GPIO117/SMB1SDA"),
	PINCTRL_PIN(118, "GPIO118/SMB2SCL"),
	PINCTRL_PIN(119, "GPIO119/SMB2SDA"),
	PINCTRL_PIN(120, "GPIO120/SMB2CSDA"),
	PINCTRL_PIN(121, "GPIO121/SMB2CSCL"),
	PINCTRL_PIN(122, "GPIO122/SMB2BSDA"),
	PINCTRL_PIN(123, "GPIO123/SMB2BSCL"),
	PINCTRL_PIN(124, "GPIO124/SMB1CSDA"),
	PINCTRL_PIN(125, "GPIO125/SMB1CSCL"),
	PINCTRL_PIN(126, "GPIO126/SMB1BSDA"),
	PINCTRL_PIN(127, "GPIO127/SMB1BSCL"),

	PINCTRL_PIN(128, "GPIO128/SMB8SCL"),
	PINCTRL_PIN(129, "GPIO129/SMB8SDA"),
	PINCTRL_PIN(130, "GPIO130/SMB9SCL"),
	PINCTRL_PIN(131, "GPIO131/SMB9SDA"),
	PINCTRL_PIN(132, "GPIO132/SMB10SCL"),
	PINCTRL_PIN(133, "GPIO133/SMB10SDA"),
	PINCTRL_PIN(134, "GPIO134/SMB11SCL"),
	PINCTRL_PIN(135, "GPIO135/SMB11SDA"),
	PINCTRL_PIN(136, "GPIO136/SD1DT0"),
	PINCTRL_PIN(137, "GPIO137/SD1DT1"),
	PINCTRL_PIN(138, "GPIO138/SD1DT2"),
	PINCTRL_PIN(139, "GPIO139/SD1DT3"),
	PINCTRL_PIN(140, "GPIO140/SD1CLK"),
	PINCTRL_PIN(141, "GPIO141/SD1WP"),
	PINCTRL_PIN(142, "GPIO142/SD1CMD"),
	PINCTRL_PIN(143, "GPIO143/SD1CD/SD1PWR"),
	PINCTRL_PIN(144, "GPIO144/PWM4"),
	PINCTRL_PIN(145, "GPIO145/PWM5"),
	PINCTRL_PIN(146, "GPIO146/PWM6"),
	PINCTRL_PIN(147, "GPIO147/PWM7"),
	PINCTRL_PIN(148, "GPIO148/MMCDT4"),
	PINCTRL_PIN(149, "GPIO149/MMCDT5"),
	PINCTRL_PIN(150, "GPIO150/MMCDT6"),
	PINCTRL_PIN(151, "GPIO151/MMCDT7"),
	PINCTRL_PIN(152, "GPIO152/MMCCLK"),
	PINCTRL_PIN(153, "GPIO153/MMCWP"),
	PINCTRL_PIN(154, "GPIO154/MMCCMD"),
	PINCTRL_PIN(155, "GPIO155/nMMCCD/nMMCRST"),
	PINCTRL_PIN(156, "GPIO156/MMCDT0"),
	PINCTRL_PIN(157, "GPIO157/MMCDT1"),
	PINCTRL_PIN(158, "GPIO158/MMCDT2"),
	PINCTRL_PIN(159, "GPIO159/MMCDT3"),

	PINCTRL_PIN(160, "GPIO160/CLKOUT/RNGOSCOUT"),
	PINCTRL_PIN(161, "GPIO161/nLFRAME/nESPICS"),
	PINCTRL_PIN(162, "GPIO162/SERIRQ"),
	PINCTRL_PIN(163, "GPIO163/LCLK/ESPICLK"),
	PINCTRL_PIN(164, "GPIO164/LAD0/ESPI_IO0"/*dscnt6*/),
	PINCTRL_PIN(165, "GPIO165/LAD1/ESPI_IO1"/*dscnt6*/),
	PINCTRL_PIN(166, "GPIO166/LAD2/ESPI_IO2"/*dscnt6*/),
	PINCTRL_PIN(167, "GPIO167/LAD3/ESPI_IO3"/*dscnt6*/),
	PINCTRL_PIN(168, "GPIO168/nCLKRUN/nESPIALERT"),
	PINCTRL_PIN(169, "GPIO169/nSCIPME"),
	PINCTRL_PIN(170, "GPIO170/nSMI"),
	PINCTRL_PIN(171, "GPIO171/SMB6SCL"),
	PINCTRL_PIN(172, "GPIO172/SMB6SDA"),
	PINCTRL_PIN(173, "GPIO173/SMB7SCL"),
	PINCTRL_PIN(174, "GPIO174/SMB7SDA"),
	PINCTRL_PIN(175, "GPIO175/PSPI1CK/FANIN19"),
	PINCTRL_PIN(176, "GPIO176/PSPI1DO/FANIN18"),
	PINCTRL_PIN(177, "GPIO177/PSPI1DI/FANIN17"),
	PINCTRL_PIN(178, "GPIO178/R1TXD0"),
	PINCTRL_PIN(179, "GPIO179/R1TXD1"),
	PINCTRL_PIN(180, "GPIO180/R1TXEN"),
	PINCTRL_PIN(181, "GPIO181/R1RXD0"),
	PINCTRL_PIN(182, "GPIO182/R1RXD1"),
	PINCTRL_PIN(183, "GPIO183/SPI3CK"),
	PINCTRL_PIN(184, "GPO184/SPI3D0/STRAP9"),
	PINCTRL_PIN(185, "GPO185/SPI3D1/STRAP10"),
	PINCTRL_PIN(186, "GPIO186/nSPI3CS0"),
	PINCTRL_PIN(187, "GPIO187/nSPI3CS1"),
	PINCTRL_PIN(188, "GPIO188/SPI3D2/nSPI3CS2"),
	PINCTRL_PIN(189, "GPIO189/SPI3D3/nSPI3CS3"),
	PINCTRL_PIN(190, "GPIO190/nPRD_SMI"),
	PINCTRL_PIN(191, "GPIO191"),

	PINCTRL_PIN(192, "GPIO192"),
	PINCTRL_PIN(193, "GPIO193/R1CRSDV"),
	PINCTRL_PIN(194, "GPIO194/SMB0BSCL"),
	PINCTRL_PIN(195, "GPIO195/SMB0BSDA"),
	PINCTRL_PIN(196, "GPIO196/SMB0CSCL"),
	PINCTRL_PIN(197, "GPIO197/SMB0DEN"),
	PINCTRL_PIN(198, "GPIO198/SMB0DSDA"),
	PINCTRL_PIN(199, "GPIO199/SMB0DSCL"),
	PINCTRL_PIN(200, "GPIO200/R2CK"),
	PINCTRL_PIN(201, "GPIO201/R1CK"),
	PINCTRL_PIN(202, "GPIO202/SMB0CSDA"),
	PINCTRL_PIN(203, "GPIO203/FANIN16"),
	PINCTRL_PIN(204, "GPIO204/DDC2SCL"),
	PINCTRL_PIN(205, "GPIO205/DDC2SDA"),
	PINCTRL_PIN(206, "GPIO206/HSYNC2"),
	PINCTRL_PIN(207, "GPIO207/VSYNC2"),
	PINCTRL_PIN(208, "GPIO208/RG2TXC/DVCK"),
	PINCTRL_PIN(209, "GPIO209/RG2TXCTL/DDRV4"),
	PINCTRL_PIN(210, "GPIO210/RG2RXD0/DDRV5"),
	PINCTRL_PIN(211, "GPIO211/RG2RXD1/DDRV6"),
	PINCTRL_PIN(212, "GPIO212/RG2RXD2/DDRV7"),
	PINCTRL_PIN(213, "GPIO213/RG2RXD3/DDRV8"),
	PINCTRL_PIN(214, "GPIO214/RG2RXC/DDRV9"),
	PINCTRL_PIN(215, "GPIO215/RG2RXCTL/DDRV10"),
	PINCTRL_PIN(216, "GPIO216/RG2MDC/DDRV11"),
	PINCTRL_PIN(217, "GPIO217/RG2MDIO/DVHSYNC"),
	PINCTRL_PIN(218, "GPIO218/nWDO1"),
	PINCTRL_PIN(219, "GPIO219/nWDO2"),
	PINCTRL_PIN(220, "GPIO220/SMB12SCL"),
	PINCTRL_PIN(221, "GPIO221/SMB12SDA"),
	PINCTRL_PIN(222, "GPIO222/SMB13SCL"),
	PINCTRL_PIN(223, "GPIO223/SMB13SDA"),

	PINCTRL_PIN(224, "GPIO224/SPIXCK"),
	PINCTRL_PIN(225, "GPO225/SPIXD0/STRAP12"),
	PINCTRL_PIN(226, "GPO226/SPIXD1/STRAP13"),
	PINCTRL_PIN(227, "GPIO227/nSPIXCS0"),
	PINCTRL_PIN(228, "GPIO228/nSPIXCS1"),
	PINCTRL_PIN(229, "GPIO229/SPIXD2/STRAP3"),
	PINCTRL_PIN(230, "GPIO230/SPIXD3"),
	PINCTRL_PIN(231, "GPIO231/nCLKREQ"),
	PINCTRL_PIN(255, "GPI255/DACOSEL"),
};

static const char *gcr_regname(int reg)
{
	switch (reg) {
	case GCR_MFSEL1: return "MFSEL1";
	case GCR_MFSEL2: return "MFSEL2";
	case GCR_MFSEL3: return "MFSEL3";
	case GCR_MFSEL4: return "MFSEL4";
	case GCR_I2CSEGSEL: return "I2CSEGSEL";
	case GCR_FLOCKR1: return "FLOCKR1";
	}
	return "xx";
}

#ifdef DEBUG
static void showmfsel(void)
{
	int module;
	void *base;

	for (module = 0; module < 8; module++) {
		base = npcm_bank[module].base;
		pr_info("-- module %d [gpio%d - %d]\n", module,
			   module * GPIO_PER_BANK,
			   (module * GPIO_PER_BANK) + GPIO_PER_BANK - 1);
		pr_info("DIN :%.8x DOUT:%.8x IE  :%.8x OE	 :%.8x\n",
			   npcm_read32(base + GPnDIN),
			   npcm_read32(base + GPnDOUT),
			   npcm_read32(base + GPnIEM),
			   npcm_read32(base + GPnOE));
		pr_info("PU  :%.8x PD  :%.8x DB  :%.8x POL :%.8x\n",
			   npcm_read32(base + GPnPU),
			   npcm_read32(base + GPnPD),
			   npcm_read32(base + GPnDBNC),
			   npcm_read32(base + GPnPOL));
		pr_info("ETYP:%.8x EVBE:%.8x EVEN:%.8x EVST:%.8x\n",
			   npcm_read32(base + GPnEVTYP),
			   npcm_read32(base + GPnEVBE),
			   npcm_read32(base + GPnEVEN),
			   npcm_read32(base + GPnEVST));
		pr_info("OTYP:%.8x OSRC:%.8x ODSC:%.8x\n",
			   npcm_read32(base + GPnOTYP),
			   npcm_read32(base + GPnOSRC),
			   npcm_read32(base + GPnODSC));
		pr_info("OBL0:%.8x OBL1:%.8x OBL2:%.8x OBL3:%.8x\n",
			   npcm_read32(base + GPnOBL0),
			   npcm_read32(base + GPnOBL1),
			   npcm_read32(base + GPnOBL2),
			   npcm_read32(base + GPnOBL3));
		pr_info("SLCK:%.8x MLCK:%.8x\n",
			   npcm_read32(base + GPnSPLCK),
			   npcm_read32(base + GPnMPLCK));
	}
	pr_info("-- GCR\n");
	base = npcm_bank[GCR_BANK].base;
	pr_info("MFSEL1:%.8x MFSEL2:%.8x MFSEL3:%.8x MFSEL4:%.8x I2C:%.8x FLOCKR1:%.8x\n",
		npcm_read32(base + GCR_MFSEL1),
		npcm_read32(base + GCR_MFSEL2),
		npcm_read32(base + GCR_MFSEL3),
		npcm_read32(base + GCR_MFSEL4),
		npcm_read32(base + GCR_I2CSEGSEL),
		npcm_read32(base + GCR_FLOCKR1));
}
#endif

static void npcm_setmode(struct npcm_reg *bank, int reg, int bit, int mode, int pin)
{
	if (reg) {
		pr_debug("Pin %d: setting reg:%x[%s.%d] = %d\n", pin,
		       reg, gcr_regname(reg), bit, mode);
		npcm_rmw32(bank->base + reg, bit, 1, mode);
	}
}

/* Enable mode in pin group */
static void npcm_setfunc(int pin, int n, int mode)
{
	struct npcm_reg *bank = &npcm_bank[GCR_BANK];
	const struct npcm_pincfg *cfg;
	unsigned long flags;

	spin_lock_irqsave(&bank->lock, flags);
	while (n--) {
		cfg = &pincfg[pin++];
		if (mode == fn_gpio || cfg->fn0 == mode || cfg->fn1 == mode || cfg->fn2 == mode) {
			npcm_setmode(bank, cfg->reg0, cfg->bit0, !!(cfg->fn0 == mode), pin-1);
			npcm_setmode(bank, cfg->reg1, cfg->bit1, !!(cfg->fn1 == mode), pin-1);
			npcm_setmode(bank, cfg->reg2, cfg->bit2, !!(cfg->fn2 == mode), pin-1);
		}
	}
	spin_unlock_irqrestore(&bank->lock, flags);
}

/* Get slew rate of pin (high/low) */
static int npcm_get_slew_rate(unsigned int pin)
{
	if (pincfg[pin].flag & SLEW)
		return gpio_bitop(opGETBIT, pin, GPnOSRC);
	/* LPC Slew rate in SRCNT register */
	if (pincfg[pin].flag & SLEWLPC)
		return !!(npcm_read32(npcm_bank[GCR_BANK].base + GCR_SRCNT) & SRCNT_ESPI);
	return -EINVAL;
}

/* Get drive strength for a pin, if supported */
static int npcm_get_drive_strength(unsigned int pin)
{
	u32 val, ds = 0;
	int flg;

	flg = pincfg[pin].flag;
	if (flg & DRIVE_STRENGTH_MASK) {
		/* Get standard reading */
		val = gpio_bitop(opGETBIT, pin, GPnODSC);
		ds = val ? DSHI(flg) : DSLO(flg);
	}
	DRV_MSG2(" pin %d strength %d = %d\n", pin, val, ds);
	return ds;
}

/* Set drive strength for a pin, if supported */
static int npcm_set_drive_strength(unsigned int pin, int nval)
{
	int v;

	v = (pincfg[pin].flag & DRIVE_STRENGTH_MASK);
	if (!nval || !v)
		return 0;
	if (DSLO(v) == nval) {
		DRV_MSG2(" setting pin %d to low strength [%d]\n", pin, nval);
		gpio_bitop(opCLRBIT, pin, GPnODSC);
		return 1;
	} else if (DSHI(v) == nval) {
		DRV_MSG2(" setting pin %d to high strength [%d]\n", pin, nval);
		gpio_bitop(opSETBIT, pin, GPnODSC);
		return 1;
	}
	return 0;
}

/* ================= pinctrl_ops ========================= */
static void npcm_pin_dbg_show(struct pinctrl_dev *pctldev,
			      struct seq_file *s, unsigned int offset)
{
	seq_printf(s, "pinctrl_ops.dbg: %d", offset);
}

static int npcm_get_groups_count(struct pinctrl_dev *pctldev)
{
	DRV_MSG2("group size: %d\n", ARRAY_SIZE(npcm_groups));
	return ARRAY_SIZE(npcm_groups);
}

static const char *npcm_get_group_name(struct pinctrl_dev *pctldev,
				       unsigned int selector)
{
	return npcm_groups[selector].name;
}

static int npcm_get_group_pins(struct pinctrl_dev *pctldev,
			       unsigned int selector,
			       const unsigned int **pins,
			       unsigned int *npins)
{
	*npins = npcm_groups[selector].npins;
	*pins  = npcm_groups[selector].pins;
	return 0;
}

static int npcm_dt_node_to_map(struct pinctrl_dev *pctldev,
			       struct device_node *np_config,
			       struct pinctrl_map **map,
			       u32 *num_maps)
{
	pr_debug("dt_node_to_map: %s\n", np_config->name);
	return pinconf_generic_dt_node_to_map(pctldev, np_config,
					      map, num_maps,
					      PIN_MAP_TYPE_INVALID);
}

static void npcm_dt_free_map(struct pinctrl_dev *pctldev,
			     struct pinctrl_map *map, u32 num_maps)
{
	kfree(map);
}

static struct pinctrl_ops npcm_pinctrl_ops = {
	.get_groups_count = npcm_get_groups_count,
	.get_group_name = npcm_get_group_name,
	.get_group_pins = npcm_get_group_pins,
	.pin_dbg_show = npcm_pin_dbg_show,
	.dt_node_to_map = npcm_dt_node_to_map,
	.dt_free_map = npcm_dt_free_map,
};

/* ================= pinmux_ops ========================= */
static int npcm_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(npcm_funcs);
}

static const char *npcm_get_function_name(struct pinctrl_dev *pctldev,
					  unsigned int function)
{
	return npcm_funcs[function].name;
}

static int npcm_get_function_groups(struct pinctrl_dev *pctldev,
				    unsigned int function,
				    const char * const **groups,
				    unsigned int * const ngroups)
{
	*ngroups = npcm_funcs[function].ngroups;
	*groups	 = npcm_funcs[function].groups;
	return 0;
}

static int npcm_pinmux_set_mux(struct pinctrl_dev *pctldev, unsigned int function,
			       unsigned int group)
{
	struct npcm_dev *npcm = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(npcm->gc.parent, "set_mux: %d, %d[%s]\n", function, group, npcm_groups[group].name);
	npcm_setfunc(0, ARRAY_SIZE(pincfg), group);
	return 0;
}

static int npcm_gpio_request_enable(struct pinctrl_dev *pctldev,
				    struct pinctrl_gpio_range *range,
				    unsigned int offset)
{
	struct npcm_dev *npcm = pinctrl_dev_get_drvdata(pctldev);

	if (!range) {
		dev_err(npcm->gc.parent, "invalid range\n");
		return -EINVAL;
	}
	if (!range->gc) {
		dev_err(npcm->gc.parent, "invalid gpiochip\n");
		return -EINVAL;
	}
	dev_dbg(npcm->gc.parent, "Enable GPIO %d\n", offset);
	npcm_setfunc(offset, 1, fn_gpio);
	return 0;
}

/* Release GPIO back to pinctrl mode */
static void npcm_gpio_request_free(struct pinctrl_dev *pctldev,
				   struct pinctrl_gpio_range *range,
				   unsigned int offset)
{
	struct npcm_dev *npcm = pinctrl_dev_get_drvdata(pctldev);
	int virq;

	virq = irq_find_mapping(npcm->domain, offset);
	DRV_MSG2("Free GPIO %d, irq=%d\n", offset, virq);
	if (virq)
		irq_dispose_mapping(virq);
}

/* Set GPIO direction */
static int npcm_gpio_set_direction(struct pinctrl_dev *pctldev,
				   struct pinctrl_gpio_range *range,
				   unsigned int offset, bool input)
{
	DRV_MSG2("GPIO Set Direction: %d = %d\n", offset, input);
	if (input) {
		gpio_bitop(opSET, offset, GPnOEC);
		gpio_bitop(opSETBIT, offset, GPnIEM);
	} else {
		gpio_bitop(opCLRBIT, offset, GPnIEM);
		gpio_bitop(opSET, offset, GPnOES);
	}
	return 0;
}

static struct pinmux_ops npcm_pinmux_ops = {
#if 0
	.request(struct pinctrl_dev *, unsigned int);
	.free(struct pinctrl_dev *, unsigned int);
#endif
	.get_functions_count = npcm_get_functions_count,
	.get_function_name = npcm_get_function_name,
	.get_function_groups = npcm_get_function_groups,

	.set_mux = npcm_pinmux_set_mux,

	.gpio_request_enable = npcm_gpio_request_enable,
	.gpio_disable_free = npcm_gpio_request_free,
	.gpio_set_direction = npcm_gpio_set_direction,
};

/* ================= pinconf_ops ========================= */

/* Get configuration setting for a pin */
static int npcm_config_get(struct pinctrl_dev *pctldev, unsigned int pin,
			   unsigned long *config)
{
	enum pin_config_param param = pinconf_to_config_param(*config);
	u32 ie, oe, pu, pd;
	int rc;

	rc = 0;
	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_UP:
	case PIN_CONFIG_BIAS_PULL_DOWN:
		pu = gpio_bitop(opGETBIT, pin, GPnPU);
		pd = gpio_bitop(opGETBIT, pin, GPnPD);
		if (param == PIN_CONFIG_BIAS_DISABLE)
			rc = (!pu && !pd);
		else if (param == PIN_CONFIG_BIAS_PULL_UP)
			rc = (pu && !pd);
		else if (param == PIN_CONFIG_BIAS_PULL_DOWN)
			rc = (!pu && pd);
		break;
	case PIN_CONFIG_OUTPUT:
	case PIN_CONFIG_INPUT_ENABLE:
		ie = gpio_bitop(opGETBIT, pin, GPnIEM);
		oe = gpio_bitop(opGETBIT, pin, GPnOE);
		if (param == PIN_CONFIG_INPUT_ENABLE)
			rc = (ie && !oe);
		else if (param == PIN_CONFIG_OUTPUT)
			rc = (!ie && oe);
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		rc = !gpio_bitop(opGETBIT, pin, GPnOTYP);
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		rc = gpio_bitop(opGETBIT, pin, GPnOTYP);
		break;
	case PIN_CONFIG_INPUT_DEBOUNCE:
		rc = gpio_bitop(opGETBIT, pin, GPnDBNC);
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		rc = npcm_get_drive_strength(pin);
		if (rc)
			*config = pinconf_to_config_packed(param, rc * 1000);
		break;
	case PIN_CONFIG_SLEW_RATE:
		rc = npcm_get_slew_rate(pin);
		if (rc >= 0)
			*config = pinconf_to_config_packed(param, rc);
		break;
	default:
		return -EINVAL;
	}
	if (!rc)
		return -EINVAL;
	return 0;
}

/* Set configuration setting for a pin */
static int npcm_config_set_one(struct npcm_dev *npcm, unsigned int pin,
			       unsigned long config)
{
	enum pin_config_param param = pinconf_to_config_param(config);
	u16 arg = pinconf_to_config_argument(config);
	int rc;

	dev_dbg(npcm->gc.parent, "param=%d %d[GPIO]\n", param, pin);
	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		npcm_setfunc(pin, 1, fn_gpio);
		gpio_bitop(opCLRBIT, pin, GPnPU);
		gpio_bitop(opCLRBIT, pin, GPnPD);
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		/* arg: 0=GND, !0=enabled */
		npcm_setfunc(pin, 1, fn_gpio);
		gpio_bitop(opCLRBIT, pin, GPnPU);
		gpio_bitop(opSETBIT, pin, GPnPD);
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		/* arg: 0=VDD, !0=enabled */
		npcm_setfunc(pin, 1, fn_gpio);
		gpio_bitop(opCLRBIT, pin, GPnPD);
		gpio_bitop(opSETBIT, pin, GPnPU);
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		/* arg: 0=disable, 1=enable */
		npcm_setfunc(pin, 1, fn_gpio);
		gpio_bitop(opSET, pin, GPnOEC);
		gpio_bitop(opSETBIT, pin, GPnIEM);
		break;
	case PIN_CONFIG_OUTPUT:
		/* arg: 0=low, 1=high */
		npcm_setfunc(pin, 1, fn_gpio);
		gpio_bitop(opCLRBIT, pin, GPnIEM);
		gpio_bitop(opSET, pin, arg ? GPnDOS : GPnDOC);
		gpio_bitop(opSET, pin, GPnOES);
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		npcm_setfunc(pin, 1, fn_gpio);
		gpio_bitop(opCLRBIT, pin, GPnOTYP);
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		npcm_setfunc(pin, 1, fn_gpio);
		gpio_bitop(opSETBIT, pin, GPnOTYP);
		break;
	case PIN_CONFIG_INPUT_DEBOUNCE:
		npcm_setfunc(pin, 1, fn_gpio);
		gpio_bitop(opSETBIT, pin, GPnDBNC);
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		/* arg is mA */
		npcm_setfunc(pin, 1, fn_gpio);
		rc = npcm_set_drive_strength(pin, arg / 1000);
		if (!rc)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/* Set multiple configuration settings for a pin */
static int npcm_config_set(struct pinctrl_dev *pctldev, unsigned int pin,
			   unsigned long *configs,
			   unsigned int num_configs)
{
	struct npcm_dev *npcm = pinctrl_dev_get_drvdata(pctldev);
	int rc;

	while (num_configs--) {
		rc = npcm_config_set_one(npcm, pin, *configs++);
		if (rc)
			return rc;
	}
	return 0;
}

static void npcm_config_dbg_show(struct pinctrl_dev *pctldev,
				 struct seq_file *s,
				 unsigned int offset)
{
}

static void npcm_config_group_dbg_show(struct pinctrl_dev *pctldev,
				       struct seq_file *s,
				       unsigned int selector)
{
}

static void npcm_config_config_dbg_show(struct pinctrl_dev *pctldev,
					struct seq_file *s,
					unsigned long config)
{
}

static struct pinconf_ops npcm_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = npcm_config_get,
	.pin_config_set = npcm_config_set,
	.pin_config_dbg_show = npcm_config_dbg_show,
	.pin_config_group_dbg_show = npcm_config_group_dbg_show,
	.pin_config_config_dbg_show = npcm_config_config_dbg_show,
};

/* ================= pinctrl_desc ========================= */
static struct pinctrl_desc npcm_desc = {
	.name = "npcm",
	.pins = npcm_pins,
	.npins = ARRAY_SIZE(npcm_pins),
	.pctlops = &npcm_pinctrl_ops,
	.pmxops = &npcm_pinmux_ops,
	.confops = &npcm_pinconf_ops,
	.owner = THIS_MODULE,
};

static struct gpio_chip npcm_gc = {
	.owner		= THIS_MODULE,
	.label		= "npcmgpio",
	.request	= npcmgpio_gpio_request,
	.free		= npcmgpio_gpio_free,
	.get_direction	= npcmgpio_get_direction,
	.direction_input = npcmgpio_direction_input,
	.direction_output = npcmgpio_direction_output,
#if (KERNEL_VERSION(4, 10, 17) >= LINUX_VERSION_CODE)
	.set_debounce	= npcmgpio_set_debounce,
#endif
	.get		= npcmgpio_get_value,
	.set		= npcmgpio_set_value,
	.dbg_show	= npcmgpio_dbg_show,
	.to_irq		= npcmgpio_to_irq,
#if ((KERNEL_VERSION(4, 7, 0) <= LINUX_VERSION_CODE) && (KERNEL_VERSION(4, 10, 17) >= LINUX_VERSION_CODE))
	.set_single_ended = npcmgpio_set_single_ended,
#endif
	.ngpio		= 256,
};

static int __init npcm_pin_init(void)
{
	int rc, i;

	if (!npcm.pdev) {
		npcm.pdev = platform_device_alloc("npcm", 0);
		if (!npcm.pdev)
			return -ENOMEM;
		rc = platform_device_add(npcm.pdev);
		if (rc) {
			platform_device_put(npcm.pdev);
			return -ENODEV;
		}
	}
	npcm.gc = npcm_gc;
	npcm.gc.parent = &npcm.pdev->dev;

	if (npcm_init_bank(GCR_BANK, GCR_BA, GCR_BANKSIZE, 0, NULL))
		return -EINVAL;
	/* Setup bank registers */
	for (i = 0; i < GPIO_NBANKS; i++) {
		if (npcm_init_bank(i, GPIO_BA + (i * GPIO_BANKSIZE),
				   GPIO_BANKSIZE, i,
				   "nuvoton,npcm750-gpio"))
			return -EINVAL;
		npcm_bank[i].irqbase = i * GPIO_PER_BANK;
#ifdef GPIO_DRIVER
		irq_set_irq_type(npcm_bank[i].irq, IRQ_TYPE_LEVEL_HIGH);
		irq_set_handler_data(npcm_bank[i].irq, &npcm_bank[i]);
		irq_set_chained_handler(npcm_bank[i].irq, npcmgpio_irq_handler);
#endif
	}
	npcm.pctl = pinctrl_register(&npcm_desc, npcm.gc.parent, &npcm);
	if (!npcm.pctl) {
		platform_device_del(npcm.pdev);
		platform_device_put(npcm.pdev);
		DRV_MSG2("could not register foo pin driver\n");
		return -EINVAL;
	}
#ifdef GPIO_DRIVER
	rc = gpiochip_add(&npcm.gc);
	if (rc) {
		DRV_MSG2("gpiochip_add fails\n");
		return rc;
	}
	rc = gpiochip_add_pin_range(&npcm.gc, dev_name(npcm.gc.parent),
				    0, 0, npcm.gc.ngpio);
	if (rc) {
		DRV_MSG2("gpiochip_add_pin_range fails\n");
		return rc;
	}
	npcm.domain = irq_domain_add_linear(NULL, npcm.gc.ngpio,
					    &npcmgpio_domain_ops,
					    &npcmgpio_irqchip);
	if (!npcm.domain) {
		DRV_MSG2("irqdomain_add fails\n");
		return -ENOMEM;
	}
#endif
	/* pinctrl_select_state(..., state); */
	pr_info("Nuvoton Pinctrl driver version %s [%s]\n", DRV_VERSION, DRV_DATE);
	return 0;
}

static void npcm_pin_exit(void)
{
	int i;

#ifdef GPIO_DRIVER
	irq_domain_remove(npcm.domain);
	gpiochip_remove(&npcm.gc);
#endif
	for (i = 0; i < GPIO_NBANKS; i++) {
#ifdef GPIO_DRIVER
		irq_set_chained_handler(npcm_bank[i].irq, NULL);
#endif
		iounmap(npcm_bank[i].base);
	}
	iounmap(npcm_bank[GCR_BANK].base);
	pinctrl_unregister(npcm.pctl);
	platform_device_unregister(npcm.pdev);
}

static int npcm_pinctrl_probe(struct platform_device *pdev)
{
	npcm.pdev = pdev;
	npcm_pin_init();
	return 0;
}

static int npcm_pinctrl_remove(struct platform_device *pdev)
{
	npcm_pin_exit();
	return 0;
}

static const struct of_device_id npcm_pinctrl_match[] = {
	{ .compatible = "nuvoton,npcmx50-pinctrl" },
	{ },
};

MODULE_DEVICE_TABLE(of, npcm_pinctrl_match);

static struct platform_driver npcm_pinctrl_driver = {
	.probe = npcm_pinctrl_probe,
	.remove = npcm_pinctrl_remove,
	.driver = {
		.name = "npcm-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(npcm_pinctrl_match),
	},
};

module_platform_driver(npcm_pinctrl_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jordan_hargrave@dell.com");
MODULE_VERSION(DRV_VERSION);
MODULE_DESCRIPTION("Provide Pinctrl/GPIO methods for NPCM7XX");
