/*
 * Support for Nuvoton UART HW.
 *
 * Copyright (c) 2014-2017 Nuvoton Technology corporation.
 *
 * Released under the GPLv2 only.
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/spinlock_types.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>

static struct regmap *gcr_regmap;

#define  MFSEL1_OFFSET 0x00C
#define  MFSEL4_OFFSET 0x0B0
#define  SPSWC_OFFSET  0x038

#define MSN(parm8)	((u8)((u8)(parm8) >> 4))
#define LSN(parm8)	((u8)((u8)parm8 & 0x0F))
#define MSB(parm16)	((u8)((u16)(parm16) >> 8))
#define LSB(parm16)	((u8)(parm16))
#define MSW(parm32)	((u16)((u32)(parm32) >> 16))
#define LSW(parm32)	((u16)(parm32))
#define MSD(parm64)	((u32)((u64)(parm64) >> 32))
#define LSD(parm64)	((u32)(parm64))
#define PORT_NPCM	101

/* Debug messages */
#ifdef DEBUG_NPCM_UART
#define NPCM_SERIAL_MSG(format, arg...) pr_notice(format, ## arg)
#else
#define NPCM_SERIAL_MSG(format, arg...) do { } while (0)
#endif

#define NPCM_UART_NUM_OF_MODULES	4

#define _1Hz_	1UL
#define _1KHz_	(1000*_1Hz_)
#define _1MHz_	(1000*_1KHz_)
#define _1GHz_	(1000*_1MHz_)

typedef struct bit_field {
	u8 offset;
	u8 size;
} bit_field_t;

/* UART REGS */

/* Receive Buffer Register (RBR) */
#define UART_RBR(n)	(npcm_serial_ports[n].port.membase + 0x0)
static const bit_field_t RBR_Transmit = { 0, 8 };

/* Transmit Holding Register (THR) */
#define UART_THR(n)	(npcm_serial_ports[n].port.membase + 0x0)

/* Interrupt Enable Register (IER) */
#define UART_IER(n)	(npcm_serial_ports[n].port.membase + 0x4)
static const bit_field_t IER_nDBGACK_EN =	{ 4, 1 };
static const bit_field_t IER_MSIE =		{ 3, 1 };
static const bit_field_t IER_RLSIE =		{ 2, 1 };
static const bit_field_t IER_THREIE =		{ 1, 1 };
static const bit_field_t IER_RDAIE =		{ 0, 1 };

/* Divisor Latch (Low Byte) Register (DLL) */
#define UART_DLL(n)	(npcm_serial_ports[n].port.membase + 0x0)

/* Divisor Latch (High Byte) Register (DLM) */
#define UART_DLM(n)	(npcm_serial_ports[n].port.membase + 0x4)
static const bit_field_t DLM_Baud =	{ 0, 8 };

/* Interrupt Identification Register (IIR) */
#define UART_IIR(n)	(npcm_serial_ports[n].port.membase + 0x8)
static const bit_field_t IIR_FMES =	{ 7, 1 };
static const bit_field_t IIR_RFTLS =	{ 5, 2 };
static const bit_field_t IIR_DMS =	{ 4, 1 };
static const bit_field_t IIR_IID =	{ 1, 3 };
static const bit_field_t IIR_NIP =	{ 0, 1 };

/* FIFO Control Register (FCR) */
#define UART_FCR(n)	(npcm_serial_ports[n].port.membase + 0x8)
static const bit_field_t FCR_RFITL =      { 4, 4 };
static const bit_field_t FCR_DMS =        { 3, 1 };
static const bit_field_t FCR_TFR =        { 2, 1 };
static const bit_field_t FCR_RFR =        { 1, 1 };
static const bit_field_t FCR_FME =        { 0, 1 };

/* Line Control Register (LCR) */
#define UART_LCR(n)	(npcm_serial_ports[n].port.membase + 0xc)
static const bit_field_t LCR_DLAB =       { 7, 1 };
static const bit_field_t LCR_BCB =        { 6, 1 };
static const bit_field_t LCR_SPE =        { 5, 1 };
static const bit_field_t LCR_EPE =        { 4, 1 };
static const bit_field_t LCR_PBE =        { 3, 1 };
static const bit_field_t LCR_NSB =        { 2, 1 };
static const bit_field_t LCR_WLS =        { 0, 2 };
static const bit_field_t LCR_Word =       { 10, 1 };

/* Modem Control Register (MCR) */
#define UART_MCR(n)	(npcm_serial_ports[n].port.membase + 0x10)
static const bit_field_t MCR_LBME =       { 4, 1 };
static const bit_field_t MCR_OUT2 =       { 3, 1 };
static const bit_field_t MCR_RTS =        { 1, 1 };
static const bit_field_t MCR_DTR =        { 0, 1 };

/* Line Status Control Register (LSR) */
#define UART_LSR(n)	(npcm_serial_ports[n].port.membase + 0x14)
static const bit_field_t LSR_ERR_Rx =     { 7, 1 };
static const bit_field_t LSR_TE =         { 6, 1 };
static const bit_field_t LSR_THRE =       { 5, 1 };
static const bit_field_t LSR_BII =        { 4, 1 };
static const bit_field_t LSR_FEI =        { 3, 1 };
static const bit_field_t LSR_PEI =        { 2, 1 };
static const bit_field_t LSR_OEI =        { 1, 1 };
static const bit_field_t LSR_RFDR =       { 0, 1 };

/* Modem Status Register (MSR) */
#define UART_MSR(n)	(npcm_serial_ports[n].port.membase + 0x18)
static const bit_field_t MSR_DCD =        { 7, 1 };
static const bit_field_t MSR_RI =         { 6, 1 };
static const bit_field_t MSR_DSR =        { 5, 1 };
static const bit_field_t MSR_CTS =        { 4, 1 };
static const bit_field_t MSR_DDCD =       { 3, 1 };
static const bit_field_t MSR_DRI =        { 2, 1 };
static const bit_field_t MSR_DDSR =       { 1, 1 };
static const bit_field_t MSR_DCTS =       { 0, 1 };

/* Timeout Register (TOR) */
#define UART_TOR(n)	(npcm_serial_ports[n].port.membase + 0x1C)
static const bit_field_t TOR_TOIE =       { 7, 1 };
static const bit_field_t TOR_TOIC =       { 0, 7 };

enum FCR_RFITL_type {
	FCR_RFITL_1B    = 0x0,
	FCR_RFITL_4B    = 0x4,
	FCR_RFITL_8B    = 0x8,
	FCR_RFITL_14B   = 0xC,
};

enum LCR_WLS_type {
	LCR_WLS_5bit    = 0x0,
	LCR_WLS_6bit    = 0x1,
	LCR_WLS_7bit    = 0x2,
	LCR_WLS_8bit    = 0x3,
};

enum IIR_IID_type {
	IIR_IID_MODEM = 0x0,
	IIR_IID_THRE  = 0x1,
	IIR_IID_TOUT  = 0x5,
	IIR_IID_RDA   = 0x2,
	IIR_IID_RLS   = 0x3,
};

/* UART ports definition */
typedef enum {
	NPCM_UART0_DEV = 0,
	NPCM_UART1_DEV = 1,
	NPCM_UART2_DEV = 2,
	NPCM_UART3_DEV = 3,
} NPCM_UART_DEV_T;

#ifdef CONFIG_MACH_NPCM650
/* Uart Mux modes definitions */
enum {
	NPCM_UART_MUX_CORE_SNOOP        = 0,
	NPCM_UART_MUX_CORE_TAKEOVER     = 1,
	NPCM_UART_MUX_CORE_SP2__SP1_SI1 = 2,
	NPCM_UART_MUX_CORE_SP2__SP1_SI2 = 3,
	NPCM_UART_MUX_SKIP_CONFIG       = 4,
} NPCM_UART_MUX_T;
#endif

#ifdef CONFIG_ARCH_NPCM7XX
/*
 * Uart Mux modes definitions. These numbers match the register field value.
 * Do not change !
 * s == snoop
 */
typedef enum NPCM_UART_MUX_T_ {
	NPCM_UART_MUX_MODE1_HSP1_SI2____HSP2_UART2__UART1_s_HSP1__UART3_s_SI2                = 0,
	NPCM_UART_MUX_MODE2_HSP1_UART1__HSP2_SI2____UART2_s_HSP2__UART3_s_SI2                = 1,
	NPCM_UART_MUX_MODE3_HSP1_UART1__HSP2_UART2__UART3_SI2                                = 2,
	NPCM_UART_MUX_MODE4_HSP1_SI1____HSP2_SI2____UART1_s_SI1___UART3_s_SI2__UART2_s_HSP1  = 3,
	NPCM_UART_MUX_MODE5_HSP1_SI1____HSP2_UART2__UART1_s_HSP1__UART3_s_SI1                = 4,
	NPCM_UART_MUX_MODE6_HSP1_SI1____HSP2_SI2____UART1_s_SI1___UART3_s_SI2__UART2_s_HSP2  = 5,
	NPCM_UART_MUX_MODE7_HSP1_SI1____HSP2_UART2__UART1_s_HSP1__UART3_SI2                  = 6,
	NPCM_UART_MUX_RESERVED                                                               = 7,
	NPCM_UART_MUX_SKIP_CONFIG                                                            = 8
} NPCM_UART_MUX_T;
#endif

/* Common baudrate definitions */
typedef enum {
	NPCM_UART_BAUDRATE_110       = 110,
	NPCM_UART_BAUDRATE_300       = 300,
	NPCM_UART_BAUDRATE_600       = 600,
	NPCM_UART_BAUDRATE_1200      = 1200,
	NPCM_UART_BAUDRATE_2400      = 2400,
	NPCM_UART_BAUDRATE_4800      = 4800,
	NPCM_UART_BAUDRATE_9600      = 9600,
	NPCM_UART_BAUDRATE_14400     = 14400,
	NPCM_UART_BAUDRATE_19200     = 19200,
	NPCM_UART_BAUDRATE_38400     = 38400,
	NPCM_UART_BAUDRATE_57600     = 57600,
	NPCM_UART_BAUDRATE_115200    = 115200,
	NPCM_UART_BAUDRATE_230400    = 230400,
	NPCM_UART_BAUDRATE_380400    = 380400,
	NPCM_UART_BAUDRATE_460800    = 460800,
} NPCM_UART_BAUDRATE_T;

/* Uart Rx Fifo Trigger level definitions */
typedef enum {
	NPCM_UART_RXFIFO_TRIGGER_1B      = 0x0,
	NPCM_UART_RXFIFO_TRIGGER_4B      = 0x1,
	NPCM_UART_RXFIFO_TRIGGER_8B      = 0x2,
	NPCM_UART_RXFIFO_TRIGGER_14B     = 0x3,
} NPCM_UART_RXFIFO_TRIGGER_T;

/* UART parity types */
typedef enum {
	NPCM_UART_PARITY_NONE    = 0x00,
	NPCM_UART_PARITY_EVEN    = 0x01,
	NPCM_UART_PARITY_ODD     = 0x02,
} NPCM_UART_PARITY_T;

/* Uart stop bits */
typedef enum {
	NPCM_UART_STOPBIT_1          = 0x00,
	NPCM_UART_STOPBIT_DYNAMIC    = 0x01,
} NPCM_UART_STOPBIT_T;

/* Callback functions for UART IRQ handler */
typedef void (*UART_irq_callback_t)(u8 devNum, void *args);

/* Default UART PORT configurations */
#ifdef _PALLADIUM_
#define NPCM_SERIAL_BAUD     NPCM_UART_BAUDRATE_600
#else
#define NPCM_SERIAL_BAUD     NPCM_UART_BAUDRATE_115200
#endif
#define NPCM_SERIAL_BITS     8
#define NPCM_SERIAL_PARITY   'n'

/* UART driver name and definitions */
#define NPCM_SERIAL_NAME     "ttyS"
#define NPCM_SERIAL_MAJOR    4
#define NPCM_SERIAL_MINOR    64

/* Default configurations for Rx and Tx FIFOs */
#define NPCM_SERIAL_RX_TIMEOUT       0x20
#define NPCM_SERIAL_RX_THRES         NPCM_UART_RXFIFO_TRIGGER_1B
#define NPCM_SERIAL_TX_FIFO_SIZE     16

/* Default configurations for Console */
#define NPCM_SERIAL_CONSOLE_PORT     NPCM_UART3_DEV

/* Our port definitions and structures */
struct npcm_uart_port {
	unsigned char      rx_claimed;
	unsigned char      tx_claimed;
	unsigned int       type;
	struct uart_port   port;
};

static int npcm_uart_init(NPCM_UART_DEV_T devNum,
			     NPCM_UART_MUX_T muxMode,
			     NPCM_UART_BAUDRATE_T baudRate);
static int npcm_uart_putc(NPCM_UART_DEV_T devNum, const u8 c);
static int npcm_uart_putc_NB(NPCM_UART_DEV_T devNum, const u8 c);
static int npcm_uart_getc_NB(NPCM_UART_DEV_T devNum, u8 *c);
static bool npcm_uart_test_rx(NPCM_UART_DEV_T devNum);
static bool npcm_uart_test_tx(NPCM_UART_DEV_T devNum);
static int npcm_uart_reset_fifo(NPCM_UART_DEV_T devNum, bool txFifo,
			    bool rxFifo);
static int npcm_uart_set_rx_irq_state(NPCM_UART_DEV_T devNum, bool On);
static int npcm_uart_set_rx_config(NPCM_UART_DEV_T devNum, u8 timeout,
				    NPCM_UART_RXFIFO_TRIGGER_T triggerLevel);
static int npcm_uart_set_parity(NPCM_UART_DEV_T devNum,
				   NPCM_UART_PARITY_T parity);
static int npcm_uart_set_bits_per_char(NPCM_UART_DEV_T devNum, u32 bits);
static int npcm_uart_set_baud_rate(NPCM_UART_DEV_T devNum,
				      NPCM_UART_BAUDRATE_T baudrate);
static int npcm_uart_set_stop_bit(NPCM_UART_DEV_T devNum,
				     NPCM_UART_STOPBIT_T stopbit);
static int npcm_uart_set_break(NPCM_UART_DEV_T devNum, bool state);
static int npcm_uart_isr(NPCM_UART_DEV_T devNum,
			    UART_irq_callback_t rxCallback,
			    void *rxParam, UART_irq_callback_t txCallback,
			    void *txParam);
static void npcm_serial_stop_tx(struct uart_port *port);
static void npcm_serial_start_tx(struct uart_port *port);
static void npcm_serial_stop_rx(struct uart_port *port);
static void npcm_serial_tx_irq(u8 devNum, void *args);
static void npcm_serial_rx_irq(u8 devNum, void *args);
static irqreturn_t npcm_serial_irq(int irq, void *dev_id);
static unsigned int npcm_serial_tx_empty(struct uart_port *port);
static void npcm_serial_break_ctl(struct uart_port *port, int break_state);
static void npcm_serial_shutdown(struct uart_port *port);
static int npcm_serial_startup(struct uart_port *port);
static void npcm_serial_set_termios(struct uart_port *port,
				       struct ktermios *new,
				       struct ktermios *old);
static const char *npcm_serial_type(struct uart_port *port);
static void npcm_serial_config_port(struct uart_port *port, int flags);
static int npcm_serial_verify_port(struct uart_port *port,
				      struct serial_struct *ser);
static void npcm_serial_set_mctrl(struct uart_port *port,
				     unsigned int mctrl);
static unsigned int npcm_serial_get_mctrl(struct uart_port *port);
static void npcm_serial_flush_buffer(struct uart_port *port);
static void npcm_serial_set_ldisc(struct uart_port *port,
				     struct ktermios *new);
static void npcm_serial_pm(struct uart_port *port, unsigned int state,
			      unsigned int oldstate);
static void npcm_serial_release_port(struct uart_port *port);
static int npcm_serial_request_port(struct uart_port *port);
static int npcm_serial_init_port(struct npcm_uart_port *ourport,
				    struct platform_device *platdev);
static int npcm_serial_probe(struct platform_device *dev);
static int npcm_serial_remove(struct platform_device *dev);
static int __init npcm_serial_modinit(void);
static void __exit npcm_serial_modexit(void);
static void npcm_console_putchar(struct uart_port *port, int ch);
static int __init npcm_console_setup(struct console *co, char *options);
static void npcm_console_write(struct console *co, const char *s,
				  unsigned int count);
static int npcm_console_init(void);

#define regwrite8(mem, val) iowrite8(val, mem)

static inline void set_reg_field8(unsigned char __iomem *mem,
				  bit_field_t bit_field, u8 val) {
	u8 tmp = ioread8(mem);

	tmp &= ~(((1 << bit_field.size) - 1) << bit_field.offset);
	tmp |= val << bit_field.offset;
	iowrite8(tmp, mem);
}

#define set_var_field(var, bit_field, value) {				\
	typeof(var) tmp = var;						\
	tmp &= ~(((1 << bit_field.size) - 1) << bit_field.offset);	\
	tmp |= value << bit_field.offset;				\
	var = tmp;							\
}

static inline u8 read_reg_field8(unsigned char __iomem *mem,
				 bit_field_t bit_field) {
	u8 tmp = ioread8(mem);

	tmp = tmp >> bit_field.offset;
	tmp &= (1 << bit_field.size) - 1;
	return tmp;
}

#define read_var_field(var, bit_field) ({	\
	typeof(var) tmp = var;			\
	tmp = tmp >> bit_field.offset;		\
	tmp &= (1 << bit_field.size) - 1;	\
	tmp;					\
})

#ifdef CONFIG_OF
static const struct of_device_id uart_dt_id[];
#endif

static struct platform_driver npcm_serial_drv;
static struct uart_port *npcm_console_port;

/* Forward declaration of driver structs */
static struct uart_ops          npcm_serial_ops;
static struct platform_driver   npcm_serial_drv;
static struct console           npcm_serial_console;

static struct npcm_uart_port
npcm_serial_ports[NPCM_UART_NUM_OF_MODULES] = {
	/* Uart0 port */
	{
		.port = {
			.lock       =
			__SPIN_LOCK_UNLOCKED(npcm_serial_ports[0].port.lock),
			.iotype     = UPIO_MEM,
			.fifosize   = NPCM_SERIAL_TX_FIFO_SIZE,
			.ops        = &npcm_serial_ops,
			.flags      = UPF_SPD_VHI | UPF_BOOT_AUTOCONF,
			.line       = 0,
		},
		.type = PORT_NPCM,
	},

	/* Uart1 port */
	{
		.port = {
			.lock       =
			__SPIN_LOCK_UNLOCKED(npcm_serial_ports[1].port.lock),
			.iotype     = UPIO_MEM,
			.fifosize   = NPCM_SERIAL_TX_FIFO_SIZE,
			.ops        = &npcm_serial_ops,
			.flags      = UPF_SPD_VHI | UPF_BOOT_AUTOCONF,
			.line       = 1,
		},
		.type = PORT_NPCM,
	},

	/* Uart2 port */
	{
		.port = {
			.lock       =
			__SPIN_LOCK_UNLOCKED(npcm_serial_ports[2].port.lock),
			.iotype     = UPIO_MEM,
			.fifosize   = NPCM_SERIAL_TX_FIFO_SIZE,
			.ops        = &npcm_serial_ops,
			.flags      = UPF_SPD_VHI | UPF_BOOT_AUTOCONF,
			.line       = 2,
		},
		.type = PORT_NPCM,
	},

	/* Uart3 port */
	{
		.port = {
			.lock       =
			__SPIN_LOCK_UNLOCKED(npcm_serial_ports[3].port.lock),
			.iotype     = UPIO_MEM,
			.fifosize   = NPCM_SERIAL_TX_FIFO_SIZE,
			.ops        = &npcm_serial_ops,
			.flags      = UPF_SPD_VHI | UPF_BOOT_AUTOCONF,
			.line       = 3,
		},
		.type = PORT_NPCM,
	},
};


/* Interrupt handling */
#define tx_enabled(port)    ((port)->unused[0])
#define rx_enabled(port)    ((port)->unused[1])

/* conversion functions between various structures */
#define npcm_convert_port_to_ourport(port)	\
		container_of(port, struct npcm_uart_port, port)
#define npcm_convert_port_to_portname(port)	\
		to_platform_device(port->dev)->name

static int npcm_uart_init(NPCM_UART_DEV_T devNum,
			     NPCM_UART_MUX_T muxMode,
			     NPCM_UART_BAUDRATE_T baudRate)
{
	u32 FCR_Val      = 0;
	bool CoreSP  = false;
	/*bool sp1     = false;
	bool sp2     = false;*/
	u32  ret     = 0;

	if (devNum >= NPCM_UART_NUM_OF_MODULES)
		return -EINVAL;

	if (devNum == NPCM_UART0_DEV)
		CoreSP = true;

	#if defined NPCM650
	else if (devNum == NPCM_UART1_DEV) {
		CoreSP = false;

		switch (muxMode) {
		case NPCM_UART_MUX_CORE_SNOOP:
		case NPCM_UART_MUX_CORE_TAKEOVER:
			sp1 = true;
			sp2 = true;
			break;
		case NPCM_UART_MUX_CORE_SP2__SP1_SI1:
			sp1 = true;
			break;
		case NPCM_UART_MUX_CORE_SP2__SP1_SI2:
			sp2 = true;
			break;
		case NPCM_UART_MUX_SKIP_CONFIG:
			break;
		default:
			return HAL_ERROR_BAD_PARAM;
		}
	}
#elif (defined NPCM750_CHIP || defined NPCM750_CP)
	switch (muxMode) {
	case NPCM_UART_MUX_MODE4_HSP1_SI1____HSP2_SI2____UART1_s_SI1___UART3_s_SI2__UART2_s_HSP1:
	case NPCM_UART_MUX_MODE6_HSP1_SI1____HSP2_SI2____UART1_s_SI1___UART3_s_SI2__UART2_s_HSP2:
	case NPCM_UART_MUX_MODE7_HSP1_SI1____HSP2_UART2__UART1_s_HSP1__UART3_SI2:
		sp1 = true;
		sp2 = true;
		break;
	case NPCM_UART_MUX_MODE5_HSP1_SI1____HSP2_UART2__UART1_s_HSP1__UART3_s_SI1:
		sp1 = true;
		break;
	case NPCM_UART_MUX_MODE1_HSP1_SI2____HSP2_UART2__UART1_s_HSP1__UART3_s_SI2:
	case NPCM_UART_MUX_MODE2_HSP1_UART1__HSP2_SI2____UART2_s_HSP2__UART3_s_SI2:
	case NPCM_UART_MUX_MODE3_HSP1_UART1__HSP2_UART2__UART3_SI2:
		sp2 = true;
		break;
	case NPCM_UART_MUX_SKIP_CONFIG:
		break;
	default:
		return -1;
	}
#endif

	if (muxMode != NPCM_UART_MUX_SKIP_CONFIG) {
		if (muxMode < 7) {
			/*regmap_update_bits(gcr_regmap, SPSWC_OFFSET, (0x7 << 0)
				, muxMode & 0x7);

			if (CoreSP)
			{
			    regmap_update_bits(gcr_regmap, MFSEL1_OFFSET, (0x1 << 9), (0x1 << 9));
			    regmap_update_bits(gcr_regmap, MFSEL4_OFFSET, (0x1 << 1), (0x1 << 1));
			}
			if (sp1)
			{
			    regmap_update_bits(gcr_regmap, MFSEL1_OFFSET, (0x1 << 10), (0x1 << 10));
			    regmap_update_bits(gcr_regmap, MFSEL4_OFFSET, (0x1 << 1), 0);
			}
			if (sp2)
			{
			    regmap_update_bits(gcr_regmap, MFSEL1_OFFSET, (0x1 << 11), (0x1 << 11));
			    regmap_update_bits(gcr_regmap, MFSEL4_OFFSET, (0x1 << 1), 0);
			}*/
		}
	}

	regwrite8(UART_LCR(devNum), 0);            // prepare to Init UART
	regwrite8(UART_IER(devNum), 0x0);          // Disable all UART interrupt

	ret += npcm_uart_set_baud_rate(devNum, baudRate);

	ret += npcm_uart_set_bits_per_char(devNum, 8);
	ret += npcm_uart_set_stop_bit(devNum, NPCM_UART_STOPBIT_1);
	ret += npcm_uart_set_parity(devNum, NPCM_UART_PARITY_NONE);

	FCR_Val = 0;
	set_var_field(FCR_Val, FCR_RFITL, FCR_RFITL_4B);
	set_var_field(FCR_Val, FCR_TFR, 1);
	set_var_field(FCR_Val, FCR_RFR, 1);
	set_var_field(FCR_Val, FCR_FME, 1);

	regwrite8(UART_FCR(devNum), FCR_Val);
	regwrite8(UART_TOR(devNum), 0x0);

	if (ret > 0)
		return -1;
	else
		return 0;
}

static int npcm_uart_putc(NPCM_UART_DEV_T devNum, const u8 c)
{

	if (devNum >= NPCM_UART_NUM_OF_MODULES)
		return -EINVAL;

	while
		(!read_reg_field8(UART_LSR(devNum), LSR_THRE));
	regwrite8(UART_THR(devNum), (c & 0xFF));

	return 0;
}

static int npcm_uart_putc_NB(NPCM_UART_DEV_T devNum, const u8 c)
{
	if (devNum >= NPCM_UART_NUM_OF_MODULES)
		return -EINVAL;

	regwrite8(UART_THR(devNum), (c & 0xFF));

	return 0;
}

static int npcm_uart_getc_NB(NPCM_UART_DEV_T devNum, u8 *c)
{
	if (devNum >= NPCM_UART_NUM_OF_MODULES)
		return -EINVAL;

	if (!npcm_uart_test_rx(devNum))
		return -1;

	*c = (ioread8(UART_RBR(devNum)) & 0xFF);

	return 0;
}

static bool npcm_uart_test_rx(NPCM_UART_DEV_T devNum)
{
	if (read_reg_field8(UART_LSR(devNum), LSR_RFDR))
		return true;
	else
		return false;
}

static bool npcm_uart_test_tx(NPCM_UART_DEV_T devNum)
{
	if (!read_reg_field8(UART_LSR(devNum), LSR_THRE))
		return true;
	else
		return false;
}

static int npcm_uart_reset_fifo(NPCM_UART_DEV_T devNum, bool txFifo,
				   bool rxFifo)
{
	if (devNum >= NPCM_UART_NUM_OF_MODULES)
		return -EINVAL;

	if (txFifo)
		set_reg_field8(UART_FCR(devNum), FCR_TFR, 1);

	if (rxFifo)
		set_reg_field8(UART_FCR(devNum), FCR_RFR, 1);

	return 0;
}

static int npcm_uart_set_tx_irq_state(NPCM_UART_DEV_T devNum, bool On)
{
	if (devNum >= NPCM_UART_NUM_OF_MODULES)
		return -EINVAL;

	if (On)
		set_reg_field8(UART_IER(devNum), IER_THREIE, 1);
	else
		set_reg_field8(UART_IER(devNum), IER_THREIE, 0);

	return 0;
}

static int npcm_uart_set_rx_irq_state(NPCM_UART_DEV_T devNum, bool On)
{
	if (devNum >= NPCM_UART_NUM_OF_MODULES)
		return -EINVAL;

	if (On) {
		set_reg_field8(UART_IER(devNum), IER_RDAIE, 1);
		set_reg_field8(UART_TOR(devNum), TOR_TOIE, 1);
	} else {
		set_reg_field8(UART_IER(devNum), IER_RDAIE, 0);
		set_reg_field8(UART_TOR(devNum), TOR_TOIE, 0);
	}

	return 0;
}

static int npcm_uart_set_rx_config(NPCM_UART_DEV_T devNum,
				      u8 timeout,
				      NPCM_UART_RXFIFO_TRIGGER_T triggerLevel)
{
	if (devNum >= NPCM_UART_NUM_OF_MODULES)
		return -EINVAL;

	set_reg_field8(UART_TOR(devNum), TOR_TOIC, (timeout & 0x7F));
	set_reg_field8(UART_FCR(devNum), FCR_RFITL, (triggerLevel << 2));

	return 0;
}

static int npcm_uart_set_parity(NPCM_UART_DEV_T devNum,
				   NPCM_UART_PARITY_T parity)
{
	if (devNum >= NPCM_UART_NUM_OF_MODULES)
		return -EINVAL;

	if (parity != NPCM_UART_PARITY_NONE) {

		set_reg_field8(UART_LCR(devNum), LCR_PBE, 1);

		if (parity == NPCM_UART_PARITY_EVEN)
			set_reg_field8(UART_LCR(devNum), LCR_EPE, 1);
		else if (parity == NPCM_UART_PARITY_ODD)
			set_reg_field8(UART_LCR(devNum), LCR_EPE, 0);
		else
			return -EINVAL;
	} else
		set_reg_field8(UART_LCR(devNum), LCR_PBE, 0);

	return 0;
}

static int npcm_uart_set_bits_per_char(NPCM_UART_DEV_T devNum, u32 bits)
{
	if (devNum >= NPCM_UART_NUM_OF_MODULES)
		return -EINVAL;

	switch (bits) {
	case 5:
		set_reg_field8(UART_LCR(devNum), LCR_WLS, LCR_WLS_5bit);
		break;
	case 6:
		set_reg_field8(UART_LCR(devNum), LCR_WLS, LCR_WLS_6bit);
		break;
	case 7:
		set_reg_field8(UART_LCR(devNum), LCR_WLS, LCR_WLS_7bit);
		break;
	default:
	case 8:
		set_reg_field8(UART_LCR(devNum), LCR_WLS, LCR_WLS_8bit);
		break;
	}

	return 0;
}

static int npcm_uart_set_baud_rate(NPCM_UART_DEV_T devNum,
				      NPCM_UART_BAUDRATE_T baudrate)
{
	int               divisor     = 0;
	u32               uart_clock  = 0;
	int               ret         = 0;

	if (devNum >= NPCM_UART_NUM_OF_MODULES)
		return -EINVAL;

	if (npcm_serial_ports[devNum].port.uartclk != 0)
		uart_clock = npcm_serial_ports[devNum].port.uartclk;
	else {
		NPCM_SERIAL_MSG("%s: warning, uart clock unknown!\n",
				   __func__);
		uart_clock = 24 * _1MHz_;
	}

	divisor = ((int)uart_clock / ((int)baudrate * 16)) - 2;

	/* since divisor is rounded down check
	 * if it is better when rounded up
	 */
	if (((int)uart_clock / (16 * (divisor + 2)) - (int)baudrate) >
		((int)baudrate - (int)uart_clock / (16 * ((divisor + 1) + 2))))
		divisor++;

	if (divisor < 0) {
		divisor = 0;
		ret = EINVAL;
	}

	/* Set baud rate to baudRate bps */
	set_reg_field8(UART_LCR(devNum), LCR_DLAB, 1);
	regwrite8(UART_DLL(devNum), LSB(divisor));
	regwrite8(UART_DLM(devNum), MSB(divisor));
	set_reg_field8(UART_LCR(devNum), LCR_DLAB, 0);

	return ret;
}

static int npcm_uart_set_stop_bit(NPCM_UART_DEV_T devNum,
				     NPCM_UART_STOPBIT_T stopbit)
{
	if (devNum >= NPCM_UART_NUM_OF_MODULES)
		return -EINVAL;

	if (stopbit == NPCM_UART_STOPBIT_1)
		set_reg_field8(UART_LCR(devNum), LCR_NSB, 0);
	else if (stopbit == NPCM_UART_STOPBIT_DYNAMIC)
		set_reg_field8(UART_LCR(devNum), LCR_NSB, 1);
	else
		return -EINVAL;

	return 0;
}

static int npcm_uart_set_break(NPCM_UART_DEV_T devNum, bool state)
{
	if (devNum >= NPCM_UART_NUM_OF_MODULES)
		return -EINVAL;

	if (state)
		set_reg_field8(UART_LCR(devNum), LCR_BCB, 1);
	else
		set_reg_field8(UART_LCR(devNum), LCR_BCB, 0);

	return 0;
}

static int npcm_uart_isr(NPCM_UART_DEV_T devNum,
			    UART_irq_callback_t rxCallback, void *rxParam,
			    UART_irq_callback_t txCallback, void *txParam)
{
	int  ret = 0;
	u32      iir = ioread8(UART_IIR(devNum)) & 0xF;

	if (read_var_field(iir, IIR_NIP)) {
		ret =  -ENXIO;
	} else {
		switch (read_var_field(iir, IIR_IID)) {
		case IIR_IID_MODEM:
			break;
			/* Tx Interrupt */
		case IIR_IID_THRE:
			txCallback(devNum, txParam);    break;
			/* Rx Interrupts */
		case IIR_IID_TOUT:
		case IIR_IID_RDA:
			rxCallback(devNum, rxParam);    break;
		default:
			break;
			/* Error interrupts */
		case IIR_IID_RLS:
			{
				u32 lsr = ioread8(UART_LSR(devNum));
				if (read_var_field(lsr, LSR_OEI))
					ret = -EOVERFLOW;
				else if (read_var_field(lsr, LSR_PEI))
					ret = -EILSEQ;
				else if (read_var_field(lsr, LSR_FEI))
					ret = -EMSGSIZE;
				else if (read_var_field(lsr, LSR_BII))
					ret = -EIO;
				else
					ret = -1;
				break;
			}
		}
	}

	return ret;
}

static void npcm_serial_stop_tx(struct uart_port *port)
{
	if (tx_enabled(port)) {
		npcm_uart_set_tx_irq_state((NPCM_UART_DEV_T)port->line,
					      false);
		tx_enabled(port) = 0;
	}
}

static void npcm_serial_start_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;

	if ((!tx_enabled(port)) && (!uart_circ_empty(xmit))) {
		npcm_uart_set_tx_irq_state((NPCM_UART_DEV_T)port->line,
					      true);
		tx_enabled(port) = 1;
	}
}

static void npcm_serial_stop_rx(struct uart_port *port)
{
	if (rx_enabled(port)) {
		npcm_uart_set_rx_irq_state((NPCM_UART_DEV_T)port->line,
					      false);
		rx_enabled(port) = 0;
	}
}

static void npcm_serial_tx_irq(u8 devNum, void *args)
{
	struct npcm_uart_port *ourport = (struct npcm_uart_port *)args;
	struct uart_port *port = &ourport->port;
	struct circ_buf *xmit = &port->state->xmit;

	int count = NPCM_SERIAL_TX_FIFO_SIZE;

	if (port->x_char) {
		if (!npcm_uart_test_tx((NPCM_UART_DEV_T)port->line)) {
			npcm_uart_putc_NB((NPCM_UART_DEV_T)port->line,
					     port->x_char);
			port->icount.tx++;
			port->x_char = 0;
		}

		return;
	}

	/*
	 * if there isnt anything more to transmit, or the uart is now
	 * stopped, disable the uart and exit
	 */
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		npcm_serial_stop_tx(port);
		return;
	}

	/*
	 * try and drain the buffer
	 */
	while (!uart_circ_empty(xmit) && count-- > 0) {
		npcm_uart_putc_NB((NPCM_UART_DEV_T)port->line,
				     xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	}

	/*
	 * if the queue is not empty, we schedule late
	 * processing after the interrupt
	 */
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	/*
	 * if the queue is empty we stop TX
	 */
	if (uart_circ_empty(xmit))
		npcm_serial_stop_tx(port);
}

static void npcm_serial_rx_irq(u8 devNum, void *args)
{
	struct npcm_uart_port *ourport   = (struct npcm_uart_port *)args;
	struct uart_port *port              = &ourport->port;
	unsigned char   ch;
	int             status;
	int             max_count = NPCM_SERIAL_TX_FIFO_SIZE;

	while ((max_count-- > 0) && npcm_uart_test_rx(devNum)) {
		status = npcm_uart_getc_NB(devNum, &ch);

		port->icount.rx++;
		if (!uart_handle_sysrq_char(port, ch) && (status == 0))
			uart_insert_char(port, 0, 0, ch, TTY_NORMAL);
	}

	tty_flip_buffer_push(&port->state->port);
}

static irqreturn_t npcm_serial_irq(int irq, void *dev_id)
{
	struct npcm_uart_port *ourport = (struct npcm_uart_port *)dev_id;
	struct uart_port *port    = &ourport->port;
	int                         ret;

	ret = npcm_uart_isr((NPCM_UART_DEV_T)port->line,
			       npcm_serial_rx_irq, dev_id,
			       npcm_serial_tx_irq, dev_id);

	if (ret == (-ENXIO))
		return IRQ_NONE;

	else if (ret != 0) {
		NPCM_SERIAL_MSG("NPCM Serial: IRQ Error\n");
		if (ret == (-EIO)) {
			NPCM_SERIAL_MSG("NPCM Serial: break!\n");
			port->icount.brk++;
			uart_handle_break(port);
		} else if (ret == (-EMSGSIZE)) {
			port->icount.frame++;
			NPCM_SERIAL_MSG("NPCM Serial: frame error!\n");
		} else if (ret == (-EOVERFLOW)) {
			port->icount.overrun++;
			NPCM_SERIAL_MSG("NPCM Serial: overrun error!\n");
		} else if (ret == (-EILSEQ)) {
			port->icount.parity++;
			NPCM_SERIAL_MSG("NPCM Serial: parity error!\n");
		}
	}

	return IRQ_HANDLED;
}

static unsigned int npcm_serial_tx_empty(struct uart_port *port)
{
	return !npcm_uart_test_tx((NPCM_UART_DEV_T)port->line);
}

static void npcm_serial_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	npcm_uart_set_break((NPCM_UART_DEV_T)port->line, break_state);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void npcm_serial_shutdown(struct uart_port *port)
{
	struct npcm_uart_port *ourport =
		npcm_convert_port_to_ourport(port);

	NPCM_SERIAL_MSG("NPCM Serial: Called function %s\n", __func__);

	if (ourport->tx_claimed || ourport->rx_claimed)
		free_irq(port->irq, ourport);

	if (ourport->tx_claimed) {
		npcm_uart_set_tx_irq_state((NPCM_UART_DEV_T)port->line,
					      false);
		tx_enabled(port) = 0;
		ourport->tx_claimed = 0;
	}

	if (ourport->rx_claimed) {
		npcm_uart_set_rx_irq_state((NPCM_UART_DEV_T)port->line,
					      false);
		ourport->rx_claimed = 0;
		rx_enabled(port) = 0;
	}
}

static int npcm_serial_startup(struct uart_port *port)
{
	struct npcm_uart_port *ourport =
		npcm_convert_port_to_ourport(port);
	int ret;

	NPCM_SERIAL_MSG("NPCM Serial: Called function %s\n", __func__);

#ifndef CONFIG_SERIAL_NPCM_CONSOLE
	npcm_uart_init((NPCM_UART_DEV_T)port->line,
			  NPCM_UART_MUX_MODE3_HSP1_UART1__HSP2_UART2__UART3_SI2,
			  NPCM_SERIAL_BAUD);
#endif

	npcm_uart_reset_fifo((NPCM_UART_DEV_T)port->line, true, true);

	npcm_uart_set_rx_config((NPCM_UART_DEV_T)port->line,
				   (u8)NPCM_SERIAL_RX_TIMEOUT,
				   NPCM_SERIAL_RX_THRES);

	ret = request_irq(port->irq, npcm_serial_irq, 0,
			  npcm_convert_port_to_portname(port),
			  (void *)ourport);

	if (ret != 0) {
		pr_err("NPCM Serial: cannot get irq %d\n", port->irq);
		npcm_serial_shutdown(port);
		return ret;
	}

	npcm_uart_set_rx_irq_state((NPCM_UART_DEV_T)port->line, true);
	rx_enabled(port) = 1;

	ourport->rx_claimed = 1;
	ourport->tx_claimed = 1;

	NPCM_SERIAL_MSG("NPCM Serial: Port %d started with flags 0x%X\n",
			   port->line, port->flags);

	return 0;
}

static void npcm_serial_set_termios(struct uart_port *port,
				       struct ktermios *new,
				       struct ktermios *old)
{
	unsigned int bits   = 0;
	unsigned int baud   = 0;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	NPCM_SERIAL_MSG("NPCM Serial: Called function %s\n", __func__);
	NPCM_SERIAL_MSG("NPCM Serial: Flags NEW = iflags[0x%X], "
			   "oflags[0x%X], cflags[0x%X], lflags[0x%X]\n",
			   new->c_iflag, new->c_oflag,
			   new->c_cflag, new->c_lflag);

	/* We don't support modem control lines */
	new->c_cflag &= ~(HUPCL | CMSPAR);
	new->c_cflag |= CLOCAL;

	switch (new->c_cflag & CSIZE) {
	case CS5:
		bits = 5;      break;
	case CS6:
		bits = 6;      break;
	case CS7:
		bits = 7;      break;
	case CS8:
	default:
		bits = 8;      break;
	}

	npcm_uart_set_bits_per_char((NPCM_UART_DEV_T)port->line, bits);

	NPCM_SERIAL_MSG("NPCM Serial: %dbits/char\n", bits);

	if (new->c_cflag & CSTOPB) {
		npcm_uart_set_stop_bit((NPCM_UART_DEV_T)port->line,
					  NPCM_UART_STOPBIT_DYNAMIC);
		NPCM_SERIAL_MSG("NPCM Serial: Stop Bits dynamic\n");
	} else {
		npcm_uart_set_stop_bit((NPCM_UART_DEV_T)port->line,
					  NPCM_UART_STOPBIT_1);
		NPCM_SERIAL_MSG("NPCM Serial: 1 Stop Bit\n");
	}

	if (new->c_cflag & PARENB) {
		if (new->c_cflag & PARODD) {
			npcm_uart_set_parity((NPCM_UART_DEV_T)port->line,
						NPCM_UART_PARITY_ODD);
			NPCM_SERIAL_MSG("NPCM Serial: ODD  parity\n");
		} else {
			npcm_uart_set_parity((NPCM_UART_DEV_T)port->line,
						NPCM_UART_PARITY_EVEN);
			NPCM_SERIAL_MSG("NPCM Serial: EVEN parity\n");
		}
	} else {
		npcm_uart_set_parity((NPCM_UART_DEV_T)port->line,
					NPCM_UART_PARITY_NONE);
		NPCM_SERIAL_MSG("NPCM Serial: NO parity\n");
	}

	switch (new->c_cflag & CBAUD) {
	case  B110:
		npcm_uart_set_baud_rate((NPCM_UART_DEV_T)port->line,
					   NPCM_UART_BAUDRATE_110);
		baud = 110;
		break;
	case  B300:
		npcm_uart_set_baud_rate((NPCM_UART_DEV_T)port->line,
					   NPCM_UART_BAUDRATE_300);
		baud = 300;
		break;
	case  B600:
		npcm_uart_set_baud_rate((NPCM_UART_DEV_T)port->line,
					   NPCM_UART_BAUDRATE_600);
		baud = 600;
		break;
	case  B1200:
		npcm_uart_set_baud_rate((NPCM_UART_DEV_T)port->line,
					   NPCM_UART_BAUDRATE_1200);
		baud = 1200;
		break;
	case  B2400:
		npcm_uart_set_baud_rate((NPCM_UART_DEV_T)port->line,
					   NPCM_UART_BAUDRATE_2400);
		baud = 2400;
		break;
	case  B4800:
		npcm_uart_set_baud_rate((NPCM_UART_DEV_T)port->line,
					   NPCM_UART_BAUDRATE_4800);
		baud = 4800;
		break;
	case  B9600:
		npcm_uart_set_baud_rate((NPCM_UART_DEV_T)port->line,
					   NPCM_UART_BAUDRATE_9600);
		baud = 9600;
		break;
	case  B19200:
		npcm_uart_set_baud_rate((NPCM_UART_DEV_T)port->line,
					   NPCM_UART_BAUDRATE_19200);
		baud = 19200;
		break;
	case  B38400:
		npcm_uart_set_baud_rate((NPCM_UART_DEV_T)port->line,
					   NPCM_UART_BAUDRATE_38400);
		baud = 38400;
		break;
	case  B57600:
		npcm_uart_set_baud_rate((NPCM_UART_DEV_T)port->line,
					   NPCM_UART_BAUDRATE_57600);
		baud = 57600;
		break;
	default:
	case  B115200:
		npcm_uart_set_baud_rate((NPCM_UART_DEV_T)port->line,
					   NPCM_UART_BAUDRATE_115200);
		baud = 115200;
		break;
	case  B230400:
		npcm_uart_set_baud_rate((NPCM_UART_DEV_T)port->line,
					   NPCM_UART_BAUDRATE_230400);
		baud = 230400;
		break;
	case  B460800:
		npcm_uart_set_baud_rate((NPCM_UART_DEV_T)port->line,
					   NPCM_UART_BAUDRATE_460800);
		baud = 460800;
		break;
	}

	NPCM_SERIAL_MSG("NPCM Serial: Baudrate %d\n", baud);

	uart_update_timeout(port, new->c_cflag, baud);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *npcm_serial_type(struct uart_port *port)
{
	NPCM_SERIAL_MSG("NPCM Serial: Called function %s\n", __func__);

	if (port->type == PORT_NPCM)
		return npcm_convert_port_to_portname(port);
	else
		return NULL;
}

static void npcm_serial_config_port(struct uart_port *port, int flags)
{
	unsigned long lock_flags;

	spin_lock_irqsave(&port->lock, lock_flags);

	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_NPCM;

	spin_unlock_irqrestore(&port->lock, lock_flags);
}

static int npcm_serial_verify_port(struct uart_port *port,
				      struct serial_struct *ser)
{
	NPCM_SERIAL_MSG("NPCM Serial: Called function %s\n", __func__);

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_NPCM)
		return -EINVAL;

	return 0;
}

static void npcm_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	NPCM_SERIAL_MSG("NPCM Serial: Called function %s\n", __func__);
}

static unsigned int npcm_serial_get_mctrl(struct uart_port *port)
{
	NPCM_SERIAL_MSG("NPCM Serial: Called function %s\n", __func__);
	return 0;
}

static void npcm_serial_flush_buffer(struct uart_port *port)
{
	NPCM_SERIAL_MSG("NPCM Serial: Called function %s\n", __func__);
}

static void npcm_serial_set_ldisc(struct uart_port *port,
				     struct ktermios *new)
{
	NPCM_SERIAL_MSG("NPCM Serial: Called function %s\n", __func__);
}

static void npcm_serial_pm(struct uart_port *port, unsigned int state,
			      unsigned int oldstate)
{
	NPCM_SERIAL_MSG("NPCM Serial: Called function %s\n", __func__);
}

static void npcm_serial_release_port(struct uart_port *port)
{
	NPCM_SERIAL_MSG("NPCM Serial: Called function %s\n", __func__);
}

static int npcm_serial_request_port(struct uart_port *port)
{
	NPCM_SERIAL_MSG("NPCM Serial: Called function %s\n", __func__);
	return 0;
}

/*
 * Uart port driver structure
 */
static struct uart_ops npcm_serial_ops = {
	.startup        = npcm_serial_startup,
	.shutdown       = npcm_serial_shutdown,
	.tx_empty       = npcm_serial_tx_empty,
	.stop_tx        = npcm_serial_stop_tx,
	.start_tx       = npcm_serial_start_tx,
	.stop_rx        = npcm_serial_stop_rx,
	.break_ctl      = npcm_serial_break_ctl,
	.type           = npcm_serial_type,
	.config_port    = npcm_serial_config_port,
	.verify_port    = npcm_serial_verify_port,
	.set_termios    = npcm_serial_set_termios,
	.set_mctrl      = npcm_serial_set_mctrl,
	.get_mctrl      = npcm_serial_get_mctrl,
	.flush_buffer   = npcm_serial_flush_buffer,
	.set_ldisc      = npcm_serial_set_ldisc,
	.pm             = npcm_serial_pm,
	.release_port   = npcm_serial_release_port,
	.request_port   = npcm_serial_request_port,

};

#ifdef CONFIG_SERIAL_NPCM_CONSOLE
#define NPCM_SERIAL_CONSOLE (&npcm_serial_console)
#else
#define NPCM_SERIAL_CONSOLE NULL
#endif

static struct uart_driver npcm_uart_drv = {
	.owner          = THIS_MODULE,
	.dev_name       = NPCM_SERIAL_NAME,
	.nr             = NPCM_UART_NUM_OF_MODULES,
	.cons           = NPCM_SERIAL_CONSOLE,
	.driver_name    = NPCM_SERIAL_NAME,
	.major          = NPCM_SERIAL_MAJOR,
	.minor          = NPCM_SERIAL_MINOR,
};


static int npcm_serial_init_port(struct npcm_uart_port *ourport,
				    struct platform_device *platdev)
{
	struct uart_port *port      = &ourport->port;
	int ret = 0;

#ifdef CONFIG_OF
	struct resource *res;
	struct clk *ser_clk = NULL;
#endif

	NPCM_SERIAL_MSG("NPCM Serial platform: Called function %s\n", __func__);

	if (platdev == NULL)
		return -ENODEV;
	port->dev   = &platdev->dev;

#ifdef CONFIG_OF
	ser_clk = devm_clk_get(&platdev->dev, NULL);
	if (IS_ERR(ser_clk))
		return PTR_ERR(ser_clk);
	NPCM_SERIAL_MSG("\tserial clock is %ld\n", clk_get_rate(ser_clk));
	port->uartclk = clk_get_rate(ser_clk);
#else
	port->uartclk = 24 * _1MHz_;
#endif // CONFIG_OF

#ifdef CONFIG_OF
	res = platform_get_resource(platdev, IORESOURCE_MEM, 0);
	NPCM_SERIAL_MSG("\tmemory resource is 0x%lx,"
			   " statically it was 0x%lx\n", (unsigned long)res,
			   (unsigned long)port->membase);
	if (!request_mem_region(res->start, resource_size(res), platdev->name))
		ret = -EBUSY;
	else {
		dev_set_drvdata(&platdev->dev, res);
		port->membase = ioremap(res->start, resource_size(res));
	}
#else
	port->membase = (void *)UART_VIRT_BASE_ADDR(port->line);
#endif

	NPCM_SERIAL_MSG("\tmemory resource is 0x%lx\n",
			   (unsigned long)port->membase);
	port->irq       = platform_get_irq(platdev, 0);
	port->fifosize  = NPCM_SERIAL_TX_FIFO_SIZE;
	NPCM_SERIAL_MSG("NPCM Serial platform: Port %2d initialized mem=%08x, "
			"irq=%d, clock=%d\n", port->line,
			(int)npcm_serial_ports[port->line].port.membase,
			port->irq, port->uartclk);

	return ret;
}

static int npcm_serial_probe(struct platform_device *dev)
{
	struct npcm_uart_port *ourport;
	int ret;

#ifdef CONFIG_OF
	struct device_node *np;
#endif

	if (dev == NULL)
		return -ENODEV;
	NPCM_SERIAL_MSG("NPCM Serial platform: Called function %s with ID=%d\n"
			, __func__, dev->id);

#ifdef CONFIG_OF
	np = dev->dev.of_node;
	dev->id = of_alias_get_id(np, "serial");
	if (dev->id < 0)
		dev->id = 0;
#endif
	ourport = &npcm_serial_ports[dev->id];
	ret = npcm_serial_init_port(ourport, dev);
	if (ret < 0)
		return ret;
	NPCM_SERIAL_MSG("NPCM Serial platform: adding port number %d\n",
			   ourport->port.line);
	npcm_uart_drv.minor = NPCM_SERIAL_MINOR +  dev->id;
	uart_add_one_port(&npcm_uart_drv, &ourport->port);
	platform_set_drvdata(dev, &ourport->port);
	gcr_regmap = syscon_regmap_lookup_by_compatible("nuvoton,npcm750-gcr");

	if (IS_ERR(gcr_regmap)) {
		pr_err("%s: failed to find nuvoton,npcm750-gcr\n", __func__);
		return IS_ERR(gcr_regmap);
	}

	return 0;
}

static int npcm_serial_remove(struct platform_device *dev)
{
	struct uart_port *port = platform_get_drvdata(dev);

	NPCM_SERIAL_MSG("NPCM Serial platform: Called function %s\n",
			   __func__);
	if (port)
		uart_remove_one_port(&npcm_uart_drv, port);

	return 0;
}

static const struct of_device_id uart_dt_id[] = {
	{ .compatible = "nuvoton,npcm750-uart",  },
	{ },
};
MODULE_DEVICE_TABLE(of, uart_dt_id);

static struct platform_driver npcm_serial_drv = {
	.probe      = npcm_serial_probe,
	.remove     = npcm_serial_remove,
	.suspend    = NULL,
	.resume     = NULL,
	.driver     = {
		.name   = "npcm-uart",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(uart_dt_id),
	},
};

static int __init npcm_serial_modinit(void)
{
	NPCM_SERIAL_MSG("NPCM Serial platform: Called function %s\n",
			   __func__);

#ifndef CONFIG_OF
	platform_add_devices(npcm_uart_devs, ARRAY_SIZE(npcm_uart_devs));
#endif

	if (uart_register_driver(&npcm_uart_drv) < 0) {
		pr_err("NPCM Serial platform:"
		       " failed to register UART driver\n");
		return -1;
	}

	platform_driver_register(&npcm_serial_drv);

	return 0;
}

static void __exit npcm_serial_modexit(void)
{
	NPCM_SERIAL_MSG("NPCM Serial platform: Called function %s\n",
			   __func__);
	platform_driver_unregister(&npcm_serial_drv);
	uart_unregister_driver(&npcm_uart_drv);
}

module_init(npcm_serial_modinit);
module_exit(npcm_serial_modexit);

#ifdef CONFIG_SERIAL_NPCM_CONSOLE

static struct uart_port *npcm_console_port;

static void npcm_console_putchar(struct uart_port *port, int ch)
{
	npcm_uart_putc((NPCM_UART_DEV_T)port->line, ch);
}

static int __init npcm_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud    = NPCM_SERIAL_BAUD;
	int bits    = NPCM_SERIAL_BITS;
	int parity  = NPCM_SERIAL_PARITY;
	int flow    = 0;

	NPCM_SERIAL_MSG("NPCM Serial Console: Called function %s\n",
			   __func__);

	if (co->index == -1 || co->index >= NPCM_UART_NUM_OF_MODULES)
		co->index = NPCM_SERIAL_CONSOLE_PORT;
	port = &npcm_serial_ports[co->index].port;

	npcm_uart_init((NPCM_UART_DEV_T)co->index,
			  NPCM_UART_MUX_MODE3_HSP1_UART1__HSP2_UART2__UART3_SI2,
			  NPCM_SERIAL_BAUD);
	npcm_console_port = port;

	NPCM_SERIAL_MSG("NPCM Serial Console: Using port %d\n",
			   port->line);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static void npcm_console_write(struct console *co, const char *s,
				  unsigned int count)
{
	uart_console_write(npcm_console_port, s, count,
			   npcm_console_putchar);
}

static struct console npcm_serial_console = {
	.name       = NPCM_SERIAL_NAME,
	.device     = uart_console_device,
	.flags      = CON_PRINTBUFFER,
	.index      = -1,
	.write      = npcm_console_write,
	.setup      = npcm_console_setup,
	.data       = &npcm_uart_drv,
};

static int npcm_console_init(void)
{
	int    rc = 0;

#ifdef CONFIG_OF
	struct resource res;
	struct device_node *np = NULL;

#ifdef CLK_TREE_SUPPORT_IN_EARLY_INIT
	struct platform_device *pdev = NULL;
	struct clk *ser_clk = NULL;
#else
	npcm_serial_ports[NPCM_SERIAL_CONSOLE_PORT].port.uartclk = 24 * _1MHz_;
#endif

	np = of_find_node_by_name(NULL, "serial3");
	if (np == NULL)
		pr_err("%s- can't get device tree node\n", __func__);

	rc = of_address_to_resource(np, 0, &res);
	if (rc) {
		pr_info("%s: of_address_to_resource fail ret %d\n",
			__func__, rc);
		return -EINVAL;
	}

	npcm_serial_ports[NPCM_SERIAL_CONSOLE_PORT].port.membase =
		ioremap(res.start, resource_size(&res));

	if (!npcm_serial_ports[NPCM_SERIAL_CONSOLE_PORT].port.membase) {
		pr_info("%s:serial_virt_addr fail\n", __func__);
		return -ENOMEM;
	}

	NPCM_SERIAL_MSG("%s: console UART base is 0x%08X\n", __func__,
			   (u32)npcm_serial_ports[NPCM_SERIAL_CONSOLE_PORT].port.membase);

#ifdef CLK_TREE_SUPPORT_IN_EARLY_INIT
	pdev = of_find_device_by_node(np);
	ser_clk = devm_clk_get(&pdev->dev, NULL);

	if (IS_ERR(ser_clk))
		return PTR_ERR(ser_clk);

	NPCM_SERIAL_MSG("%s: serial clock is %ld\n", __func__,
			    clk_get_rate(ser_clk));

	npcm_serial_ports[NPCM_SERIAL_CONSOLE_PORT].port.uartclk =
		clk_get_rate(ser_clk);
#endif

#else
	npcm_serial_ports[NPCM_SERIAL_CONSOLE_PORT].port.uartclk =
		24 * _1MHz_;
	npcm_serial_ports[0].port.membase =
		(unsigned char *)UART_VIRT_BASE_ADDR(0);
	npcm_serial_ports[1].port.membase =
		(unsigned char *)UART_VIRT_BASE_ADDR(1);
	npcm_serial_ports[2].port.membase =
		(unsigned char *)UART_VIRT_BASE_ADDR(2);
	npcm_serial_ports[3].port.membase =
		(unsigned char *)UART_VIRT_BASE_ADDR(3);
#endif // CONFIG_OF

	register_console(&npcm_serial_console);
	pr_err("NPCM Console Initialized\n");

	return 0;
}

console_initcall(npcm_console_init);

#endif /* CONFIG_SERIAL_NPCM_CONSOLE */


