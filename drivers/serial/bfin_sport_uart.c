/*
 * Blackfin On-Chip Sport Emulated UART Driver
 *
 * Copyright 2006-2008 Analog Devices Inc.
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

/*
 * This driver and the hardware supported are in term of EE-191 of ADI.
 * http://www.analog.com/UploadedFiles/Application_Notes/399447663EE191.pdf
 * This application note describe how to implement a UART on a Sharc DSP,
 * but this driver is implemented on Blackfin Processor.
 * Transmit Frame Sync is not used by this driver to transfer data out.
 */

/* #define DEBUG */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>

#include <asm/delay.h>
#include <asm/portmux.h>

#include "bfin_sport_uart.h"

unsigned short bfin_uart_pin_req_sport0[] =
	{P_SPORT0_TFS, P_SPORT0_DTPRI, P_SPORT0_TSCLK, P_SPORT0_RFS, \
	 P_SPORT0_DRPRI, P_SPORT0_RSCLK, P_SPORT0_DRSEC, P_SPORT0_DTSEC, 0};

unsigned short bfin_uart_pin_req_sport1[] =
	{P_SPORT1_TFS, P_SPORT1_DTPRI, P_SPORT1_TSCLK, P_SPORT1_RFS, \
	P_SPORT1_DRPRI, P_SPORT1_RSCLK, P_SPORT1_DRSEC, P_SPORT1_DTSEC, 0};

#define DRV_NAME "bfin-sport-uart"
#define DEVICE_NAME	"ttySS"

struct sport_uart_port {
	struct uart_port	port;
	char			*name;
	int			tx_irq;
	int			rx_irq;
	int			err_irq;
	unsigned short		csize;
	unsigned short		rxmask;
	unsigned short		txmask1;
	unsigned short		txmask2;
	unsigned char		stopb;
/*	unsigned char		parib; */
};

static void sport_uart_tx_chars(struct sport_uart_port *up);
static void sport_stop_tx(struct uart_port *port);

static inline void tx_one_byte(struct sport_uart_port *up, unsigned int value)
{
	pr_debug("%s value:%x, mask1=0x%x, mask2=0x%x\n", __func__, value,
		up->txmask1, up->txmask2);

	/* Place Start and Stop bits */
	__asm__ __volatile__ (
		"%[val] <<= 1;"
		"%[val] = %[val] & %[mask1];"
		"%[val] = %[val] | %[mask2];"
		: [val]"+d"(value)
		: [mask1]"d"(up->txmask1), [mask2]"d"(up->txmask2)
		: "ASTAT"
	);
	pr_debug("%s value:%x\n", __func__, value);

	SPORT_PUT_TX(up, value);
}

static inline unsigned char rx_one_byte(struct sport_uart_port *up)
{
	unsigned int value;
	unsigned char extract;
	u32 tmp_mask1, tmp_mask2, tmp_shift, tmp;

	if ((up->csize + up->stopb) > 7)
		value = SPORT_GET_RX32(up);
	else
		value = SPORT_GET_RX(up);

	pr_debug("%s value:%x, cs=%d, mask=0x%x\n", __func__, value,
		up->csize, up->rxmask);

	/* Extract data */
	__asm__ __volatile__ (
		"%[extr] = 0;"
		"%[mask1] = %[rxmask];"
		"%[mask2] = 0x0200(Z);"
		"%[shift] = 0;"
		"LSETUP(.Lloop_s, .Lloop_e) LC0 = %[lc];"
		".Lloop_s:"
		"%[tmp] = extract(%[val], %[mask1].L)(Z);"
		"%[tmp] <<= %[shift];"
		"%[extr] = %[extr] | %[tmp];"
		"%[mask1] = %[mask1] - %[mask2];"
		".Lloop_e:"
		"%[shift] += 1;"
		: [extr]"=&d"(extract), [shift]"=&d"(tmp_shift), [tmp]"=&d"(tmp),
		  [mask1]"=&d"(tmp_mask1), [mask2]"=&d"(tmp_mask2)
		: [val]"d"(value), [rxmask]"d"(up->rxmask), [lc]"a"(up->csize)
		: "ASTAT", "LB0", "LC0", "LT0"
	);

	pr_debug("	extract:%x\n", extract);
	return extract;
}

static int sport_uart_setup(struct sport_uart_port *up, int size, int baud_rate)
{
	int tclkdiv, rclkdiv;
	unsigned int sclk = get_sclk();

	/* Set TCR1 and TCR2, TFSR is not enabled for uart */
	SPORT_PUT_TCR1(up, (ITFS | TLSBIT | ITCLK));
	SPORT_PUT_TCR2(up, size + 1);
	pr_debug("%s TCR1:%x, TCR2:%x\n", __func__, SPORT_GET_TCR1(up), SPORT_GET_TCR2(up));

	/* Set RCR1 and RCR2 */
	SPORT_PUT_RCR1(up, (RCKFE | LARFS | LRFS | RFSR | IRCLK));
	SPORT_PUT_RCR2(up, (size + 1) * 2 - 1);
	pr_debug("%s RCR1:%x, RCR2:%x\n", __func__, SPORT_GET_RCR1(up), SPORT_GET_RCR2(up));

	tclkdiv = sclk/(2 * baud_rate) - 1;
	rclkdiv = sclk/(2 * baud_rate * 2) - 1;
	SPORT_PUT_TCLKDIV(up, tclkdiv);
	SPORT_PUT_RCLKDIV(up, rclkdiv);
	SSYNC();
	pr_debug("%s sclk:%d, baud_rate:%d, tclkdiv:%d, rclkdiv:%d\n",
			__func__, sclk, baud_rate, tclkdiv, rclkdiv);

	return 0;
}

static irqreturn_t sport_uart_rx_irq(int irq, void *dev_id)
{
	struct sport_uart_port *up = dev_id;
	struct tty_struct *tty = up->port.info->port.tty;
	unsigned char ch;

	spin_lock(&up->port.lock);

	while (SPORT_GET_STAT(up) & RXNE) {
		ch = rx_one_byte(up);
		up->port.icount.rx++;

		if (uart_handle_sysrq_char(&up->port, ch))
			;
		else
			tty_insert_flip_char(tty, ch, TTY_NORMAL);
	};
	tty_flip_buffer_push(tty);

	spin_unlock(&up->port.lock);

	return IRQ_HANDLED;
}

static irqreturn_t sport_uart_tx_irq(int irq, void *dev_id)
{
	struct sport_uart_port *up = dev_id;

	spin_lock(&up->port.lock);
	sport_uart_tx_chars(up);
	spin_unlock(&up->port.lock);

	return IRQ_HANDLED;
}

static irqreturn_t sport_uart_err_irq(int irq, void *dev_id)
{
	struct sport_uart_port *up = dev_id;
	struct tty_struct *tty = up->port.info->port.tty;
	unsigned int stat = SPORT_GET_STAT(up);

	spin_lock(&up->port.lock);

	/* Overflow in RX FIFO */
	if (stat & ROVF) {
		up->port.icount.overrun++;
		tty_insert_flip_char(tty, 0, TTY_OVERRUN);
		SPORT_PUT_STAT(up, ROVF); /* Clear ROVF bit */
	}
	/* These should not happen */
	if (stat & (TOVF | TUVF | RUVF)) {
		printk(KERN_ERR "SPORT Error:%s %s %s\n",
				(stat & TOVF)?"TX overflow":"",
				(stat & TUVF)?"TX underflow":"",
				(stat & RUVF)?"RX underflow":"");
		SPORT_PUT_TCR1(up, SPORT_GET_TCR1(up) & ~TSPEN);
		SPORT_PUT_RCR1(up, SPORT_GET_RCR1(up) & ~RSPEN);
	}
	SSYNC();

	spin_unlock(&up->port.lock);
	return IRQ_HANDLED;
}

/* Reqeust IRQ, Setup clock */
static int sport_startup(struct uart_port *port)
{
	struct sport_uart_port *up = (struct sport_uart_port *)port;
	char buffer[20];
	int retval;

	pr_debug("%s enter\n", __func__);
	memset(buffer, 20, '\0');
	snprintf(buffer, 20, "%s rx", up->name);
	retval = request_irq(up->rx_irq, sport_uart_rx_irq, IRQF_SAMPLE_RANDOM, buffer, up);
	if (retval) {
		printk(KERN_ERR "Unable to request interrupt %s\n", buffer);
		return retval;
	}

	snprintf(buffer, 20, "%s tx", up->name);
	retval = request_irq(up->tx_irq, sport_uart_tx_irq, IRQF_SAMPLE_RANDOM, buffer, up);
	if (retval) {
		printk(KERN_ERR "Unable to request interrupt %s\n", buffer);
		goto fail1;
	}

	snprintf(buffer, 20, "%s err", up->name);
	retval = request_irq(up->err_irq, sport_uart_err_irq, IRQF_SAMPLE_RANDOM, buffer, up);
	if (retval) {
		printk(KERN_ERR "Unable to request interrupt %s\n", buffer);
		goto fail2;
	}

	return 0;
fail2:
	free_irq(up->tx_irq, up);
fail1:
	free_irq(up->rx_irq, up);

	return retval;

}

static void sport_uart_tx_chars(struct sport_uart_port *up)
{
	struct circ_buf *xmit = &up->port.info->xmit;

	if (SPORT_GET_STAT(up) & TXF)
		return;

	if (up->port.x_char) {
		tx_one_byte(up, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		sport_stop_tx(&up->port);
		return;
	}

	while(!(SPORT_GET_STAT(up) & TXF) && !uart_circ_empty(xmit)) {
		tx_one_byte(up, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE -1);
		up->port.icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);
}

static unsigned int sport_tx_empty(struct uart_port *port)
{
	struct sport_uart_port *up = (struct sport_uart_port *)port;
	unsigned int stat;

	stat = SPORT_GET_STAT(up);
	pr_debug("%s stat:%04x\n", __func__, stat);
	if (stat & TXHRE) {
		return TIOCSER_TEMT;
	} else
		return 0;
}

static unsigned int sport_get_mctrl(struct uart_port *port)
{
	pr_debug("%s enter\n", __func__);
	return (TIOCM_CTS | TIOCM_CD | TIOCM_DSR);
}

static void sport_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	pr_debug("%s enter\n", __func__);
}

static void sport_stop_tx(struct uart_port *port)
{
	struct sport_uart_port *up = (struct sport_uart_port *)port;

	pr_debug("%s enter\n", __func__);

	/* Although the hold register is empty, last byte is still in shift
	 * register and not sent out yet. So, put a dummy data into TX FIFO.
	 * Then, sport tx stops when last byte is shift out and the dummy
	 * data is moved into the shift register.
	 */
	SPORT_PUT_TX(up, 0xffff);
	while (!(SPORT_GET_STAT(up) & TXHRE))
		cpu_relax();

	SPORT_PUT_TCR1(up, (SPORT_GET_TCR1(up) & ~TSPEN));
	SSYNC();

	return;
}

static void sport_start_tx(struct uart_port *port)
{
	struct sport_uart_port *up = (struct sport_uart_port *)port;

	pr_debug("%s enter\n", __func__);

	/* Write data into SPORT FIFO before enable SPROT to transmit */
	sport_uart_tx_chars(up);

	/* Enable transmit, then an interrupt will generated */
	SPORT_PUT_TCR1(up, (SPORT_GET_TCR1(up) | TSPEN));
	SSYNC();
	pr_debug("%s exit\n", __func__);
}

static void sport_stop_rx(struct uart_port *port)
{
	struct sport_uart_port *up = (struct sport_uart_port *)port;

	pr_debug("%s enter\n", __func__);
	/* Disable sport to stop rx */
	SPORT_PUT_RCR1(up, (SPORT_GET_RCR1(up) & ~RSPEN));
	SSYNC();
}

static void sport_enable_ms(struct uart_port *port)
{
	pr_debug("%s enter\n", __func__);
}

static void sport_break_ctl(struct uart_port *port, int break_state)
{
	pr_debug("%s enter\n", __func__);
}

static void sport_shutdown(struct uart_port *port)
{
	struct sport_uart_port *up = (struct sport_uart_port *)port;

	pr_debug("%s enter\n", __func__);

	/* Disable sport */
	SPORT_PUT_TCR1(up, (SPORT_GET_TCR1(up) & ~TSPEN));
	SPORT_PUT_RCR1(up, (SPORT_GET_RCR1(up) & ~RSPEN));
	SSYNC();

	free_irq(up->rx_irq, up);
	free_irq(up->tx_irq, up);
	free_irq(up->err_irq, up);
}

static const char *sport_type(struct uart_port *port)
{
	struct sport_uart_port *up = (struct sport_uart_port *)port;

	pr_debug("%s enter\n", __func__);
	return up->name;
}

static void sport_release_port(struct uart_port *port)
{
	pr_debug("%s enter\n", __func__);
}

static int sport_request_port(struct uart_port *port)
{
	pr_debug("%s enter\n", __func__);
	return 0;
}

static void sport_config_port(struct uart_port *port, int flags)
{
	struct sport_uart_port *up = (struct sport_uart_port *)port;

	pr_debug("%s enter\n", __func__);
	up->port.type = PORT_BFIN_SPORT;
}

static int sport_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	pr_debug("%s enter\n", __func__);
	return 0;
}

static void sport_set_termios(struct uart_port *port,
		struct ktermios *termios, struct ktermios *old)
{
	struct sport_uart_port *up = (struct sport_uart_port *)port;
	unsigned long flags;
	int i;

	pr_debug("%s enter, c_cflag:%08x\n", __func__, termios->c_cflag);

	switch (termios->c_cflag & CSIZE) {
	case CS8:
		up->csize = 8;
		break;
	case CS7:
		up->csize = 7;
		break;
	case CS6:
		up->csize = 6;
		break;
	case CS5:
		up->csize = 5;
		break;
	default:
		printk(KERN_ERR "%s: word lengh not supported\n",
			__func__);
	}

	if (termios->c_cflag & CSTOPB) {
		up->stopb = 1;
	}
	if (termios->c_cflag & PARENB) {
		printk(KERN_WARNING "PAREN bits is not supported yet.\n");
/*		up->parib = 1; */
	}

	port->read_status_mask = OE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= (FE | PE);
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= BI;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= FE | PE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= OE;
	}

	/* RX extract mask */
	up->rxmask = 0x01 | (((up->csize + up->stopb) * 2 - 1) << 0x8);
	/* TX masks, 8 bit data and 1 bit stop for example:
	   mask1 = b#0111111110
	   mask2 = b#1000000000
	 */
	for (i = 0, up->txmask1 = 0; i < up->csize; i++)
		up->txmask1 |= (1<<i);
	up->txmask2 = (1<<i);
	if (up->stopb) {
		++i;
		up->txmask2 |= (1<<i);
	}
	up->txmask1 <<= 1;
	up->txmask2 <<= 1;
	/* uart baud rate */
	port->uartclk = uart_get_baud_rate(port, termios, old, 0, get_sclk()/16);

	spin_lock_irqsave(&up->port.lock, flags);

	/* Disable UART */
	SPORT_PUT_TCR1(up, SPORT_GET_TCR1(up) & ~TSPEN);
	SPORT_PUT_RCR1(up, SPORT_GET_RCR1(up) & ~RSPEN);

	sport_uart_setup(up, up->csize + up->stopb, port->uartclk);

	/* driver TX line high after config, one dummy data is
	 * necessary to stop sport after shift one byte
	 */
	SPORT_PUT_TX(up, 0xffff);
	SPORT_PUT_TX(up, 0xffff);
	SPORT_PUT_TCR1(up, (SPORT_GET_TCR1(up) | TSPEN));
	SSYNC();
	while (!(SPORT_GET_STAT(up) & TXHRE))
		cpu_relax();
	SPORT_PUT_TCR1(up, SPORT_GET_TCR1(up) & ~TSPEN);
	SSYNC();

	/* Port speed changed, update the per-port timeout. */
	uart_update_timeout(port, termios->c_cflag, port->uartclk);

	/* Enable sport rx */
	SPORT_PUT_RCR1(up, SPORT_GET_RCR1(up) | RSPEN);
	SSYNC();

	spin_unlock_irqrestore(&up->port.lock, flags);
}

struct uart_ops sport_uart_ops = {
	.tx_empty	= sport_tx_empty,
	.set_mctrl	= sport_set_mctrl,
	.get_mctrl	= sport_get_mctrl,
	.stop_tx	= sport_stop_tx,
	.start_tx	= sport_start_tx,
	.stop_rx	= sport_stop_rx,
	.enable_ms	= sport_enable_ms,
	.break_ctl	= sport_break_ctl,
	.startup	= sport_startup,
	.shutdown	= sport_shutdown,
	.set_termios	= sport_set_termios,
	.type		= sport_type,
	.release_port	= sport_release_port,
	.request_port	= sport_request_port,
	.config_port	= sport_config_port,
	.verify_port	= sport_verify_port,
};

static struct sport_uart_port sport_uart_ports[] = {
	{ /* SPORT 0 */
		.name	= "SPORT0",
		.tx_irq = IRQ_SPORT0_TX,
		.rx_irq = IRQ_SPORT0_RX,
		.err_irq= IRQ_SPORT0_ERROR,
		.port	= {
			.membase	= (void __iomem *)SPORT0_TCR1,
			.mapbase	= SPORT0_TCR1,
			.irq		= IRQ_SPORT0_RX,
		},
	}, { /* SPORT 1 */
		.name	= "SPORT1",
		.tx_irq = IRQ_SPORT1_TX,
		.rx_irq = IRQ_SPORT1_RX,
		.err_irq= IRQ_SPORT1_ERROR,
		.port	= {
			.membase	= (void __iomem *)SPORT1_TCR1,
			.mapbase	= SPORT1_TCR1,
			.irq		= IRQ_SPORT1_RX,
		},
	}
};

static int nr_active_ports = ARRAY_SIZE(sport_uart_ports);

static int __init sport_uart_init_ports(void)
{
	static int first = 1;
	int i;

	if (!first)
		return 0;
	first = 0;

	if (peripheral_request_list(bfin_uart_pin_req_sport0, DRV_NAME)) {
		printk(KERN_ERR DRV_NAME
			": Requesting Peripherals on sport0 failed\n");
		return -EBUSY;
	}
	if (peripheral_request_list(bfin_uart_pin_req_sport1, DRV_NAME)) {
		peripheral_free_list(bfin_uart_pin_req_sport0);
		printk(KERN_ERR DRV_NAME
			": Requesting Peripherals on sport 1 failed\n");
		return -EBUSY;
	}

	for (i = 0; i < nr_active_ports; i++) {
		spin_lock_init(&sport_uart_ports[i].port.lock);
		sport_uart_ports[i].port.fifosize  = SPORT_TX_FIFO_SIZE,
		sport_uart_ports[i].port.ops       = &sport_uart_ops;
		sport_uart_ports[i].port.line      = i;
		sport_uart_ports[i].port.iotype    = UPIO_MEM;
		sport_uart_ports[i].port.flags     = UPF_BOOT_AUTOCONF;
	}

	return 0;
}

#ifdef CONFIG_SERIAL_BFIN_SPORT_CONSOLE
static int __init
sport_uart_console_setup(struct console *co, char *options)
{
	struct sport_uart_port *up;
	int baud = 57600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index == -1 || co->index >= nr_active_ports)
		co->index = 0;
	up = &sport_uart_ports[co->index];

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&up->port, co, baud, parity, bits, flow);
}

static void sport_uart_console_putchar(struct uart_port *port, int ch)
{
	struct sport_uart_port *up = (struct sport_uart_port *)port;

	while (SPORT_GET_STAT(up) & TXF)
		barrier();

	tx_one_byte(up, ch);
}

/*
 * Interrupts are disabled on entering
 */
static void
sport_uart_console_write(struct console *co, const char *s, unsigned int count)
{
	struct sport_uart_port *up = &sport_uart_ports[co->index];
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);

	if (SPORT_GET_TCR1(up) & TSPEN)
		uart_console_write(&up->port, s, count, sport_uart_console_putchar);
	else {
		/* dummy data to start sport */
		while (SPORT_GET_STAT(up) & TXF)
			barrier();
		SPORT_PUT_TX(up, 0xffff);
		/* Enable transmit, then an interrupt will generated */
		SPORT_PUT_TCR1(up, (SPORT_GET_TCR1(up) | TSPEN));
		SSYNC();

		uart_console_write(&up->port, s, count, sport_uart_console_putchar);

		/* Although the hold register is empty, last byte is still in shift
		 * register and not sent out yet. So, put a dummy data into TX FIFO.
		 * Then, sport tx stops when last byte is shift out and the dummy
		 * data is moved into the shift register.
		 */
		while (SPORT_GET_STAT(up) & TXF)
			barrier();
		SPORT_PUT_TX(up, 0xffff);
		while (!(SPORT_GET_STAT(up) & TXHRE))
			barrier();

		/* Stop sport tx transfer */
		SPORT_PUT_TCR1(up, (SPORT_GET_TCR1(up) & ~TSPEN));
		SSYNC();
	}

	spin_unlock_irqrestore(&up->port.lock, flags);

}

static struct uart_driver sport_uart_reg;

static struct console sport_uart_console = {
	.name		= DEVICE_NAME,
	.write		= sport_uart_console_write,
	.device		= uart_console_device,
	.setup		= sport_uart_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &sport_uart_reg,
};

static int __init sport_uart_rs_console_init(void)
{
	if (sport_uart_init_ports())
		return -EBUSY;

	register_console(&sport_uart_console);

	return 0;
}
console_initcall(sport_uart_rs_console_init);

#define SPORT_UART_CONSOLE	(&sport_uart_console)
#else
#define SPORT_UART_CONSOLE	NULL
#endif /* CONFIG_SERIAL_BFIN_CONSOLE */



static struct uart_driver sport_uart_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= DRV_NAME,
	.dev_name	= DEVICE_NAME,
	.major		= 204,
	.minor		= 84,
	.nr		= ARRAY_SIZE(sport_uart_ports),
	.cons		= SPORT_UART_CONSOLE,
};

static int sport_uart_suspend(struct platform_device *dev, pm_message_t state)
{
	struct sport_uart_port *sport = platform_get_drvdata(dev);

	pr_debug("%s enter\n", __func__);
	if (sport)
		uart_suspend_port(&sport_uart_reg, &sport->port);

	return 0;
}

static int sport_uart_resume(struct platform_device *dev)
{
	struct sport_uart_port *sport = platform_get_drvdata(dev);

	pr_debug("%s enter\n", __func__);
	if (sport)
		uart_resume_port(&sport_uart_reg, &sport->port);

	return 0;
}

static int sport_uart_probe(struct platform_device *dev)
{
	pr_debug("%s enter\n", __func__);
	sport_uart_ports[dev->id].port.dev = &dev->dev;
	uart_add_one_port(&sport_uart_reg, &sport_uart_ports[dev->id].port);
	platform_set_drvdata(dev, &sport_uart_ports[dev->id]);

	return 0;
}

static int sport_uart_remove(struct platform_device *dev)
{
	struct sport_uart_port *sport = platform_get_drvdata(dev);

	pr_debug("%s enter\n", __func__);
	platform_set_drvdata(dev, NULL);

	if (sport)
		uart_remove_one_port(&sport_uart_reg, &sport->port);

	return 0;
}

static struct platform_driver sport_uart_driver = {
	.probe		= sport_uart_probe,
	.remove		= sport_uart_remove,
	.suspend	= sport_uart_suspend,
	.resume		= sport_uart_resume,
	.driver		= {
		.name	= DRV_NAME,
	},
};

static int __init sport_uart_init(void)
{
	int ret;

	pr_debug("%s enter\n", __func__);

	if (sport_uart_init_ports())
		return -EBUSY;

	ret = uart_register_driver(&sport_uart_reg);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register %s:%d\n",
				sport_uart_reg.driver_name, ret);
		return ret;
	}

	ret = platform_driver_register(&sport_uart_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register sport uart driver:%d\n", ret);
		uart_unregister_driver(&sport_uart_reg);
	}


	pr_debug("%s exit\n", __func__);
	return ret;
}

static void __exit sport_uart_exit(void)
{
	pr_debug("%s enter\n", __func__);
	platform_driver_unregister(&sport_uart_driver);
	uart_unregister_driver(&sport_uart_reg);

	peripheral_free_list(bfin_uart_pin_req_sport1);
	peripheral_free_list(bfin_uart_pin_req_sport0);
}

module_init(sport_uart_init);
module_exit(sport_uart_exit);

MODULE_LICENSE("GPL");
