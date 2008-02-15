/*
 * Blackfin Infra-red Driver
 *
 * Copyright 2006-2008 Analog Devices Inc.
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 *
 */

#include <linux/serial_core.h>
#include <asm/dma.h>
#include <asm/portmux.h>
#include <asm/mach/bfin_serial_5xx.h>

#define SIR_UART_GET_CHAR(port)   bfin_read16((port)->membase + OFFSET_RBR)
#define SIR_UART_GET_DLL(port)    bfin_read16((port)->membase + OFFSET_DLL)
#define SIR_UART_GET_IER(port)    bfin_read16((port)->membase + OFFSET_IER)
#define SIR_UART_GET_DLH(port)    bfin_read16((port)->membase + OFFSET_DLH)
#define SIR_UART_GET_IIR(port)    bfin_read16((port)->membase + OFFSET_IIR)
#define SIR_UART_GET_LCR(port)    bfin_read16((port)->membase + OFFSET_LCR)
#define SIR_UART_GET_GCTL(port)   bfin_read16((port)->membase + OFFSET_GCTL)

#define SIR_UART_PUT_CHAR(port, v) bfin_write16(((port)->membase + OFFSET_THR), v)
#define SIR_UART_PUT_DLL(port, v)  bfin_write16(((port)->membase + OFFSET_DLL), v)
#define SIR_UART_PUT_IER(port, v)  bfin_write16(((port)->membase + OFFSET_IER), v)
#define SIR_UART_PUT_DLH(port, v)  bfin_write16(((port)->membase + OFFSET_DLH), v)
#define SIR_UART_PUT_LCR(port, v)  bfin_write16(((port)->membase + OFFSET_LCR), v)
#define SIR_UART_PUT_GCTL(port, v) bfin_write16(((port)->membase + OFFSET_GCTL), v)

struct bfin_sir_port {
    unsigned char __iomem   *membase;
    unsigned int            irq;
    unsigned int            lsr;
    unsigned long           clk;
    struct net_device       *dev;
};

struct bfin_sir_port sir_ports[NR_PORTS];

struct bfin_sir_port_res {
    unsigned long   base_addr;
    int             irq;
};

struct bfin_sir_port_res bfin_sir_port_resource[] = {
#ifdef CONFIG_BFIN_SIR0
    {
    0xFFC00400,
    IRQ_UART0_RX,
    },
#endif
#ifdef CONFIG_BFIN_SIR1
    {
    0xFFC02000,
    IRQ_UART1_RX,
    },
#endif
};

int nr_sirs = ARRAY_SIZE(bfin_sir_port_resource);

struct bfin_sir_self {
    struct bfin_sir_port    *sir_port;
    spinlock_t              lock;
    unsigned int            open;
    int                     speed;
    int                     newspeed;

    struct sk_buff          *txskb;
    struct sk_buff          *rxskb;
    struct net_device_stats stats;
    struct device           *dev;
    struct irlap_cb         *irlap;
    struct qos_info         qos;

    iobuff_t                tx_buff;
    iobuff_t                rx_buff;
};

static inline unsigned int SIR_UART_GET_LSR(struct bfin_sir_port *port)
{
    unsigned int lsr = bfin_read16(port->membase + OFFSET_LSR);
    port->lsr |= (lsr & (BI|FE|PE|OE));
    return lsr | port->lsr;
}

static inline void SIR_UART_CLEAR_LSR(struct bfin_sir_port *port)
{
    port->lsr = 0;
    bfin_read16(port->membase + OFFSET_LSR);
}

