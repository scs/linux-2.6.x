/*
 * arch/bfinnommu/mach-bf533/ints-priority.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 *
 * Sep 2003, Changed to support BlackFin BF533.
 *
 * June 2004, Support for Priority based Interrupt handling for Blackfin 
 *		by LG Soft India.
 *
 * Copyright 1996 Roman Zippel
 * Copyright 1999 D. Jeff Dionne <jeff@uclinux.org>
 * Copyright 2000-2001 Lineo, Inc. D. Jefff Dionne <jeff@lineo.ca>
 * Copyright 2002 Arcturus Networks Inc. MaTed <mated@sympatico.ca>
 * Copyright 2003 Metrowerks/Motorola
 * Copyright 2003 Bas Vermeulen <bas@buyways.nl>,
 *                BuyWays B.V. (www.buyways.nl)
 * Copyright 2004 LG Soft India 
 */

#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/seq_file.h>
#include <asm/irqchip.h>
#include <asm/traps.h>
#include <asm/blackfin.h>


/********************************************************************
 * NOTES:
 * - we have separated the physical Hardware interrupt from the
 * levels that the LINUX kernel sees (see the description in irq.h)
 * - 
 ********************************************************************/

#define INTERNAL_IRQS (32)

volatile unsigned long irq_flags = 0;

/* The number of spurious interrupts */
volatile unsigned int num_spurious;

struct ivgx	{
	int irqno;	/*irq number for request_irq, available in bf533_irq.h*/
	int isrflag;	/*corresponding bit in the SIC_ISR register*/
}ivg_table[23];

struct ivg_slice {
	struct ivgx *ifirst; /* position of first irq in ivg_table for given ivg */
	struct ivgx *istop;  
} ivg7_13[16];

extern void dump(struct pt_regs * regs);

/* BASE LEVEL interrupt handler routines */
asmlinkage void evt_nmi(void);
asmlinkage void evt_exception(void);
asmlinkage void trap(void);
asmlinkage void evt_ivhw(void);
asmlinkage void evt_timer(void);
asmlinkage void evt_evt2(void);
asmlinkage void evt_evt7(void);
asmlinkage void evt_evt8(void);
asmlinkage void evt_evt9(void);
asmlinkage void evt_evt10(void);
asmlinkage void evt_evt11(void);
asmlinkage void evt_evt12(void);
asmlinkage void evt_evt13(void);
asmlinkage void evt_soft_int1(void);
asmlinkage void evt_system_call(void);

static void program_IAR(void);
static void search_IAR(void);	

/*********
 * irq_panic
 * - calls panic with string setup
 *********/
asmlinkage void irq_panic( int reason, struct pt_regs * regs)
{
	int sig = 0;
	siginfo_t info;

  	printk("\n\nException: IRQ 0x%x entered\n", reason);
	printk(" code=[0x%08x],  ", (unsigned int)regs->seqstat);
	printk(" stack frame=0x%04x,  ",(unsigned int)(unsigned long) regs);
	printk(" bad PC=0x%04x\n", (unsigned int)regs->pc);
	if(reason == 0x5) {
 
 	printk("\n----------- HARDWARE ERROR -----------\n\n");
		
	/* There is only need to check for Hardware Errors, since other EXCEPTIONS are handled in TRAPS.c (MH)  */
	switch(((unsigned int)regs->seqstat) >> 14) {
		case (0x2):			//System MMR Error
			info.si_code = BUS_ADRALN;
			sig = SIGBUS;
			printk(HWC_x2);
			break;
		case (0x3):			//External Memory Addressing Error
		        info.si_code = BUS_ADRERR;
			sig = SIGBUS;
			printk(HWC_x3);
			break;
		case (0x12):			//Performance Monitor Overflow
			printk(HWC_x12);
			break;
		case (0x18):			//RAISE 5 instruction
			printk(HWC_x18);
			break;
		default:			//Reserved
			printk(HWC_default);
			break;
		}
	}

	regs->ipend = *pIPEND;
	dump(regs);
	if (0 == (info.si_signo = sig) || 
	    0 == user_mode(regs)) /* in kernelspace */
	    panic("Unhandled IRQ or exceptions!\n");
	else { /* in userspace */
	    info.si_errno = 0;
	    info.si_addr = (void *) regs->pc;
	    force_sig_info (sig, &info, current);
        }
}

/*Program the IAR registers*/
static void __init program_IAR()
{
		/* Program the IAR0 Register with the configured priority */
	        *pSIC_IAR0 =  ((CONFIG_PLLWAKE_ERROR-7) << PLLWAKE_ERROR_POS) |
                ((CONFIG_DMA_ERROR   -7) <<    DMA_ERROR_POS) |
                ((CONFIG_PPI_ERROR   -7) <<    PPI_ERROR_POS) |
                ((CONFIG_SPORT0_ERROR-7) << SPORT0_ERROR_POS) |
                ((CONFIG_SPI_ERROR   -7) <<    SPI_ERROR_POS) |
                ((CONFIG_SPORT1_ERROR-7) << SPORT1_ERROR_POS) |
                ((CONFIG_UART_ERROR  -7) <<   UART_ERROR_POS) |
                ((CONFIG_RTC_ERROR   -7) <<    RTC_ERROR_POS);
	        asm("ssync;");	

		*pSIC_IAR1 =	((CONFIG_DMA0_PPI-7)    << DMA0_PPI_POS) |
                ((CONFIG_DMA1_SPORT0RX-7) << DMA1_SPORT0RX_POS) |
                ((CONFIG_DMA2_SPORT0TX-7) << DMA2_SPORT0TX_POS) |
                ((CONFIG_DMA3_SPORT1RX-7) << DMA3_SPORT1RX_POS) |
                ((CONFIG_DMA4_SPORT1TX-7) << DMA4_SPORT1TX_POS) |
                ((CONFIG_DMA5_SPI-7)    << DMA5_SPI_POS)    |
                ((CONFIG_DMA6_UARTRX-7) << DMA6_UARTRX_POS) |
                ((CONFIG_DMA7_UARTTX-7) << DMA7_UARTTX_POS);
	        asm("ssync;");	
 
		*pSIC_IAR2 =	((CONFIG_TIMER0-7) << TIMER0_POS) |
		((CONFIG_TIMER1-7) << TIMER1_POS) |
		((CONFIG_TIMER2-7) << TIMER2_POS) |
		((CONFIG_PFA-7) << PFA_POS) |
		((CONFIG_PFB-7) << PFB_POS) |
		((CONFIG_MEMDMA0-7) << MEMDMA0_POS) |
		((CONFIG_MEMDMA1-7) << MEMDMA1_POS) |
		((CONFIG_WDTIMER-7) << WDTIMER_POS);
	        asm("ssync;");	
}	/*End of program_IAR*/

/* Search SIC_IAR and fill tables with the irqvalues 
and their positions in the SIC_ISR register */

static void __init search_IAR(void)	
{
    unsigned ivg, irq_pos = 0;
    for(ivg = IVG7; ivg <= IVG13; ivg++)
    {
        int irqn;

        ivg7_13[ivg].istop = 
        ivg7_13[ivg].ifirst = &ivg_table[irq_pos];        
          
        for(irqn = 0; irqn < 24; irqn++)
          if (ivg == IVG7 + (0x0f & pSIC_IAR0[irqn >> 3] >> (irqn & 7) * 4))
          {
             ivg_table[irq_pos].irqno = IVG7 + irqn;
             ivg_table[irq_pos].isrflag = 1 << irqn;
             ivg7_13[ivg].istop++;
             irq_pos++;
          }
    }
}		
			
/*
 * This is for BF533 internal IRQs
 */

static void bf533_core_mask_irq(unsigned int irq)
{
	local_irq_disable();
	irq_flags &= ~(1<<irq);
	local_irq_enable();
}

static void bf533_core_unmask_irq(unsigned int irq)
{
	/* enable the interrupt */
	local_irq_disable();
	irq_flags |= 1<<irq;
	local_irq_enable();
	return;
}

static void bf533_internal_mask_irq(unsigned int irq)
{
	unsigned long irq_mask;

	/*
 	 * If it is the interrupt for peripheral,
	 * we only disable it in SIC_IMASK register.
	 * No need to change IMASK register of CORE,
	 * since all of the IVG for peripherals was 
 	 * enabled in init_IRQ()
	 *
	 */
	local_irq_disable();
	irq_mask = (1<<(irq - (IRQ_CORETMR + 1)));
   	*pSIC_IMASK &= ~(irq_mask); 
	asm("ssync;");
	local_irq_enable();
}

static void bf533_internal_unmask_irq(unsigned int irq)
{
	unsigned long irq_mask;
	local_irq_disable();
	irq_mask = (1<<(irq - (IRQ_CORETMR+1)));
   	*pSIC_IMASK |= irq_mask;
	asm("ssync;");
	local_irq_enable();
}

static struct irqchip bf533_core_irqchip = {
	.ack		= bf533_core_mask_irq,
	.mask		= bf533_core_mask_irq,
	.unmask		= bf533_core_unmask_irq,
};

static struct irqchip bf533_internal_irqchip = {
	.ack		= bf533_internal_mask_irq,
	.mask		= bf533_internal_mask_irq,
	.unmask		= bf533_internal_unmask_irq,
};

#ifdef CONFIG_IRQCHIP_DEMUX_GPIO
static int gpio_enabled;
static int gpio_edge_triggered;

static void bf533_gpio_ack_irq(unsigned int irq)
{
	int gpionr = irq - IRQ_PF0;
	int mask = (1L << gpionr);
	*pFIO_FLAG_C = mask;
	asm("ssync");
	*pFIO_MASKB_C = mask;
	asm("ssync");
	if (gpio_edge_triggered & mask) {
		/* ack */
	} else {
		/* ack and mask */
	}
	asm("ssync");
}

static void bf533_gpio_mask_irq(unsigned int irq)
{
	int gpionr = irq - IRQ_PF0;
	int mask = (1L << gpionr);
	*pFIO_FLAG_C = mask;
	asm("ssync");
	*pFIO_MASKB_C = mask;
	asm("ssync");
}

static void bf533_gpio_unmask_irq(unsigned int irq)
{
	int gpionr = irq - IRQ_PF0;
	int mask = (1L << gpionr);
	*pFIO_MASKB_S = mask;
}

static int bf533_gpio_irq_type(unsigned int irq, unsigned int type)
{
	int gpionr = irq - IRQ_PF0;
	int mask = (1L << gpionr);

	*pFIO_DIR &= ~mask; asm("ssync");
	*pFIO_INEN |= mask; asm("ssync");

	if (type == IRQT_PROBE) {
		/* only probe unenabled GPIO interrupt lines */
		if ( gpio_enabled & mask)
			return 0;
		type = __IRQT_RISEDGE | __IRQT_FALEDGE;
	}
	if (type & (__IRQT_RISEDGE|__IRQT_FALEDGE|__IRQT_HIGHLVL|__IRQT_LOWLVL))
		gpio_enabled |= mask;
	else
		gpio_enabled &= ~mask;

	if (type & (__IRQT_RISEDGE|__IRQT_FALEDGE)) {
		gpio_edge_triggered |= mask;
		*pFIO_EDGE |= mask;
	} else {
		*pFIO_EDGE &= ~mask;
		gpio_edge_triggered &= ~mask;
	}
	asm("ssync");

	if ((type & (__IRQT_RISEDGE|__IRQT_FALEDGE)) == (__IRQT_RISEDGE|__IRQT_FALEDGE))
		*pFIO_BOTH |= mask;
	else
		*pFIO_BOTH &= ~mask;
	asm("ssync");

	if ((type & (__IRQT_FALEDGE|__IRQT_LOWLVL)) && ((type & (__IRQT_RISEDGE|__IRQT_FALEDGE)) != (__IRQT_RISEDGE|__IRQT_FALEDGE)))
		*pFIO_POLAR |= mask;  /* low or falling edge denoted by one */
	else
		*pFIO_POLAR &= ~mask; /* high or rising edge denoted by zero */
	asm("ssync");

	if (type & (__IRQT_RISEDGE|__IRQT_FALEDGE))
		set_irq_handler(irq, do_edge_IRQ);
	else
		set_irq_handler(irq, do_level_IRQ);

	return 0;
}
static struct irqchip bf533_gpio_irqchip = {
	.ack		= bf533_gpio_ack_irq,
	.mask		= bf533_gpio_mask_irq,
	.unmask		= bf533_gpio_unmask_irq,
	.type           = bf533_gpio_irq_type
};

static void bf533_demux_gpio_irq(unsigned int intb_irq, struct irqdesc *intb_desc,
				 struct pt_regs *regs)
{
	int loop = 0;
	
	do {
		int irq = IRQ_PF0;
		int flag_d = *pFIO_FLAG_D;
		int mask = flag_d & gpio_enabled;
		loop = mask;
		do {
			if (mask & 1) {
				struct irqdesc *desc = irq_desc + irq;
				desc->handle(irq, desc, regs);
			}
			irq++;
			mask >>= 1;
		} while (mask);
	} while (loop);
}
#endif /* CONFIG_IRQCHIP_DEMUX_GPIO */

/*
 * This function should be called during kernel startup to initialize
 * the BFin IRQ handling routines.
 */

int __init  init_arch_irq(void)
{
	int irq;	
	unsigned long ilat = 0;
	/*  Disable all the peripheral intrs  - page 4-29 HW Ref manual */
	*pSIC_IMASK = SIC_UNMASK_ALL;
	asm("ssync;");	
   
	local_irq_disable();
	
#ifndef CONFIG_KGDB	
	*pEVT0 = evt_nmi;
	asm("csync;");	
#endif
	*pEVT2  = evt_evt2;
	asm("csync;");	
	*pEVT3	= trap;
	asm("csync;");	
	*pEVT5 	= evt_ivhw;
	asm("csync;");	
	*pEVT6 	= evt_timer;	 
	asm("csync;");	
	*pEVT7 	= evt_evt7;
	asm("csync;");	
	*pEVT8	= evt_evt8;	
	asm("csync;");	
	*pEVT9	= evt_evt9;	
	asm("csync;");	
	*pEVT10	= evt_evt10;	
	asm("csync;");	
	*pEVT11	= evt_evt11;	
	asm("csync;");	
	*pEVT12	= evt_evt12;	
	asm("csync;");	
	*pEVT13	= evt_evt13;	
	asm("csync;");	

	*pEVT14 = evt_system_call;	
	asm("csync;");	
	*pEVT15 = evt_soft_int1;	
	asm("csync;");	

  	for (irq = 0; irq < INTERNAL_IRQS; irq++) {
		if (irq <= IRQ_CORETMR)
			set_irq_chip(irq, &bf533_core_irqchip);
		else
			set_irq_chip(irq, &bf533_internal_irqchip);
#ifdef CONFIG_IRQCHIP_DEMUX_GPIO
		if (irq != IRQ_PROG_INTB) {
#endif
			set_irq_handler(irq, do_level_IRQ);
			set_irq_flags(irq, IRQF_VALID);
#ifdef CONFIG_IRQCHIP_DEMUX_GPIO
		} else {
			set_irq_chained_handler(irq, bf533_demux_gpio_irq);
		}
#endif
	}
#ifdef CONFIG_IRQCHIP_DEMUX_GPIO
  	for (irq = IRQ_PF0; irq <= IRQ_PF15; irq++) {
		set_irq_chip(irq, &bf533_gpio_irqchip);
		set_irq_handler(irq, do_level_IRQ); /* if configured as edge, then will be changed to do_edge_IRQ */
		set_irq_flags(irq, IRQF_VALID|IRQF_PROBE);
	}
#endif
   	*pIMASK = 0;
	asm("csync;");
	ilat  = *pILAT;
	asm("csync;");
	*pILAT = ilat;
	asm("csync;");

	printk(KERN_INFO "Configuring Blackfin Priority Driven Interrupts\n");
	program_IAR();   /* IMASK=xxx is equivalent to STI xx or irq_flags=xx, local_irq_enable() */
	search_IAR();    /* Therefore it's better to setup IARs before interrupts enabled */

   	/* Enable interrupts IVG7-15 */
	*pIMASK = irq_flags = irq_flags | IMASK_IVG15 | IMASK_IVG14 |IMASK_IVG13 |IMASK_IVG12 |IMASK_IVG11 |
		IMASK_IVG10 |IMASK_IVG9 |IMASK_IVG8 |IMASK_IVG7 |IMASK_IVGHW;	
	asm("csync;");

	local_irq_enable();
	return 0;
}

extern asmlinkage void asm_do_IRQ(unsigned int irq, struct pt_regs *regs);
void do_irq(int vec, struct pt_regs *fp)
{
   	if (vec > IRQ_CORETMR)
        {
          struct ivgx *ivg = ivg7_13[vec].ifirst;
          struct ivgx *ivg_stop = ivg7_13[vec].istop;
	  unsigned long sic_status;	

	  asm("csync;");	
 	  sic_status = *pSIC_IMASK & *pSIC_ISR;

	  for(;; ivg++) {
              if (ivg >= ivg_stop)  {
		num_spurious++;
                return;
              }
              else if ((sic_status & ivg->isrflag) != 0)
                break;
         }
	  vec = ivg->irqno;
        }
	asm_do_IRQ(vec, fp);
}
