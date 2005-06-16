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
 * Copyright 2005 Analog Devices Inc. 
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

//ToDo: BF537 uses a multiplexed ERROR interrupt for CAN MAC SPORT0 SPORT1 SPI UART0 UART1 - currently only one device can request this single interrupt ... 


#define INTERNAL_IRQS		NR_IRQS
         
volatile unsigned long irq_flags = 0;

/* The number of spurious interrupts */
volatile unsigned int num_spurious;

struct ivgx	{
	int irqno;	/*irq number for request_irq, available in mach-bf537/irq.h*/
	int isrflag;	/*corresponding bit in the SIC_ISR register*/
}ivg_table[NR_PERI_INTS];

struct ivg_slice {
	struct ivgx *ifirst; /* position of first irq in ivg_table for given ivg */
	struct ivgx *istop;  
} ivg7_13[IVG13-IVG7+1];


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

/*Program the IAR registers*/
static void __init program_IAR()
{
		/* Program the IAR0 Register with the configured priority */
	      *pSIC_IAR0 =  ((CONFIG_IRQ_PLL_WAKEUP	 	-7) <<		IRQ_PLL_WAKEUP_POS) |
						((CONFIG_IRQ_DMA_ERROR      -7) <<      IRQ_DMA_ERROR_POS ) |
						((CONFIG_IRQ_ERROR          -7) <<      IRQ_ERROR_POS     ) |
						((CONFIG_IRQ_RTC            -7) <<      IRQ_RTC_POS       ) |
						((CONFIG_IRQ_PPI            -7) <<      IRQ_PPI_POS       ) |
						((CONFIG_IRQ_SPORT0_RX      -7) <<      IRQ_SPORT0_RX_POS ) |
						((CONFIG_IRQ_SPORT0_TX      -7) <<      IRQ_SPORT0_TX_POS ) |
						((CONFIG_IRQ_SPORT1_RX      -7) <<      IRQ_SPORT1_RX_POS );


		*pSIC_IAR1 =	((CONFIG_IRQ_SPORT1_TX      -7) <<      IRQ_SPORT1_TX_POS ) |	
						((CONFIG_IRQ_TWI            -7) <<      IRQ_TWI_POS       ) |  
						((CONFIG_IRQ_SPI            -7) <<      IRQ_SPI_POS       ) |  
						((CONFIG_IRQ_UART0_RX       -7) <<      IRQ_UART0_RX_POS  ) |  
						((CONFIG_IRQ_UART0_TX       -7) <<      IRQ_UART0_TX_POS  ) |  
						((CONFIG_IRQ_UART1_RX       -7) <<      IRQ_UART1_RX_POS  ) |  
						((CONFIG_IRQ_UART1_TX       -7) <<      IRQ_UART1_TX_POS  ) |  
						((CONFIG_IRQ_CAN_RX         -7) <<      IRQ_CAN_RX_POS    ); 
 
		*pSIC_IAR2 =	((CONFIG_IRQ_CAN_TX         -7) <<      IRQ_CAN_TX_POS    ) |		
						((CONFIG_IRQ_MAC_RX         -7) <<      IRQ_MAC_RX_POS    ) |  
						((CONFIG_IRQ_MAC_TX         -7) <<      IRQ_MAC_TX_POS    ) |  
						((CONFIG_IRQ_TMR0           -7) <<      IRQ_TMR0_POS      ) |  
						((CONFIG_IRQ_TMR1           -7) <<      IRQ_TMR1_POS      ) |  
						((CONFIG_IRQ_TMR2           -7) <<      IRQ_TMR2_POS      ) |  
						((CONFIG_IRQ_TMR3           -7) <<      IRQ_TMR3_POS      ) |  
						((CONFIG_IRQ_TMR4           -7) <<      IRQ_TMR4_POS      ); 

		*pSIC_IAR3 =	((CONFIG_IRQ_TMR5           -7) <<      IRQ_TMR5_POS	 ) |		
						((CONFIG_IRQ_TMR6           -7) <<      IRQ_TMR6_POS     ) |   
						((CONFIG_IRQ_TMR7           -7) <<      IRQ_TMR7_POS     ) |   
						((CONFIG_IRQ_PROG_INTA      -7) <<      IRQ_PROG_INTA_POS) |   
						((CONFIG_IRQ_PROG_INTB      -7) <<      IRQ_PROG_INTB_POS) |   
						((CONFIG_IRQ_MEM_DMA0       -7) <<      IRQ_MEM_DMA0_POS ) |   
						((CONFIG_IRQ_MEM_DMA1       -7) <<      IRQ_MEM_DMA1_POS ) |   
						((CONFIG_IRQ_WATCH          -7) <<      IRQ_WATCH_POS    );
	asm("ssync;");	

}	/*End of program_IAR*/

/* Search SIC_IAR and fill tables with the irqvalues 
and their positions in the SIC_ISR register */

static void __init search_IAR(void)	
{
    unsigned ivg, irq_pos = 0;
    for (ivg = 0; ivg <= IVG13-IVG7; ivg++)
    {
        int irqn;

        ivg7_13[ivg].istop = 
        ivg7_13[ivg].ifirst = &ivg_table[irq_pos];        
          
        for(irqn = 0; irqn < NR_PERI_INTS; irqn++)
          if (ivg == (0x0f & pSIC_IAR0[irqn >> 3] >> (irqn & 7) * 4))
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

static void bf537_core_mask_irq(unsigned int irq)
{
	local_irq_disable();
	irq_flags &= ~(1<<irq);
	local_irq_enable();
}

static void bf537_core_unmask_irq(unsigned int irq)
{
	/* enable the interrupt */
	local_irq_disable();
	irq_flags |= 1<<irq;
	local_irq_enable();
	return;
}

static void bf537_internal_mask_irq(unsigned int irq)
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

static void bf537_internal_unmask_irq(unsigned int irq)
{
	unsigned long irq_mask;
	local_irq_disable();
	irq_mask = (1<<(irq - (IRQ_CORETMR+1)));
   	*pSIC_IMASK |= irq_mask;
	asm("ssync;");
	local_irq_enable();
}

static struct irqchip bf537_core_irqchip = {
	.ack		= bf537_core_mask_irq,
	.mask		= bf537_core_mask_irq,
	.unmask		= bf537_core_unmask_irq,
};

static struct irqchip bf537_internal_irqchip = {
	.ack		= bf537_internal_mask_irq,
	.mask		= bf537_internal_mask_irq,
	.unmask		= bf537_internal_unmask_irq,
};

#ifdef CONFIG_IRQCHIP_DEMUX_GPIO
static int gpio_enabled;
static int gpio_edge_triggered;

static void bf537_gpio_ack_irq(unsigned int irq)
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

static void bf537_gpio_mask_irq(unsigned int irq)
{
	int gpionr = irq - IRQ_PF0;
	int mask = (1L << gpionr);
	*pFIO_FLAG_C = mask;
	asm("ssync");
	*pFIO_MASKB_C = mask;
	asm("ssync");
}

static void bf537_gpio_unmask_irq(unsigned int irq)
{
	int gpionr = irq - IRQ_PF0;
	int mask = (1L << gpionr);
	*pFIO_MASKB_S = mask;
}

static int bf537_gpio_irq_type(unsigned int irq, unsigned int type)
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
static struct irqchip bf537_gpio_irqchip = {
	.ack		= bf537_gpio_ack_irq,
	.mask		= bf537_gpio_mask_irq,
	.unmask		= bf537_gpio_unmask_irq,
	.type           = bf537_gpio_irq_type
};

static void bf537_demux_gpio_irq(unsigned int intb_irq, struct irqdesc *intb_desc,
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
#endif
	*pEVT2  = evt_evt2;
	*pEVT3	= trap;	
	*pEVT5 	= evt_ivhw;
	*pEVT6 	= evt_timer;	 	
	*pEVT7 	= evt_evt7;
	*pEVT8	= evt_evt8;	
	*pEVT9	= evt_evt9;		
	*pEVT10	= evt_evt10;		
	*pEVT11	= evt_evt11;		
	*pEVT12	= evt_evt12;	
	*pEVT13	= evt_evt13;	
	*pEVT14 = evt_system_call;		
	*pEVT15 = evt_soft_int1;	
	asm("csync;");	

  	for (irq = 0; irq < SYS_IRQS; irq++) {
		if (irq <= IRQ_CORETMR)
			set_irq_chip(irq, &bf537_core_irqchip);
		else
			set_irq_chip(irq, &bf537_internal_irqchip);
#ifdef CONFIG_IRQCHIP_DEMUX_GPIO
		if (irq != IRQ_PROG_INTB) {
#endif
			set_irq_handler(irq, do_level_IRQ);
			set_irq_flags(irq, IRQF_VALID);
#ifdef CONFIG_IRQCHIP_DEMUX_GPIO
		} else {
			set_irq_chained_handler(irq, bf537_demux_gpio_irq);
		}
#endif
	}
#ifdef CONFIG_IRQCHIP_DEMUX_GPIO
  	for (irq = IRQ_PF0; irq <= IRQ_PF15; irq++) {
		set_irq_chip(irq, &bf537_gpio_irqchip);
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
          struct ivgx *ivg = ivg7_13[vec-IVG7].ifirst;
          struct ivgx *ivg_stop = ivg7_13[vec-IVG7].istop;
	  unsigned long sic_status;	

	  asm("csync;");	
 	  sic_status = *pSIC_IMASK & *pSIC_ISR;

	  for(;; ivg++) {
              if (ivg >= ivg_stop)  {
		num_spurious++;
                return;
              }
              else if (sic_status & ivg->isrflag)
                break;
         }
	  vec = ivg->irqno;
        }
	asm_do_IRQ(vec, fp);
}
