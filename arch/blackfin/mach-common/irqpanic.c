 /*
  * File:        arch/blackfin/mach-common/irqpanic.c
  * Based on:
  * Author:      unknown
  *              COPYRIGHT 2005 Analog Devices
  * Created:     ?
  * Description: panic kernel with dump information
  *
  * Rev:          $Id$
  *
  * Modified:     rgetz - added cache checking code 14Feb06
  *
  *
  * Bugs:         Enter bugs at http://blackfin.uclinux.org/
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2, or (at your option)
  * any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program; see the file COPYING.
  * If not, write to the Free Software Foundation,
  * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
  */

#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <asm/traps.h>
#include <asm/blackfin.h>

#include "../oprofile/op_blackfin.h"

/*********
 * irq_panic - calls panic with string setup
 *********/
#ifdef CONFIG_DEBUG_ICACHE_CHECK
#define L1_ICACHE_START 0xffa10000
#define L1_ICACHE_END   0xffa13fff
void irq_panic(int reason, struct pt_regs *regs) __attribute__ ((section (".text.l1")));
#endif

asmlinkage void irq_panic(int reason, struct pt_regs *regs)
{
	int sig = 0;
	siginfo_t info;

#ifdef CONFIG_DEBUG_ICACHE_CHECK
        unsigned int cmd, tag, ca, cache_hi, cache_lo, *pa;
        unsigned short i, j, die;
        unsigned int bad[10][6];

	/* check entire cache for coherency                */
        /* Since printk is in cacheable memory,            */
        /* don't call it until you have checked everything */

	die = 0;
	i = 0;

	/* check icache */

	for (ca=L1_ICACHE_START; ca <=L1_ICACHE_END && i < 10; ca+= 32) {

		/* Grab various address bits for the itest_cmd fields                      */
		cmd = ( (( ca & 0x3000) <<  4 ) |   /* ca[13:12] for SBNK[1:0]             */
			(( ca & 0x0c00) << 16 ) |   /* ca[11:10] for WAYSEL[1:0]           */
			(( ca & 0x3f8)       ) |    /* ca[09:03] for SET[4:0] and DW[1:0]  */
			0 );                        /* Access Tag, Read access             */

		__builtin_bfin_ssync();
		*pITEST_COMMAND=cmd;
		__builtin_bfin_ssync();
		tag = *pITEST_DATA0;
		__builtin_bfin_ssync();

		/* if tag is marked as valid, check it */
		if ( tag & 1 ) {
			/* The icache is arranged in 4 groups of 64-bits */
			for (j = 0 ; j < 32 ; j+=8 ) {
				cmd = ( (( (ca+j) & 0x3000) <<  4 ) |   /* ca[13:12] for SBNK[1:0]             */
					(( (ca+j) & 0x0c00) << 16 ) |   /* ca[11:10] for WAYSEL[1:0]           */
					(( (ca+j) & 0x3f8)       ) |    /* ca[09:03] for SET[4:0] and DW[1:0]  */
					4 );                            /* Access Data, Read access             */

                                __builtin_bfin_ssync();
                                 *pITEST_COMMAND=cmd;
                                __builtin_bfin_ssync();

				cache_hi = *pITEST_DATA1;
				cache_lo = *pITEST_DATA0;

				pa = ((unsigned int *)((tag & 0xffffcc00)  | ((ca+j) & ~(0xffffcc00))));

				/*
				 * Debugging this, enable
				 *
				 * printk("addr: %08x %08x%08x | %08x%08x\n",
				 *  ((unsigned int *)((tag & 0xffffcc00)  | ((ca+j) & ~(0xffffcc00)))),
				 *   cache_hi, cache_lo, *(pa+1), *pa);
				 */

				if ( cache_hi != *(pa+1) || cache_lo != *pa ) {
					/* Since icache is not working, stay out of it, by not printing */
					die = 1;
					bad [i][0] = (ca+j);
					bad [i][1] = cache_hi;
					bad [i][2] = cache_lo;
					bad [i][3] = ((tag & 0xffffcc00)  | ((ca+j) & ~(0xffffcc00)));
					bad [i][4] = *(pa+1);
					bad [i][5] = *(pa);
					i++;
				}
			}
		}
	}
	if (die) {
		printk ("icache coherency error\n");
		for (j=0 ; j <= i; j++) {
			printk("cache address   : %08x  cache value : %08x%08x\n",  bad [j][0],  bad [j][1],  bad [j][2]);
			printk("physical address: %08x  SDRAM value : %08x%08x\n",  bad [j][3],  bad [j][4],  bad [j][5]);
		}
		panic("icache coherency error");
	} else {
		printk("\n\nicache checked, and OK\n");
	}
#endif

	printk("\n\nException: IRQ 0x%x entered\n", reason);
	printk(" code=[0x%08x],  ", (unsigned int)regs->seqstat);
	printk(" stack frame=0x%04x,  ", (unsigned int)(unsigned long)regs);
	printk(" bad PC=0x%04x\n", (unsigned int)regs->pc);
	if (reason == 0x5) {

		printk("\n----------- HARDWARE ERROR -----------\n\n");

		/* There is only need to check for Hardware Errors, since other
		 * EXCEPTIONS are handled in TRAPS.c (MH)
		 */
		switch (((unsigned int)regs->seqstat) >> 14) {
		case (0x2):	/* System MMR Error */
			info.si_code = BUS_ADRALN;
			sig = SIGBUS;
			printk(HWC_x2);
			break;
		case (0x3):	/* External Memory Addressing Error */
			info.si_code = BUS_ADRERR;
			sig = SIGBUS;
			printk(HWC_x3);
			break;
		case (0x12):	/* Performance Monitor Overflow */
			printk(HWC_x12);
			break;
		case (0x18):	/* RAISE 5 instruction */
			printk(HWC_x18);
			break;
		default:	/* Reserved */
			printk(HWC_default);
			break;
		}
	}

	regs->ipend = *pIPEND;
	dump(regs, regs->pc);
	if (0 == (info.si_signo = sig) || 0 == user_mode(regs))	/* in kernelspace */
		panic("Unhandled IRQ or exceptions!\n");
	else {			/* in userspace */
		info.si_errno = 0;
		info.si_addr = (void *)regs->pc;
		force_sig_info(sig, &info, current);
	}
}

#ifdef CONFIG_HARDWARE_PM
/****
 *
 *   call the handler of Performance overflow
 ****/
asmlinkage void pm_overflow(int irq, struct pt_regs *regs){
	pm_overflow_handler(irq , regs);
}
#endif
