/*
 * File:         arch/blackfin/mach-bf537/ints-priority.c
 * Based on:     arch/blackfin/mach-bf533/ints-priority.c
 * Author:       Michael Hennerich
 *               COPYRIGHT 2005 Analog Devices
 *
 * Created:
 * Description:
 *
 * Rev:         $Id$
 *
 * Modified:
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
#include <asm/blackfin.h>
#include <asm/irq.h>

void program_IAR(void);
/*Program the IAR registers*/

void program_IAR()
{
	/* Program the IAR0 Register with the configured priority */
	*pSIC_IAR0 = ((CONFIG_IRQ_PLL_WAKEUP - 7) << IRQ_PLL_WAKEUP_POS) |
	    ((CONFIG_IRQ_DMA_ERROR - 7) << IRQ_DMA_ERROR_POS) |
	    ((CONFIG_IRQ_ERROR - 7) << IRQ_ERROR_POS) |
	    ((CONFIG_IRQ_RTC - 7) << IRQ_RTC_POS) |
	    ((CONFIG_IRQ_PPI - 7) << IRQ_PPI_POS) |
	    ((CONFIG_IRQ_SPORT0_RX - 7) << IRQ_SPORT0_RX_POS) |
	    ((CONFIG_IRQ_SPORT0_TX - 7) << IRQ_SPORT0_TX_POS) |
	    ((CONFIG_IRQ_SPORT1_RX - 7) << IRQ_SPORT1_RX_POS);

	*pSIC_IAR1 = ((CONFIG_IRQ_SPORT1_TX - 7) << IRQ_SPORT1_TX_POS) |
	    ((CONFIG_IRQ_TWI - 7) << IRQ_TWI_POS) |
	    ((CONFIG_IRQ_SPI - 7) << IRQ_SPI_POS) |
	    ((CONFIG_IRQ_UART0_RX - 7) << IRQ_UART0_RX_POS) |
	    ((CONFIG_IRQ_UART0_TX - 7) << IRQ_UART0_TX_POS) |
	    ((CONFIG_IRQ_UART1_RX - 7) << IRQ_UART1_RX_POS) |
	    ((CONFIG_IRQ_UART1_TX - 7) << IRQ_UART1_TX_POS) |
	    ((CONFIG_IRQ_CAN_RX - 7) << IRQ_CAN_RX_POS);

	*pSIC_IAR2 = ((CONFIG_IRQ_CAN_TX - 7) << IRQ_CAN_TX_POS) |
	    ((CONFIG_IRQ_MAC_RX - 7) << IRQ_MAC_RX_POS) |
	    ((CONFIG_IRQ_MAC_TX - 7) << IRQ_MAC_TX_POS) |
	    ((CONFIG_IRQ_TMR0 - 7) << IRQ_TMR0_POS) |
	    ((CONFIG_IRQ_TMR1 - 7) << IRQ_TMR1_POS) |
	    ((CONFIG_IRQ_TMR2 - 7) << IRQ_TMR2_POS) |
	    ((CONFIG_IRQ_TMR3 - 7) << IRQ_TMR3_POS) |
	    ((CONFIG_IRQ_TMR4 - 7) << IRQ_TMR4_POS);

	*pSIC_IAR3 = ((CONFIG_IRQ_TMR5 - 7) << IRQ_TMR5_POS) |
	    ((CONFIG_IRQ_TMR6 - 7) << IRQ_TMR6_POS) |
	    ((CONFIG_IRQ_TMR7 - 7) << IRQ_TMR7_POS) |
	    ((CONFIG_IRQ_PROG_INTA - 7) << IRQ_PROG_INTA_POS) |
	    ((CONFIG_IRQ_PORTG_INTB - 7) << IRQ_PORTG_INTB_POS) |
	    ((CONFIG_IRQ_MEM_DMA0 - 7) << IRQ_MEM_DMA0_POS) |
	    ((CONFIG_IRQ_MEM_DMA1 - 7) << IRQ_MEM_DMA1_POS) |
	    ((CONFIG_IRQ_WATCH - 7) << IRQ_WATCH_POS);
	__builtin_bfin_ssync();

}				/*End of program_IAR */
