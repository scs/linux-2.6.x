/*
 * File:         bf53x_sport.c 
 * Description:  low level driver for sportX/dmaY on blackfin 53x
 *               this should be moved to arch/bfinnommu/
 * Rev:          $Id$
 * Created:      Tue Sep 21 10:52:42 CEST 2004
 * Author:       Luuk van Dijk
 * mail:         blackfin@mdnmttr.nl
 * 
 * Copyright (C) 2004 Luuk van Dijk, Mind over Matter B.V.
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
#ifndef BF53X_SPORT_H
#define BF53X_SPORT_H

#include <linux/types.h>
#include <asm/simple_bf533_dma.h>


struct bf53x_sport;

struct bf53x_sport* bf53x_sport_init(int sport_chan,  
                int dma_rx, dma_interrupt_t rx_handler,
                int dma_tx, dma_interrupt_t tx_handler);
void bf53x_sport_done(struct bf53x_sport* sport);

/* first use these ...*/

/* note: multichannel is in units of 8 channels, tdm_count is # channels NOT / 8 ! */
/* all channels are enabled by default */
int bf53x_sport_set_multichannel( struct bf53x_sport* sport, int tdm_count, int packed);

int bf53x_sport_config_rx( struct bf53x_sport* sport, 
			   unsigned int rcr1, unsigned int rcr2, 
			   unsigned int clkdiv, unsigned int fsdiv );

int bf53x_sport_config_tx( struct bf53x_sport* sport, 
			   unsigned int tcr1, unsigned int tcr2, 
			   unsigned int clkdiv, unsigned int fsdiv );

/* ... then these: */

/* buffer size (in bytes) == fragcount * fragsize_bytes */
/* if tdm_mask != 0 the m[rt]cs0 register is updated with it */

/* this is not a very general api, it sets the dma to 2d autobuffer mode */

int bf53x_sport_config_rx_dma( struct bf53x_sport* sport, void* buf, 
			       int fragcount, size_t fragsize_bytes, 
			       unsigned int tdm_mask );

int bf53x_sport_config_tx_dma( struct bf53x_sport* sport, void* buf, 
			       int fragcount, size_t fragsize_bytes, 
			       unsigned int tdm_mask );


/* rx and tx can only run simultanously, use a dummy buffer to have one
   of them disabled, and disable their irq's with the following */

int sport_disable_dma_rx(struct bf53x_sport* sport);
int sport_disable_dma_tx(struct bf53x_sport* sport);

int bf53x_sport_start(struct bf53x_sport* sport);
int bf53x_sport_stop(struct bf53x_sport* sport); /* idempotent */


/* for use in interrupt handler */
void* bf53x_sport_curr_addr_rx( struct bf53x_sport* sport );
void* bf53x_sport_curr_addr_tx( struct bf53x_sport* sport );

int bf53x_sport_curr_frag_rx( struct bf53x_sport* sport );
int bf53x_sport_curr_frag_tx( struct bf53x_sport* sport );


/* check and clear sport and dma irq status, call from irq handler */
/* when [TR][OU]VF are set, they will be cleared, and [TR]SPEN will be zeroed */
int bf53x_sport_check_status(struct bf53x_sport* sport, unsigned int* sport_stat, 
			     unsigned int* rx_stat, unsigned int* tx_stat);


/* for use in diagnostics */
int  bf53x_sport_dump_stat(struct bf53x_sport* sport, char* buf, size_t len);


#endif /* BF53X_SPORT_H */
