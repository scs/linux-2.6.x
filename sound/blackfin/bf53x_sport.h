/*
 * File:         bf53x_sport.c 
 * Description:  low level driver for sportX/dmaY on blackfin 53x
 *               this should be moved to arch/blackfin/
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
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <asm/simple_bf533_dma.h>

/* keep copies of the dma registers, updated by us in an irq callback 
 * to avoid queries back to this driver having to read them 
 * this has to be in the header file, because ad1836.c needs this to decide
 * wether to call bf53x_sport_shadow_update_XX()
 */

/* #define BF53X_SHADOW_REGISTERS */

#define BF53X_ANOMALY_29  /* don't use the DMA_RUN bit, keep track of running status ourselves */


/*
 * source: ADSP-BF533 Blackfin Processor Hardware Reference, 
 * chapter 12, and appendix B-12 table  B10 
 */

struct sport_register {
  unsigned short tcr1;    unsigned short reserved0;
  unsigned short tcr2;    unsigned short reserved1;
  unsigned short tclkdiv; unsigned short reserved2;
  unsigned short tfsdiv;  unsigned short reserved3;
  unsigned long tx;
  unsigned long reserved_l0;
  unsigned long rx;
  unsigned long reserved_l1;
  unsigned short rcr1;    unsigned short reserved4;
  unsigned short rcr2;    unsigned short reserved5;
  unsigned short rclkdiv; unsigned short reserved6;
  unsigned short rfsdiv;  unsigned short reserved7;
  unsigned short stat;    unsigned short reserved8;
  unsigned short chnl;    unsigned short reserved9;
  unsigned short mcmc1;   unsigned short reserved10;
  unsigned short mcmc2;   unsigned short reserved11;
  unsigned long mtcs0;
  unsigned long mtcs1;
  unsigned long mtcs2;
  unsigned long mtcs3;
  unsigned long mrcs0;
  unsigned long mrcs1;
  unsigned long mrcs2;
  unsigned long mrcs3;
};

#define DESC_ELEMENT_COUNT 9

/* (large mode) descriptor arrays */
struct bf53x_dma_desc {
  unsigned long          next_desc;
  unsigned long          start_addr;
  unsigned short         cfg;
  unsigned short         xcount;
  unsigned short         xmodify;
  unsigned short         ycount;
  unsigned short         ymodify;
} __attribute__((packed));


struct bf53x_sport {
  int sport_chan;
  int dma_rx_chan;
  int dma_tx_chan;
  struct sport_register* regs;

  dma_register_t* dma_rx;   /* a struct gratefully borrowed from asm/simple_bf533_dma.h */
  dma_register_t* dma_tx;

#define DUMMY_BUF_LEN 4
  /* for dummy dma transfer */
  unsigned long	dummy_buf_rx;
  unsigned long	dummy_buf_tx;

#ifdef BF53X_SHADOW_REGISTERS
  dma_register_t* dma_shadow_rx;   /* a struct gratefully borrowed from asm/simple_bf533_dma.h */
  dma_register_t* dma_shadow_tx;
#endif

  struct bf53x_dma_desc* dma_rx_desc;	/* DMA descriptor ring head of current audio stream*/
  struct bf53x_dma_desc* dma_tx_desc;
  struct bf53x_dma_desc* dma_rx_expired_desc;  /* DMA descriptor ring head of last audio stream*/
  struct bf53x_dma_desc* dma_tx_expired_desc;
  struct bf53x_dma_desc* dma_rx_expired2_desc; /* DMA descriptor ring head of the one before last audio stream*/
  struct bf53x_dma_desc* dma_tx_expired2_desc;
  /* DMA descriptor state in change procedure.
   * 0: DMA is walking through current DMA descriptor ring.
   * 1: New DMA descritor ring is setup, but not hook into current DMA descriptor ring. 
   * 2: The new DMA descriptor ring is hooked into current DMA descriptor ring, but it hasn't been loaded into DMA.
   */
  int dma_rx_desc_changed;
  int dma_tx_desc_changed;

  unsigned int rcr1;
  unsigned int rcr2;
  int rx_tdm_count;

  unsigned int tcr1;
  unsigned int tcr2;
  int tx_tdm_count;

#ifdef BF53X_ANOMALY_29
  int is_running;   /* little kludge to work around anomaly 29: DMA_RUN bit unreliable */
#endif

};

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

/* this is not a very general api, it sets the dma to 2d autobuffer mode */

int bf53x_sport_config_rx_dma( struct bf53x_sport* sport, void* buf, 
			       int fragcount, size_t fragsize_bytes);

int bf53x_sport_config_tx_dma( struct bf53x_sport* sport, void* buf, 
			       int fragcount, size_t fragsize_bytes);
			      
int sport_config_rx_dummy(struct bf53x_sport* sport);
int sport_config_tx_dummy(struct bf53x_sport* sport);

void bf53x_sport_hook_tx_desc( struct bf53x_sport* sport);
void bf53x_sport_hook_rx_desc( struct bf53x_sport* sport);
int bf53x_sport_is_tx_desc_changed(struct bf53x_sport* sport);
int bf53x_sport_is_rx_desc_changed(struct bf53x_sport* sport);

/* rx and tx can only run simultanously, use a dummy buffer to have one
   of them disabled, and disable their irq's with the following */

void sport_disable_dma_rx(struct bf53x_sport* sport);
void sport_disable_dma_tx(struct bf53x_sport* sport);

int bf53x_sport_start(struct bf53x_sport* sport);
int bf53x_sport_stop(struct bf53x_sport* sport); /* idempotent */

int bf53x_sport_is_running(struct bf53x_sport* sport);


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


#ifdef BF53X_SHADOW_REGISTERS 
void bf53x_sport_shadow_update_rx(struct bf53x_sport* sport);
void bf53x_sport_shadow_update_tx(struct bf53x_sport* sport);
#endif


#endif /* BF53X_SPORT_H */
