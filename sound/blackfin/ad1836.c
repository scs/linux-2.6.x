/*
 * File:         adi1836.c 
 * Description:  driver for ADI 1836 sound chip connected to bf53x sport/spi
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

/* notes: 
 * - once this code stabilizes, move the irq and dma stuff 
 *   into bf53x_spi.c and bf53x_sport.c
 * - the organisation of this file is as follows, in REVERSE order:
 *     * at the top level, the end of the file is the /module/
 *       it allocates the spi and sport, and probes for the /card/
 *     * the card allocates the /low-level device/, the /proc/, the /snd/ and /mixer/ stuff
 *     * the /snd/ and /mixer/ stuff use the methods of the low level device
 *       to control the registers over the spi, and the methods of the sport
 * - there are useful proc entries for spi, sport and ad1836 register and irq status
 *       also, you could do echo 2 > talktrough and echo 0x1234 > registers
 *       to do what you'd expect,  but we don't have redirection in sash, 
 *       so the write interface to these  isn't very useful yet (and not tested).  
 *  - the chan_mask facility may easily be split into separate masks for rx and tx
 *       by duplicating it, and using the proper one in the hw_params callback
 *  - define/undef NOCONTROLS below to omit/include all the ALSA controls
 *    note that I have no idea if I chose the control names properly.
 *
 */

/* theory of operation:
 *
 *  the ad1836 is connected to the SPI and one of the SPORT ports.
 *  Over the spi, we send 16-bit commands that set the control
 *  registers.  since that is only 1 word at a time, and very rare, we
 *  do that non-dma, and we sleep until the spi irq handler wakes us
 *  up.
 *
 *  Over the sport we have 8 channel tdm pcm data, read/written by
 *  DMA.  the 8 channels correspond to pcm (dac0,1,2 and spdif) x
 *  (L,R) for output and pcm (adc0,1,spdif,unused) x (L,R) for input.
 *  The DMA operates in 2d autobuffer mode, with the outer loop
 *  counting the 'periods' (=ALSA term for what oss calls 'fragment')
 *  an irq is generated only once per period, (not once per frame like
 *  in the VSDP example)
 * 
 *  for 48khz and a relatively small fragment size of 16kb 
 *      = 512 samples/frame * 8 channels/sample * 4 bytes / channel
 *  that's an irq rate of 93hz, which is already quite affordable.
 * 
 *  the alsa device has 1 pcm that may be opened in 2,4,6 or 8 channel
 *  mode.  The DMA operates in 'packed' mode, which means that only
 *  enabled TDM channels are supposed to occur in the dma buffer.  to
 *  select which channels are enabled we use a configurable 'channel
 *  mask', that can be set through the /proc interface
 * 
 *  all knowledge from the bfin hwref guide has been encapsulated in
 *  separate files bf53x_sp{ort,i}.[hc]
 *
 *  if no pcm streams are open, the driver can be put in stupid or
 *  smart talktrough mode. stupid sets the rx and tx dmabuffer to the
 *  same address, smart copies fragments between the rx and tx buffer
 *  in the interrupt handler.
 * 
 * TODO: rething _prepare() and _trigger() to keep rx and tx out of eachothers way
 */


#include <sound/driver.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <asm/irq.h>

#include <sound/core.h>
#include <sound/info.h>
#include <sound/control.h>
#include <sound/pcm.h>
#define SNDRV_GET_ID
#include <sound/initval.h>

#include <asm/blackfin.h>

#include "bf53x_spi.h"
#include "bf53x_sport.h"

#include "adi1836.h"
#include "adi1836_config.h"

#ifdef CONFIG_SND_DEBUG
#define snd_printk_marker() snd_printk( KERN_INFO "%s\n", __FUNCTION__ )
#else
#define snd_printk_marker() 
#endif

#undef CONFIG_SND_DEBUG_CURRPTR  /* causes output every frame! */

/* assembly helpers */
extern void b4copy(unsigned int* src, unsigned int* dst, unsigned int count_bytes); /* in b4copy.S */
extern void bf53x_cache_flush(void* start, unsigned int size_bytes);
extern void bf53x_cache_flushinv(void* start, unsigned int size_bytes);


#undef NOCONTROLS  /* define this to omit all the ALSA controls */


#define CHIP_NAME "Analog Devices AD1836A"

/* ALSA boilerplate */

static int   index[SNDRV_CARDS]  = SNDRV_DEFAULT_IDX;
static char* id[SNDRV_CARDS]     = SNDRV_DEFAULT_STR;
static int   enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;


/* Chip level */

#define AD1836_BUFFER_SIZE 0x40000 /* 256kb */
#define TALKTROUGH_FRAGMENTS 4
#define TALKTROUGH_CHANMASK 0xff

#undef INIT_TALKTROUGH

#define ad1836_t_magic  0xa15a4501

typedef struct snd_ad1836 ad1836_t;
struct snd_ad1836 {

  snd_card_t*         card;
  struct bf53x_spi*   spi;
  struct bf53x_spi_channel* spi_chan;
  struct bf53x_sport* sport;

  snd_pcm_t* pcm;

  /* define correspondence of alsa channels to ad1836 channels */
  unsigned int chan_mask[4];
  char     chan_mask_str[5]; /* last one is \0 */

  wait_queue_head_t   spi_waitq;
  uint16_t chip_registers[16];
  int      poll_reg;  /* index of the ad1836 register last queried */

  snd_pcm_substream_t* rx_substream;  /* if non-null, current subtream running */
  snd_pcm_substream_t* tx_substream;  /* if non-null, current subtream running */

  /* stats for /proc/../sport */
  long sport_irq_timestamp;
  long sport_irq_count;
  long sport_irq_count_rx;
  long sport_irq_count_tx;

  long spi_irq_timestamp;
  long spi_irq_count;

  int runmode;
#define RUN_RX 0x1
#define RUN_TX 0x2

  int talktrough_mode;
#define TALKTROUGH_OFF 0
#define TALKTROUGH_STUPID 1
#define TALKTROUGH_SMART 2

  void* rx_buf; /* in talktrough mode, these are owned */
  void* tx_buf; 

};


#ifndef NOCONTROLS
#define chip_t_magic ad1836_t_magic  /* move to include/sound/sndmagic.h in due time */
typedef ad1836_t chip_t; /* used in alsa macro's */
#endif



unsigned int dummy_buf_rx[16]; /* used for idle rx/tx channel */
unsigned int dummy_buf_tx[16]; /* used for idle rx/tx channel */

static int ad1836_spi_handler(struct bf53x_spi_channel* chan, void* buf, size_t len, void* private){
  ad1836_t *chip = (ad1836_t*) private;
  unsigned int data = *(unsigned int*) buf;
  snd_assert( chip->spi_chan == chan, return -EINVAL);
  // snd_printk( KERN_INFO "in ad1836 spi handler. polled: %x data = 0x%04x\n", chip->poll_reg, data );  
  ++(chip->spi_irq_count);

  if( (data & 0xf00f) == 0xf000 ) /* its an answer to a VU query */
    chip->chip_registers[chip->poll_reg] = 
      (chip->poll_reg << 12) | ADC_READ | (data & ADC_PEAK_MASK);

  wake_up(&chip->spi_waitq);

  // TODO: move received data to the register cache, make separate handler for set_register
  // in the sport irq: send a register read cmd for the vu meters...

  return 0;
}


static int snd_ad1836_set_register(ad1836_t *chip, unsigned int reg, unsigned int mask, unsigned int value){

  int stat;
  unsigned int data = (chip->chip_registers[reg] & ~mask) | (value & mask);

  // snd_printk( KERN_INFO "spi set reg %d = 0x%04x\n", reg, data);  

  /* the following will be much nicer if the wait stuff is moved into bf53x_spi.c */
  do {
    stat = bf53x_spi_transceive(chip->spi_chan, data, NULL, NULL);
  } while(stat != 0);

  // snd_printk( KERN_INFO "waiting for spi set reg %d\n", reg);  

  if( bf53x_spi_busy(chip->spi) )
    if( !sleep_on_timeout(&chip->spi_waitq, HZ) ){
      snd_printk( KERN_INFO "timeout setting register %d\n", reg);  
      bf53x_spi_clear_channel(chip->spi);
      return -ENODEV;
    }

  chip->chip_registers[reg] = data;
  snd_printk( KERN_INFO "set register %d to 0x%04x\n", reg, data);  

  return 0;

}



static void snd_ad1836_reset_sport_stats(ad1836_t *chip ){
  chip->sport_irq_timestamp = jiffies;
  chip->sport_irq_count = 0;
  chip->sport_irq_count_rx = 0;
  chip->sport_irq_count_tx = 0;
}

static void snd_ad1836_reset_spi_stats(ad1836_t *chip ){
  chip->spi_irq_timestamp = jiffies;
  chip->spi_irq_count = 0;
}


/* define correspondence between ALSA and ad1836 channels, default '0123' */
static int ad1836_set_chan_masks(ad1836_t* chip, char* permutation){

  int i,j,m;
  
  /* 4 characters */
  if( strlen(permutation) != 4 ) 
    return -EINVAL;

  /* between '0' and '3' inclusive */
  for( i=0;i<4; ++i )
    if( (permutation[i] < '0') || permutation[i] > '3' ) 
      return -EINVAL;

  /* a permutation, i.e. no duplicates */
  for(i=0;i<4;++i)
    for(j=i+1;j<4;++j)
      if( permutation[i] == permutation[j] )
	return -EINVAL;
  
  for( i=0;i<4; ++i )
    chip->chan_mask_str[i] = permutation[i];
  chip->chan_mask_str[4] = 0;

  m = 0;
  for( i=0;i<4;++i ){
    int chan = (permutation[i]-'0');  /* 0...3 */
    int bit  = 1 << chan;             /* 0001b ... 1000b */
    int bits = bit | (bit << 4);      /* 0x11 .. 0x88 */
    m |= bits;                        /* 0x11 .. 0xff, in order of permutation */
    chip->chan_mask[i] = m;
  }

  snd_printk( KERN_INFO "channel masks set to %s = { %02x %02x %02x %02x }\n", 
	      permutation, chip->chan_mask[0], chip->chan_mask[1], chip->chan_mask[2], chip->chan_mask[3] );
  return 0;

}





/* start/stop talktrough */
static int snd_ad1836_talktrough_mode(ad1836_t* chip, int mode){

  switch(mode){

  case TALKTROUGH_OFF: {

    bf53x_sport_stop(chip->sport);
    
    if( chip->rx_buf ) 
      kfree( chip->rx_buf );

    if( chip->tx_buf && (chip->tx_buf != chip->rx_buf) ) 
      kfree( chip->tx_buf );

    chip->rx_buf = NULL;
    chip->tx_buf = NULL;

    snd_printk( KERN_INFO "talktrough mode: switched off\n" );

    break;

  }

  case TALKTROUGH_STUPID: 
  case TALKTROUGH_SMART: {

    if( chip->talktrough_mode !=  TALKTROUGH_OFF ) 
      return -EBUSY;

    if( chip->tx_substream || chip->rx_substream )
      return -EBUSY;
    
    chip->rx_buf = kmalloc(AD1836_BUFFER_SIZE, GFP_KERNEL);

    if( mode == TALKTROUGH_SMART )
      chip->tx_buf = kmalloc(AD1836_BUFFER_SIZE, GFP_KERNEL);
    else
      chip->tx_buf = chip->rx_buf;

    if( !chip->tx_buf ){
      if( chip->rx_buf) kfree(chip->rx_buf); 
      chip->rx_buf = NULL;
      snd_printk( KERN_ERR "talktrough mode: not enough memory to start\n" );
      return -ENOMEM;
    }
    
    if( bf53x_sport_config_rx_dma( chip->sport, chip->rx_buf,  TALKTROUGH_FRAGMENTS, 
				   AD1836_BUFFER_SIZE/TALKTROUGH_FRAGMENTS, TALKTROUGH_CHANMASK ) ){
      snd_printk( KERN_ERR "talktrough mode: Unable to configure rx dma\n" );
      return -ENODEV;
    }

    if( bf53x_sport_config_tx_dma( chip->sport, chip->tx_buf,  TALKTROUGH_FRAGMENTS, 
				   AD1836_BUFFER_SIZE/TALKTROUGH_FRAGMENTS, TALKTROUGH_CHANMASK ) ){
      snd_printk( KERN_ERR "talktrough mode: Unable to configure tx dma\n" );
      return -ENODEV;
    }


    snd_ad1836_reset_sport_stats(chip);
    bf53x_sport_start(chip->sport);

    snd_printk( KERN_INFO "talktrough mode: switched on (%s) with channel mask 0x%02x\n", 
		(mode == TALKTROUGH_STUPID) ? "stupid" : "smart", TALKTROUGH_CHANMASK );

    break;

  }

  default: 
    return -EINVAL;

  } /* switch mode */

  chip->talktrough_mode = mode;

  return 0;
}


/*************************************************************
 *                 proc and control stuff 
 *************************************************************/



static void snd_ad1836_proc_registers_read( snd_info_entry_t * entry, snd_info_buffer_t * buffer){
  int i;
  ad1836_t *chip = (ad1836_t*) entry->private_data;
  static const char* reg_names[] = {
    "DAC_CTRL_1 ",    "DAC_CTRL_2 ",     "DAC_VOL_1L ",    "DAC_VOL_1R ", 
    "DAC_VOL_2L ",    "DAC_VOL_2R ",     "DAC_VOL_3L ",    "DAC_VOL_3R ",     
    "ADC_PEAK_1L",    "ADC_PEAK_1R",     "ADC_PEAK_2L",    "ADC_PEAK_2R", 
    "ADC_CTRL_1 ",    "ADC_CTRL_2 ",     "ADC_CTRL_3 ",  };

  for( i=DAC_CTRL_1; i<=DAC_VOL_3R;++i)
    snd_iprintf(buffer, "%s 0x%04x\n", reg_names[i], chip->chip_registers[i] );

  for( i=ADC_PEAK_1L; i <= ADC_PEAK_2R; ++i )
    snd_iprintf(buffer, "%s 0x%04x %d dBFS\n", reg_names[i], 
		chip->chip_registers[i], ADC_PEAK_VALUE(chip->chip_registers[i]) );

  for( i=ADC_CTRL_1; i<=ADC_CTRL_3;++i)
    snd_iprintf(buffer, "%s 0x%04x\n", reg_names[i], chip->chip_registers[i] );

  return;
}

static void snd_ad1836_proc_registers_write( snd_info_entry_t * entry, snd_info_buffer_t * buffer){
  ad1836_t *chip = (ad1836_t*) entry->private_data;
  char line[8];
  if( !snd_info_get_line(buffer, line, sizeof(line)) ){
    unsigned int val = simple_strtoul( line, NULL, 0 );
    int reg = val >> 12;
    snd_ad1836_set_register(chip, reg, 0x03ff, val);
  }
  return;
}

static void snd_ad1836_proc_spi_read( snd_info_entry_t * entry, snd_info_buffer_t * buffer){
  ad1836_t *chip = (ad1836_t*) entry->private_data;
  unsigned long timedif = jiffies - chip->spi_irq_timestamp;
  unsigned long freq = (chip->spi_irq_count*HZ*100);
  if(timedif > 0) freq /= timedif;
  snd_iprintf(buffer, "irq: %ld %ld/100s\n", chip->spi_irq_count, freq  );
  snd_ad1836_reset_spi_stats(chip);
  return;
}

static void snd_ad1836_proc_sport_read( snd_info_entry_t * entry, snd_info_buffer_t * buffer){
  ad1836_t *chip = (ad1836_t*) entry->private_data;
  unsigned long timedif = jiffies - chip->sport_irq_timestamp;
  unsigned long freq =    chip->sport_irq_count    *HZ*100;
  unsigned long freq_rx = chip->sport_irq_count_rx *HZ*100;
  unsigned long freq_tx = chip->sport_irq_count_tx *HZ*100;
  
  char buf[256];
  if(timedif > 0){
    freq /= timedif;
    freq_tx /= timedif;
    freq_rx /= timedif;
  } 
  snd_iprintf(buffer, "irq tot: %ld %ld/100s\n", chip->sport_irq_count, freq );
  snd_iprintf(buffer, "irq rx:  %ld %ld/100s\n", chip->sport_irq_count_rx, freq_rx );
  snd_iprintf(buffer, "irq tx:  %ld %ld/100s\n", chip->sport_irq_count_tx, freq_tx );
  snd_ad1836_reset_sport_stats(chip);
  bf53x_sport_dump_stat(chip->sport, buf, sizeof(buf));
  snd_iprintf(buffer, "%s", buf);
  return;
}

static void snd_ad1836_proc_talktrough_read( snd_info_entry_t * entry, snd_info_buffer_t * buffer){
  ad1836_t *chip = (ad1836_t*) entry->private_data;
  static const char* modenames[] = { "off", "stupid", "smart" };
  snd_iprintf(buffer, "%d %s\n", chip->talktrough_mode, modenames[chip->talktrough_mode] );
  return;
}

static void snd_ad1836_proc_talktrough_write( snd_info_entry_t * entry, snd_info_buffer_t * buffer){
  ad1836_t *chip = (ad1836_t*) entry->private_data;
  char line[4];
  int mode;
  if( snd_info_get_line(buffer, line, sizeof(line)) ) return;
  line[3] = 0;

  /* request to panic :-) */
  if( line[0] == 'p' ) 
    panic( "how does it sound with the rest of the kernel frozen?\n" );

  mode =  line[0] - '0' ;
  if( (mode < 0) || (mode > 2) ) return;
  snd_ad1836_talktrough_mode(chip, mode);

  return;
}

static void snd_ad1836_proc_chanmask_read( snd_info_entry_t * entry, snd_info_buffer_t * buffer){
  ad1836_t *chip = (ad1836_t*) entry->private_data;
  snd_iprintf(buffer, "%s\n", chip->chan_mask_str );
  return;
}

static void snd_ad1836_proc_chanmask_write( snd_info_entry_t * entry, snd_info_buffer_t * buffer){
  ad1836_t *chip = (ad1836_t*) entry->private_data;
  char line[5];
  if( snd_info_get_line(buffer, line, sizeof(line)) ) return;
  ad1836_set_chan_masks(chip, line);
  return;
}


/*************************************************************
 *          controls 
 *************************************************************/

#ifndef NOCONTROLS

static int snd_ad1836_loopback_control_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo)
{
  static const char* modenames[] = { "Off", "Stupid", "Smart" };
  uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
  uinfo->count = 1;
  if (uinfo->value.enumerated.item > 2)
    uinfo->value.enumerated.item = 2;
  strcpy(uinfo->value.enumerated.name, modenames[uinfo->value.enumerated.item]);
  return 0;
}


static int snd_ad1836_loopback_control_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  ucontrol->value.enumerated.item[0] = chip->talktrough_mode;
  return 0;
}


static int snd_ad1836_loopback_control_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  if (chip->talktrough_mode != ucontrol->value.enumerated.item[0]) {
    snd_ad1836_talktrough_mode(chip, ucontrol->value.enumerated.item[0]);
    return 1;
  }
  return 0;
}




static int snd_ad1836_volume_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo)
{
  uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
  uinfo->count = 6;
  uinfo->value.integer.min = 0;
  uinfo->value.integer.max = 1023;
  return 0;
}


static int snd_ad1836_volume_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  int i;
  for(i=0;i<6;++i)
    ucontrol->value.integer.value[i] = chip->chip_registers[DAC_VOL_1L+i] & DAC_VOL_MASK;
  return 0;
}


static int snd_ad1836_volume_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  int change=0;
  int i;

  for(i=0;i<6;++i){
    int vol  = ucontrol->value.integer.value[i];
    if (vol < 0) vol = 0; if (vol > 1023) vol = 1023;
    if( (chip->chip_registers[DAC_VOL_1L+i]  & DAC_VOL_MASK) != vol ){
      change = 1;
      snd_ad1836_set_register(chip, DAC_VOL_1L+i, DAC_VOL_MASK, vol);
    }
  }
  return change;
  
}

static int snd_ad1836_adc_gain_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo)
{
  uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
  uinfo->count = 2;
  uinfo->value.integer.min = 0;
  uinfo->value.integer.max = 8;
  return 0;
}

static int snd_ad1836_adc_gain_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  ucontrol->value.integer.value[0] = ADC_GAIN_LEFT( chip->chip_registers[ADC_CTRL_1]);
  ucontrol->value.integer.value[1] = ADC_GAIN_RIGHT(chip->chip_registers[ADC_CTRL_1]);
  return 0;
}

static int snd_ad1836_adc_gain_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  int change=0;
  
  int curr   = chip->chip_registers[ADC_CTRL_1];
  int left   = ucontrol->value.integer.value[0];
  int right  = ucontrol->value.integer.value[1];

  if( (ADC_GAIN_LEFT(curr)) != left ){
    change = 1;
    curr &= ~ ADC_GAIN_LEFT_MASK;
    curr |= (left << ADC_GAIN_LEFT_SHIFT) & ADC_GAIN_LEFT_MASK;
  }
  
  if( (ADC_GAIN_RIGHT(curr)) != right ){
    change = 1;
    curr &= ~ ADC_GAIN_RIGHT_MASK;
    curr |= (right /* << ADC_GAIN_RIGHT_SHIFT */) & ADC_GAIN_RIGHT_MASK;
  }
  
  if(change) 
    snd_ad1836_set_register(chip, ADC_CTRL_1, ADC_GAIN_LEFT_MASK|ADC_GAIN_RIGHT_MASK, curr);

  return change;
  
}




static int snd_ad1836_playback_mute_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo)
{
  uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
  uinfo->count = 6;
  uinfo->value.integer.min = 0;
  uinfo->value.integer.max = 1;
  return 0;
}

static int snd_ad1836_playback_mute_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  int i;
  for(i=0;i<6;++i)
    ucontrol->value.integer.value[i] = (chip->chip_registers[DAC_CTRL_2] & ( 1 << i )) ? 1:0;
  return 0;
}

static int snd_ad1836_playback_mute_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  int curr =  chip->chip_registers[DAC_CTRL_2] &  DAC_MUTE_MASK ;
  int mute = 0;
  int i;

  for(i=0;i<6;++i)
    if( ucontrol->value.integer.value[i] )
      mute |= (1<<i);

  if( curr != mute ){
    snd_ad1836_set_register(chip, DAC_CTRL_2, DAC_MUTE_MASK, mute);
    return 1;
  }

  return 0;
  
}



static int snd_ad1836_capture_mute_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo)
{
  uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
  uinfo->count = 4;
  uinfo->value.integer.min = 0;
  uinfo->value.integer.max = 1;
  return 0;
}

static int snd_ad1836_capture_mute_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  int i;
  for(i=0;i<4;++i)
    ucontrol->value.integer.value[i] = (chip->chip_registers[ADC_CTRL_2] & ( 1 << i )) ? 1:0;
  return 0;
}

static int snd_ad1836_capture_mute_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  int curr =  chip->chip_registers[ADC_CTRL_2] &  ADC_MUTE_MASK ;
  int mute = 0;
  int i;

  for(i=0;i<64;++i)
    if( ucontrol->value.integer.value[i] )
      mute |= (1<<i);

  if( curr != mute ){
    snd_ad1836_set_register(chip, ADC_CTRL_2, ADC_MUTE_MASK, mute);
    return 1;
  }

  return 0;
  
}

static int snd_ad1836_deemph_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo)
{
 static const char* names[] = { "Off", "44.1kHz", "32kHz", "48kHz" };
  uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
  uinfo->count = 1;
  if (uinfo->value.enumerated.item > 3)
    uinfo->value.enumerated.item = 3;
  strcpy(uinfo->value.enumerated.name, names[uinfo->value.enumerated.item]);
  return 0;  
}

static int snd_ad1836_deemph_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  ucontrol->value.enumerated.item[0] = DAC_DEEMPH_VALUE( chip->chip_registers[DAC_CTRL_1] );
  return 0;
}

static int snd_ad1836_deemph_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  if( ucontrol->value.enumerated.item[0] != DAC_DEEMPH_VALUE( chip->chip_registers[DAC_CTRL_1]) ){
    snd_ad1836_set_register(chip, DAC_CTRL_1, DAC_DEEMPH_MASK, ucontrol->value.enumerated.item[0] << DAC_DEEMPH_SHIFT);
    return 1;
  }
  return 0;
  
}

static int snd_ad1836_filter_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo)
{
  uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
  uinfo->count = 1;
  uinfo->value.integer.min = 0;
  uinfo->value.integer.max = 1;
  return 0;
}

static int snd_ad1836_filter_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  ucontrol->value.integer.value[0] = (chip->chip_registers[ADC_CTRL_1] & ADC_HIGHPASS) ? 1:0;
  return 0;
}

static int snd_ad1836_filter_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  if( ucontrol->value.integer.value[0] != ((chip->chip_registers[ADC_CTRL_1] & ADC_HIGHPASS) ? 1:0) ){
    snd_ad1836_set_register(chip, ADC_CTRL_1, ADC_HIGHPASS, (ucontrol->value.integer.value[0]?ADC_HIGHPASS:0) );
    return 1;
  }
  return 0;
}


static int snd_ad1836_diffip_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo)
{
  uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
  uinfo->count = 2;
  uinfo->value.integer.min = 0;
  uinfo->value.integer.max = 1;
  return 0;
}

static int snd_ad1836_diffip_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  ucontrol->value.integer.value[0] = (chip->chip_registers[ADC_CTRL_3] & ADC_LEFT_SE ) ? 1:0;
  ucontrol->value.integer.value[1] = (chip->chip_registers[ADC_CTRL_3] & ADC_RIGHT_SE) ? 1:0;
  return 0;
}

static int snd_ad1836_diffip_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  int change = 0;

  if( ucontrol->value.integer.value[0] != ((chip->chip_registers[ADC_CTRL_3] & ADC_LEFT_SE ) ? 1:0) )
    change = 1;
  if( ucontrol->value.integer.value[0] != ((chip->chip_registers[ADC_CTRL_3] & ADC_RIGHT_SE ) ? 1:0) )
    change = 1;
  if( change ){
    int val  = ucontrol->value.integer.value[0] ? ADC_LEFT_SE : 0;
    val |= ucontrol->value.integer.value[1] ? ADC_RIGHT_SE : 0;
    snd_ad1836_set_register(chip, ADC_CTRL_3, ADC_LEFT_SE|ADC_RIGHT_SE, val );
  }
  return change;
}


static int snd_ad1836_mux_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo)
{
  uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
  uinfo->count = 2;
  uinfo->value.integer.min = 0;
  uinfo->value.integer.max = 1;
  return 0;
}


static int snd_ad1836_mux_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  ucontrol->value.integer.value[0] = (chip->chip_registers[ADC_CTRL_3] & ADC_LEFT_MUX ) ? 1:0;
  ucontrol->value.integer.value[1] = (chip->chip_registers[ADC_CTRL_3] & ADC_RIGHT_MUX) ? 1:0;
  return 0;
}

static int snd_ad1836_mux_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  int change = 0;

  if( ucontrol->value.integer.value[0] != ((chip->chip_registers[ADC_CTRL_3] & ADC_LEFT_MUX ) ? 1:0) )
    change = 1;
  if( ucontrol->value.integer.value[0] != ((chip->chip_registers[ADC_CTRL_3] & ADC_RIGHT_MUX ) ? 1:0) )
    change = 1;
  if( change ){
    int val  = ucontrol->value.integer.value[0] ? ADC_LEFT_MUX : 0;
    val |= ucontrol->value.integer.value[1] ? ADC_RIGHT_MUX : 0;
    snd_ad1836_set_register(chip, ADC_CTRL_3, ADC_LEFT_MUX|ADC_RIGHT_MUX, val );
  }
  return change;
}



static int snd_ad1836_ipsel_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo)
{
  uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
  uinfo->count = 2;
  uinfo->value.integer.min = 0;
  uinfo->value.integer.max = 1;
  return 0;
}

static int snd_ad1836_ipsel_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  ucontrol->value.integer.value[0] = (chip->chip_registers[ADC_CTRL_3] & ADC_LEFT_SEL ) ? 1:0;
  ucontrol->value.integer.value[1] = (chip->chip_registers[ADC_CTRL_3] & ADC_RIGHT_SEL) ? 1:0;
  return 0;
}

static int snd_ad1836_ipsel_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  int change = 0;

  if( ucontrol->value.integer.value[0] != ((chip->chip_registers[ADC_CTRL_3] & ADC_LEFT_SEL ) ? 1:0) )
    change = 1;
  if( ucontrol->value.integer.value[0] != ((chip->chip_registers[ADC_CTRL_3] & ADC_RIGHT_SEL ) ? 1:0) )
    change = 1;
  if( change ){
    int val  = ucontrol->value.integer.value[0] ? ADC_LEFT_SEL : 0;
    val |= ucontrol->value.integer.value[1] ? ADC_RIGHT_SEL : 0;
    snd_ad1836_set_register(chip, ADC_CTRL_3, ADC_LEFT_SEL|ADC_RIGHT_SEL, val );
  }
  return change;
}






static int snd_ad1836_vu_info(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t *uinfo)
{
  uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
  uinfo->count = 4;
  uinfo->value.integer.min = -60;
  uinfo->value.integer.max = 0;
  return 0;
}


static int snd_ad1836_vu_get(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol)
{
  ad1836_t *chip = snd_kcontrol_chip(kcontrol);
  int i;
  for(i=0;i<4;++i)
    ucontrol->value.integer.value[i] = ADC_PEAK_VALUE( chip->chip_registers[ADC_PEAK_1L + i] );
  return 0;
}

static int snd_ad1836_vu_put(snd_kcontrol_t *kcontrol, snd_ctl_elem_value_t *ucontrol){ return 0; }


#define KTRL( xiface, xname, xaccess, xfuncbase ) \
     { .iface = SNDRV_CTL_ELEM_IFACE_ ## xiface, .name  = xname, .index = 0, .access = xaccess, \
       .info  = xfuncbase ## _info, .get  = xfuncbase ## _get, .put  = xfuncbase ## _put, } 

#define KTRLRW( xiface, xname, xfuncbase )  KTRL( xiface, xname, SNDRV_CTL_ELEM_ACCESS_READWRITE, xfuncbase ) 
#define KTRLRO( xiface, xname, xfuncbase )  KTRL( xiface, xname, (SNDRV_CTL_ELEM_ACCESS_READ|SNDRV_CTL_ELEM_ACCESS_VOLATILE), xfuncbase ) 

/* NOTE: I have no idea if I chose the .name fields properly.. */

static snd_kcontrol_new_t snd_ad1836_controls[] __devinitdata = { 
  KTRLRW( CARD,  "Digital Loopback Switch",  snd_ad1836_loopback_control ),
  KTRLRW( MIXER, "Master Playback Volume",   snd_ad1836_volume ),
  KTRLRW( MIXER, "Master Capture Volume",    snd_ad1836_adc_gain ),
  KTRLRW( MIXER, "Master Playback Switch",   snd_ad1836_playback_mute ),
  KTRLRW( MIXER, "Master Capture Switch",    snd_ad1836_capture_mute ),
  KTRLRW( MIXER, "Tone Contol DAC De-emphasis Switch", snd_ad1836_deemph ),
  KTRLRW( MIXER, "Tone Contol ADC High-pass Filter Switch", snd_ad1836_filter ),
  KTRLRW( MIXER, "PCM Capture Differential Switch", snd_ad1836_diffip ),  /* note: off = differential, on = single ended */
  KTRLRW( MIXER, "PCM Capture MUX Switch",   snd_ad1836_mux ),
  KTRLRW( MIXER, "PCM Capture i/p Switch",   snd_ad1836_ipsel ),
  KTRLRO( PCM,   "PCM Capture VU",           snd_ad1836_vu ),


};

#undef KTRL
#undef KTRLRW
#undef KTRLRO


#define AD1836_CONTROLS (sizeof(snd_ad1836_controls)/sizeof(snd_ad1836_controls[0]))

#endif /* ndef NOCONTROLS */


/*************************************************************
 *                pcm methods 
 *************************************************************/


static snd_pcm_hardware_t snd_ad1836_playback_hw = {
  .info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
	   SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_MMAP_VALID),
  .formats =          SNDRV_PCM_FMTBIT_S32_LE,
  .rates =            SNDRV_PCM_RATE_48000|SNDRV_PCM_RATE_96000,
  .rate_min =         48000,
  .rate_max =         96000,
  .channels_min =     2,
  .channels_max =     8,
  .buffer_bytes_max = 128*1024,
  .period_bytes_min = 4*1024, 
  .period_bytes_max = 64*1024,
  .periods_min =      2,
  .periods_max =      32,
};

/* TODO: this is identical to the capture_hw, can we use just one? */
static snd_pcm_hardware_t snd_ad1836_capture_hw = {
  .info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED | 
	   SNDRV_PCM_INFO_BLOCK_TRANSFER |  SNDRV_PCM_INFO_MMAP_VALID),
  .formats =          SNDRV_PCM_FMTBIT_S32_LE,
  .rates =            SNDRV_PCM_RATE_48000|SNDRV_PCM_RATE_96000,
  .rate_min =         48000,
  .rate_max =         96000,
  .channels_min =     2,
  .channels_max =     8,
  .buffer_bytes_max = 128*1024,
  .period_bytes_min = 4*1024, 
  .period_bytes_max = 64*1024,
  .periods_min =      2,
  .periods_max =      32,
};


static int snd_ad1836_playback_open(snd_pcm_substream_t* substream){

  ad1836_t* chip = _snd_pcm_substream_chip(substream);

  snd_printk_marker();

  if( chip->talktrough_mode != TALKTROUGH_OFF ) 
    return -EBUSY;

  substream->runtime->hw = snd_ad1836_playback_hw;
  chip->tx_substream = substream;

  return 0;

}
static int snd_ad1836_capture_open(snd_pcm_substream_t* substream){ 

  ad1836_t* chip = _snd_pcm_substream_chip(substream);

  snd_printk_marker();

  if( chip->talktrough_mode != TALKTROUGH_OFF ) 
    return -EBUSY;

  substream->runtime->hw = snd_ad1836_capture_hw;
  chip->rx_substream = substream;

  return 0;
}


static int snd_ad1836_playback_close(snd_pcm_substream_t* substream){

  ad1836_t* chip = _snd_pcm_substream_chip(substream);

  snd_printk_marker();

  chip->tx_substream = NULL;
  return 0;
}


static int snd_ad1836_capture_close(snd_pcm_substream_t* substream){

  ad1836_t* chip = _snd_pcm_substream_chip(substream);

  snd_printk_marker();

  chip->rx_substream = NULL;

  return 0;
}



static int snd_ad1836_hw_params( snd_pcm_substream_t* substream, snd_pcm_hw_params_t* hwparams){

  snd_printk_marker();

  if( snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hwparams)) < 0 )
    return -ENOMEM;

  return 0;

}


static int snd_ad1836_hw_free(snd_pcm_substream_t * substream){
  snd_printk_marker();
  snd_pcm_lib_free_pages(substream);
  return 0;
}

/* little helper */

static int sport_is_running(struct bf53x_sport* sport){
  unsigned int stat_rx, stat_tx;
  bf53x_sport_check_status(sport, NULL, &stat_rx, &stat_tx);
  return (stat_rx & DMA_RUN) || (stat_tx & DMA_RUN);
}


/* the following works for rx and tx alike, 
   we compare substream with chip->subsream->[tr]x */

static int snd_ad1836_prepare( snd_pcm_substream_t* substream ){

  ad1836_t* chip = _snd_pcm_substream_chip(substream);
  snd_pcm_runtime_t* runtime = substream->runtime;

  void* buf_addr      = (void*) runtime->dma_area;
  int  fragcount      = runtime->periods;
  int  fragsize_bytes = frames_to_bytes(runtime, runtime->period_size);
  int  chan_mask      = chip->chan_mask[((runtime->channels&0xe)>>1)-1];
  int was_running     = sport_is_running(chip->sport);
  int err=0;

  snd_printk_marker();

  snd_assert( (substream == chip->rx_substream) || (substream == chip->tx_substream), return -EINVAL );

  snd_assert( (runtime->channels == 2)||(runtime->channels == 4)
	      ||(runtime->channels == 6)||(runtime->channels == 8),
	      return -EINVAL);

  /* if we were running (because of the other direction), stop the sport */
 
  if( was_running ) 
    bf53x_sport_stop(chip->sport);


  /* one of the following two must be true */
  if( substream == chip->rx_substream )
    err = bf53x_sport_config_rx_dma( chip->sport, buf_addr , fragcount, fragsize_bytes, chan_mask );

  if( substream == chip->tx_substream )
    err = bf53x_sport_config_tx_dma( chip->sport, buf_addr , fragcount, fragsize_bytes, chan_mask );


  /* one of the following two may be true: dummy config for other channel */
  if( !err && !chip->tx_substream ){ 
    err = bf53x_sport_config_tx_dma( chip->sport, &dummy_buf_tx , 2, sizeof(dummy_buf_tx)/2, chan_mask );
    sport_disable_dma_tx(chip->sport);
  }

  if( !err && !chip->rx_substream ){
    err = bf53x_sport_config_rx_dma( chip->sport, &dummy_buf_rx , 2, sizeof(dummy_buf_rx)/2, chan_mask );
    sport_disable_dma_rx(chip->sport);
  }

  /* if we were running, continue with the new settings */
  if(!err && was_running) 
    bf53x_sport_start(chip->sport);

  return err;

}

static int snd_ad1836_trigger( snd_pcm_substream_t* substream, int cmd){

  ad1836_t* chip = _snd_pcm_substream_chip(substream);

  int runs = sport_is_running(chip->sport);
  int mask = 0;

#ifdef CONFIG_SND_DEBUG
  snd_printk( KERN_INFO "%s(%d)\n", __FUNCTION__, cmd );
#endif

  if( substream == chip->rx_substream ) mask = RUN_RX;
  if( substream == chip->tx_substream ) mask = RUN_TX;

  if( ! mask ) return -EINVAL;

  switch(cmd){
  case SNDRV_PCM_TRIGGER_START: chip->runmode |=  mask; break;
  case SNDRV_PCM_TRIGGER_STOP:  chip->runmode &= ~mask; break;
  default:
    return -EINVAL;
  }

  if( chip->runmode && !runs) bf53x_sport_start(chip->sport); 
  if(!chip->runmode &&  runs) bf53x_sport_stop(chip->sport);

  return 0;

}

/* we might as well merge the following too...*/

static snd_pcm_uframes_t snd_ad1836_playback_pointer( snd_pcm_substream_t* substream ){

  ad1836_t* chip = _snd_pcm_substream_chip(substream);
  snd_pcm_runtime_t* runtime = substream->runtime;

  char* buf  = (char*) runtime->dma_area;
  char* curr = (char*) bf53x_sport_curr_addr_tx(chip->sport);
  unsigned long diff = curr - buf;
  unsigned long bytes_per_frame = runtime->channels*sizeof(long) ;
  size_t frames = diff / bytes_per_frame;
 
#ifdef CONFIG_SND_DEBUG_CURRPTR
  snd_printk( KERN_INFO " bf53x_sport_curr_addr_tx() == %p\n", curr );
#endif

  /* the loose syncing used here is accurate enough for alsa, but 
     due to latency in the dma, the following may happen occasionally, 
     and pcm_lib shouldn't complain */
  if( frames == runtime->buffer_size ) 
    frames = 0;

  return frames;

}


static snd_pcm_uframes_t snd_ad1836_capture_pointer( snd_pcm_substream_t* substream ){

  ad1836_t* chip = _snd_pcm_substream_chip(substream);
  snd_pcm_runtime_t* runtime = substream->runtime;

  char* buf  = (char*) runtime->dma_area;
  char* curr = (char*) bf53x_sport_curr_addr_rx(chip->sport);
  unsigned long diff = curr - buf;
  unsigned long bytes_per_frame = runtime->channels*sizeof(long) ;
  size_t frames = diff / bytes_per_frame;
  
#ifdef CONFIG_SND_DEBUG_CURRPTR
  snd_printk( KERN_INFO " bf53x_sport_curr_addr_rx() == %p\n", curr );
#endif 

  /* the loose syncing used here is accurate enough for alsa, but 
     due to latency in the dma, the following may happen occasionally, 
     and pcm_lib shouldn't complain */
  if( frames == runtime->buffer_size ) 
    frames = 0;

  return frames;

}





/* pcm method tables */

static snd_pcm_ops_t snd_ad1836_playback_ops = {
  .open      = snd_ad1836_playback_open,
  .close     = snd_ad1836_playback_close,
  .ioctl     = snd_pcm_lib_ioctl,
  .hw_params = snd_ad1836_hw_params,
  .hw_free   = snd_ad1836_hw_free,
  .prepare   = snd_ad1836_prepare,
  .trigger   = snd_ad1836_trigger,
  .pointer   = snd_ad1836_playback_pointer,
};


static snd_pcm_ops_t snd_ad1836_capture_ops = {
  .open  = snd_ad1836_capture_open,
  .close = snd_ad1836_capture_close,
  .ioctl = snd_pcm_lib_ioctl,  
  .hw_params = snd_ad1836_hw_params,
  .hw_free   = snd_ad1836_hw_free,
  .prepare   = snd_ad1836_prepare,
  .trigger   = snd_ad1836_trigger,
  .pointer   = snd_ad1836_capture_pointer,
};


/************************************************************* 
 *      card and device 
 *************************************************************/


// chip-specific destructor
// (see "PCI Resource Managements")
static int snd_ad1836_free(ad1836_t *chip)
{
  
  if( chip->spi_chan ) {
    snd_ad1836_set_register(chip, DAC_CTRL_2, DAC_MUTE_MASK, DAC_MUTE_MASK);  /* mute DAC's */
    snd_ad1836_set_register(chip, ADC_CTRL_2, ADC_MUTE_MASK, ADC_MUTE_MASK);  /* mute ADC's */
    snd_ad1836_set_register(chip, DAC_CTRL_1, DAC_PWRDWN, DAC_PWRDWN);  /* power-down DAC's */
    snd_ad1836_set_register(chip, ADC_CTRL_1, ADC_PRWDWN, ADC_PRWDWN);  /* power-down ADC's */
    bf53x_spi_destroy_channel( chip->spi_chan );
  }
  
  if( chip->talktrough_mode != TALKTROUGH_OFF) 
    snd_ad1836_talktrough_mode(chip, TALKTROUGH_OFF);

  kfree(chip);
  return 0;
}

// component-destructor, wraps snd_ad1836_free for use in snd_device_ops_t
static int snd_ad1836_dev_free(snd_device_t *device)
{
  ad1836_t *chip = snd_magic_cast(ad1836_t, device->device_data, return -ENXIO);
  return snd_ad1836_free(chip);
}

static snd_device_ops_t snd_ad1836_ops = {
  .dev_free = snd_ad1836_dev_free,
};


/* create the card struct, 
 *   add - low-level device, 
 *       - spi sport and registers, 
 *       - a proc entry, 
 *       - and a pcm device 
 */

static int __devinit snd_ad1836_create(snd_card_t *card,
				       struct bf53x_spi* spi, 
				       struct bf53x_sport* sport, 
				       ad1836_t **rchip)
{
  
  ad1836_t *chip;
  int err,i;
  
  *rchip = NULL;
  
  /* spi and sport availability have been ensured by caller (the module init) */
  
  /* allocate a chip-specific data with magic-alloc */
  chip = snd_magic_kcalloc(ad1836_t, 0, GFP_KERNEL);
  if (chip == NULL)
    return -ENOMEM;
  
  chip->card  = card;
  chip->spi   = spi;
  chip->sport = sport;
  
  init_waitqueue_head(&chip->spi_waitq);
  
  chip->spi_chan = bf53x_spi_create_channel(chip->spi, 
					    1, /* master mode */
					    16, /* baud rate SCK = HCLK/(2*SPIBAUD) SCK = 2MHz */
					    1, /* word size = 16 bits */
					    (1<<SND_CONFIG_BLACKFIN_PFBIT), /* pf4 bit enabled */
					    0x0, /* no special configs */
					    &ad1836_spi_handler, chip);
  
  if( !chip->spi_chan ){
    snd_printk( KERN_ERR "Unable to allocate spi channel\n");    
    snd_ad1836_free(chip);
    return -ENODEV;
  }
  
  snd_ad1836_reset_spi_stats(chip);

  for(i=0; i<16; ++i)
    chip->chip_registers[i] = (i<<12);
  
  for(i=ADC_PEAK_1L; i<=ADC_PEAK_2R; ++i)
    chip->chip_registers[i] |= ADC_READ;
  
  /* see if we are connected by writing (preferably something useful)
   * to the chip, and see if we get an IRQ */
  
  /* sport in aux/slave mode cf daughtercard schematics */
  err = snd_ad1836_set_register(chip, ADC_CTRL_2, (ADC_AUX_MASTER|ADC_SOUT_MASK),  
				                  ( /*ADC_AUX_MASTER|*/ ADC_SOUT_PMAUX));  
  err = err || snd_ad1836_set_register(chip, DAC_CTRL_1, DAC_PWRDWN, 0);  /* power-up DAC's */
  err = err || snd_ad1836_set_register(chip, ADC_CTRL_1, ADC_PRWDWN, 0);  /* power-up ADC's */
  
  /* set volume to full scale, (you might assume these won't fail anymore) */
  err = err || snd_ad1836_set_register(chip, DAC_VOL_1L, DAC_VOL_MASK, DAC_VOL_MASK);
  err = err || snd_ad1836_set_register(chip, DAC_VOL_1R, DAC_VOL_MASK, DAC_VOL_MASK);
  err = err || snd_ad1836_set_register(chip, DAC_VOL_2L, DAC_VOL_MASK, DAC_VOL_MASK);
  err = err || snd_ad1836_set_register(chip, DAC_VOL_2R, DAC_VOL_MASK, DAC_VOL_MASK);
  err = err || snd_ad1836_set_register(chip, DAC_VOL_3L, DAC_VOL_MASK, DAC_VOL_MASK);
  err = err || snd_ad1836_set_register(chip, DAC_VOL_3R, DAC_VOL_MASK, DAC_VOL_MASK);

  chip->poll_reg = ADC_PEAK_1L;
  
  if(err){
    snd_printk( KERN_ERR "Unable to set chip registers.\n");    
    snd_ad1836_free(chip);
    return -ENODEV;
  }
  
  err = 0;
  err = err || bf53x_sport_config_rx(sport, RFSR, 0x1f /* 32 bit word len */, 0, 0 );
  err = err || bf53x_sport_config_tx(sport, TFSR, 0x1f /* 32 bit word len */, 0, 0 );
  err = err || bf53x_sport_set_multichannel(sport, 8 /* channels */, 1 /* packed */ );

  err = err || bf53x_sport_config_rx_dma( sport, &dummy_buf_rx , 2, sizeof(dummy_buf_tx)/2, 0x11 );
  err = err || bf53x_sport_config_tx_dma( sport, &dummy_buf_tx , 2, sizeof(dummy_buf_rx)/2, 0x11 );

  sport_disable_dma_rx(sport);
  sport_disable_dma_tx(sport);

  if(err){
    snd_printk( KERN_ERR "Unable to set sport configuration\n");
    snd_ad1836_free(chip);
    return -ENODEV;
  }

  ad1836_set_chan_masks(chip, "0123");

  err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, chip, &snd_ad1836_ops);


  if(!err){
    snd_info_entry_t* proc_entry;
    err = snd_card_proc_new(card, "registers", &proc_entry); 
    if(!err)
      snd_info_set_text_ops( proc_entry, chip, 1024, snd_ad1836_proc_registers_read);
      proc_entry->mode = S_IFREG | S_IRUGO | S_IWUSR;
      proc_entry->c.text.write_size = 8;
      proc_entry->c.text.write = snd_ad1836_proc_registers_write;
  }

  if(!err){
    snd_info_entry_t* proc_entry;
    err = snd_card_proc_new(card, "spi", &proc_entry); 
    if(!err)
      snd_info_set_text_ops( proc_entry, chip, 1024, snd_ad1836_proc_spi_read);
  }

  if(!err){
    snd_info_entry_t* proc_entry;
    err = snd_card_proc_new(card, "sport", &proc_entry); 
    if(!err)
      snd_info_set_text_ops( proc_entry, chip, 1024, snd_ad1836_proc_sport_read);
  }

  if(!err){
    snd_info_entry_t* proc_entry;
    err = snd_card_proc_new(card, "chan_mask", &proc_entry); 
    if(!err){
      snd_info_set_text_ops( proc_entry, chip, 1024, snd_ad1836_proc_chanmask_read);
      proc_entry->mode = S_IFREG | S_IRUGO | S_IWUSR;
      proc_entry->c.text.write_size = 5;
      proc_entry->c.text.write = snd_ad1836_proc_chanmask_write;
    }
  }

  if(!err){
    snd_info_entry_t* proc_entry;
    err = snd_card_proc_new(card, "talktrough", &proc_entry); 
    if(!err){
      snd_info_set_text_ops( proc_entry, chip, 1024, snd_ad1836_proc_talktrough_read);
      proc_entry->mode = S_IFREG | S_IRUGO | S_IWUSR;
      proc_entry->c.text.write_size = 4;
      proc_entry->c.text.write = snd_ad1836_proc_talktrough_write;
    }
  }


#ifndef NOCONTROLS
  if(!err)
    for( i=0; (i<AD1836_CONTROLS) && !err; ++i )
      err = snd_ctl_add(card, snd_ctl_new1(&(snd_ad1836_controls[i]), chip));
#endif

  if(!err){
    snd_pcm_t* pcm;
    /* 1 playback and 1 capture substream, of 2-8 channels each */
    err = snd_pcm_new(card, CHIP_NAME, 0, 1, 1, &pcm);
    if(!err){
      pcm->private_data = chip;
      chip->pcm = pcm;
      strcpy(pcm->name, CHIP_NAME);
      snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_ad1836_playback_ops);
      snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,  &snd_ad1836_capture_ops);
      snd_pcm_lib_preallocate_pages_for_all(chip->pcm, SNDRV_DMA_TYPE_CONTINUOUS,
					    snd_dma_continuous_data(GFP_KERNEL),
					    AD1836_BUFFER_SIZE, AD1836_BUFFER_SIZE);
      

      snd_assert( ((ad1836_t*)(pcm->private_data))->pcm == pcm, panic("inconsistency") );

    }
  }

  if(err) {
    snd_ad1836_free(chip);
    return err;
  }

  *rchip = chip;
  return 0;
  
}



/************************************************************* 
 *                 ALSA Card Level 
 *************************************************************/


/* probe for an ad1836 connected to spi and sport, and initialize *the_card */

static int __devinit snd_bf53x_adi1836_probe(struct bf53x_spi* spi, 
					     struct bf53x_sport* sport, 
					     snd_card_t** the_card)
{

  static int dev=0;
  snd_card_t *card;   
  ad1836_t *chip;    
  int err;

  if (dev >= SNDRV_CARDS)  return -ENODEV;

  if (!enable[dev]) {
    dev++;
    return -ENOENT;
  }


  card = snd_card_new( index[dev], id[dev], THIS_MODULE, 0 );
  if( card == NULL ) 
    return -ENOMEM;


  if( (err = snd_ad1836_create(card, spi, sport, &chip)) < 0 ) {
    snd_card_free(card);
    return err;
  }

  card->private_data = (void*) chip;

  snd_assert( ((ad1836_t*)(card->private_data))->card == card, panic("inconsistency") );

  strcpy(card->driver, "adi1836");
  strcpy(card->shortname, CHIP_NAME);
  sprintf(card->longname, "%s at SPI irq %d/%d, SPORT%d rx/tx dma %d/%d irq %d/%d/%d ", 
	  card->shortname,
	  CONFIG_SND_BLACKFIN_SPI_IRQ_DATA,   /* we could get these from the spi and */
	  CONFIG_SND_BLACKFIN_SPI_IRQ_ERR,    /* sport structs, but then we'd have to publicize them */
	  CONFIG_SND_BLACKFIN_SPORT,          /* and we set them ourselves below anyway */
	  CONFIG_SND_BLACKFIN_SPORT_DMA_RX,
	  CONFIG_SND_BLACKFIN_SPORT_DMA_TX,
	  CONFIG_SND_BLACKFIN_SPORT_IRQ_RX,
	  CONFIG_SND_BLACKFIN_SPORT_IRQ_TX,
	  CONFIG_SND_BLACKFIN_SPORT_IRQ_ERR
	  );

  if ((err = snd_card_register(card)) < 0) {
    snd_card_free(card);
    return err;
  }


  *the_card = card;
  ++dev;
  return 0;
}


static __devexit void snd_bf53x_adi1836_remove(snd_card_t* card){

  snd_card_free(card);

  return;
}


static inline void* frag2addr( void* buf, int frag, size_t fragsize_bytes){
  char* addr = buf;
  return addr + frag*fragsize_bytes;
}


/* sport irq handler, called directly from module level */

static irqreturn_t snd_adi1836_sport_handler(ad1836_t* chip, int irq){

  irqreturn_t handled = IRQ_NONE;
  unsigned int rx_stat, tx_stat;
  
  ++(chip->sport_irq_count);

  bf53x_sport_check_status( chip->sport, NULL, &rx_stat, &tx_stat );  
  
  if( rx_stat & DMA_DONE ){
    
    ++(chip->sport_irq_count_rx);
    handled = IRQ_HANDLED;
    
    if( chip->rx_substream )
      snd_pcm_period_elapsed(chip->rx_substream);

    if( chip->talktrough_mode == TALKTROUGH_SMART  ){
      // copy last rx frag to next tx frag
      void* src;
      void* dst;
      int   cnt = (AD1836_BUFFER_SIZE/TALKTROUGH_FRAGMENTS);
      static int   src_frag = -1;
      static int   dst_frag = -1;
      if( src_frag == -1 ) src_frag = bf53x_sport_curr_frag_rx(chip->sport)-1; else ++src_frag;
      if( dst_frag == -1 ) dst_frag = bf53x_sport_curr_frag_tx(chip->sport)+1; else ++dst_frag;
      if( src_frag < 0 ) src_frag += TALKTROUGH_FRAGMENTS;
      if( src_frag >= TALKTROUGH_FRAGMENTS ) src_frag -= TALKTROUGH_FRAGMENTS;
      if( dst_frag >= TALKTROUGH_FRAGMENTS ) dst_frag -= TALKTROUGH_FRAGMENTS;
      src = frag2addr( chip->rx_buf, src_frag, cnt );
      dst = frag2addr( chip->tx_buf, dst_frag, cnt );
      bf53x_cache_flushinv(src,cnt); 
#if 0
      b4copy(src,dst,cnt);    
#else
      memmove(dst,src,cnt);
#endif
      bf53x_cache_flush(dst,cnt);

    }


    /* issue a query for a vu meter, one per frag */
#if 1
    {
      ++(chip->poll_reg);
      if( (chip->poll_reg < ADC_PEAK_1L) || ( chip->poll_reg > ADC_PEAK_2R ) )
	chip->poll_reg = ADC_PEAK_1L;
      
      bf53x_spi_transceive(chip->spi_chan, (chip->poll_reg<<12) | ADC_READ , NULL, NULL);
    }
#endif
  }  /* handle rx */
  


  if( tx_stat & DMA_DONE ){
    
    ++(chip->sport_irq_count_tx);
    handled = IRQ_HANDLED;

    if( chip->tx_substream )
      snd_pcm_period_elapsed(chip->tx_substream);

    
  } /* handle tx */
    

  return handled;

} /* sport handler */



/* ************************************************************
 * Module level
 *
 * tie the spi, the sport and the card into a module
 * this module probes for 1 chip, connected to the 
 * SPORT configured at compile time, and to the SPI.
 * this assumes this sound driver is the only one 
 * using the SPI.  
 *
 * TODO: move the spi and sport api's to arch/bfinnommu
 *
 *************************************************************/


MODULE_AUTHOR("Luuk van Dijk <blackfin@mndmttr.nl>");
MODULE_DESCRIPTION("BF53x/ADI 1836");
MODULE_LICENSE("GPL");
MODULE_CLASSES("{sound}");
MODULE_DEVICES("{{Analog Devices,ADI1836}}");


static struct bf53x_spi*   spi=NULL;
static struct bf53x_sport* sport=NULL;
static snd_card_t*         card=NULL;



static irqreturn_t spi_handler(int irq, void *dev_id, struct pt_regs *regs){
  //  snd_printk( KERN_INFO "in module spi handler\n" );  
  if(spi) return bf53x_spi_irq_handler(spi, irq); 
  return IRQ_NONE;
}

static irqreturn_t sport_handler(int irq, void *dev_id, struct pt_regs *regs){
  //  snd_printk( KERN_INFO "in module sport handler\n" );  
  if(card) return snd_adi1836_sport_handler( (ad1836_t*)(card->private_data), irq );
  return IRQ_NONE;
}

static irqreturn_t sport_error_handler(int irq, void *dev_id, struct pt_regs *regs){

  unsigned int status;

  if(!sport) return IRQ_NONE;
  if( bf53x_sport_check_status(sport, &status, NULL, NULL) ){
    snd_printk( KERN_ERR "error checking status ??" );
    return IRQ_NONE;
  }

  if( status & (TOVF|TUVF|ROVF|RUVF) ){
    snd_printk( KERN_WARNING  "sport status error:%s%s%s%s\n", 
		status & TOVF ? " TOVF" : "", 
		status & TUVF ? " TUVF" : "", 
		status & ROVF ? " ROVF" : "", 
		status & RUVF ? " RUVF" : "" );
    bf53x_sport_stop(sport);
  }

  return IRQ_HANDLED;

}


/* idempotent cleanup, used in __init as well, so no __exit */
/* TODO: should release dma and irq's */

static void /* __exit */ snd_bf53x_adi1836_exit(void){
  
  if( card ){
    snd_card_t*  tmp_card = card;
    card = NULL;
    snd_bf53x_adi1836_remove(tmp_card);
  }

  if( sport ){
    struct bf53x_sport* tmp_sport = sport;
    sport = NULL;
    bf53x_sport_done( tmp_sport );
  }

  if( spi ){
    struct bf53x_spi* tmp_spi=spi;
    spi=NULL;
    bf53x_spi_done( tmp_spi );
  }

  return;

}


/* TODO: failure to alloc spi or sport should release dma and irq's */


static int __init snd_bf53x_adi1836_init(void){
  
  int err;

  if( (spi = bf53x_spi_init(/* CONFIG_SND_BLACKFIN_SPI_DMA */ -1, 
			     CONFIG_SND_BLACKFIN_SPI_IRQ_DATA, 
			     CONFIG_SND_BLACKFIN_SPI_IRQ_ERR, 0) ) == NULL ) 
    return -ENOMEM;

  if( request_irq(CONFIG_SND_BLACKFIN_SPI_IRQ_DATA, &spi_handler, SA_SHIRQ, "SPI Data", NULL ) ){
    snd_printk( KERN_ERR "Unable to allocate spi data IRQ %d\n", CONFIG_SND_BLACKFIN_SPI_IRQ_DATA);
    snd_bf53x_adi1836_exit();
    return -ENODEV;
  }

  
  if( request_irq(CONFIG_SND_BLACKFIN_SPI_IRQ_ERR, &spi_handler, SA_SHIRQ, "SPI Error", NULL ) ){
    snd_printk( KERN_ERR "Unable to allocate spi error IRQ %d\n", CONFIG_SND_BLACKFIN_SPI_IRQ_ERR);
    snd_bf53x_adi1836_exit();
    return -ENODEV;
  }

  enable_irq(CONFIG_SND_BLACKFIN_SPI_IRQ_DATA);
  enable_irq(CONFIG_SND_BLACKFIN_SPI_IRQ_ERR);

  if( (sport = bf53x_sport_init(CONFIG_SND_BLACKFIN_SPORT,  
				CONFIG_SND_BLACKFIN_SPORT_DMA_RX, 
				CONFIG_SND_BLACKFIN_SPORT_DMA_TX ) ) == NULL ){ 
    snd_bf53x_adi1836_exit();
    return -ENOMEM;
  }

  /* further configuration of the sport is in the device constuctor */

  if( request_irq(CONFIG_SND_BLACKFIN_SPORT_IRQ_ERR, &sport_error_handler, SA_SHIRQ, "SPORT Error", NULL ) ){
    snd_printk( KERN_ERR "Unable to allocate sport error IRQ %d\n", CONFIG_SND_BLACKFIN_SPORT_IRQ_ERR);
    snd_bf53x_adi1836_exit();
    return -ENODEV;
  }

  enable_irq(CONFIG_SND_BLACKFIN_SPORT_IRQ_ERR);

  /* sport_init() requested the dma channel through the official api, 
   * but we override the irq, because 
   * the implementation in bfinnommu/kernel/dma.c adds a lot of overhead, 
   * without actually solving any problem for us.  
   */

#define IRQREQFLAG 0 /* IRQ_FLG_REPLACE this is a bug(?) in bfinnommu/mach-bf533/ints-priority.c */

  if( request_irq(CONFIG_SND_BLACKFIN_SPORT_IRQ_RX, &sport_handler, IRQREQFLAG, "SPORT RX Data", NULL ) ){
    snd_printk( KERN_ERR "Unable to allocate sport RX data IRQ %d\n", CONFIG_SND_BLACKFIN_SPORT_IRQ_RX);
    snd_bf53x_adi1836_exit();
    return -ENODEV;
  }

  if( CONFIG_SND_BLACKFIN_SPORT_IRQ_RX != CONFIG_SND_BLACKFIN_SPORT_IRQ_TX )
    if( request_irq(CONFIG_SND_BLACKFIN_SPORT_IRQ_TX, &sport_handler, IRQREQFLAG, "SPORT TX Data", NULL ) ){
      snd_printk( KERN_ERR "Unable to allocate sport TX data IRQ %d\n", CONFIG_SND_BLACKFIN_SPORT_IRQ_TX);
      snd_bf53x_adi1836_exit();
      return -ENODEV;
    }
  
#undef IRQREQFLAG

  enable_irq(CONFIG_SND_BLACKFIN_SPORT_IRQ_TX);
  enable_irq(CONFIG_SND_BLACKFIN_SPORT_IRQ_RX);


  err = snd_bf53x_adi1836_probe(spi, sport, &card);

  if(err)
    snd_bf53x_adi1836_exit();

#ifdef INIT_TALKTROUGH
  snd_ad1836_talktrough_mode( (ad1836_t*) (card->private_data) , TALKTROUGH_SMART );
#endif

#if 0
  /* get a perfectly working talktrough.... */
  panic("see if this works....\n");
#endif

  return err;

}


module_init(snd_bf53x_adi1836_init);
module_exit(snd_bf53x_adi1836_exit);
