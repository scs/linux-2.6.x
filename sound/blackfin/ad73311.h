/*
 * File:         ad73311.h 
 * Description:  definitions for AD73311 registers
 * Rev:          $Id$
 * Created:      Wed Jan 11, 2006
 * Author:       Roy Huang
 * mail:         Roy.Huang@analog.com
 * 
 * Copyright (C) 2006 Analog Device Inc.
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

#if CONFIG_SND_BFIN_SPORT == 0
#define SPORT_ERR_IRQ	IRQ_SPORT0_ERROR
#define SPORT_DMA_RX	CH_SPORT0_RX
#define SPORT_DMA_TX	CH_SPORT0_TX
#define SPORT_TCR1	SPORT0_TCR1
#define SPORT_TCR2	SPORT0_TCR2
#define SPORT_TX	SPORT0_TX
#define SPORT_STAT	SPORT0_STAT
#else
#define SPORT_ERR_IRQ	IRQ_SPORT1_ERROR
#define SPORT_DMA_RX	CH_SPORT1_RX
#define SPORT_DMA_TX	CH_SPORT1_TX
#define SPORT_TCR1	SPORT1_TCR1
#define SPORT_TCR2	SPORT1_TCR2
#define SPORT_TX	SPORT1_TX
#define SPORT_STAT	SPORT1_STAT
#endif

#define AD_CONTROL	0x8000
#define AD_DATA		0x0000
#define AD_READ		0x4000
#define AD_WRITE	0x0000

/* Control register A */
#define CTRL_REG_A	(0 << 8)

#define MODE_PRO	0x00
#define MODE_DATA	0x01
#define MODE_MIXED	0x03
#define DLB		0x04
#define SLB		0x08
#define DEVC(x)		((x & 0x7) << 4)
#define RESET		0x80

/* Control register B */
#define CTRL_REG_B	(1 << 8)

#define DIRATE(x)	(x & 0x3)
#define SCDIV(x)	((x & 0x3) << 2)
#define MCDIV(x)	((x & 0x7) << 4)
#define CEE		(1 << 7)

/* Control register C */
#define CTRL_REG_C	(2 << 8)

#define PUDEV		( 1 << 0 )
#define PUADC		( 1 << 3 )
#define PUDAC		( 1 << 4 )
#define PUREF		( 1 << 5 )
#define REFUSE		( 1 << 6 )

/* Control register D */
#define CTRL_REG_D	(3 << 8)

#define IGS(x)		(x & 0x7)
#define RMOD		( 1 << 3 )
#define OGS(x)		((x & 0x7) << 4)
#define MUTE		(x << 7)

/* Control register E */
#define CTRL_REG_E	(4 << 8)

#define DA(x)		(x & 0x1f)
#define IBYP		( 1 << 5 )

/* Control register F */
#define CTRL_REG_F	(5 << 8)

#define SEEN		( 1 << 5 )
#define INV		( 1 << 6 )
#define ALB		( 1 << 7 )
