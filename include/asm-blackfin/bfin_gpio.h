/*
 * File:         arch/blackfin/kernel/bfin_gpio.h
 * Based on:
 * Author:	 Michael Hennerich (hennerich@blackfin.uclinux.org)
 *
 * Created:	
 * Description:
 *
 * Rev:          $Id$
 *
 * Modified:
 *               Copyright 2004-2006 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/*
*  Number     BF537/6/4    BF561    BF533/2/1
*             
*  GPIO_0       PF0         PF0        PF0
*  GPIO_1       PF1         PF1        PF1
*  GPIO_2       PF2         PF2        PF2
*  GPIO_3       PF3         PF3        PF3
*  GPIO_4       PF4         PF4        PF4
*  GPIO_5       PF5         PF5        PF5
*  GPIO_6       PF6         PF6        PF6
*  GPIO_7       PF7         PF7        PF7
*  GPIO_8       PF8         PF8        PF8
*  GPIO_9       PF9         PF9        PF9
*  GPIO_10      PF10        PF10       PF10
*  GPIO_11      PF11        PF11       PF11
*  GPIO_12      PF12        PF12       PF12
*  GPIO_13      PF13        PF13       PF13
*  GPIO_14      PF14        PF14       PF14
*  GPIO_15      PF15        PF15       PF15
*  GPIO_16      PG0         PF16
*  GPIO_17      PG1         PF17
*  GPIO_18      PG2         PF18
*  GPIO_19      PG3         PF19
*  GPIO_20      PG4         PF20
*  GPIO_21      PG5         PF21
*  GPIO_22      PG6         PF22
*  GPIO_23      PG7         PF23
*  GPIO_24      PG8         PF24
*  GPIO_25      PG9         PF25
*  GPIO_26      PG10        PF26
*  GPIO_27      PG11        PF27
*  GPIO_28      PG12        PF28
*  GPIO_29      PG13        PF29
*  GPIO_30      PG14        PF30
*  GPIO_31      PG15        PF31
*  GPIO_32      PH0         PF32
*  GPIO_33      PH1         PF33
*  GPIO_34      PH2         PF34
*  GPIO_35      PH3         PF35
*  GPIO_36      PH4         PF36
*  GPIO_37      PH5         PF37
*  GPIO_38      PH6         PF38
*  GPIO_39      PH7         PF39
*  GPIO_40      PH8         PF40
*  GPIO_41      PH9         PF41
*  GPIO_42      PH10        PF42
*  GPIO_43      PH11        PF43
*  GPIO_44      PH12        PF44
*  GPIO_45      PH13        PF45
*  GPIO_46      PH14        PF46
*  GPIO_47      PH15        PF47
*/

#ifndef __ARCH_BLACKFIN_GPIO_H__
#define __ARCH_BLACKFIN_GPIO_H__

#define	GPIO_0	0
#define	GPIO_1	1
#define	GPIO_2	2
#define	GPIO_3	3
#define	GPIO_4	4
#define	GPIO_5	5
#define	GPIO_6	6
#define	GPIO_7	7
#define	GPIO_8	8
#define	GPIO_9	9
#define	GPIO_10	10
#define	GPIO_11	11
#define	GPIO_12	12
#define	GPIO_13	13
#define	GPIO_14	14
#define	GPIO_15	15
#define	GPIO_16	16
#define	GPIO_17	17
#define	GPIO_18	18
#define	GPIO_19	19
#define	GPIO_20	20
#define	GPIO_21	21
#define	GPIO_22	22
#define	GPIO_23	23
#define	GPIO_24	24
#define	GPIO_25	25
#define	GPIO_26	26
#define	GPIO_27	27
#define	GPIO_28	28
#define	GPIO_29	29
#define	GPIO_30	30
#define	GPIO_31	31
#define	GPIO_32	32
#define	GPIO_33	33
#define	GPIO_34	34
#define	GPIO_35	35
#define	GPIO_36	36
#define	GPIO_37	37
#define	GPIO_38	38
#define	GPIO_39	39
#define	GPIO_40	40
#define	GPIO_41	41
#define	GPIO_42	42
#define	GPIO_43	43
#define	GPIO_44	44
#define	GPIO_45	45
#define	GPIO_46	46
#define	GPIO_47	47

#define GPIO_DIR_INPUT 1
#define GPIO_DIR_OUTPUT 0

#define GPIO_INPUT_ENABLE 1
#define GPIO_INPUT_DISABLE 0

#define GPIO_POLAR_AH_RE 0
#define GPIO_POLAR_AL_FE 1

#define GPIO_EDGE_LEVEL 0
#define GPIO_EDGE_EDGE 1

#define GPIO_BOTH_SE 0
#define GPIO_BOTH_BE 1

#define PERIPHERAL_USAGE 1
#define GPIO_USAGE 0
 
#ifdef BF533_FAMILY
#define MAX_BLACKFIN_GPIOS 16
#endif

#ifdef BF537_FAMILY
#define MAX_BLACKFIN_GPIOS 48
#define PORT_F 0
#define PORT_G 1
#define PORT_H 2

#define	GPIO_PF0	0 
#define	GPIO_PF1	1 
#define	GPIO_PF2	2 
#define	GPIO_PF3	3 
#define	GPIO_PF4	4 
#define	GPIO_PF5	5 
#define	GPIO_PF6	6 
#define	GPIO_PF7	7 
#define	GPIO_PF8	8 
#define	GPIO_PF9	9 
#define	GPIO_PF10	10
#define	GPIO_PF11	11
#define	GPIO_PF12	12
#define	GPIO_PF13	13
#define	GPIO_PF14	14
#define	GPIO_PF15	15
#define	GPIO_PG0	16
#define	GPIO_PG1	17
#define	GPIO_PG2	18
#define	GPIO_PG3	19
#define	GPIO_PG4	20
#define	GPIO_PG5	21
#define	GPIO_PG6	22
#define	GPIO_PG7	23
#define	GPIO_PG8	24
#define	GPIO_PG9	25
#define	GPIO_PG10      	26
#define	GPIO_PG11      	27
#define	GPIO_PG12      	28
#define	GPIO_PG13      	29
#define	GPIO_PG14      	30
#define	GPIO_PG15      	31
#define	GPIO_PH0	32
#define	GPIO_PH1	33
#define	GPIO_PH2	34
#define	GPIO_PH3	35
#define	GPIO_PH4	36
#define	GPIO_PH5	37
#define	GPIO_PH6	38
#define	GPIO_PH7	39
#define	GPIO_PH8	40
#define	GPIO_PH9	41
#define	GPIO_PH10      	42
#define	GPIO_PH11      	43
#define	GPIO_PH12      	44
#define	GPIO_PH13      	45
#define	GPIO_PH14      	46
#define	GPIO_PH15      	47

#endif

#ifdef BF561_FAMILY
#define MAX_BLACKFIN_GPIOS 48
#define PORT_FIO0 0
#define PORT_FIO1 1
#define PORT_FIO2 2
#endif

#ifndef __ASSEMBLY__

#pragma pack(2)
struct gpio_port_t {
	unsigned short data;
	unsigned short dummy1;
	unsigned short clear;
	unsigned short dummy2;
	unsigned short set;
	unsigned short dummy3;
	unsigned short toggle;
	unsigned short dummy4;
	unsigned short maska;
	unsigned short dummy5;
	unsigned short maska_clear;
	unsigned short dummy6;
	unsigned short maska_set;
	unsigned short dummy7;
	unsigned short maska_toggle;
	unsigned short dummy8;
	unsigned short maskb;
	unsigned short dummy9;
	unsigned short maskb_clear;
	unsigned short dummy10;
	unsigned short maskb_set;
	unsigned short dummy11;
	unsigned short maskb_toggle;
	unsigned short dummy12;
	unsigned short dir;
	unsigned short dummy13;
	unsigned short polar;
	unsigned short dummy14;
	unsigned short edge;
	unsigned short dummy15;
	unsigned short both;
	unsigned short dummy16;
	unsigned short inen;
};
#pragma pack()

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_BLACKFIN_GPIO_H__ */
