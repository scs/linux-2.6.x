/*
 * File:         drivers/char/bfin_twi_lcd.h
 * Based on:
 * Author:       Michael Hennerich
 *
 * Created:      Feb. 27th 2006
 * Description:  TWI LCD driver (HD44780) connected to a PCF8574 I2C IO expander
 *
 * Rev:          $Id$
 *
 * Modified:
 *               Copyright 2006 Analog Devices Inc.
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

#define LCD_MINOR          250	/* experiential */
#define LCD_DEVNAME       "twi_lcd"
#define LCD_DRIVER		  "TWI LCD Driver v0.00"

/* HD44780 execution timings [microseconds]
 * as these values differ from spec to spec,
 * we use the worst-case values.
 */

#define T_INIT1 4100		/* first init sequence:  4.1 msec */
#define T_INIT2  150		/* second init sequence: 100 usec */
#define T_EXEC    80		/* normal execution time */
#define T_WRCG   120		/* CG RAM Write */
#define T_CLEAR 2250		/* Clear Display */

#define SIGNAL_RW		0x20
#define SIGNAL_RS		0x10
#define SIGNAL_ENABLE 	0x40
#define SIGNAL_ENABLE2	0x80


/* I2C function headers and globals */

#define	PCF8574_LCD_DRV_NAME		"pcf8574_lcd"
static struct i2c_driver pcf8574_lcd_driver;
static struct i2c_client *pcf8574_lcd_client;

static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { CONFIG_TWI_LCD_SLAVE_ADDR, I2C_CLIENT_END };

static struct i2c_client_address_data addr_data = {
  .normal_i2c = normal_addr,
  .probe = ignore,
  .ignore = ignore,
};

/* LCD Driver function headers and globals */

static int currController = 0x2;
static unsigned int lcd_present = 0;

static void drv_HD_I2C_data (const unsigned char, const char *, const int);
static void drv_HD_I2C_command (const unsigned char, const unsigned char);
static void drv_HD_I2C_byte (const unsigned char, const unsigned char);
static int drv_HD_I2C_load (void);

#define kLCD_Addr       0x80


// Macros

#define BusyCheck()	do { } while (0)

/*
 * Function command codes for io_ctl.
 */
#define LCD_On			1
#define LCD_Off			2
#define LCD_Clear		3
#define LCD_Reset		4
#define LCD_Cursor_Left		5
#define LCD_Cursor_Right	6
#define LCD_Disp_Left		7
#define LCD_Disp_Right		8
#define LCD_Set_Cursor		10
#define LCD_Home		11
#define LCD_Curr_Controller	12
#define LCD_Cursor_Off		14
#define LCD_Cursor_On		15
#define LCD_Set_Cursor_Pos	17
#define LCD_Blink_Off           18
#define LCD_Contr           19

#define CONTROLLER_1	0x1
#define CONTROLLER_2	0x2
#define CONTROLLER_BOTH	0x3
