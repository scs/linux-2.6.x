/*
 * arch/bfinommu/asm/dpmc.h -  Miscellaneous IOCTL commands for Dynamic Power
 *   			 	Management Controller Driver.	  
 * Copyright (C) 2004 LG Soft India.
 *
 */
#ifndef _BFINNOMMU_DPMC_H_
#define _BFINNOMMU_DPMC_H_

#define SLEEP_MODE		1
#define DEEP_SLEEP_MODE		2
#define ACTIVE_PLLDISABLED	3
#define FULLON_MODE		4
#define ACTIVE_PLLENABLED	5
#define HIBERNATE_MODE		6

#define IOCTL_FULL_ON_MODE	_IO('s', 0xA0)
#define IOCTL_ACTIVE_MODE	_IO('s', 0xA1)
#define IOCTL_SLEEP_MODE	_IO('s', 0xA2)
#define IOCTL_DEEP_SLEEP_MODE	_IO('s', 0xA3)
#define IOCTL_HIBERNATE_MODE	_IO('s', 0xA4)
#define IOCTL_CHANGE_FREQUENCY	_IOW('s', 0xA5, unsigned long)
#define IOCTL_CHANGE_VOLTAGE	_IOW('s', 0xA6, unsigned long)
#define IOCTL_SET_CCLK		_IOW('s', 0xA7, unsigned long)
#define IOCTL_SET_SCLK		_IOW('s', 0xA8, unsigned long)
#define IOCTL_GET_PLLSTATUS	_IOW('s', 0xA9, unsigned long)
#define IOCTL_GET_CORECLOCK	_IOW('s', 0xAA, unsigned long)
#define IOCTL_GET_SYSTEMCLOCK	_IOW('s', 0xAB, unsigned long)
#define IOCTL_GET_VCO		_IOW('s', 0xAC, unsigned long)
#define IOCTL_DISABLE_WDOG_TIMER _IO('s', 0xAD)
#define IOCTL_UNMASK_WDOG_WAKEUP_EVENT _IO('s',0xAE)
#define IOCTL_PROGRAM_WDOG_TIMER _IOW('s',0xAF,unsigned long)
#define IOCTL_CLEAR_WDOG_WAKEUP_EVENT _IO('s',0xB0)


#define DPMC_MINOR		254

#define ON	0
#define OFF	1	

unsigned long calc_volt(void);
int calc_vlev(int vlt);
unsigned long change_voltage(unsigned long volt);
int calc_msel(int vco_hz);
unsigned long change_frequency(unsigned long vco_mhz);
int set_pll_div(unsigned short sel,unsigned char flag);
int get_vco(void);
unsigned long change_system_clock(unsigned long clock);
unsigned long change_core_clock(unsigned long clock);
unsigned long get_pll_status(void);
void change_baud(int baud);      
void fullon_mode(void);
void active_mode(void);
void sleep_mode(void);
void deep_sleep(void);
void hibernate_mode(void);
void program_wdog_timer(unsigned long);
void unmask_wdog_wakeup_evt(void);
void clear_wdog_wakeup_evt(void);
void disable_wdog_timer(void);  

extern unsigned long get_cclk(void);
extern unsigned long get_sclk(void);

#endif	/*_BFINNOMMU_DPMC_H_*/
