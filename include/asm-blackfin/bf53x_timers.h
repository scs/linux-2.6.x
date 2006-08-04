/*
 * include/asm/bf533_timers.h
 *
 * This file contains the major Data structures and constants
 * used for General Purpose Timer Implementation in BF533
 *
 * Copyright (C) 2005 John DeHority
 *
*/

#ifndef _BLACKFIN_TIMERS_H_
#define _BLACKFIN_TIMERS_H_


#define SSYNC() __builtin_bfin_ssync()

#ifndef BFIN_TIMER_DEBUG
#define assert(expr) do {} while(0)
#else
#define assert(expr)                        \
    if (!(expr)) {                      \
	    printk("Assertion failed! %s, %s, %s, line=%d \n",  \
		    #expr, __FILE__,__FUNCTION__,__LINE__);         \
	}
#endif


/*-------------------------
 * config reg bits value
 *-------------------------*/

/* used in masks for timer_enable() and timer_disable() */
#define	TIMER0bit	1  /*  0001b */
#define	TIMER1bit	2  /*  0010b */
#define	TIMER2bit	4  /*  0100b */
#define TIMER3bit   8
#define TIMER4bit   0x10
#define TIMER5bit	0x20
#define TIMER6bit	0x40
#define TIMER7bit	0x80

#define TIMER0_id	0
#define TIMER1_id	1
#define TIMER2_id	2

#ifdef CONFIG_BF537

#define MAX_BLACKFIN_GPTIMERS	8
#define TIMER3_id	3
#define TIMER4_id	4
#define TIMER5_id	5
#define TIMER6_id	6
#define TIMER7_id	7
#define	FS1_TIMER_ID	TIMER0_id
#define FS2_TIMER_ID	TIMER1_id
#define FS1_TIMER_BIT	TIMER0bit
#define FS2_TIMER_BIT	TIMER1bit

#else

#define MAX_BLACKFIN_GPTIMERS	4
#define FS1_TIMER_ID	TIMER1_id
#define	FS2_TIMER_ID	TIMER2_id
#define FS1_TIMER_BIT	TIMER1bit
#define FS2_TIMER_BIT	TIMER2bit

#endif

/*
** Timer Configuration Register Bits
*/
#define TIMER_EMU_RUN		0x0200
#define	TIMER_TOGGLE_HI		0x0100
#define	TIMER_CLK_SEL		0x0080
#define TIMER_OUT_DIS		0x0040
#define TIMER_TIN_SEL		0x0020
#define TIMER_IRQ_ENA		0x0010
#define TIMER_PERIOD_CNT	0x0008
#define TIMER_PULSE_HI		0x0004
#define TIMER_MODE			0x0003
#define TIMER_MODE_PWM		0x0001
#define TIMER_MODE_WDTH		0x0002
#define TIMER_MODE_EXT_CLK	0x0003

/*
** Timer Status Register Bits
*/
#define	TIMER_STATUS_TIMIL0	0x0001
#define TIMER_STATUS_TIMIL1	0x0002
#define TIMER_STATUS_TIMIL2	0x0004
#define TIMER_STATUS_INTR	0x0007	/* any timer interrupt */

#define TIMER_STATUS_TOVF0	0x0010	/* timer 0 overflow error */
#define TIMER_STATUS_TOVF1	0x0020
#define TIMER_STATUS_TOVF2	0x0040
#define TIMER_STATUS_OFLOW	0x0070	/* any timer overflow */

/*
** Timer Slave Enable Status : write 1 to clear
*/
#define TIMER_STATUS_TRUN0	0x1000
#define TIMER_STATUS_TRUN1	0x2000
#define TIMER_STATUS_TRUN2	0x4000
#define TIMER_STATUS_TRUN	0x7000



typedef struct {
	short	config;
	short	empty1;
	int	counter;
	int	period;
	int	width;
}GPTIMER_timer_regs;


/*
** starting address  0xFFC0 0600
** BF533 enable at address  0xFFC0 0640
** BF537 enable at address  0xFFC0 0680
*/
typedef struct {
	GPTIMER_timer_regs	a_timer[MAX_BLACKFIN_GPTIMERS];
	short		enable;
	short		empty2;
	short		disable;
	short		empty3;
#ifdef CONFIG_BF537
	int			status;
#else
	short		status;
	short		empty4;
#endif
}GPTIMER_registers;

/*******************************************************************************
*	GP_TIMER API's
*******************************************************************************/

void 	set_gptimer_pwidth		(int timer_id, int width);
int	get_gptimer_pwidth		(int timer_id);
void 	set_gptimer_period		(int timer_id, int period);
int	get_gptimer_period		(int timer_id);
int	get_gptimer_count		(int timer_id);
short	get_gptimer_intr		(int	timer_id);
void	set_gptimer_config		(int timer_id, short config);
short	get_gptimer_config		(int timer_id);
void	set_gptimer_pulse_hi	(int timer_id);
void	clear_gptimer_pulse_hi	(int timer_id);
void	enable_gptimers			(short mask);
void	disable_gptimers		(short mask);
int		get_gptimer_status(void);

#endif
