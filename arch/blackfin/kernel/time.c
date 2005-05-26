/*
 *  linux/arch/bfinnommu/kernel/time.c
 *
 *  Copyright (C) 1991, 1992, 1995  Linus Torvalds
 *  Copyright (C) 2004 LG Soft India. 
 *
 * This file contains the bfin-specific time handling details.
 * Most of the stuff is located in the machine specific files.
 *
 */

#include <linux/module.h>
#include <linux/profile.h> 

#include <asm/blackfin.h>
#include <asm/irq.h>
#include <asm/bf533_rtc.h>

#define	TICK_SIZE (tick_nsec / 1000)	

u64 jiffies_64 = INITIAL_JIFFIES;

EXPORT_SYMBOL(jiffies_64);
	
extern void time_sched_init(irqreturn_t (*timer_routine)(int, void *, struct pt_regs *));
unsigned long gettimeoffset (void);
extern unsigned long wall_jiffies;
extern int setup_irq(unsigned int, struct irqaction *);
inline static void do_leds(void);

/*
 * By setting TSCALE such that TCOUNT counts a binary fraction
 * of microseconds, we can read TCOUNT directly and then with
 * a logical shift trivially calculate how many microseconds 
 * since the last tick, allowing do_gettimeofday() to yield
 * far better time resolution for real time benchmarking.
 */

extern u_long get_cclk(void);

#define TSCALE_SHIFT 2	/* 0.25 microseconds */
#define TSCALE_COUNT ((get_cclk()/1000000) >> TSCALE_SHIFT)
#define CLOCKS_PER_JIFFY (((1000*1000/HZ) << TSCALE_SHIFT) + (1 << TSCALE_SHIFT))

/*
 *  Actual HZ values are affected by the truncation introduced
 *  by the above TSCALE_SHIFT operations..
 *  In time_sched_init(), we'll compute a more accurate value
 *  for tick_nsec and tick_usec using:
 *    tick_nsec = 10^3 * CLOCKS_PER_JIFFY * SCALE / F[cclk in MHz]
 */
unsigned long act_num, act_den;	/* numerator, denominator */
unsigned long act_tick_nsec;
unsigned long act_tick_usec;

#if (defined(CONFIG_STAMP_BOARD_ALIVE_LED) || defined(CONFIG_STAMP_BOARD_IDLE_LED))
void __init init_leds(void)
{
	unsigned int tmp = 0;

	/* config PF2/3/4 as output. */
	tmp = *pFIO_DIR;
	asm("ssync;");
	*pFIO_DIR = tmp | 0x1C;
	asm("ssync;");

	/*	First set led be off */
	tmp = *pFIO_FLAG_D;
	asm("ssync;");
	*pFIO_FLAG_D = tmp | 0x1C;	/* light off */
	asm("ssync;");
}
#else 
inline void  __init init_leds(void) {}
#endif

#if defined(CONFIG_STAMP_BOARD_ALIVE_LED)
inline static void do_leds(void)
{
	static unsigned int count = 50;
	static int	flag = 0;
	unsigned short tmp = 0;

	if( --count==0 ) {
		count = 50;
		flag = ~flag;
	}
	tmp = *pFIO_FLAG_D;
	asm("ssync;");

	if( flag )
		tmp &=~0x10;	/* light on */
	else
		tmp |=0x10;	/* light off */

	*pFIO_FLAG_D = tmp;
	asm("ssync;");

}
#else 
inline static void do_leds(void) {}
#endif

static struct irqaction bfin_timer_irq = {
	.name    = "BFIN Timer Tick",
	.flags   = SA_INTERRUPT
};

void time_sched_init(irqreturn_t (*timer_routine)(int, void *, struct pt_regs *))
{
	act_num = CLOCKS_PER_JIFFY * TSCALE_COUNT;
	act_den = get_cclk() / 1000000;
	/*
	 *  Making the numerator as large as possible increases the
	 *  precision of the calculation.
	 *  Multiplying by 10^3 in two parts avoids overflow.
	 */
	act_tick_nsec = 100 * act_num / act_den;
	act_tick_nsec *= 10;
	act_tick_usec = act_tick_nsec / 1000;

	/* update NTP tick_{n,u}sec value with more accurate values */
	tick_nsec = act_tick_nsec;
	tick_usec = act_tick_usec;

	/* power up the timer, but don't enable it just yet */

	*pTCNTL = 1;
	asm("csync;");

	/* make TCOUNT a binary fraction of microseconds using
	* the TSCALE prescaler counter.
	*/

	*pTSCALE = TSCALE_COUNT - 1;
	asm("csync;");
	*pTCOUNT = *pTPERIOD = CLOCKS_PER_JIFFY - 1;
	asm("csync;");

	/* now enable the timer */
	
	*pTCNTL = 7;
	asm("csync;");

	/* set up the timer irq */
	bfin_timer_irq.handler = timer_routine;
	/* call setup_irq instead of request_irq because request_irq calls kmalloc which has not been initialized yet */
	setup_irq(IRQ_CORETMR, &bfin_timer_irq);
}

unsigned long gettimeoffset (void)
{
	return (CLOCKS_PER_JIFFY - *pTCOUNT) >> TSCALE_SHIFT;
}

static inline int set_rtc_mmss(unsigned long nowtime)
{
    return 0;
}

static inline void do_profile (struct pt_regs * regs)
{
	unsigned long pc;
     
	pc = regs->pc;
	     
	profile_hook(regs);     
     
        if (prof_buffer && current->pid) {
		extern int _stext;
		pc -= (unsigned long) &_stext;
		pc >>= prof_shift;
		if (pc < prof_len)
			++prof_buffer[pc];
		else
		/*
		 * Don't ignore out-of-bounds PC values silently,
		 * put them into the last histogram slot, so if
		 * present, they will show up as a sharp peak.
		 */
			++prof_buffer[prof_len-1];
	}
}

/*
 * timer_interrupt() needs to keep up the real-time clock,
 * as well as call the "do_timer()" routine every clocktick
 */
static irqreturn_t timer_interrupt(int irq, void *dummy, struct pt_regs * regs)
{
	/* last time the cmos clock got updated */
	static long last_rtc_update=0;
	
	write_seqlock(&xtime_lock); 

	do_timer(regs);
	do_leds();
	
	if (!user_mode(regs))
		do_profile(regs);
	/*
	 * If we have an externally synchronized Linux clock, then update
	 * CMOS clock accordingly every ~11 minutes. Set_rtc_mmss() has to be
	 * called as close as possible to 500 ms before the new second starts.
	 */
	
	if ((time_status & STA_UNSYNC) == 0 &&
	    xtime.tv_sec > last_rtc_update + 660 &&
	    (xtime.tv_nsec / 1000) >= 500000 - ((unsigned) TICK_SIZE) / 2 &&
	    (xtime.tv_nsec  / 1000) <= 500000 + ((unsigned) TICK_SIZE) / 2) {
	  if (set_rtc_mmss(xtime.tv_sec) == 0)
	    last_rtc_update = xtime.tv_sec;
	  else
	    last_rtc_update = xtime.tv_sec - 600; /* do it again in 60 s */
	}
	write_sequnlock(&xtime_lock);
	return IRQ_HANDLED;	
}

void time_init(void)
{
	time_t secs_since_1970 = 0;

	/* Initialize the RTC sub-system*/
        rtc_init();
	/* Retrieve calendar time (secs since Jan 1970) */
	rtc_get(&secs_since_1970);

	/* Initialize xtime. From now on, xtime is updated with timer interrupts */
        xtime.tv_sec = secs_since_1970;
	xtime.tv_nsec = 0;
	
	wall_to_monotonic.tv_sec = -xtime.tv_sec; 

	time_sched_init(timer_interrupt);
}

void do_gettimeofday(struct timeval *tv)
{
	unsigned long flags;
	unsigned long lost, seq;
	unsigned long usec, sec;

	do {
		seq = read_seqbegin_irqsave(&xtime_lock, flags);
		usec = gettimeoffset();
		lost = jiffies - wall_jiffies;
		if (lost)
			usec += lost * (1000000 / HZ);
		sec = xtime.tv_sec;
		usec += (xtime.tv_nsec / 1000);
	} while (read_seqretry_irqrestore(&xtime_lock, seq, flags));

	while (usec >= 1000000) {
		usec -= 1000000;
		sec++;
	}

	tv->tv_sec = sec;
	tv->tv_usec = usec;
}

EXPORT_SYMBOL(do_gettimeofday);

int do_settimeofday(struct timespec *tv)
{
	time_t wtm_sec, sec = tv->tv_sec;
	long wtm_nsec, nsec = tv->tv_nsec;

	if ((unsigned long)tv->tv_nsec >= NSEC_PER_SEC)
		return -EINVAL;

	write_seqlock_irq(&xtime_lock);
	/*
	 * This is revolting. We need to set the xtime.tv_usec
	 * correctly. However, the value in this location is
	 * is value at the last tick.
	 * Discover what correction gettimeofday
	 * would have done, and then undo it!
	 */
	nsec -= (gettimeoffset() * 1000);

	wtm_sec  = wall_to_monotonic.tv_sec + (xtime.tv_sec - sec);
	wtm_nsec = wall_to_monotonic.tv_nsec + (xtime.tv_nsec - nsec);

	set_normalized_timespec(&xtime, sec, nsec);
	set_normalized_timespec(&wall_to_monotonic, wtm_sec, wtm_nsec);

	time_adjust = 0;		/* stop active adjtime() */
	time_status |= STA_UNSYNC;
	time_maxerror = NTP_PHASE_LIMIT;
	time_esterror = NTP_PHASE_LIMIT;

	write_sequnlock_irq(&xtime_lock);
	clock_was_set();

	/*
	 *  rtc_set() busy-waits up to a second (the next tick of the RTC)
	 *  for completion of the write.
	 *  We release xtime_lock before updating the RTC so as not to
	 *  lock out the timer_interrupt() routine which also acquires
	 *  xtime_lock.  Locking out timer_interrupt() loses ticks!
	 */
	rtc_set(sec);
	
	return 0;
}
/*
 * Scheduler clock - returns current time in nanosec units.
 */
unsigned long long sched_clock(void)
{
	return (unsigned long long)jiffies * (1000000000 / HZ);
}

EXPORT_SYMBOL(do_settimeofday);
