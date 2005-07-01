/*
 * $Id$ 
 *
 *BF5xx RTC support
 * 
 */

#include <asm/blackfin.h>
#include <asm/bf5xx_rtc.h>

#define MIN_TO_SECS(_x_) (60 * _x_)
#define HRS_TO_SECS(_x_) (60 * 60 * _x_)
#define DAYS_TO_SECS(_x_) (24 * 60 * 60 * _x_)

#define NUM_SECS_IN_DAY (24 * 3600)
#define NUM_SECS_IN_HOUR (3600)
#define NUM_SECS_IN_MIN (60)

/* Shift values for RTC_STAT register */
#define DAY_BITS_OFF    17
#define HOUR_BITS_OFF   12
#define MIN_BITS_OFF    6
#define SEC_BITS_OFF    0


static void wait_for_complete(void);

/* Initialize the RTC. Enable pre-scaler to scale RTC clock to 1Hz. */
int rtc_init()
{
	*(volatile unsigned short *) RTC_PREN = 0x1;
	wait_for_complete();
	return 0;
}

/* Set the time. time_in_secs is the number of seconds since Jan 1970 */
int rtc_set(time_t time_in_secs)
{
	unsigned long n_days_1970 = 0;
	unsigned long n_secs_rem  = 0;
	unsigned long n_hrs	  = 0;
	unsigned long n_mins	  = 0;
	unsigned long n_secs	  = 0;

	/* Compute no. of days since 1970 */
	n_days_1970 = (unsigned long) (time_in_secs / (NUM_SECS_IN_DAY));

	/* From the remining secs, compute the hrs(0-23), mins(0-59) and secs(0-59) */
	n_secs_rem = (unsigned long)(time_in_secs % (NUM_SECS_IN_DAY));
	n_hrs 	= n_secs_rem / (NUM_SECS_IN_HOUR);
	n_secs_rem = n_secs_rem % (NUM_SECS_IN_HOUR);
	n_mins = n_secs_rem / (NUM_SECS_IN_MIN);
	n_secs = n_secs_rem % (NUM_SECS_IN_MIN);

	/* Store the new time in the RTC_STAT register */
	*(volatile unsigned long *) RTC_STAT =
		((n_days_1970 << DAY_BITS_OFF) | (n_hrs << HOUR_BITS_OFF) |
		(n_mins << MIN_BITS_OFF) | (n_secs << SEC_BITS_OFF));

	wait_for_complete();
	return 0;
}

/* Read the time from the RTC_STAT. time_in_seconds is seconds since Jan 1970 */
int rtc_get(time_t *time_in_seconds)
{
	unsigned long cur_rtc_stat = 0;
	int tm_sec = 0, tm_min = 0, tm_hour = 0, tm_day = 0; 

	if ( time_in_seconds == NULL )
	{
		return -1;
	} 
	
	/* Read the RTC_STAT register */
	cur_rtc_stat = *(volatile unsigned long *) RTC_STAT;
	
	/* Get the secs (0-59), mins (0-59), hrs (0-23) and the days since Jan 1970 */
	tm_sec = (cur_rtc_stat >> SEC_BITS_OFF) & 0x3f;
	tm_min = (cur_rtc_stat >> MIN_BITS_OFF) & 0x3f;
	tm_hour = (cur_rtc_stat >> HOUR_BITS_OFF) & 0x1f;
	tm_day = (cur_rtc_stat >> DAY_BITS_OFF) & 0x7fff;

	/* Calculate the total number of seconds since Jan 1970 */
	*(time_in_seconds) = 	(tm_sec) + 
					MIN_TO_SECS(tm_min) + 
						HRS_TO_SECS(tm_hour) + 
							DAYS_TO_SECS(tm_day);
	return 0;
}

/* Wait for the previous write to a RTC register to complete */
static void wait_for_complete(void)
{
	while (!(*(volatile unsigned short *) RTC_ISTAT & 0x8000)) {
		/*printk("");*/
	}
	*(volatile unsigned short *) RTC_ISTAT = 0x8000;
}
