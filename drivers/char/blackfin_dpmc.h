#define DPMC_VERSION "0.1"

static loff_t dpmc_llseek(struct file *file, loff_t offset, int origin);
static ssize_t dpmc_read(struct file *file, char *buf, size_t count, loff_t *ppos);
static int dpmc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int dpmc_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data);

void fullon_mode(void);
void active_mode(void);
void sleep_mode(void);
void deep_sleep(void);
void hibernate_mode(void);
void program_wdog_timer(unsigned long);
void unmask_wdog_wakeup_evt(void);
void clear_wdog_wakeup_evt(void);
void disable_wdog_timer(void);  


#define SDRAM_Tref  	64       /* Refresh period in milliseconds   */
#ifdef CONFIG_EZKIT
	#define SDRAM_NRA   	4096     /* Number of row addresses in SDRAM */
#endif
#ifdef CONFIG_BLKFIN_STAMP
	#define SDRAM_NRA   	4096     /* Number of row addresses in SDRAM, should it be 8192? */
#endif
#define SDRAM_CL	2

#define FLAG_CSEL	0x0
#define FLAG_SSEL	0x1

#define MAX_VCO		600
#define MIN_VCO		50

#define MAX_SCLK	132

#ifdef CONFIG_EZKIT
#define MIN_SCLK	27
#endif
#ifdef CONFIG_BLKFIN_STAMP
#define MIN_SCLK	27 /* For now lets keep it at 22, actually it shld be 11 */
#endif

#define INTER_FREQ	MIN_SCLK*5
#define MAX_SSEL	15
#define DEF_SSEL	5

#define MIN_VOLT	850
#define MAX_VOLT	1300

/* ON some boards voltage regulated bit does not get set, ideally it should be 0xA0 but for now, retain 0x20 */
#define PLL_LOCKED	0x20
#define VOLTAGE_REGULATED	0x0080

#define MHZ		1000000

#ifdef CONFIG_BAUD_9600
#define CONSOLE_BAUD_RATE 	9600
#elif CONFIG_BAUD_19200		
#define CONSOLE_BAUD_RATE 	19200
#elif CONFIG_BAUD_38400		
#define CONSOLE_BAUD_RATE 	38400
#elif CONFIG_BAUD_57600
#define CONSOLE_BAUD_RATE 	57600
#elif CONFIG_BAUD_115200	
#define CONSOLE_BAUD_RATE 	115200	
#endif
