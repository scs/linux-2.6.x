


/*
 * File:         drivers/media/video/blackfin/mt9v032_oscar.c
 * Based on: 	 drivers/media/video/blackfin/mt9m001.c by Michael Benjamin
 *
 * Created:
 * Description:  Driver for the mt9v032 sensor used by the OSCAR framework
 *
 * Rev:          
 *
 * Modified:
 *               Copyright 2004-2006 Analog Devices Inc.
 *               Modified Dec 2007 Supercomputing Systems AG
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

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/videodev2.h>

#include <media/v4l2-dev.h>
#include "mt9v032_oscar.h"

#include <asm/blackfin.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>
#include <asm/uaccess.h>

#define INTERRUPT_MASK 0x14
#define DISABLE_PPI

#define  VID_HARDWARE_OSCCAM  13	/* experimental */
#define  I2C_DRIVERID_OSCCAM  81	/* experimental (next avail. in i2c-id.h) */
#define  NO_TRIGGER  16

# define POL_S              	0x0000 /*0=FS1 on rising edge*/
# define POL_C              	0x0000 /*0=samples data on rising edge*/
# define MAX_PIXEL_PER_LINE     752
# define MAX_LINES_PER_FRAME    480
# define CFG_GP_Input_3Syncs	0x0020
# define GP_Input_Mode      	0x000C
# define PPI_DATA_LEN       	DLEN_8
# define PPI_PACKING        	PACK_EN
# define DMA_FLOW_MODE      	0x0000	/* STOPMODE */
# define DMA_WDSIZE_16      	WDSIZE_16
# define RESTART                0x0020 /* Sync mode. Wait with interrupt until last byte written.*/
#define I2C_SENSOR_ID  (0x5C << 1)
#define MAX_FRAME_WIDTH  752
#define MAX_FRAME_HEIGHT 480

/* These are the values from the datasheets, but have not yet ben tested. */
#define MIN_FRAME_WIDTH    1
#define MIN_FRAME_HEIGHT   1

#define ROW_OFF_MIN 	4     // The minimum value for the row offset (limitation of mt9v032)
#define COL_OFF_MIN 	1     // The minimum value of the start column (limitation of mt9v032)

#define MIN_ROW_TIME      660 // The minimum number of pixels in a row (Row pixels + Horiz. Blanking)
#define MIN_HORIZ_BLANKING 43 // The minimum number of horizontal blanking pixels

#define SENSOR_NAME "MT9V032 CMOS-Sensor"

/*#define DEBUG*/
#ifndef pr_debug
#ifdef DEBUG
#define pr_debug(fmt,arg...)			\
	printk(KERN_DEBUG fmt,##arg)
#else
#define pr_debug(fmt,arg...)			\
	do { } while (0)
#endif
#endif /* pr_debug */

static int mt9v032_oscar_attach_adapter(struct i2c_adapter *adapter);
static int mt9v032_oscar_detach_client(struct i2c_client *client);
static int mt9v032_oscar_command(struct i2c_client *client, unsigned int cmd, void *arg);
static int oscCam_open(struct inode *inode, struct file *filp);
static inline int oscCam_i2c_write(struct i2c_client *client,
				 unsigned char offset, unsigned short data);

static irqreturn_t ppifcd_irq_error(int irq, void *dev_id);

struct i2c_registers {
	int regnum;
	char name[50];
};

/* The register names had to be censored, since the datasheet for the mt9v032
   containing the register description sadly is only available under NDA from 
   Micron at the time. You are welcome to fill the names back in if it becomes
   available publicly. */
struct i2c_registers i2c_regs[] = {
	{0x00, ""},
	{0x01, ""},
	{0x02, ""},
	{0x03, ""},
	{0x04, ""},
	{0x05, ""},
	{0x06, ""},
	{0x07, ""},
	{0x08, ""},
	{0x09, ""},
	{0x0A, ""},
	{0x0B, ""},
	{0x0C, ""},
	{0x0D, ""},
	{0x0E, ""},
	{0x0F, ""},
	{0x1B, ""},
	{0x1C, ""},
      {0x20, ""},
	{0x2C, ""},
	{0x31, ""},
	{0x32, ""},
	{0x33, ""},
	{0x34, ""},
	{0x35, ""},
	{0x36, ""},
	{0x42, ""},
	{0x46, ""},
	{0x47, ""},
	{0x48, ""},
	{0x4C, ""},
	{0x70, ""},
	{0x72, ""},
	{0x73, ""},
	{0x74, ""},
	{0x7F, ""},
/* 	{0x80, ""}, */
/* 	{0x81, ""}, */
/* 	{0x82, ""}, */
/* 	{0x83, ""}, */
/* 	{0x84, ""}, */
/* 	{0x85, ""}, */
/* 	{0x86, ""}, */
/* 	{0x87, ""}, */
/* 	{0x88, ""}, */
/* 	{0x89, ""}, */
/* 	{0x8A, ""}, */
/* 	{0x8B, ""}, */
/* 	{0x8C, ""}, */
/* 	{0x8D, ""}, */
/* 	{0x8E, ""}, */
/* 	{0x8F, ""}, */
/* 	{0x90, ""}, */
/* 	{0x91, ""}, */
/* 	{0x92, ""}, */
/* 	{0x93, ""}, */
/* 	{0x94, ""}, */
/* 	{0x95, ""}, */
/* 	{0x96, ""}, */
/* 	{0x97, ""}, */
/* 	{0x98, ""}, */
/* 	{0x99, ""}, */
/* 	{0x9A, ""}, */
/* 	{0x9B, ""}, */
/* 	{0x9C, ""}, */
/* 	{0x9D, ""}, */
/* 	{0x9E, ""}, */
/* 	{0x9F, ""}, */
/* 	{0xA0, ""}, */
/* 	{0xA1, ""}, */
/* 	{0xA2, ""}, */
/* 	{0xA3, ""}, */
/* 	{0xA4, ""}, */
	{0xA5, ""},
	{0xA6, ""},
	{0xA8, ""},
	{0xA9, ""},
	{0xAB, ""},
	{0xAF, ""},
	{0xB0, ""},
/* 	{0xB1, ""}, */
/* 	{0xB2, ""}, */
/* 	{0xB3, ""}, */
	{0xB4, ""},
/* 	{0xB5, ""}, */
/* 	{0xB6, ""}, */
/* 	{0xB7, ""}, */
/* 	{0xB8, ""}, */
/* 	{0xB9, ""}, */
	{0xBA, ""},
	{0xBB, ""},
	{0xBC, ""},
	{0xBD, ""},
	{0xBE, ""},
	{0xBF, ""},
	{0xC0, ""},
	{0xC1, ""},
	{0xC2, ""},
	{0xC3, ""},
	{0xC4, ""},
	{0xC5, ""},
	{0xF0, ""},
	{0xFE, ""},
	{0xFF, ""},
};

struct oscCam_device_t;

#define  MAX_BUFFER_SIZE 752 * 480

static unsigned int global_gain = 127;
static unsigned int debug = 0;

struct i2c_client *i2c_global_client = NULL;	/* just needed for proc ... */
struct mt9v032_oscar_data {
	struct i2c_client client;
	struct oscCam_device_t *oscCam_dev;
	unsigned char reg[128];
	int input;
	int enable;
	int bright;
	int contrast;
	int hue;
	int sat;
};
static const char sensor_name[] = SENSOR_NAME;
struct i2c_adapter *adapter;
static u16 normal_i2c[] = {
	I2C_SENSOR_ID >> 1,
	(I2C_SENSOR_ID >> 1) + 1,
	I2C_CLIENT_END
};

I2C_CLIENT_INSMOD;
static struct i2c_driver mt9v032_oscar_driver;

struct ppi_device_t {
	struct oscCam_device_t *oscCam_dev;
	int opened;
	int nonblock;
	int cancelled;
	int trigger_mode;
	unsigned short irqnum;
	volatile unsigned short done;
	unsigned short dma_config;
	unsigned short pixel_per_line;
	unsigned short lines_per_frame;
	unsigned short bpp;
	unsigned short ppi_control;
	unsigned short ppi_status;
	unsigned short ppi_delay;
	unsigned short ppi_trigger_gpio;
	struct fasync_struct *fasyc;
	wait_queue_head_t *rx_avail;
};

struct oscCam_buffer {
	unsigned char *data;
	unsigned int size;
	wait_queue_head_t wq;
	volatile int state;
	int flags;

	volatile unsigned long long capture_done; // The cycle count at the end of the capture
};

/*
 * States for each frame buffer.
 */
enum {
	FRAME_UNUSED = 0,	/* < Unused                           */
	FRAME_READY = 1,	/* < Ready to start grabbing          */
	FRAME_GRABBING = 2,	/* < Grabbing the frame               */
	FRAME_DONE = 3,		/* < Grabbing done, frame not synced  */
	FRAME_ERROR = 4,	/* < Error                            */
};

struct oscCam_device_t {
	struct video_device *videodev;
	struct ppi_device_t *ppidev;
	struct i2c_client *client;
	unsigned int frame_count;
	unsigned int error_count;
	int user;
	char name[32];
	struct capture_window capt_win;
	/*  int colOffset;
	    int rowOffset;
	    int width;
	    int height;*/
	size_t size;
	struct timeval *stv;
	struct timeval *etv;
	struct oscCam_buffer buffer[MAX_NR_FRAME_BUFFERS];
	struct oscCam_buffer *next_buf;
	struct oscCam_buffer *dma_buf;
	struct oscCam_buffer *ready_buf;
	spinlock_t lock;
	struct image_info last_image;
};
static struct video_device oscCam_template;
struct oscCam_device_t *oscCam_dev;
static DECLARE_WAIT_QUEUE_HEAD(oscCam_waitqueue);

static int oscCam_abortCapture(int arg);
static void oscCam_workItemResetCam(void *);

// We declare this workitem to reset the camera over I2C after an IRQ error
static DECLARE_WORK(oscCam_resetCamWork, (work_func_t)oscCam_workItemResetCam);

// Helper function to determine the current cycle count of the blackfin
// processor, which can be used to measure time. Returns a 64 bit
// number, which should not wrap in over a thousand years.
static inline unsigned long long get_current_cycles(void)
{
	unsigned long ret_low, ret_high;
	__asm__ __volatile__(
		"%0 = CYCLES;"
		"%1 = CYCLES2;"
		:"=d"(ret_low), "=d" (ret_high)
		);
	return ((unsigned long long)ret_high << 32) + ret_low;
}

static void oscCam_regReset(struct ppi_device_t *pdev)
{
	pr_debug("%s\n", __FUNCTION__);

	bfin_clear_PPI_STATUS();
	bfin_write_PPI_CONTROL(pdev->ppi_control & ~PORT_EN);
	bfin_write_PPI_DELAY(pdev->ppi_delay);
	bfin_write_PPI_COUNT(pdev->pixel_per_line - 1);
	bfin_write_PPI_FRAME(pdev->lines_per_frame);
}

// Configures the DMA to write <count> byters into the DMA <buf> and starts
// both the DMA and PPI.
// Depending on whether nonblock has been set in the <ppidev> it then waits
// until the data has arrived and wakes up any waiting processes, or just exits
static size_t oscCam_ppi2dma(struct ppi_device_t *ppidev, char *buf, size_t count)
{
	int ierr;

	if (count <= 0)
		return 0;

	pr_debug("%s: reading %zi bytes (%dx%d) into [0x%p]\n",
		 __FUNCTION__,
		 count, 
		 ppidev->oscCam_dev->capt_win.width, 
		 ppidev->oscCam_dev->capt_win.height, 
		 buf);
	

	set_dma_start_addr(CH_PPI, (u_long) buf);

	// Disable interrupts
	//bfin_write_SIC_IMASK(bfin_read_SIC_IMASK() & ~INTERRUPT_MASK);
	//disable_irq(IRQ_PPI_ERROR);

	// Enable DMA
	enable_dma(CH_PPI);
	// Enable PPI  
	bfin_write_PPI_CONTROL(ppidev->ppi_control | PORT_EN);

	// Acknowledge any old interrupt requests that were queued while
	// the DMA was off. 
	clear_dma_irqstat(CH_PPI);
	bfin_clear_PPI_STATUS();

	// Enable interrupts
	bfin_write_SIC_IMASK(bfin_read_SIC_IMASK() | INTERRUPT_MASK); 
	enable_irq(IRQ_PPI_ERROR);


	if (ppidev->nonblock) {
		pr_debug("%s: awaiting IRQ...\n", __FUNCTION__);
		return -EAGAIN;
	} else {
		pr_debug("%s: PPI wait_event_interruptible\n", __FUNCTION__);

		ierr = wait_event_interruptible(*(ppidev->rx_avail), ppidev->done);
		if (ierr) {
			pr_debug("%s: IErr: PPI_STATUS = 0x%x\tPPI_FRAME = 0x%x\n", 
				 __FUNCTION__,
				 bfin_read_PPI_STATUS(),
				 bfin_read_PPI_FRAME());

			/* waiting is broken by a signal */
			return ierr;
		}
		pr_debug("%s: PPI wait_event_interruptible done\n", __FUNCTION__);
	}

	/* Disable PPI */
	disable_dma(CH_PPI);
#ifdef DISABLE_PPI
	bfin_write_PPI_CONTROL(bfin_read_PPI_CONTROL() & ~PORT_EN);
#endif

	pr_debug("%s: done read in %zi bytes for [0x%p-0x%p]\n", 
		 __FUNCTION__,
		 count,
		 buf, 
		 (buf + count));

	return count;
}

// The interrupt routine invoked after a successful PPI transfer.
// It disables DMA, marks the target frame buffer as updated
// and wakes up any waiting processes
static irqreturn_t ppifcd_irq(int irq, void *dev_id)
{
	struct oscCam_device_t *oscCam_dev;
	struct ppi_device_t *pdev = (struct ppi_device_t *)dev_id;
	oscCam_dev = pdev->oscCam_dev;

	BUG_ON(dev_id == NULL);
	BUG_ON(oscCam_dev == NULL);
  
	// Disable interrupts
	bfin_write_SIC_IMASK(bfin_read_SIC_IMASK() & ~INTERRUPT_MASK);
	disable_irq(IRQ_PPI_ERROR);

	/*  Acknowledge DMA Interrupt  */
	clear_dma_irqstat(CH_PPI);
	bfin_clear_PPI_STATUS();

	/* Disable PPI */
	disable_dma(CH_PPI);
#ifdef DISABLE_PPI
	bfin_write_PPI_CONTROL(bfin_read_PPI_CONTROL() & ~PORT_EN);
#endif

	if(oscCam_dev->dma_buf == NULL || pdev->done == 1) {
		// This was an image we were not prepared for...
		printk(KERN_WARNING "%s: PPI complete interrupt but no transfer scheduled..\n",
		       __FUNCTION__);
		return IRQ_HANDLED;
	}
	
	oscCam_dev->dma_buf->state = FRAME_DONE;

	// Timestamp the end of the capture
	oscCam_dev->dma_buf->capture_done = get_current_cycles();

	pr_debug("-->%s: pdev->done=%d (%ld ms/%llu cyc)\n", 
		 __FUNCTION__,
		 pdev->done, 
		 jiffies * 1000 / HZ, 
		 oscCam_dev->dma_buf->capture_done);


	// Wake up any waiting processes...
	wake_up_interruptible(pdev->rx_avail);
	wake_up_interruptible(&oscCam_dev->dma_buf->wq);

	// We just finished capturing this frame so we update the pointer to the last image
	oscCam_dev->last_image.fbuf = oscCam_dev->dma_buf->data;
	memcpy(&oscCam_dev->last_image.window, &oscCam_dev->capt_win, sizeof(struct capture_window));  

	if(oscCam_dev->dma_buf->flags & FB_FLAG_CACHED) {
		// This frame buffer is in a cached memory region and therefore
		// we must invalidate the cache since the data was transferred
		// by the DMA and not by the CPU
		blackfin_dcache_invalidate_range((u_long) oscCam_dev->dma_buf->data,
						 (u_long)(oscCam_dev->dma_buf->data + oscCam_dev->size));
	}

	// Update the buffer pointers
	oscCam_dev->ready_buf = oscCam_dev->dma_buf;
	oscCam_dev->dma_buf = NULL;
	oscCam_dev->next_buf = NULL;

	// Mark as finished....
	pdev->done = 1;

	return IRQ_HANDLED;
}

// The interrupt routine invoked after an error during a transfer
// It aborts the capture and thus sets the properties of the
// involved frame buffer to an error state.
static irqreturn_t ppifcd_irq_error(int irq, void *dev_id)
{
	struct oscCam_device_t *oscCam_dev;
	struct ppi_device_t *pdev = (struct ppi_device_t *)dev_id;
	oscCam_dev = pdev->oscCam_dev;

	BUG_ON(dev_id == NULL);
	BUG_ON(oscCam_dev == NULL);

	printk(KERN_ERR "%s: PPI Status 0x%x\n",
		 __FUNCTION__,
		 bfin_read_PPI_STATUS());

	bfin_clear_PPI_STATUS();

	// Abort the capture process
	// This will take care of everything else
	oscCam_abortCapture(0);

	return IRQ_HANDLED;
}

static int ppi_fasync(int fd, struct file *filp, int on)
{
	struct ppi_device_t *pdev = oscCam_dev->ppidev;
	return fasync_helper(fd, filp, on, &(pdev->fasyc));
}

// Read a data from the CMOS sensor
static inline int oscCam_i2c_read(struct i2c_client *client, 
				unsigned char offset,
				u16 * data, unsigned int len)	
{
	u8 buf[2];
	int ret;

	BUG_ON(client == NULL);

	ret = i2c_smbus_write_byte(client, offset);
	if(ret < 0) {
		pr_debug("%s: Failed to send read command!\n", 
			 __FUNCTION__);
		return ret;
	}
	ret = i2c_master_recv(client, buf, 2);
	if(ret < 0) {
		pr_debug("%s: Failed to read register!\n",
			 __FUNCTION__);
		return ret;
	}

	*data = buf[0] << 8 | buf[1];

	return 0;
}

// Write data to the CMOS sensor over I2C
// The register is not written until after this function returns
// since it only schedules a write transfer (non-blocking).
static inline int oscCam_i2c_write(struct i2c_client *client,
				 unsigned char offset, 
				 unsigned short data)
{
	u8 buf[3];
	int ret;

	BUG_ON(client == NULL);

	buf[0] = offset;
	buf[1] = data >> 8;
	buf[2] = data & 0xFF;

	ret = i2c_master_send(client, buf, 3);
	if(ret < 0) {
		pr_debug("%s: Writing to CMOS sensor failed!\n",
			 __FUNCTION__);
		return ret;
	} 
	return 0; 
}

static struct i2c_driver mt9v032_oscar_driver = {
	.driver = {
		.name = SENSOR_NAME,
	},
	.id = I2C_DRIVERID_OSCCAM,
	.attach_adapter = mt9v032_oscar_attach_adapter,
	.detach_client = mt9v032_oscar_detach_client,
	.command = mt9v032_oscar_command,
};


static int mt9v032_oscar_init_v4l(struct mt9v032_oscar_data *data)
{
	int err, i;

	pr_debug("Registering oscCam device\n");

	err = -ENOMEM;
	oscCam_dev = kmalloc(sizeof(struct oscCam_device_t), GFP_KERNEL);
	if (oscCam_dev == NULL)
		goto error_out;
	oscCam_dev->ppidev = kmalloc(sizeof(struct ppi_device_t), GFP_KERNEL);
	if (oscCam_dev->ppidev == NULL)
		goto error_out_dev;
	oscCam_dev->videodev = kmalloc(sizeof(struct video_device), GFP_KERNEL);
	if (oscCam_dev->videodev == NULL)
		goto error_out_ppi;

	pr_debug("%s: Configuring PPIFCD\n", __FUNCTION__);
	oscCam_regReset(oscCam_dev->ppidev);
	oscCam_dev->ppidev->opened = 0;
	oscCam_dev->ppidev->cancelled = 0;
	oscCam_dev->ppidev->oscCam_dev = oscCam_dev;

	pr_debug("%s Configuring Video4Linux driver\n", __FUNCTION__);
	for (i = 0; i < MAX_NR_FRAME_BUFFERS; i++)
		init_waitqueue_head(&oscCam_dev->buffer[i].wq);

	memcpy(oscCam_dev->videodev, &oscCam_template, sizeof(oscCam_template));
	oscCam_dev->frame_count = 0;
	oscCam_dev->error_count = 0;
	err = video_register_device(oscCam_dev->videodev, VFL_TYPE_GRABBER, -1);
	if (err) {
		printk(KERN_NOTICE
		       "%s: Unable to register Video4Linux driver for %s\n",
		       __FUNCTION__,
		       oscCam_dev->videodev->name);
		goto error_out_video;
	}

	oscCam_dev->client = &data->client;
	oscCam_dev->lock = SPIN_LOCK_UNLOCKED;
	oscCam_dev->user = 0;

	data->oscCam_dev = oscCam_dev;

	printk(KERN_INFO "%s: %s: V4L driver %s now ready\n",
	       __FUNCTION__,
	       sensor_name,
	       oscCam_dev->videodev->name);

	return 0;

error_out_video:
	kfree(oscCam_dev->videodev);
error_out_ppi:
	kfree(oscCam_dev->ppidev);
error_out_dev:
	kfree(oscCam_dev);
error_out:
	return err;
}

static int mt9v032_oscar_detect_client(struct i2c_adapter *adapter, int address,
				 int kind)
{
	int err;
	struct i2c_client *new_client;
	struct mt9v032_oscar_data *data;
	u16 tmp = 0;

	printk(KERN_INFO "%s: %s: detecting client on address 0x%x\n", 
	       __FUNCTION__,
	       sensor_name,
	       address << 1);

	if (address != normal_i2c[0] && address != normal_i2c[1])
		return -ENODEV;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return 0;

	data = kzalloc(sizeof(struct mt9v032_oscar_data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;
	data->input = 0;
	data->enable = 1;

	i2c_global_client = new_client = &data->client;
	i2c_set_clientdata(new_client, data);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &mt9v032_oscar_driver;
	strcpy(new_client->name, sensor_name);

	err = i2c_attach_client(new_client);
	if (err)
		goto error_out;

	oscCam_i2c_read(new_client, 0x00, &tmp, 1);
	pr_debug("%s: %s: detected I2C client (id = %04x)\n", 
		 __FUNCTION__,
		 sensor_name, 
		 tmp);

	err = mt9v032_oscar_init_v4l(data);

	if (err)
		goto error_out;

	return 0;

error_out:
	kfree(data);
	printk(KERN_ERR "%s: %s: init error 0x%x\n", 
	       __FUNCTION__,
	       sensor_name, 
	       err);
	return err;
}

static int mt9v032_oscar_attach_adapter(struct i2c_adapter *adapter)
{
	int i;
	BUG_ON(adapter == NULL);

	pr_debug("%s: %s: starting probe for adapter %s (0x%x)\n", 
		 __FUNCTION__,
		 sensor_name,
		 adapter->name, adapter->id);

	i = i2c_probe(adapter, &addr_data, &mt9v032_oscar_detect_client);
	return i;
}

static int mt9v032_oscar_detach_client(struct i2c_client *client)
{
	struct mt9v032_oscar_data *data;
	int err;

	if ((err = i2c_detach_client(client)))
		return err;

	data = i2c_get_clientdata(client);

	video_unregister_device(data->oscCam_dev->videodev);
	kfree(data->oscCam_dev->videodev);
	kfree(data->oscCam_dev->ppidev);
	kfree(data->oscCam_dev);

	kfree(data);

	return 0;
}

static int mt9v032_oscar_command(struct i2c_client *client, unsigned int cmd,
			   void *arg)
{
	/* as yet unimplemented */
	return -EINVAL;
}


/*
 * FIXME: We should not be putting random things into proc - this is a NO-NO
 */
#if 1	//oscCam_i2c_write(new_client, 0x0F, 0x0015);


static int mt9v032_oscar_proc_read(char *buf, char **start, off_t offset, int count,
			     int *eof, void *data)
{
	/* one page size should be less than 'count' bytes. Otherwise you have */
	/* to play with the '*start' value to read in more than one page */
	int i;
	u16 tmp = 0;
	int len = 0;

	for (i = 0; i < sizeof(i2c_regs) / sizeof(*i2c_regs); ++i) {
		oscCam_i2c_read(i2c_global_client, i2c_regs[i].regnum, &tmp, 1);
		len += sprintf(&buf[len], "Reg 0x%02x: = 0x%04x %-40s",
			       i2c_regs[i].regnum, tmp, i2c_regs[i].name);
		if (!((i + 1) % 2))
			len += sprintf(&buf[len], "\n");
	}
	
	len += sprintf(&buf[len], "\n");

	*eof = 1;
	return len;
}

static int mt9v032_oscar_proc_write(struct file *file, const char *buf,
			      unsigned long count, void *data)
{
	int reg = 0, val = 0, i;
	sscanf(buf, "%i %i", &reg, &val);
	for (i = 0; i < sizeof(i2c_regs) / sizeof(*i2c_regs); ++i) {
		if (i2c_regs[i].regnum == reg) {
			printk(KERN_INFO "writing register 0x%02x (%s) with 0x%04x\n",
			       reg, i2c_regs[i].name, val);
			oscCam_i2c_write(i2c_global_client, reg, val);
			break;
		}
	}
	return count;
}

static int mt9v032_oscar_proc_init(void)
{
	struct proc_dir_entry *ptr;
	pr_debug("%s: Configuring proc\n", __FUNCTION__);
	ptr = create_proc_entry("oscCam", S_IFREG | S_IRUGO, NULL);
	if (!ptr)
		return -1;
	ptr->owner = THIS_MODULE;
	ptr->read_proc = mt9v032_oscar_proc_read;
	ptr->write_proc = mt9v032_oscar_proc_write;
	return 0;
}
#endif				/* DEBUG PROC FILE */

static void v4l_release(struct video_device *vdev)
{
	kfree(vdev);
}

// Tests if a capture is currently set up and thus the device
// is busy
inline static bool oscCam_IsBusy(void)
{
	// This function "atomically" tests whether an image capture is 
	// currently in progress on this device
	bool busy = true;

	spin_lock(oscCam_dev->lock);         // >>>>>>>>>>>
	// We need an atomic check here to ensure the device is not "in use"
	// The interrupt routine can still disturb us, but only sets done to 0
	// so we don't have a problem
	if (oscCam_dev->ppidev->done){
		busy = false;
		oscCam_dev->ppidev->done = 0;
	} else {
		busy = true;
	}
	spin_unlock(oscCam_dev->lock);       // <<<<<<<<<<<

	return busy;
}

// Configures the capture window specified by xOff, yOff, width and
// height into the CMOS sensor.
// Returns 0 on success.
static int oscCam_configWindow(int xOff, int yOff, int width, int height)
{
	int horiz_blanking;

	// Write the window to the camera
	oscCam_i2c_write(oscCam_dev->client, 
		       0x01, // Column start
		       xOff);
  
	oscCam_i2c_write(oscCam_dev->client,
		       0x02, // Row start
		       yOff);
  
	oscCam_i2c_write(oscCam_dev->client,
		       0x03, // Window height
		       height);
  
	oscCam_i2c_write(oscCam_dev->client,
		       0x04, // Window width
		       width);
  
	// Depending on the width of the window, 
	// we need to increase the horizontal blanking 
	// to adhere to the minimum row time 
	if(width > MIN_ROW_TIME - MIN_HORIZ_BLANKING)
		horiz_blanking = MIN_HORIZ_BLANKING;
	else
		horiz_blanking = MIN_ROW_TIME - width;
  
	oscCam_i2c_write(oscCam_dev->client,
		       0x05, // Horizontal blanking
		       horiz_blanking);				
	return 0;
}


static int oscCam_open(struct inode *inode, struct file *filp)
{
        int i;
	pr_debug("%s\n", __FUNCTION__);

	try_module_get(THIS_MODULE);

	if (!oscCam_dev) {
		printk("%s:  ...specified video device not found!\n",
			__FUNCTION__);
		return -ENODEV;
	}

	/* FIXME: use a proper mutex here */
	if (oscCam_dev->user || oscCam_dev->ppidev->opened) {
		printk("%s: PPI opened already (%d users)\n", 
		       __FUNCTION__,
		       oscCam_dev->user);
		return -EMFILE;
	}

	/* set oscCam_dev properties */
	oscCam_dev->capt_win.width = MAX_PIXEL_PER_LINE;
	oscCam_dev->capt_win.height = MAX_LINES_PER_FRAME;	
	oscCam_dev->capt_win.row_off = ROW_OFF_MIN;
	oscCam_dev->capt_win.col_off = COL_OFF_MIN;
	oscCam_dev->size = oscCam_dev->capt_win.width * oscCam_dev->capt_win.height;

	oscCam_dev->frame_count = 0;
	oscCam_dev->error_count = 0;
	
	oscCam_configWindow(oscCam_dev->capt_win.col_off, 
			  oscCam_dev->capt_win.row_off, 
			  oscCam_dev->capt_win.width, 
			  oscCam_dev->capt_win.height);
	
	oscCam_dev->dma_buf = NULL;
	oscCam_dev->ready_buf = NULL;
	oscCam_dev->next_buf = NULL;

	for(i = 0; i < MAX_NR_FRAME_BUFFERS; i++) {
		oscCam_dev->buffer[i].state = FRAME_UNUSED;
	}

	memset(oscCam_dev->ppidev, 0, sizeof(struct ppi_device_t));
	memset(&oscCam_dev->last_image, 0, sizeof(struct image_info));
	
 	/* Set CMOS Sensor Parameters: see function */
	
	/* Set PPI Parameters */
	oscCam_dev->ppidev->opened 	= 1;
	oscCam_dev->ppidev->cancelled     = 0;
	pr_debug("%s: oscCam open setting PPI done\n", __FUNCTION__);
	oscCam_dev->ppidev->done 		 = 1;	/* initially ppi is "done" */
	oscCam_dev->ppidev->pixel_per_line = oscCam_dev->capt_win.width;
	oscCam_dev->ppidev->lines_per_frame = oscCam_dev->capt_win.height;
	oscCam_dev->ppidev->bpp 		 = 8;
	oscCam_dev->ppidev->ppi_control	 = 0;
	oscCam_dev->ppidev->ppi_control	|= (POL_S & POL_C);
	//oscCam_dev->ppidev->ppi_control	|= 0x0800; //10bit, 8bit=PPI_DATA_LEN;
	oscCam_dev->ppidev->ppi_control	|= 0x0000; //8bit, 8bit=PPI_DATA_LEN;
	oscCam_dev->ppidev->ppi_control   |= PPI_PACKING; /*PACK_EN only for 8 bpp*/
	oscCam_dev->ppidev->ppi_control 	|= CFG_GP_Input_3Syncs;
	oscCam_dev->ppidev->ppi_control 	|= GP_Input_Mode;

	oscCam_dev->ppidev->ppi_status 	= 0;
	oscCam_dev->ppidev->ppi_delay 	= 0;
	oscCam_dev->ppidev->ppi_trigger_gpio 	= NO_TRIGGER;
	oscCam_dev->ppidev->rx_avail 	= &oscCam_waitqueue;
	oscCam_dev->ppidev->irqnum 	= IRQ_PPI;
	oscCam_dev->ppidev->nonblock 	= 0;   // 0= will wait until done during frame capture
	oscCam_dev->ppidev->oscCam_dev 	= oscCam_dev;
	
	// Set Buffers
	oscCam_dev->dma_buf = NULL;
	oscCam_dev->next_buf = NULL;
	oscCam_dev->ready_buf = NULL;

	/* Set DMA Parameters */
	oscCam_dev->ppidev->dma_config  = 0;
	oscCam_dev->ppidev->dma_config |= DMA_FLOW_MODE;
	oscCam_dev->ppidev->dma_config |= WNR;	
	oscCam_dev->ppidev->dma_config |= RESTART;	
	oscCam_dev->ppidev->dma_config |= DMA_WDSIZE_16;	
	oscCam_dev->ppidev->dma_config |= DMA2D;
	oscCam_dev->ppidev->dma_config |= DI_EN;
	/* DMA Parameters X/Y_count and X/Y_modify are set in function oscCam_read */

	/* enable interrupts for DMA0 */	
	bfin_write_SIC_IMASK(bfin_read_SIC_IMASK() | INTERRUPT_MASK);

	if (request_dma(CH_PPI, "PPI_DMA") < 0) {
		printk(KERN_ERR "%s: %s: Unable to attach PPI DMA channel\n",
		       __FUNCTION__,
		       sensor_name);
		return -EFAULT;
	} else
		set_dma_callback(CH_PPI, ppifcd_irq, oscCam_dev->ppidev);

	if (request_irq(IRQ_PPI_ERROR, ppifcd_irq_error, 0, "PPI ERROR", oscCam_dev->ppidev)) {
		printk(KERN_ERR "%s: %s: Unable to attach PPI error IRQ\n", 
		       __FUNCTION__,
		       sensor_name);
		free_dma(CH_PPI);
		return -EFAULT;
	} 
	// Disable this IRQ right away
	// We will enable it as soon as we expect data
	disable_irq(IRQ_PPI_ERROR);

/* Set BF537 Pin Configuration */
#if (defined(CONFIG_BF537) || defined(CONFIG_BF536) || defined(CONFIG_BF534))
	pr_debug("%s: Enabling PPI pins...\n", __FUNCTION__);
	//bfin_write_PORTG_FER(0x00FF); //for 8-bit interface
	bfin_write_PORTG_FER(0x03FF); //for 10-bit interface
	// Enable PPI FS1, FS2 and Clk, disable FS3
	bfin_write_PORTF_FER((bfin_read_PORTF_FER() | 0x8300) & ~0x0080);
	bfin_write_PORT_MUX(bfin_read_PORT_MUX() & ~0x0F00); /*do not enable fs3*/
#endif
	bfin_write_PPI_CONTROL(oscCam_dev->ppidev->ppi_control);

	pr_debug("%s: specified video device opened sucessfullly\n",
		__FUNCTION__);
	oscCam_dev->user++;

	return 0;
}

// Configures one of the frame buffers to use the specified
// memory location. 
static int oscCam_setFrameBuffer(struct frame_buffer * fbuf)
{
	if(fbuf == NULL)
		return -EINVAL;

	if(fbuf->id < 0 || fbuf->id >= MAX_NR_FRAME_BUFFERS) {
		pr_debug("%s: Invalid value (%d) for number of frame buffers!\n", 
			 __FUNCTION__,
			 fbuf->id);
		return -EINVAL;
	}

	if(oscCam_dev->buffer[fbuf->id].state == FRAME_GRABBING) {
		// This frame buffer is currently being written to and should thus not be replaced
		pr_debug("%s: Chosen frame buffer busy!\n", __FUNCTION__);
		return -EBUSY;
	}

	if(fbuf->data == NULL || fbuf->size == 0) { // Delete the frame buffer
		fbuf->size = 0;

		pr_debug("%s: NULL pointer specified as frame buffer!\nDeleting buffer.\n",
			__FUNCTION__);

		if(oscCam_dev->last_image.fbuf == oscCam_dev->buffer[fbuf->id].data) {
			// The buffer to be deleted contains the last valid image.
			// Therefore the last image must be invalidated
			oscCam_dev->last_image.fbuf = NULL;
		}

		if(oscCam_dev->ready_buf && oscCam_dev->ready_buf->data == oscCam_dev->buffer[fbuf->id].data) {
			// The buffer to be deleted is the current ready_buf
			oscCam_dev->ready_buf = NULL;
		}
	}

	oscCam_dev->buffer[fbuf->id].data = fbuf->data;
	oscCam_dev->buffer[fbuf->id].state = FRAME_UNUSED;
	oscCam_dev->buffer[fbuf->id].size = fbuf->size;

	return 0;	   
}

static int oscCam_setCaptureWindow(struct capture_window * cw)
{
	if(cw == NULL)
		return -EINVAL;
  
	if (cw->height < MIN_FRAME_HEIGHT ||
	    cw->row_off < 0 ||
	    (cw->height + cw->row_off) > MAX_FRAME_HEIGHT) {
		pr_debug("%s: ...no valid height (%d)\n", 
		       __FUNCTION__, 
		       cw->height);
		return -EINVAL;
	}
	if (cw->width < MIN_FRAME_WIDTH ||
	    cw->col_off < 0 ||
	    (cw->width + cw->col_off) > MAX_FRAME_WIDTH) {
		pr_debug("%s: ...no valid width (%d)\n", 
		       __FUNCTION__,
		       cw->width);
		return -EINVAL;
	}

	pr_debug("%s: ...using window %dx%d at %d/%d.\n", 
		 __FUNCTION__, 
		 cw->width, 
		 cw->height,
		 cw->col_off,
		 cw->row_off);

	oscCam_dev->capt_win.width = cw->width;
	oscCam_dev->capt_win.height = cw->height;
	oscCam_dev->size = oscCam_dev->capt_win.width * oscCam_dev->capt_win.height;
	oscCam_dev->capt_win.col_off = cw->col_off + COL_OFF_MIN;
	oscCam_dev->capt_win.row_off = cw->row_off + ROW_OFF_MIN;

	oscCam_configWindow(oscCam_dev->capt_win.col_off,
			  oscCam_dev->capt_win.row_off,
			  oscCam_dev->capt_win.width,
			  oscCam_dev->capt_win.height);
	return 0;
}

static int oscCam_getCaptureWindow(struct capture_window * cw)
{
	if(cw == NULL)
		return -EINVAL;

	cw->width = oscCam_dev->capt_win.width;
	cw->height = oscCam_dev->capt_win.height;
	cw->col_off = oscCam_dev->capt_win.col_off - COL_OFF_MIN;
	cw->row_off = oscCam_dev->capt_win.row_off - ROW_OFF_MIN;

	return 0;
}

// Prepares everything to receive image data from the 
// CMOS sensor but does not block until the data is in
// the RAM
static int oscCam_capture(struct capture_param * cp)
{
	int i;
	int inUse = true;
	size_t count;
			
	BUG_ON(oscCam_dev == NULL);
	BUG_ON(oscCam_dev->ppidev == NULL);
	BUG_ON(oscCam_dev->ppidev->rx_avail == NULL);
	BUG_ON(oscCam_dev->ppidev->cancelled);

	if(cp == NULL)
		return -EINVAL;

	i = cp->frame_buffer;
	pr_debug("%s: Capturing to frame buffer %d.\n", __FUNCTION__, i);

	////////////// Input validation /////////////
	if (i >= MAX_NR_FRAME_BUFFERS || oscCam_dev->buffer[i].size == 0) {
		pr_debug("%s: invalid frame (%d)",
			 __FUNCTION__,
			 cp->frame_buffer);
		return -EINVAL;
	}

	if(cp->trigger_mode != TRIGGER_MODE_EXTERNAL) {
		pr_debug("%s: Invalid trigger mode specified (%d)!\n",
			 __FUNCTION__,
			 cp->trigger_mode);
		return -EINVAL;
	}

	// If there is a capture in progress, return
	inUse = oscCam_IsBusy();
	if(inUse == true) {
		pr_debug("%s: Device already in use\n", __FUNCTION__);
		return -EBUSY;
	}

	/////////// Configure PPI ///////////
	oscCam_dev->ppidev->pixel_per_line = oscCam_dev->capt_win.width;
	oscCam_dev->ppidev->lines_per_frame = oscCam_dev->capt_win.height;
			
//	bfin_clear_PPI_STATUS();	
//	bfin_write_PPI_CONTROL(oscCam_dev->ppidev->ppi_control & ~PORT_EN);

	// Enable PPI interrupts
	BUG_ON(oscCam_dev->dma_buf != NULL);
//	bfin_write_SIC_IMASK(bfin_read_SIC_IMASK() | INTERRUPT_MASK); 
//	enable_irq(IRQ_PPI_ERROR);

	bfin_write_PPI_DELAY(oscCam_dev->ppidev->ppi_delay);
	bfin_write_PPI_COUNT(oscCam_dev->ppidev->pixel_per_line - 1);
	bfin_write_PPI_FRAME(oscCam_dev->ppidev->lines_per_frame);
			
	pr_debug("%s: setting PPI to %dx%d\n",
		 __FUNCTION__,
		 oscCam_dev->ppidev->pixel_per_line,
		 oscCam_dev->ppidev->lines_per_frame);

	pr_debug("%s: capture %zi byte, %dx%d (WxH) frame\n",
		 __FUNCTION__,
		 oscCam_dev->size,
		 oscCam_dev->capt_win.width, oscCam_dev->capt_win.height);

	////////// Configure DMA //////////
	set_dma_config(CH_PPI, oscCam_dev->ppidev->dma_config);

	set_dma_y_count(CH_PPI,	oscCam_dev->ppidev->lines_per_frame);			
	if (oscCam_dev->ppidev->dma_config & DMA_WDSIZE_16) {
		set_dma_x_modify(CH_PPI, 2);
		set_dma_y_modify(CH_PPI, 2);

		if(oscCam_dev->ppidev->ppi_control & PPI_PACKING) {
			// Div 2 because of 16-bit packing (2 pixel per operation)
			set_dma_x_count(CH_PPI, oscCam_dev->ppidev->pixel_per_line / 2);
			pr_debug("%s: Setting DMA to %dx%d/%dx%d (X/Y)\n",
				 __FUNCTION__,
				 oscCam_dev->ppidev->pixel_per_line / 2, 2,
				 oscCam_dev->ppidev->lines_per_frame, 2);
		} else {
			set_dma_x_count(CH_PPI, oscCam_dev->ppidev->pixel_per_line);
			pr_debug("%s: Setting DMA to %dx%d/%dx%d (X/Y)\n",
				 __FUNCTION__,
				 oscCam_dev->ppidev->pixel_per_line, 2,
				 oscCam_dev->ppidev->lines_per_frame, 2);
		}
	} else {
		set_dma_x_modify(CH_PPI, 1);
		set_dma_y_modify(CH_PPI, 1);
		set_dma_x_count(CH_PPI, oscCam_dev->ppidev->pixel_per_line);
		pr_debug("%s: Setting DMA to %dx%d/%dx%d (X/Y)\n",
			 __FUNCTION__,
			 oscCam_dev->ppidev->pixel_per_line, 1,
			 oscCam_dev->ppidev->lines_per_frame, 1);

	}

	////////////// Enable transfer ///////////////
	oscCam_dev->ppidev->trigger_mode = cp->trigger_mode;

	oscCam_dev->dma_buf = &oscCam_dev->buffer[i];
	oscCam_dev->dma_buf->state = FRAME_GRABBING;

	pr_debug("%s: Grabbing frame %d [0x%p]\n", 
		 __FUNCTION__,
		 i, 
		 oscCam_dev->dma_buf->data);
			
	// We want this access to be non-blocking, i.e. the image is
	// only triggered but not finished after return
	oscCam_dev->ppidev->nonblock = 1;
	count = oscCam_ppi2dma(oscCam_dev->ppidev,
			     oscCam_dev->dma_buf->data,
			     oscCam_dev->size);

	return 0;
}

// This is the work item belonging to oscCam_abortCapture()
static void oscCam_workItemResetCam(void *workData)
{
	// WorkItem to reset the camera (since this cannot be done in interrupt
	// context)
	// This will abort a capture in progress and reset the digital logic
	// of the CMOS chip while preserving its configuration
	int ret;
	BUG_ON(oscCam_dev->ppidev->cancelled == 0);

	ret = oscCam_i2c_write(i2c_global_client, 0x0C, 1);

	if(ret < 0) {
		pr_debug("%s: Resetting camera failed\n", __FUNCTION__);
	} else {
		pr_debug("%s: Camera reset.\n", __FUNCTION__);
	}
	oscCam_dev->ppidev->cancelled = 0;
	// Wake up any waiting processes so they do not remain
	// blocked forever
	wake_up_interruptible(oscCam_dev->ppidev->rx_avail);
	if(oscCam_dev->dma_buf)
		wake_up_interruptible(&oscCam_dev->dma_buf->wq);
	else
		printk(KERN_WARNING "%s: Resetting camera although no capture in progress...\n",
			__FUNCTION__);

	oscCam_dev->dma_buf->state = FRAME_ERROR;

	// Update the buffer pointers
	// Leave the ready_buf as it is
	oscCam_dev->dma_buf = NULL;
	oscCam_dev->next_buf = NULL;

	oscCam_dev->ppidev->done = 1;
}

// Aborts a previously scheduled capture and resets everything.
// This function uses Linux Work-Queues to reset the camera sensor
// and thus can be called from an interrupt context (it does not block).
// Since the worker thread are not running on real-time priority,
// flush_scheduled_work() should be called when wanting to be sure
// the abort action was completed.
static int oscCam_abortCapture(int arg)
{
	// This function does the most pressing part directly
	// here and schedules a WorkItem to do the things that
	// cannot be done in an interrupt context
	pr_debug("%s: Aborting capture...\n", __FUNCTION__);
	
	if(oscCam_dev->ppidev->done == 1) {
		// There is nothing to abort, since the transfer is already done
		pr_debug("%s: Nothing to abort...\n", __FUNCTION__);
		return -ENOENT;
	}

	// Disable PPI and DMA
	disable_dma(CH_PPI);
	bfin_write_PPI_CONTROL(bfin_read_PPI_CONTROL() & ~PORT_EN);

	// Disable PPI interrupts
	bfin_write_SIC_IMASK(bfin_read_SIC_IMASK() & ~INTERRUPT_MASK);
	disable_irq(IRQ_PPI_ERROR);

	clear_dma_irqstat(CH_PPI);
	bfin_clear_PPI_STATUS(); // Clear PPI error irq

	// We reset the camera last, since the communication over
	// i2c takes the longest
	// We don't do this directly but rather schedule it as a 
	// WorkItem, so this function can be called from an
	// interrupt context
	// The workitem will reset the cancelled variable after it is done
	oscCam_dev->ppidev->cancelled = 1;
	schedule_work(&oscCam_resetCamWork); // Reset the camera

	return 0;
}

// Wait until the data from a previously scheduled capture
// has arrived in the specified frame buffer. It optionally accepts a timeout
// and a maximum age of the desired image. 
// If the desired frame buffer already contains an image and no$
// capture is scheduled, it just returns with 0.
// Returns 0 on success, -EAGAIN on timeout, -ERANGE on max
// age violation.
static int oscCam_syncCapture(struct sync_param* sp)
{
	unsigned int timeout;
	unsigned long long cycles, max_cycles;

	// True if we waited for the capture to finish and thus do not need to check the age
	// of the image
	bool bCaptureJustFinished = false;                       
	int ret;

	BUG_ON(oscCam_dev->ppidev->cancelled);

	pr_debug("%s: frame %d\ttimeout %d ms\tmax_age %d ms\n", 
		 __FUNCTION__,
		 sp->frame,
		 sp->timeout,
		 sp->max_age);

	// Input Validation
	if (sp->frame >= MAX_NR_FRAME_BUFFERS || sp->frame < 0 || 
	    oscCam_dev->buffer[sp->frame].data == 0) {
		pr_debug("%s: invalid frame (%d)", __FUNCTION__, sp->frame);
		return -EINVAL;
	}


	switch (oscCam_dev->buffer[sp->frame].state) {
	case FRAME_UNUSED:
		return -ENOENT;
	case FRAME_READY:
	case FRAME_GRABBING:
		if (!oscCam_dev->ppidev)
			return -EIO;

		pr_debug("%s: Frame not ready yet. Waiting...\n", __FUNCTION__);
		if(sp->timeout == 0) { // Means no timeout

			// Wait until the DMA done interrupt wakes us up again
			ret = wait_event_interruptible(oscCam_dev->buffer[sp->frame].wq,
						       (oscCam_dev->buffer[sp->frame].state == FRAME_DONE));

		} else { // Timeout specified

			timeout = (sp->timeout * HZ) / 1000;  // sp->timeout is in Milliseconds => convert to jiffies

			// Wait until the DMA done interrupt wakes us up again or the timeout is invoked
			ret = wait_event_interruptible_timeout(oscCam_dev->buffer[sp->frame].wq,
							       (oscCam_dev->buffer[sp->frame].state == FRAME_DONE),
							       timeout);

			if(ret == 0 && oscCam_dev->buffer[sp->frame].state != FRAME_DONE) {
				// Timeout
				pr_debug("%s: Timeout (%ld)!\n", 
					 __FUNCTION__,
					 jiffies * 1000 / HZ);
				return -EAGAIN;
			}
		}

		if (ret != 0 && oscCam_dev->buffer[sp->frame].state != FRAME_DONE){
			pr_debug("%s, wait_event interrupted by signal %d!\n", 
				 __FUNCTION__,
				 ret);
			return -EINTR;
		}
				
		pr_debug("%s: Synch Ready on frame %d, grabstate = %d\n", 
			 __FUNCTION__,
			 sp->frame, 
			 oscCam_dev->buffer[sp->frame].state);

		if (oscCam_dev->buffer[sp->frame].state == FRAME_ERROR) {
			goto frame_error;
		}

		bCaptureJustFinished = true;
	case FRAME_DONE:
		//      oscCam_dev->buffer[sp->frame].state = FRAME_UNUSED;
		if(bCaptureJustFinished == false) {
			// The image capture did not just finish so we do not need to check its age
			if(sp->max_age != 0) {
				// A valid maximum age was supplied so we need to do the checking
				cycles = get_current_cycles(); // Get the current time in blackfin cycles
				cycles -= oscCam_dev->buffer[sp->frame].capture_done;
				// Better to make a multiplication and a comparison than a division
				max_cycles = (unsigned long long)sp->max_age * 
					(unsigned long long)(get_cclk()/1000); 
				if(cycles > max_cycles){
					pr_debug("Picture in selected buffer too old. Age: %llu cyc\tMax age: %llu cyc\n", 
						 cycles, 
						 max_cycles);
					return -ERANGE;
				}
			}
		}

		oscCam_dev->ready_buf = &oscCam_dev->buffer[sp->frame];
		pr_debug("%s: ready_buf = [0x%p]\n",
			 __FUNCTION__,
			 oscCam_dev->ready_buf->data);

		oscCam_dev->frame_count++;

		break;
	case FRAME_ERROR:
	frame_error:
		oscCam_dev->error_count++;
		return -EIO;
	}
	return 0;
}

// Finds the last valid image and copies its information to the
// image_info structure.
// Returns 0 on success.
static int oscCam_getLastFrame(struct image_info * img)
{
	if(img == NULL)
		return -EINVAL;

	if(oscCam_dev->last_image.fbuf == NULL) {
		pr_debug("%s: No last frame available.\n", __FUNCTION__);
		return -ENOENT;
	}

	memcpy(img, &oscCam_dev->last_image, sizeof(struct image_info));

	return 0;

}

// Write a register to the CMOS-Sensor(non-blocking).
// The register may not be written after this function
// returns. It only schedules the write-transfer.
// Returns 0 on success.
static int oscCam_setCamReg(struct reg_info * reg)
{
	int i;

	if(reg == NULL)
		return -EINVAL;

	if(reg->addr > 0x0 && reg->addr < 0x5) { // 0x1 - 0x4 set the window
		pr_debug("%s: Please set the capture window over the designated IOCTL functions!\n",
			__FUNCTION__);
		return -EINVAL;
	}

	for (i = 0; i < sizeof(i2c_regs) / sizeof(*i2c_regs); ++i) {
		if (i2c_regs[i].regnum == reg->addr) {
			pr_debug("%s: Writing register 0x%02x (%s) with 0x%04x\n",
				 __FUNCTION__,
				 reg->addr, 
				 i2c_regs[i].name, 
				 reg->value);
			oscCam_i2c_write(i2c_global_client, reg->addr, reg->value);
			return 0;
		}
	}
	return -EINVAL;
}

// Read a register from the CMOS-Sensor (blocking)
// Returns 0 on success.
static int oscCam_getCamReg(struct reg_info * reg)
{
	int i;

	if(reg == NULL)
		return -EINVAL;

	for (i = 0; i < sizeof(i2c_regs) / sizeof(*i2c_regs); ++i) {
		if (i2c_regs[i].regnum == reg->addr) {
			pr_debug("%s: Reading register 0x%02x (%s) ",
				 __FUNCTION__,
				 reg->addr, 
				 i2c_regs[i].name);
			oscCam_i2c_read(i2c_global_client, 
				      reg->addr, 
				      (unsigned short*)&reg->value, 
				      1);
			pr_debug("=> %d.\n", reg->value);
			return 0;
		}
	}
	return -EINVAL;
}

// This is the entry point for IOCTLs issued to this driver
// It finds the operation to execute and dispatches the corresponding
// functions.
static int oscCam_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
		      unsigned long arg)
{
	BUG_ON(oscCam_dev == NULL);

	if(oscCam_dev->ppidev->cancelled == 1) {
		// In case we still have a capture abort pending, we make sure 
		// it is finished before attempting anything new
		flush_scheduled_work();
	}

	switch(cmd) 
	{
	case CAM_SFRAMEBUF:       // Set framebuffers
		pr_debug("=>CAM_SFRAMEBUF.\n");

		return oscCam_setFrameBuffer((struct frame_buffer*)arg);
	case CAM_SWINDOW:         // Set the capture window
		pr_debug("=>CAM_SWINDOW.\n");

		return oscCam_setCaptureWindow((struct capture_window*)arg);
	case CAM_GWINDOW:         // Get window
		pr_debug("=>CAM_GWINDOW.\n");

		return oscCam_getCaptureWindow((struct capture_window*)arg);
	case CAM_CCAPTURE:        // Trigger image
		/* CAM_CCAPTURE starts the capture to frame
		 * when it returns, the frame is not captured yet -
		 * the driver just instructed PPI to start capture
		 * The userspace app has to use VIDIOCSYNC to wait
		 * until the capture of a frame is finished
		 */
		pr_debug("=>CAM_CCAPTURE.\n");

		return oscCam_capture((struct capture_param*)arg);
	case CAM_CABORTCAPT:     // Abort capture
		pr_debug("=>CAM_CABORTCAPT.\n");

		return oscCam_abortCapture((int)arg);
	case CAM_CSYNC:           // Synchronize capture
		/*CAM_CSYNC takes the frame number you want as argument
		 * and waits until the capture of that frame is finished
		 */
		pr_debug("=>CAM_CSYNC.\n");
		return oscCam_syncCapture((struct sync_param*)arg);
	case CAM_GLASTFRAME:      // Get the last valid frame
		pr_debug("=>CAM_GLASTFRAME.\n");

		return oscCam_getLastFrame((struct image_info*)arg);
	case CAM_SCAMREG:         // Write a register of the camera
		pr_debug("=>CAM_SCAMREG.\n");

		return oscCam_setCamReg((struct reg_info *)arg);
	case CAM_GCAMREG:         // Read a register of the camera
		pr_debug("=>CAM_GCAMREG.\n");

		return oscCam_getCamReg((struct reg_info *)arg);
	default: 
		pr_debug("=>Invalid IOCTL (cmd=%d, arg=0x%x)\n", cmd, (unsigned int)arg);
		return -ENOTTY;
	}
}
 
// Writing to the device node simply triggers an image. This does
// not set up the DMA or ensure otherwise that the data is 
// transfered to RAM
static ssize_t oscCam_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	pr_debug("%s: Buf: 0x%x, len: %d, offset: %d\n",
		 __FUNCTION__,
		 (unsigned int)buf,
		 (unsigned int)count,
		 (unsigned int)*f_pos);

	return 1;
}
   
// Reading the device node returns the last valid captured picture
static ssize_t oscCam_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	struct image_info img;
	unsigned int img_size;
	int ret;

	pr_debug("%s: Buf: 0x%x, len: %d, offset: %d\n",
		 __FUNCTION__,
		 (unsigned int)buf,
		 (unsigned int)count,
		 (unsigned int)*f_pos);

	ret = oscCam_getLastFrame(&img);
	if(!ret) {
		// There exists a last frame
		img_size = img.window.width*img.window.height;
		if(*f_pos >= img_size) {
			pr_debug("%s: Too large offset specified (%d, image size %d).\n",
				 __FUNCTION__,
				 (unsigned int)*f_pos,
				 img_size);
			return 0;
		}
		if(*f_pos + count > img_size) {
			count = img_size - *f_pos;
			pr_debug("%s: Too large length specified.\n", __FUNCTION__);
		}
		 
		if(copy_to_user(buf, img.fbuf + *f_pos, count)) {
			return -EFAULT;
		}
		pr_debug("%s: Reading last image (%dx%d at %d/%d), byte %d to %d\n",
			 __FUNCTION__,
			 img.window.width,
			 img.window.height,
			 img.window.col_off,
			 img.window.row_off,
			 (unsigned int)*f_pos,
			 (unsigned int)(*f_pos + count));
		*f_pos += count;
		return count;
	} else {
		pr_debug("%s: Getting last frame failed.\n",
			 __FUNCTION__);
		return 0;
	}
}

static int oscCam_close(struct inode *inode, struct file *filp)
{
	struct ppi_device_t *pdev = oscCam_dev->ppidev;
#if 1
	free_irq(IRQ_PPI_ERROR, oscCam_dev->ppidev);
#endif
	flush_scheduled_work();

	oscCam_regReset(pdev);
	ppi_fasync(-1, filp, 0);
	free_dma(CH_PPI);
	pdev->opened = 0;
	pdev->cancelled = 0;

	pr_debug("%s: ...specified video device closed sucessfullly\n",
		__FUNCTION__);
	oscCam_dev->user--;
	module_put(THIS_MODULE);
	oscCam_dev->frame_count = 0;
	oscCam_dev->error_count = 0;
  
	return 0;
}

static struct file_operations oscCam_fops = {
	.owner = THIS_MODULE,
	.open = oscCam_open,
	.release = oscCam_close,
	.ioctl = oscCam_ioctl,
//	.compat_ioctl = (void *)oscCam_ioctl,
	.llseek = no_llseek,
	.read = oscCam_read,
	.write = oscCam_write,
//	.mmap = oscCam_mmap,
};

static struct video_device oscCam_template = {
	//.owner = THIS_MODULE,
	.name = "Micron CMOS Camera: MT9V032",
	//.type = VFL_TYPE_GRABBER,
	//.type2 = VID_TYPE_CAPTURE | VID_TYPE_SUBCAPTURE,
	//.hardware = VID_HARDWARE_OSCCAM,
	.fops = &oscCam_fops,
	.release = &v4l_release,
	.minor = -1,
//	.debug = V4L2_DEBUG_IOCTL | V4L2_DEBUG_IOCTL_ARG,vide 
};

static __exit void mt9v032_oscar_exit(void)
{
	/* i2c_del_driver returns void */
	//if ((err = i2c_del_driver(&mt9v032_oscar_driver))) {
	//	printk(KERN_WARNING "%s: could not del i2c driver: %i\n",
	//	       sensor_name, err);
	//	return;
	//}
	i2c_del_driver(&mt9v032_oscar_driver);

	remove_proc_entry("oscCam", NULL);
}

static __init int mt9v032_oscar_init(void)
{
	int err;

	if (global_gain > 127) {
		printk(KERN_WARNING
		       "%s: %s: global gain was above 127; resetting to max of 127\n",
		       __FUNCTION__,
		       sensor_name);
		global_gain = 127;
	}

	err = i2c_add_driver(&mt9v032_oscar_driver);
	if (err) {
		printk(KERN_WARNING "%s: %s: could not add i2c driver: %i\n",
		       __FUNCTION__,
		       sensor_name, 
		       err);
		return err;
	}

	if (mt9v032_oscar_proc_init())
		printk(KERN_WARNING "%s: Could not create proc entry\n",
			__FUNCTION__);


	printk(KERN_INFO "%s: %s: Micron I2C driver ready\n", 
	       __FUNCTION__,
	       sensor_name);
	return 0;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Supercomputing Systems");

module_param(global_gain, int, 0);
module_param(debug, int, 0);

module_init(mt9v032_oscar_init);
module_exit(mt9v032_oscar_exit);

