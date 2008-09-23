/*
 * File:         drivers/media/video/blackfin/blackfin_cam.h
 * Based on:
 * Author:       Michael Hennerich <hennerich@blackfin.uclinux.org>
 *
 * Created:
 * Description:  V4L driver for Blackfin
 *
 *
 * Modified:
 *               Copyright 2004-2007 Analog Devices Inc.
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

#ifndef BLACKFIN_CAM_H
#define BLACKFIN_CAM_H

#undef USE_ITU656
#undef USE_2ND_BUF_IN_CACHED_MEM
#define USE_PPI_ERROR
#undef USE_GPIO

#if defined(CONFIG_BF537)
# define  bcap_STANDBY  GPIO_PG11
# define  bcap_LEDS     GPIO_PG8
# define  bcap_TRIGGER  GPIO_PG13
#endif

#if defined(CONFIG_BF533)
# define  bcap_STANDBY  GPIO_8
# define  bcap_LEDS     GPIO_11
# define  bcap_TRIGGER  GPIO_6
# define  bcap_FS3      GPIO_3
#endif

#undef PPI_BASE
#ifdef CONFIG_BF561
# ifdef CONFIG_VIDEO_BLACKFIN_CAM_PPI1
#  define PPI_BASE PPI1_CONTROL
# else
#  define PPI_BASE PPI0_CONTROL
# endif
#elif defined(CONFIG_BF54x)
#undef CH_PPI
#define CH_PPI 		CH_EPPI1
#define PORT_EN		EPPI_EN
#define PACK_EN 	PACKEN
#define IRQ_PPI		IRQ_EPPI1
#define IRQ_PPI_ERROR	IRQ_EPP1_ERROR
#define CONFIG_VIDEO_BLACKFIN_CAM_PPI1
#define PPI_COUNT_CORR_OFFSET	0
#else
#define PPI_COUNT_CORR_OFFSET	1
#endif

#ifdef PPI_BASE
# define bfin_read_PPI_CONTROL()      bfin_read16(PPI_BASE + 0x00)
# define bfin_write_PPI_CONTROL(val)  bfin_write16(PPI_BASE + 0x00, val)
# define bfin_read_PPI_STATUS()       bfin_read16(PPI_BASE + 0x04)
# define bfin_write_PPI_STATUS(val)   bfin_write16(PPI_BASE + 0x04, val)
# define bfin_clear_PPI_STATUS()      bfin_read_PPI_STATUS()
# define bfin_read_PPI_COUNT()        bfin_read16(PPI_BASE + 0x08)
# define bfin_write_PPI_COUNT(val)    bfin_write16(PPI_BASE + 0x08, val)
# define bfin_read_PPI_DELAY()        bfin_read16(PPI_BASE + 0x0c)
# define bfin_write_PPI_DELAY(val)    bfin_write16(PPI_BASE + 0x0c, val)
# define bfin_read_PPI_FRAME()        bfin_read16(PPI_BASE + 0x10)
# define bfin_write_PPI_FRAME(val)    bfin_write16(PPI_BASE + 0x10, val)
#endif
#ifdef EPPI1_STATUS
# define bfin_read_PPI_CONTROL()      bfin_read32(EPPI1_CONTROL)
# define bfin_write_PPI_CONTROL(val)  bfin_write32(EPPI1_CONTROL, val)
# define bfin_read_PPI_STATUS()       bfin_read16(EPPI1_STATUS)
# define bfin_write_PPI_STATUS(val)   bfin_write16(EPPI1_STATUS, val)
# define bfin_clear_PPI_STATUS()      bfin_write_PPI_STATUS(-1)
# define bfin_read_PPI_COUNT()        bfin_read16(EPPI1_LINE)
# define bfin_write_PPI_COUNT(val)    bfin_write16(EPPI1_LINE, val)
# define bfin_read_PPI_DELAY()        bfin_read16(EPPI1_HDELAY)
# define bfin_write_PPI_DELAY(val)    bfin_write16(EPPI1_HDELAY, val)
# define bfin_read_PPI_FRAME()        bfin_read16(EPPI1_FRAME)
# define bfin_write_PPI_FRAME(val)    bfin_write16(EPPI1_FRAME, val)
#endif

#define DRV_NAME	"blackfin-cam"

#ifdef CONFIG_VIDEO_BLACKFIN_CAM_PPI1
# define PPI_8 {P_PPI1_CLK, P_PPI1_D0, P_PPI1_D1, P_PPI1_D2, P_PPI1_D3, \
	P_PPI1_D4, P_PPI1_D5, P_PPI1_D6, P_PPI1_D7, P_PPI1_FS1, P_PPI1_FS2, 0}
#else
# define PPI_8 {P_PPI0_CLK, P_PPI0_D0, P_PPI0_D1, P_PPI0_D2, P_PPI0_D3, \
	P_PPI0_D4, P_PPI0_D5, P_PPI0_D6, P_PPI0_D7, P_PPI0_FS1, P_PPI0_FS2, 0}
#endif

#define BCAP_NUM_BUFS 2
#define I2C_DRIVERID_BCAP  81	/* experimental (next avail. in i2c-id.h) */
#define NO_TRIGGER  16

#define X_RES(x) (x >> 16)
#define Y_RES(x) (x & 0xFFFF)
#define MSB(x)(x >> 8)
#define LSB(x)(x & 0xFF)

#define RES(x, y)	(x << 16) | (y & 0xFFFF)

#define RES_VGA		RES(640, 480)
#define RES_QVGA	RES(320, 240)
#define RES_QQVGA	RES(160, 120)

#define RES_CIF		RES(352, 288)
#define RES_QCIF	RES(176, 144)
#define RES_SQCIF	RES(128, 96)

/* Controls */
enum {
	CAM_CMD_INIT,
	CAM_CMD_SET_RESOLUTION,
	CAM_CMD_SET_FRAMERATE,
	CAM_CMD_SET_PIXFMT,
	CAM_CMD_EXIT,
	CAM_CMD_SET_CONTRAST,
	CAM_CMD_SET_SATURATION,
	CAM_CMD_SET_HOR_MIRROR,
	CAM_CMD_SET_VERT_MIRROR,
	CAM_CMD_SET_FLICKER_FREQ,
	CAM_CMD_GET_FRAMERATE,
	CAM_CMD_GET_FLICKER_FREQ,
	CAM_CMD_GET_VERT_MIRROR,
	CAM_CMD_GET_HOR_MIRROR,
	CAM_CMD_GET_SATURATION,
	CAM_CMD_GET_CONTRAST,
};

struct ppi_device_t {
	struct bcap_device_t *bcap_dev;
	int opened;
	unsigned short irqnum;
	unsigned short done;
	unsigned short dma_config;
	unsigned short pixel_per_line;
	unsigned short lines_per_frame;
	unsigned short bpp;
#ifdef CONFIG_BF54x
	unsigned int ppi_control;
#else
	unsigned short ppi_control;
#endif
	unsigned short ppi_status;
	unsigned short ppi_delay;
	unsigned short ppi_trigger_gpio;
	wait_queue_head_t *rx_avail;
};

struct bcap_buffer {
	unsigned char *data;
	wait_queue_head_t wq;
	volatile int state;
	unsigned int scyc;	/* < start cycle for frame  */
	unsigned int ecyc;	/* < end cycle for frame    */
	unsigned long stime;	/* < start time for frame   */
	unsigned long etime;	/* < end time for frame     */
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

struct bcap_device_t {
	int frame_count;
	struct video_device *videodev;
	struct ppi_device_t *ppidev;
	struct i2c_client *client;
	struct bcap_camera_ops *cam_ops;
	int user;
	char name[32];
	int width;
	int height;
	size_t size;
	struct timeval *stv;
	struct timeval *etv;
	struct bcap_buffer buffer[BCAP_NUM_BUFS];
	struct bcap_buffer *next_buf;
	struct bcap_buffer *dma_buf;
	struct bcap_buffer *ready_buf;
	spinlock_t lock;
};

struct sensor_data {
	struct i2c_client client;
	struct bcap_device_t *bcap_dev;
	struct bcap_camera_ops *cam_ops;
};

struct bcap_camera_ops {
	int (*cam_control) (struct i2c_client * client, u32 cmd, u32 arg);
	int (*create_sysfs) (struct video_device * v4ldev);
	int (*power) (u32 arg);
};

#endif				/* BLACKFIN_CAM_H */
