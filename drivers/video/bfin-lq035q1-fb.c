/*
 * File:         drivers/video/bfin-lq035q1-fb.c
 * Based on:
 * Author:       Michael Hennerich <hennerich@blackfin.uclinux.org>
 *
 * Created:
 * Description:  Blackfin LCD Framebufer driver SHARP LQ035Q1DH02
 *
 *
 * Modified:
 *               Copyright 2008 Analog Devices Inc.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/backlight.h>
#include <linux/lcd.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>


#include <asm/blackfin.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/portmux.h>
#include <asm/gptimers.h>

#include <asm/bfin-lq035q1.h>

#define LCD_X_RES		320	/* Horizontal Resolution */
#define LCD_Y_RES		240	/* Vertical Resolution */
#define LCD_BPP			16	/* Bit Per Pixel */
#define	DMA_BUS_SIZE		16
#define CLOCKS_PER_PIX		1

	/*
	 * HS and VS timing parameters (all in number of PPI clk ticks)
	 */

#define U_LINE		4				/* Blanking Lines */

#define H_ACTPIX	(LCD_X_RES * CLOCKS_PER_PIX)	/* active horizontal pixel */
#define H_PERIOD	(336 * CLOCKS_PER_PIX)		/* HS period */
#define H_PULSE		2				/* HS pulse width */
#define H_START		7				/* first valid pixel */

#define	V_LINES		(LCD_Y_RES + U_LINE)		/* total vertical lines */
#define V_PULSE		2				/* VS pulse width (1-5 H_PERIODs) */
#define V_PERIOD	(H_PERIOD * V_LINES)		/* VS period */

#define ACTIVE_VIDEO_MEM_OFFSET		((U_LINE / 2) * LCD_X_RES * (LCD_BPP / 8))

#define BFIN_LCD_NBR_PALETTE_ENTRIES	256

#define PPI_TX_MODE			0x2
#define PPI_XFER_TYPE_11		0xC
#define PPI_PORT_CFG_01			0x10
#define PPI_POLS_1			0x8000

#define LQ035_INDEX			0x74
#define LQ035_DATA			0x76

#define LQ035_DRIVER_OUTPUT_CTL		0x1
#define LQ035_SHUT_CTL			0x11

#define LQ035_DRIVER_OUTPUT_MASK	(LQ035_LR | LQ035_TB | LQ035_BGR | LQ035_REV)
#define LQ035_DRIVER_OUTPUT_DEFAULT 	(0x2AEF & ~LQ035_DRIVER_OUTPUT_MASK)

#define LQ035_SHUT			(1 << 0)	/* Shutdown */
#define LQ035_ON			(0 << 0)	/* Shutdown */

#define DRIVER_NAME "bfin-lq035q1"
static char driver_name[] = DRIVER_NAME;

struct bfin_lq035q1fb_info {
	struct fb_info *fb;
	struct device *dev;
	struct bfin_lq035q1fb_disp_info *disp_info;
	unsigned char *fb_buffer;	/* RGB Buffer */
	dma_addr_t dma_handle;
	int lq035_mmap;
	int lq035_open_cnt;
	int irq;
	spinlock_t lock;	/* lock */
	u32 pseudo_pal[16];
};

static int nocursor;
module_param(nocursor, int, 0644);
MODULE_PARM_DESC(nocursor, "cursor enable/disable");


static struct {
	struct spi_device *spidev;
	unsigned short mode;
	unsigned short init;
} spi_control;

static int lq035q1_control(unsigned char reg, unsigned short value)
{
	int ret;
	u8 regs[3] = {LQ035_INDEX, 0, 0};
	u8 dat[3] = {LQ035_DATA, 0, 0};

	if (spi_control.spidev) {
		regs[2] = reg;
		dat[1] = value >> 8;
		dat[2] = value & 0xFF;

		ret = spi_write(spi_control.spidev, regs, ARRAY_SIZE(regs));
		ret |= spi_write(spi_control.spidev, dat, ARRAY_SIZE(dat));
	} else
		return -ENODEV;

	return ret;
}

static int lq035q1_spidev_probe(struct spi_device *spi)
{
	int ret;
	spi_control.spidev = spi;

	ret = lq035q1_control(LQ035_SHUT_CTL, LQ035_ON);
	ret |= lq035q1_control(LQ035_DRIVER_OUTPUT_CTL, spi_control.mode);

	if (ret)
		return ret;

	spi_control.init = 1;

	return 0;
}

static int lq035q1_spidev_remove(struct spi_device *spi)
{
	return lq035q1_control(LQ035_SHUT_CTL, LQ035_SHUT);
}

#ifdef CONFIG_PM
static int lq035q1_spidev_suspend(struct spi_device *spi, pm_message_t state)
{
	return lq035q1_control(LQ035_SHUT_CTL, LQ035_SHUT);
}

static int lq035q1_spidev_resume(struct spi_device *spi)
{
	int ret = lq035q1_control(LQ035_DRIVER_OUTPUT_CTL, spi_control.mode);

	if (ret)
		return ret;

	return lq035q1_control(LQ035_SHUT_CTL, LQ035_ON);
}
#else
#define lq035q1_spidev_suspend		NULL
#define lq035q1_spidev_resume		NULL
#endif

/* Power down all displays on reboot, poweroff or halt */
static void lq035q1_spidev_shutdown(struct spi_device *spi)
{
	lq035q1_control(LQ035_SHUT_CTL, LQ035_SHUT);
}

static struct spi_driver spidev_spi = {
	.driver = {
		.name =		DRIVER_NAME"-spi",
		.owner =	THIS_MODULE,
	},
	.probe =	lq035q1_spidev_probe,
	.remove =	__devexit_p(lq035q1_spidev_remove),
	.shutdown	= lq035q1_spidev_shutdown,
	.suspend	= lq035q1_spidev_suspend,
	.resume		= lq035q1_spidev_resume,
};

static int lq035q1_backlight(struct bfin_lq035q1fb_info *info, unsigned arg)
{
	if (info->disp_info->use_bl)
		gpio_set_value(info->disp_info->gpio_bl, arg);

	return 0;
}

static void bfin_lq035q1_config_ppi(struct bfin_lq035q1fb_info *fbi)
{
	bfin_write_PPI_DELAY(H_START);
	bfin_write_PPI_COUNT(H_ACTPIX - 1);
	bfin_write_PPI_FRAME(V_LINES);

	bfin_write_PPI_CONTROL(PPI_TX_MODE |	   /* output mode , PORT_DIR */
				PPI_XFER_TYPE_11 | /* sync mode XFR_TYPE */
				PPI_PORT_CFG_01 |  /* two frame sync PORT_CFG */
				DLEN_16	|	   /* 16 bit data length */
				PPI_POLS_1);	   /* faling edge syncs POLS */
}

static inline void bfin_lq035q1_disable_ppi(void)
{
	bfin_write_PPI_CONTROL(bfin_read_PPI_CONTROL() & ~PORT_EN);
}

static inline void bfin_lq035q1_enable_ppi(void)
{
	bfin_write_PPI_CONTROL(bfin_read_PPI_CONTROL() | PORT_EN);
}

static void bfin_lq035q1_start_timers(void)
{
	enable_gptimers(TIMER1bit | TIMER0bit);
}

static void bfin_lq035q1_stop_timers(void)
{
	disable_gptimers(TIMER0bit | TIMER1bit);

	set_gptimer_status(0, TIMER_STATUS_TRUN0 | TIMER_STATUS_TRUN1 |
				TIMER_STATUS_TIMIL0 | TIMER_STATUS_TIMIL1 |
				 TIMER_STATUS_TOVF0 | TIMER_STATUS_TOVF1);

}

static void bfin_lq035q1_init_timers(void)
{

	bfin_lq035q1_stop_timers();

	set_gptimer_period(TIMER0_id, H_PERIOD);
	set_gptimer_pwidth(TIMER0_id, H_PULSE);
	set_gptimer_config(TIMER0_id, TIMER_MODE_PWM | TIMER_PERIOD_CNT |
				      TIMER_TIN_SEL | TIMER_CLK_SEL|
				      TIMER_EMU_RUN);

	set_gptimer_period(TIMER1_id, V_PERIOD);
	set_gptimer_pwidth(TIMER1_id, V_PULSE);
	set_gptimer_config(TIMER1_id, TIMER_MODE_PWM | TIMER_PERIOD_CNT |
				      TIMER_TIN_SEL | TIMER_CLK_SEL |
				      TIMER_EMU_RUN);

}

static void bfin_lq035q1_config_dma(struct bfin_lq035q1fb_info *fbi)
{

	set_dma_config(CH_PPI,
		       set_bfin_dma_config(DIR_READ, DMA_FLOW_AUTO,
					   INTR_DISABLE, DIMENSION_2D,
					   DATA_SIZE_16,
					   DMA_NOSYNC_KEEP_DMA_BUF));
	set_dma_x_count(CH_PPI, (LCD_X_RES * LCD_BPP) / DMA_BUS_SIZE);
	set_dma_x_modify(CH_PPI, DMA_BUS_SIZE / 8);
	set_dma_y_count(CH_PPI, V_LINES);

	set_dma_y_modify(CH_PPI, DMA_BUS_SIZE / 8);
	set_dma_start_addr(CH_PPI, (unsigned long)fbi->fb_buffer);

}

static	u16 ppi0_req_16[] = {P_PPI0_CLK, P_PPI0_FS1, P_PPI0_FS2,
			    P_PPI0_D0, P_PPI0_D1, P_PPI0_D2,
			    P_PPI0_D3, P_PPI0_D4, P_PPI0_D5,
			    P_PPI0_D6, P_PPI0_D7, P_PPI0_D8,
			    P_PPI0_D9, P_PPI0_D10, P_PPI0_D11,
			    P_PPI0_D12, P_PPI0_D13, P_PPI0_D14,
			    P_PPI0_D15, 0};

static int bfin_lq035q1_request_ports(int action)
{
	if (action) {
		if (peripheral_request_list(ppi0_req_16, DRIVER_NAME)) {
			printk(KERN_ERR "Requesting Peripherals faild\n");
			return -EFAULT;
		}
	} else
		peripheral_free_list(ppi0_req_16);

	return 0;
}

static int bfin_lq035q1_fb_open(struct fb_info *info, int user)
{
	struct bfin_lq035q1fb_info *fbi = info->par;

	spin_lock(&fbi->lock);
	fbi->lq035_open_cnt++;

	if (fbi->lq035_open_cnt <= 1) {

		bfin_lq035q1_disable_ppi();
		SSYNC();

		bfin_lq035q1_config_dma(fbi);
		bfin_lq035q1_config_ppi(fbi);
		bfin_lq035q1_init_timers();

		/* start dma */
		enable_dma(CH_PPI);
		bfin_lq035q1_enable_ppi();
		bfin_lq035q1_start_timers();
		lq035q1_backlight(fbi, 1);
	}

	spin_unlock(&fbi->lock);

	return 0;
}

static int bfin_lq035q1_fb_release(struct fb_info *info, int user)
{
	struct bfin_lq035q1fb_info *fbi = info->par;

	spin_lock(&fbi->lock);

	fbi->lq035_open_cnt--;
	fbi->lq035_mmap = 0;

	if (fbi->lq035_open_cnt <= 0) {
		lq035q1_backlight(fbi, 0);
		bfin_lq035q1_disable_ppi();
		SSYNC();
		disable_dma(CH_PPI);
		bfin_lq035q1_stop_timers();
		memset(fbi->fb_buffer, 0, info->fix.smem_len);

	}

	spin_unlock(&fbi->lock);

	return 0;
}

static int bfin_lq035q1_fb_check_var(struct fb_var_screeninfo *var,
				   struct fb_info *info)
{

	if (var->bits_per_pixel != LCD_BPP) {
		pr_debug("%s: depth not supported: %u BPP\n", __func__,
			 var->bits_per_pixel);
		return -EINVAL;
	}

	if (info->var.xres != var->xres || info->var.yres != var->yres ||
	    info->var.xres_virtual != var->xres_virtual ||
	    info->var.yres_virtual != var->yres_virtual) {
		pr_debug("%s: Resolution not supported: X%u x Y%u \n",
			 __func__, var->xres, var->yres);
		return -EINVAL;
	}

	/*
	 *  Memory limit
	 */

	if ((info->fix.line_length * var->yres_virtual) > info->fix.smem_len) {
		pr_debug("%s: Memory Limit requested yres_virtual = %u\n",
			 __func__, var->yres_virtual);
		return -ENOMEM;
	}


	return 0;
}

static int bfin_lq035q1_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct bfin_lq035q1fb_info *fbi = info->par;

	if (fbi->lq035_mmap)
		return -1;

	spin_lock(&fbi->lock);
	fbi->lq035_mmap = 1;
	spin_unlock(&fbi->lock);

	vma->vm_start = (unsigned long)(fbi->fb_buffer + ACTIVE_VIDEO_MEM_OFFSET);

	vma->vm_end = vma->vm_start + info->fix.smem_len;
	/* For those who don't understand how mmap works, go read
	 *   Documentation/nommu-mmap.txt.
	 * For those that do, you will know that the VM_MAYSHARE flag
	 * must be set in the vma->vm_flags structure on noMMU
	 *   Other flags can be set, and are documented in
	 *   include/linux/mm.h
	 */
	vma->vm_flags |= VM_MAYSHARE | VM_SHARED;

	return 0;
}

int bfin_lq035q1_fb_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	if (nocursor)
		return 0;
	else
		return -EINVAL;	/* just to force soft_cursor() call */
}

static int bfin_lq035q1_fb_setcolreg(u_int regno, u_int red, u_int green,
				   u_int blue, u_int transp,
				   struct fb_info *info)
{
	if (regno >= BFIN_LCD_NBR_PALETTE_ENTRIES)
		return -EINVAL;

	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {

		u32 value;
		/* Place color in the pseudopalette */
		if (regno > 16)
			return -EINVAL;

		red >>= (16 - info->var.red.length);
		green >>= (16 - info->var.green.length);
		blue >>= (16 - info->var.blue.length);

		value = (red << info->var.red.offset) |
		    (green << info->var.green.offset) |
		    (blue << info->var.blue.offset);
		value &= 0xFFFFFF;

		((u32 *) (info->pseudo_palette))[regno] = value;

	}

	return 0;
}

static struct fb_ops bfin_lq035q1_fb_ops = {
	.owner = THIS_MODULE,
	.fb_open = bfin_lq035q1_fb_open,
	.fb_release = bfin_lq035q1_fb_release,
	.fb_check_var = bfin_lq035q1_fb_check_var,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_mmap = bfin_lq035q1_fb_mmap,
	.fb_cursor = bfin_lq035q1_fb_cursor,
	.fb_setcolreg = bfin_lq035q1_fb_setcolreg,
};

static irqreturn_t bfin_lq035q1_irq_error(int irq, void *dev_id)
{
	/*struct bfin_lq035q1fb_info *info = (struct bfin_lq035q1fb_info *)dev_id;*/

	u16 status = bfin_read_PPI_STATUS();
	bfin_write_PPI_STATUS(-1);

	if (status) {
		bfin_lq035q1_disable_ppi();
		disable_dma(CH_PPI);

		/* start dma */
		enable_dma(CH_PPI);
		bfin_lq035q1_enable_ppi();
		bfin_write_PPI_STATUS(-1);
	}

	return IRQ_HANDLED;
}

static int __init bfin_lq035q1_probe(struct platform_device *pdev)
{
	struct bfin_lq035q1fb_info *info;
	struct fb_info *fbinfo;
	int ret;

	printk(KERN_INFO DRIVER_NAME ": %dx%d %d-bit RGB FrameBuffer initializing...\n",
					 LCD_X_RES, LCD_Y_RES, LCD_BPP);

	ret = request_dma(CH_PPI, "CH_PPI");
	if (ret < 0) {
		printk(KERN_ERR DRIVER_NAME
		       ": couldn't request CH_PPI DMA\n");
		goto out1;
	}

	fbinfo =
	    framebuffer_alloc(sizeof(struct bfin_lq035q1fb_info), &pdev->dev);
	if (!fbinfo) {
		ret = -ENOMEM;
		goto out2;
	}

	info = fbinfo->par;
	info->fb = fbinfo;
	info->dev = &pdev->dev;

	info->disp_info = pdev->dev.platform_data;

	spi_control.mode = (info->disp_info->mode &
		LQ035_DRIVER_OUTPUT_MASK) | LQ035_DRIVER_OUTPUT_DEFAULT;

	platform_set_drvdata(pdev, fbinfo);

	strcpy(fbinfo->fix.id, driver_name);

	fbinfo->fix.type = FB_TYPE_PACKED_PIXELS;
	fbinfo->fix.type_aux = 0;
	fbinfo->fix.xpanstep = 0;
	fbinfo->fix.ypanstep = 0;
	fbinfo->fix.ywrapstep = 0;
	fbinfo->fix.accel = FB_ACCEL_NONE;
	fbinfo->fix.visual = FB_VISUAL_TRUECOLOR;

	fbinfo->var.nonstd = 0;
	fbinfo->var.activate = FB_ACTIVATE_NOW;
	fbinfo->var.height = -1;
	fbinfo->var.width = -1;
	fbinfo->var.accel_flags = 0;
	fbinfo->var.vmode = FB_VMODE_NONINTERLACED;

	fbinfo->var.xres = LCD_X_RES;
	fbinfo->var.xres_virtual = LCD_X_RES;
	fbinfo->var.yres = LCD_Y_RES;
	fbinfo->var.yres_virtual = LCD_Y_RES;
	fbinfo->var.bits_per_pixel = LCD_BPP;

	if (info->disp_info->mode & LQ035_BGR) {
		fbinfo->var.red.offset = 0;
		fbinfo->var.green.offset = 5;
		fbinfo->var.blue.offset = 11;
	} else {
		fbinfo->var.red.offset = 11;
		fbinfo->var.green.offset = 5;
		fbinfo->var.blue.offset = 0;
	}

	fbinfo->var.transp.offset = 0;

	fbinfo->var.red.length = 5;
	fbinfo->var.green.length = 6;
	fbinfo->var.blue.length = 5;
	fbinfo->var.transp.length = 0;

	fbinfo->fix.smem_len = LCD_X_RES * LCD_Y_RES * LCD_BPP / 8
				+ ACTIVE_VIDEO_MEM_OFFSET;

	fbinfo->fix.line_length = fbinfo->var.xres_virtual *
	    fbinfo->var.bits_per_pixel / 8;


	fbinfo->fbops = &bfin_lq035q1_fb_ops;
	fbinfo->flags = FBINFO_FLAG_DEFAULT;

	info->fb_buffer =
	    dma_alloc_coherent(NULL, fbinfo->fix.smem_len, &info->dma_handle,
			       GFP_KERNEL);

	if (NULL == info->fb_buffer) {
		printk(KERN_ERR DRIVER_NAME
		       ": couldn't allocate dma buffer.\n");
		ret = -ENOMEM;
		goto out3;
	}

	memset(info->fb_buffer, 0, fbinfo->fix.smem_len);

	fbinfo->screen_base = (void *)info->fb_buffer + ACTIVE_VIDEO_MEM_OFFSET;
	fbinfo->fix.smem_start = (int)info->fb_buffer + ACTIVE_VIDEO_MEM_OFFSET;

	fbinfo->fbops = &bfin_lq035q1_fb_ops;

	fbinfo->pseudo_palette = &info->pseudo_pal;

	ret = fb_alloc_cmap(&fbinfo->cmap, BFIN_LCD_NBR_PALETTE_ENTRIES, 0);
	if (ret < 0) {
		printk(KERN_ERR DRIVER_NAME
		       "Fail to allocate colormap (%d entries)\n",
		       BFIN_LCD_NBR_PALETTE_ENTRIES);
		goto out4;
	}

	ret = bfin_lq035q1_request_ports(1);
	if (ret) {
		printk(KERN_ERR DRIVER_NAME ": couldn't request gpio port.\n");
		goto out6;
	}

	info->irq = platform_get_irq(pdev, 0);
	if (info->irq < 0) {
		ret = -EINVAL;
		goto out7;
	}

	ret = request_irq(info->irq, bfin_lq035q1_irq_error, IRQF_DISABLED,
			"PPI ERROR", info);
	if (ret < 0) {
		printk(KERN_ERR DRIVER_NAME
		       ": unable to request PPI ERROR IRQ\n");
		goto out7;
	}

	ret = spi_register_driver(&spidev_spi);
	if (ret < 0) {
		printk(KERN_ERR DRIVER_NAME
		       ": couldn't register SPI Interface\n");
		goto out8;
	}

	if (info->disp_info->use_bl) {
		ret = gpio_request(info->disp_info->gpio_bl, "LQ035 Backlight");

		if (ret) {
			printk(KERN_ERR "%s: Failed to request GPIO %d\n",
			DRIVER_NAME, info->disp_info->gpio_bl);
			goto out9;
		}
		gpio_direction_output(info->disp_info->gpio_bl, 0);
	}

	ret = register_framebuffer(fbinfo);
	if (ret < 0) {
		printk(KERN_ERR DRIVER_NAME
		       ": unable to register framebuffer.\n");
		goto out10;
	}

	return 0;

out10:
	if (info->disp_info->use_bl)
		gpio_free(info->disp_info->gpio_bl);
out9:
	spi_unregister_driver(&spidev_spi);
out8:
	free_irq(info->irq, info);
out7:
	bfin_lq035q1_request_ports(0);
out6:
	fb_dealloc_cmap(&fbinfo->cmap);
out4:
	dma_free_coherent(NULL, fbinfo->fix.smem_len, info->fb_buffer,
			  info->dma_handle);
out3:
	framebuffer_release(fbinfo);
out2:
	free_dma(CH_PPI);
out1:
	platform_set_drvdata(pdev, NULL);

	return ret;
}

static int bfin_lq035q1_remove(struct platform_device *pdev)
{

	struct fb_info *fbinfo = platform_get_drvdata(pdev);
	struct bfin_lq035q1fb_info *info = fbinfo->par;

	if (info->disp_info->use_bl)
		gpio_free(info->disp_info->gpio_bl);

	spi_unregister_driver(&spidev_spi);

	unregister_framebuffer(fbinfo);

	free_dma(CH_PPI);
	free_irq(info->irq, info);

	if (info->fb_buffer != NULL)
		dma_free_coherent(NULL, fbinfo->fix.smem_len, info->fb_buffer,
				  info->dma_handle);

	fb_dealloc_cmap(&fbinfo->cmap);

	bfin_lq035q1_request_ports(0);

	platform_set_drvdata(pdev, NULL);
	framebuffer_release(fbinfo);

	printk(KERN_INFO DRIVER_NAME ": Unregister LCD driver.\n");

	return 0;
}

#ifdef CONFIG_PM
static int bfin_lq035q1_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct fb_info *fbinfo = platform_get_drvdata(pdev);
	struct bfin_lq035q1fb_info *info = fbinfo->par;

	if (info->lq035_open_cnt) {
		lq035q1_backlight(info, 0);
		bfin_lq035q1_disable_ppi();
		SSYNC();
		disable_dma(CH_PPI);
		bfin_lq035q1_stop_timers();
		bfin_write_PPI_STATUS(-1);
	}

	return 0;
}

static int bfin_lq035q1_resume(struct platform_device *pdev)
{
	struct fb_info *fbinfo = platform_get_drvdata(pdev);
	struct bfin_lq035q1fb_info *info = fbinfo->par;

	if (info->lq035_open_cnt) {
		bfin_lq035q1_disable_ppi();
		SSYNC();

		bfin_lq035q1_config_dma(info);
		bfin_lq035q1_config_ppi(info);
		bfin_lq035q1_init_timers();

		/* start dma */
		enable_dma(CH_PPI);
		bfin_lq035q1_enable_ppi();
		bfin_lq035q1_start_timers();
		lq035q1_backlight(info, 1);
	}

	return 0;
}
#else
#define bfin_lq035q1_suspend	NULL
#define bfin_lq035q1_resume	NULL
#endif

static struct platform_driver bfin_lq035q1_driver = {
	.probe = bfin_lq035q1_probe,
	.remove = bfin_lq035q1_remove,
	.suspend = bfin_lq035q1_suspend,
	.resume = bfin_lq035q1_resume,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __devinit bfin_lq035q1_driver_init(void)
{
	return platform_driver_register(&bfin_lq035q1_driver);
}

static void __exit bfin_lq035q1_driver_cleanup(void)
{
	platform_driver_unregister(&bfin_lq035q1_driver);
}

MODULE_DESCRIPTION("Blackfin TFT LCD Driver");
MODULE_LICENSE("GPL");

module_init(bfin_lq035q1_driver_init);
module_exit(bfin_lq035q1_driver_cleanup);
