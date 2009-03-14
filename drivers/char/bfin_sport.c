/* bfin_sport.c - simple interface to Blackfin SPORT peripheral
 *
 * Copyright 2004-2008 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <asm/blackfin.h>
#include <asm/bfin_sport.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>

#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/signal.h>
#include <asm/gpio.h>
#include <asm/portmux.h>

static int sport_major = SPORT_MAJOR;
static int sport_minor;
static int sport_nr_devs = SPORT_NR_DEVS;	/* number of bare sport devices */

/* XXX: this should get pushed to platform device */
#define SPORT_REQ(x) \
	[x] = {P_SPORT##x##_TFS, P_SPORT##x##_DTPRI, P_SPORT##x##_TSCLK, P_SPORT##x##_DTSEC, \
	       P_SPORT##x##_RFS, P_SPORT##x##_DRPRI, P_SPORT##x##_RSCLK, P_SPORT##x##_DRSEC, 0}
static u16 sport_req[][9] = {
#ifdef SPORT0_TCR1
	SPORT_REQ(0),
#endif
#ifdef SPORT1_TCR1
	SPORT_REQ(1),
#endif
#ifdef SPORT2_TCR1
	SPORT_REQ(2),
#endif
#ifdef SPORT3_TCR1
	SPORT_REQ(3),
#endif
};

#define SPORT_PARAMS(x) \
	[x] = { \
		.dma_rx_chan = CH_SPORT##x##_RX, \
		.dma_tx_chan = CH_SPORT##x##_TX, \
		.rx_irq      = IRQ_SPORT##x##_RX, \
		.tx_irq      = IRQ_SPORT##x##_TX, \
		.err_irq     = IRQ_SPORT##x##_ERROR, \
		.regs        = (struct sport_register *)SPORT##x##_TCR1, \
	}
static struct sport_dev sport_devices[] = {
#ifdef SPORT0_TCR1
	SPORT_PARAMS(0),
#endif
#ifdef SPORT1_TCR1
	SPORT_PARAMS(1),
#endif
#ifdef SPORT2_TCR1
	SPORT_PARAMS(2),
#endif
#ifdef SPORT3_TCR1
	SPORT_PARAMS(3),
#endif
};

#define DRV_NAME "bfin_sport"

static irqreturn_t dma_rx_irq_handler(int irq, void *dev_id);
static irqreturn_t dma_tx_irq_handler(int irq, void *dev_id);

/* note: multichannel is in units of 8 channels, tdm_count is # channels NOT / 8 ! */
static int sport_set_multichannel(struct sport_register *regs,
				  int tdm_count, int packed, int frame_delay)
{
	if (tdm_count) {
		int shift = 32 - tdm_count;
		unsigned int mask = (0xffffffff >> shift);

		regs->mcmc1 = ((tdm_count >> 3) - 1) << 12;	/* set WSIZE bits */
		regs->mcmc2 = (frame_delay << 12) | MCMEN |
		    (packed ? (MCDTXPE | MCDRXPE) : 0);

		regs->mtcs0 = regs->mrcs0 = mask;
	} else {
		regs->mcmc1 = regs->mcmc2 = 0;
		regs->mtcs0 = regs->mrcs0 = 0;
	}

	regs->mtcs1 = regs->mtcs2 = regs->mtcs3 = 0;
	regs->mrcs1 = regs->mrcs2 = regs->mrcs3 = 0;

	SSYNC();

	return 0;
}

static int sport_configure(struct sport_dev *dev, struct sport_config *config)
{
	unsigned int tcr1, tcr2, rcr1, rcr2;
	unsigned int clkdiv, fsdiv;
	struct sport_config *old_cfg = &dev->config;

	tcr1 = tcr2 = rcr1 = rcr2 = 0;
	clkdiv = fsdiv = 0;

	if ((old_cfg->dma_enabled == 0) && (config->dma_enabled)) {
		int ret;
		free_irq(dev->tx_irq, dev);
		free_irq(dev->rx_irq, dev);

		/* Request rx dma and set irq handler */
		ret = request_dma(dev->dma_rx_chan, "sport_rx_dma_chan");
		if (ret) {
			printk(KERN_ERR "Unable to request sport rx dma channel\n");
			return ret;
		}
		set_dma_callback(dev->dma_rx_chan, dma_rx_irq_handler, dev);

		/* Request tx dma and set irq handler */
		ret = request_dma(dev->dma_tx_chan, "sport_tx_dma_chan");
		if (ret) {
			printk(KERN_ERR "Unable to request sport tx dma channel\n");
			return ret;
		}
		set_dma_callback(dev->dma_tx_chan, dma_tx_irq_handler, dev);
	}
	memcpy(old_cfg, config, sizeof(*config));

	if ((dev->regs->tcr1 & TSPEN) || (dev->regs->rcr1 & RSPEN))
		return -EBUSY;

	if (config->mode == TDM_MODE) {
		if (config->channels & 0x7 || config->channels > 32)
			return -EINVAL;

		sport_set_multichannel(dev->regs, config->channels, 1,
				       config->frame_delay);
	} else if (config->mode == I2S_MODE) {
		tcr1 |= (TCKFE | TFSR);
		tcr2 |= TSFSE;

		rcr1 |= (RCKFE | RFSR);
		rcr2 |= RSFSE;
	} else {
		tcr1 |= (config->lsb_first << 4) | (config->fsync << 10) |
		      (config->data_indep << 11) | (config->act_low << 12) |
		      (config->late_fsync << 13) | (config->tckfe << 14);
		if (config->sec_en)
			tcr2 |= TXSE;

		rcr1 |= (config->lsb_first << 4) | (config->fsync << 10) |
		      (config->data_indep << 11) | (config->act_low << 12) |
		      (config->late_fsync << 13) | (config->tckfe << 14);
		if (config->sec_en)
			rcr2 |= RXSE;
	}

	/* Using internal clock */
	if (config->int_clk) {
		u_long sclk = get_sclk();

		if (config->serial_clk < 0 || config->serial_clk > sclk / 2)
			return -EINVAL;
		clkdiv = sclk / (2 * config->serial_clk) - 1;
		fsdiv = config->serial_clk / config->fsync_clk - 1;

		tcr1 |= (ITCLK | ITFS);
		rcr1 |= (IRCLK | IRFS);
	}

	/* Setting data format */
	tcr1 |= (config->data_format << 2);	/* Bit TDTYPE */
	rcr1 |= (config->data_format << 2);	/* Bit TDTYPE */
	if (config->word_len >= 3 && config->word_len <= 32) {
		tcr2 |= config->word_len - 1;
		rcr2 |= config->word_len - 1;
	} else
		return -EINVAL;

	dev->regs->rcr1 = rcr1;
	dev->regs->rcr2 = rcr2;
	dev->regs->rclkdiv = clkdiv;
	dev->regs->rfsdiv = fsdiv;
	dev->regs->tcr1 = tcr1;
	dev->regs->tcr2 = tcr2;
	dev->regs->tclkdiv = clkdiv;
	dev->regs->tfsdiv = fsdiv;
	SSYNC();

	pr_debug("tcr1:0x%x, tcr2:0x%x, rcr1:0x%x, rcr2:0x%x\n"
		 "mcmc1:0x%x, mcmc2:0x%x\n",
		 dev->regs->tcr1, dev->regs->tcr2,
		 dev->regs->rcr1, dev->regs->rcr2,
		 dev->regs->mcmc1, dev->regs->mcmc2);

	return 0;
}

static inline uint16_t sport_wordsize(int word_len)
{
	uint16_t wordsize = 0;

	if (word_len <= 8)
		wordsize = WDSIZE_8;
	else if (word_len <= 16)
		wordsize = WDSIZE_16;
	else if (word_len <= 32)
		wordsize = WDSIZE_32;
	else
		printk(KERN_ERR "%s: word_len:%d is error\n", __func__,
		       word_len);

	return wordsize;
}

static irqreturn_t dma_rx_irq_handler(int irq, void *dev_id)
{
	struct sport_dev *dev = dev_id;

	pr_debug("%s enter\n", __func__);
	dev->regs->rcr1 &= ~RSPEN;
	SSYNC();
	disable_dma(dev->dma_rx_chan);

	dev->wait_con = 1;
	wake_up(&dev->waitq);

	clear_dma_irqstat(dev->dma_rx_chan);
	return IRQ_HANDLED;
}

static irqreturn_t dma_tx_irq_handler(int irq, void *dev_id)
{
	struct sport_dev *dev = dev_id;
	unsigned int status;

	pr_debug("%s enter\n", __func__);
	status = get_dma_curr_irqstat(dev->dma_tx_chan);
	while (status & DMA_RUN) {
		status = get_dma_curr_irqstat(dev->dma_tx_chan);
		pr_debug("status:0x%04x\n", status);
	}
	status = dev->regs->stat;
	while (!(status & TXHRE)) {
		pr_debug("%s status:%x\n", __func__, status);
		udelay(1);
		status = *(volatile unsigned short *)&dev->regs->stat;
	}
	/* Wait for the last byte sent out */
	udelay(500);
	pr_debug("%s status:%x\n", __func__, status);

	dev->regs->tcr1 &= ~TSPEN;
	SSYNC();
	disable_dma(dev->dma_tx_chan);

	dev->wait_con = 1;
	wake_up(&dev->waitq);

	/* Clear the interrupt status */
	clear_dma_irqstat(dev->dma_tx_chan);

	return IRQ_HANDLED;
}

static irqreturn_t sport_rx_handler(int irq, void *dev_id)
{
	struct sport_dev *dev = dev_id;
	struct sport_config *cfg = &dev->config;

	int word_bytes = (cfg->word_len + 7) / 8;

	if (word_bytes == 3)
		word_bytes = 4;

	if (word_bytes == 1) {
		while ((dev->rx_received < dev->rx_len) &&
		       (dev->regs->stat & RXNE)) {
			*(dev->rx_buf + dev->rx_received) =
			    *(volatile unsigned char *)(&dev->regs->rx);
			dev->rx_received++;
		}
	} else if (word_bytes == 2) {
		while ((dev->rx_received < dev->rx_len) &&
		       (dev->regs->stat & RXNE)) {
			*(unsigned short *)(dev->rx_buf + dev->rx_received) =
			    *(volatile unsigned short *)(&dev->regs->rx);
			dev->rx_received += 2;
		}
	} else if (word_bytes == 4) {
		while ((dev->rx_received < dev->rx_len) &&
		       (dev->regs->stat & RXNE)) {
			*(unsigned long *)(dev->rx_buf + dev->rx_received) =
			    *(volatile unsigned long *)(&dev->regs->rx);
			dev->rx_received += 4;
		}
	}

	if (dev->rx_received >= dev->rx_len) {
		dev->regs->rcr1 &= ~RSPEN;
		dev->wait_con = 1;
		wake_up(&dev->waitq);
	}

	return IRQ_HANDLED;
}

static inline void sport_tx_write(struct sport_dev *dev)
{
	struct sport_config *cfg = &dev->config;
	int word_bytes = (cfg->word_len + 7) / 8;

	if (word_bytes == 3)
		word_bytes = 4;

	if (word_bytes == 1) {
		while ((dev->tx_sent < dev->tx_len) && !(dev->regs->stat & TXF)) {
			*(volatile unsigned char *)(&dev->regs->tx) =
			    *(dev->tx_buf + dev->tx_sent);
			dev->tx_sent++;
		}
	} else if (word_bytes == 2) {
		while ((dev->tx_sent < dev->tx_len) && !(dev->regs->stat & TXF)) {
			*(volatile unsigned short *)(&dev->regs->tx) =
			    *(unsigned short *)(dev->tx_buf + dev->tx_sent);
			dev->tx_sent += 2;
		}
	} else if (word_bytes == 4) {
		while ((dev->tx_sent < dev->tx_len) && !(dev->regs->stat & TXF)) {
			*(volatile unsigned long *)dev->regs->tx =
			    *(unsigned long *)(dev->tx_buf + dev->tx_sent);
			dev->tx_sent += 4;
		}
	}
}

static irqreturn_t sport_tx_handler(int irq, void *dev_id)
{
	struct sport_dev *dev = dev_id;

	if (dev->tx_sent < dev->tx_len)
		sport_tx_write(dev);

	if (dev->tx_len != 0 && dev->tx_sent >= dev->tx_len
	    && dev->config.int_clk) {
		unsigned int stat;

		stat = dev->regs->stat;
		while (!(stat & TXHRE)) {
			udelay(1);
			pr_debug("%s:stat:%x\n", __func__, stat);
			stat = *(volatile unsigned short *)&dev->regs->stat;
		}
		udelay(500);
		dev->regs->tcr1 &= ~TSPEN;
		SSYNC();
		pr_debug("%s:stat:%x\n", __func__, stat);
		dev->wait_con = 1;
		wake_up(&dev->waitq);
	}

	return IRQ_HANDLED;
}

static irqreturn_t sport_err_handler(int irq, void *dev_id)
{
	struct sport_dev *dev = dev_id;
	uint16_t status;

	pr_debug("%s enter\n", __func__);
	status = dev->regs->stat;

	if (status & (TOVF | TUVF | ROVF | RUVF)) {
		dev->regs->stat = (status & (TOVF | TUVF | ROVF | RUVF));
		if (dev->config.dma_enabled) {
			disable_dma(dev->dma_rx_chan);
			disable_dma(dev->dma_tx_chan);
		}
		dev->regs->tcr1 &= ~TSPEN;
		dev->regs->rcr1 &= ~RSPEN;
		SSYNC();

		if (!dev->config.dma_enabled && !dev->config.int_clk) {
			if (status & TUVF) {
				dev->wait_con = 1;
				wake_up(&dev->waitq);
			}
		} else
			printk(KERN_WARNING "sport %d status error:%s%s%s%s\n",
			       dev->sport_num,
			       status & TOVF ? " TOVF" : "",
			       status & TUVF ? " TUVF" : "",
			       status & ROVF ? " ROVF" : "",
			       status & RUVF ? " RUVF" : "");
	}

	if (dev->config.dma_enabled || dev->config.int_clk)
		send_sig(SIGABRT, dev->task, 1);

	return IRQ_HANDLED;
}

/*
 * Open and close
 */

static int sport_open(struct inode *inode, struct file *filp)
{
	int ret;
	struct sport_dev *dev;	/* device information */

	pr_debug("%s enter\n", __func__);
	dev = container_of(inode->i_cdev, struct sport_dev, cdev);
	filp->private_data = dev;	/* for other methods */

	memset(&dev->config, 0, sizeof(struct sport_config));

	dev->rx_buf = NULL;
	dev->rx_len = 0;
	dev->rx_received = 0;
	dev->tx_buf = NULL;
	dev->tx_len = 0;
	dev->tx_sent = 0;
	dev->wait_con = 0;

	ret = request_irq(dev->tx_irq, sport_tx_handler, IRQF_SHARED, DRV_NAME "-tx", dev);
	if (ret) {
		printk(KERN_ERR "Unable to request sport tx irq\n");
		goto fail;
	}

	ret = request_irq(dev->rx_irq, sport_rx_handler, IRQF_SHARED, DRV_NAME "-rx", dev);
	if (ret) {
		printk(KERN_ERR "Unable to request sport rx irq\n");
		goto fail1;
	}

	ret = request_irq(dev->err_irq, sport_err_handler, 0, DRV_NAME "-err", dev);
	if (ret) {
		printk(KERN_ERR "Unable to request sport err irq\n");
		goto fail2;
	}

	ret = peripheral_request_list(sport_req[dev->sport_num], DRV_NAME);
	if (ret) {
		printk(KERN_ERR DRV_NAME ": Requesting Peripherals failed\n");
		goto fail3;
	}

	dev->task = current;

	return 0;

 fail3:
	free_irq(dev->err_irq, dev);
 fail2:
	free_irq(dev->rx_irq, dev);
 fail1:
	free_irq(dev->tx_irq, dev);
 fail:
	free_dma(dev->dma_rx_chan);
	free_dma(dev->dma_tx_chan);

	return ret;
}

static int sport_release(struct inode *inode, struct file *filp)
{
	struct sport_dev *dev;

	pr_debug("%s enter\n", __func__);
	dev = container_of(inode->i_cdev, struct sport_dev, cdev);

	dev->regs->tcr1 &= ~TSPEN;
	dev->regs->rcr1 &= ~RSPEN;

	if (dev->config.dma_enabled) {
		free_dma(dev->dma_rx_chan);
		free_dma(dev->dma_tx_chan);
	} else {
		free_irq(dev->tx_irq, dev);
		free_irq(dev->rx_irq, dev);
	}
	free_irq(dev->err_irq, dev);

	peripheral_free_list(sport_req[dev->sport_num]);

	return 0;
}

static ssize_t sport_read(struct file *filp, char __user *buf, size_t count,
			  loff_t *f_pos)
{
	DECLARE_COMPLETION(done);
	struct sport_dev *dev = filp->private_data;
	struct sport_config *cfg = &dev->config;

	pr_debug("%s count:%ld\n", __func__, count);

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	dev->wait_con = 0;
	if (cfg->dma_enabled) {
		int word_bytes = (cfg->word_len + 7) / 8;
		uint16_t dma_config, xcount, ycount;

		if (word_bytes == 3)
			word_bytes = 4;

		/* Invalidate the buffer */
		invalidate_dcache_range((unsigned long)buf,
					(unsigned long)(buf + count));
		pr_debug("DMA mode read\n");
		/* Configure dma */
		dma_config =
		    (WNR | RESTART | sport_wordsize(cfg->word_len) | DI_EN);
		xcount = count / word_bytes;
		ycount = 0;
		if (xcount > 0x8000) {
			ycount = xcount >> 15;
			xcount = 0x8000;
			dma_config |= DMA2D;
		}
		set_dma_start_addr(dev->dma_rx_chan, (unsigned long)buf);
		set_dma_x_count(dev->dma_rx_chan, xcount);
		set_dma_x_modify(dev->dma_rx_chan, word_bytes);
		if (ycount > 0) {
			set_dma_y_count(dev->dma_rx_chan, ycount);
			set_dma_y_modify(dev->dma_rx_chan, word_bytes);
		}
		set_dma_config(dev->dma_rx_chan, dma_config);

		enable_dma(dev->dma_rx_chan);
	} else {
		dev->rx_buf = buf;
		dev->rx_len = count;
		dev->rx_received = 0;
	}

	dev->regs->rcr1 |= RSPEN;
	SSYNC();

	if (wait_event_interruptible(dev->waitq, dev->wait_con) < 0) {
		pr_debug("Receive a signal to interrupt\n");
		dev->wait_con = 0;
		mutex_unlock(&dev->mutex);
		return -ERESTARTSYS;
	}
	dev->wait_con = 0;

	pr_debug("Complete called in dma rx irq handler\n");
	mutex_unlock(&dev->mutex);

	return count;
}

static void dump_dma_regs(void)
{
#ifdef DEBUG
	struct dma_register *dma = (struct dma_register *)DMA4_NEXT_DESC_PTR;

	pr_debug("%s config:0x%04x, x_count:0x%04x,"
		 " x_modify:0x%04x\n", __func__, dma->cfg,
		 dma->x_count, dma->x_modify);
#endif
}

static ssize_t sport_write(struct file *filp, const char __user *buf,
			   size_t count, loff_t *f_pos)
{
	DECLARE_COMPLETION(done);
	struct sport_dev *dev = filp->private_data;
	struct sport_config *cfg = &dev->config;
	pr_debug("%s count:%ld  dma_tx_chan:%d\n",
		 __func__, count, dev->dma_tx_chan);

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	dev->wait_con = 0;
	/* Configure dma to start transfer */
	if (cfg->dma_enabled) {
		uint16_t dma_config, xcount, ycount;
		int word_bytes = (cfg->word_len + 7) / 8;

		if (word_bytes == 3)
			word_bytes = 4;

		pr_debug("DMA mode\n");
		flush_dcache_range((unsigned long)buf,
				   (unsigned long)(buf + count));

		/* Configure dma */
		dma_config = (RESTART | sport_wordsize(cfg->word_len) | DI_EN);
		xcount = count / word_bytes;
		ycount = 0;
		if (xcount > 0x8000) {
			ycount = xcount >> 15;
			xcount = 0x8000;
			dma_config |= DMA2D;
		}
		set_dma_start_addr(dev->dma_tx_chan, (unsigned long)buf);
		set_dma_x_count(dev->dma_tx_chan, xcount);
		set_dma_x_modify(dev->dma_tx_chan, word_bytes);
		if (ycount > 0) {
			set_dma_y_count(dev->dma_tx_chan, ycount);
			set_dma_y_modify(dev->dma_tx_chan, word_bytes);
		}
		set_dma_config(dev->dma_tx_chan, dma_config);

		enable_dma(dev->dma_tx_chan);
		dump_dma_regs();
	} else {
		/* Configure parameters to start PIO transfer */
		dev->tx_buf = buf;
		dev->tx_len = count;
		dev->tx_sent = 0;

		sport_tx_write(dev);
	}
	dev->regs->tcr1 |= TSPEN;
	SSYNC();

	pr_debug("wait for transfer finished\n");
	if (wait_event_interruptible(dev->waitq, dev->wait_con) < 0) {
		pr_debug("Receive a signal to interrupt\n");
		dev->wait_con = 0;
		mutex_unlock(&dev->mutex);
		return -ERESTARTSYS;
	}
	dev->wait_con = 0;
	pr_debug("waiting over\n");

	mutex_unlock(&dev->mutex);

	return count;
}

static int sport_ioctl(struct inode *inode, struct file *filp,
		       unsigned int cmd, unsigned long arg)
{
	struct sport_dev *dev = filp->private_data;
	struct sport_config config;

	pr_debug("%s: enter, arg:0x%lx\n", __func__, arg);
	switch (cmd) {
	case SPORT_IOC_CONFIG:
		if (copy_from_user(&config, (void *)arg, sizeof(config)))
			return -EFAULT;
		if (sport_configure(dev, &config) < 0)
			return -EFAULT;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t sport_status_show(struct class *sport_class, char *buf)
{
	char *p;
	unsigned short i;
	p = buf;

	for (i = 0; i < sport_nr_devs; ++i)
		p += sprintf(p,
			"sport%d:\nrx_irq=%d, rx_received=%d, tx_irq=%d, tx_sent=%d,\n"
			"mode=%d, channels=%d, data_format=%d, word_len=%d.\n",
			i, sport_devices[i].rx_irq,
			sport_devices[i].rx_received,
			sport_devices[i].tx_irq,
			sport_devices[i].tx_sent,
			sport_devices[i].config.mode,
			sport_devices[i].config.channels,
			sport_devices[i].config.data_format,
			sport_devices[i].config.word_len);

	return p - buf;
}

static struct file_operations sport_fops = {
	.owner = THIS_MODULE,
	.read = sport_read,
	.write = sport_write,
	.ioctl = sport_ioctl,
	.open = sport_open,
	.release = sport_release,
};

static struct class *sport_class;

static CLASS_ATTR(status, S_IRUGO, &sport_status_show, NULL);

static void __exit sport_cleanup_module(void)
{
	int i;
	dev_t devno = MKDEV(sport_major, sport_minor);

	for (i = 0; i < sport_nr_devs; ++i)
		cdev_del(&sport_devices[i].cdev);

	unregister_chrdev_region(devno, sport_nr_devs);
}
module_exit(sport_cleanup_module);

static void sport_setup_cdev(struct sport_dev *dev, int index)
{
	int err, devno = MKDEV(sport_major, sport_minor + index);

	cdev_init(&dev->cdev, &sport_fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		printk(KERN_NOTICE "Error %d adding sport%d", err, index);
}

static int __init sport_init_module(void)
{
	int minor;
	int result, i;
	dev_t dev = 0;

	dev = MKDEV(sport_major, sport_minor);
	result = register_chrdev_region(dev, sport_nr_devs, "sport");
	if (result < 0) {
		printk(KERN_WARNING "sport: can't get major %d\n", sport_major);
		return result;
	}

	sport_class = class_create(THIS_MODULE, "sport");
	result = class_create_file(sport_class, &class_attr_status);
	if (result) {
		unregister_chrdev_region(dev, sport_nr_devs);
		return result;
	}
	for (minor = 0; minor < sport_nr_devs; minor++)
		device_create(sport_class, NULL, MKDEV(sport_major, minor),
		              NULL, "sport%d", minor);

	/* Initialize each device. */
	for (i = 0; i < sport_nr_devs; ++i) {
		sport_setup_cdev(&sport_devices[i], i);
		sport_devices[i].sport_num = i;
		mutex_init(&sport_devices[i].mutex);
		init_waitqueue_head(&sport_devices[i].waitq);
	}

	return 0;
}
module_init(sport_init_module);

MODULE_AUTHOR("Roy Huang <roy.huang@analog.com>");
MODULE_DESCRIPTION("Common Blackfin SPORT driver");
MODULE_LICENSE("GPL");
