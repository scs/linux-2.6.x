/*
 *  linux/drivers/mmc/bfin_sdh.c - Analog Blackfin SDH Controller
 *
 *  Author	Roy Huang <roy.huang@analog.com>
 *  Copyright (C) 2007 Analog Device Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
/* In term of ADSP_BF54x_Blackfin_Processor_Peripheral_Hardware_Reference,
 * the SDH allows software to detect a card when it is inserted into its slot.
 * The SD_DATA3 pin powers up low due to a special pull-down resistor. When an
 * SD Card is inserted in its slot, the resistance increases and a rising edge
 * is detected by the SDH module.
 * But this doesn't work sometimes. When a MMC/SD card is inserted, the voltage
 * doesn't rise on SD_DATA3. In term of The MultiMediaCard System Specification,
 * SD_DATA3 is used as CS pin in SPI mode. The MultiMediaCard wakes up in the
 * MultiMediaCard mode. During the scan procedure, host will send CMD0 to reset
 * MMC card, if CS pin is low, MMC card will enter SPI mode. Of course Secure
 * Digital Host controller is not a SPI controller. So the Card detect function
 * has to be disabled. After card is inserted run "echo 0 > /proc/driver/sdh"
 * to trigger card scanning */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/proc_fs.h>

#include <asm/cacheflush.h>
#include <asm/dma.h>
#include <asm/portmux.h>
#include <asm/bfin_sdh.h>

#define DRIVER_NAME	"bfin-sdh"

#define NR_SG	32

#if defined(CONFIG_BF51x)
#define bfin_write_SDH_PWR_CTL		bfin_write_RSI_PWR_CTL
#define bfin_read_SDH_CLK_CTL		bfin_read_RSI_CLK_CTL
#define bfin_write_SDH_CLK_CTL		bfin_write_RSI_CLK_CTL
#define bfin_write_SDH_ARGUMENT		bfin_write_RSI_ARGUMENT
#define bfin_write_SDH_COMMAND		bfin_write_RSI_COMMAND
#define bfin_write_SDH_DATA_TIMER	bfin_write_RSI_DATA_TIMER
#define bfin_read_SDH_RESPONSE0		bfin_read_RSI_RESPONSE0
#define bfin_read_SDH_RESPONSE1		bfin_read_RSI_RESPONSE1
#define bfin_read_SDH_RESPONSE2		bfin_read_RSI_RESPONSE2
#define bfin_read_SDH_RESPONSE3		bfin_read_RSI_RESPONSE3
#define bfin_write_SDH_DATA_LGTH	bfin_write_RSI_DATA_LGTH
#define bfin_read_SDH_DATA_CTL		bfin_read_RSI_DATA_CTL
#define bfin_write_SDH_DATA_CTL		bfin_write_RSI_DATA_CTL
#define bfin_read_SDH_DATA_CNT		bfin_read_RSI_DATA_CNT
#define bfin_write_SDH_STATUS_CLR	bfin_write_RSI_STATUS_CLR
#define bfin_read_SDH_E_STATUS		bfin_read_RSI_E_STATUS
#define bfin_write_SDH_E_STATUS		bfin_write_RSI_E_STATUS
#define bfin_read_SDH_STATUS		bfin_read_RSI_STATUS
#define bfin_write_SDH_MASK0		bfin_write_RSI_MASK0
#define bfin_read_SDH_CFG		bfin_read_RSI_CFG
#define bfin_write_SDH_CFG		bfin_write_RSI_CFG
#endif

struct dma_desc_array {
	unsigned long	start_addr;
	unsigned short	cfg;
	unsigned short	x_count;
	short		x_modify;
} __attribute__((packed));

struct sdh_host {
	struct mmc_host		*mmc;
	spinlock_t		lock; /* Why I have to give a comment here? */
	struct resource		*res;
	void __iomem		*base;
	int			irq;
	int			stat_irq;
	int			dma_ch;
	int			dma_dir;
	struct dma_desc_array	*sg_cpu;
	dma_addr_t		sg_dma;
	int			dma_len;

	unsigned int		imask;
	unsigned int		power_mode;
	unsigned int		clk_div;

	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data		*data;
};

static struct bfin_sd_host *get_sdh_data(struct platform_device *pdev)
{
	return pdev->dev.platform_data;
}

static void sdh_stop_clock(struct sdh_host *host)
{
	bfin_write_SDH_CLK_CTL(bfin_read_SDH_CLK_CTL() & ~CLK_E);
	SSYNC();
}

static void sdh_enable_stat_irq(struct sdh_host *host, unsigned int mask)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->imask |= mask;
	bfin_write_SDH_MASK0(mask);
	SSYNC();
	spin_unlock_irqrestore(&host->lock, flags);
}

static void sdh_disable_stat_irq(struct sdh_host *host, unsigned int mask)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->imask &= ~mask;
	bfin_write_SDH_MASK0(host->imask);
	SSYNC();
	spin_unlock_irqrestore(&host->lock, flags);
}

static void sdh_setup_data(struct sdh_host *host, struct mmc_data *data)
{
	unsigned int length;
	unsigned int data_ctl;
	unsigned int dma_cfg;
#if defined(CONFIG_BF54x)
	int i;
#endif
	pr_debug("%s enter flags:0x%x\n", __FUNCTION__, data->flags);
	host->data = data;
	data_ctl = 0;
	dma_cfg = 0;

	length = data->blksz * data->blocks;
	bfin_write_SDH_DATA_LGTH(length);

	if (data->flags & MMC_DATA_STREAM)
		data_ctl |= DTX_MODE;

	if (data->flags & MMC_DATA_READ)
		data_ctl |= DTX_DIR;

	BUG_ON(data->blksz & (data->blksz -1));
	data_ctl |= ((ffs(data->blksz) -1) << 4);

	bfin_write_SDH_DATA_CTL(data_ctl);

	/* FIXME later */
	bfin_write_SDH_DATA_TIMER(0xFFFFFFFF);
	SSYNC();

	if (data->flags & MMC_DATA_READ) {
		host->dma_dir = DMA_FROM_DEVICE;
		dma_cfg |= WNR;
	} else
		host->dma_dir = DMA_TO_DEVICE;

	sdh_enable_stat_irq(host, (DAT_CRC_FAIL | DAT_TIME_OUT | DAT_END));
	host->dma_len = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len, host->dma_dir);
#if defined(CONFIG_BF54x)
	dma_cfg |= DMAFLOW_ARRAY | NDSIZE_5 | RESTART | WDSIZE_32 | DMAEN;
	for (i = 0; i < host->dma_len; i++) {
		host->sg_cpu[i].start_addr = sg_dma_address(&data->sg[i]);
		host->sg_cpu[i].cfg = dma_cfg;
		host->sg_cpu[i].x_count = sg_dma_len(&data->sg[i]) / 4;
		host->sg_cpu[i].x_modify = 4;
		pr_debug("%d: start_addr:0x%lx, cfg:0x%x, x_count:0x%x, x_modify:0x%x\n",
				i, host->sg_cpu[i].start_addr, host->sg_cpu[i].cfg,
				host->sg_cpu[i].x_count, host->sg_cpu[i].x_modify);
	}
	flush_dcache_range((unsigned int)host->sg_cpu, \
			(unsigned int)host->sg_cpu + \
			host->dma_len * sizeof(struct dma_desc_array));
	/* Set the last descriptor to stop mode */
	host->sg_cpu[host->dma_len - 1].cfg &= ~(DMAFLOW | NDSIZE);
	host->sg_cpu[host->dma_len - 1].cfg |= DI_EN;

	set_dma_curr_desc_addr(host->dma_ch, (unsigned long *)host->sg_dma);
	set_dma_x_count(host->dma_ch, 0);
	set_dma_x_modify(host->dma_ch, 0);
	set_dma_config(host->dma_ch, dma_cfg);
#elif defined(CONFIG_BF51x)
	dma_cfg |= WDSIZE_32 | DMAEN;
	set_dma_start_addr(host->dma_ch, sg_dma_address(&data->sg[0]));
	set_dma_x_count(host->dma_ch, length / 4);
	set_dma_x_modify(host->dma_ch, 4);
	set_dma_config(host->dma_ch, dma_cfg);
#endif
	bfin_write_SDH_DATA_CTL(bfin_read_SDH_DATA_CTL() | DTX_DMA_E | DTX_E);

	SSYNC();

	pr_debug("%s exit\n", __FUNCTION__);
}

static void sdh_start_cmd(struct sdh_host *host, struct mmc_command *cmd)
{
	unsigned int sdh_cmd;
	unsigned int stat_mask;

	pr_debug("%s enter cmd:0x%p\n", __FUNCTION__, cmd);
	WARN_ON(host->cmd != NULL);
	host->cmd = cmd;

	sdh_cmd = 0;
	stat_mask = 0;

	sdh_cmd |= cmd->opcode;

	if (cmd->flags & MMC_RSP_PRESENT) {
		sdh_cmd |= CMD_RSP;
		stat_mask |= CMD_RESP_END;
	} else
		stat_mask |= CMD_SENT;

	if (cmd->flags & MMC_RSP_136)
		sdh_cmd |= CMD_L_RSP;

	stat_mask |= CMD_CRC_FAIL | CMD_TIME_OUT;

	sdh_enable_stat_irq(host, stat_mask);

	bfin_write_SDH_ARGUMENT(cmd->arg);
	bfin_write_SDH_COMMAND(sdh_cmd | CMD_E);
	bfin_write_SDH_CLK_CTL(bfin_read_SDH_CLK_CTL() | CLK_E);
	SSYNC();
}

static void sdh_finish_request(struct sdh_host *host, struct mmc_request *mrq)
{
	pr_debug("%s enter\n", __FUNCTION__);
	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;
	mmc_request_done(host->mmc, mrq);
}

static int sdh_cmd_done(struct sdh_host *host, unsigned int stat)
{
	struct mmc_command *cmd = host->cmd;

	pr_debug("%s enter cmd:%p\n", __FUNCTION__, cmd);
	if (!cmd)
		return 0;

	host->cmd = NULL;

	if (cmd->flags & MMC_RSP_PRESENT) {
		cmd->resp[0] = bfin_read_SDH_RESPONSE0();
		if (cmd->flags & MMC_RSP_136) {
			cmd->resp[1] = bfin_read_SDH_RESPONSE1();
			cmd->resp[2] = bfin_read_SDH_RESPONSE2();
			cmd->resp[3] = bfin_read_SDH_RESPONSE3();
		}
	}
	if (stat & CMD_TIME_OUT)
		cmd->error = -ETIMEDOUT;
	else if (stat & CMD_CRC_FAIL && cmd->flags & MMC_RSP_CRC)
		cmd->error = -EILSEQ;

	sdh_disable_stat_irq(host, (CMD_SENT | CMD_RESP_END | CMD_TIME_OUT | CMD_CRC_FAIL));

	if (host->data && !cmd->error) {
		if (host->data->flags & MMC_DATA_WRITE)
			sdh_setup_data(host, host->data);

		sdh_enable_stat_irq(host, DAT_END | RX_OVERRUN | TX_UNDERRUN | DAT_TIME_OUT);
	} else
		sdh_finish_request(host, host->mrq);

	return 1;
}

static int sdh_data_done(struct sdh_host *host, unsigned int stat)
{
	struct mmc_data *data = host->data;

	pr_debug("%s enter stat:0x%x\n", __FUNCTION__, stat);
	if (!data)
		return 0;

	disable_dma(host->dma_ch);
	dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
		     host->dma_dir);

	if (stat & DAT_TIME_OUT)
		data->error = -ETIMEDOUT;
	else if (stat & DAT_CRC_FAIL)
		data->error = -EILSEQ;
	else if (stat & (RX_OVERRUN | TX_UNDERRUN))
		data->error = -EIO;

	if (!data->error)
		data->bytes_xfered = data->blocks * data->blksz;
	else
		data->bytes_xfered = data->blocks * data->blksz - \
				     bfin_read_SDH_DATA_CNT();

	sdh_disable_stat_irq(host, DAT_END | DAT_TIME_OUT | DAT_CRC_FAIL | RX_OVERRUN | TX_UNDERRUN);
	bfin_write_SDH_STATUS_CLR(DAT_END_STAT | DAT_TIMEOUT_STAT | \
			DAT_CRC_FAIL_STAT | DAT_BLK_END_STAT | RX_OVERRUN | TX_UNDERRUN);
	bfin_write_SDH_DATA_CTL(0);
	SSYNC();

	host->data = NULL;
	if (host->mrq->stop) {
		sdh_stop_clock(host);
		sdh_start_cmd(host, host->mrq->stop);
	} else
		sdh_finish_request(host, host->mrq);

	return 1;
}

static void sdh_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct sdh_host *host = mmc_priv(mmc);

	pr_debug("%s enter, mrp:%p, cmd:%p\n", __FUNCTION__, mrq, mrq->cmd);
	WARN_ON(host->mrq != NULL);

	host->mrq = mrq;
	host->data = mrq->data;

	if (mrq->data && mrq->data->flags & MMC_DATA_READ)
		sdh_setup_data(host, mrq->data);

	sdh_start_cmd(host, mrq->cmd);
}

static int sdh_get_ro(struct mmc_host *mmc)
{
	/* Host doesn't support read only detection so assume writeable */
	return 0;
}

static void sdh_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sdh_host *host;
	unsigned long flags;
	u16 clk_ctl = 0;
	u16 pwr_ctl = 0;
	u16 cfg;
	host = mmc_priv(mmc);

	spin_lock_irqsave(&host->lock, flags);
	if (ios->clock) {
		unsigned long clk_div;
		unsigned long sys_clk;
		sys_clk = get_sclk();
		if (sys_clk % (2*ios->clock) == 0)
			clk_div = sys_clk / (2*ios->clock) - 1;
		else
			clk_div = sys_clk / (2*ios->clock);
		if (clk_div > 0xff)
			clk_div = 0xFF;
		clk_ctl |= clk_div & 0xFF;
		clk_ctl |= CLK_E;
		host->clk_div = clk_div;
	} else
		sdh_stop_clock(host);

	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
#ifdef CONFIG_SDH_BFIN_MISSING_CMD_PULLUP_WORKAROUND
		pwr_ctl |= ROD_CTL;
#else
		pwr_ctl |= SD_CMD_OD | ROD_CTL;
#endif

	if (ios->bus_width == MMC_BUS_WIDTH_4) {
		cfg = bfin_read_SDH_CFG();
		cfg &= ~0x80;
		cfg |= 0x40;
		/* Enable 4 bit SDIO */
		cfg |= 0x0c;
		bfin_write_SDH_CFG(cfg);
		clk_ctl |= WIDE_BUS;
	} else {
		cfg = bfin_read_SDH_CFG();
		cfg |= 0x08;
		bfin_write_SDH_CFG(cfg);
	}

	bfin_write_SDH_CLK_CTL(clk_ctl);

	host->power_mode = ios->power_mode;
	if (ios->power_mode == MMC_POWER_ON)
		pwr_ctl |= PWR_ON;

	bfin_write_SDH_PWR_CTL(pwr_ctl);
	SSYNC();

	spin_unlock_irqrestore(&host->lock, flags);

	pr_debug("SDH: clk_div = 0x%x actual clock:%ld expected clock:%d\n",
			host->clk_div, host->clk_div?get_sclk()/(2 * (host->clk_div + 1)):0,
			ios->clock);
}

#ifdef CONFIG_SDH_BFIN_ENABLE_SDIO_IRQ
static void sdh_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	if (enable)
		pr_debug("Enable sdio irq\n");
	else
		pr_debug("Disable sdio irq\n");
}
#endif

static const struct mmc_host_ops sdh_ops = {
	.request	= sdh_request,
	.get_ro		= sdh_get_ro,
	.set_ios	= sdh_set_ios,
#ifdef CONFIG_SDH_BFIN_ENABLE_SDIO_IRQ
	.enable_sdio_irq = sdh_enable_sdio_irq,
#endif
};

static irqreturn_t sdh_dma_irq(int irq, void *devid)
{
	struct sdh_host *host = devid;

	pr_debug("%s enter, irq_stat:0x%04x\n", __FUNCTION__,\
			get_dma_curr_irqstat(host->dma_ch));
	clear_dma_irqstat(host->dma_ch);
	SSYNC();

	return IRQ_HANDLED;
}

static irqreturn_t sdh_stat_irq(int irq, void *devid)
{
	struct sdh_host *host = devid;
	unsigned int status;
	int handled = 0;

	pr_debug("%s enter\n", __FUNCTION__);
	status = bfin_read_SDH_E_STATUS();
	if (status & SD_CARD_DET) {
		mmc_detect_change(host->mmc, 0);
		bfin_write_SDH_E_STATUS(SD_CARD_DET);
		SSYNC();
	}
#ifdef CONFIG_SDH_BFIN_ENABLE_SDIO_IRQ
	if (status & SDIO_INT_DET) {
		mmc_signal_sdio_irq(host->mmc);
		bfin_write_SDH_E_STATUS(SDIO_INT_DET);
		SSYNC();
		handled = 1;
	}
#endif
	status = bfin_read_SDH_STATUS();
	if (status & (CMD_SENT | CMD_RESP_END | CMD_TIME_OUT | CMD_CRC_FAIL)) {
		handled |= sdh_cmd_done(host, status);
		bfin_write_SDH_STATUS_CLR( CMD_SENT_STAT | CMD_RESP_END_STAT | \
				CMD_TIMEOUT_STAT | CMD_CRC_FAIL_STAT);
		SSYNC();
	}

	status = bfin_read_SDH_STATUS();
	if (status & (DAT_END | DAT_TIME_OUT | DAT_CRC_FAIL | RX_OVERRUN | TX_UNDERRUN))
		handled |= sdh_data_done(host, status);

	pr_debug("%s exit\n\n", __FUNCTION__);

	return IRQ_RETVAL(handled);
}

static int proc_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	struct sdh_host *host = data;
	unsigned long cmd = simple_strtoul(buffer, NULL, 16);

	switch (cmd) {
	case 0:
		mmc_detect_change(host->mmc, 0);
		break;
	default:
		printk(KERN_ERR "bfin_sdh: cmd %lu not support\n", cmd);
		break;
	}

	return count;
}

static int sdh_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct sdh_host *host = NULL;
	struct proc_dir_entry *sd_entry;
	struct bfin_sd_host *drv_data = get_sdh_data(pdev);
	int ret;

	if (!drv_data) {
		dev_err(&pdev->dev, "missing platform driver data\n");
		ret = -EINVAL;
		goto out;
	}

	mmc = mmc_alloc_host(sizeof(struct sdh_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	mmc->ops = &sdh_ops;
	mmc->max_phys_segs = NR_SG;
	mmc->max_seg_size = 1 << 16;
	mmc->max_blk_size = 2 << 11;
	mmc->max_blk_count = 2 << 16;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->f_min = get_sclk() >> 9;
	mmc->f_max = get_sclk();
#ifndef CONFIG_SDH_BFIN_ENABLE_SDIO_IRQ
	mmc->caps = MMC_CAP_4_BIT_DATA;
#endif
	host = mmc_priv(mmc);
	host->mmc = mmc;

	spin_lock_init(&host->lock);
	host->irq = drv_data->irq_int0;
	host->dma_ch = drv_data->dma_chan;
	ret = request_irq(host->irq, sdh_stat_irq, 0, "SDH Status IRQ", host);
	if (ret) {
		dev_err(&pdev->dev, "unable to request status irq\n");
		goto out1;
	}

	ret = request_dma(host->dma_ch, DRIVER_NAME "DMA");
	if (ret) {
		dev_err(&pdev->dev, "unable to request DMA channel\n");
		goto out2;
	}

	ret = set_dma_callback(host->dma_ch, sdh_dma_irq, host);
	if (ret) {
		dev_err(&pdev->dev, "unable to request DMA irq\n");
		goto out3;
	}

	host->sg_cpu = dma_alloc_coherent(&pdev->dev, PAGE_SIZE, &host->sg_dma, GFP_KERNEL);
	if (host->sg_cpu == NULL) {
		ret = -ENOMEM;
		goto out3;
	}

	platform_set_drvdata(pdev, mmc);
	mmc_add_host(mmc);
	ret = peripheral_request_list(drv_data->pin_req, DRIVER_NAME);
	if (ret) {
		dev_err(&pdev->dev, "unable to request peripheral pins\n");
		goto out3;
	}

#if defined(CONFIG_BF54x)
	/* Secure Digital Host shares DMA with Nand controller */
	bfin_write_DMAC1_PERIMUX(bfin_read_DMAC1_PERIMUX() | 0x1);
#endif

	bfin_write_SDH_CFG(bfin_read_SDH_CFG() | CLKS_EN);
	SSYNC();

	/* Enable card detect pin */
	bfin_write_SDH_CFG((bfin_read_SDH_CFG() & 0x1F) | 0xa0);
	SSYNC();
	sd_entry = create_proc_entry("driver/sdh", 0600, NULL);
	sd_entry->read_proc = NULL;
	sd_entry->write_proc = proc_write;
	sd_entry->data = host;

	return 0;

out3:
	free_dma(host->dma_ch);
out2:
	free_irq(host->irq, host);
out1:
	mmc_free_host(mmc);
 out:
	return ret;
}

static int sdh_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (mmc) {
		struct sdh_host *host = mmc_priv(mmc);

		mmc_remove_host(mmc);

		sdh_stop_clock(host);
		free_irq(host->irq, host);
		free_dma(host->dma_ch);
		dma_free_coherent(&pdev->dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);

		mmc_free_host(mmc);
	}
	remove_proc_entry("driver/sdh", NULL);

	return 0;
}

#ifdef CONFIG_PM
static int sdh_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc)
		ret = mmc_suspend_host(mmc, state);

	return ret;
}

static int sdh_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc)
		ret = mmc_resume_host(mmc);

	return ret;
}
#else
#define sdh_suspend	NULL
#define sdh_resume	NULL
#endif

static struct platform_driver sdh_driver = {
	.probe		= sdh_probe,
	.remove		= sdh_remove,
	.suspend	= sdh_suspend,
	.resume		= sdh_resume,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};

static int __init sdh_init(void)
{
	return platform_driver_register(&sdh_driver);
}

static void __exit sdh_exit(void)
{
	platform_driver_unregister(&sdh_driver);
}

module_init(sdh_init);
module_exit(sdh_exit);

MODULE_DESCRIPTION("Blackfin Secure Digital Host Driver");
MODULE_LICENSE("GPL");
