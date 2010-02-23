/*
 * File:         arch/blackfin/mach-bf537/boards/stamp.c
 * Based on:     arch/blackfin/mach-bf533/boards/ezkit.c
 * Author:       Markus Berner, Samuel Zahnd, Beat Kueng
 *
 * Created:
 * Description:
 *
 * Modified:
 *               Copyright 2005 National ICT Australia (NICTA)
 *               Copyright 2004-2006 Analog Devices Inc.
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

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/plat-ram.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#if defined(CONFIG_USB_ISP1362_HCD) || defined(CONFIG_USB_ISP1362_HCD_MODULE)
#include <linux/usb/isp1362.h>
#endif
#include <linux/ata_platform.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <asm/dma.h>
#include <linux/i2c.h>
#include <linux/usb/sl811.h>
#include <linux/spi/mmc_spi.h>
#include <asm/dma.h>
#include <asm/bfin5xx_spi.h>
#include <asm/reboot.h>
#include <asm/portmux.h>
#include <asm/dpmc.h>
/*
 * Name the Board for the /proc/cpuinfo
 */
const char bfin_board_name[] = "BF537-leanXcam";

/*
 *  Driver needs to know address, irq and flag pin.
 */

#if defined(CONFIG_RTC_DRV_BFIN) || defined(CONFIG_RTC_DRV_BFIN_MODULE)
static struct platform_device rtc_device = {
	.name = "rtc-bfin",
	.id   = -1,
};
#endif

#if defined(CONFIG_BFIN_MAC) || defined(CONFIG_BFIN_MAC_MODULE)
static struct platform_device bfin_mii_bus = {
	.name = "bfin_mii_bus",
};

static struct platform_device bfin_mac_device = {
	.name = "bfin_mac",
	.dev.platform_data = &bfin_mii_bus,
};
#endif


static struct resource bfin_gpios_resources = {
	.start = 0,
	.end   = MAX_BLACKFIN_GPIOS - 1,
	.flags = IORESOURCE_IRQ,
};

static struct platform_device bfin_gpios_device = {
	.name = "simple-gpio",
	.id = -1,
	.num_resources = 1,
	.resource = &bfin_gpios_resources,
};

#if defined(CONFIG_SPI_BFIN) || defined(CONFIG_SPI_BFIN_MODULE)

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
#define MMC_SPI_CARD_DETECT_INT IRQ_PG15 //IRQ_PF5

static int bfin_mmc_spi_init(struct device *dev,
	irqreturn_t (*detect_int)(int, void *), void *data)
{
	return request_irq(MMC_SPI_CARD_DETECT_INT, detect_int,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "mmc-spi-detect", data);
}

static void bfin_mmc_spi_exit(struct device *dev, void *data)
{
	free_irq(MMC_SPI_CARD_DETECT_INT, data);
}

static struct mmc_spi_platform_data bfin_mmc_spi_pdata = {
	.init = bfin_mmc_spi_init,
	.exit = bfin_mmc_spi_exit,
	.detect_delay = 500, /* msecs */
};

static struct bfin5xx_spi_chip  mmc_spi_chip_info = {
	.enable_dma = 0,
	.bits_per_word = 8,
};
#endif

#if defined(CONFIG_MTD_DATAFLASH) \
	|| defined(CONFIG_MTD_DATAFLASH_MODULE)

static struct mtd_partition bfin_spi_bootflash_partitions[] = {
	{
		.name = "bootloader, 128k",
		.size = 0x00020000,
		.offset = 0,
		.mask_flags = 0
	}, {
		.name = "bootloader environment, 16k",
		.size =   0x00004000,
		.offset = 0x00020000,
		.mask_flags = 0
	}, {
		.name = "bootloader environment redundant, 16k",
		.size =   0x00004000,
		.offset = 0x00024000,
		.mask_flags = 0
	}, {
		.name = "linux, 4M",
		.size =   0x003d8000,
		.offset = 0x00028000,
		.mask_flags = 0
        }
};

static struct flash_platform_data bfin_spi_bootflash_data = {
	.name = "Boot Flash",
	.parts = bfin_spi_bootflash_partitions,
	.nr_parts = ARRAY_SIZE(bfin_spi_bootflash_partitions),
	.type = "AT45DB321D",
};

static struct mtd_partition bfin_spi_appflash_partitions[] = {
	{
		.name = "JFFS2 Application Partition",
		.size = 0x00400000,
		.offset = 0,
		.mask_flags = 0
	}
};

static struct flash_platform_data bfin_spi_appflash_data = {
	.name = "Application Flash",
	.parts = bfin_spi_appflash_partitions,
	.nr_parts = ARRAY_SIZE(bfin_spi_appflash_partitions),
	.type = "AT45DB321D",
};

/* SPI flash chip (AT45DB321D) */
static struct bfin5xx_spi_chip spi_flash_chip_info = {
	.enable_dma = 0,         /* use dma transfer with this chip*/
	.bits_per_word = 8,
};
#endif

static struct spi_board_info bfin_spi_board_info[] __initdata = {

#if defined(CONFIG_MTD_DATAFLASH) \
	|| defined(CONFIG_MTD_DATAFLASH_MODULE)
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "mtd_dataflash", /* Name of spi_driver for this device */
		.max_speed_hz = 25000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, /* Framework bus number */
		.chip_select = 1, /* Framework chip select. On leanXcam it is SPISSEL1*/
		.platform_data = &bfin_spi_bootflash_data,
		.controller_data = &spi_flash_chip_info,
		.mode = SPI_MODE_3,
	},
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "mtd_dataflash", /* Name of spi_driver for this device */
		.max_speed_hz = 25000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, /* Framework bus number */
		.chip_select = 5, /* Framework chip select. On leanXcam it is SPISSEL5*/
		.platform_data = &bfin_spi_appflash_data,
		.controller_data = &spi_flash_chip_info,
		.mode = SPI_MODE_3,
	},
#endif // CONFIG_MTD_DATAFLASH

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	{
		.modalias = "mmc_spi",
		.max_speed_hz = 20000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 2,
		.platform_data = &bfin_mmc_spi_pdata,
		.controller_data = &mmc_spi_chip_info,
		.mode = SPI_MODE_3,
	},
#endif

};

/* SPI controller data */
static struct bfin5xx_spi_master bfin_spi0_info = {
	.num_chipselect = 8,
	.enable_dma = 1,  /* master has the ability to do dma transfer */
	.pin_req = {P_SPI0_SCK, P_SPI0_MISO, P_SPI0_MOSI, 0},
};

/* SPI (0) */
static struct resource bfin_spi0_resource[] = {
	[0] = {
		.start = SPI0_REGBASE,
		.end   = SPI0_REGBASE + 0xFF,
		.flags = IORESOURCE_MEM,
		},
	[1] = {
		.start = CH_SPI,
		.end   = CH_SPI,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device bfin_spi0_device = {
	.name = "bfin-spi",
	.id = 0, /* Bus number */
	.num_resources = ARRAY_SIZE(bfin_spi0_resource),
	.resource = bfin_spi0_resource,
	.dev = {
		.platform_data = &bfin_spi0_info, /* Passed to driver */
	},
};
#endif  //CONFIG_SPI_BFIN


#if defined(CONFIG_FB_BF537_LQ035) || defined(CONFIG_FB_BF537_LQ035_MODULE)
static struct platform_device bfin_fb_device = {
	.name = "bf537-lq035",
};
#endif

#if defined(CONFIG_FB_BFIN_7393) || defined(CONFIG_FB_BFIN_7393_MODULE)
static struct platform_device bfin_fb_adv7393_device = {
	.name = "bfin-adv7393",
};
#endif

#if defined(CONFIG_SERIAL_BFIN) || defined(CONFIG_SERIAL_BFIN_MODULE)
static struct resource bfin_uart_resources[] = {
	{
		.start = 0xFFC00400,
		.end = 0xFFC004FF,
		.flags = IORESOURCE_MEM,
	}, {
		.start = 0xFFC02000,
		.end = 0xFFC020FF,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device bfin_uart_device = {
	.name = "bfin-uart",
	.id = 1,
	.num_resources = ARRAY_SIZE(bfin_uart_resources),
	.resource = bfin_uart_resources,
};
#endif

#if defined(CONFIG_I2C_BLACKFIN_TWI) || defined(CONFIG_I2C_BLACKFIN_TWI_MODULE)
static struct resource bfin_twi0_resource[] = {
	[0] = {
		.start = 0xFFC01400,
		.end   = 0xFFC014FF,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_TWI,
		.end   = IRQ_TWI,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device i2c_bfin_twi_device = {
	.name = "i2c-bfin-twi",
	.id = 0,
	.num_resources = ARRAY_SIZE(bfin_twi0_resource),
	.resource = bfin_twi0_resource,
};
#endif

#if defined(CONFIG_SERIAL_BFIN_SPORT) || defined(CONFIG_SERIAL_BFIN_SPORT_MODULE)
static struct platform_device bfin_sport0_uart_device = {
	.name = "bfin-sport-uart",
	.id = 0,
};

static struct platform_device bfin_sport1_uart_device = {
	.name = "bfin-sport-uart",
	.id = 1,
};
#endif


static struct platform_device *stamp_devices[] __initdata = {

#if defined(CONFIG_RTC_DRV_BFIN) || defined(CONFIG_RTC_DRV_BFIN_MODULE)
	&rtc_device,
#endif

#if defined(CONFIG_BFIN_MAC) || defined(CONFIG_BFIN_MAC_MODULE)
	&bfin_mii_bus,
	&bfin_mac_device,
#endif

#if defined(CONFIG_SPI_BFIN) || defined(CONFIG_SPI_BFIN_MODULE)
	&bfin_spi0_device,
#endif

#if defined(CONFIG_FB_BF537_LQ035) || defined(CONFIG_FB_BF537_LQ035_MODULE)
	&bfin_fb_device,
#endif

#if defined(CONFIG_FB_BFIN_7393) || defined(CONFIG_FB_BFIN_7393_MODULE)
	&bfin_fb_adv7393_device,
#endif

#if defined(CONFIG_SERIAL_BFIN) || defined(CONFIG_SERIAL_BFIN_MODULE)
	&bfin_uart_device,
#endif

#if defined(CONFIG_I2C_BLACKFIN_TWI) || defined(CONFIG_I2C_BLACKFIN_TWI_MODULE)
	&i2c_bfin_twi_device,
#endif

#if defined(CONFIG_SERIAL_BFIN_SPORT) || defined(CONFIG_SERIAL_BFIN_SPORT_MODULE)
	&bfin_sport0_uart_device,
	&bfin_sport1_uart_device,
#endif
	&bfin_gpios_device,
};

static int __init stamp_init(void)
{

	printk(KERN_INFO "%s(): registering device resources\n", __FUNCTION__);
	platform_add_devices(stamp_devices, ARRAY_SIZE(stamp_devices));
#if defined(CONFIG_SPI_BFIN) || defined(CONFIG_SPI_BFIN_MODULE)
	spi_register_board_info(bfin_spi_board_info,
				ARRAY_SIZE(bfin_spi_board_info));
#endif

#if defined(CONFIG_PATA_PLATFORM) || defined(CONFIG_PATA_PLATFORM_MODULE)
	irq_desc[PATA_INT].status |= IRQ_NOAUTOEN;
#endif
	return 0;
}

arch_initcall(stamp_init);

void native_machine_restart(char *cmd)
{
	if ((bfin_read_SYSCR() & 0x7) == 0x3)
		bfin_reset_boot_spi_cs(P_DEFAULT_BOOT_SPI_CS); //bfin_gpio_reset_spi0_ssel1();/* workaround reboot hang when booting from SPI  (not used anymore) */
}

/*
 * Currently the MAC address is saved in Flash by U-Boot
 */
void bfin_get_ether_addr(char *addr)
{
  addr[0] = 0x00;
  addr[1] = 0x20;
  addr[2] = 0xE3;
  addr[3] = 0x23;
  addr[4] = 0x00;
  addr[5] = 0x00;

  printk(KERN_WARNING "Using default MAC address of %x:%x:%x:%x:%x:%x!\n",
	       addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}
EXPORT_SYMBOL(bfin_get_ether_addr);
