/*
 * SPI Flash memory access on BlackFin BF533 based devices
 * 
 * Now the driver is written for the chip ST - M25P64 
 *
 * (C)	2005 Aubrey.Li@analog.com
 * 
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mtd/map.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/flashchip.h>
#include <linux/init.h>

#include <linux/interrupt.h>

#define DEVICE_TYPE_X8  (8 / 8)
#define DEVICE_TYPE_X16 (16 / 8)
#define DEVICE_TYPE_X32 (32 / 8)

struct stm_flash_private {
	int device_type;
	struct flchip chips;
};

struct stm_flash_info {
	const __u16 mfr_id;
	const __u16 dev_id;
	const char *name;
	const u_long size;
	const int numeraseregions;
	const struct mtd_erase_region_info regions[4];
};


/* Application definitions */

#define	NUM_SECTORS 	128	/* number of sectors */
#define SECTOR_SIZE		0x10000
#define NOP_NUM		1000
#define SF_PAGESIZE		256

#define COMMON_SPI_SETTINGS (SPE|MSTR|CPHA|CPOL) /* Settings to the SPI_CTL */
#define TIMOD01 (0x01)		/* stes the SPI to work with core instructions */
#define BAUD_RATE_DIVISOR 2

/* Flash commands */
#define SPI_WREN            (0x06)  /* Set Write Enable Latch */
#define SPI_WRDI            (0x04)  /* Reset Write Enable Latch */
#define SPI_RDSR            (0x05)  /* Read Status Register */
#define SPI_RDID				 (0x9F)  /* Read Identification */
#define SPI_WRSR            (0x01)  /* Write Status Register */
#define SPI_READ            (0x03)  /* Read data from memory */
#define SPI_PP              (0x02)  /* Program Data into memory */
#define SPI_SE              (0xD8)  /* Erase one sector in memory */
#define SPI_BE              (0xC7)  /* Erase all memory */
#define WIP					(0x1)	  /* Check the write in progress bit of the SPI status register */
#define WEL					(0x2)	  /* Check the write enable bit of the SPI status register */

#define TIMEOUT 350000000

static char spi_read_status(void);
static void spi_ready(void);
static void spi_setup( const int spi_setting );
static void spi_off(void);
static void spi_command( const int iCommand );

static void spi_sector_number( unsigned long ulOffset, int *pnSector );
static void spi_erase_block( int nBlock );
static void spi_read_data(  unsigned long ulStart, long lCount,int *pnData  );
static void spi_write_data( unsigned long ulStart, long lCount, int *pnData );
static int spi_wait_status( char Statusbit );
static int spi_wait_WEL(void);

static void spi_command( const int iCommand )
{
	unsigned short dummy;

	/*turns on the SPI in single write mode*/
	spi_setup( (COMMON_SPI_SETTINGS|TIMOD01) );

	/*sends the actual command to the SPI TX register*/
	*pSPI_TDBR = iCommand;
	 __builtin_bfin_ssync();

	/*The SPI status register will be polled to check the SPIF bit*/
	spi_ready();
	
	dummy = *pSPI_RDBR;

	/*The SPI will be turned off*/
	spi_off();

}

static void spi_setup( const int spi_setting )
{
	
#if defined(CONFIG_BLKFIN_CACHE) || defined(CONFIG_BLKFIN_DCACHE)  
    udelay(CONFIG_CCLK_HZ/50000000);
#endif
	/*sets up the PF2 to be the slave select of the SPI */
	*pSPI_FLG = 0xFB04;
	*pSPI_BAUD = BAUD_RATE_DIVISOR;
	*pSPI_CTL = spi_setting;
	 __builtin_bfin_ssync();
}

static void spi_off(void)
{
	
	*pSPI_CTL = 0x0400;	/* disable SPI*/
	*pSPI_FLG = 0;
	*pSPI_BAUD = 0;
	 __builtin_bfin_ssync();
	udelay(CONFIG_CCLK_HZ/50000000);
	
}

static void spi_ready(void)
{
	unsigned short dummyread;
	while( (*pSPI_STAT&TXS));
	while(!(*pSPI_STAT&SPIF));
	while(!(*pSPI_STAT&RXS));
	dummyread = *pSPI_RDBR;			
	
}

static int spi_wait_WEL(void)
{
	int i;
	int results;
	char status_register = 0;
	
		for(i = 0; i < TIMEOUT; i++)
		{
			status_register = spi_read_status();
			if( (status_register & WEL) )
			{
				results = 0;	/* tells us if there was an error erasing flash*/
				break;
			}
			results = -1;	/* Time out error*/
		}
		
	return results;
}

static int spi_wait_status( char Statusbit )
{
	int i;
	int results;
	char status_register = 0xFF;
	
		for(i = 0; i < TIMEOUT; i++)
		{
			status_register = spi_read_status();
			if( !(status_register & Statusbit) )
			{
				results = 0;
				break;
			}
			results = -1;
		}
	return results;
}

static char spi_read_status(void)
{
	char status_register = 0;
	int flags;

	local_irq_save(flags);
	spi_setup( (COMMON_SPI_SETTINGS|TIMOD01) ); /* Turn on the SPI */

	*pSPI_TDBR = SPI_RDSR;			/* send instruction to read status register */
	 __builtin_bfin_ssync();
	spi_ready();						/*wait until the instruction has been sent*/
	*pSPI_TDBR = 0;					/*send dummy to receive the status register*/
	 __builtin_bfin_ssync();
	spi_ready();						/*wait until the data has been sent*/
	status_register = *pSPI_RDBR;	/*read the status register*/
	
	spi_off();							/* Turn off the SPI */
	local_irq_restore(flags);

	return status_register;
}

static void spi_sector_number( unsigned long ulOffset, int *pnSector )
{
	int nSector = 0;
	
	if(ulOffset > (NUM_SECTORS*0x10000 -1)){
		printk(KERN_NOTICE "SPI Flash Error: invalid sector number\n");
		return;
		}
			
	nSector = (int)ulOffset/0x10000;
	*pnSector = nSector;
	
}

static void spi_erase_block( int nBlock )
{
	unsigned long ulSectorOff = 0x0, ShiftValue;
	int flags;

	/* if the block is invalid just return */
	if ( (nBlock < 0) || (nBlock > NUM_SECTORS) )
	{
		printk(KERN_NOTICE "SPI Flash Error: invalid sector number\n");
		return;
	}
	
	/* figure out the offset of the block in flash */
	if ( (nBlock >= 0) && (nBlock < NUM_SECTORS) )
	{
		ulSectorOff = (nBlock * SECTOR_SIZE);
		
	}

	/* A write enable instruction must previously have been executed */
	spi_command(SPI_WREN);
	
	//The status register will be polled to check the write enable latch "WREN"
	if(spi_wait_WEL()!=0){
		printk(KERN_NOTICE "SPI Erase block error\n");
		return;
		}

	local_irq_save(flags);
	/* Turn on the SPI to send single commands */
	spi_setup( (COMMON_SPI_SETTINGS|TIMOD01) );

	/* Send the erase block command to the flash followed by the 24 address 
	 to point to the start of a sector. */
	*pSPI_TDBR = SPI_SE;
	 __builtin_bfin_ssync();
	spi_ready();
	ShiftValue = (ulSectorOff >> 16);	/* Send the highest byte of the 24 bit address at first */
	*pSPI_TDBR = ShiftValue;			
	 __builtin_bfin_ssync();
	spi_ready();							/* Wait until the instruction has been sent */
	ShiftValue = (ulSectorOff >> 8);	/* Send the middle byte of the 24 bit address  at second */
	*pSPI_TDBR = ShiftValue;
	 __builtin_bfin_ssync();
	spi_ready();							/* Wait until the instruction has been sent */
	*pSPI_TDBR = ulSectorOff;			/* Send the lowest byte of the 24 bit address finally */
	 __builtin_bfin_ssync();
	spi_ready();							/* Wait until the instruction has been sent */
	
	spi_off();
	local_irq_restore(flags);
	/* Poll the status register to check the Write in Progress bit
	   Sector erase takes time */
	spi_wait_status(WIP);
	
}

static void spi_read_data(  unsigned long ulStart, long lCount,int *pnData  )
{
	unsigned long ShiftValue;
	char *cnData;
	int i,flags;

	cnData = (char *)pnData; /* Pointer cast to be able to increment byte wise */

	local_irq_save(flags);

	spi_setup( (COMMON_SPI_SETTINGS|TIMOD01) );

	*pSPI_TDBR = SPI_READ;			// Send the read command to SPI device
	 __builtin_bfin_ssync();
	spi_ready();							/* Wait until the instruction has been sent */
	ShiftValue = (ulStart >> 16);	// Send the highest byte of the 24 bit address at first
	*pSPI_TDBR = ShiftValue;		// Send the byte to the SPI device
	 __builtin_bfin_ssync();
	spi_ready();							/* Wait until the instruction has been sent */
	ShiftValue = (ulStart >> 8);	// Send the middle byte of the 24 bit address  at second
	*pSPI_TDBR = ShiftValue;		// Send the byte to the SPI device
	 __builtin_bfin_ssync();
	spi_ready();							/* Wait until the instruction has been sent */
	*pSPI_TDBR = ulStart;			/* Send the lowest byte of the 24 bit address finally	 */
	 __builtin_bfin_ssync();
	spi_ready();							/* Wait until the instruction has been sent */

	/* After the SPI device address has been placed on the MOSI pin the data can be
	   received on the MISO pin. */
	for (i=0; i<lCount; i++)
	{
		*pSPI_TDBR = 0;			/* send dummy */
		 __builtin_bfin_ssync();
		while(!(*pSPI_STAT&RXS));
		*cnData++  = *pSPI_RDBR;	/* read */
		
	}
	
	spi_off();
	local_irq_restore(flags);
	
}

static void spi_write_flash ( unsigned long ulStartAddr, long lTransferCount, int *iDataSource, long *lWriteCount )
{

	unsigned long ulWAddr;
	long lWTransferCount = 0;
	int i,flags;
	char iData;
	unsigned int ctr = SF_PAGESIZE - ( ulStartAddr & (SF_PAGESIZE -1));
	char *temp = (char *)iDataSource;
	int n;

	while(lTransferCount) {
		if(lTransferCount < ctr)
			n = lTransferCount;
		else
			n = ctr;
		/* First, a Write Enable Command must be sent to the SPI. */
		spi_command(SPI_WREN);
	
		/* Second, the SPI Status Register will be tested whether the 
	  	 Write Enable Bit has been set. */
		if(spi_wait_WEL()!=0)
			{
			printk(KERN_NOTICE "SPI Write Time Out\n");
			return ;
			}
		local_irq_save(flags);
		/* Third, the 24 bit address will be shifted out the SPI MOSI bytewise. */
		spi_setup( (COMMON_SPI_SETTINGS|TIMOD01) ); 
		*pSPI_TDBR = SPI_PP;
	 	__builtin_bfin_ssync();
		spi_ready();							/* Wait until the instruction has been sent */
		ulWAddr = (ulStartAddr >> 16);
		*pSPI_TDBR = ulWAddr;
	 	__builtin_bfin_ssync();
		spi_ready();							/* Wait until the instruction has been sent */
		ulWAddr = (ulStartAddr >> 8);
		*pSPI_TDBR = ulWAddr;
		 __builtin_bfin_ssync();
		spi_ready();							/* Wait until the instruction has been sent */
		ulWAddr = ulStartAddr;
		*pSPI_TDBR = ulWAddr;
		 __builtin_bfin_ssync();
		spi_ready();							/* Wait until the instruction has been sent */
		/* Fourth, maximum number of 256 bytes will be taken from the Buffer
	   	and sent to the SPI device.*/
		for (i=0; i < n; i++, lWTransferCount++) {
			iData = *temp;
			*pSPI_TDBR = iData;
			__builtin_bfin_ssync();
			spi_ready();							/* Wait until the instruction has been sent */
			temp++;
		}
		
		spi_off(); 
   		local_irq_restore(flags);

		/* Sixth, the SPI Write in Progress Bit must be toggled to ensure the 
	   	programming is done before start of next transfer. */
		if(spi_wait_status(WIP)!=0){
			printk(KERN_NOTICE "SPI Program Time out!\n");
			return;
			}
		lTransferCount 	-= n;
		ulStartAddr 		+= n;
		ctr = (lTransferCount < SF_PAGESIZE)? lTransferCount : SF_PAGESIZE;
	}
	*lWriteCount = lWTransferCount;
}


static void spi_write_data( unsigned long ulStart, long lCount, int *pnData )
{

	unsigned long ulWStart = ulStart; 
	long lWCount = lCount, lWriteCount;
	long *pnWriteCount = &lWriteCount;
	
	while (lWCount != 0)
	{
		spi_write_flash(ulWStart, lWCount, pnData, pnWriteCount);
		
		/* After each function call of WriteFlash the counter must be adjusted */
		lWCount -= *pnWriteCount;
		
		/* Also, both address pointers must be recalculated. */
		ulWStart += *pnWriteCount;
		pnData += *pnWriteCount/4;
	}

}
static int stm_flash_read(struct mtd_info *, loff_t, size_t, size_t *, unsigned char *);
static int stm_flash_write(struct mtd_info *, loff_t, size_t, size_t *, const unsigned char *);
static void stm_flash_sync(struct mtd_info *);
static int stm_flash_erase(struct mtd_info *, struct erase_info *);
static void stm_flash_destroy(struct mtd_info*);
static struct mtd_info* stm_flash_probe(struct map_info *);
static int stm_flash_suspend(struct mtd_info *mtd);
static void stm_flash_resume(struct mtd_info *mtd);

static struct mtd_chip_driver stm_flash_chipdrv = {
	probe:		stm_flash_probe,
	destroy:	stm_flash_destroy,
	name:		"stm_spi_flash",
	module:		THIS_MODULE
};

static const char im_name[] = "stm_spi_flash";

static int probe_new_chip(struct mtd_info *mtd, __u32 base, 
			  struct flchip *chips,
			  struct stm_flash_private *private,
			  const struct stm_flash_info *table)
{
	__u32 mfr_id, dev_id;
	struct map_info *map = mtd->priv;
	struct stm_flash_private temp;

	temp.device_type = DEVICE_TYPE_X8;
	map->fldrv_priv = &temp;

	spi_setup( (COMMON_SPI_SETTINGS|TIMOD01) ); /* Turn on the SPI */

	*pSPI_TDBR = SPI_RDID;	/* send instruction to read status register */
	 __builtin_bfin_ssync();
	spi_ready();							/* Wait until the instruction has been sent */
	*pSPI_TDBR = 0;			/*send dummy to receive the status register*/
	 __builtin_bfin_ssync();
	spi_ready();							/* Wait until the instruction has been sent */
	mfr_id = *pSPI_RDBR;	/*read the status register*/
	*pSPI_TDBR = 0;			/*send dummy to receive the status register*/
	 __builtin_bfin_ssync();
	spi_ready();							/* Wait until the instruction has been sent */
	dev_id = *pSPI_RDBR;	/*read the status register*/
	spi_off();		/* Turn off the SPI */

	if ((mfr_id == table->mfr_id) &&(dev_id == table->dev_id))
		{
			if (chips)
			{
				chips->start = base;
				chips->state = FL_READY;
				chips->mutex =	&chips->_spinlock;
			}
			printk("%s: Found %ldMiB %s at 0x%08x\n",
				map->name,	(table->size)/(1024*1024),table->name, base);
			
			mtd->size = table->size ;
			mtd->numeraseregions = table->numeraseregions;

		printk("mfr id 0x%02x, dev_id 0x%02x\n", mfr_id, dev_id);			

	}
	
else{
		printk(KERN_DEBUG "%s: unknown flash device at 0x%08x, "
			"mfr id 0x%02x, dev_id 0x%02x\n", map->name,
			base, mfr_id, dev_id);
		map->fldrv_priv = NULL;
		return -1;
	}

	private->device_type = temp.device_type;
	return 0;
}

static struct mtd_info* stm_flash_probe(struct map_info *map)
{
	const struct stm_flash_info table= {
		mfr_id: 0x20,
		dev_id: 0x20,
		name: "ST M25P64",
		size: 0x00800000,
		numeraseregions: 1,
		regions: {
		  { offset: 0x000000, erasesize: 0x10000, numblocks: 128}, 
		}
	};
	struct mtd_info *mtd;
	struct flchip chips;
	struct stm_flash_private temp;
	struct stm_flash_private *private;

	mtd = (struct mtd_info*)kmalloc(sizeof(*mtd), GFP_KERNEL);
	if (!mtd)
	{
		printk(KERN_WARNING "%s: kmalloc failed for info structure\n",
			map->name);
		return NULL;
	}
	memset(mtd, 0, sizeof(*mtd));
	mtd->priv = map;

	memset(&temp, 0, sizeof(temp));
	
	printk("%s: Probing for STM M25P64 compatible flash...\n", map->name);
	
	if (probe_new_chip(mtd, 0, NULL, &temp, &table) == -1)
	{
		printk(KERN_WARNING 
			"%s: Found no STM M25P64 compatible device at "
			"location zero\n", map->name);
		kfree(mtd);

		return NULL;
	}
	chips.start = 0;
	chips.state = FL_READY;
	chips.mutex = &chips._spinlock;

	mtd->eraseregions = kmalloc(sizeof(struct mtd_erase_region_info) *
					mtd->numeraseregions, GFP_KERNEL);

	if (!mtd->eraseregions)
	{
		printk(KERN_WARNING "%s: Failed to allocate memory for "
			"MTD erase region info\n", map->name);
		kfree(mtd);
		map->fldrv_priv = NULL;
		return NULL;
	}

	mtd->eraseregions->offset = table.regions[0].offset;
	mtd->eraseregions->erasesize = table.regions[0].erasesize;
	mtd->eraseregions->numblocks = table.regions[0].numblocks;
	if (mtd->erasesize < 
			mtd->eraseregions->erasesize)
			mtd->erasesize = 
				mtd->eraseregions->erasesize;

	mtd->type = MTD_NORFLASH;
	mtd->flags = MTD_CAP_NORFLASH;
	mtd->name = map->name;
	mtd->erase = stm_flash_erase;
	mtd->read = stm_flash_read;
	mtd->write = stm_flash_write;
	mtd->sync = stm_flash_sync;
	mtd->suspend = stm_flash_suspend;
	mtd->resume = stm_flash_resume;

	private = kmalloc(sizeof(*private) + 
			(sizeof(struct flchip)), GFP_KERNEL);
	if (!private) {
		printk(KERN_WARNING
		       "%s: kmalloc failed for private structure\n", map->name);
		kfree(mtd);
		map->fldrv_priv = NULL;
		return NULL;
	}
	memcpy(private, &temp, sizeof(temp));
	memcpy(&private->chips, &chips, sizeof(struct flchip));
	init_waitqueue_head(&private->chips.wq);
	spin_lock_init(&private->chips._spinlock);

	map->fldrv_priv = private;

	map->fldrv = &stm_flash_chipdrv;
	__module_get(THIS_MODULE);
	return mtd;
}

static void stm_flash_destroy(struct mtd_info *mtd)
{
	struct map_info *map = mtd->priv;
	struct stm_flash_private *private = map->fldrv_priv;
	kfree(private);
}

static void stm_flash_sync(struct mtd_info *mtd)
{
	struct map_info *map = mtd->priv;
	struct stm_flash_private *private = map->fldrv_priv;
	struct flchip *chip;

	DECLARE_WAITQUEUE(wait, current);

	chip = &private->chips;
retry:
	spin_lock_bh(chip->mutex);
	switch (chip->state)
	{
	case FL_READY:
	case FL_STATUS:
	case FL_CFI_QUERY:
	case FL_JEDEC_QUERY:
		chip->oldstate = chip->state;
		chip->state = FL_SYNCING;
	case FL_SYNCING:
		spin_unlock_bh(chip->mutex);
		break;
	default:
		/* Not an idle state */
		add_wait_queue(&chip->wq, &wait);
		spin_unlock_bh(chip->mutex);
		schedule();
		remove_wait_queue(&chip->wq, &wait);
		goto retry;
	}

	/* Unlock the chips again */
	chip = &private->chips;
	spin_lock_bh(chip->mutex);
	if (chip->state == FL_SYNCING) 
	{
		chip->state = chip->oldstate;
		wake_up(&chip->wq);
	}
	spin_unlock_bh(chip->mutex);
}

static int read_one_chip(struct map_info *map, struct flchip *chip, 
			loff_t addr, size_t len, unsigned char *buf)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long timeo = jiffies + HZ;

retry:
	spin_lock_bh(chip->mutex);

	if (chip->state != FL_READY)
	{
		printk(KERN_INFO "%s: waiting for chip to read, state = %d\n",
			map->name, chip->state);
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&chip->wq, &wait);

		spin_unlock_bh(chip->mutex);

		schedule();
		remove_wait_queue(&chip->wq, &wait);

		if (signal_pending(current))
			return -EINTR;

		timeo = jiffies + HZ;

		goto retry;
	}

	addr += chip->start;

	chip->state = FL_READY;

	map->copy_from(map, buf, addr, len);

	wake_up(&chip->wq);
	spin_unlock_bh(chip->mutex);

	return 0;
}

static int stm_flash_read(struct mtd_info *mtd, loff_t from, size_t len,
			  size_t *retlen, unsigned char *buf)
{
	struct map_info *map = mtd->priv;
	struct stm_flash_private *private = map->fldrv_priv;
	unsigned long offset;
	int ret = 0;

	if ((from + len) > mtd->size)
	{
		printk(KERN_WARNING "%s: read request past end of device "
			"(0x%lx)\n", map->name, (unsigned long)from + len);
		return -EINVAL;
	}

	offset = from;
	*retlen = 0;

	ret = read_one_chip(map, &private->chips, offset,
				len, buf);

	*retlen += len;

	return ret;
}

static int stm_flash_write(struct mtd_info *mtd, loff_t to, size_t len,
			   size_t *retlen, const unsigned char *buf)
{
	int i;
	unsigned char temp;
	unsigned char value;
	int count = 0;
	*retlen = 0;
	if(!len)
		return 0;
	spi_write_data(to, len, (int *)buf);
	*retlen += len;

	return 0;
}

static int stm_flash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	unsigned long addr, len;
	int num;
	int i, first;
	int start_block, end_block;
	struct mtd_erase_region_info *regions = mtd->eraseregions;

	if (instr->addr > mtd->size)
		return -EINVAL;

	if ((instr->len + instr->addr) > mtd->size)
		return -EINVAL;

	/*
	 * Check that both start and end of the requested erase are aligned
	 * with the erasesize at the appropriate addresses.
	 */
	i = 0;

	/*
	 * Skip all erase regions which are ended before the start of the
	 * requested erase. Actually, to save on the calculations, we skip
	 * to the first erase region which starts after the start of the 
	 * requested erase, and then go back one.
	 */
	while ((i < mtd->numeraseregions) &&
	       (instr->addr >= regions[i].offset))
		i++;
	i--;

	/*
	 * OK. Now i is pointing at the erase region in which this erase 
	 * request starts. Check the start of the requested erase range
	 * is aligned with the erase size which is in effect here.
	 */
	if (instr->addr & (regions[i].erasesize -1))
		return -EINVAL;

	/*
	 * Remember the erase region we start on.
	 */
	first = i;

	/*
	 * Next, theck that the end of the requested erase is aligned with
	 * the erase region at that address.
	 */
	while ((i < mtd->numeraseregions) &&
	       ((instr->addr + instr->len) >= regions[i].offset))
		i++;
	i--;

	if ((instr->addr + instr->len) & (regions[i].erasesize-1))
		return -EINVAL;

	addr = instr->addr;
	len = instr->len;
	
	/* Get the start block number */
	spi_sector_number(addr, &start_block);
	/* Get the end block number */
	spi_sector_number(addr + len -1, &end_block);

	for(num = start_block;num<=end_block;num++)
		spi_erase_block(num);
	
	instr->state = MTD_ERASE_DONE;
	/*if (instr->callback)
		instr->callback(instr);
	*/
	mtd_erase_callback(instr);

	return 0;
}

static int stm_flash_suspend(struct mtd_info *mtd)
{
	printk("stm_flash_suspend(): not implemented!\n");
	        return -EINVAL;
}

static void stm_flash_resume(struct mtd_info *mtd)
{
	printk("stm_flash_resume(): not implemented!\n");
}

int __init stm_flash_init(void)
{
	register_mtd_chip_driver(&stm_flash_chipdrv);
	return 0;
}

void __exit stm_flash_exit(void)
{
	unregister_mtd_chip_driver(&stm_flash_chipdrv);
}

module_init(stm_flash_init);
module_exit(stm_flash_exit);
