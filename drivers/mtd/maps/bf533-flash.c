/*
 * Flash memory access on BlackFin BF533 based devices
 * 
 * (C) 2000 Nicolas Pitre <nico@cam.org>
 * (C) 2004 LG Soft India
 * 
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/board/cdefBF533.h>

#ifndef CONFIG_BFIN
#error This is for BlackFin BF533 boards only
#endif

#define WINDOW_ADDR 0x20000000

#ifdef CONFIG_BLKFIN_STAMP
struct flash_save {
    u16 dir;
    u16 maska_d;
    u16 maskb_d;
    u16 inen;
    u16 flag_d;
    
    u32 ambctl0;
    u32 ambctl1;

    unsigned long flags;
} ;

static inline void switch_to_flash(struct flash_save *save)
{
	local_irq_save(save->flags);

	save->dir	= *pFIO_DIR;
	save->flag_d	= *pFIO_FLAG_D;
	save->maska_d	= *pFIO_MASKA_D;
	save->maskb_d	= *pFIO_MASKB_D;
	save->inen	= *pFIO_INEN;
	save->ambctl0	= *pEBIU_AMBCTL0;
	save->ambctl1	= *pEBIU_AMBCTL1;

	*pFIO_DIR 	|= PF1 | PF0;
	*pFIO_FLAG_D	&= ~(PF1 | PF0);

	*pFIO_MASKA_D	= 0;
	*pFIO_MASKB_D	= 0;
	*pFIO_INEN	= 0;

	asm("ssync;");

	*pEBIU_AMGCTL = AMGCTLVAL;      /*AMGCTL*/
	asm("ssync;");

	*pEBIU_AMBCTL0 = AMBCTL0VAL;
	asm("ssync;");
	*pEBIU_AMBCTL1 = AMBCTL1VAL;
	asm("ssync;");
}


static inline void switch_back(struct flash_save *save)
{
	*pEBIU_AMBCTL0	= save->ambctl0;
	asm("ssync;");
	*pEBIU_AMBCTL1	= save->ambctl1;
	asm("ssync;");

	*pFIO_DIR 	= save->dir;
	*pFIO_FLAG_D 	= save->flag_d;
	*pFIO_MASKA_D 	= save->maska_d;
	*pFIO_MASKB_D 	= save->maskb_d;
	*pFIO_INEN 	= save->inen;

	local_irq_restore(save->flags);
}
#endif

volatile unsigned short *FLASH_Base = (unsigned short *) 0x20000000;
unsigned volatile long *FB = (unsigned long *) 0x20000002;

static map_word bf533_read(struct map_info *map, unsigned long ofs)
{
	int nValue = 0x0;
	map_word test;
#ifdef CONFIG_BLKFIN_STAMP
	struct flash_save save;
	unsigned long offaddr = (0x20000000 + ofs);

	switch_to_flash(&save);
	asm("ssync;");
        nValue = *(volatile unsigned short *) offaddr;
        asm("ssync;");
	switch_back(&save);
#endif

#ifdef CONFIG_EZKIT
	__asm__ __volatile__ (
		"p2.l = 0x0000; \n\t"
		"p2.h = 0x2000; \n\t"
		"r3 = %1; \n\t"
		"r2 = p2; \n\t"
		"r2 = r2 + r3; \n\t"
		"p2 = r2; \n\t"
		/* The actual thing */
		"ssync; \n\t"
		"%0 = w[p2] (z); \n\t"
		"ssync; \n\t"
		: "=d" (nValue)
		: "d" (ofs));
#endif
	
	/*return (__u16)nValue;*/
	test.x[0]=(__u16)nValue;
	return test;	
}

static void bf533_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
	unsigned long i;
	map_word test;

	for (i = 0; i < len; i += 2)	{
		/* *((u16*)(to + i)) = bf533_read(map, from + i);*/
		test = bf533_read(map,from+i);
		*((u16*)(to + i)) = test.x[0];
	}
	if (len & 0x01) {
		/* *((u8*)(to + (i-1))) = (u8)(bf533_read(map, from + i) >> 8); */
		test = bf533_read(map, from + i);
		test.x[0] = (u8)(test.x[0] >>8);
		*((u8*)(to + (i-1))) = test.x[0];
	}
}

static void bf533_write(struct map_info *map, map_word d1, unsigned long ofs)
{

	__u16 d;
#ifdef CONFIG_BLKFIN_STAMP
	struct flash_save save;
#endif

	d = (__u16)d1.x[0];	


#ifdef CONFIG_BLKFIN_STAMP
	switch_to_flash(&save);
	/* asm("ssync;"); */
	if((ofs == 0x555) || (ofs == 0x2AA)) {
		FLASH_Base[ofs] = d;
		asm("ssync;");
	} else {
		*(volatile unsigned short *) (0x20000000 + ofs) = d;		
		asm("ssync;");
	}
        /* asm("ssync;"); */
	switch_back(&save);
#endif

#ifdef CONFIG_EZKIT

	__asm__ __volatile__ (
		"p2.l = 0x0000; \n\t"
		"p2.h = 0x2000; \n\t"
		"r3 = %1; \n\t"
		"r2 = p2; \n\t"
		"r2 = r2 + r3; \n\t"
		"p2 = r2; \n\t"
		/* The actual thing */
		"ssync; \n\t"
		"w[p2] = %0; \n\t"
		"ssync; \n\t"
		: 
		: "d" (d), "d" (ofs));
#endif
}

static void bf533_copy_to(struct map_info *map, unsigned long to, const void *from, ssize_t len)
{
#ifdef CONFIG_BLKFIN_STAMP
	struct flash_save save;
	switch_to_flash(&save);
#endif
	memcpy((void *)(WINDOW_ADDR + to), from, len);
#ifdef CONFIG_BLKFIN_STAMP
	switch_back(&save);
#endif
}

static struct map_info bf533_map = {
	name:    	"BF533 flash",
	0,
	0,
	0,
	NULL,		
	read:		bf533_read,
	copy_from:	bf533_copy_from,
	write:		bf533_write,
	copy_to:	bf533_copy_to
};


/*
 * Here are partition information for all known BlackFin-based devices.
 * See include/linux/mtd/partitions.h for definition of the mtd_partition
 * structure.
 * 
 * The *_max_flash_size is the maximum possible mapped flash size which
 * is not necessarily the actual flash size.  It must correspond to the 
 * value specified in the mapping definition defined by the
 * "struct map_desc *_io_desc" for the corresponding machine.
 */

#ifdef CONFIG_EZKIT
static unsigned long bf533_max_flash_size = 0x00200000;
#endif
#ifdef CONFIG_BLKFIN_STAMP
static unsigned long bf533_max_flash_size = 0x00400000;
#endif

static struct mtd_partition bf533_partitions[] = {
	{
		name: "bootloader",
		size: 0x00100000,
		offset: 0,
		mask_flags: MTD_CAP_ROM
	},{
		name: "File system image",
		size: 0x100000,
		offset: 0x100000
	},{
		name: "64K area ;)", 
		size: 0x10000,
		offset: 0x00300000,
	}	
};

#define NB_OF(x)  (sizeof(x)/sizeof(x[0]))


static struct mtd_info *mymtd;

int __init bf533_mtd_init(void)
{
	struct mtd_partition *parts;
	int nb_parts = 0;
	char *part_type;

	bf533_map.bankwidth = 2;
	bf533_map.size = bf533_max_flash_size;

	printk(KERN_NOTICE "BF533 flash: probing %d-bit flash bus\n", bf533_map.bankwidth*8);
	mymtd = do_map_probe("stm_flash", &bf533_map);
	if (!mymtd)
		return -ENXIO;

	/*
	 * Static partition definition selection
	 */
	part_type = "static";
#ifdef CONFIG_BFIN
	parts = bf533_partitions;
	nb_parts = NB_OF(bf533_partitions);
#endif

	if (nb_parts == 0) {
		printk(KERN_NOTICE "BF533 flash: no partition info available, registering whole flash at once\n");
		add_mtd_device(mymtd);
	} else {
		printk(KERN_NOTICE "Using %s partition definition\n", part_type);
		add_mtd_partitions(mymtd, parts, nb_parts);
	}
	return 0;
}

static void __exit bf533_mtd_cleanup(void)
{
	if (mymtd) {
		del_mtd_partitions(mymtd);
		map_destroy(mymtd);
	}
}

module_init(bf533_mtd_init);
module_exit(bf533_mtd_cleanup);
