/*
 * File:         arch/blackfin/mm/blackfin_sram.c
 * Based on:
 * Author:
 * Created:
 * Description:  SRAM driver for Blackfin ADSP-BF533
 *
 * Rev:          $Id$
 *
 * Modified:
 *               Copyright 2004-2005 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.
 * If not, write to the Free Software Foundation,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/autoconf.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/rtc.h>
#include <asm/blackfin.h>

spinlock_t l1sram_lock, l1_data_A_sram_lock;

#define L1_MAX_PIECE        16

#define SRAM_SLT_NULL      0
#define SRAM_SLT_FREE      1
#define SRAM_SLT_ALLOCATED 2

void l1sram_init(void);
void l1_data_A_sram_init(void);

/* the data structure for L1 scratchpad and DATA SRAM */
struct l1_sram_piece {
	unsigned long paddr;
	int size;
	int flag;
};

struct l1_sram_piece l1_ssram[L1_MAX_PIECE];

struct l1_sram_piece l1_data_A_sram[L1_MAX_PIECE];

#if 0 != L1_DATA_B_LENGTH
struct l1_sram_piece l1_data_B_sram[L1_MAX_PIECE];
#endif

/* L1 Scratchpad SRAM initialization function */
void l1sram_init(void)
{
	printk(KERN_INFO "Blackfin Scratchpad data SRAM: %d KB\n",
	       L1_SCRATCH_LENGTH >> 10);

	memset((void *)&l1_ssram, 0, sizeof(l1_ssram));
	l1_ssram[0].paddr = L1_SCRATCH_START;
	l1_ssram[0].size = L1_SCRATCH_LENGTH;
	l1_ssram[0].flag = SRAM_SLT_FREE;

	/* mutex initialize */
	spin_lock_init(&l1sram_lock);
}

void l1_data_A_sram_init(void)
{
	memset((void *)&l1_data_A_sram, 0, sizeof(l1_data_A_sram));
#if 0 != L1_DATA_A_LENGTH
	printk(KERN_INFO "Blackfin DATA_A SRAM: %d KB\n", L1_DATA_A_LENGTH >> 10);

	l1_data_A_sram[0].paddr = L1_DATA_A_START + (_ebss_l1 - _sdata_l1);
	l1_data_A_sram[0].size = L1_DATA_A_LENGTH - (_ebss_l1 - _sdata_l1);
	l1_data_A_sram[0].flag = SRAM_SLT_FREE;
#endif
#if 0 != L1_DATA_B_LENGTH
	printk(KERN_INFO "Blackfin DATA_B SRAM: %d KB\n", L1_DATA_B_LENGTH >> 10);

	memset((void *)&l1_data_B_sram, 0, sizeof(l1_data_B_sram));
	l1_data_B_sram[0].paddr = L1_DATA_B_START;
	l1_data_B_sram[0].size = L1_DATA_B_LENGTH;
	l1_data_B_sram[0].flag = SRAM_SLT_FREE;
#endif

	/* mutex initialize */
	spin_lock_init(&l1_data_A_sram_lock);
}

/* L1 memory allocate function */
static unsigned long l1_sram_alloc(unsigned long size,
				   struct l1_sram_piece *pfree, int count)
{
	int i, index = 0;
	unsigned long addr = 0;

	if (size <= 0)
		return 0;

	/* Align the size */
	size = (size + 3) & ~3;

	/* not use the good method to match the best slot !!! */
	/* search an available memeory slot */
	for (i = 0; i < count; i++) {
		if ((pfree[i].flag == SRAM_SLT_FREE) && (pfree[i].size >= size)) {
			addr = pfree[i].paddr;
			pfree[i].flag = SRAM_SLT_ALLOCATED;
			index = i;
			break;
		}
	}
	if (i >= count)
		return 0;

	/* updated the NULL memeory slot !!! */
	if (pfree[i].size > size) {
		for (i = 0; i < count; i++) {
			if (pfree[i].flag == SRAM_SLT_NULL) {
				pfree[i].flag = SRAM_SLT_FREE;
				pfree[i].paddr = addr + size;
				pfree[i].size = pfree[index].size - size;
				pfree[index].size = size;
				break;
			}
		}
	}

	memset((void *)addr, 0, size);
	return addr;
}

/* L1 memory free function */
static int l1_sram_free(unsigned long addr,
			struct l1_sram_piece *pfree, int count)
{
	int i, index = 0;

	/* search the relevant memory slot */
	for (i = 0; i < count; i++) {
		if (pfree[i].paddr == addr) {
			if (pfree[i].flag != SRAM_SLT_ALLOCATED) {
				/* error log */
				return -1;
			}
			index = i;
			break;
		}
	}
	if (i >= count)
		return -1;

	pfree[index].flag = SRAM_SLT_FREE;

	/* link the next address slot */
	for (i = 0; i < count; i++) {
		if (((pfree[index].paddr + pfree[index].size) == pfree[i].paddr)
		    && (pfree[i].flag == SRAM_SLT_FREE)) {
			pfree[i].flag = SRAM_SLT_NULL;
			pfree[index].size += pfree[i].size;
			pfree[index].flag = SRAM_SLT_FREE;
			break;
		}
	}

	/* link the last address slot */
	for (i = 0; i < count; i++) {
		if (((pfree[i].paddr + pfree[i].size) == pfree[index].paddr) &&
		    (pfree[i].flag == SRAM_SLT_FREE)) {
			pfree[index].flag = SRAM_SLT_NULL;
			pfree[i].size += pfree[index].size;
			break;
		}
	}

	return 0;
}

unsigned long l1_data_A_sram_alloc(unsigned long size)
{
	unsigned flags;
	unsigned long addr;

	/* add mutex operation */
	spin_lock_irqsave(&l1_data_A_sram_lock, flags);

	addr = l1_sram_alloc(size, l1_data_A_sram, ARRAY_SIZE(l1_data_A_sram));

#if 0 != L1_DATA_B_LENGTH
	if (!addr)
		addr = l1_sram_alloc(size,
				     l1_data_B_sram,
				     ARRAY_SIZE(l1_data_B_sram));
#endif
	/* add mutex operation */
	spin_unlock_irqrestore(&l1_data_A_sram_lock, flags);

	//printk ("Allocated address in l1sram_alloc is 0x%lx+0x%lx\n",addr,size);
	return addr;
}

int l1_data_A_sram_free(unsigned long addr)
{
	unsigned flags;
	int ret;

	/* add mutex operation */
	spin_lock_irqsave(&l1_data_A_sram_lock, flags);

#if 0 != L1_DATA_B_LENGTH
	if (L1_DATA_B_START == (addr & ~0xffff))
		ret = l1_sram_free(addr,
				   l1_data_B_sram, ARRAY_SIZE(l1_data_B_sram));
	else
#endif
		ret = l1_sram_free(addr,
				   l1_data_A_sram, ARRAY_SIZE(l1_data_A_sram));

	/* add mutex operation */
	spin_unlock_irqrestore(&l1_data_A_sram_lock, flags);

	return ret;
}

#if 0 != L1_DATA_B_LENGTH
unsigned long l1_data_B_sram_alloc(unsigned long size)
{
	unsigned flags;
	unsigned long addr;

	/* add mutex operation */
	spin_lock_irqsave(&l1_data_A_sram_lock, flags);

	addr = l1_sram_alloc(size, l1_data_B_sram, ARRAY_SIZE(l1_data_B_sram));

	/* add mutex operation */
	spin_unlock_irqrestore(&l1_data_A_sram_lock, flags);

	//printk ("Allocated address in l1sram_alloc is 0x%lx+0x%lx\n",addr,size);
	return addr;
}

int l1_data_B_sram_free(unsigned long addr)
{
	unsigned flags;
	int ret;

	/* add mutex operation */
	spin_lock_irqsave(&l1_data_A_sram_lock, flags);

	ret = l1_sram_free(addr, l1_data_B_sram, ARRAY_SIZE(l1_data_B_sram));

	/* add mutex operation */
	spin_unlock_irqrestore(&l1_data_A_sram_lock, flags);

	return ret;
}
#endif

/* L1 Scratchpad memory allocate function */
unsigned long l1sram_alloc(unsigned long size)
{
	unsigned flags;
	unsigned long addr;

	/* add mutex operation */
	spin_lock_irqsave(&l1sram_lock, flags);

	addr = l1_sram_alloc(size, l1_ssram, ARRAY_SIZE(l1_ssram));

	/* add mutex operation */
	spin_unlock_irqrestore(&l1sram_lock, flags);

	return addr;
}

/* L1 Scratchpad memory free function */
int l1sram_free(unsigned long addr)
{
	unsigned flags;
	int ret;

	/* add mutex operation */
	spin_lock_irqsave(&l1sram_lock, flags);

	ret = l1_sram_free(addr, l1_ssram, ARRAY_SIZE(l1_ssram));

	/* add mutex operation */
	spin_unlock_irqrestore(&l1sram_lock, flags);

	return ret;
}

EXPORT_SYMBOL(l1sram_alloc);
EXPORT_SYMBOL(l1sram_free);
EXPORT_SYMBOL(l1_data_A_sram_alloc);
EXPORT_SYMBOL(l1_data_A_sram_free);
