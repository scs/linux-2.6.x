/*
 * File:         arch/blackfin/mm/kmap.c
 * Based on:     arch/m68knommu/mm/kmap.c
 * Author:       
 *              
 * Created:    
 * Description:  
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

#undef DEBUG

/*
 * Map some physical address range into the kernel address space.
 */

void *__ioremap(unsigned long physaddr, unsigned long size, int cacheflag)
{
	return (void *)physaddr;
}

/*
 * Unmap a ioremap()ed region again
 */
void iounmap(void *addr)
{
}

/*
 * __iounmap unmaps nearly everything, so be careful
 * it doesn't free currently pointer/page tables anymore but it
 * wans't used anyway and might be added later.
 */
void __iounmap(void *addr, unsigned long size)
{
}

/*
 * Set new cache mode for some kernel address space.
 * The caller must push data for that range itself, if such data may already
 * be in the cache.
 */
void kernel_set_cachemode(void *addr, unsigned long size, int cmode)
{
}

int is_in_rom(unsigned long addr)
{
        extern unsigned long _ramstart, _ramend;
                                                                                
        /*
         *      What we are really trying to do is determine if addr is
         *      in an allocated kernel memory region. If not then assume
         *      we cannot free it or otherwise de-allocate it. Ideally
         *      we could restrict this to really being in a ROM or flash,
         *      but that would need to be done on a board by board basis,
         *      not globally.
         */
        if ((addr < _ramstart) || (addr >= _ramend))
                return(1);
                                                                                
        /* Default case, not in ROM */
        return(0);
}
