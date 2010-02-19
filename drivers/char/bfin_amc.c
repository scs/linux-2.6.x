/*
 * File:         drivers/char/bfin_amc.c
 * Based on:	 Borrowed from Blackfin docs "User Space Memory Usage"
 * Author:       Samuel Zahnd (samuel.zahnd@scs.ch)
 *
 * Created:      March 7 2008
 * Description:  Blackfin AMC bus driver exporting an device interface 
 *               for bank0 to user space. Intended for usage as CPLD
 *               interface for the ILCV project.
 *               Device is accessible under /dev/amc0
 *
 * Rev:          
 *
 * Modified:
 *
 * Bugs:         Send bugs to author
 *
 */


//#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>   /* printk() */
#include <linux/slab.h>   /* kmalloc() */
#include <linux/fs.h>       /* everything... */
#include <linux/errno.h>    /* error codes */
#include <linux/types.h>    /* size_t */
#include <linux/mm.h>
#include <linux/kdev_t.h>
#include <asm/page.h>
#include <linux/cdev.h>

#include <linux/device.h>

static int amc_major = 0;
static int amc_minor = 0;
module_param(amc_major, int, 0);
MODULE_AUTHOR("Samuel Zahnd");
MODULE_LICENSE("Dual BSD/GPL");

/* AMC bus (bank0) address range */
static const unsigned long cAmcBank0Start = 0x20000000;
static const unsigned long cAmcBank0Length = 0xfffff;

static bool amc_inUse = false;

/*
 * Open the device; in fact, there's nothing to do here.
 */
static int amc_open (struct inode *inode, struct file *filp)
{
	if(amc_inUse)
	{
		return -EMFILE;
	}
	amc_inUse = true;
	return 0;
}


/*
 * Closing is just as simple.
 */
static int amc_release(struct inode *inode, struct file *filp)
{
	amc_inUse = false;
	return 0;
}



/*
 * Common VMA ops.
 */

void amc_vma_open(struct vm_area_struct *vma)
{
	printk("AMC module VMA open, virt %lx, phys %lx\n",
			vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

void amc_vma_close(struct vm_area_struct *vma)
{
	printk("AMC module VMA close.\n");
}


/*
 * The remap_pfn_range version of mmap.  This one is heavily borrowed
 * from drivers/char/mem.c.
 */

static struct vm_operations_struct amc_remap_vm_ops = {
	.open =  amc_vma_open,
	.close = amc_vma_close,
};

static int amc_remap_mmap(struct file *filp, struct vm_area_struct *vma)
{
	/* the kernel passes in the vm_pgoff - the offset, in *pages*
	 * from the start of the buffer that the user space app wants
	 * to mmap. This is the last arg in the user space mmap call.
	 * We check it to make sure it is not out of bounds.
	 */
	if (vma->vm_pgoff > ((cAmcBank0Length) >> PAGE_SHIFT)) {
		return -EINVAL;
	}

	vma->vm_start = (unsigned long)cAmcBank0Start;
	vma->vm_end = (unsigned long)cAmcBank0Start + cAmcBank0Length;

	/* before remap is called, pgoff must be set to the kernel page
	 * offset from zero. Do to this, we figure out the address,
	 * ( << PAGE_SHIFT ), add it to the start of the buffer, and
	 * turn that back to pages (>> PAGE_SHIFT)
	 */
	vma->vm_pgoff = ((vma->vm_pgoff << PAGE_SHIFT ) +
			 (unsigned long)cAmcBank0Start ) >> PAGE_SHIFT;


	/*   VM_MAYSHARE limits for mprotect(), and must be set on nommu.
	 *   Other flags can be set, and are documented in
	 *   include/linux/mm.h
	 */
	vma->vm_flags |=  VM_MAYSHARE;

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start,
			    vma->vm_page_prot))
		return -EAGAIN;

	vma->vm_ops = &amc_remap_vm_ops;
	amc_vma_open(vma);
	return 0;
}

ssize_t amc_read (struct file *filp, char __user *buf, size_t count,
                loff_t *f_pos)
{
	return count;
}

/*
 * Set up the cdev structure for a device.
 */
static void amc_setup_cdev(struct cdev *dev, int minor,
		struct file_operations *fops)
{
	int err, devno = MKDEV(amc_major, minor);

	cdev_init(dev, fops);
	dev->owner = THIS_MODULE;
	dev->ops = fops;
	err = cdev_add (dev, devno, 1);
	/* Fail gracefully if need be */
	if (err)
		printk (KERN_NOTICE "Error %d adding AMC module %d", err, minor);
}


/*
 * Our various sub-devices.
 */
/* Device 0 uses remap_pfn_range */
static struct file_operations amc_remap_ops = {
	.owner   = THIS_MODULE,
	.open    = amc_open,
	.read    = amc_read,
	.release = amc_release,
	.mmap    = amc_remap_mmap,
};

/*
 *  *  Info exported via "/sys/class/amc/status".
 *   */
static ssize_t amc_status_show(struct class *class, char *buf)
{
        char *p;
/*        unsigned short i; */
        p = buf;

	p += sprintf(p, "Might be usefull to report current amc register state\n");
	p += sprintf(p, "similar to pflags. A way of sniffing is required, since\n");
	p += sprintf(p, "the AMC bus can not be accesses on read-only locations.\n");
/*
        p += sprintf(p, "PIN\t:DATA DIR INEN EDGE BOTH POLAR MASKA MASKB\n");
        p += sprintf(p, "(1/0)\t:H/L  0/I E/D  E/L  B/S   L/H   S/C   S/C\n");

        for (i = 0; i < MAX_BLACKFIN_GPIOS; i++)
                p += sprintf(p,
                             "PF%d\t: %d....%d....%d....%d....%d....%d.....%d.....%d \n",
                             i, get_gpio_data(i), get_gpio_dir(i),
                             get_gpio_inen(i), get_gpio_edge(i),
                             get_gpio_both(i), get_gpio_polar(i),
                             get_gpio_maska(i), get_gpio_maskb(i));
*/
        return p - buf;
}



static struct class *amc_class;

#define MAX_AMC_BANK0_DEV 1

/*
 * We export two amc devices.  There's no need for us to maintain any
 * special housekeeping info, so we just deal with raw cdevs.
 */
static struct cdev AmcDevs[MAX_AMC_BANK0_DEV];

static CLASS_ATTR(status, S_IRUGO, &amc_status_show, NULL);


/*
 * Module housekeeping.
 */
static int amc_init(void)
{
	int result;
	char minor_name[8];
	dev_t dev = MKDEV(amc_major, amc_minor);

	/* Figure out our device number. */
	if (amc_major)
		result = register_chrdev_region(dev, 2, "amc");
	else {
		result = alloc_chrdev_region(&dev, 0, 2, "amc");
		amc_major = MAJOR(dev);
	}
	if (result < 0) {
		printk(KERN_WARNING "AMC module: unable to get major %d\n", amc_major);
		return result;
	}
	if (amc_major == 0)
		amc_major = result;

	/* Now set up two cdevs. */
	amc_setup_cdev(AmcDevs, 0, &amc_remap_ops);

	/* Create /dev/amc entry */
        amc_class = class_create(THIS_MODULE, "amc");
        if ( class_create_file( amc_class, &class_attr_status) )
	{
	       	goto release_chrdev;
	}
        sprintf(minor_name, "amc%d", amc_minor);
        device_create(amc_class, NULL, MKDEV(amc_major, amc_minor), NULL, minor_name);

release_chrdev:
	unregister_chrdev(amc_major, "amc");

	return 0;
}


static void amc_cleanup(void)
{
	cdev_del(AmcDevs);
	unregister_chrdev_region(MKDEV(amc_major, 0), 2);
}


module_init(amc_init);
module_exit(amc_cleanup);
