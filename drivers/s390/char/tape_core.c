/*
 *  drivers/s390/char/tape_core.c
 *    basic function of the tape device driver
 *
 *  S390 and zSeries version
 *    Copyright (C) 2001,2002 IBM Deutschland Entwicklung GmbH, IBM Corporation
 *    Author(s): Carsten Otte <cotte@de.ibm.com>
 *		 Michael Holzheu <holzheu@de.ibm.com>
 *		 Tuan Ngo-Anh <ngoanh@de.ibm.com>
 *		 Martin Schwidefsky <schwidefsky@de.ibm.com>
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>	     // for kernel parameters
#include <linux/kmod.h>	     // for requesting modules
#include <linux/spinlock.h>  // for locks
#include <linux/vmalloc.h>
#include <linux/list.h>

#include <asm/types.h>	     // for variable types

#include "tape.h"
#include "tape_std.h"

#define PRINTK_HEADER "TAPE_CORE: "

static void __tape_do_irq (struct ccw_device *, unsigned long, struct irb *);
static void __tape_remove_request(struct tape_device *, struct tape_request *);

/*
 * One list to contain all tape devices of all disciplines, so
 * we can assign the devices to minor numbers of the same major
 * The list is protected by the rwlock
 */
static struct list_head tape_device_list = LIST_HEAD_INIT(tape_device_list);
static rwlock_t tape_device_lock = RW_LOCK_UNLOCKED;

/*
 * Pointer to debug area.
 */
debug_info_t *tape_dbf_area = NULL;

/*
 * Printable strings for tape enumerations.
 */
const char *tape_state_verbose[TS_SIZE] =
{
	[TS_UNUSED]   = "UNUSED",
	[TS_IN_USE]   = "IN_USE",
	[TS_BLKUSE]   = "BLKUSE",
	[TS_INIT]     = "INIT  ",
	[TS_NOT_OPER] = "NOT_OP"
};

const char *tape_op_verbose[TO_SIZE] =
{
	[TO_BLOCK] = "BLK",	[TO_BSB] = "BSB",
	[TO_BSF] = "BSF",	[TO_DSE] = "DSE",
	[TO_FSB] = "FSB",	[TO_FSF] = "FSF",
	[TO_LBL] = "LBL",	[TO_NOP] = "NOP",
	[TO_RBA] = "RBA",	[TO_RBI] = "RBI",
	[TO_RFO] = "RFO",	[TO_REW] = "REW",
	[TO_RUN] = "RUN",	[TO_WRI] = "WRI",
	[TO_WTM] = "WTM",	[TO_MSEN] = "MSN",
	[TO_LOAD] = "LOA",	[TO_READ_CONFIG] = "RCF",
	[TO_READ_ATTMSG] = "RAT",
	[TO_DIS] = "DIS",	[TO_ASSIGN] = "ASS",
	[TO_UNASSIGN] = "UAS"
};

/*
 * Some channel attached tape specific attributes.
 *
 * FIXME: In the future the first_minor and blocksize attribute should be
 *        replaced by a link to the cdev tree.
 */
static ssize_t
tape_medium_state_show(struct device *dev, char *buf)
{
	struct tape_device *tdev;

	tdev = (struct tape_device *) dev->driver_data;
	return snprintf(buf, PAGE_SIZE, "%i\n", tdev->medium_state);
}

static
DEVICE_ATTR(medium_state, 0444, tape_medium_state_show, NULL);

static ssize_t
tape_first_minor_show(struct device *dev, char *buf)
{
	struct tape_device *tdev;

	tdev = (struct tape_device *) dev->driver_data;
	return snprintf(buf, PAGE_SIZE, "%i\n", tdev->first_minor);
}

static
DEVICE_ATTR(first_minor, 0444, tape_first_minor_show, NULL);

static ssize_t
tape_state_show(struct device *dev, char *buf)
{
	struct tape_device *tdev;

	tdev = (struct tape_device *) dev->driver_data;
	return snprintf(buf, PAGE_SIZE, "%s\n", (tdev->first_minor < 0) ?
		"OFFLINE" : tape_state_verbose[tdev->tape_state]);
}

static
DEVICE_ATTR(state, 0444, tape_state_show, NULL);

static ssize_t
tape_operation_show(struct device *dev, char *buf)
{
	struct tape_device *tdev;
	ssize_t rc;

	tdev = (struct tape_device *) dev->driver_data;
	if (tdev->first_minor < 0)
		return snprintf(buf, PAGE_SIZE, "N/A\n");

	spin_lock_irq(get_ccwdev_lock(tdev->cdev));
	if (list_empty(&tdev->req_queue))
		rc = snprintf(buf, PAGE_SIZE, "---\n");
	else {
		struct tape_request *req;

		req = list_entry(tdev->req_queue.next, struct tape_request,
			list);
		rc = snprintf(buf, PAGE_SIZE, "%s\n", tape_op_verbose[req->op]);
	}
	spin_unlock_irq(get_ccwdev_lock(tdev->cdev));
	return rc;
}

static
DEVICE_ATTR(operation, 0444, tape_operation_show, NULL);

static ssize_t
tape_blocksize_show(struct device *dev, char *buf)
{
	struct tape_device *tdev;

	tdev = (struct tape_device *) dev->driver_data;

	return snprintf(buf, PAGE_SIZE, "%i\n", tdev->char_data.block_size);
}

static
DEVICE_ATTR(blocksize, 0444, tape_blocksize_show, NULL);

static struct attribute *tape_attrs[] = {
	&dev_attr_medium_state.attr,
	&dev_attr_first_minor.attr,
	&dev_attr_state.attr,
	&dev_attr_operation.attr,
	&dev_attr_blocksize.attr,
	NULL
};

static struct attribute_group tape_attr_group = {
	.attrs = tape_attrs,
};

/*
 * Tape state functions
 */
void
tape_state_set(struct tape_device *device, enum tape_state newstate)
{
	const char *str;

	if (device->tape_state == TS_NOT_OPER) {
		DBF_EVENT(3, "ts_set err: not oper\n");
		return;
	}
	DBF_EVENT(4, "ts. dev:	%x\n", device->first_minor);
	if (device->tape_state < TO_SIZE && device->tape_state >= 0)
		str = tape_state_verbose[device->tape_state];
	else
		str = "UNKNOWN TS";
	DBF_EVENT(4, "old ts:	%s\n", str);
	if (device->tape_state < TO_SIZE && device->tape_state >=0 )
		str = tape_state_verbose[device->tape_state];
	else
		str = "UNKNOWN TS";
	DBF_EVENT(4, "%s\n", str);
	DBF_EVENT(4, "new ts:\t\n");
	if (newstate < TO_SIZE && newstate >= 0)
		str = tape_state_verbose[newstate];
	else
		str = "UNKNOWN TS";
	DBF_EVENT(4, "%s\n", str);
	device->tape_state = newstate;
	wake_up(&device->state_change_wq);
}

void
tape_med_state_set(struct tape_device *device, enum tape_medium_state newstate)
{
	if (device->medium_state == newstate)
		return;
	switch(newstate){
	case MS_UNLOADED:
		device->tape_generic_status |= GMT_DR_OPEN(~0);
		PRINT_INFO("(%s): Tape is unloaded\n",
			   device->cdev->dev.bus_id);
		break;
	case MS_LOADED:
		device->tape_generic_status &= ~GMT_DR_OPEN(~0);
		PRINT_INFO("(%s): Tape has been mounted\n",
			   device->cdev->dev.bus_id);
		break;
	default:
		// print nothing
		break;
	}
	device->medium_state = newstate;
	wake_up(&device->state_change_wq);
}

/*
 * Stop running ccw. Has to be called with the device lock held.
 */
static inline int
__tape_halt_io(struct tape_device *device, struct tape_request *request)
{
	int retries;
	int rc;

	/* Check if interrupt has already been processed */
	if (request->callback == NULL)
		return 0;

	rc = 0;
	for (retries = 0; retries < 5; retries++) {
		if (retries < 2)
			rc = ccw_device_halt(device->cdev, (long) request);
		else
			rc = ccw_device_clear(device->cdev, (long) request);

		if (rc == 0) {                     /* Termination successful */
			request->rc     = -EIO;
			request->status = TAPE_REQUEST_DONE;
			return 0;
		}

		if (rc == -ENODEV)
			DBF_EXCEPTION(2, "device gone, retry\n");
		else if (rc == -EIO)
			DBF_EXCEPTION(2, "I/O error, retry\n");
		else if (rc == -EBUSY)
			DBF_EXCEPTION(2, "device busy, retry late\n");
		else
			BUG();
	}

	return rc;
}

/*
 * Add device into the sorted list, giving it the first
 * available minor number.
 */
static int
tape_assign_minor(struct tape_device *device)
{
	struct tape_device *tmp;
	int minor;

	minor = 0;
	write_lock(&tape_device_lock);
	list_for_each_entry(tmp, &tape_device_list, node) {
		if (minor < tmp->first_minor)
			break;
		minor += TAPE_MINORS_PER_DEV;
	}
	if (minor >= 256) {
		write_unlock(&tape_device_lock);
		return -ENODEV;
	}
	device->first_minor = minor;
	list_add_tail(&device->node, &tmp->node);
	write_unlock(&tape_device_lock);
	return 0;
}

/* remove device from the list */
static void
tape_remove_minor(struct tape_device *device)
{
	write_lock(&tape_device_lock);
	list_del_init(&device->node);
	device->first_minor = -1;
	write_unlock(&tape_device_lock);
}

/*
 * Enable tape device
 */
int
tape_enable_device(struct tape_device *device,
		   struct tape_discipline *discipline)
{
	int rc;

	DBF_LH(6, "tape_enable_device(%p, %p)\n", device, discipline);

	if (device->tape_state != TS_INIT) {
		DBF_LH(3, "Tapestate not INIT (%d)\n", device->tape_state);
		return -EINVAL;
	}

	/* Let the discipline have a go at the device. */
	device->discipline = discipline;
	rc = discipline->setup_device(device);
	if (rc)
		goto out;
	rc = tape_assign_minor(device);
	if (rc)
		goto out_discipline;

	rc = tapechar_setup_device(device);
	if (rc)
		goto out_minor;
	rc = tapeblock_setup_device(device);
	if (rc)
		goto out_char;

	tape_state_set(device, TS_UNUSED);
	return 0;

out_char:
	tapechar_cleanup_device(device);
out_discipline:
	device->discipline->cleanup_device(device);
	device->discipline = NULL;
out_minor:
	tape_remove_minor(device);
out:
	return rc;
}

/*
 * Disable tape device. Check if there is a running request and
 * terminate it. Post all queued requests with -EIO.
 */
void
tape_disable_device(struct tape_device *device)
{
	struct list_head *l, *n;
	struct tape_request *request;

	spin_lock_irq(get_ccwdev_lock(device->cdev));
	/* Post remaining requests with -EIO */
	list_for_each_safe(l, n, &device->req_queue) {
		request = list_entry(l, struct tape_request, list);
		if (request->status == TAPE_REQUEST_IN_IO)
			__tape_halt_io(device, request);
		list_del(&request->list);
		/* Decrease ref_count for removed request. */
		request->device = tape_put_device(device);
		request->rc = -EIO;
		if (request->callback != NULL)
			request->callback(request, request->callback_data);
	}
	spin_unlock_irq(get_ccwdev_lock(device->cdev));

	tapeblock_cleanup_device(device);
	tapechar_cleanup_device(device);
	device->discipline->cleanup_device(device);
	tape_remove_minor(device);

	tape_med_state_set(device, MS_UNKNOWN);
	device->tape_state = TS_INIT;
}

/*
 * Allocate memory for a new device structure.
 */
static struct tape_device *
tape_alloc_device(void)
{
	struct tape_device *device;

	device = (struct tape_device *)
		kmalloc(sizeof(struct tape_device), GFP_KERNEL);
	if (device == NULL) {
		DBF_EXCEPTION(2, "ti:no mem\n");
		PRINT_INFO ("can't allocate memory for "
			    "tape info structure\n");
		return ERR_PTR(-ENOMEM);
	}
	memset(device, 0, sizeof(struct tape_device));
	device->modeset_byte = (char *) kmalloc(1, GFP_KERNEL | GFP_DMA);
	if (device->modeset_byte == NULL) {
		DBF_EXCEPTION(2, "ti:no mem\n");
		PRINT_INFO("can't allocate memory for modeset byte\n");
		kfree(device);
		return ERR_PTR(-ENOMEM);
	}
	INIT_LIST_HEAD(&device->req_queue);
	INIT_LIST_HEAD(&device->node);
	init_waitqueue_head(&device->state_change_wq);
	device->tape_state = TS_INIT;
	device->medium_state = MS_UNKNOWN;
	*device->modeset_byte = 0;
	device->first_minor = -1;
	atomic_set(&device->ref_count, 1);

	return device;
}

/*
 * Get a reference to an existing device structure. This will automatically
 * increment the reference count.
 */
struct tape_device *
tape_get_device_reference(struct tape_device *device)
{
	DBF_EVENT(4, "tape_get_device_reference(%p) = %i\n", device,
		atomic_inc_return(&device->ref_count));

	return device;
}

/*
 * Decrease the reference counter of a devices structure. If the
 * reference counter reaches zero free the device structure.
 * The function returns a NULL pointer to be used by the caller
 * for clearing reference pointers.
 */
struct tape_device *
tape_put_device(struct tape_device *device)
{
	int remain;

	remain = atomic_dec_return(&device->ref_count);
	if (remain > 0) {
		DBF_EVENT(4, "tape_put_device(%p) -> %i\n", device, remain);
	} else {
		if (remain < 0) {
			DBF_EVENT(4, "put device without reference\n");
			PRINT_ERR("put device without reference\n");
		} else {
			DBF_EVENT(4, "tape_free_device(%p)\n", device);
			kfree(device->modeset_byte);
			kfree(device);
		}
	}

	return NULL;			
}

/*
 * Find tape device by a device index.
 */
struct tape_device *
tape_get_device(int devindex)
{
	struct tape_device *device, *tmp;

	device = ERR_PTR(-ENODEV);
	read_lock(&tape_device_lock);
	list_for_each_entry(tmp, &tape_device_list, node) {
		if (tmp->first_minor / TAPE_MINORS_PER_DEV == devindex) {
			device = tape_get_device_reference(tmp);
			break;
		}
	}
	read_unlock(&tape_device_lock);
	return device;
}

/*
 * Driverfs tape probe function.
 */
int
tape_generic_probe(struct ccw_device *cdev)
{
	struct tape_device *device;
	char *bus_id = cdev->dev.bus_id;

	device = tape_alloc_device();
	if (IS_ERR(device))
		return -ENODEV;
	PRINT_INFO("tape device %s found\n", bus_id);
	cdev->dev.driver_data = device;
	device->cdev = cdev;
	cdev->handler = __tape_do_irq;

	ccw_device_set_options(cdev, CCWDEV_DO_PATHGROUP);
	sysfs_create_group(&cdev->dev.kobj, &tape_attr_group);

	return 0;
}

/*
 * Driverfs tape remove function.
 */
void
tape_generic_remove(struct ccw_device *cdev)
{
	ccw_device_set_offline(cdev);
	if (cdev->dev.driver_data != NULL) {
		sysfs_remove_group(&cdev->dev.kobj, &tape_attr_group);
		cdev->dev.driver_data = tape_put_device(cdev->dev.driver_data);
	}
}

/*
 * Allocate a new tape ccw request
 */
struct tape_request *
tape_alloc_request(int cplength, int datasize)
{
	struct tape_request *request;

	if (datasize > PAGE_SIZE || (cplength*sizeof(struct ccw1)) > PAGE_SIZE)
		BUG();

	DBF_LH(6, "tape_alloc_request(%d, %d)\n", cplength, datasize);

	request = (struct tape_request *) kmalloc(sizeof(struct tape_request),
						  GFP_KERNEL);
	if (request == NULL) {
		DBF_EXCEPTION(1, "cqra nomem\n");
		return ERR_PTR(-ENOMEM);
	}
	memset(request, 0, sizeof(struct tape_request));
	/* allocate channel program */
	if (cplength > 0) {
		request->cpaddr = kmalloc(cplength*sizeof(struct ccw1),
					  GFP_ATOMIC | GFP_DMA);
		if (request->cpaddr == NULL) {
			DBF_EXCEPTION(1, "cqra nomem\n");
			kfree(request);
			return ERR_PTR(-ENOMEM);
		}
		memset(request->cpaddr, 0, cplength*sizeof(struct ccw1));
	}
	/* alloc small kernel buffer */
	if (datasize > 0) {
		request->cpdata = kmalloc(datasize, GFP_KERNEL | GFP_DMA);
		if (request->cpdata == NULL) {
			DBF_EXCEPTION(1, "cqra nomem\n");
			if (request->cpaddr != NULL)
				kfree(request->cpaddr);
			kfree(request);
			return ERR_PTR(-ENOMEM);
		}
		memset(request->cpdata, 0, datasize);
	}
	DBF_LH(6, "New request %p(%p/%p)\n", request, request->cpaddr,
		request->cpdata);

	return request;
}

/*
 * Free tape ccw request
 */
void
tape_free_request (struct tape_request * request)
{
	DBF_LH(6, "Free request %p\n", request);

	if (request->device != NULL) {
		request->device = tape_put_device(request->device);
	}
	if (request->cpdata != NULL)
		kfree(request->cpdata);
	if (request->cpaddr != NULL)
		kfree(request->cpaddr);
	kfree(request);
}

static inline void
__tape_do_io_list(struct tape_device *device)
{
	struct list_head *l, *n;
	struct tape_request *request;
	int rc;

	DBF_LH(6, "__tape_do_io_list(%p)\n", device);
	/*
	 * Try to start each request on request queue until one is
	 * started successful.
	 */
	list_for_each_safe(l, n, &device->req_queue) {
		request = list_entry(l, struct tape_request, list);
#ifdef CONFIG_S390_TAPE_BLOCK
		if (request->op == TO_BLOCK)
			device->discipline->check_locate(device, request);
#endif
		rc = ccw_device_start(device->cdev, request->cpaddr,
				      (unsigned long) request, 0x00,
				      request->options);
		if (rc == 0) {
			request->status = TAPE_REQUEST_IN_IO;
			break;
		}
		/* Start failed. Remove request and indicate failure. */
		DBF_EVENT(1, "tape: DOIO failed with er = %i\n", rc);

		/* Set ending status and do callback. */
		request->rc = rc;
		request->status = TAPE_REQUEST_DONE;
		__tape_remove_request(device, request);
	}
}

static void
__tape_remove_request(struct tape_device *device, struct tape_request *request)
{
	/* Remove from request queue. */
	list_del(&request->list);

	/* Do callback. */
	if (request->callback != NULL)
		request->callback(request, request->callback_data);

	/* Start next request. */
	if (!list_empty(&device->req_queue))
		__tape_do_io_list(device);
}

/*
 * Write sense data to console/dbf
 */
void
tape_dump_sense(struct tape_device* device, struct tape_request *request,
		struct irb *irb)
{
	unsigned int *sptr;

	PRINT_INFO("-------------------------------------------------\n");
	PRINT_INFO("DSTAT : %02x  CSTAT: %02x	CPA: %04x\n",
		   irb->scsw.dstat, irb->scsw.cstat, irb->scsw.cpa);
	PRINT_INFO("DEVICE: %s\n", device->cdev->dev.bus_id);
	if (request != NULL)
		PRINT_INFO("OP	  : %s\n", tape_op_verbose[request->op]);

	sptr = (unsigned int *) irb->ecw;
	PRINT_INFO("Sense data: %08X %08X %08X %08X \n",
		   sptr[0], sptr[1], sptr[2], sptr[3]);
	PRINT_INFO("Sense data: %08X %08X %08X %08X \n",
		   sptr[4], sptr[5], sptr[6], sptr[7]);
	PRINT_INFO("--------------------------------------------------\n");
}

/*
 * Write sense data to dbf
 */
void
tape_dump_sense_dbf(struct tape_device *device, struct tape_request *request,
		    struct irb *irb)
{
	unsigned int *sptr;
	const char* op;

	if (request != NULL)
		op = tape_op_verbose[request->op];
	else
		op = "---";
	DBF_EVENT(3, "DSTAT : %02x   CSTAT: %02x\n",
		  irb->scsw.dstat,irb->scsw.cstat);
	DBF_EVENT(3, "DEVICE: %s OP\t: %s\n", device->cdev->dev.bus_id,op);
	sptr = (unsigned int *) irb->ecw;
	DBF_EVENT(3, "%08x %08x\n", sptr[0], sptr[1]);
	DBF_EVENT(3, "%08x %08x\n", sptr[2], sptr[3]);
	DBF_EVENT(3, "%08x %08x\n", sptr[4], sptr[5]);
	DBF_EVENT(3, "%08x %08x\n", sptr[6], sptr[7]);
}

/*
 * I/O helper function. Adds the request to the request queue
 * and starts it if the tape is idle. Has to be called with
 * the device lock held.
 */
static inline int
__tape_do_io(struct tape_device *device, struct tape_request *request)
{
	int rc;

	switch (request->op) {
		case TO_MSEN:
		case TO_ASSIGN:
		case TO_UNASSIGN:
		case TO_READ_ATTMSG:
			if (device->tape_state == TS_INIT)
				break;
			if (device->tape_state == TS_UNUSED)
				break;
		default:
			if (device->tape_state == TS_BLKUSE)
				break;
			if (device->tape_state != TS_IN_USE)
				return -ENODEV;
	}

	/* Increase use count of device for the added request. */
	request->device = tape_get_device_reference(device);

	if (list_empty(&device->req_queue)) {
		/* No other requests are on the queue. Start this one. */
#ifdef CONFIG_S390_TAPE_BLOCK
		if (request->op == TO_BLOCK)
			device->discipline->check_locate(device, request);
#endif
		rc = ccw_device_start(device->cdev, request->cpaddr,
				      (unsigned long) request, 0x00,
				      request->options);
		if (rc) {
			DBF_EVENT(1, "tape: DOIO failed with rc = %i\n", rc);
			return rc;
		}
		DBF_LH(5, "Request %p added for execution.\n", request);
		list_add(&request->list, &device->req_queue);
		request->status = TAPE_REQUEST_IN_IO;
	} else {
		DBF_LH(5, "Request %p add to queue.\n", request);
		list_add_tail(&request->list, &device->req_queue);
		request->status = TAPE_REQUEST_QUEUED;
	}
	return 0;
}

/*
 * Add the request to the request queue, try to start it if the
 * tape is idle. Return without waiting for end of i/o.
 */
int
tape_do_io_async(struct tape_device *device, struct tape_request *request)
{
	int rc;

	DBF_LH(6, "tape_do_io_async(%p, %p)\n", device, request);

	spin_lock_irq(get_ccwdev_lock(device->cdev));
	/* Add request to request queue and try to start it. */
	rc = __tape_do_io(device, request);
	spin_unlock_irq(get_ccwdev_lock(device->cdev));
	return rc;
}

/*
 * tape_do_io/__tape_wake_up
 * Add the request to the request queue, try to start it if the
 * tape is idle and wait uninterruptible for its completion.
 */
static void
__tape_wake_up(struct tape_request *request, void *data)
{
	request->callback = NULL;
	wake_up((wait_queue_head_t *) data);
}

int
tape_do_io(struct tape_device *device, struct tape_request *request)
{
	wait_queue_head_t wq;
	int rc;

	init_waitqueue_head(&wq);
	spin_lock_irq(get_ccwdev_lock(device->cdev));
	/* Setup callback */
	request->callback = __tape_wake_up;
	request->callback_data = &wq;
	/* Add request to request queue and try to start it. */
	rc = __tape_do_io(device, request);
	spin_unlock_irq(get_ccwdev_lock(device->cdev));
	if (rc)
		return rc;
	/* Request added to the queue. Wait for its completion. */
	wait_event(wq, (request->callback == NULL));
	/* Get rc from request */
	return request->rc;
}

/*
 * tape_do_io_interruptible/__tape_wake_up_interruptible
 * Add the request to the request queue, try to start it if the
 * tape is idle and wait uninterruptible for its completion.
 */
static void
__tape_wake_up_interruptible(struct tape_request *request, void *data)
{
	request->callback = NULL;
	wake_up_interruptible((wait_queue_head_t *) data);
}

int
tape_do_io_interruptible(struct tape_device *device,
			 struct tape_request *request)
{
	wait_queue_head_t wq;
	int rc;

	init_waitqueue_head(&wq);
	spin_lock_irq(get_ccwdev_lock(device->cdev));
	/* Setup callback */
	request->callback = __tape_wake_up_interruptible;
	request->callback_data = &wq;
	rc = __tape_do_io(device, request);
	spin_unlock_irq(get_ccwdev_lock(device->cdev));
	if (rc)
		return rc;
	/* Request added to the queue. Wait for its completion. */
	rc = wait_event_interruptible(wq, (request->callback == NULL));
	if (rc != -ERESTARTSYS)
		/* Request finished normally. */
		return request->rc;
	/* Interrupted by a signal. We have to stop the current request. */
	spin_lock_irq(get_ccwdev_lock(device->cdev));
	rc = __tape_halt_io(device, request);
	if (rc == 0) {
		DBF_EVENT(3, "IO stopped on %s\n", device->cdev->dev.bus_id);
		rc = -ERESTARTSYS;
	}
	spin_unlock_irq(get_ccwdev_lock(device->cdev));
	return rc;
}

/*
 * Tape interrupt routine, called from the ccw_device layer
 */
static void
__tape_do_irq (struct ccw_device *cdev, unsigned long intparm, struct irb *irb)
{
	struct tape_device *device;
	struct tape_request *request;
	int final;
	int rc;

	device = (struct tape_device *) cdev->dev.driver_data;
	if (device == NULL) {
		PRINT_ERR("could not get device structure for bus_id %s "
			  "in interrupt\n", cdev->dev.bus_id);
		return;
	}
	request = (struct tape_request *) intparm;

	DBF_LH(6, "__tape_do_irq(device=%p, request=%p)\n", device, request);

	/* May be an unsolicited irq */
	if(request != NULL)
		request->rescnt = irb->scsw.count;

	if (irb->scsw.dstat != 0x0c) {
		/* Set the 'ONLINE' flag depending on sense byte 1 */
		if(*(((__u8 *) irb->ecw) + 1) & SENSE_DRIVE_ONLINE)
			device->tape_generic_status |= GMT_ONLINE(~0);
		else
			device->tape_generic_status &= ~GMT_ONLINE(~0);

		/*
		 * Any request that does not come back with channel end
		 * and device end is unusual. Log the sense data.
		 */
		DBF_EVENT(3,"-- Tape Interrupthandler --\n");
		tape_dump_sense_dbf(device, request, irb);
	} else {
		/* Upon normal completion the device _is_ online */
		device->tape_generic_status |= GMT_ONLINE(~0);
	}
	if (device->tape_state == TS_NOT_OPER) {
		DBF_EVENT(6, "tape:device is not operational\n");
		return;
	}

	/*
	 * Request that were canceled still come back with an interrupt.
	 * To detect these request the state will be set to TAPE_REQUEST_DONE.
	 */
	if(request != NULL && request->status == TAPE_REQUEST_DONE) {
		__tape_remove_request(device, request);
		return;
	}

	rc = device->discipline->irq(device, request, irb);
	/*
	 * rc < 0 : request finished unsuccessfully.
	 * rc == TAPE_IO_SUCCESS: request finished successfully.
	 * rc == TAPE_IO_PENDING: request is still running. Ignore rc.
	 * rc == TAPE_IO_RETRY: request finished but needs another go.
	 * rc == TAPE_IO_STOP: request needs to get terminated.
	 */
	final = 0;
	switch (rc) {
	case TAPE_IO_SUCCESS:
		/* Upon normal completion the device _is_ online */
		device->tape_generic_status |= GMT_ONLINE(~0);
		final = 1;
		break;
	case TAPE_IO_PENDING:
		break;
	case TAPE_IO_RETRY:
#ifdef CONFIG_S390_TAPE_BLOCK
		if (request->op == TO_BLOCK)
			device->discipline->check_locate(device, request);
#endif
		rc = ccw_device_start(cdev, request->cpaddr,
				      (unsigned long) request, 0x00,
				      request->options);
		if (rc) {
			DBF_EVENT(1, "tape: DOIO failed with er = %i\n", rc);
			final = 1;
		}
		break;
	case TAPE_IO_STOP:
		__tape_halt_io(device, request);
		break;
	default:
		if (rc > 0) {
			DBF_EVENT(6, "xunknownrc\n");
			PRINT_ERR("Invalid return code from discipline "
				  "interrupt function.\n");
			rc = -EIO;
		}
		final = 1;
		break;
	}
	if (final) {
		/* May be an unsolicited irq */
		if(request != NULL) {
			/* Set ending status. */
			request->rc = rc;
			request->status = TAPE_REQUEST_DONE;
			__tape_remove_request(device, request);
		} else {
			__tape_do_io_list(device);
		}
	}
}

/*
 * Tape device open function used by tape_char & tape_block frontends.
 */
int
tape_open(struct tape_device *device)
{
	int rc;

	spin_lock(get_ccwdev_lock(device->cdev));
	if (device->tape_state == TS_NOT_OPER) {
		DBF_EVENT(6, "TAPE:nodev\n");
		rc = -ENODEV;
	} else if (device->tape_state == TS_IN_USE) {
		DBF_EVENT(6, "TAPE:dbusy\n");
		rc = -EBUSY;
	} else if (device->tape_state == TS_BLKUSE) {
		DBF_EVENT(6, "TAPE:dbusy\n");
		rc = -EBUSY;
	} else if (device->discipline != NULL &&
		   !try_module_get(device->discipline->owner)) {
		DBF_EVENT(6, "TAPE:nodisc\n");
		rc = -ENODEV;
	} else {
		tape_state_set(device, TS_IN_USE);
		rc = 0;
	}
	spin_unlock(get_ccwdev_lock(device->cdev));
	return rc;
}

/*
 * Tape device release function used by tape_char & tape_block frontends.
 */
int
tape_release(struct tape_device *device)
{
	spin_lock(get_ccwdev_lock(device->cdev));
	if (device->tape_state == TS_IN_USE)
		tape_state_set(device, TS_UNUSED);
	module_put(device->discipline->owner);
	spin_unlock(get_ccwdev_lock(device->cdev));
	return 0;
}

/*
 * Execute a magnetic tape command a number of times.
 */
int
tape_mtop(struct tape_device *device, int mt_op, int mt_count)
{
	tape_mtop_fn fn;
	int rc;

	DBF_EVENT(6, "TAPE:mtio\n");
	DBF_EVENT(6, "TAPE:ioop: %x\n", mt_op);
	DBF_EVENT(6, "TAPE:arg:	 %x\n", mt_count);

	if (mt_op < 0 || mt_op >= TAPE_NR_MTOPS)
		return -EINVAL;
	fn = device->discipline->mtop_array[mt_op];
	if (fn == NULL)
		return -EINVAL;

	/* We assume that the backends can handle count up to 500. */
	if (mt_op == MTBSR  || mt_op == MTFSR  || mt_op == MTFSF  ||
	    mt_op == MTBSF  || mt_op == MTFSFM || mt_op == MTBSFM) {
		rc = 0;
		for (; mt_count > 500; mt_count -= 500)
			if ((rc = fn(device, 500)) != 0)
				break;
		if (rc == 0)
			rc = fn(device, mt_count);
	} else
		rc = fn(device, mt_count);
	return rc;

}

/*
 * Hutplug event support.
 */
void
tape_hotplug_event(struct tape_device *device, int devmaj, int action) {
#ifdef CONFIG_HOTPLUG
	char *argv[3];
	char *envp[8];
	char  busid[20];
	char  major[20];
	char  minor[20];

	/* Call the busid DEVNO to be compatible with old tape.agent. */
	sprintf(busid, "DEVNO=%s",   device->cdev->dev.bus_id);
	sprintf(major, "MAJOR=%d",   devmaj);
	sprintf(minor, "MINOR=%d",   device->first_minor);

	argv[0] = hotplug_path;
	argv[1] = "tape";
	argv[2] = NULL;

	envp[0] = "HOME=/";
	envp[1] = "PATH=/sbin:/bin:/usr/sbin:/usr/bin";

	switch (action) {
		case TAPE_HOTPLUG_CHAR_ADD:
		case TAPE_HOTPLUG_BLOCK_ADD:
			envp[2] = "ACTION=add";
			break;
		case TAPE_HOTPLUG_CHAR_REMOVE:
		case TAPE_HOTPLUG_BLOCK_REMOVE:
			envp[2] = "ACTION=remove";
			break;
		default:
			BUG();
	}
	switch (action) {
		case TAPE_HOTPLUG_CHAR_ADD:
		case TAPE_HOTPLUG_CHAR_REMOVE:
			envp[3] = "INTERFACE=char";
			break;
		case TAPE_HOTPLUG_BLOCK_ADD:
		case TAPE_HOTPLUG_BLOCK_REMOVE:
			envp[3] = "INTERFACE=block";
			break;
		default:
			BUG();
	}
	envp[4] = busid;
	envp[5] = major;
	envp[6] = minor;
	envp[7] = NULL;

	call_usermodehelper(argv[0], argv, envp, 0);
#endif
}

/*
 * Tape init function.
 */
static int
tape_init (void)
{
	tape_dbf_area = debug_register ( "tape", 1, 2, 4*sizeof(long));
	debug_register_view(tape_dbf_area, &debug_sprintf_view);
#ifdef DBF_LIKE_HELL
	debug_set_level(tape_dbf_area, 6);
#endif
	DBF_EVENT(3, "tape init: ($Revision$)\n");
	tape_proc_init();
	tapechar_init ();
	tapeblock_init ();
	return 0;
}

/*
 * Tape exit function.
 */
static void
tape_exit(void)
{
	DBF_EVENT(6, "tape exit\n");

	/* Get rid of the frontends */
	tapechar_exit();
	tapeblock_exit();
	tape_proc_cleanup();
	debug_unregister (tape_dbf_area);
}

MODULE_AUTHOR("(C) 2001 IBM Deutschland Entwicklung GmbH by Carsten Otte and "
	      "Michael Holzheu (cotte@de.ibm.com,holzheu@de.ibm.com)");
MODULE_DESCRIPTION("Linux on zSeries channel attached "
		   "tape device driver ($Revision$)");
MODULE_LICENSE("GPL");

module_init(tape_init);
module_exit(tape_exit);

EXPORT_SYMBOL(tape_dbf_area);
EXPORT_SYMBOL(tape_generic_remove);
EXPORT_SYMBOL(tape_disable_device);
EXPORT_SYMBOL(tape_generic_probe);
EXPORT_SYMBOL(tape_enable_device);
EXPORT_SYMBOL(tape_put_device);
EXPORT_SYMBOL(tape_get_device_reference);
EXPORT_SYMBOL(tape_state_verbose);
EXPORT_SYMBOL(tape_op_verbose);
EXPORT_SYMBOL(tape_state_set);
EXPORT_SYMBOL(tape_med_state_set);
EXPORT_SYMBOL(tape_alloc_request);
EXPORT_SYMBOL(tape_free_request);
EXPORT_SYMBOL(tape_dump_sense);
EXPORT_SYMBOL(tape_dump_sense_dbf);
EXPORT_SYMBOL(tape_do_io);
EXPORT_SYMBOL(tape_do_io_async);
EXPORT_SYMBOL(tape_do_io_interruptible);
EXPORT_SYMBOL(tape_mtop);
