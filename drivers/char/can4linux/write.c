/*
 * can_write - can4linux CAN driver module
 *
 * can4linux -- LINUX CAN device driver source
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * 
 * Copyright (c) 2001 port GmbH Halle/Saale
 * (c) 2001 Heinz-J�rgen Oertel (oe@port.de)
 *          Claus Schroeter (clausi@chemie.fu-berlin.de)
 *------------------------------------------------------------------
 * $Header: /z2/cvsroot/products/0530/software/can4linux/src/write.c,v 1.3 2007/02/07 15:26:30 oe Exp $
 *
 *--------------------------------------------------------------------------
 *
 *
 *
 *
 *
 *
 *--------------------------------------------------------------------------
 */


/**
* \file write.c
* \author Heinz-J�rgen Oertel, port GmbH
* $Revision: 3628 $
* $Date: 2007-08-29 09:08:48 -0400 (Wed, 29 Aug 2007) $
*
*/

#include "defs.h"

/* \fn size_t can_write( __LDDK_WRITE_PARAM) */
/***************************************************************************/
/**

\brief size_t write(int fd, const char *buf, size_t count);
write CAN messages to the network
\param fd The descriptor to write to.
\param buf The data buffer to write (array of CAN canmsg_t).
\param count The number of bytes to write.

write  writes  up to \a count CAN messages to the CAN controller
referenced by the file descriptor fd from the buffer
starting at buf.



\par Errors

the following errors can occur

\li \c EBADF  fd is not a valid file descriptor or is not open
              for writing.

\li \c EINVAL fd is attached to an object which is unsuitable for
              writing.

\li \c EFAULT buf is outside your accessible address space.

\li \c EINTR  The call was interrupted by a signal before any
              data was written.



\returns
On success, the number of CAN messages written are returned
(zero indicates nothing was written).
On error, -1 is returned, and errno is set appropriately.

\internal
*/

__LDDK_WRITE_TYPE can_write( __LDDK_WRITE_PARAM )
{
unsigned int minor = __LDDK_MINOR;
msg_fifo_t *TxFifo = &Tx_Buf[minor];
canmsg_t *addr;
canmsg_t tx;
unsigned long flags;  /* still needed for local_irq_save() ? */
int written        = 0;
int blocking;
unsigned long _cnt;

    DBGin("can_write");
#ifdef DEBUG_COUNTER
    Cnt1[minor] = Cnt1[minor] + 1;
#endif /* DEBUG_COUNTER */


    /* detect write mode */
    blocking = !(file->f_flags & O_NONBLOCK);

/* DEBUG_TTY(1, "write: %d", count); */
    DBGprint(DBG_DATA,(" -- write %d msg\n", (int)count));
    /* printk("w[%d/%d]", minor, TxFifo->active); */
    addr = (canmsg_t *)buffer;

    if(!access_ok(VERIFY_READ, (canmsg_t *)addr, count * sizeof(canmsg_t))) {
	    DBGout();return -EINVAL;
    }
    while( written < count ) {
	/* enter critical section */

	local_irq_save(flags);

	/* Do we really need to protect something here ????
	 * e.g. in this case the TxFifo->free[TxFifo->head] value
	 *
	 * If YES, we have to use spinlocks for synchronization
	 */

/* - new Blocking code -- */

	if(blocking) {
	    if(wait_event_interruptible(CanOutWait[minor], \
		    TxFifo->free[TxFifo->head] != BUF_FULL))
			/* TxFifo->tail != TxFifo->head )) */
			return -ERESTARTSYS;

	} else {
	    /* there are data to write to the network */
	    if(TxFifo->free[TxFifo->head] == BUF_FULL) {
		/* there is already one message at this place */;
		/* local_irq_restore(flags); */
		DBGout();
		/* return -ENOSPC; */
		return written;
	    }
	}

/* ---- */

	if( TxFifo->active ) {
	    /* more than one data and actual data in queue,
	     * add this message to the Tx queue 
	     */
	    __lddk_copy_from_user(	/* copy one message to FIFO */
		    (canmsg_t *) &(TxFifo->data[TxFifo->head]), 
		    (canmsg_t *) &addr[written],
		    sizeof(canmsg_t) );
	    TxFifo->free[TxFifo->head] = BUF_FULL; /* now this entry is FULL */
	    TxFifo->head = ++(TxFifo->head) % MAX_BUFSIZE;

	} else {
	    __lddk_copy_from_user(
		    (canmsg_t *) &tx, 
		    (canmsg_t *) &addr[written],
		    sizeof(canmsg_t) );
	  /* f - fast -- use interrupts */
	  if( count >= 1 ) {
	    /* !!! CHIP abh. !!! */
	    TxFifo->active = 1;
	  }
	  CAN_SendMessage( minor, &tx);  /* Send, no wait */
	}
        written++;
        local_irq_restore(flags);
    }
    DBGout();
    return written;
}

