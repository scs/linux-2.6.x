/*
 * can_open - can4linux CAN driver module
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
 * $Header: /z2/cvsroot/products/0530/software/can4linux/src/open.c,v 1.2 2006/06/02 12:13:14 oe Exp $
 *
 *--------------------------------------------------------------------------
 *
 *
 *
 *
 *--------------------------------------------------------------------------
 */


/**
* \file open.c
* \author Heinz-J�rgen Oertel, port GmbH
* $Revision$
* $Date$
*
*
*/


/* header of standard C - libraries */

/* header of common types */

/* shared common header */

/* header of project specific types */

/* project headers */
#include "defs.h"

/* local header */

/* constant definitions
---------------------------------------------------------------------------*/

/* local defined data types
---------------------------------------------------------------------------*/

/* list of external used functions, if not in headers
---------------------------------------------------------------------------*/

/* list of global defined functions
---------------------------------------------------------------------------*/

/* list of local defined functions
---------------------------------------------------------------------------*/

/* external variables
---------------------------------------------------------------------------*/

/* global variables
---------------------------------------------------------------------------*/
int Can_isopen[MAX_CHANNELS] = { 0 };   /* device minor already opened */

/* local defined variables
---------------------------------------------------------------------------*/
/* static char _rcsid[] = "$Id$"; */


/***************************************************************************/
/**
*
* \brief int open(const char *pathname, int flags);
* opens the CAN device for following operations
* \param pathname device pathname, usual /dev/can?
* \param flags is one of \c O_RDONLY, \c O_WRONLY or \c O_RDWR which request
*       opening  the  file  read-only,  write-only  or read/write,
*       respectively.
*
*
* The open call is used to "open" the device.
* Doing a first initialization according the to values in the /proc/sys/Can
* file system.
* Additional an ISR function is assigned to the IRQ.
*
* The CLK OUT pin is configured for creating the same frequency
* like the chips input frequency fclk (XTAL). 
*
* If Vendor Option \a VendOpt is set to 's' the driver performs
* an hardware reset befor initializing the chip.
*
* \returns
* open return the new file descriptor,
* or -1 if an error occurred (in which case, errno is set appropriately).
*
* \par ERRORS
* the following errors can occur
* \arg \c ENXIO  the file is a device special file
* and no corresponding device exists.
* \arg \c EINVAL illegal \b minor device number
* \arg \c EINVAL wrong IO-model format in /proc/sys/Can/IOmodel
* \arg \c EBUSY  IRQ for hardware is not available
* \arg \c EBUSY  I/O region for hardware is not available

*/
int can_open( __LDDK_OPEN_PARAM )
{
int retval = 0;

    DBGin("can_open");
    {

	int lasterr;	

	unsigned int minor = iminor(inode);

	if( minor > MAX_CHANNELS )
	{
	    printk(KERN_ERR "CAN: Illegal minor number %d\n", minor);
	    DBGout();
	    return -EINVAL;
	}

    /* check if device is already open, should be used only by one process */
	if(Can_isopen[minor] == 1) {
	    DBGout();
	    return -ENXIO;
	} 

	if( Base[minor] == 0x00) {
	    /* No device available */
	    printk(KERN_ERR "CAN[%d]: no device available\n", minor);
	    DBGout();
	    return -ENXIO;
	}

	/* the following does all the board specific things
	   also memory remapping if necessary */
	if( (lasterr = CAN_VendorInit(minor)) < 0 ){
	    DBGout();
	    return lasterr;
	}

	/* Access macros based in can_base[] should work now */
	/* CAN_ShowStat(minor); */

/* controller_available(curr + 0x400, 4); */
	Can_WaitInit(minor);	/* initialize wait queue for select() */
	Can_FifoInit(minor);
#if CAN_USE_FILTER
	Can_FilterInit(minor);
#endif

	if( CAN_ChipReset(minor) < 0 ) {
	    DBGout();
	    return -EINVAL;
	}
	CAN_StartChip(minor);
#if DEBUG
	CAN_ShowStat(minor);
#endif
	++Can_isopen[minor]; /* flag device in use */
    }

    DBGout();
    return retval;
}

