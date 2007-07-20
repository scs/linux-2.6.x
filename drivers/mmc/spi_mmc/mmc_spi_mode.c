/**************************************************************
*
* Copyright (C) 2005, Rubico AB. All Rights Reserve.
*
* Developed as a part the CDT project C4(www.cdt.ltu.se).
*
* FILE mmc_spi_mode.c
*
* PROGRAMMER: Hans Eklund (hans [at] rubico [dot] se) (Rubico AB)
*
* DATE OF CREATION: April, 2006.
*
* SYNOPSIS:
*
* DESCRIPTION: SPI-MMC/SD Protocol.
*
* DEPENDENCIES: Independent.
*	(well, one, for printing debug text on the target, (kernel.h for linux))
*
* TODO: Correct Multiple block read and write functions. Didnt have time
*	to make them all failsafe. Will be done soon.
*
**************************************************************
*
* This program is free software; you can distribute it and/or modify it
* under the terms of the GNU General Public License (Version 2) as
* published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
* for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
*
**************************************************************/
#include "mmc_spi_mode.h"
#include <linux/kernel.h>	/* for printk() only*/
#include <linux/poll.h>
#include <linux/delay.h>

//#define USE_MULT_BLOCK_READS


#ifdef CONFIG_SPI_MMC_DEBUG_MODE
#define DPRINTK(x...)   printk("%lu, %s(): %d ", jiffies, __PRETTY_FUNCTION__, __LINE__);printk(x);
#define DPRINTKM(x...)   printk("%lu, %s(): %d ", jiffies, __PRETTY_FUNCTION__, __LINE__);printk(x);
#else
#define DPRINTK(x...)   do { } while (0)
#define DPRINTKM(x...)   do { } while (0)
#endif

#ifdef CONFIG_SPI_MMC_DEBUG_MODE
#define DPRINT_STAT(x...)   printk("%s(): %d ", __PRETTY_FUNCTION__, __LINE__);printk(x);
#else
#define DPRINT_STAT(x...)   do { } while (0)
#endif

#ifdef LINUX
#define TRY_SCHEDULE    schedule();
#else
#define TRY_SCHEDULE    while(0);
#endif

static unsigned char mmc_cmd[6] = {0x40,0x00,0x00,0x00,0x00,0x95};
static unsigned char Null_Word[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static unsigned char latest_cmd;

static short read_mmc_reg(struct mmc_spi_dev *pdev, short csd);
static unsigned char mmc_wait_response(struct mmc_spi_dev *pdev, unsigned int timeout);

static int init_mode = 1;
static unsigned char wrb[WRB_LEN];


/**********************************************************************\
*
* MMC CSD/CID related, could be somewhat trimmed and cleaned
*
\**********************************************************************/
typedef unsigned char BOOL;
BOOL getbit(void* ptr, unsigned int n) {
	unsigned int byte_nr;
	unsigned int bit_nr;

	byte_nr = n/8;
	bit_nr = n % 8;

	return (BOOL)(((unsigned char*)ptr)[byte_nr] >> bit_nr) & 1;
}

unsigned int getvalue(void* ptr, unsigned int n, unsigned int len) {
	unsigned int value=0;
	int i=0;

	for(i=0;i<len; i++) {	
		value += ((unsigned int)getbit(ptr, n+i))<<i;
	}
	return value;
}

void mmc_spi_fill_card_struct(struct mmc_spi_dev *pdev)
{
	unsigned short c_size_mult=0;
	unsigned short c_size=0;
	
	unsigned char *raw_csd;
	unsigned char *raw_cid;

	// local, shorter names, just to keep lines below shorter
	raw_csd = pdev->raw_csd;
	raw_cid = pdev->raw_cid;
	
	pdev->csd.mmca_vsn = (raw_csd[0] & 0x3c) >> 2;
	pdev->csd.cmdclass = (((u16)raw_csd[4]) << 4) | ((raw_csd[5] & 0xf0) >> 4);
	pdev->csd.tacc_clks = raw_csd[1];
	pdev->csd.tacc_ns = raw_csd[2];
	pdev->csd.max_dtr = raw_csd[3];
	pdev->csd.read_blkbits = raw_csd[5] & 0x0f;
	
	// for calculating capacity(in blocks)
	c_size = ((((u16)raw_csd[6]) & 0x03) << 10) | (((u16)raw_csd[7]) << 2) | (((u16)raw_csd[8]) & 0xc0) >> 6;
	c_size_mult = ((raw_csd[9] & 0x03) << 1) | ((raw_csd[10] & 0x80) >> 7);
	pdev->csd.capacity = (c_size+1) * (1 << (c_size_mult + 2));
		
	pdev->cid.manfid = getvalue(raw_cid, 127-127, 8);
	memcpy(pdev->cid.prod_name, raw_cid+3, 7);
	pdev->cid.serial = getvalue(raw_cid, 127-47, 32);
	pdev->cid.oemid = getvalue(raw_cid, 127-119, 16);
	pdev->cid.year = 1997 + (getvalue(raw_cid, 127-15, 8) & 0x0F);
	pdev->cid.hwrev = (getvalue(raw_cid, 127-55, 8) & 0xF0) >> 4;
	pdev->cid.fwrev = getvalue(raw_cid, 127-55, 8) & 0x0F;
	pdev->cid.month = (getvalue(raw_cid, 127-15, 8) & 0xF0) >> 4;	
}

short mmc_spi_get_card(struct mmc_spi_dev *pdev)
{	
	//memset(pdev->raw_cid, 0, 18);
	//memset(pdev->raw_csd, 0, 18);
	
	if(read_mmc_reg(pdev, 1)) {
		DPRINTK("CSD register read failed.\n");
		return 1;
	}
	if(read_mmc_reg(pdev, 0)) {
		DPRINTK("CID register read failed.\n");
		return 1;
	}

	// Parse CSD and CID data
	mmc_spi_fill_card_struct(pdev);

	return 0;
}

static short send_cmd_and_wait(struct mmc_spi_dev *pdev, 
			       unsigned char command, 
			       unsigned int argument, 
			       unsigned short cmd_resp, 
			       unsigned int timeout)
{
	unsigned short resp=0xff;
	unsigned short rval=0;
	
	// Build command string
	mmc_cmd[0] = 0x40 + command;
	mmc_cmd[1] = (unsigned char)(argument >> 24 & 0xff);
	mmc_cmd[2] = (unsigned char)(argument >> 16 & 0xff);
	mmc_cmd[3] = (unsigned char)(argument >> 8 & 0xff);
	mmc_cmd[4] = (unsigned char)(argument & 0xff);
	mmc_cmd[5] = 0x95;	// CRC form CMD0 actually, but valid for all since SPI dont care

	// record last command if not in init mode
	if(!init_mode) {
		latest_cmd = command;
	}

	if(init_mode || pdev->sd) {
		// Send a few zeros since SDs may be sleeping
		if(pdev->write(Null_Word, SD_PRE_CMD_ZEROS, pdev->priv_data)<0) {
			DPRINTK("sending SD_PRE_CMD_ZEROS failed\n");
			rval = ERR_SPI_TIMEOUT;
			goto out;
		}
	}

	if(pdev->write(mmc_cmd, 6, pdev->priv_data) < 0) {
		DPRINTK("sending command %d failed\n", command);
		rval = ERR_SPI_TIMEOUT;
		goto out;
	}	

	if((resp=mmc_wait_response(pdev, timeout)) != cmd_resp) {
        // NOTE: ignore "illegal command" responses
        if(resp==4) {
            rval=0;
            goto out;
        }
		// Will only be active during init, seems to be needed by some SDs.
		if(init_mode) {
			udelay(1000);
		} else {
			DPRINTK("unexpected response to command %d, wanted 0x%x, got 0x%x)\n", command, cmd_resp, resp);
			//udelay(100);
		}		
		rval =  ERR_MMC_TIMEOUT;
		goto out;
	}
	out:
	// send 8 clocks for SD cards
	if(pdev->sd) {
		//mmc_spi_dummy_clocks(pdev, SD_CLK_CNTRL);
	}

	return rval;
}

static short mmc_spi_error_handler(struct mmc_spi_dev *pdev, short rval)
{
	// Handle log index, wrap if necessary
	unsigned short status=0;

	// If error, check status and log.
	if(rval) {
		//shift old log entries down
		memcpy(pdev->error_log+1, pdev->error_log, LOG_LEN-1);
		memcpy(pdev->status_log+1, pdev->status_log, LOG_LEN-1);
		memcpy(pdev->cmd_log+1, pdev->cmd_log, LOG_LEN-1);
		pdev->cmd_log[0] = latest_cmd;
		pdev->error_log[0] = rval;

		// NOTE: status may be zero even on errors.
		// 	 since data lines may be left low(if card pulled from socket for ex.)
		status = mmc_spi_read_status(pdev);
		pdev->status_log[0] =  status;
		pdev->errors++;

        DPRINTK("Latest command was: %d\n", latest_cmd);
	}
	switch(rval) {
		case ERR_SPI_TIMEOUT:
			DPRINTK("ERR_SPI_TIMEOUT\n");
			return RVAL_CRITICAL;
		case ERR_MMC_TIMEOUT:
			DPRINTK("ERR_MMC_TIMEOUT\n");
			return RVAL_ERROR;
		case ERR_MMC_PROG_TIMEOUT:
		case ERR_UNKNOWN_TOK:
		case DR_CRC_ERROR:
		case DR_WRITE_ERROR:
		default:
			if(status) {	
				return RVAL_ERROR;
			} else {
				// NOTE: could use status to determine what to do more accurately
				return RVAL_OK;
			}
	}
	return 0;
}

/**
* read_mmc_reg - reads the 128 bit CSD or CID register data + 2 byte CRC
*
*/
static short read_mmc_reg(struct mmc_spi_dev *pdev, short csd)
{
	unsigned char resp=0xff;
	unsigned char* buf;
	unsigned short rval = 0;

	pdev->assert();
	if(csd) {
		if((rval=send_cmd_and_wait(pdev, SEND_CSD, 0, R1_OK, MMC_COMMAND_TIMEOUT))) {
			goto out; 
		}
		buf = pdev->raw_csd;
	} else {
		if((rval=send_cmd_and_wait(pdev, SEND_CID, 0, R1_OK, MMC_COMMAND_TIMEOUT))) {
			goto out;
		}
		buf = pdev->raw_cid;
	}

	// start block token 
	if((resp=mmc_wait_response(pdev, MMC_COMMAND_TIMEOUT)) != SBT_S_BLOCK_READ) {
        	DPRINTK("mmc did not send 0xFE(got 0x%x)\n",resp);
		rval = resp;
		goto out;
        }
        if(pdev->read(buf, 18, pdev->priv_data) < 18) {
                DPRINTK("reading 18 bytes of data failed\n");
		rval = ERR_SPI_TIMEOUT;
		goto out;
        }
	out:
	// send clocks for SD cards
	if(pdev->sd)
		mmc_spi_dummy_clocks(pdev, SD_CLK_CNTRL);

	pdev->deassert();

	// check for errors, but dont change rval
	mmc_spi_error_handler(pdev, rval);

	return rval;
}

short mmc_spi_read_status(struct mmc_spi_dev *pdev)
{
	unsigned char b1=0;
	unsigned char b2=0;
	unsigned short r2=0xffff;
	static unsigned char status_cmd[6] = {0x4D,0x00,0x00,0x00,0x00,0x95};

	pdev->assert();

	if(pdev->sd) {
		if(pdev->write(Null_Word, SD_PRE_CMD_ZEROS, pdev->priv_data)<0) {
			DPRINTK("sending SD_PRE_CMD_ZEROS failed\n");
			pdev->deassert();
			return ERR_SPI_TIMEOUT;
		}
	}
	if(pdev->write(status_cmd, 6, pdev->priv_data)<0) {
		DPRINTK("sending of SEND_STATUS command failed\n");
		pdev->deassert();
		return ERR_SPI_TIMEOUT;
	}	
	b1=mmc_wait_response(pdev, MMC_COMMAND_TIMEOUT);
	b2=mmc_wait_response(pdev, MMC_COMMAND_TIMEOUT);

	if(b1 == ERR_MMC_TIMEOUT || b2 == ERR_MMC_TIMEOUT) {
		DPRINTK("!!!!!!!!! No status received !!!!!!!!!\n");
		pdev->deassert();
		return ERR_MMC_TIMEOUT;
	}

	r2 = b2 + (b1 << 8);
	
	if(r2) {
		DPRINT_STAT("STATUS r2: 0x%04x\n", r2);
		//mdelay(10);
	}
	pdev->deassert();

	return r2;
	
	// TODO: Implement in a finer way
	switch(b1) {
		case R1_OK:
			break;
		case R1_IDLE_STATE:
			DPRINT_STAT("R1_IDLE_STATE\n");
			break;
		case R1_ERASE_STATE:
			DPRINT_STAT("R1_ERASE_STATE\n");
			break;
		case R1_ILLEGAL_COMMAND:
			DPRINT_STAT("R1_ILLEGAL_COMMAND\n");
			break;
		case R1_COM_CRC_ERROR:
			DPRINT_STAT("R1_COM_CRC_ERROR\n");
			break;
		case R1_ERASE_SEQ_ERROR:
			DPRINT_STAT("R1_ERASE_SEQ_ERROR\n");
			break;
		case R1_ADDRESS_ERROR:
			DPRINT_STAT("R1_ADDRESS_ERROR\n");
			break;
		case R1_PARAMETER_ERROR:
			DPRINT_STAT("R1_PARAMETER_ERROR\n");
			break;
		case 0xFF:
			DPRINT_STAT("b1: STATUS RESPONSE TIMEOUT\n");
			break;
		default:
			DPRINT_STAT("b1: INVALID STATUS RESPONSE(0x%02x)\n", b1);
			break;
	}
		
	switch(b2) {
		case R2_OK:
			break;
		case R2_CARD_LOCKED:
			DPRINT_STAT("R2_CARD_LOCKED\n");
			break;
		case R2_WP_ERASE_SKIP:
			DPRINT_STAT("R2_WP_ERASE_SKIP/Unlock command failed\n");
			break;
		case R2_ERROR:
			DPRINT_STAT("R2_ERROR\n");
			break;
		case R2_CC_ERROR:
			DPRINT_STAT("R2_CC_ERROR\n");
			break;
		case R2_CARD_ECC_FAILED:
			DPRINT_STAT("R2_CARD_ECC_FAILED\n");
			break;
		case R2_WP_VIOLATION:
			DPRINT_STAT("R2_WP_VIOLATION\n");
			break;
		case R2_ERASE_PARAM:
			DPRINT_STAT("R2_ERASE_PARAM\n");
			break;
		case R2_OUT_OF_RANGE:
			DPRINT_STAT("R2_OUT_OF_RANGE, CSD_Overwrite\n");
			break;
		case 0xFF:
			DPRINT_STAT("b2: STATUS RESPONSE TIMEOUT\n");
			break;
		default:
			DPRINT_STAT("b2: INVALID STATUS RESPONSE(0x%02x)\n", b2);
			break;
	}

	return r2;
}

short mmc_spi_read_mmc_block(struct mmc_spi_dev *pdev, unsigned char* buf, unsigned int address)
{
	unsigned char resp=0xff;
	unsigned short rval = 0;

	//unsigned short status = 0;

	//DPRINTK("adr(r): %08x\n", address);
	pdev->assert();
	if((rval=send_cmd_and_wait(pdev, READ_SINGLE_BLOCK, address, R1_OK, MMC_COMMAND_TIMEOUT))) {
		goto out;
	}

	// Poll for start block token
    if((resp=mmc_wait_response(pdev, MMC_COMMAND_TIMEOUT)) != SBT_S_BLOCK_READ) {
    	DPRINTK("mmc did not send 0xFE(got 0x%x)\n",resp);
		rval = resp;
		goto out;

    }
	// Read data
        if(pdev->read(buf, 512, pdev->priv_data) < 512) {
                DPRINTK("reading 512 bytes of data failed\n");
		rval= ERR_SPI_TIMEOUT;
		goto out;
	}
	//  TODO: read CRC
	out:;

	// send 8 clocks for SD cards
	if(pdev->sd)
		mmc_spi_dummy_clocks(pdev, SD_CLK_CNTRL);
	pdev->deassert();

	return mmc_spi_error_handler(pdev, rval);
}

// Not implemented on Blackfin since DMA reads are a bit troublesome(512 bytes
//   requested could be 514 bytes read.. this could be solved with some hacks though)
#ifdef USE_MULT_BLOCK_READS
short mmc_spi_read_mult_mmc_block(struct mmc_spi_dev *pdev, unsigned char* buf, unsigned int address, int nblocks) 
{
	unsigned char resp=0xff;
	int rval=0;
	int i=0;

	if((rval=send_cmd_and_wait(pdev, READ_MULTIPLE_BLOCK, address, R1_OK, MMC_COMMAND_TIMEOUT))) {
		goto out;
	}
			
	/* idea: read n blocks in one swoop, Data, Garbage and Tokens
	* GGGGGTDDD..512..DDDGGGGTDDDD..512..DDDGGGGT - - - 
	*-------'''''''''''''.....''''''''''''''
	* Then memcpy data to the real buffer, may need a few pages of memory for this
	*/
	for(i=0; i<nblocks; i++) {
		//printk("varv: %d\n",i); 
		// Poll for start block token
		if((resp=mmc_wait_response(pdev, MMC_COMMAND_TIMEOUT)) != SBT_M_BLOCK_READ) {
			DPRINTK("mmc did not send 0xFE(got 0x%x)\n",resp);
			rval= resp;	
			goto out;
		}
		// Read data	
		if(pdev->read(buf+i*MMC_SECTOR_SIZE, MMC_SECTOR_SIZE, pdev->priv_data) < MMC_SECTOR_SIZE) {	
			DPRINTK("reading 512 bytes of data failed\n");
			rval= 1;
			goto out;
		}
	}
	rval = 0;
	out:

	// send 8 clocks for SD cards
	if(pdev->sd)
		mmc_spi_dummy_clocks(pdev, SD_CLK_CNTRL);

	// send stop command
	rval=send_cmd_and_wait(pdev, STOP_TRANSMISSION, address, R1_OK, MMC_COMMAND_TIMEOUT))) {

	return mmc_spi_error_handler(pdev, rval);
	
}
#endif

short mmc_spi_write_mmc_block(struct mmc_spi_dev *pdev, unsigned char* buf, unsigned int address)
{
	unsigned short rval = 0;
	unsigned char resp=0xff;
	unsigned char token;
	unsigned int n_polls=0;

	pdev->assert();

	if((rval=send_cmd_and_wait(pdev, WRITE_BLOCK, address, R1_OK, MMC_COMMAND_TIMEOUT))) {
		DPRINTK("write error at %08x \n", address);
		goto out;
	}

        // send start block token
        token = SBT_S_BLOCK_WRITE;
        if(pdev->write(&token, 1, pdev->priv_data)<0) {
                DPRINTK("sending START_BLOCK_TOKEN failed\n");
		rval= ERR_SPI_TIMEOUT;
		goto out;

        }
        // transmit data block
	if(pdev->write(buf, MMC_SECTOR_SIZE, pdev->priv_data) < MMC_SECTOR_SIZE) {
                DPRINTK("transmission of 512 bytes failed\n");
		rval= ERR_SPI_TIMEOUT;
		goto out;

        }
        // wait for data response token
	if((resp = (mmc_wait_response(pdev, MMC_COMMAND_TIMEOUT) & DR_MASK)) != DR_ACCEPTED) {
		// some card seem to send 0 or 1 at this point, accet that even though not according to MMC spec.
		if(resp != 0 && resp != 1 && resp != 4) {
			DPRINTK("mmc did not send DR_ACCEPTED token(got R1=0x%x)\n",resp);
			rval = ERR_MMC_TIMEOUT;
			goto out;
		}
    }
	//pdev->reset_time(MMC_PROG_TIMEOUT);
	while(1) {
		// Assume answer is not available directly
        TRY_SCHEDULE

		// NOTE: could read response block-wise(effecive if DMA is utilized) to buffer
		// and check for tokens.
		if(pdev->read(&resp, 1, pdev->priv_data) < 0) {
			DPRINTK("busy token read polling failed\n");
			rval = resp;
			goto out;
		}
		switch(resp & DR_MASK) {
			case BUSY_TOKEN:
				break;
			case DR_ACCEPTED:
				goto out;
			case DR_CRC_ERROR:
				rval = DR_CRC_ERROR;
				goto out;
			case DR_WRITE_ERROR:
				rval = DR_WRITE_ERROR;
				goto out;
			default:
				// If any other token is found, return.
				//   status will tell the story.
				goto out;
		}
		//if(pdev->elapsed_time()) {
		if(n_polls++ >= MMC_PROG_TIMEOUT) {
			rval = ERR_MMC_PROG_TIMEOUT;
			goto out;
		}
	}
	out:
	// send 8 clocks for SD cards
	if(pdev->sd)
		mmc_spi_dummy_clocks(pdev, SD_CLK_CNTRL);

	pdev->deassert();

	return mmc_spi_error_handler(pdev, rval);
}

short mmc_spi_write_mult_mmc_block(struct mmc_spi_dev *pdev, unsigned char* buf, unsigned int address, int nblocks)
{
	unsigned short rval = 0;
	unsigned char resp=0xff;
	unsigned char resp_last=0xff;
	unsigned char resp_oldest=0xff;	
	unsigned int tc=0;
	int i=0;
	unsigned char token;
	unsigned int n_polls=0;


	DPRINTK("adr(r): %08x\n", address);
	pdev->assert();
	if((rval=send_cmd_and_wait(pdev, WRITE_MULTIPLE_BLOCK, address, R1_OK, MMC_COMMAND_TIMEOUT))) {
		DPRINTK("NO MBW!!!\n");
		goto out;
	}

	for(i=0; i<nblocks; i++) {
		
		//DPRINTK("block_nr: %d of %d at address %u\n", i, nblocks, address);
		// send start block token
		token = SBT_M_BLOCK_WRITE;
		if(pdev->write(&token, 1, pdev->priv_data)<0) {
			DPRINTKM("sending START_BLOCK_TOKEN failed\n");
			rval= ERR_SPI_TIMEOUT;
			goto stop;
	
		}
	        // transmit data block
		if(pdev->write(buf+i*MMC_SECTOR_SIZE, MMC_SECTOR_SIZE, pdev->priv_data) < MMC_SECTOR_SIZE) {
			DPRINTKM("transmission of 512 bytes failed\n");
			rval= ERR_SPI_TIMEOUT;
			goto stop;
	
		}
        	// wait for data response token
		if((resp = (mmc_wait_response(pdev, MMC_COMMAND_TIMEOUT) & DR_MASK)) != DR_ACCEPTED) {
            if(resp != 0 && resp != 1 && resp != 2 && resp != 4) {
                DPRINTKM("mmc did not send DR_ACCEPTED token(got R1=0x%x)\n",resp);
                rval = ERR_MMC_TIMEOUT;
                goto stop;
            }
		}
		// send 8 clocks for SD cards
		if(pdev->sd)
			mmc_spi_dummy_clocks(pdev, SD_CLK_CNTRL);

	        // wait on busy/error token while MMC is programming new data
		tc=0;
		n_polls=0;
		//pdev->reset_time(MMC_PROG_TIMEOUT);
		
		while(1) {
			// Assume answer is not available directly
            TRY_SCHEDULE

			
			// read response byte-wise(take one or two reads only)
			if(pdev->read(&resp, 1, pdev->priv_data) < 0) {
				DPRINTKM("busy token read polling failed\n");
				rval= ERR_SPI_TIMEOUT;
				goto stop;
			}
			//printk("0x%02x\n", resp);
			//printk("0x%02x\n", resp & DR_MASK);
			switch(resp & DR_MASK) {
				case BUSY_TOKEN:
					break;
				case DR_ACCEPTED:
					goto next;
				case DR_CRC_ERROR:
					rval = DR_CRC_ERROR;
					goto stop;
				case DR_WRITE_ERROR:
					rval = DR_WRITE_ERROR;
					goto stop;
				default:
					//rval = ERR_UNKNOWN_TOK;
					goto next;
			}
			if(n_polls++ >= MMC_PROG_TIMEOUT) {
				rval = ERR_MMC_PROG_TIMEOUT;
				goto stop;
			}
		}
		next:;
	}

	stop:
	// send stop tran token (STT_M_BLOCK_WRITE)
	token = STT_M_BLOCK_WRITE;
	if(pdev->write(&token, 1, pdev->priv_data)<0) {
		DPRINTKM("sending STT_M_BLOCK_WRITE failed\n");
		rval = ERR_SPI_TIMEOUT;
		goto out;
	}

	n_polls=0;
	// wait on final busy/error token while MMC is programming new data. This is done in blocks of length
	// WRB_LEN instead of 1-byte poll(takes several 100 bytes to do at 20Mhz spi clock). Could decrease burst
	// preformance on very fast cards. But improves over-all system performance immensley when using this driver.
	while(1 && !rval) {
		// Assume answer is not available directly
        TRY_SCHEDULE

		// read response block wise
		if(pdev->read(wrb, WRB_LEN, pdev->priv_data) < 0) {
			DPRINTKM("busy token read polling failed");
			rval= ERR_SPI_TIMEOUT;
			goto out;
		}
		if(n_polls++ >= MMC_PROG_TIMEOUT) {
			DPRINTKM("POLL for last token timeout!!(resp=%x, last_resp=%x, resp_oldest=%x)\n", resp, resp_last, resp_oldest);
			rval = ERR_MMC_TIMEOUT;
			msleep(1000);
			goto out;
		}

		// Exit when card raises the data line(busy to done token transition)
		// NOTE: transition is often(allways?) 0x00, 0x00, 0x??, 0xFF, 0xFF, where ?? could be anything != 0xFF for some
		//		card brands. Nothing to do but ignore this last "token". This was a beast and caused trouble
		//		with some off-brands. Either my interpretations of MMC/SD spec was bad. Or some cards are just sloppy made.
		// if(resp == 0xFF && (resp_oldest == 0 || resp_last == 0)) {
		if(wrb[WRB_LEN-1] == 0xFF) {
			DPRINTKM("Got final MBW busy wait done(as 0x%x after %d reads @ %08x..  last_resp=%x, resp_oldest=%x \n",wrb[WRB_LEN-1], n_polls, address, resp_last, resp_oldest);
			goto out;
		}
		resp_oldest=resp_last;
		resp_last = resp;
	}
	out:

	// send 8 clocks for SD cards
	if(pdev->sd)
		mmc_spi_dummy_clocks(pdev, SD_CLK_CNTRL);

	pdev->deassert();

	// Reading status breaks compatibility with some cards, skip it
	//rval=mmc_spi_read_status(pdev);
	return mmc_spi_error_handler(pdev, rval);
}

short mmc_spi_dummy_clocks(struct mmc_spi_dev *pdev, unsigned short nbytes)
{
	int i;

	pdev->force_cs_high = 1;
        for(i=0; i<nbytes; i++) {
                if(pdev->write(Null_Word, 1, pdev->priv_data)<0) {
            			pdev->force_cs_high = 0;
                        return 1;
                }
        }
	pdev->force_cs_high = 0;

	return 0;
}
short mmc_spi_init_card(struct mmc_spi_dev *pdev)
{
        unsigned short cntr=0;

	
	// For testing SPI drivers
	/*
	unsigned char wa[8] = {0x04, 0x11,0xaa, 0x11,0xaa, 0x11,0xaa, 0x11};
	unsigned char rb[8] = {0xaa, 0x11,0xaa, 0x11,0xaa, 0x11,0xaa, 0x11};

	while(1) {
		pdev->write(wa, 1, pdev->priv_data);
		udelay(100);
		pdev->read(rb, 1, pdev->priv_data);
		//DPRINTK("rb: 0x%02x\n", rb[0]);	
	}
	*/
	
    DPRINTK("1\n");
	// for making init process beeing silent
	init_mode = 1;
	
	// save length of log for external usage
	pdev->log_len = LOG_LEN;

	// 10 bytes(80 cycles) with CS de-asserted
	mmc_spi_dummy_clocks(pdev, 10);

    DPRINTK("2\n");

    pdev->assert();
	if(send_cmd_and_wait(pdev, GO_IDLE_STATE, 0, R1_IDLE_STATE, MMC_INIT_TIMEOUT)) {
		return 1;
	}
    pdev->deassert();
    DPRINTK("3\n");

    // Send One Byte Delay
	if(pdev->write(Null_Word, 1, pdev->priv_data)<0) {
        	return 1;
	}

    pdev->assert();
		
	// Look for SD card
	for(cntr=0; cntr< 60; cntr++) {
        	// Send One Byte Delay
                if(pdev->write(Null_Word, 1, pdev->priv_data)<0) {
                        return 1;
                }

		if(send_cmd_and_wait(pdev, APP_CMD, 0, R1_OK, MMC_INIT_TIMEOUT) == 0) {
			goto next;
		}
		if(send_cmd_and_wait(pdev, APP_CMD, 0, R1_IDLE_STATE, MMC_INIT_TIMEOUT)) {
			continue;
		}
		next:
		if(send_cmd_and_wait(pdev, SD_SEND_OP_COND, 0, R1_OK, MMC_INIT_TIMEOUT) == 0) {
 			// Send One Byte Delay and return
			if(pdev->write(Null_Word, 4, pdev->priv_data)<0) {
               pdev->deassert();
				return 1;
			}
			pdev->sd = 1;
			init_mode = 0;
			//udelay(300);
			DPRINTK("SD card found!\n");
            pdev->deassert();
			return 0;
		}
	}

	// poll card by sending CMD1 and wait for card initialization complete
        //DPRINTK("Looking for MMC card...\n");
        for(cntr=0; cntr< 60; cntr++ ) {
                // Send CMD1
		if(send_cmd_and_wait(pdev, SEND_OP_COND, 0, R1_OK, MMC_INIT_TIMEOUT) == 0) {
 			// Send One Byte Delay and return
			if(pdev->write(Null_Word, 1, pdev->priv_data)<0) {
                pdev->deassert();
				return 1;
			}
			pdev->sd = 0;
			init_mode = 0;
			DPRINTK("MMC card found!\n");
            pdev->deassert();
			return 0;
		}
        	// Send One Byte Delay
                if(pdev->write(Null_Word, 1, pdev->priv_data)<0) {
                       pdev->deassert();
			            return 1;
                }
	}
        DPRINTK("doh!\n\n\n");
        pdev->deassert();
		return 1;
}

static unsigned char mmc_wait_response(struct mmc_spi_dev *pdev, unsigned int timeout)
{
	unsigned char card_resp = 0xFF;
	unsigned int n=0;
	// reset time and set to timeout ms
	//pdev->reset_time(timeout);
	while(1) {

		if(pdev->read(&card_resp, 1, pdev->priv_data) < 0) {
			DPRINTK("error: mmc_wait_response read error\n");
			return ERR_SPI_TIMEOUT;
		}
		if(card_resp != 0xFF) {
				//DPRINTK("response after %d counts(%d short before)\n", n, smalls);
 			return card_resp;
		}
		// NOTE: "timeout" in seconds may not be a good idea after all
		//	(by doing pdev->elapsed_time() )
		//	 wait for a specific amount of polls for now.
		if((n++ >= timeout)) {
			// timeout
			DPRINTK("hey! timed out after %d since %d bytes was maximum(latest_cmd=%d)\n",n, timeout, latest_cmd);
			return ERR_MMC_TIMEOUT;
		}
	}
}


#ifdef DEBUG_REGS
short mmc_spi_mmc_spi_get_card_old(struct mmc_spi_dev *pdev)
{
	int i;
	
	struct mmc_card *card = pdev->private_data->card;
	
	unsigned char raw_csd[18]; // 16 byte + 2 byte CRC
	unsigned char raw_cid[18]; // 16 byte + 2 byte CRC
	unsigned short c_size_mult=0;
	unsigned short c_size=0;
	unsigned short read_bl_len=0;
	unsigned int cap = 0;
	
	/*
	unsigned int value=0;
	unsigned int n=0;
	unsigned int cumm_step=127;
	unsigned short csd_step[] = {2,4,2,8,8,8,12,4,1,1,1,1,2,12,3,3,3,3,3,5,5,5,1,2,3,4,1,5,1,1,1,1,2,2,7,1};
	unsigned short cid_step[] = {8,16,8,8,8,8,8,8,8,32,8,7,1};
	BOOL bit=0;
	unsigned char tmp=0;
	*/
	*/	
	memset(raw_cid, 0, 18);
	memset(raw_csd, 0, 18);
	//memset(card.raw_csd, 0, sizeof(card.raw_csd));
	if(read_mmc_reg(pdev, raw_cid, 0)) {
		DPRINTK("CSD register read failed.\n");
		return 1;
	};
	if(read_mmc_reg(pdev, raw_csd, 1)) {
		DPRINTK("CID register read failed.\n");
		return 1;
	}
	/*
	for(i=0;i<128;i++) {
		printk("%d:  %d\n", i, getbit(raw_cid, i));
	}
	printk("\n");
	for(i=0;i<128;i++) {
		printk("%d:  %d\n", i, getbit(raw_csd, i));
	}
	
	for(i=0;i<16;i++) {
		printk("%02x ", raw_cid[i]);
	}
	printk("\n");
	
	for(i=0;i<16;i++) {
		printk("%02x ", raw_csd[i]);
	}
	printk("\n\n CID_REGISTER\n");
	
	while(n<13) {
		value = getvalue(raw_cid, 127-cumm_step, cid_step[n]);
		printk("%d\t%x\t%u\n",cid_step[n], value, value);
		cumm_step=cumm_step-cid_step[n];
		n++;
	}
	n=0;
	cumm_step=127;
	printk("\n\n CSD_REGISTER\n");
	while(n<36) {
		value = getvalue(raw_csd, 127-cumm_step, csd_step[n]);
		printk("%d\t%x\t%u\n",csd_step[n], value, value);
		cumm_step=cumm_step-csd_step[n];
		n++;
	}
	*/
	
	// ********* NO DEBUG CODE FROM HERE ********************* 
	card->csd.mmca_vsn = (raw_csd[0] & 0x3c) >> 2;
	card->csd.cmdclass = (((u16)raw_csd[4]) << 4) | ((raw_csd[5] & 0xf0) >> 4);
	card->csd.tacc_clks = raw_csd[1];
	card->csd.tacc_ns = raw_csd[2];
	card->csd.max_dtr = raw_csd[3];
	card->csd.read_blkbits = raw_csd[5] & 0x0f;
	
	// for calculating capacity(in blocks)
	c_size = ((((u16)raw_csd[6]) & 0x03) << 10) | (((u16)raw_csd[7]) << 2) | (((u16)raw_csd[8]) & 0xc0) >> 6;
	c_size_mult = ((raw_csd[9] & 0x03) << 1) | ((raw_csd[10] & 0x80) >> 7);
	read_bl_len = raw_csd[5] & 0x0f;	
	card->csd.capacity = (c_size+1) * (1 << (c_size_mult + 2));
	
	// for printing capacity in bytes
	cap = (c_size+1) * (1 << (c_size_mult + 2)) * (1 << read_bl_len);
	
	card->cid.manfid = getvalue(raw_cid, 127-127, 8);
	memcpy(card.cid.prod_name, raw_cid+3, 7);
	card->cid.serial = getvalue(raw_cid, 127-47, 32);
	card->cid.oemid = getvalue(raw_cid, 127-119, 16);
	card->cid.year = 1997 + (getvalue(raw_cid, 127-15, 8) & 0x0F);
	card->cid.hwrev = (getvalue(raw_cid, 127-55, 8) & 0xF0) >> 4;
	card->cid.fwrev = getvalue(raw_cid, 127-55, 8) & 0x0F;
	card->cid.month = (getvalue(raw_cid, 127-15, 8) & 0xF0) >> 4;
	
	printk("MMC found:\n\t Capacity: %dM\n\t Name: %s \n\t Rev: %d.%d \n\t Date: %d/%d \n\t Serial: 0x%x (%u)\n", cap/(1024*1024), card.cid.prod_name, card.cid.hwrev, card.cid.fwrev, card.cid.year, card.cid.month, card.cid.serial, card.cid.serial);
	return 0;	
}
#endif
