/*
 * (C) Copyright 2013
 * Darren Garnier <dgarnier@reinrag.net>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

//#define DEBUG 1

#include <common.h>
#include <environment.h>
#include <getopt.h>
#include <errno.h>
#include <crc.h>
#include <clock.h>
#include <console.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <fs.h>
#include <kfifo.h>
#include <xfuncs.h>
#include <command.h>
#include <complete.h>

#include "msp430.h"

/* some info from protocol.h */
#define kCmdResetSoft    "$0100*\r\n"
#define kCmdDeviceInfo   "$07*\r\n"
#define kCmdSerialNumber "$0E0000*\r\n"
#define kCmdBacklightOn  "$0C00FF01*\r\n"
#define kCmdBacklightOff "$0C00FF00*\r\n"

enum DCResponses {
    kRspStatus = 0x00,
    kRspDevice = 0x03,
    kRspBacklight = 0x09,
    kRspSerialNumber = 0x0A,
};

typedef uint8_t	        _uint8;
typedef uint16_t	    _uint16;
typedef uint32_t        _uint32;
typedef int8_t	        _int8;
typedef int16_t	        _int16;
typedef int32_t	        _int32;
typedef float           _float32;

#ifndef PACK 
#define PACK __attribute__((packed))
#endif

struct sRspDevice {
	_uint8         rsp;              // = kRspDevice
	_uint8         boardType;        // enum eBoardTypeID
	_uint8         freqUnits;        // tFreqData is  Hz << freqUnits
	_uint8         pluggedCutoff;    // in Hz
	_uint16        fwVersionMajor;
	_uint16        fwVersionMinor;
	_uint32        serialNumber;
	_float32       ticsPerSec;       // time ticks/second
	_uint16        intervalDataQLen; // length of DC interval data queue
	_uint16        meterDataQLen;    // length of DC meter data queue
	_float32       scaleA1;          // analog cals
	_float32       scaleA2;
	_int16         offsetA1;
	_int16         offsetA2;
	_uint32        statusRate;       // ticks period
	_int16         resistorA1;       // pullup on A1 connector pin 3
	_int16         resistorA2;       //           A2
	_float32       vX2;              // what we're pulling up with (~5V)
	_uint16        temperature;
	_uint16        vccHalf;
	_float32       scaleA1H;          // analog HiRes cals
	_float32       scaleA2H;
	_int16         offsetA1H;
	_int16         offsetA2H;
} PACK; // 60

struct sRspBacklight {
	_uint8            rsp;
	_uint8            cmd;
	union {
		_uint16        value;
		struct {
			_uint8      current;      // value to set the current when on
			_uint8      on;           // on or off
		} PACK         settings;
	} PACK            backlight;
} PACK; // 4

struct sRspSerialNumber {
	_uint8         rsp;      // kRspSerialNumber
	char           pad[3];
	_uint32        sn;
	char           fullSN[];
} PACK;

// raw data
struct sRspRaw {
	_uint8         rsp;              // = ?
	_uint8         raw[63];
} PACK; // 64

#define FULLSNLENGTH 20
#define INPUT_FIFO_SIZE 512
#define TIMEOUT_FLUSH    100 * MSECOND
#define TIMEOUT_RESPONSE 200 * MSECOND

struct msp430_device {
	struct console_device *	cdev;
	struct kfifo *			fifo;
	unsigned char 			fullSN[FULLSNLENGTH];
	uint8_t					boardtype;
	uint16_t				major;
	uint16_t				minor;
	uint8_t					bl_current;
	uint8_t					bl_on;
};

static struct msp430_device *g_msp = NULL;

static int input_fifo_fill(struct console_device *cdev, struct kfifo *fifo)
{
	while (cdev->tstc(cdev) && (kfifo_len(fifo) < INPUT_FIFO_SIZE))
		kfifo_putc(fifo, (unsigned char)(cdev->getc(cdev)));
	return kfifo_len(fifo);
}

// get line ending in \n or \r 
static int msp_getline(struct msp430_device *msp,
		      unsigned char *buf, int len, uint64_t timeout)
{
	int i;
	unsigned char c;
	uint64_t start = get_time_ns();

	for (i = 0; i < len-1; ) {
		if (is_timeout(start, timeout)) {
			i = -ETIMEDOUT;
			break;
		}
		//debug("about to fill\n");
		if (input_fifo_fill(msp->cdev, msp->fifo)) {
			//debug("got %d chars\n", kfifo_len(msp->fifo));
			kfifo_getc(msp->fifo, &c);
			buf[i++] = c;
			if (c == '\n' || c == '\r') {
				if (--i) break;
			}
		}
	}
	buf[i]='\0';

	return i;
}

static void msp_puts(struct msp430_device *msp, const unsigned char *s)
{
	// check for characters coming in while sending them out....
	unsigned char c;
	while((c = *s++)) {
		input_fifo_fill(msp->cdev, msp->fifo);
		msp->cdev->putc(msp->cdev, c);		
	}
}

static void msp_flush(struct msp430_device *msp)
{
	uint64_t start;
	struct console_device *cdev = msp->cdev;

	start = get_time_ns();
	while (cdev->tstc(cdev) &&
		!is_timeout(start, TIMEOUT_FLUSH))
			cdev->getc(cdev);
	kfifo_reset(msp->fifo);
}

static struct console_device *get_console_from_device_name(const char *dname)
{
	struct console_device *cdev;
	const char *target;

	for_each_console(cdev) {
		// lookup by device driver, not by console number
		// so we can be sure to get the atmel serial port
		target = dev_id(cdev->dev);
		if (!strcmp(dname, target))
			return cdev;
	}
	return NULL;
}

int __init cpodc2_msp430_init_console(const char *dname)
{
	struct console_device *cdev;
	
	cdev = get_console_from_device_name(dname);
	if (cdev == NULL) return -1;
	
	// deactivate the console in case it was activated...
	dev_set_param(&cdev->class_dev, "active", "");
	// now that we turned it off.. we can "print" again..
	
	// lets set the baud rate ... (defaults already to 8-N-1)
	cdev->setbrg(cdev,38400);
	dev_set_param(&cdev->class_dev, "baudrate", "38400");
	
	g_msp = xzalloc(sizeof(struct msp430_device));
	
	g_msp->cdev = cdev; // keep the pointer
	g_msp->fifo = kfifo_alloc(INPUT_FIFO_SIZE);
	
	// let init script do a reset...
	// msp_reset(g_msp);
	
	return 0;
}


static int unwrap_response(char *buff, char *resp)
{
	char *b, *bend, *rend, *u, i;
	int complete=0;

	bend = buff + INPUT_FIFO_SIZE;
	rend = resp + sizeof(struct sRspRaw);

	u = resp; b = buff;
	
	/* search for beginning of packet */
	while(*b != 0) {
		if (*b == '$') {
			debug("> $");
			if (++b == bend) b=buff;
			break;
		}
		if (++b == bend) b=buff; /* eat all before $ */
	}
	
	/* packet is begun, maybe */
	while( *b != 0 ) {
		if (*b >= '0' && *b <= '9') {
			i = *b - '0';
		} else if (*b >= 'A' && *b <= 'F') {
			i = *b - 'A' + 10;
		} else if (*b == '*') {
			complete=1;
			if (++b == bend) b = buff; /* circular buffer */
			debug("*\n");
			break;
		} else {
			debug("\n unexpected char %c\n ", *b);
			if (++b == bend) b = buff; // skip it
			break;
		}
		if (u == rend) {
			debug("Unwrap would overrun!\n");
			break;
		}
		i=i<<4;
		if (++b == bend) b = buff; /* circular buffer */
		if (*b >= '0' && *b <= '9') {
			i += *b - '0';
		} else if (*b >= 'A' && *b <= 'F') {
			i += *b - 'A' + 10;
		} else if (*b == 0) {
			/* back up */
			if (b == buff) {
				b = bend -1;
			} else b--;
			break;
		} else {
			debug("unexpected char %c", *b);
			if (++b == bend) b = buff; // skip it
			break;
		}
		debug("%02X",i);
		*u++ = i;

		if (++b == bend) b = buff; /* circular buffer */
	}

	return complete;
}

static int msp_wait_parse_response(struct msp430_device *msp)
{
	unsigned char line[INPUT_FIFO_SIZE];
	int len;
	struct sRspRaw rsp;
	struct sRspDevice *rd = (struct sRspDevice *) &rsp;
	struct sRspBacklight *bl = (struct sRspBacklight *) &rsp;

	len = msp_getline(msp, line, INPUT_FIFO_SIZE, TIMEOUT_RESPONSE);
	if (len == 0) {
		debug(" 0 length line.\n");
		return -1;
	}
	if (len < 0)
		return len;

	debug("Got a response:\n< %s\n",line);

	if (unwrap_response(line, (char *) &rsp)) {
		// got a response...
		switch (rsp.rsp) {
		case kRspStatus:
			debug("got MSP status packet.\n");
			break;
		case kRspDevice:
			debug("got device response.\n");
			msp->major= rd->fwVersionMajor;
			msp->minor= rd->fwVersionMinor;
			msp->boardtype = rd->boardType;
			break;
		case kRspBacklight:
			debug("got backlight response.\n");
			msp->bl_current = bl->backlight.settings.current;
			msp->bl_on		= bl->backlight.settings.on;
			break;
		case kRspSerialNumber:
			debug("got serial number response.\n");
			strlcpy(msp->fullSN, ((struct sRspSerialNumber *)&rsp)->fullSN, FULLSNLENGTH);
			break;
		default:
			debug("unexpected msp response (0x%02x)\n", rsp.rsp);
		}
	} else {
		debug("Couldn't parse: %s\n", line);
		return -1;
	}

	return rsp.rsp;
}

static int msp430_read_device(struct msp430_device *msp)
{
	int try, rsp;

	for (try=0; try<3; try++) {

		msp_puts(msp, kCmdDeviceInfo);

		do {
			rsp = msp_wait_parse_response(msp);
			if (rsp == kRspDevice) {
				debug("got device bt = %02x, fw = %02d.%02d\n", msp->boardtype, msp->major, msp->minor);
				return 0;
			}
		} while (rsp != -ETIMEDOUT);

		// timed out.. perhaps needs a "flush"
		msp_flush(msp);

	}
	return rsp;
}

static int msp430_read_serial(struct msp430_device *msp)
{
	int try, rsp;
		
	for (try=0; try<3; try++) {

		msp_puts(msp, kCmdSerialNumber);

		do {
			rsp = msp_wait_parse_response(msp);
			if (rsp == kRspSerialNumber) {
				debug("got the serial number %s\n",msp->fullSN);
				return 0;
			}
		} while (rsp != -ETIMEDOUT);
		msp_flush(msp);
	}
	return rsp;
}

static int msp430_backlight(struct msp430_device *msp, int on)
{
	int ret;
	if (on) {
		msp_puts(msp, kCmdBacklightOn);
	} else {
		msp_puts(msp, kCmdBacklightOff);
	}
	ret = msp_wait_parse_response(msp);
	if (ret > 0) return 0;
	return ret;
}

static int msp430_reset(struct msp430_device *msp)
{	
	msp_puts(msp,kCmdResetSoft);

#ifdef DEBUG
	if (kfifo_len(msp->fifo)) {
		unsigned char c;
		int i;
		printf("%d characters in buffer after reset: ", kfifo_len(msp->fifo));
		i = 40;
		while ( (kfifo_getc(msp->fifo, &c)==0) && (i--) )
			printf("%c",c);
		printf("\n");
	}
#endif
	msp_flush(msp);
	return 0;
}

static int do_msp430(int argc, char *argv[])
{
	int ret, opt;
	
	int info = 0;
	
	if (!g_msp) return -EINVAL;
	
	while ((opt = getopt(argc, argv, "irb:M:m:S:T:")) > 0)
		switch (opt) {
		case 'r':
			if ((ret = msp430_reset(g_msp))) 
				return ret;
			break;
		case 'b':
			if (strcmp(optarg, "on") == 0) {
				msp430_backlight(g_msp, 1);
			} else 
			if (strcmp(optarg, "off") == 0) {
				msp430_backlight(g_msp, 0);
			}
			break;
		case 'i':
			info=1;
			break;
		case 'M':
		case 'm':
		case 'T':
			if (!g_msp->major) {
				if ((ret = msp430_read_device(g_msp)))
					break;
			}
			if (optarg) {
				if (opt == 'M')
					export_env_ull(optarg, g_msp->major);
				else if (opt == 'm')
					export_env_ull(optarg, g_msp->minor);
				else
					export_env_ull(optarg, g_msp->boardtype);
			}
			break;
		case 'S':
			if (g_msp->fullSN[0] == '\0') {
				if ((ret = msp430_read_serial(g_msp)))
					break;
			}
			setenv(optarg,g_msp->fullSN);
			break;
		default:
			debug("unexpected option 0x%02x (%c)\n", opt,opt);
			return -EINVAL;
		}

		if (info) {
			printf("MSP430 MCU:");
			if (g_msp->major) {
				printf(" Boardtype: 0x%02X,", g_msp->boardtype);
				printf(" Firmware : %d.%02d,", g_msp->major, g_msp->minor);
			}
			if (g_msp->fullSN[0]) {
				printf(" Serial # : %s", g_msp->fullSN);
			}
			printf("\n");
		}

	return ret;
}

BAREBOX_CMD_HELP_START(msp430)
BAREBOX_CMD_HELP_USAGE("msp430 [OPTIONS]\n")
BAREBOX_CMD_HELP_SHORT("Communicate with MSP430 MCU.\n")
BAREBOX_CMD_HELP_OPT  ("-r          ", "Do a soft reset of the controller\n")
BAREBOX_CMD_HELP_OPT  ("-b [on, off]", "Control backlight\n")
BAREBOX_CMD_HELP_OPT  ("-M MAJOR    ", "Read Major FW version and store into $MAJOR\n")
BAREBOX_CMD_HELP_OPT  ("-m MINOR    ", "Read Minor FW version and store into $MINOR\n")
BAREBOX_CMD_HELP_OPT  ("-S SERIAL   ", "Read Serial Number and store into $SERIAL\n")
BAREBOX_CMD_HELP_OPT  ("-T TYPE     ", "Read board type and store into $TYPE\n")
BAREBOX_CMD_HELP_OPT  ("-i          ", "Print info about board.\n")
	
BAREBOX_CMD_HELP_END

BAREBOX_CMD_START(msp430)
	.cmd		= do_msp430,
	.usage		= "Communicate with MSP430 MCU.",
	BAREBOX_CMD_HELP(cmd_msp430_help)
	BAREBOX_CMD_COMPLETE(empty_complete)
BAREBOX_CMD_END





