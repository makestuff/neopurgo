/* 
 * Copyright (C) 2011 Chris McClelland
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *  
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <usbwrap.h>
#include <types.h>
#include "libnero.h"

static UsbDeviceHandle *m_deviceHandle = NULL;
extern char m_neroErrorMessage[];

#define ENDPOINT_SIZE 512
#define fail(x) returnCode = x; goto cleanup
#define bitsToBytes(x) ((x>>3) + (x&7 ? 1 : 0))
#define checkReturn() if ( status ) { return status; }

typedef enum {
	SEND_ZEROS,
	SEND_ONES,
	SEND_DATA,
	SEND_MASK
} SendType;

enum {
	IS_RESPONSE_NEEDED = 0,
	IS_LAST = 1,
	SEND_TYPE = 2
};

enum {
	CMD_CLOCK_DATA = 0x80,
	CMD_CLOCK_STATE_MACHINE,
	CMD_CLOCK
};

static NeroStatus beginShift(uint32 numBits, SendType sendType, bool isLast, bool isResponseNeeded);
static NeroStatus doSend(const uint8 *sendPtr, uint16 chunkSize);
static NeroStatus doReceive(uint8 *receivePtr, uint16 chunkSize);

// Find the NeroJTAG device, open it.
//
NeroStatus neroInitialise(uint16 vid, uint16 pid) {
	const uint32 hackLower = 0x6861636B;
	const uint32 hackUpper = 0x4841434B;
	union {
		uint32 lword;
		char bytes[16];
	} u;
	int returnCode;
	int count;

	usbInitialise();
	if ( usbOpenDevice(vid, pid, 1, 0, 0, &m_deviceHandle) ) {
		sprintf(m_neroErrorMessage, "neroInitialise(): USB init failed: %s", usbStrError());
		fail(NERO_USB_INIT);
	}

	count = 0;
	do {
		u.lword = hackLower;
		usb_bulk_write(m_deviceHandle, USB_ENDPOINT_OUT | 2, u.bytes, 4, 100);
		returnCode = usb_bulk_read(m_deviceHandle, USB_ENDPOINT_IN | 4, u.bytes, 16, 100);
		count++;
	} while ( returnCode < 0 && count < 10 );
	if ( count == 10 ) {
		sprintf(
			m_neroErrorMessage,
			"neroInitialise(): Sync failed after %d attempts: %s",
			count, usbStrError());
		fail(NERO_SYNC);
	}
	if ( returnCode != 4 ) {
		sprintf(
			m_neroErrorMessage,
			"neroInitialise(): Sync read back %d bytes instead of the expected 4",
			returnCode);
		fail(NERO_SYNC);
	}
	if ( u.lword != hackUpper ) {
		sprintf(
			m_neroErrorMessage,
			"neroInitialise(): Sync read back 0x%08lX instead of the expected 0x%08lX",
			u.lword, hackUpper);
		fail(NERO_SYNC);
	}

	return NERO_SUCCESS;

cleanup:
	usbClose(m_deviceHandle);
	m_deviceHandle = NULL;
	return returnCode;
}

// Close the cable...drop the USB connection.
//
void neroClose(void) {
	if ( m_deviceHandle ) {
		usbClose(m_deviceHandle);
	}
	m_deviceHandle = NULL;
}

// Shift data into and out of JTAG chain.
//   In pointer may be ZEROS (shift in zeros) or ONES (shift in ones).
//   Out pointer may be NULL (not interested in data shifted out of the chain).
//
NeroStatus neroShift(uint32 numBits, const uint8 *inData, uint8 *outData, bool isLast) {
	uint32 numBytes;
	uint16 chunkSize;
	SendType sendType;
	bool isResponseNeeded;
	NeroStatus status;

	if ( inData == ZEROS ) {
		sendType = SEND_ZEROS;
	} else if ( inData == ONES ) {
		sendType = SEND_ONES;
	} else {
		sendType = SEND_DATA;
	}
	if ( outData ) {
		isResponseNeeded = true;
	} else {
		isResponseNeeded = false;
	}
	status = beginShift(numBits, sendType, isLast, isResponseNeeded); checkReturn();	
	numBytes = bitsToBytes(numBits);
	while ( numBytes ) {
		chunkSize = (numBytes>=ENDPOINT_SIZE) ? ENDPOINT_SIZE : (uint16)numBytes;
		if ( sendType == SEND_DATA ) {
			status = doSend(inData, chunkSize); checkReturn();
			inData += chunkSize;
		}
		if ( isResponseNeeded ) {
			status = doReceive(outData, chunkSize); checkReturn();
			outData += chunkSize;
		}
		numBytes -= chunkSize;
	}
	return NERO_SUCCESS;
}

// Apply the supplied bit pattern to TMS, to move the TAP to a specific state.
//
NeroStatus neroClockFSM(uint32 bitPattern, uint8 transitionCount) {
	int returnCode = usb_control_msg(
		m_deviceHandle,
		USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		CMD_CLOCK_STATE_MACHINE,  // bRequest
		(uint16)transitionCount,  // wValue
		0x0000,                   // wIndex
		(char*)&bitPattern,
		4,                        // wLength
		5000                      // timeout (ms)
	);
	if ( returnCode < 0 ) {
		sprintf(m_neroErrorMessage, "neroClockFSM(): %s", usbStrError());
		return NERO_CLOCKFSM;
	}
	return NERO_SUCCESS;
}

// Cycle the TCK line for the given number of times.
//
NeroStatus neroClocks(uint32 numClocks) {
	int returnCode = usb_control_msg(
		m_deviceHandle,
		USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		CMD_CLOCK,         // bRequest
		numClocks&0xFFFF,  // wValue
		numClocks>>16,     // wIndex
		NULL,
		0,                 // wLength
		5000               // timeout (ms)
	);
	if ( returnCode < 0 ) {
		sprintf(m_neroErrorMessage, "neroClocks(): %s", usbStrError());
		return NERO_CLOCKS;
	}
	return NERO_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                         *** Private methods below here ***
////////////////////////////////////////////////////////////////////////////////////////////////////

// Kick off a shift operation on the micro. This will be followed by a bunch of sends and receives.
//
static NeroStatus beginShift(uint32 numBits, SendType sendType, bool isLast, bool isResponseNeeded) {
	uint16 wValue = 0x0000;
	if ( isLast ) {
		wValue |= (1<<IS_LAST);
	}
	if ( isResponseNeeded ) {
		wValue |= (1<<IS_RESPONSE_NEEDED);
	}
	wValue |= sendType << SEND_TYPE;
	int returnCode = usb_control_msg(
		m_deviceHandle,
		USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		CMD_CLOCK_DATA,  // bRequest
		wValue,          // wValue
		0x0000,          // wIndex
		(char*)&numBits, // send bit count
		4,               // wLength
		5000             // timeout (ms)
	);
	if ( returnCode < 0 ) {
		sprintf(m_neroErrorMessage, "beginShift(): %s", usbStrError());
		return NERO_BEGIN_SHIFT;
	}
	return NERO_SUCCESS;
}

static NeroStatus doSend(const uint8 *sendPtr, uint16 chunkSize) {
	int returnCode = usb_bulk_write(
		m_deviceHandle,
		USB_ENDPOINT_OUT | 2,    // write to endpoint 2
		(char *)sendPtr,         // write from send buffer
		chunkSize,               // write this many bytes
		5000                     // timeout in milliseconds
	);
	if ( returnCode < 0 ) {
		sprintf(m_neroErrorMessage, "doSend(): %s", usbStrError());
		return NERO_SEND;
	}
	return NERO_SUCCESS;
}

static NeroStatus doReceive(uint8 *receivePtr, uint16 chunkSize) {
	int returnCode = usb_bulk_read(
		m_deviceHandle,
		USB_ENDPOINT_IN | 4,    // read from endpoint 4
		(char *)receivePtr,     // read into the receive buffer
		chunkSize,              // read this many bytes
		5000                    // timeout in milliseconds
	);
	if ( returnCode < 0 ) {
		sprintf(m_neroErrorMessage, "doReceive(): %s", usbStrError());
		return NERO_RECEIVE;
	}
	return NERO_SUCCESS;
}
