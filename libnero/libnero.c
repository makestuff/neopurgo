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
#include "libsync.h"
#include "../vendorCommands.h"

static UsbDeviceHandle *m_deviceHandle = NULL;
static uint16 m_endpointSize = 0x0000;
extern char m_neroErrorMessage[];

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

static NeroStatus beginShift(uint32 numBits, SendType sendType, bool isLast, bool isResponseNeeded);
static NeroStatus doSend(const uint8 *sendPtr, uint16 chunkSize);
static NeroStatus doReceive(uint8 *receivePtr, uint16 chunkSize);
static uint16 getEndpointSize(void);

// Find the NeroJTAG device, open it.
//
NeroStatus neroInitialise(uint16 vid, uint16 pid) {

	int returnCode;

	usbInitialise();
	if ( usbOpenDevice(vid, pid, 1, 0, 0, &m_deviceHandle) ) {
		sprintf(m_neroErrorMessage, "neroInitialise(): USB init failed: %s", usbStrError());
		return NERO_USB_INIT;
	}

	if ( syncBulkEndpoints(m_deviceHandle, SYNC_24) != SYNC_SUCCESS ) {
		sprintf(m_neroErrorMessage, "neroInitialise(): %s", syncStrError());
		fail(NERO_SYNC);
	}

	m_endpointSize = getEndpointSize();
	if ( !m_endpointSize ) {
		fail(NERO_ENDPOINTS);
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
		chunkSize = (numBytes>=m_endpointSize) ? m_endpointSize : (uint16)numBytes;
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
		CMD_JTAG_CLOCK_FSM,       // bRequest
		(uint16)transitionCount,  // wValue
		0x0000,                   // wIndex
		(char*)&bitPattern,
		4,                        // wLength
		5000                      // timeout (ms)
	);
	if ( returnCode < 0 ) {
		sprintf(m_neroErrorMessage, "neroClockFSM(): %s", usb_strerror());
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
		CMD_JTAG_CLOCK,    // bRequest
		numClocks&0xFFFF,  // wValue
		numClocks>>16,     // wIndex
		NULL,
		0,                 // wLength
		5000               // timeout (ms)
	);
	if ( returnCode < 0 ) {
		sprintf(m_neroErrorMessage, "neroClocks(): %s", usb_strerror());
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
	int returnCode;
	if ( isLast ) {
		wValue |= (1<<IS_LAST);
	}
	if ( isResponseNeeded ) {
		wValue |= (1<<IS_RESPONSE_NEEDED);
	}
	wValue |= sendType << SEND_TYPE;
	returnCode = usb_control_msg(
		m_deviceHandle,
		USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		CMD_JTAG_CLOCK_DATA,  // bRequest
		wValue,               // wValue
		0x0000,               // wIndex
		(char*)&numBits,      // send bit count
		4,                    // wLength
		5000                  // timeout (ms)
	);
	if ( returnCode < 0 ) {
		sprintf(m_neroErrorMessage, "beginShift(): %s", usb_strerror());
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
		sprintf(m_neroErrorMessage, "doSend(): %s", usb_strerror());
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
		sprintf(m_neroErrorMessage, "doReceive(): %s", usb_strerror());
		return NERO_RECEIVE;
	}
	return NERO_SUCCESS;
}

// Find the size of the EP2OUT & EP4IN bulk endpoints (they must be the same)
//
static uint16 getEndpointSize(void) {
	char descriptorBuffer[1024];  // TODO: Fix by doing two queries
	char *ptr = descriptorBuffer;
	uint8 endpointNum;
	uint16 ep2size = 0;
	uint16 ep4size = 0;
	struct usb_config_descriptor *configDesc;
	struct usb_interface_descriptor *interfaceDesc;
	struct usb_endpoint_descriptor *endpointDesc;
	int returnCode = usb_control_msg(
		m_deviceHandle,
		USB_ENDPOINT_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
		USB_REQ_GET_DESCRIPTOR,    // bRequest
		0x0200,                    // wValue
		0x0000,     // wIndex
		descriptorBuffer,
		1024,                 // wLength
		5000               // timeout (ms)
	);
	if ( returnCode < 0 ) {
		sprintf(m_neroErrorMessage, "getEndpointSize(): Failed to get config descriptor: %s", usb_strerror());
		return 0x0000;
	}
	if ( returnCode > 0 ) {
		configDesc = (struct usb_config_descriptor *)ptr;
		ptr += configDesc->bLength;
		interfaceDesc = (struct usb_interface_descriptor *)ptr;
		ptr += interfaceDesc->bLength;			
		endpointNum = interfaceDesc->bNumEndpoints;
		while ( endpointNum-- ) {
			endpointDesc = (struct usb_endpoint_descriptor *)ptr;
			if ( endpointDesc-> bmAttributes == 0x02 ) {
				if ( endpointDesc->bEndpointAddress == 0x02 ) {
					ep2size = endpointDesc->wMaxPacketSize;
				} else if ( endpointDesc->bEndpointAddress == 0x84 ) {
					ep4size = endpointDesc->wMaxPacketSize;
				}
			}
			ptr += endpointDesc->bLength;
		}
	}
	if ( !ep2size ) {
		sprintf(m_neroErrorMessage, "getEndpointSize(): EP2OUT not found or not configured as a bulk endpoint!");
		return 0x0000;
	}
	if ( !ep4size ) {
		sprintf(m_neroErrorMessage, "getEndpointSize(): EP4IN not found or not configured as a bulk endpoint!");
		return 0x0000;
	}
	if ( ep2size != ep4size ) {
		sprintf(m_neroErrorMessage, "getEndpointSize(): EP2OUT's wMaxPacketSize differs from that of EP4IN");
		return 0x0000;
	}
	return ep2size;
}
