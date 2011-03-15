/*
xilprg is covered by the LGPL:

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the
Free Software Foundation, Inc., 59 Temple Place - Suite 330,
Boston, MA 02111-1307, USA.

NeroJTAG support copyright (c) 2010 Chris McClelland
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "xilprg.h"
#include "nero.h"
#include "utils.h"

// Exception thrown when problems occur.
//
class nero_exception : public std::exception {
	string msg;
public:
	explicit nero_exception(const char *s) : msg(s) { }
	virtual ~nero_exception() throw() { }
	virtual const char* what() const throw() {
		return msg.c_str();
	}
};

nero::nero() { }

nero::~nero() { }

// Find the NeroJTAG device, open it.
//
int nero::open() {
	const u32 hackLower = 0x6861636B;
	const u32 hackUpper = 0x4841434B;
	union {
		u32 lword;
		char byteBuf[16];
	} u;
	int returnCode;
	int count;
	usb_init();
	usb_find_busses();
	usb_find_devices();
	m_device = usbOpenDevice(0x04B4, 0x8613, 1, 0, 0);

	count = 0;
	do {
		u.lword = hackLower;
		usb_bulk_write(m_device, USB_ENDPOINT_OUT | 2, u.byteBuf, 4, 100);
		returnCode = usb_bulk_read(m_device, USB_ENDPOINT_IN | 4, u.byteBuf, 16, 100);
		count++;
	} while ( returnCode < 0 && count < 10 );

	if ( count == 10 || returnCode != 4 || u.lword != hackUpper ) {
		char errMsg[200];
		sprintf(errMsg, "Init failed returnCode %d with 0x%08X after %d attempts\n", returnCode, u.lword, count);
		throw nero_exception(errMsg);
	}

	return 0;
}

// Close the cable...drop the USB connection.
//
int nero::close() {
	usb_close(m_device);
	return 0;
}

// Shift data into and out of JTAG chain.
//   In pointer may be ALL_ZEROS (shift in zeros) or ALL_ONES (shift in ones).
//   Out pointer may be NULL (not interested in data shifted out of the chain).
//
void nero::shift(int numBits, void *const ptdi, void *const ptdo, int isLast) {
	const u8 *inPtr = (const u8*)ptdi;
	u8 *outPtr = (u8*)ptdo;
	int numBytes;
	u16 chunkSize;
	SendType sendType;
	bool isResponseNeeded;

	if ( inPtr == ALL_ZEROS ) {
		sendType = SEND_ZEROS;
	} else if ( inPtr == ALL_ONES ) {
		sendType = SEND_ONES;
	} else {
		sendType = SEND_DATA;
	}
	if ( outPtr ) {
		isResponseNeeded = true;
	} else {
		isResponseNeeded = false;
	}
	beginShift(numBits, sendType, (isLast==0) ? false : true, isResponseNeeded);
	//printf("beginShift(0x%08X, %d, %d, %d)\n", numBits, sendType, isLast, isResponseNeeded);
	numBytes = bitsToBytes(numBits);
	while ( numBytes ) {
		chunkSize = (numBytes>=ENDPOINT_SIZE) ? ENDPOINT_SIZE : (u16)numBytes;
		if ( sendType == SEND_DATA ) {
			doSend(inPtr, chunkSize);
			inPtr += chunkSize;
		}
		if ( isResponseNeeded ) {
			doReceive(outPtr, chunkSize);
			outPtr += chunkSize;
		}
		numBytes -= chunkSize;
	}
}

// Apply the supplied bit pattern to TMS, to move the TAP to a specific state.
//
void nero::tms_transition(u32 data, int numBits) {
	//printf("CMD_CLOCK_STATE_MACHINE(0x%08X, %d)\n", data, numBits);
	int returnCode = usb_control_msg(
		m_device,
		USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		CMD_CLOCK_STATE_MACHINE,  // bRequest
		(u16)numBits,             // wValue
		0x0000,                   // wIndex
		(char*)&data,             // send bit count
		4,                        // wLength
		5000                      // timeout in milliseconds
	);
	if ( returnCode < 0 ) {
		throw nero_exception(usb_strerror());
	}
}

// Cycle the TCK line for the given number of times.
//
void nero::tck_cycle(int numCycles) {
	//printf("CMD_CLOCK(%d)\n", numCycles);
	int returnCode = usb_control_msg(
		m_device,
		USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		CMD_CLOCK,         // bRequest
		numCycles&0xFFFF,  // wValue
		numCycles>>16,     // wIndex
		NULL,              // send bit count
		0,                 // wLength
		5000               // timeout in milliseconds
	);
	if ( returnCode < 0 ) {
		throw nero_exception(usb_strerror());
	}
}

// Get the cable description
//
int nero::get_description(string& desc) {
	desc = "MakeStuff NeroJTAG";
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                         *** Private methods below here ***
////////////////////////////////////////////////////////////////////////////////////////////////////

// Kick off a shift operation on the micro. This will be followed by a bunch of sends and receives.
//
void nero::beginShift(u32 numBits, SendType sendType, bool isLast, bool isResponseNeeded) {
	u16 wValue = 0x0000;
	//printf("CMD_CLOCK_DATA(%d, %s, %s, %s)\n", numBits, (sendType==SEND_DATA)?"SEND":"NOSEND", isLast?"LAST":"NOTLAST", isResponseNeeded?"RECEIVE":"NORECEIVE");

	if ( isLast ) {
		wValue |= (1<<IS_LAST);
	}
	if ( isResponseNeeded ) {
		wValue |= (1<<IS_RESPONSE_NEEDED);
	}
	wValue |= sendType << SEND_TYPE;
	int returnCode = usb_control_msg(
		m_device,
		USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		CMD_CLOCK_DATA,  // bRequest
		wValue,          // wValue
		0x0000,          // wIndex
		(char*)&numBits, // send bit count
		4,               // wLength
		5000             // timeout in milliseconds
	);
	if ( returnCode < 0 ) {
		throw nero_exception(usb_strerror());
	}
}

void nero::doSend(const u8 *sendPtr, u16 chunkSize) {
	int returnCode = usb_bulk_write(
		m_device,
		USB_ENDPOINT_OUT | 2,    // write to endpoint 2
		(char *)sendPtr,         // write from send buffer
		chunkSize,               // write this many bytes
		5000                     // timeout in milliseconds
	);
	if ( returnCode < 0 ) {
		throw nero_exception(usb_strerror());
	}
}

void nero::doReceive(u8 *receivePtr, u16 chunkSize) {
	int returnCode = usb_bulk_read(
		m_device,
		USB_ENDPOINT_IN | 4,    // read from endpoint 4
		(char *)receivePtr,     // read into the receive buffer
		chunkSize,              // read this many bytes
		5000                    // timeout in milliseconds
	);
	if ( returnCode < 0 ) {
		throw nero_exception(usb_strerror());
	}
}

// Find the descriptor of the first occurance of the specified device
//
usb_dev_handle *nero::usbOpenDevice(u16 vid, u16 pid, int configuration, int interface, int alternateInterface) {
	struct usb_bus *bus;
	struct usb_device *thisDevice;
	usb_dev_handle *deviceHandle;
	int returnCode;
	bus = usb_get_busses();
	if ( bus ) {
		// This system has one or more USB buses...let's step through them looking for the specified VID/PID
		//
		do {
			thisDevice = bus->devices;
			while ( thisDevice && (thisDevice->descriptor.idVendor != vid || thisDevice->descriptor.idProduct != pid) ) {
				thisDevice = thisDevice->next;
			}
			bus = bus->next;
		} while ( bus && !thisDevice );  // will break out if I run out of buses, or if device found
		if ( !thisDevice ) {
			// The VID/PID was not found after scanning all buses
			//
			throw nero_exception("Device not found");
		} else {
			// The VID/PID was found; let's see if we can open the device
			//
			deviceHandle = usb_open(thisDevice);
			if ( deviceHandle == NULL ) {
				throw nero_exception(usb_strerror());
			}
			returnCode = usb_set_configuration(deviceHandle, configuration);
			if ( returnCode < 0 ) {
				throw nero_exception(usb_strerror());
			}
			returnCode = usb_claim_interface(deviceHandle, interface);
			if ( returnCode < 0 ) {
				throw nero_exception(usb_strerror());
			}
			if ( alternateInterface ) {
				returnCode = usb_set_altinterface(deviceHandle, alternateInterface);
				if ( returnCode < 0 ) {
					throw nero_exception(usb_strerror());
				}
			}
			return deviceHandle;
		}
	} else {
		// No USB buses on this system!?!?
		//
		throw nero_exception("No USB buses found");
	}
}
