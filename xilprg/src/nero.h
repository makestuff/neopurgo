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

#ifndef _NERO_H_INCLUDED_
#define _NERO_H_INCLUDED_

#include "cable.h"
#include "usb.h"

#define ENDPOINT_SIZE 512

#define bitsToBytes(x) ((x>>3) + (x&7 ? 1 : 0))

enum SendType {
	SEND_ZEROS,
	SEND_ONES,
	SEND_DATA,
	SEND_MASK
};

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

class nero : public cable {

	usb_dev_handle *m_device;

	void beginShift(u32 numBits, SendType sendType, bool isLast, bool isResponseNeeded);
	void doSend(const u8 *sendPtr, u16 chunkSize);
	void doReceive(u8 *receivePtr, u16 chunkSize);
	static usb_dev_handle *usbOpenDevice(u16 vid, u16 pid, int configuration, int interface, int alternateInterface);

public:
	nero();
	virtual ~nero();

	virtual int open();
	virtual int close();
	virtual int get_description(string&);

	virtual void tms_transition(u32 seq, int cnt);
	virtual void shift(int num_bits, void* tdi, void* tdo, int isLast);
	virtual void tck_cycle(int n);
};

#endif
