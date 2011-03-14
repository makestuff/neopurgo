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
#include <fx2regs.h>
#include <fx2macros.h>
#include <delay.h>
#include "jtag.h"
#include "defs.h"

static uint32 m_numBits = 0UL;
static uint8 m_flagByte = 0x00;

// JTAG-clock the supplied byte into TDI, LSB first.
//
// Lifted from:
//   http://ixo-jtag.svn.sourceforge.net/viewvc/ixo-jtag/usb_jtag/trunk/device/c51/hw_nexys.c
//
static void shiftOut(uint8 c) {
	/* Shift out byte c:
	 *
	 * 8x {
	 *   Output least significant bit on TDI
	 *   Raise TCK
	 *   Shift c right
	 *   Lower TCK
	 * }
	 */
	
	(void)c; /* argument passed in DPL */
	
	_asm
		mov  A,DPL
		;; Bit0
		rrc  A
		mov  _TDI,C
		setb _TCK
		;; Bit1
		rrc  A
		clr  _TCK
		mov  _TDI,C
		setb _TCK
		;; Bit2
		rrc  A
		clr  _TCK
		mov  _TDI,C
		setb _TCK
		;; Bit3
		rrc  A
		clr  _TCK
		mov  _TDI,C
		setb _TCK
		;; Bit4
		rrc  A
		clr  _TCK
		mov  _TDI,C
		setb _TCK
		;; Bit5
		rrc  A
		clr  _TCK
		mov  _TDI,C
		setb _TCK
		;; Bit6
		rrc  A
		clr  _TCK
		mov  _TDI,C
		setb _TCK
		;; Bit7
		rrc  A
		clr  _TCK
		mov  _TDI,C
		setb _TCK
		nop
		clr  _TCK
		ret
	_endasm;
}

// JTAG-clock the supplied byte into TDI, MSB first. Return the byte clocked out of TDO.
//
// Lifted from:
//   http://ixo-jtag.svn.sourceforge.net/viewvc/ixo-jtag/usb_jtag/trunk/device/c51/hw_nexys.c
//
static uint8 shiftInOut(uint8 c) {
	/* Shift out byte c, shift in from TDO:
	 *
	 * 8x {
	 *   Read carry from TDO
	 *   Output least significant bit on TDI
	 *   Raise TCK
	 *   Shift c right, append carry (TDO) at left
	 *   Lower TCK
	 * }
	 * Return c.
	 */
	
	(void)c; /* argument passed in DPL */
	
	_asm
		mov  A, DPL

		;; Bit0
		mov  C, _TDO
		rrc  A
		mov  _TDI, C
		setb _TCK
		clr  _TCK
		;; Bit1
		mov  C, _TDO
		rrc  A
		mov  _TDI, C
		setb _TCK
		clr  _TCK
		;; Bit2
		mov  C, _TDO
		rrc  A
		mov  _TDI, C
		setb _TCK
		clr  _TCK
		;; Bit3
		mov  C, _TDO
		rrc  A
		mov  _TDI, C
		setb _TCK
		clr  _TCK
		;; Bit4
		mov  C, _TDO
		rrc  A
		mov  _TDI, C
		setb _TCK
		clr  _TCK
		;; Bit5
		mov  C, _TDO
		rrc  A
		mov  _TDI, C
		setb _TCK
		clr  _TCK
		;; Bit6
		mov  C, _TDO
		rrc  A
		mov  _TDI, C
		setb _TCK
		clr  _TCK
		;; Bit7
		mov  C, _TDO
		rrc  A
		mov  _TDI, C
		setb _TCK
		nop
		clr  _TCK
		
		mov  DPL, A
		ret
	_endasm;

	/* return value in DPL */

	return c;
}

// Kick off a shift operation. Next time jtagExecuteShift() runs, it will execute the shift.
//
void jtagBeginShift(uint32 numBits, uint8 flagByte) {
	m_numBits = numBits;
	m_flagByte = flagByte;
}

// The minimum number of bytes necessary to store x bits
//
#define bitsToBytes(x) ((x>>3) + (x&7 ? 1 : 0))

// Actually execute the shift operation initiated by jtagBeginShift(). This is done in a
// separate method because vendor commands cannot read & write to bulk endpoints.
//
void jtagExecuteShift(void) {
	// Are there any JTAG send/receive operations to execute?
	if ( m_numBits ) {
		if ( (m_flagByte & bmSENDMASK) == bmSENDDATA ) {
			if ( m_flagByte & bmNEEDRESPONSE ) {
				// The host is giving us data, and is expecting a response (xdr)
				uint16 bitsRead, bitsRemaining, bytesRead, bytesRemaining;
				uint8 *inPtr, *outPtr;
				while ( m_numBits ) {
					while ( EP2468STAT & bmEP2EMPTY );  // Wait for some EP2OUT data
					while ( EP2468STAT & bmEP4FULL );   // Wait for space for EP4IN data
					bitsRead = (m_numBits >= (ENDPOINT_SIZE<<3)) ? ENDPOINT_SIZE<<3 : m_numBits;
					bytesRead = MAKEWORD(EP2BCH, EP2BCL);
					if ( bytesRead != bitsToBytes(bitsRead) ) {
						// Protocol violation - give up
						m_numBits = 0UL;
						break;
					}

					inPtr = EP2FIFOBUF;
					outPtr = EP4FIFOBUF;
					if ( bitsRead == m_numBits ) {
						// This is the last chunk
						uint8 tdoByte, tdiByte, leftOver, i;
						bitsRemaining = (bitsRead-1) & 0xFFF8;        // Now an integer number of bytes
						leftOver = (uint8)(bitsRead - bitsRemaining); // How many bits in last byte (1-8)
						bytesRemaining = (bitsRemaining>>3);
						while ( bytesRemaining-- ) {
							*outPtr++ = shiftInOut(*inPtr++);
						}
						tdiByte = *inPtr++;  // Now do the bits in the final byte
						tdoByte = 0x00;
						i = 1;
						while ( i && leftOver ) {
							leftOver--;
							if ( (m_flagByte & bmISLAST) && !leftOver ) {
								TMS = 1; // Exit Shift-DR state on next clock
							}
							TDI = tdiByte & 1;
							tdiByte >>= 1;
							if ( TDO ) {
								tdoByte |= i;
							}
							TCK = 1;
							TCK = 0;
							i <<= 1;
						}
						*outPtr++ = tdoByte;
					} else {
						// This is not the last chunk
						bytesRemaining = (bitsRead>>3);
						while ( bytesRemaining-- ) {
							*outPtr++ = shiftInOut(*inPtr++);
						}
					}
					SYNCDELAY; EP4BCH = MSB(bytesRead);  // Initiate send of the copied data
					SYNCDELAY; EP4BCL = LSB(bytesRead);
					SYNCDELAY; OUTPKTEND = bmSKIP | 2;   // Acknowledge receipt of this packet

					m_numBits -= bitsRead;
				}
			} else {
				// The host is giving us data, but does not need a response (xdn)
				uint16 bitsRead, bitsRemaining, bytesRead, bytesRemaining;
				uint16 i;
				while ( m_numBits ) {
					while ( EP2468STAT & bmEP2EMPTY );  // Wait for some EP2OUT data
					bitsRead = (m_numBits >= (ENDPOINT_SIZE<<3)) ? ENDPOINT_SIZE<<3 : m_numBits;
					bytesRead = MAKEWORD(EP2BCH, EP2BCL);
					if ( bytesRead != bitsToBytes(bitsRead) ) {
						// Protocol violation - give up
						m_numBits = 0UL;
						break;
					}

					inPtr = EP2FIFOBUF;
					if ( bitsRead == m_numBits ) {
						// This is the last chunk
						uint8 tdiByte, leftOver, i;
						bitsRemaining = (bitsRead-1) & 0xFFF8;        // Now an integer number of bytes
						leftOver = (uint8)(bitsRead - bitsRemaining); // How many bits in last byte (1-8)
						bytesRemaining = (bitsRemaining>>3);
						while ( bytesRemaining-- ) {
							shiftOut(*inPtr++);
						}
						tdiByte = *inPtr++;  // Now do the bits in the final byte
						i = 1;
						while ( i && leftOver ) {
							leftOver--;
							if ( (m_flagByte & bmISLAST) && !leftOver ) {
								TMS = 1; // Exit Shift-DR state on next clock
							}
							TDI = tdiByte & 1;
							tdiByte >>= 1;
							TCK = 1;
							TCK = 0;
							i <<= 1;
						}
					} else {
						// This is not the last chunk
						bytesRemaining = (bitsRead>>3);
						while ( bytesRemaining-- ) {
							shiftOut(*inPtr++);
						}
					}

					SYNCDELAY; OUTPKTEND = bmSKIP | 2;   // Acknowledge receipt of this packet
					m_numBits -= bitsRead;
				}
			}
		} else {
			if ( m_flagByte & bmNEEDRESPONSE ) {
				// The host is not giving us data, but is expecting a response (x0r)
				uint16 bitsRead, bitsRemaining, bytesRead, bytesRemaining;
				uint8 tdiByte;
				if ( (m_flagByte & bmSENDMASK) == bmSENDZEROS ) {
					tdiByte = 0x00;
				} else {
					tdiByte = 0xFF;
				}
				while ( m_numBits ) {
					while ( EP2468STAT & bmEP4FULL );   // Wait for space for EP4IN data
					bitsRead = (m_numBits >= (ENDPOINT_SIZE<<3)) ? ENDPOINT_SIZE<<3 : m_numBits;
					bytesRead = bitsToBytes(bitsRead);

					inPtr = EP2FIFOBUF;
					outPtr = EP4FIFOBUF;
					if ( bitsRead == m_numBits ) {
						// This is the last chunk
						uint8 tdoByte, leftOver, i;
						bitsRemaining = (bitsRead-1) & 0xFFF8;        // Now an integer number of bytes
						leftOver = (uint8)(bitsRead - bitsRemaining); // How many bits in last byte (1-8)
						bytesRemaining = (bitsRemaining>>3);
						while ( bytesRemaining-- ) {
							*outPtr++ = shiftInOut(tdiByte);
						}
						tdoByte = 0x00;
						i = 1;
						TDI = tdiByte & 1;
						while ( i && leftOver ) {
							leftOver--;
							if ( (m_flagByte & bmISLAST) && !leftOver ) {
								TMS = 1; // Exit Shift-DR state on next clock
							}
							if ( TDO ) {
								tdoByte |= i;
							}
							TCK = 1;
							TCK = 0;
							i <<= 1;
						}
						*outPtr++ = tdoByte;
					} else {
						// This is not the last chunk
						bytesRemaining = (bitsRead>>3);
						while ( bytesRemaining-- ) {
							*outPtr++ = shiftInOut(tdiByte);
						}
					}
					SYNCDELAY; EP4BCH = MSB(bytesRead);  // Initiate send of the data
					SYNCDELAY; EP4BCL = LSB(bytesRead);
					m_numBits -= bitsRead;
				}
			} else {
				// The host is not giving us data, and does not need a response (x0n)
				uint32 bitsRemaining, bytesRemaining;
				uint8 tdiByte, leftOver;
				if ( (m_flagByte & bmSENDMASK) == bmSENDZEROS ) {
					tdiByte = 0x00;
				} else {
					tdiByte = 0xFF;
				}
				bitsRemaining = (m_numBits-1) & 0xFFFFFFF8;    // Now an integer number of bytes
				leftOver = (uint8)(m_numBits - bitsRemaining); // How many bits in last byte (1-8)
				bytesRemaining = (bitsRemaining>>3);
				while ( bytesRemaining-- ) {
					shiftOut(tdiByte);
				}
				TDI = tdiByte & 1;
				while ( leftOver ) {
					leftOver--;
					if ( (m_flagByte & bmISLAST) && !leftOver ) {
						TMS = 1; // Exit Shift-DR state on next clock
					}
					TCK = 1;
					TCK = 0;
				}
			}
		}
	}
}

// Transition the JTAG state machine to another state: clock "transitionCount" bits from
// "bitPattern" into TMS, LSB-first.
//
void jtagClockFSM(uint32 bitPattern, uint8 transitionCount) {
	while ( transitionCount-- ) {
		TMS = bitPattern & 1;
		TCK = 1;
		TCK = 0;
		bitPattern >>= 1;
	}
}

// Keep TMS and TDI as they are, and clock the JTAG state machine "numClocks" times.
//
void jtagClocks(uint32 numClocks) {
	while ( numClocks-- ) {
		TCK = 1;
		TCK = 0;
	}
}
