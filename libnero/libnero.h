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
#ifndef LIBNERO_H
#define LIBNERO_H

#include <types.h>

// Various defines...
#define ZEROS (const uint8*)NULL
#define ONES (const uint8*)-1

// Possible return codes
typedef enum {
	NERO_SUCCESS,
	NERO_USB_INIT,
	NERO_SYNC,
	NERO_CLOCKFSM,
	NERO_CLOCKS,
	NERO_BEGIN_SHIFT,
	NERO_SEND,
	NERO_RECEIVE
} NeroStatus;

#ifdef __cplusplus
extern "C" {
#endif

// Initialise the connection to the device implementing the NeroJTAG protocol
NeroStatus neroInitialise(uint16 vid, uint16 pid);

// Drop the connection to the device implementing the NeroJTAG protocol
void neroClose(void);

// Shift "numBits" bits from "inData" into TDI, at the same time shifting the same number of bits
// from TDO into "outData". If "isLast" is true, leave Shift-DR state on final bit. If you want
// inData to be all zeros or all ones, you can use ZEROS or ONES respectively. This is more
// efficient than composing an array containing all zeros or all 0xFFs.
NeroStatus neroShift(uint32 numBits, const uint8 *inData, uint8 *outData, bool isLast);

// Clock "transitionCount" bits from "bitPattern" into TMS, starting with the LSB.
NeroStatus neroClockFSM(uint32 bitPattern, uint8 transitionCount);

// Toggle TCK "numClocks" times.
NeroStatus neroClocks(uint32 numClocks);

// Get a message describing the last error.
const char *neroStrError(void);

#ifdef __cplusplus
}
#endif

#endif
