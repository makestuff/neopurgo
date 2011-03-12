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
#ifndef JTAG_H
#define JTAG_H

#include <types.h>

// NeroJTAG vendor commands
#define CMD_NEROJTAG_CLOCK_DATA 0x80
#define CMD_NEROJTAG_CLOCK_FSM  0x81
#define CMD_NEROJTAG_CLOCK      0x82

// Addressable bits on Port D for the four JTAG lines (named after the FPGA pins they connect to)
// TDO is an input, the rest are outputs.
sbit at 0xB0      TDO; /* Port D.0 */
sbit at 0xB2      TDI; /* Port D.2 */
sbit at 0xB3      TMS; /* Port D.3 */
sbit at 0xB4      TCK; /* Port D.4 */

// Equivalent bitmasks for OED and IOD.
#define bmTDO     bmBIT0
#define bmTDI     bmBIT2
#define bmTMS     bmBIT3
#define bmTCK     bmBIT4

// Macros for NeroJTAG implementation
#define ENDPOINT_SIZE 512
#define bmNEEDRESPONSE (1<<0)
#define bmISLAST       (1<<1)
#define bmSENDZEROS    (0<<2)
#define bmSENDONES     (1<<2)
#define bmSENDDATA     (2<<2)
#define bmSENDMASK     (3<<2)

// Public functions
void jtagBeginShift(uint32 nBits, uint8 flags);
void jtagExecuteShift(void);
void jtagClockFSM(uint32 bitPattern, uint8 transitionCount);
void jtagClocks(uint32 numClocks);

#endif
