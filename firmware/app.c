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
#include <setupdat.h>
#include <types.h>
#include "prom.h"
#include "jtag.h"
#include "sync.h"
#include "defs.h"
#include "../vendorCommands.h"

// Function declarations
void fifoSendPromData(uint32 bytesToSend);

// General-purpose diagnostic code, for debugging. See CMD_GET_DIAG_CODE vendor command.
xdata uint8 m_diagnosticCode = 0;

// Called once at startup
//
void mainInit(void) {

	xdata uint8 thisByte = 0xFF;
	xdata uint16 blockSize;

	// Global settings
	SYNCDELAY; REVCTL = (bmDYN_OUT | bmENH_PKT);
	SYNCDELAY; CPUCS = bmCLKSPD1;  // 48MHz

	// Check if the FPGA is running (it drives "10" on fifoAddr)
	//SYNCDELAY; IFCONFIG = 0x00;
	//if ( (IOA & (bmBIT4|bmBIT5)) == (bmBIT4|bmBIT5) ) {
	//	thisByte = 0x00;  // The FPGA is not running
	//}

	// Drive IFCLK at 48MHz, enable slave FIFOs
	SYNCDELAY; IFCONFIG = (bmIFCLKSRC | bm3048MHZ | bmIFCLKOE | bmFIFOS);

	// EP2OUT & EP4IN are handled by firmware, EP6OUT & EP8IN connect to Slave FIFOs
	SYNCDELAY; EP2CFG = (bmVALID | bmBULK | bmBUF2X);
	SYNCDELAY; EP4CFG = (bmVALID | bmBULK | bmBUF2X | bmDIR);
	SYNCDELAY; EP6CFG = (bmVALID | bmBULK | bmBUF2X);
	SYNCDELAY; EP8CFG = (bmVALID | bmBULK | bmBUF2X | bmDIR);

	// Reset all the FIFOs
	SYNCDELAY; FIFORESET = bmNAKALL;
	SYNCDELAY; FIFORESET = bmNAKALL | 2;  // reset EP2
	SYNCDELAY; FIFORESET = bmNAKALL | 4;  // reset EP4
	SYNCDELAY; FIFORESET = bmNAKALL | 6;  // reset EP6
	SYNCDELAY; FIFORESET = bmNAKALL | 8;  // reset EP8
	SYNCDELAY; FIFORESET = 0x00;

	// Arm the OUT buffers. Done twice because they're double-buffered
	SYNCDELAY; OUTPKTEND = bmSKIP | 2;  // EP2OUT
	SYNCDELAY; OUTPKTEND = bmSKIP | 2;
	SYNCDELAY; OUTPKTEND = bmSKIP | 6;  // EP6OUT
	SYNCDELAY; OUTPKTEND = bmSKIP | 6;

	// EP2OUT & EP4IN handled by firmware, so no FIFOs
	SYNCDELAY; EP2FIFOCFG = 0x00;
	SYNCDELAY; EP4FIFOCFG = 0x00;

	// EP6OUT & EP8IN connected to Slave FIFOs, so AUTOOUT & AUTOIN respectively
	SYNCDELAY; EP6FIFOCFG = bmAUTOOUT;
	SYNCDELAY; EP8FIFOCFG = bmAUTOIN;
	SYNCDELAY;
	
	// Auto-commit 512-byte packets from EP8IN (master may commit early by asserting PKTEND)
	SYNCDELAY; EP8AUTOINLENH = 0x02;
	SYNCDELAY; EP8AUTOINLENL = 0x00;
	
	// Turbo I2C
	I2CTL |= bm400KHZ;

	// Port lines zero'd before setting direction
	IOD = 0x00;

	// Three JTAG bits drive pins on the FPGA
	OED = bmTDI | bmTMS | bmTCK;

	// If the FPGA is active, find the end of the FX2 code in the EEPROM and then send some of the
	// following data to the FPGA
	//if ( thisByte ) {
	promStartRead(0x0000);
	if ( promPeekByte() == 0xC2 ) {
		promNextByte();    // VID(L)
		promNextByte();    // VID(H)
		promNextByte();    // PID(L)
		promNextByte();    // PID(H)
		promNextByte();    // DID(L)
		promNextByte();    // DID(H)
		promNextByte();    // Config byte
		
		promNextByte();    // Length(H)
		thisByte = promPeekByte();
		while ( !(thisByte & 0x80) ) {
			blockSize = thisByte;
			blockSize <<= 8;
			
			promNextByte();  // Length(L)
			blockSize |= promPeekByte();
			
			blockSize += 2;  // Space taken by address
			while ( blockSize-- ) {
				promNextByte();
			}
			
			promNextByte();  // Length(H)
			thisByte = promPeekByte();
		}
		promNextByte();    // Length(L)
		promNextByte();    // Address(H)
		promNextByte();    // Address(L)
		promNextByte();    // Last byte
		promNextByte();    // First byte after the end of the firmware
	}
	jtagCsvfInit();
	m_diagnosticCode = jtagCsvfPlay();
	fifoSendPromData(23+2071);  // 23 command bytes, 2071 data bytes
	promStopRead();
}

// Called repeatedly while the device is idle
//
void mainLoop(void) {
	// If there is a JTAG shift operation pending, execute it now.
	if ( jtagIsShiftPending() ) {
		jtagShiftExecute();
	} else if ( syncIsEnabled() ) {
		syncExecute();
	}
}

// Called when a vendor command is received
//
uint8 handleVendorCommand(uint8 cmd) {
	switch(cmd) {
	// Put the device in sync mode, so its bulk endpoints can be sync'd with the host software
	//
	case CMD_SYNC_MODE:
		if ( SETUP_TYPE == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
			// Enable sync mode if wValue is nonzero
			syncSetEnabled(SETUP_VALUE() == 0 ? false : true);
		} else {
			// Unrecognised operation
			return false;
		}
		break;

	// Fetch the diagnostic code, so host can query what went wrong.
	//
	case CMD_GET_DIAG_CODE:
		if ( SETUP_TYPE == (REQDIR_DEVICETOHOST | REQTYPE_VENDOR) ) {
			uint16 *xdata outArray = (uint16 *)EP0BUF;

			// It's an IN operation - prepare the response and send it
			while ( EP0CS & bmEPBUSY );
			*EP0BUF = m_diagnosticCode;
			EP0BCH = 0;
			SYNCDELAY;
			EP0BCL = 1;
		} else {
			// This command does not support OUT operations
			return false;
		}
		break;

	// Clock data into and out of the JTAG chain. Reads from EP2OUT and writes to EP4IN.
	//
	case CMD_JTAG_CLOCK_DATA:
		if ( SETUP_TYPE == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
			EP0BCL = 0x00;                                     // Allow host transfer in
			while ( EP0CS & bmEPBUSY );                        // Wait for data
			jtagShiftBegin(*((uint32 *)EP0BUF), SETUPDAT[2]);  // Init numBits & flagByte
			// Now that numBits & flagByte are set, this operation will continue in mainLoop()...
		} else {
			// Unrecognised operation
			return false;
		}
		break;
		
	// Clock an (up to) 32-bit pattern LSB-first into TMS to change JTAG TAP states
	//
	case CMD_JTAG_CLOCK_FSM:
		if ( SETUP_TYPE == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
			EP0BCL = 0x00;                                   // Allow host transfer in
			while ( EP0CS & bmEPBUSY );                      // Wait for data
			jtagClockFSM(*((uint32 *)EP0BUF), SETUPDAT[2]);  // Bit pattern, transitionCount
		} else {
			// This command does not support OUT operations
			return false;
		}
		break;
		
	// Execute a number of JTAG clocks.
	//
	case CMD_JTAG_CLOCK:
		if ( SETUP_TYPE == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
			jtagClocks(*((uint32 *)(SETUPDAT+2)));
		} else {
			// This command does not support OUT operations
			return false;
		}
		break;
		
	// Command to talk to the EEPROM
	//
	case CMD_READ_WRITE_EEPROM:
		if ( SETUP_TYPE == (REQDIR_DEVICETOHOST | REQTYPE_VENDOR) ) {
			// It's an IN operation - read from prom and send to host
			xdata uint16 address = SETUP_VALUE();
			xdata uint16 length = SETUP_LENGTH();
			xdata uint16 chunkSize;
			xdata uint8 i;
			while ( length ) {
				while ( EP0CS & bmEPBUSY );
				chunkSize = length < EP0BUF_SIZE ? length : EP0BUF_SIZE;
				for ( i = 0; i < chunkSize; i++ ) {
					EP0BUF[i] = 0x23;
				}
				promRead(address, chunkSize, EP0BUF);
				EP0BCH = 0;
				SYNCDELAY;
				EP0BCL = chunkSize;
				address += chunkSize;
				length -= chunkSize;
			}
		} else if ( SETUP_TYPE == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
			// It's an OUT operation - read from host and send to prom
			xdata uint16 address = SETUP_VALUE();
			xdata uint16 length = SETUP_LENGTH();
			xdata uint16 chunkSize;
			while ( length ) {
				EP0BCL = 0x00; // allow pc transfer in
				while ( EP0CS & bmEPBUSY ); // wait for data
				chunkSize = EP0BCL;
				promWrite(address, chunkSize, EP0BUF);
				address += chunkSize;
				length -= chunkSize;
			}
		}
		else {
			return false;
		}
		break;
	default:
		return false;  // unrecognised command
	}
	return true;
}

// Compose a packet to send on the EP6 FIFO, and commit it.
//
void fifoSendPromData(uint32 bytesToSend) {
	
	xdata uint16 i, chunkSize;
	xdata uint8 thisByte;

	while ( bytesToSend ) {
		chunkSize = (bytesToSend >= 512) ? 512 : (uint16)bytesToSend;

		while ( !(EP2468STAT & bmEP6EMPTY) );  // Wait while FIFO remains "not empty" (i.e while busy)

		SYNCDELAY; EP6FIFOCFG = 0x00;          // Disable AUTOOUT
		SYNCDELAY; FIFORESET = bmNAKALL;       // NAK all OUT packets from host
		SYNCDELAY; FIFORESET = 6;              // Advance EP6 buffers to CPU domain	

		for ( i = 0; i < chunkSize; i++ ) {
			EP6FIFOBUF[i] = promPeekByte();      // Compose packet to send to EP6 FIFO
			promNextByte();
		}
		SYNCDELAY; EP6BCH = MSB(chunkSize);    // Commit newly-sourced packet to FIFO
		SYNCDELAY; EP6BCL = LSB(chunkSize);
	
		SYNCDELAY; OUTPKTEND = bmSKIP | 6;     // Skip uncommitted second packet
		bytesToSend -= chunkSize;

		SYNCDELAY; FIFORESET = 0;              // Release "NAK all"
		SYNCDELAY; EP6FIFOCFG = bmAUTOOUT;     // Enable AUTOOUT again
	}
}
