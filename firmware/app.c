/* 
 * Copyright (C) 2009-2011 Chris McClelland
 *
 * Copyright (C) 2009 Ubixum, Inc.
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
#include <i2c.h>
#include <setupdat.h>
#include <types.h>
#include "prom.h"
#include "jtag.h"

#define SYNCDELAY SYNCDELAY4;
#define EP0BUF_SIZE 0x40

// IFCONFIG bits
#define bmPORTS 0
#define bmGPIF  (bmIFCFG1)
#define bmFIFOS (bmIFCFG1 | bmIFCFG0)

// EPxCFG bits
#define bmBULK bmBIT5
#define bmBUF2X bmBIT1

// OUTPKTEND bits
#define bmSKIP bmBIT7

// REVCTL bits
#define bmDYN_OUT (1<<1)
#define bmENH_PKT (1<<0)

// USB command macros, copied from Dean Camera's LUFA package
#define REQDIR_DEVICETOHOST (1 << 7)
#define REQDIR_HOSTTODEVICE (0 << 7)
#define REQTYPE_CLASS       (1 << 5)
#define REQTYPE_STANDARD    (0 << 5)
#define REQTYPE_VENDOR      (2 << 5)

// Function declarations
void fifoSendPromData(uint32 bytesToSend);

// Defines to allow use of camelCase.
#define mainInit(x) main_init(x)
#define mainLoop(x) main_loop(x)
#define handleVendorCommand handle_vendorcommand

// The USB vendor commands
enum {
	CMD_CLOCK_DATA = 0x80,        // NeroJTAG commands
	CMD_CLOCK_STATE_MACHINE,
	CMD_CLOCK,

	CMD_SCAN_CHAIN_TEST = 0x90,   // Temporary/testing commands
	CMD_CALCULATOR_TEST,

	CMD_READ_WRITE_EEPROM = 0xA2  // Read/Write the EEPROM
};

// Globals
static uint32 numBits = 0UL;
static uint8 flagByte = 0x00;

// Called once at startup
//
void mainInit(void) {

	uint8 thisByte = 0xFF;
	uint16 blockSize;

	// Global settings
	SYNCDELAY; REVCTL = (bmDYN_OUT | bmENH_PKT);
	SYNCDELAY; CPUCS = bmCLKSPD1;  // 48MHz

	// Check if the FPGA is running (it drives "10" on fifoAddr)
	SYNCDELAY; IFCONFIG = 0x00;
	if ( (IOA & (bmBIT4|bmBIT5)) == (bmBIT4|bmBIT5) ) {
		thisByte = 0x00;  // The FPGA is not running
	}

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
	
	// Port lines zero'd before setting direction
	IOD = 0x00;

	// Three JTAG bits drive pins on the FPGA
	OED = bmTDI | bmTMS | bmTCK;

	// If the FPGA is active, find the end of the FX2 code in the EEPROM and then send some of the
	// following data to the FPGA
	if ( thisByte ) {
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
	
			// Send the next 2071 bytes to the FPGA
			promNextByte();    // First byte of FIFO data
			fifoSendPromData(2071);
		}
		promStopRead();
	}
}

// Called repeatedly while the device is idle
//
void mainLoop(void) {
	// Are there any JTAG send/receive operations to execute?
	if ( numBits ) {
		if ( (flagByte & bmSENDMASK) == bmSENDDATA ) {
			if ( flagByte & bmNEEDRESPONSE ) {
				// The host is giving us data, and is expecting a response (xdr)
				uint16 bitsRead, bitsRemaining, bytesRead, bytesRemaining;
				uint8 *inPtr, *outPtr;
				while ( numBits ) {
					while ( EP2468STAT & bmEP2EMPTY );  // Wait for some EP2OUT data
					while ( EP2468STAT & bmEP4FULL );   // Wait for space for EP4IN data
					bitsRead = (numBits >= (ENDPOINT_SIZE<<3)) ? ENDPOINT_SIZE<<3 : numBits;
					bytesRead = MAKEWORD(EP2BCH, EP2BCL);
					if ( bytesRead != bitsToBytes(bitsRead) ) {
						// Protocol violation - give up
						numBits = 0UL;
						break;
					}

					inPtr = EP2FIFOBUF;
					outPtr = EP4FIFOBUF;
					if ( bitsRead == numBits ) {
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
							if ( (flagByte & bmISLAST) && !leftOver ) {
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

					numBits -= bitsRead;
				}
			} else {
				// The host is giving us data, but does not need a response (xdn)
				uint16 bitsRead, bitsRemaining, bytesRead, bytesRemaining;
				uint16 i;
				while ( numBits ) {
					while ( EP2468STAT & bmEP2EMPTY );  // Wait for some EP2OUT data
					bitsRead = (numBits >= (ENDPOINT_SIZE<<3)) ? ENDPOINT_SIZE<<3 : numBits;
					bytesRead = MAKEWORD(EP2BCH, EP2BCL);
					if ( bytesRead != bitsToBytes(bitsRead) ) {
						// Protocol violation - give up
						numBits = 0UL;
						break;
					}

					inPtr = EP2FIFOBUF;
					if ( bitsRead == numBits ) {
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
							if ( (flagByte & bmISLAST) && !leftOver ) {
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
					numBits -= bitsRead;
				}
			}
		} else {
			if ( flagByte & bmNEEDRESPONSE ) {
				// The host is not giving us data, but is expecting a response (x0r)
				uint16 bitsRead, bitsRemaining, bytesRead, bytesRemaining;
				uint8 tdiByte;
				if ( (flagByte & bmSENDMASK) == bmSENDZEROS ) {
					tdiByte = 0x00;
				} else {
					tdiByte = 0xFF;
				}
				while ( numBits ) {
					while ( EP2468STAT & bmEP4FULL );   // Wait for space for EP4IN data
					bitsRead = (numBits >= (ENDPOINT_SIZE<<3)) ? ENDPOINT_SIZE<<3 : numBits;
					bytesRead = bitsToBytes(bitsRead);

					inPtr = EP2FIFOBUF;
					outPtr = EP4FIFOBUF;
					if ( bitsRead == numBits ) {
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
							if ( (flagByte & bmISLAST) && !leftOver ) {
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
					numBits -= bitsRead;
				}
			} else {
				// The host is not giving us data, and does not need a response (x0n)
				uint32 bitsRemaining, bytesRemaining;
				uint8 tdiByte, leftOver;
				if ( (flagByte & bmSENDMASK) == bmSENDZEROS ) {
					tdiByte = 0x00;
				} else {
					tdiByte = 0xFF;
				}
				bitsRemaining = (numBits-1) & 0xFFFFFFF8;    // Now an integer number of bytes
				leftOver = (uint8)(numBits - bitsRemaining); // How many bits in last byte (1-8)
				bytesRemaining = (bitsRemaining>>3);
				while ( bytesRemaining-- ) {
					shiftOut(tdiByte);
				}
				TDI = tdiByte & 1;
				while ( leftOver ) {
					leftOver--;
					if ( (flagByte & bmISLAST) && !leftOver ) {
						TMS = 1; // Exit Shift-DR state on next clock
					}
					TCK = 1;
					TCK = 0;
				}
			}
		}
	}
}

// Called when a vendor command is received
//
bool handleVendorCommand(uint8 cmd) {
	uint16 address, length;
	uint8 i, j;
	switch(cmd) {
		// Clock data into and out of the JTAG chain. Reads from EP2OUT and writes to EP4IN.
		//
		case CMD_CLOCK_DATA:
			if ( SETUP_TYPE == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
				flagByte = SETUPDAT[2];          // Remember flag byte
				EP0BCL = 0x00;                   // Allow host transfer in
				while ( EP0CS & bmEPBUSY );      // Wait for data
				numBits = *((uint32 *)EP0BUF);   // Remember length
				// This operation continues in main_loop()...
			} else {
				// Unrecognised operation
				return false;
			}
			break;

		// Clock an (up to) 32-bit pattern LSB-first into TMS to change JTAG TAP states
		//
		case CMD_CLOCK_STATE_MACHINE:
			if ( SETUP_TYPE == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
				uint8 transitionCount = SETUPDAT[2];
				uint32 bitPattern;
				EP0BCL = 0x00;                     // Allow host transfer in
				while ( EP0CS & bmEPBUSY );        // Wait for data
				bitPattern = *((uint32 *)EP0BUF);  // Remember length
				while ( transitionCount-- ) {
					TMS = bitPattern & 1;
					TCK = 1;
					TCK = 0;
					bitPattern >>= 1;
				}
			} else {
				// This command does not support OUT operations
				//
				return false;
			}
			break;

		// Execute a number of JTAG clocks.
		//
		case CMD_CLOCK:
			if ( SETUP_TYPE == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
				uint32 numCycles = MAKEDWORD(SETUP_VALUE(), SETUP_INDEX());
				while ( numCycles-- ) {
					TCK = 1;
					TCK = 0;
				}
			} else {
				// This command does not support OUT operations
				//
				return false;
			}
			break;

		// Simple JTAG scan-chain operation
		//
		case CMD_SCAN_CHAIN_TEST:
			if ( SETUP_TYPE == (REQDIR_DEVICETOHOST | REQTYPE_VENDOR) ) {
				const uint16 *inArray = (uint16 *)(SETUPDAT+2);
				uint32 *outArray = (uint32 *)EP0BUF, *ptr = outArray;
				uint32 thisID;
				
				// Go to Test-Logic-Reset
				IOD = bmTMS;
				IOD = bmTMS|bmTCK; IOD = bmTMS;
				IOD = bmTMS|bmTCK; IOD = bmTMS;
				IOD = bmTMS|bmTCK; IOD = bmTMS;
				IOD = bmTMS|bmTCK; IOD = bmTMS;
				IOD = bmTMS|bmTCK; IOD = bmTMS;
				
				IOD = 0x00;  IOD = bmTCK;       IOD = 0x00;      // Now in Run-Test/Idle
				IOD = bmTMS; IOD = bmTMS|bmTCK; IOD = bmTMS;     // Now in Select-DR Scan
				IOD = 0x00;  IOD = bmTCK;       IOD = 0x00;      // Now in Capture-DR
				IOD = 0x00;  IOD = bmTCK;       IOD = 0x00;      // Now in Shift-DR
				
				for ( j = 0; j < 4; j++ ) {
					thisID = 0UL;
					for ( i = 0; i < 31; i++ ) {
						if ( TDO ) {
							thisID |= 0x80000000;
						}
						TCK = 1; TCK = 0;
						thisID >>= 1;
					}
					if ( TDO ) {
						thisID |= 0x80000000;
					}
					TCK = 1; TCK = 0;
					*ptr++ = thisID;
				}
				SYNCDELAY; EP0BCH = 0;
				SYNCDELAY; EP0BCL = 16;
			} else {
				// This command does not support OUT operations
				//
				return false;
			}
			break;

	// Simple example command which does the four arithmetic operations on the data from
	// the host, and sends the results back to the host
	//
	case CMD_CALCULATOR_TEST:
		if ( SETUP_TYPE == (REQDIR_DEVICETOHOST | REQTYPE_VENDOR) ) {
			uint16 num1 = SETUP_VALUE();
			uint16 num2 = SETUP_INDEX();
			uint16 *outArray = (uint16 *)EP0BUF;

			//fifoSend(0x01);

			// It's an IN operation - prepare the response and send it
			while ( EP0CS & bmEPBUSY );
			outArray[0] = num1 + num2;
			outArray[1] = num1 - num2;
			outArray[2] = num1 * num2;
			outArray[3] = num1 / num2;
			EP0BCH = 0;
			SYNCDELAY;
			EP0BCL = 8;
		} else {
			// This command does not support OUT operations
			//
			return false;
		}
		break;

	// Command to talk to the EEPROM
	//
	case CMD_READ_WRITE_EEPROM:
		I2CTL |= bm400KHZ;
		address = SETUP_VALUE();
		length = SETUP_LENGTH();
		if ( SETUP_TYPE == (REQDIR_DEVICETOHOST | REQTYPE_VENDOR) ) {
			// It's an IN operation - read from prom and send to host
			//
			uint16 chunkSize;
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
			//
			uint16 chunkSize;
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
	
	uint16 i, chunkSize;
	uint8 thisByte;

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
