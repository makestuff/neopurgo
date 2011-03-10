/* 
 * Copyright (C) 2009 Chris McClelland
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
bool promRead(uint16 addr, uint8 length, uint8 xdata *buf);
bool promWrite(uint16 addr, uint8 length, const uint8 xdata *buf);
void shiftOut(uint8 c);
uint8 shiftInOut(uint8 c);
void fifo_send(void);

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

// The minimum number of bytes necessary to store this many bits
#define bitsToBytes(x) ((x>>3) + (x&7 ? 1 : 0))

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
void main_init(void) {

	// Global settings
	SYNCDELAY; REVCTL = (bmDYN_OUT | bmENH_PKT);
	SYNCDELAY; CPUCS = bmCLKSPD1;  // 48MHz

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
}

// Called repeatedly while the device is idle
//
void main_loop(void) {
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
bool handle_vendorcommand(uint8 cmd) {
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

				// Go to Test-Logic-Reset
				//IOD = bmTMS;
				//IOD = bmTMS|bmTCK; IOD = bmTMS;
				//IOD = bmTMS|bmTCK; IOD = bmTMS;
				//IOD = bmTMS|bmTCK; IOD = bmTMS;
				//IOD = bmTMS|bmTCK; IOD = bmTMS;
				//IOD = bmTMS|bmTCK; IOD = bmTMS;
				
				//IOD = 0x00;  IOD = bmTCK;       IOD = 0x00;      // Now in Run-Test/Idle
				//IOD = bmTMS; IOD = bmTMS|bmTCK; IOD = bmTMS;     // Now in Select-DR Scan
				//IOD = 0x00;  IOD = bmTCK;       IOD = 0x00;      // Now in Capture-DR
				//IOD = 0x00;  IOD = bmTCK;       IOD = 0x00;      // Now in Shift-DR

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

			fifo_send();

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

// Wait for the I2C interface to complete the current send or receive operation. Return true if
// there was a bus error, else return false if the operation completed successfully.
//
bool promWaitForDone() {
	uint8 i;
	while ( !((i = I2CS) & bmDONE) );  // Poll the done bit
	if ( i & bmBERR ) {
		return true;
	} else {
		return false;
	}
}	

// Wait for the I2C interface to complete the current send operation. Return true if there was a
// bus error or the slave failed to acknowledge receipt of the byte, else return false if the
// operation completed successfully.
//
bool promWaitForAck() {
	uint8 i;
	while ( !((i = I2CS) & bmDONE) );  // Poll the done bit
	if ( i & bmBERR ) {
		return true;
	} else if ( !(i & bmACK) ) {
		return true;
	} else {
		return false;
	}
}

// Read "length" bytes from address "addr" in the attached EEPROM, and write them to RAM at "buf".
//
bool promRead(uint16 addr, uint8 length, uint8 xdata *buf) {
	uint8 i;
	
	// Wait for I2C idle
	//
	while ( I2CS & bmSTOP );

	// Send the WRITE command
	//
	I2CS = bmSTART;
	I2DAT = 0xA2;  // Write I2C address byte (WRITE)
	if ( promWaitForAck() ) {
		return true;
	}
	
	// Send the address, MSB first
	//
	I2DAT = MSB(addr);  // Write MSB of address
	if ( promWaitForAck() ) {
		return true;
	}
	I2DAT = LSB(addr);  // Write LSB of address
	if ( promWaitForAck() ) {
		return true;
	}

	// Send the READ command
	//
	I2CS = bmSTART;
	I2DAT = 0xA3;  // Write I2C address byte (READ)
	if ( promWaitForDone() ) {
		return true;
	}

	// Read dummy byte
	//
	i = I2DAT;
	if ( promWaitForDone() ) {
		return true;
	}

	// Now get the actual data
	//
	for ( i = 0; i < (length-1); i++ ) {
		buf[i] = I2DAT;
		if ( promWaitForDone() ) {
			return true;
		}
	}

	// Terminate the read operation and get last byte
	//
	I2CS = bmLASTRD;
	if ( promWaitForDone() ) {
		return true;
	}
	buf[i] = I2DAT;
	if ( promWaitForDone() ) {
		return true;
	}
	I2CS = bmSTOP;
	i = I2DAT;

	return false;
}

// Read "length" bytes from RAM at "buf", and write them to the attached EEPROM at address "addr".
//
bool promWrite(uint16 addr, uint8 length, const uint8 xdata *buf) {
	uint8 i;

	// Wait for I2C idle
	//
	while ( I2CS & bmSTOP );

	// Send the WRITE command
	//
	I2CS = bmSTART;
	I2DAT = 0xA2;  // Write I2C address byte (WRITE)
	if ( promWaitForAck() ) {
		return true;
	}

	// Send the address, MSB first
	//
	I2DAT = MSB(addr);  // Write MSB of address
	if ( promWaitForAck() ) {
		return true;
	}
	I2DAT = LSB(addr);  // Write LSB of address
	if ( promWaitForAck() ) {
		return true;
	}

	// Write the data
	//
	for ( i = 0; i < length; i++ ) {
		I2DAT = *buf++;
		if ( promWaitForDone() ) {
			return true;
		}
	}
	I2CS |= bmSTOP;

	// Wait for I2C idle
	//
	while ( I2CS & bmSTOP );

	do {
		I2CS = bmSTART;
		I2DAT = 0xA2;  // Write I2C address byte (WRITE)
		if ( promWaitForDone() ) {
			return true;
		}
		I2CS |= bmSTOP;
		while ( I2CS & bmSTOP );
	} while ( !(I2CS & bmACK) );
	
	return false;
}

// JTAG-clock the supplied byte into TDI, LSB first.
//
// Lifted from:
//   http://ixo-jtag.svn.sourceforge.net/viewvc/ixo-jtag/usb_jtag/trunk/device/c51/hw_nexys.c
//
void shiftOut(uint8 c) {
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
uint8 shiftInOut(uint8 c) {
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

// Compose a packet to send on the EP6 FIFO, and commit it.
//
void fifo_send(void) {
	SYNCDELAY; EP6FIFOCFG = 0x00;      // Disable AUTOOUT
	SYNCDELAY; FIFORESET = bmNAKALL;   // NAK all OUT packets from host
	SYNCDELAY; FIFORESET = 6;          // Advance EP6 buffers to CPU domain
	
	EP6FIFOBUF[0] = 0x01;              // Compose packet to send to EP6 FIFO
	
	SYNCDELAY; EP6BCH = 0;             // Commit newly-sourced packet to FIFO
	SYNCDELAY; EP6BCL = 1;
	
	SYNCDELAY; OUTPKTEND = bmSKIP | 6; // Skip uncommitted second packet
	
	SYNCDELAY; FIFORESET = 0;          // Release "NAK all"
	SYNCDELAY; EP6FIFOCFG = bmAUTOOUT; // Enable AUTOOUT again
}

uint8 currentConfiguration;  // Current configuration
uint8 alternateSetting = 0;  // Alternate settings

// Called when a Set Configuration command is received
//
bool handle_set_configuration(uint8 cfg) {
	currentConfiguration = cfg;
	return(true);  // Handled by user code
}

// Called when a Get Configuration command is received
//
uint8 handle_get_configuration() {
	return currentConfiguration;
}

// Called when a Get Interface command is received
//
bool handle_get_interface(uint8 ifc, uint8 *alt) {
	*alt = alternateSetting;
	return true;
}

// Called when a Set Interface command is received
//
bool handle_set_interface(uint8 ifc, uint8 alt) {
	alternateSetting = alt;
	return true;
}
