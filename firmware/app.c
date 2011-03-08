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

BOOL promRead(WORD addr, BYTE length, BYTE xdata *buf);
BOOL promWrite(WORD addr, BYTE length, const BYTE xdata *buf);

// The minimum number of bytes necessary to store this many bits
#define bitsToBytes(x) ((x>>3) + (x&7 ? 1 : 0))

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

uint16 checksum = 0;

// Called repeatedly while the device is idle
//
void main_loop(void) {
    if ( !(EP2468STAT & bmEP2EMPTY) ) {
        // EP2 is not empty (host sent us a packet)
        if  ( !(EP2468STAT & bmEP4FULL) ) {
            // EP4 is not full (we can send host a packet)
            uint16 numBytes = MAKEWORD(EP2BCH, EP2BCL);
            uint16 i;
            for ( i = 0; i < numBytes; i++ ) {
                EP4FIFOBUF[i] = EP2FIFOBUF[i];
                checksum += EP2FIFOBUF[i];
            }
            SYNCDELAY; EP4BCH = MSB(numBytes);  // Initiate send of the copied data
            SYNCDELAY; EP4BCL = LSB(numBytes);
            SYNCDELAY; OUTPKTEND = bmSKIP | 2;  // Acknowledge receipt of this packet
        }
    }
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

// Called when a vendor command is received
//
BOOL handle_vendorcommand(BYTE cmd) {
	WORD address, length;
	BYTE i, j, chunkSize;
	switch(cmd) {
	case 0x80:
		// Simple example command which does the four arithmetic operations on the data from
		// the host, and sends the results back to the host
		//
		if ( SETUP_TYPE == 0xc0 ) {
			const unsigned short *inArray = (unsigned short *)(SETUPDAT+2);
			unsigned short *outArray = (unsigned short *)EP0BUF;

			fifo_send();

			// It's an IN operation
			//
			while ( EP0CS & bmEPBUSY );
			outArray[0] = inArray[0] + inArray[1];
			outArray[1] = inArray[0] - inArray[1];
			outArray[2] = inArray[0] * inArray[1];
			outArray[3] = inArray[0] / inArray[1];
			EP0BCH = 0;
			SYNCDELAY;
			EP0BCL = 8;
		} else {
			// This command does not support OUT operations
			//
			return FALSE;
		}
		break;
		
		case 0x90:
			if ( SETUP_TYPE == 0xc0 ) {
				const unsigned short *inArray = (unsigned short *)(SETUPDAT+2);
				unsigned long *outArray = (unsigned long *)EP0BUF, *ptr = outArray;
				unsigned long thisID;
				
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
				return FALSE;
			}
			break;

	case 0xa2:
		// Command to talk to the EEPROM
		//
		I2CTL |= bm400KHZ;
		address = SETUPDAT[2];
		address |= SETUPDAT[3] << 8;
		length = SETUPDAT[6];
		length |= SETUPDAT[7] << 8;
		if ( SETUP_TYPE == 0xc0 ) {
			// It's an IN operation - read from prom and send to host
			//
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
		} else if ( SETUP_TYPE == 0x40 ) {
			// It's an OUT operation - read from host and send to prom
			//
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
			return FALSE;
		}
		break;
	default:
		return FALSE;  // unrecognised command
	}
	return TRUE;
}

BOOL promWaitForDone() {
	BYTE i;
	while ( !((i = I2CS) & 1) );  // Poll the done bit
	if ( i & bmBERR ) {
		return 1;
	} else {
		return 0;
	}
}	

BOOL promWaitForAck()
{
	BYTE i;
	while ( !((i = I2CS) & 1) );  // Poll the done bit
	if ( i & bmBERR ) {
		return 1;
	} else if ( !(i & bmACK) ) {
		return 1;
	} else {
		return 0;
	}
}

BOOL promRead(WORD addr, BYTE length, BYTE xdata *buf) {
	BYTE i;
	
	// Wait for I2C idle
	//
	while ( I2CS & bmSTOP );

	// Send the WRITE command
	//
	I2CS = bmSTART;
	I2DAT = 0xA2;  // Write I2C address byte (WRITE)
	if ( promWaitForAck() ) {
		return 1;
	}
	
	// Send the address, MSB first
	//
	I2DAT = MSB(addr);  // Write MSB of address
	if ( promWaitForAck() ) {
		return 1;
	}
	I2DAT = LSB(addr);  // Write LSB of address
	if ( promWaitForAck() ) {
		return 1;
	}

	// Send the READ command
	//
	I2CS = bmSTART;
	I2DAT = 0xA3;  // Write I2C address byte (READ)
	if ( promWaitForDone() ) {
		return 1;
	}

	// Read dummy byte
	//
	i = I2DAT;
	if ( promWaitForDone() ) {
		return 1;
	}

	// Now get the actual data
	//
	for ( i = 0; i < (length-1); i++ ) {
		*(buf+i) = I2DAT;
		if ( promWaitForDone() ) {
			return 1;
		}
	}

	// Terminate the read operation and get last byte
	//
	I2CS = bmLASTRD;
	if ( promWaitForDone() ) {
		return 1;
	}
	*(buf+i) = I2DAT;
	if ( promWaitForDone() ) {
		return 1;
	}
	I2CS = bmSTOP;
	i = I2DAT;

	return 0;
}

BOOL promWrite(WORD addr, BYTE length, const BYTE xdata *buf) {
	BYTE i;

	// Wait for I2C idle
	//
	while ( I2CS & bmSTOP );

	// Send the WRITE command
	//
	I2CS = bmSTART;
	I2DAT = 0xA2;  // Write I2C address byte (WRITE)
	if ( promWaitForAck() ) {
		return 1;
	}

	// Send the address, MSB first
	//
	I2DAT = MSB(addr);  // Write MSB of address
	if ( promWaitForAck() ) {
		return 1;
	}
	I2DAT = LSB(addr);  // Write LSB of address
	if ( promWaitForAck() ) {
		return 1;
	}

	// Write the data
	//
	for ( i = 0; i < length; i++ ) {
        I2DAT = *buf++;
		if ( promWaitForDone() ) {
			return 1;
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
			return 1;
		}
		I2CS |= bmSTOP;
		while ( I2CS & bmSTOP );
	} while ( !(I2CS & bmACK) );
	
	return 0;
}

BYTE currentConfiguration;  // Current configuration
BYTE alternateSetting = 0;  // Alternate settings

// Called when a Set Configuration command is received
//
BOOL handle_set_configuration(BYTE cfg) {
	currentConfiguration = cfg;
	return(TRUE);  // Handled by user code
}

// Called when a Get Configuration command is received
//
BYTE handle_get_configuration()
{
	return currentConfiguration;
}

// Called when a Get Interface command is received
//
BOOL handle_get_interface(BYTE ifc, BYTE *alt) {
	*alt = alternateSetting;
	return TRUE;
}

// Called when a Set Interface command is received
//
BOOL handle_set_interface(BYTE ifc, BYTE alt) {
	alternateSetting = alt;
	return TRUE;
}
