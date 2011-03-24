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
#include <stdio.h>
#include <libnero.h>

// "Member" variables
static FILE *m_csvfFile = NULL;
static uint16 m_count;
static bool m_isReadingChunk;

uint8 getRawByte(void) {
	return (uint8)fgetc(m_csvfFile);
}

// Misc defines
#define fail(x) returnCode = x; goto cleanup
#define bitsToBytes(x) ((x>>3) + (x&7 ? 1 : 0))

// XSVF commands (from xapp503 appendix B)
typedef enum {
	XCOMPLETE    = 0x00,
	XTDOMASK     = 0x01,
	XSIR         = 0x02,
	XSDR         = 0x03,
	XRUNTEST     = 0x04,
	XREPEAT      = 0x07,
	XSDRSIZE     = 0x08,
	XSDRTDO      = 0x09,
	XSETSDRMASKS = 0x0A,
	XSDRINC      = 0x0B,
	XSDRB        = 0x0C,
	XSDRC        = 0x0D,
	XSDRE        = 0x0E,
	XSDRTDOB     = 0x0F,
	XSDRTDOC     = 0x10,
	XSDRTDOE     = 0x11,
	XSTATE       = 0x12,
	XENDIR       = 0x13,
	XENDDR       = 0x14,
	XSIR2        = 0x15,
	XCOMMENT     = 0x16,
	XWAIT        = 0x17,
} Command;

// TAP states (from xapp503 appendix B)
typedef enum {
	TAPSTATE_TEST_LOGIC_RESET = 0x00,
	TAPSTATE_RUN_TEST_IDLE    = 0x01,
	TAPSTATE_SELECT_DR        = 0x02,
	TAPSTATE_CAPTURE_DR       = 0x03,
	TAPSTATE_SHIFT_DR         = 0x04,
	TAPSTATE_EXIT1_DR         = 0x05,
	TAPSTATE_PAUSE_DR         = 0x06,
	TAPSTATE_EXIT2_DR         = 0x07,
	TAPSTATE_UPDATE_DR        = 0x08,
	TAPSTATE_SELECT_IR        = 0x09,
	TAPSTATE_CAPTURE_IR       = 0x0A,
	TAPSTATE_SHIFT_IR         = 0x0B,
	TAPSTATE_EXIT1_IR         = 0x0C,
	TAPSTATE_PAUSE_IR         = 0x0D,
	TAPSTATE_EXIT2_IR         = 0x0E,
	TAPSTATE_UPDATE_IR        = 0x0F
} TAPState;

// Read the length (of the chunk or the zero run). A short block (<256 bytes) length is encoded in a
// single byte. If that single byte is zero, we know it's a long block (256-65535 bytes), so read in
// the next two bytes as a big-endian uint16.
//
static uint16 readLength(void) {
	uint16 len = getRawByte();
	if ( !len ) {
		len = getRawByte();
		len <<= 8;
		len |= getRawByte();
	}
	return len;
}

// Initialise the CSVF reader
//
void initReader(void) {
	getRawByte(); // Skip header byte
	m_count = readLength();
	m_isReadingChunk = true;
}

// Get the next byte from the uncompressed stream. Uses m_count & m_isReadingChunk to keep state.
//
static uint8 getNextByte(void) {
	if ( m_isReadingChunk ) {
		// We're in the middle of reading a chunk.
		if ( m_count ) {
			// There are still some bytes to copy verbatim into the uncompressed stream.
			m_count--;
			return getRawByte();
		} else {
			// We're at the end of this chunk; there will now be some zeros to insert into the
			// uncompressed stream.
			m_count = readLength();
			m_isReadingChunk = false;
			return getNextByte();
		}
	} else {
		// We're in the middle of a run of zeros.
		if ( m_count ) {
			// There are still some zero bytes to write to the uncompressed stream.
			m_count--;
			return 0x00;
		} else {
			// We're at the end of this run of zeros; there will now be a chunk of data to be copied
			// verbatim over to the uncompressed stream.
			m_count = readLength();
			m_isReadingChunk = true;
			return getNextByte();
		}
	}
}

// Dump some hex bytes to stdout, for debugging.
//
static void dumpSimple(const unsigned char *input, unsigned int length) {
	while ( length ) {
		printf(" %02X", *input++);
		--length;
	}
}

// Get big-endian uint16 from the stream
//
static uint16 getWord(void) {
	uint16 value;
	value = getNextByte();
	value <<= 8;
	value |= getNextByte();
	return value;
}

// Play the uncompressed CSVF stream into the JTAG port.
//
int csvfPlay(void) {
	int returnCode = 0;
	uint8 thisByte;
	uint16 numBytes;
	uint8 *ptr;
	uint8 i;
	uint16 xsdrSize = 0;  // These should be 32-bits each, but that seems a bit wasteful
	uint16 xruntest = 0;
	uint8 tdoMask[128];
	uint8 tdiData[128];
	uint8 tdoData[128];
	uint8 tdoExpected[128];
	thisByte = getNextByte();
	while ( thisByte != XCOMPLETE ) {
		switch ( thisByte ) {
		case XTDOMASK:
			numBytes = bitsToBytes(xsdrSize);
			ptr = tdoMask;
			while ( numBytes-- ) {
				*ptr++ = getNextByte();
			}
			break;

		case XRUNTEST:
			getNextByte();  // Ignore the MSW (realistically will it ever be nonzero?)
			getNextByte();
			xruntest = getWord();
			break;

		case XSIR:
			neroClockFSM(0x00000003, 4);
			thisByte = getNextByte();
			numBytes = bitsToBytes(thisByte);
			ptr = tdiData;
			while ( numBytes-- ) {
				*ptr++ = getNextByte();
			}
			neroShift(thisByte, tdiData, NULL, true);
			neroClockFSM(0x00000001, 2);
			if ( xruntest ) {
				neroClocks(xruntest);
			}
			break;

		case XSDRSIZE:
			xsdrSize = getWord();
			break;

		case XSDRTDO:
			neroClockFSM(0x00000001, 3);
			numBytes = bitsToBytes(xsdrSize);
			ptr = tdoExpected;
			while ( numBytes-- ) {
				*ptr++ = getNextByte();
			}
			numBytes = bitsToBytes(xsdrSize);
			ptr = tdiData;
			while ( numBytes-- ) {
				*ptr++ = getNextByte();
			}
			neroShift(xsdrSize, tdiData, tdoData, true);
			numBytes = bitsToBytes(xsdrSize);
			for ( i = 0; i < numBytes; i++ ) {
				if ( (tdoData[i] & tdoMask[i]) != (tdoExpected[i] & tdoMask[i]) ) {
					printf("XSDRTDO failed:\n        Got: ");
					dumpSimple(tdoData, numBytes);
					printf("\n       Mask: ");
					dumpSimple(tdoMask, numBytes);
					printf("\n  Expecting: ");
					dumpSimple(tdoExpected, numBytes);
					printf("\n");
					returnCode = -100;
					goto cleanup;
				}
			}
			neroClockFSM(0x00000001, 2);
			if ( xruntest ) {
				neroClocks(xruntest);
			}
			break;

		case XSDRB:
			neroClockFSM(0x00000001, 3);
			numBytes = bitsToBytes(xsdrSize);
			ptr = tdiData;
			while ( numBytes-- ) {
				*ptr++ = getNextByte();
			}
			neroShift(xsdrSize, tdiData, NULL, false);
			break;

		case XSDRC:
			numBytes = bitsToBytes(xsdrSize);
			ptr = tdiData;
			while ( numBytes-- ) {
				*ptr++ = getNextByte();
			}
			neroShift(xsdrSize, tdiData, NULL, false);
			break;

		case XSDRE:
			numBytes = bitsToBytes(xsdrSize);
			ptr = tdiData;
			while ( numBytes-- ) {
				*ptr++ = getNextByte();
			}
			neroShift(xsdrSize, tdiData, NULL, true);
			neroClockFSM(0x00000001, 2);
			if ( xruntest ) {
				neroClocks(xruntest);
			}
			break;

		case XSTATE:
			thisByte = getNextByte();
			if ( thisByte == TAPSTATE_TEST_LOGIC_RESET ) {
				neroClockFSM(0x0000001F, 5);
			} else {
				if ( (0xD3A5>>thisByte) & 0x0001 ) {
					neroClockFSM(0x00000001, 1);
				} else {
					neroClockFSM(0x00000000, 1);
				}
			}
			break;

		default:
			fprintf(stderr, "Unsupported command 0x%02X\n", thisByte);
			returnCode = -101;
			goto cleanup;
		}
		thisByte = getNextByte();
	}
cleanup:
	return returnCode;
}

int main(int argc, const char *argv[]) {
	int returnCode = 0;
	if ( argc != 2 ) {
		fprintf(stderr, "Synopsis: %s <csvfFile>\n", argv[0]);
		fail(-1);
	}
	m_csvfFile = fopen(argv[1], "rb");
	if ( !m_csvfFile ) {
		fprintf(stderr, "Cannot open %s!\n", argv[1]);
		fail(-2);
	}
	if ( neroInitialise(0x04B4, 0x8613) != NERO_SUCCESS ) {
		fprintf(stderr, "%s\n", neroStrError());
		fail(-3);
	}
	
	initReader();
	returnCode = csvfPlay();
	if ( returnCode ) {
		fprintf(stderr, "csvfPlay() failed returnCode %d\n", returnCode);
		goto cleanup;
	}

cleanup:
	neroClose();
	if ( m_csvfFile ) {
		fclose(m_csvfFile);
	}
	return returnCode;
}
