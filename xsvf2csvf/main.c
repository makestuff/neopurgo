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
#include <stdlib.h>

#include <types.h>
#include <buffer.h>

// XSVF commands (from xapp503 appendix B)
//
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
//
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

uint8 getNextByte(void);
#define bitsToBytes(x) ((x>>3) + (x&7 ? 1 : 0))
#define CHECK_BUF_STATUS(x) if ( status != BUF_SUCCESS ) { returnCode = x; goto cleanup; }
#define CHECK_RETURN() if ( returnCode ) { goto cleanup; }
#define FAIL(x) returnCode = x; goto cleanup

#define ENABLE_SWAP
#define BUF_SIZE               128
#define ERROR_BUF_INIT         -1
#define ERROR_BUF_APPEND       -2
#define ERROR_BUF_LOAD         -3
#define ERROR_BUF_SAVE         -4
#define ERROR_UNSUPPORTED_CMD  -5
#define ERROR_UNSUPPORTED_DATA -6
#define ERROR_UNSUPPORTED_SIZE -7
#define ERROR_CMDLINE          -8

// Global buffer and offset used to implement the iterator
//
static Buffer m_xsvfBuf;
static uint32 m_offset;

// The buffer iterator. TODO: refactor to return error code on end of buffer.
//
uint8 getNextByte(void) {
	return m_xsvfBuf.data[m_offset++];
}

// Read "numBytes" bytes from the stream into a temporary buffer, then write them out in the reverse
// order to the supplied buffer "outBuf". If ENABLE_SWAP is undefined, no swapping is done, so the
// output should be identical to the input.
//
static int swapBytes(uint32 numBytes, Buffer *outBuf) {
	int returnCode = 0;
	uint8 swapBuffer[2*BUF_SIZE];  // XSDRTDO accepts 2x XSDRSIZE bytes; all must be swapped
	uint8 *ptr = swapBuffer + numBytes - 1;
	uint32 n = numBytes;
	BufferStatus status;
	while ( n-- ) {
		*ptr-- = getNextByte();
	}
	#ifdef ENABLE_SWAP
		ptr = swapBuffer;
		while ( numBytes-- ) {
			status = bufAppendByte(outBuf, *ptr++); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
		}
	#else
		ptr = swapBuffer + numBytes - 1;
		while ( numBytes-- ) {
			status = bufAppendByte(outBuf, *ptr--); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
		}
	#endif
cleanup:
	return returnCode;
}

// Parse the XSVF, reversing the byte-ordering of all the bytestreams.
//
int xsvfSwapBytes(Buffer *outBuf, uint16 *maxBufSize) {
	int returnCode = 0;
	uint16 xsdrSize = 0;
	uint16 numBytes;
	BufferStatus status;
	uint8 thisByte;

	*maxBufSize = 0;
	thisByte = getNextByte();
	while ( thisByte != XCOMPLETE ) {
		switch ( thisByte ) {
		case XTDOMASK:
			// Swap the XTDOMASK bytes.
			numBytes = bitsToBytes(xsdrSize);
			if ( numBytes > BUF_SIZE ) {
				FAIL(ERROR_UNSUPPORTED_SIZE);
			}
			if ( numBytes > *maxBufSize ) {
				*maxBufSize = numBytes;
			}
			status = bufAppendByte(outBuf, XTDOMASK); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			returnCode = swapBytes(numBytes, outBuf); CHECK_RETURN();
			break;

		case XSDRTDO:
			// Swap the tdiValue and tdoExpected bytes.
			numBytes = bitsToBytes(xsdrSize);
			if ( numBytes > BUF_SIZE ) {
				FAIL(ERROR_UNSUPPORTED_SIZE);
			}
			if ( numBytes > *maxBufSize ) {
				*maxBufSize = numBytes;
			}
			status = bufAppendByte(outBuf, XSDRTDO); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			returnCode = swapBytes(2*numBytes, outBuf); CHECK_RETURN();
			break;

		case XREPEAT:
			// Drop XREPEAT.
			getNextByte();
			break;
			
		case XRUNTEST:
			// Copy the XRUNTEST bytes as-is.
			status = bufAppendByte(outBuf, XRUNTEST); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			status = bufAppendByte(outBuf, getNextByte()); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			status = bufAppendByte(outBuf, getNextByte()); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			status = bufAppendByte(outBuf, getNextByte()); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			status = bufAppendByte(outBuf, getNextByte()); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			break;

		case XSIR:
			// Swap the XSIR bytes.
			status = bufAppendByte(outBuf, XSIR); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			thisByte = getNextByte();
			status = bufAppendByte(outBuf, thisByte); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			returnCode = swapBytes(bitsToBytes(thisByte), outBuf); CHECK_RETURN();
			break;

		case XSDRSIZE:
			// The XSVF spec has the XSDRSIZE as a uint32. But since the bitstreams in XSVF files
			// are big-endian, they have to be buffered first, thus requiring RAM. I'm going to
			// stick my neck out and guess that Xilinx tools will never set XSDRSIZE to anything
			// bigger than 0xFFFF, so we can trim the uint32 down to uint16, and apply a further
			// size check in XSDRTDO and XTDOMASK to put a further limit on it.
			status = bufAppendByte(outBuf, XSDRSIZE); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			if ( getNextByte() ) {
				FAIL(ERROR_UNSUPPORTED_SIZE);  // Fail if either MSW bytes are nonzero
			}
			if ( getNextByte() ) {
				FAIL(ERROR_UNSUPPORTED_SIZE);
			}
			thisByte = getNextByte();  // Get MSB
			xsdrSize = thisByte;
			status = bufAppendByte(outBuf, thisByte); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			thisByte = getNextByte();  // Get LSB
			xsdrSize <<= 8;
			xsdrSize |= thisByte;
			status = bufAppendByte(outBuf, thisByte); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			break;

		case XSDRB:
			// Swap the tdiValue bytes.
			status = bufAppendByte(outBuf, XSDRB); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			returnCode = swapBytes(bitsToBytes(xsdrSize), outBuf); CHECK_RETURN();
			break;

		case XSDRC:
			// Swap the tdiValue bytes.
			status = bufAppendByte(outBuf, XSDRC); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			returnCode = swapBytes(bitsToBytes(xsdrSize), outBuf); CHECK_RETURN();
			break;

		case XSDRE:
			// Swap the tdiValue bytes.
			status = bufAppendByte(outBuf, XSDRE); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			returnCode = swapBytes(bitsToBytes(xsdrSize), outBuf); CHECK_RETURN();
			break;

		case XSTATE:
			// Only switching to states TAPSTATE_TEST_LOGIC_RESET and TAPSTATE_RUN_TEST_IDLE are
			// supported so fail quickly if there's an attempt to switch to a different state.
			status = bufAppendByte(outBuf, XSTATE); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			thisByte = getNextByte();
			if ( thisByte != TAPSTATE_TEST_LOGIC_RESET && thisByte != TAPSTATE_RUN_TEST_IDLE ) {
				FAIL(ERROR_UNSUPPORTED_DATA);
			}
			status = bufAppendByte(outBuf, thisByte); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			break;

		case XENDIR:
			// Only the default XENDIR state (TAPSTATE_RUN_TEST_IDLE) is supported. Fail fast if
			// there's an attempt to switch the XENDIR state to PAUSE_IR.
			thisByte = getNextByte();
			if ( thisByte ) {
				FAIL(ERROR_UNSUPPORTED_DATA);
			}
			break;

		case XENDDR:
			// Only the default XENDDR state (TAPSTATE_RUN_TEST_IDLE) is supported. Fail fast if
			// there's an attempt to switch the XENDDR state to PAUSE_DR.
			thisByte = getNextByte();
			if ( thisByte ) {
				FAIL(ERROR_UNSUPPORTED_DATA);
			}
			break;

		default:
			// All other commands are unsupported, so fail if they're encountered.
			FAIL(ERROR_UNSUPPORTED_CMD);
		}
		thisByte = getNextByte();
	}

	// Add the XCOMPLETE command
	status = bufAppendByte(outBuf, XCOMPLETE); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
	
cleanup:
	return returnCode;
}

int compress(const Buffer *inBuf, Buffer *outBuf) {
	int returnCode = 0;
	const uint8 *runStart, *runEnd, *bufEnd, *chunkStart, *chunkEnd;
	uint32 runLen, chunkLen;
	BufferStatus status;
	bufEnd = inBuf->data + inBuf->length;
	runStart = chunkStart = inBuf->data;
	status = bufAppendByte(outBuf, 0x00); CHECK_BUF_STATUS(ERROR_BUF_APPEND); // Hdr byte: defaults
	while ( runStart < bufEnd ) {
		// Find next zero
		while ( runStart < bufEnd && *runStart ) {
			runStart++;
		}
		
		// Remember the position of the zero
		runEnd = runStart;

		// Find the end of this run of zeros
		while ( runEnd < bufEnd && !*runEnd ) {
			runEnd++;
		}
		
		// Get the length of this run
		runLen = runEnd - runStart;
		
		// If this run is more than four zeros, break the chunk
		if ( runLen > 8 || runEnd == bufEnd ) {
			chunkEnd = runStart;
			chunkLen = chunkEnd - chunkStart;

			// There is now a chunk starting at chunkStart and ending at chunkEnd (length chunkLen),
			// Followed by a run of zeros starting at runStart and ending at runEnd (length runLen).
			//printf("Chunk: %d bytes followed by %d zeros\n", chunkLen, runLen);
			if ( chunkLen < 256 ) {
				// Short chunk: uint8
				status = bufAppendByte(outBuf, (uint8)chunkLen); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			} else {
				// Long chunk: uint16 (big-endian)
				status = bufAppendByte(outBuf, 0x00); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
				status = bufAppendByte(outBuf, (uint8)((chunkLen>>8)&0x000000FF));
				CHECK_BUF_STATUS(ERROR_BUF_APPEND);
				status = bufAppendByte(outBuf, (uint8)(chunkLen&0x000000FF));
				CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			}
			while ( chunkStart < chunkEnd ) {
				status = bufAppendByte(outBuf, *chunkStart++); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			}
			if ( runLen < 256 ) {
				// Short run: uint8
				status = bufAppendByte(outBuf, (uint8)runLen); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			} else {
				// Long run: uint16 (big-endian)
				status = bufAppendByte(outBuf, 0x00); CHECK_BUF_STATUS(ERROR_BUF_APPEND);
				status = bufAppendByte(outBuf, (uint8)((runLen>>8)&0x000000FF));
				CHECK_BUF_STATUS(ERROR_BUF_APPEND);
				status = bufAppendByte(outBuf, (uint8)(runLen&0x000000FF));
				CHECK_BUF_STATUS(ERROR_BUF_APPEND);
			}

			chunkStart = runEnd;
		}
		
		// Start the next round from the end of this run
		runStart = runEnd;
	}

cleanup:
	return returnCode;
}

// Read an XSVF file, convert it to a CSVF file and write it out.
//
int main(int argc, const char *argv[]) {
	int returnCode = 0;
	Buffer swapBuf, csvfBuf;
	BufferStatus status;
	uint16 maxBufSize;
	status = bufInitialise(&m_xsvfBuf, 0x20000, 0); CHECK_BUF_STATUS(ERROR_BUF_INIT);
	status = bufInitialise(&swapBuf, 0x20000, 0); CHECK_BUF_STATUS(ERROR_BUF_INIT);
	status = bufInitialise(&csvfBuf, 0x20000, 0); CHECK_BUF_STATUS(ERROR_BUF_INIT);

	if ( argc != 3 ) {
		fprintf(stderr, "Synopsis: %s <xsvfSource> <csvfDestination>\n", argv[0]);
		FAIL(ERROR_CMDLINE);
	}

	status = bufAppendFromBinaryFile(&m_xsvfBuf, argv[1]);
	CHECK_BUF_STATUS(ERROR_BUF_LOAD);

	returnCode = xsvfSwapBytes(&swapBuf, &maxBufSize);
	CHECK_RETURN();

	printf("#define CSVF_BUFFER_SIZE %d\n", maxBufSize);

	returnCode = compress(&swapBuf, &csvfBuf);

	status = bufWriteBinaryFile(&csvfBuf, argv[2], 0UL, csvfBuf.length);
	//status = bufWriteBinaryFile(&swapBuf, argv[2], 0UL, swapBuf.length);
	CHECK_BUF_STATUS(ERROR_BUF_SAVE);

cleanup:
	bufDestroy(&csvfBuf);
	bufDestroy(&swapBuf);
	bufDestroy(&m_xsvfBuf);

	if ( returnCode ) {
		fprintf(stderr, "Failed returnCode %d\n", returnCode);
	}
	return returnCode;
}
