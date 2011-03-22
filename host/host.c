/* 
 * Copyright (C) 2009-2011 Chris McClelland
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
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <string.h>
#include "libsync.h"
#include "usbwrap.h"
#include "argtable2.h"
#include "arg_uint.h"
#ifdef WIN32
#include <Windows.h>
#else
#include <sys/time.h>
#endif

//#define VID 0x1443
//#define PID 0x0005
#define VID 0x04B4
#define PID 0x8613

void dumpSimple(const unsigned char *input, unsigned int length) {
	while ( length ) {
		printf(" %02X", *input++);
		--length;
	}
}

#define ENDPOINT_SIZE 512

#define bmNEEDRESPONSE (1<<0)
#define bmISLAST       (1<<1)
#define bmSENDZEROS    (0<<2)
#define bmSENDONES     (1<<2)
#define bmSENDDATA     (2<<2)
#define bmSENDMASK     (3<<2)

int main(int argc, char *argv[]) {

	struct arg_uint *vidOpt  = arg_uint0("v", "vid", "<vendorID>", "  vendor ID");
	struct arg_uint *pidOpt  = arg_uint0("p", "pid", "<productID>", " product ID");
	struct arg_lit  *intOpt  = arg_lit0("i", "interactive", "       start in interactive mode");
	struct arg_lit  *helpOpt = arg_lit0("h", "help", "            print this help and exit\n");
	struct arg_end  *endOpt  = arg_end(20);
	void* argTable[] = {vidOpt, pidOpt, intOpt, helpOpt, endOpt};
	const char *progName = "bulk";
	uint32 exitCode = 0;
	int numErrors;
	const int outEndpoint = 0x06;
	const int inEndpoint = 0x08;
	FILE *inFile = NULL;
	uint8 *buffer = NULL;
	uint32 fileLen;
	UsbDeviceHandle *deviceHandle = NULL;
	int returnCode;
	uint16 vid, pid;
	char lineBuf[1026];
	char *linePtr;
	uint8 byteBuf[513];
	uint8 *bytePtr;
	int byteCount;
	uint8 highNibble, lowNibble;
	union {
		uint32 lword;
		uint8 bytes[4];
	} u;
	double totalTime, speed;
	uint16 checksum;
	#ifdef WIN32
		LARGE_INTEGER tvStart, tvEnd, freq;
		DWORD_PTR mask = 1;
		SetThreadAffinityMask(GetCurrentThread(), mask);
		QueryPerformanceFrequency(&freq);
	#else
		struct timeval tvStart, tvEnd;
		long long startTime, endTime;
	#endif

	if ( arg_nullcheck(argTable) != 0 ) {
		printf("%s: insufficient memory\n", progName);
		exitCode = 1;
		goto cleanup;
	}

	numErrors = arg_parse(argc, argv, argTable);

	if ( helpOpt->count > 0 ) {
		printf("Bulk Write Tool Copyright (C) 2009-2010 Chris McClelland\n\nUsage: %s", progName);
		arg_print_syntax(stdout, argTable, "\n");
		printf("\nWrite data to a bulk endpoint.\n\n");
		arg_print_glossary(stdout, argTable,"  %-10s %s\n");
		exitCode = 0;
		goto cleanup;
	}

	if ( numErrors > 0 ) {
		arg_print_errors(stdout, endOpt, progName);
		printf("Try '%s --help' for more information.\n", progName);
		exitCode = 2;
		goto cleanup;
	}

	vid = vidOpt->count ? (uint16)vidOpt->ival[0] : VID;
	pid = pidOpt->count ? (uint16)pidOpt->ival[0] : PID;

	usbInitialise();
	returnCode = usbOpenDevice(vid, pid, 1, 0, 0, &deviceHandle);
	if ( returnCode ) {
		fprintf(stderr, "usbOpenDevice() failed returnCode %d: %s\n", returnCode, usbStrError());
		exitCode = 6;
		goto cleanup;
	}

	if ( syncBulkEndpoints(deviceHandle) ) {
		printf("Failed to sync bulk endpoints: %s\n", syncStrError());
		exitCode = 100;
		goto cleanup;
	}

	if ( intOpt->count ) {
		for (  ; ; ) {
			printf("> ");
			if ( !fgets(lineBuf, 1026, stdin) ) {
				fprintf(stderr, "Error getting line from terminal!\n");
				break;
			}
			linePtr = lineBuf;
			bytePtr = byteBuf;
			if ( *linePtr == 'h' ) {
				printf("Write one byte 0xAA to register 0x00:\n");
				printf("  0000000001AA\n");
				printf("Write two bytes 0xCA 0xFE to register 0x00:\n");
				printf("  0000000002CAFE\n");
				printf("Write one byte 0x55 to register 0x01:\n");
				printf("  010000000155\n");
				printf("Write two bytes 0xBA 0xBE to register 0x01:\n");
				printf("  0100000002BABE\n");
				printf("Write 0xF00D1E to registers 0x01, 0x02, 0x03:\n");
				printf("  0100000001F002000000010D03000000011E\n");
				printf("Read register 0x00 once:\n");
				printf("  8000000001\n");
				printf("Read register 0x01 once:\n");
				printf("  8100000001\n");
				printf("Read register 0x02 once:\n");
				printf("  8200000001\n");
				printf("Read register 0x03 once:\n");
				printf("  8300000001\n");
				printf("Read register 0x00 twice:\n");
				printf("  8000000002\n");
				printf("Write file ../firmware/random.dat to register 0x00:\n");
				printf("  /00 ../firmware/random.dat\n");
			} else if ( *linePtr == 'q' ) {
				break;
			} else if ( *linePtr == '/' ) {
				uint32 i;
				while ( *linePtr != '\n' ) {
					linePtr++;
				}
				*linePtr = '\0';
				linePtr = lineBuf + 1;
				highNibble = *linePtr++;
				if ( highNibble >= 'A' && highNibble <= 'Z' ) {
					highNibble -= 'A' - 10;
				} else if ( highNibble >= 'a' && highNibble <= 'z' ) {
					highNibble -= 'a' - 10;
				} else if ( highNibble >= '0' && highNibble <= '9' ) {
					highNibble -= '0';
				}
				lowNibble = *linePtr++;
				if ( lowNibble >= 'A' && lowNibble <= 'Z' ) {
					lowNibble -= 'A' - 10;
				} else if ( lowNibble >= 'a' && lowNibble <= 'z' ) {
					lowNibble -= 'a' - 10;
				} else if ( lowNibble >= '0' && lowNibble <= '9' ) {
					lowNibble -= '0';
				}
				inFile = fopen(lineBuf+4, "rb");
				if ( !inFile ) {
					fprintf(stderr, "Unable to open file \"%s\"\n", lineBuf+4);
					continue;
				}
				fseek(inFile, 0, SEEK_END);
				fileLen = ftell(inFile);
				fseek(inFile, 0, SEEK_SET);
				byteCount = fileLen + 5;
				buffer = (uint8 *)malloc(byteCount);
				if ( !buffer ) {
					fprintf(stderr, "Unable to allocate memory for file \"%s\"\n", lineBuf+4);
					fclose(inFile);
					continue;
				}
				u.lword = fileLen;
				highNibble &= 0x07;
				buffer[0] = (highNibble << 4) | lowNibble;
				buffer[1] = u.bytes[3];
				buffer[2] = u.bytes[2];
				buffer[3] = u.bytes[1];
				buffer[4] = u.bytes[0];
				if ( fread(buffer+5, 1, fileLen, inFile) != fileLen ) {
					fprintf(stderr, "Unable to read file \"%s\"\n", lineBuf+4);
					fclose(inFile);
					free(buffer);
					continue;
				}
				checksum = 0x0000;
				for ( i = 0; i < fileLen; i++  ) {
					checksum += buffer[5+i];
				}
				printf("Writing five command bytes followed by %d data bytes:", fileLen);
				dumpSimple(buffer, 6);
				printf(" ...");
				dumpSimple(buffer+byteCount-1, 1);
				printf("\n");
				#ifdef WIN32
					QueryPerformanceCounter(&tvStart);
					returnCode = usb_bulk_write(deviceHandle, USB_ENDPOINT_OUT | outEndpoint, (char*)buffer, byteCount, 5000);
					QueryPerformanceCounter(&tvEnd);
				#else
					gettimeofday(&tvStart, NULL);
					returnCode = usb_bulk_write(deviceHandle, USB_ENDPOINT_OUT | outEndpoint, (char*)buffer, byteCount, 5000);
					gettimeofday(&tvEnd, NULL);
				#endif
				if ( returnCode != byteCount ) {
					printf("Expected to write %d bytes to endpoint %d but actually wrote %d: %s\n", byteCount, outEndpoint, returnCode, usb_strerror());
					fclose(inFile);
					free(buffer);
					continue;
				}
				#ifdef WIN32
					totalTime = (double)(tvEnd.QuadPart - tvStart.QuadPart);
					totalTime /= freq.QuadPart;
					printf("Time: %fms\n", totalTime*1000.0);
					speed = (double)fileLen / (1024*1024*totalTime);
				#else
					startTime = tvStart.tv_sec;
					startTime *= 1000000;
					startTime += tvStart.tv_usec;
					endTime = tvEnd.tv_sec;
					endTime *= 1000000;
					endTime += tvEnd.tv_usec;
					totalTime = endTime - startTime;
					totalTime /= 1000000;  // convert from uS to S.
					printf("Time: %fms\n", totalTime*1000.0);
					speed = (double)fileLen / (1024*1024*totalTime);
				#endif
				printf("Speed: %f MB/s\nChecksum: 0x%04X\n", speed, checksum);
				fclose(inFile);
				inFile = NULL;
				free(buffer);
				buffer = NULL;
			} else {
				while (
					(*linePtr >= '0' && *linePtr <= '9') ||
					(*linePtr >= 'A' && *linePtr <= 'Z') ||
					(*linePtr >= 'a' && *linePtr <= 'z') )
				{
					highNibble = *linePtr++;
					if ( highNibble >= 'A' && highNibble <= 'Z' ) {
						highNibble -= 'A' - 10;
					} else if ( highNibble >= 'a' && highNibble <= 'z' ) {
						highNibble -= 'a' - 10;
					} else if ( highNibble >= '0' && highNibble <= '9' ) {
						highNibble -= '0';
					}
					lowNibble = *linePtr++;
					if ( lowNibble >= 'A' && lowNibble <= 'Z' ) {
						lowNibble -= 'A' - 10;
					} else if ( lowNibble >= 'a' && lowNibble <= 'z' ) {
						lowNibble -= 'a' - 10;
					} else if ( lowNibble >= '0' && lowNibble <= '9' ) {
						lowNibble -= '0';
					}
					*bytePtr++ = (highNibble << 4) | lowNibble;
				}
				byteCount = bytePtr - byteBuf;
	
				printf("Writing: ");
				dumpSimple(byteBuf, byteCount);
				printf("\n");
				returnCode = usb_bulk_write(deviceHandle, USB_ENDPOINT_OUT | outEndpoint, (char*)byteBuf, byteCount, 5000);
				if ( returnCode != byteCount ) {
					printf("Expected to write %d bytes to endpoint %d but actually wrote %d: %s\n", byteCount, outEndpoint, returnCode, usb_strerror());
					continue;
				}
				if ( byteBuf[0] & 0x80 ) {
					returnCode = usb_bulk_read(deviceHandle, USB_ENDPOINT_IN | inEndpoint, (char*)byteBuf, 16, 5000);
					if ( returnCode > 0 ) {
						printf("Read %d bytes from endpoint %d: ", returnCode, inEndpoint);
						dumpSimple(byteBuf, returnCode);
						printf("\n");
					} else if ( returnCode < 0 ) {
						printf("Error whilst reading: %s\n", usb_strerror());
						continue;
					}
				}
			}
		}
	} else {
		uint8 i;
		for ( i = 0; i < 4; i++ ) {
			byteBuf[0] = 0x80 | i;
			byteBuf[1] = 0x00;
			byteBuf[2] = 0x00;
			byteBuf[3] = 0x00;
			byteBuf[4] = 0x01;
			returnCode = usb_bulk_write(
				deviceHandle, USB_ENDPOINT_OUT | outEndpoint, (char*)byteBuf, 5, 100);
			if ( returnCode != 5 ) {
				printf("Error whilst writing (returnCode=%d): %s\n", returnCode, usb_strerror());
				exitCode = 102;
				goto cleanup;
			}
			returnCode = usb_bulk_read(
				deviceHandle, USB_ENDPOINT_IN | inEndpoint, (char*)byteBuf, 16, 5000);
			if ( returnCode == 1 ) {
				printf("R%d = 0x%02X\n", i, byteBuf[0]);
			} else {
				printf("Error whilst reading (returnCode=%d): %s\n", returnCode, usb_strerror());
				exitCode = 103;
				goto cleanup;
			}
		}
	}
cleanup:
	if ( buffer ) {
		free(buffer);
	}
	if ( inFile ) {
		fclose(inFile);
	}
	if ( deviceHandle ) {
		usb_release_interface(deviceHandle, 0);
		usb_close(deviceHandle);
	}
	arg_freetable(argTable, sizeof(argTable)/sizeof(argTable[0]));

	return exitCode;
}
