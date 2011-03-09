/* 
 * Copyright (C) 2009 Chris McClelland
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

/*int main(int argc, char *argv[]) {
	const char *fileName = "fx2fpga.bit";
	uint32 fileLen, runLen, maxRunLen = 0;
	const uint8 *runStart, *runEnd, *bufEnd;
	uint8 *buffer;
	FILE *inFile = fopen(fileName, "rb");
	if ( !inFile ) {
		fprintf(stderr, "Unable to open file \"%s\"\n", fileName);
		exit(1);
	}
	fseek(inFile, 0, SEEK_END);
	fileLen = ftell(inFile);
	fseek(inFile, 0, SEEK_SET);
	buffer = (uint8 *)malloc(fileLen);
	if ( !buffer ) {
		fprintf(stderr, "Unable to allocate memory for file \"%s\"\n", fileName);
		fclose(inFile);
		exit(2);
	}
	if ( fread(buffer, 1, fileLen, inFile) != fileLen ) {
		fprintf(stderr, "Unable to read file \"%s\"\n", fileName);
		fclose(inFile);
		free(buffer);
		exit(3);
	}

	printf("Read %d bytes from %s\n", fileLen, fileName);

	bufEnd = buffer + fileLen;
	runStart = buffer;
	while ( runStart < bufEnd ) {
		while ( runStart < bufEnd && *runStart ) {
			runStart++;
		}
		runEnd = runStart;
		while ( runEnd < bufEnd && !*runEnd ) {
			runEnd++;
		}
		runLen = runEnd - runStart;
		if ( runLen > 128 ) {
			printf("Run length = %d\n", runLen);
		}
		if ( runLen > maxRunLen ) {
			maxRunLen = runLen;
		}
		runStart = runEnd;
	}

	printf("Max run length = %d\n", maxRunLen);

	free(buffer);
	fclose(inFile);
}
*/


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
	unsigned short checksum = 0x0000;
	char nop = 0xFF;
	//uint32 i;
	char lineBuf[1026];
	char *linePtr;
	uint8 byteBuf[513];
	uint8 *bytePtr;
	int byteCount;
	uint8 highNibble, lowNibble;

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

	if ( intOpt->count ) {
		usb_clear_halt(deviceHandle, USB_ENDPOINT_OUT | outEndpoint);
		usb_clear_halt(deviceHandle, USB_ENDPOINT_IN | inEndpoint);
		returnCode = usb_bulk_write(deviceHandle, USB_ENDPOINT_OUT | outEndpoint, &nop, 1, 1000);
		returnCode = usb_bulk_write(deviceHandle, USB_ENDPOINT_OUT | outEndpoint, &nop, 1, 1000);
		//returnCode = usb_bulk_read(deviceHandle, USB_ENDPOINT_IN | inEndpoint, (char*)byteBuf, 16, 1000);
		//returnCode = usb_bulk_read(deviceHandle, USB_ENDPOINT_IN | inEndpoint, (char*)byteBuf, 16, 1000);
		for (  ; ; ) {
			printf("> ");
			fgets(lineBuf, 1026, stdin);
			linePtr = lineBuf;
			bytePtr = byteBuf;
			if ( *linePtr == 'h' ) {
				printf("Write one byte 0xAA to register 0x00:\n");
				printf("  0001000000AA\n");
				printf("Write two bytes 0xCA 0xFE to register 0x00:\n");
				printf("  0002000000CAFE\n");
				printf("Write one byte 0x55 to register 0x01:\n");
				printf("  010100000055\n");
				printf("Write two bytes 0xBA 0xBE to register 0x01:\n");
				printf("  0102000000BABE\n");
				printf("Write file small.dat to register 0x00:\n");
				printf("  /00 small.dat\n");
				printf("Write file r1024.dat to register 0x01:\n");
				printf("  /01 r1024.dat\n");
			} else if ( *linePtr == 'q' ) {
				break;
			} else if ( *linePtr == 'x' ) {
				if ( linePtr[1] == 'd' ) {
					// -----------------------------------------------------
					// Need to send data
					const char *text = "Nor have we been wanting in attention to our British brethren. We have warned them from time to time of attempts by their legislature to extend an unwarrantable jurisdiction over us. We have reminded them of the circumstances of our emigration and settlement here. We have appealed to their native justice and magnanimity, and we have conjured them by the ties of our common kindred to disavow these usurpations, which, would inevitably interrupt our connections and correspondence. They too have been deaf to the voice of justice and of consanguinity. We must, therefore, acquiesce in the necessity, which denounces our separation, and hold them, as we hold the rest of mankind, enemies in war, in peace friends.";
					const char *ptr = text;
					uint32 numBytes = strlen(text);
					uint16 chunkSize;
					*((uint32 *)byteBuf) = numBytes;
					if ( linePtr[2] == 'r' ) {
						// Need to send data, and need a response
						returnCode = usb_control_msg(
							deviceHandle,
							USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
							0x80, (bmSENDDATA | bmNEEDRESPONSE), 0x0000, byteBuf, 4, 5000
						);
						if ( returnCode != 4 ) {
							printf("Error whilst sending control message: %s\n", usb_strerror());
							continue;
						}
		
						do {
							chunkSize = (numBytes>=512) ? 512 : (uint16)numBytes;
							strncpy(byteBuf, ptr, chunkSize);
							ptr += chunkSize;
							numBytes -= chunkSize;
							returnCode = usb_bulk_write(
								deviceHandle, USB_ENDPOINT_OUT | 2, (char*)byteBuf, chunkSize, 1000);
							if ( returnCode != chunkSize ) {
								printf("Error whilst writing (returnCode=%d): %s\n", returnCode, usb_strerror());
								continue;
							}
							byteBuf[chunkSize] = 0x00;
							printf("Wrote %d bytes to endpoint 2: \"%s\"\n", chunkSize, byteBuf);
							returnCode = usb_bulk_read(
								deviceHandle, USB_ENDPOINT_IN | 4, (char*)byteBuf, chunkSize, 1000);
							if ( returnCode != chunkSize ) {
								printf("Error whilst reading (returnCode=%d): %s\n", returnCode, usb_strerror());
								continue;
							}
							byteBuf[chunkSize] = 0x00;
							printf("Read %d bytes from endpoint 4: \"%s\"\n", returnCode, byteBuf);
							//printf("Read %d bytes from endpoint 4: ", returnCode);
							//dumpSimple(byteBuf, returnCode);
							//printf("\n");
						} while ( numBytes );
					} else {
						// Need to send data, but no response is needed
						returnCode = usb_control_msg(
							deviceHandle,
							USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
							0x80, bmSENDDATA, 0x0000, byteBuf, 4, 5000
						);
						if ( returnCode != 4 ) {
							printf("Error whilst sending control message: %s\n", usb_strerror());
							continue;
						}
						returnCode = usb_bulk_write(
							deviceHandle, USB_ENDPOINT_OUT | 2, (char*)text, numBytes, 1000);
						if ( returnCode != numBytes ) {
							printf("Error whilst writing (returnCode=%d): %s\n", returnCode, usb_strerror());
							continue;
						}
						printf("Wrote %d bytes to endpoint 2: \"%s\"\n", numBytes, text);
					}
				} else {
					// -----------------------------------------------------
					// Not sending
					*((uint32 *)byteBuf) = 400;
					if ( linePtr[2] == 'r' ) {
						// Not sending data, but a response is needed
						returnCode = usb_control_msg(
							deviceHandle,
							USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
							0x80, (bmSENDZEROS | bmNEEDRESPONSE), 0x0000, byteBuf, 4, 5000
						);
						if ( returnCode != 4 ) {
							printf("Error whilst sending control message: %s\n", usb_strerror());
							continue;
						}
						returnCode = usb_bulk_read(
							deviceHandle, USB_ENDPOINT_IN | 4, (char*)byteBuf, 400, 1000);
						if ( returnCode != 400 ) {
							printf("Error whilst reading (returnCode=%d): %s\n", returnCode, usb_strerror());
							continue;
						}
						printf("Read %d bytes from endpoint 4: ", returnCode);
						dumpSimple(byteBuf, returnCode);
						printf("\n");
					} else {
						// Not sending data, no response is needed
						returnCode = usb_control_msg(
							deviceHandle,
							USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
							0x80, bmSENDZEROS, 0x0000, byteBuf, 4, 5000
						);
						if ( returnCode != 4 ) {
							printf("Error whilst sending control message: %s\n", usb_strerror());
							continue;
						}
					}
				}
			} else if ( *linePtr == 'r' ) {
				returnCode = usb_bulk_read(deviceHandle, USB_ENDPOINT_IN | inEndpoint, (char*)byteBuf, 16, 5000);
				if ( returnCode > 0 ) {
					printf("Read %d bytes from endpoint %d: ", returnCode, inEndpoint);
					dumpSimple(byteBuf, returnCode);
					printf("\n");
				} else if ( returnCode < 0 ) {
					printf("Error whilst reading from endpoint %d: %s\n", inEndpoint, usb_strerror());
					continue;
				}
			} else if ( *linePtr == '/' ) {
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
				buffer[0] = (highNibble << 4) | lowNibble;
				*((uint32 *)(buffer+1)) = fileLen;
				if ( fread(buffer+5, 1, fileLen, inFile) != fileLen ) {
					fprintf(stderr, "Unable to read file \"%s\"\n", lineBuf+4);
					fclose(inFile);
					free(buffer);
					continue;
				}
				printf("Writing %d bytes:", byteCount);
				dumpSimple(buffer, 6);
				printf(" ...");
				dumpSimple(buffer+byteCount-1, 1);
				printf("\n");
				returnCode = usb_bulk_write(deviceHandle, USB_ENDPOINT_OUT | outEndpoint, (char*)buffer, byteCount, 5000);
				if ( returnCode != byteCount ) {
					printf("Expected to write %lu bytes to endpoint %d but actually wrote %d: %s\n", byteCount, outEndpoint, returnCode, usb_strerror());
					fclose(inFile);
					free(buffer);
					continue;
				}
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
					printf("Expected to write %lu bytes to endpoint %d but actually wrote %d: %s\n", fileLen, outEndpoint, returnCode, usb_strerror());
					continue;
				}
				//if ( byteBuf[0] & 0x80 ) {
				if ( byteBuf[0] == 0xfe ) {
					//printf("Sleeping...\n");
					//Sleep(2000);
					//printf("...done\n");
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
		// Ugly hack to clear any blockages
		usb_clear_halt(deviceHandle, USB_ENDPOINT_OUT | outEndpoint);
		usb_clear_halt(deviceHandle, USB_ENDPOINT_IN | inEndpoint);
		do {
			returnCode = usb_bulk_read(
				deviceHandle, USB_ENDPOINT_IN | inEndpoint, (char*)byteBuf, 16, 100);
		} while ( returnCode >= 0 );
		do {
			byteBuf[0] = 0xfe;
			usb_bulk_write(deviceHandle, USB_ENDPOINT_OUT | outEndpoint, byteBuf, 1, 100);
			returnCode = usb_bulk_read(
				deviceHandle, USB_ENDPOINT_IN | inEndpoint, (char*)byteBuf, 16, 100);
		} while ( returnCode < 0 );

		//returnCode = usb_control_msg(
		//	deviceHandle,
		//	USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		//	0x80, 0x0010, 0x0002, byteBuf, 8, 5000
		//);
		//if ( returnCode == 8 ) {
		//	printf("usb_control_msg(0x80, 0x0010, 0x0002) returned: ");
		//	dumpSimple(byteBuf, 8);
		//	printf("\n");
		//} else {
		//	printf("Error whilst sending control message: %s\n", usb_strerror());
		//	exitCode = 100;
		//	goto cleanup;
		//}

		byteBuf[0] = 0xfe;
		returnCode = usb_bulk_write(
			deviceHandle, USB_ENDPOINT_OUT | outEndpoint, (char*)byteBuf, 1, 1000);
		if ( returnCode != 1 ) {
			printf("Error whilst writing (returnCode=%d): %s\n", returnCode, usb_strerror());
			exitCode = 102;
			goto cleanup;
		}
		returnCode = usb_bulk_read(
			deviceHandle, USB_ENDPOINT_IN | inEndpoint, (char*)byteBuf, 16, 5000);
		if ( returnCode > 0 ) {
			printf("Read %d bytes from endpoint %d: ", returnCode, inEndpoint);
			dumpSimple(byteBuf, returnCode);
			printf("\n");
		} else if ( returnCode < 0 ) {
			printf("Error whilst reading (returnCode=%d): %s\n", returnCode, usb_strerror());
			exitCode = 103;
			goto cleanup;
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
