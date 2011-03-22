#!/bin/bash

cat > checksum.c <<EOF
#include <stdio.h>
#include <stdlib.h>
#include "types.h"
int main(int argc, char *argv[]) {
	const char *fileName;
	uint32 fileLen;
	uint32 i, numBytes;
	uint16 checkSum = 0x0000;
	uint8 *buffer, *ptr;
	FILE *inFile;
	
	if ( argc != 2 && argc != 3 ) {
		fprintf(stderr, "Synopsis: checksum <fileName> [<numBytes>]\\n");
		exit(1);
	}

	fileName = argv[1];
	
	inFile = fopen(fileName, "rb");
	if ( !inFile ) {
		fprintf(stderr, "Unable to open file \\"%s\\"\\n", fileName);
		exit(1);
	}
	fseek(inFile, 0, SEEK_END);
	fileLen = ftell(inFile);
	fseek(inFile, 0, SEEK_SET);
	buffer = (uint8 *)malloc(fileLen);
	if ( !buffer ) {
		fprintf(stderr, "Unable to allocate memory for file \\"%s\\"\\n", fileName);
		fclose(inFile);
		exit(2);
	}
	if ( fread(buffer, 1, fileLen, inFile) != fileLen ) {
		fprintf(stderr, "Unable to read file \\"%s\\"\\n", fileName);
		fclose(inFile);
		free(buffer);
		exit(3);
	}

	if ( argc == 3 ) {
		numBytes = strtoul(argv[2], NULL, 10);
	} else {
		numBytes = fileLen;
	}

	//printf("Read %d bytes from %s\\n", fileLen, fileName);

	ptr = buffer;
	for ( i = 0; i < numBytes; i++ ) {
		checkSum += *ptr++;
	}
	
	printf("0x%04X\\n", checkSum);

	free(buffer);
	fclose(inFile);
}
EOF
if [ "$OS" == "Windows_NT" ]; then
	cl -nologo -DWIN32 -I../../../include checksum.c
else
	gcc -I../../../include -Wall -Wextra -Wstrict-prototypes -Wundef -std=c99 -pedantic-errors -o checksum checksum.c
fi
printf "random.dat checksum = "
./checksum random.dat
rm -f checksum.c checksum.obj checksum checksum.exe
