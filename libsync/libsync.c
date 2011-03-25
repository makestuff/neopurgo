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
#include <usbwrap.h>
#include <types.h>
#include "libsync.h"
#include "../vendorCommands.h"

extern char m_syncErrorMessage[];

#define fail(x) returnCode = x; goto cleanup

#define MAX_TRIES 10

// Sync with the device
//
SyncStatus syncBulkEndpoints(UsbDeviceHandle *deviceHandle, SyncMode syncMode) {

	const uint32 hackLower = 0x6861636B;
	const uint32 hackUpper = 0x4841434B;
	int returnCode;
	union {
		uint32 lword;
		char bytes[16];
	} u;
	uint8 count;

	// Put the device in sync mode
	returnCode = usb_control_msg(
		deviceHandle,
		USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		CMD_SYNC_MODE,            // bRequest
		0x0001,                   // wValue
		0x0000,                   // wIndex
		NULL,
		0,                        // wLength
		100                       // timeout (ms)
	);
	if ( returnCode < 0 ) {
		sprintf(m_syncErrorMessage, "syncBulkEndpoints(): Unable to enable sync mode: %s", usb_strerror());
		return SYNC_ENABLE;
	}

	if ( syncMode == SYNC_24 || syncMode == SYNC_BOTH ) {
		// Try to sync EP2OUT->EP4IN
		count = 0;
		do {
			u.lword = hackLower;
			usb_bulk_write(deviceHandle, USB_ENDPOINT_OUT | 2, u.bytes, 4, 100);
			returnCode = usb_bulk_read(deviceHandle, USB_ENDPOINT_IN | 4, u.bytes, 16, 100);
			count++;
		} while ( returnCode < 0 && count < MAX_TRIES );
		if ( count == MAX_TRIES ) {
			sprintf(
				m_syncErrorMessage,
				"syncBulkEndpoints(): Sync of EP2OUT->EP4IN failed after %d attempts with returnCode %d: %s",
				count, returnCode, usb_strerror());
			return SYNC_FAILED;
		}
		if ( returnCode != 4 ) {
			sprintf(
				m_syncErrorMessage,
				"syncBulkEndpoints(): Sync of EP2OUT->EP4IN read back %d bytes instead of the expected 4",
				returnCode);
			return SYNC_FAILED;
		}
		if ( u.lword != hackUpper ) {
			sprintf(
				m_syncErrorMessage,
				"syncBulkEndpoints(): Sync of EP2OUT->EP4IN read back 0x%08lX instead of the expected 0x%08lX",
				u.lword, hackUpper);
			return SYNC_FAILED;
		}
	}

	if ( syncMode == SYNC_68 || syncMode == SYNC_BOTH ) {
		// Try to sync EP6OUT->EP8IN
		count = 0;
		do {
			u.lword = hackLower;
			usb_bulk_write(deviceHandle, USB_ENDPOINT_OUT | 6, u.bytes, 4, 100);
			returnCode = usb_bulk_read(deviceHandle, USB_ENDPOINT_IN | 8, u.bytes, 16, 100);
			count++;
		} while ( returnCode < 0 && count < MAX_TRIES );
		if ( count == MAX_TRIES ) {
			sprintf(
				m_syncErrorMessage,
				"syncBulkEndpoints(): Sync of EP6OUT->EP8IN failed after %d attempts: %s",
				count, usb_strerror());
			return SYNC_FAILED;
		}
		if ( returnCode != 4 ) {
			sprintf(
				m_syncErrorMessage,
				"syncBulkEndpoints(): Sync of EP6OUT->EP8IN read back %d bytes instead of the expected 4",
				returnCode);
			return SYNC_FAILED;
		}
		if ( u.lword != hackUpper ) {
			sprintf(
				m_syncErrorMessage,
				"syncBulkEndpoints(): Sync of EP6OUT->EP8IN read back 0x%08lX instead of the expected 0x%08lX",
				u.lword, hackUpper);
			return SYNC_FAILED;
		}
	}

	// Bring the device out of sync mode
	returnCode = usb_control_msg(
		deviceHandle,
		USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		CMD_SYNC_MODE,            // bRequest
		0x0000,                   // wValue
		0x0000,                   // wIndex
		NULL,
		0,                        // wLength
		100                       // timeout (ms)
	);
	if ( returnCode < 0 ) {
		sprintf(m_syncErrorMessage, "syncBulkEndpoints(): Unable to enable sync mode: %s", usb_strerror());
		return SYNC_DISABLE;
	}

	return SYNC_SUCCESS;
}
