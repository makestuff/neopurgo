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
#ifndef LIBSYNC_H
#define LIBSYNC_H

#include <usbwrap.h>
#include <types.h>

// Possible return codes
typedef enum {
	SYNC_SUCCESS,
	SYNC_ENABLE,
	SYNC_FAILED,
	SYNC_DISABLE
} SyncStatus;

typedef enum {
	SYNC_24,
	SYNC_68,
	SYNC_BOTH
} SyncMode;

#ifdef __cplusplus
extern "C" {
#endif

// Try to sync with the device
SyncStatus syncBulkEndpoints(UsbDeviceHandle *deviceHandle, SyncMode syncMode);

// Get the last error message, or junk if no error occurred.
const char *syncStrError(void);

#ifdef __cplusplus
}
#endif

#endif
