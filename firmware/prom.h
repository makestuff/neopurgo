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
#ifndef PROM_H
#define PROM_H

#include <types.h>

bool promRead(uint16 addr, uint8 length, xdata uint8 *buf);
bool promWrite(uint16 addr, uint8 length, const xdata uint8 *buf);

bool promStartRead(uint16 address);
bool promNextByte(void);
uint8 promPeekByte(void);
bool promStopRead(void);

#endif
