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
#include "jtag.h"

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
