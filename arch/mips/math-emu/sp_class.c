/* IEEE754-2008 floating point arithmetic
 * single precision: CLASS
 */
/*
 * MIPS floating point support
 * Copyright (C) 2015, Imagination Technologies, LTD.
 * Author:      Leonid.Yegoshin@imgtec.com
 *
 * ########################################################################
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * ########################################################################
 */


#include "ieee754sp.h"

/*
 * CLASS.fmt returns:
 *
 *  bit 0: SNAN
 *      1: QNAN
 *      2: INF      if value < 0
 *      3: xNORM    if value < 0

 *      4: subNORM  if value < 0
 *      5: ZERO     (-0)
 *      6: INF      if value > 0
 *      7: xNORM    if value > 0

 *      8: subNORM  if value > 0
 *      9: ZERO     (+0)
 */

int ieee754_2008sp_class(ieee754sp x)
{
	COMPXSP;

	EXPLODEXSP;

	/* convert IEEE754 to 754_2008 */
	switch (xc) {
	case IEEE754_CLASS_QNAN:
		return 1;
	case IEEE754_CLASS_SNAN:
		return 0;
	case IEEE754_CLASS_ZERO:
		if (xs)
			return 0x20;
		return 0x200;
	case IEEE754_CLASS_INF:
		if (xs)
			return 0x4;
		return 0x40;
	case IEEE754_CLASS_DNORM:
		if (xs)
			return 0x10;
		return 0x100;
	case IEEE754_CLASS_NORM:
		if (xs)
			return 0x8;
		return 0x80;
	default:
		return 0x0;
	}
}
