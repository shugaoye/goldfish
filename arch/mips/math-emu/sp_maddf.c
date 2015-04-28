/* IEEE754 floating point arithmetic
 * single precision: MADDF.f
 */
/*
 * MIPS floating point support
 * Copyright (C) 2015 Imagination Technologies, Ltd.
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

ieee754sp ieee754sp_maddf(ieee754sp z, ieee754sp x, ieee754sp y)
{
	int re;
	int rs;
	u32 rm;
	COMPXSP;
	COMPYSP;
	u32 zm; int ze; int zs __maybe_unused; int zc;

	EXPLODEXSP;
	EXPLODEYSP;
	EXPLODESP(z, zc, zs, ze, zm)

	CLEARCX;

	FLUSHXSP;
	FLUSHYSP;
	FLUSHSP(z, zc, zs, ze, zm);

	switch (zc) {
	case IEEE754_CLASS_SNAN:
		SETCX(IEEE754_INVALID_OPERATION);
		return ieee754sp_nanxcpt(ieee754sp_indef(), "maddf", z, x, y);
	case IEEE754_CLASS_DNORM:
		SPDNORMx(zm, ze);
		break;
	}

	/* ==== mult ==== */
	switch (CLPAIR(xc, yc)) {
	case CLPAIR(IEEE754_CLASS_SNAN, IEEE754_CLASS_QNAN):
	case CLPAIR(IEEE754_CLASS_QNAN, IEEE754_CLASS_SNAN):
	case CLPAIR(IEEE754_CLASS_SNAN, IEEE754_CLASS_SNAN):
	case CLPAIR(IEEE754_CLASS_ZERO, IEEE754_CLASS_SNAN):
	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_SNAN):
	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_SNAN):
	case CLPAIR(IEEE754_CLASS_INF, IEEE754_CLASS_SNAN):
	case CLPAIR(IEEE754_CLASS_SNAN, IEEE754_CLASS_ZERO):
	case CLPAIR(IEEE754_CLASS_SNAN, IEEE754_CLASS_NORM):
	case CLPAIR(IEEE754_CLASS_SNAN, IEEE754_CLASS_DNORM):
	case CLPAIR(IEEE754_CLASS_SNAN, IEEE754_CLASS_INF):
		SETCX(IEEE754_INVALID_OPERATION);
		return ieee754sp_nanxcpt(ieee754sp_indef(), "maddf", x, y);

	case CLPAIR(IEEE754_CLASS_ZERO, IEEE754_CLASS_QNAN):
	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_QNAN):
	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_QNAN):
	case CLPAIR(IEEE754_CLASS_INF, IEEE754_CLASS_QNAN):
		return y;

	case CLPAIR(IEEE754_CLASS_QNAN, IEEE754_CLASS_QNAN):
	case CLPAIR(IEEE754_CLASS_QNAN, IEEE754_CLASS_ZERO):
	case CLPAIR(IEEE754_CLASS_QNAN, IEEE754_CLASS_NORM):
	case CLPAIR(IEEE754_CLASS_QNAN, IEEE754_CLASS_DNORM):
	case CLPAIR(IEEE754_CLASS_QNAN, IEEE754_CLASS_INF):
		return x;


		/* Infinity handling */

	case CLPAIR(IEEE754_CLASS_INF, IEEE754_CLASS_ZERO):
	case CLPAIR(IEEE754_CLASS_ZERO, IEEE754_CLASS_INF):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		SETCX(IEEE754_INVALID_OPERATION);
		return ieee754sp_xcpt(ieee754sp_indef(), "mul", x, y);

	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_INF):
	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_INF):
	case CLPAIR(IEEE754_CLASS_INF, IEEE754_CLASS_NORM):
	case CLPAIR(IEEE754_CLASS_INF, IEEE754_CLASS_DNORM):
	case CLPAIR(IEEE754_CLASS_INF, IEEE754_CLASS_INF):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		return ieee754sp_inf(xs ^ ys);

	case CLPAIR(IEEE754_CLASS_ZERO, IEEE754_CLASS_ZERO):
	case CLPAIR(IEEE754_CLASS_ZERO, IEEE754_CLASS_NORM):
	case CLPAIR(IEEE754_CLASS_ZERO, IEEE754_CLASS_DNORM):
	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_ZERO):
	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_ZERO):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		if (zc == IEEE754_CLASS_INF)
			return ieee754sp_inf(zs);
		return z;

	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_DNORM):
		SPDNORMX;

	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_DNORM):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		if (zc == IEEE754_CLASS_INF)
			return ieee754sp_inf(zs);
		SPDNORMY;
		break;

	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_NORM):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		if (zc == IEEE754_CLASS_INF)
			return ieee754sp_inf(zs);
		SPDNORMX;
		break;

	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_NORM):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		if (zc == IEEE754_CLASS_INF)
			return ieee754sp_inf(zs);
		break;
	}
	/* rm = xm * ym, re = xe+ye basically */
	assert(xm & SP_HIDDEN_BIT);
	assert(ym & SP_HIDDEN_BIT);

	re = xe + ye;
	rs = xs ^ ys;

	/* shunt to top of word */
	xm <<= 32 - (SP_MBITS + 1);
	ym <<= 32 - (SP_MBITS + 1);

	/* multiply 32bits xm,ym to give high 32bits rm with stickness
	 */

	{
		unsigned short lxm = xm & 0xffff;
		unsigned short hxm = xm >> 16;
		unsigned short lym = ym & 0xffff;
		unsigned short hym = ym >> 16;
		unsigned lrm;
		unsigned hrm;

		lrm = lxm * lym; /* 16bit * 16bit => 32bit */
		hrm = hxm * hym; /* 16bit * 16bit => 32bit */

		{
			unsigned t = lxm * hym; /* 16 * 16 => 32 */
			{
				unsigned at = lrm + (t << 16);

				hrm += at < lrm;
				lrm = at;
			}
			hrm = hrm + (t >> 16);
		}

		{
			unsigned t = hxm * lym; /* 16 * 16 => 32 */
			{
				unsigned at = lrm + (t << 16);

				hrm += at < lrm;
				lrm = at;
			}
			hrm = hrm + (t >> 16);
		}
		rm = hrm | (lrm != 0);
	}

	/*
	 * sticky shift down to normal rounding precision
	 */
	if ((int) rm < 0) {
		rm =
		    (rm >> (32 - (SP_MBITS + 1 + 3))) |
		    ((rm << (SP_MBITS + 1 + 3)) != 0);
		re++;
	} else {
		rm =
		    (rm >> (32 - (SP_MBITS + 1 + 3 + 1))) |
		    ((rm << (SP_MBITS + 1 + 3 + 1)) != 0);
	}

	assert(rm & (SP_HIDDEN_BIT << 3));

	/* ==== add ==== */

	assert(zm & SP_HIDDEN_BIT);

	/* provide guard,round and stick bit space */
	zm <<= 3;

	if (ze > re) {
		/* have to shift y fraction right to align
		 */
		int s = ze - re;
		rm = XSPSRS(rm, s);
		re += s;
	} else if (re > ze) {
		/* have to shift x fraction right to align
		 */
		int s = re - ze;
		zm = XSPSRS(zm, s);
		ze += s;
	}
	assert(ze == re);
	assert(ze <= SP_EMAX);

	if (zs == rs) {
		/* generate 28 bit result of adding two 27 bit numbers
		 * leaving result in zm,zs,ze
		 */
		zm = zm + rm;
		ze = ze;
		zs = zs;

		if (zm >> (SP_MBITS + 1 + 3)) { /* carry out */
			zm = XSPSRS1(zm); /* shift preserving sticky */
			ze++;
		}
	} else {
		if (zm >= rm) {
			zm = zm - rm;
			ze = ze;
			zs = zs;
		} else {
			zm = rm - zm;
			ze = ze;
			zs = rs;
		}
		if (zm == 0)
			return ieee754sp_zero(ieee754_csr.rm ==
					      IEEE754_RD);

		/* normalize to rounding precision */
		while ((zm >> (SP_MBITS + 3)) == 0) {
			zm <<= 1;
			ze--;
		}

	}
	SPNORMRET3(zs, ze, zm, "maddf", z, x, y);

}
