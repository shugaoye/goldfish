/* IEEE754 floating point arithmetic
 * double precision: MADDF.df
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


#include "ieee754dp.h"

ieee754dp ieee754dp_maddf(ieee754dp z, ieee754dp x, ieee754dp y)
{
	int re;
	int rs;
	u64 rm;
	COMPXDP;
	COMPYDP;
	u64 zm; int ze; int zs __maybe_unused; int zc;

	EXPLODEXDP;
	EXPLODEYDP;
	EXPLODEDP(z, zc, zs, ze, zm)

	CLEARCX;

	FLUSHXDP;
	FLUSHYDP;
	FLUSHDP(z, zc, zs, ze, zm);

	switch (zc) {
	case IEEE754_CLASS_SNAN:
		SETCX(IEEE754_INVALID_OPERATION);
		return ieee754dp_nanxcpt(ieee754dp_indef(), "maddf", z, x, y);
	case IEEE754_CLASS_DNORM:
		DPDNORMx(zm, ze);
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
		return ieee754dp_nanxcpt(ieee754dp_indef(), "maddf", x, y);

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
		return ieee754dp_xcpt(ieee754dp_indef(), "mul", x, y);

	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_INF):
	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_INF):
	case CLPAIR(IEEE754_CLASS_INF, IEEE754_CLASS_NORM):
	case CLPAIR(IEEE754_CLASS_INF, IEEE754_CLASS_DNORM):
	case CLPAIR(IEEE754_CLASS_INF, IEEE754_CLASS_INF):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		return ieee754dp_inf(xs ^ ys);

	case CLPAIR(IEEE754_CLASS_ZERO, IEEE754_CLASS_ZERO):
	case CLPAIR(IEEE754_CLASS_ZERO, IEEE754_CLASS_NORM):
	case CLPAIR(IEEE754_CLASS_ZERO, IEEE754_CLASS_DNORM):
	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_ZERO):
	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_ZERO):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		if (zc == IEEE754_CLASS_INF)
			return ieee754dp_inf(zs);
		return z;

	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_DNORM):
		DPDNORMX;

	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_DNORM):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		if (zc == IEEE754_CLASS_INF)
			return ieee754dp_inf(zs);
		DPDNORMY;
		break;

	case CLPAIR(IEEE754_CLASS_DNORM, IEEE754_CLASS_NORM):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		if (zc == IEEE754_CLASS_INF)
			return ieee754dp_inf(zs);
		DPDNORMX;
		break;

	case CLPAIR(IEEE754_CLASS_NORM, IEEE754_CLASS_NORM):
		if (zc == IEEE754_CLASS_QNAN)
			return z;
		if (zc == IEEE754_CLASS_INF)
			return ieee754dp_inf(zs);
		break;
	}
	/* rm = xm * ym, re = xe+ye basically */
	assert(xm & DP_HIDDEN_BIT);
	assert(ym & DP_HIDDEN_BIT);

	re = xe + ye;
	rs = xs ^ ys;

	/* shunt to top of word */
	xm <<= 64 - (DP_MBITS + 1);
	ym <<= 64 - (DP_MBITS + 1);

	/* multiply 32bits xm,ym to give high 32bits rm with stickness
	 */

	/* 32 * 32 => 64 */
#define DPXMULT(x, y)	((u64)(x) * (u64)y)

	{
		unsigned lxm = xm;
		unsigned hxm = xm >> 32;
		unsigned lym = ym;
		unsigned hym = ym >> 32;
		u64 lrm;
		u64 hrm;

		lrm = DPXMULT(lxm, lym);
		hrm = DPXMULT(hxm, hym);

		{
			u64 t = DPXMULT(lxm, hym);
			{
				u64 at =
				    lrm + (t << 32);
				hrm += at < lrm;
				lrm = at;
			}
			hrm = hrm + (t >> 32);
		}

		{
			u64 t = DPXMULT(hxm, lym);
			{
				u64 at =
				    lrm + (t << 32);
				hrm += at < lrm;
				lrm = at;
			}
			hrm = hrm + (t >> 32);
		}
		rm = hrm | (lrm != 0);
	}

	/*
	 * sticky shift down to normal rounding precision
	 */
	if ((s64) rm < 0) {
		rm =
		    (rm >> (64 - (DP_MBITS + 1 + 3))) |
		    ((rm << (DP_MBITS + 1 + 3)) != 0);
		re++;
	} else {
		rm =
		    (rm >> (64 - (DP_MBITS + 1 + 3 + 1))) |
		    ((rm << (DP_MBITS + 1 + 3 + 1)) != 0);
	}

	assert(rm & (DP_HIDDEN_BIT << 3));

	/* ==== add ==== */

	assert(zm & DP_HIDDEN_BIT);

	/* provide guard,round and stick bit space */
	zm <<= 3;

	if (ze > re) {
		/* have to shift y fraction right to align
		 */
		int s = ze - re;
		rm = XDPSRS(rm, s);
		re += s;
	} else if (re > ze) {
		/* have to shift x fraction right to align
		 */
		int s = re - ze;
		zm = XDPSRS(zm, s);
		ze += s;
	}
	assert(ze == re);
	assert(ze <= DP_EMAX);

	if (zs == rs) {
		/* generate 60 bit result of adding two 59 bit numbers
		 * leaving result in zm,zs,ze
		 */
		zm = zm + rm;
		ze = ze;
		zs = zs;

		if (zm >> (DP_MBITS + 1 + 3)) { /* carry out */
			zm = XDPSRS1(zm);
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
			return ieee754dp_zero(ieee754_csr.rm ==
					      IEEE754_RD);

		/* normalize to rounding precision */
		while ((zm >> (DP_MBITS + 3)) == 0) {
			zm <<= 1;
			ze--;
		}

	}
	DPNORMRET3(zs, ze, zm, "maddf", z, x, y);

}
