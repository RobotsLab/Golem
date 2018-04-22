/** @file Extremum.h
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_MATH_EXTREMUM_H_
#define _GOLEM_MATH_EXTREMUM_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Function.h>
#include <vector>

namespace golem {

//------------------------------------------------------------------------------

/** Finds function extrema. */
template <typename REAL>
class _Extremum {
public:
	typedef shared_ptr< _Extremum<REAL> > Ptr;

	static const U32 PARTITIONS_DFLT = 100;
	static const REAL GAIN_DFLT;
	static const REAL EPS_DFLT;

	virtual ~_Extremum() {}

	/** Finds function local extremum starting with the interval [a, b] */
	virtual bool findLocal(REAL &x, REAL &y, const _Function<REAL> &function, REAL a, REAL b, bool min = true) const = 0;

	/** Finds function global extremum within specified interval [left, right] */
	virtual bool findGlobal(REAL &x, REAL &y, const _Function<REAL> &function, REAL left, REAL right, bool min = true) const = 0;
};

template <typename REAL> const REAL _Extremum<REAL>::GAIN_DFLT = REAL(100.0);
template <typename REAL> const REAL _Extremum<REAL>::EPS_DFLT = REAL(1.0e-6);

//------------------------------------------------------------------------------

template <typename REAL>
class _ExtremumBracket : public _Extremum<REAL> {
protected:
	const U32 partitions;
	const REAL gain;

	_ExtremumBracket(U32 partitions, REAL gain) :
		partitions(partitions), gain(gain)
	{}

	/** Brackets the extremum starting with the initial interval [a, b] */
	bool searchLocal(REAL &a, REAL &b, REAL &c, const _Function<REAL> &function, bool min = true) const {
		const REAL sign = min ? REAL(1.0) : -REAL(1.0);

		REAL fa = sign*function.get(a);
		REAL fb = sign*function.get(b);
		if (fb > fa) {
			std::swap(a, b);
			std::swap(fb, fa);
		}
		
		c = a + numeric_const<REAL>::GOLD1*(b - a);
		REAL fc = sign*function.get(c);
		REAL fu;
		while (fb > fc) {
			const REAL r = (b - a) * (fb - fc);
			const REAL q = (b - c) * (fb - fa);
			const REAL ulim = b + gain*(c - b);
			REAL u = b - ((b - c)*q - (b - a)*r)/(REAL(2.0) * Math::guard(q - r, REAL(0.0), numeric_const<REAL>::EPS));

			if ((b - u)*(u - c) > REAL(0.0)) {
				fu = sign*function.get(u);
				
				if (fu < fc) {
					Math::shift(a, b, u);
					Math::shift(fa, fb, fu);
					return true;
				}
				else if (fu > fb) {
					c = u;
					fc = fu;
					return true;
				}

				u = c + numeric_const<REAL>::GOLD1*(c - b);
				fu = sign*function.get(u);
			}
			else if ((c - u)*(u - ulim) > REAL(0.0)) {
				fu = sign*function.get(u);
				
				if (fu < fc) {
					Math::shift(b, c, u, u + numeric_const<REAL>::GOLD1*(u - b));
					Math::shift(fb, fc, fu, sign*function.get(u));
				}
			}
			else if ((u - ulim)*(ulim - c) >= REAL(0.0)) {
				u = ulim;
				fu = sign*function.get(u);
			}
			else {
				u = c + numeric_const<REAL>::GOLD1*(c - b);
				fu = sign*function.get(u);
			}

			Math::shift(a, b, c, u);
			Math::shift(fa, fb, fc, fu);
		}

		return true;
	}

	/** Brackets the extremum in the initial interval [a, c] */
	bool searchGlobal(REAL &a, REAL &b, REAL &c, const _Function<REAL> &function, bool min = true) const {
		const REAL sign = min ? REAL(1.0) : -REAL(1.0);
		const REAL dx = (c - a)/REAL(partitions);
		const REAL left = a;
		
		REAL x0, x1 = left, x2 = left + dx;
		REAL f0, f1 = sign*function.get(x1), f2 = sign*function.get(x2);
		b = x1;
		c = x2;
		REAL fu = f1;
		for (U32 i = 2; i <= partitions; ++i) {
			Math::shift(x0, x1, x2);
			Math::shift(f0, f1, f2);
			
			x2 = left + dx*REAL(i);
			f2 = sign*function.get(x2);
			
			if (f1 < fu && f1 < f0 && f1 < f2) {
				a = x0;
				b = x1;
				c = x2;
				fu = f1;
			}
		}

		if (f2 < fu) {
			a = x0;
			b = x1;
			c = x2;
		}

		return true;
	}

	/** Finds extremum given triple of points {a, b, c} so that a < b < c and f(a) > f(b) < f(c). */
	virtual bool find(REAL &x, REAL &y, const _Function<REAL> &function, REAL a, REAL b, REAL c, bool min = true) const = 0;

public:
	/** Finds function extremum within specified interval */
	virtual bool findLocal(REAL &x, REAL &y, const _Function<REAL> &function, REAL left, REAL right, bool min = true) const {
		REAL a = left, b = right, c;
		if (!searchLocal(a, b, c, function, min))
			return false;

		return find(x, y, function, a, b, c, min);
	}

	/** Finds function extremum within specified interval */
	virtual bool findGlobal(REAL &x, REAL &y, const _Function<REAL> &function, REAL left, REAL right, bool min = true) const {
		REAL a = left, b, c = right;
		if (!searchGlobal(a, b, c, function, min))
			return false;

		return find(x, y, function, a, b, c, min);
	}
};

//------------------------------------------------------------------------------

template <typename REAL>
class _ExtremumGold : public _ExtremumBracket<REAL> {
protected:
	const REAL epsilon;

	bool find(REAL &x, REAL &y, const _Function<REAL> &function, REAL a, REAL b, REAL c, bool min = true) const {
		const REAL sign = min ? REAL(1.0) : -REAL(1.0);
		
		REAL x0 = a, x1, x2, x3 = c;
		if (Math::abs(c - b) > Math::abs(b - a)) {
			x1 = b;
			x2 = b + (REAL(1.0) - numeric_const<REAL>::GOLD2) * (c - b);
		}
		else {
			x2 = b;
			x1 = b - (REAL(1.0) - numeric_const<REAL>::GOLD2) * (b - a);
		}

		REAL f1 = sign*function.get(x1);
		REAL f2 = sign*function.get(x2);
		while (Math::abs(x3 - x0) > epsilon*(Math::abs(x1) + Math::abs(x2)))
			if (f2 < f1) {
				Math::shift(x0, x1, x2, x3 - numeric_const<REAL>::GOLD2*(x3 - x2));
				Math::shift(f1, f2, sign*function.get(x2));
			}
			else {
				Math::shift(x3, x2, x1, x0 + numeric_const<REAL>::GOLD2*(x1 - x0));
				Math::shift(f2, f1, sign*function.get(x1));
			}
		
		if (f1 < f2) {
			x = x1;
			y = sign*f1;
		}
		else {
			x = x2;
			y = sign*f2;
		}

		return true;
	}

public:
	_ExtremumGold(U32 partitions = _Extremum<REAL>::PARTITIONS_DFLT, REAL epsilon = _Extremum<REAL>::EPS_DFLT, REAL gain = _Extremum<REAL>::GAIN_DFLT) :
		_ExtremumBracket<REAL>(partitions, gain), epsilon(epsilon)
	{}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_EXTREMUM_H_*/
