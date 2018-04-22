/** @file Math.h
 * 
 * Mathematical routines.
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
#ifndef _GOLEM_MATH_MATH_H_
#define _GOLEM_MATH_MATH_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Types.h>
#include <Golem/Defs/Constants.h>
#include <Golem/Defs/Defs.h>
#include <stdlib.h>
#include <cmath>
#include <float.h>
#include <algorithm>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Floating-point type used in the project */
typedef F64 Real;
/** Floating-point constants for Real type */
const Real REAL_ZERO = numeric_const<Real>::ZERO;
const Real REAL_ONE = numeric_const<Real>::ONE;
const Real REAL_TWO = numeric_const<Real>::TWO;
const Real REAL_HALF = numeric_const<Real>::HALF;
const Real REAL_PI = numeric_const<Real>::PI;
const Real REAL_2_PI = numeric_const<Real>::TWO_PI;
const Real REAL_PI_2 = numeric_const<Real>::PI_2;
const Real REAL_MIN = numeric_const<Real>::MIN;
const Real REAL_MAX = numeric_const<Real>::MAX;
const Real REAL_EPS = numeric_const<Real>::EPS;

//------------------------------------------------------------------------------

class Math {
public:
	template <typename Type> inline static void rotate(Type &a, Type &b, Type &c) {
		const Type x = a; a = b; b = c; c = x;
	}
	
	template <typename Type> inline static void rotate(Type &a, Type &b, Type &c, Type &d) {
		const Type x = a; a = b; b = c; c = d; d = x;
	}
	
	template <typename Type> inline static void shift(Type &a, Type &b, Type c) {
		a = b; b = c;
	}
	
	template <typename Type> inline static void shift(Type &a, Type &b, Type &c, Type d) {
		a = b; b = c; c = d;
	}
	
	template <typename Type> inline static Type dist(Type a, Type b) {
		return a > b ? a - b : b - a;
	}

	template <typename Type> inline static Type sign(Type a, Type sign)  {
		return sign < numeric_const<Type>::ZERO ? -a : a;
	}

	template <typename Type> inline static Type clamp(Type a, Type min, Type max)  {
		return a < min ? min : a > max ? max : a;
	}

	template <typename Ptr, typename ConstPtr> inline static void clamp(Ptr begin, Ptr end, ConstPtr min, ConstPtr max)  {
		while (begin != end) {
			if (*begin < *min) *begin = *min;
			if (*begin > *max) *begin = *max;
			++begin;
			++min;
			++max;
		}
	}

	template <typename Type> inline static Type clampCycle(Type a, Type min, Type max)  {
		const Type range = max - min;
		return a < min ? a + range : a > max ? a - range : a;
	}

	template <typename Type> inline static Type clampPI(Type a)  {
		return a < -numeric_const<Type>::PI ? a + numeric_const<Type>::TWO_PI : a > +numeric_const<Type>::PI ? a - numeric_const<Type>::TWO_PI : a;
	}

	template <typename Type> inline static void expand(Type value, Type& a, Type& b)  {
		if (a <= b) {
			if (value < a) a = value;
			if (value > b) b = value;
		}
		else {
			if (value > a) a = value;
			if (value < b) b = value;
		}
	}

	template <typename Type> inline static Type guard(Type a, Type value, Type cutoff)  {
		return Math::sign(std::max(Math::abs(a - value), cutoff), a - value);
	}

	/** For a, b >= 0 perform cyclic addition a + b  */
	template <typename Type> inline static Type cycle_add(Type size, Type a, Type b) {
		return (a + b)%size;
	}
	/** For a, b >= 0 perform cyclic subtraction a - b  */
	template <typename Type> inline static Type cycle_sub(Type size, Type a, Type b) {
		return size - 1 - (size - a + b - 1)%size;
	}

	//template <typename Type> inline static Type abs(Type a) {
	//	return std::abs(a);
	//}
	inline static F32 abs(F32 a) {
		return ::fabsf(a);
	}
	inline static F64 abs(F64 a) {
		return ::fabs(a);
	}
	inline static I32 abs(I32 a) {
		return ::abs(a);
	}

	template <typename Type> inline static bool equals(Type a, Type b, Type eps) {
		return (Math::abs(a - b) < eps);
	}

	template <typename Type> inline static Type floor(Type a) {
		return ::floor(a);
	}

	template <typename Type> inline static Type floor(Type a, Type quant) {
		a /= quant;
		return quant * (a < numeric_const<Type>::ZERO ? Math::ceil(a) : Math::floor(a));
	}

	template <typename Type> inline static Type ceil(Type a) {
		return ::ceil(a);
	}

	template <typename Type> inline static Type ceil(Type a, Type quant) {
		a /= quant;
		return quant * (a < numeric_const<Type>::ZERO ? Math::floor(a) : Math::ceil(a));
	}

	template <typename Type> inline static Type round(Type a) {
		return Math::floor(a + numeric_const<Type>::HALF);
	}

	template <typename Type> inline static Type round(Type a, Type quant) {
		return quant * Math::floor(a/quant + numeric_const<Type>::HALF);
	}

	template <typename Type> inline static Type sqr(Type a) {
		return a*a;
	}

	template <typename Type> inline static Type sqrt(Type a) {
		return ::sqrt(a);
	}

	template <typename Type> inline static Type sin(Type a) {
		return ::sin(a);
	}

	template <typename Type> inline static Type asin(Type a) {
		return
			a >= +numeric_const<Type>::ONE ? +numeric_const<Type>::PI_2 :
			a <= -numeric_const<Type>::ONE ? -numeric_const<Type>::PI_2 : ::asin(a);
	}

	template <typename Type> inline static Type cos(Type a) {
		return ::cos(a);
	}

	template <typename Type> inline static Type acos(Type a) {
		return
			a >= +numeric_const<Type>::ONE ? numeric_const<Type>::ZERO :
			a <= -numeric_const<Type>::ONE ? numeric_const<Type>::PI : ::acos(a);
	}

	template <typename Type> inline static void sinCos(Type a, Type& s, Type& c) {
		s = ::sin(a);
		c = ::cos(a);
	}

	template <typename Type> inline static Type tan(Type a) {
		return ::tan(a);
	}

	template <typename Type> inline static Type atan(Type a) {
		return ::atan(a);
	}

	template <typename Type> inline static Type atan2(Type x, Type y) {
		return ::atan2(x,y);
	}

	template <typename Type> inline static Type pow(Type x, Type y) {
		return ::pow(x,y);
	}

	template <typename Type> inline static Type exp(Type a) {
		return ::exp(a);
	}

	template <typename Type> inline static Type ln(Type a) {
		return ::log(a);
	}

	template <typename Type> inline static Type log2(Type a) {
		return ::log(a) / numeric_const<Type>::LN2;
	}

	template <typename Type> inline static Type log10(Type a) {
		return ::log10(a);
	}

	template <typename Type> inline static bool isFinite(Type a) {
#ifdef WIN32
		return !((_FPCLASS_SNAN | _FPCLASS_QNAN | _FPCLASS_NINF | _FPCLASS_PINF) & ::_fpclass(a));
#else
		return std::isfinite(a);
#endif
	}

	template <typename Type> inline static bool isZero(Type a) {
		return a == numeric_const<Type>::ZERO;
	}

	template <typename Type> inline static bool isZeroEps(Type a) {
		return Math::abs(a) < numeric_const<Type>::EPS;
	}

	template <typename Type> inline static bool isPositive(Type a) {
		return a > numeric_const<Type>::ZERO;
	}

	template <typename Type> inline static bool isPositiveEps(Type a) {
		return a > +numeric_const<Type>::EPS;
	}

	template <typename Type> inline static bool isNegative(Type a) {
		return a < numeric_const<Type>::ZERO;
	}

	template <typename Type> inline static bool isNegativeEps(Type a) {
		return a < -numeric_const<Type>::EPS;
	}

	template <typename Type> inline static Type degToRad(Type a) {
		return (Type)(0.01745329251994329547) * a;
	}

	template <typename Type> inline static Type radToDeg(Type a) {
		return (Type)(57.29577951308232286465) * a;
	}

	template <typename Type> inline static Type normaliseAngle(Type a, Type min, Type max) {
		return a - (max - min) * Math::floor((a - min) * (numeric_const<Type>::ONE/(max - min)));
	}
	
	template <typename Type> inline static Type normaliseRad(Type a) {
		return Math::normaliseAngle(a, -numeric_const<Type>::PI, numeric_const<Type>::PI);
		//return a - numeric_const<Type>::TWO_PI*Math::floor(numeric_const<Type>::INV_2_PI * (numeric_const<Type>::PI + a));
	}
	
	template <typename Type> inline static Type diffRad(Type a, Type b) {
		const Type diff = Math::abs(normaliseRad(a) - normaliseRad(b));
		return diff > numeric_const<Type>::PI ? numeric_const<Type>::TWO_PI - diff : diff;
	}
	
	template <typename Type> inline static Type diffRadNormalised(Type a, Type b) {
		const Type diff = Math::abs(a - b);
		return diff > numeric_const<Type>::PI ? numeric_const<Type>::TWO_PI - diff : diff;
	}

	// if std::copy() complains too much
	template <typename Inp, typename Out> inline static void copy(Inp first, Inp last, Out result)  {
		while (first != last) *result++ = *first++;
	}

	template <typename Inp, typename Out> inline static void copy_ptr(Inp first, Inp last, Out result)  {
		while (first != last) *result++ = first++;
	}

	template <typename Inp, typename Out, typename Op> inline static void transform(Out first, Out last, Inp inp, const Op& op)  {
		while (first != last) *first++ = op(*inp++);
	}
	
	template <typename Inp1, typename Inp2, typename Out, typename Op> inline static void transform(Out first, Out last, Inp1 inp1, Inp2 inp2, const Op& op)  {
		while (first != last) *first++ = op(*inp1++, *inp2++);
	}

	template <typename Inp, typename Out, typename Op> inline static void for_each(Out first, Out last, Inp inp, Op op)  {
		while (first != last) op(*first++, *inp++);
	}

	template <typename Arg, typename Out, typename Op> inline static void for_each_const(Out first, Out last, const Arg& arg, const Op& op)  {
		while (first != last) op(*first++, arg);
	}

	template <typename Arg1, typename Arg2, typename Out, typename Op> inline static void for_each_const(Out first, Out last, const Arg1& arg1, const Arg2& arg2, const Op& op)  {
		while (first != last) op(*first++, arg1, arg2);
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_MATH_H_*/
