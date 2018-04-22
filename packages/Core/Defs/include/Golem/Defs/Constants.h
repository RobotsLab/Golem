/** @file Constants.h
 * 
 * Numeric constants
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
#ifndef _GOLEM_DEFS_CONSTANTS_H_
#define _GOLEM_DEFS_CONSTANTS_H_

#include <Golem/Defs/Types.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Numeric constants base class */
template <typename _Type> class numeric_const_base {
public:
	/** Type */
	typedef _Type Type;
};

/** Numeric constants main class */
template <typename Type> class numeric_const : public numeric_const_base<Type> {
public:
};

//------------------------------------------------------------------------------

/** Numeric constants bool specialization */
template <> class numeric_const<bool> : public numeric_const_base<bool> {
public:
	/** Bool zero */
	static const bool ZERO = false;
	/** Bool one */
	static const bool ONE = true;
};

//------------------------------------------------------------------------------

/** Numeric constants base class for integer numbers */
template <typename Type> class numeric_const_int_base : public numeric_const_base<Type> {
public:
	/** Integer zero */
	static const Type ZERO;
	/** Integer one */
	static const Type ONE;
};
template <typename Type> const Type numeric_const_int_base<Type>::ZERO = 0;
template <typename Type> const Type numeric_const_int_base<Type>::ONE = 1;


/** Numeric constants U8 specialization */
template <> class numeric_const<U8> : public numeric_const_int_base<U8> {
public:
	/** U8 minimum */
	static const U8 MIN = 0x00;
	/** U8 maximum */
	static const U8 MAX = 0xff;
};

/** Numeric constants I8 specialization */
template <> class numeric_const<I8> : public numeric_const_int_base<I8> {
public:
	/** I8 minimum */
	static const I8 MIN = I8(0x80);
	/** I8 maximum */
	static const I8 MAX = 0x7f;
};


/** Numeric constants U16 specialization */
template <> class numeric_const<U16> : public numeric_const_int_base<U16> {
public:
	/** U16 minimum */
	static const U16 MIN = 0x0000;
	/** U16 maximum */
	static const U16 MAX = 0xffff;
};

/** Numeric constants I16 specialization */
template <> class numeric_const<I16> : public numeric_const_int_base<I16> {
public:
	/** I16 minimum */
	static const I16 MIN = I16(0x8000);
	/** I16 maximum */
	static const I16 MAX = 0x7fff;
};


/** Numeric constants U32 specialization */
template <> class numeric_const<U32> : public numeric_const_int_base<U32> {
public:
	/** U32 minimum */
	static const U32 MIN = 0x00000000;
	/** U32 maximum */
	static const U32 MAX = 0xffffffff;
};

/** Numeric constants I32 specialization */
template <> class numeric_const<I32> : public numeric_const_int_base<I32> {
public:
	/** I32 minimum */
	static const I32 MIN = I32(0x80000000);
	/** I32 maximum */
	static const I32 MAX = 0x7fffffff;
};


/** Numeric constants U64 specialization */
template <> class numeric_const<U64> : public numeric_const_int_base<U64> {
public:
	/** U64 minimum */
	static const U64 MIN = 0x0000000000000000ULL;
	/** U64 maximum */
	static const U64 MAX = 0xffffffffffffffffULL;
};

/** Numeric constants I64 specialization */
template <> class numeric_const<I64> : public numeric_const_int_base<I64> {
public:
	/** I64 minimum */
	static const I64 MIN = I64(0x8000000000000000ULL);
	/** I64 maximum */
	static const I64 MAX = 0x7fffffffffffffffULL;
};

//------------------------------------------------------------------------------

/** Numeric constants base class for floating-point numbers */
template <typename Type> class numeric_const_flt_base : public numeric_const_base<Type> {
public:
	/** Floating-point positive zero: +0.0 = 0x0... HEX */
	static const Type POS_ZERO;
	/** Floating-point negative zero: -0.0 = 0x8... HEX */
	static const Type NEG_ZERO;
	/** Floating-point zero */
	static const Type ZERO;
	/** Floating-point one */
	static const Type ONE;
	/** Floating-point two */
	static const Type TWO;
	/** Floating-point half */
	static const Type HALF;
	/** Floating-point pi */
	static const Type PI;
	/** Floating-point pi/2 */
	static const Type PI_2;
	/** Floating-point 2*pi */
	static const Type TWO_PI;
	/** Floating-point 1/pi */
	static const Type INV_PI;
	/** Floating-point 1/(2*pi) */
	static const Type INV_2_PI;
	/** Floating-point e */
	static const Type E;
	/** Floating-point ln(2) */
	static const Type LN2;
	/** Floating-point sqrt(2) */
	static const Type SQRT_2;
	/** Floating-point Golden Ratio */
	static const Type GOLD1;
	/** Floating-point Golden Ratio */
	static const Type GOLD2;
};
template <typename Type> const Type numeric_const_flt_base<Type>::POS_ZERO		= Type(+0.0);
template <typename Type> const Type numeric_const_flt_base<Type>::NEG_ZERO		= Type(-0.0);
template <typename Type> const Type numeric_const_flt_base<Type>::ZERO			= Type(+0.0);
template <typename Type> const Type numeric_const_flt_base<Type>::ONE			= Type(1.0);
template <typename Type> const Type numeric_const_flt_base<Type>::TWO			= Type(2.0);
template <typename Type> const Type numeric_const_flt_base<Type>::HALF			= Type(0.5);
template <typename Type> const Type numeric_const_flt_base<Type>::PI			= Type(3.14159265358979323846);
template <typename Type> const Type numeric_const_flt_base<Type>::PI_2			= Type(1.57079632679489661923);
template <typename Type> const Type numeric_const_flt_base<Type>::TWO_PI		= Type(6.28318530717958647692);
template <typename Type> const Type numeric_const_flt_base<Type>::INV_PI		= Type(0.31830988618379067154);
template <typename Type> const Type numeric_const_flt_base<Type>::INV_2_PI		= Type(0.15915494309189533577);
template <typename Type> const Type numeric_const_flt_base<Type>::E				= Type(2.71828182845904523536);
template <typename Type> const Type numeric_const_flt_base<Type>::LN2			= Type(0.69314718055994530942);
template <typename Type> const Type numeric_const_flt_base<Type>::SQRT_2		= Type(1.41421356237309504880);
template <typename Type> const Type numeric_const_flt_base<Type>::GOLD1			= Type(1.61803398874989484820);
template <typename Type> const Type numeric_const_flt_base<Type>::GOLD2			= Type(0.61803398874989484820);

/** Numeric constants F32 specialization */
template <> class numeric_const<F32> : public numeric_const_flt_base<F32> {
protected:
	union FConst {
		U32 u32;
		F32 f32;
	};
	/** F32 positive infinity */
	static const FConst IEEE_POS_INF;
	/** F32 negative infinity */
	static const FConst IEEE_NEG_INF;

public:
	/** F32 minimum */
	static const F32 MIN;
	/** F32 maximum */
	static const F32 MAX;
	/** F32 positive infinity */
	static const F32 POS_INF;
	/** F32 negative infinity */
	static const F32 NEG_INF;
	/** F32 infinity */
	static const F32 INF;
	/** Smallest such that 1.0+EPS != 1.0 */
	static const F32 EPS;
	/** max decimal exponent */
	static const F32 MAX_10_EXP;
	/** max binary exponent */
	static const F32 MAX_EXP;
	/** min decimal exponent */
	static const F32 MIN_10_EXP;
	/** min binary exponent */
	static const F32 MIN_EXP;
};


/** Numeric constants F64 specialization */
template <> class numeric_const<F64> : public numeric_const_flt_base<F64> {
protected:
	union FConst {
		U64 u64;
		F64 f64;
	};
	/** F64 positive infinity */
	static const FConst IEEE_POS_INF;
	/** F64 negative infinity */
	static const FConst IEEE_NEG_INF;

public:
	/** F64 minimum */
	static const F64 MIN;
	/** F64 maximum */
	static const F64 MAX;
	/** F64 positive infinity */
	static const F64 POS_INF;
	/** F64 negative infinity */
	static const F64 NEG_INF;
	/** F64 infinity */
	static const F64 INF;
	/** Smallest such that 1.0+EPS != 1.0 */
	static const F64 EPS;
	/** max decimal exponent */
	static const F64 MAX_10_EXP;
	/** max binary exponent */
	static const F64 MAX_EXP;
	/** min decimal exponent */
	static const F64 MIN_10_EXP;
	/** min binary exponent */
	static const F64 MIN_EXP;
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEFS_CONSTANTS_H_*/
