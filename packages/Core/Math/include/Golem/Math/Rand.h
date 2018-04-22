/** @file Rand.h
 * 
 * Pseudo-random generator library.
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
#ifndef _GOLEM_MATH_RAND_H_
#define _GOLEM_MATH_RAND_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Math.h>
#include <time.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Random variable */
template <typename Type>
class Variable {
private:
	U32 n;
	Type val, min, max, mean, M2;
	
public:
	Variable() {
		reset();
	}

	template <typename Ptr> Variable(Ptr begin, Ptr end) {
		set(begin, end);
	}

	template <typename Ptr> void set(Ptr begin, Ptr end) {
		reset();
		while (begin != end) update(*begin++);
	}

	void reset() {
		n = 0;
		val = mean = M2 = numeric_const<Type>::ZERO;
		min = numeric_const<Type>::MAX;
		max = numeric_const<Type>::MIN;
	}

	void update(Type val) {
		++n;

		this->val = val;
		if (val < min)
			min = val;
		if (val > max)
			max = val;

		// Donald E. Knuth (1998). The Art of Computer Programming, volume 2: Seminumerical Algorithms, 3rd edn., p. 232. Boston: Addison-Wesley.
		const Type delta = val - mean;
		mean = mean + delta/n;
		M2 = M2 + delta*(val - mean);
	}

	Type get() const {
		return val;
	}
	U32 getN() const {
		return n;
	}
	Type getMin() const {
		return min;
	}
	Type getMax() const {
		return max;
	}

	Type getMean() const {
		return mean;
	}
	Type getVariance() const {
		return M2/(n - 1);
	}
	Type getVarianceN() const {
		return M2/n;
	}
};

//------------------------------------------------------------------------------

template <int MM, int AA, int QQ, int RR, int SS>
class TI32Hash {
public:
	static inline I32 generate(I32 val) {
		val ^= SS;
		val = (U32)AA*(val%QQ) - (U32)RR*(val/QQ);
		if (val < 0) val += MM;
		return val;
	}
};

template <int MM, int AA, int QQ, int RR, int SS, int MMM, int AAA, int QQQ, int RRR, int SSS>
class TI64Hash {
public:
	static inline I64 generate(I32 lo, I32 hi) {
		lo ^= SS ^ hi;
		lo = (U32)AA*(lo%QQ) - (U32)RR*(lo/QQ);
		if (lo < 0) lo += MM;
		hi ^= SSS ^ lo;
		hi = (U32)AAA*(hi%QQQ) - (U32)RRR*(hi/QQQ);
		if (hi < 0) hi += MMM;
		return (I64)lo + ((I64)hi << 32);
	}

	static inline I64 generate(I64 val) {
		return generate((I32)(val), (I32)(val >> 32));
	}
};

typedef TI32Hash<2147483647, 48271, 44488, 3399, 0xdeadbeef> I32Hash;
typedef TI64Hash<2147483647, 48271, 44488, 3399, 0xdeadbeef, 2147483399, 40692, 52774, 3791, 0xcafefeed> I64Hash;

//------------------------------------------------------------------------------

/** 32 bit cyclic redundancy check */
class CRC32 {
private:
	static const U32 table[256];

public:
	typedef U32 Type;
	
	/** Initialise crc  */
	static void init(U32& crc) {
		crc = 0xFFFFFFFFUL;
	}
	
	/** Process crc  */
	static void process(U32& crc, const U8* data, size_t len) {
		while (len-- > 0)
			crc = table[(crc ^ (*data++)) & 0xFF] ^ (crc >> 8);
	}
	
	/** Finish crc  */
	static void finish(U32& crc) {
		crc ^= 0xFFFFFFFFUL;
	}
};

/** 64 bit cyclic redundancy check */
class CRC64 {
private:
	static const U64 table[256];

public:
	typedef U64 Type;

	/** Initialise crc  */
	static void init(U64& crc) {
		crc = 0xFFFFFFFFFFFFFFFFULL;
	}
	
	/** Process crc  */
	static void process(U64& crc, const U8* data, size_t len) {
		while (len-- > 0)
			crc = table[((U32)(crc >> 56) ^ (*data++)) & 0xFF] ^ (crc << 8);
	}
	
	/** Finish crc  */
	static void finish(U64& crc) {
		crc ^= 0xFFFFFFFFFFFFFFFFULL;
	}
};

//------------------------------------------------------------------------------

/** Seed of random generators.
*/
struct RandSeed {
	union {
		U64 _U64[1];
		U32 _U32[2];
		U16 _U16[4];
		U8 _U8[8];
	};

	inline RandSeed() {
		generate();
	}
	inline RandSeed(const RandSeed &randSeed) {
		_U64[0] = randSeed._U64[0];
	}
	inline RandSeed(U64 _U64_0) {
		_U64[0] = _U64_0;
	}
	inline RandSeed(U32 _U32_0, U32 _U32_1) {
		_U32[0] = _U32_0;
		_U32[1] = _U32_1;
	}
	inline RandSeed(U16 _U16_0, U16 _U16_1, U16 _U16_2, U16 _U16_3) {
		_U16[0] = _U16_0;
		_U16[1] = _U16_1;
		_U16[2] = _U16_2;
		_U16[3] = _U16_3;
	}
	inline RandSeed(U8 _U8_0, U8 _U8_1, U8 _U8_2, U8 _U8_3, U8 _U8_4, U8 _U8_5, U8 _U8_6, U8 _U8_7) {
		_U8[0] = _U8_0;
		_U8[1] = _U8_1;
		_U8[2] = _U8_2;
		_U8[3] = _U8_3;
		_U8[4] = _U8_4;
		_U8[5] = _U8_5;
		_U8[6] = _U8_6;
		_U8[7] = _U8_7;
	}

	inline void generate() {
		_U64[0] = I64Hash::generate((U64)::time(NULL)); // number of seconds elapsed since 00:00, Jan 1, 1970 UTC
	}
};

//------------------------------------------------------------------------------

/** Random number generator base
*/
template <typename _Type>
class TRandBase {
public:
	typedef _Type Type;
};

//------------------------------------------------------------------------------

/** Random number generator version 1.
* @see "Seminumerical Algorithms", Donald E. Knuth, 1998, ed. 3, Addison-Wesley
*/
template <int M, int A>
class TI32Rand1 : public TRandBase<I32> {
private:
	mutable I32 x;

public:
	/** Creates random generator and initialises from default RandSeed */
	inline TI32Rand1() : x(I32(A)) {
	}
	/** Creates random generator and initialises from given RandSeed */
	inline TI32Rand1(const RandSeed &randSeed) {
		setRandSeed(randSeed);
	}
	/** Initialises random generator from given RandSeed */
	inline void setRandSeed(const RandSeed &randSeed) {
		x = (I32)randSeed._U32[0];
	}
	/** Generates a pseudo random integer number <0, I32> */
	inline I32 next() const {
		return (x = A*x + M) & 0x7FFFFFFF;
	}
};

/** Random number generator version 2.
* @see "Seminumerical Algorithms", Donald E. Knuth, 1998, ed. 3, Addison-Wesley
*/
template <int MM, int AA, int QQ, int RR>
class TI32Rand2 : public TRandBase<I32> {
private:
	mutable I32 x;

public:
	/** Creates random generator and initialises from default RandSeed */
	inline TI32Rand2() : x(I32(AA)) {
	}
	/** Creates random generator and initialises from given RandSeed */
	inline TI32Rand2(const RandSeed &randSeed) {
		setRandSeed(randSeed);
	}
	/** Initialises random generator from given RandSeed */
	inline void setRandSeed(const RandSeed &randSeed) {
		x = (I32)randSeed._U32[0];
	}
	/** Generates a pseudo random integer number <0, I32> */
	inline I32 next() const {
		x = (U32)AA*(x%QQ) - (U32)RR*(x/QQ);
		if (x < 0) x += MM;
		return x;
	}
};

/** Random number generator version 3.
* @see "Seminumerical Algorithms", Donald E. Knuth, 1998, ed. 3, Addison-Wesley
*/
template <int MM, int AA, int QQ, int RR, int MMM, int AAA, int QQQ, int RRR>
class TI32Rand3 : public TRandBase<I32> {
private:
	mutable I32 x, y;

public:
	/** Creates random generator and initialises from default RandSeed */
	inline TI32Rand3() : x(I32(AA)), y(I32(AAA)) {
	}
	/** Creates random generator and initialises from given RandSeed */
	inline TI32Rand3(const RandSeed &randSeed) {
		setRandSeed(randSeed);
	}
	/** Initialises random generator from given RandSeed */
	inline void setRandSeed(const RandSeed &randSeed) {
		x = (I32)randSeed._U32[0];
		y = (I32)randSeed._U32[1];
	}
	/** Generates a pseudo random integer number <0, I32> */
	inline I32 next() const {
		x = (U32)AA*(x%QQ) - (U32)RR*(x/QQ);
		if (x < 0) x += MM;
		y = (U32)AAA*(y%QQQ) - (U32)RRR*(y/QQQ);
		if (y < 0) y += MMM;
		I32 z = x - y;
		if (z < 0) z += MM;
		return z;
	}
};

typedef TI32Rand1<2147483647, 65539> I32Rand1;
typedef TI32Rand2<2147483647, 48271, 44488, 3399> I32Rand2;
typedef TI32Rand3<2147483647, 48271, 44488, 3399, 2147483399, 40692, 52774, 3791> I32Rand3;

//------------------------------------------------------------------------------

/** Floating-point pseudo random number generator
*/
template <typename Rand> class TRand : public Rand {
public:
	/** Creates random generator and initialises from default RandSeed */
	inline TRand() {
	}
	/** Creates random generator and initialises from given RandSeed */
	inline TRand(const RandSeed &randSeed) : Rand(randSeed) {
	}
	/** Generates a pseudo random integer number <0, max) */
	inline typename Rand::Type operator () (typename Rand::Type max) const {
		return Rand::next()%max;
	}
	/** Uniform distribution in range <0, 1) */
	template <typename Type> inline Type nextUniform() const {
		return Type(Rand::next())/(numeric_const<typename Rand::Type>::MAX + numeric_const<Type>::ONE);
	}
	/** Uniform distribution in range <min, max) */
	template <typename Type> inline Type nextUniform(Type min, Type max) const {
		return nextUniform<Type>()*(max - min) + min;
	}
	/**  Uses Marsaglia polar method (improved Box & Muller transformation).
	* @see "Seminumerical Algorithms", Donald E. Knuth, 1998, ed. 3, Addison-Wesley
	*/
	template <typename Type, typename Pointer> inline void nextNVariable(Pointer begin, Pointer end) const {
		// generate random N-dimensional point contained in a N-dimensional ball of radius 1
		// this rejection sampling becomes inefficient for large N:
		// probability of rejection is proportional to 1 - V_N/2^N where V_N is the volume of N-dimensional ball of radius 1
		// V_N = 2*pi*R^2/N * V_(N-2), V_1 = 1; V_2 = 2*R and R = 1;
		Type s;
		do {
			s = numeric_const<Type>::ZERO;
			for (Pointer i = begin; i != end; ++i)
				s += Math::sqr(*i = nextUniform<Type>()*numeric_const<Type>::TWO - numeric_const<Type>::ONE);
		} while (s > numeric_const<Type>::ONE - numeric_const<Type>::EPS || s < numeric_const<Type>::EPS);
		
		const Type m = Math::sqrt(-numeric_const<Type>::TWO * Math::ln(s)/s);
		for (Pointer i = begin; i != end; ++i)
			*i *= m;
	}
	/** Univariate Gaussian distribution given mean and standard deviation.
	* TODO use second Gaussian
	*/
	template <typename Type> inline Type nextGaussian(Type mean = numeric_const<Type>::ZERO, Type stddev = numeric_const<Type>::ONE) const {
		Type density[2];
		nextGaussianArray<Type, Type*>(density, density + 2);
		return mean + stddev*density[0];
	}
	/** Multivariate Gaussian distribution with 0 mean and unit standard deviation.
	*/
	template <typename Type, typename Pointer> inline void nextGaussianArray(Pointer begin, Pointer end) const {
		for (;;) {
			if (begin == end) break;
			Type density[2];
			nextNVariable<Type, Type*>(density, density + 2);
			*begin++ = density[0];
			if (begin == end) break;
			*begin++ = density[1];
		}
	}
	/** Multivariate Gaussian distribution given mean and standard deviation.
	*/
	template <typename Type, typename Pointer, typename ConstPointer> inline void nextGaussianArray(Pointer begin, Pointer end, ConstPointer mean, ConstPointer stddev) const {
		for (;;) {
			if (begin == end) break;
			Type density[2];
			nextNVariable<Type, Type*>(density, density + 2);
			*begin++ = density[0]**stddev++ + *mean++;
			if (begin == end) break;
			*begin++ = density[1]**stddev++ + *mean++;
		}
	}
	/** Multivariate Gaussian distribution given mean and standard deviation.
	*/
	template <typename Type, typename Pointer> inline void nextGaussianArray(Pointer begin, Pointer end, Type mean, Type stddev) const {
		for (;;) {
			if (begin == end) break;
			Type density[2];
			nextNVariable<Type, Type*>(density, density + 2);
			*begin++ = density[0]*stddev + mean;
			if (begin == end) break;
			*begin++ = density[1]*stddev + mean;
		}
	}
};

//------------------------------------------------------------------------------

/** Default random number generators */
typedef TRand<I32Rand2> Rand;

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_RAND_H_*/
