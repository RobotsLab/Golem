/** @file VecN.h
 * 
 * Multi-dimensional real-valued vector with variable size and mask operations.
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
#ifndef _GOLEM_MATH_VECN_H_
#define _GOLEM_MATH_VECN_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Math.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Default zero flag for mask operations */
struct ZeroFlag {
	template <typename Type> static inline bool is(Type val) {
		return val == numeric_const<Type>::ZERO; // by default test for zero
	}
};

/** Default non-Zero flag for mask operations */
struct NZeroFlag {
	template <typename Type> static inline bool is(Type val) {
		return val != numeric_const<Type>::ZERO; // by default test for zero
	}
};

/** Multi-dimensional real-valued vector with variable size (up to _N). */
template <typename _Real, size_t _N> class _VecN {
public:
	/** Real */
	typedef _Real Real;

	/** Maximum vector size */
	inline static size_t N() {
		return _N;
	}

	/** vector components */
	_Real v[_N];

	/** Default constructor sets size to 0 */
	inline _VecN() {
		n = 0;
	}
	/** Sets vector size without initialisation */
	inline _VecN(size_t n) {
		resize(n);
	}
	/** Resizes vector and initialises with given value */
	inline _VecN(size_t n, _Real value) {
		assign(n, value);
	}
	/** Copying constructor */
	inline _VecN(const _VecN& vec) {
		set(vec);
	}
	/** Resizes vector and initialises with given value */
	template <typename Ptr> inline _VecN(Ptr begin, Ptr end) {
		assign(begin, end);
	}

	/** Resizes vector */
	inline void resize(size_t n) {
		this->n = std::min(n, _N);
	}
	
	/** Vector size */
	inline size_t size() const {
		return n;
	}

	/** Data access */
	inline _Real* data() {
		return v;
	}
	/** Data access */
	inline const _Real* data() const {
		return v;
	}

	/** Pointer */
	inline _Real* begin() {
		return v;
	}
	/** Pointer */
	inline const _Real* begin() const {
		return v;
	}

	/** Pointer */
	inline _Real* end() {
		return v + n;
	}
	/** Pointer */
	inline const _Real* end() const {
		return v + n;
	}

	/** Initialises data with given value */
	inline void fill(_Real value) {
		for (size_t i = 0; i < n; ++i)
			v[i] = value;
	}

	/** Resizes vector and initialises data with given value */
	inline void assign(size_t n, _Real value) {
		resize(n);
		fill(value);
	}

	/** Resizes vector and initialises data with given value */
	template <typename Ptr> inline void assign(Ptr begin, Ptr end) {
		resize(size_t(end - begin));
		for (size_t i = 0; i < n; ++i)
			v[i] = _Real(*begin++);
	}

	/** Sets vector */
	inline void set(const _VecN& vec) {
		n = vec.n;
		for (size_t i = 0; i < vec.n; ++i)
			v[i] = vec[i];
	}
	
	/** Sets zero without resizing */
	inline void setZero() {
		fill(numeric_const<_Real>::ZERO);
	}
	
	/** Sets negative of the specified vector */
	inline void setNegative(const _VecN& vec) {
		n = vec.n;
		for (size_t i = 0; i < n; ++i)
			v[i] = -vec[i];
	}

	/** Sets inverse of the specified vector */
	inline void setInverse(const _VecN& vec) {
		n = vec.n;
		for (size_t i = 0; i < n; ++i) {
			const _Real c = vec[i];
			v[i] = c != numeric_const<_Real>::ZERO ? numeric_const<_Real>::ONE/c : numeric_const<_Real>::ZERO;
		}
	}

	/** Sets absolute of the specified vector */
	inline void setAbs(const _VecN& vec) {
		n = vec.n;
		for (size_t i = 0; i < n; ++i)
			v[i] = Math::abs(vec[i]);
	}

	/** sets the vector's magnitude
	*/
	inline void setMagnitude(_Real length) {
		const _Real m = magnitude();

		if (Math::abs(m) > numeric_const<_Real>::ZERO) {
			const _Real newLength = length / m;
			for (size_t i = 0; i < n; ++i)
				v[i] *= newLength;
		}
	}

	/** normalises the vector
	*/
	inline _Real normalise() {
		const _Real m = magnitude();
		
		if (Math::abs(m) > numeric_const<_Real>::ZERO) {
			const _Real length = numeric_const<_Real>::ONE / m;
			for (size_t i = 0; i < n; ++i)
				v[i] *= length;
		}
		
		return m;
	}

	/** this = element wise min(this,other) */
	inline void min(const _VecN& vec) {
		for (size_t i = 0; i < n; ++i)
			if (v[i] > vec[i])
				v[i] = vec[i];
	}
	/** this = element wise max(this,other)
	*/
	inline void max(const _VecN& vec) {
		for (size_t i = 0; i < n; ++i)
			if (v[i] < vec[i])
				v[i] = vec[i];
	}

	/** this = a + b
	*/
	inline void add(const _VecN& a, const _VecN& b) {
		for (size_t i = 0; i < n; ++i)
			v[i] = a[i] + b[i];
	}

	/** this = a - b
	*/
	inline void subtract(const _VecN& a, const _VecN& b) {
		for (size_t i = 0; i < n; ++i)
			v[i] = a[i] - b[i];
	}

	/** this = s * a;
	*/
	inline void multiply(_Real s, const _VecN& a) {
		for (size_t i = 0; i < n; ++i)
			v[i] = s*a[i];
	}

	/** this[i] = a[i] * b[i], for all i.
	*/
	inline void arrayMultiply(const _VecN& a, const _VecN& b) {
		for (size_t i = 0; i < n; ++i)
			v[i] = a[i]*b[i];
	}

	/** this = s * a + b;
	*/
	inline void multiplyAdd(_Real s, const _VecN& a, const _VecN& b) {
		for (size_t i = 0; i < n; ++i)
			v[i] = s*a[i] + b[i];
	}

	/** this = s * a + t * b;
	*/
	inline void linear(_Real s, const _VecN& a, _Real t, const _VecN& b) {
		for (size_t i = 0; i < n; ++i)
			v[i] = s*a[i] + t*b[i];
	}

	/** this = a + s * (b - a);
	*/
	inline void interpolate(const _VecN& a, const _VecN& b, _Real s) {
		for (size_t i = 0; i < n; ++i)
			v[i] = a[i] + s*(b[i] - a[i]);
	}

	/** returns the magnitude
	*/
	inline _Real magnitude() const {
		return Math::sqrt(magnitudeSqr());
	}

	/** returns the squared magnitude
	*/
	inline _Real magnitudeSqr() const {
		_Real s = numeric_const<_Real>::ZERO;
		for (size_t i = 0; i < n; ++i)
			s += Math::sqr(v[i]);
		return s;
	}

	/** returns (this - other).magnitude();
	*/
	inline _Real distance(const _VecN& vec) const {
		return Math::sqrt(distanceSqr(vec));
	}

	/** returns weighted distance (this - other).magnitude();
	*/
	inline _Real distanceWeight(const _VecN& weight, const _VecN& vec) const {
		return Math::sqrt(distanceWeightSqr(weight, vec));
	}

	/** returns (this - other).magnitudeSqr();
	*/
	inline _Real distanceSqr(const _VecN& vec) const {
		_Real s = numeric_const<_Real>::ZERO;
		for (size_t i = 0; i < n; ++i)
			s += Math::sqr(v[i] - vec[i]);
		return s;
	}

	/** returns weighted distance (this - other).magnitudeSqr();
	*/
	inline _Real distanceWeightSqr(const _VecN& weight, const _VecN& vec) const {
		_Real s = numeric_const<_Real>::ZERO;
		for (size_t i = 0; i < n; ++i)
			s += weight[i]*Math::sqr(v[i] - vec[i]);
		return s;
	}

	/** returns the dot/scalar product of this and other.
	*/
	inline _Real dot(const _VecN& vec) const {
		_Real s = numeric_const<_Real>::ZERO;
		for (size_t i = 0; i < n; ++i)
			s += v[i]*vec[i];
		return s;
	}

	/** tests for exact zero vector
	*/
	inline bool isZero() const {
		for (size_t i = 0; i < n; ++i)
			if (!Math::isZero(v[i]))
				return false;
		return true;
	}

	/** tests for positive vector
	*/
	inline bool isPositive() const {
		for (size_t i = 0; i < n; ++i)
			if (v[i] < numeric_const<_Real>::ZERO)
				return false;
		return true;
	}

	/** tests for finite vector
	*/
	inline bool isFinite() const {
		for (size_t i = 0; i < n; ++i)
			if (!Math::isFinite(v[i]))
				return false;
		return true;
	}

	/** returns true if this and arg's elems are within epsilon of each other.
	*/
	inline bool equals(const _VecN& vec, _Real epsilon) const {
		for (size_t i = 0; i < n; ++i)
			if (!Math::equals(v[i], vec[i], epsilon))
				return false;
		return true;
	}
	
	/**	Assignment operator. */
	inline _VecN& operator = (const _VecN &vec) {
		set(vec);
		return *this;
	}
	/** Access vector as an array. */
	inline _Real& operator [] (size_t idx) {
		return v[idx];
	}
	/** Access vector as an array. */
	inline const _Real& operator [] (size_t idx) const {
		return v[idx];
	}

protected:
	/** Vector size */
	size_t n;
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_VECN_H_*/
