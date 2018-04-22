/** @file Vec2.h
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
#ifndef _GOLEM_MATH_VEC2_H_
#define _GOLEM_MATH_VEC2_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Math.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** 2 Element vector class.
*/
template <typename _Real> class _Vec2 {
public:
	/** Real */
	typedef _Real Real;

	/** vector components */
	union {
		struct {
			Real x, y;
		};
		struct {
			Real v1, v2;
		};
		Real v[2];
	};

	/** Default constructor does not do any initialisation.
	*/
	inline _Vec2() {}

	/**	Assigns scalar parameters to all elements.
	*	@param	a		Value to assign to elements.
	*/
	template <typename _Type> inline _Vec2(_Type a) : v1((Real)a), v2((Real)a) {}

	/** Initialises from 3 scalar parameters.
	*	@param	v1		Value to initialise v1 component.
	*	@param	v2		Value to initialise v1 component.
	*/
	template <typename _Type> inline _Vec2(_Type v1, _Type v2) : v1((Real)v1), v2((Real)v2) {}

	/**	Copy constructor.
	*	@param	v		Value to initialise vector.
	*/
	template <typename _Type> inline _Vec2(const _Vec2<_Type> &v) : v1((_Real)v.v1), v2((_Real)v.v2) {}

	/** Static initialisation member - zero.
	*/
	static inline _Vec2 zero() {
		return _Vec2(numeric_const<Real>::ZERO);
	}
	/** Static initialisation member - X axis.
	*/
	static inline _Vec2 axisX() {
		return _Vec2(numeric_const<Real>::ONE, numeric_const<Real>::ZERO);
	}
	/** Static initialisation member - Y axis.
	*/
	static inline _Vec2 axisY() {
		return _Vec2(numeric_const<Real>::ZERO, numeric_const<Real>::ONE);
	}

	/** returns true if the object is valid
	*/
	inline bool isValid() const {
		return isFinite();
	}

	/** the default configuration
	*/
	inline void setToDefault() {
		setZero();
	}

	/**	Access the data as an array.
	*	@return		Array of 3 floats.
	*/
	inline const Real *data() const {
		return v;
	}
	
	/**	Access the data as an array.
	*	@return		Array of 3 floats.
	*/
	inline Real* data() {
		return v;
	}

	/** Assignment
	*/
	template <typename _Type> inline void set(const _Vec2<_Type>& v) {
		v1 = (Real)v.v1;
		v2 = (Real)v.v2;
	}

	template <typename _Type> inline void set(_Type a) {
		v1 = (Real)a;
		v2 = (Real)a;
	}

	template <typename _Type> inline void set(_Type v1, _Type v2) {
		this->v1 = (Real)v1;
		this->v2 = (Real)v2;
	}
	
	/** Reads 2 values to dest.
	*	@param	v	Array to write elements to.
	*/
	template <typename _Type> inline void getColumn2(_Type* v) const {
		v[0] = (_Type)this->v1;
		v[1] = (_Type)this->v2;
	}

	/** Writes 2 consecutive values from the ptr passed
	*	@param	v	Array to read elements from.
	*/
	template <typename _Type> inline void setColumn2(const _Type* v) {
		v1 = (Real)v[0];
		v2 = (Real)v[1];
	}

	/** this = -a
	*/
	inline void setNegative(const _Vec2& v) {
		v1 = -v.v1;
		v2 = -v.v2;
	}
	/** this = -this
	*/
	inline void setNegative() {
		v1 = -v1;
		v2 = -v2;
	}

	inline void setZero() {
		v1 = numeric_const<Real>::ZERO;
		v2 = numeric_const<Real>::ZERO;
	}
	
	/** sets the vector's magnitude
	*/
	inline void setMagnitude(Real length) {
		Real m = magnitude();

		if (Math::abs(m) > numeric_const<Real>::EPS) {
			Real newLength = length / m;
			v1 *= newLength;
			v2 *= newLength;
		}
	}

	/** normalises the vector
	*/
	inline Real normalise() {
		Real m = magnitude();
		
		if (Math::abs(m) > numeric_const<Real>::EPS) {
			const Real length = numeric_const<Real>::ONE / m;
			v1 *= length;
			v2 *= length;
		}
		
		return m;
	}

	/** Generates uniform random direction
	*/
	template <typename Rand> void next(const Rand &rand) {
		const Real phi = numeric_const<Real>::TWO_PI * rand.template nextUniform<Real>();
		set(Math::cos(phi), Math::sin(phi));
	}

	/** this = element wise min(this,other)
	*/
	inline void min(const _Vec2& v) {
		if (v1 < v.v1) v1 = v.v1;
		if (v2 < v.v2) v2 = v.v2;
	}
	/** this = element wise max(this,other)
	*/
	inline void max(const _Vec2& v) {
		if (v1 > v.v1) v1 = v.v1;
		if (v2 > v.v2) v2 = v.v2;
	}

	/** this = a + b
	*/
	inline void add(const _Vec2& a, const _Vec2& b) {
		v1 = a.v1 + b.v1;
		v2 = a.v2 + b.v2;
	}

	/** this = a - b
	*/
	inline void subtract(const _Vec2& a, const _Vec2& b) {
		v1 = a.v1 - b.v1;
		v2 = a.v2 - b.v2;
	}

	/** this = s * a;
	*/
	inline void multiply(Real s, const _Vec2& a) {
		v1 = a.v1 * s;
		v2 = a.v2 * s;
	}

	/** this[i] = a[i] * b[i], for all i.
	*/
	inline void arrayMultiply(const _Vec2& a, const _Vec2& b) {
		v1 = a.v1 * b.v1;
		v2 = a.v2 * b.v2;
	}

	/** this = s * a + b;
	*/
	inline void multiplyAdd(Real s, const _Vec2& a, const _Vec2& b) {
		v1 = s * a.v1 + b.v1;
		v2 = s * a.v2 + b.v2;
	}

	/** this = s * a + t * b;
	*/
	inline void linear(Real s, const _Vec2& a, Real t, const _Vec2& b) {
		v1 = s * a.v1 + t * b.v1;
		v2 = s * a.v2 + t * b.v2;
	}

	/** returns the magnitude
	*/
	inline Real magnitude() const {
		return Math::sqrt(magnitudeSqr());
	}

	/** returns the squared magnitude
	*/
	inline Real magnitudeSqr() const {
		return v1 * v1 + v2 * v2;
	}

	/** returns (this - other).magnitude();
	*/
	inline Real distance(const _Vec2& v) const {
		return Math::sqrt(distanceSqr(v));
	}

	/** returns (this - other).magnitudeSqr();
	*/
	inline Real distanceSqr(const _Vec2& v) const {
		Real dv1 = v1 - v.v1;
		Real dv2 = v2 - v.v2;
		return dv1 * dv1 + dv2 * dv2;
	}

	/** returns the dot/scalar product of this and other.
	*/
	inline Real dot(const _Vec2& v) const {
		return v1 * v.v1 + v2 * v.v2;
	}

	/** tests for exact zero vector
	*/
	inline bool isZero() const {
		return Math::isZero(v1) && Math::isZero(v2);
	}

	/** tests for positive vector
	*/
	inline bool isPositive() const {
		return v1 > numeric_const<Real>::ZERO && v2 > numeric_const<Real>::ZERO;
	}

	/** tests for finite vector
	*/
	inline bool isFinite() const {
		return Math::isFinite(v1) && Math::isFinite(v2);
	}

	/** returns true if this and arg's elems are within epsilon of each other.
	*/
	inline bool equals(const _Vec2& v, Real epsilon) const {
		return
			Math::equals(v1, v.v1, epsilon) &&
			Math::equals(v2, v.v2, epsilon);
	}

	/**	Assignment operator.
	*/
	template <typename _Type> inline const _Vec2& operator = (const _Vec2<_Type>& v) {
		set(v);
		return *this;
	}

	/** Access the data as an array.
	*	@param	idx	Array index.
	*	@return		Array element pointed by idx.
	*/
	inline Real& operator [] (size_t idx) {
		return (&v1)[idx];
	}
	inline const Real& operator [] (size_t idx) const {
		return (&v1)[idx];
	}
	
	/** true if all the members are smaller.
	*/
	inline bool operator < (const _Vec2& v) const {
		return (v1 < v.v1) && (v2 < v.v2);
	}

	/** true if all the members are larger.
	*/
	inline bool operator > (const _Vec2& v) const {
		return (v1 > v.v1) && (v2 > v.v2);
	}

	/** returns true if the two vectors are exactly equal.
	*/
	inline bool operator == (const _Vec2& v) const {
		return (v1 == v.v1) && (v2 == v.v2);
	}

	/** returns true if the two vectors are exactly unequal.
	*/
	inline bool operator != (const _Vec2& v) const {
		return (v1 != v.v1) || (v2 != v.v2);
	}

	/** negation
	*/
	_Vec2 operator - () const {
		return _Vec2(-v1, -v2);
	}
	/** vector addition
	*/
	_Vec2 operator + (const _Vec2 & v) const {
		return _Vec2(v1 + v.v1, v2 + v.v2);
	}
	/** vector difference
	*/
	_Vec2 operator - (const _Vec2 & v) const {
		return _Vec2(v1 - v.v1, v2 - v.v2);
	}
	/** scalar post-multiplication
	*/
	_Vec2 operator * (Real f) const {
		return _Vec2(v1 * f, v2 * f);
	}
	/** scalar division
	*/
	_Vec2 operator / (Real f) const {
		f = Real(1.0) / f;
		return _Vec2(v1 * f, v2 * f);
	}
	/** vector addition
	*/
	_Vec2& operator += (const _Vec2& v) {
		v1 += v.v1;
		v2 += v.v2;
		return *this;
	}
	/** vector difference
	*/
	_Vec2& operator -= (const _Vec2& v) {
		v1 -= v.v1;
		v2 -= v.v2;
		return *this;
	}
	/** scalar multiplication
	*/
	_Vec2& operator *= (Real f) {
		v1 *= f;
		v2 *= f;
		return *this;
	}
	/** scalar division
	*/
	_Vec2& operator /= (Real f) {
		f = Real(1.0) / f;
		v1 *= f;
		v2 *= f;
		return *this;
	}
	/** dot product
	*/
	Real operator | (const _Vec2& v) const {
		return v1 * v.v1 + v2 * v.v2;
	}
};

//------------------------------------------------------------------------------

/** Default type */
typedef _Vec2<Real> Vec2;

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_VEC2_H_*/
