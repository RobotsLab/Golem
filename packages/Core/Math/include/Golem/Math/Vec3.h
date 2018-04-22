/** @file Vec3.h
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
#ifndef _GOLEM_MATH_VEC3_H_
#define _GOLEM_MATH_VEC3_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Vec2.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** 3 Element vector class.
*/
template <typename _Real> class _Vec3 {
public:
	/** Real */
	typedef _Real Real;

	/** vector components */
	union {
		struct {
			Real x, y, z;
		};
		struct {
			Real v1, v2, v3;
		};
		Real v[3];
	};

	/** Default constructor does not do any initialisation.
	*/
	inline _Vec3() {}

	/**	Assigns scalar parameters to all elements.
	*	@param	a		Value to assign to elements.
	*/
	template <typename _Type> inline _Vec3(_Type a) : v1((Real)a), v2((Real)a), v3((Real)a) {}

	/** Initialises from 3 scalar parameters.
	*	@param	v1		Value to initialise v1 component.
	*	@param	v2		Value to initialise v1 component.
	*	@param	v3		Value to initialise v1 component.
	*/
	template <typename _Type> inline _Vec3(_Type v1, _Type v2, _Type v3) : v1((Real)v1), v2((Real)v2), v3((Real)v3) {}

	/**	Copy constructor.
	*/
	template <typename _Type> inline _Vec3(const _Vec3<_Type> &v) : v1((Real)v.v1), v2((Real)v.v2), v3((Real)v.v3) {}

	/** Static initialisation member - zero.
	*/
	static inline _Vec3 zero() {
		return _Vec3(numeric_const<Real>::ZERO);
	}
	/** Static initialisation member - X.
	*/
	static inline _Vec3 axisX() {
		return _Vec3(numeric_const<Real>::ONE , numeric_const<Real>::ZERO, numeric_const<Real>::ZERO);
	}
	/** Static initialisation member - Y.
	*/
	static inline _Vec3 axisY() {
		return _Vec3(numeric_const<Real>::ZERO, numeric_const<Real>::ONE , numeric_const<Real>::ZERO);
	}
	/** Static initialisation member - Z.
	*/
	static inline _Vec3 axisZ() {
		return _Vec3(numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ONE );
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
	*/
	inline Real* data() {
		return v;
	}
	inline const Real *data() const {
		return v;
	}

	/** this = v
	*/
	template <typename _Type> inline void set(const _Vec3<_Type>& v) {
		v1 = (Real)v.v1;
		v2 = (Real)v.v2;
		v3 = (Real)v.v3;
	}

	template <typename _Type> inline void set(_Type a) {
		v1 = (Real)a;
		v2 = (Real)a;
		v3 = (Real)a;
	}

	template <typename _Type> inline void set(_Type v1, _Type v2, _Type v3) {
		this->v1 = (Real)v1;
		this->v2 = (Real)v2;
		this->v3 = (Real)v3;
	}
	
	/** Reads 3 values to dest.
	*	@param	v	Array to write elements to.
	*/
	template <typename _Type> inline void getColumn3(_Type* v) const {
		v[0] = (_Type)this->v1;
		v[1] = (_Type)this->v2;
		v[2] = (_Type)this->v3;
	}

	/** Writes 3 consecutive values from the ptr passed
	*	@param	v	Array to read elements from.
	*/
	template <typename _Type> inline void setColumn3(const _Type* v) {
		v1 = (Real)v[0];
		v2 = (Real)v[1];
		v3 = (Real)v[2];
	}
	
	/** this = -a
	*/
	inline void setNegative(const _Vec3& v) {
		v1 = -v.v1;
		v2 = -v.v2;
		v3 = -v.v3;
	}

	/** this = -this
	*/
	inline void  setNegative() {
		v1 = -v1;
		v2 = -v2;
		v3 = -v3;
	}

	/** this = 0
	*/
	inline void setZero() {
		v1 = numeric_const<Real>::ZERO;
		v2 = numeric_const<Real>::ZERO;
		v3 = numeric_const<Real>::ZERO;
	}
	
	/** sets the vector's magnitude
	*/
	inline void setMagnitude(Real length) {
		const Real m = magnitude();

		if (Math::abs(m) > numeric_const<Real>::ZERO) {
			const Real newLength = length / m;
			v1 *= newLength;
			v2 *= newLength;
			v3 *= newLength;
		}
	}

	/** normalises the vector
	*/
	inline Real normalise() {
		const Real m = magnitude();
		
		if (Math::abs(m) > numeric_const<Real>::ZERO) {
			const Real length = numeric_const<Real>::ONE / m;
			v1 *= length;
			v2 *= length;
			v3 *= length;
		}
		
		return m;
	}

	/** this = element wise min(this)
	*/
	inline Real min() const {
		Real val = v1;
		if (val > v2) val = v2;
		if (val > v3) val = v3;
		return val;
	}
	/** this = element wise min(this,other)
	*/
	inline void min(const _Vec3& v) {
		if (v1 > v.v1) v1 = v.v1;
		if (v2 > v.v2) v2 = v.v2;
		if (v3 > v.v3) v3 = v.v3;
	}
	/** element wise min(a, b)
	*/
	static inline _Vec3 min(const _Vec3& a, const _Vec3& b) {
		return _Vec3(std::min(a.v1, b.v1), std::min(a.v2, b.v2), std::min(a.v3, b.v3));
	}
	
	/** this = element wise max(this)
	*/
	inline Real max() const {
		Real val = v1;
		if (val < v2) val = v2;
		if (val < v3) val = v3;
		return val;
	}
	/** this = element wise max(this,other)
	*/
	inline void max(const _Vec3& v) {
		if (v1 < v.v1) v1 = v.v1;
		if (v2 < v.v2) v2 = v.v2;
		if (v3 < v.v3) v3 = v.v3;
	}
	/** element wise max(a, b)
	*/
	static inline _Vec3 max(const _Vec3& a, const _Vec3& b) {
		return _Vec3(std::max(a.v1, b.v1), std::max(a.v2, b.v2), std::max(a.v3, b.v3));
	}

	/** element wise abs()
	*/
	static inline _Vec3 abs(const _Vec3& v) {
		return _Vec3(Math::abs(v.v1), Math::abs(v.v2), Math::abs(v.v3));
	}

	/** this = element wise clamp(this,other)
	*/
	inline void clamp(const _Vec3& min, const _Vec3& max) {
		this->max(min);
		this->min(max);
	}
	/** element wise clamp(this,other)
	*/
	inline static _Vec3 clamp(const _Vec3& v, const _Vec3& min, const _Vec3& max) {
		_Vec3 tmp = v;
		tmp.clamp(min, max);
		return tmp;
	}

	/** this = a + b
	*/
	inline void add(const _Vec3& a, const _Vec3& b) {
		v1 = a.v1 + b.v1;
		v2 = a.v2 + b.v2;
		v3 = a.v3 + b.v3;
	}

	/** this = a - b
	*/
	inline void subtract(const _Vec3& a, const _Vec3& b) {
		v1 = a.v1 - b.v1;
		v2 = a.v2 - b.v2;
		v3 = a.v3 - b.v3;
	}

	/** this = s * a;
	*/
	inline void multiply(Real s, const _Vec3& a) {
		v1 = a.v1 * s;
		v2 = a.v2 * s;
		v3 = a.v3 * s;
	}

	/** this[i] = a[i] * b[i], for all i.
	*/
	inline void arrayMultiply(const _Vec3& a, const _Vec3& b) {
		v1 = a.v1 * b.v1;
		v2 = a.v2 * b.v2;
		v3 = a.v3 * b.v3;
	}

	/** this = s * a + b;
	*/
	inline void multiplyAdd(Real s, const _Vec3& a, const _Vec3& b) {
		v1 = s * a.v1 + b.v1;
		v2 = s * a.v2 + b.v2;
		v3 = s * a.v3 + b.v3;
	}

	/** this = s * a + t * b;
	*/
	inline void linear(Real s, const _Vec3& a, Real t, const _Vec3& b) {
		v1 = s * a.v1 + t * b.v1;
		v2 = s * a.v2 + t * b.v2;
		v3 = s * a.v3 + t * b.v3;
	}

	/** this = a + s * (b - a);
	*/
	inline void interpolate(const _Vec3& a, const _Vec3& b, Real s) {
		v1 = a.v1 + s * (b.v1 - a.v1);
		v2 = a.v2 + s * (b.v2 - a.v2);
		v3 = a.v3 + s * (b.v3 - a.v3);
	}

	/** returns the magnitude
	*/
	inline Real magnitude() const {
		return Math::sqrt(magnitudeSqr());
	}

	/** returns the squared magnitude
	*/
	inline Real magnitudeSqr() const {
		return v1 * v1 + v2 * v2 + v3 * v3;
	}

	/** returns (this - other).distance();
	*/
	inline Real distance(const _Vec3& v) const {
		return Math::sqrt(distanceSqr(v));
	}

	/** returns (this - other).distanceSqr();
	*/
	inline Real distanceSqr(const _Vec3& v) const {
		const Real dv1 = v1 - v.v1;
		const Real dv2 = v2 - v.v2;
		const Real dv3 = v3 - v.v3;

		return dv1 * dv1 + dv2 * dv2 + dv3 * dv3;
	}

	/** returns the dot/scalar product of this and other.
	*/
	inline Real dot(const _Vec3& v) const {
		return v1 * v.v1 + v2 * v.v2 + v3 * v.v3;
	}

	/** cross product, this = left v1 right
	*/
	inline void cross(const _Vec3& left, const _Vec3& right) {
		// temps needed in case left or right is this.
		const Real a = (left.v2 * right.v3) - (left.v3 * right.v2);
		const Real b = (left.v3 * right.v1) - (left.v1 * right.v3);
		const Real c = (left.v1 * right.v2) - (left.v2 * right.v1);

		v1 = a;
		v2 = b;
		v3 = c;
	}
	/** cross product
	*/
	_Vec3 cross(const _Vec3& v) const {
		return _Vec3(v2*v.v3 - v3*v.v2, v3*v.v1 - v1*v.v3, v1*v.v2 - v2*v.v1);
	}

	/** Generates uniform random direction
	*/
	template <typename Rand> void next(const Rand &rand) {
		const Real phi = numeric_const<Real>::TWO_PI * rand.template nextUniform<Real>();
		const Real cos = numeric_const<Real>::TWO*rand.template nextUniform<Real>() - numeric_const<Real>::ONE;
		const Real sin = Math::sqrt(numeric_const<Real>::ONE - cos*cos);
		set(cos, sin * Math::cos(phi), sin * Math::sin(phi));
	}
	/** Generates random direction around (0, 0, 1) on 3-sphere with dispersion parameter k (1/k is analogous to sigma^2),
	*	according to Kent distribution.
	*	For k = 0 the distribution is uniform, for k >> 0 the distribution is concentrated around (0, 0, 1).
	*/
	template <typename Rand> void next(const Rand &rand, const Real k) {
		//const Real b = Real(0.66666666666666666667)*(Math::sqrt(Math::sqr(k) + Real(2.25)) - k); // DIM=4, 2/3*(sqrt(k^2 + 9/4) - k)
		const Real b = Math::sqrt(Math::sqr(k) + numeric_const<Real>::ONE) - k; // DIM=3, sqrt(k^2 + 1) - k
		const Real x = (numeric_const<Real>::ONE - b) / (numeric_const<Real>::ONE + b);
		//const Real c = k*x + Real(3.0)*Math::ln(numeric_const<Real>::ONE - Math::sqr(x)); // DIM=4 
		const Real c = k*x + Real(2.0)*Math::ln(numeric_const<Real>::ONE - Math::sqr(x)); // DIM=3

		for (;;) {
			// beta distribution with a = b = (DIM - 1)/2 = 1, where DIM=3
			const Real o = numeric_const<Real>::TWO*rand.template nextUniform<Real>() - numeric_const<Real>::ONE;
			const Real p = rand.template nextUniform<Real>();
			const Real s = Math::sqr(o) + Math::sqr(p);
			if (s > numeric_const<Real>::ONE)
				continue;
			const Real z = numeric_const<Real>::HALF + o*p*Math::sqrt(numeric_const<Real>::ONE - s) / s;

			const Real u = rand.template nextUniform<Real>();
			const Real v = numeric_const<Real>::ONE - z*(numeric_const<Real>::ONE - b);
			const Real w = (numeric_const<Real>::ONE - z*(numeric_const<Real>::ONE + b)) / v;
			//const Real t = k*w + Real(3.0)*Math::ln(numeric_const<Real>::ONE - x*w) - c; // DIM=4
			const Real t = k*w + Real(2.0)*Math::ln(numeric_const<Real>::ONE - x*w) - c; // DIM=3
			if (t < Math::ln(u))
				continue;

			const Real r = Math::sqrt(numeric_const<Real>::ONE - Math::sqr(w));
			this->z = w;
			_Vec2<Real> vec;
			vec.template next<Rand>(rand);
			this->x = r*vec.x;
			this->y = r*vec.y;
			break;
		}
	}

	/** tests for exact zero vector
	*/
	inline bool isZero() const {
		return Math::isZero(v1) && Math::isZero(v2) && Math::isZero(v3);
	}

	/** tests for positive vector
	*/
	inline bool isPositive(const Real eps = numeric_const<Real>::ZERO) const {
		return v1 > eps && v2 > eps && v3 > eps;
	}

	/** tests for negative vector
	*/
	inline bool isNegative(const Real eps = numeric_const<Real>::ZERO) const {
		return v1 < eps && v2 < eps && v3 < eps;
	}

	/** tests for finite vector
	*/
	inline bool isFinite() const {
		return Math::isFinite(v1) && Math::isFinite(v2) && Math::isFinite(v3);
	}

	/** returns true if this and args elements are within epsilon of each other.
	*/
	inline bool equals(const _Vec3& v, Real epsilon) const {
		return
			Math::equals(v1, v.v1, epsilon) &&
			Math::equals(v2, v.v2, epsilon) &&
			Math::equals(v3, v.v3, epsilon);
	}

	/**	Assignment operator.
	*/
	template <typename _Type> inline const _Vec3& operator = (const _Vec3<_Type>& v) {
		set(v);
		return *this;
	}

	/** Access the data as an array.
	*	@param	idx	Array index.
	*	@return		Array element pointed by idx.
	*/
	inline Real& operator [] (size_t idx) {
		return v[idx];
	}
	inline const Real& operator [] (size_t idx) const {
		return v[idx];
	}
	
	/** true if all the members are smaller.
	*/
	inline bool operator < (const _Vec3& v) const {
		return (v1 < v.v1) && (v2 < v.v2) && (v3 < v.v3);
	}

	/** true if all the members are smaller or equal.
	*/
	inline bool operator <= (const _Vec3& v) const {
		return (v1 <= v.v1) && (v2 <= v.v2) && (v3 <= v.v3);
	}

	/** true if all the members are larger.
	*/
	inline bool operator > (const _Vec3& v) const {
		return (v1 > v.v1) && (v2 > v.v2) && (v3 > v.v3);
	}

	/** true if all the members are larger or equal.
	*/
	inline bool operator >= (const _Vec3& v) const {
		return (v1 >= v.v1) && (v2 >= v.v2) && (v3 >= v.v3);
	}

	/** returns true if the two vectors are exactly equal.
	*/
	inline bool operator == (const _Vec3& v) const {
		return (v1 == v.v1) && (v2 == v.v2) && (v3 == v.v3);
	}

	/** returns true if the two vectors are exactly unequal.
	*/
	inline bool operator != (const _Vec3& v) const {
		return (v1 != v.v1) || (v2 != v.v2) || (v3 != v.v3);
	}

	/** negation
	*/
	_Vec3 operator - () const {
		return _Vec3(-v1, -v2, -v3);
	}
	/** vector addition
	*/
	_Vec3 operator + (const _Vec3 & v) const {
		return _Vec3(v1 + v.v1, v2 + v.v2, v3 + v.v3);
	}
	/** vector difference
	*/
	_Vec3 operator - (const _Vec3 & v) const {
		return _Vec3(v1 - v.v1, v2 - v.v2, v3 - v.v3);
	}
	/** scalar post-multiplication
	*/
	_Vec3 operator * (Real f) const {
		return _Vec3(v1 * f, v2 * f, v3 * f);
	}
	/** vector array multiplication
	*/
	_Vec3 operator * (const _Vec3 & v) const {
		return _Vec3(v1 * v.v1, v2 * v.v2, v3 * v.v3);
	}
	/** scalar division
	*/
	_Vec3 operator / (Real f) const {
		f = golem::numeric_const<Real>::ONE / f;
		return _Vec3(v1 * f, v2 * f, v3 * f);
	}
	/** vector array division
	*/
	_Vec3 operator / (const _Vec3 & v) const {
		return _Vec3(v1 / v.v1, v2 / v.v2, v3 / v.v3);
	}
	/** vector addition
	*/
	_Vec3& operator += (const _Vec3& v) {
		v1 += v.v1;
		v2 += v.v2;
		v3 += v.v3;
		return *this;
	}
	/** vector difference
	*/
	_Vec3& operator -= (const _Vec3& v) {
		v1 -= v.v1;
		v2 -= v.v2;
		v3 -= v.v3;
		return *this;
	}
	/** scalar multiplication
	*/
	_Vec3& operator *= (Real f) {
		v1 *= f;
		v2 *= f;
		v3 *= f;
		return *this;
	}
	/** vector array multiplication
	*/
	_Vec3& operator *= (const _Vec3& v) {
		v1 *= v.v1;
		v2 *= v.v2;
		v3 *= v.v3;
		return *this;
	}
	/** scalar division
	*/
	_Vec3& operator /= (Real f) {
		f = golem::numeric_const<Real>::ONE / f;
		v1 *= f;
		v2 *= f;
		v3 *= f;
		return *this;
	}
	/** vector array division
	*/
	_Vec3& operator /= (const _Vec3& v) {
		v1 /= v.v1;
		v2 /= v.v2;
		v3 /= v.v3;
		return *this;
	}
	/** cross product
	*/
	_Vec3 operator ^ (const _Vec3& v) const {
		return _Vec3(v2*v.v3 - v3*v.v2, v3*v.v1 - v1*v.v3, v1*v.v2 - v2*v.v1);
	}
	/** dot product
	*/
	Real operator | (const _Vec3& v) const {
		return v1 * v.v1 + v2 * v.v2 + v3 * v.v3;
	}

	/** inverse
	*/
	inline _Vec3 operator ~ () const {
		return _Vec3(-v1, -v2, -v3);
	}
};

//------------------------------------------------------------------------------

/** Default type */
typedef _Vec3<Real> Vec3;

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_VEC3_H_*/
