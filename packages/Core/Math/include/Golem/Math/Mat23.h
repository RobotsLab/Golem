/** @file Mat23.h
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
#ifndef _GOLEM_MATH_MAT23_H_
#define _GOLEM_MATH_MAT23_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Mat22.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Homogeneous representation of SE(2) rigid body transformations.
*/
template <typename _Real> class _Mat23 {
public:
	/** Real */
	typedef _Real Real;
	/** Vec2 */
	typedef _Vec2<_Real> Vec2;
	/** Mat22 */
	typedef _Mat22<_Real> Mat22;

	/** rotation matrix	*/
	Mat22 R;
	/** translation	*/
	Vec2 p;

	/** Default constructor does not do any initialisation.
	*/
	inline _Mat23() {
	}

	/** Creates matrix from rotation matrix and translation vector
	*/
	template <typename _Type> inline _Mat23(const _Mat22<_Type>& R, const _Vec2<_Type>& p) : R(R), p(p) {
	}

	/** Copy constructor.
	*/
	template <typename _Type> inline _Mat23(const _Mat23<_Type>& m) : R(m.R), p(m.p) {}

	/** Creates matrix from rotation angle and translation vector
	*/
	inline _Mat23(Real angle, const Vec2& trn) : R(angle), p(trn) {
	}

	/** Static initialisation member - identity.
	*/
	static inline _Mat23 identity() {
		return _Mat23(Mat22::identity(), Vec2::zero());
	}

	/** returns true if the object is valid
	*/
	inline bool isValid() const {
		return isFinite();
	}

	/** the default configuration
	*/
	inline void setToDefault() {
		setId();
	}

	/**	Access the data as an array.
	*/
	inline Real* data() {
		return R.data();
	}
	inline const Real *data() const {
		return R.data();
	}

	/** set the matrix
	*/
	template <typename _Type> inline void set(const _Mat23<_Type>& m) {
		this->R.set(m.R);
		this->p.set(m.p);
	}

	/** set the matrix
	*/
	template <typename _Type> inline void set(const _Mat22<_Type>& R, const _Vec2<_Type>& p) {
		this->R.set(R);
		this->p.set(p);
	}

	/** set the matrix given a column matrix
	*/
	template <typename _Type> inline void setColumn33(const _Type m[]) {
		R.setColumn33(m);
		p.v1 = (Real)m[6];
		p.v2 = (Real)m[7];
	}

	/** retrieve the matrix in a column format
	*/
	template <typename _Type> inline void getColumn33(_Type m[]) const {
		R.getColumn33(m);
		m[6] = (_Type)p.v1;
		m[7] = (_Type)p.v2;
		m[2] = m[5] = numeric_const<_Type>::ZERO;
		m[8] = numeric_const<_Type>::ONE;
	}

	/** set the matrix given a row matrix.
	*/
	template <typename _Type> inline void setRow33(const _Type m[]) {
		R.setRow33(m);
		p.v1 = (Real)m[2];
		p.v2 = (Real)m[5];
	}

	/** retrieve the matrix in a row format.
	*/
	template <typename _Type> inline void getRow33(_Type m[]) const {
		R.getRow33(m);
		m[2] = (_Type)p.v1;
		m[5] = (_Type)p.v2;
		m[6] = m[7] = numeric_const<_Type>::ZERO;
		m[8] = numeric_const<_Type>::ONE;
	}

	inline void setZero() {
		R.setZero();
		p.setZero();
	}

	inline void setId() {
		R.setId();
		p.setZero();
	}

	/** Creates matrix from rotation angle and translation
	*/
	inline void fromAngleTranslation(Real angle, const Vec2& p) {
		this->R.fromAngle(angle);
		this->p = p;
	}

	/** Creates rotation angle and translation from matrix
	*/
	inline void toAngleTranslation(Real& angle, Vec2& p) const {
		this->R.toAngle(angle);
		p = this->p;
	}

	/** Returns true for identity matrix
	*/
	inline bool isIdentity() const {
		return R.isIdentity() && p.isZero();
	}

	/** Returns true if all elems are finite.
	*/
	inline bool isFinite() const {
		return R.isFinite() && p.isFinite();
	}

	/** returns true if this and arg's elems are within epsilon of each other.
	*/
	inline bool equals(const _Mat23& m, Real epsilon) const {
		return p.equals(m.p, epsilon) && R.equals(m.R, epsilon);
	}

	/** this = inverse(m): [ inv(R) , inv(R) * -p ], for orthonormal m: [ RT , RT * -p ].
	*/
	inline void setInverseRT(const _Mat23& m) {
		R.setTransposed(m.R);
		p.multiply(-numeric_const<Real>::ONE, m.p);
		R.multiply(p, p); 
	}

	/** a = this * b
	*/
	inline void multiply(Vec2& a, const Vec2& b) const {
		// a = R * b + p;
		R.multiply(a, b);
		a.add(a, p);
	}

	/** a = inverse(this) * b, (assumes R is rotation matrix)
	*/
	inline void multiplyByInverseRT(Vec2& a, const Vec2& b) const {
		// b = R * a + p => a = RT * b - RT * p = RT * (b - p) = R^-1 * (b - p)
		a.subtract(b, p);
		R.multiplyByTranspose(a, a);
	}

	/** this = a * b: [aR, ap] * [bR, bp] = [aR * bR, aR * bp + ap].
	*/
	inline void multiply(const _Mat23& a, const _Mat23& b) {
		Vec2 tmp;

		a.R.multiply(tmp, b.p);
		p.add(tmp, a.p);
		R.multiply(a.R, b.R);
	}

	/**	Assignment operator.
	*/
	template <typename _Type> inline const _Mat23& operator = (const _Mat23<_Type> &m) {
		set(m);
		return *this;
	}

	/** operator wrapper for multiply
	*/
	inline Vec2 operator * (const Vec2& a) const {
		Vec2 tmp;
		multiply(tmp, a);
		return tmp;
	}

	/** operator wrapper for multiply
	*/
	inline _Mat23 operator * (const _Mat23& b) const {
		_Mat23 a;
		a.multiply(*this, b);
		return a;
	}
};

//------------------------------------------------------------------------------

/** Default type */
typedef _Mat23<Real> Mat23;

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_MAT23_H_*/
