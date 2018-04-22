/** @file Mat22.h
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
#ifndef _GOLEM_MATH_MAT22_H_
#define _GOLEM_MATH_MAT22_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Vec2.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Matrix representation of SO(2) group of rotations.
*/
template <typename _Real> class _Mat22 {
public:
	/** Real */
	typedef _Real Real;
	/** Vec2 */
	typedef _Vec2<_Real> Vec2;

	/** matrix elements */
	union {
		struct {
			Real m11, m12;
			Real m21, m22;
		};
		Real m[2][2];
	};

	/** Default constructor does not do any initialisation.
	*/
	inline _Mat22() {}

	/** Creates matrix from sequence of numbers.
	*/
	template <typename _Type> inline _Mat22(_Type m11, _Type m12, _Type m21, _Type m22) : m11((Real)m11), m12((Real)m12), m21((Real)m21), m22((Real)m22) {
	}

	/**	Copy constructor.
	*/
	template <typename _Type> inline _Mat22(const _Mat22<_Type> &m) : m11((_Real)m.m11), m12((_Real)m.m12), m21((_Real)m.m21), m22((_Real)m.m22) {}

	/** Creates SO(2) matrix from the specified coordinate frame axes.
	*/
	inline _Mat22(const Vec2& xb, const Vec2& yb) {
		fromAxes(xb, yb);
	}

	/** Creates SO(2) matrix from rotation angle.
	*/
	inline _Mat22(Real angle) {
		fromAngle(angle);
	}

	/** Static initialisation member - zero.
	*/
	static inline _Mat22 zero() {
		return _Mat22(numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO);
	}

	/** Static initialisation member - identity.
	*/
	static inline _Mat22 identity() {
		return _Mat22(numeric_const<Real>::ONE, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ONE);
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
		return &m11;
	}
	inline const Real *data() const {
		return &m11;
	}

	/** this = m
	*/
	template <typename _Type> inline void set(const _Mat22<_Type> &m) {
		m11 = (Real)m.m11;		m12 = (Real)m.m12;
		m21 = (Real)m.m21;		m22 = (Real)m.m22;
	}

	template <typename _Type> inline void setRow22(const _Type m[]) {
		m11 = (Real)m[0];		m12 = (Real)m[1];
		m21 = (Real)m[2];		m22 = (Real)m[3];
	}

	template <typename _Type> inline void setColumn22(const _Type m[]) {
		m11 = (Real)m[0];		m12 = (Real)m[2];
		m21 = (Real)m[1];		m22 = (Real)m[3];
	}

	template <typename _Type> inline void getRow22(_Type m[]) const {
		m[0] = (_Type)m11;		m[1] = (_Type)m12;
		m[2] = (_Type)m21;		m[3] = (_Type)m22;
	}

	template <typename _Type> inline void getColumn22(_Type m[]) const {
		m[0] = (_Type)m11;		m[2] = (_Type)m12;
		m[1] = (_Type)m21;		m[3] = (_Type)m22;
	}

	//for loose 3-padded data.
	template <typename _Type> inline void setRow33(const _Type m[]) {
		m11 = (Real)m[0];		m12 = (Real)m[1];
		m21 = (Real)m[3];		m22 = (Real)m[4];
	}

	template <typename _Type> inline void setColumn33(const _Type m[]) {
		m11 = (Real)m[0];		m12 = (Real)m[3];
		m21 = (Real)m[1];		m22 = (Real)m[4];
	}

	template <typename _Type> inline void getRow33(_Type m[]) const {
		m[0] = (_Type)m11;		m[1] = (_Type)m12;
		m[2] = (_Type)m21;		m[3] = (_Type)m22;
	}

	template <typename _Type> inline void getColumn33(_Type m[]) const {
		m[0] = (_Type)m11;		m[3] = (_Type)m12;
		m[1] = (_Type)m21;		m[4] = (_Type)m22;
	}

	inline void setRow(size_t row, const Vec2& v) {
		m[row][0] = v.v1;		m[row][1] = v.v2;
	}

	inline void setColumn(size_t col, const Vec2& v) {
		m[0][col] = v.v1;		m[1][col] = v.v2;
	}

	inline void getRow(size_t row, Vec2& v) const {
		v.v1 = m[row][0];		v.v2 = m[row][1];
	}

	inline void getColumn(size_t col, Vec2& v) const {
		v.v1 = m[0][col];		v.v2 = m[1][col];
	}

	inline Vec2 getRow(size_t row) const {
		return Vec2(m[row][0], m[row][1]);
	}

	inline Vec2 getColumn(size_t col) const {
		return Vec2(m[0][col], m[1][col]);
	}

	inline Real& operator () (size_t row, size_t col) {
		return m[row][col];
	}

	/** returns true for exact identity matrix
	*/
	inline bool isIdentity() const {
		return
			m11 == numeric_const<Real>::ONE && Math::isZero(m12) &&
			Math::isZero(m21) && m22 == numeric_const<Real>::ONE;
	}

	/** returns true for exact zero matrix
	*/
	inline bool isZero() const {
		return
			Math::isZero(m11) && Math::isZero(m12) &&
			Math::isZero(m21) && Math::isZero(m22);
	}

	/** returns true if all elems are finite
	*/
	inline bool isFinite() const {
		return
			Math::isFinite(m11) && Math::isFinite(m12) &&
			Math::isFinite(m21) && Math::isFinite(m22);
	}

	/** returns true if this and arg's elems are within epsilon of each other.
	*/
	inline bool equals(const _Mat22& m, Real epsilon) const {
		return
			Math::equals(m11, m.m11, epsilon) && Math::equals(m12, m.m12, epsilon) &&
			Math::equals(m21, m.m21, epsilon) && Math::equals(m22, m.m22, epsilon);
	}

	/** sets this matrix to the zero matrix.
	*/
	inline void setZero() {
		m11 = numeric_const<Real>::ZERO;			m12 = numeric_const<Real>::ZERO;
		m21 = numeric_const<Real>::ZERO;			m22 = numeric_const<Real>::ZERO;
	}

	/** sets this matrix to the identity matrix.
	*/
	inline void setId() {
		m11 = numeric_const<Real>::ONE;				m12 = numeric_const<Real>::ZERO;
		m21 = numeric_const<Real>::ZERO;			m22 = numeric_const<Real>::ONE;
	}

	/** this = -this
	*/
	inline void setNegative() {
		m11 = -m11;									m12 = -m12;
		m21 = -m21;									m22 = -m22;
	}

	/** sets this matrix to the diagonal matrix.
	*/
	inline void setDiagonal(const Vec2 &v) {
		m11 = v.v1;									m12 = numeric_const<Real>::ZERO;
		m21 = numeric_const<Real>::ZERO;			m22 = v.v2;
	}

	/** Creates SO(2) matrix from rotation angle.
	*/
	inline void fromAngle(Real angle) {
		Real s, c;
		Math::sinCos(angle,	s, c);

		m11 = c;			m12 = -s;
		m21 = s;			m22 = c;
	}

	/** Returns rotation angle of SO(2) matrix
	*/
	inline void toAngle(Real& angle) const {
		angle = Math::atan2(m21, m11);
	}

	/** Creates SO(2) matrix from the specified coordinate frame axes.
	*/
	inline void fromAxes(const Vec2& xb, const Vec2& yb) {
		Vec2 xa(numeric_const<Real>::ONE, numeric_const<Real>::ZERO);
		Vec2 ya(numeric_const<Real>::ZERO, numeric_const<Real>::ONE);
		
		m11 = xa.dot(xb);
		m21 = ya.dot(xb);
		
		m12 = xa.dot(yb);
		m22 = ya.dot(yb);
	}

	/** returns trace
	*/
	inline Real trace() const {
		return m11 + m22;
	}

	/** returns determinant
	*/
	inline Real determinant() const {
		return
			m11*m22 - m12*m21;
	}

	/** this = transpose(m), (inverse if  R is rotation matrix)
	*/
	inline void setTransposed(const _Mat22& m) {
		if (this != &m) {
			m11 = m.m11;		m12 = m.m21;
			m21 = m.m12;		m22 = m.m22;
		}
		else
			setTransposed();
	}

	/** this = transposed(this), (inverse if  R is rotation matrix)
	*/
	inline void setTransposed() {
		std::swap(m12, m21);
	}

	/** a = this * b
	*/
	inline void multiply(Vec2& a, const Vec2& b) const {
		Real v1 = m11 * b.v1 + m12 * b.v2;
		Real v2 = m21 * b.v1 + m22 * b.v2;

		a.v1 = v1;
		a.v2 = v2;
	}

	/** a = transpose(this) * b, (inverse if R is rotation matrix)
	*/
	inline void multiplyByTranspose(Vec2& a, const Vec2& b) const {
		Real v1 = m11 * b.v1 + m21 * b.v2;
		Real v2 = m12 * b.v1 + m22 * b.v2;

		a.v1 = v1;
		a.v2 = v2;
	}

	/** this = a + b
	*/
	inline void  add(const _Mat22& a, const _Mat22& b) {
		m11 = a.m11 + b.m11;		m12 = a.m12 + b.m12;
		m21 = a.m21 + b.m21;		m22 = a.m22 + b.m22;
	}

	/** this = a - b
	*/
	inline void  subtract(const _Mat22& a, const _Mat22& b) {
		m11 = a.m11 - b.m11;		m12 = a.m12 - b.m12;
		m21 = a.m21 - b.m21;		m22 = a.m22 - b.m22;
	}

	/** this = s * m;
	*/
	inline void multiply(Real s, const _Mat22& m) {
		m11 = m.m11 * s;		m12 = m.m12 * s;
		m21 = m.m21 * s;		m22 = m.m22 * s;
	}

	/** this = a * b
	*/
	inline void multiply(const _Mat22& a, const _Mat22& b) {
		Real a11 = a.m11 * b.m11 + a.m12 * b.m21;
		Real a12 = a.m11 * b.m12 + a.m12 * b.m22;

		Real a21 = a.m21 * b.m11 + a.m22 * b.m21;
		Real a22 = a.m21 * b.m12 + a.m22 * b.m22;

		m11 = a11;		m12 = a12;
		m21 = a21;		m22 = a22;
	}

	/**	Assignment operator.
	*/
	template <typename _Type> inline _Mat22& operator = (const _Mat22<_Type>& m) {
		set(m);
		return *this;
	}

	inline _Mat22& operator += (const _Mat22 &m) {
		m11 += m.m11;		m12 += m.m12;
		m21 += m.m21;		m22 += m.m22;
		return *this;
	}

	inline _Mat22& operator -= (const _Mat22 &m) {
		m11 -= m.m11;		m12 -= m.m12;
		m21 -= m.m21;		m22 -= m.m22;
		return *this;
	}

	inline _Mat22& operator *= (const _Mat22& m) {
		multiply(*this, m);
		return *this;
	}

	inline _Mat22& operator *= (Real s) {
		m11 *= s;			m12 *= s;
		m21 *= s;			m22 *= s;
		return *this;
	}

	inline _Mat22& operator /= (Real s) {
		s = numeric_const<Real>::ONE / s;
		m11 *= s;			m12 *= s;
		m21 *= s;			m22 *= s;
		return *this;
	}

	/** matrix vector product
	*/
	inline Vec2 operator * (const Vec2& v) const {
		Vec2 tmp;
		multiply(tmp, v);
		return tmp;
	}

	/** matrix difference
	*/
	inline _Mat22 operator - (const _Mat22& m) const {
		_Mat22 tmp;
		tmp.subtract(*this, m);
		return tmp;
	}

	/** matrix addition
	*/
	inline _Mat22 operator + (const _Mat22& m) const {
		_Mat22 tmp;
		tmp.add(*this, m);
		return tmp;
	}

	/** matrix product
	*/
	inline _Mat22 operator * (const _Mat22& m) const {
		_Mat22 tmp;
		tmp.multiply(*this, m);
		return tmp;
	}

	/** matrix scalar product
	*/
	inline _Mat22 operator * (Real s) const {
		_Mat22 tmp;
		tmp.multiply(s, *this);
		return tmp;
	}
};

//------------------------------------------------------------------------------

/** Default type */
typedef _Mat22<Real> Mat22;

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_MAT22_H_*/
