/** @file Mat33.h
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
#ifndef _GOLEM_MATH_MAT33_H_
#define _GOLEM_MATH_MAT33_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Vec3.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

template <typename _Mat33, typename _Quat> inline void QuatToMat33(_Mat33& m, const _Quat& q) {
	typedef typename _Mat33::Real Real;

	m.m11 = numeric_const<Real>::ONE - q.q2*q.q2*numeric_const<Real>::TWO - q.q3*q.q3*numeric_const<Real>::TWO;
	m.m12 = q.q1*q.q2*numeric_const<Real>::TWO - q.q0*q.q3*numeric_const<Real>::TWO;
	m.m13 = q.q1*q.q3*numeric_const<Real>::TWO + q.q0*q.q2*numeric_const<Real>::TWO;

	m.m21 = q.q1*q.q2*numeric_const<Real>::TWO + q.q0*q.q3*numeric_const<Real>::TWO;
	m.m22 = numeric_const<Real>::ONE - q.q1*q.q1*numeric_const<Real>::TWO - q.q3*q.q3*numeric_const<Real>::TWO;
	m.m23 = q.q2*q.q3*numeric_const<Real>::TWO - q.q0*q.q1*numeric_const<Real>::TWO;

	m.m31 = q.q1*q.q3*numeric_const<Real>::TWO - q.q0*q.q2*numeric_const<Real>::TWO;
	m.m32 = q.q2*q.q3*numeric_const<Real>::TWO + q.q0*q.q1*numeric_const<Real>::TWO;
	m.m33 = numeric_const<Real>::ONE - q.q1*q.q1*numeric_const<Real>::TWO - q.q2*q.q2*numeric_const<Real>::TWO;
}

template <typename _Mat33, typename _Quat> inline void Mat33ToQuat(_Quat& q, const _Mat33& m) {
	typedef typename _Mat33::Real Real;

	Real trace = m.trace(), s;

	if (trace >= numeric_const<Real>::ZERO) {
		s = Math::sqrt(trace + numeric_const<Real>::ONE);
		q.q0 = s * numeric_const<Real>::HALF;
		s = numeric_const<Real>::HALF / s;

		q.q1 = (m.m32 - m.m23) * s;
		q.q2 = (m.m13 - m.m31) * s;
		q.q3 = (m.m21 - m.m12) * s;
	}
	else {
		int idx = 0;
		if (m.m22 > m.m11) idx = 1;
		if (m.m33 > m.m[idx][idx]) idx = 2;

		switch (idx) {
#define GOLEM_MAT33TOQUAT_MACRO(i, j, k, I, J, K)\
		case I:\
			s = Math::sqrt(m.m[I][I] - m.m[J][J] - m.m[K][K] + numeric_const<Real>::ONE);\
			q.i = s * numeric_const<Real>::HALF;\
			s = numeric_const<Real>::HALF / s;\
			q.j = (m.m[I][J] + m.m[J][I]) * s;\
			q.k = (m.m[K][I] + m.m[I][K]) * s;\
			q.q0 = (m.m[K][J] - m.m[J][K]) * s;\
			break
			GOLEM_MAT33TOQUAT_MACRO(q1, q2, q3, 0, 1, 2);
			GOLEM_MAT33TOQUAT_MACRO(q2, q3, q1, 1, 2, 0);
			GOLEM_MAT33TOQUAT_MACRO(q3, q1, q2, 2, 0, 1);
#undef GOLEM_MAT33TOQUAT_MACRO
		}
	}
}

//------------------------------------------------------------------------------

/** Matrix representation of SO(3) group of rotations.
*/
template <typename _Real> class _Mat33 {
public:
	/** Real */
	typedef _Real Real;
	/** Vec3 */
	typedef _Vec3<_Real> Vec3;

	/** matrix elements */
	union {
		struct {
			Real m11, m12, m13;
			Real m21, m22, m23;
			Real m31, m32, m33;		
		};
		Real m[3][3];
	};

	/** Default constructor does not do any initialisation.
	*/
	inline _Mat33() {}

	/** Creates matrix from sequence of numbers.
	*/
	template <typename _Type> inline _Mat33(_Type m11, _Type m12, _Type m13, _Type m21, _Type m22, _Type m23, _Type m31, _Type m32, _Type m33) : m11((Real)m11), m12((Real)m12), m13((Real)m13), m21((Real)m21), m22((Real)m22), m23((Real)m23), m31((Real)m31), m32((Real)m32), m33((Real)m33) {}

	/**	Copy constructor.
	*/
	template <typename _Type> inline _Mat33(const _Mat33<_Type> &m) : m11((Real)m.m11), m12((Real)m.m12), m13((Real)m.m13), m21((Real)m.m21), m22((Real)m.m22), m23((Real)m.m23), m31((Real)m.m31), m32((Real)m.m32), m33((Real)m.m33) {}

	/** Creates SO(3) matrix from the specified coordinate frame axes.
	*/
	inline _Mat33(const Vec3& xb, const Vec3& yb, const Vec3& zb) {
		fromAxes(xb, yb, zb);
	}

	/** Creates SO(3) matrix from rotation around axis for normalised axis.
	*/
	inline _Mat33(Real angle, const Vec3& axis) {
		fromAngleAxis(angle, axis);
	}

	/** Creates from quaternion.
	*	@param	q	quaterion to extract rotation matrix from.
	*/
	template <typename _Quat> inline _Mat33(const _Quat &q) {
		fromQuat(q);
	}

	/** Rotation matrix from Euler angles - rotation about: X (roll) -> Y(pitch) -> Z(yaw).
	*   roll, pitch, yaw  in <-PI/2, PI/2>
	*/
	inline _Mat33(Real roll, Real pitch, Real yaw) {
		fromEuler(roll, pitch, yaw);
	}

	/** Static initialisation member - zero.
	*/
	static inline _Mat33 zero() {
		return _Mat33(numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO);
	}

	/** Static initialisation member - identity.
	*/
	static inline _Mat33 identity() {
		return _Mat33(numeric_const<Real>::ONE, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ONE, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ONE);
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
	template <typename _Type> inline void set(const _Mat33<_Type> &m) {
		m11 = (Real)m.m11;		m12 = (Real)m.m12;		m13 = (Real)m.m13;
		m21 = (Real)m.m21;		m22 = (Real)m.m22;		m23 = (Real)m.m23;
		m31 = (Real)m.m31;		m32 = (Real)m.m32;		m33 = (Real)m.m33;
	}

	template <typename _Type> inline void setRow33(const _Type m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[1];	m13 = (Real)m[2];
		m21 = (Real)m[3];	m22 = (Real)m[4];	m23 = (Real)m[5];
		m31 = (Real)m[6];	m32 = (Real)m[7];	m33 = (Real)m[8];
	}

	template <typename _Type> inline void setColumn33(const _Type m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[3];	m13 = (Real)m[6];
		m21 = (Real)m[1];	m22 = (Real)m[4];	m23 = (Real)m[7];
		m31 = (Real)m[2];	m32 = (Real)m[5];	m33 = (Real)m[8];
	}

	template <typename _Type> inline void getRow33(_Type m[]) const {
		m[0] = (_Type)m11;	m[1] = (_Type)m12;	m[2] = (_Type)m13;
		m[3] = (_Type)m21;	m[4] = (_Type)m22;	m[5] = (_Type)m23;
		m[6] = (_Type)m31;	m[7] = (_Type)m32;	m[8] = (_Type)m33;
	}

	template <typename _Type> inline void getColumn33(_Type m[]) const {
		m[0] = (_Type)m11;	m[3] = (_Type)m12;	m[6] = (_Type)m13;
		m[1] = (_Type)m21;	m[4] = (_Type)m22;	m[7] = (_Type)m23;
		m[2] = (_Type)m31;	m[5] = (_Type)m32;	m[8] = (_Type)m33;
	}

	//for loose 4-padded data.
	template <typename _Type> inline void setRow44(const _Type m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[1];	m13 = (Real)m[2];
		m21 = (Real)m[4];	m22 = (Real)m[5];	m23 = (Real)m[6];
		m31 = (Real)m[8];	m32 = (Real)m[9];	m33 = (Real)m[10];
	}

	template <typename _Type> inline void setColumn44(const _Type m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[4];	m13 = (Real)m[8];
		m21 = (Real)m[1];	m22 = (Real)m[5];	m23 = (Real)m[9];
		m31 = (Real)m[2];	m32 = (Real)m[6];	m33 = (Real)m[10];
	}

	template <typename _Type> inline void getRow44(_Type m[]) const {
		m[0] = (_Type)m11;	m[1] = (_Type)m12;	m[2] = (_Type)m13;
		m[4] = (_Type)m21;	m[5] = (_Type)m22;	m[6] = (_Type)m23;
		m[8] = (_Type)m31;	m[9] = (_Type)m32;	m[10] = (_Type)m33;
	}

	template <typename _Type> inline void getColumn44(_Type m[]) const {
		m[0] = (_Type)m11;	m[4] = (_Type)m12;	m[8] = (_Type)m13;
		m[1] = (_Type)m21;	m[5] = (_Type)m22;	m[9] = (_Type)m23;
		m[2] = (_Type)m31;	m[6] = (_Type)m32;	m[10] = (_Type)m33;
	}

	inline void setRow(size_t row, const Vec3& v) {
		m[row][0] = v.v1;	m[row][1] = v.v2;	m[row][2] = v.v3;
	}

	inline void setColumn(size_t col, const Vec3& v) {
		m[0][col] = v.v1;	m[1][col] = v.v2;	m[2][col] = v.v3;
	}

	inline void getRow(size_t row, Vec3& v) const {
		v.v1 = m[row][0];	v.v2 = m[row][1];	v.v3 = m[row][2];
	}

	inline void getColumn(size_t col, Vec3& v) const {
		v.v1 = m[0][col];	v.v2 = m[1][col];	v.v3 = m[2][col];
	}

	inline Vec3 getRow(size_t row) const {
		return Vec3(m[row][0], m[row][1], m[row][2]);
	}

	inline Vec3 getColumn(size_t col) const {
		return Vec3(m[0][col], m[1][col], m[2][col]);
	}

	inline Real& operator () (size_t row, size_t col) {
		return m[row][col];
	}

	/** returns true for exact identity matrix
	*/
	inline bool isIdentity() const {
		return
			m11 == numeric_const<Real>::ONE && Math::isZero(m12) && Math::isZero(m13) &&
			Math::isZero(m21) && m22 == numeric_const<Real>::ONE && Math::isZero(m23) &&
			Math::isZero(m31) && Math::isZero(m32) && m33 == numeric_const<Real>::ONE;
	}

	/** returns true for exact zero matrix
	*/
	inline bool isZero() const {
		return
			Math::isZero(m11) && Math::isZero(m12) && Math::isZero(m13) &&
			Math::isZero(m21) && Math::isZero(m22) && Math::isZero(m23) &&
			Math::isZero(m31) && Math::isZero(m32) && Math::isZero(m33);
	}

	/** returns true if all elems are finite
	*/
	inline bool isFinite() const {
		return
			Math::isFinite(m11) && Math::isFinite(m12) && Math::isFinite(m13) &&
			Math::isFinite(m21) && Math::isFinite(m22) && Math::isFinite(m23) &&
			Math::isFinite(m31) && Math::isFinite(m32) && Math::isFinite(m33);
	}

	/** returns true if this and arg's elems are within epsilon of each other.
	*/
	inline bool equals(const _Mat33& m, Real epsilon) const {
		return
			Math::equals(m11, m.m11, epsilon) && Math::equals(m12, m.m12, epsilon) && Math::equals(m13, m.m13, epsilon) &&
			Math::equals(m21, m.m21, epsilon) && Math::equals(m22, m.m22, epsilon) && Math::equals(m23, m.m23, epsilon) &&
			Math::equals(m31, m.m31, epsilon) && Math::equals(m32, m.m32, epsilon) && Math::equals(m33, m.m33, epsilon);
	}

	/** sets this matrix to the zero matrix.
	*/
	inline void setZero() {
		m11 = numeric_const<Real>::ZERO;	m12 = numeric_const<Real>::ZERO;	m13 = numeric_const<Real>::ZERO;
		m21 = numeric_const<Real>::ZERO;	m22 = numeric_const<Real>::ZERO;	m23 = numeric_const<Real>::ZERO;
		m31 = numeric_const<Real>::ZERO;	m32 = numeric_const<Real>::ZERO;	m33 = numeric_const<Real>::ZERO;
	}

	/** sets this matrix to the identity matrix.
	*/
	inline void setId() {
		m11 = numeric_const<Real>::ONE;		m12 = numeric_const<Real>::ZERO;	m13 = numeric_const<Real>::ZERO;
		m21 = numeric_const<Real>::ZERO;	m22 = numeric_const<Real>::ONE;		m23 = numeric_const<Real>::ZERO;
		m31 = numeric_const<Real>::ZERO;	m32 = numeric_const<Real>::ZERO;	m33 = numeric_const<Real>::ONE;
	}

	/** this = -this
	*/
	inline void setNegative() {
		m11 = -m11;		m12 = -m12;		m13 = -m13;
		m21 = -m21;		m22 = -m22;		m23 = -m23;
		m31 = -m31;		m32 = -m32;		m33 = -m33;
	}

	/** sets this matrix to the diagonal matrix.
	*/
	inline void setDiagonal(const Vec3 &v) {
		m11 = v.v1;							m12 = numeric_const<Real>::ZERO;	m13 = numeric_const<Real>::ZERO;
		m21 = numeric_const<Real>::ZERO;	m22 = v.v2;							m23 = numeric_const<Real>::ZERO;
		m31 = numeric_const<Real>::ZERO;	m32 = numeric_const<Real>::ZERO;	m33 = v.v3;
	}

	/** Finds axis of SO(3) matrix: R * axis = axis
	Returns false for degenerate case, i.e. if R == Id
	*/
	inline bool getAxis(Vec3 &axis) const {
		if (isIdentity()) {
			axis.setZero();
			return false;
		}

		// TODO: solve system of linear equations R * axis = axis

		return true;
	}

	/** Creates skew-symmetric so(3) matrix from axis (wedge operator ^).
	*/
	inline void axisToSkew(const Vec3 &axis) {
		m11 = numeric_const<Real>::ZERO;	m12 = -axis.v3;						m13 = axis.v2;
		m21 = axis.v3;						m22 = numeric_const<Real>::ZERO;	m23 = -axis.v1;
		m31 = -axis.v2;						m32 = axis.v1;						m33 = numeric_const<Real>::ZERO;
	}

	/** Creates axis from skew-symmetric so(3) matrix (vee operator v).
	*/
	inline void skewToAxis(Vec3 &axis) const {
		axis.v1 = -m23;
		axis.v2 =  m13;
		axis.v3 = -m12;
	}

	/** Creates SO(3) matrix from the specified coordinate frame axes.
	*/
	inline void fromAxes(const Vec3& xb, const Vec3& yb, const Vec3& zb) {
		Vec3 xa(numeric_const<Real>::ONE, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO);
		Vec3 ya(numeric_const<Real>::ZERO, numeric_const<Real>::ONE, numeric_const<Real>::ZERO);
		Vec3 za(numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ONE);
		
		m11 = xa.dot(xb);
		m21 = ya.dot(xb);
		m31 = za.dot(xb);
		
		m12 = xa.dot(yb);
		m22 = ya.dot(yb);
		m32 = za.dot(yb);
		
		m13 = xa.dot(zb);
		m23 = ya.dot(zb);
		m33 = za.dot(zb);
	}

	/** Creates SO(3) matrix from rotation around axis for normalised axis.
		Rodrigues' formula for exponential maps:
			exp(axis^ angle) = Id + axis^ sin(angle) + axis^ axis^ (1 - cos(angle)),
		where axis^ is so(3) matrix generated from axis.
	*/
	inline void fromAngleAxis(Real angle, const Vec3& axis) {
		Real s, c;
		Math::sinCos(angle, s, c);
		Real v = numeric_const<Real>::ONE - c, v1 = axis.v1*v, v2 = axis.v2*v, v3 = axis.v3*v;

		m11 = axis.v1*v1 + c;			m12 = axis.v1*v2 - axis.v3*s;	m13 = axis.v1*v3 + axis.v2*s;	
		m21 = axis.v2*v1 + axis.v3*s;	m22 = axis.v2*v2 + c;			m23 = axis.v2*v3 - axis.v1*s;	
		m31 = axis.v3*v1 - axis.v2*s;	m32 = axis.v3*v2 + axis.v1*s;	m33 = axis.v3*v3 + c;	
	}

	/** Returns rotation angle and axis of rotation of SO(3) matrix
	*/
	inline void toAngleAxis(Real& angle, Vec3& axis) const {
		angle = Math::acos(numeric_const<Real>::HALF * (trace() - numeric_const<Real>::ONE));
		Real s = numeric_const<Real>::TWO * Math::sin(angle);
		if (Math::abs(s) > numeric_const<Real>::EPS)
			axis.set((m32 - m23)/s, (m13 - m31)/s, (m21 - m12)/s);
		else
			axis.setZero();
	}

	template <typename _Quat> inline void fromQuat(const _Quat& q) {
		QuatToMat33(*this, q);
	}

	template <typename _Quat> inline void toQuat(_Quat& q) const {
		Mat33ToQuat(q, *this);
	}

	/** this = rotation matrix around X axis
	*/
	inline void rotX(Real angle) {
		Real s, c;
		Math::sinCos(angle,	s, c);

		m11 = numeric_const<Real>::ONE;		m12 = numeric_const<Real>::ZERO;	m13 = numeric_const<Real>::ZERO;
		m21 = numeric_const<Real>::ZERO;	m22 = c;							m23 = -s;
		m31 = numeric_const<Real>::ZERO;	m32 = s;							m33 = c;
	}

	/** this = rotation matrix around Y axis
	*/
	inline void rotY(Real angle) {
		Real s, c;
		Math::sinCos(angle,	s, c);

		m11 = c;							m12 = numeric_const<Real>::ZERO;	m13 = s;
		m21 = numeric_const<Real>::ZERO;	m22 = numeric_const<Real>::ONE;		m23 = numeric_const<Real>::ZERO;
		m31 = -s;							m32 = numeric_const<Real>::ZERO;	m33 = c;
	}

	/** this = rotation matrix around Z axis
	*/
	inline void rotZ(Real angle) {
		Real s, c;
		Math::sinCos(angle,	s, c);

		m11 = c;							m12 = -s;							m13 = numeric_const<Real>::ZERO;
		m21 = s;							m22 = c;							m23 = numeric_const<Real>::ZERO;
		m31 = numeric_const<Real>::ZERO;	m32 = numeric_const<Real>::ZERO;	m33 = numeric_const<Real>::ONE;
	}

	/** Rotation matrix from Euler angles - rotation about: X (roll) -> Y(pitch) -> Z(yaw).
	*   roll, pitch, yaw  in <-PI/2, PI/2>
	*/
	inline void fromEuler(Real roll, Real pitch, Real yaw) {
		Real sg, cg;
		Math::sinCos(roll, sg, cg);
		Real sb, cb;
		Math::sinCos(pitch, sb, cb);
		Real sa, ca;
		Math::sinCos(yaw, sa, ca);

		m11 = ca*cb;		m12 = ca*sb*sg - sa*cg;		m13 = ca*sb*cg + sa*sg;
		m21 = sa*cb;		m22 = sa*sb*sg + ca*cg;		m23 = sa*sb*cg - ca*sg;
		m31 = -sb;			m32 = cb*sg;				m33 = cb*cg;
	}

	/** Rotation matrix to Euler angles - rotation about: X (roll) -> Y(pitch) -> Z(yaw).
	*/
	inline void toEuler(Real &roll, Real &pitch, Real &yaw) const {
		roll = Math::atan2(m32, m33);
		pitch = Math::atan2(-m31, Math::sqrt(m32*m32 + m33*m33));
		yaw = Math::atan2(m21, m11);
	}

	/** returns trace
	*/
	inline Real trace() const {
		return m11 + m22 + m33;
	}

	/** returns determinant
	*/
	inline Real determinant() const {
		return
			m11*m22*m33 + m12*m23*m31 + m13*m21*m32 -
			m13*m22*m31 - m12*m21*m33 - m11*m23*m32;
	}

	/** this = inverse(m).
	@return false if singular (i.e. if no inverse exists)
	*/
	inline bool setInverse(const _Mat33& m) {
		Real temp11 = m.m22*m.m33 - m.m23*m.m32;
		Real temp12 = m.m13*m.m32 - m.m12*m.m33;
		Real temp13 = m.m12*m.m23 - m.m13*m.m22;
		
		Real temp21 = m.m23*m.m31 - m.m21*m.m33;
		Real temp22 = m.m11*m.m33 - m.m13*m.m31;
		Real temp23 = m.m13*m.m21 - m.m11*m.m23;

		Real temp31 = m.m21*m.m32 - m.m22*m.m31;
		Real temp32 = m.m12*m.m31 - m.m11*m.m32;
		Real temp33 = m.m11*m.m22 - m.m12*m.m21;
		
		Real det = temp11*m.m11 + temp12*m.m21 + temp13*m.m31;
		
		if (Math::abs(det) < numeric_const<Real>::EPS)
			return false;
		
		det = numeric_const<Real>::ONE / det;
		
		m11 = temp11*det;		m12 = temp12*det;		m13 = temp13*det;
		m21 = temp21*det;		m22 = temp22*det;		m23 = temp23*det;
		m31 = temp31*det;		m32 = temp32*det;		m33 = temp33*det;
		
		return true;
	}

	/** this = transpose(m), (inverse if  R is rotation matrix)
	*/
	inline void setTransposed(const _Mat33& m) {
		if (this != &m) {
			m11 = m.m11;		m12 = m.m21;		m13 = m.m31;
			m21 = m.m12;		m22 = m.m22;		m23 = m.m32;
			m31 = m.m13;		m32 = m.m23;		m33 = m.m33;
		}
		else
			setTransposed();
	}

	/** this = transpose(this), (inverse if  R is rotation matrix)
	*/
	inline void setTransposed() {
		std::swap(m12, m21);
		std::swap(m23, m32);
		std::swap(m13, m31);
	}

	/** this = this * [ v.v1 0 0; 0 v.v2 0; 0 0 v.v3].
	*/
	inline void multiplyDiagonal(const Vec3& v) {
		m11 *= v.v1;		m12 *= v.v2;		m13 *= v.v3;
		m21 *= v.v1;		m22 *= v.v2;		m23 *= v.v3;
		m31 *= v.v1;		m32 *= v.v2;		m33 *= v.v3;
	}

	/** this = transpose(this) * [ v.v1 0 0; 0 v.v2 0; 0 0 v.v3].
	*/
	inline void multiplyDiagonalTranspose(const Vec3& v) {
		m11 *= v.v1;		m12 *= v.v1;		m13 *= v.v1;
		m21 *= v.v2;		m22 *= v.v2;		m23 *= v.v2;
		m31 *= v.v3;		m32 *= v.v3;		m33 *= v.v3;
	}

	/** m = this * [ v.v1 0 0; 0 v.v2 0; 0 0 v.v3];
	*/
	inline void multiplyDiagonal(_Mat33& m, const Vec3& v) const {
		m.m11 = m11 * v.v1;		m.m12 = m12 * v.v2;		m.m13 = m13 * v.v3;
		m.m21 = m21 * v.v1;		m.m22 = m22 * v.v2;		m.m23 = m23 * v.v3;
		m.m31 = m31 * v.v1;		m.m32 = m32 * v.v2;		m.m33 = m33 * v.v3;
	}

	/** m = transpose(this) * [ v.v1 0 0; 0 v.v2 0; 0 0 v.v3];
	*/
	inline void multiplyDiagonalTranspose(_Mat33& m, const Vec3& v) const {
		m.m11 = m11 * v.v1;		m.m12 = m21 * v.v2;		m.m13 = m31 * v.v3;
		m.m21 = m12 * v.v1;		m.m22 = m22 * v.v2;		m.m23 = m32 * v.v3;
		m.m31 = m13 * v.v1;		m.m32 = m23 * v.v2;		m.m33 = m33 * v.v3;
	}

	/** a = this * b
	*/
	inline void multiply(Vec3& a, const Vec3& b) const {
		Real v1 = m11 * b.v1 + m12 * b.v2 + m13 * b.v3;
		Real v2 = m21 * b.v1 + m22 * b.v2 + m23 * b.v3;
		Real v3 = m31 * b.v1 + m32 * b.v2 + m33 * b.v3;

		a.v1 = v1;
		a.v2 = v2;
		a.v3 = v3;	
	}

	/** a = transpose(this) * b
	*/
	inline void multiplyByTranspose(Vec3& a, const Vec3& b) const {
		Real v1 = m11 * b.v1 + m21 * b.v2 + m31 * b.v3;
		Real v2 = m12 * b.v1 + m22 * b.v2 + m32 * b.v3;
		Real v3 = m13 * b.v1 + m23 * b.v2 + m33 * b.v3;

		a.v1 = v1;
		a.v2 = v2;
		a.v3 = v3;
	}

	/** this = a + b
	*/
	inline void  add(const _Mat33& a, const _Mat33& b) {
		m11 = a.m11 + b.m11;	m12 = a.m12 + b.m12;	m13 = a.m13 + b.m13;
		m21 = a.m21 + b.m21;	m22 = a.m22 + b.m22;	m23 = a.m23 + b.m23;
		m31 = a.m31 + b.m31;	m32 = a.m32 + b.m32;	m33 = a.m33 + b.m33;
	}

	/** this = a - b
	*/
	inline void  subtract(const _Mat33& a, const _Mat33& b) {
		m11 = a.m11 - b.m11;		m12 = a.m12 - b.m12;		m13 = a.m13 - b.m13;
		m21 = a.m21 - b.m21;		m22 = a.m22 - b.m22;		m23 = a.m23 - b.m23;
		m31 = a.m31 - b.m31;		m32 = a.m32 - b.m32;		m33 = a.m33 - b.m33;
	}

	/** this = s * m;
	*/
	inline void multiply(Real s, const _Mat33& m) {
		m11 = m.m11 * s;		m12 = m.m12 * s;		m13 = m.m13 * s;
		m21 = m.m21 * s;		m22 = m.m22 * s;		m23 = m.m23 * s;
		m31 = m.m31 * s;		m32 = m.m32 * s;		m33 = m.m33 * s;
	}

	/** this = a * b
	*/
	inline void multiply(const _Mat33& a, const _Mat33& b) {
		Real a11 = a.m11 * b.m11 + a.m12 * b.m21 + a.m13 * b.m31;
		Real a12 = a.m11 * b.m12 + a.m12 * b.m22 + a.m13 * b.m32;
		Real a13 = a.m11 * b.m13 + a.m12 * b.m23 + a.m13 * b.m33;

		Real a21 = a.m21 * b.m11 + a.m22 * b.m21 + a.m23 * b.m31;
		Real a22 = a.m21 * b.m12 + a.m22 * b.m22 + a.m23 * b.m32;
		Real a23 = a.m21 * b.m13 + a.m22 * b.m23 + a.m23 * b.m33;

		Real a31 = a.m31 * b.m11 + a.m32 * b.m21 + a.m33 * b.m31;
		Real a32 = a.m31 * b.m12 + a.m32 * b.m22 + a.m33 * b.m32;
		Real a33 = a.m31 * b.m13 + a.m32 * b.m23 + a.m33 * b.m33;

		m11 = a11;		m12 = a12;		m13 = a13;
		m21 = a21;		m22 = a22;		m23 = a23;
		m31 = a31;		m32 = a32;		m33 = a33;
	}

	/**	Assignment operator.
	*/
	template <typename _Type> inline _Mat33& operator = (const _Mat33<_Type>& m) {
		set(m);
		return *this;
	}

	inline _Mat33& operator += (const _Mat33 &m) {
		m11 += m.m11;		m12 += m.m12;		m13 += m.m13;
		m21 += m.m21;		m22 += m.m22;		m23 += m.m23;
		m31 += m.m31;		m32 += m.m32;		m33 += m.m33;
		return *this;
	}

	inline _Mat33& operator -= (const _Mat33 &m) {
		m11 -= m.m11;		m12 -= m.m12;		m13 -= m.m13;
		m21 -= m.m21;		m22 -= m.m22;		m23 -= m.m23;
		m31 -= m.m31;		m32 -= m.m32;		m33 -= m.m33;
		return *this;
	}

	inline _Mat33& operator *= (const _Mat33& m) {
		multiply(*this, m);
		return *this;
	}

	inline _Mat33& operator *= (Real s) {
		m11 *= s;			m12 *= s;			m13 *= s;
		m21 *= s;			m22 *= s;			m23 *= s;
		m31 *= s;			m32 *= s;			m33 *= s;
		return *this;
	}

	inline _Mat33& operator /= (Real s) {
		s = numeric_const<Real>::ONE / s;
		m11 *= s;			m12 *= s;			m13 *= s;
		m21 *= s;			m22 *= s;			m23 *= s;
		m31 *= s;			m32 *= s;			m33 *= s;
		return *this;
	}

	/** returns transpose(this)*a
	*/
	inline Vec3 operator % (const Vec3& a) const {
		Vec3 tmp;
		multiplyByTranspose(tmp, a);
		return tmp;
	}

	/** matrix vector product
	*/
	inline Vec3 operator * (const Vec3& v) const {
		Vec3 tmp;
		multiply(tmp, v);
		return tmp;
	}

	/** matrix difference
	*/
	inline _Mat33 operator - (const _Mat33& m) const {
		_Mat33 tmp;
		tmp.subtract(*this, m);
		return tmp;
	}

	/** matrix addition
	*/
	inline _Mat33 operator + (const _Mat33& m) const {
		_Mat33 tmp;
		tmp.add(*this, m);
		return tmp;
	}

	/** matrix product
	*/
	inline _Mat33 operator * (const _Mat33& m) const {
		_Mat33 tmp;
		tmp.multiply(*this, m);
		return tmp;
	}

	/** matrix scalar product
	*/
	inline _Mat33 operator * (Real s) const {
		_Mat33 tmp;
		tmp.multiply(s, *this);
		return tmp;
	}

	/** inverse (transposed)
	*/
	inline _Mat33 operator ~ () const {
		return _Mat33(m11, m21, m31, m12, m22, m32, m13, m23, m33);
	}
};

//------------------------------------------------------------------------------

/** Default type */
typedef _Mat33<Real> Mat33;

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_MAT33_H_*/
