/** @file Mat34.h
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
#ifndef _GOLEM_MATH_MAT34_H_
#define _GOLEM_MATH_MAT34_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Mat33.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

template <typename _Mat34, typename _Twist, typename _Real> inline void TwistToMat34(_Mat34& m, const _Twist& t, _Real theta) {
	typedef _Real Real;
	typedef typename _Mat34::Vec3 Vec3;
	typedef typename _Mat34::Mat33 Mat33;

	if (t.w.isZero()) {
		// [ Id, v * theta ]
		m.R.setId();
		m.p.multiply(theta, t.v);
	}
	else {
		// [ exp(w^ theta), [Id - exp(w^ theta)] w^ v + w wT v * theta ]
		const Real c1 = t.w.v2*t.v.v3 - t.w.v3*t.v.v2;
		const Real c2 = t.w.v3*t.v.v1 - t.w.v1*t.v.v3;
		const Real c3 = t.w.v1*t.v.v2 - t.w.v2*t.v.v1;

		const Real s = t.w.dot(t.v) * theta;

		m.R.fromAngleAxis(theta, t.w);

		m.p.v1 = (numeric_const<Real>::ONE - m.R.m11) * c1 - m.R.m12 * c2 - m.R.m13 * c3 + t.w.v1 * s;
		m.p.v2 = -m.R.m21 * c1 + (numeric_const<Real>::ONE - m.R.m22) * c2 - m.R.m23 * c3 + t.w.v2 * s;
		m.p.v3 = -m.R.m31 * c1 - m.R.m32 * c2 + (numeric_const<Real>::ONE - m.R.m33) * c3 + t.w.v3 * s;
	}
}

template <typename _Mat34, typename _Twist, typename _Real> inline void Mat34ToTwist(_Twist& t, _Real& theta, const _Mat34& m) {
	typedef _Real Real;
	typedef typename _Mat34::Vec3 Vec3;
	typedef typename _Mat34::Mat33 Mat33;

	m.R.toAngleAxis(theta, t.w);

	if (Math::abs(theta) < numeric_const<Real>::EPS) { // see Mat33::toAngleAxis()
		// no rotation
		theta = m.p.magnitude();

		if (Math::abs(theta) > numeric_const<Real>::EPS)
			// no translation
			t.v.multiply(numeric_const<Real>::ONE / theta, m.p);
		else
			t.v.setZero();

		t.w.setZero();
	}
	else {
		// A * v = p => v = inv(A) * p, where A := [Id - exp(w^ theta)] w^ + w wT * theta
		// A is nonsingular for all theta E (0, 2*PI), because matrices which comprise A
		// have mutually orthogonal null spaces for theta != 0 and w != 0 (R != Id)
		Mat33 temp;

		temp.m11 = m.R.m13*t.w.v2 - m.R.m12*t.w.v3 + t.w.v1*t.w.v1*theta;
		temp.m12 = -m.R.m13*t.w.v1 - (numeric_const<Real>::ONE - m.R.m11)*t.w.v3 + t.w.v1*t.w.v2*theta;
		temp.m13 = (numeric_const<Real>::ONE - m.R.m11)*t.w.v2 + m.R.m12*t.w.v1 + t.w.v1*t.w.v3*theta;

		temp.m21 = (numeric_const<Real>::ONE - m.R.m22)*t.w.v3 + m.R.m23*t.w.v2 + t.w.v2*t.w.v1*theta;
		temp.m22 = m.R.m21*t.w.v3 - m.R.m23*t.w.v1 + t.w.v2*t.w.v2*theta;
		temp.m23 = -m.R.m21*t.w.v2 - (numeric_const<Real>::ONE - m.R.m22)*t.w.v1 + t.w.v2*t.w.v3*theta;

		temp.m31 = -m.R.m32*t.w.v3 - (numeric_const<Real>::ONE - m.R.m33)*t.w.v2 + t.w.v3*t.w.v1*theta;
		temp.m32 = (numeric_const<Real>::ONE - m.R.m33)*t.w.v1 + m.R.m31*t.w.v3 + t.w.v3*t.w.v2*theta;
		temp.m33 = m.R.m32*t.w.v1 - m.R.m31*t.w.v2 + t.w.v3*t.w.v3*theta;

		temp.multiplyByTranspose(t.v, m.p);
	}
}

// | R  p^R || v |   | Rv + p^Rw |
// | 0  R   || w | = | Rw        |
template <typename _Mat34, typename _Twist> inline void AdjointTransform(_Twist& dst, const _Twist& t, const _Mat34& m) {
	typedef typename _Mat34::Real Real;
	typedef typename _Mat34::Vec3 Vec3;
	typedef typename _Mat34::Mat33 Mat33;

	//Mat33 S;
	//S.axisToSkew(m.p);
	//Vec3 tmp;
	//m.R.multiply(tmp, t.w);
	//S.multiply(tmp, tmp);

	//m.R.multiply(dst.v, t.v);
	//dst.v.add(tmp, dst.v);
	//m.R.multiply(dst.w, t.w);

	const Vec3 p(
		m.R.m11*t.w.v1 + m.R.m12*t.w.v2 + m.R.m13*t.w.v3,
		m.R.m21*t.w.v1 + m.R.m22*t.w.v2 + m.R.m23*t.w.v3,
		m.R.m31*t.w.v1 + m.R.m32*t.w.v2 + m.R.m33*t.w.v3
	);
	dst.v.set(
		m.R.m11*t.v.v1 + m.R.m12*t.v.v2 + m.R.m13*t.v.v3 + (m.p.v2*p.v3 - m.p.v3*p.v2),
		m.R.m21*t.v.v1 + m.R.m22*t.v.v2 + m.R.m23*t.v.v3 + (m.p.v3*p.v1 - m.p.v1*p.v3),
		m.R.m31*t.v.v1 + m.R.m32*t.v.v2 + m.R.m33*t.v.v3 + (m.p.v1*p.v2 - m.p.v2*p.v1)
	);
	dst.w.set(
		m.R.m11*t.w.v1 + m.R.m12*t.w.v2 + m.R.m13*t.w.v3,
		m.R.m21*t.w.v1 + m.R.m22*t.w.v2 + m.R.m23*t.w.v3,
		m.R.m31*t.w.v1 + m.R.m32*t.w.v2 + m.R.m33*t.w.v3
	);
}

// | RT  -RTp^ || v |   | RT*v - RTp^*w |
// | 0    RT   || w | = | RT*w          |
template <typename _Mat34, typename _Twist> inline void AdjointInverseTransform(_Twist& dst, const _Twist& t, const _Mat34& m) {
	typedef typename _Mat34::Real Real;
	typedef typename _Mat34::Vec3 Vec3;
	typedef typename _Mat34::Mat33 Mat33;

	//Mat33 S;
	//S.axisToSkew(m.p);
	//Vec3 tmp;
	//S.multiply(tmp, t.w);
	//m.R.multiplyByTranspose(tmp, tmp);

	//m.R.multiplyByTranspose(dst.v, t.v);
	//dst.v.multiplyAdd(-numeric_const<Real>::ONE, tmp, dst.v);
	//m.R.multiplyByTranspose(dst.w, t.w);

	const Vec3 p(
		m.p.v2*t.w.v3 - m.p.v3*t.w.v2,
		m.p.v3*t.w.v1 - m.p.v1*t.w.v3,
		m.p.v1*t.w.v2 - m.p.v2*t.w.v1
	);
	dst.v.set(
		m.R.m11*t.v.v1 + m.R.m21*t.v.v2 + m.R.m31*t.v.v3 - (m.R.m11*p.v1 + m.R.m21*p.v2 + m.R.m31*p.v3),
		m.R.m12*t.v.v1 + m.R.m22*t.v.v2 + m.R.m32*t.v.v3 - (m.R.m12*p.v1 + m.R.m22*p.v2 + m.R.m32*p.v3),
		m.R.m13*t.v.v1 + m.R.m23*t.v.v2 + m.R.m33*t.v.v3 - (m.R.m13*p.v1 + m.R.m23*p.v2 + m.R.m33*p.v3)
	);
	dst.w.set(
		m.R.m11*t.w.v1 + m.R.m21*t.w.v2 + m.R.m31*t.w.v3,
		m.R.m12*t.w.v1 + m.R.m22*t.w.v2 + m.R.m32*t.w.v3,
		m.R.m13*t.w.v1 + m.R.m23*t.w.v2 + m.R.m33*t.w.v3
	);
}

// | RT     0  || v |   | RT*v          |
// | -RTp^  RT || w | = | RT*w - RTp^*v |
template <typename _Mat34, typename _Twist> inline void AdjointTransposedTransform(_Twist& dst, const _Twist& t, const _Mat34& m) {
	typedef typename _Mat34::Real Real;
	typedef typename _Mat34::Vec3 Vec3;
	typedef typename _Mat34::Mat33 Mat33;

	//Mat33 S;
	//S.axisToSkew(m.p);
	//Vec3 tmp;
	//S.multiply(tmp, t.v);
	//m.R.multiplyByTranspose(tmp, tmp);
	//
	//m.R.multiplyByTranspose(dst.w, t.w);
	//dst.w.multiplyAdd(-numeric_const<Real>::ONE, tmp, dst.w);
	//m.R.multiplyByTranspose(dst.v, t.v);

	const Vec3 p(
		m.p.v2*t.v.v3 - m.p.v3*t.v.v2,
		m.p.v3*t.v.v1 - m.p.v1*t.v.v3,
		m.p.v1*t.v.v2 - m.p.v2*t.v.v1
	);
	dst.v.set(
		m.R.m11*t.v.v1 + m.R.m21*t.v.v2 + m.R.m31*t.v.v3,
		m.R.m12*t.v.v1 + m.R.m22*t.v.v2 + m.R.m32*t.v.v3,
		m.R.m13*t.v.v1 + m.R.m23*t.v.v2 + m.R.m33*t.v.v3
	);
	dst.w.set(
		m.R.m11*t.w.v1 + m.R.m21*t.w.v2 + m.R.m31*t.w.v3 - (m.R.m11*p.v1 + m.R.m21*p.v2 + m.R.m31*p.v3),
		m.R.m12*t.w.v1 + m.R.m22*t.w.v2 + m.R.m32*t.w.v3 - (m.R.m12*p.v1 + m.R.m22*p.v2 + m.R.m32*p.v3),
		m.R.m13*t.w.v1 + m.R.m23*t.w.v2 + m.R.m33*t.w.v3 - (m.R.m13*p.v1 + m.R.m23*p.v2 + m.R.m33*p.v3)
	);
}

//------------------------------------------------------------------------------

/** Homogeneous representation of SE(3) rigid body transformations.
*/
template <typename _Real> class _Mat34 {
public:
	/** Real */
	typedef _Real Real;
	/** Vec3 */
	typedef _Vec3<_Real> Vec3;
	/** Mat33 */
	typedef _Mat33<_Real> Mat33;

	/** rotation matrix	*/
	Mat33 R;
	/** translation	*/
	Vec3 p;

	/** Default constructor does not do any initialisation.
	*/
	inline _Mat34() {}

	/** Creates matrix from rotation matrix and translation vector
	*/
	template <typename _Type> inline _Mat34(const _Mat33<_Type>& R, const _Vec3<_Type>& p) : R(R), p(p) {}

	/** Copy constructor.
	*/
	template <typename _Type> inline _Mat34(const _Mat34<_Type>& m) : R(m.R), p(m.p) {}

	/** Creates matrix from twist coordinates
	*/
	template <typename _Twist> inline _Mat34(const _Twist& t, Real theta) {
		TwistToMat34(*this, t, theta);
	}

	/** Static initialisation member - identity.
	*/
	static inline _Mat34 identity() {
		return _Mat34(Mat33::identity(), Vec3::zero());
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
	template <typename _Type> inline void set(const _Mat34<_Type>& m) {
		this->R.set(m.R);
		this->p.set(m.p);
	}

	/** set the matrix
	*/
	template <typename _Type> inline void set(const _Mat33<_Type>& R, const _Vec3<_Type>& p) {
		this->R.set(R);
		this->p.set(p);
	}

	/** set the matrix given a column matrix
	*/
	template <typename _Type> inline void setColumn44(const _Type m[]) {
		R.setColumn44(m);
		p.v1 = (Real)m[12];
		p.v2 = (Real)m[13];
		p.v3 = (Real)m[14];
	}

	/** retrieve the matrix in a column format
	*/
	template <typename _Type> inline void getColumn44(_Type m[]) const {
		R.getColumn44(m);
		m[12] = (_Type)p.v1;
		m[13] = (_Type)p.v2;
		m[14] = (_Type)p.v3;
		m[3] = m[7] = m[11] = numeric_const<_Type>::ZERO;
		m[15] = numeric_const<_Type>::ONE;
	}

	/** set the matrix given a row matrix.
	*/
	template <typename _Type> inline void setRow44(const _Type m[]) {
		R.setRow44(m);
		p.v1 = (Real)m[3];
		p.v2 = (Real)m[7];
		p.v3 = (Real)m[11];
	}

	/** retrieve the matrix in a row format.
	*/
	template <typename _Type> inline void getRow44(_Type m[]) const {
		R.getRow44(m);
		m[3] = (_Type)p.v1;
		m[7] = (_Type)p.v2;
		m[11] = (_Type)p.v3;
		m[12] = m[13] = m[14] = numeric_const<_Type>::ZERO;
		m[15] = numeric_const<_Type>::ONE;
	}

	inline void setZero() {
		R.setZero();
		p.setZero();
	}

	inline void setId() {
		R.setId();
		p.setZero();
	}

	/** Creates matrix from twist coordinates
	*/
	template <typename _Twist> inline void fromTwist(const _Twist& t, Real theta) {
		TwistToMat34(*this, t, theta);
	}

	/** Creates twist from matrix
	*/
	template <typename _Twist> inline void toTwist(_Twist& t, Real& theta) const {
		Mat34ToTwist(t, theta, *this);
	}

	/** Adjoint transformation.
	*/
	template <typename _Twist> inline void adjointTransform(_Twist& dst, const _Twist& t) const {
		AdjointTransform(dst, t, *this);
	}

	/** Inverse adjoint transformation.
	*/
	template <typename _Twist> inline void adjointInverseTransform(_Twist& dst, const _Twist& t) const {
		AdjointInverseTransform(dst, t, *this);
	}

	/** Transposed adjoint transformation.
	*/
	template <typename _Twist> inline void adjointTransposedTransform(_Twist& dst, const _Twist& t) const {
		AdjointTransposedTransform(dst, t, *this);
	}

	/** Creates special Euklidean SE(3) matrix from twist (wedge operator ^).
	*/
	template <typename _Twist> inline void twistToSpecial(const _Twist &twist) {
		R.axisToSkew(twist.w);
		p = twist.v;
	}

	/** Creates twist from special Euklidean SE(3) matrix (vee operator v).
	*/
	template <typename _Twist> inline void specialToTwist(_Twist &twist) const {
		R.skewToAxis(twist.w);
		twist.v = p;
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
	inline bool equals(const _Mat34& m, Real epsilon) const {
		return p.equals(m.p, epsilon) && R.equals(m.R, epsilon);
	}

	/** this = inverse(m): [ inv(R) , inv(R) * -p ].
	Returns false if singular (i.e. if no inverse exists), setting dest to identity.
	*/
	inline bool setInverse(const _Mat34& m) {
		if (!R.setInverse(m.R))
			return false;
		p.multiply(-numeric_const<Real>::ONE, m.p);
		((const Mat33&)R).multiply(p, p); 
		return true;
	}

	/** this = inverse(m): [ inv(R) , inv(R) * -p ], for orthonormal m: [ RT , RT * -p ].
	*/
	inline void setInverseRT(const _Mat34& m) {
		R.setTransposed(m.R);
		p.multiply(-numeric_const<Real>::ONE, m.p);
		((const Mat33&)R).multiply(p, p);
	}

	/** a = this * b
	*/
	inline void multiply(Vec3& a, const Vec3& b) const {
		// a = R * b + p;
		R.multiply(a, b);
		a.add(a, p);
	}

	/** this = a * b: [aR, ap] * [bR, bp] = [aR * bR, aR * bp + ap].
	*/
	inline void multiply(const _Mat34& a, const _Mat34& b) {
		Vec3 tmp;

		a.R.multiply(tmp, b.p);
		p.add(tmp, a.p);
		R.multiply(a.R, b.R);
	}

	/** a = inverse(this) * b, assumes R is rotation matrix
	*/
	inline void multiplyByInverseRT(Vec3& a, const Vec3& b) const {
		// b = RT * a + p => a = RT * b - RT * p = RT * (b - p)
		a.subtract(b, p);
		R.multiplyByTranspose(a, a);
	}

	/**	Assignment operator.
	*/
	template <typename _Type> inline const _Mat34& operator = (const _Mat34<_Type> &m) {
		set(m);
		return *this;
	}

	/** operator wrapper for multiply
	*/
	inline Vec3 operator * (const Vec3& a) const {
		Vec3 tmp;
		multiply(tmp, a);
		return tmp;
	}

	/** operator wrapper for multiply
	*/
	inline _Mat34 operator * (const _Mat34& b) const {
		_Mat34 a;
		a.multiply(*this, b);
		return a;
	}

	/** operator wrapper for multiplyByInverseRT
	*/
	inline Vec3 operator % (const Vec3& a) const {
		Vec3 tmp;
		multiplyByInverseRT(tmp, a);
		return tmp;
	}

	/** inverse (transposed)
	*/
	inline _Mat34 operator ~ () const {
		//_Mat34 m;
		//m.setInverseRT(*this);
		_Mat34 m;
		m.R = ~R;
		m.p = m.R * ~p;
		return m;
	}
};

//------------------------------------------------------------------------------

/** Default type */
typedef _Mat34<Real> Mat34;

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_MAT34_H_*/
