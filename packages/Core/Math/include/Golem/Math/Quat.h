/** @file Quat.h
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
#ifndef _GOLEM_MATH_QUAT_H_
#define _GOLEM_MATH_QUAT_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Mat33.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Quaternion class Q = q0 + q1*i + q2*j + q3*k
*/
template <typename _Real> class _Quat {
public:
	/** Real */
	typedef _Real Real;
	/** Vec3 */
	typedef _Vec3<_Real> Vec3;
	/** Mat33 */
	typedef _Mat33<_Real> Mat33;

	/** q0 - scalar component; q1, q2, q3 - vector components */
	union {
		struct {
			Real w, x, y, z;
		};
		struct {
			Real q0, q1, q2, q3;
		};
		Real q[4];
	};

	/** Default constructor does not do any initialisation.
	*/
	inline _Quat() {}

	/** Initialise with q0, q1, q2, q3.
	*/
	template <typename _Type> inline _Quat(_Type q0, _Type q1, _Type q2, _Type q3) : q0((Real)q0), q1((Real)q1), q2((Real)q2), q3((Real)q3) {}

	/** Copy constructor.
	*/
	template <typename _Type> inline _Quat(const _Quat<_Type>& q) : q0((Real)q.q0), q1((Real)q.q1), q2((Real)q.q2), q3((Real)q.q3) {}

	/** Copies elements from v, and scalar from q0 (defaults to 0).
	*/
	inline explicit _Quat(const Vec3& v, Real q0 = numeric_const<Real>::ZERO) : q0(q0), q1(v.v1), q2(v.v2), q3(v.v3) {}

	/** Creates quaternion from rotation around axis for normalised axis.
	*/
	inline _Quat(Real angle, const Vec3& axis) {
		fromAngleAxis(angle, axis);
	}

	/** Creates from rotation matrix.
	*	@param	m	rotation matrix to extract quaterion from.
	*/
	inline _Quat(const Mat33 &m) {
		Mat33ToQuat(*this, m);
	}

	/** Static initialisation member - zero.
	*/
	static inline _Quat zero() {
		return _Quat(numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO);
	}

	/** Static initialisation member - identity.
	*/
	static inline _Quat identity() {
		return _Quat(numeric_const<Real>::ONE, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO, numeric_const<Real>::ZERO);
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
		return q;
	}
	inline const Real *data() const {
		return q;
	}

	/** Set the members of the quaterion
	*/
	template <typename _Type> inline void set(const _Quat<_Type>& q) {
		this->q0 = (Real)q.q0;
		this->q1 = (Real)q.q1;
		this->q2 = (Real)q.q2;
		this->q3 = (Real)q.q3;
	}

	/** Set the members of the quaterion
	*/
	template <typename _Type> inline void set(_Type q0, _Type q1, _Type q2, _Type q3) {
		this->q0 = (Real)q0;
		this->q1 = (Real)q1;
		this->q2 = (Real)q2;
		this->q3 = (Real)q3;
	}

	/** Set the members of the quaterion from array
	*/
	template <typename _Type> inline void set(const _Type q[]) {
		q0 = (Real)q[0];
		q1 = (Real)q[1];
		q2 = (Real)q[2];
		q3 = (Real)q[3];
	}

	/** Set the quaternion to the identity rotation [1,0,0,0]
	*/
	inline void setId() {
		q0 = numeric_const<Real>::ONE;
		q1 = numeric_const<Real>::ZERO;
		q2 = numeric_const<Real>::ZERO;
		q3 = numeric_const<Real>::ZERO;
	}

	/** sets the quat to Id
	*/
	inline void setZero() {
		q0 = numeric_const<Real>::ZERO;
		q1 = numeric_const<Real>::ZERO;
		q2 = numeric_const<Real>::ZERO;
		q3 = numeric_const<Real>::ZERO;
	}

	template <typename _Type> inline void get(_Type q[]) const {
		q[0] = (_Type)q0;
		q[1] = (_Type)q1;
		q[2] = (_Type)q2;
		q[3] = (_Type)q3;
	}

	/** Creates quaternion from rotation matrix.
	*/
	inline void fromMat33(const Mat33& m) {
		Mat33ToQuat(*this, m);
	}

	/** Creates rotation matrix from quaternion.
	*/
	inline void toMat33(Mat33& m) {
		QuatToMat33(m, *this);
	}

	/** Creates quaternion from rotation around axis for normalised axis.
	*/
	inline void fromAngleAxis(Real angle, const Vec3& axis) {
		Real s;
		Math::sinCos(angle * numeric_const<Real>::HALF, s, q0);
		//s /= axis.magnitude();
		q1 = axis.v1 * s;
		q2 = axis.v2 * s;
		q3 = axis.v3 * s;
	}

	/** Returns rotation angle and axis of quaternion
	*/
	inline void toAngleAxis(Real& angle, Vec3& axis) const {
		angle = Math::acos(q0) * numeric_const<Real>::TWO;
		Real s = Math::sqrt(numeric_const<Real>::ONE - q0*q0);// sin(angle/2), q0=cos(angle/2)
		if (Math::abs(s) > numeric_const<Real>::ZERO)
			axis.set(q1/s, q2/s, q3/s);
		else
			axis.setZero();
	}

	/** Gets the angle between this quat and the identity quaternion.
	*/
	inline Real getAngle() const {
		return Math::acos(q0) * numeric_const<Real>::TWO;
	}

	/** Gets the angle between this quaternion and the argument
	*/
	inline Real getAngle(const _Quat& q) const {
		return Math::acos(dot(q)) * numeric_const<Real>::TWO;
	}

	/** 4D vector length.
	*/
	inline Real magnitude() const {
		return Math::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	}

	/** squared 4D vector length.
	*/
	inline Real magnitudeSqr() const {
		return q0*q0 + q1*q1 + q2*q2 + q3*q3;
	}

	/** returns the scalar product of this and other.
	*/
	inline Real dot(const _Quat& q) const {
		return q0*q.q0 + q1*q.q1 + q2*q.q2 + q3*q.q3;
	}

	/** maps to the closest unit quaternion.
	*/
	inline Real normalise() {
		Real m = magnitude();
		
		if (m > numeric_const<Real>::ZERO) {
			const Real length = numeric_const<Real>::ONE / m;
			q0 *= length;
			q1 *= length;
			q2 *= length;
			q3 *= length;
		}

		return m;
	}

	/* assigns its own conjugate to itself.
	*/
	inline void conjugate() {
		q1 = -q1;
		q2 = -q2;
		q3 = -q3;
	}

	/** Sets this to the opposite rotation of this.
	*/
	inline void invert() {
		conjugate();
	}

	/** Sets this to the opposite rotation of this.
	*/
	inline void invertNormalise() {
		normalise();
		invert();
	}

	/** this = q * p = (q0p0 - (q * p), q0p + p0q + (q x p))
	*/
	inline void multiply(const _Quat& q, const _Quat& p) {
		Real a = q.q0*p.q0 - (q.q1*p.q1 + q.q2*p.q2 + q.q3*p.q3);
		Real b = q.q0*p.q1 + p.q0*q.q1 + (q.q2*p.q3 - p.q2*q.q3);
		Real c = q.q0*p.q2 + p.q0*q.q2 + (q.q3*p.q1 - p.q3*q.q1);
		Real d = q.q0*p.q3 + p.q0*q.q3 + (q.q1*p.q2 - p.q1*q.q2);

		q0 = a;
		q1 = b;
		q2 = c;
		q3 = d;
	}

	/** this = q * v, v is interpreted as quaternion [0, q1, q2, q3]
	*/
	inline void multiply(const _Quat& q, const Vec3& v) {
		Real a = - (q.q1*v.v1 + q.q2*v.v2 + q.q3*v.v3);
		Real b = q.q0*v.v1 + (q.q2*v.v3 - v.v2*q.q3);
		Real c = q.q0*v.v2 + (q.q3*v.v1 - v.v3*q.q1);
		Real d = q.q0*v.v3 + (q.q1*v.v2 - v.v1*q.q2);

		q0 = a;
		q1 = b;
		q2 = c;
		q3 = d;
	}

	/** a = this * b
	*/
	inline void multiply(Vec3& a, const Vec3& b) const {
		//Real m = numeric_const<Real>::ONE/magnitudeSqr();
		//_Quat inverse(q0/m, -q1/m, -q2/m, -q3/m);
		_Quat inverse(q0, -q1, -q2, -q3);

		//a = (this * b) x inverse;
		_Quat q;
		q.multiply(*this, b);
		a.v1 = q.q0*inverse.q1 + inverse.q0*q.q1 + q.q2*inverse.q3 - inverse.q2*q.q3;
		a.v2 = q.q0*inverse.q2 + inverse.q0*q.q2 + q.q3*inverse.q1 - inverse.q3*q.q1;
		a.v3 = q.q0*inverse.q3 + inverse.q0*q.q3 + q.q1*inverse.q2 - inverse.q1*q.q2;
	}

	/** a = inverse(this) * b
	*/
	inline void multiplyByInverse(Vec3& a, const Vec3& b) const {
		//Real m = numeric_const<Real>::ONE/magnitudeSqr();
		//_Quat inverse(q0/m, -q1/m, -q2/m, -q3/m);
		_Quat inverse(q0, -q1, -q2, -q3);

		//a = (inverse * b) x this;
		_Quat q;
		q.multiply(inverse, b);
		a.v1 = q.q0*q1 + q0*q.q1 + q.q2*q3 - q2*q.q3;
		a.v2 = q.q0*q2 + q0*q.q2 + q.q3*q1 - q3*q.q1;
		a.v3 = q.q0*q3 + q0*q.q3 + q.q1*q2 - q1*q.q2;
	}

	/**	Slerp - minimum torque rotation interpolation.
	*/
	inline void slerp(const _Quat& p, const _Quat& q, Real t) {
		const Real dot = p.dot(q);
		Math::clamp(dot, -numeric_const<Real>::ONE, numeric_const<Real>::ONE);
		
		const Real acos = Math::acos(dot);
		if (acos > numeric_const<Real>::ONE - numeric_const<Real>::EPS) {
			// optionally interpolate linearly
			*this = p;
			return;
		}
	
		q0 = q.q0 - dot*p.q0;
		q1 = q.q1 - dot*p.q1;
		q2 = q.q2 - dot*p.q2;
		q3 = q.q3 - dot*p.q3;
		normalise();

		Real theta = acos*t, sin, cos;
		Math::sinCos(theta, sin, cos);
		q0 = cos*p.q0 + sin*q0;
		q1 = cos*p.q1 + sin*q1;
		q2 = cos*p.q2 + sin*q2;
		q3 = cos*p.q3 + sin*q3;
	}

	/** Generates uniform random rotation.
	*	see "Uniform Random Rotations", Ken Shoemake, Graphics Gems III, pg. 124-132
	*/
	template <typename Rand> void next(const Rand &rand) {
		const Real x0 = rand.template nextUniform<Real>();
		const Real r1 = Math::sqrt(numeric_const<Real>::ONE - x0);
		const Real r2 = Math::sqrt(x0);
		const Real t1 = numeric_const<Real>::TWO_PI * rand.template nextUniform<Real>();
		const Real t2 = numeric_const<Real>::TWO_PI * rand.template nextUniform<Real>();
		const Real c1 = Math::cos(t1);
		const Real s1 = Math::sin(t1);
		const Real c2 = Math::cos(t2);
		const Real s2 = Math::sin(t2);
		
		set(c2 * r2, s1 * r1, c1 * r1, s2 * r2);
	}

	/** Generates random rotation around (1, 0, 0, 0) on 4-sphere with dispersion parameter k (1/k is analogous to sigma^2),
	*	according to a von Mises-Fisher distribution.
	*	For k = 0 the distribution is uniform, for k >> 0 the distribution is concentrated around (1, 0, 0, 0).
	*	see A.T.A. Wood., "Simulation of the von-Mises Distribution", Communications of Statistics, Simulation and Computation, 23:157{164, 1994}
	*/
	template <typename Rand> void next(const Rand &rand, const Real k) {
		const Real b = Real(0.66666666666666666667)*(Math::sqrt(Math::sqr(k) + Real(2.25)) - k); // DIM=4, 2/3*(sqrt(k^2 + 9/4) - k)
		//const Real b = Math::sqrt(Math::sqr(k) + numeric_const<Real>::ONE) - k; // DIM=3, sqrt(k^2 + 1) - k
		const Real x = (numeric_const<Real>::ONE - b)/(numeric_const<Real>::ONE + b);
		const Real c = k*x + Real(3.0)*Math::ln(numeric_const<Real>::ONE - Math::sqr(x)); // DIM=4 
		//const Real c = k*x + Real(2.0)*Math::ln(numeric_const<Real>::ONE - Math::sqr(x)); // DIM=3
		
		for (;;) {
			// beta distribution with a = b = (DIM - 1)/2 = 1.5, where DIM=4
			const Real o = numeric_const<Real>::TWO*rand.template nextUniform<Real>() - numeric_const<Real>::ONE;
			const Real p = rand.template nextUniform<Real>();
			const Real s = Math::sqr(o) + Math::sqr(p);
			if (s > numeric_const<Real>::ONE)
				continue;
			const Real z = numeric_const<Real>::HALF + o*p*Math::sqrt(numeric_const<Real>::ONE - s)/s;
			
			const Real u = rand.template nextUniform<Real>();
			const Real v = numeric_const<Real>::ONE - z*(numeric_const<Real>::ONE - b);
			const Real w = (numeric_const<Real>::ONE - z*(numeric_const<Real>::ONE + b))/v;
			const Real t = k*w + Real(3.0)*Math::ln(numeric_const<Real>::ONE - x*w) - c; // DIM=4
			//const Real t = k*w + Real(2.0)*Math::ln(numeric_const<Real>::ONE - x*w) - c; // DIM=3
			if (t < Math::ln(u))
				continue;

			const Real r = Math::sqrt(numeric_const<Real>::ONE - Math::sqr(w));
			this->w = w;
			Vec3 vec;
			vec.template next<Rand>(rand);
			this->x = r*vec.x;
			this->y = r*vec.y;
			this->z = r*vec.z;
			break;
		}
	}

	/** Quaternion distance
	*/
	inline Real distance(const _Quat& q) const {
		return numeric_const<Real>::ONE - Math::abs(dot(q));
	}

	/**	negates all the elements of the quat: q and -q represent the same rotation.
	*/
	inline void negate() {
		q0 = -q0;
		q1 = -q1;
		q2 = -q2;
		q3 = -q3;
	}

	/** Test if the quaterion is the identity rotation.
	*/
	inline bool isId() const {
		return q0 == numeric_const<Real>::ONE && Math::isZero(q1) && Math::isZero(q2) && Math::isZero(q3);
	}

	/** Test if the quaterion is zero.
	*/
	inline bool isZero() const {
		return Math::isZero(q0) && Math::isZero(q1) && Math::isZero(q2) && Math::isZero(q3);
	}

	/** Test if the quaterion is normalised.
	*/
	inline bool isNormalised() const {
		return Math::equals(magnitudeSqr(), numeric_const<Real>::ONE, numeric_const<Real>::EPS);
	}

	/** tests for positive quaternion coefficients
	*/
	inline bool isPositive() const {
		return q0 > numeric_const<Real>::ZERO && q1 > numeric_const<Real>::ZERO && q2 > numeric_const<Real>::ZERO && q3 > numeric_const<Real>::ZERO;
	}

	/** tests for negative quaternion coefficients
	*/
	inline bool isNegative() const {
		return q0 < numeric_const<Real>::ZERO && q1 < numeric_const<Real>::ZERO && q2 < numeric_const<Real>::ZERO && q3 < numeric_const<Real>::ZERO;
	}

	/** tests for finite quaternion
	*/
	inline bool isFinite() const {
		return Math::isFinite(q0) && Math::isFinite(q1) && Math::isFinite(q2) && Math::isFinite(q3);
	}

	/**	Assignment operator.
	*/
	template <typename _Type> inline _Quat& operator = (const _Quat<_Type>& q) {
		set(q);
		return *this;
	}

	/** Access the data as an array.
	*	@param	idx	Array index.
	*	@return		Array element pointed by idx.
	*/
	inline Real& operator [] (size_t idx) {
		return q[idx];
	}
	inline const Real& operator [] (size_t idx) const {
		return q[idx];
	}
	
	/** Implicitly extends vector by a 0 q0 element.
	*/
	inline _Quat& operator = (const Vec3& v) {
		q0 = numeric_const<Real>::ZERO;
		q1 = v.v1;
		q2 = v.v2;
		q3 = v.v3;
		return *this;
	}

	inline _Quat operator - () const {
		return _Quat(-q0, -q1, -q2, -q3);
	}

	inline _Quat& operator *= (const _Quat& q) {
		multiply(*this, q);
		return *this;
	}

	inline _Quat& operator += (const _Quat& q) {
		q0 += q.q0;
		q1 += q.q1;
		q2 += q.q2;
		q3 += q.q3;
		return *this;
	}

	inline _Quat& operator -= (const _Quat& q) {
		q0 -= q.q0;
		q1 -= q.q1;
		q2 -= q.q2;
		q3 -= q.q3;
		return *this;
	}

	inline _Quat& operator *= (const Real s) {
		q0 *= s;
		q1 *= s;
		q2 *= s;
		q3 *= s;
		return *this;
	}

	/** quaternion multiplication this * p = (q0p0 - (this * p), q0p + p0q + (this x p)
	*/
	inline _Quat operator * (const _Quat& p) const {
		return
			_Quat(
				q0*p.q0 - (q1*p.q1 + q2*p.q2 + q3*p.q3),
				q0*p.q1 + p.q0*q1 + (q2*p.q3 - p.q2*q3),
				q0*p.q2 + p.q0*q2 + (q3*p.q1 - p.q3*q1),
				q0*p.q3 + p.q0*q3 + (q1*p.q2 - p.q1*q2)
			);
	}
 
	/** quaternion addition
	*/
	inline _Quat operator + (const _Quat& q) const {
		return _Quat(q.q0 + q0, q.q1 + q1, q.q2 + q2, q.q3 + q3);
	}
 
	/** quaternion subtraction
	*/
	inline _Quat operator - (const _Quat& q) const {
		return _Quat(q.q0 - q0, q.q1 - q1, q.q2 - q2, q.q3 - q3);
	}
 
	/** quaternion conjugate
	*/
	inline _Quat operator ! () const {
		return _Quat(q0, -q1, -q2, -q3);
	}
};

//------------------------------------------------------------------------------

/** Default type */
typedef _Quat<Real> Quat;

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_QUAT_H_*/
