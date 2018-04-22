/** @file Twist.h
 * 
 * Implementation of twists.
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
#ifndef _GOLEM_MATH_TWIST_H_
#define _GOLEM_MATH_TWIST_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Mat34.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Twist coordinates of rigid body transformation (generator of exponential mapping: se(3) -> SE(3)).
*/
template <typename _Real> class _Twist {
public:
	/** Real */
	typedef _Real Real;
	/** Vec3 */
	typedef _Vec3<_Real> Vec3;
	/** Mat33 */
	typedef _Mat33<_Real> Mat33;
	/** Mat34 */
	typedef _Mat34<_Real> Mat34;

	/** linear component */
	Vec3 v;
	/** angular component */
	Vec3 w;

	/** Default constructor does not do any initialisation.
	*/
	inline _Twist() {}

	/** Initialise with the all parameters unrolled.
	*/
	template <typename _Type> inline _Twist(_Type v1, _Type v2, _Type v3, _Type w1, _Type w2, _Type w3) : v(v1, v2, v3), w(w1, w2, w3) {}

	/** Initialise with v, w.
	*/
	template <typename _Type> inline _Twist(const _Vec3<_Type>& v, const _Vec3<_Type>& w) : v(v), w(w) {}

	/** Copy constructor.
	*/
	template <typename _Type> inline _Twist(const _Twist<_Type>& t) : v(t.v), w(t.w) {}

	/** Creates twist from screw motion.
	*/
	inline _Twist(Real h, const Vec3& anchor, const Vec3& axis) {
		fromScrew(h, anchor, axis);
	}

	/** Creates twist from homogeneous matrix.
	*	@param	m	homogeneous matrix to extract twist from.
	*/
	inline _Twist(Real& theta, const Mat34 &m) {
		Mat34ToTwist(*this, theta, m);
	}

	/** Static initialisation member - zero.
	*/
	static inline _Twist zero() {
		return _Twist(Vec3::zero(), Vec3::zero());
	}

	/** Static initialisation member - identity.
	*/
	static inline _Twist identity() {
		return zero();
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
		return v.data();
	}
	inline const Real *data() const {
		return v.data();
	}

	/** Set the members of the twist
	*/
	template <typename _Type> inline void set(const _Twist<_Type>& t) {
		this->v.set(t.v);
		this->w.set(t.w);
	}

	/** Set the members of the twist.
	*/
	template <typename _Type> inline void set(_Type v1, _Type v2, _Type v3, _Type w1, _Type w2, _Type w3) {
		this->v.set(v1, v2, v3);
		this->w.set(w1, w2, w3);
	}

	/** Set the members of the twist
	*/
	template <typename _Type> inline void set(const _Vec3<_Type>& v, const _Vec3<_Type>& w) {
		this->v.set(v);
		this->w.set(w);
	}

	/** Sets linear component v.
	*/
	template <typename _Type> inline void setV(_Type v1, _Type v2, _Type v3) {
		this->v.set(v1, v2, v3);
	}

	/** Sets linear component v.
	*/
	template <typename _Type> inline void setV(const _Vec3<_Type>& v) {
		this->v.set(v);
	}

	/** Sets angular component w.
	*/
	template <typename _Type> inline void setW(_Type w1, _Type w2, _Type w3) {
		this->w.set(w1, w2, w3);
	}

	/** Sets angular component w.
	*/
	template <typename _Type> inline void setW(const _Vec3<_Type>& w) {
		this->w.set(w);
	}

	/** Set the members of the quaterion from array
	*/
	template <typename _Type> inline void set(const _Type t[]) {
		v.setColumn3(&t[0]);
		w.setColumn3(&t[3]);
	}

	/** sets the twist to zero
	*/
	inline void setZero() {
		v.setZero();
		w.setZero();
	}

	/** Set twist to the identity transformation
	*/
	inline void setId() {
		setZero();
	}

	template <typename _Type> inline void get(_Type t[]) const {
		v.getColumn3(&t[0]);
		w.getColumn3(&t[3]);
	}

	/** Returns linear component v.
	*/
	inline const Vec3& getV() const {
		return v;
	}
	inline Vec3& getV() {
		return v;
	}

	/** Returns angular component w.
	*/
	inline const Vec3& getW() const {
		return w;
	}
	inline Vec3& getW() {
		return w;
	}

	/** Creates twist from homogeneous matrix.
	*/
	inline void fromMat34(Real& theta, const Mat34& m) {
		Mat34ToTwist(*this, theta, m);
	}

	/** Creates homogeneous matrix from twist.
	*/
	inline void toMat34(Mat34& m, Real theta) const {
		TwistToMat34(m, *this, theta);
	}

	/** Creates twist from a screw motion.
	*/
	inline void fromScrew(Real h, const Vec3& anchor, const Vec3& axis) {
		if (Math::isFinite(h)) {
			// Rotation and translation
			// [ v, w ] = [ -axis x anchor + h axis = anchor x axis + h axis, axis ]
			v.cross(anchor, axis);
			w.multiply(h, axis); // temp
			v.add(v, w);
			w = axis;
		}
		else {
			// Pure translation
			// [ v, w ] = [ axis, 0 ]
			v = axis;
			w.setZero();
		}
	}

	/** Creates screw motion from twist.
	*/
	inline void toScrew(Vec3& anchor, Vec3& axis) const {
		getAxis(anchor, axis);
	}

	/** Returns pitch of the corresponding screw motion.
	*/
	inline Real getPitch() const {
		return w.isZero() ? numeric_const<Real>::INF : w.dot(v) / w.magnitudeSqr();
	}

	/** Returns magnitude of the corresponding screw motion.
	*/
	inline Real getMagnitude() const {
		return w.isZero() ? v.magnitude() : w.magnitude();
	}

	/** Returns anchor and axis of the corresponding screw motion.
	*/
	inline void getAxis(Vec3& anchor, Vec3& axis) const {
		if (w.isZero()) {
			anchor.setZero();
			axis = v;
		}
		else {
			anchor.cross(w, v);
			anchor.multiply(numeric_const<Real>::ONE / w.magnitudeSqr(), anchor);
			axis = w;
		}
	}

	/** Adjoint transformation.
	*/
	inline void adjointTransform(const _Twist& t, const Mat34& m) {
		AdjointTransform(*this, t, m);
	}

	/** Inverse adjoint transformation.
	*/
	inline void adjointInverseTransform(const _Twist& t, const Mat34& m) {
		AdjointInverseTransform(*this, t, m);
	}

	/** Transposed adjoint transformation.
	*/
	inline void adjointTransposedTransform(const _Twist& t, const Mat34& m) {
		AdjointTransposedTransform(*this, t, m);
	}

	/** maps to the closest unit twist.
	*/
	inline Real normalise() {
		return w.isZero() ? v.normalise() : w.normalise();
	}

	/** Test if the twist is the identity transformation.
	*/
	inline bool isIdentityTransformation() const {
		return v.isZero() && w.isZero();
	}

	/** this = element wise min(this)
	*/
	inline Real min() const {
		return std::min(v.min(), w.min());
	}
	/** this = element wise min(this,other)
	*/
	inline void min(const _Twist& t) {
		v.min(t.v);
		w.min(t.w);
	}
	/** element wise min(a, b)
	*/
	inline _Twist min(const _Twist& a, const _Twist& b) {
		return _Twist(std::min(a.v.v1, b.v.v1), std::min(a.v.v2, b.v.v2), std::min(a.v.v3, b.v.v3), std::min(a.w.v1, b.w.v1), std::min(a.w.v2, b.w.v2), std::min(a.w.v3, b.w.v3));
	}

	/** this = element wise max(this)
	*/
	inline Real max() const {
		return std::max(v.max(), w.max());
	}
	/** this = element wise max(this,other)
	*/
	inline void max(const _Twist& t) {
		v.max(t.v);
		w.max(t.w);
	}
	/** element wise max(a, b)
	*/
	inline _Twist max(const _Twist& a, const _Twist& b) {
		return _Twist(std::max(a.v.v1, b.v.v1), std::max(a.v.v2, b.v.v2), std::max(a.v.v3, b.v.v3), std::max(a.w.v1, b.w.v1), std::max(a.w.v2, b.w.v2), std::max(a.w.v3, b.w.v3));
	}
	
	/** element wise abs()
	*/
	static inline _Twist abs(const _Twist& t) {
		return _Twist(Math::abs(t.v.v1), Math::abs(t.v.v2), Math::abs(t.v.v3), Math::abs(t.w.v1), Math::abs(t.w.v2), Math::abs(t.w.v3));
	}

	/** this = element wise clamp(this,other)
	*/
	inline void clamp(const _Twist& min, const _Twist& max) {
		v.clamp(min.v, max.v);
		w.clamp(min.w, max.w);
	}
	/** element wise clamp(this,other)
	*/
	inline static _Twist clamp(const _Twist& t, const _Twist& min, const _Twist& max) {
		_Twist tmp = t;
		tmp.clamp(min, max);
		return tmp;
	}

	/** this = a + b
	*/
	inline void add(const _Twist& a, const _Twist& b) {
		v.add(a.v, b.v);
		w.add(a.w, b.w);
	}

	/** this = a - b
	*/
	inline void subtract(const _Twist& a, const _Twist& b) {
		v.subtract(a.v, b.v);
		w.subtract(a.w, b.w);
	}

	/** this = s * a;
	*/
	inline void multiply(Real s, const _Twist& a) {
		v.multiply(s, a.v);
		w.multiply(s, a.w);
	}

	/** this = b * a;
	*/
	inline void arrayMultiply(const _Twist& a, const _Twist& b) {
		v.arrayMultiply(a.v, b.v);
		w.arrayMultiply(a.w, b.w);
	}

	/** this = s * a + b;
	*/
	inline void multiplyAdd(Real s, const _Twist& a, const _Twist& b) {
		v.multiplyAdd(s, a.v, b.v);
		w.multiplyAdd(s, a.w, b.w);
	}

	/** tests for exact zero twist
	*/
	inline bool isZero() const {
		return v.isZero() && w.isZero();
	}

	/** tests for positive twist
	*/
	inline bool isPositive(const Real eps = numeric_const<Real>::ZERO) const {
		return v.isPositive(eps) && w.isPositive(eps);
	}

	/** tests for exact Id twist
	*/
	inline bool isId() const {
		return isZero();
	}

	/** tests for finite twist
	*/
	inline bool isFinite() const {
		return v.isFinite() && w.isFinite();
	}

	/**	Assignment operator.
	*/
	template <typename _Type> inline _Twist& operator = (const _Twist<_Type>& t) {
		set(t);
		return *this;
	}
	
	/** Access the data as an array.
	*	@param	idx	Array index.
	*	@return		Array element pointed by idx.
	*/
	inline Real& operator [] (size_t idx) {
		return data()[idx];
	}
	inline const Real& operator [] (size_t idx) const {
		return data()[idx];
	}

	/** negation
	*/
	_Twist operator - () const {
		return _Twist(-v.v1, -v.v2, -v.v3, -w.v1, -w.v2, -w.v3);
	}
	/** twist array addition
	*/
	_Twist operator + (const _Twist& t) const {
		return _Twist(v.v1 + t.v.v1, v.v2 + t.v.v2, v.v3 + t.v.v3, w.v1 + t.w.v1, w.v2 + t.w.v2, w.v3 + t.w.v3);
	}
	/** twist array difference
	*/
	_Twist operator - (const _Twist& t) const {
		return _Twist(v.v1 - t.v.v1, v.v2 - t.v.v2, v.v3 - t.v.v3, w.v1 - t.w.v1, w.v2 - t.w.v2, w.v3 - t.w.v3);
	}
	/** scalar post-multiplication
	*/
	_Twist operator * (Real f) const {
		return _Twist(v.v1 * f, v.v2 * f, v.v3 * f, w.v1 * f, w.v2 * f, w.v3 * f);
	}
	/** twist array multiplication
	*/
	_Twist operator * (const _Twist& t) const {
		return _Twist(v.v1 * t.v.v1, v.v2 * t.v.v2, v.v3 * t.v.v3, w.v1 * t.w.v1, w.v2 * t.w.v2, w.v3 * t.w.v3);
	}
	/** scalar division
	*/
	_Twist operator / (Real f) const {
		f = golem::numeric_const<Real>::ONE / f;
		return _Twist(v.v1 * f, v.v2 * f, v.v3 * f, w.v1 * f, w.v2 * f, w.v3 * f);
	}
	/** twist array division
	*/
	_Twist operator / (const _Twist& t) const {
		return _Twist(v.v1 / t.v.v1, v.v2 / t.v.v2, v.v3 / t.v.v3, w.v1 / t.w.v1, w.v2 / t.w.v2, w.v3 / t.w.v3);
	}
	/** twist addition
	*/
	_Twist& operator += (const _Twist& t) {
		v += t.v;
		w += t.w;
		return *this;
	}
	/** twist difference
	*/
	_Twist& operator -= (const _Twist& t) {
		v -= t.v;
		w -= t.w;
		return *this;
	}
	/** scalar multiplication
	*/
	_Twist& operator *= (Real f) {
		v *= f;
		w *= f;
		return *this;
	}
	/** twist array multiplication
	*/
	_Twist& operator *= (const _Twist& t) {
		v *= t.v;
		w *= t.w;
		return *this;
	}
	/** scalar division
	*/
	_Twist& operator /= (Real f) {
		f = golem::numeric_const<Real>::ONE / f;
		v *= f;
		w *= f;
		return *this;
	}
	/** twist array division
	*/
	_Twist& operator /= (const _Twist& t) {
		v /= t.v;
		w /= t.w;
		return *this;
	}
};

//------------------------------------------------------------------------------

/** Default type */
typedef _Twist<Real> Twist;

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_TWIST_H_*/
