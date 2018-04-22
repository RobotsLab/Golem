/** @file RB.h
 * 
 * Rigid body tools
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
#ifndef _GOLEM_MATH_RB_H_
#define _GOLEM_MATH_RB_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Mat34.h>
#include <Golem/Math/Quat.h>
#include <Golem/Math/Sample.h>
#include <vector>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Rigid body coordinate */
template <typename _Real> class _RBCoord {
public:
	/** Real */
	typedef _Real Real;
	/** Vec3 */
	typedef golem::_Vec3<_Real> Vec3;
	/** Mat33 */
	typedef golem::_Mat33<_Real> Mat33;
	/** Quat */
	typedef golem::_Quat<_Real> Quat;
	/** Mat34 */
	typedef golem::_Mat34<_Real> Mat34;

	typedef std::vector<_RBCoord> Seq;

	/** Linear dimensions per transformation - 3D vector */
	static const size_t LIN_N = 3;
	/** Angular dimensions per transformation - 3D rotation as quaternion */
	static const size_t ANG_N = 4;
	/** Dimensions per transformation */
	static const size_t N = LIN_N + ANG_N;

	/** Flann rigid body distance metric (Gaussian kernel) */
	class FlannDistGauss {
	public:
		typedef bool is_kdtree_distance;
		typedef Real ElementType;
		typedef Real ResultType;

		const Vec3 lin;
		const Quat ang;

		FlannDistGauss(const Vec3& lin, const Quat& ang) : lin(lin), ang(ang) {
		}
		template <typename _Iter1, typename _Iter2> Real operator() (_Iter1 a, _Iter2 b, size_t size = N, Real worst_dist = -golem::numeric_const<Real>::ONE) const {
			const Real d0 = lin[0]*golem::Math::sqr(a[0] - b[0]) + lin[1]*golem::Math::sqr(a[1] - b[1]) + lin[2]*golem::Math::sqr(a[2] - b[2]);
			const Real d1 = ang[0]*golem::Math::sqr(a[3] - b[3]) + ang[1]*golem::Math::sqr(a[4] - b[4]) + ang[2]*golem::Math::sqr(a[5] - b[5]) + ang[3]*golem::Math::sqr(a[6] - b[6]);
			const Real d2 = ang[0]*golem::Math::sqr(a[3] + b[3]) + ang[1]*golem::Math::sqr(a[4] + b[4]) + ang[2]*golem::Math::sqr(a[5] + b[5]) + ang[3]*golem::Math::sqr(a[6] + b[6]);

			return golem::numeric_const<Real>::HALF*(d0 + std::min(d1, d2));
		}
		inline Real accum_dist(const Real& a, const Real& b, int) const {
			return golem::Math::sqr(a - b);
		}
	};

	/** Flann rigid body distance metric (von Mises-Fisher) */
	class FlannDistVMF {
	public:
		typedef bool is_kdtree_distance;
		typedef Real ElementType;
		typedef Real ResultType;

		const Real lin;
		const Real ang;

		FlannDistVMF(const Real& lin, const Real& ang) : lin(lin), ang(ang) {
		}
		template <typename _Iter1, typename _Iter2> Real operator() (_Iter1 a, _Iter2 b, size_t size = N, Real worst_dist = -golem::numeric_const<Real>::ONE) const {
			const Real d0 = golem::Math::sqr(a[0] - b[0]) + golem::Math::sqr(a[1] - b[1]) + golem::Math::sqr(a[2] - b[2]);
			const Real d1 = golem::numeric_const<Real>::ONE - golem::Math::abs(a[3]*b[3] + a[4]*b[4] + a[5]*b[5] + a[6]*b[6]);
			
			return lin*d0 + ang*d1;
		}
		inline Real accum_dist(const Real& a, const Real& b, int) const {
			return golem::Math::sqr(a - b);
		}
	};

	/** Position */
	Vec3 p;
	/** Orientation */
	Quat q;

	_RBCoord() {}
	_RBCoord(const _RBCoord<golem::F32>& c) : p(c.p), q(c.q) {}
	_RBCoord(const _RBCoord<golem::F64>& c) : p(c.p), q(c.q) {}
	_RBCoord(const golem::_Vec3<golem::F32>& p, const golem::_Quat<golem::F32>& q) : p(p), q(q) {}
	_RBCoord(const golem::_Vec3<golem::F64>& p, const golem::_Quat<golem::F64>& q) : p(p), q(q) {}
	_RBCoord(const golem::_Mat34<golem::F32>& m) : p(m.p), q(m.R) {}
	_RBCoord(const golem::_Mat34<golem::F64>& m) : p(m.p), q(m.R) {}

	/** Identity transformation. */
	static inline _RBCoord identity() {
		return _RBCoord(Vec3::zero(), Quat::identity());
	}

	/** Finite? */
	bool isValid() const {
		if (!p.isValid() || !q.isValid())
			return false;
		return true;
	}

	/** Sets p & q */
	inline void set(const _RBCoord& c) {
		this->p = c.p;
		this->q = c.q;
	}
	/** Sets p & q */
	inline void set(const Vec3& p, const Quat& q) {
		this->p = p;
		this->q = q;
	}
	/** Sets zero */
	inline void setZero() {
		p.setZero();
		q.setZero();
	}
	/** Sets Id */
	inline void setId() {
		p.setZero();
		q.setId();
	}

	/** Conversion */
	Mat34 toMat34() const {
		return Mat34(Mat33(q), p);
	}
	/** Conversion */
	void fromMat34(const Mat34& m) {
		p.set(m.p);
		q.fromMat33(m.R);
	}

	/** Quaternion duality */
	inline static Quat getQuatMin(const Quat& a, const Quat& b) {
		const Quat neg(-b.q0, -b.q1, -b.q2, -b.q3);
		return a.dot(neg) > a.dot(b) ? neg : b;
	}

	/** a = q * b + p */
	inline void multiply(Vec3& a, const Vec3& b) const {
		q.multiply(a, b);
		a.add(a, p);
	}
	/** this = a * b: [aq, ap] * [bq, bp] = [aq * bq, aq * bp + ap] */
	inline void multiply(const _RBCoord& a, const _RBCoord& b) {
		Vec3 tmp;
		a.q.multiply(tmp, b.p);
		p.add(tmp, a.p);
		q.multiply(a.q, b.q);
	}
	/** this = inverse(c): [ inv(q) , inv(q) * -p ] */
	inline void setInverse(const _RBCoord& c) {
		q.set(c.q.q0, -c.q.q1, -c.q.q2, -c.q.q3);
		p.multiply(-golem::numeric_const<Real>::ONE, c.p);
		q.multiply(p, p);
	}
	/** SE(3) interpolation */
	inline void interpolate(const _RBCoord& a, const _RBCoord& b, Real s) {
		// linear interpolation
		p.interpolate(a.p, b.p, s);
		// angular interpolation: account for quaternion dual cover of SO(3)
		q.slerp(a.q, getQuatMin(a.q, b.q), s);
	}

	/** operator wrapper for multiply */
	inline _RBCoord operator * (const _RBCoord& b) const {
		_RBCoord a;
		a.multiply(*this, b);
		return a;
	}
	/** operator wrapper for multiply */
	inline _RBCoord& operator *= (const Real s) {
		p *= s;
		q *= s;
		return *this;
	}

	/** addition */
	inline _RBCoord& operator += (const _RBCoord& a) {
		p += a.p;
		q += getQuatMin(q, a.q);
		return *this;
	}
	/** addition */
	inline _RBCoord operator + (const _RBCoord& a) const {
		//_RBCoord c(*this);
		//c += a;
		_RBCoord c;
		c.p = p + a.p;
		c.q = q + getQuatMin(q, a.q);
		return c;
	}

	/** Data size. */
	static inline size_t size() {
		return 7;
	}

	/** Access to data. */
	inline Real* data() {
		return (Real*)&p;
	}
	/** Access to data. */
	inline const Real* data() const {
		return (const Real*)&p;
	}

	/** Access coordinates as an array. */
	inline Real& operator [] (size_t idx) {
		return data()[idx];
	}
	/** Access coordinates as an array. */
	inline const Real& operator [] (size_t idx) const {
		return data()[idx];
	}
};

//------------------------------------------------------------------------------

/** Default type */
typedef _RBCoord<golem::Real> RBCoord;

//------------------------------------------------------------------------------

/** Rigid body distance */
template <typename _Real> class _RBDist {
public:
	/** Real */
	typedef _Real Real;
	/** RBDist */
	typedef _RBDist<_Real> RBDist;
	
	typedef std::vector<_RBDist> Seq;

	/** Linear */
	Real lin;
	/** Angular */
	Real ang;

	_RBDist(const _RBDist<golem::F32>& d) : lin((Real)d.lin), ang((Real)d.ang) {
	}
	_RBDist(const _RBDist<golem::F64>& d) : lin((Real)d.lin), ang((Real)d.ang) {
	}

	_RBDist(Real lin = golem::numeric_const<Real>::ZERO, Real ang = golem::numeric_const<Real>::ZERO) {
		set(lin, ang);
	}
	_RBDist(const RBCoord& c1, const RBCoord& c2) {
		set(c1, c2);
	}

	void set(Real lin = golem::numeric_const<Real>::ZERO, Real ang = golem::numeric_const<Real>::ZERO) {
		this->lin = lin;
		this->ang = ang;
	}
	void set(const _RBDist& d) {
		this->lin = d.lin;
		this->ang = d.ang;
	}
	void setSqr(const _RBDist& d) {
		this->lin = golem::Math::sqr(d.lin);
		this->ang = golem::Math::sqr(d.ang);
	}
	void setSqrt(const _RBDist& d) {
		this->lin = golem::Math::sqrt(golem::Math::abs(d.lin));
		this->ang = golem::Math::sqrt(golem::Math::abs(d.ang));
	}
	void setInv(const _RBDist& d) {
		this->lin = golem::numeric_const<Real>::ONE / d.lin;
		this->ang = golem::numeric_const<Real>::ONE / d.ang;
	}

	_RBDist getSqr() const {
		return _RBDist(golem::Math::sqr(lin), golem::Math::sqr(ang));
	}
	_RBDist getSqrt() const {
		return _RBDist(golem::Math::sqrt(golem::Math::abs(lin)), golem::Math::sqrt(golem::Math::abs(ang)));
	}

	void set(const RBCoord& c1, const RBCoord& c2) {
		this->lin = c1.p.distanceSqr(c2.p);
		this->ang = c1.q.distance(c2.q);
	}
	void setSqrt(const RBCoord& c1, const RBCoord& c2) {
		this->lin = c1.p.distance(c2.p);
		this->ang = golem::Math::sqrt(golem::Math::abs(c1.q.distance(c2.q)));
	}

	bool isValid() const {
		if (golem::Math::isNegative(lin) || golem::Math::isNegative(ang))
			return false;
		return true;
	}
	bool isPositive() const {
		return golem::Math::isPositive(lin) && golem::Math::isPositive(ang);
	}

	void add(const _RBDist& a, const _RBDist& b) {
		lin = a.lin + b.lin;
		ang = a.ang + b.ang;
	}
	void multiply(Real s, const _RBDist& a) {
		lin = s * a.lin;
		ang = s * a.ang;
	}
	void multiply(const _RBDist& a, const _RBDist& b) {
		lin = a.lin * b.lin;
		ang = a.ang * b.ang;
	}
	Real dot(const _RBDist& d) const {
		return lin*d.lin + ang*d.ang;
	}

	/** operator wrapper for amultiply */
	inline _RBDist operator * (const _RBDist& b) const {
		_RBDist a;
		a.multiply(*this, b);
		return a;
	}
};

//------------------------------------------------------------------------------

/** Default type */
typedef _RBDist<golem::Real> RBDist;

//------------------------------------------------------------------------------

/** Rigid body distance */
template <typename _Real> class _RBCoordDist {
public:
	/** Real */
	typedef _Real Real;
	/** RBDist */
	typedef _RBDist<_Real> RBDist;
	/** RBCoord */
	typedef _RBCoord<_Real> RBCoord;

	/** Covariance (sampling) */
	RBDist cov;
	/** Std deviation (sampling) */
	RBDist covSqrt;
	
	/** Covariance inverse/distance (evaluation) */
	RBDist covInv;
	/** Covariance inverse/distance */
	RBDist covSqrtInv;

	/** Distance maximum, covariance units (evaluation) */
	RBDist distMax;

	/** No initialisation */
	_RBCoordDist()
	{}
	/** Copying */
	_RBCoordDist(const _RBCoordDist<golem::F32>& k) :
		cov(k.cov), covSqrt(k.covSqrt), covInv(k.covInv), covSqrtInv(k.covSqrtInv), distMax(k.distMax)
	{}
	/** Copying */
	_RBCoordDist(const _RBCoordDist<golem::F64>& k) :
		cov(k.cov), covSqrt(k.covSqrt), covInv(k.covInv), covSqrtInv(k.covSqrtInv), distMax(k.distMax)
	{}
	/** From covariance and normalised distance (in covariance units) */
	_RBCoordDist(const RBDist& cov, const RBDist& distMax) {
		setCov(cov);
		setDistMax(distMax);
	}

	/** From covariance */
	inline void setCov(const RBDist& cov) {
		this->cov.set(cov);
		this->covSqrt.setSqrt(this->cov);
		this->covInv.setInv(this->cov);
		this->covSqrtInv.setInv(this->covSqrt);
	}
	/** From covariance inverse */
	inline void setCovInv(const RBDist& covInv) {
		this->cov.setInv(covInv);
		this->covSqrt.setSqrt(this->cov);
		this->covInv.set(covInv);
		this->covSqrtInv.setInv(this->covSqrt);
	}
	/** From distance */
	inline void setDistMax(const RBDist& distMax) {
		this->distMax = distMax;
	}
};

//------------------------------------------------------------------------------

/** Default type */
typedef _RBCoordDist<golem::Real> RBCoordDist;

//------------------------------------------------------------------------------

/** Rigid body distance with collisions */
template <typename _Real> class _RBDistEx : public _RBDist<_Real> {
public:
	/** Real */
	typedef _Real Real;
	/** RBCoord */
	typedef _RBCoord<_Real> RBCoord;
	/** RBDist */
	typedef _RBDist<_Real> RBDist;

	typedef std::vector<_RBDistEx> Seq;

	/** Collision */
	bool collision;

	_RBDistEx(const _RBDist<golem::F32>& rbdist, bool collision = false) : RBDist(rbdist), collision(collision) {
	}
	_RBDistEx(const _RBDist<golem::F64>& rbdist, bool collision = false) : RBDist(rbdist), collision(collision) {
	}

	_RBDistEx(Real lin = golem::numeric_const<Real>::ZERO, Real ang = golem::numeric_const<Real>::ZERO, bool collision = false) : RBDist(lin, ang), collision(collision) {
	}

	void set(Real lin = golem::numeric_const<Real>::ZERO, Real ang = golem::numeric_const<Real>::ZERO, bool collision = false) {
		this->lin = lin;
		this->ang = ang;
		this->collision = collision;
	}
	void set(const RBDist& rbdist, bool collision = false) {
		set(rbdist.lin, rbdist.ang);
		this->collision = collision;
	}
	
	Real dot(const _RBDistEx& d, Real c = golem::numeric_const<Real>::MAX) const {
		return collision && d.collision ? c : this->lin*d.lin + this->ang*d.ang;
	}
	void add(const _RBDistEx& a, const _RBDistEx& b) {
		this->lin = a.lin + b.lin;
		this->ang = a.ang + b.ang;
		collision = a.collision || b.collision;
	}
};

//------------------------------------------------------------------------------

/** Default type */
typedef _RBDistEx<golem::Real> RBDistEx;

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_MATH_RB_H_*/
