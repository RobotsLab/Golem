/** @file Triangle.h
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
#ifndef _GOLEM_MATH_TRIANGLE_H_
#define _GOLEM_MATH_TRIANGLE_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Vec3.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Triangle */
template <typename _Real> class _Triangle {
public:
	/** Real */
	typedef _Real Real;
	/** Vec3 */
	typedef golem::_Vec3<_Real> Vec3;

	typedef std::vector<_Triangle> Seq;

	Vec3 t1, t2, t3, t21, t32, t13, tn;
	Real tnl, tnd, eps;

	_Triangle() {}
	template <typename _Vec3> _Triangle(const _Vec3& _t1, const _Vec3& _t2, const _Vec3& _t3, Real eps = golem::numeric_const<Real>::EPS) :
		eps(eps), t1(_t1), t2(_t2), t3(_t3), t21(_t2 - _t1), t32(t3 - t2), t13(_t1 - _t3), tn(t13 ^ t21), tnl(tn.magnitude()), tnd(tn | t1)
	{}
	
	inline bool isZeroEps(Real a) const {
		return golem::Math::abs(a) < eps;
	}
	bool isValid() const {
		return !isZeroEps(tn.x) || !isZeroEps(tn.y) || !isZeroEps(tn.z);
	}
	/** Ray intersection */
	inline bool intersect(const Vec3& r1, const Vec3& rn, Vec3& p) const {
		const Real trd = tn | rn;
		if (isZeroEps(trd))
			return false;
		const Real s = ((tn | r1) + tnd)/trd; // TODO golem normals are inverse!
		if (s < golem::numeric_const<Real>::ZERO)
			return false;

		p.multiplyAdd(s, rn, r1);

		if ((tn | (t21 ^ (p - t1))) < golem::numeric_const<Real>::ZERO)
			return false;
		if ((tn | (t32 ^ (p - t2))) < golem::numeric_const<Real>::ZERO)
			return false;
		if ((tn | (t13 ^ (p - t3))) < golem::numeric_const<Real>::ZERO)
			return false;

		return true;
	}

	/** Vertex to point distance */
	inline static void distanceVertex(const Vec3& t, const Vec3& p, Vec3& proj, Real& dist) {
		const Real d = t.distance(p);
		if (golem::Math::abs(dist) < d)
			return;
		proj = t;
		dist = d;
	}
	/** Triangle vertices to point distance */
	inline void distanceVertex(const Vec3& p, Vec3& proj, Real& dist) const {
		distanceVertex(t1, p, proj, dist);
		distanceVertex(t2, p, proj, dist);
		distanceVertex(t3, p, proj, dist);
	}
	/** Edge to point distance */
	inline static void distanceEdge(const Vec3& t, const Vec3& tt, const Vec3& p, Vec3& proj, Real& dist) {
		Vec3 pt = p - t;
		const Real ttll = tt.magnitudeSqr();
		const Real ptd = pt | tt;
	
		if (ptd < golem::numeric_const<Real>::ZERO || ptd > ttll)
			return;

		Vec3 u;
		u.multiplyAdd(ptd/ttll, tt, t);

		const Real d = u.distance(p);
		if (d < golem::Math::abs(dist)) {
			proj = u;
			dist = d;
		}
	}
	/** Triangle edges to point distance */
	inline void distanceEdge(const Vec3& p, Vec3& proj, Real& dist) const {
		distanceEdge(t1, t21, p, proj, dist);
		distanceEdge(t2, t32, p, proj, dist);
		distanceEdge(t3, t13, p, proj, dist);
	}
	/** Triangle-point distance */
	inline void distanceTriangle(const Vec3& p, Vec3& proj, Real& dist) const {
		const Real d = ((tn | p) - tnd)/tnl;
		if (golem::Math::abs(d) > golem::Math::abs(dist))
			return;
		
		Vec3 u;
		u.multiplyAdd(-d/tnl, tn, p);
		
		const bool sign = (tn | (t21 ^ (u - t1))) < golem::numeric_const<Real>::ZERO;

		if (sign != ((tn | (t32 ^ (u - t2))) < golem::numeric_const<Real>::ZERO))
			return;
		if (sign != ((tn | (t13 ^ (u - t3))) < golem::numeric_const<Real>::ZERO))
			return;
		
		proj = u;
		dist = d;
	}
	/** Triangle-point minimum distance */
	inline void distance(const Vec3& p, Vec3& proj, Real& dist) const {
		distanceVertex(p, proj, dist);
		distanceEdge(p, proj, dist);
		distanceTriangle(p, proj, dist);
	}
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------


#endif /*_GOLEM_MATH_TRIANGLE_H_*/
