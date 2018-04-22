/** @file Collision.h
 *
 * Collision model
 *
 * @author Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_CONTACT_COLLISION_H_
#define _GOLEM_CONTACT_COLLISION_H_

//------------------------------------------------------------------------------

#include <Golem/Contact/Manipulator.h>
#include <Golem/Plugin/Point.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Collision model */
class Collision {
public:
	typedef golem::shared_ptr<Collision> Ptr;
	typedef std::vector<Collision::Ptr> Seq;

	/** Bounds */
	template <typename _Real, typename _RealEval> class _Bounds {
	public:
		typedef _Real Real;
		typedef _RealEval RealEval;
		typedef golem::_Vec3<_Real> Vec3;
		typedef golem::_Mat33<_Real> Mat33;
		typedef golem::_Mat34<_Real> Mat34;
		typedef std::vector<Vec3> Vec3Seq;

		//typedef std::vector<_Bounds> Seq;
		typedef golem::ScalarCoord<_Bounds, golem::Configspace> Coord;

		/** Surface */
		struct Surface {
			typedef std::vector<Surface> Seq;
			typedef std::vector<Seq> SeqSeq;
			Vec3 point;
			Vec3 normal;
		};
		/** Triangle */
		struct Triangle : public Surface {
			typedef std::vector<Triangle> Seq;
			typedef std::vector<Seq> SeqSeq;
			Real distance;
		};

		/** Create bounds from convex meshes */
		inline void create(const golem::Bounds::Seq& bounds, golem::Real offset) {
			for (size_t i = 0; i < bounds.size(); ++i) {
				const golem::BoundingConvexMesh* mesh = dynamic_cast<const golem::BoundingConvexMesh*>(bounds[i].get());
				if (mesh != nullptr) {
					surfaces.resize(surfaces.size() + 1);
					triangles.resize(triangles.size() + 1);
					surfaces.back().resize(mesh->getTriangles().size());
					triangles.back().resize(mesh->getTriangles().size());
					for (size_t j = 0; j < mesh->getTriangles().size(); ++j) {
						triangles.back()[j].normal = surfaces.back()[j].normal = Vec3(mesh->getNormals()[j]);
						surfaces.back()[j].point = Vec3(mesh->getVertices()[mesh->getTriangles()[j].t1]); // e.g. first triangle
						//surfaces.back()[j].point = (mesh->getVertices()[mesh->getTriangles()[j].t1] + mesh->getVertices()[mesh->getTriangles()[j].t2] + mesh->getVertices()[mesh->getTriangles()[j].t3])/Real(3.0); // centroid
						triangles.back()[j].distance = Real(mesh->getDistances()[j]);
					}
				}
			}
		}

		/** Pose */
		static inline void setPose(const Mat34& pose, const Surface& surface, Triangle& triangle) {
			pose.multiply(triangle.point, surface.point);
			pose.R.multiply(triangle.normal, surface.normal);
			triangle.distance = triangle.normal.dot(triangle.point);
		}
		/** Pose */
		static inline void setPose(const Mat34& pose, const typename Surface::Seq& surfaces, typename Triangle::Seq& triangles) {
			triangles.resize(surfaces.size());
			for (size_t i = 0; i < triangles.size(); ++i)
				setPose(pose, surfaces[i], triangles[i]);
		}
		/** Pose */
		inline void setPose(const Mat34& pose) {
			for (size_t i = 0; i < triangles.size(); ++i)
				setPose(pose, surfaces[i], triangles[i]);
		}

		/** Penetration depth of a given point */
		static inline Real getDepth(const typename Triangle::Seq& triangles, const Vec3& point, Real offset) {
			Real depth = golem::numeric_const<Real>::MAX;
			for (typename Triangle::Seq::const_iterator i = triangles.begin(); i != triangles.end(); ++i) {
				const Real d = i->distance + offset - i->normal.dot(point);
				if (d < golem::numeric_const<Real>::ZERO) // no collision
					return d;
				if (depth > d) // search for minimum
					depth = d;
			}
			return depth;
		}
		/** Penetration depth of a given point, zero if none */
		inline Real getDepth(const Vec3& point, Real offset) const {
			Real depth = golem::numeric_const<Real>::ZERO; // if no bounds or collisions, no effect
			for (typename Triangle::SeqSeq::const_iterator i = triangles.begin(); i != triangles.end(); ++i) {
				const Real d = getDepth(*i, point, offset);
				if (depth < d) // search for maximum depth (can collide only with one mesh within bounds)
					depth = d;
			}
			return depth;
		}


		/** Collision likelihood model */
		template <typename _Ptr> inline void evaluate(_Ptr begin, _Ptr end, _RealEval depthOffset, _RealEval depthStdDev, _RealEval& eval, _RealEval& depth, size_t& collisions) const {
			for (_Ptr i = begin; i < end; ++i) {
				const Real pointDepth = getDepth(*i, depthOffset);
				if (pointDepth > golem::numeric_const<Real>::ZERO) {
					const _RealEval pointEval = golem::Math::exp(depthStdDev*_RealEval(pointDepth)) - golem::numeric_const<Real>::ONE;
					eval += pointEval;
					depth += pointDepth;
					++collisions;
				}
			}
		}

		/** Empty */
		inline bool empty() const {
			return surfaces.empty();
		}

		/** Triangles */
		inline const typename Triangle::SeqSeq& getTriangles() const {
			return triangles;
		}
		/** Surfaces */
		inline const typename Surface::SeqSeq& getSurfaces() const {
			return surfaces;
		}

	private:
		/** Triangles */
		typename Triangle::SeqSeq triangles;
		/** Surfaces */
		typename Surface::SeqSeq surfaces;
	};

	/** Bounds */
	typedef _Bounds<golem::F32, golem::F32> Bounds;

	/** Collision waypoint */
	class Waypoint {
	public:
		typedef std::vector<Waypoint> Seq;

		/** Distance offset controls expansion(>0) or contraction (<0) of bounds in [m] */
		golem::Real depthOffset;
		/** Distance standard deviation controls the rate of penetration costs: cost = exp(depthStdDev*penetration) - 1, for penetration > 0, 0 otherwise */
		golem::Real depthStdDev;
		/** Path distance */
		golem::Real pathDist;
		/** Likelihood multiplier */
		golem::Real weight;
		/** Number of points */
		golem::U32 points;

		/** Constructs description object */
		Waypoint() {
			Waypoint::setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			depthOffset = golem::Real(0.0);
			depthStdDev = golem::Real(10.0);
			pathDist = golem::Real(0.0);
			weight = golem::Real(1.0);
			points = -1; // all
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(golem::Math::isFinite(depthOffset), ac, "depthOffset: invalid");
			Assert::valid(depthStdDev > golem::REAL_EPS, ac, "depthStdDev: < eps");
			Assert::valid(golem::Math::isFinite(pathDist), ac, "pathDist: invalid");
			Assert::valid(weight >= golem::REAL_ZERO, ac, "weight: < 0");
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Collision description */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Surface distance offset */
		golem::Real offset;
		/** Collision waypoints */
		Waypoint::Seq waypoints;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Nothing to do here */
		virtual ~Desc() {}
		/** Creates the object from the description. */
		virtual Collision::Ptr create(const Manipulator& manipulator) const {
			return Collision::Ptr(new Collision(manipulator, *this));
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			offset = golem::REAL_ZERO;
			waypoints.clear();
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Assert::valid(golem::Math::isFinite(offset), ac, "offset: invalid");
			Assert::valid(!waypoints.empty(), ac, "waypoints: empty");
			for (Waypoint::Seq::const_iterator i = waypoints.begin(); i != waypoints.end(); ++i) {
				i->assertValid(Assert::Context(ac, "waypoints[]."));
			}
		}
		/** Load descritpion from xml context. */
		virtual void load(const golem::XMLContext* xmlcontext);
	};

	/** Create from points */
	virtual void create(golem::Rand& rand, const Bounds::Vec3Seq& points);
	/** Create from Point3D interface */
	virtual void create(golem::Rand& rand, const data::Point3D& points);

	/** Collision likelihood estimation at a given waypoint */
	virtual golem::Real evaluate(const Waypoint& waypoint, const Manipulator::Config& config, bool debug = false);

	/** Collision likelihood estimation on a given path */
	virtual golem::Real evaluate(const Manipulator::Waypoint::Seq& path, bool debug = false);

	/** Joints bounds */
	inline const Bounds::Coord& getJointBounds() const {
		return jointBounds;
	}
	/** Base bounds */
	inline const Bounds& getBaseBounds() const {
		return baseBounds;
	}

	/** Points */
	inline const Bounds::Vec3Seq& getPoints() const {
		return points;
	}
	/** Points */
	inline void setPoints(const Bounds::Vec3Seq& points) {
		this->points = points;
	}

	/** Description */
	const Desc& getDesc() const {
		return desc;
	}

protected:
	/** Manipulator */
	const Manipulator& manipulator;
	/** Description */
	const Desc desc;

	/** Joints */
	Bounds::Coord jointBounds;
	/** Base */
	Bounds baseBounds;
	/** Points */
	Bounds::Vec3Seq points;

	/** Create */
	Collision(const Manipulator& manipulator, const Desc& desc);
};

void XMLData(Collision::Waypoint& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CONTACT_COLLISION_H_*/
