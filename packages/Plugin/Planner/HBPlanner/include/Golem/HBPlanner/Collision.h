//------------------------------------------------------------------------------
//               Simultaneous Perception And Manipulation (SPAM)
//------------------------------------------------------------------------------
//
// Copyright (c) 2010 University of Birmingham.
// All rights reserved.
//
// Intelligence Robot Lab.
// School of Computer Science
// University of Birmingham
// Edgbaston, Brimingham. UK.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy 
// of this software and associated documentation files (the "Software"), to deal 
// with the Software without restriction, including without limitation the 
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
// sell copies of the Software, and to permit persons to whom the Software is 
// furnished to do so, subject to the following conditions:
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimers.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimers in 
//       the documentation and/or other materials provided with the 
//       distribution.
//     * Neither the names of the Simultaneous Perception And Manipulation 
//		 (SPAM), University of Birmingham, nor the names of its contributors  
//       may be used to endorse or promote products derived from this Software 
//       without specific prior written permission.
//
// The software is provided "as is", without warranty of any kind, express or 
// implied, including but not limited to the warranties of merchantability, 
// fitness for a particular purpose and noninfringement.  In no event shall the 
// contributors or copyright holders be liable for any claim, damages or other 
// liability, whether in an action of contract, tort or otherwise, arising 
// from, out of or in connection with the software or the use of other dealings 
// with the software.
//
//------------------------------------------------------------------------------
//! @Author:   Claudio Zito
//! @Date:     27/10/2012
//------------------------------------------------------------------------------
#pragma once
#ifndef _GOLEM_HBPLANNER_HBCollision_H_
#define _GOLEM_HBPLANNER_HBCollision_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Cloud.h>
#include <Golem/Tools/Search.h>
#include <Golem/Planner/GraphPlanner/GraphPlanner.h>
#include <Golem/HBPlanner/JContact.h>

//------------------------------------------------------------------------------

namespace flann {
	template <typename T> struct L2_Simple;
};

namespace pcl {
	struct PointXYZ;
	template <typename T, typename Dist> class KdTreeFLANN;
	struct PolygonMesh;
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Performance monitor */
//#define _HBCOLLISION_PERFMON

//------------------------------------------------------------------------------

/** Forward declaration */
class Hypothesis;

//------------------------------------------------------------------------------

/** HBCollision model */
class GOLEM_LIBRARY_DECLDIR HBCollision {
public:
	friend class Hypothesis;
	typedef shared_ptr<HBCollision> Ptr;
	typedef std::vector<HBCollision::Ptr> Seq;
	typedef std::vector<NNSearch::Ptr> NNSearchPtrSeq;

	class GOLEM_LIBRARY_DECLDIR Feature {
	public:
		typedef std::vector<Feature> Seq;

		/** As real vector: two surface normals, distance between points, rgb colour */
		static const size_t N = 3;

		/** Appearance */
		class GOLEM_LIBRARY_DECLDIR Appearance {
		public:
			/** Line colour */
			RGBA lineColour;

			/** Show normals */
			bool normalShow;
			/** Points' normals size */
			Real normalSize;
			/** Normal colour */
			RGBA normalColour;

			/** Show frame */
			bool frameShow;
			/** Frame axes size */
			Vec3 frameSize;

			/** Constructs description. */
			Appearance() {
				setToDefault();
			}
			/** Sets the parameters to the default values. */
			void setToDefault() {
				lineColour = RGBA::YELLOW;
				normalShow = true;
				normalSize = Real(0.005);
				normalColour = RGBA::CYAN;
				frameShow = false;
				frameSize.set(Real(0.02));
			}
			/** Checks if the description is valid. */
			bool isValid() const {
				if (normalSize <= REAL_ZERO || !frameSize.isPositive())
					return false;
				return true;
			}
		};

		/** Feature distance metric */
		class GOLEM_LIBRARY_DECLDIR FlannDist {
		public:
			typedef bool is_kdtree_distance;
			typedef Real ElementType;
			typedef Real ResultType;

			/** constructor */
			FlannDist() {
			}

			template <typename _Iter1, typename _Iter2> Real operator() (_Iter1 a, _Iter2 b, size_t size = Feature::N, Real worst_dist = -numeric_const<Real>::ONE) const {
				//printf("a=<%f>, b=<%f>, d=%f\n", a[0], b[0], Math::sqrt(Math::sqr(a[0] - b[0])));
				//return Math::sqrt(Math::sqr(a[0] - b[0]));
				return Math::sqrt(Math::sqr(a[0] - b[0]) + Math::sqr(a[1] - b[1]) + Math::sqr(a[2] - b[2]));
			}
			inline Real accum_dist(const Real& a, const Real& b, int) const {
				return Math::sqr(a - b);
			}
		};

		/** Feature normals: points' normals in a local frame  */
		Vec3 point;
		Vec3 normal;
		Mat34 frame;

		/** Constructor */
		Feature(const Vec3 &point, const Vec3 &normal = Vec3::zero()) {
			this->point.set(point);
			frame.set(Mat33::identity(), point);
			this->normal.set(normal);
		}
		/** Access to data as a single vector */
		inline Real* data() {
			return (Real*)&point.v;
		}
		/** Access to data as a single vector */
		inline const Real* data() const {
			return (const Real*)&point.v;
		}

		/** Access to data as a 3D point */
		inline Vec3 getPoint() {
			return point;
		}
		/** Access to data as a 3D point */
		inline const Vec3 getPoint() const {
			return (const Vec3)point;
		}

		/** Access to data as a 3D point */
		inline Vec3 getNormal() {
			return normal;
		}
		/** Access to data as a 3D point */
		inline const Vec3 getNormal() const {
			return (const Vec3)normal;
		}
		/** Draw feature */
		void draw(const Appearance& appearance, DebugRenderer& renderer) const;
	};

	/** Face name */
	static const char* FaceName[];

	/** Bounds */
	template <typename _Real, typename _RealEval> class _Bounds {
	public:
		typedef _Real Real;
		typedef _RealEval RealEval;
		typedef _Vec3<_Real> Vec3;
		typedef _Mat33<_Real> Mat33;
		typedef _Mat34<_Real> Mat34;
		typedef std::vector<Vec3> Vec3Seq;

		//typedef std::vector<_Bounds> Seq;
		typedef ScalarCoord<_Bounds, Configspace> Coord;

		/** Appearance */
		class Appearance {
		public:
			/** Line colour */
			RGBA lineColour;

			/** Show normals */
			bool normalShow;
			/** Points' normals size */
			Real normalSize;
			/** Normal colour */
			RGBA normalColour;

			/** Show frame */
			bool frameShow;
			/** Frame axes size */
			Vec3 frameSize;

			/** Show bounds */
			bool boundsShow;
			/** Bounds line size */
			Vec3 boundsLineSize;
			/** Bounds colour */
			RGBA boundsColour;


			/** Constructs description. */
			Appearance() {
				setToDefault();
			}
			/** Sets the parameters to the default values. */
			void setToDefault() {
				lineColour = RGBA::YELLOW;
				normalShow = true;
				normalSize = Real(0.005);
				normalColour = RGBA::CYAN;
				frameShow = true;
				frameSize.set(Real(0.05));
				boundsShow = true;
				boundsLineSize = Real(0.01);
				boundsColour = RGBA::RED;

			}
			/** Checks if the description is valid. */
			bool isValid() const {
				if (normalSize <= REAL_ZERO || !frameSize.isPositive())
					return false;
				return true;
			}
		};

		/** Surface */
		struct Surface {
			typedef std::vector<Surface> Seq;
			typedef std::vector<Seq> SeqSeq;
			Vec3 point;
			Vec3 t1;
			Vec3 t2;
			Vec3 t3;
			Vec3 normal;
			Face face;
		};
		/** Triangle */
		struct Triangle : public Surface {
			typedef std::vector<Triangle> Seq;
			typedef std::vector<Seq> SeqSeq;
			Real distance;
		};

		/** Create bounds from convex meshes */
		inline void create(const golem::Bounds::Seq& bounds) {
			for (size_t i = 0; i < bounds.size(); ++i) {
				const BoundingConvexMesh* mesh = dynamic_cast<const BoundingConvexMesh*>(bounds[i].get());
				if (mesh != nullptr) {
					surfaces.resize(surfaces.size() + 1);
					triangles.resize(triangles.size() + 1);
					surfaces.back().resize(mesh->getTriangles().size());
					triangles.back().resize(mesh->getTriangles().size());
					for (size_t j = 0; j < mesh->getTriangles().size(); ++j) {
						triangles.back()[j].normal = surfaces.back()[j].normal = Vec3(mesh->getNormals()[j]);
						surfaces.back()[j].t1 = Vec3(mesh->getVertices()[mesh->getTriangles()[j].t1]); // e.g. first triangle
						surfaces.back()[j].t2 = Vec3(mesh->getVertices()[mesh->getTriangles()[j].t2]); // e.g. first triangle
						surfaces.back()[j].t3 = Vec3(mesh->getVertices()[mesh->getTriangles()[j].t3]); // e.g. first triangle
//						surfaces.back()[j].point = Vec3(mesh->getVertices()[mesh->getTriangles()[j].t1]); // e.g. first triangle
						surfaces.back()[j].point = (mesh->getVertices()[mesh->getTriangles()[j].t1] + mesh->getVertices()[mesh->getTriangles()[j].t2] + mesh->getVertices()[mesh->getTriangles()[j].t3]) / Real(3.0); // centroid
						triangles.back()[j].distance = Real(mesh->getDistances()[j]);
						surfaces.back()[j].face = Face::UNKNOWN;
					}
				}
			}
		}

		/** Pose */
		static inline void setPose(const Mat34& pose, const Surface& surface, Triangle& triangle) {
			pose.multiply(triangle.point, surface.point);
			pose.multiply(triangle.t1, surface.t1);
			pose.multiply(triangle.t2, surface.t2);
			pose.multiply(triangle.t3, surface.t3);
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
		static inline Real getDepth(const typename Triangle::Seq& triangles, const Vec3& point) {
			Real depth = numeric_const<Real>::MAX;
			for (typename Triangle::Seq::const_iterator i = triangles.begin(); i != triangles.end(); ++i) {
				const Real d = i->distance - i->normal.dot(point);
				if (d < numeric_const<Real>::ZERO) // no HBCollision
					return d;
				if (depth > d) // search for minimum
					depth = d;
			}
			return depth;
		}
		/** Penetration depth of a given point, zero if none */
		inline Real getDepth(const Vec3& point, const bool distance = false) const {
			//static size_t jj = 0;
			// if distance true, and there are not HBCollisions, then returns the closest point outside the bounds (negative values) 
			Real depth = distance ? -numeric_const<Real>::MAX : numeric_const<Real>::ZERO; // if no bounds or HBCollisions, no effect
			for (typename Triangle::SeqSeq::const_iterator i = triangles.begin(); i != triangles.end(); ++i) {
				const Real d = getDepth(*i, point);
				if (depth < d) // search for maximum depth (can collide only with one mesh within bounds)
					depth = d;
			}
			//if (jj % 100 == 0) printf("getdepth=%f\n", depth);
			return depth;
		}

		inline Real getDepth(const Feature &data, const bool distance = false) const {
			return getDepth((Bounds::Vec3)data.getPoint(), distance);
		}

		/** Get surface distance */
		static inline Real getSurfaceDistance(const typename Triangle::Seq& triangles, const Vec3& point, const Face face) {
			Real distance = numeric_const<Real>::MAX;
			Real penetration = numeric_const<Real>::ONE;
			for (typename Triangle::Seq::const_iterator i = triangles.begin(); i != triangles.end(); ++i) {
				if (face != Face::UNKNOWN && i->face != face)
					continue;
				const Real d2 = i->point.distance(point);
				if (distance > d2) // search for minimum distance
					distance = d2;
				// check for penetration
				if (penetration > numeric_const<Real>::ZERO) {
					const Real d = i->distance - i->normal.dot(point);
					if (d > numeric_const<Real>::ZERO) // no HBCollision
						penetration = -numeric_const<Real>::ONE;
				}
			}
			return penetration*distance;
		}
		/** Get surface distance */
		inline Real getSurfaceDistance(const Vec3& point, const Face face) const {
			// penetration value is positive if there is a penetration
//			const Real penetration = getDepth(point) > REAL_ZERO ? REAL_ONE : -REAL_ONE;
			// distance to the closest surface of this bound
			Real distance = -numeric_const<Real>::MAX;
			for (typename Triangle::SeqSeq::const_iterator i = triangles.begin(); i != triangles.end(); ++i) {
				const Real d = getSurfaceDistance(*i, point, face);
				if (distance < d) // search for minimum distance
					distance = d;
			}
			//if (jj % 100 == 0) printf("getdepth=%f\n", depth);
			return /*penetration**/distance;
		}
		
		/** Get surface distance: negative is outside the bounds */
		inline Real getSurfaceDistance(const Feature &data, const Face face) const {
			return getSurfaceDistance((Bounds::Vec3)data.getPoint(), face);
		}

		/** Computes the contribution of each point to calculare the median frame */
		inline Real frameWeight(const Real distance) const {
			return Math::exp(-distance);
		}

		/** Compute minimal distance or avarage penetration */
		template <typename _Ptr> inline _RealEval distance(_Ptr begin, _Ptr end, Vec3& frame, U32& HBCollisions, const Face face = Face::UNKNOWN) const {
			Vec3 inFrame, outFrame; inFrame.setZero(); outFrame.setZero();
			_RealEval eval = numeric_const<_RealEval>::ZERO, c = numeric_const<_RealEval>::ZERO, minDist = -numeric_const<Real>::MAX;
			for (_Ptr i = begin; i < end; ++i) {
				const Real distance = getSurfaceDistance(*i, face);
				if (distance > numeric_const<Real>::ZERO) {
					kahanSum(eval, c, distance);
					inFrame += (*i).getPoint()*frameWeight(distance);
					++HBCollisions;
				}
				else if (minDist < distance) {
					minDist = distance;
					outFrame = (*i).getPoint();
				}
			}
			frame = HBCollisions > 0 ? inFrame : outFrame;
			return HBCollisions > 0 ? eval/HBCollisions : minDist;
		}

		/** HBCollision likelihood model. No evaluation for points outside the bounds. */
		template <typename _Ptr> inline _RealEval evaluate(_Ptr begin, _Ptr end, _RealEval depthStdDev, size_t& HBCollisions) const {
			_RealEval eval = numeric_const<_RealEval>::ZERO, c = numeric_const<_RealEval>::ZERO;
			for (_Ptr i = begin; i < end; ++i) {
				const Real depth = getDepth(*i);
				if (depth > numeric_const<Real>::ZERO) {
					const _RealEval pointEval = Math::exp(-depthStdDev*Real(depth));
					kahanSum(eval, c, pointEval);
					++HBCollisions;
				}
			}
			return eval;
		}

		/** HBCollision likelihood model. Evaluate also the likelihood of HBCollision for point outside the bounds. */
		template <typename _Ptr> inline _RealEval evaluate(_Ptr begin, _Ptr end, _RealEval depthStdDev, _RealEval distanceStdDev, bool observation, const Mat34 &pose, const Real &force, size_t& HBCollisions) const {
			_RealEval eval = numeric_const<_RealEval>::EPS, c = numeric_const<_RealEval>::ZERO;
			Real norm = numeric_const<Real>::ONE / (depthStdDev*Math::sqrt(2 * numeric_const<Real>::PI));
			for (_Ptr i = begin; i < end; ++i) {
				const Real depth = getDepth(*i, true); // penetration depth (positive values) or distance from the closest surfaces (negative values)
				const bool direction = match(pose, *i, force);
				// this hypothesis generate a contact that does not match the observation
				if (!observation && depth > numeric_const<Real>::ZERO) {
//					printf("wrong direction\n");
					return numeric_const<_RealEval>::ZERO;
				}
				// compute likelihood only for bounds were a contact occurred
				if (observation) {
					const _RealEval pointEval = norm*Math::exp(-.5*Math::sqr(Real(depth)/Real(depthStdDev))); // gaussian 
					kahanSum(eval, c, pointEval);
					// penalise concats not on the bound's surface
					if (depth >= 0.0008 || !direction) {
						const _RealEval penalise = -.5*pointEval;  //-Math::exp(-Real(depth)); //-.1*pointEval;
						kahanSum(eval, c, penalise);
					}
					++HBCollisions;
				}
			}
			//if (observation) printf("eval = %f\n", eval);
			return eval;
		}

		inline bool match(const Mat34& pose, const Feature& point, const Real &force) const {
			Mat34 inverse; inverse.setInverse(pose); // compute the inverse of the joint frame
			Vec3 v; inverse.multiply(v, point.getPoint()); //v.normalise(); // compute the point in the joint's reference frame

			return ((v.z < 0 && force <= 0) || (v.z > 0 && force >= 0)) ? false : true; // return false if the the point does not match the observation
		}


		/** Penetration depth of a given point, zero if none */
		inline Real getDistance(const Mat34& pose, const Vec3& point, const Real maxDist) const {
			Mat34 inverse; inverse.setInverse(pose); // compute the inverse of the joint frame
			Vec3 v; inverse.multiply(v, point); v.normalise(); // compute the point in the joint's reference frame

			return v.z < REAL_ZERO ? pose.p.distance(point) : maxDist; // compute the distance only for point in front of the finger tip
		}

		/** Penetration depth of a given point, zero if none */
		inline Real getDistance(const Mat34& pose, const Feature& feature, const Real maxDist) const {
			return getDistance(pose, (Bounds::Vec3)feature.getPoint(), maxDist);
		}

		/** Expected HBCollision likelihood model */
		template <typename _Ptr> inline _RealEval estimate(Mat34& pose, _Ptr begin, _Ptr end, _RealEval depthStdDev, size_t& HBCollisions, const _RealEval maxDist = numeric_const<_RealEval>::MAX) const {
			_RealEval eval = numeric_const<_RealEval>::ZERO, c = numeric_const<_RealEval>::ZERO;
			for (_Ptr i = begin; i < end; ++i) {
				const Real distance = getDistance(pose, *i, maxDist);
				if (distance > numeric_const<Real>::ZERO && distance < maxDist) {
					const _RealEval pointEval = Math::exp(-depthStdDev*Real(distance));
					kahanSum(eval, c, pointEval);
					++HBCollisions;
				}
			}
			return eval;
		}


		/** Empty */
		inline bool empty() const {
			return surfaces.empty();
		}

		/** Triangles */
		inline const typename Triangle::SeqSeq& getTriangles() const {
			return triangles;
		}
		/** Triangles */
		inline typename Triangle::SeqSeq& getTriangles() {
			return triangles;
		}
		/** Surfaces */
		inline const typename Surface::SeqSeq& getSurfaces() const {
			return surfaces;
		}
		/** Surfaces */
		inline typename Surface::SeqSeq& getSurfaces() {
			return surfaces;
		}

	private:
		/** Triangles */
		typename Triangle::SeqSeq triangles;
		/** Surfaces */
		typename Surface::SeqSeq surfaces;
	};

	/** HBCollision waypoint */
	class GOLEM_LIBRARY_DECLDIR Waypoint {
	public:
		typedef std::vector<Waypoint> Seq;

		/** Path distance */
		Real pathDist;
		/** Number of points */
		U32 points;
		/** Distance standard deviation */
		Real depthStdDev;
		/** Likelihood multiplier */
		Real likelihood;

		/** Constructs description object */
		Waypoint() {
			Waypoint::setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			pathDist = Real(0.0);
			points = 1000;
			depthStdDev = Real(1000.0);
			likelihood = Real(1000.0);
		}
		/** Checks if the parameters are valid. */
		bool isValid() const {
			if (!Math::isFinite(pathDist) || depthStdDev < REAL_EPS || likelihood < REAL_ZERO)
				return false;
			return true;
		}
	};

	/** Bounds */
	typedef _Bounds<F64, F64> Bounds;

	/** Flann description */
	class GOLEM_LIBRARY_DECLDIR FlannDesc {
	public:
		/** Neighbour points during a query */
		U32 neighbours;
		/** Max neighbouring points selected for HBCollision detection */
		U32 points;
		/** Distance standard deviation */
		Real depthStdDev;
		/** Likelihood multiplier */
		Real likelihood;

		/** Consider a point as a sphere to increase accuracy */
		Real radius;

		/** Contructor */
		FlannDesc() {
			setToDefault();
		}
		/** Nothing to do here */
		virtual ~FlannDesc() {}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			neighbours = 1000;
			points = 250;
			depthStdDev = Real(1000.0);
			likelihood = Real(1000.0);

			radius = REAL_ZERO;
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (neighbours <= 0 || points < 0)
				return false;
			if (depthStdDev < REAL_EPS || likelihood < REAL_ZERO)
				return false;
			return true;
		}

	};
	
	/** Model for the FT sensor in DLR Hit II */
	class GOLEM_LIBRARY_DECLDIR FTSensorDesc {
	public:
		/** Force sensor limit */
		RealSeq ftMedian;
		/** Force sensor limit */
		RealSeq ftStd;

		FTSensorDesc() {
			FTSensorDesc::setToDefault();
		}
		void setToDefault() {
			ftMedian.clear();
			ftStd.clear();
		}
		bool isValid() const {
			for (size_t i = 0; i < ftMedian.size(); ++i)
				if (!Math::isFinite(ftMedian[i]))
					return false;
			for (size_t i = 0; i < ftStd.size(); ++i)
				if (!Math::isPositive(ftStd[i]))
					return false;
			return true;
		}
	};

	/** HBCollision description */
	class GOLEM_LIBRARY_DECLDIR Desc {
	public:
		typedef shared_ptr<Desc> Ptr;

		/** Switch between kdtrees and random selected points */
		bool kdtree;
		FlannDesc flannDesc;

		/** Force sensor reading in free space */
		FTSensorDesc ftBase;
		/** Force sensor limit */
		FTSensorDesc ftContact;

		/** HBCollision waypoints */
		Waypoint::Seq waypoints;

		/** KDTree decription*/
		KDTreeDesc nnSearchDesc;

		/** Feature appearence */
		Feature::Appearance featureAppearence;
		/** Bounds appearence */
		Bounds::Appearance boundsAppearence;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Nothing to do here */
		virtual ~Desc() {}
		/** Creates the object from the description. */
		virtual HBCollision::Ptr create(const Manipulator& manipulator) const {
			return HBCollision::Ptr(new HBCollision(manipulator, *this));
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			waypoints.clear();
			waypoints.push_back(Waypoint());
			nnSearchDesc.setToDefault();
			kdtree = true;
			flannDesc.setToDefault();
			ftBase.setToDefault();
			ftContact.setToDefault();
			featureAppearence.setToDefault();
			boundsAppearence.setToDefault();
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			//if (waypoints.empty())
			//	return false;
			if (!flannDesc.isValid())
				return false;
			if (!ftBase.isValid() || !ftContact.isValid())
				return false;
			//for (Waypoint::Seq::const_iterator i = waypoints.begin(); i != waypoints.end(); ++i)
			//	if (!i->isValid())
			//		return false;
			return true;
		}
	};

	/** Create */
	virtual void create(Rand& rand, const Cloud::PointSeq& points);

	/** HBCollision detection at a given waypoint */
//	virtual bool check(const Waypoint& waypoint, const Manipulator::Config& config, bool debug = false) const;
	/** HBCollision detection using kdtree */
	virtual bool checkNN(const FlannDesc& desc, const Manipulator::Config& config, bool debug = false) const;

	/** HBCollision detection for the observational model durin hypopthesis-based planning */
	virtual Real estimate(const FlannDesc& desc, const Manipulator::Config& config, Real maxDist = REAL_MAX, bool debug = false) const;
	/** HBCollision detection using kdtree */
//	virtual bool estimate(const FlannDesc& desc, const Rand& rand, const Manipulator::Pose& pose, bool debug = false) const;

	/** HBCollision detection to simulate contact at execution time */
	virtual size_t simulate(const FlannDesc& desc, const Rand& rand, const Manipulator::Config& config, std::vector<Configspace::Index>& joints, RealSeq& forces, bool debug = false) const;
	/** HBCollision detection to simulate contact at execution time */
	virtual size_t simulate(const FlannDesc& desc, const Rand& rand, const Manipulator::Config& config, FTGuard::Seq& joints, bool debug = false) const;
	/** HBCollision detection to simulate contact at execution time */
	virtual size_t simulate(const FlannDesc& desc, const Rand& rand, const Manipulator::Config& config, RealSeq& forces, bool debug = false) const;
	/** HBCollision detection to simulate contact at execution time */
	inline size_t simulateFT(const FlannDesc& desc, const Rand& rand, const Controller::State& state, RealSeq& forces) const {
		DebugRenderer renderer;
		return simulateFT(renderer, desc, rand, manipulator.getConfig(state), forces, false);
	}
	/** HBCollision detection to simulate contact at execution time */
	virtual size_t simulateFT(DebugRenderer& renderer, const FlannDesc& desc, const Rand& rand, const Manipulator::Config& config, RealSeq& forces, bool debug = false) const;

	/** HBCollision likelihood estimation at a given waypoint */
	virtual Real evaluate(const Waypoint& waypoint, const Manipulator::Config& config, bool debug = false) const;

	/** HBCollision likelihood estimation at a given waypoint for belief update */
	virtual Real evaluate(const Waypoint& waypoint, const Manipulator::Config& config, const FTGuard::Seq& triggeredGuards, bool debug = false) const;

	/** HBCollision likelihood estimation at a given waypoint for belief update */
	virtual Real evaluate(const FlannDesc& desc, const Manipulator::Config& config, FTGuard::Seq& triggeredGuards, bool debug = false) const;

	/** HBCollision likelihood estimation at a given waypoint for belief update */
	virtual Real evaluateFT(DebugRenderer& renderer, const FlannDesc& desc, const Manipulator::Config& config, FTGuard::SeqPtr& triggeredGuards, /*Vec3Seq& handForcesVec,*/ bool debug = false) const;

	/** HBCollision likelihood estimation at a given waypoint */
	virtual Real evaluate(const FlannDesc& desc, const Manipulator::Config& config, bool debug = false) const;

	/** HBCollision likelihood estimation on a given path */
	virtual Real evaluate(const Manipulator::Waypoint::Seq& path, bool debug = false) const;

	/** Joints bounds */
	inline const Bounds::Coord& getJointBounds() const {
		return jointBounds;
	}
	/** Base bounds */
	inline const Bounds& getBaseBounds() const {
		return baseBounds;
	}
	/** Points */
	//inline const Bounds::Vec3Seq& getPoints() const {
	//	return points;
	//}
	inline const Feature::Seq& getPoints() const {
		return points;
	}


	/** Draw HBCollisions */
	void draw(const Waypoint& waypoint, const Manipulator::Config& config, DebugRenderer& renderer) const;
	/** Draw HBCollisions with kdtree */
	void draw(DebugRenderer& renderer, const Rand& rand, const Manipulator::Config& config, const HBCollision::FlannDesc& desc) const;
	/** Draw estimate */
	void draw(DebugRenderer& renderer, const Manipulator::Config& config, const HBCollision::FlannDesc& desc) const;
	/** Draw simulate */
	void draw(DebugRenderer& renderer, const Manipulator::Config& config, std::vector<Configspace::Index> &joints, RealSeq &forces, const HBCollision::FlannDesc& desc) const;
	/** HBCollision likelihood estimation at a given waypoint for belief update */
	void draw(DebugRenderer& renderer, const Waypoint& waypoint, const Manipulator::Config& config, const FTGuard::Seq &triggeredGuards, bool debug = false) const;

	/** Draw sequence of Feature */
	void draw(Feature::Seq::const_iterator begin, Feature::Seq::const_iterator end, const Feature::Appearance appearence, DebugRenderer& renderer) const;
	/** Draw FT bounds */
	void draw(DebugRenderer& renderer, const Bounds::Appearance appearance, const Mat34& pose, const Bounds& bounds, Face face = Face::UNKNOWN) const;

	/** Returns ft_sensor desc in free space */
	FTSensorDesc getFTBaseSensor() {
		return desc.ftBase;
	}
	/** Returns ft_sensor desc in contact */
	FTSensorDesc getFTContactSensor() {
		return desc.ftContact;
	}

#ifdef _HBCOLLISION_PERFMON
	static U32 perfEvalPoints, perfCheckNN, perfEstimate, perfSimulate;
	static SecTmReal tperfEvalPoints, tperfCheckNN, tperfEstimate, tperfSimulate;

	static void resetLog();
	static void writeLog(Context &context, const char *str);

#endif

protected:
	/** Manipulator */
	const Manipulator& manipulator;
	Manipulator::BoundsAppearance manipulatorAppearance;
	/** Description */
	const Desc desc;

	/** Joints with FTs */
	std::vector<Configspace::Index> ftJoints;
	/** Joints */
	Bounds::Coord jointBounds;
	/** FTs */
	Bounds::Coord ftBounds;
	/** Base */
	Bounds baseBounds;
	/** Points */
	Feature::Seq points;
	//	Bounds::Vec3Seq points;

	/** Trinagles order */
	std::vector<Face> faces;

	/** KD tree pointer */
	NNSearch::Ptr nnSearch;
	/** Create */
	HBCollision(const Manipulator& manipulator, const Desc& desc);
};

void XMLData(HBCollision::Waypoint& val, XMLContext* xmlcontext, bool create = false);
void XMLData(HBCollision::FlannDesc& val, XMLContext* xmlcontext, bool create = false);
void XMLData(HBCollision::FTSensorDesc& val, XMLContext* xmlcontext, bool create = false);
void GOLEM_LIBRARY_DECLDIR XMLData(HBCollision::Desc& val, XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

}; /** namespace */

#endif /** _GOLEM_HBPLANNER_HBCollision_H_ */
