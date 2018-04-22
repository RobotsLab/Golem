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
#ifndef _GOLEM_HBPLANNER_HEURISTIC_H_
#define _GOLEM_HBPLANNER_HEURISTIC_H_

//------------------------------------------------------------------------------

#include <Golem/Planner/Heuristic.h>
#include <Golem/Math/RB.h>
#include <Golem/HBPlanner/Belief.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Performance monitor */
//#define _HBHEURISTIC_PERFMON

//------------------------------------------------------------------------------

typedef std::vector<Real> RealSeq;

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR HBHeuristic : public golem::Heuristic {
public:
	typedef shared_ptr<HBHeuristic> Ptr;
	friend class Desc;
	friend class FTModelDesc;

	/** Observational model for force/torque model of arm/hand robot 
		Describes a cone over the normal of the end effector in which
		the likelihood of sensing a contact is greater than zero.
	*/
	class GOLEM_LIBRARY_DECLDIR FTModelDesc {
	public:
		/** Enabled/disables likelihood computation */
		bool enabledLikelihood;
		/** Exponential intrinsic parameter */
		Real lambda;
		/** Observational weight for each joint of the hand */
		RealSeq jointFac;
		/** Density normaliser */
		Real hitNormFac;

		/** Threashold (max linear dist) of sensing a contact */
		Real distMax;
		/** Threashold (max angle) of sensing contact over orizontal axis */
		Real coneTheta1;
		/** Threashold (max angle) of sensing contact over vertical axis */
		Real coneTheta2;

		/** Mahalanobis distance factor */
		Real mahalanobisDistFac;

		/** k-nearest neighbours */
		int k;

		/** Number of points to evaluate */
		size_t points;

		/** Descriptor */
		FTModelDesc() {
			setToDefault();
		}
		/** Sets parameters to default values */
		void setToDefault() {
			enabledLikelihood = true;
			distMax = Real(0.10); // 10 cm
			coneTheta1 = coneTheta2 = REAL_PI; // 90 degrees
			mahalanobisDistFac = Real(0.01);
			hitNormFac = REAL_ZERO;
			lambda = REAL_ONE;
			k = 50;
			points = 10000;
		}
		/** Checks if the description is valid */
		bool isValid() const {
			return true;
		}
	};

	/** Description file */
	class GOLEM_LIBRARY_DECLDIR Desc : public Heuristic::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(HBHeuristic, Heuristic::Ptr, Controller&)

	public:
		/** FTModel descriptor pointer */
		FTModelDesc ftModelDesc;

		/** Kernel diagonal covariance scale metaparameter */
		RBCoord covariance;
		/** Covariance determinant scale parameter */
		Real covarianceDet;
		/** Covariance determinant threashold (minimum) */
		Real covarianceDetMin;

		/** Contact factor in belief update */
		Real contactFac;
		/** Non contact factor in belief update */
		Real nonContactFac;

		/** Rewards directions for a confortable grasp (approaching with the palm towards the object) */
		Real directionFac;

		/** Enables/disable using k nearest neighbours */
		bool knearest;
		/** Enables/disables use of trimesh */
		bool trimesh;
		/** Number of indeces used to estimate the closest surface */
		size_t numIndeces;
		/** Number of points in used to create the KD-trees (subsample of model points) */
		size_t maxSurfacePoints;

		/** Evaluation descritption file */
		HBCollision::FlannDesc evaluationDesc;

		/** Check description file */
		HBCollision::FlannDesc checkDesc;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault(){
			Heuristic::Desc::setToDefault();	
			ftModelDesc.setToDefault();
			std::fill(&covariance[0], &covariance[3], Real(0.02)); // Vec3
			std::fill(&covariance[3], &covariance[7], Real(0.005)); // Quat
			covarianceDet = 10;
			covarianceDetMin = 0.00001;
			contactFac = REAL_ONE;
			nonContactFac = REAL_ONE;
			directionFac = Real(0.25);
			numIndeces = 5;
			knearest = true;
			trimesh = false;
			maxSurfacePoints = 10000;
			evaluationDesc.setToDefault();
			checkDesc.setToDefault();
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Heuristic::Desc::isValid())
				return false;

			if (!ftModelDesc.isValid())
				return false;

			if (!evaluationDesc.isValid() || !checkDesc.isValid())
				return false;

			for (size_t i = 0; i < RBCoord::N; ++i)
				if (!Math::isPositive(covariance[i]))
					return false;
			return true;
		}
		/** virtual destructor is required */
		virtual ~Desc() {
		}
	};

#ifdef _HBHEURISTIC_PERFMON
	static U32 perfCollisionWaypoint, perfCollisionPath, perfCollisionGroup, perfCollisionBounds, perfCollisionSegs, perfBoundDist, perfH;

	static void resetLog();
	static void writeLog(Context &context, const char *str);
#endif

	/** Evaluates a single waypoint */
	virtual Real cost(const Waypoint &w, const Waypoint &root, const Waypoint &goal) const;
	/** Objective cost function of a path between specified waypoints */
	virtual Real cost(const Waypoint &w0, const  Waypoint &w1) const;

	/** Sets heuristic description */
	void setDesc(const Desc& desc);
	void setDesc(const Heuristic::Desc& desc);
	/** Current heuristic description */
	inline const Heuristic::Desc& getDesc() const {
		return this->desc;
	}
	inline const Desc& getFTDrivenDesc() const {
		return ftDrivenDesc;
	}

	/** Acquires pose distribution **/
	inline void setBelief(Belief *belief) { pBelief.reset(belief); };

	/** Acquires manipulator */
	inline void setManipulator(Manipulator *ptr) { manipulator.reset(ptr); pBelief->setManipulator(ptr); /*collision.reset(new Collision(context, *manipulator));*/ };

	/** Collision detection test function for the single waypoint.
	* @param w			waypoint
	* @return			<code>true</code> if a collision has been detected; <code>false</code> otherwise
	*/
	virtual bool collides(const Waypoint &w) const;

	/** Collision detection test function for the single waypoint with thread data.
	* @param w			waypoint
	* @param data		a pointer to the current thread data
	* @return			<code>true</code> if a collision has been detected; <code>false</code> otherwise
	*/
	virtual bool collides(const Waypoint &w, ThreadData* data) const;

	/** Collision detection test function for the single waypoint.
	* @param w0			waypoint
	* @param w1			waypoint
	* @return			<code>true</code> if a collision has been detected; <code>false</code> otherwise
	*/
	virtual bool collides(const Waypoint &w0, const Waypoint &w1) const;

	/** Collision detection test function for the single waypoint with thread data.
	* @param w0			waypoint
	* @param w1			waypoint
	* @param data		a pointer to the current thread data
	* @return			<code>true</code> if a collision has been detected; <code>false</code> otherwise
	*/
	virtual bool collides(const Waypoint &w0, const Waypoint &w1, ThreadData* data) const;

	/** Sets collision checking with the object to be grasped */
	inline void setPointCloudCollision(const bool cloudCollision) { 
		pointCloudCollision = cloudCollision; 

		//if (collision.get()) {
		//	U32 jobs = 1;
		//	const Parallels* parallels = context.getParallels();
		//	if (parallels != NULL) {
		//		jobs = parallels->getNumOfThreads();
		//	}
		//	collision->clearKDTrees();
		//	for (U32 j = 0; j < jobs; ++j)
		//		for (auto i = pBelief->getHypotheses().begin(); i != pBelief->getHypotheses().end(); ++i)
		//			collision->setKDTree((*i)->getCloud(), j);
		//}
	};

	// Gets collision checking with object's points
	inline bool getPointCloudCollision() {
		return pointCloudCollision;
	}

	/** Sets surface points to be checked for collision */
	inline void setNumCollisionPoints(const U32 points) { waypoint.points = points; };
	/** Returns surface points to be checked for collision */
	inline U32 getNumCollisionPoints() { return waypoint.points; };

	/** Mahalanobis distance between rigid bodies a and b */
	Real distance(const RBCoord &a, const RBCoord &b, bool enableAng = false) const;

	inline RBCoord& getCovarianceSqrt() { return sampleProperties.covarianceSqrt; }

	/** test observational model */
	Real testObservations(const RBCoord &pose, const bool normal = false) const;

	/** virtual destructor */
	virtual ~HBHeuristic() {}

	/** Enable/disable of incorporating uncertainty into the cost function */
	bool enableUnc;
	/** Pointer to descriptor */
	HBHeuristic::Desc ftDrivenDesc;

	bool testCollision;
	inline Real getDist2(const Waypoint& w0, const Waypoint& w1) const {
		return Heuristic::getDist(w0, w1);
	};

	/** Returns a pointer to the collision interface */
	HBCollision* getCollision() { return collision.get(); };

	/** Returns true only if expected collisions are likely to happen */
	inline bool expectedCollisions(const Controller::State& state) const {
		const Manipulator::Config config = manipulator->getConfig(state);
		return hypothesisBoundsSeq.empty() ? false : intersect(manipulator->getBounds(config.config, config.frame.toMat34()), hypothesisBoundsSeq, false);
	}

	void renderHypothesisCollisionBounds(DebugRenderer& renderer) {
		renderer.setColour(RGBA(U8(255), U8(255), U8(0), U8(127)));
		renderer.setLineWidth(Real(1.0));
		renderer.addWire(hypothesisBoundsSeq.begin(), hypothesisBoundsSeq.end());
	}
	
	/** Reset collision bounds */
	void setHypothesisBounds();

	/** Chainspace coordinates properties. */
	inline const ChainDesc::ChainSeq& getChainDesc() const {
		return chainDesc;
	}
	/** Configspace coordinates properties. */
	inline const JointDesc::JointSeq& getJointDesc() const {
		return jointDesc;
	}

	/** Chainspace coordinates properties. */
	inline ChainDesc::ChainSeq& getChainDesc() {
		return chainDesc;
	}
	/** Configspace coordinates properties. */
	inline JointDesc::JointSeq& getJointDesc() {
		return jointDesc;
	}

protected:
	/** Generator of pseudo random numbers */
	Rand rand;

	/** Pose distribution **/
	Belief::Ptr pBelief;

	/** Manipulator pointer */
	Manipulator::Ptr manipulator;
	/** Collision interface */
	HBCollision::Ptr collision;
	/** Collision waypoint */
	HBCollision::Waypoint waypoint;

	/** Collision bound for check for expected collisions */
	Bounds::Seq hypothesisBoundsSeq;

	/** Sampled poses */
//	HypSample::Map samples;
	/** Transformation samples properties */
	SampleProperty<Real, RBCoord> sampleProperties;
	///** Inverse covariance matrix associated with the samples */
	//RBCoord covarianceInv;
	///** Squared covariance matrix associated with the samples */
	//RBCoord covarianceSqrt;
	/** Determinant of covariance matrix associated with the samples */
	Real covarianceDet;

	/** Observational weight for each joint of the hand */
	RealSeq jointFac;

	/** Model cloud points */
	Cloud::PointSeq modelPoints;

	/** Hit normalising factor */
	Real hitNormFacInv;

	/** Controller state info */
	Controller::State::Info armInfo;
	/** Controller state info */
	Controller::State::Info handInfo;

	/** Enables/disables collision checking with the object to be grasped */
	bool pointCloudCollision;

	/** Check the approaching direction of the grasp */
	Real directionApproach(const Waypoint &w) const;

	/** Bounded distance between a waypoint and the set of samples */
	Real getBoundedDist(const Waypoint& w) const;
	/** Distance between two waypoints in workspace */
	Real getMahalanobisDist(const Waypoint& w0, const Waypoint& goal) const;
	/** Observational cost function over a trajectory */
	Real expectedObservationCost(const Waypoint &wi, const Waypoint &wj) const;
	/** Computes the distance between future observations */
	Real psi(const Waypoint& wi, const Waypoint& wj, Hypothesis::Seq::const_iterator p) const;
	/** Pair observational function over a trajectory */
//	void h(const Waypoint &wi, const Waypoint &wj, Belief::Hypothesis::Seq::const_iterator p, std::vector<Real> &y) const;
	/** Pair observational function over a trajectory */
	void h(const Waypoint &wi, const Waypoint &wj, std::vector<Real> &y) const;

	/** Evaluate contact with hyptothesis */
	Real evaluate(const Hypothesis::Seq::const_iterator &hypothesis, const Waypoint &w) const;

	/** Estimate contact with hyptothesis */
	inline Real estimate(const Hypothesis::Seq::const_iterator &hypothesis, const Waypoint& w) const {
		const Manipulator::Config config(w.cpos);
		return (*hypothesis)->estimate(ftDrivenDesc.evaluationDesc, config, ftDrivenDesc.ftModelDesc.distMax);
	}

	/** Penalises configurations which collide with the mean hypothesis */
	Real getCollisionCost(const Waypoint &wi, const Waypoint &wj, Hypothesis::Seq::const_iterator p) const;

	/** Create heuristic from the description */
	bool create(const Desc &desc);
	/** Constructor */
	HBHeuristic(Controller &controller);
};

//------------------------------------------------------------------------------

void GOLEM_LIBRARY_DECLDIR XMLData(HBHeuristic::FTModelDesc& val, XMLContext* context, bool create = false);

void GOLEM_LIBRARY_DECLDIR XMLData(HBHeuristic::Desc& val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

}; // namespace golem

//------------------------------------------------------------------------------

#endif /** _GOLEM_HBPLANNER_HEURISTIC_H_ **/