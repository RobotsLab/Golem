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
#ifndef _GOLEM_HBPLANNER_BELIEF_H_
#define _GOLEM_HBPLANNER_BELIEF_H_

//------------------------------------------------------------------------------

#include <Golem/HBPlanner/Hypothesis.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Forward declaration */
//class R2GPlanner;

//------------------------------------------------------------------------------

/** Pose distribution and estimation with tactile and visual feedback */
class GOLEM_LIBRARY_DECLDIR Belief {
public:
	friend class HBHeuristic;
//	friend class R2GPlanner;
	typedef shared_ptr<Belief> Ptr;
	typedef RBHeuristic<RBCoord, 1> Heuristic;
	typedef DEOptimisation<Heuristic> Optimisation;

	/** Description file */
	class GOLEM_LIBRARY_DECLDIR Desc/* : public RBPose::Desc*/ {
	public:
		typedef shared_ptr<Desc> Ptr;

		/** Pointer to RBPose desc file */
		RBPose::Desc::Ptr rbPoseDescPtr;

		/** Number of distribution kernels */
		size_t kernels;
		/** number of neighbours */
		size_t neighbours;
		/** Maximum distance between vector and kernel */
		Real distanceRange;

		/** Kernel diagonal covariance scale metaparameter */
		RBCoord covariance;
		/** Standard deviation */
		RBDist poseStdDev;
		/** Feature normal epsilon */
		Real featNormEps;
		/** Feature product */
		bool distProd;
		/** Rigid body distance */
		RBDist dist;
		/** Feature distance */
		Real distFeature;

		/** local alignment enabled */
		bool localEnabled;
		/** population size */
		size_t populationSize;
		/** number of generations min */
		size_t generationsMin;
		/** number of generations max */
		size_t generationsMax;
		/** distance difference threshold */
		Real distanceDiff;

		/** Optimisation */
		Optimisation::Desc optimisationDesc;
		/** Max point model distance */
		Real distanceMax;

		/** Hypothesis description file */
		Hypothesis::Desc::Ptr hypothesisDescPtr;

		/** Number of hypothesis per model **/
		size_t numHypotheses;
		/** Max number of surface points in the kd-trees **/
		size_t maxSurfacePoints;

		/** Metaparameter for density function, e.g. e^(lambda*x) **/
		Real lambda;

		/** Enable/disable hierarchical clustering */
		bool cluster;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			rbPoseDescPtr.reset(new RBPose::Desc());

			kernels = 250;
			neighbours = 100;
			distanceRange = Real(10.0);

			poseStdDev.set(Real(0.01), Real(100.0));
			std::fill(&covariance[0], &covariance[3], Real(0.01)); // Vec3
			std::fill(&covariance[3], &covariance[7], Real(0.01)); // Quat
			featNormEps = Real(1e-7);
			distProd = true;
			dist.set(Real(1.0), Real(1.0));
			distFeature = Real(0.0);

			localEnabled = true;
			populationSize = 100;
			generationsMin = 10;
			generationsMax = 100;
			distanceDiff = Real(1e-5);

			optimisationDesc.setToDefault();
			distanceMax = Real(0.01);
			hypothesisDescPtr.reset(new Hypothesis::Desc());
			numHypotheses = 5;
			maxSurfacePoints = 10000;
			lambda = 1;

			cluster = true;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(rbPoseDescPtr != nullptr, ac, "RBPose desc: null pointer.");

			Assert::valid(kernels > 0, ac, "kernels: < 1");
			Assert::valid(neighbours > 0, ac, "neighbours: < 1");
			Assert::valid(neighbours <= kernels, ac, "neighbours: > kernels");
			Assert::valid(distanceRange > REAL_ZERO, ac, "distanceRange: <= 0");

			for (size_t i = 0; i < RBCoord::N; ++i)
				Assert::valid(covariance[i] > REAL_ZERO, ac, "covariance[]: <= 0");
			Assert::valid(poseStdDev.isValid(), ac, "poseStdDev: invalid");
			Assert::valid(dist.isValid(), ac, "dist: invalid");
			Assert::valid(distFeature >= REAL_ZERO, ac, "distFeature: < 0");
			Assert::valid(!Math::equals(featNormEps, REAL_ZERO, REAL_EPS), ac, "featNormEps: ~ 0");

			Assert::valid(populationSize > 0, ac, "populationSize: < 1");
			Assert::valid(generationsMin > 0, ac, "generationsMin: < 1");
			Assert::valid(generationsMin <= generationsMax, ac, "generationsMin: > generationsMax");
			Assert::valid(distanceDiff >= REAL_ZERO, ac, "distanceDiff: < 0");

			Assert::valid(optimisationDesc.isValid(), ac, "optimisationDesc: invalid");
			Assert::valid(!Math::equals(distanceMax, REAL_ZERO, REAL_EPS), ac, "distanceMax: ~ 0");

			Assert::valid(hypothesisDescPtr != nullptr, ac, "Hypothesis: null pointer.");
			Assert::valid(hypothesisDescPtr->isValid(), ac, "HypothesisDesc: invalid.");
		}
		/** Creates the object from the description. */
		/*virtual*/ Belief::Ptr create(Context &context) const;
	};

	/** Transformation samples */
	const RBPose::Sample::Seq& getSamples() const {
		return poses;
	}
	/** Transformation samples */
	RBPose::Sample::Seq& getSamples() {
		return poses;
	}
	/** Transformation samples */
	inline const Hypothesis::Seq& getHypotheses() const {
		return hypotheses;
	}
	/** Transformation samples */
	inline Hypothesis::Seq& getHypotheses() {
		return hypotheses;
	}
	/** Returns samples properties */
	inline SampleProperty<Real, RBCoord> getSampleProperties() { return sampleProperties; };

	/** Sets the hypothesis for planning. NOTE: returns the action frame **/
	RBPose::Sample createHypotheses(const Cloud::PointSeq& model, const Mat34 &transform/*, const bool init = true*/);
	/** Returns the low-dimensional representation of the density **/
	RBPose::Sample::Seq getHypothesesToSample() const;

	/** Creates object frame */
	inline Mat34 createFrame(const Vec3Seq& points) {
		return rbPosePtr->createFrame(points);
	};

	/** Creates query object pose distribution */
	void createQuery(const Cloud::PointSeq& points);
	/** Creates a new set of poses (resampling wheel algorithm) */
	/*virtual*/ void createResample(/*const Manipulator::Config& robotPose*/);
	/** Creates belief update (on importance weights) given the robot's pose and the current belief state. NOTE: weights are normalised. */
	void createUpdate(DebugRenderer& renderer, const Waypoint &w, FTGuard::SeqPtr& triggeredGuards);

	/** Normalised weight for the input pose */
	inline Real normalise(const RBPose::Sample &pose) const {
		return normaliseFac > REAL_ZERO ? pose.weight / normaliseFac : pose.weight;
	}

	/** Returns the max weight associated to the corrent samples */
	Real maxWeight(const bool normalised = false) const;

	/** Sets belief */
	void set(const RBPose::Sample::Seq &poseSeq, const RBPose::Sample::Seq &hypothesisSeq, const Mat34 &trn, const Cloud::PointSeq &points);

	/** Sets poses */
	void setPoses(const RBPose::Sample::Seq &poseSeq); // throws exception

	/** Sets hypotheses */
	void setHypotheses(const RBPose::Sample::Seq &hypothesisSeq); // throws exception

	/** Resets the high representation distribution */
	inline void reset() {
		sampleProperties = initProperties;
		poses.clear();
		for (RBPose::Sample::Seq::const_iterator i = initPoses.begin(); i != initPoses.end(); ++i)
			poses.push_back(*i);
		context.write("spam::Belief::createQuery(): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", sampleProperties.covariance[0], sampleProperties.covariance[1], sampleProperties.covariance[2], sampleProperties.covariance[3], sampleProperties.covariance[4], sampleProperties.covariance[5], sampleProperties.covariance[6]);
	}

	/** Gets the covariance det associated with the hypotheses **/
	inline Real getCovarianceDet() { return covarianceDet; };

	/** Draw samples */
	void drawSamples(const size_t numSamples, DebugRenderer& renderer) const;

	/** Draw hypotheses */
	void drawHypotheses( DebugRenderer &renderer, const bool showOnlyMeanPose = false) const;

	/** Draw volumetric region for uncertainty */
	Bounds::Seq uncertaintyRegionBounds();

	/** Acquires manipulator */
	inline void setManipulator(Manipulator *ptr) { manipulator.reset(ptr); /*collision.reset(new Collision(context, *manipulator));*/ };

	/** Pose description */
	Desc desc;

	/** Real pose of the object */
	RBCoord realPose;

	/** Creates model features and search index fro point cloud */
	template <typename _Seq> void createModel(const Vec3Seq& vertices, const TriangleSeq& triangles, _Seq& points) {
		rbPosePtr->createModel(vertices, triangles, points);
	}

	/** Creates model features and search index fro point cloud */
	template <typename _Seq> void createModel(const _Seq& points) {
		rbPosePtr->createModel(points);
	}

	/** Maximum likelihood estimation */
	RBPose::Sample maximum();

	/** Sample pose from distribution */
	RBCoord sample() const;

	/** Prints out debug info for cluster analysis */
	std::string clusterToStr() { return clusterStr; }

	/** Probability density value=p(s) for c */
	Real density(const RBCoord &c) const;

//	~Belief();

protected:
	/** Context object */
	Context &context;
	/** Generator of pseudo random numbers */
	Rand rand;
	/** Pointer to the RBPose object */
	RBPose::Ptr rbPosePtr;

	/** Transformation samples */
	RBPose::Sample::Seq poses;
	/** Transformation samples properties */
	SampleProperty<Real, RBCoord> pose;
	/** Pose distribution covariance */
	RBDist poseCov, poseCovInv;

	/** Distance between a and b */
	Real distance(const RBCoord& a, const RBCoord& b) const;

	/** Kernel function */
	Real kernel(Real distance) const;

	/** Global alignment */
	void alignGlobal(RBPose::Sample& solution, Real& solutionEval, U32& votes);
	/** Mean-shift clustering */
	void meanShiftClustering(RBPose::Sample& solution, RBPose::Sample::Seq& clusters);

	/** Stream string to print out the debug info for cluster analysis */
	std::string clusterStr;

	/** Appearance */
	Hypothesis::Appearance appearance;

	/** Model point cloud **/
	Cloud::PointSeq modelPoints;
	/** Model frme reference **/
	Mat34 modelFrame;

	/** Hypothesis container **/
	Hypothesis::Seq hypotheses;
	/** Covariance det associated with the samples **/
	Real covarianceDet;

	///** Returns the max weight associated to the corrent samples */
	//Real maxWeight(const bool normalised = false) const;

	/** Initial belief distribution. NOTE: Used for the reset method. */
	RBPose::Sample::Seq initPoses;
	/** Transformation samples properties */
	SampleProperty<Real, RBCoord> sampleProperties, initProperties;
	/** Normalise factor */
	Real normaliseFac;

	/** Uncertainty region */
	BoundingBox::Desc uncertaintyDesc;

	/** Manipulator pointer */
	Manipulator::Ptr manipulator;

	/** Creates/initialises the object */
	bool create(const Desc& desc);
	/** Creates the object */
	Belief(Context& context);
};

//------------------------------------------------------------------------------

void GOLEM_LIBRARY_DECLDIR XMLData(Belief::Desc& val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

}; // namespace golem

//------------------------------------------------------------------------------

namespace golem {
	template <> void Stream::read(spam::Belief& belief) const;
	template <> void Stream::write(const spam::Belief& belief);
};	// namespace

//------------------------------------------------------------------------------

#endif /** _GOLEM_HBPLANNER_BELIEF_H_ */
