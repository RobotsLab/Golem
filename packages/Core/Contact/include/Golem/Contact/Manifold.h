/** @file Manifold.h
 *
 * Contact manifold
 *
 * @author Marek Kopicki
 *
 * @copyright  Copyright (C) 2017 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_CONTACT_MANIFOLD_H_
#define _GOLEM_CONTACT_MANIFOLD_H_

//------------------------------------------------------------------------------

#include <Golem/Contact/Contact3D.h>
#include <Golem/Contact/Configuration.h>
#include <Golem/Contact/Data.h>
#include <Golem/Tools/Cloud.h>
#include <Golem/Math/RBOptimisation.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Math/Optimisation.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Contact manifold */
class Manifold {
public:
	typedef golem::shared_ptr<Manifold> Ptr;

	/** Manifold vector */
	typedef golem::RBCoord Vec;
	/** Manifold heuristic */
	typedef golem::DEHeuristicFunc<Point3DKernel::NNData, Vec, golem::Real> Heuristic;
	/** Manifold optimisation */
	typedef golem::DEOptimisation<Heuristic> Optimisation;

	/** Dimension description */
	class DimDesc {
	public:
		typedef std::vector<DimDesc> Seq;

		/** Weight */
		Real weight;

		/** Axis type */
		bool axisTypeLin;
		/** Axis index */
		U32 axisIndex;

		/** Dimension steps */
		RealSeq steps;

		/** Constructs description object */
		DimDesc() {
			DimDesc::setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			weight = REAL_ONE;
			axisTypeLin = true;
			axisIndex = 0;
			steps.clear();
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(weight >= REAL_ZERO, ac, "weight: < 0");
			Assert::valid(axisIndex < 3, ac, "axisIndex: invalid");
			Assert::valid(!steps.empty(), ac, "steps: empty");
			for (RealSeq::const_iterator i = steps.begin(); i != steps.end(); ++i)
				Assert::valid(Math::isFinite(*i), ac, "steps[i]: invalid");
		}
	};

	/** Description */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Manifold compressed models */
		bool manifoldCompressed;

		/** Manifold space enabled */
		bool manifoldSpaceEnable;
		/** Manifold space enabled */
		bool manifoldViewEnable;

		/** Density distance enabled */
		bool densityDistEnable;
		/** Density distance parameters */
		Point3DKernelDist densityDist;
		/** Density distance bandwidth multiplier */
		golem::Real densityDistBandwidth;

		/** NN search (data::Point3D): description */
		KDTreeDesc nnSearchDesc;
		/** NN search (data::Point3D): number of neighbours */
		golem::U32 nnNeighbours;

		/** Cloud downsampling */
		Cloud::DownsampleDesc downsampleDesc;

		/** Direction waypoint index */
		U32 directionWaypoint;
		/** Direction variation */
		Real directionVar;
		/** Position range */
		Vec3 positionRange;
		/** Frame range */
		RBDist frameDist;
		/** Dimensions */
		DimDesc::Seq dimDescSeq;
		/** Dimensions range */
		RBDist dimDist;
		/** Optimisation */
		Optimisation::Desc optimisationDesc;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		virtual ~Desc() {}
		/** Creates the object from the description. */
		virtual Manifold::Ptr create(const Configuration& configuration) const {
			return Manifold::Ptr(new Manifold(*this, configuration));
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			manifoldCompressed = true;

			manifoldSpaceEnable = true;
			manifoldViewEnable = true;

			densityDistEnable = true;
			densityDist.setToDefault();
			densityDistBandwidth = golem::Real(5.0);

			nnSearchDesc.setToDefault();
			nnNeighbours = 100;

			downsampleDesc.setToDefault();
			downsampleDesc.enabledWithNormals = true;

			directionWaypoint = 2;
			directionVar = Real(100.0);
			positionRange = Vec3(Real(0.05), Real(0.05), Real(0.05));
			frameDist = RBDist(Real(10000.0), Real(100.0));
			dimDescSeq.clear();
			dimDist = RBDist(Real(0.0), Real(1.0));
			optimisationDesc.setToDefault();
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			densityDist.assertValid(Assert::Context(ac, "densityDist."));
			Assert::valid(densityDistBandwidth >= golem::REAL_ONE, ac, "densityDistBandwidth: < 1");

			nnSearchDesc.assertValid(Assert::Context(ac, "nnSearchDesc."));
			Assert::valid(nnNeighbours > 0, ac, "nnNeighbours: <= 0");

			downsampleDesc.assertValid(Assert::Context(ac, "downsampleDesc."));

			Assert::valid(directionVar > golem::REAL_ZERO, ac, "directionVar: <= 0");
			Assert::valid(positionRange.isPositive(), ac, "positionRange: < 0");
			Assert::valid(frameDist.isPositive(), ac, "frameDist: < 0");
			Assert::valid(!dimDescSeq.empty(), ac, "dimDescSeq: empty");
			for (DimDesc::Seq::const_iterator i = dimDescSeq.begin(); i != dimDescSeq.end(); ++i)
				i->assertValid(Assert::Context(ac, "dimDescSeq[i]."));
			Assert::valid(dimDist.isPositive(), ac, "dimDist: < 0");
			Assert::valid(optimisationDesc.isValid(), ac, "optimisationDesc: invalid");
		}
		/** Load descritpion from xml context. */
		virtual void load(const golem::XMLContext* xmlcontext);
	};

	/** Pre-process uncompressed data chunk */
	virtual void process(golem::data::ContactModel::Data& data, UICallback* pUICallback = nullptr) const;

	/** Bye bye */
	virtual ~Manifold() {}

protected:
	/** Configuration */
	const Configuration &configuration;
	/** Manipulator */
	const Manipulator &manipulator;
	/** Context object */
	golem::Context &context;

	/** Manifold space enabled */
	bool manifoldSpaceEnable;
	/** Manifold space enabled */
	bool manifoldViewEnable;

	/** Density distance enabled */
	bool densityDistEnable;
	/** Density distance parameters */
	Point3DKernelDist densityDist;
	/** Density distance bandwidth multiplier */
	golem::Real densityDistBandwidth;

	/** NN search (data::Point3D): description */
	KDTreeDesc nnSearchDesc;
	/** NN search (data::Point3D): number of neighbours */
	golem::U32 nnNeighbours;

	/** Cloud downsampling */
	Cloud::DownsampleDesc downsampleDesc;

	/** Direction waypoint index */
	U32 directionWaypoint;
	/** Direction variation */
	Real directionVar;
	/** Position range */
	Vec3 positionRange;
	/** Frame range */
	RBDist frameDist;
	/** Dimensions */
	DimDesc::Seq dimDescSeq;
	/** Dimensions range */
	RBDist dimDist;
	/** Optimisation */
	Optimisation::Desc optimisationDesc;

	/** Process space manifold */
	virtual void processSpaceManifold(U32 spaceIndex, const U32Seq& views, golem::data::ContactModel::Data& data) const;
	/** Process view manifold */
	virtual void processViewManifold(U32 viewIndex, golem::data::ContactModel::Data& data) const;

	/** Creates Manifold */
	Manifold(const Desc& desc, const Configuration& configuration);
};

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(Manifold::DimDesc::Seq::value_type &val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace


//------------------------------------------------------------------------------

#endif /*_GOLEM_CONTACT_MANIFOLD_H_*/
