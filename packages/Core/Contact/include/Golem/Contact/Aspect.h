/** @file Aspect.h
 *
 * Contact view aspect
 *
 * @author Marek Kopicki
 *
 * @copyright  Copyright (C) 2016 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_CONTACT_ASPECT_H_
#define _GOLEM_CONTACT_ASPECT_H_

//------------------------------------------------------------------------------

#include <Golem/Contact/Contact3D.h>
#include <Golem/Contact/Configuration.h>
#include <Golem/Contact/Solver.h>
#include <Golem/Contact/Data.h>
#include <Golem/Math/RBOptimisation.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Math/Clustering.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Contact view aspect */
class Aspect {
public:
	typedef golem::shared_ptr<Aspect> Ptr;

	/** Similarity function */
	typedef std::function<golem::Real(golem::U32, golem::U32)> SimFunc;

	/** Similarity */
	typedef std::pair<golem::Real, golem::Mat34> Similarity;
	/** Similarity sequence */
	typedef std::vector<Similarity> SimilaritySeq;

	/** Clustering */
	typedef golem::Clustering<golem::I32, golem::Real> Clustering;
	/** Clustering with similarity matrix definition */
	typedef golem::Clustering<golem::I32, Similarity> ClusteringSim;

	/** Contact3D kernel */
	typedef _Point3DKernel<golem::F32, golem::F64> SimPoint;

	/** Similarity floating point */
	typedef SimPoint::Real SimReal;
	/** Similarity Vec3 */
	typedef SimPoint::Vec3 SimVec3;
	/** Similarity Mat33 */
	typedef SimPoint::Mat33 SimMat33;
	/** Similarity Mat34 */
	typedef SimPoint::Mat34 SimMat34;
	/** Similarity RBCoord */
	typedef SimPoint::RBCoord SimRBCoord;
	/** Similarity RBDist */
	typedef SimPoint::RBDist SimRBDist;
	/** Feature vector */
	typedef SimPoint::Feature SimFeature;

	/** Contact3D Vec3 sequence */
	typedef std::vector<SimVec3> SimVec3Seq;
	/** Contact3D scalar sequence */
	typedef std::vector<SimReal> SimRealSeq;

	/** Optimisation heuristic */
	typedef golem::RBHeuristic<RBCoord, 1> Heuristic;
	typedef golem::DEOptimisation<Heuristic> Optimisation;

	/** importance within the cluster */
	class Importance: public golem::Sample<golem::Real> {
	public:
		/** Sequence */
		typedef std::vector<Importance> Seq;

		/** Contact index */
		golem::U32 index;
		/** Contact importance */
		golem::Real importance;
	};

	/** View cross-evaluation and validation procedure results */
	class ViewData {
	public:
		/** Sequence */
		typedef std::vector<ViewData> Seq;

		/** View cross-evaluation procedure: predicted trajectories and parameters */
		class Eval {
		public:
			/** Sequence */
			typedef std::vector<Eval> Seq;

			/** Trajectories */
			Contact::Config::Seq configs;

			/** Normalised score values for collisions with invisible parts */
			RealSeq evals;

			/** Normalised score value for collisions with invisible parts */
			golem::Real eval;

			/** Default constructor */
			Eval() : eval(golem::REAL_ZERO) {}
		};

		/** Viem map index (out of range if excluded) */
		golem::U32 index;

		/** Normalised overall score value for a given view */
		golem::Real eval;

		/** Normalised score value for view-view collisions with invisible parts */
		Eval::Seq evalSeq;

		/** Default constructor */
		ViewData(golem::U32 index = golem::numeric_const<golem::U32>::MAX) : eval(golem::REAL_ZERO), index(index) {}

		/** Access the data as an array. */
		inline Eval& operator [] (size_t idx) {
			return evalSeq[idx];
		}
		inline const Eval& operator [] (size_t idx) const {
			return evalSeq[idx];
		}

		/** allocate */
		static void allocate(size_t size, ViewData::Seq& data) {
			data.resize(size);
			for (size_t i = 0; i < size; ++i)
				data[i].evalSeq.resize(size);
		}
	};

	/** Aspect data	*/
	class Data {
	public:
		/** Data collection: contact type -> data */
		typedef std::multimap<std::string, Data> Map;

		/** Contact model mapping one to many */
		typedef std::multimap<golem::U32, golem::U32> PartialMap;
		/** Contact model mapping one to many */
		typedef std::multimap<golem::U32, std::pair<golem::U32, golem::Mat34> > CompleteMap;

		/** Points map */
		typedef std::map<golem::U32, F32Vec3Seq> PointsMap;

		/** Appearance */
		class Appearance {
		public:
			/** Visualisation reference frame for pairs */
			golem::Mat34 referenceFramePair;
			/** Visualisation reference frame for views */
			golem::Mat34 referenceFrameView;

			/** Visualisation reference frame */
			golem::Mat34 referenceFrame;

			/** Show frames */
			bool frameShow;
			/** Frame size */
			golem::Vec3 frameSize;
			/** Frame transformation */
			golem::Vec3 frameTrn;
			/** Frame transformation for complete view contacts */
			golem::Vec3 frameTrnComplete;
			/** Frame transformation for partial view contacts */
			golem::Vec3 frameTrnPartial;

			/** Frame size */
			golem::Real normalLen;

			/** Point colour */
			golem::RGBA pointColour;
			/** Point colour */
			golem::RGBA pointSimColour;
			/** Point size */
			golem::Real pointSize;

			/** Constructs description. */
			Appearance() {
				setToDefault();
			}
			/** Sets the parameters to the default values. */
			void setToDefault() {
				referenceFramePair.setId();
				referenceFrameView.setId();
				referenceFrame.setId();
				frameShow = true;
				frameSize = golem::Vec3(0.1);
				frameTrn.setZero();
				frameTrnComplete.setZero();
				frameTrnPartial.setZero();
				normalLen = golem::Real(0.01);
				pointColour = golem::RGBA::BLACK;
				pointSimColour = golem::RGBA::RED;
				pointSize = golem::Real(3.0);
			}
			/** Assert that the description is valid. */
			void assertValid(const Assert::Context& ac) const {
				Assert::valid(referenceFramePair.isValid(), ac, "referenceFramePair: invalid");
				Assert::valid(referenceFrameView.isValid(), ac, "referenceFrameViews: invalid");
				Assert::valid(referenceFrame.isValid(), ac, "referenceFrame: invalid");
				Assert::valid(frameSize.isPositive(), ac, "frameSize: invalid");
				Assert::valid(frameTrn.isValid(), ac, "frameTrn: invalid");
				Assert::valid(frameTrnComplete.isValid(), ac, "frameTrnComplete: invalid");
				Assert::valid(frameTrnPartial.isValid(), ac, "frameTrnPartial: invalid");
				Assert::valid(normalLen >= golem::REAL_ZERO, ac, "normalLen: < 0");
				Assert::valid(pointSize > golem::REAL_ZERO, ac, "pointSize: <= 0");
			}
			/** Load descritpion from xml context. */
			void load(const golem::XMLContext* xmlcontext);
		};

		/** Contact partial views */
		Contact3D::Data::Map contactPartial;
		/** Contact complete views */
		Contact3D::Data::Map contactComplete;

		/** Uncompressed complete model -> partial model */
		PartialMap partialMap;
		/** Compressed model -> complete model */
		CompleteMap completeMap;

		/** Similarity matrix */
		ClusteringSim::ValueSeqSeq simMat;

		/** Processing: Compressed model -> complete model */
		CompleteMap procCompleteMap;
		/** Processing: Contact models */
		Contact3D::Data::Map procContacts;
		/** Processing: Contact views */
		Contact::View::Seq procViews;

		/** View cross-evaluation procedure: predicted trajectories and parameters */
		ViewData::Seq viewDataMap;

			/** Processing: Contact views pruning stage */
		Contact::View::Seq procViewsPruning;

		/** Registered views */
		PointsMap pointsMap;

		/** empty */
		bool empty() const;
		/** clear */
		void clear();

		/** Draw training data */
		void draw(const Appearance& appearance, const data::ContactModel::Data& processedData, golem::DebugRenderer& renderer) const;
	};

	/** Description */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** View aspect enabled */
		bool enabled;
		/** Debug level */
		golem::U32 debugLevel;

		/** Data appearance description */
		Data::Appearance dataAppearance;

		/** Feature3D distance parameters */
		Point3DKernelDist contactDist;

		/** Optimisation enabled */
		bool contactSimOpt;
		/** Initial frame guess linear scale */
		golem::Real contactSimLinScale;
		/** DE optimisation */
		Optimisation::Desc contactSimOptDesc;
		/** DE optimisation population selection trials */
		golem::U32 contactSimOptInitTrials;

		/** Local outlier factor kNN, scaling factor (0..1) */
		golem::Real contactOutlierNeighboursFac;
		/** Local outlier factor F-test confidence */
		golem::Real contactOutlierFTest;

		/** Contact clustering */
		ClusteringDesc contactClusteringDesc;

		/** Contact cluster exemplar subsampling */
		bool contactExemplarSubsample;
		/** Contact cluster exemplar subsampling distance factor */
		golem::Real contactExemplarSubsampleDistFac;

		/** View pruning */
		bool viewPruning;
		/** Minimum number of contacts */
		golem::U32 viewMinContacts;
		/** View pruning solver description */
		golem::Solver::Desc::Ptr viewSolverDesc;

		/** View clustering downsample grid */
		golem::Real viewDownsampleGridLeafSize;

		/** View clustering */
		ClusteringDesc viewClusteringDesc;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		virtual ~Desc() {}
		/** Creates the object from the description. */
		virtual Aspect::Ptr create(const Configuration& configuration) const {
			return Aspect::Ptr(new Aspect(*this, configuration));
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			enabled = false;
			debugLevel = 0;

			dataAppearance.setToDefault();

			contactDist.setToDefault();

			contactSimOpt = true;
			contactSimLinScale = golem::Real(1.0);
			contactSimOptDesc.setToDefault();
			contactSimOptDesc.numOfThreads = 0;
			contactSimOptInitTrials = 100;

			contactOutlierNeighboursFac = golem::Real(0.1);
			contactOutlierFTest = golem::Real(0.05);

			contactClusteringDesc.setToDefault();

			contactExemplarSubsample = false;
			contactExemplarSubsampleDistFac = golem::Real(1.0);

			viewPruning = false;
			viewMinContacts = 1;
			viewSolverDesc.reset(new golem::Solver::Desc);

			viewDownsampleGridLeafSize = golem::Real(0.002);

			viewClusteringDesc.setToDefault();
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			dataAppearance.assertValid(Assert::Context(ac, "dataAppearance."));

			contactDist.assertValid(Assert::Context(ac, "contactDist."));

			Assert::valid(contactSimLinScale >= golem::REAL_ZERO, ac, "contactSimLinScale: < 0");
			Assert::valid(contactSimOptDesc.isValid() && contactSimOptDesc.numOfThreads <= 0, ac, "contactSimOptDesc: invalid");
			Assert::valid(contactSimOptInitTrials > 0, ac, "contactSimOptInitTrials: 0");

			Assert::valid(contactOutlierNeighboursFac > golem::REAL_ZERO && contactOutlierNeighboursFac < golem::REAL_ONE, ac, "contactOutlierNeighboursFac: invalid");
			Assert::valid(contactOutlierFTest >= golem::REAL_ZERO && contactOutlierFTest < golem::REAL_ONE, ac, "contactOutlierFTest: invalid");

			contactClusteringDesc.assertValid(Assert::Context(ac, "contactClusteringDesc."));

			Assert::valid(contactExemplarSubsampleDistFac >= golem::REAL_ZERO, ac, "contactExemplarSubsampleDistFac: < 0");

			Assert::valid(viewMinContacts > 0, ac, "viewMinContacts: 0");
			Assert::valid(viewSolverDesc != nullptr, ac, "viewSolverDesc: null");
			viewSolverDesc->assertValid(Assert::Context(ac, "viewSolverDesc->"));

			Assert::valid(viewDownsampleGridLeafSize > golem::REAL_EPS, ac, "viewDownsampleGridLeafSize: < eps");

			viewClusteringDesc.assertValid(Assert::Context(ac, "viewClusteringDesc."));
		}
		/** Load descritpion from xml context. */
		virtual void load(const golem::XMLContext* xmlcontext);
	};

	/** Process */
	virtual void process(golem::data::ContactModel::Data& data, golem::Aspect::Data& aspectData, UICallback* pUICallback = nullptr);

	/** Aspect density description */
	const Desc& getDesc() const {
		return desc;
	}

	/** Print data */
	void print(const std::string& str, const golem::data::ContactModel::Data& data, const golem::Aspect::Data& aspectData) const;

	/** Bye bye */
	virtual ~Aspect() {}

protected:
	/** Configuration */
	const Configuration &configuration;
	/** Manipulator */
	const Manipulator &manipulator;
	/** Context object */
	golem::Context &context;
	/** Parallels */
	golem::Parallels* parallels;

	/** Solver */
	golem::Solver::Ptr viewSolver;

	/** Pseudo-random number gen */
	golem::Rand rand;

	/** Aspect density description */
	Desc desc;

	/** Clustering */
	void clustering(ClusteringDesc& clusteringDesc, golem::U32 Size, SimFunc simFunc, Clustering::SizeSeq& assignments, UICallback* pUICallback = nullptr) const;

	/** Contact similarity */
	void contactSim(golem::Rand& rand, golem::U32 i, golem::U32 j, const Contact3D::Data::SeqMap& contactSeqMap, const SimPoint::SeqSeq& pointSeqSeq, const SimVec3Seq& meanSeq, const SimRealSeq& diamSeq, Similarity& sim, std::string& msg) const;

	/** Cloud frame */
	golem::Mat34 contactSimFrame(golem::Rand& rand, golem::U32 i, golem::U32 j, const SimVec3Seq& meanSeq, const SimRealSeq& diamSeq) const;

	/** Contact merge */
	void contactCluster(golem::Rand& rand, golem::U32 exemplar, const golem::U32Seq& cluster, const golem::Contact3D::Data::SeqMap& contactSeqMap, const ClusteringSim::ValueSeqSeq& simMat, golem::Contact3D::Seq& contactSeq) const;

	/** Pruning views */
	void viewsPruning(golem::data::ContactModel::Data& data, golem::Aspect::Data& aspectData, UICallback* pUICallback = nullptr);

	/** Compact data */
	virtual void compactData(golem::data::ContactModel::Data& data, golem::Aspect::Data& aspectData);
	/** Data preparation */
	virtual void preprocessData(golem::data::ContactModel::Data& data, golem::Aspect::Data& aspectData);

	/** Process contacts */
	virtual void processContacts(golem::data::ContactModel::Data& data, golem::Aspect::Data& aspectData, UICallback* pUICallback = nullptr);
	/** Process views */
	virtual void processViews(golem::data::ContactModel::Data& data, golem::Aspect::Data& aspectData, UICallback* pUICallback = nullptr);

	/** Creates Aspect */
	Aspect(const Desc& desc, const Configuration& configuration);
};

//------------------------------------------------------------------------------

};	// namespace

	//------------------------------------------------------------------------------

namespace golem {
	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::Aspect::Data::PointsMap::value_type& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::Aspect::Data::PointsMap::value_type& value);

	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::Aspect::ClusteringSim::ValueSeqSeq::value_type& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::Aspect::ClusteringSim::ValueSeqSeq::value_type& value);

	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::Aspect::ViewData::Eval::Seq::value_type& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::Aspect::ViewData::Eval::Seq::value_type& value);

	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::Aspect::ViewData::Seq::value_type& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::Aspect::ViewData::Seq::value_type& value);

	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::Aspect::Data::Map::value_type& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::Aspect::Data::Map::value_type& value);
};	// namespace

	//------------------------------------------------------------------------------

#endif /*_GOLEM_CONTACT_ASPECT_H_*/
