/** @file Query.h
 *
 * Query density
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
#ifndef _GOLEM_CONTACT_QUERY_H_
#define _GOLEM_CONTACT_QUERY_H_

//------------------------------------------------------------------------------

#include <Golem/Contact/Contact3D.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Query density */
class Query : public golem::Sample<golem::Real> {
public:
	/** Pointer */
	typedef golem::shared_ptr<Query> Ptr;
	/** Container */
	typedef std::map<std::string, Ptr> Map;

	/** Kernel */
	class Kernel : public RBCoord, public RBCoordDist {
	public:
		/** Model */
		data::Part3D::Index model;

		/** No initialisation */
		Kernel() {}
		/** From coords and kernel */
		Kernel(const RBCoord& coord, const RBCoordDist& coordDist, data::Part3D::Index model = data::Part3D::INDEX_DEFAULT) : RBCoord(coord), RBCoordDist(coordDist), model(model) {}
	};

	/** Query pose hypothesis. Inheritance order: RBCoord first, Sample last */
	class Pose : public Kernel, public golem::Sample<golem::Real> {
	public:
		/** Pose collection/distribution */
		typedef std::vector<Pose> Seq;

		/** Dereferencing template */
		struct Ref {
			template <typename _Ptr> static inline const Kernel& get(_Ptr& ptr) {
				return ptr; // nothing to do
			}
		};
		/** Copying */
		inline const Pose& operator = (const Kernel& kernel) {
			(Kernel&)*this = kernel;
			return *this;
		}

		Pose(golem::Real weight = golem::REAL_ONE, golem::Real cdf = -golem::REAL_ONE) : golem::Sample<golem::Real>(weight, cdf) {}
		Pose(const Kernel& kernel, golem::Real weight = golem::REAL_ONE, golem::Real cdf = -golem::REAL_ONE) : Kernel(kernel), golem::Sample<golem::Real>(weight, cdf) {}
	};

	/** Description */
	class Contact3DDesc {
	public:
		/** Type-desc pair */
		typedef std::pair<Contact3D::Type, Contact3DDesc> TypeDesc;
		/** Collection */
		typedef std::vector<TypeDesc> Seq;

		/** Query density normalisation constant */
		golem::Real weight;

		/** Number of kernels */
		golem::U32 kernels;
		/** Query density normalisation epsilon */
		golem::Real epsilon;
		/** Maximum number of subsequent trials to compute a single kernel of M(u|r) */
		golem::U32 trials;
		/** Maximum number of subsequent trials to increment bandwidth */
		golem::U32 trialsBand;
		/** Trial bandwidth multiplier */
		golem::Real trialsBandFac;
		/** Maximum distance between features in standard deviations */
		golem::Real featureStdDevMax;
		/** Maximum distance between frames in standard deviations */
		golem::Real poseStdDevMax;

		/** Density distance enabled */
		bool densityDistEnable;
		/** Density distance parameters */
		Point3DKernelDist densityDist;
		/** Density distance bandwidth multiplier */
		golem::Real densityDistBandwidth;

		/** NN search (Contact3D): description */
		KDTreeDesc nnSearchDesc;
		/** NN search (Contact3D): number of neighbours */
		golem::U32 nnNeighbours;

		/** Constructs description object */
		Contact3DDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			weight = golem::REAL_ONE;
			kernels = 2000;
			epsilon = golem::REAL_EPS;
			trials = 100;
			trialsBand = 0;
			trialsBandFac = golem::Real(2.0);
			featureStdDevMax = golem::Real(5.0);
			poseStdDevMax = golem::Real(5.0);

			densityDistEnable = true;
			densityDist.setToDefault();
			densityDistBandwidth = golem::Real(5.0);

			nnSearchDesc.setToDefault();
			nnNeighbours = 100;
		}
		/** Assert that the description is valid. */
		void assertValid(Contact3D::Type type, const Assert::Context& ac) const {
			Assert::valid(weight >= golem::REAL_ZERO, ac, "weight: < 0");
			Assert::valid(kernels > 0, ac, "kernels: <= 0");
			Assert::valid(epsilon >= golem::REAL_ZERO, ac, "epsilon: < 0");
			Assert::valid(trials >= 0, ac, "trials: < 0");
			Assert::valid(trialsBand >= 0, ac, "trialsBand: < 0");
			Assert::valid(trialsBandFac >= golem::REAL_ONE, ac, "trialsBandFac: < 1");
			Assert::valid(featureStdDevMax > golem::REAL_EPS, ac, "featureStdDevMax: < eps");
			Assert::valid(poseStdDevMax > golem::REAL_EPS, ac, "poseStdDevMax: < eps");

			densityDist.assertValid(Assert::Context(ac, "densityDist."));
			Assert::valid(densityDistBandwidth >= golem::REAL_ONE, ac, "densityDistBandwidth: < 1");

			nnSearchDesc.assertValid(Assert::Context(ac, "nnSearchDesc."));
			Assert::valid(nnNeighbours >= 0, ac, "nnNeighbours: < 0");
		}
		/** Load descritpion from xml context. */
		void load(Contact3D::Type type, const golem::XMLContext* xmlcontext);
	};

	/** Description */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		/** Collection */
		typedef std::map<std::string, Ptr> Map;

		/** Debug level */
		golem::U32 debugLevel;

		/** Contact descriptions */
		Contact3DDesc::Seq contactDescSeq;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		virtual ~Desc() {}
		/** Creates the object from the description. */
		virtual Query::Ptr create(golem::Context& context, const std::string& name) const {
			return Query::Ptr(new Query(*this, context, name));
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			debugLevel = 0;
			contactDescSeq.clear();
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Assert::valid(!contactDescSeq.empty(), ac, "contactDescSeq: empty");
			for (Contact3DDesc::Seq::const_iterator i = contactDescSeq.begin(); i != contactDescSeq.end(); ++i)
				i->second.assertValid(i->first, Assert::Context(ac, "contactDescSeq[]."));
		}
		/** Load descritpion from xml context. */
		virtual void load(const golem::XMLContext* xmlcontext);
	};

	/** Create 3D contact query density */
	virtual void create(const Contact3D::Data& contacts, const data::Point3D& points);
	/** Clear query density data */
	virtual void clear();

	/** Model density */
	const Contact3D::Data& getContacts() const {
		return contacts;
	}
	/** Query density poses */
	const Pose::Seq& getPoses() const {
		return poses;
	}

	/** Sample query density */
	RBCoord sample(golem::Rand& rand) const;
	/** Sample query density */
	RBCoord sample(golem::Rand& rand, const Kernel& kernel) const;
	/** Evaluate pose given query density */
	golem::Real evaluate(const RBCoord& coord) const;
	
	/** Query density name */
	const std::string& getName() const {
		return name;
	}

	/** Query density description */
	const Desc& getDesc() const {
		return desc;
	}
	/** Query contact description */
	const Contact3DDesc::TypeDesc& getContact3DDesc() const;
	/** Query contact description */
	const Contact3DDesc::TypeDesc& getContact3DDesc(const data::Point3D& points) const;

	/** Point3D kernels (object density) */
	void setObject(Point3DKernel::SeqPtr object) {
		this->object = object;
	}
	/** Point3D kernels (object density) */
	Point3DKernel::SeqPtr getObject() const {
		return object;
	}

	/** NNSearch (data::Point3D) */
	void setNNSearch(NNSearch::Ptr nnSearch) {
		this->nnSearch = nnSearch;
	}
	/** NNSearch (data::Point3D) */
	NNSearch::Ptr getNNSearch() const {
		return nnSearch;
	}

	/** Bye bye */
	virtual ~Query() {}

protected:
	/** Context object */
	golem::Context &context;
	/** Parallels */
	golem::Parallels* parallels;

	/** Query density name */
	const std::string name;
	/** Query density description */
	Desc desc;
	/** Query contact description */
	const Contact3DDesc::TypeDesc* contact3DDesc;

	/** Model density */
	Contact3D::Data contacts;
	/** Query density */
	Pose::Seq poses;

	/** Point3D kernels (object density) */
	Point3DKernel::SeqPtr object;
	/** NNSearch */
	NNSearch::Ptr nnSearch;

	/** Create 3D contact query density */
	virtual void create(const Contact3DDesc& desc, const Contact3D::Data& contacts, const data::Point3D& points);
	/** Create 3D contact query density */
	virtual void create(const Contact3DDesc& desc, const Contact3D::Data& contacts, const data::Normal3D& normals);
	/** Create 3D contact query density */
	virtual void create(const Contact3DDesc& desc, const Contact3D::Data& contacts, const data::Feature3D& features);
	/** Create 3D contact query density */
	virtual void create(const Contact3DDesc& desc, const Contact3D::Data& contacts, const data::Part3D& parts);

	/** Creates query density */
	Query(const Desc& desc, golem::Context& context, const std::string& name);
};

void XMLData(golem::Query::Contact3DDesc::Seq::value_type& val, golem::XMLContext* context, bool create = false);
void XMLData(golem::Query::Desc::Map::value_type& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CONTACT_QUERY_H_*/
