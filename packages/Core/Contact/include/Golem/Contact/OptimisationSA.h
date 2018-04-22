/** @file OptimisationSA.h
 *
 * Contact optimisation using simulated annealing (SA)
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
#ifndef _GOLEM_CONTACT_OPTIMISATIONSA_H_
#define _GOLEM_CONTACT_OPTIMISATIONSA_H_

//------------------------------------------------------------------------------

#include <Golem/Contact/Contact.h>
#include <Golem/Contact/OptimisationEvalCPU.h>

//------------------------------------------------------------------------------

// Optimisation debug
//#define _GOLEM_CONTACT_OPTIMISATION_DEBUG

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Contact optimisation using CPU */
class OptimisationSA : public Contact::Optimisation {
public:
	/** Configuration model */
	typedef EvalConfigModelCPU<golem::F32, golem::F32> ConfigModel;
	/** Contact model */
	typedef EvalContactModelCPU<golem::F32, golem::F32> ContactModel;

	/** Selection step description */
	class SelectionDesc {
	public:
		typedef std::vector<SelectionDesc> Seq;

		/** Collisions during last optimisation step */
		bool collisionLast;
		/** Collisions during all optimisation steps */
		bool collisionAll;

		/** Constructs description object */
		SelectionDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			collisionLast = true;
			collisionAll = false;
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Optimisation description */
	class Desc : public Contact::Optimisation::Desc {
	public:
		/** number of runs */
		size_t runs;
		/** number of steps per run */
		size_t steps;
		/** number of tries per run */
		size_t tries;

		/** Simulated annealing minimum temperature */
		golem::Real saTemp;
		/** Simulated annealing temperature to local coordinate scaling factor */
		golem::RBDist saDelta;
		/** Simulated annealing temperature to energy scaling factor */
		golem::Real saEnergy;

		/** Component validity test */
		golem::Real epsilon;

		/** Selection step description */
		SelectionDesc::Seq selectionDesc;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Creates the object from the description. */
		virtual Contact::Optimisation::Ptr create(Contact& contact) const {
			return Contact::Optimisation::Ptr(new OptimisationSA(contact, *this));
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			runs = 500;
			steps = 500;
			tries = 100;

			saTemp = golem::Real(0.1);
			saDelta.set(golem::Real(1.0), golem::Real(1.0));
			saEnergy = golem::Real(0.5);
			epsilon = golem::REAL_EPS;

			selectionDesc.resize(1);
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Assert::valid(runs > 0, ac, "runs: <= 0");
			
			Assert::valid(saTemp >= golem::REAL_ZERO, ac, "saTemp: < 0");
			Assert::valid(saDelta.isValid(), ac, "saDelta: invalid");
			Assert::valid(saEnergy > golem::REAL_ZERO, ac, "saEnergy: <= 0");
			Assert::valid(golem::Math::isFinite(epsilon), ac, "epsilon: invalid");

			Assert::valid(!selectionDesc.empty(), ac, "selectionDesc: empty");
			for (SelectionDesc::Seq::const_iterator i = selectionDesc.begin(); i != selectionDesc.end(); ++i)
				i->assertValid(Assert::Context(ac, "selection[i]."));
		}
		/** Load descritpion from xml context. */
		virtual void load(const golem::XMLContext* xmlcontext);
	};

	/** Optimisation: initialisation */
	virtual void create(const data::Point3D& points, golem::data::ContactQueryCallback::ContactEval* contactEval = nullptr);

	/** Optimisation: find initial solutions */
	virtual Contact::Config::Seq::iterator find(Contact::Config::Seq& configs, Contact::Config::Seq::iterator ptr);
	/** Optimisation: improve specified solutions [begin, end), using steps [beginStep, endStep], where: 0 <= beginStep < endStep <= 1 */
	virtual void find(Contact::Config::Seq::const_iterator begin, Contact::Config::Seq::const_iterator end, golem::U32 selectionStep, golem::Real beginStep, golem::Real endStep);
	
	/** Optimisation: evaluate config with optional collision expert */
	virtual void evaluate(const golem::Contact::Config& config, golem::Contact::Likelihood& likelihood, bool collisions = false) const;

	/** Config models */
	const ConfigModel::Seq& getConfigs() const {
		return configs;
	}
	/** Contact models */
	const ContactModel::Seq& getContacts() const {
		return contacts;
	}

	/** Description */
	Desc& getDesc() {
		return desc;
	}
	/** Description */
	const Desc& getDesc() const {
		return desc;
	}

protected:
	golem::Parallels* parallels;
	const golem::Manipulator& manipulator;
	const golem::Configuration& configuration;

	/** Description */
	Desc desc;
	
	/** Configuration models per sub-space */
	ConfigModel::Seq configs;
	/** Contact models */
	ContactModel::Seq contacts;
	/** Collision models */
	Collision::Seq collision;

	/** Contact query callback interface */
	golem::data::ContactQueryCallback::ContactEval* contactEval;

	void evaluate(golem::U32 jobId, const golem::Contact::Config& config, golem::Contact::Likelihood& likelihood, bool collisions = false) const;
	bool isFeasible(const golem::Configuration::Path& path) const;

	/** Create and copy description */
	OptimisationSA(Contact& contact, const Desc& desc);
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

namespace golem {
	void XMLData(golem::OptimisationSA::SelectionDesc::Seq::value_type& val, golem::XMLContext* xmlcontext, bool create = false);
};	// namespace


//------------------------------------------------------------------------------

#endif /*_GOLEM_CONTACT_OPTIMISATIONSA_H_*/
