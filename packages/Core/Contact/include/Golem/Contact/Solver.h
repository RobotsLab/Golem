/** @file Solver.h
 * 
 * Contact solver
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
#ifndef _GOLEM_CONTACT_SOLVER_H_
#define _GOLEM_CONTACT_SOLVER_H_

//------------------------------------------------------------------------------

#include <Golem/Contact/Contact.h>
#include <Golem/Contact/Data.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Contact solver */
class Solver {
public:
	typedef golem::shared_ptr<Solver> Ptr;
	/** Solver collection: solver name -> solver */
	typedef std::map<std::string, Ptr> Map;
	
	/** Contact estimator collection: contact type name -> contact estimator name */
	typedef std::map<std::string, std::string> ContactTypeMap;
	/** Default contact type */
	static const std::string ContactTypeAny;

	/** Contact type selection */
	class SelectionDesc {
	public:
		/** Sequence */
		typedef std::vector<SelectionDesc> Seq;

		/** Scaled size [0..1] */
		golem::Real size;
		/** Scaled step begin [0..1] */
		golem::Real begin;
		/** Scaled step end [0..1] */
		golem::Real end;

		/** View [0..1] */
		golem::Real view;
		/** Space [0..1] */
		golem::Real space;

		/** Constructs description object */
		SelectionDesc() {
			SelectionDesc::setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			size = golem::REAL_ONE;
			end = golem::REAL_ZERO;
			begin = golem::REAL_ONE;
			view = golem::REAL_ZERO;
			space = golem::REAL_ZERO;
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (size <= golem::REAL_ZERO || size > golem::REAL_ONE)
				return false;
			if (begin < golem::REAL_ZERO || begin >= end || end > golem::REAL_ONE)
				return false;
			if (view < golem::REAL_ZERO || view > golem::REAL_ONE)
				return false;
			if (space < golem::REAL_ZERO || space > golem::REAL_ONE)
				return false;
			return true;
		}
		/** Checks if the description is valid. */
		template <typename _Seq> static bool isValid(const _Seq& seq) {
			if (seq.empty())
				return false;
			for (typename _Seq::const_iterator i = seq.begin(); i != seq.end(); ++i)
				if (!i->isValid())
					return false;
			return true;
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Solver description */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		/** Solver description collection: solver name -> solver description */
		typedef std::map<std::string, Ptr> Map;

		/** Name */
		std::string name;

		/** Contact estimator collection: contact type name -> contact estimator name */
		ContactTypeMap contactTypeMap;
		/** Contact estimator */
		Contact::Desc::Map contactDescMap;

		/** Contact type selection */
		SelectionDesc::Seq selectionDesc;

		/** NN search (data::Point3D): description */
		KDTreeDesc nnSearchDesc;
		/** NN search (data::Point3D): number of neighbours */
		golem::U32 nnNeighbours;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Destroy */
		virtual ~Desc() {}
		/** Creates the object from the description. */
		virtual Solver::Ptr create(Manipulator& manipulator) const {
			return Solver::Ptr(new Solver(*this, manipulator));
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			name.clear();
			contactTypeMap.clear();
			contactDescMap.clear();
			selectionDesc.clear();
			selectionDesc.push_back(SelectionDesc());
			nnSearchDesc.setToDefault();
			nnNeighbours = 100;
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(name.length() > 0, ac, "name: empty");

			Assert::valid(!contactTypeMap.empty(), ac, "contactTypeMap: empty");
			bool typeAny = false;
			for (ContactTypeMap::const_iterator i = contactTypeMap.begin(); i != contactTypeMap.end(); ++i) {
				Assert::valid(!i->first.empty() && contactDescMap.find(i->second) != contactDescMap.end(), ac, "contactDescMap[]: invalid");
				if (i->first == ContactTypeAny) typeAny = true;
			}
			Assert::valid(typeAny, ac, std::string("contactTypeMap: missing type " + ContactTypeAny).c_str());

			for (Contact::Desc::Map::const_iterator i = contactDescMap.begin(); i != contactDescMap.end(); ++i) {
				Assert::valid(!i->first.empty() && i->second != nullptr, ac, "contactDescMap[]: null");
				i->second->assertValid(Assert::Context(ac, "contactDescMap[]->"));
			}

			Assert::valid(SelectionDesc::isValid(selectionDesc), ac, "selectionDesc: invalid");

			nnSearchDesc.assertValid(Assert::Context(ac, "nnSearchDesc."));
			Assert::valid(nnNeighbours > 0, ac, "nnNeighbours: <= 0");
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Find most likely contact class from test data set */
	virtual void find(const data::Point3D& points, Contact::Config::Seq& configs, const StringSet* types = nullptr, golem::data::ContactQueryCallback::ContactEval* contactEval = nullptr);

	/** Add training data */
	virtual Contact::Map::const_iterator add(const std::string& type, const golem::data::ContactModel::Data& data);
	/** Clear training data */
	virtual void clear();

	/** Solver name */
	const std::string& getName() const {
		return name;
	}

	/** Contact estimator collection: contact estimator name -> contact estimator */
	Contact::Map& getContactMap() {
		return contactMap;
	}
	/** Contact estimator collection: contact estimator name -> contact estimator */
	const Contact::Map& getContactMap() const {
		return contactMap;
	}

	/** Release resources */
	virtual ~Solver();

protected:
	/** Manipulator */
	Manipulator& manipulator;
	/** Context object */
	golem::Context &context;

	/** Solver name */
	std::string name;

	/** Contact estimator collection: contact estimator name -> contact estimator */
	Contact::Map contactMap;

	/** Contact estimator config collection: contact type name -> contact estimator configs */
	Contact::Config::SeqMap contactConfigSeqMap;
		
	/** Contact type selection */
	SelectionDesc::Seq selectionDesc;

	/** Point3D kernels (object density) */
	Point3DKernel::SeqPtr object;
	/** NN search (data::Point3D): description */
	KDTreeDesc nnSearchDesc;
	/** NN search (data::Point3D): number of neighbours */
	golem::U32 nnNeighbours;
	/** NNSearch */
	NNSearch::Ptr nnSearch;

	Solver(const Desc& desc, Manipulator& manipulator);
};

//------------------------------------------------------------------------------

void XMLData(golem::Solver::ContactTypeMap::value_type& val, golem::XMLContext* xmlcontext, bool create = false);
void XMLData(golem::Solver::SelectionDesc::Seq::value_type& val, golem::XMLContext* xmlcontext, bool create = false);
void XMLData(golem::Solver::Desc::Map::value_type& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CONTACT_SOLVER_H_*/
