/** @file Solver.cpp
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

//------------------------------------------------------------------------------

#include <Golem/Contact/Solver.h>
#include <Golem/Contact/OptimisationSA.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <Golem/Math/Clustering.h>
#ifdef WIN32
#pragma warning (push)
#pragma warning (disable : 4291 4244 4996 4305 4267 4334)
#endif
#include <flann/flann.hpp>
#ifdef WIN32
#pragma warning (pop)
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const std::string golem::Solver::ContactTypeAny = "Any";

//------------------------------------------------------------------------------

void golem::Solver::SelectionDesc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("size", size, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("begin", begin, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("end", end, const_cast<golem::XMLContext*>(xmlcontext), false);
	try {
		golem::XMLData("view", view, const_cast<golem::XMLContext*>(xmlcontext), false);
	}
	catch (golem::MsgXMLParserAttributeNotFound&) {}
	try {
		golem::XMLData("space", space, const_cast<golem::XMLContext*>(xmlcontext), false);
	}
	catch (golem::MsgXMLParserAttributeNotFound&) {}
}

void golem::Solver::Desc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("name", name, const_cast<golem::XMLContext*>(xmlcontext), false);
	contactTypeMap.clear();
	golem::XMLData(contactTypeMap, contactTypeMap.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "type", false);
	contactDescMap.clear();
	golem::XMLData(contactDescMap, contactDescMap.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "contact", false);
	selectionDesc.clear();
	golem::XMLData(selectionDesc, selectionDesc.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "selection", false);
	
	golem::XMLContext* xmlnnsearch = xmlcontext->getContextFirst("nn_search", false);
	golem::XMLData("neighbours", nnNeighbours, xmlnnsearch, false);
	golem::XMLData(nnSearchDesc, xmlnnsearch, false);
}

//------------------------------------------------------------------------------

golem::Solver::Solver(const Desc& desc, Manipulator& manipulator) : manipulator(manipulator), context(manipulator.getContext()) {
	desc.assertValid(Assert::Context("Solver()."));

	name = desc.name;
	selectionDesc = desc.selectionDesc;
	nnSearchDesc = desc.nnSearchDesc;
	nnNeighbours = desc.nnNeighbours;

	for (ContactTypeMap::const_iterator i = desc.contactTypeMap.begin(); i != desc.contactTypeMap.end(); ++i) {
		Contact::Desc::Map::const_iterator j = desc.contactDescMap.find(i->second);
		if (j == desc.contactDescMap.end() || j->second == nullptr)
			throw Message(Message::LEVEL_ERROR, "Solver::Solver(): %s: unable to find %s contact description", i->second.c_str(), getName().c_str());
		context.verbose("Solver::Solver(): %s: Creating contact estimator: type = %s, name = %s\n", getName().c_str(), i->first.c_str(), i->second.c_str());
		j->second->type = i->first; // update type
		this->contactMap.insert(Contact::Map::value_type(i->first, j->second->create(manipulator)));
	}
}

golem::Solver::~Solver() {
}

//------------------------------------------------------------------------------

void golem::Solver::find(const data::Point3D& points, Contact::Config::Seq& configs, const StringSet* types, golem::data::ContactQueryCallback::ContactEval* contactEval) {
	// contact estimator pointer + data
	struct ContactData : public Contact::Config::Range {
		typedef std::vector<ContactData> Seq;
		Contact::Config::Seq::iterator ptr; // sort pointer
		Contact* contact; // contact estimator
		SecTmReal tCreate, tFind; // performance time
		U32 nContacts;
		ContactData(const Contact::Map::value_type& value) : contact(value.second.get()), tCreate(SEC_TM_REAL_ZERO), tFind(SEC_TM_REAL_ZERO), nContacts(0){}
	};
	ContactData::Seq contactData;

	// insertion pointer
	size_t insertPtr = 0;

	// select contact estimator and data ranges
	if (types == nullptr || types->empty()) {
		configs.clear(); // clear configs
		std::copy(contactMap.begin(), contactMap.end(), std::inserter(contactData, contactData.begin()));
	}
	else {
		// find contacts from contact map
		for (StringSet::const_iterator i = types->begin(); i != types->end(); ++i) {
			Contact::Map::const_iterator j = contactMap.find(*i);
			if (j != contactMap.end())
				contactData.push_back(*j);
		}
		if (contactData.empty())
			throw Message(Message::LEVEL_ERROR, "Solver::find(): unspecified contact types");

		// move null pointers and the queried contact types to the end if they exist (operator<)
		std::sort(configs.begin(), configs.end(), [=] (const Contact::Config::Ptr& l, const Contact::Config::Ptr& r) -> bool { return r == nullptr || l != nullptr && types->find(r->type) != types->end(); });

		// find insertion pointer
		for (Contact::Config::Seq::const_iterator i = configs.begin(); i != configs.end() && *i != nullptr && types->find((*i)->type) == types->end(); ++i, ++insertPtr);
	}

	context.verbose("Solver::find(): Initialising object model\n");
	PerfTimer t;

	// clusters
	data::Cluster3D::IndexSet clusterPoints;
	data::Cluster3D::getIndices(data::Cluster3D::CLUSTER_PROCESSING, &points, clusterPoints);

	// points
	object.reset(new Point3DKernel::Seq(clusterPoints.empty() ? points.getSize() : clusterPoints.size()));
	data::Cluster3D::IndexSet::const_iterator clusterPointPtr = clusterPoints.begin();
	for (size_t i = 0; i < object->size(); ++i)
		(*object)[i].set(points, clusterPoints.empty() ? i : (size_t)*clusterPointPtr++);
	if (!golem::Sample<Point3DKernel::RealEval>::normalise<golem::Ref1>(*object))
		throw Message(Message::LEVEL_ERROR, "Solver::find(): Unable to normalise object distribution");

	// initialise nn-search
	typedef golem::KDTree<Point3DKernel::Real, Point3DKernel::NNDist, flann::SearchParams> KDTree;
	flann::SearchParams search;
	flann::KDTreeSingleIndexParams index;
	nnSearchDesc.getKDTreeSingleIndex(search, index);
	//flann::KDTreeIndexParams index;
	//nnSearchDesc.getKDTreeIndex(search, index);
	nnSearch.reset(new KDTree(search, index, object->front().data(), object->size(), sizeof(Point3DKernel), Point3DKernel::NNDist::DIM, Point3DKernel::NNDist()));

	context.verbose("Solver::find(): Contact search\n");

	// initialise estimators
	for (ContactData::Seq::iterator contact = contactData.begin(); contact != contactData.end(); ++contact) {
		Contact::Config::Seq& contactConfigSeq = this->contactConfigSeqMap[contact->contact->getType()]; // type, NOT name!!!! create if empty
		contact->first = contact->second = contactConfigSeq.end(); // reset
		if (contact->contact->empty())
			continue;
		try {
			// initialisation of contact models, etc
			t.reset();
			contact->contact->create(points, object, nnSearch, contactEval);
			contact->tCreate = t.elapsed();
			contact->nContacts = (U32)contact->contact->getQuery().size();
			// create initial contacts, set data range
			t.reset();
			contact->second = contact->contact->find(contactConfigSeq, contactConfigSeq.begin());
			contact->first = contactConfigSeq.begin();
			contact->tFind += t.elapsed();
		}
		catch (const golem::Message& msg) {
			context.write(msg);
			continue;
		}
	}

	// refine solutions over all types
	for (SelectionDesc::Seq::const_iterator sel = selectionDesc.begin(); sel != selectionDesc.end(); ++sel) {
		// Index [1..selectionDesc.size()]
		const golem::U32 step = U32(sel - selectionDesc.begin()) + 1;
		// find max number of active contacts
		const size_t activeContacts = Contact::Config::getActiveContacts(contactData.begin(), contactData.end());
		// normalise
		Contact::Config::makeLogNormProduct(contactData.begin(), contactData.end(), activeContacts);
		
		// offset
		size_t offset = 0;

		// sort solutions within contact types, (re)set sort pointers
		for (ContactData::Seq::iterator contact = contactData.begin(); contact != contactData.end(); ++contact) {
			// old size (per contact)
			const size_t oldSize = (size_t)std::distance(contact->first, contact->second);
			// new size (per contact)
			const size_t newSize = (size_t)Math::round(sel->size*oldSize);

			// view and space selection
			bool viewSel = sel->view > REAL_EPS;
			bool spaceSel = !viewSel && sel->space > REAL_EPS; // exclusive view or space
			size_t size = 0;
			if (viewSel || spaceSel) {
				// count number of views and spaces
				typedef std::set<U32> U32Set;
				U32Set views, spaces;
				for (Contact::Config::Seq::const_iterator i = contact->first; i != contact->second; ++i) {
					if (viewSel) views.insert((*i)->view);
					if (spaceSel) spaces.insert((*i)->space);
				}
				// selection size
				if (viewSel) {
					if (views.size() <= 0)
						throw Message(Message::LEVEL_ERROR, "Solver::find(): No available views for selection");
					size = (size_t)Math::round(sel->size*sel->view*oldSize / views.size());
					if (size <= 0) {
						viewSel = false;
						context.warning("Solver::find(): Insufficient number of views for selection\n");
					}
					else
						context.verbose("Solver::find(): %u * %u = %u/%u selection views\n", (U32)views.size(), (U32)size, (U32)views.size()*size, (U32)newSize);
				}
				if (spaceSel) {
					if (spaces.size() <= 0)
						throw Message(Message::LEVEL_ERROR, "Solver::find(): No available spaces for selection");
					size = (size_t)Math::round(sel->size*sel->space*oldSize / spaces.size());
					if (size <= 0) {
						spaceSel = false;
						context.warning("Solver::find(): Insufficient number of spaces for selection\n");
					}
					else
						context.verbose("Solver::find(): %u * %u = %u/%u selection spaces\n", (U32)spaces.size(), (U32)size, (U32)spaces.size()*size, (U32)newSize);
				}
			}

			// sort with optional selection
			std::sort(contact->first, contact->second, [&] (const Contact::Config::Ptr& l, const Contact::Config::Ptr& r) -> bool {
				if (viewSel)
					return l->view < r->view || l->view == r->view && l->likelihood.value > r->likelihood.value;
				if (spaceSel)
					return l->space < r->space || l->space == r->space && l->likelihood.value > r->likelihood.value;

				return l->likelihood.value > r->likelihood.value;
			});

			// reset sort pointer
			contact->ptr = contact->first;

			// view and space selection
			if (viewSel || spaceSel) {
				// DEBUG
				//if (newSize < 1000) {
				//	for (Contact::Config::Seq::iterator i = contact->first; i != contact->second; ++i)
				//		printf("BEFORE: %u, %f\n", (*i)->space, (*i)->likelihood.value);
				//}
				
				// move top views or spaces to the beginning, the first solution is already there
				size_t n = 1;
				++offset;
				for (Contact::Config::Seq::iterator i = contact->first; ++i != contact->second;) {
					const bool equal = viewSel ? (*contact->ptr)->view == (*i)->view : (*contact->ptr)->space == (*i)->space;
					if (!equal)
						n = 0;
					if (n < size) {
						++contact->ptr;
						std::swap(*contact->ptr, *i);
						++offset;
					}
					++n;
				}
				// point to the unselected range
				++contact->ptr;
				// sort remaining solutions
				std::sort(contact->ptr, contact->second, [&](const Contact::Config::Ptr& l, const Contact::Config::Ptr& r) -> bool {
					return l->likelihood.value > r->likelihood.value;
				});
				
				// DEBUG
				//if (newSize < 1000) {
				//	for (Contact::Config::Seq::iterator i = contact->first; i != contact->ptr; ++i)
				//		printf("AFTER (SPACE): %u, %f\n", (*i)->space, (*i)->likelihood.value);
				//	for (Contact::Config::Seq::iterator i = contact->ptr; i != contact->second; ++i)
				//		printf("AFTER (ALL): %u, %f\n", (*i)->space, (*i)->likelihood.value);
				//}
			}
		}

		// old size
		const size_t oldSize = Contact::Config::getSize(contactData.begin(), contactData.end());
		// shrunk old size into new size
		const size_t newSize = (size_t)Math::round(sel->size*oldSize);

		// select solutions across contact types by moving sort pointers
		for (size_t i = offset; i < newSize; ++i) {
			ContactData::Seq::iterator best = contactData.begin();
			for (ContactData::Seq::iterator contact = ++contactData.begin(); contact != contactData.end(); ++contact)
				if (contact->ptr != contact->second && (best->ptr == best->second || contact->ptr != contact->second && best->ptr->get()->likelihood.value < contact->ptr->get()->likelihood.value))
					best = contact;
			if (best->ptr != best->second)
				++best->ptr;
		}
		// new ranges ends up in sort pointers
		for (ContactData::Seq::iterator contact = contactData.begin(); contact != contactData.end(); ++contact)
			contact->second = contact->ptr;
		
		// refine
		for (ContactData::Seq::iterator contact = contactData.begin(); contact != contactData.end(); ++contact) {
			try {
				t.reset();
				contact->contact->find(contact->first, contact->second, step, sel->begin, sel->end);
				contact->tFind += t.elapsed();
			}
			catch (const Message& msg) {
				context.write(msg);
			}
		}
	}

	// find max number of active contacts
	const size_t activeContacts = Contact::Config::getActiveContacts(contactData.begin(), contactData.end());
	// normalise
	Contact::Config::makeLogNormProduct(contactData.begin(), contactData.end(), activeContacts);
	// size
	const size_t size = Contact::Config::getSize(contactData.begin(), contactData.end());
	// resize output collection to hold all configs
	if (configs.size() < insertPtr + size)
		configs.resize(insertPtr + size);
	// copy results onto output collection
	SecTmReal tCreate = SEC_TM_REAL_ZERO, tFind = SEC_TM_REAL_ZERO; // performance time
	U32 nContacts = 0;
	Contact::Config::Seq::iterator config = configs.begin() + insertPtr;
	for (ContactData::Seq::iterator contact = contactData.begin(); contact != contactData.end(); ++contact) {
		for (Contact::Config::Seq::iterator i = contact->first; i != contact->second; ++i, ++config) {
			Contact::Config::alloc(*config); // allocate if null
			**config = **i; // deep copy
		}
		tCreate += contact->tCreate;
		tFind += contact->tFind;
		nContacts += contact->nContacts;
	}
	context.debug("Solver::find(): Done! time_{contacts, optimisation}={%.4f (%.4f, %u) [sec], %.4f [sec]}\n", tCreate, tCreate/nContacts, nContacts, tFind);

#ifdef _GOLEM_CONTACT_OPTIMISATION_DEBUG
	golem::Real evalDiff = REAL_ZERO;
	int evals = 0, cnt = 0, cntDiff = 0;
	for (ContactData::Seq::iterator contact = contactData.begin(); contact != contactData.end(); ++contact) {
		const OptimisationSA* optimisation = dynamic_cast<const OptimisationSA*>(contact->contact->getOptimisation());
		if (!optimisation)
			continue;
		for (OptimisationSA::ContactModel::Seq::const_iterator i = optimisation->getLinks().begin(); i != optimisation->getLinks().end(); ++i) {
			for (OptimisationSA::ContactModel::NNData::Seq::iterator j = i->nnData.begin(); j != i->nnData.end(); ++j) {
				evalDiff += j->evalDiff;
				evals += j->evals;
				cnt += j->cnt;
				cntDiff += j->cntDiff;
			}

		}
	}
	const Query::Desc& desc = *contactData.begin()->contact->getDesc().queryDescMap.begin()->second;
	context.verbose("Solver::find(): neighbours=%u, checks=%u, trees=%u, leaf_size=%u, branching=%u, iterations=%u, eval_diff=%.8f, cnt_diff=%.8f (%u/%u)\n",
		desc.neighbours, desc.nnSearchDesc.searchChecks, desc.nnSearchDesc.searchKDTrees, desc.nnSearchDesc.searchLeafMaxSize, desc.nnSearchDesc.searchBranching, desc.nnSearchDesc.searchIterations,
		evalDiff/evals, cnt > 0 ? Real(cntDiff)/cnt : REAL_ZERO, cntDiff, cnt
		);
#endif
}

//------------------------------------------------------------------------------

Contact::Map::const_iterator golem::Solver::add(const std::string& type, const golem::data::ContactModel::Data& data) {
	Contact::Map::const_iterator contact = contactMap.find(type);
	if (contact == contactMap.end())
		contact = contactMap.find(ContactTypeAny);

	if (contact == contactMap.end()) {
		std::stringstream str;
		for (Contact::Map::const_iterator i = contactMap.begin(); i != contactMap.end(); ++i)
			str << i->first << (i == --contactMap.end() ? "" : ", ");
		throw golem::Message(golem::Message::LEVEL_ERROR, "Solver::add(): unknown contact type %s not in {%s}", type.c_str(), str.str().c_str());
	}
	
	contact->second->add(data.contacts, data.views, data.spaces, data.selectorMap);
	
	return contact;
}

void golem::Solver::clear() {
	for (Contact::Map::iterator i = contactMap.begin(); i != contactMap.end(); ++i)
		i->second->clear();
}

//------------------------------------------------------------------------------

void golem::XMLData(golem::Solver::ContactTypeMap::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("type", const_cast<std::string&>(val.first), xmlcontext, create);
	golem::XMLData("name", val.second, xmlcontext, create);
}

void golem::XMLData(golem::Solver::SelectionDesc::Seq::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	val.load(xmlcontext);
}

void golem::XMLData(golem::Solver::Desc::Map::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("name", const_cast<std::string&>(val.first), xmlcontext, create);
	val.second.reset(new golem::Solver::Desc);
	val.second->load(xmlcontext);
}

//------------------------------------------------------------------------------
