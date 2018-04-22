/** @file OptimisationSA.cpp
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

//------------------------------------------------------------------------------

#include <Golem/Contact/OptimisationSA.h>
#include <Golem/Sys/XMLData.h>
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

void golem::OptimisationSA::SelectionDesc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("collision_last", collisionLast, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("collision_all", collisionAll, const_cast<golem::XMLContext*>(xmlcontext), false);
}

void golem::OptimisationSA::Desc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("runs", runs, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("steps", steps, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("tries", tries, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("sa_temp", saTemp, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("sa_delta_lin", saDelta.lin, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("sa_delta_ang", saDelta.ang, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("sa_energy", saEnergy, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("epsilon", epsilon, const_cast<golem::XMLContext*>(xmlcontext), false);
	selectionDesc.clear();
	golem::XMLData(selectionDesc, selectionDesc.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "selection", false);
}

//------------------------------------------------------------------------------

OptimisationSA::OptimisationSA(Contact& contact, const Desc& desc) : Optimisation(contact), parallels(context.getParallels()), manipulator(contact.getManipulator()), configuration(*contact.getConfiguration()), contactEval(nullptr), desc(desc) {
	// check description
	desc.assertValid(Assert::Context("OptimisationSA()."));

	// check parallels
	if (!parallels || parallels->getNumOfThreads() < 1)
		throw Message(Message::LEVEL_CRIT, "OptimisationSA(): Parallels required");

	// collision models from description
	collision.resize(parallels->getNumOfThreads());
	for (Collision::Seq::iterator i = collision.begin(); i != collision.end(); ++i)
		(*i) = contact.getCollision()->create(contact.getManipulator());
}

void OptimisationSA::create(const data::Point3D& points, golem::data::ContactQueryCallback::ContactEval* contactEval) {
	// configuration model
	configs.resize(configuration.getSpaces().size());
	for (size_t i = 0; i < configuration.getSpaces().size(); ++i)
		configs[i].create(configuration, configuration.getSpaces()[i]);

	// nn search
	typedef golem::KDTree<ContactModel::Real, ContactModel::NNDist, flann::SearchParams> KDTree;

	// contact models
	contacts.resize(this->contact.getQuery().size()); // 1:1
	size_t i = 0;
	CriticalSection cs;
	ParallelsTask(parallels, [&] (ParallelsTask*) {
		for (size_t j = 0;;) {
			{
				CriticalSectionWrapper csw(cs);
				if (i >= this->contact.getQuery().size())
					break;
				j = i++;
			}

			const Query* query = *(this->contact.getQuery().data() + j);
			if (query) {
				ContactModel& contact = *(contacts.data() + j);

				// contact description
				const Query::Contact3DDesc::TypeDesc& contact3DDesc = query->getContact3DDesc(points);

				// contact data
				contact.create(contact3DDesc, query->getPoses(), (size_t)parallels->getNumOfThreads());
				if (!contact.nnData.empty()) {
					flann::SearchParams search;
					flann::KDTreeSingleIndexParams index;
					contact.nnSearchDesc.getKDTreeSingleIndex(search, index);
					//flann::KDTreeIndexParams index;
					//contact.nnSearchDesc.getKDTreeIndex(search, index);
					contact.nnSearch.reset(new KDTree(
						search, index, contact.kernels.front().data(), contact.kernels.size(), sizeof(ContactModel::Kernel), ContactModel::Kernel::DIM, ContactModel::NNDist()
					));
				}
			}
		}
	});

	// collision models
	Rand rand(context.getRandSeed());
	for (Collision::Seq::iterator i = collision.begin(); i != collision.end(); ++i)
		(*i)->create(rand, points);

	// Contact query callback interface
	this->contactEval = contactEval;
}

//------------------------------------------------------------------------------

void OptimisationSA::evaluate(golem::U32 jobId, const golem::Contact::Config& config, golem::Contact::Likelihood& likelihood, bool collisions) const {
	// reset likelihood
	likelihood.setToDefault();

	// view
	const Contact::View& view = contact.getViews()[config.view];
	// sub-space
	const ConfigModel& space = configs[config.space];
	// grip waypoint
	const golem::Manipulator::Waypoint& waypoint = config.path.getGrip();
	const golem::Mat34 waypointFrame = waypoint.frame.toMat34();

	// robot links poses
	WorkspaceJointCoord joints;
	manipulator.getJointFrames(waypoint.config, waypointFrame, joints);

	// evaluate query densities
	// TODO use all query densities
	for (Contact::View::QueryPtr::Seq::const_iterator i = view.getQueryPtrSeq().begin(); i != view.getQueryPtrSeq().end(); ++i) {
		// make sure query density is not empty
		if (contacts[i->index].empty())
			throw Message(Message::LEVEL_ERROR, "OptimisationSA::evaluate(): empty query density #%u for view #%u", i->index + 1, U32(i - view.getQueryPtrSeq().begin()) + 1);
		
		// compute frame, transform to local frame
		const golem::Mat34 frame = (i->link.getType() == Manipulator::Link::TYPE_JOINT ? joints[i->link.getJoint()] : waypointFrame) * i->frame;
		
		// compute likelihood, use query weight
		likelihood.contacts[*i->link] = contact.getNormalisation() * static_cast<golem::Real>(contacts[i->index].evaluate(jobId, golem::RBCoord(frame)));
	}

	// configuration model
	likelihood.config = space.evaluate(ConfigModel::Config(space.begin, space.end, waypoint.config.data()));

	// evaluate collision model
	if (collisions)
		likelihood.collision = collision[jobId]->evaluate(config.path);

	// Contact query callback interface
	if (contactEval)
		likelihood.user = contactEval->contactEval(config.path);

	// create product
	likelihood.makeLogProduct();
}

void OptimisationSA::evaluate(const golem::Contact::Config& config, golem::Contact::Likelihood& likelihood, bool collisions) const {
	OptimisationSA::evaluate(0, config, likelihood, collisions);
}

bool OptimisationSA::isFeasible(const golem::Configuration::Path& path) const {
	// TODO
	return true;
}

//------------------------------------------------------------------------------

Contact::Config::Seq::iterator OptimisationSA::find(Contact::Config::Seq& configs, Contact::Config::Seq::iterator ptr) {
	// allocate space if required
	const size_t free = (size_t)(configs.end() - ptr); // free space
	const size_t insert = (size_t)(ptr - configs.begin()); // insertion pointer
	if (free < desc.runs) {
		configs.resize(configs.size() + desc.runs - free); // this invalidates pointers
		ptr = configs.begin() + insert; // recover insertion pointer
	}

	// Selection description (0)
	const SelectionDesc& selectionDesc = desc.selectionDesc[0];

	// generate up to desc.runs initial solutions
	size_t i = 0;
	CriticalSection cs;
	ParallelsTask(parallels, [&] (ParallelsTask*) {
		const U32 jobId = parallels->getCurrentJob()->getJobId();
		Rand rand(RandSeed(this->context.getRandSeed()._U32[0] + jobId, (U32)0));

		Contact::Config solution, test;

		for (bool accept = false;;) {
			// select next pointer
			{
				CriticalSectionWrapper csw(cs);
				if (accept) {
					accept = false;
					Contact::Config::alloc(*ptr); // alloc if null
					(*ptr)->set(contact, solution); // set data
					++ptr; // next
				}
				if (++i > desc.runs)
					break;
			}

			// global search: always provides a solution
			for (size_t i = 0; i <= desc.tries; ++i) {
				// initial solution
				contact.sample(rand, test);
				// evaluate config
				evaluate(jobId, test, test.likelihood, selectionDesc.collisionAll);
				// test
				if (i == 0 || test.likelihood.value > solution.likelihood.value) {
					// store the best so far
					solution = test;
					// done?
					if (solution.likelihood.isValid(desc.epsilon))
						break;
				}
			}

			// feasibility test: may invalidate solution
			if (!isFeasible(solution.path))
				continue;

			// collision test
			if (!selectionDesc.collisionAll && selectionDesc.collisionLast) {
				solution.likelihood.collision = collision[jobId]->evaluate(solution.path);
				solution.likelihood.makeLogProduct();
			}

			// request next pointer
			accept = true;
		}
	});

	// print debug information
	context.debug("OptimisationSA::find(): type=%s, name=%s, solutions=%u/%u\n",
		contact.getType().c_str(), contact.getName().c_str(), (ptr - configs.begin()) - insert, desc.runs
	);

	return ptr;
}

void OptimisationSA::find(Contact::Config::Seq::const_iterator begin, Contact::Config::Seq::const_iterator end, golem::U32 selectionStep, golem::Real beginStep, golem::Real endStep) {
	// check if there is anything to do
	if (begin == end)
		return;

	const size_t runs = (size_t)(end - begin);
	const size_t beginStepN = (size_t)Math::round(beginStep*desc.steps);
	const size_t endStepN = std::max(beginStepN + 1, (size_t)Math::round(endStep*desc.steps));

	// Selection description (selectionStep)
	const SelectionDesc& selectionDesc = selectionStep < desc.selectionDesc.size() ? desc.selectionDesc[selectionStep] : desc.selectionDesc.back();

	const Configspace::Range range = manipulator.getConfigRange();
	size_t acceptGreedy = 0, acceptSA = 0;

	// local search
	Contact::Config::Seq::const_iterator ptr = begin;
	CriticalSection cs;
	ParallelsTask(parallels, [&] (ParallelsTask*) {
		const U32 jobId = parallels->getCurrentJob()->getJobId();
		Rand rand(RandSeed(this->context.getRandSeed()._U32[0] + jobId, (U32)0));

		Contact::Config *solution = nullptr, test;
		Manipulator::Config config;

		for (;;) {
			// select next pointer
			{
				CriticalSectionWrapper csw(cs);
				if (ptr == end)
					break;
				solution = ptr++->get();
			}

			// View and sub-space
			test.view = solution->view;
			test.space = solution->space;
			test.manifold = solution->manifold;
			const Configuration::Space& space = configuration.getSpaces()[test.space];
			const Contact::View& view = contact.getViews()[test.view];

			// local search: try to find better solution using simulated annealing
			for (size_t i = beginStepN; i < endStepN; ++i) {
				// linear schedule
				const Real Scale = Real(desc.steps - i - 1) / desc.steps; // 1..0 for i=0..steps-1
				const Real Temp = (REAL_ONE - Scale)*desc.saTemp + Scale;
				const RBDist Delta(desc.saDelta.lin*Temp, desc.saDelta.ang*Temp);
				const Real Energy = desc.saEnergy*Temp;

				// sample
				// TODO View manifold
				configuration.sample(rand, space, view.manifold, solution->path.getGrip(), Delta, config);

				// generate new path
				configuration.generate(rand, space, config, solution->path, test.path);

				// evaluate test
				evaluate(jobId, test, test.likelihood, selectionDesc.collisionAll);

				// accept if better or
				if (test.likelihood.value > solution->likelihood.value || Math::exp((test.likelihood.value - solution->likelihood.value) / Energy) > rand.nextUniform<Real>()) {
					// debug
					test.likelihood.value > solution->likelihood.value ? ++acceptGreedy : ++acceptSA;
					// update
					solution->path = test.path;
					solution->likelihood = test.likelihood;
				}
			}

			// collision test
			if (!selectionDesc.collisionAll && selectionDesc.collisionLast) {
				solution->likelihood.collision = collision[jobId]->evaluate(solution->path);
				solution->likelihood.makeLogProduct();
			}
		}
	});

	// print debug information
	context.debug("OptimisationSA::find(): type=%s, name=%s, solutions=%u, steps_{begin, end}={%u, %u}, energy=%f, greedy accept=%d, SA accept=%d, eps=%e\n",
		contact.getType().c_str(), contact.getName().c_str(), runs, beginStepN, endStepN, desc.saEnergy, acceptGreedy, acceptSA, desc.epsilon
	);
}

//------------------------------------------------------------------------------

void golem::XMLData(golem::OptimisationSA::SelectionDesc::Seq::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	val.load(xmlcontext);
}

//------------------------------------------------------------------------------
