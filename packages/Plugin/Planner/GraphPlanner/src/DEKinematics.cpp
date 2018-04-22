/** @file DEKinematics.cpp
 * 
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

//#include <Golem/Tools/Debug.h>
#include <Golem/Planner/GraphPlanner/DEKinematics.h>
#include <Golem/Sys/Context.h>
#include <Golem/Sys/Message.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

DEKinematics::DEKinematics(golem::Heuristic &heuristic) : Kinematics(heuristic) {
}

void DEKinematics::create(const Desc& desc) {
	setDesc(desc); // throws
	pHeuristic.reset(new Heuristic(*this));
	pOptimisation.reset(new Optimisation(*pHeuristic));
#ifdef _KINEMATICS_PERFMON
	trials = 0;
	tAvr= SEC_TM_REAL_ZERO;
	tMin = SEC_TM_REAL_MAX;
	tMax = SEC_TM_REAL_ZERO;
	errLinAvr = errAngAvr = REAL_ZERO;
#endif
}

void DEKinematics::setPopulation(const ConfigspaceCoord::Seq* population) {
	if (population)
		this->population = *population;
	else
		this->population.clear();

	pHeuristic->initialise();
}

void DEKinematics::setDesc(const Desc& desc) {
	if (!desc.isValid())
		throw MsgKinematicsInvalidDesc(Message::LEVEL_CRIT, "DEKinematics::setDesc(): Invalid description");
	this->desc = desc;
	this->distRootFac = REAL_ZERO; // reset by default
}

//------------------------------------------------------------------------------

bool DEKinematics::findGoal(const GenConfigspaceState &croot, const GenWorkspaceChainState &wgoal, GenConfigspaceState &cgoal, MSecTmU32 timeOut) {
	try {
#ifdef _KINEMATICS_PERFMON
		PerfTimer t;
#endif
		// setup the root waypoint (required by Heuristic::cost())
		root.setup(controller, croot.cpos, true, heuristic.hasCollisionDetection());
		// setup the goal waypoint (required by Heuristic::cost())
		goal.wpos = wgoal.wpos; // wgoal takes into account reference poses
		for (Chainspace::Index i = controller.getStateInfo().getChains().begin(); i < controller.getStateInfo().getChains().end(); ++i)
			goal.qrot[i].fromMat33(goal.wpos[i].R);
		// prepare and start computation
		pOptimisation->start(desc, timeOut);
		// fetch results
		const size_t solution = pOptimisation->stop(true);
		cgoal.t = wgoal.t;
		cgoal.cpos = croot.cpos;
		pHeuristic->copy(pOptimisation->getVectors()[solution], cgoal.cpos);
		// print debug info
#ifdef _KINEMATICS_PERFMON
		const SecTmReal elasped = t.elapsed();
		++trials;
		tAvr += elasped;
		tMin = std::min(tMin, elasped);
		tMax = std::max(tMax, elasped);
		const Chainspace::Index ch = controller.getStateInfo().getChains().begin();
		Waypoint w(controller, cgoal.cpos, true, false);
		const Real errLin = goal.wpos[ch].p.distance(w.wpos[ch].p);
		errLinAvr += errLin;
		const Real errAng = REAL_ONE - Math::abs(goal.qrot[ch].dot(w.qrot[ch]));
		errAngAvr += errAng;

		context.debug("DEKinematics::findGoal(): tries = %u, time_{cur, avr, min, max} = {%f, %f, %f, %f}, err_cur_{lin, ang} = {%f, %f}, err_avr_{lin, ang} = {%f, %f}, solution = #%u, cost = %f, var = %f, steps = %d, collisions = %u\n",
			trials, elasped, tAvr/trials, tMin, tMax, errLin, errAng, errLinAvr/trials, errAngAvr/trials,
			solution, pOptimisation->getValues()[solution], pOptimisation->getVariance(), pOptimisation->getGenerations(), pHeuristic->collisions);
#endif
		return true;
	}
	catch (const golem::Message& msg) {
		Kinematics::context.write(msg);
		return false;
	}
}

//------------------------------------------------------------------------------
