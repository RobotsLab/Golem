/** @file GraphPlanner.cpp
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

#include <Golem/Planner/GraphPlanner/GraphPlanner.h>
#include <Golem/Planner/GraphPlanner/Data.h>
#include <Golem/Sys/Message.h>
#include <Golem/Sys/Context.h>
#include <algorithm>

//------------------------------------------------------------------------------

/** Performance monitor */
//#define _GRAPHPLANNER_PERFMON

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadPlannerDesc(void* pContext, void* pXMLContext, void* pPlannerDesc) {
	//loadObjectDesc<golem::GraphPlanner::Desc, Planner::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Planner::Desc::Ptr*)pPlannerDesc);

	// initialise context in a module
	((Context*)pContext)->initModule();

	// Create description and load xml configuration
	GraphPlanner::Desc* pDesc = new GraphPlanner::Desc;
	((golem::shared_ptr<Planner::Desc>*)pPlannerDesc)->reset(pDesc);
	golem::XMLData(*pDesc, (XMLContext*)pXMLContext);
}

//------------------------------------------------------------------------------

GraphPlanner::GraphPlanner(golem::Controller& controller) :	Planner(controller) {
}

void GraphPlanner::create(const Desc& desc) {
	Planner::create(desc); // throws

	if (desc.localFinderDesc.range.size() < (size_t)stateInfo.getJoints().size())
		throw MsgPlannerInvalidDesc(Message::LEVEL_CRIT, "GraphPlanner::create(): Invalid size of local range");

	pKinematics = dynamic_cast<DEKinematics*>(this->Planner::pKinematics.get());
	if (pKinematics == NULL)
		throw MsgPlanner(Message::LEVEL_CRIT, "GraphPlanner::create(): DEKinematics kinematics solver required");

	pathFinderDesc = desc.pathFinderDesc;

	pGlobalPathFinder = pathFinderDesc.pGlobalPathFinderDesc->create(*pHeuristic); // throws
	if (pathFinderDesc.pLocalPathFinderDesc != NULL) {
		pLocalPathFinder = pathFinderDesc.pLocalPathFinderDesc->create(*pHeuristic); // throws
	}
	
	pathOptimisationDesc = desc.pathOptimisationDesc;
	localFinderDesc = desc.localFinderDesc;
}

//------------------------------------------------------------------------------

bool GraphPlanner::localFind(const ConfigspaceCoord &begin, const ConfigspaceCoord &end, Waypoint::Seq &localPath) {
	Real scale = REAL_ONE;
	for (U32 i = 0; i < pathFinderDesc.numOfIterations; ++i) {
		// scale maximum distance between waypoints
		scale *= pathFinderDesc.distScaleFac;
		pHeuristic->setScale(scale);

		// set graph generators delta
		ConfigspaceCoord delta;
		for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
			delta[j] = pathFinderDesc.rangeFac*pHeuristic->getJointDesc()[j]->distMax;

		// create online graph generators
		WaypointGenerator::Desc::Seq generators;
		for (Waypoint::Seq::const_iterator j = localPath.begin(); j != localPath.end(); ++j) {
			WaypointGenerator::Desc::Ptr generator(new WaypointGenerator::Desc);
			generator->name = "local";
			generator->delta = delta;
			generator->mean = j->cpos;
			generator->seed = j == localPath.begin() ? WaypointGenerator::SEED_ROOT : j == --localPath.end() ? WaypointGenerator::SEED_GOAL : WaypointGenerator::SEED_USER;
			generators.push_back(generator);
		}

		// allocate and generate local graph
		pLocalPathFinder->allocateGraph((U32)(scale*localPath.size()*pLocalPathFinder->getOnlineGraphSize()), pLocalPathFinder->getOnlineGraphSize(), pLocalPathFinder->getGraphNeighbours());
		pLocalPathFinder->generateOnlineGraph(begin, end, &generators);

		// find node path on local graph
		localPath.clear();
		if (!pLocalPathFinder->findPath(end, localPath, localPath.begin()))
			return false;
	}

	return true;
}

//------------------------------------------------------------------------------

void GraphPlanner::optimize(Waypoint::Seq &path, const ConfigspaceCoord &dbegin, const ConfigspaceCoord &dend) const {
	if (path.size() < 3) // nothing to optimise
		return;

#ifdef _GRAPHPLANNER_PERFMON
	PerfTimer t;
	const U32 pathSize = (U32)path.size();
	static Real saCost = REAL_ZERO;
	static U32 saN = 0;
#endif

	// generate graph waypoint's cost
	struct Task : public Runnable {
		const PathOptimisationDesc desc;
		const Controller::State::Info stateInfo;
		const Heuristic& heuristic;
		const Profile::CallbackDist& callbackDist;
		Waypoint::Seq &path;
		const ConfigspaceCoord dbegin;
		const ConfigspaceCoord dend;
		bool hasDiffBegin, hasDiffEnd;
		U32 index, indexMax;
		golem::CriticalSection cs;
		Configspace::Map indexMap;

#ifdef _GRAPHPLANNER_PERFMON
		U32 saGreedy, saTemp, collisions;
		Real saCost;
#endif

		inline static Real getWeight(const Waypoint& w0, const Waypoint& w1) {
			const Real w0c = Math::sqr(w0.cost);
			const Real w1c = Math::sqr(w1.cost);
			return Math::abs(w0c - w1c);
			//return REAL_ONE;
		}
		inline static Real getConfigspaceDistDiff(const Configspace::Map& indexMap, const ConfigspaceCoord& c0, const ConfigspaceCoord& c1, const ConfigspaceCoord& dc) {
			Real dist = REAL_ZERO, ln0 = REAL_ZERO, ln1 = REAL_ZERO;
			for (Configspace::Map::const_iterator i = indexMap.begin(); i != indexMap.end(); ++i) {
				const Real l0 = c1[*i] - c0[*i];
				const Real l1 = dc[*i];
				dist += l0 * l1; // dot product
				ln0 += Math::sqr(l0);
				ln1 += Math::sqr(l1);
			}

			// dist = 1 - norm(dc) * norm(c1 - c0)
			const Real norm = Math::sqrt(ln0*ln1);
			return norm > REAL_EPS ? REAL_ONE - dist / norm : REAL_ZERO;
		}

		Real makeDiffCost(size_t ptr, const Waypoint& w0, const Waypoint& w1) const {
			// begin constraint at each waypoint: distBeg = dist = 0..1
			// end constraint at each waypoint: distEnd = 1 - distBeg = 1..0

			Real cost = REAL_ZERO;

			if (hasDiffBegin) {
				const Real weight = desc.diffBeginFac * Math::exp(-REAL_HALF*(w0.dist + w1.dist) * desc.diffDist);
				const Real diff = getConfigspaceDistDiff(indexMap, w0.cpos, w1.cpos, w0.dbegin);
				cost += weight * diff;
			}
			if (hasDiffEnd) {
				const Real weight = desc.diffEndFac * Math::exp(-(REAL_ONE - REAL_HALF*(w0.dist + w1.dist)) * desc.diffDist);
				const Real diff = getConfigspaceDistDiff(indexMap, w0.cpos, w1.cpos, w1.dend);
				cost += weight * diff;
			}

			return cost;
		}

		Task(const GraphPlanner* graphPlanner, Waypoint::Seq &path, const ConfigspaceCoord &dbegin, const ConfigspaceCoord &dend) : desc(graphPlanner->pathOptimisationDesc), stateInfo(graphPlanner->stateInfo), heuristic(graphPlanner->getHeuristic()), callbackDist(*graphPlanner), path(path), dbegin(dbegin), dend(dend) {
			// there must be at least 3 waypoints
			if (path.size() < 3)
				return;

			// initialise index
			index = 0;
			for (Configspace::Index i = stateInfo.getJoints().begin(); i < stateInfo.getJoints().end(); ++i)
				if (heuristic.getJointDesc()[i]->enabled)
					indexMap.push_back(i);
#ifdef _GRAPHPLANNER_PERFMON
			saGreedy = saTemp = collisions = 0;
			saCost = REAL_ZERO;
#endif

			hasDiffBegin = desc.diffBeginFac > REAL_EPS && dbegin.magnitudeSqr(indexMap) > REAL_EPS;
			hasDiffEnd = desc.diffEndFac > REAL_EPS && dend.magnitudeSqr(indexMap) > REAL_EPS;

			// setup path, compute reference cost as the maximum path segment cost
			for (size_t i = 1; i < path.size(); ++i) {
				Waypoint* w[2] = {&path[i - 1], &path[i]};
				// update cost
				w[1]->cost = heuristic.cost(*w[0], *w[1]);
				if (w[1]->cost >= Node::COST_INF)
					throw MsgPlanner(Message::LEVEL_ERROR, "GraphPlanner::optimize(): Infinite path cost at segment #%u", i);
				w[1]->cost += makeDiffCost(i, *w[0], *w[1]);
				// set unique index excluding root and goal waypoints
				if (i < path.size() - 1) w[1]->index = -(I32)i;
			}

			// try to remove all redundant waypoints
			const Real referenceCost = desc.distPathThr*graphPlanner->pGlobalPathFinder->getCostMax()*Math::pow(graphPlanner->pathFinderDesc.distScaleFac, Real(graphPlanner->pathFinderDesc.numOfIterations));
			while (purge(referenceCost, 1, path.size() - 1, path, heuristic.getThreadData()));

			// update weights
			for (size_t i = 0; i < path.size(); ++i)
				path[i].weight = i <= 0 || i >= path.size() - 1 ? REAL_ZERO : getWeight(path[i], path[i + 1]);
			Waypoint::normalise<Ref1>(path);

			// update distance
			if (hasDiffBegin || hasDiffEnd) {
				Waypoint::makeDist(path, [=](const ConfigspaceCoord &c0, const ConfigspaceCoord &c1) -> Real { return callbackDist.distConfigspaceCoord(c0, c1); });
				Waypoint::makeDiff(path, indexMap, dbegin, dend);
			}

			// number of steps
			indexMax = (U32)path.size()*desc.numOfIterations;

			// run Parallels
			Parallels *parallels = graphPlanner->context.getParallels();
			if (parallels != NULL) {
				// launch Parallels
				for (U32 t = parallels->getNumOfThreads(); t > 0; --t) {
					Job* job = parallels->startJob(this);
					if (!job)
						throw MsgPlanner(Message::LEVEL_ERROR, "GraphPlanner::optimize(): Unable to start job");
				}
				(void)parallels->joinJobs(MSEC_TM_U32_INF);
			}
			else
				run();

			// check validity
			Heuristic::ThreadData* data = heuristic.getThreadData();
			for (size_t i = 1; i < path.size(); ++i) {
				Waypoint* w[2] = {&path[i - 1], &path[i]};
				// check collisions, do not test root and goal
				if (i > 1 && heuristic.collides(*w[0], data))
					throw MsgPlanner(Message::LEVEL_ERROR, "GraphPlanner::optimize(): Collision at waypoint #%u", i - 1);
				if (heuristic.collides(*w[0], *w[1], data))
					throw MsgPlanner(Message::LEVEL_ERROR, "GraphPlanner::optimize(): Collision at segment (%u, %u)", i - 1, i);
			}
		}

		bool purge(Real referenceCost, size_t begin, size_t end, Waypoint::Seq &path, Heuristic::ThreadData* data) const {
			// there must be at least 4 waypoints
			if (path.size() < 4)
				return false;

			// rank path costs
			typedef std::map<Real, size_t> Rank;
			Rank rank;
			for (size_t i = begin; i < end; ++i)
				rank[path[i].cost] = i;

			// try to remove too dense path segments
			for (Rank::const_iterator i = rank.begin(); i != rank.end(); ++i) {
				// check if the segment can be removed, if not, the others cannot be removed either
				if (path[i->second].cost > referenceCost)
					return false;
				// compute segment cost after removing the current waypoint (always possible)
				const Real rightCost = heuristic.cost(path[i->second - 1], path[i->second + 1]) + makeDiffCost(i->second + 1, path[i->second - 1], path[i->second + 1]);
				// and the waypoint before to see if it is an better option
				const Real leftCost = i->second > 1 ? heuristic.cost(path[i->second - 2], path[i->second]) + makeDiffCost(i->second, path[i->second - 2], path[i->second]) : Node::COST_INF;
				// attempt to remove the least costly waypoint
				// if it is the "right" one
				if (rightCost < leftCost) {
					if (!heuristic.collides(path[i->second - 1], path[i->second + 1], data)) {
						path[i->second + 1].cost = rightCost;
						path.erase(path.begin() + i->second);
						return true;
					}
				}
				// or the "left" one
				if (leftCost < Node::COST_INF) {
					if (!heuristic.collides(path[i->second - 2], path[i->second], data)) {
						path[i->second].cost = leftCost;
						path.erase(path.begin() + i->second - 1);
						return true;
					}
				}
			}

			return false;
		}
		void run() {
			Heuristic::ThreadData* data = heuristic.getThreadData();
			//Rand rand(heuristic.getController().getContext().getRandSeed());
			const U32 jobId = heuristic.getController().getContext().getParallels()->getCurrentJob()->getJobId();
			Rand rand(RandSeed(heuristic.getController().getContext().getRandSeed()._U32[0] + jobId, (U32)0));
			Waypoint wprev, wcurr, wnext;
			ConfigspaceCoord cprev, cnext;
			U32 ptr, index;
			bool bcprev, bcnext, update = false;

			for (;;) {
				{
					CriticalSectionWrapper csw(cs);

					// data to update?
					if (update) {
						update = false;
						// check if they are no update collisions
						if (path[ptr - 1].index == wprev.index && path[ptr].index == wcurr.index && path[ptr + 1].index == wnext.index) {
							path[ptr] = wcurr;
							path[ptr].index = index;
							path[ptr + 1].cost = wnext.cost; // new cost
							path[ptr + 1].index = index;
							if (hasDiffBegin || hasDiffEnd) {
								Waypoint::makeDist(path, [=](const ConfigspaceCoord &c0, const ConfigspaceCoord &c1) -> Real { return callbackDist.distConfigspaceCoord(c0, c1); });
								Waypoint::makeDiff(path, indexMap, dbegin, dend);
							}
							Waypoint::normalise<Ref1>(path);
						}
#ifdef _GRAPHPLANNER_PERFMON
						else
							++collisions;
#endif
					}

					// assign new index and path size
					if (this->index >= indexMax) return;
					index = ++this->index;

					// select random waypoint
					//ptr = 1 + rand.next()%((U32)path.size() - 2);
					ptr = (U32)(Waypoint::sample<Ref1, Waypoint::Seq::const_iterator>(path, rand) - path.begin());

					// cache waypoints in the nearest neighbourhood of ptr
					wprev = path[ptr - 1];
					wnext = path[ptr + 1];
					wcurr = path[ptr];
					// this is optonal
					bcprev = ptr > 1;
					if (bcprev) cprev = path[ptr - 2].cpos;
					bcnext = ptr < path.size() - 2;
					if (bcnext) cnext = path[ptr + 2].cpos;
				}

				// temperature cooling schedule
				const Real T = desc.Tinit + (desc.Tfinal - desc.Tinit)*index/indexMax;

				// correlate changes with local configuration space ranges: isotropic in the configuration space
				//Real rangeSeg[4] = {REAL_ZERO};
				//for (size_t j = 0; j < indexMap.size(); ++j) {
				//	const Configspace::Index i = indexMap[j];
				//	if (bcprev) rangeSeg[0] += Math::sqr(wprev.cpos[i] - cprev[i]);
				//	rangeSeg[1] += Math::sqr(wcurr.cpos[i] - wprev.cpos[i]);
				//	rangeSeg[2] += Math::sqr(wcurr.cpos[i] - wnext.cpos[i]);
				//	if (bcnext) rangeSeg[3] += Math::sqr(wnext.cpos[i] - cnext[i]);
				//}
				//const Real range = Real(2.0)*(Math::sqrt(rangeSeg[1]) + Math::sqrt(rangeSeg[2])) + Real(1.0)*(Math::sqrt(rangeSeg[0]) + Math::sqrt(rangeSeg[3]));

				// generate a candidate solution
				const size_t crossPoint = rand.next()%indexMap.size();
				for (size_t j = 0; j < indexMap.size(); ++j) {
					const Configspace::Index i = indexMap[(crossPoint + j)%indexMap.size()];

					// correlate changes with local configuration space ranges: exploit non-isotropic configuration space variations 
					Real range = Real(2.0)*(Math::abs(wcurr.cpos[i] - wprev.cpos[i]) + Math::abs(wcurr.cpos[i] - wnext.cpos[i]));
					if (bcprev) range += Math::abs(wprev.cpos[i] - cprev[i]);
					if (bcnext) range += Math::abs(wnext.cpos[i] - cnext[i]);

					// generate new coordinate
					wcurr.cpos[i] = Math::clamp(wcurr.cpos[i] + range*rand.nextUniform(-T, +T), heuristic.getMin()[i].pos, heuristic.getMax()[i].pos);

					if (desc.crossProb < rand.nextUniform<Real>())
						break;
				}

				// setup waypoint
				wcurr.setup(heuristic.getController(), true, true);

				// optimise squared mutual waypoint distance to reinforce uniform waypoint distribution on a path

				// old cost
				const Real costOld = Math::sqr(wcurr.cost) + Math::sqr(wnext.cost);

				// modification of a single waypoint on path affects neighbours as well
				wcurr.cost = heuristic.cost(wprev, wcurr);
				if (wcurr.cost >= Node::COST_INF)
					continue;
				wnext.cost = heuristic.cost(wcurr, wnext);
				if (wnext.cost >= Node::COST_INF)
					continue;

				// differential constraints: <prev, curr> segment
				wcurr.cost += makeDiffCost(ptr, wprev, wcurr);
				// differential constraints: <curr, next> segment
				wnext.cost += makeDiffCost(ptr + 1, wcurr, wnext);

				// new cost
				const Real costNew = Math::sqr(wcurr.cost) + Math::sqr(wnext.cost);

				// weights
				wcurr.weight = getWeight(wcurr, wnext);

				// SA parameters
				const Real dC = costNew - costOld;
				const Real dE = desc.Enorm*dC;
				const bool bGreedy = dE < REAL_ZERO;
				const bool bTemp = !bGreedy && Math::exp(-dE) > rand.nextUniform<Real>();

				// update old cost if the new cost is lower and there are no collisions
				if ((bGreedy || bTemp) && !heuristic.collides(wcurr, data) && !heuristic.collides(wprev, wcurr, data) && !heuristic.collides(wcurr, wnext, data)) {
					update = true;
#ifdef _GRAPHPLANNER_PERFMON
					if (bGreedy) ++saGreedy;
					if (bTemp) ++saTemp;
					saCost += -dC;
#endif //_GRAPHPLANNER_PERFMON
				}
			}
		}
	} task(this, path, dbegin, dend);

#ifdef _GRAPHPLANNER_PERFMON
	saCost += task.saCost/task.indexMax;
	++saN;
	context.debug(
		"GraphPlanner::optimize(): time_elapsed = %f [sec], path_size %u -> %u, steps = %u, collisions = %u, sa_{greedy, temp} = {%u, %u}, perfmon_cost = {%.8f, %.8f}\n",
		t.elapsed(), pathSize, path.size(), task.indexMax, task.collisions, task.saGreedy, task.saTemp, task.saCost/task.indexMax, saCost/saN
	);
#endif //_GRAPHPLANNER_PERFMON
}

//------------------------------------------------------------------------------

bool GraphPlanner::findTarget(const GenConfigspaceState &begin, const GenWorkspaceChainState& wend, GenConfigspaceState &cend) {
	CriticalSectionWrapper csw(csCommand);

#ifdef _HEURISTIC_PERFMON
	pHeuristic->resetLog();
#endif
#ifdef _GRAPHPLANNER_PERFMON
	PerfTimer t;
#endif

	getCallbackDataSync()->syncCollisionBounds();

	// generate graph
	pGlobalPathFinder->generateOnlineGraph(begin.cpos, wend.wpos);

	// create waypoints pointers
	const Waypoint::Seq& graph = pGlobalPathFinder->getGraph();
	WaypointPtr::Seq waypointPtrGraph;
	waypointPtrGraph.reserve(graph.size());
	for (U32 i = 0; i < graph.size(); ++i)
		waypointPtrGraph.push_back(WaypointPtr(&graph[i]));

	// Create waypoint population
	const U32 populationSize = std::min(U32(pKinematics->getDesc().populationSize), (U32)waypointPtrGraph.size());

	// sort waypoints (pointers) from the lowest to the highest cost
	std::partial_sort(waypointPtrGraph.begin(), waypointPtrGraph.begin() + populationSize, waypointPtrGraph.end(), WaypointPtr::cost_less());

	// create initial population for kinematics solver
	ConfigspaceCoord::Seq population;
	population.reserve(populationSize);
	for (WaypointPtr::Seq::const_iterator i = waypointPtrGraph.begin(); population.size() < populationSize && i != waypointPtrGraph.end(); ++i)
		population.push_back((*i)->cpos);
	pKinematics->setPopulation(&population);

	// set global root distance factor
	pKinematics->setDistRootFac(pKinematics->getDesc().distRootGlobalFac);

	GenConfigspaceState root;
	root.setToDefault(controller.getStateInfo().getJoints().begin(), controller.getStateInfo().getJoints().end());
	root.cpos = graph[Node::IDX_ROOT].cpos;

	// find the goal state
	if (!pKinematics->findGoal(root, wend, cend)) {
		context.error("GraphPlanner::findTarget(): unable to find target\n");
		return false;
	}

	cend.t = wend.t;
	cend.cvel.fill(REAL_ZERO);
	cend.cacc.fill(REAL_ZERO);

#ifdef _GRAPHPLANNER_PERFMON
	context.debug(
		"GraphPlanner::findTarget(): time_elapsed = %f [sec]\n", t.elapsed()
	);
#endif
#ifdef _HEURISTIC_PERFMON
	pHeuristic->writeLog(context, "GraphPlanner::findTarget()");
#endif

	return true;
}

//------------------------------------------------------------------------------

bool GraphPlanner::findGlobalTrajectory(const golem::Controller::State &begin, const golem::Controller::State &end, Controller::Trajectory &trajectory, Controller::Trajectory::iterator iter, const GenWorkspaceChainState* wend) {
	CriticalSectionWrapper csw(csCommand);

#ifdef _GRAPHPLANNER_PERFMON
	PerfTimer t;
#endif
	
#ifdef _GRAPHPLANNER_PERFMON
#ifdef _HEURISTIC_PERFMON
	Heuristic::resetLog();
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::resetLog();
#endif
#endif

	getCallbackDataSync()->syncCollisionBounds();

#ifdef _GRAPHPLANNER_PERFMON
	t.reset();
#endif
	// generate global graph
	pGlobalPathFinder->generateOnlineGraph(begin.cpos, end.cpos);

	// find node path on global graph
	globalPath.clear();
	if (!pGlobalPathFinder->findPath(end.cpos, globalPath, globalPath.begin())) {
		context.error("GlobalPathFinder::findPath(): unable to find global path\n");
		return false;
	}
#ifdef _GRAPHPLANNER_PERFMON
	context.debug(
		"GlobalPathFinder::findPath(): time_elapsed = %f [sec], len = %d\n",
		t.elapsed(), globalPath.size()
	);
#endif

	if (pLocalPathFinder != NULL) {
#ifdef _GRAPHPLANNER_PERFMON
		t.reset();
#endif
		PARAMETER_GUARD(Heuristic, Real, Scale, *pHeuristic);

		for (U32 i = 0;;) {
			localPath = globalPath;
			if (localFind(begin.cpos, end.cpos, localPath))
				break;
			else if (++i > pathFinderDesc.numOfTrials) {
				context.error("LocalPathFinder::findPath(): unable to find local path\n");
				return false;
			}
		}
#ifdef _GRAPHPLANNER_PERFMON
		context.debug(
			"LocalPathFinder::findPath(): time_elapsed = %f [sec], len = %d\n",
			t.elapsed(), localPath.size()
		);
#endif
		// copy localPath
		optimisedPath = localPath;
	}
	else {
		// copy globalPath
		optimisedPath = globalPath;
	}

#ifdef _GRAPHPLANNER_PERFMON
#ifdef _HEURISTIC_PERFMON
	Heuristic::writeLog(context, "PathFinder::find()");
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::writeLog(context, "PathFinder::find()");
#endif
#endif

#ifdef _GRAPHPLANNER_PERFMON
#ifdef _HEURISTIC_PERFMON
	Heuristic::resetLog();
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::resetLog();
#endif
	t.reset();
#endif
	optimize(optimisedPath, begin.cacc, end.cacc);
#ifdef _GRAPHPLANNER_PERFMON
	context.debug(
		"GraphPlanner::optimize(): time_elapsed = %f [sec], len = %d\n", t.elapsed(), optimisedPath.size()
	);
#ifdef _HEURISTIC_PERFMON
	Heuristic::writeLog(context, "GraphPlanner::optimize()");
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::writeLog(context, "GraphPlanner::optimize()");
#endif
#endif

#ifdef _GRAPHPLANNER_PERFMON
	t.reset();
#endif
	
	Controller::Trajectory::iterator iend = iter;
	pProfile->create(optimisedPath.begin(), optimisedPath.end(), begin, end, trajectory, iter, iend);
	pProfile->profile(trajectory, iter, iend);
#ifdef _GRAPHPLANNER_PERFMON
	context.debug(
		"GraphPlanner::profile(): time_elapsed = %f [sec], len = %d\n",
		t.elapsed(), trajectory.size()
	);
#endif

	getCallbackDataSync()->syncFindTrajectory(trajectory.begin(), trajectory.end(), wend);
	
	return true;
}

//------------------------------------------------------------------------------

bool GraphPlanner::findLocalTrajectory(const Controller::State &cbegin, GenWorkspaceChainState::Seq::const_iterator wbegin, GenWorkspaceChainState::Seq::const_iterator wend, Controller::Trajectory &trajectory, Controller::Trajectory::iterator iter, MSecTmU32 timeOut) {
	CriticalSectionWrapper csw(csCommand);

#ifdef _GRAPHPLANNER_PERFMON
	PerfTimer t;
#endif

	// trajectory size
	const size_t size = 1 + (size_t)(wend - wbegin);
	// check initial size
	if (size < 2) {
		context.error("GraphPlanner::findLocalTrajectory(): Invalid workspace sequence size\n");
		return false;
	}
	// time out
	const MSecTmU32 segTimeOut = timeOut == MSEC_TM_U32_INF ? MSEC_TM_U32_INF : timeOut/MSecTmU32(size - 1);
	// fill trajectory with cbegin
	const Controller::State cinit = cbegin; // backup
	Controller::Trajectory::iterator end = ++trajectory.insert(iter, cinit);
	for (GenWorkspaceChainState::Seq::const_iterator i = wbegin; i != wend; ++i)
		end = ++trajectory.insert(end, cinit);
	Controller::Trajectory::iterator begin = end - size;

	getCallbackDataSync()->syncCollisionBounds();
	optimisedPath.resize(size - 1);
	population.assign(1, cbegin.cpos); // always keep initial solution in case no transformation is required
	pKinematics->setPopulation(&population);
	PARAMETER_GUARD(DEKinematics, Real, DistRootFac, *pKinematics);
	pKinematics->setDistRootFac(pKinematics->getDesc().distRootLocalFac);

	// find configspace trajectory
	PARAMETER_GUARD(Heuristic, GenCoordConfigspace, Min, *pHeuristic);
	PARAMETER_GUARD(Heuristic, GenCoordConfigspace, Max, *pHeuristic);
	for (size_t i = 1; i < size; ++i) {
		// pointers
		const Controller::Trajectory::iterator c[2] = {begin + i - 1, begin + i};
		const GenWorkspaceChainState::Seq::const_iterator w = wbegin + i - 1;
		
		// setup search limits
		GenCoordConfigspace min = pHeuristic->getMin();
		GenCoordConfigspace max = pHeuristic->getMin();
		for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j) {
			const idx_t k = j - stateInfo.getJoints().begin();
			min[j].pos = c[0]->cpos[j] - localFinderDesc.range[k];
			max[j].pos = c[0]->cpos[j] + localFinderDesc.range[k];
		}
		pHeuristic->setMin(min);
		pHeuristic->setMax(max);
		
		// and search for a solution
		if (!pKinematics->findGoal(*c[0], *w, *c[1], segTimeOut)) {
			context.error("GraphPlanner::findLocalTrajectory(): unable to solve inverse kinematics\n");
			return false;
		}
		
		// visualisation
		optimisedPath[i - 1].cpos = c[1]->cpos;
		optimisedPath[i - 1].wpos = w->wpos;
	}

	// profile configspace trajectory
	pProfile->profile(trajectory, begin, end);

	getCallbackDataSync()->syncFindTrajectory(begin, end, &*(wend - 1));

#ifdef _GRAPHPLANNER_PERFMON
	context.debug("GraphPlanner::findLocalTrajectory(): time_elapsed = %f [sec], len = %d\n", t.elapsed(), size);
#endif
	
	return true;
}

//------------------------------------------------------------------------------

DEKinematics& GraphPlanner::getKinematics() {
	return *pKinematics;
}
const DEKinematics& GraphPlanner::getKinematics() const {
	return *pKinematics;
}

PathFinder::Ptr GraphPlanner::getGlobalPathFinder() const {
	return pGlobalPathFinder;
}
void GraphPlanner::setGlobalPathFinder(const PathFinder::Ptr &pPathFinder) {
	this->pGlobalPathFinder = pPathFinder;
}

PathFinder::Ptr GraphPlanner::getLocalPathFinder() const {
	return pLocalPathFinder;
}
void GraphPlanner::setLocalPathFinder(const PathFinder::Ptr &pPathFinder) {
	this->pLocalPathFinder = pPathFinder;
}

const GraphPlanner::PathOptimisationDesc &GraphPlanner::getPathOptimisationDesc() const {
	return pathOptimisationDesc;
}
void GraphPlanner::setPathOptimisationDesc(const PathOptimisationDesc &pathOptimisationDesc) {
	this->pathOptimisationDesc = pathOptimisationDesc;
}

//------------------------------------------------------------------------------

