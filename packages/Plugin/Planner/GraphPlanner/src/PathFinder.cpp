/** @file PathFinder.cpp
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

#include <Golem/Planner/GraphPlanner/PathFinder.h>
#include <Golem/Sys/Message.h>
#include <Golem/Sys/Context.h>
#include <algorithm>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

/** Setup generator given heuristic */
WaypointGenerator::WaypointGenerator(const Heuristic &heuristic, const WaypointGenerator::Desc& desc) : heuristic(heuristic), stateInfo(heuristic.getController().getStateInfo()) {
	if (!desc.isValid())
		throw MsgPathFinder(Message::LEVEL_CRIT, "WaypointGenerator.%s::WaypointGenerator(): invalid description");

	Sample<Real>::setToDefault();
	
	weight = desc.weight;
	cdf = REAL_ZERO;
	
	name = desc.name;
	seed = desc.seed;
	trials = desc.trials;

	bandwidthFactor = desc.bandwidthFactor;
	bandwidthTrials = desc.bandwidthTrials;

	mean = desc.mean;

	min = max = mean;
	if (bandwidthTrials > 0) {
		minLo.resize(bandwidthTrials, mean);
		maxLo.resize(bandwidthTrials, mean);
		minHi.resize(bandwidthTrials, mean);
		maxHi.resize(bandwidthTrials, mean);
	}

	const Configspace::Range range = heuristic.getController().getStateInfo().getJoints();
	for (Configspace::Index i = range.begin(); i < range.end(); ++i) {
		const Heuristic::JointDesc* jointDesc = heuristic.getJointDesc()[i];
		if (!jointDesc->enabled)
			continue;

		min[i] = std::max(heuristic.getMin()[i].pos, mean[i] - desc.delta[i]);
		max[i] = std::min(heuristic.getMax()[i].pos, mean[i] + desc.delta[i]);

		for (U32 j = 0; j < bandwidthTrials; ++j) {
			// low bandwidth
			const Real loFac = Math::pow(REAL_ONE / bandwidthFactor, Real(j + 1));
			minLo[j][i] = std::max(heuristic.getMin()[i].pos, mean[i] - loFac*desc.delta[i]);
			maxLo[j][i] = std::min(heuristic.getMax()[i].pos, mean[i] + loFac*desc.delta[i]);

			// high bandwidth
			const Real hiFac = Math::pow(bandwidthFactor, Real(j + 1));
			minLo[j][i] = std::max(heuristic.getMin()[i].pos, mean[i] - hiFac*desc.delta[i]);
			maxLo[j][i] = std::min(heuristic.getMax()[i].pos, mean[i] + hiFac*desc.delta[i]);
		}
	}
}

bool WaypointGenerator::create(Heuristic::ThreadData* data, const ConfigspaceCoord& cc, Waypoint& w) const {
	w.setup(heuristic.getController(), cc, true, true);
	w.cost = Node::COST_INF; // invalidate cost
	return !heuristic.collides(w, data);
}

bool WaypointGenerator::next(Heuristic::ThreadData* data, const ConfigspaceCoord& min, const ConfigspaceCoord& max, const Rand& rand, Waypoint& w) const {
	// attempt to generate non-colliding waypoint
	for (U32 k = 0; k < trials; ++k) {
		ConfigspaceCoord cc;
		for (Configspace::Index i = stateInfo.getJoints().begin(); i < stateInfo.getJoints().end(); ++i)
			cc[i] = rand.nextUniform(min[i], max[i]);

		if (create(data, cc, w))
			return true;
	}

	// failed
	return false;
}

bool WaypointGenerator::next(Heuristic::ThreadData* data, const Rand& rand, Waypoint& w, U32 bandwidthIter) const {
	// default bandwidth: attempt to generate non-colliding waypoint
	if (next(data, min, max, rand, w))
		return true;

	// adapt bandwidth
	if (bandwidthIter > 0 && bandwidthIter <= bandwidthTrials) {
		// low bandwidth
		if (next(data, minLo[bandwidthIter - 1], maxLo[bandwidthIter - 1], rand, w))
			return true;
		// high bandwidth
		if (next(data, minHi[bandwidthIter - 1], maxHi[bandwidthIter - 1], rand, w))
			return true;
	}

	return false;
}

//------------------------------------------------------------------------------

PathFinder::PathFinder(Heuristic& heuristic) :
	heuristic(heuristic), controller(heuristic.getController()), context(controller.getContext()),
	rand(context.getRandSeed()),
	stateInfo(heuristic.getController().getStateInfo()),
	graphSearchForward(Node::IDX_GOAL, Node::IDX_ROOT),
	graphSearchBackward(Node::IDX_ROOT, Node::IDX_GOAL),
	costMax(Node::COST_ZERO)
{}

PathFinder::~PathFinder() {
}

void PathFinder::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgPathFinder(Message::LEVEL_ERROR, "PathFinder.%s::create(): invalid description", desc.name.c_str());

	name = desc.name;

	generatorsOffline = desc.generatorsOffline;
	generatorsOnline = desc.generatorsOnline;
	graphSizeOnline = desc.graphSizeOnline;
	graphSizeOffline = desc.graphSizeOffline;
	graphNeighbours = desc.graphNeighbours;

	allocateGraph(desc.graphSizeOnline, desc.graphSizeOffline, desc.graphNeighbours);

	generateOfflineGraph();
}

//------------------------------------------------------------------------------

void PathFinder::allocateGraph(U32 graphSizeOnline, U32 graphSizeOffline, U32 graphNeighbours) {
	this->graphPartition = 2 + graphSizeOnline;

	graph.resize(graphPartition + graphSizeOffline);
	neighbourMap.resize(graph.size());
	for (size_t i = graphPartition; i < graph.size(); ++i)
		neighbourMap[i].reserve(graphNeighbours);
}

void PathFinder::generateGraph(U32 begin, U32 end, const WaypointGenerator::Desc::Seq& generators) {
	//context.debug("PathFinder.%s::generateGraph(): bounds=%u, generators = %u\n", name.c_str(), (U32)heuristic.getCollisionBounds().size(), (U32)generators.size());

	if (begin >= end)
		throw MsgPathFinder(Message::LEVEL_ERROR, "PathFinder.%s::generateGraph(): invalid graph range <%u, %u)", name.c_str(), begin, end);
	if (graph.size() < end)
		throw MsgPathFinder(Message::LEVEL_ERROR, "PathFinder.%s::generateGraph(): graph range too large %u", name.c_str(), end);
	if (generators.empty())
		throw MsgPathFinder(Message::LEVEL_ERROR, "PathFinder.%s::generateGraph(): no generators specified", name.c_str());

#ifdef _PATHFINDER_PERFMON
	PerfTimer t;
#endif

	struct Task : public Runnable {
		typedef std::map<WaypointGenerator::Seq::const_iterator, U32> BandwidthIterMap;

		PathFinder* pathFinder;
		Waypoint::Seq& graph;
		WaypointGenerator::Seq generators;
		U32 begin;
		const U32 end;
		BandwidthIterMap bandwidthIterMap;
		golem::CriticalSection cs;

		Task(PathFinder* pathFinder, const WaypointGenerator::Desc::Seq& generators, U32 begin, U32 end) : pathFinder(pathFinder), begin(begin), end(end), graph(pathFinder->graph) {
			// create generators
			for (WaypointGenerator::Desc::Seq::const_iterator i = generators.begin(); i != generators.end(); ++i) {
				// root and goal generators have root and goal means
				if ((*i)->seed == WaypointGenerator::SEED_ROOT)
					(*i)->mean = pathFinder->graph[Node::IDX_ROOT].cpos;
				else if ((*i)->seed == WaypointGenerator::SEED_GOAL)
					(*i)->mean = pathFinder->graph[Node::IDX_GOAL].cpos;
				// create
				this->generators.push_back((*i)->create(pathFinder->heuristic));
			}
			
			// normalise generators
			if (!WaypointGenerator::normalise<Ref2>(this->generators))
				throw MsgPathFinder(Message::LEVEL_ERROR, "PathFinder.%s::generateGraph(): unable to normalise generators", pathFinder->name.c_str());
			
			// generate means specified by user
			for (WaypointGenerator::Seq::const_iterator i = this->generators.begin(); i != this->generators.end() && this->begin < this->end; ++i)
				if ((*i)->getSeed() == WaypointGenerator::SEED_USER && (*i)->create(pathFinder->heuristic.getThreadData(), (*i)->getMean(), graph[this->begin]))
					++this->begin;
			
			// update bandwidth iterators
			for (WaypointGenerator::Seq::const_iterator i = this->generators.begin(); i != this->generators.end(); ++i)
				bandwidthIterMap[i] = 0;

			// run Parallels
			Parallels *parallels = pathFinder->context.getParallels();
			if (parallels != NULL) {
				// launch Parallels
				for (U32 t = parallels->getNumOfThreads(); t > 0; --t) {
					Job* job = parallels->startJob(this);
					if (!job)
						throw MsgPathFinder(Message::LEVEL_ERROR, "PathFinder.%s::generateGraph(): Unable to start job", pathFinder->name.c_str());
				}
				(void)parallels->joinJobs(MSEC_TM_U32_INF);
			}
			else
				run();

			// check if generation failed
			if (this->begin < end)
				throw MsgPathFinder(Message::LEVEL_ERROR, "PathFinder.%s::generateGraph(): failed to generate online graph", pathFinder->name.c_str());
		}

		void run() {
			Heuristic::ThreadData* data = pathFinder->heuristic.getThreadData();

			WaypointGenerator::Seq::const_iterator waypointGenerator = generators.end();
			for (U32 index, bandwidthIter;;) {
				{
					CriticalSectionWrapper csw(cs);
					if (waypointGenerator != generators.end()) {
						// failure
						if (bandwidthIter >= bandwidthIterMap[waypointGenerator]) {
							if (++bandwidthIterMap[waypointGenerator] <= (*waypointGenerator)->getBandwidthTrials())
								pathFinder->getHeuristic().getContext().verbose("PathFinder::generateGraph(): unable to generate random waypoint: generator=%s, bandwidth_{step=%u, range=(%f, %f)}\n",
									(*waypointGenerator)->getName().c_str(), bandwidthIter + 1, Math::pow(REAL_ONE / (*waypointGenerator)->getBandwidthFactor(), Real(bandwidthIter + 1)), Math::pow((*waypointGenerator)->getBandwidthFactor(), Real(bandwidthIter + 1)));
							else
								return; // failure: no more adaptation steps to try
						}
					}
					else {
						// success
						if (begin >= end)
							return; // success: no more waypoints to generate
						index = begin++;
						graph[index].index = index;
					}

					// sample generators, update iterator
					waypointGenerator = Sample<Real>::sample<Ref2, WaypointGenerator::Seq::const_iterator>(generators, pathFinder->rand);
					bandwidthIter = bandwidthIterMap[waypointGenerator];
				}

				if (!(*waypointGenerator)->next(data, pathFinder->rand, graph[index], bandwidthIter))
					continue; // failure
				waypointGenerator = generators.end(); // success
			}
		}
	} task(this, generators, begin, end);

#ifdef _PATHFINDER_PERFMON
	context.debug(
		"PathFinder.%s::generateGraph(): time_elapsed = %f [sec], graph_{partition, size, begin, end} = {%u, %u, %u, %u}, generators = %u\n", name.c_str(), t.elapsed(), graphPartition, graph.size(), begin, end, generators.size()
	);
#endif
}

void PathFinder::generateNeighbourMap(U32 begin, U32 end) {
	if (begin >= end)
		throw MsgPathFinder(Message::LEVEL_ERROR, "PathFinder.%s::generateNeighbourMap(): invalid graph range <%u, %u)", name.c_str(), begin, end);
	if (graph.size() < end)
		throw MsgPathFinder(Message::LEVEL_ERROR, "PathFinder.%s::generateNeighbourMap(): graph range too large %u", name.c_str(), end);

#ifdef _PATHFINDER_PERFMON
	PerfTimer t;
#endif

	struct Task : public Runnable {
		const Heuristic& heuristic;
		Waypoint::Seq& graph;
		IndexCostSeqSeq& neighbourMap;
		const U32 begin, end;
		U32 i;
		Real costMax;

		golem::CriticalSection cs;

#ifdef _PATHFINDER_PERFMON
		U32 size;
#endif

		Task(PathFinder* pathFinder, U32 begin, U32 end) : heuristic(pathFinder->getHeuristic()), begin(begin), end(end), graph(pathFinder->graph), neighbourMap(pathFinder->neighbourMap) {
			// initialise the first index
			i = begin;
			costMax = Node::COST_ZERO;
#ifdef _PATHFINDER_PERFMON
			size = 0;
#endif
			// run Parallels
			Parallels *parallels = pathFinder->context.getParallels();
			if (parallels != NULL) {
				// launch Parallels
				for (U32 t = parallels->getNumOfThreads(); t > 0; --t) {
					Job* job = parallels->startJob(this);
					if (!job)
						throw MsgPathFinder(Message::LEVEL_ERROR, "PathFinder.%s::generateNeighbourMap(): Unable to start job", pathFinder->name.c_str());
				}
				(void)parallels->joinJobs(MSEC_TM_U32_INF);
			}
			else
				run();

			pathFinder->costMax = costMax;
		}

		void run() {
			Heuristic::ThreadData* data = heuristic.getThreadData();

			U32 i = Node::IDX_UINI;
			for (Real costMax = Node::COST_ZERO;;) {
				{
					CriticalSectionWrapper csw(cs);
					if (i != Node::IDX_UINI) {
#ifdef _PATHFINDER_PERFMON
						size += (U32)neighbourMap[i].size();
#endif
						if (this->costMax < costMax)
							this->costMax = costMax;
					}
					if (this->i >= end) return; // no more path costs to generate
					i = this->i++;
				}
				
				IndexCostSeq& neighbourMap = this->neighbourMap[i];
				for (U32 j = begin; j < end; ++j)
					if (i != j) {
						const Real cost = heuristic.cost(graph[i], graph[j]);
						if (cost < Node::COST_INF) {
							neighbourMap.push_back(std::make_pair(j, cost));
							if (costMax < cost)
								costMax = cost;
						}
					}
			}
		}
	} task(this, begin, end);

#ifdef _PATHFINDER_PERFMON
	context.debug(
		"PathFinder.%s::generateNeighbourMap(): time_elapsed = %f [sec], graph_{begin, end} = {%u, %u}, neighbour_map_size = %u\n", name.c_str(), t.elapsed(), begin, end, task.size
	);
#endif
}

void PathFinder::setupGraph(const ConfigspaceCoord& croot, const ConfigspaceCoord& cgoal, const WorkspaceChainCoord& wgoal) {
	// setup GOAL graph waypoint (incomplete information)
	ASSERT(Node::IDX_GOAL == 0)
	Waypoint& goal = this->graph[Node::IDX_GOAL];
	goal.index = Node::IDX_GOAL;
	goal.cpos = cgoal; // may not be known, but should never be random
	goal.wpos = wgoal; // always given, required by cost(const Waypoint&)
	for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i)
		goal.qrot[i].fromMat33(goal.wpos[i].R); // required by cost(const Waypoint&)
	goal.cost = Node::COST_INF; // required by both: PathFinder::findGoal() and PathFinder::findPath()

	// setup ROOT graph waypoint
	ASSERT(Node::IDX_ROOT == 1)
	Waypoint& root = this->graph[Node::IDX_ROOT];
	root.index = Node::IDX_ROOT;
	root.cpos = croot;
	root.setup(controller, true, true);
	root.cost = heuristic.cost(root, graph[Node::IDX_ROOT], graph[Node::IDX_GOAL]); // infinite cost not allowed, but collisions yes
	if (root.cost >= Node::COST_INF)
		throw MsgPathFinder(Message::LEVEL_ERROR, "PathFinder.%s::setupGraph(): root waypoint has infinite cost", name.c_str());

#ifdef _PATHFINDER_PERFMON
	PerfTimer t;
#endif

	// generate graph waypoint's cost
	struct Task : public Runnable {
		const Heuristic& heuristic;
		Waypoint::Seq& graph;
		U32 index;
		golem::CriticalSection cs;

		Task(PathFinder* pathFinder) : heuristic(pathFinder->getHeuristic()), graph(pathFinder->graph) {
			// initialise the first index
			index = 0;
			// run Parallels
			Parallels *parallels = pathFinder->context.getParallels();
			if (parallels != NULL) {
				// launch Parallels
				for (U32 t = parallels->getNumOfThreads(); t > 0; --t) {
					Job* job = parallels->startJob(this);
					if (!job)
						throw MsgPathFinder(Message::LEVEL_ERROR, "PathFinder.%s::setupGraph(): Unable to start job", pathFinder->name.c_str());
				}
				(void)parallels->joinJobs(MSEC_TM_U32_INF);
			}
			else
				run();
		}

		void run() {
			Heuristic::ThreadData* data = heuristic.getThreadData();

			U32 i = Node::IDX_UINI;
			Real cost = Node::COST_INF;
			bool collides = false;
			for (;;) {

				{
					CriticalSectionWrapper csw(cs);
					
					if (i != Node::IDX_UINI) {
						graph[i].cost = cost;
						graph[i].collides = collides;
					}

					if (index >= graph.size()) return; // no more waypoint costs to generate
					i = index++;
				}

				// root and goal may be in a collision state, but they are excluded from the collision report
				cost = heuristic.cost(graph[i], graph[Node::IDX_ROOT], graph[Node::IDX_GOAL]);
				collides = i != Node::IDX_ROOT && i != Node::IDX_GOAL && (cost >= Node::COST_INF || heuristic.collides(graph[i], data));
				if (collides)
					cost = Node::COST_INF;
			}
		}
	} task(this);

#ifdef _PATHFINDER_PERFMON
	context.debug(
		"PathFinder.%s::setupGraph(): time_elapsed = %f [sec], graph_size = %u\n", name.c_str(), t.elapsed(), graph.size()
	);
#endif
}

//------------------------------------------------------------------------------

void PathFinder::generateOfflineGraph(const WaypointGenerator::Desc::Seq* generators) {
	if (graphPartition < (U32)graph.size()) {
		generateGraph(graphPartition, (U32)graph.size(), generators ? *generators : generatorsOffline);
		generateNeighbourMap(graphPartition, (U32)graph.size());
	}
}

void PathFinder::generateOnlineGraph(const ConfigspaceCoord &croot, const ConfigspaceCoord &cgoal, const WaypointGenerator::Desc::Seq* generators) {
	if (2 < graphPartition)
		generateGraph(2, graphPartition, generators ? *generators : generatorsOnline); // use custom generators if available

	WorkspaceChainCoord wgoal;
	controller.chainForwardTransform(cgoal, wgoal); // compute wgoal
	setupGraph(croot, cgoal, wgoal);
}

void PathFinder::generateOnlineGraph(const ConfigspaceCoord &croot, const WorkspaceChainCoord &wgoal, const WaypointGenerator::Desc::Seq* generators) {
	if (2 < graphPartition)
		generateGraph(2, graphPartition, generators ? *generators : generatorsOnline); // use custom generators if available
	
	setupGraph(croot, croot, wgoal);
}

//------------------------------------------------------------------------------

bool PathFinder::findPath(const ConfigspaceCoord &goal, Waypoint::Seq &path, Waypoint::Seq::iterator iter) {
#ifdef _HEURISTIC_PERFMON
	heuristic.resetLog();
#endif
#ifdef _PATHFINDER_PERFMON
	PerfTimer t;
#endif

	// update goal node
	graph[Node::IDX_GOAL].setup(controller, goal, true, true);
#ifdef _PATHFINDER_PERFMON
	if (heuristic.collides(graph[Node::IDX_GOAL]))
		context.info("PathFinder::findPath(): collision at the goal node\n");
#endif

	// find node path
	Node::Seq nodePath;
	BidirectionalSearch search(this, nodePath);

	for (Node::Seq::const_iterator i = nodePath.begin(); i != nodePath.end(); ++i)
		iter = ++path.insert(iter, graph[i->index]);

#ifdef _PATHFINDER_PERFMON
	context.debug(
		"PathFinder.%s::findPath(): time_elapsed = %f [sec], len = %u, cost_{online, offline} = {%u, %u}, collision_{online, offline, online_state} = {%u, %u, %u}\n",
		name.c_str(), t.elapsed(), nodePath.size(),
		search.graphHeuristicForward.costOnline + search.graphHeuristicBackward.costOnline,
		search.graphHeuristicForward.costOffline + search.graphHeuristicBackward.costOffline,
		search.graphHeuristicForward.collisionOnline + search.graphHeuristicBackward.collisionOnline,
		search.graphHeuristicForward.collisionOffline + search.graphHeuristicBackward.collisionOffline,
		search.graphHeuristicForward.collisionOnlineState + search.graphHeuristicBackward.collisionOnlineState
		);
#endif
#ifdef _HEURISTIC_PERFMON
	heuristic.writeLog(context, "PathFinder::findPath()");
#endif

	if (nodePath.size() < 2) {
		// path not found
		context.info("PathFinder.%s::findPath(): cannot find path\n", name.c_str());
		return false;
	}

	return true;
}

//------------------------------------------------------------------------------
