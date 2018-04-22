/** @file GraphPlanner.cpp
 * 
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/HBPlanner/GraphPlanner.h>
#include <Golem/Planner/GraphPlanner/Data.h>
#include <Golem/Sys/Message.h>
#include <Golem/Sys/Context.h>
#include <algorithm>
#include <iomanip> // std::setprecision

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

//std::string hbplannerDebug(Planner& planner) {
//	std::stringstream str;
//
//	const Heuristic& heuristic = planner.getHeuristic();
//	HBHeuristic *pHeuristic = dynamic_cast<HBHeuristic*>(&planner.getHeuristic());
//	const Controller& controller = planner.getController();
//	const Heuristic::ChainDesc::ChainSeq& chainDesc = pHeuristic->getChainDesc();
//	const Heuristic::JointDesc::JointSeq& jointDesc = pHeuristic->getJointDesc();
//	const Chainspace::Range chains = controller.getStateInfo().getChains();
//	U32 enabled = 0, enabledObs = 0;
//	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i) {
//		if (chainDesc[i]->enabledObs) ++enabledObs;
//		const Configspace::Range joints = controller.getStateInfo().getJoints(i);
//		for (Configspace::Index j = joints.begin(); j < joints.end(); ++j)
//			if (jointDesc[j]->enabled) ++enabled;
//	}
//
//	str << controller.getName() << ": chains=" << controller.getStateInfo().getChains().size() << "(enabledObs=" << enabledObs << "), joints=" << controller.getStateInfo().getJoints().size() << "(enabled=" << enabled << "), collisions=" << (heuristic.getDesc().collisionDesc.enabled ? "ENABLED" : "DISABLED") << ", non-Euclidian metrics=" << (pHeuristic && pHeuristic->enableUnc ? "ENABLE" : "DISABLE") << ", point collisions=" << (pHeuristic && pHeuristic->getPointCloudCollision() ? "ENABLE" : "DISABLE");
//
//	return str.str();
//}
//
//std::string hbplannerConfigspaceDebug(Planner& planner, const ConfigspaceCoord* c) {
//	std::stringstream str;
//
//	const Heuristic& heuristic = planner.getHeuristic();
//	const Controller& controller = planner.getController();
//	const Heuristic::JointDesc::JointSeq& jointDesc = heuristic.getJointDesc();
//	const Chainspace::Range chains = controller.getStateInfo().getChains();
//	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i) {
//		str << std::setprecision(3) << "{" << controller.getChains()[i]->getName() << ": ";
//		const Configspace::Range joints = controller.getStateInfo().getJoints(i);
//		for (Configspace::Index j = joints.begin(); j < joints.end(); ++j) {
//			str << "(" << j - controller.getStateInfo().getJoints().begin() << ", ";
//			str << (jointDesc[j]->enabled ? "Y" : "N") << ", ";
//			if (c != nullptr) str << (*c)[j] << "/";
//			str << jointDesc[j]->dfltPos << (j < joints.end() - 1 ? "), " : ")");
//		}
//		str << (i < chains.end() - 1 ? "}, " : "}");
//	}
//
//	return str.str();
//}
//
//std::string hbplannerWorkspaceDebug(Planner& planner, const WorkspaceChainCoord* w) {
//	std::stringstream str;
//
//	const Heuristic& heuristic = planner.getHeuristic();
//	const Controller& controller = planner.getController();
//	const Heuristic::ChainDesc::ChainSeq& chainDesc = heuristic.getChainDesc();
//	const Chainspace::Range chains = controller.getStateInfo().getChains();
//	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i) {
//		str << std::setprecision(3) << "{" << controller.getChains()[i]->getName() << ": ";
//		if (w != nullptr) {
//			const Vec3 p((*w)[i].p);
//			const Quat q((*w)[i].R);
//			chainDesc[i]->enabledLin ? str << "lin=(" << p.x << ", " << p.y << ", " << p.z << "), " : str << "lin=N, ";
//			chainDesc[i]->enabledAng ? str << "ang=(" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << "), " : str << "ang=N";
//		}
//		else {
//			chainDesc[i]->enabledLin ? str << "lin=Y, " : str << "lin=N, ";
//			chainDesc[i]->enabledAng ? str << "ang=Y, " : str << "ang=N";
//		}
//
//		str << (i < chains.end() - 1 ? "}, " : "}");
//	}
//
//	return str.str();
//}
//
//------------------------------------------------------------------------------

Planner::Desc::Ptr RagGraphPlanner::Desc::load(Context* context, const std::string& libraryPath, const std::string& configPath) {
	Planner::Desc::Ptr pDesc;
	
	context->debug("RagPlanner::Desc::load(): loading library %s and config %s.xml...\n", libraryPath.c_str(), configPath.c_str());
	
	// TODO load library rather than explicitly create GraphPlanner description
	RagGraphPlanner::Desc* pGraphPlannerDesc = new RagGraphPlanner::Desc;
	pDesc.reset(pGraphPlannerDesc);

	// load config
	XMLData(*pGraphPlannerDesc, XMLParser::load(configPath + ".xml")->getContextRoot()->getContextFirst("golem planner"));

	return pDesc;
}

Planner::Desc::Ptr RagGraphPlanner::Desc::load(Context* context, XMLContext* xmlcontext) {
	// driver and config paths must be specified in xmlcontext
	std::string libraryPath, configPath;
	XMLData("library_path", libraryPath, xmlcontext);
	XMLData("config_path", configPath, xmlcontext);
	// load driver and config
	return load(context, libraryPath, configPath);
}

//------------------------------------------------------------------------------

RagGraphPlanner::RagGraphPlanner(Controller& controller) :	GraphPlanner(controller) {
}

bool RagGraphPlanner::create(const Desc& desc) {
	GraphPlanner::create(desc); // throws	
	
	jointDescSeq.reserve(handInfo.getJoints().size());
	for (Configspace::Index j = handInfo.getJoints().begin(); j < handInfo.getJoints().end(); ++j)
		jointDescSeq.push_back(pHeuristic->getJointDesc()[j]->enabled);

	enableUnc = false;

//	context.debug("RagGraphPlanner::create: %s\n", hbplannerDebug(*this).c_str());
	return true;
}

//------------------------------------------------------------------------------

HBHeuristic* RagGraphPlanner::getHBHeuristic() const {
	 return dynamic_cast<HBHeuristic*>(pHeuristic.get()); 
}

void RagGraphPlanner::disableHandPlanning() {
	for (Configspace::Index j = handInfo.getJoints().begin(); j < handInfo.getJoints().end(); ++j) {
		getHBHeuristic()->getJointDesc()[j]->enabled = false;
//		pLocalPathFinder->getHeuristic().getJointDesc()[j]->enabled = false;
	}
}

void RagGraphPlanner::enableHandPlanning() {
	for (Configspace::Index j = handInfo.getJoints().begin(); j < handInfo.getJoints().end(); ++j) {
		const size_t i = (size_t)(j - handInfo.getJoints().begin());
		getHBHeuristic()->getJointDesc()[j]->enabled = jointDescSeq[i];
//		pLocalPathFinder->getHeuristic().getJointDesc()[j]->enabled = jointDescSeq[i];
	}	
}

//------------------------------------------------------------------------------

bool RagGraphPlanner::localFind(const ConfigspaceCoord &begin, const ConfigspaceCoord &end, Waypoint::Seq &localPath) {
	enableHandPlanning();
//	context.write("localFind: %s\n", hbplannerDebug(*this).c_str());

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
	disableHandPlanning();
	return true;
}

//------------------------------------------------------------------------------

bool RagGraphPlanner::findTarget(const GenConfigspaceState &begin, const GenWorkspaceChainState& wend, GenConfigspaceState &cend) {
	CriticalSectionWrapper csw(csCommand);

	HBHeuristic *heuristic = getHBHeuristic();
	if (heuristic) {
		enableUnc = heuristic->enableUnc;
		heuristic->enableUnc = false;
		context.debug("RagGraphPlanner::findTarget(): enable unc %s\n", heuristic->enableUnc ? "ON" : "OFF");
	}
	// TODO: Find why the pre-grasp pose returns with close fingers
	disableHandPlanning();
//	context.debug("findTarget: %s\n", hbplannerDebug(*this).c_str());
	//context.write("RagGraphPlanner::findTarget: %s\n", hbplannerConfigspaceDebug(*this).c_str());
	//context.write("RagGraphPlanner::findTarget: %s\n", hbplannerWorkspaceDebug(*this).c_str());

#ifdef _HBHEURISTIC_PERFMON
	heuristic->resetLog();
	heuristic->getCollision()->resetLog();
#endif
#ifdef _HBGRAPHPLANNER_PERFMON
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

#ifdef _HBGRAPHPLANNER_PERFMON
	context.write(
		"GraphPlanner::findTarget(): time_elapsed = %f [sec]\n", t.elapsed()
		);
#endif
#ifdef _HBHEURISTIC_PERFMON
	heuristic->writeLog(context, "GraphPlanner::findTarget()");
	heuristic->getCollision()->writeLog(context, "GraphPlanner::findTarget()");;
#endif

	if (heuristic)
		heuristic->enableUnc = enableUnc;

	//enableHandPlanning();

//	context.write("RagGraphPlanner::findTarget(): done.\n");
	return true;
}

//------------------------------------------------------------------------------

bool RagGraphPlanner::findGlobalTrajectory(const Controller::State &begin, const Controller::State &end, Controller::Trajectory &trajectory, Controller::Trajectory::iterator iter, const GenWorkspaceChainState* wend) {
	CriticalSectionWrapper csw(csCommand);

#ifdef _HBGRAPHPLANNER_PERFMON
	PerfTimer t;
#endif

#ifdef _HBGRAPHPLANNER_PERFMON
#ifdef _HBHEURISTIC_PERFMON
	HBHeuristic::resetLog();
	HBCollision::resetLog();
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::resetLog();
#endif
#endif

	getCallbackDataSync()->syncCollisionBounds();

#ifdef _HBGRAPHPLANNER_PERFMON
	t.reset();
#endif
	// generate global graph only for the arm
	context.debug("GraphPlanner::findGlobalTrajectory(): Enabled Uncertainty %s. disable hand planning...\n", getHBHeuristic()->enableUnc ? "ON" : "OFF");
	disableHandPlanning();
//	context.write("findGlobalTrajectory(): %s\n", hbplannerDebug(*this).c_str());
	//context.write("GraphPlanner::findGlobalTrajectory(): %s\n", hbplannerConfigspaceDebug(*this).c_str());
	//context.write("GraphPlanner::findGlobalTrajectory(): %s\n", hbplannerWorkspaceDebug(*this).c_str());

	// generate global graph
	pGlobalPathFinder->generateOnlineGraph(begin.cpos, end.cpos);
	// find node path on global graph
	globalPath.clear();
	if (!pGlobalPathFinder->findPath(end.cpos, globalPath, globalPath.begin())) {
		context.error("GlobalPathFinder::findPath(): unable to find global path\n");
		return false;
	}
#ifdef _HBGRAPHPLANNER_PERFMON
	context.write(
		"GlobalPathFinder::findPath(): time_elapsed = %f [sec], len = %d\n",
		t.elapsed(), globalPath.size()
		);
#endif

	if (pLocalPathFinder != NULL) {
#ifdef _HBGRAPHPLANNER_PERFMON
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
#ifdef _HBGRAPHPLANNER_PERFMON
		context.write(
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

#ifdef _HBGRAPHPLANNER_PERFMON
#ifdef _HBHEURISTIC_PERFMON
//	context.debug("Enabled Uncertainty %s\n", getHBHeuristic()->enableUnc ? "ON" : "OFF");
	//HBHeuristic::writeLog(context, "PathFinder::find()");
	HBCollision::writeLog(context, "PathFinder::find()");
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::writeLog(context, "PathFinder::find()");
#endif
#endif

#ifdef _HBGRAPHPLANNER_PERFMON
#ifdef _HBHEURISTIC_PERFMON
	HBHeuristic::resetLog();
	HBCollision::resetLog();
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::resetLog();
#endif
	t.reset();
#endif
	optimize(optimisedPath, begin.cacc, end.cacc);
#ifdef _HBGRAPHPLANNER_PERFMON
	context.write(
		"GraphPlanner::optimize(): time_elapsed = %f [sec], len = %d\n", t.elapsed(), optimisedPath.size()
		);
#ifdef _HBHEURISTIC_PERFMON
	HBHeuristic::writeLog(context, "GraphPlanner::optimize()");
	HBCollision::writeLog(context, "GraphPlanner::optimize()");
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::writeLog(context, "GraphPlanner::optimize()");
#endif
#endif

#ifdef _HBGRAPHPLANNER_PERFMON
	t.reset();
#endif
	Controller::Trajectory::iterator iend = iter;
	pProfile->create(optimisedPath.begin(), optimisedPath.end(), begin, end, trajectory, iter, iend);
	pProfile->profile(trajectory, iter, iend);
#ifdef _HBGRAPHPLANNER_PERFMON
	context.write(
		"GraphPlanner::profile(): time_elapsed = %f [sec], len = %d\n",
		t.elapsed(), trajectory.size()
		);
#endif

	getCallbackDataSync()->syncFindTrajectory(trajectory.begin(), trajectory.end(), wend);

	return true;
}

//------------------------------------------------------------------------------

bool RagGraphPlanner::findLocalTrajectory(const Controller::State &cbegin, GenWorkspaceChainState::Seq::const_iterator wbegin, GenWorkspaceChainState::Seq::const_iterator wend, Controller::Trajectory &trajectory, Controller::Trajectory::iterator iter, MSecTmU32 timeOut) {
	CriticalSectionWrapper csw(csCommand);

#ifdef _HBGRAPHPLANNER_PERFMON
	PerfTimer t;
#ifdef _HBHEURISTIC_PERFMON
	HBHeuristic::resetLog();
	HBCollision::resetLog();
#endif
#endif
	HBHeuristic *heuristic = getHBHeuristic();
	if (heuristic/* && heuristic->enableUnc*/)
		enableHandPlanning();

//	context.write("findLocalTrajectory(): %s\n", hbplannerDebug(*this).c_str());
	//context.write("RagGraphPlanner::findLocalTrajectory: %s\n", hbplannerConfigspaceDebug(*this).c_str());
	//context.write("RagGraphPlanner::findLocalTrajectory: %s\n", hbplannerWorkspaceDebug(*this).c_str());

	// trajectory size
	const size_t size = 1 + (size_t)(wend - wbegin);
	// check initial size
	if (size < 2) {
		context.error("GraphPlanner::findLocalTrajectory(): Invalid workspace sequence size\n");
		return false;
	}
	// time out
	const MSecTmU32 segTimeOut = timeOut == MSEC_TM_U32_INF ? MSEC_TM_U32_INF : timeOut / MSecTmU32(size - 1);
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
	pKinematics->setDistRootFac(pKinematics->getDesc().distRootLocalFac);

	// find configspace trajectory
	PARAMETER_GUARD(Heuristic, GenCoordConfigspace, Min, *pHeuristic);
	PARAMETER_GUARD(Heuristic, GenCoordConfigspace, Max, *pHeuristic);
	for (size_t i = 1; i < size; ++i) {
		// pointers
		const Controller::Trajectory::iterator c[2] = { begin + i - 1, begin + i };
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


#ifdef _HBGRAPHPLANNER_PERFMON
	context.write("GraphPlanner::findLocalTrajectory(): time_elapsed = %f [sec], len = %d\n", t.elapsed(), size);
#ifdef _HBHEURISTIC_PERFMON
	if (heuristic) {
		context.write("Enabled Uncertainty %s\n", heuristic->enableUnc ? "ON" : "OFF");
		//heuristic->writeLog(context, "GraphPlanner::findTarget()");
		heuristic->getCollision()->writeLog(context, "GraphPlanner::findTarget()");;
	}
#endif
#endif

	if (heuristic/* && heuristic->enableUnc*/)
		disableHandPlanning();

	return true;
}

//------------------------------------------------------------------------------

void golem::XMLData(RagGraphPlanner::Desc &val, XMLContext* context, bool create) {
	XMLData((golem::GraphPlanner::Desc&)val, context, create);
}

//------------------------------------------------------------------------------