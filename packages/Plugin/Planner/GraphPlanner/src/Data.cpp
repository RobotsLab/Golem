/** @file Data.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Planner/GraphPlanner/Data.h>
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(DEKinematics::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)

	XMLData((Kinematics::Desc&)val, context, create);
	val.load(context);

	XMLData("dist_root_global_fac", val.distRootGlobalFac, context, create);
	XMLData("dist_root_local_fac", val.distRootLocalFac, context, create);
}

//------------------------------------------------------------------------------

void golem::XMLData(WaypointGenerator::Desc &val, XMLContext* context, bool create) {
	XMLData("name", val.name, context, create);

	std::string seed;
	XMLData("seed", seed, context, create);
	val.seed =
		seed.compare("root") == 0 ? WaypointGenerator::SEED_ROOT :
		seed.compare("goal") == 0 ? WaypointGenerator::SEED_GOAL : WaypointGenerator::SEED_USER;

	XMLData("weight", val.weight, context, create);
	XMLData("trials", val.trials, context, create);

	XMLData(val.delta, "c", context->getContextFirst("delta"), create);
	try {
		XMLData(val.mean, "c", context->getContextFirst("mean"), create);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}

	XMLData("trials", val.bandwidthTrials, context->getContextFirst("bandwidth"), create);
	XMLData("factor", val.bandwidthFactor, context->getContextFirst("bandwidth"), create);
}

void golem::XMLData(WaypointGenerator::Desc::Ptr &val, XMLContext* context, bool create) {
	// decide which WaypointGenerator to instantiate
	val.reset(new WaypointGenerator::Desc);
	XMLData(*val, context, create);
}

void golem::XMLData(PathFinder::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)

	XMLData("name", val.name, context, create);

	try {
		XMLData(val.generatorsOffline, val.generatorsOffline.max_size(), context, "generator_offline", create);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}
	try {
		XMLData(val.generatorsOnline, val.generatorsOnline.max_size(), context, "generator_online", create);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}

	XMLData("graph_size_online", val.graphSizeOnline, context, create);
	XMLData("graph_size_offline", val.graphSizeOffline, context, create);
	XMLData("graph_neighbours", val.graphNeighbours, context, create);
}

//------------------------------------------------------------------------------

void golem::XMLData(GraphPlanner::PathFinderDesc &val, XMLContext* context, bool create) {
	ASSERT(context)

	XMLData(*val.pGlobalPathFinderDesc.get(), context->getContextFirst("global_path_finder"), create);
	XMLData(*val.pLocalPathFinderDesc.get(), context->getContextFirst("local_path_finder"), create);

	XMLData("dist_scale_fac", val.distScaleFac, context, create);
	XMLData("range_fac", val.rangeFac, context, create);
	XMLData("num_iterations", val.numOfIterations, context, create);
	XMLData("num_trials", val.numOfTrials, context, create);
}

void golem::XMLData(GraphPlanner::PathOptimisationDesc &val, XMLContext* context, bool create) {
	ASSERT(context)

	XMLData("num_iterations", val.numOfIterations, context, create);
	XMLData("t_init", val.Tinit, context, create);
	XMLData("t_final", val.Tfinal, context, create);
	XMLData("e_norm", val.Enorm, context, create);
	XMLData("cross_prob", val.crossProb, context, create);
	XMLData("dist_path_thr", val.distPathThr, context, create);

	try {
		XMLData("diff_dist", val.diffDist, context, create);
	}
	catch (const MsgXMLParser&) {}
	try {
		XMLData("diff_begin_fac", val.diffBeginFac, context, create);
	}
	catch (const MsgXMLParser&) {}
	try {
		XMLData("diff_end_fac", val.diffEndFac, context, create);
	}
	catch (const MsgXMLParser&) {}
}

void golem::XMLData(GraphPlanner::LocalFinderDesc &val, XMLContext* context, bool create) {
	ASSERT(context)

	XMLData(val.range, context->getContextFirst("range"), create);
}

void golem::XMLData(GraphPlanner::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)

	XMLData((Planner::Desc&)val, context, create);

	XMLData(val.pathFinderDesc, context->getContextFirst("global_finder"), create);
	XMLData(val.pathOptimisationDesc, context->getContextFirst("optimisation"), create);
	XMLData(val.localFinderDesc, context->getContextFirst("local_finder"), create);

	XMLData(*static_cast<DEKinematics::Desc*>(val.pKinematicsDesc.get()), context->getContextFirst("kinematics"), create);
}

//------------------------------------------------------------------------------
