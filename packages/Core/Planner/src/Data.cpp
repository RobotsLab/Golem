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

#include <Golem/Planner/Data.h>
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(Heuristic::ChainDesc &val, XMLContext* context, bool create) {
	ASSERT(context)

	try {
		XMLData("enabled_obs", val.enabledObs, context, create);
	}
	catch (const MsgXMLParserAttributeNotFound&) {
		// optional
	}
	XMLData("enabled_lin", val.enabledLin, context, create);
	XMLData("enabled_ang", val.enabledAng, context, create);
	XMLData("dist_norm", val.distNorm, context, create);
	XMLData("dist_linear_max", val.distLinearMax, context, create);
	XMLData("dist_angular_max", val.distAngularMax, context, create);
	XMLData("dist_configspace_workspace_norm", val.distConfigspaceWorkspaceNorm, context, create);

	try {
		val.bounds.clear();
		golem::XMLData(val.bounds, val.bounds.max_size(), context, "bounds", false);
	}
	catch (const MsgXMLParser&) {
		// optional
	}
}

void golem::XMLData(Heuristic::JointDesc &val, XMLContext* context, bool create) {
	ASSERT(context)

	XMLData("enabled", val.enabled, context, create);
	try {
		XMLData("interpolate", val.interpolate, context, create);
	}
	catch (const MsgXMLParser&) {
		// optional
	}

	XMLData("dflt_pos", val.dfltPos, context, create);
	XMLData("dist_dflt_fac", val.distDfltFac, context, create);
	XMLData("dist_max", val.distMax, context, create);

	XMLData("collision_bounds", val.collisionBounds, context, create);
	std::stringstream ss(context->getAttribute("collision_joints")); // e.g. "1,2,4,8,9"
	U32 index;
	while (ss >> index) {
		val.collisionJoints.push_back(index);
		if (ss.peek() == ',') ss.ignore();
	}

	try {
		val.bounds.clear();
		golem::XMLData(val.bounds, val.bounds.max_size(), context, "bounds", false);
	}
	catch (const MsgXMLParser&) {
		// optional
	}
}

void golem::XMLData(Heuristic::CostDesc &val, XMLContext* context, bool create) {
	ASSERT(context)

	XMLData("dist_root_fac", val.distRootFac, context, create);
	XMLData("dist_dflt_fac", val.distDfltFac, context, create);
	XMLData("dist_limits_fac", val.distLimitsFac, context, create);
}

void golem::XMLData(Heuristic::CollisionDesc &val, XMLContext* context, bool create) {
	ASSERT(context)

	XMLData("enabled", val.enabled, context, create);
	XMLData("path_dist_delta", val.pathDistDelta, context, create);
	XMLData("skin_thickness", val.skinThickness, context, create);
}

void golem::XMLData(Heuristic::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)

	XMLData(val.chains, Chainspace::DIM, context, "chain", create);
	XMLData(val.joints, Configspace::DIM, context, "joint", create);
	XMLData(val.costDesc, context->getContextFirst("cost", create), create);
	XMLData(val.collisionDesc, context->getContextFirst("collision", create), create);
}

//------------------------------------------------------------------------------

void golem::XMLData(Kinematics::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
}

void golem::XMLData(Profile::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)

	XMLData(val.velocity, context->getContextFirst("velocity"), create);
	XMLData(val.acceleration, context->getContextFirst("acceleration"), create);

	try {
		XMLData("pruning", val.pruning, context, create);
		XMLData("pruning_distance", val.pruningDistance, context, create);
	}
	catch (const MsgXMLParserAttributeNotFound&) {
	}
}

//------------------------------------------------------------------------------

void golem::XMLData(Planner::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)

	XMLData(*val.pProfileDesc.get(), context->getContextFirst("profile"), create);
	XMLData(*val.pHeuristicDesc.get(), context->getContextFirst("heuristic"), create);
}

//------------------------------------------------------------------------------
