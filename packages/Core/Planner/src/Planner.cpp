/** @file Planner.cpp
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

#include <Golem/Planner/Planner.h>
#include <Golem/Planner/Data.h>
#include <Golem/Sys/Message.h>
#include <Golem/Sys/Context.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Planner::Desc::Ptr Planner::Desc::load(Context* context, const std::string& libraryPath, const std::string& configPath) {
	Planner::Desc::Ptr pDesc;

	context->debug("Planner::Desc::load(): loading library %s and config %s.xml...\n", libraryPath.c_str(), configPath.c_str());

	// open library and load function
	Handle library = context->getLibrary(libraryPath);
	LoadObjectDesc loadDesc = (LoadObjectDesc)library.getFunction("loadPlannerDesc");

	// load config
	golem::XMLParser::Ptr parser;
	try {
		// first try to load directly the specified config
		parser = XMLParser::load(configPath + ".xml");
	}
	catch (const golem::Message&) {
		// if failed, attempt to load from library location
		parser = XMLParser::load(library.getDir() + configPath + ".xml");
	}
	loadDesc(context, parser->getContextRoot()->getContextFirst("golem planner"), &pDesc);

	pDesc->libraryPath = libraryPath;
	pDesc->configPath = configPath;

	return pDesc;
}

Planner::Desc::Ptr Planner::Desc::load(Context* context, XMLContext* xmlcontext) {
	// driver and config paths must be specified in xmlcontext
	std::string libraryPath, configPath;
	XMLData("library_path", libraryPath, xmlcontext);
	XMLData("config_path", configPath, xmlcontext);
	// load driver and config
	return load(context, libraryPath, configPath);
}

//------------------------------------------------------------------------------

Planner::Planner(golem::Controller& controller) :
	controller(controller),
	context(controller.getContext()),
	rand(context.getRandSeed()),
	pCallbackDataSync(NULL)
{
}

Planner::~Planner() {
}

void Planner::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgPlannerInvalidDesc(Message::LEVEL_CRIT, "Planner::create(): Invalid description");

	libraryPath = desc.libraryPath;
	configPath = desc.configPath;
	// type and id
	const std::string header = "GolemPlanner";
	type = libraryPath.substr(libraryPath.find(header) == 0 ? header.length() : 0, std::string::npos);
	id = type + "+" + configPath.substr(configPath.find(header) == 0 ? header.length() : 0, std::string::npos);

	stateInfo = controller.getStateInfo();

	pHeuristic = desc.pHeuristicDesc->create(controller); // throws
	pKinematics = desc.pKinematicsDesc->create(*pHeuristic); // throws
	
	desc.pProfileDesc->pCallbackDist = this;
	pProfile = desc.pProfileDesc->create(controller); // throws
}

//------------------------------------------------------------------------------

Real Planner::distConfigspaceCoord(const ConfigspaceCoord& prev, const ConfigspaceCoord& next) const {
	Real dist = REAL_ZERO;
	for (Configspace::Index i = stateInfo.getJoints().begin(); i < stateInfo.getJoints().end(); ++i)
		if (pHeuristic->getJointDesc()[i]->enabled)
			dist += Math::sqr(prev[i] - next[i]);
	return Math::sqrt(dist);
}

Real Planner::distCoord(Real prev, Real next) const {
	return Math::abs(prev - next);
}

bool Planner::distCoordPlanning(const Configspace::Index& index) const {
	return pHeuristic->getJointDesc()[index]->enabled;
}

bool Planner::distCoordInterpolation(const Configspace::Index& index) const {
	return pHeuristic->getJointDesc()[index]->interpolate;
}

//------------------------------------------------------------------------------

Controller::State Planner::makeDiffConstraintBegin(const Controller::Trajectory& trajectory) const {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Planner::makeDiffConstraintBegin(): at least two waypoints required");
	Controller::State state = trajectory.front();
	for (golem::Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
		state.cacc[j] = (trajectory.begin() + 1)->cpos[j] - (trajectory.begin())->cpos[j];
	return state;
}

Controller::State Planner::makeDiffConstraintEnd(const Controller::Trajectory& trajectory) const {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Planner::makeDiffConstraintEnd(): at least two waypoints required");
	Controller::State state = trajectory.back();
	for (golem::Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
		state.cacc[j] = (trajectory.rbegin() + 1)->cpos[j] - (trajectory.rbegin())->cpos[j];
	return state;
}

//------------------------------------------------------------------------------
