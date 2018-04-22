/** @file Player.cpp
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/App/Player.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <Golem/App/Data.h>
#include <Golem/Ctrl/CtrlServer/CtrlServer.h>
#include <iomanip>
#include <GL/glut.h>

// debugging of the collision model
//#define _GOLEM_PLAYER_COLLISION_DEBUG_
#ifdef _GOLEM_PLAYER_COLLISION_DEBUG_
#include <Golem/Contact/Collision.h>
#pragma comment(lib, "GolemContact.lib")
#endif // _GOLEM_PLAYER_COLLISION_DEBUG_

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(golem::Player::PlannerInfo::Seq::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	val.controllerIDSeq.clear();
	golem::XMLData(val.controllerIDSeq, golem::Chainspace::DIM, xmlcontext, "controller");

	try {
		val.sensorIDSeq.clear();
		XMLGetValue(val.sensorIDSeq, "sensor", "id", xmlcontext);
	}
	catch (const golem::MsgXMLParserNameNotFound&) {}

	golem::XMLContext* pxmlcontext = xmlcontext->getContextFirst("trajectory");
	golem::XMLData("duration", val.trajectoryDuration, pxmlcontext, false);
	golem::XMLData("idle_begin", val.trajectoryIdleBegin, pxmlcontext, false);
	golem::XMLData("idle_end", val.trajectoryIdleEnd, pxmlcontext, false);
	golem::XMLData("idle_perf", val.trajectoryIdlePerf, pxmlcontext, false);
	golem::XMLData("trials", val.trajectoryTrials, pxmlcontext, false);
	golem::XMLData("handler", val.trajectoryHandler, pxmlcontext, false);

	try {
		golem::XMLData("handler", val.workspacectrlHandler, xmlcontext->getContextFirst("workspacectrl"));
	}
	catch (const golem::MsgXMLParser&) {}
}

void golem::Player::PlannerInfo::set(golem::Planner& planner, const Sensor::Map& sensorMap) {
	this->planner = &planner;

	if (controllerIDSeq.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Player::PlannerInfo::set(): two controllers required");
	armInfo = controllerIDSeq[0].findInfo(planner.getController()); // controller #1
	handInfo = controllerIDSeq[1].findInfo(planner.getController()); // controller #2
	
	sensorSeq.clear();
	findSensor(sensorMap, sensorIDSeq, sensorSeq);
}

//------------------------------------------------------------------------------

void golem::Player::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Recorder::Desc::load(context, xmlcontext);

	golem::XMLContext* pxmlcontext = xmlcontext->getContextFirst("player");

	golem::XMLData(*uiPlannerDesc, &context, pxmlcontext, false);
	
	plannerInfoSeq.clear();
	golem::XMLData(plannerInfoSeq, plannerInfoSeq.max_size(), pxmlcontext, "planner");

	golem::XMLData("planner_index", plannerIndex, pxmlcontext);

	activectrls.clear();
	try {
		golem::XMLData(activectrls, activectrls.max_size(), pxmlcontext, "activectrl");
	}
	catch (const golem::MsgXMLParser&) {}

	try {
		objectsDesc.clear();
		golem::XMLData(objectsDesc, objectsDesc.max_size(), pxmlcontext->getContextFirst("objects"), "bounds", false);
	}
	catch (const golem::MsgXMLParser&) {}
	try {
		objectsAppearance.clear();
		golem::XMLData(objectsAppearance, objectsAppearance.max_size(), pxmlcontext->getContextFirst("objects"), "bounds", false);
	}
	catch (const golem::MsgXMLParser&) {}

	try {
		golem::XMLData(collisionPointColour, pxmlcontext->getContextFirst("collision point_colour"));
	}
	catch (const golem::MsgXMLParser&) {}

	try {
		golem::XMLData("port", serverPort, pxmlcontext->getContextFirst("server"), false);
		golem::XMLData("clients", serverClients, pxmlcontext->getContextFirst("server"), false);
		golem::XMLData("message_interval", serverMessageInterval, pxmlcontext->getContextFirst("server"), false);
	}
	catch (const golem::MsgXMLParser&) {}

	golem::XMLData("trajectory_name", trajectoryName, pxmlcontext, false);
	golem::XMLData("trajectory_profile_semi_auto", trajectoryProfileSemiAuto, pxmlcontext, false);
}

//------------------------------------------------------------------------------

golem::Player::Player(Scene &scene) : Recorder(scene), uiPlanner(nullptr), controller(nullptr), trajectoryIndex(0) {
}

void golem::Player::release() {
	// clearing map does not destroy Active controllers
	activectrlMap.clear();
	// release in the reverse order of initialisation due to the internal stack of CallbackIO interface
	for (ActiveCtrl::Seq::reverse_iterator i = activectrlSeq.rbegin(); i != activectrlSeq.rend(); ++i)
		i->release();
	// master controller is released after this

	Recorder::release();
}

void golem::Player::create(const Desc& desc) {
	desc.assertValid(Assert::Context("Player::Desc."));

	// create object
	Recorder::create(desc); // throws

	// create objects
	objects.clear();
	for (size_t i = 0; i != desc.objectsDesc.size(); ++i) {
		Actor::Desc::Ptr obstacleDesc = scene.createActorDesc();
		obstacleDesc->kinematic = true;
		obstacleDesc->appearance = desc.objectsAppearance[i];
		obstacleDesc->boundsDescSeq.push_back(desc.objectsDesc[i]);
		objects.push_back(scene.createObject(*obstacleDesc));
	}

	collisionPointColour = desc.collisionPointColour;

	// master controller and planner
	uiPlanner = is<UIPlanner>(scene.createObject(*desc.uiPlannerDesc));
	if (uiPlanner == nullptr)
		throw Message(Message::LEVEL_CRIT, "Player::create(): UIPlanner required");
	controller = &uiPlanner->getController();
	info = controller->getStateInfo();

	// planner info
	if (desc.plannerInfoSeq.size() != uiPlanner->getPlannerSeq().size())
		throw Message(Message::LEVEL_CRIT, "Player::create(): Number of planners and descriptions does not match");
	if (desc.plannerIndex >= uiPlanner->getPlannerSeq().size())
		throw Message(Message::LEVEL_CRIT, "Player::create(): Invalid planner index %u", desc.plannerIndex);
	plannerIndex = desc.plannerIndex;
	plannerInfoSeq = desc.plannerInfoSeq;
	for (size_t i = 0; i < plannerInfoSeq.size(); ++i)
		plannerInfoSeq[i].set(*uiPlanner->getPlannerSeq()[i], sensorMap);

	// active controllers
	activectrlMap.clear();
	activectrlSeq.clear();
	for (golem::Library::Path::Seq::const_iterator i = desc.activectrls.begin(); i != desc.activectrls.end(); ++i) {
		ActiveCtrl::Desc::Ptr activectrlDesc = golem::Library::Desc::loadLibrary<ActiveCtrl::Desc>(context, *i);

		if (activectrlDesc->plannerIndex >= plannerInfoSeq.size())
			throw Message(Message::LEVEL_CRIT, "Player::create(): %s: Invalid planner index %u", activectrlDesc->getID().c_str(), activectrlDesc->plannerIndex);
		PlannerInfo& plannerInfo = plannerInfoSeq[activectrlDesc->plannerIndex];

		if (activectrlDesc->controllerIDSeq.empty() && !plannerInfo.controllerIDSeq.empty())
			activectrlDesc->controllerIDSeq = plannerInfo.controllerIDSeq;

		try {
			// create active controller
			ActiveCtrl::Ptr activectrl = activectrlDesc->create(*plannerInfo.planner, plannerInfo.sensorSeq);
			activectrlMap.insert(std::make_pair(activectrl->getID(), activectrl));
			activectrlSeq.push_back(activectrl);
			// Add callback if activectrl supports it
			UI::addCallback(*this, activectrl.get());
		}
		catch (const Message& msg) {
			context.write(msg);
		}
	}
	activectrlCurrentPtr = activectrlMap.begin();

	// initialise data handlers
	for (golem::data::Handler::Map::const_iterator i = handlerMap.begin(); i != handlerMap.end(); ++i) {
		data::HandlerPlanner* handlerPlan = is<data::HandlerPlanner>(i);
		if (handlerPlan) {
			if (handlerPlan->getPlannerIndex() >= plannerInfoSeq.size())
				throw Message(Message::LEVEL_CRIT, "Player::create(): %s: Invalid planner index %u", i->second->getID().c_str(), handlerPlan->getPlannerIndex());
			PlannerInfo& plannerInfo = plannerInfoSeq[handlerPlan->getPlannerIndex()];

			handlerPlan->set(*plannerInfo.planner, plannerInfo.controllerIDSeq);
		}
	}

	serverPort = desc.serverPort;
	serverClients = desc.serverClients;
	serverMessageInterval = desc.serverMessageInterval;

	profileApproach = desc.profileApproachDesc->create(*controller);
	profileManipulation = desc.profileManipulationDesc->create(*controller);

	trajectoryName = desc.trajectoryName;
	trajectoryProfileSemiAuto = desc.trajectoryProfileSemiAuto;

	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("055", "  *                                       controller state\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F2", "  P                                       menu planner\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F3", "  A                                       menu active controller\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F4", "  T                                       menu trajectory\n"));

	// controller state
	menuCmdMap.insert(std::make_pair("*", [=] () {
		// print current robot joint position
		Controller::State state = WaypointCtrl::lookup(*controller).state;
		std::stringstream str;
		for (golem::Configspace::Index i = state.getInfo().getJoints().begin(); i < state.getInfo().getJoints().end(); ++i)
			str << " c" << (*i - *state.getInfo().getJoints().begin() + 1) << "=\"" << state.cpos[i] << "\"";
		context.write("<pose dim=\"%d\"%s/>\n", state.getInfo().getJoints().size(), str.str().c_str());
		golem::WorkspaceJointCoord wc;
		controller->jointForwardTransform(state.cpos, wc);
		for (golem::Chainspace::Index i = info.getChains().begin(); i < info.getChains().end(); ++i) {
			context.write("Name: %s\n", controller->getChains()[i]->getName().c_str());
			for (golem::Configspace::Index j = info.getJoints(i).begin(); j < info.getJoints(i).end(); ++j) {
				const U32 k = U32(j - info.getJoints(i).begin());
				const Mat34& m = wc[j];
				context.write("Joint %d: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", k, m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
			}
		}
	}));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("P", [=] (MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: (S)elect planner...";
	}));
	menuCmdMap.insert(std::make_pair("PS", [&]() {
		// select trajectory planner
		PlannerInfo::Seq::iterator plannerInfoPtr = plannerInfoSeq.begin() + plannerIndex;
		select(plannerInfoPtr, plannerInfoSeq.begin(), plannerInfoSeq.end(), "Select planner:\n", [](PlannerInfo::Seq::const_iterator ptr) -> std::string {
			return ptr->planner->getID();
		});
		if (plannerInfoPtr != plannerInfoSeq.end())
			plannerIndex = U32(plannerInfoPtr - plannerInfoSeq.begin());
		// done!
		context.write("Done!\n");
	}));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("A", [=] (MenuCmdMap& menuCmdMap, std::string& desc) {
		if (activectrlMap.empty())
			throw Cancel("No available active controllers");

		select(activectrlCurrentPtr, activectrlMap.begin(), activectrlMap.end(), "Select controller:\n", [] (ActiveCtrl::Map::const_iterator ptr) -> std::string {
			const std::string active = ptr->second->isActive() ? " (Active)" : " (Inactive)";
			return ptr->second->getID() + active;
		});

		if (activectrlCurrentPtr->second->isActive()) {
			desc = "Press a key to: (D)e-activate...";
			menuCmdMap.erase("AA");
		}
		else {
			desc = "Press a key to: (A)ctivate...";
			menuCmdMap.erase("AD");
		}
	}));
	menuCmdMap.insert(std::make_pair("AA", [=] () {
		activectrlCurrentPtr->second->setActive(true);
	}));
	menuCmdMap.insert(std::make_pair("AD", [=] () {
		activectrlCurrentPtr->second->setActive(false);
	}));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("T", [=] (MenuCmdMap& menuCmdMap, std::string& desc) {
		data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true);
		const bool isTrajectory = item != to<Data>(dataCurrentPtr)->itemMap.end() && is<data::Trajectory>(item->second.get());

		if (isTrajectory) {
			desc = "Press a key to: (C)reate/(T)ransform/(P)lay trajectory, (A)dd/(R)emove/(O)verwrite/re(V)erse waypoints, goto (W)aypoint/pos(E)...";
		}
		else {
			desc = "Press a key to: (C)reate trajectory, goto pos(E)...";
			menuCmdMap.erase("TT");
			menuCmdMap.erase("TP");
			menuCmdMap.erase("TA");
			menuCmdMap.erase("TR");
			menuCmdMap.erase("TO");
			menuCmdMap.erase("TV");
			menuCmdMap.erase("TW");
		}
	}));
	menuCmdMap.insert(std::make_pair("TC", [&]() {
		// find handlers supporting data::Trajectory
		typedef std::vector<data::Handler*> TrajectorySeq;
		TrajectorySeq trajectorySeq;
		for (data::Handler::Map::const_iterator i = handlerMap.begin(); i != handlerMap.end(); ++i)
			if (i->second->isItem<data::Trajectory>())
				trajectorySeq.push_back(i->second.get());
		if (trajectorySeq.empty())
			throw Cancel("No handlers support Trajectory interface");
		// pick up handler
		TrajectorySeq::const_iterator trajectoryPtr = trajectorySeq.begin();
		select(trajectoryPtr, trajectorySeq.begin(), trajectorySeq.end(), "Trajectory:\n", [](TrajectorySeq::const_iterator ptr) -> std::string {
			return std::string("Handler: ") + (*ptr)->getID();
		});
		// trajectory label
		readString("Enter label: ", trajectoryName);
		// create empty trajectory
		data::Item::Ptr item = (*trajectoryPtr)->create();
		RenderBlock renderBlock(*this);
		{
			golem::CriticalSectionWrapper cswData(scene.getCS());
			const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(trajectoryName, item));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// done!
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("TT", [&] () {
		data::Trajectory* trajectory = is<data::Trajectory>(to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true)->second.get());
		Controller::State::Seq seq = WaypointCtrl::make(trajectory->getWaypoints());
		if (seq.empty()) throw Cancel("No waypoints");
		// select and remove
		size_t index = 1;
		Menu::selectIndex(seq, index, "waypoint");
		// frames
		const Mat34 source = forwardTransformArm(seq[index - 1]);
		const Mat34 target = forwardTransformArm(WaypointCtrl::lookup(*controller).state);
		// target = trn*source ==> trn = target*source^-1
		Mat34 trn;
		trn.setInverse(source);
		trn.multiply(target, trn);
		// transform trajectory
		Controller::State::Seq out;
		transformTrajectory(trn, seq.begin(), seq.end(), out);
		trajectory->setWaypoints(WaypointCtrl::make(out, out));
		// done!
		createRender();
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("TP", [&] () {
		data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true);
		// select collision object
		CollisionBounds::Ptr collisionBounds = selectCollisionBounds();
		// perform
		Controller::State::Seq seq;
		perform(dataCurrentPtr->first, item->first, is<data::Trajectory>(item->second.get()), seq, true, option(0, "Differential constraints: ", { "NO", "YES" }) == 1, option(1, "Autonomous mode: ", { "NO", "YES" }) == 1);
		// done!
		createRender();
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("TA", [&] () {
		data::Trajectory* trajectory = is<data::Trajectory>(to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true)->second.get());
		// add current state
		WaypointCtrl::Seq waypoints = trajectory->getWaypoints();
		waypoints.push_back(WaypointCtrl::lookup(*controller));
		trajectory->setWaypoints(waypoints);
		// done!
		createRender();
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("TR", [&] () {
		data::Trajectory* trajectory = is<data::Trajectory>(to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true)->second.get());
		WaypointCtrl::Seq waypoints = trajectory->getWaypoints();
		if (waypoints.empty()) throw Cancel("No waypoints");
		// select and remove
		size_t index = 1;
		Menu::selectIndex(waypoints, index, "waypoint");
		waypoints.erase(waypoints.begin() + index - 1);
		trajectory->setWaypoints(waypoints);
		// done!
		createRender();
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("TO", [&] () {
		data::Trajectory* trajectory = is<data::Trajectory>(to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true)->second.get());
		WaypointCtrl::Seq waypoints = trajectory->getWaypoints();
		if (waypoints.empty()) throw Cancel("No waypoints");
		// select and overwrite
		size_t index = 1;
		Menu::selectIndex(waypoints, index, "waypoint");
		waypoints[index - 1] = WaypointCtrl::lookup(*controller);
		trajectory->setWaypoints(waypoints);
		// done!
		createRender();
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("TV", [&] () {
		data::Trajectory* trajectory = is<data::Trajectory>(to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true)->second.get());
		WaypointCtrl::Seq waypoints = trajectory->getWaypoints();
		if (waypoints.empty()) throw Cancel("No waypoints");
		// reverse
		WaypointCtrl::Seq waypointsInv;
		for (WaypointCtrl::Seq::const_reverse_iterator i = waypoints.rbegin(); i != waypoints.rend(); ++i) waypointsInv.push_back(*i);
		trajectory->setWaypoints(waypointsInv);
		// done!
		createRender();
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("TW", [&] () {
		data::Trajectory* trajectory = is<data::Trajectory>(to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true)->second.get());
		WaypointCtrl::Seq waypoints = trajectory->getWaypoints();
		if (waypoints.empty()) throw Cancel("No waypoints");
		// select and remove
		size_t index = 1;
		Menu::selectIndex(waypoints, index, "waypoint");
		ConfigMat34 pose;
		pose.c.resize(info.getJoints().size());
		waypoints[index - 1].state.cpos.get(pose.c.begin(), pose.c.end());
		gotoPose(pose);
		// done!
		createRender();
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("TE", [&] () {
		StringIndexMap::value_type trajectoryIndexVal = std::make_pair(std::string(), trajectoryIndex);
		ConfigMat34::Range range = selectPoseRange(poseMap, &trajectoryIndexVal);
		trajectoryIndex = trajectoryIndexVal.second;
		// prepare a list of poses
		ConfigMat34::Seq seq;
		for (ConfigMat34::Map::const_iterator i = range.first; i != range.second; ++i) seq.push_back(i->second);
		if (seq.empty()) throw Cancel("No poses");
		// select and go
		StringIndexMap::const_iterator trajectoryIndexPtr = trajectoryIndexMap.find(trajectoryIndexVal.first);
		size_t index = trajectoryIndexPtr != trajectoryIndexMap.end() ? size_t(trajectoryIndexPtr->second) : 1;
		Menu::selectIndex(seq, index, "pose");
		gotoPose(seq[index - 1]);
		trajectoryIndexMap[trajectoryIndexVal.first] = static_cast<StringIndexMap::mapped_type>(index);
		// done!
		createRender();
		context.write("Done!\n");
	}));

	// debugging of the collision model
#ifdef _GOLEM_PLAYER_COLLISION_DEBUG_
	menuCmdMap.insert(std::make_pair("TL", [&]() {
		static Collision::Desc desc;
		static Collision::Waypoint waypoint;
		readNumber("offset: ", waypoint.depthOffset);
		readNumber("std_dev: ", waypoint.depthStdDev);
		readNumber("path: ", waypoint.pathDist);
		readNumber("weight: ", waypoint.weight);
		readNumber("points: ", waypoint.points);
		desc.waypoints.push_back(waypoint);
		// setup collisions
		const Manipulator::Ptr manipulator = Manipulator::Desc().create(*getPlanner().planner, getPlanner().controllerIDSeq);
		const Collision::Ptr collision = desc.create(*manipulator);
		if (option(0, "Single point: ", { "YES", "NO" }) == 0) {
			Collision::Bounds::Vec3Seq points;
			{
				const Vec3 point(0.5, -0.6, 0.0);
				points.push_back(Collision::Bounds::Vec3(point));
				golem::CriticalSectionWrapper csw(getCS());
				objectRenderer.reset();
				objectRenderer.addAxes3D(Mat34(Mat33::identity(), point), Vec3(0.05));
				objectRenderer.setPointSize(4.0);
				objectRenderer.addPoint(point, RGBA::BLACK);
			}
			option("\x0D", "Continue <Enter>...");
			collision->setPoints(points);
		}
		else {
			const data::Point3D* points = is<const data::Point3D>(to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true)->second.get());
			if (!points) throw Cancel("data::Point3D required");
			golem::Rand rand(context.getRandSeed());
			collision->create(rand, *points);
		}
		// compute collisions
		collision->evaluate(waypoint, manipulator->getConfig(WaypointCtrl::lookup(*controller).state), true);
		// done!
		context.write("Done!\n");
	}));
#endif // _GOLEM_PLAYER_COLLISION_DEBUG_
}

//------------------------------------------------------------------------------

void golem::Player::render() const {
	Recorder::render();
	objectRenderer.render();
}

void golem::Player::mouseHandler(int button, int state, int x, int y) {
	Manager::mouseHandler(button, state, x, y);

	for (ActiveCtrl::Map::const_iterator i = activectrlMap.begin(); i != activectrlMap.end(); ++i)
		if (i->second->isActive()) {
			golem::UIKeyboardMouse* uiKeyboardMouse = is<golem::UIKeyboardMouse>(i->second.get());
			if (uiKeyboardMouse)
				uiKeyboardMouse->mouseHandler(button, state, x, y);
		}
}

void golem::Player::motionHandler(int x, int y) {
	Manager::motionHandler(x, y);

	for (ActiveCtrl::Map::const_iterator i = activectrlMap.begin(); i != activectrlMap.end(); ++i)
		if (i->second->isActive()) {
			golem::UIKeyboardMouse* uiKeyboardMouse = is<golem::UIKeyboardMouse>(i->second.get());
			if (uiKeyboardMouse)
				uiKeyboardMouse->motionHandler(x, y);
		}
}

void golem::Player::keyboardHandler(int key, int x, int y) {
	Recorder::keyboardHandler(key, x, y);

	U32 keyIndex = 0;
	for (ActiveCtrl::Map::const_iterator i = activectrlMap.begin(); i != activectrlMap.end(); ++i, ++keyIndex) {
		// F1..12 + Alt
		if (key == ((GLUT_KEY_F1 + keyIndex) | UIKeyboardMouseCallback::KEY_SPECIAL | UIKeyboardMouseCallback::KEY_ALT)) {
			i->second->setActive(!i->second->isActive());
			context.notice("%s %s\n", i->second->getID().c_str(), i->second->isActive() ? "(Active)" : "(Inactive)");
		}
		if (i->second->isActive()) {
			golem::UIKeyboardMouse* uiKeyboardMouse = is<golem::UIKeyboardMouse>(i->second.get());
			if (uiKeyboardMouse)
				uiKeyboardMouse->keyboardHandler(key, x, y);
		}
	}
}

//------------------------------------------------------------------------------

const golem::Player::PlannerInfo& golem::Player::getPlanner() const {
	if (plannerIndex >= plannerInfoSeq.size())
		throw Message(Message::LEVEL_ERROR, "Player::getPlanner(): Invalid planner index %u", plannerIndex);
	return plannerInfoSeq[plannerIndex];
}

GenWorkspaceChainState golem::Player::forwardTransformChains(const golem::ConfigspaceCoord& cc) const {
	GenWorkspaceChainState gwcs;
	controller->chainForwardTransform(cc, gwcs.wpos);
	gwcs.wpos[getPlanner().armInfo.getChains().begin()].multiply(gwcs.wpos[getPlanner().armInfo.getChains().begin()], controller->getChains()[getPlanner().armInfo.getChains().begin()]->getReferencePose()); // 1:1
	return gwcs;
}

golem::Mat34 golem::Player::forwardTransformArm(const golem::ConfigspaceCoord& cc) const {
	return forwardTransformChains(cc).wpos[getPlanner().armInfo.getChains().begin()];
}

size_t golem::Player::findTrajectory(const golem::Controller::State::Seq& trajectory, golem::Controller::State::Seq& approachTrajectory, golem::Controller::State::Seq& manipulationTrajectory, bool mergeTrajectories) {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Player::findTrajectory(): At least two waypoints required");

	// differential constraints
	const golem::Controller::State target = mergeTrajectories ? getPlanner().planner->makeDiffConstraintBegin(trajectory) : trajectory.front();

	// planning
	findTrajectory(WaypointCtrl::lookup(*controller).command, &target, nullptr, SEC_TM_REAL_ZERO, approachTrajectory);

	manipulationTrajectory = trajectory;

	// differential constraints
	if (mergeTrajectories) {
		size_t approachTrajectorySize = approachTrajectory.size(), mergeWaypoint;

		// find stationary waypoint
		for (mergeWaypoint = 1; mergeWaypoint < trajectory.size(); ++mergeWaypoint) {
			bool zero = true;
			for (golem::Configspace::Index j = info.getJoints().begin(); zero && j < info.getJoints().end(); ++j)
				zero = Math::abs(trajectory[mergeWaypoint].cvel[j]) < REAL_EPS;
			if (zero) break;
		}
		// found
		if (mergeWaypoint < trajectory.size()) {
			// move waypoints
			const SecTmReal dt = manipulationTrajectory[mergeWaypoint].t;
			for (size_t i = 0; i <= mergeWaypoint; ++i) {
				approachTrajectory.push_back(*manipulationTrajectory.begin());
				manipulationTrajectory.erase(manipulationTrajectory.begin());
			}
			// profile
			approachTrajectory.back().t += dt;
			ManipDist dist(*getPlanner().planner->getProfile());
			getPlanner().planner->getProfile()->profile(approachTrajectory);
		}
		
		return approachTrajectorySize;
	}
	else
		return -1;
}

RBDist golem::Player::findTrajectory(const golem::Controller::State& begin, const golem::Controller::State* pcend, const golem::Mat34* pwend, golem::SecTmReal t, golem::Controller::State::Seq& trajectory) {
	if (!pcend && !pwend)
		throw Message(Message::LEVEL_ERROR, "Player::findTrajectory(): no target specified");

	// Trajectory from initial position to end position
	for (golem::U32 i = 0; i < getPlanner().trajectoryTrials; ++i) {
		if (universe.interrupted())
			throw Exit();
		context.debug("Player::findTrajectory(): Planning movement...\n");
		// All bounds are treated as obstacles
		uiPlanner->setCollisionBoundsGroup(Bounds::GROUP_ALL);
		// planner debug
		//context.verbose("%s\n", plannerDebug(*getPlanner().planner).c_str());

		RBDist err; // set to 0
		// Setup configspace target
		Controller::State cend = pcend ? *pcend : begin;
		// Workspace target
		if (pwend) {
			// Setup workspace target
			GenWorkspaceChainState wend;
			wend.setToDefault(info.getChains().begin(), info.getChains().end()); // all used chains
			wend.wpos[getPlanner().armInfo.getChains().begin()] = *pwend;
			// planner debug
			//context.verbose("%s\n", plannerWorkspaceDebug(*getPlanner().planner, &wend.wpos).c_str());
			if (!getPlanner().planner->findTarget(begin, wend, cend))
				continue;
			// update configspace coords of the hand
			if (pcend) cend.cpos.set(getPlanner().handInfo.getJoints(), pcend->cpos);
			// error
			WorkspaceChainCoord wcc;
			controller->chainForwardTransform(cend.cpos, wcc);
			wcc[getPlanner().armInfo.getChains().begin()].multiply(wcc[getPlanner().armInfo.getChains().begin()], controller->getChains()[getPlanner().armInfo.getChains().begin()]->getReferencePose());
			err.setSqrt(RBCoord(*pwend), RBCoord(wcc[getPlanner().armInfo.getChains().begin()]));
			context.debug("Player::findTrajectory(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
		}

		// planner debug
		//context.verbose("%s\n", plannerConfigspaceDebug(*getPlanner().planner, &cend.cpos).c_str());
		// Find collision-free trajectory and wait until the device is ready for new commands
		cend.t = begin.t + (t > SEC_TM_REAL_ZERO ? t : getPlanner().trajectoryDuration);
		if (getPlanner().planner->findGlobalTrajectory(begin, cend, trajectory, trajectory.begin()))
			return err;// success
	}

	throw Message(Message::LEVEL_ERROR, "Player::findTrajectory(): unable to find trajectory");
}

RBDist golem::Player::transformTrajectory(const golem::Mat34& trn, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory) {
	if (begin == end)
		throw Message(Message::LEVEL_ERROR, "Player::transformTrajectory(): Empty input trajectory");
	// arm chain and joints pointers
	const golem::Chainspace::Index armChain = getPlanner().armInfo.getChains().begin();
	const golem::Configspace::Range armJoints = getPlanner().armInfo.getJoints();
	// Compute a sequence of targets corresponding to the transformed arm end-effector
	GenWorkspaceChainState::Seq seq;
	for (Controller::State::Seq::const_iterator i = begin; i != end; ++i) {
		GenWorkspaceChainState gwcs;
		controller->chainForwardTransform(i->cpos, gwcs.wpos);
		gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1
		gwcs.wpos[armChain].multiply(trn, gwcs.wpos[armChain]); // new waypoint frame
		gwcs.t = i->t;
		seq.push_back(gwcs);
	}
	// planner debug
	//context.verbose("%s\n", plannerDebug(*planner).c_str());
	Controller::State::Seq ctrajectory;
	{
		// Find initial target position
		Controller::State cend = *begin;
		// planner debug
		//context.verbose("%s\n", plannerWorkspaceDebug(*getPlanner().planner, &seq[0].wpos).c_str());
		if (!getPlanner().planner->findTarget(*begin, seq[0], cend))
			throw Message(Message::LEVEL_ERROR, "Player::transformTrajectory(): Unable to find initial target configuration");
		// Find remaining position sequence
		if (seq.size() == 1)
			ctrajectory.push_back(cend);
		else if (seq.size() > 1 && !getPlanner().planner->findLocalTrajectory(cend, ++seq.begin(), seq.end(), ctrajectory, ctrajectory.end()))
			throw Message(Message::LEVEL_ERROR, "Player::transformTrajectory(): Unable to find trajectory");
	}
	// update arm configurations and compute average error
	RBDist err; // set to 0
	Controller::State::Seq::const_iterator j = begin;
	for (size_t i = 0; i < ctrajectory.size(); ++i, ++j) {
		// copy config
		trajectory.push_back(*j);
		trajectory.back().set(getPlanner().armInfo.getJoints().begin(), getPlanner().armInfo.getJoints().end(), ctrajectory[i]);
		// error
		WorkspaceChainCoord wcc;
		controller->chainForwardTransform(trajectory.back().cpos, wcc);
		wcc[armChain].multiply(wcc[armChain], controller->getChains()[armChain]->getReferencePose());
		err.add(err, RBDist(RBCoord(wcc[armChain]), RBCoord(seq[i].wpos[armChain])).getSqrt());
	}
	context.debug("Player::transformTrajectory(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
	return err;
}

void golem::Player::sendTrajectory(const golem::Controller::State::Seq& trajectory, bool clear) {
	if (trajectory.empty())
		throw Message(Message::LEVEL_ERROR, "Player::sendTrajectory(): empty trajectory!");
	context.debug("Player::sendTrajectory(): Moving...\n");
	golem::Controller::State::Seq trj = trajectory;
	// shift trajectory in time to avoid initial high hand joints accelerations (due to soft fingers)
	golem::SecTmReal dt = context.getTimer().elapsed() + getPlanner().trajectoryIdleBegin - trj.begin()->t;
	for (Controller::Trajectory::iterator i = trj.begin(); i != trj.end(); ++i) {
		i->t += dt;
//auto c = i->cpos.data(); context.debug("TARGET: t=%f, (%f, %f, %f, %f, %f, %f, %f), (%f, %f, %f, %f, %f, %f, %f)\n", i->t, c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[27], c[28], c[29], c[30], c[31], c[32], c[33]);
	}
	// Move the robot
	(void)controller->send(&trj.front(), &trj.back() + 1, clear);
}

//------------------------------------------------------------------------------

CollisionBounds::Ptr golem::Player::selectCollisionBounds(bool draw) {
	CollisionBounds::Ptr collisionBounds;

	// select collision object
	context.write("Select collision object...\n");
	processItems([&] (const Data::Selection::List& list) {
		if (!list.empty()) {
			const data::Point3D* point = is<const data::Point3D>(list.front().getPtr()->second.get());
			if (point) {
				// create collision bounds
				collisionBounds.reset(new CollisionBounds(
					*getPlanner().planner,
					CollisionBounds::getBounds([=](size_t i, Vec3& p) -> bool { if (i < point->getSize()) p = point->getPoint(i); return i < point->getSize(); }, is<const data::Cluster3D>(point)),
					draw ? &objectRenderer : nullptr,
					draw ? &scene.getCS() : nullptr
				));
				// draw points
				golem::CriticalSectionWrapper csw(scene.getCS());
				for (size_t i = 0; i < point->getSize(); ++i)
					objectRenderer.addPoint(point->getPoint(i), collisionPointColour);
			}
			else
				context.write("Object collisions unsupported\n");
		}
		else
			context.write("Object collisions disabled\n");
	}, nullptr, false, true);

	return collisionBounds;
}

void golem::Player::perform(const std::string& data, const std::string& item, data::Trajectory* trajectory, Controller::State::Seq& seq, bool testTrajectory, bool mergeTrajectories, bool performAuto) {
	if (!trajectory)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): Trajectory interface required");

	if (!performAuto)
		trajectory->setProfile(trajectoryProfileSemiAuto);

	trajectory->createTrajectory(seq);

	// find trajectory
	golem::Controller::State::Seq approachTrajectory, manipulationTrajectory;
	const size_t mergeWaypoint = findTrajectory(seq, approachTrajectory, manipulationTrajectory, mergeTrajectories);
	golem::Controller::State::Seq completeTrajectory = approachTrajectory;
	completeTrajectory.insert(completeTrajectory.end(), manipulationTrajectory.begin(), manipulationTrajectory.end());
	
	// create trajectory item
	data::Handler::Map::const_iterator handlerPtr = handlerMap.find(getPlanner().trajectoryHandler);
	if (handlerPtr == handlerMap.end())
		throw Message(Message::LEVEL_ERROR, "Player::perform(): unknown default trajectory handler %s", getPlanner().trajectoryHandler.c_str());
	data::Handler* handler = is<data::Handler>(handlerPtr);
	if (!handler)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): invalid default trajectory handler %s", getPlanner().trajectoryHandler.c_str());
	data::Item::Ptr itemTrajectory = handler->create();
	data::Trajectory* trajectoryIf = is<data::Trajectory>(itemTrajectory.get());
	if (!trajectoryIf)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): unable to create trajectory using handler %s", getPlanner().trajectoryHandler.c_str());
	trajectoryIf->setWaypoints(WaypointCtrl::make(completeTrajectory, completeTrajectory));

	// reset camera
	ScopeGuard resetCamera([&]() {
		this->resetCamera();
	});

	// block displaying the current item
	RenderBlock renderBlock(*this);

	// test trajectory
	if (performAuto && testTrajectory) {
		// insert trajectory to data with temporary name
		const std::string itemLabelTmp = item + dataDesc->sepName + makeString("%f", context.getTimer().elapsed());
		ScopeGuard removeItem([&] () {
			//UI::removeCallback(*this, getCurrentHandler());
			clearRenderers();
			{
				golem::CriticalSectionWrapper csw(scene.getCS());
				to<Data>(dataCurrentPtr)->itemMap.erase(itemLabelTmp);
			}
			createRender();
		});
		{
			golem::CriticalSectionWrapper csw(scene.getCS());
			const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(itemLabelTmp, itemTrajectory));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// enable GUI interaction and refresh
		UI::addCallback(*this, getCurrentHandler());
		createRender();
		// prompt user
		EnableKeyboardMouse enableKeyboardMouse(*this);
		option("\x0D", "Press <Enter> to accept trajectory...");
	}

	context.write("Performance start\n");

	// autonomous run
	if (performAuto)
		this->performAuto(approachTrajectory, manipulationTrajectory, mergeWaypoint, data, item);
	else {
		this->performSemiAuto(approachTrajectory, manipulationTrajectory, trajectory->getManifold(), mergeWaypoint, data, item);
		// copy manifold
		trajectoryIf->setManifold(trajectory->getManifold());
	}

	// insert trajectory
	{
		golem::CriticalSectionWrapper csw(scene.getCS());
		data::Data::Map::iterator dataPtr = dataMap.find(data);
		if (dataPtr == dataMap.end())
			throw Message(Message::LEVEL_ERROR, "Player::perform(): unable to find Data %s", data.c_str());
		dataPtr->second->itemMap.insert(std::make_pair(recorderItem + makeString("%s%.3f", dataDesc->sepName.c_str(), recorderStart), itemTrajectory));
	}

	context.write("Performance finished\n");
}

void golem::Player::performAuto(golem::Controller::State::Seq& approachTrajectory, golem::Controller::State::Seq& manipulationTrajectory, size_t mergeWaypoint, const std::string& data, const std::string& item) {
	auto wait = [&] (U32& step, SecTmReal timeWait) {
		controller->waitForBegin();
		const SecTmReal t = context.getTimer().elapsed();

		// repeat every send waypoint until trajectory end
		for (; controller->waitForBegin(); ++step) {
			if (universe.interrupted()) {
				controller->stop();
				throw Exit();
			}
			if (universe.waitKey(0) == 27) {
				controller->stop();
				throw Cancel("Performance interrupted");
			}
			if (timeWait > SEC_TM_REAL_ZERO && t + timeWait < context.getTimer().elapsed()) {
				break;
			}
			if (controller->waitForEnd(0)) {
				break;
			}

			// print every 10th robot state
			if (step%10 == 0)
				context.write("State #%d\r", step);
		}
	};
	
	// execute approach trajectory (go to initial state)
	sendTrajectory(approachTrajectory);

	U32 step = 0;
	wait(step, mergeWaypoint < approachTrajectory.size() ? (approachTrajectory.begin() + mergeWaypoint)->t - approachTrajectory.begin()->t : SEC_TM_REAL_ZERO);

	// stop recording
	ScopeGuard recordingGuard([&]() {
		recordingStop(getPlanner().trajectoryIdlePerf);
		recordingWaitToStop();
		context.write("Recording stop\n");
	});

	// start recording
	recordingStart(data, item, true);
	recordingWaitToStart();
	context.write("Recording start\n");

	// repeat every send waypoint until trajectory end
	wait(step, SEC_TM_REAL_ZERO);

	// execute manipulation trajectory
	sendTrajectory(manipulationTrajectory);

	// repeat every send waypoint until trajectory end
	wait(step, SEC_TM_REAL_ZERO);
}

void golem::Player::performSemiAuto(golem::Controller::State::Seq& approachTrajectory, golem::Controller::State::Seq& manipulationTrajectory, const ManifoldCtrl& manifold, size_t mergeWaypoint, const std::string& data, const std::string& item) {
	const ActiveCtrl::Map::const_iterator activectrlPtr = getWorkspaceCtrlPtr();
	IActiveCtrl* activectrlIf = getWorkspaceCtrlIf(*activectrlPtr->second);

	const Controller::State::Info info = activectrlPtr->second->getInfo();
	const Mat34 referencePose = controller->getChains()[info.getChains().begin()]->getReferencePose();

	IActiveCtrl::Control control = [&]() {
		// run task
		for (;;) {
			const int key = universe.waitKey(1);
			if (universe.interrupted()) {
				controller->stop();
				throw Exit();
			}
			if (key == 27) {
				controller->stop();
				throw Cancel("Movement interrupted");
			}
			if (key == 13) {
				break;
			}
		}
	};

	// recorder data
	recorderItem = item;
	recorderData = data;
	recorderStart = context.getTimer().elapsed();

	// profile
	struct ProfileCallback : Profile::CallbackDist {
		const Controller::State::Info info;
		ProfileCallback(const Controller::State::Info& info) : info(info) {}
		Real distConfigspaceCoord(const ConfigspaceCoord& prev, const ConfigspaceCoord& next) const {
			Real dist = REAL_ZERO;
			for (Configspace::Index i = info.getJoints().begin(); i < info.getJoints().end(); ++i)
				dist += Math::sqr(prev[i] - next[i]);
			return Math::sqrt(dist);
		}
		Real distCoord(Real prev, Real next) const {
			return Math::abs(prev - next);
		}
		bool distCoordPlanning(const Configspace::Index& index) const {
			return info.getJoints().contains(index);
		}

		bool distCoordInterpolation(const Configspace::Index& index) const {
			return info.getJoints().contains(index);
		}
	} profileCallback(info);

	// mandatory
	if (approachTrajectory.size() > 1) {
		profileApproach->setCallbackDist(profileCallback);
		profileApproach->profile(approachTrajectory);
		activectrlIf->trajectoryCtrl(approachTrajectory, SEC_TM_REAL_ONE, Mat34::identity(), manifold.frame, manifold.frameDev, true, control, nullptr, [&](SecTmReal t) -> SecTmReal { return Math::sqr(t); });
	}

	// optional
	if (manipulationTrajectory.size() > 1) {
		profileManipulation->setCallbackDist(profileCallback);
		profileManipulation->profile(approachTrajectory);
		activectrlIf->trajectoryCtrl(manipulationTrajectory, SEC_TM_REAL_ZERO, activectrlIf->trajectoryCtrlTargetFrame(), manifold.frame, manifold.frameDev, false, control, nullptr, [&](SecTmReal t) -> SecTmReal { return Math::sqr(SEC_TM_REAL_ONE - t); });
	}
}

//------------------------------------------------------------------------------

ActiveCtrl::Map::const_iterator golem::Player::getWorkspaceCtrlPtr() const {
	ActiveCtrl::Map::const_iterator activectrlPtr = activectrlMap.find(getPlanner().workspacectrlHandler);
	if (activectrlPtr == activectrlMap.end() || activectrlPtr->second == nullptr)
		throw Message(Message::LEVEL_ERROR, "Player::getWorkspaceCtrlPtr(): unable to find active controller %s", getPlanner().workspacectrlHandler.c_str());
	return activectrlPtr;
}

IActiveCtrl* golem::Player::getWorkspaceCtrlIf(ActiveCtrl& activeCtrl) const {
	IActiveCtrl* activectrlIf = const_cast<IActiveCtrl*>(is<IActiveCtrl>(&activeCtrl));
	if (!activectrlIf)
		throw Message(Message::LEVEL_ERROR, "Player::getWorkspaceCtrlIf(): unable to cast to IActiveCtrl from active controller %s", getPlanner().workspacectrlHandler.c_str());
	return activectrlIf;
}

//------------------------------------------------------------------------------

void golem::Player::gotoConfig(const golem::Controller::State& state) {
	golem::Controller::State begin = WaypointCtrl::lookup(*controller).command;
	// find trajectory
	golem::Controller::State::Seq trajectory;
	findTrajectory(begin, &state, nullptr, getPlanner().trajectoryDuration, trajectory);
	sendTrajectory(trajectory);
	// wait for end
	controller->waitForEnd();
	// sleep
	Sleep::msleep(SecToMSec(getPlanner().trajectoryIdleEnd));
}

void golem::Player::gotoPose(const ConfigMat34& pose) {
	// set planner
	if (pose.plannerIndex >= plannerInfoSeq.size())
		throw Message(Message::LEVEL_ERROR, "Player::gotoPose(): Invalid planner index %u", plannerIndex);
	const U32 currentPlannerIndex = plannerIndex;
	plannerIndex = pose.plannerIndex;
	ScopeGuard restorePlannerIndex([&]() { plannerIndex = currentPlannerIndex; });
	// current state
	golem::Controller::State begin = WaypointCtrl::lookup(*controller).command;
//auto c = begin.cpos.data(); context.debug("BEGIN: t=%f, (%f, %f, %f, %f, %f, %f, %f), (%f, %f, %f, %f, %f, %f, %f)\n", begin.t, c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[27], c[28], c[29], c[30], c[31], c[32], c[33]);
	// target
	golem::Controller::State end = begin;
	end.cpos.set(pose.c.data(), pose.c.data() + std::min(pose.c.size(), (size_t)info.getJoints().size()));
//c = end.cpos.data(); context.debug("END: t=%f, (%f, %f, %f, %f, %f, %f, %f), (%f, %f, %f, %f, %f, %f, %f)\n", begin.t, c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[27], c[28], c[29], c[30], c[31], c[32], c[33]);
	// find trajectory
	golem::Controller::State::Seq trajectory;
	findTrajectory(begin, &end, nullptr, getPlanner().trajectoryDuration, trajectory);
	// send trajectory
	sendTrajectory(trajectory);
	// wait for end
	//controller->waitForEnd();
	for (; !controller->waitForEnd(1);) {
		const int key = universe.waitKey(0);
		if (universe.interrupted()) {
			controller->stop();
			throw Exit();
		}
		if (key == 27) {
			controller->stop();
			throw Cancel("Movement interrupted");
		}
	}
	// sleep
	Sleep::msleep(SecToMSec(getPlanner().trajectoryIdleEnd));
}

void golem::Player::getPose(golem::U32 joint, ConfigMat34& pose) const {
	// current state
	golem::Controller::State state = WaypointCtrl::lookup(*controller).state;
	// forward transform
	golem::WorkspaceJointCoord wjc;
	controller->jointForwardTransform(state.cpos, wjc);
	// return value
	pose.c.assign(state.cpos.data() + *info.getJoints().begin(), state.cpos.data() + *info.getJoints().end());
	pose.w = wjc[info.getJoints().begin() + joint];
}

//------------------------------------------------------------------------------

void golem::Player::main(bool runMenu) {
	if (serverClients > 0) {
		try {
			// Create timer
			SMTimer timer(&context.getTimer());
			// Create shared memory message stream
			SMMessageStream msgStr(*context.getMessageStream());

			// Create server
			SMHandler handler(*controller);
			SMServer server(serverPort, timer, &msgStr, &handler, serverClients);

			// Handle i/o
			Controller::State state = controller->createState();
			SecTmReal begin = context.getTimer().elapsed();
			for (U32 i = 0; !universe.interrupted(); ++i) {
				(void)controller->waitForBegin();
				const SecTmReal end = context.getTimer().elapsed();

				// send state
				controller->lookupState(end, state);
				server.write(sizeof(Controller::State), &state);

				// control message
				if (end > begin + serverMessageInterval) {
					context.verbose("packets = %u, packetsize = %u, cycle duration = %f\n", i, sizeof(Controller::State), controller->getCycleDuration());
					begin = end;
				}
			}
		}
		catch (const std::exception& ex) {
			context.write("%s\n", ex.what());
		}
	}
	else
		Recorder::main(runMenu);
}

//------------------------------------------------------------------------------
