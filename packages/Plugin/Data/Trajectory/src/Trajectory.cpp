/** @file Trajectory.cpp
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Planner/Data.h>
#include <Golem/Math/Data.h>
#include <Golem/App/Data.h>
#include <Golem/Tools/Import.h>
#include <Golem/Plugin/Ctrl.h>
#include <Golem/Tools/Menu.h>
#include <Golem/Data/Trajectory/Trajectory.h>
#include <boost/algorithm/string.hpp>
#include <GL/glut.h>

using namespace golem;
using namespace golem::data;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new HandlerTrajectory::Desc();
}

//------------------------------------------------------------------------------

void golem::data::HandlerTrajectory::ImportState::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("type", type, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("inp", inpStr, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("out", outStr, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("offset", offset, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("scale", scale, const_cast<golem::XMLContext*>(xmlcontext), false);
}

void golem::data::HandlerTrajectory::ImportState::update(const golem::RealSeq& data, golem::Controller::State& state) const {
	// ignore if input pointer is out of range
	if (data.size() <= inp)
		return;
	// transform input
	const Real val = offset + scale*data[inp];
	// update
	if (type == TYPE_TIME)
		state.t = val; // assign
	else if (type == TYPE_POSITION)
		state.cpos.data()[out] += val; // add
	else if (type == TYPE_VELOCITY)
		state.cvel.data()[out] += val; // add
	else if (type == TYPE_ACCELERATION)
		state.cacc.data()[out] += val; // add
	else
		state.get<Real>(out) += val; // add
}

void golem::data::HandlerTrajectory::ImportState::update(const Map& map, const golem::RealSeq& data, golem::Controller::State& state) {
	for (HandlerTrajectory::ImportState::Map::const_iterator i = map.begin(); i != map.end(); ++i)
		i->update(data, state);
}

void golem::data::HandlerTrajectory::ImportState::extract(const std::string& str, U32Seq& seq) {
	// pointers
	std::stringstream spointers(str + " "); // HACK std::stringstream::read(&c, 1) reads one character too much without reporting eof
	IStream ipointers(spointers, " -,");
	U32Seq pointers;
	try {
		while (!ipointers.eos())
			pointers.push_back(ipointers.nextInt<U32>());
	}
	catch (...) {
		// no numbers
		seq.push_back(golem::U32(-1));
		return;
	}
		// operations
	std::stringstream soperations(str + " "); // HACK std::stringstream::read(&c, 1) reads one character too much without reporting eof
	IStream ioperations(soperations, " 0123456789");
	BoolSeq operations;
	try {
		while (!ioperations.eos())
			operations.push_back(std::strcmp(ioperations.next<const char*>(), "-") == 0); // true for a range of indices, false for a single index
	}
	catch (...) {
		// no operations
	}
	// test length for validity
	if (operations.size() + 1 != pointers.size())
		throw Message(Message::LEVEL_CRIT, "HandlerTrajectory::ImportState::extract(): invalid pointer range formatting");
	// extract
	if (operations.empty()) {
		seq.push_back(pointers.front());
		return;
	}
	U32Seq::const_iterator j = pointers.begin(), k = ++pointers.begin();
	for (BoolSeq::const_iterator i = operations.begin(); i != operations.end(); ++i, ++j, ++k) {
		// range of indices
		if (*i) {
			for (U32 index = *j; index <= *k; ++index)
				seq.push_back(index);
			continue;
		}
		// a single index
		// at the beginning or if there was no range before
		if (i == operations.begin() || !*(i - 1))
			seq.push_back(*j);
		// at the end
		if (i + 1 == operations.end())
			seq.push_back(*k);
	}
}

void golem::data::HandlerTrajectory::ImportState::extract(Map& map) const {
	// input pointer
	U32Seq inpPtr;
	extract(inpStr, inpPtr);
	// output pointer
	U32Seq outPtr;
	extract(outStr, outPtr);
	// create all pairs
	map.reserve(map.size() + inpPtr.size()*outPtr.size());
	for (U32Seq::const_iterator ip = inpPtr.begin(); ip != inpPtr.end(); ++ip)
		for (U32Seq::const_iterator op = outPtr.begin(); op != outPtr.end(); ++op)
			map.push_back(ImportState(type, *ip < golem::U32(-1) ? *ip : *op, *op < golem::U32(-1) ? *op : *ip, offset, scale));
}

void golem::data::HandlerTrajectory::ImportState::extract(const Map& inp, Map& out) {
	Map buf;
	for (Map::const_iterator i = inp.begin(); i != inp.end(); ++i) {
		buf.clear();
		i->extract(buf);
		out.insert(out.end(), buf.begin(), buf.end());
	}
	//for (auto &i : out)
	//	printf("%i: %i->%i, off=%f, s=%f\n", (int)i.type, i.inp, i.out, i.offset, i.scale);
}

void golem::data::HandlerTrajectory::ImportFrame::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("type", type, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("lin", lin, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("ang", ang, const_cast<golem::XMLContext*>(xmlcontext), false);
}

void golem::data::HandlerTrajectory::ImportFrame::update(const golem::RealSeq& data, golem::Mat34& trn) const {
	if (size_t(lin + 3) >= data.size())
		throw Message(Message::LEVEL_CRIT, "HandlerTrajectory::ImportFrame::update(): linear pointer %u not in range <0, %zu)", lin + 3, data.size());
	const size_t size = type == TYPE_QUAT ? 4 : 3;
	if (size_t(ang + size) >= data.size())
		throw Message(Message::LEVEL_CRIT, "HandlerTrajectory::ImportFrame::update(): angular pointer %u not in range <0, %zu)", ang + U32(size), data.size());
	// linear component
	trn.p.setColumn3(data.data() + lin);
	// angular component
	if (type == TYPE_QUAT) {
		Quat quat;
		quat.x = data[ang + 0];
		quat.y = data[ang + 1];
		quat.z = data[ang + 2];
		quat.w = data[ang + 3]; // w last
		//quat.w = data[ang + 0]; // w first
		//quat.x = data[ang + 1];
		//quat.y = data[ang + 2];
		//quat.z = data[ang + 3];
		trn.R.fromQuat(quat);
	}
	else if (type == TYPE_EULER)
		trn.R.fromEuler(data[ang + 0], data[ang + 1], data[ang + 2]);
	else {
		Vec3 axis(data[ang + 0], data[ang + 1], data[ang + 2]);
		const Real angle = axis.magnitude();
		if (angle < REAL_EPS)
			trn.R.setId();
		else {
			axis.normalise();
			trn.R.fromAngleAxis(angle, axis);
		}
	}
}

//------------------------------------------------------------------------------

golem::data::ItemTrajectory::ItemTrajectory(HandlerTrajectory& handler) : Item(handler), handler(handler), waypointFile(handler.file), pathPosition(golem::REAL_ZERO), pathWaypoint(0), pathInterpol(golem::REAL_ZERO), contactPathWaypoint(-1), contactPathInterpol(golem::REAL_ONE) {
}

Item::Ptr golem::data::ItemTrajectory::clone() const {
	Item::Ptr item(handler.create());
	ItemTrajectory* itemTrajectory = to<ItemTrajectory>(item.get());
	itemTrajectory->waypoints = waypoints;
	itemTrajectory->waypointFile.setModified(true);
	itemTrajectory->pathPosition = pathPosition;
	itemTrajectory->pathWaypoint = pathWaypoint;
	itemTrajectory->pathInterpol = pathInterpol;
	itemTrajectory->contactPathWaypoint = contactPathWaypoint;
	itemTrajectory->contactPathInterpol = contactPathInterpol;
	// done!
	return item;
}

void golem::data::ItemTrajectory::createRender() {
	handler.createRender(*this);
}

void golem::data::ItemTrajectory::load(const std::string& prefix, const golem::XMLContext* xmlcontext) {
	// waypoint
	std::string waypointSuffix;
	golem::XMLData("waypoint", waypointSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (waypointSuffix.length() > 0) {
		waypointFile.load(prefix + waypointSuffix, [&] (const std::string& path) {
			if (!handler.controller)
				throw Message(Message::LEVEL_ERROR, "ItemTrajectory::load(): controller is not set");

			waypoints.clear();
			try {
				FileReadStream frs(path.c_str());
				frs.setHandler(&handler);
				frs.read(waypoints, waypoints.end(), golem::WaypointCtrl::create(*handler.controller));
			}
			catch (const golem::Message& msg) {
				// try legacy format
				golem::Controller::State::Seq trajectory;
				try {
					FileReadStream frs(path.c_str());
					frs.setHandler(&handler);
					frs.read(trajectory, trajectory.end(), handler.controller->createState());
				}
				catch (...) {
					throw msg;
				}
				// commands = states
				waypoints = golem::WaypointCtrl::make(trajectory, trajectory);
			}
		});
	}

	// path settings
	golem::XMLData("path_position", pathPosition, const_cast<golem::XMLContext*>(xmlcontext), false);
	try {
		golem::XMLData("path_waypoint", contactPathWaypoint, xmlcontext->getContextFirst("contact", false), false);
		golem::XMLData("path_interpol", contactPathInterpol, xmlcontext->getContextFirst("contact", false), false);
	}
	catch (const golem::MsgXMLParser&) {
	}

	// manifold
	try {
		golem::XMLData(manifoldCtrl, xmlcontext->getContextFirst("manifold", false), false);
	}
	catch (const golem::MsgXMLParser&) {
	}
}

void golem::data::ItemTrajectory::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	// waypoint xml
	std::string waypointSuffix = !waypoints.empty() ? handler.waypointSuffix : "";
	golem::XMLData("waypoint", waypointSuffix, xmlcontext, true);

	// waypoint binary
	if (waypointSuffix.length() > 0) {
		waypointFile.save(prefix + waypointSuffix, [=] (const std::string& path) {
			FileWriteStream(path.c_str()).write(waypoints.begin(), waypoints.end());
		});
	}
	else
		waypointFile.remove();

	// path settings
	golem::XMLData("path_position", const_cast<Real&>(pathPosition), xmlcontext, true);
	golem::XMLData("path_waypoint", const_cast<size_t&>(contactPathWaypoint), xmlcontext->getContextFirst("contact", true), true);
	golem::XMLData("path_interpol", const_cast<Real&>(contactPathInterpol), xmlcontext->getContextFirst("contact", true), true);

	// manifold
	golem::XMLData(const_cast<ManifoldCtrl&>(manifoldCtrl), xmlcontext->getContextFirst("manifold", true), true);
}

void golem::data::ItemTrajectory::setWaypoints(const golem::WaypointCtrl::Seq& waypoints) {
	this->waypoints = waypoints;
	waypointFile.setModified(true);
}

const golem::WaypointCtrl::Seq& golem::data::ItemTrajectory::getWaypoints() const {
	return waypoints;
}

void golem::data::ItemTrajectory::setManifold(const ManifoldCtrl& manifold) {
	this->manifoldCtrl = manifold;
}

const ManifoldCtrl& golem::data::ItemTrajectory::getManifold() const {
	return manifoldCtrl;
}

StringSeq golem::data::ItemTrajectory::getProfiles() const {
	return handler.getProfiles();
}

void golem::data::ItemTrajectory::setProfile(const std::string& profile) {
	handler.setProfile(profile);
}

void golem::data::ItemTrajectory::createTrajectory(golem::Controller::State::Seq& trajectory) {
	handler.createTrajectory(*this, trajectory);
}

//------------------------------------------------------------------------------

void golem::data::HandlerTrajectory::FactorDesc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("arm", arm, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("hand", hand, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("other", other, const_cast<golem::XMLContext*>(xmlcontext), false);
}

void golem::data::HandlerTrajectory::ImportRobotTrjDesc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("interval", interval, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("begin", begin, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("end", end, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("subsampling", subsampling, const_cast<golem::XMLContext*>(xmlcontext), false);
	stateMap.clear();
	golem::XMLData(stateMap, stateMap.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "state");
	commandMap.clear();
	golem::XMLData(commandMap, commandMap.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "command");
}

//------------------------------------------------------------------------------

std::string golem::data::HandlerTrajectory::ProfileDesc::getDefaultName() {
	return std::string("default");
}

void golem::data::HandlerTrajectory::ProfileDesc::load(const golem::XMLContext * xmlcontext) {
	golem::XMLContext* pxmlcontext = const_cast<golem::XMLContext*>(xmlcontext);

	golem::XMLData(*profileDesc, pxmlcontext, false);

	golem::XMLDataSeq(distance, "c", pxmlcontext->getContextFirst("distance"), false, golem::REAL_ZERO);
	golem::XMLDataSeq(extrapolation, "c", pxmlcontext->getContextFirst("extrapolation"), false, golem::REAL_ZERO);
	golem::XMLDataSeq(command, "c", pxmlcontext->getContextFirst("command"), false, golem::REAL_ZERO);

	velFac.load(pxmlcontext->getContextFirst("velocity"));
	accFac.load(pxmlcontext->getContextFirst("acceleration"));
	disFac.load(pxmlcontext->getContextFirst("distance"));
	extFac.load(pxmlcontext->getContextFirst("extrapolation"));
	cmdFac.load(pxmlcontext->getContextFirst("command"));

	golem::XMLData("extrapolation", trjExtrapolation, pxmlcontext, false);
	golem::XMLData("extrapolation_index", trjExtrapolationIndex, pxmlcontext, false);
	golem::XMLData("duration", trjDuration, pxmlcontext, false);
	golem::XMLData("idle", trjIdle, pxmlcontext, false);

	action.clear();
	try {
		golem::XMLData(action, action.max_size(), pxmlcontext, "action");
	}
	catch (const golem::MsgXMLParser&) {}
}

void golem::XMLData(data::HandlerTrajectory::ProfileDesc::Map::value_type &val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("name", const_cast<std::string&>(val.first), xmlcontext, create);
	val.second.load(xmlcontext);
}

//------------------------------------------------------------------------------

void golem::data::HandlerTrajectory::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	golem::data::Handler::Desc::load(context, xmlcontext);

	golem::XMLData("planner_index", plannerIndex, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLContext* pxmlcontext = xmlcontext->getContextFirst("trajectory", false);

	golem::XMLData("waypoint_suffix", waypointSuffix, pxmlcontext, false);

	try {
		profileDescMap.clear();
		XMLData(profileDescMap, profileDescMap.max_size(), pxmlcontext, "profile");
	}
	catch (const golem::MsgXMLParserNameNotFound&) {}

	golem::XMLData(boundsSolidColour, pxmlcontext->getContextFirst("appearance bounds solid_colour"), false);
	golem::XMLData(pathRenderer, pxmlcontext->getContextFirst("appearance path"), false);
	golem::XMLData("inc", pathInc, pxmlcontext->getContextFirst("appearance path"), false);

	importRobotTrj.load(pxmlcontext->getContextFirst("import robot_trj"));
	golem::XMLData("file_ext", importHDF5FileExt, pxmlcontext->getContextFirst("import hdf5", false), false);
	golem::XMLData("robot_trj", importHDF5RobotTrj, pxmlcontext->getContextFirst("import hdf5", false), false);
}

golem::data::Handler::Ptr golem::data::HandlerTrajectory::Desc::create(golem::Context &context) const {
	data::Handler::Ptr handler(new HandlerTrajectory(context));
	to<HandlerTrajectory>(handler.get())->create(*this);
	return handler;
}

//------------------------------------------------------------------------------

golem::data::HandlerTrajectory::HandlerTrajectory(golem::Context &context) : data::Handler(context), planner(nullptr), controller(nullptr), pProfileDesc(nullptr), pathIncStep(0), pathPositionInc(golem::REAL_ZERO), boundsShow(false), contactPathRequest(false) {
}

void golem::data::HandlerTrajectory::create(const Desc& desc) {
	golem::data::Handler::create(desc);

	plannerIndex = desc.plannerIndex;

	waypointSuffix = desc.waypointSuffix;

	profileDescMap.clear();
	for (auto& k : desc.profileDescMap) {
		ProfileDesc profileDesc;
		
		profileDesc.profileDesc = k.second.profileDesc; // shallow copy
		profileDesc.profileDesc->pCallbackDist = this;

		profileDesc.velFac = k.second.velFac;
		k.second.profileDesc->velocity.resize(Configspace::DIM, golem::REAL_ONE);
		profileDesc.velFac.coord.set(k.second.profileDesc->velocity.begin(), k.second.profileDesc->velocity.end());

		profileDesc.accFac = k.second.accFac;
		k.second.profileDesc->acceleration.resize(Configspace::DIM, golem::REAL_ONE);
		profileDesc.accFac.coord.set(k.second.profileDesc->acceleration.begin(), k.second.profileDesc->acceleration.end());


		profileDesc.disFac = k.second.disFac;
		profileDesc.disFac.coord.fill(golem::REAL_ONE);
		profileDesc.disFac.coord.set(k.second.distance.data(), k.second.distance.data() + std::min(k.second.distance.size(), (size_t)Configspace::DIM));

		profileDesc.extFac = k.second.extFac;
		profileDesc.extFac.coord.fill(golem::REAL_ZERO);
		profileDesc.extFac.coord.set(k.second.extrapolation.data(), k.second.extrapolation.data() + std::min(k.second.extrapolation.size(), (size_t)Configspace::DIM));

		profileDesc.cmdFac = k.second.cmdFac;
		profileDesc.cmdFac.coord.fill(golem::REAL_ZERO);
		profileDesc.cmdFac.coord.set(k.second.command.data(), k.second.command.data() + std::min(k.second.command.size(), (size_t)Configspace::DIM));
		
		profileDesc.trjExtrapolation = k.second.trjExtrapolation;
		profileDesc.trjExtrapolationIndex = k.second.trjExtrapolationIndex;
		profileDesc.trjDuration = k.second.trjDuration;
		profileDesc.trjIdle = k.second.trjIdle;

		profileDesc.action = k.second.action;

		
		profileDescMap.insert(std::make_pair(k.first, profileDesc));
	}

	// select default profile
	pProfileDesc = &*profileDescMap.begin();
	pProfile.reset();

	distance = pProfileDesc->second.disFac.coord; // default
	extrapolation = pProfileDesc->second.extFac.coord; // default
	command = pProfileDesc->second.cmdFac.coord; // default


	boundsSolidColour = desc.boundsSolidColour;
	pathRenderer = desc.pathRenderer;
	pathInc = desc.pathInc;

	showCommands = desc.showCommands;

	importRobotTrj = desc.importRobotTrj;
	importRobotTrj.stateMap.clear();
	importRobotTrj.commandMap.clear();
	ImportState::extract(desc.importRobotTrj.stateMap, importRobotTrj.stateMap);
	ImportState::extract(desc.importRobotTrj.commandMap, importRobotTrj.commandMap);
	importRobotTrj.assertValid(Assert::Context("HandlerTrajectory::create(): importRobotTrj."));
	
	importHDF5FileExt = desc.importHDF5FileExt;
	importHDF5RobotTrj = desc.importHDF5RobotTrj;

	importTypes = {
		importHDF5FileExt,
	};
}

//------------------------------------------------------------------------------

std::string golem::data::HandlerTrajectory::getFileExtWaypoint() {
	return std::string(".wp");
}

golem::data::Item::Ptr golem::data::HandlerTrajectory::create() const {
	return Item::Ptr(new ItemTrajectory(*const_cast<HandlerTrajectory*>(this))); // safe: no modification here - only passing reference
}

//------------------------------------------------------------------------------

golem::U32 golem::data::HandlerTrajectory::getPlannerIndex() const {
	return plannerIndex;
}

void golem::data::HandlerTrajectory::set(const golem::Planner& planner, const ControllerId::Seq& controllerIDSeq) {
	this->planner = &planner;
	this->controller = &planner.getController();

	if (controllerIDSeq.size() < 2)
		throw Message(Message::LEVEL_CRIT, "HandlerTrajectory::set(): arm and hand are required");

	// joint and chain info
	info = controller->getStateInfo();
	armInfo = controllerIDSeq[0].findInfo(*const_cast<golem::Controller*>(controller));
	handInfo = controllerIDSeq[1].findInfo(*const_cast<golem::Controller*>(controller));
	defaultState.reset(new Controller::State(controller->createState()));
	controller->setToDefault(*defaultState);

	// set profile
	setProfile(ProfileDesc::getDefaultName());

	boundsSeq.clear();
	// chain bounds
	for (Chainspace::Index i = info.getChains().begin(); i != info.getChains().end(); ++i) {
		const Chain* chain = controller->getChains()[i];
		chainBoundsSeq[i].clear();
		chainMat34Seq[i].clear();
		const Bounds::Desc::SeqPtr seq = chain->getBoundsDescSeq();
		for (Bounds::Desc::Seq::const_iterator j = seq->begin(); j != seq->end(); ++j)
			if (*j != nullptr) {
				chainBoundsSeq[i].push_back(j->get()->create());
				chainMat34Seq[i].push_back(chainBoundsSeq[i].back()->getPose());
				boundsSeq.push_back(chainBoundsSeq[i].back().get());
			}
	}
	// joint bounds
	for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i) {
		const Joint* joint = controller->getJoints()[i];
		jointBoundsSeq[i].clear();
		jointMat34Seq[i].clear();
		const Bounds::Desc::SeqPtr seq = joint->getBoundsDescSeq();
		for (Bounds::Desc::Seq::const_iterator j = seq->begin(); j != seq->end(); ++j)
			if (*j != nullptr) {
				jointBoundsSeq[i].push_back(j->get()->create());
				jointMat34Seq[i].push_back(jointBoundsSeq[i].back()->getPose());
				boundsSeq.push_back(jointBoundsSeq[i].back().get());
			}
	}
}

void golem::data::HandlerTrajectory::create(U32 index, const golem::ConfigspaceCoord& delta, golem::Controller::State::Seq& trajectory) const {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::create(): At least two waypoints required");

	index = std::min(index, (U32)trajectory.size() - 2);
	while (index-- > 0)
		trajectory.pop_back();

	Controller::State state = trajectory.back();
	// last two waypoint used to extrapolate coordinates
	const Controller::State* s[2] = { &trajectory[trajectory.size() - 1], &trajectory[trajectory.size() - 2] };
	// linear extrapolation
	for (Configspace::Index i = state.getInfo().getJoints().begin(); i != state.getInfo().getJoints().end(); ++i)
		state.cpos[i] = Math::clamp(s[0]->cpos[i] + delta[i] * (s[0]->cpos[i] - s[1]->cpos[i]), controller->getMin().cpos[i], controller->getMax().cpos[i]);

	trajectory.push_back(state);
}

void golem::data::HandlerTrajectory::profile(golem::SecTmReal duration, golem::Controller::State::Seq& trajectory) const {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::profile(): At least two waypoints required");

	trajectory.front().t = golem::SEC_TM_REAL_ZERO;
	trajectory.back().t = duration;
	pProfile->profile(trajectory);
}

StringSeq golem::data::HandlerTrajectory::getProfiles() const {
	StringSeq profiles;

	for (auto& k : profileDescMap)
		profiles.push_back(k.first);

	return profiles;
}

void golem::data::HandlerTrajectory::setProfile(const std::string& profile) {
	pProfile.reset();

	// select profile
	ProfileDesc::Map::iterator profilePtr = profileDescMap.find(profile);
	if (profilePtr == profileDescMap.end()) {
		profilePtr =  profileDescMap.find(ProfileDesc::getDefaultName());
		if (profilePtr == profileDescMap.end())
			throw Message(Message::LEVEL_CRIT, "HandlerTrajectory::setProfile(): unable to find default profile \"%s\"", ProfileDesc::getDefaultName().c_str());
		context.notice("HandlerTrajectory::setProfile(): unable to find profile \"%s\", using default profile \"%s\"\n", profile, ProfileDesc::getDefaultName().c_str());
	}
	pProfileDesc = &*profilePtr;

	// initialise
	for (Configspace::Index i = info.getJoints().begin(); i < info.getJoints().end(); ++i) {
		const bool isArm = armInfo.getJoints().contains(i);
		const bool isHand = handInfo.getJoints().contains(i);
		const size_t j = i - info.getJoints().begin();

		// velocity
		pProfileDesc->second.profileDesc->velocity[j] = pProfileDesc->second.velFac.coord[i] * (isArm ? pProfileDesc->second.velFac.arm : isHand ? pProfileDesc->second.velFac.hand : pProfileDesc->second.velFac.other);
		// accleleration
		pProfileDesc->second.profileDesc->acceleration[j] = pProfileDesc->second.accFac.coord[i] * (isArm ? pProfileDesc->second.accFac.arm : isHand ? pProfileDesc->second.accFac.hand : pProfileDesc->second.accFac.other);

		// distance
		distance[i] = pProfileDesc->second.disFac.coord[i] * (isArm ? pProfileDesc->second.disFac.arm : isHand ? pProfileDesc->second.disFac.hand : pProfileDesc->second.disFac.other);
		// extrapolation
		extrapolation[i] = pProfileDesc->second.extFac.coord[i] * (isArm ? pProfileDesc->second.extFac.arm : isHand ? pProfileDesc->second.extFac.hand : pProfileDesc->second.extFac.other);
		// command
		command[i] = pProfileDesc->second.cmdFac.coord[i] * (isArm ? pProfileDesc->second.cmdFac.arm : isHand ? pProfileDesc->second.cmdFac.hand : pProfileDesc->second.cmdFac.other);
	}

	pProfile = pProfileDesc->second.profileDesc->create(*controller); // throws
}

void golem::data::HandlerTrajectory::createTrajectory(const ItemTrajectory& item, golem::Controller::State::Seq& trajectory) {
	if (!planner || !controller)
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): planner and controller are not set");
	if (pProfileDesc == nullptr || pProfile == nullptr)
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): invalid trajectory profile");
	if (item.getWaypoints().size() < 2)
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): At least two waypoints required");

	Real trjExtrapolation = pProfileDesc->second.trjExtrapolation;
	Real trjDuration = pProfileDesc->second.trjDuration;
	bool inverse = false;
	bool createAction = !pProfileDesc->second.action.empty();

	if (getUICallback() && getUICallback()->hasInputEnabled()) {
		Menu menu(context, *getUICallback());
		//inverse = menu.option("FI", "Use (F)orward/(I)nverse trajectory... ") == 'I';
		menu.readNumber("Trajectory duration: ", trjDuration);
		menu.readNumber("Trajectory extrapolation: ", trjExtrapolation);
		if (createAction) createAction = menu.option(0, "Create action: ", { "YES", "NO" }) == 0;
	}

	// make trajectory
	golem::Controller::State::Seq states = golem::WaypointCtrl::make(item.getWaypoints(), false), commands = golem::WaypointCtrl::make(item.getWaypoints(), true);
	
	// extrapolation delta
	golem::ConfigspaceCoord delta;
	for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i) 
		delta[i] = trjExtrapolation*extrapolation[i];

	// extrapolation
	const bool extrapol = trjExtrapolation > REAL_EPS;
	if (extrapol) {
		create(pProfileDesc->second.trjExtrapolationIndex, delta, states);
		create(pProfileDesc->second.trjExtrapolationIndex, delta, commands);
	}

	const size_t size = commands.size();
	{
		// waypoint pruning
		ScopeGuard guard([&]() {distRemovedCallback = nullptr;});
		distRemovedCallback = [&](size_t index) {
			//context.debug("data::HandlerTrajectory::import(): removing index #%u/%u\n", U32(index + 1), states.size());
			states.erase(states.begin() + index);
			commands.erase(commands.begin() + index);
		};
		// trajectory profiling
		trajectory = commands;
		for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i)
			if (Math::abs(command[i]) > REAL_EPS) { // no impedance control
				// replace with states
				for (size_t j = 0; j < trajectory.size(); ++j) {
					trajectory[j].cpos[i] = states[j].cpos[i];
					trajectory[j].cvel[i] = states[j].cvel[i];
					trajectory[j].cacc[i] = states[j].cacc[i];
				}
			}
		profile(trjDuration, trajectory);
	}

	// assign if command[i] > 0
	for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i)
		if (Math::abs(command[i]) > REAL_EPS) { // no impedance control
			// replace with commands
			for (size_t j = 0; j < trajectory.size(); ++j) {
				trajectory[j].cpos[i] = commands[j].cpos[i];
				trajectory[j].cvel[i] = commands[j].cvel[i];
				trajectory[j].cacc[i] = commands[j].cacc[i];
			}
			if (extrapol)
				trajectory.back().cpos[i] = command[i];
		}

	// debug
	context.debug("HandlerTrajectory::createTrajectory(): Trajectory size %u -> %u\n", (U32)size, (U32)trajectory.size());

	// stationary waypoint in the end
	trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + pProfileDesc->second.trjIdle));

	// action
	if (createAction) {
		GenWorkspaceChainState gwcc;
		controller->chainForwardTransform(trajectory.back().cpos, gwcc.wpos);
		for (Chainspace::Index i = info.getChains().begin(); i < info.getChains().end(); ++i)
			gwcc.wpos[i].multiply(gwcc.wpos[i], controller->getChains()[i]->getReferencePose()); // reference pose
		gwcc.t = trajectory.back().t;

		GenWorkspaceChainState::Seq wAction;
		//wAction.push_back(gwcc);
		for (Mat34Seq::const_iterator i = pProfileDesc->second.action.begin(); i != pProfileDesc->second.action.end(); ++i) {
			GenWorkspaceChainState gwccTrn = gwcc;
			gwccTrn.wpos[armInfo.getChains().begin()].multiply(*i, gwcc.wpos[armInfo.getChains().begin()]);
			gwccTrn.t += pProfileDesc->second.trjDuration;
			wAction.push_back(gwccTrn);
		}
		// find trajectory
		golem::Controller::State::Seq cAction;
		if (!const_cast<Planner*>(planner)->findLocalTrajectory(trajectory.back(), wAction.begin(), wAction.end(), cAction, cAction.end()))
			throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): Unable to find action trajectory");
		// add
		trajectory.insert(trajectory.end(), ++cAction.begin(), cAction.end());
		trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + pProfileDesc->second.trjIdle));
	}
}

//------------------------------------------------------------------------------

Real golem::data::HandlerTrajectory::distConfigspaceCoord(const ConfigspaceCoord& prev, const ConfigspaceCoord& next) const {
	Real dist = REAL_ZERO;
	for (Configspace::Index i = info.getJoints().begin(); i < info.getJoints().end(); ++i)
		dist += distance[i]*Math::sqr(prev[i] - next[i]);
	return Math::sqrt(dist);
}

Real golem::data::HandlerTrajectory::distCoord(Real prev, Real next) const {
	return Math::abs(prev - next);
}

void golem::data::HandlerTrajectory::distRemoved(size_t index) const {
	if (distRemovedCallback) distRemovedCallback(index);
}

bool golem::data::HandlerTrajectory::distCoordPlanning(const Configspace::Index& index) const {
	return (armInfo.getJoints().contains(index) || handInfo.getJoints().contains(index));
}

bool golem::data::HandlerTrajectory::distCoordInterpolation(const Configspace::Index& index) const {
	return !armInfo.getJoints().contains(index) && !handInfo.getJoints().contains(index);
}

//------------------------------------------------------------------------------

Item::Ptr golem::data::HandlerTrajectory::import(const std::string& path) {
	// check extension
	if (std::find(importTypes.begin(), importTypes.end(), getExt(path)) == importTypes.end())
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::import(): unknown file type %s", getExt(path).c_str());

	RenderBlock renderBlock(*this); // do not show cache

	Item::Ptr item(create());
	ItemTrajectory* itemTrajectory = to<ItemTrajectory>(item.get());
	Menu::Ptr menu(getUICallback() && getUICallback()->hasInputEnabled() ? new Menu(context, *getUICallback()) : nullptr);

	// HDF5 format
	if (path.rfind(importHDF5FileExt) != std::string::npos) {
		Controller::State state = controller->createState(), command = state;

		golem::Import::HDF5DumpRealSeqMap map;
		if (menu != nullptr) {
			menu->readString("Enter robot trajectory dataset name: ", importHDF5RobotTrj);
		}

		map.insert(std::make_pair(importHDF5RobotTrj, [&](const std::string& name, size_t index, const golem::RealSeq& seq) {
			// subsampling in range <begin, end)
			if (index >= importRobotTrj.begin && index < importRobotTrj.end && (index - importRobotTrj.begin) % importRobotTrj.subsampling == 0) {
				// state: reset
				controller->setToDefault(state);
				// state: increment time stamp by default
				if (!itemTrajectory->waypoints.empty()) state.t = itemTrajectory->waypoints.back().state.t + importRobotTrj.interval;
				// state: update using custom map
				ImportState::update(importRobotTrj.stateMap, seq, state);

				// command: reset
				controller->setToDefault(command);
				// command: increment time stamp by default
				if (!itemTrajectory->waypoints.empty()) command.t = itemTrajectory->waypoints.back().command.t + importRobotTrj.interval;
				// command: update using custom map
				ImportState::update(importRobotTrj.commandMap, seq, command);

				// update waypoints
				itemTrajectory->waypoints.push_back(WaypointCtrl(state, command));
			}
		}));

		// read datasets from file
		golem::Import().hdf5DumpRealSeq(path, map);
		// assert data validity
		if (itemTrajectory->waypoints.empty())
			throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::import(): empty robot trajectory");
	}

	itemTrajectory->waypointFile.setModified(true);

	return item;
}

const StringSeq& golem::data::HandlerTrajectory::getImportFileTypes() const {
	return importTypes;
}

//------------------------------------------------------------------------------

void golem::data::HandlerTrajectory::convert(const golem::Configspace::Range& inpConfigspaceRange, const golem::Configspace::Range& outConfigspaceRange, const golem::Chainspace::Range& inpChainspaceRange, const golem::Chainspace::Range& outChainspaceRange, golem::ConfigspaceCoord& val) {
	context.info("HandlerTrajectory::convert(Configspace): config_[%zd, %zd) -> config_[%zd, %zd), chain_[%zd, %zd) -> chain_[%zd, %zd)\n", *inpConfigspaceRange.begin(), *inpConfigspaceRange.end(), *outConfigspaceRange.begin(), *outConfigspaceRange.end(), *inpChainspaceRange.begin(), *inpChainspaceRange.end(),	*outChainspaceRange.begin(), *outChainspaceRange.end());
	// set to default everything out of range
	const golem::ConfigspaceCoord coords = val;
	val = defaultState->cpos;
	val.set(outConfigspaceRange, coords);
}

void golem::data::HandlerTrajectory::convert(const golem::Reservedspace::Range& inpRange, const golem::Reservedspace::Range& outRange, golem::ReservedCoord& val) {
	context.info("HandlerTrajectory::convert(Reservedspace): reserved_[%zd, %zd) -> reserved_[%zd, %zd)\n", *inpRange.begin(), *inpRange.end(), *outRange.begin(), *outRange.end());
	// set to default
	val = defaultState->reserved;
}

//------------------------------------------------------------------------------

void golem::data::HandlerTrajectory::createRender(const ItemTrajectory& item) {
	UI::CriticalSectionWrapper cs(getUICallback());

	boundsShow = false;
	pathRenderer.reset();
	if (!controller || item.waypoints.empty())
		return;
	
	// commands/states
	const Controller::State::Seq seq = golem::WaypointCtrl::make(item.waypoints, showCommands);

	// path
	if (pathRenderer.show)
		pathRenderer.fromConfigspace(*controller, seq.begin(), seq.end());

	// bounds interpolation
	ConfigspaceCoord cc;
	if (seq.size() > 1) {
		// compute distances
		RealSeq distance;
		distance.push_back(REAL_ZERO);
		for (Controller::State::Seq::const_iterator j = seq.begin(), i = j++; j != seq.end(); ++i, ++j) {
			Real d = REAL_ZERO;
			for (Configspace::Index k = info.getJoints().begin(); k != info.getJoints().end(); ++k)
				d += Math::sqr(i->cpos[k] - j->cpos[k]);
			distance.push_back(distance.back() + Math::sqrt(d));
		}
		// find neighbours
		const_cast<Real&>(item.pathPosition) = Math::clamp(item.pathPosition + pathPositionInc, REAL_ZERO, REAL_ONE);
		pathPositionInc = REAL_ZERO;
		const Real pos = item.pathPosition * distance.back();
		size_t i = 1;
		for (; i < distance.size() - 1 && pos > distance[i]; ++i);
		const_cast<size_t&>(item.pathWaypoint) = i;
		const Real d = distance[i] - distance[i - 1];
		const Real s = d > REAL_EPS ? (pos - distance[i - 1])/d : REAL_ZERO;
		const_cast<Real&>(item.pathInterpol) = s;
		// interpolate between neighbours
		const ConfigspaceCoord prev = seq[i - 1].cpos, next = seq[i].cpos;
		for (Configspace::Index k = info.getJoints().begin(); k != info.getJoints().end(); ++k)
			cc[k] = (REAL_ONE - s)*prev[k] + (s)*next[k];
	}
	else {
		cc = seq.front().cpos;
		const_cast<Real&>(item.pathPosition) = REAL_ZERO;
		const_cast<size_t&>(item.pathWaypoint) = 0;
		const_cast<Real&>(item.pathInterpol) = REAL_ZERO;
	}

	if (contactPathRequest) {
		contactPathRequest = false;
		const_cast<size_t&>(item.contactPathWaypoint) = item.pathWaypoint;
		const_cast<Real&>(item.contactPathInterpol) = item.pathInterpol;
	}

	WorkspaceJointCoord wjc;
	controller->jointForwardTransform(cc, wjc);

	// chain bounds
	for (Chainspace::Index i = info.getChains().begin(); i != info.getChains().end(); ++i) {
		const Bounds::Seq& boundsSeq = chainBoundsSeq[i];
		const Mat34Seq& mat34Seq = chainMat34Seq[i];
		if (boundsSeq.empty())
			continue;
		const Chain* chain = controller->getChains()[i];
		const Chainspace::Index linkedChainIndex = chain->getLinkedChainIndex();
		Mat34 pose = linkedChainIndex < i ? wjc[info.getJoints(linkedChainIndex).end() - 1] : controller->getGlobalPose();
		pose.multiply(pose, chain->getLocalPose());
		for (size_t j = 0; j < boundsSeq.size(); ++j)
			boundsSeq[j]->multiplyPose(pose, mat34Seq[j]);
	}
	// joint bounds
	for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i) {
		const Bounds::Seq& boundsSeq = jointBoundsSeq[i];
		const Mat34Seq& mat34Seq = jointMat34Seq[i];
		const Mat34 pose = wjc[i];
		for (size_t j = 0; j < boundsSeq.size(); ++j)
			boundsSeq[j]->multiplyPose(pose, mat34Seq[j]);
	}

	boundsShow = true;
}

void golem::data::HandlerTrajectory::render() const {
	if (hasRenderBlock()) return;

	// path
	pathRenderer.render();
	// bounds
	if (boundsShow) {
		boundsRenderer.setSolidColour(boundsSolidColour);
		boundsRenderer.renderSolid(boundsSeq.begin(), boundsSeq.end());
	}
}

void golem::data::HandlerTrajectory::customRender() const {
}

//------------------------------------------------------------------------------

void golem::data::HandlerTrajectory::mouseHandler(int button, int state, int x, int y) {
	if (hasRenderBlock()) return;

	const int mask = button & ~UIKeyboardMouseCallback::KEY_MASK;
	button &= UIKeyboardMouseCallback::KEY_MASK;

	if (state == GLUT_DOWN && (mask & UIKeyboardMouseCallback::KEY_CTRL)) {
		if (button == 3 && pathIncStep > 0)
			--pathIncStep;
		if (button == 4 && pathIncStep < U32(Math::abs(golem::numeric_const<Real>::MIN_EXP)) - 1)
			++pathIncStep;

		const Real increment = pathInc*Math::pow(REAL_TWO, -Real(pathIncStep));
		context.write("Increment path = %f\n", increment);
	}
	if (state == GLUT_DOWN && (mask == 0)) {
		const Real increment = pathInc*Math::pow(REAL_TWO, -Real(pathIncStep));
		pathPositionInc += button == 3 ? +increment : button == 4 ? -increment : button == 1 ? -REAL_MAX : REAL_ZERO;

		requestRender();
	}
}

void golem::data::HandlerTrajectory::motionHandler(int x, int y) {
}

void golem::data::HandlerTrajectory::keyboardHandler(int key, int x, int y) {
	if (hasRenderBlock()) return;

	switch (key) {
	case '3':
		showCommands = !showCommands;
		context.write("Show %s\n", showCommands ? "commands" : "states");
		requestRender();
		break;
	case '4':
		pathRenderer.show = !pathRenderer.show;
		context.write("Show path %s\n", pathRenderer.show ? "ON" : "OFF");
		requestRender();
		break;
	case '5':
		context.write("Setting contact trajectory position\n");
		contactPathRequest = true;
		requestRender();
		break;
	};
}

//------------------------------------------------------------------------------

void golem::XMLData(const std::string &attr, data::HandlerTrajectory::ImportState::Type& val, golem::XMLContext* xmlcontext, bool create) {
	std::string type;
	XMLData(attr, type, xmlcontext, create);
	transform(type.begin(), type.end(), type.begin(), tolower);
	val =
		type == "t" || type == "time" ? data::HandlerTrajectory::ImportState::TYPE_TIME :
		type == "pos" || type == "position" ? data::HandlerTrajectory::ImportState::TYPE_POSITION :
		type == "vel" || type == "velocity" ? data::HandlerTrajectory::ImportState::TYPE_VELOCITY :
		type == "acc" || type == "acceleration" ? data::HandlerTrajectory::ImportState::TYPE_VELOCITY :
		data::HandlerTrajectory::ImportState::TYPE_RESERVED;
}

void golem::XMLData(const std::string &attr, data::HandlerTrajectory::ImportFrame::Type& val, golem::XMLContext* xmlcontext, bool create) {
	std::string type;
	XMLData(attr, type, xmlcontext, create);
	transform(type.begin(), type.end(), type.begin(), tolower);
	val =
		type == "euler" ? data::HandlerTrajectory::ImportFrame::TYPE_EULER :
		type == "axis" ? data::HandlerTrajectory::ImportFrame::TYPE_AXIS :
		data::HandlerTrajectory::ImportFrame::TYPE_QUAT;
}

void golem::data::XMLData(data::HandlerTrajectory::ImportState::Map::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	val.load(xmlcontext);
}

