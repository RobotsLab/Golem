/** @file Ctrl.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Plugin/Defs.h>
#include <Golem/Plugin/Ctrl.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <sstream>
#include <iomanip>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

golem::Controller* golem::ControllerId::findController(const ControllerId& id, golem::Controller &controller) {
	if (controller.getID() == id.id)
		return &controller;

	golem::Controller* pController = nullptr;
	for (golem::Controller::PtrSeq::const_iterator i = controller.getControllers().begin(); i != controller.getControllers().end() && !pController; ++i)
		if (*i) pController = findController(id, **i);

	return pController;
}

golem::Controller* golem::ControllerId::findController(golem::Controller &controller) const {
	if (!hasId())
		throw Message(Message::LEVEL_ERROR, "ControllerId::findController(): controller id required");

	golem::Controller* pController = findController(*this, controller);
	if (pController)
		return pController;
	
	throw Message(Message::LEVEL_ERROR, "ControllerId::findController(): unknown controller id: %s", id.c_str());
}

golem::ControllerId::Info golem::ControllerId::findInfo(const golem::Controller &controller) const {
	const golem::ControllerId::Info info = hasId() ? findController(const_cast<golem::Controller&>(controller))->getStateInfo() : controller.getStateInfo();
	if (!hasRange())
		return info;

	if (!info.getChains().contains(range.begin()) && !info.getChains().contains(range.end() - 1))
		throw Message(Message::LEVEL_ERROR, "ControllerId::findInfo(): chains %zu..%zu not in controller range %zu..%zu", *range.begin(), *range.end(), *info.getChains().begin(), *info.getChains().end());

	golem::ControllerId::Info customInfo;
	customInfo.resetJoints(*info.getJoints(range.begin()).begin(), *range.begin());
	for (Chainspace::Index i = range.begin(); i < range.end(); ++i)
		customInfo.addJoints(info.getJoints(i).size());
	return customInfo;
}

std::string golem::ControllerId::toString() const {
	std::stringstream str;
	str << "(";
	if (hasId())
		str << id;
	if (hasId() && hasRange())
		str << ", ";
	if (hasRange())
		str << *range.begin() << "-" << *range.end();
	str << ")";
	return str.str();
}

//------------------------------------------------------------------------------

void golem::XMLData(golem::ControllerId& val, golem::XMLContext* context, bool create) {
	if (!create)
		val.clear();

	try {
		golem::XMLData("id", val.id, context, create);
	}
	catch (golem::MsgXMLParserAttributeNotFound&) {}
	try {
		golem::U32 begin = 0;
		golem::XMLData("begin", begin, context, create);
		golem::U32 end = 0;
		golem::XMLData("end", end, context, create);
		val.range.set(begin, end - begin);
	}
	catch (golem::MsgXMLParserAttributeNotFound&) {}
}

//------------------------------------------------------------------------------

std::string golem::plannerDebug(const golem::Planner& planner) {
	std::stringstream str;

	const Heuristic& heuristic = planner.getHeuristic();
	const Controller& controller = planner.getController();
	str << controller.getID() << ": chains=" << controller.getStateInfo().getChains().size() << ", joints=" << controller.getStateInfo().getJoints().size() << ", collisions=" << (heuristic.getDesc().collisionDesc.enabled ? "ENABLED" : "DISABLED");

	return str.str();
}

std::string golem::plannerConfigspaceDebug(const golem::Planner& planner, const golem::ConfigspaceCoord* c) {
	std::stringstream str;

	const Heuristic& heuristic = planner.getHeuristic();
	const Controller& controller = planner.getController();
	const Heuristic::JointDesc::JointSeq& jointDesc = heuristic.getJointDesc();
	const Chainspace::Range chains = controller.getStateInfo().getChains();
	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i) {
		str << std::setprecision(3) << "{" << controller.getChains()[i]->getName() << ": ";
		const Configspace::Range joints = controller.getStateInfo().getJoints(i);
		for (Configspace::Index j = joints.begin(); j < joints.end(); ++j) {
			str << "(" << j - controller.getStateInfo().getJoints().begin() << ", ";
			str << (jointDesc[j]->enabled ? "Y" : "N") << ", ";
			if (c != nullptr) str << (*c)[j] << "/";
			str << jointDesc[j]->dfltPos << (j < joints.end() - 1 ? "), " : ")");
		}
		str << (i < chains.end() - 1 ? "}, " : "}");
	}

	return str.str();
}

std::string golem::plannerWorkspaceDebug(const golem::Planner& planner, const golem::WorkspaceChainCoord* w) {
	std::stringstream str;

	const Heuristic& heuristic = planner.getHeuristic();
	const Controller& controller = planner.getController();
	const Heuristic::ChainDesc::ChainSeq& chainDesc = heuristic.getChainDesc();
	const Chainspace::Range chains = controller.getStateInfo().getChains();
	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i) {
		str << std::setprecision(3) << "{" << controller.getChains()[i]->getName() << ": ";
		if (w != nullptr) {
			const Vec3 p((*w)[i].p);
			const Quat q((*w)[i].R);
			chainDesc[i]->enabledLin ? str << "lin=(" << p.x << ", " << p.y << ", " << p.z << "), " : str << "lin=N, ";
			chainDesc[i]->enabledAng ? str << "ang=(" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << "), " : str << "ang=N";
		}
		else {
			chainDesc[i]->enabledLin ? str << "lin=Y, " : str << "lin=N, ";
			chainDesc[i]->enabledAng ? str << "ang=Y, " : str << "ang=N";
		}

		str << (i < chains.end() - 1 ? "}, " : "}");
	}

	return str.str();
}

//------------------------------------------------------------------------------

golem::WaypointCtrl::WaypointCtrl(const golem::Controller::State& state, const golem::Controller::State& command) : state(state), command(command) {
}

golem::WaypointCtrl::WaypointCtrl(const golem::Controller& controller) : state(controller.createState()), command(state) {
	// no initialisation
}

golem::WaypointCtrl golem::WaypointCtrl::create(const golem::Controller& controller) {
	golem::WaypointCtrl waypoint(controller);
	controller.setToDefault(waypoint.state);
	waypoint.command = waypoint.state;
	return waypoint;
}

golem::WaypointCtrl golem::WaypointCtrl::lookup(const golem::Controller& controller, golem::SecTmReal t) {
	golem::WaypointCtrl waypoint(controller);
	controller.lookupState(t, waypoint.state);
	controller.lookupCommand(t, waypoint.command);
	return waypoint;
}

golem::Controller::State::Seq golem::WaypointCtrl::make(const golem::WaypointCtrl::Seq& waypoints, bool command) {
	golem::Controller::State::Seq seq;
	std::for_each(waypoints.begin(), waypoints.end(), [&](const golem::WaypointCtrl& waypoint) {seq.push_back(command ? waypoint.command : waypoint.state); });
	return seq;
}

golem::WaypointCtrl::Seq golem::WaypointCtrl::make(const golem::Controller::State::Seq& states, const golem::Controller::State::Seq& commands) {
	if (states.size() != commands.size())
		throw Message(Message::LEVEL_ERROR, "WaypointCtrl::make(): invalid number of states or commands");
	
	golem::WaypointCtrl::Seq waypoints;
	for (size_t i = 0; i < commands.size(); ++i)
		waypoints.push_back(golem::WaypointCtrl(states[i], commands[i]));

	return waypoints;
}

//------------------------------------------------------------------------------

const std::string golem::ManifoldCtrl::Header::ID = "golem::ManifoldCtrl";
const golem::Header::Version golem::ManifoldCtrl::Header::VERSION = golem::Header::Version({ (1 << 0) /* major */ | (0 << 16) /* minor */ });

const char* golem::ManifoldCtrl::name[ManifoldCtrl::SIZE] = {
	"TrnX",
	"TrnY",
	"TrnZ",
	"RotX",
	"RotY",
	"RotZ",
};

void golem::ManifoldCtrl::Appearance::load(const golem::XMLContext* xmlcontext) {
	//golem::XMLData(norm, xmlcontext->getContextFirst("norm"));
	golem::XMLData(visNorm, xmlcontext->getContextFirst("vis_norm"));
	golem::XMLData(visRange, xmlcontext->getContextFirst("vis_range"));

	U32 transparency = static_cast<U32>(this->transparency);
	golem::XMLData("transparency", transparency, const_cast<golem::XMLContext*>(xmlcontext));
	this->transparency = static_cast<U8>(transparency);;

	golem::XMLData("show", show, const_cast<golem::XMLContext*>(xmlcontext));
	try {
		golem::XMLData("show_frame", showFrame, const_cast<golem::XMLContext*>(xmlcontext));
	}
	catch (const golem::MsgXMLParser&) {}
}

bool golem::ManifoldCtrl::isAvailable() const {
	return (this->frameDev.v.x > REAL_EPS || this->frameDev.v.y > REAL_EPS || this->frameDev.v.z > REAL_EPS) && (this->frameDev.w.x > REAL_EPS || this->frameDev.w.y > REAL_EPS || this->frameDev.w.z > REAL_EPS);
}

void golem::XMLData(golem::ManifoldCtrl& val, golem::XMLContext* context, bool create) {
	golem::XMLData(val.frame, context->getContextFirst("frame", create), create);
	golem::XMLData(val.frameDev, context->getContextFirst("frame_dev", create), create);
}

void golem::ManifoldCtrl::Appearance::draw(const ManifoldCtrl& manifold, const Mat34& frame, golem::DebugRenderer& renderer) const {
	if (show) {
		const Mat34 trn = frame * manifold.frame;

		if (manifold.isAvailable()) {
			Twist trnDev = manifold.frameDev;

			const Real vmin = std::min({ trnDev.v.x, trnDev.v.y, trnDev.v.z });
			const Real vmax = std::max({ trnDev.v.x, trnDev.v.y, trnDev.v.z });
			trnDev.v += (visRange.lin*vmax - vmin) / (REAL_ONE - visRange.lin);
			const Real vmag = std::max({ trnDev.v.x, trnDev.v.y, trnDev.v.z });
			trnDev.v *= visNorm.lin / vmag;

			const Real wmin = std::min({ trnDev.w.x, trnDev.w.y, trnDev.w.z });
			const Real wmax = std::max({ trnDev.w.x, trnDev.w.y, trnDev.w.z });
			trnDev.w += (visRange.ang*wmax - wmin) / (REAL_ONE - visRange.ang);
			const Real wmag = std::max({ trnDev.w.x, trnDev.w.y, trnDev.w.z });
			trnDev.w *= visNorm.ang / wmag;
			trnDev.w /= trnDev.v;

			renderer.addAxis3D(0, trn, trnDev.v.x, RGBA(RGBA::RED._rgba.r, RGBA::RED._rgba.g, RGBA::RED._rgba.b, transparency), trnDev.w.x, REAL_ONE);
			renderer.addAxis3D(1, trn, trnDev.v.y, RGBA(RGBA::GREEN._rgba.r, RGBA::GREEN._rgba.g, RGBA::GREEN._rgba.b, transparency), trnDev.w.y, REAL_ONE);
			renderer.addAxis3D(2, trn, trnDev.v.z, RGBA(RGBA::BLUE._rgba.r, RGBA::BLUE._rgba.g, RGBA::BLUE._rgba.b, transparency), trnDev.w.z, REAL_ONE);
		}
		else if (showFrame)
			renderer.addAxes3D(trn, Vec3(visNorm.lin));
	}
}

//------------------------------------------------------------------------------

template <> void golem::Stream::read(Controller::State& state) const {
	U32 begin, end, size;

	bool useHandlerConfigspace = false;
	StreamHandlerConfigspace* handlerConfigspace = is<StreamHandlerConfigspace>(getHandler());

	*this >> begin >> end >> size;
	const Configspace::Range jointsRange = Configspace::Range(begin, size);
	if (state.getInfo().getJoints().size() != jointsRange.size())
		if (handlerConfigspace)
			useHandlerConfigspace = true;
		else
			throw Message(Message::LEVEL_ERROR, "Stream::read(Controller::State&): configspace size %d does not match data %d", state.getInfo().getJoints().size(), jointsRange.size());

	*this >> begin >> end >> size;
	const Chainspace::Range chainsRange = Chainspace::Range(begin, size);
	if (state.getInfo().getChains().size() != chainsRange.size())
		if (handlerConfigspace)
			useHandlerConfigspace = true;
		else
			throw Message(Message::LEVEL_ERROR, "Stream::read(Controller::State&): chainspace size %d does not match data %d", state.getInfo().getChains().size(), chainsRange.size());

	bool useHandlerReservedspace = false;
	StreamHandlerReservedspace* handlerReservedspace = is<StreamHandlerReservedspace>(getHandler());

	*this >> begin >> end >> size;
	const Reservedspace::Range reservedRange = Reservedspace::Range(begin, size);
	if (state.getInfo().getReserved().size() != reservedRange.size())
		if (handlerReservedspace)
			useHandlerReservedspace = true;
		else
			throw Message(Message::LEVEL_ERROR, "Stream::read(Controller::State&): reservedspace size %d does not match data %d", state.getInfo().getReserved().size(), reservedRange.size());

	*this >> state.t;

	read(state.cpos.data() + *jointsRange.begin(), (size_t)jointsRange.size());
	read(state.cvel.data() + *jointsRange.begin(), (size_t)jointsRange.size());
	read(state.cacc.data() + *jointsRange.begin(), (size_t)jointsRange.size());
	if (useHandlerConfigspace) {
		handlerConfigspace->convert(jointsRange, state.getInfo().getJoints(), chainsRange, state.getInfo().getChains(), state.cpos);
		handlerConfigspace->convert(jointsRange, state.getInfo().getJoints(), chainsRange, state.getInfo().getChains(), state.cvel);
		handlerConfigspace->convert(jointsRange, state.getInfo().getJoints(), chainsRange, state.getInfo().getChains(), state.cacc);
	}

	read(state.reserved.data() + *reservedRange.begin(), (size_t)reservedRange.size());
	if (useHandlerReservedspace) {
		handlerReservedspace->convert(reservedRange, state.getInfo().getReserved(), state.reserved);
	}
}

template <> void golem::Stream::write(const Controller::State& state) {
	*this << U32(*state.getInfo().getJoints().begin()) << U32(*state.getInfo().getJoints().end()) << U32(state.getInfo().getJoints().size());
	*this << U32(*state.getInfo().getChains().begin()) << U32(*state.getInfo().getChains().end()) << U32(state.getInfo().getChains().size());
	*this << U32(*state.getInfo().getReserved().begin()) << U32(*state.getInfo().getReserved().end()) << U32(state.getInfo().getReserved().size());
	*this << state.t;
	write(state.cpos.data() + *state.getInfo().getJoints().begin(), (size_t)state.getInfo().getJoints().size());
	write(state.cvel.data() + *state.getInfo().getJoints().begin(), (size_t)state.getInfo().getJoints().size());
	write(state.cacc.data() + *state.getInfo().getJoints().begin(), (size_t)state.getInfo().getJoints().size());
	write(state.reserved.data() + *state.getInfo().getReserved().begin(), (size_t)state.getInfo().getReserved().size());
}

template <> void golem::Stream::read(golem::WaypointCtrl& waypoint) const {
	read(waypoint.state);
	read(waypoint.command);
}

template <> void golem::Stream::write(const golem::WaypointCtrl& waypoint) {
	write(waypoint.state);
	write(waypoint.command);
}

//------------------------------------------------------------------------------

golem::ManipDist::ManipDist(Profile & profile) : profile(profile), callbackDist(profile.getCallbackDist()) {
	profile.setCallbackDist(*this);
}

golem::ManipDist::~ManipDist() {
	profile.setCallbackDist(*callbackDist);
}

Real golem::ManipDist::distConfigspaceCoord(const ConfigspaceCoord & prev, const ConfigspaceCoord & next) const {
	return callbackDist->distConfigspaceCoord(prev, next);
}

Real golem::ManipDist::distCoord(Real prev, Real next) const {
	return callbackDist->distCoord(prev, next);
}

bool golem::ManipDist::distCoordPlanning(const Configspace::Index & index) const {
	return callbackDist->distCoordPlanning(index) || callbackDist->distCoordInterpolation(index);
}

//------------------------------------------------------------------------------

template <> void golem::Stream::read(golem::ManifoldCtrl& value) const {
	value.header.load(*this);

	read(value.frame);
	read(value.frameDev);
}

template <> void golem::Stream::write(const golem::ManifoldCtrl& value) {
	value.header.store(*this);

	write(value.frame);
	write(value.frameDev);
}

//------------------------------------------------------------------------------

