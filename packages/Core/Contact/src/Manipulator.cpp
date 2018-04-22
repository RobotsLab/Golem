/** @file Manipulator.cpp
 *
 * Contact manipulator interface
 *
 * @author Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

//------------------------------------------------------------------------------

#include <Golem/Contact/Manipulator.h>
#include <Golem/Plugin/Ctrl.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const std::string Manipulator::Link::name[] = { "Joint", "Base", "Any" };

bool Manipulator::Link::is(const std::string& key) const {
	// Any
	if (key.compare(getName(TYPE_ANY)) == 0)
		return true;
	// TODO range compare
	return false;
}

//------------------------------------------------------------------------------

Manipulator::Manipulator(const Desc& desc, const golem::Planner& planner, const ControllerId::Seq& controllerIDSeq) : planner(planner), controller(planner.getController()), context(const_cast<golem::Context&>(planner.getController().getContext())) {
	desc.assertValid(Assert::Context("Manipulator()."));
	this->desc = desc;

	if (controllerIDSeq.size() < 2)
		throw Message(Message::LEVEL_CRIT, "Manipulator(): arm and hand are required");

	// joint and chain info
	info = controller.getStateInfo();
	armInfo = controllerIDSeq[0].findInfo(controller);
	handInfo = controllerIDSeq[1].findInfo(controller);

	// local frame
	try {
		localFrame = controllerIDSeq[1].findController(const_cast<golem::Controller&>(controller))->getGlobalPose();
	}
	catch (const Message&) {
		context.info("Manipulator::Manipulator(): using chain local frame\n");
		localFrame = controller.getChains()[armInfo.getChains().begin()]->getLocalPose();
	}
	//localFrame = controller.getChains()[armInfo.getChains().begin()]->getLocalPose();

	// range
	if (armInfo.getJoints().end() != handInfo.getJoints().begin())
		throw Message(Message::LEVEL_CRIT, "Manipulator(): arm joints must precede hand joints");
	configRange.set(*armInfo.getJoints().begin(), armInfo.getJoints().size() + handInfo.getJoints().size());
	
	// joint position limits
	configMin = controller.getMin().cpos;
	configMax = controller.getMax().cpos;
}

Manipulator::~Manipulator() {
}

//------------------------------------------------------------------------------

void Manipulator::clamp(Config& config) const {
	golem::GenConfigspaceCoord coords;
	coords.cpos = config.config;
	coords.cvel.setToDefault(configRange);
	coords.cacc.setToDefault(configRange);
	controller.clampConfig(coords);
	config.config = coords.cpos;
	// ranges
	//Math::clamp(&config.config[configRange.begin()], &config.config[configRange.end()], &configMin[configRange.begin()], &configMax[configRange.begin()]);
	// mapping
	//for (Configspace::Index i = configRange.begin(); i < configRange.end(); ++i) {
	//	const I32 index = desc.configMap[i];
	//	if (index != 0)
	//		config.config[i] = index > 0 ? +config.config[configRange.begin() + (index - 1)] : -config.config[configRange.begin() - (index + 1)];
	//}
}

golem::Mat34 Manipulator::getBaseFrame(const golem::ConfigspaceCoord& config) const {
	Mat34 frame;

	const golem::Chain* chain = controller.getChains()[armInfo.getChains().begin()];
	chain->chainForwardTransform(&config[armInfo.getJoints().begin()], frame);

	return (controller.getGlobalPose() * frame);
}

void Manipulator::getJointFrames(const golem::ConfigspaceCoord& config, const golem::Mat34& base, golem::WorkspaceJointCoord& joints) const {
	for (Chainspace::Index j = handInfo.getChains().begin(); j < handInfo.getChains().end(); ++j) {
		const Chain* chain = controller.getChains()[j];
		const Configspace::Index joint = info.getJoints(j).begin();
		chain->jointForwardTransform(&config[joint], &joints[joint]);
	}
	for (Configspace::Index j = handInfo.getJoints().begin(); j < handInfo.getJoints().end(); ++j)
		joints[j].multiply(base, joints[j]);
}

golem::Mat34 Manipulator::getLocalFrame() const {
	return localFrame;
}

golem::Mat34 Manipulator::getReferenceFrame() const {
	return controller.getChains()[armInfo.getChains().begin()]->getReferencePose();
}

//------------------------------------------------------------------------------

bool Manipulator::hasBaseBounds() const {
	for (Chainspace::Index j = handInfo.getChains().begin(); j < handInfo.getChains().end(); ++j)
		if (controller.getChains()[j]->hasBoundsDesc())
			return true;
	return false;
}

bool Manipulator::hasJointBounds(golem::Configspace::Index joint) const {
	return controller.getJoints()[joint]->hasBoundsDesc();
}

golem::Bounds::Seq Manipulator::getBaseBounds() const {
	golem::Bounds::Seq boundsSeq;
	getBaseBounds(golem::Mat34::identity(), boundsSeq);
	return boundsSeq;
}

void Manipulator::getBaseBounds(const golem::Mat34& frame, golem::Bounds::Seq& boundsSeq) const {
	for (Chainspace::Index j = handInfo.getChains().begin(); j < handInfo.getChains().end(); ++j) {
		const Chain* chain = controller.getChains()[j];
		golem::Bounds::Desc::SeqPtr boundsDescSeq = chain->getBoundsDescSeq();
		for (golem::Bounds::Desc::Seq::const_iterator i = boundsDescSeq->begin(); i != boundsDescSeq->end(); ++i) {
			boundsSeq.push_back(i->get()->create());
			boundsSeq.back()->multiplyPose(frame * chain->getLocalPose(), boundsSeq.back()->getPose());
		}
	}
}

golem::Bounds::Seq Manipulator::getJointBounds(golem::Configspace::Index joint) const {
	golem::Bounds::Seq boundsSeq;
	getJointBounds(joint, golem::Mat34::identity(), boundsSeq);
	return boundsSeq;
}

void Manipulator::getJointBounds(golem::Configspace::Index joint, const golem::Mat34& frame, golem::Bounds::Seq& boundsSeq) const {
	golem::Bounds::Desc::SeqPtr boundsDescSeq = controller.getJoints()[joint]->getBoundsDescSeq();
	for (golem::Bounds::Desc::Seq::const_iterator i = boundsDescSeq->begin(); i != boundsDescSeq->end(); ++i) {
		boundsSeq.push_back(i->get()->create());
		boundsSeq.back()->multiplyPose(frame, boundsSeq.back()->getPose());
	}
}

golem::Bounds::Seq Manipulator::getBounds(const golem::ConfigspaceCoord& config, const golem::Mat34& frame) const {
	golem::Bounds::Seq boundsSeq;
	getBaseBounds(frame, boundsSeq);
	golem::WorkspaceJointCoord joints;
	getJointFrames(config, frame, joints);
	for (Configspace::Index j = handInfo.getJoints().begin(); j < handInfo.getJoints().end(); ++j)
		getJointBounds(j, joints[j], boundsSeq);
	return boundsSeq;
}

//------------------------------------------------------------------------------

golem::Controller::State Manipulator::getState(const Waypoint& waypoint) const {
	golem::Controller::State state = controller.createState();

	// setup reserved area - required for real devices!
	//controller.setToDefault(state);
	controller.lookupState(golem::SEC_TM_REAL_MAX, state);

	// position control
	state.cpos.set(configRange, waypoint.config);
	state.cvel.setToDefault(info.getJoints());
	state.cacc.setToDefault(info.getJoints());

	return state;
}

Manipulator::Config Manipulator::getConfig(const golem::Controller::State& state, const Config* prev) const {
	Config config(state.cpos, getBaseFrame(state.cpos));

	// account for quaternion dual cover of SO(3)
	if (prev && prev->frame.q.dot(config.frame.q) < prev->frame.q.dot(-config.frame.q))
		config.frame.q.negate();

	// create waypoint
	return config;
}

//------------------------------------------------------------------------------

RBDist Manipulator::find(Waypoint::Seq& path) const {
	if (path.empty())
		throw golem::Message(golem::Message::LEVEL_ERROR, "Manipulator::find(): Empty path");

	// Compute a sequence of targets corresponding to the transformed end-effector
	GenWorkspaceChainState::Seq seq;
	for (Waypoint::Seq::const_iterator i = path.begin(); i != path.end(); ++i) {
		GenWorkspaceChainState gwcs;
		gwcs.wpos[armInfo.getChains().begin()].multiply(i->frame.toMat34(), getReferenceFrame()); // 1:1
		gwcs.t = REAL_ONE*(i - path.begin()); // just not zero
		seq.push_back(gwcs);
	}

	// Find initial target position
	Controller::State cend = getState(path.front());
	if (!const_cast<golem::Planner&>(planner).findTarget(cend, seq[0], cend))
		throw golem::Message(golem::Message::LEVEL_ERROR, "Manipulator::find(): Unable to find initial configuration");
	// Find remaining position sequence
	Controller::State::Seq trajectory;
	if (seq.size() == 1)
		trajectory.push_back(cend);
	{
		// disable waypoint prunning
		const bool prunning = planner.getProfile()->hasPruning();
		planner.getProfile()->setPruning(false);
		golem::ScopeGuard guard([=]() { planner.getProfile()->setPruning(prunning); });
		// run planner
		if (seq.size() > 1 && !const_cast<golem::Planner&>(planner).findLocalTrajectory(cend, ++seq.begin(), seq.end(), trajectory, trajectory.end(), golem::SecToMSec(desc.trajectoryTimeout * seq.size())))
			throw golem::Message(golem::Message::LEVEL_ERROR, "Manipulator::find(): Unable to find trajectory");
	}

	// update configurations and compute average error
	RBDist err;
	Waypoint::Seq::iterator j = path.begin();
	GenWorkspaceChainState::Seq::const_iterator k = seq.begin();
	for (Controller::State::Seq::const_iterator i = trajectory.begin(); i != trajectory.end(); ++i, ++j, ++k) {
		// copy arm position only
		j->config.set(armInfo.getJoints(), i->cpos);
		// error
		WorkspaceChainCoord wcc;
		controller.chainForwardTransform(i->cpos, wcc);
		wcc[armInfo.getChains().begin()].multiply(wcc[armInfo.getChains().begin()], getReferenceFrame());
		
		const RBDist werr(RBDist(RBCoord(wcc[armInfo.getChains().begin()]), RBCoord(k->wpos[armInfo.getChains().begin()])).getSqrt());
		//controller.getContext().verbose("Manipulator::findPath(): Pose error #%i/%i: lin=%.9f, ang=%.9f\n", (i - trajectory.begin()) + 1, trajectory.size(), werr.lin, werr.ang);
		err.add(err, werr);
	}

	err.multiply(REAL_ONE / path.size(), err);	
	//controller.getContext().verbose("Manipulator::findPath(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
	return err;
}

void Manipulator::create(const Waypoint::Seq& path, golem::WaypointCtrl::Seq& waypoints) const {
	if (path.empty())
		throw golem::Message(golem::Message::LEVEL_ERROR, "Manipulator::copy(): At least one waypoints required");

	waypoints.clear();
	SecTmReal t = context.getTimer().elapsed();
	for (Waypoint::Seq::const_iterator i = path.begin(); i != path.end(); ++i) {
		const Controller::State state(getState(*i), t += controller.getCycleDuration());
		Controller::State command = state; // by default
		if (desc.controlToCommand) desc.controlToCommand(i->control, command); // optional
		waypoints.push_back(golem::WaypointCtrl(state, command));
	}
}

Manipulator::Config Manipulator::interpolate(const Waypoint::Seq& path, golem::Real distance) const {
	if (path.empty())
		throw golem::Message(golem::Message::LEVEL_ERROR, "Manipulator::interpolate(): At least one waypoints required");
	else if (path.size() < 2)
		return path[0];

	// find upper and lower limits
	Waypoint::Seq::const_iterator lo, hi;
	Waypoint::bounds(path.begin(), path.end(), distance, lo, hi);

	// interpolation coefficients
	const Real dt = hi->getDistance() - lo->getDistance();
	if (dt <= REAL_EPS)
		throw Message(Message::LEVEL_ERROR, "Manipulator::interpolate(): Invalid waypoint distance: %f", dt);
	const Real s = (distance - lo->getDistance())/dt;

	//std::stringstream str;
	//for (auto&i : path)
	//	str << i.weight << ", ";
	//printf("path={%s}, lo-hi=%u-%u, lo-hi=%f-%f, dist=%f, s=%f\n",
	//	str.str().c_str(), std::distance(path.begin(), lo), std::distance(path.begin(), hi), lo->weight, hi->weight, distance, s);

	// interpolate frame and config
	Manipulator::Config config;
	config.frame.interpolate(lo->frame, hi->frame, s);
	Math::transform(&config.config[configRange.begin()], &config.config[configRange.end()], &lo->config[configRange.begin()], &hi->config[configRange.begin()], [=] (const Real& l, const Real& r) { return l + s*(r - l); });
	clamp(config);

	return config;
}

//------------------------------------------------------------------------------

void Manipulator::BoundsAppearance::draw(const golem::Bounds::Seq& bounds, golem::DebugRenderer& renderer) const {
	if (showSolid) {
		renderer.setColour(solidColour);
		renderer.addSolid(bounds.begin(), bounds.end());
	}
	if (showWire) {
		renderer.setColour(wireColour);
		renderer.setLineWidth(wireWidth);
		renderer.addWire(bounds.begin(), bounds.end());
	}
}

void Manipulator::Appearance::draw(const Manipulator& manipulator, const Config& config, golem::DebugRenderer& renderer) const {
	if (selectionIndex >= 0 && selectionIndex <= (U32)manipulator.getConfigRange().size()) {
		golem::WorkspaceJointCoord joints;
		manipulator.getJointFrames(config.config, config.frame.toMat34(), joints);
		Bounds::Seq bounds;
		if (selectionIndex < (U32)manipulator.getConfigRange().size()) {
			const Configspace::Index jointSelectionIndex = manipulator.getConfigRange().begin() + selectionIndex;
			manipulator.getJointBounds(jointSelectionIndex, joints[jointSelectionIndex], bounds);
		}
		else
			manipulator.getBaseBounds(config.frame.toMat34(), bounds);

		boundsSelection.draw(bounds, renderer);
	}

	if (showBounds) {
		// bounds
		bounds.draw(manipulator.getBounds(config.config, config.frame.toMat34()), renderer);
	}

	if (showFrames) {
		// chains frames
		renderer.addAxes3D(config.frame.toMat34(), chainsFrameSize);
		//renderer.addAxes3D(config.frame.toMat34() * manipulator.getBaseFrame(), chainsFrameSize);

		// joints frames
		golem::WorkspaceJointCoord joints;
		manipulator.getJointFrames(config.config, config.frame.toMat34(), joints);
		for (Configspace::Index j = manipulator.handInfo.getJoints().begin(); j < manipulator.handInfo.getJoints().end(); ++j)
			renderer.addAxes(joints[j], jointsFrameSize);
	}
}

//------------------------------------------------------------------------------

void golem::Manipulator::BoundsAppearance::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("show_solid", showSolid, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("show_wire", showWire, const_cast<golem::XMLContext*>(xmlcontext), false);

	golem::XMLData(solidColour, xmlcontext->getContextFirst("solid_colour"), false);
	golem::XMLData(wireColour, xmlcontext->getContextFirst("wire_colour"), false);
	golem::XMLData("wire_width", wireWidth, const_cast<golem::XMLContext*>(xmlcontext), false);
}

void golem::Manipulator::Appearance::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("show_bounds", showBounds, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("show_frames", showFrames, const_cast<golem::XMLContext*>(xmlcontext), false);

	bounds.load(xmlcontext->getContextFirst("bounds"));
	boundsSelection.load(xmlcontext->getContextFirst("bounds_select"));

	golem::XMLData(chainsFrameSize, xmlcontext->getContextFirst("chains_frame_size"), false);
	golem::XMLData(jointsFrameSize, xmlcontext->getContextFirst("joints_frame_size"), false);
}

void golem::Manipulator::Desc::load(const golem::XMLContext* context) {
	golem::XMLData(trajectoryErr, context->getContextFirst("trajectory"), false);
	golem::XMLData("timeout", trajectoryTimeout, context->getContextFirst("trajectory"), false);
	golem::XMLData("cluster_size", trajectoryClusterSize, context->getContextFirst("trajectory"), false);
	golem::XMLData("throw", trajectoryThrow, context->getContextFirst("trajectory"), false);
}

//------------------------------------------------------------------------------

template <> void golem::Stream::read(golem::Manipulator::Link& value) const {
	golem::U32 type = golem::Manipulator::Link::TYPE_JOINT, index = 0;
	read(type);
	read(index);
	value.set(static_cast<golem::Manipulator::Link::Type>(type), index);
}
template <> void golem::Stream::write(const golem::Manipulator::Link& value) {
	write(static_cast<golem::U32>(value.getType()));
	write(value.getIndex());
}

template <> void golem::Stream::read(golem::Manipulator::Link::Set& value) const {
	std::string string;
	read(string);
	const size_t size = std::min(string.length(), value.size());
	value.reset();
	for (size_t i = 0; i < size; ++i)
		if (string[i] != '0')
			value.set(i);
}
template <> void golem::Stream::write(const golem::Manipulator::Link::Set& value) {
	write(value.to_string('0'));
}

//------------------------------------------------------------------------------
