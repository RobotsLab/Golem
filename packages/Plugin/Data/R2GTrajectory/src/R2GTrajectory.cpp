/** @file PosePlanner.cpp
 * 
 * @author	Claudio Zito
 *
 * @version 1.0
 *
 */

#include <Golem/Data/R2GTrajectory/R2GTrajectory.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Planner/Data.h>
#include <Golem/Math/Data.h>
#include <Golem/App/Data.h>
#include <Golem/Tools/Import.h>
#include <Golem/Plugin/Ctrl.h>
#include <Golem/Tools/Menu.h>
#include <boost/algorithm/string.hpp>
#include <GL/glut.h>

//-----------------------------------------------------------------------------

using namespace golem;
using namespace golem::data;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new HandlerR2GTrajectory::Desc();
}

//------------------------------------------------------------------------------

void HandlerR2GTrajectory::ImportState::load(const XMLContext* xmlcontext) {
	XMLData("type", type, const_cast<XMLContext*>(xmlcontext), false);
	XMLData("inp", inpStr, const_cast<XMLContext*>(xmlcontext), false);
	XMLData("out", outStr, const_cast<XMLContext*>(xmlcontext), false);
	XMLData("offset", offset, const_cast<XMLContext*>(xmlcontext), false);
	XMLData("scale", scale, const_cast<XMLContext*>(xmlcontext), false);
}

void HandlerR2GTrajectory::ImportState::update(const RealSeq& data, Controller::State& state) const {
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

void HandlerR2GTrajectory::ImportState::update(const Map& map, const RealSeq& data, Controller::State& state) {
	for (HandlerR2GTrajectory::ImportState::Map::const_iterator i = map.begin(); i != map.end(); ++i)
		i->update(data, state);
}

void HandlerR2GTrajectory::ImportState::extract(const std::string& str, U32Seq& seq) {
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
		seq.push_back(U32(-1));
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
		throw Message(Message::LEVEL_CRIT, "HandlerR2GTrajectory::ImportState::extract(): invalid pointer range formatting");
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

void HandlerR2GTrajectory::ImportState::extract(Map& map) const {
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
		map.push_back(ImportState(type, *ip < U32(-1) ? *ip : *op, *op < U32(-1) ? *op : *ip, offset, scale));
}

void HandlerR2GTrajectory::ImportState::extract(const Map& inp, Map& out) {
	Map buf;
	for (Map::const_iterator i = inp.begin(); i != inp.end(); ++i) {
		buf.clear();
		i->extract(buf);
		out.insert(out.end(), buf.begin(), buf.end());
	}
	//for (auto &i : out)
	//	printf("%i: %i->%i, off=%f, s=%f\n", (int)i.type, i.inp, i.out, i.offset, i.scale);
}

void HandlerR2GTrajectory::ImportFrame::load(const XMLContext* xmlcontext) {
	XMLData("type", type, const_cast<XMLContext*>(xmlcontext), false);
	XMLData("lin", lin, const_cast<XMLContext*>(xmlcontext), false);
	XMLData("ang", ang, const_cast<XMLContext*>(xmlcontext), false);
}

void HandlerR2GTrajectory::ImportFrame::update(const RealSeq& data, Mat34& trn) const {
	if (size_t(lin + 3) >= data.size())
		throw Message(Message::LEVEL_CRIT, "HandlerR2GTrajectory::ImportFrame::update(): linear pointer %lu not in range <0, %lu)", lin + 3, data.size());
	const size_t size = type == TYPE_QUAT ? 4 : 3;
	if (size_t(ang + size) >= data.size())
		throw Message(Message::LEVEL_CRIT, "HandlerR2GTrajectory::ImportFrame::update(): angular pointer %lu not in range <0, %lu)", ang + size, data.size());
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

ItemR2GTrajectory::ItemR2GTrajectory(HandlerR2GTrajectory& handler) : Item(handler), handler(handler), waypointFile(handler.file), pathPosition(REAL_ZERO), pathWaypoint(0), pathInterpol(REAL_ZERO), contactPathWaypoint(-1), contactPathInterpol(REAL_ONE) {
	pregraspIdx = 0;
}

Item::Ptr ItemR2GTrajectory::clone() const {
	throw Message(Message::LEVEL_ERROR, "ItemR2GTrajectory::clone(): not implemented");
}

void ItemR2GTrajectory::createRender() {
	handler.createRender(*this);
}

void ItemR2GTrajectory::load(const std::string& prefix, const XMLContext* xmlcontext) {
	// waypoint
	std::string waypointSuffix;
	golem::XMLData("waypoint", waypointSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (waypointSuffix.length() > 0) {
		waypointFile.load(prefix + waypointSuffix, [&](const std::string& path) {
			if (!handler.controller)
				throw Message(Message::LEVEL_ERROR, "ItemR2GTrajectory::load(): controller is not set");

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
}

void ItemR2GTrajectory::save(const std::string& prefix, XMLContext* xmlcontext) const {
	// waypoint xml
	std::string waypointSuffix = !waypoints.empty() ? handler.waypointSuffix : "";
	XMLData("waypoint", waypointSuffix, xmlcontext, true);

	// waypoint binary
	if (waypointSuffix.length() > 0) {
		waypointFile.save(prefix + waypointSuffix, [=](const std::string& path) {
			FileWriteStream(path.c_str()).write(waypoints.begin(), waypoints.end());
		});
	}
	else
		waypointFile.remove();

	// path settings
	XMLData("path_position", const_cast<Real&>(pathPosition), xmlcontext, true);
	XMLData("path_waypoint", const_cast<size_t&>(contactPathWaypoint), xmlcontext->getContextFirst("contact", true), true);
	XMLData("path_interpol", const_cast<Real&>(contactPathInterpol), xmlcontext->getContextFirst("contact", true), true);
}

void ItemR2GTrajectory::setWaypoints(const WaypointCtrl::Seq& waypoints) {
	this->waypoints = waypoints;
	waypointFile.setModified(true);
}

static const char* TypeName[] = {
	"None",
	"Approach",
	"Action",
};

void ItemR2GTrajectory::setWaypoints(const WaypointCtrl::Seq& waypoints, const R2GTrajectory::Type type) {
	const WaypointMap::value_type val(type, waypoints);
	waypointMap.insert(val);
}

const golem::WaypointCtrl::Seq& ItemR2GTrajectory::getWaypoints() const {
	return waypoints;
}

const golem::WaypointCtrl::Seq& ItemR2GTrajectory::getWaypoints(const R2GTrajectory::Type type) const {
	WaypointMap::const_iterator i = waypointMap.find(type);
	if (i == waypointMap.end())
		throw Message(Message::LEVEL_ERROR, "Error: no waypoint of type %s", TypeName[type]);
	return i->second;
}

void ItemR2GTrajectory::createTrajectory(Controller::State::Seq& trajectory) {
	handler.createTrajectory(*this, trajectory);
}

void ItemR2GTrajectory::createActionTrajectory(const Controller::State& begin, Controller::State::Seq& trajectory) {
	handler.createActionTrajectory(*this, begin, trajectory);
}

void ItemR2GTrajectory::createTrajectory(const Controller::State& begin, Controller::State::Seq& trajectory) {
	handler.createTrajectory(*this, begin, trajectory);
}

void ItemR2GTrajectory::createIGTrajectory(const Controller::State& begin, Controller::State::Seq& trajectory) {
	handler.createIGTrajectory(*this, begin, trajectory);
}

//------------------------------------------------------------------------------

void HandlerR2GTrajectory::FactorDesc::load(Context& context, const XMLContext* xmlcontext) {
	XMLData("arm", arm, const_cast<XMLContext*>(xmlcontext), false);
	XMLData("hand", hand, const_cast<XMLContext*>(xmlcontext), false);
	XMLData("other", other, const_cast<XMLContext*>(xmlcontext), false);
}

void HandlerR2GTrajectory::ImportRobotTrjDesc::load(const XMLContext* xmlcontext) {
	XMLData("interval", interval, const_cast<XMLContext*>(xmlcontext), false);
	XMLData("begin", begin, const_cast<XMLContext*>(xmlcontext), false);
	XMLData("end", end, const_cast<XMLContext*>(xmlcontext), false);
	XMLData("subsampling", subsampling, const_cast<XMLContext*>(xmlcontext), false);
	stateMap.clear();
	XMLData(stateMap, stateMap.max_size(), const_cast<XMLContext*>(xmlcontext), "state");
	commandMap.clear();
	XMLData(commandMap, commandMap.max_size(), const_cast<XMLContext*>(xmlcontext), "command");
}

void HandlerR2GTrajectory::Desc::load(Context& context, const XMLContext* xmlcontext) {
	data::Handler::Desc::load(context, xmlcontext);

	XMLContext* pxmlcontext = xmlcontext->getContextFirst("trajectory", false);

	XMLData("waypoint_suffix", waypointSuffix, pxmlcontext, false);

	XMLData(*profileDesc, pxmlcontext->getContextFirst("profile"), false);

	XMLDataSeq(distance, "c", pxmlcontext->getContextFirst("profile distance"), false, REAL_ZERO);
	XMLDataSeq(extrapolation, "c", pxmlcontext->getContextFirst("profile extrapolation"), false, REAL_ZERO);
	XMLDataSeq(command, "c", pxmlcontext->getContextFirst("profile command"), false, REAL_ZERO);

	velFac.load(context, pxmlcontext->getContextFirst("profile velocity"));
	accFac.load(context, pxmlcontext->getContextFirst("profile acceleration"));
	disFac.load(context, pxmlcontext->getContextFirst("profile distance"));
	extFac.load(context, pxmlcontext->getContextFirst("profile extrapolation"));
	cmdFac.load(context, pxmlcontext->getContextFirst("profile command"));

	XMLData("extrapolation", trjExtrapolation, pxmlcontext->getContextFirst("profile"), false);
	XMLData("duration", trjDuration, pxmlcontext->getContextFirst("profile"), false);
	try {
		XMLData("r2g_duration", trjR2GDuration, pxmlcontext->getContextFirst("profile"), false);
	}
	catch (const MsgXMLParser&) {}
	XMLData("idle", trjIdle, pxmlcontext->getContextFirst("profile"), false);

	action.clear();
	try {
		XMLData(action, action.max_size(), const_cast<XMLContext*>(xmlcontext), "action");
	}
	catch (const MsgXMLParser&) {}

	XMLData(boundsSolidColour, pxmlcontext->getContextFirst("appearance bounds solid_colour"), false);
	XMLData(pathRenderer, pxmlcontext->getContextFirst("appearance path"), false);
	XMLData("inc_large", pathIncLarge, pxmlcontext->getContextFirst("appearance path"), false);
	XMLData("inc_small", pathIncSmall, pxmlcontext->getContextFirst("appearance path"), false);

	importRobotTrj.load(pxmlcontext->getContextFirst("import robot_trj"));
	XMLData("file_ext", importHDF5FileExt, pxmlcontext->getContextFirst("import hdf5", false), false);
	XMLData("robot_trj", importHDF5RobotTrj, pxmlcontext->getContextFirst("import hdf5", false), false);
}

data::Handler::Ptr HandlerR2GTrajectory::Desc::create(Context &context) const {
	data::Handler::Ptr handler(new HandlerR2GTrajectory(context));
	to<HandlerR2GTrajectory>(handler.get())->create(*this);
	return handler;
}

//------------------------------------------------------------------------------

HandlerR2GTrajectory::HandlerR2GTrajectory(Context &context) : 
	data::Handler(context), planner(nullptr), controller(nullptr), 
	arm(nullptr), hand(nullptr), pathPositionInc(REAL_ZERO), 
	boundsShow(false), contactPathRequest(false) {
}

void HandlerR2GTrajectory::create(const Desc& desc) {
	data::Handler::create(desc);

	waypointSuffix = desc.waypointSuffix;

	plannerIndex = desc.plannerIndex;

	profileDesc = desc.profileDesc; // shallow copy
	profileDesc->pCallbackDist = this;

	desc.profileDesc->velocity.resize(Configspace::DIM, REAL_ONE);
	velocityFac.set(desc.profileDesc->velocity.begin(), desc.profileDesc->velocity.end());

	desc.profileDesc->acceleration.resize(Configspace::DIM, REAL_ONE);
	accelerationFac.set(desc.profileDesc->acceleration.begin(), desc.profileDesc->acceleration.end());

	distance.fill(REAL_ONE);
	distance.set(desc.distance.data(), desc.distance.data() + std::min(desc.distance.size(), (size_t)Configspace::DIM));
	distanceFac = distance; // backup

	extrapolation.fill(REAL_ZERO);
	extrapolation.set(desc.extrapolation.data(), desc.extrapolation.data() + std::min(desc.extrapolation.size(), (size_t)Configspace::DIM));
	extrapolationFac = extrapolation; // backup

	command.fill(REAL_ZERO);
	command.set(desc.command.data(), desc.command.data() + std::min(desc.command.size(), (size_t)Configspace::DIM));
	commandFac = command; // backup

	// multipliers
	velFac = desc.velFac;
	accFac = desc.accFac;
	disFac = desc.disFac;
	extFac = desc.extFac;
	cmdFac = desc.cmdFac;

	trjExtrapolation = desc.trjExtrapolation;
	trjDuration = desc.trjDuration;
	trjR2GDuration = desc.trjR2GDuration;

	trjIdle = desc.trjIdle;

	action = desc.action;

	boundsSolidColour = desc.boundsSolidColour;
	pathRenderer = desc.pathRenderer;
	pathIncLarge = desc.pathIncLarge;
	pathIncSmall = desc.pathIncSmall;

	showCommands = desc.showCommands;

	importRobotTrj = desc.importRobotTrj;
	importRobotTrj.stateMap.clear();
	importRobotTrj.commandMap.clear();
	ImportState::extract(desc.importRobotTrj.stateMap, importRobotTrj.stateMap);
	ImportState::extract(desc.importRobotTrj.commandMap, importRobotTrj.commandMap);
	importRobotTrj.assertValid(Assert::Context("HandlerR2GTrajectory::create(): importRobotTrj."));

	importHDF5FileExt = desc.importHDF5FileExt;
	importHDF5RobotTrj = desc.importHDF5RobotTrj;

	importTypes = {
		importHDF5FileExt,
	};
	transformInterfaces = {
		"Belief",
		"Trajectory",
	};

	handPregraspPose = desc.handPregraspPose;
	handGraspPose = desc.handGraspPose;
}

//------------------------------------------------------------------------------

std::string HandlerR2GTrajectory::getFileExtWaypoint() {
	return std::string(".wp");
}

Item::Ptr HandlerR2GTrajectory::create() const {
	return Item::Ptr(new ItemR2GTrajectory(*const_cast<HandlerR2GTrajectory*>(this))); // safe: no modification here - only passing reference
}

//------------------------------------------------------------------------------

Item::Ptr HandlerR2GTrajectory::transform(const Item::List& input) {
	Item::Ptr item(create());
	ItemR2GTrajectory* itemR2GTrajectory = to<ItemR2GTrajectory>(item.get());

	// block item rendering (hasRenderBlock()=true)
	RenderBlock renderBlock(*this);

	const BeliefState* bstate = nullptr;
	const Trajectory* trajectory = nullptr;
	// collect data
	for (Item::List::const_iterator i = input.begin(); i != input.end(); ++i) {
		const BeliefState* ptr = is<const BeliefState>(*i);
		if (!ptr)
			trajectory = is<const Trajectory>(*i);
		else
			bstate = ptr;
	}
	if (!bstate)
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::transform(): no belief state specified");
	if (!trajectory)
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::transform(): no grasping trajectory specified");

	// arm chain and joints pointers
	const Chainspace::Index armChain = armInfo.getChains().begin();
	const Configspace::Range armJoints = armInfo.getJoints();

	WaypointCtrl::Seq waypoints = trajectory->getWaypoints(), seq;
	const Mat34 modelFrame = bstate->getModelFrame();
	const Mat34 queryTransform = bstate->getQueryTransform();
	for (WaypointCtrl::Seq::const_iterator i = waypoints.begin(); i != waypoints.end(); ++i) {
		// Compute a sequence of targets corresponding to the transformed arm end-effector
		GenWorkspaceChainState gwcs;
		controller->chainForwardTransform(i->command.cpos, gwcs.wpos);
		gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1

		// compute transformation over the query mean pose
		Mat34 poseFrameInv, graspFrame, graspFrameInv;
		poseFrameInv.setInverse(gwcs.wpos[armChain]);
		graspFrame.multiply(poseFrameInv, modelFrame);
		graspFrameInv.setInverse(graspFrame);
		gwcs.wpos[armChain].multiply(modelFrame, graspFrameInv);
		gwcs.wpos[armChain].multiply(queryTransform, gwcs.wpos[armChain]);
		gwcs.t = i->command.t;

		Controller::State cend = i->command;
		// Find target position
		if (!planner->findTarget(i->command, gwcs, cend))
			throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::transform(): Unable to find initial target configuration");
		// update configspace coords of the hand
		cend.cpos.set(handInfo.getJoints(), i->command.cpos);
		seq.push_back(WaypointCtrl(cend, cend));

		// update arm configurations and compute average error
		RBDist err;
		WorkspaceChainCoord wcc;
		controller->chainForwardTransform(cend.cpos, wcc);
		wcc[armChain].multiply(wcc[armChain], controller->getChains()[armChain]->getReferencePose());
		err.add(err, RBDist(RBCoord(wcc[armChain]), RBCoord(gwcs.wpos[armChain])));
		context.write("HandlerR2GTrajectory::transform(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
	}

	// set tranformed waypoint to the R2G trajectory
	itemR2GTrajectory->setWaypoints(seq);

	return item;
}

const StringSeq& HandlerR2GTrajectory::getTransformInterfaces() const {
	return transformInterfaces;
}

bool HandlerR2GTrajectory::isTransformSupported(const Item& item) const {
	return is<const BeliefState>(&item) || is<const Trajectory>(&item);
}

//------------------------------------------------------------------------------

U32 HandlerR2GTrajectory::getPlannerIndex() const {
	return plannerIndex;
}

void HandlerR2GTrajectory::set(Planner& planner, const ControllerId::Seq& controllerIDSeq) {
	this->planner = &planner;
	this->pHeuristic = dynamic_cast<golem::HBHeuristic*>(&planner.getHeuristic());
	if (!pHeuristic)
		throw Message(Message::LEVEL_CRIT, "HandlerR2GTrajectory(): Not IG heuristic is implemented.");

	this->controller = &planner.getController();

	if (controllerIDSeq.size() < 2)
		throw Message(Message::LEVEL_CRIT, "HandlerR2GTrajectory(): arm and hand are required");

	// joint and chain info
	info = controller->getStateInfo();
	armInfo = controllerIDSeq[0].findInfo(*const_cast<Controller*>(controller));
	handInfo = controllerIDSeq[1].findInfo(*const_cast<Controller*>(controller));
	defaultState.reset(new Controller::State(controller->createState()));
	controller->setToDefault(*defaultState);

	for (Configspace::Index i = info.getJoints().begin(); i < info.getJoints().end(); ++i) {
		const bool isArm = armInfo.getJoints().contains(i);
		const bool isHand = handInfo.getJoints().contains(i);
		const size_t j = i - info.getJoints().begin();

		// velocity
		profileDesc->velocity[j] = velocityFac[i] * (isArm ? velFac.arm : isHand ? velFac.hand : velFac.other);
		// accleleration
		profileDesc->acceleration[j] = accelerationFac[i] * (isArm ? accFac.arm : isHand ? accFac.hand : accFac.other);
		// distance
		distance[i] = distanceFac[i] * (isArm ? disFac.arm : isHand ? disFac.hand : disFac.other);
		// extrapolation
		extrapolation[i] = extrapolationFac[i] * (isArm ? extFac.arm : isHand ? extFac.hand : extFac.other);
		// command
		command[i] = commandFac[i] * (isArm ? cmdFac.arm : isHand ? cmdFac.hand : cmdFac.other);
	}

	pProfile = profileDesc->create(*controller); // throws

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

void HandlerR2GTrajectory::create(const ConfigspaceCoord& delta, Controller::State::Seq& trajectory) const {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::create(): At least two waypoints required");

	Controller::State state = trajectory.back();

	// last two waypoint used to extrapolate coordinates
	const Controller::State* s[2] = { &trajectory[trajectory.size() - 1], &trajectory[trajectory.size() - 2] };
	// linear extrapolation
	for (Configspace::Index i = state.getInfo().getJoints().begin(); i != state.getInfo().getJoints().end(); ++i)
		state.cpos[i] = Math::clamp(s[0]->cpos[i] + delta[i] * (s[0]->cpos[i] - s[1]->cpos[i]), controller->getMin().cpos[i], controller->getMax().cpos[i]);

	trajectory.push_back(state);
}

void HandlerR2GTrajectory::profile(SecTmReal duration, Controller::State::Seq& trajectory) const {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::profile(): At least two waypoints required");

	trajectory.front().t = SEC_TM_REAL_ZERO;
	trajectory.back().t = duration;
	pProfile->profile(trajectory);
}

void HandlerR2GTrajectory::createTrajectory(const ItemR2GTrajectory& item, Controller::State::Seq& trajectory) {
	if (!planner || !controller)
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): planner and controller are not set");
	if (item.getWaypoints().size() < 2)
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): At least two waypoints required");

	// disable IG for grasping
	this->pHeuristic->enableUnc = false;
	this->pHeuristic->setPointCloudCollision(true);

	Real trjExtrapolation = this->trjExtrapolation;
	Real trjDuration = this->trjDuration;
	bool inverse = false;
	bool createAction = !action.empty();

	if (getUICallback() && getUICallback()->hasInputEnabled()) {
		Menu menu(context, *getUICallback());
		//inverse = menu.option("FI", "Use (F)orward/(I)nverse trajectory... ") == 'I';
		menu.readNumber("Trajectory duration: ", trjDuration);
		menu.readNumber("Trajectory extrapolation: ", trjExtrapolation);
		if (createAction) createAction = menu.option("YN", "Create action (Y/N)... ") == 'Y';
	}

	// make trajectory
	Controller::State::Seq states = WaypointCtrl::make(item.getWaypoints(), false), commands = WaypointCtrl::make(item.getWaypoints(), true);

	// extrapolation delta
	ConfigspaceCoord delta;
	for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i)
		delta[i] = trjExtrapolation*extrapolation[i];

	// extrapolation
	const bool extrapol = trjExtrapolation > REAL_EPS;
	if (extrapol) {
		create(delta, states);
		create(delta, commands);
	}

	const size_t size = commands.size();
	{
		// waypoint pruning
		ScopeGuard guard([&]() {distRemovedCallback = nullptr; });
		distRemovedCallback = [&](size_t index) {
			//context.debug("HandlerTrajectory::import(): removing index #%u/%u\n", U32(index + 1), states.size());
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
	trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + this->trjIdle));

	// action
	if (createAction) {
		GenWorkspaceChainState gwcc;
		controller->chainForwardTransform(trajectory.back().cpos, gwcc.wpos);
		for (Chainspace::Index i = info.getChains().begin(); i < info.getChains().end(); ++i)
			gwcc.wpos[i].multiply(gwcc.wpos[i], controller->getChains()[i]->getReferencePose()); // reference pose
		gwcc.t = trajectory.back().t;

		GenWorkspaceChainState::Seq wAction;
		wAction.push_back(gwcc);
		for (Mat34Seq::const_iterator i = action.begin(); i != action.end(); ++i) {
			GenWorkspaceChainState gwccTrn = gwcc;
			gwccTrn.wpos[armInfo.getChains().begin()].multiply(*i, gwcc.wpos[armInfo.getChains().begin()]);
			gwccTrn.t += trjDuration;
			wAction.push_back(gwccTrn);
		}
		// find trajectory
		Controller::State::Seq cAction;
		if (!const_cast<Planner*>(planner)->findLocalTrajectory(trajectory.back(), wAction.begin(), wAction.end(), cAction, cAction.end()))
			throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): Unable to find action trajectory");
		// add
		trajectory.insert(trajectory.end(), ++cAction.begin(), cAction.end());
		trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + this->trjIdle));
	}
}

void HandlerR2GTrajectory::createActionTrajectory(const ItemR2GTrajectory& item, const Controller::State& begin, Controller::State::Seq& trajectory) {
	if (!planner || !controller)
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): planner and controller are not set");
	if (item.getWaypoints().size() < 2)
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): At least two waypoints required");

	// disable IG for grasping
	this->pHeuristic->enableUnc = false;
	this->pHeuristic->setPointCloudCollision(false);

	Real trjExtrapolation = this->trjExtrapolation;
	Real trjDuration = this->trjDuration;
	bool inverse = false;
	bool createAction = !action.empty();

	if (getUICallback() && getUICallback()->hasInputEnabled()) {
		Menu menu(context, *getUICallback());
		//inverse = menu.option("FI", "Use (F)orward/(I)nverse trajectory... ") == 'I';
		menu.readNumber("Trajectory duration: ", trjDuration);
		menu.readNumber("Trajectory extrapolation: ", trjExtrapolation);
		if (createAction) createAction = menu.option("YN", "Create action (Y/N)... ") == 'Y';
	}

	// make trajectory
	Controller::State::Seq states = WaypointCtrl::make(item.getWaypoints(), false), commands = WaypointCtrl::make(item.getWaypoints(), true);

	// extrapolation delta
	ConfigspaceCoord delta;
	for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i)
		delta[i] = trjExtrapolation*extrapolation[i];

	// extrapolation
	const bool extrapol = trjExtrapolation > REAL_EPS;
	if (extrapol) {
		create(delta, states);
		create(delta, commands);
	}

	// overwrites the arm position with the current pose of the robot
	for (auto &w : commands)
		w.cpos.set(armInfo.getJoints(), begin.cpos);
	for (auto &w : states)
		w.cpos.set(armInfo.getJoints(), begin.cpos);
	// overwrites the fingers pose only for the first pose of the grasp
	commands.front().cpos.set(handInfo.getJoints(), begin.cpos);

	const size_t size = commands.size();
	{
		// waypoint pruning
		ScopeGuard guard([&]() {distRemovedCallback = nullptr; });
		distRemovedCallback = [&](size_t index) {
			//context.debug("HandlerTrajectory::import(): removing index #%u/%u\n", U32(index + 1), states.size());
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
	trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + this->trjIdle));

	// action
	if (createAction) {
		GenWorkspaceChainState gwcc;
		controller->chainForwardTransform(trajectory.back().cpos, gwcc.wpos);
		for (Chainspace::Index i = info.getChains().begin(); i < info.getChains().end(); ++i)
			gwcc.wpos[i].multiply(gwcc.wpos[i], controller->getChains()[i]->getReferencePose()); // reference pose
		gwcc.t = trajectory.back().t;

		GenWorkspaceChainState::Seq wAction;
		wAction.push_back(gwcc);
		for (Mat34Seq::const_iterator i = action.begin(); i != action.end(); ++i) {
			GenWorkspaceChainState gwccTrn = gwcc;
			gwccTrn.wpos[armInfo.getChains().begin()].multiply(*i, gwcc.wpos[armInfo.getChains().begin()]);
			gwccTrn.t += trjDuration;
			wAction.push_back(gwccTrn);
		}
		// find trajectory
		Controller::State::Seq cAction;
		if (!const_cast<Planner*>(planner)->findLocalTrajectory(trajectory.back(), wAction.begin(), wAction.end(), cAction, cAction.end()))
			throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): Unable to find action trajectory");
		// add
		trajectory.insert(trajectory.end(), ++cAction.begin(), cAction.end());
		trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + this->trjIdle));
	}
}

void HandlerR2GTrajectory::createTrajectory(const ItemR2GTrajectory& item, const Controller::State& begin, Controller::State::Seq& trajectory) {
	if (!planner || !controller)
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): planner and controller are not set");
	if (item.getWaypoints().size() < 2)
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): At least two waypoints required");

	// Enable IG for reach-to-grasp trajectories and collisions with target object
	this->pHeuristic->enableUnc = false;
//	this->pHeuristic->setPointCloudCollision(true);

	Real trjDuration = this->trjR2GDuration;

	// make trajectory
	Controller::State::Seq states = WaypointCtrl::make(item.getWaypoints(), false), commands = WaypointCtrl::make(item.getWaypoints(), true);

	if (getUICallback() && getUICallback()->hasInputEnabled()) {
		Menu menu(context, *getUICallback());
		//inverse = menu.option("FI", "Use (F)orward/(I)nverse trajectory... ") == 'I';
		menu.readNumber("Trajectory duration: ", trjDuration);
	}

	 if(!planner->findGlobalTrajectory(begin, commands.front(), trajectory, trajectory.begin()))
		 throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): Unable to find reach-to-grasp trajectory");
	
	for (Controller::State::Seq::iterator i = trajectory.begin(); i != trajectory.end(); ++i)
			i->cpos.set(handInfo.getJoints(), commands.front().cpos);

	profile(trjDuration, trajectory);

	// debug
	context.debug("HandlerTrajectory::createTrajectory(): Trajectory size %u\n", (U32)trajectory.size());

	// stationary waypoint in the end
	trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + this->trjIdle));

	// disable IG for grasping and collisions with target object
	this->pHeuristic->enableUnc = false;
	this->pHeuristic->setPointCloudCollision(false);
}

void HandlerR2GTrajectory::createIGTrajectory(const ItemR2GTrajectory& item, const Controller::State& begin, Controller::State::Seq& trajectory) {
	if (!planner || !controller)
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): planner and controller are not set");
	if (item.getWaypoints().size() < 2)
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): At least two waypoints required");

	// Enable IG for reach-to-grasp trajectories and collisions with target object
	this->pHeuristic->enableUnc = true;
//	this->pHeuristic->setPointCloudCollision(true);

	Real trjDuration = this->trjR2GDuration;

	// make trajectory
	Controller::State::Seq states = WaypointCtrl::make(item.getWaypoints(), false), commands = WaypointCtrl::make(item.getWaypoints(), true);

	if (getUICallback() && getUICallback()->hasInputEnabled()) {
		Menu menu(context, *getUICallback());
		//inverse = menu.option("FI", "Use (F)orward/(I)nverse trajectory... ") == 'I';
		menu.readNumber("Trajectory duration: ", trjDuration);
	}

	if (!planner->findGlobalTrajectory(begin, commands.front(), trajectory, trajectory.begin()))
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): No reach-to-grasp trajectory found");

	for (Controller::State::Seq::iterator i = trajectory.begin(); i != trajectory.end(); ++i)
		i->cpos.set(handInfo.getJoints(), commands.front().cpos);

	profile(trjDuration, trajectory);

	// debug
	context.debug("HandlerTrajectory::createTrajectory(): Trajectory size %u\n", (U32)trajectory.size());

	// stationary waypoint in the end
	trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + this->trjIdle));

	// disable IG for grasping and collisions with target object
	this->pHeuristic->enableUnc = false;
	this->pHeuristic->setPointCloudCollision(false);
}

//------------------------------------------------------------------------------

Real HandlerR2GTrajectory::distConfigspaceCoord(const ConfigspaceCoord& prev, const ConfigspaceCoord& next) const {
	Real dist = REAL_ZERO;
	for (Configspace::Index i = info.getJoints().begin(); i < info.getJoints().end(); ++i)
		dist += distance[i] * Math::sqr(prev[i] - next[i]);
	return Math::sqrt(dist);
}

Real HandlerR2GTrajectory::distCoord(Real prev, Real next) const {
	return Math::abs(prev - next);
}

bool HandlerR2GTrajectory::distCoordEnabled(const Configspace::Index& index) const {
	return (arm->getStateInfo().getJoints().contains(index) || hand->getStateInfo().getJoints().contains(index));
}

bool HandlerR2GTrajectory::distCoordInterpolate(const Configspace::Index& index) const {
	return !arm->getStateInfo().getJoints().contains(index) && !hand->getStateInfo().getJoints().contains(index);
}

//------------------------------------------------------------------------------

void HandlerR2GTrajectory::distRemoved(size_t index) const {
	if (distRemovedCallback) distRemovedCallback(index);
}

//------------------------------------------------------------------------------

Item::Ptr HandlerR2GTrajectory::import(const std::string& path) {
	// check extension
	if (std::find(importTypes.begin(), importTypes.end(), getExt(path)) == importTypes.end())
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::import(): unknown file type %s", getExt(path).c_str());

	RenderBlock renderBlock(*this); // do not show cache

	Item::Ptr item(create());
	ItemR2GTrajectory* itemTrajectory = to<ItemR2GTrajectory>(item.get());
	Menu::Ptr menu(getUICallback() && getUICallback()->hasInputEnabled() ? new Menu(context, *getUICallback()) : nullptr);

	// HDF5 format
	if (path.rfind(importHDF5FileExt) != std::string::npos) {
		Controller::State state = controller->createState(), command = state;

		golem::Import::HDF5DumpRealSeqMap map;
		if (menu != nullptr) {
			menu->readString("Enter robot trajectory dataset name: ", importHDF5RobotTrj);
		}

		map.insert(std::make_pair(importHDF5RobotTrj, [&](const std::string& name, size_t index, const RealSeq& seq) {
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
			throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::import(): empty robot trajectory");
	}

	itemTrajectory->waypointFile.setModified(true);

	return item;
}

const StringSeq& HandlerR2GTrajectory::getImportFileTypes() const {
	return importTypes;
}

//------------------------------------------------------------------------------

void HandlerR2GTrajectory::convert(const golem::Configspace::Range& inpConfigspaceRange, const golem::Configspace::Range& outConfigspaceRange, const golem::Chainspace::Range& inpChainspaceRange, const golem::Chainspace::Range& outChainspaceRange, golem::ConfigspaceCoord& val) {
	context.info("HandlerR2GTrajectory::convert(Configspace): config_[%zd, %zd) -> config_[%zd, %zd), chain_[%zd, %zd) -> chain_[%zd, %zd)\n", *inpConfigspaceRange.begin(), *inpConfigspaceRange.end(), *outConfigspaceRange.begin(), *outConfigspaceRange.end(), *inpChainspaceRange.begin(), *inpChainspaceRange.end(), *outChainspaceRange.begin(), *outChainspaceRange.end());
	// set to default everything out of range
	const golem::ConfigspaceCoord coords = val;
	val = defaultState->cpos;
	val.set(outConfigspaceRange, coords);
}

void HandlerR2GTrajectory::convert(const golem::Reservedspace::Range& inpRange, const golem::Reservedspace::Range& outRange, golem::ReservedCoord& val) {
	context.info("HandlerR2GTrajectory::convert(Reservedspace): reserved_[%zd, %zd) -> reserved_[%zd, %zd)\n", *inpRange.begin(), *inpRange.end(), *outRange.begin(), *outRange.end());
	// set to default
	val = defaultState->reserved;
}


//------------------------------------------------------------------------------

void HandlerR2GTrajectory::createRender(const ItemR2GTrajectory& item) {
	UI::CriticalSectionWrapper cs(getUICallback());

	boundsShow = false;
	pathRenderer.reset();
	if (!controller || item.waypoints.empty())
		return;

	// commands/states
	const Controller::State::Seq seq = WaypointCtrl::make(item.waypoints, showCommands);

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
		const Real s = d > REAL_EPS ? (pos - distance[i - 1]) / d : REAL_ZERO;
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

void HandlerR2GTrajectory::render() const {
	if (hasRenderBlock()) return;

	// path
	pathRenderer.render();
	// bounds
	if (boundsShow) {
		boundsRenderer.setSolidColour(boundsSolidColour);
		boundsRenderer.renderSolid(boundsSeq.begin(), boundsSeq.end());
	}
}

void HandlerR2GTrajectory::customRender() const {
}

//------------------------------------------------------------------------------

void HandlerR2GTrajectory::mouseHandler(int button, int state, int x, int y) {
	if (hasRenderBlock()) return;

	if (state == 0) {
		pathPositionInc +=
			button == 3 ? +pathIncLarge : button == 19 ? +pathIncSmall :
			button == 4 ? -pathIncLarge : button == 20 ? -pathIncSmall :
			button == 1 ? -REAL_MAX : REAL_ZERO;
		requestRender();
	}
}

void HandlerR2GTrajectory::motionHandler(int x, int y) {
}

void HandlerR2GTrajectory::keyboardHandler(int key, int x, int y) {
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

void golem::XMLData(const std::string &attr, HandlerR2GTrajectory::ImportState::Type& val, XMLContext* xmlcontext, bool create) {
	std::string type;
	XMLData(attr, type, xmlcontext, create);
	transform(type.begin(), type.end(), type.begin(), tolower);
	val =
		type == "t" || type == "time" ? HandlerR2GTrajectory::ImportState::TYPE_TIME :
		type == "pos" || type == "position" ? HandlerR2GTrajectory::ImportState::TYPE_POSITION :
		type == "vel" || type == "velocity" ? HandlerR2GTrajectory::ImportState::TYPE_VELOCITY :
		type == "acc" || type == "acceleration" ? HandlerR2GTrajectory::ImportState::TYPE_VELOCITY :
		HandlerR2GTrajectory::ImportState::TYPE_RESERVED;
}

void golem::XMLData(const std::string &attr, HandlerR2GTrajectory::ImportFrame::Type& val, XMLContext* xmlcontext, bool create) {
	std::string type;
	XMLData(attr, type, xmlcontext, create);
	transform(type.begin(), type.end(), type.begin(), tolower);
	val =
		type == "euler" ? HandlerR2GTrajectory::ImportFrame::TYPE_EULER :
		type == "axis" ? HandlerR2GTrajectory::ImportFrame::TYPE_AXIS :
		HandlerR2GTrajectory::ImportFrame::TYPE_QUAT;
}

void golem::data::XMLData(HandlerR2GTrajectory::ImportState::Map::value_type& val, XMLContext* xmlcontext, bool create) {
	val.load(xmlcontext);
}

//------------------------------------------------------------------------------