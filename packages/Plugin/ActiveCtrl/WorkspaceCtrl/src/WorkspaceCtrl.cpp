/** @file WorkspaceCtrl.cpp
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/ActiveCtrl/WorkspaceCtrl/WorkspaceCtrl.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <Golem/App/Data.h>
#include <GL/glut.h>
#include <iomanip>

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new WorkspaceCtrl::Desc();
}

//------------------------------------------------------------------------------

const std::string WorkspaceCtrl::ModeName[MODE_LAST + 1] = {
	"Disabled",
	"Position",
	"Orientation",
	"Position X",
	"Position Y",
	"Position Z",
	"Orientation X",
	"Orientation Y",
	"Orientation Z",
};

void golem::XMLData(golem::WorkspaceCtrl::CoordDesc::Map::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("out", const_cast<U32&>(val.first), xmlcontext, create);

	golem::XMLData("inp", val.second.inp, xmlcontext, create);

	golem::XMLData("gain", val.second.gain, xmlcontext, create);
	try {
		golem::XMLData("offset", val.second.offset, xmlcontext, create);
	}
	catch (const golem::MsgXMLParser&) {}
}

void WorkspaceCtrl::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	ActiveCtrl::Desc::load(context, xmlcontext);

	golem::XMLData("ctrl_thread_sleep", ctrlThreadSleep, const_cast<golem::XMLContext*>(xmlcontext), false);

	golem::XMLData(frameIncrement, xmlcontext->getContextFirst("frame increment"), false);
	golem::XMLData("time", timeIncrement, xmlcontext->getContextFirst("frame increment"), false);
	golem::XMLData("low", incrementLow, xmlcontext->getContextFirst("frame increment"), false);

	golem::XMLData("global", modeGlobal, xmlcontext->getContextFirst("frame"), false);

	golem::XMLData("pred_time", ctrlSimPred, xmlcontext->getContextFirst("sim"), false);
	golem::XMLData("reac_time", ctrlSimReac, xmlcontext->getContextFirst("sim"), false);
	golem::XMLData(ctrlSimGain, xmlcontext->getContextFirst("sim gain"), false);

	golem::XMLData(frameSize, xmlcontext->getContextFirst("frame size"), false);

	golem::XMLData("lin_keys", linKeys, xmlcontext->getContextFirst("frame"));
	golem::XMLData("ang_keys", angKeys, xmlcontext->getContextFirst("frame"));
	golem::XMLData("wpt_keys", wptKeys, xmlcontext->getContextFirst("frame"));

	try {
		golem::XMLData("cache_file", file, const_cast<golem::XMLContext*>(xmlcontext), false);
	}
	catch (const golem::MsgXMLParserAttributeNotFound&) {
	}

	golem::XMLData("mode_simple", modeSimple, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("mode_map", modeMap, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("increment", trjIncrement, xmlcontext->getContextFirst("trajectory"), false);
	golem::XMLData(trjRendererChains, xmlcontext->getContextFirst("trajectory appearance chains"), false);
	golem::XMLData(trjRendererMean, xmlcontext->getContextFirst("trajectory appearance mean"), false);
	golem::XMLData(trjRendererDev, xmlcontext->getContextFirst("trajectory appearance dev"), false);

	golem::XMLData("mean", trjShowMean, xmlcontext->getContextFirst("trajectory appearance show"), false);
	golem::XMLData("mean_frame", trjShowMeanFrame, xmlcontext->getContextFirst("trajectory appearance show"), false);
	golem::XMLData("dev", trjShowDev, xmlcontext->getContextFirst("trajectory appearance show"), false);
	golem::XMLData("dev_frame", trjShowDevFrame, xmlcontext->getContextFirst("trajectory appearance show"), false);
	golem::XMLData("target", trjShowTar, xmlcontext->getContextFirst("trajectory appearance show"), false);
	golem::XMLData("current", trjShowCurr, xmlcontext->getContextFirst("trajectory appearance show"), false);
	golem::XMLData("global_frame", trjShowGlobalFrame, xmlcontext->getContextFirst("trajectory appearance show"), false);
	golem::XMLData("global_inc_frame", trjShowGlobalIncFrame, xmlcontext->getContextFirst("trajectory appearance show"), false);

	manipulatorDesc->load(xmlcontext->getContextFirst("manipulator"));
	appearanceMean.load(xmlcontext->getContextFirst("manipulator appearance mean"));
	appearanceDev.load(xmlcontext->getContextFirst("manipulator appearance dev"));

	appearanceManifold.load(xmlcontext->getContextFirst("manifold appearance"));

	try {
		golem::XMLData("enabled", handDirCtrl, xmlcontext->getContextFirst("hand_dir_ctrl"), false);
		golem::XMLDataSeq(handDirCtrlOpen, "c", xmlcontext->getContextFirst("hand_dir_ctrl open"), false, golem::REAL_ZERO);
		golem::XMLDataSeq(handDirCtrlClosed, "c", xmlcontext->getContextFirst("hand_dir_ctrl closed"), false, golem::REAL_ZERO);
	}
	catch (const golem::MsgXMLParser&) {
		handDirCtrl = false;
	}

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML
	try {
		inputCtrlDesc.load(context, xmlcontext);
	}
	catch (const golem::MsgXMLParser&) {
		//context.write(msg);
	}
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
	try {
		virtuose6DDesc = Controller::Desc::load(&context, xmlcontext->getContextFirst("virtuose6d"));
		
		golem::XMLData(poseGainIncrement, xmlcontext->getContextFirst("virtuose6d pose_gain_increment"), false);
		golem::XMLData(forceGainManifold, xmlcontext->getContextFirst("virtuose6d force_gain_manifold"), false);
		golem::XMLData(forceGainIncrement, xmlcontext->getContextFirst("virtuose6d force_gain_increment"), false);

		trjCoordDescMap.clear();
		golem::XMLData(trjCoordDescMap, trjCoordDescMap.max_size(), xmlcontext->getContextFirst("virtuose6d trj_coord"), "item");
		poseCoordDescMap.clear();
		golem::XMLData(poseCoordDescMap, poseCoordDescMap.max_size(), xmlcontext->getContextFirst("virtuose6d pose_coord"), "item");
	}
	catch (const golem::MsgXMLParser&) {
		if (virtuose6DDesc != nullptr)
			throw;
	}
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
}

//------------------------------------------------------------------------------

WorkspaceCtrl::WorkspaceCtrl(golem::Planner &planner, const Sensor::Seq& sensorSeq) : ActiveCtrl(planner, sensorSeq) {
#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
	virtuose6D = nullptr;
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
}

void WorkspaceCtrl::create(const Desc& desc) {
	ActiveCtrl::create(desc); // throws

	// controllers
	if (controllerIDSeq.size() < 1)
		throw Message(Message::LEVEL_CRIT, "WorkspaceCtrl::create(): At least one controller required!");

	// default values
	ctrlInfo = controllerIDSeq[0].findInfo(controller);
	name = controller.getName().c_str();
	commandLatency = controller.getCommandLatency() + controller.getCycleDuration();

	try {
		// available controller
		golem::Controller* ctrl = controllerIDSeq[0].findController(const_cast<golem::Controller&>(controller));
		name = ctrl->getName().c_str();
		commandLatency = ctrl->getCommandLatency() + ctrl->getCycleDuration();
	}
	catch (const Message& msg) {
		context.notice("WorkspaceCtrl::create(): Direct controller access not available (%s)\n", msg.msg());
	}

	frameIncrement = desc.frameIncrement;
	timeIncrement = desc.timeIncrement;

	ctrlSimGain = desc.ctrlSimGain;
	ctrlSimReac = desc.ctrlSimReac;
	ctrlSimPred = desc.ctrlSimPred;
	ctrlSimModeTrn.setZero();
	ctrlSimModeScr.setZero();
	ctrlSimMode = false;

	ctrlUpdateArm = false;
	ctrlUpdateHand = false;

	frameSize = desc.frameSize;

	linKeys = desc.linKeys;
	angKeys = desc.angKeys;
	wptKeys = desc.wptKeys;

	file = desc.file;

	coordGlobal.setToDefault(ctrlInfo.getJoints());
	frameGlobal.setId();
	frameLocal.setId();
	frameTarget.setId();
	incrementLow = desc.incrementLow;
	incrementStep = 0;

	// load cache
	load();

	modeMap = false;
	modeSimple = false;
	modeForce = false;
	modeWaypoint = false;
	modeTrj = false;

	waypointIndex = modeSimple ? (U32)waypoints.size() : 0;

	handDirCtrlOpenCmd = false;
	handDirCtrlClosedCmd = false;
	handDirCtrlAvailable = desc.handDirCtrl && controllerIDSeq.size() >= 2;
	if (handDirCtrlAvailable) {
		try {
			// available controller
			golem::Controller* hand = controllerIDSeq[1].findController(const_cast<golem::Controller&>(controller));
			handInfo = hand->getStateInfo();
			for (golem::idx_t i = 0; i < handInfo.getJoints().size(); ++i) {
				const Configspace::Index j = handInfo.getJoints().begin() + i;
				handDirCtrlOpen[j] = Math::clamp((size_t)i < desc.handDirCtrlOpen.size() ? desc.handDirCtrlOpen[(size_t)i] : desc.handDirCtrlOpen.back(), hand->getMin().cpos[j], hand->getMax().cpos[j]);
				handDirCtrlClosed[j] = Math::clamp((size_t)i < desc.handDirCtrlClosed.size() ? desc.handDirCtrlClosed[(size_t)i] : desc.handDirCtrlClosed.back(), hand->getMin().cpos[j], hand->getMax().cpos[j]);
			}
		}
		catch (const Message& msg) {
			context.notice("WorkspaceCtrl::create(): Direct controller access not available (%s)\n", msg.msg());
			handDirCtrlAvailable = false;
		}
	}

	trjUpdateCalback = nullptr;
	trjScaleCalback = nullptr;
	trjTime = SEC_TM_REAL_ZERO;
	trjTimeDelta = SEC_TM_REAL_ZERO;
	trjTimeSent = SEC_TM_REAL_ZERO;

	trjIncrement = desc.trjIncrement;
	trjRendererChains = desc.trjRendererChains;
	trjRendererMean = desc.trjRendererMean;
	trjRendererDev = desc.trjRendererDev;

	trjShowMean = desc.trjShowMean;
	trjShowMeanFrame = desc.trjShowMeanFrame;
	trjShowDev = desc.trjShowDev;
	trjShowDevFrame = desc.trjShowDevFrame;
	trjShowTar = desc.trjShowTar;
	trjShowCurr = desc.trjShowCurr;
	trjShowGlobalFrame = desc.trjShowGlobalFrame;
	trjShowGlobalIncFrame = desc.trjShowGlobalIncFrame;

	manipulator = desc.manipulatorDesc->create(getPlanner(), getControllerIDSeq());
	appearanceMean = desc.appearanceMean;
	appearanceDev = desc.appearanceDev;

	appearanceManifold = desc.appearanceManifold;

	ctrlThreadSleep = desc.ctrlThreadSleep;

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML
	try {
		inputCtrlPtr = desc.inputCtrlDesc.create(context);
		modeSimple = desc.modeSimple;

		// number of controllable DOFs
		U32Set ctrlSet;
		for (auto& i : inputCtrlPtr->getJoystickDescSeq())
			for (auto& j : i.axisDescMap)
				ctrlSet.insert(j.second.ctrl);
		modeMap = desc.modeMap && ctrlSet.size() >= ManifoldCtrl::SIZE;
	}
	catch (const Message& msg) {
		context.notice("WorkspaceCtrl::create(): Joystick input device not available (%s)\n", msg.msg());
	}
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
	stiffnessDflt.setZero();
	dampingDflt.setZero();
	poseGainIncrement = desc.poseGainIncrement;
	forceGainManifold = desc.forceGainManifold;
	forceGainIncrement = desc.forceGainIncrement;
	trjCoordDescMap = desc.trjCoordDescMap;
	poseCoordDescMap = desc.poseCoordDescMap;
	modeSimulationVirt = false;
	try {
		if (desc.virtuose6DDesc == nullptr)
			throw Message(Message::LEVEL_ERROR, "WorkspaceCtrl::create(): Virtuose6D haption device description not provided");
		controllerPtr = desc.virtuose6DDesc->create(context);
		virtuose6D = is<Virtuose6D>(controllerPtr.get());
		if (virtuose6D == nullptr)
			throw Message(Message::LEVEL_ERROR, "WorkspaceCtrl::create(): unable to cast Virtuose6D pointer");

		this->stiffnessDflt = virtuose6D->getStiffnessDflt();
		this->dampingDflt = virtuose6D->getDampingDflt();

		Controller::State command = virtuose6D->createState();
		virtuose6D->lookupCommand(SEC_TM_REAL_MAX, command);
		this->referencePoseMaster = virtuose6D->getCartImpPose(command);
		this->referencePoseSlave = getCommand(getCommandTime()).cpos;
		this->referenceTime = SEC_TM_REAL_ZERO;
		
		modeSimulationVirt = virtuose6D->isSimulation();
		modeSimple = desc.modeSimple && !modeSimulationVirt;//true
		modeMap = desc.modeMap && !modeSimulationVirt;
		modeForce = true;

		if (modeMap) {
			virtuose6D->getCartImpStiffness(command).setZero();
			virtuose6D->getCartImpDamping(command).setZero();
		}
		
		virtuose6D->send(&command, &command + 1, true);
	}
	catch (const Message& msg) {
		if (virtuose6D) {
			virtuose6D = nullptr;
			throw;
		}
		else
			context.notice("WorkspaceCtrl::create(): Virtuose6D haption device not available (%s)\n", msg.msg());
	}
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D

	modeGlobal = modeMap || desc.modeGlobal;
	mode = modeSimple ? MODE_POSITION : MODE_DISABLED;

	// control task
	task.reset(new ThreadTask([&]() {
#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML
		typedef std::map<U32, bool> ActiveMap;
		ActiveMap activeMap;
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML
#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
		bool virtStateUpdate = false;
		Virtuose6D::VirtuoseState virtState;
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
		SecTmReal referenceTimeDelta = SEC_TM_REAL_ZERO;

		for (; !task->isTerminating();) {
			try {
				if (!isActive())
					continue;

				// current command and frame
				const SecTmReal t = context.getTimer().elapsed();
				const SecTmReal tSleep = golem::MSecToSec(ctrlThreadSleep);
				Controller::State cbegin = getCommand(t + commandLatency + ctrlSimReac);
				Controller::Trajectory trj;

				U32 mode = MODE_DISABLED;
				bool modeGlobal = false, modeTrj = false;
				bool ctrlUpdateArm = false, ctrlUpdateHand = false;
				Twist ctrlSimModeTrn = Twist::zero();
				SecTmReal ctrlTime = SEC_TM_REAL_ZERO;

				for (; !ctrlUpdateArm && !ctrlUpdateHand && t + tSleep > context.getTimer().elapsed();) {
					Sleep::msleep(1);

					UI::CriticalSectionWrapper csw(getUICallback());

					mode = this->mode;
					modeGlobal = this->modeGlobal;
					modeTrj = isTrjMode();

					ctrlUpdateArm = this->ctrlUpdateArm;
					this->ctrlUpdateArm = false;
					ctrlUpdateHand = this->ctrlUpdateHand;
					this->ctrlUpdateHand = false;

					ctrlSimModeTrn = this->ctrlSimModeTrn;
					ctrlTime = this->trjTimeDelta;
				}

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML
				if (inputCtrlPtr != nullptr && (mode == MODE_POSITION || mode == MODE_ORIENTATION)) {
					inputCtrlPtr->update();
					inputCtrlPtr->getJoystickButtonState([&](U32 index, U32 ctrl, bool value) {
						if (ctrl == 0)
							activeMap[index] = value;
						if (ctrl == 1 && value)
							context.notice("WorkspaceCtrl(): Joystick: robot MOVE\n");
						if (ctrl == 2 && value)
							context.notice("WorkspaceCtrl(): Joystick: robot STOP\n");
						if (ctrl == 3 && value) {
							handDirCtrlOpenCmd = false;
							handDirCtrlClosedCmd = true;
							ctrlUpdateHand = true;
							context.notice("WorkspaceCtrl(): Joystick: gripper CLOSE\n");
						}
						if (ctrl == 4 && value) {
							handDirCtrlOpenCmd = true;
							handDirCtrlClosedCmd = false;
							ctrlUpdateHand = true;
							context.notice("WorkspaceCtrl(): Joystick: gripper OPEN\n");
						}
					});
					inputCtrlPtr->getJoystickAxisPosition([&](U32 index, U32 ctrl, Real value) {
						if (activeMap[index] && Math::abs(value) > REAL_ZERO && ctrl < U32(RBCoord::N)) {
							//const RBDist frameInc = getFrameInc();
							if (modeTrj) {
								if (ctrl < this->trjContactManifoldMap.size())
									ctrlSimModeTrn[trjContactManifoldMap[ctrl].second] = trjContactManifoldMap[ctrl].first * value;
							}
							else {
								ctrlSimModeTrn[ctrl] = value;
								//ctrlSimModeTrn[ctrl] = (ctrl < 2 ? frameInc.lin : frameInc.ang) * value;
							}
							ctrlUpdateArm = true;
						}
					});
				}
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
				if (virtuose6D != nullptr && (mode == MODE_POSITION || mode == MODE_ORIENTATION)) {
					Controller::State state = virtuose6D->createState();
					virtuose6D->lookupState(SEC_TM_REAL_MAX, state);
					const RBCoord pose = virtuose6D->getCartImpPose(state);
					const Twist force = virtuose6D->getCartForceTcpFT(state);
					const Virtuose6D::VirtuoseState vs = virtuose6D->getVirtuoseState(state);
					
					// update reference poses and time
					auto updateReference = [&]() {
						if (!modeTrj) {
							this->referencePoseSlave = cbegin.cpos;
							this->referencePoseMaster = pose;
						}
						this->referenceTime = referenceTimeDelta;
						virtState = vs; //!
					};

					if (virtStateUpdate) {
						// reset reference pose
						if (!virtState.buttonForce.isDn() && vs.buttonForce.isDn()) {
							context.notice("WorkspaceCtrl(): Virtuose6D: reference pose UPDATE\n");

							Controller::State command = virtuose6D->createState();
							virtuose6D->lookupCommand(SEC_TM_REAL_MAX, command);
							virtuose6D->getCartImpPose(command) = pose;
							virtuose6D->send(&command, &command + 1, true);

							// update reference poses
							if (this->modeMap && modeTrj) {
								this->referencePoseSlave = cbegin.cpos;
								this->referencePoseMaster = pose;
								updateReference();
								continue;
							}
						}
						// move robotic arm
						if (vs.buttonLeft.isDn()) {
							if (!virtState.buttonLeft.isDn()) {
								context.notice("WorkspaceCtrl(): Virtuose6D: robot MOVE\n");

								// update reference poses
								if (this->modeMap) {
									updateReference();
									continue;
								}
							}

							// pose update
							if (!modeSimulationVirt)
								getUpdate(modeTrj, pose, cbegin.cpos, force, ctrlSimModeTrn, ctrlTime);

							ctrlUpdateArm = true;
						}
						else {
							if (virtState.buttonLeft.isDn()) {
								context.notice("WorkspaceCtrl(): Virtuose6D: robot STOP\n");
							}
						}
						// open/close gripper
						if (!virtState.buttonRight.isUp() && vs.buttonRight.isUp()) {
							if (handDirCtrlOpenCmd) {
								context.notice("WorkspaceCtrl(): Virtuose6D: gripper CLOSE\n");
								handDirCtrlOpenCmd = false;
								handDirCtrlClosedCmd = true;
							}
							else {
								handDirCtrlOpenCmd = true;
								context.notice("WorkspaceCtrl(): Virtuose6D: gripper OPEN\n");
								handDirCtrlClosedCmd = false;
							}

							// update reference poses
							if (this->modeMap && !modeTrj) {
								updateReference();
								//continue;
							}

							ctrlUpdateHand = true;
						}
					}

					virtState = vs;
					virtStateUpdate = true;

					//context.write("S_{r=%u, l=%u, m=%u, f=%u}, P={(%f, %f, %f)}, FT={(%f, %f, %f) (%f, %f, %f)}\n", (U32)vs.buttonRight, (U32)vs.buttonLeft, (U32)vs.buttonMiddle, (U32)vs.buttonForce, cd.p.x, cd.p.y, cd.p.z, ft.v.x, ft.v.y, ft.v.z, ft.w.x, ft.w.y, ft.w.z);
				}
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D

				// update global varibles
				if (ctrlUpdateArm) {
					ctrlUpdate(cbegin.cpos, &ctrlSimModeTrn, &ctrlTime);
					referenceTimeDelta = this->trjTime; // bounded
				}

				// copy global variables
				Mat34 frameTarget, frameGlobal, frameLocal;
				ConfigspaceCoord trjCoord;
				Real trjTime = REAL_ZERO;
				{
					golem::CriticalSectionWrapper csw(cs);
					frameTarget = this->frameTarget;
					frameLocal = this->frameLocal;
					if (modeTrj) {
						trjCoord = cbegin.cpos;
						cbegin.cpos = this->trjCoord;
						trjTime = this->trjTime;
						frameGlobal = this->frameGlobal;
					}
					else if (ctrlUpdateArm) {
						this->frameTarget.setId(); // clear target frame
					}
				}

				// 
				if (modeTrj && this->trjUpdateCalback)
					this->trjUpdateCalback(frameGlobal, frameLocal, frameTarget, trjTime);
				if (!ctrlUpdateArm && !ctrlUpdateHand)
					continue;

				// compute workspace target
				const Mat34 frameCurr = getFrame(
#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
					this->modeMap && !this->modeSimulationVirt && !modeTrj ? this->referencePoseSlave :
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
					cbegin.cpos
				);
				const golem::Mat34 target = getTarget(modeTrj && modeGlobal, frameGlobal, frameLocal, frameTarget, frameCurr, trjTime) * getRef();

				// waypoints
				if (modeWaypoint) {
					if (waypointIndex < (U32)waypoints.size()) {
						// Find collision-free trajectory and wait until the device is ready for new commands
						Controller::State cend = waypoints[waypointIndex];
						cend.t = cbegin.t + ctrlSimPred;
						if (!planner.findGlobalTrajectory(cbegin, cend, trj, trj.begin())) {
							context.error("WorkspaceCtrl(): Unable to find path\n");
							continue;
						}
						// Move the robot
						(void)controller.send(&trj.front(), &trj.back() + 1, true);
						// block
						controller.waitForEnd();
					}
					modeWaypoint = false;
				}
				else if (mode == MODE_POSITION || mode == MODE_ORIENTATION) {
					// Setup target position
					GenWorkspaceChainState::Seq seq(1);
					seq[0].setToDefault(info.getChains().begin(), info.getChains().end()); // all used chains
					seq[0].wpos[ctrlInfo.getChains().begin()] = target;
					seq[0].t = cbegin.t + ctrlSimPred;
					{
						// All bounds are treated as obstacles
						//uiPlanner->setCollisionBoundsGroup(Bounds::GROUP_ALL);
						// Find end position
						planner.findLocalTrajectory(cbegin, seq.begin(), seq.end(), trj, trj.begin(), golem::SecToMSec(ctrlSimReac));
						// direct hand control
						// USE variables handDirCtrlOpenCmd or handDirCtrlClosedCmd
						if (handDirCtrlAvailable) {
							if (handDirCtrlOpenCmd)
								trj.back().cpos.set(handInfo.getJoints(), handDirCtrlOpen);
							else if (handDirCtrlClosedCmd)
								trj.back().cpos.set(handInfo.getJoints(), handDirCtrlClosed);
						}
						trj.back().cvel.setToDefault(info.getJoints()); // target velocity = 0
						trj.back().cacc.setToDefault(info.getJoints()); // target acceleration = 0
						//if (modeTrj)
						{
							//trj.front().t = cbegin.t; // restore time specs
							//trj.front().cpos = trjCoord; // restore coords
							trj.front() = getCommand(context.getTimer().elapsed() + commandLatency);
							trj.back().t = trj.front().t + ctrlSimPred; // restore time specs
							ManipDist dist(*planner.getProfile());
							planner.getProfile()->profile(trj);
							planner.getCallbackDataSync()->syncFindTrajectory(trj.begin(), trj.end(), &seq[0]);
						}
					}
					// Move the robot, skip the first waypoint
					//const SecTmReal tEnd = context.getTimer().elapsed();
					(void)controller.send(&trj.front() + 1, &trj.back() + 1, true);
					// debug
					//context.write("t=%f, proc=%f, reac=%f/%f, pred=%f/%f, sent=%f\n", tEnd, tEnd - t, cbegin.t - t, commandLatency + ctrlSimReac, seq[0].t - t, commandLatency + ctrlSimReac + ctrlSimPred, trj.back().t - t);
				}
				else {
					// Setup workspace target
					GenWorkspaceChainState wend;
					wend.setToDefault(info.getChains().begin(), info.getChains().end()); // all used chains
					wend.wpos[ctrlInfo.getChains().begin()] = target;
					Controller::State cend = cbegin;
					if (!planner.findTarget(cbegin, wend, cend)) {
						context.error("WorkspaceCtrl(): Unable to find target\n");
						continue;
					}
					// Find collision-free trajectory and wait until the device is ready for new commands
					cend.t = cbegin.t + ctrlSimPred;
					if (!planner.findGlobalTrajectory(cbegin, cend, trj, trj.begin())) {
						context.error("WorkspaceCtrl(): Unable to find path\n");
						continue;
					}
					// Move the robot
					(void)controller.send(&trj.front(), &trj.back() + 1, true);
					// block
					controller.waitForEnd();
				}
			}
			catch (const Message& msg) {
				context.write(msg);
			}
		}
	}));

	task->start();

	setActive(desc.active);
}

WorkspaceCtrl::~WorkspaceCtrl() {
	// save cache
	save();
	task.release();
}

//------------------------------------------------------------------------------

void WorkspaceCtrl::load() {
	if (file.length() <= 0)
		return;
	
	try {
		// load cache
		golem::XMLParser::Ptr parser = XMLParser::load(file);
		
		// reference frame
		golem::XMLData(frameLocal, parser->getContextRoot()->getContextFirst("golem activectrl frame_local"));

		// waypoints (optional)
		try {
			waypointIndex = 0;
			waypoints.clear();
			Controller::State command = controller.createState();
			controller.lookupCommand(context.getTimer().elapsed(), command);
			ConfigMat34::Seq configs;
			golem::XMLData(configs, configs.max_size(), parser->getContextRoot()->getContextFirst("golem activectrl"), "waypoint");
			for (ConfigMat34::Seq::const_iterator i = configs.begin(); i != configs.end(); ++i) {
				command.cpos.set(i->c.data(), i->c.data() + std::min(i->c.size(), (size_t)info.getJoints().size()));
				waypoints.push_back(command);
			}
		}
		catch (const golem::MsgXMLParser&) {}
	}
	catch (const golem::Message& msg) {
		context.notice("WorkspaceCtrl::load(): cache not available: %s\n", msg.msg());
	}
}

void WorkspaceCtrl::save() const {
	if (file.length() <= 0)
		return;

	try {
		// Create XML parser
		XMLParser::Ptr parser = XMLParser::Desc().create();

		// reference frame
		golem::XMLData(const_cast<Mat34&>(frameLocal), parser->getContextRoot()->getContextFirst("golem activectrl frame_local", true), true);

		// waypoints
		ConfigMat34::Seq configs;
		for (golem::Controller::State::Seq::const_iterator i = waypoints.begin(); i != waypoints.end(); ++i) {
			// forward transform
			golem::WorkspaceChainCoord wcc;
			controller.chainForwardTransform(i->cpos, wcc);
			// return value
			ConfigMat34 config;
			config.plannerIndex = plannerIndex;
			config.c.assign(i->cpos.data() + *info.getJoints().begin(), i->cpos.data() + *info.getJoints().end());
			config.w = wcc[ctrlInfo.getChains().begin()];
			configs.push_back(config);
		}
		golem::XMLData(configs, configs.max_size(), parser->getContextRoot()->getContextFirst("golem activectrl", true), "waypoint", true);

		// Save config into file
		FileWriteStream fws(file.c_str());
		parser->store(fws);
	}
	catch (const golem::Message& msg) {
		context.notice("WorkspaceCtrl::save(): unable to save cache: %s\n", msg.msg());
	}
}

//------------------------------------------------------------------------------

golem::Real WorkspaceCtrl::getInc() const {
	return Math::pow(REAL_TWO, -Real(incrementStep));
}

golem::Real WorkspaceCtrl::getTimeInc(golem::Real sign) const {
	const Real increment = getInc();
	return sign*increment*trjIncrement;
}

golem::RBDist WorkspaceCtrl::getFrameInc(golem::Real sign) const {
	const Real increment = getInc();
	const RBDist frameIncrement = RBDist(this->modeMap ? REAL_ONE : this->frameIncrement.lin, this->modeMap ? REAL_ONE : this->frameIncrement.ang);
	return golem::RBDist(sign*increment*frameIncrement.lin, sign*increment*frameIncrement.ang);
}

golem::SecTmReal WorkspaceCtrl::getCommandTime() const {
	return context.getTimer().elapsed() + commandLatency + ctrlSimReac;
}

golem::Controller::State WorkspaceCtrl::getCommand(golem::SecTmReal t) const {
	golem::Controller::State cmd = controller.createState();
	controller.lookupCommand(t, cmd);
	return cmd;
}

golem::Mat34 WorkspaceCtrl::getTarget(bool modeGlobal, const Mat34& frameGlobal, const Mat34& frameLocal, const Mat34& frameTarget, const Mat34& frameCurr, golem::SecTmReal trjTime) const {
	return modeGlobal ? getFrameInterpol(frameGlobal * frameLocal, frameTarget, trjTime) * frameCurr : frameCurr * frameLocal * frameTarget * ~frameLocal;
}

golem::Controller::State WorkspaceCtrl::interpolate(const golem::Controller::State::Seq& trajectory, golem::SecTmReal t) const {
	// interpolate
	Controller::State state = trajectory.front();
	controller.interpolateSeq(trajectory.begin(), trajectory.end(), t*(trajectory.back().t - trajectory.front().t) + trajectory.front().t, state);
	return state;
}

golem::Mat34 golem::WorkspaceCtrl::getRef() const {
	return controller.getChains()[ctrlInfo.getChains().begin()]->getReferencePose();
}

golem::Mat34 WorkspaceCtrl::getFrame(const golem::ConfigspaceCoord& coord) const {
	golem::WorkspaceChainCoord wcc;
	controller.chainForwardTransform(coord, wcc);
	return wcc[ctrlInfo.getChains().begin()];
}

golem::Mat34 WorkspaceCtrl::getFrameLocal(const golem::ConfigspaceCoord& coord) const {
	return getFrame(coord) * frameLocal;
}

golem::Mat34 golem::WorkspaceCtrl::getFrameInterpol(const Mat34& frame, const golem::Mat34& trn, Real s) const {
	RBCoord trjInterpol;
	trjInterpol.interpolate(RBCoord::identity(), RBCoord(frame * trn * ~frame), trjScaleCalback ? trjScaleCalback(s) : REAL_ONE);
	return trjInterpol.toMat34();
}

//------------------------------------------------------------------------------

Twist golem::WorkspaceCtrl::toTwist(const Mat34& m) {
	Twist twist;
	twist.v.set(m.p.x, m.p.y, m.p.z);
	m.R.toEuler(twist.w.x, twist.w.y, twist.w.z);
	return twist;
}

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
void golem::WorkspaceCtrl::profile(golem::Controller::State::Seq& trajectory) const {
	// TODO
}

Twist golem::WorkspaceCtrl::getStiffness() const {
	auto trn = [] (Real val) -> Real {
		return Math::sqrt(REAL_ONE - Math::clamp(val < REAL_ZERO ? -val : val, REAL_ZERO, REAL_ONE));
	};
	return this->modeMap ? this->stiffnessDflt * Twist(
		trn(this->trjContactManifold.v.x), trn(this->trjContactManifold.v.y), trn(this->trjContactManifold.v.z),
		trn(this->trjContactManifold.w.x), trn(this->trjContactManifold.w.y), trn(this->trjContactManifold.w.z)
	) : this->stiffnessDflt;
}

Twist golem::WorkspaceCtrl::getDamping() const {
	auto trn = [] (Real val) -> Real {
		return Math::sqrt(REAL_ONE - Math::clamp(val < REAL_ZERO ? -val : val, REAL_ZERO, REAL_ONE));
	};
	return this->modeMap ? this->dampingDflt * Twist(
		trn(this->trjContactManifold.v.x), trn(this->trjContactManifold.v.y), trn(this->trjContactManifold.v.z),
		trn(this->trjContactManifold.w.x), trn(this->trjContactManifold.w.y), trn(this->trjContactManifold.w.z)
	) : this->dampingDflt;
}

Twist golem::WorkspaceCtrl::getCoordTrn(bool modeTrj, const Twist& t) const {
	Twist tmp = t;
	
	if (modeTrj) {
		CoordDesc::trn(trjCoordDescMap, t.data(), tmp.data());
		//std::swap(tmp.v.x, tmp.v.z);
	}
	else {
		CoordDesc::trn(poseCoordDescMap, t.data(), tmp.data());
		//std::swap(tmp.v.x, tmp.v.z);
		//tmp.v.z = -tmp.v.z;
	}

	return tmp;
}

void golem::WorkspaceCtrl::getUpdate(bool modeTrj, const RBCoord& poseMaster, const ConfigspaceCoord& poseSlave, const Twist& force, Twist& updateFrame, SecTmReal& updateTime) const {
	if (this->modeMap) {
		// to Virtuose frame
		updateFrame = getCoordTrn(modeTrj, toTwist(~this->referencePoseMaster.toMat34() * poseMaster.toMat34()));

		if (modeTrj) {
			if (!modeForce)
				updateFrame.arrayMultiply(this->trjContactManifold, updateFrame);
		}
		else {
			updateFrame.arrayMultiply(this->poseGainIncrement, updateFrame);
		}
	}
	else {
		if (modeTrj) {
			updateFrame.arrayMultiply(force, this->forceGainManifold);
			if (!this->trjContactManifoldMap.empty()) {
				updateFrame.arrayMultiply(this->trjContactManifold, updateFrame);
			}
		}
		else {
			updateFrame.arrayMultiply(force, this->forceGainIncrement);
		}
	}
}
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D

//------------------------------------------------------------------------------

void golem::WorkspaceCtrl::trajectoryCtrl(const golem::Controller::State::Seq& trajectory, SecTmReal referenceTime, const Mat34& targetFrame, const Mat34& manifoldFrame, const Twist& manifoldFrameDev, bool globalMode, IActiveCtrl::Control control, IActiveCtrl::Update update, IActiveCtrl::Scale scale) {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "WorkspaceCtrl::trajectoryCtrl(): invalid trajectory size");

	const SecTmReal t = context.getTimer().elapsed();
	
	golem::U32 mode;
	bool modeGlobal;

	ScopeGuard trajectoryGuard([&]() {
		{
			golem::CriticalSectionWrapper csw(cs);
			this->frameTarget.setId();

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
			if (virtuose6D) {
				// restore stiffness and damping
				Controller::State command = virtuose6D->createState();
				virtuose6D->lookupCommand(SEC_TM_REAL_MAX, command);
				
				if (this->modeMap) {
					virtuose6D->getCartImpStiffness(command).setZero();
					virtuose6D->getCartImpDamping(command).setZero();
				}
				
				// set stiffness and damping
				virtuose6D->send(&command, &command + 1, true);
			}
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
		}
		{
			UI::CriticalSectionWrapper cs(getUICallback());
			this->mode = mode;
			this->modeGlobal = modeGlobal;
			this->modeTrj = false;
			trjRendererChains.reset();
		}
	});

	{
		golem::CriticalSectionWrapper csw(cs);
		
		this->trjUpdateCalback = update;
		this->trjScaleCalback = scale;

		this->trjSeq = trajectory;
#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
		if (virtuose6D) {
			// re-profile trjSeq
			profile(this->trjSeq);
		}
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D

		this->trjSeqMat34.clear();
		for (auto &i: this->trjSeq)
			trjSeqMat34.push_back(getFrame(i.cpos));

		this->coordGlobal = interpolate(this->trjSeq, referenceTime).cpos;
		this->frameGlobal = getFrame(coordGlobal);
		this->frameTarget = targetFrame;

		this->frameLocal = manifoldFrame;
		this->trjContactManifold = manifoldFrameDev;
		//this->trjContactManifold.v.z = 0.5;

		this->ctrlSimModeTrn.setZero();

		// find
		this->trjContactManifoldMap.clear();
		for (size_t i = 0; i < ManifoldCtrl::SIZE; ++i) {
			if (this->trjContactManifold[i] < REAL_ZERO)
				throw Message(Message::LEVEL_ERROR, "WorkspaceCtrl::trajectoryCtrl(): invalid manifold value[%u] = %f < 0", (U32)i, this->trjContactManifold[i]);
			this->trjContactManifoldMap.push_back(std::make_pair(this->trjContactManifold[i], i));
		}
		
		// sort if the input device has no all 6 degrees of freedom of control
		if (!this->modeMap)
			std::sort(this->trjContactManifoldMap.begin(), this->trjContactManifoldMap.end(), [] (const ContactManifoldVal& l, const ContactManifoldVal& r) -> bool { return l.first > r.first; });

		// display
		for (U32 i = 0; i < (U32)this->trjContactManifoldMap.size(); ++i)
			context.verbose("Contact manifold #%u: axis=%s, magnitude=%f\n", i + 1, ManifoldCtrl::name[this->trjContactManifoldMap[i].second], this->trjContactManifoldMap[i].first);

		this->trjTime = SEC_TM_REAL_ZERO;
		this->trjTimeDelta = SEC_TM_REAL_ZERO;
		this->trjTimeSent = SEC_TM_REAL_ZERO;

		// target frame adaptation
		const Mat34 frameInterpol = getFrame(interpolate(this->trjSeq, this->trjTime).cpos);
		const Mat34 frameCurr = getFrame(getCommand().cpos);
		//const Mat34 target = getTarget(this->modeGlobal, this->frameGlobal, this->frameLocal, this->frameTarget, frameCurr, this->trjTime);
		if (this->modeGlobal) {
			// TODO
		}
		else {
			// frameInterpol * frameLocal * frameTarget * ~frameLocal = frameCurr |==> frameTarget = ~frameLocal * ~frameInterpol * frameCurr * frameLocal
			this->frameTarget = ~this->frameLocal * ~frameInterpol * frameCurr * this->frameLocal;
			// TODO joint adaptation
		}

		this->trjCoord = interpolate(this->trjSeq, this->trjTime).cpos;

		// initial hand open
		handDirCtrlOpenCmd = true;
		handDirCtrlClosedCmd = false;

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
		if (virtuose6D) {
			virtuose6D->setCallbackTransform([](const Mat34& inp) -> Mat34 {
				Mat34 out = inp;
				out.R.setId();
				return out;
			});

			// set stiffness and damping from contact manifold
			Controller::State command = virtuose6D->createState();
			virtuose6D->lookupCommand(SEC_TM_REAL_MAX, command);

			this->referencePoseMaster = virtuose6D->getCartImpPose(command);
			
			const Twist stiffness = getStiffness();
			virtuose6D->getCartImpStiffness(command) = stiffness;
			context.notice("WorkspaceCtrl::trajectoryCtrl(): Virtuose6D: stiffness UPDATE {(%f, %f, %f)(%f, %f, %f)} -> {(%f, %f, %f)(%f, %f, %f)}\n",
				this->stiffnessDflt.v.x, this->stiffnessDflt.v.y, this->stiffnessDflt.v.z, this->stiffnessDflt.w.x, this->stiffnessDflt.w.y, this->stiffnessDflt.w.z, stiffness.v.x, stiffness.v.y, stiffness.v.z, stiffness.w.x, stiffness.w.y, stiffness.w.z
			);
			
			const Twist damping = getDamping();
			virtuose6D->getCartImpDamping(command) = damping;
			context.notice("WorkspaceCtrl::trajectoryCtrl(): Virtuose6D: damping UPDATE {(%f, %f, %f)(%f, %f, %f)} -> {(%f, %f, %f)(%f, %f, %f)}\n",
				this->dampingDflt.v.x, this->dampingDflt.v.y, this->dampingDflt.v.z, this->dampingDflt.w.x, this->dampingDflt.w.y, this->dampingDflt.w.z, damping.v.x, damping.v.y, damping.v.z, damping.w.x, damping.w.y, damping.w.z
			);

			// set stiffness and damping
			virtuose6D->send(&command, &command + 1, true);
		}
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
	}
	{
		UI::CriticalSectionWrapper cs(getUICallback());

		mode = this->mode;
		modeGlobal = this->modeGlobal;

		this->mode = MODE_POSITION; // default
		this->modeGlobal = globalMode;

		this->modeTrj = true;
		
		trjRendererChains.fromConfigspace(controller, trajectory.begin(), trajectory.end());
		// initial update
		if (this->trjUpdateCalback)
			this->trjUpdateCalback(this->frameGlobal, this->frameLocal, this->frameTarget, this->trjTime);
	}

	// run task
	if (control)
		control();
}

Mat34 golem::WorkspaceCtrl::trajectoryCtrlLocalFrame() const {
	golem::CriticalSectionWrapper csw(cs);
	return frameLocal;
}

Mat34 golem::WorkspaceCtrl::trajectoryCtrlTargetFrame() const {
	golem::CriticalSectionWrapper csw(cs);
	return frameTarget;
}

//------------------------------------------------------------------------------

void WorkspaceCtrl::ctrlUpdate(const golem::ConfigspaceCoord& coord, const Twist* pUpdateFrame, const SecTmReal* pUpdateTime) {
	const bool modeTrj = isTrjMode();
	
	Twist updateFrame = Twist::zero();
	SecTmReal updateTime = SEC_TM_REAL_ZERO;

	{
		UI::CriticalSectionWrapper cs(getUICallback());
		
		updateFrame = pUpdateFrame ? *pUpdateFrame : this->ctrlSimModeTrn;
		this->ctrlSimModeTrn.setZero();
		
		updateTime = pUpdateTime ? *pUpdateTime : this->trjTimeDelta;
		this->trjTimeDelta = SEC_TM_REAL_ZERO;
	}
	{
		golem::CriticalSectionWrapper csw(cs);

		if ((mode == MODE_POSITION || mode == MODE_ORIENTATION) && !updateFrame.isZero()) {
			// compute frame update
			const RBDist frameInc = getFrameInc();

			Mat34 trn = Mat34::identity();
			trn.p = updateFrame.v * frameInc.lin;
			const Vec3 euler = updateFrame.w * frameInc.ang;
			trn.R.fromEuler(euler.x, euler.y, euler.z);
			
			if (modeTrj) {
				this->frameTarget = this->modeForce ? trn : Mat34(Mat33::identity(), trn.p) * this->frameTarget * Mat34(trn.R, Vec3::zero());
			}
			else if (modeGlobal) {
				const Mat34 currentFrame = getFrameLocal(coord);
				//this->frameTarget = Mat34(~currentFrame.R, Vec3::zero()) * trn * Mat34(currentFrame.R, Vec3::zero()) * this->frameTarget; // translation and rotation
				this->frameTarget = Mat34(~currentFrame.R, Vec3::zero()) * Mat34(Mat33::identity(), trn.p) * Mat34(currentFrame.R, Vec3::zero()) * Mat34(trn.R, Vec3::zero()) * this->frameTarget; // only translation
			}
			else {
				this->frameTarget = trn;
			}
		}
		//updateFrame.setZero();

		if (modeTrj && Math::abs(updateTime) > SEC_TM_REAL_ZERO) {
			// interpolate trajectory
			const SecTmReal t = context.getTimer().elapsed();
			const SecTmReal dt = (t - this->trjTimeSent)/(this->trjSeq.back().t - this->trjSeq.front().t);
			//this->trjTime = Math::clamp(this->modeMap ? updateTime : (this->trjTime + Math::clamp(updateTime, -dt, +dt)), SEC_TM_REAL_ZERO, SEC_TM_REAL_ONE);
			this->trjTime = Math::clamp((this->trjTime + Math::clamp(updateTime, -dt, +dt)), SEC_TM_REAL_ZERO, SEC_TM_REAL_ONE);
			this->trjTimeSent = t;
			this->trjCoord = interpolate(this->trjSeq, this->trjTime).cpos;
		}
		//updateTime = SEC_TM_REAL_ZERO;
	}

	if (modeTrj && this->trjUpdateCalback)
		this->trjUpdateCalback(this->frameGlobal, this->frameLocal, this->frameTarget, this->trjTime);
}

bool golem::WorkspaceCtrl::isTrjMode() const {
	return this->modeTrj && (mode == MODE_POSITION || mode == MODE_ORIENTATION);
}

//------------------------------------------------------------------------------

void WorkspaceCtrl::render() const {
	renderer.reset();
	
	if (mode != MODE_DISABLED) {
		if (isTrjMode()) {
			Mat34 frameGlobal, frameLocal, frameTarget;
			golem::ConfigspaceCoord trjCoord;
			{
				golem::CriticalSectionWrapper csw(cs);
				frameGlobal = this->frameGlobal;
				frameLocal = this->frameLocal;
				frameTarget = this->frameTarget;
				trjCoord = this->trjCoord;
			}

			Mat34 frame = getFrame(getCommand().cpos), frameCurr = getFrame(trjCoord);

			// current
			if (trjShowCurr)
				renderer.addAxes3D(frame, frameSize);
			if (trjShowMeanFrame)
				renderer.addAxes3D(frameCurr * frameLocal, frameSize/* * REAL_HALF*/);
			if (trjShowGlobalFrame) {
				ManifoldCtrl manifoldCtrl;
				manifoldCtrl.frame = frameLocal;
				manifoldCtrl.frameDev = this->trjContactManifold;
				appearanceManifold.draw(manifoldCtrl, (modeGlobal ? frameGlobal : frameCurr), renderer);
				if (modeGlobal)
					appearanceMean.draw(manipulator->getBounds(this->coordGlobal, frameGlobal), renderer);
			}

			// mean trajectory
			if (trjShowMean) {
				trjSeqMat34Mean.clear();
				for (auto& trn : trjSeqMat34)
					trjSeqMat34Mean.push_back(trn * frameLocal);
				trjRendererMean.fromFrame(trjSeqMat34Mean.begin(), trjSeqMat34Mean.end());
			}
			trjRendererMean.render();
			trjRendererMean.reset();

			// global reference frame
			if (modeGlobal) {
				const Mat34 trn1 = frameGlobal * frameLocal * frameTarget;
				const Mat34 trn2 = trn1 * ~frameLocal;
				if (trjShowGlobalIncFrame) {
					//renderer.addAxes3D(trn1, frameSize);
					appearanceDev.draw(manipulator->getBounds(this->coordGlobal, trn2), renderer);
				}

				const Mat34 trn3 = trn2 * ~frameGlobal;
				if (trjShowDevFrame)
					renderer.addAxes3D(trn3 * frameCurr * frameLocal, frameSize/* * REAL_HALF*/);

				// deviation trajectory
				if (trjShowDev) {
					trjSeqMat34Dev.clear();
					for (auto& trn : trjSeqMat34)
						trjSeqMat34Dev.push_back(trn3 * trn * frameLocal);
					trjRendererDev.fromFrame(trjSeqMat34Dev.begin(), trjSeqMat34Dev.end());
				}
				trjRendererDev.render();
				trjRendererDev.reset();

				// target
				if (trjShowTar)
					renderer.addAxes3D(getFrameInterpol(frameGlobal * frameLocal, frameTarget, this->trjTime) * frameCurr * frameLocal, frameSize);
			}
			else {
				// target
				if (trjShowTar)
					renderer.addAxes3D(frame * frameLocal, frameSize);
			}

			// render chains
			trjRendererChains.render();
		}
		else {
			Mat34 frame = getFrameLocal(getCommand().cpos);

			// target
			//renderer.addAxes3D(frame, frameSize);

			// target
			{
				golem::CriticalSectionWrapper csw(cs);
				renderer.addAxes3D(frame * this->frameTarget, frameSize);
			}

			// global reference frame
			if (modeGlobal) {
				frame.R.setId();
				renderer.addAxes3D(frame, frameSize/*  * REAL_TWO*/);
			}

			// current waypoint
			if (waypointIndex < waypoints.size())
				renderer.addAxes3D(getFrameLocal(waypoints[waypointIndex].cpos), frameSize * REAL_HALF);
		}
	}

	renderer.render();
}

void WorkspaceCtrl::keyboardHandler(int key, int x, int y) {
	switch (key) {
	case (32): // <space>
		if (isTrjMode() || modeSimple)
			return;
		if (mode == MODE_DISABLED)
			return;
		mode = MODE_DISABLED;
		{
			golem::CriticalSectionWrapper csw(cs);
			this->frameTarget.setId(); // clear target frame
		}
		break;
	case (GLUT_KEY_F5 | UIKeyboardMouseCallback::KEY_SPECIAL) : // F5
		if (modeSimple)
			return;
		mode = mode > MODE_FIRST ? mode - 1 : (isTrjMode() ? MODE_LAST_TRJ : MODE_LAST);
		break;
	case (GLUT_KEY_F6 | UIKeyboardMouseCallback::KEY_SPECIAL) : // F6
		if (modeSimple)
			return;
		mode = mode < U32(isTrjMode() ? MODE_LAST_TRJ : MODE_LAST) ? mode + 1 : MODE_FIRST;
		break;
	case (int('o') | UIKeyboardMouseCallback::KEY_ALT):
		if (handDirCtrlAvailable && (mode == MODE_POSITION || mode == MODE_ORIENTATION)) {
			handDirCtrlOpenCmd = true;
			handDirCtrlClosedCmd = false;
			ctrlUpdateHand = true;
			context.write("Hand direct control OPEN\n");
		}
		return;
	case (int('c') | UIKeyboardMouseCallback::KEY_ALT):
		if (handDirCtrlAvailable && (mode == MODE_POSITION || mode == MODE_ORIENTATION)) {
			handDirCtrlOpenCmd = false;
			handDirCtrlClosedCmd = true;
			ctrlUpdateHand = true;
			context.write("Hand direct control CLOSE\n");
		}
		return;
	case (int('d') | UIKeyboardMouseCallback::KEY_ALT):
		if (handDirCtrlAvailable && (mode == MODE_POSITION || mode == MODE_ORIENTATION)) {
			handDirCtrlOpenCmd = false;
			handDirCtrlClosedCmd = false;
			ctrlUpdateHand = true;
			context.write("Hand direct control DISABLED\n");
		}
		return;
	default:
#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
		if (modeSimulationVirt) {
			switch (key) {
			case (int('l') | UIKeyboardMouseCallback::KEY_ALT):
				virtuose6D->setSimulationButtonLeft(!virtuose6D->getSimulationButtonLeft());
				return;
			case (int('r') | UIKeyboardMouseCallback::KEY_ALT):
				virtuose6D->setSimulationButtonRight(!virtuose6D->getSimulationButtonRight());
				return;
			}
		}
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
		if (mode == MODE_DISABLED)
			return;
		if (modeSimple) {
			if (key == '9') {
				incrementStep = 0;
				const RBDist frameInc = getFrameInc();
				context.write("Workspace control increment: time = %f [sec], position = %f [m], orientation = %f [deg]\n", getTimeInc(), frameInc.lin, Math::radToDeg(frameInc.ang));
				return;
			}
			if (key == '0') {
				incrementStep = incrementLow;
				const RBDist frameInc = getFrameInc();
				context.write("Workspace control increment: time = %f [sec], position = %f [m], orientation = %f [deg]\n", getTimeInc(), frameInc.lin, Math::radToDeg(frameInc.ang));
				return;
			}
		}
		switch (key) {
		case (9) : // tab
			if (modeSimple)
				return;
			modeGlobal = !modeGlobal;
			if (isTrjMode()) {
				//ctrlUpdateArm = true;
			}
			context.write("Frame adjustment %s\n", modeGlobal ? "global" : "local");
			return;
		case ('+') : // insert
			if (isTrjMode()) {
				trjTimeDelta = +getTimeInc();
				ctrlUpdateArm = true;
			}
			else if (modeSimple) {
				incrementStep = 0;
				const RBDist frameInc = getFrameInc();
				context.write("Workspace control increment: time = %f [sec], position = %f [m], orientation = %f [deg]\n", getTimeInc(), frameInc.lin, Math::radToDeg(frameInc.ang));
			}
			else if (waypointIndex <= (U32)waypoints.size()) {
				context.write("Waypoint #%u inserted\n", waypointIndex + 1);
				golem::CriticalSectionWrapper csw(cs);
				waypoints.insert(waypoints.begin() + waypointIndex, getCommand(getCommandTime()));
				++waypointIndex;
			}
			return;
		case ('-') : // remove
			if (isTrjMode()) {
				trjTimeDelta = -getTimeInc();
				ctrlUpdateArm = true;
			}
			else if (modeSimple) {
				incrementStep = incrementLow;
				const RBDist frameInc = getFrameInc();
				context.write("Workspace control increment: time = %f [sec], position = %f [m], orientation = %f [deg]\n", getTimeInc(), frameInc.lin, Math::radToDeg(frameInc.ang));
			}
			else if (waypointIndex < (U32)waypoints.size()) {
				context.write("Waypoint #%u deleted\n", waypointIndex + 1);
				golem::CriticalSectionWrapper csw(cs);
				waypoints.erase(waypoints.begin() + waypointIndex);
				if (waypointIndex > 0) --waypointIndex;
			}
			return;
		case ('=') : // assign
			if (modeSimple)
				return;
			if (waypointIndex < (U32)waypoints.size()) {
				context.write("Waypoint #%u set\n", waypointIndex + 1);
				golem::CriticalSectionWrapper csw(cs);
				waypoints[waypointIndex] = getCommand(getCommandTime());
			}
			return;
		default:
			if (modeSimple)
				return;
			if (!(key & UIKeyboardMouseCallback::KEY_SPECIAL)) {
				const size_t linPos = linKeys.find((char)(key & UIKeyboardMouseCallback::KEY_MASK));
				if (linPos != std::string::npos && linPos < 6) {
					ctrlSimModeTrn.v[linPos / 2] = linPos % 2 ? -REAL_ONE : +REAL_ONE;
					mode = MODE_POSITION;
					ctrlUpdateArm = true;
				}
				const size_t angPos = angKeys.find((char)(key & UIKeyboardMouseCallback::KEY_MASK));
				if (angPos != std::string::npos && angPos < 6) {
					ctrlSimModeTrn.w[angPos / 2] = angPos % 2 ? -REAL_ONE : +REAL_ONE;
					mode = MODE_ORIENTATION;
					ctrlUpdateArm = true;
				}
				const size_t wptPos = wptKeys.find((char)(key & UIKeyboardMouseCallback::KEY_MASK));
				if (wptPos != std::string::npos) {
					if (wptPos < waypoints.size()) {
						context.write("Waypoint #%u run\n", waypointIndex + 1);
						waypointIndex = (U32)wptPos;
						modeWaypoint = true;
						ctrlUpdateArm = true;
					}
					else
						context.write("Waypoint #%u not available\n", wptPos + 1);
				}
			}
			return;
		}
		break;
	}

	// info
	context.write("%s: Workspace control: %s\n", name.c_str(), ModeName[(size_t)mode].c_str());
};

void WorkspaceCtrl::mouseHandler(int button, int state, int x, int y) {
	if (modeSimple)
		return;
	if (mode == MODE_DISABLED)
		return;

	//#define  GLUT_LEFT_BUTTON                   0
	//#define  GLUT_MIDDLE_BUTTON                 1
	//#define  GLUT_RIGHT_BUTTON                  2
	//
	//SCROLL_UP_FWD				                  3
	//SCROLL_DOWN				                  4
	//SCROLL_UP_FWD_RIGHT_BUTTON                  7
	//SCROLL_DOWN_RIGHT_BUTTON                    8
	//
	// 3 - mid fwd
	// 4 - mid dn
	// 19 - mid fwd + ctrl
	// 20 - mid dn + ctrl
	// 11 - mid fwd + shift
	// 12 - mid dn + shift
	// 27 - mid fwd + ctrl + shift
	// 28 - mid dn + ctrl + shift
	//
	// 7 - mid fwd + right
	// 8 - mid dn + right
	// 23 - mid fwd + ctrl + right
	// 24 - mid dn + ctrl + right
	// 15 - mid fwd + shift + right
	// 16 - mid dn + shift + right
	// 31 - mid fwd + ctrl + shift + right
	// 32 - mid dn + ctrl + shift + right

	const int mask = button & ~UIKeyboardMouseCallback::KEY_MASK;
	button &= UIKeyboardMouseCallback::KEY_MASK;

	const bool modeSim = (mask == 0) && (button == 7 || button == 8); // mid fwd/dn + right
	const bool modeTar = (mask == 0) && (button == 3 || button == 4); // mid fwd/dn
	const bool modeRef = (mask & UIKeyboardMouseCallback::KEY_SHIFT) && (button == 3 || button == 4); // mid fwd/dn + shift
	const bool modeAdj = (mask & UIKeyboardMouseCallback::KEY_CTRL) && (button == 3 || button == 4); // mid fwd/dn + ctrl
	const bool modeWpt = (mask & UIKeyboardMouseCallback::KEY_ALT) && (button == GLUT_MIDDLE_BUTTON || button == GLUT_RIGHT_BUTTON || button == 3 || button == 4);

	if (state == GLUT_DOWN && modeAdj) {
		if (button == 3 && incrementStep > 0)
			--incrementStep;
		if (button == 4 && incrementStep < U32(Math::abs(golem::numeric_const<Real>::MIN_EXP)) - 1)
			++incrementStep;

		const RBDist frameInc = getFrameInc();
		context.write("Workspace control increment: time = %f [sec], position = %f [m], orientation = %f [deg]\n", getTimeInc(), frameInc.lin, Math::radToDeg(frameInc.ang));
	}
	else if (state == GLUT_DOWN && modeWpt) {
		if (button == GLUT_MIDDLE_BUTTON) {
			if (waypointIndex < waypoints.size()) {
				modeWaypoint = true;
				ctrlUpdateArm = true;
				context.write("Waypoint #%u run\n", waypointIndex + 1);
			}
		}
		else if (button == GLUT_RIGHT_BUTTON) {
			if (waypointIndex < waypoints.size()) {
				context.write("Waypoint #%u set as target\n", waypointIndex + 1);
				// cj * r * rr * t = wj * r * rr |==> t = (cj * r * rr)^-1 * wj * r * rr
				Mat34 currentFrameInv = getFrameLocal(getCommand(getCommandTime()).cpos);
				currentFrameInv.setInverse(currentFrameInv);
				
				golem::CriticalSectionWrapper csw(cs);
				this->frameTarget = currentFrameInv * getFrameLocal(waypoints[waypointIndex].cpos);
			}
		}
		else if (button == 3 || button == 4) {
			waypointIndex = button == 3 ? (waypointIndex < waypoints.size() ? waypointIndex + 1 : 0) : button == 4 ? (waypointIndex > 0 ? waypointIndex - 1 : (U32)waypoints.size()) : waypointIndex;
			if (waypoints.empty())
				context.write("No waypoints\n");
			else if (waypointIndex < waypoints.size())
				context.write("Waypoint #%u\n", waypointIndex + 1);
		}
	}
	else if (state == GLUT_DOWN && button == GLUT_MIDDLE_BUTTON) {
		if (isTrjMode()) {
			golem::CriticalSectionWrapper csw(cs);
			frameTarget.setId();
		}
		ctrlUpdateArm = true;
	}
	else if (mode == MODE_POSITION || mode == MODE_ORIENTATION) {
		if (state == GLUT_DOWN && button == GLUT_RIGHT_BUTTON) {
			ctrlSimMode = true;
			ctrlSimModeScr.set(x, y);
			ctrlSimModeTrn.setZero();
		}
		else if (state == GLUT_UP && button != 7 && button != 8) {
			ctrlSimMode = false;
			ctrlSimModeTrn.setZero();
		}

		if (ctrlSimMode && state == GLUT_DOWN) {
			if (isTrjMode()) {
				const size_t index = mode == MODE_POSITION ? 2 : 5;
				if (index < this->trjContactManifoldMap.size())
					ctrlSimModeTrn[trjContactManifoldMap[index].second] += (trjContactManifoldMap[index].first * ctrlSimGain.z) * (button == 7 ? +REAL_ONE : button == 8 ? -REAL_ONE : REAL_ZERO);
			}
			else {
				(mode == MODE_POSITION ? ctrlSimModeTrn.v : ctrlSimModeTrn.w).z += button == 7 ? +ctrlSimGain.z : button == 8 ? -ctrlSimGain.z : REAL_ZERO;
			}

			ctrlUpdateArm = true;
		}
	}
	else if (state == GLUT_DOWN) {
		if (modeTar || modeRef) {
			const Real sign = button == 3 ? +REAL_ONE : -REAL_ONE;
			const RBDist frameInc = getFrameInc(sign);

			const Mat34 frameIncrement =
				mode == MODE_POSITION_X ? Mat34(Mat33::identity(), Vec3(frameInc.lin, REAL_ZERO, REAL_ZERO)) :
				mode == MODE_POSITION_Y ? Mat34(Mat33::identity(), Vec3(REAL_ZERO, frameInc.lin, REAL_ZERO)) :
				mode == MODE_POSITION_Z ? Mat34(Mat33::identity(), Vec3(REAL_ZERO, REAL_ZERO, frameInc.lin)) :
				mode == MODE_ORIENTATION_X ? Mat34(Mat33(frameInc.ang, REAL_ZERO, REAL_ZERO), Vec3::zero()) :
				mode == MODE_ORIENTATION_Y ? Mat34(Mat33(REAL_ZERO, frameInc.ang, REAL_ZERO), Vec3::zero()) :
				mode == MODE_ORIENTATION_Z ? Mat34(Mat33(REAL_ZERO, REAL_ZERO, frameInc.ang), Vec3::zero()) : Mat34::identity();

			Mat34& frame = modeTar ? frameTarget : frameLocal;
			if (isTrjMode()) {
				golem::CriticalSectionWrapper csw(cs);
				frame = frame * frameIncrement;
			}
			else if (modeGlobal) {
				Mat34 currentFrame = getFrameLocal(getCommand(getCommandTime()).cpos), currentFrameInv;
				currentFrameInv.setInverse(currentFrame);
				golem::CriticalSectionWrapper csw(cs);
				frame = Mat34(currentFrameInv.R, Vec3::zero()) * frameIncrement * Mat34(currentFrame.R, Vec3::zero()) * frame; // only rotation
			}
			else {
				golem::CriticalSectionWrapper csw(cs);
				frame = frame * frameIncrement;
			}
		}
		else if (button == GLUT_RIGHT_BUTTON && modeRef) {
			context.write("Reference frame reset\n");
			golem::CriticalSectionWrapper csw(cs);
			frameLocal.setId();
		}
	}
}

void WorkspaceCtrl::motionHandler(int x, int y) {
	if (modeSimple)
		return;
	if (ctrlSimMode && (mode == MODE_POSITION || mode == MODE_ORIENTATION)) {
		if (isTrjMode()) {
			const size_t index = mode == MODE_POSITION ? 0 : 3;
			if (index + 0 < this->trjContactManifoldMap.size())
				ctrlSimModeTrn[trjContactManifoldMap[index + 0].second] = trjContactManifoldMap[index + 0].first * ctrlSimGain.x * (ctrlSimModeScr.x - x);
			if (index + 1 < this->trjContactManifoldMap.size())
				ctrlSimModeTrn[trjContactManifoldMap[index + 1].second] = trjContactManifoldMap[index + 1].first * ctrlSimGain.y * (y - ctrlSimModeScr.y); // reversed - (0, 0) at the upper left corner!
		}
		else {
			(mode == MODE_POSITION ? ctrlSimModeTrn.v : ctrlSimModeTrn.w).x = ctrlSimGain.x*(ctrlSimModeScr.x - x);
			(mode == MODE_POSITION ? ctrlSimModeTrn.v : ctrlSimModeTrn.w).y = ctrlSimGain.y*(y - ctrlSimModeScr.y); // reversed - (0, 0) at the upper left corner!
		}

		ctrlSimModeScr.x = x;
		ctrlSimModeScr.y = y;

		ctrlUpdateArm = true;
	}
}

//------------------------------------------------------------------------------
