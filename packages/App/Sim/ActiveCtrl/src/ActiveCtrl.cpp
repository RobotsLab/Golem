/** @file ActiveCtrl.cpp
*
* @author	Marek Kopicki
*
* @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
*
*/

#include <Golem/Planner/GraphPlanner/DEKinematics.h>
#include <Golem/App/Application.h>
#include <Golem/App/UIPlanner.h>
#include <Golem/App/Data.h>
#include <Golem/App/Common/Tools.h>
#include <GL/glut.h>

using namespace golem;

//------------------------------------------------------------------------------

class Robot : public golem::Object, protected golem::Runnable, protected Profile::CallbackDist {
public:
	/** Arm workspace control mode */
	enum WorkspaceMode {
		/** Inactive */
		WORKSPACE_MODE_DISABLED = 0,
		/** Tool position */
		WORKSPACE_MODE_POSITION,
		/** Tool orientation */
		WORKSPACE_MODE_ORIENTATION,
	};

	class Desc : public golem::Object::Desc {
	public:
		/** Controller + planner */
		UIPlanner::Desc uiPlannerDesc;

		/** Arm workspace control gains */
		golem::Twist workspaceCtrlGain;
		/** Arm workspace control reaction period of time */
		golem::Real workspaceCtrlReac;
		/** Arm workspace control prediction period of time */
		golem::Real workspaceCtrlPred;
		/** Arm workspace control frame size */
		golem::Vec3 workspaceCtrlFrameSize;
		/** convergence test variance threshold */
		golem::Real testVariance;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Object::Desc::setToDefault();

			uiPlannerDesc.setToDefault();
			workspaceCtrlGain.set(golem::Real(0.0002), golem::Real(0.0002), golem::Real(0.001), golem::Real(0.001), golem::Real(0.001), golem::Real(0.01));
			workspaceCtrlReac = golem::Real(0.1);
			workspaceCtrlPred = golem::Real(0.2);
			workspaceCtrlFrameSize.set(0.1);
			testVariance = Real(1e-9);
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Object::Desc::isValid())
				return false;
			if (!uiPlannerDesc.isValid())
				return false;
			if (!workspaceCtrlGain.isValid() || workspaceCtrlReac <= golem::REAL_ZERO || workspaceCtrlReac > workspaceCtrlPred || !workspaceCtrlFrameSize.isPositive() || !golem::Math::isPositive(testVariance))
				return false;
			return true;
		}
	protected:
		CREATE_FROM_OBJECT_DESC_1(Robot, golem::Object::Ptr, golem::Scene&)
	};

	/** Create trajectory */
	bool createTrajectory(const RobotPose::Seq& inp, Controller::State::Seq& out) const {
		if (inp.empty())
			return false;

		Controller::State dflt = controller->createState();
		controller->setToDefault(dflt);

		Controller::State cinit = dflt;
		controller->lookupState(golem::SEC_TM_REAL_MAX, cinit);
		cinit.cvel.setToDefault(info.getJoints());
		cinit.cacc.setToDefault(info.getJoints());

		WorkspaceChainCoord winit;
		controller->chainForwardTransform(cinit.cpos, winit);
		winit[chain].multiply(winit[chain], controller->getChains()[chain]->getReferencePose());

		GenWorkspaceChainState::Seq wtrj(1);
		Controller::State::Seq ctrj;

		out.clear();

		for (RobotPose::Seq::const_iterator i = inp.begin(); i != inp.end(); ++i) {
			out.push_back(dflt);

			if (i->configspace) {
				out.back().cpos.set(i->c.begin(), i->c.end(), *info.getJoints(chain).begin());
			}
			else {
				wtrj.front().wpos[chain].multiply(winit[chain], i->w); // increment
				ctrj.clear();
				planner->findLocalTrajectory(cinit, wtrj.begin(), wtrj.end(), ctrj, ctrj.begin());
				out.back().cpos = ctrj.back().cpos;
			}

			out.back().t = cinit.t + i->t;
		}

		if (out.size() > 2) {
			Profile::Desc desc;
			desc.pCallbackDist = this;
			//desc.pTrajectoryDesc.reset(new Polynomial4::Desc); // 4-th deg polynomial - quadratic velocity
			desc.pTrajectoryDesc.reset(new Polynomial1::Desc); // 1-st deg polynomial - constant velocity
			Profile::Ptr profile = desc.create(*controller);
			if (profile == NULL)
				throw Message(Message::LEVEL_CRIT, "Robot::createTrajectory(): unable to create profile");
			profile->profile(out);
		}

		return true;
	}

	Controller* getController() {
		return controller;
	}
	const Controller* getController() const {
		return controller;
	}
	Planner* getPlanner() {
		return planner;
	}
	const Planner* getPlanner() const {
		return planner;
	}

private:
	/** Arm workspace control mode */
	golem::U32 workspaceMode;
	/** Arm workspace control gains */
	golem::Twist workspaceCtrlGain;
	/** Arm workspace control reaction period of time */
	golem::Real workspaceCtrlReac;
	/** Arm workspace control prediction period of time */
	golem::Real workspaceCtrlPred;
	/** Arm workspace control frame size */
	golem::Vec3 workspaceCtrlFrameSize;
	/** convergence test variance threshold */
	golem::Real testVariance;

	/** Simulated mode */
	bool simMode;
	/** Simulated mode vector */
	golem::Vec3 simModeVec;
	/** Simulated mode screen coords */
	golem::Vec2 simModeScr;

	/** Thread */
	golem::Thread thread;
	/** New task */
	golem::Event ev;
	/** Stop condition variable */
	bool terminate;

	UIPlanner *pUIPlanner;
	Controller* controller;
	Planner* planner;
	Controller::State::Info info;
	Chainspace::Index chain;
	golem::CriticalSection csController;
	golem::DebugRenderer workspaceCtrlRenderer;

	void keyboardHandler(int key, int x, int y) {
		switch (key) {
		case 32: // <space>
			workspaceMode = WORKSPACE_MODE_DISABLED;
			break;
		case (GLUT_KEY_F3 | Universe::KEY_SPECIAL) : // F3
			simModeVec.setZero();
			workspaceMode = workspaceMode >= WORKSPACE_MODE_ORIENTATION ? WORKSPACE_MODE_DISABLED : workspaceMode + 1;
			context.write("%s: workspace control: %s\n", controller->getName().c_str(), workspaceMode == WORKSPACE_MODE_DISABLED ? "disabled" : workspaceMode == WORKSPACE_MODE_POSITION ? "position" : "orientation");
			break;
		default:
			return;
		}
	};

	void mouseHandler(int button, int state, int x, int y) {
		if (state == GLUT_DOWN && button == GLUT_RIGHT_BUTTON) {
			simMode = true;
			simModeScr.set(x, y);
			simModeVec.setZero();
		}
		else if (state == GLUT_UP && button != 7 && button != 8) {
			simMode = false;
			simModeVec.setZero();
		}

		if (simMode) {
			simModeVec.z += button == 7 ? +Real(2.0) : button == 8 ? -Real(2.0) : Real(0.0);
		}

		if (workspaceMode != WORKSPACE_MODE_DISABLED)
			workspaceCtrlRun();
	}

	void motionHandler(int x, int y) {
		if (simMode) {
			simModeVec.x = Real(simModeScr.x - x);
			simModeVec.y = Real(y - simModeScr.y); // (0, 0) at the upper left corner!
		}

		if (workspaceMode != WORKSPACE_MODE_DISABLED)
			workspaceCtrlRun();
	}

	virtual void render() const {
		workspaceCtrlRenderer.render();
	}

	void workspaceCtrlGoal(golem::Controller::State& cbegin, golem::Mat34& wend, golem::Mat34* frames = NULL) const {
		// Joints' config at time t0
		const SecTmReal t = context.getTimer().elapsed() + controller->getCommandLatency() + controller->getCycleDuration() + workspaceCtrlReac;
		controller->lookupCommand(t, cbegin);
		cbegin.t = t;
		
		// Workspace config at time t0
		golem::Mat34 wbegin;
		golem::WorkspaceChainCoord wcc;
		controller->chainForwardTransform(cbegin.cpos, wcc);
		wbegin.multiply(wcc[chain], controller->getChains()[chain]->getReferencePose());

		wend = wbegin;

		Vec3 inc;
		if (workspaceMode == WORKSPACE_MODE_POSITION) {
			inc.arrayMultiply(simModeVec, workspaceCtrlGain.v);
			wend.multiply(wend, Mat34(Mat33::identity(), inc));
		}
		else if (workspaceMode == WORKSPACE_MODE_ORIENTATION) {
			Mat33 R;
			inc.arrayMultiply(simModeVec, workspaceCtrlGain.w);
			R.fromEuler(inc.x, inc.y, inc.z);
			wend.R.multiply(wend.R, R);
		}

		if (frames) { frames[0] = wbegin; frames[1] = wend; }
	}

	void workspaceCtrlRun() {
		workspaceCtrlRenderer.reset();
		if (simMode) {
			golem::Controller::State cbegin = controller->createState();
			golem::Mat34 wend, frames[2];
			workspaceCtrlGoal(cbegin, wend, frames);
			workspaceCtrlRenderer.addAxes3D(frames[0], workspaceCtrlFrameSize);
			workspaceCtrlRenderer.addAxes3D(frames[1], workspaceCtrlFrameSize);
			ev.set(true);
		}
	}

	Real distConfigspaceCoord(const ConfigspaceCoord& prev, const ConfigspaceCoord& next) const {
		WorkspaceChainCoord wprev, wnext;
		controller->chainForwardTransform(prev, wprev);
		wprev[chain].multiply(wprev[chain], controller->getChains()[chain]->getReferencePose());
		controller->chainForwardTransform(next, wnext);
		wnext[chain].multiply(wnext[chain], controller->getChains()[chain]->getReferencePose());
		return wnext[chain].p.distance(wprev[chain].p);

		//Real dist = REAL_ZERO;
		//for (Configspace::Index i = info.getJoints().begin(); i < info.getJoints().end(); ++i)
		//	dist += Math::sqr(prev[i] - next[i]);
		//return Math::sqrt(dist);
	}

	Real distCoord(Real prev, Real next) const {
		return Math::abs(prev - next);
	}

	void run() {
		for (;;) {
			try {
				if (!ev.wait())
					continue;
				ev.set(false);
				if (terminate)
					break;

				Controller::State cbegin = controller->createState();
				golem::Mat34 wend;
				workspaceCtrlGoal(cbegin, wend);
				simModeVec.setZero(); // TODO not safe: writes in mouse handler thread!!!!
				// Setup target position
				GenWorkspaceChainState::Seq seq(1);
				seq[0].setToDefault(info.getChains().begin(), info.getChains().end()); // all used chains
				seq[0].wpos[chain] = wend;
				seq[0].t = cbegin.t + workspaceCtrlPred;
				Controller::Trajectory trj;
				{
					// lock controller
					golem::CriticalSectionWrapper csw(csController);
					// All bounds are treated as obstacles
					pUIPlanner->setCollisionBoundsGroup(Bounds::GROUP_ALL);
					// Find end position
					planner->findLocalTrajectory(cbegin, seq.begin(), seq.end(), trj, trj.begin(), golem::SecToMSec(workspaceCtrlReac));
					trj.back().cvel.setToDefault(info.getJoints()); // target velocity = 0
					trj.back().cacc.setToDefault(info.getJoints()); // target acceleration = 0
				}
				// Move the robot, skip the first waypoint
				(void)controller->send(&trj.front() + 1, &trj.back() + 1, true);
			}
			catch (const Message& msg) {
				context.write(msg);
			}
		}
	}

	Robot(golem::Scene &scene) : golem::Object(scene), pUIPlanner(NULL), controller(NULL), planner(NULL) {}

	void create(const Desc& desc) {
		Object::create(desc); // throws

		// Create UIPlanner
		pUIPlanner = dynamic_cast<UIPlanner*>(scene.createObject(desc.uiPlannerDesc));
		if (pUIPlanner == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create UIPlanner");

		// some useful pointers
		controller = &pUIPlanner->getController();
		planner = pUIPlanner->getPlannerSeq()[0];
		info = controller->getStateInfo();
		chain = info.getChains().begin();

		// rotate tool
		Mat34 tool = controller->getChains()[chain]->getReferencePose();
		Mat34 rot = Mat34::identity();
		rot.R.rotY(Real(0.25)*REAL_PI);
		tool.multiply(tool, rot);
		controller->getChains()[chain]->setReferencePose(tool);

		simMode = false;
		simModeVec.setZero();
		workspaceMode = WORKSPACE_MODE_DISABLED;
		workspaceCtrlGain = desc.workspaceCtrlGain;
		workspaceCtrlReac = desc.workspaceCtrlReac;
		workspaceCtrlPred = desc.workspaceCtrlPred;
		workspaceCtrlFrameSize = desc.workspaceCtrlFrameSize;
		testVariance = desc.testVariance;

		// set high accuracy of IK solver
		DEKinematics* kinematics = dynamic_cast<DEKinematics*>(&planner->getKinematics());
		if (kinematics) {
			DEKinematics::Desc desc = kinematics->getDesc();
			desc.testVariance = testVariance;
			kinematics->setDesc(desc);
		}

		terminate = false;
		if (!thread.start(this))
			throw golem::Message(golem::Message::LEVEL_CRIT, "Robot(): Unable to start active workspace control thread");
		(void)thread.setPriority(golem::Thread::NORMAL); // ignore retval
	}

	void release() {
		terminate = true;
		ev.set(true);
		(void)thread.join(golem::MSEC_TM_U32_INF); // ignore retval
	}
};

void XMLData(Robot::Desc &val, Context* context, XMLContext* xmlcontext, bool create = false) {
	golem::XMLData(val.uiPlannerDesc, context, xmlcontext, create);

	xmlcontext = xmlcontext->getContextFirst("robot");

	golem::XMLData("test_variance", val.testVariance, xmlcontext->getContextFirst("active_workspace_ctrl_arm"), create);
	golem::XMLData("pred_time", val.workspaceCtrlPred, xmlcontext->getContextFirst("active_workspace_ctrl_arm"), create);
	golem::XMLData("reac_time", val.workspaceCtrlReac, xmlcontext->getContextFirst("active_workspace_ctrl_arm"), create);
	golem::XMLData(val.workspaceCtrlGain, xmlcontext->getContextFirst("active_workspace_ctrl_arm gain"), create);
	golem::XMLData(val.workspaceCtrlFrameSize, xmlcontext->getContextFirst("active_workspace_ctrl_arm frame_size"), create);
}

//------------------------------------------------------------------------------

/** MyApplication */
class MyApplication : public Application {
protected:
	/** Get trajectory by name */
	RobotPose::Map::const_iterator getTrajectory(const Controller& controller, const RobotPose::Map& map) {
		const std::string chars = "0123456789";
		std::string name;

		for (;;) {
			context()->write("\rEnter trajectory name: %s ", name.c_str());

			const int key = universe()->waitKey();

			if (universe()->interrupted() || key == 27) {
				return map.end();
			}
			else if (key == '*') {
				Controller::State state = controller.createState();
				controller.lookupState(SEC_TM_REAL_MAX, state);
				context()->write("\n<waypoint dt=\"1\"");
				for (golem::Configspace::Index i = state.getInfo().getJoints().begin(); i < state.getInfo().getJoints().end(); ++i)
					context()->write(" c%d=%0.5f", (*i - *state.getInfo().getJoints().begin() + 1), state.cpos[i]);
				context()->write("/>\n");
			}
			else if (key == 13) { // enter
				context()->write("\n");
				break;
			}
			else if (key == 8) {  // <Bkspace>
				if (name.length() > 0)
					name.erase(name.length() - 1);
			}
			else if (chars.find((char)key) != std::string::npos) { // numbers
				name += (char)key;
			}
			else if (key == '?' || key == ' ') {
				const std::string blank(name.length(), ' ');
				context()->write("\rAvailable trajectories:%s\n", blank.c_str());
				for (RobotPose::Map::const_iterator i = map.begin(); i != map.end(); ++i)
					context()->write(" %s\n", i->first.c_str());
			}
		}

		return map.find(name);
	}

	/** Runs MyApplication */
	virtual void run(int argc, char *argv[]) {
		// Setup planner
		Robot::Desc robotDesc;
		XMLData(robotDesc, context(), xmlcontext());

		// Create UIPlanner
		context()->info("Initialising Robot...\n");
		Robot *robot = dynamic_cast<Robot*>(scene()->createObject(robotDesc));
		if (robot == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create Robot");

		// some useful pointers
		Controller &controller = *robot->getController();
		Planner &planner = *robot->getPlanner();

		RobotPose::Map map;
		loadTrajectoryMap(controller, xmlcontext(), map);

		// main loop
		for (; !universe()->interrupted();) {
			RobotPose::Map::const_iterator trajectory = getTrajectory(controller, map);
			Controller::State::Seq seq;

			if (trajectory != map.end() && robot->createTrajectory(trajectory->second, seq))
				controller.send(seq.data(), seq.data() + seq.size(), true);
		}

		context()->info("Good bye!\n");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}
