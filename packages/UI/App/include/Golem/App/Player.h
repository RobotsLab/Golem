/** @file Player.h
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_APP_PLAYER_H_
#define _GOLEM_APP_PLAYER_H_

//------------------------------------------------------------------------------

#include <Golem/App/Recorder.h>
#include <Golem/Plugin/PlannerI.h>
#include <Golem/App/UIPlanner.h>
#include <Golem/Plugin/ActiveCtrl.h>
#include <Golem/Plugin/Point.h>
#include <Golem/Tools/CollisionBounds.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Player. */
class Player : public Recorder {
public:
	/** Planner info */
	class PlannerInfo {
	public:
		/** Sequence */
		typedef std::vector<PlannerInfo> Seq;

		/** Controller leafs ids in the controller tree */
		ControllerId::Seq controllerIDSeq;
		/** Sensors ids */
		StringSeq sensorIDSeq;

		/** Min trajectory duration */
		golem::SecTmReal trajectoryDuration;
		/** Trajectory idle time begin */
		golem::SecTmReal trajectoryIdleBegin;
		/** Trajectory idle time end */
		golem::SecTmReal trajectoryIdleEnd;
		/** Trajectory idle time performance */
		golem::SecTmReal trajectoryIdlePerf;
		/** Max trajectory plan trials */
		golem::U32 trajectoryTrials;
		/** Default handler */
		std::string trajectoryHandler;

		/** ActiveCtrl handler */
		std::string workspacectrlHandler;

		/** Planner */
		golem::Planner* planner;
		/** Used sensors */
		Sensor::Seq sensorSeq;
		
		/** Arm controller info */
		golem::Controller::State::Info armInfo;
		/** Hand controller info */
		golem::Controller::State::Info handInfo;

		/** Constructs from description object */
		PlannerInfo() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			controllerIDSeq.clear();
			sensorIDSeq.clear();

			trajectoryDuration = golem::SecTmReal(1.0);
			trajectoryIdleBegin = golem::SecTmReal(0.5);
			trajectoryIdleEnd = golem::SecTmReal(1.0);
			trajectoryIdlePerf = golem::SecTmReal(5.0);
			trajectoryTrials = 5;
			trajectoryHandler = "Trajectory+Trajectory";

			workspacectrlHandler.clear();

			planner = nullptr;
			sensorSeq.clear();
			armInfo.resetJoints();
			armInfo.resetReserved();
			handInfo.resetJoints();
			handInfo.resetReserved();
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(controllerIDSeq.size() > 1, ac, "controllerIDSeq: two controllers required");
			for (ControllerId::Seq::const_iterator i = controllerIDSeq.begin(); i != controllerIDSeq.end(); ++i)
				Assert::valid(i->isValid(), ac, "controllerIDSeq[i]: invalid");

			Assert::valid(trajectoryDuration > golem::SecTmReal(0.0), ac, "trajectoryDuration <= 0");
			Assert::valid(trajectoryIdleBegin >= golem::SecTmReal(0.0), ac, "trajectoryIdleBegin < 0");
			Assert::valid(trajectoryIdleEnd >= golem::SecTmReal(0.0), ac, "trajectoryIdleEnd < 0");
			Assert::valid(trajectoryIdlePerf >= golem::SecTmReal(0.0), ac, "trajectoryIdleEnd < 0");
			Assert::valid(trajectoryTrials > 0, ac, "trajectoryTrials <= 0");
			Assert::valid(!trajectoryHandler.empty(), ac, "trajectoryHandler: empty");
		}
		/** Set planner. */
		void set(golem::Planner& planner, const Sensor::Map& sensorMap);
	};

	/** Player description */
	class Desc : public Recorder::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** UIPlanner. */
		golem::UIPlanner::Desc::Ptr uiPlannerDesc;
		/** Planner info */
		PlannerInfo::Seq plannerInfoSeq;
		/** Planner index */
		golem::U32 plannerIndex;

		/** Active controller libraries */
		Library::Path::Seq activectrls;

		/** Trajectory profile */
		golem::Profile::Desc::Ptr profileApproachDesc;
		/** Trajectory profile */
		golem::Profile::Desc::Ptr profileManipulationDesc;

		/** Object bounds */
		golem::Bounds::Desc::Seq objectsDesc;
		/** Object bounds appearance */
		golem::Appearance::Seq objectsAppearance;
		
		/** Collision points colour */
		golem::RGBA collisionPointColour;

		/** Server port */
		unsigned short serverPort;
		/** Server clients */
		golem::U32 serverClients;
		/** Server message interval */
		golem::SecTmReal serverMessageInterval;

		/** Default trajectory name */
		std::string trajectoryName;
		/** Semi-auto trajectory name */
		std::string trajectoryProfileSemiAuto;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Recorder::Desc::setToDefault();

			uiPlannerDesc.reset(new golem::UIPlanner::Desc);
			plannerInfoSeq.clear();
			plannerIndex = 0;

			activectrls.clear();

			profileApproachDesc.reset(new Profile::Desc());
			profileApproachDesc->pTrajectoryDesc.reset(new Polynomial4::Desc);
			profileManipulationDesc.reset(new Profile::Desc());
			profileManipulationDesc->pTrajectoryDesc.reset(new Polynomial4::Desc);

			objectsDesc.clear();
			objectsAppearance.clear();
			collisionPointColour = golem::RGBA::BLACK;
			
			serverPort = 54312;
			serverClients = 0; // disable by default
			serverMessageInterval = golem::SecTmReal(60.0);

			trajectoryName = "trajectory";
			trajectoryProfileSemiAuto.clear();
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Recorder::Desc::assertValid(ac);

			Assert::valid(uiPlannerDesc != nullptr && uiPlannerDesc->isValid(), ac, "uiPlannerDesc: invalid");
			Assert::valid(!plannerInfoSeq.empty(), ac, "plannerInfoSeq: empty");
			for (PlannerInfo::Seq::const_iterator i = plannerInfoSeq.begin(); i != plannerInfoSeq.end(); ++i)
				i->assertValid(Assert::Context(ac, "plannerInfoSeq[i]"));

			Assert::valid(profileApproachDesc != nullptr && profileApproachDesc->isValid(), ac, "profileApproachDesc: invalid");
			Assert::valid(profileManipulationDesc != nullptr && profileManipulationDesc->isValid(), ac, "profileManipulationDesc: invalid");

			for (golem::Bounds::Desc::Seq::const_iterator i = objectsDesc.begin(); i != objectsDesc.end(); ++i)
				Assert::valid(*i != nullptr && (*i)->isValid(), ac, "objectsDesc[i]: invalid");
			Assert::valid(objectsAppearance.size() == objectsDesc.size(), ac, "objectsAppearance: invalid size");
			for (golem::Appearance::Seq::const_iterator i = objectsAppearance.begin(); i != objectsAppearance.end(); ++i)
				Assert::valid(i->isValid(), ac, "objectsAppearance[i]: invalid");

			Assert::valid(serverPort > 0, ac, "serverPort: invalid");
			Assert::valid(serverMessageInterval >= golem::SEC_TM_REAL_ZERO, ac, "serverMessageInterval: < 0");

			Assert::valid(!trajectoryName.empty(), ac, "trajectoryName: empty");
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GOLEM_CREATE_FROM_OBJECT_DESC1(Player, golem::Object::Ptr, golem::Scene&)
	};

	/** Runs main task */
	virtual void main(bool runMenu = true);

protected:
	/** UIPlanner */
	golem::UIPlanner* uiPlanner;
	/** Robot controller */
	golem::Controller* controller;
	/** Controller state info */
	golem::Controller::State::Info info;

	/** Planner info */
	PlannerInfo::Seq plannerInfoSeq;
	/** Planner index */
	golem::U32 plannerIndex;

	/** Object bounds */
	golem::Bounds::Desc::Seq objectsDesc;
	/** Object actors */
	golem::Object::Seq objects;

	/** Server port */
	unsigned short serverPort;
	/** Server clients */
	golem::U32 serverClients;
	/** Server message interval */
	golem::SecTmReal serverMessageInterval;

	/** ActiveCtrl interface */
	ActiveCtrl::Map activectrlMap;
	/** ActiveCtrl sequence in order of initialisation */
	ActiveCtrl::Seq activectrlSeq;
	/** ActiveCtrl pointer */
	ActiveCtrl::Map::iterator activectrlCurrentPtr;

	/** Trajectory profile */
	golem::Profile::Ptr profileApproach;
	/** Trajectory profile */
	golem::Profile::Ptr profileManipulation;

	/** Trajectory set index */
	StringIndexMap::value_type::second_type trajectoryIndex;
	/** Trajectory set pointers */
	StringIndexMap trajectoryIndexMap;

	/** Default trajectory name */
	std::string trajectoryName;
	/** Semi-auto trajectory name */
	std::string trajectoryProfileSemiAuto;

	/** Collision points colour */
	golem::RGBA collisionPointColour;

	/** Object renderer */
	golem::DebugRenderer objectRenderer;

	/** Planner info */
	const PlannerInfo& getPlanner() const;

	/** Forward kinematics all chains */
	virtual golem::GenWorkspaceChainState forwardTransformChains(const golem::ConfigspaceCoord& cc) const;
	inline golem::GenWorkspaceChainState forwardTransformChains(const golem::Controller::State& state) const { return forwardTransformChains(state.cpos); }
	/** Forward kinematics arm end-effector */
	virtual golem::Mat34 forwardTransformArm(const golem::ConfigspaceCoord& cc) const;
	inline golem::Mat34 forwardTransformArm(const golem::Controller::State& state) const { return forwardTransformArm(state.cpos); }
	/** (Global search) trajectory */
	virtual size_t findTrajectory(const golem::Controller::State::Seq& trajectory, golem::Controller::State::Seq &approachTrajectory, golem::Controller::State::Seq &manipulationTrajectory, bool mergeTrajectories = false);
	/** (Global search) trajectory from the configuration space and/or workspace target */
	virtual RBDist findTrajectory(const golem::Controller::State& begin, const golem::Controller::State* pcend, const golem::Mat34* pwend, golem::SecTmReal t, golem::Controller::State::Seq& trajectory);
	/** (Local search) trajectory of the arm only from a sequence of configuration space targets in a new reference frame */
	virtual RBDist transformTrajectory(const golem::Mat34& trn, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory);
	/** Send trajectory */
	virtual void sendTrajectory(const golem::Controller::State::Seq& trajectory, bool clear = false);

	/** Colision bounds */
	CollisionBounds::Ptr selectCollisionBounds(bool draw = true);
	/** Perform trajectory */
	virtual void perform(const std::string& data, const std::string& item, data::Trajectory* trajectory, Controller::State::Seq& seq, bool testTrajectory = true, bool mergeTrajectories = false, bool performAuto = true);
	/** Perform trajectory (auto) */
	virtual void performAuto(golem::Controller::State::Seq &approachTrajectory, golem::Controller::State::Seq &manipulationTrajectory, size_t mergeWaypoint, const std::string& data, const std::string& item);
	/** Perform trajectory (semiauto) */
	virtual void performSemiAuto(golem::Controller::State::Seq &approachTrajectory, golem::Controller::State::Seq &manipulationTrajectory, const ManifoldCtrl& manifold, size_t mergeWaypoint, const std::string& data, const std::string& item);

	/** Actice controller pointer */
	virtual ActiveCtrl::Map::const_iterator getWorkspaceCtrlPtr() const;
	/** Actice controller interface */
	virtual IActiveCtrl* getWorkspaceCtrlIf(ActiveCtrl& activeCtrl) const;

	/** Move to the specified configuration */
	virtual void gotoConfig(const golem::Controller::State& state);
	/** Move to the specified configuration */
	virtual void gotoPose(const ConfigMat34& pose);
	/** Read current configuration */
	virtual void getPose(golem::U32 joint, ConfigMat34& pose) const;

	// golem::Object interface
	virtual void render() const;

	/** UIKeyboardMouse: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	/** UIKeyboardMouse: Mouse motion handler. */
	virtual void motionHandler(int x, int y);
	/** UIKeyboardMouse: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);

	virtual void release();

	void create(const Desc& desc);
	Player(golem::Scene &scene);
};

/** Reads/writes object from/to a given XML context */
void XMLData(golem::Player::PlannerInfo::Seq::value_type& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace


#endif /*_GOLEM_APP_PLAYER_H_*/
