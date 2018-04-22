/** @file JointCtrl.h
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
#ifndef _GOLEM_ACTIVECTRL_JOINT_CTRL_JOINT_CTRL_H_
#define _GOLEM_ACTIVECTRL_JOINT_CTRL_JOINT_CTRL_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/ActiveCtrl.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Math/RB.h>
#include <Golem/Math/Vec2.h>
#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_JOINTCTRL_SFML
#include <Golem/ActiveCtrl/ActiveCtrl/InputCtrl.h>
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_JOINTCTRL_SFML

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** JointCtrl is the interafce to a robot (right arm + hand), sensing devices and objects. */
class GOLEM_LIBRARY_DECLDIR JointCtrl : public ActiveCtrl, public UI {
public:
	/** Key -> joint map */
	typedef std::multimap<char, std::pair<golem::U32, golem::Real>> KeyMap;

	/** JointCtrl factory */
	class GOLEM_LIBRARY_DECLDIR Desc : public ActiveCtrl::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Control thread idle sleep */
		golem::MSecTmU32 ctrlThreadSleep;

		/** Coordinate tolerance at limits */
		golem::RBDist tolerance;

		/** Trajectory velocity multiplier */
		golem::Real velocity;
		/** Trajectory acceleration multiplier */
		golem::Real acceleration;
		/** Trajectory duration */
		golem::SecTmReal duration;

		/** Trajectory increment */
		golem::RBDist increment;
		/** Low increment */
		golem::U32 incrementLow;

		/** Frame size */
		golem::Vec3 frameSize;
		/** Frame size */
		golem::Vec3 frameSizeActive;

		/** Increment keys */
		KeyMap incrementKeyMap;
		/** Target keys */
		KeyMap targetKeyMap;

		/** Simplified operation mode */
		bool simpleMode;

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_JOINTCTRL_SFML
		/** Input device descriptor */
		InputCtrl::Desc inputCtrlDesc;
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_JOINTCTRL_SFML

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			ActiveCtrl::Desc::setToDefault();

			ctrlThreadSleep = MSecTmU32(100);
			increment.set(golem::Real(0.01), golem::Real(0.05)*golem::REAL_PI);
			tolerance.set(golem::Real(0.001), golem::Real(0.005)*golem::REAL_PI);
			velocity = golem::Real(0.1);
			acceleration = golem::Real(0.1);
			duration = golem::SecTmReal(1.0);
			incrementLow = 4;
			frameSize.set(golem::Real(0.2), golem::Real(0.2), golem::Real(0.2));
			frameSizeActive.set(golem::Real(0.3), golem::Real(0.3), golem::Real(0.3));
			incrementKeyMap.clear();
			targetKeyMap.clear();
			simpleMode = false;

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_JOINTCTRL_SFML
			inputCtrlDesc.setToDefault();
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_JOINTCTRL_SFML
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			ActiveCtrl::Desc::assertValid(ac);

			Assert::valid(ctrlThreadSleep >= 0, ac, "ctrlThreadSleep: < 0");
			Assert::valid(increment.isPositive(), ac, "increment: <= 0");
			Assert::valid(tolerance.isPositive(), ac, "tolerance: <= 0");
			Assert::valid(velocity > golem::REAL_ZERO, ac, "velocity: <= 0");
			Assert::valid(acceleration > golem::REAL_ZERO, ac, "acceleration: <= 0");
			Assert::valid(duration > golem::SEC_TM_REAL_ZERO, ac, "duration: <= 0");
			Assert::valid(frameSize.isPositive(), ac, "frameSize: <= 0");
			Assert::valid(frameSizeActive.isPositive(), ac, "frameSizeActive: <= 0");

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_JOINTCTRL_SFML
			inputCtrlDesc.assertValid(Assert::Context(ac, "inputCtrlDesc."));
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_JOINTCTRL_SFML
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		GOLEM_CREATE_FROM_OBJECT_DESC2(JointCtrl, ActiveCtrl::Ptr, golem::Planner&, const Sensor::Seq&)
	};

protected:
	/** Bool */
	typedef golem::Configspace::Coord<bool> ConfigspaceBool;

	/** Limits min */
	golem::GenConfigspaceCoord cmin;
	/** Limits max */
	golem::GenConfigspaceCoord cmax;
	
	/** Control thread idle sleep */
	golem::MSecTmU32 ctrlThreadSleep;
	/** Coordinate tolerance at limits */
	golem::RBDist tolerance;
	/** Trajectory velocity multiplier */
	golem::Real velocity;
	/** Trajectory acceleration multiplier */
	golem::Real acceleration;
	/** Trajectory duration */
	golem::SecTmReal duration;
	/** Trajectory increment */
	golem::RBDist increment;
	/** Low increment */
	golem::U32 incrementLow;
	/** Trajectory increment step */
	golem::U32 incrementStep;
	/** Current joint */
	golem::U32 joint;
	/** Frame size */
	golem::Vec3 frameSize;
	/** Frame size */
	golem::Vec3 frameSizeActive;
	/** Trajectory time stamp */
	golem::SecTmReal timeStamp;

	/** Increment keys */
	KeyMap incrementKeyMap;
	/** Target keys */
	KeyMap targetKeyMap;

	/** Simplified operation mode */
	bool simpleMode;

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_JOINTCTRL_SFML
	/** Input devices */
	InputCtrl::Ptr inputCtrlPtr;
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_JOINTCTRL_SFML

	/** Render data */
	mutable golem::DebugRenderer renderer;

	/** Controller task */
	ThreadTask::Ptr task;

	/** Joint increment */
	golem::RBDist getJointIncrement() const;
	/** Revolute joint */
	bool isRevolute(golem::Configspace::Index index) const;
	/** Joint increment */
	golem::Real getJointIncrement(golem::Configspace::Index index) const;
	/** Joint tolerance */
	golem::Real getJointTolerance(golem::Configspace::Index index) const;

	/** send target state */
	void sendNext(golem::ConfigspaceCoord& target, const ConfigspaceBool* positionMode = nullptr);

	/** golem::UIRenderer: Render on output device. */
	virtual void render() const;
	/** golem::UIRenderer: Render on output device. */
	virtual void customRender() const {}

	/** golem::UIKeyboardMouse: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	/** golem::UIKeyboardMouse: Mouse motion handler. */
	virtual void motionHandler(int x, int y);
	/** golem::UIKeyboardMouse: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);

	/** Creates/initialises the ActiveCtrl */
	void create(const Desc& desc);

	/** Constructs the ActiveCtrl */
	JointCtrl(golem::Planner &planner, const Sensor::Seq& sensorSeq);
};

//------------------------------------------------------------------------------

};	// namespace

namespace golem {
	/** Reads/writes object from/to a given XML context */
	void XMLData(golem::JointCtrl::KeyMap::value_type& val, golem::XMLContext* xmlcontext, bool create = false);
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_ACTIVECTRL_JOINT_CTRL_JOINT_CTRL_H_*/
