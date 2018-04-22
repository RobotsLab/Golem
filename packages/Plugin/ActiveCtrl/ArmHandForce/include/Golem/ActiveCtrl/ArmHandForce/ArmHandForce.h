/** @file ArmHandForce.h
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
#ifndef _GOLEM_ACTIVECTRL_ARMHAND_FORCE_ARMHAND_FORCE_H_
#define _GOLEM_ACTIVECTRL_ARMHAND_FORCE_ARMHAND_FORCE_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/ActiveCtrl.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Tools/FT.h>
#include <Golem/ActiveCtrl/ActiveCtrl/ActiveCtrlForce.h>
#include <Golem/Math/Vec2.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** ArmHandForce is the interafce to a robot (right arm + hand), sensing devices and objects. */
class GOLEM_LIBRARY_DECLDIR ArmHandForce : public ActiveCtrl, public UI {
public:
	/** Simulated arm/hand force control mode */
	enum ForceMode {
		/** Inactive */
		FORCE_MODE_DISABLED = 0,
		/** Torque */
		FORCE_MODE_TORQUE,
		/** Force */
		FORCE_MODE_FORCE,
	};
	
	/** ArmHandForce factory */
	class GOLEM_LIBRARY_DECLDIR Desc : public ActiveCtrl::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Active arm controller. */
		ActiveCtrlForce::Desc::Ptr armCtrlDesc;
		/** Active hand controller. */
		ActiveCtrlForce::Desc::Ptr handCtrlDesc;
		
		/** FT sensor gain */
		golem::Twist ftGain;
		/** FT sensor force/torque limit */
		golem::Twist ftLimit;
		/** Force sensor limit */
		RealSeq fLimit;

		/** Simulated force gains */
		golem::Twist simForceGain;

		/** Hand impedance control, min and max stiffness levels */
		RealSeq impedStiffMin, impedStiffMax;
		/** Hand impedance control, stiffness num of steps */
		golem::U32 impedStiffSteps;
		/** Hand impedance control, stiffness initial step */
		golem::U32 impedStiffStepInit;

		/** Hand impedance control, min and max damping levels */
		RealSeq impedDampMin, impedDampMax;
		/** Hand impedance control, damping num of steps */
		golem::U32 impedDampSteps;
		/** Hand impedance control, damping initial step */
		golem::U32 impedDampStepInit;

		/** Emergency mode handler */
		ThreadTask::Function emergencyModeHandler;
		/** Arm force reader */
		ActiveCtrlForce::ForceReader armForceReader;
		/** Hand force reader */
		ActiveCtrlForce::ForceReader handForceReader;

		/** Joint force show */
		bool appearanceShowForce;
		/** FT sensor values show */
		bool appearanceShowForceTorque;
		/** Joint force scaling factor */
		golem::Real appearanceFScale;
		/** F/T sensor force and torque */
		golem::Vec3 appearanceFTSize;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			ActiveCtrl::Desc::setToDefault();

			armCtrlDesc.reset(new ActiveCtrlFT::Desc);
			handCtrlDesc.reset(new ActiveCtrlForce::Desc);

			ftGain.set(golem::Real(-1.0), golem::Real(-1.0), golem::Real(-1.0), golem::Real(-1.0), golem::Real(-1.0), golem::Real(-1.0));
			ftLimit.set(golem::Real(10.0), golem::Real(10.0), golem::Real(10.0), golem::Real(2.0), golem::Real(2.0), golem::Real(2.0));
			fLimit.clear();

			simForceGain.set(golem::Real(0.02), golem::Real(0.02), golem::Real(0.02), golem::Real(0.02), golem::Real(0.02), golem::Real(0.02));
			
			impedStiffMin.assign(4, golem::Real(0.0));
			impedStiffMax.assign(4, golem::Real(100.0));
			impedStiffSteps = 10;
			impedStiffStepInit = 2;

			impedDampMin.assign(4, golem::Real(0.0));
			impedDampMax.assign(4, golem::Real(100.0));
			impedDampSteps = 10;
			impedDampStepInit = 2;

			emergencyModeHandler = nullptr;
			armForceReader = nullptr;
			handForceReader = nullptr;

			appearanceShowForce = true;
			appearanceShowForceTorque = true;
			appearanceFScale = golem::Real(0.2);
			appearanceFTSize.set(golem::Real(1.0), golem::Real(1.0), golem::Real(1.0));
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			ActiveCtrl::Desc::assertValid(ac);

			Assert::valid(armCtrlDesc != nullptr, ac, "armCtrlDesc == NULL");
			armCtrlDesc->assertValid(Assert::Context(ac, "armCtrlDesc->"));
			Assert::valid(handCtrlDesc != nullptr, ac, "handCtrlDesc == NULL");
			handCtrlDesc->assertValid(Assert::Context(ac, "handCtrlDesc->"));

			Assert::valid(ftGain.isValid(), ac, "ftGain: invalid");
			Assert::valid(ftLimit.isPositive(REAL_EPS), ac, "ftLimit < eps");

			for (RealSeq::const_iterator i = fLimit.begin(); i != fLimit.end(); ++i)
				Assert::valid(golem::Math::isPositive(*i), ac, "fLimit[i] <= 0");

			Assert::valid(simForceGain.isValid(), ac, "simForceGain: invalid");

			Assert::valid(!impedStiffMin.empty(), ac, "impedStiffMin[].size() == 0");
			Assert::valid(impedStiffMin.size() == impedStiffMax.size(), ac, "impedStiffMin[].size() != impedStiffMax[].size()");
			for (size_t i = 0; i < impedStiffMin.size(); ++i) {
				Assert::valid(impedStiffMin[i] >= golem::REAL_ZERO, ac, "impedStiffMin[i] < 0");
				Assert::valid(impedStiffMin[i] + golem::REAL_EPS < impedStiffMax[i], ac, "impedStiffMin[i] < impedStiffMax[i] + eps");
			}
			Assert::valid(impedStiffSteps > 1, ac, "impedStiffSteps < 2");
			Assert::valid(impedStiffStepInit <= impedStiffSteps, ac, "impedStiffStepInit > impedStiffSteps");

			Assert::valid(!impedDampMin.empty(), ac, "impedDampMin[].size() == 0");
			Assert::valid(impedDampMin.size() == impedDampMax.size(), ac, "impedDampMin[].size() != impedDampMax[].size()");
			for (size_t i = 0; i < impedDampMin.size(); ++i) {
				Assert::valid(impedDampMin[i] >= golem::REAL_ZERO, ac, "impedDampMin[i] < 0");
				Assert::valid(impedDampMin[i] + golem::REAL_EPS < impedDampMax[i], ac, "impedDampMin[i] < impedDampMax[i] + eps");
			}
			Assert::valid(impedDampSteps > 1, ac, "impedDampSteps < 2");
			Assert::valid(impedDampStepInit <= impedDampSteps, ac, "impedDampStepInit > impedDampSteps");

			Assert::valid(appearanceFScale > golem::REAL_ZERO, ac, "appearanceFScale <= 0");
			Assert::valid(appearanceFTSize.isPositive(), ac, "appearanceFTSize <= 0");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		GOLEM_CREATE_FROM_OBJECT_DESC2(ArmHandForce, ActiveCtrl::Ptr, golem::Planner&, const Sensor::Seq&)
	};

	/** Activate/deactivate */
	virtual void setActive(bool active = true);

	/** Arm: F/T limits */
	void getLimits(golem::Twist &wrench) const;
	/** Hand: force limits */
	void getLimits(RealSeq &force) const;
	/** Arm: F/T limits */
	void setLimits(const golem::Twist &wrench);
	/** Hand: force limits */
	void setLimits(const RealSeq &force);

	/** Arm: current force */
	void getArmForce(golem::Twist &wrench) const;
	/** Hand: current force */
	void getHandForce(RealSeq& force) const;

	/** Emergency mode handler */
	void setEmergencyModeHandler(ThreadTask::Function emergencyModeHandler);
	/** Arm force reader */
	void setArmForceReaderr(ActiveCtrlForce::ForceReader armForceReader);
	/** Hand force reader */
	void setHandForceReader(ActiveCtrlForce::ForceReader handForceReader);
	/** Arm force reader */
	ActiveCtrlForce::ForceReader getArmForceReaderDflt() const {
		return armForceReaderDflt;
	}
	/** Hand force reader */
	ActiveCtrlForce::ForceReader getHandForceReaderDflt() const {
		return handForceReaderDflt;
	}

	/** Active arm controller. */
	ActiveCtrlForce* getArmCtrl() {
		return armCtrl.get();
	}
	/** Active arm controller. */
	const ActiveCtrlForce* getArmCtrl() const {
		return armCtrl.get();
	}
	/** Active hand controller. */
	ActiveCtrlForce* getHandCtrl() {
		return handCtrl.get();
	}
	/** Active hand controller. */
	const ActiveCtrlForce* getHandCtrl() const {
		return handCtrl.get();
	}

protected:
	/** ArmHandForce arm */
	golem::SingleCtrl *arm;
	/** ArmHandForce hand */
	golem::SingleCtrl *hand;
	/** Force and torque sensor interface */
	golem::FT* ftSensor;

	/** Controller state info */
	golem::Controller::State::Info armInfo;
	/** Controller state info */
	golem::Controller::State::Info handInfo;
	
	/** Force access cs */
	mutable golem::CriticalSection csData;

	/** Simulated arm force mode */
	golem::U32 simArmForceMode;
	/** Simulated hand force mode */
	golem::U32 simHandForceMode;
	/** Simulated force gains */
	golem::Twist simForceGain;

	/** Simulated mode */
	bool simMode;
	/** Simulated mode vector */
	golem::Vec3 simModeVec;
	/** Simulated mode screen coords */
	golem::Vec2 simModeScr;

	/** F/T limit emergency mode on/off */
	bool emgMode;

	/** Active arm controller. */
	ActiveCtrlForce::Ptr armCtrl;
	/** Active hand controller. */
	ActiveCtrlForce::Ptr handCtrl;

	/** FT sensor frame with respect to the tool frame */
	golem::Mat34 ftFrame, ftFrameInv;
	/** FT sensor gain */
	golem::Twist ftGain;
	/** FT sensor force/torque limit */
	golem::Twist ftLimit;
	/** Force sensor limit */
	RealSeq fLimit;

	/** Hand impedance control, min and max stiffness levels */
	RealSeq impedStiffMin, impedStiffMax;
	/** Hand impedance control, stiffness level */
	RealSeq impedStiff;
	/** Hand impedance control, stiffness num of steps */
	golem::U32 impedStiffSteps;
	/** Hand impedance control, stiffness step */
	golem::U32 impedStiffStep;
	/** Hand impedance control, min and max damping levels */
	RealSeq impedDampMin, impedDampMax;
	/** Hand impedance control, damping level */
	RealSeq impedDamp;
	/** Hand impedance control, damping num of steps */
	golem::U32 impedDampSteps;
	/** Hand impedance control, damping step */
	golem::U32 impedDampStep;

	/** Arm force */
	golem::Twist armForce;
	/** Hand force */
	RealSeq handForce;
	
	/** Emergency mode handler */
	ThreadTask::Function emergencyModeHandler;
	/** Emergency mode handler thread */
	ThreadTask emergencyModeHandlerThread;
	/** Arm force reader */
	ActiveCtrlForce::ForceReader armForceReader, armForceReaderDflt;
	/** Hand force reader */
	ActiveCtrlForce::ForceReader handForceReader, handForceReaderDflt;

	/** Hand offset */
	golem::U32 offsetIdx;
	/** Arm joint torques */
	mutable RealSeq armJointTorques;
	/** Hand joint torques */
	mutable RealSeq handJointTorques;

	/** Joint force show */
	bool appearanceShowForce;
	/** FT sensor values show */
	bool appearanceShowForceTorque;
	/** Joint force scaling factor */
	golem::Real appearanceFScale;
	/** F/T sensor force and torque */
	golem::Vec3 appearanceFTSize;

	/** Render data */
	mutable golem::DebugRenderer renderer;

	/** Key */
	int key;

	/** Compute step-th value in <min, max> assuming logarythmic scale */
	virtual void level(const char* name, const RealSeq& min, const RealSeq& max, golem::U32 steps, golem::U32 step, RealSeq& val);

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
	ArmHandForce(golem::Planner &planner, const Sensor::Seq& sensorSeq);
	/** Cleaning */
	~ArmHandForce();
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_ACTIVECTRL_ARMHAND_FORCE_ARMHAND_FORCE_H_*/
