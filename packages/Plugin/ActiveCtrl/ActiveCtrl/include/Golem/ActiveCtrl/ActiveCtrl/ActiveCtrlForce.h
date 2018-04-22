/** @file ActiveCtrlForce.h
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
#ifndef _GOLEM_ACTIVECTRL_ACTIVECTRL_ACTIVECTRL_FORCE_H_
#define _GOLEM_ACTIVECTRL_ACTIVECTRL_ACTIVECTRL_FORCE_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Defs.h>
#include <Golem/ActiveCtrl/ActiveCtrl/ActiveSingleCtrl.h>
#include <Golem/Sys/XMLParser.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Active manipulator controll using force sensors. */
class ActiveCtrlForce : public Active, public ActiveSingleCtrl {
public:
	typedef golem::shared_ptr<ActiveCtrlForce> Ptr;

	/** Force reader */
	typedef std::function<void(const golem::Controller::State&, RealSeq&)> ForceReader;
	/** Action filter */
	typedef std::function<void(const golem::Controller::State&, golem::Controller::State&, bool, bool)> ActionFilter;

	/** Active mode operation states */
	enum Mode {
		/** Active control disabled */
		MODE_DISABLED,
		/** Active control enabled */
		MODE_ENABLED,
		/** Emergency mode, no control possible */
		MODE_EMERGENCY,
	};

	/** Active mode operation on/off step */
	static const golem::I32 STEP_DEFAULT = golem::numeric_const<golem::I32>::MAX;

	/** Mode name */
	static const char* ModeName[];

	/** Object description */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
	
		/** Number of steps before starting */
		golem::U32 startSteps;
		/** Number of steps before stopping */
		golem::U32 stopSteps;
		/** Force first order low-pass filter smoothing factor */
		RealSeq filter;
		/** Force threshold */
		RealSeq threshold;
		/** Gain */
		RealSeq gain;
		/** Velocity limit */
		RealSeq velocity;

		/** Force reader */
		ForceReader forceReader;
		/** Action filter */
		ActionFilter actionFilter;
		
		/** Init timeout */
		golem::MSecTmU32 timeOut;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}
		
		/** virtual destructor */
		virtual ~Desc() {}
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			startSteps = 20;
			stopSteps = 20;
			filter.assign(golem::Configspace::DIM, golem::Real(0.1));
			threshold.assign(golem::Configspace::DIM, golem::Real(0.02));
			gain.assign(golem::Configspace::DIM, golem::Real(-0.001));
			velocity.assign(golem::Configspace::DIM, golem::Real(0.05)*golem::REAL_PI);

			forceReader = [] (const golem::Controller::State&, RealSeq& force) {
				force.assign(force.size(), golem::REAL_ZERO);
			};
			actionFilter = nullptr;
			timeOut = golem::MSEC_TM_U32_INF;
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Assert::valid(startSteps > 0 && startSteps < 1000, ac, "startSteps: not in (0, 1000)");
			Assert::valid(stopSteps > 0, ac, "stopSteps: <= 0");

			Assert::valid(!filter.empty(), ac, "filter[]: empty");
			for (RealSeq::const_iterator j = filter.begin(); j < filter.end(); ++j)
				Assert::valid(*j > golem::REAL_EPS && *j <= golem::REAL_ONE, ac, "filter[]: not in (eps, 1]");

			Assert::valid(!threshold.empty(), ac, "threshold[]: empty");
			for (RealSeq::const_iterator j = threshold.begin(); j < threshold.end(); ++j)
				Assert::valid(*j >= golem::REAL_ZERO, ac, "threshold[]: < 0");

			Assert::valid(!velocity.empty(), ac, "velocity[]: empty");
			for (RealSeq::const_iterator j = velocity.begin(); j < velocity.end(); ++j)
				Assert::valid(*j >= golem::REAL_EPS, ac, "threshold[]: < eps");

			Assert::valid(!gain.empty(), ac, "gain[]: empty");
			for (RealSeq::const_iterator j = gain.begin(); j < gain.end(); ++j)
				Assert::valid(golem::Math::isFinite(*j), ac, "gain[]: infinite");

			Assert::valid(forceReader != nullptr, ac, "forceReader: null");
		}

		/** Creates object from description. */
		GOLEM_CREATE_FROM_OBJECT_DESC1(ActiveCtrlForce, ActiveCtrlForce::Ptr, golem::SingleCtrl&)
	};

private:
	/** Number of averaging steps before starting */
	golem::U32 startSteps;
	/** Number of averaging steps before stopping */
	golem::U32 stopSteps;
	/** Force first order low-pass filter smoothing factor */
	RealSeq filter;
	/** Force threshold */
	RealSeq threshold;
	/** Gain */
	RealSeq gain;

	/** Active mode operation state */
	Mode mode;
	/** Active mode operation on/off step */
	golem::I32 step;

	/** Force offset filtered */
	RealSeq forceInpOffset;
	/** Input force filtered */
	RealSeq forceInpCurrent;
	/** Input force at sensor */
	RealSeq forceInpSensor;
	/** Input force at sensor, sequence */
	std::vector<RealSeq> forceInpSensorSeq;
	/** Input force thresholded */
	RealSeq forceInp;
	/** Input force thresholded buffered */
	RealSeq forceInpBuff;
	/** Output torque */
	RealSeq forceOut;
	/** Output torque buffered */
	RealSeq forceOutBuff;

	/** Force reader */
	ForceReader forceReader;
	/** Action filter */
	ActionFilter actionFilter;

	/** Init timeout */
	golem::MSecTmU32 timeOut;

	/** Data access cs */
	golem::CriticalSection csCallback;
	golem::CriticalSection csData;

protected:
	golem::SingleCtrl& controller;

	/** the configuration space limits */
	golem::GenConfigspaceCoord min, max;
	
	/** Force (input) dimension */
	virtual size_t dimensions() const;
	/** Force input -> output transform */
	virtual void transform(const golem::Controller::State& state, const RealSeq& inp, RealSeq& out);
	
	/** SingleCtrl::CallbackIO: Data receive */
	virtual void sysRecv(golem::SingleCtrl* ctrl, golem::Controller::State& state);
	/** SingleCtrl::CallbackIO: Data send */
	virtual void sysSend(golem::SingleCtrl* ctrl, const golem::Controller::State& prev, golem::Controller::State& next, bool bSendPrev, bool bSendNext);

	/** Creates object from the description. */
	void create(const Desc& desc);
	/** Constructor */
	ActiveCtrlForce(golem::SingleCtrl& controller);

public:
	/** Descructor */
	virtual ~ActiveCtrlForce();

	/** Current mode */
	Mode getMode() const;
	/** Set current mode */
	void setMode(Mode mode, golem::I32 steps = STEP_DEFAULT);

	/** Input force */
	void getForceInp(RealSeq& force);
	/** Output torque */
	void getForceOut(RealSeq& force);
};

/** Reads/writes object from/to a given XML context */
void XMLData(ActiveCtrlForce::Desc &val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

/** Active manipulator controll with F/T sensor. */
class ActiveCtrlFT : public ActiveCtrlForce {
public:
	/** Object description */
	class Desc : public ActiveCtrlForce::Desc {
	public:
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}
		
		/** Sets the parameters to the default values */
		void setToDefault() {
			ActiveCtrlForce::Desc::setToDefault();
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			ActiveCtrlForce::Desc::assertValid(ac);
		}

		/** Creates the object from the description. */
		GOLEM_CREATE_FROM_OBJECT_DESC1(ActiveCtrlFT, ActiveCtrlForce::Ptr, golem::SingleCtrl&)
	};

protected:
	/** Jacobian */
	std::vector<golem::Twist> jacobian;

	/** Force (input) dimension */
	virtual size_t dimensions() const;
	/** Force input -> output transform */
	virtual void transform(const golem::Controller::State& state, const RealSeq& inp, RealSeq& out);

	/** Creates object from the description. */
	void create(const Desc& desc);
	/** Constructor */
	ActiveCtrlFT(golem::SingleCtrl& controller);
};

/** Reads/writes object from/to a given XML context */
void XMLData(ActiveCtrlFT::Desc &val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_ACTIVECTRL_ACTIVECTRL_ACTIVECTRL_FORCE_H_*/
