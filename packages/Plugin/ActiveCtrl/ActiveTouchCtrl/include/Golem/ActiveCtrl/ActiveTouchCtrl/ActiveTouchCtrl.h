/** @file ActiveTouchCtrl.h
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
#ifndef _GOLEM_ACTIVECTRL_TOUCH_CTRL_H_
#define _GOLEM_ACTIVECTRL_TOUCH_CTRL_H_

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

/** ActiveTouchCtrl is the interafce to a robot (right arm + hand), sensing devices and objects. */
class GOLEM_LIBRARY_DECLDIR ActiveTouchCtrl : public ActiveCtrl, public UI {
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
	
	class GOLEM_LIBRARY_DECLDIR Filter {
	public:
		/** Pointer */
		typedef golem::shared_ptr<Filter> Ptr;

		class GOLEM_LIBRARY_DECLDIR Desc {
		public:
			/** Pointer */
			typedef golem::shared_ptr<Desc> Ptr;

			/** Dimensionality of the filter (DoFs)*/
			golem::U32 dimensionality;
			/** Size of reading collected */
			golem::U32 windowSize;
			/** Initial step */
			golem::I32 steps;
			/** Incrememnt */
			golem::Real delta;
			/** gaussian filter mask */
			golem::RealSeq mask;
			/** Variance for the gaussian filter */
			golem::Real sigma;

			Desc() {
				setToDefault();
			}

			void setToDefault() {
				dimensionality = 18;
				steps = 0;
				windowSize = 40;
				delta = .1;
				mask.assign(windowSize, golem::REAL_ZERO);
				sigma = golem::Real(2.65);
			}

			virtual void assertValid(const golem::Assert::Context &ac) const {
				golem::Assert::valid(dimensionality > golem::U32(0), ac, "Sensor Filter description: invalid dimensionality");
				golem::Assert::valid(windowSize > golem::U32(0), ac, "Force Filter description: invalid window size");
				golem::Assert::valid(delta > golem::REAL_ZERO, ac, "Force Filter description: invalid sigma");
				golem::Assert::valid(sigma > golem::REAL_ZERO, ac, "Force Filter description: invalid sigma");
			}
			/** Load descritpion from xml context. */
			virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
			/** Creates the object from the description. */
			GOLEM_CREATE_FROM_OBJECT_DESC1(Filter, Filter::Ptr, golem::Context&)
		};

		/* Return hand DOFs */
		virtual inline size_t dimensions() const { return dimensionality; }// (size_t)handInfo.getJoints().size();
		/** Filter implementation */
		virtual golem::RealSeq filter(const golem::RealSeq& force);

	protected:
		/** Context */
		golem::Context context;
		/** Critical sections */
		golem::CriticalSection cs, csInp;

		/** Dimensionality of the filter (DoFs)*/
		golem::U32 dimensionality;
		/** Number of averaging steps before stopping collecting readings */
		golem::U32 windowSize;
		golem::I32 steps;
		golem::Real delta;
		// gaussian filter mask
		golem::RealSeq mask;
		/** Variance for the gaussian filter */
		golem::Real sigma;

		/** Input force at sensor, sequence */
		std::vector<golem::RealSeq> forceInpSensorSeq;

		template <typename _Real> inline _Real N(const _Real x, const _Real stdev) {
			const _Real norm = golem::numeric_const<_Real>::ONE / (stdev*golem::Math::sqrt(2 * golem::numeric_const<_Real>::PI));
			return norm*golem::Math::exp(-.5*golem::Math::sqr(_Real(x) / _Real(stdev))); // gaussian
		};
		// computes guassian on a vector
		template <typename _Ptr, typename _Real> inline std::vector<_Real> N(_Ptr begin, _Ptr end, const size_t dim, const _Real stdev) {
			std::vector<_Real> output;
			output.assign(dim, golem::numeric_const<_Real>::ZERO);
			size_t idx = 0;
			for (_Ptr i = begin; i != end; ++i) {
				output[idx++] = N(*i, stdev);
			}
			return output;
		};

		/** Create from description */
		void create(const Desc& desc);
		/** C'tor */
		Filter(golem::Context& context) : context(context) {};
	};

	class GOLEM_LIBRARY_DECLDIR SensorReader : public golem::Runnable {
	public:
		friend class ActiveTouchCtrl;
		/** Pointer */
		typedef golem::shared_ptr<SensorReader> Ptr;

		class GOLEM_LIBRARY_DECLDIR Desc {
		public:
			/** Pointer */
			typedef golem::shared_ptr<Desc> Ptr;

			/** Simulated Arm/hand force reader */
			ActiveCtrlForce::ForceReader forceReader;
			/** Filter description file */
			Filter::Desc::Ptr filterDescPtr;

			/** Working thread priority */
			golem::Thread::Priority threadPriority;
			/** Cycle time */
			golem::SecTmReal tCycle;
			/** Idle time */
			golem::SecTmReal tIdle;

			Desc() {
				setToDefault();
			}

			void setToDefault() {
				forceReader = nullptr;
				filterDescPtr.reset(new Filter::Desc);

				threadPriority = golem::Thread::HIGHEST;
				tCycle = golem::SecTmReal(0.02);
				tIdle = golem::SecTmReal(0.01);
			}

			virtual void assertValid(const golem::Assert::Context &ac) const {
				Assert::valid(filterDescPtr != nullptr, ac, "filterDescPtr == NULL");
				filterDescPtr->assertValid(ac);
			}
			/** Load descritpion from xml context. */
			virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
			/** Creates the object from the description. */
			GOLEM_CREATE_FROM_OBJECT_DESC2(SensorReader, SensorReader::Ptr, golem::Context&, golem::SingleCtrl&)
		};

		/** Read buffer */
		void readForce(const golem::Controller::State& state, RealSeq& force) {
			{
				golem::CriticalSectionWrapper csw(csData);
				force = filteredForce;
			}
		}

		/** Stop thread and release resources */
		inline void stop() {
			bTerminate = true;
			release();
		}

	protected:
		/** Context */
		golem::Context context;
		/** Controller */
		golem::SingleCtrl ctrl;
		/** force reader access cs */
		golem::CriticalSection csData;
		/** Filtered forces for the hand */
		golem::RealSeq filteredForce;
		/** Description file */
		Desc desc;

		/** Filter */
		Filter::Ptr filter;

		/** Thread terminate condition */
		volatile bool bTerminate;
		/** I/O thread */
		golem::Thread thread;
		/** I/O thread priority */
		golem::Thread::Priority threadPriority;
		/** Inter-thread signalling time out */
		golem::MSecTmU32 threadTimeOut;

		/** I/O simulator timer */
		golem::shared_ptr<golem::Sleep> sleep;
		/** Time */
		golem::SecTmReal tRead;
		/** Cycle time */
		golem::SecTmReal tCycle;
		/** Idle time */
		golem::SecTmReal tIdle;

		/** Communication function working in a separate thread. */
		virtual void run();

		/** Shuts down control thread and releases related resources */
		void release();

		/** User create is called just before launching I/O thread. */
		virtual void userCreate() {};

		/** Create from description */
		void create(const Desc& desc);
		/** C'tor */
		SensorReader(golem::Context& context, golem::SingleCtrl& ctrl) : context(context), ctrl(ctrl) {};
	};

	/** ActiveTouchCtrl factory */
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
		/** Force display scaling factor */
		golem::Real forceDispScale;

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
		/** Simulated hand force reader */
		SensorReader::Desc::Ptr sensorForceReaderDesc;

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

			forceDispScale = golem::Real(0.2);

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
			sensorForceReaderDesc.reset(new SensorReader::Desc);
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			ActiveCtrl::Desc::assertValid(ac);

			Assert::valid(armCtrlDesc != nullptr, ac, "armCtrlDesc == NULL");
			armCtrlDesc->assertValid(Assert::Context(ac, "armCtrlDesc->"));
			Assert::valid(handCtrlDesc != nullptr, ac, "handCtrlDesc == NULL");
			handCtrlDesc->assertValid(Assert::Context(ac, "handCtrlDesc->"));

			Assert::valid(ftGain.isValid(), ac, "ftGain: invalid");
			Assert::valid(ftLimit.isPositive(), ac, "ftLimit <= 0");

			for (RealSeq::const_iterator i = fLimit.begin(); i != fLimit.end(); ++i)
				Assert::valid(golem::Math::isPositive(*i), ac, "fLimit[i] <= 0");

			Assert::valid(forceDispScale > golem::REAL_ZERO, ac, "forceDispScale <= 0");
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

			Assert::valid(sensorForceReaderDesc != nullptr, ac, "simHandForceReaderDesc == NULL");
			sensorForceReaderDesc->assertValid(ac);
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		GOLEM_CREATE_FROM_OBJECT_DESC2(ActiveTouchCtrl, ActiveCtrl::Ptr, golem::Planner&, const Sensor::Seq&)
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
	void setArmForceReader(ActiveCtrlForce::ForceReader armForceReader);
	/** Hand force reader */
	void setHandForceReader(ActiveCtrlForce::ForceReader handForceReader);
	/** Simulated Hand force reader */
	void setSensorForceReader(ActiveCtrlForce::ForceReader sensorForceReader);
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

	/** Single control for the hand */
	const golem::SingleCtrl* getHand() const {
		return hand;
	}

protected:
	/** ActiveTouchCtrl arm */
	golem::SingleCtrl *arm;
	/** ActiveTouchCtrl hand */
	golem::SingleCtrl *hand;
	/** Force and torque sensor interface */
	golem::FT* ftSensor;

	/** Controller state info */
	golem::Controller::State::Info armInfo;
	/** Controller state info */
	golem::Controller::State::Info handInfo;
	
	/** Simulated hand force reader */
	SensorReader::Ptr sensorForceReader;

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
	RealSeq armForce;
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
	/** Force display scaling factor */
	golem::Real forceDispScale;
	/** Arm joint torques */
	mutable RealSeq armJointTorques;
	/** Hand joint torques */
	mutable RealSeq handJointTorques;

	/** Render data */
	mutable golem::DebugRenderer renderer;

	/** Key */
	int key;

	/** Force (input) dimension */
	virtual size_t dimensions() const;

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
	ActiveTouchCtrl(golem::Planner &planner, const Sensor::Seq& sensorSeq);
	/** Cleaning */
	~ActiveTouchCtrl();
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_ACTIVECTRL_TOUCH_CTRL_H_*/
