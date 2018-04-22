//------------------------------------------------------------------------------
//               Simultaneous Perception And Manipulation (SPAM)
//------------------------------------------------------------------------------
//
// Copyright (c) 2010 University of Birmingham.
// All rights reserved.
//
// Intelligence Robot Lab.
// School of Computer Science
// University of Birmingham
// Edgbaston, Brimingham. UK.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy 
// of this software and associated documentation files (the "Software"), to deal 
// with the Software without restriction, including without limitation the 
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
// sell copies of the Software, and to permit persons to whom the Software is 
// furnished to do so, subject to the following conditions:
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimers.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimers in 
//       the documentation and/or other materials provided with the 
//       distribution.
//     * Neither the names of the Simultaneous Perception And Manipulation 
//		 (SPAM), University of Birmingham, nor the names of its contributors  
//       may be used to endorse or promote products derived from this Software 
//       without specific prior written permission.
//
// The software is provided "as is", without warranty of any kind, express or 
// implied, including but not limited to the warranties of merchantability, 
// fitness for a particular purpose and noninfringement.  In no event shall the 
// contributors or copyright holders be liable for any claim, damages or other 
// liability, whether in an action of contract, tort or otherwise, arising 
// from, out of or in connection with the software or the use of other dealings 
// with the software.
//
//------------------------------------------------------------------------------
//! @Author:   Claudio Zito
//! @Date:     27/10/2012
//------------------------------------------------------------------------------
#pragma once
#ifndef _GOLEM_APP_R2GPLANNER_R2GPLANNER_H_
#define _GOLEM_APP_R2GPLANNER_R2GPLANNER_H_

//------------------------------------------------------------------------------

#include <Golem/App/R2GPlanner/PosePlanner.h>
#include <Golem/App/R2GPlanner/Sensor.h>
#include <Golem/ActiveCtrl/ActiveTouchCtrl/ActiveTouchCtrl.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

//class SensorBundle : public golem::Runnable  {
//public:
//	/** Pointer */
//	typedef golem::shared_ptr<SensorBundle> Ptr;
//	/** Force reader. Overwrite the ActiveCtrl reader. this retrieves the triggered joints */
//	typedef std::function<std::vector<size_t>(golem::RealSeq&)> ForceReader;
//	/** Force reader. Overwrite the ActiveCtrl reader. this retrieves the triggered joints */
//	typedef std::function<void(golem::RealSeq&)> ForceFilter;
//	/** FT Sequence */
//	typedef std::vector<golem::FT*> FTSensorSeq;
//	/** Print FT values */
//	typedef std::function<void(std::ostream&, const golem::Twist&, const golem::SecTmReal)> StrFT;
//	/** Print header */
//	typedef std::function<void(std::ostream&)> StrFTDesc;
//
//	class Desc {
//	public:
//		/** Force reader */
//		ForceReader forceReader;
//		/** Collect history of FT readings for statistics */
//		ForceFilter collectInp;
//		/** FT 2-oreder high-pass filter */
//		ForceFilter forceFilter;
//
//		/** String function */
//		StrFT strFT;
//		StrFTDesc strFTDesc;
//		std::string path, sepField, ext;
//		bool write;
//
//		/** Read ft values from file */
//		bool read;
//
//		bool simulate;
//		/** Collision description file. Used for collision with the ground truth */
//		Collision::Desc::Ptr objCollisionDescPtr;
//
//		golem::Sensor::Seq sensorSeq;
//
//		/** Dimensionality of the filter (DoFs)*/
//		golem::U32 dimensionality;
//
//		/** Number of averaging steps before stopping collecting readings */
//		golem::U32 windowSize;
//		golem::I32 steps;
//		/** Incrememnt */
//		golem::Real delta;
//		/** gaussian filter mask */
//		golem::RealSeq mask;
//		/** Variance for the gaussian filter */
//		golem::Real sigma;
//
//		/** Working thread priority */
//		golem::Thread::Priority threadPriority;
//		/** Cycle time */
//		golem::SecTmReal tCycle;
//		/** Idle time */
//		golem::SecTmReal tIdle;
//	
//		Desc() {
//			setToDefault();
//		}
//		void setToDefault() {
//			forceReader = nullptr;
//			collectInp = nullptr;
//			forceFilter = nullptr;
//			strFT = [=](std::ostream& ostr, const golem::Twist& twist, const golem::SecTmReal t) {
//				ostr << t << "\t" << twist.v.x << "\t" << twist.v.y << "\t" << twist.v.z << "\t" << twist.w.x << "\t" << twist.w.y << "\t" << twist.w.z << std::endl;
//			};
//			strFTDesc = [=](std::ostream& ostr) {
//				ostr << "t" << "vx" << "\t" << "vy" << "\t" << "vz" << "\t" << "wx" << "\t" << "wy" << "\t" << "wz" << std::endl;
//			};
//			path = "./data/boris/experiments/ftsensors/";
//			sepField = "-";
//			ext = ".txt";
//			write = false;
//
//			read = false;
//
//			simulate = true;
//			objCollisionDescPtr.reset(new Collision::Desc());
//
//			sensorSeq.clear();
//
//			dimensionality = 18;
//
//			steps = 0;
//			windowSize = 40;
//			delta = .1;
//			mask.assign(windowSize, golem::REAL_ZERO);
//			sigma = golem::Real(2.65);
//
//			threadPriority = golem::Thread::HIGHEST;
//			tCycle = golem::SecTmReal(0.02);
//			tIdle = golem::SecTmReal(0.01);
//		}
//		virtual void assertValid(const golem::Assert::Context &ac) const {
//			golem::Assert::valid(objCollisionDescPtr != nullptr, ac, "Collision description: invalid");
//		}
//		/** Load descritpion from xml context. */
//		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
//
//		GRASP_CREATE_FROM_OBJECT_DESC1(SensorBundle, SensorBundle::Ptr, golem::Controller&)
//	};
//
//	/** Force reader */
//	ForceReader forceReader;
//	/** Collect history of FT readings for statistics */
//	ForceFilter collectInp;
//	/** FT 2-oreder high-pass filter */
//	ForceFilter forceFilter;
//
//	/** Set the sequence of sensors */
//	void setSensorSeq(const golem::Sensor::Seq& sensorSeq);
//
//	inline golem::RealSeq getFilteredForces() {
//		golem::CriticalSectionWrapper csw(cs);
//		return handFilteredForce;
//	}
//
//	inline void stop() {
//		bTerminate = true;
//		release();
//	}
//
//	inline void increment() {
//		if (idx < ft.size() - 1)
//			idx++;
//		else
//			context.write("End of trajectories! idx=%d ft.size()=%d\n", idx, ft.size());
//	}
//
//	inline void setManipulator(golem::Manipulator* manipulator) {
//		this->manipulator.reset(manipulator);
//		collisionPtr = desc.objCollisionDescPtr->create(*this->manipulator.get());
//	}
//
//	bool start2read;
//	///** Emergency mode handler */
//	//void setForceReaderHandler(ThreadTask::Function forceReaderHandler);
//
//	/** Pointer to collision detection with the ground truth */
//	Collision::Ptr collisionPtr;
//
//	/** Render */
//	void render() {
//		golem::CriticalSectionWrapper csw(csRender);
//		debugRenderer.render();
//	}
//
//protected:
//	/** Context */
//	golem::Context context;
//	/** Genrator of pseudo-random numbers */
//	golem::Rand rand;
//	/** Pointer to controller */
//	golem::Controller* controller;
//	/** Manipulator interface */
//	golem::Manipulator::Ptr manipulator;
//	/** Debug renderer */
//	golem::DebugRenderer debugRenderer;
//	/** Description file */
//	Desc desc;
//
//	/** String function */
//	StrFT strFT;
//	StrFTDesc strFTDesc;
//
//	/** Thread terminate condition */
//	volatile bool bTerminate;
//	/** I/O thread */
//	golem::Thread thread;
//	/** I/O thread priority */
//	golem::Thread::Priority threadPriority;
//	/** Inter-thread signalling time out */
//	golem::MSecTmU32 threadTimeOut;
//
//	/** File to collect data from the ft sensor of the hand */
////	std::vector<std::ofstream&> dataFTRawSeq, dataFTFilteredSeq, lowpassSeq;
//	bool write, read, simulate;
//	std::ifstream ftWristFile, ftThumbFile, ftIndexFile;
//	std::vector<golem::RealSeq> ftWrist, ftThumb, ftIndex, ft;
//	std::ofstream thumbSS, indexSS, middleSS, forceSS;
//
//	/** Time stamp of the reading forces */
//	golem::U32 idx;
//
//	/** Sequence of FT sensors */
//	FTSensorSeq ftSensorSeq;
//
//	/** Dimensionality of the filter (DoFs)*/
//	golem::U32 dimensionality;
//
//	/** Number of averaging steps before stopping collecting readings */
//	golem::U32 windowSize;
//	golem::I32 steps;
//	golem::Real delta;
//	// gaussian filter mask
//	golem::RealSeq mask;
//	/** Variance for the gaussian filter */
//	golem::Real sigma;
//
//	/** force reader access cs */
//	golem::CriticalSection cs, csRender;
//	/** Input force at sensor, sequence */
//	std::vector<golem::RealSeq> forceInpSensorSeq;
//	/** Filtered forces for the hand */
//	golem::RealSeq handFilteredForce;
//
//	// Return hand DOFs */
//	inline size_t dimensions() const { return dimensionality; }// (size_t)handInfo.getJoints().size();
//
//	///** Sersors read handler */
//	//ThreadTask::Function forceReaderHandler;
//	///** Force Reader handler thread */
//	//ThreadTask forceReaderHandlerThread;
//
//	template <typename _Real> inline _Real N(const _Real x, const _Real stdev) {
//		const _Real norm = golem::numeric_const<_Real>::ONE / (stdev*golem::Math::sqrt(2 * golem::numeric_const<_Real>::PI));
//		return norm*golem::Math::exp(-.5*golem::Math::sqr(_Real(x) / _Real(stdev))); // gaussian
//	};
//	// computes guassian on a vector
//	template <typename _Ptr, typename _Real> inline std::vector<_Real> N(_Ptr begin, _Ptr end, const size_t dim, const _Real stdev) {
//		std::vector<_Real> output;
//		output.assign(dim, golem::numeric_const<_Real>::ZERO);
//		size_t idx = 0;
//		for (_Ptr i = begin; i != end; ++i) {
//			output[idx++] = N(*i, stdev);
//		}
//		return output;
//	};
//
//	/** Communication function working in a separate thread. */
//	virtual void run();
//
//	/** Shuts down control thread and releases related resources */
//	void release();
//
//	/** User create is called just before launching I/O thread. */
//	virtual void userCreate() {}
//
//	/** I/O simulator timer */
//	golem::shared_ptr<golem::Sleep> sleep;
//	/** Time */
//	golem::SecTmReal tRead;
//	/** Cycle time */
//	golem::SecTmReal tCycle;
//	/** Idle time */
//	golem::SecTmReal tIdle;
//
//	SensorBundle(golem::Controller& ctrl);
//
//	void create(const Desc& desc);
//};

//------------------------------------------------------------------------------

/** 
	R2GPlanner. 
	Basic class for planning Reach-and-grasp trajectory.
	This class provides (1) a sampling method to generate a low dimension
	representation of uncertainty over object poses, and (2) a planner which
	builds trajectories that are more likely to distinguish between the 
	miximum likelihood pose of the object and any sampled poses.
	Based on Platt R. et al. "A hypothesis-based algorithm for planning and
	control in non-Gaussian belief spaces", 2011.
*/
class GOLEM_LIBRARY_DECLDIR R2GPlanner : public PosePlanner {
public:
	/** Force reader. Overwrite the ActiveCtrl reader. this retrieves the triggered joints */
	typedef std::function<void(const golem::Controller::State&, golem::RealSeq&, std::vector<golem::Configspace::Index>&)> GuardsReader;
	/** Force reader. Overwrite the ActiveCtrl reader. this retrieves the triggered joints */
	typedef std::function<std::vector<size_t>(const golem::Controller::State&, golem::RealSeq&)> GuardFTReader;
	/** FT Sequence */
	typedef std::vector<golem::FT*> FTSensorSeq;

	/** Data */
	class GOLEM_LIBRARY_DECLDIR Data : public PosePlanner::Data {
	public:
		friend class R2GPlanner;
		
		/** Data bundle description */
		class GOLEM_LIBRARY_DECLDIR Desc : public PosePlanner::Data::Desc {
		public:
			/** Creates the object from the description. */
			virtual golem::data::Data::Ptr create(golem::Context &context) const;
		};

		/** Specifies if guard have been triggered during the perform of the action */
		int triggered;

		/** Specifies if the replanning should be triggered */
		bool replanning;
		/** Enable the release of the object before withdrawing */
		bool release;

		/** Manager */
		virtual void setOwner(golem::Manager* owner);

		/** Creates render buffer of the bundle without items */
		virtual void createRender();

		/** Clone data */
		virtual Data* clone() const;

	protected:
		/** Demo */
		R2GPlanner* owner;

		/** Load from xml context */
		virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext, const golem::data::Handler::Map& handlerMap);
		/** Save to xml context */
		virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

		/** Creates data bundle */
		void create(const Desc& desc);
		/** Creates data bundle */
		Data(golem::Context &context);
	};

	class GOLEM_LIBRARY_DECLDIR Desc : public PosePlanner::Desc {
	protected:
		GOLEM_CREATE_FROM_OBJECT_DESC1(R2GPlanner, golem::Object::Ptr, golem::Scene&)

	public:
		/** Smart pointer */
		typedef golem::shared_ptr<Desc> Ptr;

		/** Enables/disables explicit use of uncertainty in planning */
		bool uncEnable;
		/** Enables/disables replanning */
		bool singleGrasp;
		/** Enables/disables withdrawing to the home pose */
		bool withdrawToHomePose;

		/** Downsampling parameter */
		size_t maxModelPoints;

		/** Collision description file. Used for collision with the ground truth */
		HBCollision::Desc::Ptr objCollisionDescPtr;

		/** Guards to retrieve a contact */
		golem::RealSeq fLimit;

		/** Active Ctrl */
		std::string activeCtrlStr;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			PosePlanner::Desc::setToDefault();

			dataDesc.reset(new Data::Desc);

			uncEnable = true;
			singleGrasp = false;
			withdrawToHomePose = false;

			fLimit.assign(20, golem::REAL_ZERO);

			maxModelPoints = 5000;

			//sensorBundleDesc.reset(new SensorBundle::Desc);
			activeCtrlStr = "ArmHandForce+ArmHandForce";
			objCollisionDescPtr.reset(new HBCollision::Desc());
		}

		/** Checks if the description is valid. */
		virtual void assertValid(const golem::Assert::Context &ac) const {
			PosePlanner::Desc::assertValid(ac);

			golem::Assert::valid(maxModelPoints > 0, ac, "Max model point is not positive");
			//sensorBundleDesc->assertValid(ac);
			golem::Assert::valid(objCollisionDescPtr != nullptr, ac, "Collision description: invalid");
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
	};

	inline golem::Real getFilterForce(const golem::I32 i) {
		return handFilteredForce[i];
	}
	inline golem::RealSeq getFilterForce() {
		return handFilteredForce;
	}

protected:
	/** Mode of Active Ctrl */
	golem::ActiveCtrlForce::Mode armMode, handMode;
	golem::FT* wristFTSensor;
	/** Sequence of FT sensors */
	FTSensorSeq ftSensorSeq;
	/** Pointer to the hand active controller */
	golem::ActiveTouchCtrl *armHandForce;
	/** Guards to retrieve a contact */
	golem::RealSeq fLimit;

	/** Collision used for collision with the ground truth */
	HBCollision::Ptr collisionPtr;
	/** Filtered forces for the hand */
	golem::RealSeq handFilteredForce;
	// Return hand DOFs */
	inline size_t dimensions() const { return 18; }// (size_t)handInfo.getJoints().size();

	/** Checks for reions with high likelihood of contacts */
	inline bool expectedCollisions(const golem::Controller::State& state) const {
		return pHeuristic->expectedCollisions(state);
	}

	/** Particles renderer */
	golem::DebugRenderer sampleRenderer;
	/** ground truth renderer */
	golem::DebugRenderer gtRenderer;

	/** Object pose data */
	Data::Map::iterator poseDataPtr;
	
	/** Description file */
	Desc ragDesc;

	/** Show original colour of the point cloud */
	bool showSampleColour;
	/** Show Sample frame */
	bool showSampleFrame;
	/** Show high dimension distribution */
	bool showDistribution;
	/** Show maximum likelihood estimation distribution */
	bool showMLEFrame;
	/** Show MLE points */
	bool showMLEPoints;
	/** Enables/disables esplicit use of uncertainty in planning */
	bool uncEnable;
	/** Enables/disables replanning */
	bool singleGrasp;
	/** Enables/disables withdrawing to the home pose */
	bool withdrawToHomePose;
	/** Shows the posterior distribution */
	bool posterior;

	/** Select index */
	template <typename _Seq, typename _Index> void selectIndex(const _Seq& seq, _Index& index, const std::string& name) {
		if (seq.empty())
            throw golem::Cancel("Empty!");
		// select index within the range
		std::stringstream str;
		golem::Math::clamp(index, (_Index)1, (_Index)seq.size());
		str << "Enter " << name << " index <1.." << seq.size() << ">: ";
		readNumber(str.str().c_str(), index);
		if (size_t(index) < 1 || size_t(index) > seq.size())
			throw golem::Cancel("Invalid index");
    };

	bool printing;

	/** Item to remove from data after a trial is saved */
	std::vector<std::string> itemPerformedTrj;

	/** Resets the controllers */
	bool enableControllers;
	/** Checks collision with the query point cloud */
	bool enableSimContact;
	/** Enables/disables force reading */
	bool enableForceReading;
	bool forcereadersilent;
	bool contactOccured;
	
	/** Structure for FT sensors */
	FTGuard::SeqPtr ftGuards;

	/** Iterations counter */
	size_t iterations;

	/** Object real point cloud (testing purposes) */
	golem::shared_ptr<golem::Cloud::PointSeq> objectPointCloudPtr;

	/** Updates belief state */
	void updateAndResample(Data::Map::iterator dataPtr);

	/** Finds a target in configuration space in a new reference frame */
	void findTarget(const golem::Mat34 &trn, const golem::Controller::State &target, golem::Controller::State &cend, const bool lifting = false);
	// Finds a grasp frame w.r.t. the world's reference frame
	void findTarget(const golem::Mat34& queryTrn, const golem::Mat34& modelFrame, const golem::Controller::State& target, golem::Controller::State& cend, const bool lifting = false);

	/** (Global search) trajectory of the entire robot from the configuration space and/or workspace target */
	virtual void createTrajectory(const golem::Controller::State& begin, const golem::Controller::State* pcend, const golem::Mat34* pwend, golem::SecTmReal t, const golem::Controller::State::Seq& waypoints, golem::Controller::State::Seq& trajectory);
	/** (Local search) trajectory of the arm only from a sequence of configuration space targets in a new reference frame */
	golem::RBDist trnTrajectory(const golem::Mat34& actionFrame, const golem::Mat34& modelFrame, const golem::Mat34& trn, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory);
	/** (Local search) trajectory of the arm only from a sequence of configuration space targets in a new reference frame */
	golem::RBDist findTrnTrajectory(const golem::Mat34& trn, const golem::Controller::State& startPose, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory);

	/** Perform trajectory */
	virtual void perform(const std::string& data, const std::string& item, const golem::Controller::State::Seq& trajectory, bool testTrajectory = true);
	/** Plan and execute r2g, grasp, lift operations */
	bool execute(golem::data::Data::Map::iterator dataPtr, golem::WaypointCtrl::Seq& trajectory);

	virtual void render() const;
	void renderHand(const golem::Controller::State &state, const golem::Bounds::Seq &bounds, bool clear = true);

	R2GPlanner(golem::Scene &scene);
	~R2GPlanner();
	bool create(const Desc& desc);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /* _GOLEM_APP_R2GPLANNER_R2GPLANNER_H_*/
