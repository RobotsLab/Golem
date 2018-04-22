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
//! @Date:     25/03/2014
//------------------------------------------------------------------------------
#pragma once
#ifndef _GOLEM_APP_POSEPLANNER_POSEPLANNER_H_
#define _GOLEM_APP_POSEPLANNER_POSEPLANNER_H_

//------------------------------------------------------------------------------

#include <Golem/HBPlanner/Data.h>
#include <Golem/HBPlanner/Heuristic.h>
#include <Golem/App/Player.h>
#include <Golem/Contact/Model.h>
#include <Golem/Plugin/Ctrl.h>
#include <Golem/Contact/Model.h>
#include <Golem/Contact/Query.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Application of HBPlanner for grasping under object-pose uncertainty. */
class GOLEM_LIBRARY_DECLDIR PosePlanner : public golem::Player {
public:
	/** Callback to handle a ground truth pose of the object */
	typedef std::function<void(const golem::Cloud::PointSeq&)> SimulateHandler;

	/** Action types */
	enum  action {
		NONE_ACTION = 0,
		GRASP,
		IG_PLAN_ON_QUERY,
		IG_PLAN_M2Q,
		IG_PLAN_LIFT,
		IG_TRAJ_OPT
	};
	/** Prints actions */
	static std::string actionToString(action type) {
		std::string str;
		switch (type) {
		case action::NONE_ACTION:
			str.assign("NONE");
			break;
		case action::GRASP:
			str.assign("GRASP");
			break;
		case action::IG_PLAN_M2Q:
			str.assign("IG_PLAN_M2Q");
			break;
		case action::IG_PLAN_ON_QUERY:
			str.assign("IG_PLAN_ON_QUERY");
			break;
		case action::IG_TRAJ_OPT:
			str.assign("IG_TRAJ_OPT");
			break;
		case action::IG_PLAN_LIFT:
			str.assign("IG_PLAN_LIFT");
			break;
		}

		return str;
	}

	/** Type of implemented algorithms */
	enum Strategy {
		NONE_STRATEGY = 0,
		ELEMENTARY,
		MYCROFT,
		IR3NE,
	};

	/** Data */
	class GOLEM_LIBRARY_DECLDIR Data : public golem::Player::Data {
	public:
		friend class PosePlanner;

		/** Mode */
		enum Mode {
			/** DEFAULT */
			MODE_DEFAULT,
			/** Model data */
			MODE_MODEL,
			/** Query density */
			MODE_QUERY,
		};

		/** Mode name */
		static const std::string ModeName[MODE_QUERY + 1];

		/** Data bundle description */
		class GOLEM_LIBRARY_DECLDIR Desc : public golem::Player::Data::Desc {
		public:
			/** Creates the object from the description. */
			virtual golem::data::Data::Ptr create(golem::Context &context) const;
		};

		/** Data bundle default name */
		std::string dataName;

		/** Current Mode */
		Mode mode;

		/** Query frame */
		golem::Mat34 modelFrame;
		/** Model points */
		golem::Cloud::PointSeq modelPoints;

		/** Query transformation */
		golem::Mat34 queryTransform;
		/** Query frame: queryTransform * modelFrame */
		golem::Mat34 queryFrame;
		/** Query points */
		golem::Cloud::PointSeq queryPoints;
		/** Simulated location of the object */
		golem::Cloud::PointSeq simulateObjectPose;

		/** Belief file name extension */
		std::string extSamples;

		/** type of action to execute */
		action actionType;

		/** Strategy to execute */
		Strategy stratType;

		/** Manager */
		virtual void setOwner(golem::Manager* owner);

		/** Creates render buffer of the bundle without items */
		virtual void createRender();

	protected:
		/** Data */
		PosePlanner* owner;

		/** Creates data bundle */
		void create(const Desc& desc);
		/** Creates data bundle */
		Data(golem::Context &context);
	};

	friend class Data;

	/** Pose planner description */
	class GOLEM_LIBRARY_DECLDIR Desc : public golem::Player::Desc {
	protected:
		GOLEM_CREATE_FROM_OBJECT_DESC1(PosePlanner, golem::Object::Ptr, golem::Scene&)

	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Data bundle default name */
		std::string dataName;

		/** Pose distribution */
		Belief::Desc::Ptr pBeliefDesc;

		/** Model pose estimation camera */
		std::string modelCamera;
		/** Model data handler (scan) */
		std::string modelHandlerScan;
		/** Model data handler */
		std::string modelHandler;
		/** Model data item */
		std::string modelItem;
		/** Model data item object */
		std::string modelItemObj;
		/** Model data handler */
		std::string modelGraspHandler;
		/** Model data item */
		std::string modelGraspItem;

		/** Model scan pose */
		golem::ConfigMat34::Seq modelScanPoseSeq;

		/** Model trajectory handler */
		std::string modelHandlerTrj;
		/** Model trajectory item */
		std::string modelItemTrj;

		/** Query pose estimation camera */
		std::string queryCamera;
		/** Model data handler (scan) */
		std::string queryHandlerScan;
		/** Query data handler */
		std::string queryHandler;
		/** Query data item */
		std::string queryItem;
		/** Query data item object */
		std::string queryItemObj;
		/** Query data handler */
		std::string queryGraspHandler;
		/** Query data item */
		std::string queryGraspItem;

		/** Object scan pose */
		golem::ConfigMat34::Seq queryScanPoseSeq;

		/** Query trajectory handler */
		std::string queryHandlerTrj;
		/** Query trajectory item */
		std::string queryItemTrj;

		/** Pose estimation for simulated object */
		golem::RBPose::Desc::Ptr simRBPoseDesc;
		/** Ground truth data item */
		std::string simulateItem;
		/** Ground truth data handler */
		std::string simulateHandler;

		/** Belief data handler */
		std::string beliefHandler;
		/** Belief data item */
		std::string beliefItem;

		/** Model descriptions */
		golem::Model::Desc::Map modelDescMap;
		/** Contact appearance */
		golem::Contact3D::Appearance contactAppearance;

		/** Query descriptions */
		golem::Query::Desc::Map queryDescMap;

		/** Grasp force sensor */
		std::string graspSensorForce;
		/** Grasp force threshold */
		golem::Twist graspThresholdForce;
		/** Grasp force event - hand close time wait */
		golem::SecTmReal graspEventTimeWait;
		/** Grasp hand close duration */
		golem::SecTmReal graspCloseDuration;
		/** Grasp pose open (pre-grasp) */
		golem::ConfigMat34 graspPoseOpen;
		/** Grasp pose closed (grasp) */
		golem::ConfigMat34 graspPoseClosed;

		/** Manipulator description */
		golem::Manipulator::Desc::Ptr manipulatorDesc;
		/** Manipulator Appearance */
		golem::Manipulator::Appearance manipulatorAppearance;

		/** Appereance for point clouds: hypothesis point clouds */
		golem::Cloud::Appearance hypothesisAppearance;
		/** Appereance for point clouds: debug point clouds */
		golem::Cloud::Appearance meanposeAppearance;
		/** Appereance for point clouds: ground truth point clouds */
		golem::Cloud::Appearance groundTruthAppearance;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Player::Desc::setToDefault();

			dataDesc.reset(new Data::Desc);
			dataName.clear();

			pBeliefDesc.reset(new Belief::Desc);

			modelCamera.clear();
			modelHandlerScan.clear();
			modelHandler.clear();
			modelItem.clear();
			modelItemObj.clear();
			modelGraspHandler.clear();
			modelGraspItem.clear();

			modelScanPoseSeq.clear();

			modelHandlerTrj.clear();
			modelItemTrj.clear();

			queryCamera.clear();
			queryHandlerScan.clear();
			queryHandler.clear();
			queryItem.clear();
			queryItemObj.clear();
			queryGraspHandler.clear();
			queryGraspItem.clear();

			queryScanPoseSeq.clear();

			queryHandlerTrj.clear();
			queryItemTrj.clear();

			simRBPoseDesc.reset(new golem::RBPose::Desc);
			simulateItem.clear();
			simulateHandler.clear();

			beliefHandler.clear();
			beliefItem.clear();

			modelDescMap.clear();
			contactAppearance.setToDefault();

			queryDescMap.clear();

			graspSensorForce.clear();
			graspThresholdForce.setZero();
			graspEventTimeWait = golem::SecTmReal(2.0);
			graspCloseDuration = golem::SecTmReal(2.0);
			graspPoseOpen.setToDefault();
			graspPoseClosed.setToDefault();

			manipulatorDesc.reset(new golem::Manipulator::Desc);
			manipulatorAppearance.setToDefault();

			hypothesisAppearance.setToDefault();
			meanposeAppearance.setToDefault();
			groundTruthAppearance.setToDefault();
		}
		/** Checks if the description is valid. */
		virtual void assertValid(const golem::Assert::Context& ac) const {
			Player::Desc::assertValid(ac);
			
			golem::Assert::valid(dataDesc != nullptr && golem::is<Data::Desc>(dataDesc.get()), ac, "dataDesc: unknown type");

			golem::Assert::valid(dataName.length() > 0, ac, "dataName: invalid");
			pBeliefDesc->assertValid(golem::Assert::Context(ac, "Belief desc: invalid"));

			golem::Assert::valid(manipulatorDesc != nullptr, ac, "manipulatorDesc: null");
			manipulatorDesc->assertValid(golem::Assert::Context(ac, "manipulatorDesc->"));
			manipulatorAppearance.assertValid(golem::Assert::Context(ac, "manipulatorAppearance."));
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
	};

	/** Reset belief state */
	inline void createBeliefState() {
		pBelief = myDesc.pBeliefDesc->create(context);  // throws
	}

protected:
	/** Generator of pseudo random numbers */
	golem::Rand rand;

	/** Descriptor file */
	Desc myDesc;

	/** Disable break points */
	bool silent;

	/** Appereance for point clouds: debug point clouds */
	golem::Cloud::Appearance hypothesisAppearance, meanposeAppeareance, groundtruthAppearance;
	bool showMeanPoseOnly, showSimulate;
	/** Callback to handle simulate ground truth */
	SimulateHandler simulateHandlerCallback;

	/** Force to draw the belief state */
	bool drawBeliefState;
	/** Pointer to the current belief state */
	golem::data::Item::Map::iterator currentBeliefPtr;
	/** Belief data handler */
	golem::data::Handler* beliefHandler;
	/** Belief data item */
	std::string beliefItem;
	/** Current belief data item */
	std::string currentBeliefItem;
	/** Pose distribution */
	Belief::Ptr pBelief; //golem::RBPose::Ptr pRBPose;

	/** Model pose estimation camera */
	golem::Camera* modelCamera;
	/** Model data handler (scan) */
	golem::data::Handler* modelHandlerScan;
	/** Model data handler */
	golem::data::Handler* modelHandler;
	/** Model data item */
	std::string modelItem;
	/** Model data item object */
	std::string modelItemObj;
	/** Model data handler */
	golem::data::Handler* modelGraspHandler;
	/** Model data item */
	std::string modelGraspItem;
	/** Model scan pose */
	golem::ConfigMat34::Seq modelScanPoseSeq;

	/** Model trajectory handler */
	golem::data::Handler* modelHandlerTrj;
	/** Model trajectory item */
	std::string modelItemTrj;

	/** Query pose estimation camera */
	golem::Camera* queryCamera;
	/** Query data handler (scan) */
	golem::data::Handler* queryHandlerScan;
	/** Query data handler */
	golem::data::Handler* queryHandler;
	/** Query data item */
	std::string queryItem;
	/** Query data item object */
	std::string queryItemObj;
	/** Query data handler */
	golem::data::Handler* queryGraspHandler;
	/** Query data item */
	std::string queryGraspItem;
	/** Query scan pose */
	golem::ConfigMat34::Seq queryScanPoseSeq;

	/** Query trajectory handler */
	golem::data::Handler* queryHandlerTrj;
	/** Query trajectory item */
	std::string queryItemTrj;

	/** Grasp force sensor */
	golem::FT* graspSensorForce;
	/** Grasp force threshold */
	golem::Twist graspThresholdForce;
	/** Grasp force event - hand close time wait */
	golem::SecTmReal graspEventTimeWait;
	/** Grasp hand close duration */
	golem::SecTmReal graspCloseDuration;
	/** Grasp pose open (pre-grasp) */
	golem::ConfigMat34 graspPoseOpen;
	/** Grasp pose closed (grasp) */
	golem::ConfigMat34 graspPoseClosed;

	/** Accurate pose estimation for the simulated object */
	golem::RBPose::Ptr simRBPose;
	/** Reference frames */
	golem::Mat34 simModelFrame, simQueryFrame;
	/** Query data item */
	std::string simulateItem;
	/** Ground truth data handler */
	golem::data::Handler* simulateHandler;

	/** Models */
	golem::Model::Map modelMap;
	/** Contact appearance */
	golem::Contact3D::Appearance contactAppearance;

	/** Query densities */
	golem::Query::Map queryMap;

	/** Manipulator */
	golem::Manipulator::Ptr manipulator;
	/** Manipulator Appearance */
	golem::Manipulator::Appearance manipulatorAppearance;

	/** Distribution num of samples */
	size_t distribSamples;

	/** Query views (random selected) */
	size_t queryViews;

	/** Model data */
	Data::Map::iterator modelDataPtr;
	/** Model frame */
	golem::Mat34 modelFrame;
	/** Model points */
	golem::Cloud::PointSeq modelPoints;

	golem::Bounds::Seq handBounds;
	golem::DebugRenderer debugRenderer;

	/** Smart pointer to the ft driven heuristic */
	golem::HBHeuristic* pHeuristic;

	/** Pose estimation */
	golem::data::Item::Map::iterator estimatePose(const Data::Mode mode, std::string &itemName);
	/** Grasp and capture object */
	golem::data::Item::Map::iterator objectCapture(const Data::Mode mode, std::string &itemName);
	/** Process object image and add to data bundle */
	golem::data::Item::Map::iterator objectProcess(const Data::Mode mode, golem::data::Item::Map::iterator ptr);
	/** Retrieve a point cloud from the data budle */
	golem::Cloud::PointSeq getPoints(Data::Map::iterator dataPtr, const std::string &itemName) const;

	/** Reset data pointers */
	void resetDataPointers();

	inline golem::Controller::State lookupState(golem::SecTmReal time = golem::SEC_TM_REAL_MAX) const {
		golem::Controller::State dflt = controller->createState();
		controller->setToDefault(dflt);

		controller->lookupState(time, dflt);
		dflt.cvel.setToDefault(info.getJoints());
		dflt.cacc.setToDefault(info.getJoints());
		return dflt;
	}

	inline golem::Controller::State lookupCommand(golem::SecTmReal time = golem::SEC_TM_REAL_MAX) const {
		golem::Controller::State dflt = controller->createState();
		controller->setToDefault(dflt);

		controller->lookupCommand(time, dflt);
		dflt.cvel.setToDefault(info.getJoints());
		dflt.cacc.setToDefault(info.getJoints());
		return dflt;
	}

	virtual void render() const;

	bool create(const Desc& desc);
	PosePlanner(golem::Scene &scene);
	~PosePlanner();
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_APP_POSEPLANNER_POSEPLANNER_H_*/
