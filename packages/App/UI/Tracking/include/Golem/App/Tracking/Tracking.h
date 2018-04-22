/** @file Tracking.h
 *
 * Grasp tracking
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
#ifndef _GOLEM_APP_TRACKING_TRACKING_H_ // if #pragma once is not supported
#define _GOLEM_APP_TRACKING_TRACKING_H_

#include <Golem/App/Player.h>
#include <Golem/Tools/Cloud.h>
#include <Golem/Contact/Contact.h>
#include <Golem/Planner/GraphPlanner/DEKinematics.h>

/** Grasp name space */
namespace golem {

//------------------------------------------------------------------------------

/** Tracking demo. */
class AppTracking : public golem::Player {
public:
	/** Inverse kinematic solution */
	class Solution {
	public:
		typedef std::vector<Solution> Seq;

		/** Graph waypoint pointer */
		golem::Waypoint::Seq::const_iterator waypoint;
		
		/** SE(3) distance between grasp trajectory waypoint and graph waypoint */
		golem::RBDist wdist;
		/** Weighted distance */
		golem::Real dist;

		/** Distance comparison */
		inline bool operator < (const Solution &solution) const {
			return dist < solution.dist;
		}
	};

	/** Grasp selection */
	class Grasp {
	public:
		typedef std::map<Contact::Config::Seq::const_iterator, Grasp> Map;

		/** Target path */
		Configuration::Path targetPath;
		/** Target pose */
		golem::RBCoord targetFrame;

		/** Solutions */
		Solution::Seq solutions;

		/** Best solution frame */
		golem::Mat34 frame;
		/** Best solution coordinates */
		golem::ConfigspaceCoord coord;

		/** SE(3) distance between grasp trajectory waypoint and graph waypoint */
		golem::RBDist wdist;
		/** Configuration space distance between given manipulator configuration and graph waypoint */
		golem::Real cdist;
		/** Weighted distance */
		golem::Real dist;
	};

	/** Trajectory profile */
	class Profile : public golem::Profile, public golem::Profile::CallbackDist {
		const PlannerInfo& plannerInfo;
		const golem::ConfigspaceCoord distance;
		virtual golem::Real distConfigspaceCoord(const golem::ConfigspaceCoord& prev, const golem::ConfigspaceCoord& next) const {
			golem::Real dist = golem::REAL_ZERO;
			for (golem::Configspace::Index i = plannerInfo.armInfo.getJoints().begin(); i < plannerInfo.armInfo.getJoints().end(); ++i)
				dist += distance[i] * golem::Math::sqr(prev[i] - next[i]);
			for (golem::Configspace::Index i = plannerInfo.handInfo.getJoints().begin(); i < plannerInfo.handInfo.getJoints().end(); ++i)
				dist += distance[i] * golem::Math::sqr(prev[i] - next[i]);
			return golem::Math::sqrt(dist);
		}
		virtual golem::Real distCoord(golem::Real prev, golem::Real next) const { return golem::Math::abs(prev - next); }
		virtual bool distCoordPlanning(const golem::Configspace::Index& index) const { return plannerInfo.armInfo.getJoints().contains(index) || plannerInfo.handInfo.getJoints().contains(index); }
		virtual bool distCoordInterpolation(const golem::Configspace::Index& index) const { return plannerInfo.handInfo.getJoints().contains(index); }
	public:
		Profile(const PlannerInfo& plannerInfo, const golem::Profile::Desc& desc, const golem::ConfigspaceCoord& distance) : golem::Profile(plannerInfo.planner->getController()), plannerInfo(plannerInfo), distance(distance) {
			const_cast<golem::Profile::Desc&>(desc).pCallbackDist = this;
			golem::Profile::create(desc);
		}
	};

	/** Grasp demo description */
	class Desc : public golem::Player::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Depth camera */
		std::string camera;
		/** Depth camera used for tracking */
		std::string cameraTracking;

		/** Data bundle default name */
		std::string bundle;
		/** Data bundle default name */
		std::string bundleLog;

		/** Raw image handler */
		std::string imageHandler;
		/** Raw image item */
		std::string imageItem;

		/** Processed image/feature handler */
		std::string processHandler;
		/** Processed image/feature item */
		std::string processItem;
		/** Processed image/feature break point */
		bool processBreak;

		/** Contact model handler */
		std::string modelHandler;
		/** Contact model item */
		std::string modelItem;

		/** Contact query handler */
		std::string queryHandler;
		/** Contact query item */
		std::string queryItem;

		/** Log handler */
		std::string logHandler;
		/** Log item */
		std::string logItem;

		/** Video handler */
		std::string videoHandler;
		/** Video item */
		std::string videoItem;

		/** Scan poses */
		ConfigMat34::Seq poseScanSeq;
		/** Region in global coordinates */
		golem::Bounds::Desc::Seq scanRegionDesc;

		/** Number of threads for tracking */
		unsigned int trackingThreads;

		/** Manipulator description */
		Manipulator::Desc::Ptr manipulatorDesc;

		/** Kinematics solver description */
		golem::DEKinematics::Desc::Ptr kinematicsDesc;
		/** Number of threads in Parallels */
		golem::U32 kinematicsThreadParallels;
		/** Parallels thread joint time out */
		golem::MSecTmU32 kinematicsThreadTimeOut;

		/** Trajectory profile */
		golem::Profile::Desc profileDesc;
		/** Trajectory profile configspace distance multiplier */
		RealSeq distance;
		/** Trajectory grip waypoint configspace increment */
		RealSeq grip;

		/** Control distance threshold */
		golem::Real ctrlDistThr;

		/** Control inverse kinematics period of time */
		golem::Real ctrlIKin;
		/** Control reaction period of time */
		golem::Real ctrlReac;
		/** Control prediction period of time */
		golem::Real ctrlPred;
		/** Control grasp execution period of time */
		golem::Real ctrlExec;

		/** Manipulator bounds appearance */
		Manipulator::BoundsAppearance appearanceBounds;
		/** Manipulator solution appearance */
		Manipulator::BoundsAppearance appearanceSolution;
		/** Manipulator selection appearance */
		Manipulator::BoundsAppearance appearanceSelection;

		/** Object point size */
		golem::Real objectPointSize;
		/** Object point colour */
		golem::RGBA objectPointColour;
		/** Object bounding box solid colour */
		golem::RGBA objectSolidColour;
		/** Object bounding box wireframe colour */
		golem::RGBA objectWireColour;
		/** Object tracking point colour */
		golem::RGBA objectTrackingColour;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			golem::Player::Desc::setToDefault();

			camera.clear();
			cameraTracking.clear();
			bundle.clear();
			bundleLog.clear();
			imageHandler.clear();
			imageItem.clear();
			processHandler.clear();
			processItem.clear();
			processBreak = false;
			modelHandler.clear();
			modelItem.clear();
			queryHandler.clear();
			queryItem.clear();
			logHandler.clear();
			logItem.clear();
			videoHandler.clear();
			videoItem.clear();

			poseScanSeq.clear();
			scanRegionDesc.clear();

			trackingThreads = 0;

			manipulatorDesc.reset(new Manipulator::Desc);

			kinematicsDesc.reset(new golem::DEKinematics::Desc);
			kinematicsThreadParallels = 0;
			kinematicsThreadTimeOut = 5000;

			profileDesc.setToDefault();
			distance.assign(golem::Configspace::DIM, golem::REAL_ONE);
			grip.assign(golem::Configspace::DIM, golem::REAL_ONE);

			ctrlDistThr = golem::Real(0.2);

			ctrlIKin = golem::Real(0.05);
			ctrlReac = golem::Real(0.1);
			ctrlPred = golem::Real(0.15);
			ctrlExec = golem::Real(1.0);

			appearanceBounds.setToDefault();
			appearanceSolution.setToDefault();
			appearanceSelection.setToDefault();

			objectPointSize = golem::Real(3.0);
			objectPointColour = golem::RGBA::BLACK;
			objectSolidColour = golem::RGBA(0, 0, 255, 64);
			objectWireColour = golem::RGBA(0, 0, 255, 255);
			objectTrackingColour = golem::RGBA(0, 255, 0, 255);
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const golem::Assert::Context& ac) const {
			golem::Player::Desc::assertValid(ac);

			golem::Assert::valid(camera.length() > 0, ac, "camera: invalid");
			golem::Assert::valid(cameraTracking.length() > 0, ac, "cameraTracking: invalid");
			golem::Assert::valid(bundle.length() > 0, ac, "bundle: invalid");
			golem::Assert::valid(bundleLog.length() > 0, ac, "bundleLog: invalid");
			golem::Assert::valid(imageHandler.length() > 0, ac, "imageHandler: invalid");
			golem::Assert::valid(imageItem.length() > 0, ac, "imageItem: invalid");
			golem::Assert::valid(processHandler.length() > 0, ac, "processHandler: invalid");
			golem::Assert::valid(processItem.length() > 0, ac, "processItem: invalid");
			golem::Assert::valid(modelHandler.length() > 0, ac, "modelHandler: invalid");
			golem::Assert::valid(modelItem.length() > 0, ac, "modelItem: invalid");
			golem::Assert::valid(queryHandler.length() > 0, ac, "queryHandler: invalid");
			golem::Assert::valid(queryItem.length() > 0, ac, "queryItem: invalid");
			golem::Assert::valid(logHandler.length() > 0, ac, "logHandler: invalid");
			golem::Assert::valid(logItem.length() > 0, ac, "logItem: invalid");
			golem::Assert::valid(videoHandler.length() > 0, ac, "videoHandler: invalid");
			golem::Assert::valid(videoItem.length() > 0, ac, "videoItem: invalid");

			golem::Assert::valid(!poseScanSeq.empty(), ac, "poseScanSeq: empty");
			for (ConfigMat34::Seq::const_iterator i = poseScanSeq.begin(); i != poseScanSeq.end(); ++i)
				i->assertValid(golem::Assert::Context(ac, "poseScanSeq[]."));
			for (golem::Bounds::Desc::Seq::const_iterator i = scanRegionDesc.begin(); i != scanRegionDesc.end(); ++i)
				Assert::valid((*i)->isValid(), ac, "scanRegionDesc[]: invalid");

			Assert::valid(manipulatorDesc != nullptr, ac, "manipulatorDesc: null");
			manipulatorDesc->assertValid(Assert::Context(ac, "manipulatorDesc->"));
			Assert::valid(kinematicsDesc != nullptr && kinematicsDesc->isValid(), ac, "kinematicsDesc: invalid");
			golem::Assert::valid(kinematicsThreadParallels > 0, ac, "kinematicsThreadParallels: zero");
			golem::Assert::valid(kinematicsThreadTimeOut > 0, ac, "kinematicsThreadTimeOut: zero");

			Assert::valid(profileDesc.isValid(), ac, "profileDesc: invalid");
			Assert::valid(!distance.empty(), ac, "distance: invalid");
			for (RealSeq::const_iterator i = distance.begin(); i != distance.end(); ++i)
				Assert::valid(*i >= golem::REAL_ZERO, ac, "distance[i] < 0");
			Assert::valid(!grip.empty(), ac, "grip: invalid");
			for (RealSeq::const_iterator i = grip.begin(); i != grip.end(); ++i)
				Assert::valid(golem::Math::isFinite(*i), ac, "grip[i]: invalid");

			Assert::valid(ctrlDistThr > golem::REAL_EPS, ac, "ctrlDistThr: < eps");

			Assert::valid(ctrlIKin > golem::REAL_EPS, ac, "ctrlIKin: < eps");
			Assert::valid(ctrlReac > golem::REAL_EPS, ac, "ctrlReac: < eps");
			Assert::valid(ctrlPred > golem::REAL_EPS, ac, "ctrlPred: < eps");
			Assert::valid(ctrlExec > golem::REAL_EPS, ac, "ctrlExec: < eps");

			appearanceBounds.assertValid(Assert::Context(ac, "appearanceBounds."));
			appearanceSolution.assertValid(Assert::Context(ac, "appearanceSolution."));
			appearanceSelection.assertValid(Assert::Context(ac, "appearanceSelection."));

			golem::Assert::valid(objectPointSize >= golem::REAL_ZERO, ac, "objectPointSize: < 0");
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GOLEM_CREATE_FROM_OBJECT_DESC1(AppTracking, golem::Object::Ptr, golem::Scene&)
	};

protected:
	/** Depth camera */
	golem::CameraDepth* camera;
	/** Depth camera used for tracking */
	golem::CameraDepth* cameraTracking;

	/** Data bundle default name */
	std::string bundle;
	/** Data log bundle default name */
	std::string bundleLog;

	/** Raw image handler */
	golem::data::Handler* imageHandler;
	/** Raw image item */
	std::string imageItem;

	/** Processed image/feature handler */
	golem::data::Handler* processHandler;
	/** Processed image/feature item */
	std::string processItem;
	/** Processed image/feature break point */
	bool processBreak;

	/** Contact model handler */
	golem::data::Handler* modelHandler;
	/** Contact model item */
	std::string modelItem;

	/** Contact query handler */
	golem::data::Handler* queryHandler;
	/** Contact query item */
	std::string queryItem;

	/** Log handler */
	golem::data::Handler* logHandler;
	/** Log item */
	std::string logItem;

	/** Video handler */
	golem::Camera* videoHandler;
	/** Video item */
	std::string videoItem;

	/** Scan poses */
	ConfigMat34::Seq poseScanSeq;
	/** Region in global coordinates */
	golem::Bounds::Seq scanRegion;

	/** Number of threads for tracking */
	unsigned int trackingThreads;

	/** Manipulator */
	Manipulator::Ptr manipulator;

	/** Kinematics solver */
	golem::DEKinematics::Ptr kinematics;
	/** Kinematics parallels */
	golem::shared_ptr<golem::Parallels> parallels;
	/** Parallels thread joint time out */
	golem::MSecTmU32 kinematicsThreadTimeOut;

	/** Trajectory profile */
	golem::Profile::Desc profileDesc;
	/** Trajectory profile configspace distance multiplier */
	golem::ConfigspaceCoord distance;
	/** Trajectory grip waypoint configspace increment */
	golem::ConfigspaceCoord grip;

	/** Control distance threshold */
	golem::Real ctrlDistThr;

	/** Control inverse kinematics period of time */
	golem::Real ctrlIKin;
	/** Control reaction period of time */
	golem::Real ctrlReac;
	/** Control prediction period of time */
	golem::Real ctrlPred;
	/** Control grasp execution period of time */
	golem::Real ctrlExec;

	/** Manipulator bounds appearance */
	Manipulator::BoundsAppearance appearanceBounds;
	/** Manipulator solution appearance */
	Manipulator::BoundsAppearance appearanceSolution;
	/** Manipulator selection appearance */
	Manipulator::BoundsAppearance appearanceSelection;

	/** Object point size */
	golem::Real objectPointSize;
	/** Object point colour */
	golem::RGBA objectPointColour;
	/** Object bounding box solid colour */
	golem::RGBA objectSolidColour;
	/** Object bounding box wireframe colour */
	golem::RGBA objectWireColour;
	/** Object tracking colour */
	golem::RGBA objectTrackingColour;

	/** Demo renderer */
	golem::DebugRenderer demoRenderer;
	/** Demo renderer: configs */
	golem::DebugRenderer demoRendererConfigs;
	/** Demo renderer: object */
	golem::DebugRenderer demoRendererObject;

	/** golem::UIRenderer interface */
	virtual void render() const;

	void create(const Desc& desc);
	AppTracking(golem::Scene &scene);
	~AppTracking();
};

//------------------------------------------------------------------------------

};

#endif // _GOLEM_APP_TRACKING_TRACKING_H_