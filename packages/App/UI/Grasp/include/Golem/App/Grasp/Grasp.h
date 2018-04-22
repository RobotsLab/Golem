/** @file Grasp.h
 *
 * Demo Grasp
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
#ifndef _GOLEM_APP_GRASP_GRASP_H_ // if #pragma once is not supported
#define _GOLEM_APP_GRASP_GRASP_H_

#include <Golem/App/Player.h>
#include <Golem/Tools/Cloud.h>
#include <Golem/Contact/Data.h>
#ifdef _GOLEM_APP_GRASP_INTEROP_
#include <Golem/App/Grasp/GraspInterop.h>
#include <Golem/Tools/Image.h>
#endif // _GOLEM_APP_GRASP_INTEROP_

/** Golem namespace */
namespace golem {

//------------------------------------------------------------------------------

/** Grasp demo. */
class AppGrasp : public golem::Player {
public:
	/** Robot pose */
	class Pose : public golem::ConfigMat34 {
	public:
		typedef std::vector<Pose> Seq;

		/** Duration */
		golem::SecTmReal dt;
		/** Flags */
		std::string flags;

		/** Configspace type */
		bool configspace;
		/** Workspace position and orientation type */
		bool position, orientation;

		Pose() {
			setToDefault();
		}
		Pose(const RealSeq& c) : ConfigMat34(c), configspace(true), position(false), orientation(false), dt(golem::SEC_TM_REAL_ZERO) {
			w.setToDefault();
		}
		Pose(const RealSeq& c, const golem::Mat34& w) : ConfigMat34(c, w), configspace(true), position(true), orientation(true), dt(golem::SEC_TM_REAL_ZERO) {
		}

		void setToDefault() {
			ConfigMat34::setToDefault();
			dt = golem::SEC_TM_REAL_ZERO;
			flags.clear();
			configspace = true;
			position = orientation = false;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			ConfigMat34::assertValid(ac);
			Assert::valid(dt >= golem::SEC_TM_REAL_ZERO, ac, "dt: negative");
		}
	};

	/** Trajectory bundle */
	class Bundle {
	public:
		typedef std::vector<Bundle> Seq;

		/** Name */
		std::string path;
		/** Image */
		std::string image;
		/** Trajectory */
		std::string trajectory;

		Bundle() {
			setToDefault();
		}

		void setToDefault() {
			path.clear();
			image.clear();
			trajectory.clear();
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(!path.empty(), ac, "path: invalid");
		}
	};

	/** Interop description */
	class InteropDesc {
	public:
		/** enabled */
		bool enabled;

		/** enabled input cloud (GRASP_INTEROP_INPUT_CLOUD) */
		bool enabledInputCloud;
		/** enabled training (GRASP_INTEROP_TRAINING) */
		bool enabledTraining;
		/** enabled inference (GRASP_INTEROP_INFERENCE) */
		bool enabledInference;

		/** override breaks in the processing loop */
		bool overrideBreak;

		/** server host name */
		std::string host;
		/** server host port */
		unsigned short port;

		/** Set to default */
		InteropDesc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			enabled = false;
			enabledInputCloud = true;
			enabledTraining = false;
			enabledInference = true;
			overrideBreak = false;
			host = "localhost";
			port = 26783;
		}

		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(host.length() > 0, ac, "host: empty");
			Assert::valid(port > 0, ac, "port: invalid");

			Assert::valid(!enabled || (enabledInputCloud || enabledTraining || enabledInference), ac, "enabled: one of interop flags must be set");
		}

		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Grasp demo description */
	class Desc : public golem::Player::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Depth camera */
		std::string camera;

		/** Data bundle default path */
		std::string bundle;

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
		/** Processed query break point */
		bool queryBreak;

		/** Trajectory handler */
		std::string trjHandler;
		/** Trajectory item */
		std::string trjItem;
		/** Processed trajectory autonomous perform */
		bool trjAutoPerf;
		/** Processed trajectory merge */
		bool trjMerge;
		/** Processed trajectory break point */
		bool trjBreak;

		/** Filtering algortihm descriptiton */
		golem::Cloud::FilterDesc detectFilterDesc;
		/** Filtering algortihm descriptiton */
		golem::U32 detectThreadChunkSize;
		/** Region in global coordinates */
		golem::Bounds::Desc::Seq detectRegionDesc;
		/** Object detection minimum size */
		golem::U32 detectMinSize;
		/** Object detection delta change */
		golem::U32 detectDeltaSize;
		/** Object detection delta depth */
		golem::Real detectDeltaDepth;

		/** Scan poses */
		Pose::Seq poseScanSeq;
		/** Action poses */
		Pose::Seq poseActionSeq;

		/** Show point */
		bool pointShow;
		/** Point colour */
		golem::RGBA pointColour;

		/** Trajectory bundles */
		Bundle::Seq bundleSeq;

		/** Interop description */
		InteropDesc interopDesc;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			golem::Player::Desc::setToDefault();

			camera.clear();
			bundle.clear();
			imageHandler.clear();
			imageItem.clear();
			processHandler.clear();
			processItem.clear();
			processBreak = false;
			modelHandler.clear();
			modelItem.clear();
			queryHandler.clear();
			queryItem.clear();
			queryBreak = false;
			trjHandler.clear();
			trjItem.clear();
			trjAutoPerf = true;
			trjMerge = false;
			trjBreak = false;

			detectFilterDesc.setToDefault();
			detectThreadChunkSize = 1000;
			detectRegionDesc.clear();
			detectMinSize = 10000;
			detectDeltaSize = 100;
			detectDeltaDepth = golem::Real(0.001);

			poseScanSeq.clear();
			poseActionSeq.clear();

			pointShow = true;
			pointColour = golem::RGBA::BLACK;

			bundleSeq.clear();

			interopDesc.setToDefault();
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const golem::Assert::Context& ac) const {
			golem::Player::Desc::assertValid(ac);

			golem::Assert::valid(camera.length() > 0, ac, "camera: invalid");
			golem::Assert::valid(bundle.length() > 0, ac, "bundle: invalid");
			golem::Assert::valid(imageHandler.length() > 0, ac, "imageHandler: invalid");
			golem::Assert::valid(imageItem.length() > 0, ac, "imageItem: invalid");
			golem::Assert::valid(processHandler.length() > 0, ac, "processHandler: invalid");
			golem::Assert::valid(processItem.length() > 0, ac, "processItem: invalid");
			golem::Assert::valid(modelHandler.length() > 0, ac, "modelHandler: invalid");
			golem::Assert::valid(modelItem.length() > 0, ac, "modelItem: invalid");
			golem::Assert::valid(queryHandler.length() > 0, ac, "queryHandler: invalid");
			golem::Assert::valid(queryItem.length() > 0, ac, "queryItem: invalid");
			golem::Assert::valid(trjHandler.length() > 0, ac, "trjHandler: invalid");
			golem::Assert::valid(trjItem.length() > 0, ac, "trjItem: invalid");

			detectFilterDesc.assertValid(Assert::Context(ac, "detectFilterDesc."));
			Assert::valid(detectThreadChunkSize > 0, ac, "detectThreadChunkSize: < 1");
			for (golem::Bounds::Desc::Seq::const_iterator i = detectRegionDesc.begin(); i != detectRegionDesc.end(); ++i)
				Assert::valid((*i)->isValid(), ac, "detectRegionDesc[]: invalid");
			golem::Assert::valid(detectMinSize > 0, ac, "detectMinSize: zero");
			golem::Assert::valid(detectDeltaSize > 0, ac, "detectDeltaSize: zero");
			golem::Assert::valid(detectDeltaDepth > golem::REAL_EPS, ac, "detectDeltaDepth: < eps");

			golem::Assert::valid(!poseScanSeq.empty(), ac, "poseScanSeq: empty");
			for (Pose::Seq::const_iterator i = poseScanSeq.begin(); i != poseScanSeq.end(); ++i)
				i->assertValid(golem::Assert::Context(ac, "poseScanSeq[]."));
			//golem::Assert::valid(!poseActionSeq.empty(), ac, "poseActionSeq: empty");
			for (Pose::Seq::const_iterator i = poseActionSeq.begin(); i != poseActionSeq.end(); ++i)
				i->assertValid(golem::Assert::Context(ac, "poseActionSeq[]."));

			for (Bundle::Seq::const_iterator i = bundleSeq.begin(); i != bundleSeq.end(); ++i)
				i->assertValid(golem::Assert::Context(ac, "bundleSeq[]."));

			interopDesc.assertValid(golem::Assert::Context(ac, "interopDesc."));
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GOLEM_CREATE_FROM_OBJECT_DESC1(AppGrasp, golem::Object::Ptr, golem::Scene&)
	};

protected:
	/** Depth camera */
	golem::CameraDepth* camera;

	/** Data bundle default path */
	std::string bundle;

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
	/** Processed query break point */
	bool queryBreak;

	/** Trajectory handler */
	golem::data::Handler* trjHandler;
	/** Trajectory item */
	std::string trjItem;
	/** Processed trajectory autonomous perform */
	bool trjAutoPerf;
	/** Processed trajectory merge */
	bool trjMerge;
	/** Processed trajectory break point */
	bool trjBreak;

	/** Filtering algortihm descriptiton */
	golem::Cloud::FilterDesc detectFilterDesc;
	/** Filtering algortihm descriptiton */
	golem::U32 detectThreadChunkSize;
	/** Region in global coordinates */
	golem::Bounds::Desc::Seq detectRegionDesc;
	/** Object detection minimum size */
	golem::U32 detectMinSize;
	/** Object detection delta change */
	golem::U32 detectDeltaSize;
	/** Object detection delta depth */
	golem::Real detectDeltaDepth;

	/** Scan poses */
	Pose::Seq poseScanSeq;
	/** Action poses */
	Pose::Seq poseActionSeq;
	
	/** Show point */
	bool pointShow;
	/** Point colour */
	golem::RGBA pointColour;

	/** Trajectory bundles */
	Bundle::Seq bundleSeq;

	/** Interop description */
	InteropDesc interopDesc;

	/** Demo renderer */
	golem::DebugRenderer demoRenderer;

	/** (Global search) trajectory from the configuration space and/or workspace target */
	virtual void findTrajectory(const golem::Controller::State& begin, const Pose& pose, golem::Controller::State::Seq& trajectory);
	/** Move to the specified configuration */
	virtual void gotoPose(const Pose& pose);

	/** golem::UIRenderer interface */
	virtual void render() const;

	virtual void release();

	void create(const Desc& desc);
	AppGrasp(golem::Scene &scene);
};

/** Reads/writes object from/to a given XML context */
void XMLData(AppGrasp::Pose& val, golem::XMLContext* xmlcontext, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(AppGrasp::Bundle& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};

#endif // _GOLEM_APP_GRASP_GRASP_H_