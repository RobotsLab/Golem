/** @file NNLearning.h
 *
 * Demo NNLearning
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

/** Golem namespace */
namespace golem {

//------------------------------------------------------------------------------

/** NNLearning demo. */
class AppNNLearning : public golem::Player {
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

	/** NNLearning demo description */
	class Desc : public golem::Player::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Depth camera */
		std::string camera;

		/** Data bundle default name */
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
			trjBreak = false;

			detectFilterDesc.setToDefault();
			detectThreadChunkSize = 1000;
			detectRegionDesc.clear();
			detectMinSize = 10000;
			detectDeltaSize = 100;
			detectDeltaDepth = golem::Real(0.001);

			poseScanSeq.clear();
			poseActionSeq.clear();
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
			golem::Assert::valid(!poseActionSeq.empty(), ac, "poseActionSeq: empty");
			for (Pose::Seq::const_iterator i = poseActionSeq.begin(); i != poseActionSeq.end(); ++i)
				i->assertValid(golem::Assert::Context(ac, "poseActionSeq[]."));
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GOLEM_CREATE_FROM_OBJECT_DESC1(AppNNLearning, golem::Object::Ptr, golem::Scene&)
	};

protected:
	/** Depth camera */
	golem::CameraDepth* camera;

	/** Data bundle default name */
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

	/** Demo renderer */
	golem::DebugRenderer demoRenderer;

	/** Move to the specified configuration */
	void gotoPose(const Pose& pose);

	/** golem::UIRenderer interface */
	virtual void render() const;

	void create(const Desc& desc);
	AppNNLearning(golem::Scene &scene);
	~AppNNLearning();
};

/** Reads/writes object from/to a given XML context */
void XMLData(AppNNLearning::Pose& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};

#endif // _GOLEM_APP_GRASP_GRASP_H_