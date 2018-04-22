/** @file Recorder.h
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
#ifndef _GOLEM_APP_RECORDER_H_
#define _GOLEM_APP_RECORDER_H_

//------------------------------------------------------------------------------

#include <Golem/App/Manager.h>
#include <Golem/Plugin/SensorI.h>
#include <Golem/Tools/Camera.h>
#include <Golem/Tools/FT.h>
#include <Golem/Tools/RBPose.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Recorder. */
class Recorder : public Manager {
public:
	/** Scan pose callback */
	typedef std::function<bool()> ScanPoseCommand;

	/** Recorder Task */
	class Task : public ThreadTask {
	public:
		typedef golem::shared_ptr<Task> Ptr;
		typedef std::map<Sensor*, Ptr> Map;

		/** Description */
		class Desc {
		public:
			typedef std::map<std::string, Desc> Map;

			/** Snapshot */
			bool snapshot;
			/** Sequence */
			bool sequence;

			/** Set to default */
			Desc() {
				setToDefault();
			}
			/** Sets the parameters to the default values */
			void setToDefault() {
				snapshot = true;
				sequence = true;
			}
			/** Assert that the description is valid. */
			void assertValid(const Assert::Context& ac) const {
				Assert::valid(snapshot || sequence, ac, "snapshot|sequence: invalid");
			}
			/** Load descritpion from xml context. */
			void load(const golem::XMLContext* context);
		};

		/** Image handler */
		data::Capture* captureSnapshot;
		/** Video handler */
		data::Capture* captureSequence;

		/** Initialise */
		Task(const Desc& desc, Recorder& recorder, Sensor& sensor);
	};
	friend class Task;

	/** Recorder description */
	class Desc : public Manager::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Sensor libraries */
		Library::Path::Seq sensors;

		/** Recording task description */
		Task::Desc::Map recordingDescMap;
		/** Recording label */
		std::string recordingLabel;

		/** Cloud frame operations */
		RBAdjust cloudAdjust;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Manager::Desc::setToDefault();

			sensors.clear();

			recordingDescMap.clear();
			recordingLabel = "recorder";

			cloudAdjust.setToDefault();
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Manager::Desc::assertValid(ac);

			for (Library::Path::Seq::const_iterator i = sensors.begin(); i != sensors.end(); ++i)
				i->assertValid(Assert::Context(ac, "sensors[]->"));

			for (Task::Desc::Map::const_iterator i = recordingDescMap.begin(); i != recordingDescMap.end(); ++i) {
				Assert::valid(i->first.length() > 0, ac, "recordingDesc[].sensor: empty");
				i->second.assertValid(Assert::Context(ac, "recordingDesc[]."));
			}
			Assert::valid(recordingLabel.length() > 0, ac, "recordingLabel: empty");

			cloudAdjust.assertValid(Assert::Context(ac, "cloudAdjust."));
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GOLEM_CREATE_FROM_OBJECT_DESC1(Recorder, golem::Object::Ptr, golem::Scene&)
	};

protected:
	/** Sensor libraries */
	Sensor::Map sensorMap;
	/** Sensor pointer */
	Sensor::Map::iterator sensorCurrentPtr;

	/** Current pose */
	ConfigMat34 currentPose;

	/** Recording task description */
	Task::Desc::Map recordingDescMap;
	/** Recording label */
	std::string recordingLabel;

	/** Cloud frame operations */
	RBAdjust cloudAdjust;

	/** Sensor access cs */
	mutable golem::CriticalSection csSensor;
	/** Sensor renderer */
	golem::DebugRenderer sensorRenderer;
	/** Cloud renderer */
	golem::DebugRenderer cloudRenderer;

	/** Current camera */
	mutable Camera* currentCamera;
	/** Current image */
	mutable Image currentImage;
	/** Current image id */
	mutable unsigned currentImageId;

	/** Recorder cs */
	mutable golem::CriticalSection csRecorder;
	/** Recorder sync */
	golem::Event evRecorderStart, evRecorderStop;
	/** Recorder count */
	size_t recorderCount, recorderCountStart, recorderCountStop;
	/** Recorder task */
	Task::Map recorderMap;
	/** Recorder sequence */
	bool recorderSequence;
	/** Recorder data */
	std::string recorderData;
	/** Recorder item */
	std::string recorderItem;
	/** Recorder stop time */
	golem::SecTmReal recorderStart, recorderStop;

	/** Recording number of recorders */
	virtual size_t recordingSize(bool video = true) const;
	/** Recording start, non-blockig */
	virtual bool recordingStart(const std::string& data, const std::string& item, bool sequence = true);
	/** Recording stop, non-blockig */
	virtual void recordingStop(golem::SecTmReal stop = golem::SEC_TM_REAL_ZERO);
	/** Recording active */
	virtual bool recordingActive();
	/** Recording wait for command completion */
	virtual bool recordingWaitToStart(golem::MSecTmU32 timeOut = golem::MSEC_TM_U32_INF);
	/** Recording wait for command completion */
	virtual bool recordingWaitToStop(golem::MSecTmU32 timeOut = golem::MSEC_TM_U32_INF);
	/** Recording save item */
	virtual void recordingSave(const Sensor& sensor, const data::Item::Ptr& item);

	/** Scan pose */
	virtual void scanPose(ScanPoseCommand scanPoseCommand = nullptr);

	/** Move to the specified configuration */
	virtual void gotoPose(const ConfigMat34& pose) {
		currentPose = pose;
	}
	/** Read current configuration */
	virtual void getPose(golem::U32 joint, ConfigMat34& pose) const {
		pose = currentPose;
	}

	/** Set current image */
	virtual void setImage(const IplImage* pImage = nullptr, Camera* pCamera = nullptr) const;
	/** Set current camera */
	virtual bool setCamera(Camera* pCamera, bool video = true) const;
	/** Is the given camera set as current */
	virtual bool isCamera(const Camera* pCamera) const;
	/** Reset current camera */
	virtual void resetCamera() const;
	/** Calibrate the given camera */
	virtual void calibrateCamera(Camera* pCamera, bool bIntrinsic, CameraCalibration::ConfigCommand configCommand = nullptr);

	/** golem::Object (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);
	// golem::Object interface
	virtual void render() const;
	// golem::Object interface
	virtual void customRender() const;

	/** UIKeyboardMouse: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);

	/** Releases resources */
	virtual void release();

	void create(const Desc& desc);
	Recorder(golem::Scene &scene);
};

/** Reads/writes object from/to a given XML context */
void XMLData(golem::Recorder::Task::Desc::Map::value_type& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace


#endif /*_GOLEM_APP_RECORDER_H_*/
