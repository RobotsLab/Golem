/** @file Video.h
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
#ifndef _GOLEM_DATA_VIDEO_VIDEO_H_
#define _GOLEM_DATA_VIDEO_VIDEO_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Library.h>
#include <Golem/Sys/Stream.h>
#include <Golem/Plugin/Renderer.h>
#include <Golem/Tools/CameraCalibration.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Tools/Cloud.h>
#include <Golem/Plugin/DataI.h>
#include <Golem/Plugin/SensorI.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

class ItemVideo;
class HandlerVideo;

/** Data item representing video and point cloud sequence with buffering and temporary files.
*/
class GOLEM_LIBRARY_DECLDIR ItemVideo : public Item, public golem::Image, public Convert {
public:
	friend class HandlerVideo;

	/** Frame information */
	class GOLEM_LIBRARY_DECLDIR Frame {
	public:
		typedef std::vector<Frame> Seq;

		/** Data time stamp. */
		TimeStamp timeStamp;
		/** Robot configuration */
		ConfigMat34 config;
		/** Cloud pose */
		golem::Mat34 cloudPose;
	};

	/** Frame information */
	Frame::Seq frame;
	/** Current frame index */
	golem::U32 frameIndex;
	/** A single cloud file, use Frame::cloudPose transforms */
	bool useCloudFrame;

	/** Camera parameters */
	golem::CameraCalibration::Parameters parameters;

	/** Video file */
	mutable File videoFile;
	/** Cloud file */
	mutable File::Seq cloudFile;

	/** Frame file */
	mutable File frameFile;

	/** Clones item. */
	virtual Item::Ptr clone() const;

	/** Creates render buffer, the buffer can be shared and allocated on Handler */
	virtual void createRender();

protected:
	/** Data handler */
	HandlerVideo& handler;

	// prefix
	std::string prefix;
	// video
	std::string videoSuffix;
	// cloud
	std::string cloudSuffix;

	/** Convert: Convert current item */
	virtual Item::Ptr convert(const Handler& handler);
	/** Convert: return available interfaces */
	virtual const StringSeq& getConvertInterfaces() const;
	/** Convert: is supported by the interface */
	virtual bool isConvertSupported(const Handler& handler) const;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	/** Load video and point clouds. */
	virtual void load();

	/** Initialise data item */
	ItemVideo(HandlerVideo& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerVideo : public Handler, public UI, public Capture {
public:
	friend class ItemVideo;

	/** Video appearance */
	class GOLEM_LIBRARY_DECLDIR VideoAppearance {
	public:
		/** Show video */
		bool show;
		/** Show video camera frame */
		bool showCameraFrame;

		/** Constructs description. */
		VideoAppearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			show = true;
			showCameraFrame = true;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
		}
		/** Reads/writes object from/to a given XML context */
		void xmlData(golem::XMLContext* context, bool create = false) const;
	};

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Handler::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Handler temporary folder */
		std::string tmpDir;

		/** Cloud appearance */
		Cloud::Appearance cloudAppearance;
		/** Cloud suffix */
		std::string cloudSuffix;

		/** Capture region in global coordinates */
		golem::Bounds::Desc::Seq regionCaptureDesc;

		/** Video appearance */
		VideoAppearance videoAppearance;
		/** Video suffix */
		std::string videoSuffix;
		/** Video format */
		std::string videoFormat;
		/** Video fps */
		golem::Real videoFPS;

		/** Frame suffix */
		std::string frameSuffix;

		/** Set to default */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			golem::data::Handler::Desc::setToDefault();

			tmpDir = "";

			cloudAppearance.setToDefault();
			cloudSuffix = golem::Import::FILE_EXT_CLOUD_PCD;

			regionCaptureDesc.clear();

			videoAppearance.setToDefault();
			videoSuffix = ".mp4";
			videoFormat = "X264"; // CV_CAP_PROP_FOURCC
			videoFPS = golem::Real(30.0);

			frameSuffix = ".frame";
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			golem::data::Handler::Desc::assertValid(ac);

			cloudAppearance.assertValid(Assert::Context(ac, "cloudAppearance."));
			Assert::valid(cloudSuffix.length() > 0, ac, "cloudSuffix: empty");

			for (golem::Bounds::Desc::Seq::const_iterator i = regionCaptureDesc.begin(); i != regionCaptureDesc.end(); ++i)
				Assert::valid((*i)->isValid(), ac, "regionCaptureDesc[]: invalid");

			videoAppearance.assertValid(Assert::Context(ac, "videoAppearance."));
			Assert::valid(videoSuffix.length() > 0, ac, "videoSuffix: empty");
			Assert::valid(videoFormat.length() == 4 || !videoFormat.compare("-1"), ac, "videoFormat: length != 4");

			Assert::valid(frameSuffix.length() > 0, ac, "frameSuffix: empty");
			Assert::valid(videoFPS > golem::REAL_EPS, ac, "videoFPS: < eps");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual Handler::Ptr create(golem::Context &context) const;
	};

	/** Cloud appearance */
	const Cloud::Appearance& getCloudAppearance() const {
		return cloudAppearance;
	}

protected:
	/** Image cache */
	golem::ImageData image;
	mutable unsigned imageID;
	/** Point cloud rendering */
	golem::DebugRenderer cloudRenderer;

	/** Handler temporary folder */
	std::string tmpDir;

	/** Cloud appearance */
	Cloud::Appearance cloudAppearance;
	/** Cloud suffix */
	std::string cloudSuffix;

	/** Region capture */
	golem::Bounds::Seq regionCapture;

	/** Video appearance */
	VideoAppearance videoAppearance;
	/** Video suffix */
	std::string videoSuffix;
	/** Video format */
	std::string videoFormat;
	/** Video fps */
	golem::Real videoFPS;
	/** Video frame request */
	golem::I32 videoFrameRequest;

	/** Frame suffix */
	std::string frameSuffix;

	/** Convert interfaces */
	StringSeq convertInterfaces;

	/** Creates render buffer */
	void createRender(const ItemVideo& item);
	/** golem::UIRenderer: Render on output device. */
	virtual void render() const;
	/** golem::UIRenderer: Render on output device. */
	virtual void customRender() const;

	/** golem::UIRenderer: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	/** golem::UIRenderer: Mouse motion handler. */
	virtual void motionHandler(int x, int y);
	/** golem::UIRenderer: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);

	/** Construct empty item, accessible only by Data. */
	virtual Item::Ptr create() const;

	/** Capture: Captures snapshot or sequence from sensor. */
	virtual Item::Ptr capture(golem::Sensor& sensor, Control control = nullptr);

	/** Convert: Convert current item */
	virtual Item::Ptr convert(ItemVideo& itemVideo, const Handler& handler);
	/** Convert: is supported by the interface */
	virtual bool isConvertSupported(const Handler& handler) const;

	/** Create cloud index string */
	std::string cloudIndex(golem::U32 frame) const;

	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerVideo(golem::Context &context);
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

namespace golem {
	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::ConfigMat34& pose) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::ConfigMat34& pose);

	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::data::ItemVideo::Frame& frame) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::data::ItemVideo::Frame& frame);
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_DATA_VIDEO_VIDEO_H_*/
