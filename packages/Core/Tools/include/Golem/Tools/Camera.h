/** @file Camera.h
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
#ifndef _GOLEM_TOOLS_CAMERA_H_
#define _GOLEM_TOOLS_CAMERA_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Sensor.h>
#include <Golem/Tools/CameraCalibration.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Camera interface */
class Camera : public Sensor, protected golem::Runnable {
public:
	/** Camera control */
	enum State {
		/** Default inactive state */
		DEFAULT,
		/** Stop */
		CMD_STOP,
		/** Capture single image */
		CMD_IMAGE,
		/** Capture image sequence */
		CMD_VIDEO,
	};

	/** Image transformation */
	enum Transform {
		/** None */
		TRANSFORM_NONE = 0,
		/** IR */
		TRANSFORM_FLIP_HORIZONTAL = (1 << 0),
		/** Colour */
		TRANSFORM_FLIP_VERTICAL = (1 << 1),
	};

	/** Camera stream properties */
	class Property {
	public:
		/** Sequence */
		typedef std::vector<Property> Seq;

		/** Image width */
		golem::U32 width;
		/** Image height */
		golem::U32 height;
		/** FPS */
		golem::Real fps;
		
		/** Camera mode */
		std::string mode;
		/** Camera pixel format */
		std::string format;

		/** Image channels */
		golem::U32 channels;
		/** Image depth */
		golem::U32 depth;

		/** Image transformation */
		golem::U32 transform;

		/** Constructs description. */
		Property() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			width = 0; // auto
			height = 0; // auto
			fps = golem::Real(0.0); // auto
			mode.clear();
			format.clear();
			channels = 3;
			depth = IPL_DEPTH_8U;
			transform = 0;
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Assert::valid(fps >= golem::REAL_ZERO, ac, "fps: < 0");
		}
		/** Equals */
		friend bool operator == (const Property& l, const Property& r) {
			if (l.width != r.width || l.height != r.height)
				return false;
			if (!golem::Math::equals(l.fps, r.fps, golem::REAL_EPS))
				return false;
			if (l.mode.compare(r.mode) != 0 || l.format.compare(r.format) != 0)
				return false;
			return true;
		}
		/** Equals */
		friend bool operator != (const Property& l, const Property& r) {
			return !(l == r);
		}
	};

	/** Camera device */
	class Device {
	public:
		typedef golem::shared_ptr<Device> Ptr;

		/** Virtual destructor */
		Device(Camera* camera, const Property& property) : camera(camera), property(property) {}
		/** Virtual destructor */
		virtual ~Device() {}

		/** Capture an image alocating pImage if nullptr */
		virtual void capture(Image::Ptr& pImage) = 0;
		/** Camera property */
		Property getProperty() const {
			return property;
		}

	protected:
		/** Owner */
		Camera* camera;
		/** Camera property */
		Property property;
	};
	
	/** Camera description */
	class Desc : public Sensor::Desc {
	public:
		/** Camera index */
		golem::I32 index;

		/** Properties */
		Property::Seq properties;

		/** Camera calibration */
		CameraCalibration::Desc::Map calibrationDescMap;
		/** Camera calibration file */
		std::string calibrationFile;

		/** Image buffer length */
		size_t bufferLen;
		/** Camera thread time out */
		golem::MSecTmU32 threadTimeOut;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		
		/** Sets the parameters to the default values. */
		void setToDefault() {
			Sensor::Desc::setToDefault();

			libraryPrefix = "GolemCamera";

			index = 0;
			
			properties.resize(1);
			properties[0].setToDefault();

			calibrationDescMap.clear();
			calibrationFile = "GolemCamera.cal";

			bufferLen = 600; // 2 sec at 30 fps
			threadTimeOut = 10000; //[msec]
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Sensor::Desc::assertValid(ac);

			Assert::valid(!properties.empty(), ac, "properties: empty");
			for (Property::Seq::const_iterator i = properties.begin(); i != properties.end(); ++i)
				i->assertValid(Assert::Context(ac, "properties[]."));

			Assert::valid(!calibrationDescMap.empty() && calibrationDescMap.begin()->second != nullptr, ac, "calibrationDesc: unspecified");
			for (CameraCalibration::Desc::Map::const_iterator i = calibrationDescMap.begin(); i != calibrationDescMap.end(); ++i) {
				Assert::valid(!i->first.empty(), ac, "calibrationDesc[].file: empty");
				i->second->assertValid(Assert::Context(ac, "calibrationDesc[]->"));
			}
			Assert::valid(!calibrationFile.empty(), ac, "calibrationFile: empty");

			Assert::valid(bufferLen > 0, ac, "bufferLen: 0");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
	};

	/** Camera control */
	virtual bool set(State state, golem::MSecTmU32 timeOut = golem::MSEC_TM_U32_INF);
	/** Camera control */
	virtual State get();
	/** Is terminating */
	bool isTerminating() const {
		return bTerminate;
	}
	
	/** Look into image sequence */
	template <typename _Peek> void peek(_Peek peek) const {
		golem::CriticalSectionWrapper csw(csData);
		peek(images);
	}
	/** Look into image sequence */
	template <typename _Peek> void waitAndPeek(_Peek peek, golem::MSecTmU32 timeOut = golem::MSEC_TM_U32_INF) {
		const bool result = evData.wait(timeOut);
		golem::CriticalSectionWrapper csw(csData);
		if (result) evData.set(false);
		peek(images, result);
	}
	/** Pop the subsequent image (from the front of the image queue) */
	Image::Ptr pop_front(golem::MSecTmU32 timeOut = golem::MSEC_TM_U32_INF);
	/** Pop the latest image (from the back of the image queue) - skip all subsequent images and move them to the buffer queue */
	Image::Ptr pop_back(golem::MSecTmU32 timeOut = golem::MSEC_TM_U32_INF);
	
	/** Push images to the buffer queue */
	void push(Image::Ptr pImage);
	
	/** The buffer queue size */
	size_t size();
	/** Clear the buffer queue */
	void clear();

	/** Camera capture property */
	const Property::Seq& getProperties() const {
		return properties;
	}

	/** Current camera property */
	virtual void setProperty(const Property& property) {
		this->property = property;
	}
	/** Current camera property */
	virtual Property getProperty() const {
		return device != nullptr ? device->getProperty() : property;
	}

	/** Get calibration map */
	const CameraCalibration::Map& getCalibrationMap() const {
		return calibrationMap;
	}
	
	/** Get Current calibration */
	CameraCalibration* getCurrentCalibration() {
		return calibrationCurrentPtr->second.get();
	}
	/** Get Current calibration */
	const CameraCalibration* getCurrentCalibration() const {
		return calibrationCurrentPtr->second.get();
	}

	/** Get Current calibration */
	const std::string& getCurrentCalibrationFile() const {
		return calibrationCurrentPtr->first;
	}
	/** Set Current calibration */
	void setCurrentCalibrationFile(const std::string& calibrationFile);

	/** String sequence helper */
	static void getStrings(const std::string& inp, StringSeq& out);

	/** Curent sensor frame */
	virtual golem::Mat34 getFrame() const;

	/** String image transformation helper */
	static golem::U32 getTransform(const std::string& str);

	/** Destroys the Camera */
	virtual ~Camera();
	
protected:
	/** Camera device */
	Device::Ptr device;

	/** Camera index */
	golem::I32 index;
	/** Properties */
	Property::Seq properties;
	/** Camera property */
	Property property;

	/** Image buffer length */
	size_t bufferLen;
	/** Camera thread time out */
	golem::MSecTmU32 threadTimeOut;
	/** Camera calibration map */
	CameraCalibration::Map calibrationMap;
	/** Current Camera calibration */
	const CameraCalibration::Map::value_type* calibrationCurrentPtr;

	State state;
	Image::List images, buffer;

	mutable golem::CriticalSection csData;
	golem::Event evCmd, evCmdComplete;
	golem::Event evData;
	golem::Thread thread;
	volatile bool bTerminate;

	/** Image transformation */
	virtual void transform(golem::U32 imageTransform, Image& image) const;

	/** Capture sequence start */
	virtual void start() = 0;
	/** Capture sequence stop */
	virtual void stop() = 0;

	/** Player thread */
	virtual void run();

	/** Creates/initialises the Camera */
	void create(const Desc& desc);
	
	/** Constructs the Camera */
	Camera(golem::Context& context);
};

//------------------------------------------------------------------------------

void XMLData(Camera::Property& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

/** Depth Camera */
class CameraDepth : public Camera {
public:
	/** Camera description */
	class Desc : public Camera::Desc {
	public:
		/** Default colour */
		golem::RGBA colour;
		/** transform from colour to ir camera frame */
		golem::Mat34 colourToIRFrame;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			Camera::Desc::setToDefault();
			colour = golem::RGBA::RED;
			colourToIRFrame.setId();
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Camera::Desc::assertValid(ac);

			Assert::valid(properties.size() >= 2, ac, "properties: size < 2");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
	};

	/** Image camera property */
	Property getImageProperty() const {
		return properties[0];
	}
	/** Depth camera property */
	Property getDepthProperty() const {
		return properties[1];
	}

	const golem::Mat34& getColourToIRFrame() const {
		return colourToIRFrame;
	}
	void setColourToIRFrame(const golem::Mat34& frame);

	/** Draw sensor at the current pose */
	virtual void draw(const Appearance& appearance, golem::DebugRenderer& renderer) const;

protected:
	/** Default colour */
	golem::RGBA colour;

	/** transform from colour to ir camera frame */
	golem::Mat34 colourToIRFrame;

	/** Creates/initialises the Camera */
	void create(const Desc& desc);
	/** Constructs the Camera */
	CameraDepth(golem::Context& context);
};

//------------------------------------------------------------------------------

/** Image.
*	(Optionally) Implemented by camera.
*/
typedef golem::UICapture CameraImage;

//------------------------------------------------------------------------------

/** Object model.
*	(Optionally) Implemented by camera.
*/
class CameraDepthModel {
public:
	/** Sets model */
	virtual void setModel(const golem::Bounds::Seq& bounds) = 0;
};

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_TOOLS_CAMERA_H_*/
