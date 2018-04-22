/** @file CameraRGBSim.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2018 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sensor/CameraRGBSim/CameraRGBSim.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <opencv2/opencv.hpp>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new CameraRGBSim::Desc();
}

//------------------------------------------------------------------------------

void golem::CameraRGBSim::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Camera::Desc::load(context, xmlcontext);

	XMLData("input_file", inputFile, const_cast<golem::XMLContext*>(xmlcontext), false);
	XMLData("frame_index", frameIndex, const_cast<golem::XMLContext*>(xmlcontext), false);
}

//------------------------------------------------------------------------------

class CameraRGBSimDevice : public Camera::Device {
public:
	CameraRGBSimDevice(Camera* camera, const Camera::Property& property, const std::string& inputFile, golem::U32 frameIndex) : Camera::Device(camera, property), pCapture(nullptr), pFrame(nullptr) {
		pCapture = cvCreateFileCapture(inputFile.c_str());
		if (pCapture == nullptr)
			throw Message(Message::LEVEL_CRIT, "CameraRGBSimDevice::create(): unable to create cvcapture");
		
		this->frameIndex = frameIndex;
		this->frameIndexPrev = this->frameIndex;
		if (!cvSetCaptureProperty(pCapture, CV_CAP_PROP_POS_FRAMES, frameIndex))
			throw Message(Message::LEVEL_ERROR, "CameraRGBSimDevice::create(): invalid frame %u", frameIndex);
		pFrame = cvQueryFrame(pCapture); // required
	
		this->property.width = (U32)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_WIDTH);
		this->property.height = (U32)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_HEIGHT);
		this->property.fps = (Real)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FPS);
		if (Math::equals(this->property.fps, golem::REAL_ZERO, golem::REAL_EPS)) { // HACK: an OpenCV problem (?)
			const Real fps = Real(30.0);
			camera->getContext().warning("CameraRGBSimDevice::create(): invalid frame rate reported by cvGetCaptureProperty(), setting %f --> %f\n", this->property.fps, fps);
			this->property.fps = fps;
		}
		this->property.channels = (U32)pFrame->nChannels;
		this->property.depth = (U32)pFrame->depth;
	}
	~CameraRGBSimDevice() {
		if (pCapture != nullptr)
			cvReleaseCapture(&pCapture);
	}
	void capture(Image::Ptr& pImage) {
		const golem::SecTmReal systemTime1 = camera->getContext().getTimer().elapsed();
		if (frameIndexPrev != frameIndex) {
			if (!cvSetCaptureProperty(pCapture, CV_CAP_PROP_POS_FRAMES, frameIndex))
				throw Message(Message::LEVEL_ERROR, "CameraRGBSimDevice::capture(): invalid frame %u", frameIndex);
			pFrame = cvQueryFrame(pCapture);
			if (pFrame == nullptr)
				throw Message(Message::LEVEL_ERROR, "CameraRGBSimDevice::capture(): no frame data");
			frameIndexPrev  = frameIndex;
		}
		const golem::SecTmReal systemTime2 = camera->getContext().getTimer().elapsed();

		const golem::MSecTmU32 sleep = golem::SecToMSec(REAL_ONE/property.fps - (systemTime2 - systemTime1));
		if (sleep > 0)
			golem::Sleep::msleep(sleep);

		if (pImage == nullptr)
			pImage.reset(new Image());
		pImage->set(pFrame);
		pImage->timeStamp = REAL_HALF*(systemTime1 + systemTime2); // TODO use better method
	}

private:
	golem::U32 frameIndex;
	golem::U32 frameIndexPrev;
	CvCapture* pCapture;
	IplImage* pFrame;
};

CameraRGBSim::CameraRGBSim(golem::Context& context) : Camera(context) {
}

void CameraRGBSim::create(const Desc& desc) {
	desc.assertValid(Assert::Context("CameraRGBSim::Desc."));

	inputFile = desc.inputFile;
	frameIndex = desc.frameIndex;

	device.reset(new CameraRGBSimDevice(this, desc.properties[0], inputFile, frameIndex)); // throws
	property = device->getProperty();
	properties = desc.properties;
	device.reset();
	
	Camera::create(desc); // throws
}

void CameraRGBSim::start() {
	device.reset(new CameraRGBSimDevice(this, property, inputFile, frameIndex)); // throws
}

void CameraRGBSim::stop() {
	device.reset();
}

//------------------------------------------------------------------------------
