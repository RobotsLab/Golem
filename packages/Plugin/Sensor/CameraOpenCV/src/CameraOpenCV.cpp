/** @file OpenCV.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sensor/CameraOpenCV/CameraOpenCV.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <opencv2/opencv.hpp>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new CameraOpenCV::Desc();
}

//------------------------------------------------------------------------------

class CameraOpenCVDevice : public Camera::Device {
public:
	CameraOpenCVDevice(Camera* camera, const Camera::Property& property, golem::I32 index) : Camera::Device(camera, property), pCapture(nullptr) {
		pCapture = cvCreateCameraCapture(int(index));
		if (pCapture == nullptr)
			throw Message(Message::LEVEL_CRIT, "CameraOpenCVDevice::create(): unable to create cvcapture");

		IplImage* pFrame = cvQueryFrame(pCapture); // required
	
		this->property.width = (U32)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_WIDTH);
		this->property.height = (U32)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_HEIGHT);
		this->property.fps = (Real)cvGetCaptureProperty(pCapture, CV_CAP_PROP_FPS);
		this->property.channels = (U32)pFrame->nChannels;
		this->property.depth = (U32)pFrame->depth;
		if (Math::equals(this->property.fps, golem::REAL_ZERO, golem::REAL_EPS)) { // HACK: an OpenCV problem (?)
			const Real fps = Real(30.0);
			camera->getContext().warning("CameraOpenCVDevice::create(): invalid frame rate reported by cvGetCaptureProperty(), setting %f --> %f\n", this->property.fps, fps);
			this->property.fps = fps;
		}
	}
	~CameraOpenCVDevice() {
		if (pCapture != nullptr)
			cvReleaseCapture(&pCapture);
	}
	void capture(Image::Ptr& pImage) {
		const golem::SecTmReal systemTime1 = camera->getContext().getTimer().elapsed();
		if (cvGrabFrame(pCapture) == 0)
			throw Message(Message::LEVEL_ERROR, "CameraOpenCVDevice::capture(): unable to grab frame");
		const golem::SecTmReal systemTime2 = camera->getContext().getTimer().elapsed();

		IplImage* pFrame = cvRetrieveFrame(pCapture);
		if (pFrame == nullptr)
			throw Message(Message::LEVEL_ERROR, "CameraOpenCVDevice::capture(): no frame data");

		if (pImage == nullptr)
			pImage.reset(new Image());
		pImage->set(pFrame);
		pImage->timeStamp = REAL_HALF*(systemTime1 + systemTime2); // TODO use better method
	}

private:
	CvCapture* pCapture;
};

//class CameraOpenCVDevice : public Camera::Device {
//public:
//	CameraOpenCVDevice(Camera* camera, const Camera::Property& property, golem::I32 index) : Camera::Device(camera, property) {
//		if (!cvCapture.open(int(index)))
//			throw Message(Message::LEVEL_CRIT, "CameraOpenCVDevice::create(): unable to create cvcapture");
//
//		this->property.width = (U32)cvCapture.get(CV_CAP_PROP_FRAME_WIDTH);
//		this->property.height = (U32)cvCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
//		this->property.fps = (Real)cvCapture.get(CV_CAP_PROP_FPS);
//		this->property.channels = 3; // fixed
//		this->property.depth = IPL_DEPTH_8U; // fixed
//		if (Math::equals(this->property.fps, golem::REAL_ZERO, golem::REAL_EPS)) { // HACK: an OpenCV problem (?)
//			const Real fps = Real(30.0);
//			camera->getContext().warning("CameraOpenCVDevice::create(): invalid frame rate reported by cvGetCaptureProperty(), setting %f --> %f\n", this->property.fps, fps);
//			this->property.fps = fps;
//		}
//	}
//	~CameraOpenCVDevice() {}
//	void capture(Image::Ptr& pImage) {
//		const golem::SecTmReal systemTime1 = camera->getContext().getTimer().elapsed();
//		if (!cvCapture.grab())
//			throw Message(Message::LEVEL_ERROR, "CameraOpenCVDevice::capture(): unable to grab frame");
//		//cvCapture.read(frame);
//		const golem::SecTmReal systemTime2 = camera->getContext().getTimer().elapsed();
//
//		if (pImage == nullptr)
//			pImage.reset(new Image());
//		pImage->timeStamp = REAL_HALF*(systemTime1 + systemTime2); // TODO use better method
//		if (!cvCapture.retrieve(frame))
//			throw Message(Message::LEVEL_ERROR, "CameraOpenCVDevice::capture(): no frame data");
//
//		//*pImage->image = frame;
//	}
//
//private:
//	cv::Mat frame;
//	cv::VideoCapture cvCapture;
//};

CameraOpenCV::CameraOpenCV(golem::Context& context) : Camera(context) {
}

void CameraOpenCV::create(const Desc& desc) {
	desc.assertValid(Assert::Context("CameraOpenCV::Desc."));

	device.reset(new CameraOpenCVDevice(this, desc.properties[0], desc.index)); // throws
	property = device->getProperty();
	properties = desc.properties;
	device.reset();
	
	Camera::create(desc); // throws
}

void CameraOpenCV::start() {
	device.reset(new CameraOpenCVDevice(this, property, index)); // throws
}

void CameraOpenCV::stop() {
	device.reset();
}

//------------------------------------------------------------------------------
