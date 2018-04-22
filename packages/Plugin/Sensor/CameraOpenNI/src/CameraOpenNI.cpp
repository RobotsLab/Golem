/** @file OpenNI.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sensor/CameraOpenNI/CameraOpenNI.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <OpenNI.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new CameraOpenNI::Desc();
}

//------------------------------------------------------------------------------

void golem::CameraOpenNI::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	CameraDepth::Desc::load(context, xmlcontext);

	XMLData("buffer_off", bufferOff, const_cast<golem::XMLContext*>(xmlcontext), false);
	XMLData("stream_timeout", streamTimeOut, const_cast<golem::XMLContext*>(xmlcontext), false);
}

//------------------------------------------------------------------------------

class CameraOpenNIDevice : public Camera::Device {
public:
	const size_t bufferOff;
	const RGBA colour;
	const int timeout;

	typedef std::map<U32, U32> ModeFormatMap;

	CameraOpenNIDevice(Camera* camera, const Camera::Property& property, golem::I32 index, size_t bufferOff, const RGBA& colour, int timeout) : Device(camera, property), bufferOff(bufferOff), colour(colour), timeout(timeout) {
		StringSeq modeStr, formatStr;
		CameraOpenNI::getStrings(this->property.mode, modeStr);
		CameraOpenNI::getStrings(this->property.format, formatStr);
		if (modeStr.empty() || modeStr.size() != formatStr.size())
			throw golem::Message(golem::Message::LEVEL_CRIT, "CameraOpenNIDevice::CameraOpenNIDevice(): Camera modes do not correspond to formats");
		
		ModeFormatMap modeFormatMap;
		mode = CameraOpenNI::MODE_UNDEF;
		for (size_t i = 0; i < modeStr.size(); ++i)
			mode |= modeFormatMap.insert(ModeFormatMap::value_type(CameraOpenNI::getMode(modeStr[i]), std::stoul(formatStr[i]))).first->first;

		assertValid(openni::OpenNI::initialize(), "CameraOpenNIDevice::CameraOpenNIDevice(): openni::OpenNI::initialize(): ");

		openni::Array<openni::DeviceInfo> deviceInfoList;
		openni::OpenNI::enumerateDevices(&deviceInfoList);
		if (deviceInfoList.getSize() == 0)
			throw golem::Message(golem::Message::LEVEL_CRIT, "CameraOpenNIDevice::CameraOpenNIDevice(): No connected devices");
		if (deviceInfoList.getSize() <= index) {
			camera->getContext().write("Available devices:\n");
			for (int i = 0; i < deviceInfoList.getSize(); ++i) {
				const openni::DeviceInfo info = deviceInfoList[i];
				camera->getContext().write("index=%d, name=%s, vendor=%s, uri=%s\n", i, info.getName(), info.getVendor(), info.getUri());
			}
			throw golem::Message(golem::Message::LEVEL_CRIT, "CameraOpenNIDevice::CameraOpenNIDevice(): Invalid device index");
		}
		
		assertValid(device.open(deviceInfoList[index].getUri()), "CameraOpenNIDevice::CameraOpenNIDevice(): openni::OpenNI::initialize(): ");

		hasDepthStream = hasImageStream = hasRGBStream = false;
		
		if (mode & CameraOpenNI::MODE_IR) {
			assertValid(imageStream.create(device, openni::SENSOR_IR), "CameraOpenNIDevice::CameraOpenNIDevice(): openni::Device::create(): ");
			assertValid(setVideoMode(this->property.width, this->property.height, this->property.fps, modeFormatMap[CameraOpenNI::MODE_IR], imageStream), "CameraOpenNIDevice::CameraOpenNIDevice(): openni::VideoStream::setVideoMode(): ");
			hasImageStream = true;
		}
		else if (mode & CameraOpenNI::MODE_COLOUR) {
			assertValid(imageStream.create(device, openni::SENSOR_COLOR), "CameraOpenNIDevice::CameraOpenNIDevice(): openni::Device::create(): ");
			assertValid(setVideoMode(this->property.width, this->property.height, this->property.fps, modeFormatMap[CameraOpenNI::MODE_COLOUR], imageStream), "CameraOpenNIDevice::CameraOpenNIDevice(): openni::VideoStream::setVideoMode(): ");
			hasImageStream = hasRGBStream = true;
		}
		if (mode & CameraOpenNI::MODE_DEPTH) {
			assertValid(depthStream.create(device, openni::SENSOR_DEPTH), "CameraOpenNIDevice::CameraOpenNIDevice(): openni::Device::create(): ");
			assertValid(setVideoMode(this->property.width, this->property.height, this->property.fps, modeFormatMap[CameraOpenNI::MODE_DEPTH], depthStream), "CameraOpenNIDevice::CameraOpenNIDevice(): openni::VideoStream::setVideoMode(): ");
			hasDepthStream = true;
		}

		if (hasDepthStream) {
			if (hasImageStream)
				assertValid(device.setDepthColorSyncEnabled(true), "CameraOpenNIDevice::CameraOpenNIDevice(): openni::OpenNI::setDepthColorSyncEnabled(): ");
			if (hasRGBStream)
				assertValid(device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR), "CameraOpenNIDevice::CameraOpenNIDevice(): openni::OpenNI::setImageRegistrationMode(): ");
		}
		else if (hasImageStream) {
			//assertValid(device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF), "CameraOpenNIDevice::CameraOpenNIDevice(): openni::OpenNI::setImageRegistrationMode(): ");
		}
	}
	~CameraOpenNIDevice() {
		//openni::OpenNI::shutdown();
	}

	void start() {
		bufferOffCnt = 0;
		if (mode & CameraOpenNI::MODE_IR || mode & CameraOpenNI::MODE_COLOUR) {
			assertValid(imageStream.start(), "CameraOpenNIDevice::start(): image.openni::VideoStream::start(): ");
			assertVideoMode(property, imageStream);
		}
		if (mode & CameraOpenNI::MODE_DEPTH) {
			assertValid(depthStream.start(), "CameraOpenNIDevice::start(): depth.openni::VideoStream::start(): ");
			assertVideoMode(property, depthStream);
		}
	}
	void stop() {
		if (mode & CameraOpenNI::MODE_DEPTH) {
			depthStream.stop();
		}
		if (mode & CameraOpenNI::MODE_IR) {
			imageStream.stop();
		}
	}
	void captureOnce(Image::Ptr& pImage) {
		bool bDepthStream = hasDepthStream;
		bool bImageStream = hasImageStream;
		openni::VideoStream* stream[2] = {&depthStream, &imageStream};
		golem::SecTmReal timeStamp = golem::SEC_TM_REAL_ZERO;

		// read streams
		do {
			int streamIndex;
			if (camera->isTerminating())
				return;
			if (!assertValid(openni::OpenNI::waitForAnyStream(hasDepthStream ? stream : stream + 1, hasDepthStream ? 2 : 1, &streamIndex, timeout), "CameraOpenNIDevice::captureOnce(): openni::OpenNI::waitForAnyStream(): ", false))
				continue;

			switch (streamIndex) {
			case 0:
				stream[hasDepthStream ? 0 : 1]->readFrame(hasDepthStream ? &depthFrame : &imageFrame);
				(hasDepthStream ? bDepthStream : bImageStream) = false;
				timeStamp = camera->getContext().getTimer().elapsed(); // TODO 
				break;
			case 1:
				stream[1]->readFrame(&imageFrame);
				bImageStream = false;
				break;
			}
		} while (bDepthStream || bImageStream);

		// initialize buffer
		if (pImage == nullptr)
			pImage.reset(new Image());
		if (pImage->cloud == nullptr) // should never happen
			pImage->cloud.reset(new PointSeq());

		if (!hasImageStream)
			pImage->release(); // could be already queued
		else
			pImage->reserve((int)property.width, (int)property.height, (int)property.depth, (int)property.channels);
		
		if (!hasDepthStream)
			pImage->cloud->clear();
		else if (pImage->cloud->width != (uint32_t)property.width || pImage->cloud->height != (uint32_t)property.height)
			pImage->resize(property.width, property.height);
		
		pImage->timeStamp = timeStamp;

		// ir stream
		if (mode & CameraOpenNI::MODE_IR) {
			const size_t inpStride = size_t(imageFrame.getStrideInBytes())/sizeof(openni::Grayscale16Pixel);
			const openni::Grayscale16Pixel* pInpWidth = (const openni::Grayscale16Pixel*)imageFrame.getData();
			openni::RGB888Pixel* pOutWidth = ((openni::RGB888Pixel*)pImage->image->imageData) + imageFrame.getCropOriginY()*property.width;

			openni::Grayscale16Pixel max = 0;
			for (int y = 0; y < imageFrame.getHeight(); ++y) {
				const openni::Grayscale16Pixel* pInp = pInpWidth + imageFrame.getWidth() - 1;

				for (int x = 0; x < imageFrame.getWidth(); ++x, --pInp) {
					if (max < *pInp)
						max = *pInp;
				}

				pInpWidth += inpStride;
			}
			const float scale = max > openni::Grayscale16Pixel(golem::numeric_const<uint8_t>::MAX) ? float(golem::numeric_const<uint8_t>::MAX)/float(max) : golem::numeric_const<float>::ONE;

			pInpWidth = (const openni::Grayscale16Pixel*)imageFrame.getData();

			for (int y = 0; y < imageFrame.getHeight(); ++y) {
				const openni::Grayscale16Pixel* pInp = pInpWidth + imageFrame.getWidth() - 1;
				openni::RGB888Pixel* pOut = pOutWidth + imageFrame.getCropOriginX();

				for (int x = 0; x < imageFrame.getWidth(); ++x, --pInp, ++pOut) {
					pOut->r = pOut->g = pOut->b = (uint8_t)(scale**pInp);
				}

				pInpWidth += inpStride;
				pOutWidth += property.width;
			}
		}
		// rgb stream
		else if (mode & CameraOpenNI::MODE_COLOUR) {
			const size_t inpStride = size_t(imageFrame.getStrideInBytes())/sizeof(openni::RGB888Pixel);
			const openni::RGB888Pixel* pInpWidth = (const openni::RGB888Pixel*)imageFrame.getData();
			openni::RGB888Pixel* pOutWidth = ((openni::RGB888Pixel*)pImage->image->imageData) + imageFrame.getCropOriginY()*property.width;

			for (int y = 0; y < imageFrame.getHeight(); ++y) {
				const openni::RGB888Pixel* pInp = pInpWidth + imageFrame.getWidth() - 1;
				openni::RGB888Pixel* pOut = pOutWidth + imageFrame.getCropOriginX();

				for (int x = 0; x < imageFrame.getWidth(); ++x, --pInp, ++pOut) {
					pOut->b = pInp->r;
					pOut->g = pInp->g;
					pOut->r = pInp->b;
				}

				pInpWidth += inpStride;
				pOutWidth += property.width;
			}
		}
		// depth stream
		if (mode & CameraOpenNI::MODE_DEPTH) {
			const float scale = U32(depthStream.getVideoMode().getPixelFormat()) == openni::PIXEL_FORMAT_DEPTH_100_UM ? float(0.0001) : float(0.001);
			const size_t inpStride = size_t(depthFrame.getStrideInBytes())/sizeof(openni::DepthPixel);
			const openni::DepthPixel* pInpWidth = (const openni::DepthPixel*)depthFrame.getData();
			const openni::RGB888Pixel* pInpImageWidth = hasImageStream ? ((openni::RGB888Pixel*)pImage->image->imageData) + depthFrame.getCropOriginY()*property.width : nullptr;
			golem::Point* pOutWidth = &pImage->cloud->front() + depthFrame.getCropOriginY()*property.width;

			for (int y = 0; y < depthFrame.getHeight(); ++y) {
				const openni::DepthPixel* pInp = pInpWidth + depthFrame.getWidth() - 1;
				const openni::RGB888Pixel* pInpImage = pInpImageWidth + depthFrame.getCropOriginX();
				golem::Point* pOut = pOutWidth + depthFrame.getCropOriginX();

				for (int x = 0; x < depthFrame.getWidth(); ++x, --pInp, ++pInpImage, ++pOut) {
					if (*pInp != 0) {
						(void)openni::CoordinateConverter::convertDepthToWorld(depthStream, x, y, *pInp, &pOut->x, &pOut->y, &pOut->z);
						pOut->x *= scale;
						pOut->y *= -scale;
						pOut->z *= scale;
					}
					else {
						pOut->x = pOut->y = pOut->z = std::numeric_limits<float>::quiet_NaN();
					}
					pOut->normal_x = pOut->normal_y = pOut->normal_z = golem::numeric_const<float>::ZERO;
					
					if (hasImageStream) {
						pOut->r = pInpImage->b;
						pOut->g = pInpImage->g;
						pOut->b = pInpImage->r;
						pOut->a = colour._rgba.a;
					}
					else {
						pOut->r = colour._rgba.r;
						pOut->g = colour._rgba.g;
						pOut->b = colour._rgba.b;
						pOut->a = colour._rgba.a;
					}
				}

				pInpWidth += inpStride;
				pInpImageWidth += property.width;
				pOutWidth += property.width;
			}
		}
	}
	void capture(Image::Ptr& pImage) {
		do
			captureOnce(pImage);
		while (!camera->isTerminating() && bufferOffCnt++ < bufferOff);
	}
	bool assertValid(const openni::Status& rc, const char* context, bool bThrow = true) {
		if (rc != openni::STATUS_OK) {
			if (bThrow)
				throw Message(Message::LEVEL_CRIT, "%s: %s", context, openni::OpenNI::getExtendedError());
			else
				camera->getContext().error("%s: %s", context, openni::OpenNI::getExtendedError());
			return false;
		}
		return true;
	}
	static openni::Status setVideoMode(U32 width, U32 height, Real fps, U32 format, openni::VideoStream& stream) {
		openni::VideoMode videoMode = stream.getVideoMode();
		videoMode.setPixelFormat((openni::PixelFormat)format);
		videoMode.setFps(golem::Math::round(fps));
		videoMode.setResolution((int)width, (int)height);
		return stream.setVideoMode(videoMode);
	}
	static void assertVideoMode(const Camera::Property& property, openni::VideoStream& stream) {
		openni::VideoMode videoMode = stream.getVideoMode();
		if (videoMode.getResolutionX() != (int)property.width || videoMode.getResolutionY() != (int)property.height)
			throw Message(Message::LEVEL_CRIT, "CameraOpenNIDevice::assertVideoMode(): %dx%d != %dx%d", videoMode.getResolutionX(), videoMode.getResolutionY(), property.width, property.height);
	}

private:
	/** Image buffer offset counter */
	size_t bufferOffCnt;
	/** Mode */
	golem::U32 mode;
	openni::Status rc;
	openni::Device device;
	openni::VideoStream depthStream, imageStream;
	openni::VideoFrameRef depthFrame, imageFrame;
	bool hasDepthStream, hasImageStream, hasRGBStream;
};

CameraOpenNI::CameraOpenNI(golem::Context& context) : CameraDepth(context) {
}

void CameraOpenNI::create(const Desc& desc) {
	desc.assertValid(Assert::Context("CameraOpenNI::Desc."));

	colour = desc.colour;
	bufferOff = desc.bufferOff;
	streamTimeOut = desc.streamTimeOut;
	device.reset(new CameraOpenNIDevice(this, desc.properties[0], desc.index, bufferOff, colour, (int)streamTimeOut));
	property = device->getProperty();
	properties = desc.properties;
	device.reset();

	CameraDepth::create(desc); // throws
}

golem::U32 CameraOpenNI::getMode(const std::string& str) {
	if (str.compare("depth") == 0 || str.compare("d") == 0)
		return CameraOpenNI::MODE_DEPTH;
	if (str.compare("colour") == 0 || str.compare("color") == 0 || str.compare("rgb") == 0)
		return CameraOpenNI::MODE_COLOUR;
	if (str.compare("ir") == 0)
		return CameraOpenNI::MODE_IR;
	throw golem::Message(golem::Message::LEVEL_CRIT, "CameraOpenNI::getMode(): Unknown camera mode %s", str.c_str());
}

void CameraOpenNI::start() {
	device.reset(new CameraOpenNIDevice(this, property, index, bufferOff, colour, (int)streamTimeOut));
	static_cast<CameraOpenNIDevice*>(device.get())->start();
}

void CameraOpenNI::stop() {
	static_cast<CameraOpenNIDevice*>(device.get())->stop();
	device.reset();
}


//------------------------------------------------------------------------------
