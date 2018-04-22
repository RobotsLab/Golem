/** @file PointGrey.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sensor/CameraPointGrey/CameraPointGrey.h>
#include <FlyCapture2.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new CameraPointGrey::Desc();
}

//------------------------------------------------------------------------------

class CameraPointGreyDevice : public Camera::Device {
public:
	CameraPointGreyDevice(Camera* camera, const Camera::Property& property, golem::I32 index) : Camera::Device(camera, property) {
		// find number of cameras
		FlyCapture2::BusManager busMgr;
		unsigned int numCameras;
		assertValid(busMgr.GetNumOfCameras(&numCameras), "CameraPointGreyDevice::create(): BusManager::GetNumOfCameras(): ");
		if (numCameras <= 0)
			throw golem::Message(golem::Message::LEVEL_CRIT, "CameraPointGreyDevice::create(): No cameras detected");
		if (numCameras <= (unsigned)index)
			throw golem::Message(golem::Message::LEVEL_CRIT, "CameraPointGreyDevice::create(): Invalid camera index");

		FlyCapture2::PGRGuid guid;
		assertValid(busMgr.GetCameraFromIndex((unsigned)index, &guid), "CameraPointGreyDevice::create(): BusManager::GetCameraFromIndex(): ");
		assertValid(device.Connect(&guid), "CameraPointGreyDevice::create(): Camera::Connect(): ");

		// Get the camera information
		FlyCapture2::CameraInfo cameraInfo;
		assertValid(device.GetCameraInfo(&cameraInfo), "CameraPointGreyDevice::create(): Camera::GetCameraInfo(): ");
		camera->getContext().debug("CameraPointGreyDevice::create(): serial: %u, model: %s, vendor: %s, sensor: %s, resolution: %s, firmware: %s\n",
			cameraInfo.serialNumber, cameraInfo.modelName, cameraInfo.vendorName, cameraInfo.sensorInfo, cameraInfo.sensorResolution, cameraInfo.firmwareVersion);

		// Query for available Format 7 modes
		FlyCapture2::Format7Info format7Info;
		bool supported;
		format7Info.mode = (FlyCapture2::Mode)std::stoul(this->property.mode);
		assertValid(device.GetFormat7Info(&format7Info, &supported), "CameraPointGreyDevice::create(): Camera::GetFormat7Info(): ");
		camera->getContext().debug("CameraPointGreyDevice::create(): Max image pixels: (%u, %u), image unit size: (%u, %u), offset unit size: (%u, %u), pixel format bitfield: 0x%08x\n",
			format7Info.maxWidth, format7Info.maxHeight, format7Info.imageHStepSize, format7Info.imageVStepSize, format7Info.offsetHStepSize, format7Info.offsetVStepSize, format7Info.pixelFormatBitField);

		// new settings
		FlyCapture2::Format7ImageSettings format7ImageSettings;
		format7ImageSettings.pixelFormat = (FlyCapture2::PixelFormat)std::stoul(this->property.format);
		if ((format7ImageSettings.pixelFormat & format7Info.pixelFormatBitField) == 0)
			throw golem::Message(golem::Message::LEVEL_CRIT, "CameraPointGreyDevice::create(): Desired pixel format is not supported");
		format7ImageSettings.mode = format7Info.mode;
		format7ImageSettings.offsetX = 0;
		format7ImageSettings.offsetY = 0;
		format7ImageSettings.width = this->property.width > 0 ? this->property.width : format7Info.maxWidth;
		format7ImageSettings.height = this->property.height > 0 ? this->property.height : format7Info.maxHeight;

		// Validate the settings to make sure that they are valid
		bool valid;
		FlyCapture2::Format7PacketInfo format7PacketInfo;
		assertValid(device.ValidateFormat7Settings(&format7ImageSettings, &valid, &format7PacketInfo), "CameraPointGreyDevice::create(): Camera::ValidateFormat7Settings(): ");
		if (!valid)
			throw golem::Message(golem::Message::LEVEL_CRIT, "CameraPointGreyDevice::create(): Format7 settings are not valid");

		// Set the settings to the camera
		assertValid(device.SetFormat7Configuration(&format7ImageSettings, format7PacketInfo.recommendedBytesPerPacket), "CameraPointGreyDevice::create(): Camera::SetFormat7Configuration(): ");

		// Retrieve camera video mode
		FlyCapture2::Property frameRate;
		frameRate.type = FlyCapture2::FRAME_RATE;
		assertValid(device.StartCapture(), "CameraPointGreyDevice::create(): Camera::StartCapture(): ");
		assertValid(device.GetProperty(&frameRate), "CameraPointGreyDevice::create(): Camera::GetProperty(): ");
		assertValid(device.StopCapture(), "CameraPointGreyDevice::create(): Camera::StopCapture(): ");
		this->property.width = (U32)format7ImageSettings.width;
		this->property.height = (U32)format7ImageSettings.height;
		this->property.fps = Real(frameRate.absValue);
		this->property.channels = 3; // fixed
		this->property.depth = IPL_DEPTH_8U; // fixed
	}
	~CameraPointGreyDevice() {
		// Disconnect the camera
		assertValid(device.Disconnect(), "CameraPointGreyDevice::release(): Camera::Disconnect(): ");
	}
	void capture(Image::Ptr& pImage) {
		// Retrieve an image
		const golem::SecTmReal systemTime1 = camera->getContext().getTimer().elapsed();
		assertValid(device.RetrieveBuffer(&rawImage), "CameraPointGreyDevice::capture(): Camera::StopCapture(): ");
		const golem::SecTmReal systemTime2 = camera->getContext().getTimer().elapsed();

		// Get the raw image dimensions
		FlyCapture2::PixelFormat pixelFormat;
		unsigned int rows, cols, stride;
		rawImage.GetDimensions(&rows, &cols, &stride, &pixelFormat);

		// initialize buffer
		if (pImage == nullptr)
			pImage.reset(new Image());
		pImage->reserve((int)cols, (int)rows);

		// time stamp
		pImage->timeStamp = REAL_HALF*(systemTime1 + systemTime2);

		// time stamp (works only for Firewire cameras)
		//FlyCapture2::TimeStamp timeStampFC1;
		//const golem::SecTmReal systemTime1 = camera->getContext().getTimer().elapsed();
		//assertValid(device.GetCycleTime(&timeStampFC1), "CameraPointGreyDevice::capture(): Camera::GetCycleTime(): ");
		//const golem::SecTmReal systemTime2 = camera->getContext().getTimer().elapsed();
		//const FlyCapture2::TimeStamp timeStampFC2 = rawImage.GetTimeStamp();
		//const golem::SecTmReal cycleTime = golem::SecTmReal(timeStampFC1.seconds) + golem::SecTmReal(1e6)*timeStampFC1.microSeconds;
		//const golem::SecTmReal captureTime = golem::SecTmReal(timeStampFC2.seconds) + golem::SecTmReal(1e6)*timeStampFC2.microSeconds;
		//pImage->timeStamp = REAL_HALF*(systemTime1 + systemTime2) + captureTime - cycleTime;
		//camera->getContext().write("t=%f, cycleTime=%f, captureTime=%f\n", pImage->timeStamp, cycleTime, captureTime);

		// Convert the raw image
		FlyCapture2::Image convertedImage((unsigned char*)pImage->image->imageData, cols*rows*property.channels);
		assertValid(rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &convertedImage), "CameraPointGreyDevice::capture(): Image::Convert(): ");
	}
	void start() {
		// stats
		assertValid(device.ResetStats(), "CameraPointGreyDevice::start(): Camera::ResetStats(): ");
		// start
		assertValid(device.StartCapture(), "CameraPointGreyDevice::start(): Camera::StartCapture(): ");
	}
	void stop() {
		// stop
		assertValid(device.StopCapture(), "CameraPointGreyDevice::stop(): Camera::StopCapture(): ");
		// stats
		FlyCapture2::CameraStats cameraStats;
		assertValid(device.GetStats(&cameraStats), "CameraPointGreyDevice::stop(): Camera::GetStats(): ");
		if (cameraStats.imageDropped > 0 || cameraStats.imageCorrupt > 0)
			camera->getContext().debug("CameraPointGreyDevice::stop(): imageDropped=%u, imageCorrupt=%u\n", cameraStats.imageDropped, cameraStats.imageCorrupt);
	}

private:
	/** Camera */
	FlyCapture2::Camera device;
	/** Raw image */
	FlyCapture2::Image rawImage;    

	void assertValid(const FlyCapture2::Error& error, const char* context, bool bThrow = true) {
		if (error != FlyCapture2::PGRERROR_OK)
			if (bThrow)
				throw Message(Message::LEVEL_CRIT, "%s: %s", context, error.GetDescription());
			else
				camera->getContext().error("%s: %s\n", context, error.GetDescription());
	}
};

CameraPointGrey::CameraPointGrey(golem::Context& context) : Camera(context) {
}

void CameraPointGrey::create(const Desc& desc) {
	desc.assertValid(Assert::Context("CameraPointGrey::Desc."));

	device.reset(new CameraPointGreyDevice(this, desc.properties[0], desc.index)); // throws
	properties = desc.properties;
	property = device->getProperty();

	Camera::create(desc); // throws
}

void CameraPointGrey::start() {
	static_cast<CameraPointGreyDevice*>(device.get())->start();
}

void CameraPointGrey::stop() {
	static_cast<CameraPointGreyDevice*>(device.get())->stop();
}

//------------------------------------------------------------------------------
