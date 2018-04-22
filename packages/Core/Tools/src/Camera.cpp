/** @file Camera.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Tools/Camera.h>
#include <Golem/Tools/Import.h>
#include <Golem/Sys/XMLData.h>
#include <opencv/cv.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(Camera::Property& val, golem::XMLContext* context, bool create) {
	golem::XMLData("width", val.width, context, create);
	golem::XMLData("height", val.height, context, create);
	golem::XMLData("fps", val.fps, context, create);
	golem::XMLData("mode", val.mode, context, create);
	golem::XMLData("format", val.format, context, create);
	try {
		std::string transform;
		golem::XMLData("transform", transform, context, create);
		StringSeq transformSeq;
		Camera::getStrings(transform, transformSeq);
		val.transform = 0;
		for (size_t i = 0; i < transformSeq.size(); ++i)
			val.transform |= Camera::getTransform(transformSeq[i]);
	}
	catch (const golem::MsgXMLParser&) {}
}

void golem::Camera::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Sensor::Desc::load(context, xmlcontext);

	golem::XMLData(calibrationDescMap, calibrationDescMap.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "calibration", false);
	golem::XMLData("calibration_file", calibrationFile, const_cast<golem::XMLContext*>(xmlcontext), false);
	
	golem::XMLData("index", index, const_cast<golem::XMLContext*>(xmlcontext), false);
	properties.clear();
	golem::XMLData(properties, properties.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "property", false);

	golem::XMLData("buffer_len", bufferLen, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("thread_timeout", threadTimeOut, const_cast<golem::XMLContext*>(xmlcontext), false);
}

//------------------------------------------------------------------------------

Camera::Camera(golem::Context& context) : Sensor(context), calibrationCurrentPtr(nullptr) {
}

Camera::~Camera() {
	bTerminate = true;
	evCmd.set(true);
	if (!thread.join(threadTimeOut))
		context.error("Camera::release(): Unable to stop camera thread\n");
}

void Camera::create(const Desc& desc) {
	Sensor::create(desc);

	calibrationMap.clear();
	for (CameraCalibration::Desc::Map::const_iterator i = desc.calibrationDescMap.begin(); i != desc.calibrationDescMap.end(); ++i)
		calibrationMap.insert(calibrationMap.end(), std::make_pair(i->first, i->second->create(*this)))->second->load(); // throws
	calibrationCurrentPtr = &*calibrationMap.begin();
	setCurrentCalibrationFile(desc.calibrationFile); // throws

	index = desc.index;
	if (properties.empty()) {
		properties = desc.properties;
		property = desc.properties[0];
	}

	bufferLen = desc.bufferLen;
	threadTimeOut = desc.threadTimeOut;

	context.info("Camera::create(): id=%s, index=%d, size=%dx%d, fps=%f, mode=%s, format=%s\n", getID().c_str(), index, property.width, property.height, property.fps, property.mode.c_str(), property.format.c_str());

	state = Camera::DEFAULT;
	bTerminate = false;
	if (!thread.start(this))
		throw golem::Message(golem::Message::LEVEL_CRIT, "Camera::create(): Unable to launch camera thread");
}

void Camera::setCurrentCalibrationFile(const std::string& calibrationFile) {
	CameraCalibration::Map::const_iterator calibrationCurrentPtr = calibrationMap.find(calibrationFile);
	if (calibrationCurrentPtr != calibrationMap.end())
		this->calibrationCurrentPtr = &*calibrationCurrentPtr; // atomic C++ pointer copy
	else
		throw Message(Message::LEVEL_ERROR, "Camera::setCurrentCalibrationFile(): invalid file name %s", calibrationFile.c_str());
}

//------------------------------------------------------------------------------

bool Camera::set(State state, golem::MSecTmU32 timeOut) {
	if (state != Camera::DEFAULT) {
		if (this->state == state || state == Camera::CMD_STOP && this->state == Camera::DEFAULT)
			return true;
		this->state = state;
		evData.set(false);
		evCmdComplete.set(false);
		evCmd.set(true);
		return evCmdComplete.wait(timeOut);
	}
	else {
		return this->state == Camera::DEFAULT;
	}
}

Camera::State Camera::get() {
	return state;
}

void Camera::run() {
	bool bStart = false;
	bool bVideo = true;
	golem::U32 captureIndex = 0;

	try {
		for (;;) {
			if (!bStart) {
				evCmd.set(false);
				(void)evCmd.wait();
			}

			if (bTerminate)
				break;

			Image::Ptr pImage;
			{
				golem::CriticalSectionWrapper csw(csData);
				if (!bStart) {
					buffer.insert(buffer.end(), images.begin(), images.end());
					images.clear();
					evData.set(false);
				}
				if (!buffer.empty()) {
					pImage = buffer.front();
					buffer.pop_front();
				}
				if (images.size() >= bufferLen) {
					if (pImage == nullptr) { // TODO make it nicer
						pImage = images.front();
						images.pop_front();
					}
				}
			}

			const bool bStop = state == Camera::CMD_STOP;
			const bool bVideo = state == Camera::CMD_VIDEO;

			if (bStop)
				goto STOP;
			if (!bStart) {
				start();
				bStart = true;
				evCmdComplete.set(true);
			}

			TRY:
			try {
				if (bTerminate)
					break;
				if (bStop)
					goto STOP;
				
				device->capture(pImage);
				
				if (pImage != nullptr) {
					pImage->captureIndex = captureIndex;
					if (this->property.transform != 0)
						transform(this->property.transform, *pImage);
				}
				++captureIndex;
			}
			catch (const golem::Message& msg) {
				context.error("Camera::run(): %s\n", msg.msg());
				goto TRY;
			}
			catch (const std::exception& ex) {
				context.error("Camera::run(): C++ exception: %s\n", ex.what());
				goto TRY;
			}

			{
				golem::CriticalSectionWrapper csw(csData);
				images.push_back(pImage);
				evData.set(true);
			}

			if (bVideo)
				continue;

			STOP:
			if (bStart) {
				stop();
				state = Camera::DEFAULT;
				bStart = false;
				evCmdComplete.set(true);
			}
		}
	}
	catch (const std::exception& ex) {
		context.write("Camera::run(): %s\n", ex.what());
	}
}

Image::Ptr Camera::pop_front(golem::MSecTmU32 timeOut) {
	Image::Ptr pImage;
	
	if (!evData.wait(timeOut))
		return pImage;
	
	golem::CriticalSectionWrapper csw(csData);

	if (images.empty())
		return pImage;

	pImage = images.front();
	images.pop_front();
	evData.set(!images.empty());

	return pImage;
}

Image::Ptr Camera::pop_back(golem::MSecTmU32 timeOut) {
	Image::Ptr pImage;
	
	if (!evData.wait(timeOut))
		return pImage;
	
	golem::CriticalSectionWrapper csw(csData);

	if (images.empty())
		return pImage;

	pImage = images.back();
	images.pop_back();
	//buffer.insert(buffer.end(), images.begin(), images.end());
	//images.clear();
	evData.set(!images.empty());
	
	return pImage;
}

void Camera::push(Image::Ptr pImage) {
	golem::CriticalSectionWrapper csw(csData);
	buffer.push_back(pImage);
}

size_t Camera::size() {
	golem::CriticalSectionWrapper csw(csData);
	return images.size();
}

void Camera::clear() {
	golem::CriticalSectionWrapper csw(csData);
	buffer.insert(buffer.end(), images.begin(), images.end());
	images.clear();
	evData.set(false);
}

void Camera::getStrings(const std::string& inp, StringSeq& out) {
	std::stringstream sstream(inp + " "); // HACK std::stringstream::read(&c, 1) reads one character too much without reporting eof
	IStream istream(sstream, "\t,;| ");
	while (!istream.eos())
		out.push_back(std::string(istream.next<const char*>()));
}

//------------------------------------------------------------------------------

golem::Mat34 Camera::getFrame() const {
	return getCurrentCalibration()->getFrame(true);
}

//------------------------------------------------------------------------------

golem::U32 golem::Camera::getTransform(const std::string& str) {
	if (str.compare("none") == 0 || str.compare("n") == 0 || str.length() <= 0)
		return Camera::TRANSFORM_NONE;
	if (str.compare("flip_horizontal") == 0)
		return Camera::TRANSFORM_FLIP_HORIZONTAL;
	if (str.compare("flip_vertical") == 0)
		return Camera::TRANSFORM_FLIP_VERTICAL;
	throw golem::Message(golem::Message::LEVEL_CRIT, "Camera::getTransform(): Unknown image transform %s", str.c_str());
}

void golem::Camera::transform(golem::U32 imageTransform, Image& image) const {
	if (imageTransform & (Camera::TRANSFORM_FLIP_HORIZONTAL | Camera::TRANSFORM_FLIP_VERTICAL))
		::cvFlip(image.image, nullptr, (imageTransform & Camera::TRANSFORM_FLIP_HORIZONTAL) && (imageTransform & Camera::TRANSFORM_FLIP_VERTICAL) ? -1 : imageTransform & Camera::TRANSFORM_FLIP_VERTICAL ? 1 : 0);
}

//------------------------------------------------------------------------------

void golem::CameraDepth::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Camera::Desc::load(context, xmlcontext);
	
	XMLData(colour, xmlcontext->getContextFirst("colour"), false);

	try	{
		XMLData(colourToIRFrame, xmlcontext->getContextFirst("colour_to_ir_frame"), false);
	}
	catch (const golem::MsgXMLParserNameNotFound&) {
		//context.verbose("CameraDepth::Desc::load(): no colourToIRFrame transform specified\n");
	}
}

CameraDepth::CameraDepth(golem::Context& context) : Camera(context) {
}

void CameraDepth::create(const Desc& desc) {
	Camera::create(desc);

	colour = desc.colour;
	colourToIRFrame = desc.colourToIRFrame;
}

//------------------------------------------------------------------------------

void CameraDepth::setColourToIRFrame(const golem::Mat34& frame) {
	colourToIRFrame = frame;
}

void CameraDepth::draw(const Appearance& appearance, golem::DebugRenderer& renderer) const {
	Sensor::draw(appearance, renderer);

	if (appearance.frameShow)
	{
		const Mat34 C = getFrame();
		Mat34 depthCameraFrame;
		depthCameraFrame.multiply(C, colourToIRFrame);

		renderer.addAxes3D(depthCameraFrame, appearance.frameSize * 0.5);
	}
}
