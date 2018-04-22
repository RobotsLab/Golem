/** @file Video.cpp
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sys/XMLData.h>
#include <Golem/Tools/Import.h>
#include <Golem/Tools/Camera.h>
#include <Golem/Data/Image/Image.h>
#include <Golem/Data/Video/Video.h>
#include <boost/algorithm/string.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>

using namespace golem;
using namespace golem::data;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new HandlerVideo::Desc();
}

//------------------------------------------------------------------------------

template <> void Stream::read(golem::ConfigMat34& pose) const {
	pose.c.clear();
	read(pose.c, pose.c.begin());
	read(pose.w);
}
template <> void Stream::write(const golem::ConfigMat34& pose) {
	write(pose.c.begin(), pose.c.end());
	write(pose.w);
}

template <> void golem::Stream::read(golem::data::ItemVideo::Frame& frame) const {
	read(frame.timeStamp);
	read(frame.config);
	read(frame.cloudPose);
}
template <> void golem::Stream::write(const golem::data::ItemVideo::Frame& frame) {
	write(frame.timeStamp);
	write(frame.config);
	write(frame.cloudPose);
}

//------------------------------------------------------------------------------

golem::data::ItemVideo::ItemVideo(HandlerVideo& handler) : Item(handler), handler(handler), videoFile(handler.file), frameFile(handler.file), frameIndex(0), useCloudFrame(false) {
}

Item::Ptr golem::data::ItemVideo::clone() const {
	throw Message(Message::LEVEL_ERROR, "ItemVideo::clone(): not implemented");
}

void golem::data::ItemVideo::createRender() {
	handler.createRender(*this);
}

void golem::data::ItemVideo::load(const std::string& prefix, const golem::XMLContext* xmlcontext) {
	// parameters
	golem::XMLData(parameters, const_cast<golem::XMLContext*>(xmlcontext), false);

	// frame index
	golem::XMLData("frame_index", frameIndex, const_cast<golem::XMLContext*>(xmlcontext), false);
	// use a single cloud with frames?
	golem::XMLData("use_cloud_frame", useCloudFrame, const_cast<golem::XMLContext*>(xmlcontext), false);

	// frames
	std::string frameSuffix;
	golem::XMLData("frame", frameSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	frame.clear();
	frameFile.load(prefix + frameSuffix, [&] (const std::string& path) {
		FileReadStream(path.c_str()).read(frame, frame.begin());
	});

	// video
	golem::XMLData("video", videoSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	// cloud
	golem::XMLData("cloud", cloudSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (cloudSuffix.length() > 0) {
		cloudFile.clear(); // this will remove temporary files
		cloudFile.resize(useCloudFrame ? 1 : frame.size());
		for (File::Seq::iterator i = cloudFile.begin(); i != cloudFile.end(); ++i)
			i->reset(new File(handler.file));
	}

	// load from file
	this->prefix = prefix;
	load();
}

void golem::data::ItemVideo::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	// parameters
	golem::XMLData(const_cast<CameraCalibration::Parameters&>(parameters), xmlcontext, true);

	// frame index
	golem::XMLData("frame_index", const_cast<golem::U32&>(frameIndex), xmlcontext, true);
	// use a single cloud with frames?
	golem::XMLData("use_cloud_frame", const_cast<bool&>(useCloudFrame), xmlcontext, true);

	// frames
	golem::XMLData("frame", const_cast<std::string&>(handler.frameSuffix), xmlcontext, true);

	// video xml
	std::string videoSuffix = image != nullptr ? handler.videoSuffix : "";
	golem::XMLData("video", videoSuffix, xmlcontext, true);

	// cloud xml
	std::string cloudSuffix = cloud != nullptr && !cloud->empty() ? handler.cloudSuffix : "";
	golem::XMLData("cloud", cloudSuffix, xmlcontext, true);

	// frames binaries
	frameFile.setTemporary(false); // no longer temporary
	frameFile.save(prefix + handler.frameSuffix, [] (const std::string& path) {
		// ignore - direct write not supported
	});

	// video binary
	if (videoSuffix.length() > 0) {
		videoFile.setTemporary(false); // no longer temporary
		videoFile.save(prefix + videoSuffix, [] (const std::string& path) {
			// ignore - direct write not supported
		});
	}
	else
		videoFile.remove();

	// cloud binary
	for (File::Seq::const_iterator i = cloudFile.begin(); i != cloudFile.end(); ++i) {
		(*i)->setTemporary(false); // no longer temporary
		if (cloudSuffix.length() > 0) {
			(*i)->save(prefix + handler.cloudIndex(i - cloudFile.begin()) + cloudSuffix, [] (const std::string& path) {
				// ignore - direct write not supported
			});
		}
		else
			(*i)->remove();
	}
}

void golem::data::ItemVideo::load() {
	if (frame.empty())
		throw Message(Message::LEVEL_ERROR, "ItemVideo::load(): no frames");

	if (frameIndex >= frame.size())
		frameIndex  = frame.size() - 1;
	
	// update time stamp
	timeStamp = frame[frameIndex].timeStamp.timeStamp;

	if (videoSuffix.length() > 0) {
		videoFile.load(prefix + videoSuffix, [&] (const std::string& path) {
			boost::shared_ptr<CvCapture> capture(cvCreateFileCapture(path.c_str()), [] (CvCapture* p) { cvReleaseCapture(&p); });
			if (capture == nullptr)
				throw Message(Message::LEVEL_ERROR, "ItemVideo::load(): unable to create CvCapture");
			if (!cvSetCaptureProperty(capture.get(), CV_CAP_PROP_POS_FRAMES, frameIndex))
				throw Message(Message::LEVEL_ERROR, "ItemVideo::load(): invalid frame %u", frameIndex);
			ImageData::set(cvQueryFrame(capture.get())); // no memory leak here - see OpenCV documentation
		});
	}

	if (cloudSuffix.length() > 0) {
		const golem::U32 cloudIndex = useCloudFrame ? 0 : frameIndex;
		for (File::Seq::const_iterator i = cloudFile.begin(); i != cloudFile.end(); ++i)
			(*i)->load(prefix + handler.cloudIndex(i - cloudFile.begin()) + cloudSuffix, [&] (const std::string& path) {
			if ((i - cloudFile.begin()) == cloudIndex) {
					// block annoying pcl console messages
					const pcl::console::VERBOSITY_LEVEL pclLevel = pcl::console::getVerbosityLevel();
					golem::ScopeGuard guard([=] () { pcl::console::setVerbosityLevel(pclLevel); });
					pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_ALWAYS);
					if (cloud == nullptr)
						cloud.reset(new PointSeq());
					if (pcl::PCDReader().read(path, *cloud) != 0)
						throw golem::Message(golem::Message::LEVEL_ERROR, "ItemVideo::load().pcl::PCDReader(): unable to read from %s", path.c_str());
				}
			});
	}
}

//------------------------------------------------------------------------------

Item::Ptr golem::data::ItemVideo::convert(const Handler& handler) {
	return this->handler.convert(*this, handler);
}

const StringSeq& golem::data::ItemVideo::getConvertInterfaces() const {
	return this->handler.convertInterfaces;
}

bool golem::data::ItemVideo::isConvertSupported(const Handler& handler) const {
	return this->handler.isConvertSupported(handler);
}

//------------------------------------------------------------------------------

void golem::data::HandlerVideo::VideoAppearance::xmlData(golem::XMLContext* context, bool create) const {
	golem::XMLData("show", const_cast<bool&>(show), context, create);
	golem::XMLData("show_camera_frame", const_cast<bool&>(showCameraFrame), context, create);
}

void golem::data::HandlerVideo::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	golem::data::Handler::Desc::load(context, xmlcontext);

	golem::XMLData("tmp_dir", tmpDir, const_cast<golem::XMLContext*>(xmlcontext), false);

	cloudAppearance.xmlData(xmlcontext->getContextFirst("cloud appearance", false), false);
	golem::XMLData("suffix", cloudSuffix, xmlcontext->getContextFirst("cloud", false), false);

	try {
		XMLData(regionCaptureDesc, regionCaptureDesc.max_size(), xmlcontext->getContextFirst("cloud region_capture"), "bounds", false);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}

	videoAppearance.xmlData(xmlcontext->getContextFirst("video appearance", false), false);
	golem::XMLData("suffix", videoSuffix, xmlcontext->getContextFirst("video", false), false);
	golem::XMLData("format", videoFormat, xmlcontext->getContextFirst("video", false), false);
	golem::XMLData("fps", videoFPS, xmlcontext->getContextFirst("video", false), false);

	golem::XMLData("frame_suffix", frameSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
}

golem::data::Handler::Ptr golem::data::HandlerVideo::Desc::create(golem::Context &context) const {
	Handler::Ptr handler(new HandlerVideo(context));
	to<HandlerVideo>(handler.get())->create(*this);
	return handler;
}

//------------------------------------------------------------------------------

golem::data::HandlerVideo::HandlerVideo(golem::Context &context) : Handler(context), imageID(0), videoFrameRequest(0) {
}

void golem::data::HandlerVideo::create(const Desc& desc) {
	golem::data::Handler::create(desc);

	try {
		tmpDir = desc.tmpDir;
		if (tmpDir.empty())
			tmpDir = "./";
		else {
			if (tmpDir[tmpDir.length() - 1] != '/' || tmpDir[tmpDir.length() - 1] != '\\')
				tmpDir += "/";
			golem::mkdir(tmpDir.c_str());
		}
	}
	catch (const golem::Message& msg) {
		context.warning("data::HandlerVideo::create(): Video capture will not work correctly: %s\n", msg.msg());
	}

	cloudAppearance = desc.cloudAppearance;
	cloudSuffix = desc.cloudSuffix;

	for (Bounds::Desc::Seq::const_iterator i = desc.regionCaptureDesc.begin(); i != desc.regionCaptureDesc.end(); ++i)
		regionCapture.push_back((*i)->create());

	videoAppearance = desc.videoAppearance;
	videoSuffix = desc.videoSuffix;
	videoFormat = desc.videoFormat;
	videoFPS = desc.videoFPS;

	frameSuffix = desc.frameSuffix;

	convertInterfaces = {
		"Image",
	};
}


//------------------------------------------------------------------------------

golem::data::Item::Ptr golem::data::HandlerVideo::create() const {
	return Item::Ptr(new ItemVideo(*const_cast<HandlerVideo*>(this))); // safe: no modification here - only passing reference
}

//------------------------------------------------------------------------------

Item::Ptr golem::data::HandlerVideo::capture(golem::Sensor& sensor, Control control) {
	Camera* camera = dynamic_cast<Camera*>(&sensor);
	if (camera == nullptr)
		throw Message(Message::LEVEL_ERROR, "HandlerVideo::capture(): Camera required");

	Item::Ptr item(create());
	ItemVideo* itemVideo = to<ItemVideo>(item.get());
	CameraDepth* cameraDepth = is<CameraDepth>(camera);

	if (control == nullptr)
		throw Message(Message::LEVEL_ERROR, "HandlerVideo::capture(): camera control required");

	ScopeGuard guardProperty([&] { if (cameraDepth) cameraDepth->setProperty(cameraDepth->getImageProperty()); }); // execute last
	if (cameraDepth) cameraDepth->setProperty(cameraDepth->getDepthProperty());

	ScopeGuard guardCMD([&] { camera->set(Camera::CMD_STOP); }); // execute first

	if (!camera->set(Camera::CMD_VIDEO))
		throw Message(Message::LEVEL_ERROR, "HandlerVideo::capture(): unable to start capture");

	// create files in temporary folder
	itemVideo->prefix = makeString("%s~%6.6f", tmpDir.c_str(), context.getTimer().elapsed());
	itemVideo->frameFile.setTemporary(true);
	itemVideo->frameFile.save(itemVideo->prefix + frameSuffix, [] (const std::string& path) {});

	// properties
	const Real videoFPS = camera->getProperty().fps > SEC_TM_REAL_ZERO ? camera->getProperty().fps : this->videoFPS;

	// frames
	boost::shared_ptr<CvVideoWriter> videoWriter;

	SecTmReal stopFrame = SEC_TM_REAL_MAX;
	for (itemVideo->frameIndex = 0;; ++itemVideo->frameIndex) {
		// synchronisation
		if (itemVideo->frameIndex == 0) {
			if (!control(nullptr))
				throw Cancel("HandlerVideo::capture(): cancelled");
			// clear image queue
			itemVideo->timeStamp = context.getTimer().elapsed();
			itemVideo->captureIndex = -1;
		}

		// retrieve a next frame
		for (bool stop = false;;) {
			camera->waitAndPeek([&](const Image::List& images, bool result) {
				if (images.empty())
					throw Message(Message::LEVEL_ERROR, "HandlerVideo::capture(): empty image sequence");
				// search from the back, i.e. the latest frames
				Image::List::const_reverse_iterator frame = images.rend();
				for (Image::List::const_reverse_iterator i = images.rbegin(); i != images.rend() && (*i)->timeStamp > itemVideo->timeStamp; ++i)
					frame = i;
				if (frame == images.rend() && stopFrame != SEC_TM_REAL_MAX) {
					stop = true;
					stopFrame = SEC_TM_REAL_ZERO; // no more frames, stop now
				}
				if (frame != images.rend() && *frame != nullptr) {
					if (itemVideo->captureIndex != -1 && itemVideo->captureIndex + 1 < (*frame)->captureIndex)
						context.warning("HandlerVideo::capture(): %u frames dropped\n", (*frame)->captureIndex - itemVideo->captureIndex - 1);
					itemVideo->set(**frame);
					stop = true;
				}
			}, stopFrame == SEC_TM_REAL_MAX ? golem::MSEC_TM_U32_INF : 0);
			if (stop)
				break;
		}

		// stop?
		if (stopFrame == SEC_TM_REAL_ZERO)
			break;

		// video
		if (itemVideo->image != nullptr) {
			// enable
			if (itemVideo->frameIndex == 0)
				itemVideo->videoSuffix = videoSuffix;
			// create video writer
			if (videoWriter == nullptr) {
				itemVideo->videoFile.save(itemVideo->prefix + itemVideo->videoSuffix, [] (const std::string& path) {});
				
				videoWriter.reset(cvCreateVideoWriter(itemVideo->videoFile.getPath().c_str(), !videoFormat.compare("-1") ? -1 : CV_FOURCC(videoFormat[0], videoFormat[1], videoFormat[2], videoFormat[3]), videoFPS, cvSize(itemVideo->image->width, itemVideo->image->height)), [] (CvVideoWriter* p) { cvReleaseVideoWriter(&p); });
				if (videoWriter == nullptr)
					throw Message(Message::LEVEL_ERROR, "HandlerVideo::capture(): unable to create CvVideoWriter: path=%s, codec=%s, size=%ux%u, fps=%f", itemVideo->videoFile.getPath().c_str(), videoFormat.c_str(), itemVideo->image->width, itemVideo->image->height, videoFPS);

				itemVideo->videoFile.setTemporary(true);
			}
			// write frame
			if (videoWriter != nullptr) {
				cvWriteFrame(videoWriter.get(), itemVideo->image);
			}
		}
		// depth
		if (cameraDepth) {
			// enable
			if (itemVideo->frameIndex == 0)
				itemVideo->cloudSuffix = cloudSuffix;
			// camera frame, use deformation map (can be overridden in CameraCalibration::enableDeformationMap())
			const Mat34 cameraFrame = camera->getFrame();
			// transform points
			Cloud::transform(cameraFrame, *itemVideo->cloud, *itemVideo->cloud);
			// region clipping, single thread version WITHOUT Parallels
			if (!regionCapture.empty())
				Cloud::regionClip(regionCapture, *itemVideo->cloud, Cloud::isNanXYZ<Cloud::Point>);

			File::Ptr cloudFile(new File(file)); // pointer required to maintain a single copy, i.e. single file ownership!
			cloudFile->setTemporary(true);
			cloudFile->save(itemVideo->prefix + cloudIndex(itemVideo->frameIndex) + itemVideo->cloudSuffix, [] (const std::string& path) {});
			try {
				pcl::PCDWriter().writeBinaryCompressed(cloudFile->getPath(), *itemVideo->cloud);
			}
			catch (std::exception& ex) {
				throw golem::Message(golem::Message::LEVEL_ERROR, "HandlerVideo::capture().pcl::pcdWrite(): unable to write to %s (%s)", cloudFile->getPath().c_str(), ex.what());
			}
			itemVideo->cloudFile.push_back(cloudFile);
		}
		// append frame
		{
			ItemVideo::Frame frame;
			frame.timeStamp = itemVideo->timeStamp;
			camera->getConfig(frame.config);
			frame.cloudPose.setId(); // id pose because each frame has different cloud
			itemVideo->frame.push_back(frame);
		}

		// stop?
		if (stopFrame == SEC_TM_REAL_MAX && !control(itemVideo)) {
			stopFrame = context.getTimer().elapsed();
			camera->set(Camera::CMD_STOP);
		}
		if (stopFrame < itemVideo->timeStamp)
			break;
	}

	// write frames
	FileWriteStream(itemVideo->frameFile.getPath().c_str()).write(itemVideo->frame.begin(), itemVideo->frame.end());
	// type of point cloud data
	itemVideo->useCloudFrame = false;

	// camera parameters
	itemVideo->parameters = camera->getCurrentCalibration()->getParameters();

	return item;
}

//------------------------------------------------------------------------------

Item::Ptr golem::data::HandlerVideo::convert(ItemVideo& itemVideo, const Handler& handler) {
	// only one type of output
	Item::Ptr item(handler.create());
	ItemImage* itemImage = is<ItemImage>(item.get());
	if (!itemImage)
		throw Message(Message::LEVEL_ERROR, "HandlerVideo::convert(): Item %s does not support ItemImage interface", handler.getType().c_str());

	if (getUICallback() && getUICallback()->hasInputEnabled()) {
		//context.write("Press a key to: accept <Enter>, (()) to select video frame...");

		//for (bool accept = false;;) {
		//	createRender(itemVideo);

		//	if (accept) break;

		//	const int key = getUICallback()->waitKey();
		//	switch (key) {
		//	case 27: throw Cancel("\nCancelled");
		//	case '\x0D': accept = true; context.write("\n");  break;
		//	case '5': if (itemVideo.frameIndex > 0) { --itemVideo.frameIndex; itemVideo.load(); } break;
		//	case '6': if (itemVideo.frameIndex < itemVideo.frame.size() - 1) { ++itemVideo.frameIndex; itemVideo.load(); } break;
		//	};
		//	context.write("Video frame #%u/%u\n", itemVideo.frameIndex + 1, itemVideo.frame.size());
		//}
	}

	// copy current image, cloud and time stamp
	itemImage->set(itemVideo);
	// transform cloud
	if (itemVideo.useCloudFrame)
		Cloud::transform(itemVideo.frame[itemVideo.frameIndex].cloudPose, *itemImage->cloud, *itemImage->cloud);
	// copy current config
	itemImage->config = itemVideo.frame[itemVideo.frameIndex].config;

	itemImage->imageFile.setModified(true);
	itemImage->cloudFile.setModified(true);

	return item;
}

bool golem::data::HandlerVideo::isConvertSupported(const Handler& handler) const {
	return handler.isItem<const data::ItemImage>();
}

//------------------------------------------------------------------------------

std::string golem::data::HandlerVideo::cloudIndex(golem::U32 frame) const {
	return makeString("-%06d", frame);
}

//------------------------------------------------------------------------------

void golem::data::HandlerVideo::createRender(const ItemVideo& item) {
	UI::CriticalSectionWrapper cs(getUICallback());

	if (videoFrameRequest != 0) {
		const_cast<ItemVideo&>(item).frameIndex = videoFrameRequest < 0 ? std::max((I32)0, (I32)item.frameIndex + videoFrameRequest) : std::min((I32)item.frame.size() - 1, (I32)item.frameIndex + videoFrameRequest);
		context.write("Video frame #%u/%u\n", item.frameIndex + 1, item.frame.size());
		const_cast<ItemVideo&>(item).load();
		videoFrameRequest  = 0;
	}

	if (videoAppearance.show && item.image) {
		this->image.reserve(item.image);
		::cvConvertImage(item.image, this->image.image, CV_CVTIMG_SWAP_RB);
	}
	else {
		this->image.release();
	}
	imageID = 0;

	cloudRenderer.reset();
	if (item.cloud != nullptr)
		cloudAppearance.drawPoints(*item.cloud, cloudRenderer, item.useCloudFrame ? &item.frame[item.frameIndex].cloudPose : nullptr);
}

void golem::data::HandlerVideo::render() const {
	cloudRenderer.render();
}

void golem::data::HandlerVideo::customRender() const {
	if (videoAppearance.show && image.image) {
		image.draw(&imageID);
	}
}

//------------------------------------------------------------------------------

void golem::data::HandlerVideo::mouseHandler(int button, int state, int x, int y) {
}

void golem::data::HandlerVideo::motionHandler(int x, int y) {
}

void golem::data::HandlerVideo::keyboardHandler(int key, int x, int y) {
	switch (key) {
	case '3':
		videoAppearance.show = !videoAppearance.show;
		context.write("Video turn %s\n", videoAppearance.show ? "ON" : "OFF");
		requestRender();
		break;
	case '4':
		cloudAppearance.mode = Cloud::Appearance::Mode((cloudAppearance.mode + 1)%(Cloud::Appearance::MODE_NORMAL + 1));
		context.write("Cloud appearance: %s\n", Cloud::Appearance::ModeName[size_t(cloudAppearance.mode)]);
		requestRender();
		break;
	case '(':
		videoFrameRequest = -1;
		requestRender();
		break;
	case ')':
		videoFrameRequest = +1;
		requestRender();
		break;
	};
}

//------------------------------------------------------------------------------
