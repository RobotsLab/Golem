/** @file Image.cpp
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
#include <Golem/Data/Image/Image.h>
#include <Golem/Tools/Menu.h>
#include <Golem/Tools/Camera.h>
#include <boost/algorithm/string.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace golem;
using namespace golem::data;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new HandlerImage::Desc();
}

//------------------------------------------------------------------------------

golem::data::ItemImage::ItemImage(HandlerImage& handler) : Item(handler), handler(handler), imageFile(handler.file), cloudFile(handler.file) {
	normalStdDev = handler.normalStdDev;
}

Item::Ptr golem::data::ItemImage::clone() const {
	Item::Ptr item(handler.create());
	ItemImage* itemImage = to<ItemImage>(item.get());
	// Image
	itemImage->set((const Image&)*this);
	// ItemImage
	itemImage->parameters = parameters;
	itemImage->config = config;
	itemImage->imageFile.setModified(true);
	itemImage->cloudFile.setModified(true);
	itemImage->normalStdDev = normalStdDev;
	// done!
	return item;
}

void golem::data::ItemImage::createRender() {
	handler.createRender(*this);
}

void golem::data::ItemImage::load(const std::string& prefix, const golem::XMLContext* xmlcontext) {
	// time stamp
	TimeStamp::xmlData(const_cast<golem::XMLContext*>(xmlcontext), false);
	
	// parameters
	golem::XMLData(parameters, const_cast<golem::XMLContext*>(xmlcontext), false);

	// config
	config.xmlData(const_cast<golem::XMLContext*>(xmlcontext->getContextFirst("config", false)), false);

	// image
	std::string imageSuffix;
	golem::XMLData("image", imageSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (imageSuffix.length() > 0) {
		imageFile.load(prefix + imageSuffix, [&] (const std::string& path) {
			release();
			if ((image = ::cvLoadImage(path.c_str())) == nullptr)
				throw golem::Message(golem::Message::LEVEL_ERROR, "ItemImage::load().cvLoadImage(): unable to read from %s", path.c_str());
		});
	}
	
	// cloud
	std::string cloudSuffix;
	golem::XMLData("cloud", cloudSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (cloudSuffix.length() > 0) {
		cloudFile.load(prefix + cloudSuffix, [&] (const std::string& path) {
			if (cloud == nullptr)
				cloud.reset(new PointSeq());
			Cloud::load(handler.getContext(), path, *cloud);
		});
	}
}

void golem::data::ItemImage::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	// time stamp
	TimeStamp::xmlData(xmlcontext, true);

	// parameters
	golem::XMLData(const_cast<CameraCalibration::Parameters&>(parameters), xmlcontext, true);

	// config
	config.xmlData(xmlcontext->getContextFirst("config", true), true);

	// image xml
	std::string imageSuffix = image != nullptr ? handler.imageSuffix : "";
	golem::XMLData("image", imageSuffix, xmlcontext, true);

	// cloud xml
	std::string cloudSuffix = cloud != nullptr && !cloud->empty() ? handler.cloudSuffix : "";
	golem::XMLData("cloud", cloudSuffix, xmlcontext, true);

	// image binary
	if (imageSuffix.length() > 0) {
		imageFile.save(prefix + imageSuffix, [=] (const std::string& path) {
			if (!::cvSaveImage(path.c_str(), image, handler.imageFormat.empty() ? nullptr : handler.imageFormat.data()))
				throw golem::Message(golem::Message::LEVEL_ERROR, "ItemImage::save().cvSaveImage(): unable to write to %s", path.c_str());
		});
	}
	else
		imageFile.remove();

	// cloud binary
	if (cloudSuffix.length() > 0) {
		cloudFile.save(prefix + cloudSuffix, [=] (const std::string& path) {
			Cloud::save(handler.getContext(), path, *cloud);
		});
	}
	else
		cloudFile.remove();
}

//------------------------------------------------------------------------------

void golem::data::ItemImage::model(golem::ConfigMat34& config, golem::Mat34& frame, Vec3Seq* vertices, TriangleSeq* triangles) {
	handler.model(*this, config, frame, vertices, triangles);
}

size_t golem::data::ItemImage::getSize() const {
	return cloud->size();
}

void golem::data::ItemImage::getDimensions(size_t& width, size_t& height) const {
	width = (size_t)cloud->width;
	height = (size_t)cloud->height;
}

size_t golem::data::ItemImage::samplePoint(golem::Rand& rand) const {
	// uniform sampling
	return size_t(rand.next()) % cloud->size();
}

golem::data::Point3D::Point golem::data::ItemImage::getPoint(size_t index) const {
	return Point(Cloud::getPointNan<Point3D::Real>((*cloud)[index]));
}

golem::RGBA golem::data::ItemImage::getColour(size_t index) const {
	return Cloud::getColour((*cloud)[index]);
}

void golem::data::ItemImage::transform(const golem::Mat34& trn) {
	Cloud::transform(trn, *cloud, *cloud);
	cloudFile.setModified(true);
}

void golem::data::ItemImage::getSensorFrame(golem::Mat34& frame) const {
	frame = this->cloud != nullptr ? Cloud::getSensorFrame(*this->cloud) : Mat34::identity();
}

golem::data::Point3D::Real golem::data::ItemImage::getNormalCovariance() const {
	return REAL_ONE / golem::data::Point3D::Real(normalStdDev); // stdDev ~ 1/cov
}

golem::data::Point3D::Vec3 golem::data::ItemImage::getNormal(size_t index) const {
	return this->cloud != nullptr ? Cloud::getNormalNan<golem::data::Point3D::Real>((*cloud)[index]) : golem::data::Point3D::Vec3::axisZ();
}

//------------------------------------------------------------------------------

void golem::data::HandlerImage::ImageAppearance::xmlData(golem::XMLContext* context, bool create) const {
	golem::XMLData("show", const_cast<bool&>(show), context, create);
	golem::XMLData("show_with_cloud", const_cast<bool&>(showWithCloud), context, create);
	golem::XMLData("show_camera_frame", const_cast<bool&>(showCameraFrame), context, create);
}

void golem::data::HandlerImage::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	golem::data::Handler::Desc::load(context, xmlcontext);
	
	cloudAppearance.xmlData(xmlcontext->getContextFirst("cloud appearance", false), false);
	cloudImport.xmlData(xmlcontext->getContextFirst("cloud import", false), false);
	golem::XMLData("ext", cloudImportExt, xmlcontext->getContextFirst("cloud import", false), false);
	golem::XMLData("suffix", cloudSuffix, xmlcontext->getContextFirst("cloud", false), false);

	XMLData(cloudDesc, xmlcontext->getContextFirst("cloud", false), false);

	try {
		XMLData(regionCaptureDesc, regionCaptureDesc.max_size(), xmlcontext->getContextFirst("cloud region_capture"), "bounds", false);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}

	imageAppearance.xmlData(xmlcontext->getContextFirst("image appearance", false), false);
	golem::XMLData("suffix", imageSuffix, xmlcontext->getContextFirst("image", false), false);
	golem::XMLData("format", imageFormat, xmlcontext->getContextFirst("image", false), false);

	golem::XMLData("normal_std_dev", normalStdDev, xmlcontext->getContextFirst("properties"), false);

	if (rbPoseDesc == nullptr)
		rbPoseDesc.reset(new RBPose::Desc);
	XMLData(*rbPoseDesc, xmlcontext->getContextFirst("model", false), false);
	golem::XMLData("file", modelPath, xmlcontext->getContextFirst("model", false), false);
}

golem::data::Handler::Ptr golem::data::HandlerImage::Desc::create(golem::Context &context) const {
	Handler::Ptr handler(new HandlerImage(context));
	to<HandlerImage>(handler.get())->create(*this);
	return handler;
}

//------------------------------------------------------------------------------

golem::data::HandlerImage::HandlerImage(golem::Context &context) : Handler(context), imageID(0) {
}

void golem::data::HandlerImage::create(const Desc& desc) {
	golem::data::Handler::create(desc);

	cloudAppearance = desc.cloudAppearance;
	cloudImport = desc.cloudImport;
	cloudImportExt = desc.cloudImportExt;
	cloudSuffix = desc.cloudSuffix;
	
	cloudDesc = desc.cloudDesc;
	for (Bounds::Desc::Seq::const_iterator i = cloudDesc.regionDesc.begin(); i != cloudDesc.regionDesc.end(); ++i)
		region.push_back((*i)->create());
	for (Bounds::Desc::Seq::const_iterator i = desc.regionCaptureDesc.begin(); i != desc.regionCaptureDesc.end(); ++i)
		regionCapture.push_back((*i)->create());

	imageAppearance = desc.imageAppearance;
	imageSuffix = desc.imageSuffix;
	std::stringstream sstream(desc.imageFormat + " "); // HACK std::stringstream::read(&c, 1) reads one character too much without reporting eof
	IStream istream(sstream, "\t,; ");
	while (!istream.eos())
		imageFormat.push_back(istream.nextInt<int>());
	if (!imageFormat.empty())
		imageFormat.push_back(0);

	normalStdDev = desc.normalStdDev;

	// see OpenCV documentaion
	importTypes = {
		golem::Import::FILE_EXT_CLOUD_OBJ, golem::Import::FILE_EXT_CLOUD_PLY, golem::Import::FILE_EXT_CLOUD_PCD, cloudImportExt,
		".bmp", ".dib", ".jpeg", ".jpg", ".jpe", ".jp2", ".png", ".webp", ".pbm", ".pgm", ".ppm", ".sr", ".ras", ".tiff", ".tif",
	};

	transformInterfaces = {
		"ItemImage",
	};

	rbPose = desc.rbPoseDesc->create(context);
	modelPath = desc.modelPath;
}

//------------------------------------------------------------------------------

golem::data::Item::Ptr golem::data::HandlerImage::create() const {
	return Item::Ptr(new ItemImage(*const_cast<HandlerImage*>(this))); // safe: no modification here - only passing reference
}

//------------------------------------------------------------------------------

Item::Ptr golem::data::HandlerImage::import(const std::string& path) {
	// check extension
	if (std::find(importTypes.begin(), importTypes.end(), getExt(path)) == importTypes.end())
		throw Message(Message::LEVEL_ERROR, "HandlerImage::import(): unknown file type %s", getExt(path).c_str());

	Item::Ptr item(create());
	ItemImage* itemImage = to<ItemImage>(item.get());

	// try point cloud first
	const bool isOBJ = path.rfind(golem::Import::FILE_EXT_CLOUD_OBJ) != std::string::npos;
	const bool isPLY = path.rfind(golem::Import::FILE_EXT_CLOUD_PLY) != std::string::npos;
	const bool isPCD = path.rfind(golem::Import::FILE_EXT_CLOUD_PCD) != std::string::npos;
	const bool isGEN = path.rfind(cloudImportExt) != std::string::npos;
	if (isOBJ || isPLY || isPCD || isGEN) {
		Menu::Ptr menu(getUICallback() && getUICallback()->hasInputEnabled() ? new Menu(context, *getUICallback()) : nullptr);
		if (isPCD) {
			cloudImport.pointCloudPCLXYZRGBA(context, path, *itemImage->cloud);
		}
		else {
			if (menu != nullptr) {
				menu->readNumber("Size: ", cloudImport.size);
				if (isOBJ || isPLY)
					cloudImport.scale = Real(0.001);
				menu->readNumber("Scale: ", cloudImport.scale);
			}
			if (isOBJ || isPLY) {
				if (menu != nullptr) {
					menu->readNumber("Triangle order: ", cloudImport.clockwise);
				}
				if (isOBJ)
					cloudImport.pointCloudObj(context, path, *itemImage->cloud);
				else
					cloudImport.pointCloudPly(context, path, *itemImage->cloud);
			}
			else {
				cloudImport.pointCloudXYZNormal(context, path, *itemImage->cloud);
			}
		}
		Cloud::transform(cloudImport.transform, *itemImage->cloud, *itemImage->cloud);
		itemImage->cloudFile.setModified(true);
	}
	// image is recognised by contents
	else {
		itemImage->image = ::cvLoadImage(path.c_str());
		if (!itemImage->image)
			throw Message(Message::LEVEL_ERROR, "HandlerImage::import().cvLoadImage(): unable to load %s", path.c_str());
		itemImage->imageFile.setModified(true);
	}

	itemImage->cloudFile.setModified(true);

	return item;
}

const StringSeq& golem::data::HandlerImage::getImportFileTypes() const {
	return importTypes;
}

//------------------------------------------------------------------------------

Item::Ptr golem::data::HandlerImage::transform(const Item::List& input) {
	Item::Ptr item(create());
	ItemImage* itemImage = to<ItemImage>(item.get());

	// assert input type
	for (Item::List::const_iterator ptr = input.begin(); ptr != input.end(); ++ptr)
		if (!is<ItemImage>(*ptr))
			throw Message(Message::LEVEL_ERROR, "HandlerImage::transform(): Item %s is not supported", (*ptr)->second->getHandler().getType().c_str());

	// block item rendering (hasRenderBlock()=true)
	RenderBlock renderBlock(*this);

	// clear renderer
	ScopeGuard clearRenderer([&]() {
		UI::CriticalSectionWrapper cs(getUICallback());
		cloudRenderer.reset();
	});

	// align clouds
	Cloud::PointSeq tmp;
	Cloud::align(context, cloudDesc, input.begin(), input.end(), *itemImage->cloud,
	[&] (Item::Map::const_iterator ptr) -> const Cloud::PointSeq& {
		if (region.empty() || cloudDesc.regionFinalClip)
			return *to<ItemImage>(ptr)->cloud;

		tmp = *to<ItemImage>(ptr)->cloud;
		Cloud::regionClip(context, region, cloudDesc.threadChunkSize, tmp);
		return tmp;
	},
	[&] (Item::Map::const_iterator ptr, const Cloud::PointSeq& seq) {
		UI::CriticalSectionWrapper cs(getUICallback());
		this->image.release();
		modelRenderer.reset();
		cloudRenderer.reset();
		cloudAppearance.drawPoints(seq, cloudRenderer);
		if (!region.empty()) {
			cloudRenderer.setColour(cloudDesc.regionColourSolid);
			cloudRenderer.addSolid(region.begin(), region.end());
			cloudRenderer.setColour(cloudDesc.regionColourWire);
			cloudRenderer.addWire(region.begin(), region.end());
		}
	});

	// region clipping
	if (!region.empty() && cloudDesc.regionFinalClip)
		Cloud::regionClip(context, region, cloudDesc.threadChunkSize, *itemImage->cloud);
	// remove NaNs
	Cloud::nanRem(context, *itemImage->cloud, Cloud::isNanXYZNormal<Cloud::Point>);
	// copy last config and parameters
	itemImage->config = to<ItemImage>(input.back())->config;
	itemImage->parameters = to<ItemImage>(input.back())->parameters;

	itemImage->cloudFile.setModified(true);

	return item;
}

const StringSeq& golem::data::HandlerImage::getTransformInterfaces() const {
	return transformInterfaces;
}

bool golem::data::HandlerImage::isTransformSupported(const Item& item) const {
	return is<const ItemImage>(&item) != nullptr;
}

//------------------------------------------------------------------------------

Item::Ptr golem::data::HandlerImage::capture(golem::Sensor& sensor, Control control) {
	Camera* camera = dynamic_cast<Camera*>(&sensor);
	if (camera == nullptr)
		throw Message(Message::LEVEL_ERROR, "HandlerImage::capture(): Camera required");

	Item::Ptr item(create());
	ItemImage* itemImage = to<ItemImage>(item.get());
	CameraDepth* cameraDepth = is<CameraDepth>(camera);

	if (control == nullptr)
		throw Message(Message::LEVEL_ERROR, "HandlerImage::capture(): camera control required");

	ScopeGuard guardProperty([&] { if (cameraDepth) cameraDepth->setProperty(cameraDepth->getImageProperty()); }); // execute last
	if (cameraDepth) cameraDepth->setProperty(cameraDepth->getDepthProperty());

	ScopeGuard guardCMD([&] { camera->set(Camera::CMD_STOP); }); // execute first

	if (!camera->set(Camera::CMD_VIDEO))
		throw Message(Message::LEVEL_ERROR, "HandlerImage::capture(): unable to start capture");

	// synchronisation
	if (!control(nullptr))
		throw Cancel("HandlerImage::capture(): cancelled");
	// clear image queue
	itemImage->timeStamp = context.getTimer().elapsed();

	auto capture = [&] () {
		// retrieve a next frame
		for (bool stop = false; !stop;)
			camera->waitAndPeek([&] (const Image::List& images, bool result) {
			if (!result || images.empty())
				throw Message(Message::LEVEL_ERROR, "HandlerImage::capture(): unable to capture image");
			// search from the back, i.e. the latest frames
			for (Image::List::const_reverse_iterator i = images.rbegin(); i != images.rend(); ++i)
				if ((*i)->timeStamp > itemImage->timeStamp) {
					itemImage->set(**i);
					stop = true;
				}
		});
	};

	if (!cameraDepth) {
		capture();
	}
	else {
		Cloud::PointSeqQueue pointSeqQueue(cloudDesc.filterDesc.window);
		for (;;) {
			capture();

			Image::assertData(itemImage->cloud);
			if (cloudDesc.filterDesc.window <= 1)
				break;
			pointSeqQueue.push_back(*itemImage->cloud);
			if (pointSeqQueue.full()) {
				Cloud::filter(context, cloudDesc.filterDesc, cloudDesc.threadChunkSize, pointSeqQueue, *itemImage->cloud);
				break;
			}
		}

		// camera frame, use deformation map (can be overridden in CameraCalibration::enableDeformationMap())
		const Mat34 cameraFrame = camera->getFrame();
		Mat34 depthCameraFrame;
		depthCameraFrame.multiply(cameraFrame, cameraDepth->getColourToIRFrame());

		// transform points
		Cloud::transform(depthCameraFrame, *itemImage->cloud, *itemImage->cloud);
		// region clipping
		if (!regionCapture.empty())
			Cloud::regionClip(regionCapture, *itemImage->cloud, Cloud::isNanXYZ<Cloud::Point>);
	}

	camera->getConfig(itemImage->config);
	itemImage->parameters = camera->getCurrentCalibration()->getParameters();

	itemImage->imageFile.setModified(true);
	itemImage->cloudFile.setModified(true);

	return item;
}

//------------------------------------------------------------------------------

void golem::data::HandlerImage::model(const ItemImage& image, golem::ConfigMat34& config, golem::Mat34& frame, Vec3Seq* pvertices, TriangleSeq* ptriangles) {
	if (image.cloud == nullptr)
		throw Message(Message::LEVEL_ERROR, "HandlerImage::model(): empty cloud");

	Menu::Ptr menu(getUICallback() && getUICallback()->hasInputEnabled() ? new Menu(context, *getUICallback()) : nullptr);

	StringSeq types;
	types.push_back(golem::Import::FILE_EXT_CLOUD_OBJ);
	types.push_back(golem::Import::FILE_EXT_CLOUD_PLY);
	std::string modelPath = this->modelPath;
	if (menu != nullptr) {
		menu->readPath("Enter model file path: ", modelPath, types);
		cloudImport.scale = Real(0.001);
		menu->readNumber("Scale: ", cloudImport.scale);
	}
	Vec3Seq vertices;
	TriangleSeq triangles;
	if (modelPath.rfind(golem::Import::FILE_EXT_CLOUD_OBJ) != std::string::npos)
		((const golem::Import&)cloudImport).pointCloudObj(modelPath, vertices, triangles);
	else if (modelPath.rfind(golem::Import::FILE_EXT_CLOUD_PLY) != std::string::npos)
		((const golem::Import&)(cloudImport)).pointCloudPly(modelPath, vertices, triangles);
	else
		throw golem::Message(golem::Message::LEVEL_ERROR, "HandlerImage::model(): Unknown file type: %s", modelPath.c_str());

	if (this->modelPath != modelPath || modelCloud.empty()) {
		rbPose->createModel(vertices, triangles, modelCloud);
		this->modelPath = modelPath;
	}
	
	// block item rendering (hasRenderBlock()=true)
	RenderBlock renderBlock(*this);

	{
		UI::CriticalSectionWrapper cs(getUICallback());
		this->image.release();
		modelRenderer.reset();
		cloudRenderer.reset();
		cloudAppearance.drawPoints(*image.cloud, cloudRenderer);
	}

	// results
	config = image.config;
	frame = rbPose->align(
		*image.cloud,
		[&] (const golem::DebugRenderer& renderer) {
			UI::CriticalSectionWrapper cs(getUICallback());
			modelRenderer = renderer;
		},
		getUICallback() && getUICallback()->hasInputEnabled() ? getUICallback() : nullptr
	).toMat34();
	if (pvertices) {
		*pvertices = vertices;
		for (Vec3Seq::iterator i = pvertices->begin(); i != pvertices->end(); ++i)
			frame.multiply(*i, *i);
	}
	if (ptriangles) {
		*ptriangles = triangles;
	}
}

//------------------------------------------------------------------------------

void golem::data::HandlerImage::createRender(const ItemImage& item) {
	UI::CriticalSectionWrapper cs(getUICallback());

	if (imageAppearance.show && item.image && (imageAppearance.showWithCloud || item.cloud == nullptr || item.cloud->empty())) {
		image.reserve(item.image);
		::cvConvertImage(item.image, image.image, CV_CVTIMG_SWAP_RB);
	}
	else {
		image.release();
	}
	imageID = 0;

	cloudRenderer.reset();
	if (item.cloud != nullptr)
		cloudAppearance.drawPoints(*item.cloud, cloudRenderer);

	modelRenderer.reset();
}

void golem::data::HandlerImage::render() const {
	cloudRenderer.render();
	modelRenderer.render();
}

void golem::data::HandlerImage::customRender() const {
	if (imageAppearance.show && image.image) {
		image.draw(&imageID);
	}
}

//------------------------------------------------------------------------------

void golem::data::HandlerImage::mouseHandler(int button, int state, int x, int y) {
}

void golem::data::HandlerImage::motionHandler(int x, int y) {
}

void golem::data::HandlerImage::keyboardHandler(int key, int x, int y) {
	if (hasRenderBlock()) return;

	switch (key) {
	case '3':
		imageAppearance.show = !imageAppearance.show;
		context.write("Image turn %s\n", imageAppearance.show ? "ON" : "OFF");
		requestRender();
		break;
	case '4':
		cloudAppearance.mode = Cloud::Appearance::Mode((cloudAppearance.mode + 1)%(Cloud::Appearance::MODE_NORMAL + 1));
		context.write("Cloud appearance: %s\n", Cloud::Appearance::ModeName[size_t(cloudAppearance.mode)]);
		requestRender();
		break;
	};
}

//------------------------------------------------------------------------------
