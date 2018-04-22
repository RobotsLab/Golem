/** @file Feature3D.cpp
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
#include <Golem/Tools/Menu.h>
#include <Golem/Data/Image/Image.h>
#include <Golem/Data/Feature3D/Feature3D.h>
#include "GolemInteropDefs.h"
#include <boost/algorithm/string.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace golem;
using namespace golem::data;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new HandlerFeature3D::Desc();
}

//------------------------------------------------------------------------------

namespace {
	void glMat44Multiply(const GLfloat* glMat, const golem::Vec3& inp, golem::Vec3& out) {
		// out = inp * glMat;
		const golem::Real x = inp.x * glMat[ 0] + inp.y * glMat[ 4] + inp.z * glMat[ 8] + glMat[12];
		const golem::Real y = inp.x * glMat[ 1] + inp.y * glMat[ 5] + inp.z * glMat[ 9] + glMat[13];
		const golem::Real z = inp.x * glMat[ 2] + inp.y * glMat[ 6] + inp.z * glMat[10] + glMat[14];
		const golem::Real w = inp.x * glMat[ 3] + inp.y * glMat[ 7] + inp.z * glMat[11] + glMat[15];
		out.set(x/w, y/w, z/w);
	}
};

//------------------------------------------------------------------------------

golem::data::ItemFeature3D::ItemFeature3D(HandlerFeature3D& handler) : Item(handler), handler(handler), type(handler.getType()), cloudFile(handler.file), clusterFile(handler.file), featureIndex(0) {
	cloud.reset(new Cloud::PointFeatureSeq);
	normalRandAng = handler.normalRandAng;
	frameStdDev = handler.frameStdDev;
	debugReset();
}

Item::Ptr golem::data::ItemFeature3D::clone() const {
	Item::Ptr item(handler.create());
	ItemFeature3D* itemFeature3D = to<ItemFeature3D>(item.get());
	// copy to new format
	*itemFeature3D->cloud = *this->cloud;
	// cloud file container
	itemFeature3D->cloudFile = cloudFile;
	itemFeature3D->cloudFile.setModified(true);
	// type
	itemFeature3D->type = type;
	// other stuff
	itemFeature3D->config = config;
	// model
	itemFeature3D->normalRandAng = normalRandAng;
	itemFeature3D->frameStdDev = frameStdDev;
	// cluster file container
	itemFeature3D->clusterFile = clusterFile;
	itemFeature3D->clusterFile.setModified(true);
	// done!
	return item;
}

void golem::data::ItemFeature3D::createRender() {
	handler.createRender(*this);
}

void golem::data::ItemFeature3D::load(const std::string& prefix, const golem::XMLContext* xmlcontext) {
	// time stamp
	TimeStamp::xmlData(const_cast<golem::XMLContext*>(xmlcontext), false);

	// feature type
	std::string strType;
	golem::XMLData("type", strType, xmlcontext->getContextFirst("feature3d"), false);
	this->type = Cloud::Feature::getType(strType);

	// cloud
	std::string cloudSuffix;
	golem::XMLData("cloud", cloudSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (cloudSuffix.length() > 0) {
		cloudFile.load(prefix + cloudSuffix, [&] (const std::string& path) {
			if (cloud == nullptr)
				cloud.reset(new Cloud::PointFeatureSeq());
			Cloud::load(handler.getContext(), path, *cloud);
		});
	}

	// config
	config.xmlData(const_cast<golem::XMLContext*>(xmlcontext->getContextFirst("config", false)), false);

	// cluster
	try {
		std::string clusterSuffix;
		golem::XMLData("cluster", clusterSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
		try {
			selection.clear();
			golem::XMLData(selection, selection.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "selection", false);
		}
		catch (const golem::MsgXMLParser&) {}
		if (clusterSuffix.length() > 0) {
			clusterFile.load(prefix + clusterSuffix, [&](const std::string& path) {
				indices.clear();
				clusters.clear();
				FileReadStream frs(path.c_str());
				frs.read(indices, indices.end());
				frs.read(clusters, clusters.end());
			});
		}
	}
	catch (const golem::MsgXMLParser&) {}

	// region
	try {
		std::string regionSuffix;
		golem::XMLData("region_growing", regionSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
		if (regionSuffix.length() > 0) {
			regionFile.load(prefix + regionSuffix, [&](const std::string& path) {
				regIndices.clear();
				regClusters.clear();
				FileReadStream frs(path.c_str());
				frs.read(regIndices, regIndices.end());
				frs.read(regClusters, regClusters.end());
			});
		}
	}
	catch (const golem::MsgXMLParser&) {}


	// process data
	process();
}

void golem::data::ItemFeature3D::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	// time stamp
	TimeStamp::xmlData(xmlcontext, true);

	// feature type
	std::string strType(Cloud::Feature::TypeName[(size_t)type]);
	golem::XMLData("type", strType, xmlcontext->getContextFirst("feature3d", true), true);

	// cloud xml
	std::string cloudSuffix = cloud != nullptr && !cloud->empty() ? handler.cloudSuffix : "";
	golem::XMLData("cloud", cloudSuffix, xmlcontext, true);

	// cloud binary
	if (cloudSuffix.length() > 0) {
		cloudFile.save(prefix + cloudSuffix, [=] (const std::string& path) {
			Cloud::save(handler.getContext(), path, *cloud);
		});
	}
	else
		cloudFile.remove();

	// config
	config.xmlData(xmlcontext->getContextFirst("config", true), true);

	// cluster xml
	std::string clusterSuffix = !indices.empty() && !clusters.empty() ? handler.clusterSuffix : "";
	golem::XMLData("cluster", clusterSuffix, xmlcontext, true);
	golem::XMLData(const_cast<golem::data::Cluster3D::IndexMap&>(selection), selection.max_size(), xmlcontext, "selection", true);
	if (clusterSuffix.length() > 0) {
		clusterFile.save(prefix + clusterSuffix, [=](const std::string& path) {
			FileWriteStream fws(path.c_str());
			fws.write(indices.begin(), indices.end());
			fws.write(clusters.begin(), clusters.end());
		});
	}
	else
		clusterFile.remove();

	// region xml
	std::string regionSuffix = !regIndices.empty() && !regClusters.empty() ? handler.regionSuffix : "";
	golem::XMLData("region_growing", regionSuffix, xmlcontext, true);
	if (regionSuffix.length() > 0) {
		regionFile.save(prefix + regionSuffix, [=](const std::string& path) {
			FileWriteStream fws(path.c_str());
			fws.write(regIndices.begin(), regIndices.end());
			fws.write(regClusters.begin(), regClusters.end());
		});
	}
	else
		regionFile.remove();
}

//------------------------------------------------------------------------------

void golem::data::ItemFeature3D::process() {
}

//------------------------------------------------------------------------------

size_t golem::data::ItemFeature3D::getSize() const {
	return cloud->size();
}

void golem::data::ItemFeature3D::getDimensions(size_t& width, size_t& height) const {
	width = (size_t)cloud->width;
	height = (size_t)cloud->height;
}

size_t golem::data::ItemFeature3D::samplePoint(golem::Rand& rand) const {
	// uniform sampling
	return size_t(rand.next()) % cloud->size();
}

golem::data::Point3D::Point golem::data::ItemFeature3D::getPoint(size_t index) const {
	return Point(Cloud::getPointNan<Point3D::Real>((*cloud)[index]));
}

golem::RGBA golem::data::ItemFeature3D::getColour(size_t index) const {
	return Cloud::getColour((*cloud)[index]);
}

golem::data::Point3D::Real golem::data::ItemFeature3D::getNormalCovariance() const {
	return REAL_ONE / golem::data::Point3D::Real(frameStdDev.ang); // stdDev ~ 1/cov
}

golem::data::Point3D::Vec3 golem::data::ItemFeature3D::getNormal(size_t index) const {
	return Cloud::getNormalNan<golem::data::Point3D::Real>((*cloud)[index]);
}

void golem::data::ItemFeature3D::transform(const golem::Mat34& trn) {
	Cloud::transform(trn, *cloud, *cloud);
	cloudFile.setModified(true);
}

void golem::data::ItemFeature3D::getSensorFrame(golem::Mat34& frame) const {
	frame = Cloud::getSensorFrame(*cloud);
}

void golem::data::ItemFeature3D::getFeature(size_t index, Feature& feature, Mat33& orientation) const {
	const size_t size = Cloud::Feature::TypeSize[this->type];
	const Cloud::PointFeature& pointFeature = (*cloud)[index];

	// clean-up feature to avoid random data in the vector for better compression
	feature.assign(Feature3D::Feature::N(), golem::numeric_const<Point3D::Real>::ZERO);

	// feature
	feature.resize(size);
	for (size_t i = 0; i < size; ++i)
		feature[i] = static_cast<Point3D::Real>(pointFeature.descriptor_data[i]);

	// orientation
	orientation = Cloud::getFrame<Point3D::Real>(pointFeature).R;
}

void golem::data::ItemFeature3D::sampleSensorModel(golem::Rand& rand, size_t index, Mat33& orientation) const {
	const Cloud::PointFeature& pointFeature = (*cloud)[index];

	if (type == Cloud::Feature::TYPE_PRINCIPAL_CURVATURE) {
		// apply random rotation in 2 cases:
		// 1) x and y curvatures are very similar to each other, x ~ y
		// 2) ambiguity due to curvature direction
		// x >= y forr all x, y
		// scaling
		const golem::Real x = static_cast<golem::Real>(pointFeature.descriptor_data[0]);
		const golem::Real y = static_cast<golem::Real>(pointFeature.descriptor_data[1]);
		const golem::Real curvExp = golem::Math::abs(x) > golem::REAL_EPS ? normalRandAng*golem::Math::sqr((x - y) / x) : golem::REAL_ZERO;
		const golem::Real angleMag = golem::REAL_PI*golem::Math::exp(-curvExp);
		const golem::Real angle = rand.nextGaussian(rand.nextUniform<golem::Real>() < golem::REAL_HALF ? golem::REAL_PI : golem::REAL_ZERO, angleMag);
		orientation.rotZ((Point3D::Real)angle);
		// debug info
		curvRandMag += angleMag;
	}
	else {
		// no normal axis uncertainty
		orientation.setId();
	}
	
	++samplesCount;
}

golem::data::Point3D::RBDist golem::data::ItemFeature3D::getFrameCovariance() const {
	return golem::data::Point3D::RBDist(Math::sqr(frameStdDev.lin), REAL_ONE/frameStdDev.ang); // stdDev ~ 1/cov
}

void golem::data::ItemFeature3D::getClusters(Cluster3D::IndexSeq& indices, Cluster3D::IndexSeq& clusters) const {
	indices = this->indices;
	clusters = this->clusters;
}

void golem::data::ItemFeature3D::getClustersSelection(Cluster3D::IndexMap& selection) const {
	selection = this->selection;
}

void golem::data::ItemFeature3D::getRegions(Region3D::IndexSeq& indices, Region3D::IndexSeq& clusters) const {
	indices = this->regIndices;
	clusters = this->regClusters;
}

void golem::data::ItemFeature3D::debugReset() const {
	samplesCount = 0;
	curvRandMag = golem::REAL_ZERO;
}

void golem::data::ItemFeature3D::debugString(std::string& str) const {
	str = makeString("rand_angle=%f", samplesCount > 0 ? curvRandMag/samplesCount : golem::REAL_ZERO);
}

//------------------------------------------------------------------------------

void golem::data::HandlerFeature3D::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	golem::data::Handler::Desc::load(context, xmlcontext);

	try {
		golem::XMLData("debug_level", debugLevel, const_cast<golem::XMLContext*>(xmlcontext), false);
	}
	catch (const MsgXMLParser&) {}

	std::string strType;
	golem::XMLData("type", strType, xmlcontext->getContextFirst("feature3d"), false);
	this->type = Cloud::Feature::getType(strType);

	cloudAppearance.xmlData(xmlcontext->getContextFirst("cloud appearance", false), false);
	golem::XMLData("suffix", cloudSuffix, xmlcontext->getContextFirst("cloud", false), false);
	XMLData(cloudDesc, xmlcontext->getContextFirst("cloud", false), false);
	try {
		golem::XMLData(cloudTrnMap, cloudTrnMap.max_size(), xmlcontext->getContextFirst("cloud", false), "transform", false);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}

	golem::XMLData("rand_ang", normalRandAng, xmlcontext->getContextFirst("feature3d properties"), false);
	golem::XMLData(frameStdDev, xmlcontext->getContextFirst("feature3d properties frame_std_dev"), false);

	try {
		golem::XMLContext* pxmlcontext = xmlcontext->getContextFirst("clustering", false);
		golem::XMLData("suffix", clusterSuffix, pxmlcontext, false);
		auto range = pxmlcontext->getContextMap().equal_range("selection");
		for (golem::XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second; ++i) {
			std::string name;
			golem::XMLData("name", name, const_cast<XMLContext*>(&i->second), false);
			StringSet& clusterNames = const_cast<StringSet&>(this->clusterNames);
			clusterNames.insert(clusterNames.end(), name);
		}
	}
	catch (const MsgXMLParser&) {
	}
}

golem::data::Handler::Ptr golem::data::HandlerFeature3D::Desc::create(golem::Context &context) const {
	Handler::Ptr handler(new HandlerFeature3D(context));
	to<HandlerFeature3D>(handler.get())->create(*this);
	return handler;
}

//------------------------------------------------------------------------------

golem::data::HandlerFeature3D::HandlerFeature3D(golem::Context &context) : Handler(context), modeRequest(false), featureIndexRequest(0) {
}

void golem::data::HandlerFeature3D::create(const Desc& desc) {
	golem::data::Handler::create(desc);

	debugLevel = desc.debugLevel;

	cloudAppearance = desc.cloudAppearance;
	cloudSuffix = desc.cloudSuffix;

	cloudDesc = desc.cloudDesc;
	for (Bounds::Desc::Seq::const_iterator i = cloudDesc.regionDesc.begin(); i != cloudDesc.regionDesc.end(); ++i)
		region.push_back((*i)->create());

	cloudTrnMap = desc.cloudTrnMap;

	type = desc.type;

	normalRandAng = desc.normalRandAng;
	frameStdDev = desc.frameStdDev;

	clusterSuffix = desc.clusterSuffix;
	clusterNames = desc.clusterNames;

	clusterSetSelection = 0;
	clusterIdxSelection = 0;
	clusterSelection = clusterSelectionAddRemove = false;
	clusterX = clusterY = 0;

	regionSuffix = desc.regionSuffix;
	showRegions = false;

	transformInterfaces = {
		"ItemImage",
	};
}

//------------------------------------------------------------------------------

golem::data::Item::Ptr golem::data::HandlerFeature3D::create() const {
	return Item::Ptr(new ItemFeature3D(*const_cast<HandlerFeature3D*>(this))); // safe: no modification here - only passing reference
}

//------------------------------------------------------------------------------

Item::Ptr golem::data::HandlerFeature3D::transform(const Item::List& input) {
	Item::Ptr item(create());
	ItemFeature3D* itemFeature3D = to<ItemFeature3D>(item.get());
	
	// assert input type
	for (Item::List::const_iterator ptr = input.begin(); ptr != input.end(); ++ptr)
		if (!is<ItemImage>(*ptr))
			throw Message(Message::LEVEL_ERROR, "HandlerFeature3D::transform(): Item %s is not supported", (*ptr)->second->getHandler().getType().c_str());

	// block item rendering (hasRenderBlock()=true)
	RenderBlock renderBlock(*this);

	// clear renderer
	ScopeGuard clearRenderer([&]() {
		UI::CriticalSectionWrapper cs(getUICallback());
		cloudRenderer.reset();
		clusterRenderer.reset();
	});
	clearRenderer.run();

	Menu::Ptr menu(getUICallback() && getUICallback()->hasInputEnabled() ? new Menu(context, *getUICallback()) : nullptr);

	// select type
	if (menu != nullptr) {
		const StringSeq optionStr(Cloud::Feature::TypeName, Cloud::Feature::TypeName + Cloud::Feature::TYPE_SIZE);
		this->type = itemFeature3D->type = static_cast<Cloud::Feature::Type>(menu->option(static_cast<size_t>(itemFeature3D->type), "Feature extraction algorithm: ", optionStr));
	}

	Cloud::PointSeq out;

	// align clouds
	if (itemFeature3D->type == Cloud::Feature::TYPE_PRINCIPAL_CURVATURE || itemFeature3D->type == Cloud::Feature::TYPE_FPFH) {
		// options
		if (menu != nullptr) {
			// downsampling
			cloudDesc.downsampleSegmentation.enabled = menu->option(cloudDesc.downsampleSegmentation.enabled ? 0 : 1, "Downsampling: ", { "YES", "NO" }) == 0;
			if (cloudDesc.downsampleSegmentation.enabled)
				menu->readNumber("Enter downsample leaf size: ", cloudDesc.downsampleSegmentation.gridLeafSize);
			// normals
			bool * option [] = { &cloudDesc.normal.enabledPCA, &cloudDesc.normal.enabledMLS, &cloudDesc.normal.enabledII };
			StringSeq optionStr = { "PCA", "MLS", "II" };
			size_t index = 0;
			for (; index + 1 < sizeof(option) / sizeof(bool*) && !*option[index]; ++index);
			index = menu->option(index, "Normal estimation algorithm: ", optionStr);
			for (size_t i = 0; i < sizeof(option) / sizeof(bool*); ++i) *option[i] = (i == index);
			if (cloudDesc.normal.enabledPCA || cloudDesc.normal.enabledMLS)
				menu->readNumber("Enter normal search radius: ", cloudDesc.normal.radiusSearch);
			// curvature
			menu->readNumber("Enter curvature search radius: ", cloudDesc.curvature.radiusSearch);
		}
		Mat34Map::const_iterator cloudTrnMapPtr = cloudTrnMap.begin();
		Cloud::PointSeq tmp;
		Cloud::align(context, cloudDesc, input.begin(), input.end(), out,
			[&](Item::Map::const_iterator ptr) -> const Cloud::PointSeq&{
				if (menu == nullptr && !cloudTrnMap.empty() && (region.empty() || cloudDesc.regionFinalClip))
					return *to<ItemImage>(ptr)->cloud;

				tmp = *to<ItemImage>(ptr)->cloud;

				if (!region.empty() && !cloudDesc.regionFinalClip)
					Cloud::regionClip(context, region, cloudDesc.threadChunkSize, tmp);

				if (menu != nullptr && !cloudTrnMap.empty()) {
					menu->select(cloudTrnMapPtr, cloudTrnMap.begin(), cloudTrnMap.end(), "Select transformation:\n", [] (Mat34Map::const_iterator ptr) -> std::string { return ptr->first; });
					Cloud::transform(cloudTrnMapPtr->second, tmp, tmp);
				}

				return tmp;
		},
			[&] (Item::Map::const_iterator ptr, const Cloud::PointSeq& seq) {
			UI::CriticalSectionWrapper cs(getUICallback());
			cloudRenderer.reset();
			cloudAppearance.drawPoints(seq, cloudRenderer);
			if (!region.empty()) {
				cloudRenderer.setColour(cloudDesc.regionColourSolid);
				cloudRenderer.addSolid(region.begin(), region.end());
				cloudRenderer.setColour(cloudDesc.regionColourWire);
				cloudRenderer.addWire(region.begin(), region.end());
			}
		});
	}
	else if (itemFeature3D->type == Cloud::Feature::TYPE_NARF36) {
		// options
		if (menu != nullptr) {
			cloudDesc.narf36.usePlanarProjection = menu->option(cloudDesc.narf36.usePlanarProjection ? 0 : 1, "NARF36 image projection: ", { "Planar", "Spherical" }) == 0;
			cloudDesc.narf36.upsampling = menu->option(cloudDesc.narf36.upsampling ? 1 : 0, "NARF36 upsampling: ", { "NO", "YES" }) == 1;
		}
		// only first cloud
		out = *to<ItemImage>(*input.begin())->cloud;
		// draw
		{
			UI::CriticalSectionWrapper cs(getUICallback());
			cloudRenderer.reset();
			cloudAppearance.drawPoints(out, cloudRenderer);
			if (!region.empty()) {
				cloudRenderer.setColour(cloudDesc.regionColourSolid);
				cloudRenderer.addSolid(region.begin(), region.end());
				cloudRenderer.setColour(cloudDesc.regionColourWire);
				cloudRenderer.addWire(region.begin(), region.end());
			}
		}
		// update image size
		cloudDesc.narf36.imageWidth = (U32)out.width;
		cloudDesc.narf36.imageHeight = (U32)out.height;
		// clip region
		if (!region.empty() && !cloudDesc.regionFinalClip)
			Cloud::regionClip(context, region, cloudDesc.threadChunkSize, out);
		// remove NaNs
		Cloud::nanRem(context, out, Cloud::isNanXYZNormal<Cloud::Point>);
	}
	else {
	}

	if (itemFeature3D->type == Cloud::Feature::TYPE_PRINCIPAL_CURVATURE) {
		Cloud::curvature(context, cloudDesc.curvature, out, *itemFeature3D->cloud);
	}
	else if (itemFeature3D->type == Cloud::Feature::TYPE_NARF36) {
		Cloud::narf36(context, cloudDesc.narf36, out, *itemFeature3D->cloud);
	}
	else if (itemFeature3D->type == Cloud::Feature::TYPE_FPFH) {
		Cloud::PointFeatureSeq tmp;
		// compute local frames first
		Cloud::curvature(context, cloudDesc.curvature, out, tmp);
		// compute FPFH descriptors
		Cloud::fpfh(context, cloudDesc.fpfh, tmp, *itemFeature3D->cloud);
	}
	else {
		throw Message(Message::LEVEL_ERROR, "HandlerFeature3D::transform(): Algorithm %s is not supported", Cloud::Feature::TypeName[itemFeature3D->type]);
	}

	// region clipping
	if (!region.empty() && cloudDesc.regionFinalClip) {
		Cloud::regionClip(context, region, cloudDesc.threadChunkSize, *itemFeature3D->cloud);
		// remove NaNs (required after region clipping!)
		Cloud::nanRem(context, *itemFeature3D->cloud, Cloud::isNanXYZNormalFeature<Cloud::PointFeature>);
	}

	if (cloudDesc.clustering.enabled) {
		// copy back
		Cloud::copy(*itemFeature3D->cloud, out, Cloud::copyPointXYZNormalRGBA<Cloud::PointFeature, Cloud::Point>);
		// clustering
		IntSeq indices, clusters;
		Cloud::clustering(context, cloudDesc.clustering, out, indices, clusters);
		interop::convert(indices, itemFeature3D->indices);
		interop::convert(clusters, itemFeature3D->clusters);

		// cluster selection
		//Cluster3D::IndexSet selection;
		//for (size_t index = 0; index < clusters.size(); ++index)
		//	selection.insert(static_cast<Cluster3D::Index>(index));
		// add all
		//for (StringSeq::const_iterator i = clusterNames.begin(); i != clusterNames.end(); ++i)
		//	itemFeature3D->selection.insert(std::make_pair(*i, selection));
		// processing
		//itemFeature3D->selection.insert(std::make_pair(Cluster3D::CLUSTER_PROCESSING, selection));
		// collisions
		//itemFeature3D->selection.insert(std::make_pair(Cluster3D::CLUSTER_COLLISIONS, selection));

		// make sure it will be saved
		itemFeature3D->clusterFile.setModified(true);
	}

	if (cloudDesc.regionGrowing.enabled) {
		// Smoothness threshold
		menu->readNumber("Enter smoothness threshold: ", cloudDesc.regionGrowing.smoothThreshold);
		// Curvature threshold
		menu->readNumber("Enter curvature threshold: ", cloudDesc.regionGrowing.curvatureThreshold);		
		// copy back
		Cloud::copy(*itemFeature3D->cloud, out, Cloud::copyPointXYZNormalRGBA<Cloud::PointFeature, Cloud::Point>);
		// clustering
		IntSeq indices, clusters;
		Cloud::regionGrowing(context, cloudDesc.regionGrowing, out, indices, clusters);
		interop::convert(indices, itemFeature3D->regIndices);
		interop::convert(clusters, itemFeature3D->regClusters);

		// Todo: create selection with informative names
		// make sure it will be saved
		itemFeature3D->regionFile.setModified(true);
	}

	// copy last config
	itemFeature3D->config = to<ItemImage>(input.back())->config;
	// process
	itemFeature3D->process();
	// make sure it will be saved
	itemFeature3D->cloudFile.setModified(true);

	return item;
}

const StringSeq& golem::data::HandlerFeature3D::getTransformInterfaces() const {
	return transformInterfaces;
}

bool golem::data::HandlerFeature3D::isTransformSupported(const Item& item) const {
	return is<const ItemImage>(&item) != nullptr;
}

//------------------------------------------------------------------------------

std::string golem::data::HandlerFeature3D::toString(golem::U32 mode, golem::U32 type, golem::U32 index) {
	std::string str;
	str += "Mode (";
	str += Cloud::Appearance::ModeName[mode];
	str += "), feature type (";
	str += Cloud::Feature::TypeName[type];
	str += ")";
	if (mode == Cloud::Appearance::MODE_FEATURE) {
		str += ", feature index #";
		str += std::to_string(index + 1);
		str += "/";
		str += std::to_string(Cloud::Feature::TypeSize[type]);
	}
	return str;
}

void golem::data::HandlerFeature3D::createRender(const ItemFeature3D& item) {
	if (!getUICallback() || item.cloud == nullptr || item.cloud->size() <= 0)
		return;

	// feature index
	if (modeRequest || featureIndexRequest != 0) {
		item.featureIndex = (golem::U32)golem::Math::clamp(golem::I32(item.featureIndex) + featureIndexRequest, 0, golem::I32(Cloud::Feature::TypeSize[item.type]) - 1);
		context.write("%s\n", toString(cloudAppearance.mode, item.type, item.featureIndex).c_str());
		modeRequest = false;
		featureIndexRequest  = 0;
	}

	golem::OpenGL openGL;
	getUICallback()->getOpenGL(openGL); // locks Universe::cs, keep outside of csRenderer

	UI::CriticalSectionWrapper cs(getUICallback());
	cloudRenderer.reset();
	clusterRenderer.reset();

	// regions
	if (showRegions && !item.regIndices.empty() && !item.regClusters.empty()) {
		cloudAppearance.drawRegions(*item.cloud, item.regIndices, item.regClusters, cloudRenderer);
		return;
	}

	// feature drawing
	const golem::Real curvFrameSize = Real(20.0);
	cloudAppearance.drawFeatures(*item.cloud, item.featureIndex, [=] (const Cloud::PointFeature& p) -> golem::Vec3 {
		if (item.type == Cloud::Feature::TYPE_PRINCIPAL_CURVATURE) {
			return golem::Vec3(
				curvFrameSize*powerScale(static_cast<golem::Real>(p.descriptor_data[0]), static_cast<golem::Real>(cloudAppearance.featureCurvPow)),
				curvFrameSize*powerScale(static_cast<golem::Real>(p.descriptor_data[1]), static_cast<golem::Real>(cloudAppearance.featureCurvPow)),
				REAL_ONE
			);
		}
		else {
			return golem::Vec3(REAL_ONE);
		}
	}, cloudRenderer, &openGL);

	// clusters
	if (!item.indices.empty() && !item.clusters.empty()) {
		const bool update = clusterSelection || clusterSelectionAddRemove;
		this->clusterSelection = false;

		// x, y --> cluster
		if (clusterSelectionAddRemove && clusterX > 0 && clusterY > 0) {
			unsigned d = -1;

			Mat34 frame;
			frame.setColumn44(openGL.glMatExtrinsic);

			for (data::Cluster3D::IndexSeq::const_iterator j = item.clusters.begin(), i = j++; i != item.clusters.end(); ++i, ++j) {
				const size_t begin = (size_t)*i;
				const size_t end = j == item.clusters.end() ? item.indices.size() : (size_t)*j;

				for (size_t j = begin; j < end; ++j) {
					const size_t k = (size_t)item.indices[j];
					Vec3 p((*item.cloud)[k].x, (*item.cloud)[k].y, (*item.cloud)[k].z);
					frame.multiply(p, p);
					glMat44Multiply(openGL.glMatIntrinsic, p, p);
					p.x = REAL_HALF*(+p.x + REAL_ONE);
					p.y = REAL_HALF*(-p.y + REAL_ONE);
					// to screen coordinates: [-1, 1] X [-1, 1] |=> [0, openGL.x] X [0, openGL.y]
					//clusterRenderer.addPoint(p, RGBA::RED); // test
					const int dx = Math::abs(static_cast<int>(p.x*openGL.x) - clusterX);
					const int dy = Math::abs(static_cast<int>(p.y*openGL.y) - clusterY);
					const int dd = dx*dx + dy*dy;
					if (d > dd) {
						d = dd;
						clusterIdxSelection = U32(i - item.clusters.begin());
					}
				}
			}

			//clusterRenderer.addAxes3D(Mat34::identity(), Vec3(REAL_ONE));
			clusterX = clusterY = 0;
		}

		// cluster index
		data::Cluster3D::IndexSeq::const_iterator cluster = item.clusters.begin();
		selectIndex(clusterIdxSelection, cluster, item.clusters);

		// cluster type
		StringSet names = clusterNames;
		for (Cluster3D::IndexMap::const_iterator i = item.selection.begin(); i != item.selection.end(); ++i)
			names.insert(names.end(), i->first);
		if (names.empty()) {
			if (update)
				context.write("Cluster types not available\n");
			return;
		}
		StringSet::const_iterator name = names.begin();
		selectIndex(clusterSetSelection, name, names);

		if (update)
			context.write("Cluster type %s, index %u\n", name->c_str(), clusterIdxSelection + 1);

		// selection
		if (clusterSelectionAddRemove) {
			Cluster3D::IndexMap& map = const_cast<Cluster3D::IndexMap&>(item.selection);
			Cluster3D::IndexMap::iterator selection = map.find(*name);
			if (selection != item.selection.end()) {
				Cluster3D::IndexSet::iterator set = selection->second.find(Cluster3D::Index(clusterIdxSelection));
				if (set != selection->second.end()) {
					selection->second.erase(Cluster3D::Index(clusterIdxSelection));
					if (selection->second.empty())
						map.erase(*name);
				}
				else
					selection->second.insert(Cluster3D::Index(clusterIdxSelection));
			}
			else
				map[*name].insert(Cluster3D::Index(clusterIdxSelection));
			clusterSelectionAddRemove = false;
		}

		// render
		Cluster3D::IndexMap::const_iterator selection = item.selection.find(*name);
		if (selection != item.selection.end()) {
			cloudAppearance.clusterColour._rgba.a = 255;
			cloudAppearance.drawCluster(&item.cloud->front().x, sizeof(Cloud::PointFeature), item.cloud->size(), item.indices, item.clusters, selection->second, clusterRenderer);
		}

		// selection
		if (update) {
			Cluster3D::IndexSet selection;
			selection.insert((Cluster3D::Index)clusterIdxSelection);
			cloudAppearance.clusterColour._rgba.a = 127;
			cloudAppearance.drawCluster(&item.cloud->front().x, sizeof(Cloud::PointFeature), item.cloud->size(), item.indices, item.clusters, selection, clusterRenderer);
		}
	};
}

void golem::data::HandlerFeature3D::render() const {
	cloudRenderer.render();
	clusterRenderer.render();
}

void golem::data::HandlerFeature3D::customRender() const {
}

//------------------------------------------------------------------------------

void golem::data::HandlerFeature3D::mouseHandler(int button, int state, int x, int y) {
	if (hasRenderBlock()) return;

	// clusters
	if (button == 2 && state == 0) {
		clusterSelectionAddRemove = true;
		clusterX = x;
		clusterY = y;
		requestRender();
	}
}

void golem::data::HandlerFeature3D::motionHandler(int x, int y) {
}

void golem::data::HandlerFeature3D::keyboardHandler(int key, int x, int y) {
	if (hasRenderBlock()) return;

	switch (key) {
	case '4':
		cloudAppearance.mode = Cloud::Appearance::Mode((cloudAppearance.mode + 1)%(Cloud::Appearance::MODE_SIZE));
		modeRequest = true;
		requestRender();
		break;
	case '5':
		if (cloudAppearance.mode == Cloud::Appearance::MODE_FEATURE) {
			featureIndexRequest = -1;
			requestRender();
		}
		break;
	case '6':
		if (cloudAppearance.mode == Cloud::Appearance::MODE_FEATURE) {
			featureIndexRequest = +1;
			requestRender();
		}
		break;
	case '7':
		clusterIdxSelection = clusterIdxSelection > 0 ? clusterIdxSelection - 1 : 0;
		clusterSelection = true;
		requestRender();
		break;
	case '8':
		clusterIdxSelection = clusterIdxSelection + 1;
		clusterSelection = true;
		requestRender();
		break;
	case '9':
		clusterSetSelection = clusterSetSelection > 0 ? clusterSetSelection - 1 : 0;
		clusterSelection = true;
		requestRender();
		break;
	case '0':
		clusterSetSelection = clusterSetSelection + 1;
		clusterSelection = true;
		requestRender();
		break;
	case '-':
		showRegions = !showRegions;
		requestRender();
		break;
	case 32:
		clusterSelectionAddRemove = true;
		requestRender();
		break;
	};
}

//------------------------------------------------------------------------------
