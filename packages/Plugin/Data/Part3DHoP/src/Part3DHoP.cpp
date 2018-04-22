/** @file Part3DHoP.cpp
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
#include <Golem/Data/Part3DHoP/Part3DHoP.h>
#include <Golem/Math/Data.h>
#include <boost/algorithm/string.hpp>
#include "GolemInteropStreamDefs.h"
#include "GolemInteropGolemDefs.h"
#include "GolemInteropPCLDefs.h"
#include <pcl/io/pcd_io.h>

using namespace golem;
using namespace golem::data;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new HandlerPart3DHoP::Desc();
}

//------------------------------------------------------------------------------

namespace golem {
namespace interop {
	void convert(const part3d::Part3D& src, data::Part3D::Part& dst) {
		convert(src.model, dst.model);
		convert(src.frame, dst.frame);
		convert(src.weight, dst.weight);
		//dst.cdf = -golem::numeric_const<data::Part3D::Real>::ONE;
	}
}
}

//------------------------------------------------------------------------------

void golem::data::XMLData(golem::data::Part3DProperty::Map::value_type& val, golem::XMLContext* context, bool create) {
	golem::XMLData("id", const_cast<golem::data::Part3D::Index&>(val.first), context, create);
	golem::XMLData("weight", val.second.weight, context, create);
	//golem::XMLData(val.second.frameStdDev, context->getContextFirst("frame_stddev", create), create);
	golem::XMLData("frame_stddev_lin", val.second.frameStdDev.lin, context, create);
	golem::XMLData("frame_stddev_ang", val.second.frameStdDev.ang, context, create);
}

void golem::data::XMLData(golem::data::Part3DProperty::Set& val, golem::XMLContext* context, bool create) {
	// level: mandatory
	golem::XMLData(val.level, val.level.max_size(), context, "level", create);
	// model: optional
	try {
		golem::XMLData(val.model, val.model.max_size(), context, "model", create);
	}
	catch (const MsgXMLParserNameNotFound&) {
		if (create) throw;
	}
	// realisation: optional
	try {
		golem::XMLData(val.realisation, val.realisation.max_size(), context, "realisation", create);
	}
	catch (const MsgXMLParserNameNotFound&) {
		if (create) throw;
	}
}

//------------------------------------------------------------------------------

golem::data::ItemPart3DHoP::ItemPart3DHoP(HandlerPart3DHoP& handler) : Item(handler), handler(handler), cloudFile(handler.file), part3DFile(handler.file), cloudIndex(0), levelIndex(0), partIndex(0) {
	partProperties = handler.partProperties;
	normalStdDev = handler.normalStdDev;
}

Item::Ptr golem::data::ItemPart3DHoP::clone() const {
	throw Message(Message::LEVEL_ERROR, "ItemPart3DHoP::clone(): not implemented");
}

void golem::data::ItemPart3DHoP::createRender() {
	handler.createRender(*this);
}

void golem::data::ItemPart3DHoP::load(const std::string& prefix, const golem::XMLContext* xmlcontext) {
	// cloud index
	golem::XMLData("cloud_index", cloudIndex, const_cast<golem::XMLContext*>(xmlcontext), false);
	// level index
	golem::XMLData("level_index", levelIndex, const_cast<golem::XMLContext*>(xmlcontext), false);
	// part index
	golem::XMLData("part_index", partIndex, const_cast<golem::XMLContext*>(xmlcontext), false);

	// cloud
	std::string cloudSuffix;
	golem::XMLData("cloud", cloudSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (cloudSuffix.length() > 0) {
		cloudFile.load(prefix + cloudSuffix, [&](const std::string& path) {
			// block annoying pcl console messages
			const pcl::console::VERBOSITY_LEVEL pclLevel = pcl::console::getVerbosityLevel();
			golem::ScopeGuard guard([=]() { pcl::console::setVerbosityLevel(pclLevel); });
			pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_ALWAYS);
			if (pcl::PCDReader().read(path, *cloud) != 0)
				throw golem::Message(golem::Message::LEVEL_ERROR, "ItemPart3DHoP::load().pcl::PCDReader(): unable to read from %s", path.c_str());
		});
	}

	// parts
	std::string part3DSuffix;
	golem::XMLData("part3d", part3DSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (part3DSuffix.length() > 0) {
		part3DFile.load(prefix + part3DSuffix, [&](const std::string& path) {
			models.clear();
			realisations.clear();
			parts.clear();
			indices.clear();
			FileReadStream frs(path.c_str());
			frs.read(models, models.end());
			frs.read(realisations, realisations.end());
			frs.read(parts, parts.end());
			frs.read(indices, indices.end());
		});
	}

	//partProperties.setToDefault();
	//golem::data::XMLData(partProperties, xmlcontext->getContextFirst("properties"), false);
	//golem::XMLData("normal_std_dev", normalStdDev, xmlcontext->getContextFirst("query"), false);

	partProperties = handler.partProperties;
	normalStdDev = handler.normalStdDev;

	// processing
	process();
}

void golem::data::ItemPart3DHoP::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	// cloud index
	golem::XMLData("cloud_index", const_cast<golem::U32&>(cloudIndex), xmlcontext, true);
	// level index
	golem::XMLData("level_index", const_cast<golem::U32&>(levelIndex), xmlcontext, true);
	// part index
	golem::XMLData("part_index", const_cast<golem::U32&>(partIndex), xmlcontext, true);

	// cloud
	std::string cloudSuffix = cloud != nullptr && !cloud->empty() ? handler.cloudSuffix : "";
	golem::XMLData("cloud", cloudSuffix, xmlcontext, true);
	if (cloudSuffix.length() > 0) {
		cloudFile.save(prefix + cloudSuffix, [=](const std::string& path) {
			try {
				pcl::PCDWriter().writeBinaryCompressed(path, *cloud);
			}
			catch (std::exception& ex) {
				throw golem::Message(golem::Message::LEVEL_ERROR, "ItemPart3DHoP::save().pcl::pcdWrite(): unable to write to %s (%s)", path.c_str(), ex.what());
			}
		});
	}
	else
		cloudFile.remove();

	// part3d
	std::string part3DSuffix = !parts.empty() || !indices.empty() || !models.empty() ? handler.part3DSuffix : "";
	golem::XMLData("part3d", part3DSuffix, xmlcontext, true);
	if (part3DSuffix.length() > 0) {
		part3DFile.save(prefix + part3DSuffix, [=](const std::string& path) {
			FileWriteStream fws(path.c_str());
			fws.write(models.begin(), models.end());
			fws.write(realisations.begin(), realisations.end());
			fws.write(parts.begin(), parts.end());
			fws.write(indices.begin(), indices.end());
		});
	}
	else
		part3DFile.remove();

	golem::data::XMLData(const_cast<golem::data::Part3DProperty::Set&>(partProperties), xmlcontext->getContextFirst("properties", true), true);

	golem::XMLData("normal_std_dev", const_cast<golem::Real&>(normalStdDev), xmlcontext->getContextFirst("query", true), true);
}

//------------------------------------------------------------------------------

golem::data::Part3DProperty::Map::const_iterator golem::data::ItemPart3DHoP::getPart3DProperty(Part3D::Index realisation) const {
	// try realisation
	golem::data::Part3DProperty::Map::const_iterator part3DProperty = partProperties.realisation.find(realisation);
	if (part3DProperty != partProperties.realisation.end())
		return part3DProperty;
	
	// try model
	const data::Part3D::Part::Map::const_iterator part = parts.find(realisation);
	if (part == parts.end())
		throw Message(Message::LEVEL_ERROR, "ItemPart3DHoP::getPart3DProperty(): unable to find part for realisation #%u", realisation);
	part3DProperty = partProperties.model.find(part->second.model);
	if (part3DProperty != partProperties.model.end())
		return part3DProperty;

	// try level
	const data::Part3D::IndexMap::const_iterator level = partToLevelMap.find(realisation);
	if (level == partToLevelMap.end())
		throw Message(Message::LEVEL_ERROR, "ItemPart3DHoP::getPart3DProperty(): unable to find level for realisation #%u", realisation);
	part3DProperty = partProperties.level.find(level->second);
	if (part3DProperty == partProperties.model.end())
		throw Message(Message::LEVEL_ERROR, "ItemPart3DHoP::getPart3DProperty(): unable to find properties for realisation #%u", realisation);

	return part3DProperty;
}

size_t golem::data::ItemPart3DHoP::getSize() const {
	return cloud->size();
}

void golem::data::ItemPart3DHoP::getDimensions(size_t& width, size_t& height) const {
	width = (size_t)cloud->width;
	height = (size_t)cloud->height;
}

size_t golem::data::ItemPart3DHoP::samplePoint(golem::Rand& rand) const {
	// uniform sampling
	return size_t(rand.next()) % cloud->size();
}

golem::data::Point3D::Point golem::data::ItemPart3DHoP::getPoint(size_t index) const {
	return Point(Cloud::getPointNan<Point3D::Real>((*cloud)[index]));
}

golem::RGBA golem::data::ItemPart3DHoP::getColour(size_t index) const {
	return Cloud::getColour((*cloud)[index]);
}

void golem::data::ItemPart3DHoP::transform(const golem::Mat34& trn) {
	Cloud::transform(trn, *cloud, *cloud);
	cloudFile.setModified(true);
}

void golem::data::ItemPart3DHoP::getSensorFrame(golem::Mat34& frame) const {
	frame = this->cloud != nullptr ? Cloud::getSensorFrame(*this->cloud) : Mat34::identity();
}

golem::data::Point3D::Real golem::data::ItemPart3DHoP::getNormalCovariance() const {
	return REAL_ONE / golem::data::Point3D::Real(normalStdDev); // stdDev ~ 1/cov
}

golem::data::Point3D::Vec3 golem::data::ItemPart3DHoP::getNormal(size_t index) const {
	return this->cloud != nullptr ? Cloud::getNormalNan<golem::data::Point3D::Real>((*cloud)[index]) : golem::data::Point3D::Vec3::axisZ();
}

void golem::data::ItemPart3DHoP::getPart(Part3D::Part::Map& partMap) const {
	partMap = parts;
}

void golem::data::ItemPart3DHoP::getPart(size_t index, Part3D::Part::Map& partMap) const {
	partMap.clear();

	const data::Part3D::IndexSetMap::const_iterator i = indexToPartMap.find((data::Part3D::Index)index);
	if (i == indexToPartMap.end())
		return;

	for (data::Part3D::IndexSet::const_iterator j = i->second.begin(); j != i->second.end(); ++j) {
		data::Part3D::Part::Map::const_iterator part = parts.find(*j);
		if (part == parts.end())
			throw Message(Message::LEVEL_ERROR, "ItemPart3DHoP::getPart(): unknown part #%u", *j);
		const Part3D::Part::Map::iterator ptr = partMap.insert(partMap.end(), *part);

		// update weight
		//ptr->second.weight *= getPart3DProperty(*j)->second.weight;
	}
}

golem::data::Point3D::RBDist golem::data::ItemPart3DHoP::getFrameCovariance(Part3D::Index realisation) const {
	golem::data::Part3DProperty::Map::const_iterator part3DProperty = getPart3DProperty(realisation);

	return golem::data::Point3D::RBDist(Math::sqr(part3DProperty->second.frameStdDev.lin), REAL_ONE / part3DProperty->second.frameStdDev.ang); // stdDev ~ 1/cov
}

//------------------------------------------------------------------------------

void golem::data::ItemPart3DHoP::process() {
	if (models.empty() || realisations.empty() || parts.empty() || indices.empty())
		throw Message(Message::LEVEL_ERROR, "ItemPart3DHoP::process(): no models or realisations");

	// compute model to level map
	modelToLevelMap.clear();
	handler.findLevel(models, modelToLevelMap);

	// compute part to level map
	partToLevelMap.clear();
	handler.findLevel(realisations, partToLevelMap);

	// compute level to part map
	levelToPartMap.clear();
	for (data::Part3D::IndexMap::const_iterator i = partToLevelMap.begin(); i != partToLevelMap.end(); ++i)
		levelToPartMap[i->second].insert(i->first);

	// compute bottom-up parts map
	realisationsRev.clear();
	for (data::Part3D::IndexSetMap::const_iterator i = realisations.begin(); i != realisations.end(); ++i)
		for (data::Part3D::IndexSet::const_iterator j = i->second.begin(); j != i->second.end(); ++j)
			realisationsRev[*j].insert(i->first); // reverse map

	// Cloud point to part indices map
	// indices: part->points, indexToPartMap: point->parts
	indexToPartMap.clear();
	for (data::Part3D::IndexSetMap::const_iterator i = indices.begin(); i != indices.end(); ++i) {
		// only first level - skip indirect realisation-point associations
		if (!handler.partIndirect) {
			const data::Part3D::IndexMap::const_iterator level = partToLevelMap.find(i->first);
			if (level == partToLevelMap.end() || level->second > 0)
				continue;
		}

		// check if the part is on the list
		data::Part3D::Part::Map::iterator part = parts.find(i->first);
		if (part == parts.end()) {
			handler.context.warning("ItemPart3DHoP::process(): unable to find part #%u\n", i->first);
			continue;
		}
		
		for (data::Part3D::IndexSet::const_iterator j = i->second.begin(); j != i->second.end(); ++j)
			indexToPartMap[*j].insert(i->first);
	}
	// include with higher-level/indirect part realisations
	for (data::Part3D::IndexSetMap::iterator i = indexToPartMap.begin(); i != indexToPartMap.end(); ++i) {
		data::Part3D::IndexSet bottomUpParts;
		for (data::Part3D::IndexSet::const_iterator j = i->second.begin(); j != i->second.end(); ++j)
			handler.findIndices(realisationsRev, *j, bottomUpParts);
		i->second.insert(bottomUpParts.begin(), bottomUpParts.end());
	}

	// all parts have equal weights
	const Real norm = REAL_ONE / parts.size();
	for (data::Part3D::Part::Map::iterator i = parts.begin(); i != parts.end(); ++i) {
		i->second.weight = norm;
		// getPart3DProperty() will throw if part does not belong to partToLevelMap
		// partToLevelMap is created from realisations, only for this item, while parts are valid for all realisations in the training set
		try {
			i->second.weight *= getPart3DProperty(i->first)->second.weight;// golem::numeric_const<data::Point3D::Real>::ONE;
		}
		catch (const golem::Message&) {
			// ignore
		}
	}
	//// update weights according to the spatial extent (surface area) of parts
	//for (data::Part3D::Part::Map::iterator i = parts.begin(); i != parts.end(); ++i)
	//	i->second.weight = golem::numeric_const<data::Point3D::Real>::ZERO;
	//for (data::Part3D::IndexSetMap::iterator i = indexToPartMap.begin(); i != indexToPartMap.end(); ++i)
	//	for (data::Part3D::IndexSet::const_iterator j = i->second.begin(); j != i->second.end(); ++j) {
	//		data::Part3D::Part::Map::iterator part = parts.find(*j);
	//		if (part == parts.end())
	//			throw Message(Message::LEVEL_ERROR, "ItemPart3DHoP::process(): unknown part #%u", *j);
	//		part->second.weight += golem::numeric_const<data::Point3D::Real>::ONE;
	//	}
	//// normalise with respect to the points which have parts representations
	//for (data::Part3D::Part::Map::iterator i = parts.begin(); i != parts.end(); ++i)
	//	i->second.weight /= (data::Point3D::Real)indexToPartMap.size();
}

void golem::data::ItemPart3DHoP::load(interop::Stream& stream, bool frameTrn) {
	interop::Point3DCloud cloud;
	interop::part3d::IndexSetMap realisations;
	interop::part3d::Part3D::Map parts;
	interop::part3d::IndexSetMap indices;

	// response
	interop::StreamRead(stream, cloud);
	interop::convert(cloud, *this->cloud);
	interop::StreamRead(stream, realisations);
	interop::convert(realisations, this->realisations);
	interop::StreamRead(stream, parts);
	interop::convert(parts, this->parts);
	interop::StreamRead(stream, indices);
	interop::convert(indices, this->indices);

	// common data
	models = handler.models;
	cloudIndex = 0;
	levelIndex = 0;
	partIndex = 0;
	
	// cloud local frame transform
	if (frameTrn) {
		const Mat34 frame = Cloud::getSensorFrame(*this->cloud);
		Cloud::transform(frame, *this->cloud, *this->cloud);
		Cloud::setSensorFrame(frame, *this->cloud); // restore
	}

	// process data
	process();
	
	// make sure files will be updated
	cloudFile.setModified(true);
	part3DFile.setModified(true);
}

void golem::data::ItemPart3DHoP::printPart() const {
	auto toString = [] (data::Part3D::Index index, const data::Part3D::Part::Map& parts) -> std::string {
		data::Part3D::Part::Map::const_iterator part = parts.find(index);
		std::stringstream str;
		str << "(" << index << ", " << (part != parts.end() ? part->second.model : data::Part3D::INDEX_DEFAULT) << ", " << (part != parts.end() ? part->second.weight : -golem::numeric_const<data::Point3D::Real>::ONE) << ")";
		return str.str();
	};

	std::ofstream ofs(handler.getPath().config + golem::data::HandlerPart3DHoP::FILE_EXT_LOG);

	for (auto &i : models) {
		std::string str;
		str += std::to_string(i.first) + " -> ";
		for (auto &j : i.second)
			str += std::to_string(j) + ", ";
		//handler.context.write("model -> models: %s\n", str.c_str());
		printf("model -> models: %s\n", str.c_str());
		if (ofs) ofs << "model->models: " << str << "\n";
	}
	for (auto &i : realisations) {
		std::string str;
		str += toString(i.first, parts) + " -> ";
		for (auto &j : i.second)
			str += toString(j, parts) + ", ";
		//handler.context.write("part (dn) -> parts: %s\n", str.c_str());
		printf("part (dn) -> parts: %s\n", str.c_str());
		if (ofs) ofs << "part (dn) -> parts: " << str << "\n";
	}
	
	for (auto &i : realisationsRev) {
		std::string str;
		str += toString(i.first, parts) + " -> ";
		for (auto &j : i.second)
			str += toString(j, parts) + ", ";
		//handler.context.write("part (up) -> parts: %s\n", str.c_str());
		printf("part (up) -> parts: %s\n", str.c_str());
		if (ofs) ofs << "part (up) -> parts: " << str << "\n";
	}

	// DEBUG: part -> level
	for (auto &i : partToLevelMap) {
		const std::string str = toString(i.first, parts);
		//handler.context.write("part -> level: %s -> %u\n", str.c_str(), (U32)i.second);
		printf("part -> level: %s -> %u\n", str.c_str(), (U32)i.second);
		if (ofs) ofs << "part -> level: " << str << " -> " << i.second << "\n";
	}
	// DEBUG: level -> parts
	for (auto &i : levelToPartMap) {
		std::string str;
		str += std::to_string(i.first) + " -> ";
		for (auto &j : i.second)
			str += toString(j, parts) + ", ";
		//handler.context.write("level -> parts: %s\n", str.c_str());
		printf("level -> parts: %s\n", str.c_str());
		if (ofs) ofs << "level -> parts: " << str << "\n";
	}

	for (auto &i : indexToPartMap) {
		std::string str;
		str += std::to_string(i.first) + " -> ";
		for (auto &j : i.second)
			str += toString(j, parts) + ", ";
		//handler.context.write("index -> parts: %s\n", str.c_str());
		printf("index -> parts: %s\n", str.c_str());
		if (ofs) ofs << "index -> parts: " << str << "\n";
	}
}

//------------------------------------------------------------------------------

void golem::data::HandlerPart3DHoP::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	golem::data::Handler::Desc::load(context, xmlcontext);

	golem::XMLData("host", host, xmlcontext->getContextFirst("server", false), false);
	golem::XMLData("port", port, xmlcontext->getContextFirst("server", false), false);

	cloudAppearance.xmlData(xmlcontext->getContextFirst("cloud appearance", false), false);
	golem::XMLData("suffix", cloudSuffix, xmlcontext->getContextFirst("cloud", false), false);
	golem::XMLData("frame_trn", cloudFrameTrn, xmlcontext->getContextFirst("cloud", false), false);

	golem::XMLData(partColour, xmlcontext->getContextFirst("part3D appearance colour", false), false);
	golem::XMLData(partColourActive, xmlcontext->getContextFirst("part3D appearance colour_active", false), false);
	golem::XMLData(partColourIndirect, xmlcontext->getContextFirst("part3D appearance colour_indirect", false), false);
	golem::XMLData("frame_show", partFrameShow, xmlcontext->getContextFirst("part3D appearance", false), false);
	golem::XMLData(partFrameSize, xmlcontext->getContextFirst("part3D appearance frame_size", false), false);
	golem::XMLData(partFrameSizeSelect, xmlcontext->getContextFirst("part3D appearance frame_size_select", false), false);
	golem::XMLData("point_size", partCloudPointSize, xmlcontext->getContextFirst("part3D appearance", false), false);
	golem::XMLData("suffix", part3DSuffix, xmlcontext->getContextFirst("part3D", false), false);

	partProperties.setToDefault();
	golem::data::XMLData(partProperties, xmlcontext->getContextFirst("part3D properties"), false);

	golem::XMLData("normal_std_dev", normalStdDev, xmlcontext->getContextFirst("properties"), false);
	golem::XMLData("part_indirect", partIndirect, xmlcontext->getContextFirst("properties"), false);

	golem::XMLData("debug", debug, const_cast<golem::XMLContext*>(xmlcontext), false);
}

golem::data::Handler::Ptr golem::data::HandlerPart3DHoP::Desc::create(golem::Context &context) const {
	Handler::Ptr handler(new HandlerPart3DHoP(context));
	to<HandlerPart3DHoP>(handler.get())->create(*this);
	return handler;
}

//------------------------------------------------------------------------------

const std::string golem::data::HandlerPart3DHoP::FILE_EXT_CONTACT_PART3D = ".p3d";
const std::string golem::data::HandlerPart3DHoP::FILE_EXT_LOG = ".log";

golem::data::HandlerPart3DHoP::HandlerPart3DHoP(golem::Context &context) : Handler(context), cloudInfoRequest(false), cloudLoadRequest(false), cloudRequest(0), levelRequest(0), partRequest(0) {
}

void golem::data::HandlerPart3DHoP::create(const Desc& desc) {
	golem::data::Handler::create(desc);

	debug = desc.debug;

	host = desc.host;
	port = desc.port;

	cloudFrameTrn = desc.cloudFrameTrn;

	cloudAppearance = desc.cloudAppearance;
	cloudSuffix = desc.cloudSuffix;

	part3DSuffix = desc.part3DSuffix;
	partColour = desc.partColour;
	partColourActive = desc.partColourActive;
	partColourIndirect = desc.partColourIndirect;
	partFrameShow = desc.partFrameShow;
	partFrameSize = desc.partFrameSize;
	partFrameSizeSelect = desc.partFrameSizeSelect;
	partCloudPointSize = desc.partCloudPointSize;

	partProperties = desc.partProperties;

	normalStdDev = desc.normalStdDev;
	partIndirect = desc.partIndirect;

	try {
		// open connection to Part3D HoP server
		interop::Stream::Ptr stream = client.connect<interop::StreamSocket>(host, port);

		// query available point cloud names
		interop::StreamWrite(*stream, interop::part3d::HOP_REQ_TRAINING_CLOUD_LIST);
		interop::StreamRead(*stream, names);

		// query models
		interop::part3d::IndexSetMap models;
		interop::StreamWrite(*stream, interop::part3d::HOP_REQ_PART_MODEL_LIST);
		interop::StreamRead(*stream, models);
		interop::convert(models, this->models);

		process();

		// DEBUG: models
		if (debug)
			printModel();

		// print info
		context.debug("HandlerPart3DHoP::create(): number of point clouds: %u, number of part models: %u\n", (U32)names.size(), (U32)models.size());
	}
	catch (const std::exception& ex) {
		names.clear();
		models.clear();
		context.warning("HandlerPart3DHoP::create(): Requests unavailable (%s) for host: %s, port: %i\n", ex.what(), host.c_str(), port);
	}

	transformInterfaces = {
		"ItemImage",
	};
}

void golem::data::HandlerPart3DHoP::findLevel(const data::Part3D::IndexSetMap& map, data::Part3D::IndexMap& level) const {
	// find levels of models from the tree
	std::function<data::Part3D::Index(const data::Part3D::IndexSetMap&, data::Part3D::IndexSetMap::const_iterator, data::Part3D::IndexMap&)> find = [&](const data::Part3D::IndexSetMap& map, data::Part3D::IndexSetMap::const_iterator ptr, data::Part3D::IndexMap& depth) -> data::Part3D::Index {
		data::Part3D::Index level = 0; // at least level 0

		// map is empty
		if (ptr == map.end())
			return level;

		// check if it is already on the map
		data::Part3D::IndexMap::const_iterator i = depth.find(ptr->first);
		if (i != depth.end())
			if (i->second == data::Part3D::INDEX_DEFAULT) {
				context.warning("HandlerPart3DHoP::findLevel(): graph cycle detected for element #%u\n", i->first);
				return level;
			}
			else
				return i->second;

		// invalidate level
		depth[ptr->first] = data::Part3D::INDEX_DEFAULT;

		// not on the map, can be determined iteratively
		for (data::Part3D::IndexSet::const_iterator j = ptr->second.begin(); j != ptr->second.end(); ++j)
			level = std::max(level, find(map, map.find(*j), depth) + 1);
		return depth[ptr->first] = level;
	};

	for (data::Part3D::IndexSetMap::const_iterator i = map.begin(); i != map.end(); ++i)
		(void)find(map, i, level);
}

void golem::data::HandlerPart3DHoP::findIndices(const Part3D::IndexSetMap& map, data::Part3D::Index index, data::Part3D::IndexSet& incl, const data::Part3D::IndexSet* excl) const {
	// graph search procedure - [&] obligatory for lambda (runtime error otherwise)
	// return if on exclusion list
	if (excl && excl->find(index) != excl->end())
		return;
	// insert to the inclusion list
	incl.insert(index);
	// find sub-part indices
	data::Part3D::IndexSetMap::const_iterator set = map.find(index);
	if (set != map.end())
		for (data::Part3D::IndexSet::const_iterator i = set->second.begin(); i != set->second.end(); ++i)
			findIndices(map, *i, incl, excl);
}

void golem::data::HandlerPart3DHoP::process() {
	// compute model to level map
	modelToLevelMap.clear();
	findLevel(models, modelToLevelMap);

	// compute level to model map
	levelToModelMap.clear();
	for (data::Part3D::IndexMap::const_iterator i = modelToLevelMap.begin(); i != modelToLevelMap.end(); ++i)
		levelToModelMap[i->second].insert(i->first);
}

void golem::data::HandlerPart3DHoP::printModel() const {
	for (auto &i : models) {
		std::string str;
		str += std::to_string(i.first) + " -> ";
		for (auto &j : i.second)
			str += std::to_string(j) + ", ";
		context.write("model -> models: %s\n", str.c_str());
	}
	// DEBUG: model -> level
	for (auto &i : modelToLevelMap)
		context.write("model -> level: %u -> %u\n", i.first, i.second);
	// DEBUG: level -> models
	for (auto &i : levelToModelMap) {
		std::string str;
		str += std::to_string(i.first) + " -> ";
		for (auto &j : i.second)
			str += std::to_string(j) + ", ";
		context.write("level -> models: %s\n", str.c_str());
	}
}

//------------------------------------------------------------------------------

golem::data::Item::Ptr golem::data::HandlerPart3DHoP::create() const {
	return Item::Ptr(new ItemPart3DHoP(*const_cast<HandlerPart3DHoP*>(this)));
}

//------------------------------------------------------------------------------

void golem::data::HandlerPart3DHoP::generate(Item::Map& ouput) {
	if (names.empty())
		throw Message(Message::LEVEL_ERROR, "HandlerPart3DHoP::generate(): models not available from host: %s, port: %i\n", host.c_str(), port);

	StringSeq names;
	for (StringSeq::const_iterator i = this->names.begin(); i != this->names.end(); ++i) {
		const size_t pos = i->find_last_of("\\/|?");
		names.push_back(i->substr(pos != std::string::npos ? pos + 1 : 0, std::string::npos));
	}

	Menu::Ptr menu(getUICallback() && getUICallback()->hasInputEnabled() ? new Menu(context, *getUICallback()) : nullptr);
	std::string prefix;
	if (menu != nullptr)
		menu->readString("Trim prefix: ", prefix);
	//std::string suffix;
	//if (menu != nullptr)
	//	menu->readString("Trim suffix: ", suffix);

	typedef std::function<bool(const std::string&, const std::string&, size_t)> Stop;
	auto find = [] (const StringSeq& list, size_t& ptr, Stop stop) {
		for (; list.size() > 1; ++ptr)
			for (StringSeq::const_iterator i = ++list.begin(); i != list.end(); ++i)
				if (stop(*list.begin(), *i, ptr))
					return;
	};
	size_t begin = 0;
	find(names, begin, [] (const std::string& l, const std::string& r, size_t p) -> bool { return l.size() <= p || r.size() <= p || *(l.begin() + p) != *(r.begin() + p); });
	size_t end = 0;
	find(names, end, [] (const std::string& l, const std::string& r, size_t p) -> bool { return l.size() <= p || r.size() <= p || *(l.rbegin() + p) != *(r.rbegin() + p); });

	interop::Stream::Ptr stream = client.connect<interop::StreamSocket>(host, port);
	for (size_t i = 0; i < names.size(); ++i) {
		const size_t pprefix = names[i].find(prefix, begin);
		if (pprefix != std::string::npos)
			begin = pprefix + prefix.length();

		const std::string name = names[i].substr(begin, names[i].size() - end - begin);

		// load training data
		Item::Ptr item(create());
		ItemPart3DHoP* itemPart3DHoP = to<ItemPart3DHoP>(item.get());

		context.write("Loading cloud #%u: %s -> %s ... ", i + 1, this->names[i].c_str(), name.c_str());

		interop::StreamWrite(*stream, interop::part3d::HOP_REQ_TRAINING_DATA);
		interop::StreamWrite(*stream, this->names[i]);
		const_cast<ItemPart3DHoP&>(*itemPart3DHoP).load(*stream, cloudFrameTrn);

		// DEBUG: parts
		if (debug)
			itemPart3DHoP->printPart();

		context.write("Points: %u, Realisations: %u, Indices: %u\n", (U32)itemPart3DHoP->cloud->size(), (U32)itemPart3DHoP->parts.size(), (U32)itemPart3DHoP->indices.size());
		ouput.insert(std::make_pair(name, item));

		// render
		createRender(*itemPart3DHoP);
	}
}

//------------------------------------------------------------------------------

Item::Ptr golem::data::HandlerPart3DHoP::transform(const Item::List& input) {
	const data::Normal3D* points = input.empty() || input.front()->second == nullptr ? nullptr : is<const data::Normal3D>(input.front()->second.get());
	if (!points)
		throw Message(Message::LEVEL_ERROR, "HandlerPart3DHoP::transform(): data::Point3D interface required\n");

	Item::Ptr item(create());
	ItemPart3DHoP* itemPart3DHoP = to<ItemPart3DHoP>(item.get());

	interop::Stream::Ptr stream = client.connect<interop::StreamSocket>(host, port);

	// block item rendering (hasRenderBlock()=true)
	RenderBlock renderBlock(*this);

	// query point cloud
	interop::StreamWrite(*stream, interop::part3d::HOP_REQ_TEST_DATA);
	interop::Point3DCloud cloud;
	// dimensions
	size_t width = 0, height = 0;
	points->getDimensions(width, height);
	if (width <= 1 || height <= 1)
		throw Message(Message::LEVEL_ERROR, "HandlerPart3DHoP::transform(): invalid cloud dimensions %ux%u\n", (U32)width, (U32)height);
	interop::convert(width, cloud.width);
	interop::convert(height, cloud.height);
	// points
	for (size_t i = 0; i < points->getSize(); ++i) {
		interop::Point3D point;
		interop::convert((const Point3D::Vec3&)points->getPoint(i), point.position);
		interop::convert(points->getNormal(i), point.normal);
		interop::convert(points->getColour(i), point.colour);
		cloud.push_back(point);
	}
	// frame
	golem::Mat34 frame;
	points->getSensorFrame(frame);
	interop::convert(golem::RBCoord(frame), cloud.frame);
	// send
	interop::StreamWrite(*stream, cloud);
	
	// realisations
	const_cast<ItemPart3DHoP&>(*itemPart3DHoP).load(*stream, cloudFrameTrn);

	// DEBUG: parts
	if (debug)
		itemPart3DHoP->printPart();

	itemPart3DHoP->cloudFile.setModified(true);
	itemPart3DHoP->part3DFile.setModified(true);

	return item;
}

const StringSeq& golem::data::HandlerPart3DHoP::getTransformInterfaces() const {
	return transformInterfaces;
}

bool golem::data::HandlerPart3DHoP::isTransformSupported(const Item& item) const {
	return is<const data::Normal3D>(&item) != nullptr;
}

//------------------------------------------------------------------------------

void golem::data::HandlerPart3DHoP::createRender(const ItemPart3DHoP& item) {
	bool update = levelRequest != 0 || partRequest != 0;

	if (cloudInfoRequest) {
		//printModel();
		item.printPart();
		cloudInfoRequest = false;
	}

	if (cloudLoadRequest) {
		try {
			cloudLoadRequest = false;
			context.write("Loading cloud #%u: %s ... ", item.cloudIndex + 1, names[item.cloudIndex].c_str());

			// load training data
			interop::Stream::Ptr stream = client.connect<interop::StreamSocket>(host, port);
			interop::StreamWrite(*stream, interop::part3d::HOP_REQ_TRAINING_DATA);
			interop::StreamWrite(*stream, names[item.cloudIndex]);
			const_cast<ItemPart3DHoP&>(item).load(*stream, cloudFrameTrn);

			// DEBUG: parts
			if (debug)
				item.printPart();

			context.write("Points: %u, Realisations: %u, Indices: %u\n", (U32)item.cloud->size(), (U32)item.parts.size(), (U32)item.indices.size());

			// update cloud index
			update = true;
		}
		catch (const std::exception& ex) {
			context.write("HandlerPart3DHoP::createRender(): %s\n", ex.what());
		}
	}

	if (cloudRequest != 0 && !names.empty()) {
		const_cast<ItemPart3DHoP&>(item).cloudIndex = cloudRequest < 0 ? std::max((I32)0, (I32)item.cloudIndex + cloudRequest) : std::min((I32)names.size() - 1, (I32)item.cloudIndex + cloudRequest);
		context.write("Cloud #%u: %s\n", item.cloudIndex + 1, names[item.cloudIndex].c_str());
		cloudRequest = 0;
	}

	if (update && !item.realisationsRev.empty()) {
		const U32 levelIndex = golem::Math::clamp(I32(item.levelIndex) + levelRequest, I32(0), I32(item.levelToPartMap.size() - 1));
		if (item.levelIndex != levelIndex) {
			partRequest = 0;
			const_cast<ItemPart3DHoP&>(item).partIndex = 0;
		}

		const_cast<ItemPart3DHoP&>(item).levelIndex = levelIndex;
		data::Part3D::IndexSetMap::const_iterator parts = item.levelToPartMap.find(item.levelIndex);
		if (parts != item.levelToPartMap.end() && !parts->second.empty())
			const_cast<ItemPart3DHoP&>(item).partIndex = golem::Math::clamp(I32(item.partIndex) + partRequest, I32(0), I32(parts->second.size() - 1));

		levelRequest = 0;
		partRequest = 0;
	}

	UI::CriticalSectionWrapper cs(getUICallback());

	rendererParts.reset();
	rendererCloud.reset();

	data::Part3D::IndexSetMap::const_iterator parts = item.levelToPartMap.find(item.levelIndex);
	if (parts != item.levelToPartMap.end() && !parts->second.empty()) {
		// from index to id
		data::Part3D::IndexSet::const_iterator id = parts->second.begin();
		std::advance(id, std::min(parts->second.size() - 1, (size_t)item.partIndex));

		// find parts' ids
		data::Part3D::IndexSet indices, indicesRev, indicesTop;
		// top-down parts
		findIndices(item.realisations, *id, indices);
		// bottom-up - find top-most root part which contains id
		findIndices(item.realisationsRev, *id, indicesRev);
		indicesRev.erase(*id);
		for (data::Part3D::IndexSet::const_iterator i = indicesRev.begin(); i != indicesRev.end(); ++i)
			findIndices(item.realisations, *i, indicesTop, &indices);

		// Cloud points and realisation frames
		typedef std::map<data::Part3D::Index, Mat34> Mat34Map;
		auto findFramesAndPoints = [&](const data::Part3D::IndexSet& indices, Mat34Map& frames, data::Part3D::IndexSet& points, data::Part3D::IndexSet& pointsIndirect) {
			for (data::Part3D::IndexSet::const_iterator i = indices.begin(); i != indices.end(); ++i) {
				// frames
				data::Part3D::Part::Map::const_iterator part = item.parts.find(*i);
				if (part != item.parts.end())
					frames.insert(std::make_pair(*i, part->second.frame));
				else
					context.warning("HandlerPart3DHoP::createRender(): unable to find part #%u\n", *i);
				// points
				data::Part3D::IndexSetMap::const_iterator point = item.indices.find(*i);
				if (point != item.indices.end()) {
					data::Part3D::IndexMap::const_iterator level = item.partToLevelMap.find(*i);
					if (level == item.partToLevelMap.end() || level->second == 0)
						points.insert(point->second.begin(), point->second.end());
					else
						pointsIndirect.insert(point->second.begin(), point->second.end());
				}
			}
		};
		Mat34Map frames;
		data::Part3D::IndexSet points, pointsTop, pointsIndirect, pointsTopIndirect;
		findFramesAndPoints(indices, frames, points, pointsIndirect);
		findFramesAndPoints(indicesTop, frames, pointsTop, pointsTopIndirect);

		// Info message
		data::Part3D::Part::Map::const_iterator part = item.parts.find(*id);
		context.write("Level #%u, Part #%u, Realisation #%u, Model #%i: bottom-parts_{ids, points, indir}={%u, %u, %u}, top-parts_{ids, points, indir}={%u, %u, %u}\n",
			item.levelIndex + 1, item.partIndex + 1, *id, (part != item.parts.end() ? (int)part->second.model : (int)data::Part3D::INDEX_DEFAULT), (U32)indices.size(), (U32)points.size(), (U32)pointsIndirect.size(), (U32)indicesTop.size(), (U32)pointsTop.size(), (U32)pointsTopIndirect.size());

		// DEBUG: parts
		if (debug) {
			std::stringstream indicesStr;
			for (auto &i : indices) indicesStr << i << ",";
			context.write("Parts-bottom: %s\n", indicesStr.str().c_str());
			std::stringstream indicesRevStr;
			for (auto &i : indicesRev) indicesRevStr << i << ",";
			context.write("Parts-reverse: %s\n", indicesRevStr.str().c_str());
			std::stringstream indicesTopStr;
			for (auto &i : indicesTop) indicesTopStr << i << ",";
			context.write("Parts-top: %s\n", indicesTopStr.str().c_str());
		}

		// update renderer - frames
		for (Mat34Map::const_iterator i = frames.begin(); i != frames.end(); ++i) {
			if (i->first == *id)
				rendererParts.addAxes3D(i->second, partFrameSizeSelect);
			else if (partFrameShow)
				rendererParts.addAxes3D(i->second, partFrameSize);
			
			if (debug > 1) {
				if (Math::abs(i->second.R.determinant() - REAL_ONE) > Real(1e-6) || i->second.p.magnitude() > Real(1e+1)) {
					const RBCoord c(i->second);
					context.warning("HandlerPart3DHoP::createRender(): Id #%u: Invalid SE(3) frame: DetErr=%e, NormErr=%e, (%f, %f, %f), (%f, %f, %f, %f)\n", i->first, Math::abs(i->second.R.determinant() - REAL_ONE), Math::abs(c.q.magnitude() - REAL_ONE), c.p.x, c.p.y, c.p.z, c.q.x, c.q.y, c.q.z, c.q.w);
				}
			}
		}
		// update renderer - points
		rendererParts.setPointSize(partCloudPointSize);
		auto addPoints = [&] (const data::Part3D::IndexSet& points, const RGBA& colour, DebugRenderer& renderer) {
			for (data::Part3D::IndexSet::const_iterator i = points.begin(); i != points.end(); ++i) {
				if (*i < item.cloud->size())
					renderer.addPoint(Cloud::getPoint<Real>((*item.cloud)[*i]), colour);
				else
					context.warning("HandlerPart3DHoP::createRender(): invalid cloud point index #%u\n", *i);
			}
		};
		addPoints(points, partColour, rendererParts);
		addPoints(pointsTop, partColourActive, rendererParts);
		addPoints(pointsIndirect, partColourIndirect, rendererParts);
		addPoints(pointsTopIndirect, partColourIndirect, rendererParts);
	}

	if (item.cloud != nullptr)
		cloudAppearance.drawPoints(*item.cloud, rendererCloud);
}

void golem::data::HandlerPart3DHoP::render() const {
	rendererParts.render();
	rendererCloud.render();
}

void golem::data::HandlerPart3DHoP::customRender() const {
}

//------------------------------------------------------------------------------

void golem::data::HandlerPart3DHoP::mouseHandler(int button, int state, int x, int y) {
}

void golem::data::HandlerPart3DHoP::motionHandler(int x, int y) {
}

void golem::data::HandlerPart3DHoP::keyboardHandler(int key, int x, int y) {
	if (hasRenderBlock()) return;

	switch (key) {
	case '3':
		//requestRender();
		break;
	case '4':
		cloudAppearance.mode = Cloud::Appearance::Mode((cloudAppearance.mode + 1) % (Cloud::Appearance::MODE_NORMAL + 1));
		context.write("Cloud appearance: %s\n", Cloud::Appearance::ModeName[size_t(cloudAppearance.mode)]);
		requestRender();
		break;
	case '5':
		levelRequest = -1;
		requestRender();
		break;
	case '6':
		levelRequest = +1;
		requestRender();
		break;
	case '7':
		partRequest = -1;
		requestRender();
		break;
	case '8':
		partRequest = +1;
		requestRender();
		break;
	case '9':
		cloudLoadRequest = true;
		requestRender();
		break;
	case '0':
		cloudInfoRequest = true;
		requestRender();
		break;
	case '(':
		cloudRequest = -1;
		requestRender();
		break;
	case ')':
		cloudRequest = +1;
		requestRender();
		break;
	};
}

//------------------------------------------------------------------------------

template <> void golem::Stream::read(golem::data::Part3D::IndexSetMap::value_type& value) const {
	read(const_cast<data::Part3D::Index&>(value.first));
	read(value.second, value.second.begin());
}

template <> void golem::Stream::write(const golem::data::Part3D::IndexSetMap::value_type& value) {
	write(value.first);
	write(value.second.begin(), value.second.end());
}

//------------------------------------------------------------------------------
