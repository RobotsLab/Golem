/** @file Sensor.cpp
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
#include <Golem/Math/Data.h>
#include <Golem/Plugin/Sensor.h>
//#include <Golem/Math/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void Sensor::Appearance::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData(frameSize, xmlcontext->getContextFirst("frame"), false);
	golem::XMLData("show", frameShow, xmlcontext->getContextFirst("frame"), false);
	golem::XMLData(shapeColour, xmlcontext->getContextFirst("shape"), false);
	golem::XMLData("show", shapeShow, xmlcontext->getContextFirst("shape"), false);
}

//------------------------------------------------------------------------------

void golem::Sensor::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	context.initModule();

	golem::XMLData("snapshot", snapshotHandler, xmlcontext->getContextFirst("handler"), false);
	golem::XMLData("sequence", sequenceHandler, xmlcontext->getContextFirst("handler"), false);

	appearance.load(xmlcontext->getContextFirst("appearance"));
	try {
		golem::XMLData(shapeDesc, shapeDesc.max_size(), xmlcontext->getContextFirst("appearance shape"), "bounds", false);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}

	golem::XMLData("config_joint", configJoint, const_cast<golem::XMLContext*>(xmlcontext), false);
}

//------------------------------------------------------------------------------

Sensor::Sensor(golem::Context& context) : Library(context), configJoint(0), configQuery(nullptr) {
}

void Sensor::create(const Desc& desc) {
	Library::create(desc);

	snapshotHandler = desc.snapshotHandler;
	sequenceHandler = desc.sequenceHandler;

	configJoint = desc.configJoint;
	if (hasVariableMounting() && !desc.configQuery)
		throw golem::Message(golem::Message::LEVEL_CRIT, "Sensor::create(): Null config callback interface");
	if (hasVariableMounting())
		configQuery = desc.configQuery;

	appearance = desc.appearance;
	shape.clear();
	shapeFrame.clear();
	for (golem::Bounds::Desc::Seq::const_iterator i = desc.shapeDesc.begin(); i != desc.shapeDesc.end(); ++i) {
		shape.push_back((*i)->create());
		shapeFrame.push_back(shape.back()->getPose());
	}
}

//------------------------------------------------------------------------------

void golem::Sensor::getConfig(Config& config) const {
	if (configQuery != nullptr) {
		configQuery(configJoint - 1, config);
	}
	else {
		config.setToDefault();
	}
}

//------------------------------------------------------------------------------

void Sensor::draw(const Appearance& appearance, golem::DebugRenderer& renderer) const {
	const Mat34 frame = getFrame();
	if (appearance.shapeShow){
		for (size_t i = 0; i < shape.size(); ++i) {
			shape[i]->setPose(frame * shapeFrame[i]);
			renderer.setColour(appearance.shapeColour);
			renderer.addSolid(*shape[i]);
		}
	}
	if (appearance.frameShow) {
		renderer.addAxes3D(frame, appearance.frameSize);
	}
}

//------------------------------------------------------------------------------

std::string golem::SensorId::toString() const {
	std::stringstream str;
	str << "(";
	if (hasId())
		str << id;
	if (hasId() && hasIndex())
		str << ", ";
	if (hasIndex())
		str << index;
	str << ")";
	return str.str();
}

golem::Sensor* golem::SensorId::findSensor(const Sensor::Seq& sensorSeq) const {
	golem::Sensor* sensor = nullptr;
	if (hasId())
		for (Sensor::Seq::const_iterator i = sensorSeq.begin(); i != sensorSeq.end() && !sensor; ++i)
			if (*i != nullptr && (*i)->getID() == id)
				sensor = *i;
	if (!sensor && hasIndex())
		if ((size_t)index < sensorSeq.size())
			sensor = sensorSeq[index];
		else
			throw golem::Message(golem::Message::LEVEL_ERROR, "SensorId::findSensor(): sensor index out of range %u", index);

	if (!sensor)
		throw golem::Message(golem::Message::LEVEL_ERROR, "SensorId::findSensor(): unable to find sensor %s", toString().c_str());

	return sensor;
}

//------------------------------------------------------------------------------

void golem::findSensor(const Sensor::Map& sensors, const StringSeq& idSeq, Sensor::Seq& sensorSeq) {
	for (StringSeq::const_iterator i = idSeq.begin(); i != idSeq.end(); ++i) {
		golem::Sensor::Map::const_iterator pSensor = std::find_if(sensors.begin(), sensors.end(), [=](const golem::Sensor::Map::value_type& val) -> bool { return val.first == *i; });
		if (pSensor == sensors.end())
			throw Message(Message::LEVEL_CRIT, "findSensor(): unknown sensor id: %s", i->c_str());
		sensorSeq.push_back(pSensor->second.get());
	}
}

//------------------------------------------------------------------------------

void golem::XMLData(golem::SensorId& val, golem::XMLContext* context, bool create) {
	if (!create)
		val.clear();

	try {
		golem::XMLData("id", val.id, context, create);
	}
	catch (golem::MsgXMLParserAttributeNotFound&) {}
	try {
		golem::XMLData("index", val.index, context, create);
	}
	catch (golem::MsgXMLParserAttributeNotFound&) {}
}

//------------------------------------------------------------------------------
