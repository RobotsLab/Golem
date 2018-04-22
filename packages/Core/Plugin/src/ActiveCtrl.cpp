/** @file ActiveCtrl.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Plugin/ActiveCtrl.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::ActiveCtrl::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	context.initModule();

	golem::XMLData("planner_index", plannerIndex, const_cast<golem::XMLContext*>(xmlcontext));

	try {
		controllerIDSeq.clear();
		golem::XMLData(controllerIDSeq, golem::Chainspace::DIM, const_cast<golem::XMLContext*>(xmlcontext), "controller");
	}
	catch (const golem::MsgXMLParserNameNotFound&) {}

	try {
		sensorIDSeq.clear();
		golem::XMLData(sensorIDSeq, sensorIDSeq.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "sensor");
	}
	catch (const golem::MsgXMLParser&) {}

	golem::XMLData("active", active, const_cast<golem::XMLContext*>(xmlcontext));
}

ActiveCtrl::ActiveCtrl(golem::Planner &planner, const Sensor::Seq& sensorSeq) : Active(false), Library(planner.getController().getContext()), planner(planner), controller(planner.getController()), sensorSeqAll(sensorSeq) {
}

void ActiveCtrl::create(const Desc& desc) {
	Library::create(desc);

	plannerIndex = desc.plannerIndex;

	controllerIDSeq = desc.controllerIDSeq;

	std::stringstream controllerSeqStr;
	for (ControllerId::Seq::const_iterator i = controllerIDSeq.begin(); i != controllerIDSeq.end(); ++i) {
		controllerSeqStr << i->toString();
		if (i + 1 != controllerIDSeq.end())
			controllerSeqStr << ", ";
	}

	sensorIDSeq = desc.sensorIDSeq;

	sensorSeq.clear();
	std::stringstream sensorSeqStr;
	if (!sensorSeqAll.empty())
		for (SensorId::Seq::const_iterator i = sensorIDSeq.begin(); i != sensorIDSeq.end(); ++i) {
			Sensor* sensor = i->findSensor(sensorSeqAll);
			sensorSeq.push_back(sensor);

			sensorSeqStr << sensor->getID();
			if (i + 1 != sensorIDSeq.end())
				sensorSeqStr << ",";
		}

	info = controller.getStateInfo();
	Active::setActive(desc.active);

	context.info("ActiveCtrl::create(): id=%s, planner_index=%u, controllers={%s} sensors={%s}\n", getID().c_str(), plannerIndex, controllerSeqStr.str().c_str(), sensorSeqStr.str().c_str());
}

//------------------------------------------------------------------------------
