/** @file FTSensor.cpp
 * 
 * @author	Marek Kopicki
 * @author	Sebastian Zurek (UoB)
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Tools/FT.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

FT::Data::Data(golem::SecTmReal timeStamp) : TimeStamp(timeStamp) {
	wrench.setId();
}

FT::Data::Data(const golem::Twist& wrench, golem::SecTmReal timeStamp) : TimeStamp(timeStamp), wrench(wrench) {
}

//------------------------------------------------------------------------------

void golem::FT::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Sensor::Desc::load(context, xmlcontext);

	golem::XMLData(calibrationDesc, xmlcontext->getContextFirst("calibration"), false);
}

//------------------------------------------------------------------------------

golem::FT::FT(golem::Context& context) : Sensor(context) {
}

void golem::FT::create(const Desc& desc) {
	Sensor::create(desc);

	calibration = desc.calibrationDesc->create(*this);

	// load calibration
	calibration->load();
}

//------------------------------------------------------------------------------

void golem::FT::read(Data& data, bool inertia) {
	readSensor(data.wrench, data.timeStamp);
	getConfig(data.config);
	if (inertia)
		getCurrentCalibration()->transform(data.config, data.wrench, data.wrench);
}

golem::Mat34 golem::FT::getFrame() const {
	Config config;
	getConfig(config);
	return getCurrentCalibration()->getFrame(config);
}

//------------------------------------------------------------------------------
