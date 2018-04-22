/** @file Katana300Sim.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/Katana300Sim/Katana300Sim.h>
#include <Golem/Ctrl/Katana300Sim/Data.h>
#include <Golem/Sys/LoadObjectDesc.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc) {
	loadObjectDesc<golem::Katana300Sim::Desc, Controller::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Controller::Desc::Ptr*)pControllerDesc);
}

//------------------------------------------------------------------------------

Katana300Sim::Katana300Sim(golem::Context& context) : KatanaGripper(&context), SingleCtrl(context) {
}

Katana300Sim::~Katana300Sim() {
	SingleCtrl::release();
}

void Katana300Sim::create(const Desc& desc) {
	KatanaGripper::create(desc); // throws
	SingleCtrl::create(desc); // throws
}

bool Katana300Sim::gripperRecvSensorData(SensorDataSet& sensorData, MSecTmU32 timeWait) {
	return true;
}
	
bool Katana300Sim::gripperRecvEncoderData(GripperEncoderData& encoderData, MSecTmU32 timeWait) {
	encoderData.closed = 0;
	encoderData.open = 0;
	encoderData.current = 0;
	return true;
}
	
bool Katana300Sim::gripperOpen(MSecTmU32 timeWait) {
	return true;
}

bool Katana300Sim::gripperClose(const SensorDataSet& sensorThreshold, MSecTmU32 timeWait) {
	return true;
}
	
bool Katana300Sim::gripperFreeze(MSecTmU32 timeWait) {
	return true;
}

//------------------------------------------------------------------------------
