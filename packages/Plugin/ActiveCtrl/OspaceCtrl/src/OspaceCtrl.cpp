/** @file OspaceCtrl.cpp
 * 
 * @author	Maxime Adjigble
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/ActiveCtrl/OspaceCtrl/OspaceCtrl.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <GL/glut.h>

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new OspaceCtrl::Desc();
}

//------------------------------------------------------------------------------

void OspaceCtrl::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	ActiveCtrl::Desc::load(context, xmlcontext);

	golem::XMLData("time_out", timeOut, const_cast<golem::XMLContext*>(xmlcontext));
}

//------------------------------------------------------------------------------

OspaceCtrl::OspaceCtrl(golem::Planner &planner, const Sensor::Seq& sensorSeq) : ActiveCtrl(planner, sensorSeq), ctrl(nullptr) {
}
	
OspaceCtrl::~OspaceCtrl() {
	// unregister before releasing, required because they use each other resources
	(void)registerIO();
}

void OspaceCtrl::create(const Desc& desc) {
	ActiveCtrl::create(desc); // throws

	// controller
	if (controllerIDSeq.size() < 1)
		throw Message(Message::LEVEL_CRIT, "OspaceCtrl::create(): At least one controller required!");
	ctrl = is<SingleCtrl>(controllerIDSeq[0].findController(const_cast<golem::Controller&>(controller)));
	if (!ctrl)
		throw Message(Message::LEVEL_CRIT, "OspaceCtrl::create(): SingleCtrl controller required!");

	timeOut = desc.timeOut;

	// register io
	if (!registerIO(ctrl, timeOut))
		throw Message(Message::LEVEL_CRIT, "OspaceCtrl::create(): registerIO timeout! Make sure you are not running controller client.");

	// TODO: initialisation of op space controller

	setActive(desc.active);
}

//------------------------------------------------------------------------------

void OspaceCtrl::setActive(bool active) {
	// TODO: activation/deactivation

	Active::setActive(active);
}

//------------------------------------------------------------------------------

void OspaceCtrl::sysRecv(SingleCtrl* ctrl, golem::Controller::State& state) {
	// ioSysRecv() should be called first, always - do not comment it
	ioSysRecv(state);

	if (isActive() && isRegisteredIO()) {
		// TODO
	}
}

void OspaceCtrl::sysSend(SingleCtrl* ctrl, const Controller::State& prev, Controller::State& next, bool bSendPrev, bool bSendNext) {
	if (isActive() && isRegisteredIO()) {
		// TODO
	}

	// ioSysSend() should be called last, always - do not comment it
	ioSysSend(prev, next, bSendPrev, bSendNext);
}

//------------------------------------------------------------------------------

void OspaceCtrl::render() const {
	renderer.render();
}
	
void OspaceCtrl::keyboardHandler(int key, int x, int y) {
};

void OspaceCtrl::mouseHandler(int button, int state, int x, int y) {
}

void OspaceCtrl::motionHandler(int x, int y) {
}

//------------------------------------------------------------------------------
