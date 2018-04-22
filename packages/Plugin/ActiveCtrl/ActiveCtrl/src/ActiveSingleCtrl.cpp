/** @file ActiveSingleCtrl.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/ActiveCtrl/ActiveCtrl/ActiveSingleCtrl.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Sys/XMLData.h>
#include <algorithm>

using namespace golem;

//------------------------------------------------------------------------------

ActiveSingleCtrl::ActiveSingleCtrl() : singleCtrl(nullptr), callbackIO(nullptr) {
}

ActiveSingleCtrl::~ActiveSingleCtrl() {
	(void)registerIO();
}

//------------------------------------------------------------------------------

bool ActiveSingleCtrl::registerIO(golem::SingleCtrl* singleCtrl, golem::MSecTmU32 timeWait) {
	if (singleCtrl && !this->singleCtrl) {
		stateInfo = singleCtrl->getStateInfo();
		callbackIO = singleCtrl->getCallbackIO();
		// install new io
		if (!singleCtrl->setCallbackIO(this, timeWait))
			return false;
	}
	if (!singleCtrl && this->singleCtrl) {
		// restore old io
		if (!this->singleCtrl->setCallbackIO(callbackIO, timeWait))
			return false;
	}
	
	this->singleCtrl = singleCtrl;
	return true;
}

bool ActiveSingleCtrl::isRegisteredIO() const {
	return singleCtrl != nullptr;
}

void ActiveSingleCtrl::ioSysRecv(golem::Controller::State& state) {
	if (singleCtrl && callbackIO)
		callbackIO->sysRecv(singleCtrl, state);
}

void ActiveSingleCtrl::ioSysSend(const Controller::State& prev, Controller::State& next, bool bSendPrev, bool bSendNext) {
	if (singleCtrl && callbackIO)
		callbackIO->sysSend(singleCtrl, prev, next, bSendPrev, bSendNext);
}

//------------------------------------------------------------------------------
