/** @file ActiveSingleCtrl.h
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_ACTIVECTRL_ACTIVECTRL_ACTIVE_SINGLECTRL_H_
#define _GOLEM_ACTIVECTRL_ACTIVECTRL_ACTIVE_SINGLECTRL_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/ActiveCtrl.h>
#include <Golem/Ctrl/SingleCtrl/SingleCtrl.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Active manipulator controll using force sensors. */
class ActiveSingleCtrl : private golem::SingleCtrl::CallbackIO {
public:
	typedef golem::shared_ptr<ActiveSingleCtrl> Ptr;

private:
	/** Controller */
	golem::SingleCtrl* singleCtrl;
	/** Original golem::SingleCtrl::CallbackIO */
	golem::SingleCtrl::CallbackIO* callbackIO;

protected:
	/** the configuration space properties */
	golem::Controller::State::Info stateInfo;

	/** Data receive */
	void ioSysRecv(golem::Controller::State& state);
	/** Data send */
	void ioSysSend(const golem::Controller::State& prev, golem::Controller::State& next, bool bSendPrev, bool bSendNext);

	/** Constructor */
	ActiveSingleCtrl();

public:
	/** Descructor */
	virtual ~ActiveSingleCtrl();

	/** Register callback io */
	virtual bool registerIO(golem::SingleCtrl* singleCtrl = nullptr, golem::MSecTmU32 timeWait = golem::MSEC_TM_U32_INF);
	/** Is registered callback io  */
	virtual bool isRegisteredIO() const;
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_ACTIVECTRL_ACTIVECTRL_ACTIVE_SINGLECTRL_H_*/
