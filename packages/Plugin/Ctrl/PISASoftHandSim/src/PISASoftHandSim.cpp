/** @file PISASoftHandSim.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/PISASoftHandSim/PISASoftHandSim.h>
#include <Golem/Ctrl/PISASoftHandSim/Data.h>
#include <Golem/Sys/LoadObjectDesc.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc) {
	loadObjectDesc<golem::PISASoftHandSim::Desc, Controller::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Controller::Desc::Ptr*)pControllerDesc);
}

//------------------------------------------------------------------------------

PISASoftHandSim::PISASoftHandSim(golem::Context& context) : PISASoftHand(context) {
}

PISASoftHandSim::~PISASoftHandSim() {
	PISASoftHand::release();
}

//------------------------------------------------------------------------------

void PISASoftHandSim::sysRecv(State& state) {
	// default interpolation
	SingleCtrl::sysRecv(state);
	// apply synergy
	getSynergyMap(getSynergy(state), state);
}

//------------------------------------------------------------------------------