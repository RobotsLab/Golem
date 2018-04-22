/** @file SchunkDexHandSim.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/SchunkDexHandSim/SchunkDexHandSim.h>
#include <Golem/Ctrl/SchunkDexHandSim/Data.h>
#include <Golem/Sys/LoadObjectDesc.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc) {
	loadObjectDesc<golem::SchunkDexHandSim::Desc, Controller::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Controller::Desc::Ptr*)pControllerDesc);
}

//------------------------------------------------------------------------------

SchunkDexHandSim::SchunkDexHandSim(golem::Context& context) : SchunkDexHand(context) {
}

SchunkDexHandSim::~SchunkDexHandSim() {
	SchunkDexHand::release();
}

//------------------------------------------------------------------------------
