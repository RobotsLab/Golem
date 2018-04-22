/** @file KukaLWRSim.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/KukaLWRSim/KukaLWRSim.h>
#include <Golem/Ctrl/KukaLWRSim/Data.h>
#include <Golem/Sys/LoadObjectDesc.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc) {
	loadObjectDesc<golem::KukaLWRSim::Desc, Controller::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Controller::Desc::Ptr*)pControllerDesc);
}

//------------------------------------------------------------------------------

KukaLWRSim::KukaLWRSim(golem::Context& context) : KukaLWR(context) {
}

KukaLWRSim::~KukaLWRSim() {
	SingleCtrl::release();
}

//------------------------------------------------------------------------------
