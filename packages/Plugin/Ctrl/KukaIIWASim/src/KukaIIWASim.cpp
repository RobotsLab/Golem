/** @file KukaIIWASim.cpp
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/KukaIIWASim/KukaIIWASim.h>
#include <Golem/Ctrl/KukaIIWASim/Data.h>
#include <Golem/Sys/LoadObjectDesc.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc) {
	loadObjectDesc<golem::KukaIIWASim::Desc, Controller::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Controller::Desc::Ptr*)pControllerDesc);
}

//------------------------------------------------------------------------------

KukaIIWASim::KukaIIWASim(golem::Context& context) : KukaIIWA(context) {
}

KukaIIWASim::~KukaIIWASim() {
	SingleCtrl::release();
}

//------------------------------------------------------------------------------
