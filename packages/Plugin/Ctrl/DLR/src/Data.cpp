/** @file Data.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/SingleCtrl/Data.h>
#include <Golem/Ctrl/DLR/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(DLRHitHandIIChain::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	golem::XMLData("L0", val.L0, context->getContextFirst("links"), create);
	golem::XMLData("L1", val.L1, context->getContextFirst("links"), create);
	golem::XMLData("L2", val.L2, context->getContextFirst("links"), create);
}

void golem::XMLData(DLRHitHandII::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData((SingleCtrl::Desc&)val, context, create);
	XMLDataPtr<DLRHitHandIIChain::Desc>(val.chains.begin(), val.chains.end(), context, "chain", create);
}

//------------------------------------------------------------------------------

void golem::XMLData(DLRHandIIChain::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	golem::XMLData("L0", val.L0, context->getContextFirst("links"), create);
	golem::XMLData("L1", val.L1, context->getContextFirst("links"), create);
	golem::XMLData("L2", val.L2, context->getContextFirst("links"), create);
}

void golem::XMLData(DLRHandII::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData((SingleCtrl::Desc&)val, context, create);
	XMLDataPtr<DLRHandIIChain::Desc>(val.chains.begin(), val.chains.end(), context, "chain", create);
}

//------------------------------------------------------------------------------
