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
#include <Golem/Ctrl/Kuka/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(KukaKR5SixxChain::ChainModel &val, XMLContext* context, bool create) {
	ASSERT(context)
	golem::XMLData("L0", val.L0, context->getContextFirst("links"), create);
	golem::XMLData("L10", val.L10, context->getContextFirst("links"), create);
	golem::XMLData("L11", val.L11, context->getContextFirst("links"), create);
	golem::XMLData("L20", val.L20, context->getContextFirst("links"), create);
	golem::XMLData("L21", val.L21, context->getContextFirst("links"), create);
	golem::XMLData("L3", val.L3, context->getContextFirst("links"), create);
	golem::XMLData(val.encoderOffset, val.encoderOffset + KukaKR5SixxChain::NUM_JOINTS, "c", context->getContextFirst("encoder_offset"), create);
}

void golem::XMLData(KukaKR5SixxChain::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	golem::XMLData(val.chainModel, context, create);
}

void golem::XMLData(KukaKR5Sixx::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData((SingleCtrl::Desc&)val, context, create);
	XMLDataPtr<KukaKR5SixxChain::Desc>(val.chains.begin(), val.chains.end(), context, "chain", create);
}

//------------------------------------------------------------------------------

void golem::XMLData(KukaLWRChain::ChainModel &val, XMLContext* context, bool create) {
	ASSERT(context)
	golem::XMLData("L0", val.L0, context->getContextFirst("links"), create);
//	golem::XMLData("L1", val.L1, context->getContextFirst("links"), create);
	golem::XMLData("L2", val.L2, context->getContextFirst("links"), create);
//	golem::XMLData("L3", val.L3, context->getContextFirst("links"), create);
	golem::XMLData("L4", val.L4, context->getContextFirst("links"), create);
//	golem::XMLData("L5", val.L5, context->getContextFirst("links"), create);
//	golem::XMLData("L6", val.L6, context->getContextFirst("links"), create);
	golem::XMLData("L7", val.L7, context->getContextFirst("links"), create);
	golem::XMLData(val.encoderOffset, val.encoderOffset + KukaLWRChain::NUM_JOINTS, "c", context->getContextFirst("encoder_offset"), create);
}

void golem::XMLData(KukaLWRChain::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	golem::XMLData(val.chainModel, context, create);
}

void golem::XMLData(KukaLWR::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData((SingleCtrl::Desc&)val, context, create);
	XMLDataPtr<KukaLWRChain::Desc>(val.chains.begin(), val.chains.end(), context, "chain", create);
}

//------------------------------------------------------------------------------

void golem::XMLData(KukaIIWAChain::ChainModel &val, XMLContext* context, bool create) {
	ASSERT(context)
	golem::XMLData("L0", val.L0, context->getContextFirst("links"), create);
//	golem::XMLData("L1", val.L1, context->getContextFirst("links"), create);
	golem::XMLData("L2", val.L2, context->getContextFirst("links"), create);
//	golem::XMLData("L3", val.L3, context->getContextFirst("links"), create);
	golem::XMLData("L4", val.L4, context->getContextFirst("links"), create);
//	golem::XMLData("L5", val.L5, context->getContextFirst("links"), create);
//	golem::XMLData("L6", val.L6, context->getContextFirst("links"), create);
	golem::XMLData("L7", val.L7, context->getContextFirst("links"), create);
	golem::XMLData(val.encoderOffset, val.encoderOffset + KukaLWRChain::NUM_JOINTS, "c", context->getContextFirst("encoder_offset"), create);
}

void golem::XMLData(KukaIIWAChain::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	golem::XMLData(val.chainModel, context, create);
}

void golem::XMLData(KukaIIWA::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData((SingleCtrl::Desc&)val, context, create);
	XMLDataPtr<KukaIIWAChain::Desc>(val.chains.begin(), val.chains.end(), context, "chain", create);
}

//------------------------------------------------------------------------------
