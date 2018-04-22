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

#include <Golem/Ctrl/DLRHitHandII/Data.h>
#include <Golem/Ctrl/DLR/Data.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(DLRHitHandIIDev::Desc &val, XMLContext* context, bool create) {
	XMLData((DLRHitHandII::Desc&)val, context, create);

	golem::XMLData("peer_addr", val.peerAddr, context->getContextFirst("ard"), create);
	golem::XMLData("peer_port", val.peerPort, context->getContextFirst("ard"), create);
	golem::XMLData("local_port", val.localPort, context->getContextFirst("ard"), create);
	golem::XMLData("comm_timeout", val.commTimeout, context->getContextFirst("ard"), create);
	golem::XMLData("init_wakeup", val.initWakeup, context->getContextFirst("ard"), create);
	golem::XMLData("init_comm_sleep", val.initCommSleep, context->getContextFirst("ard"), create);

	golem::XMLData(val.offset, val.offset + DLRHitHandII::NUM_JOINTS, "c", context->getContextFirst("offset"), create);
	golem::XMLData(val.finger, val.finger + DLRHitHandII::NUM_CHAINS, "i", context->getContextFirst("finger"), create);
	golem::XMLData(val.enabled, val.enabled + DLRHitHandII::NUM_CHAINS, "f", context->getContextFirst("enabled"), create);
}

//------------------------------------------------------------------------------
