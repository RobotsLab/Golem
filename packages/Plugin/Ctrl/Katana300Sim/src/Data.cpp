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

#include <Golem/Ctrl/Katana300Sim/Data.h>
#include <Golem/Ctrl/Katana/Data.h>
#include <Golem/Ctrl/SingleCtrl/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(Katana300Sim::Desc &val, XMLContext* context, bool create) {
	XMLData((SingleCtrl::Desc&)val, context, create);
	XMLData((KatanaGripper::Desc&)val, context, create);
	XMLDataPtr<KatanaChain::Desc>(val.chains.begin(), val.chains.end(), context, "chain", create); // repeat for KatanaChain::Desc
}

//------------------------------------------------------------------------------
