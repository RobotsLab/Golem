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

#include <Golem/Ctrl/SchunkDexHandSim/Data.h>
#include <Golem/Ctrl/Schunk/Data.h>
#include <Golem/Ctrl/SingleCtrl/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(SchunkDexHandSim::Desc &val, XMLContext* context, bool create) {
	XMLData((SchunkDexHand::Desc&)val, context, create);
}

//------------------------------------------------------------------------------
 