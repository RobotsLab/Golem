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

#include <Golem/Ctrl/KukaKR5SixxSim/Data.h>
#include <Golem/Ctrl/Kuka/Data.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(KukaKR5SixxSim::Desc &val, XMLContext* context, bool create) {
	XMLData((KukaKR5Sixx::Desc&)val, context, create);
}

//------------------------------------------------------------------------------
