/** @file KITHead.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/KIT/KITHead.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const U32 KITHead::CHAIN_NUM_JOINTS[KITHead::NUM_CHAINS] = {KITHead::NUM_JOINTS_NECK, KITHead::NUM_JOINTS_LEFT_EYE, KITHead::NUM_JOINTS_RIGHT_EYE};

KITHead::KITHead(golem::Context& context) : SingleCtrl(context) {
}

//------------------------------------------------------------------------------
