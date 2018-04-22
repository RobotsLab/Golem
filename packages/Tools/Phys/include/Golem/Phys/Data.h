/** @file Data.h
 * 
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_PHYS_DATA_H_
#define _GOLEM_PHYS_DATA_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Data.h>
#include <Golem/Phys/PhysScene.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

void XMLData(PhysScene::Physics& desc, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYS_DATA_H_*/
