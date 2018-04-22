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
#ifndef _GOLEM_CTRL_SCHUNKDEXHANDSIM_DATA_H_
#define _GOLEM_CTRL_SCHUNKDEXHANDSIM_DATA_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/SchunkDexHandSim/SchunkDexHandSim.h>
#include <Golem/Sys/XMLParser.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(SchunkDexHandSim::Desc &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_SCHUNKDEXHANDSIM_DATA_H_*/
