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
#ifndef _GOLEM_CTRL_DLR_DATA_H_
#define _GOLEM_CTRL_DLR_DATA_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/DLR/DLRHitHandII.h>
#include <Golem/Ctrl/DLR/DLRHandII.h>
#include <Golem/Sys/XMLParser.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(DLRHitHandIIChain::Desc &val, XMLContext* context, bool create = false);
void XMLData(DLRHitHandII::Desc &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

void XMLData(DLRHandIIChain::Desc &val, XMLContext* context, bool create = false);
void XMLData(DLRHandII::Desc &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_DLR_DATA_H_*/
