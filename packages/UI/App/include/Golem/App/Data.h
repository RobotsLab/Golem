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
#ifndef _GOLEM_APP_DATA_H_
#define _GOLEM_APP_DATA_H_

//------------------------------------------------------------------------------

//#include <Golem/App/GraphRenderer.h>
#include <Golem/App/UIController.h>
#include <Golem/App/UIPlanner.h>
#include <Golem/Sys/XMLParser.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

void XMLData(UIPlannerVis::BoundsData::Appearance &val, XMLContext* context, bool create = false);
void XMLData(GraphRenderer &val, XMLContext* context, bool create = false);
void XMLData(UIPlannerVis::GraphRenderData &val, XMLContext* context, bool create = false);

/** Loads controller description from a given XML context, without loading a driver */
void XMLData(UIControllerVis::Desc &val, XMLContext* xmlcontext, bool create = false);
/** Loads controller description from a given XML context, without loading a driver */
void XMLData(UIController::Desc &val, XMLContext* xmlcontext, bool create = false);
/** Loads controller description from a given XML context and loads a driver */
void XMLData(UIController::Desc &val, Context* context, XMLContext* xmlcontext, bool create = false);

/** Loads planner description from a given XML context, without loading a driver */
void XMLData(UIPlannerVis::Desc &val, XMLContext* xmlcontext, bool create = false);
/** Loads planner description from a given XML context, without loading a driver */
void XMLData(UIPlanner::Desc &val, XMLContext* xmlcontext, bool create = false);
/** Loads planner description from a given XML context and loads a driver */
void XMLData(UIPlanner::Desc &val, Context* context, XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_APP_DATA_H_*/
