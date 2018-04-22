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
#ifndef _GOLEM_PLANNER_GRAPH_PLANNER_DATA_H_
#define _GOLEM_PLANNER_GRAPH_PLANNER_DATA_H_

//------------------------------------------------------------------------------

#include <Golem/Planner/GraphPlanner/DEKinematics.h>
#include <Golem/Planner/GraphPlanner/PathFinder.h>
#include <Golem/Planner/GraphPlanner/GraphPlanner.h>
#include <Golem/Planner/Data.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void GOLEM_LIBRARY_DECLDIR XMLData(DEKinematics::Desc &val, XMLContext* context, bool create = false);

void GOLEM_LIBRARY_DECLDIR XMLData(WaypointGenerator::Desc &val, XMLContext* context, bool create = false);
void GOLEM_LIBRARY_DECLDIR XMLData(WaypointGenerator::Desc::Ptr &val, XMLContext* context, bool create = false);
void GOLEM_LIBRARY_DECLDIR XMLData(PathFinder::Desc &val, XMLContext* context, bool create = false);

void GOLEM_LIBRARY_DECLDIR XMLData(GraphPlanner::PathFinderDesc &val, XMLContext* context, bool create = false);
void GOLEM_LIBRARY_DECLDIR XMLData(GraphPlanner::PathOptimisationDesc &val, XMLContext* context, bool create = false);
void GOLEM_LIBRARY_DECLDIR XMLData(GraphPlanner::LocalFinderDesc &val, XMLContext* context, bool create = false);
void GOLEM_LIBRARY_DECLDIR XMLData(GraphPlanner::Desc &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PLANNER_GRAPH_PLANNER_DATA_H_*/
