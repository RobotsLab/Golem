/** @file Kinematics.cpp
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

#include <Golem/Planner/Kinematics.h>
#include <Golem/Sys/Context.h>
#include <Golem/Sys/Message.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Kinematics::Kinematics(golem::Heuristic &heuristic) : heuristic(heuristic), controller(heuristic.getController()), context(controller.getContext()) {
}

Kinematics::~Kinematics() {
}

//------------------------------------------------------------------------------
