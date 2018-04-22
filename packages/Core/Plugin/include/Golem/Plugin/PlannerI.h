/** @file PlannerI.h
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
#ifndef _GOLEM_PLUGIN_PLANNERI_H_
#define _GOLEM_PLUGIN_PLANNERI_H_

//------------------------------------------------------------------------------

#include <Golem/Planner/Planner.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Plugin/Ctrl.h>

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

/** Initialises handler.
*	(Optionally) Implemented by Handler.
*/
class HandlerPlanner {
public:
	/** HandlerPlanner: Planner index. */
	virtual golem::U32 getPlannerIndex() const = 0;
	/** HandlerPlanner: Sets planner and controllers. */
	virtual void set(const golem::Planner& planner, const ControllerId::Seq& controllerIDSeq) = 0;
};

/** Trajectory collection and tools.
*	(Optionally) Implemented by Item.
*/
class Trajectory {
public:
	/** Trajectory: Profile set. */
	virtual StringSeq getProfiles() const = 0;
	/** Trajectory: Profile set. */
	virtual void setProfile(const std::string& profile) = 0;

	/** Trajectory: Sets waypoint collection with no velocity profile. */
	virtual void setWaypoints(const WaypointCtrl::Seq& waypoints) = 0;
	/** Trajectory: Returns waypoints without velocity profile. */
	virtual const WaypointCtrl::Seq& getWaypoints() const = 0;

	/** Trajectory: Sets manifold. */
	virtual void setManifold(const ManifoldCtrl& manifold) = 0;
	/** Trajectory: Returns manifold. */
	virtual const ManifoldCtrl& getManifold() const = 0;

	/** Returns command trajectory with velocity profile. */
	virtual void createTrajectory(golem::Controller::State::Seq& trajectory) = 0;
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_PLUGIN_PLANNERI_H_*/
