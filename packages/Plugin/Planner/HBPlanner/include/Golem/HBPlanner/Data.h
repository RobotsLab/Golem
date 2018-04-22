//------------------------------------------------------------------------------
//               Simultaneous Perception And Manipulation (SPAM)
//------------------------------------------------------------------------------
//
// Copyright (c) 2010 University of Birmingham.
// All rights reserved.
//
// Intelligence Robot Lab.
// School of Computer Science
// University of Birmingham
// Edgbaston, Brimingham. UK.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy 
// of this software and associated documentation files (the "Software"), to deal 
// with the Software without restriction, including without limitation the 
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
// sell copies of the Software, and to permit persons to whom the Software is 
// furnished to do so, subject to the following conditions:
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimers.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimers in 
//       the documentation and/or other materials provided with the 
//       distribution.
//     * Neither the names of the Simultaneous Perception And Manipulation 
//		 (SPAM), University of Birmingham, nor the names of its contributors  
//       may be used to endorse or promote products derived from this Software 
//       without specific prior written permission.
//
// The software is provided "as is", without warranty of any kind, express or 
// implied, including but not limited to the warranties of merchantability, 
// fitness for a particular purpose and noninfringement.  In no event shall the 
// contributors or copyright holders be liable for any claim, damages or other 
// liability, whether in an action of contract, tort or otherwise, arising 
// from, out of or in connection with the software or the use of other dealings 
// with the software.
//
//------------------------------------------------------------------------------
//! @Author:   Claudio Zito
//! @Date:     27/10/2012
//------------------------------------------------------------------------------
#pragma once
#ifndef _GOLEM_HBPLANNER_DATA_H_
#define _GOLEM_HBPLANNER_DATA_H_

//------------------------------------------------------------------------------

#include <Golem\HBPlanner\Belief.h>

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

class BeliefState {
public:
	virtual void set(Belief* belief) = 0;
	virtual Belief::Desc::Ptr getBeliefDesc() const = 0;

	virtual void set(const Mat34 queryTransform, const RBPose::Sample::Seq& poses, const RBPose::Sample::Seq& hypotheses) = 0;
	virtual void setModelPoints(const std::string modelItem, const Mat34 modelFrame, const Cloud::PointSeq& points) = 0;
	virtual void setQueryPoints(const std::string queryItem, const Cloud::PointSeq& points) = 0;

	virtual void setSimObject(const std::string queryItem, const Mat34& queryTransform, const Cloud::PointSeq& points) = 0;

	virtual RBPose::Sample::Seq getPoses() const = 0;
	virtual RBPose::Sample::Seq getHypotheses() const = 0;

	virtual const Mat34& getModelFrame() const = 0;
	virtual const Mat34& getQueryTransform() const = 0;
	virtual const std::string& getModelItem() const = 0;
	virtual const std::string& getQueryItem() const = 0;

	virtual const std::string& getQueryItemSim() const = 0;
	virtual const Mat34& getQueryTransformSim() const = 0;

	virtual void showMeanPoseOnly(const bool show) = 0;
	virtual void showQuery(const bool show) = 0;
	virtual void showGroundTruth(const bool show) = 0;
};

//------------------------------------------------------------------------------

/** Initialises handler.
*	(Optionally) Implemented by Handler.
*/
class HandlerR2GPlan {
public:
	/** Planner index. */
	virtual U32 getPlannerIndex() const = 0;
	/** Sets planner and controllers. */
	virtual void set(Planner& planner, const ControllerId::Seq& controllerIDSeq) = 0;
};

/** Trajectory collection and tools.
*	(Optionally) Implemented by Item.
*/
class R2GTrajectory {
public:
	enum Type {
		NONE = 0,
		APPROACH,
		ACTION
	};

	/** Sets waypoint collection with no velocity profile. */
	virtual void setWaypoints(const WaypointCtrl::Seq& waypoints) = 0;
	/** Trajectory: Sets waypoint collection with no velocity profile. */
	virtual void setWaypoints(const WaypointCtrl::Seq& waypoints, const Type type) = 0;
	/** Returns waypoints without velocity profile. */
	virtual const WaypointCtrl::Seq& getWaypoints() const = 0;
	/** Trajectory: Returns waypoints without velocity profile. */
	virtual const WaypointCtrl::Seq& getWaypoints(const Type type) const = 0;

	/** Returns command trajectory with velocity profile. */
	virtual void createTrajectory(Controller::State::Seq& trajectory) = 0;
	/** Trajectory: Compute grasping and lifting trajectory: Returns waypoints with velocity profile. */
	virtual void createActionTrajectory(const Controller::State& begin, Controller::State::Seq& trajectory) = 0;
	/** (Mycroft) Compute approaching trajectory: Returns waypoints with velocity profile. */
	virtual void createTrajectory(const Controller::State& begin, Controller::State::Seq& trajectory) = 0;
	/** (IR3ne) Compute approaching trajectory: Returns waypoints with velocity profile. */
	virtual void createIGTrajectory(const Controller::State& begin, Controller::State::Seq& trajectory) = 0;

	/** Last waypoint of the reach-to-grasp trajectory */
	size_t pregraspIdx;
};

//------------------------------------------------------------------------------

};	// namespace
}; // namespace

//------------------------------------------------------------------------------

#endif /** _GOLEM_HBPLANNER_DATA_H_ */
