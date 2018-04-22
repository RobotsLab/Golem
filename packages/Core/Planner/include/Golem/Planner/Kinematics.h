/** @file Kinematics.h
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
#ifndef _GOLEM_PLANNER_KINEMATICS_H_
#define _GOLEM_PLANNER_KINEMATICS_H_

//------------------------------------------------------------------------------

#include <Golem/Planner/Heuristic.h>
#include <Golem/Math/Rand.h>
#include <Golem/Sys/Message.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgKinematics, Message)
MESSAGE_DEF(MsgKinematicsInvalidDesc, MsgKinematics)

//------------------------------------------------------------------------------

/** Inverse kinematic solver. */
class Kinematics {
public:
	typedef shared_ptr<Kinematics> Ptr;

	/** Kinematics description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
	
		/** Creates Kinematics given the description object. 
		* @return		pointer to the Kinematics, if no errors have occured; <code>NULL</code> otherwise 
		*/
		virtual Kinematics::Ptr create(golem::Heuristic &heuristic) const = 0;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}
		
		/** virtual destructor */
		virtual ~Desc() {}
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			return true;
		}
	};

protected:	
	/** Heuristic */
	golem::Heuristic &heuristic;
	/** Controller */
	golem::Controller& controller;
	/** Context object */
	golem::Context &context;
		
	/** Kinematics constructor */
	Kinematics(golem::Heuristic &heuristic);

public:
	/** Descructor */
	virtual ~Kinematics();

	/** Inverse kinematics solver
	* @param croot		configuration space root state
	* @param wgoal		workspace goal state
	* @param cgoal		configuration space goal state
	* @param timeOut	maximum work time
	* @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	*/
	virtual bool findGoal(const GenConfigspaceState &croot, const GenWorkspaceChainState &wgoal, GenConfigspaceState &cgoal, MSecTmU32 timeOut = MSEC_TM_U32_INF) = 0;

	/** Access to Heuristic */
	const Heuristic &getHeuristic() const {
		return heuristic;
	}
	Heuristic &getHeuristic() {
		return heuristic;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PLANNER_KINEMATICS_H_*/
