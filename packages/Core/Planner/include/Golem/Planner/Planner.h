/** @file Planner.h
 * 
 * Arm movement control library.
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
#ifndef _GOLEM_PLANNER_PLANNER_H_
#define _GOLEM_PLANNER_PLANNER_H_

#include <Golem/Planner/Kinematics.h>
#include <Golem/Planner/Heuristic.h>
#include <Golem/Planner/Profile.h>
#include <Golem/Math/Rand.h>
#include <Golem/Sys/Message.h>
#include <Golem/Sys/LoadObjectDesc.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgPlanner, Message)
MESSAGE_DEF(MsgPlannerInvalidDesc, MsgPlanner)

//------------------------------------------------------------------------------

class XMLContext;

/** Base class for Arm movement control. */
class Planner : protected Profile::CallbackDist {
public:
	typedef shared_ptr<Planner> Ptr;
	typedef std::vector<Ptr> Seq;
	typedef std::vector<Planner*> PtrSeq;

	/** Callback interface for data synchronization */
	class CallbackDataSync {
	public:
		virtual ~CallbackDataSync() {}
		/** synchronization of collision bounds */
		virtual void syncCollisionBounds() {};
		/** synchronization of find data */
		virtual void syncFindTrajectory(Controller::Trajectory::const_iterator begin, Controller::Trajectory::const_iterator end, const GenWorkspaceChainState* wend = NULL) {};
	};

	/** Planner description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		typedef std::vector<Ptr> Seq;

		friend class Planner;

		/** Heuristic description */
		Heuristic::Desc::Ptr pHeuristicDesc;
		/** Kinematics solver description */
		Kinematics::Desc::Ptr pKinematicsDesc;
		/** Trajectory profile description */
		Profile::Desc::Ptr pProfileDesc;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		virtual ~Desc() {}
		
		/** Creates Planner given the description object. 
		* @return		pointer to the Planner, if no errors have occured;
		*				<code>NULL</code> otherwise 
		*/
		virtual Planner::Ptr create(golem::Controller& controller) const = 0;
		
		/** Sets the planner parameters to the default values */
		virtual void setToDefault() {
			pHeuristicDesc.reset(new Heuristic::Desc);
			pKinematicsDesc.reset();
			pProfileDesc.reset(new Profile::Desc);
		}

		/** Checks if the planner description is valid. */
		virtual bool isValid() const {
			if (pProfileDesc == NULL || !pProfileDesc->isValid())
				return false;
			if (pHeuristicDesc == NULL || !pHeuristicDesc->isValid())
				return false;
			if (pKinematicsDesc == NULL || !pKinematicsDesc->isValid())
				return false;
			
			return true;
		}

		/** Loads planner description from dynamic library.
		* @param context		program context
		* @param libraryPath		library path
		* @param configPath		xml configuration path
		* @return				pointer to the Planner description if no errors have occured, throws otherwise
		*/
		static Planner::Desc::Ptr load(Context* context, const std::string& libraryPath, const std::string& configPath);

		/** Loads planner description from dynamic library.
		* @param context		program context
		* @param xmlcontext		xmlcontext which contains information about planner
		* @return				pointer to the Planner description if no errors have occured, throws otherwise
		*/
		static Planner::Desc::Ptr load(Context* context, XMLContext* xmlcontext);

	private:
		/** Library path */
		std::string libraryPath;
		/** Config path */
		std::string configPath;
	};

private:
	/** Library type */
	std::string type;
	/** Library id */
	std::string id;

	/** Library path */
	std::string libraryPath;
	/** Config path */
	std::string configPath;

	/** Callback interface for auto synchronization of collision bounds */
	CallbackDataSync dummyCallbackDataSync;
	CallbackDataSync* pCallbackDataSync;

protected:
	/** Arm controller interface */
	golem::Controller& controller;
	/** Context object */
	golem::Context &context;

	/** Trajectory profile */
	Profile::Ptr pProfile;
	/** Heuristic object */
	Heuristic::Ptr pHeuristic;
	/** Kinematics solver */
	Kinematics::Ptr pKinematics;
	
	/** the configuration space properties */
	Controller::State::Info stateInfo;
	/** Generator of pseudo random numbers */
	Rand rand;
	
	/** Planner command critical section */
	mutable CriticalSection csCommand;

	/** Profile::CallbackDist: Configuration space coordinate distance metric */
	virtual Real distConfigspaceCoord(const ConfigspaceCoord& prev, const ConfigspaceCoord& next) const;
	/** Profile::CallbackDist: Coordinate distance metric */
	virtual Real distCoord(Real prev, Real next) const;

	/** Profile::CallbackDist: Coordinate enabled state */
	virtual bool distCoordPlanning(const Configspace::Index& index) const;
	/** Profile::CallbackDist: Coordinate interpolate state */
	virtual bool distCoordInterpolation(const Configspace::Index& index) const;

	/** Creates Planner from the description.
	* @param desc		Planner description
	*/
	void create(const Desc& desc);

	/** Planner constructor */
	Planner(golem::Controller& controller);

public:
	/** Descructor */
	virtual ~Planner();

	/** Finds (optimal) trajectory target in the obstacle-free configuration space.
	 * @param begin		trajectory begin in the configuration space
	 * @param wend		query trajectory end (target) in the workspace
	 * @param cend		computed trajectory end (target) in the configuration space
	 * @return			<code>true</code> success; <code>false</code> otherwise 
	 */
	virtual bool findTarget(const GenConfigspaceState &begin, const GenWorkspaceChainState& wend, GenConfigspaceState &cend) = 0;

	/** Finds obstacle-free (global search) trajectory in the configuration space from begin to end.
	 * @param begin		trajectory begin in the configuration space
	 * @param end		trajectory end in the configuration space
	 * @param path		trajectory
	 * @param iter		instertion point
	 * @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	 */
	virtual bool findGlobalTrajectory(const Controller::State &begin, const Controller::State &end, Controller::Trajectory &trajectory, Controller::Trajectory::iterator iter, const GenWorkspaceChainState* wend = NULL) = 0;

	/** Finds obstacle-free (local search) trajectory in the workspace form trajectory workspace increments.
	 * @param cbegin	trajectory begin in the configuration space
	 * @param wbegin	trajectory begin in the workspace
	 * @param end		trajectory end in the workspace
	 * @param path		generated trajectory in the configuration space
	 * @param iter		trajectory instertion point
	 * @param timeOut	maximum work time
	 * @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	 */
	virtual bool findLocalTrajectory(const Controller::State &cbegin, GenWorkspaceChainState::Seq::const_iterator wbegin, GenWorkspaceChainState::Seq::const_iterator end, Controller::Trajectory &trajectory, Controller::Trajectory::iterator iter, MSecTmU32 timeOut = MSEC_TM_U32_INF) = 0;

	/** Differential constraints of trajectory begin
	*/
	virtual Controller::State makeDiffConstraintBegin(const Controller::Trajectory& trajectory) const;

	/** Differential constraints of trajectory end
	*/
	virtual Controller::State makeDiffConstraintEnd(const Controller::Trajectory& trajectory) const;

	/** Returns Trajectory profile
	 * @return			Trajectory profile
	 */
	virtual Profile::Ptr getProfile() const {
		return pProfile;
	}
	/** Sets Trajectory profile
	 * @param pProfile	Trajectory profile
	 */
	virtual void setProfile(const Profile::Ptr &pProfile) {
		this->pProfile = pProfile;
	}

	/** Callback interface for data synchronization
	 * @return						pointer to the interface
	*/
	CallbackDataSync* getCallbackDataSync() {
		return pCallbackDataSync ? pCallbackDataSync : &dummyCallbackDataSync;
	}
	/** Callback interface for data synchronization
	 * @return						pointer to the interface
	*/
	const CallbackDataSync* getCallbackDataSync() const {
		return pCallbackDataSync ? pCallbackDataSync : &dummyCallbackDataSync;
	}
	/** Callback interface for data synchronization
	 * @param pCallbackDataSync		pointer to the interface
	*/
	void setCallbackDataSync(CallbackDataSync* pCallbackDataSync) {
		this->pCallbackDataSync = pCallbackDataSync;
	}
	
	/** Heuristic object */
	Heuristic& getHeuristic() {
		return *pHeuristic;
	}
	const Heuristic& getHeuristic() const {
		return *pHeuristic;
	}
	
	/** Kinematics solver */
	Kinematics& getKinematics() {
		return *pKinematics;
	}
	const Kinematics& getKinematics() const {
		return *pKinematics;
	}

	/** Access to the controller */
	Controller &getController() {
		return controller;
	}
	const Controller &getController() const {
		return controller;
	}

	/** Planner command critical section */
	CriticalSection& getCommandCS() {
		return csCommand;
	}

	/** Library type */
	const std::string& getType() const {
		return type;
	}
	/** Library id */
	const std::string& getID() const {
		return id;
	}

	/** Library path */
	const std::string& getLibPath() const {
		return libraryPath;
	}
	/** Config path */
	const std::string& getConfigPath() const {
		return configPath;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PLANNER_PLANNER_H_*/
