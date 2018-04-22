/** @file GraphPlanner.h
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
#ifndef _GOLEM_PLANNER_GRAPHPLANNER_H_
#define _GOLEM_PLANNER_GRAPHPLANNER_H_

//------------------------------------------------------------------------------

#include <Golem/Planner/Planner.h>
#include <Golem/Planner/GraphPlanner/PathFinder.h>
#include <Golem/Planner/GraphPlanner/DEKinematics.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadPlannerDesc(void* pContext, void* pXMLContext, void* pPlannerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Performance monitor */
//#define _GRAPHPLANNER_PERFMON

//------------------------------------------------------------------------------

/** Abstract class for Arm movement planinng using Probabilistic Road Map approach. */
class GOLEM_LIBRARY_DECLDIR GraphPlanner : public Planner {
public:
	typedef shared_ptr<GraphPlanner> Ptr;
	friend class Desc;

	/** Path finder description */
	class GOLEM_LIBRARY_DECLDIR PathFinderDesc {
	public:
		/** Global path finder description */
		PathFinder::Desc::Ptr pGlobalPathFinderDesc;
		/** Local path finder description */
		PathFinder::Desc::Ptr pLocalPathFinderDesc;
		/** Bounded distance scale factor */
		Real distScaleFac;
		/** Waypoint generator range factor */
		Real rangeFac;
		/** Local path finder number of iterations */
		U32 numOfIterations;
		/** Local path finder number of trials */
		U32 numOfTrials;

		/** Constructs the description object. */
		PathFinderDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			pGlobalPathFinderDesc.reset(new PathFinder::Desc);
			pGlobalPathFinderDesc->graphSizeOnline = 0;
			pGlobalPathFinderDesc->graphSizeOffline = 10000;
			pLocalPathFinderDesc.reset(new PathFinder::Desc);
			pLocalPathFinderDesc->graphSizeOnline = 1000;
			pLocalPathFinderDesc->graphSizeOffline = 0;
			distScaleFac = Real(0.7);
			rangeFac = Real(0.7);
			numOfIterations = 3;
			numOfTrials = 3;
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (pGlobalPathFinderDesc == NULL || !pGlobalPathFinderDesc->isValid())
				return false;
			if (pLocalPathFinderDesc != NULL && !pLocalPathFinderDesc->isValid())
				return false;
			if (distScaleFac <= REAL_ZERO || distScaleFac > REAL_ONE)
				return false;
			if (rangeFac <= REAL_ZERO)
				return false;
			if (numOfTrials < 1)
				return false;

			return true;
		}
	};

	/** Path optimiser description */
	class GOLEM_LIBRARY_DECLDIR PathOptimisationDesc {
	public:
		/** number of iterations per path waypoint: (0, inf) */
		U32 numOfIterations;
		/** temperature settings: inf > Tinit > Tfinal > 0 */
		Real Tinit, Tfinal;
		/** energy normalization */
		Real Enorm;
		/** crossover probability: [0, 1] */
		Real crossProb;
		/** path (minimum) distance threshold */
		Real distPathThr;
		
		/** Differential trajectory constraint distance factor */
		Real diffDist;
		/** Differential trajectory constraint begin */
		Real diffBeginFac;
		/** Differential trajectory constraint end */
		Real diffEndFac;

		/** Constructs the description object. */
		PathOptimisationDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			numOfIterations = 500;
			Tinit = Real(0.1);
			Tfinal = Real(0.01);
			Enorm = Real(3.0e4);
			crossProb = Real(0.3);
			distPathThr = Real(0.25); // (0, 1)

			diffDist = Real(2.0);
			diffBeginFac = REAL_ZERO;
			diffEndFac = REAL_ZERO;
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (numOfIterations <= 0)
				return false;
			if (Tinit <= REAL_ZERO || Tfinal <= REAL_ZERO)
				return false;
			if (Enorm <= REAL_ZERO)
				return false;
			if (crossProb < REAL_ZERO || crossProb > REAL_ONE)
				return false;
			if (distPathThr <= REAL_ZERO || distPathThr >= REAL_ONE)
				return false;

			if (diffDist < REAL_EPS || diffBeginFac < REAL_ZERO || diffEndFac < REAL_ZERO)
				return false;

			return true;
		}
	};

	/** Local finder description */
	class GOLEM_LIBRARY_DECLDIR LocalFinderDesc {
	public:
		/** Search range */
		std::vector<Real> range;

		/** Constructs the description object. */
		LocalFinderDesc() {
			setToDefault();
		}
		
		/** Sets the parameters to the default values */
		void setToDefault() {
			range.assign(Configspace::DIM, Real(0.1)*REAL_PI);
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			for (std::vector<Real>::const_iterator i = range.begin(); i != range.end(); ++i)
				if (*i <= REAL_ZERO)
					return false;
			return true;
		}
	};

	/** Planner description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Planner::Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		
		/** Path finder description */
		PathFinderDesc pathFinderDesc;
		/** Path optimisation description */
		PathOptimisationDesc pathOptimisationDesc;
		/** Local finder description */
		LocalFinderDesc localFinderDesc;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(GraphPlanner, Planner::Ptr, Controller&)
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Planner::Desc::setToDefault();

			pKinematicsDesc.reset(new DEKinematics::Desc);
			pathFinderDesc.setToDefault();
			pathOptimisationDesc.setToDefault();
			localFinderDesc.setToDefault();
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Planner::Desc::isValid())
				return false;
			
			if (!pathFinderDesc.isValid() || !pathOptimisationDesc.isValid() || !localFinderDesc.isValid())
				return false;

			return true;
		}
	};

protected:
	// Waypoint pointer wrapper
	struct GOLEM_LIBRARY_DECLDIR WaypointPtr {
	private:
		const Waypoint *pWaypoint;

	public:
		typedef std::vector<WaypointPtr> Seq;

		// Cost comparator
		struct GOLEM_LIBRARY_DECLDIR cost_less {
			inline bool operator () (const WaypointPtr &left, const WaypointPtr &right) const {
				return left.pWaypoint->cost < right.pWaypoint->cost;
			}
		};

		const Waypoint &operator * () const {
			return *pWaypoint;
		}
		const Waypoint *operator -> () const {
			return pWaypoint;
		}
		WaypointPtr(const Waypoint *pWaypoint) : pWaypoint(pWaypoint) {
		}
	};

	/** Global path finder */
	PathFinder::Ptr pGlobalPathFinder;
	/** Local path finder */
	PathFinder::Ptr pLocalPathFinder;
	/** Kinematics solver */
	DEKinematics* pKinematics;
	
	/** Path finder description */
	PathFinderDesc pathFinderDesc;
	/** Path optimisation description */
	PathOptimisationDesc pathOptimisationDesc;
	/** Local finder description */
	LocalFinderDesc localFinderDesc;

	/** Global waypoint path */
	Waypoint::Seq globalPath;
	/** Local waypoint path */
	Waypoint::Seq localPath;
	/** Optimised waypoint path */
	Waypoint::Seq optimisedPath;

	/** Population used in IK */
	ConfigspaceCoord::Seq population;

	/** Performs local search on waypoint path */
	virtual bool localFind(const ConfigspaceCoord &begin, const ConfigspaceCoord &end, Waypoint::Seq &localPath);
	
	/** Optimises waypoint path */
	virtual void optimize(Waypoint::Seq &optimisedPath, const ConfigspaceCoord &dbegin, const ConfigspaceCoord &dend) const;

	/** Creates Planner from the description. 
	* @param desc		Planner description
	*/
	void create(const Desc& desc);

	/** Planner constructor */
	GraphPlanner(golem::Controller& controller);

public:
	/** Finds (optimal) trajectory target in the obstacle-free configuration space.
	 */
	virtual bool findTarget(const GenConfigspaceState &begin, const GenWorkspaceChainState& wend, GenConfigspaceState &cend);

	/** Finds obstacle-free (global search) trajectory in the configuration space from begin to end.
	 * @param begin		trajectory begin in the configuration space
	 * @param end		trajectory end in the configuration space
	 * @param path		trajectory
	 * @param iter		instertion point
	 * @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	 */
	virtual bool findGlobalTrajectory(const Controller::State &begin, const Controller::State &end, Controller::Trajectory &trajectory, Controller::Trajectory::iterator iter, const GenWorkspaceChainState* wend = NULL);

	/** Finds obstacle-free (local search) trajectory in the workspace form trajectory workspace increments.
	 * @param cbegin	trajectory begin in the configuration space
	 * @param wbegin	trajectory begin in the workspace
	 * @param end		trajectory end in the workspace
	 * @param path		generated trajectory in the configuration space
	 * @param iter		trajectory instertion point
	 * @param timeOut	maximum work time
	 * @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	 */
	virtual bool findLocalTrajectory(const Controller::State &cbegin, GenWorkspaceChainState::Seq::const_iterator wbegin, GenWorkspaceChainState::Seq::const_iterator end, Controller::Trajectory &trajectory, Controller::Trajectory::iterator iter, MSecTmU32 timeOut = MSEC_TM_U32_INF);

	/** Kinematics solver */
	DEKinematics& getKinematics();
	/** Kinematics solver */
	const DEKinematics& getKinematics() const;

	/** Global path finder */
	virtual PathFinder::Ptr getGlobalPathFinder() const;
	/** Global path finder */
	virtual void setGlobalPathFinder(const PathFinder::Ptr &pPathFinder);

	/** Local path finder */
	virtual PathFinder::Ptr getLocalPathFinder() const;
	/** Local path finder */
	virtual void setLocalPathFinder(const PathFinder::Ptr &pPathFinder);

	/** Path optimiser description */
	virtual const PathOptimisationDesc &getPathOptimisationDesc() const;
	/** Path optimiser description */
	virtual void setPathOptimisationDesc(const PathOptimisationDesc &pathOptimisationDesc);
	
	/** Returns global waypoint path */
	const Waypoint::Seq &getGlobalPath() const {
		return globalPath;
	}
	/** Returns local waypoint path */
	const Waypoint::Seq &getLocalPath() const {
		return localPath;
	}
	/** Returns optimised waypoint path */
	const Waypoint::Seq &getOptimisedPath() const {
		return optimisedPath;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PLANNER_GRAPHPLANNER_H_*/
