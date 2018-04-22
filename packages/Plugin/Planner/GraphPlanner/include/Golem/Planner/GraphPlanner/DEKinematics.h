/** @file DEKinematics.h
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
#ifndef _GOLEM_PLANNER_GRAPH_PLANNER_DEKINEMATICS_H_
#define _GOLEM_PLANNER_GRAPH_PLANNER_DEKINEMATICS_H_

//------------------------------------------------------------------------------

#include <Golem/Planner/Kinematics.h>
#include <Golem/Math/Optimisation.h>

//------------------------------------------------------------------------------

/** Performance monitor */
//#define _KINEMATICS_PERFMON

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgKinematicsTimeInterval, MsgKinematics)

//------------------------------------------------------------------------------

/** Approximate inverse kinematic problem solver using differential evolution. */
class GOLEM_LIBRARY_DECLDIR DEKinematics : public Kinematics {
public:
	typedef shared_ptr<DEKinematics> Ptr;

	/** Optimisation vector */
	class GOLEM_LIBRARY_DECLDIR Vec {
	public:
		/** Type */
		typedef ConfigspaceCoord::Type Type;
		/** Size */
		static const size_t Size = Configspace::DIM;

		/** Vector element */
		inline Type& operator [] (size_t index) {
			return elements[index];
		}
		/** Vector element */
		inline const Type& operator [] (size_t index) const {
			return elements[index];
		}

	private:
		/** Elements */
		Type elements[Size];
	};

	/** Thread context */
	class GOLEM_LIBRARY_DECLDIR ThreadData {
	public:
		golem::Heuristic::ThreadData* threadData;
		Waypoint w;

		ThreadData(golem::Heuristic::ThreadData* threadData = NULL) : threadData(threadData) {}
	};

	/** Defines access to configspace coordinates */
	class GOLEM_LIBRARY_DECLDIR Heuristic : public DEHeuristic<ThreadData, Vec, Vec::Type> {
	public:
		typedef shared_ptr<Heuristic> Ptr;
		typedef std::vector<Configspace::Index> ConfigspaceMap;
		typedef std::vector<Type> TypeSeq;
		typedef std::vector<Vec> VecSeq;

#ifdef _KINEMATICS_PERFMON
		mutable U32 collisions;
#endif

		/** Context initialisation */
		Heuristic(DEKinematics& kinematics) : DEHeuristic(kinematics.getHeuristic().getContext()), kinematics(kinematics), heuristic(kinematics.getHeuristic()), controller(heuristic.getController()) {
			initialise();
		}

		/** Initialisation */
		inline void initialise() {
			//chainspaceMap.clear();
			configspaceMap.clear();
			min.clear();
			max.clear();
			for (Configspace::Index j = controller.getStateInfo().getJoints().begin(); j < controller.getStateInfo().getJoints().end(); ++j)
				if (heuristic.getJointDesc()[j]->enabled) {
					configspaceMap.push_back(j);
					min.push_back(heuristic.getMin()[j].pos);
					max.push_back(heuristic.getMax()[j].pos);
				}
			populationPtr = 0;
#ifdef _KINEMATICS_PERFMON
			collisions = 0;
#endif
			resize(configspaceMap.size());
		}
		
		/** Vector sampling */
		inline bool sample(Rand& rand, Vec& nextVector, Type& value) {
			// copy initial population if available
			if (populationPtr < kinematics.getPopulation().size())
				copy(kinematics.getPopulation()[populationPtr++], nextVector);
			else {
				// generate random vectors
				const size_t size = this->size();
				for (size_t i = 0; i < size; ++i) {
					const Configspace::Index j = configspaceMap[i];
					nextVector[i] = rand.nextUniform<Type>(heuristic.getMin()[j].pos, heuristic.getMax()[j].pos);
				}
			}
			return true;
		}
		/** Vector processing */
		inline void process(Vec& vector, ThreadData& threadData) const {
			clamp(vector);
			threadData.w.cpos = kinematics.root.cpos; // use root as default
			copy(vector, threadData.w.cpos);
			threadData.w.setup(controller, true, heuristic.hasCollisionDetection());
		}
		/** Objective function */
		inline Type value(Vec& vector, ThreadData& threadData) const {
			Real c = heuristic.getWorkspaceDistSqr(threadData.w, kinematics.goal);
			
			if (kinematics.distRootFac > REAL_ZERO) {
				c += kinematics.distRootFac*heuristic.getConfigspaceDistSqr(threadData.w.cpos, kinematics.root.cpos);
			}

			return (Type)c;
		}
		/** Collision function */
		inline bool collides(Vec& vector, ThreadData& threadData) const {
#ifdef _KINEMATICS_PERFMON
			if (heuristic.hasCollisionDetection()) {
				++collisions;
				return heuristic.collides(threadData.w, threadData.threadData);
			}
			else
				return false;
#else
			return heuristic.hasCollisionDetection() && heuristic.collides(threadData.w, threadData.threadData);
#endif
		}
		/** Default distance function implemented as Euclidean distance */
		inline Type distance(const Vec& a, const Vec& b) const {
			Type d = numeric_const<Type>::ZERO;
			const size_t size = this->size();
			for (size_t i = 0; i < size; ++i)
				d += Math::sqr(a[i] - b[i]);
			return Math::sqrt(d);
		}

		/** Thread context */
		inline ThreadData threadData() {
			return ThreadData(heuristic.getThreadData());
		}

		/** Conversion */
		inline void clamp(Vec& vec) const {
			const size_t size = this->size();
			for (size_t i = 0; i < size; ++i)
				vec[i] = Math::clamp(vec[i], min[i], max[i]);
		}

		/** Conversion */
		inline void copy(const ConfigspaceCoord& cc, Vec& vec) const {
			const size_t size = this->size();
			for (size_t i = 0; i < size; ++i)
				vec[i] = cc[configspaceMap[i]];
		}
		/** Conversion */
		inline void copy(const Vec& vec, ConfigspaceCoord& cc) const {
			const size_t size = this->size();
			for (size_t i = 0; i < size; ++i)
				cc[configspaceMap[i]] = vec[i];
		}

	private:
		golem::DEKinematics& kinematics;
		golem::Heuristic& heuristic;
		golem::Controller& controller;

		ConfigspaceMap configspaceMap;
		TypeSeq min, max;
		size_t populationPtr;
	};

	/** Optimisation */
	typedef DEOptimisation<Heuristic> Optimisation;

	/** Kinematics description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Kinematics::Desc, public Optimisation::Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
	
		/** root distance global factor */
		Real distRootGlobalFac;
		/** root distance local factor */
		Real distRootLocalFac;

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(DEKinematics, Kinematics::Ptr, golem::Heuristic&)
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Kinematics::Desc::setToDefault();
			Optimisation::Desc::setToDefault();

			distRootGlobalFac = REAL_ZERO;
			distRootLocalFac = REAL_ZERO;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Kinematics::Desc::isValid() || !Optimisation::Desc::isValid())
				return false;
			if (distRootGlobalFac < REAL_ZERO || distRootGlobalFac < REAL_ZERO)
				return false;
			return true;
		}
	};

protected:
	/** Initial population */
	ConfigspaceCoord::Seq population;
	/** Root & goal */
	Waypoint root, goal;

	/** Heuristic */
	Heuristic::Ptr pHeuristic;
	/** Optimisation */
	Optimisation::Ptr pOptimisation;
	/** Kinematics description */
	Desc desc;
	/** root distance factor */
	Real distRootFac;

#ifdef _KINEMATICS_PERFMON
	U32 trials;
	SecTmReal tAvr, tMin, tMax;
	Real errLinAvr, errAngAvr;
#endif

	/** Creates Kinematics from the description.
	* @param desc		Kinematics description
	*/
	void create(const Desc& desc);

	/** Kinematics constructor */
	DEKinematics(golem::Heuristic &heuristic);

public:
	/** Inverse kinematics solver
	* @param begin		begin of the past trajectory in configuration space.
	* @param end		end of the past trajectory in configuration space.
	* @param croot		configuration space root state
	* @param wgoal		workspace goal state
	* @param cgoal		configuration space goal state
	* @param timeOut	maximum work time
	* @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	*/
	virtual bool findGoal(const GenConfigspaceState &croot, const GenWorkspaceChainState &wgoal, GenConfigspaceState &cgoal, MSecTmU32 timeOut = MSEC_TM_U32_INF);

	/** Optimisation */
	const Optimisation* getOptimisation() const {
		return pOptimisation.get();
	}
	/** Optimisation */
	Optimisation* getOptimisation() {
		return pOptimisation.get();
	}

	/** Returns waypoint population */
	void setPopulation(const ConfigspaceCoord::Seq* population = NULL);
	/** Returns waypoint population */
	const ConfigspaceCoord::Seq &getPopulation() const {
		return population;
	}

	/** root distance factor */
	void setDistRootFac(Real distRootFac = REAL_ZERO) {
		this->distRootFac = distRootFac;
	}
	/** root distance factor */
	Real getDistRootFac() const {
		return distRootFac;
	}

	/** Kinematics description */
	void setDesc(const Desc& desc);
	/** Kinematics description */
	const Desc& getDesc() const {
		return desc;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PLANNER_GRAPH_PLANNER_DEKINEMATICS_H_*/
