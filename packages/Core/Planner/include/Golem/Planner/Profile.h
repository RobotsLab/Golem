/** @file Profile.h
 * 
 * Profile interface.
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
#ifndef _GOLEM_PLANNER_PROFILE_H_
#define _GOLEM_PLANNER_PROFILE_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Types.h>
#include <Golem/Ctrl/Controller.h>
#include <Golem/Ctrl/Trajectory.h>

namespace golem {

//------------------------------------------------------------------------------

class MsgProfile : public Message {};
class MsgProfileInvalidDesc : public MsgProfile {MESSAGE_BODY(MsgProfileInvalidDesc)};
class MsgProfileSize : public MsgProfile {MESSAGE_BODY(MsgProfileSize)};
class MsgProfileDistance : public MsgProfile {MESSAGE_BODY(MsgProfileDistance)};
class MsgProfileInverse : public MsgProfile {MESSAGE_BODY(MsgProfileInverse)};

//------------------------------------------------------------------------------

/** Trajectory profile. */
class Profile {
public:
	typedef shared_ptr<Profile> Ptr;

	/** Callback interface for configspace distance metric */
	class CallbackDist {
	public:
		virtual ~CallbackDist() {}
		
		/** Configuration space coordinate distance metric */
		virtual Real distConfigspaceCoord(const ConfigspaceCoord& prev, const ConfigspaceCoord& next) const = 0;
		/** Coordinate distance metric */
		virtual Real distCoord(Real prev, Real next) const = 0;

		/** Prune state sequence state */
		virtual void distRemoved(size_t index) const {}

		/** Coordinate enabled planning */
		virtual bool distCoordPlanning(const Configspace::Index& index) const { return true; }
		/** Coordinate enabled interpolation */
		virtual bool distCoordInterpolation(const Configspace::Index& index) const { return true; }
	};

	/** Profile description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
	
		/** Trajectory description */
		Trajectory::Desc::Ptr pTrajectoryDesc;

		/** Callback interface for configspace distance metric */
		const CallbackDist* pCallbackDist;

		/** Maximum joint velocity (absolute value) normlised to joint velocity limits */
		std::vector<Real> velocity;
		/** Maximum joint acceleration (absolute value) normlised to joint acceleration limits */
		std::vector<Real> acceleration;

		/** Waypoints pruning */
		bool pruning;
		/** Waypoints pruning distance */
		Real pruningDistance;

		/** Creates Profile given the description object. 
		* @return		pointer to the Profile, if no errors have occured; <code>NULL</code> otherwise 
		*/
		CREATE_FROM_OBJECT_DESC_1(Profile, Profile::Ptr, const Controller&)
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}
		
		/** virtual destructor */
		virtual ~Desc() {}
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			pTrajectoryDesc.reset(new Polynomial4::Desc);
			pCallbackDist = NULL;
			
			velocity.assign(Configspace::DIM, Real(1.0));
			acceleration.assign(Configspace::DIM, Real(1.0));
			
			pruning = true;
			pruningDistance = REAL_EPS;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (pTrajectoryDesc == NULL || !pTrajectoryDesc->isValid())
				return false;
			
			if (velocity.empty() || acceleration.empty())
				return false;
			for (std::vector<Real>::const_iterator i = velocity.begin(); i != velocity.end(); ++i)
				if (*i < REAL_ZERO)
					return false;
			for (std::vector<Real>::const_iterator i = acceleration.begin(); i != acceleration.end(); ++i)
				if (*i < REAL_ZERO)
					return false;
			
			if (pruningDistance < REAL_EPS)
				return false;

			return true;
		}
	};

protected:
	/** Controller */
	const golem::Controller& controller;
	/** the configuration space properties */
	Controller::State::Info stateInfo;

	/** Trajectory description */
	Trajectory::Ptr pTrajectory;
	/** Callback interface for configspace distance metric */
	const CallbackDist* pCallbackDist;
	/** Maximum joint velocity (absolute value) normlised to joint velocity limits */
	ConfigspaceCoord velocity;
	/** Maximum joint acceleration (absolute value) normlised to joint acceleration limits */
	ConfigspaceCoord acceleration;

	/** Waypoints pruning */
	bool pruning;
	/** Waypoints pruning distance */
	Real pruningDistance;

	/** Velocity gain */
	void velocityGain(const Polynomial4& trj, Real min, Real max, Real& gain) const;
	/** Acceleration gain */
	void accelerationGain(const Polynomial4& trj, Real min, Real max, Real& gain) const;

	/** Compute velocities */
	template <typename _Iter> void profile(_Iter first, _Iter last) const {
		// 3-rd degre polynomial interpolation
		for (Configspace::Index n = controller.getStateInfo().getJoints().begin(); n < controller.getStateInfo().getJoints().end(); ++n)
			if (pCallbackDist->distCoordPlanning(n)) {
				for (_Iter pi = first, pj = first + 1, pk = first + 2; pj < last; ++pi, ++pj, ++pk) {
					Polynomial4 trj;
					trj.set3pvpp(pi->t, pj->t, pk->t, pi->cpos[n], pi->cvel[n], pj->cpos[n], pk->cpos[n]);
					pj->cvel[n] = trj.getVelocity(pj->t);
					pj->cacc[n] = REAL_ZERO;
				}
			}
			else if (pCallbackDist->distCoordInterpolation(n)) {
				Polynomial4 trj;
				trj.set2pvpv(first->t, last->t, first->cpos[n], first->cvel[n], last->cpos[n], last->cvel[n]);
				for (_Iter pi = first, pj = first + 1, pk = first + 2; pj < last; ++pi, ++pj, ++pk) {
					pj->cpos[n] = trj.getPosition(pj->t);
					pj->cvel[n] = trj.getVelocity(pj->t);
					pj->cacc[n] = REAL_ZERO;
				}
			}
	}

	/** Creates Profile from the description.
	* @param desc		Profile description
	*/
	void create(const Desc& desc);
	/** Profile constructor */
	Profile(const Controller &controller);

public:
	/** Descructor */
	virtual ~Profile();

	/** Maximal joint velocity (absolute value) normlised to joint velocity limits */
	ConfigspaceCoord& getVelocity() {
		return velocity;
	}
	/** Maximal joint velocity (absolute value) normlised to joint velocity limits */
	const ConfigspaceCoord& getVelocity() const {
		return velocity;
	}
	
	/** Maximal joint acceleration (absolute value) normlised to joint acceleration limits */
	ConfigspaceCoord& getAcceleration() {
		return acceleration;
	}
	/** Maximal joint acceleration (absolute value) normlised to joint acceleration limits */
	const ConfigspaceCoord& getAcceleration() const {
		return acceleration;
	}

	/** Waypoints pruning */
	const CallbackDist* getCallbackDist() const {
		return pCallbackDist;
	}
	/** Waypoints pruning */
	void setCallbackDist(const CallbackDist& callbackDist);

	/** Waypoints pruning */
	inline bool hasPruning() const {
		return pruning;
	}
	/** Waypoints pruning */
	inline void setPruning(bool pruning) {
		this->pruning = pruning;
	}

	/** Waypoints pruning distance */
	inline Real getPruningDistance() const {
		return pruningDistance;
	}
	/** Waypoints pruning distance */
	inline void setPruningDistance(Real pruningDistance) {
		this->pruningDistance = pruningDistance;
	}

	/** Creates state sequence from pointers */
	template <typename _Ptr> void create(const _Ptr pbegin, const _Ptr pend, const Controller::State& first, const Controller::State& last, Controller::State::Seq &seq, Controller::State::Seq::iterator& begin, Controller::State::Seq::iterator& end) const {
		const size_t size = size_t(pend - pbegin);
		if (size < 2)
			throw MsgProfileSize(Message::LEVEL_ERROR, "Profile::create(): Insuffucient number of waypoints %u", size);

		for (_Ptr ptr = pbegin; ptr < pend; ++ptr) {
			// first by default
			Controller::State state = first;

			// interpolation
			const Real s = Real(ptr - pbegin) / (size - 1), p[2] = { REAL_ONE - s, s };

			// time stamp
			state.t = p[0] * first.t + p[1] * last.t;

			// coordinates
			for (Configspace::Index n = controller.getStateInfo().getJoints().begin(); n < controller.getStateInfo().getJoints().end(); ++n)
				if (pCallbackDist->distCoordPlanning(n)) {
					state.cpos[n] = ptr->cpos[n];
					state.cvel[n] = state.cacc[n] = REAL_ZERO;
				}
				else if (pCallbackDist->distCoordInterpolation(n)) {
					state.cpos[n] = p[0] * first.cpos[n] + p[1] * last.cpos[n];
					state.cvel[n] = state.cacc[n] = REAL_ZERO;
				}

			begin = ++seq.insert(begin, state);
		}
		
		end = begin;
		begin = end - size;
	}

	/** Profiles given state sequence */
	virtual void profile(Controller::State::Seq &seq, Controller::State::Seq::iterator& begin, Controller::State::Seq::iterator& end) const;
	/** Profiles given state sequence */
	virtual void profile(Controller::State::Seq &seq) const;
	
	/** Initialise coordinates */
	virtual void prepare(Controller::State::Seq::iterator begin, Controller::State::Seq::iterator end) const;
	/** Find time stamps */
	virtual void profileTime(Controller::State::Seq &seq, Controller::State::Seq::iterator& begin, Controller::State::Seq::iterator& end) const;
	/** Find velocities */
	virtual void profileVelocity(Controller::State::Seq::iterator begin, Controller::State::Seq::iterator end) const;
	/** Optimise trajectory */
	virtual void optimise(Controller::State::Seq::iterator begin, Controller::State::Seq::iterator end) const;
	/** Re-scale trajectory duration */
	virtual void rescale(Controller::State::Seq::iterator begin, Controller::State::Seq::iterator end) const;
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PLANNER_PROFILE_H_*/
