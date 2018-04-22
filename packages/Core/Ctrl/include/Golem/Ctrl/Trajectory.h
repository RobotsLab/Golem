/** @file Trajectory.h
 * 
 * Trajectory interface.
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
#ifndef _GOLEM_CTRL_TRAJECTORY_H_
#define _GOLEM_CTRL_TRAJECTORY_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Root.h>
#include <Golem/Math/Extremum.h>
#include <Golem/Math/Derivative.h>
#include <Golem/Defs/Defs.h>
#include <Golem/Sys/Message.h>
#include <Golem/Ctrl/Types.h>

namespace golem {

//------------------------------------------------------------------------------

class MsgTrajectory : public Message {};
class MsgTrajectoryInvalidDesc : public MsgTrajectory {MESSAGE_BODY(MsgTrajectoryInvalidDesc)};

//------------------------------------------------------------------------------

/** Abstract class representing trajectory of moving objects. */
class Trajectory {
public:
	typedef shared_ptr<Trajectory> Ptr;
	typedef _Root<Real>::Seq Seq;

	/** Trajectory description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		
		/** Beginning in seconds */
		Real begin;
		/** End in seconds */
		Real end;
		
		/** Root finder */
		_Root<Real>::Ptr root;
		/** Extremum finder */
		_Extremum<Real>::Ptr extremum;
		/** Derivative finder */
		_Derivative<Real>::Ptr derivative;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		virtual ~Desc() {}
		
		/** Sets the profile parameters to the default values */
		virtual void setToDefault() {
			begin = Real(0.0);
			end = Real(1.0);

			root.reset(new _RootSecant<Real>);
			extremum.reset(new _ExtremumGold<Real>);
			derivative.reset(new _DerivativeSecant<Real>);
		}

		/** Checks if the profile description is valid. */
		virtual bool isValid() const {
			if (root == NULL || extremum == NULL || derivative == NULL)
				return false;

			return true;
		}
		
		/** Creates Trajectory given the description object. 
		* @return		pointer to Trajectory, if no errors have occured;
		*				<code>NULL</code> otherwise 
		*/
		virtual Trajectory::Ptr create() const = 0;
	};

private:
	struct Position : public _Function<Real> {
		const Trajectory *pTrajectory;
		virtual Real get(Real x) const {
			return pTrajectory->getPosition(x);
		}
	};

	struct Velocity : public _Function<Real> {
		const Trajectory *pTrajectory;
		virtual Real get(Real x) const {
			return pTrajectory->getVelocity(x);
		}
	};

	struct Acceleration : public _Function<Real> {
		const Trajectory *pTrajectory;
		virtual Real get(Real x) const {
			return pTrajectory->getAcceleration(x);
		}
	};

	/** Beginning in seconds */
	Real begin;
	/** End in seconds */
	Real end;

	/** Root finder */
	_Root<Real>::Ptr root;
	/** Extremum finder */
	_Extremum<Real>::Ptr extremum;
	/** Derivative finder */
	_Derivative<Real>::Ptr derivative;
	
	Position position;
	Velocity velocity;
	Acceleration acceleration;

public:
	Trajectory();

	/** Descructor */
	virtual ~Trajectory() {}

	/** Creates Trajectory from the description. 
	* @param desc	Trajectory description
	*/
	void create(const Desc& desc);
	
	/** Returns trajectory beginning. */
	inline Real getBegin() const {
		return begin;
	}

	/** Sets trajectory beginning. */
	inline void setBegin(Real begin) {
		this->begin = begin;
	}
	
	/** Returns trajectory end. */
	inline Real getEnd() const {
		return end;
	}

	/** Sets trajectory end. */
	inline void setEnd(Real end) {
		this->end = end;
	}
	
	/** Returns trajectory duration. */
	inline Real getDuration() const {
		return end - begin;
	}

	/** Re-scales the with the specified duration time. */
	virtual void scaleDuration(Real duration = Real(1.0)) = 0;

	/** Returns trajectory length. */
	virtual Real getLength() const;

	/** Scales the with the specified length (total travelled distance) */
	virtual void scaleLength(Real length = Real(1.0)) = 0;
	
	/** Position f:time -> distance */
	virtual Real getPosition(Real t) const = 0;

	/** Inverse trajectory f:distance -> time.
	*	There can exist no more than one solution (travelled distance cannot decrease).
	*/
	virtual bool getPositionInverse(Real &t, Real position) const;

	/** Trajectory minima and maximum */
	virtual void getPositionExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const;
	
	/** Velocity f:time -> velocity */
	virtual Real getVelocity(Real t) const;

	/** Inverse trajectory f:velocity -> time.
	*	There can exist more than one solution.
	*/
	virtual bool getVelocityInverse(Seq &t, Real velocity) const;

	/** Velocity minimum and maximum */
	virtual void getVelocityExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const;

	/** Acceleration f:time -> acceleration */
	virtual Real getAcceleration(Real t) const;

	/** Inverse trajectory f:acceleration -> time.
	*	There can exist more than one solution.
	*/
	virtual bool getAccelerationInverse(Seq &t, Real acceleration) const;

	/** Velocity minimum and maximum */
	virtual void getAccelerationExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const;
};

//------------------------------------------------------------------------------

/** Constant velocity trajectory. */ 
class Polynomial1 : public Trajectory {
public:
	/** Trajectory description */
	class Desc : public Trajectory::Desc {
	public:
		/** Length */
		Real length;

		Desc() {
			Desc::setToDefault();
		}

		/** Sets the trajectory parameters to the default values */
		virtual void setToDefault() {
			Trajectory::Desc::setToDefault();

			length = Real(1.0);
		}

		/** Checks if the trajectory description is valid. */
		virtual bool isValid() const {
			if (!Trajectory::Desc::isValid())
				return false;
			if (length <= REAL_ZERO)
				return false;

			return true;
		}
		
		/** Sets the trajectory parameters to the default values */
		CREATE_FROM_OBJECT_DESC_0(Polynomial1, Trajectory::Ptr)
	};

protected:
	/** Length */
	Real length;

public:
	/** Creates a default trajectory of unit length and unit duration */
	void create(const Desc& desc);
	
	/** Re-scales the trajectory with the specified duration time. */
	virtual void scaleDuration(Real duration = REAL_ONE) {
		setEnd(getBegin() + duration);
	}
	
	/** Scales the trajectory with the specified length (total travelled distance) */
	virtual void scaleLength(Real length = REAL_ONE) {
		this->length = length;
	}

	/** Trajectory minimum and maximum */
	virtual Real getPosition(Real t) const {
		return (t - getBegin())*length/getDuration();
	}
	
	/** Velocity f:time -> velocity */
	virtual Real getVelocity(Real t) const {
		return length/getDuration();
	}

	/** Acceleration f:time -> acceleration */
	virtual Real getAcceleration(Real t) const {
		return Real(0.0);
	}
};

//------------------------------------------------------------------------------

/** 2rd-degree polynomial trajectory.
* <p>
* f(t) = a2*t<sup>2</sup> + a1*t + a0
* <p>
*/
class Polynomial3 : public Trajectory {
public:
	/** Trajectory description */
	class Desc : public Trajectory::Desc {
	public:
		/** Polynomial coefficients */
		Real a[3];

		Desc() {
			Desc::setToDefault();
		}

		/** virtual destructor */
		virtual ~Desc() {}

		/** Sets the trajectory parameters to the default values */
		virtual void setToDefault() {
			Trajectory::Desc::setToDefault();

			// The trajectory of a unit duration and a unit length (for {begin, end} == {0, 1}):
			// f(t) = 2*t^3; f(0) = 0, f(1) = 1
			a[0] = Real(0.0);
			a[1] = Real(0.0);
			a[2] = Real(3.0);
		}

		/** Checks if the trajectory description is valid. */
		virtual bool isValid() const {
			if (!Trajectory::Desc::isValid())
				return false;
			if (a[2] == REAL_ZERO)
				return false;

			return true;
		}

		/** Sets the trajectory parameters to the default values */
		CREATE_FROM_OBJECT_DESC_0(Polynomial3, Trajectory::Ptr)
	};

protected:
	/** Polynomial coefficients */
	Real a[3];

public:
	/** Creates a default trajectory of unit length and unit duration */
	void create(const Desc& desc);

	/** Re-scales the trajectory with the specified duration time. */
	virtual void scaleDuration(Real duration = REAL_ONE);

	/** Scales the trajectory with the specified length (total travelled distance) */
	virtual void scaleLength(Real length = REAL_ONE);

	/** Position and maximum */
	virtual Real getPosition(Real t) const;

	/** Trajectory minima and maximum */
	virtual void getPositionExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const;

	/** Velocity f:time -> velocity */
	virtual Real getVelocity(Real t) const;

	/** Velocity minimum and maximum */
	virtual void getVelocityExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const;

	/** Acceleration f:time -> acceleration */
	virtual Real getAcceleration(Real t) const;

	/** Velocity minimum and maximum */
	virtual void getAccelerationExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const;

	/** Creates trajectory from 3 waypoints.
	* Created trajectory begins at t0 and ends at t1.
	* Symbols d, v, a stand for suitably position, velocity, acceleration.
	*
	* @param t0		waypoint #0 [sec]
	* @param t1		waypoint #1 [sec]
	* @param p0		waypoint position #0
	* @param v0		waypoint velocity #0
	* @param p1		waypoint position #1
	*/
	void set2pvp(Real t0, Real t1, Real p0, Real v0, Real p1);

	/** Creates trajectory from 3 waypoints.
	* Created trajectory begins at t1 and ends at t2.
	* Symbols d, v, a stand for suitably position, velocity, acceleration.
	*
	* @param t0		waypoint #0 [sec]
	* @param t1		waypoint #1 [sec]
	* @param p0		waypoint position #0
	* @param p1		waypoint position #1
	* @param v1		waypoint velocity #1
	*/
	void set2ppv(Real t0, Real t1, Real p0, Real p1, Real v1);

	/** Creates trajectory from 4 waypoints.
	* Created trajectory begins at t1 and ends at t2.
	* Symbols d, v, a stand for suitably position, velocity, acceleration.
	*
	* @param t0		waypoint #0 [sec]
	* @param t1		waypoint #1 [sec]
	* @param t2		waypoint #2 [sec]
	* @param t3		waypoint #3 [sec]
	* @param p0		waypoint position #0
	* @param p1		waypoint position #1
	* @param p2		waypoint position #2
	*/
	void set3ppp(Real t0, Real t1, Real t2, Real p0, Real p1, Real p2);

	/** Returns polynomial coefficients */
	void getCoeffs(F32 *a) const {
		a[0] = F32(this->a[0]);
		a[1] = F32(this->a[1]);
		a[2] = F32(this->a[2]);
	}

	/** Returns polynomial coefficients */
	void getCoeffs(F64 *a) const {
		a[0] = F64(this->a[0]);
		a[1] = F64(this->a[1]);
		a[2] = F64(this->a[2]);
	}

	/** Returns polynomial coefficients */
	const Real *getCoeffs() const {
		return a;
	}
};

//------------------------------------------------------------------------------

/** 3rd-degree polynomial trajectory. 
 * <p>
 * f(t) = a3*t<sup>3</sup> + a2*t<sup>2</sup> + a1*t + a0
 * <p>
 * A polynomial f() within a single window can be unambiguously specified by 
 * positions (p0, p1) and velocities (v0, v1) on its boundaries 
 * at the time t0 and t1 suitably, i.e. there are 4 equations:
 * <p>
 * f(t0) = p0
 * <p>
 * f'(t0) = v0
 * <p>
 * f(t1) = p1
 * <p>
 * f'(t1) = v1
 * <p>
 */ 
class Polynomial4 : public Trajectory {
public:
	/** Trajectory description */
	class Desc : public Trajectory::Desc {
	public:
		/** Polynomial coefficients */
		Real a[4];
		
		Desc() {
			Desc::setToDefault();
		}
		
		/** virtual destructor */
		virtual ~Desc() {}
		
		/** Sets the trajectory parameters to the default values */
		virtual void setToDefault() {
			Trajectory::Desc::setToDefault();

			// The trajectory of a unit duration and a unit length (for {begin, end} == {0, 1}):
			// f(t) = 3*t^2*(1 - 2/3*t) = 3*t^2 - 2*t^3; f(0) = 0, f(1) = 1
			a[0] = Real(0.0);
			a[1] = Real(0.0);
			a[2] = Real(3.0);
			a[3] = Real(-2.0);
		}

		/** Checks if the trajectory description is valid. */
		virtual bool isValid() const {
			if (!Trajectory::Desc::isValid())
				return false;
			if (a[3] == REAL_ZERO)
				return false;

			return true;
		}
		
		/** Sets the trajectory parameters to the default values */
		CREATE_FROM_OBJECT_DESC_0(Polynomial4, Trajectory::Ptr)
	};

protected:
	/** Polynomial coefficients */
	Real a[4];
	
public:
	/** Creates a default trajectory of unit length and unit duration */
	void create(const Desc& desc);
	
	/** Re-scales the trajectory with the specified duration time. */
	virtual void scaleDuration(Real duration = REAL_ONE);
	
	/** Scales the trajectory with the specified length (total travelled distance) */
	virtual void scaleLength(Real length = REAL_ONE);

	/** Position and maximum */
	virtual Real getPosition(Real t) const;
	
	/** Trajectory minima and maximum */
	virtual void getPositionExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const;

	/** Velocity f:time -> velocity */
	virtual Real getVelocity(Real t) const;

	/** Velocity minimum and maximum */
	virtual void getVelocityExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const;
	
	/** Acceleration f:time -> acceleration */
	virtual Real getAcceleration(Real t) const;

	/** Velocity minimum and maximum */
	virtual void getAccelerationExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const;

	/** Creates trajectory from 2 waypoints.
	* Created trajectory begins at t0 and ends at t1.
	* Symbols d, v, a stand for suitably position, velocity, acceleration.
	*
	* @param t0		waypoint #0 [sec]
	* @param t1		waypoint #1 [sec]
	* @param p0		waypoint position #0
	* @param v0		waypoint velocity #0
	* @param p1		waypoint position #1
	* @param v1		waypoint velocity #1
	*/
	void set2pvpv(Real t0, Real t1, Real p0, Real v0, Real p1, Real v1);

	/** Creates trajectory from 2 waypoints.
	* Created trajectory begins at t0 and ends at t1.
	* Symbols d, v, a stand for suitably position, velocity, acceleration.
	*
	* @param t0		waypoint #0 [sec]
	* @param t1		waypoint #1 [sec]
	* @param p0		waypoint position #0
	* @param v0		waypoint velocity #0
	* @param a0		waypoint acceleration #1
	* @param v1		waypoint velocity #1
	*/
	void set2pvav(Real t0, Real t1, Real p0, Real v0, Real a0, Real v1);

	/** Creates trajectory from 3 waypoints.
	* Created trajectory begins at t0 and ends at t1.
	* Symbols d, v, a stand for suitably position, velocity, acceleration.
	*
	* @param t0		waypoint #0 [sec]
	* @param t1		waypoint #1 [sec]
	* @param t2		waypoint #2 [sec]
	* @param p0		waypoint position #0
	* @param v0		waypoint velocity #0
	* @param p1		waypoint position #1
	* @param p2		waypoint position #2
	*/
	void set3pvpp(Real t0, Real t1, Real t2, Real p0, Real v0, Real p1, Real p2);
	
	/** Creates trajectory from 3 waypoints.
	* Created trajectory begins at t1 and ends at t2.
	* Symbols d, v, a stand for suitably position, velocity, acceleration.
	*
	* @param t0		waypoint #0 [sec]
	* @param t1		waypoint #1 [sec]
	* @param t2		waypoint #2 [sec]
	* @param p0		waypoint position #0
	* @param p1		waypoint position #1
	* @param p2		waypoint position #2
	* @param v2		waypoint velocity #2
	*/
	void set3pppv(Real t0, Real t1, Real t2, Real p0, Real p1, Real p2, Real v2);

	/** Creates trajectory from 4 waypoints.
	* Created trajectory begins at t1 and ends at t2.
	* Symbols d, v, a stand for suitably position, velocity, acceleration.
	*
	* @param t0		waypoint #0 [sec]
	* @param t1		waypoint #1 [sec]
	* @param t2		waypoint #2 [sec]
	* @param t3		waypoint #3 [sec]
	* @param p0		waypoint position #0
	* @param p1		waypoint position #1
	* @param p2		waypoint position #2
	* @param p3		waypoint position #3
	*/
	void set4pppp(Real t0, Real t1, Real t2, Real t3, Real p0, Real p1, Real p2, Real p3);

	/** Returns polynomial coefficients */
	void getCoeffs(F32 *a) const {
		a[0] = F32(this->a[0]);
		a[1] = F32(this->a[1]);
		a[2] = F32(this->a[2]);
		a[3] = F32(this->a[3]);
	}

	/** Returns polynomial coefficients */
	void getCoeffs(F64 *a) const {
		a[0] = F64(this->a[0]);
		a[1] = F64(this->a[1]);
		a[2] = F64(this->a[2]);
		a[3] = F64(this->a[3]);
	}

	/** Returns polynomial coefficients */
	const Real *getCoeffs() const {
		return a;
	}
};

//------------------------------------------------------------------------------

/** Trajectory segment */
class GenCoordTrj : public Polynomial4 {
public:
	/** Trajectory description */
	class Desc : public Polynomial4::Desc {
	public:
		/** Creates object from the description. */
		CREATE_FROM_OBJECT_DESC_0(GenCoordTrj, Trajectory::Ptr)
	};
	
	/** Default constructor */
	GenCoordTrj() {
		setBegin(REAL_ZERO);
		setEnd(REAL_ZERO); // zero is required by simulator, use create() otherwise
	}
	/** From boundaries */
	GenCoordTrj(Real t0, Real t1, const GenCoord& c0, const GenCoord& c1) {
		set(t0, t1, c0, c1);
	}
	
	void set(Real t0, Real t1, const GenCoord& c0, const GenCoord& c1) {
		Polynomial4::set2pvpv(t0, t1, c0.pos, c0.vel, c1.pos, c1.vel);
	}

	GenCoord get(Real t) const {
		t = Math::clamp(t, getBegin(), getEnd());
		return GenCoord(getPosition(t), getVelocity(t), getAcceleration(t));
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_TRAJECTORY_H_*/
