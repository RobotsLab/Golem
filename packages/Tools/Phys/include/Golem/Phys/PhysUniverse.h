/** @file PhysUniverse.h
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
#ifndef _GOLEM_PHYS_UNIVERSE_H_
#define _GOLEM_PHYS_UNIVERSE_H_

//------------------------------------------------------------------------------

#include <Golem/Sim/Universe.h>
#include <Golem/Phys/PhysScene.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgPhysUniverse, MsgUniverse)
MESSAGE_DEF(MsgPhysUniversePhysXInit, MsgPhysUniverse)
MESSAGE_DEF(MsgPhysUniversePhysXCookInit, MsgPhysUniverse)

//------------------------------------------------------------------------------

/** NVIDIA driver error stream */
class NxErrorStream : public NxUserOutputStream {
private:
	Context &context;

public:
	NxErrorStream(Context &context);
	virtual void reportError(NxErrorCode code, const char* message, const char* file, int line);
	virtual NxAssertResponse reportAssertViolation(const char* message, const char* file, int line);
	virtual void print(const char* message);
};

//------------------------------------------------------------------------------

class PhysUniverse : public Universe {
public:
	friend class PhysScene;

	class Desc : public Universe::Desc {
	public:
		Real skinWidth;
		Real sleepLinVelSquared;
		Real sleepAngVelSquared;
		Real maxAngularVelocity;
		Real bounceThreshold;
		//Real dynFrictScaling;
		//Real staFrictScaling;

		/** Constructs PhysUniverse description. */
		Desc() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			Universe::Desc::setToDefault();

			skinWidth = Real(0.005);//NX_SKIN_WIDTH 0.025
			sleepLinVelSquared = Real(0.05*0.05);//NX_DEFAULT_SLEEP_LIN_VEL_SQUARED 0.15*0.15
			sleepAngVelSquared = Real(0.05*0.05);//NX_DEFAULT_SLEEP_ANG_VEL_SQUARED 0.14*0.14
			maxAngularVelocity = Real(7.0);//NX_MAX_ANGULAR_VELOCITY 7
			bounceThreshold = Real(-2.0); // NX_BOUNCE_THRESHOLD -2
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Universe::Desc::isValid())
				return false;

			if (skinWidth <= REAL_ZERO)
				return false;
			if (sleepLinVelSquared <= REAL_ZERO)
				return false;
			if (sleepAngVelSquared <= REAL_ZERO)
				return false;
			if (maxAngularVelocity <= REAL_ZERO)
				return false;
			if (!Math::isFinite(bounceThreshold))
				return false;

			return true;
		}
		/** Loads description from xml. */
		virtual void load(const golem::XMLContext* xmlcontext);

		/** Creates/initialises the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(PhysUniverse, Universe::Ptr, Context&)
	};

protected:
	NxPhysicsSDK* pNxPhysicsSDK;
	NxErrorStream nxErrorStream;

	/** Process scene */
	virtual void process(Scene& scene, SecTmReal timeElapsed);

	/** Constructs the PhysUniverse without initialisation */
	PhysUniverse(golem::Context& context);

	/** Releases resources */
	void release();

	/** Creates/initialises the PhysUniverse */
	void create(const Desc& desc);
	
public:
	/** Releases resources */
	virtual ~PhysUniverse();

	/** Creates Scene description */
	virtual Scene::Desc::Ptr createSceneDesc() const;

	inline NxPhysicsSDK* getNxPhysicsSDK() {
		return pNxPhysicsSDK;
	}
	inline const NxPhysicsSDK* getNxPhysicsSDK() const {
		return pNxPhysicsSDK;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYS_UNIVERSE_H_*/
