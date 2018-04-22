/** @file Object.h
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
#ifndef _GOLEM_PHYS_OBJECT_H_
#define _GOLEM_PHYS_OBJECT_H_

//------------------------------------------------------------------------------

#include <Golem/Sim/Object.h>
#include <NxPhysics.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgPhysActor, MsgActor)
MESSAGE_DEF(MsgPhysActorBoundsCreate, MsgPhysActor)
MESSAGE_DEF(MsgPhysActorBoundsDescCreate, MsgPhysActor)
MESSAGE_DEF(MsgPhysActorNxActorCreate, MsgPhysActor)
MESSAGE_DEF(MsgPhysActorNxShapeCreate, MsgPhysActor)
MESSAGE_DEF(MsgPhysActorNxShapeDescCreate, MsgPhysActor)

//------------------------------------------------------------------------------

class PhysUniverse;
class PhysScene;

//------------------------------------------------------------------------------

/** PhysActor is a base class of all physically interacting objects/bodies. */
class PhysActor : public Actor {
public:
	friend class Desc;
	
	/** Object description */
	class Desc : virtual public Actor::Desc {
	public:
		/** Default PhysX Actor body description */
		NxBodyDesc nxBodyDesc;
		/** PhysX Actor description */
		mutable NxActorDesc nxActorDesc;

		/** Constructs PhysActor description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the objet parameters to the default values */
		virtual void setToDefault() {
			Actor::Desc::setToDefault();

			nxBodyDesc.setToDefault();
			nxBodyDesc.solverIterationCount = 255;//10;
			nxBodyDesc.sleepEnergyThreshold = NxReal(0.0);

			nxActorDesc.setToDefault();
			nxActorDesc.density = NxReal(1.0);
			nxActorDesc.body = &nxBodyDesc;
		}
		/** Checks if the object description is valid. */
		virtual bool isValid() const {
			if (!Actor::Desc::isValid())
				return false;

			if (!nxBodyDesc.isValid())
				return false;
			if (!nxActorDesc.shapes.empty() && !nxActorDesc.isValid())
				return false;

			return true;
		}
		/** Loads description from xml. */
		virtual void load(const golem::XMLContext* xmlcontext);

	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(PhysActor, Object::Ptr, Scene&)
	};

protected:
	/** Universe is the container of Scenes */
	PhysUniverse *universe;
	/** Scene is the container of objects */
	PhysScene *scene;
	/** Pointer to Novodex PhysActor. */
	NxActor *pNxActor;
	
	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(SecTmReal elapsedTime);

	/** Add bounds */
	virtual BoundsData* addBounds(Bounds::Desc::Ptr pDesc);
	/** Remove bounds */
	virtual BoundsData removeBounds(const Bounds* pBounds);

	/** Creates PhysActor from description. */
	void create(const PhysActor::Desc& desc);

	/** Releases resources */
	virtual void release();
	
	/** Objects can be constructed only within given PhysScene. */
	PhysActor(Scene &scene);

public:
	/** Sets a new Actor pose
	* @param pose			target Actor pose
	* @param bMove			move Actor
	*/
	virtual void setPose(const Mat34& pose, bool bMove = true);

	/** Fast new Actor pose
	* @param pose			target Actor pose
	* @param bMove			move Actor
	*/
	virtual void setPoseSync(const Mat34& pose, bool bMove = true);

	/** Creates the bounds in the local coordinate frame
	 * @param pDesc			description of the bounds
	 * @return				pointer to the bounds; <code>NULL</code> in case of failure
	*/
	virtual const Bounds* createBounds(Bounds::Desc::Ptr pDesc);

	/** Removes the specified bounds
	 * @param bounds		the bounds to be removed
	*/
	virtual void releaseBounds(const Bounds& bounds);
	
	/** Returns Novodex PhysActor 
	 * @return				pointer to the actor
	*/
	inline NxActor *getNxActor() {
		return pNxActor;
	}

	/** Returns Novodex PhysActor 
	 * @return				constant pointer to the actor
	*/
	inline const NxActor *getNxActor() const {
		return pNxActor;
	}

	inline PhysScene& getScene() {
		return *scene;
	}
	inline const PhysScene& getScene() const {
		return *scene;
	}

	inline PhysUniverse& getUniverse() {
		return *universe;
	}
	inline const PhysUniverse& getUniverse() const {
		return *universe;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYS_OBJECT_H_*/
