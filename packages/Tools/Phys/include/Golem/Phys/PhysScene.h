/** @file PhysScene.h
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
#ifndef _GOLEM_PHYS_SCENE_H_
#define _GOLEM_PHYS_SCENE_H_

//------------------------------------------------------------------------------

#include <Golem/Sim/Scene.h>
#include <Golem/Phys/PhysObject.h>
#include <Golem/Phys/NxDebugRenderer.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgPhysScene, MsgScene)
MESSAGE_DEF(MsgPhysSceneNxSceneCreate, MsgPhysScene)
MESSAGE_DEF(MsgPhysSceneNxShapeDescInvalidDesc, MsgPhysScene)
MESSAGE_DEF(MsgPhysSceneNxShapeDescCreate, MsgPhysScene)
MESSAGE_DEF(MsgPhysSceneNxShapeDescRelease, MsgPhysScene)

//------------------------------------------------------------------------------

class PhysUniverse;

class PhysScene : public Scene {
public:
	friend class PhysUniverse;

	/** Physics */
	class Physics {
	public:
		/** Novodex PhysX scene description */
		NxSceneDesc nxSceneDesc;

		/** Default restitution coefficient */
		Real restitution;
		/** Default static friction coefficient */
		Real staticFriction;
		/** Default dynamic friction coefficient */
		Real dynamicFriction;

		/** Construction */
		Physics() {
			setToDefault();
		}

		/** Sets to the default state */
		void setToDefault() {
			nxSceneDesc.setToDefault();
			nxSceneDesc.gravity.set(0.0f, 0.0f, -9.81f); // Z-axis

			restitution = (Real)0.1;
			staticFriction = (Real)0.2;
			dynamicFriction = (Real)0.1;
		}
		/** Checks if it is valid. */
		bool isValid() const {
			if (!nxSceneDesc.isValid())
				return false;
			if (restitution < REAL_ZERO || staticFriction < REAL_ZERO || dynamicFriction < REAL_ZERO)
				return false;
			return true;
		}
	};

	class Desc : public Scene::Desc {
	public:
		friend class PhysUniverse;

		/** Physics */
		Physics physics;

		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			Scene::Desc::setToDefault();

			physics.setToDefault();
		}

		virtual bool isValid() const {
			if (!Scene::Desc::isValid())
				return false;

			if (!physics.isValid())
				return false;

			return true;
		}
		/** Loads description from xml. */
		virtual void load(const golem::XMLContext* xmlcontext);

	protected:
		/** Creates/initialises the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(PhysScene, Scene::Ptr, Universe&)
	};

protected:
	typedef shared_ptr<NxShapeDesc> NxShapeDescPtr;
	typedef std::map<const Bounds::Desc*, NxShapeDescPtr> NxShapeDescMap;
	typedef std::map<const NxShapeDesc*, Bounds::Desc::Ptr> BoundsDescMap;

	/** Physics */
	Physics physics;

	PhysUniverse *universe;
	NxPhysicsSDK* pNxPhysicsSDK;
	NxScene* pNxScene;
	
	/** Collection owns pointers to all shape and bounds descriptions */
	NxShapeDescMap nxShapeDescMap;	
	BoundsDescMap boundsDescMap;
	
	::NxDebugRenderer nxDebugRenderer;

	NxShapeDescPtr createNxShapeDesc(const BoundingPlane::Desc* pDesc) const;
	Bounds::Desc::Ptr createBoundsDesc(const NxPlaneShapeDesc *pDesc) const;
	
	NxShapeDescPtr createNxShapeDesc(const BoundingSphere::Desc* pDesc) const;
	Bounds::Desc::Ptr createBoundsDesc(const NxSphereShapeDesc *pDesc) const;
	
	NxShapeDescPtr createNxShapeDesc(const BoundingBox::Desc* pDesc) const;
	Bounds::Desc::Ptr createBoundsDesc(const NxBoxShapeDesc *pDesc) const;
	
	NxShapeDescPtr createNxShapeDesc(const BoundingCylinder::Desc* pDesc);
	
	void copyShapeDesc(const ::NxConvexMeshDesc& src, BoundingConvexMesh::Desc& dst) const;
	NxShapeDescPtr createNxShapeDesc(BoundingConvexMesh::Desc* pDesc);
	Bounds::Desc::Ptr createBoundsDesc(const NxConvexShapeDesc *pDesc);
	
	void releaseNxShapeDescResources(NxShapeDescPtr& nxShapeDescPtr);

	/** Init OpenGL */
	virtual void initOpenGL();

	/** Renders the scene. */
	virtual void render() const;

	/** Creates the PhysScene from the PhysScene description. */
	void create(const Desc& desc);
	
	/** Releases resources */
	virtual void release();
	
	/** Scenes can be constructed only in the PhysUniverse context. */
	PhysScene(Universe &universe);

public:
	/** Creates Actor description */
	virtual Actor::Desc::Ptr createActorDesc() const;

	/** Creates Novodex shape description from bounds description. */
	virtual NxShapeDesc* createNxShapeDesc(Bounds::Desc::Ptr pBoundsDesc);
	
	/** Releases the Novodex shape description and related bounds description. */
	virtual void releaseNxShapeDesc(NxShapeDesc &nxShapeDesc);
	
	/** Creates bounds description from Novodex shape description. */
	virtual Bounds::Desc::Ptr createBoundsDesc(const NxShapeDesc &nxShapeDesc);

	/** Releases the bounds description and related Novodex shape description. */
	virtual void releaseBoundsDesc(Bounds::Desc::Ptr pBoundsDesc);
	
	inline PhysUniverse& getUniverse() {
		return *universe;
	}
	inline const PhysUniverse& getUniverse() const {
		return *universe;
	}

	inline NxScene* getNxScene() {
		return pNxScene;
	}
	inline const NxScene* getNxScene() const {
		return pNxScene;
	}

	inline NxPhysicsSDK* getNxPhysicsSDK() {
		return pNxPhysicsSDK;
	}
	inline const NxPhysicsSDK* getNxPhysicsSDK() const {
		return pNxPhysicsSDK;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYS_SCENE_H_*/
