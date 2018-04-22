/** @file UIController.h
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
#ifndef _GOLEM_APP_UICONTROLLER_H_
#define _GOLEM_APP_UICONTROLLER_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Controller.h>
#include <Golem/Sim/Universe.h>
#include <Golem/App/GraphRenderer.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgUIController, MsgObject)

//------------------------------------------------------------------------------

/** Device body Actor */
class BodyActor : public Actor {
public:
	typedef Chainspace::Coord<BodyActor*> ChainSeq;
	typedef Configspace::Coord<BodyActor*> ConfigSeq;
	
	/** Object description */
	class Desc : public Actor::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(BodyActor, Object::Ptr, Scene&)

	public:
		/** Actor description */
		Actor::Desc::Ptr pActorDesc;
		/** Pointer to the controlled chain */
		Chain* pChain;
		/** Pointer to the controlled joint */
		Joint* pJoint;

		/** Constructs Actor description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the objet parameters to the default values */
		virtual void setToDefault() {
			Actor::Desc::setToDefault();
			
			pActorDesc.reset();
			appearance.invisible = true;
			pChain = NULL;
			pJoint = NULL;
		}

		/** Checks if the object description is valid. */
		virtual bool isValid() const {
			if (!Actor::Desc::isValid())
				return false;
			if (pActorDesc == NULL || !pActorDesc->isValid())
				return false;
			if (pChain == NULL && pJoint == NULL)
				return false;
			return true;
		}
	};

protected:
	/** Actor */
	Actor* pActor;
	/** Pointer to the controlled chain */
	Chain* pChain;
	/** Pointer to the controlled joint */
	Joint* pJoint;

	/** Releases resources */
	virtual void release();
	/** Creates Actor from description. */
	void create(const BodyActor::Desc& desc);
	/** Objects can be constructed only within given Scene. */
	BodyActor(Scene &scene);

public:
	/** Actor */
	const Actor* getActor() const {
		return pActor;
	}

	/** Returns Actor pose */
	virtual Mat34 getPose() const;
	/** Fast Actor pose */
	virtual Mat34 getPoseSync() const;
	/** Sets a new Actor pose */
	virtual void setPose(const Mat34& pose, bool bMove = true);
	/** Sets a new Actor pose */
	virtual void setPoseAsync(const Mat34& pose, bool bMove = true);
	/** Creates the bounds in the local coordinate frame */
	virtual const Bounds* createBounds(Bounds::Desc::Ptr pDesc);
	/** Removes the specified bounds */
	virtual void releaseBounds(const Bounds& bounds);
	/** Returns bounds collection of the Actor */
	const BoundsConstSeq& getBoundsSeq() const {
		return pActor->getBoundsSeq();
	}
	/** Returns collection of bounds in local Actor coordinates	*/
	virtual Bounds::SeqPtr getLocalBoundsSeq(U32 group = Bounds::GROUP_ALL) const;
	/** Returns collection of bounds in global Actor coordinates */
	virtual Bounds::SeqPtr getGlobalBoundsSeq(U32 group = Bounds::GROUP_ALL) const;
	/** Returns bounds description for given bounds	*/
	virtual const Bounds::Desc* getBoundsDesc(const Bounds& bounds) const;
	/** Sets the bounds group for given bounds */
	virtual void setBoundsGroup(const Bounds& bounds, U32 group);
	/** Sets the bounds group for all bounds */
	virtual void setBoundsGroup(U32 group);
	/** Returns shape appearance */
	const Appearance& getAppearance() const {
		return pActor->getAppearance();
	}
	/** Sets shape appearance */
	virtual void setAppearance(const Appearance &appearance);
};

//------------------------------------------------------------------------------

/** Controller visualisation */
class UIControllerVis : public Object {
public:
	/** Object description */
	class Desc {
	public:
		/** Joint appearance */
		Appearance jointAppearance;

		/** Path interval [tmCurrent + pathTimePast, tmCurrent + pathTimeFuture] */
		SecTmReal pathTimePast, pathTimeFuture;
		/** Path maximum number of segments */
		U32 pathSegments;
		/** Path segment duration */
		SecTmReal pathSegmentDuration;
		/** Path renderer */
		GraphRenderer pathRenderer;

		/** State renderer */
		GraphRenderer stateRenderer;
		/** Use commands not the actual state (smooth, but may not be accurate in other than position controlled devices) */
		bool stateUseCommands;
		/** Show joint frames */
		bool stateJointFramesShow;
		/** Joint frames size */
		Vec3 stateJointFramesSize;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Constructs object */
		virtual Object::Ptr create(Scene& scene, Controller& controller) const {
			Object::Ptr object(new UIControllerVis(scene, controller));
			static_cast<UIControllerVis&>(*object).create(*this);
			return object;
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			jointAppearance.solidColour.set(192, 192, 0, 100); // set yellow
			jointAppearance.wireColour.set(127, 127, 127, 255); // set grey

			pathTimePast = SecTmReal(-0.0); // 1 sec history
			pathTimeFuture = SEC_TM_REAL_MAX; // all planned trajectory
			pathSegments = 5000;
			pathSegmentDuration = SecTmReal(0.2);
			pathRenderer.setToDefault();
			pathRenderer.edgeShow = true;
			pathRenderer.show = true;
			
			stateRenderer.setToDefault();
			stateRenderer.vertexFrameShow = true;
			stateRenderer.show = true;
			stateUseCommands = true;
			stateJointFramesShow = false;
			stateJointFramesSize.set(Real(0.02));
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!jointAppearance.isValid())
				return false;
			if (pathTimePast > pathTimeFuture || pathSegments < 2 || pathSegmentDuration <= SecTmReal(0.0))
				return false;
			if (!pathRenderer.isValid() || !stateRenderer.isValid())
				return false;
			return true;
		}
	};

protected:
	/** Controller interface */
	Controller& controller;
	/** the configuration space properties */
	Controller::State::Info stateInfo;
	
	/** Controller bounds group */
	U32 controllerGroup;
	/** Collision bounds group */
	U32 collisionGroup;

	/** Joint appearance */
	Appearance jointAppearance;
	/** Path interval [tmCurrent + pathTimePast, tmCurrent + pathTimeFuture] */
	SecTmReal pathTimePast, pathTimeFuture;
	/** Path maximum number of segments */
	U32 pathSegments;
	/** Path segment duration */
	SecTmReal pathSegmentDuration;
	/** Path renderer */
	GraphRenderer pathRenderer;
	/** State renderer */
	GraphRenderer stateRenderer;
	/** Use commands not the actual state */
	bool stateUseCommands;
	/** Show joint frames */
	bool stateJointFramesShow;
	/** Joint frames size */
	Vec3 stateJointFramesSize;

	BodyActor::ChainSeq chainActors;
	BodyActor::ConfigSeq jointActors;

	Controller::State state;
	Controller::State::Seq commands;
	WorkspaceJointCoord coords;

	/** Finds controller bounds group	*/
	template <typename _Ptr, typename _Seq> U32 findControllerBoundsGroup(_Ptr begin, _Ptr end, const _Seq& seq) const {
		U32 group = 0x0;
	
		for (_Ptr i = begin; i < end; ++i) {
			const BodyActor *pBodyActor = seq[i];
			if (pBodyActor == NULL)
				continue;

			const Bounds::ConstSeq& boundsSeq = pBodyActor->getBoundsSeq();
			for (Bounds::ConstSeq::const_iterator j = boundsSeq.begin(); j != boundsSeq.end(); ++j)
				group |= (*j)->getGroup();
		}

		return group;
	}

	/** Create body Actors. */
	BodyActor* createBodyActor(Chain* pChain, Joint* pJoint);
	/** Release body Actors. */
	void releaseBodyActor(BodyActor** pBodyActor);

	/** Get current state. */
	bool getCoords(SecTmReal t, Controller::State& state, WorkspaceJointCoord& coords) const;

	/** Synchronise body Actors pose with the controller pose. */
	void syncBodyActorsPose(const WorkspaceJointCoord& coords, bool bMove = true);
	
	/** preprocess Renderer data. */
	void preprocessRenderer(SecTmReal t, const Controller::State& state, const WorkspaceJointCoord& coords);
	
	/** Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);
	
	/** (Pre)processing function called BEFORE every simulation step and before randering. */
	virtual void preprocess(SecTmReal elapsedTime);
	
	/** Renders the UIControllerVis. */
	virtual void render() const;

	/** Creates UIControllerVis. */
	void create(const Desc& desc);
	
	/** Releases resources */
	virtual void release();

	/** Objects can be constructed only in the Scene context. */
	UIControllerVis(Scene &scene, Controller& controller);

public:
	/** Returns collection of all scene bounds different than the controller bounds.
	* @return				pointer to the collection of bounds
	*/
	virtual Bounds::SeqPtr getCollisionBounds() const;
	
	/** Returns controller joints' bounds group
	 * @return				bounds group
	*/
	U32 getControllerBoundsGroup() const {
		return controllerGroup;
	}

	/** Returns bounds group of the Actors which can collide with device
	 * @return				bounds group
	*/
	U32 getCollisionBoundsGroup() const {
		return collisionGroup;
	}

	/** Sets bounds group of the Actors which can collide with device
	 * @param	collisionGroup	bounds group
	*/
	void setCollisionBoundsGroup(U32 collisionGroup) {
		this->collisionGroup = this->controllerGroup & collisionGroup;
	}

	/** Access to Actors attached to chains
	 * @return		reference to body Actors
	 */
	const BodyActor::ChainSeq& getChainActors() const {
		return chainActors;
	}
	/** Access to Actors attached to joints
	 * @return		reference to body Actors
	 */
	const BodyActor::ConfigSeq& getJointActors() const {
		return jointActors;
	}

	/** Returns the controller */
	const Controller &getController() const {
		return controller;
	}
	Controller &getController() {
		return controller;
	}
};

//------------------------------------------------------------------------------

class UIController : public Object {
public:
	typedef shared_ptr<UIController> Ptr;

	/** Object description */
	class Desc : public Object::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(UIController, Object::Ptr, Scene&)

	public:
		typedef shared_ptr<Desc> Ptr;

		/** Controller description */
		Controller::Desc::Ptr pControllerDesc;
		/** Appearance */
		UIControllerVis::Desc uiControllerVisDesc;

		/** Constructs UIController description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Object::Desc::setToDefault();
			pControllerDesc.reset();
			uiControllerVisDesc.setToDefault();
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Object::Desc::isValid())
				return false;
			if (pControllerDesc == NULL || !pControllerDesc->isValid() || !uiControllerVisDesc.isValid())
				return false;
			return true;
		}
	};

protected:
	/** Controller interface */
	Controller::Ptr pController;
	/** Controller visualisation */
	UIControllerVis* pUIControllerVis;

	/** Creates UIController. */
	void create(const Desc& desc);

	/** Releases resources */
	virtual void release();

	/** Objects can be constructed only in the Scene context. */
	UIController(Scene &scene);

public:
	/** Returns the controller visualisation */
	const UIControllerVis &getUIControllerVis() const {
		return *pUIControllerVis;
	}
	UIControllerVis &getUIControllerVis() {
		return *pUIControllerVis;
	}

	/** Returns collection of all scene bounds different than the controller bounds.
	* @return				pointer to the collection of bounds
	*/
	Bounds::SeqPtr getCollisionBounds() const {
		return pUIControllerVis->getCollisionBounds();
	}

	/** Returns controller joints' bounds group
	* @return				bounds group
	*/
	U32 getControllerBoundsGroup() const {
		return pUIControllerVis->getControllerBoundsGroup();
	}

	/** Returns bounds group of the Actors which can collide with device
	* @return				bounds group
	*/
	U32 getCollisionBoundsGroup() const {
		return pUIControllerVis->getCollisionBoundsGroup();
	}

	/** Sets bounds group of the Actors which can collide with device
	* @param	collisionGroup	bounds group
	*/
	void setCollisionBoundsGroup(U32 collisionGroup) {
		pUIControllerVis->setCollisionBoundsGroup(collisionGroup);
	}

	/** Access to Actors attached to chains
	* @return		reference to body Actors
	*/
	const BodyActor::ChainSeq& getChainActors() const {
		return pUIControllerVis->getChainActors();
	}
	/** Access to Actors attached to joints
	* @return		reference to body Actors
	*/
	const BodyActor::ConfigSeq& getJointActors() const {
		return pUIControllerVis->getJointActors();
	}

	/** Returns the controller */
	const Controller &getController() const {
		return *pController;
	}
	Controller &getController() {
		return *pController;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_APP_UICONTROLLER_H_*/
