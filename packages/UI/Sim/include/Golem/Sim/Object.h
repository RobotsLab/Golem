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
#ifndef _GOLEM_SIM_OBJECT_H_
#define _GOLEM_SIM_OBJECT_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Bounds.h>
#include <Golem/Sys/Context.h>
#include <Golem/Sys/Message.h>
#include <Golem/Plugin/Renderer.h>
#include <vector>
#include <map>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgObject, Message)
MESSAGE_DEF(MsgObjectInvalidDesc, MsgObject)
MESSAGE_DEF(MsgActor, MsgObject)
MESSAGE_DEF(MsgActorBounds, MsgActor)

//------------------------------------------------------------------------------

class Universe;
class Scene;
class XMLContext;

//------------------------------------------------------------------------------

/** Object is a base class of all objects in the simulated world. */
class Object : protected UIRenderer, protected UIKeyboardMouse {
public:
	friend class Scene;
	friend class Desc;
	typedef shared_ptr<Object> Ptr;
	typedef std::vector<Object*> Seq;
	
	/** Object description */
	class Desc {
	public:
		friend class Scene;
		typedef shared_ptr<Desc> Ptr;
		typedef std::vector<Ptr> Seq;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}
		/** Virtual descructor */
		virtual ~Desc() {
		}

		/** Sets the objet parameters to the default values */
		virtual void setToDefault() {
		}
		/** Checks if the object description is valid. */
		virtual bool isValid() const {
			return true;
		}
		/** Loads description from xml. */
		virtual void load(const golem::XMLContext* xmlcontext);

	protected:
		/** Creates/initialises the object. */
		virtual Object::Ptr create(Scene &scene) const = 0;
	};

protected:
	/** Universe is the container of Scenes */
	Universe &universe;
	/** Scene is the container of objects */
	Scene &scene;
	/** golem::Context object */
	golem::Context &context;
	
	/** golem::UIKeyboardMouse: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y) {}
	
	/** golem::UIKeyboardMouse: Mouse motion handler. */
	virtual void motionHandler(int x, int y) {}
	
	/** golem::UIKeyboardMouse: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y) {}

	/** (Pre)processing function called BEFORE every physics simulation step and before randering. */
	virtual inline void preprocess(SecTmReal elapsedTime) {}

	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual inline void postprocess(SecTmReal elapsedTime) {}

	/** golem::UIRenderer: Custom renderer. */
	virtual void customRender() const {}

	/** golem::UIRenderer: Renders the object. */
	virtual void render() const {}

	/** Creates object from description. */
	void create(const Object::Desc& desc);

	/** Releases resources */
	virtual void release();
	
	/** Objects can be constructed only in the Scene context. */
	Object(Scene &scene);

public:
	/** Destructor is inaccesible */
	virtual ~Object();

	inline golem::Context& getContext() {
		return context;
	}
	
	inline const golem::Context& getContext() const {
		return context;
	}

	inline Scene& getScene() {
		return scene;
	}
	
	inline const Scene& getScene() const {
		return scene;
	}
};

//------------------------------------------------------------------------------

/** Actor is a base class of all physically interacting objects/bodies. */
class Actor : public Object {
public:
	friend class Desc;
	typedef shared_ptr<Object> Ptr;
	typedef std::vector<Actor*> Seq;
	typedef Bounds::ConstSeq BoundsConstSeq;	

	/** Object description */
	class Desc : virtual public Object::Desc {
	public:
		friend class Scene;
		typedef shared_ptr<Desc> Ptr;

		/** Actor pose */
		Mat34 pose;
		/** Actor bounds */
		Bounds::Desc::Seq boundsDescSeq;
		/** Shape appearance */
		Appearance appearance;
		/** Kinematic object */
		bool kinematic;

		/** Constructs Actor description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the objet parameters to the default values */
		virtual void setToDefault() {
			Object::Desc::setToDefault();

			pose.setId();
			boundsDescSeq.clear();
			appearance.setToDefault();
			kinematic = false;
		}
		/** Checks if the object description is valid. */
		virtual bool isValid() const {
			if (!Object::Desc::isValid())
				return false;

			if (!pose.isValid())
				return false;

			//if (boundsDescSeq.empty())
			//	return false;
			for (Bounds::Desc::Seq::const_iterator i = boundsDescSeq.begin(); i != boundsDescSeq.end(); ++i)
				if (!i->get() || !i->get()->isValid())
					return false;

			if (!appearance.isValid())
				return false;

			return true;
		}
		/** Loads description from xml. */
		virtual void load(const golem::XMLContext* xmlcontext);

	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(Actor, Object::Ptr, Scene&)
	};

protected:
	/** Bounds collection */
	struct BoundsData {
		typedef std::map<const Bounds*, BoundsData> Map;

		Bounds::Desc::Ptr pBoundsDesc;
		Bounds::Ptr pBounds;

		void* pShapeDesc;
		void* pShape;

		BoundsData();
	};
	
	/** Bounds renderer */
	mutable BoundsRenderer boundsRenderer;
	/** Shape appearance */
	Appearance appearance;
	
	/** Bounds collection */
	BoundsData::Map boundsDataMap;
	/** Bounds pointer collection */
	BoundsConstSeq boundsConstSeq;

	/** Actor pose */
	Mat34 pose;
	/** Kinematic object */
	bool kinematic;

	/** Add bounds */
	virtual BoundsData* addBounds(Bounds::Desc::Ptr pDesc);
	/** Remove bounds */
	virtual BoundsData removeBounds(const Bounds* pBounds);

	/** Renders the object. */
	virtual void render() const;

	/** Creates Actor from description. */
	void create(const Actor::Desc& desc);

	/** Releases resources */
	virtual void release();
	
	/** Objects can be constructed only within given Scene. */
	Actor(Scene &scene);

public:
	/** Destructor is inaccesible */
	virtual ~Actor();

	/** Returns Actor pose
	 * @return				current Actor pose
	*/
	virtual Mat34 getPose() const;
	
	/** Fast Actor pose
	* @return				current Actor pose
	*/
	virtual Mat34 getPoseSync() const;

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
	
	/** Returns bounds collection of the Actor
	 * @param group			bounds group
	 * @return				bounds collection
	*/
	const BoundsConstSeq& getBoundsSeq() const {
		return boundsConstSeq;
	}
	
	/** Returns collection of bounds in local Actor coordinates
	 * @param group			bounds group
	 * @return				bounds collection
	*/
	virtual Bounds::SeqPtr getLocalBoundsSeq(U32 group = Bounds::GROUP_ALL) const;
	
	/** Returns collection of bounds in global Actor coordinates
	 * @param group			bounds group
	 * @return				bounds collection
	*/
	virtual Bounds::SeqPtr getGlobalBoundsSeq(U32 group = Bounds::GROUP_ALL) const;
	
	/** Returns bounds description for given bounds
	* @param bounds		bounds
	* @return				bounds description
	*/
	virtual const Bounds::Desc* getBoundsDesc(const Bounds& bounds) const;

	/** Sets the bounds group for given bounds
	 * @param bounds		bounds to be set
	 * @param group			bounds group
	*/
	virtual void setBoundsGroup(const Bounds& bounds, U32 group);

	/** Sets the bounds group for all bounds
	 * @param group			bounds group
	*/
	virtual void setBoundsGroup(U32 group);

	/** Returns shape appearance */
	const Appearance& getAppearance() const {
		return appearance;
	}

	/** Sets shape appearance */
	virtual void setAppearance(const Appearance &appearance);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_SIM_OBJECT_H_*/
