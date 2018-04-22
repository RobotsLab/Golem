/** @file Scene.h
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
#ifndef _GOLEM_SIM_SCENE_H_
#define _GOLEM_SIM_SCENE_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Bounds.h>
#include <Golem/Math/Collection.h>
#include <Golem/Sys/Message.h>
#include <Golem/Sim/Object.h>
#include <vector>
#include <map>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgScene, Message)
MESSAGE_DEF(MsgSceneInvalidDesc, MsgScene)
MESSAGE_DEF(MsgSceneObjectCreate, MsgScene)
MESSAGE_DEF(MsgSceneObjectRelease, MsgScene)
MESSAGE_DEF(MsgSceneBoundsDescInvalidDesc, MsgScene)
MESSAGE_DEF(MsgSceneBoundsDescCreate, MsgScene)

//------------------------------------------------------------------------------

class Universe;

/** Golem Scene */
class Scene : protected UIKeyboardMouse, public UIRendererCallback {
	friend class Universe;

public:
	typedef shared_ptr<Scene> Ptr;
	typedef std::vector<Scene*> Seq;	
	typedef PrivateList<Object*, Object::Ptr> ObjectList;

	/** Help strings */
	typedef std::map<std::string, std::string> StrMap;
	typedef StrMap::value_type StrMapVal;

	/** Scene description */
	class Desc {
	public:
		friend class Universe;
		typedef shared_ptr<Desc> Ptr;
		typedef std::vector<Ptr> Seq;

		/** Scene name */
		std::string name;
		/** OpenGL settings */
		OpenGL::Seq openGLSeq;
		/** Asynchronous scene - no background processing: pre/post process, rendering and physx */
		bool asyncScene;
		/** view mouse button */
		int mouseButton;

		/** Help message */
		StrMap help;

		/** Constructs Scene description object */
		Desc() {
			setToDefault();
		}
		/** Destructor should be virtual */
		virtual ~Desc() {
		}

		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			name = "Scene";

			openGLSeq.clear();
			openGLSeq.push_back(OpenGL());
			asyncScene = false;
			mouseButton = 0;

			help.clear();
			help.insert(StrMapVal("0000", "Program help:\n"));
			help.insert(StrMapVal("0101", "  ?                                       scene help message\n"));
			help.insert(StrMapVal("0102", "  <pgup><pgdn>                            scene index\n"));
			help.insert(StrMapVal("0103", "  !                                       scene drawing mode\n"));
			help.insert(StrMapVal("0104", "  @                                       scene physics mode\n"));
			help.insert(StrMapVal("0105", "  <ctrl+p>                                scene rendering pause\n"));
			help.insert(StrMapVal("0201", "  <>                                      view point index\n"));
			help.insert(StrMapVal("0202", "  <arrows>                                view point translation\n"));
			help.insert(StrMapVal("0203", "  <mouse_left_button+motion>              view point rotation\n"));
			help.insert(StrMapVal("0204", "  <ctrl+v>                                view point info\n"));
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (openGLSeq.empty())
				return false;
			for (OpenGL::Seq::const_iterator i = openGLSeq.begin(); i != openGLSeq.end(); ++i)
				if (!i->isValid())
					return false;		
			
			return true;
		}
		/** Loads description from xml. */
		virtual void load(const golem::XMLContext* xmlcontext);

	protected:
		/** Creates/initialises the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(Scene, Scene::Ptr, Universe&)
	};

protected:
	/** Scene name */
	std::string name;
	/** OpenGL settings */
	OpenGL::Seq openGLSeq;
	/** Asynchronous scene - no background processing: pre/post process, rendering and physx */
	bool asyncScene;
	/** view mouse button */
	int viewMouseButton;

	/** Help message */
	StrMap help;

	Universe &universe;
	golem::Context &context;
	
	/** Collection owns pointers to all objects */
	ObjectList objectList;

	/** OpenGL settings */
	OpenGL::Seq::iterator openGLPtr;
	OpenGL openGL;
	bool glMat;
	U32 frameMode;
	
	int mouseButton;
	int mouseX;
	int mouseY;

	DebugRenderer debugRenderer;

	/** golem::UIKeyboardMouse: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	
	/** golem::UIKeyboardMouse: Mouse motion handler. */
	virtual void motionHandler(int x, int y);

	/** golem::UIKeyboardMouse: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);

	/** (Pre)processing function called BEFORE every physics simulation step and before randering. */
	virtual void preprocess(SecTmReal elapsedTime);

	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(SecTmReal elapsedTime);
	
	/** Renders the scene. */
	virtual void render() const;
	
	/** Init OpenGL */
	virtual void initOpenGL();

	/** Init OpenGL camera. */
	virtual void initCamera() const;
	
	/** Creates the Scene from the Scene description. */
	void create(const Desc& desc);
	
	/** Releases resources */
	virtual void release();
	
	/** Scenes can be constructed only in the Universe context. */
	Scene(Universe &universe);

public:
	/** Destructor is inaccesible */
	virtual ~Scene();

	/** Creates Actor description */
	virtual Actor::Desc::Ptr createActorDesc() const;

	/** Creates Object from the description. */
	virtual Object* createObject(const Object::Desc& desc);

	/** Inserts Object to the container. */
	virtual Object* insertObject(const Object::Ptr& pObject);

	/** Releases the Object. */
	virtual void releaseObject(Object& object);

	/** Returns collection of Objects */
	virtual const ObjectList& getObjectList() const;

	U32 getDraw() const;
	
	void setDraw(U32 draw);

	/** Set current OpenGL settings */
	void setOpenGL(const OpenGL& openGL);

	/** Get OpenGL settings */
	const OpenGL::Seq& getOpenGL() const {
		return openGLSeq;
	}
	
	/** UIRendererCallback: Get current OpenGL settings */
	virtual void getOpenGL(OpenGL& openGL) const;

	/** UIRendererCallback: Synchronisation with rendering thread */
	virtual CriticalSection &getCS() const;

	/** GL matrix camera mode */
	bool isGLMat() const {
		return glMat;
	}
	/** GL matrix camera mode */
	void setGLMat(bool enable);

	/** Asynchronous scene - no background processing: pre/post process, rendering and physx */
	bool isAsync() const {
		return asyncScene;
	}
	
	/** Help messages */
	const StrMap& getHelp() const {
		return help;
	}
	/** Help messages */
	StrMap& getHelp() {
		return help;
	}
	/** Help message print */
	virtual void printHelp() const;

	inline golem::Context& getContext() {
		return context;
	}
	inline const golem::Context& getContext() const {
		return context;
	}

	inline Universe& getUniverse() {
		return universe;
	}
	inline const Universe& getUniverse() const {
		return universe;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_SIM_SCENE_H_*/
