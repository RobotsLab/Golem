/** @file Universe.h
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
#ifndef _GOLEM_SIM_UNIVERSE_H_
#define _GOLEM_SIM_UNIVERSE_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/System.h>
#include <Golem/Sys/Message.h>
#include <Golem/Sim/Scene.h>
#include <vector>
#include <map>
#include <functional>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class XMLContext;

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgUniverse, Message)
MESSAGE_DEF(MsgUniverseMultipleInstances, MsgUniverse)
MESSAGE_DEF(MsgUniverseInvalidDesc, MsgUniverse)
MESSAGE_DEF(MsgUniverseNoScenes, MsgUniverse)
MESSAGE_DEF(MsgUniverseThreadLaunch, MsgUniverse)
MESSAGE_DEF(MsgUniverseThreadNotAlive, MsgUniverse)
MESSAGE_DEF(MsgUniverseGlutInit, MsgUniverse)
MESSAGE_DEF(MsgUniverseSceneCreate, MsgUniverse)
MESSAGE_DEF(MsgUniverseSceneRelease, MsgUniverse)

//------------------------------------------------------------------------------

/** Golem Universe */
class Universe : public UIKeyboardMouseCallback, protected Runnable {
	friend class Scene;
	
public:
	typedef shared_ptr<Universe> Ptr;
	typedef PrivateList<Scene*, Scene::Ptr> SceneList;

	typedef std::function<void(void)> HandlerTerminate;

	/** Universe description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;

		/** Program name */
		std::string name;

		/** Number of program arguments */
		int argc;
		/** Program arguments */
		char** argv;
		
		/** GUI window position X */
		int windowX;
		/** GUI window position Y */
		int windowY;
		/** GUI window width */
		int windowWidth;
		/** GUI window height */
		int windowHeight;

		/** GUI window position X offset */
		unsigned windowXOffs;
		/** GUI window position Y offset */
		unsigned windowYOffs;

		/** Working thread priority */
		Thread::Priority threadPriority;
		/** Inter-thread signalling time out */
		MSecTmU32 threadTimeOut;
		
		/** Simulation real time */
		bool simulationRealTime;
		/** Simulation FPS */
		SecTmReal simulationFPS;
		/** Simulation Scale */
		Real simulationScale;

		/** Constructs Universe description. */
		Desc() {
			setToDefault();
		}
		/** Destructor should be virtual */
		virtual ~Desc() {
		}
		
		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			argc = 0;
			argv = NULL;
			
			name = "Golem";
			
			windowX = 0;
			windowY = 0;
			windowWidth = 1200;
			windowHeight = 800;
			windowXOffs = 8;
			windowYOffs = 8;

			threadPriority = Thread::ABOVE_NORMAL;
			threadTimeOut = 30000; //[msec]
			
			simulationRealTime = false;	
			simulationFPS = SecTmReal(30.0); // avr. frames/sec
			simulationScale = Real(10.0);
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			//if (argc <= 0 || argv == NULL)
			//	return false;
			if (windowWidth <= 0 || windowHeight <= 0)
				return false;
			if (simulationFPS <= SEC_TM_REAL_ZERO)
				return false;
			if (simulationScale <= REAL_ZERO)
				return false;

			return true;
		}
		/** Loads description from xml. */
		virtual void load(const golem::XMLContext* xmlcontext);

		/** Creates/initialises the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(Universe, Universe::Ptr, Context&)
	};

protected:
	/** Program name */
	std::string name;

	/** Number of program arguments */
	int argc;
	/** Program arguments */
	char** argv;

	/** GUI window position X */
	int windowX;
	/** GUI window position Y */
	int windowY;
	/** GUI window width */
	int windowWidth;
	/** GUI window height */
	int windowHeight;
	/** GUI window position X offset */
	unsigned windowXOffs;
	/** GUI window position Y offset */
	unsigned windowYOffs;

	/** Working thread priority */
	Thread::Priority threadPriority;
	/** Inter-thread signalling time out */
	MSecTmU32 threadTimeOut;

	/** Simulation real time */
	bool simulationRealTime;
	/** Simulation FPS */
	SecTmReal simulationFPS;
	/** Simulation Scale */
	Real simulationScale;

	golem::Context &context;
	
	Thread thread;
	mutable CriticalSection cs;
	volatile bool bPause, bTerminate;
	Event evLoop, evWaitKey;

	int key;
	bool keyInterrupted;

	int glWindowHandle;
	GLdouble glAspectRatio;

	golem::UICapture* uiCapture;

	bool bTitle;
	std::string sTitle; 

	bool bSize;

	/** Collection owns pointers to all scenes */
	SceneList sceneList;
	SceneList::const_iterator currentScene;

	SecTmReal maxTimeStep;
	SecTmReal simulationTimeStamp;
	SecTmReal renderTimeStamp;

	/** Universe terminate handler */
	HandlerTerminate handlerTerminate;

	/** Main loop */
	virtual void run();
	
	/** Scene initialisation */
	virtual void setupCurrentScene();
	/** Glut initialisation */
	virtual void initGlut();

	/** Sets window title */
	void setTitleSync(const char* title);

	/** Glut callback */
	static void renderCallback();
	/** Glut callback */
	static void reshapeCallback(int width, int height);
	/** Glut callback */
	static void idleCallback();
	/** Glut callback */
	static void mouseCallback(int button, int state, int x, int y);
	/** Glut callback */
	static void motionCallback(int x, int y);
	/** Glut callback */
	static void keyboardCallback(unsigned char key, int x, int y);
	/** Glut callback */
	static void arrowKeyCallback(int key, int x, int y);

	/** Glut keyboard special keys mask */
	static int keyboardMask();

	/** Singal handler */
	static void signalHandler(int signal);

	/** Glut keyboard callback */
	virtual void keyboardHandler(int key, int x, int y);
	
	/** Process */
	virtual void process();
	/** Process scene */
	virtual void process(Scene& scene, SecTmReal timeElapsed);

	/** Creates/initialises the Universe */
	void create(const Desc& desc);

	/** Releases resources */
	void release();

	/** Constructs the Universe without initialisation */
	Universe(golem::Context& context);

public:
	/** Release resources */
	virtual ~Universe();

	/** Launches the Universe */
	virtual bool launch();

	/** Terminates the Universe */
	virtual void exit();
	
	/** Pauses/suspends the Universe */
	virtual void pause();

	/** Resumes the Universe */
	virtual void resume();

	/** Checks if the Universe is alive */
	virtual bool interrupted();

	/** Checks if the Universe is stopped/suspended */
	virtual bool suspended();

	/** golem::UIKeyboardMouseCallback: Read a key, wait no longer than timeOut */
	virtual int waitKey(MSecTmU32 timeOut = MSEC_TM_U32_INF);
	
	/** Sets window title */
	virtual void setTitle(const char* title);
	
	/** Sets window size */
	virtual void setSize(int width, int height);
	
	/** Creates Scene description */
	virtual Scene::Desc::Ptr createSceneDesc() const;

	/** Creates Scene from the description. */
	virtual Scene* createScene(const Scene::Desc& desc);
	
	/** Inserts Scene to the container. */
	virtual Scene* insertScene(const Scene::Ptr& pScene);

	/** Releases the Scene. */
	virtual void releaseScene(Scene& scene);

	/** Returns collection of Scenes */
	virtual SceneList getSceneList() const;
	
	/** Returns render frame rate (FPS) */
	virtual SecTmReal getSimulationFPS() const;
	
	/** Returns window width */
	virtual int getWindowWidth() const;
	
	/** Returns window height */
	virtual int getWindowHeight() const;
	
	/** golem::UICapture: OpenGL capture */
	virtual void setCapture(golem::UICapture* uiCapture = NULL);

	/** golem::UICapture: OpenGL capture */
	virtual golem::UICapture* getCapture();

	/** golem::UICapture: OpenGL capture */
	virtual const golem::UICapture* getCapture() const;

	/** Universe terminate handler */
	virtual void setHandlerTerminate(HandlerTerminate& handlerTerminate);

	/** Synchronisation with rendering thread */
	inline CriticalSection &getCS() const {
		return cs;
	}

	inline golem::Context& getContext() {
		return context;
	}
	
	inline const golem::Context& getContext() const {
		return context;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_SIM_UNIVERSE_H_*/
