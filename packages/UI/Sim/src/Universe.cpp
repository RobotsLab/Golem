/** @file Universe.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sim/Universe.h>
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Plugin/Data.h>
#include <GL/freeglut.h>
#include <signal.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void Universe::Desc::load(const golem::XMLContext* xmlcontext) {
	XMLData("name", name, const_cast<golem::XMLContext*>(xmlcontext), false);

	XMLData("x", windowX, xmlcontext->getContextFirst("window"), false);
	XMLData("y", windowY, xmlcontext->getContextFirst("window"), false);
	XMLData("width", windowWidth, xmlcontext->getContextFirst("window"), false);
	XMLData("height", windowHeight, xmlcontext->getContextFirst("window"), false);

	try {
		XMLData("x_offs", windowXOffs, xmlcontext->getContextFirst("window"), false);
		XMLData("y_offs", windowYOffs, xmlcontext->getContextFirst("window"), false);
	}
	catch (const golem::MsgXMLParserAttributeNotFound&) {
	}
	XMLData("real_time", simulationRealTime, xmlcontext->getContextFirst("simulation"), false);
	XMLData("fps", simulationFPS, xmlcontext->getContextFirst("simulation"), false);
}

//------------------------------------------------------------------------------

Universe* pUniverse = NULL;

Universe::Universe(golem::Context& context) : context(context) {
	uiCapture = NULL;
	bPause = false;
	bTerminate = false;
	bTitle = false;
	bSize = false;
	key = 0;
	keyInterrupted = false;
	handlerTerminate = nullptr;
	currentScene = sceneList.end();
}

Universe::~Universe() {
	Universe::release();
}

void Universe::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgUniverseInvalidDesc(Message::LEVEL_CRIT, "Universe::create(): Invalid description");
	
	name = desc.name;
	argc = desc.argc;
	argv = desc.argv;
	windowX = desc.windowX;
	windowY = desc.windowY;
	windowWidth = desc.windowWidth;
	windowHeight = desc.windowHeight;
	windowXOffs = desc.windowXOffs;
	windowYOffs = desc.windowYOffs;
	threadPriority = desc.threadPriority;
	threadTimeOut = desc.threadTimeOut;
	simulationRealTime = desc.simulationRealTime;
	simulationFPS = desc.simulationFPS;
	simulationScale = desc.simulationScale;

	glAspectRatio = 1.0;

	simulationTimeStamp = renderTimeStamp = context.getTimer().elapsed();
	maxTimeStep = SecTmReal(1.0)/(SecTmReal(2.0)*simulationFPS);
}

void Universe::release() {
	exit();
	while (!sceneList.empty())
		releaseScene(*sceneList.back());
}

//------------------------------------------------------------------------------

bool Universe::launch() {
	if (pUniverse != NULL)
		throw MsgUniverseMultipleInstances(Message::LEVEL_CRIT, "Universe::launch(): Universe has been already created");
	pUniverse = this;

	if (thread.isAlive()) {
		context.warning("Universe::launch(): Universe has been launched already\n");
		return true;
	}

	currentScene = sceneList.begin();
	if (currentScene == sceneList.end())
		throw MsgUniverseNoScenes(Message::LEVEL_CRIT, "Universe::launch(): no Scenes have been created");

	if (!thread.start(this))
		throw MsgUniverseThreadLaunch(Message::LEVEL_CRIT, "Universe::launch(): Unable to launch Universe thread");
	
	if (!thread.setPriority(threadPriority))
		context.warning("Universe::launch(): Unable to change Universe thread priority\n");
	
	if (!evLoop.wait(threadTimeOut))
		throw MsgUniverseThreadNotAlive(Message::LEVEL_CRIT, "Universe::launch(): Universe thread is not alive");
	
	return true;
}

void Universe::run() {
	initGlut();

	// register ctrl+c handler
	if (signal(SIGINT, signalHandler) == SIG_ERR)
		context.warning("Universe::run(): unable to register SIGINT handler\n");

	evLoop.set(true);
	
	try {
		// setup
		setupCurrentScene();
		// start the main loop
		::glutMainLoop();
	}
	catch (const std::exception& ex) {
		context.write("%s\n", ex.what());
	}

	evLoop.set(false);
	evWaitKey.set(true);
	
	context.info("Universe::run(): Closing Universe....\n");

	CriticalSectionWrapper csw(cs);
	if (handlerTerminate)
		handlerTerminate();
}

void Universe::exit() {
	bTerminate = true;
	if (!thread.join(threadTimeOut))
		context.error("Universe::exit(): Unable to stop Universe thread\n");
}

void Universe::pause() {
	bPause = true;
}

void Universe::resume() {
	bPause = false;
}

bool Universe::interrupted() {
	return !evLoop.wait(0) || keyInterrupted;
}

bool Universe::suspended() {
	return bPause;
}

int Universe::waitKey(MSecTmU32 timeOut) {
	//if (interrupted())
	//	return 0;

	//evWaitKey.set(false);
	//return evWaitKey.wait(timeOut) ? key : 0;
	
	if (evWaitKey.wait(timeOut))
		evWaitKey.set(false);
	int key = this->key;
	this->key = 0;
	return interrupted() ? 0 : key;
}

//------------------------------------------------------------------------------

void Universe::initGlut() {
	::glutInit(&pUniverse->argc, pUniverse->argv);
	::glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	::glutInitWindowPosition(pUniverse->windowX, pUniverse->windowY);
	::glutInitWindowSize(pUniverse->windowWidth + 2 * pUniverse->windowXOffs, pUniverse->windowHeight + 2 * pUniverse->windowYOffs);
	::glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glWindowHandle = ::glutCreateWindow(pUniverse->name.c_str());
	::glutSetWindow(glWindowHandle);
	::glutDisplayFunc(renderCallback);
	::glutReshapeFunc(reshapeCallback);
	::glutIdleFunc(idleCallback);
	::glutKeyboardFunc(keyboardCallback);
	::glutSpecialFunc(arrowKeyCallback);
	::glutMouseFunc(mouseCallback);
	::glutMotionFunc(motionCallback);
	reshapeCallback(::glutGet(GLUT_WINDOW_WIDTH), ::glutGet(GLUT_WINDOW_HEIGHT));
}

void Universe::setupCurrentScene() {
	if (currentScene == sceneList.end())
		return;

	setTitleSync((*currentScene)->name.c_str());
	
	(*currentScene)->initOpenGL();
}

void Universe::setTitleSync(const char* title) {
	this->sTitle = name;
	if (title != NULL) {
		this->sTitle.append(" - ");
		this->sTitle.append(title);
	}
	bTitle = true;
}

//------------------------------------------------------------------------------

void Universe::renderCallback() {
	if (pUniverse->bPause)
		return;

	{
		// rendering and processing thread
		CriticalSectionWrapper csw(pUniverse->cs);
		
		if (pUniverse->currentScene == pUniverse->sceneList.end())
			return;
		
		if (pUniverse->bTitle) {
			pUniverse->bTitle = false;
			::glutSetWindowTitle(pUniverse->sTitle.c_str());
		}
		if (pUniverse->bSize) {
			pUniverse->bSize = false;
			::glutReshapeWindow(pUniverse->windowWidth + 2 * pUniverse->windowXOffs, pUniverse->windowHeight + 2 * pUniverse->windowYOffs);
		}
			
		// Clear buffers
		::glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		::glViewport(0, 0, pUniverse->windowWidth - pUniverse->windowXOffs, pUniverse->windowHeight - pUniverse->windowYOffs);

		if (!(*pUniverse->currentScene)->isAsync())
			(*pUniverse->currentScene)->render();

		if (pUniverse->uiCapture)
			pUniverse->uiCapture->capture(0, 0, pUniverse->windowWidth - pUniverse->windowXOffs, pUniverse->windowHeight - pUniverse->windowYOffs);
	}

	::glutSwapBuffers();
}

void Universe::reshapeCallback(int width, int height) {
	// rendering and processing thread
	CriticalSectionWrapper csw(pUniverse->cs);
	
	pUniverse->windowWidth = width;
	pUniverse->windowHeight = height;
	pUniverse->glAspectRatio = GLdouble(width)/GLdouble(height);
}

void Universe::idleCallback() {
	const SecTmReal timeCurrent = pUniverse->context.getTimer().elapsed();
	const SecTmReal delay = std::max(SecTmReal(0.0), SecTmReal(1.0)/pUniverse->simulationFPS - (timeCurrent - pUniverse->renderTimeStamp));
	Sleep::msleep(SecToMSec(delay));
	pUniverse->renderTimeStamp = pUniverse->context.getTimer().elapsed();

	// TODO ughhh: use better stuff than freeglut
	if (pUniverse->bTerminate) {
		::glutLeaveMainLoop();
		return;
	}
		
	if (!pUniverse->bPause) {
		// rendering and processing thread
		CriticalSectionWrapper csw(pUniverse->cs);

		pUniverse->process();
	}

	::glutPostRedisplay();
}

//#include <bitset>

void Universe::mouseCallback(int button, int state, int x, int y) {
	// clear special key modifiers to resolve incompatibilities Windows vs Linux
	const int buttonMask = 0x7; // GLUT mouse left, right, wheel, scroll up/dn, right+up/dn
	
	//std::bitset<32> inp(button);
	//std::bitset<32> out(((button & buttonMask) | (button == 0x8 ? 0x8 : 0x0)) | keyboardMask());
	//printf("mouseCallback: %s -> %s\n", inp.to_string().c_str(), out.to_string().c_str());

	// rendering and processing thread
	CriticalSectionWrapper csw(pUniverse->cs);

	if (pUniverse->currentScene != pUniverse->sceneList.end() && !(*pUniverse->currentScene)->isAsync())
		(*pUniverse->currentScene)->mouseHandler(((button & buttonMask) | (button == 0x8 ? 0x8 : 0x0)) | keyboardMask(), state, x, y);
}

void Universe::motionCallback(int x, int y) {
	// rendering and processing thread
	CriticalSectionWrapper csw(pUniverse->cs);
	
	if (pUniverse->currentScene != pUniverse->sceneList.end() && !(*pUniverse->currentScene)->isAsync())
		(*pUniverse->currentScene)->motionHandler(x, y);
}

void Universe::keyboardCallback(unsigned char key, int x, int y) {
	// rendering and processing thread
	CriticalSectionWrapper csw(pUniverse->cs);

	pUniverse->keyboardHandler(key | (keyboardMask() & ~(UIKeyboardMouseCallback::KEY_SHIFT | UIKeyboardMouseCallback::KEY_CTRL)), x, y);
}

void Universe::arrowKeyCallback(int key, int x, int y) {
	// rendering and processing thread
	CriticalSectionWrapper csw(pUniverse->cs);
	
	pUniverse->keyboardHandler(key | (keyboardMask() & ~(UIKeyboardMouseCallback::KEY_SHIFT | UIKeyboardMouseCallback::KEY_CTRL)) | KEY_SPECIAL, x, y);
}

void Universe::keyboardHandler(int key, int x, int y) {
	switch (key) {
	//case 27: // esc
	//	keyInterrupted = true;
	//	::glutLeaveMainLoop();
	//	goto WAIT_KEY;
	case (GLUT_KEY_PAGE_UP | KEY_SPECIAL):// pgup
		if (currentScene == sceneList.begin())
			currentScene = --sceneList.end();
		else
			--currentScene;

		// setup the new scene
		setupCurrentScene();
		goto WAIT_KEY;
	case (GLUT_KEY_PAGE_DOWN | KEY_SPECIAL): // pgdown
		if (currentScene == --sceneList.end())
			currentScene = sceneList.begin();
		else
			++currentScene;

		// setup the new scene
		setupCurrentScene();
		goto WAIT_KEY;
	case 16: // ctrl+p
		bPause = !bPause;
		break;
	}

	if (currentScene != sceneList.end() && !(*currentScene)->isAsync())
		(*currentScene)->keyboardHandler(key, x, y);

	WAIT_KEY:
	this->key = key;
	evWaitKey.set(true);
}

int Universe::keyboardMask() {
	const int mask = ::glutGetModifiers();
	return (mask & GLUT_ACTIVE_SHIFT ? UIKeyboardMouseCallback::KEY_SHIFT : 0) | (mask & GLUT_ACTIVE_CTRL ? UIKeyboardMouseCallback::KEY_CTRL : 0) | (mask & GLUT_ACTIVE_ALT ? UIKeyboardMouseCallback::KEY_ALT : 0);
}

void Universe::signalHandler(int signal) {
	if (!pUniverse)
		return;

	if (signal == SIGINT) {
		pUniverse->context.info("Universe::signalHandler(): SIGINT detected, terminating...\n");
		pUniverse->bTerminate = true;
	}
}

void Universe::setHandlerTerminate(HandlerTerminate& handlerTerminate) {
	CriticalSectionWrapper csw(pUniverse->cs);
	this->handlerTerminate = handlerTerminate;
}

//------------------------------------------------------------------------------

void Universe::process() {
	const SecTmReal timeCurrent = context.getTimer().elapsed();
	const SecTmReal timeElapsed = simulationRealTime ? timeCurrent - simulationTimeStamp : SecTmReal(1.0) / simulationFPS;
	simulationTimeStamp = timeCurrent;

	// pre-processing
	for (SceneList::iterator i = sceneList.begin(); i != sceneList.end(); ++i)
		if (!(*i)->isAsync())
			(*i)->preprocess(timeElapsed);
		
	// processing
	for (SceneList::iterator i = sceneList.begin(); i != sceneList.end(); ++i)
		if (!(*i)->isAsync())
			process(**i, timeElapsed);
		
	// post-processing
	for (SceneList::iterator i = sceneList.begin(); i != sceneList.end(); ++i)
		if (!(*i)->isAsync())
			(*i)->postprocess(timeElapsed);
}

void Universe::process(Scene& scene, SecTmReal timeElapsed) {
}

//------------------------------------------------------------------------------

void Universe::setTitle(const char* title) {
	if (interrupted())
		return;

	// main thread
	CriticalSectionWrapper csw(cs);

	setTitleSync(title);
}

void Universe::setSize(int width, int height) {
	reshapeCallback(width, height);	
	bSize = true;
}

Scene::Desc::Ptr Universe::createSceneDesc() const {
	return Scene::Desc::Ptr(new Scene::Desc);
}

Scene* Universe::insertScene(const Scene::Ptr& pScene) {
	if (pScene == NULL)
		throw MsgUniverseSceneCreate(Message::LEVEL_CRIT, "Universe::insertScene(): Invalid scene");

	{
		// main thread
		CriticalSectionWrapper csw(cs);
		sceneList.push_back(SceneList::Pair(pScene.get(), pScene));
	}

	return pScene.get();
}

Scene* Universe::createScene(const Scene::Desc& desc) {
	return insertScene(desc.create(*this));
}

void Universe::releaseScene(Scene& scene) {
	if (!sceneList.contains(&scene))
		throw MsgUniverseSceneRelease(Message::LEVEL_ERROR, "Universe::releaseScene(): Unable to find Scene");

	{
		// main thread
		scene.release();
		CriticalSectionWrapper csw(cs);
		sceneList.erase(&scene);
		currentScene = sceneList.begin();
	}
}

Universe::SceneList Universe::getSceneList() const {
	// main thread
	CriticalSectionWrapper csw(cs);
	return sceneList; // copy constructor called before ~CriticalSectionWrapper()
}

//------------------------------------------------------------------------------

SecTmReal Universe::getSimulationFPS() const {
	return simulationFPS;
}

int Universe::getWindowWidth() const {
	return windowWidth;
}

int Universe::getWindowHeight() const {
	return windowHeight;
}

//------------------------------------------------------------------------------

void Universe::setCapture(golem::UICapture* uiCapture) {
	// main thread
	CriticalSectionWrapper csw(cs);
	this->uiCapture = uiCapture;
}

golem::UICapture* Universe::getCapture() {
	return uiCapture;
}

const golem::UICapture* Universe::getCapture() const {
	return uiCapture;
}

//------------------------------------------------------------------------------
