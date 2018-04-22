/** @file Scene.cpp
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Math/Quat.h>
#include <Golem/Sys/Stream.h>
#include <Golem/Sim/Scene.h>
#include <Golem/Sim/Universe.h>
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Plugin/Data.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif


//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void Scene::Desc::load(const golem::XMLContext* xmlcontext) {
	XMLData("name", name, const_cast<golem::XMLContext*>(xmlcontext), false);
	openGLSeq.clear();
	XMLData(openGLSeq, openGLSeq.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "opengl", false);
}

//------------------------------------------------------------------------------

Scene::Scene(Universe &universe) :	universe(universe), context(universe.getContext()) {
}

Scene::~Scene() {
}

void Scene::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgSceneInvalidDesc(Message::LEVEL_CRIT, "Scene::create(): Invalid description");

	name = desc.name;
	openGLSeq = desc.openGLSeq;
	asyncScene = desc.asyncScene;

	for (OpenGL::Seq::iterator i = this->openGLSeq.begin(); i != this->openGLSeq.end(); ++i)
		i->init(REAL_ONE / universe.simulationScale);
	openGLPtr = this->openGLSeq.begin();
	openGL = *openGLPtr;

	mouseButton = viewMouseButton = desc.mouseButton;
	mouseX = 0;
	mouseY = 0;
	motionHandler(0, 0);

	help = desc.help;

	glMat = false;

	frameMode = OpenGL::FRAME_DISABLED;
}

void Scene::release() {
	// remove objects explicitly because they can use other resources which have to be released manually
	while (!objectList.empty())
		releaseObject(*objectList.back());
}

//------------------------------------------------------------------------------

void Scene::preprocess(SecTmReal elapsedTime) {
	debugRenderer.reset();

	for (ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); ++i) {
		(*i)->preprocess(elapsedTime);

		if (openGL.draw & OpenGL::DRAW_DEBUG_NORMALS) {
			const Actor* actor = dynamic_cast<const Actor*>(*i);
			if (actor) {
				const Mat34 pose = actor->getPoseSync();
				const Actor::BoundsConstSeq& bounds = actor->getBoundsSeq();
				for (Actor::BoundsConstSeq::const_iterator j = bounds.begin(); j != bounds.end(); ++j) {
					const BoundingConvexMesh* mesh = dynamic_cast<const BoundingConvexMesh*>(*j);
					if (mesh) {
						const U32 numOfTriangles = (U32)mesh->getTriangles().size();
						const Vec3* vertex = mesh->getVertices().data();
						const Triangle* triangle = mesh->getTriangles().data();
						for (U32 i = 0; i < numOfTriangles; ++i) {
							Vec3 p0;
							p0.add(vertex[triangle[i].t1], vertex[triangle[i].t2]);
							p0.add(p0, vertex[triangle[i].t3]);
							p0 /= 3;
							pose.multiply(p0, p0);
							Vec3 p1;
							pose.R.multiply(p1, mesh->getNormals()[i]);
							p1.multiplyAdd(openGL.drawNormalLen, p1, p0);
							debugRenderer.addLine(p0, p1, RGBA::CYAN);
						}
					}
				}
			}
		}
	}
}

void Scene::postprocess(SecTmReal elapsedTime) {
	for (ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); ++i)
		(*i)->postprocess(elapsedTime);
}

//------------------------------------------------------------------------------

void Scene::initOpenGL() {
	// Setup default render states
	::glClearColor(
		GLclampf(openGL.clearColor._rgba.r)/255.0f,
		GLclampf(openGL.clearColor._rgba.g)/255.0f,
		GLclampf(openGL.clearColor._rgba.b)/255.0f,
		GLclampf(openGL.clearColor._rgba.a)/255.0f
	);
	::glEnable(GL_DEPTH_TEST);
	::glEnable(GL_COLOR_MATERIAL);
	::glEnable(GL_CULL_FACE);

	// Setup lighting
	GLfloat ambientColor [] = {
		GLfloat(openGL.ambientColor._rgba.r)/255.0f,
		GLfloat(openGL.ambientColor._rgba.g)/255.0f,
		GLfloat(openGL.ambientColor._rgba.b)/255.0f,
		GLfloat(openGL.ambientColor._rgba.a)/255.0f
	};
	::glLightfv(GL_LIGHT0, GL_AMBIENT, ambientColor);
	GLfloat diffuseColor [] = {
		GLfloat(openGL.diffuseColor._rgba.r)/255.0f,
		GLfloat(openGL.diffuseColor._rgba.g)/255.0f,
		GLfloat(openGL.diffuseColor._rgba.b)/255.0f,
		GLfloat(openGL.diffuseColor._rgba.a)/255.0f
	};
	::glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseColor);
	GLfloat specularColor [] = {
		GLfloat(openGL.specularColor._rgba.r)/255.0f,
		GLfloat(openGL.specularColor._rgba.g)/255.0f,
		GLfloat(openGL.specularColor._rgba.b)/255.0f,
		GLfloat(openGL.specularColor._rgba.a)/255.0f
	};
	::glLightfv(GL_LIGHT0, GL_SPECULAR, specularColor);
	GLfloat position[] = {100.0f, 100.0f, 400.0f, 1.0f};
	::glLightfv(GL_LIGHT0, GL_POSITION, position);
	::glEnable(GL_LIGHTING);
	::glEnable(GL_LIGHT0);

	::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	::glEnable(GL_BLEND);
}

void Scene::initCamera() const {
	// Setup camera
	if (glMat) {
		std::vector<double> mat(16);

		// intrinsic camera parameters
		::glMatrixMode(GL_PROJECTION);
		::glLoadMatrixf(openGL.glMatIntrinsic);

		// extrinsic camera parameters
		::glMatrixMode(GL_MODELVIEW);
		::glLoadMatrixf(openGL.glMatExtrinsic);
	}
	else {
		::glMatrixMode(GL_PROJECTION);
		::glLoadIdentity();
		::gluPerspective(
			GLdouble(60.0),
			GLdouble(universe.glAspectRatio),
			GLdouble(0.1 / universe.simulationScale),
			GLdouble(100.0 / universe.simulationScale)
		);
		::glGetFloatv(GL_PROJECTION_MATRIX, openGL.glMatIntrinsic);
		::glMatrixMode(GL_MODELVIEW);
		::glLoadIdentity();
		::gluLookAt(
			GLdouble(openGL.viewPoint.v1), GLdouble(openGL.viewPoint.v2), GLdouble(openGL.viewPoint.v3),
			GLdouble(openGL.viewPoint.v1 + openGL.viewDir.v1), GLdouble(openGL.viewPoint.v2 + openGL.viewDir.v2), GLdouble(openGL.viewPoint.v3 + openGL.viewDir.v3),
			GLdouble(openGL.viewUp.v1), GLdouble(openGL.viewUp.v2), GLdouble(openGL.viewUp.v3)
		);
		::glGetFloatv(GL_MODELVIEW_MATRIX, openGL.glMatExtrinsic);

		//::glMatrixMode(GL_PROJECTION);
		//::glLoadIdentity();
		//::gluPerspective(
		//	GLdouble(60.0),
		//	GLdouble(universe.glAspectRatio),
		//	GLdouble(0.1/universe.simulationScale),
		//	GLdouble(100.0/universe.simulationScale)
		//);
		//::gluLookAt(
		//	GLdouble(openGL.viewPoint.v1), GLdouble(openGL.viewPoint.v2), GLdouble(openGL.viewPoint.v3),
		//	GLdouble(openGL.viewPoint.v1 + openGL.viewDir.v1), GLdouble(openGL.viewPoint.v2 + openGL.viewDir.v2), GLdouble(openGL.viewPoint.v3 + openGL.viewDir.v3),
		//	GLdouble(openGL.viewUp.v1), GLdouble(openGL.viewUp.v2), GLdouble(openGL.viewUp.v3)
		//);
		//::glMatrixMode(GL_MODELVIEW);
		//::glLoadIdentity();
	}
}

void Scene::render() const {
	for (ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); ++i)
		(*i)->customRender();

	initCamera();

	for (ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); ++i)
		(*i)->render();

	if (openGL.draw & OpenGL::DRAW_DEBUG) { // csOpenGL is always locked here
		if (openGL.draw & OpenGL::DRAW_DEBUG_NORMALS)
			debugRenderer.render();
	}
}

//------------------------------------------------------------------------------

void Scene::mouseHandler(int button, int state, int x, int y) {
	this->mouseButton = button;
	this->mouseX = x;
	this->mouseY = y;

	for (ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); ++i)
		(*i)->mouseHandler(button, state, x, y);
}

void Scene::motionHandler(int x, int y) {
	if (!glMat && mouseButton == viewMouseButton) {
		int dx = mouseX - x;
		int dy = mouseY - y;

		openGL.viewDir.normalise();
		Vec3 viewNormal;
		viewNormal.cross(openGL.viewDir, openGL.viewUp);

		Quat qx(REAL_PI*1.0*dx/180.0, openGL.viewUp);
		qx.multiply(openGL.viewDir, openGL.viewDir);
		Quat qy(REAL_PI*1.0*dy/180.0, viewNormal);
		qy.multiply(openGL.viewDir, openGL.viewDir);

		mouseX = x;
		mouseY = y;
	}

	for (ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); ++i)
		(*i)->motionHandler(x, y);
}

void Scene::keyboardHandler(int key, int x, int y) {
	Vec3 viewNormal;
	viewNormal.cross(openGL.viewDir, openGL.viewUp);

	switch (key) {
	case '?':
		printHelp();
		return;
	case (GLUT_KEY_UP | Universe::KEY_SPECIAL):// arrow up
		if (!glMat) openGL.viewPoint += openGL.viewDir*openGL.viewInc/universe.simulationScale;
		break;
	case (GLUT_KEY_DOWN | Universe::KEY_SPECIAL):// arrow down
		if (!glMat) openGL.viewPoint -= openGL.viewDir*openGL.viewInc/universe.simulationScale;
		break;
	case (GLUT_KEY_LEFT | Universe::KEY_SPECIAL):// arrow left
		if (!glMat) openGL.viewPoint -= viewNormal*openGL.viewInc/universe.simulationScale;
		break;
	case (GLUT_KEY_RIGHT | Universe::KEY_SPECIAL):// arrow right
		if (!glMat) openGL.viewPoint += viewNormal*openGL.viewInc/universe.simulationScale;
		break;
	case (GLUT_KEY_UP | Universe::KEY_SPECIAL | Universe::KEY_ALT):// arrow up
		if (!glMat) {
			frameMode = frameMode + 1 < OpenGL::FRAME_SIZE ? frameMode + 1 : OpenGL::FRAME_DISABLED;
			context.write("Frame mode: %s\n", OpenGL::frameName[frameMode]);
		}
		break;
	case (GLUT_KEY_DOWN | Universe::KEY_SPECIAL | Universe::KEY_ALT):// arrow down
		if (!glMat) {
			frameMode = frameMode > OpenGL::FRAME_DISABLED ? frameMode - 1 : OpenGL::FRAME_SIZE - 1;
			context.write("Frame mode: %s\n", OpenGL::frameName[frameMode]);
		}
		break;
	case (GLUT_KEY_LEFT | Universe::KEY_SPECIAL | Universe::KEY_ALT):// arrow left
		if (!glMat) {
			if (frameMode == OpenGL::FRAME_X || frameMode == OpenGL::FRAME_Y || frameMode == OpenGL::FRAME_Z)
				(frameMode == OpenGL::FRAME_X ? openGL.viewPoint.x : frameMode == OpenGL::FRAME_Y ? openGL.viewPoint.y : openGL.viewPoint.z) -= openGL.viewInc / universe.simulationScale;
			else if (frameMode == OpenGL::FRAME_ROLL || frameMode == OpenGL::FRAME_PITCH || frameMode == OpenGL::FRAME_YAW) {
				Mat33 rot;
				rot.fromEuler(
					frameMode == OpenGL::FRAME_ROLL ? -REAL_PI * openGL.viewInc / universe.simulationScale : REAL_ZERO,
					frameMode == OpenGL::FRAME_PITCH ? -REAL_PI * openGL.viewInc / universe.simulationScale : REAL_ZERO,
					frameMode == OpenGL::FRAME_YAW ? -REAL_PI * openGL.viewInc / universe.simulationScale : REAL_ZERO
					);
				openGL.viewDir = rot * openGL.viewDir;
			}
		}
		break;
	case (GLUT_KEY_RIGHT | Universe::KEY_SPECIAL | Universe::KEY_ALT):// arrow right
		if (!glMat) {
			if (frameMode == OpenGL::FRAME_X || frameMode == OpenGL::FRAME_Y || frameMode == OpenGL::FRAME_Z)
				(frameMode == OpenGL::FRAME_X ? openGL.viewPoint.x : frameMode == OpenGL::FRAME_Y ? openGL.viewPoint.y : openGL.viewPoint.z) += openGL.viewInc / universe.simulationScale;
			else if (frameMode == OpenGL::FRAME_ROLL || frameMode == OpenGL::FRAME_PITCH || frameMode == OpenGL::FRAME_YAW) {
				Mat33 rot;
				rot.fromEuler(
					frameMode == OpenGL::FRAME_ROLL ? +REAL_PI * openGL.viewInc / universe.simulationScale : REAL_ZERO,
					frameMode == OpenGL::FRAME_PITCH ? +REAL_PI * openGL.viewInc / universe.simulationScale : REAL_ZERO,
					frameMode == OpenGL::FRAME_YAW ? +REAL_PI * openGL.viewInc / universe.simulationScale : REAL_ZERO
				);
				openGL.viewDir = rot * openGL.viewDir;
			}
		}
		break;
	case 22:// ctrl+v
		context.write(
			"<view_point v1=\"%.4f\" v2=\"%.4f\" v3=\"%.4f\"/>\n<view_dir v1=\"%.4f\" v2=\"%.4f\" v3=\"%.4f\"/>\n<view_up v1=\"%.4f\" v2=\"%.4f\" v3=\"%.4f\"/>\n",
			universe.simulationScale*openGL.viewPoint.v1, universe.simulationScale*openGL.viewPoint.v2, universe.simulationScale*openGL.viewPoint.v3,
			openGL.viewDir.v1, openGL.viewDir.v2, openGL.viewDir.v3,
			openGL.viewUp.v1, openGL.viewUp.v2, openGL.viewUp.v3
		);
		return;
	case '<':
		if (!glMat) {
			openGLPtr = openGLPtr == openGLSeq.begin() ? --openGLSeq.end() : openGLPtr - 1;
			openGL = *openGLPtr;
			context.write("View point: %s\n", openGL.viewName.c_str());
			initOpenGL();
		}
		return;
	case '>':
		if (!glMat) {
			openGLPtr = openGLPtr == --openGLSeq.end() ? openGLSeq.begin() : openGLPtr + 1;
			openGL = *openGLPtr;
			context.write("View point: %s\n", openGL.viewName.c_str());
			initOpenGL();
		}
		return;
	case '!':
	{
		const U32 mode = openGL.draw & OpenGL::DRAW_DEFAULT;
		openGL.draw &= ~OpenGL::DRAW_DEFAULT;
		openGL.draw |= (mode + 1) & OpenGL::DRAW_DEFAULT;
		break;
	}
	case '@':
	{
		const U32 mode = openGL.draw & OpenGL::DRAW_DEBUG;
		openGL.draw &= ~OpenGL::DRAW_DEBUG;
		openGL.draw |= (mode > 0 ? mode << 1 : OpenGL::DRAW_DEBUG_SIMULATION) & OpenGL::DRAW_DEBUG;
		break;
	}
	}

	for (ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); ++i)
		(*i)->keyboardHandler(key, x, y);
}

//------------------------------------------------------------------------------

void Scene::printHelp() const {
	for (StrMap::const_iterator i = help.begin(); i != help.end(); ++i)
		context.write("%s", i->second.c_str());
}

//------------------------------------------------------------------------------

Actor::Desc::Ptr Scene::createActorDesc() const {
	return Actor::Desc::Ptr(new Actor::Desc);
}

Object* Scene::insertObject(const Object::Ptr& pObject) {
	if (pObject == NULL)
		throw MsgSceneObjectCreate(Message::LEVEL_CRIT, "Scene::insertObject(): Invalid object");

	{
		// main thread
		CriticalSectionWrapper csw(universe.getCS());
		objectList.push_back(ObjectList::Pair(pObject.get(), pObject));
	}

	return pObject.get();
}

Object* Scene::createObject(const Object::Desc& desc) {
	return insertObject(desc.create(*this));
}

void Scene::releaseObject(Object& object) {
	if (!objectList.contains(&object)) {
		context.error("Scene::releaseObject(): Unable to find specified object\n");
		return;
	}

	{
		// main thread
		object.release();
		CriticalSectionWrapper csw(universe.getCS());
		objectList.erase(&object);
	}
}

const Scene::ObjectList& Scene::getObjectList() const {
	// main thread
	CriticalSectionWrapper csw(universe.getCS());
	return objectList; // copy constructor called before ~CriticalSectionWrapper()
}

//------------------------------------------------------------------------------

U32 Scene::getDraw() const {
	return openGL.draw; // atomic
}

void Scene::setDraw(U32 draw) {
	openGL.draw = draw; // atomic
}

void Scene::getOpenGL(OpenGL& openGL) const {
	// main thread
	CriticalSectionWrapper csw(universe.getCS());
	openGL = this->openGL;
	openGL.x = this->universe.windowWidth;
	openGL.y = this->universe.windowHeight;
}

void Scene::setOpenGL(const OpenGL& openGL) {
	// main thread
	CriticalSectionWrapper csw(universe.getCS());
	this->openGL = openGL;
	initOpenGL();
}

void Scene::setGLMat(bool enable) {
	glMat = enable;
}

CriticalSection &Scene::getCS() const {
	return universe.getCS();
}

//------------------------------------------------------------------------------

