/** @file PhysUniverse.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Phys/PhysUniverse.h>
#include <Golem/Phys/NxCooking.h>
#include <Golem/Phys/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

// HACK: FreeGLUT window buffer size offset
#define GLUT_WINDOW_OFFSET_X 8
#define GLUT_WINDOW_OFFSET_Y 8

//------------------------------------------------------------------------------

NxErrorStream::NxErrorStream(Context &context) : context(context) {
}

void NxErrorStream::reportError(NxErrorCode code, const char* message, const char* file, int line) {
	Message::Level level;
	std::string codeStr;
	
	switch (code) {
	case NXE_INVALID_PARAMETER:
		codeStr = "invalid parameter";
		level = Message::LEVEL_ERROR;
		break;
	case NXE_INVALID_OPERATION:
		codeStr = "invalid operation";
		level = Message::LEVEL_ERROR;
		break;
	case NXE_OUT_OF_MEMORY:
		codeStr = "out of memory";
		level = Message::LEVEL_ERROR;
		break;
	case NXE_DB_WARNING:
		codeStr = "warning";
		level = Message::LEVEL_WARNING;
		break;
	case NXE_DB_INFO:
		codeStr = "info";
		level = Message::LEVEL_INFO;
		break;
	default:
		codeStr = "unknown error";
		level = Message::LEVEL_ERROR;
	}

	context.getMessageStream()->write(level, "NxErrorStream::reportError(): %s (%d): %s: %s\n", file, line, codeStr.c_str(), message);
}

NxAssertResponse NxErrorStream::reportAssertViolation(const char* message, const char* file, int line) {
	context.error("NxErrorStream::reportAssertViolation(): %s (%d): %s\n", file, line, message);

#ifdef WIN32
	switch (MessageBox(0, message, "AssertViolation, see console for details.", MB_ABORTRETRYIGNORE)) {
	case IDRETRY:
		return NX_AR_CONTINUE;
	case IDIGNORE:
		return NX_AR_IGNORE;
	case IDABORT:
	default:
		return NX_AR_BREAKPOINT;
	}
#endif
}

void NxErrorStream::print(const char* message) {
	context.error("NxErrorStream::print(): %s\n", message);
}

//------------------------------------------------------------------------------

void PhysUniverse::Desc::load(const golem::XMLContext* xmlcontext) {
	Universe::Desc::load(xmlcontext);

	XMLData("skin_width", skinWidth, xmlcontext->getContextFirst("simulation"));
	XMLData("sleep_lin_vel_squared", sleepLinVelSquared, xmlcontext->getContextFirst("simulation"));
	XMLData("sleep_ang_vel_squared", sleepAngVelSquared, xmlcontext->getContextFirst("simulation"));
	XMLData("max_angular_velocity", maxAngularVelocity, xmlcontext->getContextFirst("simulation"));
	XMLData("bounce_threshold", bounceThreshold, xmlcontext->getContextFirst("simulation"));
}

//------------------------------------------------------------------------------

PhysUniverse::PhysUniverse(golem::Context& context) : Universe(context), nxErrorStream(context) {
	pNxPhysicsSDK = NULL;
}

PhysUniverse::~PhysUniverse() {
	Universe::release();
	PhysUniverse::release();
}

void PhysUniverse::create(const Desc& desc) {
	Universe::create(desc); // throws

	// Initialise PhysicsSDK
	pNxPhysicsSDK = ::NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION, 0, &nxErrorStream);
	if (pNxPhysicsSDK == NULL)
		throw MsgPhysUniversePhysXInit(Message::LEVEL_CRIT, "PhysUniverse::create(): Unable to initialize PhysX");

	// Initialise cooking
	if (!::InitCooking())
		throw MsgPhysUniversePhysXCookInit(Message::LEVEL_CRIT, "PhysUniverse::create(): Unable to initialize PhysX cooking library");
	
	const Real simulationScaleInv = REAL_ONE / simulationScale;
	
	pNxPhysicsSDK->setParameter(NX_SKIN_WIDTH,
		NxReal(desc.skinWidth * simulationScaleInv));
	pNxPhysicsSDK->setParameter(NX_DEFAULT_SLEEP_LIN_VEL_SQUARED,
		NxReal(desc.sleepLinVelSquared * simulationScaleInv * simulationScaleInv));
	pNxPhysicsSDK->setParameter(NX_DEFAULT_SLEEP_ANG_VEL_SQUARED,
		NxReal(desc.sleepAngVelSquared));
	pNxPhysicsSDK->setParameter(NX_MAX_ANGULAR_VELOCITY,
		NxReal(desc.maxAngularVelocity));
	pNxPhysicsSDK->setParameter(NX_BOUNCE_THRESHOLD,
		NxReal(desc.bounceThreshold * simulationScaleInv));
	//pNxPhysicsSDK->setParameter(NX_DYN_FRICT_SCALING,
	//	NxReal(desc.dynFrictScaling * simulationScaleInv));
	//pNxPhysicsSDK->setParameter(NX_STA_FRICT_SCALING,
	//	NxReal(desc.staFrictScaling * simulationScaleInv));
}

void PhysUniverse::release() {
	if (pNxPhysicsSDK != NULL) {
		::CloseCooking();
		::NxReleasePhysicsSDK(pNxPhysicsSDK);
		pNxPhysicsSDK = NULL;
	}
}

//------------------------------------------------------------------------------

void PhysUniverse::process(Scene& scene, SecTmReal timeElapsed) {
	PhysScene* const physScene = dynamic_cast<PhysScene*>(&scene);
	if (physScene) {
		NxScene* pNxScene = physScene->getNxScene();
		pNxScene->simulate((NxReal)timeElapsed);
		pNxScene->flushStream();
		pNxScene->fetchResults(NX_RIGID_BODY_FINISHED, true); // blocking call
	}
}

//------------------------------------------------------------------------------

Scene::Desc::Ptr PhysUniverse::createSceneDesc() const {
	return Scene::Desc::Ptr(new PhysScene::Desc);
}

//------------------------------------------------------------------------------