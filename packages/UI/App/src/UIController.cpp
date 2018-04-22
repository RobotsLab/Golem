/** @file UIController.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/App/UIController.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void BodyActor::release() {
	scene.releaseObject(*pActor);
}

void BodyActor::create(const BodyActor::Desc& desc) {
	Actor::create(desc); // throws
	
	pActor = dynamic_cast<Actor*>(scene.createObject(*desc.pActorDesc));
	if (!pActor)
		throw Message(Message::LEVEL_CRIT, "BodyActor::create(): unable to create body actor");

	pChain = desc.pChain;
	pJoint = desc.pJoint;
}

BodyActor::BodyActor(Scene &scene) : Actor(scene), pActor(NULL), pChain(NULL), pJoint(NULL) {
}

//------------------------------------------------------------------------------

Mat34 BodyActor::getPose() const {
	return pActor->getPose();
}

Mat34 BodyActor::getPoseSync() const {
	return pActor->getPoseSync();
}

void BodyActor::setPose(const Mat34& pose, bool bMove) {
	pActor->setPose(pose, bMove);
}

void BodyActor::setPoseAsync(const Mat34& pose, bool bMove) {
	pActor->setPoseSync(pose, bMove);
}

const Bounds* BodyActor::createBounds(Bounds::Desc::Ptr pDesc) {
	const Bounds* pBounds = pActor->createBounds(pDesc);
	
	if (pBounds != NULL)
		pChain != NULL ? pChain->addBoundsDesc(pDesc) : pJoint->addBoundsDesc(pDesc);

	return pBounds;
}

void BodyActor::releaseBounds(const Bounds& bounds) {
	const Bounds::Desc* pDesc = pActor->getBoundsDesc(bounds);
	
	pActor->releaseBounds(bounds);
	
	if (pDesc != NULL)
		pChain != NULL ? pChain->removeBoundsDesc(pDesc) : pJoint->removeBoundsDesc(pDesc);
}

Bounds::SeqPtr BodyActor::getLocalBoundsSeq(U32 group) const {
	return pActor->getLocalBoundsSeq(group);
}

Bounds::SeqPtr BodyActor::getGlobalBoundsSeq(U32 group) const {
	return pActor->getGlobalBoundsSeq(group);
}

const Bounds::Desc* BodyActor::getBoundsDesc(const Bounds& bounds) const {
	return pActor->getBoundsDesc(bounds);
}

void BodyActor::setBoundsGroup(const Bounds& bounds, U32 group) {
	pActor->setBoundsGroup(bounds, group);
}

void BodyActor::setBoundsGroup(U32 group) {
	pActor->setBoundsGroup(group);
}

void BodyActor::setAppearance(const Appearance &appearance) {
	pActor->setAppearance(appearance);
}

//------------------------------------------------------------------------------

UIControllerVis::UIControllerVis(Scene& scene, Controller& controller) : Object(scene), controller(controller), state(controller.createState()) {
}

void UIControllerVis::create(const UIControllerVis::Desc& desc) {
	if (!desc.isValid())
		throw Message(Message::LEVEL_CRIT, "UIControllerVis::create(): invalid description");

	stateInfo = controller.getStateInfo();

	jointAppearance = desc.jointAppearance;
	pathTimePast = desc.pathTimePast;
	pathTimeFuture = desc.pathTimeFuture;
	pathSegments = desc.pathSegments;
	pathSegmentDuration = desc.pathSegmentDuration;
	pathRenderer = desc.pathRenderer;
	stateRenderer = desc.stateRenderer;
	stateUseCommands = desc.stateUseCommands;
	stateJointFramesShow = desc.stateJointFramesShow;
	stateJointFramesSize = desc.stateJointFramesSize;

	chainActors.fill((BodyActor*)NULL);
	for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i)
		chainActors[i] = createBodyActor(controller.getChains()[i], NULL);
	jointActors.fill((BodyActor*)NULL);
	for (Configspace::Index i = stateInfo.getJoints().begin(); i < stateInfo.getJoints().end(); ++i)
		jointActors[i] = createBodyActor(NULL, controller.getJoints()[i]);

	collisionGroup = controllerGroup = findControllerBoundsGroup(stateInfo.getChains().begin(), stateInfo.getChains().end(), chainActors) | findControllerBoundsGroup(stateInfo.getJoints().begin(), stateInfo.getJoints().end(), jointActors);

	// initialise joints pose
	(void)getCoords(SEC_TM_REAL_ZERO, state, coords);
	syncBodyActorsPose(coords, false);

	scene.getHelp().insert(Scene::StrMapVal("0301", "  #                                       planner state/goal/path show\n"));
}

void UIControllerVis::release() {
	for (Configspace::Index i = stateInfo.getJoints().end(); i != stateInfo.getJoints().begin();)
		releaseBodyActor(&jointActors[--i]);
	for (Chainspace::Index i = stateInfo.getChains().end(); i != stateInfo.getChains().begin();)
		releaseBodyActor(&chainActors[--i]);
}

//------------------------------------------------------------------------------

BodyActor* UIControllerVis::createBodyActor(Chain* pChain, Joint* pJoint) {
	BodyActor* pBodyActor = NULL;

	// check if the bounds are not empty
	Bounds::Desc::SeqPtr pBoundsDescSeq = pChain != NULL ? pChain->getBoundsDescSeq() : pJoint->getBoundsDescSeq();
	if (!pBoundsDescSeq->empty()) {
		// create Joint Actors
		BodyActor::Desc actorDesc;
		actorDesc.pChain = pChain;
		actorDesc.pJoint = pJoint;

		actorDesc.pActorDesc = scene.createActorDesc(); // create description
		actorDesc.pActorDesc->appearance = jointAppearance;
		actorDesc.pActorDesc->boundsDescSeq.clear();
		actorDesc.pActorDesc->kinematic = true;

		for (Bounds::Desc::Seq::const_iterator j = pBoundsDescSeq->begin(); j != pBoundsDescSeq->end(); ++j) {
			// Cloning descriptions results in different pointers in Joint and in Actor, therefore Actor::getBoundsDesc() will point
			// to bounds description which are *not* in Joint.
			// In effect UIControllerVis::BodyActor::releaseBounds() will *not* work, which is fine because these are initial bounds
			// which cannot be released anyway (see Actor::releaseBounds()).
			// On the other hand UIControllerVis::BodyActor::createBounds() and UIControllerVis::BodyActor::releaseBounds() will work fine, because
			// smart pointers are copied to both Joint and Actor.
			actorDesc.pActorDesc->boundsDescSeq.push_back((*j)->clone()); // clone description
		}

		pBodyActor = dynamic_cast<BodyActor*>(scene.createObject(actorDesc)); // throws
		if (pBodyActor == NULL)
			throw MsgUIController(Message::LEVEL_CRIT, "UIControllerVis::createBodyActor(): Unable to cast to body Actor");
	}

	return pBodyActor;
}

/*BodyActor* UIControllerVis::createBodyActor(Chain* pChain, Joint* pJoint) {
BodyActor* pBodyActor = NULL;

// check if the bounds are not empty
Bounds::Desc::SeqPtr pBoundsDescSeq = pChain != NULL ? pChain->getBoundsDescSeq() : pJoint->getBoundsDescSeq();
if (!pBoundsDescSeq->empty()) {
// create Joint Actors
BodyActor::Desc actorDesc;
NxBodyDesc nxBodyDesc;
actorDesc.nxActorDesc.body = &nxBodyDesc;
actorDesc.nxActorDesc.density = (NxReal)1.0;
actorDesc.kinematic = true;
actorDesc.appearance = jointAppearance;
actorDesc.pChain = pChain;
actorDesc.pJoint = pJoint;

for (Bounds::Desc::Seq::const_iterator j = pBoundsDescSeq->begin(); j != pBoundsDescSeq->end(); ++j) {
// Cloning descriptions results in different pointers in Joint and in Actor, therefore Actor::getBoundsDesc() will point
// to bounds description which are *not* in Joint.
// In effect UIControllerVis::BodyActor::releaseBounds() will *not* work, which is fine because these are initial bounds
// which cannot be released anyway (see Actor::releaseBounds()).
// On the other hand UIControllerVis::BodyActor::createBounds() and UIControllerVis::BodyActor::releaseBounds() will work fine, because
// smart pointers are copied to both Joint and Actor.
NxShapeDesc *pNxShapeDesc = scene.createNxShapeDesc((*j)->clone()); // clone description
if (pNxShapeDesc == NULL)
throw MsgUIControllerShapeDescCreate(Message::LEVEL_CRIT, "UIControllerVis::createBodyActor(): Unable to create shape description of %s", pChain != NULL ? pChain->getName().c_str() : pJoint->getName().c_str());
actorDesc.nxActorDesc.shapes.push_back(pNxShapeDesc);
}

pBodyActor = dynamic_cast<BodyActor*>(scene.createObject(actorDesc)); // throws
if (pBodyActor == NULL)
throw MsgUIControllerBodyActorCreate(Message::LEVEL_CRIT, "UIControllerVis::createBodyActor(): Unable to create body Actor");
}

return pBodyActor;
}*/

void UIControllerVis::releaseBodyActor(BodyActor** pBodyActor) {
	if (*pBodyActor != NULL) {
		scene.releaseObject(**pBodyActor);
		*pBodyActor = NULL;
	}
}

bool UIControllerVis::getCoords(SecTmReal t, Controller::State& state, WorkspaceJointCoord& coords) const {
	// get the current time and input pose (input to the controller - the desired pose)
	try {
		stateUseCommands ? controller.lookupCommand(t, state) : controller.lookupState(t, state);
	}
	catch (const Message& msg) {
		context.write(msg);
		return false;
	}

	// compute full forward kinematics
	controller.jointForwardTransform(state.cpos, coords);

	return true;
}

void UIControllerVis::syncBodyActorsPose(const WorkspaceJointCoord& coords, bool bMove) {
	// update positions of all visible joints
	for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i)
		if (chainActors[i] != NULL) {
			const Chain* chain = controller.getChains()[i];
			const Chainspace::Index linkedChainIndex = chain->getLinkedChainIndex();
			Mat34 pose = linkedChainIndex < i ? coords[stateInfo.getJoints(linkedChainIndex).end() - 1] : controller.getGlobalPose();
			pose.multiply(pose, chain->getLocalPose());

			chainActors[i]->setPoseAsync(pose, bMove);
		}

	for (Configspace::Index i = stateInfo.getJoints().begin(); i < stateInfo.getJoints().end(); ++i)
		if (jointActors[i] != NULL) {
			jointActors[i]->setPoseAsync(coords[i], bMove);
		}
}

//------------------------------------------------------------------------------

void UIControllerVis::preprocessRenderer(SecTmReal t, const Controller::State& state, const WorkspaceJointCoord& coord) {
	Controller::State command = controller.createState();

	try {
		controller.lookupCommand(SEC_TM_REAL_MAX, command);
	}
	catch (const Message& msg) {
		context.write(msg);
		return;
	}

	if (stateRenderer.show) {
		stateRenderer.reset(); // reset buffer
			
		// state
		stateRenderer.fromConfigspace(controller, &state, &state + 1);
		if (stateJointFramesShow) {
			for (golem::Configspace::Index i = state.getInfo().getJoints().begin(); i < state.getInfo().getJoints().end(); ++i)
				stateRenderer.addAxes3D(coord[i], stateJointFramesSize);
		}

		// command
		stateRenderer.fromConfigspace(controller, &command, &command + 1);
	}

	if (pathRenderer.show) {
		// begin with the target
		commands.clear();
		commands.push_back(command);
		
		// repeat until reaching the current time stamp
		for (U32 i = 1; i < pathSegments && command.t > t + pathTimePast; ++i) {
			command.t = std::max(t + pathTimePast, command.t - pathSegmentDuration);

			try {
				controller.lookupCommand(command.t, command);
			}
			catch (const Message& msg) {
				context.write(msg);
				return;
			}
			
			commands.push_back(command);
		}

		// send to the buffer
		pathRenderer.reset(); // reset buffer
		pathRenderer.fromConfigspace(controller, commands.begin(), commands.end());
	}
}

void UIControllerVis::keyboardHandler(int key, int x, int y) {
	switch (key) {
	case '#':
		stateRenderer.show = !stateRenderer.show;
		pathRenderer.show = !pathRenderer.show;
		break;
	}
}

void UIControllerVis::preprocess(SecTmReal elapsedTime) {
	const SecTmReal t = context.getTimer().elapsed();
	if (!getCoords(t, this->state, this->coords))
		return;

	syncBodyActorsPose(this->coords, true);
	preprocessRenderer(t, this->state, this->coords);
}

void UIControllerVis::render() const {
	stateRenderer.render();
	pathRenderer.render();
}

//------------------------------------------------------------------------------

Bounds::SeqPtr UIControllerVis::getCollisionBounds() const {
	const U32 group = getCollisionBoundsGroup() & getControllerBoundsGroup();
	Bounds::SeqPtr pCollisionBounds(new Bounds::Seq());
	
	if (group != 0x0) {
		// main thread
		CriticalSectionWrapper csw(universe.getCS());

		const Scene::ObjectList& objectList = scene.getObjectList();
		for (Scene::ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); ++i) {
			const Actor *pActor = dynamic_cast<const Actor*>(*i);
			// TODO use binary search here with a sorted container e.g. std::set
			if (pActor == NULL)
				continue;

			for (Chainspace::Index j = stateInfo.getChains().begin(); j != stateInfo.getChains().end(); ++j)
				if (chainActors[j] && (chainActors[j] == pActor || chainActors[j]->getActor() == pActor))
					goto NEXT;
			for (Configspace::Index j = stateInfo.getJoints().begin(); j != stateInfo.getJoints().end(); ++j)
				if (jointActors[j] && (jointActors[j] == pActor || jointActors[j]->getActor() == pActor))
					goto NEXT;

			{
				Bounds::SeqPtr pBoundsSeq = pActor->getGlobalBoundsSeq(group);
				pCollisionBounds->insert(pCollisionBounds->end(), pBoundsSeq->begin(), pBoundsSeq->end());
			}

			NEXT:;
		}
	}

	return pCollisionBounds;
}

//------------------------------------------------------------------------------

UIController::UIController(Scene &scene) : Object(scene) {
}

void UIController::create(const UIController::Desc& desc) {
	Object::create(desc); // throws

	pController = desc.pControllerDesc->create(context); // throws
	pUIControllerVis = dynamic_cast<UIControllerVis*>(scene.insertObject(desc.uiControllerVisDesc.create(scene, *pController))); // throws
	if (!pUIControllerVis)
		throw Message(Message::LEVEL_CRIT, "UIController::create(): unable to create controller visualisation");
}

void UIController::release() {
	scene.releaseObject(*pUIControllerVis);
	pController.release();
}

//------------------------------------------------------------------------------
