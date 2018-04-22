/** @file Object.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Phys/PhysObject.h>
#include <Golem/Phys/PhysUniverse.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void PhysActor::Desc::load(const golem::XMLContext* xmlcontext) {
	Actor::Desc::load(xmlcontext);
}

//------------------------------------------------------------------------------

PhysActor::PhysActor(Scene &scene) : Actor(scene), scene(NULL), universe(NULL) {
	this->scene = dynamic_cast<PhysScene*>(&scene);
	if (!this->scene)
		throw MsgPhysActor(Message::LEVEL_CRIT, "PhysActor::PhysActor(): only PhysScene can create PhysActor");
	this->universe = &this->scene->getUniverse();
}

void PhysActor::create(const PhysActor::Desc& desc) {
	Object::create(desc); // throws

	pose = desc.pose;
	appearance = desc.appearance;
	kinematic = desc.kinematic;

	boundsDataMap.clear();
	boundsConstSeq.clear();

	// NxShape
	for (NxArray<NxShapeDesc*, NxAllocatorDefault>::const_iterator i = desc.nxActorDesc.shapes.begin(); i != desc.nxActorDesc.shapes.end(); ++i) {
		BoundsData boundsData;

		boundsData.pBoundsDesc = scene->createBoundsDesc(**i); // Access to PhysX (for ConvexMesh)
		if (boundsData.pBoundsDesc == NULL)
			throw MsgPhysActorBoundsDescCreate(Message::LEVEL_CRIT, "PhysActor::create(): Unable to create bounds description");

		boundsData.pBounds = boundsData.pBoundsDesc->create();
		if (boundsData.pBounds == NULL)
			throw MsgPhysActorBoundsCreate(Message::LEVEL_CRIT, "PhysActor::create(): Unable to create bounds");

		boundsConstSeq.push_back(boundsData.pBounds.get());
		boundsDataMap.insert(std::make_pair(boundsData.pBounds.get(), boundsData));
	}

	try {
		// Bounds -> NxShape
		for (Bounds::Desc::Seq::const_iterator i = desc.boundsDescSeq.begin(); i != desc.boundsDescSeq.end(); ++i) {
			BoundsData* const pBoundsData = addBounds(*i); // throws

			desc.nxActorDesc.shapes.push_back(reinterpret_cast<NxShapeDesc*>(pBoundsData->pShapeDesc));

			// HACK - plane must have no body
			if ((*i)->getType() == Bounds::TYPE_PLANE)
				desc.nxActorDesc.body = NULL;
		}
	}
	catch (const std::exception& ex) {
		throw MsgPhysActor(Message::LEVEL_CRIT, "PhysActor::create(): %s", ex.what()); // LEVEL_ERROR ==> LEVEL_CRIT
	}

	// NxActor
	desc.nxActorDesc.globalPose.M.setRowMajor(&pose.R.m11);
	desc.nxActorDesc.globalPose.t.set(&pose.p.v1);

	if (!desc.nxActorDesc.isValid())
		throw MsgPhysActorNxActorCreate(Message::LEVEL_CRIT, "PhysActor::create(): invalid NxActorDesc");

	{
		// main thread
		// Access to PhysX requires CS, despite the object is not activated yet
		CriticalSectionWrapper csw(universe->getCS());

		pNxActor = scene->getNxScene()->createActor(desc.nxActorDesc);
		if (pNxActor == NULL)
			throw MsgPhysActorNxActorCreate(Message::LEVEL_CRIT, "PhysActor::create(): Unable to create NxActor");

		if (kinematic)
			pNxActor->raiseBodyFlag(NX_BF_KINEMATIC); // overwrite
	}

	//context.debug("PhysActor::create(): ptr=%x, kinematic=%s, shapes=%u\n", (size_t)pNxActor, kinematic ? "YES" : "NO", pNxActor->getNbShapes());
}

void PhysActor::release() {
	if (pNxActor != NULL) {
		// main thread
		CriticalSectionWrapper csw(universe->getCS());

		//context.debug("PhysActor::release(): ptr=%x\n", (size_t)pNxActor);
		
		scene->getNxScene()->releaseActor(*pNxActor);
		pNxActor = NULL;
		// TODO release resources on Scene
	}
	Actor::release();
}

//------------------------------------------------------------------------------

Actor::BoundsData* PhysActor::addBounds(Bounds::Desc::Ptr pDesc) {
	// create first NxShapeDesc, this updates pDesc if required (e.g. convex mesh cooking)
	NxShapeDesc* pShapeDesc = scene->createNxShapeDesc(pDesc); // Access to PhysX cooking (for ConvexMesh)
	if (pShapeDesc == NULL)
		throw MsgPhysActorNxShapeDescCreate(Message::LEVEL_ERROR, "PhysActor::addBounds(): Unable to create NxShape description");

	BoundsData* const pBoundsData = Actor::addBounds(pDesc);
	pBoundsData->pShapeDesc = pShapeDesc;
	
	return pBoundsData;
}

const Bounds* PhysActor::createBounds(Bounds::Desc::Ptr pDesc) {
	BoundsData* pBoundsData = NULL;

	// main thread
	CriticalSectionWrapper csw(universe->getCS());

	pBoundsData = PhysActor::addBounds(pDesc);
	pBoundsData->pShape = pNxActor->createShape(*reinterpret_cast<NxShapeDesc*>(pBoundsData->pShapeDesc));
	if (pBoundsData->pShape == NULL)
		throw MsgPhysActorNxShapeCreate(Message::LEVEL_ERROR, "PhysActor::createBounds(): unable to find NxShape");

	return pBoundsData->pBounds.get();
}

//------------------------------------------------------------------------------

Actor::BoundsData PhysActor::removeBounds(const Bounds* pBounds) {
	const BoundsData data = Actor::removeBounds(pBounds);
	
	scene->releaseBoundsDesc(data.pBoundsDesc);
	
	return data;
}

void PhysActor::releaseBounds(const Bounds& bounds) {
	BoundsData data;

	// main thread
	CriticalSectionWrapper csw(universe->getCS());

	data = PhysActor::removeBounds(&bounds);
	
	NxShape* pNxShape = reinterpret_cast<NxShape*>(data.pShape);
	if (pNxShape == NULL)
		throw MsgActorBounds(Message::LEVEL_ERROR, "PhysActor::releaseBounds(): unable to find NxShape");
	pNxActor->releaseShape(*pNxShape);
}

//------------------------------------------------------------------------------

void PhysActor::setPoseSync(const Mat34& pose, bool bMove) {
	if (!kinematic) {
		context.warning("PhysActor::setPose(): Cannot change pose of dynamic actor\n");
		return;
	}

	NxMat34 nxPose;
	nxPose.M.setRowMajor(&pose.R.m11);
	nxPose.t.set(&pose.p.v1);
	
	if (bMove)
		pNxActor->moveGlobalPose(nxPose);
	// set the actor pose in any case to avoid peculiar PhysX behavior (at small scale) when moving pose only
	pNxActor->setGlobalPose(nxPose);
}

void PhysActor::setPose(const Mat34& pose, bool bMove) {
	// main thread
	CriticalSectionWrapper csw(universe->getCS());

	setPoseSync(pose, bMove);
}

//------------------------------------------------------------------------------

void PhysActor::postprocess(SecTmReal elapsedTime) {
	// thread-safe (in sync) with PhysX, only async with main thread
	const NxMat34 nxPose = pNxActor->getGlobalPose();
	nxPose.M.getRowMajor(&pose.R.m11);
	nxPose.t.get(&pose.p.v1);
}

//------------------------------------------------------------------------------
