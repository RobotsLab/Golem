/** @file PhysScene.cpp
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Phys/PhysScene.h>
#include <Golem/Phys/PhysUniverse.h>
#include <Golem/Phys/NxCooking.h>
#include <Golem/Phys/NxStream.h>
#include <Golem/Phys/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void PhysScene::Desc::load(const golem::XMLContext* xmlcontext) {
	Scene::Desc::load(xmlcontext);

	XMLData(physics, xmlcontext->getContextFirst("simulation"), false);
}

//------------------------------------------------------------------------------

PhysScene::PhysScene(Universe &universe) : Scene(universe), universe(NULL) {
	this->universe = dynamic_cast<PhysUniverse*>(&universe);
	if (!this->universe)
		throw MsgPhysScene(Message::LEVEL_CRIT, "PhysScene::PhysScene(): only PhysUniverse can create PhysScene");
}

void PhysScene::create(const Desc& desc) {
	Scene::create(desc);

	{
		// Access to PhysX requires CS, despite the object is not activated yet
		CriticalSectionWrapper csw(universe->getCS());

		pNxPhysicsSDK = universe->pNxPhysicsSDK;

		pNxScene = pNxPhysicsSDK->createScene(desc.physics.nxSceneDesc);
		if (pNxScene == NULL)
			throw MsgPhysSceneNxSceneCreate(Message::LEVEL_CRIT, "PhysScene::create(): Unable to create NxScene");

		// Set simulation parameters
		pNxScene->setTiming((NxReal)universe->maxTimeStep);

		NxMaterial* defaultMaterial = pNxScene->getMaterialFromIndex(0);
		defaultMaterial->setRestitution((NxReal)desc.physics.restitution);
		defaultMaterial->setStaticFriction((NxReal)desc.physics.staticFriction);
		defaultMaterial->setDynamicFriction((NxReal)desc.physics.dynamicFriction);
	}
}

void PhysScene::release() {
	Scene::release();

	if (pNxScene != NULL) {
		// main thread
		CriticalSectionWrapper csw(universe->getCS());

		for (NxShapeDescMap::iterator i = nxShapeDescMap.begin(); i != nxShapeDescMap.end(); ++i)
			releaseNxShapeDescResources(i->second); // Access to PhysX
		nxShapeDescMap.clear();

		pNxPhysicsSDK->releaseScene(*pNxScene);
		pNxScene = NULL;
	}

	boundsDescMap.clear();
}

//------------------------------------------------------------------------------

PhysScene::NxShapeDescPtr PhysScene::createNxShapeDesc(const BoundingPlane::Desc* pDesc) const {
	ASSERT(pDesc != NULL)
	NxShapeDescPtr nxShapeDescPtr;
	NxPlaneShapeDesc *pNxShapeDesc = new NxPlaneShapeDesc;
	nxShapeDescPtr.reset(pNxShapeDesc);

	//pNxShapeDesc->localPose.M.setRowMajor(&pDesc->pose.R.m11);
	//pNxShapeDesc->localPose.t.set(&pDesc->pose.p.v1);
	// Ignore NxSphereShape local pose (it is not used by Novodex)
	pNxShapeDesc->normal.set(&pDesc->normal.v1);
	pNxShapeDesc->d = (NxReal)pDesc->distance;

	return nxShapeDescPtr;
}

Bounds::Desc::Ptr PhysScene::createBoundsDesc(const NxPlaneShapeDesc *pDesc) const {
	ASSERT(pDesc != NULL)
	Bounds::Desc::Ptr pBoundsDesc;
	BoundingPlane::Desc *pBoundingPlaneDesc = new BoundingPlane::Desc;
	pBoundsDesc.reset(pBoundingPlaneDesc);

	//pDesc->localPose.M.getRowMajor(&pBoundingPlaneDesc->pose.R.m11);
	//pDesc->localPose.t.get(&pBoundingPlaneDesc->pose.p.v1);
	// Ignore NxSphereShape local pose (it is not used by Novodex)
	pBoundingPlaneDesc->normal.setColumn3(&pDesc->normal.x);
	pBoundingPlaneDesc->distance = (Real)pDesc->d;

	return pBoundsDesc;
}

PhysScene::NxShapeDescPtr PhysScene::createNxShapeDesc(const BoundingSphere::Desc* pDesc) const {
	ASSERT(pDesc != NULL)
	NxShapeDescPtr nxShapeDescPtr;
	NxSphereShapeDesc *pNxShapeDesc = new NxSphereShapeDesc;
	nxShapeDescPtr.reset(pNxShapeDesc);

	pNxShapeDesc->localPose.M.setRowMajor(&pDesc->pose.R.m11);
	pNxShapeDesc->localPose.t.set(&pDesc->pose.p.v1);
	pNxShapeDesc->radius = (NxReal)pDesc->radius;

	return nxShapeDescPtr;
}

Bounds::Desc::Ptr PhysScene::createBoundsDesc(const NxSphereShapeDesc *pDesc) const {
	ASSERT(pDesc != NULL)
	Bounds::Desc::Ptr pBoundsDesc;
	BoundingSphere::Desc *pBoundingSphereDesc = new BoundingSphere::Desc;
	pBoundsDesc.reset(pBoundingSphereDesc);

	pDesc->localPose.M.getRowMajor(&pBoundingSphereDesc->pose.R.m11);
	pDesc->localPose.t.get(&pBoundingSphereDesc->pose.p.v1);
	pBoundingSphereDesc->radius = (Real)pDesc->radius;

	return pBoundsDesc;
}

PhysScene::NxShapeDescPtr PhysScene::createNxShapeDesc(const BoundingBox::Desc* pDesc) const {
	ASSERT(pDesc != NULL)
	NxShapeDescPtr nxShapeDescPtr;
	NxBoxShapeDesc *pNxShapeDesc = new NxBoxShapeDesc;
	nxShapeDescPtr.reset(pNxShapeDesc);

	pNxShapeDesc->localPose.M.setRowMajor(&pDesc->pose.R.m11);
	pNxShapeDesc->localPose.t.set(&pDesc->pose.p.v1);
	pNxShapeDesc->dimensions.set(&pDesc->dimensions.v1);

	return nxShapeDescPtr;
}

Bounds::Desc::Ptr PhysScene::createBoundsDesc(const NxBoxShapeDesc *pDesc) const {
	ASSERT(pDesc != NULL)
	Bounds::Desc::Ptr pBoundsDesc;
	BoundingBox::Desc *pBoundingBoxDesc = new BoundingBox::Desc;
	pBoundsDesc.reset(pBoundingBoxDesc);

	pDesc->localPose.M.getRowMajor(&pBoundingBoxDesc->pose.R.m11);
	pDesc->localPose.t.get(&pBoundingBoxDesc->pose.p.v1);
	pDesc->dimensions.get(&pBoundingBoxDesc->dimensions.v1);

	return pBoundsDesc;
}

PhysScene::NxShapeDescPtr PhysScene::createNxShapeDesc(const BoundingCylinder::Desc* pDesc) {
	ASSERT(pDesc != NULL)
	NxShapeDescPtr nxShapeDescPtr;

	// Create corresponding triangle mesh
	TriangleMesh mesh;
	pDesc->createTriangleMesh(mesh); // throws

	// and BoundingConvexMesh description
	BoundingConvexMesh::Desc desc;
	desc.vertices = mesh.vertices;
	desc.triangles = mesh.triangles;
	desc.pose = pDesc->pose;
	desc.bCook = true;//false;

	nxShapeDescPtr = createNxShapeDesc(&desc);
	return nxShapeDescPtr;
}

void PhysScene::copyShapeDesc(const ::NxConvexMeshDesc& src, BoundingConvexMesh::Desc& dst) const {
	if ((NxU32)dst.vertices.size() != src.numVertices) {
		dst.vertices.resize(src.numVertices);
	}
	// TODO recognize FP precision
	for (U32 i = 0; i < (U32)src.numVertices; ++i)
		dst.vertices[i].setColumn3(&((const NxReal*)src.points)[3*i]);

	if ((NxU32)dst.triangles.size() != src.numTriangles) {
		dst.triangles.resize(src.numTriangles);
	}
	// TODO recognize index size
	for (U32 i = 0; i < (U32)src.numTriangles; ++i)
		dst.triangles[i].set(&((const NxU32*)src.triangles)[3*i]);

	dst.bCook = (src.flags & NX_CF_COMPUTE_CONVEX) > 0;
}

PhysScene::NxShapeDescPtr PhysScene::createNxShapeDesc(BoundingConvexMesh::Desc* pDesc) {
	ASSERT(pDesc != NULL)
	NxShapeDescPtr nxShapeDescPtr;
	NxConvexMeshDesc nxConvexMeshDesc;

	nxConvexMeshDesc.numVertices = (NxU32)pDesc->vertices.size();
	// For some reason Ageia is unable to handle double precision floating point vectors
	std::vector<NxVec3> nxVertices(nxConvexMeshDesc.numVertices);
	for (U32 i = 0; i < (U32)nxConvexMeshDesc.numVertices; ++i)
		pDesc->vertices[i].getColumn3(&nxVertices[i].x);
	nxConvexMeshDesc.points = &nxVertices.front();
	nxConvexMeshDesc.pointStrideBytes = (NxU32)sizeof(NxVec3);

	if (pDesc->bCook) {
		nxConvexMeshDesc.flags |= NX_CF_COMPUTE_CONVEX;
	}
	else {
		nxConvexMeshDesc.numTriangles = (NxU32)pDesc->triangles.size();
		nxConvexMeshDesc.triangles = &pDesc->triangles.front();
		nxConvexMeshDesc.triangleStrideBytes = (NxU32)sizeof(Triangle);
	}

	// main thread
	CriticalSectionWrapper csw(universe->getCS());

	::MemoryWriteBuffer writeBufffer;
	if (!::CookConvexMesh(nxConvexMeshDesc, writeBufffer)) {
		context.error("PhysScene::createNxShapeDesc(): Unable to cook the convex mesh\n");
		return nxShapeDescPtr;
	}

	NxConvexShapeDesc *pNxShapeDesc = new NxConvexShapeDesc;
	nxShapeDescPtr.reset(pNxShapeDesc);

	pNxShapeDesc->localPose.M.setRowMajor(&pDesc->pose.R.m11);
	pNxShapeDesc->localPose.t.set(&pDesc->pose.p.v1);

	::MemoryReadBuffer readBufffer(writeBufffer.data);
	pNxShapeDesc->meshData = pNxPhysicsSDK->createConvexMesh(readBufffer);
	if (pNxShapeDesc->meshData == NULL) {
		context.error("PhysScene::createNxShapeDesc(): Unable to create the convex mesh\n");
		nxShapeDescPtr.release();
		return nxShapeDescPtr;
	}

	if (pDesc->bCook) {
		NxConvexMeshDesc nxConvexMeshDesc;
		if (!pNxShapeDesc->meshData->saveToDesc(nxConvexMeshDesc)) {
			context.error("PhysScene::createNxShapeDesc(): Unable to save Novodex convex mesh description\n");
			nxShapeDescPtr.release();
			return nxShapeDescPtr;
		}

		//const U32* pt = (const U32*)nxConvexMeshDesc.triangles;
		//for (size_t i = 0; i < nxConvexMeshDesc.numTriangles; ++i)
		//	printf("(%i, %i, %i)\n", pt[3*i+0], pt[3*i+1], pt[3*i+2]);

		copyShapeDesc(nxConvexMeshDesc, *pDesc);
	}

	return nxShapeDescPtr;
}

Bounds::Desc::Ptr PhysScene::createBoundsDesc(const NxConvexShapeDesc *pDesc) {
	ASSERT(pDesc != NULL)
	Bounds::Desc::Ptr pBoundsDesc;
	BoundingConvexMesh::Desc *pBoundingConvexMeshDesc = new BoundingConvexMesh::Desc;
	pBoundsDesc.reset(pBoundingConvexMeshDesc);

	// main thread
	CriticalSectionWrapper csw(universe->getCS());

	NxConvexMeshDesc nxConvexMeshDesc;
	if (pDesc->meshData == NULL || !pDesc->meshData->saveToDesc(nxConvexMeshDesc)) {
		context.error("PhysScene::createBoundsDesc(): Unable to save Novodex convex mesh description\n");
		pBoundsDesc.release();
		return pBoundsDesc;
	}

	copyShapeDesc(nxConvexMeshDesc, *pBoundingConvexMeshDesc);

	return pBoundsDesc;
}

void PhysScene::releaseNxShapeDescResources(NxShapeDescPtr& nxShapeDescPtr) {
	NxConvexShapeDesc *pNxConvexShapeDesc = dynamic_cast<NxConvexShapeDesc*>(nxShapeDescPtr.get());
	if (pNxConvexShapeDesc != NULL && pNxConvexShapeDesc->meshData != NULL) {
		pNxPhysicsSDK->releaseConvexMesh(*pNxConvexShapeDesc->meshData);
	}
}

//------------------------------------------------------------------------------

void PhysScene::initOpenGL() {
	Scene::initOpenGL();

	// Debug data visualization
	const Real visualizationScaleInv = REAL_ONE / universe->simulationScale;
	pNxPhysicsSDK->setParameter(NX_VISUALIZATION_SCALE, NxReal(visualizationScaleInv)); // secure use of draw.debug
	//pNxPhysicsSDK->setParameter(NX_VISUALIZE_WORLD_AXES, (NxReal)(1.0));
	//pNxPhysicsSDK->setParameter(NX_VISUALIZE_BODY_AXES, (NxReal)1.0);
	pNxPhysicsSDK->setParameter(NX_VISUALIZE_ACTOR_AXES, (NxReal)1.0);
	//pNxPhysicsSDK->setParameter(NX_VISUALIZE_CONTACT_FORCE, (NxReal)1.0);
	//pNxPhysicsSDK->setParameter(NX_VISUALIZE_JOINT_LOCAL_AXES, (NxReal)1.0);
	//pNxPhysicsSDK->setParameter(NX_VISUALIZE_JOINT_WORLD_AXES, (NxReal)1.0);
	//pNxPhysicsSDK->setParameter(NX_VISUALIZE_JOINT_LIMITS, (NxReal)1.0);
}

void PhysScene::render() const {
	Scene::render();

	if (openGL.draw & OpenGL::DRAW_DEBUG) { // csOpenGL is always locked here
		if (openGL.draw & OpenGL::DRAW_DEBUG_SIMULATION)
			nxDebugRenderer.renderData(*pNxScene->getDebugRenderable()); // always pNxScene != NULL and always called in the same thread as PhysX
	}
}

//------------------------------------------------------------------------------

Actor::Desc::Ptr PhysScene::createActorDesc() const {
	return Actor::Desc::Ptr(new PhysActor::Desc);
}

//------------------------------------------------------------------------------

/** Creates Novodex shape description from bounds description. */
NxShapeDesc* PhysScene::createNxShapeDesc(Bounds::Desc::Ptr pBoundsDesc) {
	// check if shape description has been already created
	NxShapeDescMap::iterator pos = nxShapeDescMap.find(pBoundsDesc.get());
	if (pos != nxShapeDescMap.end())
		return pos->second.get();

	// check if description is valid
	if (!pBoundsDesc->isValid())
		throw MsgSceneBoundsDescInvalidDesc(Message::LEVEL_ERROR, "PhysScene::createNxShapeDesc(): Invalid bounds description");

	NxShapeDescPtr nxShapeDescPtr;

	switch (pBoundsDesc->getType()) {
	case Bounds::TYPE_PLANE:
		nxShapeDescPtr = createNxShapeDesc(dynamic_cast<const BoundingPlane::Desc*>(pBoundsDesc.get()));
		break;
	case Bounds::TYPE_SPHERE:
		nxShapeDescPtr = createNxShapeDesc(dynamic_cast<const BoundingSphere::Desc*>(pBoundsDesc.get()));
		break;
	case Bounds::TYPE_CYLINDER:
		nxShapeDescPtr = createNxShapeDesc(dynamic_cast<const BoundingCylinder::Desc*>(pBoundsDesc.get()));
		break;
	case Bounds::TYPE_BOX:
		nxShapeDescPtr = createNxShapeDesc(dynamic_cast<const BoundingBox::Desc*>(pBoundsDesc.get()));
		break;
	case Bounds::TYPE_CONVEX_MESH:
		nxShapeDescPtr = createNxShapeDesc(dynamic_cast<BoundingConvexMesh::Desc*>(pBoundsDesc.get())); // Access to PhysX
		break;
	default:
		ASSERT(false)
		break;
	}

	if (nxShapeDescPtr == NULL)
		throw MsgSceneBoundsDescCreate(Message::LEVEL_ERROR, "PhysScene::createNxShapeDesc(): Unable to create bounds description");

	nxShapeDescMap[pBoundsDesc.get()] = nxShapeDescPtr;
	boundsDescMap[nxShapeDescPtr.get()] = pBoundsDesc;

	return nxShapeDescPtr.get();
}

/** Releases the Novodex shape description. */
void PhysScene::releaseNxShapeDesc(NxShapeDesc& nxShapeDesc) {
	BoundsDescMap::iterator pos1 = boundsDescMap.find(&nxShapeDesc);
	if (pos1 == boundsDescMap.end())
		throw MsgPhysSceneNxShapeDescRelease(Message::LEVEL_ERROR, "PhysScene::releaseNxShapeDesc(): Unable to to find the specified shape description\n");

	// main thread
	CriticalSectionWrapper csw(universe->getCS());

	NxShapeDescMap::iterator pos2 = nxShapeDescMap.find(pos1->second.get());
	releaseNxShapeDescResources(pos2->second); // Access to PhysX (for ConvexMesh)
	boundsDescMap.erase(pos1);
	nxShapeDescMap.erase(pos2);
}

/** Returns bounds description associated with the Novodex shape description. */
Bounds::Desc::Ptr PhysScene::createBoundsDesc(const NxShapeDesc &nxShapeDesc) {
	// check if bound description has been already created
	BoundsDescMap::const_iterator pos = boundsDescMap.find(&nxShapeDesc);
	if (pos != boundsDescMap.end())
		return pos->second;

	Bounds::Desc::Ptr pBoundsDesc;

	// check if description is valid
	if (!nxShapeDesc.isValid())
		throw MsgPhysSceneNxShapeDescInvalidDesc(Message::LEVEL_ERROR, "PhysScene::createBoundsDesc(): invalid Novodex shape description");

	NxShapeDescPtr nxShapeDescPtr;

	if (nxShapeDesc.getType() == NX_SHAPE_PLANE) {
		const NxPlaneShapeDesc *pDescSrc = dynamic_cast<const NxPlaneShapeDesc*>(&nxShapeDesc);
		if (pDescSrc != NULL) {
			NxPlaneShapeDesc *pDescDst = new NxPlaneShapeDesc();
			nxShapeDescPtr.reset(pDescDst);
			::memcpy(pDescDst, pDescSrc, sizeof(NxPlaneShapeDesc));
			pBoundsDesc = createBoundsDesc(pDescDst);
		}
	}
	else if (nxShapeDesc.getType() == NX_SHAPE_SPHERE) {
		const NxSphereShapeDesc *pDescSrc = dynamic_cast<const NxSphereShapeDesc*>(&nxShapeDesc);
		if (pDescSrc != NULL) {
			NxSphereShapeDesc *pDescDst = new NxSphereShapeDesc();
			nxShapeDescPtr.reset(pDescDst);
			::memcpy(pDescDst, pDescSrc, sizeof(NxSphereShapeDesc));
			pBoundsDesc = createBoundsDesc(pDescDst);
		}
	}
	else if (nxShapeDesc.getType() == NX_SHAPE_BOX) {
		const NxBoxShapeDesc *pDescSrc = dynamic_cast<const NxBoxShapeDesc*>(&nxShapeDesc);
		if (pDescSrc != NULL) {
			NxBoxShapeDesc *pDescDst = new NxBoxShapeDesc();
			nxShapeDescPtr.reset(pDescDst);
			::memcpy(pDescDst, pDescSrc, sizeof(NxBoxShapeDesc));
			pBoundsDesc = createBoundsDesc(pDescDst);
		}
	}
	else if (nxShapeDesc.getType() == NX_SHAPE_CONVEX) {
		const NxConvexShapeDesc *pDescSrc = dynamic_cast<const NxConvexShapeDesc*>(&nxShapeDesc);
		if (pDescSrc != NULL) {
			NxConvexShapeDesc *pDescDst = new NxConvexShapeDesc();
			nxShapeDescPtr.reset(pDescDst);
			::memcpy(pDescDst, pDescSrc, sizeof(NxConvexShapeDesc));
			pBoundsDesc = createBoundsDesc(pDescDst); // Access to PhysX
		}
	}
	else {
		ASSERT(false)
	}

	if (pBoundsDesc == NULL)
		throw MsgSceneBoundsDescCreate(Message::LEVEL_ERROR, "PhysScene::createBoundsDesc(): failed to create bounds description");

	nxShapeDescMap[pBoundsDesc.get()] = nxShapeDescPtr;
	boundsDescMap[nxShapeDescPtr.get()] = pBoundsDesc;

	return pBoundsDesc;
}

/** Releases the bounds description and related Novodex shape description. */
void PhysScene::releaseBoundsDesc(Bounds::Desc::Ptr pBoundsDesc) {
	NxShapeDescMap::iterator pos1 = nxShapeDescMap.find(pBoundsDesc.get());
	if (pos1 == nxShapeDescMap.end())
		throw MsgPhysSceneNxShapeDescRelease(Message::LEVEL_ERROR, "PhysScene::releaseBoundsDesc(): Unable to to find the specified bouds description\n");

	// main thread
	CriticalSectionWrapper csw(universe->getCS());

	BoundsDescMap::iterator pos2 = boundsDescMap.find(pos1->second.get());
	releaseNxShapeDescResources(pos1->second); // Access to PhysX
	nxShapeDescMap.erase(pos1);
	boundsDescMap.erase(pos2);
}

//------------------------------------------------------------------------------

