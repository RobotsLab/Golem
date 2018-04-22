/** @file TinyI.cpp
 * 
 * Implementation of Golem Tiny Ice server (source file).
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sys/Context.h>
#include <Golem/App/UIPlanner.h>
#include <Golem/Phys/PhysUniverse.h>
#include <Golem/Sys/XMLParser.h>
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/App/Data.h>
#include <Golem/Phys/Data.h>
#include <Golem/Ctrl/Katana/Data.h>
#include <Golem/TinyIce/TinyIceI.h>
#include <Golem/TinyIce/Types.h>
#include <IceUtil/UUID.h>

using namespace golem::tinyice;

//------------------------------------------------------------------------------

class golem::tinyice::BoundsDesc {
public:
	golem::Bounds::Desc::Ptr pBoundsDesc;
	BoundsDesc(const golem::Bounds::Desc::Ptr pBoundsDesc) : pBoundsDesc(pBoundsDesc) {}
};

class golem::tinyice::UIPlannerDesc : public golem::UIPlanner::Desc {
public:
};

//------------------------------------------------------------------------------

ShapeI::ShapeI(RigidBodyI &rigidBody) : rigidBody(rigidBody), adapter(rigidBody.adapter), pNxShapeDesc(NULL), pBounds(NULL), owner(false) {
	id.name = IceUtil::generateUUID(); 
}

ShapeI::~ShapeI() {
	if (pBounds == NULL || !owner)
		return;
	rigidBody.pActor->releaseBounds(*pBounds);
}

void ShapeI::create(const ShapeDesc& desc, ShapeType type, const golem::shared_ptr<BoundsDesc>& pBoundsDesc) {
	this->type = type;
	this->density = desc.density;
	this->color = desc.color;

	// pBoundsDesc must be initialized before
	if (pBoundsDesc == NULL)
		throw ExTinyShape("ShapeI::create(): Unable to create bounds description");
	
	this->pBoundsDesc = pBoundsDesc;

	PhysScene* const pPhysScene = static_cast<PhysScene*>(&rigidBody.scene);
	if (!pPhysScene)
		throw ExTinyShapeCreate("ShapeI::create(): Unable to cast to PhysScene");

	pNxShapeDesc = pPhysScene->createNxShapeDesc(this->pBoundsDesc->pBoundsDesc); // throws
	pNxShapeDesc->density = NxReal(this->density);
	// pBounds created in container
}

ShapePrx ShapeI::activate() {
	return ShapePrx::uncheckedCast(adapter->add(this, id));
}

void ShapeI::deactivate() {
	adapter->remove(id);
}

ShapeType ShapeI::getType(const ::Ice::Current&) const {
	return type;
}

RGBA ShapeI::getColor(const ::Ice::Current&) const {
	return color;
}

Mat34 ShapeI::getLocalPose(const ::Ice::Current&) const {
	return make(pBounds->getPose());
}

::Ice::Int ShapeI::getGroup(const ::Ice::Current&) const {
	return (::Ice::Int)pBounds->getGroup();
}

void ShapeI::setGroup(::Ice::Int group, const ::Ice::Current&) {
	rigidBody.pActor->setBoundsGroup(*pBounds, U32(group));
}

ShapeDescPtr ShapeI::getDesc(const ::Ice::Current&) const {
	return pShapeDesc;
}

//------------------------------------------------------------------------------

PlaneShapeI::PlaneShapeI(RigidBodyI &rigidBody) : ShapeI(rigidBody) {
}
void PlaneShapeI::create(const PlaneShapeDesc& desc) {
	BoundingPlane::Desc* pDesc = new BoundingPlane::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = make(desc.localPose);
	pDesc->normal = make(desc.normal);
	pDesc->distance = (Real)desc.distance;
	ShapeI::create(desc, ShapeTypePlane, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc)));
}

SphereShapeI::SphereShapeI(RigidBodyI &rigidBody) : ShapeI(rigidBody) {
}
void SphereShapeI::create(const SphereShapeDesc& desc) {
	BoundingSphere::Desc* pDesc = new BoundingSphere::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = make(desc.localPose);
	pDesc->radius = (Real)desc.radius;
	ShapeI::create(desc, ShapeTypeSphere, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc)));
}

CylinderShapeI::CylinderShapeI(RigidBodyI &rigidBody) : ShapeI(rigidBody) {
}
void CylinderShapeI::create(const CylinderShapeDesc& desc) {
	BoundingCylinder::Desc* pDesc = new BoundingCylinder::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = make(desc.localPose);
	pDesc->radius = (Real)desc.radius;
	pDesc->length = (Real)desc.length;
	ShapeI::create(desc, ShapeTypeCylinder, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc)));
}

BoxShapeI::BoxShapeI(RigidBodyI &rigidBody) : ShapeI(rigidBody) {
}
void BoxShapeI::create(const BoxShapeDesc& desc) {
	BoundingBox::Desc* pDesc = new BoundingBox::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = make(desc.localPose);
	pDesc->dimensions = make(desc.dimensions);
	ShapeI::create(desc, ShapeTypeBox, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc)));
}

ConvexMeshShapeI::ConvexMeshShapeI(RigidBodyI &rigidBody) : ShapeI(rigidBody) {
}
void ConvexMeshShapeI::create(const ConvexMeshShapeDesc& desc) {
	BoundingConvexMesh::Desc* pDesc = new BoundingConvexMesh::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = make(desc.localPose);
	for (::golem::tinyice::Vec3Seq::const_iterator i = desc.vertices.begin(); i != desc.vertices.end(); ++i)
		pDesc->vertices.push_back(make(*i));
	pDesc->bCook = true;
	ShapeI::create(desc, ShapeTypeConvexMesh, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc)));
}

//------------------------------------------------------------------------------

ActorI::ActorI(TinyI &tiny) : tiny(tiny), adapter(tiny.adapter), context(*tiny.pContext), scene(*tiny.pScene), pXMLContext(tiny.pXMLContext), owner(false) {
	id.name = IceUtil::generateUUID(); 
}

ActorI::~ActorI() {
}

ActorPrx ActorI::activate() {
	return ActorPrx::uncheckedCast(adapter->add(this, id));
}

void ActorI::deactivate() {
	adapter->remove(id);
}

RigidBodyI::RigidBodyI(TinyI &tiny) : ActorI(tiny), pActor(NULL) {
}

RigidBodyI::~RigidBodyI() {
	shapeList.clear(); // release shapes before releasing the Actor
	if (pActor == NULL || !owner)
		return;
	scene.releaseObject(*pActor);
}

ShapeIPtr RigidBodyI::createShape(const ShapeDesc* pDesc) {
	if (!pDesc)
		throw ExTinyShapeCreate("RigidBodyI::createShape(): NULL Shape description");
	
	ShapeIPtr pShape;

	if (dynamic_cast<const PlaneShapeDesc*>(pDesc)) {
		PlaneShapeI* pPlaneShape = new PlaneShapeI(*this);
		pShape = pPlaneShape;
		pPlaneShape->create(*dynamic_cast<const PlaneShapeDesc*>(pDesc));
	}
	else if (dynamic_cast<const SphereShapeDesc*>(pDesc)) {
		SphereShapeI* pSphereShape = new SphereShapeI(*this);
		pShape = pSphereShape;
		pSphereShape->create(*dynamic_cast<const SphereShapeDesc*>(pDesc));
	}
	else if (dynamic_cast<const CylinderShapeDesc*>(pDesc)) {
		CylinderShapeI* pCylinderShape = new CylinderShapeI(*this);
		pShape = pCylinderShape;
		pCylinderShape->create(*dynamic_cast<const CylinderShapeDesc*>(pDesc));
	}
	else if (dynamic_cast<const BoxShapeDesc*>(pDesc)) {
		BoxShapeI* pBoxShape = new BoxShapeI(*this);
		pShape = pBoxShape;
		pBoxShape->create(*dynamic_cast<const BoxShapeDesc*>(pDesc));
	}
	else if (dynamic_cast<const ConvexMeshShapeDesc*>(pDesc)) {
		ConvexMeshShapeI* pConvexMeshShape = new ConvexMeshShapeI(*this);
		pShape = pConvexMeshShape;
		pConvexMeshShape->create(*dynamic_cast<const ConvexMeshShapeDesc*>(pDesc));
	}
	else
		throw ExTinyShapeCreate("RigidBodyI::createShape(): Unknown shape description");

	return pShape;
}

ShapeIPtr RigidBodyI::createShape(const golem::Bounds* pBounds) {
	ShapeDescPtr pShapeDesc;
	ShapeIPtr pShape;

	switch (pBounds->getType()) {
	case Bounds::TYPE_PLANE:
		{
			PlaneShapeDesc* pDesc = new PlaneShapeDesc;
			pShapeDesc = pDesc;
			pDesc->normal = make(static_cast<const BoundingPlane*>(pBounds)->getNormal());
			pDesc->distance = (Ice::Double)static_cast<const BoundingPlane*>(pBounds)->getDistance();
		}
		break;
	case Bounds::TYPE_SPHERE:
		{
			SphereShapeDesc* pDesc = new SphereShapeDesc;
			pShapeDesc = pDesc;
			pDesc->radius = (Ice::Double)static_cast<const BoundingSphere*>(pBounds)->getRadius();
		}
		break;
	case Bounds::TYPE_CYLINDER:
		{
			CylinderShapeDesc* pDesc = new CylinderShapeDesc;
			pShapeDesc = pDesc;
			pDesc->radius = (Ice::Double)static_cast<const BoundingCylinder*>(pBounds)->getRadius();
			pDesc->length = (Ice::Double)static_cast<const BoundingCylinder*>(pBounds)->getLength();
		}
		break;
	case Bounds::TYPE_BOX:
		{
			BoxShapeDesc* pDesc = new BoxShapeDesc;
			pShapeDesc = pDesc;
			pDesc->dimensions = make(static_cast<const BoundingBox*>(pBounds)->getDimensions());
		}
		break;
	case Bounds::TYPE_CONVEX_MESH:
		{
			ConvexMeshShapeDesc* pDesc = new ConvexMeshShapeDesc;
			pShapeDesc = pDesc;
			const BoundingConvexMesh* pBoundingConvexMesh = static_cast<const BoundingConvexMesh*>(pBounds);
			for (std::vector<golem::Vec3>::const_iterator i = pBoundingConvexMesh->getVertices().begin(); i != pBoundingConvexMesh->getVertices().end(); ++i)
				pDesc->vertices.push_back(make(*i));
		}
		break;
	default:
		throw ExTinyShapeCreate("RigidBodyI::createShape(): Unknown shape type");
	}

	pShapeDesc->localPose = make(pBounds->getPose());
	pShapeDesc->group = (Ice::Int)pBounds->getGroup();

	pShape = createShape(pShapeDesc.get());// throws
	pShape->pShapeDesc = pShapeDesc;
	pShape->pBounds = pBounds;
	
	return pShape;
}

void RigidBodyI::create(const RigidBodyDesc& desc) {
	kinematic = desc.kinematic;

	golem::PhysActor::Desc actorDesc;
	actorDesc.kinematic = desc.kinematic;
	actorDesc.pose = make(desc.globalPose);

	shapeList.clear();
	for (ShapeDescSeq::const_iterator i = desc.shapes.begin(); i != desc.shapes.end(); ++i) {
		ShapeIPtr pShape = createShape((*i).get()); // throws
		pShape->pShapeDesc = *i;

		if (pShape->getType() == ShapeTypePlane)
			actorDesc.nxActorDesc.body = NULL; // Actors with Plane Shapes cannot have a body
		actorDesc.nxActorDesc.shapes.push_back(pShape->pNxShapeDesc);
		
		ShapePrx shapePrx = pShape->activate();
		shapeList.push_back(ShapeList::Pair(shapePrx, pShape));
	}

	pActor = dynamic_cast<golem::Actor*>(scene.createObject(actorDesc)); // throws
	if (pActor == NULL)
		throw ExTinyShapeCreate("RigidBodyI::create(): Unable to cast to Actor");

	const golem::Actor::BoundsConstSeq& boundsSeq = pActor->getBoundsSeq();
	golem::Actor::BoundsConstSeq::const_iterator i = boundsSeq.begin();
	ShapeList::iterator j = shapeList.begin();
	while (i != boundsSeq.end() && j != shapeList.end())
		(j++)->second->pBounds = *i++; // order is preserved
}

ActorPrx RigidBodyI::activate() {
	return ActorI::activate();
}

void RigidBodyI::deactivate() {
	ActorI::deactivate();
	
	// deactivate shapes
	for (ShapeList::iterator i = shapeList.begin(); i != shapeList.end(); ++i)
		i->second->deactivate();
}

ShapePrx RigidBodyI::createShape(const ::golem::tinyice::ShapeDescPtr& pShapeDesc, const ::Ice::Current&) {
	try {
		if (pShapeDesc.get() == NULL)
			throw ExTinyShapeCreate("RigidBodyI::createShape(): empty description");

		ShapeIPtr pShape = createShape(pShapeDesc.get()); // throws
		pShape->pShapeDesc = pShapeDesc;
		
		pShape->pBounds = pActor->createBounds(pShape->pBoundsDesc->pBoundsDesc); // throws
		pShape->owner = true;

		ShapePrx shapePrx = pShape->activate();
		shapeList.push_back(ShapeList::Pair(shapePrx, pShape));
		return shapePrx;
	}
	catch (const Message& ex) {
		std::string str("RigidBodyI::createShape(): ");
		str.append(ex.what());
		throw ExTinyShapeCreate(str);
	}
	catch (const std::exception &ex) {
		std::string str("RigidBodyI::createShape(): C++ exception: ");
		str.append(ex.what());
		throw ExTinyShapeCreate(str);
	}
}

void RigidBodyI::releaseShape(const ShapePrx& shape, const ::Ice::Current&) {
	ShapeList::iterator pos = shapeList.find(shape);
	if (pos == shapeList.end())
		throw ExTinyShapeNotFound("RigidBodyI::releaseShape(): Unable to find specified Shape");
	if (!pos->second->owner)
		throw ExTinyShapeNotRemovable("RigidBodyI::releaseShape(): The specified Shape cannot be removed");

	pos->second->deactivate();
	shapeList.erase(pos);
}

ShapeSeq RigidBodyI::getShapes(const ::Ice::Current&) const {
	ShapeSeq shapeSeq;
	for (ShapeList::const_iterator i = shapeList.begin(); i != shapeList.end(); ++i)
		shapeSeq.push_back(i->first);
	return shapeSeq;
}

Mat34 RigidBodyI::getGlobalPose(const ::Ice::Current&) const {
	return make(pActor->getPose());
}

void RigidBodyI::setGlobalPose(const Mat34& pose, const ::Ice::Current&) {
	pActor->setPose(make(pose));
}

void RigidBodyI::setGroup(::Ice::Int group, const ::Ice::Current&) {
	pActor->setBoundsGroup(U32(group));
}

//------------------------------------------------------------------------------

JointI::JointI(ControllerI &armI) : RigidBodyI(armI.tiny), armI(armI) {
}

void JointI::create(const JointDesc& desc, golem::BodyActor* pJointActor) {
	pActor = pJointActor;
	kinematic = true;
	
	shapeList.clear();
	const golem::Actor::BoundsConstSeq& boundsSeq = pActor->getBoundsSeq();
	for (golem::Actor::BoundsConstSeq::const_iterator i = boundsSeq.begin(); i != boundsSeq.end(); ++i) {
		ShapeIPtr pShape = RigidBodyI::createShape(*i); // throws
		ShapePrx shapePrx = pShape->activate();
		shapeList.push_back(ShapeList::Pair(shapePrx, pShape));
	}
}

ControllerI::ControllerI(TinyI &tiny) : ActorI(tiny), pUIPlanner(NULL) {
}

ControllerI::~ControllerI() {
	jointList.clear(); // joints wrappers must be released before UIPlanner!
	if (pUIPlanner == NULL || !owner)
		return;
	scene.releaseObject(*pUIPlanner);
}

JointIPtr ControllerI::createJoint(const JointDesc* pDesc, golem::BodyActor* pJointActor) {
	JointIPtr pJoint;
	
	if (dynamic_cast<const JointDesc*>(pDesc)) {
		JointI* pJointI = new JointI(*this);
		pJoint = pJointI;
		pJointI->create(*static_cast<const JointDesc*>(pDesc), pJointActor); // throws
	}
	else
		throw ExTinyActorCreate("ControllerI::createJoint(): Unknown Joint description");
	
	return pJoint;
}

void ControllerI::create(const ControllerDesc& desc) {
	golem::tinyice::UIPlannerDesc uiPlannerDesc;
	// phys planner load from local xml context description
	golem::XMLData((golem::UIPlanner::Desc&)uiPlannerDesc, pXMLContext);
	// load controller description
	uiPlannerDesc.pUIControllerDesc->pControllerDesc = golem::Controller::Desc::load(&context, desc.libraryPathCtrl, desc.configPathCtrl);
	// overwrite default parameters
	uiPlannerDesc.pUIControllerDesc->pControllerDesc->globalPose = make(desc.globalPose);
	// load planner description
	uiPlannerDesc.plannerDescSeq.clear();
	uiPlannerDesc.plannerDescSeq.push_back(golem::Planner::Desc::load(&context, desc.libraryPathPlanner, desc.configPathPlanner));
	// create
	ControllerI::create(uiPlannerDesc); // throws
}

void ControllerI::create(const UIPlannerDesc& desc) {
	// create controller
	pUIPlanner = dynamic_cast<UIPlanner*>(scene.createObject(desc));// throws
	if (pUIPlanner == NULL)
		throw ExTinyController("ControllerI::create(): unable to cast to UIPlanner");

	// create joints
	const golem::Configspace::Range joints = pUIPlanner->getController().getStateInfo().getJoints();
	const golem::BodyActor::ConfigSeq& jointActorSeq = pUIPlanner->getUIController().getJointActors();

	jointList.clear();
	for (golem::Configspace::Index i = joints.begin(); i < joints.end(); ++i) {
		JointIPtr pJoint;
		if (jointActorSeq[i] != 0) { // Joint may have no body
			JointDescPtr pJointDesc(new JointDesc);
			pJoint = createJoint(pJointDesc.get(), jointActorSeq[i]); // throws
			ActorPrx actorPrx = pJoint->activate();
			jointList.push_back(JointList::Pair(actorPrx, pJoint));
		}
	}
}

ActorPrx ControllerI::activate() {
	return ActorI::activate();
}

void ControllerI::deactivate() {
	ActorI::deactivate();

	// deactivate joints
	for (JointList::iterator i = jointList.begin(); i != jointList.end(); ++i)
		i->second->deactivate();
}

GenConfigspaceState ControllerI::recvGenConfigspaceState(Ice::Double t, const ::Ice::Current&) {
	golem::Controller::State gcs = pUIPlanner->getController().createState();
	try {
		pUIPlanner->getController().lookupCommand(t, gcs);
	}
	catch (const Message&) {
		throw ExTinyControllerRecv("ControllerI::recvGenConfigspaceState(): lookup command error");
	}
	return make(pUIPlanner->getController().getStateInfo().getJoints(), gcs);
}

GenWorkspaceState ControllerI::recvGenWorkspaceState(Ice::Double t, const ::Ice::Current&) {
	golem::Controller::State gcs = pUIPlanner->getController().createState();
	try {
		pUIPlanner->getController().lookupCommand(t, gcs);
	}
	catch (const Message&) {
		throw ExTinyControllerRecv("ControllerI::recvGenWorkspaceState(): lookup command error");
	}
	golem::GenWorkspaceChainState gws;
	pUIPlanner->getController().chainForwardTransform(gcs.cpos, gws.wpos);
	const golem::Chainspace::Range chains = pUIPlanner->getController().getStateInfo().getChains();
	for (golem::Chainspace::Index i = chains.begin(); i < chains.end(); ++i)
		gws.wpos[i].multiply(gws.wpos[i], pUIPlanner->getController().getChains()[i]->getReferencePose()); // reference pose
	gws.t = gcs.t;
	return make(pUIPlanner->getController().getStateInfo().getChains(), gws);
}

void ControllerI::send(const GenConfigspaceStateSeq& trajectory, double timeWait, const ::Ice::Current&) {
	const golem::Controller::Trajectory tmp(makeSeq<golem::Controller::Trajectory>(pUIPlanner->getController(), trajectory));
	if (pUIPlanner->getController().send(&tmp.front(), &tmp.back() + 1) != &tmp.back() + 1)
		throw ExTinyControllerSend("ControllerI::sendTrajectory(): send trajectory error");

	(void)pUIPlanner->getController().waitForEnd(SecToMSec(timeWait));
}

void ControllerI::stop(const ::Ice::Current&) {
	pUIPlanner->getController().stop();
}

GenConfigspaceState ControllerI::findTarget(const GenConfigspaceState &begin, const GenWorkspaceState& end, const ::Ice::Current&) {
	golem::GenConfigspaceState cend;
	if (!pUIPlanner->getPlannerSeq()[0]->findTarget(make(pUIPlanner->getController(), begin), make(pUIPlanner->getController().getStateInfo().getChains(), end), cend))
		throw ExTinyControllerFindTarget("ControllerI::findTarget(): find target error");

	return make(pUIPlanner->getController().getStateInfo().getJoints(), cend);
}

GenConfigspaceStateSeq ControllerI::findGlobalTrajectory(const GenConfigspaceState &begin, const GenConfigspaceState &end, const ::Ice::Current&) {
	golem::Controller::Trajectory tmp;
	if (!pUIPlanner->getPlannerSeq()[0]->findGlobalTrajectory(make(pUIPlanner->getController(), begin), make(pUIPlanner->getController(), end), tmp, tmp.begin()))
		throw ExTinyControllerFindTrajectory("ControllerI::findGlobalTrajectory(): find trajectory error");

	return makeSeq<GenConfigspaceStateSeq>(tmp);
}

GenConfigspaceStateSeq ControllerI::findLocalTrajectory(const GenConfigspaceState &begin, const GenWorkspaceStateSeq &trajectory, const ::Ice::Current&) {
	golem::GenWorkspaceChainState::Seq inp = makeSeq<golem::GenWorkspaceChainState::Seq>(pUIPlanner->getController().getStateInfo().getChains(), trajectory);
	golem::Controller::Trajectory out;
	if (!pUIPlanner->getPlannerSeq()[0]->findLocalTrajectory(make(pUIPlanner->getController(), begin),
		inp.begin(), inp.end(),
		out, out.begin()))
		throw ExTinyControllerFindTrajectory("ControllerI::findLocalTrajectory(): find trajectory error");

	return makeSeq<GenConfigspaceStateSeq>(out);
}

::golem::tinyice::WorkspaceCoord ControllerI::getChainForwardTransform(const ::golem::tinyice::ConfigspaceCoord& cc, const ::Ice::Current&) const {
	golem::WorkspaceChainCoord tmp;
	pUIPlanner->getController().chainForwardTransform(make(pUIPlanner->getController().getStateInfo().getJoints(), cc), tmp);
	return make(pUIPlanner->getController().getStateInfo().getChains(), tmp);
}

::golem::tinyice::WorkspaceCoord ControllerI::getJointForwardTransform(const ::golem::tinyice::ConfigspaceCoord& cc, const ::Ice::Current&) const {
	golem::WorkspaceJointCoord tmp;
	pUIPlanner->getController().jointForwardTransform(make(pUIPlanner->getController().getStateInfo().getJoints(), cc), tmp);
	return make(pUIPlanner->getController().getStateInfo().getJoints(), tmp);
}

::golem::tinyice::Jacobian ControllerI::getJacobian(const ::golem::tinyice::ConfigspaceCoord& cc, const ::Ice::Current&) const {
	golem::Jacobian jacobian;
	pUIPlanner->getController().jacobian(make(pUIPlanner->getController().getStateInfo().getJoints(), cc), jacobian);
	return make(pUIPlanner->getController().getStateInfo().getJoints(), jacobian);
}

JointSeq ControllerI::getJoints(const ::Ice::Current&) const {
	JointSeq jointSeq;
	for (JointList::const_iterator i = jointList.begin(); i != jointList.end(); ++i)
		jointSeq.push_back(JointPrx::uncheckedCast(i->first));
	return jointSeq;
}

::Ice::Int ControllerI::getControllerGroup(const ::Ice::Current&) const {
	return (::Ice::Int)pUIPlanner->getControllerBoundsGroup();
}

::Ice::Int ControllerI::getCollisionGroup(const ::Ice::Current&) const {
	return (::Ice::Int)pUIPlanner->getCollisionBoundsGroup();
}

void ControllerI::setCollisionGroup(::Ice::Int group, const ::Ice::Current&) {
	pUIPlanner->setCollisionBoundsGroup((int)group);
}

Mat34 ControllerI::getGlobalPose(const ::Ice::Current&) const {
	return make(pUIPlanner->getController().getGlobalPose());
}

void ControllerI::setGlobalPose(const Mat34& pose, const ::Ice::Current&) {
	pUIPlanner->getController().setGlobalPose(make(pose));
}

WorkspaceCoord ControllerI::getReferencePose(const ::Ice::Current&) const {
	const golem::Chainspace::Range chains = pUIPlanner->getController().getStateInfo().getChains();
	golem::WorkspaceChainCoord tmp;
	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i)
		tmp[i] = pUIPlanner->getController().getChains()[i]->getReferencePose();
	return make(pUIPlanner->getController().getStateInfo().getChains(), tmp);
}

void ControllerI::setReferencePose(const WorkspaceCoord& pose, const ::Ice::Current&) {
	const golem::Chainspace::Range chains = pUIPlanner->getController().getStateInfo().getChains();
	const golem::WorkspaceChainCoord tmp = make(chains, pose);
	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i)
		pUIPlanner->getController().getChains()[i]->setReferencePose(tmp[i]);
}

//------------------------------------------------------------------------------

KatanaI::KatanaI(TinyI &tiny) : ControllerI(tiny) {
}

void KatanaI::create(const KatanaDesc& desc) {
	golem::tinyice::UIPlannerDesc uiPlannerDesc;
	// phys planner load from local xml context description
	golem::XMLData((golem::UIPlanner::Desc&)uiPlannerDesc, pXMLContext);
	// load controller description
	uiPlannerDesc.pUIControllerDesc->pControllerDesc = golem::Controller::Desc::load(&context, desc.libraryPathCtrl, desc.configPathCtrl);
	// overwrite default parameters
	uiPlannerDesc.pUIControllerDesc->pControllerDesc->globalPose = make(desc.globalPose);
	// load planner description
	uiPlannerDesc.plannerDescSeq.clear();
	uiPlannerDesc.plannerDescSeq.push_back(golem::Planner::Desc::load(&context, desc.libraryPathPlanner, desc.configPathPlanner));

	golem::KatanaGripper::Desc* pDesc = dynamic_cast<golem::KatanaGripper::Desc*>(&*uiPlannerDesc.pUIControllerDesc->pControllerDesc);
	if (pDesc == NULL)
		throw ExTinyKatana("KatanaI::create(): Unknown arm type");

	pDesc->bGripper = desc.bGripper;
	pDesc->sensorIndexSet = desc.sensorIndexSet;

	// create
	ControllerI::create(uiPlannerDesc);

	pKatanaGripper = dynamic_cast<golem::KatanaGripper*>(&pUIPlanner->getController());
	if (pKatanaGripper == NULL)
		throw ExTinyKatana("Katana::create(): unable to cast to KatanaGripper");
}

KatanaSensorDataSet KatanaI::gripperRecvSensorData(double timeOut, const ::Ice::Current&) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent("KatanaI::gripperRecvSensorData(): gripper not present");

	golem::KatanaGripper::SensorDataSet data;
	if (!pKatanaGripper->gripperRecvSensorData(data, SecToMSec(timeOut)))
		throw ExTinyKatanaGripperIOError("KatanaI::gripperRecvSensorData(): gripper IO error");

	return make(data);
}

KatanaGripperEncoderData KatanaI::gripperRecvEncoderData(double timeOut, const ::Ice::Current&) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent("KatanaI::gripperRecvEncoderData(): gripper not present");

	golem::KatanaGripper::GripperEncoderData data;
	if (!pKatanaGripper->gripperRecvEncoderData(data, SecToMSec(timeOut)))
		throw ExTinyKatanaGripperIOError("KatanaI::gripperRecvEncoderData(): gripper IO error");

	return make(data);
}

void KatanaI::gripperOpen(double timeOut, const ::Ice::Current&) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent("KatanaI::gripperOpen(): gripper not present");

	if (!pKatanaGripper->gripperOpen(SecToMSec(timeOut)))
		throw ExTinyKatanaGripperIOError("KatanaI::gripperOpen(): gripper IO error");
}

void KatanaI::gripperClose(const KatanaSensorDataSet& sensorThreshold, double timeOut, const ::Ice::Current&) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent("KatanaI::gripperClose(): gripper not present");

	if (!pKatanaGripper->gripperClose(make(sensorThreshold), SecToMSec(timeOut)))
		throw ExTinyKatanaGripperIOError("KatanaI::gripperClose(): gripper IO error");
}

void KatanaI::gripperFreeze(double timeOut, const ::Ice::Current&) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent("KatanaI::gripperFreeze(): gripper not initialized");

	if (!pKatanaGripper->gripperFreeze(SecToMSec(timeOut)))
		throw ExTinyKatanaGripperIOError("KatanaI::gripperFreeze(): gripper IO error");
}

//------------------------------------------------------------------------------

TinyI::TinyI(int argc, char *argv[], Ice::CommunicatorPtr communicator) : communicator(communicator) {
	try {
		// Determine configuration file name
		std::string cfg;
		if (argc == 1) {
			// default configuration file name
			cfg.assign(argv[0]);
#ifdef WIN32
			size_t pos = cfg.rfind(".exe"); // Windows only
			if (pos != std::string::npos) cfg.erase(pos);
#endif
			cfg.append(".xml");
		}
		else
			cfg.assign(argv[1]);

		// Create XML parser and load configuration file
		XMLParser::Desc parserDesc;
		pParser = parserDesc.create();
		try {
			FileReadStream fs(cfg.c_str());
			pParser->load(fs); // throw
		} catch (const Message&) {
			throw ExTinyCreate("Unable to open configuration file " + cfg + " - usage: " + argv[0] + " <configuration_file>");
		}

		// Find program XML root context
		pXMLContext = pParser->getContextRoot()->getContextFirst("golem");
		if (pXMLContext == NULL) {
			throw ExTinyCreate("Unknown configuration file " + cfg);
		}

		// Create program context
		Context::Desc contextDesc;
		XMLData(contextDesc, pXMLContext);
		pContext = contextDesc.create(); // throw
		
		// Create Universe
		golem::PhysUniverse::Desc universeDesc;
		universeDesc.name = "Golem (Tiny)";
		universeDesc.load(pXMLContext->getContextFirst("universe"));
		universeDesc.argc = argc;
		universeDesc.argv = argv;
		pUniverse = universeDesc.create(*pContext);

		// Create scene
		golem::PhysScene::Desc sceneDesc;
		sceneDesc.name = "Ice server";
		sceneDesc.load(pXMLContext->getContextFirst("scene"));
		pScene = pUniverse->createScene(sceneDesc);

		// Launching Universe
		pUniverse->launch();

		// Activate Golem Tiny
		id.name = "GolemTiny";
		XMLData("identity", id.name, pXMLContext->getContextFirst("ice"));
		std::string adapterName = id.name;
		adapterName.append("Adapter");
		std::string adapterTransport = "default -p 8172";
		XMLData("transport", adapterTransport, pXMLContext->getContextFirst("ice"));

		adapter = communicator->createObjectAdapterWithEndpoints(adapterName, adapterTransport);
		activate();
		adapter->activate();
	}
	catch (const ExTiny& ex) {
		throw ex;
	}
	catch (const Message& ex) {
		std::string str("TinyI::createActor(): ");
		str.append(ex.what());
		throw ExTinyActorCreate(str);
	}
	catch (const std::exception &ex) {
		std::string str("TinyI::createActor(): C++ exception: ");
		str.append(ex.what());
		throw ExTinyActorCreate(str);
	}
}

TinyI::~TinyI() {
	actorList.clear();
	pUniverse.release();
}

void TinyI::activate() {
	adapter->add(this, id);
}

void TinyI::deactivate() {
	adapter->remove(id);

	// deactivate actors
	for (ActorList::iterator i = actorList.begin(); i != actorList.end(); ++i)
		i->second->deactivate();
}

::Ice::Double TinyI::getTime(const ::Ice::Current&) const {
	return (::Ice::Double)pContext->getTimer().elapsed();
}

void TinyI::sleep(::Ice::Double duration, const ::Ice::Current&) const {
	Sleep::msleep(SecToMSec(duration));
}

bool TinyI::interrupted(const ::Ice::Current&) {
	return pUniverse->interrupted();
}

ActorIPtr TinyI::createActor(const ActorDesc* pDesc) {
	if (!pDesc)
		throw ExTinyActorCreate("TinyI::createActor(): NULL Actor description");
	
	ActorIPtr pActor;

	if (dynamic_cast<const KatanaDesc*>(pDesc)) {
		KatanaI* pKatanaI = new KatanaI(*this);
		pActor = pKatanaI;
		pKatanaI->create(*dynamic_cast<const KatanaDesc*>(pDesc));
	}
	else if (dynamic_cast<const ControllerDesc*>(pDesc)) {
		ControllerI* pArmI = new ControllerI(*this);
		pActor = pArmI;
		pArmI->create(*dynamic_cast<const ControllerDesc*>(pDesc));
	}
	else if (dynamic_cast<const RigidBodyDesc*>(pDesc)) {
		RigidBodyI* pRigidBodyI = new RigidBodyI(*this);
		pActor = pRigidBodyI;
		pRigidBodyI->create(*dynamic_cast<const RigidBodyDesc*>(pDesc));
	}
	else
		throw ExTinyActorCreate("TinyI::createActor(): Unknown Actor description");

	return pActor;
}

ActorPrx TinyI::createActor(const ActorDescPtr& pActorDesc, const ::Ice::Current&) {
	try {
		if (pActorDesc.get() == NULL)
			throw ExTinyActorCreate("TinyI::createActor(): empty description");

		ActorIPtr pActor = createActor(pActorDesc.get()); // throws
		if (pActor.get() == NULL)
			return NULL;
		pActor->owner = true;
		
		ActorPrx actorPrx = pActor->activate();
		actorList.push_back(ActorList::Pair(actorPrx, pActor));
		return actorPrx;
	}
	catch (const Message& ex) {
		std::string str("TinyI::createActor(): ");
		str.append(ex.what());
		throw ExTinyActorCreate(str);
	}
	catch (const std::exception &ex) {
		std::string str("TinyI::createActor(): C++ exception: ");
		str.append(ex.what());
		throw ExTinyActorCreate(str);
	}

	return ActorPrx();
}

void TinyI::releaseActor(const ActorPrx& actor, const ::Ice::Current&) {
	ActorList::iterator pos = actorList.find(actor);
	if (pos == actorList.end())
		throw ExTinyActorNotFound("TinyI::releaseActor(): Unable to find specified Actor");
	if (!pos->second->owner)
		throw ExTinyActorNotRemovable("TinyI::releaseActor(): The specified Actor cannot be removed");

	pos->second->deactivate();
	actorList.erase(pos);
}

ActorSeq TinyI::getActors(const ::Ice::Current&) const {
	ActorSeq actorSeq;
	for (ActorList::const_iterator i = actorList.begin(); i != actorList.end(); ++i)
		actorSeq.push_back(i->first);
	return actorSeq;
}

void TinyI::bang(const ::Ice::Current&) {
	pContext->write("Bang!\n");

	//Rand rand(pContext->getRandSeed());
	//Creator creator(*pScene);
	//golem::Actor::Desc *pActorDesc = creator.createTreeDesc(rand.nextUniform(Real(0.07), Real(0.10)));
	//pActorDesc->nxActorDesc.globalPose.t.set(
	//	(NxReal)rand.nextUniform(Real(-0.3), Real(0.3)),
	//	(NxReal)rand.nextUniform(Real(-0.3), Real(0.3)),
	//	(NxReal)rand.nextUniform(Real(+0.3), Real(0.9))
	//);
	//pScene->createObject(*pActorDesc);
}

//------------------------------------------------------------------------------

class TinyIceApp : public Ice::Application {
private:
	// Golem Tiny must be destroyed outside run() and ~Application() because
	// Ice keeps references to all created objects and destroys them after Golem Tiny
	IceInternal::Handle<TinyI> pTiny;

public:
	virtual int run(int argc, char *argv[]) {
		shutdownOnInterrupt();
		pTiny = new TinyI(argc, argv, communicator());
		communicator()->waitForShutdown();
		return 0;
	}
};

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	try {
		return TinyIceApp().main(argc, argv);
	}
	catch (const ExTiny& ex) {
		std::cerr << ex << std::endl;
	}
	return 1;
}
