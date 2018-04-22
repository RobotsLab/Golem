/** @file Tiny.cpp
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
#include <Golem/App/Data.h>
#include <Golem/Phys/Data.h>
#include <Golem/Ctrl/Katana/Data.h>
#include <Golem/Tiny/Tiny.h>
#include <Golem/Tiny/Types.h>

//------------------------------------------------------------------------------

using namespace golem::tiny;

//------------------------------------------------------------------------------

class golem::tiny::BoundsDesc {
public:
	golem::Bounds::Desc::Ptr pBoundsDesc;
	BoundsDesc(const golem::Bounds::Desc::Ptr pBoundsDesc) : pBoundsDesc(pBoundsDesc) {}
};

class golem::tiny::UIPlannerDesc : public golem::UIPlanner::Desc {
public:
};

//------------------------------------------------------------------------------

Shape::Shape(RigidBody& rigidBody) : rigidBody(rigidBody), pNxShapeDesc(0), pBounds(0), owner(false) {
}

Shape::~Shape() {
	if (pBounds == 0 || !owner)
		return;
	golem::Mat34 m = pBounds->getPose();
	rigidBody.pActor->releaseBounds(*pBounds);
}

void Shape::create(const ShapeDesc& desc, ShapeType type, const golem::shared_ptr<BoundsDesc>& pBoundsDesc) {
	this->type = type;
	this->density = desc.density;
	this->color = desc.color;

	// pBoundsDesc must be initialized before
	if (pBoundsDesc == 0)
		throw ExTinyShape(Message::LEVEL_CRIT, "Shape::create(): Unable to create bounds description");
	
	this->pBoundsDesc = pBoundsDesc;

	PhysScene* const pPhysScene = static_cast<PhysScene*>(&rigidBody.scene);
	if (!pPhysScene)
		throw ExTinyShapeCreate(Message::LEVEL_CRIT, "Shape::create(): Unable to cast to PhysScene");

	pNxShapeDesc = pPhysScene->createNxShapeDesc(this->pBoundsDesc->pBoundsDesc); // throws
	pNxShapeDesc->density = NxReal(this->density);
	// pBounds created in container
}

ShapeType Shape::getType() const {
	return type;
}

RGBA Shape::getColor() const {
	return color;
}

golem::Mat34 Shape::getLocalPose() const {
	return pBounds->getPose();
}

int Shape::getGroup() const {
	return (int)pBounds->getGroup();
}

void Shape::setGroup(int group) {
	rigidBody.pActor->setBoundsGroup(*pBounds, U32(group));
}

ShapeDescPtr Shape::getDesc() const {
	return pShapeDesc;
}

//------------------------------------------------------------------------------

PlaneShape::PlaneShape(RigidBody& rigidBody) : Shape(rigidBody) {
}
void PlaneShape::create(const PlaneShapeDesc& desc) {
	BoundingPlane::Desc* pDesc = new BoundingPlane::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = desc.localPose;
	pDesc->normal = desc.normal;
	pDesc->distance = (Real)desc.distance;
	Shape::create(desc, ShapeTypePlane, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc))); // throws
}

SphereShape::SphereShape(RigidBody& rigidBody) : Shape(rigidBody) {
}
void SphereShape::create(const SphereShapeDesc& desc) {
	BoundingSphere::Desc* pDesc = new BoundingSphere::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = desc.localPose;
	pDesc->radius = (Real)desc.radius;
	Shape::create(desc, ShapeTypeSphere, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc))); // throws
}

CylinderShape::CylinderShape(RigidBody& rigidBody) : Shape(rigidBody) {
}
void CylinderShape::create(const CylinderShapeDesc& desc) {
	BoundingCylinder::Desc* pDesc = new BoundingCylinder::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = desc.localPose;
	pDesc->radius = (Real)desc.radius;
	pDesc->length = (Real)desc.length;
	Shape::create(desc, ShapeTypeCylinder, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc))); // throws
}

BoxShape::BoxShape(RigidBody& rigidBody) : Shape(rigidBody) {
}
void BoxShape::create(const BoxShapeDesc& desc) {
	BoundingBox::Desc* pDesc = new BoundingBox::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = desc.localPose;
	pDesc->dimensions = desc.dimensions;
	Shape::create(desc, ShapeTypeBox, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc))); // throws
}

ConvexMeshShape::ConvexMeshShape(RigidBody& rigidBody) : Shape(rigidBody) {
}
void ConvexMeshShape::create(const ConvexMeshShapeDesc& desc) {
	BoundingConvexMesh::Desc* pDesc = new BoundingConvexMesh::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = desc.localPose;
	pDesc->vertices = desc.vertices;
	pDesc->bCook = true;
	Shape::create(desc, ShapeTypeConvexMesh, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc))); // throws
}

//------------------------------------------------------------------------------

Actor::Actor(Tiny& tiny) : tiny(tiny), context(*tiny.pContext), scene(*tiny.pScene), pXMLContext(tiny.pXMLContext), owner(false) {
}

Actor::~Actor() {
}

RigidBody::RigidBody(Tiny& tiny) : Actor(tiny), pActor(0) {
}

RigidBody::~RigidBody() {
	shapeList.clear(); // release shapes before releasing the Actor
	if (pActor == 0 || !owner)
		return;
	scene.releaseObject(*pActor);
}

ShapePtr RigidBody::createShape(const golem::Bounds* pBounds) {
	ShapeDescPtr pShapeDesc;
	ShapePtr pShape;

	switch (pBounds->getType()) {
	case Bounds::TYPE_PLANE:
		{
			PlaneShapeDesc* pDesc = new PlaneShapeDesc;
			pShapeDesc.reset(pDesc);
			pDesc->normal = dynamic_cast<const BoundingPlane*>(pBounds)->getNormal();
			pDesc->distance = (double)dynamic_cast<const BoundingPlane*>(pBounds)->getDistance();
		}
		break;
	case Bounds::TYPE_SPHERE:
		{
			SphereShapeDesc* pDesc = new SphereShapeDesc;
			pShapeDesc.reset(pDesc);
			pDesc->radius = (double)dynamic_cast<const BoundingSphere*>(pBounds)->getRadius();
		}
		break;
	case Bounds::TYPE_CYLINDER:
		{
			CylinderShapeDesc* pDesc = new CylinderShapeDesc;
			pShapeDesc.reset(pDesc);
			pDesc->radius = (double)dynamic_cast<const BoundingCylinder*>(pBounds)->getRadius();
			pDesc->length = (double)dynamic_cast<const BoundingCylinder*>(pBounds)->getLength();
		}
		break;
	case Bounds::TYPE_BOX:
		{
			BoxShapeDesc* pDesc = new BoxShapeDesc;
			pShapeDesc.reset(pDesc);
			pDesc->dimensions = dynamic_cast<const BoundingBox*>(pBounds)->getDimensions();
		}
		break;
	case Bounds::TYPE_CONVEX_MESH:
		{
			ConvexMeshShapeDesc* pDesc = new ConvexMeshShapeDesc;
			pShapeDesc.reset(pDesc);
			const BoundingConvexMesh* pBoundingConvexMesh = dynamic_cast<const BoundingConvexMesh*>(pBounds);
			pDesc->vertices = pBoundingConvexMesh->getVertices();
		}
		break;
	default:
		throw ExTinyShapeCreate(Message::LEVEL_ERROR, "RigidBody::createShape(): Unknown shape type");
	}

	pShapeDesc->localPose = pBounds->getPose();
	pShapeDesc->group = (int)pBounds->getGroup();

	pShape = pShapeDesc->create(*this);// throws
	pShape->pShapeDesc = pShapeDesc;
	pShape->pBounds = pBounds;
	
	return pShape;
}

void RigidBody::create(const RigidBodyDesc& desc) {
	kinematic = desc.kinematic;

	golem::PhysActor::Desc actorDesc;
	actorDesc.kinematic = desc.kinematic;
	actorDesc.pose = desc.globalPose;

	shapeList.clear();
	for (ShapeDescSeq::const_iterator i = desc.shapes.begin(); i != desc.shapes.end(); ++i) {
		ShapePtr pShape = (**i).create(*this); // throws
		pShape->pShapeDesc = *i;
		
		if (pShape->getType() == ShapeTypePlane)
			actorDesc.nxActorDesc.body = 0; // Actors with Plane Shapes cannot have a body
		actorDesc.nxActorDesc.shapes.push_back(pShape->pNxShapeDesc);
		
		shapeList.push_back(ShapeList::Pair(pShape.get(), pShape));
	}

	pActor = dynamic_cast<golem::Actor*>(scene.createObject(actorDesc)); // throws
	if (pActor == NULL)
		throw ExTinyShapeCreate(Message::LEVEL_CRIT, "RigidBody::create(): Unable to cast to Actor");

	const golem::Actor::BoundsConstSeq& boundsSeq = pActor->getBoundsSeq();
	golem::Actor::BoundsConstSeq::const_iterator i = boundsSeq.begin();
	ShapeList::iterator j = shapeList.begin();
	while (i != boundsSeq.end() && j != shapeList.end())
		(**j++).pBounds = *i++; // order is preserved
}

Shape* RigidBody::createShape(const ShapeDescPtr& pShapeDesc) {
	try {
		if (pShapeDesc == 0)
			throw ExTinyShapeCreate(Message::LEVEL_CRIT, "RigidBody::createShape(): empty description");

		ShapePtr pShape = pShapeDesc->create(*this); // throws
		pShape->pShapeDesc = pShapeDesc;

		pShape->pBounds = pActor->createBounds(pShape->pBoundsDesc->pBoundsDesc); // throws
		pShape->owner = true;

		shapeList.push_back(ShapeList::Pair(pShape.get(), pShape));
		return pShape.get();
	}
	catch (const Message& ex) {
		throw ExTinyShapeCreate(Message::LEVEL_CRIT, "RigidBody::createShape(): %s", ex.msg()); // translate
	}
	catch (const std::exception &ex) {
		throw ExTinyShapeCreate( Message::LEVEL_CRIT, "RigidBody::createShape(): C++ exception: %s", ex.what()); // translate
	}
}

void RigidBody::releaseShape(Shape* shape) {
	if (!shapeList.contains(shape))
		throw ExTinyShapeNotFound(Message::LEVEL_CRIT, "RigidBody::releaseShape(): Unable to find specified Shape");
	if (!shape->owner)
		throw ExTinyShapeNotRemovable(Message::LEVEL_CRIT, "RigidBody::releaseShape(): The specified Shape cannot be removed");
	shapeList.erase(shape);
}

ShapeSeq RigidBody::getShapes() const {
	return ShapeSeq(shapeList.begin(), shapeList.end());
}

golem::Mat34 RigidBody::getGlobalPose() const {
	return pActor->getPose();
}

void RigidBody::setGlobalPose(const golem::Mat34& pose) {
	pActor->setPose(pose);
}

void RigidBody::setGroup(int group) {
	pActor->setBoundsGroup(U32(group));
}

//------------------------------------------------------------------------------

Joint::Joint(Controller& controller) : RigidBody(controller.tiny), controller(controller) {
}

void Joint::create(const JointDesc& desc) {
	pActor = desc.pJointActor;
	kinematic = true;
	
	shapeList.clear();
	const golem::Actor::BoundsConstSeq& boundsSeq = pActor->getBoundsSeq();
	for (golem::Actor::BoundsConstSeq::const_iterator i = boundsSeq.begin(); i != boundsSeq.end(); ++i) {
		ShapePtr pShape = RigidBody::createShape(*i); // throws
		shapeList.push_back(ShapeList::Pair(pShape.get(), pShape));
	}
}

ActorPtr JointDesc::create(Tiny& tiny) const {
	// Joint cannot be created as a normal Actor, return 0
	return ActorPtr();
}

Controller::Controller(Tiny& tiny) : Actor(tiny), pUIPlanner(0) {
}

Controller::~Controller() {
	jointList.clear(); // joints wrappers must be released before UIPlanner!
	if (pUIPlanner == 0 || !owner)
		return;
	scene.releaseObject(*pUIPlanner);
}

void Controller::create(const ControllerDesc& desc) {
	golem::tiny::UIPlannerDesc uiPlannerDesc;
	// phys planner load from local xml context description
	golem::XMLData((golem::UIPlanner::Desc&)uiPlannerDesc, pXMLContext);
	// load controller description
	uiPlannerDesc.pUIControllerDesc->pControllerDesc = golem::Controller::Desc::load(&context, desc.libraryPathCtrl, desc.configPathCtrl);
	// overwrite default parameters
	uiPlannerDesc.pUIControllerDesc->pControllerDesc->globalPose = desc.globalPose;
	// load planner description
	uiPlannerDesc.plannerDescSeq.clear();
	uiPlannerDesc.plannerDescSeq.push_back(golem::Planner::Desc::load(&context, desc.libraryPathPlanner, desc.configPathPlanner));
	// create
	Controller::create(uiPlannerDesc);
}

void Controller::create(const UIPlannerDesc& desc) {
	// create controller
	pUIPlanner = dynamic_cast<golem::UIPlanner*>(scene.createObject(desc));// throws
	if (pUIPlanner == NULL)
		throw ExTinyController(Message::LEVEL_CRIT, "Controller::create(): unable to cast to UIPlanner");
	
	// create joints
	const golem::Configspace::Range joints = pUIPlanner->getController().getStateInfo().getJoints();
	const golem::BodyActor::ConfigSeq& jointActorSeq = pUIPlanner->getUIController().getJointActors();

	jointList.clear();
	for (golem::Configspace::Index i = joints.begin(); i < joints.end(); ++i) {
		JointPtr pJoint;
		if (jointActorSeq[i] != 0) { // Joint may have no body
			JointDesc jointDesc;
			jointDesc.pJointActor = jointActorSeq[i];
			pJoint = jointDesc.create(*this); // throws
			jointList.push_back(JointList::Pair(pJoint.get(), pJoint));
		}
	}
}

GenConfigspaceState Controller::recvGenConfigspaceState(double t) const {
	golem::Controller::State gcs = pUIPlanner->getController().createState();
	try {
		pUIPlanner->getController().lookupCommand(t, gcs);
	}
	catch (const Message&) {
		throw ExTinyControllerRecv("Controller::recvGenConfigspaceState(): lookup command error");
	}
	return make(gcs);
}

GenWorkspaceChainState Controller::recvGenWorkspaceState(double t) const {
	golem::Controller::State gcs = pUIPlanner->getController().createState();
	try {
		pUIPlanner->getController().lookupCommand(t, gcs);
	}
	catch (const Message&) {
		throw ExTinyControllerRecv("Controller::recvGenWorkspaceState(): lookup command error");
	}
	golem::GenWorkspaceChainState gws;
	pUIPlanner->getController().chainForwardTransform(gcs.cpos, gws.wpos);
	const golem::Chainspace::Range chains = pUIPlanner->getController().getStateInfo().getChains();
	for (golem::Chainspace::Index i = chains.begin(); i < chains.end(); ++i)
		gws.wpos[i].multiply(gws.wpos[i], pUIPlanner->getController().getChains()[i]->getReferencePose()); // reference pose
	gws.t = gcs.t;
	return make(chains, gws);
}

void Controller::send(const GenConfigspaceStateSeq& trajectory, double timeWait) {
	const golem::Controller::Trajectory tmp(makeSeq<golem::Controller::Trajectory>(pUIPlanner->getController(), trajectory));
	if (pUIPlanner->getController().send(&tmp.front(), &tmp.back() + 1) != &tmp.back() + 1)
		throw ExTinyControllerSend(Message::LEVEL_CRIT, "Controller::sendTrajectory(): send trajectory error");

	(void)pUIPlanner->getController().waitForEnd(SecToMSec(timeWait));
}

void Controller::stop() {
	pUIPlanner->getController().stop();
}

GenConfigspaceState Controller::findTarget(const GenConfigspaceState &begin, const GenWorkspaceChainState& end) {
	golem::GenConfigspaceState cend;
	if (!pUIPlanner->getPlannerSeq()[0]->findTarget(make(pUIPlanner->getController(), begin), make(pUIPlanner->getController().getStateInfo().getChains(), end), cend))
		throw ExTinyControllerFindTarget(Message::LEVEL_CRIT, "Controller::findTarget(): find target error");

	return make(pUIPlanner->getController().getStateInfo().getJoints(), cend);
}

GenConfigspaceStateSeq Controller::findGlobalTrajectory(const GenConfigspaceState &begin, const GenConfigspaceState &end) {
	golem::Controller::Trajectory tmp;
	if (!pUIPlanner->getPlannerSeq()[0]->findGlobalTrajectory(make(pUIPlanner->getController(), begin), make(pUIPlanner->getController(), end), tmp, tmp.begin()))
		throw ExTinyControllerFindTrajectory(Message::LEVEL_CRIT, "Controller::findGlobalTrajectory(): find trajectory error");

	return makeSeq<GenConfigspaceStateSeq>(tmp);
}

GenConfigspaceStateSeq Controller::findLocalTrajectory(const GenConfigspaceState &begin, const GenWorkspaceChainStateSeq &trajectory) {
	golem::GenWorkspaceChainState::Seq inp = makeSeq<golem::GenWorkspaceChainState::Seq>(pUIPlanner->getController().getStateInfo().getChains(), trajectory);
	golem::Controller::Trajectory out;
	if (!pUIPlanner->getPlannerSeq()[0]->findLocalTrajectory(make(pUIPlanner->getController(), begin),
		inp.begin(), inp.end(),
		out, out.begin()))
		throw ExTinyControllerFindTrajectory(Message::LEVEL_CRIT, "Controller::findLocalTrajectory(): find trajectory error");

	return makeSeq<GenConfigspaceStateSeq>(out);
}

WorkspaceChainCoord Controller::getChainForwardTransform(const ConfigspaceCoord& cc) const {
	golem::WorkspaceChainCoord tmp;
	pUIPlanner->getController().chainForwardTransform(make(pUIPlanner->getController().getStateInfo().getJoints(), cc), tmp);
	return make(pUIPlanner->getController().getStateInfo().getChains(), tmp);
}

WorkspaceJointCoord Controller::getJointForwardTransform(const ConfigspaceCoord& cc) const {
	golem::WorkspaceJointCoord tmp;
	pUIPlanner->getController().jointForwardTransform(make(pUIPlanner->getController().getStateInfo().getJoints(), cc), tmp);
	return make(pUIPlanner->getController().getStateInfo().getJoints(), tmp);
}

Jacobian Controller::getJacobian(const ConfigspaceCoord& cc) const {
	golem::Jacobian jacobian;
	pUIPlanner->getController().jacobian(make(pUIPlanner->getController().getStateInfo().getJoints(), cc), jacobian);
	return make(pUIPlanner->getController().getStateInfo().getJoints(), jacobian);
}

JointSeq Controller::getJoints() const {
	return JointSeq(jointList.begin(), jointList.end());
}

int Controller::getControllerGroup() const {
	return (int)pUIPlanner->getControllerBoundsGroup();
}

int Controller::getCollisionGroup() const {
	return (int)pUIPlanner->getCollisionBoundsGroup();
}

void Controller::setCollisionGroup(int group) {
	pUIPlanner->setCollisionBoundsGroup((U32)group);
}

golem::Mat34 Controller::getGlobalPose() const {
	return pUIPlanner->getController().getGlobalPose();
}

void Controller::setGlobalPose(const golem::Mat34& pose) {
	pUIPlanner->getController().setGlobalPose(pose);
}

WorkspaceChainCoord Controller::getReferencePose() const {
	const golem::Chainspace::Range chains = pUIPlanner->getController().getStateInfo().getChains();
	golem::WorkspaceChainCoord tmp;
	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i)
		tmp[i] = pUIPlanner->getController().getChains()[i]->getReferencePose();
	return make(chains, tmp);
}

void Controller::setReferencePose(const WorkspaceChainCoord& pose) {
	const golem::Chainspace::Range chains = pUIPlanner->getController().getStateInfo().getChains();
	const golem::WorkspaceChainCoord tmp = make(chains, pose);
	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i)
		pUIPlanner->getController().getChains()[i]->setReferencePose(tmp[i]);
}

//------------------------------------------------------------------------------

Katana::Katana(Tiny& tiny) : Controller(tiny) {
}

void Katana::create(const KatanaDesc& desc) {
	golem::tiny::UIPlannerDesc uiPlannerDesc;
	// phys planner load from local xml context description
	golem::XMLData((golem::UIPlanner::Desc&)uiPlannerDesc, pXMLContext);
	// load controller description
	uiPlannerDesc.pUIControllerDesc->pControllerDesc = golem::Controller::Desc::load(&context, desc.libraryPathCtrl, desc.configPathCtrl);
	// overwrite default parameters
	uiPlannerDesc.pUIControllerDesc->pControllerDesc->globalPose = desc.globalPose;
	// load planner description
	uiPlannerDesc.plannerDescSeq.clear();
	uiPlannerDesc.plannerDescSeq.push_back(golem::Planner::Desc::load(&context, desc.libraryPathPlanner, desc.configPathPlanner));

	golem::KatanaGripper::Desc* pDesc = dynamic_cast<golem::KatanaGripper::Desc*>(&*uiPlannerDesc.pUIControllerDesc->pControllerDesc);
	if (pDesc == NULL)
		throw ExTinyKatana(Message::LEVEL_CRIT, "Katana::create(): Unknown controller type");
	pDesc->bGripper = desc.bGripper;
	pDesc->sensorIndexSet = desc.sensorIndexSet;

	// create
	Controller::create(uiPlannerDesc);

	pKatanaGripper = dynamic_cast<golem::KatanaGripper*>(&pUIPlanner->getController());
	if (pKatanaGripper == NULL)
		throw ExTinyKatana(Message::LEVEL_CRIT, "Katana::create(): unable to cast to KatanaGripper");
}

KatanaSensorDataSet Katana::gripperRecvSensorData(double timeOut) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent(Message::LEVEL_CRIT, "Katana::gripperRecvSensorData(): gripper not present");

	golem::KatanaGripper::SensorDataSet data;
	if (!pKatanaGripper->gripperRecvSensorData(data, SecToMSec(timeOut)))
		throw ExTinyKatanaGripperIOError(Message::LEVEL_CRIT, "Katana::gripperRecvSensorData(): gripper IO error");

	return make(data);
}

KatanaGripperEncoderData Katana::gripperRecvEncoderData(double timeOut) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent(Message::LEVEL_CRIT, "Katana::gripperRecvEncoderData(): gripper not present");

	golem::KatanaGripper::GripperEncoderData data;
	if (!pKatanaGripper->gripperRecvEncoderData(data, SecToMSec(timeOut)))
		throw ExTinyKatanaGripperIOError(Message::LEVEL_CRIT, "Katana::gripperRecvEncoderData(): gripper IO error");

	return make(data);
}

void Katana::gripperOpen(double timeOut) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent(Message::LEVEL_CRIT, "Katana::gripperOpen(): gripper not present");

	if (!pKatanaGripper->gripperOpen(SecToMSec(timeOut)))
		throw ExTinyKatanaGripperIOError(Message::LEVEL_CRIT, "Katana::gripperOpen(): gripper IO error");
}

void Katana::gripperClose(const KatanaSensorDataSet& sensorThreshold, double timeOut) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent(Message::LEVEL_CRIT, "Katana::gripperClose(): gripper not present");

	if (!pKatanaGripper->gripperClose(make(sensorThreshold), SecToMSec(timeOut)))
		throw ExTinyKatanaGripperIOError(Message::LEVEL_CRIT, "Katana::gripperClose(): gripper IO error");
}

void Katana::gripperFreeze(double timeOut) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent(Message::LEVEL_CRIT, "Katana::gripperFreeze(): gripper not initialized");

	if (!pKatanaGripper->gripperFreeze(SecToMSec(timeOut)))
		throw ExTinyKatanaGripperIOError(Message::LEVEL_CRIT, "Katana::gripperFreeze(): gripper IO error");
}

//------------------------------------------------------------------------------

Tiny::Tiny(int argc, char *argv[]) {
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
			throw ExTinyCreate(Message::LEVEL_CRIT, "Unable to open configuration file %s - usage: %s <configuration_file>", cfg.c_str(), argv[0]);
		}

		// Find program XML root context
		pXMLContext = pParser->getContextRoot()->getContextFirst("golem");
		if (pXMLContext == 0)
			throw ExTinyCreate(Message::LEVEL_CRIT, "Unknown configuration file %s", cfg.c_str());

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
		sceneDesc.name = "Tiny interface";
		sceneDesc.load(pXMLContext->getContextFirst("scene"));
		pScene = pUniverse->createScene(sceneDesc);

		// Launching Universe
		pUniverse->launch();
	}
	catch (const ExTiny& ex) {
		throw ex;
	}
	catch (const golem::Message& ex) {
		throw ExTinyCreate(Message::LEVEL_CRIT, "Tiny::Tiny(): %s", ex.what()); // translate
	}
	catch (const std::exception &ex) {
		throw ExTinyCreate(Message::LEVEL_CRIT, "Tiny::Tiny(): C++ exception: %s", ex.what()); // translate
	}
}

Tiny::~Tiny() {
	actorList.clear();
	pUniverse.release();
}

//------------------------------------------------------------------------------

double Tiny::getTime() const {
	return (double)pContext->getTimer().elapsed();
}

void Tiny::sleep(double duration) const {
	Sleep::msleep(SecToMSec(duration));
}

void Tiny::print(const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	pContext->getMessageStream()->write(Message::TIME_UNDEF, Message::THREAD_UNDEF, Message::LEVEL_UNDEF, Message::CODE_UNDEF, format, argptr);
	va_end(argptr);
}

bool Tiny::interrupted() const {
	return pUniverse->interrupted();
}

int Tiny::waitKey(double timeOut) {
	return pUniverse->waitKey(SecToMSec(timeOut));
}

const golem::XMLContext* Tiny::getXMLContext() const {
	return pXMLContext;
}
golem::XMLContext* Tiny::getXMLContext() {
	return pXMLContext;
}

const golem::Context* Tiny::getContext() const {
	return pContext.get();
}
golem::Context* Tiny::getContext() {
	return pContext.get();
}

Actor* Tiny::createActor(const ActorDescPtr& pActorDesc) {
	try {
		if (pActorDesc == 0)
			throw ExTinyActorCreate(Message::LEVEL_CRIT, "Tiny::createActor(): empty description");

		ActorPtr pActor = pActorDesc->create(*this);
		if (pActor == 0)
			return 0;
		
		pActor->owner = true;
		actorList.push_back(ActorList::Pair(pActor.get(), pActor));
		return pActor.get();
	}
	catch (const Message& ex) {
		throw ExTinyActorCreate(Message::LEVEL_CRIT, "Tiny::createActor(): %s", ex.msg()); // translate
	}
	catch (const std::exception &ex) {
		throw ExTinyActorCreate(Message::LEVEL_CRIT, "Tiny::createActor(): C++ exception: %s", ex.what()); // translate
	}

	return 0;
}

void Tiny::releaseActor(Actor* actor) {
	if (!actorList.contains(actor))
		throw ExTinyActorNotFound(Message::LEVEL_CRIT, "Tiny::releaseActor(): Unable to find specified Actor");
	if (!actor->owner)
		throw ExTinyActorNotRemovable(Message::LEVEL_CRIT, "Tiny::releaseActor(): The specified Actor cannot be removed");

	actorList.erase(actor);
}

ActorSeq Tiny::getActors() const {
	return ActorSeq(actorList.begin(), actorList.end());
}

//------------------------------------------------------------------------------

void Tiny::bang() {
	pContext->write("Bang!\n");

	//Rand rand(pContext->getRandSeed());
	//golem::Creator creator(*pScene);
	//golem::Actor::Desc *pActorDesc = creator.createTreeDesc(rand.nextUniform(Real(0.07), Real(0.10)));
	//pActorDesc->nxActorDesc.globalPose.t.set(
	//	rand.nextUniform(NxReal(-0.3), NxReal(0.3)),
	//	rand.nextUniform(NxReal(-0.3), NxReal(0.3)),
	//	rand.nextUniform(NxReal(+0.3), NxReal(0.9))
	//);
	//pScene->createObject(*pActorDesc);
}

//------------------------------------------------------------------------------
