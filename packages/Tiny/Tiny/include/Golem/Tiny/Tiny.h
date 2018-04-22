/** @file Tiny.h
 * 
 * Golem "Tiny" interface
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
#ifndef _GOLEM_TINY_TINY_H_
#define _GOLEM_TINY_TINY_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/System.h>
#include <Golem/Defs/Pointers.h>
#include <Golem/Defs/Defs.h>
#include <Golem/Math/Twist.h>
#include <Golem/Math/Collection.h>
#include <Golem/Sys/Message.h>
#include <string>
#include <vector>

//------------------------------------------------------------------------------

class NxShapeDesc;

namespace golem {
class Context;
class XMLContext;
class XMLParser;
class Bounds;
class BoundingPlane;
class BoundingSphere;
class BoundingCylinder;
class BoundingBox;
class BoundingConvexMesh;
class Actor;
class Scene;
class Universe;
class BodyActor;
class KatanaGripper;
class UIPlanner;

namespace tiny {
class BoundsDesc;
class UIPlannerDesc;
};
};

//------------------------------------------------------------------------------

namespace golem {
namespace tiny {

//------------------------------------------------------------------------------

/** Golem Tiny exceptions */
MESSAGE_DEF(ExTiny, Message)

MESSAGE_DEF(ExTinyCreate, ExTiny)

MESSAGE_DEF(ExTinyActor, ExTiny)
MESSAGE_DEF(ExTinyActorCreate, ExTinyActor)
MESSAGE_DEF(ExTinyActorNotFound, ExTinyActor)
MESSAGE_DEF(ExTinyActorNotRemovable, ExTinyActor)

MESSAGE_DEF(ExTinyShape, ExTiny)
MESSAGE_DEF(ExTinyShapeCreate, ExTinyShape)
MESSAGE_DEF(ExTinyShapeNotFound, ExTinyShape)
MESSAGE_DEF(ExTinyShapeNotRemovable, ExTinyShape)

MESSAGE_DEF(ExTinyController, ExTiny)
MESSAGE_DEF(ExTinyControllerSend, ExTinyController)
MESSAGE_DEF(ExTinyControllerRecv, ExTinyController)
MESSAGE_DEF(ExTinyControllerFindTarget, ExTinyController)
MESSAGE_DEF(ExTinyControllerFindTrajectory, ExTinyController)

MESSAGE_DEF(ExTinyKatana, ExTinyController)
MESSAGE_DEF(ExTinyKatanaGripperNotPresent, ExTinyKatana)
MESSAGE_DEF(ExTinyKatanaGripperIOError, ExTinyKatana)

//------------------------------------------------------------------------------

/** Forward declarations */
class Shape;
class ShapeDesc;
class PlaneShapeDesc;
class SphereShapeDesc;
class CylinderShapeDesc;
class BoxShapeDesc;
class ConvexMeshShapeDesc;
class Actor;
class ActorDesc;
class RigidBody;
class RigidBodyDesc;
class Joint;
class JointDesc;
class Controller;
class ControllerDesc;
class Katana;
class KatanaDesc;
class Tiny;

//------------------------------------------------------------------------------

/** Chain space consists of kinematic chains (equals golem::Chainspace::DIM) */
const int CHAINSPACE_DIM = 6;

/** Configuration space consists of coordinates of kinematic chains (equals golem::Configspace::DIM) */
const int CONFIGSPACE_DIM = 27;

//------------------------------------------------------------------------------

/** Real-valued coordinates */
template <int _DIM> class RealCoord {
public:
	RealCoord() {
		setToDefault();
	}
	void setToDefault() {
		std::fill(c, c + _DIM, 0.);
	}
	/** coordinates */
	double c[_DIM];
};
/** Chainspace coordinates */
typedef RealCoord<CHAINSPACE_DIM> ChainspaceCoord;
/** Configspace coordinates */
typedef RealCoord<CONFIGSPACE_DIM> ConfigspaceCoord;
typedef std::vector<ConfigspaceCoord> ConfigspaceCoordSeq;

/** Generalized configuration space state */
class GenConfigspaceState {
public:
	GenConfigspaceState() {
		setToDefault();
	}
	void setToDefault() {
		pos.setToDefault();
		vel.setToDefault();
		t = 0.;
	}
	/** position */
	ConfigspaceCoord pos;
	/** velocity */
	ConfigspaceCoord vel;
	/** time */
	double t;
};
typedef std::vector<GenConfigspaceState> GenConfigspaceStateSeq;

/** Manipulator Jacobian (in twist coordinates). */
class Jacobian {
public:
	Twist c[CONFIGSPACE_DIM];
};

//------------------------------------------------------------------------------

/** Workspace coordinates */
template <int _DIM> class WorkspaceCoord {
public:
	WorkspaceCoord() {
		setToDefault();
	}
	void setToDefault() {
		for (int i = 0; i < _DIM; ++i)
			c[i].setId();
	}
	/** coordinates */
	Mat34 c[_DIM];
};
/** Workspace coordinates (tool frames) */
typedef WorkspaceCoord<CHAINSPACE_DIM> WorkspaceChainCoord;
/** Workspace coordinates (all joints) */
typedef WorkspaceCoord<CONFIGSPACE_DIM> WorkspaceJointCoord;

/** Generalized workspace state */
template <int _DIM> class GenWorkspaceState {
public:
	GenWorkspaceState() {
		setToDefault();
	}
	void setToDefault() {
		pos.setToDefault();
		t = 0.;
	}
	/** pose */
	WorkspaceCoord<_DIM> pos;
	/** time */
	double t;
};
/** Workspace coordinates (tool frames) */
typedef GenWorkspaceState<CHAINSPACE_DIM> GenWorkspaceChainState;
typedef std::vector<GenWorkspaceChainState> GenWorkspaceChainStateSeq;
/** Workspace coordinates (all joints) */
typedef GenWorkspaceState<CONFIGSPACE_DIM> GenWorkspaceJointState;
typedef std::vector<GenWorkspaceJointState> GenWorkspaceJointStateSeq;

//----------------------------------------------------------------------------

/** Shape types. */
enum ShapeType {
	/** plane */
	ShapeTypePlane,
	/** sphere */
	ShapeTypeSphere,
	/** cylinder */
	ShapeTypeCylinder,
	/** capsule */
	ShapeTypeCapsule,
	/** parallelepiped */
	ShapeTypeBox,
	/** generic triangle mesh */
	ShapeTypeTriangleMesh,
	/** convex triangle mesh */
	ShapeTypeConvexMesh,
};

/** RGBA Color */
class RGBA {
public:
	RGBA() {
		setToDefault();
	}
	RGBA(float r, float g, float b, float a) : r(r), g(g), b(b), a(a) {
	}
	void setToDefault() {
		r = 1.;
		g = 1.;
		b = 1.;
		a = 1.;
	}
	float r;
	float g;
	float b;
	float a;
};

typedef shared_ptr<Shape> ShapePtr;
typedef std::vector<Shape*> ShapeSeq;

typedef shared_ptr<ShapeDesc> ShapeDescPtr;
typedef std::vector<ShapeDescPtr> ShapeDescSeq;

/** Shape interface */
class Shape {
private:
	friend class RigidBody;
	NxShapeDesc* pNxShapeDesc;
	ShapeType type;
	double density;
	RGBA color;

protected:
	RigidBody& rigidBody;
	shared_ptr<BoundsDesc> pBoundsDesc;
	ShapeDescPtr pShapeDesc;
	const Bounds* pBounds;
	bool owner;

	void create(const ShapeDesc& desc, ShapeType type, const shared_ptr<BoundsDesc>& pBoundsDesc);
	Shape(RigidBody& rigidBody);
	
public:
	virtual ~Shape();
	/** Shape type */
	ShapeType getType() const;
	/** Shape color */
	RGBA getColor() const;
	/** Local pose */
	Mat34 getLocalPose() const;
	/** Returns shape group */
	int getGroup() const;
	/** Sets shape group */
	void setGroup(int group);
	/** Description */
	ShapeDescPtr getDesc() const;
};

/** Shape description */
class ShapeDesc {
protected:
	friend class RigidBody;

	virtual ShapePtr create(RigidBody& rigidBody) const = 0;
	ShapeDesc() {
		setToDefault();
	}

public:
	virtual ~ShapeDesc() {}
	void setToDefault() {
		localPose.setId();
		density = 1.;
		group = 1<<0;
		color.setToDefault();
	}
	/** Local pose */
	Mat34 localPose;
	/** Density */
	double density;
	/** Shape group */
	int group;
	/** Color */
	RGBA color;
};

/** Plane shape */
class PlaneShape : public Shape {
protected:
	friend class PlaneShapeDesc;
	const BoundingPlane* pBoundingPlane;

	void create(const PlaneShapeDesc& desc);
	PlaneShape(RigidBody& rigidBody);
public:
};

/** Plane shape description */
class PlaneShapeDesc : public ShapeDesc {
protected:
	CREATE_FROM_OBJECT_DESC_1(PlaneShape, ShapePtr, RigidBody&)
	virtual ~PlaneShapeDesc() {}
public:
	PlaneShapeDesc() {
		setToDefault();
	}
	void setToDefault() {
		ShapeDesc::setToDefault();
		normal.set((Real)0.0, (Real)0.0, (Real)1.0);
		distance = 0.;
	}
	/** The plane normal vector. */
	Vec3 normal;
	/** The distance from the origin. */
	double distance;
};

/** Sphere shape */
class SphereShape : public Shape {
protected:
	friend class SphereShapeDesc;
	const BoundingSphere* pBoundingSphere;
	
	void create(const SphereShapeDesc& desc);
	SphereShape(RigidBody& rigidBody);
public:
};

/** Sphere shape description */
class SphereShapeDesc : public ShapeDesc {
protected:
	CREATE_FROM_OBJECT_DESC_1(SphereShape, ShapePtr, RigidBody&)
	virtual ~SphereShapeDesc() {}
public:
	SphereShapeDesc() {
		setToDefault();
	}
	void setToDefault() {
		ShapeDesc::setToDefault();
		radius = 0.1;
	}
	/** The radius of the sphere. */
	double radius;
};

/** Cylinder shape */
class CylinderShape : public Shape {
protected:
	friend class CylinderShapeDesc;
	const BoundingCylinder* pBoundingCylinder;
	
	void create(const CylinderShapeDesc& desc);
	CylinderShape(RigidBody& rigidBody);
public:
};

/** Cylinder shape description */
class CylinderShapeDesc : public ShapeDesc {
protected:
	CREATE_FROM_OBJECT_DESC_1(CylinderShape, ShapePtr, RigidBody&)
	virtual ~CylinderShapeDesc() {}
public:
	CylinderShapeDesc() {
		setToDefault();
	}
	void setToDefault() {
		ShapeDesc::setToDefault();
		radius = 0.1;
		length = 0.1;
	}
	/** The radius of the cylinder. */
	double radius;
	/** The length of the cylinder. */
	double length;
};

/** Box shape */
class BoxShape : public Shape {
protected:
	friend class BoxShapeDesc;
	const BoundingBox* pBoundingBox;
	
	void create(const BoxShapeDesc& desc);
	BoxShape(RigidBody& rigidBody);
public:
};

/** Box shape description */
class BoxShapeDesc : public ShapeDesc {
protected:
	CREATE_FROM_OBJECT_DESC_1(BoxShape, ShapePtr, RigidBody&)
	virtual ~BoxShapeDesc() {}
public:
	BoxShapeDesc() {
		setToDefault();
	}
	void setToDefault() {
		ShapeDesc::setToDefault();
		dimensions.set((Real)0.1, (Real)0.1, (Real)0.1);
	}
	/** The dimensions are the radius of the shape, meaning 1/2 extents in each dimension. */
	Vec3 dimensions;
};

/** ConvexMesh shape */
class ConvexMeshShape : public Shape {
protected:
	friend class ConvexMeshShapeDesc;
	const BoundingConvexMesh* pBoundingConvexMesh;
	
	void create(const ConvexMeshShapeDesc& desc);
	ConvexMeshShape(RigidBody& rigidBody);
public:
};

/** ConvexMesh shape description */
class ConvexMeshShapeDesc : public ShapeDesc {
protected:
	CREATE_FROM_OBJECT_DESC_1(ConvexMeshShape, ShapePtr, RigidBody&)
	virtual ~ConvexMeshShapeDesc() {}
public:
	ConvexMeshShapeDesc() {
		setToDefault();
	}
	void setToDefault() {
		ShapeDesc::setToDefault();
		vertices.clear();
	}
	/** Vertices of the mesh to be transformed into convex hull */
	std::vector<Vec3> vertices;
};

//----------------------------------------------------------------------------

typedef shared_ptr<Actor> ActorPtr;
typedef std::vector<Actor*> ActorSeq;

typedef shared_ptr<ActorDesc> ActorDescPtr;
typedef std::vector<ActorDescPtr> ActorDescSeq;

/** Actor interface */
class Actor {
protected:
	friend class ActorDesc;
	friend class Tiny;
	Tiny& tiny;
	Context& context;
	golem::Scene& scene;
	XMLContext* pXMLContext;
	bool owner;

	Actor(Tiny& tiny);

public:
	virtual ~Actor();
	/** Returns global pose */
	virtual Mat34 getGlobalPose() const = 0;
	/** Sets global pose */
	virtual void setGlobalPose(const Mat34& pose) = 0;
};

/** Actor description */
class ActorDesc {
protected:
	friend class Tiny;
	virtual ActorPtr create(Tiny& tiny) const = 0;
	ActorDesc() {
		setToDefault();
	}
public:
	virtual ~ActorDesc() {}
	void setToDefault() {
		globalPose.setId();
	}
	/** Global pose */
	Mat34 globalPose;
};


/** Rigid boody interface */
class RigidBody : public Actor {
protected:
	friend class RigidBodyDesc;
	friend class Shape;
	typedef golem::PrivateList<Shape*, ShapePtr> ShapeList;

	ShapeList shapeList;
	golem::Actor *pActor;
	bool kinematic;
	
	ShapePtr createShape(const Bounds* pBounds);
	void create(const RigidBodyDesc& desc);
	RigidBody(Tiny& tiny);
	virtual ~RigidBody();

public:
	/** Create a shape from description */
	Shape* createShape(const ShapeDescPtr& pShapeDesc);
	/** Releases a given shape */
	void releaseShape(Shape* shape);
	/** Returns shapes */
	ShapeSeq getShapes() const;

	/** Returns global pose */
	virtual Mat34 getGlobalPose() const;
	/** Sets global pose */
	virtual void setGlobalPose(const Mat34& pose);
	
	/** Sets group for all shapes */
	void setGroup(int group);
};

/** Rigid boody description */
class RigidBodyDesc : public ActorDesc {
protected:
	CREATE_FROM_OBJECT_DESC_1(RigidBody, ActorPtr, Tiny&)
	virtual ~RigidBodyDesc() {}
public:
	RigidBodyDesc() {
		setToDefault();
	}
	void setToDefault() {
		ActorDesc::setToDefault();
		shapes.clear();
		kinematic = false;
	}
	/** Shape descriptions */
	ShapeDescSeq shapes;
	/** Kinematic body not controlled by physics simulator */
	bool kinematic;
};

//----------------------------------------------------------------------------

typedef shared_ptr<Joint> JointPtr;
typedef std::vector<Joint*> JointSeq;

typedef shared_ptr<JointDesc> JointDescPtr;
typedef std::vector<JointDescPtr> JointDescSeq;

/** Joint interface */
class Joint : public RigidBody {
protected:
	friend class JointDesc;
	Controller& controller;

	void create(const JointDesc& desc);
	Joint(Controller& controller);
public:
	// TODO empty for now
};

/** Joint description */
class JointDesc : public RigidBodyDesc {
protected:
	friend class Controller;
	friend class Joint;
	golem::BodyActor* pJointActor;
	
	virtual ActorPtr create(Tiny& tiny) const;
	CREATE_FROM_OBJECT_DESC_1(Joint, JointPtr, Controller&)
	virtual ~JointDesc() {}

public:
	JointDesc() {
		setToDefault();
	}
	void setToDefault() {
		RigidBodyDesc::setToDefault();
		pJointActor = 0;
	}
};

/** Controller interface */
class Controller : public Actor {
protected:
	friend class ControllerDesc;
	friend class Joint;
	typedef golem::PrivateList<Joint*, JointPtr> JointList;

	golem::UIPlanner *pUIPlanner;
	JointList jointList;

	void create(const ControllerDesc& desc);
	void create(const UIPlannerDesc& desc);
	Controller(Tiny& tiny);
	virtual ~Controller();

public:
	/** Returns position in configuration space coordinates at time t (non-blocking call) */
	GenConfigspaceState recvGenConfigspaceState(double t) const;
	/** Returns global poses of reference frames in workspace coordinates at time t (non-blocking call) */
	GenWorkspaceChainState recvGenWorkspaceState(double t) const;
	
	/** Sends trajectory and waits for movement completion: if timeWait=0 returns when movement starts, if timeWait=INF returns when movement finishes (blocking call). */
	void send(const GenConfigspaceStateSeq& trajectory, double timeWait);
	/** Stops movement. */
	void stop();

	/** Finds (optimal) trajectory target in the obstacle-free configuration space (blocking call). */
	GenConfigspaceState findTarget(const GenConfigspaceState &begin, const GenWorkspaceChainState& end);
	/** Finds obstacle-free (global search) trajectory in the configuration space from begin to end (blocking call). */
	GenConfigspaceStateSeq findGlobalTrajectory(const GenConfigspaceState &begin, const GenConfigspaceState &end);
	/** Finds obstacle-free (local search) trajectory in the workspace form trajectory workspace increments (blocking call). */
	GenConfigspaceStateSeq findLocalTrajectory(const GenConfigspaceState &begin, const GenWorkspaceChainStateSeq &trajectory);
	
	/** Forward transformation for tool joints only */
	WorkspaceChainCoord getChainForwardTransform(const ConfigspaceCoord& cc) const;
	/** Forward transformation for all joints */
	WorkspaceJointCoord getJointForwardTransform(const ConfigspaceCoord& cc) const;
	/** Manipulator (spatial) Jacobian */
	Jacobian getJacobian(const ConfigspaceCoord& cc) const;

	/** Returns group of the shapes of the device */
	int getControllerGroup() const;
	/** Returns group of the shapes which can collide with the device */
	int getCollisionGroup() const;
	/** Sets group of the shapes which can collide with the device */
	void setCollisionGroup(int group);

	/** Controller joints */
	JointSeq getJoints() const;
	
	/** Returns global pose of the device */
	virtual Mat34 getGlobalPose() const;
	/** Sets global pose of the device */
	virtual void setGlobalPose(const Mat34& pose);

	/** Returns reference frames in local tool frames */
	WorkspaceChainCoord getReferencePose() const;
	/** Sets reference frames in local tool frames */
	void setReferencePose(const WorkspaceChainCoord& pose);
};

/** Controller description */
class ControllerDesc : public ActorDesc {
protected:
	CREATE_FROM_OBJECT_DESC_1(Controller, ActorPtr, Tiny&)
	virtual ~ControllerDesc() {}

public:
	ControllerDesc() {
		setToDefault();
	}
	void setToDefault() {
		ActorDesc::setToDefault();
		libraryPathCtrl = "GolemCtrlKatana300Sim"; // Katana 300 simulator
		configPathCtrl = "GolemCtrlKatana300Sim"; // Katana 300 simulator
		libraryPathPlanner = "GolemPlannerGraphPlanner"; // GraphPlanner
		configPathPlanner = "GolemPlannerKatana300"; // Katana 300
	}
	/** Controler library path: directory + name (without OS specific prefix and suffix) */
	std::string libraryPathCtrl;
	/** Controler XML configuration path */
	std::string configPathCtrl;
	/** Planner library path: directory + name (without OS specific prefix and suffix) */
	std::string libraryPathPlanner;
	/** Planner XML configuration path */
	std::string configPathPlanner;
};

//----------------------------------------------------------------------------

/** Katana 300/450 sensor data */
class KatanaSensorData {
public:
	/** Sensor index */
	int index;
	/** Sensor value */
	int value;
};
typedef std::vector<KatanaSensorData> KatanaSensorDataSet;

/** Katana 300/450 gripper encoder data */
class KatanaGripperEncoderData {
public:
	/** Open gripper encoder value */
	int open;
	/** Closed gripper encoder value */
	int closed;
	/** Current gripper encoder value */
	int current;
};

/** Katana 300/450 interface */
class Katana : public Controller {
protected:
	friend class KatanaDesc;

	golem::KatanaGripper* pKatanaGripper;

	void create(const KatanaDesc& desc);
	Katana(Tiny& tiny);

public:
	/** Receives gripper sensor values */
	KatanaSensorDataSet gripperRecvSensorData(double timeOut);
	/** Receives gripper encoder values */
	KatanaGripperEncoderData gripperRecvEncoderData(double timeOut);
	/** Opens the gripper */
	void gripperOpen(double timeOut);
	/** Closes the gripper, stops if only a signal from any sensor is above the given threshold */
	void gripperClose(const KatanaSensorDataSet& sensorThreshold, double timeOut);
	/** Freezes the gripper */
	void gripperFreeze(double timeOut);
};

/** Katana 300/450 description (creates Katana interface) */
class KatanaDesc : public ControllerDesc {
protected:
	CREATE_FROM_OBJECT_DESC_1(Katana, ActorPtr, Tiny&)
	virtual ~KatanaDesc() {}

public:
	KatanaDesc() {
		setToDefault();
	}
	void setToDefault() {
		ControllerDesc::setToDefault();
		libraryPathCtrl = "GolemCtrlKatana300"; // Katana 300
		configPathCtrl = "GolemCtrlKatana300"; // Katana 300
		bGripper = false;
		sensorIndexSet.clear();
		// Katana Finger Type S03.02, force sensors
		sensorIndexSet.push_back(7);	// Right finger, Front
		sensorIndexSet.push_back(15);	// Left finger, Front
		sensorIndexSet.push_back(6);	// Right finger, Rear
		sensorIndexSet.push_back(14);	// Left finger, Rear
	}
	/** Katana gripper */
	bool bGripper;
	/** Katana sensors */
	std::vector<int> sensorIndexSet;
};

//------------------------------------------------------------------------------

/** Golem Tiny - a "tiny" version of Golem framework
*/
class Tiny {
protected:
	friend class Actor;
	typedef golem::PrivateList<Actor*, ActorPtr> ActorList;

	golem::shared_ptr<XMLParser> pParser;
	XMLContext* pXMLContext;
	golem::shared_ptr<Context> pContext;
	golem::shared_ptr<golem::Universe> pUniverse;
	golem::Scene* pScene;
	ActorList actorList;

public:
	/** Constructs Golem Tiny */
	Tiny(int argc, char *argv[]);
	/** Releases Golem Tiny objects */
	~Tiny();
	
	/** Current local time */
	double getTime() const;
	/** Sleeps for a given time */
	void sleep(double duration) const;
	/** Prints a message */
	void print(const char* format, ...);
	/** Interrupted */
	bool interrupted() const;
	/** Read a key, wait no longer than timeOut */
	int waitKey(double timeOut = numeric_const<double>::MAX);

	/** XML context (default XML configuration file) */
	const golem::XMLContext* getXMLContext() const;
	golem::XMLContext* getXMLContext();
	/** Golem context */
	const golem::Context* getContext() const;
	golem::Context* getContext();
	
	/** Create an Actor from description */
	Actor* createActor(const ActorDescPtr& pActorDesc);
	/** Releases a given Actor */
	void releaseActor(Actor* actor);
	/** Returns Actors */
	ActorSeq getActors() const;
	
	/** Bang! */
	void bang();
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

#endif /*_GOLEM_TINY_TINY_H_*/
