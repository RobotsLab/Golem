/** @file TinyIce.ice
 * 
 * Golem TinyIce interface in ZeroC Ice.
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#ifndef TINYICE_ICE
#define TINYICE_ICE

module golem {
module tinyice {

//----------------------------------------------------------------------------

/** Tiny exceptions */

exception ExTiny { string what; };

exception ExTinyCreate extends ExTiny {};

exception ExTinyActor extends ExTiny {};
exception ExTinyActorCreate extends ExTinyActor {};
exception ExTinyActorNotFound extends ExTinyActor {};
exception ExTinyActorNotRemovable extends ExTinyActor {};

exception ExTinyShape extends ExTiny {};
exception ExTinyShapeCreate extends ExTinyShape {};
exception ExTinyShapeNotFound extends ExTinyShape {};
exception ExTinyShapeNotRemovable extends ExTinyShape {};

exception ExTinyController extends ExTiny {};
exception ExTinyControllerSend extends ExTinyController {};
exception ExTinyControllerRecv extends ExTinyController {};
exception ExTinyControllerFindTarget extends ExTinyController {};
exception ExTinyControllerFindTrajectory extends ExTinyController {};

exception ExTinyKatana extends ExTinyController {};
exception ExTinyKatanaGripperNotPresent extends ExTinyKatana {};
exception ExTinyKatanaGripperIOError extends ExTinyKatana {};

//----------------------------------------------------------------------------

sequence<byte> ByteSeq;
sequence<int> IntSeq;
sequence<double> DoubleSeq;

/** 3 Vector (translation or point) */
struct Vec3 {
	double v1;
	double v2;
	double v3;
};
sequence<Vec3> Vec3Seq;

/** 3x3 Matrix (rigid body rotation) */
struct Mat33 {
	double m11;
	double m12;
	double m13;

	double m21;
	double m22;
	double m23;

	double m31;
	double m32;
	double m33;
};
sequence<Mat33> Mat33Seq;

/** 3x4 Matrix (rigid body transformation)  */
struct Mat34 {
	/** rotation matrix	*/
	Mat33 R;
	/** translation	*/
	Vec3 p;
};
sequence<Mat34> Mat34Seq;

/** Twist */
struct Twist {
	/** linear component */
	Vec3 v;
	/** angular component (rotation axis = w/|w| and rotation speed = |w|) */
	Vec3 w;
};
sequence<Twist> TwistSeq;

/** Manipulator Jacobian (in twist coordinates). */
struct Jacobian {
	/** coordinates */
	TwistSeq j;
};
sequence<Jacobian> JacobianSeq;

//----------------------------------------------------------------------------

/** Configuration space coordinates */
struct ConfigspaceCoord {
	/** coordinates */
	DoubleSeq c;
};
sequence<ConfigspaceCoord> ConfigspaceCoordSeq;

/** Chainspace coordinates */
struct ChainspaceCoord {
	/** coordinates */
	DoubleSeq c;
};
sequence<ChainspaceCoord> ChainspaceCoordSeq;

/** Generalized configuration space state */
struct GenConfigspaceState {
	/** position */
	ConfigspaceCoord pos;
	/** velocity */
	ConfigspaceCoord vel;
	/** time */
	double t;
};
sequence<GenConfigspaceState> GenConfigspaceStateSeq;


/** Workspace coordinates */
struct WorkspaceCoord {
	/** coordinates */
	Mat34Seq c;
};
sequence<WorkspaceCoord> WorkspaceCoordSeq;

/** Generalized workspace state */
struct GenWorkspaceState {
	/** pose */
	WorkspaceCoord pos;
	/** time */
	double t;
};
sequence<GenWorkspaceState> GenWorkspaceStateSeq;

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
struct RGBA {
	float r;
	float g;
	float b;
	float a;
};

/** Shape description */
class ShapeDesc {
	/** Local pose */
	Mat34 localPose;
	/** Density */
	double density;
	/** Shape group */
	int group;
	/** Color */
	RGBA color;
};
sequence<ShapeDesc> ShapeDescSeq;

/** Shape interface */
interface Shape {
	/** Shape type */
	["cpp:const"] idempotent ShapeType getType();
	/** Shape color */
	["cpp:const"] idempotent RGBA getColor();
	/** Local pose */
	["cpp:const"] idempotent Mat34 getLocalPose();
	/** Returns collision group */
	["cpp:const"] idempotent int getGroup();
	/** Sets collision group */
	void setGroup(int group);
	/** Description */
	["cpp:const"] idempotent ShapeDesc getDesc();
};
sequence<Shape*> ShapeSeq;


/** Plane shape description */
class PlaneShapeDesc extends ShapeDesc {
	/** The plane normal vector. */
	Vec3 normal;
	/** The distance from the origin. */
	double distance;
};

/** Plane shape interface */
interface PlaneShape extends Shape {
	// TODO interface
};


/** Sphere shape description */
class SphereShapeDesc extends ShapeDesc {
	/** The radius of the sphere. */
	double radius;
};

/** Sphere shape interface */
interface SphereShape extends Shape {
	// TODO interface
};


/** Cylinder shape description */
class CylinderShapeDesc extends ShapeDesc {
	/** The radius of the cylinder. */
	double radius;
	/** The length of the cylinder. */
	double length;
};

/** Cylinder shape interface */
interface CylinderShape extends Shape {
	// TODO interface
};


/** Box shape description */
class BoxShapeDesc extends ShapeDesc {
	/** The dimensions are the radius of the shape, meaning 1/2 extents in each dimension. */
	Vec3 dimensions;
};

/** Box shape interface */
interface BoxShape extends Shape {
	// TODO interface
};


/** ConvexMesh shape description */
class ConvexMeshShapeDesc extends ShapeDesc {
	/** Vertices of the mesh to be transformed into convex hull */
	Vec3Seq vertices;
};

/** ConvexMesh shape interface */
interface ConvexMeshShape extends Shape {
	// TODO interface
};

//----------------------------------------------------------------------------

/** Actor description */
class ActorDesc {
	/** Global pose */
	Mat34 globalPose;
};

/** Actor interface */
interface Actor {
	/** Returns global pose */
	["cpp:const"] idempotent Mat34 getGlobalPose();
	/** Sets global pose */
	void setGlobalPose(Mat34 pose);
};
sequence<Actor*> ActorSeq;

//----------------------------------------------------------------------------

/** Rigid boody description */
class RigidBodyDesc extends ActorDesc {
	/** Shape descriptions */
	ShapeDescSeq shapes;
	/** Kinematic body not controlled by physics simulator */
	bool kinematic;
};

/** Rigid boody interface */
interface RigidBody extends Actor {
	/** Create a shape from description */
	Shape* createShape(ShapeDesc desc) throws ExTiny;
	/** Releases a given shape */
	void releaseShape(Shape* pShape) throws ExTiny;
	/** Returns shapes */
	["cpp:const"] idempotent ShapeSeq getShapes();
	
	/** Sets collision group for all shapes */
	void setGroup(int group);
};
sequence<RigidBody*> RigidBodySeq;

//----------------------------------------------------------------------------

/** Joint description */
class JointDesc extends RigidBodyDesc {
	// TODO empty for now
};

/** Joint interface */
interface Joint extends RigidBody {
	// TODO interface
};
sequence<Joint*> JointSeq;


/** Controller description */
class ControllerDesc extends ActorDesc {
	/** Controller library path: directory + name (without OS specific prefix and suffix) */
	string libraryPathCtrl;
	/** Controller XML configuration path */
	string configPathCtrl;
	/** Planner library path: directory + name (without OS specific prefix and suffix) */
	string libraryPathPlanner;
	/** Planner XML configuration path */
	string configPathPlanner;
};

/** Controller interface */
interface Controller extends Actor {
	/** Returns position in configuration space coordinates at time t (non-blocking call) */
	GenConfigspaceState recvGenConfigspaceState(double t) throws ExTinyController;
	/** Returns global poses of reference frames in workspace coordinates at time t (non-blocking call) */
	GenWorkspaceState recvGenWorkspaceState(double t) throws ExTinyController;
	
	/** Sends trajectory and waits for movement completion: if timeWait=0 returns when movement starts, if timeWait=INF returns when movement finishes (blocking call). */
	void send(GenConfigspaceStateSeq trajectory, double timeWait) throws ExTinyController;
	/** Stops movement. */
	void stop() throws ExTinyController;

	/** Finds (optimal) trajectory target in the obstacle-free configuration space (blocking call). */
	GenConfigspaceState findTarget(GenConfigspaceState begin, GenWorkspaceState end) throws ExTinyController;
	/** Finds obstacle-free (global search) trajectory in the configuration space from begin to end (blocking call). */
	GenConfigspaceStateSeq findGlobalTrajectory(GenConfigspaceState begin, GenConfigspaceState end) throws ExTinyController;
	/** Finds obstacle-free (local search) trajectory in the workspace form trajectory workspace increments (blocking call). */
	GenConfigspaceStateSeq findLocalTrajectory(GenConfigspaceState begin, GenWorkspaceStateSeq trajectory) throws ExTinyController;

	/** Forward transformation for tool joints only */
	["cpp:const"] idempotent WorkspaceCoord getChainForwardTransform(ConfigspaceCoord cc);
	/** Forward transformation for all joints */
	["cpp:const"] idempotent WorkspaceCoord getJointForwardTransform(ConfigspaceCoord cc);
	/** Manipulator (spatial) Jacobian */
	["cpp:const"] idempotent Jacobian getJacobian(ConfigspaceCoord cc);

	/** Returns group of the shapes of the device */
	["cpp:const"] idempotent int getControllerGroup();
	/** Returns group of the shapes which can collide with the device */
	["cpp:const"] idempotent int getCollisionGroup();
	/** Sets group of the shapes which can collide with the device */
	void setCollisionGroup(int collisionGroup);

	/** Controller joints */
	["cpp:const"] idempotent JointSeq getJoints();
	
	/** Returns reference frames in local tool frames */
	["cpp:const"] idempotent WorkspaceCoord getReferencePose();
	/** Sets reference frames in local tool frames */
	void setReferencePose(WorkspaceCoord pose);
};

//----------------------------------------------------------------------------

/** Katana 300/450 sensor data */
struct KatanaSensorData {
	/** Sensor index */
	int index;
	/** Sensor value */
	int value;
};
sequence<KatanaSensorData> KatanaSensorDataSet;

/** Katana 300/450 gripper encoder data */
struct KatanaGripperEncoderData {
	/** Open gripper encoder value */
	int open;
	/** Closed gripper encoder value */
	int closed;
	/** Current gripper encoder value */
	int current;
};

/** Katana 300/450 description (creates Katana interface) */
class KatanaDesc extends ControllerDesc {
	/** Katana gripper */
	bool bGripper;
	/** Katana sensors */
	IntSeq sensorIndexSet;
};

/** Katana 300/450 interface */
interface Katana extends Controller {
	/** Receives gripper sensor values */
	KatanaSensorDataSet gripperRecvSensorData(double timeOut) throws ExTinyKatana;
	/** Receives gripper encoder values */
	KatanaGripperEncoderData gripperRecvEncoderData(double timeOut) throws ExTinyKatana;
	/** Opens the gripper */
	void gripperOpen(double timeOut) throws ExTinyKatana;
	/** Closes the gripper, stops if only a signal from any sensor is above the given threshold */
	void gripperClose(KatanaSensorDataSet sensorThreshold, double timeOut) throws ExTinyKatana;
	/** Freezes the gripper */
	void gripperFreeze(double timeOut) throws ExTinyKatana;
};

//----------------------------------------------------------------------------

interface Tiny {
	/** Current local time */
	["cpp:const"] double getTime();
	/** Sleeps for a given time */
	["cpp:const"] void sleep(double duration);
	
	/** Returns true if universe has been interrupted */
	bool interrupted();
	
	/** Create an Actor from description */
	Actor* createActor(ActorDesc desc) throws ExTiny;
	/** Releases a given Actor */
	void releaseActor(Actor* pActor) throws ExTiny;
	/** Returns Actors */
	["cpp:const"] ActorSeq getActors();
	
	/** Bang! */
	void bang();
};

//----------------------------------------------------------------------------

};
};

#endif /*TINYICE_ICE*/
