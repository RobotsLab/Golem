/** @file TinyGrasp.cpp
 * 
 * Program demonstrating Tiny Golem interface
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Tiny/Tiny.h>
#include <Golem/App/TinyGrasp/TinyGrasp.h>
#include <Golem/Math/Rand.h>
#include <Golem/Math/Quat.h>
#include <Golem/Phys/Data.h>
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>
#include <iostream>

using namespace golem;
using namespace golem::tiny;

//------------------------------------------------------------------------------

PoseError::PoseError(Real lin, Real ang) : lin(lin), ang(ang) {
}

PoseError::PoseError(const Mat34& a, const Mat34& b) {
	lin = getLinearDist(a.p, b.p);
	ang = getAngularDist(Quat(a.R), Quat(b.R));
}

Real PoseError::getLinearDist(const Vec3& v0, const Vec3& v1) {
	return v0.distance(v1);
}

Real PoseError::getAngularDist(const Quat& q0, const Quat& q1) {
	const Real d = q0.dot(q1);
	return REAL_ONE - ::fabs(d);
}

//------------------------------------------------------------------------------

namespace golem {
namespace tiny {

class Debug: public Object {
private:
	CriticalSection cs;
	DebugRenderer debugRenderer;

	Debug(Scene &scene) : Object(scene) {
	}
	void create(const Object::Desc& desc) {
		Object::create(desc);
	}
	void render() {
		CriticalSectionWrapper csw(cs);
		debugRenderer.render();
	}

public:
	/** Object description */
	class Desc : public Object::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(Debug, Object::Ptr, Scene&)
	};

	void render(const DebugRenderer* debugRenderer = NULL) {
		CriticalSectionWrapper csw(cs);
		if (debugRenderer)
			this->debugRenderer = *debugRenderer;
		else
			this->debugRenderer.reset();
	}
};

};
};

TinyEx::TinyEx(int argc, char *argv[]) : Tiny(argc, argv) {
	Debug::Desc desc; 
	debug = dynamic_cast<Debug*>(pScene->createObject(desc));
}
void TinyEx::render(const DebugRenderer* debugRenderer) {
	debug->render(debugRenderer);
}

XMLContext* TinyEx::getXMLContext() {
	return pXMLContext;
}

//------------------------------------------------------------------------------

TinyGrasp::TinyGrasp() :
	objectGrasped(NULL),
	object(NULL)
{
	int argc = 1;
	char* argv [] = {(char*)"GolemAppTinyGrasp"};
	tiny.reset(new TinyEx(argc, argv));

	// XML data
	XMLContext* context = tiny->getXMLContext()->getContextFirst("robot");	
	XMLData(controllerPose, context->getContextFirst("controller pose"));
	XMLData("planner_tries", plannerTries, context->getContextFirst("controller"));
	XMLData("approach_offset", approachOffset, context->getContextFirst("controller grasping"));
	XMLData("grasp_offset", graspOffset, context->getContextFirst("controller grasping"));
	XMLData("error_lin", graspError.lin, context->getContextFirst("controller grasping"));
	XMLData("error_ang", graspError.ang, context->getContextFirst("controller grasping"));
	XMLData("sensor_threshold", sensorThreshold, context->getContextFirst("controller grasping"));
	XMLData("encoder_threshold", encoderThreshold, context->getContextFirst("controller grasping"));
	XMLData("robot_distance", robotDistance, context->getContextFirst("controller grasping"));

	// create controller
	KatanaDesc* pControllerDesc = new KatanaDesc; // specialised Katana 300/450 description
	//ControllerDesc* pControllerDesc = new ControllerDesc; // generic description
	pControllerDesc->globalPose = controllerPose;
	pControllerDesc->libraryPathCtrl = "GolemCtrlKatana300Sim"; // specify library path
	pControllerDesc->configPathCtrl = "GolemCtrlKatana300"; // specify xml config path
	pControllerDesc->libraryPathPlanner = "GolemPlannerGraphPlanner"; // specify library path
	pControllerDesc->configPathPlanner = "GolemPlannerKatana300"; // specify xml config path
	pControllerDesc->bGripper = true;
	tiny->print("Creating the controller...\n");
	//controller = (Katana*)tiny->createActor(ActorDescPtr(pControllerDesc));
	controller = dynamic_cast<Katana*>(tiny->createActor(ActorDescPtr(pControllerDesc)));
	if (controller == NULL)
		throw ExTiny("TinyGrasp::TinyGrasp(): Katana driver required!");

	// get sensor data assuming no object is in the gripper
	zero = controller->gripperRecvSensorData(numeric_const<double>::INF);
	
	// robot base
	RigidBodyDesc* pBaseDesc = new RigidBodyDesc;
	pBaseDesc->kinematic = true;
	XMLData(pBaseDesc->globalPose, context->getContextFirst("base pose"));
	BoxShapeDesc* pBaseShapeDesc = new BoxShapeDesc;
	XMLData(pBaseShapeDesc->dimensions, context->getContextFirst("base dimensions"));
	pBaseDesc->shapes.push_back(ShapeDescPtr(pBaseShapeDesc));
	tiny->createActor(ActorDescPtr(pBaseDesc));
	
	// robot beam
	RigidBodyDesc* pBeamDesc = new RigidBodyDesc;
	pBeamDesc->kinematic = true;
	XMLData(pBeamDesc->globalPose, context->getContextFirst("beam pose"));
	BoxShapeDesc* pBeamShapeDesc = new BoxShapeDesc;
	XMLData(pBeamShapeDesc->dimensions, context->getContextFirst("beam dimensions"));
	pBeamDesc->shapes.push_back(ShapeDescPtr(pBeamShapeDesc));
	tiny->createActor(ActorDescPtr(pBeamDesc));

	// attached a finger to the end-effector (the last joint)
	Joint* effector = controller->getJoints().back();
	// get the end-effector reference pose
	Mat34 referencePose = controller->getReferencePose().c[chain];
	// construct a finger from a box and a sphere in local end-effector coordinates (arm at reference configuration stretched along Y-axis)
	Real fingerLength = 0.08;
	XMLData("length", fingerLength, context->getContextFirst("controller gripper"));
	Real fingerDiam = 0.015;
	XMLData("diam", fingerDiam, context->getContextFirst("controller gripper"));
	Real fingerGap = 0.08;
	XMLData("gap", fingerGap, context->getContextFirst("controller gripper"));
	
	BoxShapeDesc* pFingerLeftShapeDesc = new BoxShapeDesc;
	pFingerLeftShapeDesc->dimensions.set(Real(fingerDiam/2.0), Real(fingerLength/2.0), Real(fingerDiam/2.0));
	pFingerLeftShapeDesc->localPose = referencePose;
	pFingerLeftShapeDesc->localPose.p.v1 += Real(fingerGap/2.0);
	pFingerLeftShapeDesc->localPose.p.v2 += Real(fingerLength/2.0);
	(void)effector->createShape(ShapeDescPtr(pFingerLeftShapeDesc));
	BoxShapeDesc* pFingerRightShapeDesc = new BoxShapeDesc;
	pFingerRightShapeDesc->dimensions.set(Real(fingerDiam/2.0), Real(fingerLength/2.0), Real(fingerDiam/2.0));
	pFingerRightShapeDesc->localPose = referencePose;
	pFingerRightShapeDesc->localPose.p.v1 -= Real(fingerGap/2.0);
	pFingerRightShapeDesc->localPose.p.v2 += Real(fingerLength/2.0);
	(void)effector->createShape(ShapeDescPtr(pFingerRightShapeDesc));
}

//------------------------------------------------------------------------------

RigidBody* TinyGrasp::createObject(const Mat34& pose, const Vec3& dimensions) {
	RigidBodyDesc* pObjectDesc = new RigidBodyDesc;
	pObjectDesc->kinematic = true;
	pObjectDesc->globalPose = pose;
	BoxShapeDesc* pObjectShapeDesc = new BoxShapeDesc;
	pObjectShapeDesc->dimensions = dimensions;
	pObjectDesc->shapes.push_back(ShapeDescPtr(pObjectShapeDesc));

	RigidBody* object = dynamic_cast<RigidBody*>(tiny->createActor(ActorDescPtr(pObjectDesc)));
	if (object == NULL)
		throw ExTiny("TinyGrasp::createObject(): Oops");
	return object;
}

void TinyGrasp::removeObject(tiny::RigidBody*& object) {
	if (object) {
		tiny->releaseActor(object);
		object = NULL;
	}
}

void TinyGrasp::attachObject() {
	if (objectGrasped)
		throw ExTiny("TinyGrasp::attachObject(): Object has been already attached");
	if (object == NULL)
		throw ExTiny("TinyGrasp::attachObject(): No object to attach");

	BoxShapeDesc* pObjectShapeDesc = dynamic_cast<BoxShapeDesc*>(object->getShapes().back()->getDesc().get());
	if (pObjectShapeDesc == NULL)
		throw ExTinyShape("TinyGrasp::attachObject(): Must be box shape");
	BoxShapeDesc* pBoxShapeDesc = new BoxShapeDesc(*pObjectShapeDesc);

	// objectPose = toolPose * localPose => localPose = toolPose^-1 * objectPose
	Mat34 pose = controller->getChainForwardTransform(controller->recvGenConfigspaceState(numeric_const<double>::INF).pos).c[chain];
	pBoxShapeDesc->localPose.setInverseRT(pose);
	pBoxShapeDesc->localPose.multiply(pBoxShapeDesc->localPose, object->getGlobalPose());

	Joint* effector = controller->getJoints().back();
	objectGrasped = effector->createShape(ShapeDescPtr(pBoxShapeDesc));

	// HACK: move object pose to infinity
	pose.setId();
	pose.p.z = 1e2; // not too high
	pose.p.x = 1e2; // not too high
	object->setGlobalPose(pose);
}

void TinyGrasp::releaseObject() {
	if (objectGrasped == NULL || object == NULL)
		throw ExTiny("TinyGrasp::releaseObject(): Object has not been attached");

	// objectPose = toolPose * localPose
	Mat34 pose = controller->getChainForwardTransform(controller->recvGenConfigspaceState(numeric_const<double>::INF).pos).c[chain];
	pose.multiply(pose, objectGrasped->getLocalPose());

	// remove shape
	Joint* effector = controller->getJoints().back();
	effector->releaseShape(objectGrasped);
	objectGrasped = NULL;

	// restore object pose
	object->setGlobalPose(pose);
}

//------------------------------------------------------------------------------

Mat34 TinyGrasp::read() {
	return TinyGrasp::getToolPose();
}

Mat34 TinyGrasp::moveTry(const Mat34& pose) {
	// compute movement end/target in joint configuration space
	cbegin = controller->recvGenConfigspaceState(numeric_const<double>::INF);
	end.pos.c[chain] = pose;
	end.t = cbegin.t + 2.0; // movement will last no shorter than 1 sec
	cend = controller->findTarget(cbegin, end);
	
	// compute actual pose, note that the target is always computed with respect to the reference pose in the tool frame (the last joint)
	Mat34 actual;
	actual.multiply(controller->getChainForwardTransform(cend.pos).c[chain], controller->getReferencePose().c[chain]);
	return actual;
}

void TinyGrasp::moveExec(Real duration) {
	// compute trajectory using path planning with collision detection
	GenConfigspaceStateSeq trajectory;
	
	for (U32 i = 0;; ++i) {
		try {		
			tiny->print("TinyGrasp::moveExec(): %u...\n", i + 1);
			cend.t = cbegin.t + duration;
			trajectory = controller->findGlobalTrajectory(cbegin, cend);
		}
		catch (ExTiny& ex) {
			if (i < plannerTries) continue;
			throw ex;
		}
		break;
	}

	// move the controller and wait until it stops
	tiny->print("TinyGrasp::moveExec(): OK\n");
	controller->send(trajectory, numeric_const<double>::INF);
}

//------------------------------------------------------------------------------

void TinyGrasp::setGraspObject(RigidBody* object) {
	if (object == NULL)
		throw ExTiny("TinyGrasp::setGraspObject(): Null pointer");
	this->object = object;
}

GraspPose::Seq TinyGrasp::getGraspPoses() const {
	BoxShapeDesc* pObjectShapeDesc = getObjectBoxDesc(object);
	const Mat34 pose = object->getGlobalPose();
	const Vec3 dimensions = pObjectShapeDesc->dimensions;

	GraspPose graspPoses[2][4];
	Real min = numeric_const<Real>::MAX;
	for (U32 i = 0; i < 3; ++i) {
		if (min > dimensions[i]) {
			min = dimensions[i];
			
			for (U32 j = 0; j < 2; ++j) {
				// object approach axis index
				const U32 k = (i + j + 1)%3;

				// debug info
				//const char* coords [] = {"X", "Y", "Z"};
				//tiny->print("dim[%s] -> %s\n", coords[i], coords[k]);

				// local poses: 0 - approach from positive to negative, 1 - from negative to positive
				GraspPose* gp = graspPoses[j];
				switch (k) {
				case 0:	// X
					gp[0].approach.R = i == 1 ? rotAxis(2, +REAL_PI_2) : rotAxis(0, -REAL_PI_2, 2, +REAL_PI_2);
					gp[2].approach.R = i == 1 ? rotAxis(2, -REAL_PI_2) : rotAxis(0, -REAL_PI_2, 2, -REAL_PI_2);
					break;
				case 1:	// Y
					gp[0].approach.R = i == 0 ? rotAxis(0, +REAL_PI) : rotAxis(1, -REAL_PI_2, 0, +REAL_PI);
					gp[2].approach.R = i == 0 ? rotAxis(0, +REAL_ZERO) : rotAxis(1, -REAL_PI_2);
					break;
				case 2:	// Z
					gp[0].approach.R = i == 0 ? rotAxis(0, -REAL_PI_2) : rotAxis(2, -REAL_PI_2, 0, -REAL_PI_2);
					gp[2].approach.R = i == 0 ? rotAxis(2, +REAL_PI, 0, +REAL_PI_2) : rotAxis(2, +REAL_PI_2, 0, +REAL_PI_2);
					break;
				}

				for (U32 l = 0; l < 2; ++l) {
					const U32 m = 2*l;

					gp[m].approach.p.setZero();
					gp[m].grasp = gp[m].approach;
					gp[m].approach.p[k] = (dimensions[k] + approachOffset)*(Real(1) - m);
					gp[m].grasp.p[k] = (dimensions[k] + graspOffset)*(Real(1) - m);
				
					gp[m + 1].approach = gp[m].approach;
					gp[m + 1].approach.R.multiply(rotAxis(k, REAL_PI), gp[m + 1].approach.R);
					gp[m + 1].grasp = gp[m].grasp;
					gp[m + 1].grasp.R.multiply(rotAxis(k, REAL_PI), gp[m + 1].grasp.R);

					gp[m].approach.multiply(pose, gp[m].approach);
					gp[m].grasp.multiply(pose, gp[m].grasp);
					gp[m + 1].approach.multiply(pose, gp[m + 1].approach);
					gp[m + 1].grasp.multiply(pose, gp[m + 1].grasp);
				}
			}
		}
	}

	GraspPose::Seq poses;
	DebugRenderer debugRenderer;

	for (U32 i = 0; i < 2*4; ++i) {
		poses.push_back(graspPoses[i/4][i%4]);
		debugRenderer.addAxes(graspPoses[i/4][i%4].approach, Vec3(0.05));
		debugRenderer.addAxes(graspPoses[i/4][i%4].grasp, Vec3(0.05));
	}
	
	tiny->render(&debugRenderer);

	return poses;
}

bool TinyGrasp::graspTry(const GraspPose::Seq& poses) {	
	GraspPose::Seq::const_iterator index = poses.begin();
	PoseError errorMin(moveTry(index->grasp), index->grasp);

	for (GraspPose::Seq::const_iterator i = ++poses.begin(); i != poses.end(); ++i) {
		PoseError error(moveTry(i->grasp), i->grasp);
		if (error.ang < errorMin.ang && error.lin < graspError.lin || error.lin < errorMin.lin && error.ang < graspError.ang) {
			errorMin = error;
			index = i;
		}
	}

	return graspTry(*index);
}

bool TinyGrasp::graspTry(const GraspPose& pose) {
	gend = pose;

	for (;!tiny->interrupted();) {
		GraspPose actual;
		actual.grasp = moveTry(gend.grasp);
		actual.approach = moveTry(gend.approach);

		const PoseError error(actual.grasp, gend.grasp);
		if (error.lin < graspError.lin && error.ang < graspError.ang)
			return true;

		tiny->print("TinyGrasp::graspTry(): FAILED: error = (%f, %f)\n", error.lin, error.ang);
	}
	
	return false;
}

bool TinyGrasp::graspExec(Real duration) {
	Mat34 actual;

	gripperOpen();

	actual = moveTry(gend.approach);
	PoseError approachError(actual, gend.approach);
	tiny->print("TinyGrasp::graspExec(): approach error = (%f, %f)\n", approachError.lin, approachError.ang);
	moveExec(duration);

	actual = moveTry(gend.grasp);
	PoseError graspError(actual, gend.grasp);
	tiny->print("TinyGrasp::graspExec(): grasp error = (%f, %f)\n", graspError.lin, graspError.ang);
	moveExec();

	Mat34 pose = read();
	gapproach = toBody(pose, diff(pose, gend.approach));

	gripperClose();

	const KatanaGripperEncoderData data = controller->gripperRecvEncoderData(numeric_const<double>::INF);
	const I32 error = Math::abs(data.closed - data.current);
	if (error < encoderThreshold && data.open != data.closed) {
		tiny->print("TinyGrasp::graspExec(): FAILED: error = %i\n", error);
		return false;
	}

	// OBJECT: open=31000, closed=-16768, current=19300
	// NO OBJECT: open=31000, closed=-16768, current=11493
	//tiny->print("TinyGrasp::graspExec(): SUCCESS: (open=%i, closed=%i, current=%i)\n", data.open, data.closed, data.current);

	attachObject();
	tiny->render();

	return true;
}

void TinyGrasp::graspRelease() {
	gripperOpen();

	releaseObject();
	
	Mat34 pose = read();
	pose.multiply(fromBody(pose, gapproach), pose);

	(void)moveTry(pose);
	moveExec();

	tiny->render();
}

//------------------------------------------------------------------------------

Mat23Seq TinyGrasp::getRobotPoses(const GraspPose::Seq& poses) const {
	if (object == NULL)
		throw ExTiny("TinyGrasp::getRobotPoses(): No objects to process");

	const Mat34 pose = object->getGlobalPose();
	std::map<Real, Vec2> projections;
	for (GraspPose::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i) {
		Vec3 p3;
		p3.subtract(pose.p, i->grasp.p);
		Vec2 p2(p3.x, p3.y);
		projections[p2.magnitude()] = p2;

		//tiny->print("x=%f, y=%f\n", p2.x, p2.y);
	}
	
	if (projections.empty() || projections.rbegin()->first < REAL_EPS)
		throw ExTiny("TinyGrasp::getRobotPoses(): Invalid grasp points");

	Vec2 o(pose.p.x, pose.p.y);
	Vec2 p[2] = {projections.rbegin()->second, -projections.rbegin()->second, };
	Vec2 n[2] = {p[0], p[1], };

	n[0].normalise();
	p[0].multiplyAdd(robotDistance, n[0], p[0]);
	p[0].add(o, p[0]);
	
	n[1].normalise();
	p[1].multiplyAdd(robotDistance, n[1], p[1]);
	p[1].add(o, p[1]);
	
	Real a[2] = {Math::atan2(n[1].y, n[1].x), Math::atan2(n[0].y, n[0].x), }; // must be reverse
	
	//tiny->print("x=%f, y=%f, a=%f\n", p[0].x, p[0].y, a[0]);
	//tiny->print("x=%f, y=%f, a=%f\n", p[1].x, p[1].y, a[1]);

	Mat23Seq robotPoses;
	robotPoses.push_back(Mat23(a[0], p[0]));
	robotPoses.push_back(Mat23(a[1], p[1]));
	return robotPoses;
}

//------------------------------------------------------------------------------

void TinyGrasp::gripperOpen() {
	controller->gripperOpen(numeric_const<double>::INF);
}

void TinyGrasp::gripperClose() {
	KatanaSensorDataSet threshold = zero;
	for (KatanaSensorDataSet::iterator i = threshold.begin(); i != threshold.end(); ++i)
		i->value += sensorThreshold;
	controller->gripperClose(threshold, numeric_const<double>::INF);
}

//------------------------------------------------------------------------------

Mat34 TinyGrasp::getToolPose() {
	return controller->recvGenWorkspaceState(numeric_const<double>::INF).pos.c[chain];
}

BoxShapeDesc* TinyGrasp::getObjectBoxDesc(RigidBody* object) const {
	if (object == NULL)
		throw ExTiny("TinyGrasp::getObjectBoxDesc(): No objects to process");

	BoxShapeDesc* pObjectShapeDesc = dynamic_cast<BoxShapeDesc*>(object->getShapes().back()->getDesc().get());
	if (pObjectShapeDesc == NULL)
		throw ExTinyShape("TinyGrasp::getObjectBoxDesc(): No box shape");

	return pObjectShapeDesc;
}

Mat33 TinyGrasp::rotAxis(U32 axis, Real angle) {
	Mat33 R;
	switch (axis) {
	case 0: R.rotX(angle); break;
	case 1: R.rotY(angle); break;
	default: R.rotZ(angle); break;
	}
	return R;
}

Mat33 TinyGrasp::rotAxis(U32 axis1, Real angle1, U32 axis2, Real angle2) {
	Mat33 R1 = rotAxis(axis1, angle1);
	Mat33 R2 = rotAxis(axis2, angle2);
	R1.multiply(R1, R2);
	return R1;
}

Mat34 TinyGrasp::diff(const Mat34& a, const Mat34& b) {
	Mat34 ab;
	ab.setInverseRT(a); // a.R*(a.R)^T = Id, det(a.R) = 1 (rotation matrix)
	ab.multiply(b, ab);
	return ab;
}

Mat34 TinyGrasp::toBody(const Mat34& A, const Mat34& G) {
	Mat34 H;
	H.setInverseRT(A);
	H.multiply(H, G);
	H.multiply(H, A);
	return H;
}

Mat34 TinyGrasp::fromBody(const Mat34& A, const Mat34& H) {
	Mat34 G;
	G.setInverseRT(A);
	G.multiply(H, G);
	G.multiply(A, G);
	return G;
}

//------------------------------------------------------------------------------
