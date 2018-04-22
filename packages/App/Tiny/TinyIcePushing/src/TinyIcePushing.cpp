/** @file TinyIcePushing.cpp
 * 
 * Example client for Golem TinyIce interface.
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

//------------------------------------------------------------------------------

#include <Ice/Ice.h>
#include <Golem/TinyIce/TinyIce.hh>
#include <Golem/TinyIce/Desc.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

using namespace golem::tinyice;

//------------------------------------------------------------------------------

void rotX(Mat33& m, double angle) {
	const double s = ::sin(angle), c = ::cos(angle);
	
	m.m11 = 1.;		m.m12 = 0.;		m.m13 = 0.;
	m.m21 = 0.;		m.m22 = c;		m.m23 = -s;
	m.m31 = 0.;		m.m32 = s;		m.m33 = c;
}

void rotZ(Mat33& m, double angle) {
	const double s = ::sin(angle), c = ::cos(angle);
	
	m.m11 = c;		m.m12 = -s;		m.m13 = 0.;
	m.m21 = s;		m.m22 = c;		m.m23 = 0.;
	m.m31 = 0.;		m.m32 = 0.;		m.m33 = 1.;
}

double frand(double min = 0., double max = 1.) {
	return min + (max - min)*::rand()/RAND_MAX;
}

GenWorkspaceStateSeq createLine(const GenWorkspaceState& begin, const GenWorkspaceState& end, const int chain = 0, const int segments = 5) {
	GenWorkspaceStateSeq seq;
	
	for (int i = 0; i < segments; ++i) {
		const double s = (double)(i + 1)/segments;

		GenWorkspaceStateI w(chain + 1);
		w.t = begin.t + s*(end.t - begin.t);
		w.pos.c[chain].R = begin.pos.c[chain].R; // TODO interpolate orientation
		w.pos.c[chain].p.v1 = begin.pos.c[chain].p.v1 + s*(end.pos.c[chain].p.v1 - begin.pos.c[chain].p.v1);
		w.pos.c[chain].p.v2 = begin.pos.c[chain].p.v2 + s*(end.pos.c[chain].p.v2 - begin.pos.c[chain].p.v2);
		w.pos.c[chain].p.v3 = begin.pos.c[chain].p.v3 + s*(end.pos.c[chain].p.v3 - begin.pos.c[chain].p.v3);
		seq.push_back(w);
	}
	
	return seq;
}

const double PI = 3.14159265358979323846;
const double INF = 1.7976931348623158e+308;

//------------------------------------------------------------------------------

class TinyIceApp : virtual public Ice::Application {
public:
	virtual int run(int, char*[]) {
		Ice::ObjectPrx base = communicator()->stringToProxy("GolemTiny:default -p 8172");
		
		TinyPrx pTiny = TinyPrx::checkedCast(base);
		if (!pTiny)
			throw ExTiny("Invalid proxy");

		shutdownOnInterrupt();
		
		// create controller of a robotic arm
		//KatanaDesc* pCtrlDesc = new KatanaDescI; // specialised Katana 300/450 description
		ControllerDesc* pCtrlDesc = new ControllerDescI; // generic description
		pCtrlDesc->libraryPathCtrl = "GolemCtrlKatana300Sim"; // specify library path
		pCtrlDesc->configPathCtrl = "GolemCtrlKatana300"; // specify xml config path
		pCtrlDesc->libraryPathPlanner = "GolemPlannerGraphPlanner"; // specify library path
		pCtrlDesc->configPathPlanner = "GolemPlannerKatana300"; // specify xml config path
		printf("Creating the arm...\n");
		ControllerPrx pCtrl = ControllerPrx::checkedCast(pTiny->createActor(ActorDescPtr(pCtrlDesc)));

		// open chain manipulators have only one kinematic chain with index 0
		const int chain = 0;

		// attached a finger to the end-effector (the last joint)
		JointPrx pEffector = pCtrl->getJoints().back();
		// get the end-effector reference pose
		WorkspaceCoord referencePose = pCtrl->getReferencePose();
		// construct a finger from a box and a sphere in local end-effector coordinates (arm at reference configuration stretched along Y-axis)
		const double fingerLength = 0.1;
		const double fingerDiam = 0.005;
		const double fingerTipRadius = 0.015;
		BoxShapeDesc* pFingerRodShapeDesc = new BoxShapeDescI;
		pFingerRodShapeDesc->dimensions.v1 = fingerDiam/2.0;
		pFingerRodShapeDesc->dimensions.v2 = fingerLength/2.0;
		pFingerRodShapeDesc->dimensions.v3 = fingerDiam/2.0;
		pFingerRodShapeDesc->localPose = referencePose.c[chain];
		pFingerRodShapeDesc->localPose.p.v2 += fingerLength/2.0;
		ShapePrx pFingerRodShape = pEffector->createShape(ShapeDescPtr(pFingerRodShapeDesc));
		SphereShapeDesc* pFingerTipShapeDesc = new SphereShapeDescI;
		pFingerTipShapeDesc->radius = fingerTipRadius;
		pFingerTipShapeDesc->localPose = referencePose.c[chain];
		pFingerTipShapeDesc->localPose.p.v2 += fingerLength;
		ShapePrx pFingerTipShape = pEffector->createShape(ShapeDescPtr(pFingerTipShapeDesc));
		// change reference pose, so the end-effector pose will be further referred to the finger tip
		referencePose.c[chain].p.v2 += fingerLength;
		pCtrl->setReferencePose(referencePose);

		// trajectory
		GenConfigspaceStateSeq trajectory;
		// movement begin and end position in join configuration space
		GenConfigspaceState cbegin, cend;
		// initial configuration (it is the current joint configuration)
		GenConfigspaceState initial = pCtrl->recvGenConfigspaceState(INF);

		// setup home end-effector pose (the joint configuration is not known)
		GenWorkspaceStateI home(chain + 1);
		rotX(home.pos.c[chain].R, -0.5*PI); // end-effector pointing downwards
		home.pos.c[chain].p.v1 = 0.0;
		home.pos.c[chain].p.v2 = 0.12;
		home.pos.c[chain].p.v3 = 0.1;
		
		// create ground plane using plane shape
		RigidBodyDesc* pGroundPlaneDesc = new RigidBodyDescI;
		PlaneShapeDesc* pGroundPlaneShapeDesc = new PlaneShapeDescI;
		pGroundPlaneDesc->shapes.push_back(ShapeDescPtr(pGroundPlaneShapeDesc));
		RigidBodyPrx pGroundPlane = RigidBodyPrx::checkedCast(pTiny->createActor(ActorDescPtr(pGroundPlaneDesc)));

		// Object pointer
		RigidBodyPrx pObject;

		// initialise random seed
		::srand((unsigned int)::time(NULL));

		// Experiment main loop
		while (!interrupted()) {
			// setup movement to home position and compute movement end/target in joint configuration space
			cbegin = pCtrl->recvGenConfigspaceState(INF);
			home.t = cbegin.t + 1.0; // movement will last no shorter than 1 sec
			cend = pCtrl->findTarget(cbegin, home);
			// compute trajectory using path planning with collision detection
			trajectory = pCtrl->findGlobalTrajectory(cbegin, cend);
			// move the arm and wait until it stops
			printf("Moving to home position...\n");
			pCtrl->send(trajectory, INF);

			// setup an object (polyflap) as a set of two boxes
			RigidBodyDesc* pObjectDesc = new RigidBodyDescI;
			const double objectWidthY = 0.07;
			const double objectWidthZ = 0.07;
			const double objectLength = 0.07;
			const double objectHeight = 0.07;
			const double objectAngle = 0.5*PI;
			const double objectThickness = 0.0001;
			// Y-up shape
			BoxShapeDesc* pYShapeDesc = new BoxShapeDescI;
			pYShapeDesc->dimensions.v1 = objectWidthY;
			pYShapeDesc->dimensions.v2 = objectLength;
			pYShapeDesc->dimensions.v3 = objectThickness;
			pYShapeDesc->localPose.p.v1 = 0.0;
			pYShapeDesc->localPose.p.v2 = objectLength;
			pYShapeDesc->localPose.p.v3 = objectThickness;
			pObjectDesc->shapes.push_back(ShapeDescPtr(pYShapeDesc));
			// Y-up shape
			BoxShapeDesc* pZShapeDesc = new BoxShapeDescI;
			double objectSin = ::sin(objectAngle), objectCos = ::cos(objectAngle);
			pZShapeDesc->dimensions.v1 = objectWidthZ;
			pZShapeDesc->dimensions.v2 = objectHeight;
			pZShapeDesc->dimensions.v3 = objectThickness;
			pZShapeDesc->localPose.p.v1 = 0.0;
			pZShapeDesc->localPose.p.v2 = objectCos*objectHeight;
			pZShapeDesc->localPose.p.v3 = objectSin*objectHeight + objectThickness;
			rotX(pZShapeDesc->localPose.R, objectAngle);
			pObjectDesc->shapes.push_back(ShapeDescPtr(pZShapeDesc));
			// global pose
			pObjectDesc->globalPose.p.v2 += 0.25;
			rotZ(pObjectDesc->globalPose.R, 2.0*PI*frand());
			// delete previous object, create a new one. Optionally only global pose can be set
			if (pObject)
				pTiny->releaseActor(pObject);
			pObject = RigidBodyPrx::checkedCast(pTiny->createActor(ActorDescPtr(pObjectDesc)));

			// setup end-effector begin pose
			GenWorkspaceStateI begin(chain + 1);
			rotX(begin.pos.c[chain].R, -0.5*PI); // end-effector pointing downwards
			begin.pos.c[chain].p.v1 = frand(0.10, 0.15);
			begin.pos.c[chain].p.v2 = frand(0.22, 0.28);
			begin.pos.c[chain].p.v3 = frand(0.05, 0.15);
			// compute movement end/target in joint configuration space
			cbegin = pCtrl->recvGenConfigspaceState(INF);
			begin.t = cbegin.t + 1.0; // movement will last no shorter than 1 sec
			cend = pCtrl->findTarget(cbegin, begin);
			// compute trajectory using path planning with collision detection
			trajectory = pCtrl->findGlobalTrajectory(cbegin, cend);
			// move the arm and wait until it stops
			printf("Moving to the begin position...\n");
			pCtrl->send(trajectory, INF);

			// setup end-effector random workspace target coordinates
			GenWorkspaceStateI end(chain + 1);
			rotX(end.pos.c[chain].R, -0.5*PI); // end-effector pointing downwards
			end.pos.c[chain].p.v1 = frand(-0.15, -0.10);
			end.pos.c[chain].p.v2 = frand(0.22, 0.28);
			end.pos.c[chain].p.v3 = frand(0.05, 0.15);
			// with random movement duration
			end.t = cbegin.t + frand(2.0, 5.0);
			// turn off collision detection - clear collision group mask
			pCtrl->setCollisionGroup(0x0);
			// compute line-shaped trajectory
			cbegin = pCtrl->recvGenConfigspaceState(INF);
			trajectory = pCtrl->findLocalTrajectory(cbegin, createLine(begin, end, chain));
			// move the arm and wait until it stops
			printf("Moving to the end position...\n");
			pCtrl->send(trajectory, INF);
			
			// reset collision detection - fill collision group mask with 1s to indicate shapes with all possible group masks
			pCtrl->setCollisionGroup(0xFFFFFFFF);
		}

		// compute movement end/target in joint configuration space
		cbegin = pCtrl->recvGenConfigspaceState(INF);
		cend = initial;
		cend.t = cbegin.t + 5.0; // movement will last no shorter than 5 sec
		// compute trajectory using path planning with collision detection
		trajectory = pCtrl->findGlobalTrajectory(cbegin, cend);
		// move the arm and wait until it stops
		printf("Moving back to the initial configuration...\n");
		pCtrl->send(trajectory, INF);

		if (pObject)
			pTiny->releaseActor(pObject);
		pTiny->releaseActor(pCtrl);
		pTiny->releaseActor(pGroundPlane);
		
		printf("Good bye!\n");

		return 0;
	}
};

//------------------------------------------------------------------------------

int main(int argc, char* argv[]) {
	return TinyIceApp().main(argc, argv);
}