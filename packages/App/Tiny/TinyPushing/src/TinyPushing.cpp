/** @file TinyPushing.cpp
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
#include <Golem/Math/Rand.h>
#include <Golem/Math/Quat.h>
#include <iostream>

using namespace golem;
using namespace golem::tiny;

//------------------------------------------------------------------------------

double getLinearDist(const Vec3& v0, const Vec3& v1) {
	return v0.distance(v1);
}

double getAngularDist(const Quat& q0, const Quat& q1) {
	const double d = q0.dot(q1);
	return REAL_ONE - ::fabs(d);
}

GenWorkspaceChainStateSeq createLine(const GenWorkspaceChainState& begin, const GenWorkspaceChainState& end, const int chain = 0, const int segments = 5) {
	const Quat qbegin(begin.pos.c[chain].R);
	const Quat qend(end.pos.c[chain].R);
	GenWorkspaceChainStateSeq seq;
	
	for (int i = 0; i < segments; ++i) {
		const double s = (double)(i + 1)/segments;
		Quat q;
		q.slerp(qbegin, qend, s);
		
		GenWorkspaceChainState w;
		w.t = begin.t + s*(end.t - begin.t);
		w.pos.c[chain].R.fromQuat(q);
		w.pos.c[chain].p.interpolate(begin.pos.c[chain].p, end.pos.c[chain].p, s);
		seq.push_back(w);
	}
	
	return seq;
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	try {
		Tiny tiny(argc, argv);

		// create controller of a robotic arm
		//KatanaDesc* pCtrlDesc = new KatanaDesc; // specialised Katana 300/450 description
		ControllerDesc* pCtrlDesc = new ControllerDesc; // generic description
		pCtrlDesc->libraryPathCtrl = "GolemCtrlKatana300Sim"; // specify library path
		pCtrlDesc->configPathCtrl = "GolemCtrlKatana300"; // specify xml config path
		pCtrlDesc->libraryPathPlanner = "GolemPlannerGraphPlanner"; // specify library path
		pCtrlDesc->configPathPlanner = "GolemPlannerKatana300"; // specify xml config path
		tiny.print("Creating the arm...\n");
		Controller* pCtrl = dynamic_cast<Controller*>(tiny.createActor(ActorDescPtr(pCtrlDesc)));

		// open chain manipulators have only one kinematic chain with index 0
		const int chain = 0;

		// attached a finger to the end-effector (the last joint)
		Joint* pEffector = pCtrl->getJoints().back();
		// get the end-effector reference pose
		WorkspaceChainCoord referencePose = pCtrl->getReferencePose();
		// construct a finger from a box and a sphere in local end-effector coordinates (arm at reference configuration stretched along Y-axis)
		const double fingerLength = 0.1;
		const double fingerDiam = 0.005;
		const double fingerTipRadius = 0.015;
		BoxShapeDesc* pFingerRodShapeDesc = new BoxShapeDesc;
		pFingerRodShapeDesc->dimensions.set(Real(fingerDiam/2.0), Real(fingerLength/2.0), Real(fingerDiam/2.0));
		pFingerRodShapeDesc->localPose = referencePose.c[chain];
		pFingerRodShapeDesc->localPose.p.v2 += Real(fingerLength/2.0);
		Shape* pFingerRodShape = pEffector->createShape(ShapeDescPtr(pFingerRodShapeDesc));
		SphereShapeDesc* pFingerTipShapeDesc = new SphereShapeDesc;
		pFingerTipShapeDesc->radius = fingerTipRadius;
		pFingerTipShapeDesc->localPose = referencePose.c[chain];
		pFingerTipShapeDesc->localPose.p.v2 += Real(fingerLength);
		Shape* pFingerTipShape = pEffector->createShape(ShapeDescPtr(pFingerTipShapeDesc));
		// change reference pose, so the end-effector pose will be further referred to the finger tip
		referencePose.c[chain].p.v2 += Real(fingerLength);
		pCtrl->setReferencePose(referencePose);

		// trajectory
		GenConfigspaceStateSeq trajectory;
		// movement begin and end position in join configuration space
		GenConfigspaceState cbegin, cend;
		// initial configuration (it is the current joint configuration)
		GenConfigspaceState initial = pCtrl->recvGenConfigspaceState(numeric_const<double>::INF);

		// setup home end-effector pose (the joint configuration is not known)
		GenWorkspaceChainState home;
		home.pos.c[chain].R.rotX(Real(-0.5)*REAL_PI); // end-effector pointing downwards
		home.pos.c[chain].p.set(Real(0.0), Real(0.12), Real(0.1));
		
		// create ground plane using plane shape
		RigidBodyDesc* pGroundPlaneDesc = new RigidBodyDesc;
		PlaneShapeDesc* pGroundPlaneShapeDesc = new PlaneShapeDesc;
		pGroundPlaneDesc->shapes.push_back(ShapeDescPtr(pGroundPlaneShapeDesc));
		RigidBody* pGroundPlane = dynamic_cast<RigidBody*>(tiny.createActor(ActorDescPtr(pGroundPlaneDesc)));

		// Object pointer
		RigidBody* pObject = NULL;

		// Random number generator
		Rand frand;
		frand.setRandSeed(RandSeed());
		
		// Experiment main loop
		while (!tiny.interrupted()) {
			// setup movement to home position and compute movement end/target in joint configuration space
			cbegin = pCtrl->recvGenConfigspaceState(numeric_const<double>::INF);
			home.t = cbegin.t + 1.0; // movement will last no shorter than 1 sec
			cend = pCtrl->findTarget(cbegin, home);
			// compute trajectory using path planning with collision detection
			trajectory = pCtrl->findGlobalTrajectory(cbegin, cend);
			// move the arm and wait until it stops
			tiny.print("Moving to home position...\n");
			pCtrl->send(trajectory, numeric_const<double>::INF);

			// setup an object (polyflap) as a set of two boxes
			RigidBodyDesc* pObjectDesc = new RigidBodyDesc;
			const double objectWidthY = 0.07;
			const double objectWidthZ = 0.07;
			const double objectLength = 0.07;
			const double objectHeight = 0.07;
			const double objectAngle = numeric_const<double>::PI_2;
			const double objectThickness = 0.0001;
			// Y-up shape
			BoxShapeDesc* pYShapeDesc = new BoxShapeDesc;
			pYShapeDesc->dimensions.set(Real(objectWidthY), Real(objectLength), Real(objectThickness));
			pYShapeDesc->localPose.p.set(Real(0.0), Real(objectLength), Real(objectThickness));
			pObjectDesc->shapes.push_back(ShapeDescPtr(pYShapeDesc));
			// Z-up shape
			BoxShapeDesc* pZShapeDesc = new BoxShapeDesc;
			double objectSin, objectCos;
			Math::sinCos(objectAngle, objectSin, objectCos);
			pZShapeDesc->dimensions.set(Real(objectWidthZ), Real(objectHeight), Real(objectThickness));
			pZShapeDesc->localPose.p.set(Real(0.0), Real(objectCos*objectHeight), Real(objectSin*objectHeight + objectThickness));
			pZShapeDesc->localPose.R.rotX(objectAngle);
			pObjectDesc->shapes.push_back(ShapeDescPtr(pZShapeDesc));
			// global pose
			pObjectDesc->globalPose.p.v2 += Real(0.25);
			pObjectDesc->globalPose.R.rotZ(REAL_2_PI*frand.nextUniform<Real>());
			// delete previous object, create a new one. Optionally only global pose can be set
			if (pObject != NULL)
				tiny.releaseActor(pObject);
			pObject = dynamic_cast<RigidBody*>(tiny.createActor(ActorDescPtr(pObjectDesc)));

			// setup end-effector begin pose
			GenWorkspaceChainState begin;
			begin.pos.c[chain].R.rotX(Real(-0.5)*REAL_PI); // end-effector pointing downwards
			begin.pos.c[chain].p.set(
				frand.nextUniform(+Real(0.10), +Real(0.15)),
				frand.nextUniform(Real(0.22), Real(0.28)),
				frand.nextUniform(Real(0.05), Real(0.15))
			);
			// compute movement end/target in joint configuration space
			cbegin = pCtrl->recvGenConfigspaceState(numeric_const<double>::INF);
			begin.t = cbegin.t + 1.0; // movement will last no shorter than 1 sec
			cend = pCtrl->findTarget(cbegin, begin);
			// print pose error, note that the target is always computed with respect to the reference pose in the tool frame (the last joint)
			Mat34 actual;
			actual.multiply(pCtrl->getChainForwardTransform(cend.pos).c[chain], pCtrl->getReferencePose().c[chain]);
			tiny.print("Pose error = (%f, %f)\n", getLinearDist(begin.pos.c[chain].p, actual.p), getAngularDist(Quat(begin.pos.c[chain].R), Quat(actual.R)));
			// compute trajectory using path planning with collision detection
			trajectory = pCtrl->findGlobalTrajectory(cbegin, cend);
			// move the arm and wait until it stops
			tiny.print("Moving to the begin position...\n");
			pCtrl->send(trajectory, numeric_const<double>::INF);

			// setup end-effector random workspace target coordinates
			GenWorkspaceChainState end;
			end.pos.c[chain].R.rotX(Real(-0.5)*REAL_PI); // end-effector pointing downwards
			end.pos.c[chain].p.set(
				frand.nextUniform(-Real(0.15), -Real(0.10)),
				frand.nextUniform(Real(0.22), Real(0.28)),
				frand.nextUniform(Real(0.05), Real(0.15))
			);
			// with random movement duration
			end.t = cbegin.t + frand.nextUniform(Real(2.0), Real(5.0));
			// turn off collision detection - clear collision group mask
			pCtrl->setCollisionGroup(0x0);
			// compute line-shaped trajectory
			cbegin = pCtrl->recvGenConfigspaceState(numeric_const<double>::INF);
			trajectory = pCtrl->findLocalTrajectory(cbegin, createLine(begin, end, chain));
			// move the arm and wait until it stops
			tiny.print("Moving to the end position...\n");
			pCtrl->send(trajectory, numeric_const<double>::INF);

			// reset collision detection - fill collision group mask with 1s to indicate shapes with all possible group masks
			pCtrl->setCollisionGroup(0xFFFFFFFF);
		}

		// compute movement end/target in joint configuration space
		cbegin = pCtrl->recvGenConfigspaceState(numeric_const<double>::INF);
		cend = initial;
		cend.t = cbegin.t + 5.0; // movement will last no shorter than 5 sec
		// compute trajectory using path planning with collision detection
		trajectory = pCtrl->findGlobalTrajectory(cbegin, cend);
		// move the arm and wait until it stops
		tiny.print("Moving back to the initial configuration...\n");
		pCtrl->send(trajectory, numeric_const<double>::INF);
		
		tiny.print("Good bye!\n");
	}
	catch (const ExTiny& ex) {
		std::cerr << ex.what() << std::endl;
	}
	catch (const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
	}

	return 0;
}
