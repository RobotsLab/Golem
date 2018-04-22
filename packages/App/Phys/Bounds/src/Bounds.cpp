/** @file Bounds.cpp
 * 
 * Program demonstrates how to add/remove/modify shape of joints.
 * Demo requires PhysX simulator to cook triangle mesh.
 *
 * Program can be run in two modes:
 * - the first uses real robot
 * - the second runs robot simulator
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

//#include <Golem/Tools/Debug.h>
#include <Golem/App/Application.h>
#include <Golem/App/UIPlanner.h>
#include <Golem/Phys/PhysUniverse.h>
#include <Golem/App/Common/Tools.h>
#include <Golem/App/Common/Creator.h>
#include <Golem/App/Data.h>

using namespace golem;

//------------------------------------------------------------------------------

void setupObjects(Scene* scene) {
	// Creator
	Creator creator(*scene);
	Actor::Desc::Ptr pActorDesc;

	// Create ground plane.
	pActorDesc = creator.createGroundPlaneDesc();
	scene->createObject(*pActorDesc);

	// "Left post"
	pActorDesc = creator.createBoxDesc(Real(0.05), Real(0.3), Real(0.25));
	pActorDesc->pose.p.set(Real(0.25), Real(-0.05), Real(0.25));
	scene->createObject(*pActorDesc);

	//// "Right post"
	//pActorDesc = creator.createBoxDesc(Real(0.07), Real(0.1), Real(0.25));
	//pActorDesc->pose.p.set(Real(-0.15), Real(0.05), Real(0.25));
	//scene->createObject(*pActorDesc);
	//
	//// "Crossbar"
	//pActorDesc = creator.createBoxDesc(Real(0.3), Real(0.05), Real(0.025));
	//pActorDesc->pose.p.set(Real(0.0), Real(0.15), Real(0.525));
	//scene->createObject(*pActorDesc);
}

//------------------------------------------------------------------------------

template <typename _PVEC, typename _MAT> void trn(_PVEC begin, _PVEC end, const _MAT &pose) {
	for (_PVEC v = begin; v != end; ++v) pose.multiply(*v, *v);
}

// create mace
void createMace(std::vector<Bounds::Desc::Ptr> &boundsDescSeq, Mat34 &referencePose, const Mat34 &pose) {
	// mace characteristic dimensions
	const Real length = Real(0.15);
	const Real begin = Real(0.01);
	const Real end = Real(0.025);
	const Real diam = Real(0.02);
	const Real height = Real(0.05);
	const Real pos = length - end;

	// reference pose at the end (on Y-axis)
	referencePose.setId();
	referencePose.p.v2 += length;
	referencePose.multiply(pose, referencePose);
	
	boundsDescSeq.clear();

	// objects data
	BoundingConvexMesh::Desc hilt;
	hilt.vertices.resize(8);
	hilt.bCook = true;
	BoundingConvexMesh::Desc spike;
	spike.vertices.resize(5);
	spike.bCook = true;

	// create hilt (along Y-axis)
	hilt.vertices[0].set(-begin, Real(0.0), -begin);
	hilt.vertices[1].set(+begin, Real(0.0), -begin);
	hilt.vertices[2].set(-begin, Real(0.0), +begin);
	hilt.vertices[3].set(+begin, Real(0.0), +begin);
	hilt.vertices[4].set(-end, length, -end);
	hilt.vertices[5].set(+end, length, -end);
	hilt.vertices[6].set(-end, length, +end);
	hilt.vertices[7].set(+end, length, +end);
	trn(hilt.vertices.begin(), hilt.vertices.end(), pose);
	boundsDescSeq.push_back(hilt.clone());

	// create spike #1
	spike.vertices[0].set(-diam, pos - diam, end);
	spike.vertices[1].set(+diam, pos - diam, end);
	spike.vertices[2].set(-diam, pos + diam, end);
	spike.vertices[3].set(+diam, pos + diam, end);
	spike.vertices[4].set(Real(0.0), pos, end + height);
	trn(spike.vertices.begin(), spike.vertices.end(), pose);
	boundsDescSeq.push_back(spike.clone());
	
	// create spike #2
	spike.vertices[0].set(-diam, pos - diam, -end);
	spike.vertices[1].set(+diam, pos - diam, -end);
	spike.vertices[2].set(-diam, pos + diam, -end);
	spike.vertices[3].set(+diam, pos + diam, -end);
	spike.vertices[4].set(Real(0.0), pos, - end - height);
	trn(spike.vertices.begin(), spike.vertices.end(), pose);
	boundsDescSeq.push_back(spike.clone());
	
	// create spike #3
	spike.vertices[0].set(end, pos - diam, -diam);
	spike.vertices[1].set(end, pos - diam, +diam);
	spike.vertices[2].set(end, pos + diam, -diam);
	spike.vertices[3].set(end, pos + diam, +diam);
	spike.vertices[4].set(end + height, pos, Real(0.0));
	trn(spike.vertices.begin(), spike.vertices.end(), pose);
	boundsDescSeq.push_back(spike.clone());
	
	// create spike #4
	spike.vertices[0].set(-end, pos - diam, -diam);
	spike.vertices[1].set(-end, pos - diam, +diam);
	spike.vertices[2].set(-end, pos + diam, -diam);
	spike.vertices[3].set(-end, pos + diam, +diam);
	spike.vertices[4].set(-end - height, pos, Real(0.0));
	trn(spike.vertices.begin(), spike.vertices.end(), pose);
	boundsDescSeq.push_back(spike.clone());
}

// create finger
void createFinger(std::vector<Bounds::Desc::Ptr> &boundsDescSeq, Mat34 &referencePose, const Mat34 &pose) {
	// finger characteristic dimensions
	const Real length = Real(0.1);
	const Real begin = Real(0.015);
	const Real end = Real(0.01);

	// reference pose at the end (on Y-axis)
	referencePose.setId();
	referencePose.p.v2 += length;
	referencePose.multiply(pose, referencePose);
	
	boundsDescSeq.clear();
	
	// objects data
	BoundingConvexMesh::Desc finger;
	finger.vertices.resize(8);
	finger.bCook = true;

	// create finger (along Y-axis)
	finger.vertices[0].set(-begin, Real(0.0), -begin);
	finger.vertices[1].set(+begin, Real(0.0), -begin);
	finger.vertices[2].set(-begin, Real(0.0), +begin);
	finger.vertices[3].set(+begin, Real(0.0), +begin);
	finger.vertices[4].set(-end, length, -end);
	finger.vertices[5].set(+end, length, -end);
	finger.vertices[6].set(-end, length, +end);
	finger.vertices[7].set(+end, length, +end);
	trn(finger.vertices.begin(), finger.vertices.end(), pose);
	boundsDescSeq.push_back(finger.clone());
}

//------------------------------------------------------------------------------

// Add new bounds to the Joint Actor.
void addBounds(Actor* actor, Bounds::ConstSeq &boundsSeq, const Bounds::Desc::Seq &boundsDescSeq) {
	ASSERT(actor)
	boundsSeq.clear();
	for (Bounds::Desc::Seq::const_iterator i = boundsDescSeq.begin(); i != boundsDescSeq.end(); ++i)
		boundsSeq.push_back(actor->createBounds(*i));
}

// Remove bounds of the Joint Actor.
void removeBounds(Actor* actor, Bounds::ConstSeq &boundsSeq) {
	ASSERT(actor)
	for (Bounds::ConstSeq::const_iterator i = boundsSeq.begin(); i != boundsSeq.end(); ++i)
		actor->releaseBounds(**i);
}

//------------------------------------------------------------------------------

Real getLinearDist(const Vec3& v0, const Vec3& v1) {
	return v0.distance(v1);
}

Real getAngularDist(const Quat& q0, const Quat& q1) {
	const Real d = q0.dot(q1);
	return REAL_ONE - Math::abs(d);
}

//------------------------------------------------------------------------------

/** MyApplication */
class MyApplication : public Application {
protected:
	/** Runs MyApplication */
	virtual void run(int argc, char *argv[]) {
		// help message
		scene()->printHelp();
		
		// Random number generator seed
		context()->info("Random number generator seed %d\n", context()->getRandSeed()._U32[0]);

		// Setup planner
		UIPlanner::Desc uiPlannerDesc;
		XMLData(uiPlannerDesc, context(), xmlcontext());

		// Create UIPlanner
		context()->info("Initialising planner...\n");
		UIPlanner *pUIPlanner = dynamic_cast<UIPlanner*>(scene()->createObject(uiPlannerDesc));
		if (pUIPlanner == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create Planner");
		
		// some useful pointers
		Controller &controller = pUIPlanner->getController();
		Planner &planner = *pUIPlanner->getPlannerSeq()[0];
		
		// Display information
		controllerInfo(controller);

		// Scene objects setup
		setupObjects(scene());

		// use the first kinematic chain
		const Chainspace::Index chain = controller.getStateInfo().getChains().begin();

		// Create bounds to be attached to the end-effector (the last joint) 
		Bounds::Desc::Seq boundsDescSeq;
		const Mat34 initReferencePose = controller.getChains()[chain]->getReferencePose();
		Mat34 referencePose;
		createMace(boundsDescSeq, referencePose, initReferencePose);
		//createFinger(boundsDescSeq, referencePose, initReferencePose);
		// set new reference pose
		controller.getChains()[chain]->setReferencePose(referencePose);
		// Modify shape of the joint by adding new bounds to the Actor representing the end-effector.
		Bounds::ConstSeq boundsSeq;
		addBounds(pUIPlanner->getUIController().getJointActors()[controller.getStateInfo().getJoints().end() - 1], boundsSeq, boundsDescSeq);

		// Move the arm to random destination poses
		Rand rand(context()->getRandSeed());
		while (!universe()->interrupted()) {
			// generate random end pose
			GenWorkspaceChainState end;
			// random position
			Vec3 position(
				rand.nextUniform(Real(-0.4), Real(+0.4)),
				rand.nextUniform(Real(-0.4), Real(+0.4)),
				rand.nextUniform(Real(+0.05), Real(+0.5))
			);
			// random orientation
			Vec3 orientation(
				rand.nextUniform(Real(-0.25)*REAL_PI, Real(+0.25)*REAL_PI),
				rand.nextUniform(Real(-0.25)*REAL_PI, Real(+0.25)*REAL_PI),
				-Math::atan2(position.v1, position.v2)// + rand.nextUniform(Real(-0.2)*REAL_PI, Real(+0.2)*REAL_PI)
			);
			fromCartesianPose(end.wpos[chain], position, orientation);

			// controller state at a time t and finishes at some time later
			Controller::State begin = controller.createState();
			controller.lookupCommand(SEC_TM_REAL_INF, begin); // last sent trajectory waypoint
			begin.cvel.fill(controller.getStateInfo().getJoints(), REAL_ZERO);
			begin.cacc.fill(controller.getStateInfo().getJoints(), REAL_ZERO);
			begin.t += controller.getCycleDuration(); // concatenate trajectories
			end.t = begin.t + SecTmReal(2.0); // minimum trajectory duration
			
			// Velocity and acceleration limits
			//planner.getProfile()->setVelocity(Real(1.0));
			//planner.getProfile()->setAcceleration(Real(1.0));
			
			// Main ON/OFF collision detection switch
			//planner.getHeuristic().setCollisionDetection(false);
			
			// All bounds are treated as obstacles
			pUIPlanner->setCollisionBoundsGroup(Bounds::GROUP_ALL);

			// find target
			Controller::State cend = begin;
			if (!planner.findTarget(begin, end, cend))
				continue;

			Controller::Trajectory trajectory;
			// find trajectory and wait until the device is ready for new commands
			if (!planner.findGlobalTrajectory(begin, cend, trajectory, trajectory.begin(), &end) || !controller.waitForEnd())
				continue;

			// print pose error, note that the target is always computed with respect to the reference pose in the tool frame (the last joint)
			WorkspaceChainCoord actual;
			planner.getController().chainForwardTransform(cend.cpos, actual);
			actual[chain].multiply(actual[chain], controller.getChains()[chain]->getReferencePose());
			context()->info("Pose error = (%f, %f)\n", getLinearDist(end.wpos[chain].p, actual[chain].p), getAngularDist(Quat(actual[chain].R), Quat(end.wpos[chain].R)));

			// send to the controller
			if (controller.send(&*trajectory.begin(), &*trajectory.end()) != &*trajectory.end())
				continue;
		}

		context()->info("Good bye!\n");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv, PhysUniverse::Desc());
}
