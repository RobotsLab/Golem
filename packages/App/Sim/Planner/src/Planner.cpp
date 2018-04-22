/** @file Planner.cpp
 * 
 * Demonstration program which moves the arm to random poses with
 * trajectory planner and collision detection.
 * 
 * Program can be run in two modes:
 * - the first uses real robotic arm
 * - the second runs the robotic arm simulator
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/App/Application.h>
#include <Golem/App/UIPlanner.h>
#include <Golem/App/Data.h>
#include <Golem/App/Common/Tools.h>
#include <Golem/App/Common/Creator.h>

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
	pActorDesc = creator.createBoxDesc(Real(0.1), Real(0.6), Real(0.4));
	pActorDesc->pose.p.set(Real(0.5), Real(-0.1), Real(0.4));
	scene->createObject(*pActorDesc);

	// "Right post"
	pActorDesc = creator.createBoxDesc(Real(0.14), Real(0.2), Real(0.4));
	pActorDesc->pose.p.set(Real(-0.5), Real(0.1), Real(0.4));
	scene->createObject(*pActorDesc);
	
	// "Crossbar"
	pActorDesc = creator.createBoxDesc(Real(0.6), Real(0.1), Real(0.05));
	pActorDesc->pose.p.set(Real(0.0), Real(0.3), Real(0.85));
	scene->createObject(*pActorDesc);
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
		
		// Scene objects setup
		setupObjects(scene());

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

		// use the first kinematic chain
		const Chainspace::Index chain = controller.getStateInfo().getChains().begin();
		
		// Move the arm to random destination poses
		Rand rand(context()->getRandSeed());
		for (U32 poseId = 0; !universe()->interrupted();) {
			// generate random end pose
			GenWorkspaceChainState end;
			// random position
			Vec3 position(
				rand.nextUniform(Real(-0.7), Real(+0.7)),
				rand.nextUniform(Real(-0.7), Real(+0.7)),
				rand.nextUniform(Real(+0.05), Real(+0.7))
			);
			// random orientation
			Vec3 orientation(
				rand.nextUniform(Real(-0.5)*REAL_PI, Real(+0.5)*REAL_PI),
				rand.nextUniform(Real(-0.5)*REAL_PI, Real(+0.5)*REAL_PI),
				-Math::atan2(position.v1, position.v2) + rand.nextUniform(Real(-0.25)*REAL_PI, Real(+0.25)*REAL_PI)
			);
			fromCartesianPose(end.wpos[chain], position, orientation);
			// increment pose id
			++poseId;

			// controller state at a time t and finishes at some time later
			Controller::State begin = controller.createState();
			controller.lookupCommand(SEC_TM_REAL_INF, begin); // last sent trajectory waypoint
			begin.cvel.fill(controller.getStateInfo().getJoints(), REAL_ZERO);
			begin.cacc.fill(controller.getStateInfo().getJoints(), REAL_ZERO);
			begin.t += controller.getCycleDuration(); // concatenate trajectories
			end.t = begin.t + SecTmReal(2.0); // minimum trajectory duration
			
			// Velocity and acceleration limits
			planner.getProfile()->getVelocity().fill(Real(0.5));
			planner.getProfile()->getAcceleration().fill(Real(0.5));
			
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
			context()->info("Pose #%u: error_{lin, ang} = {%f, %f}\n", poseId, getLinearDist(end.wpos[chain].p, actual[chain].p), getAngularDist(Quat(actual[chain].R), Quat(end.wpos[chain].R)));

			// pause?
			if (universe()->waitKey(0) == 'p') {
				(void)controller.send(&*trajectory.begin(), &*trajectory.begin());
				while (universe()->waitKey() != 'p');
			}

			// send to the controller
			if (controller.send(trajectory.data(), trajectory.data() + trajectory.size()) != trajectory.data() + trajectory.size())
				continue;
		}

		context()->info("Good bye!\n");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}
