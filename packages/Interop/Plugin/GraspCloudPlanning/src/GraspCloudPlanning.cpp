/** @file GraspCloudPlanning.cpp
*
* @author	Marek Kopicki
*
*/

#include <Golem/Interop/GraspCloudPlanning/GraspCloudPlanning.h>
#include "GolemInteropPCLDefs.h"
#include <exception>
#include <memory>
#include <thread>
#include <chrono>

using namespace golem;
using namespace golem::interop;

//------------------------------------------------------------------------------

std::shared_ptr<Interface> pInterface;

GOLEM_INTEROP_LIBRARY_DECLDIR void* InterfaceLoader(const char* param) {
	if (!pInterface)
		pInterface.reset(new GraspCloudPlanning(std::string(param)));
	return pInterface.get();
}

//------------------------------------------------------------------------------

golem::interop::GraspCloudPlanning::GraspCloudPlanning(const std::string& param) : MasterConfig(param) {}

//------------------------------------------------------------------------------

void golem::interop::GraspCloudPlanning::run(const Map& interfaces) {
	///////////////////////////////////////////////////////////////////////////
	//
	// Initialisation
	//
	///////////////////////////////////////////////////////////////////////////

	SensorCloud* pSensorCloud = getInterface<SensorCloud>(interfaces);
	Controller* pController = getInterface<Controller>(interfaces);
	Planner* pPlanner = getInterface<Planner>(interfaces);
	GraspCloud* pGraspCloud = getInterface<GraspCloud>(interfaces);
	Application* pApplication = getInterface<Application>(interfaces, true);

	///////////////////////////////////////////////////////////////////////////
	//
	// Test
	//
	///////////////////////////////////////////////////////////////////////////

	// Load clouds from files
	Point3DCloud::Seq testClouds(2);
	loadPCLCloud<pcl::PointXYZRGBNormal>("GolemInteropPluginGraspCloudPlanning_Test_1_1.pcd", testClouds[0]);
	printf("TEST: points=%zu\n", testClouds[0].size());
	loadPCLCloud<pcl::PointXYZRGBNormal>("GolemInteropPluginGraspCloudPlanning_Test_1_2.pcd", testClouds[1]);
	printf("TEST: points=%zu\n", testClouds[1].size());

	// Find possible grasp trajectories
	Trajectory::Seq testTrajectories;
	pGraspCloud->findTrajectories(testClouds, testTrajectories);
	for (size_t i = 0; i < testTrajectories.size(); ++i) {
		printf("TEST: trajectory=%zu#, type=%s, waypoints=%zu, error=(%f, %f)\n", i + 1, testTrajectories[i].type.c_str(), testTrajectories[i].trajectory.size(), testTrajectories[i].error[0].lin, testTrajectories[i].error[0].ang);
	}

	// Select first trajectory
	const Trajectory& trajectory = testTrajectories[0];

	// Obtain current state
	Config state;
	pController->lookupState(pController->time(), state);
	// Find trajectory to the beginning of grasp trajectory
	Config::Seq ctrajectory;
	pPlanner->findTrajectory(state.cpos, trajectory.trajectory.front().cpos, ctrajectory);

	printf("TEST: grasping!\n");
	// Go to to the beginning of grasp trajectory, wait to finish
	pController->sendCommand(ctrajectory.data(), ctrajectory.size());
	pController->waitForTrajectoryEnd();
	// Perform grasp trajectory, wait to finish
	pController->sendCommand(trajectory.trajectory.data(), trajectory.trajectory.size());
	pController->waitForTrajectoryEnd();

	// Run interactive menu
	if (pApplication) {
		printf("Ready!\n");
		while (pApplication->dispatch());
	}
}

//------------------------------------------------------------------------------
