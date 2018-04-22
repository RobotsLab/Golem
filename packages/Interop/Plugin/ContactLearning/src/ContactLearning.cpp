/** @file ContactLearning.cpp
*
* @author	Marek Kopicki
*
*/

#include <Golem/Interop/ContactLearning/ContactLearning.h>
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
		pInterface.reset(new ContactLearning(std::string(param)));
	return pInterface.get();
}

//------------------------------------------------------------------------------

ContactLearning::ContactLearning(const std::string& param) : MasterConfig(param) {}

//------------------------------------------------------------------------------

void ContactLearning::run(const Map& interfaces) {
	///////////////////////////////////////////////////////////////////////////
	//
	// Initialisation
	//
	///////////////////////////////////////////////////////////////////////////

	SensorCloud* pSensorCloud = getInterface<SensorCloud>(interfaces);
	Controller* pController = getInterface<Controller>(interfaces);
	Planner* pPlanner = getInterface<Planner>(interfaces);
	Contact* pContact = getInterface<Contact>(interfaces);
	Application* pApplication = getInterface<Application>(interfaces, true);

	///////////////////////////////////////////////////////////////////////////
	//
	// Training
	//
	///////////////////////////////////////////////////////////////////////////

	// load at least one model point cloud
	Point3DCloud::Seq trainingClouds(2);
	// preferred: load/store directly Point3DCloud::Seq, NOT '*.pcd' files
	loadPCLCloud<pcl::PointXYZRGBNormal>("GolemInteropPluginContactLearning_Training_1.pcd", trainingClouds[0]);
	printf("TRAINING: points[1]=%zu\n", trainingClouds[0].size());
	// preferred: load/store directly Point3DCloud::Seq, NOT '*.pcd' files
	loadPCLCloud<pcl::PointXYZRGBNormal>("GolemInteropPluginContactLearning_Training_2.pcd", trainingClouds[1]);
	printf("TRAINING: points[2]=%zu\n", trainingClouds[1].size());

	// compute features
	Feature3D::Seq trainingFeatures;
	pContact->findFeatures(trainingClouds, trainingFeatures);
	printf("TRAINING: features=%zu\n", trainingFeatures.size());

	// load model trajectory
	Config::Seq trainingTrajectory;
	const float_t positions[][27] = {
		{ 2.180801, -1.592194, -1.521202, 1.712632, -0.050701, -0.562921, 1.380559, -0.011319, 0.088750, 0.000000, 0.000000, 0.000000, 0.051394, 0.000000, 0.000000, 0.000000, 0.926323, 1.057778, 1.057778, 0.000000, 1.483530, 1.134464, 1.134464, 0.000000, 1.483530, 1.134464, 1.134464, },
		{ 1.991930, -1.639876, -1.497247, 1.625777, -0.039985, -0.470176, 1.401919, -0.011319, 0.252084, 0.005093, 0.005093, 0.000000, 0.264099, 0.065219, 0.065219, 0.000000, 0.926323, 1.057778, 1.057778, 0.000000, 1.483530, 1.134464, 1.134464, 0.000000, 1.483530, 1.134464, 1.134464, },
		{ 1.970219, -1.640791, -1.498651, 1.615291, -0.040220, -0.464042, 1.401852, -0.011319, 0.476014, 0.008186, 0.008186, 0.000000, 0.498640, 0.058318, 0.058318, 0.000000, 0.926323, 1.057778, 1.057778, 0.000000, 1.483530, 1.134464, 1.134464, 0.000000, 1.483530, 1.134464, 1.134464, },
	};
	for (size_t i = 0; i < sizeof(positions) / sizeof(positions[0]); ++i) {
		Config config;
		std::copy(positions[i], positions[i] + sizeof(positions[0]) / sizeof(float_t), config.cpos.v);
		// velocities can be ignored because only path is used to create configuration model
		trainingTrajectory.push_back(config);
	}
	printf("TRAINING: waypoints=%zu\n", trainingTrajectory.size());

	// create training data bundle
	Training3D contactTraining3D;
	contactTraining3D.features = trainingFeatures;
	contactTraining3D.trajectory = trainingTrajectory;
	// contact type
	const std::string contactName = "pinchsupp";
	Training3D::Map contactTraining3DMap;
	contactTraining3DMap.insert(std::make_pair(contactName, contactTraining3D));
	// contact models
	Model3D::Map contactModelsMap;
	// optional: load/store directly Model3D::Map
	// compute contact models from training data
	pContact->findModel(contactTraining3DMap, contactModelsMap);
	for (auto &i : contactModelsMap) {
		printf("TRAINING: type=%s, contact_models=%zu, contact_views=%zu, config_spaces=%zu, reserved_size=%zu\n",
			i.first.c_str(), i.second.contacts.size(), i.second.views.size(), i.second.spaces.size(), i.second.reserved.size());
		for (auto &j : i.second.contacts)
			printf("\tcontact_model=#%u, kernels=%zu\n", j.first, j.second.model.size());
	}

	///////////////////////////////////////////////////////////////////////////
	//
	// Test
	//
	///////////////////////////////////////////////////////////////////////////

	// optional - move robot: obtain current state
	Config state;
	//		pController->lookupState(pController->time(), state);

	// optional - move robot: use pre-defined target in configuration space
	//		ConfigspaceCoord ctarget;
	//ctarget[0] = float_t(2.05575);
	//ctarget[1] = float_t(-0.926678);
	//ctarget[2] = float_t(0.688772);
	//ctarget[3] = float_t(-1.6953);
	//ctarget[4] = float_t(0.721298);
	//ctarget[5] = float_t(0.410064);
	//ctarget[6] = float_t(0.531536);

	// optional - move robot: alternative: find target from workspace coordinates
	//		Mat34 frame;
	//frame.p[0] = float_t(0.188549);
	//frame.p[1] = float_t(-0.938590);
	//frame.p[2] = float_t(0.170118);
	//frame.R.m11 = float_t(0.520564); frame.R.m12 = float_t(0.828488); frame.R.m13 = float_t(-0.206449);
	//frame.R.m21 = float_t(0.726453); frame.R.m22 = float_t(-0.556817); frame.R.m23 = float_t(-0.402767);
	//frame.R.m31 = float_t(-0.448641); frame.R.m32 = float_t(0.059690); frame.R.m33 = float_t(-0.891716);
	//		WorkspaceCoord wtarget;
	//		wtarget[0] = frame; // by default only the first kinematic chain is used in path planning (configurable in xmls)
	//		WorkspaceDist werr;
	//		pController->findTarget(state.cpos, wtarget, ctarget, werr);
	//		printf("Planner::findTarget(): error=(%f, %f)\n", werr[0].lin, werr[0].ang);

	// optional - move robot: find trajectory to the target pose
	Config::Seq ctrajectory;
	//		pController->findTrajectory(state.cpos, ctarget, ctrajectory);
	//		printf("Planner::findTrajectory(): length=%zu\n", ctrajectory.size());

	// optional - move robot: send trajectory
	//		pController->sendCommand(ctrajectory.data(), ctrajectory.size());

	// optional - move robot: wait until the last waypoint has been sent to the controller
	//  + 2 x control cycle duration (max command latency) to make sure the robot is not moving
	//  + extra 1 second to stabilise the robot
	//		pController->waitForTrajectoryEnd();
	//		std::this_thread::sleep_for(std::chrono::milliseconds(std::uint32_t(1000.0 * 2 * pController->cycleDuration()) + 1000));

	Point3DCloud::Seq testClouds(1);
	// optional - use depth camera: capture point cloud
	//		pSensor->capture(testClouds[0]);
	// alternative: load from disk
	loadPCLCloud<pcl::PointXYZRGBNormal>("GolemInteropPluginContactLearning_Test.pcd", testClouds[0]);
	printf("TEST: points=%zu\n", testClouds[0].size());

	// compute features
	Feature3D::Seq testFeatures;
	pContact->findFeatures(testClouds, testFeatures);
	printf("TEST: features=%zu\n", testFeatures.size());

	// Find path hypotheses
	Query query;
	// with user-trained model
	pContact->findQuery(contactModelsMap, testFeatures, query);
	// alternatively with default built-in model (comment out all training part)
	//pContact->findQuery(testFeatures, query);
	printf("TEST: hypotheses=%zu\n", query.paths.size());

	// select feasible contact trajectory
	Trajectory testTrajectory;
	pContact->selectTrajectory(query, testTrajectory);
	printf("TEST: waypoints=%zu\n", testTrajectory.trajectory.size());

	// optional - move robot: obtain current state
	pController->lookupState(pController->time(), state);
	// optional - move robot: find trajectory to the beginning of contact trajectory
	pPlanner->findTrajectory(state.cpos, testTrajectory.trajectory.front().cpos, ctrajectory);

	printf("TEST: grasping!\n");
	// optional - move robot: go to to the beginning of contact trajectory, wait to finish
	pController->sendCommand(ctrajectory.data(), ctrajectory.size());
	pController->waitForTrajectoryEnd();
	// optional - move robot: perform contact trajectory, wait to finish
	pController->sendCommand(testTrajectory.trajectory.data(), testTrajectory.trajectory.size());
	pController->waitForTrajectoryEnd();

	// optional: run interactive menu
	if (pApplication) {
		printf("Ready!\n");
		while (pApplication->dispatch());
	}
}

//------------------------------------------------------------------------------
