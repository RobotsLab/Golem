/** @file GraspServerNNLearning.cpp
*
* @author	Marek Kopicki
*
* @copyright  Copyright (C) 2018 Marek Kopicki, University of Birmingham, UK
*
* @license  This file copy is licensed to you under the terms described in
*           the License.txt file included in this distribution.
*
*/

#include <Golem/App/Grasp/GraspInterop.h>
#include "GolemInteropStreamSocket.h"
#include "GolemInteropContactStreamDefs.h"
#include "GolemInteropPCLDefs.h"
#include "util/Path.h"
#include "sensor/VirtualCamera.h"
#ifdef _EXTERN_LIB_
// TODO includes
#endif // _EXTERN_LIB_
#include <string>
#include <exception>
#include <cstdlib>
#include <sys/stat.h>
#include <windows.h>
#include <stdio.h>

char str[] = "cloud.png";
char strTrj[] = "data.trj";
char strData[] = "data.bin";
char strRes[] = "result.txt";

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

#ifdef _EXTERN_LIB_
// TODO local definitions
#endif // _EXTERN_LIB_

std::vector<std::vector<float>> ReadTrajectories(int numberOfGrasps, Eigen::Vector3f camPos, Eigen::Vector3f gazeDir, Eigen::Vector3f objectCenter);


int objectCounter = 24;
//------------------------------------------------------------------------------

void requestHandler(interop::Stream& stream) {
	typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PCLPointCloud;
	typedef std::vector<PCLPointCloud> PCLPointCloudSeq;
		
	// first 4 bytes encode request type
	std::uint32_t cmd;
	interop::StreamRead(stream, cmd);

	// respond to request
	switch (cmd) {
	case interop::GRASP_INTEROP_INPUT_CLOUD:
	{

		char cloudStr[1000], objectId[10];
		strcpy(cloudStr, "./objects-test-raw/data-item-image-");
		sprintf(objectId, "%d", objectCounter);
		strcat(cloudStr, objectId);
		strcat(cloudStr, ".pcd");
		if (objectCounter < 49)
			objectCounter++;
		else objectCounter = 1;

		std::cout << "The client requested a point cloud. Sending test point cloud:" << cloudStr << std::endl;
		PCLPointCloudSeq cloudPCLSeq;

		// TEST load cloud from a file
		const std::string path = cloudStr;
		cloudPCLSeq.resize(1);
		if (pcl::PCDReader().read(path, cloudPCLSeq[0]) != 0)
			throw std::runtime_error("pcl::PCDReader(): unable to read from " + path);

#ifdef _EXTERN_LIB_
		// TODO generate cloud(s)
		// TODO convert to cloudPCLSeq
#endif // _EXTERN_LIB_

		// convert clouds
		interop::Point3DCloud::Seq cloudSeq;
		for (auto &cloudPCL : cloudPCLSeq) {
			interop::Point3DCloud cloud;
			interop::convert(cloudPCL, cloud);
			cloudSeq.push_back(cloud);
		}

		interop::StreamWrite(stream, cloudSeq);
		break;
	}
	case interop::GRASP_INTEROP_TRAINING:
	case interop::GRASP_INTEROP_INFERENCE:
	{

		std::cout << "Inference started." << std::endl;
		// receive cloud
		interop::Point3DCloud::Seq cloudSeq;
		interop::StreamRead(stream, cloudSeq);
		// convert clouds
		PCLPointCloud cloudPCL;
		PCLPointCloudSeq cloudPCLSeq;
		for (auto &cloud : cloudSeq) {
			interop::convert(cloud, cloudPCL);
			cloudPCLSeq.push_back(cloudPCL);
			break; // we only process one point cloud
		}

		// Remove all files
		std::cout << "Removing old files." << std::endl;
		remove(str);
		remove(strTrj);
		remove(strData);
		remove(strRes);

		// receive paths
		interop::Path::Seq inpPaths;
		interop::StreamRead(stream, inpPaths);

		std::cout << "Input paths read. Number of trajectories:" << inpPaths.size() << std::endl;

		// Get camera frame, 
		Eigen::Vector4f sensorOrigin(cloudPCL.sensor_origin_.x(), cloudPCL.sensor_origin_.y(), cloudPCL.sensor_origin_.z(),1.0);
		Eigen::Quaternionf sensorOrientation = Eigen::Quaternionf(cloudPCL.sensor_orientation_.w(), cloudPCL.sensor_orientation_.x(), cloudPCL.sensor_orientation_.y(), cloudPCL.sensor_orientation_.z());
		
		std::cout << "Creating the trajectory file." << std::endl;
		// Write trajectories to file
		// Go through each and every grasp and print grasp parameters.
		FILE * fp = fopen(strTrj, "wb");
		int totalCounter = 0;
		float tmp[7];
		float fingers[20];
		bool validGrasps[10000];

		for (int i = 0; i < inpPaths.size(); i++)
		{
			validGrasps[i] = true;

			// Obtain path
			interop::Path path = inpPaths[i];

			// Get grasp parameters
			interop::ConfigModel::Seq wp = path.path;
			unsigned int graspType = path.space;
			if (graspType < 0 || graspType > 9)
				graspType = 0;
			int pathSize = wp.size();
	//		std::cout << "Trajectory " << i << " has size " << pathSize << " and type " << graspType << std::endl;

			fwrite(&graspType, 4, 1, fp);
			fwrite(&pathSize, 4, 1, fp);

			// Go through the waypoints
			for (int k = 0; k < wp.size(); k++)
			{
				// Obtain data
				interop::ConfigspaceCoord config = wp[k].config;
				interop::Frame3D frame = wp[k].frame;

				// Frame
				interop::Vec3 location = frame.p;
				interop::Quat quat = frame.q;
					
				// Very simple collision check
				if (location[2] < -0.35)
					validGrasps[i] = false;

				// Get joint parameters
				tmp[0] = (float)location[0], tmp[1] = (float)location[1], tmp[2] = (float)location[2];
				tmp[3] = (float)quat.x, tmp[4] = (float)quat.y, tmp[5] = (float)quat.z, tmp[6] = (float)quat.w;
				for (int l = 7; l < 27; l++)
					fingers[l - 7] = (float)config[l];

				// Write to file
				fwrite(tmp, 4, 7, fp);
				fwrite(fingers, 4, 20, fp);
			}

			// Convert path into 

			totalCounter++;
		}
		fclose(fp);


		std::cout << "Reprojecting the point cloud." << std::endl;

		// Obtain depth image from the cloud
		cv::Mat depth_map = cv::Mat(480, 640, CV_16UC1);
		Eigen::Vector3f newGaze, centerPoint;
		Grasp::VirtualCamera vc;
		vc.ReprojectPointCloud(&cloudPCL, depth_map, sensorOrigin, newGaze, centerPoint,
			sensorOrientation, sensorOrientation, true);
		std::cout << "New gaze of the camera:" << newGaze(0) << " " << newGaze(1) << " " << newGaze(2) << std::endl;
		std::cout << "New center point of the scene:" << centerPoint(0) << " " << centerPoint(1) << " " << centerPoint(2) << std::endl;

		// Write depth image to a file
		// Crop first
		cv::Rect roi;
		roi.x = 90;
		roi.y = 10;
		roi.width = 460;
		roi.height = 460;

		// Write depth image
		std::cout << "Depth file being created now." << std::endl;
		cv::Mat cropDepth = depth_map(roi);
		cv::Mat outDepth;
		cv::flip(cropDepth, outDepth, -1);
		cv::Mat dstDepth, filteredDepth;
		medianBlur(outDepth, filteredDepth, 3);
		// We had to implement our own resize function here.
		cv::resize(filteredDepth, filteredDepth, cv::Size(224, 224), 0, 0, cv::INTER_NEAREST);
		cv::imwrite(str, filteredDepth);

		std::cout << "Reading trajectories from file." << std::endl;
		std::cout << totalCounter << "trajectories. Sensor origin:" << sensorOrigin(0) << " " << sensorOrigin(1) << " " << sensorOrigin(2) << std::endl;
		std::cout << "GAZE:" << newGaze(0) << " " << newGaze(1) << " " << newGaze(2) << std::endl;
		// Read trajectories from the file and obtain full grasp parameters.
		std::vector<std::vector<float>> graspParams = ReadTrajectories(totalCounter, 
			Eigen::Vector3f(sensorOrigin(0), sensorOrigin(1), sensorOrigin(2)), newGaze, centerPoint);

		// Print grasp params to a file
		std::cout << "Processing trajectories." << strData << std::endl;
		FILE * dataFP = fopen(strData, "wb");
		float success = 0;
		float stabilityArr[] = { 0.0, 0.0, 0.0, 0.0 };
		for (int i = 0; i < totalCounter; i++)
		{
			fwrite(&success, 4, 1, dataFP); // Writing success probability
			fwrite(stabilityArr, 4, 4, dataFP); // Writing stability values
			std::vector<float> tmpVec = graspParams[i];
			float * tmpArr = new float[tmpVec.size()];
			for (int k = 0; k < tmpVec.size(); k++)
			{
				tmpArr[k] = graspParams[i][k];
			}
			fwrite(tmpArr, 4, tmpVec.size(), dataFP); // Writing grasp parameters
		}
		fclose(dataFP);

		std::cout << "Now, we'll be waiting for the neural network prediction results." << std::endl;
		// Wait here till the results are back.
		while (true)
		{
			struct stat buf;
			if (stat(strRes, &buf) == 0)
			{
				std::cout << "Arrived!" << inpPaths.size() << std::endl;
				Sleep(1000); // Sleep for a second
				break;
			}
			else{
				Sleep(1000); // Sleep for a second
			}
		}

		std::cout << "Re-ordering the trajectories based on the ranking, and sending them back. "<< std::endl;
		// The results are back! Re-order the paths based on the received ranking.
		interop::Path::Seq outPaths;
		FILE * resFP = fopen(strRes, "r");
		int graspId;
		for (int i = 0; i < totalCounter; i++)
		{
			fscanf(resFP, "%d", &graspId);
		//	if (validGrasps[graspId])
			outPaths.push_back(inpPaths[graspId]);
		}
		fclose(resFP);

		std::cout<<"Returned path count: "<<outPaths.size() << std::endl;

#ifdef _EXTERN_LIB_
		// TODO convert from inpPaths and 
		if (cmd == interop::GRASP_INTEROP_TRAINING) {
			// TODO training
			// TODO convert successful paths to outPaths, remove unsuccessful ones
		}
		else {
			// TODO inference
			// TODO convert tested paths to outPaths with updated likelihoods (weights), sort according to likelihoods
		}
#endif // _EXTERN_LIB_

		interop::StreamWrite(stream, outPaths);
		break;
	}
	default:
		throw std::runtime_error("Unknown request: " + std::to_string(cmd));
	}
}

//------------------------------------------------------------------------------

int main(int argc, char* argv[]) {
	if (argc < 2) {
		fprintf(stdout, "Usage: GraspServer <server_port> [<extern_config>]");
		return 0;
	}

	try {
		// parameters
		const unsigned short port((unsigned short)std::strtol(argv[1], nullptr, 10));
		const std::string config(argc > 2 ? argv[2] : "GraspServer.config");

#ifdef _EXTERN_LIB_
		// TODO initialisation
#endif // _EXTERN_LIB_

		// run server
		interop::Server server(port);
		for (;;) {
			// wait for incomming connection
			fprintf(stdout, "Listening on port: %i\n", port);
			interop::Stream::Ptr stream = server.accept<interop::StreamSocket>();

			// debug info
			const boost::asio::ip::tcp::socket& socket = static_cast<const interop::StreamSocket&>(*stream).getSocket();
			fprintf(stdout, "Accepted connection from host: %s, port: %i\n", socket.remote_endpoint().address().to_string().c_str(), socket.remote_endpoint().port());

			try {
				// handle requests
				for (;;)
					requestHandler(*stream);
			}
			catch (const std::exception& ex) {
				fprintf(stderr, "%s\n", ex.what());
			}
		}
	}
	catch (const std::exception& ex) {
		fprintf(stderr, "%s\n", ex.what());
		return 1;
	}

	return 0;
}

std::vector<std::vector<float>> ReadTrajectories(int numberOfGrasps, Eigen::Vector3f camPos, Eigen::Vector3f gazeDir, Eigen::Vector3f objectCenter){
	FILE* fp = fopen(strTrj, "rb");
	Grasp::Path **finalApproachArr = new Grasp::Path*[numberOfGrasps];
	std::vector<std::vector<float>> graspParams;

	std::cout << "Path created" << std::endl;

	for (int readCtr = 0; readCtr<numberOfGrasps; readCtr++)
	{
		// Reads the next trajectory from the file, and sets path variable.
		float extraGrip = 0.523; // 0.523 for 30 degrees
		int wpCount = 0;
		int graspType = 0;
		float wristPos[3];
		float wristQuat[4];
		float fingerPos[20];
		fread(&graspType, 4, 1, fp);
		fread(&wpCount, 4, 1, fp);
		if (graspType < 0 || graspType > 9)
			graspType = 0;
		finalApproachArr[readCtr] = new Grasp::Path(wpCount + 2);
		finalApproachArr[readCtr]->graspType = graspType;

		// Read trajectory waypoint by waypoint.
		for (int i = 1; i< wpCount + 2; i++)
		{
			if (i<wpCount + 1)
			{
				fread(wristPos, 4, 3, fp);
				fread(wristQuat, 4, 4, fp);
				fread(fingerPos, 4, 20, fp);
			}

			float multiplyFactor = 0, maxVal = 0;
			float differences[20];
			if (i == wpCount + 1)
			{
				for (int k = 0; k<20; k++){
					float newVal = finalApproachArr[readCtr]->waypoints[i - 1].jointAngles[k] -
						finalApproachArr[readCtr]->waypoints[i - 2].jointAngles[k];
					differences[k] = newVal;
					if (newVal > maxVal)
						maxVal = newVal;
				}
				multiplyFactor = extraGrip / maxVal;
			}
			else
			{
				for (int k = 0; k<20; k++)
					differences[k] = 0;
			}

			// Read waypoints.
			finalApproachArr[readCtr]->waypoints[i].pos[0] = wristPos[0], finalApproachArr[readCtr]->waypoints[i].pos[1] = wristPos[1], finalApproachArr[readCtr]->waypoints[i].pos[2] = wristPos[2];
			finalApproachArr[readCtr]->waypoints[i].quat.x() = wristQuat[0];
			finalApproachArr[readCtr]->waypoints[i].quat.y() = wristQuat[1];
			finalApproachArr[readCtr]->waypoints[i].quat.z() = wristQuat[2];
			finalApproachArr[readCtr]->waypoints[i].quat.w() = wristQuat[3];

			for (int k = 0; k<20; k++)
			{
				if (!(k % 4))
				{
					// Sideways joints
					if (!k)
						finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = fingerPos[k] + 0.05; // 0.087266
					else if (k == 4)
						finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = fingerPos[k] + 0.043633;
					else if (k == 8)
						finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = fingerPos[k];
					else if (k == 12)
						finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = fingerPos[k] - 0.043633;
					else if (k == 16)
						finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = fingerPos[k] - (0.0537);
				}
				else if (k % 4 == 2 || k % 4 == 3)
					// Middle joints
					finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = fingerPos[k] + 0.087266 + multiplyFactor * differences[k];
				else if (k < 4)
				{	// thumb
					finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = (fingerPos[k] - 0.02908866667) + multiplyFactor * differences[k];
				}
				else if (k < 8)
				{	// index
					finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = fingerPos[k] + (0.07) + multiplyFactor * differences[k];
				}
				else if (k < 12)
				{	// middle
					finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = (fingerPos[k] - 0.01) + multiplyFactor * differences[k];
				}
				else if (k < 16)
				{	// ring
					finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = (fingerPos[k] - 0.06) + multiplyFactor * differences[k];
				}
				else{	// little
					finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = (fingerPos[k] - 0.160899) + multiplyFactor * differences[k];
				}
			}
		}

		// Set the first waypoint as an initial approach from outside.
		finalApproachArr[readCtr]->waypoints[0].pos[0] = 2 * finalApproachArr[readCtr]->waypoints[1].pos[0] - objectCenter(0);
		finalApproachArr[readCtr]->waypoints[0].pos[1] = 2 * finalApproachArr[readCtr]->waypoints[1].pos[1] - objectCenter(1);
		finalApproachArr[readCtr]->waypoints[0].pos[2] = finalApproachArr[readCtr]->waypoints[1].pos[2];
		finalApproachArr[readCtr]->waypoints[0].quat.x() = finalApproachArr[readCtr]->waypoints[1].quat.x();
		finalApproachArr[readCtr]->waypoints[0].quat.y() = finalApproachArr[readCtr]->waypoints[1].quat.y();
		finalApproachArr[readCtr]->waypoints[0].quat.z() = finalApproachArr[readCtr]->waypoints[1].quat.z();
		finalApproachArr[readCtr]->waypoints[0].quat.w() = finalApproachArr[readCtr]->waypoints[1].quat.w();
		for (int k = 0; k<20; k++)
			finalApproachArr[readCtr]->waypoints[0].jointAngles[k] = 0;

		// Finally, save grasp parameter data
		std::vector<float> curParams = finalApproachArr[readCtr]->getGraspParams(gazeDir, camPos);
		graspParams.push_back(curParams);
	}
	fclose(fp);
	return graspParams;
}