/** @file GraspServer.cpp
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
#ifdef _EXTERN_LIB_
// TODO includes
#endif // _EXTERN_LIB_
#include <string>
#include <exception>
#include <cstdlib>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

#ifdef _EXTERN_LIB_
// TODO local definitions
#endif // _EXTERN_LIB_

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
		PCLPointCloudSeq cloudPCLSeq;

		// TEST load cloud from a file
		const std::string path = "GolemInteropPluginContactLearning_Test.pcd";
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
		// receive cloud
		interop::Point3DCloud::Seq cloudSeq;
		interop::StreamRead(stream, cloudSeq);
		// convert clouds
		PCLPointCloudSeq cloudPCLSeq;
		for (auto &cloud : cloudSeq) {
			PCLPointCloud cloudPCL;
			interop::convert(cloud, cloudPCL);
			cloudPCLSeq.push_back(cloudPCL);
		}

		// receive paths
		interop::Path::Seq inpPaths;
		interop::StreamRead(stream, inpPaths);

		interop::Path::Seq outPaths;
		
		// TEST just make a copy
		outPaths = inpPaths;

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
