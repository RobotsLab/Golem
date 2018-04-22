/** @file Part3DHoPViewServer.cpp
*
* @author	Marek Kopicki
* @author	Dominik Belter (PUT)
*
* @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
*
* @license  This file copy is licensed to you under the terms described in
*           the License.txt file included in this distribution.
*
*/

#include <Golem/Data/Part3DHoP/Part3DHoPInterop.h>
#include "GolemInteropStreamSocket.h"
#include "GolemInteropStreamDefs.h"
#include "GolemInteropPCLDefs.h"
#ifdef _HOP3D_LIB_
#include <hop3d/HOP3D/HOP3DBham.h>
#endif // _HOP3D_LIB_
#include <string>
#include <cstdlib>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

#ifdef _HOP3D_LIB_
void fromName(const hop3d::HOP3D& lhop3d, const std::string& name, int& categoryNo, int& objectNo, int& imageNo) {
	hop3d::DatasetInfo info;
	lhop3d.getDatasetInfo(info, false);

	categoryNo = objectNo = imageNo = 0;
	for (auto category = info.categories.begin(); category != info.categories.end(); ++category, ++categoryNo)
		for (auto object = category->objects.begin(); object != category->objects.end(); ++object, ++objectNo)
			for (auto path = object->fullPaths.begin(); path != object->fullPaths.end(); ++path, ++imageNo)
				if (!name.compare(*path))
					return;

	throw std::runtime_error("fromName(): unable to find name: " + name);
}
#endif // _HOP3D_LIB_

//------------------------------------------------------------------------------

namespace golem {
namespace interop {

template <typename _Type> void convert(const _Type& inp, interop::Vec3& out) {
	convert(inp(0), out[0]);
	convert(inp(1), out[1]);
	convert(inp(2), out[2]);
}
template <typename _Type> void convert(const interop::Vec3& inp, _Type& out) {
	convert(inp[0], out(0));
	convert(inp[1], out(1));
	convert(inp[2], out(2));
}

template <typename _Type> void convert(const _Type& inp, interop::Mat33& out) {
	convert(inp(0, 0), out.m[0][0]);	convert(inp(0, 1), out.m[0][1]);	convert(inp(0, 2), out.m[0][2]);
	convert(inp(1, 0), out.m[1][0]);	convert(inp(1, 1), out.m[1][1]);	convert(inp(1, 2), out.m[1][2]);
	convert(inp(2, 0), out.m[2][0]);	convert(inp(2, 1), out.m[2][1]);	convert(inp(2, 2), out.m[2][2]);
}
template <typename _Type> void convert(const interop::Mat33& inp, _Type& out) {
	convert(inp.m[0][0], out(0, 0));	convert(inp.m[0][1], out(0, 1));	convert(inp.m[0][2], out(0, 2));
	convert(inp.m[1][0], out(1, 0));	convert(inp.m[1][1], out(1, 1));	convert(inp.m[1][2], out(1, 2));
	convert(inp.m[2][0], out(2, 0));	convert(inp.m[2][1], out(2, 1));	convert(inp.m[2][2], out(2, 2));
}

#ifdef _HOP3D_LIB_
template <> void convert(const hop3d::Quaternion& inp, interop::Quat& out) {
	convert(inp.w(), out.w);
	convert(inp.x(), out.x);
	convert(inp.y(), out.y);
	convert(inp.z(), out.z);
}
template <> void convert(const interop::Quat& inp, hop3d::Quaternion& out) {
	convert(inp.w, out.w());
	convert(inp.x, out.x());
	convert(inp.y, out.y());
	convert(inp.z, out.z());
}

template <> void convert(const hop3d::Mat34& inp, interop::Mat34& out) {
	convert(inp.matrix().block<3, 1>(0, 3), out.p);
	convert(inp.matrix().block<3, 3>(0, 0), out.R);
}
template <> void convert(const interop::Mat34& inp, hop3d::Mat34& out) {
    convert(inp.p, const_cast<Eigen::Block<Eigen::Matrix<double, 4, 4>, 3, 1, false>&>((const Eigen::Block<Eigen::Matrix<double, 4, 4>, 3, 1, false>&)out.matrix().block<3, 1>(0, 3)));
    convert(inp.R, const_cast<Eigen::Block<Eigen::Matrix<double, 4, 4>, 3, 3, false>&>((const Eigen::Block<Eigen::Matrix<double, 4, 4>, 3, 3, false>&)out.matrix().block<3, 3>(0, 0)));
}

template <> void convert(const hop3d::Mat34& inp, interop::Frame3D& out) {
	convert(hop3d::Vec3(inp.matrix().block<3, 1>(0, 3)), out.p);
	convert(hop3d::Quaternion(inp.matrix().block<3, 3>(0, 0)), out.q);
}
template <> void convert(const interop::Frame3D& inp, hop3d::Mat34& out) {
	convert(inp.p, const_cast<Eigen::Block<Eigen::Matrix<double, 4, 4>, 3, 1, false>&>((const Eigen::Block<Eigen::Matrix<double, 4, 4>, 3, 1, false>&)out.matrix().block<3, 1>(0, 3)));
	hop3d::Quaternion quat;
	convert(inp.q, quat);
	out.matrix().block<3, 3>(0, 0) = hop3d::Mat33(quat);
}

template <> void convert(const hop3d::PointCloud& inp, std::vector<interop::Point3D>& out) {
	out.clear();
	out.reserve(inp.size());
	for (const auto &pointHop : inp){
		interop::Point3D point;
		// point and normal
		interop::convert(pointHop.position, point.position);
		interop::convert(pointHop.normal, point.normal);
		// colour: TODO
		out.push_back(point);
	}
}
template <> void convert(const std::vector<interop::Point3D>& inp, hop3d::PointCloud& out) {
	out.clear();
	out.reserve(inp.size());
	for (const auto &point : inp){
		hop3d::PointNormal pointHop;
		// point and normal
		interop::convert(point.position, pointHop.position);
		interop::convert(point.normal, pointHop.normal);
		// colour: TODO
		out.push_back(pointHop);
	}
}

void convert(const hop3d::PointCloud& inp, const hop3d::Mat34& frame, interop::Point3DCloud& out) {
	convert(inp, (std::vector<interop::Point3D>&)out);
	convert(frame, out.frame);
}
void convert(const interop::Point3DCloud& inp, hop3d::PointCloud& out, hop3d::Mat34& frame) {
    convert((const std::vector<interop::Point3D>&)inp, out);
	convert(inp.frame, frame);
}

void convert(const hop3d::ViewIndependentPart::Part3D& inp, interop::part3d::Part3D& out) {
	convert(inp.id, out.model);
	convert(inp.pose, out.frame);
}
void convert(const std::vector<hop3d::ViewIndependentPart::Part3D>& inp, interop::part3d::Part3D::Map& out) {
	out.clear();
	for (const auto& partHop : inp) {
		interop::part3d::Part3D part;
		convert(partHop, out[partHop.realisationId]);
	}
}
#endif // _HOP3D_LIB_

};
};

//------------------------------------------------------------------------------

#ifdef _HOP3D_LIB_
void requestHandler(interop::Stream& stream, hop3d::HOP3D& lhop3d)
#else // _HOP3D_LIB_
void requestHandler(interop::Stream& stream)
#endif // _HOP3D_LIB_
{
	// first 4 bytes encode request type
	std::uint32_t cmd;
	interop::StreamRead(stream, cmd);

	// respond to request
	switch (cmd) {
	case interop::part3d::HOP_REQ_TRAINING_CLOUD_LIST:
	{
#ifdef _HOP3D_LIB_
		hop3d::DatasetInfo info;
		lhop3d.getDatasetInfo(info, false);
#endif // _HOP3D_LIB_

		// create cloud names as paths
		interop::StringSeq names;
		//for (const auto &category : info.categories)
		//	for (const auto &object : category.objects)
		//		for (const auto &path : object.fullPaths)
		//			names.push_back(path);
#ifdef _HOP3D_LIB_
		lhop3d.getCloudPaths(names, false);
#endif // _HOP3D_LIB_

		interop::StreamWrite(stream, names);
		break;
	}
	case interop::part3d::HOP_REQ_PART_MODEL_LIST:
	{
		interop::part3d::IndexSetMap models;

		// convert model graph
#ifdef _HOP3D_LIB_
		hop3d::Hierarchy::IndexSeqMap modelsHop;
		lhop3d.getHierarchy(modelsHop);
		interop::convert(modelsHop, models);
#endif // _HOP3D_LIB_

		interop::StreamWrite(stream, models);
		break;
	}
	case interop::part3d::HOP_REQ_TRAINING_DATA:
	{
		// receive training cloud name
		std::string name;
		interop::StreamRead(stream, name);

		interop::Point3DCloud cloud;
		interop::part3d::IndexSetMap realisations;
		interop::part3d::Part3D::Map parts;
		interop::part3d::IndexSetMap indices;
		
		// create training data
		//int categoryNo, objectNo, imageNo;
		//fromName(lhop3d, name, categoryNo, objectNo, imageNo);

		// convert cloud
#ifdef _HOP3D_LIB_
		hop3d::PointCloud cloudHop;
		//lhop3d.getCloud(categoryNo, objectNo, imageNo, cloudHop, false);
		lhop3d.getCloud(name, cloudHop, false);
		hop3d::Mat34 frameHop;
		//lhop3d.getSensorFrame(categoryNo, objectNo, imageNo, frameHop, false);
		lhop3d.getSensorFrame(name, frameHop, false);
		interop::convert(cloudHop, frameHop, cloud);
#endif // _HOP3D_LIB_

		// convert realisation graph
#ifdef _HOP3D_LIB_
		hop3d::Hierarchy::IndexSetMap realisationsHop;
		//lhop3d.getRealisationsGraph(categoryNo, objectNo, imageNo, realisationsHop, false);
		lhop3d.getRealisationsGraph(name, realisationsHop, false);
		interop::convert(realisationsHop, realisations);
#endif // _HOP3D_LIB_

		// convert parts
#ifdef _HOP3D_LIB_
		std::vector<hop3d::ViewIndependentPart::Part3D> partsHop;
		//lhop3d.getPartsRealisation(categoryNo, objectNo, imageNo, partsHop, false);
		lhop3d.getPartsRealisation(name, partsHop, false);
		interop::convert(partsHop, parts);
#endif // _HOP3D_LIB_

		// convert indices
#ifdef _HOP3D_LIB_
		hop3d::HOP3D::PartsClouds indicesHop;
		//lhop3d.getPartsRealisationCloud(categoryNo, objectNo, imageNo, indicesHop, false);
		lhop3d.getPartsRealisationCloud(name, indicesHop, false);
		interop::convert(indicesHop, indices);
#endif // _HOP3D_LIB_

		interop::StreamWrite(stream, cloud);
		interop::StreamWrite(stream, realisations);
		interop::StreamWrite(stream, parts);
		interop::StreamWrite(stream, indices);
		break;
	}
	case interop::part3d::HOP_REQ_TEST_DATA:
	{
		// receive test cloud
		interop::Point3DCloud inputCloud;
		interop::StreamRead(stream, inputCloud);

		// convert input cloud
		typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PCLPointCloud;
		PCLPointCloud::Ptr inputCloudPCL(new PCLPointCloud());
		interop::convert(inputCloud, *inputCloudPCL);

		// inference
		const std::string name = "testCloud.pcd"; // temporary
		std::map<std::string, PCLPointCloud::Ptr> inputCloudPCLMap;
		inputCloudPCLMap.insert(std::make_pair(name, inputCloudPCL)); // one image
#ifdef _HOP3D_LIB_
		lhop3d.inference(inputCloudPCLMap);
#endif // _HOP3D_LIB_

		interop::Point3DCloud outputCloud;
		interop::part3d::IndexSetMap realisations;
		interop::part3d::Part3D::Map parts;
		interop::part3d::IndexSetMap indices;

		// convert cloud
#ifdef _HOP3D_LIB_
		hop3d::PointCloud cloudHop;
		lhop3d.getCloud(name, cloudHop, true);
		hop3d::Mat34 frameHop;
		lhop3d.getSensorFrame(name, frameHop, true);
		interop::convert(cloudHop, frameHop, outputCloud);
#endif // _HOP3D_LIB_

		// convert realisation graph
#ifdef _HOP3D_LIB_
		hop3d::Hierarchy::IndexSetMap realisationsHop;
		lhop3d.getRealisationsGraph(name, realisationsHop, true);
		interop::convert(realisationsHop, realisations);
#endif // _HOP3D_LIB_

		// convert parts
#ifdef _HOP3D_LIB_
		std::vector<hop3d::ViewIndependentPart::Part3D> partsHop;
		lhop3d.getPartsRealisation(name, partsHop, true);
		interop::convert(partsHop, parts);
#endif // _HOP3D_LIB_

		// convert indices
#ifdef _HOP3D_LIB_
		hop3d::HOP3D::PartsClouds indicesHop;
		lhop3d.getPartsRealisationCloud(name, indicesHop, true);
		interop::convert(indicesHop, indices);
#endif // _HOP3D_LIB_

		interop::StreamWrite(stream, outputCloud);
		interop::StreamWrite(stream, realisations);
		interop::StreamWrite(stream, parts);
		interop::StreamWrite(stream, indices);
		break;
	}
	default:
		throw std::runtime_error("Unknown request: " + std::to_string(cmd));
	}
}

//------------------------------------------------------------------------------

int main(int argc, char* argv[]) {
	if (argc < 3) {
		fprintf(stdout, "Usage: GraspPart3DHoPViewServer <server_port> <hop3d_config> [<hop3d_hierarchy>]");
		return 0;
	}

	try {
		// parameters
		const unsigned short port((unsigned short)std::strtol(argv[1], nullptr, 10));
		const std::string config(argv[2]);
		const std::string hierarchy(argc > 3 ? argv[3] : "");

#ifdef _HOP3D_LIB_
		// initialise HOP3D
		hop3d::HOP3D* lhop3d = hop3d::createHOP3DBham(config);

		// learn or load hierarchy of parts
		if (hierarchy.empty())
			lhop3d->learn();
		else
			lhop3d->load(hierarchy);
#endif // _HOP3D_LIB_

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
#ifdef _HOP3D_LIB_
					requestHandler(*stream, *lhop3d);
#else // _HOP3D_LIB_
					requestHandler(*stream);
#endif // _HOP3D_LIB_
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
