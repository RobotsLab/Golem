/** @file ContactServer.cpp
*
* @author	Marek Kopicki
*
*/

#include <Golem/Interop/ContactServer/ContactServer.h>
#include <Golem/Interop/ContactClient/ContactClientDefs.h>
#include "GolemInteropContactStreamDefs.h"
#include <exception>
#include <boost/system/error_code.hpp>

using namespace golem;
using namespace golem::interop;

//------------------------------------------------------------------------------

std::shared_ptr<Interface> pInterface;

GOLEM_INTEROP_LIBRARY_DECLDIR void* InterfaceLoader(const char* param) {
	if (!pInterface)
		pInterface.reset(new ContactServer(std::string(param)));
	return pInterface.get();
}

//------------------------------------------------------------------------------

golem::interop::ContactServer::ContactServer(const std::string& param) :
	MasterConfig(std::string(param, 0, param.find_first_of(" "))),
	port((unsigned short)std::stoul(std::string(param, param.find_last_of(" ") + 1, std::string::npos))),
	server(port),
	terminate(false)
{
}

//------------------------------------------------------------------------------

void golem::interop::ContactServer::handler(Stream& stream, const Map& interfaces) {
	// first 4 bytes encode request type
	std::uint32_t cmd;
	StreamRead(stream, cmd);

	// respond to request
	switch (cmd) {
	case CONTACT_STREAM_SENSORCLOUD_CAPTURE: {
		Point3DCloud cloud;
		getInterface<SensorCloud>(interfaces)->capture(cloud);
		StreamWrite(stream, cloud);
		break;
	}
	case CONTACT_STREAM_CONTROLLER_LOOKUPSTATE: {
		double t;
		StreamRead(stream, t);
		Config state;
		getInterface<Controller>(interfaces)->lookupState(t, state);
		StreamWrite(stream, state);
		break;
	}
	case CONTACT_STREAM_CONTROLLER_SENDCOMMAND: {
		std::uint32_t size;
		StreamRead(stream, size);
		Config::Seq configs(size);
		for (auto& config: configs)
			StreamRead(stream, config);
		getInterface<Controller>(interfaces)->sendCommand(configs.data(), std::uintptr_t(configs.size()));
		break;
	}
	case CONTACT_STREAM_CONTROLLER_WAITFORTRAJECTORYEND: {
		double timewait;
		StreamRead(stream, timewait);
		StreamWrite(stream, getInterface<Controller>(interfaces)->waitForTrajectoryEnd(timewait));
		break;
	}
	case CONTACT_STREAM_CONTROLLER_WAITFORCYCLEBEGIN: {
		double timewait;
		StreamRead(stream, timewait);
		StreamWrite(stream, getInterface<Controller>(interfaces)->waitForTrajectoryEnd(timewait));
		break;
	}
	case CONTACT_STREAM_CONTROLLER_CYCLEDURATION: {
		StreamWrite(stream, getInterface<Controller>(interfaces)->cycleDuration());
		break;
	}
	case CONTACT_STREAM_CONTROLLER_TIME: {
		StreamWrite(stream, getInterface<Controller>(interfaces)->time());
		break;
	}
	case CONTACT_STREAM_PLANNER_FINDTARGET: {
		ConfigspaceCoord cbegin;
		StreamRead(stream, cbegin);
		WorkspaceCoord wend;
		StreamRead(stream, wend);
		ConfigspaceCoord cend;
		WorkspaceDist werr;
		getInterface<Planner>(interfaces)->findTarget(cbegin, wend, cend, werr);
		StreamWrite(stream, cend);
		StreamWrite(stream, werr);
		break;
	}
	case CONTACT_STREAM_PLANNER_FINDTRAJECTORY: {
		ConfigspaceCoord cbegin;
		StreamRead(stream, cbegin);
		ConfigspaceCoord cend;
		StreamRead(stream, cend);
		Config::Seq ctrajectory;
		getInterface<Planner>(interfaces)->findTrajectory(cbegin, cend, ctrajectory);
		StreamWrite(stream, ctrajectory);
		break;
	}
	case CONTACT_STREAM_CONTACT_FINDFEATURES: {
		Point3DCloud::Seq inp;
		StreamRead(stream, inp);
		Feature3D::Seq out;
		getInterface<Contact>(interfaces)->findFeatures(inp, out);
		StreamWrite(stream, out);
		break;
	}
	case CONTACT_STREAM_CONTACT_FINDMODEL: {
		Training3D::Map training;
		StreamRead(stream, training);
		Model3D::Map models;
		getInterface<Contact>(interfaces)->findModel(training, models);
		StreamWrite(stream, models);
		break;
	}
	case CONTACT_STREAM_CONTACT_FINDQUERY: {
		Model3D::Map models;
		StreamRead(stream, models);
		Feature3D::Seq features;
		StreamRead(stream, features);
		Query query;
		getInterface<Contact>(interfaces)->findQuery(models, features, query);
		StreamWrite(stream, query);
		break;
	}
	case CONTACT_STREAM_CONTACT_FINDQUERY_FROM_DEFAULT_MODEL: {
		Feature3D::Seq features;
		StreamRead(stream, features);
		Query query;
		getInterface<Contact>(interfaces)->findQuery(features, query);
		StreamWrite(stream, query);
		break;
	}
	case CONTACT_STREAM_CONTACT_SELECTTRAJECTORY: {
		Query query;
		StreamRead(stream, query);
		Trajectory trajectory;
		getInterface<Contact>(interfaces)->selectTrajectory(query, trajectory);
		StreamWrite(stream, trajectory);
		break;
	}
	case CONTACT_STREAM_GRASPCLOUD_FINDTRAJECTORIES: {
		Point3DCloud::Seq clouds;
		StreamRead(stream, clouds);
		Trajectory::Seq trajectories;
		getInterface<GraspCloud>(interfaces)->findTrajectories(clouds, trajectories);
		StreamWrite(stream, trajectories);
		break;
	}
	default:
		throw std::runtime_error("Unknown request: " + std::to_string(cmd));
	}
}

//------------------------------------------------------------------------------

void golem::interop::ContactServer::run(const Map& interfaces) {
	Application* pApplication = getInterface<Application>(interfaces, true);
	if (pApplication)
		pApplication->setCallbackTerminate([&] () {
			fprintf(stdout, "ContactServer: Terminating...\n");
			this->terminate = true;
			boost::system::error_code ec = boost::system::errc::make_error_code(boost::system::errc::errc_t::success);
			server.getAcceptor().close(ec);
		});

	while (!this->terminate) {
		// wait for incomming connection
		fprintf(stdout, "ContactServer: Listening on port: %i\n", port);
		Stream::Ptr stream = server.accept<StreamSocket>();

		// debug info
		const boost::asio::ip::tcp::socket& socket = static_cast<const StreamSocket&>(*stream).getSocket();
		fprintf(stdout, "ContactServer: Accepted connection from host: %s, port: %i\n", socket.remote_endpoint().address().to_string().c_str(), socket.remote_endpoint().port());

		try {
			// handle requests
			while (!this->terminate)
				handler(*stream, interfaces);
		}
		catch (const std::exception& ex) {
			fprintf(stderr, "ContactServer: %s\n", ex.what());
		}
	}
}

//------------------------------------------------------------------------------
