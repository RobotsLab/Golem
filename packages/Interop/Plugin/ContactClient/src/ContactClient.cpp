/** @file ContactClient.cpp
*
* @author	Marek Kopicki
*
*/

#include <Golem/Interop/ContactClient/ContactClient.h>
#include <Golem/Interop/ContactClient/ContactClientDefs.h>
#include "GolemInteropContactStreamDefs.h"
#include <exception>

using namespace golem;
using namespace golem::interop;

//------------------------------------------------------------------------------

std::shared_ptr<Interface> pInterface;

GOLEM_INTEROP_LIBRARY_DECLDIR void* InterfaceLoader(const char* param) {
	if (!pInterface)
		pInterface.reset(new ContactClient(std::string(param)));
	return pInterface.get();
}

//------------------------------------------------------------------------------

golem::interop::ContactClient::ContactClient(const std::string& param) {
	const std::string host(param, 0, param.find_first_of(" "));
	const std::string port(param, param.find_last_of(" ") + 1, std::string::npos);
	
	fprintf(stdout, "ContactClient: trying to connect to host: %s port: %s\n", host.c_str(), port.c_str());
	stream = client.connect<StreamSocket>(host, (unsigned short)std::stoul(port));
}

//------------------------------------------------------------------------------

void golem::interop::ContactClient::capture(Point3DCloud& cloud) {
	StreamWrite(*stream, CONTACT_STREAM_SENSORCLOUD_CAPTURE);
	StreamRead(*stream, cloud);
}


void golem::interop::ContactClient::lookupState(double t, Config& state) const {
	StreamWrite(*stream, CONTACT_STREAM_CONTROLLER_LOOKUPSTATE);
	StreamWrite(*stream, t);
	StreamRead(*stream, state);
}

void golem::interop::ContactClient::sendCommand(const Config* command, std::uintptr_t size) {
	StreamWrite(*stream, CONTACT_STREAM_CONTROLLER_SENDCOMMAND);
	StreamWrite(*stream, std::uint32_t(size));
	while (size--)
		StreamWrite(*stream, *command++);
}

bool golem::interop::ContactClient::waitForTrajectoryEnd(double timewait) {
	StreamWrite(*stream, CONTACT_STREAM_CONTROLLER_WAITFORTRAJECTORYEND);
	StreamWrite(*stream, timewait);
	bool value = false;
	StreamRead(*stream, value);
	return value;
}

bool golem::interop::ContactClient::waitForCycleBegin(double timewait) {
	StreamWrite(*stream, CONTACT_STREAM_CONTROLLER_WAITFORCYCLEBEGIN);
	StreamWrite(*stream, timewait);
	bool value = false;
	StreamRead(*stream, value);
	return value;
}

double golem::interop::ContactClient::cycleDuration() const {
	StreamWrite(*stream, CONTACT_STREAM_CONTROLLER_CYCLEDURATION);
	double value = 0.;
	StreamRead(*stream, value);
	return value;
}

double golem::interop::ContactClient::time() const {
	StreamWrite(*stream, CONTACT_STREAM_CONTROLLER_TIME);
	double value = 0.;
	StreamRead(*stream, value);
	return value;
}

void golem::interop::ContactClient::findTarget(const ConfigspaceCoord& cbegin, const WorkspaceCoord& wend, ConfigspaceCoord& cend, WorkspaceDist& werr) {
	StreamWrite(*stream, CONTACT_STREAM_PLANNER_FINDTARGET);
	StreamWrite(*stream, cbegin);
	StreamWrite(*stream, wend);
	StreamRead(*stream, cend);
	StreamRead(*stream, werr);
}

void golem::interop::ContactClient::findTrajectory(const ConfigspaceCoord& cbegin, const ConfigspaceCoord& cend, Config::Seq& ctrajectory) {
	StreamWrite(*stream, CONTACT_STREAM_PLANNER_FINDTRAJECTORY);
	StreamWrite(*stream, cbegin);
	StreamWrite(*stream, cend);
	StreamRead(*stream, ctrajectory);
}

void golem::interop::ContactClient::findFeatures(const Point3DCloud::Seq& inp, Feature3D::Seq& out) {
	StreamWrite(*stream, CONTACT_STREAM_CONTACT_FINDFEATURES);
	StreamWrite(*stream, inp);
	StreamRead(*stream, out);
}

void golem::interop::ContactClient::findModel(const Training3D::Map& training, Model3D::Map& models) {
	StreamWrite(*stream, CONTACT_STREAM_CONTACT_FINDMODEL);
	StreamWrite(*stream, training);
	StreamRead(*stream, models);
}

void golem::interop::ContactClient::findQuery(const Model3D::Map& models, const Feature3D::Seq& features, Query& query) {
	StreamWrite(*stream, CONTACT_STREAM_CONTACT_FINDQUERY);
	StreamWrite(*stream, models);
	StreamWrite(*stream, features);
	StreamRead(*stream, query);
}

void golem::interop::ContactClient::findQuery(const Feature3D::Seq& features, Query& query) {
	StreamWrite(*stream, CONTACT_STREAM_CONTACT_FINDQUERY_FROM_DEFAULT_MODEL);
	StreamWrite(*stream, features);
	StreamRead(*stream, query);
}

void golem::interop::ContactClient::selectTrajectory(const Query& query, Trajectory& trajectory) {
	StreamWrite(*stream, CONTACT_STREAM_CONTACT_SELECTTRAJECTORY);
	StreamWrite(*stream, query);
	StreamRead(*stream, trajectory);
}

void golem::interop::ContactClient::findTrajectories(const Point3DCloud::Seq& clouds, Trajectory::Seq& trajectories) {
	StreamWrite(*stream, CONTACT_STREAM_GRASPCLOUD_FINDTRAJECTORIES);
	StreamWrite(*stream, clouds);
	StreamRead(*stream, trajectories);
}

//------------------------------------------------------------------------------
