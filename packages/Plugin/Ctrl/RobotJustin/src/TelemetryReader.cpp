/** @file TelemetryReader.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/RobotJustin/TelemetryReader.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <Golem/Ctrl/Kuka/KukaLWR.h>
#include <Golem/Ctrl/DLR/DLRHandII.h>
#include <Golem/Ctrl/RobotJustin/util.h>

#ifdef __WIN32__
#include <winsock2.h>
#define SHUT_RDWR SD_BOTH
#else // __LINUX__
#include <sys/select.h>
#include <sys/socket.h>
#include <fcntl.h>
#endif

using namespace golem;
using namespace bham_planner_interface;

//------------------------------------------------------------------------------

TelemetryReader::TelemetryReader(golem::Controller& controller) : controller(controller), context(controller.getContext()), socket(-1) {
}

TelemetryReader::~TelemetryReader() {
	close();
#ifdef __WIN32__
	WSACleanup();
#endif
}

void TelemetryReader::create(const Desc& desc) {
	if (!desc.isValid())
		throw Message(Message::LEVEL_CRIT, "TelemetryReader::create(): Invalid description");

	state.reset(new Controller::State(controller.createState()));
	controller.setToDefault(*state);

#ifdef __WIN32__
	WSADATA wsaData = {0};
	DWORD iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0)
		throw Message(Message::LEVEL_CRIT, "TelemetryReader::create(): WSAStartup failed: %d", iResult);
#endif

	// default UDP connection (affect only open)
	// open socket
	open(desc.port, desc.host, desc.remote);
	// block until data arrives
	context.write("TelemetryReader::create(): Waiting for telemetry data...\n");
	if (recvTelemetryPacket(packet))
		telemetryPacketToState(packet, *this->state);
	context.write("Done!\n");
}

//------------------------------------------------------------------------------

void TelemetryReader::open(unsigned short port, const std::string &host, bool tcp) {
	if (socket != -1)
		return;

	// alternative: use tcp connection
	// open tcp socket
	if (tcp) {
		socket = (int)::socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (socket == -1)
			throw Message(Message::LEVEL_CRIT, "TelemetryReader::open(): TCP socket");

		struct sockaddr_in addr;
		addr.sin_family = AF_INET;
		addr.sin_port = htons(port);
		//addr.sin_addr.s_addr = INADDR_ANY; // todo: bham planner interface target ip
		if(resolve_hostname(host.c_str(), &addr) == -1)
			throw Message(Message::LEVEL_CRIT, "TelemetryReader::open(): cannot resolve target hostname %s", host.c_str());
		if (::connect(socket, (struct sockaddr*)&addr, sizeof(addr)))
			throw Message(Message::LEVEL_CRIT, "TelemetryReader::open(): TCP connect");
	}
	else {
		// open udp socket
		socket = (int)::socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (socket == -1)
			throw Message(Message::LEVEL_CRIT, "TelemetryReader::open(): UDP socket");

		struct sockaddr_in addr;
		addr.sin_family = AF_INET;
		addr.sin_port = htons(port);
		addr.sin_addr.s_addr = INADDR_ANY;

		if (::bind(socket, (struct sockaddr*)&addr, sizeof(addr)))
			throw Message(Message::LEVEL_CRIT, "TelemetryReader::open(): UDP bind");
	}
/*
  select will check for data. blocking recv is okay!
	// Put the socket in non-blocking mode
#ifdef __WIN32__
	int iMode = 1;
	if (::ioctlsocket(socket, FIONBIO, (u_long*)&iMode) != NO_ERROR)
		throw Message(Message::LEVEL_CRIT, "TelemetryReader::open(): ioctlsocket");
#else // __LINUX__
	if (::fcntl(socket, F_SETFL, O_NONBLOCK) < 0)
		throw Message(Message::LEVEL_CRIT, "TelemetryReader::open(): fcntl");
#endif
*/
}

void TelemetryReader::close() {
	if (socket == -1)
		return;

	// close udp socket
	::shutdown(socket, SHUT_RDWR);
#ifndef __WIN32__
	::close(socket);
#endif
	socket = -1;
}

bool TelemetryReader::recv(void* data, size_t size, MSecTmU32 timeWait) {
	if (socket == -1)
		return false;

	// wait no longer than 'timeWait' for new data
	fd_set read_fds;
	FD_ZERO(&read_fds);
	FD_SET(socket, &read_fds);

	timeval timeout = MSecToTimeval(timeWait);
	const int retval = ::select(socket + 1, &read_fds, NULL, NULL, &timeout);
	if (retval == 0) // timeout
		return false;
	if (retval == -1) // error
		throw Message(Message::LEVEL_CRIT, "TelemetryReader::recv(): select");

	// there is data waiting
	if (FD_ISSET(socket, &read_fds)) {
		const int retsize = ::recv(socket, (char*)data, (int)size, 0); // tcp: blocking recv!
//		context.debug("TelemetryReader::recv(): retsize=%d\n", retsize);
		//if (retsize != size) // importatnt for tcp recv!!
		//	throw Message(Message::LEVEL_CRIT, "TelemetryReader::recv(): recv: invalid packet size %d != %d", retsize, size);
		return retsize == size;
	}

	return false;
}

//------------------------------------------------------------------------------

void TelemetryReader::installArmCallback(golem::SingleCtrl& arm) {
	armCallback.reset(new Callback(*this, &arm));
	// because armCallback was NULL before, recvTelemetryPacket could not update the state variable - do it now
	telemetryPacketToState(packet, *this->state);
}

void TelemetryReader::installHandCallback(golem::SingleCtrl& hand) {
	handCallback.reset(new Callback(*this, &hand));
	// because handCallback was NULL before, recvTelemetryPacket could not update the state variable - do it now
	telemetryPacketToState(packet, *this->state);
}

bool TelemetryReader::recvState(Controller::State& state, MSecTmU32 timeWait) {
	if (recvTelemetryPacket(packet, timeWait)) {
		telemetryPacketToState(packet, *this->state);
		state = *this->state; // updated state
		return true;
	}
	state = *this->state; // old state
	return false;
}

//------------------------------------------------------------------------------

bool TelemetryReader::recvTelemetryPacket(telemetry_packet& packet, MSecTmU32 timeWait) {
	return recv(&packet, sizeof(packet), timeWait);
}

void TelemetryReader::telemetryPacketToState(const telemetry_packet& packet, golem::Controller::State& state) const {
	if (armCallback != NULL) {
		const Controller::State::Info info = armCallback->ctrl->getStateInfo();
		const float *cpos = &packet.q_act[3]; // right arm, measured position, skip torso
		state.cpos.set(cpos, cpos + info.getJoints().size(), *info.getJoints().begin()); // 1:1 mapping
		//std::stringstream str;
		//for (Configspace::Index i = info.getJoints().begin(); i < info.getJoints().end(); ++i)
		//	str << state.cpos[i] << ", ";
		//context.write("arm_pos={%s}\n", str.str().c_str());
		// TODO velocity
	}
	if (handCallback != NULL) {
		const Controller::State::Info info = handCallback->ctrl->getStateInfo();
		const float *cpos = &packet.hand_q_act[0]; // right hand, measured position
		const float *tau = &packet.hand_tau_act[0]; // right hand, mesuared position
		const ptrdiff_t forceOffset = handCallback->ctrl->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
		for (Chainspace::Index i = info.getChains().begin(); i < info.getChains().end(); ++i) {
			const Configspace::Index j = info.getJoints(i).begin();
			const size_t k = (i - info.getChains().begin())*DLRHandIIChain::NUM_JOINTS_CTRL;
			state.cpos[j + 0] = cpos[k + 0];
			state.cpos[j + 1] = cpos[k + 1];
			state.cpos[j + 2] = cpos[k + 2];
			state.cpos[j + 3] = cpos[k + 2]; // duplicate joint #2
			state.get<ConfigspaceCoord>(forceOffset)[j + 0] = tau[k + 0];
			state.get<ConfigspaceCoord>(forceOffset)[j + 1] = tau[k + 1];
			state.get<ConfigspaceCoord>(forceOffset)[j + 2] = tau[k + 2];
			state.get<ConfigspaceCoord>(forceOffset)[j + 3] = tau[k + 2];  // duplicate joint #2
		}
		//std::stringstream str;
		//for (Configspace::Index i = info.getJoints().begin(); i < info.getJoints().end(); ++i)
		//	str << state.cpos[i] << " - " << state.get<ConfigspaceCoord>(forceOffset)[i] << ", ";
		//context.write("hand_pos={%s}\n", str.str().c_str());
		// TODO velocity
	}
}

//------------------------------------------------------------------------------

void golem::XMLData(TelemetryReader::Desc &val, XMLContext* xmlcontext, bool create) {
	golem::XMLData("port", val.port, xmlcontext, create);
	golem::XMLData("host", val.host, xmlcontext, create);
	golem::XMLData("remote", val.remote, xmlcontext, create);
}

//------------------------------------------------------------------------------
