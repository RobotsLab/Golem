/** @file KukaKR5Sixx.cpp
 * 
 * @author	Chris Burbridge
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki and Chris Burbridge, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/KukaKR5Sixx/KukaKR5Sixx.h>
#include <Golem/Ctrl/KukaKR5Sixx/Data.h>
#include <Golem/Sys/LoadObjectDesc.h>
#ifdef WIN32
#include <winsock2.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc) {
	loadObjectDesc<golem::KukaKR5SixxRSI::Desc, Controller::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Controller::Desc::Ptr*)pControllerDesc);
}

//------------------------------------------------------------------------------

KukaKR5SixxRSI::KukaKR5SixxRSI(golem::Context& context) : KukaKR5Sixx(context), serverfd(-1), clientfd(-1) {
}

KukaKR5SixxRSI::~KukaKR5SixxRSI() {
	SingleCtrl::release();

#ifdef WIN32
	if (clientfd >= 0)
		::closesocket(clientfd);
	if (serverfd >= 0)
		::closesocket(serverfd);

	WSACleanup();
#else // WIN32
	if (clientfd >= 0)
		::close(clientfd);
	if (serverfd >= 0)
		::close(serverfd);
#endif // WIN32
}

//------------------------------------------------------------------------------

void KukaKR5SixxRSI::create(const Desc& desc) {
	localPort = desc.localPort;
	driveMode = desc.driveMode;
	feedbackGain = desc.feedbackGain;

#ifdef WIN32
	::WSADATA wsaData;
	if (::WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
		throw Message(Message::LEVEL_CRIT, "KukaKR5SixxRSI::create(): unable to initialise sockets: %d", WSAGetLastError());
#endif // WIN32

	KukaKR5Sixx::create(desc);
}

void KukaKR5SixxRSI::userCreate() {
	state.reset(new State(createState()));
	setToDefault(*state);
	joint = getStateInfo().getJoints().begin();

	isConnected = false; // not connected to begin with
	incomingDataPos = 0;

	// XML input buffer Default size 10K
	incomingDataBuffer.resize(10240);
	// Formatted XML output buffer
	outgoingDataBuffer.resize(10240);

	currentCommand.axisCorrection.a1 = currentCommand.axisCorrection.a2 = currentCommand.axisCorrection.a3 =
			currentCommand.axisCorrection.a4 = currentCommand.axisCorrection.a5 = currentCommand.axisCorrection.a6 = 0;
	currentCommand.cartCorrection.x = currentCommand.cartCorrection.y = currentCommand.cartCorrection.z =
			currentCommand.cartCorrection.a = currentCommand.cartCorrection.b = currentCommand.cartCorrection.c = 0;

	setMessage("Message from PC: comms initiated.");

	headState = 0;
	received_count = 0;

	// Create streaming socket
	serverfd = (int)::socket(AF_INET, SOCK_STREAM, 0);
	if (serverfd < 0)
		throw Message(Message::LEVEL_CRIT, "KukaKR5SixxRSI::userCreate(): unable to create socket: %s", strerror(errno));

	// Assign a port number to the socket
	sockaddr_in serveraddr;
	::memset(&serveraddr, 0, sizeof(serveraddr));
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_port = htons(localPort);
	serveraddr.sin_addr.s_addr = INADDR_ANY;
    if (::bind(serverfd, (sockaddr*)&serveraddr, sizeof(serveraddr)) != 0)
		throw Message(Message::LEVEL_CRIT, "KukaKR5SixxRSI::userCreate(): unable to bind port %d: %s", localPort, strerror(errno));

	// server socket with one incomming connection
	if (::listen(serverfd, 1) != 0)
		throw Message(Message::LEVEL_CRIT, "KukaKR5SixxRSI::userCreate(): unable to make server socket: %s", strerror(errno));

	// Start listening on the TCP port, receive positions and respond
	// with corrections.

	for (isConnected = false; !isConnected; ) {
		// This will block until a connection happens.....
		context.info("KukaKR5SixxRSI::userCreate(): Waiting for incomming connection on port %d\n", localPort);

		sockaddr_in clientaddr;
		int clientaddrlen = (int)sizeof(clientaddr);
#ifdef WIN32
		clientfd = (int)::accept(serverfd, (sockaddr*)&clientaddr, &clientaddrlen);
#else
		clientfd = (int)::accept(serverfd, (sockaddr*)&clientaddr, (socklen_t*)&clientaddrlen);
#endif // WIN32
		if (clientfd < 0)
			throw Message(Message::LEVEL_CRIT, "KukaKR5SixxRSI::userCreate(): accept error: %s", strerror(errno));

#ifdef WIN32
#pragma warning (push)
#pragma warning (disable:4996)
#endif
		context.info("KukaKR5SixxRSI::userCreate(): %s:%d connected\n", inet_ntoa(clientaddr.sin_addr), ntohs(clientaddr.sin_port));
#ifdef WIN32
#pragma warning (pop)
#endif

		this->isConnected = true;
	}
}

void KukaKR5SixxRSI::sysSync() {
	// Read an XML package...
	char *location = NULL;
	for (;;) {
		const int len = (int)::recv(clientfd, incomingDataBuffer.data() + incomingDataPos, (int)incomingDataBuffer.size() - (int)incomingDataPos - 1, 0);
		if (len < 0 && errno == 0)
			throw Message(Message::LEVEL_NOTICE, "KukaKR5SixxRSI::sysSync(): connection closed");
		if (len < 0)
			throw Message(Message::LEVEL_ERROR, "KukaKR5SixxRSI::sysSync(): recv error: %s", strerror(errno));

		incomingDataPos += len;

		// stop the string search at the end
		incomingDataBuffer[incomingDataPos+1] = '\0';
		location = strstr((char*)&(incomingDataBuffer[0]), "</Rob>");
		// reached end of robot xml blip
		if (location)
			break;
		// error: not all data arrived, but transfer is finished
		if (len == 0)
			throw Message(Message::LEVEL_ERROR, "KukaKR5SixxRSI::sysSync(): connection closed unexpectedly");
	}

//	std::cout << (location + 6 - incomingDataBuffer ) << " == " << ( incomingDataPos) << std::endl;
	if (location + 6 +2 - incomingDataBuffer.data() == incomingDataPos) { // +2 for cr lf that appears to come?
		//	safe to return to buffer beginning....
		incomingDataPos = 0;
		// TODO: This should be fine, but just in case need to check for stray. if it is never safe then a seg fault will come in.
	}

	// Parse the XML information from the robot
	this->parseIncomingState(incomingDataBuffer.data());


	// Assign to this->state
	this->state->cpos[joint + 0] = currentState[headState].actualAxisPosition.a1 / 180.0 * REAL_PI;
	this->state->cpos[joint + 1] = currentState[headState].actualAxisPosition.a2 / 180.0 * REAL_PI;
	this->state->cpos[joint + 2] = currentState[headState].actualAxisPosition.a3 / 180.0 * REAL_PI;
	this->state->cpos[joint + 3] = currentState[headState].actualAxisPosition.a4 / 180.0 * REAL_PI;
	this->state->cpos[joint + 4] = currentState[headState].actualAxisPosition.a5 / 180.0 * REAL_PI;
	this->state->cpos[joint + 5] = currentState[headState].actualAxisPosition.a6 / 180.0 * REAL_PI;

//	std::cout << "IPOC: \n";
//	std::cout << currentState[headState].ipoc << "\n";

	// interpolate current and previous positions to obtain velocity and acceleration
	int	prevHead = headState-1;
	prevHead = (prevHead < 0) ? 5+prevHead : prevHead;
	int	prevPrevHead = headState-1;
	prevPrevHead = (prevPrevHead < 0) ? 5+prevPrevHead : prevPrevHead;

	// Fill in velocity if enough history
	if (received_count > 0) {
		this->state->cvel[joint + 0] = (currentState[headState].actualAxisPosition.a1 - currentState[prevHead].actualAxisPosition.a1)/0.012;
		this->state->cvel[joint + 1] = (currentState[headState].actualAxisPosition.a2 - currentState[prevHead].actualAxisPosition.a2)/0.012;
		this->state->cvel[joint + 2] = (currentState[headState].actualAxisPosition.a3 - currentState[prevHead].actualAxisPosition.a3)/0.012;
		this->state->cvel[joint + 3] = (currentState[headState].actualAxisPosition.a4 - currentState[prevHead].actualAxisPosition.a4)/0.012;
		this->state->cvel[joint + 4] = (currentState[headState].actualAxisPosition.a5 - currentState[prevHead].actualAxisPosition.a5)/0.012;
		this->state->cvel[joint + 5] = (currentState[headState].actualAxisPosition.a6 - currentState[prevHead].actualAxisPosition.a6)/0.012;
	} else {
		received_count++;
	}

	// Fill in acceleration if enough history
	if (received_count == 2) {
		this->state->cacc[joint + 0] = (currentState[headState].actualAxisPosition.a1 - 2* currentState[prevHead].actualAxisPosition.a1 + currentState[prevHead].actualAxisPosition.a1)/(0.012*0.012);
		this->state->cacc[joint + 1] = (currentState[headState].actualAxisPosition.a2 - 2* currentState[prevHead].actualAxisPosition.a2 + currentState[prevHead].actualAxisPosition.a2)/(0.012*0.012);
		this->state->cacc[joint + 2] = (currentState[headState].actualAxisPosition.a3 - 2* currentState[prevHead].actualAxisPosition.a3 + currentState[prevHead].actualAxisPosition.a3)/(0.012*0.012);
		this->state->cacc[joint + 3] = (currentState[headState].actualAxisPosition.a4 - 2* currentState[prevHead].actualAxisPosition.a4 + currentState[prevHead].actualAxisPosition.a4)/(0.012*0.012);
		this->state->cacc[joint + 4] = (currentState[headState].actualAxisPosition.a5 - 2* currentState[prevHead].actualAxisPosition.a5 + currentState[prevHead].actualAxisPosition.a5)/(0.012*0.012);
		this->state->cacc[joint + 5] = (currentState[headState].actualAxisPosition.a6 - 2* currentState[prevHead].actualAxisPosition.a6 + currentState[prevHead].actualAxisPosition.a6)/(0.012*0.012);
	} else {
		received_count++;
	}


	// TODO find out the actuall received state delay - here we assume it is equal zero
	this->state->t = context.getTimer().elapsed();
}

void KukaKR5SixxRSI::sysRecv(State& state) {
	state = *this->state;
}

void KukaKR5SixxRSI::sysSend(const State& prev, const State& next, bool bSendPrev, bool bSendNext) {
	// set the current command data
	switch (driveMode) {
	case JOINT_CORRECTION:
		currentCommand.axisCorrection.a1 = Math::radToDeg(next.cpos[joint + 0] - prev.cpos[joint + 0] +	feedbackGain[0]*(prev.cpos[joint + 0] - state->cpos[joint + 0])); // in degreees
		currentCommand.axisCorrection.a2 = Math::radToDeg(next.cpos[joint + 1] - prev.cpos[joint + 1] +	feedbackGain[1]*(prev.cpos[joint + 1] - state->cpos[joint + 1])); // in degreees
		currentCommand.axisCorrection.a3 = Math::radToDeg(next.cpos[joint + 2] - prev.cpos[joint + 2] +	feedbackGain[2]*(prev.cpos[joint + 2] - state->cpos[joint + 2])); // in degreees
		currentCommand.axisCorrection.a4 = Math::radToDeg(next.cpos[joint + 3] - prev.cpos[joint + 3] +	feedbackGain[3]*(prev.cpos[joint + 3] - state->cpos[joint + 3])); // in degreees
		currentCommand.axisCorrection.a5 = Math::radToDeg(next.cpos[joint + 4] - prev.cpos[joint + 4] +	feedbackGain[4]*(prev.cpos[joint + 4] - state->cpos[joint + 4])); // in degreees
		currentCommand.axisCorrection.a6 = Math::radToDeg(next.cpos[joint + 5] - prev.cpos[joint + 5] +	feedbackGain[5]*(prev.cpos[joint + 5] - state->cpos[joint + 5])); // in degreees
		break;
	case CARTESIAN_CORRECTION:
		currentCommand.cartCorrection.x = 0.;
		currentCommand.cartCorrection.y = 0.;
		currentCommand.cartCorrection.z = 0.;
		currentCommand.cartCorrection.a = 0.;
		currentCommand.cartCorrection.b = 0.;
		currentCommand.cartCorrection.c = 0.;
		break;
	}
	
	currentCommand.driveMode = driveMode;

	// create the xml string in the outgoing buffer
	this->fillOutgoingBuffer();

	//static int s = 0;
	//if (s++%30==0)
	//	context.write("next=%s: cor={%.8lf, %.8lf, %.8lf, %.8lf, %.8lf, %.8lf}, err={%.8lf, %.8lf, %.8lf, %.8lf, %.8lf, %.8lf}\n",
	//		bSendNext ? "Y" : "N",
	//		(next.cpos[joint + 0] - prev.cpos[joint + 0])/0.012,
	//		(next.cpos[joint + 1] - prev.cpos[joint + 1])/0.012,
	//		(next.cpos[joint + 2] - prev.cpos[joint + 2])/0.012,
	//		(next.cpos[joint + 3] - prev.cpos[joint + 3])/0.012,
	//		(next.cpos[joint + 4] - prev.cpos[joint + 4])/0.012,
	//		(next.cpos[joint + 5] - prev.cpos[joint + 5])/0.012,
	//		(prev.cpos[joint + 0] - (*state).cpos[joint + 0])/0.012,
	//		(prev.cpos[joint + 1] - (*state).cpos[joint + 1])/0.012,
	//		(prev.cpos[joint + 2] - (*state).cpos[joint + 2])/0.012,
	//		(prev.cpos[joint + 3] - (*state).cpos[joint + 3])/0.012,
	//		(prev.cpos[joint + 4] - (*state).cpos[joint + 4])/0.012,
	//		(prev.cpos[joint + 5] - (*state).cpos[joint + 5])/0.012
	//	);

	// send the xml to the robot
	if (::send(clientfd, outgoingDataBuffer.data(), (int)strlen(outgoingDataBuffer.data()), 0) < 0)
		throw Message(Message::LEVEL_ERROR, "KukaKR5SixxRSI::sysSend(): send error: %s", strerror(errno));
	// TODO: handle errors
}

void KukaKR5SixxRSI::setMessage(const char *msg) {
	strcpy(currentCommand.message,msg);
}

bool KukaKR5SixxRSI::parseIncomingState( char *xml ) {
	// For keeping track of a 5 long history of incomings...
	headState+=1;
	if (headState > 4)
		headState = 0;

	char *position;

	currentState[headState].actualCartPosition.x = KukaKR5SixxRSI::getTagVal<double>(xml, "<RIst X=\"",  "\"",  &atof, &position);
	currentState[headState].actualCartPosition.y = KukaKR5SixxRSI::getTagVal<double>(position, "Y=\"",  "\"",  &atof, &position);
	currentState[headState].actualCartPosition.z = KukaKR5SixxRSI::getTagVal<double>(position, "Z=\"",  "\"",  &atof, &position);
	currentState[headState].actualCartPosition.a = KukaKR5SixxRSI::getTagVal<double>(position, "A=\"",  "\"",  &atof, &position);
	currentState[headState].actualCartPosition.b = KukaKR5SixxRSI::getTagVal<double>(position, "B=\"",  "\"",  &atof, &position);
	currentState[headState].actualCartPosition.c = KukaKR5SixxRSI::getTagVal<double>(position, "C=\"",  "\"",  &atof, &position);

	currentState[headState].setpointCartPosition.x = KukaKR5SixxRSI::getTagVal<double>(position, "<RSol X=\"",  "\"",  &atof, &position);
	currentState[headState].setpointCartPosition.y = KukaKR5SixxRSI::getTagVal<double>(position, "Y=\"",  "\"",  &atof, &position);
	currentState[headState].setpointCartPosition.z = KukaKR5SixxRSI::getTagVal<double>(position, "Z=\"",  "\"",  &atof, &position);
	currentState[headState].setpointCartPosition.a = KukaKR5SixxRSI::getTagVal<double>(position, "A=\"",  "\"",  &atof, &position);
	currentState[headState].setpointCartPosition.b = KukaKR5SixxRSI::getTagVal<double>(position, "B=\"",  "\"",  &atof, &position);
	currentState[headState].setpointCartPosition.c = KukaKR5SixxRSI::getTagVal<double>(position, "C=\"",  "\"",  &atof, &position);

	currentState[headState].actualAxisPosition.a1 = KukaKR5SixxRSI::getTagVal<double>(position, "<AIPos A1=\"",  "\"",  &atof, &position);
	currentState[headState].actualAxisPosition.a2 = KukaKR5SixxRSI::getTagVal<double>(position, "A2=\"",  "\"",  &atof, &position);
	currentState[headState].actualAxisPosition.a3 = KukaKR5SixxRSI::getTagVal<double>(position, "A3=\"",  "\"",  &atof, &position);
	currentState[headState].actualAxisPosition.a4 = KukaKR5SixxRSI::getTagVal<double>(position, "A4=\"",  "\"",  &atof, &position);
	currentState[headState].actualAxisPosition.a5 = KukaKR5SixxRSI::getTagVal<double>(position, "A5=\"",  "\"",  &atof, &position);
	currentState[headState].actualAxisPosition.a6 = KukaKR5SixxRSI::getTagVal<double>(position, "A6=\"",  "\"",  &atof, &position);

	currentState[headState].setpointAxisPosition.a1 = KukaKR5SixxRSI::getTagVal<double>(position, "<ASPos A1=\"",  "\"",  &atof, &position);
	currentState[headState].setpointAxisPosition.a2 = KukaKR5SixxRSI::getTagVal<double>(position, "A2=\"",  "\"",  &atof, &position);
	currentState[headState].setpointAxisPosition.a3 = KukaKR5SixxRSI::getTagVal<double>(position, "A3=\"",  "\"",  &atof, &position);
	currentState[headState].setpointAxisPosition.a4 = KukaKR5SixxRSI::getTagVal<double>(position, "A4=\"",  "\"",  &atof, &position);
	currentState[headState].setpointAxisPosition.a5 = KukaKR5SixxRSI::getTagVal<double>(position, "A5=\"",  "\"",  &atof, &position);
	currentState[headState].setpointAxisPosition.a6 = KukaKR5SixxRSI::getTagVal<double>(position, "A6=\"",  "\"",  &atof, &position);

	currentState[headState].motorCurrents.a1 = KukaKR5SixxRSI::getTagVal<double>(position, "<MACur A1=\"",  "\"",  &atof, &position);
	currentState[headState].motorCurrents.a2 = KukaKR5SixxRSI::getTagVal<double>(position, "A2=\"",  "\"",  &atof, &position);
	currentState[headState].motorCurrents.a3 = KukaKR5SixxRSI::getTagVal<double>(position, "A3=\"",  "\"",  &atof, &position);
	currentState[headState].motorCurrents.a4 = KukaKR5SixxRSI::getTagVal<double>(position, "A4=\"",  "\"",  &atof, &position);
	currentState[headState].motorCurrents.a5 = KukaKR5SixxRSI::getTagVal<double>(position, "A5=\"",  "\"",  &atof, &position);
	currentState[headState].motorCurrents.a6 = KukaKR5SixxRSI::getTagVal<double>(position, "A6=\"",  "\"",  &atof, &position);

#ifdef WIN32
	currentState[headState].delayedPackets = (int)KukaKR5SixxRSI::getTagVal<I64>(xml, "<Delay D=\"",  "\" />",  &_atoi64, &position);
	currentState[headState].ipoc = KukaKR5SixxRSI::getTagVal<I64>(xml, "<IPOC>",  "</IPOC>",  &_atoi64, &position);
#else
#ifdef LINUX32
	currentState[headState].delayedPackets = KukaKR5SixxRSI::getTagVal<I64>(xml, "<Delay D=\"",  "\" />",  &atoll, &position);
	currentState[headState].ipoc = KukaKR5SixxRSI::getTagVal<I64>(xml, "<IPOC>",  "</IPOC>",  &atoll, &position);
#else
    currentState[headState].delayedPackets = KukaKR5SixxRSI::getTagVal<long long int>(xml, "<Delay D=\"",  "\" />",  &atoll, &position);
    currentState[headState].ipoc = KukaKR5SixxRSI::getTagVal<long long int>(xml, "<IPOC>",  "</IPOC>",  &atoll, &position);
#endif
#endif

	return true;
}

const char* KukaKR5SixxRSI::outgoingXMLFormat =
"<Sen Type=\"KukaAlive\"> \n\
<EStr>%s</EStr> \n\
<RKorr X=\"%g\" Y=\"%g\" Z=\"%g\" A=\"%g\" B=\"%g\" C=\"%g\" /> \n\
<AKorr A1=\"%g\" A2=\"%g\" A3=\"%g\" A4=\"%g\" A5=\"%g\" A6=\"%g\" /> \n\
<IPOC>%d</IPOC> \n\
</Sen> \n\
";

bool KukaKR5SixxRSI::fillOutgoingBuffer() {

	// if safety problem
	//    zero it all.

	switch (currentCommand.driveMode) {
	case JOINT_CORRECTION:
		sprintf(outgoingDataBuffer.data(), outgoingXMLFormat,
				currentCommand.message,
				0.0, 0.0, 0.0,  0.0, 0.0, 0.0, // zero cartesian corrections...
				currentCommand.axisCorrection.a1,
				currentCommand.axisCorrection.a2,
				currentCommand.axisCorrection.a3,
				currentCommand.axisCorrection.a4,
				currentCommand.axisCorrection.a5,
				currentCommand.axisCorrection.a6,
				currentState[headState].ipoc
				);
		break;
	case CARTESIAN_CORRECTION:
		sprintf(outgoingDataBuffer.data(), outgoingXMLFormat,
				currentCommand.message,
				currentCommand.cartCorrection.x,
				currentCommand.cartCorrection.y,
				currentCommand.cartCorrection.z,
				currentCommand.cartCorrection.a,
				currentCommand.cartCorrection.b,
				currentCommand.cartCorrection.c,
				0.0,0.0,0.0,0.0,0.0,0.0, // zero axis corrections...
				currentState[headState].ipoc
				);
		break;
	}

	return true;

}


//------------------------------------------------------------------------------
