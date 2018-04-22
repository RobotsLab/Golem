/** @file KukaRSI.cpp
*
* @author	Marek Kopicki
*
* @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
*
* @license  This file copy is licensed to you under the terms described in
*           the License.txt file included in this distribution.
*
*/

#include <Golem/Ctrl/KukaRSI/KukaRSI.h>
#include <Golem/Sys/LoadObjectDesc.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Ctrl/SingleCtrl/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc) {
	loadObjectDesc<golem::KukaRSI::Desc, Controller::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Controller::Desc::Ptr*)pControllerDesc);
}

//------------------------------------------------------------------------------

KukaRSI::KukaRSI(Context& context) : SingleCtrl(context), sendSocket(-1), recvSocket(-1) {
}

KukaRSI::~KukaRSI() {
	SingleCtrl::release();

#ifdef WIN32
	if (sendSocket >= 0)
		::closesocket(sendSocket);
	if (recvSocket >= 0)
		::closesocket(recvSocket);
	WSACleanup();
#else // WIN32
	if (sendSocket >= 0)
		::close(sendSocket);
	if (recvSocket >= 0)
		::close(recvSocket);
#endif // WIN32
}

void KukaRSI::create(const Desc& desc) {
#ifdef WIN32
	::WSADATA wsaData;
	if (::WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
		throw Message(Message::LEVEL_ERROR, "KukaRSI::create(): unable to initialise sockets: %s", strerror(errno));
#endif // WIN32

	driveMode = JOINT_CORRECTION;

	this->desc = desc;

	SingleCtrl::create(desc); // throws
}

//------------------------------------------------------------------------------

void KukaRSI::userCreate() {
	state.reset(new State(createState()));
	setToDefault(*state);
	context.write("KukaRSI::userCreate(): Initialisation...\n");

	currentCommand.message = desc.message;
	currentCommand.type = desc.type;

	joint = getStateInfo().getJoints().begin();

	// XML input buffer Default size 10K
	inBuf.resize(10240);
	// Formatted XML output buffer
	outBuf.resize(10240);

	// Create a receiver socket to receive datagrams
	recvSocket = (int)::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (recvSocket < 0)
		throw Message(Message::LEVEL_ERROR, "KukaRSI::userCreate(): unable to create socket recvSocket: %s", strerror(errno));

	// Bind the socket to any address and the specified port
	memset(&serverAddr, 0, sizeof(serverAddr));
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(desc.port);
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (::bind(recvSocket, reinterpret_cast<sockaddr*>(&serverAddr), sizeof(serverAddr)) != 0)
		throw Message(Message::LEVEL_ERROR, "KukaRSI::userCreate(): Failed to bind recvSocket: %s", strerror(errno));

	// Create a socket for sending datagrams
	sendSocket = (int)::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sendSocket < 0)
		throw Message(Message::LEVEL_ERROR, "KukaRSI::userCreate(): unable to create socket sendSocket: %s", strerror(errno));

	memset(&clientAddr, 0, sizeof(clientAddr));

	this->sysSync();

	context.write("KukaRSI::userCreate(): Initialisation finished\n");
}

void KukaRSI::sysSync() {
	// Call the recvfrom function to receive datagrams on the bound socket.
	socklen_t clientAddrSize = (int)sizeof(clientAddr);
	if (::recvfrom(recvSocket, inBuf.data(), (int)inBuf.size(), 0, reinterpret_cast<sockaddr*>(&clientAddr), &clientAddrSize) < 0)
		throw Message(Message::LEVEL_ERROR, "KukaRSI::sysSync(): recvfrom Failed: %s", strerror(errno));

	//Parse the received buffer
	this->parseIncomingState(inBuf.data());

	// Assign to this->state
	for (U32 i = 0; i < NUM_JOINTS; ++i)
		this->state->cpos[joint + i] = Math::degToRad(currentState.actualAxisPosition.a[i]);


	// TODO find out the actuall received state delay - here we assume it is equal zero
	this->state->t = context.getTimer().elapsed();
}

void KukaRSI::sysRecv(State& state) {
	// just copy
	state = *this->state;
}

void KukaRSI::sysSend(const State& prev, const State& next, bool bSendPrev, bool bSendNext) {
	//switch (driveMode) {
	//case JOINT_CORRECTION:
		for (U32 i = 0; i < NUM_JOINTS; ++i)
			currentCommand.axisCorrection.a[i] = Math::radToDeg(
				next.cpos[joint + i] - prev.cpos[joint + i] + desc.gain[i] * (prev.cpos[joint + i] - state->cpos[joint + i])
			);
	//	break;
	//case CARTESIAN_CORRECTION:
	//	currentCommand.cartCorrection.x = 0.;
	//	currentCommand.cartCorrection.y = 0.;
	//	currentCommand.cartCorrection.z = 0.;
	//	currentCommand.cartCorrection.a = 0.;
	//	currentCommand.cartCorrection.b = 0.;
	//	currentCommand.cartCorrection.c = 0.;
	//	break;
	//}

	//currentCommand.driveMode = driveMode;

	// create the xml string in the outgoing buffer
	this->fillOutgoingBuffer();

	// TEST
	//static int k = 0;
	//if (k++ % 50 == 0) {
	//	context.write("%s\n", outBuf.data());
	//	context.write("prev=(%f, %f, %f, %f, %f, %f), next=(%f, %f, %f, %f, %f, %f), state=(%f, %f, %f, %f, %f, %f), correction=(%f, %f, %f, %f, %f, %f)\n",
	//		prev.cpos[joint + 0], prev.cpos[joint + 1], prev.cpos[joint + 2], prev.cpos[joint + 3], prev.cpos[joint + 4], prev.cpos[joint + 5],
	//		next.cpos[joint + 0], next.cpos[joint + 1], next.cpos[joint + 2], next.cpos[joint + 3], next.cpos[joint + 4], next.cpos[joint + 5],
	//		state->cpos[joint + 0], state->cpos[joint + 1], state->cpos[joint + 2], state->cpos[joint + 3], state->cpos[joint + 4], state->cpos[joint + 5],
	//		currentCommand.axisCorrection.a1,
	//		currentCommand.axisCorrection.a2,
	//		currentCommand.axisCorrection.a3,
	//		currentCommand.axisCorrection.a4,
	//		currentCommand.axisCorrection.a5,
	//		currentCommand.axisCorrection.a6
	//	);
	//}

	if (::sendto(sendSocket, outBuf.data(), int(strlen(outBuf.data())), 0, reinterpret_cast<sockaddr*>(&clientAddr), sizeof(clientAddr)) < 0)
		throw Message(Message::LEVEL_ERROR, "KukaRSI::sysSend(): sendto Failed: %s", strerror(errno));
}

bool KukaRSI::parseIncomingState(char *xml) {
	char *position;

	currentState.actualCartPosition.x = KukaRSI::getTagVal<double>(xml, "<Rob Type=\"KUKA\"><RIst X=\"", "\"", &atof, &position);
	currentState.actualCartPosition.y = KukaRSI::getTagVal<double>(position, "Y=\"", "\"", &atof, &position);
	currentState.actualCartPosition.z = KukaRSI::getTagVal<double>(position, "Z=\"", "\"", &atof, &position);
	currentState.actualCartPosition.a = KukaRSI::getTagVal<double>(position, "A=\"", "\"", &atof, &position);
	currentState.actualCartPosition.b = KukaRSI::getTagVal<double>(position, "B=\"", "\"", &atof, &position);
	currentState.actualCartPosition.c = KukaRSI::getTagVal<double>(position, "C=\"", "\"", &atof, &position);

	currentState.actualAxisPosition.a1 = KukaRSI::getTagVal<double>(xml, "<AIPos A1=\"", "\"", &atof, &position);
	currentState.actualAxisPosition.a2 = KukaRSI::getTagVal<double>(position, "A2=\"", "\"", &atof, &position);
	currentState.actualAxisPosition.a3 = KukaRSI::getTagVal<double>(position, "A3=\"", "\"", &atof, &position);
	currentState.actualAxisPosition.a4 = KukaRSI::getTagVal<double>(position, "A4=\"", "\"", &atof, &position);
	currentState.actualAxisPosition.a5 = KukaRSI::getTagVal<double>(position, "A5=\"", "\"", &atof, &position);
	currentState.actualAxisPosition.a6 = KukaRSI::getTagVal<double>(position, "A6=\"", "\"", &atof, &position);

#ifdef WIN32
	currentState.ipoc = KukaRSI::getTagVal<I64>(xml, "<IPOC>", "</IPOC>", &_atoi64, &position);
#else
#ifdef LINUX32
	currentState.ipoc = KukaRSI::getTagVal<I64>(xml, "<IPOC>", "</IPOC>", &atoll, &position);
#else
	currentState.ipoc = KukaRSI::getTagVal<long long int>(xml, "<IPOC>", "</IPOC>", &atoll, &position);
#endif
#endif

	return true;
}

const char* KukaRSI::outgoingXMLFormat =
//"<Sen Type=\"%s\"> \n\
//<EStr>%s</EStr> \n\
//<RKorr X=\"%g\" Y=\"%g\" Z=\"%g\" A=\"%g\" B=\"%g\" C=\"%g\" /> \n\
//<AKorr A1=\"%g\" A2=\"%g\" A3=\"%g\" A4=\"%g\" A5=\"%g\" A6=\"%g\" /> \n\
//<IPOC>%d</IPOC> \n\
//</Sen> \n\
//";
"<Sen Type=\"%s\"> \n\
<EStr>%s</EStr> \n\
<AK A1=\"%g\" A2=\"%g\" A3=\"%g\" A4=\"%g\" A5=\"%g\" A6=\"%g\" /> \n\
<IPOC>%d</IPOC> \n\
</Sen> \n\
";

bool KukaRSI::fillOutgoingBuffer() {
	//switch (currentCommand.driveMode) {
	//case JOINT_CORRECTION:
		sprintf(outBuf.data(), outgoingXMLFormat,
			currentCommand.type.data(),
			currentCommand.message.data(),
			//0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // zero cartesian corrections...
			currentCommand.axisCorrection.a1,
			currentCommand.axisCorrection.a2,
			currentCommand.axisCorrection.a3,
			currentCommand.axisCorrection.a4,
			currentCommand.axisCorrection.a5,
			currentCommand.axisCorrection.a6,
			currentState.ipoc
		);
	//	break;
	//case CARTESIAN_CORRECTION:
	//	sprintf(outBuf.data(), outgoingXMLFormat,
	//		currentCommand.type.data(),
	//		currentCommand.message.data(),
	//		currentCommand.cartCorrection.x,
	//		currentCommand.cartCorrection.y,
	//		currentCommand.cartCorrection.z,
	//		currentCommand.cartCorrection.a,
	//		currentCommand.cartCorrection.b,
	//		currentCommand.cartCorrection.c,
	//		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // zero axis corrections...
	//		currentState.ipoc
	//	);
	//	break;
	//}

	return true;
}

//------------------------------------------------------------------------------

void golem::XMLData(Joint::Desc::Ptr &val, XMLContext* context, bool create) {
	val.reset(new Joint::Desc());
	XMLData((Joint::Desc&)*val, context, create);
}

void golem::XMLData(Chain::Desc::Ptr &val, XMLContext* context, bool create) {
	val.reset(new Chain::Desc());
	XMLData((Chain::Desc&)*val, context, create);

	// TODO number of joints is determined by URDF file
	// create default joints
	XMLData(val->joints, golem::Configspace::DIM, context, "joint", create);
}

void golem::XMLData(KukaRSI::Desc &val, XMLContext* context, bool create) {
	XMLData((SingleCtrl::Desc&)val, context, create);

	// TODO number of chains is determined by URDF file
	// create default chains
	XMLData(val.chains, golem::Chainspace::DIM, context, "chain", create);

	XMLData("port", val.port, context->getContextFirst("rsi"), create);
	XMLData("message", val.message, context->getContextFirst("rsi"), create);
	XMLData("type", val.type, context->getContextFirst("rsi"), create);

	val.gain.clear();
	golem::XMLDataSeq(val.gain, "k", context->getContextFirst("rsi gain"), false, golem::REAL_ZERO);
}

//------------------------------------------------------------------------------
