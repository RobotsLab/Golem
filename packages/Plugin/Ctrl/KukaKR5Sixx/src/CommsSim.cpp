/*
 * CommsSim.cpp
 *
 *  Created on: 5 Dec 2011
 *      Author: burbrcjc
 *
 *  Quick and dirty Kuka "simulator" connects to the device driver and pretends to be the real Kuka
 *  arm. Sends information every 12ms, receives corrections, etc.
 *
 *  Does not do asynchronous comms, so not like Kuka...
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <exception>
#include <stdarg.h>
#ifdef WIN32
#include <winsock2.h>
#include <time.h>
#else
#include <netdb.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>
#endif

enum {
	max_length = 1024
};

const char *outgoingXML =
"<Rob> \n\
<RIst X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\" /> \n\
<RSol X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\" /> \n\
<AIPos A1=\"%g\" A2=\"%g\" A3=\"%g\" A4=\"%g\" A5=\"%g\" A6=\"%g\" /> \n\
<ASPos A1=\"0.0\" A2=\"0.0\" A3=\"0.0\" A4=\"0.0\" A5=\"0.0\" A6=\"0.0\" /> \n\
<MACur A1=\"0.0\" A2=\"0.0\" A3=\"0.0\" A4=\"0.0\" A5=\"0.0\" A6=\"0.0\" /> \n\
<Delay D=\"%d\" /> \n\
<IPOC>%d</IPOC> \n\
</Rob>\n\r";

struct OutgoingData {
	struct {
		double x,y,z,a,b,c;
	} actualCartPosition, setpointCartPosition ;
	struct {
		double a1,a2,a3,a4,a5,a6;
	} actualAxisPosition, setpointAxisPosition, motorCurrents;
	int delayedPackets; // How many packets have we missed/been late sending - more than 9 is a stop condition....
	long long int ipoc;

};

enum driveMode_t {CART_VEL,JOINT_VEL,JOINT_POS,STOPPED};

struct IncomingData {
	char message[300];
	struct {
		double a1,a2,a3,a4,a5,a6;
	} axisCorrection;

	// The ipoc cylce that we aim for this command to be completed or stopped by
	long long int ipoc_response;
};

template < class RT > static RT getTagVal( char *xml, const char *tagname, const char *closetag, RT (*atov)(const char*), char **pos) {
	const int tagLength = (int)strlen(tagname);   // could be hard coded...
	char *start = strstr(xml,tagname); // TODO: could start the search nearer to position for more efficiency
	if (start == NULL){
		*pos = NULL;
		return 0;
	}
	char *end = strstr(start+tagLength,closetag);
	if (end == NULL){
		*pos = NULL;
		return 0;
	}
	*end='\0';
	RT ret = atov(start+tagLength);
	*end=closetag[0];
	*pos = end+strlen(closetag);
	return ret;
}

bool parseIncomingCommand(  char *xml, IncomingData& incoming ) {
	char *position;


	incoming.axisCorrection.a1 = getTagVal<double>(xml, "<AKorr A1=\"",  "\"",  &atof, &position);
	incoming.axisCorrection.a2 = getTagVal<double>(position, "A2=\"",  "\"",  &atof, &position);
	incoming.axisCorrection.a3 = getTagVal<double>(position, "A3=\"",  "\"",  &atof, &position);
	incoming.axisCorrection.a4 = getTagVal<double>(position, "A4=\"",  "\"",  &atof, &position);
	incoming.axisCorrection.a5 = getTagVal<double>(position, "A5=\"",  "\"",  &atof, &position);
	incoming.axisCorrection.a6 = getTagVal<double>(position, "A6=\"",  "\"",  &atof, &position);


#ifdef WIN32
	incoming.ipoc_response = getTagVal<__int64>(xml, "<IPOC>",  "</IPOC>",  &_atoi64, &position);
#else
	incoming.ipoc_response = getTagVal<long long int>(xml, "<IPOC>",  "</IPOC>",  &atoll, &position);
#endif

	char *start = strstr(xml,"<EStr>");
	char *end = strstr(start+6,"</EStr>");
	*end = '\0';
	strcpy(incoming.message, start+6);

	return true;
}

double frand(double min, double max) {
	return min + (max - min)*double(::rand())/RAND_MAX;
}

class str_exception : public std::exception {  
public:
	str_exception(const char* format, ...) {
		va_list argptr;
		va_start(argptr, format);
#ifdef WIN32
		#pragma warning (push)
		#pragma warning (disable:4996)
		#define vsnprintf _vsnprintf
#endif
		::vsnprintf(buf, BUFSIZ - 1, format, argptr);
#ifdef WIN32
		#undef vsnprintf
		#pragma warning (pop)
#endif
		va_end(argptr);
	}
	virtual const char* what() const throw() {
		return buf;
	}

private:
	char buf[BUFSIZ];
};

int main(int argc, char* argv[]) {
	int sockfd = -1;
	FILE *file = NULL;

	try {
		char* name = argv[0] + strlen(argv[0]);
		while (name > argv[0] && *(name - 1) != '/' && *(name - 1) != '\\') name--;

		if (argc < 3)
			throw str_exception("Usage: %s <host> <port> [log_file]\nWhere <host> is the PC running the Kuka driver and <port> is usually 6008\n", name);

#ifdef WIN32
		::WSADATA wsaData;
		if (::WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
			throw str_exception("%s: unable to initialise sockets: %d", name, WSAGetLastError());
#endif // WIN32

		sockfd = (int)::socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0)
			throw str_exception("%s: unable to create socket: %s", name, strerror(errno));

#ifdef WIN32
#pragma warning (push)
#pragma warning (disable:4996)
#endif
		::hostent *server = ::gethostbyname(argv[1]);
#ifdef WIN32
#pragma warning (pop)
#endif
		if (server == NULL)
			throw str_exception("%s: unable to find server %s: %s", name, argv[1], strerror(errno));

		::sockaddr_in serveraddr;
		::memset(&serveraddr, 0, sizeof(serveraddr));
		serveraddr.sin_family = AF_INET;
		::memcpy(&serveraddr.sin_addr.s_addr, server->h_addr, server->h_length);
		const unsigned short port = (unsigned short)std::atoi(argv[2]);
		serveraddr.sin_port = htons(port);

		if (::connect(sockfd, (sockaddr*)&serveraddr, sizeof(serveraddr)) < 0)
			throw str_exception("%s: unable to connect to %s:%d", name, argv[1], port);

		char outgoingBuffer[10240];
		OutgoingData robotState;
		robotState.ipoc = 0;

		char incomingBuffer[10240];
		size_t incomingDataPos = 0;
		IncomingData incomingCommand;
		char message[300];

		using namespace std;
#ifndef WIN32
		timeval start_time;
		timeval now_time;
		timeval passed;
#endif

		const double Delta = 1e-5;
		::srand((unsigned)::time(NULL));

		if (argc > 3) {
			file = ::fopen(argv[3],"w");
			if (file == NULL)
				throw str_exception("%s: unable to open %s", name, argv[3]);
		}

		while (true) {
			robotState.ipoc++;
			// Send state
			const int outLength = sprintf(outgoingBuffer, outgoingXML,
					robotState.actualAxisPosition.a1,
					robotState.actualAxisPosition.a2,
					robotState.actualAxisPosition.a3,
					robotState.actualAxisPosition.a4,
					robotState.actualAxisPosition.a5,
					robotState.actualAxisPosition.a6,
					0,
					robotState.ipoc	);
			if (::send(sockfd, outgoingBuffer, outLength, 0) < 0)
				throw str_exception("%s: send error: %s", name, strerror(errno));

			char *location;
			for (;;) {
				const int len = (int)::recv(sockfd, incomingBuffer + incomingDataPos, 10240 - (int)incomingDataPos - 1, 0);
				if (len < 0 && errno == 0)
					throw str_exception("%s: connaction closed", name);
				if (len < 0)
					throw str_exception("%s: recv error: %s", name, strerror(errno));

				incomingDataPos += len;
				incomingBuffer[incomingDataPos+1] = '\0'; // stop the string search at the end
				location = strstr((char*)&(incomingBuffer[0]), "</Sen>");
				if (location)
					break;

				if (len == 0)
					throw str_exception("%s: connection closed unexpectedly", name);
			}

			if (location + 8 - incomingBuffer == incomingDataPos) {
				//	safe to return to buffer beginning....
				incomingDataPos = 0;
			}
			parseIncomingCommand(incomingBuffer,incomingCommand);

			if (file)
				fprintf(file,"%f\t%f\t%f\t%f\t%f\t%f\n",incomingCommand.axisCorrection.a1,
						incomingCommand.axisCorrection.a2,
						incomingCommand.axisCorrection.a3,
						incomingCommand.axisCorrection.a4,
						incomingCommand.axisCorrection.a5,
						incomingCommand.axisCorrection.a6 );

			// Apply whatever correction is to the state...no proper simulation....
			robotState.actualAxisPosition.a1 += incomingCommand.axisCorrection.a1*(1. + frand(-Delta, +Delta));
			robotState.actualAxisPosition.a2 += incomingCommand.axisCorrection.a2*(1. + frand(-Delta, +Delta));
			robotState.actualAxisPosition.a3 += incomingCommand.axisCorrection.a3*(1. + frand(-Delta, +Delta));
			robotState.actualAxisPosition.a4 += incomingCommand.axisCorrection.a4*(1. + frand(-Delta, +Delta));
			robotState.actualAxisPosition.a5 += incomingCommand.axisCorrection.a5*(1. + frand(-Delta, +Delta));
			robotState.actualAxisPosition.a6 += incomingCommand.axisCorrection.a6*(1. + frand(-Delta, +Delta));

			// If there is a message display it
			if (strcmp(message, incomingCommand.message) != 0 ) {
				strcpy(message, incomingCommand.message);
				std::cout <<incomingCommand.ipoc_response <<": " << incomingCommand.message << endl;
			}

			if (robotState.ipoc % 1000 == 0)
				cout <<incomingCommand.ipoc_response  << endl;

#ifdef WIN32
			::Sleep(12);
#else
			usleep(12000);
#endif
		}
	} catch (std::exception& ex) {
		std::cerr << ex.what() << "\n";
	}

	if (file)
		::fclose(file);

#ifdef WIN32
	if (sockfd >= 0)
		::closesocket(sockfd);
	WSACleanup();
#else // WIN32
	if (sockfd >= 0)
		::close(sockfd);
#endif // WIN32

	return 0;
}
