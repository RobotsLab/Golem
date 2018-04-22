/** @file SMServer.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/SM/SMHelper.h>
#include <stdio.h>
//#include <conio.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

int main(int argc, char* argv[]) {
	if (argc < 2) {
		std::cerr << "Usage: SMServer <port> [packet_size] [packets_per_sec] [packets_num]" << std::endl;
		return 1;
	}

	const unsigned short port = (unsigned short)atoi(argv[1]);
	const unsigned capacity = 10000000;
	const unsigned size = argc >= 3 ? std::min(capacity, (unsigned)atoi(argv[2])) : 1000;
	const double ppsec = argc >= 4 ? std::max(0.01, atof(argv[3])) : 100;
	const unsigned packets = argc >= 5 ? (unsigned)atoi(argv[4]) : (unsigned)-1;

	try {
		SMTimer timer;
		SM::MessageStream msgstr;
		SMServer server(port, timer, &msgstr);

		std::vector<char> data(capacity);
		for (unsigned j = 0; j < size; ++j)
			data[j] = (char)0;

		//unsigned j = 0;

		for (unsigned i = 0; i < packets; ++i) {
			timer.sleep(1.0/ppsec);
			
			//if (j > 1) --j; else if (j == 1) {
			//	printf("RESET\n");
			//	*(double*)data.data() = 0;
			//	j = 0;
			//}
			//if (kbhit()) {
			//	(void)getchar(); // clear
			//	printf("SET\n");
			//	*(double*)data.data() = 100.0;
			//	j = 100;
			//}

			server.write(size, &data.front());
		}
	}
	catch (const std::exception& ex) {
		printf("%s\n", ex.what());
		return 1;
	}

	return 0;
}