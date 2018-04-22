/** @file FTServerTest.cpp
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
#include <vector>
#include <string>
#include <stdlib.h>
#include <stdio.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

class FTSensor {
	std::string calibrationFile;

public:
	/** Real type */
	typedef double Type;
	/** Vector type */
	typedef std::vector<Type> Vec;
	/** Vector size: 3 force + 3 torque */
	static const size_t SIZE = 6;

	/** Sensor initialisation */
	FTSensor(const std::string& calibrationFile) : calibrationFile(calibrationFile) {
		// TODO
	}
	/** Read F/T values */
	void read(Vec& data) {
		data.assign(SIZE, (Type)0.);
		// TODO
	}
};

//------------------------------------------------------------------------------

int main(int argc, char* argv[]) {
	if (argc < 3) {
		fprintf(stderr, "Usage: GolemFTServerTest <port> <calibration_file>");
		return 1;
	}

	try {
		// initialise sensor
		FTSensor sensor(argv[2]);
		FTSensor::Vec ft;

		// launch server, default port=26873 (FTClient)
		const unsigned short port = (unsigned short)std::strtol(argv[1], nullptr, 10);
		SMTimer timer;
		SM::MessageStream msgstr;
		golem::SMServer server(port, timer, &msgstr);

		// continously read buffer and update server state
		for (;;) {
			// block until new sensor values arrive
			sensor.read(ft);

			// update server
			server.write((unsigned)ft.size()*sizeof(FTSensor::Type), ft.data());
		}
	}
	catch (const std::exception& ex) {
		fprintf(stderr, "%s\n", ex.what());
		return 1;
	}

	return 0;
}
