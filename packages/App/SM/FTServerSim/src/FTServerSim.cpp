/** @file FTServerSim.cpp
*
* @author	Marek Kopicki
*
* @copyright  Copyright (C) 2017 Marek Kopicki, University of Birmingham, UK
*
* @license  This file copy is licensed to you under the terms described in
*           the License.txt file included in this distribution.
*
*/

#include <Golem/SM/SMHelper.h>
#include <stdexcept>
#include <vector>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#ifdef WIN32
#include <conio.h>
#pragma warning (push)
#pragma warning (disable : 4996)
#else
#include <curses.h>
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

class FTSensorSim {
	const double _lin, _ang;
	double lin, ang;
	int inc;

public:
	/** Real type */
	typedef double Type;
	/** Vector type */
	typedef std::vector<Type> Vec;
	/** Vector size: 3 force + 3 torque */
	static const size_t SIZE = 6;

	/** Sensor initialisation */
	FTSensorSim(double lin, double ang) : _lin(lin), lin(lin), _ang(ang), ang(ang), inc(0) {
#ifndef WIN32
		initscr();
		timeout(-1);
		cbreak();
		noecho();
#endif
	}
	/** Read F/T values */
	void read(Vec& data) {
		golem::Sleep::msleep(1);
#ifdef WIN32
		if (!kbhit())
			return;
#endif

		switch (getch()) {
		case '+': inc += 1; break;
		case '-': inc -= 1; break;
		case ' ': data.assign(SIZE, 0.0); break;
		case 'w': data[0] += lin; break;
		case 's': data[0] -= lin; break;
		case 'x': data[0]  = 0.0; break;
		case 'W': data[3] += ang; break;
		case 'S': data[3] -= ang; break;
		case 'X': data[3]  = 0.0; break;
		case 'e': data[1] += lin; break;
		case 'd': data[1] -= lin; break;
		case 'c': data[1]  = 0.0; break;
		case 'E': data[4] += ang; break;
		case 'D': data[4] -= ang; break;
		case 'C': data[4]  = 0.0; break;
		case 'r': data[2] += lin; break;
		case 'f': data[2] -= lin; break;
		case 'v': data[2]  = 0.0; break;
		case 'R': data[5] += ang; break;
		case 'F': data[5] -= ang; break;
		case 'V': data[5]  = 0.0; break;
		default: return;
		}

		const double s = inc < 0 ? 1.0 / std::pow(2.0, double(-inc)) : std::pow(2.0, double(+inc));
		lin = s * _lin;
		ang = s * _ang;

		fprintf(stdout, "FT_inc={(%3i), (%11.5f), (%11.5f)}, FT_val={(%11.5f, %11.5f, %11.5f), (%11.5f, %11.5f, %11.5f)}\r\n", inc, lin, ang, data[0], data[1], data[2], data[3], data[4], data[5]);
	}
};

//------------------------------------------------------------------------------

int main(int argc, char* argv[]) {
	if (argc < 2) {
		fprintf(stderr, "Usage: GolemFTServerSim <port> [<force_base> <torque_base>]\n");
		fprintf(stderr, "       <space>      to set F/T to zero\n");
		fprintf(stderr, "       <+>/<->      to increase/decrease F/T step\n");
		fprintf(stderr, "       <w>/<s>/<x>  to increase/decrease/reset X-axis force\n");
		fprintf(stderr, "       <e>/<d>/<c>  to increase/decrease/reset Y-axis force\n");
		fprintf(stderr, "       <r>/<f>/<v>  to increase/decrease/reset Z-axis force\n");
		fprintf(stderr, "       <W>/<S>/<X>  to increase/decrease/reset X-axis torque\n");
		fprintf(stderr, "       <E>/<D>/<C>  to increase/decrease/reset Y-axis torque\n");
		fprintf(stderr, "       <R>/<F>/<V>  to increase/decrease/reset Z-axis torque\n");
		return 1;
	}

	try {
		// launch server, default port=26873 (FTClient)
		const unsigned short port = (unsigned short)std::strtol(argv[1], nullptr, 10);
		char* ptr = nullptr;
		const double lin = argc < 3 ? 1.0 : std::strtod(argv[2], &ptr);
		const double ang = argc < 4 ? 1.0 : std::strtod(argv[3], &ptr);

		// initialise sensor
		FTSensorSim sensor(lin, ang);
		FTSensorSim::Vec ft(FTSensorSim::SIZE, 0.);

		SMTimer timer;
		SM::MessageStream msgstr;
		golem::SMServer server(port, timer, &msgstr);

		// continously read buffer and update server state
		for (;;) {
			// update server
			server.write((unsigned)ft.size()*sizeof(FTSensorSim::Type), ft.data());

			// block until new sensor values arrive
			sensor.read(ft);
		}
	}
	catch (const std::exception& ex) {
		fprintf(stderr, "%s\n", ex.what());
		return 1;
	}

	return 0;
}
