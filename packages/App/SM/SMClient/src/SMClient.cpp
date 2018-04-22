/** @file SMClient.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#ifdef _GOLEM_BUILD_APP_SM_STANDALONE
#include <Golem/SM/SM.h>
#include <time.h>
#ifdef WIN32
#include <Mmsystem.h>
#else
#include <sys/time.h>
#endif

#else // _GOLEM_BUILD_APP_SM_STANDALONE
#include <Golem/SM/SMHelper.h>

#endif // _GOLEM_BUILD_APP_SM_STANDALONE

#include <stdio.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

#ifdef _GOLEM_BUILD_APP_SM_STANDALONE
#ifndef WIN32
void timespecnorm(timespec& ts) {
	if (ts.tv_nsec >= 1000000000L) {
		++ts.tv_sec;
		ts.tv_nsec -= 1000000000L;
	}
	if (ts.tv_nsec < 0) {
		--ts.tv_sec;
		ts.tv_nsec += 1000000000L;
	}
}

#ifdef __CYGWIN__
#define CLOCK_ID CLOCK_REALTIME
#else	// Cygwin
#define CLOCK_ID CLOCK_REALTIME//CLOCK_PROCESS_CPUTIME_ID//CLOCK_HIGHRES
#endif	// Cygwin
#endif	// Unix

/** Custom message stream */
class SMTimer : public SM::Timer {
public:
	SMTimer() {
#ifdef WIN32
		(void)::QueryPerformanceFrequency(&sysFreq);
		(void)::QueryPerformanceCounter(&perfStamp);
#else
		::clock_gettime(CLOCK_ID, &stamp);
#endif
	}

	virtual double elapsed() const {
#ifdef WIN32
		LARGE_INTEGER t;
		(void)::QueryPerformanceCounter(&t);
		return (double)(t.QuadPart - perfStamp.QuadPart) / sysFreq.QuadPart;
#else
		struct timespec ts;
		::clock_gettime(CLOCK_ID, &ts);
		ts.tv_sec -= stamp.tv_sec;
		ts.tv_nsec -= stamp.tv_nsec;
		timespecnorm(ts);
		return (double)ts.tv_sec + 1.0e-9*ts.tv_nsec;
#endif
	}
	virtual void sleep(double sec) const {
		if (sec > 0.) {
#ifdef WIN32
			::Sleep(unsigned(1000. * sec));
#else
			struct timespec t;
			t.tv_sec = (time_t)(sec);
			t.tv_nsec = (time_t)(1000000000L * (sec - t.tv_sec));
			::nanosleep(&t, NULL/*no interruptions is assumed*/);
#endif
		}
	}

private:
#ifdef WIN32
	LARGE_INTEGER sysFreq, perfStamp;
#else
	timespec stamp;
#endif
};

#endif // _GOLEM_BUILD_APP_SM_STANDALONE


int main(int argc, char* argv[]) {
	if (argc < 3) {
		fprintf(stderr, "Usage: SMClient <host> <port> [packets_num]");
		return 1;
	}

	// single FT sensor
	//const size_t capacity = 6*sizeof(double);
	const size_t capacity = 10000000;

	const std::string host = argv[1];
	const unsigned short port = (unsigned short)atoi(argv[2]);
	const unsigned packets = argc > 3 ? (unsigned)atoi(argv[3]) : 1000;

	try {
		SMTimer timer;
		SM::MessageStream msgstr;
		SMClient client(host, port, timer, &msgstr);

		client.syncTimers();
		
		client.start();

		// single FT sensor
		//std::vector<double> data(6);
		std::vector<char> data(capacity);

		unsigned id = 0;
		for (unsigned i = 0; i < packets; ++i) {
			SM::Header header(capacity, id);
			if (!client.read(header, &data.front(), boost::posix_time::seconds(1)))
				continue;

			if (i > 0 && id < header.id - 1)
				fprintf(stderr, "%i data packages have been lost\n", (header.id - id - 1));
			id = header.id;

			// comment out for real application
			printf("size=%u, id=%i, time=%f, delay=%f\n", header.size, header.id, header.time, (client.timer.elapsed() - header.time));
		}

		client.stop();
	}
	catch (const std::exception& ex) {
		fprintf(stderr, "%s\n", ex.what());
		return 1;
	}

	return 0;
}