/** @file Timer.cpp
 * 
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sys/Timer.h>

#ifdef WIN32
#include <Mmsystem.h>
#else
#include <time.h>
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

#ifdef __APPLE__
int golem::clock_gettime(int clk_id, struct timespec* t) {
    struct timeval now;
    int rv = gettimeofday(&now, NULL);
    if (rv) return rv;
    t->tv_sec  = now.tv_sec;
    t->tv_nsec = now.tv_usec * 1000;
    return 0;
}
#endif

//------------------------------------------------------------------------------

void golem::timevalnorm(timeval& tv) {
	if (tv.tv_usec >= 1000000L) {
		++tv.tv_sec;
		tv.tv_usec -= 1000000L;
	}
	if (tv.tv_usec < 0) {
		--tv.tv_sec;
		tv.tv_usec += 1000000L;
	}
}

#ifndef WIN32
void golem::timespecnorm(timespec& ts) {
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

//------------------------------------------------------------------------------

MSecTmU32 golem::SecToMSec(SecTmReal t) {
	return	t <= SEC_TM_REAL_ZERO ? MSEC_TM_U32_ZERO : t >= SecTmReal(MSEC_TM_U32_MAX/1000) ? MSEC_TM_U32_INF : MSecTmU32(1000.*t);
}

SecTmReal golem::MSecToSec(MSecTmU32 t) {
	return	t == MSEC_TM_U32_INF ? SEC_TM_REAL_INF : SecTmReal(t/1000.);
}

MSecTmU32 golem::TimevalToMSec(const timeval& tv) {
	return MSecTmU32(tv.tv_sec) > MSEC_TM_U32_MAX/1000 || MSecTmU32(tv.tv_sec)*1000 > MSEC_TM_U32_MAX - tv.tv_usec/1000 ? MSEC_TM_U32_MAX : MSecTmU32(tv.tv_sec)*1000 + tv.tv_usec/1000;
}

timeval golem::MSecToTimeval(MSecTmU32 t) {
	if (t >= MSEC_TM_U32_MAX)
		return TIMEVAL_MAX;
	const timeval tv = { long(t/1000), long((t%1000)*1000)};
	return tv;
}

#ifndef WIN32
MSecTmU32 golem::TimespecToMSec(const timespec& ts) {
	return MSecTmU32(ts.tv_sec) > MSEC_TM_U32_MAX/1000 || MSecTmU32(ts.tv_sec)*1000 > MSEC_TM_U32_MAX - ts.tv_nsec/1000000 ? MSEC_TM_U32_MAX : MSecTmU32(ts.tv_sec)*1000 + ts.tv_nsec/1000000;
}

timespec golem::MSecToTimespec(MSecTmU32 t) {
	if (t >= MSEC_TM_U32_MAX)
		return TIMESPEC_MAX;
	const timespec ts = { long(t/1000), long((t%1000)*1000000)};
	return ts;
}
#endif	// Unix

//------------------------------------------------------------------------------

Sleep::Sleep() {
#ifdef WIN32
	//::timeBeginPeriod(1);
	//hTimer = ::CreateWaitableTimer(NULL, TRUE, NULL);
#else
#endif
}

Sleep::~Sleep() {
#ifdef WIN32
	//if (hTimer != NULL) ::CloseHandle(hTimer);
	//::timeEndPeriod(1);
#else
#endif
}

void Sleep::sleep(SecTmReal interval) const {
	if (interval > SEC_TM_REAL_ZERO) {
#ifdef WIN32
		::Sleep(SecToMSec(interval));
		//if (hTimer == NULL)
		//	return;
		//LARGE_INTEGER liDueTime;
		//liDueTime.QuadPart = -LONGLONG(interval*10000000); // 100ns increments
		//if (!::SetWaitableTimer(hTimer, &liDueTime, 0, NULL, NULL, 0))
		//	return;
		//if (::WaitForSingleObject(hTimer, INFINITE) != WAIT_OBJECT_0)
		//	return;
#else
		struct timespec t;
		t.tv_sec = (time_t)interval;
		t.tv_nsec = (time_t)(1.0e9*(interval - t.tv_sec));
		::nanosleep(&t, NULL/*no interruptions is assumed*/);
#endif
	}
}

void Sleep::msleep(MSecTmU32 interval) {
	if (interval > 0) {
#ifdef WIN32
		::Sleep(interval);
#else
		struct timespec t;
		t.tv_sec = (time_t)(interval/1000L);
		t.tv_nsec = (time_t)(1000000L*(interval%1000L));
		::nanosleep(&t, NULL/*no interruptions is assumed*/);
#endif
	}
}

//------------------------------------------------------------------------------

SysTimer::SysTimer() {
#ifdef WIN32
	::timeBeginPeriod(1);
#else
#endif
	reset();
}

SysTimer::~SysTimer() {
#ifdef WIN32
	::timeEndPeriod(1);
#else
#endif
}

void SysTimer::reset() {
#ifdef WIN32
	stamp = ::timeGetTime();
#else
	::clock_gettime(CLOCK_REALTIME, &stamp);
#endif
}

MSecTmU32 SysTimer::elapsed() const {
#ifdef WIN32
	return ::timeGetTime() - stamp;
#else
	struct timespec ts;
	::clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_sec -= stamp.tv_sec;
	ts.tv_nsec -= stamp.tv_nsec;
	timespecnorm(ts);
	return TimespecToMSec(ts);
#endif
}

//------------------------------------------------------------------------------

PerfTimer::PerfTimer() {
#ifdef WIN32
#else
#endif
	reset();
}

PerfTimer::~PerfTimer() {
#ifdef WIN32
#else
#endif
}
	
void PerfTimer::reset() {
#ifdef WIN32
	(void)::QueryPerformanceFrequency(&sysFreq);
	(void)::QueryPerformanceCounter(&perfStamp);
#else
	::clock_gettime(CLOCK_ID, &stamp);
#endif
}
	
SecTmReal PerfTimer::elapsed() const {
#ifdef WIN32
	LARGE_INTEGER t;
	(void)::QueryPerformanceCounter(&t);
	return (SecTmReal)(t.QuadPart - perfStamp.QuadPart)/sysFreq.QuadPart;
#else
	struct timespec ts;
	::clock_gettime(CLOCK_ID, &ts);
	ts.tv_sec -= stamp.tv_sec;
	ts.tv_nsec -= stamp.tv_nsec;
	timespecnorm(ts);
	return (SecTmReal)ts.tv_sec + 1.0e-9*ts.tv_nsec;
#endif
}
	
//------------------------------------------------------------------------------

#ifdef WIN32
#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
#define DELTA_EPOCH_IN_MICROSECS 11644473600000000Ui64
#else
#define DELTA_EPOCH_IN_MICROSECS 11644473600000000ULL
#endif

int golem::gettimeofday(struct timeval *tv, struct timezone *tz) {
	FILETIME ft;
	unsigned __int64 tmpres = 0;
	static int tzflag = 0;

	if (tv != NULL) {
		GetSystemTimeAsFileTime(&ft);

		tmpres |= ft.dwHighDateTime;
		tmpres <<= 32;
		tmpres |= ft.dwLowDateTime;

		tmpres /= 10;// convert into microseconds
		// converting file time to unix epoch
		tmpres -= DELTA_EPOCH_IN_MICROSECS; 
		tv->tv_sec = (long)(tmpres / 1000000UL);
		tv->tv_usec = (long)(tmpres % 1000000UL);
	}

	if (tz != NULL) {
		if (!tzflag) {
			_tzset();
			++tzflag;
		}
		tz->tz_minuteswest = _timezone/60;
		tz->tz_dsttime = _daylight;
	}

	return 0;
}
#endif

timeval golem::timevaldiff(const timeval& tv0, const timeval& tv1) {
	timeval tv = tv1;

	if (tv0.tv_usec < tv.tv_usec) {
		const long usec = (tv.tv_usec - tv0.tv_usec)/1000000 + 1;
		tv.tv_usec -= 1000000*usec;
		tv.tv_sec += usec;
	}
	if (tv0.tv_usec - tv.tv_usec > 1000000) {
		const long usec = (tv0.tv_usec - tv.tv_usec)/1000000;
		tv.tv_usec += 1000000*usec;
		tv.tv_sec -= usec;
	}

	tv.tv_sec = tv0.tv_sec - tv.tv_sec;
	tv.tv_usec = tv0.tv_usec - tv.tv_usec;

	return tv;
}

//------------------------------------------------------------------------------
