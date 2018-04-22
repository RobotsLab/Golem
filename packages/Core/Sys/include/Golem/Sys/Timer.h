/** @file Timer.h
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_SYS_TIMER_H_
#define _GOLEM_SYS_TIMER_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Types.h>
#include <Golem/Defs/Constants.h>

#ifdef WIN32
#include <time.h>
#include <Winsock2.h>
#else
#include <sys/time.h>
#ifdef __APPLE__
#include <mach/mach.h>
#include <mach/mach_time.h>
#endif
#endif

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Millisecond timer type (integer) */
typedef U32 MSecTmU32;
/** Time constants for millisecond timer type */
const MSecTmU32 MSEC_TM_U32_ZERO = numeric_const<MSecTmU32>::ZERO;
const MSecTmU32 MSEC_TM_U32_ONE = numeric_const<MSecTmU32>::ONE;
const MSecTmU32 MSEC_TM_U32_INF = numeric_const<MSecTmU32>::MAX;
const MSecTmU32 MSEC_TM_U32_MIN = numeric_const<MSecTmU32>::MIN;
const MSecTmU32 MSEC_TM_U32_MAX = numeric_const<MSecTmU32>::MAX;

/** Second timer type (floating point) */
typedef F64 SecTmReal;
/** Time constants for second timer type */
const SecTmReal SEC_TM_REAL_ZERO = numeric_const<SecTmReal>::ZERO;
const SecTmReal SEC_TM_REAL_ONE = numeric_const<SecTmReal>::ONE;
const SecTmReal SEC_TM_REAL_INF = numeric_const<SecTmReal>::MAX;
const SecTmReal SEC_TM_REAL_MIN = numeric_const<SecTmReal>::MIN;
const SecTmReal SEC_TM_REAL_MAX = numeric_const<SecTmReal>::MAX;

/** Time constant for timeval */
const timeval TIMEVAL_ZERO = {(long)numeric_const<I32>::ZERO, (long)numeric_const<I32>::ZERO};
const timeval TIMEVAL_MAX = {(long)numeric_const<I32>::MAX, (long)1000000};

#ifndef WIN32
/** Time constant for timespec */
const timespec TIMESPEC_ZERO = {(time_t)numeric_const<I32>::ZERO, (long)numeric_const<I32>::ZERO};
const timespec TIMESPEC_MAX = {(time_t)numeric_const<I32>::MAX, (long)1000000};
    
#ifdef __APPLE__
//clock_gettime is not implemented on OSX
#define CLOCK_REALTIME ((clockid_t)0)
int clock_gettime(int clk_id, struct timespec* t);
#endif // APPLE
#endif	// UNIX

//------------------------------------------------------------------------------

#ifdef WIN32
/** Windows implementation of timezone */
struct timezone {
	/** minutes W of Greenwich */
	int tz_minuteswest;
	/** type of dst correction */
	int tz_dsttime;
};

/** Windows implementation of gettimeofday() */
int gettimeofday(timeval *tv, timezone *tz);
#endif // Windows

/** computes difference between two timeval */
timeval timevaldiff(const timeval& tv0, const timeval& tv1);

/** Normalizes timeval */
void timevalnorm(timeval& tv);

#ifndef WIN32
/** Normalizes timespec */
void timespecnorm(timespec& ts);
#endif	// Unix

//------------------------------------------------------------------------------

/** Conversion SecTmReal to MSecTmU32 */
MSecTmU32 SecToMSec(SecTmReal t);
/** Conversion MSecTmU32 to SecTmReal */
SecTmReal MSecToSec(MSecTmU32 t);
/** Conversion MSecTmU32 to timeval */
MSecTmU32 TimevalToMSec(const timeval& tv);
/** Conversion timeval to MSecTmU32 */
timeval MSecToTimeval(MSecTmU32 t);
#ifndef WIN32
/** Conversion MSecTmU32 to timespec */
MSecTmU32 TimespecToMSec(const timespec& ts);
/** Conversion timespec to MSecTmU32 */
timespec MSecToTimespec(MSecTmU32 t);
#endif	// Unix

//------------------------------------------------------------------------------

/** Sleep object.
 */
class Sleep {
private:
#ifdef WIN32
	HANDLE hTimer;
#else
#endif

public:
	/** Default constructor initialises sleep object.
	*/
	Sleep();

	~Sleep();

	/** Suspends the calling thread for <code>interval</code> seconds.
	 * 
	 * @param interval	time in seconds
	 */
	void sleep(SecTmReal interval) const;

	/** Suspends the calling thread for <code>interval</code> milliseconds.
	 * 
	 * @param interval	time in miliseconds
	 */
	static void msleep(MSecTmU32 interval);
};

//------------------------------------------------------------------------------

/** Standard system timer with millisecond accuracy.
 */
class SysTimer {
private:
#ifdef WIN32
	U32 stamp;
#else
	timespec stamp;
#endif

public:
	/** Default constructor initialises and resets the timer.
	*/
	SysTimer();

	~SysTimer();

	/** Resets the timer.
	 */
	void reset();

	/** Calculates the time which elapsed since the last reset of the timer.
	 * 
	 * @return	elapsed time in milliseconds
	 */
	MSecTmU32 elapsed() const;
};

//------------------------------------------------------------------------------

/** Floating-point precision timer.
 * 
 * Timer with microsecond accuracy.
 */
class PerfTimer {
private:
#ifdef WIN32
	LARGE_INTEGER sysFreq, perfStamp;
#else
	timespec stamp;
#endif
	
public:
	/** Default constructor initialises and resets the timer.
	*/
	PerfTimer();

	~PerfTimer();
	
	/** Resets the timer.
	 */
	void reset();

	/** Calculates the time which elapsed since the last reset of the timer.
	 * 
	 * @return	elapsed time in seconds
	 */
	SecTmReal elapsed() const;
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_SYS_TIMER_H_*/
