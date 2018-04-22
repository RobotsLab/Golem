/** @file Parallels.h
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
#ifndef _GOLEM_SYS_PARALLELS_H_
#define _GOLEM_SYS_PARALLELS_H_

#include <Golem/Defs/Pointers.h>
#include <Golem/Sys/Thread.h>
#include <Golem/Sys/Message.h>
#include <vector>
#include <set>
#include <map>

#ifdef WIN32
#else
#include <pthread.h>
#endif

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgParallels, Message)
MESSAGE_DEF(MsgParallelsThreadLaunch, MsgParallels)
//MESSAGE_DEF(MsgParallelsUnknownThread, MsgParallels)

//------------------------------------------------------------------------------

class Parallels;

class Job : private Runnable {
public:
	typedef shared_ptr<Job> Ptr;
	typedef std::vector<Ptr> Seq;
	typedef std::set<Job*> Set;
	typedef std::map<U32, Job*> Map;

	friend class Parallels;

protected:
	Parallels *pParallels;
	U32 jobId;
	MSecTmU32 threadTimeOut;
	Thread thread;
	Event evThread, evJob;
	Runnable *pRunnable;

	virtual void run();

	bool startJob(Runnable *pRunnable, MSecTmU32 timeOut = MSEC_TM_U32_INF);
	
	bool joinJob(MSecTmU32 timeOut = MSEC_TM_U32_INF);
	
	bool startThread();
	
	bool joinThread(MSecTmU32 timeOut = MSEC_TM_U32_INF);
	
	Job(Parallels *pParallels, U32 jobId, MSecTmU32 threadTimeOut);

public:
	~Job();
	
	/** Sets the new job thread priority.
	 * 
	 * @return	<code>true</code> if the thread priority was successfully 
	 * 			updated; <code>false</code> otherwise
	 */
	bool setThreadPriority(Thread::Priority priority);
	
	/** Returns the current priority of the job thread.
	 * 
	 * @return	the thread priority 
	 */
	Thread::Priority getThreadPriority() const;
	
	/** Returns an identifier of the job.
	 * 
	 * @return	4-bit value identifies the job
	 */
	U32 getJobId() const;

	/** Returns an identifier of the job thread.
	 * 
	 * @return	4-bit value identifies the job thread
	 */
	U32 getThreadId() const;
};

/** Class implementing platform-independent parallel working threads.
 */
class Parallels {
protected:
	friend class Job;

	volatile bool bStop;

	Semaphore sm;
	CriticalSection cs;
	Job::Seq jobs;
	Job::Set freeJobs;
	Job::Map jobsMap;

public:
	/** Initialises the <code>Parallels</code> class.
	 * 
	 * @param numOfThreads	number of working threads
	 * @param threadTimeOut	working thread time out
	 */
	Parallels(U32 numOfThreads, MSecTmU32 threadTimeOut = MSEC_TM_U32_INF);
	~Parallels();
	
	/** Launches threads.
	 */
	void startThreads();
	
	/** Join all jobs and terminate threads.
	 */
	bool joinThreads(MSecTmU32 timeOut = MSEC_TM_U32_INF);
	
	/** Launches a job in a thread.
	 */
	Job* startJob(Runnable *pRunnable, MSecTmU32 timeOut = MSEC_TM_U32_INF);
	
	/** Join the given job, but not the corresponding thread.
	 */
	bool joinJob(Job *pJob, MSecTmU32 timeOut = MSEC_TM_U32_INF);
	
	/** Join all jobs, but not threads.
	 */
	bool joinJobs(MSecTmU32 timeOut = MSEC_TM_U32_INF);
	
	/** Returns the current job.
	 */
	Job* getCurrentJob() const;
	
	/** Returns the number of threads.
	 */
	U32 getNumOfThreads() const;
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_SYS_PARALLELS_H_*/
