/** @file Thread.h
 * 
 * Multithread computing library.
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
#ifndef _GOLEM_SYS_THREAD_H_
#define _GOLEM_SYS_THREAD_H_

#include <Golem/Sys/Timer.h>

#ifdef WIN32
#else
#include <pthread.h>
#include <limits.h>
#endif

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Class implementing platform-independent events.
 * 
 * Events provide a simple and effective way of signalising a waiting thread
 * about an event from another thread.
 */
class Event {
private:
#ifdef WIN32
	HANDLE hEvent;
#else
	pthread_mutex_t mutex;
	pthread_cond_t condition;
	bool bEvent;
#endif

public:

	/** Specifies an initial event state.
	 * 
	 * @param bEvent		<code>true</code> set/signal an event
	 * 						<code>false</code> otherwise
	 */
	Event(bool bEvent = false);
	~Event();
	
	/** Sets/signals an event.
	 * 
	 * @param bEvent		<code>true</code> set/signal an event;
	 * 						<code>false</code> otherwise
	 * @return				<code>true</code> no errors;
	 * 						<code>false</code> otherwise
	 */
	bool set(bool bEvent);
	
	/** Waits for an event but no longer than specified amount of time.
	 * 
	 * @param timeOut		waiting time in miliseconds
	 * @return				<code>true</code> if waiting for an event was
	 * 						successful; <code>false</code> otherwise
	 */
	bool wait(MSecTmU32 timeOut = MSEC_TM_U32_INF);
};

//------------------------------------------------------------------------------

/** Class implementing platform-independent semaphores.
 * 
 * Semaphores provide a simple and effective way of signalising client threads 
 * if the resources they are waiting for are available.
 */
class Semaphore {
private:
#ifdef WIN32
	HANDLE hSemaphore;
#else
	pthread_mutex_t mutex;
	pthread_cond_t condition;
	int count, maxCount;
#endif

public:
	/** Creates semaphore.
	 * 
	 * The total number of resources (counter) ready to allocate by clients
	 * 
	 * @param initCount		initial count
	 * @param maxCount		maximal count
	 */
	Semaphore(int initCount, int maxCount);
	~Semaphore();
	
	/** Releases allocated resource by specified number
	 * 
	 * @param count			count to release;
	 * 						<code>count</code> must be greater than 0
	 * @return				<code>true</code> no errors;
	 * 						<code>false</code> otherwise
	 */
	bool release(int count = 1);
	
	/** Waits for counter larger than zero and allocates an resource.
	 * 
	 * @param timeOut		waiting time in miliseconds
	 * @return				<code>true</code> if waiting for a resource was
	 * 						successful; <code>false</code> otherwise
	 */
	bool wait(MSecTmU32 timeOut = MSEC_TM_U32_INF);
};

//------------------------------------------------------------------------------

/** Class implementing platform-independent critical sections.
 * 
 * Critical section secures the critical part of a code from access from
 * other threads.
*/
class CriticalSection {
private:
#ifdef WIN32
	CRITICAL_SECTION cs;
#else
	//pthread_t owner;
	pthread_mutex_t mutex;
#endif

public:
	/** Initialises critical section
	 */
	CriticalSection();

	/** Destroys critical section
	 */
	~CriticalSection();

	/** Locks critical part of a code
	 */
	void lock();
	
	/** Try to lock critical part of a code
	 * 
	 * @return	<code>true</code> if successful;
	 * 			<code>false</code> otherwise	 
	 */
	bool trylock();
	
	/** Unlocks/frees the critical part of a code
	 */
	void unlock();
};

//------------------------------------------------------------------------------

/** Wrapper class for critical sections.
 *
 */
class CriticalSectionWrapper {
private:
	CriticalSection& cs;
	bool autoLock;
	
public:
	/** Initialises critical section wrapper
	 * 
	 * @param cs	critical section to be wrapped
	 * @param lock	auto locking
	 */
	CriticalSectionWrapper(CriticalSection& cs, bool lock = true);
	
	/** Destroys critical section wrapper
	 */
	~CriticalSectionWrapper();
	
	/** Locks the critical section
	 */
	void lock();
	
	/** Tries to lock critical section
	 * 
	 * @return	<code>true</code> if successful;
	 * 			<code>false</code> otherwise	 
	 */
	bool trylock();
	
	/** Unlocks/frees the critical section
	 */
	void unlock();
};

//------------------------------------------------------------------------------

/** Interlocked increment/decrement. 
 */
#ifdef WIN32
#else

#ifndef WINNT_LONG_FOR_LINUX
#define WINNT_LONG_FOR_LINUX
typedef long LONG;
#endif

inline long InterlockedIncrement(volatile LONG* pVal) { return __sync_add_and_fetch(pVal, 1L); }
inline long InterlockedDecrement(volatile LONG* pVal) { return __sync_sub_and_fetch(pVal, 1L); }

#endif

//------------------------------------------------------------------------------

#ifdef WIN32
	typedef unsigned long THREAD_START_ROUTINE_RET;
#else
	typedef void* THREAD_START_ROUTINE_RET;
	typedef THREAD_START_ROUTINE_RET (*LPTHREAD_START_ROUTINE)(void*);
#endif

/** Interface used by <code>Thread</code> for launching new threads. 
 */
class Runnable {
protected:
	virtual ~Runnable() {}

public:
	/** Implementation of a function of the thread. 
	 */
	virtual void run() = 0;
};

/** Class implementing platform-independent threads.
 * 
 * Provides basic control of threads.
 */
class Thread {
protected:
#ifdef WIN32
	HANDLE hThread;
	U32 threadId;
#else
	pthread_t threadId;
#endif
	Runnable *pRunnable;
	Event evExit;
	volatile bool bAlive;
	
	/** This functions calls a virtual function <code>Runnable#run()</code>.
	 */
	static THREAD_START_ROUTINE_RET threadProc(Thread* pThread);
	
	/** Close the thread and frees resources.
	 */
	inline void cleanup();

public:

	/** Specifies thread priorities.
	 * 
	 * @see	<code>Thread#setPriority(Priority)</code> and 
	 * 		<code>Thread#getPriority()</code>.
	 */
	enum Priority {
#ifdef WIN32
		TIME_CRITICAL	= THREAD_PRIORITY_TIME_CRITICAL,
		HIGHEST			= THREAD_PRIORITY_HIGHEST,
		ABOVE_NORMAL	= THREAD_PRIORITY_ABOVE_NORMAL,
		NORMAL			= THREAD_PRIORITY_NORMAL,
		BELOW_NORMAL	= THREAD_PRIORITY_BELOW_NORMAL,
		LOWEST			= THREAD_PRIORITY_LOWEST,
		IDLE			= THREAD_PRIORITY_IDLE,
		ERROR_RETURN	= THREAD_PRIORITY_ERROR_RETURN,
#else
		TIME_CRITICAL	= -NZERO,
		HIGHEST			= -15,
		ABOVE_NORMAL	= -6,
		NORMAL			= 0,
		BELOW_NORMAL	= 6,
		LOWEST			= 15,
		IDLE			= NZERO-1,
		ERROR_RETURN	= -NZERO-1,
#endif
	};
	
	/** Returns an identifier of the calling thread.
	 * 
	 * @return	4-bit value identifies the calling thread
	 */
	static U32 getCurrentThreadId();

	/** Initialises the <code>Thread</code> class.
	 * 
	 * Constructor of <code>Thread</code> class must have specified a pointer
	 * to the <code>Runnable</code> interface implementing the thread function.
	 * 
	 */
	Thread();
	~Thread();
	
	/** Launches a thread with default priority.
	 * 
	 * @param pRunnable	a pointer to the <code>Runnable</code> interface 
	 * 					implementing the thread function
	 * @return	<code>true</code> if a thread was successfully created;
	 * 			<code>false</code> otherwise
	 */
	bool start(Runnable *pRunnable);
	
	/** Sets the new thread priority.
	 * 
	 * @return	<code>true</code> if the thread priority was successfully 
	 * 			updated; <code>false</code> otherwise
	 */
	bool setPriority(Priority priority);
	
	/** Returns the current priority of the thread.
	 * 
	 * @return	the thread priority 
	 */
	Priority getPriority() const;
	
	/** Waits for the thread to complete/finish.
	 * 
	 * It is assumed that the thread is signalised to finish before calling 
	 * this function.
	 * 
	 * @param timeOut		waiting time in miliseconds
	 * @return				<code>true</code> if the thread has completed 
	 * 						successfully; <code>false</code> otherwise
	 * @see					<code>Runnable#run</code>
	 */
	bool join(MSecTmU32 timeOut = MSEC_TM_U32_INF);
	
	/** Chesks if the thread is alive.
	 * 
	 * @returns <code>true</code> if the thread is alive;
	 * 			<code>false</code> otherwise
	 */
	bool isAlive() const;
	
	/** Returns an identifier of the thread.
	 * 
	 * @return	4-bit value identifies the thread
	 */
	U32 getThreadId() const;
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_SYS_THREAD_H_*/
