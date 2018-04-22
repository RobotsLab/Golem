/** @file Thread.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sys/Thread.h>
//#include <Golem/Sys/Message.h>

#ifdef WIN32
#else
#include <errno.h>
#include <sys/resource.h>
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Event::Event(bool bEvent) {
#ifdef WIN32
	hEvent = ::CreateEvent(0, boolean_cast<BOOL>(true)/*manual-reset*/, boolean_cast<BOOL>(bEvent), 0);
#else
	::pthread_mutex_init(&mutex, NULL);
	//::pthread_mutexattr_t attr;
	//::pthread_mutexattr_init(&attr);
	//::pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
	//::pthread_mutex_init(&mutex, &attr);
	//::pthread_mutexattr_destroy(&attr);
	
	::pthread_cond_init(&condition, NULL);
	this->bEvent = bEvent;
#endif
}

Event::~Event() {
#ifdef WIN32
	if (hEvent != INVALID_HANDLE_VALUE)
		::CloseHandle(hEvent);
#else
	::pthread_cond_destroy(&condition);
	::pthread_mutex_destroy(&mutex);
#endif
}

bool Event::set(bool bEvent) {
#ifdef WIN32
	if (hEvent == INVALID_HANDLE_VALUE)
		return false;

	// If bEvent==true sets the object (hEvent) to signaled state,
	// no matter if there are any waiting threads or not 
	// Otherwise if bEvent==false sets the object (hEvent) to non-signaled state
	return boolean_cast<bool>(bEvent ? ::SetEvent(hEvent) : ::ResetEvent(hEvent));
#else
	::pthread_mutex_lock(&mutex);
	//int err = ::pthread_mutex_lock(&mutex);
	//if (err != 0)
	//	throw Message("Event::set(): failed mutex lock: %s", strerror(err));
	
	this->bEvent = bEvent;
	// wake up at least one waiting thread (fast)
	// it has no effect if there are no waiting threads
	if (bEvent)
		::pthread_cond_signal(&condition);
	
	::pthread_mutex_unlock(&mutex);
	//err = ::pthread_mutex_unlock(&mutex);
	//if (err != 0)
	//	throw Message("Event::set(): failed mutex unlock: %s", strerror(err));

	return true;
#endif
}

bool Event::wait(MSecTmU32 timeOut) {
#ifdef WIN32
	if (hEvent == INVALID_HANDLE_VALUE)
		return false;

	return ::WaitForSingleObject(hEvent, (U32)timeOut) == WAIT_OBJECT_0;
#else
	::pthread_mutex_lock(&mutex);
	//int err = ::pthread_mutex_lock(&mutex);
	//if (err != 0)
	//	throw Message("Event::wait(): failed mutex lock: %s", strerror(err));

	// if the object is in non-signaled state wait for for start from other 
	// threads, but no longer than timeOut
	struct timespec ts;
	
	::clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_sec += timeOut/1000;
	ts.tv_nsec += (timeOut%1000)*1000000;
	timespecnorm(ts);
	
	while (!bEvent) {
		int iRet = ::pthread_cond_timedwait(&condition, &mutex, &ts);
		if (iRet > 0 && errno != EINTR)
			break;
	}

	::pthread_mutex_unlock(&mutex);
	//err = ::pthread_mutex_unlock(&mutex);
	//if (err != 0)
	//	throw Message("Event::wait(): failed mutex unlock: %s", strerror(err));

	return bEvent;
#endif
}

//------------------------------------------------------------------------------

Semaphore::Semaphore(int initCount, int maxCount) {
#ifdef WIN32
	// Create unnamed semaphore
	hSemaphore = ::CreateSemaphore(NULL, initCount, maxCount, NULL);
#else
	//::pthread_mutex_init(&mutex, NULL);
	
	::pthread_mutexattr_t attr;
	::pthread_mutexattr_init(&attr);
	::pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
	::pthread_mutex_init(&mutex, &attr);
	::pthread_mutexattr_destroy(&attr);

	::pthread_cond_init(&condition, NULL);
	//int iRet = ::pthread_cond_init(&condition, NULL);
	//if (iRet != 0)
	//	throw Message("Semaphore::Semaphore(): failed to init condition: %s", strerror(errno));
	this->count = initCount;
	this->maxCount = maxCount;
#endif
}

Semaphore::~Semaphore() {
#ifdef WIN32
	if (hSemaphore != INVALID_HANDLE_VALUE)
		::CloseHandle(hSemaphore);
#else
	::pthread_cond_destroy(&condition);
	::pthread_mutex_destroy(&mutex);
#endif
}

bool Semaphore::release(int count) {
#ifdef WIN32
	if (hSemaphore == INVALID_HANDLE_VALUE)
		return false;
		
	return boolean_cast<bool>(::ReleaseSemaphore(hSemaphore, count, NULL));
#else
	bool bRet = false;

	::pthread_mutex_lock(&mutex);
	//int err = ::pthread_mutex_lock(&mutex);
	//if (err != 0)
	//	throw Message("Semaphore::release(): failed mutex lock: %s", strerror(err));	

	if (this->count + count <= maxCount) {
		this->count += count;
		// wake up at least one waiting thread (fast)
		// it has no effect if there are no waiting threads
		::pthread_cond_signal(&condition);
		
		bRet = true;
	}
	
	::pthread_mutex_unlock(&mutex);
	//err = ::pthread_mutex_unlock(&mutex);
	//if (err != 0)
	//	throw Message("Semaphore::release(): failed mutex unlock: %s", strerror(err));	

	return bRet;
#endif
}

bool Semaphore::wait(MSecTmU32 timeOut) {
#ifdef WIN32
	if (hSemaphore == INVALID_HANDLE_VALUE)
		return false;

	return ::WaitForSingleObject(hSemaphore, (U32)timeOut) == WAIT_OBJECT_0;
#else
	bool bRet = false;
	
	// if the object is in non-signaled state wait for for start from other 
	// threads, but no longer than timeOut
	::pthread_mutex_lock(&mutex);
	//int err = ::pthread_mutex_lock(&mutex);
	//if (err != 0)
	//	throw Message("Semaphore::wait(): failed mutex lock: %s", strerror(err));

	struct timespec ts;
	
	::clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_sec += timeOut/1000;
	ts.tv_nsec += (timeOut%1000)*1000000;
	timespecnorm(ts);
	
	while (!(bRet = count > 0)) {
		int iRet = ::pthread_cond_timedwait(&condition, &mutex, &ts);
		if (iRet > 0 && errno != EINTR)
			break;
	}
	
	if (bRet)
		--count;
	
	::pthread_mutex_unlock(&mutex);
	//err = ::pthread_mutex_unlock(&mutex);
	//if (err != 0)
	//	throw Message("Semaphore::wait(): failed mutex unlock: %s", strerror(err));

	return bRet;
#endif
}

//------------------------------------------------------------------------------

CriticalSection::CriticalSection() {
#ifdef WIN32
	::InitializeCriticalSection(&cs);
#else
	//::pthread_mutex_init(&mutex, NULL);
	
	::pthread_mutexattr_t attr;
	::pthread_mutexattr_init(&attr);
	::pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	::pthread_mutex_init(&mutex, &attr);
	::pthread_mutexattr_destroy(&attr);

	//::pthread_mutexattr_t attr;
	//::pthread_mutexattr_init(&attr);
	//::pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
	//::pthread_mutex_init(&mutex, &attr);
	//::pthread_mutexattr_destroy(&attr);
#endif
}

CriticalSection::~CriticalSection() {
#ifdef WIN32
	::DeleteCriticalSection(&cs);
#else
	::pthread_mutex_destroy(&mutex);
#endif
}

void CriticalSection::lock() {
#ifdef WIN32
	::EnterCriticalSection(&cs);
#else
	(void)::pthread_mutex_lock(&mutex);
	//owner = pthread_self();
	//int err = ::pthread_mutex_lock(&mutex);
	//if (err != 0)
	//	throw Message("CriticalSection::lock(): failed mutex lock: %s, owner is %u self is %u",
	//		strerror(err), owner, pthread_self()
	//	);
#endif
}

bool CriticalSection::trylock() {
#ifdef WIN32
	if (::TryEnterCriticalSection(&cs))
		return true;
#else
	if (::pthread_mutex_trylock(&mutex) == 0)
		return true;
	//int err = ::pthread_mutex_trylock(&mutex);
	//if (err == 0)
	//	return true;
	//else
	//	throw Message("CriticalSection:trylock(): failed mutex trylock: %s", strerror(err));
#endif

	return false;
}

void CriticalSection::unlock() {
#ifdef WIN32
	::LeaveCriticalSection(&cs);
#else
	(void)::pthread_mutex_unlock(&mutex);
	//int err = ::pthread_mutex_unlock(&mutex);
	//if (err != 0)
	//	throw Message("CriticalSection::unlock(): failed mutex unlock: %s", strerror(err));
#endif
}

//------------------------------------------------------------------------------


CriticalSectionWrapper::CriticalSectionWrapper(CriticalSection& cs, bool lock) :
	cs(cs), autoLock(lock)
{
	if (autoLock) cs.lock();
}

CriticalSectionWrapper::~CriticalSectionWrapper() {
	if (autoLock) cs.unlock();
}
	
void CriticalSectionWrapper::lock() {
	cs.lock();
}

bool CriticalSectionWrapper::trylock() {
	return cs.trylock();
}

void CriticalSectionWrapper::unlock() {
	cs.unlock();
}

//------------------------------------------------------------------------------

Thread::Thread() {
	pRunnable = NULL;
#ifdef WIN32
	hThread = NULL;
#endif
	threadId = 0;
	bAlive = false;
}

Thread::~Thread() {
	cleanup();
}

THREAD_START_ROUTINE_RET Thread::threadProc(Thread* pThread) {
	pThread->pRunnable->run();
	pThread->bAlive = false;
	pThread->evExit.set(true);
#ifdef WIN32
	return 0;
#else
	return NULL;
#endif
}

bool Thread::start(Runnable *pRunnable) {
	if (pRunnable == NULL || this->pRunnable != NULL)
		return false;
	this->pRunnable = pRunnable;

#ifdef WIN32
	hThread = ::CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)threadProc, this, 0, (LPDWORD)&threadId);
	if (hThread == NULL) {
		pRunnable = NULL;
		threadId = 0;
		return false;
	}
#else
	if (::pthread_create(&threadId, NULL, (LPTHREAD_START_ROUTINE)threadProc, this) != 0) {
		pRunnable = NULL;
		threadId = 0;
		return false;
	}
#endif
	bAlive = true;
	
	return true;
}

bool Thread::join(MSecTmU32 timeOut) {
	if (pRunnable == NULL)
		return true;
	if (!evExit.wait(timeOut))
		return false;

#ifdef WIN32
	hThread = NULL;
#endif
	threadId = 0;
	return true;
}

void Thread::cleanup() {
#ifdef WIN32
	if (hThread != NULL) {
		(void)::TerminateThread(hThread, 0);
		(void)::CloseHandle(hThread);
		hThread = NULL;
	}
#else
	if (threadId != 0) {
		// TODO Thread termination - see pthread_cancel 
	}
#endif

	threadId = 0;
	bAlive = false;
}

bool Thread::isAlive() const {
	return bAlive;
}

golem::U32 Thread::getThreadId() const {
	return (U32)(size_t)threadId;
}

golem::U32 Thread::getCurrentThreadId() {
#ifdef WIN32
	return ::GetCurrentThreadId();
#else
#ifdef __APPLE__
	// covert first to size_t because it is always of the same size of a pointer.
	// Apple's compiler would throw an error if a pointer is casted into a unsigned int
	return (U32)(size_t)::pthread_self();
#else
	return (U32)::pthread_self();
#endif
#endif
}

bool Thread::setPriority(Priority priority) {
#ifdef WIN32
	return	hThread != NULL && ::SetThreadPriority(hThread, (int)priority);
#else
	if (threadId == 0)
		return false;	
#ifdef __CYGWIN__
	// setpriority/getpriority priority are not implemented in Cygwin
	return true;
#else	// Cygwin
	return setpriority(PRIO_PROCESS, 0/*current process*/, (int)priority) == 0;
#endif	// Cygwin
#endif
}

Thread::Priority Thread::getPriority() const {
#ifdef WIN32
	return hThread != NULL ? (Priority)GetThreadPriority(hThread) : ERROR_RETURN;
#else
	if (threadId == 0)
		return ERROR_RETURN;
#ifdef __CYGWIN__
	// setpriority/getpriority priority are not implemented in Cygwin
	return NORMAL;
#else	// Cygwin
	errno = 0;
	int priority = getpriority(PRIO_PROCESS, 0/*current process*/);
	return priority == -1 && errno != 0 ? ERROR_RETURN : (Priority)priority;
#endif	// Cygwin
#endif
}

//------------------------------------------------------------------------------
