/** @file Debug.h
 * 
 * Debugging tools for multithreaded real-time systems.
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
#ifndef _GOLEM_TOOLS_DEBUG_H_
#define _GOLEM_TOOLS_DEBUG_H_

#include <Golem/Sys/Context.h>
#include <map>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Thread debugging */
class ThreadWatch : protected Runnable {
protected:
	typedef std::map<U32, SecTmReal> Points;
	typedef std::map<U32, Points> Threads;
	Threads threads;
	
	CriticalSection cs;
	Thread thread;
	MSecTmU32 exitTimeOut;
	Event evStop;

	Context* pContext;
	
	SecTmReal threadTimeOut;

	virtual void run() {
		do {
			if (pContext == NULL)
				break;

			bool bOK = true;

			const SecTmReal time = pContext->getTimer().elapsed();

			for (Threads::iterator t = threads.begin(); t != threads.end(); ++t) {
				Points::iterator lastest = t->second.begin();

				for (Points::iterator p = t->second.begin(); p != t->second.end(); ++p) {
					if (lastest->second < p->second)
						lastest = p;
				}

				if (lastest->second < time - threadTimeOut) {
					bOK = false;
					pContext->debug("ThreadWatch: thread: %u, point: %d, time out: %f\n",
						t->first, lastest->first, (time - lastest->second)
					);
				}
			}

			//if (bOK)
			//	pContext->debug("ThreadWatch: OK");

		} while (!evStop.wait(MSecTmU32(1000*threadTimeOut)));
	}

public:
	ThreadWatch(SecTmReal threadTimeOut = SecTmReal(10.0), MSecTmU32 exitTimeOut = 5000) : threadTimeOut(threadTimeOut), exitTimeOut(exitTimeOut), pContext(NULL) {
	}
	virtual ~ThreadWatch() {
		evStop.set(true);
		thread.join(exitTimeOut);
	}
	void launch(Context& context) {
		if (thread.isAlive())
			return;
		pContext = &context;
		thread.start(this);
	}
	void point(U32 index) {
		if (pContext == NULL)
			return;

		CriticalSectionWrapper csw(cs);
		const U32 thread = Thread::getCurrentThreadId();
		const SecTmReal time = pContext->getTimer().elapsed();
		threads[thread][index] = time;
	}
	SecTmReal getThreadTimeOut() const {
		return threadTimeOut;
	}
	void setThreadTimeOut(SecTmReal threadTimeOut) {
		CriticalSectionWrapper csw(cs);
		this->threadTimeOut = threadTimeOut;
	}
};

extern ThreadWatch threadWatch;

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_TOOLS_DEBUG_H_*/
