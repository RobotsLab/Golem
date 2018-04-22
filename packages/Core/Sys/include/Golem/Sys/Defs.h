/** @file Defs.h
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
#ifndef _GOLEM_SYS_DEFS_H_
#define _GOLEM_SYS_DEFS_H_

//------------------------------------------------------------------------------

#include <Golem/Sys/Thread.h>
#include <Golem/Sys/Message.h>
#include <Golem/Sys/Parallels.h>
#include <functional>
#include <stdexcept>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

typedef std::function<void()> Function;

//------------------------------------------------------------------------------

/** Assert */
class Assert {
public:
	/** Context */
	class Context {
	public:
		friend class Assert;

		/** Creates linked list of messages */ 
		explicit inline Context(const char* info) : context(nullptr), info(info) {}
		/** Creates linked list of messages */ 
		explicit inline Context(const Context& context, const char* info) : context(&context), info(info) {}

	private:
		const Context *context;
		const char *info;
	};

	/** Throw an exception is expression=false */
	static void valid(bool expression, const char* format, ...);
	/** Throw an exception is expression=false */
	static void valid(bool expression, const Context& context, const char* format, ...);
};

//------------------------------------------------------------------------------

class ScopeGuard {
public:
	typedef golem::shared_ptr<ScopeGuard> Ptr;
	/** Guard function */
	typedef golem::Function Function;

	/** Set guard function */
	ScopeGuard(Function function) : function(function) {
	}
	/** Run guard function on object deletion */
	~ScopeGuard() {
		function();
	}
	/** Run guard function on demand */
	void run() {
		function();
	}

private:
	Function function;
};

//------------------------------------------------------------------------------

/** golem::Thread helper class */
class ThreadTask : protected golem::Runnable {
public:
	typedef golem::shared_ptr<ThreadTask> Ptr;
	/** Task function */
	typedef golem::Function Function;

	/** Task construction */
	ThreadTask(Function function = nullptr, golem::Thread::Priority priority = golem::Thread::NORMAL, golem::MSecTmU32 timeOut = golem::MSEC_TM_U32_INF) : timeOut(timeOut), function(function), terminating(false) {
		if (!thread.start(this))
			throw golem::Message(golem::Message::LEVEL_CRIT, "ThreadTask::ThreadTask(): Unable to start thread");
		(void)thread.setPriority(priority); // ignore retval
	}
	~ThreadTask() {
		terminating = true;
		ev.set(true);
		(void)thread.join(timeOut); // ignore retval
	}
	/** Set new thread task */
	void start(Function function = nullptr) {
		if (function)
			this->function = function; // atomic
		ev.set(true);
	}
	/** Terminate or not */
	void setTerminate(bool terminate = true) {
		terminating = true;
	}
	/** Is terminating? */
	bool isTerminating() const {
		return terminating;
	}

protected:
	/** Thread */
	golem::Thread thread;
	/** New task */
	golem::Event ev;
	/** Time to stop */
	golem::MSecTmU32 timeOut;
	/** Stop condition variable */
	bool terminating;
	/** Task function */
	Function function;

	/** Runnable: thread function */
	virtual void run() {
		for (;;) {
			if (!ev.wait(timeOut))
				continue;
			ev.set(false);
			if (terminating)
				break;
			const Function function = this->function; // atomic
			if (function)
				function();
		}
	}
};

//------------------------------------------------------------------------------

/** golem::Parallels helper class */
class ParallelsTask : protected golem::Runnable {
public:
	typedef golem::shared_ptr<ParallelsTask> Ptr;
	/** Task function */
	typedef std::function<void(ParallelsTask*)> Function;

	/** Task construction */
	ParallelsTask(golem::Parallels* parallels, Function function, golem::Thread::Priority priority = golem::Thread::LOWEST, golem::MSecTmU32 timeOut = golem::MSEC_TM_U32_INF) : function(function), stopping(false) {
		// setup Parallels
		if (parallels == nullptr)
			throw golem::Message(golem::Message::LEVEL_CRIT, "ParallelsTask::ParallelsTask(): Parallels required");
		// launch Parallels
		for (size_t t = (size_t)parallels->getNumOfThreads(); t > 0; --t) {
			golem::Job* job = parallels->startJob(this);
			if (!job)
				throw golem::Message(golem::Message::LEVEL_CRIT, "ParallelsTask::ParallelsTask(): Unable to start job");
			job->setThreadPriority(priority); // ignore retval
		}
		// wait for completion or stop now
		if (!parallels->joinJobs(timeOut)) {
			stopping = true;
			(void)parallels->joinJobs(golem::MSEC_TM_U32_INF); // ignore retval
		}
		// check for exceptions
		golem::CriticalSectionWrapper csw(cs);
		if (!message.empty())
			throw message;
	}
	/** Stop condition */
	bool stop() const {
		return stopping;
	}

protected:
	/** Stop condition variable */
	bool stopping;
	/** Task function */
	Function function;
	/** Task critical section */
	golem::CriticalSection cs;
	/** Task exception */
	golem::Message message;

	/** Runnable: parallels thread function */
	virtual void run() {
		try {
			function(this);
		}
		catch (const golem::Message& msg) {
			golem::CriticalSectionWrapper csw(cs);
			if (message.empty())
				message = msg;
		}
		catch (const std::exception& ex) {
			golem::CriticalSectionWrapper csw(cs);
			if (message.empty())
				message = golem::Message(golem::Message::LEVEL_CRIT, "ParallelsTask::run(): %s", ex.what());
		}
		catch (...) {
			golem::CriticalSectionWrapper csw(cs);
			if (message.empty())
				message = golem::Message(golem::Message::LEVEL_CRIT, "ParallelsTask::run(): unknown exception");
		}
	}
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_SYS_DEFS_H_*/
