/** @file SMHelper.h
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
#ifndef _GOLEM_SM_SMHELPER_SMHELPER_H_
#define _GOLEM_SM_SMHELPER_SMHELPER_H_

//------------------------------------------------------------------------------

#include <Golem/Sys/MessageStream.h>
#include <Golem/SM/SM.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Custom message stream */
class SMTimer : public SM::Timer {
public:
	SMTimer(const golem::PerfTimer* timer = NULL) {
		if (timer)
			this->timer = timer;
		else {
			pTimer.reset(new golem::PerfTimer);
			this->timer = pTimer.get();
		}
	}
	virtual double elapsed() const {
		return (double)timer->elapsed();
	}
	virtual void sleep(double sec) const {
		Sleep::msleep(SecToMSec((SecTmReal)sec));
	}

private:
	boost::shared_ptr<golem::PerfTimer> pTimer;
	const golem::PerfTimer* timer;
};

/** Custom message stream */
class SMMessageStream : public SM::MessageStream {
public:
	SMMessageStream(golem::MessageStream& messageStream) : messageStream(messageStream) {
	}
	virtual void debug(const char* format, va_list argptr) const {
		messageStream.write(golem::Message::getCurrentTime(), golem::Message::getCurrentThread(), golem::Message::LEVEL_DEBUG, golem::Message::CODE_UNDEF, format, argptr);
	}
	virtual void info(const char* format, va_list argptr) const {
		messageStream.write(golem::Message::getCurrentTime(), golem::Message::getCurrentThread(), golem::Message::LEVEL_INFO, golem::Message::CODE_UNDEF, format, argptr);
	}
	virtual void error(const char* format, va_list argptr) const {
		messageStream.write(golem::Message::getCurrentTime(), golem::Message::getCurrentThread(), golem::Message::LEVEL_ERROR, golem::Message::CODE_UNDEF, format, argptr);
	}
	
private:
	golem::MessageStream& messageStream;
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_SM_SMHELPER_SMHELPER_H_*/
