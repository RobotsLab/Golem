/** @file Message.cpp
 * 
 * Messages for multithread systems.
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
#include <Golem/Sys/Message.h>
#include <stdio.h>
#include <algorithm>
#include <cstring>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

char* golem::vsnprintf(char* begin, const char* end, const char* format, va_list argptr) {
	if (begin >= end)
		return begin;

	const size_t size = end - begin - 1;
	
	#ifdef WIN32
		#pragma warning (push)
		#pragma warning (disable:4996)
		#define vsnprintf _vsnprintf
	#endif
	
	const int inc = ::vsnprintf(begin, size, format, argptr);

	#ifdef WIN32
		#undef vsnprintf
		#pragma warning (pop)
	#endif
	
	if (inc < 0)
		begin += size;
	else if (size > size_t(inc))
		return begin + inc; // with NULL character
	else
		begin += inc;
	
	*begin = '\0'; // always terminate with NULL character

	return begin;
}

char* golem::snprintf(char* begin, const char* end, const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	begin = vsnprintf(begin, end, format, argptr);
	va_end(argptr);
	return begin;
}

//------------------------------------------------------------------------------

const SecTmReal Message::TIME_UNDEF = SEC_TM_REAL_INF;

const char *const Message::LEVEL_STR[] = {
	"UNDEF",
	"VERBOSE",
	"DEBUG",
	"INFO",
	"NOTICE",
	"WARNING",
	"ERROR",
	"CRIT",
};

const PerfTimer* Message::_timer = NULL;

//------------------------------------------------------------------------------

Message::Message() {
	clear();
}

Message::~Message() throw () {
}

Message::Message(const Message& message) {
	*this = message;
}

Message::Message(const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	create(TIME_UNDEF, THREAD_UNDEF, LEVEL_UNDEF, CODE_UNDEF, format, argptr);
	va_end(argptr);
}

Message::Message(Level level, const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	create(getCurrentTime(), getCurrentThread(), level, CODE_UNDEF, format, argptr);
	va_end(argptr);
}

Message::Message(Level level, Code code, const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	create(getCurrentTime(), getCurrentThread(), level, code, format, argptr);
	va_end(argptr);
}

//------------------------------------------------------------------------------

Message& Message::operator = (const Message& message) {
	_time = message._time;
	_thread = message._thread;
	_level = message._level;
	_code = message._code;
	_msg = message._msg;
	std::strncpy(_str, message._str, sizeof(_str) - 1);
	return *this;
}

void  Message::setTimer(const PerfTimer* timer) {
	if (timer != NULL) _timer = timer;
}

SecTmReal Message::getCurrentTime() {
	return _timer ? _timer->elapsed() : TIME_UNDEF;
}

U32 Message::getCurrentThread() {
	return Thread::getCurrentThreadId();
}

//------------------------------------------------------------------------------

void Message::create(SecTmReal time, U32 thread, Level level, Code code, const char* format, va_list argptr) {
	char *begin = _str, *const end = _str + sizeof(_str);

	_time = time;
	_thread = thread;
	_level = level;
	_code = code;
	
	if (_time != TIME_UNDEF || _thread != THREAD_UNDEF || _level != LEVEL_UNDEF || _code != CODE_UNDEF) {
		if (begin < end)
			*begin++ = '[';
		
		if (_time != TIME_UNDEF)
			begin = golem::snprintf(begin, end, "time=%.6f, ", _time);
		
		if (_thread != THREAD_UNDEF)
			begin = golem::snprintf(begin, end, "thread=%u", _thread);

		if (_level != LEVEL_UNDEF)
			begin = golem::snprintf(begin, end, ", level=%s", LEVEL_STR[_level]);

		if (_code != CODE_UNDEF)
			begin = golem::snprintf(begin, end, ", code=%u", _code);

		if (begin < end)
			*begin++ = ']';
		
		if (begin < end)
			*begin++ = ' ';
	}
	_msg = begin;
	begin = golem::vsnprintf(begin, end, format, argptr);
}

//------------------------------------------------------------------------------

void Message::clear() {
	_time = TIME_UNDEF;
	_thread = THREAD_UNDEF;
	_level = LEVEL_UNDEF;
	_code = CODE_UNDEF;
	_msg = _str;
	_str[0] = '\0';
}

bool Message::empty() const {
	return _str[0] == '\0';
}

//------------------------------------------------------------------------------
