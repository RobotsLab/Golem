/** @file Defs.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sys/Defs.h>
#include <string.h>
#include <stdarg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void Assert::valid(bool expression, const char* format, ...) {
	if (!expression) {
		Message message;
		va_list argptr;
		va_start(argptr, format);
		message.create(Message::getCurrentTime(), Message::getCurrentThread(), Message::LEVEL_CRIT, Message::CODE_UNDEF, format, argptr);
		va_end(argptr);
		throw message;
	}
}

void Assert::valid(bool expression, const Context& context, const char* format, ...) {
	if (!expression) {
		const char* infobuf[BUFSIZ];
		size_t i = 0;
		for (const Context* ptr = &context; ptr != nullptr && i < BUFSIZ; ptr = ptr->context, ++i)
			infobuf[i] = ptr->info;
		char buf[BUFSIZ], *ptr = buf, *end = buf + BUFSIZ - 1;
		while (i-- > 0)
			for (const char* str = infobuf[i]; ptr < end && str && *str != 0; ++ptr, ++str)
				*ptr = *str;
		strncpy(ptr, format, end - ptr);
		*end = 0;

		Message message;
		va_list argptr;
		va_start(argptr, format);
		message.create(Message::getCurrentTime(), Message::getCurrentThread(), Message::LEVEL_CRIT, Message::CODE_UNDEF, buf, argptr);
		va_end(argptr);
		throw message;
	}
}

//------------------------------------------------------------------------------

