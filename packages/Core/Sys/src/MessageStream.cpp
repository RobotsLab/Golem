/** @file MessageStream.cpp
*
* Message stream.
*
* @author	Marek Kopicki
*
* @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
*
* @license  This file copy is licensed to you under the terms described in
*           the License.txt file included in this distribution.
*
*/

#include <Golem/Sys/MessageStream.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

StreamMessageCallback::StreamMessageCallback(const Desc& desc, std::ostream& ostr) : desc(desc), ostr(ostr) {
#if defined(WIN32) && defined(ENABLE_VIRTUAL_TERMINAL_PROCESSING)
	if (desc.enableFormat) {
		const HANDLE hConsole = ::GetStdHandle(STD_OUTPUT_HANDLE);

		DWORD consoleMode;
		if (!::GetConsoleMode(hConsole, &consoleMode))
			throw MsgMessageStream(Message::LEVEL_CRIT, "StreamMessageCallback(): GetConsoleMode(): unable to obtain console mode: error=%u", ::GetLastError());

		consoleMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
		if (!::SetConsoleMode(hConsole, consoleMode))
			throw MsgMessageStream(Message::LEVEL_CRIT, "StreamMessageCallback(): SetConsoleMode(): unable to set console mode: error=%u", ::GetLastError());
	}
#endif
}

void StreamMessageCallback::write(const Message& message) {
#if defined(ENABLE_MESSAGE_CSTREAM)
	if (desc.enableFormat)
		fprintf(desc.file, "%s%s%s", desc.escSeqLevel[message.level()].c_str(), message.what(), desc.escSeqReset.c_str());
	else
		fprintf(desc.file, "%s", message.what());
#else
	if (desc.enableFormat)
		std::cout << desc.escSeqLevel[message.level()] << message.what() << desc.escSeqReset << std::flush;
	else
		std::cout << message.what() << std::flush;
#endif
	//ostr << desc.escSeqLevel[message.level()] << message.what() << desc.escSeqReset;
}

//------------------------------------------------------------------------------

MessageStream::MessageStream() {
}

void MessageStream::create(const Desc &desc) {
	if (!desc.isValid())
		throw MsgMessageStreamInvalidDesc(Message::LEVEL_CRIT, "MessageStream::create(): Invalid description");

	filter = desc.messageFilter;
	callback = desc.messageCallback;
}

void MessageStream::write(SecTmReal time, U32 thread, Message::Level level, Message::Code code, const char* format, va_list argptr) {
	if (filter == NULL || (*filter)(time, thread, level, code)) {
		Message message;
		message.create(time, thread, level, code, format, argptr);
		write(message);
	}
}

void MessageStream::write(const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	write(Message::TIME_UNDEF, Message::THREAD_UNDEF, Message::LEVEL_UNDEF, Message::CODE_UNDEF, format, argptr);		
	va_end(argptr);
}

void MessageStream::write(Message::Level level, const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	write(Message::getCurrentTime(), Message::getCurrentThread(), level, Message::CODE_UNDEF, format, argptr);		
	va_end(argptr);
}

void MessageStream::write(Message::Level level, Message::Code code, const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	write(Message::getCurrentTime(), Message::getCurrentThread(), level, code, format, argptr);
	va_end(argptr);
}

void MessageStream::write(SecTmReal time, U32 thread, Message::Level level, Message::Code code, const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	write(time, thread, level, code, format, argptr);
	va_end(argptr);
}

//------------------------------------------------------------------------------

BufferedStream::~BufferedStream() {
	terminate = true;
	evQueue.set(true);
	if (!thread.join(threadTimeOut)) {
		// ignore
	}
}

void BufferedStream::create(const Desc &desc) {
	MessageStream::create(desc); // throws

	messages.reserve(desc.queueSize);
	threadTimeOut = desc.threadTimeOut;
	terminate = false;

	std::ios::sync_with_stdio(false);

	if (!thread.start(this))
		throw MsgMessageStreamThreadLaunch(Message::LEVEL_CRIT, "BufferedStream::create(): Unable to launch thread");

	if (!thread.setPriority(desc.threadPriority)) {
		// ignore
	}
}

void BufferedStream::run() {
	for (;;) {
		(void)evQueue.wait();

		MessageCallback::Ptr callback;
		Message message;
		{
			CriticalSectionWrapper csw(csQueue);
			if (messages.empty()) {
				if (terminate)
					break;
				evQueue.set(false);
				continue;
			}
			message = messages.front();
			messages.pop_front();
			callback = this->callback;
		}
		if (callback != nullptr)
			callback->write(message);
	}
}

void BufferedStream::write(const Message& message) {
	CriticalSectionWrapper csw(csQueue);
	messages.push_back(message);
	evQueue.set(true);
}

//------------------------------------------------------------------------------
