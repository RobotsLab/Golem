#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdarg.h>

#include <Golem/Ctrl/RobotJustin/string_util.h>
#include <Golem/Ctrl/RobotJustin/my_exceptions.h>

errno_exception::errno_exception(const char* format, ...) {
	va_list ap;
	va_start(ap, format);
	vsnprintf(msg, 1024, format, ap);
	va_end(ap);
	string stlmsg(msg);
#ifdef __WIN32__
	{ 
		LPVOID lpMsgBuf;
		DWORD dw = GetLastError(); 
		
		FormatMessage(
			FORMAT_MESSAGE_ALLOCATE_BUFFER | 
			FORMAT_MESSAGE_FROM_SYSTEM |
			FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL,
			dw,
			MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
			(LPTSTR) &lpMsgBuf,
			0, NULL );

		stlmsg = format_string(
			"%s\nGetLastError(): %d\nFormatMessage(): %s\nerrno",
			stlmsg.c_str(),
			dw,
			lpMsgBuf);

		LocalFree(lpMsgBuf);
	}
#endif
    if(stlmsg.size())
		_snprintf(this->msg, 512, "%s: %s (errno %d)", stlmsg.c_str(), strerror(errno), errno);
    else
		_snprintf(this->msg, 512, "%s (errno %d)", strerror(errno), errno);
}

errno_exception::errno_exception(int retval, const char* format, ...) {
	va_list ap;
	va_start(ap, format);
	vsnprintf(msg, 1024, format, ap);
	va_end(ap);
	string stlmsg(msg);
    if(stlmsg.size())
		_snprintf(this->msg, 512, "%s: %s (retval %d)", stlmsg.c_str(), strerror(retval), retval);
    else
		_snprintf(this->msg, 512, "%s (retval %d)", strerror(retval), retval);
}

str_exception::str_exception(const char* format, ...) {
	va_list ap;
	va_start(ap, format);
	vsnprintf(msg, 1024 * 10, format, ap);
	va_end(ap);
}
