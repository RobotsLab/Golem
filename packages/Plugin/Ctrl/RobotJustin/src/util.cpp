#include <Golem/Ctrl/RobotJustin/util.h>

#include <stdio.h>
#include <sstream>
#include <stdarg.h>


void debug(const char* format, ...) {
	va_list ap;
	va_start(ap, format);
	vprintf(format, ap);
	fflush(stdout);
	va_end(ap);
}

int resolve_hostname(const char* hostname, struct sockaddr_in* sa) {
	// resolve hostname
	if(!hostname || hostname[0] == 0)
		return -1;
#ifdef __WIN32__
	struct addrinfo *result = NULL;

	int ret = getaddrinfo(hostname, NULL, NULL, &result);
	if(ret) {
		// error!
		debug("resolve_hostname: gai_strerror: %s\n", gai_strerror(ret));
		return -1;
	}

	struct addrinfo *ptr = NULL;
	int found = 0;
    for(ptr = result; ptr != NULL; ptr = ptr->ai_next) {
        if (ptr->ai_family == AF_INET) {
			//memcpy(&sa->sin_addr, ptr->ai_addr, sizeof(sa->sin_addr));
			memcpy(&sa->sin_addr, &((struct sockaddr_in*)ptr->ai_addr)->sin_addr, sizeof(sa->sin_addr));
			sa->sin_family = AF_INET;
			found = 1;
			break;
		}
    }
    freeaddrinfo(result);
	if(!found)
		return -1;
#else
	struct hostent* he = gethostbyname(hostname);
	if(!he)
		return -1;
	if(he->h_addrtype != AF_INET)
		return -1;
	memcpy(&sa->sin_addr, he->h_addr_list[0], sizeof(sa->sin_addr));
	sa->sin_family = AF_INET;
#endif
	return 0;
}
