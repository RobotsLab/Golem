#if defined(WIN32) && !defined(__WIN32__)
#define __WIN32__
#endif
#if defined(LINUX) && !defined(__LINUX__)
#define __LINUX__
#endif

#ifdef __WIN32__
#ifndef WINVER
#define WINVER 0x0600 // since WinXp
#endif
#undef UNICODE
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <process.h>

#define SHUT_RDWR SD_BOTH
#ifndef __func__
#define __func__ __FUNCTION__
#endif
#endif

#ifdef __LINUX__
#include <sys/select.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <unistd.h>
#include <string.h>
#define _snprintf snprintf
#endif

int resolve_hostname(const char* hostname, struct sockaddr_in* sa);
void debug(const char* format, ...);
