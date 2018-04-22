#pragma once
#ifndef _TYPES_H_
#define _TYPES_H_


// ------------------------------------------------------------------------
//
#define OC_VXWORKS_PPC 1
#define OC_VXWORKS_INTEL 2
#define OC_QNX_PPC 11
#define OC_QNX_INTEL 12
#define OC_LINUX_PPC 21
#define OC_LINUX_INTEL 22
#define OC_WIN32_INTEL 32

#if  (LINUX)
#define OSCPU_TYPE OC_LINUX_INTEL
#endif

#if (__QNX__)
#define OSCPU_TYPE OC_QNX_INTEL
#endif

#if (__VxWorks__)
#define OSCPU_TYPE OC_VXWORKS_INTEL
#endif

#if ( _WIN32 )
#define OSCPU_TYPE OC_WIN32_INTEL
#include <winsock2.h>
#include <windows.h>
#endif

#include <cstddef>

// ------------------------------------------------------------------------
// types with defined number of bytes on any plattform !
//
#if (OSCPU_TYPE == OC_VXWORKS_INTEL) || (OSCPU_TYPE == OC_VXWORKS_PPC) || \
 (OSCPU_TYPE == OC_QNX_INTEL) || (OSCPU_TYPE == OC_LINUX_INTEL) || (OSCPU_TYPE == OC_WIN32_INTEL)

typedef unsigned char UINT8;
typedef signed char INT8;
typedef unsigned short UINT16;
typedef signed short INT16;
typedef unsigned int UINT32;
typedef signed int INT32;

// -----------------------------------------------------------------
//
extern void  hton_packet32(void *hpacket, void *npacket, int size);
extern void  ntoh_packet32(void *hpacket, void *npacket, int size);
#else
#error "OS not supported!"


#endif



#endif

