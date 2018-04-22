
#include <Golem/Ctrl/DLRHitHandII/Types.h>
#if ((OSCPU_TYPE == OC_QNX_INTEL) || (OSCPU_TYPE == OC_LINUX_INTEL))

#include <netinet/in.h>
#endif
#if (OSCPU_TYPE == OC_WIN32_INTEL)
#include <winsock2.h>
#else
#include <arpa/inet.h>
#endif
#include <assert.h>

// ----------------------------------------------------------------
// convert a given packet to network byte order; it is assumed, that the packet consists
// of 32-bit entries only !
//
void  hton_packet32(void *hpacket, void *npacket, int size) {

  int i;
  
  assert( (size % 4) == 0);
  
  for(i = 0; i < (size/4); i++) {
    *((UINT32 *)npacket) = htonl(*((UINT32 *)hpacket));
    npacket = (void *)(((UINT32 *)npacket) + 1);
    hpacket = (void *)(((UINT32 *)hpacket) + 1);
  }
}


// ----------------------------------------------------------------
// convert a given packet to host byte order; it is assumed, that the packet consists
// of 32-bit entries only !
//
void  ntoh_packet32(void *npacket, void *hpacket, int size) {

  int i;

  assert( (size % 4) == 0);
  

  for(i = 0; i < (size/4); i++) {
    *((UINT32 *)hpacket) = ntohl(*((UINT32 *)npacket));
    hpacket = (void *)(((UINT32 *)hpacket) + 1);
	npacket  = (void *)(((UINT32 *)npacket) + 1);
  }
}


