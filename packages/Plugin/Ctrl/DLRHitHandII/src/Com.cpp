
#include <Golem/Ctrl/DLRHitHandII/Com.h>
#include <Golem/Ctrl/DLRHitHandII/Types.h>

#if (OSCPU_TYPE == OC_VXWORKS_PPC)
#include <sockLib.h>
#include <ioLib.h>
#include <inetLib.h>
#include <netinet/ip.h>
#include <hostLib.h>
#endif
#if (OSCPU_TYPE == OC_VXWORKS_INTEL)
#include <sockLib.h>
#include <ioLib.h>
#include <inetLib.h>
#include <netinet/ip.h>
#include <hostLib.h>
#include <strings.h>

#define socklen_t int
#endif


#include <errno.h>

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#if (OSCPU_TYPE == OC_WIN32_INTEL)
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netdb.h>
#endif
#include <string.h>
#include <fcntl.h>

#if (OSCPU_TYPE == OC_LINUX_INTEL)
#include <unistd.h>
#endif

#include <iostream>

using namespace std;
using namespace golem;

// --------------------------------------------------------------------------
//
Com::Com() : _peer_bound(0) {
}

// --------------------------------------------------------------------------
//
int Com::initrec(int udp_port, int max_packetsize) {
  int res;

  res = init(udp_port, max_packetsize);
  if(res < 0)
    return res;


  // bind the socket to the host interface 
  //
  memset(&_inLocal, 0, sizeof(_inLocal));

  _inLocal.sin_family = AF_INET;
  _inLocal.sin_addr.s_addr  =  htonl(INADDR_ANY);
  _inLocal.sin_port = htons(_port);


  if(bind(_sockFD, (struct sockaddr*) &_inLocal, sizeof(_inLocal)) < 0) {
    //perror("Socket bind error\n");
    return -1;
  }
  _peer_bound = 1;


  // Set receive buffer size of this socket to hold only one packet.
  // So we get unbuffered behaviour, which is as deterministic as we can
  // get with the a UDP/IP-stack.
  // (on QNX 6.3 systems this buffer size has to be 16 Byte larger than the packet size;
  // on Linux systems the actual buffer size can be much larger than the specified size
  // also depending on if one big packet or multiple small packets are received.)
  // BB: 11.09.2009: minimal buffer size forced to 256 Bytes, as under Linux a smaller buffer is not allowed and the result is not receiving packets any more !
  //

#if( OSCPU_TYPE == OC_WIN32_INTEL)
  int tmp = _max_packetsize + 16; // on QNX 6.3 system the actual size of the buffer is 16 Bytes smaller than specified (strange ...)
  if(setsockopt(_sockFD,SOL_SOCKET , SO_RCVBUF, (char*)&tmp, sizeof(tmp)) < 0)
	return -1;
#else
  int tmp = _max_packetsize + 16; // on QNX 6.3 system the actual size of the buffer is 16 Bytes smaller than specified (strange ...)
  if(tmp < 256)
	  tmp =256;
  if(setsockopt(_sockFD,SOL_SOCKET , SO_RCVBUF, &tmp, sizeof(tmp)) < 0)
	return -1;
#endif
  
  return 1;
}

// --------------------------------------------------------------------------
//
int Com::initsend(const char *peer_addr, int udp_port, int max_packetsize) {
  int res;

  res = init(udp_port, max_packetsize);
  if(res < 0)
    return res;

  _sendcount = 0; // number of sent packets


  // fill in address structure for the peer
  //
  memset(&_inPeer, 0, sizeof(_inPeer));   
  
  _inPeer.sin_family = AF_INET;
  
#if (OSCPU_TYPE == OC_VXWORKS_PPC) || (OSCPU_TYPE == OC_VXWORKS_INTEL)
  _inPeer.sin_addr.s_addr  = hostGetByName((char*)peer_addr);
#elif ((OSCPU_TYPE == OC_QNX_INTEL) || (OSCPU_TYPE == OC_LINUX_INTEL) || (OSCPU_TYPE == OC_WIN32_INTEL))
  struct hostent *hent;

  hent = gethostbyname(peer_addr);
  if(hent == NULL) 
    return -1;
  
  _inPeer.sin_addr.s_addr  = *(UINT32 *)(hent->h_addr);
#endif
  
  _inPeer.sin_port = htons(_port);
  _inPeerSize = sizeof(_inPeer);



  // Set send buffer size of this socket to hold only one packet.
  // So we get unbuffered behaviour, which is as deterministic as we can
  // get with the a UDP/IP-stack.
  // (on QNX 6.3 systems this buffer size is quite precisely what one sees, when sending packets;
  // on Linux systems the actual buffer size can be much larger than the specified size
  // also depending on if one big packet or multiple small packets are sent.)
  //
  int tmp = _max_packetsize; 
  if(setsockopt(_sockFD,SOL_SOCKET , SO_SNDBUF, (char*)&tmp, sizeof(tmp)) < 0)
	return -1;

  
  return 1;
}

// --------------------------------------------------------------------------
//
int Com::init(int udp_port, int max_packetsize) {

  int res = 1;

#if (OSCPU_TYPE == OC_WIN32_INTEL)
	WSADATA wsaData;

	/* Use the MAKEWORD(lowbyte, highbyte) macro declared in Windef.h */
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		 /* Tell the user that we could not find a usable Winsock DLL */
		 //printf("WSAStartup failed with error: %d\n", GetLastError());
		 return -1;
	 }
#endif
	
  _port = udp_port;

  // create a UDP socket
  //
  _sockFD = (int)socket(AF_INET, SOCK_DGRAM, 0);
  if( _sockFD < 0 ) {
    //perror("\nerror while creating a socket\n");
    return -1;
  }
  
#if (OSCPU_TYPE == OC_VXWORKS_PPC) || (OSCPU_TYPE == OC_VXWORKS_INTEL)
  // set socket options
  //
  int optval = 0;   
  setsockopt (_sockFD, IPPROTO_IP, IP_RECVDSTADDR, (char*) &optval, sizeof(optval));
  optval = IPTOS_LOWDELAY;   
  setsockopt (_sockFD, IPPROTO_IP, IP_TOS, (char*) &optval, sizeof(optval));
#endif
  
  // check, if a buffer for conversion to and from network byte order should be created
  //
  _max_packetsize = max_packetsize + sizeof(_PacketFrame);
  _npacket.resize(_max_packetsize);
  
  return res;
}

// -------------------------------------------------------------------------
// check if there is a received packet pending, remove it; 1: pending packet removed, 0: no packet pending, -1: error
//
int Com::is_pending() {

  int res;
  
  fd_set readfds;
  struct timeval tv;
  struct sockaddr_in inPeer;
  int inPeerSize;


  tv.tv_sec = 0;
  tv.tv_usec = 0;
  
  FD_ZERO(&readfds);
  FD_SET(_sockFD, &readfds);

  res = select(_sockFD+1, &readfds, NULL, NULL, &tv); // returns the number of file handles, that are ready for reading, 0 for timeout and -1 for error
  if(res == 1) {
	  if(recvfrom(_sockFD, (char *)_npacket.data(), _max_packetsize, 0, (struct sockaddr*) &inPeer, (socklen_t *)&inPeerSize) == -1)
	  res = -1;
	else
	  res  = 1;
  }
  return res;

}
// -------------------------------------------------------------------------
// receive a  packet non-blocking, 1: packet received, 0: no packet pending, -1: error
//
int Com::rec_raw(void *packet, int packetsize, unsigned timeout_usec) {
	int res = 1;
  
	if (timeout_usec != -1) {
		struct timeval tv;
		tv.tv_sec = timeout_usec/1000000;
		tv.tv_usec = timeout_usec%1000000;
		
		fd_set readfds;
		FD_ZERO(&readfds);
		FD_SET(_sockFD, &readfds);

		res = select(_sockFD+1, &readfds, NULL, NULL, &tv); // returns the number of file handles, that are ready for reading, 0 for timeout and -1 for error
	}

	if (res == 1) {
		struct sockaddr_in inPeer;
		int inPeerSize = sizeof(inPeer);
		res = recvfrom(_sockFD, (char *)packet, packetsize, 0, (struct sockaddr*) &inPeer, (socklen_t *)&inPeerSize);
		
		if (res != packetsize) {
			//printf("Com::rec_raw || invalid packet size (is %d should be %d)\n", res, packetsize);
			return -1;
		}
	}

	return res;
}
// -----------------------------------------------------------------------
// receive a packet (with frame added) of 32 bit entries  and convert it from network to host byte order
//
int Com::rec32(void *packet, int packetsize, UINT32 *packetid, UINT32 *packetstatus, UINT32 *packetnr, unsigned timeout_usec ) {
  
  int res;
  struct _PacketFrame packetframe;
  int framedpacketsize = packetsize + sizeof(_PacketFrame);
 
  assert( framedpacketsize <= _max_packetsize);

  res = rec_raw(_npacket.data(), framedpacketsize, timeout_usec);

  if(res > 0) {
	ntoh_packet32(_npacket.data(), _npacket.data(), framedpacketsize);

    memcpy(&packetframe, _npacket.data(), sizeof(_PacketFrame));
    memcpy(packet, ((char *)_npacket.data())+sizeof(_PacketFrame), packetsize);

    *packetnr = packetframe.packetnr;
    *packetid = packetframe.packetid;
    *packetstatus = packetframe.status;
  }

  return res;

}
// -------------------------------------------------------------------------
//
int Com::send_raw(void *packet, int packetsize) {
	int res = 1;

    res = sendto(_sockFD, (char *)packet, packetsize,0, (struct sockaddr*) &_inPeer, _inPeerSize);
	// if the socket send buffer is full then do not treat this as an error 
	// (our policy is, that we do not guarantee to transmit every packet !)
	//
#if (OSCPU_TYPE == OC_WIN32_INTEL)
	if((res < 0) && (errno == WSAENOBUFS))
#else
	if((res < 0) && (errno == ENOBUFS))
#endif
		return 1;

	return res;
}
// -------------------------------------------------------------------------
// send a packet (with frame added) of 32 bit entries after conversion to network byte order, in packetid the actual 
// _sendcount is returned
//
int Com::send32(void *packet, int packetsize, UINT32 packetid, UINT32 packetstatus) {

	int res;
	struct _PacketFrame packetframe;
	int framedpacketsize = packetsize + sizeof(_PacketFrame);
	
	assert( framedpacketsize <= _max_packetsize);
	
	
	packetframe.packetnr = _sendcount;
	packetframe.packetid = packetid;
	packetframe.status = packetstatus;
	
	memcpy(_npacket.data(), &packetframe, sizeof(_PacketFrame));
	memcpy( ((char *)_npacket.data())+sizeof(_PacketFrame), packet, packetsize);

	hton_packet32(_npacket.data(), _npacket.data(), framedpacketsize);
	
	res = send_raw(_npacket.data(), framedpacketsize);
	if(res > 0) {
	  _sendcount++;
	}
	

	return res;		

}
// -----------------------------------------------------------------------
//
//
Com::~Com() {
#if (OSCPU_TYPE == OC_WIN32_INTEL)	
	if(_peer_bound) 
		closesocket(_sockFD);
	//WSACleanup();
#else
	if(_peer_bound) 
		close(_sockFD);
#endif
}	
