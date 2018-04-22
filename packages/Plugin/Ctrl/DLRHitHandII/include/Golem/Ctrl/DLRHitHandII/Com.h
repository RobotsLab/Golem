#pragma once
#ifndef _COM_H_
#define _COM_H_

#include <Golem/Ctrl/DLRHitHandII/Types.h>
#include <vector>

#if (OSCPU_TYPE == OC_VXWORKS_PPC)
#include <in.h>
#endif

#if ((OSCPU_TYPE == OC_QNX_INTEL) || (OSCPU_TYPE == OC_LINUX_INTEL))
#include <sys/select.h>
#endif

#if( OSCPU_TYPE == OC_WIN32_INTEL)
#include <winsock2.h>
#else
#include <arpa/inet.h>
#endif

#if (OSCPU_TYPE == OC_VXWORKS_INTEL)
#include <sys/time.h>
#endif

namespace golem {

// -------------------------------------------------------------

class Com {
	struct _PacketFrame {
		UINT32 packetnr;
		UINT32 packetid; // an individual id, the user provides for every packet in the send call. The rec call returns this id.
		UINT32 status;
	};

	int _sockFD;
	struct sockaddr_in _inLocal;
	struct sockaddr_in _inPeer;
	int _inPeerSize;
	int _port;
	int _max_packetsize;
	std::vector<char> _npacket;
	UINT32 _sendcount;
	int _peer_bound;

	int init(int udp_port, int max_packetsize = 0);

public:
	Com();
	~Com();

	int initrec(int udp_port, int max_packetsize);
	int initsend(const char *peer_addr, int udp_port, int max_packetsize);

	int is_pending();
	int send_raw(void *packet, int packetsize);
	int rec_raw(void *packet, int packetsize, unsigned timeout_usec = -1);
	int send32(void *packet, int packetsize, UINT32 packetid, UINT32 packetstatus);
	int rec32(void *packet, int packetsize, UINT32 *packetid, UINT32 *packetstatus, UINT32 *packetnr, unsigned timeout_usec = -1);
};

}
#endif

