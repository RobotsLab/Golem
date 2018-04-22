#include <Golem/Ctrl/DLRHitHandII/DevComOut.h>
#include <Golem/Ctrl/DLRHitHandII/Com.h>

using namespace golem;

// ----------------------------------------------------------

DevComOut::DevComOut() : com(NULL) {
	com = new Com;
}

DevComOut::~DevComOut() {
	if (com != NULL)
		delete com;
}

// ----------------------------------------------------------

int DevComOut::init(const std::string& peer_addr, int port, int size) {
	return com->initsend(peer_addr.c_str(), port, size);
}

// ----------------------------------------------------------

int DevComOut::send(void *packet, int size) {
	UINT32 send_packetid = 0;
	UINT32 send_packetstatus = 0;
	return com->send32(packet, size, send_packetid, send_packetstatus);
}
