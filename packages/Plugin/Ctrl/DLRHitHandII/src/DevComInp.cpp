#include <Golem/Ctrl/DLRHitHandII/DevComInp.h>
#include <Golem/Ctrl/DLRHitHandII/Com.h>

using namespace golem;

// ----------------------------------------------------------

DevComInp::DevComInp() : com(NULL) {
	com = new Com;
}

DevComInp::~DevComInp() {
	if (com != NULL)
		delete com;
}

// ----------------------------------------------------------

int DevComInp::init(int port, int size) {
	return com->initrec(port, size);
}

// ----------------------------------------------------------

int DevComInp::rec(void *packet, int size, unsigned timeout_usec) {
	UINT32 rec_packetid ;
	UINT32 rec_packetstatus;
	UINT32 rec_packetnr;
	return com->rec32(packet, size, &rec_packetid, &rec_packetstatus, &rec_packetnr, timeout_usec);
}
