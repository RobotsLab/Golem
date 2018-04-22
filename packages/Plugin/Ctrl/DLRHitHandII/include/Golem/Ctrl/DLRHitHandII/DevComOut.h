#pragma once
#ifndef _ARDCOMOUTUDP_H_
#define _ARDCOMOUTUDP_H_

#include <string>

namespace golem {

class Com;

class DevComOut {
	Com* com;

public:
	DevComOut();
	~DevComOut();

	int init(const std::string& peer_addr, int port, int size);
	int send(void *packet, int size);
};

};

#endif
