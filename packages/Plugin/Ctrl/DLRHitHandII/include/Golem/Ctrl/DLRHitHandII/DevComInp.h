#pragma once
#ifndef _DEV_COM_INP_H_
#define _DEV_COM_INP_H_

namespace golem {

class Com;

class DevComInp {
	Com* com;

public:
	DevComInp();
	~DevComInp();

	int init(int port, int size);
	int rec(void *packet, int size, unsigned timeout_usec = -1);
};

};

#endif
