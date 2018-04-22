/*
 * Connector.h
 *
 *  Created on: 22 Aug 2017
 *      Author: rusi
 */

#ifndef UTIL_CONNECTOR_H_
#define UTIL_CONNECTOR_H_

#define SERVER_ADDRESS "http://www.google.com"
#define PORT 8000
#define DROPBOX_PATH "C:\\Users\\Admin\\Dropbox\\"
#define DROPBOX_OUT_PATH "C:\\Users\\Admin\\Dropbox\\data\\"

namespace Grasp {

class Connector {
public:
	Connector();
	virtual ~Connector();

	// Function to send files to server (Blocking)
	// name should be the absolute path (client side).
	static bool UploadFile(const char *name);

	// Function to get files from server. (Unblocking)
	// If you wish to have next set of points, name field should be "".
	// The file name should be an absolute path otherwise (on client side)
	static bool DownloadFile(char *name, int serverID);

};

} /* namespace Grasp */

#endif /* UTIL_CONNECTOR_H_ */
