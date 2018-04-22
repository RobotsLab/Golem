/*
 * Connector.cpp
 *
 *  Created on: 22 Aug 2017
 *      Author: rusi
 This function, sadly, depends on Dropbox 
 to be able to communicate with its clients. 
 The files to be transmitted are placed in dropbox folder, 
 and are copied from there.
 */

#include "Connector.h"
#include <cstdlib>
#include <cerrno>
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <filesystem>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
#include <string.h>

namespace Grasp {

Connector::Connector() {
	// TODO Auto-generated constructor stub
}

Connector::~Connector() {
	// TODO Auto-generated destructor stub
}
// Function that uploads a file to the server.
bool Connector::UploadFile(const char *name){
	bool flag = false;
	char buf[1000];
	buf[0] = 0;

	// Set up request options.
	char remotePath[100];
	remotePath[0] = 0;
	strcat(remotePath, DROPBOX_OUT_PATH);

	// Get filename and extension.
	char * fn = (char *)strrchr(name, '/');
	char * fn2 = (char *)strrchr(name, '.');
	fn = fn + 1;
	int nameLength = fn2 - fn;
	strncat(remotePath, fn, nameLength);
	strcat(remotePath, "/");
	strncat(remotePath, fn, nameLength);
	std::cout<<"Remote path:"<<remotePath<<std::endl;

	boost::filesystem::rename(name, remotePath);
    return flag;
}

// Function that downloads file from a server.
bool Connector::DownloadFile(char *name, int serverID){
	char serverFolder[1000], numb[10];
	strcpy(serverFolder, DROPBOX_PATH);
	strcat(serverFolder, "server\\");
	sprintf(numb, "%d", serverID);
	strcat(serverFolder, numb);
	strcat(serverFolder, "\\");

	boost::filesystem::path p(serverFolder);
	boost::filesystem::directory_iterator end_itr;
	bool flag = false;
	char oldestName[100];
	time_t oldestTime = std::numeric_limits<time_t>::max();

	// Take the first file you see.
	int count = 0;
	for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
	{
		if (i->path().extension().string() == ".pcd")
		{
			time_t newTime = boost::filesystem::last_write_time(i->path());
			if (newTime < oldestTime)
			{
				oldestTime = newTime;
				strcpy(oldestName, i->path().string().c_str());
			}
			flag = true;
		}
	}
	if (flag)
	{
		std::cout << oldestName << std::endl;
		strcpy(name, oldestName);
	}
	
	// Copy file.
	return flag;
}

} /* namespace Grasp */
