/** @file Context.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sys/Context.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Context::Context() {
}

Context::~Context() {
	if (parallels != NULL && !parallels->joinThreads(threadTimeOut)) {
	}
	releaseLibraries();
}

void Context::create(const Desc &desc) {
	if (!desc.isValid())
		throw MsgContextInvalidDesc(Message::LEVEL_CRIT, "Context::create(): Invalid description");

	initModule();

	randSeed = desc.randSeed;
	messages = desc.messageStreamDesc->create();

	threadTimeOut = desc.threadTimeOut;
	if (desc.threadParallels > 0) {
		parallels.reset(new Parallels(desc.threadParallels, desc.threadTimeOut));
		parallels->startThreads();
	}
}

void Context::initModule() {
	Message::setTimer(&timer);
}

Handle Context::getLibrary(const std::string& path) {
	Handle::Map::iterator ptr = libraryMap.find(path);
	if (ptr == libraryMap.end()) {
		//fprintf(stderr, "Context::getLibrary(): context %x, loading library %s\n", this, path.c_str());
		verbose("Context::getLibrary(): context %x, loading library %s\n", this, path.c_str());
		ptr = libraryMap.insert(libraryMap.begin(), Handle::Map::value_type(path, Handle(path.c_str())));
		libraryList.push_back(ptr);
	}
	return ptr->second;
}

void Context::releaseLibraries() {
	while (!libraryList.empty()) {
		Handle::Map::iterator ptr = libraryList.back();
		//fprintf(stderr, "Context::releaseLibraries(): context %x, releasing library %s\n", this, ptr->second.getName().c_str());
		verbose("Context::releaseLibraries(): context %x, releasing library %s\n", this, ptr->second.getName().c_str());
		libraryList.pop_back();
		libraryMap.erase(ptr);
	}
}

//------------------------------------------------------------------------------
