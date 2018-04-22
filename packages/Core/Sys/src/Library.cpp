/** @file Device.cpp
 * 
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sys/Library.h>
#ifndef WIN32
#include <dlfcn.h>
#ifndef __APPLE__
#include <link.h>
#endif
#include <inttypes.h>
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void Handle::release(HANDLE* pHandle) {
	HANDLE handle = *pHandle;
	delete pHandle;
	if (handle) {
#ifdef WIN32
		::FreeLibrary(handle);
#else	// WIN32
		::dlclose(handle);
#endif	// WIN32
	}
}

Handle::Handle(const std::string& path) : handle(new HANDLE(NULL)), name(path) {
	const size_t pos = path.find_last_of("\\/") + 1;
#ifdef WIN32
#ifdef _DEBUG
	const std::string name = path.substr(0, pos) + "" + path.substr(pos) + "DEBUG.dll";
#else	// WIN32
	const std::string name = path.substr(0, pos) + "" + path.substr(pos) + ".dll";
#endif	// WIN32
	*handle = ::LoadLibrary(name.c_str());
	if (!*handle)
		throw MsgHandleOpen(Message::LEVEL_CRIT, "Handle::Handle(): Unable to open library: %s", name.c_str());
#else	// WIN32
#ifdef __APPLE__
	const std::string name = path.substr(0, pos) + "lib" + path.substr(pos) + ".dylib";
#else
	const std::string name = path.substr(0, pos) + "lib" + path.substr(pos) + ".so";
#endif
	*handle = ::dlopen(name.c_str(), RTLD_LAZY);
	if (!*handle)
		throw MsgHandleOpen(Message::LEVEL_CRIT, "Handle::Handle(): Unable to open library: %s (%s)", name.c_str(), dlerror());
#endif	// WIN32
}

std::string Handle::getPath() const {
	if (getHandle() == 0)
		throw MsgHandle(Message::LEVEL_CRIT, "Handle::getPath(): invalid handle");
#ifdef WIN32
	TCHAR path[MAX_PATH];
	if (!::GetModuleFileName(*handle, path, MAX_PATH))
		throw MsgHandle(Message::LEVEL_CRIT, "Handle::getPath(): unable to retrieve library path (%d)", GetLastError());
	return std::string(path);
#else
#ifdef __APPLE__
	Dl_info *info = NULL;
	if (dladdr(*handle, info) || info == NULL)
		throw MsgHandle(Message::LEVEL_CRIT, "Handle::getPath(): unable to retrieve library path (%s)", dlerror());
	return std::string(info->dli_fname);
#else
	link_map *linkMap = NULL;
	if (::dlinfo(*handle, RTLD_DI_LINKMAP, &linkMap) < 0 || linkMap == NULL)
		throw MsgHandle(Message::LEVEL_CRIT, "Handle::getPath(): unable to retrieve library path (%s)", dlerror());
	return std::string(linkMap->l_name);
#endif
#endif
}

std::string Handle::getDir() const {
	std::string path = getPath();
	const std::size_t posdir = path.find_last_of("/\\");
	return posdir != std::string::npos ? path.substr(0, posdir + 1) : path.length() > 0 ? path + '/' : std::string("./");
}

const std::string& Handle::getName() const {
	return name;
}

Handle::HANDLE Handle::getHandle() const {
	return *handle;
}

Handle::FUNCTION Handle::getFunction(const char* name) {
#ifdef WIN32
	FUNCTION function = ::GetProcAddress(*handle, name);
#else
	FUNCTION function = ::dlsym(*handle, name);
#endif
	if (!function)
		throw MsgHandleFunc(Message::LEVEL_CRIT, "Handle::getFunction(): Unable to obtain pointer to %s", name);
	return function;
}

//------------------------------------------------------------------------------
