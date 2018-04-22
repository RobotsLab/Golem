/** @file GolemInteropLibrary.cpp
 * 
 * @author	Marek Kopicki
 *
 */

#include "GolemInteropLibrary.h"
#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#define NOMINMAX
#include <windows.h>
#else
#include <dlfcn.h>
#ifndef __APPLE__
#include <link.h>
#endif
#include <inttypes.h>
#endif
#include <stdexcept>
#include <algorithm>

//------------------------------------------------------------------------------

using namespace golem;
using namespace golem::interop;

//------------------------------------------------------------------------------

Library::Library(const std::string& library, const std::string& prefix, const std::string& suffix) : library(library), prefix(prefix), suffix(suffix), interface(nullptr) {
	handle.reset(new Handle, [] (Handle* phandle) {
		Handle handle = *phandle;
		delete phandle;
		if (handle) {
#ifdef WIN32
			::FreeLibrary((HMODULE)handle);
#else	// WIN32
			::dlclose(handle);
#endif	// WIN32
		}
	});

	const size_t pos = library.find_last_of("\\/") + 1;
	const std::string path = library.substr(0, pos) + getPrefix() + library.substr(pos) + getSuffix();

#ifdef WIN32
	*handle = (Handle)::LoadLibrary(path.c_str());
	if (!*handle)
		throw std::runtime_error("golem::interop::Library::Library(): Unable to open library: " + path);
#else	// WIN32
	*handle = ::dlopen(path.c_str(), RTLD_LAZY);
	if (!*handle)
		throw std::runtime_error("golem::interop::Library::Library(): Unable to open library: " + path);
#endif	// WIN32
}

std::string Library::getPath() const {
	if (!*handle)
		throw std::runtime_error("golem::interop::Library::getPath(): invalid handle");
#ifdef WIN32
	TCHAR path[MAX_PATH];
	if (!::GetModuleFileName((HMODULE)*handle, path, MAX_PATH))
		throw std::runtime_error("golem::interop::Library::getPath(): unable to retrieve library path: " + std::to_string(GetLastError()));
	return std::string(path);
#else
#ifdef __APPLE__
	Dl_info *info = NULL;
	if (dladdr(*handle, info) || info == NULL)
		throw std::runtime_error("golem::interop::Library::getPath(): unable to retrieve library path: " + std::string(dlerror()));
	return std::string(info->dli_fname);
#else
	link_map *linkMap = NULL;
	if (::dlinfo(*handle, RTLD_DI_LINKMAP, &linkMap) < 0 || linkMap == NULL)
		throw std::runtime_error("golem::interop::Library::getPath(): unable to retrieve library path: " + std::string(dlerror()));
	return std::string(linkMap->l_name);
#endif
#endif
}

Library::~Library() {
	release();
}

Library::Symbol Library::loadSymbol(const std::string& name) {
#ifdef WIN32
	Symbol symbol = (Symbol)::GetProcAddress((HMODULE)*handle, name.c_str());
#else
	Symbol symbol = ::dlsym(*handle, name.c_str());
#endif
	if (!symbol)
		throw std::runtime_error("golem::interop::Library::loadSymbol(): Unable to load symbol: " + name);
	return symbol;
}

std::string Library::getDefaultPrefix() {
#ifdef WIN32
	return std::string("");
#else
	return std::string("lib");
#endif
}

std::string Library::getDefaultSuffix() {
#ifdef WIN32
#ifdef _DEBUG
	return std::string("DEBUG.dll");
#else	// WIN32
	return std::string(".dll");
#endif	// WIN32
#else
#ifdef __APPLE__
	return std::string(".dylib");
#else
	return std::string(".so");
#endif
#endif
}

Interface* Library::getInterface(const std::string& param) {
	release();
	return interface = reinterpret_cast<Interface*>(getInterfaceLoader()(param.c_str()));
}

void Library::release() {
	if (interface) {
		interface->release();
		interface = nullptr;
	}
}

//------------------------------------------------------------------------------

Library::List::Base::iterator Library::List::insert(const std::string& library, const std::string& prefix, const std::string& suffix) {
	// find if the element is already on the list (slow linear search but only single container required)
	const Library::List::Base::iterator ptr = std::find(begin(), end(), library);
	return ptr != end() ? ptr : Base::insert(end(), Library(library, prefix, suffix)); // = push_back()
}

void Library::List::clear() {
	// LIFO release
	while (!empty())
		pop_back();
}

Library::List::~List() {
	clear();
}

//------------------------------------------------------------------------------

