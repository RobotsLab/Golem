/** @file Library.h
 * 
 * Library tools.
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_SYS_LIBRARY_H_
#define _GOLEM_SYS_LIBRARY_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Pointers.h>
#include <Golem/Defs/System.h>
#include <Golem/Sys/Message.h>
#include <string>
#include <map>
#include <list>

//------------------------------------------------------------------------------

#undef GOLEM_LIBRARY_DECLDIR
#define GOLEM_LIBRARY_DECLDIR

#ifdef GOLEM_LIBRARY_DECLDIR_EXPORT // export DLL
#undef GOLEM_LIBRARY_DECLDIR
#define GOLEM_LIBRARY_DECLDIR __declspec(dllexport)
#endif

#ifdef GOLEM_LIBRARY_DECLDIR_IMPORT // import DLL
#undef GOLEM_LIBRARY_DECLDIR
#define GOLEM_LIBRARY_DECLDIR __declspec(dllimport)
#endif

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgHandle, Message)
MESSAGE_DEF(MsgHandleOpen, MsgHandle)
MESSAGE_DEF(MsgHandleFunc, MsgHandle)

//------------------------------------------------------------------------------

/** Library */
class Handle {
public:
#ifdef WIN32
	typedef HINSTANCE HANDLE; // Windows
	typedef FARPROC FUNCTION; // Windows
#else	// WIN32
	typedef void* HANDLE;
	typedef void* FUNCTION;
#endif	// WIN32
	/** Map */
	typedef std::map<std::string, Handle> Map;
	/** List */
	typedef std::list<Map::iterator> List;

	/** Open library */
	Handle(const std::string& path);
	/** Library path */
	std::string getPath() const;
	/** Library directory */
	std::string getDir() const;
	/** Library name */
	const std::string& getName() const;
	/** Handle */
	HANDLE getHandle() const;
	/** Function */
	FUNCTION getFunction(const char* name);

private:
	/** Description relese */
	static void release(HANDLE* pHandle);
	friend class reference_cnt<HANDLE, Handle>;

	/** Library name */
	std::string name;
	/** Library address */
	shared_ptr<HANDLE, reference_cnt<HANDLE, Handle> > handle;
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_SYS_LIBRARY_H_*/
