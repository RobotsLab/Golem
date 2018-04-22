/** @file TypeInfo.h
*
* @author	Marek Kopicki
*
*/

#pragma once
#ifndef _GOLEM_INTEROP_INTEROP_TYPEINFO_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_INTEROP_TYPEINFO_H_

//------------------------------------------------------------------------------

#include <string>
#include <typeinfo>
#ifdef WIN32
#include <cstring>
#else
#include <memory>
#include <cxxabi.h>
#endif

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	/** Implementation-independent type name
	*	TODO Use <boost/core/demangle.hpp> for Boost ver >= 1.59
	*/
	template <typename _Type> static std::string getTypeName() {
		const char* name = typeid(_Type).name();
		// unmangle names
#ifdef WIN32
		// Visual Studio
		const char* demangled = std::strstr(name, " "); // skip "Class" or "struct"
		return std::string(demangled ? demangled + 1 : name);
#else
		// gcc
		int status = -1;
		std::unique_ptr<char> demangled(abi::__cxa_demangle(name, nullptr, nullptr, &status)); // allocates on heap
		return std::string(status == 0 && demangled.get() ? demangled.get() : name);
#endif
	}

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_INTEROP_TYPEINFO_H_