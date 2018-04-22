/** @file HelloWorld.h
*
* @author	Marek Kopicki
*
*/

#pragma once
#ifndef _GOLEM_INTEROP_PLUGIN_HELLOWORLD_HELLOWORLD_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_PLUGIN_HELLOWORLD_HELLOWORLD_H_

//------------------------------------------------------------------------------

#include "GolemInteropInterface.h"

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_INTEROP_LIBRARY_DECLDIR void* InterfaceLoader(const char*);
};

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	/** Master interface */
	class HelloWorld : public Master {
	public:
		/** Master: Take over control given set of interfaces
		*	@param[in]	interfaces		set of interfaces
		*/
		virtual void run(const Map& interfaces);
	};

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_PLUGIN_HELLOWORLD_HELLOWORLD_H_