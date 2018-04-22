/** @file ContactServer.h
*
* @author	Marek Kopicki
*
*/

#pragma once
#ifndef _GOLEM_INTEROP_PLUGIN_CONTACT_SERVER_CONTACT_SERVER_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_PLUGIN_CONTACT_SERVER_CONTACT_SERVER_H_

//------------------------------------------------------------------------------

#include "GolemInteropMaster.h"
#include "GolemInteropStreamSocket.h"

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_INTEROP_LIBRARY_DECLDIR void* InterfaceLoader(const char*);
};

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	/** Master interface implementation */
	class ContactServer : public MasterConfig {
	public:
		/** Master: Take over control given set of interfaces
		*	@param[in]	interfaces		set of interfaces
		*/
		virtual void run(const Map& interfaces);

		/** Create implementation */
		ContactServer(const std::string& param);

	private:
		/** Port */
		const unsigned short port;
		/** TCP/IP server */
		Server server;
		/** Terminating */
		bool terminate;

		/** Request handler */
		void handler(Stream& stream, const Map& interfaces);
	};

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_PLUGIN_CONTACT_SERVER_CONTACT_SERVER_H_