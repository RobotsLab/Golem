/** @file ContactLearning.h
*
* @author	Marek Kopicki
*
*/

#pragma once
#ifndef _GOLEM_INTEROP_PLUGIN_CONTACT_LEARNING_CONTACT_LEARNING_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_PLUGIN_CONTACT_LEARNING_CONTACT_LEARNING_H_

//------------------------------------------------------------------------------

#include "GolemInteropContactInterface.h"
#include "GolemInteropMaster.h"

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
	class ContactLearning : public MasterConfig {
	public:
		/** Master: Take over control given set of interfaces
		*	@param[in]	interfaces		set of interfaces
		*/
		virtual void run(const Map& interfaces);

		/** Create implementation */
		ContactLearning(const std::string& param);
	};

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_PLUGIN_CONTACT_LEARNING_CONTACT_LEARNING_H_