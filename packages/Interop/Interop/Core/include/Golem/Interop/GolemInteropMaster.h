/** @file Master.h
*
* @author	Marek Kopicki
*
*/

#pragma once
#ifndef _GOLEM_INTEROP_INTEROP_MASTER_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_INTEROP_MASTER_H_

//------------------------------------------------------------------------------

#include "GolemInteropInterface.h"
#include "GolemInteropTypeInfo.h"

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	/** Master config loader abstract class */
	class MasterConfig : public Master {
	public:
		/** Create Master config */
		MasterConfig(const std::string& param);

	private:
		typedef std::map<std::string, Interface::Info> InterfaceMap;
		InterfaceMap interfaceMap;

	protected:
		template <typename _Type> _Type* getInterface(const Map& interfaces, bool noThrow = false) {
			const std::string name(std::move(getTypeName<_Type>()));

			InterfaceMap::const_iterator pinfo = interfaceMap.find(name);
			if (pinfo == interfaceMap.end())
				if (noThrow)
					return nullptr;
				else
					throw std::runtime_error("golem::interop::MasterConfig::getInterface(): unable to find interface: " + name);
			
			Interface::Map::const_iterator pinterface = interfaces.begin();
			if (pinfo->second.param.empty())
				for (; pinterface != interfaces.end() && pinterface->first.library != pinfo->second.library; ++pinterface);
			else
				pinterface = interfaces.find(pinfo->second);
			
			if (pinterface == interfaces.end())
				if (noThrow)
					return nullptr;
				else
					throw std::runtime_error("golem::interop::MasterConfig::getInterface(): interface not available: " + name);

			_Type* type = is<_Type>(pinterface->second);
			if (!type)
				if (noThrow)
					return nullptr;
				else
					throw std::runtime_error("golem::interop::MasterConfig::getInterface(): unable to cast to: " + name);
			
			return type;
		}
	};

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_INTEROP_MASTER_H_