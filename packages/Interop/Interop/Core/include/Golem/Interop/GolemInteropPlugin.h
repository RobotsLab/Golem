/** @file GolemInteropPlugin.h
*
* @author	Marek Kopicki
*
*/

#pragma once
#ifndef _GOLEM_INTEROP_INTEROP_PLUGIN_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_INTEROP_PLUGIN_H_

//------------------------------------------------------------------------------

#include <string>
#include <map>

//------------------------------------------------------------------------------

#undef GOLEM_INTEROP_LIBRARY_DECLDIR
#define GOLEM_INTEROP_LIBRARY_DECLDIR

#ifdef GOLEM_INTEROP_LIBRARY_DECLDIR_EXPORT // export DLL
#undef GOLEM_INTEROP_LIBRARY_DECLDIR
#define GOLEM_INTEROP_LIBRARY_DECLDIR __declspec(dllexport)
#endif

#ifdef GOLEM_INTEROP_LIBRARY_DECLDIR_IMPORT // import DLL
#undef GOLEM_INTEROP_LIBRARY_DECLDIR
#define GOLEM_INTEROP_LIBRARY_DECLDIR __declspec(dllimport)
#endif

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	/*************************************************************************
	*
	* Base system interface
	*
	**************************************************************************/

	/** Base system interface */
	class Interface {
	public:
		/** Interface info */
		class Info {
		public:
			/** Library path */
			std::string library;
			/** Interface loader parameter (e.g. configuration file) */
			std::string param;

			/** Set to default */
			Info() {
				clear();
			}
			/** Construction */
			Info(const std::string& library, const std::string& param) : library(library), param(param) {
			}

			/** Sets the parameters to the default values */
			void clear() {
				library.clear();
				param.clear();
			}
			/** Comparator */
			friend bool operator < (const Info &left, const Info &right) {
				return left.library < right.library || left.library == right.library && left.param < right.param;
			}
		};

		/** Interface map links paths and library pointers with corresponding interface */
		typedef std::map<Info, Interface*> Map;

		/** Dynamic casting _Inp to _Out */
		template <typename _Type> static inline _Type* is(Interface* inp) {
			return dynamic_cast<_Type*>(inp);
		}

		/** Out-of-order release of resources */
		virtual void release() {}

	protected:
		/** Empty protected default implementation */
		virtual ~Interface() {}
	};

	/** Interface loader type */
	typedef void*(*InterfaceLoaderFunction)(const char*);
	/** Interface loader name */
	const std::string InterfaceLoaderName("InterfaceLoader");

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_INTEROP_PLUGIN_H_