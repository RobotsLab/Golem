/** @file Library.h
 * 
 * @author	Marek Kopicki
 *
 */

#pragma once
#ifndef _GOLEM_INTEROP_INTEROP_LIBRARY_H_
#define _GOLEM_INTEROP_INTEROP_LIBRARY_H_

//------------------------------------------------------------------------------

#include "GolemInteropInterface.h"
#include <memory>
#include <list>

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	//------------------------------------------------------------------------------

	/** Library container */
	class Library {
	public:
		/** Library list with unique elements and preserving insertion and release order */
		class List : public std::list<Library> {
		public:
			typedef std::list<Library> Base;

			/** Release libraries in reverse order */
			~List();

			/** Insert library always in order and if not on the list already */
			Base::iterator insert(const std::string& library, const std::string& prefix = getDefaultPrefix(), const std::string& suffix = getDefaultSuffix());

			/** Release libraries in reverse order */
			void clear();
		};

		/** Make friends */
		friend class List;

		/** Requested library path */
		const std::string& getLibrary() const {
			return library;
		}

		/** Actual library path */
		std::string getPath() const;

		/** Library prefix */
		const std::string& getPrefix() const {
			return prefix;
		}
		/** Library suffix */
		const std::string& getSuffix() const {
			return suffix;
		}

		/** Default library prefix */
		static std::string getDefaultPrefix();

		/** Default library suffix */
		static std::string getDefaultSuffix();

		/** Get symbol */
		template <typename _Type> _Type getSymbol(const std::string& name) {
			return reinterpret_cast<_Type>(loadSymbol(name));
		}

		/** Get Interface loader */
		InterfaceLoaderFunction getInterfaceLoader() {
			return getSymbol<InterfaceLoaderFunction>(InterfaceLoaderName);
		}

		/** Get Interface */
		Interface* getInterface(const std::string& param);

		/** Comparator */
		friend bool operator == (const Library &left, const std::string &right) {
			return left.getLibrary() == right;
		}

		/** Close library */
		~Library();

	private:
		typedef void* Handle;
		typedef void* Symbol;

		/** Library prefix */
		std::string prefix;
		/** Library suffix */
		std::string suffix;
		/** Library path */
		std::string library;
		/** Library handle can be safely shared when Library is copied */
		std::shared_ptr<Handle> handle;
		/** Interface */
		Interface* interface;

		/** Open library */
		Library(const std::string& library, const std::string& prefix = getDefaultPrefix(), const std::string& suffix = getDefaultSuffix());

		/** Load symbol */
		Symbol loadSymbol(const std::string& name);
		/** Release interface */
		void release();
	};

	//------------------------------------------------------------------------------

}; // namespace interop
}; // namespace golem

#endif /*_GOLEM_INTEROP_INTEROP_LIBRARY_H_*/
