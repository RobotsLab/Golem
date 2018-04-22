/** @file Library.h
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
#ifndef _GOLEM_PLUGIN_LIBRARY_H_
#define _GOLEM_PLUGIN_LIBRARY_H_

//------------------------------------------------------------------------------

#include <Golem/Sys/Library.h>
#include <Golem/Sys/Context.h>
#include <Golem/Sys/XMLParser.h>
#include <Golem/Sys/Defs.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Loads object description - the library function must be implemented in each dynamic library */
typedef void*(*GolemDescLoader)(void);

//------------------------------------------------------------------------------

/** Library interface */
class Library {
public:
	typedef golem::shared_ptr<Library> Ptr;

	/** Library name separator */
	static const std::string NAME_SEP;

	/** Dynamic library path */
	class Path {
	public:
		typedef std::vector<Path> Seq;

		/** Library path without system-specific dynamic library extension (.dll or .so) or version (e.g. DEBUG) */
		std::string library;
		/** Description path without file extension (.xml) */
		std::string config;

		/** Set to default */
		Path() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			library = "Library";
			config = "Config";
		}

		/** Assert that the path is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(library.length() > 0, ac, "library: invalid");
			Assert::valid(config.length() > 0, ac, "config: invalid");
		}
	};

	/** Library description */
	class Desc {
	public:
		friend class Library;
		typedef golem::shared_ptr<Desc> Ptr;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		/** Destroys description. */
		virtual ~Desc() {}
		
		/** Sets the parameters to the default values. */
		void setToDefault() {
			libraryPrefix = "Golem";
			libraryName = "golem library";
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Assert::valid(libraryName.length() > 0, ac, "libraryName: empty");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext) = 0;

		/** Loads camera description from dynamic library.
		* @param context		program context
		* @param lib			library path and configuration
		* @return				pointer to the description if no errors have occured, throws otherwise
		*/
		template <typename _Desc> static golem::shared_ptr<_Desc> loadLibrary(golem::Context& context, const Path& path) {
			context.debug("Library::Desc::loadLibrary(): loading library %s and config %s.xml...\n", path.library.c_str(), path.config.c_str());

			// open library and load function
			golem::Handle handle = context.getLibrary(path.library); // throws
			GolemDescLoader golemDescLoader = (GolemDescLoader)handle.getFunction("golemDescLoader"); // throws

			// load description
			golem::shared_ptr<_Desc> pDesc(reinterpret_cast<_Desc*>(golemDescLoader()));
			if (pDesc == nullptr)
				throw golem::Message(golem::Message::LEVEL_CRIT, "golem::Library::Desc::loadLibrary(): unable to load description");

			// load config
			golem::XMLParser::Ptr parser;
			try {
				// first try to load directly the specified config
				parser = golem::XMLParser::load(path.config + ".xml");
			}
			catch (const golem::Message&) {
				// if failed, attempt to load from library location
				parser = golem::XMLParser::load(handle.getDir() + path.config + ".xml");
			}
			// load config from xml context
			pDesc->load(context, parser->getContextRoot()->getContextFirst(pDesc->libraryName.c_str())); // throws
			pDesc->path = path;

			return pDesc;
		}

		/** Library */
		std::string getLibrary() const;
		/** Config */
		std::string getConfig() const;

		/** Handler id is a combination of dynamic library and configuration file names without Desc::libraryPrefix */
		std::string getID() const;

	protected:
		/** Library family prefix */
		std::string libraryPrefix;
		/** Library family XML name */
		std::string libraryName;

	private:
		/** Library and config path */
		Path path;
	};

	/** Handler type is a dynamic library name without Desc::libraryPrefix */
	const std::string& getType() const {
		return type;
	}
	/** Handler id is a combination of dynamic library and configuration file names without Desc::libraryPrefix */
	const std::string& getID() const {
		return id;
	}
	/** Handler library description contains paths to dynamic library and configuration file */
	const Path& getPath() const {
		return path;
	}

	/** Context */
	golem::Context& getContext() {
		return context;
	}
	const golem::Context& getContext() const {
		return context;
	}

	/** Destroys the Library */
	virtual ~Library();
	
protected:
	/** Context reference */
	golem::Context& context;

	/** Creates/initialises the Library */
	void create(const Desc& desc);
	
	/** Constructs the Library */
	Library(golem::Context& context);

private:
	/** Library and config path */
	Path path;
	/** Type */
	std::string type;
	/** Id */
	std::string id;
};

/** Reads/writes object from/to a given XML context */
void XMLData(golem::Library::Path::Seq::value_type& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_PLUGIN_LIBRARY_H_*/
