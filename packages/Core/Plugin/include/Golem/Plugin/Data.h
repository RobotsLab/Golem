/** @file Data.h
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
#ifndef _GOLEM_PLUGIN_DATA_H_
#define _GOLEM_PLUGIN_DATA_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Defs.h>
#include <Golem/Plugin/Library.h>
#include <Golem/Plugin/Renderer.h>
#ifdef LINUX
#define BOOST_NO_CXX11_SCOPED_ENUMS
#define BOOST_NO_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#undef BOOST_NO_SCOPED_ENUMS
#else
#include <boost/filesystem.hpp>
#endif
#include <memory>
#include <list>

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

/** File status interface helper.
*/
class Status {
public:
	/** Is modified? */
	bool isModified() const {
		return modified;
	}
	/** Is modified? */
	void setModified(bool modified) {
		this->modified = modified;
	}

	/** Is temporary? */
	bool isTemporary() const {
		return temporary;
	}
	/** Is temporary? */
	void setTemporary(bool temporary) {
		this->temporary = temporary;
	}

	/** Required by interface */
	Status(bool modified = false, bool temporary = false) : modified(modified), temporary(temporary) {}

protected:
	/** Modifed flag */
	bool modified;
	/** Temporary flag */
	bool temporary;
};

/** File container helper.
*/
class File : public Status {
public:
	/** Pointer */
	typedef std::shared_ptr<File> Ptr;
	/** Container */
	typedef std::vector<Ptr> Seq;

	/** File operations description */
	class Desc {
	public:
		/** Delete file at old location after copying it to a new place */
		bool deleteIfMoved;
		/** Delete file if unlinked from memory resources */
		bool deleteIfUnlinked;
		/** Delete file if it is temporary file */
		bool deleteIfTemporary;

		/** Set to default */
		Desc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			deleteIfMoved = true;
			deleteIfUnlinked = true;
			deleteIfTemporary = true;
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Create file container */
	File() {
	}
	/** Create file container */
	File(const Desc& desc) : desc(desc) {
	}
	/** Delete if temporary */
	~File() {
		if (isTemporary() && desc.deleteIfTemporary) remove(this->path);
	}

	/** Load from disk */
	template <typename _Handler> void load(const std::string& path, _Handler handler) {
		handler(path);
		this->path = path;
		setModified(false);
	}
	/** Save to disk */
	template <typename _Handler> void save(const std::string& path, _Handler handler) {
		// new path
		const bool hasNewPath = this->path != path;
		if (isModified()) {
			// always save if modified
			handler(path);
			setModified(false);
			// new path
			if (hasNewPath) {
				// delete old file
				if (desc.deleteIfMoved) remove(this->path);
				// update current path
				this->path = path;
			}
		}
		else if (hasNewPath) {
			if (boost::filesystem::exists(boost::filesystem::path(this->path))) {
				if (desc.deleteIfMoved) {
					// there exists a new path - remove it
					if (boost::filesystem::exists(boost::filesystem::path(path))) remove(path);
					// path is different - move
					boost::filesystem::rename(boost::filesystem::path(this->path), boost::filesystem::path(path));
				}
				else {
					// path is different - copy
					boost::filesystem::copy_file(boost::filesystem::path(this->path), boost::filesystem::path(path), boost::filesystem::copy_option::overwrite_if_exists);
				}
			}
			// update current path
			this->path = path;
		}
	}
	/** Delete */
	void remove() {
		if (desc.deleteIfUnlinked) {
			remove(path);
			path.clear();
		}
	}

	/** File operations description */
	void setDesc(const Desc& desc) {
		this->desc = desc;
	}
	/** File operations description */
	const Desc& getdesc() const {
		return desc;
	}

	/** File path */
	const std::string& getPath() const {
		return path;
	}

protected:
	/** File operations description */
	Desc desc;
	/** File path */
	std::string path;

	/** Delete */
	static void remove(const std::string& path) {
		// remove file
		if (path.length() > 0) boost::filesystem::remove(boost::filesystem::path(path));
	}
};

//------------------------------------------------------------------------------

class Item;
class Handler;
class Data;

/** Data item is the smallest data unit created by associated Handler.
*	Examples: RGB/depth image, a point cloud, video sequence, tactile array, F/T data, robot trajectory, hierarchies of parts of an image, etc.
*/
class Item {
public:
	friend class Handler;
	friend class Data;

	typedef golem::shared_ptr<Item> Ptr;
	typedef std::multimap<std::string, Ptr> Map;
	typedef std::list<Map::const_iterator> List;

	/** Creates render buffer, the buffer can be shared and allocated on Handler. No default implementation. */
	virtual void createRender() {}

	/** Clones item. */
	virtual Item::Ptr clone() const = 0;

	/** Data handler */
	Handler& getHandler() {
		return handler;
	}
	/** Data handler */
	const Handler& getHandler() const {
		return handler;
	}

	/** Required by interface */
	virtual ~Item() {}

protected:
	/** Data handler */
	Handler& handler;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext) = 0;
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const = 0;

	/** Initialise data item */
	Item(Handler& handler) : handler(handler) {}
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class Handler : public Library {
public:
	friend class Data;

	typedef golem::shared_ptr<Handler> Ptr;
	typedef std::map<std::string, Ptr> Map;

	/** Data handler description */
	class Desc : public Library::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** File operations description */
		File::Desc file;

		/** Set to default */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			Library::Desc::setToDefault();

			libraryPrefix = "GolemData";
			libraryName = "golem data";

			file.setToDefault();
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Library::Desc::assertValid(ac);
		}

		/** Creates the object from the description. */
		virtual Handler::Ptr create(golem::Context &context) const = 0;

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
	};

	/** Construct empty item. */
	virtual Item::Ptr create() const = 0;

	/** Test item interface. */
	template <typename _Interface> bool isItem() const {
		return is<_Interface>(itemTest.get()) != nullptr;
	}

	/** Set file description */
	virtual void setFileDesc(const File::Desc& file) {
		this->file = file;
	}
	/** Get file description */
	virtual File::Desc getFileDesc() const {
		return file;
	}

protected:
	/** Empty item. */
	Item::Ptr itemTest;
	/** File operations description */
	File::Desc file;

	/** Construct handler */
	void create(const Desc& desc);
	/** Initialise handler */
	Handler(golem::Context &context);
};

/** Data bundle - collections of items.
*/
class Data {
public:
	typedef golem::shared_ptr<Data> Ptr;
	typedef std::map<std::string, Ptr> Map;

	/** Data bundle description */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Handler name in xml resource files */
		std::string xmlHandler;
		/** Item name in xml resource files */
		std::string xmlItem;
		/** Label name in xml resource files */
		std::string xmlLabel;
		/** Prefix name in xml resource files */
		std::string xmlPrefix;
		/** Use name in xml resource files */
		std::string xmlName;
		/** Name separator */
		std::string sepName;
		/** Field separator */
		std::string sepField;

		/** File operations description */
		File::Desc file;

		/** Set to default */
		Desc() {
			setToDefault();
		}
		/** Virtual descrutor required */
		virtual ~Desc() {}

		/** Sets the parameters to the default values */
		void setToDefault() {
			xmlHandler = "handler";
			xmlItem = "item";
			xmlLabel = "label";
			xmlPrefix = "prefix";
			xmlName = "use_name";
			sepName = "-";
			sepField = "\t";
			file.setToDefault();
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Assert::valid(xmlHandler.length() > 0, ac, "xmlHandler: invalid name");
			Assert::valid(xmlItem.length() > 0, ac, "xmlItem: invalid name");
			Assert::valid(xmlLabel.length() > 0, ac, "xmlLabel: invalid name");
			Assert::valid(xmlPrefix.length() > 0, ac, "xmlPrefix: invalid name");
			Assert::valid(xmlName.length() > 0, ac, "xmlName: invalid name");
			Assert::valid(!sepName.empty(), ac, "sepName: empty");
			Assert::valid(!sepField.empty(), ac, "sepField: empty");
		}

		/** Load descritpion from xml context. */
		virtual void load(const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual Data::Ptr create(golem::Context &context) const;
	};

	/** Item collection */
	Item::Map itemMap;

	/** Handler name in xml resource files */
	std::string xmlHandler;
	/** Item name in xml resource files */
	std::string xmlItem;
	/** Label name in xml resource files */
	std::string xmlLabel;
	/** Prefix name in xml resource files */
	std::string xmlPrefix;
	/** Use name in xml resource files */
	std::string xmlName;
	/** Name separator */
	std::string sepName;
	/** Field separator */
	std::string sepField;

	/** Load from file */
	virtual void load(const std::string& path, const Handler::Map& handlerMap);
	/** Save to file */
	virtual void save(const std::string& path) const;

	/** Required by interface */
	virtual ~Data() {}

protected:
	/** Program context */
	golem::Context &context;

	/** Data file */
	mutable File dataFile;
	/** File operations description */
	File::Desc file;
	/** Use name */
	bool useName;

	/** Load from xml context */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext, const Handler::Map& handlerMap);
	/** Save to xml context */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	/** Constructs data bundle */
	void create(const Desc& desc);
	/** Initialises data bundle */
	Data(golem::Context &context);
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_PLUGIN_DATA_H_*/
