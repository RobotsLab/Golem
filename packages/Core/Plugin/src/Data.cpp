/** @file Data.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sys/XMLData.h>
#include <Golem/Plugin/Data.h>

using namespace golem;
using namespace golem::data;

//------------------------------------------------------------------------------

void golem::data::File::Desc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("delete_if_moved", deleteIfMoved, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("delete_if_unlinked", deleteIfUnlinked, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("delete_if_temporary", deleteIfTemporary, const_cast<golem::XMLContext*>(xmlcontext));
}

//------------------------------------------------------------------------------

void golem::data::Handler::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	context.initModule();
	file.load(xmlcontext);
}

golem::data::Handler::Handler(golem::Context &context) : Library(context) {
}

void golem::data::Handler::create(const Desc& desc) {
	Library::create(desc);

	itemTest = create();
	file = desc.file;
}

//------------------------------------------------------------------------------

void golem::data::Data::Desc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("xml_handler", xmlHandler, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("xml_item", xmlItem, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("xml_label", xmlLabel, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("xml_prefix", xmlPrefix, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("xml_name", xmlName, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("sep_name", sepName, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("sep_field", sepField, const_cast<golem::XMLContext*>(xmlcontext));
	file.load(xmlcontext);
}

golem::data::Data::Ptr golem::data::Data::Desc::create(golem::Context &context) const {
	Data::Ptr data(new Data(context));
	data->create(*this);
	return data;
}

//------------------------------------------------------------------------------

golem::data::Data::Data(golem::Context &context) : context(context) {
}

void golem::data::Data::create(const Desc& desc) {
	desc.assertValid(Assert::Context("Data::Desc."));

	xmlHandler = desc.xmlHandler;
	xmlItem = desc.xmlItem;
	xmlLabel = desc.xmlLabel;
	xmlPrefix = desc.xmlPrefix;
	xmlName = desc.xmlName;
	sepName = desc.sepName;
	sepField = desc.sepField;
	file = desc.file;
	useName = true;
	dataFile.setDesc(file);
}

void golem::data::Data::load(const std::string& prefix, const golem::XMLContext* xmlcontext, const Handler::Map& handlerMap) {
	// items
	itemMap.clear();
	auto range = xmlcontext->getContextMap().equal_range(xmlItem);
	for (golem::XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second; ++i) {
		try {
			// try to load the specified handle first
			std::string handler;
			golem::XMLData(xmlHandler, handler, const_cast<golem::XMLContext*>(&i->second), false);
			Handler::Map::const_iterator ptr = handlerMap.find(handler);
			if (ptr == handlerMap.end()) {
				// if failed, extract handle name
				std::string handlerType = handler.substr(0, handler.find(Library::NAME_SEP));
				// default handler
				std::string handlerDefault = handlerType + Library::NAME_SEP + handlerType;
				ptr = handlerMap.find(handlerDefault);
				if (ptr == handlerMap.end()) {
					context.warning("golem::data::Data::load(): unknown handler %s\n", handler.c_str());
					continue;
				}
				else
					context.info("golem::data::Data::load(): unable to find handler %s, using %s\n", handler.c_str(), handlerDefault.c_str());
			}

			Item::Map::value_type val;
			golem::XMLData(xmlLabel, const_cast<std::string&>(val.first), const_cast<golem::XMLContext*>(&i->second), false);
			val.second = ptr->second->create();

			std::string name;
			golem::XMLData(xmlPrefix, name, const_cast<golem::XMLContext*>(&i->second), false);

			val.second->load(prefix + name, &i->second);
			itemMap.insert(val);
		}
		catch (const golem::Message& msg) {
			context.write(msg);
		}
		catch (const std::exception& ex) {
			context.error("golem::data::Data::load(): %s\n", ex.what());
		}
	}
}

void golem::data::Data::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	// items
	U32 n = 0;
	for (Item::Map::const_iterator i = itemMap.begin(), j = i; i != itemMap.end(); ++i, ++n) {
		if (j->first.compare(i->first) != 0) {
			n = 0;
			j = i;
		}

		try {
			golem::XMLContext* context = xmlcontext->createContext(xmlItem.c_str());
		
			golem::XMLData(xmlLabel, const_cast<std::string&>(i->first), context, true);
			golem::XMLData(xmlHandler, const_cast<std::string&>(i->second->getHandler().getID()), context, true);

			std::string name = makeString("%s%s%s%s%s%u", sepName.c_str(), xmlItem.c_str(), sepName.c_str(), i->first.c_str(), sepName.c_str(), n + 1);
			golem::XMLData(xmlPrefix, name, context, true);

			i->second->save(prefix + name, context);
		}
		catch (const golem::Message& msg) {
			context.write(msg);
		}
		catch (const std::exception& ex) {
			context.error("golem::data::Data::save(): %s\n", ex.what());
		}
	}
}

void golem::data::Data::load(const std::string& path, const Handler::Map& handlerMap) {
	// Load xml context and data
	dataFile.load(path, [&] (const std::string& path) {
		XMLParser::Ptr pParser = XMLParser::load(path);
		try {
			golem::XMLData(xmlName.c_str(), useName, pParser->getContextRoot()->getContextFirst("golem data", false), false);
		}
		catch (const golem::MsgXMLParserAttributeNotFound&) {}
		load(useName ? getDir(path) + getName(path) : getDir(path), pParser->getContextRoot()->getContextFirst("golem data", false), handlerMap);
	});
}

void golem::data::Data::save(const std::string& path) const {
	// Always save xml context and data, remove old file
	dataFile.setModified(true);
	dataFile.save(path, [&] (const std::string& path) {
		// Create XML parser
		XMLParser::Ptr pParser = XMLParser::Desc().create();
		// Create xml context and save data
		golem::mkdir(path.c_str()); // make sure that the directory exists
		save(getDir(path) + getName(path), pParser->getContextRoot()->getContextFirst("golem data", true));
		FileWriteStream fws(path.c_str());
		pParser->store(fws);
	});
}

//------------------------------------------------------------------------------
