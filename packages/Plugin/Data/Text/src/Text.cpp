/** @file Text.cpp
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
#include <Golem/Data/Text/Text.h>
#include <fstream>

using namespace golem;
using namespace golem::data;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new HandlerText::Desc();
}

//------------------------------------------------------------------------------

golem::data::ItemText::ItemText(HandlerText& handler) : Item(handler), handler(handler), textFile(handler.file) {
}

Item::Ptr golem::data::ItemText::clone() const {
	throw Message(Message::LEVEL_ERROR, "ItemText::clone(): not implemented");
}

void golem::data::ItemText::load(const std::string& prefix, const golem::XMLContext* xmlcontext) {
	// cloud
	std::string textSuffix;
	golem::XMLData("text", textSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (textSuffix.length() > 0) {
		textFile.load(prefix + textSuffix, [&] (const std::string& path) {
			std::ifstream file(path);
			if (!file.good())
				throw golem::Message(golem::Message::LEVEL_ERROR, "ItemText::load(): unable to read from %s", path.c_str());
			file >> textStream.rdbuf();
		});
	}
}

void golem::data::ItemText::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	// text xml
	std::string textSuffix = textStream.str().length() > 0 ? handler.textSuffix : "";
	golem::XMLData("text", textSuffix, xmlcontext, true);
	if (textSuffix.length() > 0) {
		textFile.setModified(true); // always save
		textFile.save(prefix + textSuffix, [=](const std::string& path) {
			std::ofstream file(path);
			file << textStream.rdbuf();
			if (!file.good())
				throw golem::Message(golem::Message::LEVEL_ERROR, "ItemText::save(): unable to write to %s", path.c_str());
		});
	}
	else
		textFile.remove();
}

//------------------------------------------------------------------------------

std::iostream& golem::data::ItemText::getTextStream() {
	return textStream;
}

const std::iostream& golem::data::ItemText::getTextStream() const {
	return textStream;
}

//------------------------------------------------------------------------------

void golem::data::HandlerText::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	golem::data::Handler::Desc::load(context, xmlcontext);
	
	golem::XMLData("suffix", textSuffix, xmlcontext->getContextFirst("text", false), false);
}

golem::data::Handler::Ptr golem::data::HandlerText::Desc::create(golem::Context &context) const {
	Handler::Ptr handler(new HandlerText(context));
	to<HandlerText>(handler.get())->create(*this);
	return handler;
}

//------------------------------------------------------------------------------

golem::data::HandlerText::HandlerText(golem::Context &context) : Handler(context) {
}

void golem::data::HandlerText::create(const Desc& desc) {
	golem::data::Handler::create(desc);

	textSuffix = desc.textSuffix;
}

//------------------------------------------------------------------------------

golem::data::Item::Ptr golem::data::HandlerText::create() const {
	return Item::Ptr(new ItemText(*const_cast<HandlerText*>(this))); // safe: no modification here - only passing reference
}

//------------------------------------------------------------------------------
