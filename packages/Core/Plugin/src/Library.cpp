/** @file Library.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Plugin/Library.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(golem::Library::Path::Seq::value_type& val, golem::XMLContext* context, bool create) {
	golem::XMLData("library_path", val.library, context);
	golem::XMLData("config_path", val.config, context);
}

//------------------------------------------------------------------------------

std::string Library::Desc::getLibrary() const {
	return path.library.substr(path.library.find(libraryPrefix) == 0 ? libraryPrefix.length() : 0, std::string::npos);
}

std::string Library::Desc::getConfig() const {
	return path.config.substr(path.config.find(libraryPrefix) == 0 ? libraryPrefix.length() : 0, std::string::npos);
}

std::string Library::Desc::getID() const {
	return getLibrary() + NAME_SEP + getConfig();
}

//------------------------------------------------------------------------------

const std::string Library::NAME_SEP = "+";

Library::Library(golem::Context& context) : context(context) {
}

Library::~Library() {
}

void Library::create(const Desc& desc) {
	desc.assertValid(Assert::Context("Library::Desc."));

	path = desc.path;
	type = desc.getLibrary();
	id = type + NAME_SEP + desc.getConfig();
}

//------------------------------------------------------------------------------
