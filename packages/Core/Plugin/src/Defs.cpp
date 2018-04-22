/** @file Defs.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Plugin/Defs.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(ConfigMat34 &val, golem::XMLContext* xmlcontext, bool create) {
	if (!create)
		val.setToDefault();
	
	//golem::XMLData(val.c, xmlcontext, create);
	golem::XMLDataSeq(val.c, "c", xmlcontext, create, golem::REAL_ZERO);
	try {
		golem::XMLData(val.w, xmlcontext, create);
	}
	catch (const golem::MsgXMLParser&) {
		if (create) throw;
	}
	try {
		XMLData("planner_index", val.plannerIndex, xmlcontext, create);
	}
	catch (const golem::MsgXMLParser&) {
		if (create) throw;
	}
}

void golem::XMLData(ConfigMat34::Map::value_type &val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("name", const_cast<std::string&>(val.first), xmlcontext, create);
	golem::XMLData(val.second, xmlcontext, create);
}

//void golem::XMLData(const std::string &attr, golem::U32 &val, golem::XMLContext* context, bool create) {
//	golem::XMLData(attr, val, context, create);
//}

//------------------------------------------------------------------------------

std::string golem::makeString(const char* format, ...) {
	char buf[BUFSIZ];

	va_list argptr;
	va_start(argptr, format);
	(void)golem::vsnprintf(buf, buf + BUFSIZ, format, argptr);
	va_end(argptr);

	return std::string(buf);
}

std::string golem::getDir(const std::string& path) {
	const std::size_t posdir = path.find_last_of("/\\");
	return posdir != std::string::npos ? path.substr(0, posdir + 1) : path.length() > 0 ? path + '/' : std::string("./");
}

std::string golem::getName(const std::string& path) {
	const std::size_t posdir = path.find_last_of("/\\");
	const std::size_t posext = path.find_last_of(".");
	return path.substr(posdir != std::string::npos ? posdir + 1 : 0, posdir == std::string::npos ? posext : posext > posdir ? posext - posdir - 1 : std::string::npos);
}

std::string golem::getExt(const std::string& path) {
	const std::size_t posdir = path.find_last_of("/\\");
	const std::size_t posext = path.find_last_of(".");

	if (posext == std::string::npos || (posdir != std::string::npos && posdir > posext))
		return std::string("");

	std::string ext = path.substr(posext);
	std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
	return ext;
}

bool golem::isExt(const std::string& path, const std::string& ext) {
	return path.length() >= ext.length() && ext == &path[path.length() - ext.length()];
}

//------------------------------------------------------------------------------

void golem::XMLData(Mat34Map::value_type& val, golem::XMLContext* context, bool create) {
	golem::XMLData("name", const_cast<std::string&>(val.first), context, create);
	golem::XMLData(val.second, context, create);
}

//------------------------------------------------------------------------------

void golem::TimeStamp::xmlData(golem::XMLContext* context, bool create) const {
	golem::XMLData("time_stamp", const_cast<golem::SecTmReal&>(timeStamp), context, create);
}

//------------------------------------------------------------------------------

golem::Header::Version golem::Header::getVersionCurrent() const {
	return version;
}

bool golem::Header::isValid() const {
	return valid;
}

void golem::Header::load(const Stream& stream) {
	valid = false;
	const Stream::Position pos = stream.getRead();
	std::string id;
	const std::string& ID = getId();
	stream.read(&id, std::min(ID.length(), (size_t)ID_LENGTH) + 1);
	if (ID.length() <= 0 || ID != id) {
		version.version = VERSION_UNDEF;
		stream.resetRead(pos);
		//printf("default_{version=%u.%u, id=%s}, current_{version=%u.%u, id=%s}\n", (U32)getVersion().major, (U32)getVersion().minor, ID.data(), (U32)version.major, (U32)version.minor, id.data());
		return;
	}
	stream.read(version.version);
	valid = true;
}

void golem::Header::store(Stream& stream) const {
	stream.write(&getId(), ID_LENGTH);
	stream.write(getVersion().version);
}

golem::Header::Header() : version({ VERSION_UNDEF }), valid(false) {
}

//------------------------------------------------------------------------------

