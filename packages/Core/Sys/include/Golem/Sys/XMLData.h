/** @file XMLData.h
 * 
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
#ifndef _GOLEM_SYS_XMLDATA_H_
#define _GOLEM_SYS_XMLDATA_H_

//------------------------------------------------------------------------------

#include <Golem/Sys/XMLParser.h>
#include <Golem/Sys/Context.h>
#include <sstream>
#include <typeinfo>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Reads value from a given context */
template <typename VAL> void XMLGetValue(VAL &val, const XMLContext* context) {
	ASSERT(context)

	std::istringstream iss(context->getValue(), std::istringstream::in);	
	iss >> val;
	if (iss.fail())
		throw MsgXMLParser(Message::LEVEL_ERROR, "XMLGetValue(): Value parse error at: %s", context->getName().c_str());
}

/** Reads string from a given context */
template <> void XMLGetValue(std::string &val, const XMLContext* context);

/** Writes value to a given context */
template <typename VAL> void XMLSetValue(const VAL &val, XMLContext* context) {
	ASSERT(context)

	std::ostringstream oss(std::ostringstream::out);	
	oss << val;
	if (oss.fail())
		throw MsgXMLParser(Message::LEVEL_ERROR, "XMLSetValue(): Value conversion error at: %s", context->getName().c_str());
	
	context->setValue(oss.str());
}

/** Writes string to a given context */
template <> void XMLSetValue(const std::string &val, XMLContext* context);

/** Reads/writes value from/to a given context
*/
template <typename VAL> void XMLData(VAL &val, XMLContext* context, bool create = false) {
	if (create)
		XMLSetValue(val, context);
	else
		XMLGetValue(val, context);
}
//------------------------------------------------------------------------------

/** Reads attribute from a given context */
template <typename VAL> void XMLGetAttribute(const std::string &attr, VAL &val, const XMLContext* context) {
	ASSERT(context)

	std::istringstream iss(context->getAttribute(attr), std::istringstream::in);	
	iss >> val;
	if (iss.fail())
		throw MsgXMLParser(Message::LEVEL_ERROR, "XMLGetAttribute(): Attribute (%s) parse error at: %s %s", typeid(VAL).name(), context->getName().c_str(), attr.c_str());
}

/** Reads string from a given context */
template <> void XMLGetAttribute(const std::string &attr, std::string &val, const XMLContext* context);

/** Writes attribute to a given context */
template <typename VAL> void XMLSetAttribute(const std::string &attr, const VAL &val, XMLContext* context) {
	ASSERT(context)

	std::ostringstream oss(std::ostringstream::out);	
	oss << val;
	if (oss.fail())
		throw MsgXMLParser(Message::LEVEL_ERROR, "XMLSetValue(): Attribute (%s) conversion error at: %s %s", typeid(VAL).name(), context->getName().c_str(), attr.c_str());
	
	context->setAttribute(attr, oss.str());
}

/** Writes string to a given context */
template <> void XMLSetAttribute(const std::string &attr, const std::string &val, XMLContext* context);

/** Reads/writes attribute from/to a given context
*/
template <typename VAL> void XMLData(const std::string &attr, VAL &val, XMLContext* context, bool create = false) {
	if (create)
		XMLSetAttribute(attr, val, context);
	else
		XMLGetAttribute(attr, val, context);
}

//------------------------------------------------------------------------------

/** Reads sequence of values from a given context
*/
template <typename PTR> void XMLGetValue(PTR begin, PTR end, const XMLContext* context, const char* name) {
	ASSERT(context)
	
	std::pair<XMLContext::XMLContextMap::const_iterator, XMLContext::XMLContextMap::const_iterator> range = context->getContextMap().equal_range(name);
	if (range.first == range.second)
		throw MsgXMLParserNameNotFound(Message::LEVEL_ERROR, "XMLGetValue(): Name not found: %s %s", context->getName().c_str(), name);

	for (XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second && begin != end; ++i, ++begin)
		XMLData(*begin, const_cast<XMLContext*>(&i->second), false);

	if (begin != end)
		throw MsgXMLParserIncompleteData(Message::LEVEL_ERROR, "XMLGetValue(): Incomplete data sequence: %s %s", context->getName().c_str(), name);
}
template <typename TYPE, typename PTR> void XMLGetValuePtr(PTR begin, PTR end, const XMLContext* context, const char* name) {
	ASSERT(context)
	
	std::pair<XMLContext::XMLContextMap::const_iterator, XMLContext::XMLContextMap::const_iterator> range = context->getContextMap().equal_range(name);
	if (range.first == range.second)
		throw MsgXMLParserNameNotFound(Message::LEVEL_ERROR, "XMLGetValuePtr(): Name not found: %s %s", context->getName().c_str(), name);

	for (XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second && begin != end; ++i, ++begin) {
		TYPE* ptr = dynamic_cast<TYPE*>(&**begin);
		if (ptr == NULL)
			throw MsgXMLParserInvalidCast(Message::LEVEL_ERROR, "XMLGetValuePtr(): unable to cast a pointer: %s %s", context->getName().c_str(), name);
		XMLData(*ptr, const_cast<XMLContext*>(&i->second), false);
	}

	if (begin != end)
		throw MsgXMLParserIncompleteData(Message::LEVEL_ERROR, "XMLGetValuePtr(): Incomplete data sequence: %s %s", context->getName().c_str(), name);
}


/** Writes sequence of values to a given context
*/
template <typename PTR> void XMLSetValue(PTR begin, PTR end, XMLContext* context, const char* name) {
	ASSERT(context)

	while (begin != end)
		XMLData(*begin++, context->createContext(name), true);
}
template <typename TYPE, typename PTR> void XMLSetValuePtr(PTR begin, PTR end, XMLContext* context, const char* name) {
	ASSERT(context)

	for (;begin != end; ++begin) {
		TYPE* ptr = dynamic_cast<TYPE*>(&**begin);
		if (ptr == NULL)
			throw MsgXMLParserInvalidCast(Message::LEVEL_ERROR, "XMLSetValuePtr(): unable to cast a pointer: %s %s", context->getName().c_str(), name);
		XMLData(const_cast<TYPE&>(*ptr), context->createContext(name), true);
	}
}

/** Reads/writes sequence of values from/to a given context
*/
template <typename PTR> void XMLData(PTR begin, PTR end, XMLContext* context, const char* name, bool create = false) {
	if (create)
		XMLSetValue(begin, end, context, name);
	else
		XMLGetValue(begin, end, context, name);
}
template <typename TYPE, typename PTR> void XMLDataPtr(PTR begin, PTR end, XMLContext* context, const char* name, bool create = false) {
	if (create)
		XMLSetValuePtr<TYPE>(begin, end, context, name);
	else
		XMLGetValuePtr<TYPE>(begin, end, context, name);
}

//------------------------------------------------------------------------------

/** Reads sequence of values from a given context
*/
template <typename SEQ> void XMLGetValue(SEQ &seq, size_t n, const XMLContext* context, const char* name, const typename SEQ::value_type& dflt = typename SEQ::value_type()) {
	ASSERT(context)
	
	std::pair<XMLContext::XMLContextMap::const_iterator, XMLContext::XMLContextMap::const_iterator> range = context->getContextMap().equal_range(name);
	if (range.first == range.second)
		throw MsgXMLParserNameNotFound(Message::LEVEL_ERROR, "XMLGetValue(): Name not found: %s %s", context->getName().c_str(), name);

	for (XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second && n-- > 0; ++i) {
		typename SEQ::value_type val = dflt;
		XMLData(val, const_cast<XMLContext*>(&i->second), (bool)false);
		seq.insert(seq.end(), val);
	}
}

/** Writes sequence of values to a given context
*/
template <typename SEQ> void XMLSetValue(const SEQ &seq, size_t n, XMLContext* context, const char* name) {
	ASSERT(context)
	
	for (typename SEQ::const_iterator i = seq.begin(); i != seq.end() && n-- > 0; ++i) {
		typedef typename SEQ::value_type VAL;
		XMLData(const_cast<VAL&>(*i), context->createContext(name), true);
	}
}

/** Reads/writes sequence of values from/to a given context
*/
template <typename SEQ> void XMLData(SEQ &seq, size_t n, XMLContext* context, const char* name, bool create = false, const typename SEQ::value_type& dflt = typename SEQ::value_type()) {
	if (create)
		XMLSetValue(seq, n, context, name);
	else
		XMLGetValue(seq, n, context, name, dflt);
}

//------------------------------------------------------------------------------

template <typename PTR> void XMLData(PTR begin, PTR end, const char* attr, XMLContext* context, bool create = false) {
	for (PTR i = begin; i != end; ++i) {
		char str[16];
		sprintf(str, "%s%d", attr, int((i - begin) + 1));
		XMLData(str, *i, context, create);
	}
}

template <typename SEQ> void XMLDataSeq(SEQ &seq, const char* attr, XMLContext* context, bool create = false, const typename SEQ::value_type& dflt = typename SEQ::value_type()) {
	ASSERT(context)
	
	U32 dim = create ? (U32)seq.size() : (U32)-1;
	try {
		XMLData("dim", dim, context, create);
	}
	catch (const MsgXMLParser&) {
	}

	if (!create) seq.clear();
	for (U32 i = 0; i < dim; ++i) {
		char str[16];
		sprintf(str, "%s%d", attr, int(i + 1));

		typename SEQ::value_type val = dflt;
		try {
			if (create) val = seq[i];
			XMLData(str, val, context, create);
			if (!create) seq.insert(seq.end(), val);
		}
		catch (const MsgXMLParser& msg) {
			if (i == 0 || i < dim && dim != -1)
				throw msg;
			else
				break;
		}
	}
}

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
template <typename _Seq> void XMLGetValue(_Seq &seq, const std::string &name, const std::string &attr, golem::XMLContext* context, bool create = false) {
	std::pair<typename XMLContext::XMLContextMap::const_iterator, typename XMLContext::XMLContextMap::const_iterator> range = context->getContextMap().equal_range(name);
	for (typename XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second; ++i) {
		try {
			typename _Seq::value_type val;
			XMLGetAttribute(attr, val, const_cast<XMLContext*>(&i->second));
			seq.push_back(val);
		}
		catch (const golem::MsgXMLParserAttributeNotFound&) {}
	}
}

//------------------------------------------------------------------------------

template <const U32 _FLAG> void XMLDataFlag(U32& val, const char* _name, XMLContext* context, bool create) {
	bool bVal = (val & _FLAG) > 0;
	XMLData(_name, bVal, context, create);
	val &= ~_FLAG;
	if (bVal)
		val |= _FLAG;
}

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_SYS_XMLDATA_H_*/
