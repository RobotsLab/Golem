/** @file XMLParser.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sys/XMLParser.h>
#include <expat.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

std::string XMLContext::getName() const {
	std::string name;
	if (parent != NULL) {
		name = parent->getName();
		name.append(XMLParser::sSpace);
	}
	name.append(this->name);
	return name;
}

const std::string& XMLContext::getValue() const {
	return value;
}

void XMLContext::setValue(const std::string& value) {
	this->value = value;
}

const std::string& XMLContext::getAttribute(const std::string& attribute) const {
	XMLAttributeMap::const_iterator pos = attributeMap.find(attribute);
	if (pos == attributeMap.end())
		throw MsgXMLParserAttributeNotFound(Message::LEVEL_ERROR, "XMLContext::getAttribute(): Attribute not found: %s %s", getName().c_str(), attribute.c_str());
	return pos->second;
}

void XMLContext::setAttribute(const std::string& attribute, const std::string& value) {
	attributeMap.insert(XMLAttributePair(attribute, value));
}

XMLContext* XMLContext::createContext(const char* name) {
	if (name == NULL || *name == '\0')
		throw MsgXMLParserInvalidName(Message::LEVEL_ERROR, "XMLContext::createContext(): Invalid name at: %s", getName().c_str());
	
	return &contextMap.insert(XMLContextPair(name, XMLContext(parser, this, name)))->second;
}

XMLContext* XMLContext::getContextFirst(const char* name, bool create) const {
	if (name == NULL)
		throw MsgXMLParserInvalidName(Message::LEVEL_ERROR, "XMLContext::getContextFirst(): Invalid name at: %s", getName().c_str());
	
	const char* p0 = name;
	const char* p1 = name;
	XMLContext* result = const_cast<XMLContext*>((const XMLContext*)this);
	
	for (;;) {
		while (*p1 != '\0' && *p1 != parser->cSeparator) ++p1;
		if (p0 == p1)
			throw MsgXMLParserInvalidName(Message::LEVEL_ERROR, "XMLContext::getContextFirst(): Invalid name at: %s", result->getName().c_str());

		const std::string key(p0, p1 - p0);
		XMLContextMap::iterator i = result->contextMap.find(key);
		if (i != result->contextMap.end())
			result = &i->second;
		else if (create)
			result = &result->contextMap.insert(XMLContextPair(key, XMLContext(parser, result, key)))->second;
		else
			throw MsgXMLParserNameNotFound(Message::LEVEL_ERROR, "XMLContext::getContextFirst(): Name not found: %s %s", result->getName().c_str(), key.c_str());
		
		if (*p1 == '\0')
			break;
		p0 = ++p1;
	};

	return result;
}

//------------------------------------------------------------------------------

const char *XMLParser::sOpen = "<";
const char *XMLParser::sClose = ">";
const char *XMLParser::sEnd = "/";
const char *XMLParser::sType = "?";
const char *XMLParser::sQuot = "\"";
const char *XMLParser::sSpace = " ";
const char *XMLParser::sEq = "=";
const char *XMLParser::sXML = "xml";
const char *XMLParser::sVer = "version";
const char *XMLParser::sEnc = "encoding";

void XMLParser::create(const Desc &desc) {
	if (!desc.isValid())
		throw MsgXMLParserInvalidDesc(Message::LEVEL_CRIT, "XMLParser::create(): Invalid description");

	buffSize = desc.buffSize;
	cSeparator = desc.cSeparator;
	sVersion = desc.sVersion;
	sEncoding = desc.sEncoding;
	sDepth = desc.sDepth;
	sNewLine = desc.sNewLine;
}

XMLParser::XMLParser() {
	root.parser = this;
	root.parent = NULL;
	clear();
}

XMLParser::~XMLParser() {
}

//------------------------------------------------------------------------------

void XMLCALL XMLParser::startElementHandler(void *userData, const XML_Char *name, const XML_Char **atts) {
	XMLParser *pXMLParser = (XMLParser*)userData;
	
	pXMLParser->current = pXMLParser->current->createContext(name);

	for (size_t i = 0; atts[i] != NULL; i += 2)
		pXMLParser->current->getAttributeMap().insert(
			XMLContext::XMLAttributePair(std::string(atts[i]), std::string(atts[i + 1]))
		);
}

void XMLCALL XMLParser::endElementHandler(void *userData, const XML_Char *name) {
	XMLParser *pXMLParser = (XMLParser*)userData;
	pXMLParser->current = pXMLParser->current->getContextParent();
}

void XMLCALL XMLParser::characterDataHandler(void *userData, const XML_Char *s, int len) {
	XMLParser *pXMLParser = (XMLParser*)userData;
	pXMLParser->current->setValue(std::string(s, len));
}

XMLParser::Ptr XMLParser::load(const std::string& path) {
	// Create XML parser and load configuration file
	Desc parserDesc;
	Ptr pParser = parserDesc.create();
	// Load config
	FileReadStream fs(path.c_str());
	pParser->load(fs);
	return pParser;
}

void XMLParser::load(const Stream &stream) {
	XML_Parser parser = XML_ParserCreate(NULL);

	XML_SetUserData(parser, this);
	XML_SetElementHandler(parser, startElementHandler, endElementHandler);
	XML_SetCharacterDataHandler(parser, characterDataHandler);

	for (;;) {
		void *buff = XML_GetBuffer(parser, (int)buffSize);
		if (buff == NULL)
			throw MsgXMLParserBufferAlloc(Message::LEVEL_CRIT, "XMLParser::load(): Unable to allocate Expat buffer");

		try {
			stream.read((char*)buff, buffSize);
		}
		catch (golem::MsgStream&) {
		}
		const size_t bytesRead = stream.lastBytes();
		const bool isFinal = bytesRead <= 0;

		switch (XML_ParseBuffer(parser, (int)bytesRead, isFinal)) {
		case XML_STATUS_ERROR:
			//XML_ErrorString(XML_GetErrorCode(parser));
			//XML_GetCurrentLineNumber(parser));
			throw MsgXMLParserStatusErr(Message::LEVEL_CRIT, "XMLParser::load(): Expat status error");
		case XML_STATUS_SUSPENDED:
			throw MsgXMLParserStatusSuspendErr(Message::LEVEL_CRIT, "XMLParser::load(): Expat status suspended error");
		default:
			break;
		}
		
		if (isFinal)
			break;
	}
}

void XMLParser::store(Stream &stream, const XMLContext* xmlContext, size_t depth) const {
	std::string head;
	for (size_t i = 0; i < depth; ++i)
		head.append(sDepth);

	for (XMLContext::XMLContextMap::const_iterator i = xmlContext->getContextMap().begin(); i != xmlContext->getContextMap().end(); ++i) {
		std::string str;
		
		str.append(head);
		open(str, i->first, i->second.getAttributeMap());
		if (!i->second.isLeaf())
			str.append(sNewLine);
		stream.write(str.c_str(), str.length());
	
		if (i->second.isLeaf()) {
			if (!i->second.getValue().empty())
				stream.write(i->second.getValue().c_str(), i->second.getValue().length());
		}
		else {
			store(stream, &i->second, depth + 1);
		}

		str.clear();
		if (!i->second.isLeaf())
			str.append(head);
		close(str, i->first);
		str.append(sNewLine);
		stream.write(str.c_str(), str.length());
	}
}

void XMLParser::clear() {
	root.setValue("");
	root.getAttributeMap().clear();
	root.getContextMap().clear();
	current = &root;
}

void XMLParser::open(std::string &str, const std::string& name, const XMLContext::XMLAttributeMap& attributeMap) const {
	str.append(sOpen);
	str.append(name);
	
	for (XMLContext::XMLAttributeMap::const_iterator i = attributeMap.begin(); i != attributeMap.end(); ++i) {
		str.append(sSpace);
		str.append(i->first);
		str.append(sEq);
		str.append(sQuot);
		str.append(i->second);
		str.append(sQuot);
	}
	
	str.append(sClose);
}

void XMLParser::close(std::string &str, const std::string& name) const {
	str.append(sOpen);
	str.append(sEnd);
	str.append(name);
	str.append(sClose);
}

void XMLParser::store(Stream &stream) const {
	std::string str;

	// header
	str.append(sOpen);
	str.append(sType);
	str.append(sXML);
	str.append(sSpace);
	str.append(sVer);
	str.append(sEq);
	str.append(sQuot);
	str.append(sVersion);
	str.append(sQuot);
	str.append(sSpace);
	str.append(sEnc);
	str.append(sEq);
	str.append(sQuot);
	str.append(sEncoding);
	str.append(sQuot);
	str.append(sType);
	str.append(sClose);
	str.append(sNewLine);
	str.append(sNewLine);
	stream.write(str.c_str(), str.length());
	
	store(stream, &root, 0);
}

//------------------------------------------------------------------------------
