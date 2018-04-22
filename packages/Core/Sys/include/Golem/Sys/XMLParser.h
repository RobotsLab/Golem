/** @file XMLParser.h
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
#ifndef _GOLEM_SYS_XMLPARSER_H_
#define _GOLEM_SYS_XMLPARSER_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Pointers.h>
#include <Golem/Sys/Stream.h>
#include <Golem/Sys/Message.h>
#include <expat_external.h>
#include <string>
#include <map>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgXMLParser, Message)
MESSAGE_DEF(MsgXMLParserInvalidDesc, MsgXMLParser)
MESSAGE_DEF(MsgXMLParserBufferAlloc, MsgXMLParser)
MESSAGE_DEF(MsgXMLParserStatusErr, MsgXMLParser)
MESSAGE_DEF(MsgXMLParserStatusSuspendErr, MsgXMLParser)
MESSAGE_DEF(MsgXMLParserNullContext, MsgXMLParser)
MESSAGE_DEF(MsgXMLParserInvalidName, MsgXMLParser)
MESSAGE_DEF(MsgXMLParserInvalidCast, MsgXMLParser)
MESSAGE_DEF(MsgXMLParserNameNotFound, MsgXMLParser)
MESSAGE_DEF(MsgXMLParserIncompleteData, MsgXMLParser)
MESSAGE_DEF(MsgXMLParserAttributeNotFound, MsgXMLParser)

//------------------------------------------------------------------------------

class XMLParser;

/** XMLContext
*/
class XMLContext {
public:
	typedef std::multimap<std::string, XMLContext> XMLContextMap;
	typedef std::pair<std::string, XMLContext> XMLContextPair;
	typedef std::map<std::string, std::string> XMLAttributeMap;
	typedef std::pair<std::string, std::string> XMLAttributePair;
	//typedef XMLContextMap::const_iterator XMLContextIterator;
	//typedef std::pair<XMLContextIterator, XMLContextIterator> XMLContextRange;

	friend class XMLParser;
	
protected:
	std::string name;
	std::string value;
	XMLAttributeMap attributeMap;
	XMLContextMap contextMap;
	
	XMLParser *parser;
	XMLContext* parent;

	XMLContext() : parser(NULL), parent(NULL) {
	}

	XMLContext(XMLParser *parser, XMLContext* parent, const std::string& name) : parser(parser), parent(parent), name(name) {
	}

public:
	/** Creates context for a give name */
	XMLContext* createContext(const char* name);
	
	/** Tests if the context is the leaf context */
	bool isLeaf() const {
		return contextMap.empty();
	}

	/** Returns context name */
	std::string getName() const;

	/** Returns context value */
	const std::string& getValue() const;

	/** Sets context value */
	void setValue(const std::string& value);
	
	/** Returns context attribute */
	const std::string& getAttribute(const std::string& attribute) const;

	/** sets context attribute */
	void setAttribute(const std::string& attribute, const std::string& value);
	
	/** Get context parent */
	XMLContext* getContextParent() {
		return parent;
	}
	const XMLContext* getContextParent() const {
		return parent;
	}

	/** Get attribute map */
	XMLAttributeMap& getAttributeMap() {
		return attributeMap;
	}
	const XMLAttributeMap& getAttributeMap() const {
		return attributeMap;
	}

	/** Get context map */
	XMLContextMap& getContextMap() {
		return contextMap;
	}
	const XMLContextMap& getContextMap() const {
		return contextMap;
	}

	/** Get first context for a sequence of names, create if it does not exist and create==true otherwise return NULL. */
	XMLContext* getContextFirst(const char* name, bool create = false) const;
};

//------------------------------------------------------------------------------

/** XMLParser
*/
class XMLParser : public Serializable {
public:
	typedef shared_ptr<XMLParser> Ptr;
	friend class Desc;

	static const char *sOpen;
	static const char *sClose;
	static const char *sEnd;
	static const char *sType;
	static const char *sQuot;
	static const char *sSpace;
	static const char *sEq;
	static const char *sXML;
	static const char *sVer;
	static const char *sEnc;

	/** Context description */
	class Desc {
	public:
		/** Buffer size */
		size_t buffSize;
		/** Name separator */
		char cSeparator;
		/** XML version */
		std::string sVersion;
		/** XML encoding */
		std::string sEncoding;
		/** Depth sequence */
		std::string sDepth;
		/** New line sequence */
		std::string sNewLine;

		/** Constructs Logger description. */
		Desc() {
			setToDefault();
		}
		
		/** Destructor should be virtual */
		virtual ~Desc() {}
		
		/** Creates Context from the description. */
		virtual XMLParser::Ptr create() const {
			XMLParser::Ptr pXMLParser(new XMLParser());
			pXMLParser->create(*this); // throws
			return pXMLParser;
		}
		
		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			buffSize = 10000;
			cSeparator = ' ';
			sVersion = "1.0";
			sEncoding = "utf-8";
			sDepth = "  ";
#ifdef WIN32
			sNewLine = "\r\n";
#else
			sNewLine = "\n";
#endif
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (buffSize <= 0)
				return false;
			if (sVersion.empty() || sEncoding.empty())
				return false;

			return true;
		}
	};
	
protected:
	friend class XMLContext;
	
	static void XMLCALL startElementHandler(void *userData, const XML_Char *name, const XML_Char **atts);
	static void XMLCALL endElementHandler(void *userData, const XML_Char *name);
	static void XMLCALL characterDataHandler(void *userData, const XML_Char *s, int len);

	/** Buffer size */
	size_t buffSize;
	/** Name separator */
	char cSeparator;
	/** XML version */
	std::string sVersion;
	/** XML encoding */
	std::string sEncoding;
	/** Depth sequence */
	std::string sDepth;
	/** New line sequence */
	std::string sNewLine;
	
	XMLContext root;
	XMLContext* current;

	/** Stores context to the specified stream */
	void store(Stream &stream, const XMLContext* xmlContext, size_t depth) const;

	/** Open */
	void open(std::string &str, const std::string& name, const XMLContext::XMLAttributeMap& attributeMap) const;

	/** Close */
	void close(std::string &str, const std::string& name) const;

	/** Creates context from description */
	void create(const Desc &desc);

	/** Default constructror sets data to the default values */
	XMLParser();

public:
	/** Destructor is inaccesible */
	~XMLParser();
	
	/** Create parser with default settings and load specified xml file */
	static Ptr load(const std::string& path);

	/** Loads data from the specified stream */
	void load(const Stream &stream);
	
	/** Stores data to the specified stream */
	void store(Stream &stream) const;

	/** Initialises parser */
	void clear();

	/** Get root context */
	XMLContext* getContextRoot() {
		return &root;
	}
	const XMLContext* getContextRoot() const {
		return &root;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_SYS_XMLPARSER_H_*/
