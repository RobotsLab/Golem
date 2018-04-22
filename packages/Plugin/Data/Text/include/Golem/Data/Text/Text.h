/** @file Text.h
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
#ifndef _GOLEM_DATA_TEXT_TEXT_H_
#define _GOLEM_DATA_TEXT_TEXT_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Library.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Plugin/Text.h>
#include <sstream>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

class ItemText;
class HandlerText;

/** Data item representing text file.
*/
class GOLEM_LIBRARY_DECLDIR ItemText : public Item, public Text {
public:
	friend class HandlerText;

	/** Text file */
	mutable File textFile;
	/** Text stream */
	std::stringstream textStream;

	/** Clones item. */
	virtual Item::Ptr clone() const;

	/** Text: access iostream. */
	virtual std::iostream& getTextStream();
	/** Text: access iostream. */
	virtual const std::iostream& getTextStream() const;

protected:
	/** Data handler */
	HandlerText& handler;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	/** Initialise data item */
	ItemText(HandlerText& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerText : public Handler {
public:
	friend class ItemText;

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Handler::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Text suffix */
		std::string textSuffix;

		/** Set to default */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			golem::data::Handler::Desc::setToDefault();

			textSuffix = ".txt";
		}
		
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			golem::data::Handler::Desc::assertValid(ac);

			Assert::valid(textSuffix.length() > 0, ac, "textSuffix: empty");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual Handler::Ptr create(golem::Context &context) const;
	};

protected:
	/** Cloud suffix */
	std::string textSuffix;

	/** Construct empty item, accessible only by Data. */
	virtual Item::Ptr create() const;
	
	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerText(golem::Context &context);
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_DATA_TEXT_TEXT_H_*/
