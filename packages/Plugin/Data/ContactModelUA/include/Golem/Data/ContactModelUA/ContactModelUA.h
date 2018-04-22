/** @file ContactModel.h
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
#ifndef _GOLEM_DATA_CONTACTMODELUA_CONTACTMODELUA_H_
#define _GOLEM_DATA_CONTACTMODELUA_CONTACTMODELUA_H_

//------------------------------------------------------------------------------

#include <Golem/Data/ContactModel/ContactModel.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

class ItemContactModelUA;
class HandlerContactModelUA;

/** Data item representing contact training data.
*/
class GOLEM_LIBRARY_DECLDIR ItemContactModelUA : public ItemContactModel {
public:
	friend class HandlerContactModelUA;

	/** Clones item. */
	virtual Item::Ptr clone() const;

	/** Creates render buffer, the buffer can be shared and allocated on Handler */
	virtual void createRender();

protected:
	/** Data handler */
	HandlerContactModelUA& handler;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	/** Initialise data item */
	ItemContactModelUA(HandlerContactModelUA& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerContactModelUA : public HandlerContactModel {
public:
	friend class ItemContactModelUA;

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public HandlerContactModel::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Set to default */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			golem::data::HandlerContactModel::Desc::setToDefault();

			//configurationDesc.reset(new Configuration::Desc);
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			golem::data::HandlerContactModel::Desc::assertValid(ac);

		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual Handler::Ptr create(golem::Context &context) const;
	};

protected:
	/** Creates render buffer */
	void createRender(const ItemContactModelUA& item);
	/** golem::UIRenderer: Render on output device. */
	virtual void render() const;
	/** golem::UIRenderer: Render on output device. */
	virtual void customRender() const;

	/** golem::UIRenderer: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	/** golem::UIRenderer: Mouse motion handler. */
	virtual void motionHandler(int x, int y);
	/** golem::UIRenderer: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);

	/** Transform: Transform input items */
	virtual Item::Ptr transform(const Item::List& input);

	/** HandlerPlanner: Sets planner and controllers. */
	virtual void set(const golem::Planner& planner, const ControllerId::Seq& controllerIDSeq);

	/** Construct empty item, accessible only by Data. */
	virtual Item::Ptr create() const;

	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerContactModelUA(golem::Context &context);
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_DATA_CONTACTMODELUA_CONTACTMODELUA_H_*/
