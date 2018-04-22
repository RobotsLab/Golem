/** @file ContactQueryUA.h
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
#ifndef _GOLEM_DATA_CONTACTQUERYUA_CONTACTQUERYUA_H_
#define _GOLEM_DATA_CONTACTQUERYUA_CONTACTQUERYUA_H_

//------------------------------------------------------------------------------

#include <Golem/Data/ContactQuery/ContactQuery.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

class ItemContactQueryUA;
class HandlerContactQueryUA;

/** Data item representing a collection of contact query densities.
*/
class GOLEM_LIBRARY_DECLDIR ItemContactQueryUA : public ItemContactQuery {
public:
	friend class HandlerContactQueryUA;

	/** Clones item. */
	virtual Item::Ptr clone() const;

	/** Creates render buffer, the buffer can be shared and allocated on Handler */
	virtual void createRender();

protected:
	/** Data handler */
	HandlerContactQueryUA& handler;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	/** Initialise data item */
	ItemContactQueryUA(HandlerContactQueryUA& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerContactQueryUA : public HandlerContactQuery {
public:
	friend class ItemContactQueryUA;

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public HandlerContactQuery::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Set to default */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			golem::data::HandlerContactQuery::Desc::setToDefault();

			//manipulatorDesc.reset(new Manipulator::Desc);
			//plannerDesc.reset(new Planner::Desc);
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			golem::data::HandlerContactQuery::Desc::assertValid(ac);

		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual Handler::Ptr create(golem::Context &context) const;
	};

protected:
	/** Creates render buffer */
	void createRender(const ItemContactQueryUA& item);
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

	/** HandlerPlanner: Sets planner and controllers. */
	virtual void set(const golem::Planner& planner, const ControllerId::Seq& controllerIDSeq);

	/** Construct empty item, accessible only by Data. */
	virtual Item::Ptr create() const;

	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerContactQueryUA(golem::Context &context);
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_DATA_CONTACTQUERYUA_CONTACTQUERYUA_H_*/
