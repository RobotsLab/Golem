/** @file ContactQueryUA.cpp
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
#include <Golem/Math/Data.h>
#include <Golem/Tools/Import.h>
#include <Golem/Plugin/Ctrl.h>
#include <Golem/Data/ContactQueryUA/ContactQueryUA.h>
#include <boost/algorithm/string.hpp>

using namespace golem;
using namespace golem::data;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new HandlerContactQueryUA::Desc();
}

//------------------------------------------------------------------------------

golem::data::ItemContactQueryUA::ItemContactQueryUA(HandlerContactQueryUA& handler) :
	ItemContactQuery(handler), handler(handler)
{}

Item::Ptr golem::data::ItemContactQueryUA::clone() const {
	throw Message(Message::LEVEL_ERROR, "ItemContactQueryUA::clone(): not implemented");
}

void golem::data::ItemContactQueryUA::createRender() {
	handler.createRender(*this);
}

void golem::data::ItemContactQueryUA::load(const std::string& prefix, const golem::XMLContext* xmlcontext) {
	ItemContactQuery::load(prefix, xmlcontext);

}

void golem::data::ItemContactQueryUA::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	ItemContactQuery::save(prefix, xmlcontext);

}

//------------------------------------------------------------------------------

void golem::data::HandlerContactQueryUA::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	golem::data::HandlerContactQuery::Desc::load(context, xmlcontext);

}

golem::data::Handler::Ptr golem::data::HandlerContactQueryUA::Desc::create(golem::Context &context) const {
	Handler::Ptr handler(new HandlerContactQueryUA(context));
	to<HandlerContactQueryUA>(handler.get())->create(*this);
	return handler;
}

//------------------------------------------------------------------------------

golem::data::HandlerContactQueryUA::HandlerContactQueryUA(golem::Context &context) :
	HandlerContactQuery(context)
{}

void golem::data::HandlerContactQueryUA::create(const Desc& desc) {
	golem::data::HandlerContactQuery::create(desc);

}

//------------------------------------------------------------------------------

void golem::data::HandlerContactQueryUA::set(const golem::Planner& planner, const ControllerId::Seq& controllerIDSeq) {
	HandlerContactQuery::set(planner, controllerIDSeq);

	manipulator->getDesc().controlToCommand = [=](const Manipulator::ControlCoord& control, golem::Controller::State& command) {
		// PISA Soft Hand
		command.cpos[manipulator->getHandInfo().getJoints().begin()] = control[0]; // first joint is mapped onto synergy
	};
}

//------------------------------------------------------------------------------

golem::data::Item::Ptr golem::data::HandlerContactQueryUA::create() const {
	return Item::Ptr(new ItemContactQueryUA(*const_cast<HandlerContactQueryUA*>(this))); // safe: no modification here - only passing reference
}

//------------------------------------------------------------------------------

void golem::data::HandlerContactQueryUA::createRender(const ItemContactQueryUA& item) {
	HandlerContactQuery::createRender(item);

}

void golem::data::HandlerContactQueryUA::render() const {
	HandlerContactQuery::render();

}

void golem::data::HandlerContactQueryUA::customRender() const {
	HandlerContactQuery::customRender();

}

//------------------------------------------------------------------------------

void golem::data::HandlerContactQueryUA::mouseHandler(int button, int state, int x, int y) {
	HandlerContactQuery::mouseHandler(button, state, x, y);

}

void golem::data::HandlerContactQueryUA::motionHandler(int x, int y) {
	HandlerContactQuery::motionHandler(x, y);

}

void golem::data::HandlerContactQueryUA::keyboardHandler(int key, int x, int y) {
	HandlerContactQuery::keyboardHandler(key, x, y);

}

//------------------------------------------------------------------------------
