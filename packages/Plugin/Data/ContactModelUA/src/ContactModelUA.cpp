/** @file ContactModel.cpp
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
#include <Golem/Data/ContactModelUA/ContactModelUA.h>
#include <boost/algorithm/string.hpp>

using namespace golem;
using namespace golem::data;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new HandlerContactModelUA::Desc();
}

//------------------------------------------------------------------------------

golem::data::ItemContactModelUA::ItemContactModelUA(HandlerContactModelUA& handler) : ItemContactModel(handler), handler(handler) {
}

Item::Ptr golem::data::ItemContactModelUA::clone() const {
	throw Message(Message::LEVEL_ERROR, "ItemContactModelUA::clone(): not implemented");
}

void golem::data::ItemContactModelUA::createRender() {
	handler.createRender(*this);
}

void golem::data::ItemContactModelUA::load(const std::string& prefix, const golem::XMLContext* xmlcontext) {
	ItemContactModel::load(prefix, xmlcontext);

}

void golem::data::ItemContactModelUA::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	ItemContactModel::save(prefix, xmlcontext);

}

//------------------------------------------------------------------------------

void golem::data::HandlerContactModelUA::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	golem::data::HandlerContactModel::Desc::load(context, xmlcontext);

}

golem::data::Handler::Ptr golem::data::HandlerContactModelUA::Desc::create(golem::Context &context) const {
	Handler::Ptr handler(new HandlerContactModelUA(context));
	to<HandlerContactModelUA>(handler.get())->create(*this);
	return handler;
}

//------------------------------------------------------------------------------

golem::data::HandlerContactModelUA::HandlerContactModelUA(golem::Context &context) : HandlerContactModel(context) {
}

void golem::data::HandlerContactModelUA::create(const Desc& desc) {
	golem::data::HandlerContactModel::create(desc);

}

//------------------------------------------------------------------------------

void golem::data::HandlerContactModelUA::set(const golem::Planner& planner, const ControllerId::Seq& controllerIDSeq) {
	HandlerContactModel::set(planner, controllerIDSeq);

	manipulator->getDesc().commandToControl = [=](const golem::Controller::State& command, Manipulator::ControlCoord& control) {
		// PISA Soft Hand
		control.resize(1);
		control[0] = command.cpos[manipulator->getHandInfo().getJoints().begin()]; // first joint is mapped onto synergy
	};
}

//------------------------------------------------------------------------------

golem::data::Item::Ptr golem::data::HandlerContactModelUA::create() const {
	return Item::Ptr(new ItemContactModelUA(*const_cast<HandlerContactModelUA*>(this))); // safe: no modification here - only passing reference
}

//------------------------------------------------------------------------------

Item::Ptr golem::data::HandlerContactModelUA::transform(const Item::List& input) {
	Item::Ptr item(HandlerContactModel::transform(input));
	ItemContactModel* itemContactModel = is<ItemContactModel>(item.get());

	// TODO perform dimensionality reduction on generated configuration models
	//itemContactModel->dataMap = ;

	return item;
}

//------------------------------------------------------------------------------

void golem::data::HandlerContactModelUA::createRender(const ItemContactModelUA& item) {
	HandlerContactModel::createRender(item);

}

void golem::data::HandlerContactModelUA::render() const {
	HandlerContactModel::render();

	if (hasRenderBlock()) return;
}

void golem::data::HandlerContactModelUA::customRender() const {
	HandlerContactModel::customRender();

}

//------------------------------------------------------------------------------

void golem::data::HandlerContactModelUA::mouseHandler(int button, int state, int x, int y) {
	HandlerContactModel::mouseHandler(button, state, x, y);

	if (hasRenderBlock()) return;

}

void golem::data::HandlerContactModelUA::motionHandler(int x, int y) {
	HandlerContactModel::motionHandler(x, y);

}

void golem::data::HandlerContactModelUA::keyboardHandler(int key, int x, int y) {
	HandlerContactModel::keyboardHandler(key, x, y);

	if (hasRenderBlock()) return;

}

//------------------------------------------------------------------------------