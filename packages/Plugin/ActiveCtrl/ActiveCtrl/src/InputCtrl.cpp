/** @file InputCtrl.cpp
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/ActiveCtrl/ActiveCtrl/InputCtrl.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <SFML/Window/Joystick.hpp>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void InputCtrl::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	joystickDescSeq.clear();
	golem::XMLData(joystickDescSeq, joystickDescSeq.max_size(), xmlcontext->getContextFirst("input_devices"), "joystick");
}

void golem::XMLData(golem::InputCtrl::JoystickDesc::Seq::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("label", val.label, xmlcontext, create);
	golem::XMLData("index", val.index, xmlcontext, create);

	val.axisDescMap.clear();
	golem::XMLData(val.axisDescMap, val.axisDescMap.max_size(), xmlcontext->getContextFirst("axis_map"), "item");
	val.buttonDescMap.clear();
	golem::XMLData(val.buttonDescMap, val.buttonDescMap.max_size(), xmlcontext->getContextFirst("button_map"), "item");
}

void golem::XMLData(golem::InputCtrl::AxisDesc::Map::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("axis", const_cast<U32&>(val.first), xmlcontext, create);

	golem::XMLData("ctrl", val.second.ctrl, xmlcontext, create);
	golem::XMLData("gain", val.second.gain, xmlcontext, create);
	
	try {
		golem::XMLData("offset", val.second.offset, xmlcontext, create);
	}
	catch (const golem::MsgXMLParser&) {}
	try {
		golem::XMLData("lo", val.second.lo, xmlcontext, create);
	}
	catch (const golem::MsgXMLParser&) {}
	try {
		golem::XMLData("hi", val.second.hi, xmlcontext, create);
	}
	catch (const golem::MsgXMLParser&) {}
}

void golem::XMLData(golem::InputCtrl::ButtonDesc::Map::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("button", const_cast<U32&>(val.first), xmlcontext, create);

	golem::XMLData("ctrl", val.second.ctrl, xmlcontext, create);

	try {
		golem::XMLData("up", val.second.up, xmlcontext, create);
	}
	catch (const golem::MsgXMLParser&) {}
	try {
		golem::XMLData("down", val.second.down, xmlcontext, create);
	}
	catch (const golem::MsgXMLParser&) {}
}

//------------------------------------------------------------------------------

InputCtrl::InputCtrl(golem::Context& context) : context(context) {
}

void InputCtrl::create(const Desc& desc) {
	sf::Joystick::update();
	
	if (desc.joystickDescSeq.empty())
		throw Message(Message::LEVEL_CRIT, "InputCtrl::create(): no joystick description provided");

	joystickDescSeq.clear();
	for (JoystickDesc::Seq::const_iterator joystick = desc.joystickDescSeq.begin(); joystick != desc.joystickDescSeq.end(); ++joystick) {
		if (!sf::Joystick::isConnected(joystick->index)) {
			context.warning("InputCtrl::create(): joystick=%s not found\n", joystick->label.c_str());
			continue;
		}

		sf::Joystick::Identification id = sf::Joystick::getIdentification(joystick->index);
		context.debug("InputCtrl::create(): joystick=%s, index=%u, vendorID=%u, productID=%u\n", joystick->label.c_str(), joystick->index, id.vendorId, id.productId);

		for (AxisDesc::Map::const_iterator axis = joystick->axisDescMap.begin(); axis != joystick->axisDescMap.end(); ++axis) {
			if (!sf::Joystick::hasAxis(joystick->index, (sf::Joystick::Axis)((U32)sf::Joystick::X + axis->first)))
				throw Message(Message::LEVEL_CRIT, "InputCtrl::create(): joystick=%s: axis #%u required", joystick->label.c_str(), axis->first);
			
			context.verbose("InputCtrl::create(): axis: %u -> %u\n", axis->first, axis->second.ctrl);
		}

		const U32 buttonCount = sf::Joystick::getButtonCount(joystick->index);
		for (ButtonDesc::Map::const_iterator button = joystick->buttonDescMap.begin(); button != joystick->buttonDescMap.end(); ++button) {
			if (button->first > buttonCount)
				throw Message(Message::LEVEL_CRIT, "InputCtrl::create(): joystick=%s: button #%u required", joystick->label.c_str(), button->first);
			
			context.verbose("InputCtrl::create(): button: %u -> %u (%s)\n", button->first, button->second.ctrl, 
				button->second.up && !button->second.down ? "UP" : !button->second.up && button->second.down ? "DOWN" : button->second.up && button->second.down ? "UP/DOWN" : "CONTINUOUS"
			);

			joystickButtonMap[joystick->index][button->second.ctrl] = sf::Joystick::isButtonPressed(joystick->index, (unsigned int)button->first);
		}

		joystickDescSeq.push_back(*joystick);
	}
}

//------------------------------------------------------------------------------

void InputCtrl::update() {
	sf::Joystick::update();
}

golem::Real InputCtrl::getJoystickAxisPosition(const JoystickDesc::Seq::value_type& joystick, const AxisDesc::Map::value_type& axis) const {
	Real val = sf::Joystick::getAxisPosition( joystick.index, ( sf::Joystick::Axis )axis.first );
	val = ( val > axis.second.lo && val < axis.second.hi ) ? 0.0 : ( (val >= axis.second.hi ) ? (val - axis.second.hi): ( val - axis.second.lo ) );
	val = axis.second.offset + axis.second.gain*val;

	return val;
}

bool InputCtrl::getJoystickButtonState(const JoystickDesc::Seq::value_type& joystick, const ButtonDesc::Map::value_type& button) const {
	const bool state = sf::Joystick::isButtonPressed(joystick.index, (unsigned int)button.first);
	
	const bool stateOld = joystickButtonMap[joystick.index][button.second.ctrl];
	joystickButtonMap[joystick.index][button.second.ctrl] = state;

	return
		 button.second.up && !button.second.down ? !state &&  stateOld :
		!button.second.up &&  button.second.down ?  state && !stateOld :
		 button.second.up &&  button.second.down ?  state !=  stateOld : state;
}

//------------------------------------------------------------------------------

void InputCtrl::getJoystickAxisPosition(HandlerAxis handler) const {
	for (JoystickDesc::Seq::const_iterator joystick = joystickDescSeq.begin(); joystick != joystickDescSeq.end(); ++joystick)
		for (AxisDesc::Map::const_iterator axis = joystick->axisDescMap.begin(); axis != joystick->axisDescMap.end(); ++axis)
			handler(joystick->index, axis->second.ctrl, getJoystickAxisPosition(*joystick, *axis));
}

void InputCtrl::getJoystickButtonState(HandlerButton handler) const {
	for (JoystickDesc::Seq::const_iterator joystick = joystickDescSeq.begin(); joystick != joystickDescSeq.end(); ++joystick)
		for (ButtonDesc::Map::const_iterator button = joystick->buttonDescMap.begin(); button != joystick->buttonDescMap.end(); ++button)
			handler(joystick->index, button->second.ctrl, getJoystickButtonState(*joystick, *button));
}

void InputCtrl::testJoystick() {

	int deviceId = 0;

	double valX = sf::Joystick::getAxisPosition(deviceId , sf::Joystick::X );
	double valY = sf::Joystick::getAxisPosition( deviceId, sf::Joystick::Y );
	double valZ = sf::Joystick::getAxisPosition( deviceId, sf::Joystick::Z);
	double valR = sf::Joystick::getAxisPosition( deviceId, sf::Joystick::R );
	double valU = sf::Joystick::getAxisPosition( deviceId, sf::Joystick::U );
	double valV = sf::Joystick::getAxisPosition( deviceId, sf::Joystick::V );
	double valPovX = sf::Joystick::getAxisPosition( deviceId, sf::Joystick::PovX );
	double valPovY = sf::Joystick::getAxisPosition( deviceId, sf::Joystick::PovY );


	std::cout << "[Joystick] " << " valX (" << sf::Joystick::X << ") " << valX
		<< " | " << " valY (" << sf::Joystick::Y << ") " << valY
		<< " | " << " valZ (" << sf::Joystick::Z << ") " << valZ
		<< " | " << " valR (" << sf::Joystick::R << ") " << valR
		<< " | " << " valU (" << sf::Joystick::U << ") " << valU
		<< " | " << " valV (" << sf::Joystick::V << ") " << valV
		<< " | " << " valPovX (" << sf::Joystick::PovX << ") " << valPovX
		<< " | " << " valPovY (" << sf::Joystick::PovY << ") " <<  valPovY
		<< "\r" ;

}
//------------------------------------------------------------------------------