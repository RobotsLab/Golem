/** @file InputCtrl.h
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
#ifndef _GOLEM_ACTIVECTRL_ACTIVECTRL_INPUTCTRL_H_
#define _GOLEM_ACTIVECTRL_ACTIVECTRL_INPUTCTRL_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/ActiveCtrl.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** InputCtrl input device library. */
class InputCtrl {
public:
	/** Smart pointer to the class */
	typedef golem::shared_ptr<InputCtrl> Ptr;
	
	/** Axis handler */
	typedef std::function<void(U32, U32, Real)> HandlerAxis;
	/** Button handler */
	typedef std::function<void(U32, U32, bool)> HandlerButton;

	/** Joystick axis description */
	class AxisDesc {
	public:
		/** Axis index -> description */
		typedef std::multimap<golem::U32, AxisDesc> Map;

		/** Control output index */
		golem::U32 ctrl;

		/** Control output offset */
		golem::Real offset;
		/** Control output gain */
		golem::Real gain;

		/** Control output threshold lo */
		golem::Real lo;
		/** Control output threshold hi */
		golem::Real hi;

		/** Constructs from description object */
		AxisDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			ctrl = 0;
			offset = REAL_ZERO;
			gain = REAL_ONE;
			lo = REAL_ZERO;
			hi = REAL_ZERO;
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(Math::isFinite(offset), ac, "offset: inf");
			Assert::valid(Math::isFinite(gain), ac, "gain: inf");
			Assert::valid(lo <= hi, ac, "lo > hi");
		}
	};

	/** Joystick button description */
	class ButtonDesc {
	public:
		/** Button index -> description */
		typedef std::multimap<golem::U32, ButtonDesc> Map;

		/** Control output index */
		golem::U32 ctrl;
		
		/** Register change up */
		bool up;
		/** Register change down */
		bool down;

		/** Constructs from description object */
		ButtonDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			ctrl = 0;
			up = false;
			down = false;
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			// TODO
		}
	};

	/** Joystick description */
	class JoystickDesc {
	public:
		typedef std::vector<JoystickDesc> Seq;

		/** Name of the joystick */
		std::string label;
		/** Index of the joystick */
		golem::U32 index;

		/** Control Axies */
		AxisDesc::Map axisDescMap;
		/** Control Buttons  */
		ButtonDesc::Map buttonDescMap;

		/** Constructs from description object */
		JoystickDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			label = "";
			index = 0;
			axisDescMap.clear();
			buttonDescMap.clear();
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Assert::valid(!label.empty(), ac, "empty label");
			Assert::valid(index >= 0, ac, "index < 0");
		}
	};

	/** InputCtrl */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Joysticks */
		JoystickDesc::Seq joystickDescSeq;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			joystickDescSeq.clear();
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			for (JoystickDesc::Seq::const_iterator i = joystickDescSeq.begin(); i != joystickDescSeq.end(); ++i) {
				i->assertValid(Assert::Context(ac, "joystickDescSeq[]->"));
			}
		}
		/** Load descritpion from xml context. */
		void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		GOLEM_CREATE_FROM_OBJECT_DESC1(InputCtrl, InputCtrl::Ptr, golem::Context&) 
	};

	/** Update state */
	void update();

	/** Axis handler callbacks */
	void getJoystickAxisPosition(HandlerAxis handler) const;

	/** Button handler callbacks */
	void getJoystickButtonState(HandlerButton handler) const;

	/** Joysticks */
	const JoystickDesc::Seq& getJoystickDescSeq() const {
		return joystickDescSeq;
	}

	/** */
	void testJoystick();

protected:
	/** Button state map */
	typedef std::map<U32, bool> ButtonMap;
	/** Joystick button state map */
	typedef std::map<U32, ButtonMap> JoystickButtonMap;

	/** Context */
	golem::Context& context;

	/** Joysticks */
	JoystickDesc::Seq joystickDescSeq;

	/** Joystick button state map */
	mutable JoystickButtonMap joystickButtonMap;

	/** Get axis position */
	golem::Real getJoystickAxisPosition(const JoystickDesc::Seq::value_type& joystick, const AxisDesc::Map::value_type& axis) const;
	
	/** Get button state */
	bool getJoystickButtonState(const JoystickDesc::Seq::value_type& joystick, const ButtonDesc::Map::value_type& button) const;

	/** Creates/initialises the class */
	void create(const Desc& desc);

	/** C'ctor */
	InputCtrl(golem::Context& context);
};


/** Reads/writes object from/to a given XML context */
void XMLData(golem::InputCtrl::JoystickDesc::Seq::value_type& val, golem::XMLContext* xmlcontext, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(golem::InputCtrl::AxisDesc::Map::value_type& val, golem::XMLContext* xmlcontext, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(golem::InputCtrl::ButtonDesc::Map::value_type& val, golem::XMLContext* xmlcontext, bool create = false);

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_ACTIVECTRL_ACTIVECTRL_INPUTCTRL_H_*/
