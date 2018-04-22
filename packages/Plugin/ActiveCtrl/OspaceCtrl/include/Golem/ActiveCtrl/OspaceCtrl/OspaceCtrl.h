/** @file OspaceCtrl.h
 * 
 * @author	Maxime Adjigble 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_ACTIVECTRL_OSPACE_CTRL_OSPACE_CTRL_H_
#define _GOLEM_ACTIVECTRL_OSPACE_CTRL_OSPACE_CTRL_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/ActiveCtrl.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Tools/FT.h>
#include <Golem/ActiveCtrl/ActiveCtrl/ActiveSingleCtrl.h>
#include <Golem/Math/Vec2.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** OspaceCtrl implements operational space control. */
class GOLEM_LIBRARY_DECLDIR OspaceCtrl : public ActiveCtrl, public ActiveSingleCtrl, public UI {
public:
	/** OspaceCtrl factory */
	class GOLEM_LIBRARY_DECLDIR Desc : public ActiveCtrl::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Init timeout */
		golem::MSecTmU32 timeOut;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			ActiveCtrl::Desc::setToDefault();

			timeOut = golem::MSEC_TM_U32_INF;
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			ActiveCtrl::Desc::assertValid(ac);

		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		GOLEM_CREATE_FROM_OBJECT_DESC2(OspaceCtrl, ActiveCtrl::Ptr, golem::Planner&, const Sensor::Seq&)
	};

	/** Activate/deactivate */
	virtual void setActive(bool active = true);

protected:
	golem::SingleCtrl* ctrl;

	/** Init timeout */
	golem::MSecTmU32 timeOut;

	/** Force access cs */
	mutable golem::CriticalSection csData;
	/** Render data */
	mutable golem::DebugRenderer renderer;

	/** SingleCtrl::CallbackIO: Data receive */
	virtual void sysRecv(golem::SingleCtrl* ctrl, golem::Controller::State& state);
	/** SingleCtrl::CallbackIO: Data send */
	virtual void sysSend(golem::SingleCtrl* ctrl, const golem::Controller::State& prev, golem::Controller::State& next, bool bSendPrev, bool bSendNext);

	/** golem::UIRenderer: Render on output device. */
	virtual void render() const;
	/** golem::UIRenderer: Render on output device. */
	virtual void customRender() const {}

	/** golem::UIKeyboardMouse: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	/** golem::UIKeyboardMouse: Mouse motion handler. */
	virtual void motionHandler(int x, int y);
	/** golem::UIKeyboardMouse: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);

	/** Creates/initialises the ActiveCtrl */
	void create(const Desc& desc);

	/** Constructs the ActiveCtrl */
	OspaceCtrl(golem::Planner &planner, const Sensor::Seq& sensorSeq);
	/** Cleaning */
	~OspaceCtrl();
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_ACTIVECTRL_OSPACE_CTRL_OSPACE_CTRL_H_*/
