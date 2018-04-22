/** @file ActiveCtrl.h
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
#ifndef _GOLEM_PLUGIN_ACTIVECTRL_H_
#define _GOLEM_PLUGIN_ACTIVECTRL_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Library.h>
#include <Golem/Plugin/Sensor.h>
#include <Golem/Plugin/Ctrl.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Active interface */
class Active {
public:
	/** Activate/deactivate */
	virtual void setActive(bool active = true) {
		this->active = active;
	}
	/** Is active */
	virtual bool isActive() const {
		return active;
	}

	/** Reset */
	Active(bool active = false) : active(active) {}

private:
	/** Active flag */
	bool active;
};

//------------------------------------------------------------------------------

/** ActiveCtrl interface */
class ActiveCtrl : public Active, public Library {
public:
	typedef golem::shared_ptr<ActiveCtrl> Ptr;
	typedef std::map<std::string, Ptr> Map;
	typedef std::vector<Ptr> Seq;

	/** ActiveCtrl description */
	class Desc : public Library::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Planner index */
		golem::U32 plannerIndex;
		
		/** Controller ids */
		ControllerId::Seq controllerIDSeq;
		/** Sensor ids */
		SensorId::Seq sensorIDSeq;

		/** Start as active */
		bool active;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		
		/** Sets the parameters to the default values. */
		void setToDefault() {
			Library::Desc::setToDefault();

			libraryPrefix = "Golem";
			libraryName = "golem activectrl";

			plannerIndex = 0;

			controllerIDSeq.clear();
			sensorIDSeq.clear();

			active = true;
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Library::Desc::assertValid(ac);

			Assert::valid(!controllerIDSeq.empty(), ac, "controllerIDSeq: empty");
			for (ControllerId::Seq::const_iterator i = controllerIDSeq.begin(); i != controllerIDSeq.end(); ++i)
				i->assertValid(Assert::Context(ac, "controllerIDSeq."));

			for (SensorId::Seq::const_iterator i = sensorIDSeq.begin(); i != sensorIDSeq.end(); ++i)
				i->assertValid(Assert::Context(ac, "sensorIDSeq."));
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual ActiveCtrl::Ptr create(golem::Planner &planner, const Sensor::Seq& sensorSeq) const = 0;
	};

	/** Controller state info */
	const golem::Controller::State::Info& getInfo() const {
		return info;
	}

	/** Planner reference */
	golem::Planner& getPlanner() {
		return planner;
	}
	/** Planner reference */
	const golem::Planner& getPlanner() const {
		return planner;
	}

	/** Controller reference */
	golem::Controller& getController() {
		return controller;
	}
	/** Controller reference */
	const golem::Controller& getController() const {
		return controller;
	}

	/** Controller ids in the controller tree */
	const ControllerId::Seq& getControllerIDSeq() const {
		return controllerIDSeq;
	}

	/** Sensor ids */
	const SensorId::Seq& getSensorIDSeq() const {
		return sensorIDSeq;
	}
	/** Used sensors */
	const Sensor::Seq& getSensorSeq() const {
		return sensorSeq;
	}

protected:
	/** Planner index */
	golem::U32 plannerIndex;

	/** Planner reference */
	golem::Planner &planner;
	/** Controller reference */
	golem::Controller &controller;

	/** Controller state info */
	golem::Controller::State::Info info;

	/** Controller ids in the controller tree */
	ControllerId::Seq controllerIDSeq;
	
	/** Sensor ids */
	SensorId::Seq sensorIDSeq;
	/** All sensors */
	Sensor::Seq sensorSeqAll;
	/** Used sensors */
	Sensor::Seq sensorSeq;

	/** Creates/initialises the ActiveCtrl */
	void create(const Desc& desc);
	
	/** Constructs the ActiveCtrl */
	ActiveCtrl(golem::Planner &planner, const Sensor::Seq& sensorSeq);
};

//------------------------------------------------------------------------------

/** Active control interface */
class IActiveCtrl {
public:
	/** IActiveCtrl: Update callback */
	typedef std::function<void(const Mat34&, const Mat34&, const Mat34&, SecTmReal)> Update;
	/** IActiveCtrl: Control callback */
	typedef std::function<void(void)> Control;
	/** IActiveCtrl: Scale callback */
	typedef std::function<SecTmReal(SecTmReal)> Scale;

	/** IActiveCtrl: trajectory control */
	virtual void trajectoryCtrl(const golem::Controller::State::Seq& trajectory, SecTmReal referenceTime, const Mat34& targetFrame, const Mat34& manifoldFrame, const Twist& manifoldFrameDev, bool globalMode, IActiveCtrl::Control control, IActiveCtrl::Update update, IActiveCtrl::Scale scale = nullptr) = 0;
	
	/** IActiveCtrl: variable access */
	virtual Mat34 trajectoryCtrlLocalFrame() const = 0;
	/** IActiveCtrl: variable access */
	virtual Mat34 trajectoryCtrlTargetFrame() const = 0;
};

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_PLUGIN_ACTIVECTRL_H_*/
