/** @file MultiCtrl.h
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
#ifndef _GOLEM_CTRL_MULTICTRL_MULTICTRL_H_
#define _GOLEM_CTRL_MULTICTRL_MULTICTRL_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Controller.h>
#include <Golem/Ctrl/MultiCtrl/Load.h>
#include <Golem/Sys/XMLParser.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */

//------------------------------------------------------------------------------

/** Multi controller interface. */
class GOLEM_LIBRARY_DECLDIR MultiCtrl : public Controller {
public:
	/** Controller property */
	class GOLEM_LIBRARY_DECLDIR ControllerProperty {
	public:
		/** Controller property collection */
		typedef std::vector<ControllerProperty> Seq;

		/** Wait for begin sync */
		bool syncBegin;
		/** Wait for end sync */
		bool syncEnd;

		/** Constructs the property object. */
		ControllerProperty() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			syncBegin = true;
			syncEnd = true;
		}
		/** Checks if the object is valid. */
		bool isValid() const {
			return true;
		}
		/** Checks if the object sequence is valid. */
		template <typename _Seq> static bool isValid(const _Seq& seq) {
			bool syncBegin = false;
			bool syncEnd = false;
			for (typename _Seq::const_iterator i = seq.begin(); i != seq.end(); ++i) {
				if (!i->isValid())
					return false;
				if (i->syncBegin) syncBegin = true;
				if (i->syncEnd) syncEnd = true;
			}
			if (!syncBegin || !syncEnd)
				return false;

			return true;
		}
	};

	/** Controller description */
	class GOLEM_LIBRARY_DECLDIR ControllerDesc : public ControllerProperty {
	public:
		/** Controller collection */
		typedef std::vector<ControllerDesc> Seq;

		/** Controller description */
		Controller::Desc::Ptr pControllerDesc;
		/** Local pose */
		Mat34 localPose;
		/** Linked chain index */
		U32 linkedChainIndex;

		/** Constructs the description object. */
		ControllerDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			ControllerProperty::setToDefault();
			pControllerDesc.reset();
			localPose.setId();
			linkedChainIndex = -1;
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (!ControllerProperty::isValid())
				return false;

			if (pControllerDesc == NULL || !pControllerDesc->isValid() || !localPose.isValid())
				return false;

			return true;
		}
		/** Create controllers */
		static void create(Context& context, Seq& controllerDescSeq, Controller::Seq& controllerSeq, Controller::Desc& desc);
	};

	/** Controller description */
	class GOLEM_LIBRARY_DECLDIR Desc : protected Controller::Desc {
	public:
		friend class MultiCtrl;
		friend void ::loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
		typedef shared_ptr<Desc> Ptr;
		
		using Controller::Desc::name;
		using Controller::Desc::globalPose;

		/** Collection of controllers */
		ControllerDesc::Seq controllers;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Controller::Desc::setToDefault();
			
			name = "Multi Controller";
			globalPose.setId();
			controllers.clear();
		}
		
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!globalPose.isFinite())
				return false;
			
			if (controllers.empty())
				return false;
			for (ControllerDesc::Seq::const_iterator i = controllers.begin(); i != controllers.end(); ++i)
				if (!i->isValid())
					return false;
			if (!ControllerProperty::isValid(controllers))
				return false;

			return true;
		}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(MultiCtrl, Controller::Ptr, Context&)
	};

protected:
	/** Multiple controller critical section guard */
	template <typename _Seq> class Guard {
		_Seq controllers;

	public:
		Guard(const _Seq& controllers) {
			for (typename _Seq::const_iterator i = controllers.begin(); i != controllers.end(); ++i) {
				this->controllers.push_back(*i);
				(*i)->getCommandCS().lock();
			}
		}
		~Guard() {
			for (typename _Seq::const_iterator i = controllers.begin(); i != controllers.end(); ++i) {
				(*i)->getCommandCS().unlock();
			}
		}
	};
	
	/** Controllers */
	Controller::Seq controllerSeq;
	/** Controller properties */
	ControllerProperty::Seq properties;

	/** Releases controllers */
	void release();

	/** Creates Controller from the description. */
	void create(const Desc& desc);

	MultiCtrl(Context& context);

public:
	/** Virtual destructor releasing resources */
	virtual ~MultiCtrl();

	/** Sends a sequence of motor commands preserving maximum time wait. */
	virtual const State* send(const State* begin, const State* end, bool clear = false, bool limits = true, MSecTmU32 timeWait = MSEC_TM_U32_INF);

	/** Stops the device. */
	virtual void stop();

	/** Resumes the device. */
	virtual void resume();

	/** Controller has just sent new trajectory segment */
	virtual bool waitForBegin(MSecTmU32 timeWait = MSEC_TM_U32_INF);
	
	/** Controller has just sent the last trajectory segment */
	virtual bool waitForEnd(MSecTmU32 timeWait = MSEC_TM_U32_INF);

	/** Interpolates the controller state at time t. */
	virtual void lookupState(SecTmReal t, State &state) const;

	/** Interpolates the controller command at time t. */
	virtual void lookupCommand(SecTmReal t, State &command) const;

	/** I/O cycle duration. */
	virtual SecTmReal getCycleDuration() const;

	/** I/O command (minimum) latency. */
	virtual SecTmReal getCommandLatency() const;

	/** I/O command time stamp. */
	virtual SecTmReal getCommandTime() const;

	/** I/O command queue capacity. */
	virtual U32 getCommandCapacity() const;

	/** Sets to default a given state. */
	virtual void setToDefault(State& state) const;

	/** Clamp configuration. */
	virtual void clampConfig(GenConfigspaceCoord& config) const;

	/** Clamp state. */
	virtual void clampState(State& state) const;

	/** Interpolates configuration at time t. */
	virtual void interpolateConfig(const GenConfigspaceState& prev, const GenConfigspaceState& next, SecTmReal t, GenConfigspaceState& state) const;

	/** Interpolates the controller state at time t. */
	virtual void interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const;

	/** Asserts that the controller trajectory segment is in the limits. */
	virtual void assertLimits(const State& prev, const State& next) const;
};

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
GOLEM_LIBRARY_DECLDIR void XMLData(MultiCtrl::ControllerProperty &val, XMLContext* xmlcontext, bool create = false);
/** Reads/writes object from/to a given XML context */
GOLEM_LIBRARY_DECLDIR void XMLData(MultiCtrl::ControllerDesc &val, Context* context, XMLContext* xmlcontext, bool create = false);
/** Reads/writes object from/to a given XML context */
GOLEM_LIBRARY_DECLDIR void XMLData(MultiCtrl::Desc &val, Context* context, XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CTRL_MULTICTRL_MULTICTRL_H_*/
