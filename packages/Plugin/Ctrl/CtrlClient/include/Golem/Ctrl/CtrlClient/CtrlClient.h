/** @file CtrlClient.h
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
#ifndef _GOLEM_CTRL_CTRLCLIENT_CTRLCLIENT_H_
#define _GOLEM_CTRL_CTRLCLIENT_CTRLCLIENT_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/SingleCtrl/SingleCtrl.h>
#include <Golem/Ctrl/MultiCtrl/MultiCtrl.h>
#include <Golem/SM/SM.h>
#include <Golem/SM/SMHelper.h>
#include <Golem/Ctrl/CtrlServer/CtrlServer.h>
#include <Golem/Sys/Library.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR CtrlClient: public SingleCtrl {
public:
	friend class CallbackIO;

	/** Callback interface for synchronisation, sending and receiving  */
	class CallbackIO : public SingleCtrl::CallbackIO {
	public:
		typedef shared_ptr<CallbackIO> Ptr;

		/** Construct */
		CallbackIO(CtrlClient& controller, const std::string& host, unsigned short port, SecTmReal timeOut, U32 lostPackages);

		/** Data synchronisation */
		virtual void sysSync(SingleCtrl* ctrl);
		/** Data receive */
		virtual void sysRecv(SingleCtrl* ctrl, State& state);
		/** Data send */
		virtual void sysSend(SingleCtrl* ctrl, const State& prev, State& next, bool bSendPrev, bool bSendNext);

	private:
		/** Controller */
		golem::CtrlClient &controller;
		/** Context object */
		golem::Context &context;

		/** Controller::State: received state */
		golem::Controller::State inp;
		/** Controller::State: command */
		SMCommand out;
		/** Controller::State::Info */
		golem::Controller::State::Info infoCli, infoSrv;

		/** I/O timeout */
		const SecTmReal timeOut;
		/** Minimum number of packages to be reported */
		U32 lostPackages;

		/** SMClient */
		shared_ptr<SMClient> client;
		/** SMTimer */
		SMTimer timer;
		/** SMMessageStream */
		SMMessageStream msgstr;

		/** Message id */
		unsigned id;
	};

	/** Controller description */
	class GOLEM_LIBRARY_DECLDIR Desc : public SingleCtrl::Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		
		using SingleCtrl::Desc::name;
		using SingleCtrl::Desc::globalPose;

		/** Collection of controllers */
		MultiCtrl::ControllerDesc::Seq controllers;
		/** Host name */
		std::string host;
		/** Port number */
		unsigned short port;
		/** I/O timeout */
		SecTmReal timeOut;
		/** Minimum number of packages to be reported */
		U32 lostPackages;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			SingleCtrl::Desc::setToDefault();
			
			name = "Controller Client";
			globalPose.setId();
			controllers.clear();
			host = "localhost";
			port = 54312;
			timeOut = SecTmReal(1.0);
			lostPackages = 1;
		}
		
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			//if (!SingleCtrl::Desc::isValid())
			//	return false;

			if (!globalPose.isFinite())
				return false;
			
			if (!isValidIO() || !isValidLimits() || !isValidSim())
				return false;

			if (controllers.empty())
				return false;
			for (MultiCtrl::ControllerDesc::Seq::const_iterator i = controllers.begin(); i != controllers.end(); ++i)
				if (!i->isValid())
					return false;
			if (!MultiCtrl::ControllerProperty::isValid(controllers))
				return false;
			if (host.empty() || timeOut <= SEC_TM_REAL_ZERO || lostPackages < 1)
				return false;

			return true;
		}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(CtrlClient, Controller::Ptr, Context&)
	};

	/** Sends a sequence of motor commands preserving maximum time wait. */
	virtual const State* send(const State* begin, const State* end, bool clear = false, bool limits = true, MSecTmU32 timeWait = MSEC_TM_U32_INF);

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

	/** Release resources */
	virtual ~CtrlClient();

protected:
	/** Controllers */
	Controller::Seq controllerSeq;
	/** Controller properties */
	MultiCtrl::ControllerProperty::Seq properties;

	/** Callback interface for synchronisation, sending and receiving  */
	CallbackIO::Ptr callbackIO;

	/** Command buffer size */
	size_t qCommandSize;
	/** Command queue */
	shared_ptr<queue<SMCommand> > qSMCommand;
	/** Command queue cs */
	CriticalSection cs;

	/** Releases controllers */
	void release();
	/** User create is called just before launching I/O thread. */
	void userCreate();
	/** Creates Controller from the description. */
	void create(const Desc& desc);
	/** */
	CtrlClient(Context& context);
};

/** Reads/writes object from/to a given XML context */
GOLEM_LIBRARY_DECLDIR void XMLData(CtrlClient::Desc &val, Context* context, XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_CTRLCLIENT_CTRLCLIENT_H_*/
