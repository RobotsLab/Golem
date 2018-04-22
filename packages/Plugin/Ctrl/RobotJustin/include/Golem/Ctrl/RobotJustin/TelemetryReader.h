/** @file TelemetryReader.h
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
#ifndef _GOLEM_CTRL_ROBOTJUSTIN_TELEMETRYREADER_H_
#define _GOLEM_CTRL_ROBOTJUSTIN_TELEMETRYREADER_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/SingleCtrl/SingleCtrl.h>
#include <Golem/Sys/XMLParser.h>
#include <Golem/Ctrl/RobotJustin/bham_planner_interface.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Justin telemetry reader. */
class TelemetryReader {
public:
	friend class Callback;
	typedef shared_ptr<TelemetryReader> Ptr;

	/** Golem controller recv callback. */
	class Callback : public SingleCtrl::CallbackIO {
	public:
		friend class TelemetryReader;
		typedef shared_ptr<Callback> Ptr;
		
		/** Data receive */
		virtual void sysRecv(SingleCtrl* ctrl, Controller::State& state) {
			SingleCtrl::CallbackIO::sysRecv(ctrl, state); // simulation
			telemetryReader.recvState(state, 0); // non-blocking
		}

	private:
		/** Justin telemetry reader. */
		TelemetryReader& telemetryReader;
		/** Controller */
		SingleCtrl* ctrl;
		/** Register callback */
		Callback(TelemetryReader& telemetryReader, SingleCtrl* ctrl) : telemetryReader(telemetryReader), ctrl(ctrl) {
			ctrl->setCallbackIO(this);
		}
	};

	/** Object description */
	class Desc {
	public:	
		/** Option for remote connection. Switch to TCP connection */
		bool remote;
		/** Justin host */
		std::string host;
		/** Justin udp port */
		unsigned short port;

		/** Creates object from description. */
		CREATE_FROM_OBJECT_DESC_1(TelemetryReader, TelemetryReader::Ptr, Controller&)
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}	
		/** Sets the parameters to the default values */
		void setToDefault() {
			remote = false;
			host = "localhost";
			port = 54454;
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (host.empty() || port <= 0)
				return false;
			return true;
		}
	};

	/** Install arm callback */
	void installArmCallback(SingleCtrl& arm);
	/** Install arm callback */
	void installHandCallback(SingleCtrl& hand);

	/** Receive state */
	bool recvState(Controller::State& state, MSecTmU32 timeWait = MSEC_TM_U32_INF);

	/** Destructor */
	virtual ~TelemetryReader();

private:
	/** Controller */
	Controller &controller;
	/** Context object */
	Context &context;

	/** Arm callback */
	Callback::Ptr armCallback;
	/** Hand callback */
	Callback::Ptr handCallback;

	/** Telemetry packet */
	telemetry_packet packet;
	/** Current state */
	shared_ptr<Controller::State> state;

	/** UDP Socket */
	int socket;

	/** Open connection */
	void open(unsigned short port, const std::string &host = "", const bool tcp = false);
	/** Close connection */
	void close();
	/** Receive data */
	bool recv(void* data, size_t size, MSecTmU32 timeWait = MSEC_TM_U32_INF);

	/** Receive telemetry packet */
	bool recvTelemetryPacket(telemetry_packet& packet, MSecTmU32 timeWait = MSEC_TM_U32_INF);
	/** Telemetry packet to state */
	void telemetryPacketToState(const telemetry_packet& packet, Controller::State& state) const;

	/** Creates object from the description. */
	void create(const Desc& desc);
	/** Constructor */
	TelemetryReader(Controller &controller);
};

/** Reads/writes object from/to a given XML context */
void XMLData(TelemetryReader::Desc &val, XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_ROBOTJUSTIN_TELEMETRYREADER_H_*/
