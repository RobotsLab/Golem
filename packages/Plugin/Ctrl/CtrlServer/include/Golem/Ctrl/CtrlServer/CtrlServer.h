/** @file CtrlServer.h
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
#ifndef _GOLEM_CTRL_CTRLSERVER_CTRLSERVER_H_
#define _GOLEM_CTRL_CTRLSERVER_CTRLSERVER_H_

//------------------------------------------------------------------------------

#include <Golem/Sys/MessageStream.h>
#include <Golem/Ctrl/Controller.h>
#include <Golem/SM/SM.h>
#include <Golem/SM/SMHelper.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class SMCommand {
public:
	Controller::State state;
	U32 clear;
	U32 limits;

	SMCommand(const Controller::State& state, bool clear = false, bool limits = true) : state(state), clear(U32(clear)), limits(U32(limits)) {}
};


class SMHandler : public SMServer::Handler {
public:
	SMHandler(Controller& controller) : controller(controller) {}

	virtual void set(const SMServer::Endpoint& endpoint, const SM::Header& header, const void* data) {
		if (header.size != sizeof(SMCommand)) {
			controller.getContext().error("Handler::set(): host: %s, port: %i: invalid command size %d/%d\n", endpoint.address.c_str(), endpoint.port, header.size, sizeof(SMCommand));
			return;
		}

		const SMCommand* smcommand = reinterpret_cast<const SMCommand*>(data);
		if (smcommand->state.getInfo() != controller.getStateInfo()) {
			controller.getContext().error("Handler::set(): host: %s, port: %i: invalid state type\n", endpoint.address.c_str(), endpoint.port);
			return;
		}

		try {
			if (controller.send(&smcommand->state, &smcommand->state + 1, smcommand->clear > 0, smcommand->limits > 0, 0) == &smcommand->state) {
				controller.getContext().error("Handler::set(): host: %s, port: %i: unable to send command\n", endpoint.address.c_str(), endpoint.port);
				return;
			}
		}
		catch (const std::exception& ex) {
			controller.getContext().write("%s: host: %s, port: %i\n", ex.what(), endpoint.address.c_str(), endpoint.port);
		}
	}

private:
	Controller& controller;
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CTRL_CTRLSERVER_CTRLSERVER_H_*/
