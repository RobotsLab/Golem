/** @file CtrlServer.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/CtrlServer/CtrlServer.h>
#include <Golem/App/Application.h>
#include <Golem/App/UIController.h>
#include <Golem/App/Data.h>
#include <Golem/Sys/XMLData.h>
#include <iostream>

using namespace golem;

//------------------------------------------------------------------------------

/** MyApplication */
class MyApplication : public Application {
protected:
	/** Runs MyApplication */
	virtual void run(int argc, char *argv[]) {
		// help message
		scene()->printHelp();
		
		// Setup controller
		UIController::Desc controllerDesc;
		XMLData(controllerDesc, context(), xmlcontext());

		// Create controller
		context()->info("Initialising controller...\n");
		UIController *pUIController = dynamic_cast<UIController*>(scene()->createObject(controllerDesc));
		if (pUIController == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create controller");
		Controller &controller = pUIController->getController();
		
		// Create timer
		SMTimer timer(&context()->getTimer());

		// Create shared memory message stream
		SMMessageStream msgStr(*context()->getMessageStream());

		// setup server
		unsigned short port = 54312;
		XMLData("port", port, xmlcontext()->getContextFirst("server"));
		size_t clients = 1;
		XMLData("clients", clients, xmlcontext()->getContextFirst("server"));
		SecTmReal interval = SecTmReal(1.0);
		try {
			XMLData("message_interval", interval, xmlcontext()->getContextFirst("server"));
		}
		catch (golem::MsgXMLParser&) {}
		
		// Create server
		SMHandler handler(controller);
		SMServer server(port, timer, &msgStr, &handler, clients);

		// Handle i/o
		Controller::State state = controller.createState();
		SecTmReal begin = context()->getTimer().elapsed();
		for (U32 i = 0; !universe()->interrupted(); ++i) {
			(void)controller.waitForBegin();
			const SecTmReal end = context()->getTimer().elapsed();
			
			// send state
			controller.lookupState(end, state);
			server.write(sizeof(Controller::State), &state);

			// control message
			if (end > begin + interval) {
				context()->verbose("packets = %d, packetsize = %d, cycle duration = %f\n", i, sizeof(Controller::State), controller.getCycleDuration());
				begin = end;
			}
		}
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}

//------------------------------------------------------------------------------
