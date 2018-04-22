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
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>
#include <iostream>

using namespace golem;

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	try {
		// Determine configuration file name
		std::string cfg;
		if (argc == 1) {
			// default configuration file name
			cfg.assign(argv[0]);
#ifdef WIN32
			size_t pos = cfg.rfind(".exe"); // Windows only
			if (pos != std::string::npos) cfg.erase(pos);
#endif
			cfg.append(".xml");
		}
		else
			cfg.assign(argv[1]);

		// Create XML parser and load configuration file
		XMLParser::Desc parserDesc;
		XMLParser::Ptr pParser = parserDesc.create();
		try {
			FileReadStream fs(cfg.c_str());
			pParser->load(fs);
		}
		catch (const Message& msg) {
			std::cerr << msg.what() << std::endl;
			std::cout << "Usage: " << argv[0] << " <configuration_file>" << std::endl;
			return 1;
		}

		// Find program XML root context
		XMLContext* pXMLContext = pParser->getContextRoot()->getContextFirst("golem");
		if (pXMLContext == NULL)
			throw Message(Message::LEVEL_CRIT, "Unknown configuration file: %s", cfg.c_str());

		// Create program context
		golem::Context::Desc contextDesc;
		XMLData(contextDesc, pXMLContext);
		golem::Context::Ptr pContext = contextDesc.create(); // throws
		if (pContext == NULL)
			return 1;

		//-----------------------------------------------------------------------------

		// Load driver
		Controller::Desc::Ptr pControllerDesc = Controller::Desc::load(pContext.get(), pXMLContext->getContextFirst("controller"));
		if (pControllerDesc == NULL)
			throw Message(Message::LEVEL_CRIT, "Empty controller description!");

		// Create controller
		pContext->info("Initialising controller...\n");
		Controller::Ptr pController = pControllerDesc->create(*pContext);

		// Create timer
		SMTimer timer(&pContext->getTimer());

		// Create shared memory message stream
		SMMessageStream msgStr(*pContext->getMessageStream());

		// setup server
		unsigned short port = 54312;
		XMLData("port", port, pXMLContext->getContextFirst("server"));
		size_t clients = 1;
		XMLData("clients", clients, pXMLContext->getContextFirst("server"));
		SecTmReal interval = SecTmReal(1.0);
		try {
			XMLData("message_interval", interval, pXMLContext->getContextFirst("server"));
		}
		catch (golem::MsgXMLParser&) {}
		
		// Create server
		SMHandler handler(*pController);
		SMServer server(port, timer, &msgStr, &handler, clients);

		// Handle i/o
		Controller::State state = pController->createState();
		SecTmReal begin = pContext->getTimer().elapsed();
		for (U32 i = 0; ; ++i) {
			(void)pController->waitForBegin();
			const SecTmReal end = pContext->getTimer().elapsed();
			
			// send state
			pController->lookupState(end, state);
			server.write(sizeof(Controller::State), &state);

			// control message
			if (end > begin + interval) {
				pContext->verbose("packets = %d, packetsize = %d, cycle duration = %f\n", i, sizeof(Controller::State), pController->getCycleDuration());
				begin = end;
			}
		}
	}
	catch (const Message& msg) {
		std::cerr << msg.what() << std::endl;
	}
	catch (const std::exception &ex) {
		std::cerr << Message(Message::LEVEL_CRIT, "C++ exception: %s", ex.what()).what() << std::endl;
	}

	return 0;
}

//------------------------------------------------------------------------------
