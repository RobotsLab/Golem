/** @file Joint.cpp
 * 
 * Demonstration program which moves joints to the zero pose.
 * 
 * Program can be run in two modes:
 * - the first uses real robot
 * - the second runs robot simulator
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/Controller.h>
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/App/Common/Tools.h>
#include <iostream>

using namespace golem;

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	XMLParser::Ptr pParser;
	Context::Ptr pContext;

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
		pParser = parserDesc.create();
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
		Context::Desc contextDesc;
		XMLData(contextDesc, pXMLContext);
		pContext = contextDesc.create(); // throws
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

		// Display information
		controllerInfo(*pController);

		// Get the initial position
		Controller::State begin = pController->createState();
		pController->lookupState(pContext->getTimer().elapsed(), begin);

		// Go to zero pose
		pContext->info("Moving to the ZERO pose\n");
		Controller::State end = pController->createState();
		pController->setToDefault(end);
		end.t = pContext->getTimer().elapsed() + pController->getCycleDuration() + pController->getCommandLatency() + SecTmReal(10.0);
		pController->send(&end, &end + 1);

		// text output constants
		const SecTmReal cyclePrint = 0.1; // in seconds
		const U32 cycleLatency = 10;
		const U32 cycleDrop = std::max((U32)1, (U32)Math::round(cyclePrint/pController->getCycleDuration()));

		// print the device state along sent trajectory, iterate a few more times to account for commands' delay
		for (U32 i = 0, j = 0; pController->waitForBegin() && (!pController->waitForEnd(0) || j++ < cycleLatency); ++i)
			if (i%cycleDrop == 0) {
				Controller::State state = pController->createState();
				pController->lookupState(SEC_TM_REAL_MAX, state);
				controllerPosition(*pController, state);
			}

		// Wait for some time
		Sleep::msleep(SecToMSec(5.0));

		// and back to the initial pose
		pContext->info("Moving to the INITIAL pose\n");
		begin.t = pContext->getTimer().elapsed() + pController->getCycleDuration() + pController->getCommandLatency() + SecTmReal(10.0);
		pController->send(&begin, &begin + 1);
		
		// print the device state along sent trajectory, iterate a few more times to account for commands' delay
		for (U32 i = 0, j = 0; pController->waitForBegin() && (!pController->waitForEnd(0) || j++ < cycleLatency); ++i)
			if (i%cycleDrop == 0) {
				Controller::State state = pController->createState();
				pController->lookupState(SEC_TM_REAL_MAX, state);
				controllerPosition(*pController, state);
			}

		pContext->info("Good bye!\n");
	}
	catch (const Message& msg) {
		std::cerr << msg.what() << std::endl;
	}
	catch (const std::exception &ex) {
		std::cerr << Message(Message::LEVEL_CRIT, "C++ exception: %s", ex.what()).what() << std::endl;
	}

	return 0;
}
