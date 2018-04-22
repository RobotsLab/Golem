/** @file Gripper.cpp
 * 
 * Demonstration program testing Katana gripper.
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/Katana/Katana.h>
#include <Golem/Math/Data.h>
#include <Golem/App/Common/Tools.h>
#include <iostream>

using namespace golem;

//------------------------------------------------------------------------------

void demo(KatanaGripper &gripper, MessageStream* stream) {
	KatanaGripper::SensorDataSet zero, reading, threshold;

	gripper.gripperRecvSensorData(reading);
	for (size_t i = 0; i < reading.size(); ++i)
		stream->write("Sensor #%d: {%d, %d}", i, reading[i].index, reading[i].value);

	threshold = zero = reading;
	for (size_t i = 0; i < threshold.size(); ++i)
		threshold[i].value += 5;

	stream->write("Closing gripper, high sensitivity...");
	if(gripper.gripperClose(threshold))
	  stream->write(" OK.");
	else
	  stream->write(" failed.");
	
	gripper.gripperRecvSensorData(reading);
	for (size_t i = 0; i < reading.size(); ++i)
		stream->write("Sensor #%d: {%d, %d}", i, reading[i].index, reading[i].value);
	
	stream->write("Opening gripper ...");
	if(gripper.gripperOpen())
	  stream->write(" OK.");
	else
	  stream->write(" failed.");
	
	threshold = zero;
	for (size_t i = 0; i < threshold.size(); ++i)
		threshold[i].value += 100;
	
	stream->write("Closing gripper, low sensitivity...");
	if(gripper.gripperClose(threshold))
	  stream->write(" OK.");
	else
	  stream->write(" failed.");
	
	gripper.gripperRecvSensorData(reading);
	for (size_t i = 0; i < reading.size(); ++i)
		stream->write("Sensor #%d: {%d, %d}", i, reading[i].index, reading[i].value);
	
	stream->write("Opening gripper ...");
	if(gripper.gripperOpen())
	  stream->write(" OK.");
	else
	  stream->write(" failed.");
}

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

		//-----------------------------------------------------------------------------

		// Create controller description
		// Load Katana driver
		Controller::Desc::Ptr pDesc = Controller::Desc::load(pContext.get(), pXMLContext->getContextFirst("controller"));

		// Create controller
		pContext->info("Initialising %s...\n", pDesc->name.c_str());
		Controller::Ptr pController = pDesc->create(*pContext);
		KatanaGripper* pKatanaGripper = dynamic_cast<KatanaGripper*>(&*pController);
		if (pKatanaGripper == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to obtain pointer to KatanaGripper", cfg.c_str());
		
		// Display information
		controllerInfo(*pController);
		
		// Gripper demo
		demo(*pKatanaGripper, pContext->getMessageStream());

		// Wait for some time
		Sleep::msleep(SecToMSec(5.0));

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
