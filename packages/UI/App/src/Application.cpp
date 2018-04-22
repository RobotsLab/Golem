/** @file Application.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/App/Application.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Math/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Application::Application() : pXMLContext(NULL), pScene(NULL) {
}

Application::~Application() {
	pUniverse.reset();
	pContext.reset();
	pParser.reset();
}

int Application::main(int argc, char *argv[], const Universe::Desc& universeDesc) {
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

		try {
			// Create XML parser and load configuration file
			pParser = XMLParser::load(cfg);
		}
		catch (const Message& msg) {
			std::cerr << msg.what() << std::endl;
			std::cout << "Usage: " << argv[0] << " <configuration_file> [<log_file>] [stdout|stderr]" << std::endl;
			return 1;
		}

		// Find program XML root context
		pXMLContext = pParser->getContextRoot()->getContextFirst("golem");
		if (pXMLContext == NULL)
			throw MsgApplication(Message::LEVEL_CRIT, "Unknown configuration file: %s", cfg.c_str());

		// Create program context
		golem::Context::Desc contextDesc;
		XMLData(contextDesc, pXMLContext);
		// log file
		if (argc >= 3) {
			struct FileMessageCallback : public MessageCallback {
				StreamMessageCallback console;
				std::ofstream file;
				FileMessageCallback(const char* name, StreamMessageCallback::Desc& desc) : file(name, std::ios_base::out | std::ios_base::trunc), console(desc) {}
				void write(const Message& message) {
					console.write(message);
					// append to file
					file << message.what();
				}
			};
			StreamMessageCallback::Desc desc;
			desc.enableFormat = argc >= 4;
			if (desc.enableFormat)
				desc.file = !std::strcmp(argv[3], "stderr") ? stderr : stdout;
			contextDesc.messageStreamDesc->messageCallback.reset(new FileMessageCallback(argv[2], desc));
		}

		pContext = contextDesc.create(); // throws
		
		// Create Universe
		const_cast<Universe::Desc&>(universeDesc).load(pXMLContext->getContextFirst("universe"));
		const_cast<Universe::Desc&>(universeDesc).argc = argc;
		const_cast<Universe::Desc&>(universeDesc).argv = argv;
		pUniverse = universeDesc.create(*pContext);
		
		// Create scene
		Scene::Desc::Ptr sceneDescPtr = pUniverse->createSceneDesc();
		sceneDescPtr->load(pXMLContext->getContextFirst("scene"));
		pScene = pUniverse->createScene(*sceneDescPtr);
		
		// Launch universe
		pUniverse->launch();

		// run application
		run(argc, argv);

		return 0;
	}
	catch (const Message& msg) {
		if (pContext != NULL)
			pContext->write(msg);
		else
			std::cerr << msg.what() << std::endl;
	}
	catch (const std::exception &ex) {
		const Message msg(Message::LEVEL_CRIT, "%s\n", ex.what());
		if (pContext != NULL)
			pContext->write(msg);
		else
			std::cerr << msg.what() << std::endl;
	}

	return 1;
}

//------------------------------------------------------------------------------
