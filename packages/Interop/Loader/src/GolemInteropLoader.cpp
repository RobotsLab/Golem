/** @file GolemInteropLoader.cpp
*
* @author	Marek Kopicki
*
*/

#include "GolemInteropLoader.h"
#include "GolemInteropInterface.h"
#include <fstream>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

using namespace golem;
using namespace golem::interop;

int main(int argc, char *argv[]) {
	// Library container should be alive during exception catching
	Library::List libraries;

	try {
		// check if all arguments are provided
		if (argc < 2) {
			fprintf(stderr, "GolemInteropLoader <configuration_file>\n");
			return 1;
		}

		/*************************************************************************
		*
		* Initialisation
		*
		**************************************************************************/

		// read config
		std::ifstream ifsconfig(argv[1]);
		if (!ifsconfig.good())
			throw std::runtime_error("golem::interop::GolemInteropLoader: Unable to open: " + std::string(argv[1]));
		boost::property_tree::ptree ptconfig;
		boost::property_tree::read_xml(ifsconfig, ptconfig);

		// extract interface info, load libraries and interfaces
		Interface::Map interfaces;
		Master* master = nullptr;
		BOOST_FOREACH(const boost::property_tree::ptree::value_type& value, ptconfig.get_child("golem.loader")) {
			if (value.first == "library") {
				const Interface::Info info(value.second.get<std::string>("<xmlattr>.library"), value.second.get<std::string>("<xmlattr>.param"));
				if (interfaces.find(info) == interfaces.end()) {
					// load library and create interface
					Interface *pinterface = libraries.insert(info.library)->getInterface(info.param);
					// find Master interface
					if (!master)
						master = Interface::is<Master>(pinterface);
					// update interface container
					interfaces.insert(std::make_pair(info, pinterface));
				}
			}
		}

		if (!master)
			throw std::runtime_error("golem::interop::GolemInteropLoader: No Master interfaces specified: " + std::string(argv[1]));

		/*************************************************************************
		*
		* Run master
		*
		**************************************************************************/

		master->run(interfaces);
	}
	catch (const std::exception& ex) {
		fprintf(stderr, "%s\n", ex.what());
		return 1;
	}
	catch (...) {
		fprintf(stderr, "golem::interop::GolemInteropLoader: unknown exception\n");
		return 1;
	}

	return 0;
}