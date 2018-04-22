/** @file GolemInteropMaster.cpp
*
* @author	Marek Kopicki
*
*/

#include "GolemInteropMaster.h"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <exception>

using namespace golem;
using namespace golem::interop;

//------------------------------------------------------------------------------

MasterConfig::MasterConfig(const std::string& param) {
	// read config
	std::ifstream ifsconfig(param);
	if (!ifsconfig.good())
		throw std::runtime_error("golem::interop::MasterConfig::MasterConfig(): Unable to open: " + param);
	boost::property_tree::ptree ptconfig;
	boost::property_tree::read_xml(ifsconfig, ptconfig);

	// extract interface - library mapping
	BOOST_FOREACH(const boost::property_tree::ptree::value_type& value, ptconfig.get_child("golem.master_config")) {
		if (value.first == "interface")
			interfaceMap.insert(std::make_pair(value.second.get<std::string>("<xmlattr>.interface"), Interface::Info(value.second.get<std::string>("<xmlattr>.library"), value.second.get<std::string>("<xmlattr>.param"))));
	}
}

//------------------------------------------------------------------------------
