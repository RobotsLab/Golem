/** @file Search.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Tools/Search.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

void golem::XMLData(KDTreeDesc& val, golem::XMLContext* context, bool create) {
	golem::XMLData("search_checks", val.searchChecks, context, create);
	try {
		golem::XMLData("search_kdtrees", val.searchKDTrees, context, create);
	}
	catch (golem::MsgXMLParser&) {}
	try {
		golem::XMLData("search_leaf_max_size", val.searchLeafMaxSize, context, create);
	}
	catch (golem::MsgXMLParser&) {}
	try {
		golem::XMLData("search_branching", val.searchBranching, context, create);
	}
	catch (golem::MsgXMLParser&) {}
	try {
		golem::XMLData("search_iterations", val.searchIterations, context, create);
	}
	catch (golem::MsgXMLParser&) {}
}

//------------------------------------------------------------------------------
