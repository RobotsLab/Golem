/** @file Load.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/RobotJustin/RobotJustin.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc) {
	//loadObjectDesc<golem::RobotJustin::Desc, Controller::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Controller::Desc::Ptr*)pControllerDesc);
	
	// initialise context in a module
	((Context*)pContext)->initModule();
	
	// Create description and load xml configuration
	RobotJustin::Desc* pDesc = new RobotJustin::Desc;
	((golem::shared_ptr<Controller::Desc>*)pControllerDesc)->reset(pDesc);
	golem::XMLData(*pDesc, (Context*)pContext, (XMLContext*)pXMLContext);
}

//------------------------------------------------------------------------------
