/** @file LoadObjectDesc.h
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
#ifndef _GOLEM_SYS_LOADOBJECTDESC_H_
#define _GOLEM_SYS_LOADOBJECTDESC_H_

//------------------------------------------------------------------------------

#include <Golem/Sys/Context.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Loads driver description - the library function must be implemented in each device library */
typedef void (*LoadObjectDesc)(void* pContext, void *pXMLContext, void* pDesc);

//------------------------------------------------------------------------------

class XMLContext;

/** Library function template */
template <typename _Desc, typename _DescPtr> void loadObjectDesc(Context* context, XMLContext* xmlcontext, _DescPtr* pObjectDesc) {
	using namespace golem;

	// initialise context in a module
	context->initModule();

	// Create description and load xml configuration
	_Desc* pDesc = new _Desc;
	pObjectDesc->reset(pDesc);
	XMLData(*pDesc, xmlcontext);
}

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_SYS_LOADOBJECTDESC_H_*/
