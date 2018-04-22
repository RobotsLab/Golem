/** @file XMLData.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

template <> void golem::XMLGetValue(std::string &val, const XMLContext* context) {
	ASSERT(context)
	val = context->getValue();
}

template <> void golem::XMLSetValue(const std::string &val, XMLContext* context) {
	ASSERT(context)
	context->setValue(val);
}

//------------------------------------------------------------------------------

template <> void golem::XMLGetAttribute(const std::string &attr, std::string &val, const XMLContext* context) {
	ASSERT(context)
	val = context->getAttribute(attr);
}

template <> void golem::XMLSetAttribute(const std::string &attr, const std::string &val, XMLContext* context) {
	ASSERT(context)
	context->setAttribute(attr, val);
}

//------------------------------------------------------------------------------
