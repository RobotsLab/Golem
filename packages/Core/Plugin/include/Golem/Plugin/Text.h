/** @file Text.h
 * 
 * Text interface
 * 
 * @author Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_PLUGIN_TEXT_H_
#define _GOLEM_PLUGIN_TEXT_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Defs.h>
#include <iostream>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

namespace data {

/** Text interface.
*/
class Text {
public:
	/** Text: access iostream. */
	virtual std::iostream& getTextStream() = 0;
	/** Text: access iostream. */
	virtual const std::iostream& getTextStream() const = 0;
};

};	// namespace

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_PLUGIN_TEXT_H_*/
