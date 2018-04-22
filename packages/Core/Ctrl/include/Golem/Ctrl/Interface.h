/** @file Interface.h
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
#ifndef _GOLEM_CTRL_INTERFACE_H_
#define _GOLEM_CTRL_INTERFACE_H_

//------------------------------------------------------------------------------

#include <Golem/Sys/Timer.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Binary state controller. */
class BinaryController {
public:
	/** BinaryController: set on/off */
	virtual bool binaryController(bool setOn, MSecTmU32 timeWait = -1) = 0;

	/** BinaryController: set enabled */
	virtual void binaryControllerSetEnabled(bool enabled) = 0;
	/** BinaryController: is enabled */
	virtual bool binaryControllerIsEnabled() const = 0;
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CTRL_INTERFACE_H_*/
