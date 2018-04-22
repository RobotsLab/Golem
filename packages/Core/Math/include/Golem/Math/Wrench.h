/** @file Wrench.h
 * 
 * Implementation of wrenches.
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
#ifndef _GOLEM_MATH_WRENCH_H_
#define _GOLEM_MATH_WRENCH_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Twist.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Wrench representation of a generalised force acting on a rigid body.
*	Wrench is a dual of twist.
*/
typedef Twist Wrench;

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_WRENCH_H_*/
