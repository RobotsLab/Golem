/** @file GraphSearch.cpp
 * 
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Math/GraphSearch.h>
#include <memory.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const U32 Node::IDX_UINI = numeric_const<U32>::MAX;
const Real Node::COST_ZERO = REAL_ZERO;
const Real Node::COST_INF = numeric_const<Real>::MAX;

//------------------------------------------------------------------------------
