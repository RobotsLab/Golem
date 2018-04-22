/** @file Constants.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Defs/Constants.h>
#include <limits.h>
#include <float.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const numeric_const<F32>::FConst numeric_const<F32>::IEEE_POS_INF = {0x7f800000};
const numeric_const<F32>::FConst numeric_const<F32>::IEEE_NEG_INF = {0xff800000};
const F32 numeric_const<F32>::MIN = FLT_MIN;
const F32 numeric_const<F32>::MAX = FLT_MAX;
const F32 numeric_const<F32>::POS_INF = numeric_const<F32>::IEEE_POS_INF.f32;
const F32 numeric_const<F32>::NEG_INF = numeric_const<F32>::IEEE_NEG_INF.f32;
const F32 numeric_const<F32>::INF = numeric_const<F32>::IEEE_POS_INF.f32;
const F32 numeric_const<F32>::EPS = FLT_EPSILON;
const F32 numeric_const<F32>::MAX_10_EXP = FLT_MAX_10_EXP;
const F32 numeric_const<F32>::MAX_EXP = FLT_MAX_EXP;
const F32 numeric_const<F32>::MIN_10_EXP = FLT_MIN_10_EXP;
const F32 numeric_const<F32>::MIN_EXP = FLT_MIN_EXP;

const numeric_const<F64>::FConst numeric_const<F64>::IEEE_POS_INF = {0x7ff0000000000000ULL};
const numeric_const<F64>::FConst numeric_const<F64>::IEEE_NEG_INF = {0xfff0000000000000ULL};
const F64 numeric_const<F64>::MIN = DBL_MIN;
const F64 numeric_const<F64>::MAX = DBL_MAX;
const F64 numeric_const<F64>::POS_INF = numeric_const<F64>::IEEE_POS_INF.f64;
const F64 numeric_const<F64>::NEG_INF = numeric_const<F64>::IEEE_NEG_INF.f64;
const F64 numeric_const<F64>::INF = numeric_const<F64>::IEEE_POS_INF.f64;
const F64 numeric_const<F64>::EPS = DBL_EPSILON;
const F64 numeric_const<F64>::MAX_10_EXP = DBL_MAX_10_EXP;
const F64 numeric_const<F64>::MAX_EXP = DBL_MAX_EXP;
const F64 numeric_const<F64>::MIN_10_EXP = DBL_MIN_10_EXP;
const F64 numeric_const<F64>::MIN_EXP = DBL_MIN_EXP;

//------------------------------------------------------------------------------
