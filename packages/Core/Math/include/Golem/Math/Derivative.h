/** @file Derivative.h
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
#ifndef _GOLEM_MATH_DERIVATIVE_H_
#define _GOLEM_MATH_DERIVATIVE_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Function.h>
#include <vector>

namespace golem {

//------------------------------------------------------------------------------

/** Equation solver for single variable real functions */
template <typename REAL>
class _Derivative {
public:
	typedef shared_ptr< _Derivative<REAL> > Ptr;
	
	static const REAL EPS_DFLT;
	
	virtual ~_Derivative() {}

	/** Single solution equation solver */
	virtual REAL findFirst(REAL x, const _Function<REAL> &function) const = 0;
	
	/** Multiple solutions equation solver */
	virtual REAL findSecond(REAL x, const _Function<REAL> &function) const = 0;
};

template <typename REAL> const REAL _Derivative<REAL>::EPS_DFLT = REAL(1.0e-6);

//------------------------------------------------------------------------------

template <typename REAL>
class _DerivativeSecant : public _Derivative<REAL> {
protected:
	const REAL epsilon;

public:
	_DerivativeSecant(REAL epsilon = _Derivative<REAL>::EPS_DFLT) :
		epsilon(epsilon)
	{}

	virtual REAL findFirst(REAL x, const _Function<REAL> &function) const {
		return (function.get(x + epsilon) - function.get(x - epsilon))/(REAL(2.0)*epsilon);
	}

	virtual REAL findSecond(REAL x, const _Function<REAL> &function) const {
		return (function.get(x + epsilon) - REAL(2.0)*function.get(x) + function.get(x - epsilon))/(epsilon*epsilon);
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_DERIVATIVE_H_*/
