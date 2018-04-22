/** @file Root.h
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
#ifndef _GOLEM_MATH_ROOT_H_
#define _GOLEM_MATH_ROOT_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Function.h>
#include <vector>

namespace golem {

//------------------------------------------------------------------------------

/** Equation solver for single variable real functions */
template <typename REAL>
class _Root {
public:
	typedef shared_ptr< _Root<REAL> > Ptr;
	typedef std::vector<REAL> Seq;

	static const U32 PARTITIONS_DFLT = 100;
	static const U32 STEPS_DFLT = 100;
	static const REAL EPS_DFLT;

	virtual ~_Root() {}

	/** Single solution equation solver	*/
	virtual bool findSingle(REAL &x, REAL y, const _Function<REAL> &function, REAL left, REAL right) const = 0;
	
	/** Multiple solutions equation solver */
	virtual bool findMultiple(Seq &xSeq, REAL y, const _Function<REAL> &function, REAL left, REAL right) const = 0;
};

template <typename REAL> const REAL _Root<REAL>::EPS_DFLT = REAL(1.0e-6);

//------------------------------------------------------------------------------

template <typename REAL>
class _RootBracket : public _Root<REAL> {
protected:
	const U32 partitions;
	const REAL epsilon;

	_RootBracket(U32 partitions, REAL epsilon) :
		partitions(partitions), epsilon(epsilon)
	{}

public:
	typedef std::vector<REAL> Seq;
	
	/** Multiple solutions equation solver with bracketing */
	virtual bool findMultiple(Seq &xSeq, REAL y, const _Function<REAL> &function, REAL left, REAL right) const {
		const REAL dx = (right - left)/REAL(partitions);
		
		xSeq.clear();
		REAL a = left;
		for (U32 i = 0; i < partitions; ++i) {
			REAL b = left + dx*REAL(i + 1);
			
			// look for sign change
			if ((function.get(a) - y)*(function.get(b) - y) <= REAL(0.0)) {
				REAL x;
				// find roots in [a, b]
				if (this->findSingle(x, y, function, a, b)) {
					// check if solutions do not lie on the same boundary
					if (!xSeq.empty() && Math::abs(x - xSeq.back()) < epsilon)
						continue;

					xSeq.push_back(x);
				}
			}

			a = b;
		}

		return !xSeq.empty();
	}
};

//------------------------------------------------------------------------------

template <typename REAL>
class _RootBisection : public _RootBracket<REAL> {
protected:
	const U32 steps;

public:
	_RootBisection(U32 partitions = _Root<REAL>::PARTITIONS_DFLT, REAL epsilon = _Root<REAL>::EPS_DFLT, U32 steps = _Root<REAL>::STEPS_DFLT) :
		_RootBracket<REAL>(partitions, epsilon), steps(steps)
	{}

	/** Single solution equation solver */
	virtual bool findSingle(REAL &x, REAL y, const _Function<REAL> &function, REAL left, REAL right) const {
		REAL x0 = left, x1 = right;
		for (U32 i = 0; i < steps; ++i) {
			x = (x0 + x1)/REAL(2.0);
			if (Math::abs(x0 - x) < _RootBracket<REAL>::epsilon) {
				// check if the solution lies in boundaries
				if (Math::abs(left - x) < _RootBracket<REAL>::epsilon && Math::abs(function.get(left) - y) > _RootBracket<REAL>::epsilon)
					return false;
				if (Math::abs(right - x) < _RootBracket<REAL>::epsilon && Math::abs(function.get(right) - y) > _RootBracket<REAL>::epsilon)
					return false;

				return true;
			}
			
			((function.get(x0) - y)*(function.get(x) - y) > REAL(0.0) ? x0 : x1) = x;
		}
		
		return false;
	}
};

//------------------------------------------------------------------------------

template <typename REAL>
class _RootSecant : public _RootBracket<REAL> {
protected:
	const U32 steps;

public:
	_RootSecant(U32 partitions = _Root<REAL>::PARTITIONS_DFLT, REAL epsilon = _Root<REAL>::EPS_DFLT, U32 steps = _Root<REAL>::STEPS_DFLT) :
		_RootBracket<REAL>(partitions, epsilon), steps(steps)
	{}

	/** Single solution equation solver */
	virtual bool findSingle(REAL &x, REAL y, const _Function<REAL> &function, REAL left, REAL right) const {
		REAL x0 = left, x1 = right;
		for (U32 i = 0; i < steps; ++i) {
			x = x1 - (function.get(x1) - y)*(x1 - x0)/(function.get(x1) - function.get(x0));
			x0 = x1;
			x1 = x;
			
			if (Math::abs(x0 - x1) < _RootBracket<REAL>::epsilon) {
				// check if the solution lies in boundaries
				if (x < left || x > right)
					return false;

				return true;
			}
		}
		
		return false;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_ROOT_H_*/
