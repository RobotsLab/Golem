/** @file RBOptimisation.h
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
#ifndef _GOLEM_MATH_RBOPTIMISATION_H_
#define _GOLEM_MATH_RBOPTIMISATION_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Optimisation.h>
#include <Golem/Math/RB.h>
#include <vector>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Rigid body optimisation vector */
template <typename _RBCoord, size_t _RB_N, size_t _M = 0> class RBVec {
public:
	/** Sequence of vectors*/
	typedef std::vector<RBVec> Seq;
	/** Rigid body coordinate */
	typedef _RBCoord Coord;
	/** Transformations */
	static const size_t RB_N = _RB_N;
	/** Reserved dimensions */
	static const size_t M = _M;
	/** Total dimensions */
	static const size_t N = RB_N*_RBCoord::N + M;

	/** Data */
	golem::Real data[N];
	
	/** Access to reserved dimensions. */
	inline golem::Real& getReserved(size_t idx) {
		return data[RB_N*_RBCoord::N + idx];
	}
	/** Access to reserved dimensions. */
	inline const golem::Real& getReserved(size_t idx) const {
		return data[RB_N*_RBCoord::N + idx];
	}
	
	/** Access vector of coordinates. */
	inline _RBCoord& getRB(size_t idx) {
		return (_RBCoord&)data[idx*_RBCoord::N];
	}
	/** Access vector of coordinates. */
	inline const _RBCoord& getRB(size_t idx) const {
		return (const _RBCoord&)data[idx*_RBCoord::N];
	}

	/** Access vector of reals as an array. */
	inline golem::Real& operator [] (size_t idx) {
		return data[idx];
	}
	/** Access vector of reals as an array. */
	inline const golem::Real& operator [] (size_t idx) const {
		return data[idx];
	}
};

/** Rigid body optimisation heuristic. */
template <typename _RBCoord, size_t _RB_N, size_t _M = 0> class RBHeuristic : public golem::DEHeuristicFunc<golem::U32, golem::RBVec<_RBCoord, _RB_N, _M>, golem::Real> {
public:
	/** Heuristic base class */
	typedef golem::DEHeuristicFunc<golem::U32, golem::RBVec<_RBCoord, _RB_N, _M>, golem::Real> Heuristic;

	/** Thread data */
	typedef typename Heuristic::ThreadData ThreadData;
	/** Vector type */
	typedef typename Heuristic::Vec Vec;
	/** Value type */
	typedef typename Heuristic::Type Type;

	/** Initialisation */
	RBHeuristic(golem::Context& context) : Heuristic(context, Vec::N + _M) {
	}
};

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_MATH_RBOPTIMISATION_H_*/
