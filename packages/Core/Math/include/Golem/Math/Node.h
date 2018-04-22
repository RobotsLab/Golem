/** @file Node.h
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
#ifndef _GOLEM_MATH_NODE_H_
#define _GOLEM_MATH_NODE_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Math.h>
#include <vector>
//#include <unordered_set>
//#include <unordered_map>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Node base class.
*/
class Node {
public:
	/** Node sequence */
	typedef std::vector<Node> Seq;

	/** Index */
	typedef U32 Index;
	/** Index pair */
	typedef U64 IndexPair;

	/** Index set */
	//typedef std::unordered_set<U32> IndexSet;
	/** Index pair set */
	//typedef std::unordered_set<U64> IndexPairSet;

	/** Goal node index */
	static const U32 IDX_GOAL = 0;
	/** Root node index */
	static const U32 IDX_ROOT = 1;
	/** Uninitialised node index */
	static const U32 IDX_UINI;// = numeric_const<U32>::MAX;

	/** Zero/uninitialised cost */
	static const Real COST_ZERO;
	/** Infinite cost (node unreachable) */
	static const Real COST_INF;
	
	/** Cost map */
	//typedef std::unordered_map<U64, Real> CostMap;

	/** Index comparator */
	struct index_less {
		inline bool operator () (const Node &left, const Node &right) const {
			return left.index < right.index;
		}
	};
	
	/** Cost comparator */
	struct cost_less {
		inline bool operator () (const Node &left, const Node &right) const {
			return left.cost < right.cost;
		}
	};

	/** Make pair */
	static inline U64 makeIndexPair(U32 left, U32 right) {
		return (U64)right << 32 | left;
	}
	/** Make pair */
	static inline U64 makeIndexPair(const Node& left, const Node& right) {
		return makeIndexPair(left.index, right.index);
	}

	/** Node index, index E <0, size - 1> */
	U32 index;
	/** (Minimal) cost of reaching the goal node, cost E <COST_ZERO, COST_INF> */
	Real cost;
	/** Collision */
	bool collides;

	Node(U32 index = IDX_UINI, Real cost = COST_ZERO, bool collides = false) : index(index), cost(cost), collides(false) {
	}
	Node(const Node& node) : index(node.index), cost(node.cost), collides(node.collides) {
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_NODE_H_*/
