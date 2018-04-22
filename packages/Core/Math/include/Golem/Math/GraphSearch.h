/** @file GraphSearch.h
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
#ifndef _GOLEM_MATH_GRAPHSEARCH_H_
#define _GOLEM_MATH_GRAPHSEARCH_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Math.h>
#include <Golem/Math/Node.h>
#include <iterator>
#include <vector>
#include <set>
#include <algorithm>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Abstract class implementing a generic A* graph search algorithm.
*
*	A* graph search algorithm searches for the minimum cost path between root and goal nodes.
*	By overriding pure abstract findCost(), nextNode() and other functions
*	from protected part of the class, one can obtain eg. breath-first or depth-first search
*	strategies, as well as different memory management strategies for finite or infinite
*	(say very large) number of nodes.
*	GraphSearch has a nearly optimal design - the only penalty (negligible in practise) are virtual
*	function calls. GraphSearch uses some extra improvements like efficient node cost sorting.
*
* @see "Artificial Intelligence: A new synthesis", Nils J. Nilsson, 1998, Morgan Kaufmann
*/
class GraphSearch {
protected:
	/** Sorted set with no more than ~2log2(n+1) extra complexity (red-black tree) */
	typedef std::set<Node, Node::cost_less> NodeRank;

	U32 _size;
	U32 nextNodeIndex;

	std::vector<Real> costMat;
	std::vector<U32> openVec;
	std::vector<U32> closedVec;

	/** node cost rank */
	NodeRank nodeRank;

public:
	/** Goal node index */
	const U32 IDX_GOAL;
	/** Root node index */
	const U32 IDX_ROOT;

	/**	Creates GraphSearch and initialises basic graph data structures.
	*	@param	size		current size of the graph (number of nodes).
	*/
	GraphSearch(U32 IDX_GOAL = Node::IDX_GOAL, U32 IDX_ROOT = Node::IDX_ROOT, U32 size = 0) : IDX_GOAL(IDX_GOAL), IDX_ROOT(IDX_ROOT) {
		initialise(size);
	}
	
	/** Heuristic::goal() - Goal test function.
	*
	*	@param	i		node to be tested, i E <0, size - 1>
	*	@return			true if i is the goal
	*/

	/** Heuristic::expand() - Node expansion function iterates through all nodes.
	*
	*	@param	i		node to be expanded, i E <0, size - 1>
	*	@return			destination node directly reacheable from node i, or
	*					IDX_UINI to finish expansion
	*/

	/** Heuristic::cost() - graph heuristic function.
	*
	*	Graph heuristic function is defined as f() := g() + h(), where
	*	h() is the estimated cost of reaching the goal node,
	*	g() is the relative cost of reaching node j from i
	*	The root and the goal nodes have a priori specified indices IDX_ROOT and IDX_GOAL.
	*
	*	@param	i		current node, j E <0, size - 1>
	*	@param	j		destination node, i E <0, size - 1>
	*	@return			<COST_ZERO, COST_INF> estimated cost of reaching the goal node from j
	*/

	/** Heuristic::collision() - collision detection function
	*	@param	i		current node, j E <0, size - 1>
	*	@param	j		destination node, i E <0, size - 1>
	*	@return			<COST_ZERO, COST_INF> estimated cost of reaching the goal node from j
	*/

	/** Search algorithm finds the minimum cost path between root and goal nodes.
	*
	*	The first and the last element of the returned sequence are suitably:
	*	- the root node
	*	- a node from which the goal node is directly reacheable
	*	The returned index is IDX_UINI if there is no finite-cost route to the goal node.
	*
	*	@param	heuristic	graph search heuristic
	*	@return				goal node index
	*/
	template <class _Heuristic> U32 find(_Heuristic& heuristic) {
		// the current node.
		U32 nCurrent = IDX_ROOT;
		// the node through which passes the best path to nCurrent.
		U32 rCurrent = IDX_ROOT;
		// a cost of reaching nCurrent.
		Real cCurrent = Node::COST_ZERO;
		// clear node cost rank
		nodeRank.clear();

		for (;;) {
			// The best path to nCurrent goes through rCurrent.
			closedVec[nCurrent] = rCurrent;

			// Break the loop if nCurrent is the goal.
			if (heuristic.goal(nCurrent))
				break;

			// then remove nCurrent from open.
			openVec[nCurrent] = Node::IDX_UINI;

			for (;;) {
				// Expand the current node nCurrent,
				// find for all possible destination nodes j
				const U32 j = heuristic.expand(nCurrent);

				// Break if there are no more destination nodes
				if (j == Node::IDX_UINI)
					break;
				// Do not expand if the destination node j is:
				// equal the expanded node nCurrent, or is already on close.
				if (j == nCurrent || closedVec[j] != Node::IDX_UINI)
					continue;

				// Calculate the cost of reaching j through nCurrent.
				Real cNew = heuristic.cost(nCurrent, j);

				// if j is reachable
				if (cNew < Node::COST_INF) {
					// Calculate the total cost
					cNew += cCurrent;

					// Extract a node, through which passes the best path to j.
					const U32 k = openVec[j];
					// Redirect the best path to j if there was no
					// path to j before, or the new path is less costly.
					if (k == Node::IDX_UINI) {
						if (heuristic.collision(nCurrent, j))
							continue;
					}
					else {
						const Real cOld = costMat[j];
						if (cOld <= cNew || heuristic.collision(nCurrent, j))
							continue;
						nodeRank.erase(Node(j, cOld));
					}

					// the best path to j passes nCurrent
					openVec[j] = nCurrent;
					// Update cost map and rank
					costMat[j] = cNew;
					nodeRank.insert(Node(j, cNew));
				}
			}

			// Get a node nCurrent with the lowest cost value.
			NodeRank::iterator node = nodeRank.begin();
			// Break the loop if nodeRank/open is empty - goal node is unreacheable.
			if (node == nodeRank.end())
				return Node::IDX_UINI;

			nCurrent = node->index;
			rCurrent = openVec[nCurrent];
			cCurrent = costMat[nCurrent];

			// remove nCurrent from nodeRank
			nodeRank.erase(node);
		}

		return nCurrent;
	}

	/** Extracts the minimum cost path between root and goal nodes.
	*
	*	@param	nCurrent	start node
	*	@param	seq			collection of nodes of the graph
	*	@param	iter		iterator locating the insertion point
	*	@return				iterator locating the first element of the path, NULL if there is no finite-cost route to the goal node
	*/
	template <class _Seq, class _Iter> _Iter extract(U32 nCurrent, _Seq& seq, _Iter iter) {
		if (nCurrent < _size) {
			// Build the outcoming path as an array of waypoints by
			// simple iteration close(n) -> n until n points the root.

			// Goal node: put elements in reverse order, requires a suitable (implicit) copy constructor!
			iter = seq.insert(iter, Node(nCurrent, Node::COST_ZERO));

			for (;;) {
				const U32 rCurrent = closedVec[nCurrent];

				// break if the node has not been initialised
				if (rCurrent >= _size || rCurrent == nCurrent)
					break;

				// Other nodes: put elements in reverse order, requires a suitable (implicit) copy constructor!
				iter = seq.insert(iter, Node(rCurrent, costMat[nCurrent]));

				// break if the root node has been reached
				if (rCurrent == IDX_ROOT)
					break;

				nCurrent = rCurrent;
			}
		}

		return iter;
	}

	/** Cost matrix */
	inline const std::vector<Real>& getCostMat() const {
		return costMat;
	}
	
	/** Open nodes vector */
	inline const std::vector<U32>& getOpenVec() const {
		return openVec;
	}
	
	/** Closed nodes vector */
	inline const std::vector<U32>& getClosedVec() const {
		return closedVec;
	}

	/** Sets number of nodes
	*	@param	size	current size of the graph (number of nodes).
	*/
	inline void initialise(U32 size) {
		this->_size = size;
		if (size > 0) {
			costMat.assign(size, Node::COST_ZERO);
			openVec.assign(size, Node::IDX_UINI);
			closedVec.assign(size, Node::IDX_UINI);
		}
	}

	/** Current number of nodes */
	inline U32 size() const {
		return _size;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_GRAPHSEARCH_H_*/
