/** @file Waypoint.h
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
#ifndef _GOLEM_PLANNER_WAYPOINT_H_
#define _GOLEM_PLANNER_WAYPOINT_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Sample.h>
#include <Golem/Math/Node.h>
#include <Golem/Ctrl/Controller.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Waypoint */
class Waypoint : public Node, public Sample<Real> {
public:
	/** Sequence of waypoints. */
	typedef std::vector<Waypoint> Seq;
	/** Sequence of waypoint pointers. */
	typedef std::vector<const Waypoint*> PtrSeq;

	/** Controller position in configuration space coordinates. */
	ConfigspaceCoord cpos;

	/** Tool positions and orientations. */
	WorkspaceChainCoord wpos;
	/** All joints positions and orientations. */
	WorkspaceJointCoord wposex;
	/** Positions and orientations of chain origins. */
	WorkspaceChainCoord wposchainex;
	/** Tool orientations as quaternions. */
	Chainspace::Coord<Quat> qrot;

	/** Path distance. */
	Real dist;
	/** Differential constraints. */
	ConfigspaceCoord dbegin, dend;

	/** Default constructor initialises only GraphSearch::Node */
	Waypoint(U32 index = Node::IDX_UINI, Real cost = Node::COST_ZERO) : Node(index, cost), dist(REAL_ZERO) {
	}
	/** Constructs Waypoint from node */
	Waypoint(const Controller &controller, const ConfigspaceCoord &cpos, bool tr = true, bool ex = true, U32 index = Node::IDX_UINI, Real cost = Node::COST_ZERO) : Node(index, cost), dist(REAL_ZERO) {
		setup(controller, cpos, tr, ex);
	}

	/** Setup waypoint */
	void setup(const Controller &controller, const ConfigspaceCoord &cpos, bool tr, bool ex) {
		this->cpos = cpos;
		setup(controller, tr, ex);
	}
	/** Setup waypoint */
	void setup(const Controller &controller, bool tr, bool ex) {
		const Controller::State::Info& stateInfo = controller.getStateInfo();

		if (ex) {
			controller.jointForwardTransform(cpos, wposex);
			for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
				const Chain* chain = controller.getChains()[i];
				const Chainspace::Index linkedChainIndex = chain->getLinkedChainIndex();

				wpos[i] = wposex[stateInfo.getJoints(i).end() - 1]; // the last joint in the chain i
				wposchainex[i].multiply(linkedChainIndex < i ? wpos[linkedChainIndex] : controller.getGlobalPose(), chain->getLocalPose());
			}
		}
		else {
			controller.chainForwardTransform(cpos, wpos);
		}

		if (tr) {
			for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
				wpos[i].multiply(wpos[i], controller.getChains()[i]->getReferencePose()); // reference pose
				qrot[i].fromMat33(wpos[i].R);
			}
		}
	}
	
	/** Compute distance */
	template <typename _Dist> static void makeDist(Seq& seq, _Dist dist) {
		if (seq.size() < 2)
			throw Message(Message::LEVEL_ERROR, "Waypoint::makeDist(): invalid size");
		
		seq.front().dist = REAL_ZERO;
		for (Waypoint::Seq::iterator i = seq.begin(), j = ++seq.begin(); j != seq.end(); ++i, ++j)
			j->dist = i->dist + dist(i->cpos, j->cpos);
		if (seq.back().dist < REAL_EPS)
			throw Message(Message::LEVEL_ERROR, "Waypoint::makeDist(): invalid distance");
		
		const Real norm = REAL_ONE/seq.back().dist;
		for (Waypoint::Seq::iterator i = seq.begin(); i != seq.end(); ++i)
			i->dist *= norm;
	}

	/** Compute distance */
	static void makeDiff(Seq& seq, const Configspace::Map& indexMap, const ConfigspaceCoord& dbegin, const ConfigspaceCoord& dend) {
		if (seq.size() < 2)
			throw Message(Message::LEVEL_ERROR, "Waypoint::makeDiff(): invalid size");

		seq.front().dbegin = dbegin;
		seq.back().dend = dend;
		for (Waypoint::Seq::iterator i = seq.begin(), j = ++seq.begin(); j != seq.end(); ++i, ++j) {
			j->dbegin.subtract(indexMap, j->cpos, i->cpos);
			i->dend = j->dbegin;
		}
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PLANNER_WAYPOINT_H_*/
