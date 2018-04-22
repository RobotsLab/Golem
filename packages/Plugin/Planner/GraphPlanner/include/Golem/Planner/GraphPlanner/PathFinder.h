/** @file PathFinder.h
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
#ifndef _GOLEM_PLANNER_GRAPH_PLANNER_PATHFINDER_H_
#define _GOLEM_PLANNER_GRAPH_PLANNER_PATHFINDER_H_

//------------------------------------------------------------------------------

#include <Golem/Planner/Waypoint.h>
#include <Golem/Planner/Heuristic.h>
#include <Golem/Math/GraphSearch.h>
#include <Golem/Math/Sample.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Performance monitor */
//#define _PATHFINDER_PERFMON

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgPathFinder, Message)

//------------------------------------------------------------------------------

/** Waypoint generator */
class GOLEM_LIBRARY_DECLDIR WaypointGenerator : public Sample<Real> {
public:
	typedef shared_ptr<WaypointGenerator> Ptr;
	typedef std::vector<Ptr> Seq;
	friend class Desc;

	/** Seed type */
	enum Seed {
		/** Root */
		SEED_ROOT = 0x1,
		/** Goal */
		SEED_GOAL = 0x2,
		/** User */
		SEED_USER = 0x4,
	};

	/** Waypoint generator description */
	class GOLEM_LIBRARY_DECLDIR Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		typedef std::vector<Ptr> Seq;
		
		/** Name/id */
		std::string name;

		/** Seed type */
		Seed seed;
		/** Weight */
		Real weight;

		/** Mean value */
		ConfigspaceCoord mean;
		/** Delta */
		ConfigspaceCoord delta;

		/** maximum number of collision detection trials */
		U32 trials;

		/** bandwidth adaptation multiplier */
		Real bandwidthFactor;
		/** bandwidth adaptation trials */
		U32 bandwidthTrials;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}
		/** Nothing to do here */
		virtual ~Desc() {
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			name = "Default";
			seed = SEED_USER;
			weight = REAL_ONE;
			mean.fill(REAL_ZERO);
			delta.fill(REAL_2_PI);
			trials = 1000;
			bandwidthFactor = Real(1.5);
			bandwidthTrials = 10;
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (weight < REAL_ZERO)
				return false;
			if (bandwidthFactor <= REAL_EPS || Math::equals(bandwidthFactor, REAL_ONE, REAL_EPS))
				return false;
			return true;
		}
		/** Clones description. */
		virtual Ptr clone() const {
			return Ptr(new Desc(*this));
		}
		/** Creates a single generator from the description. */
		virtual WaypointGenerator::Ptr create(const Heuristic &heuristic) const {
			return WaypointGenerator::Ptr(new WaypointGenerator(heuristic, *this));
		}

		/** Clones a sequence of descriptions with a given seed */
		template <typename _Seq> static void clone(const _Seq& inp, _Seq &out, U32 seed = numeric_const<U32>::MAX) {
			for (typename _Seq::const_iterator i = inp.begin(); i != inp.end(); ++i)
				if ((*i)->seed & seed)
					out.insert(out.end(), i->clone());
		}
	};

	/** Name */
	inline const std::string& getName() const {
		return name;
	}
	/** Seed type */
	inline Seed getSeed() const {
		return seed;
	}
	/** Mean */
	inline const ConfigspaceCoord& getMean() const {
		return mean;
	}
	
	/** bandwidth adaptation multiplier */
	inline Real getBandwidthFactor() const {
		return bandwidthFactor;
	}
	/** bandwidth adaptation trials */
	inline Real getBandwidthTrials() const {
		return bandwidthTrials;
	}

	/** Generate waypoint; return success result */
	bool create(Heuristic::ThreadData* data, const ConfigspaceCoord& cc, Waypoint& w) const;

	/** Generate random waypoint; return success result */
	bool next(const Rand& rand, Waypoint& w, U32 bandwidthIter = 0) const {
		return next(heuristic.getThreadData(), rand, w, bandwidthIter);
	}

	/** Generate random waypoint; return success result */
	virtual bool next(Heuristic::ThreadData* data, const Rand& rand, Waypoint& w, U32 bandwidthIter) const;

protected:
	/** Heuristic reference */
	const Heuristic &heuristic;
	/** the configuration space properties */
	const Controller::State::Info stateInfo;

	/** Name */
	std::string name;
	
	/** Seed type */
	Seed seed;
	/** maximum number of collision detection trials */
	U32 trials;

	/** bandwidth adaptation multiplier */
	Real bandwidthFactor;
	/** bandwidth adaptation trials */
	U32 bandwidthTrials;

	/** Mean */
	ConfigspaceCoord mean;
	/** Range */
	ConfigspaceCoord min, max;

	/** Low bandwidth range */
	ConfigspaceCoord::Seq minLo, maxLo;
	/** High bandwidth range */
	ConfigspaceCoord::Seq minHi, maxHi;

	/** Generate random waypoint; return success result */
	virtual bool next(Heuristic::ThreadData* data, const ConfigspaceCoord& min, const ConfigspaceCoord& max, const Rand& rand, Waypoint& w) const;

	/** Setup generator given heuristic */
	WaypointGenerator(const Heuristic &heuristic, const WaypointGenerator::Desc& desc);
};

//------------------------------------------------------------------------------

/** Probabilistic path planner which uses a mixture of online and offline random graphs. */
class GOLEM_LIBRARY_DECLDIR PathFinder {
public:
	typedef shared_ptr<PathFinder> Ptr;
	friend class Desc;

	/** Index and cost of reaching a node */
	typedef std::pair<U32, Real> IndexCost;
	/** Neighbour map of a single node */
	typedef std::vector<IndexCost> IndexCostSeq;
	/** Neighbour map */
	typedef std::vector<IndexCostSeq> IndexCostSeqSeq;

	/** Path finder description */
	class GOLEM_LIBRARY_DECLDIR Desc {
	public:
		typedef shared_ptr<Desc> Ptr;

		/** Name */
		std::string name;

		/** Waypoints online generators */
		WaypointGenerator::Desc::Seq generatorsOnline;
		/** Waypoints offline generators */
		WaypointGenerator::Desc::Seq generatorsOffline;

		/** number of waypoints of the online graph */
		U32 graphSizeOnline;
		/** number of waypoints of the offline graph */
		U32 graphSizeOffline;
		/** average number of waypoint neighbours in the graph */
		U32 graphNeighbours;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}
		
		/** virtual destructor is required */
		virtual ~Desc() {}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(PathFinder, PathFinder::Ptr, Heuristic&)

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			name = "PathFinder";
			generatorsOnline.clear();
			generatorsOffline.clear();
			graphSizeOnline = 0;
			graphSizeOffline = 10000;
			graphNeighbours = 100;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			//if (generatorsOnline.empty() ? graphSizeOnline > 0 : graphSizeOnline == 0)
			//	return false;
			for (WaypointGenerator::Desc::Seq::const_iterator i = generatorsOnline.begin(); i != generatorsOnline.end(); ++i)
				if (!(*i)->isValid())
					return false;
			if (generatorsOffline.empty() ? graphSizeOffline > 0 : graphSizeOffline == 0)
				return false;
			for (WaypointGenerator::Desc::Seq::const_iterator i = generatorsOffline.begin(); i != generatorsOffline.end(); ++i)
				if (!(*i)->isValid())
					return false;
			if (graphNeighbours < 1 || graphNeighbours > graphSizeOnline + graphSizeOffline)
				return false;
			return true;
		}
	};

protected:
	// Graph search heuristic
	struct GOLEM_LIBRARY_DECLDIR GraphHeuristic {
		const Heuristic& heuristic;
		const Waypoint::Seq& graph;
		GraphSearch& graphSearch;

		const U32 graphPartition;
		const U32 graphSize;

		const IndexCostSeqSeq& neighbourMap;

		Heuristic::ThreadData* threadData;
		U32 neighbourIndex;
		IndexCost node;

#ifdef _PATHFINDER_PERFMON
		mutable U32 costOnline, costOffline;
		mutable U32 collisionOnline, collisionOffline, collisionOnlineState;
#endif
		// lightweight construction
		GraphHeuristic(const PathFinder* pathFinder, GraphSearch& graphSearch) :
			heuristic(pathFinder->heuristic), graph(pathFinder->graph), graphSearch(graphSearch),
			graphPartition(pathFinder->graphPartition), graphSize((U32)pathFinder->graph.size()),
			neighbourMap(pathFinder->neighbourMap)
		{
			threadData = heuristic.getThreadData();
			node.first = neighbourIndex = Node::IDX_UINI;
#ifdef _PATHFINDER_PERFMON
			costOnline = costOffline = 0;
			collisionOnline = collisionOffline = collisionOnlineState = 0;
#endif
		}
		// goal test
		inline bool goal(U32 i) const {
			return i == graphSearch.IDX_GOAL;
		}
		// Node expansion
		inline U32 expand(U32 i) {
			++node.first;

			if (i < graphPartition) {
				if (node.first >= graphSize)
					node.first = Node::IDX_UINI;
			}
			else {
				if (node.first <  graphPartition)
					neighbourIndex = 0;
				else {
					const IndexCostSeq& neighbours = neighbourMap[i];
					if (neighbourIndex < neighbours.size())
						node = neighbours[neighbourIndex++];
					else
						node.first = Node::IDX_UINI;
				}
			}

			return node.first;
		}
		// cost function
		inline Real cost(U32 i, U32 j) const {
			if (i >= graphPartition && j >= graphPartition) {
#ifdef _PATHFINDER_PERFMON
				++costOffline;
#endif
				return node.second;
			}
			else {
#ifdef _PATHFINDER_PERFMON
				++costOnline;
#endif
				return heuristic.cost(graph[i], graph[j]);
			}
		}
		// collision detection
		inline bool collision(U32 i, U32 j) const {
			if (graph[i].collides || graph[j].collides) {
#ifdef _PATHFINDER_PERFMON
				++collisionOffline;
#endif
				return true;
			}
			else {
#ifdef _PATHFINDER_PERFMON
				++collisionOnline;
				const bool collides = heuristic.collides(graph[i], graph[j], threadData);
				if (collides) ++collisionOnlineState;
				return collides;
#else
				return heuristic.collides(graph[i], graph[j], threadData);
#endif
			}
		}
	};

	// Graph search heuristic
	struct GOLEM_LIBRARY_DECLDIR GraphHeuristicBidirectional : public GraphHeuristic {
		struct Stop {};
		GraphHeuristicBidirectional& graphHeuristicReverse;
		bool init, stop;

		GraphHeuristicBidirectional(PathFinder* pathFinder, GraphSearch& graphSearch, GraphHeuristicBidirectional& graphHeuristicReverse) :
			GraphHeuristic(pathFinder, graphSearch), graphHeuristicReverse(graphHeuristicReverse), init(false), stop(false)
		{}
		// goal test
		inline bool goal(U32 i) const {
			if (graphHeuristicReverse.init) {
				if (graphHeuristicReverse.stop) throw Stop();
				return graphHeuristicReverse.graphSearch.getClosedVec()[i] != Node::IDX_UINI;
			}
			else {
				return i == graphSearch.IDX_GOAL;
			}
		}
	};

	// Bidirectional search
	struct GOLEM_LIBRARY_DECLDIR BidirectionalSearch : public Runnable {
		PathFinder* pathFinder;
		GraphHeuristicBidirectional graphHeuristicForward, graphHeuristicBackward;
		U32 nodeIndex;

		golem::CriticalSection cs;
		bool forward;

		BidirectionalSearch(PathFinder* pathFinder, Node::Seq& nodePath) :
			pathFinder(pathFinder),
			graphHeuristicForward(pathFinder, pathFinder->graphSearchForward, graphHeuristicBackward),
			graphHeuristicBackward(pathFinder, pathFinder->graphSearchBackward, graphHeuristicForward)
		{
			// Parallels
			Parallels *parallels = pathFinder->context.getParallels();
			const bool bidirectional = parallels && parallels->getNumOfThreads() >= 2;
			if (!bidirectional)
				pathFinder->context.notice("PathFinder::findPath(): Bidirectional graph search requires at least 2 Parallels threads\n");

			// initialise graph search data
			nodeIndex = Node::IDX_UINI;
			forward = true;

			// run Parallels
			if (bidirectional) {
				// launch Parallels
				for (U32 t = 0; t < 2; ++t) {
					Job* job = parallels->startJob(this);
					if (!job)
						throw MsgPathFinder(Message::LEVEL_ERROR, "PathFinder.%s::findPath(): Unable to start job", pathFinder->name.c_str());
				}
				(void)parallels->joinJobs(MSEC_TM_U32_INF);
			}
			else
				run();

			// extract path
			if (nodeIndex != Node::IDX_UINI) {
				graphHeuristicForward.graphSearch.extract(nodeIndex, nodePath, nodePath.end());
				Node::Seq tmp;
				graphHeuristicBackward.graphSearch.extract(nodeIndex, tmp, tmp.end());
				if (!tmp.empty()) {
					tmp.pop_back(); // do not duplicate nodeIndex
					nodePath.insert(nodePath.end(), tmp.rbegin(), tmp.rend()); // reverse order
				}
			}
		}

		void run() {
			// choose current search direction
			bool forward;
			{
				CriticalSectionWrapper csw(cs);
				forward = this->forward;
				this->forward = !this->forward;
			}

			GraphHeuristicBidirectional& graphHeuristic = forward ? graphHeuristicForward : graphHeuristicBackward;

			try {
				graphHeuristic.threadData = pathFinder->heuristic.getThreadData(); // set thread data

				graphHeuristic.graphSearch.initialise((U32)pathFinder->graph.size());
				graphHeuristic.init = true; // inform the other thread

				const U32 nodeIndex = graphHeuristic.graphSearch.find(graphHeuristic);
				graphHeuristic.stop = true; // inform the other thread

				CriticalSectionWrapper csw(cs);
				if (this->nodeIndex == Node::IDX_UINI) // this thread is the first one
					this->nodeIndex = nodeIndex;
			}
			catch (const GraphHeuristicBidirectional::Stop&) {}
		}
	};

	/** Heuristic */
	golem::Heuristic& heuristic;
	/** Arm controller interface */
	golem::Controller& controller;
	/** Context object */
	golem::Context &context;
	/** the configuration space properties */
	const Controller::State::Info stateInfo;

	/** Name */
	std::string name;

	/** Offline waypoint generator descriptions */
	WaypointGenerator::Desc::Seq generatorsOffline;
	/** Online waypoint generator descriptions */
	WaypointGenerator::Desc::Seq generatorsOnline;

	/** number of waypoints of the online graph */
	U32 graphSizeOnline;
	/** number of waypoints of the offline graph */
	U32 graphSizeOffline;
	/** average number of waypoint neighbours in the graph */
	U32 graphNeighbours;
	/** graph partition point */
	U32 graphPartition;

	/** Forward graph search */
	GraphSearch graphSearchForward;
	/** Backward graph search */
	GraphSearch graphSearchBackward;
	/** Waypoint graph */
	Waypoint::Seq graph;
	/** Neighbour map */
	IndexCostSeqSeq neighbourMap;
	/** Maximum cost */
	Real costMax;

	/** Random number generator */
	Rand rand;

	/** Generates online or offline graph. */
	virtual void generateGraph(U32 begin, U32 end, const WaypointGenerator::Desc::Seq& generators);
	
	/** Generates path cost map. */
	virtual void generateNeighbourMap(U32 begin, U32 end);

	/** Sets root and goal on the graph. */
	virtual void setupGraph(const ConfigspaceCoord& root, const ConfigspaceCoord& cgoal, const WorkspaceChainCoord& wgoal);

	/** Creates PathFinder from the description.
	* @param desc		PathFinder description
	*/
	void create(const Desc& desc);

	/** PathFinder constructor */
	PathFinder(Heuristic& heuristic);

public:
	/** PathFinder destructor */
	virtual ~PathFinder();

	/** Allocate graph resources. */
	void allocateGraph(U32 graphSizeOnline, U32 graphSizeOffline, U32 graphNeighbours);

	/** Generates offline graph with root and goal configurations.
	* @param	generators	optional custimised graph generators
	*/
	virtual void generateOfflineGraph(const WaypointGenerator::Desc::Seq* generators = NULL);

	/** Generates online graph with root and goal configurations.
	 * @param	root		path begin (root) in the configuration space
	 * @param	goal		path end (goal) in the configuration space
	 * @param	generators	optional custimised graph generators
	 */
	virtual void generateOnlineGraph(const ConfigspaceCoord &root, const ConfigspaceCoord& goal, const WaypointGenerator::Desc::Seq* generators = NULL);

	/** Generates online graph with root and goal configurations.
	 * @param	root		path begin (root) in the configuration space
	 * @param	goal		path end (goal) in the workspace
	 * @param	generators	optional custimised graph generators
	 */
	virtual void generateOnlineGraph(const ConfigspaceCoord &root, const WorkspaceChainCoord& goal, const WaypointGenerator::Desc::Seq* generators = NULL);

	/** Finds path (a sequence of waypoints) in the configuration space from root to goal given graph.
	* @param	goal		path end (goal) in the configuration space
	* @param	path		a sequence of waypoints
	 * @param	iter		insertion pointer
	 * @return				<code>true</code> success; <code>false</code> otherwise 
	 */
	virtual bool findPath(const ConfigspaceCoord &goal, Waypoint::Seq &path, Waypoint::Seq::iterator iter);

	/** Offline waypoint generator descriptions */
	const WaypointGenerator::Desc::Seq& getOfflineGenerators() const {
		return generatorsOffline;
	}
	/** Online waypoint generator descriptions */
	const WaypointGenerator::Desc::Seq& getOnlineGenerators() const {
		return generatorsOnline;
	}

	/** number of waypoints of the online graph */
	inline U32 getOnlineGraphSize() const {
		return graphSizeOnline;
	}
	/** number of waypoints of the offline graph */
	inline U32 getOfflineGraphSize() const {
		return graphSizeOffline;
	}
	/** average number of waypoint neighbours in the graph */
	inline U32 getGraphNeighbours() const {
		return graphNeighbours;
	}
	/** Returns graph partition point */
	inline U32 getGraphPartition() const {
		return graphPartition;
	}

	/** Maximum cost */
	inline Real getCostMax() const {
		return costMax;
	}

	/** Waypoint graph */
	const Waypoint::Seq& getGraph() const {
		return graph;
	}

	/** Access to heuristic */
	inline const Heuristic &getHeuristic() const {
		return heuristic;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PLANNER_GRAPH_PLANNER_PATHFINDER_H_*/
