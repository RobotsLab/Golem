/** @file Heuristic.h
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
#ifndef _GOLEM_PLANNER_HEURISTIC_H_
#define _GOLEM_PLANNER_HEURISTIC_H_

//------------------------------------------------------------------------------

#include <Golem/Planner/Waypoint.h>
#include <Golem/Sys/Message.h>
#include <set>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Performance monitor */
//#define _HEURISTIC_PERFMON

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgHeuristic, Message)
MESSAGE_DEF(MsgHeuristicInvalidDesc, MsgHeuristic)
MESSAGE_DEF(MsgHeuristicInvalidDescChains, MsgHeuristic)
MESSAGE_DEF(MsgHeuristicInvalidDescJoints, MsgHeuristic)
MESSAGE_DEF(MsgHeuristicCollision, MsgHeuristic)
MESSAGE_DEF(MsgHeuristicBoundsCreate, MsgHeuristic)
MESSAGE_DEF(MsgHeuristicBoundsExpand, MsgHeuristic)
MESSAGE_DEF(MsgHeuristicInvalidScale, MsgHeuristic)

//------------------------------------------------------------------------------

/** Waypoint path optimisation atomic routines */
class Heuristic {// : protected Runnable {
public:
	typedef shared_ptr<Heuristic> Ptr;
	friend class Desc;

	typedef std::vector<golem::U32> U32Seq;
	typedef std::vector<golem::Mat34> Mat34Seq;
	typedef std::vector<golem::Chainspace::Index> ChainspaceSeq;
	typedef std::vector<golem::Configspace::Index> ConfigspaceSeq;

	/** Bounds description comparator */
	struct compare_desc {
		bool operator() (const golem::Bounds::Desc::Ptr& l, const golem::Bounds::Desc::Ptr& r) const {
			return l.get() < r.get();
		}
	};
	/** Bounds description set */
	typedef std::set<golem::Bounds::Desc::Ptr, compare_desc> BoundsSet;
	/** Chain bounds description set */
	typedef Chainspace::Coord<BoundsSet> ChainBoundsSet;
	/** Joint bounds description set */
	typedef Configspace::Coord<BoundsSet> JointBoundsSet;

	/** Thread data */
	class ThreadData {
	protected:
		typedef std::map<U32, ThreadData> Set;
		friend class Heuristic;

		/** Chain bounds */
		Chainspace::Coord<golem::Bounds::Seq> chainBounds;	
		/** Poses of bounds of joints */
		Chainspace::Coord<Mat34Seq> chainBoundsPoses;
		/** Joint bounds */
		Configspace::Coord<golem::Bounds::Seq> jointBounds;	
		/** Poses of bounds of joints */
		Configspace::Coord<Mat34Seq> jointBoundsPoses;
	};

	/** Chain parameters */
	class ChainDesc {
	public:
		/** Chainspace sequence. */
		typedef std::vector<ChainDesc> Seq;
		/** Chainspace coordinates. */
		typedef Chainspace::Coord<ChainDesc*> ChainSeq;

		/** Enabled/disabled observational computation */
		bool enabledObs;
		/** Enabled/disabled linear distance computation */
		bool enabledLin;
		/** Enabled/disabled angular distance computation */
		bool enabledAng;

		/** linear-angular convex combination distance factor */
		Real distNorm;
		/** Linear distance maximum */
		Real distLinearMax;
		/** Angular distance maximum */
		Real distAngularMax;

		/** configuration space and workspace distance convex combination factor */
		Real distConfigspaceWorkspaceNorm;

		/** Bounds */
		golem::Bounds::Desc::Seq bounds;

		/** Sets the parameters to the default values */
		void setToDefault() {
			enabledObs = true;
			enabledLin = true;
			enabledAng = true;
			distNorm = Real(0.8);//Real(0.7); // [0, 1]
			distLinearMax = Real(0.25);
			distAngularMax = Real(0.25);
			distConfigspaceWorkspaceNorm = Real(0.8); // [0, 1]
			bounds.clear();
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (distNorm < REAL_ZERO || distNorm > REAL_ONE)
				return false;
			if (distLinearMax <= REAL_ZERO || distAngularMax <= REAL_ZERO)
				return false;
			if (distConfigspaceWorkspaceNorm < REAL_ZERO || distConfigspaceWorkspaceNorm > REAL_ONE)
				return false;
			for (golem::Bounds::Desc::Seq::const_iterator i = bounds.begin(); i != bounds.end(); ++i)
				if (*i == NULL || !(*i)->isValid())
					return false;
			return true;
		}
	};
	
	/** Joint parameters */
	class JointDesc {
	public:
		/** Configspace sequence. */
		typedef std::vector<JointDesc> Seq;
		/** Configspace coordinates. */
		typedef Configspace::Coord<JointDesc*> JointSeq;

		/** Colliding bounds */
		bool collisionBounds;
		/** Colliding joints list */
		U32Seq collisionJoints;
		
		/** Default position */
		Real dfltPos;
		
		/** Enabled/disabled planning */
		bool enabled;
		/** Enabled/disabled interpolation */
		bool interpolate;
		/** Default position distance factor */
		Real distDfltFac;
		/** maximum distance between two configuration space coordinates */
		Real distMax, distMaxSqr;

		/** Bounds */
		golem::Bounds::Desc::Seq bounds;

		/** Sets the parameters to the default values */
		void setToDefault() {
			collisionBounds = true;
			collisionJoints.clear();
			dfltPos = Real(0.0);
			enabled = true;
			interpolate = true;
			distDfltFac = Real(0.01);
			distMax = Real(0.75)*REAL_PI;
			bounds.clear();
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (!Math::isFinite(dfltPos))
				return false;
			if (!Math::isFinite(distDfltFac) || distDfltFac < REAL_ZERO)
				return false;
			if (!Math::isFinite(distMax) || distMax < REAL_ZERO)
				return false;
			for (golem::Bounds::Desc::Seq::const_iterator i = bounds.begin(); i != bounds.end(); ++i)
				if (*i == NULL || !(*i)->isValid())
					return false;
			return true;
		}
	};

	/** Collision detection description */
	class CollisionDesc {
	public:
		/** Enabled/disabled */
		bool enabled;		
		/** collision path distance delta */
		Real pathDistDelta;
		/** Objects skin thickness */
		Real skinThickness;
		
		/** Initial ollision bounds	*/
		Bounds::Seq collisionBounds;

		/** Sets the parameters to the default values */
		void setToDefault() {
			enabled = true;
			pathDistDelta = Real(0.025);
			skinThickness = Real(0.01);
			collisionBounds.clear();
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (!Math::isFinite(pathDistDelta) || pathDistDelta <= REAL_EPS)
				return false;
			if (!Math::isFinite(skinThickness) || skinThickness < REAL_ZERO)
				return false;
			for (Bounds::Seq::const_iterator i = collisionBounds.begin(); i != collisionBounds.end(); ++i)
				if (*i == NULL)
					return false;
			return true;
		}
	};

	/** Objective/cost function description */
	class CostDesc {
	public:
		/** root distance factor */
		Real distRootFac;
		/** default position distance factor */
		Real distDfltFac;
		/** min/max limit distance factor */
		Real distLimitsFac;
		
		/** Sets the parameters to the default values */
		void setToDefault() {
			distRootFac = Real(0.9);
			distDfltFac = Real(0.01);
			distLimitsFac = Real(0.01);
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (distRootFac < REAL_ZERO || distDfltFac < REAL_ZERO || distLimitsFac < REAL_ZERO)
				return false;
			return true;
		}
	};

	/** Heuristic description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;

		/** Chain parameters */
		ChainDesc::Seq chains;
		/** Joint parameters */
		JointDesc::Seq joints;

		/** Objective/cost function description */
		CostDesc costDesc;
		/** Collision detection description */
		CollisionDesc collisionDesc;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** virtual destructor is required */
		virtual ~Desc() {}
		
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(Heuristic, Heuristic::Ptr, Controller&)
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			chains.clear();
			joints.clear();
			costDesc.setToDefault();
			collisionDesc.setToDefault();
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (chains.empty() || joints.empty())
				return false;
			for (ChainDesc::Seq::const_iterator i = chains.begin(); i != chains.end(); ++i)
				if (!i->isValid())
					return false;
			for (JointDesc::Seq::const_iterator i = joints.begin(); i != joints.end(); ++i)
				if (!i->isValid())
					return false;
			
			if (!costDesc.isValid() || !collisionDesc.isValid())
				return false;

			return true;
		}
	};

protected:
	/** Path data */
	//class PathData {
	//public:
	//	typedef std::vector<PathData> Seq;

	//	/** Left waypoint index */
	//	U32 leftIndex;
	//	/** Right waypoint index */
	//	U32 rightIndex;

	//	/** Left waypoint coordinates */
	//	const Real* leftCC;
	//	/** Right waypoint coordinates */
	//	const Real* rightCC;

	//	/** Segment size */
	//	U32 size;
	//	/** Segment index */
	//	U32 index;

	//	/** Collision state */
	//	bool collides;

	//	/** Default */
	//	inline PathData() {}
	//	/** Initialisation */
	//	inline PathData(const Waypoint& left, const Waypoint& right, U32 size) {
	//		create(left, right, size);
	//	}
	//	/** Create */
	//	inline void create(const Waypoint& left, const Waypoint& right, U32 size) {
	//		this->leftIndex = left.index;
	//		this->rightIndex = right.index;
	//		this->leftCC = left.cpos.data();
	//		this->rightCC = right.cpos.data();
	//		this->index = this->size = size;
	//		this->collides = false;
	//	}
	//};

	/** Controller */
	golem::Controller &controller;
	/** Context object */
	golem::Context &context;
	
	/** Heuristic description */
	Desc desc;
	/** Chainspace coordinates. */
	ChainDesc::ChainSeq chainDesc;
	/** Configspace coordinates. */
	JointDesc::JointSeq jointDesc;
	
	/** the configuration space properties */
	Controller::State::Info stateInfo;
	
	/** Linear distance maximum */
	RealCoord<Chainspace> distLinearMax;
	/** Angular distance maximum */
	RealCoord<Chainspace> distAngularMax;
	/** maximum distance between two configuration space coordinates */
	RealCoord<Configspace> distMax;
	/** Enabled chainspace */
	ChainspaceSeq chainspaceSeq;
	/** Enabled configspace */
	ConfigspaceSeq configspaceSeq;
	/** Scale */
	Real scale, scaleInv;
	/** Default pose */
	ConfigspaceCoord dfltPos;
	/** Minimum override */
	GenCoordConfigspace min;
	/** Maximum override */
	GenCoordConfigspace max;
	
	/** Thread data */
	mutable ThreadData::Set threadData;
	/** Path data */
	//mutable PathData::Seq pathData;
	/** Path iteration */
	//mutable golem::U32 pathIndex;
	/** Critical section */
	mutable golem::CriticalSection cs;

	/** The collection of bounds of objects */
	golem::Bounds::Seq collisionBounds;	
	/** Chain bounds */
	ChainBoundsSet chainBoundsDesc;
	/** Joint bounds */
	JointBoundsSet jointBoundsDesc;

	template <typename _Ptr, typename _Bounds, typename _Poses> void addBounds(_Ptr begin, _Ptr end, _Bounds& bounds, _Poses& poses) const {
		for (_Ptr j = begin; j != end; ++j) {
			Bounds::Ptr pBounds = (*j)->create();
			if (pBounds == NULL)
				throw MsgHeuristicBoundsCreate(Message::LEVEL_CRIT, "Heuristic::addBounds(): unable to create bounds: %s", (*j)->getName());

			//context.debug("Heuristic::addBounds(): group #%08x, name: %s\n", pBounds->getGroup(), pBounds->getName());
			bounds.push_back(pBounds);
			poses.push_back(pBounds->getPose());
		}
	}
	template <typename _Idx, typename _Seq, typename _Bounds, typename _Poses> void syncBounds(_Idx begin, _Idx end, const _Seq& seq, _Bounds& bounds, _Poses& poses) const {
		for (_Idx i = begin; i < end; ++i) {
			Bounds::Desc::SeqPtr pBoundsDescSeq = seq[i]->getBoundsDescSeq();
			if (pBoundsDescSeq->empty())
				continue;

			bounds[i].clear();
			poses[i].clear();

			addBounds(pBoundsDescSeq->begin(), pBoundsDescSeq->end(), bounds[i], poses[i]);
		}
	}

	void setPose(const Mat34Seq& boundsPoses, const Mat34& pose, golem::Bounds::Seq& boundsSeq) const;
	bool intersect(const golem::Bounds::Seq& boundsSeq0, const golem::Bounds::Seq& boundsSeq1, bool groupTest) const;

	/** Runnable::run(): Collision detection on a path */
	//virtual void run();

	/** Creates Heuristic from the description. 
	* @param desc		Heuristic description
	*/
	void create(const Desc& desc);

	/** Heuristic constructor */
	Heuristic(golem::Controller &controller);

public:
	/** virtual destructor */
	virtual ~Heuristic();
	
#ifdef _HEURISTIC_PERFMON
	static U32 perfCollisionWaypoint, perfCollisionPath, perfCollisionGroup, perfCollisionBounds, perfCollisionSegs;

	static void resetLog();
	static void writeLog(Context &context, const char *str);
#endif
	
	/** Objective cost function of a single waypoint.
	 * @param w			waypoint
	 * @param root		root waypoint
	 * @param goal		goal waypoint
	 * @return			cost value
	 */
	virtual Real cost(const Waypoint &w, const Waypoint &root, const Waypoint &goal) const;

	/** Objective cost function of a path between specified waypoints.
	 * @param w0		waypoint
	 * @param w1		waypoint
	 * @return			cost value
	 */
	virtual Real cost(const Waypoint &w0, const Waypoint &w1) const;
	
	/** Collision detection test function for the single waypoint.
	 * @param w			waypoint
	 * @return			<code>true</code> if a collision has been detected; <code>false</code> otherwise
	 */
	virtual bool collides(const Waypoint &w) const;

	/** Collision detection test function for the single waypoint with thread data.
	* @param w			waypoint
	* @param data		a pointer to the current thread data
	* @return			<code>true</code> if a collision has been detected; <code>false</code> otherwise
	*/
	virtual bool collides(const Waypoint &w, ThreadData* data) const;

	/** Collision detection test function for the single waypoint.
	* @param w0			waypoint
	* @param w1			waypoint
	* @return			<code>true</code> if a collision has been detected; <code>false</code> otherwise
	*/
	virtual bool collides(const Waypoint &w0, const Waypoint &w1) const;

	/** Collision detection test function for the single waypoint with thread data.
	* @param w0			waypoint
	* @param w1			waypoint
	* @param data		a pointer to the current thread data
	* @return			<code>true</code> if a collision has been detected; <code>false</code> otherwise
	*/
	virtual bool collides(const Waypoint &w0, const Waypoint &w1, ThreadData* data) const;

	/** Collision detection test function for the single waypoint.
	* @param w0			waypoint
	* @param w1			waypoint
	* @return			<code>true</code> if a collision has been detected; <code>false</code> otherwise
	*/
	//bool collidesCPU(const Waypoint &w0, const Waypoint &w1) const;

	/** Collision detection test function on a specified path with Parallels.
	 * @param seq		waypoint path
	 * @param set		collision set
	 */
	//bool collidesCPU(const Waypoint::PtrSeq &seq, Node::IndexPairSet &set) const;

	/** Access to the thread local data.
	* @return			a pointer to the current thread data
	*/
	ThreadData* getThreadData() const;
	
	/** Clear the thread local data.
	*/
	void clearThreadData();

	/** Synchronise bounds of all chains */
	virtual void syncChainBounds(ThreadData& data) const;

	/** Synchronise bounds of all chains */
	virtual void syncChainBounds();
	
	/** Add bounds to a chain */
	template <typename _Ptr> void addChainBounds(_Ptr begin, _Ptr end, Chainspace::Index chain) {
		if (stateInfo.getChains().contains(chain))
			for (_Ptr i = begin; i != end; ++i)
				chainBoundsDesc[chain].insert(*i);
	}
	/** Get chain bounds */
	const ChainBoundsSet& getChainBounds() const {
		return chainBoundsDesc;
	}

	/** Synchronise bounds of all joints */
	virtual void syncJointBounds(ThreadData& data) const;

	/** Synchronise bounds of all joints */
	virtual void syncJointBounds();
	
	/** Add bounds to a joint */
	template <typename _Ptr> void addJointBounds(_Ptr begin, _Ptr end, Configspace::Index joint) {
		if (stateInfo.getJoints().contains(joint))
			for (_Ptr i = begin; i != end; ++i)
				jointBoundsDesc[joint].insert(*i);
	}
	/** Get joint bounds */
	const JointBoundsSet& getJointBounds() const {
		return jointBoundsDesc;
	}

	/** Clear the collection of collision bounds */
	virtual void clearCollisionBounds();
	/** Sets the collection of collision bounds
	 * @param collisionBounds	collection of collision bounds
	*/
	virtual void addCollisionBounds(const Bounds& collisionBounds);
	/** Sets the collection of collision bounds
	 * @param collisionBoundsSeq	collection of collision bounds
	*/
	virtual void addCollisionBounds(const Bounds::Seq &collisionBoundsSeq);
	/** Get collection of collision bound */
	const golem::Bounds::Seq& getCollisionBounds() const {
		return collisionBounds;
	}

	/** Sets heuristic description */
	void setDesc(const Desc& desc);
	/** Current heuristic description */
	inline const Desc& getDesc() const {
		return desc;
	}

	/** Chainspace coordinates properties. */
	inline const ChainDesc::ChainSeq& getChainDesc() const {
		return chainDesc;
	}
	/** Configspace coordinates properties. */
	inline const JointDesc::JointSeq& getJointDesc() const {
		return jointDesc;
	}

	/** Current scale */
	Real getScale() const {
		return scale;
	}
	/** Scale set */
	void setScale(Real scale);

	/** Configspace minimum override */
	void setMin(const GenCoordConfigspace& min);
	/** Current configspace minimum */
	const GenCoordConfigspace& getMin() const {
		return min;
	}
	/** Configspace maximum override */
	void setMax(const GenCoordConfigspace& max);
	/** Current configspace maximum */
	const GenCoordConfigspace& getMax() const {
		return max;
	}

	/** Sets cost description */
	void setCostDesc(const CostDesc& costDesc);
	/** Current heuristic description */
	inline const CostDesc& getCostDesc() const {
		return desc.costDesc;
	}

	/** Current collision detection */
	bool hasCollisionDetection() const {
		return desc.collisionDesc.enabled;
	}
	/** Collision detection set */
	void setCollisionDetection(bool collisionDetection) {
		this->desc.collisionDesc.enabled = collisionDetection;
	}

	/** Linear distance between two waypoints in workspace */
	inline Real getLinearDist(const Vec3& p0, const Vec3& p1) const {
		return p0.distance(p1);
	}
	/** Linear distance between two waypoints in workspace */
	inline Real getLinearDistSqr(const Vec3& p0, const Vec3& p1) const {
		return p0.distanceSqr(p1);
	}
	/** Angular distance between two waypoints in workspace */
	inline Real getAngularDist(const Quat& q0, const Quat& q1) const {
		const Real d = q0.dot(q1);

		// since body displacement represented by rotation around axis a by angle b corresponds to rotation around -a by angle -b
		// use max of a quaternion product as below (or equivalently abs)
		// abs(quat0 * quat1): range: <0, 1>, 1 - identity, 0 - the largest distance
		return REAL_ONE - Math::abs(d);

		// acos(max(quat0 * quat1)): range: <0, Pi/2>, 0 - identity, Pi/2 - the largest distance
		//return (REAL_ONE/REAL_PI_2)*Math::acos(Math::abs(d));
	}
	
	/** Distance between two waypoints in configuration space */
	Real getConfigspaceDist(const ConfigspaceCoord& c0, const ConfigspaceCoord& c1) const;
	/** Distance between two waypoints in configuration space */
	Real getConfigspaceDistSqr(const ConfigspaceCoord& c0, const ConfigspaceCoord& c1) const;

	/** Distance between two waypoints in workspace */
	Real getWorkspaceDist(const Waypoint& w0, const Waypoint& w1) const;
	/** Distance between two waypoints in workspace */
	Real getWorkspaceDistSqr(const Waypoint& w0, const Waypoint& w1) const;
	
	/** Combined weighted distance between two waypoints */
	Real getDist(const Waypoint& w0, const Waypoint& w1) const;
	/** Combined weighted distance between two waypoints */
	Real getBoundedDist(const Waypoint& w0, const Waypoint& w1) const;
	/** Distance to the configuration space limits */
	Real getConfigspaceLimitsDist(const ConfigspaceCoord& cc) const;

	/** Access to the controller */
	const Controller &getController() const {
		return controller;
	}
	/** Access to the controller */
	Controller &getController() {
		return controller;
	}

	/** Access to the context */
	const Context &getContext() const {
		return context;
	}
	/** Access to the context */
	Context &getContext() {
		return context;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PLANNER_HEURISTIC_H_*/
