/** @file Heuristic.cpp
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

#include <Golem/Planner/Heuristic.h>
#include <Golem/Sys/Message.h>
#include <Golem/Sys/Context.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

#ifdef _HEURISTIC_PERFMON
U32 Heuristic::perfCollisionWaypoint;
U32 Heuristic::perfCollisionPath;
U32 Heuristic::perfCollisionGroup;
U32 Heuristic::perfCollisionBounds;
U32 Heuristic::perfCollisionSegs;

void Heuristic::resetLog() {
	perfCollisionWaypoint = 0;
	perfCollisionPath = 0;
	perfCollisionGroup = 0;
	perfCollisionBounds = 0;
	perfCollisionSegs = 0;
}

void Heuristic::writeLog(Context &context, const char *str) {
	context.debug(
		"%s: collision_{waypoint, path, group, bounds, <segments>} = {%u, %u, %u, %u, %f}\n", str, perfCollisionWaypoint, perfCollisionPath, perfCollisionGroup, perfCollisionBounds, perfCollisionPath > 0 ? Real(perfCollisionSegs)/perfCollisionPath : REAL_ZERO
	);
}
#endif

//------------------------------------------------------------------------------

Heuristic::Heuristic(golem::Controller &controller) :
	controller(controller), context(controller.getContext())
{}

Heuristic::~Heuristic() {
}

void Heuristic::create(const Desc& desc) {
	//threadData.resize(context.getParallels() != NULL ? context.getParallels()->getNumOfThreads() : 1);
	
	stateInfo = controller.getStateInfo();

	setDesc(desc); // throws

	syncChainBounds();
	syncJointBounds();
}

Heuristic::ThreadData* Heuristic::getThreadData() const {
	const golem::U32 id = Thread::getCurrentThreadId();
	ThreadData::Set::iterator ptr = threadData.find(id);
	if (ptr == threadData.end()) {
		golem::CriticalSectionWrapper csw(cs);
		ptr = threadData.insert(threadData.end(), std::make_pair(id, ThreadData()));
		syncChainBounds(ptr->second);
		syncJointBounds(ptr->second);
	}
	return &ptr->second;
}

void Heuristic::clearThreadData() {
	golem::CriticalSectionWrapper csw(cs);
	threadData.clear();
}

void Heuristic::setDesc(const Desc& desc) {
	if (!desc.isValid())
		throw MsgHeuristicInvalidDesc(Message::LEVEL_CRIT, "Heuristic::setDesc(): Invalid description");

	this->desc = desc;

	jointDesc.fill((JointDesc*)NULL);
	if (stateInfo.getJoints().size() > (idx_t)this->desc.joints.size())
		throw MsgHeuristicInvalidDescJoints(Message::LEVEL_CRIT, "Heuristic::setDesc(): Number of joint descriptions too small %d", this->desc.joints.size());
	configspaceSeq.clear();
	for (idx_t n = 0; n < (idx_t)stateInfo.getJoints().size(); ++n) {
		const Configspace::Index i = stateInfo.getJoints().begin() + n;
		jointDesc[i] = &this->desc.joints[n];
		JointDesc* desc = jointDesc[i];

		dfltPos[i] = desc->dfltPos;
		min[i] = controller.getJoints()[i]->getMinOffset();
		max[i] = controller.getJoints()[i]->getMaxOffset();
		if (!desc->enabled)
			min[i].pos = max[i].pos = desc->dfltPos;
		
		distMax[i] = desc->distMax; // scale reference
		
		for (U32Seq::iterator j = desc->collisionJoints.begin(); j != desc->collisionJoints.end(); ++j) {
			*j += (U32)*stateInfo.getJoints().begin();
			if (*j == i)
				throw MsgHeuristicCollision(Message::LEVEL_CRIT, "Heuristic::setDesc(): Joint %d self-collision test not allowed", *i);
			if (*j >= stateInfo.getJoints().end())
				throw MsgHeuristicCollision(Message::LEVEL_CRIT, "Heuristic::setDesc(): Joint %d collision test with non-existing joint %d", *i, *j);
		}

		jointBoundsDesc[i].clear();
		for (golem::Bounds::Desc::Seq::const_iterator j = desc->bounds.begin(); j != desc->bounds.end(); ++j)
			jointBoundsDesc[i].insert((*j)->clone());

		if (desc->enabled)
			configspaceSeq.push_back(i);
	}
	if (configspaceSeq.empty())
		throw MsgHeuristicInvalidDescJoints(Message::LEVEL_CRIT, "Heuristic::setDesc(): No enabled joints");

	chainDesc.fill((ChainDesc*)NULL);
	if (stateInfo.getChains().size() > (idx_t)this->desc.chains.size())
		throw MsgHeuristicInvalidDescChains(Message::LEVEL_CRIT, "Heuristic::setDesc(): Number of chains descriptions too small %d", this->desc.chains.size());
	chainspaceSeq.clear();
	for (idx_t n = 0; n < (idx_t)stateInfo.getChains().size(); ++n) {
		const Chainspace::Index i = stateInfo.getChains().begin() + n;
		chainDesc[i] = &this->desc.chains[n];
		const ChainDesc* desc = chainDesc[i];
		
		distLinearMax[i] = desc->distLinearMax; // scale reference
		distAngularMax[i] = desc->distAngularMax; // scale reference

		bool enabledJoints = false;
		for (Configspace::Index j = stateInfo.getJoints(i).begin(); j < stateInfo.getJoints(i).end(); ++j)
			if (jointDesc[j]->enabled) {
				enabledJoints = true;
				break;
			}

		chainBoundsDesc[i].clear();
		for (golem::Bounds::Desc::Seq::const_iterator j = desc->bounds.begin(); j != desc->bounds.end(); ++j)
			chainBoundsDesc[i].insert((*j)->clone());

		if (desc->enabledLin || desc->enabledAng || enabledJoints)
			chainspaceSeq.push_back(i);
	}
	if (chainspaceSeq.empty())
		throw MsgHeuristicInvalidDescChains(Message::LEVEL_CRIT, "Heuristic::setDesc(): No enabled chains");
	

	// reset scale
	setScale(REAL_ONE);

	// add collision bounds
	clearCollisionBounds();
	addCollisionBounds(desc.collisionDesc.collisionBounds);
}

void Heuristic::setScale(Real scale) {
	if (scale <= REAL_EPS)
		throw MsgHeuristicInvalidScale(Message::LEVEL_ERROR, "Heuristic::setScale(): Scale factor must be larger than zero");

	this->scale = scale;
	this->scaleInv = REAL_ONE/scale;
	
	for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
		ChainDesc* desc = this->chainDesc[i];
		
		desc->distLinearMax = this->scale*distLinearMax[i];
		desc->distAngularMax = this->scale*distAngularMax[i];
	}
	
	for (Configspace::Index i = stateInfo.getJoints().begin(); i < stateInfo.getJoints().end(); ++i) {
		JointDesc* desc = this->jointDesc[i];

		desc->distMax = this->scale*distMax[i];
		desc->distMaxSqr = Math::sqr(desc->distMax);
	}
}

void Heuristic::setMin(const GenCoordConfigspace& min) {
	// combine limits
	for (Configspace::Index i = stateInfo.getJoints().begin(); i != stateInfo.getJoints().end(); ++i)
		if (getJointDesc()[i]->enabled) {
			const GenCoord minOffset = controller.getJoints()[i]->getMinOffset();
			const GenCoord maxOffset = controller.getJoints()[i]->getMaxOffset();
			this->min[i].pos = Math::clamp(min[i].pos, minOffset.pos, maxOffset.pos - REAL_EPS);
			this->min[i].vel = Math::clamp(min[i].vel, minOffset.vel, maxOffset.vel - REAL_EPS);
			this->min[i].acc = Math::clamp(min[i].acc, minOffset.acc, maxOffset.acc - REAL_EPS);
		}
}

void Heuristic::setMax(const GenCoordConfigspace& max) {
	// combine limits
	for (Configspace::Index i = stateInfo.getJoints().begin(); i != stateInfo.getJoints().end(); ++i)
		if (getJointDesc()[i]->enabled) {
			const GenCoord minOffset = controller.getJoints()[i]->getMinOffset();
			const GenCoord maxOffset = controller.getJoints()[i]->getMaxOffset();
			this->max[i].pos = Math::clamp(max[i].pos, minOffset.pos + REAL_EPS, maxOffset.pos);
			this->max[i].vel = Math::clamp(max[i].vel, minOffset.vel + REAL_EPS, maxOffset.pos);
			this->max[i].acc = Math::clamp(max[i].acc, minOffset.acc + REAL_EPS, maxOffset.pos);
		}
}

void Heuristic::setCostDesc(const CostDesc& costDesc) {
	if (!costDesc.isValid())
		throw MsgHeuristicInvalidDesc(Message::LEVEL_ERROR, "Heuristic::setCostDesc(): Invalid cost description");
	desc.costDesc = costDesc;
}

//------------------------------------------------------------------------------

void Heuristic::setPose(const Mat34Seq& boundsPoses, const Mat34& pose, golem::Bounds::Seq& boundsSeq) const {
	const U32 numOfBounds = (U32)std::min(boundsSeq.size(), boundsPoses.size());
	for (U32 i = 0; i < numOfBounds; ++i)
		boundsSeq[i]->multiplyPose(pose, boundsPoses[i]);
}

bool Heuristic::intersect(const golem::Bounds::Seq& boundsSeq0, const golem::Bounds::Seq& boundsSeq1, bool groupTest) const {
#ifdef _HEURISTIC_PERFMON
	++perfCollisionGroup;
#endif
	for (golem::Bounds::Seq::const_iterator i = boundsSeq0.begin(); i != boundsSeq0.end(); ++i) {
		const golem::Bounds& b0 = **i;
		for (golem::Bounds::Seq::const_iterator j = boundsSeq1.begin(); j != boundsSeq1.end(); ++j) {
			const golem::Bounds& b1 = **j;
			if (!groupTest || b0.getGroup() & b1.getGroup()) {
#ifdef _HEURISTIC_PERFMON
				++perfCollisionBounds;
#endif
				if (b0.intersect(b1))
					return true;
			}
		}
	}

	return false;
}

//------------------------------------------------------------------------------

bool Heuristic::collides(const Waypoint &w, ThreadData* data) const {
#ifdef _HEURISTIC_PERFMON
	++perfCollisionWaypoint;
#endif

	// all chains
	//for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i)
	for (Chainspace::Index i = stateInfo.getChains().end() - 1; i >= stateInfo.getChains().begin(); --i)
	{
		// all joints in a chain
		//for (Configspace::Index j = stateInfo.getJoints(i).begin(); j < stateInfo.getJoints(i).end(); ++j)
		for (Configspace::Index j = stateInfo.getJoints(i).end() - 1; j >= stateInfo.getJoints(i).begin(); --j)
		{
			const JointDesc* jdesc = getJointDesc()[j];
			Bounds::Seq& jointBounds = data->jointBounds[j];

			if (jointBounds.empty() || !jdesc->collisionBounds)
				continue;

			// reset to the current joint pose
			setPose(data->jointBoundsPoses[j], w.wposex[j], jointBounds);

			// check for collisions between the current joint and the environment bounds, test bounds groups
			if (!collisionBounds.empty() && intersect(jointBounds, collisionBounds, true))
				return true;

			// check for collisions between the current joint and the collision joints, do not test bounds groups
			const U32Seq& collisionJoints = jdesc->collisionJoints;
			for (U32Seq::const_iterator k = collisionJoints.begin(); k != collisionJoints.end(); ++k) {
				const Configspace::Index collisionIndex(*k);

				golem::Bounds::Seq &collisionJointBounds = data->jointBounds[collisionIndex];
				if (collisionJointBounds.empty())
					continue;

				setPose(data->jointBoundsPoses[collisionIndex], w.wposex[collisionIndex], collisionJointBounds);
				if (intersect(jointBounds, collisionJointBounds, false))
					return true;
			}

			// TODO joint bounds - chain bounds collisions
		}

		// TODO bounds - chain bounds collisions
	}

	return false;
}

bool Heuristic::collides(const Waypoint &w0, const Waypoint &w1, ThreadData* data) const {
	const Real dist = getDist(w0, w1);
	const U32 size = (U32)Math::round(dist/desc.collisionDesc.pathDistDelta) +1;
	Real p[2];
	Waypoint w;

#ifdef _HEURISTIC_PERFMON
	++perfCollisionPath;
	perfCollisionSegs += size - 1;
#endif

	// test for collisions in the range (w0, w1) - excluding w0 and w1
	for (U32 i = 1; i < size; ++i) {
		p[0] = Real(i)/Real(size);
		p[1] = REAL_ONE - p[0];

		// lineary interpolate coordinates
		for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
			w.cpos[j] = p[0]*w0.cpos[j] + p[1]*w1.cpos[j];

		// skip reference pose computation
		w.setup(controller, false, true);

		// test for collisions
		if (collides(w, data))
			return true;
	}

	return false;
}

//------------------------------------------------------------------------------

bool Heuristic::collides(const Waypoint &w) const {
	if (!desc.collisionDesc.enabled)
		return false;

	// find data pointer for the current thread
	ThreadData* data = getThreadData();

	return collides(w, data);
}

bool Heuristic::collides(const Waypoint &w0, const Waypoint &w1) const {
	if (!desc.collisionDesc.enabled)
		return false;

	// find data pointer for the current thread
	ThreadData* data = getThreadData();

	return collides(w0, w1, data);
}

//------------------------------------------------------------------------------

//void Heuristic::run() {
//	ThreadData* threadData = getThreadData(); // data pointer for the current thread
//	PathData* pathData = NULL; // current path data, null if there is no collisions
//	Real p[2];
//	Waypoint w;
//
//	for (;;) {
//		{
//			golem::CriticalSectionWrapper csw(cs);
//			if (pathData)
//				pathData->collides = true;
//
//			for (U32 pathIndexInit = this->pathIndex;;) {
//				pathData = &this->pathData[this->pathIndex];
//				this->pathIndex = (this->pathIndex + 1)%this->pathData.size();
//				if (!pathData->collides && pathData->index > 0)
//					break;
//				if (pathIndexInit == this->pathIndex)
//					return;
//			}
//
//			p[1] = Real(pathData->index)/Real(pathData->size);
//			p[0] = REAL_ONE - p[1];
//
//			--pathData->index;
//		}
//
//		// lineary interpolate coordinates
//		const Real* left = pathData->leftCC;
//		const Real* right = pathData->rightCC;
//		for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
//			w.cpos[j] = p[0]*left[*j] + p[1]*right[*j];
//
//		// skip reference pose computation
//		w.setup(controller, false, true);
//
//		// test for collisions
//		if (!collides(w, threadData))
//			pathData = NULL; // no collisions - reset pointer
//	}
//}
//
//bool Heuristic::collidesCPU(const Waypoint &w0, const Waypoint &w1) const {
//	if (!desc.collisionDesc.enabled)
//		return false;
//
//#ifdef _HEURISTIC_PERFMON
//	++perfCollisionPath;
//#endif
//
//	// prepare path data
//	const Real dist = getDist(w0, w1);
//	const U32 size = (U32)Math::round(dist/desc.collisionDesc.pathDistDelta) + 1;
//	const U32 threads = size - 1;
//	pathData.resize(1);
//	pathData.front().create(w0, w1, size);
//
//	// reset path index
//	this->pathIndex = 0;
//
//	// setup Parallels
//	Parallels *parallels = context.getParallels();
//	if (parallels != NULL) {
//		// launch Parallels
//		for (U32 t = std::min(parallels->getNumOfThreads(), threads); t > 0; --t) {
//			Job* job = parallels->startJob(const_cast<Heuristic*>(this));
//			if (!job)
//				throw Message(Message::LEVEL_CRIT, "Heuristic::collidesCPU(): Unable to start job");
//		}
//		(void)parallels->joinJobs(MSEC_TM_U32_INF);
//	}
//	else
//		const_cast<Heuristic*>(this)->run();
//
//	// collect results
//	return pathData.front().collides;
//}
//
//bool Heuristic::collidesCPU(const Waypoint::PtrSeq &seq, Node::IndexPairSet &set) const {
//	if (seq.size() < 2)
//		throw Message(Message::LEVEL_CRIT, "Heuristic::collidesCPU(): invalid path size %u", seq.size());
//
//	if (!desc.collisionDesc.enabled)
//		return false;
//
//#ifdef _HEURISTIC_PERFMON
//	perfCollisionPath += (U32)seq.size() - 1;
//#endif
//
//	// prepare path data
//	U32 threads = 0;
//	pathData.clear();
//	pathData.reserve(seq.size() - 1);
//	for (Waypoint::PtrSeq::const_iterator i = seq.begin(), j = ++seq.begin(); j != seq.end(); ++i, ++j) {
//		const Real dist = getDist(**i, **j);
//		const U32 size = (U32)Math::round(dist/desc.collisionDesc.pathDistDelta) + 1;
//		pathData.push_back(PathData(**i, **j, size));
//		threads += size - 1;
//	}
//
//	// reset path index
//	this->pathIndex = 0;
//
//	// setup Parallels
//	Parallels *parallels = context.getParallels();
//	if (parallels != NULL) {
//		// launch Parallels
//		for (U32 t = std::min(parallels->getNumOfThreads(), threads); t > 0; --t) {
//			Job* job = parallels->startJob(const_cast<Heuristic*>(this));
//			if (!job)
//				throw Message(Message::LEVEL_CRIT, "Heuristic::collidesCPU(): Unable to start job");
//		}
//		(void)parallels->joinJobs(MSEC_TM_U32_INF);
//	}
//	else
//		const_cast<Heuristic*>(this)->run();
//
//	// collect results
//	bool collides = false;
//	for (PathData::Seq::const_iterator i = pathData.begin(); i != pathData.end(); ++i)
//		if (i->collides) {
//			set.insert(Node::makeIndexPair(i->leftIndex, i->rightIndex));
//			collides = true;
//		}
//
//	return collides;
//}

//------------------------------------------------------------------------------

void Heuristic::syncChainBounds(ThreadData& data) const {
	syncBounds(stateInfo.getChains().begin(), stateInfo.getChains().end(), controller.getChains(), data.chainBounds, data.chainBoundsPoses);
	for (Chainspace::Index j = stateInfo.getChains().begin(); j < stateInfo.getChains().end(); ++j)
		addBounds(chainBoundsDesc[j].begin(), chainBoundsDesc[j].end(), data.chainBounds[j], data.chainBoundsPoses[j]);
}

void Heuristic::syncChainBounds() {
	golem::CriticalSectionWrapper csw(cs);
	for (ThreadData::Set::iterator i = threadData.begin(); i != threadData.end(); ++i)
		syncChainBounds(i->second);
}

void Heuristic::syncJointBounds(ThreadData& data) const {
	syncBounds(stateInfo.getJoints().begin(), stateInfo.getJoints().end(), controller.getJoints(), data.jointBounds, data.jointBoundsPoses);
	for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
		addBounds(jointBoundsDesc[j].begin(), jointBoundsDesc[j].end(), data.jointBounds[j], data.jointBoundsPoses[j]);
}

void Heuristic::syncJointBounds() {
	golem::CriticalSectionWrapper csw(cs);
	for (ThreadData::Set::iterator i = threadData.begin(); i != threadData.end(); ++i)
		syncJointBounds(i->second);
}

//------------------------------------------------------------------------------

void Heuristic::clearCollisionBounds() {
	this->collisionBounds.clear();

	//context.debug("Heuristic::clearCollisionBounds()\n");
}

void Heuristic::addCollisionBounds(const Bounds& collisionBounds) {
	class Expander {
	public:
		const Real skinThickness;

		Expander(Real skinThickness) : skinThickness(skinThickness) {
		}

		Bounds::Ptr expand(const Bounds &bounds) const {
			Bounds::Ptr pBounds;

			switch (bounds.getType()) {
			case Bounds::TYPE_PLANE:
				pBounds = expand(dynamic_cast<const BoundingPlane&>(bounds));
				break;
			case Bounds::TYPE_SPHERE:
				pBounds = expand(dynamic_cast<const BoundingSphere&>(bounds));
				break;
			case Bounds::TYPE_CYLINDER:
				pBounds = expand(dynamic_cast<const BoundingCylinder&>(bounds));
				break;
			case Bounds::TYPE_BOX:
				pBounds = expand(dynamic_cast<const BoundingBox&>(bounds));
				break;
			case Bounds::TYPE_CONVEX_MESH:
				pBounds = expand(dynamic_cast<const BoundingConvexMesh&>(bounds));
				break;
			default:
				break;
			}

			return pBounds;
		}

		Bounds::Ptr expand(const BoundingPlane &bounds) const {
			BoundingPlane::Desc desc;
			desc.normal = bounds.getNormal();
			desc.distance = bounds.getDistance();// + skinThickness;
			return desc.create();
		}

		Bounds::Ptr expand(const BoundingSphere &bounds) const {
			BoundingSphere::Desc desc;
			desc.pose = bounds.getPose();
			desc.radius = bounds.getRadius() + skinThickness;
			return desc.create();
		}

		Bounds::Ptr expand(const BoundingCylinder &bounds) const {
			BoundingCylinder::Desc desc;
			desc.pose = bounds.getPose();
			desc.length = bounds.getLength() + REAL_TWO*skinThickness;
			desc.radius = bounds.getRadius() + skinThickness;
			return desc.create();
		}

		Bounds::Ptr expand(const BoundingBox &bounds) const {
			BoundingBox::Desc desc;
			desc.pose = bounds.getPose();
			desc.dimensions.set(
				bounds.getDimensions().v1 + skinThickness,
				bounds.getDimensions().v2 + skinThickness,
				bounds.getDimensions().v3 + skinThickness
			);
			return desc.create();
		}

		Bounds::Ptr expand(const BoundingConvexMesh &bounds) const {
			BoundingConvexMesh::Desc desc;

			const U32 numOfVertices = (U32)bounds.getVertices().size();
			desc.vertices.resize(numOfVertices);

			Vec3 centroid(REAL_ZERO);
			for (U32 i = 0; i < numOfVertices; ++i)
				centroid.add(centroid, bounds.getVertices()[i]);
			centroid.multiply(REAL_ONE/numOfVertices, centroid);
			
			// expand along lines drawn from centroid to vertices
			for (U32 i = 0; i < numOfVertices; ++i) {
				Vec3 axis;
				axis.subtract(bounds.getVertices()[i], centroid);
				axis.normalise();
				desc.vertices[i].multiplyAdd(skinThickness, axis, bounds.getVertices()[i]);
			}
			
			const U32 numOfTriangles = (U32)bounds.getTriangles().size();

			if (numOfTriangles > 0) {
				desc.bCook = false;
				desc.triangles = bounds.getTriangles();
			}
			else
				desc.bCook = true;
			
			return desc.create();
		}
	};

	// Expand bounds
	Bounds::Ptr pBounds = Expander(desc.collisionDesc.skinThickness).expand(collisionBounds);
	if (pBounds == NULL)
		throw MsgHeuristicBoundsExpand(Message::LEVEL_ERROR, "Heuristic::addCollisionBounds(): unable to create %s", collisionBounds.getName());
	
	//context.debug("Heuristic::setCollisionBounds(): group #%08x, name: %s\n", pBounds->getGroup(), pBounds->getName());

	// add to the collection
	this->collisionBounds.push_back(pBounds);
}

void Heuristic::addCollisionBounds(const Bounds::Seq &collisionBoundsSeq) {
	for (Bounds::Seq::const_iterator i = collisionBoundsSeq.begin(); i != collisionBoundsSeq.end(); ++i)
		addCollisionBounds(**i);
}

//------------------------------------------------------------------------------

Real Heuristic::getConfigspaceDist(const ConfigspaceCoord& c0, const ConfigspaceCoord& c1) const {
	Real dist = REAL_ZERO;
	
	for (ConfigspaceSeq::const_iterator i = configspaceSeq.begin(); i != configspaceSeq.end(); ++i) {
		const Configspace::Index j = *i;
		dist += Math::sqr(c0[j] - c1[j]);
	}

	return Math::sqrt(dist);
}

Real Heuristic::getConfigspaceDistSqr(const ConfigspaceCoord& c0, const ConfigspaceCoord& c1) const {
	Real dist = REAL_ZERO;

	for (ConfigspaceSeq::const_iterator i = configspaceSeq.begin(); i != configspaceSeq.end(); ++i) {
		const Configspace::Index j = *i;
		dist += Math::sqr(c0[j] - c1[j]);
	}

	return dist;
}

Real Heuristic::getWorkspaceDist(const Waypoint& w0, const Waypoint& w1) const {
	Real dist = REAL_ZERO;

	const Chainspace::Range chains = stateInfo.getChains();
	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i) {
		const ChainDesc* cdesc = getChainDesc()[i];

		if (cdesc->enabledLin) {
			const Real p = Heuristic::getLinearDist(w0.wpos[i].p, w1.wpos[i].p);
			dist += cdesc->distNorm*scaleInv*p;
		}
		if (cdesc->enabledAng) {
			const Real q = Heuristic::getAngularDist(w0.qrot[i], w1.qrot[i]);
			dist += (REAL_ONE - cdesc->distNorm)*q;
		}
	}

	//for (ChainspaceSeq::const_iterator i = chainspaceSeq.begin(); i != chainspaceSeq.end(); ++i) {
	//	const Chainspace::Index j = *i;
	//	const ChainDesc* desc = getChainDesc()[j];

	//	if (desc->enabledLin) {
	//		// linear distance
	//		const Real p = Heuristic::getLinearDist(w0.wpos[j].p, w1.wpos[j].p);
	//		dist += desc->distNorm*p;
	//	}
	//	if (desc->enabledAng) {
	//		// angular distance
	//		const Real q = Heuristic::getAngularDist(w0.qrot[j], w1.qrot[j]);
	//		dist += (REAL_ONE - desc->distNorm)*q;
	//	}
	//}

	return dist;
}

Real Heuristic::getWorkspaceDistSqr(const Waypoint& w0, const Waypoint& w1) const {
	Real dist = REAL_ZERO;

	const Chainspace::Range chains = stateInfo.getChains();
	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i) {
		const ChainDesc* cdesc = getChainDesc()[i];

		if (cdesc->enabledLin) {
			const Real p = Heuristic::getLinearDistSqr(w0.wpos[i].p, w1.wpos[i].p);
			dist += cdesc->distNorm*scaleInv*p;
		}
		if (cdesc->enabledAng) {
			const Real q = Heuristic::getAngularDist(w0.qrot[i], w1.qrot[i]);
			dist += (REAL_ONE - cdesc->distNorm)*q;
		}
	}

	//for (ChainspaceSeq::const_iterator i = chainspaceSeq.begin(); i != chainspaceSeq.end(); ++i) {
	//	const Chainspace::Index j = *i;
	//	const ChainDesc* desc = getChainDesc()[j];

	//	if (desc->enabledLin) {
	//		// linear distance
	//		const Real p = Heuristic::getLinearDistSqr(w0.wpos[j].p, w1.wpos[j].p);
	//		dist += desc->distNorm*p;
	//	}
	//	if (desc->enabledAng) {
	//		// angular distance
	//		const Real q = Heuristic::getAngularDist(w0.qrot[j], w1.qrot[j]);
	//		dist += (REAL_ONE - desc->distNorm)*q;
	//	}
	//}

	return dist;
}

Real Heuristic::getDist(const Waypoint& w0, const Waypoint& w1) const {
	Real dist = REAL_ZERO;

	const Chainspace::Range chains = stateInfo.getChains();
	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i) {
		const Configspace::Range joints = stateInfo.getJoints(i);

		Real jdist = REAL_ZERO;
		for (Configspace::Index j = joints.begin(); j < joints.end(); ++j) {
			const JointDesc* jdesc = getJointDesc()[j];
			if (jdesc->enabled) {
				const Real d = Math::sqr(w0.cpos[j] - w1.cpos[j]);
				jdist += d;
			}
		}
		jdist = Math::sqrt(jdist)/joints.size();

		Real cdist = REAL_ZERO;
		const ChainDesc* cdesc = getChainDesc()[i];
		if (cdesc->enabledLin) {
			const Real p = Heuristic::getLinearDist(w0.wpos[i].p, w1.wpos[i].p);
			cdist += cdesc->distNorm*scaleInv*p;
		}
		if (cdesc->enabledAng) {
			const Real q = Heuristic::getAngularDist(w0.qrot[i], w1.qrot[i]);
			cdist += (REAL_ONE - cdesc->distNorm)*q;
		}

		dist += cdesc->distConfigspaceWorkspaceNorm*jdist + (REAL_ONE - cdesc->distConfigspaceWorkspaceNorm)*cdist;
	}

	//for (size_t i = 0; i < chainspaceSeq.size(); ++i) {
	//	const Chainspace::Index j = chainspaceSeq[i];
	//	
	//	Real jdist = REAL_ZERO;
	//	const Configspace::Range joints = stateInfo.getJoints(j);
	//	for (Configspace::Index k = joints.begin(); k < joints.end(); ++k) {
	//		const JointDesc* jdesc = getJointDesc()[k];
	//		if (jdesc->enabled) {
	//			const Real d = Math::sqr(w0.cpos[k] - w1.cpos[k]);
	//			jdist += d;
	//		}
	//	}
	//	jdist = Math::sqrt(jdist)/joints.size();

	//	const ChainDesc* cdesc = getChainDesc()[j];
	//	Real cdist = REAL_ZERO;
	//	if (cdesc->enabledLin) {
	//		// linear distance
	//		const Real p = Heuristic::getLinearDist(w0.wpos[j].p, w1.wpos[j].p);
	//		cdist += cdesc->distNorm*p;
	//	}
	//	if (cdesc->enabledAng) {
	//		// angular distance
	//		const Real q = Heuristic::getAngularDist(w0.qrot[j], w1.qrot[j]);
	//		cdist += (REAL_ONE - cdesc->distNorm)*q;
	//	}

	//	dist += cdesc->distConfigspaceWorkspaceNorm*jdist + (REAL_ONE - cdesc->distConfigspaceWorkspaceNorm)*cdist;
	//}

	return dist;
}

Real Heuristic::getBoundedDist(const Waypoint& w0, const Waypoint& w1) const {
	Real dist = REAL_ZERO;

	const Chainspace::Range chains = stateInfo.getChains();
	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i) {
		const Configspace::Range joints = stateInfo.getJoints(i);

		Real jdist = REAL_ZERO;
		for (Configspace::Index j = joints.begin(); j < joints.end(); ++j) {
			const JointDesc* jdesc = getJointDesc()[j];
			if (jdesc->enabled) {
				const Real d = Math::sqr(w0.cpos[j] - w1.cpos[j]);
				if (d > jdesc->distMaxSqr)
					return Node::COST_INF;
				jdist += d;
			}
		}
		jdist = Math::sqrt(jdist)/joints.size();

		Real cdist = REAL_ZERO;
		const ChainDesc* cdesc = getChainDesc()[i];
		if (cdesc->enabledLin) {
			const Real p = Heuristic::getLinearDist(w0.wpos[i].p, w1.wpos[i].p);
			if (p > cdesc->distLinearMax)
				return Node::COST_INF;
			cdist += cdesc->distNorm*scaleInv*p;
		}
		if (cdesc->enabledAng) {
			const Real q = Heuristic::getAngularDist(w0.qrot[i], w1.qrot[i]);
			if (q > cdesc->distAngularMax)
				return Node::COST_INF;
			cdist += (REAL_ONE - cdesc->distNorm)*q;
		}

		dist += cdesc->distConfigspaceWorkspaceNorm*jdist + (REAL_ONE - cdesc->distConfigspaceWorkspaceNorm)*cdist;
	}
	
	//for (size_t i = 0; i < chainspaceSeq.size(); ++i) {
	//	const Chainspace::Index j = chainspaceSeq[i];
	//	const Configspace::Range joints = stateInfo.getJoints(j);

	//	Real jdist = REAL_ZERO;
	//	for (Configspace::Index k = joints.begin(); k < joints.end(); ++k) {
	//		const JointDesc* jdesc = getJointDesc()[k];
	//		if (jdesc->enabled) {
	//			const Real d = Math::sqr(w0.cpos[k] - w1.cpos[k]);
	//			if (d > jdesc->distMaxSqr)
	//				return Node::COST_INF;
	//			jdist += d;
	//		}
	//	}
	//	jdist = Math::sqrt(jdist)/joints.size();

	//	const ChainDesc* cdesc = getChainDesc()[j];
	//	Real cdist = REAL_ZERO;
	//	if (cdesc->enabledLin) {
	//		// linear distance
	//		const Real p = Heuristic::getLinearDist(w0.wpos[j].p, w1.wpos[j].p);
	//		if (p > cdesc->distLinearMax)
	//			return Node::COST_INF;
	//		cdist += cdesc->distNorm*p;
	//	}
	//	if (cdesc->enabledAng) {
	//		// angular distance
	//		const Real q = Heuristic::getAngularDist(w0.qrot[j], w1.qrot[j]);
	//		if (q > cdesc->distAngularMax)
	//			return Node::COST_INF;
	//		cdist += (REAL_ONE - cdesc->distNorm)*q;
	//	}

	//	dist += cdesc->distConfigspaceWorkspaceNorm*jdist + (REAL_ONE - cdesc->distConfigspaceWorkspaceNorm)*cdist;
	//}

	return dist;
}

Real Heuristic::getConfigspaceLimitsDist(const ConfigspaceCoord& cc) const {
	//Real dist = REAL_ZERO;
	//size_t count = 0;

	//for (Configspace::Index i = stateInfo.getJoints().begin(); i < stateInfo.getJoints().end(); ++i) {
	//	const JointDesc* desc = getJointDesc()[i];

	//	if (desc->enabled) {
	//		const Real dmin = min[i].pos - cc[i];
	//		if (dmin > REAL_ZERO)
	//			dist += Math::sqr(dmin);
	//		const Real dmax = cc[i] - max[i].pos;
	//		if (dmax > REAL_ZERO)
	//			dist += Math::sqr(dmax);
	//		++count;
	//	}
	//}

	//return Math::sqrt(dist)/count;

	Real dist = REAL_ZERO;

	for (ConfigspaceSeq::const_iterator i = configspaceSeq.begin(); i != configspaceSeq.end(); ++i) {
		const Configspace::Index j = *i;

		const Real dmin = min[j].pos - cc[j];
		if (dmin > REAL_ZERO)
			dist += Math::sqr(dmin);
		const Real dmax = cc[j] - max[j].pos;
		if (dmax > REAL_ZERO)
			dist += Math::sqr(dmax);
	}

	return Math::sqrt(dist)/configspaceSeq.size();
}

//------------------------------------------------------------------------------

Real Heuristic::cost(const Waypoint &w, const Waypoint &root, const Waypoint &goal) const {
	Real c = getWorkspaceDistSqr(w, goal);

	if (desc.costDesc.distRootFac > REAL_ZERO)
		c += desc.costDesc.distRootFac*getConfigspaceDist(w.cpos, root.cpos);

	if (desc.costDesc.distLimitsFac > REAL_ZERO)
		c += desc.costDesc.distLimitsFac*getConfigspaceLimitsDist(w.cpos);
	if (desc.costDesc.distDfltFac > REAL_ZERO)
		c += desc.costDesc.distDfltFac*getConfigspaceDist(w.cpos, dfltPos);

	return c;
}

Real Heuristic::cost(const Waypoint &w0, const Waypoint &w1) const {
	Real c = getBoundedDist(w0, w1);
	if (c >= Node::COST_INF)
		return Node::COST_INF;

	if (desc.costDesc.distLimitsFac > REAL_ZERO)
		c += desc.costDesc.distLimitsFac*getConfigspaceLimitsDist(w1.cpos);
	if (desc.costDesc.distDfltFac > REAL_ZERO)
		c += desc.costDesc.distDfltFac*getConfigspaceDist(w1.cpos, dfltPos);

	return c;
}

//------------------------------------------------------------------------------

