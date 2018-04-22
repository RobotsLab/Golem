/** @file Heuristic.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/HBPlanner/Heuristic.h>
#include <Golem/Ctrl/Controller.h>
#include <Golem/Planner/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

#ifdef _HBHEURISTIC_PERFMON
U32 HBHeuristic::perfCollisionWaypoint;
U32 HBHeuristic::perfCollisionPath;
U32 HBHeuristic::perfCollisionGroup;
U32 HBHeuristic::perfCollisionBounds;
U32 HBHeuristic::perfCollisionSegs;
U32 HBHeuristic::perfBoundDist;
U32 HBHeuristic::perfH;

void HBHeuristic::resetLog() {
	perfCollisionWaypoint = 0;
	perfCollisionPath = 0;
	perfCollisionGroup = 0;
	perfCollisionBounds = 0;
	perfCollisionSegs = 0;
	perfBoundDist = 0;
	perfH = 0;
}

void HBHeuristic::writeLog(Context &context, const char *str) {
	context.write(
		"%s: collision_{waypoint, path, group, bounds, <segments>} = {%u, %u, %u, %u, %u, %f}, calls_{getBoundedDist(), h()} = {%u, %u}\n", str, perfCollisionWaypoint, perfCollisionPath, perfCollisionGroup, perfCollisionBounds, perfCollisionPath > 0 ? Real(perfCollisionSegs) / perfCollisionPath : REAL_ZERO, perfBoundDist, perfH
		);
}
#endif

//------------------------------------------------------------------------------

inline Vec3 dot(const Vec3 &v, const Vec3 &diagonal) {
	Vec3 result;
	for (size_t i = 0; i < 3; ++i)
		result[i] = v[i] * diagonal[i];
	return result;
}

void dotInverse(const std::vector<Real>& v0, const std::vector<Real>& v1, std::vector<Real> &result) {
	if (v0.size() != v1.size())
		throw Message(Message::LEVEL_CRIT, "dot(std::vector<Real>, std::vector<Real>): Invalid vectors size.");
	result.reserve(v0.size());
	for (size_t i = 0; i < v0.size(); ++i) {
		const Real r = v0[i] * (REAL_ONE/v1[i]);
		result.push_back(r);
	}
}

Real dot(const std::vector<Real>& v0, const std::vector<Real>& v1) {
	if (v0.size() != v1.size())
		throw Message(Message::LEVEL_CRIT, "dot(std::vector<Real>, std::vector<Real>): Invalid vectors size.");
	
	Real result;
	result = REAL_ZERO;
	for (size_t i = 0; i < v0.size(); ++i)
		result += v0[i] * v1[i];

	return result;
}

//------------------------------------------------------------------------------

HBHeuristic::HBHeuristic(Controller &controller) : Heuristic(controller), rand(context.getRandSeed()) {}

bool HBHeuristic::create(const Desc &desc) {
	Heuristic::create((Heuristic::Desc&)desc);

	setDesc(desc); // throws

	syncChainBounds();
	syncJointBounds();

	jointFac = desc.ftModelDesc.jointFac;

	manipulator.reset();

	enableUnc = false;

	pointCloudCollision = false;

	collision.reset();
	waypoint.setToDefault();
	waypoint.points = 100000;//ftDrivenDesc.ftModelDesc.points;

	hypothesisBoundsSeq.clear();

	testCollision = false;

	return true;
}

void HBHeuristic::setDesc(const Desc &desc) {
	Heuristic::setDesc((Heuristic::Desc&)desc);
	this->ftDrivenDesc = desc;
	this->ftDrivenDesc.ftModelDesc = desc.ftModelDesc;
}

void HBHeuristic::setDesc(const Heuristic::Desc &desc) {
	Heuristic::setDesc(desc);
}

//------------------------------------------------------------------------------

Real HBHeuristic::cost(const Waypoint &w, const Waypoint &root, const Waypoint &goal) const {
	Real c = REAL_ZERO;
	const bool enable = enableUnc && pBelief->getHypotheses().size() > 0;

	if (desc.costDesc.distRootFac > REAL_ZERO)
		c += desc.costDesc.distRootFac*getDist(w, root);
	
	const Real dist = (enable)?getMahalanobisDist(w, goal):getWorkspaceDist(w, goal);
	c += dist;

	if (desc.costDesc.distLimitsFac > REAL_ZERO)
		c += desc.costDesc.distLimitsFac*getConfigspaceLimitsDist(w.cpos);
	if (desc.costDesc.distDfltFac > REAL_ZERO)
		c += desc.costDesc.distDfltFac*getConfigspaceDist(w.cpos, dfltPos);

	return c;
}

Real HBHeuristic::cost(const Waypoint &w0, const Waypoint &w1) const {
	Real c = REAL_ZERO, d;
	const bool enable = enableUnc && (pBelief.get() && pBelief->getHypotheses().size() > 0);

	const Chainspace::Index chainArm = stateInfo.getChains().begin();
	d = Heuristic::getBoundedDist(w0, w1);
	if (d >= Node::COST_INF)
		return Node::COST_INF;

	const Real r = (enable && getBoundedDist(w1) < Node::COST_INF) ? expectedObservationCost(w0, w1) : 1;//(d < 5*bsDesc.ftModelDesc.tactileRange)?getObservationalCost(w0, w1):1;
	c += d*r;
	
	if (desc.costDesc.distLimitsFac > REAL_ZERO)
		c += desc.costDesc.distLimitsFac*getConfigspaceLimitsDist(w1.cpos);
	if (desc.costDesc.distDfltFac > REAL_ZERO)
		c += desc.costDesc.distDfltFac*getConfigspaceDist(w1.cpos, dfltPos);

//	context.debug("HBHeuristic::cost(enable=%s): preforming time %.7f\n", enableUnc ? "ON" : "OFF", context.getTimer().elapsed() - init);
	return c;
}

/**
computes the bounded distance between the chains' poses (only for chains with enabled obs)
and the reference frame of the samples. If at least one of the distance is <= of the max distance
it returns the distance.. otherwise it returns INF.
*/
Real HBHeuristic::getBoundedDist(const Waypoint& w) const {
#ifdef _HBHEURISTIC_PERFMON
	++perfBoundDist;
#endif
	const Real ret = Node::COST_INF;
	if (pBelief.get() && pBelief->getHypotheses().empty())
		return ret;

	Hypothesis::Seq::const_iterator maxLhdPose = pBelief->getHypotheses().begin();
	for (Chainspace::Index i = stateInfo.getChains().end() - 1; i >= stateInfo.getChains().begin(); --i){
		const RBCoord c(w.wpos[i]);
		const Real dist = c.p.distance((*maxLhdPose)->toRBPoseSampleGF().p);
		if (dist < ftDrivenDesc.ftModelDesc.distMax)
				return dist;
	}

	//for (Belief::Hypothesis::Seq::const_iterator s = ++pBelief->getHypotheses().begin(); s != pBelief->getHypotheses().end(); ++s) {
	//	for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
	//		const ChainDesc* cdesc = getChainDesc()[i];
	//		if (cdesc->enabledObs) {
	//			const RBCoord c(w.wpos[i]);
	//			const Real dist = c.p.distance((*s)->toRBPoseSampleGF().p);
	//			if (dist < this->ftDrivenDesc.ftModelDesc.distMax)
	//				return dist;
	//		}
	//	}
	//}
	return ret;
}

Real HBHeuristic::getMahalanobisDist(const Waypoint& w0, const Waypoint& goal) const {
//	context.debug("FTHeuristic::getMahalanobisDist()\n");
	Real dist = REAL_ZERO;
		
	for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
		const ChainDesc* desc = getChainDesc()[i];

		const RBCoord a(w0.wpos[i]), b(goal.wpos[i]);
		// linear distance
		if (desc->enabledLin) {
			const Vec3 p = w0.wpos[i].p - goal.wpos[i].p;
			const Real d = dot(p, pBelief->getSampleProperties().covarianceInv.p).dot(p);
			dist += Math::sqrt(d);
		}
		// angular distance
		if (desc->enabledAng) {
			const Real q = Heuristic::getAngularDist(w0.qrot[i], goal.qrot[i]);
			dist += (REAL_ONE - desc->distNorm)*q;
		}
	}
	return ftDrivenDesc.ftModelDesc.mahalanobisDistFac*dist;
}

/*
 J(wi, wj)=SUM_k e^-PSI(wi, wj, k)
 where PSI(wi, wj, k)=||h(wj,p^k)-h(wj,p^1)||^2_Q
*/
Real HBHeuristic::expectedObservationCost(const Waypoint &wi, const Waypoint &wj) const {
	if (pBelief->getHypotheses().size() == 0)
		return REAL_ONE;

	Real cost = REAL_ZERO;

	std::stringstream str;
	std::vector<Real> hij;
	hij.clear();
	h(wi, wj, /*k,*/ hij);
	Real value(REAL_ZERO);
	for (std::vector<Real>::const_iterator i = hij.begin(); i != hij.end(); ++i)
		value += Math::sqr(*i);
	cost += Math::exp(-Math::sqrt(value)); // REAL_HALF*

	return cost/(pBelief->getHypotheses().size() - 1);
}

void HBHeuristic::h(const Waypoint &wi, const Waypoint &wj, std::vector<Real> &y) const {	
#ifdef _HBHEURISTIC_PERFMON
	++perfH;
#endif
	auto kernel = [&] (Real x, Real lambda) -> Real {
		return /*lambda**/Math::exp(-lambda*x);
	};
	Hypothesis::Seq::const_iterator maxLhdPose = pBelief->getHypotheses().begin();
	const Real norm = (REAL_ONE - /*pBelief->*/kernel(ftDrivenDesc.ftModelDesc.distMax, /*pBelief->*/ftDrivenDesc.ftModelDesc.lambda));//(ftDrivenDesc.ftModelDesc.enabledLikelihood)?/*REAL_ONE/*/(REAL_ONE - kernel(ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.lambda)):REAL_ONE;
	y.clear();
	const U32 meanIdx = 0;
	U32 steps = 1;

	if (false) {
		const Real dist = getDist(wi, wj);	
		steps = (U32)Math::round(dist/(desc.collisionDesc.pathDistDelta));
	}
	y.reserve((pBelief->getHypotheses().size() - 1)*steps/**(armInfo.getChains().size() + handInfo.getJoints().size())*/);

	Waypoint w;
	U32 i = (steps == 1) ? 0 : 1;
	for (; i < steps; ++i) {
		if (steps != 1) {
			for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
				w.cpos[j] = wj.cpos[j] - (wj.cpos[j] - wi.cpos[j])*Real(i)/Real(steps);
		
			// skip reference pose computation
			w.setup(controller, false, true);
		}
		else w = wj;

		const Real likelihood_p1 = estimate(pBelief->getHypotheses().cbegin(), w); //Math::exp(estimate_p1 - estimates[0]);//pBelief->getHypotheses().front()->evaluate(ftDrivenDesc.evaluationDesc, manipulator->getPose(w.cpos), testCollision);
		for (auto p = ++pBelief->getHypotheses().cbegin(); p != pBelief->getHypotheses().cend(); ++p) 
			y.push_back(estimate(p, w) - likelihood_p1);

	}
}

Real HBHeuristic::evaluate(const Hypothesis::Seq::const_iterator &hypothesis, const Waypoint &w) const {
	Real eval = REAL_ZERO;
	Controller::State state = manipulator->getController().createState();
	manipulator->getController().lookupState(SEC_TM_REAL_MAX, state);
	// position control
	state.cpos = w.cpos;
	Manipulator::Config config(w.cpos, manipulator->getBaseFrame(w.cpos));
	if (intersect(manipulator->getBounds(config.config, config.frame.toMat34()), (*hypothesis)->bounds(), false))
		eval = (*hypothesis)->evaluate(ftDrivenDesc.evaluationDesc, config.config);
	else {
		const Real dist = config.frame.toMat34().p.distance((*hypothesis)->toRBPoseSampleGF().p);
		if (dist < ftDrivenDesc.ftModelDesc.distMax)
			eval = dist;
	}
	return eval;
}

//------------------------------------------------------------------------------

Real HBHeuristic::directionApproach(const Waypoint &w) const {
	const Chainspace::Index armIndex = armInfo.getChains().begin();
	
	Vec3 v;
	Mat34 tcpFrameInv, hypothesis;
	hypothesis = (*pBelief->getHypotheses().begin())->toRBPoseSampleGF().toMat34();
	tcpFrameInv.setInverse(w.wpos[armIndex]);
	tcpFrameInv.multiply(v, hypothesis.p);
	v.normalise();
	return v.z > 0 ? REAL_ONE - ftDrivenDesc.directionFac : REAL_ONE;
}

//------------------------------------------------------------------------------

bool HBHeuristic::collides(const Waypoint &w, ThreadData* data) const {
#ifdef _HBHEURISTIC_PERFMON
	++perfCollisionWaypoint;
#endif
	// check for collisions with the object to grasp. only the hand
	if (pointCloudCollision && !pBelief->getHypotheses().empty()) {
		Hypothesis::Seq::const_iterator maxLhdPose = pBelief->getHypotheses().begin();
		Manipulator::Config config(w.cpos, manipulator->getBaseFrame(w.cpos));
		if (intersect(manipulator->getBounds(config.config, config.frame.toMat34()), (*maxLhdPose)->bounds(), false)) {
#ifdef _HBHEURISTIC_PERFMON
//			++perfCollisionPointCloud;
#endif
			if ((*maxLhdPose)->checkNN(ftDrivenDesc.checkDesc, config))
				return true;
		}
	}

	return Heuristic::collides(w, data);
}

bool HBHeuristic::collides(const Waypoint &w0, const Waypoint &w1, ThreadData* data) const {
	const Real dist = getDist(w0, w1);
	const U32 size = (U32)Math::round(dist / desc.collisionDesc.pathDistDelta) + 1;
	Real p[2];
	Waypoint w;

#ifdef _HBHEURISTIC_PERFMON
	++perfCollisionPath;
	perfCollisionSegs += size - 1;
#endif

	// test for collisions in the range (w0, w1) - excluding w0 and w1
	for (U32 i = 1; i < size; ++i) {
		p[0] = Real(i) / Real(size);
		p[1] = REAL_ONE - p[0];

		// lineary interpolate coordinates
		for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
			w.cpos[j] = p[0] * w0.cpos[j] + p[1] * w1.cpos[j];

		// skip reference pose computation
		w.setup(controller, false, true);

		// test for collisions
		if (collides(w, data))
			return true;
	}

	return false;
}

//------------------------------------------------------------------------------

bool HBHeuristic::collides(const Waypoint &w) const {
	if (!desc.collisionDesc.enabled && !pointCloudCollision)
		return false;

	// find data pointer for the current thread
	ThreadData* data = getThreadData();

	return collides(w, data);
}

bool HBHeuristic::collides(const Waypoint &w0, const Waypoint &w1) const {
	if (!desc.collisionDesc.enabled && !pointCloudCollision)
		return false;

	// find data pointer for the current thread
	ThreadData* data = getThreadData();

	return collides(w0, w1, data);
}


//------------------------------------------------------------------------------

Real HBHeuristic::getCollisionCost(const Waypoint &wi, const Waypoint &wj, Hypothesis::Seq::const_iterator p) const {
	auto kernel = [&](Real x, Real lambda) -> Real {
		return /*lambda**/Math::exp(-lambda*x);
	};

	auto density = [&](const Real dist) -> Real {
		return (dist > ftDrivenDesc.ftModelDesc.distMax) ? REAL_ZERO : (dist < REAL_EPS) ? kernel(REAL_EPS, ftDrivenDesc.ftModelDesc.lambda) : kernel(dist, ftDrivenDesc.ftModelDesc.lambda); // esponential up to norm factor
	};

	Hypothesis::Seq::const_iterator maxLhdPose = pBelief->getHypotheses().begin();
	const Real norm = (REAL_ONE - density(ftDrivenDesc.ftModelDesc.distMax/*pBelief->myDesc.sensory.sensoryRange*/));//const Real norm = //(ftDrivenDesc.ftModelDesc.enabledLikelihood)?/*REAL_ONE/*/(REAL_ONE - kernel(ftDrivenDesc.ftModelDesc.lambda*ftDrivenDesc.ftModelDesc.distMax)):REAL_ONE;
	Real threshold(0.01), cost(REAL_ZERO);
	for (Chainspace::Index i = handInfo.getChains().begin(); i < handInfo.getChains().end(); ++i) {
		const RBCoord c(wj.wpos[i]);

		const Real d = 0;
		// penalise collisions with the shape of the mean pose
		if (d < threshold)
			return Node::COST_INF;
	}

	return cost;
}

Real HBHeuristic::testObservations(const RBCoord &pose, const bool normal) const {
//	const Real norm = (ftDrivenDesc.ftModelDesc.enabledLikelihood)?/*REAL_ONE/*/(REAL_ONE - density(ftDrivenDesc.ftModelDesc.distMax)):REAL_ONE;
	auto kernel = [&](Real x, Real lambda) -> Real {
		return /*lambda**/Math::exp(-lambda*x);
	};

	auto density = [&](const Real dist) -> Real {
		return (dist > ftDrivenDesc.ftModelDesc.distMax) ? REAL_ZERO : (dist < REAL_EPS) ? kernel(REAL_EPS, ftDrivenDesc.ftModelDesc.lambda) : kernel(dist, ftDrivenDesc.ftModelDesc.lambda); // esponential up to norm factor
	};
	const Real norm = (REAL_ONE - density(ftDrivenDesc.ftModelDesc.distMax));//(ftDrivenDesc.ftModelDesc.enabledLikelihood)?/*REAL_ONE/*/(REAL_ONE - kernel(ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.lambda)):REAL_ONE;
	pcl::PointXYZ searchPoint;
	searchPoint.x = (float)pose.p.x;
	searchPoint.y = (float)pose.p.y;
	searchPoint.z = (float)pose.p.z;

	std::vector<int> indeces;
	std::vector<float> distances;
	Real result = REAL_ZERO;
	Vec3 median;
	median.setZero();
	
	context.write("HBHeuristic:testObs(normal=%s):\n", normal ? "ON" : "OFF");
	if (normal) {
		Vec3 v;
		Mat34 jointFrame(Mat33(pose.q), pose.p);
		Mat34 jointFrameInv;
		jointFrameInv.setInverse(jointFrame);
		jointFrameInv.multiply(v, (*pBelief->getHypotheses().begin())->sample.p);
		v.normalise();
		if (v.z < 0)
			result = ftDrivenDesc.ftModelDesc.distMax + 1;

		context.write("joint pose <%f %f %f> mug frame <%f %f %f> v <%f %f %f> distance=%f result=%f density=%f\n",
			pose.p.x, pose.p.y, pose.p.z, (*pBelief->getHypotheses().begin())->sample.p.x, (*pBelief->getHypotheses().begin())->sample.p.y, (*pBelief->getHypotheses().begin())->sample.p.z,
			v.x, v.y, v.z, pose.p.distance(median), result, norm*density(result));
	}
	else {
	context.write("joint pose <%f %f %f> mug frame <%f %f %f> distance=%f result=%f density=%f\n",
		pose.p.x, pose.p.y, pose.p.z, (*pBelief->getHypotheses().begin())->sample.p.x, (*pBelief->getHypotheses().begin())->sample.p.y, (*pBelief->getHypotheses().begin())->sample.p.z,
		pose.p.distance(median), result, norm*density(result));
	}
	return result;
}

/** Reset collision bounds */

void golem::HBHeuristic::setHypothesisBounds() {
	hypothesisBoundsSeq.clear();
	for (auto i = pBelief->getHypotheses().begin(); i != pBelief->getHypotheses().end(); ++i) {
		Bounds::Seq tmp = (*i)->bounds();
		hypothesisBoundsSeq.insert(hypothesisBoundsSeq.begin(), tmp.begin(), tmp.end());
	}
}


//------------------------------------------------------------------------------

void golem::XMLData(HBHeuristic::FTModelDesc& val, XMLContext* context, bool create) {
	XMLData("dist_max", val.distMax, context, create);
	XMLData("cone_theta_orizontal_axis", val.coneTheta1, context, create);
	XMLData("cone_theta_vertical_axis", val.coneTheta2, context, create);
	XMLData("num_nearest_points", val.k, context, create);
	XMLData("num_points", val.points, context, create);
	XMLData("mahalanobis_fac", val.mahalanobisDistFac, context, create);
	XMLData("enabled_likelihood", val.enabledLikelihood, context, create);
	XMLData("intrinsic_exp_parameter", val.lambda, context, create);
	XMLData(val.jointFac, context->getContextFirst("joint_fac"), create);
}

void golem::XMLData(HBHeuristic::Desc& val, XMLContext* context, bool create) {
	//	XMLData((Heuristic::Desc)val, context, create);
	//	XMLData(val.beliefDesc, context->getContextFirst("belief_space"), create);
	XMLData("contact_fac", val.contactFac, context, create);
	XMLData("non_contact_fac", val.nonContactFac, context, create);
	XMLData("max_surface_points_inkd", val.maxSurfacePoints, context, create);
	XMLData(val.ftModelDesc, context->getContextFirst("ftmodel"), create);
	XMLData(&val.covariance[0], &val.covariance[RBCoord::N], "c", context->getContextFirst("covariance"), create);
	XMLData(val.evaluationDesc, context->getContextFirst("evaluation_model"), create);
	XMLData(val.checkDesc, context->getContextFirst("check_model"), create);
}
