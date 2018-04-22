/** @file Profile.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Planner/Profile.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Profile::Profile(const Controller &controller) :
	controller(controller)
{
}

Profile::~Profile() {
}

void Profile::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgProfileInvalidDesc(Message::LEVEL_CRIT, "Profile::create(): Invalid description");

	stateInfo = controller.getStateInfo();

	if (desc.velocity.size() < (size_t)stateInfo.getJoints().size() || desc.velocity.size() < (size_t)stateInfo.getJoints().size())
		throw Message(Message::LEVEL_CRIT, "Profile::create(): Invalid dimensionality of velocity limit");
	velocity.set(desc.velocity.data(), desc.velocity.data() + stateInfo.getJoints().size(), *stateInfo.getJoints().begin());

	if (desc.acceleration.size() < (size_t)stateInfo.getJoints().size() || desc.acceleration.size() < (size_t)stateInfo.getJoints().size())
		throw Message(Message::LEVEL_CRIT, "Profile::create(): Invalid dimensionality of acceleration limit");
	acceleration.set(desc.acceleration.data(), desc.acceleration.data() + stateInfo.getJoints().size(), *stateInfo.getJoints().begin());

	pTrajectory = desc.pTrajectoryDesc->create(); // throws
	pCallbackDist = desc.pCallbackDist;

	pruning = desc.pruning;
	pruningDistance = desc.pruningDistance;
}

//------------------------------------------------------------------------------

void Profile::setCallbackDist(const CallbackDist& callbackDist) {
	this->pCallbackDist = &callbackDist;
}

void Profile::velocityGain(const Polynomial4& trj, Real min, Real max, Real& gain) const {
	Real tmin, tmax, vmin, vmax;
	trj.getVelocityExtrema(tmin, tmax, vmin, vmax);

	gain = std::max(gain, Math::abs(vmin/min));
	gain = std::max(gain, Math::abs(vmax/max));
}

void Profile::accelerationGain(const Polynomial4& trj, Real min, Real max, Real& gain) const {
	Real tmin, tmax, amin, amax;
	trj.getAccelerationExtrema(tmin, tmax, amin, amax);
	
	gain = std::max(gain, Math::sqrt(Math::abs(amin/min)));
	gain = std::max(gain, Math::sqrt(Math::abs(amax/max)));
}

//------------------------------------------------------------------------------

void Profile::profileTime(Controller::State::Seq &seq, Controller::State::Seq::iterator& begin, Controller::State::Seq::iterator& end) const {
	size_t size = 0;
	const SecTmReal start = begin->t;
	const SecTmReal duration = std::max((end - 1)->t - start, controller.getCycleDuration());

	for (;;) {
		// memorise pointers before modifying container
		const size_t offset = (size_t)(begin - seq.begin());
		size = (size_t)(end - begin);
		if (size < 2)
			throw MsgProfileSize(Message::LEVEL_ERROR, "Profile::profileTime(): At least two states required");

		// debug
		//std::string str;

		// compute distances
		Real minDist = REAL_MAX;
		size_t minIndex = 0;
		begin->t = SEC_TM_REAL_ZERO;
		for (size_t j = 1; j < size; ++j) {
			(begin + j)->t = (SecTmReal)pCallbackDist->distConfigspaceCoord((begin + (j - 1))->cpos, (begin + j)->cpos);
			if (minDist >(Real)(begin + j)->t) {
				minDist = (Real)(begin + j)->t;
				minIndex = j;
			}

			// debug
			//str += std::to_string((begin + j)->t) + ", ";
		}

		// debug
		//printf("Profile::profileTime(%x): {%s}\n", (size_t)this, str.c_str());

		// make sure there is anything to remove
		if (!pruning || size < 3 || minDist > pruningDistance || minIndex <= 0)
			break;
		// minimise maximum length of the remaining left (minIndex - 2, minIndex - 1) and right (minIndex, minIndex + 1) segments
		const size_t index = minIndex <= 1 || minIndex < size - 1 && (begin + minIndex + 1)->t < (begin + minIndex - 1)->t ? minIndex : minIndex - 1;
		//controller.getContext().debug("Profile::profile(): pruning #%u/%u, distance=%e/%e\n", index + 1, size, minDist, pruningDistance);

		// prune a single waypoint
		seq.erase(begin + index);

		// update pointers
		begin = seq.begin() + offset;
		end = seq.begin() + (offset + size - 1); // one less

		// callback
		pCallbackDist->distRemoved(index);
	}

	ConfigspaceCoord::Seq coordSeq(size);// reserve space
	for (size_t j = 1; j < size; ++j) {
		// accumulate distances
		(begin + j)->t += (begin + (j - 1))->t;

		// Coordinate distances
		for (ConfigspaceCoord::Index n = stateInfo.getJoints().begin(); n < stateInfo.getJoints().end(); ++n)
			if (pCallbackDist->distCoordPlanning(n)) {
				// coordinates dist
				const Real coordDist = pCallbackDist->distCoord((begin + (j - 1))->cpos[n], (begin + j)->cpos[n]);

				if (j == 1) coordSeq[0][n] = REAL_ZERO; // init with 0
				coordSeq[j][n] = coordSeq[j - 1][n] + coordDist;
			}
	}

	const Controller::State::Seq::iterator last = end - 1;

	// compute distance ---> time
	for (size_t i = 0; i < size; ++i) {
		Controller::State::Seq::iterator curr = begin + i;

		Real t = duration / (size - 1);
		if (last->t > REAL_EPS && !pTrajectory->getPositionInverse(t, curr->t / last->t))
			throw MsgProfileInverse(Message::LEVEL_ERROR, "Profile::profileTime(): Unable to compute inverse average trajectory profile: size=%d, time=%f, duration=%f", size, curr->t, last->t);
		curr->t = start + duration*t;

		for (ConfigspaceCoord::Index n = stateInfo.getJoints().begin(); n < stateInfo.getJoints().end(); ++n)
			if (pCallbackDist->distCoordPlanning(n)) {
				Real t = REAL_ZERO;
				if (coordSeq.back()[n] > REAL_EPS && !pTrajectory->getPositionInverse(t, coordSeq[i][n] / coordSeq.back()[n]))
					throw MsgProfileInverse(Message::LEVEL_ERROR, "Profile::profileTime(): Unable to compute inverse joint trajectory profile: size=%d, time=%f, duration=%f", size, coordSeq[i][n], coordSeq.back()[n]);
				coordSeq[i][n] = start + duration*t;
			}
	}

	// nothing to do if there are only two almost identical waypoints left
	if (size <= 2 && last->t - begin->t < REAL_EPS) {
		begin->t = start; // restore time
		last->t = start + duration; // restore time
		return;
	}

	// make sure duration is valid
	if (last->t - begin->t < REAL_EPS)
		throw MsgProfileSize(Message::LEVEL_ERROR, "Profile::profileTime(): Invalid waypoint distance %.9f < eps", last->t);
}

void Profile::profileVelocity(Controller::State::Seq::iterator begin, Controller::State::Seq::iterator end) const {
	const size_t size = (size_t)(end - begin);
	if (size < 3)
		return;

	// compute velocities using 3rd-degree polynomial
	//profile(begin, begin + size / 2);
	//const Controller::State::Seq::reverse_iterator rbegin(end);
	//profile(rbegin, rbegin + size / 2 + size % 2);

	//Rand rand(controller.getContext().getRandSeed());
	// compute velocities using 2nd-degree polynomial
	const Controller::State::Seq::iterator last = end - 1;
	for (Configspace::Index n = controller.getStateInfo().getJoints().begin(); n < controller.getStateInfo().getJoints().end(); ++n)
		if (pCallbackDist->distCoordPlanning(n)) {
			for (Controller::State::Seq::iterator pi = begin, pj = begin + 1, pk = begin + 2; pj < last; ++pi, ++pj, ++pk) {
				Polynomial3 trj;
				trj.set3ppp(pi->t, pj->t, pk->t, pi->cpos[n], pj->cpos[n], pk->cpos[n]);
				pj->cvel[n] = trj.getVelocity(pj->t);
				pj->cacc[n] = REAL_ZERO;
			}
		}
		else if (pCallbackDist->distCoordInterpolation(n)) {
			Polynomial4 trj;
			trj.set2pvpv(begin->t, last->t, begin->cpos[n], begin->cvel[n], last->cpos[n], last->cvel[n]);
			for (Controller::State::Seq::iterator pj = begin + 1; pj < last; ++pj) {
				pj->cpos[n] = trj.getPosition(pj->t);
				pj->cvel[n] = trj.getVelocity(pj->t);
				pj->cacc[n] = REAL_ZERO;
			}
		}
}

void Profile::optimise(Controller::State::Seq::iterator begin, Controller::State::Seq::iterator end) const {
	const size_t size = (size_t)(end - begin);
	if (size < 3)
		return;

	//Rand rand(controller.getContext().getRandSeed());
	const Controller::State::Seq::iterator last = end - 1;
	for (Configspace::Index n = controller.getStateInfo().getJoints().begin(); n < controller.getStateInfo().getJoints().end(); ++n)
		if (pCallbackDist->distCoordPlanning(n)) {
			//const size_t steps = 10000 * size;
			//for (size_t i = 0; i < steps; ++i) {
			//	const size_t ptr = rand.next() % (size - 2);
			//	const Controller::State::Seq::iterator pi = begin + ptr, pj = begin + ptr + 1, pk = begin + ptr + 2;
			//	Polynomial4 l, r;
			//	Real min, max, tmin, tmax, lcurr, ltest, rcurr, rtest;

			//	l.set2pvpv(pi->t, pj->t, pi->cpos[n], pi->cvel[n], pj->cpos[n], pj->cvel[n]);
			//	l.getAccelerationExtrema(tmin, tmax, min, max);
			//	lcurr = std::max(Math::abs(min), Math::abs(max));
			//	r.set2pvpv(pj->t, pk->t, pj->cpos[n], pj->cvel[n], pk->cpos[n], pk->cvel[n]);
			//	r.getAccelerationExtrema(tmin, tmax, min, max);
			//	rcurr = std::max(Math::abs(min), Math::abs(max));
			//	
			//	const Real vtest = (REAL_ONE + rand.nextUniform<Real>(-0.1, 0.1))*pj->cvel[n];

			//	l.set2pvpv(pi->t, pj->t, pi->cpos[n], pi->cvel[n], pj->cpos[n], vtest);
			//	l.getAccelerationExtrema(tmin, tmax, min, max);
			//	ltest = std::max(Math::abs(min), Math::abs(max));
			//	r.set2pvpv(pj->t, pk->t, pj->cpos[n], vtest, pk->cpos[n], pk->cvel[n]);
			//	r.getAccelerationExtrema(tmin, tmax, min, max);
			//	rtest = std::max(Math::abs(min), Math::abs(max));

			//	if (lcurr + rcurr > ltest + rtest) {
			//		//printf("(%u, %u): acc: %f -> %f, vel: %f -> %f\n", *n + 1, ptr + 1, (lcurr + rcurr), (ltest + rtest), pj->cvel[n], vtest);
			//		pj->cvel[n] = vtest;
			//	}
			//}
		}
}

void Profile::rescale(Controller::State::Seq::iterator begin, Controller::State::Seq::iterator end) const {
	const size_t size = (size_t)(end - begin);
	if (size < 2)
		throw MsgProfileSize(Message::LEVEL_ERROR, "Profile::rescale(): At least two states required");

	const Controller::State::Seq::iterator last = end - 1;

	// compute gain
	Real gain = REAL_ONE;
	const GenConfigspaceCoord gccMin = controller.getMin();
	const GenConfigspaceCoord gccMax = controller.getMax();
	for (Configspace::Index n = controller.getStateInfo().getJoints().begin(); n < controller.getStateInfo().getJoints().end(); ++n)
		if (pCallbackDist->distCoordPlanning(n))
			for (Controller::State::Seq::const_iterator pi = begin, pj = begin + 1; pi != last; ++pi, ++pj) {
				Polynomial4 trj;
				trj.set2pvpv(pi->t, pj->t, pi->cpos[n], pi->cvel[n], pj->cpos[n], pj->cvel[n]);
				velocityGain(trj, velocity[n] * gccMin.cvel[n], velocity[n] * gccMax.cvel[n], gain);
				accelerationGain(trj, acceleration[n] * gccMin.cacc[n], acceleration[n] * gccMax.cacc[n], gain);
			}
	const Real gainInv = REAL_ONE / gain;

	// update time stamps
	for (Controller::State::Seq::iterator pi = begin, pj = begin + 1; pi != last; ++pi, ++pj)
		pj->t = begin->t + gain*(pj->t - begin->t);

	// update velocities
	for (Configspace::Index n = controller.getStateInfo().getJoints().begin(); n < controller.getStateInfo().getJoints().end(); ++n)
		if (pCallbackDist->distCoordPlanning(n) || pCallbackDist->distCoordInterpolation(n))
			for (Controller::State::Seq::iterator pi = begin, pj = begin + 1; pi != last; ++pi, ++pj)
				pj->cvel[n] *= gainInv;
}

void Profile::prepare(Controller::State::Seq::iterator begin, Controller::State::Seq::iterator end) const {
	// make sure unused coordinates are consistent
	for (Controller::State::Seq::iterator pj = begin + 1; begin < end && pj < end; ++pj)
		for (Configspace::Index n = controller.getStateInfo().getJoints().begin(); n < controller.getStateInfo().getJoints().end(); ++n)
			if (!pCallbackDist->distCoordPlanning(n) && !pCallbackDist->distCoordInterpolation(n)) {
				pj->cpos[n] = begin->cpos[n];
				pj->cvel[n] = REAL_ZERO;
				pj->cacc[n] = REAL_ZERO;
			}
}

//------------------------------------------------------------------------------

void Profile::profile(Controller::State::Seq &seq) const {
	Controller::State::Seq::iterator begin = seq.begin(), end = seq.end();

	profile(seq, begin, end);
}

void Profile::profile(Controller::State::Seq &seq, Controller::State::Seq::iterator& begin, Controller::State::Seq::iterator& end) const {
	// initialise sequence
	prepare(begin, end);

	// find time stamps
	profileTime(seq, begin, end);

	// find velocities
	profileVelocity(begin, end);

	// Optimise trajectory
	optimise(begin, end);

	// Re-scale trajectory duration
	rescale(begin, end);
}

//------------------------------------------------------------------------------
