/** @file DLRHitHandII.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/DLR/DLRHitHandII.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

DLRHitHandIIChain::DLRHitHandIIChain(Controller& controller) : Chain(controller) {
}

void DLRHitHandIIChain::create(const Desc& desc) {
	Chain::create(desc); // throws

	L0 = desc.L0;
	L1 = desc.L1;
	L2 = desc.L2;
}

//------------------------------------------------------------------------------

// DLRHitHandII froward kinematics
void DLRHitHandIIChain::chainForwardTransform(const Real* cc, Mat34& trn) const {
	//if (!customKinematics) {
		const Real _cc[DLRHitHandIIChain::NUM_JOINTS] = {cc[0], cc[1], cc[2], cc[2],}; // duplicate last joint
		Chain::chainForwardTransform(_cc, trn);
		return;
	//}

	// TODO
}

void DLRHitHandIIChain::jointForwardTransform(const Real* cc, Mat34* trn) const {
	//if (!customKinematics) {
		const Real _cc[DLRHitHandIIChain::NUM_JOINTS] = {cc[0], cc[1], cc[2], cc[2],}; // duplicate last joint
		Chain::jointForwardTransform(_cc, trn);
		return;
	//}

	// TODO
}

void DLRHitHandIIChain::velocitySpatial(const Real* cc, const Real* dcc, Twist& vs) const {
	//if (!customKinematics) {
		const Real _cc[DLRHitHandIIChain::NUM_JOINTS] = {cc[0], cc[1], cc[2], cc[2],}; // duplicate last joint
		const Real _dcc[DLRHitHandIIChain::NUM_JOINTS] = {dcc[0], dcc[1], dcc[2], dcc[2],}; // duplicate last joint
		Chain::velocitySpatial(_cc, _dcc, vs);
		return;
	//}

	// TODO
}

void DLRHitHandIIChain::jacobianSpatial(const Real* cc, Twist* jac) const {
	//if (!customKinematics) {
		const Real _cc[DLRHitHandIIChain::NUM_JOINTS] = {cc[0], cc[1], cc[2], cc[2],}; // duplicate last joint
		Chain::jacobianSpatial(_cc, jac);
		return;
	//}

	// TODO
}

//------------------------------------------------------------------------------

const Controller::ReservedOffset DLRHitHandII::RESERVED_OFFSET[RESERVED_INDEX_SIZE] = {
	/** Position (Configspace, Real) */
	ReservedOffset("Position", ReservedOffset::TYPE_CONFIGSPACE, 0, 0), // unavailable
	/** Velocity (Configspace, Real) */
	ReservedOffset("Velocity", ReservedOffset::TYPE_CONFIGSPACE, 0, 0), // unavailable
	/** Acceleration (Configspace, Real) */
	ReservedOffset("Acceleration", ReservedOffset::TYPE_CONFIGSPACE, 0, 0), // unavailable
	/** Force/Torque (Configspace, Real) */
	ReservedOffset("Force/Torque", ReservedOffset::TYPE_CONFIGSPACE, sizeof(Real), NUM_JOINTS),
	/** Joint stiffness (Configspace, Real) */
	ReservedOffset("Stiffness", ReservedOffset::TYPE_CONFIGSPACE, sizeof(Real), NUM_JOINTS),
	/** Joint damping (Configspace, Real) */
	ReservedOffset("Damping", ReservedOffset::TYPE_CONFIGSPACE, sizeof(Real), NUM_JOINTS),
	/** Finger enable (out) (Chainspace, bool) */
	ReservedOffset("Enable request", ReservedOffset::TYPE_CHAINSPACE, sizeof(bool), NUM_CHAINS),
	/** Finger enabled (inp) (Chainspace, bool) */
	ReservedOffset("Enabled", ReservedOffset::TYPE_CHAINSPACE, sizeof(bool), NUM_CHAINS),
	/** Maximum joint velocity (inp, out) (Configspace, Real) */
	ReservedOffset("Max velocity", ReservedOffset::TYPE_CONFIGSPACE, sizeof(Real), NUM_JOINTS),
	/** Kp (out) (Configspace, Real) */
	ReservedOffset("Kp", ReservedOffset::TYPE_CONFIGSPACE, sizeof(Real), NUM_JOINTS),
	/** Emergency (out) (bool) */
	ReservedOffset("Emergency", ReservedOffset::TYPE_DEFAULT, sizeof(bool), 1),
	/** Con mod (out) (int) */
	ReservedOffset("Con mod", ReservedOffset::TYPE_DEFAULT, sizeof(int), 1),
	/** Brakestatus (inp) (int) */
	ReservedOffset("Brake status", ReservedOffset::TYPE_DEFAULT, sizeof(int), 1),
	/** Commstatus (inp) (int) */
	ReservedOffset("Comm status", ReservedOffset::TYPE_DEFAULT, sizeof(int), 1),
	/** Handconfig (inp) (int) */
	ReservedOffset("Hand config", ReservedOffset::TYPE_DEFAULT, sizeof(int), 1),
};

DLRHitHandII::DLRHitHandII(golem::Context& context) : SingleCtrl(context) {
}

void DLRHitHandII::setStateInfo(const Controller::Desc& desc) {
	Controller::setStateInfo(desc); // throws
	ReservedOffset::getOffset(*this, RESERVED_OFFSET, RESERVED_OFFSET + RESERVED_INDEX_SIZE, desc.reservedBegin, desc.chainBegin, desc.jointBegin, reservedOffset);
}

ptrdiff_t DLRHitHandII::getReservedOffset(U32 type) const {
	return reservedOffset[type];
}

//------------------------------------------------------------------------------

void DLRHitHandII::setToDefault(State& state) const {
	SingleCtrl::setToDefault(state);
	// TODO: velocity limits (and other constants) equal zero before calling Controller::create()!!!!
	state.get<ConfigspaceCoord>(reservedOffset[RESERVED_INDEX_VELOCITY_MAX]).set(getStateInfo().getJoints(), getMax().cvel);
	state.get<int>(reservedOffset[RESERVED_INDEX_CON_MOD]) = 1;
	state.get<ChainspaceCoordBool>(reservedOffset[RESERVED_INDEX_ENABLE]).fill(getStateInfo().getChains(), true);
}

void DLRHitHandII::clampConfig(GenConfigspaceCoord& config) const {
	SingleCtrl::clampConfig(config);

	for (Chainspace::Index i = getStateInfo().getChains().begin(); i < getStateInfo().getChains().end(); ++i) {
		// mechanical coupling of the last finger joints
		const Configspace::Index src = getStateInfo().getJoints(i + 2).begin();
		const Configspace::Index dst = getStateInfo().getJoints(i + 3).begin();

		config.cpos[dst] = config.cpos[src];
		config.cvel[dst] = config.cvel[src];
		config.cacc[dst] = config.cacc[src];
	}
}

void DLRHitHandII::interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const {
	// interpolate config
	SingleCtrl::interpolateConfig(prev, next, t, state);
	
	// clamp on limits
	//state.t = Math::clamp(t, prev.t, next.t);
	
	// Linear interpolation of the real-valued continuous variables
	const bool interpol = prev.t + numeric_const<Real>::EPS < next.t;
	const Real p[2] = {interpol ? (next.t - state.t)/(next.t - prev.t) : REAL_ZERO, interpol ? (state.t - prev.t)/(next.t - prev.t) : REAL_ONE};
	const ptrdiff_t offset[] = {
		reservedOffset[RESERVED_INDEX_FORCE_TORQUE],
		reservedOffset[RESERVED_INDEX_STIFFNESS],
		reservedOffset[RESERVED_INDEX_DAMPING],
		reservedOffset[RESERVED_INDEX_VELOCITY_MAX],
		reservedOffset[RESERVED_INDEX_KP],
	};
	const size_t size = sizeof(offset)/sizeof(offset[0]);
	for (Configspace::Index i = getStateInfo().getJoints().begin(); i < getStateInfo().getJoints().end(); ++i)
		for (size_t j = 0; j < size; ++j)
			state.get<ConfigspaceCoord>(offset[j])[i] = p[0]*prev.get<ConfigspaceCoord>(offset[j])[i] + p[1]*next.get<ConfigspaceCoord>(offset[j])[i];

	// Copy binary variables from next
	getEnable(state).set(getStateInfo().getChains(), getEnable(next));
	getEnabled(state).set(getStateInfo().getChains(), getEnabled(next));
	getEmergency(state) = getEmergency(next);
	getConMod(state) = getConMod(next);
	getBrakeStatus(state) = getBrakeStatus(next);
	getCommStatus(state) = getCommStatus(next);
	getHandConfig(state) = getHandConfig(next);
}

//------------------------------------------------------------------------------
