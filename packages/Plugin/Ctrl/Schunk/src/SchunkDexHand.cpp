/** @file SchunkDexHand.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/Schunk/SchunkDexHand.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const U32 SchunkDexHand::CHAIN_NUM_JOINTS[SchunkDexHand::NUM_CHAINS] = {SchunkDexHand::NUM_JOINTS_MIDDLE, SchunkDexHand::NUM_JOINTS_LEFT, SchunkDexHand::NUM_JOINTS_RIGHT};

const Controller::ReservedOffset SchunkDexHand::RESERVED_OFFSET[RESERVED_INDEX_SIZE] = {
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
};

SchunkDexHand::SchunkDexHand(golem::Context& context) : SingleCtrl(context) {
}

void SchunkDexHand::setStateInfo(const Controller::Desc& desc) {
	Controller::setStateInfo(desc); // throws
	ReservedOffset::getOffset(*this, RESERVED_OFFSET, RESERVED_OFFSET + RESERVED_INDEX_SIZE, desc.reservedBegin, desc.chainBegin, desc.jointBegin, reservedOffset);
}

ptrdiff_t SchunkDexHand::getReservedOffset(U32 type) const {
	return reservedOffset[type];
}

//------------------------------------------------------------------------------

void SchunkDexHand::lookupState(SecTmReal t, State &state) const {
	SingleCtrl::lookupState(t, state);
	
	// mechanical coupling of the base of left and right finger
	const Configspace::Index src = getStateInfo().getJoints(getStateInfo().getChains().begin() + LEFT_FINGER_CHAIN_INDEX).begin();
	const Configspace::Index dst = getStateInfo().getJoints(getStateInfo().getChains().begin() + RIGHT_FINGER_CHAIN_INDEX).begin();

	state.cpos[dst] = state.cpos[src];
	state.cvel[dst] = state.cvel[src];
	state.cacc[dst] = state.cacc[src];

	// TODO reserved area
}

void SchunkDexHand::setToDefault(State& state) const {
	SingleCtrl::setToDefault(state);
	// TODO
}

void SchunkDexHand::clampConfig(GenConfigspaceCoord& config) const {
	SingleCtrl::clampConfig(config);

	// mechanical coupling of the base of left and right finger
	const Configspace::Index src = getStateInfo().getJoints(getStateInfo().getChains().begin() + LEFT_FINGER_CHAIN_INDEX).begin();
	const Configspace::Index dst = getStateInfo().getJoints(getStateInfo().getChains().begin() + RIGHT_FINGER_CHAIN_INDEX).begin();

	config.cpos[dst] = config.cpos[src];
	config.cvel[dst] = config.cvel[src];
	config.cacc[dst] = config.cacc[src];
}

void SchunkDexHand::interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const {
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
	};
	const size_t size = sizeof(offset)/sizeof(offset[0]);
	for (Configspace::Index i = getStateInfo().getJoints().begin(); i < getStateInfo().getJoints().end(); ++i)
		for (size_t j = 0; j < size; ++j)
			state.get<ConfigspaceCoord>(offset[j])[i] = p[0]*prev.get<ConfigspaceCoord>(offset[j])[i] + p[1]*next.get<ConfigspaceCoord>(offset[j])[i];

	// copy binary variables from next
}

//------------------------------------------------------------------------------
