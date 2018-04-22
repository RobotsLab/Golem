/** @file DLRHandII.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/DLR/DLRHandII.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

DLRHandIIChain::DLRHandIIChain(Controller& controller) : Chain(controller) {
}

void DLRHandIIChain::create(const Desc& desc) {
	Chain::create(desc); // throws

	L0 = desc.L0;
	L1 = desc.L1;
	L2 = desc.L2;
}

//------------------------------------------------------------------------------

// DLRHandII froward kinematics
void DLRHandIIChain::chainForwardTransform(const Real* cc, Mat34& trn) const {
	//if (!customKinematics) {
		const Real _cc[DLRHandIIChain::NUM_JOINTS] = {cc[0], cc[1], cc[2], cc[2],}; // duplicate last joint
		Chain::chainForwardTransform(_cc, trn);
		return;
	//}

	// TODO
}

void DLRHandIIChain::jointForwardTransform(const Real* cc, Mat34* trn) const {
	//if (!customKinematics) {
		const Real _cc[DLRHandIIChain::NUM_JOINTS] = {cc[0], cc[1], cc[2], cc[2],}; // duplicate last joint
		Chain::jointForwardTransform(_cc, trn);
		return;
	//}

	// TODO
}

void DLRHandIIChain::velocitySpatial(const Real* cc, const Real* dcc, Twist& vs) const {
	//if (!customKinematics) {
		const Real _cc[DLRHandIIChain::NUM_JOINTS] = {cc[0], cc[1], cc[2], cc[2],}; // duplicate last joint
		const Real _dcc[DLRHandIIChain::NUM_JOINTS] = {dcc[0], dcc[1], dcc[2], dcc[2],}; // duplicate last joint
		Chain::velocitySpatial(_cc, _dcc, vs);
		return;
	//}

	// TODO
}

void DLRHandIIChain::jacobianSpatial(const Real* cc, Twist* jac) const {
	//if (!customKinematics) {
		const Real _cc[DLRHandIIChain::NUM_JOINTS] = {cc[0], cc[1], cc[2], cc[2],}; // duplicate last joint
		Chain::jacobianSpatial(_cc, jac);
		return;
	//}

	// TODO
}

//------------------------------------------------------------------------------

const Controller::ReservedOffset DLRHandII::RESERVED_OFFSET[RESERVED_INDEX_SIZE] = {
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

DLRHandII::DLRHandII(golem::Context& context) : SingleCtrl(context) {
}

void DLRHandII::setStateInfo(const Controller::Desc& desc) {
	Controller::setStateInfo(desc); // throws
	ReservedOffset::getOffset(*this, RESERVED_OFFSET, RESERVED_OFFSET + RESERVED_INDEX_SIZE, desc.reservedBegin, desc.chainBegin, desc.jointBegin, reservedOffset);
}

ptrdiff_t DLRHandII::getReservedOffset(U32 type) const {
	return reservedOffset[type];
}

//------------------------------------------------------------------------------

void DLRHandII::setToDefault(State& state) const {
	SingleCtrl::setToDefault(state);
	// TODO
}

void DLRHandII::interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const {
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
