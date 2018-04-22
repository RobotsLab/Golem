/** @file PISASoftHand.cpp
 *
 * @author	Marek Kopicki
 * @author	Maxime Adjigble
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki and Maxime Adjigble, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/PISA/PISASoftHand.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const U32 PISASoftHand::CHAIN_NUM_JOINTS[PISASoftHand::NUM_CHAINS] = {
	PISASoftHand::NUM_JOINTS_THUMB,
	PISASoftHand::NUM_JOINTS_INDEX,
	PISASoftHand::NUM_JOINTS_MIDDLE,
	PISASoftHand::NUM_JOINTS_RING,
	PISASoftHand::NUM_JOINTS_SMALL,
};

//------------------------------------------------------------------------------

const Controller::ReservedOffset PISASoftHand::RESERVED_OFFSET[RESERVED_INDEX_SIZE] = {
	/** Position (Configspace, Real) */
	ReservedOffset("Position", ReservedOffset::TYPE_CONFIGSPACE, 0, 0), // unavailable
	/** Velocity (Configspace, Real) */
	ReservedOffset("Velocity", ReservedOffset::TYPE_CONFIGSPACE, 0, 0), // unavailable
	/** Acceleration (Configspace, Real) */
	ReservedOffset("Acceleration", ReservedOffset::TYPE_CONFIGSPACE, 0, 0), // unavailable
	/** Force/Torque (Configspace, Real) */
	ReservedOffset("Force/Torque", ReservedOffset::TYPE_CONFIGSPACE, 0, 0), // unavailable
	/** Joint stiffness (Configspace, Real) */
	ReservedOffset("Stiffness", ReservedOffset::TYPE_CONFIGSPACE, 0, 0), // unavailable
	/** Joint damping (Configspace, Real) */
	ReservedOffset("Damping", ReservedOffset::TYPE_CONFIGSPACE, 0, 0), // unavailable
	/** Motor position (inp/out) (1, short) */
	ReservedOffset("Motor position", ReservedOffset::TYPE_DEFAULT, sizeof(short), 1),
	/** Motor current (inp) (1, short) */
	ReservedOffset("Motor current", ReservedOffset::TYPE_DEFAULT, sizeof(short), 1),
};

//------------------------------------------------------------------------------

PISASoftHand::PISASoftHand(golem::Context& context) : SingleCtrl(context) {
	synergyPointer = 0; // use position, first joint
}

void PISASoftHand::create(const Desc& desc) {
	SingleCtrl::create(desc); // throws

	synergy = desc.synergy;
}


void PISASoftHand::setStateInfo(const Controller::Desc& desc) {
	Controller::setStateInfo(desc); // throws
	ReservedOffset::getOffset(*this, RESERVED_OFFSET, RESERVED_OFFSET + RESERVED_INDEX_SIZE, desc.reservedBegin, desc.chainBegin, desc.jointBegin, reservedOffset);
}

ptrdiff_t PISASoftHand::getReservedOffset(U32 type) const {
	return reservedOffset[type];
}

//------------------------------------------------------------------------------

Real PISASoftHand::motorPositionToSynergy(short position) const {
	return Real(position) / SYNERGY_GAIN;
}

short PISASoftHand::synergyToMotorPosition(Real synergy) const {
	return (short)Math::clamp(Math::round(synergy * SYNERGY_GAIN), Real(numeric_const<short>::MIN), Real(numeric_const<short>::MAX));
}

Real PISASoftHand::getSynergy(const State& state) const {
	return synergyPointer < NUM_JOINTS ? state.cpos[getStateInfo().getJoints().begin() + synergyPointer] : motorPositionToSynergy(getMotorPosition(state));
}

void PISASoftHand::setSynergy(Real s, State& state) const {
	if (synergyPointer < NUM_JOINTS)
		state.cpos[getStateInfo().getJoints().begin() + synergyPointer] = s;
	else
		getMotorPosition(state) = synergyToMotorPosition(s);
}

void PISASoftHand::getSynergyMap(Real s, Real* c) const {
	// TODO
	for (U32 i = 0; i < NUM_JOINTS; ++i)
		c[i] = s*synergy.data()[i];

	// Test only - comment out if required
	//const Real f = REAL_ONE; // scale
	//
	//c[0] = REAL_ZERO; // thumb, knuckle joint
	//c[1] = c[2] = f*s; // thumb, phalanges
	//
	//c[3] = REAL_ZERO; // index finger, knuckle joint
	//c[4] = c[5] = c[6] = f*s; // index finger, phalanges
	//
	//c[7] = REAL_ZERO; // middle finger, knuckle joint
	//c[8] = c[9] = c[10] = f*s; // middle finger, phalanges
	//
	//c[11] = REAL_ZERO; // ring finger, knuckle joint
	//c[12] = c[13] = c[14] = f*s; // ring finger, phalanges
	//
	//c[15] = REAL_ZERO; // small finger, knuckle joint
	//c[16] = c[17] = c[18] = f*s; // small finger, phalanges
}

void PISASoftHand::getSynergyMap(Real s, State& out) const {
	getSynergyMap(s, out.cpos.data() + *getStateInfo().getJoints().begin());
}

//------------------------------------------------------------------------------

void PISASoftHand::setToDefault(State& state) const {
	SingleCtrl::setToDefault(state); // sets all reserved area to 0
	
	//getMotorPosition(state) = 0;
	//getMotorCurrent(state) = 0;
}

void PISASoftHand::interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const {
	SingleCtrl::interpolateConfig(prev, next, t, state);

	// clamp on limits
	//state.t = Math::clamp(t, prev.t, next.t);

	// Linear interpolation of the real-valued continuous variables
	const bool interpol = prev.t + numeric_const<Real>::EPS < next.t;
	const Real p[2] = {interpol ? (next.t - state.t)/(next.t - prev.t) : REAL_ZERO, interpol ? (state.t - prev.t)/(next.t - prev.t) : REAL_ONE};
	
	// linearly interpolate synergy
	if (synergyPointer < NUM_JOINTS)
		state.cpos[getStateInfo().getJoints().begin() + synergyPointer] = p[0]*prev.cpos[getStateInfo().getJoints().begin() + synergyPointer] + p[1]*next.cpos[getStateInfo().getJoints().begin() + synergyPointer];

	// type: short
	const ptrdiff_t offsetShort[] = {
		reservedOffset[RESERVED_INDEX_MOTOR_POSITION],
		reservedOffset[RESERVED_INDEX_MOTOR_CURRENT],
	};
	for (idx_t i = sizeof(offsetShort)/sizeof(offsetShort[0]); --i > 0;) {
		const ptrdiff_t offset = offsetShort[i];
		state.get<short>(offset) = (short)Math::clamp(p[0]*prev.get<short>(offset) + p[1]*next.get<short>(offset), Real(numeric_const<short>::MIN), Real(numeric_const<short>::MAX));
	}
}

//------------------------------------------------------------------------------
