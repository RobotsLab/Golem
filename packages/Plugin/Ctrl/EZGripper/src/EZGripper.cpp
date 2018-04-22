/** @file EZGripper.cpp
*
* @author	Claudio Zito
*
* @copyright  Copyright (C) 2018 CLaudio Zito, University of Birmingham, UK
*
* @license  This file copy is licensed to you under the terms described in
*           the License.txt file included in this distribution.
*
*/

#include <Golem/Ctrl/EZGripper/EZGripper.h>
#include <Golem/Ctrl/SingleCtrl/Data.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Sys/LoadObjectDesc.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc) {
	loadObjectDesc<golem::EZGripper::Desc, Controller::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Controller::Desc::Ptr*)pControllerDesc);
}

const U32 EZGripper::CHAIN_NUM_JOINTS[EZGripper::NUM_CHAINS] = { EZGripper::NUM_JOINTS_LEFT, EZGripper::NUM_JOINTS_RIGHT };

const Controller::ReservedOffset EZGripper::RESERVED_OFFSET[RESERVED_INDEX_SIZE] = {
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

EZGripper::EZGripper(golem::Context& context) : SingleCtrl(context) {
	synergyPointer = 0; // use position, first joint
}

EZGripper::~EZGripper() {
	SingleCtrl::release();
}


void EZGripper::setStateInfo(const Controller::Desc& desc) {
	Controller::setStateInfo(desc); // throws
	ReservedOffset::getOffset(*this, RESERVED_OFFSET, RESERVED_OFFSET + RESERVED_INDEX_SIZE, desc.reservedBegin, desc.chainBegin, desc.jointBegin, reservedOffset);
}

ptrdiff_t EZGripper::getReservedOffset(U32 type) const {
	return reservedOffset[type];
}

//------------------------------------------------------------------------------

Real EZGripper::motorPositionToSynergy(short position) const {
	return Real(position) / SYNERGY_GAIN;
}

short EZGripper::synergyToMotorPosition(Real synergy) const {
	return (short)Math::clamp(Math::round(synergy * SYNERGY_GAIN), Real(numeric_const<short>::MIN), Real(numeric_const<short>::MAX));
}

Real EZGripper::getSynergy(const State& state) const {
	return synergyPointer < NUM_JOINTS ? state.cpos[getStateInfo().getJoints().begin() + synergyPointer] : motorPositionToSynergy(getMotorPosition(state));
}

void EZGripper::setSynergy(Real s, State& state) const {
	if (synergyPointer < NUM_JOINTS)
		state.cpos[getStateInfo().getJoints().begin() + synergyPointer] = s;
	else
		getMotorPosition(state) = synergyToMotorPosition(s);
}

void EZGripper::getSynergyMap(Real s, Real* c) const {
	// TODO
	for (U32 i = 0; i < NUM_JOINTS; ++i)
		c[i] = s*synergy.data()[i];

}

void EZGripper::getSynergyMap(Real s, State& out) const {
	getSynergyMap(s, out.cpos.data() + *getStateInfo().getJoints().begin());
}

//------------------------------------------------------------------------------

void EZGripper::create(const Desc& desc) {
	SingleCtrl::create(desc);

	synergy = desc.synergy;
}

void EZGripper::lookupState(SecTmReal t, State &state) const {
	SingleCtrl::lookupState(t, state);

	// mechanical coupling of the base of left and right finger
	const Configspace::Index src = getStateInfo().getJoints(getStateInfo().getChains().begin() + LEFT_FINGER_CHAIN_INDEX).begin();
	const Configspace::Index dst = getStateInfo().getJoints(getStateInfo().getChains().begin() + RIGHT_FINGER_CHAIN_INDEX).begin();

	state.cpos[dst] = state.cpos[src];
	state.cvel[dst] = state.cvel[src];
	state.cacc[dst] = state.cacc[src];

	// TODO reserved area
}

void EZGripper::setToDefault(State& state) const {
	SingleCtrl::setToDefault(state);
	// TODO
}

void EZGripper::clampConfig(GenConfigspaceCoord& config) const {
	SingleCtrl::clampConfig(config);

	// mechanical coupling of the base of left and right finger
	const Configspace::Index src = getStateInfo().getJoints(getStateInfo().getChains().begin() + LEFT_FINGER_CHAIN_INDEX).begin();
	const Configspace::Index dst = getStateInfo().getJoints(getStateInfo().getChains().begin() + RIGHT_FINGER_CHAIN_INDEX).begin();

	config.cpos[dst] = config.cpos[src];
	config.cvel[dst] = config.cvel[src];
	config.cacc[dst] = config.cacc[src];
}

void EZGripper::interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const {
	// interpolate config
	SingleCtrl::interpolateConfig(prev, next, t, state);

	// clamp on limits
	//state.t = Math::clamp(t, prev.t, next.t);

	// Linear interpolation of the real-valued continuous variables
	const bool interpol = prev.t + numeric_const<Real>::EPS < next.t;
	const Real p[2] = { interpol ? (next.t - state.t) / (next.t - prev.t) : REAL_ZERO, interpol ? (state.t - prev.t) / (next.t - prev.t) : REAL_ONE };

	// linearly interpolate synergy
	if (synergyPointer < NUM_JOINTS)
		state.cpos[getStateInfo().getJoints().begin() + synergyPointer] = p[0] * prev.cpos[getStateInfo().getJoints().begin() + synergyPointer] + p[1] * next.cpos[getStateInfo().getJoints().begin() + synergyPointer];

	// type: short
	const ptrdiff_t offsetShort[] = {
		reservedOffset[RESERVED_INDEX_MOTOR_POSITION],
		reservedOffset[RESERVED_INDEX_MOTOR_CURRENT],
	};
	for (idx_t i = sizeof(offsetShort) / sizeof(offsetShort[0]); --i > 0;) {
		const ptrdiff_t offset = offsetShort[i];
		state.get<short>(offset) = (short)Math::clamp(p[0] * prev.get<short>(offset) + p[1] * next.get<short>(offset), Real(numeric_const<short>::MIN), Real(numeric_const<short>::MAX));
	}
}


//------------------------------------------------------------------------------

void EZGripper::sysRecv(State& state) {
	// default interpolation
	SingleCtrl::sysRecv(state);
	// apply synergy
	getSynergyMap(getSynergy(state), state);
}
//------------------------------------------------------------------------------

void golem::XMLData(EZGripper::Desc &val, XMLContext* context, bool create) {
	XMLData((SingleCtrl::Desc&)val, context, create);

	XMLData(val.synergy, "c", context->getContextFirst("synergy"), create);
}

//------------------------------------------------------------------------------
