/** @file KukaIIWA.cpp
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/Kuka/KukaIIWA.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void KukaIIWAChain::ChainModel::create() {
	// TODO at some point
}

// Kuka custom froward kinematics
void KukaIIWAChain::ChainModel::chainForwardTransform(const Mat34& localPose, const Real* cc, Mat34& trn) const {
	Mat34 tmp;

	trn.fromTwist(trnInitCoord[NUM_JOINTS - 1].twist, trnInitCoord[NUM_JOINTS - 1].theta);

	for (I32 i = NUM_JOINTS - 1; i >= 0; --i) {
		tmp.fromTwist(trnCoord[i].twist, cc[i] + encoderOffset[i]);
		trn.multiply(tmp, trn);
	}

	trn.multiply(localPose, trn);
}

void KukaIIWAChain::ChainModel::jointForwardTransform(const Mat34& localPose, const Real* cc, Mat34* trn) const {
	ASSERT(trn != NULL)
	Mat34 tmp;

	trn[0].fromTwist(trnCoord[0].twist, cc[0] + encoderOffset[0]);
	for (U32 i = 1; i < NUM_JOINTS; ++i) {
		trn[i].fromTwist(trnCoord[i].twist, cc[i] + encoderOffset[i]);
		trn[i].multiply(trn[i - 1], trn[i]);
	}

	for (U32 i = 0; i < NUM_JOINTS; ++i) {
		tmp.fromTwist(trnInitCoord[i].twist, trnInitCoord[i].theta);
		trn[i].multiply(trn[i], tmp);
		trn[i].multiply(localPose, trn[i]);
	}
}

void KukaIIWAChain::ChainModel::jacobianSpatial(const Real* cc, Twist* jac) const {
	Mat34 tmp, trn;
	trn.setId();

	jac[0] = trnCoord[0].twist;
	for (U32 i = 1; i < NUM_JOINTS; ++i) {
		tmp.fromTwist(trnCoord[i - 1].twist, cc[i - 1]);
		trn.multiply(trn, tmp);
		trn.adjointTransform(jac[i], trnCoord[i].twist);
	}
}

//------------------------------------------------------------------------------

KukaIIWAChain::KukaIIWAChain(Controller& controller) : Chain(controller) {
}

void KukaIIWAChain::create(const Desc& desc) {
	Chain::create(desc); // throws
	setChainModel(desc.chainModel);
}

void KukaIIWAChain::setChainModel(const ChainModel& chainModel) {
	if (!chainModel.isValid())
		throw MsgChainInvalidDesc(Message::LEVEL_CRIT, "KukaIIWAChain::setChainModel(): Invalid chain model");
	this->chainModel = chainModel;
	this->chainModel.create();
}

//------------------------------------------------------------------------------

// Kuka froward kinematics
void KukaIIWAChain::chainForwardTransform(const Real* cc, Mat34& trn) const {
	if (customKinematics)
		chainModel.chainForwardTransform(getLocalPose(), cc, trn);
	else
		Chain::chainForwardTransform(cc, trn);
}

void KukaIIWAChain::jointForwardTransform(const Real* cc, Mat34* trn) const {
	if (customKinematics)
		chainModel.jointForwardTransform(getLocalPose(), cc, trn);
	else
		Chain::jointForwardTransform(cc, trn);
}

void KukaIIWAChain::jacobianSpatial(const Real* cc, Twist* jac) const {
	if (customKinematics)
		chainModel.jacobianSpatial(cc, jac);
	else
		Chain::jacobianSpatial(cc, jac);
}

//------------------------------------------------------------------------------

const Controller::ReservedOffset KukaIIWA::RESERVED_OFFSET[RESERVED_INDEX_SIZE] = {
		/** Position (Configspace, Real) */
		ReservedOffset("Position", ReservedOffset::TYPE_CONFIGSPACE, 0, 0), // unavailable
		/** Velocity (Configspace, Real) */
		ReservedOffset("Velocity", ReservedOffset::TYPE_CONFIGSPACE, 0, 0), // unavailable
		/** Acceleration (Configspace, Real) */
		ReservedOffset("Acceleration", ReservedOffset::TYPE_CONFIGSPACE, 0, 0), // unavailable
		/** Force/Torque (Configspace, Real) */
		ReservedOffset("Force/Torque", ReservedOffset::TYPE_CONFIGSPACE, sizeof(Real), KukaIIWAChain::NUM_JOINTS),
		/** Joint stiffness (Configspace, Real) */
		ReservedOffset("Stiffness", ReservedOffset::TYPE_CONFIGSPACE, sizeof(Real), KukaIIWAChain::NUM_JOINTS),
		/** Joint damping (Configspace, Real) */
		ReservedOffset("Damping", ReservedOffset::TYPE_CONFIGSPACE, sizeof(Real), KukaIIWAChain::NUM_JOINTS),
		/**  Change the control mode*/
		ReservedOffset("Next Control Mode", ReservedOffset::TYPE_DEFAULT, sizeof(int), 1),
		/** In joint impedence mode - specifies the desired position*/
		ReservedOffset("JntImp Next Pos", ReservedOffset::TYPE_CONFIGSPACE, sizeof(Real), KukaIIWAChain::NUM_JOINTS),
		/** In joint impedence mode - offset on the torque value*/
		ReservedOffset("JntImp Add Torque", ReservedOffset::TYPE_CONFIGSPACE, sizeof(Real), KukaIIWAChain::NUM_JOINTS),
		/** In cartesian impedance mode - desired position*/
		ReservedOffset("CartImp Next Pos", ReservedOffset::TYPE_DEFAULT, sizeof(Real), KukaIIWA::SE3_DIM),
		/** In cartesian impedance mode - desired stiffness*/
		ReservedOffset("CartImp Stiffness", ReservedOffset::TYPE_DEFAULT, sizeof(Real), KukaIIWA::TWIST_DIM),
		/** In cartesian impedance mode - desired damping*/
		ReservedOffset("CartImp Damping", ReservedOffset::TYPE_DEFAULT, sizeof(Real), KukaIIWA::TWIST_DIM),
		/** In cartesian impedance mode - offset on the end effector force and torque*/
		ReservedOffset("CartImp Add Tcp FT", ReservedOffset::TYPE_DEFAULT, sizeof(Real), KukaIIWA::TWIST_DIM),
		/** In cartesian impedance mode - joint null space control*/
		ReservedOffset("CartImp Jnt Null Space", ReservedOffset::TYPE_CONFIGSPACE, sizeof(Real), KukaIIWAChain::NUM_JOINTS),
		/** Value of the current control mode*/
		ReservedOffset("Current Control Mode", ReservedOffset::TYPE_DEFAULT, sizeof(int), 1),
		/** Current communication quality*/
		ReservedOffset("Comm Quality", ReservedOffset::TYPE_DEFAULT, sizeof(int), 1),
		/** State*/
		ReservedOffset("Robot State", ReservedOffset::TYPE_DEFAULT, sizeof(int), 1),
		/** Power state*/
		ReservedOffset("Robot Power Is On?", ReservedOffset::TYPE_DEFAULT, sizeof(int), 1),
};

KukaIIWA::KukaIIWA(golem::Context& context) : SingleCtrl(context) {
}

//------------------------------------------------------------------------------

void KukaIIWA::setToDefault(State& state) const {
	SingleCtrl::setToDefault(state);
	/** Reset the reserved area */
	for (Configspace::Index i = getStateInfo().getJoints().begin(); i < getStateInfo().getJoints().end(); ++i)
	{
		/** Force/Torque*/
		state.get<ConfigspaceCoord>(reservedOffset[RESERVED_INDEX_FORCE_TORQUE])[i] = 0.0;
		/** Joint stiffness 0=Gravity compensation mode min=0.01 max=2000 default=1000*/
		state.get<ConfigspaceCoord>(reservedOffset[RESERVED_INDEX_STIFFNESS])[i] = 0.0;
		/** Joint damping min=0.1 max=1.0 default=0.7*/
		state.get<ConfigspaceCoord>(reservedOffset[RESERVED_INDEX_DAMPING])[i] = 0.7;
		/** JntImp desired position*/
		state.get<ConfigspaceCoord>(reservedOffset[RESERVED_INDEX_JNTIMP_NEXT_POSITION])[i] = 0.0;
		/** torque offset value*/
		state.get<ConfigspaceCoord>(reservedOffset[RESERVED_INDEX_JNTIMP_ADD_TORQUE])[i] = 0.0;
		/** cartesian impedance mode - joint null space control*/
		state.get<ConfigspaceCoord>(reservedOffset[RESERVED_INDEX_CARTIMP_JNT_NULL_SPACE])[i] = 0.0;
	}

	/** Change the control mode 10-position control 20-Cartesian impedance control 30-Joint impedance control*/
	state.get<int>(reservedOffset[RESERVED_INDEX_NEXT_CONTROL_MODE]) = 10;
	/** cartesian impedance mode - desired position*/
	state.get<golem::Mat34>(reservedOffset[RESERVED_INDEX_CARTIMP_NEXT_POSITION]).setToDefault();
	/** cartesian impedance mode - desired stiffness 0=Gravity compensation (Fmin=0.01 Tmin=0.01) (Fmax=5000 Tmax=300) (Fdefault=2000 Tdefault=200)*/
	state.get<golem::Twist>(reservedOffset[RESERVED_INDEX_CARTIMP_STIFFNESS]).setZero();
	/** cartesian impedance mode - desired damping min=0.1 max=1.0 default=0.7*/
	state.get<golem::Twist>(reservedOffset[RESERVED_INDEX_CARTIMP_DAMPING]).set(0.7,0.7,0.7,0.7,0.7,0.7);
	/** cartesian impedance mode - offset on the end effector force and torque*/
	state.get<golem::Twist>(reservedOffset[RESERVED_INDEX_CARTIMP_ADD_TCP_FT]).setZero();
	/** current control mode*/
	state.get<int>(reservedOffset[RESERVED_INDEX_CURRENT_CONTROL_MODE]) = -1;
	/** communication quality*/
	state.get<int>(reservedOffset[RESERVED_INDEX_COM_QUALITY]) = 0;
	/** Robot State FRI_STATE_INVALID =-1, FRI_STATE_OFF = 0, FRI_STATE_MON = 1, FRI_STATE_CMD = 2  */
	state.get<int>(reservedOffset[RESERVED_INDEX_ROBOT_STATE]) = 0;
	/** Power state*/
	state.get<int>(reservedOffset[RESERVED_INDEX_ROBOT_POWER_IS_ON]) = 0;

}

void KukaIIWA::setStateInfo(const Controller::Desc& desc) {
	Controller::setStateInfo(desc); // throws
	ReservedOffset::getOffset(*this, RESERVED_OFFSET, RESERVED_OFFSET + RESERVED_INDEX_SIZE, desc.reservedBegin, desc.chainBegin, desc.jointBegin, reservedOffset);
}

ptrdiff_t KukaIIWA::getReservedOffset(U32 type) const {
	return reservedOffset[type];
}

void KukaIIWA::lookupState(SecTmReal t, State &state) const {
	SingleCtrl::lookupState(t, state);

	//Todo Reserved area
}

void KukaIIWA::interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const {
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
			reservedOffset[RESERVED_INDEX_JNTIMP_NEXT_POSITION],
			reservedOffset[RESERVED_INDEX_JNTIMP_ADD_TORQUE],
			/*	reservedOffset[RESERVED_INDEX_CARTIMP_NEXT_POSITION],
                reservedOffset[RESERVED_INDEX_CARTIMP_STIFFNESS],
                reservedOffset[RESERVED_INDEX_CARTIMP_DAMPING],
                reservedOffset[RESERVED_INDEX_CARTIMP_ADD_TCP_FT],
                reservedOffset[RESERVED_INDEX_CARTIMP_JNT_NULL_SPACE],*/
	};
	const size_t size = sizeof(offset)/sizeof(offset[0]);
	for (Configspace::Index i = getStateInfo().getJoints().begin(); i < getStateInfo().getJoints().end(); ++i)
		for (size_t j = 0; j < size; ++j)
			state.get<ConfigspaceCoord>(offset[j])[i] = p[0]*prev.get<ConfigspaceCoord>(offset[j])[i] + p[1]*next.get<ConfigspaceCoord>(offset[j])[i];

	// copy binary variables from next
	//Reserved area
	/*  getCurrentControlMode(state) = getCurrentControlMode(next);

      CONTROLLER_STATE_RESERVED(Torque, ConfigspaceCoord, RESERVED_INDEX_FORCE_TORQUE)
      CONTROLLER_STATE_RESERVED(NextControlMode, int, RESERVED_INDEX_NEXT_CONTROL_MODE)
      CONTROLLER_STATE_RESERVED(JntImpNextPos, ConfigspaceCoord, RESERVED_INDEX_JNTIMP_NEXT_POSITION)
      CONTROLLER_STATE_RESERVED(JntImpStiffness, ConfigspaceCoord, RESERVED_INDEX_STIFFNESS)
      CONTROLLER_STATE_RESERVED(JntImpDamping, ConfigspaceCoord, RESERVED_INDEX_DAMPING)
      CONTROLLER_STATE_RESERVED(JntImpAddTorque, ConfigspaceCoord, RESERVED_INDEX_JNTIMP_ADD_TORQUE)
      CONTROLLER_STATE_RESERVED(CartImpNextPos, golem::Mat34, RESERVED_INDEX_CARTIMP_NEXT_POSITION)
      CONTROLLER_STATE_RESERVED(CartImpStiffness, golem::Twist, RESERVED_INDEX_CARTIMP_STIFFNESS)
      CONTROLLER_STATE_RESERVED(CartImpDamping, golem::Twist, RESERVED_INDEX_CARTIMP_DAMPING)
      CONTROLLER_STATE_RESERVED(CartImpAddTcpFT, golem::Twist, RESERVED_INDEX_CARTIMP_ADD_TCP_FT)
      CONTROLLER_STATE_RESERVED(CartImpJntNullSpace, ConfigspaceCoord, RESERVED_INDEX_CARTIMP_JNT_NULL_SPACE)
      CONTROLLER_STATE_RESERVED(CurrentControlMode, int, RESERVED_INDEX_CURRENT_CONTROL_MODE)
      CONTROLLER_STATE_RESERVED(ComQuality, int, RESERVED_INDEX_COM_QUALITY)
      CONTROLLER_STATE_RESERVED(RobotState, int, RESERVED_INDEX_ROBOT_STATE)
      CONTROLLER_STATE_RESERVED(PowerIsOn, int, RESERVED_INDEX_ROBOT_POWER_IS_ON)*/

	/** Reset the reserved area */
	for (Configspace::Index i = getStateInfo().getJoints().begin(); i < getStateInfo().getJoints().end(); ++i)
	{
		/** Force/Torque*/
		getTorque(state)[i] = getTorque(next)[i];
		/** Joint stiffness 0=Gravity compensation mode min=0.01 max=2000 default=1000*/
		getJntImpStiffness(state)[i] = getJntImpStiffness(next)[i];
		/** Joint damping min=0.1 max=1.0 default=0.7*/
		getJntImpDamping(state)[i] = getJntImpDamping(next)[i];
		/** JntImp desired position*/
		getJntImpNextPos(state)[i] = getJntImpNextPos(next)[i];
		/** torque offset value*/
		getJntImpAddTorque(state)[i] = getJntImpAddTorque(next)[i];
		/** cartesian impedance mode - joint null space control*/
		getCartImpJntNullSpace(state)[i] = getCartImpJntNullSpace(next)[i];
	}

	/** Change the control mode 10-position control 20-Cartesian impedance control 30-Joint impedance control*/
	//get(next) = get(state);
	getNextControlMode(state) = getNextControlMode(next);
	/** cartesian impedance mode - desired position*/
	//state.get<golem::Mat34>(reservedOffset[RESERVED_INDEX_CARTIMP_NEXT_POSITION]).setToDefault();
	/** cartesian impedance mode - desired stiffness 0=Gravity compensation (Fmin=0.01 Tmin=0.01) (Fmax=5000 Tmax=300) (Fdefault=2000 Tdefault=200)*/
	//state.get<golem::Twist>(reservedOffset[RESERVED_INDEX_CARTIMP_STIFFNESS]).setZero();
	/** cartesian impedance mode - desired damping min=0.1 max=1.0 default=0.7*/
	//state.get<golem::Twist>(reservedOffset[RESERVED_INDEX_CARTIMP_DAMPING]).set(0.7,0.7,0.7,0.7,0.7,0.7);
	/** cartesian impedance mode - offset on the end effector force and torque*/
	//state.get<golem::Twist>(reservedOffset[RESERVED_INDEX_CARTIMP_ADD_TCP_FT]).setZero();
	/** current control mode*/
	getCurrentControlMode(state) = getCurrentControlMode(next);
	/** communication quality*/
	getComQuality(state) = getComQuality(next);
	/** Robot State FRI_STATE_INVALID =-1, FRI_STATE_OFF = 0, FRI_STATE_MON = 1, FRI_STATE_CMD = 2  */
	getRobotState(state) = getRobotState(next);
	/** Power state*/
	getPowerIsOn(state) = getPowerIsOn(next);

}

//------------------------------------------------------------------------------
