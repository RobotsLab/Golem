/** @file KukaKR5Sixx.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/Kuka/KukaKR5Sixx.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void KukaKR5SixxChain::ChainModel::create() {
	//<links L0="0.335" L10="0.075" L11="0.365" L20="0.09" L21="0.405" L3="0.08"/>

	//<trn th="0.0" v1="0.0" v2="0.0" v3="0.0" w1="0.0" w2="0.0" w3="-1.0"></trn>
	trnCoord[0].set(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO, -REAL_ONE);
	//<trn_init th="0.0" v1="0.0" v2="0.0" v3="0.0" w1="0.0" w2="0.0" w3="0.0"></trn_init>
	trnInitCoord[0].set(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO);

	//<trn th="0.0" v1="0.0" v2="-0.335" v3="0.075" w1="-1.0" w2="0.0" w3="0.0"></trn>
	trnCoord[1].set(REAL_ZERO, REAL_ZERO, -L0, L10, -REAL_ONE, REAL_ZERO, REAL_ZERO);
	//<trn_init th="1.0" v1="0.0" v2="0.075" v3="0.335" w1="0.0" w2="0.0" w3="0.0"></trn_init>
	trnInitCoord[1].set(REAL_ONE, REAL_ZERO, L10, L0, REAL_ZERO, REAL_ZERO, REAL_ZERO);

	//<trn th="0.0" v1="0.0" v2="-0.335" v3="0.44" w1="-1.0" w2="0.0" w3="0.0"></trn>
	trnCoord[2].set(REAL_ZERO, REAL_ZERO, -L0, L10 + L11, -REAL_ONE, REAL_ZERO, REAL_ZERO);
	//<trn_init th="1.0" v1="0.0" v2="0.44" v3="0.335" w1="0.0" w2="0.0" w3="0.0"></trn_init>
	trnInitCoord[2].set(REAL_ONE, REAL_ZERO, L10 + L11, L0, REAL_ZERO, REAL_ZERO, REAL_ZERO);
	
	//<trn th="0.0" v1="0.425" v2="0.0" v3="0.0" w1="0.0" w2="-1.0" w3="0.0"></trn>
	trnCoord[3].set(REAL_ZERO, L0 + L20, REAL_ZERO, REAL_ZERO, REAL_ZERO, -REAL_ONE, REAL_ZERO);
	//<trn_init th="1.0" v1="0.0" v2="0.845" v3="0.425" w1="0.0" w2="0.0" w3="0.0"></trn_init>
	trnInitCoord[3].set(REAL_ONE, REAL_ZERO, L10 + L11 + L21, L0 + L20, REAL_ZERO, REAL_ZERO, REAL_ZERO);
	
	//<trn th="0.0" v1="0.0" v2="-0.425" v3="0.845" w1="-1.0" w2="0.0" w3="0.0"></trn>
	trnCoord[4].set(REAL_ZERO, REAL_ZERO, -(L0 + L20), L10 + L11 + L21, -REAL_ONE, REAL_ZERO, REAL_ZERO);
	//<trn_init th="1.0" v1="0.0" v2="0.845" v3="0.425" w1="0.0" w2="0.0" w3="0.0"></trn_init>
	trnInitCoord[4].set(REAL_ONE, REAL_ZERO, L10 + L11 + L21, L0 + L20, REAL_ZERO, REAL_ZERO, REAL_ZERO);
	
	//<trn th="0.0" v1="0.425" v2="0.0" v3="0.0" w1="0.0" w2="-1.0" w3="0.0"></trn>
	trnCoord[5].set(REAL_ZERO, L0 + L20, REAL_ZERO, REAL_ZERO, REAL_ZERO, -REAL_ONE, REAL_ZERO);
	//<trn_init th="1.0" v1="0.0" v2="0.845" v3="0.425" w1="0.0" w2="0.0" w3="0.0"></trn_init>
	trnInitCoord[5].set(REAL_ONE, REAL_ZERO, L10 + L11 + L21, L0 + L20, REAL_ZERO, REAL_ZERO, REAL_ZERO);
}

// Kuka custom froward kinematics
void KukaKR5SixxChain::ChainModel::chainForwardTransform(const Mat34& localPose, const Real* cc, Mat34& trn) const {
	Mat34 tmp;

	trn.fromTwist(trnInitCoord[NUM_JOINTS - 1].twist, trnInitCoord[NUM_JOINTS - 1].theta);

	for (I32 i = NUM_JOINTS - 1; i >= 0; --i) {
		tmp.fromTwist(trnCoord[i].twist, cc[i] + encoderOffset[i]);
		trn.multiply(tmp, trn);
	}

	trn.multiply(localPose, trn);
}

void KukaKR5SixxChain::ChainModel::jointForwardTransform(const Mat34& localPose, const Real* cc, Mat34* trn) const {
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

void KukaKR5SixxChain::ChainModel::jacobianSpatial(const Real* cc, Twist* jac) const {
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

KukaKR5SixxChain::KukaKR5SixxChain(Controller& controller) : Chain(controller) {
}

void KukaKR5SixxChain::create(const Desc& desc) {
	Chain::create(desc); // throws
	setChainModel(desc.chainModel);
}

void KukaKR5SixxChain::setChainModel(const ChainModel& chainModel) {
	if (!chainModel.isValid())
		throw MsgChainInvalidDesc(Message::LEVEL_CRIT, "KukaKR5SixxChain::setChainModel(): Invalid chain model");
	this->chainModel = chainModel;
	this->chainModel.create();
}

//------------------------------------------------------------------------------

// Kuka froward kinematics
void KukaKR5SixxChain::chainForwardTransform(const Real* cc, Mat34& trn) const {
	if (customKinematics)
		chainModel.chainForwardTransform(getLocalPose(), cc, trn);
	else
		Chain::chainForwardTransform(cc, trn);
}

void KukaKR5SixxChain::jointForwardTransform(const Real* cc, Mat34* trn) const {
	if (customKinematics)
		chainModel.jointForwardTransform(getLocalPose(), cc, trn);
	else
		Chain::jointForwardTransform(cc, trn);
}

void KukaKR5SixxChain::jacobianSpatial(const Real* cc, Twist* jac) const {
	if (customKinematics)
		chainModel.jacobianSpatial(cc, jac);
	else
		Chain::jacobianSpatial(cc, jac);
}

//------------------------------------------------------------------------------

KukaKR5Sixx::KukaKR5Sixx(golem::Context& context) : SingleCtrl(context) {
}

//------------------------------------------------------------------------------

void KukaKR5Sixx::interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const {
	// interpolate config
	SingleCtrl::interpolateConfig(prev, next, t, state);

	// TODO reserved data interpolation (e.g. Cartesian coordinates)
}

//------------------------------------------------------------------------------
