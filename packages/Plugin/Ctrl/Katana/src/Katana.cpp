/** @file Katana.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/Katana/Katana.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const SecTmReal KatanaJoint::TRJ_TIME_QUANT = (SecTmReal)0.01;

//------------------------------------------------------------------------------

KatanaJoint::KatanaJoint(Chain& chain) : Joint(chain) {
}
	
void KatanaJoint::create(const Desc& desc) {
	Joint::create(desc); // throws

	angleOffset = desc.angleOffset*REAL_2_PI/360.0;
	angleRange = desc.angleRange*REAL_2_PI/360.0;
	encoderOffset = desc.encoderOffset;
	encodersPerCycle = desc.encodersPerCycle;
	rotationDirection = desc.rotationDirection;
	encoderPositionAfter = desc.encoderPositionAfter;
	offset = desc.offset;
	gain = desc.gain;

	kniMinPos = offset + gain*angleOffset;
	kniMaxPos = offset + gain*(angleOffset + angleRange);
	if (kniMinPos > kniMaxPos)
		std::swap(kniMinPos, kniMaxPos);

	// overwrite min, max
	min.pos = this->getKNIMinPos();
	max.pos = this->getKNIMaxPos();
}

Real KatanaJoint::getKNIMinPos() const {
	return kniMinPos;
}

Real KatanaJoint::getKNIMaxPos() const {
	return kniMaxPos;
}

Real KatanaJoint::posFromEnc(I32 pos) const {
	return offset + gain*(angleOffset - REAL_2_PI*(pos - encoderOffset)/Real(encodersPerCycle*rotationDirection));
}

I32 KatanaJoint::posToEnc(Real pos) const {
	return (I32)Math::round(Real(encoderOffset) + (angleOffset - (pos - offset)/gain)*Real(encodersPerCycle*rotationDirection)/REAL_2_PI);
}

Real KatanaJoint::velFromEnc(I32 vel) const {
	return -REAL_2_PI*gain*vel/Real(encodersPerCycle*rotationDirection);
}

I32 KatanaJoint::velToEnc(Real vel) const {
	return (I32)Math::round(-vel*Real(encodersPerCycle*rotationDirection)/(REAL_2_PI*gain));
}

//------------------------------------------------------------------------------

KatanaChain::KatanaChain(Controller& controller) : Chain(controller) {
}

void KatanaChain::create(const Desc& desc) {
	Chain::create(desc); // throws

	L0 = desc.L0;
	L1 = desc.L1;
	L2 = desc.L2;
	L3 = desc.L3;
}

//------------------------------------------------------------------------------

// Katana froward kinematics
void KatanaChain::chainForwardTransform(const Real* cc, Mat34& trn) const {
	if (!customKinematics) {
		Chain::chainForwardTransform(cc, trn);
		return;
	}

	Real s1, c1, s2, c2, s3, c3, s4, c4, s5, c5, s23, c23, a1, a2, a3;

	Math::sinCos(cc[0], s1, c1);
	Math::sinCos(cc[1], s2, c2);
	Math::sinCos(cc[2], s3, c3);
	Math::sinCos(cc[3], s4, c4);
	Math::sinCos(cc[4], s5, c5);

	s23 = Math::sin(cc[1] + cc[2]); c23 = Math::cos(cc[1] + cc[2]);

	trn.p.v1 = -s1*(c2*L1 + c23*L2);
	trn.p.v2 = c1*(c2*L1 + c23*L2);
	trn.p.v3 = L0 - s2*L1 - s23*L2;

	a1 = s1*s23; a2 = s1*c23; a3 = a1*c4 + s4*a2;
	trn.R.m11 =  c5*c1 + s5*a3;
	trn.R.m12 = -c4*a2 + s4*a1;
	trn.R.m13 = -c5*a3 + s5*c1;

	a1 = c1*s23; a2 = c1*c23; a3 = a1*c4 + s4*a2;
	trn.R.m21 =  c5*s1 - s5*a3;
	trn.R.m22 =  c4*a2 - s4*a1;
	trn.R.m23 =  c5*a3 + s5*s1;

	a1 = c23*c4 - s23*s4;
	trn.R.m31 = -s5*a1;
	trn.R.m32 = -s4*c23 - s23*c4;
	trn.R.m33 =  c5*a1;
}

void KatanaChain::velocitySpatial(const Real* cc, const Real* dcc, Twist& vs) const {
	if (!customKinematics) {
		Chain::velocitySpatial(cc, dcc, vs);
		return;
	}

	Real s1, c1, s2, c2, s3, c3, s4, c4, s5, c5;

	Math::sinCos(cc[0], s1, c1);
	Math::sinCos(cc[1], s2, c2);
	Math::sinCos(cc[2], s3, c3);
	Math::sinCos(cc[3], s4, c4);
	Math::sinCos(cc[4], s5, c5);

	Real s23 = Math::sin(cc[1] + cc[2]);
	Real c23 = Math::cos(cc[1] + cc[2]);
	
	Vec3 q(
		-s1*(L1*c2 + L2*c23),
		c1*(L1*c2 + L2*c23),
		L0 - (L1*s2 + L2*s23)
	);

	Twist tmp;

	// twist #1
	vs.getV().set(
		REAL_ZERO, REAL_ZERO, REAL_ZERO
	);
	vs.getW().set(
		REAL_ZERO, REAL_ZERO, dcc[0]
	);

	// twist #2
	tmp.getV().set(
		L0*s1, -L0*c1, REAL_ZERO
	);
	tmp.getW().set(
		-c1, -s1, REAL_ZERO
	);
	vs.multiplyAdd(dcc[1], tmp, vs);

	// twist #3
	tmp.getV().set(
		s1*(L0 - L1*s2), c1*(L1*s2 - L0), L1*c2
	);
	tmp.getW().set(
		-c1, -s1, REAL_ZERO
	);
	vs.multiplyAdd(dcc[2], tmp, vs);

	// twist #4
	tmp.getW().set(
		-c1, -s1, REAL_ZERO
	);
	tmp.getV().cross(q, tmp.getW());
	vs.multiplyAdd(dcc[3], tmp, vs);

	// twist #5
	tmp.getW().set(
		-s1*(c4*c23 - s4*s23), c1*(c4*c23 - s4*s23), -(c4*s23 + s4*c23)
	);
	tmp.getV().cross(q, tmp.getW());
	vs.multiplyAdd(dcc[4], tmp, vs);

	// transform to SE(3)
	//trn.twistToSpecial(velocity);
}

void KatanaChain::jacobianSpatial(const Real* cc, Twist* jac) const {
	if (!customKinematics) {
		Chain::jacobianSpatial(cc, jac);
		return;
	}

	Real s1, c1, s2, c2, s3, c3, s4, c4, s5, c5;

	Math::sinCos(cc[0], s1, c1);
	Math::sinCos(cc[1], s2, c2);
	Math::sinCos(cc[2], s3, c3);
	Math::sinCos(cc[3], s4, c4);
	Math::sinCos(cc[4], s5, c5);

	Real s23 = Math::sin(cc[1] + cc[2]);
	Real c23 = Math::cos(cc[1] + cc[2]);
	
	Vec3 q(
		-s1*(L1*c2 + L2*c23),
		c1*(L1*c2 + L2*c23),
		L0 - (L1*s2 + L2*s23)
	);

	// twist #1
	jac[0].getV().set(
		REAL_ZERO, REAL_ZERO, REAL_ZERO
	);
	jac[0].getW().set(
		REAL_ZERO, REAL_ZERO, REAL_ONE
	);

	// twist #2
	jac[1].getV().set(
		L0*s1, -L0*c1, REAL_ZERO
	);
	jac[1].getW().set(
		-c1, -s1, REAL_ZERO
	);

	// twist #3
	jac[2].getV().set(
		s1*(L0 - L1*s2), c1*(L1*s2 - L0), L1*c2
	);
	jac[2].getW().set(
		-c1, -s1, REAL_ZERO
	);

	// twist #4
	jac[3].getW().set(
		-c1, -s1, REAL_ZERO
	);
	jac[3].getV().cross(q, jac[3].getW());

	// twist #5
	jac[4].getW().set(
		-s1*(c4*c23 - s4*s23), c1*(c4*c23 - s4*s23), -(c4*s23 + s4*c23)
	);
	jac[4].getV().cross(q, jac[4].getW());
}

//------------------------------------------------------------------------------

KatanaGripper::KatanaGripper(Context* context) : pContext(context) {
}

KatanaGripper::~KatanaGripper() {
}

void KatanaGripper::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgControllerInvalidDesc(Message::LEVEL_CRIT, "KatanaGripper::create(): Invalid description");

	bGripper = desc.bGripper;
	sensorIndexSet = desc.sensorIndexSet;
}

//------------------------------------------------------------------------------
