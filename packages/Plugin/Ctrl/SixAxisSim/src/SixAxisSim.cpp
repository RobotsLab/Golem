/** @file SixAxisSim.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/SixAxisSim/SixAxisSim.h>
#include <Golem/Ctrl/SixAxisSim/Data.h>
#include <Golem/Sys/LoadObjectDesc.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc) {
	loadObjectDesc<golem::SixAxisSim::Desc, Controller::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Controller::Desc::Ptr*)pControllerDesc);
}

//------------------------------------------------------------------------------

SixAxisChainSim::SixAxisChainSim(Controller& controller) : Chain(controller) {
}

void SixAxisChainSim::create(const Desc& desc) {
	(void)Chain::create(desc); // throws

	L0 = desc.L0;
	L1 = desc.L1;
	L2 = desc.L2;
	L3 = desc.L3;
}

//------------------------------------------------------------------------------

void SixAxisChainSim::chainForwardTransform(const Real* cc, Mat34& trn) const {
	if (!customKinematics) {
		Chain::chainForwardTransform(cc, trn);
		return;
	}

	Real s1, c1, s2, c2, s3, c3, s4, c4, s5, c5, s6, c6, s23, c23, a1, a2, a3, a4, a5;

	Math::sinCos(cc[0], s1, c1);
	Math::sinCos(cc[1], s2, c2);
	Math::sinCos(cc[2], s3, c3);
	Math::sinCos(cc[3], s4, c4);
	Math::sinCos(cc[4], s5, c5);
	Math::sinCos(cc[5], s6, c6);

	s23 = Math::sin(cc[1] + cc[2]); c23 = Math::cos(cc[1] + cc[2]);

	trn.p.v1 = -s1*(c2*L1 + c23*L2);
	trn.p.v2 = c1*(c2*L1 + c23*L2);
	trn.p.v3 = L0 - s2*L1 - s23*L2;

	a1 = s1*s23; a2 = s1*c23; a3 = c1*c4 - a2*s4; a4 = a2*c4 + c1*s4; a5 = a1*c5 + s5*a4;
	trn.R.m11 =  c6*a3 + s6*a5;
	trn.R.m12 = -c5*a4 + s5*a1;
	trn.R.m13 = -c6*a5 + s6*a3;

	a1 = c1*s23; a2 = c1*c23; a3 = c4*s1 + a2*s4; a4 = a2*c4 - s1*s4; a5 = a1*c5 + s5*a4;
	trn.R.m21 =  c6*a3 - s6*a5;
	trn.R.m22 =  c5*a4 - s5*a1;
	trn.R.m23 =  c6*a5 + s6*a3;

	a1 = c23*c5 - s23*c4*s5;
	trn.R.m31 = -s6*a1 - s23*s4*c6;
	trn.R.m32 = -s5*c23- s23*c4*c5;
	trn.R.m33 =  c6*a1 - s23*s4*s6;

	trn.multiply(getLocalPose(), trn);
}

void SixAxisChainSim::velocitySpatial(const Real* cc, const Real* dcc, Twist& vs) const {
	if (!customKinematics) {
		Chain::velocitySpatial(cc, dcc, vs);
		return;
	}

	Real s1, c1, s2, c2, s3, c3, s4, c4, s5, c5, s6, c6;

	Math::sinCos(cc[0], s1, c1);
	Math::sinCos(cc[1], s2, c2);
	Math::sinCos(cc[2], s3, c3);
	Math::sinCos(cc[3], s4, c4);
	Math::sinCos(cc[4], s5, c5);
	Math::sinCos(cc[5], s6, c6);

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
		-s1*s23, c1*s23, c23
	);
	tmp.getV().cross(q, tmp.getW());
	vs.multiplyAdd(dcc[3], tmp, vs);

	// twist #5
	tmp.getW().set(
		s1*s4*c23 - c1*c4, -s4*c1*c23 - s1*c4, s4*s23
	);
	tmp.getV().cross(q, tmp.getW());
	vs.multiplyAdd(dcc[4], tmp, vs);

	// twist #6
	tmp.getW().set(
		-s4*c1*c5 - s1*(c4*c5*c23 - s5*s23), -s1*s4*c5 + c1*(c4*c5*c23 - s5*s23), -(c4*c5*s23 + s5*c23)
	);
	tmp.getV().cross(q, tmp.getW());
	vs.multiplyAdd(dcc[5], tmp, vs);

	// transform to SE(3)
//	trn.twistToSpecial(velocity);
}

void SixAxisChainSim::jacobianSpatial(const Real* cc, Twist* jac) const {
	if (!customKinematics) {
		Chain::jacobianSpatial(cc, jac);
		return;
	}

	Real s1, c1, s2, c2, s3, c3, s4, c4, s5, c5, s6, c6;

	Math::sinCos(cc[0], s1, c1);
	Math::sinCos(cc[1], s2, c2);
	Math::sinCos(cc[2], s3, c3);
	Math::sinCos(cc[3], s4, c4);
	Math::sinCos(cc[4], s5, c5);
	Math::sinCos(cc[5], s6, c6);

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
		-s1*s23, c1*s23, c23
	);
	jac[3].getV().cross(q, jac[3].getW());

	// twist #5
	jac[4].getW().set(
		s1*s4*c23 - c1*c4, -s4*c1*c23 - s1*c4, s4*s23
	);
	jac[4].getV().cross(q, jac[4].getW());

	// twist #6
	jac[5].getW().set(
		-s4*c1*c5 - s1*(c4*c5*c23 - s5*s23), -s1*s4*c5 + c1*(c4*c5*c23 - s5*s23), -(c4*c5*s23 + s5*c23)
	);
	jac[5].getV().cross(q, jac[5].getW());
}

//------------------------------------------------------------------------------

SixAxisSim::SixAxisSim(Context& context) : SingleCtrl(context) {
}

SixAxisSim::~SixAxisSim() {
	SingleCtrl::release();
}

//------------------------------------------------------------------------------
