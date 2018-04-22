/** @file Data.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/Data.h>
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(std::vector<Real> &seq, XMLContext* context, bool create) {
	golem::XMLDataSeq(seq, "c", context, create);
}

//------------------------------------------------------------------------------

void golem::XMLData(GenCoord &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData("pos", val.pos, context, create);
	XMLData("vel", val.vel, context, create);
	XMLData("acc", val.acc, context, create);
}

void golem::XMLData(ExpCoord &val, XMLContext* context, bool create) {
	// axis data
	try {
		if (val.axis || !create) {
			XMLData("l1", val.twist.v.v1, context, create);
			XMLData("l2", val.twist.v.v2, context, create);
			XMLData("l3", val.twist.v.v3, context, create);
			XMLData("n1", val.twist.w.v1, context, create);
			XMLData("n2", val.twist.w.v2, context, create);
			XMLData("n3", val.twist.w.v3, context, create);
			val.axis = true;
		}
	}
	catch (const MsgXMLParser&) {
		val.axis = false;
	}
	// no axis data
	if (!val.axis)
		XMLData(val.twist, context, create);

	// optional
	try {
		XMLData("th", val.theta, context, create);
	}
	catch (const MsgXMLParser& ex) {
		if (create)
			throw ex;
		else
			val.theta = numeric_const<Real>::ZERO;
	}
}

void golem::XMLData(ConfigspaceCoord &val, const char* attr, XMLContext* context, bool create) {
	for (U32 i = 0; i < (U32)Configspace::DIM; ++i) {
		char str[16];
		sprintf(str, "%s%u", attr, U32(i + 1));

		ConfigspaceCoord::Type& c = val.data()[i];
		try {
			XMLData(str, c, context, create);
		}
		catch (const Message& msg) {
			if (create)
				throw msg;
		}
	}
}

//------------------------------------------------------------------------------

void golem::XMLData(Joint::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData("name", val.name, context, create);
	
	XMLData(val.min, context->getContextFirst("min"), create);
	XMLData(val.max, context->getContextFirst("max"), create);
	XMLData(val.offset, context->getContextFirst("offset"), create);

	if (create) {
		XMLData(val.trnSeq.begin(), val.trnSeq.end(), context, "trn", true);
	}
	else {
		val.trnInit.setId();
		val.trnSeq.clear();
		XMLData(val.trnSeq, val.trnSeq.max_size(), context, "trn", false);

		for (size_t i = 0; i < val.trnSeq.size(); ++i) {
			ExpCoord& trn = val.trnSeq[i];
			// if axis & position available, not for prismatic joints
			if (trn.axis && trn.twist.v.magnitude() > REAL_EPS && trn.twist.w.magnitude() > REAL_EPS) {
				const Vec3 l = trn.twist.v, n = trn.twist.w;
				trn.twist.v.cross(l, n);
				val.trnInit.p = l;
			}
		}
	}

	try {
		XMLData(val.trnInit, context->getContextFirst("trn_init"), create);
	}
	catch (const MsgXMLParser& msg) {
		if (create) throw msg;
	}

	try {
		XMLData(val.bounds, val.bounds.max_size(), context, "bounds", create);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}
}

void golem::XMLData(Chain::Desc &val, XMLContext* context, bool create) {
	XMLData("name", val.name, context, create);
	XMLDataPtr<Joint::Desc>(val.joints.begin(), val.joints.end(), context, "joint", create);
	XMLData("linked_chain_index", val.linkedChainIndex, context, create);
	XMLData(val.localPose, context->getContextFirst("local_pose"), create);
	XMLData(val.referencePose, context->getContextFirst("reference_pose"), create);
	XMLData("custom_kinematics", val.customKinematics, context, create);
	try {
		XMLData(val.bounds, val.bounds.max_size(), context, "bounds", create);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}
}

void golem::XMLData(Controller::Desc &val, XMLContext* context, bool create) {
	XMLData("name", val.name, context, create);
	XMLDataPtr<Chain::Desc>(val.chains.begin(), val.chains.end(), context, "chain", create);
	XMLData(val.globalPose, context->getContextFirst("global_pose"), create);
	XMLData("enable_io", val.enableIO, context, create);
}

//------------------------------------------------------------------------------
