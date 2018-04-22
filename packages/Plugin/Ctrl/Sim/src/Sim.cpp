/** @file Sim.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/Sim/Sim.h>
#include <Golem/Sys/LoadObjectDesc.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Ctrl/SingleCtrl/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc) {
	loadObjectDesc<golem::Sim::Desc, Controller::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Controller::Desc::Ptr*)pControllerDesc);
}

//------------------------------------------------------------------------------

Sim::Sim(Context& context) : SingleCtrl(context) {
}

Sim::~Sim() {
	SingleCtrl::release();
}

void Sim::create(const Desc& desc) {
	SingleCtrl::create(desc); // throws

	coordMapSeq = desc.coordMapSeq;
}

//------------------------------------------------------------------------------

void Sim::lookupState(SecTmReal t, State &state) const {
	SingleCtrl::lookupState(t, state);

	// coord map
	CoordMap::apply(getStateInfo().getJoints(), coordMapSeq, getMin().cpos, getMax().cpos, state.cpos, state.cpos);
	CoordMap::apply(getStateInfo().getJoints(), coordMapSeq, getMin().cvel, getMax().cvel, state.cvel, state.cvel);
	CoordMap::apply(getStateInfo().getJoints(), coordMapSeq, getMin().cacc, getMax().cacc, state.cacc, state.cacc);
}

void Sim::setToDefault(State& state) const {
	SingleCtrl::setToDefault(state);
}

void Sim::clampConfig(GenConfigspaceCoord& config) const {
	SingleCtrl::clampConfig(config);

	// coord map
	CoordMap::apply(getStateInfo().getJoints(), coordMapSeq, getMin().cpos, getMax().cpos, config.cpos, config.cpos);
	CoordMap::apply(getStateInfo().getJoints(), coordMapSeq, getMin().cvel, getMax().cvel, config.cvel, config.cvel);
	CoordMap::apply(getStateInfo().getJoints(), coordMapSeq, getMin().cacc, getMax().cacc, config.cacc, config.cacc);
}

//------------------------------------------------------------------------------

void golem::XMLData(Joint::Desc::Ptr &val, XMLContext* context, bool create) {
	val.reset(new Joint::Desc());
	XMLData((Joint::Desc&)*val, context, create);
}

void golem::XMLData(Chain::Desc::Ptr &val, XMLContext* context, bool create) {
	val.reset(new Chain::Desc());
	XMLData((Chain::Desc&)*val, context, create);

	// create default joints
	XMLData(val->joints, golem::Configspace::DIM, context, "joint", create);
}

void golem::XMLData(Sim::Desc &val, XMLContext* context, bool create) {
	XMLData((SingleCtrl::Desc&)val, context, create);

	// create default chains
	XMLData(val.chains, golem::Chainspace::DIM, context, "chain", create);

	// coordinate map
	try {
		XMLData(val.coordMapSeq, val.coordMapSeq.max_size(), context, "coord_map", false);
	}
	catch (const MsgXMLParserNameNotFound&) {}
}

//------------------------------------------------------------------------------
