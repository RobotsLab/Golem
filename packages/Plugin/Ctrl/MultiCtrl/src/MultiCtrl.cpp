/** @file MultiCtrl.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/MultiCtrl/MultiCtrl.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void MultiCtrl::ControllerDesc::create(Context& context, Seq& controllerDescSeq, Controller::Seq& controllerSeq, Controller::Desc& desc) {
	Chainspace::Index chainBegin(desc.chainBegin);
	Configspace::Index jointBegin(desc.jointBegin);
	Reservedspace::Index reservedBegin(desc.reservedBegin);
	desc.reservedSize = 0; // reservedSize must be computed 
	const idx_t chainInit = *chainBegin;

	for (idx_t i = 0; i < (idx_t)controllerDescSeq.size(); ++i) {
		ControllerDesc* pControllerDesc = &controllerDescSeq[i];

		pControllerDesc->pControllerDesc->chainBegin = (U32)*chainBegin; // overwrite
		pControllerDesc->pControllerDesc->jointBegin = (U32)*jointBegin; // overwrite
		pControllerDesc->pControllerDesc->reservedBegin = (U32)*reservedBegin; // overwrite
		pControllerDesc->pControllerDesc->globalPose = pControllerDesc->localPose; // overwrite
		pControllerDesc->pControllerDesc->enableIO = desc.enableIO;// overwrite
		
		const Controller::Ptr pController = pControllerDesc->pControllerDesc->create(context); // throws
		controllerSeq.push_back(pController);
		
		desc.chains.insert(desc.chains.end(), pControllerDesc->pControllerDesc->chains.begin(), pControllerDesc->pControllerDesc->chains.end());
		
		for (idx_t j = 0; j < (idx_t)pControllerDesc->pControllerDesc->chains.size(); ++j) {
			const idx_t k = *chainBegin + j; // absolute index
			const idx_t l = k - chainInit; // index counted from 0
			Chain::Desc* pChainDesc = desc.chains[l].get();

			if ((U32)pChainDesc->linkedChainIndex >= (U32)j)
				pChainDesc->localPose.multiply(pControllerDesc->localPose, pChainDesc->localPose);
			if ((U32)pControllerDesc->linkedChainIndex < (U32)l || (U32)pChainDesc->linkedChainIndex < (U32)j)
				pChainDesc->linkedChainIndex = (U32)pChainDesc->linkedChainIndex < (U32)j ? (U32)(*chainBegin - chainInit + pChainDesc->linkedChainIndex) : pControllerDesc->linkedChainIndex;
		}

		chainBegin += pController->getStateInfo().getChains().size();
		jointBegin += pController->getStateInfo().getJoints().size();
		reservedBegin += pController->getStateInfo().getReserved().size();
		
		desc.reservedSize += (U32)pController->getStateInfo().getReserved().size();
	}
}

MultiCtrl::MultiCtrl(golem::Context& context) : Controller(context) {
}

MultiCtrl::~MultiCtrl() {
	release();
}

void MultiCtrl::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgControllerInvalidDesc(Message::LEVEL_CRIT, "MultiCtrl::create(): Invalid description");

	// TODO clone description instead of casting
	Desc* pDesc = const_cast<Desc*>(&desc);
	pDesc->chains.clear();
	controllerSeq.clear();
	// create slave controllers
	ControllerDesc::create(context, pDesc->controllers, controllerSeq, *pDesc);

	(void)Controller::create(*pDesc); // chains and joints are added here

	// Copy pointers and properties
	controllers.clear();
	properties.clear();
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i) {
		controllers.push_back(i->get());
		properties.push_back(pDesc->controllers[i - controllerSeq.begin()]);
	}
}

void MultiCtrl::release() {
	while (!controllerSeq.empty()) {
		controllers.pop_back();
		controllerSeq.pop_back(); // throws
	}
}

//------------------------------------------------------------------------------

const Controller::State* MultiCtrl::send(const State* begin, const State* end, bool clear, bool limits, MSecTmU32 timeWait) {
	SysTimer timer;
	const State* ptr = begin;
	SecTmReal shift = SEC_TM_REAL_ZERO;
	Controller* j = NULL;

	while (ptr != end) {
		const bool init = ptr == begin;

		if (j != NULL) {
			const MSecTmU32 elapsed = timer.elapsed();
			if (!j->waitForBegin(timeWait < elapsed ? MSEC_TM_U32_ZERO : timeWait - elapsed))
				return ptr;
			j = NULL;
		}
		
		Guard<Controller::Seq> guard(controllerSeq);

		for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i) {
			if (init)
				shift = std::max((*i)->getCommandTime() + (*i)->getCycleDuration() - ptr->t, shift);
			if ((*i)->getCommandCapacity() < 1) {
				j = i->get();
				continue;
			}
		}

		for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i) {
			State state = (*i)->createState();
			state = *ptr;
			state.t += shift;
			(void)(*i)->send(&state, &state + 1, init && clear, limits, 0);
		}

		++ptr;
	}

	// inform the associated interface
	getCallbackDataSync()->syncSend(&*begin);

	return ptr;
}

void MultiCtrl::stop() {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->stop();
}

void MultiCtrl::resume() {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->resume();
}

bool MultiCtrl::waitForBegin(MSecTmU32 timeWait) {
	SysTimer timer;
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		if (properties[i - controllerSeq.begin()].syncBegin) {
			const MSecTmU32 elapsed = timeWait > MSEC_TM_U32_ZERO ? timer.elapsed() : MSEC_TM_U32_ZERO;
			if (!(*i)->waitForBegin(timeWait < elapsed ? MSEC_TM_U32_ZERO : timeWait - elapsed))
				return false;
		}
	return true;
}

bool MultiCtrl::waitForEnd(MSecTmU32 timeWait) {
	SysTimer timer;
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		if (properties[i - controllerSeq.begin()].syncEnd) {
			const MSecTmU32 elapsed = timeWait > MSEC_TM_U32_ZERO ? timer.elapsed() : MSEC_TM_U32_ZERO;
			if (!(*i)->waitForEnd(timeWait < elapsed ? MSEC_TM_U32_ZERO : timeWait - elapsed))
				return false;
		}
	return true;
}

void MultiCtrl::lookupState(SecTmReal t, State &state) const {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->lookupState(t, state);
}

void MultiCtrl::lookupCommand(SecTmReal t, State &command) const {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->lookupCommand(t, command);
}

//------------------------------------------------------------------------------

SecTmReal MultiCtrl::getCycleDuration() const {
	SecTmReal cycleDuration = SEC_TM_REAL_ZERO;
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		cycleDuration = std::max(cycleDuration, (*i)->getCycleDuration());
	return cycleDuration;
}

SecTmReal MultiCtrl::getCommandLatency() const {
	SecTmReal commandLatency = SEC_TM_REAL_ZERO;
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		commandLatency = std::max(commandLatency, (*i)->getCommandLatency());
	return commandLatency;
}

SecTmReal MultiCtrl::getCommandTime() const {
	SecTmReal commandTime = SEC_TM_REAL_MIN;
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		commandTime = std::max(commandTime, (*i)->getCommandTime());
	return commandTime;
}

U32 MultiCtrl::getCommandCapacity() const {
	U32 commandCapacity = numeric_const<U32>::MAX;
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		commandCapacity = std::min(commandCapacity, (*i)->getCommandCapacity());
	return commandCapacity;
}

void MultiCtrl::setToDefault(State& state) const {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->setToDefault(state);
}

void MultiCtrl::clampConfig(GenConfigspaceCoord& config) const {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->clampConfig(config);
}

void MultiCtrl::clampState(State& state) const {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->clampState(state);
}

void MultiCtrl::interpolateConfig(const GenConfigspaceState& prev, const GenConfigspaceState& next, SecTmReal t, GenConfigspaceState& state) const {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->interpolateConfig(prev, next, t, state);
}

void MultiCtrl::interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->interpolateState(prev, next, t, state);
}

void MultiCtrl::assertLimits(const State& prev, const State& next) const {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->assertLimits(prev, next);
}

//------------------------------------------------------------------------------

void golem::XMLData(MultiCtrl::ControllerProperty &val, XMLContext* xmlcontext, bool create) {
	XMLData("sync_begin", val.syncBegin, xmlcontext, create);
	XMLData("sync_end", val.syncEnd, xmlcontext, create);
}

void golem::XMLData(MultiCtrl::ControllerDesc &val, Context* context, XMLContext* xmlcontext, bool create) {
	XMLData(val.localPose, xmlcontext->getContextFirst("local_pose"), create);
	XMLData("linked_chain_index", val.linkedChainIndex, xmlcontext, create);
	golem::XMLData((MultiCtrl::ControllerProperty&)val, xmlcontext, create);
}

void golem::XMLData(MultiCtrl::Desc &val, Context* context, XMLContext* xmlcontext, bool create) {
	XMLData("name", val.name, xmlcontext, create);
	XMLData(val.globalPose, xmlcontext->getContextFirst("global_pose"), create);

	const std::string name = "controller";
	std::pair<XMLContext::XMLContextMap::const_iterator, XMLContext::XMLContextMap::const_iterator> range = xmlcontext->getContextMap().equal_range(name);
	if (range.first == range.second)
		throw MsgXMLParserNameNotFound(Message::LEVEL_ERROR, "XMLData(): Name not found: %s %s", xmlcontext->getName().c_str(), name.c_str());

	for (XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second; ++i) {
		MultiCtrl::ControllerDesc desc;
		desc.pControllerDesc = Controller::Desc::load(context, const_cast<XMLContext*>(&i->second));
		XMLData(desc, context, const_cast<XMLContext*>(&i->second), (bool)false);
		val.controllers.insert(val.controllers.end(), desc);
	}
}

//------------------------------------------------------------------------------
