/** @file CtrlClient.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/CtrlClient/CtrlClient.h>
#include <Golem/Sys/LoadObjectDesc.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Ctrl/SingleCtrl/Data.h>
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc) {
	//loadObjectDesc<golem::CtrlClient::Desc, Controller::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Controller::Desc::Ptr*)pControllerDesc);

	// initialise context in a module
	((Context*)pContext)->initModule();
	
	// Create description and load xml configuration
	CtrlClient::Desc* pDesc = new CtrlClient::Desc;
	((golem::shared_ptr<Controller::Desc>*)pControllerDesc)->reset(pDesc);
	golem::XMLData(*pDesc, (Context*)pContext, (XMLContext*)pXMLContext);
}

//------------------------------------------------------------------------------

CtrlClient::CallbackIO::CallbackIO(CtrlClient& controller, const std::string& host, unsigned short port, SecTmReal timeOut, U32 lostPackages) : controller(controller), context(controller.getContext()), timeOut(timeOut), lostPackages(lostPackages), inp(controller.createState()), out(controller.createState()), timer(&context.getTimer()), msgstr(*context.getMessageStream()) {
	// create client
	client.reset(new SMClient(host, port, timer, &msgstr));
	client->syncTimers();
	client->start();
	id = 0;
	// Info
	infoCli = controller.getStateInfo();
	for (Chainspace::Index chain = infoCli.getChains().begin(); chain < infoCli.getChains().end(); ++chain)
		infoSrv.addJoints(infoCli.getJoints(chain).size());
	infoSrv.addReserved(infoCli.getReserved().size());
}

void CtrlClient::CallbackIO::sysSync(SingleCtrl*) {
	SM::Header header(sizeof(Controller::State), id);
	if (!client->read(header, &inp, boost::posix_time::milliseconds(SecToMSec(timeOut)))) {
		context.error("CtrlClient::CallbackIO::sysSync(): timeout\n");
		return;
	}
	if (infoSrv != inp.getInfo()) {
		context.error("CtrlClient::CallbackIO::sysSync(): invaid state type: joints=[%u, %u), chains=[%u, %u), reserved=[%u, %u)\n", *infoSrv.getJoints().begin(), *infoSrv.getJoints().end(), *infoSrv.getChains().begin(), *infoSrv.getChains().end(), *infoSrv.getReserved().begin(), *infoSrv.getReserved().end());
		return;
	}

	inp.t = (SecTmReal)client->toClientTime((double)inp.t);

	if (id > 0 && id + lostPackages < header.id)
		context.warning("CtrlClient::CallbackIO::sysSync(): %i data packages have been lost\n", (header.id - id - 1));
	id = header.id;
}

void CtrlClient::CallbackIO::sysRecv(SingleCtrl*, State& state) {
	// assume that remote controller data has offset = 0
	state.cpos.set(inp.cpos.data(), inp.cpos.data() + infoCli.getJoints().size(), *infoCli.getJoints().begin());
	state.cvel.set(inp.cvel.data(), inp.cvel.data() + infoCli.getJoints().size(), *infoCli.getJoints().begin());
	state.cacc.set(inp.cacc.data(), inp.cacc.data() + infoCli.getJoints().size(), *infoCli.getJoints().begin());
	state.reserved.set(inp.reserved.data(), inp.reserved.data() + infoCli.getReserved().size(), *infoCli.getReserved().begin());
	state.t = inp.t;
	//state = inp;
};

void CtrlClient::CallbackIO::sysSend(SingleCtrl*, const State&, State&, bool, bool) {
	CriticalSectionWrapper csw(controller.cs);
	
	if (!controller.qSMCommand->empty()) {
		controller.qSMCommand->front().state.t = client->toServerTime(controller.qSMCommand->front().state.t);
		
		// assume that remote controller data has offset = 0
		const Controller::State& state = controller.qSMCommand->front().state;
		out.state.cpos.set(state.cpos.data() + *infoCli.getJoints().begin(), state.cpos.data() + *infoCli.getJoints().end(), 0);
		out.state.cvel.set(state.cvel.data() + *infoCli.getJoints().begin(), state.cvel.data() + *infoCli.getJoints().end(), 0);
		out.state.cacc.set(state.cacc.data() + *infoCli.getJoints().begin(), state.cacc.data() + *infoCli.getJoints().end(), 0);
		out.state.reserved.set(state.reserved.data() + *infoCli.getReserved().begin(), state.reserved.data() + *infoCli.getReserved().end(), 0);
		out.state.t = state.t;
		out.state.setInfo(infoSrv);

		out.clear = controller.qSMCommand->front().clear;
		out.limits = controller.qSMCommand->front().limits;

		client->write(sizeof(SMCommand), &out);
		controller.qSMCommand->pop_front();
	}
}

//------------------------------------------------------------------------------

CtrlClient::CtrlClient(Context& context) : SingleCtrl(context) {
}

CtrlClient::~CtrlClient() {
	SingleCtrl::release();
}

void CtrlClient::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgControllerInvalidDesc(Message::LEVEL_CRIT, "CtrlClient::create(): Invalid description");

	qCommandSize = desc.qCommandSize;

	// TODO clone description instead of casting
	Desc* pDesc = const_cast<Desc*>(&desc);

	// create slave controllers
	pDesc->chains.clear();
	pDesc->enableIO = false;
	controllerSeq.clear();
	MultiCtrl::ControllerDesc::create(context, pDesc->controllers, controllerSeq, *pDesc);

	pDesc->enableIO = true;
	(void)SingleCtrl::create(*pDesc); // chains and joints are added here

	// create controller I/O callback, AFTER the controller state info is created
	callbackIO.reset(new CallbackIO(*this, desc.host, desc.port, desc.timeOut, desc.lostPackages));
	setCallbackIO(callbackIO.get());
	initControlCycle();

	// Copy pointers and properties
	controllers.clear();
	properties.clear();
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i) {
		controllers.push_back(i->get());
		properties.push_back(pDesc->controllers[i - controllerSeq.begin()]);
	}
}

void CtrlClient::userCreate() {
	// command queue
	qSMCommand.reset(new queue<SMCommand>(qCommandSize, createState()));
}

void CtrlClient::release() {
	while (!controllerSeq.empty()) {
		controllers.pop_back();
		controllerSeq.pop_back(); // throws
	}
}

const CtrlClient::State* CtrlClient::send(const State* begin, const State* end, bool clear, bool limits, MSecTmU32 timeWait) {
	{
		CriticalSectionWrapper csw(cs);
		
		for (const State* ptr = begin; ptr != end; ++ptr)
			if (qSMCommand->full()) {
				context.error("CtrlClient::send(): SMCommand queue full\n");
				break;
			}
			else
				qSMCommand->push_back(SMCommand(*ptr, clear && ptr == begin, limits));
	}

	return SingleCtrl::send(begin, end, clear, limits, timeWait);
}

void CtrlClient::setToDefault(State& state) const {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->setToDefault(state);
}

void CtrlClient::clampConfig(GenConfigspaceCoord& config) const {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->clampConfig(config);
}

void CtrlClient::clampState(State& state) const {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->clampState(state);
}

void CtrlClient::interpolateConfig(const GenConfigspaceState& prev, const GenConfigspaceState& next, SecTmReal t, GenConfigspaceState& state) const {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->interpolateConfig(prev, next, t, state);
}

void CtrlClient::interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->interpolateState(prev, next, t, state);
}

void CtrlClient::assertLimits(const State& prev, const State& next) const {
	for (Controller::Seq::const_iterator i = controllerSeq.begin(); i != controllerSeq.end(); ++i)
		(*i)->assertLimits(prev, next);
}

//------------------------------------------------------------------------------

void golem::XMLData(CtrlClient::Desc &val, Context* context, XMLContext* xmlcontext, bool create) {
	XMLData("name", val.name, xmlcontext, create);

	try {
		golem::XMLDataIO(val, xmlcontext->getContextFirst("io"), create);
		XMLData(val.globalPose, xmlcontext->getContextFirst("global_pose"), create);
	}
	catch (const MsgXMLParser&) {
	}

	XMLData("host", val.host, xmlcontext->getContextFirst("client"), create);
	XMLData("port", val.port, xmlcontext->getContextFirst("client"), create);
	XMLData("time_out", val.timeOut, xmlcontext->getContextFirst("client"), create);
	XMLData("lost_packages", val.lostPackages, xmlcontext->getContextFirst("client"), create);

	const std::string name = "controller";
	std::pair<XMLContext::XMLContextMap::const_iterator, XMLContext::XMLContextMap::const_iterator> range = xmlcontext->getContextMap().equal_range(name);
	if (range.first == range.second)
		throw MsgXMLParserNameNotFound(Message::LEVEL_ERROR, "XMLData(): Name not found: %s %s", xmlcontext->getName().c_str(), name.c_str());

	for (XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second; ++i) {
		MultiCtrl::ControllerDesc desc;
		desc.pControllerDesc = Controller::Desc::load(context, const_cast<XMLContext*>(&i->second));
		try {
			XMLData(desc, context, const_cast<XMLContext*>(&i->second), (bool)false);
		}
		catch (const MsgXMLParser&) {
		}
		val.controllers.insert(val.controllers.end(), desc);
	}
}

//------------------------------------------------------------------------------