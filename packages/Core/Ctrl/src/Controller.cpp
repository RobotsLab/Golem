/** @file Controller.cpp
 * 
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sys/Library.h>
#include <Golem/Sys/Context.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Ctrl/Controller.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

template <> const char* golem::Chainspace::Name = "Chainspace";
template <> const char* golem::Configspace::Name = "Configspace";
template <> const char* golem::Reservedspace::Name = "Reservedspace";

//------------------------------------------------------------------------------

const ptrdiff_t Controller::ReservedOffset::UNAVAILABLE = numeric_const<ptrdiff_t>::MAX;

const std::string Controller::ReservedOffset::TypeName [] = {
	std::string("Pointer"),
	std::string("Chainspace"),
	std::string("Configpace"),
};

//------------------------------------------------------------------------------

Controller::Desc::Ptr Controller::Desc::load(Context* context, const std::string& libraryPath, const std::string& configPath) {
	Controller::Desc::Ptr pDesc;

	context->debug("Controller::Desc::load(): loading library %s and config %s.xml...\n", libraryPath.c_str(), configPath.c_str());

	// open library and load function
	Handle library = context->getLibrary(libraryPath);
	LoadObjectDesc loadDesc = (LoadObjectDesc)library.getFunction("loadControllerDesc");
	
	// load config
	golem::XMLParser::Ptr parser;
	try {
		// first try to load directly the specified config
		parser = XMLParser::load(configPath + ".xml");
	}
	catch (const golem::Message&) {
		// if failed, attempt to load from library location
		parser = XMLParser::load(library.getDir() + configPath + ".xml");
	}
	loadDesc(context, parser->getContextRoot()->getContextFirst("golem controller"), &pDesc);

	pDesc->libraryPath = libraryPath;
	pDesc->configPath = configPath;

	return pDesc;
}

Controller::Desc::Ptr Controller::Desc::load(Context* context, XMLContext* xmlcontext) {
	// driver and config paths must be specified in xmlcontext
	std::string libraryPath, configPath;
	XMLData("library_path", libraryPath, xmlcontext);
	XMLData("config_path", configPath, xmlcontext);
	// load driver and config
	return load(context, libraryPath, configPath);
}

//------------------------------------------------------------------------------

Controller::Controller(golem::Context& context) : context(context), pCallbackDataSync(NULL) {
}

Controller::~Controller() {
	// release data in each destructor of a derived class
}

void Controller::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgControllerInvalidDesc(Message::LEVEL_CRIT, "Controller::create(): Invalid description");

	libraryPath = desc.libraryPath;
	configPath = desc.configPath;
	// type and id
	const std::string header = "GolemCtrl";
	type = libraryPath.substr(libraryPath.find(header) == 0 ? header.length() : 0, std::string::npos);
	id = type + "+" + configPath.substr(configPath.find(header) == 0 ? header.length() : 0, std::string::npos);

	name = desc.name;
	
	globalPose = desc.globalPose;
	pCallbackDataSync = desc.pCallbackDataSync;

	setStateInfo(desc); // throws

	chainSeq.clear();
	chains.fill((Chain*)NULL);
	joints.fill((Joint*)NULL);
	for (idx_t i = 0; i < (idx_t)desc.chains.size(); ++i) {
		const Chainspace::Index j = stateInfo.getChains().begin() + i;

		Chain::Desc* pDesc = desc.chains[i].get();
		pDesc->index = j; // index
		pDesc->indexLocal = i; // index within controller
		Chain::Ptr pChain = pDesc->create(*this); // throws
		chainSeq.push_back(pChain);
		
		chains[j] = pChain.get();
		for (idx_t k = 0; k < (idx_t)pChain->getJoints().size(); ++k) {
			const Configspace::Index l = stateInfo.getJoints(j).begin() + k;
			
			joints[l] = pChain->getJoints()[k];
		}
	}
	
	for (Configspace::Index i = stateInfo.getJoints().begin(); i < stateInfo.getJoints().end(); ++i) {
		const Joint* pJoint = getJoints()[i];

		configspaceMin.fromGenCoord(i, pJoint->getMin());
		configspaceMax.fromGenCoord(i, pJoint->getMax());
		configspaceOffset.fromGenCoord(i, pJoint->getOffset());
	}

	//for (Chainspace::Index i = getStateInfo().getChains().begin(); i < getStateInfo().getChains().end(); ++i) {
	//	for (Configspace::Index j = getStateInfo().getJoints(i).begin(); j < getStateInfo().getJoints(i).end(); ++j) {
	//		printf("ctrl %s: chain=%d, joint=%d, joint_size=%d, linked_chain=%d\n", getName().c_str(), *i, *j, getChains()[i]->getJoints().size(), getChains()[i]->getLinkedChainIndex());
	//	}
	//}

	//controllers.clear();
	//controllers.push_back(this);
}

void Controller::setStateInfo(const Desc& desc) {
	stateInfo.resetJoints(desc.jointBegin, desc.chainBegin); // throws
	for (Chain::Desc::Seq::const_iterator i = desc.chains.begin(); i != desc.chains.end(); ++i)
		stateInfo.addJoints((idx_t)(*i)->joints.size()); // throws

	stateInfo.resetReserved(desc.reservedBegin); // throws
	stateInfo.addReserved(desc.reservedSize); // throws
}

Controller::State Controller::createState() const {
	return Controller::State(*this);
}

void Controller::setToDefault(Controller::State& state) const {
	state.t = SEC_TM_REAL_ZERO;
	state.cpos.setToDefault(state.getInfo().getJoints());
	state.cvel.setToDefault(state.getInfo().getJoints());
	state.cacc.setToDefault(state.getInfo().getJoints());
	state.reserved.setToDefault(state.getInfo().getReserved());
}

//------------------------------------------------------------------------------

void Controller::chainForwardTransform(const ConfigspaceCoord& cc, WorkspaceChainCoord& wc) const {
	const Mat34 globalPose = getGlobalPose();
	
	for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
		const Chain* chain = getChains()[i];
		const Chainspace::Index linkedChainIndex = chain->getLinkedChainIndex();
		
		chain->chainForwardTransform(&cc[stateInfo.getJoints(i).begin()], wc[i]);
		
		wc[i].multiply(linkedChainIndex < i ? wc[linkedChainIndex] : globalPose, wc[i]);
	}
}

void Controller::jointForwardTransform(const ConfigspaceCoord& cc, WorkspaceJointCoord& wc) const {
	const Mat34 globalPose = getGlobalPose();
	
	for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
		const Chain* chain = getChains()[i];
		const Configspace::Index begin = stateInfo.getJoints(i).begin();
		const Configspace::Index end = stateInfo.getJoints(i).end();
		const Chainspace::Index linkedChainIndex = chain->getLinkedChainIndex();
		const Configspace::Index linkedJointIndex = linkedChainIndex < i ? stateInfo.getJoints(linkedChainIndex).end() - 1 : stateInfo.getJoints().end();
		
		chain->jointForwardTransform(&cc[begin], &wc[begin]);

		for (Configspace::Index j = begin; j < end; ++j)
			wc[j].multiply(linkedJointIndex < j ? wc[linkedJointIndex] : globalPose, wc[j]);
	}
}

void Controller::jacobian(const ConfigspaceCoord& cc, Jacobian& jac) const {
	// TODO connected/dependent chains: e.g. arm + hand fingers
	
	for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
		const Chain* chain = getChains()[i];
		
		chain->jacobian(&cc[stateInfo.getJoints(i).begin()], &jac[stateInfo.getJoints(i).begin()]);
	}
}

//------------------------------------------------------------------------------

Mat34 Controller::getGlobalPose() const {
	CriticalSectionWrapper csw(csData);
	return globalPose;
}

void Controller::setGlobalPose(const Mat34 &globalPose) {
	CriticalSectionWrapper csw(csData);
	this->globalPose = globalPose;
}

//------------------------------------------------------------------------------
