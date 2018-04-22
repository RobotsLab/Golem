/** @file DLRHitHandII.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/DLRHitHandII/DLRHitHandII.h>
#include <Golem/Ctrl/DLRHitHandII/Data.h>
#include <Golem/Sys/LoadObjectDesc.h>
#include <Golem/Ctrl/DLRHitHandII/DevComInp.h>
#include <Golem/Ctrl/DLRHitHandII/DevComOut.h>

//#define DLR_DLRHITHANDII_DEBUG

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc) {
	loadObjectDesc<golem::DLRHitHandIIDev::Desc, Controller::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Controller::Desc::Ptr*)pControllerDesc);
}

unsigned SecToUSec(SecTmReal t) {
	return	t <= SEC_TM_REAL_ZERO ? numeric_const<unsigned>::ZERO : t >= SecTmReal(numeric_const<unsigned>::MAX/1000000) ? numeric_const<unsigned>::MAX : unsigned(1000000.*t);
}

//------------------------------------------------------------------------------

#ifdef DLR_DLRHITHANDII_DEBUG
static int x = 0;
#endif

void DLRHitHandIIDev::fromState(const State& state, OutPacket& rawOut) const {
	// offset
	Real offset[NUM_JOINTS] = {REAL_ZERO};
	{
		golem::CriticalSectionWrapper csw(csOffset);
		std::copy(this->offset, this->offset + NUM_JOINTS, offset);
	}

#ifdef DLR_DLRHITHANDII_DEBUG
const bool show = x++%500 == 0;
float p[NUM_JOINTS], v[NUM_JOINTS], s[NUM_JOINTS], d[NUM_JOINTS], kp[NUM_JOINTS];
#endif

	for (U32 i = 0; i < NUM_CHAINS; ++i) {
		const Chainspace::Index chain = getStateInfo().getChains().begin() + (idx_t)i;
		const U32 l = finger[i];

		rawOut.Enable[l] = getEnable(state)[chain] ? float(1.0) : float(0.0);

		// skip the last non-controllable joint
		for (U32 j = 0; j < DLRHitHandIIChain::NUM_JOINTS_CTRL; ++j) {
			// chain: joint: 0->0, 1->1, 2->2, 3->ignored
			//const U32 k[2] = {i*DLRHitHandIIChain::NUM_JOINTS + j, l*DLRHitHandIIChain::NUM_JOINTS_CTRL + j};
			// chain: joint: 0->2, 1->1, 2->0, 3->ignored
			const U32 k[2] = {i*DLRHitHandIIChain::NUM_JOINTS + j, (l + 1)*DLRHitHandIIChain::NUM_JOINTS_CTRL - j - 1};
			const Configspace::Index joint = getStateInfo().getJoints().begin() + (idx_t)k[0];
			
			// update joint target position + offset correction
#ifdef DLR_DLRHITHANDII_DEBUG
p[j]=
#endif
			rawOut.Pos_Com[k[1]] = (float)Math::radToDeg(state.cpos[joint] - offset[k[0]]);
			// HACK: for some unknown reason the sent (target) position of the first joint in each chain is reversed as compared to the received joint one
			if (j == 0)	rawOut.Pos_Com[k[1]] = -rawOut.Pos_Com[k[1]];

#ifdef DLR_DLRHITHANDII_DEBUG
v[j]=
#endif
			rawOut.Velocity[k[1]] = (float)Math::radToDeg(getVelocityMax(state)[joint]);
#ifdef DLR_DLRHITHANDII_DEBUG
s[j]=
#endif
			rawOut.Stiffness[k[1]] = (float)getStiffness(state)[joint];
#ifdef DLR_DLRHITHANDII_DEBUG
d[j]=
#endif
			rawOut.Damping[k[1]] = (float)getDamping(state)[joint];
#ifdef DLR_DLRHITHANDII_DEBUG
kp[j]=
#endif
			rawOut.Kp[k[1]] = (float)getKP(state)[joint];
		}
#ifdef DLR_DLRHITHANDII_DEBUG
if (show)
	context.debug("%d: E=%d, p={%f, %f, %f}, v={%f, %f, %f}, s={%f, %f, %f}, d={%f, %f, %f}, k={%f, %f, %f}\n", 
		i, (int)rawOut.Enable[i],
		Math::degToRad(p[0]),Math::degToRad(p[1]),Math::degToRad(p[1]),
		Math::degToRad(v[0]),Math::degToRad(v[1]),Math::degToRad(v[1]),
		Math::degToRad(s[0]),Math::degToRad(s[1]),Math::degToRad(s[1]),
		Math::degToRad(d[0]),Math::degToRad(d[1]),Math::degToRad(d[1]),
		Math::degToRad(kp[0]),Math::degToRad(kp[1]),Math::degToRad(kp[1])
	);
#endif
	}

	rawOut.Con_Mod = (float)getConMod(state);
	rawOut.emergency = getEmergency(state) ? float(1.0) : float(0.0);
}

void DLRHitHandIIDev::toState(const InpPacket& rawInp, State& state) const {
	// offset
	Real offset[NUM_JOINTS] = {REAL_ZERO};
	{
		golem::CriticalSectionWrapper csw(csOffset);
		std::copy(this->offset, this->offset + NUM_JOINTS, offset);
	}

	for (U32 i = 0; i < NUM_CHAINS; ++i) {
		const Chainspace::Index chain = getStateInfo().getChains().begin() + (idx_t)i;
		const U32 l = finger[i];

		getEnabled(state)[chain] = rawInp.Enabled[l] == float(1.0) ? true : false;

		// duplicate data for the last non-controllable joint
		for (U32 j = 0; j < DLRHitHandIIChain::NUM_JOINTS; ++j) {
			// chain: joint: 0->0, 1->1, 2->2, 2->3
			//const U32 k[2] = {i*DLRHitHandIIChain::NUM_JOINTS + j, l*DLRHitHandIIChain::NUM_JOINTS_CTRL + std::min(j, DLRHitHandIIChain::NUM_JOINTS_CTRL - 1)};
			// chain: joint: 0->3, 0->2, 1->1, 2->0
			const U32 k[2] = {i*DLRHitHandIIChain::NUM_JOINTS + j, (l + 1)*DLRHitHandIIChain::NUM_JOINTS_CTRL - std::min(j, DLRHitHandIIChain::NUM_JOINTS_CTRL - 1) - 1};
			const Configspace::Index joint = getStateInfo().getJoints().begin() + (idx_t)k[0];
			
			//state.cpos[joint] = REAL_ZERO;  // mapped via inp().pos()
			state.cvel[joint] = REAL_ZERO; // inp().velocity() mapped onto reserved area
			state.cacc[joint] = REAL_ZERO; // not mapped directly

			state.cpos[joint] = Math::degToRad((Real)rawInp.Pos[k[1]]) + offset[k[0]];
			//getVelocityMax(state)[joint] = Math::degToRad((Real)rawInp.Velocity[k[1]]); //ignore
			getTorque(state)[joint] = (Real)rawInp.Toruqe[k[1]];
		}
	}

	getBrakeStatus(state) = (int)Math::round(rawInp.brakestatus);
	getHandConfig(state) = (int)Math::round(rawInp.handconfig[0]);
	getCommStatus(state) = (int)Math::round(rawInp.commstatus);
}

//------------------------------------------------------------------------------

DLRHitHandIIDev::DLRHitHandIIDev(golem::Context& context) : DLRHitHandII(context) {
}

DLRHitHandIIDev::~DLRHitHandIIDev() {
	DLRHitHandII::release();
}

void DLRHitHandIIDev::create(const Desc& desc) {
	peerAddr = desc.peerAddr;
	peerPort = desc.peerPort;
	localPort = desc.localPort;
	commTimeout = desc.commTimeout;
	initWakeup = desc.initWakeup;
	initCommSleep = desc.initCommSleep;
	initCommTries = desc.initCommTries;
	std::copy(desc.offset, desc.offset + NUM_JOINTS, offset);
	std::copy(desc.finger, desc.finger + NUM_CHAINS, finger);
	std::copy(desc.enabled, desc.enabled + NUM_CHAINS, enabled);

	DLRHitHandII::create(desc); // throws
}

void DLRHitHandIIDev::userCreate() {
	pComInp.reset(new reference_cnt_vt<DevComInp>(new DevComInp));
	if (pComInp->init(localPort, sizeof(InpPacket)) < 0)
		throw MsgDLRHitHandIIDevOpenPort(Message::LEVEL_CRIT, "DLRHitHandIIDev::userCreate(): Unable to open port %d", localPort);

	pComOut.reset(new reference_cnt_vt<DevComOut>(new DevComOut));
	if (pComOut->init(peerAddr, peerPort, sizeof(OutPacket)) < 0)
		throw MsgDLRHitHandIIDevConnect(Message::LEVEL_CRIT, "DLRHitHandIIDev::userCreate(): Unable to initialise %s:%d", peerAddr.c_str(), peerPort);

	::memset(&rawInp, 0, sizeof(InpPacket));
	::memset(&rawOut, 0, sizeof(OutPacket));
	
	state.reset(new State(createState()));
	setToDefault(*state);
	
	getEnable(*state).fill(getStateInfo().getChains(), false); // disable
	getConMod(*state) = 0; // position control OFF

	const Chainspace::Index chainBegin = getStateInfo().getChains().begin();
	for (U32 i = 0, j = 0, k = 0; i < NUM_CHAINS;) {
#ifdef DLR_DLRHITHANDII_DEBUG
x = 0;
#endif
		// wait for incomming connection
		if (!recv(*state)) {
			context.info("DLRHitHandIIDev::userCreate(): Waiting for incomming connection on port %d\n", localPort);
			continue;
		}

		// initial check - see QT DLR hand demo
		if (i == 0 && j == 0) {
			if (getHandConfig(*state) == 0 || getCommStatus(*state) == 5)
				throw MsgDLRHitHandIINoHand(Message::LEVEL_CRIT, "DLRHitHandIIDev::userCreate(): No hand connected!");
			if (getCommStatus(*state) == 13)
				throw MsgDLRHitHandIIWrongPort(Message::LEVEL_CRIT, "DLRHitHandIIDev::userCreate(): Hand connected to a wrong port!");

			// let the QNX ArdNet to initialise ("wake up") before the first send (required only in the case it has just been started what cannot be detected)
			// if it is too short, the hand bahaviour can be unpredictable
			Sleep::msleep(SecToMSec(initWakeup));
		}

		// sequentially go through all init stages
		switch (j) {
		case 0:
			getEnable(*state)[chainBegin + (idx_t)i] = false; // disable finger #i
			j = 1;
			break;
		case 1:
			//if (!getEnabled(*state)[chainBegin + (idx_t)i]) // wait until finger #i is disabled
			{
				getEnable(*state)[chainBegin + (idx_t)i] = enabled[i]; // enable finger #i
				j = 2;
				k = 0;
			}
			break;
		case 2:
			if (getEnabled(*state)[chainBegin + (idx_t)i] == enabled[i] || ++k > initCommTries) // wait until finger #i is enabled
			{
				if (k > initCommTries)
					context.error("DLRHitHandIIDev::userCreate(): %s cannot be %s!\n", this->getChains()[chainBegin + (idx_t)i]->getName().c_str(), enabled[i] ? "enabled" : "disabled");
				if (++i < NUM_CHAINS)
					j = 0;
				else
					getConMod(*state) = 1; // position control ON
			}
			break;
		}

		const U32 l = i < NUM_CHAINS ? i : NUM_CHAINS - 1;
		context.debug("DLRHitHandIIDev::userCreate(): handconfig %d, commstatus %d, finger %d, stage %d, out_enable %d, inp_enabled {%d, %d, %d, %d, %d}\n",
			getHandConfig(*state), getCommStatus(*state), l, j,
			(int)getEnable(*state)[chainBegin + (idx_t)l],
			(int)getEnabled(*state)[chainBegin + 0],
			(int)getEnabled(*state)[chainBegin + 1],
			(int)getEnabled(*state)[chainBegin + 2],
			(int)getEnabled(*state)[chainBegin + 3],
			(int)getEnabled(*state)[chainBegin + 4]
		);

		// send command
		if (!send(*state))
			throw MsgDLRHitHandIISendErr(Message::LEVEL_ERROR, "DLRHitHandIIDev::userCreate(): send error");

		Sleep::msleep(SecToMSec(initCommSleep));
	}
}

//------------------------------------------------------------------------------

void DLRHitHandIIDev::stop() {
	CriticalSectionWrapper csw(csCommand);	// there can be readers and writers in other threads

	State next = qCommand->front();
	next.cvel.setToDefault(getStateInfo().getJoints().begin(), getStateInfo().getJoints().end());
	next.cacc.setToDefault(getStateInfo().getJoints().begin(), getStateInfo().getJoints().end());
	getEmergency(next) = true;
	next.t += getCycleDuration();
	qCommand->erase(++qCommand->begin(), qCommand->end());
	qCommand->push_back(next);
}

void DLRHitHandIIDev::resume() {
	CriticalSectionWrapper csw(csCommand);	// there can be readers and writers in other threads

	State next = qCommand->back();
	getEmergency(next) = false;
	next.t += getCycleDuration();
	qCommand->push_back(next);
}

void DLRHitHandIIDev::getOffset(Real* offset) const {
	golem::CriticalSectionWrapper csw(csOffset);
	//std::copy(this->offset, this->offset + NUM_JOINTS, offset); // warnings
	//std::copy(this->offset, this->offset + NUM_JOINTS, stdext::checked_array_iterator<Real*>(offset, NUM_JOINTS));
	for (U32 ptr = 0; ptr < NUM_JOINTS; ++ptr)
		offset[ptr] = this->offset[ptr];
}

void DLRHitHandIIDev::setOffset(const Real* offset) {
	golem::CriticalSectionWrapper csw(csOffset);
	//std::copy(offset, offset + NUM_JOINTS, this->offset); // warnings
	//std::copy(offset, offset + NUM_JOINTS, stdext::checked_array_iterator<Real*>(this->offset, NUM_JOINTS));
	for (U32 ptr = 0; ptr < NUM_JOINTS; ++ptr)
		this->offset[ptr] = offset[ptr];
}

//------------------------------------------------------------------------------

bool DLRHitHandIIDev::recv(State& state) {
	if (pComInp->rec(&rawInp, sizeof(InpPacket), SecToUSec(commTimeout)) > 0) {
		toState(rawInp, state);
		state.t = context.getTimer().elapsed();
		return true;
	}
	else
		return false;
}

bool DLRHitHandIIDev::send(const State& state) {
	fromState(state, rawOut);
	return pComOut->send(&rawOut, sizeof(OutPacket)) > 0;
}

void DLRHitHandIIDev::sysSync() {
	if (!recv(*state))
		context.error("DLRHitHandIIDev::sysSync(): communication timeout\n");
}

void DLRHitHandIIDev::sysRecv(State& state) {
	state = *this->state;
}

void DLRHitHandIIDev::sysSend(const State& prev, const State& next, bool bSendPrev, bool bSendNext) {
	if (!send(next))
		context.error("DLRHitHandIIDev::sysSend(): send error\n");
}

//------------------------------------------------------------------------------
