/** @file DLRHitHandII.h
 * 
 * DLR Hit Hand II controller
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_DLRHITHANDII_DLRHITHANDII_H_
#define _GOLEM_CTRL_DLRHITHANDII_DLRHITHANDII_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/DLR/DLRHitHandII.h>
#include <memory>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgDLRHitHandIIDev, MsgController)
MESSAGE_DEF(MsgDLRHitHandIIDevOpenPort, MsgDLRHitHandIIDev)
MESSAGE_DEF(MsgDLRHitHandIIDevConnect, MsgDLRHitHandIIDev)
MESSAGE_DEF(MsgDLRHitHandIINoHand, MsgDLRHitHandIIDev)
MESSAGE_DEF(MsgDLRHitHandIIWrongPort, MsgDLRHitHandIIDev)
MESSAGE_DEF(MsgDLRHitHandIISendErr, MsgDLRHitHandIIDev)

//------------------------------------------------------------------------------

class DevComInp;
class DevComOut;

class GOLEM_LIBRARY_DECLDIR DLRHitHandIIDev : public DLRHitHandII {
public:
	/** DLRHitHandIIDev description */
	class GOLEM_LIBRARY_DECLDIR Desc : public DLRHitHandII::Desc {
	public:
		/** QNX controller box address */
		std::string peerAddr;
		/** QNX controller box port */
		unsigned peerPort;
		/** Local port */
		unsigned localPort;
		/** Communication timeout */
		SecTmReal commTimeout;
		/** Wakeup period during initialisation */
		SecTmReal initWakeup;
		/** Sleep period during initialisation */
		SecTmReal initCommSleep;
		/** Max initialisation tries per finger */
		golem::U32 initCommTries;
		/** Joints offsets */
		Real offset[NUM_JOINTS]; // [rad]
		/** Finger mapping Golem -> DLR */
		golem::U32 finger[NUM_CHAINS];
		/** Finger status */
		bool enabled[NUM_CHAINS];

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(DLRHitHandIIDev, Controller::Ptr, Context&)

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			DLRHitHandII::Desc::setToDefault();

			name = "DLR Hit Hand II";
			
			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.0005);
			simDeltaSend = SecTmReal(0.0005);

			peerAddr = "192.168.200.1";
			peerPort = 4445;
			localPort = 4444;
			commTimeout = SecTmReal(5.0);
			initWakeup = SecTmReal(2.0);
			initCommSleep = SecTmReal(0.1);
			initCommTries = 50;

			std::fill(offset, offset + NUM_JOINTS, REAL_ZERO);
			for (golem::U32 i = 0; i < NUM_CHAINS; ++i)
				finger[i] = i;
			std::fill(enabled, enabled + NUM_CHAINS, true);
		}

		virtual bool isValid() const {
			if (!DLRHitHandII::Desc::isValid())
				return false;

			if (peerAddr.empty() || commTimeout < SEC_TM_REAL_ZERO || commTimeout >= threadTimeOut)
				return false;
			if (initCommSleep <= SEC_TM_REAL_ZERO || initCommTries < 1 || initWakeup <= SEC_TM_REAL_ZERO)
				return false;
			
			for (size_t i = 0; i < NUM_JOINTS_CTRL; ++i)
				if (!Math::isFinite(offset[i]))
					return false;

			for (golem::U32 i = 0; i < NUM_CHAINS; ++i)
				if (finger[i] >= NUM_CHAINS)
					return false;

			return true;
		}
	};

	/** Stops the device. */
	virtual void stop();

	/** Resumes the device. */
	virtual void resume();

	/** Joint offset. */
	virtual void getOffset(Real* offset) const;
	virtual void setOffset(const Real* offset);

	/** Release resources */
	virtual ~DLRHitHandIIDev();

protected:
	/** Raw Output Packet */
	struct OutPacket {
		float Con_Mod;
		float Enable[NUM_CHAINS];
		float Pos_Com[NUM_JOINTS_CTRL];
		float Stiffness[NUM_JOINTS_CTRL];
		float Damping[NUM_JOINTS_CTRL];
		float Velocity[NUM_JOINTS_CTRL];
		float Kp[NUM_JOINTS_CTRL];
		float Con_Mod1;
		float Enable1[NUM_CHAINS];
		float Pos_Com1[NUM_JOINTS_CTRL];
		float Stiffness1[NUM_JOINTS_CTRL];
		float Damping1[NUM_JOINTS_CTRL];
		float Velocity1[NUM_JOINTS_CTRL];
		float Kp1[NUM_JOINTS_CTRL];
		float emergency;
		float Reserved[87];
	};
	/** Raw Input Packet */
	struct InpPacket {
		float Pos[NUM_JOINTS_CTRL];
		float Toruqe[NUM_JOINTS_CTRL];
		float Velocity[NUM_JOINTS_CTRL];
		float Enabled[NUM_CHAINS];
		float brakestatus;
		float handconfig[2];
		float commstatus;
		float Pos1[NUM_JOINTS_CTRL];
		float Toruqe1[NUM_JOINTS_CTRL];
		float Velocity1[NUM_JOINTS_CTRL];
		float Enabled1[NUM_CHAINS];
		float Reserved[146];
	};

	/** QNX controller box address */
	std::string peerAddr;
	/** QNX controller box port */
	unsigned peerPort;
	/** Local port */
	unsigned localPort;
	/** Communication timeout */
	SecTmReal commTimeout;
	/** Wakeup period during initialisation */
	SecTmReal initWakeup;
	/** Sleep period during initialisation */
	SecTmReal initCommSleep;
	/** Max initialisation tries per finger */
	golem::U32 initCommTries;
	/** Joints offsets */
	Real offset[NUM_JOINTS]; // [rad]
	/** Finger mapping Golem -> DLR */
	golem::U32 finger[NUM_CHAINS];
	/** Finger status */
	bool enabled[NUM_CHAINS];
	/** Joints offsets CS */
	mutable CriticalSection csOffset;
	
	/** Current device state */
	State::Ptr state;
	/** Low level input interface */
	golem::shared_ptr<DevComInp, reference_cnt_vt_base<DevComInp> > pComInp;
	/** Low level output interface */
	golem::shared_ptr<DevComOut, reference_cnt_vt_base<DevComOut> > pComOut;
	/** Input packet */
	InpPacket rawInp;
	/** Output packet */
	OutPacket rawOut;

	void fromState(const State& state, OutPacket& rawOut) const;
	void toState(const InpPacket& rawInp, State& state) const;

	bool recv(State& state);
	bool send(const State& state);

	virtual void sysSync();
	virtual void sysRecv(State& state);
	virtual void sysSend(const State& prev, const State& next, bool bSendPrev, bool bSendNext);

	virtual void userCreate();
	void create(const Desc& desc);

	DLRHitHandIIDev(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_DLRHITHANDII_DLRHITHANDII_H_*/
