/** @file EZGripper.h
*
* Zimmer Parallel Gripper controller
*
* @author	Claudio Zito
*
* @copyright  Copyright (C) 2018 Claudio Zito, University of Birmingham, UK
*
* @license  This file copy is licensed to you under the terms described in
*           the License.txt file included in this distribution.
*
*/

#pragma once
#ifndef _GOLEM_CTRL_EZGRIPPER_EZGRIPPER_H_
#define _GOLEM_CTRL_EZGRIPPER_EZGRIPPER_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/SingleCtrl/SingleCtrl.h>
#include <Golem/Sys/Library.h>
#include <Golem/Sys/XMLParser.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Basic class for Sake Robotics EZ Gripper */
class GOLEM_LIBRARY_DECLDIR EZGripper : public SingleCtrl {
public:
	/** Number of fingers */
	static const U32 NUM_CHAINS = 2;
	/** Number of joints */
	static const U32 NUM_JOINTS_LEFT = 2;
	/** Number of joints */
	static const U32 NUM_JOINTS_RIGHT = 2;

	/** Finger chain index */
	static const U32 LEFT_FINGER_CHAIN_INDEX = 0;
	/** Finger chain index */
	static const U32 RIGHT_FINGER_CHAIN_INDEX = 1;

	/** Number of joints */
	static const U32 CHAIN_NUM_JOINTS[NUM_CHAINS];
	/** Number of joints */
	static const U32 NUM_JOINTS = NUM_JOINTS_LEFT + NUM_JOINTS_RIGHT;
	/** Number of controllable joints */
	static const U32 NUM_JOINTS_CTRL = NUM_JOINTS - 1;

	/** Maximum open position for gripper */
	static const U32 GRIP_MAX = 2500; 
	/** Maximum torque */
	static const U32 TORQUE_MAX = 800;
	/** Minimum torque */
	static const U32 TORQUE_HOLD = 100;

	/** Number of motors */
	static const U32 NUM_MOTORS = 1;

	/** Synergy scale factor */
	static const short SYNERGY_GAIN = 17000;

	/** EZGripper state reserved area variable indices */
	enum ReservedIndex {
		/** Motor position (inp/out) (1, short) */
		RESERVED_INDEX_MOTOR_POSITION = Controller::RESERVED_INDEX_SIZE,
		/** Motor current (inp) (1, short) */
		RESERVED_INDEX_MOTOR_CURRENT,
		/** Reserved Index size */
		RESERVED_INDEX_SIZE = RESERVED_INDEX_MOTOR_CURRENT + 1,
	};

	/** EZGripper description */
	class GOLEM_LIBRARY_DECLDIR Desc : public SingleCtrl::Desc {
	public:
		/** Synergy vector */
		ConfigspaceCoord synergy;

		Desc() {
			setToDefault();
		}
		virtual void setToDefault() {
			SingleCtrl::Desc::setToDefault();

			reservedSize = (U32)ReservedOffset::getSize(RESERVED_OFFSET, RESERVED_OFFSET + RESERVED_INDEX_SIZE);

			name = "SAKE Robotics EZ Gripper";

			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.0005);
			simDeltaSend = SecTmReal(0.0005);

			for (size_t i = 0; i < NUM_CHAINS; ++i) {
				chains.push_back(Chain::Desc::Ptr(new Chain::Desc));
				for (U32 j = 0; j < CHAIN_NUM_JOINTS[i]; ++j) {
					chains.back()->joints.push_back(Joint::Desc::Ptr(new Joint::Desc));
				}
			}

			synergy.fill(REAL_ZERO);
		}
		virtual bool isValid() const {
			if (!SingleCtrl::Desc::isValid())
				return false;

			if (chains.size() != NUM_CHAINS)
				return false;
			for (size_t i = 0; i < NUM_CHAINS; ++i)
				if (chains[i]->joints.size() != CHAIN_NUM_JOINTS[i])
					return false;

			return true;
		}
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(EZGripper, Controller::Ptr, Context&)
	};

	/** Set synergy pointer, if synergyPointer < NUM_JOINTS, use specified joint position, otherwise MotorPosition[0] */
	inline void setSynergyPointer(U32 synergyPointer = 0) {
		this->synergyPointer = synergyPointer;
	}
	/** Synergy pointer */
	inline U32 getSynergyPointer() const {
		return synergyPointer;
	}

	/** Synergy position */
	Real getSynergy(const State& state) const;
	/** Synergy position */
	void setSynergy(Real s, State& state) const;

	/** Synergy map: Synergy coord -> Joints coord */
	void getSynergyMap(Real s, Real* c) const;
	/** Synergy map: Synergy coord -> State */
	void getSynergyMap(Real s, State& out) const;

	/** Interpolates the controller state at time t. */
	virtual void lookupState(SecTmReal t, State &state) const;

	/** Sets to default a given state. */
	virtual void setToDefault(State& state) const;

	/** Returns reserved data offset. */
	virtual ptrdiff_t getReservedOffset(U32 type) const;

	/** Clamp configuration. */
	virtual void clampConfig(GenConfigspaceCoord& config) const;

	/** Interpolates the controller state at time t. */
	virtual void interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const;

	/** Reserved access */
	CONTROLLER_STATE_RESERVED(MotorPosition, short, RESERVED_INDEX_MOTOR_POSITION)
	CONTROLLER_STATE_RESERVED(MotorCurrent, short, RESERVED_INDEX_MOTOR_CURRENT)

	/** Release resources */
	virtual ~EZGripper();

protected:
	/** Reserved area size */
	static const ReservedOffset RESERVED_OFFSET[RESERVED_INDEX_SIZE];

	/** Synergy vector */
	ConfigspaceCoord synergy;

	/** Synergy pointer */
	U32 synergyPointer;
	/** Reserved data offset: type -> offset mapping */
	std::vector<ptrdiff_t> reservedOffset;

	/** Conversion */
	Real motorPositionToSynergy(short position) const;
	/** Conversion */
	short synergyToMotorPosition(Real synergy) const;

	/** Sets controller state properties */
	virtual void setStateInfo(const Controller::Desc& desc);

	/** Receives device state. */
	virtual void sysRecv(State& state);

	void create(const Desc& desc);
	EZGripper(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

namespace golem {

	/** Reads/writes object from/to a given XML context */
	void XMLData(EZGripper::Desc &val, XMLContext* context, bool create = false);

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CTRL_EZGRIPPER_EZGRIPPER_H_*/
