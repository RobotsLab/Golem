/** @file PISASoftHand.h
 *
 * PISA Soft Hand base classes
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
#ifndef _GOLEM_CTRL_PISA_PISASOFTHAND_H_
#define _GOLEM_CTRL_PISA_PISASOFTHAND_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/SingleCtrl/SingleCtrl.h>
#include <Golem/Sys/Library.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** PISA Hand controller base class. */
class GOLEM_LIBRARY_DECLDIR PISASoftHand: public SingleCtrl {
public:
	/** Number of fingers */
	static const U32 NUM_CHAINS = 5;
	/** Number of joints */
	static const U32 NUM_JOINTS_THUMB = 3;
	/** Number of joints */
	static const U32 NUM_JOINTS_INDEX = 4;
	/** Number of joints */
	static const U32 NUM_JOINTS_MIDDLE = 4;
	/** Number of joints */
	static const U32 NUM_JOINTS_RING = 4;
	/** Number of joints */
	static const U32 NUM_JOINTS_SMALL = 4;
	/** Number of joints */
	static const U32 CHAIN_NUM_JOINTS[NUM_CHAINS];
	/** Number of joints */
	static const U32 NUM_JOINTS = NUM_JOINTS_THUMB + NUM_JOINTS_INDEX + NUM_JOINTS_MIDDLE + NUM_JOINTS_RING + NUM_JOINTS_SMALL;

	/** Number of sensors */
	static const U32 NUM_SENSORS = 3;
	/** Number of motors */
	static const U32 NUM_MOTORS = 2;

	/** Synergy scale factor */
	static const short SYNERGY_GAIN = 17000;

	/** DLRHitHandII state reserved area variable indices */
	enum ReservedIndex {
		/** Motor position (inp/out) (1, short) */
		RESERVED_INDEX_MOTOR_POSITION = Controller::RESERVED_INDEX_SIZE,
		/** Motor current (inp) (1, short) */
		RESERVED_INDEX_MOTOR_CURRENT,
		/** Reserved Index size */
		RESERVED_INDEX_SIZE = RESERVED_INDEX_MOTOR_CURRENT + 1,
	};

	/** PISA Hand description */
	class GOLEM_LIBRARY_DECLDIR Desc : public SingleCtrl::Desc {
	public:
		/** Synergy vector */
		ConfigspaceCoord synergy;

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			SingleCtrl::Desc::setToDefault();

			cycleDurationCtrl = false;
			timeQuant = SecTmReal(0.0001);
			cycleDurationInit = SecTmReal(0.1);

			for (size_t i = 0; i < NUM_CHAINS; ++i) {
				chains.push_back(Chain::Desc::Ptr(new Chain::Desc));
				for (U32 j = 0; j < CHAIN_NUM_JOINTS[i]; ++j){
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

	/** Sets to default a given state. */
	virtual void setToDefault(State& state) const;

	/** Returns reserved data offset. */
	virtual ptrdiff_t getReservedOffset(U32 type) const;

	/** Interpolates the controller state at time t. */
	virtual void interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const;

	CONTROLLER_STATE_RESERVED(MotorPosition, short, RESERVED_INDEX_MOTOR_POSITION)
	CONTROLLER_STATE_RESERVED(MotorCurrent, short, RESERVED_INDEX_MOTOR_CURRENT)

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

	void create(const Desc& desc);
	PISASoftHand(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_PISA_PISASOFTHAND_H_*/
