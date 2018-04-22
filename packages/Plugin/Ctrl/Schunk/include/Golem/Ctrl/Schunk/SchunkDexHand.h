/** @file SchunkDexHand.h
 *
 * Schunk Dextrous Hand base classes
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
#ifndef _GOLEM_CTRL_SCHUNK_SCHUNKDEXHAND_H_
#define _GOLEM_CTRL_SCHUNK_SCHUNKDEXHAND_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/SingleCtrl/SingleCtrl.h>
#include <Golem/Sys/Library.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

//
//            t1    t2                t3               t4
//            ^ Z  ^                 ^           ^ Z  ^
//            |   /                 /            |   /
//            |  /                 /             |  /
//            | /                 /              | /
//            |/      Y    l0    /       l1      |/       Y
//            O-------->********O****************O--------->
//           /                 /                /
//          /                 /                /  T
//         /                 /                /
//        v X               /                v X
//
//

//------------------------------------------------------------------------------

/** Schunk Dextrous Hand controller base class. */
class GOLEM_LIBRARY_DECLDIR SchunkDexHand: public SingleCtrl {
public:
	typedef ScalarCoord<bool, Chainspace> ChainspaceCoordBool;

	/** Number of fingers */
	static const U32 NUM_CHAINS = 3;
	/** Number of joints */
	static const U32 NUM_JOINTS_MIDDLE = 2;
	/** Number of joints */
	static const U32 NUM_JOINTS_LEFT = 3;
	/** Number of joints */
	static const U32 NUM_JOINTS_RIGHT = 3;

	/** Finger chain index */
	static const U32 LEFT_FINGER_CHAIN_INDEX = 1;
	/** Finger chain index */
	static const U32 MIDDLE_FINGER_CHAIN_INDEX = 0;
	/** Finger chain index */
	static const U32 RIGHT_FINGER_CHAIN_INDEX = 2;

	/** Number of joints */
	static const U32 CHAIN_NUM_JOINTS[NUM_CHAINS];
	/** Number of joints */
	static const U32 NUM_JOINTS = NUM_JOINTS_MIDDLE + NUM_JOINTS_LEFT + NUM_JOINTS_RIGHT;
	/** Number of controllable joints */
	static const U32 NUM_JOINTS_CTRL = NUM_JOINTS - 1;

	/** SchunkDexHand description */
	class GOLEM_LIBRARY_DECLDIR Desc : public SingleCtrl::Desc {
	public:
		Desc() {
			setToDefault();
		}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(SchunkDexHand, Controller::Ptr, golem::Context&);

		virtual void setToDefault() {
			SingleCtrl::Desc::setToDefault();

			reservedSize = (U32)ReservedOffset::getSize(RESERVED_OFFSET, RESERVED_OFFSET + RESERVED_INDEX_SIZE);

			cycleDurationCtrl = false;
			timeQuant = SecTmReal(0.0001);
			cycleDurationInit = SecTmReal(0.1);

			for (size_t i = 0; i < NUM_CHAINS; ++i) {
				chains.push_back(Chain::Desc::Ptr(new Chain::Desc));
				for (U32 j = 0; j < CHAIN_NUM_JOINTS[i]; ++j){
					chains.back()->joints.push_back(Joint::Desc::Ptr(new Joint::Desc));
				}
			}
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
	CONTROLLER_STATE_RESERVED(Torque, ConfigspaceCoord, RESERVED_INDEX_FORCE_TORQUE)
	CONTROLLER_STATE_RESERVED(Stiffness, ConfigspaceCoord, RESERVED_INDEX_STIFFNESS)
	CONTROLLER_STATE_RESERVED(Damping, ConfigspaceCoord, RESERVED_INDEX_DAMPING)

protected:
	/** Reserved area size */
	static const ReservedOffset RESERVED_OFFSET[RESERVED_INDEX_SIZE];

	/** Reserved data offset: type -> offset mapping */
	std::vector<ptrdiff_t> reservedOffset;

	/** Sets controller state properties */
	virtual void setStateInfo(const Controller::Desc& desc);

	SchunkDexHand(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_SCHUNK_SCHUNKDEXHAND_H_*/
