/** @file DLRHitHandII.h
 * 
 * DLR Hit Hand II base classes
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
#ifndef _GOLEM_CTRL_DLR_DLRHITHANDII_H_
#define _GOLEM_CTRL_DLR_DLRHITHANDII_H_

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

/** DLR Hit Hand finger base class */
class GOLEM_LIBRARY_DECLDIR DLRHitHandIIChain : public Chain {
public:
	/** Number of the finger joints */
	static const U32 NUM_JOINTS = 4;
	/** Number of the controllable finger joints */
	static const U32 NUM_JOINTS_CTRL = 3;

	/** DLRHitHandIIChain description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Chain::Desc {
	public:
		/** Links lengths */
		Real L0, L1, L2; // [m]

		Desc() {
			setToDefault();
		}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(DLRHitHandIIChain, Chain::Ptr, Controller&)

		virtual void setToDefault() {
			Chain::Desc::setToDefault();

			L0 = Real(0.055); // [m]
			L1 = Real(0.025); // [m]
			L2 = Real(0.025); // [m]

			referencePose.p.v2 += L2;

			joints.clear();
			for (U32 i = 0; i < NUM_JOINTS; ++i)
				joints.push_back(Joint::Desc::Ptr(new Joint::Desc));
		}

		virtual bool isValid() const {
			if (!Chain::Desc::isValid())
				return false;

			if (L0 <= REAL_ZERO || L1 <= REAL_ZERO || L2 <= REAL_ZERO)
				return false;

			if (joints.size() != NUM_JOINTS)
				return false;
			for (Joint::Desc::Seq::const_iterator i = joints.begin(); i != joints.end(); ++i)
				if (dynamic_cast<const Joint::Desc*>(i->get()) == NULL)
					return false;

			return true;
		}
	};

protected:
	/** Links lengths */
	Real L0, L1, L2; // [m]

	// Initialisation
	void create(const Desc& desc);
	DLRHitHandIIChain(Controller& controller);

public:
	// Inverse and forward transforms
	virtual void chainForwardTransform(const Real* cc, Mat34& trn) const;
	virtual void jointForwardTransform(const Real* cc, Mat34* trn) const;
	virtual void velocitySpatial(const Real* cc, const Real* dcc, Twist& vs) const;
	virtual void jacobianSpatial(const Real* cc, Twist* jac) const;
};

//------------------------------------------------------------------------------

typedef ScalarCoord<bool, Chainspace> ChainspaceCoordBool;

/** DLRHitHandII controller base class. */
class GOLEM_LIBRARY_DECLDIR DLRHitHandII: public SingleCtrl {
public:

	/** Number of fingers */
	static const U32 NUM_CHAINS = 5;
	/** Total number of joints */
	static const U32 NUM_JOINTS = NUM_CHAINS * DLRHitHandIIChain::NUM_JOINTS;
	/** Total number of controllable joints */
	static const U32 NUM_JOINTS_CTRL = NUM_CHAINS * DLRHitHandIIChain::NUM_JOINTS_CTRL;

	/** DLRHitHandII state reserved area variable indices */
	enum ReservedIndex {
		/** Finger enable (out) (Chainspace, bool) */
		RESERVED_INDEX_ENABLE = Controller::RESERVED_INDEX_SIZE,
		/** Finger enabled (inp) (Chainspace, bool) */
		RESERVED_INDEX_ENABLED,
		/** Maximum joint velocity (inp, out) (Configspace, Real) */
		RESERVED_INDEX_VELOCITY_MAX,
		/** Kp (out) (Configspace, Real) */
		RESERVED_INDEX_KP,
		/** Emergency (out) (bool) */
		RESERVED_INDEX_EMERGENCY,
		/** Con mod (out) (int) */
		RESERVED_INDEX_CON_MOD,
		/** Brakestatus (inp) (int) */
		RESERVED_INDEX_BRAKESTATUS,
		/** Commstatus (inp) (int) */
		RESERVED_INDEX_COMMSTATUS,
		/** Handconfig (inp) (int) */
		RESERVED_INDEX_HANDCONFIG,
		/** Reserved Index size */
		RESERVED_INDEX_SIZE = RESERVED_INDEX_HANDCONFIG + 1,
	};

	/** DLRHitHandII description */
	class GOLEM_LIBRARY_DECLDIR Desc : public SingleCtrl::Desc {
	public:
		Desc() {
			setToDefault();
		}
		
		virtual void setToDefault() {
			SingleCtrl::Desc::setToDefault();

			reservedSize = (U32)ReservedOffset::getSize(RESERVED_OFFSET, RESERVED_OFFSET + RESERVED_INDEX_SIZE);

			cycleDurationCtrl = false;
			timeQuant = SecTmReal(0.0001);
			cycleDurationInit = SecTmReal(0.1);

			chains.clear();
			chains.push_back(Chain::Desc::Ptr(new DLRHitHandIIChain::Desc));
			chains.push_back(Chain::Desc::Ptr(new DLRHitHandIIChain::Desc));
			chains.push_back(Chain::Desc::Ptr(new DLRHitHandIIChain::Desc));
			chains.push_back(Chain::Desc::Ptr(new DLRHitHandIIChain::Desc));
			chains.push_back(Chain::Desc::Ptr(new DLRHitHandIIChain::Desc));
		}
		
		virtual bool isValid() const {
			if (!SingleCtrl::Desc::isValid())
				return false;

			if (chains.size() != NUM_CHAINS)
				return false;
			for (Chain::Desc::Seq::const_iterator i = chains.begin(); i != chains.end(); ++i)
				if (dynamic_cast<const DLRHitHandIIChain::Desc*>(i->get()) == NULL)
					return false;
			
			return true;
		}
	};

	/** Sets to default a given state. */
	virtual void setToDefault(State& state) const;

	/** Returns reserved data offset. */
	virtual ptrdiff_t getReservedOffset(U32 type) const;

	/** Clamp configuration. */
	virtual void clampConfig(GenConfigspaceCoord& config) const;

	/** Interpolates the controller state at time t. */
	virtual void interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const;

	/** Joint offset. */
	virtual void getOffset(Real* offset) const {}
	virtual void setOffset(const Real* offset) {}

	/** Reserved access */
	CONTROLLER_STATE_RESERVED(Torque, ConfigspaceCoord, RESERVED_INDEX_FORCE_TORQUE)
	CONTROLLER_STATE_RESERVED(Stiffness, ConfigspaceCoord, RESERVED_INDEX_STIFFNESS)
	CONTROLLER_STATE_RESERVED(Damping, ConfigspaceCoord, RESERVED_INDEX_DAMPING)
	CONTROLLER_STATE_RESERVED(Enable, ChainspaceCoordBool, RESERVED_INDEX_ENABLE)
	CONTROLLER_STATE_RESERVED(Enabled, ChainspaceCoordBool, RESERVED_INDEX_ENABLED)
	CONTROLLER_STATE_RESERVED(VelocityMax, ConfigspaceCoord, RESERVED_INDEX_VELOCITY_MAX)
	CONTROLLER_STATE_RESERVED(KP, ConfigspaceCoord, RESERVED_INDEX_KP)
	CONTROLLER_STATE_RESERVED(Emergency, bool, RESERVED_INDEX_EMERGENCY)
	CONTROLLER_STATE_RESERVED(ConMod, int, RESERVED_INDEX_CON_MOD)
	CONTROLLER_STATE_RESERVED(BrakeStatus, int, RESERVED_INDEX_BRAKESTATUS)
	CONTROLLER_STATE_RESERVED(CommStatus, int, RESERVED_INDEX_COMMSTATUS)
	CONTROLLER_STATE_RESERVED(HandConfig, int, RESERVED_INDEX_HANDCONFIG)

protected:
	/** Reserved area size */
	static const ReservedOffset RESERVED_OFFSET[RESERVED_INDEX_SIZE];

	/** Reserved data offset: type -> offset mapping */
	std::vector<ptrdiff_t> reservedOffset;

	/** Sets controller state properties */
	virtual void setStateInfo(const Controller::Desc& desc);

	DLRHitHandII(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_DLR_DLRHITHANDII_H_*/
