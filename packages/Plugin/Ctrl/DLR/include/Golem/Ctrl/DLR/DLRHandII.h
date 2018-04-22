/** @file DLRHandII.h
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
#ifndef _GOLEM_CTRL_DLR_DLRHandII_H_
#define _GOLEM_CTRL_DLR_DLRHandII_H_

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
class GOLEM_LIBRARY_DECLDIR DLRHandIIChain : public Chain {
public:
	/** Number of the finger joints */
	static const U32 NUM_JOINTS = 4;
	/** Number of the controllable finger joints */
	static const U32 NUM_JOINTS_CTRL = 3;

	/** DLRHandIIChain description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Chain::Desc {
	public:
		/** Links lengths */
		Real L0, L1, L2; // [m]

		Desc() {
			setToDefault();
		}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(DLRHandIIChain, Chain::Ptr, Controller&)

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
	DLRHandIIChain(Controller& controller);

public:
	// Inverse and forward transforms
	virtual void chainForwardTransform(const Real* cc, Mat34& trn) const;
	virtual void jointForwardTransform(const Real* cc, Mat34* trn) const;
	virtual void velocitySpatial(const Real* cc, const Real* dcc, Twist& vs) const;
	virtual void jacobianSpatial(const Real* cc, Twist* jac) const;
};

//------------------------------------------------------------------------------

/** DLRHandII controller base class. */
class GOLEM_LIBRARY_DECLDIR DLRHandII: public SingleCtrl {
public:
	typedef ScalarCoord<bool, Chainspace> ChainspaceCoordBool;

	/** Number of fingers */
	static const U32 NUM_CHAINS = 4;
	/** Total number of joints */
	static const U32 NUM_JOINTS = NUM_CHAINS * DLRHandIIChain::NUM_JOINTS;
	/** Total number of controllable joints */
	static const U32 NUM_JOINTS_CTRL = NUM_CHAINS * DLRHandIIChain::NUM_JOINTS_CTRL;

	/** DLRHandII description */
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
			for (U32 i = 0; i < NUM_CHAINS; ++i)
				chains.push_back(Chain::Desc::Ptr(new DLRHandIIChain::Desc));
		}
		
		virtual bool isValid() const {
			if (!SingleCtrl::Desc::isValid())
				return false;

			if (chains.size() != NUM_CHAINS)
				return false;
			for (Chain::Desc::Seq::const_iterator i = chains.begin(); i != chains.end(); ++i)
				if (dynamic_cast<const DLRHandIIChain::Desc*>(i->get()) == NULL)
					return false;
			
			return true;
		}
	};

	/** Sets to default a given state. */
	virtual void setToDefault(State& state) const;

	/** Returns reserved data offset. */
	virtual ptrdiff_t getReservedOffset(U32 type) const;

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

	DLRHandII(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_DLR_DLRHandII_H_*/
