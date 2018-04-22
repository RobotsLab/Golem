/** @file SixAxisSim.h
 * 
 * 6-axis arm simulator
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
#ifndef _GOLEM_CTRL_SIXAXISSIM_SIXAXISSIM_H_
#define _GOLEM_CTRL_SIXAXISSIM_SIXAXISSIM_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/SingleCtrl/SingleCtrl.h>
#include <Golem/Sys/Library.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

//
//            t1    t2                t3         t4    t5       
//            ^    ^                 ^           ^ Z  ^         
//            |   /                 /            |   /          
//            |  /                 /             |  /           
//            | /                 /              | /            
//            |/       l1        /       l2      |/       Y     
//            O*****************O****************O---------> t6 
//           /*                /                /|              
//          / *               /                / | T            
//         /  *              /                /  |              
//        /   * l0          /                v X |              
//            *                                                 
//            *^ Z                                              
//            *|                                                
//            *|                                                
//            *| S     Y                                        
//             O-------->                                       
//            /                                                 
//           /                                                  
//          /                                                   
//         v X                                                   
//

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR SixAxisChainSim : public Chain {
public:	
	/** Number of the arm joints */
	static const U32 NUM_JOINTS = 6;

	/** SixAxisSim description */
	GOLEM_LIBRARY_DECLDIR class Desc : public Chain::Desc {
	public:
		/** Links lengths */
		Real L0; // [m]
		Real L1; // [m]
		Real L2; // [m]
		Real L3; // [m]

		Desc() {
			setToDefault();
		}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(SixAxisChainSim, Chain::Ptr, Controller&)

		virtual void setToDefault() {
			Chain::Desc::setToDefault();

			L0 = Real(0.2); // [m]
			L1 = Real(0.2); // [m]
			L2 = Real(0.2); // [m]
			L3 = Real(0.05); // [m]

			referencePose.p.v2 += L3;

			joints.clear();
			for (U32 i = 0; i < NUM_JOINTS; ++i)
				joints.push_back(Joint::Desc::Ptr(new Joint::Desc));
		}
		
		virtual bool isValid() const {
			if (!Chain::Desc::isValid())
				return false;

			if (joints.size() != NUM_JOINTS)
				return false;
			if (L0 <= REAL_ZERO || L1 <= REAL_ZERO || L2 <= REAL_ZERO || L3 <= REAL_ZERO)
				return false;

			return true;
		}
	};

protected:
	/** Links lengths */
	Real L0; // [m]
	Real L1; // [m]
	Real L2; // [m]
	Real L3; // [m]

	/** Initialisation */
	void create(const Desc& desc);
	SixAxisChainSim(Controller& controller);

public:
	virtual void chainForwardTransform(const Real* cc, Mat34& trn) const;
	virtual void velocitySpatial(const Real* cc, const Real* dcc, Twist& vs) const;
	virtual void jacobianSpatial(const Real* cc, Twist* jac) const;
};

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR SixAxisSim: public SingleCtrl {
public:	
	/** SixAxisSim description */
	class GOLEM_LIBRARY_DECLDIR Desc : public SingleCtrl::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(SixAxisSim, Controller::Ptr, Context&)

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			SingleCtrl::Desc::setToDefault();

			cycleDurationCtrl = true;
			timeQuant = SecTmReal(0.0001);
			cycleDurationInit = SecTmReal(0.2);
			cycleDurationOffs = SecTmReal(0.01);
			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.02);
			simDeltaSend = SecTmReal(0.02);

			name = "6-axis arm simulator";

			chains.clear();
			chains.push_back(Chain::Desc::Ptr(new SixAxisChainSim::Desc));
		}
		
		virtual bool isValid() const {
			if (!SingleCtrl::Desc::isValid())
				return false;

			if (chains.size() != 1)
				return false;
			if (dynamic_cast<const SixAxisChainSim::Desc*>(chains.begin()->get()) == NULL)
				return false;

			return true;
		}
	};
	
	/** Release resources */
	virtual ~SixAxisSim();

protected:
	SixAxisSim(Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_SIXAXISSIM_SIXAXISSIM_H_*/
