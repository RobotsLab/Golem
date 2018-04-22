/** @file KukaKR5Sixx.h
 * 
 * KukaKR5Sixx-family arm base class
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
#ifndef _GOLEM_CTRL_KUKA_KUKAKR5SIXX_H_
#define _GOLEM_CTRL_KUKA_KUKAKR5SIXX_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/SingleCtrl/SingleCtrl.h>
#include <Golem/Sys/Library.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

//                                                               
//                                                                  t5
//                                                                  ^
//            t1       t2                t3                        /
//            |       ^                 ^                         /
//            |      /                 /                         /
//            |     /                 /     l21         t6 & t4 /   l3
//            v    /               ********************<-------O********
//             l10/       l11   l20*/                         /   
//            ***O*****************O                         /    
//            * /                 /                         /     
//			  */                 /                         /      
//            /                 /                                 
//           /* l0             /                                 
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

/** KukaKR5Sixx kinematic chain base class */
class GOLEM_LIBRARY_DECLDIR KukaKR5SixxChain : public Chain {
public:
	/** Number of the manipulator joints */
	static const U32 NUM_JOINTS = 6;

	/** KukaKR5Sixx Chain model */
	class GOLEM_LIBRARY_DECLDIR ChainModel {
	public:
		/** Links lengths */
		Real L0, L10, L11, L20, L21, L3; // [m]
		/** Encoder offsets */
		Real encoderOffset[NUM_JOINTS];

		ChainModel() {
			setToDefault();
		}
		void setToDefault() {
			L0  = Real(0.335); // [m]
			L10 = Real(0.075); // [m]
			L11 = Real(0.365); // [m]
			L20 = Real(0.09); // [m]
			L21 = Real(0.405); // [m]
			L3  = Real(0.08); // [m]
			std::fill(encoderOffset, encoderOffset + NUM_JOINTS, REAL_ZERO);
		}
		bool isValid() const {
			if (L0 <= REAL_ZERO || L10 <= REAL_ZERO || L11 <= REAL_ZERO || L20 <= REAL_ZERO || L21 <= REAL_ZERO || L3 <= REAL_ZERO)
				return false;
			for (size_t i = 0; i < NUM_JOINTS; ++i)
				if (!Math::isFinite(encoderOffset[i]))
					return false;
			return true;
		}

		void create();
		void chainForwardTransform(const Mat34& localPose, const Real* cc, Mat34& trn) const;
		void jointForwardTransform(const Mat34& localPose, const Real* cc, Mat34* trn) const;
		void jacobianSpatial(const Real* cc, Twist* jac) const;

	protected:
		/** exponential coordinates defining joint transformation */
		ExpCoord trnCoord[NUM_JOINTS];
		/** exponential coordinates defining initial frame transformation */
		ExpCoord trnInitCoord[NUM_JOINTS];
	};

	/** KukaKR5SixxChain description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Chain::Desc {
	public:
		/** KukaKR5Sixx Chain model */
		ChainModel chainModel;

		Desc() {
			setToDefault();
		}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(KukaKR5SixxChain, Chain::Ptr, Controller&)

		virtual void setToDefault() {
			Chain::Desc::setToDefault();

			chainModel.setToDefault();

			referencePose.p.v2 += chainModel.L3;

			joints.clear();
			for (U32 i = 0; i < NUM_JOINTS; ++i)
				joints.push_back(Joint::Desc::Ptr(new Joint::Desc));
		}

		virtual bool isValid() const {
			if (!Chain::Desc::isValid())
				return false;

			if (!chainModel.isValid())
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
	/** KukaKR5Sixx Chain model */
	ChainModel chainModel;

	/** exponential coordinates defining joint transformation */
	ExpCoord customTrn[NUM_JOINTS];
	/** exponential coordinates defining initial frame transformation */
	ExpCoord customTrnInit[NUM_JOINTS];

	// Initialisation
	void create(const Desc& desc);
	KukaKR5SixxChain(Controller& controller);

public:
	// Inverse and forward transforms
	virtual void chainForwardTransform(const Real* cc, Mat34& trn) const;
	virtual void jointForwardTransform(const Real* cc, Mat34* trn) const;
	virtual void jacobianSpatial(const Real* cc, Twist* jac) const;

	/** Sets KukaKR5Sixx Chain model */
	void setChainModel(const ChainModel& chainModel);
	/** Current KukaKR5Sixx Chain model */
	inline const ChainModel& getChainModel() const {
		return chainModel;
	}
};

//------------------------------------------------------------------------------

/** KukaKR5Sixx controller base class. */
class GOLEM_LIBRARY_DECLDIR KukaKR5Sixx: public SingleCtrl {
public:
	/** KukaKR5Sixx description */
	class GOLEM_LIBRARY_DECLDIR Desc : public SingleCtrl::Desc {
	public:
		Desc() {
			setToDefault();
		}
		
		virtual void setToDefault() {
			SingleCtrl::Desc::setToDefault();

			cycleDurationCtrl = false;
			timeQuant = SecTmReal(0.0001);
			cycleDurationInit = SecTmReal(0.1);
		}
		
		virtual bool isValid() const {
			if (!SingleCtrl::Desc::isValid())
				return false;

			return true;
		}
	};

protected:
	/** Interpolates the controller state at time t. */
	virtual void interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const;

	KukaKR5Sixx(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_KUKA_KUKAKR5SIXX_H_*/
