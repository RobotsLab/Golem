/** @file KukaIIWA.h
 *
 * KukaIIWA-family arm base class
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
#ifndef _GOLEM_CTRL_KUKA_KUKAIIWA_H_
#define _GOLEM_CTRL_KUKA_KUKAIIWA_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/SingleCtrl/SingleCtrl.h>
#include <Golem/Sys/Library.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** KukaIIWA kinematic chain base class */
class GOLEM_LIBRARY_DECLDIR KukaIIWAChain : public Chain {
public:
	/** Number of the manipulator joints */
	static const U32 NUM_JOINTS = 7;

	/** KukaIIWA Chain model */
	class GOLEM_LIBRARY_DECLDIR ChainModel {
	public:
		/** Links lengths */
//		Real L0, L1, L2, L3, L4, L5, L6, L7; // [m]
		Real L0, L2, L4, L7; // [m]
		/** Encoder offsets */
		Real encoderOffset[NUM_JOINTS];

		ChainModel() {
			setToDefault();
		}
		void setToDefault() {
			L0  = Real(0.200); // [m]
//			L1 = REAL_ZERO; // [m]
			L2 = Real(0.400); // [m]
//			L3 = REAL_ZERO; // [m]
			L4 = Real(0.390); // [m]
//			L5  = REAL_ZERO; // [m]
//			L6 = REAL_ZERO; // [m]
			L7  = Real(0.1041); // [m]
			std::fill(encoderOffset, encoderOffset + NUM_JOINTS, REAL_ZERO);
		}
		bool isValid() const {
			if (L0 <= REAL_ZERO || L2 <= REAL_ZERO || L4 <= REAL_ZERO || L7 <= REAL_ZERO)
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

	/** KukaIIWAChain description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Chain::Desc {
	public:
		/** KukaIIWA Chain model */
		ChainModel chainModel;

		Desc() {
			setToDefault();
		}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(KukaIIWAChain, Chain::Ptr, Controller&)

		virtual void setToDefault() {
			Chain::Desc::setToDefault();

			chainModel.setToDefault();

			referencePose.p.v2 += chainModel.L7;

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
			for (Joint::Desc::Seq::const_iterator i = joints.begin(); i != joints.end(); ++i) {
				if (dynamic_cast<const Joint::Desc*>(i->get()) == NULL)
					return false;
			}

			return true;
		}
	};

protected:
	/** KukaIIWA Chain model */
	ChainModel chainModel;

	/** exponential coordinates defining joint transformation */
	ExpCoord customTrn[NUM_JOINTS];
	/** exponential coordinates defining initial frame transformation */
	ExpCoord customTrnInit[NUM_JOINTS];

	// Initialisation
	void create(const Desc& desc);
	KukaIIWAChain(Controller& controller);

public:
	// Inverse and forward transforms
	virtual void chainForwardTransform(const Real* cc, Mat34& trn) const;
	virtual void jointForwardTransform(const Real* cc, Mat34* trn) const;
	virtual void jacobianSpatial(const Real* cc, Twist* jac) const;

	/** Sets KukaIIWA Chain model */
	void setChainModel(const ChainModel& chainModel);
	/** Current KukaIIWA Chain model */
	inline const ChainModel& getChainModel() const {
		return chainModel;
	}
};

//------------------------------------------------------------------------------

/** KukaIIWA controller base class. */
class GOLEM_LIBRARY_DECLDIR KukaIIWA: public SingleCtrl {
public:

	/** SE(3) vector dimensions */
	static const U32 SE3_DIM = 12;

	/** Twist/wrench vector dimensions */
	static const U32 TWIST_DIM = 6;

	/** KukaIIWA state reserved area variable indices */
	enum ReservedIndex {
		/**To FRI */
				RESERVED_INDEX_NEXT_CONTROL_MODE = Controller::RESERVED_INDEX_SIZE,
		/**To FRI */
				RESERVED_INDEX_JNTIMP_NEXT_POSITION,
		/**To FRI */
				RESERVED_INDEX_JNTIMP_ADD_TORQUE,
		/**To FRI */
				RESERVED_INDEX_CARTIMP_NEXT_POSITION,
		/**To FRI */
				RESERVED_INDEX_CARTIMP_STIFFNESS,
		/**To FRI */
				RESERVED_INDEX_CARTIMP_DAMPING,
		/**To FRI */
				RESERVED_INDEX_CARTIMP_ADD_TCP_FT,
		/**To FRI */
				RESERVED_INDEX_CARTIMP_JNT_NULL_SPACE,
		/**From FRI */
				RESERVED_INDEX_CURRENT_CONTROL_MODE,
		/**From FRI */
				RESERVED_INDEX_COM_QUALITY,
		/**From FRI */
				RESERVED_INDEX_ROBOT_STATE,
		/**From FRI */
				RESERVED_INDEX_ROBOT_POWER_IS_ON,
		/**  */
				RESERVED_INDEX_SIZE = RESERVED_INDEX_ROBOT_POWER_IS_ON + 1,
	};


	/** KukaIIWA description */
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
		}

		virtual bool isValid() const {
			if (!SingleCtrl::Desc::isValid())
				return false;

			return true;
		}
	};

	/** Sets to default a given state. */
	virtual void setToDefault(State& state) const;

	/** Returns reserved data offset. */
	virtual ptrdiff_t getReservedOffset(U32 type) const;

	/** Get the complete state of the system at time t. */
	virtual void lookupState(SecTmReal t, State &state) const;

	/** Interpolates the controller state at time t. */
	virtual void interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const;

	/** Reserved access */
	CONTROLLER_STATE_RESERVED(Torque, ConfigspaceCoord, RESERVED_INDEX_FORCE_TORQUE)
	CONTROLLER_STATE_RESERVED(NextControlMode, int, RESERVED_INDEX_NEXT_CONTROL_MODE)
	CONTROLLER_STATE_RESERVED(JntImpNextPos, ConfigspaceCoord, RESERVED_INDEX_JNTIMP_NEXT_POSITION)
	CONTROLLER_STATE_RESERVED(JntImpStiffness, ConfigspaceCoord, RESERVED_INDEX_STIFFNESS)
	CONTROLLER_STATE_RESERVED(JntImpDamping, ConfigspaceCoord, RESERVED_INDEX_DAMPING)
	CONTROLLER_STATE_RESERVED(JntImpAddTorque, ConfigspaceCoord, RESERVED_INDEX_JNTIMP_ADD_TORQUE)
	CONTROLLER_STATE_RESERVED(CartImpNextPos, golem::Mat34, RESERVED_INDEX_CARTIMP_NEXT_POSITION)
	CONTROLLER_STATE_RESERVED(CartImpStiffness, golem::Twist, RESERVED_INDEX_CARTIMP_STIFFNESS)
	CONTROLLER_STATE_RESERVED(CartImpDamping, golem::Twist, RESERVED_INDEX_CARTIMP_DAMPING)
	CONTROLLER_STATE_RESERVED(CartImpAddTcpFT, golem::Twist, RESERVED_INDEX_CARTIMP_ADD_TCP_FT)
	CONTROLLER_STATE_RESERVED(CartImpJntNullSpace, ConfigspaceCoord, RESERVED_INDEX_CARTIMP_JNT_NULL_SPACE)
	CONTROLLER_STATE_RESERVED(CurrentControlMode, int, RESERVED_INDEX_CURRENT_CONTROL_MODE)
	CONTROLLER_STATE_RESERVED(ComQuality, int, RESERVED_INDEX_COM_QUALITY)
	CONTROLLER_STATE_RESERVED(RobotState, int, RESERVED_INDEX_ROBOT_STATE)
	CONTROLLER_STATE_RESERVED(PowerIsOn, int, RESERVED_INDEX_ROBOT_POWER_IS_ON)


protected:
	/** Reserved area size */
	static const ReservedOffset RESERVED_OFFSET[RESERVED_INDEX_SIZE];

	/** Reserved data offset: type -> offset mapping */
	std::vector<ptrdiff_t> reservedOffset;

	/** Sets controller state properties */
	virtual void setStateInfo(const Controller::Desc& desc);

	KukaIIWA(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_KUKA_KUKAIIWA_H_*/
