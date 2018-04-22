/** @file Katana.h
 * 
 * Katana (6M180)
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
#ifndef _GOLEM_CTRL_KATANA_KATANA_H_
#define _GOLEM_CTRL_KATANA_KATANA_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Controller.h>
#include <Golem/Sys/Library.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

//
//            t1    t2                t3               t4       
//            ^    ^                 ^           ^ Z  ^         
//            |   /                 /            |   /          
//            |  /                 /             |  /           
//            | /                 /              | /            
//            |/       l1        /       l2      |/       Y     
//            O*****************O****************O---------> t5 
//           /*                /                /               
//          / *               /                /   T            
//         /  *              /                /                  
//        /   * l0          /                V X                  
//            *                                                 
//            *^ Z                                              
//            *|                                                
//            *|                                                
//            *| S     Y                                        
//             O-------->                                       
//            /                                                 
//           /                                                  
//          / X                                                 
//         v                                                    
//

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR KatanaJoint : public Joint {
public:
	/** Katana trajectory time quant [sec] */
	static const SecTmReal TRJ_TIME_QUANT;

	/** Katana Joint description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Joint::Desc {
	public:
		/** KNI angleOffset [deg] */
		Real angleOffset;
		/** KNI angleRange [deg] */
		Real angleRange;
		/** KNI encoderOffset */
		I32 encoderOffset;
		/** KNI encodersPerCycle */
		I32 encodersPerCycle;
		/** KNI rotationDirection {+1=DIR_POSITIVE, -1=DIR_NEGATIVE} */
		I32 rotationDirection;
		/** KNI encoderPositionAfter */
		I32 encoderPositionAfter;
		/** Rescaling offset [rad] */
		Real offset;
		/** Rescaling gain */
		Real gain;
		
		Desc() {
			Desc::setToDefault();
		}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(KatanaJoint, Joint::Ptr, Chain&)

		virtual void setToDefault() {
			Joint::Desc::setToDefault();

			angleOffset = Real(0.0);
			angleRange = Real(360.0);
			encoderOffset = 10000;
			encodersPerCycle = 100000;
			rotationDirection = 1;
			encoderPositionAfter = 11000;
			offset = Real(0.0);
			gain = 1.0;
		}

		virtual bool isValid() const {
			if (!Joint::Desc::isValid())
				return false;

			if (Math::isNegative(angleOffset) || Math::isZero(angleRange))
				return false;
			if (encoderOffset == encoderPositionAfter || encodersPerCycle <= 0 || !(rotationDirection == 1 || rotationDirection == -1))
				return false;
			if (Math::isZero(gain))
				return false;

			return true;
		}
	};

private:
	/** KNI min position [rad] */
	Real kniMinPos;
	/** KNI max position [rad] */
	Real kniMaxPos;

protected:
	/** KNI angleOffset [deg] */
	Real angleOffset;
	/** KNI angleRange [deg] */
	Real angleRange;
	/** KNI encoderOffset */
	I32 encoderOffset;
	/** KNI encodersPerCycle */
	I32 encodersPerCycle;
	/** KNI rotationDirection {+1=DIR_POSITIVE, -1=DIR_NEGATIVE} */
	I32 rotationDirection;
	/** KNI encoderPositionAfter */
	I32 encoderPositionAfter;
	/** Rescaling offset [rad] */
	Real offset;
	/** Rescaling gain */
	Real gain;

	/** Min [rad] */
	Real getKNIMinPos() const;
	/** Max [rad] */
	Real getKNIMaxPos() const;

	/** Creates Katana Joint from the description. */
	void create(const Desc& desc);
	KatanaJoint(Chain& chain);

public:
	/** Encoder position -> position in radians */
	virtual Real posFromEnc(I32 pos) const;
	/** Position in radians -> encoder position */
	virtual I32 posToEnc(Real pos) const;
	/** Encoder velocity -> velocity in radians/sec */
	virtual Real velFromEnc(I32 vel) const;
	/** Velocity in radians/sec -> encoder velocity */
	virtual I32 velToEnc(Real vel) const;
};

//------------------------------------------------------------------------------

/** Katana 300/450 base class (6M180) */
class GOLEM_LIBRARY_DECLDIR KatanaChain : public Chain {
public:
	/** Number of the manipulator joints (without a gripper) */
	static const U32 NUM_JOINTS = 5;

	/** KatanaChain description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Chain::Desc {
	public:
		/** Links lengths */
		Real L0, L1, L2, L3; // [m]

		Desc() {
			setToDefault();
		}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(KatanaChain, Chain::Ptr, Controller&)

		virtual void setToDefault() {
			Chain::Desc::setToDefault();

			L0 = Real(0.2035); // [m]
			L1 = Real(0.1902); // [m]
			L2 = Real(0.1391); // [m]
			L3 = Real(0.1916); // [m]

			referencePose.p.v2 += L3;

			joints.clear();
			for (U32 i = 0; i < NUM_JOINTS; ++i)
				joints.push_back(Joint::Desc::Ptr(new KatanaJoint::Desc));
		}

		virtual bool isValid() const {
			if (!Chain::Desc::isValid())
				return false;

			if (L0 <= REAL_ZERO || L1 <= REAL_ZERO || L2 <= REAL_ZERO || L3 <= REAL_ZERO)
				return false;

			if (joints.size() != NUM_JOINTS)
				return false;
			for (Joint::Desc::Seq::const_iterator i = joints.begin(); i != joints.end(); ++i)
				if (dynamic_cast<const KatanaJoint::Desc*>(i->get()) == NULL)
					return false;

			return true;
		}
	};

protected:
	/** Links lengths */
	Real L0, L1, L2, L3; // [m]

	// Initialisation
	void create(const Desc& desc);
	KatanaChain(Controller& controller);

public:
	// Inverse and forward transforms
	virtual void chainForwardTransform(const Real* cc, Mat34& trn) const;
	virtual void velocitySpatial(const Real* cc, const Real* dcc, Twist& vs) const;
	virtual void jacobianSpatial(const Real* cc, Twist* jac) const;
};

//------------------------------------------------------------------------------

/** Katana 300/450 gripper interface */
class GOLEM_LIBRARY_DECLDIR KatanaGripper {
public:
	/** Katana sensor data */
	class GOLEM_LIBRARY_DECLDIR SensorData {
	public:
		/** Sensor index */
		I32 index;
		/** Sensor value */
		I32 value;

		SensorData(I32 index = 0, I32 value = 0) : index(index), value(value) {}
		SensorData(const SensorData& sensorData) : index(sensorData.index), value(sensorData.value) {}
	};

	/** Katana gripper encoder data */
	class GOLEM_LIBRARY_DECLDIR GripperEncoderData {
	public:
		/** Open gripper encoder value */
		I32 open;
		/** Closed gripper encoder value */
		I32 closed;
		/** Current gripper encoder value */
		I32 current;
	};

	typedef std::vector<SensorData> SensorDataSet;
	typedef std::vector<I32> SensorIndexSet;

	/** KatanaChain description */
	class GOLEM_LIBRARY_DECLDIR Desc {
	public:
		/** Katana gripper */
		bool bGripper;
		/** Katana sensors */
		SensorIndexSet sensorIndexSet;

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			bGripper = false;

			sensorIndexSet.clear();
			// Katana Finger Type S03.02, force sensors
			sensorIndexSet.push_back(7);	// Right finger, Front
			sensorIndexSet.push_back(15);	// Left finger, Front
			sensorIndexSet.push_back(6);	// Right finger, Rear
			sensorIndexSet.push_back(14);	// Left finger, Rear
		}

		virtual bool isValid() const {
			if (bGripper && sensorIndexSet.empty())
				return false;

			return true;
		}
	};

private:
	Context* pContext;

protected:
	/** Katana gripper */
	bool bGripper;
	/** Katana sensors */
	SensorIndexSet sensorIndexSet;

	// Initialisation
	void create(const Desc& desc);
	KatanaGripper(Context* context);

public:
	virtual ~KatanaGripper();

	/** Receives gripper sensor values */
	virtual bool gripperRecvSensorData(SensorDataSet& sensorData, MSecTmU32 timeWait = MSEC_TM_U32_INF) = 0;
	
	/** Receives gripper encoder values */
	virtual bool gripperRecvEncoderData(GripperEncoderData& encoderData, MSecTmU32 timeWait = MSEC_TM_U32_INF) = 0;
	
	/** Opens the gripper */
	virtual bool gripperOpen(MSecTmU32 timeWait = MSEC_TM_U32_INF) = 0;

	/** Closes the gripper, stops if only a signal from any sensor is above the given threshold */
	virtual bool gripperClose(const SensorDataSet& sensorThreshold, MSecTmU32 timeWait = MSEC_TM_U32_INF) = 0;
	
	/** Freezes the gripper */
	virtual bool gripperFreeze(MSecTmU32 timeWait = MSEC_TM_U32_INF) = 0;

	/** Katana gripper */
	bool hasGripper() const {
		return bGripper;
	}

	/** Returns Katana sensor indexes */
	const SensorIndexSet& getSensorIndexSet() const {
		return sensorIndexSet;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_KATANA_KATANA_H_*/
