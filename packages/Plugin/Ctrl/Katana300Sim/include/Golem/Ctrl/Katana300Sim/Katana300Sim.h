/** @file Katana300Sim.h
 * 
 * Katana 300 (6M180) simulator.
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
#ifndef _GOLEM_CTRL_KATANA300SIM_KATANA300SIM_H_
#define _GOLEM_CTRL_KATANA300SIM_KATANA300SIM_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Katana/Katana.h>
#include <Golem/Ctrl/SingleCtrl/SingleCtrl.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR Katana300Sim : public SingleCtrl, public KatanaGripper {
public:
	/** Katana300Sim description */
	class GOLEM_LIBRARY_DECLDIR Desc : public SingleCtrl::Desc, public KatanaGripper::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(Katana300Sim, Controller::Ptr, Context&)

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			SingleCtrl::Desc::setToDefault();
			KatanaGripper::Desc::setToDefault();

			name = "Katana 300 (6M180) Simulator";
			
			cycleDurationCtrl = true;
			timeQuant = SecTmReal(0.01);
			cycleDurationInit = SecTmReal(0.2);
			cycleDurationOffs = SecTmReal(0.03);
			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.02);
			simDeltaSend = SecTmReal(0.05);

			chains.clear();
			chains.push_back(Chain::Desc::Ptr(new KatanaChain::Desc));
		}

		virtual bool isValid() const {
			if (!SingleCtrl::Desc::isValid() || !KatanaGripper::Desc::isValid())
				return false;

			if (chains.size() != 1)
				return false;
			if (dynamic_cast<const KatanaChain::Desc*>(chains.begin()->get()) == NULL)
				return false;
			
			return true;
		}
	};

	/** Release resources */
	virtual ~Katana300Sim();
	/** Receives gripper sensor values */
	virtual bool gripperRecvSensorData(SensorDataSet& sensorData, MSecTmU32 timeWait = MSEC_TM_U32_INF);
	/** Receives gripper encoder values */
	virtual bool gripperRecvEncoderData(GripperEncoderData& encoderData, MSecTmU32 timeWait = MSEC_TM_U32_INF);
	/** Opens the gripper */
	virtual bool gripperOpen(MSecTmU32 timeWait = MSEC_TM_U32_INF);
	/** Closes the gripper, stops if only a signal from any sensor is above the given threshold */
	virtual bool gripperClose(const SensorDataSet& sensorThreshold, MSecTmU32 timeWait = MSEC_TM_U32_INF);
	/** Freezes the gripper */
	virtual bool gripperFreeze(MSecTmU32 timeWait = MSEC_TM_U32_INF);

protected:
	/** Controller initialisation */
	void create(const Desc& desc);
	Katana300Sim(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_KATANA300SIM_KATANA300SIM_H_*/
