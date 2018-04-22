/** @file PISASoftHandSerial.h
 *
 * PISA Soft Hand controller
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 * @copyright  Copyright (c) 2012, qbrobotics.
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_PISASOFTHAND_PISASOFTHAND_H_
#define _GOLEM_CTRL_PISASOFTHAND_PISASOFTHAND_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/PISA/PISASoftHand.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

struct comm_settings;

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR PISASoftHandSerial : public PISASoftHand {
public:
	/** PISASoftHandSerial description */
	class GOLEM_LIBRARY_DECLDIR Desc : public PISASoftHand::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(PISASoftHandSerial, Controller::Ptr, Context&)

        /** USB port */
        std::string serialPort;
		/** Motor file */
		std::string motorFile;

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			PISASoftHand::Desc::setToDefault();

			name = "PISA Soft Hand controller";
#ifdef WIN32
			serialPort = "USB0";
#else
			serialPort = "tty/USB0";
#endif
			motorFile = "PISASoftHandMotor.conf";
		}

		virtual bool isValid() const {
			if (!PISASoftHand::Desc::isValid())
				return false;

			if (serialPort.length() <= 0 || motorFile.length() <= 0)
				return false;

			return true;
		}
	};

	/** Release resources */
	virtual ~PISASoftHandSerial();

protected:
	/** USB port */
	std::string serialPort;
	/** Motor file */
	std::string motorFile;

	/** Current device state */
	State::Ptr state;

    /** I/O interface */
	golem::shared_ptr<comm_settings, reference_cnt_vt_base<comm_settings> > comm;

	//----------------------------------------------------------------------
	/*!
	Send the desired Joints Position commands

	\param position     - desired position value

	\details
	position - Percentage of closure of the hand
	*/
	void setMotorPosition(short int position);

	//----------------------------------------------------------------------
	/*!
	Get the current motor position

	\param position     - current position value
	*/
	void getMotorPosition(short& position);


	//----------------------------------------------------------------------
	/*!
	Get the motor current

	\param position     - current value
	*/
	void getMotorCurrent(short& current);

	//----------------------------------------------------------------------
	/*!
	Set the zero position fo the motor

	\details
	It is important to have the hand completely open before calling that function

	\return true if succeed
	*/
	void resetHand();

	virtual void sysSync();
	virtual void sysRecv(State& state);
	virtual void sysSend(const State& prev, const State& next, bool bSendPrev, bool bSendNext);

	virtual void userCreate();
	void create(const Desc& desc);

	PISASoftHandSerial(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_PISASOFTHAND_PISASOFTHAND_H_*/
