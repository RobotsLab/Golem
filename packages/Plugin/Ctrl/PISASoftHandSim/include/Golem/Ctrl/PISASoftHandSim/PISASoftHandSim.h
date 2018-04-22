/** @file PISASoftHandSim.h
 * 
 * PISA Soft Hand simulator
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
#ifndef _GOLEM_CTRL_PISASOFTHANDSIM_PISASOFTHANDSIM_H_
#define _GOLEM_CTRL_PISASOFTHANDSIM_PISASOFTHANDSIM_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/PISA/PISASoftHand.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR PISASoftHandSim : public PISASoftHand {
public:
	/** DLRHitHandIISim description */
	class GOLEM_LIBRARY_DECLDIR Desc : public PISASoftHand::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(PISASoftHandSim, Controller::Ptr, Context&)

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			PISASoftHand::Desc::setToDefault();

			name = "PISA Soft Hand simulator";
			
			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.005);
			simDeltaSend = SecTmReal(0.005);
		}

		virtual bool isValid() const {
			if (!PISASoftHand::Desc::isValid())
				return false;

			return true;
		}
	};

	/** Release resources */
	virtual ~PISASoftHandSim();

protected:
	/** Receives device state. */
	virtual void sysRecv(State& state);

	PISASoftHandSim(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_PISASOFTHANDSIM_PISASOFTHANDSIM_H_*/
