/** @file SchunkDexHandSim.h
 * 
 * Schunk Dextrous Hand controller simulator
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
#ifndef _GOLEM_CTRL_SCHUNKDEXHANSIM_SCHUNKDEXHANSIM_H_
#define _GOLEM_CTRL_SCHUNKDEXHANSIM_SCHUNKDEXHANSIM_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Schunk/SchunkDexHand.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR SchunkDexHandSim : public SchunkDexHand {
public:
	/** SchunkDexHandSim description */
	class GOLEM_LIBRARY_DECLDIR Desc : public SchunkDexHand::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(SchunkDexHandSim, Controller::Ptr, Context&)

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			SchunkDexHand::Desc::setToDefault();

			name = "Schunk Dextrous Hand simulator";
			
			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.0005);
			simDeltaSend = SecTmReal(0.0005);
		}

		virtual bool isValid() const {
			if (!SchunkDexHand::Desc::isValid())
				return false;

			return true;
		}
	};

	/** Release resources */
	virtual ~SchunkDexHandSim();

protected:
	SchunkDexHandSim(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_SCHUNKDEXHANSIM_SCHUNKDEXHANSIM_H_*/
