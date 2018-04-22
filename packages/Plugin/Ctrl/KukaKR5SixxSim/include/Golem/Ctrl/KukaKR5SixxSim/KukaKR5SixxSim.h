/** @file KukaKR5SixxSim.h
 * 
 * Kuka KR 5 Sixx R650/R850 controller simulator
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
#ifndef _GOLEM_CTRL_KUKAKR5SIXXSIM_KUKAKR5SIXXSIM_H_
#define _GOLEM_CTRL_KUKAKR5SIXXSIM_KUKAKR5SIXXSIM_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Kuka/KukaKR5Sixx.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR KukaKR5SixxSim : public KukaKR5Sixx {
public:
	/** KukaKR5SixxSim description */
	class GOLEM_LIBRARY_DECLDIR Desc : public KukaKR5Sixx::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(KukaKR5SixxSim, Controller::Ptr, Context&)

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			KukaKR5Sixx::Desc::setToDefault();

			name = "Kuka KR 5 Sixx simulator";
			
			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.01);
			simDeltaSend = SecTmReal(0.002);

			chains.clear();
			chains.push_back(Chain::Desc::Ptr(new KukaKR5SixxChain::Desc));
		}

		virtual bool isValid() const {
			if (!KukaKR5Sixx::Desc::isValid())
				return false;

			if (chains.size() != 1)
				return false;
			if (dynamic_cast<const KukaKR5SixxChain::Desc*>(chains.begin()->get()) == NULL)
				return false;
			
			return true;
		}
	};

	/** Release resources */
	virtual ~KukaKR5SixxSim();

protected:
	KukaKR5SixxSim(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_KUKAKR5SIXXSIM_KUKAKR5SIXXSIM_H_*/
