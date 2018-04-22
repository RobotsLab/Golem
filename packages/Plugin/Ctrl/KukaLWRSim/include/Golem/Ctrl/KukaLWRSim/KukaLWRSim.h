/** @file KukaLWRSim.h
 * 
 * Kuka Light Weight Robot simulator
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
#ifndef _GOLEM_CTRL_KUKALWRSIM_KUKALWRSIM_H_
#define _GOLEM_CTRL_KUKALWRSIM_KUKALWRSIM_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Kuka/KukaLWR.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR KukaLWRSim : public KukaLWR {
public:
	/** KukaLWRSim description */
	class GOLEM_LIBRARY_DECLDIR Desc : public KukaLWR::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(KukaLWRSim, Controller::Ptr, Context&)

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			KukaLWR::Desc::setToDefault();

			name = "Kuka LWR simulator";
			
			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.01);
			simDeltaSend = SecTmReal(0.002);

			chains.clear();
			chains.push_back(Chain::Desc::Ptr(new KukaLWRChain::Desc));
		}

		virtual bool isValid() const {
			if (!KukaLWR::Desc::isValid())
				return false;

			if (chains.size() != 1)
				return false;
			if (dynamic_cast<const KukaLWRChain::Desc*>(chains.begin()->get()) == NULL)
				return false;
			
			return true;
		}
	};

	/** Release resources */
	virtual ~KukaLWRSim();

protected:
	KukaLWRSim(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_KUKALWRSIM_KUKALWRSIM_H_*/
