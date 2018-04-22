/** @file KukaIIWASim.h
 *
 * Kuka IIWA simulator
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
#ifndef _GOLEM_CTRL_KUKAIIWASIM_KUKAIIWASIM_H_
#define _GOLEM_CTRL_KUKAIIWASIM_KUKAIIWASIM_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Kuka/KukaIIWA.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR KukaIIWASim : public KukaIIWA {
public:
	/** KukaIIWASim description */
	class GOLEM_LIBRARY_DECLDIR Desc : public KukaIIWA::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(KukaIIWASim, Controller::Ptr, Context&)

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			KukaIIWA::Desc::setToDefault();

			name = "Kuka LWR simulator";

			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.01);
			simDeltaSend = SecTmReal(0.002);

			chains.clear();
			chains.push_back(Chain::Desc::Ptr(new KukaIIWAChain::Desc));
		}

		virtual bool isValid() const {
			if (!KukaIIWA::Desc::isValid())
				return false;

			if (chains.size() != 1)
				return false;
			if (dynamic_cast<const KukaIIWAChain::Desc*>(chains.begin()->get()) == NULL)
				return false;

			return true;
		}
	};

	/** Release resources */
	virtual ~KukaIIWASim();

protected:
	KukaIIWASim(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_KUKAIIWASIM_KUKAIIWASIM_H_*/
