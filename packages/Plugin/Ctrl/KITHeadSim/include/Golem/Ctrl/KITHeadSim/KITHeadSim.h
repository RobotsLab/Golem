/** @file KITHeadSim.h
 * 
 * DLR Hit Hand II controller simulator
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
#ifndef _GOLEM_CTRL_KITHEADSIM_KITHEADSIM_H_
#define _GOLEM_CTRL_KITHEADSIM_KITHEADSIM_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/KIT/KITHead.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR KITHeadSim : public KITHead {
public:
	/** KITHeadSim description */
	class GOLEM_LIBRARY_DECLDIR Desc : public KITHead::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(KITHeadSim, Controller::Ptr, Context&)

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			KITHead::Desc::setToDefault();

			name = "KIT Head simulator";
			
			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.005);
			simDeltaSend = SecTmReal(0.005);
		}

		virtual bool isValid() const {
			if (!KITHead::Desc::isValid())
				return false;

			return true;
		}
	};

protected:
	KITHeadSim(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_KITHEADSIM_KITHEADSIM_H_*/
