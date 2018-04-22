/** @file DLRHitHandIISim.h
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
#ifndef _GOLEM_CTRL_DLRHITHANDIISIM_DLRHITHANDIISIM_H_
#define _GOLEM_CTRL_DLRHITHANDIISIM_DLRHITHANDIISIM_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/DLR/DLRHitHandII.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR DLRHitHandIISim : public DLRHitHandII {
public:
	/** DLRHitHandIISim description */
	class GOLEM_LIBRARY_DECLDIR Desc : public DLRHitHandII::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(DLRHitHandIISim, Controller::Ptr, Context&)

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			DLRHitHandII::Desc::setToDefault();

			name = "DLR Hit Hand II simulator";
			
			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.0005);
			simDeltaSend = SecTmReal(0.0005);
		}

		virtual bool isValid() const {
			if (!DLRHitHandII::Desc::isValid())
				return false;

			return true;
		}
	};

	/** Release resources */
	virtual ~DLRHitHandIISim();

protected:
	DLRHitHandIISim(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_DLRHITHANDIISIM_DLRHITHANDIISIM_H_*/
