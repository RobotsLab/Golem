/** @file DLRHandIISim.h
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
#ifndef _GOLEM_CTRL_DLRHandIISim_DLRHandIISim_H_
#define _GOLEM_CTRL_DLRHandIISim_DLRHandIISim_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/DLR/DLRHandII.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR DLRHandIISim : public DLRHandII {
public:
	/** DLRHandIISim description */
	class GOLEM_LIBRARY_DECLDIR Desc : public DLRHandII::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(DLRHandIISim, Controller::Ptr, Context&)

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			DLRHandII::Desc::setToDefault();

			name = "DLR Hand II simulator";
			
			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.0005);
			simDeltaSend = SecTmReal(0.0005);
		}

		virtual bool isValid() const {
			if (!DLRHandII::Desc::isValid())
				return false;

			return true;
		}
	};

	/** Release resources */
	virtual ~DLRHandIISim();

protected:
	DLRHandIISim(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_DLRHandIISim_DLRHandIISim_H_*/
