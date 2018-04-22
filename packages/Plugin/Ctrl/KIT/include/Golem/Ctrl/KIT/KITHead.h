/** @file KITHead.h
 * 
 * KIT Head base classes
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
#ifndef _GOLEM_CTRL_KIT_KITHEAD_H_
#define _GOLEM_CTRL_KIT_KITHEAD_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/SingleCtrl/SingleCtrl.h>
#include <Golem/Sys/Library.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** KIT Head controller base class. */
class GOLEM_LIBRARY_DECLDIR KITHead: public SingleCtrl {
public:
	/** Number of chains: neck + 2 eyes */
	static const U32 NUM_CHAINS = 3;
	/** Number of joints */
	static const U32 NUM_JOINTS_NECK = 5;
	/** Number of joints */
	static const U32 NUM_JOINTS_LEFT_EYE = 1;
	/** Number of joints */
	static const U32 NUM_JOINTS_RIGHT_EYE = 1;
	/** Number of joints */
	static const U32 CHAIN_NUM_JOINTS[NUM_CHAINS];
	/** Number of joints */
	static const U32 NUM_JOINTS = NUM_JOINTS_NECK + NUM_JOINTS_LEFT_EYE + NUM_JOINTS_RIGHT_EYE;

	/** KIT Head description */
	class GOLEM_LIBRARY_DECLDIR Desc : public SingleCtrl::Desc {
	public:
		Desc() {
			setToDefault();
		}
		
		virtual void setToDefault() {
			SingleCtrl::Desc::setToDefault();

			cycleDurationCtrl = false;
			timeQuant = SecTmReal(0.0001);
			cycleDurationInit = SecTmReal(0.1);

			for (size_t i = 0; i < NUM_CHAINS; ++i) {
				chains.push_back(Chain::Desc::Ptr(new Chain::Desc));
				for (U32 j = 0; j < CHAIN_NUM_JOINTS[i]; ++j){
					chains.back()->joints.push_back(Joint::Desc::Ptr(new Joint::Desc));
				}
			}
		}
		
		virtual bool isValid() const {
			if (!SingleCtrl::Desc::isValid())
				return false;

			if (chains.size() != NUM_CHAINS)
				return false;
			for (size_t i = 0; i < NUM_CHAINS; ++i)
				if (chains[i]->joints.size() != CHAIN_NUM_JOINTS[i])
					return false;

			return true;
		}
	};

protected:
	KITHead(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_KIT_KITHEAD_H_*/
