/** @file Sim.h
 * 
 * Controller simulator
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
#ifndef _GOLEM_CTRL_SIM_SIM_H_
#define _GOLEM_CTRL_SIM_SIM_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/SingleCtrl/SingleCtrl.h>
#include <Golem/Sys/Library.h>
#include <Golem/Sys/XMLParser.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR Sim: public SingleCtrl {
public:	
	/** Sim description */
	class GOLEM_LIBRARY_DECLDIR Desc : public SingleCtrl::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(Sim, Controller::Ptr, Context&)

		/** Coordinate linear map */
		CoordMap::Seq coordMapSeq;

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			SingleCtrl::Desc::setToDefault();

			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.01);
			simDeltaSend = SecTmReal(0.002);

			name = "Controller simulator";

			chains.clear();
			// number of kinematic chains and joints is controlled by URDF files

			coordMapSeq.clear();
		}
		
		virtual bool isValid() const {
			if (!SingleCtrl::Desc::isValid())
				return false;

			for (CoordMap::Seq::const_iterator i = coordMapSeq.begin(); i != coordMapSeq.end(); ++i)
				if (!i->isValid())
					return false;

			return true;
		}
	};
	
	/** Release resources */
	virtual ~Sim();

	/** Interpolates the controller state at time t. */
	virtual void lookupState(SecTmReal t, State &state) const;

	/** Sets to default a given state. */
	virtual void setToDefault(State& state) const;

	/** Clamp configuration. */
	virtual void clampConfig(GenConfigspaceCoord& config) const;

protected:
	/** Coordinate linear map */
	CoordMap::Seq coordMapSeq;

	void create(const Desc& desc);
	Sim(Context& context);
};

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(Joint::Desc::Ptr &val, XMLContext* context, bool create = false);
void XMLData(Chain::Desc::Ptr &val, XMLContext* context, bool create = false);
void XMLData(Sim::Desc &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_SIM_SIM_H_*/
