/** @file Creator.h
 * 
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
#ifndef _GOLEM_PHYSCTRL_CREATOR_H_
#define _GOLEM_PHYSCTRL_CREATOR_H_

//------------------------------------------------------------------------------

#include <Golem/Sim/Universe.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Creator
*/
class Creator {
public:
	/** Creation mode. */
	enum Mode {
		/** Create objects in the frame centre */
		MODE_CENTRE = 0,
		/** Create objects on the ground plane */
		MODE_GROUNDPLANE = 1,
	};
	
protected:
	golem::Scene &scene;
	Context &context;

public:
	/** Generator of pseudo random numbers */
	Rand rand;
	/** Creation mode. */
	Mode mode;
	
	/** Constructs the Creator object on a given Scene. */
	Creator(golem::Scene &scene);

	/** Virtual descructor */
	virtual ~Creator();

	/** Sets the objet parameters to the default values */
	virtual void setToDefault() {
		mode = MODE_GROUNDPLANE;
	}
	/** Checks if the object description is valid. */
	virtual bool isValid() const {
		return true;
	}
	
	inline golem::Context& getContext() {
		return context;
	}
	
	inline const golem::Context& getContext() const {
		return context;
	}
	
	inline golem::Scene& getScene() {
		return scene;
	}
	
	inline const golem::Scene& getScene() const {
		return scene;
	}
	
	/** Generic bounds setup */
	void setupBoundsDesc(golem::Actor::Desc *pActorDesc, const Bounds::Desc &desc);

	/** Bounding plane setup */
	void setupBoundsDesc(golem::Actor::Desc *pActorDesc, const BoundingPlane::Desc &desc);

	/** Creates ground plane description */
	golem::Actor::Desc::Ptr createGroundPlaneDesc();
	
	/** Creates box description */
	golem::Actor::Desc::Ptr createBoxDesc(Real x = Real(1.0), Real y = Real(1.0), Real z = Real(1.0));
	
	/** Creates tree description */
	golem::Actor::Desc::Ptr createTreeDesc(Real radius = Real(1.0), Real thickness = Real(0.25));
	
	/** Creates box-based 2-flap description  */
	golem::Actor::Desc::Ptr createSimple2FlapDesc(Real width = Real(1.0), Real height = Real(1.0), Real length = Real(1.0), Real thickness = Real(0.002), Real angle = REAL_PI_2);

	/** Creates box-based 2-flap description  */
	golem::Actor::Desc::Ptr create2FlapDesc(Real width1 = Real(1.0), Real width2 = Real(1.0), Real shift1 = Real(0.0), Real shift2 = Real(0.0), Real height = Real(1.0), Real length = Real(1.0), Real thickness = Real(0.002), Real angle = REAL_PI_2, bool moveFrame = false);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYSCTRL_CREATOR_H_*/
