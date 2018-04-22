/** @file PointGrey.h
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
#ifndef _GOLEM_CAMERA_POINTGREY_POINTGREY_H_
#define _GOLEM_CAMERA_POINTGREY_POINTGREY_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Camera.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** PGR Fly Capture camera */
class GOLEM_LIBRARY_DECLDIR CameraPointGrey : public Camera {
public:
	/** Camera description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Camera::Desc {
	public:
		/** Creates Camera from the description. */
		GOLEM_CREATE_FROM_OBJECT_DESC1(CameraPointGrey, Sensor::Ptr, golem::Context&)
	};

protected:
	/** Capture sequence start */
	virtual void start();
	/** Capture sequence stop */
	virtual void stop();

	/** Creates/initialises the Camera */
	void create(const Desc& desc);
	/** Creates the Camera */
	CameraPointGrey(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_CAMERA_POINTGREY_POINTGREY_H_*/
