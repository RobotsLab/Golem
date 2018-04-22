/** @file OpenGL.h
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
#ifndef _GOLEM_CAMERA_OPENGL_OPENGL_H_
#define _GOLEM_CAMERA_OPENGL_OPENGL_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Camera.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class CameraOpenGLDevice;

/** OpenGL Camera */
class GOLEM_LIBRARY_DECLDIR CameraOpenGL : public Camera, public CameraImage {
public:
	friend class golem::CameraOpenGLDevice;

	/** Camera description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Camera::Desc {
	public:
		/** Creates Camera from the description. */
		GOLEM_CREATE_FROM_OBJECT_DESC1(CameraOpenGL, Sensor::Ptr, golem::Context&)
	};

	/** Destroys the Camera */
	virtual ~CameraOpenGL();

	/** CameraImage: Capture OpenGL context. */
	virtual void capture(int x, int y, int width, int height);

protected:
	mutable golem::CriticalSection csImage;
	golem::Event evImage;
	bool imageQuery;
	
	/** Image */
	ImageData image;
	/** Image time stamp. */
	TimeStamp timeStamp;

	/** Capture sequence start */
	virtual void start();
	/** Capture sequence stop */
	virtual void stop();

	/** Creates/initialises the Camera */
	void create(const Desc& desc);
	/** Constructs the Camera */
	CameraOpenGL(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_CAMERA_OPENGL_OPENGL_H_*/
