/** @file Ensenso.h
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
#ifndef _GOLEM_CAMERA_ENSENSO_ENSENSO_H_
#define _GOLEM_CAMERA_ENSENSO_ENSENSO_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Camera.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Ensenso Depth Camera */
class GOLEM_LIBRARY_DECLDIR CameraEnsenso : public CameraDepth {
public:
	/** Capture mode */
	enum Mode {
		/** Undefined */
		MODE_UNDEF = 0,
		/** IR */
		MODE_IR = (1<<0),
		/** Colour */
		MODE_COLOUR = (1<<1),
		/** Depth */
		MODE_DEPTH = (1<<2),
	};

	/** Camera description */
	class GOLEM_LIBRARY_DECLDIR Desc : public CameraDepth::Desc {
	public:
		/** Image buffer offset */
		size_t bufferOff;
	
		/** Camera stream time out */
		golem::MSecTmU32 streamTimeOut;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		/** Creates Camera from the description. */
		GOLEM_CREATE_FROM_OBJECT_DESC1(CameraEnsenso, Sensor::Ptr, golem::Context&)

		/** Sets the parameters to the default values. */
		void setToDefault() {
			CameraDepth::Desc::setToDefault();
			
			bufferOff = 30;
			streamTimeOut = 1000;
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			CameraDepth::Desc::assertValid(ac);

			Assert::valid(streamTimeOut >= golem::SEC_TM_REAL_ZERO, ac, "streamTimeOut: < 0");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
	};

	/** String camera mode helper */
	static golem::U32 getMode(const std::string& str);

protected:
	/** Image buffer offset */
	size_t bufferOff;
	/** Camera stream time out */
	golem::MSecTmU32 streamTimeOut;

	/** Capture sequence start */
	virtual void start();
	/** Capture sequence stop */
	virtual void stop();

	/** Creates/initialises the Camera */
	void create(const Desc& desc);
	/** Constructs the Camera */
	CameraEnsenso(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_CAMERA_ENSENSO_ENSENSO_H_*/
