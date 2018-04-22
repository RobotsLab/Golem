/** @file CameraRGBSim.h
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2018 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_SENSOR_CAMERARGBSIM_CAMERARGBSIM_H_
#define _GOLEM_SENSOR_CAMERARGBSIM_CAMERARGBSIM_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Camera.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** CameraRGBSim Camera */
class GOLEM_LIBRARY_DECLDIR CameraRGBSim : public Camera {
public:
	/** Camera description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Camera::Desc {
	public:
		/** Input file */
		std::string inputFile;
		/** Frame index */
		golem::U32 frameIndex;

		/** Creates Camera from the description. */
		GOLEM_CREATE_FROM_OBJECT_DESC1(CameraRGBSim, Sensor::Ptr, golem::Context&)

		/** Sets the parameters to the default values. */
		void setToDefault() {
			Camera::Desc::setToDefault();

			inputFile.clear();
			frameIndex = 0;
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Camera::Desc::assertValid(ac);

			Assert::valid(!inputFile.empty(), ac, "inputFile: empty");
			Assert::valid(frameIndex >= 0, ac, "frameIndex: < 0");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
	};

protected:
	/** Input file */
	std::string inputFile;
	/** Frame index */
	golem::U32 frameIndex;

	/** Capture sequence start */
	virtual void start();
	/** Capture sequence stop */
	virtual void stop();

	/** Creates/initialises the Camera */
	void create(const Desc& desc);
	/** Constructs the Camera */
	CameraRGBSim(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_SENSOR_CAMERARGBSIM_CAMERARGBSIM_H_*/
