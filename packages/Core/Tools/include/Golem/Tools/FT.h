/** @file FT.h
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
#ifndef _GOLEM_TOOLS_FT_H_
#define _GOLEM_TOOLS_FT_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Sensor.h>
#include <Golem/Tools/FTCalibration.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Force and torque sensor interface */
class FT : public Sensor {
public:
	typedef golem::ConfigMat34 Config;

	/** FT sensor data */
	class Data : public TimeStamp {
	public:
		/** Wrench */
		golem::Twist wrench;

		/** Config*/
		Config config;

		/** No data allocation */
		Data(golem::SecTmReal timeStamp = golem::SEC_TM_REAL_ZERO);
		/** Copies image */
		Data(const golem::Twist& wrench, golem::SecTmReal timeStamp = golem::SEC_TM_REAL_ZERO);
	};

	/** FT sensor description */
	class Desc : public Sensor::Desc {
	public:
		/** FT sensor calibration */
		FTCalibration::Desc::Ptr calibrationDesc;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values. */
		void setToDefault() {
			Sensor::Desc::setToDefault();

			//header = "GolemFT";

			calibrationDesc.reset(new FTCalibration::Desc);
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Sensor::Desc::assertValid(ac);

			Assert::valid(calibrationDesc != nullptr, ac, "calibrationDesc: NULL");
			calibrationDesc->assertValid(Assert::Context(ac, "calibrationDesc->"));
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
	};

	/** Read newest F&T with calibration, blocking call */
	virtual void read(Data& data, bool inertia = true); // throws

	/** Read newest F&T, blocking call */
	virtual void readSensor(golem::Twist& wrench, golem::SecTmReal& timeStamp) = 0; // throws

	/** Get Current calibration */
	FTCalibration* getCurrentCalibration() {
		return calibration.get();
	}
	/** Get Current calibration */
	const FTCalibration* getCurrentCalibration() const {
		return calibration.get();
	}

	/** Curent sensor frame */
	virtual golem::Mat34 getFrame() const;

protected:
	/** FT calibration */
	FTCalibration::Ptr calibration;

	/** Creates/initialises sensor */
	void create(const Desc& desc);
	/** Constructs sensor */
	FT(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_TOOLS_FT_H_*/
