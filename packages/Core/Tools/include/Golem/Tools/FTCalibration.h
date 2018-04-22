/** @file FTCalibration.h
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
#ifndef _GOLEM_TOOLS_FTCALIBRATION_H_
#define _GOLEM_TOOLS_FTCALIBRATION_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Defs.h>
#include <Golem/Tools/Defs.h>

//------------------------------------------------------------------------------

namespace golem {

class FT;

/** FTCalibration interface */
class FTCalibration {
public:
	typedef golem::shared_ptr<FTCalibration> Ptr;

	typedef golem::ConfigMat34 Config;
	typedef std::pair<Config, golem::Twist> Equation;
	typedef std::vector<Equation> EquationSeq;

	friend class FT;

	/** FTSensor description */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Local frame */
		golem::Mat34 localFrame;
		/** Calibration file */
		std::string file;
		/** Use inertia */
		bool useInertia;
		/** Sampling interval in milliseconds */
		golem::U32 samplingInterval;
		/** Number of samples for median filter */
		golem::U32 sampleWindowSize;

		/** Mass parameter range */
		golem::Real massParamMagnitude;
		/** F/T offset parameter range */
		golem::Real offsetParamMagnitude;
		/** Local frame orientation range with Euler angles */
		golem::Vec3 localFrameParamMagnitude;

		/** Create description. */
		Desc() {
			setToDefault();
		}
		/** Destroys description. */
		virtual ~Desc() {}

			/** Sets the parameters to the default values */
		void setToDefault() {
			localFrame.setId();
			file = "GolemFT.cal";
			useInertia = true;
			samplingInterval = 5;
			sampleWindowSize = 100;

			massParamMagnitude = golem::Real(10.0);
			offsetParamMagnitude = golem::Real(10.0);
			localFrameParamMagnitude.set(golem::Real(0.0));
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Assert::valid(localFrame.isValid(), ac, "localFrame: invalid");
			Assert::valid(samplingInterval > 0, ac, "samplingInterval < 1");
			Assert::valid(sampleWindowSize > 0, ac, "sampleWindowSize < 1");
			Assert::valid(massParamMagnitude > golem::REAL_EPS, ac, "massParamMagnitude < EPS");
			Assert::valid(offsetParamMagnitude > golem::REAL_EPS, ac, "offsetParamMagnitude < EPS");
			Assert::valid(localFrameParamMagnitude.isFinite(), ac, "localFrameParamMagnitude < EPS");
		}

		/** Creates the object from the description. */
		GOLEM_CREATE_FROM_OBJECT_DESC1(FTCalibration, FTCalibration::Ptr, FT&)
	};
	
	/** Calibrate */
	virtual void calibrate();
	/** Transforms wrench */
	virtual void transform(const Config& config, const golem::Twist& inp, golem::Twist& out) const;

	/** Adds calibration point */
	void add();
	/** Clear */
	void clear();

	/** Use inertia */
	bool isEnabledIneria() const {
		return useInertia;
	}
	/** Enable inertia */
	void enableInertia(bool useInertia);

	/** Sensor frame */
	virtual golem::Mat34 getFrame(const Config& config) const;

	/** Local frame */
	golem::Mat34 getLocalFrame() const {
		return localFrame;
	}

	/** Load calibration from file */
	virtual void load();
	/** Save calibration to file */
	virtual void save() const;

	/** Context */
	golem::Context& getContext() {
		return context;
	}
	/** Context */
	const golem::Context& getContext() const {
		return context;
	}

	/** Destroy FT sensor calibration */
	virtual ~FTCalibration();

protected:
	/** FT */
	FT& ft;
	/** Context */
	golem::Context& context;
	
	/** Local frame */
	golem::Mat34 localFrame;
	/** Sampling interval in milliseconds */
	golem::U32 samplingInterval;
	/** Number of samples for median filter */
	golem::U32 sampleWindowSize;
	/** Calibration file */
	std::string file;
	/** Calibration points */
	EquationSeq points;

	/** Mass parameter range */
	golem::Real massParamMagnitude;
	/** F/T offset parameter range */
	golem::Real offsetParamMagnitude;
	/** Local frame orientation range with Euler angles */
	golem::Vec3 localFrameParamMagnitude;

	/** Mass parameter */
	golem::Twist massParam;
	/** F/T offset parameter */
	golem::Twist offsetParam;
	/** Local frame orientation parameter */
	golem::Mat33 localFrameParam;

	/** Use inertia */
	bool useInertia;
	/** Calibration state */
	bool calibrated;

	/** Create FT sensor calibration */
	void create(const Desc& desc);

	/** Create FT sensor calibration */
	FTCalibration(FT& ft);

	/** Identification transform for F/T sensor*/
	void transformMass(const golem::Mat33& frame, const golem::Twist& massParam, const golem::Twist& offsetParam, const golem::Mat33& localFrameParam, const golem::Twist& inp, golem::Twist& out) const;
};

/** Reads/writes object from/to a given XML context */
void XMLData(FTCalibration::Equation &val, golem::XMLContext* xmlcontext, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(FTCalibration::Desc &val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(FTCalibration::Desc::Ptr &val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_TOOLS_FTCALIBRATION_H_*/
