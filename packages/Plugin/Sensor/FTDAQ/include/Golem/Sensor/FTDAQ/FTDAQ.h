/** @file FTDAQ.h
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
#ifndef _GOLEM_CORE_FTDAQ_H_
#define _GOLEM_CORE_FTDAQ_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/FT.h>
#include <Golem/DAQFT/FTSensor.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** FT sensor network client */
class GOLEM_LIBRARY_DECLDIR FTDAQ : public FT {
public:
	/** FTSensor description */
	class GOLEM_LIBRARY_DECLDIR Desc : public FT::Desc {
	public:
		/** the frequency at which to sample the transducer voltages */
		double samplingRate;
		/** the number of raw samples to average together to make one output sample */
		unsigned samplesPerChannel;
		/** sample bias */
		unsigned samplesBias;
		/** the type of connection to use */
		golem::DAQSystem::ConnectionType connectionType;

		/** path to the calibration file for the transducer you are using */
		std::string calibrationFile;
		/** the name of the device which the transducer is connected to */
		std::string channelMap;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			FT::Desc::setToDefault();

			samplingRate = 10000.0; // 10kHz
			samplesPerChannel = 10;
			samplesBias = 0;
			connectionType = golem::DAQSystem::ConnectionType::CONNECTION_DIFFERENTIAL;

			calibrationFile = "10000.cal";
			channelMap = "Dev1/ai0:FX,Dev1/ai1:FY,Dev1/ai2:FZ,Dev1/ai3:TX,Dev1/ai4:TY,Dev1/ai5:TZ";
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			FT::Desc::assertValid(ac);

			Assert::valid(samplingRate > golem::REAL_EPS, ac, "samplingRate: <= 0");
			Assert::valid(samplesPerChannel > 0, ac, "samplesPerChannel: <= 0");
			Assert::valid(calibrationFile.length() > 0, ac, "calibrationFile: empty");
			Assert::valid(channelMap.length() > 0, ac, "channelMap: empty");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		GOLEM_CREATE_FROM_OBJECT_DESC1(FTDAQ, Sensor::Ptr, golem::Context&)
	};

	/** Read newest F&T, blocking call */
	virtual void readSensor(golem::Twist& wrench, golem::SecTmReal& timeStamp); // throws

protected:
	/** DAQ device */
	struct Device {
		typedef golem::shared_ptr<Device> Ptr;
		/** DAQ device */
		golem::shared_ptr<golem::DAQSystem> daq;
		/** DAQ cs */
		golem::CriticalSection cs;
		/** DAQ reading time stamp */
		golem::SecTmReal reading;
		/** Min sampling interval */
		double samplingInterval;
		/** DAQ buffer */
		std::vector<double> buffer;
	};
	/** DAQ device */
	Device::Ptr device;

	/** the frequency at which to sample the transducer voltages */
	double samplingRate;
	/** the number of raw samples to average together to make one output sample */
	unsigned samplesPerChannel;
	/** sample bias */
	unsigned samplesBias;
	/** the type of connection to use */
	golem::DAQSystem::ConnectionType connectionType;

	/** F/T sensor */
	golem::shared_ptr<golem::FTSensor> ftSensor;

	/** Batch initialisation */
	virtual void init(const Map& map);

	/** Creates/initialises sensor */
	void create(const Desc& desc);
	/** Constructs sensor */
	FTDAQ(golem::Context& context);
};

//------------------------------------------------------------------------------

const char* getDAQFTConnectionType(const golem::DAQSystem::ConnectionType& connType);
golem::DAQSystem::ConnectionType getDAQFTConnectionType(const char* connType);
void XMLData(const std::string &attr, golem::DAQSystem::ConnectionType& val, golem::XMLContext* context, bool create);

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_CORE_FTDAQ_H_*/
