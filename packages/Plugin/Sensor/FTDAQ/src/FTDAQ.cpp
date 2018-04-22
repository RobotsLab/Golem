/** @file FTDAQ.cpp
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sensor/FTDAQ/FTDAQ.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new FTDAQ::Desc();
}

//------------------------------------------------------------------------------

void golem::FTDAQ::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	FT::Desc::load(context, xmlcontext);

	golem::XMLData("sampling_rate", samplingRate, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("samples_per_channel", samplesPerChannel, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("samples_bias", samplesBias, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("connection_type", connectionType, const_cast<golem::XMLContext*>(xmlcontext), false);

	golem::XMLData("calibration_file", calibrationFile, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("channel_map", channelMap, const_cast<golem::XMLContext*>(xmlcontext), false);
}

//------------------------------------------------------------------------------

golem::FTDAQ::FTDAQ(golem::Context& context) : FT(context) {
}

void golem::FTDAQ::create(const Desc& desc) {
	FT::create(desc);

	samplingRate = desc.samplingRate;
	samplesPerChannel = desc.samplesPerChannel;
	samplesBias = desc.samplesBias;
	connectionType = desc.connectionType;

	ftSensor.reset(new golem::FTSensor(desc.calibrationFile, 1, golem::FTSensor::createChannelTypeMap(desc.channelMap)));
}

void golem::FTDAQ::init(const Map& map) {
	// cannot be run concurrently!!!
	
	// already initialised
	if (device != nullptr)
		return;
	device.reset(new Device);

	context.verbose("FTDAQ::init(): Master %s\n", getID().c_str());

	// not initialised
	golem::DAQSystem::SensorSet sensorSet;

	// update DAQSystem config
	for (Map::const_iterator i = map.begin(); i != map.end(); ++i) {
		FTDAQ* pFTDAQ = is<FTDAQ>(i);
		if (pFTDAQ)
			sensorSet.insert(pFTDAQ->ftSensor.get());
	}

	// create DAQSystem
	device->daq.reset(new golem::DAQSystem(sensorSet, connectionType, golem::DAQSystem::SAMPLING_CALLBACK, samplingRate, samplesPerChannel));

	// buffers and timing parameters
	device->buffer.resize(device->daq->getBuffSize(), 0.);
	device->samplingInterval = 1. / samplingRate;
	device->reading = golem::SEC_TM_REAL_ZERO;

	// start sampling
	device->daq->start();

	// bias
	std::vector<double> bias(device->daq->getBuffSize(), 0.);
	if (samplesBias > 0) {
		// read requested number of times
		for (unsigned i = 0; i < samplesBias; ++i) {
			device->daq->read(device->buffer.data(), device->buffer.size());
			for (size_t j = 0; j < bias.size(); ++j)
				bias[j] += device->buffer[j];
		}

		// compute average
		for (size_t j = 0; j < bias.size(); ++j)
			bias[j] /= samplesBias;
	}

	// update other sensors
	for (Map::const_iterator i = map.begin(); i != map.end(); ++i) {
		FTDAQ* pFTDAQ = is<FTDAQ>(i);
		if (pFTDAQ) {
			context.verbose("FTDAQ::init(): Initialising %s\n", pFTDAQ->getID().c_str());
			pFTDAQ->device = device; // device
			if (samplesBias > 0) pFTDAQ->ftSensor->setBias(bias.data()); // sample bias
		}
	}
}

void golem::FTDAQ::readSensor(golem::Twist& wrench, golem::SecTmReal& timeStamp) {
	if (device == nullptr)
		throw Message(Message::LEVEL_CRIT, "FTDAQ::readSensor(): DAQSystem has not been initialised");

	golem::CriticalSectionWrapper csw(device->cs);

	const golem::SecTmReal t = context.getTimer().elapsed();
	if (device->reading + device->samplingInterval < t) {
		device->daq->read(device->buffer.data(), device->buffer.size());
		device->reading = t;
	}

	ftSensor->getValue(device->buffer.data(), &wrench.v.x);

	//static int j = 0;
	//if (j++ % 10 == 0) {
	//	context.write("%s: (% 8.4f, % 8.4f, % 8.4f) (% 8.4f, % 8.4f, % 8.4f)\n", getID().c_str(), wrench.v.x, wrench.v.y, wrench.v.z, wrench.w.x, wrench.w.y, wrench.w.z);
	//}
}

//------------------------------------------------------------------------------

const char* golem::getDAQFTConnectionType(const golem::DAQSystem::ConnectionType& connType) {
	return
		connType == golem::DAQSystem::CONNECTION_REFERENCED_SINGLE_ENDED ? "referenced_single_ended" :
		connType == golem::DAQSystem::CONNECTION_NON_REFERENCED_SINGLE_ENDED ? "non_referenced_single_ended" :
		"differential";
}

golem::DAQSystem::ConnectionType golem::getDAQFTConnectionType(const char* connType) {
	return
		std::strcmp(connType, "referenced_single_ended") == 0 ? golem::DAQSystem::CONNECTION_REFERENCED_SINGLE_ENDED :
		std::strcmp(connType, "non_referenced_single_ended") == 0 ? golem::DAQSystem::CONNECTION_NON_REFERENCED_SINGLE_ENDED :
		golem::DAQSystem::CONNECTION_DIFFERENTIAL;
}

void golem::XMLData(const std::string &attr, golem::DAQSystem::ConnectionType& val, golem::XMLContext* context, bool create) {
	std::string connType;
	XMLData(attr, connType, context, create);
	std::transform(connType.begin(), connType.end(), connType.begin(), tolower);
	val = getDAQFTConnectionType(connType.c_str());
}

//------------------------------------------------------------------------------
