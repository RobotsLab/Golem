/** @file FTServer.cpp
*
* @author	Marek Kopicki
*
* @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
*
* @license  This file copy is licensed to you under the terms described in
*           the License.txt file included in this distribution.
*
*/

#include <Golem/SM/SMHelper.h>
#include <Golem/DAQFT/FTSensor.h>
#include <vector>
#include <stdlib.h>
#include <stdio.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

int main(int argc, char* argv[]) {
	if (argc < 7) {
		fprintf(stderr, "Usage: GolemFTServer [\"debug\"] <port> <sampling_rate> <samples_per_channel> <samples_bias> <sensor1_calibration_file> <sensor1_channel_map> [... <sensorN_calibration_file> <sensorN_channel_map>]");
		return 1;
	}

	try {
		// debug mode
		const bool debug = std::strcmp(argv[1], "debug") == 0;
		if (debug) { --argc; ++argv; }

		// Server and DAQSystem parameters (independent)
		const unsigned short port = (unsigned short)std::strtol(argv[1], nullptr, 10);
		const double samplingRate = std::max(1.0, std::strtod(argv[2], nullptr));
		const unsigned samplesPerChannel = std::max((unsigned)1, (unsigned)std::strtol(argv[3], nullptr, 10));
		const unsigned samplesBias = (unsigned)std::strtol(argv[4], nullptr, 10);

		// initialise sensors
		std::vector< boost::shared_ptr<FTSensor> > sensors;
		DAQSystem::SensorSet sensorSet;
		for (int ptr = 6; ptr < argc; ptr += 2)
			sensorSet.insert(sensors.insert(sensors.end(), boost::shared_ptr<FTSensor>(new FTSensor(argv[ptr - 1], 1, FTSensor::createChannelTypeMap(argv[ptr]))))->get());

		// create DAQSystem
		DAQSystem daqSystem(sensorSet, DAQSystem::CONNECTION_DIFFERENTIAL, DAQSystem::SAMPLING_DIRECT, samplingRate, samplesPerChannel);

		// DAQ parameters
		printf("DAQSystem(): channel_set=%s, num_channels=%u, sampling_frequency=%.4f, samples_per_channel=%u\n", DAQSystem::createDeviceChannelSetString(daqSystem.getDeviceChannelSet()).c_str(), daqSystem.getNumChannels(), daqSystem.getSamplingFrequency(), daqSystem.getSamplesPerChannel());

		// start sampling
		daqSystem.start();

		// buffer to store samples
		std::vector<double> buffer(daqSystem.getBuffSize());

		// compute bias if requested
		if (samplesBias > 0) {
			std::vector<double> bias(daqSystem.getBuffSize(), 0.);

			// read requested number of times
			for (unsigned i = 0; i < samplesBias; ++i) {
				daqSystem.read(buffer.data(), buffer.size());
				for (size_t j = 0; j < bias.size(); ++j)
					bias[j] += buffer[j];
			}

			// compute average
			for (size_t j = 0; j < bias.size(); ++j)
				bias[j] /= samplesBias;

			// update sensors bias
			for (size_t i = 0; i < sensors.size(); ++i)
				sensors[i]->setBias(bias.data());
		}

		// launch server
		SMTimer timer;
		SM::MessageStream msgstr;
		golem::SMServer server(port, timer, &msgstr);

		// F/T values for all sensors
		std::vector<double> ft(sensors.size() * FTSensor::NUM_FT_AXES, 0.);

		// continously read buffer and update server state
		for (double t0 = timer.elapsed();;) {
			// read DAQ buffer
			daqSystem.read(buffer.data(), buffer.size());

			// convert to F/T values
			for (size_t i = 0; i < sensors.size(); ++i) {
				try {
					sensors[i]->getValue(buffer.data(), ft.data() + i * FTSensor::NUM_FT_AXES);
				}
				catch (const std::exception& ex) {
					fprintf(stderr, "%s\n", ex.what());
				}
			}

			// update server
			server.write((unsigned)ft.size()*sizeof(double), ft.data());

			// print F/T values every 0.5 second in debug mode. TODO use separate thread to avoid blocking
			if (debug) {
				const double t1 = timer.elapsed();
				if (t0 + 0.5 < t1) {
					t0 = t1;
					for (size_t i = 0; i < sensors.size(); ++i)
						fprintf(stdout, "FT%zu = {(%10.5f, %10.5f, %10.5f), (%10.5f, %10.5f, %10.5f)}; ", i + 1, ft[i * FTSensor::NUM_FT_AXES + 0], ft[i * FTSensor::NUM_FT_AXES + 1], ft[i * FTSensor::NUM_FT_AXES + 2], ft[i * FTSensor::NUM_FT_AXES + 3], ft[i * FTSensor::NUM_FT_AXES + 4], ft[i * FTSensor::NUM_FT_AXES + 5]);
					fprintf(stdout, "\n");
				}
			}
		}
	}
	catch (const std::exception& ex) {
		fprintf(stderr, "%s\n", ex.what());
		return 1;
	}

	return 0;
}
