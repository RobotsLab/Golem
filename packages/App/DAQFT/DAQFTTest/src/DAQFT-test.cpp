/** @file FTSensor.h
 *
 * DAQFT-test.cpp - test program for ATI F/T sensor
 * reads voltages and converts to forces/torques
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <iostream>
#include <stdlib.h>
#include <stdexcept>
#ifdef LINUX
#include "unistd.h"
void Sleep(const int ms) { sleep(ms/1000); }
#else
#include <Windows.h> /* for Sleep() */
#endif
#include <Golem/DAQFT/FTSensor.h>

// sampling frequency test
//#define SAMPLING_TEST
#ifdef SAMPLING_TEST
#include <Mmsystem.h>
#include <fstream>
#pragma comment(lib, "winmm.lib")

class Timer {
	LARGE_INTEGER sysFreq, perfStamp;
public:
	Timer() {
		(void)::QueryPerformanceFrequency(&sysFreq);
		(void)::QueryPerformanceCounter(&perfStamp);
	}
	double elapsed() const {
		LARGE_INTEGER t;
		(void)::QueryPerformanceCounter(&t);
		return (double)(t.QuadPart - perfStamp.QuadPart) / sysFreq.QuadPart;
	}
};
#endif // SAMPLING_TEST

using namespace golem;

int main(int argc, char* argv[]) {
	try {
		FTSensor::ChannelTypeMap channelMap;
		DAQSystem::SensorSet sensorSet;

		// define F/T sensor channels - 2 methods:
		// from string
		channelMap = FTSensor::createChannelTypeMap("Dev2/ai0:FX,Dev2/ai1:FY,Dev2/ai2:FZ,Dev2/ai3:TX,Dev2/ai4:TY,Dev2/ai5:TZ");
		// from numerical values
		//channelMap.clear();
		//channelMap.insert(std::make_pair(DAQSystem::DeviceChannel("Dev2", 0), FTSensor::CHANNEL_FX));
		//channelMap.insert(std::make_pair(DAQSystem::DeviceChannel("Dev2", 1), FTSensor::CHANNEL_FY));
		//channelMap.insert(std::make_pair(DAQSystem::DeviceChannel("Dev2", 2), FTSensor::CHANNEL_FZ));
		//channelMap.insert(std::make_pair(DAQSystem::DeviceChannel("Dev2", 3), FTSensor::CHANNEL_TX));
		//channelMap.insert(std::make_pair(DAQSystem::DeviceChannel("Dev2", 4), FTSensor::CHANNEL_TY));
		//channelMap.insert(std::make_pair(DAQSystem::DeviceChannel("Dev2", 5), FTSensor::CHANNEL_TZ));
		// create F/T sensor
		FTSensor sensor1("FT10731.cal", 1, channelMap);
		sensorSet.insert(&sensor1);
	
		// define F/T sensor channels - 2 methods:
		// from string
		channelMap = FTSensor::createChannelTypeMap("Dev2/ai50:FX,Dev2/ai51:FY,Dev2/ai52:FZ,Dev2/ai53:TX,Dev2/ai54:TY,Dev2/ai55:TZ");
		// from numerical values
		//channelMap.clear();
		//channelMap.insert(std::make_pair(DAQSystem::DeviceChannel("Dev2", 50), FTSensor::CHANNEL_FX));
		//channelMap.insert(std::make_pair(DAQSystem::DeviceChannel("Dev2", 51), FTSensor::CHANNEL_FY));
		//channelMap.insert(std::make_pair(DAQSystem::DeviceChannel("Dev2", 52), FTSensor::CHANNEL_FZ));
		//channelMap.insert(std::make_pair(DAQSystem::DeviceChannel("Dev2", 53), FTSensor::CHANNEL_TX));
		//channelMap.insert(std::make_pair(DAQSystem::DeviceChannel("Dev2", 54), FTSensor::CHANNEL_TY));
		//channelMap.insert(std::make_pair(DAQSystem::DeviceChannel("Dev2", 55), FTSensor::CHANNEL_TZ));
		// create F/T sensor
		FTSensor sensor2("FT14509.cal", 1, channelMap);
		sensorSet.insert(&sensor2);

		// Initialise DAQ device
		DAQSystem daqSystem(sensorSet, DAQSystem::CONNECTION_DIFFERENTIAL, DAQSystem::SAMPLING_DIRECT, 2000, 5);

		// DAQ parameters
		printf("DAQSystem(): channel_set=%s, num_channels=%u, sampling_frequency=%.4f, samples_per_channel=%u\n", DAQSystem::createDeviceChannelSetString(daqSystem.getDeviceChannelSet()).c_str(), daqSystem.getNumChannels(), daqSystem.getSamplingFrequency(), daqSystem.getSamplesPerChannel());

		// start sampling
		daqSystem.start();

		// Create buffer for readings
		std::vector<double> buffer(daqSystem.getBuffSize());

		// Setup sensors bias to measure relative F/T values (optional)
		daqSystem.read(buffer.data(), buffer.size());
		sensor1.setBias(buffer.data());
		sensor2.setBias(buffer.data());

		// test samples
		const size_t samples = 1000;

#ifdef SAMPLING_TEST
		Timer t;
		const double t1 = t.elapsed();
		double ft1_test[FTSensor::NUM_FT_AXES] = { 0. };
		size_t updates = 0;
		std::vector<double> data;
		data.reserve(FTSensor::NUM_FT_AXES * samples);
#endif // SAMPLING_TEST

		// Read F/T values
		// integral duration: requires duration_cast
		for (size_t s = 0; s < samples; ++s) {
			// raw buffer
			daqSystem.read(buffer.data(), buffer.size());
			
			// transform to F/T
			double ft1[FTSensor::NUM_FT_AXES];
			sensor1.getValue(buffer.data(), ft1);
			double ft2[FTSensor::NUM_FT_AXES];
			sensor2.getValue(buffer.data(), ft2);
			
#ifdef SAMPLING_TEST
			bool update = false;
			for (size_t i = 0; i < FTSensor::NUM_FT_AXES; ++i) {
				if (std::abs(ft1_test[i] - ft1[i]) > 1e-18)
					update = true;
				ft1_test[i] = ft1[i];
				data.push_back(ft1[i]);
			}
			if (update)
				++updates;
#else // SAMPLING_TEST
			// print F/T values
			printf("FT1 = {(% 8.4f, % 8.4f, % 8.4f), (% 8.4f, % 8.4f, % 8.4f)}, FT2 = {(% 8.4f, % 8.4f, % 8.4f), (% 8.4f, % 8.4f, % 8.4f)}\n",
				ft1[0], ft1[1], ft1[2], ft1[3], ft1[4], ft1[5], ft2[0], ft2[1], ft2[2], ft2[3], ft2[4], ft2[5]);
#endif // SAMPLING_TEST
		}
		
#ifdef SAMPLING_TEST
		const double t2 = t.elapsed();
		printf("time=%fsec, updates=%u, frequency=%fHz\n", t2-t1, unsigned(updates), updates/(t2-t1));

		std::ofstream file("log.txt");
		for (size_t i = 0; i < data.size();) {
			for (size_t j = 0; j < FTSensor::NUM_FT_AXES; ++i, ++j)
				file << data[i] << ", ";
			file << "\n";
		}

#endif // SAMPLING_TEST
	}
	catch (const std::exception& ex) {
		fprintf(stderr, "%s\n", ex.what());
	}
}
