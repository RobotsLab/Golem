/** @file FTSensor.h
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

#ifndef _GOLEM_DAQFT_FTSENSOR_H
#define _GOLEM_DAQFT_FTSENSOR_H

#include <vector>
#include <set>
#include <map>
#include <string>
#include <atomic>
#include <mutex>
#include <condition_variable>

//------------------------------------------------------------------------------

namespace DAQFT {
	struct Calibration;
};

//------------------------------------------------------------------------------

namespace golem {

class Helper;
class NITaskHandle;

/** Data acquisition device interface */
class DAQSystem {
public:
	/** Channel index type */
	typedef unsigned Channel;
	/** Device, channel pair */
	struct DeviceChannel {
		/** Device name */
		std::string device;
		/** Channel index */
		Channel channel;

		DeviceChannel(const std::string& device = "Dev1", const Channel channel = 0) : device(device), channel(channel) {}

		/** Comparator */
		inline friend bool operator < (const DeviceChannel &left, const DeviceChannel &right) {
			return left.device < right.device || left.device == right.device && left.channel < right.channel;
		}
	};

	/** Unique set of device-channel pairs */
	typedef std::set<DeviceChannel> DeviceChannelSet;

	static const char CHANNEL_DEVICE_NAME [];
	static const char CHANNEL_RANGE_SEP;
	static const char CHANNEL_GROUP_SEP;

	/** Connection type */
	enum ConnectionType {
		CONNECTION_DIFFERENTIAL,
		CONNECTION_REFERENCED_SINGLE_ENDED,
		CONNECTION_NON_REFERENCED_SINGLE_ENDED,
	};

	/** Sampling type */
	enum SamplingType {
		SAMPLING_DIRECT,
		SAMPLING_CALLBACK,
	};

	/** Data acquisition sensor */
	class Sensor {
	public:
		/** Retrieve channel mapping and volatage range required by DAQSystem */
		virtual void updateDAQSystemConfig(DeviceChannelSet& set, double& minVoltage, double& maxVoltage) const = 0;
		/** Setup buffer-channel mapping (mandatory) */
		virtual void setMapping(const DAQSystem::DeviceChannelSet& set) = 0;
		/** Release resources */
		virtual ~Sensor() {}
	};

	/** Data acquisition sensor collection */
	typedef std::set<Sensor*> SensorSet;

	/** DAQ system initialisation */
	DAQSystem(const SensorSet& sensors, ConnectionType connectionType, SamplingType samplingType, double samplingFrequency, unsigned samplesPerChannel, unsigned deviceBufferSize = -1);
	/** DAQ system cleanup */
	~DAQSystem();

	/** Start sampling */
	void start(unsigned timeOutMs = -1);
	/** Stop sampling */
	void stop();
	/** Read buffer */
	bool read(double* buffer, size_t size, unsigned timeOutMs = -1);

	/** Device-channel mapping */
	const DeviceChannelSet& getDeviceChannelSet() const {
		return m_channelSet;
	}
	/** Number of channels */
	unsigned getNumChannels() const {
		return m_numChannels;
	}

	/** Sampling frequency */
	double getSamplingFrequency() const {
		return m_samplingFrequency;
	}
	/** Number of averaged samples per channel */
	unsigned getSamplesPerChannel() const {
		return m_samplesPerChannel;
	}
	
	/** Raw buffer size */
	size_t getBuffSize() const {
		return size_t(m_numChannels);
	}
	/** Buffer samples */
	size_t getBuffSamples() const {
		return m_rawBufferSamples;
	}

	/** Create channel format string */
	static std::string createDeviceChannelSetString(const DeviceChannelSet& set);

private:
	friend class Helper;

	/** Buffer */
	typedef std::vector<double> Buffer;

	/** the minimum number of samples per channel that daqmx will allow us to specify */
	static const unsigned MIN_SAMPLES_PER_CHANNEL = 1;

	/** Update raw buffer */
	bool updateRawBuffer(double timeOut);
	/** Read buffer */
	void readBuffer(double* buffer, size_t size) const;

	/** Read buffer */
	void readRawBuffer();

	/** Extract DAQ string error message */
	static std::string getLastError(int rc);

	const SamplingType samplingType;
	const double m_samplingFrequency;
	const unsigned m_samplesPerChannel;

	NITaskHandle* pNITaskHandle;

	DeviceChannelSet m_channelSet;
	unsigned m_numChannels;

	std::mutex m_bufferMutex;
	std::condition_variable m_bufferMutexCV;
	Buffer m_buffer;

	Buffer m_rawBuffer;
	size_t m_rawBufferPtr;
	
	std::atomic<size_t> m_rawBufferSamples;

	std::string m_exception;
};

/** F/T sensor interface */
class FTSensor : public DAQSystem::Sensor {
public:
	/** the number of force/torque axes */
	static const unsigned NUM_FT_AXES = 6;
	/** gauges are considered saturated when they reach 99.5% of their maximum load */
	static const double GAUGE_SATURATION_LEVEL;

	enum ChannelType {
		/** Force X */
		CHANNEL_FX = 0,
		/** Force Y */
		CHANNEL_FY = 1,
		/** Force Z */
		CHANNEL_FZ = 2,
		/** Torque X */
		CHANNEL_TX = 3,
		/** Torque Y */
		CHANNEL_TY = 4,
		/** Torque Z */
		CHANNEL_TZ = 5,
		/** Temperature compensation */
		CHANNEL_TEMP = 6,
		/** Dummy variable (not mapped) */
		CHANNEL_DUMMY,
	};

	typedef std::map<DAQSystem::DeviceChannel, ChannelType> ChannelTypeMap;

	typedef std::vector< std::pair<DAQSystem::DeviceChannel, std::pair<size_t, size_t> > > ChannelTypeIndex;

	/** Create F/T sensor */
	FTSensor(const std::string& calFile, unsigned calIndex, const ChannelTypeMap& map);
	/** Release resources */
	virtual ~FTSensor();

	/** Retrieve channel mapping and volatage range required by DAQSystem */
	virtual void updateDAQSystemConfig(DAQSystem::DeviceChannelSet& set, double& minVoltage, double& maxVoltage) const;

	/** Setup buffer-channel mapping (mandatory) */
	virtual void setMapping(const DAQSystem::DeviceChannelSet& set);

	/** Setup sensor bias (optional) */
	void setBias(const double* buff);

	/** Extract from buffer raw sensor values: raw[7] = {FX, FY, FZ, TX, TY, TZ, TEMP} */
	void getValueRaw(const double* buff, double* raw, const bool ignoreSaturation = false) const;

	/** Extract from buffer F/T sensor values: ft[6] = {FX, FY, FZ, TX, TY, TZ} */
	void getValue(const double* buff, double* ft, const bool ignoreSaturation = false) const;

	/** Create channel map from string - for example: "Dev1/ai0:FX,Dev1/ai1:FY,Dev1/ai2:FZ,Dev1/ai9:TX,Dev1/ai10:TY,Dev1/ai11:TZ,Dev1/ai16:TEMP" */
	static ChannelTypeMap createChannelTypeMap(const std::string& str);

	/** Device-channel type mapping */
	const ChannelTypeMap& getChannelTypeMap() const {
		return m_map;
	}
	/** Buffer mapping */
	const ChannelTypeIndex& getChannelTypeIndex() const {
		return m_index;
	}

private:
	DAQFT::Calibration* pCalibration;

	const ChannelTypeMap m_map;
	ChannelTypeIndex m_index;
	double ftGain[NUM_FT_AXES];
	
	/** the maximum voltage of the gauges in the transducer */
	int m_iMaxVoltage;
	/** the minimum voltage of the gauges in the transducer */
	int m_iMinVoltage;
	/** the upper voltage at which the gauges are considered saturated */
	double m_dUpperSaturationVoltage;
	/** the lower voltage at which the gauges are considered saturated */
	double m_dLowerSaturationVoltage;
};

} // namespace DAQFT

//------------------------------------------------------------------------------

#endif /* _GOLEM_DAQFT_FTSENSOR_H */