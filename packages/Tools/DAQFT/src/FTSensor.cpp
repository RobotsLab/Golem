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

#include <stdlib.h>
#include <algorithm>
#include <sstream>
#include <stdarg.h>
#include <stdexcept>
#include <chrono>

#ifdef LINUX
#include <unistd.h> // sleep

#include "NIDAQmxBase.h"
#define DAQmxClearTask DAQmxBaseClearTask
#define DAQmxCreateTask DAQmxBaseCreateTask
#define DAQmxStartTask DAQmxBaseStartTask
#define DAQmxCreateAIVoltageChan DAQmxBaseCreateAIVoltageChan
#define DAQmxCfgSampClkTiming DAQmxBaseCfgSampClkTiming
#define DAQmxGetReadAttribute DAQmxBaseGetReadAttribute
#define DAQmxReadAnalogF64 DAQmxBaseReadAnalogF64
#define DAQmxGetExtendedErrorInfo DAQmxBaseGetExtendedErrorInfo
#else
#define NOMINMAX
#include <Windows.h> // sleep

#include "NIDAQmx.h"
#endif
#include <DAQFT/daqft/ftconfig.h>

#include <Golem/DAQFT/FTSensor.h>

using namespace golem;
using namespace DAQFT;

//------------------------------------------------------------------------------

namespace golem {
class Helper {
public:
	static char* vsnprintf(char* begin, const char* end, const char* format, va_list argptr) {
		if (begin >= end)
			return begin;

		const size_t size = end - begin - 1;

#ifdef WIN32
#pragma warning (push)
#pragma warning (disable:4996)
#define vsnprintf _vsnprintf
#endif

		const int inc = ::vsnprintf(begin, size, format, argptr);

#ifdef WIN32
#undef vsnprintf
#pragma warning (pop)
#endif

		if (inc < 0)
			begin += size;
		else if (size > size_t(inc))
			return begin + inc; // with NULL character
		else
			begin += inc;

		*begin = '\0'; // always terminate with NULL character

		return begin;
	}

	static void Helper::assertValid(bool valid, const char* format, ...) {
		if (!valid) {
			char buf[BUFSIZ];

			va_list argptr;
			va_start(argptr, format);
			(void)vsnprintf(buf, buf + sizeof(buf), format, argptr);
			va_end(argptr);

			throw std::runtime_error(std::string(buf));
		}
	}

	static void Helper::assertValidNI(int rc, const char* format, ...) {
		if (rc != 0) {
			char buf[BUFSIZ];

			va_list argptr;
			va_start(argptr, format);
			char* ptr = vsnprintf(buf, buf + sizeof(buf), format, argptr);
			va_end(argptr);

			throw std::runtime_error(std::string(buf) + DAQSystem::getLastError(rc));
		}
	}

#ifdef LINUX
	static void Sleep(const unsigned ms) {
		sleep(ms / 1000);
	}
#endif

	static int32 CVICALLBACK callback(TaskHandle taskHandle, int32 signalID, void *callbackData) {
		reinterpret_cast<DAQSystem*>(callbackData)->readRawBuffer();
		return 0;
	}
};

class NITaskHandle {
public:
	TaskHandle* pTask;
	NITaskHandle() : pTask(new TaskHandle) {}
	~NITaskHandle() {
		(void)DAQmxClearTask(*pTask);
		delete pTask;
	}
};
};

//------------------------------------------------------------------------------

const char DAQSystem::CHANNEL_DEVICE_NAME[] = "/ai";
const char DAQSystem::CHANNEL_RANGE_SEP = ':';
const char DAQSystem::CHANNEL_GROUP_SEP = ',';

std::string DAQSystem::getLastError(int rc) {
	char buf[2048];
	//(void)DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
	(void)DAQmxGetErrorString(rc, buf, sizeof(buf));
	return std::string(buf);
}

std::string DAQSystem::createDeviceChannelSetString(const DeviceChannelSet& set) {
	Helper::assertValid(!set.empty(),
		"DAQSystem::createDeviceChannelSetString(): empty device-channel set");

	std::stringstream str;

	for (DeviceChannelSet::const_iterator next = set.begin(), first = set.begin(), last = set.begin();;) {
		if (++next == set.end() || last->device != next->device || last->channel + 1 < next->channel) {
			str << first->device << CHANNEL_DEVICE_NAME << first->channel;
			if (first != last)
				str << CHANNEL_RANGE_SEP << last->channel;

			// finish
			if (next == set.end())
				break;

			first = next;
			// there will be a new range group
			str << CHANNEL_GROUP_SEP;
		}
		last = next;
	}
	
	return str.str();
}

DAQSystem::DAQSystem(const SensorSet& sensors, ConnectionType connectionType, SamplingType samplingType, double samplingFrequency, unsigned samplesPerChannel, unsigned deviceBufferSize) :
	pNITaskHandle(new NITaskHandle),
	samplingType(samplingType),
	m_samplingFrequency(samplingFrequency),
	m_samplesPerChannel(std::max((unsigned)1, samplesPerChannel)),
	m_rawBufferPtr(0),
	m_rawBufferSamples(0)
{
	Helper::assertValid(!sensors.empty(),
		"DAQSystem::DAQSystem(): no specified sensors");

	// update DAQ parameters
	m_channelSet.clear();
	double minVoltage = 0.;
	double maxVoltage = 0.;
	for (SensorSet::const_iterator sensor = sensors.begin(); sensor != sensors.end(); ++sensor) {
		Helper::assertValid(*sensor != nullptr,
			"DAQSystem::DAQSystem(): invalid sensor pointer");

		(*sensor)->updateDAQSystemConfig(m_channelSet, minVoltage, maxVoltage);
	}

	// setup buffer channel mapping for each sensor
	for (SensorSet::const_iterator sensor = sensors.begin(); sensor != sensors.end(); ++sensor) {
		(*sensor)->setMapping(m_channelSet);
	}

	// number of channels
	m_numChannels = (unsigned)m_channelSet.size();

	// channel format string
	const std::string channelString = createDeviceChannelSetString(m_channelSet);

	// task
	Helper::assertValidNI(DAQmxCreateTask("", pNITaskHandle->pTask),
		"DAQSystem::DAQSystem(): DAQmxCreateTask(): ");
	
	// add the analog input channels to the task
	int32 terminalConfig = DAQmx_Val_Diff;
	switch (connectionType) {
	case CONNECTION_DIFFERENTIAL: terminalConfig = DAQmx_Val_Diff; break;
	case CONNECTION_REFERENCED_SINGLE_ENDED: terminalConfig = DAQmx_Val_RSE; break;
	case CONNECTION_NON_REFERENCED_SINGLE_ENDED: terminalConfig = DAQmx_Val_NRSE; break;
	};
	Helper::assertValidNI(DAQmxCreateAIVoltageChan(*pNITaskHandle->pTask, channelString.c_str(), "", terminalConfig, minVoltage, maxVoltage, DAQmx_Val_Volts, NULL),
		"DAQSystem::DAQSystem(): DAQmxCreateAIVoltageChan(): ");

	// set up timing for the task
	Helper::assertValidNI(DAQmxCfgSampClkTiming(*pNITaskHandle->pTask, NULL, (float64)m_samplingFrequency, DAQmx_Val_Rising, DAQmx_Val_ContSamps, MIN_SAMPLES_PER_CHANNEL),
		"DAQSystem::DAQSystem(): DAQmxCfgSampClkTiming(): ");

	// set up callback
	if (samplingType == SAMPLING_CALLBACK)
		Helper::assertValidNI(DAQmxRegisterSignalEvent(*pNITaskHandle->pTask, DAQmx_Val_SampleCompleteEvent,
#ifdef LINUX
		0,
#else
		0,
		//DAQmx_Val_SynchronousEventCallbacks, // fails in Windows
#endif
		Helper::callback, this),
		"DAQSystem::DAQSystem(): DAQmxRegisterSignalEvent(): ");

	// DAQ device buffer size adjustment is optional
	if (deviceBufferSize != -1)
		Helper::assertValidNI(DAQmxCfgInputBuffer(*pNITaskHandle->pTask, deviceBufferSize),
			"DAQSystem::DAQSystem(): DAQmxCfgInputBuffer(): ");

	//Helper::assertValidNI(DAQmxSetReadRelativeTo(*pNITaskHandle->pTask, DAQmx_Val_MostRecentSamp), "DAQSystem::DAQSystem(): DAQmxSetReadRelativeTo(): ");
	//Helper::assertValidNI(DAQmxSetReadOffset(*pNITaskHandle->pTask, -int32(m_samplesPerChannel)), "DAQSystem::DAQSystem(): DAQmxSetReadOffset(): ");
	//Helper::assertValidNI(DAQmxSetReadOverWrite(*pNITaskHandle->pTask, DAQmx_Val_OverwriteUnreadSamps), "DAQSystem::DAQSystem(): DAQmxSetReadOverWrite(): ");

	// allocate buffers
	m_rawBuffer.resize(m_numChannels * m_samplesPerChannel, 0.);
	m_buffer.resize(m_numChannels, 0.);
}

DAQSystem::~DAQSystem() {
	// do not throw
	try {
		stop();
	}
	catch (...) {}

	delete pNITaskHandle;
}

void DAQSystem::start(unsigned timeOutMs) {
	// reset buffer variables
	m_rawBufferPtr = 0;
	m_rawBufferSamples = 0;
	// exception
	m_exception.clear();

	// start acquisition
	Helper::assertValidNI(DAQmxStartTask(*pNITaskHandle->pTask),
		"DAQSystem::start(): DAQmxStartTask(): ");

	// wait until buffer is filled out
	if (samplingType == SAMPLING_CALLBACK) {
		std::unique_lock<std::mutex> lock(m_bufferMutex);
		Helper::assertValid(m_bufferMutexCV.wait_for(lock, std::chrono::milliseconds(timeOutMs), [=]() -> bool {
			// check exception
			if (m_exception.length() > 0)
				Helper::assertValid(false, "%s", m_exception.data());
			
			// test number of samples
			return (unsigned)getBuffSamples() >= m_samplesPerChannel;
		}),
			"DAQSystem::start(): reading time out");
	}
	else {
		// update buffer
		for (unsigned i = 0; i < m_samplesPerChannel - 1; ++i)
			Helper::assertValid(updateRawBuffer(1e-3*timeOutMs),
			"DAQSystem::start(): reading time out");
	}
}

void DAQSystem::stop() {
	// stop acquisition
	Helper::assertValidNI(DAQmxStopTask(*pNITaskHandle->pTask),
		"DAQSystem::start(): DAQmxStopTask(): ");
}

bool DAQSystem::updateRawBuffer(double timeOut) {
	// current buffer pointer
	double* rawBufferPtr = m_rawBuffer.data() + m_numChannels*m_rawBufferPtr;

	// number samples read per channel
	int32 read = 0;

	// have to acquire more samples
	Helper::assertValidNI(DAQmxReadAnalogF64(*pNITaskHandle->pTask, 1, timeOut > 1. ? 1. : timeOut, DAQmx_Val_GroupByScanNumber, rawBufferPtr, m_numChannels, &read, NULL),
		"DAQSystem::updateRawBuffer(): DAQmxReadAnalogF64(): ");

	// check that number read is what was requested
	if (read <= 0)
		return false;

	// update pointers
	m_rawBufferPtr = (m_rawBufferPtr + 1) % m_samplesPerChannel;
	++m_rawBufferSamples;

	return true;
}

void DAQSystem::readBuffer(double* buffer, size_t size) const {
	//Helper::assertValid(size >= m_numChannels,
	//	"DAQSystem::readBuffer(): Too small buffer: %u < %u", size, m_numChannels);

	// update buffer
	const unsigned samples = std::max((unsigned)m_rawBufferPtr, (unsigned)m_samplesPerChannel);
	for (unsigned i = 0; i < m_numChannels; ++i) {
		buffer[i] = 0.0;
		if (samples > 0) {
			for (unsigned j = 0; j < samples; ++j)
				buffer[i] += m_rawBuffer[i + (j * m_numChannels)];
			buffer[i] /= (double)samples;
		}
	}
}

void DAQSystem::readRawBuffer() {
	try {
		// allow a full second for Windows timing inaccuracies
		const float64 timeOut = (1.0 / m_samplingFrequency) + 1.0;

		// update buffer
		if (!updateRawBuffer(timeOut))
			return;

		std::lock_guard<std::mutex> guard(m_bufferMutex);

		// read buffer
		readBuffer(m_buffer.data(), m_buffer.size());
	}
	catch (const std::exception& ex) {
		std::lock_guard<std::mutex> guard(m_bufferMutex);
		
		// copy exception
		m_exception = ex.what();
	}

	m_bufferMutexCV.notify_all();
}

bool DAQSystem::read(double* buffer, size_t size, unsigned timeOutMs) {
	Helper::assertValid(size >= m_numChannels,
		"DAQSystem::read(): Too small buffer: %u < %u", size, m_numChannels);

	if (samplingType == SAMPLING_CALLBACK) {
		std::unique_lock<std::mutex> lock(m_bufferMutex);
		if (m_bufferMutexCV.wait_for(lock, std::chrono::milliseconds(timeOutMs)) == std::cv_status::timeout)
			return false;

		// check exception
		if (m_exception.length() > 0)
			Helper::assertValid(false, "%s", m_exception.data());

		// copy buffer
		std::copy(m_buffer.begin(), m_buffer.end(), buffer);
	}
	else {
		// update buffer
		Helper::assertValid(updateRawBuffer(1e-3*timeOutMs),
			"DAQSystem::read(): reading time out");

		// read buffer
		readBuffer(buffer, size);
	}

	return true;
}

//------------------------------------------------------------------------------

const double FTSensor::GAUGE_SATURATION_LEVEL = 0.995;

FTSensor::ChannelTypeMap FTSensor::createChannelTypeMap(const std::string& str) {
	FTSensor::ChannelTypeMap map;

	for (size_t pos = 0; ;) {
		pos = str.find(DAQSystem::CHANNEL_DEVICE_NAME, pos);
		if (pos == std::string::npos)
			break;

		// device name
		std::string device("");
		for (size_t i = pos; i > 0 && str[--i] != DAQSystem::CHANNEL_GROUP_SEP;)
			device.insert(device.begin(), str[i]);

		pos += std::strlen(DAQSystem::CHANNEL_DEVICE_NAME);

		// channel index
		std::string channel("");
		for (; pos < str.length() && str[pos] != DAQSystem::CHANNEL_RANGE_SEP; ++pos)
			channel.insert(channel.end(), str[pos]);

		pos += 1;

		// channel type
		std::string type("");
		for (; pos < str.length() && str[pos] != DAQSystem::CHANNEL_GROUP_SEP; ++pos)
			type.insert(type.end(), str[pos]);
		std::transform(type.begin(), type.end(), type.begin(), ::tolower);

		// add new channel
		map.insert(std::make_pair(DAQSystem::DeviceChannel(
			device,
			(DAQSystem::Channel)std::strtol(channel.data(), nullptr, 10)),
			type.compare("fx") == 0 ? FTSensor::CHANNEL_FX : type.compare("fy") == 0 ? FTSensor::CHANNEL_FY : type.compare("fz") == 0 ? FTSensor::CHANNEL_FZ :
			type.compare("tx") == 0 ? FTSensor::CHANNEL_TX : type.compare("ty") == 0 ? FTSensor::CHANNEL_TY : type.compare("tz") == 0 ? FTSensor::CHANNEL_TZ :
			type.compare("temp") == 0 ? FTSensor::CHANNEL_TEMP : FTSensor::CHANNEL_DUMMY
		));
	}

	return map;
}

FTSensor::FTSensor(const std::string& calFile, unsigned calIndex, const ChannelTypeMap& map) :
	pCalibration(createCalibration(calFile.c_str(), (unsigned short)calIndex)),
	m_map(map)
{
	Helper::assertValid(pCalibration != nullptr,
		"FTSensor::FTSensor(): Failed to create calibration");

	// determine max and min voltages and saturation levels
	m_iMinVoltage = 0;
	m_iMaxVoltage = pCalibration->VoltageRange;
	if (pCalibration->BiPolar) {
		m_iMinVoltage -= (pCalibration->VoltageRange / 2);
		m_dLowerSaturationVoltage = m_iMinVoltage * GAUGE_SATURATION_LEVEL;
		m_iMaxVoltage -= (pCalibration->VoltageRange / 2);
		m_dUpperSaturationVoltage = m_iMaxVoltage * GAUGE_SATURATION_LEVEL;
	}

	// temp compensation
	pCalibration->cfg.TempCompEnabled = false;
	for (ChannelTypeMap::const_iterator i = map.begin(); i != map.end(); ++i)
		if (i->second == CHANNEL_TEMP)
			pCalibration->cfg.TempCompEnabled = true;

	// force gain: NewUnits: units for force output ("lb","klb","N","kN","g","kg")
	const std::string ForceUnits(pCalibration->ForceUnits);
	ftGain[0] = ftGain[1] = ftGain[2] = ForceUnits.compare("kN") == 0 ? 1e+3 : 1.;

	// torque gain: NewUnits: units for torque output ("in-lb","ft-lb","N-m","N-mm","kg-cm")
	const std::string TorqueUnits(pCalibration->TorqueUnits);
	ftGain[3] = ftGain[4] = ftGain[5] = TorqueUnits.compare("N-mm") == 0 ? 1e-3 : 1.;
}

FTSensor::~FTSensor() {
	if (pCalibration != nullptr)
		destroyCalibration(pCalibration);
}

void FTSensor::updateDAQSystemConfig(DAQSystem::DeviceChannelSet& set, double& minVoltage, double& maxVoltage) const {
	// add channels, make sure they are not in use
	for (ChannelTypeMap::const_iterator i = m_map.begin(); i != m_map.end(); ++i) {
		Helper::assertValid(set.find(i->first) == set.end(), "FTSensor::updateDAQSystemConfig(): device-channel pair {%s, %u} already used", i->first.device.c_str(), i->first.channel);
		set.insert(i->first);
	}

	// update min/max ranges, make sure they are the same
	const int testMinVoltage = static_cast<int>(::floor(minVoltage + 0.5));
	Helper::assertValid(testMinVoltage == 0 || testMinVoltage == m_iMinVoltage, "FTSensor::updateDAQSystemConfig(): Min voltage %i != %i", testMinVoltage, m_iMinVoltage);
	minVoltage = static_cast<double>(m_iMinVoltage);
	const int testMaxVoltage = static_cast<int>(::floor(maxVoltage + 0.5));
	Helper::assertValid(testMaxVoltage == 0 || testMaxVoltage == m_iMaxVoltage, "FTSensor::updateDAQSystemConfig(): Max voltage %i != %i", testMaxVoltage, m_iMaxVoltage);
	maxVoltage = static_cast<double>(m_iMaxVoltage);
}

void FTSensor::setMapping(const DAQSystem::DeviceChannelSet& set) {
	// buffer-channel mapping
	m_index.clear();
	size_t index = 0;
	for (DAQSystem::DeviceChannelSet::const_iterator i = set.begin(); i != set.end(); ++i, ++index) {
		const ChannelTypeMap::const_iterator j = m_map.find(*i);

		if (j != m_map.end() && j->second != CHANNEL_DUMMY) // ignore dummy
			m_index.push_back(std::make_pair(j->first, std::make_pair((size_t)j->second, index)));
	}
}

void FTSensor::setBias(const double* buff) {
	// optional
	if (buff) {
		double raw[NUM_FT_AXES + 1];
		getValueRaw(buff, raw);

		float raw_flt[NUM_FT_AXES + 1];
		for (size_t i = 0; i < NUM_FT_AXES + 1; i++)
			raw_flt[i] = (float)raw[i];

		Bias(pCalibration, raw_flt);
	}
}

void FTSensor::getValueRaw(const double* buff, double* raw, const bool ignoreSaturation) const {
	for (ChannelTypeIndex::const_iterator i = m_index.begin(); i != m_index.end(); ++i) {
		const double val = buff[i->second.second];

		Helper::assertValid(ignoreSaturation || val >= m_dLowerSaturationVoltage && val <= m_dUpperSaturationVoltage,
			"FTSensor::getValueRaw(): %s%s%u gauge saturation: %.4f not in range <%.4f, %.4f>",
			i->first.device.c_str(), DAQSystem::CHANNEL_DEVICE_NAME, i->first.channel, val, m_dLowerSaturationVoltage, m_dUpperSaturationVoltage);
		
		raw[i->second.first] = val;
	}
}

void FTSensor::getValue(const double* buff, double* ft, const bool ignoreSaturation) const {
	double raw[NUM_FT_AXES + 1];
	getValueRaw(buff, raw, ignoreSaturation);

	float raw_flt[NUM_FT_AXES + 1];
	for (size_t i = 0; i < NUM_FT_AXES + 1; i++)
		raw_flt[i] = (float)raw[i];

	float ft_flt[NUM_FT_AXES];
	ConvertToFT(pCalibration, raw_flt, ft_flt);

	for (size_t i = 0; i < NUM_FT_AXES; i++)
		ft[i] = ftGain[i] * ft_flt[i];
}

//------------------------------------------------------------------------------
