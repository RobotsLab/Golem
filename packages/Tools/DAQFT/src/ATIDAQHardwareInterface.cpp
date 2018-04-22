/*
ATIDAQHardwareInterface.cpp
implementation of ATIDAQHardwareInterface Class

History
Dec.13.2004 - Sam Skuce (ATI Industrial Automation) - Initial Version Started
july.22.2005 - ss - added support for users to set connection mode
dec.10.2007 - SS - Changed calls to ati_itoa to itoa_s, removed ati_itoa().
*/
#include "DAQFT/daqft/stdafx.h"
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#ifdef LINUX
#include <stdio.h>
#include <stdexcept>
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
#include "NIDAQmx.h"
#endif
#include "DAQFT/daqft/ATIDAQHardwareInterface.h"

using namespace std;

namespace DAQFT
{

#define MIN_SAMPLES_PER_CHANNEL 2 /*the minimum number of samples per channel that daqmx will allow us to specify*/


class NITaskHandle
{
public:
	TaskHandle* pTask;
};



ATIDAQHardwareInterface::ATIDAQHardwareInterface()
{
	m_pDAQTask = new NITaskHandle;
	m_pDAQTask->pTask = new TaskHandle;
	SetConnectionMode( DAQmx_Val_Diff );
}

ATIDAQHardwareInterface::~ATIDAQHardwareInterface()
{
	/*stop and clear tasks, without regard to error*/
	DAQmxClearTask( *(m_pDAQTask->pTask) );
	delete (m_pDAQTask->pTask);
	delete m_pDAQTask;
}

string ATIDAQHardwareInterface::GetErrorInfo()
{
	const size_t n = 2048;
	char errorBuffer[n];
	DAQmxGetExtendedErrorInfo( errorBuffer, n );
	m_sErrorInfo = string(errorBuffer);

	return m_sErrorInfo;
}

string ATIDAQHardwareInterface::GetErrorCodeDescription( long errCode )
{
	char errorBuffer[BUFSIZ];
#ifdef LINUX
	sprintf(errorBuffer, "ATIDAQHardwareInterface error code %d", errCode );
#else
	DAQmxGetErrorString( errCode, errorBuffer, BUFSIZ );
#endif
	return string(errorBuffer);
}

int32 rawReadFromBufferAnalogF64( TaskHandle* pTH, const int numChannels, const int numRequested,
		const float64 timeOut, float64* buf, const int buflen )
{
	// numRequested: number of samples per channel to obtain
	// buflen: size of buffer as max number of elements (not bytes)
	// will read-out whole buffer, and return the last numToRead entries
	// blocks if necessary to acquire more samples if buffer has not enough
	// !!! use this with FIFO mode for reading from DAQ device buffer !!!
	int32 numRead=0, rc;

	// @@@ check works for numRequested=1 and availPerChan=0,1

	const int buflenPerChan = buflen / numChannels;
	if ( numRequested > buflenPerChan )
		throw runtime_error("DAQFT::rawReadFromBufferAnalogF64: numRequested > buflenPerChan => reduce averaging");

	fill(buf,buf+buflen,-1.0e99); // fill with guard value

	int availPerChan;
	DAQmxGetReadAttribute(*pTH, DAQmx_Read_AvailSampPerChan, &availPerChan);

	// to empty device buffer we could request to read -1 samples ???

	// have we enough samples in the DAQ device buffer?
	if ( availPerChan >= numRequested )
	{
		// timeOut should be 0 here ???
		const int nstub = availPerChan % buflenPerChan;
		if (nstub > 0)
			rc = DAQmxReadAnalogF64( *pTH, nstub, timeOut, DAQmx_Val_GroupByScanNumber,
					buf, buflen, &numRead, NULL );
		
		// read in chunks of buflenPerChan
		const int n = availPerChan / buflenPerChan;
		for (int i = 0; i < n; ++i)
		{
			rc = DAQmxReadAnalogF64( *pTH, buflenPerChan, timeOut, DAQmx_Val_GroupByScanNumber,
				buf, buflen, &numRead, NULL );
			if (rc!=0 || numRead < buflenPerChan)
			{
				// @@@ write error log
				cerr << "rc!=0 || numRead < buflenPerChan" << endl;
				break;
			}
		}
		// last numRequested elements is what is wanted
		if (numRead > numRequested)
			copy(buf+numRead-numRequested, buf+numRead, buf);
	}
	else
	{
		// have to acquire more samples => will block
		rc = DAQmxReadAnalogF64( *pTH, numRequested, timeOut, DAQmx_Val_GroupByScanNumber,
					buf, buflen, &numRead, NULL );
	}

	return numRead;
}

int ATIDAQHardwareInterface::ReadSingleSample( vector<double>& buffer )
{
	int32 rc;
	const size_t MAX_RAW_SAMPLES = 6 * 101;
	const unsigned long numRawSamples = m_uiNumChannels * m_uiAveragingSize;

	assert( buffer.size() >= m_uiNumChannels );

	if (numRawSamples > MAX_RAW_SAMPLES)
		throw runtime_error("ATIDAQHardwareInterface::ReadSingleSample: numRawSamples > MAX_RAW_SAMPLES => reduce averaging");

	float64 rawBuffer[MAX_RAW_SAMPLES]; // holds the raw, unaveraged gauge values

	// allow a full second for Windows timing inaccuracies
	const float64 timeOut = ( m_uiAveragingSize / m_SamplingFrequency ) + 1.0;

	int32 read; // number samples read per channel

#ifdef LINUX1
	read = rawReadFromBufferAnalogF64( m_pDAQTask->pTask, m_uiNumChannels, m_uiAveragingSize, timeOut, rawBuffer, MAX_RAW_SAMPLES );
#else
	rc = DAQmxReadAnalogF64( *(m_pDAQTask->pTask), m_uiAveragingSize, timeOut, DAQmx_Val_GroupByScanNumber, 
				rawBuffer, MAX_RAW_SAMPLES, &read, NULL );
#endif

	// check that number read is what was requested
	if (read != m_uiAveragingSize)
		cerr << "*********** read != m_uiAveragingSize ***********" << endl;

	const size_t numAvg = std::min( read, (int32)m_uiAveragingSize );
	for ( size_t i = 0; i < m_uiNumChannels; ++i )
	{
		buffer[i] = 0.0;
		if (numAvg>0)
		{
			for ( size_t j = 0; j < numAvg; ++j )
				buffer[i] += rawBuffer[ i + ( j * m_uiNumChannels ) ]; // @@@ check i <-> j in RHS
			buffer[i] /= (double)numAvg;
		}
	}

	return rc;
}

int ATIDAQHardwareInterface::StopCollection()
{
	const int32 retVal = DAQmxClearTask( *(m_pDAQTask->pTask) );	
	return retVal;
}

string ATIDAQHardwareInterface::ati_Channel_Name_Format( const string& deviceName )
{
	string channelString(deviceName);

	if ( channelString.find('/') == string::npos )
	{
		// no slash, so add channel info to the name
		char s[BUFSIZ];
		sprintf(s, "%.100s/ai%d:%d", deviceName.c_str(), m_uiFirstChannel, m_uiFirstChannel + m_uiNumChannels - 1 );
		channelString = string(s);
	}

	return channelString;
}

int ATIDAQHardwareInterface::ConfigSingleSampleTask( double sampleRate, 
														int averaging, 
														const string& deviceName, 
														int firstChannel,
														int numChannels, 
														int minVoltage, 
														int maxVoltage )
{

	m_SamplingFrequency = sampleRate;
	m_uiAveragingSize = ( averaging > 0 ) ? averaging : 1; // averaging must be at least 1
	m_sDeviceName = deviceName;
	m_uiFirstChannel = firstChannel;
	m_uiNumChannels = numChannels;
	m_iMinVoltage = minVoltage;
	m_iMaxVoltage = maxVoltage;

	unsigned int numSamplesPerChannel = m_uiAveragingSize; /*the number of samples per channel that
														   daqmx is configured with*/
	/*in a perfect world, NI-DAQmx would allow us to set up single scan acquisitions, but they don't.
	even if we were to make the single sample task a finite sample task, they still require you to use
	at least 2 samples per channel.  Therefore, we pass daqmx a number of samples per channel that it
	will accept, and then set up our task to only read the most recent samples*/
	if ( MIN_SAMPLES_PER_CHANNEL > numSamplesPerChannel ) numSamplesPerChannel = MIN_SAMPLES_PER_CHANNEL;

	const string channelString = ati_Channel_Name_Format( m_sDeviceName );

	StopCollection(); // stop currently running task

	TaskHandle* pTH = m_pDAQTask->pTask;

	// if any function fails (returns non-zero), don't execute any more daqmx functions

	// @@@ consider clean up of task if a call fails
	int32 rc = DAQmxCreateTask( "", pTH );

	if( rc==0 )
		// add the analog input channels to the task - july.22.2005 - ss - now uses m_iConnectionMode
		rc = DAQmxCreateAIVoltageChan( *pTH, channelString.c_str(), "", m_iConnectionMode, 
										m_iMinVoltage, m_iMaxVoltage, DAQmx_Val_Volts, NULL );
	if( rc==0 )
		// set up timing for the task
		rc = DAQmxCfgSampClkTiming( *pTH, NULL, (float64)m_SamplingFrequency, 
					DAQmx_Val_Rising, DAQmx_Val_ContSamps, numSamplesPerChannel );

#ifdef LINUX
	// DAQmxBase has no control over buffering, so will have to read data eagerly,
	// empty the whole buffer, and, if required, rely on user to do buffering in a dedicated thread
#else
	if( rc==0 )
		// set read position relative to next sample to be read
		rc = DAQmxSetReadRelativeTo( *pTH, DAQmx_Val_MostRecentSamp );
	if( rc==0 )
	{
		// offset of -1 from the next sample, meaning we read the most recent sample
		const int32 n = m_uiAveragingSize;
		rc = DAQmxSetReadOffset( *pTH, -n ); // @@@ -n
	}
	if( rc==0 )
		rc = DAQmxSetReadOverWrite( *pTH, DAQmx_Val_OverwriteUnreadSamps ); // @@@
#endif

	if( rc==0 )
		rc = DAQmxStartTask( *pTH );						

	return rc;
}


string ATIDAQHardwareInterface::GetDeviceName()
{
	return m_sDeviceName;
}

int ATIDAQHardwareInterface::ConfigBufferTask( double sampleRate, int averaging, const string& deviceName, int firstChannel,
		int numChannels, int minVoltage, int maxVoltage, int bufferSize )
{
	int32 retVal; /*the return value*/

	m_SamplingFrequency = sampleRate;
	m_uiAveragingSize = ( averaging > 0 )? averaging : 1; /*averaging must be at least 1*/
	m_sDeviceName = deviceName;
	m_uiFirstChannel = firstChannel;
	m_uiNumChannels = numChannels;
	m_iMinVoltage = minVoltage;
	m_iMaxVoltage = maxVoltage;
	m_ulBufferedSize = bufferSize;
	unsigned int numSamplesPerChannel = m_uiAveragingSize * m_ulBufferedSize; /*the number of samples per channel that
														   daqmx is configured with*/
	/*NI-DAQmx requires a minimum number of samples per channel*/
	if ( MIN_SAMPLES_PER_CHANNEL > numSamplesPerChannel ) numSamplesPerChannel = MIN_SAMPLES_PER_CHANNEL;

	const string channelString = ati_Channel_Name_Format( m_sDeviceName );

	/*if the following confuses you, I suggest you read the NI-DAQmx C Reference Help, included
	with NI-DAQmx*/	
	StopCollection(); /*stop any currently running task*/
	/*if any function fails (returns non-zero), don't execute any more daqmx functions*/
	/*create the daqmx task*/
	if( !( retVal = DAQmxCreateTask( "", (m_pDAQTask->pTask) ) ) )
		/*add the analog input channels to the task - july.22.2005 - ss - now uses m_iConnectionMode*/
		if( !( retVal = DAQmxCreateAIVoltageChan( *(m_pDAQTask->pTask), channelString.c_str(), "", m_iConnectionMode, 
					m_iMinVoltage, m_iMaxVoltage, DAQmx_Val_Volts, NULL ) ) )
			/*set up timing for the task*/
			if( !( retVal = DAQmxCfgSampClkTiming( *(m_pDAQTask->pTask), NULL, (float64)m_SamplingFrequency, 
						DAQmx_Val_Rising, DAQmx_Val_ContSamps, numSamplesPerChannel ) ) )
				/*start the task*/
				retVal = DAQmxStartTask( *(m_pDAQTask->pTask) );						
	return retVal;
}

int ATIDAQHardwareInterface::ReadBufferedSamples( const size_t numSamples, vector<double>& buffer )
{
	int32 retVal;
	
	const int32 sampsPerChannel = m_uiAveragingSize * (int32)numSamples;
	const unsigned int numRawSamples = sampsPerChannel * m_uiNumChannels; // total number of individual gauge readings to be read

	float64* rawBuffer = new float64[numRawSamples]; // holds the raw, unaveraged gauge values

	const float64 timeOut = ( sampsPerChannel / m_SamplingFrequency ) + 1.0; // timeout value. allows a full second of slack

	int32 read;// number of samples read
	const unsigned int rawSetSize = m_uiNumChannels * m_uiAveragingSize; // the number of raw data sets per one output set

	retVal = DAQmxReadAnalogF64( *(m_pDAQTask->pTask), sampsPerChannel, timeOut, DAQmx_Val_GroupByScanNumber, 
				rawBuffer, numRawSamples, &read, NULL );
	/*
	precondition: rawBuffer has the raw samples from the DAQ hardware.  rawSetSize is the number of
		data points in one output reading (one raw data point is a single reading of all 6 or 7 gauges).
	postcondition: buffer has the output (averaged) data points.  the first data point in each raw 'set' has the
		sum of all the readings in that set. i = numSamples, j = m_uiNumChannels, k = m_uiAveragingSize
	*/
	size_t i, j, k;
	for ( i = 0; i < numSamples; i++ )
	{
		const unsigned int firstDataPointInSetIndex = (unsigned int)i * rawSetSize; /*the position of the first element
																in the first data point in the raw
																set we're currently averaging*/
		for ( j = 0; (unsigned int)j < m_uiNumChannels; j++ )
		{
			const unsigned int sumIndex = firstDataPointInSetIndex + (unsigned int)j; /*the index where we're storing
																  the sum of the gauge we're averaging
																  the raw values for*/
			for ( k = 1; (unsigned int)k < m_uiAveragingSize; k++ )
			{ /*put the sum of this set into the first reading in the set*/
                rawBuffer[ sumIndex ] += rawBuffer[ sumIndex + ( k * m_uiNumChannels ) ];			
			}
			
			/*put the averages into the output buffer*/
			buffer[ ( i * m_uiNumChannels ) + j ] = rawBuffer[ sumIndex ] / m_uiAveragingSize;
		}		
	}

	delete[] rawBuffer;

	return retVal;
}

/*july.22.2005 - ss - added SetConnectionMode*/
void ATIDAQHardwareInterface::SetConnectionMode( int DAQConnMode )
{
	m_iConnectionMode = DAQConnMode;
}

/*july.22.2005 - ss - added GetConnectionMode*/
int ATIDAQHardwareInterface::GetConnectionMode( )
{
	return m_iConnectionMode;
}


// number of samples (per channel) available in internal DAQ device buffer
int ATIDAQHardwareInterface::numSamplesInBuffer()
{
	int nb;
	DAQmxGetReadAttribute(*(m_pDAQTask->pTask), DAQmx_Read_AvailSampPerChan, &nb);
	return nb;
}


} // namespace DAQFT
