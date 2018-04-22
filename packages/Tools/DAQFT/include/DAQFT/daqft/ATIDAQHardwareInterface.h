/*ATIDAQHardwareInterface.h
header file for ATIDAQHardwareInterface Class

history
Dec.13.2004 - Sam Skuce (ATI Industrial Automation) - Initial Revision Started.  Works
	only with NIDAQmx hardware.
July.22.2005 - ss - added support for user setting connection mode
*/
#ifndef _ATIDAQHARDWAREINTERFACE_H
#define _ATIDAQHARDWAREINTERFACE_H

#include <vector>
#include <string>


namespace DAQFT
{
class NITaskHandle; // opaque decl

class ATIDAQHardwareInterface
{
public:
	ATIDAQHardwareInterface(); /*constructor*/
	~ATIDAQHardwareInterface(); /*destructor*/

	/*
	int32 ConfigSingleSampleTask( float64 sampleRate, int averaging, const std::string& deviceName, int firstChannel,
		int numChannels, float64 minVoltage, float64 maxVoltage )
	configures the single sample task
	arguments:
		sampleRate - the rate at which to sample the hardware
		averaging - the number of raw samples to average together to make one output sample.  useful for
			reducing noise
		deviceName - the name of the NI-DAQmx device which the transducer is attached to
		firstChannel - the lowest-numbered channel that the transducer is attached to
		numChannels - the number of channels that the transducer uses.  should be 6 if temperature compensation
			is not being used, 7 if it is.
		minVoltage - the minimum voltage output by the transducer, usually -10 or -5
		maxVoltage - the maximum voltage output by the transducer, usually 10 or 5
	returns: NI-DAQmx error code (0 if successful, <0 if error, >0 if warning).
	*/
	int ConfigSingleSampleTask( double sampleRate, int averaging, const std::string& deviceName, int firstChannel,
		int numChannels, int minVoltage, int maxVoltage );

	/*
	int32 StopCollection();
	stops and clears the currently running DAQ task, freeing it's resources
	returns: NI-DAQmx error code (0 if successful, <0 if error, >0 if warning).
	*/
	int StopCollection();

	/*
	int32 ReadSingleSample( std::vector<float64>& buffer )
	reads a single sample from the card, performing any averaging necessary.
	arguments:
		buffer - out - a single sample, containing 6 or 7 gauge values (7 if software temp comp is enabled)
	returns:
		NI-DAQmx return code ( 0 if success, <0 if error, >0 if warning )
	*/
	int ReadSingleSample( std::vector<double>& buffer );

	/*
	std::string GetErrorInfo()
	get a description of the most recent error
	returns:
		NIDAQmx Error Info
	side effects:
		sets m_sErrorInfo to description
	*/
	std::string  GetErrorInfo();	

	/*
	GetMinVoltage()
	get the minimum voltage of the transducer
	returns:
		the minimum voltage of the transducer
	*/
	int GetMinVoltage() { return m_iMinVoltage; }

	/*
	GetMaxVoltage()
	get the maximum voltage of the transducer
	returns:
		the maximum voltage of the transducer
	*/
	int GetMaxVoltage() { return m_iMaxVoltage; }
	
	/*
	GetFirstChannel()
	get the first channel that the transducer is connected to
	returns:
		the number of the first (lowest-numbered) channel that the transducer is connected
		to
	*/
	unsigned int GetFirstChannel() { return m_uiFirstChannel; }
	    
	/*
	GetDeviceName()
	get the name of the DAQ device that the transducer is connected to
	returns:
		the name of the device that the transducer is connected to
	*/
	std::string GetDeviceName();

	/*
	GetNumChannels()
	get the number of channels used by the transducer, should be 6 if software temperature compensation
		is not used, or 7 if it is used.
	returns:
		the number of channels used by the transducer
	*/
	unsigned int GetNumChannels() { return m_uiNumChannels; }

	/*
	GetSampleFrequency()
	get the frequency at which to sample the transducer
	returns:
		the frequency at which the transducer is sampled
	*/
	double GetSampleFrequency() { return m_SamplingFrequency; }

	/*
	GetAveragingSamples()
	get the number of samples which are averaged together for noise reduction
	returns:
		the number of raw samples which are averaged together to form one output
		sample
	*/
	unsigned int GetAveragingSamples() { return m_uiAveragingSize; }	

	/*
	std::string  GetErrorCodeDescription( long errCode )
	get the description of a known error code
	arguments:
		errCode - the error code you wish to get a description for
	returns:
		a std::string with the description of the error
	*/
	std::string  GetErrorCodeDescription( long errCode );


	/*
	int32 ConfigBufferTask( float64 sampleRate, int averaging, const std::string& deviceName, int firstChannel,
		int numChannels, float64 minVoltage, float64 maxVoltage, int bufferSize )
	configures and starts buffered acquisition task
	arguments:
		sampleRate - the rate at which to sample the hardware
		averaging - the number of raw samples to average together to make one output sample.  useful for
			reducing noise
		deviceName - the name of the NI-DAQmx device which the transducer is attached to
		firstChannel - the lowest-numbered channel that the transducer is attached to
		numChannels - the number of channels that the transducer uses.  should be 6 if temperature compensation
			is not being used, 7 if it is.
		minVoltage - the minimum voltage output by the transducer, usually -10 or -5
		maxVoltage - the maximum voltage output by the transducer, usually 10 or 5
		bufferSize - the size of the buffer to allocate for the acquisition.  This is the number of output samples
			(after averaging) you want in the buffer, so the actual number of raw samples in the buffer will be
			( bufferSize * averaging )
	returns: NI-DAQmx error code (0 if successful, <0 if error, >0 if warning).
	*/
	int ConfigBufferTask( double sampleRate, int averaging, const std::string& deviceName, int firstChannel,
		int numChannels, int minVoltage, int maxVoltage, int bufferSize );
	/*
	int 32 ReadBufferedSamples( int numSamples, std::vector<float64>& buffer )
	reads buffered samples from the DAQ hardware, performing any averaging necessary
	arguments:
		numSamples - the number of samples to output
		buffer - out - the averaged output samples from the DAQ hardware.		
	*/
	int ReadBufferedSamples( const size_t numSamples, std::vector<double>& buffer );

	/*
	july.22.2005 - ss  added
	void SetConnectionMode( int DAQConnMode )
	sets the DAQ connection mode
	arguments:
		DAQConnMode - the connection mode for the DAQ device to use
	side effects:
		sets m_iConnectionMode to DAQConnMode
	*/
	void SetConnectionMode( int DAQConnMode );

	/*
	july.22.2005 - ss - added
	int GetConnectionMode( )
	get the DAQ connection mode
	returns:
		the connection mode of the DAQ device
	*/
	int GetConnectionMode( );
	
	int numSamplesInBuffer(); // number of samples (per channel) available in internal DAQ device buffer

private:
	// format routine for National Instruments channel names
	std::string ati_Channel_Name_Format( const std::string& deviceName );


private:
	NITaskHandle*		m_pDAQTask;		/*the task which is used to get a data*/	
	unsigned int		m_uiAveragingSize;	/*the number of samples to average together to smooth data*/
	unsigned int		m_UobNumFtSensors;	/*Number of ft sensors connected to the NI-DAQmx card*/
	double				m_SamplingFrequency; /*the frequency at which to sample data, applies to both buffered and
													single-point acquisitions*/
	unsigned long		m_ulBufferedSize;	/*the buffer size to use with buffered continuous acquisition*/
	unsigned int		m_uiNumChannels;	/*the number of channels to sample, should always be either 6 (no software
												temp comp.) or 7 (with software temp comp)*/	
	std::string			m_sDeviceName ;		/*the name of the NI-DAQmx device which the transducer is attached to*/
	unsigned int		m_uiFirstChannel;	/*the first channel occupied by the transducer*/
	int					m_iMinVoltage;		/*the minimum voltage*/
	int					m_iMaxVoltage;		/*the maximum voltage*/
	std::string			m_sErrorInfo;		/*information about the last error*/
	int					m_iConnectionMode;	/*connection mode of the DAQ device - july.22.2005 - ss*/
}; // ATIDAQHardwareInterface

} // namespace DAQFT

#endif /* _ATIDAQHARDWAREINTERFACE_H */