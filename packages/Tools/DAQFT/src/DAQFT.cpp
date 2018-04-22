// This is the main DLL file.

/*modifications
july.22.2005 - Sam Skuce(ATI Industrial Automation) - added support for user setting connection mode
aug.5.2005a - ss - added GetHardwareTempComp
may.21.2008 - ss - added support for buffered gauge collection.
*/

#include "DAQFT/daqft/stdafx.h"
#ifdef LINUX
#include "NIDAQmxBase.h"
#else
#include "NIDAQmx.h"
#define GOT_PSEUD
#endif
#include "DAQFT/daqft/ftconfig.h"
#include "DAQFT/daqft/ATIDAQHardwareInterface.h"
#include "DAQFT/daqft/DAQFT.h"

#define FIRST_TORQUE_INDEX 3 /*the index of the first torque reading in 
								the standard output list order (fx, fy, fz, tx, ty, tz)*/
#define NUM_STRAIN_GAUGES 6 /*the number of strain gauges*/
#define NUM_MATRIX_ELEMENTS 36		/*the total number of elements in a calibration matrix*/
#define GAUGE_SATURATION_LEVEL 0.995 /*gauges are considered saturated when they reach 99.5% of their maximum load*/

using namespace std;

namespace DAQFT
{


FTSystem::FTSystem() :
m_hiHardware( new ATIDAQHardwareInterface ), m_Calibration ( NULL ),
m_iMaxVoltage( 10 ), m_iMinVoltage( -10 )
{	
	m_dUpperSaturationVoltage = m_iMaxVoltage * GAUGE_SATURATION_LEVEL;
	m_dLowerSaturationVoltage = m_iMinVoltage * GAUGE_SATURATION_LEVEL;
}

FTSystem::~FTSystem()
{
	destroyCalibration( m_Calibration );
}

int FTSystem::LoadCalibrationFile( const string& calFile, int calibrationIndex )
{
	// first, unload any currently loaded calibration
	if( m_Calibration != NULL )
		destroyCalibration( m_Calibration );

	m_Calibration = createCalibration( calFile.c_str(), (unsigned short)calibrationIndex );

	if ( m_Calibration == NULL )
	{
		m_sErrorInfo = string( "Could not load calibration file successfully" );
		return -1;
	}	
	/*determine max and min voltages and saturation levels*/
	m_iMinVoltage = 0;
	m_iMaxVoltage = m_Calibration->VoltageRange;
	if ( m_Calibration->BiPolar )
	{
		m_iMinVoltage -= ( m_Calibration->VoltageRange / 2 );
		m_dLowerSaturationVoltage = m_iMinVoltage * GAUGE_SATURATION_LEVEL;
		m_iMaxVoltage -= ( m_Calibration->VoltageRange / 2 );
		m_dUpperSaturationVoltage = m_iMaxVoltage * GAUGE_SATURATION_LEVEL;
	}
	return 0;
}


int FTSystem::ReadSingleGaugePoint( vector<double>& gaugeValues )
{
	if ( m_hiHardware->ReadSingleSample( gaugeValues ) )
	{
		m_sErrorInfo = m_hiHardware->GetErrorInfo();
		return -1;
	}

	for( size_t i = 0; i < m_hiHardware->GetNumChannels(); i++ )
	{
		if ( ( gaugeValues[i] > m_dUpperSaturationVoltage ) || ( gaugeValues[i] < m_dLowerSaturationVoltage ) )
		{
			m_sErrorInfo = string("Gauge Saturation");
			return 2;
		}		
	}
	return 0;
}

string  FTSystem::GetErrorInfo()
{
	return m_sErrorInfo;
}


string  FTSystem::GetDeviceName() { return m_hiHardware->GetDeviceName(); }
		
double FTSystem::GetSampleFrequency( ) { return m_hiHardware->GetSampleFrequency(); }

int FTSystem::GetAveragingSize() { return m_hiHardware->GetAveragingSamples(); }

int FTSystem::GetFirstChannel() { return m_hiHardware->GetFirstChannel(); }


int FTSystem::StartSingleSampleAcquisition( const string& deviceName, 
																double sampleFrequency, 
																int averaging, 
																int firstChannel, 
																bool useTempComp )
{
	int numChannels = NUM_STRAIN_GAUGES + ( (useTempComp)?1:0 );

	const int status = m_hiHardware->ConfigSingleSampleTask( sampleFrequency, 
													averaging, 
													deviceName, 
													firstChannel, 
													numChannels, 
													m_iMinVoltage, 
													m_iMaxVoltage );
	if ( status )
	{
		if ( status < 0 ) /*hardware error*/
			m_sErrorInfo = m_hiHardware->GetErrorInfo();
		else /*hardware warning*/
			m_sErrorInfo = m_hiHardware->GetErrorCodeDescription( status );
		return -1;
	}
	/*set calibration's use of temp comp*/
	if ( NULL != m_Calibration )
		m_Calibration->cfg.TempCompEnabled = useTempComp;
	return 0;
}

string FTSystem::GetSerialNumber()
{
	if ( NULL == m_Calibration )
	{
		return string();
	}
	return string( m_Calibration->Serial );
}

string  FTSystem::GetCalibrationDate()
{
	if ( NULL == m_Calibration )
	{
		return string();
	}
	return string( m_Calibration->CalDate );
}

double FTSystem::GetMaxLoad( int axisIndex )
{
	if ( NULL == m_Calibration )
	{
		return 0;
	}
	double retVal;

	/*find the maximum load in the current output units, not necessarily the same as the
	calibration units*/
	retVal = m_Calibration->MaxLoads[axisIndex]; /*this is the max load in the calibration
												units*/
	if ( axisIndex < FIRST_TORQUE_INDEX ) /*this is a force axis, convert to output force units*/
	{
		retVal *= ForceConv( m_Calibration->cfg.ForceUnits ) / ForceConv( m_Calibration->ForceUnits );
	} else /*this is a torque axis, convert to output torque units*/
	{
		retVal *= TorqueConv( m_Calibration->cfg.TorqueUnits ) / TorqueConv( m_Calibration->TorqueUnits );
	}
	return retVal;
}

string  FTSystem::GetForceUnits()
{
	if ( NULL == m_Calibration )
		return string();
	return string( m_Calibration->cfg.ForceUnits );
}


int FTSystem::SetForceUnits( const string& forceUnits )
{
	return DAQFT::SetForceUnits( m_Calibration, (Units)forceUnits.c_str() );
}

int FTSystem::SetTorqueUnits( const string& torqueUnits )
{
	return DAQFT::SetTorqueUnits( m_Calibration, (Units)torqueUnits.c_str() );
}

string FTSystem::GetTorqueUnits( )
{
	if ( NULL == m_Calibration )
		return string();
	return string( m_Calibration->cfg.TorqueUnits );
}

bool FTSystem::GetTempCompAvailable()
{
	if ( NULL == m_Calibration )
		return false;	
	return ( 0 != m_Calibration->TempCompAvailable );
}

//int FTSystem::SetTempCompEnabled( bool EnableTempComp )
//{		
//	m_hiHardware->SetNumChannels( NUM_STRAIN_GAUGES + ( (EnableTempComp)?1:0 ) );
//	return SetTempComp( m_Calibration, (int) EnableTempComp );
//}

bool FTSystem::GetTempCompEnabled()
{
	if ( NULL == m_Calibration )
		return false;
	return ( 0 != m_Calibration->cfg.TempCompEnabled );
}

int FTSystem::ToolTransform( const vector<double>& transformVector, const string& distanceUnits, const string& angleUnits )
{
	const int n = 6;
	assert( transformVector.size() == n );
	float tempTransforms[n];
	for ( size_t i = 0; i < n; i++ )
	{
		tempTransforms[i] = (float)transformVector[i];
	}
	return SetToolTransform( m_Calibration, tempTransforms,
											(Units)distanceUnits.c_str(), (Units)angleUnits.c_str() );
}

int FTSystem::GetTransformVector( vector<double>& transformVector )
{
	if ( NULL == m_Calibration )
		return 1; /*calibration not initialized*/

	transformVector.resize(NUM_FT_AXES, 0.0);
	for ( size_t i = 0; i < NUM_FT_AXES; i++ )
	{
		transformVector[i] = m_Calibration->cfg.UserTransform.TT[i];
	}
	return 0;
}

string  FTSystem::GetTransformDistanceUnits()
{
	if ( NULL == m_Calibration ) 
		return string();
	return string( m_Calibration->cfg.UserTransform.DistUnits );
}

string  FTSystem::GetTransformAngleUnits()
{
	if ( NULL == m_Calibration )
		return string();
	return string( m_Calibration->cfg.UserTransform.AngleUnits );
}

string  FTSystem::GetBodyStyle()
{
	if ( NULL == m_Calibration )
		return string();
	return string( m_Calibration->BodyStyle );
}

string  FTSystem::GetCalibrationType()
{
	if ( NULL == m_Calibration )
		return string();
	return string( m_Calibration->PartNumber );
}

int FTSystem::GetWorkingMatrix( double w[] ) // w[NUM_FT_AXES][NUM_STRAIN_GAUGES]
{
	if ( NULL == m_Calibration )
		return 1;

	size_t i, j;
	for ( i = 0; i < NUM_FT_AXES; i++ )
	{
		for( j = 0; j < NUM_STRAIN_GAUGES; j++ )
		{
			w[i * NUM_STRAIN_GAUGES + j] = m_Calibration->rt.working_matrix[i][j];
		}
	}
	return 0;
}

int FTSystem::ReadSingleFTRecord( vector<double>& readings, const bool ignoreSaturation )
{
	assert( readings.size() == NUM_FT_AXES );

	if ( m_Calibration == NULL )
		return 1;

	vector<double> voltages(NUM_STRAIN_GAUGES + 1);  // allow an extra reading for the thermistor

	const int rc = ReadSingleGaugePoint( voltages );

	if ( rc != 0 )
	{
		if ( !(ignoreSaturation && rc==2) )
		{
			// check for error when reading from hardware
			if ( rc == 2 )
				return 2; // saturation
			return 1; // some other error
		}
	}

	float voltages_flt[NUM_STRAIN_GAUGES + 1];
	for ( size_t i = 0; i <= NUM_STRAIN_GAUGES; i++ )
	{
		voltages_flt[i] = (float)voltages[i];
	}

	float tempResult[NUM_FT_AXES];
	ConvertToFT( m_Calibration, voltages_flt, tempResult );

	for ( size_t i = 0; i < NUM_FT_AXES; i++ )
	{
		readings[i] = tempResult[i];
	}

	return rc;
}

int FTSystem::BiasCurrentLoad()
{
	if ( NULL == m_Calibration )
		return 1; /*calibration not initialized*/

	vector<double> curVoltages(NUM_STRAIN_GAUGES + 1);  // the current strain gauge load

	const int retVal = ReadSingleGaugePoint( curVoltages );
	if ( retVal )
		return retVal; /*error reading from hardware*/

	return BiasKnownLoad( curVoltages );
}

int FTSystem::BiasKnownLoad( const vector<double>& biasVoltages )
{
	if ( NULL == m_Calibration )
		return 1;

	float voltages_flt[NUM_STRAIN_GAUGES];
	for ( size_t i = 0; i < NUM_STRAIN_GAUGES; i++ )
	{
		voltages_flt[i] = (float)biasVoltages[i];
	}
	Bias( m_Calibration, voltages_flt );
	return 0;
}

int FTSystem::GetNumChannels()
{
	return m_hiHardware->GetNumChannels();
}

int FTSystem::StopAcquisition()
{
	if ( m_hiHardware->StopCollection() )
		return -1;
	return 0;
}

int FTSystem::StartBufferedAcquisition( const string& deviceName, double sampleFrequency, int averaging,
			int firstChannel, bool useTempComp, int bufferSize )
{
	int numChannels = NUM_STRAIN_GAUGES + ( useTempComp?1:0 );
	const int status = m_hiHardware->ConfigBufferTask( sampleFrequency, averaging, deviceName, firstChannel, numChannels,
		m_iMinVoltage, m_iMaxVoltage, bufferSize );		
	if ( status ) 
	{
		m_sErrorInfo = m_hiHardware->GetErrorCodeDescription( status );
		return -1;
	}
	/*set calibration's use of temp comp*/
	if ( NULL != m_Calibration )
		m_Calibration->cfg.TempCompEnabled = useTempComp;
	return 0;
}



int FTSystem::ReadBufferedGaugeRecords( int numRecords, vector<double>& readings )
{
	long status; /* The status of hardware reads. */
	//const int numGauges = NUM_STRAIN_GAUGES + ( GetTempCompEnabled()?1:0 ); /* The number of gauges in the scan list. */	
	status = m_hiHardware->ReadBufferedSamples( numRecords, readings );
	if ( status )
	{
		m_sErrorInfo = m_hiHardware->GetErrorCodeDescription( status );
		return (int)status;
	}	
	if ( CheckForGaugeSaturation( readings ) ) status = 2;
	return status;
}

bool FTSystem::CheckForGaugeSaturation( const vector<double>& readings )
{
	for( size_t i = 0; i < readings.size(); i++ )
	{
		if ( ( m_dUpperSaturationVoltage < readings[i] ) || 
			 ( m_dLowerSaturationVoltage > readings[i] ) )
		{
			return true;
		}
	}
	return false;
}

int FTSystem::ReadBufferedFTRecords( int numRecords, vector<double>& readings )
{
	if ( m_Calibration == NULL ) // invalid calibration
		return 1;
	int i, j;

	const int numGauges = NUM_STRAIN_GAUGES + ( GetTempCompEnabled() ? 1 : 0 );
	const unsigned int numGaugeValues = numRecords * numGauges; // the number of individual gauge readings

	vector<double> gaugeValues(numGaugeValues); // the gauge readings which are fed to the c library

	float currentGaugeReadings[NUM_STRAIN_GAUGES+1]; //current gauge reading
	float currentFTValues[NUM_FT_AXES]; // current f/t reading

	const long status = m_hiHardware->ReadBufferedSamples( numRecords, gaugeValues );		
	if ( status )
	{
		m_sErrorInfo = m_hiHardware->GetErrorCodeDescription( status );
		return (int) status;		
	}
	/*
	precondition: gaugeValues has the buffered gauge readings, numGauges has the
		number of active gauges (6 or 7)
	postcondition: currentGaugeReadings contains the last gauge reading,
		currentFTValues contains the last ft value, readings contains all ft values,
		i = numRecords, j = NUM_FT_AXES, or the function will have already returned due
		to gauge saturation.
	*/
	for ( i = 0; i < numRecords; i++ )
	{
		/*
		precondition: i is the number of the f/t record we are currently calculating.
		postcondition: currentGaugeReadings contains the i'th gauge readings,
			j = numGauges, or the function will have returned due to gauge saturation
		*/
		for ( j = 0; j < numGauges; j++ )
		{
			currentGaugeReadings[j] = (float)gaugeValues[ j + ( i * numGauges ) ];
			/*check for saturation*/
			if ( m_dUpperSaturationVoltage < currentGaugeReadings[j] )
				return 2;
			if ( m_dLowerSaturationVoltage > currentGaugeReadings[j] )
				return 2;
		}
		ConvertToFT( m_Calibration, currentGaugeReadings, currentFTValues );
		/*
		precondition: currentFTValues has the i'th ft reading from the buffer
		postcondition: j = NUM_FT_AXES, the i'th f/t record in readings contains the values
			from currentFTValues
		*/
		for ( j = 0; j < NUM_FT_AXES; j++ )
		{
			readings[ j + ( i * NUM_FT_AXES ) ] = currentFTValues[j];
		}
	}
	return 0;
}


int FTSystem::GetBiasVector( vector<double>& biasVector )
{
	assert( biasVector.size() == NUM_STRAIN_GAUGES );

	if ( NULL == m_Calibration )
		return 1;

	for ( size_t i = 0; i < NUM_STRAIN_GAUGES; i++ )
	{
		biasVector[i] = m_Calibration->rt.bias_vector[i];
	}
	return 0;
}

double FTSystem::GetThermistorValue()
{
	if ( NULL == m_Calibration ) return 0;
	return m_Calibration->rt.thermistor;
}

double FTSystem::GetBiasSlope( int index )
{
	if ( NULL == m_Calibration ) return 0;
	return m_Calibration->rt.bias_slopes[index];
}

double FTSystem::GetGainSlope( int index )
{
	if ( NULL == m_Calibration ) return 0;
	return m_Calibration->rt.gain_slopes[index];
}

/*july.22.2005 - ss - added SetConnectionMode*/
void FTSystem::SetConnectionMode( ConnectionType connType )
{
	int ct = DAQmx_Val_Diff;

	switch (connType)
	{
	case DIFFERENTIAL:
		ct = DAQmx_Val_Diff;
		break;
	case REFERENCED_SINGLE_ENDED:
		ct = DAQmx_Val_RSE;
		break;
	case NON_REFERENCED_SINGLE_ENDED:
		ct = DAQmx_Val_NRSE;
		break;
#ifdef GOT_PSEUD
	case PSEUDO_DIFFERENTIAL:
		ct = DAQmx_Val_PseudoDiff;
		break;
#endif
	}

	m_hiHardware->SetConnectionMode( ct );
}

/*july.22.2005 - ss - added GetConnectionmode*/
ConnectionType FTSystem::GetConnectionMode()
{
	ConnectionType connType = DIFFERENTIAL;

	const int ct = m_hiHardware->GetConnectionMode();

	switch (ct)
	{
	case DAQmx_Val_Diff:
		connType = DIFFERENTIAL;
		break;
	case DAQmx_Val_RSE:
		connType =REFERENCED_SINGLE_ENDED;
		break;
	case DAQmx_Val_NRSE:
		connType =NON_REFERENCED_SINGLE_ENDED;
		break;
#ifdef GOT_PSEUD
	case DAQmx_Val_PseudoDiff:
		connType =PSEUDO_DIFFERENTIAL;
		break;
#endif
	}

	return connType;
}

/*aug.5.2005a - ss - added GetHardwareTempComp*/
bool FTSystem::GetHardwareTempComp( )
{
	if ( m_Calibration->HWTempComp ) return true;
	return false;
}

// number of samples (per channel) available in internal DAQ device buffer
int FTSystem::numSamplesInBuffer()
{
	return m_hiHardware->numSamplesInBuffer();
}

} // namespace DAQFT
