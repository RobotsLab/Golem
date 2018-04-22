/** @file PISASoftHandSerial.cpp
 *
 * @author	Marek Kopicki
 * @author	Maxime Adjigble
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki and Maxime Adjigble, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/PISASoftHand/PISASoftHand.h>
#include <Golem/Ctrl/PISASoftHand/Data.h>
#include <Golem/Sys/LoadObjectDesc.h>
#include <qbmove_communications.h>
#include <cstdio>

//------------------------------------------------------------------------------

//#define DEFAULT_RESOLUTION 1
//#define DEFAULT_INF_LIMIT -30000
//#define DEFAULT_SUP_LIMIT 30000
//#define BROADCAST_ID 0
//#define DEFAULT_PID_P 0.1
//#define DEFAULT_PID_I 0
//#define DEFAULT_PID_D 0.8
//#define DEFAULT_INCREMENT 1 //in degree
//#define DEFAULT_STIFFNESS 30 //in degree
//#define DEFAULT_MAX_EXCURSION 330 //in degree
//
//#define DEG_TICK_MULTIPLIER (65536.0 / (360.0 * (pow(2, DEFAULT_RESOLUTION))))
//
//#define SIN_FILE "PISAHandSin.conf"
//#define MOTOR_FILE "PISAHandMotor.conf"
//#define QBMOVE_FILE "PISAHandQbmove.conf"
//#define HAND_CALIBRATION "PISAHandCalibration.csv"
//#define EMG_SAVED_VALUES "PISAHandEmg_values.csv"

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc) {
	loadObjectDesc<golem::PISASoftHandSerial::Desc, Controller::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Controller::Desc::Ptr*)pControllerDesc);
}

//------------------------------------------------------------------------------

PISASoftHandSerial::PISASoftHandSerial(golem::Context& context) : PISASoftHand(context) {
}

PISASoftHandSerial::~PISASoftHandSerial() {
	PISASoftHand::release();

	//Desactivate the device
	if (comm != NULL) {
		setMotorPosition(0);
		golem::Sleep::msleep(1000);
		commActivate(comm.get(), 0, 0);
		closeRS485(comm.get());
	}
}

void PISASoftHandSerial::create(const Desc& desc) {
    serialPort = desc.serialPort;
	motorFile = desc.motorFile;

	PISASoftHand::create(desc); // throws
}

void PISASoftHandSerial::userCreate() {
	state.reset(new State(createState()));
	setToDefault(*state);

	//Get the gear ratio
	FILE *file = fopen(motorFile.c_str(), "r");
	if (!file)
		throw Message(Message::LEVEL_CRIT, "PISASoftHandSerial::userCreate(): unable to open motor file %s", motorFile.c_str());
	float gear_ratio[NUM_MOTORS];
	fscanf(file, "gear_ratio_1 %f\n", &gear_ratio[0]);
	fscanf(file, "gear_ratio_2 %f\n", &gear_ratio[1]);
	fclose(file);

	char ports[10][255];
	const int num_ports = RS485listPorts(ports);
	if (num_ports <= 0)
		throw Message(Message::LEVEL_CRIT, "PISASoftHandSerial::userCreate(): no ports available");

	int indexOfPort = 0;
	for (; indexOfPort < num_ports && serialPort.compare(ports[indexOfPort]) != 0; ++indexOfPort);
	if (indexOfPort >= num_ports)
		throw Message(Message::LEVEL_CRIT, "PISASoftHandSerial::userCreate(): unable to find %s", serialPort.c_str());
	//indexOfPort = 1;
	//context.write("PISASoftHandSerial::userCreate(): port %s found\n", ports[indexOfPort]);

	//Open the communication
#ifndef WIN32
	const std::string strSysPerm = "sudo chmod 777 " + serialPort;
	system(strSysPerm.c_str());
#endif
	comm.reset(new reference_cnt_vt<comm_settings>(new comm_settings));
	openRS485(comm.get(), serialPort.c_str());
	if (comm->file_handle == INVALID_HANDLE_VALUE)
		throw Message(Message::LEVEL_CRIT, "PISASoftHandSerial::userCreate(): unable to open port %s", serialPort.c_str());

	U8 resolution[NUM_SENSORS];         // sensors resolution set on the board
	// calculate correction factor
	// retrieve current resolution
	// while(commGetParam(comm, global_args.device_id, PARAM_POS_RESOLUTION, resolution, NUM_OF_SENSORS) != 0) {}
	resolution[0] = 1;
	resolution[1] = 1;
	resolution[2] = 1;

	float correction_factor[NUM_SENSORS];    // correction factor calculated

	// calculate correction factor for every sensors
	for (int i = 0; i < NUM_SENSORS; i++)
		correction_factor[i] = (float)65536.0/(360 << resolution[i]);

	// add gear ratio to correction factor
	for (int i = 0; i < NUM_MOTORS; i++)
		correction_factor[i] = correction_factor[i] * gear_ratio[i];

	//Get the useful informations
	char infoMessage[10000]; // used to store PING reply
	RS485GetInfo(comm.get(), infoMessage);

	// reset
	resetHand();

	//Wait for the initialisation to finish
	golem::Sleep::msleep(1000);

	//Set the command buffer to zero
	setMotorPosition(0);

	//Wait for the initialisation to finish
	golem::Sleep::msleep(1000);
}

//------------------------------------------------------------------------------

void PISASoftHandSerial::setMotorPosition(short int position) {
	if (comm == NULL)
		return;
	short int inputs[NUM_MOTORS];
	inputs[0] = position;
	inputs[1] = 0;
	commSetInputs(comm.get(), 0, inputs);
}

void PISASoftHandSerial::getMotorPosition(short int &position) {
	if (comm == NULL)
		return;
	short int measurements[NUM_SENSORS];
	commGetMeasurements(comm.get(), 0, measurements);
	position = measurements[0];
}

void PISASoftHandSerial::getMotorCurrent(short int &current) {
	if (comm == NULL)
		return;
	short int currents[NUM_MOTORS];
	commGetCurrents(comm.get(), 0, currents);
	current = currents[0];
}

void PISASoftHandSerial::resetHand() {
	if (comm == NULL)
		return;
	//Desactivate the device
	//commActivate(comm.get(), 0, 0);

	// Reset all the offsets
	//float measurement_offset[NUM_SENSORS];
	//std::fill_n(measurement_offset, NUM_SENSORS, numeric_const<float>::ZERO);
	//commSetParam(comm.get(), 0, PARAM_MEASUREMENT_OFFSET, measurement_offset, NUM_SENSORS);
	
	//Activate the device
	commActivate(comm.get(), 0, 1);
}

//------------------------------------------------------------------------------

void PISASoftHandSerial::sysSync() {
	short motorPosition;
    getMotorPosition(motorPosition);
	PISASoftHand::getMotorPosition(*state) = motorPosition;

	short motorCurrent;
	getMotorCurrent(motorCurrent);
	PISASoftHand::getMotorCurrent(*state) = motorCurrent;

	// TODO find out the actual received state delay - here we assume it is equal zero
	state->t = context.getTimer().elapsed();

	// Use default synergy to update the state of the robot
	getSynergyMap(motorPositionToSynergy(motorPosition), *state);

	// TODO use external sensors to find the actual state

    // Wait for data (?)
    golem::Sleep::msleep(1);
}

void PISASoftHandSerial::sysRecv(State& state) {
	state = *this->state;
}

void PISASoftHandSerial::sysSend(const State& prev, const State& next, bool bSendPrev, bool bSendNext) {
	const Real s = getSynergy(next);
	const short motorPosition = synergyToMotorPosition(s);
	setMotorPosition(motorPosition);
}

//------------------------------------------------------------------------------
