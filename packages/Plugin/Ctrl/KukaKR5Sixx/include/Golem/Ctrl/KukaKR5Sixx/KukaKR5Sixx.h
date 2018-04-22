/** @file KukaKR5Sixx.h
 * 
 * Kuka KR 5 Sixx R650/R850 controller
 * 
 * @author	Chris Burbridge
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki and Chris Burbridge, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_KUKAKR5SIXX_KUKAKR5SIXX_H_
#define _GOLEM_CTRL_KUKAKR5SIXX_KUKAKR5SIXX_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Kuka/KukaKR5Sixx.h>
#include <memory>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR KukaKR5SixxRSI : public KukaKR5Sixx {
public:
	/** KukaKR5Sixx drive mode */
	enum DriveMode {
		/** Joint velocity/correction */
		JOINT_CORRECTION,
		/** Cartesian velocity */
		CARTESIAN_CORRECTION,
	};

	/** KukaKR5SixxRSI description */
	class GOLEM_LIBRARY_DECLDIR Desc : public KukaKR5Sixx::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(KukaKR5SixxRSI, Controller::Ptr, Context&)

		/** Local port */
		unsigned localPort;
		/** Kuka drive mode */
		DriveMode driveMode;
		/** Feedback gain */
		std::vector<Real> feedbackGain;
			
		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			KukaKR5Sixx::Desc::setToDefault();

			name = "Kuka KR 5 Sixx";
			
			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.01);
			simDeltaSend = SecTmReal(0.002);

			chains.clear();
			chains.push_back(Chain::Desc::Ptr(new KukaKR5SixxChain::Desc));

			localPort = 6008;
			driveMode = JOINT_CORRECTION;
			feedbackGain.assign(KukaKR5SixxChain::NUM_JOINTS, Real(0.02));
		}

		virtual bool isValid() const {
			if (!KukaKR5Sixx::Desc::isValid())
				return false;

			if (chains.size() != 1)
				return false;
			if (dynamic_cast<const KukaKR5SixxChain::Desc*>(chains.begin()->get()) == NULL)
				return false;

			if (feedbackGain.size() < 6)
				return false;
			for (std::vector<Real>::const_iterator i = feedbackGain.begin(); i != feedbackGain.end(); ++i)
				if (*i < REAL_ZERO || *i > REAL_ONE)
					return false;

			return true;
		}
	};

	/** Release resources */
	virtual ~KukaKR5SixxRSI();

protected:
	/** Local port */
	unsigned localPort;
	/** Kuka drive mode */
	DriveMode driveMode;
	/** Feedback gain */
	std::vector<Real> feedbackGain;

	State::Ptr state;
	Configspace::Index joint;

	virtual void sysSync();
	virtual void sysRecv(State& state);
	virtual void sysSend(const State& prev, const State& next, bool bSendPrev, bool bSendNext);

	virtual void userCreate();
	void create(const Desc& desc);

	KukaKR5SixxRSI(golem::Context& context);

private:
	template < class RT > static RT getTagVal( char* xml, const char *tagname, const char *closetag, RT (*atov)(const char*), char **pos) {
		int tagLength = (int)strlen(tagname);   // could be hard coded...
		char *start = strstr(xml,tagname); // TODO: could start the search nearer to position for more efficiency
		if (start == NULL){
			*pos = NULL;
			return 0;
		}
		char *end = strstr(start+tagLength,closetag);
		if (end == NULL){
			*pos = NULL;
			return 0;
		}
		*end='\0';
		RT ret = atov(start+tagLength);
		*end=closetag[0];
		*pos = end+strlen(closetag);
		return ret;
	}

	struct IncomingData {
		struct {
			double x,y,z,a,b,c;
		} actualCartPosition, setpointCartPosition ;
		struct {
			double a1,a2,a3,a4,a5,a6;
		} actualAxisPosition, setpointAxisPosition, motorCurrents;
		int delayedPackets; // How many packets have we missed/been late sending - more than 9 is a stop condition....
		I64 ipoc;

	};

	struct OutgoingData {
		char message[300];
		struct {
			double x,y,z,a,b,c;
		} cartCorrection;
		struct {
			double a1,a2,a3,a4,a5,a6;
		} axisCorrection;
		DriveMode driveMode;

		// The ipoc cylce that we aim for this command to be completed or stopped by
		I64 target_ipoc;
	};

	void setMessage(const char* msg);
	bool parseIncomingState( char *xml );
	bool fillOutgoingBuffer();

	int serverfd, clientfd;

	size_t incomingDataPos;

	std::vector<char> incomingDataBuffer;
	std::vector<char> outgoingDataBuffer;

	static const char* outgoingXMLFormat;

	bool isConnected;

	int headState;	// tracking which element of currentState is latest
	IncomingData currentState[5]; // keep history of up to 5 states
	OutgoingData currentCommand;

	int received_count;
};


//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_KUKAKR5SIXX_KUKAKR5SIXX_H_*/
