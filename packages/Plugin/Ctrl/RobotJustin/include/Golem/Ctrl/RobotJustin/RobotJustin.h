/** @file RobotJustin.h
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
#ifndef _GOLEM_CTRL_ROBOTJUSTIN_ROBOTJUSTIN_H_
#define _GOLEM_CTRL_ROBOTJUSTIN_ROBOTJUSTIN_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/MultiCtrl/MultiCtrl.h>
#include <Golem/Ctrl/Kuka/KukaLWR.h>
#include <Golem/Ctrl/DLR/DLRHandII.h>
#include <Golem/Ctrl/RobotJustin/Load.h>
#include <Golem/Sys/XMLParser.h>
#include <Golem/Ctrl/RobotJustin/bham_planner_interface.h>
#include <Golem/Ctrl/RobotJustin/TelemetryReader.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */

//------------------------------------------------------------------------------

/** Multi controller interface. */
class RobotJustin : public virtual MultiCtrl {
public:
	/** Custom message stream */
	class BhamMessageStream : public bham_planner_interface::message_stream {
	public:
		BhamMessageStream(golem::MessageStream& messageStream) : messageStream(messageStream) {
		}
		virtual void debug(const char* format, va_list argptr) {
			messageStream.write(golem::Message::getCurrentTime(), golem::Message::getCurrentThread(), golem::Message::LEVEL_DEBUG, golem::Message::CODE_UNDEF, format, argptr);
		}
		virtual void info(const char* format, va_list argptr) {
			messageStream.write(golem::Message::getCurrentTime(), golem::Message::getCurrentThread(), golem::Message::LEVEL_INFO, golem::Message::CODE_UNDEF, format, argptr);
		}
		virtual void error(const char* format, va_list argptr) {
			messageStream.write(golem::Message::getCurrentTime(), golem::Message::getCurrentThread(), golem::Message::LEVEL_ERROR, golem::Message::CODE_UNDEF, format, argptr);
		}
	
	private:
		golem::MessageStream& messageStream;
	};

	/** Controller description */
	class Desc : protected MultiCtrl::Desc {
	public:
		friend class RobotJustin;
		friend void ::loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);

		/** Justin host */
		std::string host;
		/** Justin port */
		unsigned short port;
		/** First request name */
		std::string requestName;
		/** Telemetry Reader */
		TelemetryReader::Desc telemetryReaderDesc;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			MultiCtrl::Desc::setToDefault();
			
			name = "DLR Justin";

			host = "localhost";
			port = 54363;
			requestName = "bham_grasp";
			telemetryReaderDesc.setToDefault();
		}
		
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!MultiCtrl::Desc::isValid())
				return false;
			
			if (host.empty() || port <= 0 || !telemetryReaderDesc.isValid())
				return false;

			return true;
		}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(RobotJustin, Controller::Ptr, Context&)
	};

private:
	/** Controllers */
	KukaLWR *arm;
	DLRHandII *hand;
	/** Custom message stream */
	BhamMessageStream bhamMessageStream;
	/** Telemetry Reader */
	TelemetryReader::Ptr telemetryReader;

	/** bham_planner_interface connection */
	golem::shared_ptr<bham_planner_interface::connection> connection;
	/** bham_planner_interface request and response */
	golem::shared_ptr<bham_planner_interface::request_t> request, response;

	/** Label guards */
	static const char *guardNames [];
	/** buffer of guards to be sent */
	std::vector<golem::Real> guardsBuff;
	/** Verbose conditions */
	std::vector<std::string> guardConditions;

	/** List of object in the scene */
	std::list<bham_planner_interface::scene_object_t> objects;

	/** Return the contact points between fingers and the object */
	virtual bool getContacts();

	/** Creates Controller from the description. */
	void create(const Desc& desc);

	RobotJustin(Context& context);

public:
	/** Virtual destructor releasing resources */
	virtual ~RobotJustin();

	/** bham_planner_interface connection */
	bham_planner_interface::connection* getConnection() {
		return connection.get();
	}
	/** bham_planner_interface connection */
	const bham_planner_interface::connection* getConnection() const {
		return connection.get();
	}

	/** bham_planner_interface request */
	virtual bham_planner_interface::request_t* createRequest() const;

	/** Sends a sequence of motor commands preserving maximum time wait. */
	virtual const State* send(const State* begin, const State* end, bool clear = false, bool limits = true, MSecTmU32 timeWait = MSEC_TM_U32_INF);

	/** Controller has just sent the last trajectory segment */
	virtual bool waitForEnd(MSecTmU32 timeWait = MSEC_TM_U32_INF);

	/** Sets tcp guards */
	virtual void setGuards(const golem::Twist &wrench);
	/** Sets hand guards */
	virtual void setGuards(const std::vector<golem::Real> &force);
	/** Sets single guard in the hand */
	virtual void setGuard(const size_t idx, const golem::Real value);
	/** Sets single guard in the string format for bham_interface */
	virtual void setGuard(const std::string &guard);

	/** Retrieves a vector of indeces and the timestamp at which the contacts occur, if any 
		Returns: 
			-1	occasional contact with the environment
			0	no contact
			1	contact with the object to be grasped
	*/
	virtual int getTriggeredGuards(std::vector<int> &triggeredGuards, Controller::State &state);
	/** Checks if guards have been triggered */
	virtual bool hasTriggeredGuards();
	/** Enables/disables guards */
	bool guardsEnable;

	/** Sets stiffness of the hand */
	virtual void setHandStiffness(const std::vector<golem::Real> &stiffness);

	/** Gets frame of an object on the scene */
	virtual void getFrameObject(golem::Mat34 &frame, const std::string &id = "") const;
	/** Computes the FCA. Returns zero if no fca is found */
	virtual Real analyseGrasp();
	/** Return the contact points between fingers and the object */
	virtual bool getContacts(golem::Vec3 &contact, golem::Vec3 &normal, const bool onlyFingerTips = false);

	/** Finds actual global pose of Justin with respect to its base */
	virtual Mat34 recvGlobalPose();
};

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(RobotJustin::Desc &val, Context* context, XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CTRL_ROBOTJUSTIN_ROBOTJUSTIN_H_*/
