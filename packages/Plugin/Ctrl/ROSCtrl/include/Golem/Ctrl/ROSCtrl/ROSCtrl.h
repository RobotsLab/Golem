/** @file ROSCtrl.h
*
* ROS controller interface
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
#ifndef _GOLEM_CTRL_ROSCTRL_ROSCTRL_H_
#define _GOLEM_CTRL_ROSCTRL_ROSCTRL_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/SingleCtrl/SingleCtrl.h>
#include <Golem/Tools/Library.h>
#include <Golem/Tools/XMLParser.h>

// Includes for the ROS communication
#include <ros/ros.h>
#include <Golem/Ctrl/ROSCtrl/ros_msgs/JointState.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

//struct RosOut;
//struct RosInp;

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR ROSCtrl : public SingleCtrl {
public:
	/** ROS inteface description */
	class GOLEM_LIBRARY_DECLDIR ROSDesc {
	public:
		/** ROS primary uri */
		std::string master_url;
		/** ROS topic: read the state of the robot */
		std::string rostopic_joint_state;
		/** ROS topic: position control */
		std::string rostopic_move_joints;

		/** Position control parameters */
		bool blocking;
		golem::U32 blending;
		golem::U32 velocity;


		ROSDesc() {
			setToDefault();
		}

		virtual void setToDefault() {
			master_url = "http://ubuntu:11311";//"http://sirius:11311";
			rostopic_joint_state = "/mtc_iiwa/joint_state";
			rostopic_move_joints = "/mtc_iiwa/move_joints";

			blocking = false;
			blending = 0.2;
			velocity = 0.1;
		}

		virtual bool isValid() const {
			return true;
		}
	};

	/** ROSCtrl description */
	class GOLEM_LIBRARY_DECLDIR Desc : public SingleCtrl::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(ROSCtrl, Controller::Ptr, Context&)

		/** Coordinate linear map */
		CoordMap::Seq coordMapSeq;
		/** Ros descrition */
		ROSDesc rosDesc;

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			SingleCtrl::Desc::setToDefault();

			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.01);
			simDeltaSend = SecTmReal(0.002);

			rosDesc.setToDefault();

			name = "ROS controller interface";

			chains.clear();
			// number of kinematic chains and joints is controlled by URDF files
			coordMapSeq.clear();
		}

		virtual bool isValid() const {
			if (!SingleCtrl::Desc::isValid())
				return false;

			if (!rosDesc.isValid())
				return false;

			for (CoordMap::Seq::const_iterator i = coordMapSeq.begin(); i != coordMapSeq.end(); ++i)
			if (!i->isValid())
				return false;

			return true;
		}
	};

	/** Interpolates the controller state at time t. */
	virtual void lookupState(SecTmReal t, State &state) const;
	/** Sets to default a given state. */
	virtual void setToDefault(State& state) const;
	/** Clamp configuration. */
	virtual void clampConfig(GenConfigspaceCoord& config) const;

	/** Release resources */
	virtual ~ROSCtrl();

protected:
	/** Golem controller state */
	State::Ptr state;
	/** Description file */
	Desc desc;

	/** Coordinate linear map */
	CoordMap::Seq coordMapSeq;

	virtual void sysSync();
	virtual void sysRecv(State& state);
	virtual void sysSend(const State& prev, const State& next, bool bSendPrev, bool bSendNext);

	virtual void userCreate();
	void create(const Desc& desc);

	//void fromState(const State& state, RosOut& iiwaOut) const;
	//void toState(const RosInp& iiwaInp, State& state) const;

	ROSCtrl(Context& context);

private:
	ros::Subscriber joint_state_sub_;
	ros::ServiceClient move_arm_client_;
	void received_state(const ros_msgs::JointState::ConstPtr& msg);
};

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(Joint::Desc::Ptr &val, XMLContext* context, bool create = false);
void XMLData(Chain::Desc::Ptr &val, XMLContext* context, bool create = false);
void XMLData(ROSCtrl::ROSDesc &val, XMLContext* context, bool create = false);
void XMLData(ROSCtrl::Desc &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_ROSCTRL_ROSCTRL_H_*/
