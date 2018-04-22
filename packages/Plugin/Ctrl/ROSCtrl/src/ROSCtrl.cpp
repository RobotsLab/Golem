/** @file ROSCtrl.cpp
*
* @author	Marek Kopicki
*
* @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
*
* @license  This file copy is licensed to you under the terms described in
*           the License.txt file included in this distribution.
*
*/

#include <Golem/Ctrl/ROSCtrl/ROSCtrl.h>
#include <Golem/Tools/LoadObjectDesc.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Ctrl/SingleCtrl/Data.h>
#include <Golem/Tools/XMLData.h>
#include <sensor_msgs/JointState.h>
//#include <Golem/Ctrl/ROSCtrl/ros_msgs/MoveJoints.h>  // ROS service def for moving joints
//#include <Golem/Ctrl/ROSCtrl/ros_msgs/StartGravComp.h>
//#include <Golem/Ctrl/ROSCtrl/ros_msgs/StopGravComp.h>

//------------------------------------------------------------------------------

//#define IIWA_NB_AXES 7
//#define FRI_CART_VEC 6

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc) {
	loadObjectDesc<golem::ROSCtrl::Desc, Controller::Desc::Ptr>((Context*)pContext, (XMLContext*)pXMLContext, (Controller::Desc::Ptr*)pControllerDesc);
}

//------------------------------------------------------------------------------

//namespace golem {
//	struct RosOut {
//		int nextControlMode;
//		float newJntPosition[IIWA_NB_AXES];
//		float newJntStiff[IIWA_NB_AXES];
//		float newJntDamp[IIWA_NB_AXES];
//		float newJntAddTorque[IIWA_NB_AXES];
//
//		//float newCartPosition[FRI_CART_FRM_DIM];
//		float newCartStiff[FRI_CART_VEC];
//		float newCartDamp[FRI_CART_VEC];
//		float newAddTcpFT[FRI_CART_VEC];
//		float newJntNullspace[IIWA_NB_AXES];
//	};
//
//	struct RosInp {
//		float jntTrq[IIWA_NB_AXES];
//		int currentControlMode;
//	};
//
//	class RosState {
//	public:
//		RosState(Context& context) {
//		}
//
//		RosOut out;
//		RosInp inp;
//	};
//};

//------------------------------------------------------------------------------

ROSCtrl::ROSCtrl(Context& context) : SingleCtrl(context) {
}

ROSCtrl::~ROSCtrl() {
	SingleCtrl::release();
}

void ROSCtrl::create(const Desc& desc) {
	SingleCtrl::create(desc); // throws

	coordMapSeq = desc.coordMapSeq;
	this->desc = desc;
	userCreate();
}

//------------------------------------------------------------------------------

void ROSCtrl::lookupState(SecTmReal t, State &state) const {
	SingleCtrl::lookupState(t, state);

	// coord map
	CoordMap::apply(getStateInfo().getJoints(), coordMapSeq, getMin().cpos, getMax().cpos, state.cpos, state.cpos);
	CoordMap::apply(getStateInfo().getJoints(), coordMapSeq, getMin().cvel, getMax().cvel, state.cvel, state.cvel);
	CoordMap::apply(getStateInfo().getJoints(), coordMapSeq, getMin().cacc, getMax().cacc, state.cacc, state.cacc);
}

void ROSCtrl::setToDefault(State& state) const {
	SingleCtrl::setToDefault(state);
}

void ROSCtrl::clampConfig(GenConfigspaceCoord& config) const {
	SingleCtrl::clampConfig(config);

	// coord map
	CoordMap::apply(getStateInfo().getJoints(), coordMapSeq, getMin().cpos, getMax().cpos, config.cpos, config.cpos);
	CoordMap::apply(getStateInfo().getJoints(), coordMapSeq, getMin().cvel, getMax().cvel, config.cvel, config.cvel);
	CoordMap::apply(getStateInfo().getJoints(), coordMapSeq, getMin().cacc, getMax().cacc, config.cacc, config.cacc);
}

//------------------------------------------------------------------------------

void ROSCtrl::userCreate() {
	state.reset(new State(createState()));
	setToDefault(*state);
	context.write("ROSCtrl::userCreate(): Initialisation...\n");

	std::vector<std::pair<std::string, std::string> > remappings;
	std::pair<std::string, std::string> master_uri("__master", desc.master_url);
	remappings.push_back(master_uri);
	ros::init(remappings, "golem_kuka_miiwa");

	context.write("ROSCtrl::userCreate(): Created ros node...\n");
	ros::NodeHandle ros_node_handle;
	joint_state_sub_ = ros_node_handle.subscribe(desc.rostopic_joint_state, 1000, &ROSCtrl::received_state, this);
	move_arm_client_ = ros_node_handle.serviceClient<ros_msgs::MoveJoints>(desc.rostopic_move_joints);
	context.write("ROSCtrl::userCreate(): Initialisation finished\n");
}

void ROSCtrl::sysSync() {
	ros::spinOnce();
}

void ROSCtrl::sysRecv(State& state) {
	// just copy
	state = *this->state;
}

//void ROSCtrl::fromState(const State& state, RosOut& out) const {
//	//out.nextControlMode = getNextControlMode(state);
//}
//
//
//void ROSCtrl::toState(const RosInp& inp, State& state) const {
//	//for (Configspace::Index i = getStateInfo().getJoints().begin(); i < getStateInfo().getJoints().end(); ++i)
//	//	getTorque(state)[i] = inp.jntTrq[i - getStateInfo().getJoints().begin()];
//
//	//getCurrentControlMode(state) = inp.currentControlMode;
//}

void ROSCtrl::sysSend(const State& prev, const State& next, bool bSendPrev, bool bSendNext) {
	ros_msgs::MoveJoints srv;
	for (Configspace::Index i = getStateInfo().getJoints().begin(); i != getStateInfo().getJoints().end(); ++i)
		srv.request.desired_joint_positions.values.push_back((float)next.cpos[i]);
	srv.request.parameter.blocking = desc.rosDesc.blocking;
	srv.request.parameter.blending = desc.rosDesc.blending;
	srv.request.parameter.velocity = desc.rosDesc.velocity;

	if (!move_arm_client_.call(srv))
		throw std::runtime_error("ROSCtrl -- Failed to send positions over ROS service.\n");
}


void ROSCtrl::received_state(const sensor_msgs::JointState::ConstPtr& msg) { // throws
	// received the current state of the arm
	if (msg->positions.size() != getStateInfo().getJoints().size())
		throw Message("ROSCtrl::received_state: Invalid robot description.");

	// Update the state of the robot
	for (unsigned int i = 0; i < getStateInfo().getJoints().size(); ++i) {
		this->state->cpos[getStateInfo().getJoints().begin() + i] = msg->positions[i];
		//measuredTorque[i] = msg->measured_torques[i];
	}
}

//------------------------------------------------------------------------------

void golem::XMLData(Joint::Desc::Ptr &val, XMLContext* context, bool create) {
	val.reset(new Joint::Desc());
	XMLData((Joint::Desc&)*val, context, create);
}

void golem::XMLData(Chain::Desc::Ptr &val, XMLContext* context, bool create) {
	val.reset(new Chain::Desc());
	XMLData((Chain::Desc&)*val, context, create);

	// TODO number of joints is determined by URDF file
	// create default joints
	XMLData(val->joints, golem::Configspace::DIM, context, "joint", create);
}

void golem::XMLData(ROSCtrl::ROSDesc &val, XMLContext* context, bool create) {
	XMLData("master_uri", val.master_url, context, create);
	XMLData("name", val.rostopic_joint_state, context->getContextFirst("rostopic_joint_state"), create);
	XMLData("name", val.rostopic_move_joints, context->getContextFirst("rostopic_move_joints"), create);
	XMLData("blocking", val.blocking, context->getContextFirst("rostopic_move_joints"), create);
	XMLData("blending", val.blending, context->getContextFirst("rostopic_move_joints"), create);
	XMLData("velocity", val.velocity, context->getContextFirst("rostopic_move_joints"), create);
}


void golem::XMLData(ROSCtrl::Desc &val, XMLContext* context, bool create) {
	XMLData((SingleCtrl::Desc&)val, context, create);

	// TODO number of chains is determined by URDF file
	// create default chains
	XMLData(val.chains, golem::Chainspace::DIM, context, "chain", create);

	// coordinate map
	try {
		XMLData(val.coordMapSeq, val.coordMapSeq.max_size(), context, "coord_map", false);
	}
	catch (const MsgXMLParserNameNotFound&) {}

	// TODO ROS-related params come here
	try {
		XMLData(val.rosDesc, context->getContextFirst("ros"), create);
	}
	catch (const MsgXMLParserNameNotFound&) {}
}

//------------------------------------------------------------------------------
