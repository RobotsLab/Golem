/** @file RobotJustin.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/RobotJustin/RobotJustin.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;
using namespace bham_planner_interface;

//------------------------------------------------------------------------------

RobotJustin::RobotJustin(golem::Context& context) : MultiCtrl(context), arm(NULL), hand(NULL), bhamMessageStream(*context.getMessageStream()) {
}

RobotJustin::~RobotJustin() {
	MultiCtrl::release();
}

void RobotJustin::create(const Desc& desc) {
	MultiCtrl::create(desc); // throws

	if (getControllers().size() != 2)
		throw Message(Message::LEVEL_CRIT, "RobotJustin::create(): Two connected devices required");	
	arm = dynamic_cast<KukaLWR*>(getControllers()[0]);
	if (arm == NULL)
		throw Message(Message::LEVEL_CRIT, "RobotJustin::create(): Kuka LWR arm required");
	hand = dynamic_cast<DLRHandII*>(getControllers()[1]);
	if (hand == NULL)
		throw Message(Message::LEVEL_CRIT, "RobotJustin::create(): DLR Hand II required");

	context.write("Justin::create(): Waiting for Justin 'bham_grasp' request...\n");
	connection.reset(connection::create(desc.host, desc.port, &bhamMessageStream));
	request.reset(connection->wait_for_request(desc.requestName));
	// get the list of objects in the scene before reset the request
	objects = request->get_list_of_objects("objects");
	request.reset(); // clear
	context.write("Done!\n");

	telemetryReader = desc.telemetryReaderDesc.create(*this);
	telemetryReader->installArmCallback(*arm);
	arm->initControlCycle();
	telemetryReader->installHandCallback(*hand);
	hand->initControlCycle();

	// sets the guards to -1 (default for not set)
	guardsBuff.assign(6 + 2*DLRHandII::NUM_CHAINS*DLRHandIIChain::NUM_JOINTS_CTRL, Real(2.0));
	guardConditions.reserve(guardsBuff.size());
	guardsEnable = false;
}

//------------------------------------------------------------------------------

const char* RobotJustin::guardNames [] = {
	"right_tcp 0 |> ",
	"right_tcp 1 |> ",
	"right_tcp 2 |> ",
	"right_tcp 3 |> ",
	"right_tcp 4 |> ",
	"right_tcp 5 |> ",
	"right_thumb 0 |> ",
	"right_thumb 1 |> ",
	"right_thumb 2 |> ",
	"right_tip 0 |> ",
	"right_tip 1 |> ",
	"right_tip 2 |> ",
	"right_middle 0 |> ",
	"right_middle 1 |> ",
	"right_middle 2 |> ",
	"right_ring 0 |> ",
	"right_ring 1 |> ",
	"right_ring 2 |> ",
};

bham_planner_interface::request_t* RobotJustin::createRequest() const {
	return bham_planner_interface::request_t::create();
}

const Controller::State* RobotJustin::send(const State* begin, const State* end, bool clear, bool limits, MSecTmU32 timeWait) {
	//context.debug("RobotJustin::sendTrajectory(): Justin 'execute_path' request (%d waypoints)...\n", size_t(end - begin));

	std::list<std::string> actuator_names;
	actuator_names.push_back("right_arm");
	actuator_names.push_back("right_hand");
	
	path_t justinTraj;
	for (const State* waypoint = begin; waypoint != end; ++waypoint) {
		config_t justinConfig(KukaLWRChain::NUM_JOINTS + DLRHandII::NUM_JOINTS_CTRL);
		size_t config_idx = 0;

		for (Chainspace::Index i = arm->getStateInfo().getChains().begin(); i != arm->getStateInfo().getChains().end(); ++i) {
			for (Configspace::Index j = arm->getStateInfo().getJoints(i).begin(); j != arm->getStateInfo().getJoints(i).end(); ++j)
				justinConfig[config_idx++] = (float)waypoint->cpos[j];
		}
		for (Chainspace::Index i = hand->getStateInfo().getChains().begin(); i != hand->getStateInfo().getChains().end(); ++i) {
			// the passive (last) joint of the hand is not copied
			for (Configspace::Index j = hand->getStateInfo().getJoints(i).begin(); j != hand->getStateInfo().getJoints(i).end()-1; ++j)
				justinConfig[config_idx++] = (float)waypoint->cpos[j];
		}

		justinTraj.push_back(justinConfig);
	}

	request.reset(request_t::create());
	request->set("request", "execute_path");
	request->set("path", justinTraj);
	request->set("actuators", actuator_names);

	//guardConditions.clear();
	//guardConditions.reserve(guardsBuff.size());
	//// guard condition: right_{[tcp],thumb,index,middle,tip} {0,1,2,[4,5]} > value
	//for (size_t i = 6; i != guardsBuff.size(); ++i)			
	//	guardConditions.push_back(guardNames[i] + boost::lexical_cast<std::string>(guardsBuff[i]));

	if (guardsEnable && !guardConditions.empty())
		request->set("guard_conditions", guardConditions);

	connection->send_request(request.get());
	response.reset(); // clear

	// inform the associated interface
	getCallbackDataSync()->syncSend(&*begin);

	return MultiCtrl::send(begin, end, clear, limits, timeWait);
}

bool RobotJustin::waitForEnd(MSecTmU32 timeWait) {
	if (MultiCtrl::waitForEnd(timeWait) && request == NULL && response != NULL)
		return true;

	const std::string name = request->get("request", "");
	if (name == "")
		throw Message(Message::LEVEL_ERROR, "RobotJustin::waitForEnd(): no request has been forwarded");

	response.reset(connection->wait_for_request(name, timeWait));
	
	return response != NULL;
}

//------------------------------------------------------------------------------

void RobotJustin::setGuards(const Twist &wrench) {
	for (int i = 0; i < 3; ++i) {
		setGuard(i, wrench.v[i]);
		setGuard(i + 3, wrench.w[i]);
	}
}

void RobotJustin::setGuards(const std::vector<golem::Real> &force) {
	for (size_t i = 0; i < force.size(); ++i)
		setGuard(6 + i, force[i]);
}

void RobotJustin::setGuard(const size_t idx, const Real value) {
	guardsBuff[idx] = value;	
}

void RobotJustin::setGuard(const std::string &guard) {
	guardConditions.push_back(guard);	
}

int RobotJustin::getTriggeredGuards(std::vector<int> &triggeredGuards, Controller::State &state) {
	golem::Sleep::msleep(1000); // guarantees to have received the correct state of the robot  
	SecTmReal t = context.getTimer().elapsed();
	lookupState(t, state);
	state.t = t;
//	context.write("golem::RobotJustin::getTriggeredGuards(): response==NULL (%d)\n", response == NULL);
	triggeredGuards.clear();
	if (hasTriggeredGuards()) {
		triggeredGuards = response->get_vector_of_ints("triggered_guards");
		context.write("RobotJustin::getTriggeredGuards(): triggered %d guards:\n", triggeredGuards.size());
		for(std::vector<int>::const_iterator i = triggeredGuards.begin(); i != triggeredGuards.end(); ++i) {
////			guardsBuff[*i + 6] *= 2; 
			context.write(" %d. %s\n", *i, guardConditions[*i].c_str());
		}
		//Returns: 
		//	-1	occasional contact with the environment
		//	0	no contact
		//	1	contact with the object to be grasped
		return getContacts() ? (int)triggeredGuards.size() : triggeredGuards.empty() ? 0 : -1;
	}
	return false;
}

bool RobotJustin::hasTriggeredGuards() {
	context.write("golem::RobotJustin::hasTriggeredGuards: \n");
	//if (request != NULL && response == NULL) {
	//	const std::string name = request->get("request", "");
	//	if (name == "")
	//		throw Message(Message::LEVEL_CRIT, "RobotJustin::waitForEnd(): no request has been forwarded");
	//	context.write("waiting for response...\n");
	//	response.reset(connection->wait_for_request(name, MSEC_TM_U32_INF));
	//}
	if(response != NULL && response->has("triggered_guards")) {
		context.write("response != NULL && response->has_guards (%s)\n", response->get_vector_of_ints("triggered_guards").size() > 0 ? "true" : "false");
		return response->get_vector_of_ints("triggered_guards").size() > 0;
	}
	context.write("false\n");
	return false;
}

//------------------------------------------------------------------------------

void RobotJustin::setHandStiffness(const std::vector<golem::Real> &stiffness) {
	const size_t size = DLRHandII::NUM_CHAINS*DLRHandIIChain::NUM_JOINTS_CTRL;
	if (stiffness.size() != size)
		throw Message(Message::LEVEL_CRIT, "RobotJustin::setHandStiffness(): vector stiffness (size %d) does not match the number of joints in the hand (size %d).\n", stiffness.size(), size);
	
	context.write("RobotJustin::setHandStiffness():");
	for (size_t i = 0; i < size; ++i)
		context.write(" %2.5f", stiffness[i]);
	context.write("\n");

	request.reset(request_t::create());
	request->set("request", "set_hand_stiffness");
	request->set("side", "right");
	request->set("joint_stiffness", stiffness);
	connection->send_request(request.get());
	request.reset(); // clear
}

//------------------------------------------------------------------------------

void RobotJustin::getFrameObject(golem::Mat34 &frame, const std::string &id) const {
	if (objects.empty())
		throw Message(Message::LEVEL_INFO, "RobotJustin::getFrameObject(): no object in the scene.");
	
	bham_planner_interface::scene_object_t object;
	for (std::list<bham_planner_interface::scene_object_t>::const_iterator i = objects.begin(); i != objects.end(); ++i) {
		if (id == "" || i->object_id == id) {
			object = *i;
			break;
		}
	}
	// frame_t (row major 3x4) to mat34
	for (size_t i = 0; i < 3; ++i) {
		const size_t j = i*4;
		frame.R.setRow(i, golem::Vec3(object.frame[j], object.frame[j+1], object.frame[j+2]));
		frame.p.v[i] = object.frame[j+3];
	}
}

Real RobotJustin::analyseGrasp() {
	// create and set the request to get the transformation from Justin's base to shoulder
	request.reset(request_t::create());
	request->set("request", "analyze_grasp");
	request->set("object_name", "mug1_0");
	request->set("friction_coefficient", 0.4f);
	request->set("only_fingertips", 0);
	connection->send_request(request.get());

	// wait for the response (blocking call)
	response.reset(connection->wait_for_request("analyze_grasp", MSEC_TM_U32_INF));
	if (response == NULL)
		throw Message(Message::LEVEL_INFO, "RobotJustin::updateGlobalPose(): no response has been received");
	
	if (response->get_int("have_contacts")) 
		return response->get_int("is_force_closure") ? Real(response->get_double("quality")) : REAL_ZERO;

	return REAL_ZERO;
}

bool RobotJustin::getContacts() {
	Vec3 contact, normal;
	return getContacts(contact, normal);
}

bool RobotJustin::getContacts(golem::Vec3 &contact, golem::Vec3 &normal, const bool onlyFingerTips) {
	request.reset(request_t::create());
	request->set("request", "get_contacts");
	request->set("object_name", "mug1_0");
	request->set("only_fingertips", onlyFingerTips ? 1.0f : 0.0f);
	connection->send_request(request.get());

	// wait for the response (blocking call)
	// if exception is sent means we are working on the real robot (need to be cleaned!)
	try {
		response.reset(connection->wait_for_request("get_contacts", MSEC_TM_U32_INF));
	} catch (std::exception e) {
		context.warning(e.what());
		return true;
	}
	printf("hand and object have %d contacts.\n", response->get_int("n_contacts"));
	if(response->get_int("n_contacts")) {
		std::list<bham_planner_interface::hand_contact_t> contacts = response->get_contacts("contacts");
		printf("all origins and normals in object-frame!\n");
		for(std::list<bham_planner_interface::hand_contact_t>::iterator i = contacts.begin(); i != contacts.end(); ++i) {
			hand_contact_t& c = *i;
			printf("contact at hand link '%s' origin: %.3f, %.3f, %.3f, normal: %.3f, %.3f, %.3f\n",
				    c.link_name.c_str(),
				    c.origin[0], c.origin[1], c.origin[2], 
				    c.normal[0], c.normal[1], c.normal[2]);
			for (size_t j = 0; j < 3; ++j) {
				contact[j] = c.origin[j];
				normal[j] = c.normal[j];
			}
		}
		return true;
	}
	return false;
}

//------------------------------------------------------------------------------

Mat34 RobotJustin::recvGlobalPose() {
	// create and set the request to get the transformation from Justin's base to shoulder
	request.reset(request_t::create());
	request->set("request", "get_arm_base");
	request->set("side", "right");
	connection->send_request(request.get());

	// wait for the response (blocking call)
	response.reset(connection->wait_for_request("get_arm_base", MSEC_TM_U32_INF));
	if (response == NULL)
		throw Message(Message::LEVEL_INFO, "RobotJustin::updateGlobalPose(): no response has been received");
	
	// set the global pose of the multicntr with the new pose
	frame_t base = response->get_frame("arm_base");
	golem::Mat34 m;
	// frame_t (row major 3x4) to mat34
	for (size_t i = 0; i < 3; ++i) {
		const size_t j = i*4;
		m.R.setRow(i, golem::Vec3(base[j], base[j+1], base[j+2]));
		m.p.v[i] = base[j+3];
	}
	request.reset();
	response.reset();
	return m;
}

//------------------------------------------------------------------------------

void golem::XMLData(RobotJustin::Desc &val, Context* context, XMLContext* xmlcontext, bool create) {
	XMLData((MultiCtrl::Desc&)val, context, xmlcontext, create);

	golem::XMLData("host", val.host, xmlcontext->getContextFirst("bham_planner_interface"), create);
	golem::XMLData("port", val.port, xmlcontext->getContextFirst("bham_planner_interface"), create);
	golem::XMLData("request", val.requestName, xmlcontext->getContextFirst("bham_planner_interface"), create);
	
	golem::XMLData(val.telemetryReaderDesc, xmlcontext->getContextFirst("telemetry_reader"), create);
}

//------------------------------------------------------------------------------
