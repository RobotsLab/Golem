/** @file Tools.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/App/Common/Creator.h>
#include <Golem/App/Common/Tools.h>
#include <Golem/Math/Data.h>
#include <Golem/Ctrl/Data.h>
#include <sstream>
#include <iomanip>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

char* golem::strdup(const char* src) {
	if (src == NULL)
		return NULL;
	
	size_t len = strlen(src) + 1;
	char* dst = new char [len];

	if (dst != NULL) {
		memcpy(dst, src, len);
	}

	return dst;
}

char* golem::strndup(const char* src, size_t n) {
	if (src == NULL)
		return NULL;
	
	size_t len = strlen(src) + 1;
	if (len > n + 1)
		len = n + 1;

	char* dst = new char [len];

	if (dst != NULL) {
		memcpy(dst, src, len);
		dst[n] = 0;
	}

	return dst;
}

char* golem::istreamstrdup(std::istream& istr, char delim) {
	std::streamoff pos = istr.tellg(), shift = 0;
	size_t len = 1;
	
	while (shift < 2) {
		char c;
		istr.get(c);
		
		if (!istr)
			break;
		else if (c == delim) {
			if (len > 1)
				break;
			shift++;
		}
		else
			len++;
	}

	char* dst = new char [len];
	dst[0] = 0;
	
	istr.seekg(pos + shift);
	if (shift < 2)
		istr >> dst;
	
	return dst;
}

//------------------------------------------------------------------------------

void golem::controllerInfo(Controller& controller) {
	MessageStream *stream = controller.getContext().getMessageStream();
	const Controller::State::Info& stateInfo = controller.getStateInfo();
	
	// Controller info
	stream->write(Message::LEVEL_INFO, "%s\n", controller.getName().c_str());
	stream->write(Message::LEVEL_INFO, "Cycle duration          = %.5f [sec]\n", controller.getCycleDuration());
	stream->write(Message::LEVEL_INFO, "Command latency         = %.5f [sec]\n", controller.getCommandLatency());
	
	for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
		const Chain* chain = controller.getChains()[i];

		stream->write(Message::LEVEL_INFO, "%s\n", chain->getName().c_str());
		
		for (Configspace::Index j = stateInfo.getJoints(i).begin(); j < stateInfo.getJoints(i).end(); ++j) {
			const Joint* joint = controller.getJoints()[j];

			stream->write(Message::LEVEL_INFO, "\t%s\n", joint->getName().c_str());
			stream->write(Message::LEVEL_INFO, "\t\tpos = <%.4f, %.4f> [rad]\n", joint->getMin().pos,  joint->getMax().pos);
			stream->write(Message::LEVEL_INFO, "\t\tvel = <%.4f, %.4f> [rad/sec]\n", joint->getMin().vel,  joint->getMax().vel);
			stream->write(Message::LEVEL_INFO, "\t\tacc = <%.4f, %.4f> [rad/sec^2]\n", joint->getMin().acc,  joint->getMax().acc);
		}
	}
}

void golem::controllerState(Controller& controller, const GenConfigspaceState &state) {
	MessageStream *stream = controller.getContext().getMessageStream();
	const Controller::State::Info& stateInfo = controller.getStateInfo();

	stream->write(Message::LEVEL_INFO, "%s, t = %.4f [sec]\n", controller.getName().c_str(), state.t);
	
	for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
		const Chain* chain = controller.getChains()[i];
		
		for (Configspace::Index j = stateInfo.getJoints(i).begin(); j < stateInfo.getJoints(i).end(); ++j) {
			const Joint* joint = controller.getJoints()[j];
			
			stream->write(Message::LEVEL_INFO,
				"%s, %s: (%.4f [rad], %.4f [rad/sec], %.4f [rad/sec^2])\n",
				chain->getName().c_str(), joint->getName().c_str(), state.cpos[j], state.cvel[j], state.cacc[j]
			);
		}
	}
}

void golem::controllerPosition(Controller& controller, const GenConfigspaceState &state) {
	MessageStream *stream = controller.getContext().getMessageStream();
	const Controller::State::Info& stateInfo = controller.getStateInfo();
	
	std::stringstream string;
	string << std::setw(7) << std::setprecision(4) << std::fixed;

	string << controller.getName() << ": t = " << state.t;

	for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
		//const Chain* chain = controller.getChains()[i];
		
		string << ", {";

		for (Configspace::Index j = stateInfo.getJoints(i).begin(); j < stateInfo.getJoints(i).end(); ++j) {
			//const Joint* joint = controller.getJoints()[j];
			
			string << state.cpos[j];
			if (j + 1 < stateInfo.getJoints(i).end())
				string << ", ";
		}

		string << "}";
	}

	stream->write(Message::LEVEL_INFO, "%s\n", string.str().c_str());
}

//------------------------------------------------------------------------------

void golem::fromCartesianPose(golem::Mat34& pose, const golem::Vec3 &p, const golem::Vec3 &q) {
	pose.p = p;
	pose.R.setId();

	golem::Mat33 tmp;
	
	tmp.rotX(q.v1);
	pose.R.multiply(tmp, pose.R);
	
	tmp.rotY(q.v2);
	pose.R.multiply(tmp, pose.R);
	
	tmp.rotZ(q.v3);
	pose.R.multiply(tmp, pose.R);
}

//------------------------------------------------------------------------------

void golem::XMLData(GenConfigspaceState &val, XMLContext* context, bool create) {
	golem::XMLData("dt", val.t, context, create); // time increment
	golem::XMLData(val.cpos, "c", context, create);
	try {
		golem::XMLData(val.cvel, "v", context, create);
		golem::XMLData(val.cacc, "a", context, create);
	}
	catch (const Message&) {}
}

void golem::XMLData(Controller::State &val, XMLContext* context, bool create) {
	golem::XMLData((GenConfigspaceState&)val, context, create);
}

//------------------------------------------------------------------------------

void golem::XMLData(golem::RobotPose &val, golem::XMLContext* xmlcontext, bool create) {
	if (create) {
		golem::XMLData(val.c, xmlcontext, true);
		golem::XMLData(val.w, xmlcontext, true);
		golem::XMLData("dt", val.t, xmlcontext, true);
		golem::XMLData("flags", val.flags, xmlcontext, true);
	}
	else {
		val.configspace = false;
		val.position = false;
		val.orientation = false;

		try {
			golem::XMLData(val.c, xmlcontext);
			val.configspace = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::XMLData(val.w.p, xmlcontext);
			val.position = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::Quat quat;
			golem::XMLData(quat, xmlcontext);
			val.w.R.fromQuat(quat);
			val.orientation = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::Real angle;
			golem::Vec3 axis;
			golem::XMLDataAngleAxis(angle, axis, xmlcontext);
			val.w.R.fromAngleAxis(angle, axis);
			val.orientation = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::Real roll, pitch, yaw;
			golem::XMLDataEuler(roll, pitch, yaw, xmlcontext);
			val.w.R.fromEuler(roll, pitch, yaw);
			val.orientation = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::XMLData(val.w.R, xmlcontext);
			val.orientation = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::Twist twist;
			golem::Real theta;
			golem::XMLData(twist, theta, xmlcontext);
			val.w.fromTwist(twist, theta);
			val.orientation = true;
			val.position = true;
		}
		catch (golem::MsgXMLParser&) {}

		if (!val.configspace && !val.position && !val.orientation)
			throw Message(Message::LEVEL_ERROR, "XMLData(): %s: invalid pose description", xmlcontext->getName().c_str());

		try {
			golem::XMLData("dt", val.t, xmlcontext);
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::XMLData("flags", val.flags, xmlcontext);
		}
		catch (golem::MsgXMLParser&) {}
	}
}

//------------------------------------------------------------------------------
