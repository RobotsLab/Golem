/** @file Tools.h
 * 
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
#ifndef _GOLEM_DEMO_COMMON_TOOLS_H_
#define _GOLEM_DEMO_COMMON_TOOLS_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Controller.h>
#include <Golem/Sys/XMLData.h>
#include <map>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

char* strdup(const char* src);

char* strndup(const char* src, size_t n);

char* istreamstrdup(std::istream& istr, char delim = '\n');

//------------------------------------------------------------------------------

/* Displays most important controller parameters. */
void controllerInfo(Controller& controller);

/* Displays controller state (time, pose, pose velocity, pose acceleration).
* @param controller		Controller interface
* @param state			state to be displayed
*/
void controllerState(Controller& controller, const GenConfigspaceState &state);

/* Displays controller position + time.
* @param controller		Controller interface
* @param state			state to be displayed
*/
void controllerPosition(Controller& controller, const GenConfigspaceState &state);

//------------------------------------------------------------------------------

/* Computes pose from given position and Euler angles.
* @param pose			computed pose
* @param p				XYZ position
* @param q				XYZ Euler angles
*/
void fromCartesianPose(golem::Mat34& pose, const golem::Vec3 &p, const golem::Vec3 &q);

//------------------------------------------------------------------------------

/** Loads configuration state from xml */
void XMLData(GenConfigspaceState &val, XMLContext* context, bool create = false);

/** Loads configuration state from xml */
void XMLData(Controller::State &val, XMLContext* context, bool create = false);

/** Trajectory map */
typedef std::map<std::string, Controller::State::Seq> TrajectoryConfigspaceMap;

//------------------------------------------------------------------------------

/** Robot coordinates */
class RobotPose {
public:
	typedef std::vector<golem::Real> RealSeq;
	typedef std::vector<RobotPose> Seq;
	typedef std::map<std::string, Seq> Map;

	/** Configuration space coordinates */
	RealSeq c;
	/** Workspace coordinates */
	golem::Mat34 w;

	/** Duration */
	golem::SecTmReal t;
	/** Flags */
	std::string flags;

	/** Configspace type */
	bool configspace;
	/** Workspace position and orientation type */
	bool position, orientation;

	RobotPose() {
		setToDefault();
	}
	RobotPose(const RealSeq& c, const golem::Mat34& w) : c(c), w(w) {
	}

	void setToDefault() {
		c.clear();
		w.setToDefault();
		t = golem::SEC_TM_REAL_ZERO;
		flags.clear();
		configspace = true;
		position = orientation = false;
	}
};

/** Reads/writes object from/to a given XML context */
void XMLData(RobotPose& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

/** Loads trajectory map */
template <typename _TrajectoryMap> void loadTrajectoryMap(const Controller& controller, XMLContext* context, _TrajectoryMap &map, const typename _TrajectoryMap::mapped_type::value_type& dflt = typename _TrajectoryMap::mapped_type::value_type()) {
	const std::string name = "trajectory";
	std::pair<XMLContext::XMLContextMap::const_iterator, XMLContext::XMLContextMap::const_iterator> range = context->getContextMap().equal_range(name);
	if (range.first == range.second)
		throw MsgXMLParserNameNotFound(Message::LEVEL_ERROR, "loadTrajectoryMap(): Name not found: %s %s", context->getName().c_str(), name.c_str());

	for (XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second; ++i) {
		try {
			XMLContext* context = const_cast<XMLContext*>(&i->second);

			typename _TrajectoryMap::mapped_type seq;
			XMLData(seq, seq.max_size(), context, "waypoint", false, dflt);
			for (typename _TrajectoryMap::mapped_type::iterator j = seq.begin(), k = seq.begin(); j != seq.end() && ++k != seq.end(); ++j)
				k->t += j->t;

			std::string name;
			XMLData("name", name, context);

			map[name] = seq;
		}
		catch (const Message&) {}
	}
}

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_TOOLS_H_*/
