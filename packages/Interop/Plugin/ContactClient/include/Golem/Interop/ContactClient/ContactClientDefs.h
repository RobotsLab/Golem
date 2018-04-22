/** @file ContactClientDefs.h
*
* @author	Marek Kopicki
*
*/

#pragma once
#ifndef _GOLEM_INTEROP_PLUGIN_CONTACT_CLIENT_CONTACT_CLIENTDEFS_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_PLUGIN_CONTACT_CLIENT_CONTACT_CLIENTDEFS_H_

//------------------------------------------------------------------------------

#include "GolemInteropInterface.h"
#include "GolemInteropContactInterface.h"

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	/** Contact communication defined as request/response service. */
	enum ContactStream {
		/** SensorCloud::capture() */
		CONTACT_STREAM_SENSORCLOUD_CAPTURE = 0,

		/** Controller::lookupState() */
		CONTACT_STREAM_CONTROLLER_LOOKUPSTATE,

		/** Controller::sendCommand() */
		CONTACT_STREAM_CONTROLLER_SENDCOMMAND,

		/** Controller::waitForTrajectoryEnd() */
		CONTACT_STREAM_CONTROLLER_WAITFORTRAJECTORYEND,

		/** Controller::waitForCycleBegin() */
		CONTACT_STREAM_CONTROLLER_WAITFORCYCLEBEGIN,

		/** Controller::cycleDuration() */
		CONTACT_STREAM_CONTROLLER_CYCLEDURATION,

		/** Controller::time() */
		CONTACT_STREAM_CONTROLLER_TIME,

		/** Planner::findTarget() */
		CONTACT_STREAM_PLANNER_FINDTARGET,

		/** Planner::findTrajectory() */
		CONTACT_STREAM_PLANNER_FINDTRAJECTORY,

		/** Contact::findFeatures() */
		CONTACT_STREAM_CONTACT_FINDFEATURES,

		/** Contact::findModel() */
		CONTACT_STREAM_CONTACT_FINDMODEL,

		/** Contact::findQuery() */
		CONTACT_STREAM_CONTACT_FINDQUERY,

		/** Contact::findQuery() */
		CONTACT_STREAM_CONTACT_FINDQUERY_FROM_DEFAULT_MODEL,

		/** Contact::selectTrajectory() */
		CONTACT_STREAM_CONTACT_SELECTTRAJECTORY,

		/** GraspCloud::findTrajectories() */
		CONTACT_STREAM_GRASPCLOUD_FINDTRAJECTORIES,
	};

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_PLUGIN_CONTACT_CLIENT_CONTACT_CLIENTDEFS_H_