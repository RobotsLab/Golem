/** @file GolemInteropInterface.h
*
* @author	Marek Kopicki
*
*/

#pragma once
#ifndef _GOLEM_INTEROP_INTEROP_INTERFACE_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_INTEROP_INTERFACE_H_

//------------------------------------------------------------------------------

#include "GolemInteropDefs.h"
#include "GolemInteropPlugin.h"

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	class Master;
	class Sensor;
	class SensorCloud;
	class Grasp;
	class GraspCloud;
	class Controller;
	class Planner;

	/*************************************************************************
	*
	* Master interface
	*
	**************************************************************************/

	/** Master interface implements "logic" of the system */
	class Master : virtual public Interface {
	public:
		/** Take over control given set of interfaces
		*	@param[in]	interfaces		set of interfaces
		*/
		virtual void run(const Map& interfaces) = 0;
	};

	/*************************************************************************
	*
	* Application interface
	*
	**************************************************************************/

	/** Application interface communicates with user interface */
	class Application : virtual public Interface {
	public:
		/** Terminate calback
		*/
		typedef std::function<void(void)> CallbackTerminate;

		/** Dispatch events once (called by Master)
		*	@return						false for application exit, true otherwise
		*/
		virtual bool dispatch() = 0;

		/** Terminate callback (called by application)
		*	@param[in]	callback		terminate callback
		*/
		virtual void setCallbackTerminate(CallbackTerminate callback) = 0;
	};

	/*************************************************************************
	*
	* Sensor interfaces
	*
	**************************************************************************/

	/** Sensor interface */
	class Sensor : virtual public Interface {
	public:
	};

	/** Point cloud sensor interface */
	class SensorCloud : public Sensor {
	public:
		/** Capture point cloud
		*	@param[out]	cloud			captured point cloud
		*/
		virtual void capture(Point3DCloud& cloud) = 0;
	};

	/*************************************************************************
	*
	* Grasp interfaces
	*
	**************************************************************************/

	/** Grasp interface */
	class Grasp : virtual public Interface {
	public:
	};

	/** Grasp point cloud interface */
	class GraspCloud : public Grasp {
	public:
		/** Test: Generate grasp trajectories from a sequence of point clouds
		*	@param[in]	clouds			point cloud sequence to be processed
		*	@param[out]	trajectories	set of trajectories
		*/
		virtual void findTrajectories(const Point3DCloud::Seq& clouds, Trajectory::Seq& trajectories) = 0;
	};

	/*************************************************************************
	*
	* Control interface
	*
	**************************************************************************/

	/** Controller interface */
	class Controller : virtual public Interface {
	public:
		/**	State of a robot at time t
		*	@param[in]	t				query time [sec]
		*	@param[out]	state			robot state
		*/
		virtual void lookupState(double t, Config& state) const = 0;

		/**	Send position command without blocking
		*	@param[in]	command			command sequence begin
		*	@param[in]	size			number of commands in the sequence
		*/
		virtual void sendCommand(const Config* command, std::uintptr_t size = 1) = 0;

		/**	Wait for end of the robot trajectory execution
		*	@param[in]	timewait		maximum time wait [sec]
		*	@return						true if success, false otherwise
		*/
		virtual bool waitForTrajectoryEnd(double timewait = std::numeric_limits<double>::max()) = 0;

		/**	Wait for begin of the robot control cycle
		*	@param[in]	timewait		maximum time wait [sec]
		*	@return						true if success, false otherwise
		*/
		virtual bool waitForCycleBegin(double timewait = std::numeric_limits<double>::max()) = 0;

		/** Robot control cycle duration
		*	@return						cycle duration [sec]
		*/
		virtual double cycleDuration() const = 0;

		/** Current time counted from an arbitrary moment
		*	@return						current time [sec]
		*/
		virtual double time() const = 0;
	};

	/*************************************************************************
	*
	* Planner interface
	*
	**************************************************************************/

	/** Planner interface */
	class Planner : virtual public Interface {
	public:
		/** Finds (optimal) trajectory target in the obstacle-free configuration space.
		* @param[in]	cbegin			trajectory begin in the configuration space
		* @param[in]	wend			query trajectory end (target) in the workspace
		* @param[out]	cend			computed trajectory end (target) in the configuration space
		* @param[out]	werr			workspace error per kinematic chain
		*/
		virtual void findTarget(const ConfigspaceCoord &cbegin, const WorkspaceCoord& wend, ConfigspaceCoord &cend, WorkspaceDist& werr) = 0;

		/** Finds obstacle-free trajectory in the configuration space from begin to end.
		* @param[in]	cbegin			trajectory begin in the configuration space
		* @param[in]	cend			trajectory end in the configuration space
		* @param[out]	ctrajectory		output trajectory in the configuration space
		*/
		virtual void findTrajectory(const ConfigspaceCoord &cbegin, const ConfigspaceCoord &cend, Config::Seq &ctrajectory) = 0;
	};

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_INTEROP_INTERFACE_H_