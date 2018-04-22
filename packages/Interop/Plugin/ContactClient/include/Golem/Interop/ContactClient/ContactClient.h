/** @file ContactClient.h
*
* @author	Marek Kopicki
*
*/

#pragma once
#ifndef _GOLEM_INTEROP_PLUGIN_CONTACT_CLIENT_CONTACT_CLIENT_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_PLUGIN_CONTACT_CLIENT_CONTACT_CLIENT_H_

//------------------------------------------------------------------------------

#include "GolemInteropContactInterface.h"
#include "GolemInteropStreamSocket.h"

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_INTEROP_LIBRARY_DECLDIR void* InterfaceLoader(const char*);
};

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	/** Interface implementation */
	class ContactClient : public golem::interop::SensorCloud, public golem::interop::Controller, public golem::interop::Planner, public golem::interop::Contact, public golem::interop::GraspCloud {
	public:
		/** Create implementation */
		ContactClient(const std::string& param);

		/** SensorCloud: Capture point cloud
		*	@param[out]	cloud			captured point cloud
		*/
		virtual void capture(Point3DCloud& cloud);


		/**	Controller: State of a robot at time t
		*	@param[in]	t				query time [sec]
		*	@param[out]	state			robot state
		*/
		virtual void lookupState(double t, Config& state) const;

		/**	Controller: Send position command without blocking
		*	@param[in]	command			command sequence begin
		*	@param[in]	size			number of commands in the sequence
		*/
		virtual void sendCommand(const Config* command, std::uintptr_t size = 1);

		/**	Controller: Wait for end of the robot trajectory execution
		*	@param[in]	timewait		maximum time wait [sec]
		*	@return						true if success, false otherwise
		*/
		virtual bool waitForTrajectoryEnd(double timewait = std::numeric_limits<double>::max());

		/**	Controller: Wait for begin of the robot control cycle
		*	@param[in]	timewait		maximum time wait [sec]
		*	@return						true if success, false otherwise
		*/
		virtual bool waitForCycleBegin(double timewait = std::numeric_limits<double>::max());

		/** Controller: Robot control cycle duration
		*	@return						cycle duration [sec]
		*/
		virtual double cycleDuration() const;

		/** Controller: Current time counted from an arbitrary moment
		*	@return						current time [sec]
		*/
		virtual double time() const;


		/** Planner: Finds (optimal) trajectory target in the obstacle-free configuration space.
		* @param[in]	cbegin			trajectory begin in the configuration space
		* @param[in]	wend			query trajectory end (target) in the workspace
		* @param[out]	cend			computed trajectory end (target) in the configuration space
		* @param[out]	werr			workspace error per kinematic chain
		*/
		virtual void findTarget(const ConfigspaceCoord &cbegin, const WorkspaceCoord& wend, ConfigspaceCoord &cend, WorkspaceDist& werr);

		/** Planner: Finds obstacle-free trajectory in the configuration space from begin to end.
		* @param[in]	cbegin			trajectory begin in the configuration space
		* @param[in]	cend			trajectory end in the configuration space
		* @param[out]	ctrajectory		output trajectory in the configuration space
		*/
		virtual void findTrajectory(const ConfigspaceCoord &cbegin, const ConfigspaceCoord &cend, Config::Seq &ctrajectory);


		/** Contact: Training and test: Process a sequence of point clouds
		*	@param[in]	inp				point cloud sequence to be processed
		*	@param[out]	out				set of features
		*/
		virtual void findFeatures(const Point3DCloud::Seq& inp, Feature3D::Seq& out);

		/** Contact: Training: Creates contact models from training data - a set of trajectories and point cloud features
		*	@param[in]	training		training data
		*	@param[out]	models			contact and configuration models
		*/
		virtual void findModel(const Training3D::Map& training, Model3D::Map& models);

		/** Contact: Test: Finds path hypotheses from contact models and point cloud features
		*	@param[in]	models			contact and configuration models
		*	@param[in]	features		point cloud features
		*	@param[out]	query			query data
		*/
		virtual void findQuery(const Model3D::Map& models, const Feature3D::Seq& features, Query& query);

		/** Contact: Test: Finds path hypotheses from default contact models and point cloud features
		*	@param[in]	features		point cloud features
		*	@param[out]	query			query data
		*/
		virtual void findQuery(const Feature3D::Seq& features, Query& query);

		/** Contact: Selection: Selects most likely path hypothesis and computes collision-free trajectory
		*	@param[in]	query			query data
		*	@param[out]	trajectory		most likely feasible trajectory
		*/
		virtual void selectTrajectory(const Query& query, Trajectory& trajectory);


		/** GraspCloud: Test: Generate grasp trajectories from a sequence of point clouds
		*	@param[in]	clouds			point cloud sequence to be processed
		*	@param[out]	trajectories	set of trajectories
		*/
		virtual void findTrajectories(const Point3DCloud::Seq& clouds, Trajectory::Seq& trajectories);

	private:
		/** TCP/IP client */
		Client client;
		/** Stream */
		Stream::Ptr stream;
	};

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_PLUGIN_CONTACT_CLIENT_CONTACT_CLIENT_H_