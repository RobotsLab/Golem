/** @file Contact.h
*
* @author	Marek Kopicki
*
*/

#pragma once
#ifndef _GOLEM_INTEROP_PLUGIN_GOLEM_CONTACT_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_PLUGIN_GOLEM_CONTACT_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Cloud.h>
#include <Golem/App/Player.h>
#include "GolemInteropContactInterface.h"
#include "GolemInteropPCLDefs.h"

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_INTEROP_LIBRARY_DECLDIR void* InterfaceLoader(const char*);
};

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	/** PointFeature conversion */
	void convert(const golem::Cloud::PointFeature& src, Feature3D& dst) {
		convert(static_cast<const pcl::PointXYZRGBNormal&>(src), static_cast<Point3D&>(dst));
		convert(src.direction_data[0], dst.direction.v[0]);
		convert(src.direction_data[1], dst.direction.v[1]);
		convert(src.direction_data[2], dst.direction.v[2]);
		convert(src.descriptor_type, dst.featureType);
		convert(src.descriptor_size, dst.featureSize);
		golem::interop::copy(src.descriptor_data, src.descriptor_data + src.descriptor_size, dst.featureData.v);
	}
	/** PointFeature conversion */
	void convert(const Feature3D& src, golem::Cloud::PointFeature& dst) {
		convert(static_cast<const Point3D&>(src), static_cast<pcl::PointXYZRGBNormal&>(dst));
		convert(src.direction.v[0], dst.direction_data[0]);
		convert(src.direction.v[1], dst.direction_data[1]);
		convert(src.direction.v[2], dst.direction_data[2]);
		convert(src.featureType, dst.descriptor_type);
		convert(src.featureSize, dst.descriptor_size);
		golem::interop::copy(src.featureData.v, src.featureData.v + src.featureSize, dst.descriptor_data);
	}

	/** Interface implementation */
	class GolemContact : public golem::Player {
	public:
		/** ContactDesc description */
		class ContactDesc {
		public:
			/** Data bundle default name */
			std::string bundle;

			/** Depth camera */
			std::string camera;

			/** Raw image handler */
			std::string imageHandler;
			/** Raw image item */
			std::string imageItem;

			/** Processed image/feature handler */
			std::string processHandler;
			/** Processed image/feature item */
			std::string processItem;

			/** Contact model handler */
			std::string modelHandler;
			/** Contact model item */
			std::string modelItem;
			/** Contact model item used in Grasp interface */
			std::string modelGraspItem;

			/** Contact query handler */
			std::string queryHandler;
			/** Contact query item */
			std::string queryItem;

			/** Trajectory handler */
			std::string trjHandler;
			/** Trajectory item */
			std::string trjItem;
			/** Trajectory selection item */
			std::string trjSelectionItem;

			/** Constructs from description object */
			ContactDesc() {
				ContactDesc::setToDefault();
			}
			/** Sets the parameters to the default values */
			void setToDefault() {
				bundle.clear();
				camera.clear();
				imageHandler.clear();
				imageItem.clear();
				processHandler.clear();
				processItem.clear();
				modelHandler.clear();
				modelItem.clear();
				modelGraspItem.clear();
				queryHandler.clear();
				queryItem.clear();
				trjHandler.clear();
				trjItem.clear();
				trjSelectionItem.clear();
			}
			/** Assert that the description is valid. */
			void assertValid(const golem::Assert::Context& ac) const {
				golem::Assert::valid(bundle.length() > 0, ac, "bundle: invalid");
				golem::Assert::valid(camera.length() > 0, ac, "camera: invalid");
				golem::Assert::valid(imageHandler.length() > 0, ac, "imageHandler: invalid");
				golem::Assert::valid(imageItem.length() > 0, ac, "imageItem: invalid");
				golem::Assert::valid(processHandler.length() > 0, ac, "processHandler: invalid");
				golem::Assert::valid(processItem.length() > 0, ac, "processItem: invalid");
				golem::Assert::valid(modelHandler.length() > 0, ac, "modelHandler: invalid");
				golem::Assert::valid(modelItem.length() > 0, ac, "modelItem: invalid");
				golem::Assert::valid(modelGraspItem.length() > 0, ac, "modelGraspItem: invalid");
				golem::Assert::valid(queryHandler.length() > 0, ac, "queryHandler: invalid");
				golem::Assert::valid(queryItem.length() > 0, ac, "queryItem: invalid");
				golem::Assert::valid(trjHandler.length() > 0, ac, "trjHandler: invalid");
				golem::Assert::valid(trjItem.length() > 0, ac, "trjItem: invalid");
			}
			/** Load descritpion from xml context. */
			void load(golem::Context& context, const golem::XMLContext* xmlcontext) {
				golem::XMLData("bundle", bundle, const_cast<golem::XMLContext*>(xmlcontext));
				golem::XMLData("camera", camera, const_cast<golem::XMLContext*>(xmlcontext));
				golem::XMLData("handler", imageHandler, xmlcontext->getContextFirst("image"));
				golem::XMLData("item", imageItem, xmlcontext->getContextFirst("image"));
				golem::XMLData("handler", processHandler, xmlcontext->getContextFirst("process"));
				golem::XMLData("item", processItem, xmlcontext->getContextFirst("process"));
				golem::XMLData("handler", modelHandler, xmlcontext->getContextFirst("model"));
				golem::XMLData("item", modelItem, xmlcontext->getContextFirst("model"));
				golem::XMLData("item_grasp", modelGraspItem, xmlcontext->getContextFirst("model"));
				golem::XMLData("handler", queryHandler, xmlcontext->getContextFirst("query"));
				golem::XMLData("item", queryItem, xmlcontext->getContextFirst("query"));
				golem::XMLData("handler", trjHandler, xmlcontext->getContextFirst("trajectory"));
				golem::XMLData("item", trjItem, xmlcontext->getContextFirst("trajectory"));
				golem::XMLData("item_select", trjSelectionItem, xmlcontext->getContextFirst("trajectory"));
			}
		};

		/** GolemContact interface container controls life time of the encapsulated interfaces */
		class Desc : public golem::Player::Desc, public golem::interop::Application, public golem::interop::SensorCloud, public golem::interop::Controller, public golem::interop::Planner, public golem::interop::Contact, public golem::interop::GraspCloud {
		public:
			/** GolemContact description */
			ContactDesc contactDesc;

			/** Sets the parameters to the default values */
			virtual void setToDefault() {
				golem::Player::Desc::setToDefault();
				contactDesc.setToDefault();
			}
			/** Assert that the description is valid. */
			virtual void assertValid(const golem::Assert::Context& ac) const {
				golem::Player::Desc::assertValid(ac);
				contactDesc.assertValid(golem::Assert::Context(ac, "contactDesc."));
			}
			/** Load descritpion from xml context. */
			virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext) {
				golem::Player::Desc::load(context, xmlcontext);
				contactDesc.load(context, xmlcontext->getContextFirst("contact"));
			}

			/** Creates interface */
			GOLEM_CREATE_FROM_OBJECT_DESC1(GolemContact, golem::Object::Ptr, golem::Scene&)

			/** Construct from config file */
			Desc(const std::string& param);
			/** Runs Application */
			virtual void run(int argc, char *argv[]);

			virtual void capture(Point3DCloud& cloud) { pInterface->capture(cloud); }
			virtual void lookupState(double t, Config& state) const { pInterface->lookupState(t, state); }
			virtual void sendCommand(const Config* command, std::uintptr_t size = 1) { pInterface->sendCommand(command, size); }
			virtual bool waitForTrajectoryEnd(double timewait = std::numeric_limits<double>::max()) { return pInterface->waitForTrajectoryEnd(timewait); }
			virtual bool waitForCycleBegin(double timewait = std::numeric_limits<double>::max()) { return pInterface->waitForCycleBegin(timewait); }
			virtual double cycleDuration() const { return pInterface->cycleDuration(); }
			virtual double time() const { return pInterface->time(); }
			virtual void findTarget(const ConfigspaceCoord &cbegin, const WorkspaceCoord& wend, ConfigspaceCoord &cend, WorkspaceDist& werr) { pInterface->findTarget(cbegin, wend, cend, werr); }
			virtual void findTrajectory(const ConfigspaceCoord &cbegin, const ConfigspaceCoord &cend, Config::Seq &ctrajectory) { pInterface->findTrajectory(cbegin, cend, ctrajectory); }
			virtual void findFeatures(const Point3DCloud::Seq& inp, Feature3D::Seq& out) { pInterface->findFeatures(inp, out); }
			virtual void findModel(const Training3D::Map& training, Model3D::Map& models) { pInterface->findModel(training, models); }
			virtual void findQuery(const Model3D::Map& models, const Feature3D::Seq& features, Query& query) { pInterface->findQuery(models, features, query); }
			virtual void findQuery(const Feature3D::Seq& features, Query& query) { pInterface->findQuery(features, query); }
			virtual void selectTrajectory(const Query& query, Trajectory& trajectory) { pInterface->selectTrajectory(query, trajectory); }
			virtual void findTrajectories(const Point3DCloud::Seq& clouds, Trajectory::Seq& trajectories) { pInterface->findTrajectories(clouds, trajectories); }
			virtual bool dispatch() { return pInterface->dispatch(); }
			virtual void setCallbackTerminate(CallbackTerminate callback) { pInterface->setCallbackTerminate(callback); }
			virtual void release();

		private:
			GolemContact* pInterface;
		};

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

		/** Application: Dispatch events once
		*	@return						false for application exit, true otherwise
		*/
		virtual bool dispatch();

		/** Application: Terminate callback (called by application)
		*	@param[in]	callback		terminate callback
		*/
		virtual void setCallbackTerminate(Application::CallbackTerminate callback);

	private:
		/** Contact description */
		ContactDesc contactDesc;

		/** Concurrent interface calls are not allowed */
		golem::CriticalSection cs;

		/** Raw image handler */
		golem::data::Handler* imageHandler;
		/** Processed image/feature handler */
		golem::data::Handler* processHandler;
		/** Contact model handler */
		golem::data::Handler* modelHandler;
		/** Contact query handler */
		golem::data::Handler* queryHandler;
		/** Trajectory handler */
		golem::data::Handler* trjHandler;

		/** Add data item */
		golem::data::Item::Map::iterator addItem(const std::string& name, golem::data::Item::Ptr item, bool erase = false);

		/** Create implementation */
		void create(const Desc& desc);

		/** Basic initialisation */
		GolemContact(golem::Scene &scene);
	};

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_PLUGIN_GOLEM_CONTACT_H_