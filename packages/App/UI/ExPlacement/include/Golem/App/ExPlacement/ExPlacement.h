/** @file AppExPlacement.h
*
* Object placement demo
*
* @author	Marek Kopicki
* @author	Sebastian Zurek
*
* @copyright  Copyright (C) 2015 Marek Kopicki and Sebastian Zurek, University of Birmingham, UK
*
* @license  This file copy is licensed to you under the terms described in
*           the License.txt file included in this distribution.
*
*/

#pragma once
#ifndef _GOLEM_APP_EXPLACEMENT_EXPLACEMENT_H_ // if #pragma once is not supported
#define _GOLEM_APP_EXPLACEMENT_EXPLACEMENT_H_

#include <Golem/App/Player.h>
#include <Golem/Contact/Model.h>
#include <Golem/Tools/RBPose.h>
#include <Golem/Plugin/Ctrl.h>
#include <Golem/Contact/Model.h>
#include <Golem/Contact/Query.h>
#include <Golem/Contact/Manipulator.h>

//------------------------------------------------------------------------------

/** Name space */
namespace golem {

//------------------------------------------------------------------------------

/** AppExPlacement. */
class AppExPlacement : public golem::Player {
public:
	/** Model/Query any identifier */
	static const std::string ID_ANY;

	/** Data */
	class Data : public golem::Player::Data {
	public:
		/** Mode */
		enum Mode {
			/** Model data */
			MODE_MODEL,
			/** Query density */
			MODE_QUERY,
			/** Solution */
			MODE_SOLUTION,

			/** First */
			MODE_FIRST = MODE_MODEL,
			/** Last */
			MODE_LAST = MODE_SOLUTION,
		};

		/** Mode name */
		static const std::string ModeName[MODE_LAST + 1];

		/** Model training data */
		class Training {
		public:
			/** Collection */
			typedef std::multimap<std::string, Training> Map;
			/** Range */
			typedef std::pair<Map::const_iterator, Map::const_iterator> Range;

			class Waypoint {
			public:
				typedef std::vector<Waypoint> Seq;
	
				/** Model frame */
				golem::Mat34 frame;
				/** Contacts */
				golem::Contact3D::Data contacts;
				/** Points */
				golem::data::Point3D::Point::Seq points;
				/** Robot state */
				golem::Controller::State::Ptr state;
				
				Waypoint(const golem::Mat34& frame, const golem::Contact3D::Data& contacts, const golem::data::Point3D::Point::Seq& points, golem::Controller::State& state) {
					this->frame = frame;
					this->contacts = contacts;
					this->points = points;
					this->state.reset(&state);
				};

				Waypoint() {
					this->frame.setId();
					this->contacts.clear();
					this->points.clear();
					this->state.reset();
				};
			};
			
			Waypoint::Seq waypoints;

			/** Initialisation */
			Training() {
				waypoints.clear();
			};

			size_t insert(const Waypoint& w) {
				waypoints.insert(waypoints.end(), w);
				return waypoints.size();
			}
		};

		/** Query density */
		class Density : public golem::Sample<golem::Real> {
		public:
			/** Collection of distributions */
			typedef std::vector<Density> Seq;

			/** Type */
			std::string type;
			/** Object density */
			golem::Query::Pose::Seq object;
			/** Pose density */
			golem::Query::Pose::Seq pose;
			/** Path */
			golem::Manipulator::Waypoint::Seq path;
			/** Points */
			golem::data::Point3D::Point::Seq points;
			/** End-effector frame */
			golem::Mat34 frame;

			typedef std::multimap<std::string, Density::Seq> denMap;
		};

		/** Solution */
		class Solution {
		public:
			/** Collection of solutions */
			typedef std::vector<Solution> Seq;

			/** Solution likelihood */
			class Likelihood {
			public:
				/** Likelihood contact */
				golem::Real contact;
				/** Likelihood pose */
				golem::Real pose;
				/** Likelihood collision */
				golem::Real collision;
				/** Likelihood */
				golem::Real likelihood;

				Likelihood() {
					setToDefault();
				}
				void setToDefault() {
					contact = pose = collision = likelihood = golem::numeric_const<golem::Real>::MIN_EXP;
				}
				static bool isValid(golem::Real likelihood) {
					return likelihood > golem::REAL_ZERO;
				}
				void make() {
					likelihood = golem::numeric_const<golem::Real>::ONE;
					if (isValid(contact))
						likelihood *= contact;
					if (isValid(pose))
						likelihood *= pose;
					if (isValid(collision))
						likelihood *= collision;
				}
				void makeLog() {
					likelihood = golem::numeric_const<golem::Real>::ZERO;
					if (isValid(contact))
						likelihood += golem::Math::ln(contact);
					if (isValid(pose))
						likelihood += golem::Math::ln(pose);
					if (isValid(collision))
						likelihood += golem::Math::ln(collision);
				}
			};

			/** Type */
			std::string type;
			/** Pose */
			golem::RBCoord pose;
			/** Path */
			golem::Manipulator::Waypoint::Seq path;

			/** Likelihood */
			Likelihood likelihood;

			/** Query index */
			golem::U32 queryIndex;

			typedef std::multimap<std::string, Solution::Seq> solMap;
		};

		/** Cluster */
		class Cluster {
		public:
			/** Type-slot map */
			typedef std::multimap<std::string, Cluster> Map;
			/** Slot set */
			typedef std::set<golem::U32> Set;
			/** Slot counter */
			typedef std::map<std::string, Set> Counter;

			/** Slot */
			std::string slot;

			/** Set to default */
			static void setToDefault(const Map& map, Counter& counter, const Training::Map& training, bool ordered = true);
			/** Set occupied */
			static void setOccupied(const Map& map, Counter& counter, const std::string& type, golem::U32 index, bool ordered = true);
			/** Is occupied */
			static bool isOccupied(const Map& map, const Counter& counter, const std::string& type, golem::U32 index);
		};

		/** Data bundle default name */
		std::string dataName;

		/** Current Mode */
		Mode mode;

		/** Model triangles */
		golem::Vec3Seq modelVertices;
		/** Model triangles */
		golem::TriangleSeq modelTriangles;
		/** Model frame */
		golem::Mat34 modelFrame;
		/** Model frame offset */
		golem::Mat34 modelFrameOffset;

		/** Model robot state */
		golem::Controller::State::Ptr modelState;
		
		/** Query triangles */
		golem::Vec3Seq queryVertices;
		/** Query triangles */
		golem::TriangleSeq queryTriangles;
		/** Query frame */
		golem::Mat34 queryFrame;

		/** Training data */
		Training::Map training;

		/** Show query densities */
		bool queryShowDensities;

		/** Model training data type index */
		golem::U32 indexType;
		/** Model training data item index */
		golem::U32 indexItem;
		/** Model contact relation */
		golem::Contact3D::Relation contactRelation;

		/** Collection of distributions */
		Density::denMap densities;
		/** Density index */
		golem::U32 indexDensity;

		/** Solutions */
		Solution::solMap solutions;
		/** Solution index */
		golem::U32 indexSolution;

		/** Cluster counter */
		Data::Cluster::Counter clusterCounter;

		/** Data bundle description */
		class Desc : public golem::Player::Data::Desc {
		public:
			/** Creates the object from the description. */
			virtual golem::data::Data::Ptr create(golem::Context &context) const;
		};

		/** Get training item */
		Training::Map::iterator getTrainingItem();
		/** Set training item */
		void setTrainingItem(Training::Map::const_iterator ptr);

		/** Manager */
		virtual void setOwner(golem::Manager* owner);

		/** Creates render buffer of the bundle without items */
		virtual void createRender();

	protected:
		/** AppExPlacement */
		AppExPlacement* owner;

		/** Load from xml context */
		virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext, const golem::data::Handler::Map& handlerMap);
		/** Save to xml context */
		virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

		/** Creates data bundle */
		void create(const Desc& desc);
		/** Creates data bundle */
		Data(golem::Context &context);
	};

	friend class Data;

	/** Pose density description */
	class PoseDensity {
	public:
		/** map */
		typedef std::map<std::string, PoseDensity> Map;

		/** Std dev */
		golem::RBDist stdDev;
		/** Kernels */
		golem::U32 kernels;
		/** Path distance */
		golem::RBDist pathDist;
		/** Path distance std dev */
		golem::Real pathDistStdDev;

		/** Constructs from description object */
		PoseDensity() {
			PoseDensity::setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			stdDev.set(golem::Real(0.01), golem::Real(200.0));
			kernels = 100;
			pathDist.set(golem::Real(0.1), golem::Real(20.0));
			pathDistStdDev = golem::Real(1.0);
		};
		/** Assert that the description is valid. */
		void assertValid(const golem::Assert::Context& ac) const {
			golem::Assert::valid(stdDev.isValid(), ac, "stdDev: invalid");
			golem::Assert::valid(kernels > 0, ac, "kernels: <= 0");
			golem::Assert::valid(pathDist.isValid(), ac, "pathDist: invalid");
			golem::Assert::valid(pathDistStdDev > golem::REAL_EPS, ac, "pathDistStdDev: < eps");
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Optimisation description */
	class Optimisation {
	public:
		/** number of runs */
		size_t runs;
		/** number of steps per run */
		size_t steps;

		/** Simulated annealing minimum temperature */
		golem::Real saTemp;
		/** Simulated annealing temperature to local coordinate scaling factor */
		golem::RBDist saDelta;
		/** Simulated annealing temperature to energy scaling factor */
		golem::Real saEnergy;

		/** Constructs description object */
		Optimisation() {
			Optimisation::setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			runs = 1000;
			steps = 1000;

			saTemp = golem::Real(0.1);
			saDelta.set(golem::Real(1.0), golem::Real(1.0));
			saEnergy = golem::Real(0.5);
		}
		/** Assert that the description is valid. */
		void assertValid(const golem::Assert::Context& ac) const {
			golem::Assert::valid(runs > 0, ac, "runs: <= 0");
			golem::Assert::valid(steps > 0, ac, "steps: <= 0");
			golem::Assert::valid(saTemp >= golem::REAL_ZERO, ac, "saTemp: < 0");
			golem::Assert::valid(saDelta.isValid(), ac, "saDelta: invalid");
			golem::Assert::valid(saEnergy > golem::REAL_ZERO, ac, "saEnergy: <= 0");
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** AppExPlacement description */
	class Desc : public golem::Player::Desc {
	public:
		/** Data bundle default name */
		std::string dataName;

		/** Model pose estimation camera */
		std::string modelCamera;
		/** Model data handler */
		std::string modelHandler;
		/** Model data item */
		std::string modelItem;
		/** Model data item object */
		std::string modelItemObj;

		/** Model scan pose */
		golem::ConfigMat34 modelScanPose;
		/** Model colour solid */
		golem::RGBA modelColourSolid;
		/** Model colour wireframe */
		golem::RGBA modelColourWire;

		/** Model trajectory handler */
		std::string modelHandlerTrj;
		/** Model trajectory item */
		std::string modelItemTrj;

		/** Query pose estimation camera */
		std::string queryCamera;
		/** Query data handler */
		std::string queryHandler;
		/** Query data item */
		std::string queryItem;
		/** Query data item object */
		std::string queryItemObj;

		/** Query trajectory handler */
		std::string queryHandlerTrj;
		/** Query trajectory item */
		std::string queryItemTrj;

		/** Grasp force sensor */
		std::string graspSensorForce;
		/** Grasp force threshold */
		golem::Twist graspThresholdForce;
		/** Grasp force event - hand close time wait */
		golem::SecTmReal graspEventTimeWait;
		/** Grasp hand close duration */
		golem::SecTmReal graspCloseDuration;
		/** Grasp pose open (pre-grasp) */
		golem::ConfigMat34 graspPoseOpen;
		/** Grasp pose closed (grasp) */
		golem::ConfigMat34 graspPoseClosed;

		/** Object capture camera */
		std::string objectCamera;
		/** Object data handler (scan) */
		std::string objectHandlerScan;
		/** Object data handler (processed) */
		std::string objectHandler;
		/** Object data item (scan) */
		std::string objectItemScan;
		/** Object data item (processed) */
		std::string objectItem;
		/** Object scan pose */
		golem::ConfigMat34::Seq objectScanPoseSeq;
		/** Object manual frame adjustment */
		golem::RBAdjust objectFrameAdjustment;

		/** Model descriptions */
		golem::Model::Desc::Map modelDescMap;
		/** Contact appearance */
		golem::Contact3D::Appearance contactAppearance;

		/** Query descriptions */
		golem::Query::Desc::Map queryDescMap;
		/** Pose descriptions */
		PoseDensity::Map poseMap;

		/** Optimisation description */
		Optimisation optimisation;

		/** Manipulator description */
		golem::Manipulator::Desc::Ptr manipulatorDesc;
		/** Manipulator Appearance */
		golem::Manipulator::Appearance manipulatorAppearance;
		/** Manipulator pose distribution standard deviation */
		golem::RBDist manipulatorPoseStdDev;
		/** Maximum distance between frames in standard deviations */
		golem::Real manipulatorPoseStdDevMax;
		/** Manipulator trajectory item */
		std::string manipulatorItemTrj;

		/** Manipulation trajectory duration */
		golem::SecTmReal manipulatorTrajectoryDuration;
		/** Manipulation trajectory force threshold */
		golem::Twist trajectoryThresholdForce;

		/* Withdraw action hand release fraction */
		golem::Real withdrawReleaseFraction;
		/* Withdraw action lift distance */
		golem::Real withdrawLiftDistance;

		/** Cluster */
		Data::Cluster::Map clusterMap;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			golem::Player::Desc::setToDefault();


			dataDesc.reset(new Data::Desc);

			dataName.clear();

			modelCamera.clear();
			modelHandler.clear();
			modelItem.clear();
			modelItemObj.clear();

			modelScanPose.setToDefault();
			modelColourSolid.set(golem::RGBA::GREEN._U8[0], golem::RGBA::GREEN._U8[1], golem::RGBA::GREEN._U8[2], golem::numeric_const<golem::U8>::MAX / 8);
			modelColourWire.set(golem::RGBA::GREEN);

			modelHandlerTrj.clear();
			modelItemTrj.clear();

			queryCamera.clear();
			queryHandler.clear();
			queryItem.clear();
			queryItemObj.clear();

			queryHandlerTrj.clear();
			queryItemTrj.clear();

			graspSensorForce.clear();
			graspThresholdForce.setZero();
			graspEventTimeWait = golem::SecTmReal(2.0);
			graspCloseDuration = golem::SecTmReal(2.0);
			graspPoseOpen.setToDefault();
			graspPoseClosed.setToDefault();

			objectCamera.clear();
			objectHandlerScan.clear();
			objectHandler.clear();
			objectItemScan.clear();
			objectItem.clear();
			objectScanPoseSeq.clear();
			objectFrameAdjustment.setToDefault();

			modelDescMap.clear();
			contactAppearance.setToDefault();

			queryDescMap.clear();
			poseMap.clear();

			optimisation.setToDefault();

			manipulatorDesc.reset(new golem::Manipulator::Desc);
			manipulatorAppearance.setToDefault();
			manipulatorPoseStdDev.set(golem::Real(0.002), golem::Real(1000.0));
			manipulatorPoseStdDevMax = golem::Real(5.0);
			manipulatorItemTrj.clear();

			manipulatorTrajectoryDuration = golem::SecTmReal(5.0);
			trajectoryThresholdForce.setZero();

			withdrawReleaseFraction = golem::Real(0.5);
			withdrawLiftDistance = golem::Real(0.20);

			clusterMap.clear();
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const golem::Assert::Context& ac) const {
			golem::Player::Desc::assertValid(ac);

			golem::Assert::valid(dataDesc != nullptr && golem::is<Data::Desc>(dataDesc.get()), ac, "dataDesc: unknown type");

			golem::Assert::valid(dataName.length() > 0, ac, "dataName: invalid");

			golem::Assert::valid(modelCamera.length() > 0, ac, "modelCamera: invalid");
			golem::Assert::valid(modelHandler.length() > 0, ac, "modelHandler: invalid");
			golem::Assert::valid(modelItem.length() > 0, ac, "modelItem: invalid");
			golem::Assert::valid(modelItemObj.length() > 0, ac, "modelItemObj: invalid");

			modelScanPose.assertValid(golem::Assert::Context(ac, "modelScanPose."));

			golem::Assert::valid(modelHandlerTrj.length() > 0, ac, "modelHandlerTrj: invalid");
			golem::Assert::valid(modelItemTrj.length() > 0, ac, "modelItemTrj: invalid");

			golem::Assert::valid(queryCamera.length() > 0, ac, "queryCamera: invalid");
			golem::Assert::valid(queryHandler.length() > 0, ac, "queryHandler: invalid");
			golem::Assert::valid(queryItem.length() > 0, ac, "queryItem: invalid");
			golem::Assert::valid(queryItemObj.length() > 0, ac, "queryItemObj: invalid");

			golem::Assert::valid(queryHandlerTrj.length() > 0, ac, "queryHandlerTrj: invalid");
			golem::Assert::valid(queryItemTrj.length() > 0, ac, "queryItemTrj: invalid");

			golem::Assert::valid(graspSensorForce.length() > 0, ac, "graspSensorForce: invalid");
			golem::Assert::valid(graspThresholdForce.isPositive(), ac, "graspThresholdForce: negative");
			golem::Assert::valid(graspEventTimeWait > golem::SEC_TM_REAL_ZERO, ac, "graspEventTimeWait: <= 0");
			golem::Assert::valid(graspCloseDuration > golem::SEC_TM_REAL_ZERO, ac, "graspCloseDuration: <= 0");
			graspPoseOpen.assertValid(golem::Assert::Context(ac, "graspPoseOpen."));
			graspPoseClosed.assertValid(golem::Assert::Context(ac, "graspPoseClosed."));

			golem::Assert::valid(objectCamera.length() > 0, ac, "objectCamera: invalid");
			golem::Assert::valid(objectHandlerScan.length() > 0, ac, "objectHandlerScan: invalid");
			golem::Assert::valid(objectHandler.length() > 0, ac, "objectHandler: invalid");
			golem::Assert::valid(objectItemScan.length() > 0, ac, "objectItemScan: invalid");
			golem::Assert::valid(objectItem.length() > 0, ac, "objectItem: invalid");
			golem::Assert::valid(!objectScanPoseSeq.empty(), ac, "objectScanPoseSeq: empty");
			for (golem::ConfigMat34::Seq::const_iterator i = objectScanPoseSeq.begin(); i != objectScanPoseSeq.end(); ++i)
				i->assertValid(golem::Assert::Context(ac, "objectScanPoseSeq[]."));
			objectFrameAdjustment.assertValid(golem::Assert::Context(ac, "objectFrameAdjustment."));

			golem::Assert::valid(!modelDescMap.empty(), ac, "modelDescMap: empty");
			for (golem::Model::Desc::Map::const_iterator i = modelDescMap.begin(); i != modelDescMap.end(); ++i) {
				golem::Assert::valid(i->second != nullptr, ac, "modelDescMap[]: null");
				i->second->assertValid(golem::Assert::Context(ac, "modelDescMap[]->"));
			}
			contactAppearance.assertValid(golem::Assert::Context(ac, "contactAppearance."));

			golem::Assert::valid(!queryDescMap.empty(), ac, "queryDescMap: empty");
			for (golem::Query::Desc::Map::const_iterator i = queryDescMap.begin(); i != queryDescMap.end(); ++i) {
				golem::Assert::valid(i->second != nullptr, ac, "queryDescMap[]: null");
				i->second->assertValid(golem::Assert::Context(ac, "queryDescMap[]->"));
			}
			golem::Assert::valid(!poseMap.empty(), ac, "poseMap: empty");
			for (PoseDensity::Map::const_iterator i = poseMap.begin(); i != poseMap.end(); ++i) {
				i->second.assertValid(golem::Assert::Context(ac, "poseMap[]->"));
			}

			optimisation.assertValid(golem::Assert::Context(ac, "optimisation."));

			golem::Assert::valid(manipulatorDesc != nullptr, ac, "manipulatorDesc: null");
			manipulatorDesc->assertValid(golem::Assert::Context(ac, "manipulatorDesc->"));
			manipulatorAppearance.assertValid(golem::Assert::Context(ac, "manipulatorAppearance."));
			golem::Assert::valid(manipulatorPoseStdDev.isValid(), ac, "manipulatorPoseStdDev: invalid");
			golem::Assert::valid(manipulatorPoseStdDevMax > golem::REAL_EPS, ac, "manipulatorPoseStdDevMax: < eps");
			golem::Assert::valid(manipulatorItemTrj.length() > 0, ac, "manipulatorItemTrj: invalid");

			golem::Assert::valid(manipulatorTrajectoryDuration > golem::SEC_TM_REAL_ZERO, ac, "manipulatorTrajectoryDuration: <= 0");
			golem::Assert::valid(trajectoryThresholdForce.isPositive(), ac, "trajectoryThresholdForce: negative");

			golem::Assert::valid(withdrawReleaseFraction >= golem::REAL_ZERO, ac, "withdrawReleaseFraction: < 0");
			golem::Assert::valid(withdrawReleaseFraction <= golem::REAL_ONE,  ac, "withdrawReleaseFraction: > 1");
			golem::Assert::valid(withdrawLiftDistance >= golem::REAL_ZERO, ac, "withdrawLiftDistance: < 0");

			//golem::Assert::valid(!clusterMap.empty(), ac, "clusterMap: empty");
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GOLEM_CREATE_FROM_OBJECT_DESC1(AppExPlacement, golem::Object::Ptr, golem::Scene&)
	};

protected:
	/** Data bundle default name */
	std::string dataName;

	/** Model pose estimation camera */
	golem::Camera* modelCamera;
	/** Model data handler */
	golem::data::Handler* modelHandler;
	/** Model data item */
	std::string modelItem;
	/** Model data item object */
	std::string modelItemObj;

	/** Model scan pose */
	golem::ConfigMat34 modelScanPose;
	/** Model colour solid */
	golem::RGBA modelColourSolid;
	/** Model colour wireframe */
	golem::RGBA modelColourWire;
	
	/** Model renderer */
	golem::DebugRenderer modelRenderer;

	/** Model trajectory handler */
	golem::data::Handler* modelHandlerTrj;
	/** Model trajectory item */
	std::string modelItemTrj;

	/** Query pose estimation camera */
	golem::Camera* queryCamera;
	/** Query data handler */
	golem::data::Handler* queryHandler;
	/** Query data item */
	std::string queryItem;
	/** Query data item object */
	std::string queryItemObj;

	/** Query trajectory handler */
	golem::data::Handler* queryHandlerTrj;
	/** Query trajectory item */
	std::string queryItemTrj;

	/** Grasp force sensor */
	golem::FT* graspSensorForce;
	/** Grasp force threshold */
	golem::Twist graspThresholdForce;
	/** Grasp force event - hand close time wait */
	golem::SecTmReal graspEventTimeWait;
	/** Grasp hand close duration */
	golem::SecTmReal graspCloseDuration;
	/** Grasp pose open (pre-grasp) */
	golem::ConfigMat34 graspPoseOpen;
	/** Grasp pose closed (grasp) */
	golem::ConfigMat34 graspPoseClosed;

	/** Object capture camera */
	golem::Camera* objectCamera;
	/** Object data handler (scan) */
	golem::data::Handler* objectHandlerScan;
	/** Object data handler (processed) */
	golem::data::Handler* objectHandler;
	/** Object data item (scan) */
	std::string objectItemScan;
	/** Object data item (processed) */
	std::string objectItem;
	/** Object scan pose */
	golem::ConfigMat34::Seq objectScanPoseSeq;
	/** Object manual frame adjustment */
	golem::RBAdjust objectFrameAdjustment;
	/** Object renderer */
	golem::DebugRenderer objectRenderer;

	/** Models */
	golem::Model::Map modelMap;
	/** Contact appearance */
	golem::Contact3D::Appearance contactAppearance;

	/** Query densities */
	golem::Query::Map queryMap;
	/** Pose descriptions */
	PoseDensity::Map poseMap;

	/** Optimisation description */
	Optimisation optimisation;

	/** Manipulator */
	golem::Manipulator::Ptr manipulator;
	/** Manipulator Appearance */
	golem::Manipulator::Appearance manipulatorAppearance;
	/** Manipulator trajectory item */
	std::string manipulatorItemTrj;
	/** Manipulator pose distribution covariance */
	golem::RBDist poseCov, poseCovInv;
	/** Maximum distance between frames in squared standard deviations */
	golem::Real poseDistanceMax;

	/** Manipulation trajectory duration */
	golem::SecTmReal manipulatorTrajectoryDuration;

	/** Manipulation trajectory force threshold */
	golem::Twist trajectoryThresholdForce;

	/* Withdraw action hand release fraction */
	golem::Real withdrawReleaseFraction;
	/* Withdraw action lift distance */
	golem::Real withdrawLiftDistance;

	/** Cluster map */
	Data::Cluster::Map clusterMap;

	/** Item selection */
	typedef std::function<void(Data::Training::Map&, Data::Training::Map::iterator&)> ItemSelectFunc;
	typedef std::function<void(ItemSelectFunc)> ItemSelect;
	ItemSelect itemSelect;

	/** Access to Manager::menuCmdMap by unfriendly objects */
	Menu::MenuCmdMap& getMenuCmdMap() {
		return Manager::menuCmdMap;
	};

	/** golem::UIRenderer interface */
	virtual void render() const;

	/** Pose estimation */
	golem::data::Item::Map::iterator estimatePose(Data::Mode mode);
	/** Grasp and capture object */
	golem::data::Item::Map::iterator objectGraspAndCapture(const bool stopAtBreakPoint = false);
	/** Process object image and add to data bundle */
	golem::data::Item::Map::iterator objectProcess(golem::data::Item::Map::iterator ptr);
	/** Create trajectory name */
	std::string getTrajectoryName(const std::string& prefix, const std::string& type) const;

	/** Create query densities */
	void createQuery(golem::data::Item::Ptr item, const golem::Mat34& frame, const Data::Cluster::Counter* clusterCounter = nullptr);

	/** Generate solutions */
	void generateSolutions();
	/** Sort solutions */
	void sortSolutions(Data::Solution::Seq& seq) const;

	/** Select trajectory */
	void selectTrajectory();

	/** Perform trajectory */
	void performTrajectory(bool testTrajectory);

	/** Evaluation */
	template <typename _Ptr> inline static golem::Real evaluateSample(_Ptr begin, _Ptr end, const golem::RBCoord& coord) {
		golem::Real likelihood = golem::numeric_const<golem::Real>::ZERO, c = golem::numeric_const<golem::Real>::ZERO;
		for (_Ptr kernel = begin; kernel < end; ++kernel) {
			const golem::Real dlin = kernel->p.distanceSqr(coord.p);
			if (dlin < kernel->distMax.lin) {
				const golem::Real dang = kernel->q.distance(coord.q);
				if (dang < kernel->distMax.ang) {
					const golem::Real distance = kernel->covInv.lin*dlin + kernel->covInv.ang*dang;
					const golem::Real sampleLikelihood = kernel->weight*golem::Math::exp(-golem::Real(distance));
					golem::kahanSum(likelihood, c, sampleLikelihood);
				}
			}
		}
		return likelihood;
	}

	golem::Camera* getWristCamera(const bool dontThrow = false) const;
	golem::Mat34 getWristPose() const;
	golem::Controller::State::Seq getTrajectoryFromPose(const golem::Mat34& w, const golem::SecTmReal duration);
	golem::ConfigMat34 getConfigFromPose(const golem::Mat34& w);
	golem::Controller::State lookupStateArmCommandHand() const;

	void setHandConfig(golem::Controller::State::Seq& trajectory, const golem::ConfigMat34& handPose);
	void gotoWristPose(const golem::Mat34& w, const golem::SecTmReal duration = golem::SEC_TM_REAL_ZERO);
	void gotoPose2(const golem::ConfigMat34& pose, const golem::SecTmReal duration);
	void releaseHand(const double openFraction, const golem::SecTmReal duration);
	void closeHand(const double closeFraction, const golem::SecTmReal duration);
	void liftWrist(const double verticalDistance, const golem::SecTmReal duration);
	void haltRobot();

	void nudgeWrist();
	void rotateObjectInHand();

	void create(const Desc& desc);

	AppExPlacement(golem::Scene &scene);
	~AppExPlacement();
};

//------------------------------------------------------------------------------

void XMLData(golem::AppExPlacement::Data::Cluster::Map::value_type& val, golem::XMLContext* context, bool create = false);

void XMLData(golem::AppExPlacement::PoseDensity::Map::value_type& val, golem::XMLContext* context, bool create = false);

template <> void Stream::read(golem::AppExPlacement::Data::Training::Map::value_type& value) const;
template <> void Stream::write(const golem::AppExPlacement::Data::Training::Map::value_type& value);

template <> void Stream::read(golem::AppExPlacement::Data::Density::Seq::value_type& value) const;
template <> void Stream::write(const golem::AppExPlacement::Data::Density::Seq::value_type& value);

template <> void Stream::read(golem::AppExPlacement::Data::Solution::Seq::value_type& value) const;
template <> void Stream::write(const golem::AppExPlacement::Data::Solution::Seq::value_type& value);

template <> void Stream::read(golem::Manipulator::Waypoint& value) const;
template <> void Stream::write(const golem::Manipulator::Waypoint& value);

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif // _GOLEM_APP_PLACEMENT_PLACEMENT_H_
