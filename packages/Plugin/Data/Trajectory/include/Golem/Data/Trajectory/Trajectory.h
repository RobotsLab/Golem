/** @file Trajectory.h
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
#ifndef _GOLEM_DATA_TRAJECTORY_TRAJECTORY_H_
#define _GOLEM_DATA_TRAJECTORY_TRAJECTORY_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Library.h>
#include <Golem/Plugin/Renderer.h>
#include <Golem/App/GraphRenderer.h>
#include <Golem/Planner/Profile.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Plugin/Ctrl.h>
#include <Golem/Plugin/DataI.h>
#include <Golem/Plugin/PlannerI.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

class ItemTrajectory;
class HandlerTrajectory;

/** Data item representing trajectory.
*/
class GOLEM_LIBRARY_DECLDIR ItemTrajectory : public Item, public Trajectory {
public:
	friend class HandlerTrajectory;

	/** Waypoints collection */
	golem::WaypointCtrl::Seq waypoints;

	/** Waypoints file */
	mutable File waypointFile;

	/** Path position */
	golem::Real pathPosition;
	
	/** Path waypoint */
	size_t pathWaypoint;
	/** Path interpolation */
	golem::Real pathInterpol;

	/** Path waypoint at contact */
	size_t contactPathWaypoint;
	/** Path interpolation at contact */
	golem::Real contactPathInterpol;

	/** Manifold */
	ManifoldCtrl manifoldCtrl;

	/** Clones item. */
	virtual Item::Ptr clone() const;

	/** Creates render buffer, the buffer can be shared and allocated on Handler */
	virtual void createRender();

	/** Trajectory: Sets waypoint collection with no velocity profile. */
	virtual void setWaypoints(const golem::WaypointCtrl::Seq& waypoints);
	/** Trajectory: Returns waypoints without velocity profile. */
	virtual const golem::WaypointCtrl::Seq& getWaypoints() const;

	/** Trajectory: Sets manifold. */
	virtual void setManifold(const ManifoldCtrl& manifold);
	/** Trajectory: Returns manifold. */
	virtual const ManifoldCtrl& getManifold() const;

	/** Trajectory: Profile set. */
	virtual StringSeq getProfiles() const;
	/** Trajectory: Profile set. */
	virtual void setProfile(const std::string& profile);
	/** Trajectory: Returns waypoints with velocity profile. */
	virtual void createTrajectory(golem::Controller::State::Seq& trajectory);

protected:
	/** Data handler */
	HandlerTrajectory& handler;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	/** Initialise data item */
	ItemTrajectory(HandlerTrajectory& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerTrajectory : public data::Handler, public UI, public HandlerPlanner, public Import, public golem::Profile::CallbackDist, public virtual StreamHandlerConfigspace, public virtual StreamHandlerReservedspace {
public:
	friend class ItemTrajectory;

	/** Bounds of kinematic chains */
	typedef golem::Chainspace::Coord<golem::Bounds::Seq> ChainBoundsSeq;
	/** Bounds poses of kinematic chains */
	typedef golem::Chainspace::Coord<Mat34Seq> ChainMat34Seq;
	/** Bounds of kinematic joints */
	typedef golem::Configspace::Coord<golem::Bounds::Seq> JointBoundsSeq;
	/** Bounds poses of kinematic joints */
	typedef golem::Configspace::Coord<Mat34Seq> JointMat34Seq;

	/** Import state */
	class GOLEM_LIBRARY_DECLDIR ImportState {
	public:
		/** Map */
		typedef std::vector<ImportState> Map;

		/** State variable pointer type */
		enum Type {
			/** Time stamp (Real) */
			TYPE_TIME = 0,
			/** Position (Configspace, Real) */
			TYPE_POSITION,
			/** Velocity (Configspace, Real) */
			TYPE_VELOCITY,
			/** Acceleration (Configspace, Real) */
			TYPE_ACCELERATION,
			/** Reserved area (Real) */
			TYPE_RESERVED,
		};

		/** State variable pointer type */
		Type type;
		/** Input pointer */
		golem::U32 inp;
		/** Output pointer */
		golem::U32 out;
		/** Offset */
		golem::Real offset;
		/** Scale */
		golem::Real scale;

		/** Input pointer */
		std::string inpStr;
		/** Output pointer */
		std::string outStr;

		/** Set to default */
		ImportState() {
			setToDefault();
		}
		/** Custom create */
		ImportState(Type type, golem::U32 inp, golem::U32 out, golem::Real offset, golem::Real scale) : type(type), inp(inp), out(out), offset(offset), scale(scale) {
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			type = TYPE_POSITION;
			inp = 0;
			out = 0;
			offset = golem::REAL_ZERO;
			scale = golem::REAL_ONE;
			inpStr.clear();
			outStr.clear();
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(type == TYPE_RESERVED ? out < (golem::U32)golem::Reservedspace::DIM : out < (golem::U32)golem::Configspace::DIM, ac, "out: not in range");
			Assert::valid(golem::Math::isFinite(offset), ac, "offset: invalid");
			Assert::valid(golem::Math::isFinite(scale), ac, "scale: invalid");
		}
		/** Assert that the description is valid. */
		static void assertValid(const Assert::Context& ac, const Map& map) {
			Assert::valid(!map.empty(), ac, ": empty");
			for (ImportState::Map::const_iterator i = map.begin(); i != map.end(); ++i)
				i->assertValid(Assert::Context(ac, "[i]."));
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);

		/** Update controller state */
		void update(const golem::RealSeq& data, golem::Controller::State& state) const;
		/** Update controller state */
		static void update(const Map& map, const golem::RealSeq& data, golem::Controller::State& state);

		/** Extract rule */
		static void extract(const std::string& str, U32Seq& seq);
		/** Extract inp and out rules */
		void extract(Map& map) const;
		/** Extract inp and out rules */
		static void extract(const Map& inp, Map& out);
	};

	/** Import rigid body pose/velocity/acceleration */
	class GOLEM_LIBRARY_DECLDIR ImportFrame {
	public:
		/** Variable type */
		enum Type {
			/** Quaternion */
			TYPE_QUAT = 0,
			/** Euler angles */
			TYPE_EULER,
			/** Axis */
			TYPE_AXIS,
		};

		/** Variable type */
		Type type;
		/** Linear variable pointer */
		golem::U32 lin;
		/** Angular variable pointer */
		golem::U32 ang;

		/** Set to default */
		ImportFrame() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			type = TYPE_QUAT;
			lin = 0;
			ang = 3;
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);

		/** Update frame */
		void update(const golem::RealSeq& data, golem::Mat34& trn) const;
	};

	/** Arm and hand setup */
	class GOLEM_LIBRARY_DECLDIR FactorDesc {
	public:
		/** Arm */
		golem::Real arm;
		/** Hand */
		golem::Real hand;
		/** Other controllers */
		golem::Real other;

		/** Coordinates */
		golem::ConfigspaceCoord coord;

		/** Set to default */
		FactorDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			arm = golem::REAL_ONE;
			hand = golem::REAL_ONE;
			other = golem::REAL_ONE;
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(golem::Math::isFinite(arm), ac, "arm: invalid");
			Assert::valid(golem::Math::isFinite(hand), ac, "hand: invalid");
			Assert::valid(golem::Math::isFinite(other), ac, "other: invalid");
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Trajectory import */
	class GOLEM_LIBRARY_DECLDIR ImportRobotTrjDesc {
	public:
		/** Time interval */
		golem::Real interval;
		/** Begin waypoint */
		golem::U32 begin;
		/** End waypoint */
		golem::U32 end;
		/** Subsampling interval */
		golem::U32 subsampling;

		/** State map */
		ImportState::Map stateMap;
		/** Command map */
		ImportState::Map commandMap;

		/** Set to default */
		ImportRobotTrjDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			interval = golem::REAL_ONE;
			subsampling = 1;
			begin = 0;
			end = golem::U32(-1);
			stateMap.clear();
			commandMap.clear();
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(interval > golem::REAL_EPS, ac, "interval: < eps");
			Assert::valid(begin < end, ac, ": begin >= end");
			Assert::valid(subsampling > 0, ac, "subsampling: = 0");
			ImportState::assertValid(Assert::Context(ac, "stateMap"), stateMap);
			ImportState::assertValid(Assert::Context(ac, "commandMap"), commandMap);
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Trajectory import */
	class GOLEM_LIBRARY_DECLDIR ProfileDesc {
	public:
		typedef std::map<std::string, ProfileDesc> Map;

		/** Default name */
		static std::string getDefaultName();

		/** Trajectory profile description */
		golem::Profile::Desc::Ptr profileDesc;

		/** Velocity multiplication factor */
		FactorDesc velFac;
		/** Acceleration multiplication factor */
		FactorDesc accFac;
		/** Distance multiplication factor */
		FactorDesc disFac;
		/** Extrapolation multiplication factor */
		FactorDesc extFac;
		/** Command multiplication factor */
		FactorDesc cmdFac;

		/** Trajectory profile configspace distance multiplier */
		RealSeq distance;
		/** Trajectory extrapolation */
		RealSeq extrapolation;
		/** Trajectory motor command */
		RealSeq command;

		/** Trajectory extrapolation */
		golem::Real trjExtrapolation;
		/** Trajectory extrapolation waypoint index (from the back) */
		golem::U32 trjExtrapolationIndex;
		/** Trajectory duration */
		golem::Real trjDuration;
		/** Trajectory idle */
		golem::Real trjIdle;

		/** Action */
		Mat34Seq action;

		/** Set to default */
		ProfileDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			profileDesc.reset(new golem::Profile::Desc);
			distance.assign(golem::Configspace::DIM, golem::REAL_ONE);
			extrapolation.assign(golem::Configspace::DIM, golem::REAL_ONE);
			command.assign(golem::Configspace::DIM, golem::REAL_ONE);

			velFac.setToDefault();
			accFac.setToDefault();
			disFac.setToDefault();
			extFac.setToDefault();
			cmdFac.setToDefault();

			trjExtrapolation = golem::Real(0.0);
			trjExtrapolationIndex = 0;
			trjDuration = golem::Real(5.0);
			trjIdle = golem::Real(1.0);

			action.clear();
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(profileDesc != nullptr && profileDesc->isValid(), ac, "profileDesc: invalid");
			Assert::valid(!distance.empty(), ac, "distance: invalid");
			for (RealSeq::const_iterator i = distance.begin(); i != distance.end(); ++i)
				Assert::valid(*i >= golem::REAL_ZERO, ac, "distance[i] < 0");
			Assert::valid(!extrapolation.empty(), ac, "extrapolation: invalid");
			for (RealSeq::const_iterator i = extrapolation.begin(); i != extrapolation.end(); ++i)
				Assert::valid(*i >= golem::REAL_ZERO, ac, "extrapolation[i] < 0");
			Assert::valid(!command.empty(), ac, "command: invalid");
			for (RealSeq::const_iterator i = command.begin(); i != command.end(); ++i)
				Assert::valid(*i >= golem::REAL_ZERO, ac, "command[i] < 0");

			velFac.assertValid(Assert::Context(ac, "velFac."));
			accFac.assertValid(Assert::Context(ac, "accFac."));
			disFac.assertValid(Assert::Context(ac, "disFac."));
			extFac.assertValid(Assert::Context(ac, "extFac."));
			cmdFac.assertValid(Assert::Context(ac, "cmdFac."));

			Assert::valid(trjExtrapolation >= golem::REAL_ZERO, ac, "trjExtrapolation < 0");
			Assert::valid(trjDuration > golem::REAL_EPS, ac, "trjDuration < eps");
			Assert::valid(trjIdle >= golem::REAL_ZERO, ac, "trjIdle < 0");

			for (Mat34Seq::const_iterator i = action.begin(); i != action.end(); ++i)
				Assert::valid(i->isValid(), ac, "action[i]: invalid");
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public data::Handler::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Waypoint suffix */
		std::string waypointSuffix;

		/** Planner index */
		golem::U32 plannerIndex;

		/** Trajectory profiles */
		ProfileDesc::Map profileDescMap;

		/** Bounds solid colour */
		golem::RGBA boundsSolidColour;
		/** Path renderer */
		golem::GraphRenderer pathRenderer;
		/** Path increment */
		golem::Real pathInc;

		/** Show commands/states */
		bool showCommands;

		/** Robot trajectory import */
		ImportRobotTrjDesc importRobotTrj;
		/** Import from HDF5 dump file: file extension */
		std::string importHDF5FileExt;
		/** Import from HDF5 dump file: robot trajectory dataset */
		std::string importHDF5RobotTrj;

		/** Set to default */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			golem::data::Handler::Desc::setToDefault();

			waypointSuffix = getFileExtWaypoint();

			plannerIndex = 0;

			profileDescMap.insert(std::make_pair(ProfileDesc::getDefaultName(), ProfileDesc()));

			boundsSolidColour = golem::RGBA(192, 192, 0, 100);
			pathRenderer.setToDefault();
			pathRenderer.edgeShow = true;
			pathRenderer.show = true;
			pathInc = golem::Real(0.05);

			showCommands = false;

			importRobotTrj.setToDefault();
			importHDF5FileExt = ".hdf5dump";
			importHDF5RobotTrj = "RobotTrajectory";
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			golem::data::Handler::Desc::assertValid(ac);

			Assert::valid(waypointSuffix.length() > 0, ac, "waypointSuffix: empty");

			Assert::valid(!profileDescMap.empty(), ac, "profileDescMap: empty");
			Assert::valid(profileDescMap.find(ProfileDesc::getDefaultName()) != profileDescMap.end(), ac, "profileDescMap: unable to find profile \"%s\"", ProfileDesc::getDefaultName().c_str());
			for (ProfileDesc::Map::const_iterator i = profileDescMap.begin(); i != profileDescMap.end(); ++i)
				i->second.assertValid(Assert::Context(ac, std::string("importRobotTrj[" + i->first + "]").c_str()));

			Assert::valid(pathRenderer.isValid(), ac, "pathRenderer: invalid");
			Assert::valid(pathInc > golem::REAL_EPS && pathInc <= golem::REAL_ONE, ac, "pathInc < eps or pathInc > 1");

			importRobotTrj.assertValid(Assert::Context(ac, "importRobotTrj."));
			Assert::valid(!importHDF5FileExt.empty(), ac, "importHDF5FileExt: invalid");
			Assert::valid(!importHDF5RobotTrj.empty(), ac, "importHDF5RobotTrj: invalid");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual data::Handler::Ptr create(golem::Context &context) const;
	};

	/** File extension: waypoint */
	static std::string getFileExtWaypoint();
	
protected:
	/** distRemoved callback */
	typedef std::function<void(size_t)> DistRemovedCallback;

	/** Planner index */
	golem::U32 plannerIndex;

	/** Rendering */
	mutable golem::BoundsRenderer boundsRenderer;
	/** Bounds solid colour */
	golem::RGBA boundsSolidColour;
	/** Path renderer */
	golem::GraphRenderer pathRenderer;
	/** Path increment */
	golem::Real pathInc;
	/** Path increment */
	golem::U32 pathIncStep;

	/** Bounds of kinematic chains */
	ChainBoundsSeq chainBoundsSeq;
	/** Bounds poses of kinematic chains */
	ChainMat34Seq chainMat34Seq;
	/** Bounds of kinematic joints */
	JointBoundsSeq jointBoundsSeq;
	/** Bounds poses of kinematic joints */
	JointMat34Seq jointMat34Seq;
	/** All bounds */
	golem::Bounds::ConstSeq boundsSeq;
	/** Show bounds */
	bool boundsShow;
	/** Path position increment */
	golem::Real pathPositionInc;
	/** Path request at contact */
	bool contactPathRequest;

	/** waypoint suffix */
	std::string waypointSuffix;

	/** Planner */
	const golem::Planner* planner;
	/** Controller */
	const golem::Controller* controller;
	/** Controller state info */
	golem::Controller::State::Info armInfo;
	/** Controller state info */
	golem::Controller::State::Info handInfo;
	/** Controller state info */
	golem::Controller::State::Info info;
	/** Controller state info */
	golem::Controller::State::Ptr defaultState;

	/** Trajectory profiles */
	ProfileDesc::Map profileDescMap;

	/** Trajectory profiles */
	ProfileDesc::Map::value_type* pProfileDesc;
	/** Trajectory profile */
	golem::Profile::Ptr pProfile;

	/** Trajectory profile configspace distance multiplier */
	golem::ConfigspaceCoord distance;
	/** Trajectory extrapolation */
	golem::ConfigspaceCoord extrapolation;
	/** Trajectory motor command */
	golem::ConfigspaceCoord command;

	/** distRemoved callback */
	DistRemovedCallback distRemovedCallback;

	/** Import types */
	StringSeq importTypes;

	/** Show commands/states */
	bool showCommands;

	/** Robot trajectory import */
	ImportRobotTrjDesc importRobotTrj;
	/** Import from HDF5 dump file: file extension */
	std::string importHDF5FileExt;
	/** Import from HDF5 dump file: robot trajectory dataset */
	std::string importHDF5RobotTrj;

	/** Creates render buffer */
	void createRender(const ItemTrajectory& item);
	/** golem::UIRenderer: Render on output device. */
	virtual void render() const;
	/** golem::UIRenderer: Render on output device. */
	virtual void customRender() const;

	/** golem::UIRenderer: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	/** golem::UIRenderer: Mouse motion handler. */
	virtual void motionHandler(int x, int y);
	/** golem::UIRenderer: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);

	/** Construct empty item, accessible only by Data. */
	virtual Item::Ptr create() const;

	/** golem::Profile::CallbackDist: Configuration space coordinate distance metric */
	virtual golem::Real distConfigspaceCoord(const golem::ConfigspaceCoord& prev, const golem::ConfigspaceCoord& next) const;
	/** golem::Profile::CallbackDist: Coordinate distance metric */
	virtual golem::Real distCoord(golem::Real prev, golem::Real next) const;
	/** golem::Profile::CallbackDist: Prune state sequence state */
	virtual void distRemoved(size_t index) const;

	/** golem::Profile::CallbackDist: Coordinate enabled state */
	virtual bool distCoordPlanning(const golem::Configspace::Index& index) const;
	/** golem::Profile::CallbackDist: Coordinate interpolate state */
	virtual bool distCoordInterpolation(const golem::Configspace::Index& index) const;

	/** Creates trajectory from state sequence */
	virtual void create(U32 index, const golem::ConfigspaceCoord& delta, golem::Controller::State::Seq& trajectory) const;

	/** Profile state sequence */
	virtual void profile(golem::SecTmReal duration, golem::Controller::State::Seq& trajectory) const;

	/** HandlerPlanner: Planner index. */
	virtual golem::U32 getPlannerIndex() const;
	/** HandlerPlanner: Sets planner and controllers. */
	virtual void set(const golem::Planner& planner, const ControllerId::Seq& controllerIDSeq);

	/** Trajectory: Profile set. */
	virtual StringSeq getProfiles() const;
	/** Trajectory: Profile set. */
	virtual void setProfile(const std::string& profile);
	/** Trajectory: Returns waypoints with velocity profile. */
	virtual void createTrajectory(const ItemTrajectory& item, golem::Controller::State::Seq& trajectory);

	/** Import: Import from file */
	virtual Item::Ptr import(const std::string& path);
	/** Import: Available file types */
	virtual const StringSeq& getImportFileTypes() const;

	/** StreamHandlerConfigspace: handle Configspace */
	virtual void convert(const golem::Configspace::Range& inpConfigspaceRange, const golem::Configspace::Range& outConfigspaceRange, const golem::Chainspace::Range& inpChainspaceRange, const golem::Chainspace::Range& outChainspaceRange, golem::ConfigspaceCoord& val);
	/** StreamHandlerReservedspace: handle Reservedspace */
	virtual void convert(const golem::Reservedspace::Range& inpRange, const golem::Reservedspace::Range& outRange, golem::ReservedCoord& val);

	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerTrajectory(golem::Context &context);
};

/** Reads/writes object from/to a given XML context */
void XMLData(data::HandlerTrajectory::ImportState::Map::value_type& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(const std::string &attr, data::HandlerTrajectory::ImportState::Type& val, golem::XMLContext* xmlcontext, bool create = false);

/** Reads/writes object from/to a given XML context */
void XMLData(const std::string &attr, data::HandlerTrajectory::ImportFrame::Type& val, golem::XMLContext* xmlcontext, bool create = false);

/** Reads/writes object from/to a given XML context */
void XMLData(data::HandlerTrajectory::ProfileDesc::Map::value_type &val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_DATA_TRAJECTORY_TRAJECTORY_H_*/
