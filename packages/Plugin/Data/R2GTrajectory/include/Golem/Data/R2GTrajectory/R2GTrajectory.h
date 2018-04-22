//------------------------------------------------------------------------------
//               Simultaneous Perception And Manipulation (SPAM)
//------------------------------------------------------------------------------
//
// Copyright (c) 2010 University of Birmingham.
// All rights reserved.
//
// Intelligence Robot Lab.
// School of Computer Science
// University of Birmingham
// Edgbaston, Brimingham. UK.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy 
// of this software and associated documentation files (the "Software"), to deal 
// with the Software without restriction, including without limitation the 
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
// sell copies of the Software, and to permit persons to whom the Software is 
// furnished to do so, subject to the following conditions:
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimers.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimers in 
//       the documentation and/or other materials provided with the 
//       distribution.
//     * Neither the names of the Simultaneous Perception And Manipulation 
//		 (SPAM), University of Birmingham, nor the names of its contributors  
//       may be used to endorse or promote products derived from this Software 
//       without specific prior written permission.
//
// The software is provided "as is", without warranty of any kind, express or 
// implied, including but not limited to the warranties of merchantability, 
// fitness for a particular purpose and noninfringement.  In no event shall the 
// contributors or copyright holders be liable for any claim, damages or other 
// liability, whether in an action of contract, tort or otherwise, arising 
// from, out of or in connection with the software or the use of other dealings 
// with the software.
//
//------------------------------------------------------------------------------
//! @Author:   Claudio Zito
//! @Date:     25/03/2014
//------------------------------------------------------------------------------
#pragma once
#ifndef _GOLEM_DATA_R2GTRAJECTORY_H_
#define _GOLEM_DATA_R2GTRAJECTORY_H_

//------------------------------------------------------------------------------

#include <Golem/HBPlanner/Data.h>
#include <Golem/Plugin/Library.h>
#include <Golem/Plugin/Renderer.h>
#include <Golem/App/GraphRenderer.h>
#include <Golem/Planner/Profile.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Plugin/Ctrl.h>
#include <Golem/Plugin/DataI.h>
#include <Golem/Plugin/PlannerI.h>
#include <Golem/HBPlanner/Heuristic.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

class ItemR2GTrajectory;
class HandlerR2GTrajectory;

/** Data item representing trajectory.
*/
class GOLEM_LIBRARY_DECLDIR ItemR2GTrajectory : public Item, public R2GTrajectory {
public:
	friend class HandlerR2GTrajectory;

	typedef std::map<R2GTrajectory::Type, WaypointCtrl::Seq> WaypointMap;

	/** Waypoints collection */
	WaypointCtrl::Seq waypoints;

	/** Waypoints collection map */
	WaypointMap waypointMap;

	/** Waypoints file */
	mutable File waypointFile;

	/** Path position */
	Real pathPosition;

	/** Path waypoint */
	size_t pathWaypoint;
	/** Path interpolation */
	Real pathInterpol;

	/** Path waypoint at contact */
	size_t contactPathWaypoint;
	/** Path interpolation at contact */
	Real contactPathInterpol;

	/** Clones item. */
	virtual Item::Ptr clone() const;

	/** Creates render buffer, the buffer can be shared and allocated on Handler */
	virtual void createRender();

	/** Trajectory: Sets waypoint collection with no velocity profile. */
	virtual void setWaypoints(const WaypointCtrl::Seq& waypoints);
	/** Trajectory: Sets waypoint collection with no velocity profile. */
	virtual void setWaypoints(const WaypointCtrl::Seq& waypoints, const R2GTrajectory::Type type);

	/** Trajectory: Returns waypoints without velocity profile. */
	virtual const WaypointCtrl::Seq& getWaypoints() const;
	/** Trajectory: Returns waypoints without velocity profile. */
	virtual const WaypointCtrl::Seq& getWaypoints(const R2GTrajectory::Type type) const;

	/** Trajectory: Returns waypoints with velocity profile. */
	virtual void createTrajectory(Controller::State::Seq& trajectory);
	/** Trajectory: Compute grasping and lifting trajectory: Returns waypoints with velocity profile. */
	virtual void createActionTrajectory(const Controller::State& begin, Controller::State::Seq& trajectory);
	/** (Mycroft) Compute approaching trajectory: Returns waypoints with velocity profile. */
	virtual void createTrajectory(const Controller::State& begin, Controller::State::Seq& trajectory);
	/** (IR3ne) Compute approaching trajectory: Returns waypoints with velocity profile. */
	virtual void createIGTrajectory(const Controller::State& begin, Controller::State::Seq& trajectory);

protected:
	/** Data handler */
	HandlerR2GTrajectory& handler;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, XMLContext* xmlcontext) const;

	/** Initialise data item */
	ItemR2GTrajectory(HandlerR2GTrajectory& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerR2GTrajectory : public Handler, public UI, public HandlerR2GPlan, public Import, public Transform, public Profile::CallbackDist, public virtual StreamHandlerConfigspace, public virtual StreamHandlerReservedspace {
public:
	friend class ItemR2GTrajectory;

	/** Bounds of kinematic chains */
	typedef Chainspace::Coord<Bounds::Seq> ChainBoundsSeq;
	/** Bounds poses of kinematic chains */
	typedef Chainspace::Coord<Mat34Seq> ChainMat34Seq;
	/** Bounds of kinematic joints */
	typedef Configspace::Coord<Bounds::Seq> JointBoundsSeq;
	/** Bounds poses of kinematic joints */
	typedef Configspace::Coord<Mat34Seq> JointMat34Seq;

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
		U32 inp;
		/** Output pointer */
		U32 out;
		/** Offset */
		Real offset;
		/** Scale */
		Real scale;

		/** Input pointer */
		std::string inpStr;
		/** Output pointer */
		std::string outStr;

		/** Set to default */
		ImportState() {
			setToDefault();
		}
		/** Custom create */
		ImportState(Type type, U32 inp, U32 out, Real offset, Real scale) : type(type), inp(inp), out(out), offset(offset), scale(scale) {
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			type = TYPE_POSITION;
			inp = 0;
			out = 0;
			offset = REAL_ZERO;
			scale = REAL_ONE;
			inpStr.clear();
			outStr.clear();
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(type == TYPE_RESERVED ? out < (U32)Reservedspace::DIM : out < (U32)Configspace::DIM, ac, "out: not in range");
			Assert::valid(Math::isFinite(offset), ac, "offset: invalid");
			Assert::valid(Math::isFinite(scale), ac, "scale: invalid");
		}
		/** Assert that the description is valid. */
		static void assertValid(const Assert::Context& ac, const Map& map) {
			Assert::valid(!map.empty(), ac, ": empty");
			for (ImportState::Map::const_iterator i = map.begin(); i != map.end(); ++i)
				i->assertValid(Assert::Context(ac, "[i]."));
		}
		/** Load descritpion from xml context. */
		void load(const XMLContext* xmlcontext);

		/** Update controller state */
		void update(const RealSeq& data, Controller::State& state) const;
		/** Update controller state */
		static void update(const Map& map, const RealSeq& data, Controller::State& state);

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
		U32 lin;
		/** Angular variable pointer */
		U32 ang;

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
		void load(const XMLContext* xmlcontext);

		/** Update frame */
		void update(const RealSeq& data, Mat34& trn) const;
	};

	/** Arm and hand setup */
	class GOLEM_LIBRARY_DECLDIR FactorDesc {
	public:
		/** Arm */
		Real arm;
		/** Hand */
		Real hand;
		/** Other controllers */
		Real other;

		/** Set to default */
		FactorDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			arm = REAL_ONE;
			hand = REAL_ONE;
			other = REAL_ONE;
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(Math::isFinite(arm), ac, "arm: invalid");
			Assert::valid(Math::isFinite(hand), ac, "hand: invalid");
			Assert::valid(Math::isFinite(other), ac, "other: invalid");
		}
		/** Load descritpion from xml context. */
		void load(Context& context, const XMLContext* xmlcontext);
	};

	/** Trajectory import */
	class GOLEM_LIBRARY_DECLDIR ImportRobotTrjDesc {
	public:
		/** Time interval */
		Real interval;
		/** Begin waypoint */
		U32 begin;
		/** End waypoint */
		U32 end;
		/** Subsampling interval */
		U32 subsampling;

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
			interval = REAL_ONE;
			subsampling = 1;
			begin = 0;
			end = U32(-1);
			stateMap.clear();
			commandMap.clear();
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(interval > REAL_EPS, ac, "interval: < eps");
			Assert::valid(begin < end, ac, ": begin >= end");
			Assert::valid(subsampling > 0, ac, "subsampling: = 0");
			//ImportState::assertValid(Assert::Context(ac, "stateMap"), stateMap);
			//ImportState::assertValid(Assert::Context(ac, "commandMap"), commandMap);
		}
		/** Load descritpion from xml context. */
		void load(const XMLContext* xmlcontext);
	};

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public data::Handler::Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Waypoint suffix */
		std::string waypointSuffix;

		/** Planner index */
		U32 plannerIndex;

		/** Trajectory profile description */
		Profile::Desc::Ptr profileDesc;
		/** Trajectory profile configspace distance multiplier */
		RealSeq distance;
		/** Trajectory extrapolation */
		RealSeq extrapolation;
		/** Trajectory motor command */
		RealSeq command;

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

		/** Trajectory extrapolation */
		Real trjExtrapolation;
		/** Trajectory duration */
		Real trjDuration;
		/** IG Trajectory duration */
		Real trjR2GDuration;
		/** Trajectory idle */
		Real trjIdle;

		/** Action */
		Mat34Seq action;

		/** Bounds solid colour */
		RGBA boundsSolidColour;
		/** Path renderer */
		GraphRenderer pathRenderer;
		/** Path increment */
		Real pathIncLarge;
		/** Path increment */
		Real pathIncSmall;

		/** Show commands/states */
		bool showCommands;

		/** Robot trajectory import */
		ImportRobotTrjDesc importRobotTrj;
		/** Import from HDF5 dump file: file extension */
		std::string importHDF5FileExt;
		/** Import from HDF5 dump file: robot trajectory dataset */
		std::string importHDF5RobotTrj;

		/** Pregrasp hand pose */
		RealSeq handPregraspPose;
		/** Grasp hand pose */
		RealSeq handGraspPose;

		/** Set to default */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			data::Handler::Desc::setToDefault();

			waypointSuffix = getFileExtWaypoint();

			plannerIndex = 0;

			profileDesc.reset(new Profile::Desc);
			distance.assign(Configspace::DIM, REAL_ONE);
			extrapolation.assign(Configspace::DIM, REAL_ONE);
			command.assign(Configspace::DIM, REAL_ONE);

			velFac.setToDefault();
			accFac.setToDefault();
			disFac.setToDefault();
			extFac.setToDefault();
			cmdFac.setToDefault();

			trjExtrapolation = Real(0.0);
			trjDuration = Real(5.0);
			trjR2GDuration = Real(20.0);
			trjIdle = Real(1.0);

			action.clear();

			boundsSolidColour = RGBA(192, 192, 0, 100);
			pathRenderer.setToDefault();
			pathRenderer.edgeShow = true;
			pathRenderer.show = true;
			pathIncLarge = Real(0.05);
			pathIncSmall = Real(0.005);

			showCommands = false;

			importRobotTrj.setToDefault();
			importHDF5FileExt = ".hdf5dump";
			importHDF5RobotTrj = "RobotTrajectory";

			handPregraspPose.assign(20, REAL_ZERO);
			handGraspPose.assign(20, REAL_ONE);
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			data::Handler::Desc::assertValid(ac);

			Assert::valid(waypointSuffix.length() > 0, ac, "waypointSuffix: empty");

			Assert::valid(profileDesc != nullptr && profileDesc->isValid(), ac, "profileDesc: invalid");
			Assert::valid(!distance.empty(), ac, "distance: invalid");
			for (RealSeq::const_iterator i = distance.begin(); i != distance.end(); ++i)
				Assert::valid(*i >= REAL_ZERO, ac, "distance[i] < 0");
			Assert::valid(!extrapolation.empty(), ac, "extrapolation: invalid");
			for (RealSeq::const_iterator i = extrapolation.begin(); i != extrapolation.end(); ++i)
				Assert::valid(*i >= REAL_ZERO, ac, "extrapolation[i] < 0");
			Assert::valid(!command.empty(), ac, "command: invalid");
			for (RealSeq::const_iterator i = command.begin(); i != command.end(); ++i)
				Assert::valid(*i >= REAL_ZERO, ac, "command[i] < 0");

			velFac.assertValid(Assert::Context(ac, "velFac."));
			accFac.assertValid(Assert::Context(ac, "accFac."));
			disFac.assertValid(Assert::Context(ac, "disFac."));
			extFac.assertValid(Assert::Context(ac, "extFac."));
			cmdFac.assertValid(Assert::Context(ac, "cmdFac."));

			Assert::valid(trjExtrapolation >= REAL_ZERO, ac, "trjExtrapolation < 0");
			Assert::valid(trjDuration > REAL_EPS, ac, "trjDuration < eps");
			Assert::valid(trjR2GDuration > REAL_EPS, ac, "trjDuration < eps");
			Assert::valid(trjIdle >= REAL_ZERO, ac, "trjIdle < 0");

			for (Mat34Seq::const_iterator i = action.begin(); i != action.end(); ++i)
				Assert::valid(i->isValid(), ac, "action[i]: invalid");

			Assert::valid(pathRenderer.isValid(), ac, "pathRenderer: invalid");
			Assert::valid(pathIncLarge > REAL_EPS && pathIncLarge <= REAL_ONE, ac, "pathIncLarge < eps or pathIncLarge > 1");
			Assert::valid(pathIncSmall > REAL_EPS && pathIncSmall <= REAL_ONE, ac, "pathIncSmall < eps or pathIncSmall > 1");

			importRobotTrj.assertValid(Assert::Context(ac, "importRobotTrj."));
			Assert::valid(!importHDF5FileExt.empty(), ac, "importHDF5FileExt: invalid");
			Assert::valid(!importHDF5RobotTrj.empty(), ac, "importHDF5RobotTrj: invalid");
		}

		/** Load descritpion from xml context. */
		virtual void load(Context& context, const XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual data::Handler::Ptr create(Context &context) const;
	};

	/** File extension: waypoint */
	static std::string getFileExtWaypoint();

protected:
	/** distRemoved callback */
	typedef std::function<void(size_t)> DistRemovedCallback;

	/** Rendering */
	mutable BoundsRenderer boundsRenderer;
	/** Bounds solid colour */
	RGBA boundsSolidColour;
	/** Path renderer */
	GraphRenderer pathRenderer;
	/** Path increment */
	Real pathIncLarge;
	/** Path increment */
	Real pathIncSmall;

	/** Bounds of kinematic chains */
	ChainBoundsSeq chainBoundsSeq;
	/** Bounds poses of kinematic chains */
	ChainMat34Seq chainMat34Seq;
	/** Bounds of kinematic joints */
	JointBoundsSeq jointBoundsSeq;
	/** Bounds poses of kinematic joints */
	JointMat34Seq jointMat34Seq;
	/** All bounds */
	Bounds::ConstSeq boundsSeq;
	/** Show bounds */
	bool boundsShow;
	/** Path position increment */
	Real pathPositionInc;
	/** Path request at contact */
	bool contactPathRequest;

	/** waypoint suffix */
	std::string waypointSuffix;

	/** Planner index */
	U32 plannerIndex;

	/** Planner */
	Planner* planner;
	/** Pointer to IG heuristic */
	golem::HBHeuristic* pHeuristic;
	/** Controller */
	const Controller* controller;
	/** Arm controller */
	const Controller* arm;
	/** Hand controller */
	const Controller* hand;
	/** Controller state info */
	Controller::State::Info info;
	/** Arm controller state info */
	Controller::State::Info armInfo;
	/** Hand controller state info */
	Controller::State::Info handInfo;
	/** Controller state info */
	golem::Controller::State::Ptr defaultState;


	/** Trajectory profile description */
	Profile::Desc::Ptr profileDesc;
	/** Trajectory profile */
	Profile::Ptr pProfile;

	/** Trajectory profile configspace distance multiplier */
	ConfigspaceCoord distance;
	/** Trajectory extrapolation */
	ConfigspaceCoord extrapolation;
	/** Trajectory motor command */
	ConfigspaceCoord command;

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

	/** Trajectory extrapolation */
	Real trjExtrapolation;
	/** Trajectory duration */
	Real trjDuration;
	/** IG Trajectory duration */
	Real trjR2GDuration;

	/** Trajectory idle */
	Real trjIdle;

	/** Trajectory velocity */
	ConfigspaceCoord velocityFac;
	/** Trajectory acceleration */
	ConfigspaceCoord accelerationFac;
	/** Trajectory distance */
	ConfigspaceCoord distanceFac;
	/** Trajectory extrapolation */
	ConfigspaceCoord extrapolationFac;
	/** Trajectory command */
	ConfigspaceCoord commandFac;

	/** Action */
	Mat34Seq action;

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
	void createRender(const ItemR2GTrajectory& item);
	/** UIRenderer: Render on output device. */
	virtual void render() const;
	/** UIRenderer: Render on output device. */
	virtual void customRender() const;

	/** UIRenderer: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	/** UIRenderer: Mouse motion handler. */
	virtual void motionHandler(int x, int y);
	/** UIRenderer: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);

	/** Construct empty item, accessible only by Data. */
	virtual Item::Ptr create() const;

	/** Profile::CallbackDist: Configuration space coordinate distance metric */
	virtual Real distConfigspaceCoord(const ConfigspaceCoord& prev, const ConfigspaceCoord& next) const;
	/** Profile::CallbackDist: Coordinate distance metric */
	virtual Real distCoord(Real prev, Real next) const;
	/** Profile::CallbackDist: Coordinate enabled state */
	virtual bool distCoordEnabled(const Configspace::Index& index) const;
	/** Profile::CallbackDist: Coordinate interpolate state */
	virtual bool distCoordInterpolate(const Configspace::Index& index) const;
	/** Profile::CallbackDist: Prune state sequence state */
	virtual void distRemoved(size_t index) const;

	/** Creates trajectory from state sequence */
	virtual void create(const ConfigspaceCoord& delta, Controller::State::Seq& trajectory) const;

	/** Profile state sequence */
	virtual void profile(SecTmReal duration, Controller::State::Seq& trajectory) const;

	/** HandlerPlanner: Planner index. */
	virtual U32 getPlannerIndex() const;
	/** HandlerPlan: Sets planner and controllers. */
	virtual void set(Planner& planner, const ControllerId::Seq& controllerIDSeq);

	/** Trajectory: Returns waypoints with velocity profile. */
	virtual void createTrajectory(const ItemR2GTrajectory& item, Controller::State::Seq& trajectory);
	/** Trajectory: Compute grasping and lifting trajectory: Returns waypoints with velocity profile. */
	virtual void createActionTrajectory(const ItemR2GTrajectory& item, const Controller::State& begin, Controller::State::Seq& trajectory);
	/** (Mycroft) Compute approaching trajectory: Returns waypoints with velocity profile. */
	virtual void createTrajectory(const ItemR2GTrajectory& item, const Controller::State& begin, Controller::State::Seq& trajectory);
	/** (IR3ne) Compute approaching trajectory: Returns waypoints with velocity profile. */
	virtual void createIGTrajectory(const ItemR2GTrajectory& item, const Controller::State& begin, Controller::State::Seq& trajectory);

	/** Pregrasp hand pose */
	RealSeq handPregraspPose;
	/** Grasp hand pose */
	RealSeq handGraspPose;

	/** Import: Import from file */
	virtual Item::Ptr import(const std::string& path);
	/** Import: Available file types */
	virtual const StringSeq& getImportFileTypes() const;

	/** Transform interfaces */
	StringSeq transformInterfaces;

	/** Transform: Transform input items */
	virtual Item::Ptr transform(const Item::List& input);
	/** Transform: return available interfaces */
	virtual const StringSeq& getTransformInterfaces() const;
	/** Transform: is supported by the interface */
	virtual bool isTransformSupported(const Item& item) const;

	/** StreamHandlerConfigspace: handle Configspace */
	virtual void convert(const golem::Configspace::Range& inpConfigspaceRange, const golem::Configspace::Range& outConfigspaceRange, const golem::Chainspace::Range& inpChainspaceRange, const golem::Chainspace::Range& outChainspaceRange, golem::ConfigspaceCoord& val);
	/** StreamHandlerReservedspace: handle Reservedspace */
	virtual void convert(const golem::Reservedspace::Range& inpRange, const golem::Reservedspace::Range& outRange, golem::ReservedCoord& val);

	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerR2GTrajectory(Context &context);
};

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void GOLEM_LIBRARY_DECLDIR XMLData(HandlerR2GTrajectory::ImportState::Map::value_type& val, XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};  // namespace data

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void GOLEM_LIBRARY_DECLDIR XMLData(const std::string &attr, data::HandlerR2GTrajectory::ImportState::Type& val, XMLContext* xmlcontext, bool create = false);

/** Reads/writes object from/to a given XML context */
void GOLEM_LIBRARY_DECLDIR XMLData(const std::string &attr, data::HandlerR2GTrajectory::ImportFrame::Type& val, XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

}; // namespace golem

#endif /**_GOLEM_DATA_R2GTRAJECTORY_H_**/
