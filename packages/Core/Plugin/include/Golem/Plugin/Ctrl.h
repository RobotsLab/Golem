/** @file Ctrl.h
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
#ifndef _GOLEM_PLUGIN_CTRL_H_
#define _GOLEM_PLUGIN_CTRL_H_

//------------------------------------------------------------------------------

#include <Golem/Sys/Defs.h>
#include <Golem/Sys/Stream.h>
#include <Golem/Planner/Planner.h>
#include <Golem/Plugin/Renderer.h>
#include <Golem/Math/RB.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Real and/or virtual controller identifier finds 2 types of mapping:
*	1) findController(): string id (library+config) -> golem::Controller*
*	2) findInfo(): string id (library+config) or chain range (for empty id) -> golem::Controller::State::Info
*/
class ControllerId {
public:
	/** Seq */
	typedef std::vector<ControllerId> Seq;
	/** SeqSeq */
	typedef std::vector<Seq> SeqSeq;

	/** Info */
	typedef golem::Controller::State::Info Info;
	/** Seq */
	typedef std::vector<Info> InfoSeq;

	/** String id */
	std::string id;
	/** Chain range */
	golem::Chainspace::Range range;

	/** Default init */
	ControllerId() {
		clear();
	}
	/** Default init */
	void clear() {
		id.clear();
		range.set(0, 0);
	}

	/** Available id */
	bool hasId() const {
		return id.length() > 0;
	}
	/** Available range */
	bool hasRange() const {
		return range.size() > 0;
	}

	/** String */
	std::string toString() const;

	/** Assert that the description is valid. */
	bool isValid() const {
		return hasId() || hasRange();
	}
	/** Assert that the description is valid. */
	void assertValid(const Assert::Context& ac) const {
		Assert::valid(isValid(), ac, "invalid chain range for empty id");
	}

	/** Find controller */
	golem::Controller* findController(golem::Controller &controller) const;
	/** Find info */
	Info findInfo(const golem::Controller &controller) const;

protected:
	/** Find controller */
	static golem::Controller* findController(const ControllerId& id, golem::Controller &controller);
};

//------------------------------------------------------------------------------

/** General planner debug info */
std::string plannerDebug(const golem::Planner& planner);
/** Configspace coordinate planner debug info */
std::string plannerConfigspaceDebug(const golem::Planner& planner, const golem::ConfigspaceCoord* c = nullptr);
/** Workspace coordinate planner debug info */
std::string plannerWorkspaceDebug(const golem::Planner& planner, const golem::WorkspaceChainCoord* w = nullptr);

//------------------------------------------------------------------------------

/** Waypoint */
class WaypointCtrl {
public:
	/** Trajectory */
	typedef std::vector<WaypointCtrl> Seq;

	/** Current state */
	golem::Controller::State state;
	/** Current command */
	golem::Controller::State command;

	/** From state and command */
	WaypointCtrl(const golem::Controller::State& state, const golem::Controller::State& command);

	/** Empty waypoint */
	static WaypointCtrl create(const golem::Controller& controller);
	/** Waypoint at time t */
	static WaypointCtrl lookup(const golem::Controller& controller, golem::SecTmReal t = golem::SEC_TM_REAL_MAX);

	/** Make commands or states */
	static golem::Controller::State::Seq make(const WaypointCtrl::Seq& waypoints, bool command = false);
	/** Make commands or states */
	static golem::WaypointCtrl::Seq make(const golem::Controller::State::Seq& states, const golem::Controller::State::Seq& commands);

protected:
	/** Creates blank waypoint, requires controller */
	WaypointCtrl(const golem::Controller& controller);
};

//------------------------------------------------------------------------------

/** Manifold */
class ManifoldCtrl {
public:
	/** Collection */
	typedef std::vector<ManifoldCtrl> Seq;

	/** Header */
	class Header : public golem::Header {
	public:
		/** Version */
		static const Version VERSION;
		/** Version */
		virtual Version getVersion() const { return VERSION; }
		/** Id */
		static const std::string ID;
		/** Id */
		virtual const std::string& getId() const { return ID; }
	} header;

	/** Frame */
	golem::Mat34 frame;
	/** Frame deviation */
	golem::Twist frameDev;

	/** Manifold size */
	static const size_t SIZE = 6;
	/** Manifold name string */
	static const char* name[SIZE];

	/** Manifold Appearance */
	class Appearance {
	public:
		/** Manifold distance norm/multiplier */
		//golem::Twist norm;
		/** Manifold visualisation norm/multiplier */
		golem::RBDist visNorm;
		/** Manifold visualisation range */
		golem::RBDist visRange;

		/** Manifold visualisation transparency */
		golem::U8 transparency;

		/** Show manifold */
		bool show;
		/** Show manifold frame */
		bool showFrame;

		/** Constructs from description object */
		Appearance() {
			Appearance::setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			//norm.set(Vec3(golem::Real(1.0)), Vec3(golem::Real(1.0)));
			visNorm.set(golem::Real(0.1), golem::Real(0.01));
			visRange.set(golem::Real(0.2), golem::Real(0.2));

			transparency = 127;

			show = true;
			showFrame = false;
		}
		/** Assert that the description is valid. */
		void assertValid(const golem::Assert::Context& ac) const {
			//golem::Assert::valid(norm.isPositive(), ac, "norm: < eps");
			golem::Assert::valid(visNorm.isPositive(), ac, "visNorm: < eps");
			golem::Assert::valid(visRange.isPositive(), ac, "visRange: < eps");
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);

		/** Draw Manifold */
		void draw(const ManifoldCtrl& manifold, const Mat34& frame, golem::DebugRenderer& renderer) const;
	};

	/** Constructs default manifold */
	ManifoldCtrl() {
		ManifoldCtrl::setToDefault();
	}
	/** Sets the parameters to the default values */
	void setToDefault() {
		frame.setId();
		frameDev.setZero();
	}
	/** Assert that the description is valid. */
	void assertValid(const Assert::Context& ac) const {
		Assert::valid(frame.isValid(), ac, "frame: invalid");
		Assert::valid(frameDev.isFinite(), ac, "frameDev: invalid");
	}

	/** Availability */
	bool isAvailable() const;
};

/** Reads/writes object from/to a given XML context */
void XMLData(golem::ManifoldCtrl& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

/** Manipulation trajectory profile */
class ManipDist : public Profile::CallbackDist {
private:
	Profile& profile;
	const Profile::CallbackDist* callbackDist;

public:
	ManipDist(Profile& profile);
	virtual ~ManipDist();
	virtual Real distConfigspaceCoord(const ConfigspaceCoord& prev, const ConfigspaceCoord& next) const;
	virtual Real distCoord(Real prev, Real next) const;
	virtual bool distCoordPlanning(const Configspace::Index& index) const;
};

//------------------------------------------------------------------------------

/** Stream Handler Configspace */
class StreamHandlerConfigspace : public virtual golem::Stream::Handler {
public:
	/** handle Configspace */
	virtual void convert(const golem::Configspace::Range& inpConfigspaceRange, const golem::Configspace::Range& outConfigspaceRange, const golem::Chainspace::Range& inpChainspaceRange, const golem::Chainspace::Range& outChainspaceRange, golem::ConfigspaceCoord& val) = 0;
};

/** Stream Handler Reservedspace */
class StreamHandlerReservedspace : public virtual golem::Stream::Handler {
public:
	/** handle Reservedspace */
	virtual void convert(const golem::Reservedspace::Range& inpRange, const golem::Reservedspace::Range& outRange, golem::ReservedCoord& val) = 0;
};

//------------------------------------------------------------------------------

template <> void Stream::read(Controller::State& state) const;
template <> void Stream::write(const Controller::State& state);

template <> void Stream::read(golem::WaypointCtrl& waypoint) const;
template <> void Stream::write(const golem::WaypointCtrl& waypoint);

/** Reads/writes object from/to a given XML context */
void XMLData(golem::ControllerId& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

namespace golem {
	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::ManifoldCtrl& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::ManifoldCtrl& value);
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_PLUGIN_CTRL_H_*/
