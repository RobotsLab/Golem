/** @file Manipulator.h
 *
 * Contact manipulator interface
 *
 * @author Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_CONTACT_MANIPULATOR_H_
#define _GOLEM_CONTACT_MANIPULATOR_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Defs.h>
#include <Golem/Plugin/Ctrl.h>
#include <Golem/Math/RB.h>
#include <Golem/Planner/Planner.h>
#include <Golem/Plugin/Renderer.h>
#include <Golem/Math/VecN.h>
#include <Golem/Sys/Stream.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Manipulator arm + hand proxy */
class Manipulator {
public:
	typedef golem::shared_ptr<Manipulator> Ptr;

	/** Control vector coordinates */
	typedef golem::_VecN<golem::Real, (size_t)golem::Configspace::DIM> ControlCoord;
	/** Configspace map */
	typedef golem::ScalarCoord<golem::I32, golem::Configspace> ConfigspaceMap;

	/** Command -> Control */
	typedef std::function<void(const golem::Controller::State&, ControlCoord&)> CommandToControl;
	/** Control -> Command */
	typedef std::function<void(const ControlCoord&, golem::Controller::State&)> ControlToCommand;

	/** Link identifier */
	class Link {
	public:
		/** Link type */
		enum Type {
			/** Joint */
			TYPE_JOINT = 0,
			/** Base */
			TYPE_BASE,
			/** Any */
			TYPE_ANY,
		};

		/** Link real map */
		typedef std::map<Link, golem::Real> RealMap;
		/** Link index map */
		typedef std::map<Link, golem::U32> IndexMap;

		/** Link coordinates */
		template <typename _Coord, size_t _DIM> class Coord {
		public:
			/** Type */
			typedef typename _Coord::Type Type;
			/** Size */
			static const size_t SIZE = _DIM + 1;

			/** Link coordinates components */
			union {
				struct {
					/** Joints contact */
					_Coord joints;
					/** Base contact */
					Type base;
				};
				/** As an array */
				Type links[SIZE];
			};

			/** With initialisation */
			Coord(Type value = Type()) {
				fill(value);
			}
			/** Sets the parameters to the default values */
			inline void fill(Type value = Type()) {
				std::fill(links, links + SIZE, value);
			}
			/**	Array subscript operator. */
			inline Type &operator [] (size_t index) {
				return links[index];
			}
			/**	Array subscript operator. */
			inline const Type &operator [] (size_t index) const {
				return links[index];
			}
		};

		/** Real link coordinates */
		typedef Coord<golem::ConfigspaceCoord, (size_t)golem::Configspace::DIM> RealCoord;
		/** Natural link coordinates */
		typedef Coord<golem::ScalarCoord<golem::U32, golem::Configspace>, (size_t)golem::Configspace::DIM> IndexCoord;

		/** Size */
		static const golem::U32 SIZE = (golem::U32)golem::Configspace::DIM + 1;
		/** Set of links */
		typedef std::bitset<SIZE> Set;

		/** Sets default values */
		Link(Type type = TYPE_JOINT, golem::U32 index = 0, golem::U32 offset = 0) : offset(offset) {
			set(type, index);
		}
		/** Configspace index */
		Link(golem::Configspace::Index index, golem::U32 offset = 0) : offset(offset) {
			set(TYPE_JOINT, static_cast<golem::U32>(*index));
		}
		/** Conversion from string */
		Link(const std::string& str, golem::U32 offset = 0) : offset(offset) {
			fromString(str);
		}

		/** Link index */
		golem::U32 getIndex() const {
			return index;
		}
		/** Link offset */
		golem::U32 getOffset() const {
			return offset;
		}
		/** Get type */
		Type getType() const {
			return index < (size_t)golem::Configspace::DIM ? TYPE_JOINT : TYPE_BASE;
		}
		/** Get joint index */
		golem::Configspace::Index getJoint() const {
			if (index >= (size_t)golem::Configspace::DIM)
				throw golem::Message(golem::Message::LEVEL_ERROR, "Manipulator::Link::getJoint(): not a joint!");
			return golem::Configspace::Index(index);
		}
		
		/** Set type and index */
		void set(Type type = TYPE_JOINT, golem::U32 index = 0) {
			if (type == TYPE_JOINT) {
				if (index >= (size_t)golem::Configspace::DIM)
					throw golem::Message(golem::Message::LEVEL_ERROR, "Manipulator::Link::set(): invalid index %u", index);
				this->index = index;
			}
			else if (type == TYPE_BASE)
				this->index = (golem::U32)golem::Configspace::DIM;
			else
				throw golem::Message(golem::Message::LEVEL_ERROR, "Manipulator::Link::set(): invalid type #%u", type);
		}
		/** Set index and offset */
		void setIndexOffset(golem::U32 index = 0, golem::U32 offset = 0) {
			if (index > (size_t)golem::Configspace::DIM)
				throw golem::Message(golem::Message::LEVEL_ERROR, "Manipulator::Link::setIndexOffset(): invalid index %u", index);
			this->index = index;
			this->offset = offset;
		}

		/** Link name */
		static const std::string& getName(Type type) {
			return name[size_t(type)];
		}
		/** Link name */
		std::string toString() const {
			const Type type = getType();
			std::string name = getName(type);
			if (type == TYPE_JOINT) name += std::to_string(getIndex() + 1 - offset);
			return name;
		}
		/** Link id */
		void fromString(const std::string& str) {
			if (str.find_first_of(getName(TYPE_JOINT)) == 0)
				set(TYPE_JOINT, (golem::U32)golem::Math::clamp(std::atoi(&str[getName(TYPE_JOINT).length()]), int(1), int(golem::Configspace::DIM)) - 1 + offset);
			else
				set(TYPE_BASE);
		}
		/** Key match with the link */
		bool is(const std::string& key) const;

		/** Get map item */
		template <typename _Iter, typename _Map> _Iter get(_Map& map) const {
			const std::string name = toString();
			// Fast 1:1 mapping: Joint-Joint/Base-Base
			_Iter iter = map.find(name);
			if (iter != map.end())
				return iter;
			// Slow linear search
			for (_Iter iter = map.begin(); iter != map.end(); ++iter)
				if (is(iter->first))
					return iter;
			// No match
			return map.end();
		}

		/** Link pointer <0..SIZE) */
		inline golem::U32 operator * () const {
			return index;
		}
		/** Link comparison */
		inline friend bool operator < (const Link& left, const Link& right) {
			return *left < *right;
		}

	private:
		/** Link default name */
		static const std::string name[TYPE_ANY + 1];

		/** Link index */
		golem::U32 index;
		/** Link offset */
		golem::U32 offset;
	};

	/** Manipulator kinematic configuration in configspace (joints) and workspace (effector frame) */
	class Config {
	public:
		/** Collection of configurations */
		typedef std::vector<Config> Seq;

		/** Kinematic parameter: Configuration */
		golem::ConfigspaceCoord config;
		/** Kinematic parameter: Base (end-effector) frame */
		RBCoord frame;

		/** No initialisation */
		Config() {}
		/** Copy constructor */
		Config(const golem::ConfigspaceCoord& config) : config(config) {}
		/** Copy constructor */
		Config(const RBCoord& frame) : frame(frame) {}
		/** Copy constructor */
		Config(const golem::ConfigspaceCoord& config, const RBCoord& frame) : config(config), frame(frame) {}

		/** Sets the parameters to the default values */
		inline void setToDefault(golem::Real val = golem::REAL_ZERO) {
			config.fill(val);
			frame.setId();
		}
	};

	/** Weighted manipulator configuration with control vector */
	class Waypoint : public Config, public golem::Sample<golem::Real> {
	public:
		/** Collection of waypoints */
		typedef std::vector<Waypoint> Seq;

		/** Dynamic parameter: Control vector coordinates (such as motor currents) */
		ControlCoord control;

		/** Default constructor */
		Waypoint(golem::Real weight = golem::REAL_ONE, golem::Real cdf = -golem::REAL_ONE) : golem::Sample<golem::Real>(weight, cdf) {}
		/** Copy constructor */
		Waypoint(const Config& config, const ControlCoord& control, golem::Real weight = golem::REAL_ONE, golem::Real cdf = -golem::REAL_ONE) : Config(config), golem::Sample<golem::Real>(weight, cdf), control(control) {}
		/** From Pose */
		explicit Waypoint(const golem::ConfigspaceCoord& config, const RBCoord& frame, golem::Real weight = golem::REAL_ONE, golem::Real cdf = -golem::REAL_ONE) : Config(config, frame), golem::Sample<golem::Real>(weight, cdf) {}

		/** Distance value */
		inline golem::Real getDistance() const {
			return weight;
		}
		/** Distance alignment */
		template <typename _Iter> static void alignDistance(_Iter begin, _Iter end, golem::Real offset) {
			for (; begin != end; ++begin)
				begin->weight += offset;
		}

		/** Low and high bounds of a waypoint sequence */
		template <typename _Iter> static void bounds(_Iter begin, _Iter end, golem::Real distance, _Iter& lo, _Iter& hi) {
			if (begin + 2 > end)
				throw golem::Message(golem::Message::LEVEL_ERROR, "Manipulator::Waypoint::bounds(): At least two waypoints required");

			// binary search
			hi = std::lower_bound(begin, end, distance, [] (const golem::Sample<golem::Real>& l, const golem::Real& r) {return l.weight < r; });
			if (hi == end)
				--hi;
			if (hi == begin)
				++hi;
			lo = hi - 1;
		}

		/** Find the closest waypoint */
		template <typename _Iter> static _Iter find(_Iter begin, _Iter end, golem::Real distance = golem::REAL_ZERO) {
			if (begin + 1 > end)
				throw golem::Message(golem::Message::LEVEL_ERROR, "Manipulator::Waypoint::find(): At least one waypoints required");
			else if (begin + 2 > end)
				return begin;
			else {
				_Iter lo, hi;
				bounds(begin, end, distance, lo, hi);
				return golem::Math::abs(lo->weight - distance) <= golem::Math::abs(hi->weight - distance) ? lo : hi;
			}
		}
	};

	/** Bounds Appearance */
	class BoundsAppearance {
	public:
		/** Show bounds solid */
		bool showSolid;
		/** Show bounds wire frames */
		bool showWire;
		/** Bounds solid colour */
		golem::RGBA solidColour;
		/** Bounds wire colour */
		golem::RGBA wireColour;
		/** Bounds wireframe thickness */
		golem::Real wireWidth;

		/** Constructs from description object */
		BoundsAppearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			showSolid = true;
			showWire = false;
			solidColour = golem::RGBA(golem::U8(255), golem::U8(255), golem::U8(0), golem::U8(127));
			wireColour = golem::RGBA(golem::U8(255), golem::U8(255), golem::U8(0), golem::U8(127));
			wireWidth = golem::Real(1.0);
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(wireWidth > golem::REAL_ZERO, ac, "wireWidth: <= 0");
		}
		/** Draw bounds */
		void draw(const golem::Bounds::Seq& bounds, golem::DebugRenderer& renderer) const;
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Appearance */
	class Appearance {
	public:
		/** Show bounds */
		bool showBounds;
		/** Show frames */
		bool showFrames;

		/** Bounds colour */
		BoundsAppearance bounds;
		/** Bounds selection colour */
		BoundsAppearance boundsSelection;

		/** Frame size joints */
		golem::Vec3 jointsFrameSize;
		/** Frame size chain */
		golem::Vec3 chainsFrameSize;

		/** Joint selection index */
		mutable golem::U32 selectionIndex;

		/** Constructs from description object */
		Appearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			bounds.setToDefault();
			boundsSelection.setToDefault();
			showBounds = true;
			showFrames = true;
			jointsFrameSize.set(golem::Real(0.02));
			chainsFrameSize.set(golem::Real(0.1));
			selectionIndex = (golem::U32)-1;
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			bounds.assertValid(Assert::Context(ac, "bounds."));
			boundsSelection.assertValid(Assert::Context(ac, "boundsSelection."));
			Assert::valid(jointsFrameSize >= golem::REAL_ZERO, ac, "jointsFrameSize: < 0");
			Assert::valid(chainsFrameSize >= golem::REAL_ZERO, ac, "chainsFrameSize: < 0");
		}
		/** Draw robot */
		void draw(const Manipulator& manipulator, const Config& config, golem::DebugRenderer& renderer) const;
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Configuration model description */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Trajectory error multiplier */
		RBDistEx trajectoryErr;
		/** Trajectory computation timeout */
		golem::SecTmReal trajectoryTimeout;
		/** Trajectory search maximum number of tries per cluster */
		golem::U32 trajectoryClusterSize;
		/** Trajectory throw */
		bool trajectoryThrow;

		/** State -> Control */
		CommandToControl commandToControl;
		/** Control -> State */
		ControlToCommand controlToCommand;

		/** Constructs object description */
		Desc() {
			Desc::setToDefault();
		}
		virtual ~Desc() {
		}
		/** Creates the object from the description. */
		virtual Manipulator::Ptr create(const golem::Planner& planner, const ControllerId::Seq& controllerIDSeq) const {
			return Manipulator::Ptr(new Manipulator(*this, planner, controllerIDSeq));
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			trajectoryErr.set(golem::Real(10000.0), golem::Real(10000.0), true);
			trajectoryTimeout = golem::SecTmReal(1.0);
			trajectoryClusterSize = 10;
			trajectoryThrow = false;
			commandToControl = nullptr;
			controlToCommand = nullptr;
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Assert::valid(trajectoryErr.isValid(), ac, "trajectoryErr: invalid");
			Assert::valid(trajectoryTimeout > golem::REAL_ZERO, ac, "trajectoryTimeout: <= 0");
			Assert::valid(trajectoryClusterSize > 0, ac, "trajectoryClusterSize: <= 0");
		}
		/** Load descritpion from xml context. */
		virtual void load(const golem::XMLContext* xmlcontext);
	};

	/** Create waypoints from trajectory with distance metrics */
	template <typename _Distance> Waypoint::Seq create(const golem::WaypointCtrl::Seq& waypoints, _Distance distance) {
		if (waypoints.empty())
			throw golem::Message(golem::Message::LEVEL_ERROR, "Manipulator::create(): At least one waypoints required");

		Waypoint::Seq seq;
		seq.reserve(waypoints.size());

		// create waypoints
		for (golem::WaypointCtrl::Seq::const_iterator i = waypoints.begin(); i != waypoints.end(); ++i) {
			const Waypoint* prev = i != waypoints.begin() ? &seq.back() : nullptr;
			const Config config(getConfig(i->state, prev));
			ControlCoord control;
			if (desc.commandToControl) desc.commandToControl(i->command, control); // optional
			seq.push_back(Waypoint(config, control, prev ? prev->getDistance() + distance(*prev, config) : golem::REAL_ZERO));
		}

		return seq;
	}
	
	/** Find path in the configuration space */
	virtual RBDist find(Waypoint::Seq& path) const;

	/** Find best path in the configuration space */
	template <typename _Iter, typename _Find> std::pair<_Iter, RBDistEx> find(_Iter begin, _Iter end, _Find find) const {
		if (begin == end)
			throw golem::Message(golem::Message::LEVEL_ERROR, "Manipulator::find(): At least one waypoint required");

		golem::Real err = golem::REAL_MAX;
		RBDistEx dist;
		_Iter j = begin;
		for (_Iter i = begin; i != end; ++i) {
			const RBDistEx newDist = find(i);
			const golem::Real newErr = desc.trajectoryErr.dot(newDist);
			if (err >= newErr && !newDist.collision || i == begin) {
				err  = newErr;
				dist = newDist;
				j = i;
			}
			if (newErr < golem::REAL_ONE && !newDist.collision)
				return std::make_pair(i, newDist);
		}

		if (desc.trajectoryThrow)
			throw golem::Message(golem::Message::LEVEL_ERROR, "Manipulator::find(): No trajectory found");

		return std::make_pair(j, dist);
	}
	/** Create waypoints from path */
	virtual void create(const Waypoint::Seq& path, golem::WaypointCtrl::Seq& waypoints) const;
	/** Path interpolation */
	virtual Config interpolate(const Waypoint::Seq& path, golem::Real distance) const;

	/** Manipulator configuration */
	virtual golem::Controller::State getState(const Waypoint& waypoint) const;

	/** Manipulator configuration */
	virtual Config getConfig(const golem::Controller::State& state, const Config* prev = nullptr) const;

	/** Fix joint limits and dependencies */
	virtual void clamp(Config& config) const;

	/** Manipulator base frame */
	golem::Mat34 getBaseFrame(const golem::ConfigspaceCoord& config) const;
	/** Joint frames */
	void getJointFrames(const golem::ConfigspaceCoord& config, const golem::Mat34& base, golem::WorkspaceJointCoord& joints) const;
	/** Local frame */
	golem::Mat34 getLocalFrame() const;
	/** Reference frame */
	golem::Mat34 getReferenceFrame() const;

	/** Base bounds */
	virtual bool hasBaseBounds() const;
	/** Base bounds */
	virtual golem::Bounds::Seq getBaseBounds() const;
	/** Base bounds */
	virtual void getBaseBounds(const golem::Mat34& frame, golem::Bounds::Seq& boundsSeq) const;
	/** Joint bounds */
	virtual bool hasJointBounds(golem::Configspace::Index joint) const;
	/** Joint bounds */
	virtual golem::Bounds::Seq getJointBounds(golem::Configspace::Index joint) const;
	/** Joint bounds */
	virtual void getJointBounds(golem::Configspace::Index joint, const golem::Mat34& frame, golem::Bounds::Seq& boundsSeq) const;
	/** Manipulator bounds at given pose */
	virtual golem::Bounds::Seq getBounds(const golem::ConfigspaceCoord& config, const golem::Mat34& frame) const;

	/** Planner */
	const golem::Planner& getPlanner() const {
		return planner;
	}
	/** Controller */
	const golem::Controller& getController() const {
		return controller;
	}
	/** Controller state info */
	inline const golem::Controller::State::Info& getInfo() const {
		return info;
	}
	
	/** Controller arm state info */
	inline const golem::Controller::State::Info& getArmInfo() const {
		return armInfo;
	}

	/** Controller hand state info */
	inline const golem::Controller::State::Info& getHandInfo() const {
		return handInfo;
	}

	/** Manipulator range */
	inline const golem::Configspace::Range& getConfigRange() const {
		return configRange;
	}
	/** Manipulator joint position limits */
	inline const golem::ConfigspaceCoord& getConfigMin() const {
		return configMin;
	}
	/** Manipulator joint position limits */
	inline const golem::ConfigspaceCoord& getConfigMax() const {
		return configMax;
	}

	/** Context */
	inline golem::Context& getContext() {
		return context;
	}
	inline const golem::Context& getContext() const {
		return context;
	}

	/** Configuration description */
	inline Desc& getDesc() {
		return desc;
	}
	inline const Desc& getDesc() const {
		return desc;
	}

	/** Object deletion */
	virtual ~Manipulator();

private:
	golem::Context& context;
	const golem::Planner& planner;
	const golem::Controller& controller;

	/** Controller state info */
	golem::Controller::State::Info info;
	/** Controller state info */
	golem::Controller::State::Info armInfo;
	/** Controller state info */
	golem::Controller::State::Info handInfo;

	/** Manipulator range */
	golem::Configspace::Range configRange;
	/** Manipulator joint position limits */
	golem::ConfigspaceCoord configMin, configMax;

	/** Local frame */
	golem::Mat34 localFrame;

	/** Manipulator description */
	Desc desc;

	/** From Golem controller */
	Manipulator(const Desc& desc, const golem::Planner& planner, const ControllerId::Seq& controllerIDSeq);
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

namespace golem {
	template <> void Stream::read(golem::Manipulator::Link& value) const;
	template <> void Stream::write(const golem::Manipulator::Link& value);

	template <> void Stream::read(golem::Manipulator::Link::Set& value) const;
	template <> void Stream::write(const golem::Manipulator::Link::Set& value);
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CONTACT_MANIPULATOR_H_*/
