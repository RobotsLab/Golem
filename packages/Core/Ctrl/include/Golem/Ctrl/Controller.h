/** @file Controller.h
 * 
 * Implementation of controller interface of a collection of kinematic chains. 
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
#ifndef _GOLEM_CTRL_CONTROLLER_H_
#define _GOLEM_CTRL_CONTROLLER_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Chain.h>
#include <Golem/Sys/Message.h>
#include <Golem/Sys/Context.h>
#include <Golem/Sys/LoadObjectDesc.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgController, Message)
MESSAGE_DEF(MsgControllerInvalidDesc, MsgController)
MESSAGE_DEF(MsgControllerOpenLib, MsgController)
MESSAGE_DEF(MsgControllerLoadDesc, MsgController)
MESSAGE_DEF(MsgControllerLimits, MsgController)
MESSAGE_DEF(MsgControllerLookup, MsgController)

//------------------------------------------------------------------------------

class XMLContext;

/** Kinematic chains controller interface. */
class Controller {
	friend class Joint;
	friend class Chain;

public:
	typedef shared_ptr<Controller> Ptr;
	typedef std::vector<Ptr> Seq;
	typedef std::vector<Controller*> PtrSeq;

	/** Controller state vector */
	class State : public GenConfigspaceState {
	public:
		/** Only Controller can create it */
		friend class Controller;
		/** Pointer */
		typedef shared_ptr<State> Ptr;
		/** Trajectory */
		typedef std::vector<State> Seq;

		/** Defines state properties */
		class Info {
		public:
			/** Chain space -> configuration space range mapping */
			typedef Chainspace::Coord<Configspace::Range> ChainspaceRangeMap;

			/** No data */
			Info() {
				resetJoints();
				resetReserved();
			}

			/** Resets to the default value: no joint & chain data */
			inline void resetJoints(idx_t joint = 0, idx_t chain = 0) {
				this->joints.set(joint, 0);
				this->chains.set(chain, 0);
				this->map[chains.begin()].set(*this->joints.begin(), 0);
			}
			/** Resets to the default value: no reserved data */
			inline void resetReserved(idx_t reserved = 0) {
				this->reserved.set(reserved, 0);
			}
		
			/** Adds joints to a chain */
			inline void addJoints(idx_t size) {
				map[chains.end()].set(*joints.end(), size);
				joints.add(size);
				chains.add(1);
			}
			/** Adds joints to a chain */
			inline void addReserved(idx_t size) {
				reserved.add(size);
			}

			/**	Returns joint range. */
			inline const Configspace::Range& getJoints() const {
				return joints;
			}
			/**	Returns joint range for a given chain. */
			inline const Configspace::Range& getJoints(const Chainspace::Index& chain) const {
				return map[chain];
			}
			/**	Returns chain range. */
			inline const Chainspace::Range& getChains() const {
				return chains;
			}
			/**	Returns reserved area range. */
			inline const Reservedspace::Range& getReserved() const {
				return reserved;
			}

			inline friend bool operator == (const Info &l, const Info &r) {
				return l.joints == r.joints && l.chains == r.chains && l.reserved == r.reserved;
			}
			inline friend bool operator != (const Info &l, const Info &r) {
				return l.joints != r.joints || l.chains != r.chains || l.reserved != r.reserved;
			}

		protected:
			/** joints range */
			Configspace::Range joints;
			/** chains range */
			Chainspace::Range chains;
			/** chain -> joint range mapping */
			ChainspaceRangeMap map;
			/** reserved area range */
			Reservedspace::Range reserved;
		};

		/** reserved area */
		ReservedCoord reserved;

		/** Copy constructor acquires ranges */
		State(const State& state) : info(state.info) {
			State::operator = (state);
		}
		/** Acquires ranges and overwrites time stamp */
		State(const State& state, SecTmReal t) : info(state.info) {
			State::operator = (state);
			this->t = t;
		}

		/** Assignment operator doeas not acquire new ranges */
		State &operator = (const State &state) {
			GenConfigspaceCoord::set(info.getJoints().begin(), info.getJoints().end(), state);
			reserved.set(info.getReserved().begin(), info.getReserved().end(), state.reserved);
			t = state.t;
			return *this;
		}
		/** Assignment operator from GenConfigspaceState */
		State &operator = (const GenConfigspaceState &state) {
			GenConfigspaceCoord::set(info.getJoints().begin(), info.getJoints().end(), state);
			t = state.t;
			return *this;
		}

		/** Info */
		const Info& getInfo() const {
			return info;
		}
		/** Info */
		void setInfo(const Info& info) {
			this->info = info;
		}

		/** Reserved area access: type */
		template <typename _Type> inline _Type& get(ptrdiff_t offset) {
			return (*(_Type*)(reserved.data() + offset));
		}
		template <typename _Type> inline const _Type& get(ptrdiff_t offset) const {
			return (*(const _Type*)(reserved.data() + offset));
		}

	protected:
		/** Info */
		Info info;

		/** Initialise ranges from controller */
		State(const Controller& controller) : info(controller.getStateInfo()) {
		}
	};

	/** State reserved area variable indices */
	enum ReservedIndex {
		/** Position (Configspace, Real) */
		RESERVED_INDEX_POSITION = 0,
		/** Velocity (Configspace, Real) */
		RESERVED_INDEX_VELOCITY,
		/** Acceleration (Configspace, Real) */
		RESERVED_INDEX_ACCELERATION,
		/** Force/Torque (Configspace, Real) */
		RESERVED_INDEX_FORCE_TORQUE,
		/** Joint stiffness (Configspace, Real) */
		RESERVED_INDEX_STIFFNESS,
		/** Joint damping (Configspace, Real) */
		RESERVED_INDEX_DAMPING,
		/** Reserved Type size */
		RESERVED_INDEX_SIZE = RESERVED_INDEX_DAMPING + 1,
	};

	/** State reserved area offset */
	struct ReservedOffset {
		typedef std::vector<idxdiff_t> Seq;

		/** Offset not available */
		static const ptrdiff_t UNAVAILABLE;

		/** Type */
		enum Type {
			/** Generic pointer array */
			TYPE_DEFAULT = 0,
			/** Chainspace array */
			TYPE_CHAINSPACE,
			/** Configpace array */
			TYPE_CONFIGSPACE,
		};

		static const std::string TypeName [];

		/** Name */
		const std::string name;
		/** Type */
		const Type type;
		/** Type size */
		const size_t size;
		/** Type count */
		const size_t count;

		/** Initialisation */
		ReservedOffset(const std::string& name, Type type, size_t size, size_t count) : name(name), type(type), size(size), count(count) {}

		/** Size */
		template <typename _Ptr> static size_t getSize(_Ptr begin, _Ptr end) {
			size_t size = 0;
			for (_Ptr i = begin; i != end; ++i)
				size += i->size*i->count;
			return size;
		}
		/** Offset */
		template <typename _Ptr> static void getOffset(Controller& controller, _Ptr begin, _Ptr end, size_t reservedBegin, size_t chainBegin, size_t jointBegin, Seq& seq) {
			seq.clear();
			idxdiff_t offset = 0;
			for (_Ptr i = begin; i != end; ++i) {
				const idxdiff_t size = i->size*i->count;
				if (size > 0) {
					idxdiff_t shift = reservedBegin;
					
					switch (i->type) {
					case TYPE_CHAINSPACE:
						shift -= chainBegin*i->size;
						break;
					case TYPE_CONFIGSPACE:
						shift -= jointBegin*i->size;
						break;
					}

					controller.getContext().verbose("%s: reserved area {id=%s, type=%s, size=%d, rel_offset=%d, abs_offset=%d}, {reserved=%d, chain=%d, joint=%d}\n", controller.getID().c_str(), i->name.c_str(), TypeName[i->type].c_str(), size, offset, offset + shift, reservedBegin, chainBegin, jointBegin);

					seq.push_back(offset + shift);
					offset += size;
				}
				else {
					controller.getContext().verbose("%s: reserved area (not available) {id=%s, type=%s}\n", controller.getID().c_str(), i->name.c_str(), TypeName[i->type].c_str());

					seq.push_back(UNAVAILABLE);
				}
			}
		}
	};

	/** Defines reserved data access */
	#define CONTROLLER_STATE_RESERVED(NAME, TYPE, INDEX)\
		inline TYPE& get##NAME(Controller::State& state) const {\
			return state.get<TYPE>(getReservedOffset(INDEX));\
		}\
		inline const TYPE& get##NAME(const Controller::State& state) const {\
			return state.get<const TYPE>(getReservedOffset(INDEX));\
		}

	/** Controller state trajectory */
	typedef State::Seq Trajectory;
	/** Controller state vector queue */
	typedef golem::queue<State> Queue;

	/** Callback interface for data synchronization */
	class CallbackDataSync {
	public:
		virtual ~CallbackDataSync() {}
		/** Auto synchronization of Chain bounds descriptions */
		virtual void syncChainBoundsDesc(Chain* pChain) {};
		/** Auto synchronization of Joint bounds descriptions */
		virtual void syncJointBoundsDesc(Joint* pJoint) {};
		/** synchronization of sent trajectory */
		virtual void syncSend(const State* begin) {};
	};

	/** Controller description */
	class Desc {
	public:
		friend class Controller;

		/** description pointer */
		typedef shared_ptr<Desc> Ptr;
		
		/** Name ASCII string */
		std::string name;
		/** Chains */
		Chain::Desc::Seq chains;
		/** Controller global pose */
		Mat34 globalPose;
		
		/** joints range begin */
		U32 jointBegin;
		/** chains range begin */
		U32 chainBegin;
		/** reserved area range begin */
		U32 reservedBegin;
		/** reserved area range size */
		U32 reservedSize;

		/** Callback interface for data synchronization */
		CallbackDataSync* pCallbackDataSync;

		/** Enable I/O */
		bool enableIO;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}
		
		/** virtual destructor is required */
		virtual ~Desc() {}
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			name = "";
			chains.clear();
			globalPose.setId();
			jointBegin = 0;
			chainBegin = 0;
			reservedBegin = 0;
			reservedSize = 0;
			pCallbackDataSync = NULL;
			enableIO = true;
		}
		
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (chains.size() <= 0 || chains.size() > Chainspace::DIM)
				return false;
			for (Chain::Desc::Seq::const_iterator i = chains.begin(); i != chains.end(); ++i)
				if (*i == NULL || !(*i)->isValid())
					return false;
			
			if (!globalPose.isFinite())
				return false;

			//if ((idx_t)joint >= Configspace::DIM || (idx_t)chain >= Chainspace::DIM || (idx_t)reserved >= Reservedspace::DIM)
			//	return false;

			return true;
		}
		
		/** Creates Controller given the description object. 
		* @param context	Context object
		* @return			pointer to the Controller interface if no errors have occured, throws otherwise
		*/
		virtual Controller::Ptr create(Context& context) const = 0;

		/** Loads controller description from dynamic library.
		* @param context		program context
		* @param libraryPath	library path
		* @param configPath		xml configuration path
		* @return				pointer to the Controller description if no errors have occured, throws otherwise
		*/
		static Controller::Desc::Ptr load(Context* context, const std::string& libraryPath, const std::string& configPath);

		/** Loads controller description from dynamic library.
		* @param context		program context
		* @param xmlcontext		xmlcontext which contains information about controller
		* @return				pointer to the Controller description if no errors have occured, throws otherwise
		*/
		static Controller::Desc::Ptr load(Context* context, XMLContext* xmlcontext);

	private:
		/** Library path */
		std::string libraryPath;
		/** Config path */
		std::string configPath;
	};

private:
	/** Library type */
	std::string type;
	/** Library id */
	std::string id;

	/** Library path */
	std::string libraryPath;
	/** Config path */
	std::string configPath;

	/** Name ASCII string */
	std::string name;
	
	/** Controller global pose */
	Mat34 globalPose;

	/** kinematic chains */
	Chain::Seq chainSeq;
	/** Chains pointers */
	Chain::SpacePtrSeq chains;
	/** Joints pointers */
	Joint::SpacePtrSeq joints;	
	
	/** Minimum value of extended coordinates */
	GenConfigspaceCoord configspaceMin;
	/** Maximum value of extended coordinates */
	GenConfigspaceCoord configspaceMax;
	/** Minimum and maximum limits' offset */
	GenConfigspaceCoord configspaceOffset;

	/** Controller state properties */
	State::Info stateInfo;

	/** Callback interface for data synchronization */
	CallbackDataSync dummyCallbackDataSync;
	CallbackDataSync* pCallbackDataSync;

protected:
	/** golem::Context object. */
	golem::Context& context;
	
	/** Controller pointers */
	Controller::PtrSeq controllers;

	/** Controller command critical section */
	mutable CriticalSection csCommand;
	/** Controller data members critical section */
	mutable CriticalSection csData;

	/** Sets controller state properties */
	virtual void setStateInfo(const Desc& desc);

	/** Creates Controller from the description. 
	* @param desc	Controller description
	*/
	void create(const Desc& desc);

	Controller(Context& context);

public:
	/** Each derived class should have virtual destructor releasing resources
	*	to avoid calling virtual functions of non-existing objects
	*/
	virtual ~Controller();

	/** Access to collection of controllers
	 * @return		reference to collection of controllers
	 */
	const Controller::PtrSeq& getControllers() const {
		return controllers;
	}

	/** Sends a sequence of motor commands preserving maximum time wait.
	 *
	 * @param begin			begin of the command sequence
	 * @param end			end of the command sequence
	 * @param clear			clear the command queue
	 * @param limits		assert that the entire trajectory is in the limits
	 * @param timeWait		total waiting time in milliseconds
	 * @return				pointer to the last sent waypoint
	*/
	virtual const State* send(const State* begin, const State* end, bool clear = false, bool limits = true, MSecTmU32 timeWait = MSEC_TM_U32_INF) = 0;
	
	/** Stops the device. */
	virtual void stop() = 0;

	/** Resumes the device. */
	virtual void resume() = 0;

	/** Controller has just sent new trajectory segment */
	virtual bool waitForBegin(MSecTmU32 timeWait = MSEC_TM_U32_INF) = 0;
	
	/** Controller has just sent the last trajectory segment */
	virtual bool waitForEnd(MSecTmU32 timeWait = MSEC_TM_U32_INF) = 0;

	/** Interpolates the controller state at time t.
	* @param state		state of the controller
	* @param t			query time t in seconds (t usually refers to the past)
	*/
	virtual void lookupState(SecTmReal t, State &state) const = 0;

	/** Interpolates the controller command at time t.
	* @param command		command of the controller
	* @param t				query time t in seconds (t usually refers to the future)
	*/
	virtual void lookupCommand(SecTmReal t, State &command) const = 0;

	/** I/O cycle duration. */
	virtual SecTmReal getCycleDuration() const = 0;

	/** I/O command (minimum) latency. */
	virtual SecTmReal getCommandLatency() const = 0;

	/** I/O command time stamp. */
	virtual SecTmReal getCommandTime() const = 0;

	/** I/O command queue capacity. */
	virtual U32 getCommandCapacity() const = 0;
	
	/** Clamp configuration. */
	virtual void clampConfig(GenConfigspaceCoord& config) const = 0;

	/** Clamp state. */
	virtual void clampState(State& state) const = 0;

	/** Interpolates configuration at time t. */
	virtual void interpolateConfig(const GenConfigspaceState& prev, const GenConfigspaceState& next, SecTmReal t, GenConfigspaceState& state) const = 0;

	/** Interpolates the controller state at time t. */
	virtual void interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const = 0;

	/** Interpolates the controller state at time t. */
	template <typename _SeqPtr> bool interpolateSeq(_SeqPtr begin, _SeqPtr end, SecTmReal t, State& state) const {
		_SeqPtr first, last;
		if (!Controller::State::lookup(begin, end, t, first, last))
			return false;

		// interpolate
		interpolateState(*first, *last, t, state);

		return true;
	}

	/** Asserts that the controller trajectory segment is in the limits. */
	virtual void assertLimits(const State& prev, const State& next) const = 0;

	/** Creates controller state with no initialisation. */
	virtual State createState() const;

	/** Sets to default a given state. */
	virtual void setToDefault(State& state) const;

	/** Controller state properties */
	inline const State::Info& getStateInfo() const {
		return stateInfo;
	}

	/** Returns reserved data offset, by default not available. */
	virtual ptrdiff_t getReservedOffset(U32 index) const {
		return ReservedOffset::UNAVAILABLE;
	}

	/** Forward transformation for tool frame each chain
	 * @param cc	configuration space coordinates
	 * @param wc	SE(3) transformation matrices for each chain:
	 *				tool frame pose -> base frame pose (includes the chain local pose)
	 */
	virtual void chainForwardTransform(const ConfigspaceCoord& cc, WorkspaceChainCoord& wc) const;

	/** Forward transformation for each joint
	 * @param cc	configuration space coordinates
	 * @param wc	array of SE(3) transformation matrices for joints in all chains:
	 *				joint frame poses -> base frame pose (includes the chain local pose)
	 */
	virtual void jointForwardTransform(const ConfigspaceCoord& cc, WorkspaceJointCoord& wc) const;

	/** Manipulator Jacobian
	 * @param cc	configuration space coordinates
	 * @param jac	manipulator Jacobian
	 */
	virtual void jacobian(const ConfigspaceCoord& cc, Jacobian& jac) const;

	/** Access to kinematic chains
	 * @return		reference to Chain container
	 */
	const Chain::SpacePtrSeq& getChains() const {
		return chains;
	}

	/** Access to Joints
	 * @return		reference to Joint container
	 */
	const Joint::SpacePtrSeq& getJoints() const {
		return joints;
	}

	/** Returns minimum value of extended coordinates */
	const GenConfigspaceCoord& getMin() const {
		return configspaceMin;
	}	
	/** Returns maximum value of extended coordinates */
	const GenConfigspaceCoord& getMax() const {
		return configspaceMax;
	}
	/** Returns min and max limits' Offset */
	const GenConfigspaceCoord& getOffset() const {
		return configspaceOffset;
	}

	/** Returns Controller global pose
	 * @return				Controller global pose
	 */
	virtual Mat34 getGlobalPose() const;

	/** Sets Controller global pose
	 * @param pose	Controller global pose
	 */
	virtual void setGlobalPose(const Mat34 &globalPose);
	
	/** Callback interface for data synchronization	*/
	CallbackDataSync* getCallbackDataSync() {
		return pCallbackDataSync ? pCallbackDataSync : &dummyCallbackDataSync;
	}
	/** Callback interface for data synchronization	*/
	const CallbackDataSync* getCallbackDataSync() const {
		return pCallbackDataSync ? pCallbackDataSync : &dummyCallbackDataSync;
	}
	
	/** Callback interface for data synchronization	*/
	void setCallbackDataSync(CallbackDataSync* pCallbackDataSync) {
		this->pCallbackDataSync = pCallbackDataSync;
	}

	/** Controller critical section */
	CriticalSection& getCommandCS() {
		return csCommand;
	}

	/** Library type */
	const std::string& getType() const {
		return type;
	}
	/** Library id */
	const std::string& getID() const {
		return id;
	}

	/** Library path */
	const std::string& getLibPath() const {
		return libraryPath;
	}
	/** Config path */
	const std::string& getConfigPath() const {
		return configPath;
	}

	/** Returns Name ASCII string */
	const std::string& getName() const {
		return name;
	}

	/** golem::Context object */
	const golem::Context& getContext() const {
		return context;
	}
	golem::Context& getContext() {
		return context;
	}
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CTRL_CONTROLLER_H_*/
