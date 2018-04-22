/** @file SingleCtrl.h
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
#ifndef _GOLEM_CTRL_SINGLECTRL_SINGLECTRL_H_
#define _GOLEM_CTRL_SINGLECTRL_SINGLECTRL_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Controller.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgControllerThreadLaunch, MsgController)
MESSAGE_DEF(MsgControllerInitFailure, MsgController)
MESSAGE_DEF(MsgControllerInvalidStamp, MsgController)
MESSAGE_DEF(MsgControllerNullCallback, MsgController)
MESSAGE_DEF(MsgControllerOutOfRange, MsgControllerLimits)

//------------------------------------------------------------------------------

/** Single controller interface. */
class SingleCtrl : public Controller, protected Runnable {
public:
	/** Median */
	template <typename _Type> class Median {
	public:
		template <typename _Ptr> _Type get(_Ptr begin, _Ptr end) {
			seq.assign(begin, end);
			const size_t n = seq.size()/2;
			//std::nth_element(seq.begin(), seq.begin() + n + 1, seq.end()); // not working!
			std::partial_sort(seq.begin(), seq.begin() + n + 1, seq.end());
			return seq[n];
		}

	private:
		std::vector<_Type> seq;
	};

	/** Callback interface for synchronisation, sending and receiving  */
	class CallbackIO {
	public:
		virtual ~CallbackIO() {}
		/** Data synchronisation */
		virtual void sysSync(SingleCtrl* ctrl) {
			ctrl->sysSync();
		}
		/** Data receive */
		virtual void sysRecv(SingleCtrl* ctrl, State& state) {
			ctrl->sysRecv(state);
		};
		/** Data send */
		virtual void sysSend(SingleCtrl* ctrl, const State& prev, State& next, bool bSendPrev, bool bSendNext) {
			ctrl->sysSend(prev, next, bSendPrev, bSendNext);
		};
	};

	/** Coordinate linear map */
	class GOLEM_LIBRARY_DECLDIR CoordMap {
	public:
		/** Collection */
		typedef std::vector<CoordMap> Seq;

		/** Source */
		U32 src;
		/** Destination */
		U32 dst;
		/** Gain */
		Real gain;
		/** Offset */
		Real offset;

		CoordMap() {
			setToDefault();
		}
		void setToDefault() {
			src = 0;
			dst = 1;
			gain = REAL_ONE;
			offset = REAL_ZERO;
		}
		bool isValid() const {
			if (src >= (U32)Configspace::DIM || dst >= (U32)Configspace::DIM || src == dst)
				return false;
			if (!Math::isFinite(gain) || !Math::isFinite(offset))
				return false;
			return true;
		}

		/** Applies a single map entry */
		void apply(const Configspace::Range& range, const ConfigspaceCoord& min, const ConfigspaceCoord& max, const ConfigspaceCoord& src, ConfigspaceCoord& dst) const;
		/** Applies map entries */
		static void apply(const Configspace::Range& range, const Seq& seq, const ConfigspaceCoord& min, const ConfigspaceCoord& max, const ConfigspaceCoord& src, ConfigspaceCoord& dst) {
			for (Seq::const_iterator i = seq.begin(); i != seq.end(); ++i)
				i->apply(range, min, max, src, dst);
		}
	};

	/** Controller description */
	class Desc : public Controller::Desc {
	public:
		/** Working thread priority */
		Thread::Priority threadPriority;
		/** Inter-thread signalling time out */
		MSecTmU32 threadTimeOut;
		
		/** State buffer size */
		size_t qStateSize;
		/** Command buffer size */
		size_t qCommandSize;
		
		/** Models the resolution of internal timers of a system - the smallest allowable time increment. */
		SecTmReal timeQuant;
		/** I/O cycle duration control */
		bool cycleDurationCtrl;
		/** I/O cycle continuos control */
		bool cycleContinuousCtrl;
		/** I/O cycle initialisation/calibration steps */
		U32 cycleLengthInit;
		/** I/O cycle moving average length */
		U32 cycleLengthAverage;
		/** I/O cycle initial duration. */
		SecTmReal cycleDurationInit;
		/** I/O cycle duration offset. */
		SecTmReal cycleDurationOffs;
		/** I/O cycle maximum deviation multiplier. */
		SecTmReal cycleDurationMaxDev;

		/** I/O simulator: recv() time duration */
		SecTmReal simDeltaRecv;
		/** I/O simulator: send() time duration */
		SecTmReal simDeltaSend;

		/** Callback interface for data sending and receiving  */
		CallbackIO* pCallbackIO;

		/** minimum and maximum limits' offset */
		GenCoord assertOffset;

		/** Controller debugging */
		U32 debug;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Controller::Desc::setToDefault();

			threadPriority = Thread::HIGHEST;
			threadTimeOut = 10000; //[msec]
		
			qStateSize = 5000;
			qCommandSize = 500;
		
			timeQuant = SecTmReal(0.0001);
			cycleDurationCtrl = true;
			cycleContinuousCtrl = false;
			cycleLengthInit = 20;
			cycleLengthAverage = 50;
			cycleDurationInit = SecTmReal(0.02);
			cycleDurationOffs = SecTmReal(0.001);
			cycleDurationMaxDev = SecTmReal(5.0);

			simDeltaRecv = SecTmReal(0.005);
			simDeltaSend = SecTmReal(0.005);

			pCallbackIO = NULL;

			assertOffset.set(REAL_ZERO, REAL_ZERO, REAL_ZERO);

			debug = 0;
		}
		
		/** Checks if the description is valid. */
		inline bool isValidIO() const {
			if (threadTimeOut <= 0)
				return false;
			if (qStateSize < 3 || qCommandSize < 3 || cycleLengthInit < 2 || cycleLengthAverage < 2)
				return false;
			if (timeQuant <= SecTmReal(0.0) || cycleDurationOffs < SecTmReal(0.0) || cycleDurationInit < timeQuant || cycleDurationMaxDev < SecTmReal(0.0))
				return false;
			return true;
		}
		/** Checks if the description is valid. */
		inline bool isValidLimits() const {
			if (simDeltaRecv < REAL_ZERO || simDeltaSend < REAL_ZERO)
				return false;
			if (!assertOffset.isValid())
				return false;
			return true;
		}
		/** Checks if the description is valid. */
		inline bool isValidSim() const {
			if (simDeltaRecv < timeQuant || simDeltaSend < timeQuant)
				return false;
			return true;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Controller::Desc::isValid())
				return false;

			if (!isValidIO() || !isValidLimits() || !isValidSim())
				return false;

			return true;
		}
	};

private:
	/** Default initial state */
	State::Ptr initStateDflt;

	/** Thread terminate condition */
	volatile bool bTerminate;
	/** I/O thread */
	Thread thread;
	/** I/O thread priority */
	Thread::Priority threadPriority;
	/** Inter-thread signalling time out */
	MSecTmU32 threadTimeOut;
		
	/** Controller has just sent new trajectory segment */
	Event evCommandBegin;
	/** Controller has just sent the last trajectory segment */
	Event evCommandEnd;
	/** Controller has just set new CallbackIO */
	Event evCallbackIO;

	/** Models the resolution of internal timers of a system - the smallest allowable time increment. */
	SecTmReal timeQuant;
	/** I/O cycle initialisation request */
	bool cycleInitRequest;
	/** I/O cycle initialisation/calibration steps */
	U32 cycleLengthInit;
	/** I/O cycle moving average length */
	U32 cycleLengthAverage;
	/** Cycle duration control */
	bool cycleDurationCtrl;
	/** I/O cycle continuos control */
	bool cycleContinuousCtrl;
	/** I/O cycle initial duration. */
	SecTmReal cycleDurationInit;
	/** I/O cycle duration offset. */
	SecTmReal cycleDurationOffs;
	/** I/O cycle maximum deviation multiplier. */
	SecTmReal cycleDurationMaxDev;
	/** I/O cycle duration. */
	SecTmReal cycleDuration;
	/** I/O command (minimum) latency. */
	SecTmReal commandLatency;
	/** I/O next command end */
	SecTmReal commandTime;

	/** I/O simulator recv() time duration */
	SecTmReal simDeltaRecv;
	/** I/O simulator send() time duration */
	SecTmReal simDeltaSend;
	/** I/O simulator sync() timers */
	golem::queue< std::pair<SecTmReal, PerfTimer> > simTimeSync;
	/** I/O simulator timer */
	golem::shared_ptr<Sleep> sleep;
	
	/** Callback interface for data sending and receiving  */
	CallbackIO defaultCallbackIO;
	CallbackIO *pCallbackIO, *pNewCallbackIO;

	/** Minimum value of extended coordinates */
	GenConfigspaceCoord assertMin;
	/** Maximum value of extended coordinates */
	GenConfigspaceCoord assertMax;

	/** Controller initialisation state. */
	bool bCreated;
	/** Controller io. */
	bool bIO;
	/** Controller debugging */
	U32 debug;

protected:
	/** Controll loop critical section */
	mutable CriticalSection csCycle;
	/** Controller state queue critical section */
	mutable CriticalSection csState;
	/** Controller state queue */
	golem::shared_ptr<Queue> qState;
	/** Controller sent command queue */
	golem::shared_ptr<Queue> qSent;
	/** Controller awaiting command queue */
	golem::shared_ptr<Queue> qCommand;

	/** Interpolates the controller state at time t. */
	virtual bool interpolate(Queue::const_iterator begin, Queue::const_iterator end, SecTmReal t, State& state) const;

	/** Models the resolution of internal timers of a system - the smallest allowable time increment. */
	SecTmReal getTimeQuant() const {
		return timeQuant;
	}

	/** Terminating */
	bool isTerminating() const {
		return bTerminate;
	}

	/** Assert limit */
	void assertLimit(bool cond, const char* str, Configspace::Index j, Real val, SecTmReal t, Real min, Real max, const State& prev, const State& next) const;

	/** Models the internal buffer of the device.
	 */
	virtual void sysSync();

	/** Receives device state.
	 * @param state			device state
	 */
	virtual void sysRecv(State& state);

	/** Models the input of a system.
	 * @param prev			previous state
	 * @param next			next state
	 */
	virtual void sysSend(const State& prev, const State& next, bool bSendPrev, bool bSendNext);
	
	/** Communication function working in a separate thread. */
	virtual void run();

	/** Shuts down control thread and releases related resources */
	void release();

	/** User create is called just before launching I/O thread. */
	virtual void userCreate() {}

	/** Creates Controller from the description. 
	* @param desc	Controller description
	*/
	void create(const Desc& desc);

	SingleCtrl(Context& context);

public:
	/** virtual destructor */
	virtual ~SingleCtrl();

	/** Sends a sequence of motor commands preserving maximum time wait. */
	virtual const State* send(const State* begin, const State* end, bool clear = false, bool limits = true, MSecTmU32 timeWait = MSEC_TM_U32_INF) {
		return send<const State*>(begin, end, clear, limits, timeWait);
	}

	template <typename _Ptr> _Ptr send(_Ptr begin, _Ptr end, bool clear = false, bool limits = true, MSecTmU32 timeWait = MSEC_TM_U32_INF) {
		_Ptr ptr = begin;

		if (begin != end) {
			SysTimer timer;

			// assert that trajectory segments are in the limits
			if (limits)
				for (_Ptr i = begin, j = begin; ++j != end; ++i)
					assertLimits(*i, *j);

			SecTmReal shift;

			{
				CriticalSectionWrapper csw(csCommand);

				// compute time shift if sent trajectory segments time stamps corresponds to past
				shift = std::max(commandTime + cycleDuration - ptr->t, SEC_TM_REAL_ZERO);
				// find trajectory segments to remove
				const Queue::iterator last = std::lower_bound(qCommand->begin(), clear ? ++qCommand->begin() : qCommand->end(), ptr->t + shift);
				// regardless of limits flag, assert that the first trajectory segment is in the limits
				if (!qCommand->empty() && last != qCommand->begin()) {
					State state = createState(); // range from this controller
					state = *ptr; // data copying, no range acquisition from ptr
					state.t += shift;
					if (limits)
						assertLimits(*(last - 1), state);
				}
				// clear events
				evCommandBegin.set(false);
				evCommandEnd.set(false);
				// erase all the trajectory waypoints with time stamps larger than begin->t
				qCommand->erase(last, qCommand->end());
				// send as much segments as possible
				for (size_t capacity = qCommand->capacity() - qCommand->size(); capacity > 0 && ptr != end; ++ptr, --capacity) {
					qCommand->push_back(*ptr); // copy constructor, no range acquisition
					qCommand->back().t += shift;
				}
			}

			// queue up next segments if there are any left and if there is time left
			if (timeWait > MSEC_TM_U32_ZERO)
				while (ptr != end) {
					const MSecTmU32 elapsed = timer.elapsed();
					if (!waitForBegin(timeWait < elapsed ? MSEC_TM_U32_ZERO : timeWait - elapsed))
						return ptr;

					CriticalSectionWrapper csw(csCommand);
			
					// if the queue is still full, try again
					if (qCommand->full())
						continue;

					qCommand->push_back(*ptr);
					qCommand->back().t += shift;
					++ptr;
				}
		}

		// inform the associated interface
		getCallbackDataSync()->syncSend(&*begin);

		return ptr;
	}

	/** Stops the device. */
	virtual void stop();

	/** Resumes the device. */
	virtual void resume();

	/** Controller has just sent new trajectory segment */
	virtual bool waitForBegin(MSecTmU32 timeWait = MSEC_TM_U32_INF);
	
	/** Controller has just sent the last trajectory segment */
	virtual bool waitForEnd(MSecTmU32 timeWait = MSEC_TM_U32_INF);

	/** Interpolates the controller state at time t. */
	virtual void lookupState(SecTmReal t, State &state) const;

	/** Interpolates the controller command at time t. */
	virtual void lookupCommand(SecTmReal t, State &command) const;

	/** Cycle duration control */
	bool hasCycleDurationCtrl() const;

	/** Cycle Continuous control */
	bool hasCycleContinuousCtrl() const;

	/** I/O cycle duration. */
	virtual SecTmReal getCycleDuration() const;

	/** I/O command (minimum) latency. */
	virtual SecTmReal getCommandLatency() const;

	/** I/O command time stamp. */
	virtual SecTmReal getCommandTime() const;

	/** I/O command queue capacity. */
	virtual U32 getCommandCapacity() const;

	/** Clamp configuration. */
	virtual void clampConfig(GenConfigspaceCoord& config) const;

	/** Clamp state. */
	virtual void clampState(State& state) const;

	/** Interpolates configuration at time t. */
	virtual void interpolateConfig(const GenConfigspaceState& prev, const GenConfigspaceState& next, SecTmReal t, GenConfigspaceState& state) const;

	/** Interpolates the controller state at time t. */
	virtual void interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const;

	/** Asserts that the controller trajectory segment is in the limits. */
	virtual void assertLimits(const State& prev, const State& next) const;

	/** I/O cycle initialisation/calibration steps */
	void initControlCycle();

	/** Get current callback interface  */
	inline CallbackIO* getCallbackIO() {
		return pCallbackIO;
	}
	/** Get current callback interface  */
	inline const CallbackIO* getCallbackIO() const {
		return pCallbackIO;
	}
	/** Set new callback interface  */
	bool setCallbackIO(CallbackIO* pCallbackIO, MSecTmU32 timeWait = MSEC_TM_U32_INF);
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CTRL_SINGLECTRL_SINGLECTRL_H_*/
