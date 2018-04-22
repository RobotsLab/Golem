/** @file SingleCtrl.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/SingleCtrl/SingleCtrl.h>
#include <Golem/Ctrl/Trajectory.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void SingleCtrl::CoordMap::apply(const Configspace::Range& range, const ConfigspaceCoord& min, const ConfigspaceCoord& max, const ConfigspaceCoord& src, ConfigspaceCoord& dst) const {
	const Configspace::Index is = range.begin() + std::min(range.size() - 1, (idx_t)this->src);
	const Configspace::Index id = range.begin() + std::min(range.size() - 1, (idx_t)this->dst);
	dst[id] = Math::clamp(gain*src[is] + offset, min[id], max[id]);
}

//------------------------------------------------------------------------------

SingleCtrl::SingleCtrl(golem::Context& context) : Controller(context), pCallbackIO(NULL), pNewCallbackIO(NULL) {
	bTerminate = bCreated = bIO = false;
}

SingleCtrl::~SingleCtrl() {
	SingleCtrl::release();
	// also call release() in each destructor of a derived class
}

void SingleCtrl::create(const Desc& desc) {
	Controller::create(desc); // throws
	
	initStateDflt.reset(new State(createState()));
	setToDefault(*initStateDflt);
	for (Configspace::Index i = getStateInfo().getJoints().begin(); i < getStateInfo().getJoints().end(); ++i) {
		initStateDflt->cpos[i] = getJoints()[i]->getTrn().theta;
	}

	threadPriority = desc.threadPriority;
	threadTimeOut = desc.threadTimeOut;
		
	cycleDurationCtrl = desc.cycleDurationCtrl;
	cycleContinuousCtrl = desc.cycleContinuousCtrl;
	cycleDurationInit = desc.cycleDurationInit;
	cycleDurationOffs = desc.cycleDurationOffs;
	cycleLengthAverage = desc.cycleLengthAverage;
	cycleLengthInit = desc.cycleLengthInit;
	cycleInitRequest = true;
	timeQuant = desc.timeQuant;
	cycleDurationMaxDev = desc.cycleDurationMaxDev;
		
	simDeltaRecv = desc.simDeltaRecv;
	simDeltaSend = desc.simDeltaSend;

	for (Configspace::Index i = getStateInfo().getJoints().begin(); i < getStateInfo().getJoints().end(); ++i) {
		assertMin.cpos[i] = getMin().cpos[i] + desc.assertOffset.pos - REAL_EPS;
		assertMin.cvel[i] = getMin().cvel[i] + desc.assertOffset.vel - REAL_EPS;
		assertMin.cacc[i] = getMin().cacc[i] + desc.assertOffset.acc - REAL_EPS;
		assertMax.cpos[i] = getMax().cpos[i] - desc.assertOffset.pos + REAL_EPS;
		assertMax.cvel[i] = getMax().cvel[i] - desc.assertOffset.vel + REAL_EPS;
		assertMax.cacc[i] = getMax().cacc[i] - desc.assertOffset.acc + REAL_EPS;
	}

	debug = desc.debug;

	// reserve queue space
	qState.reset(new Queue(desc.qStateSize, *initStateDflt));
	qSent.reset(new Queue(desc.qStateSize, *initStateDflt));
	qCommand.reset(new Queue(desc.qCommandSize, *initStateDflt));
	simTimeSync.reserve(2); // a duration of the exactly two commands

	// init cycle constants
	cycleDuration = cycleDurationInit;
	commandLatency = cycleDuration + cycleDuration;
	commandTime = context.getTimer().elapsed();
	evCommandBegin.set(false);
	evCommandEnd.set(false);

	// i/o callback
	pCallbackIO = pNewCallbackIO = desc.pCallbackIO ? desc.pCallbackIO : &defaultCallbackIO;

	if (desc.enableIO) {
		// user create
		userCreate();

		// launch I/O thread
		if (!thread.start(this))
			throw MsgControllerThreadLaunch(Message::LEVEL_CRIT, "SingleCtrl::create(): %s: Unable to launch Controller thread", getName().c_str());
		if (!thread.setPriority(desc.threadPriority))
			context.error("SingleCtrl::create(): %s: Unable to change Controller thread priority\n", getName().c_str());
	
		bIO = true;

		// and wait to finish initial cycles
		if (!waitForBegin(threadTimeOut))
			throw MsgControllerInitFailure(Message::LEVEL_CRIT, "SingleCtrl::create(): %s: Initialisation timeout", getName().c_str());
	}

	bCreated = true;
}

void SingleCtrl::release() {
	if (bIO) {
		bTerminate = true;
		if (!thread.join(threadTimeOut))
			context.error("SingleCtrl::release(): %s: Unable to stop working thread\n", getName().c_str());
		else
			bIO = false;
	}
}

//------------------------------------------------------------------------------

void SingleCtrl::sysSync() {
	if (!cycleDurationCtrl)
		simTimeSync.push_back(std::pair<SecTmReal, PerfTimer>(cycleDurationInit, PerfTimer()));

	if (simTimeSync.size() > 1) {
		// sleep until previous trajectory is complete
		sleep->sleep(std::max(SEC_TM_REAL_ZERO, simTimeSync.front().first - simTimeSync.front().second.elapsed()));
		simTimeSync.pop_front();
	}

	// new trajectory just started - reset timer
	simTimeSync.front().second.reset();
}

void SingleCtrl::sysRecv(State& state) {
	{
		CriticalSectionWrapper csw(csCommand);	// there can be writers in other threads
		if (!qSent->empty() && !interpolate(qSent->begin(), qSent->end(), context.getTimer().elapsed(), state))
			context.error("SingleCtrl::sysRecv(): %s: qSent interpolation error\n", getName().c_str());
	}

	sleep->sleep(simDeltaRecv);
}

void SingleCtrl::sysSend(const State& prev, const State& next, bool bSendPrev, bool bSendNext) {
	// nothing to do here in simulation mode
	
	sleep->sleep(simDeltaSend);
}

void SingleCtrl::run() {
	try {
		// control variables
		State state = *initStateDflt, prev = createState(), next = createState();

		bool bPrev = false;
		U32 cycleInit = 0;
		sleep.reset(new Sleep);

		// measurement variables
		SecTmReal tm[5] = {SEC_TM_REAL_ZERO}, tmPrev[5] = {SEC_TM_REAL_ZERO};
		golem::queue<SecTmReal> qCycle(cycleLengthAverage), qLatency(cycleLengthAverage), qRecv(cycleLengthAverage), qSend(cycleLengthAverage);
		Median<SecTmReal> median;

		while (!isTerminating()) {
			tm[0] = context.getTimer().elapsed();
		
			// initialisation
			{
				CriticalSectionWrapper csw(csCycle);
				// cycle init
				if (cycleInitRequest) {
					cycleInitRequest = false;
					cycleInit = cycleLengthInit;
				}
				// update callback
				pCallbackIO = pNewCallbackIO;
				evCallbackIO.set(true);
			}

			// Block until new trajectory is started, report start time tSync
			pCallbackIO->sysSync(this);
		
			tm[1] = context.getTimer().elapsed();
		
			// Receive the current state
			pCallbackIO->sysRecv(this, state);
		
			// process state queue
			{
				CriticalSectionWrapper csw(csState);	// there can be readers in other threads
				// queue up received data
				qState->push_back(state);
			}
		
			tm[2] = context.getTimer().elapsed();

			// previous trajectory end // tm[3] ~ tm[2] + (tmPrev[3] - tmPrev[2]) - just after setting on send events - ***NOT*** tm[1]
			const SecTmReal tPrev = tm[1] + cycleDuration;//tm[2] + commandLatency;
			// next trajectory end
			const SecTmReal tNext = tPrev + cycleDuration;
			// send flags
			bool bCurr, bNext;

			// process command queue
			{
				CriticalSectionWrapper csw(csCommand);	// there can be readers and writers in other threads

				// update next command end time stamp
				commandTime = tNext;
			
				// initialisation
				if (qCommand->empty() || qSent->empty() || cycleInit >= cycleLengthInit) {
					// clear to assure that there are no other waypoints in the queue
					qCommand->clear();
					qSent->clear();
					// generate the initial state
					prev = state;
					prev.t = tPrev;
					qCommand->push_back(prev);
					qSent->push_back(prev);
				}

				// calibration
				if (cycleInit > 0) {
					// generate next using the previous one
					next = prev;
					next.t = tNext;
					qCommand->push_back(next);
					// sending now, but conditionally in the next step
					bCurr = true;
					bNext = !cycleDurationCtrl || cycleInit > 1;
				}
				else
				// regular operation
				{
					const Queue::const_iterator last = --qCommand->end();
					// there is still something in the command queue
					bCurr = cycleContinuousCtrl || !cycleDurationCtrl || qCommand->size() > 1;
					// at least half of the cycle duration after tNext 
					bNext = last->t > tNext + SecTmReal(0.5)*cycleDuration;
				}

				// interpolate next command
				if (bCurr) {
					if (interpolate(qCommand->begin(), qCommand->end(), tNext, next)) {
						// update time stamps (due to possible range clamping)
						prev.t = tPrev;
						next.t = tNext;
						// remove all the commands before and including time tNext
						const Queue::iterator last = std::upper_bound(qCommand->begin(), qCommand->end(), tNext);
						qCommand->erase(qCommand->begin(), last);
						// bound the front of the command queue with next
						qCommand->push_front(next);
						// and insert the latest command to the sent queue
						qSent->push_back(next);
					}
					else {
						// this should never happen - interpolate should always succeed
						context.error("SingleCtrl::run(): %s: qCommand interpolation error\n", getName().c_str());
						// TODO handle the problem
					}
				}
				else {
					// cycleDurationCtrl==true
					qCommand->front().t = next.t = tNext;
				}

				// wakeup threads waiting to send new commands
				evCommandBegin.set(cycleInit <= 1);
				evCommandEnd.set(!bNext);
			}
		
			//if (bCurr) {
			//	// send callback
			//	getCallbackDataIO()->send(prev, next, bPrev, bNext);
			//}

			//// process command queue
			//{
			//	CriticalSectionWrapper csw(csCommand);	// there can be readers and writers in other threads

			//	if (bCurr) {
			//		// update the front of the command queue
			//		qCommand->front() = next;
			//		// update the back of the sent queue
			//		qSent->back() = next;
			//	}

			//	// wakeup threads waiting to send new commands
			//	evCommandBegin.set(cycleInit <= 1);
			//	evCommandEnd.set(!bNext);
			//}

			//tm[3] = context.getTimer().elapsed();
		
			//if (bCurr) {
			//	// send data
			//	sysSend(prev, next, bPrev, bNext);
			//	// queue up simulator data
			//	if (cycleDurationCtrl)
			//		simTimeSync.push_back(std::pair<SecTmReal, PerfTimer>(next.t - prev.t, PerfTimer()));
			//}

			//tm[4] = context.getTimer().elapsed();

			tm[3] = context.getTimer().elapsed();

			if (bCurr) {
				// send data
				pCallbackIO->sysSend(this, prev, next, bPrev, bNext);
				// queue up simulator data
				if (cycleDurationCtrl)
					simTimeSync.push_back(std::pair<SecTmReal, PerfTimer>(next.t - prev.t, PerfTimer()));
			}

			tm[4] = context.getTimer().elapsed();

			// update queues, trigger events
			{
				CriticalSectionWrapper csw(csCommand);	// there can be readers and writers in other threads

				if (bCurr) {
					// update the front of the command queue
					qCommand->front() = next;
					// update the back of the sent queue
					qSent->back() = next;
				}
			}

			// update cycle variables
			if (bPrev && bCurr) {
				// update cycle duration
				qCycle.push_back(tm[1] - tmPrev[1]);
				const SecTmReal qCycleDuration = median.get(qCycle.begin(), qCycle.end());
				// update cycle (minimum) latency (from the moment of setting on send events)
				qLatency.push_back(tm[1] - tmPrev[3]);
				const SecTmReal qLatencyDuration = median.get(qLatency.begin(), qLatency.end());

				if (cycleDurationCtrl) {
					// update recv() duration
					qRecv.push_back(tm[2] - tm[1]);
					const SecTmReal qRecvDuration = median.get(qRecv.begin(), qRecv.end());
					// update send() duration
					qSend.push_back(tm[4] - tm[3]);
					const SecTmReal qSendDuration = median.get(qSend.begin(), qSend.end());
					cycleDuration = qRecvDuration + qSendDuration + cycleDurationOffs;
				}
				else
					cycleDuration = qCycleDuration;
				
				commandLatency = cycleDuration + qLatencyDuration;
				cycleDuration = Math::round(cycleDuration, timeQuant);
				commandLatency = Math::round(commandLatency, timeQuant);

				// report unexpected cycle delays
				if (cycleInit <= 0 && qCycle.back() > qCycleDuration + cycleDurationMaxDev*qCycleDuration)
					context.warning("SingleCtrl::run(): %s: I/O cycle timeout = %f/%f\n", getName().c_str(), qCycle.back(), qCycleDuration);
			}

			if (debug > 0) {
				context.info(
					"%s: (bPrev=%s, bCurr=%s, bNext=%s), (qRecv=%d, qCmd=%d, qSent=%d), tPrev=%.5f, tNext=%.5f, tLast=%f, dt=%.5f, dCycleDur=%.5f, dCmdLat=%.5f\n",
					getName().c_str(), bPrev?"Y":"N", bCurr?"Y":"N", bNext?"Y":"N", qState->size(), qCommand->size(), qSent->size(), prev.t, next.t, qCommand->back().t, next.t - prev.t, cycleDuration, commandLatency
				);
			}

			bPrev = bCurr;
			prev = next;
			if (cycleInit > 0 && --cycleInit <= 0)
				context.debug("SingleCtrl::run(): %s: I/O cycle duration ctrl = %s, I/O cycle duration=%.5f, command latency=%.5f\n", getName().c_str(), cycleDurationCtrl ? "YES" : "NO", cycleDuration, commandLatency);
		
			std::copy(tm, tm + 5, tmPrev);
		}
	}
	catch (const std::exception& ex) {
		context.crit("SingleCtrl::run(): %s: controller stopped: %s\n", getName().c_str(), ex.what());
	}

	bTerminate = true;
	evCommandBegin.set(true);
	evCommandEnd.set(true);
}

//------------------------------------------------------------------------------

void SingleCtrl::stop() {
	CriticalSectionWrapper csw(csCommand);	// there can be readers and writers in other threads

	// last sent target
	const State& prev = qCommand->front();
	
	// compute next target such that vel=0 for each joint
	State next = prev;
	next.t = prev.t + cycleDuration;
	GenCoordTrj trj;
	for (Configspace::Index i = getStateInfo().getJoints().begin(); i < getStateInfo().getJoints().end(); ++i) {
		// boundary conditions are: at t0 {pos, vel, acc}; at t1 {pos=?, vel=0, acc=?}
		trj.set2pvav(prev.t, next.t, prev.cpos[i], prev.cvel[i], prev.cacc[i], Real(0.0));
		next.fromGenCoord(i, trj.get(next.t));
	}
	
	// queue up
	qCommand->erase(++qCommand->begin(), qCommand->end());
	qCommand->push_back(next);
}

void SingleCtrl::resume() {
}

bool SingleCtrl::waitForBegin(MSecTmU32 timeWait) {
	const bool bCmd = evCommandBegin.wait(timeWait);

	if (isTerminating()) {
		context.notice("SingleCtrl::waitForBegin(): %s: controller stopped\n", getName().c_str());
		return false;
	}

	if (bCmd) {
		CriticalSectionWrapper csw(csCommand);
		evCommandBegin.set(false);
		return true;
	}
	return false;
}
	
bool SingleCtrl::waitForEnd(MSecTmU32 timeWait) {
	const bool bCmd = evCommandEnd.wait(timeWait);

	if (isTerminating()) {
		context.notice("SingleCtrl::waitForEnd(): %s: controller stopped\n", getName().c_str());
		return false;
	}

	if (bCmd) {
		CriticalSectionWrapper csw(csCommand);
		evCommandEnd.set(false);
		return true;
	}
	return false;
}

void SingleCtrl::lookupState(SecTmReal t, State &state) const {
	CriticalSectionWrapper csw(csState);
	if (!interpolate(qState->begin(), qState->end(), t, state))
		throw MsgControllerLookup(Message::LEVEL_ERROR, "SingleCtrl::lookupState(): %s: interpolation error", getName().c_str());
}

void SingleCtrl::lookupCommand(SecTmReal t, State &command) const {
	CriticalSectionWrapper csw(csCommand);
	if (qCommand->front() <= t ? !interpolate(qCommand->begin(), qCommand->end(), t, command) : !interpolate(qSent->begin(), qSent->end(), t, command))
		throw MsgControllerLookup(Message::LEVEL_ERROR, "SingleCtrl::lookupCommand(): %s: interpolation error", getName().c_str());
}

//------------------------------------------------------------------------------

bool SingleCtrl::hasCycleDurationCtrl() const {
	return cycleDurationCtrl;
}

bool SingleCtrl::hasCycleContinuousCtrl() const {
	return cycleContinuousCtrl;
}

SecTmReal SingleCtrl::getCycleDuration() const {
	CriticalSectionWrapper csw(csCommand);
	return cycleDuration;
}

SecTmReal SingleCtrl::getCommandLatency() const {
	CriticalSectionWrapper csw(csCommand);
	return commandLatency;
}

SecTmReal SingleCtrl::getCommandTime() const {
	CriticalSectionWrapper csw(csCommand);
	return commandTime;
}

U32 SingleCtrl::getCommandCapacity() const {
	CriticalSectionWrapper csw(csCommand);
	return (U32)(qCommand->capacity() - qCommand->size());
}

bool SingleCtrl::interpolate(Queue::const_iterator begin, Queue::const_iterator end, SecTmReal t, State& state) const {
	return interpolateSeq(begin, end, t, state);
}

void SingleCtrl::clampConfig(GenConfigspaceCoord& config) const {
	for (Configspace::Index i = getStateInfo().getJoints().begin(); i < getStateInfo().getJoints().end(); ++i) {
		config.cpos[i] = Math::clamp(config.cpos[i], getMin().cpos[i], getMax().cpos[i]);
		config.cvel[i] = Math::clamp(config.cvel[i], getMin().cvel[i], getMax().cvel[i]);
		config.cacc[i] = Math::clamp(config.cacc[i], getMin().cacc[i], getMax().cacc[i]);
	}
}

void SingleCtrl::clampState(State& state) const {
	// config only
	clampConfig(state);
}


void SingleCtrl::interpolateConfig(const GenConfigspaceState& prev, const GenConfigspaceState& next, SecTmReal t, GenConfigspaceState& state) const {
	// clamp on limits
	state.t = Math::clamp(t, prev.t, next.t);

	// 3-rd degree polynomial interpolation 	
	const bool interpol = prev.t + numeric_const<Real>::EPS < next.t;
	GenCoordTrj trj;
	for (Configspace::Index i = getStateInfo().getJoints().begin(); i < getStateInfo().getJoints().end(); ++i)
		// boundary conditions are: at prev.t {pos, vel}; at next.t {pos, vel}
		if (interpol) {
			trj.set2pvpv(prev.t, next.t, prev.cpos[i], prev.cvel[i], next.cpos[i], next.cvel[i]);
			state.fromGenCoord(i, trj.get(state.t));
		}
		else {
			state.fromGenCoord(i, next.toGenCoord(i));
		}
}

void SingleCtrl::interpolateState(const State& prev, const State& next, SecTmReal t, State& state) const {
	// interpolate config only
	interpolateConfig(prev, next, t, state);
}

void SingleCtrl::assertLimit(bool cond, const char* str, Configspace::Index j, Real val, SecTmReal t, Real min, Real max, const State& prev, const State& next) const {
	if (!cond)
		throw MsgControllerOutOfRange(Message::LEVEL_ERROR, 
			"SingleCtrl::assertLimit(): %s: Joint %d %s %.3f at %.3f not in range <%.3f, %.3f>: prev={%.5f, (%.5f, %.5f, %.5f)}, next={%.5f, (%.5f, %.5f, %.5f)}",
			getName().c_str(),
			*j + 1, str, val, t, min, max,
			prev.t, prev.cpos[j], prev.cvel[j], prev.cacc[j], next.t, next.cpos[j], next.cvel[j], next.cacc[j]
		);
}

void SingleCtrl::assertLimits(const State& prev, const State& next) const {
	if (prev.t + SecTmReal(0.5)*cycleDuration >= next.t)
		throw MsgControllerInvalidStamp(Message::LEVEL_ERROR, "SingleCtrl::assertLimits(): %s: Trajectory time increment too small: %f -> %f", getName().c_str(), prev.t, next.t);

	for (Configspace::Index i = getStateInfo().getJoints().begin(); i < getStateInfo().getJoints().end(); ++i) {
		GenCoordTrj trj;
		trj.set2pvpv(prev.t, next.t, prev.cpos[i], prev.cvel[i], next.cpos[i], next.cvel[i]);

		Real tmin, tmax, min, max;
		
		trj.getPositionExtrema(tmin, tmax, min, max);
		assertLimit(min > assertMin.cpos[i], "position", i, min, tmin, assertMin.cpos[i], assertMax.cpos[i], prev, next);
		assertLimit(max < assertMax.cpos[i], "position", i, max, tmax, assertMin.cpos[i], assertMax.cpos[i], prev, next);
		
		trj.getVelocityExtrema(tmin, tmax, min, max);
		assertLimit(min > assertMin.cvel[i], "velocity", i, min, tmin, assertMin.cvel[i], assertMax.cvel[i], prev, next);
		assertLimit(max < assertMax.cvel[i], "velocity", i, max, tmax, assertMin.cvel[i], assertMax.cvel[i], prev, next);

		trj.getAccelerationExtrema(tmin, tmax, min, max);
		assertLimit(min > assertMin.cacc[i], "acceleration", i, min, tmin, assertMin.cacc[i], assertMax.cacc[i], prev, next);
		assertLimit(max < assertMax.cacc[i], "acceleration", i, max, tmax, assertMin.cacc[i], assertMax.cacc[i], prev, next);
	}
}

void SingleCtrl::initControlCycle() {
	CriticalSectionWrapper csw(csCycle);
	cycleInitRequest = true;
}

bool SingleCtrl::setCallbackIO(CallbackIO* pCallbackIO, MSecTmU32 timeWait) {
	if (bTerminate)
		return false;
	{
		CriticalSectionWrapper csw(csCycle);
		this->pNewCallbackIO = pCallbackIO != NULL ? pCallbackIO : &defaultCallbackIO;
		evCallbackIO.set(false);
	}
	return evCallbackIO.wait(timeWait);
}

//------------------------------------------------------------------------------
