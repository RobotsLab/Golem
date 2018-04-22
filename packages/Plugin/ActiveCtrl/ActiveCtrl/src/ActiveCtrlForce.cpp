/** @file ActiveCtrlForce.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/ActiveCtrl/ActiveCtrl/ActiveCtrlForce.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Sys/XMLData.h>
#include <algorithm>

using namespace golem;

//------------------------------------------------------------------------------

const char* golem::ActiveCtrlForce::ModeName[] = {
	"Active control OFF",
	"Active control ON",
	"Emergency mode",
};

//------------------------------------------------------------------------------

ActiveCtrlForce::ActiveCtrlForce(golem::SingleCtrl& controller) : controller(controller) {
}

ActiveCtrlForce::~ActiveCtrlForce() {
	(void)registerIO();
}

void ActiveCtrlForce::create(const Desc& desc) {
	desc.assertValid(Assert::Context("ActiveCtrlForce::Desc."));

	stateInfo = controller.getStateInfo();
	min = controller.getMin();
	max = controller.getMax();

	if (desc.filter.size() < dimensions() || desc.threshold.size() < dimensions())
		throw Message(Message::LEVEL_CRIT, "ActiveCtrlForce::create(): Invalid dimensionality of input parameters");
	if (desc.velocity.size() < (size_t)stateInfo.getJoints().size() || desc.gain.size() < (size_t)stateInfo.getJoints().size())
		throw Message(Message::LEVEL_CRIT, "ActiveCtrlForce::create(): Invalid dimensionality of output parameters");
	
	mode = MODE_DISABLED;
	startSteps = desc.startSteps;
	stopSteps = desc.stopSteps;
	step = -1;

	filter.assign(desc.filter.begin(), desc.filter.begin() + dimensions());
	threshold.assign(desc.threshold.begin(), desc.threshold.begin() + dimensions());
	
	forceInpSensor.assign(dimensions(), REAL_ZERO);
	forceInpSensorSeq.resize(dimensions());
	for (std::vector<RealSeq>::iterator i = forceInpSensorSeq.begin(); i != forceInpSensorSeq.end(); ++i)
		i->assign(startSteps, REAL_ZERO);

	forceInpOffset.assign(dimensions(), REAL_ZERO);
	forceInpCurrent.assign(dimensions(), REAL_ZERO);
	forceInp.assign(dimensions(), REAL_ZERO);
	forceInpBuff.assign(dimensions(), REAL_ZERO);

	// correct velocity limits
	for (Configspace::Index i = stateInfo.getJoints().begin(); i < stateInfo.getJoints().end(); ++i) {
		const Real velocity = desc.velocity[i - stateInfo.getJoints().begin()];
		min.cvel[i] = std::max(min.cvel[i], -velocity);
		max.cvel[i] = std::min(max.cvel[i], +velocity);
	}

	gain.assign(desc.gain.begin(), desc.gain.begin() + stateInfo.getJoints().size());
	forceOut.assign(stateInfo.getJoints().size(), REAL_ZERO);
	forceOutBuff.assign(stateInfo.getJoints().size(), REAL_ZERO);

	forceReader = desc.forceReader;
	actionFilter = desc.actionFilter;

	timeOut = desc.timeOut;

	if (!registerIO(&controller, timeOut))
		throw Message(Message::LEVEL_CRIT, "ActiveCtrlForce::create(): registerIO timeout! Make sure you are not running controller client.");
}

//------------------------------------------------------------------------------

void ActiveCtrlForce::sysRecv(SingleCtrl* ctrl, golem::Controller::State& state) {
	ioSysRecv(state);

	if (isActive() && isRegisteredIO())
		forceReader(state, forceInpSensor);
}

void ActiveCtrlForce::sysSend(SingleCtrl* ctrl, const Controller::State& prev, Controller::State& next, bool bSendPrev, bool bSendNext) {
	if (isActive() && isRegisteredIO()) {
		U32 mode;
		I32 step;

		{
			golem::CriticalSectionWrapper csw(csCallback);
			step = this->step < 0 ? -1 : --this->step;
			mode = this->mode;
		}

		// update array of forces
		if (mode != MODE_DISABLED && step >= 0) {
			for (size_t i = 0; i < dimensions(); ++i)
				forceInpSensorSeq[i][step] = forceInpSensor[i];
		}
		// initialisation
		if (mode != MODE_DISABLED && step == 0) {
			if (mode == MODE_EMERGENCY) {
				// previously used offset
				forceInpOffset.assign(dimensions(), REAL_ZERO);
				// sensory input overwritten with offset
				forceInpSensor = forceInpOffset;
			}
			else {
				// compute offset as median force, for each dimension separately
				for (size_t i = 0; i < dimensions(); ++i) {
					RealSeq& seq = forceInpSensorSeq[i];
					const size_t N = seq.size(), M = N/2;
					std::nth_element(seq.begin(), seq.begin() + M + 1, seq.end());
					forceInpOffset[i] = seq[M];
				}
			}
			// initialise filtered force
			forceInpCurrent.assign(dimensions(), REAL_ZERO);
		}
		// nothing to do
		else if (mode != MODE_DISABLED ? step > 0 : step < 0) {
			// TODO simulate latencies here
			goto FILTER;
		}

		// compute input force
		for (size_t i = 0; i < dimensions(); ++i) {
			// substract offset force
			const Real df = forceInpSensor[i] - forceInpOffset[i];
			// first order low-pass filter
			const Real a = filter[i];
			const Real f = forceInpCurrent[i] = a*df + (REAL_ONE - a)*forceInpCurrent[i];
			// input force with a threshold (not in disabled mode)
			forceInp[i] = mode != MODE_DISABLED && Math::abs(f) < threshold[i] ? REAL_ZERO : f;
		}

		// transform input forces to joints' torques
		transform(next, forceInp, forceOut);

		// update inp/out force buffer
		{
			golem::CriticalSectionWrapper csw(csData);
			if (mode != MODE_DISABLED || step > 1) {
				forceInpBuff = forceInp;
				forceOutBuff = forceOut;
			}
			else {
				forceInpBuff.assign(forceInp.size(), REAL_ZERO);
				forceOutBuff.assign(forceOut.size(), REAL_ZERO);
			}
		}

		// update target state
		for (Configspace::Index i = stateInfo.getJoints().begin(); i < stateInfo.getJoints().end(); ++i) {
			const idx_t j = i - stateInfo.getJoints().begin();

			// time delta
			const Real dt = (Real)(next.t - prev.t);

			// base position correction
			const Real ds = mode == MODE_EMERGENCY ? REAL_ZERO : gain[j] * forceOut[j];

			// base position reference: in emergency mode ignore all external updates (from next)
			const Real s = mode == MODE_EMERGENCY ? prev.cpos[i] : next.cpos[i];

			// update target position within velocity and position limits
			next.cpos[i] = Math::clamp(s + Math::clamp(ds, min.cvel[i]*dt, max.cvel[i]*dt), min.cpos[i], max.cpos[i]);

			// in emergency mode also clear velocities and accelerations
			if (mode == MODE_EMERGENCY) {
				next.cvel[i] = REAL_ZERO;
				next.cacc[i] = REAL_ZERO;
			}
		}

		FILTER:
		// action filter is always called
		if (actionFilter != nullptr) actionFilter(prev, next, bSendPrev, bSendNext);
	}

	ioSysSend(prev, next, bSendPrev, bSendNext);
}

size_t ActiveCtrlForce::dimensions() const {
	return (size_t)stateInfo.getJoints().size();
}

void ActiveCtrlForce::transform(const golem::Controller::State&, const RealSeq& inp, RealSeq& out) {
	out = inp;
}

ActiveCtrlForce::Mode ActiveCtrlForce::getMode() const {
	return mode;
}

void ActiveCtrlForce::setMode(Mode mode, golem::I32 steps) {
	{
		golem::CriticalSectionWrapper csw(csCallback);

		switch (mode) {
		case MODE_DISABLED:
			if (this->mode == MODE_EMERGENCY) {
				this->mode = MODE_DISABLED;
				step = steps == STEP_DEFAULT ? -1 : steps;
			}
			else if (step < 0) {
				this->mode = MODE_DISABLED;
				step = steps == STEP_DEFAULT ? (I32)stopSteps : steps;
			}
			break;
		case MODE_ENABLED:
			//if (this->mode != MODE_EMERGENCY) {
				this->mode = MODE_ENABLED;
				step = steps == STEP_DEFAULT ? (I32)startSteps : steps;
			//}
			break;
		case MODE_EMERGENCY:
			if (this->mode != MODE_EMERGENCY) {
				this->mode = MODE_EMERGENCY;
				step = steps == STEP_DEFAULT ? 1 : steps;
			}
			break;
		};
	}

	controller.getContext().info("%s::ActiveCtrlForce::setMode(): %s\n", controller.getName().c_str(), ModeName[this->mode]);
}

void ActiveCtrlForce::getForceInp(RealSeq& force) {
	golem::CriticalSectionWrapper csw(csData);
	force = forceInpBuff;
}

void ActiveCtrlForce::getForceOut(RealSeq& force) {
	golem::CriticalSectionWrapper csw(csData);
	force = forceOutBuff;
}

//------------------------------------------------------------------------------

void golem::XMLData(ActiveCtrlForce::Desc &val, XMLContext* context, bool create) {
	golem::XMLData("start_steps", val.startSteps, context, create);
	golem::XMLData("stop_steps", val.startSteps, context, create);

	golem::XMLData(val.filter, context->getContextFirst("filter"), create);
	golem::XMLData(val.threshold, context->getContextFirst("threshold"), create);
	golem::XMLData(val.gain, context->getContextFirst("gain"), create);
	golem::XMLData(val.velocity, context->getContextFirst("velocity"), create);

	golem::XMLData("time_out", val.timeOut, context);
}

//------------------------------------------------------------------------------

ActiveCtrlFT::ActiveCtrlFT(golem::SingleCtrl& controller) : ActiveCtrlForce(controller) {
	jacobian.resize(controller.getStateInfo().getJoints().size()); // before installing CallbackIO!
}

void ActiveCtrlFT::create(const Desc& desc) {
	ActiveCtrlForce::create(desc); // throws
}

size_t ActiveCtrlFT::dimensions() const {
	return 6;
}

void ActiveCtrlFT::transform(const golem::Controller::State& state, const RealSeq& inp, RealSeq& out) {
	const Chainspace::Index chainIdx = stateInfo.getChains().begin();
	const Chain* chain = controller.getChains()[chainIdx];
	const Configspace::Range jointRange = stateInfo.getJoints(chainIdx);
	const Configspace::Index jointIdx = jointRange.begin();
	const size_t jointSize = (size_t)jointRange.size();
	
	chain->jacobianBody(&state.cpos[jointIdx], jacobian.data());

	out.assign(jointSize, REAL_ZERO);
	for (size_t i = 0; i < jointSize; ++i) {
		const Twist* twist = &jacobian[i];
		
		Real theta = REAL_ZERO;
		theta += twist->v.x*inp[0];
		theta += twist->v.y*inp[1];
		theta += twist->v.z*inp[2];
		theta += twist->w.x*inp[3];
		theta += twist->w.y*inp[4];
		theta += twist->w.z*inp[5];
		out[i] = theta;
	}
}

//------------------------------------------------------------------------------

void golem::XMLData(ActiveCtrlFT::Desc &val, XMLContext* context, bool create) {
	XMLData((ActiveCtrlForce::Desc&)val, context, create);

}

//------------------------------------------------------------------------------
