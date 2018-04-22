/** @file JointCtrl.cpp
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/ActiveCtrl/JointCtrl/JointCtrl.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <Golem/Planner/Data.h>
#include <GL/glut.h>
#include <iomanip>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new JointCtrl::Desc();
}

//------------------------------------------------------------------------------

void JointCtrl::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	ActiveCtrl::Desc::load(context, xmlcontext);

	golem::XMLData("ctrl_thread_sleep", ctrlThreadSleep, const_cast<golem::XMLContext*>(xmlcontext), false);

	golem::XMLData(tolerance, xmlcontext->getContextFirst("tolerance"), false);
	golem::XMLData("velocity", velocity, xmlcontext->getContextFirst("profile"), false);
	golem::XMLData("acceleration", acceleration, xmlcontext->getContextFirst("profile"), false);
	golem::XMLData("duration", duration, xmlcontext->getContextFirst("profile"), false);
	golem::XMLData(increment, xmlcontext->getContextFirst("increment"), false);
	golem::XMLData("low", incrementLow, xmlcontext->getContextFirst("increment"), false);
	golem::XMLData(frameSize, xmlcontext->getContextFirst("frame_size"), false);
	golem::XMLData(frameSizeActive, xmlcontext->getContextFirst("frame_size_active"), false);

	incrementKeyMap.clear();
	try {
		golem::XMLData(incrementKeyMap, incrementKeyMap.max_size(), xmlcontext->getContextFirst("key_map"), "increment");
	}
	catch (const golem::MsgXMLParser&) {}
	targetKeyMap.clear();
	try {
		golem::XMLData(targetKeyMap, targetKeyMap.max_size(), xmlcontext->getContextFirst("key_map"), "target");
	}
	catch (const golem::MsgXMLParser&) {}

	golem::XMLData("simple_mode", simpleMode, const_cast<golem::XMLContext*>(xmlcontext));

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_JOINTCTRL_SFML
	try {
		inputCtrlDesc.load(context, xmlcontext);
	}
	catch (const golem::MsgXMLParser&) {}
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_JOINTCTRL_SFML
}

void golem::XMLData(golem::JointCtrl::KeyMap::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("key", const_cast<char&>(val.first), xmlcontext, create);
	golem::XMLData("joint", const_cast<U32&>(val.second.first), xmlcontext, create);
	golem::XMLData("value", val.second.second, xmlcontext, create);
}

//------------------------------------------------------------------------------

JointCtrl::JointCtrl(golem::Planner &planner, const Sensor::Seq& sensorSeq) : ActiveCtrl(planner, sensorSeq) {
}

void JointCtrl::create(const Desc& desc) {
	ActiveCtrl::create(desc); // throws

	ctrlThreadSleep = desc.ctrlThreadSleep;

	tolerance = desc.tolerance;
	velocity = desc.velocity;
	acceleration = desc.acceleration;
	duration = desc.duration;
	increment = desc.increment;
	incrementLow = desc.incrementLow;
	frameSize = desc.frameSize;
	frameSizeActive = desc.frameSizeActive;
	incrementKeyMap = desc.incrementKeyMap;
	targetKeyMap = desc.targetKeyMap;
	simpleMode = desc.simpleMode;
	
	joint = 0;
	incrementStep = 0;
	timeStamp = SEC_TM_REAL_ZERO;

	cmin = controller.getMin();
	cmax = controller.getMax();

	// control task
#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_JOINTCTRL_SFML
	try {
		inputCtrlPtr = desc.inputCtrlDesc.create(context);
		task.reset(new ThreadTask([&]() {
			try {
				typedef std::map<U32, bool> ActiveMap;
				ActiveMap activeMap;

				while (!task->isTerminating()) {
					Sleep::msleep(ctrlThreadSleep);
					if (!isActive() || !simpleMode && joint <= 0)
						continue;

					golem::ConfigspaceCoord target;
					target.setToDefault(info.getJoints());
					bool update = false;

					inputCtrlPtr->update();
					inputCtrlPtr->getJoystickButtonState([&](U32 index, U32 ctrl, bool value) {
						if (ctrl == 0)
							activeMap[index] = value;
						if (ctrl == 1 && value)
							context.notice("JointCtrl(): Joystick: robot MOVE\n");
						if (ctrl == 2 && value)
							context.notice("JointCtrl(): Joystick: robot STOP\n");
					});
					inputCtrlPtr->getJoystickAxisPosition([&](U32 index, U32 ctrl, Real value) {
						if (activeMap[index] && Math::abs(value) > REAL_ZERO) {
							const Configspace::Index j = info.getJoints().begin() + ctrl;
							const Real increment = getJointIncrement(info.getJoints().begin() + ctrl);
							target[j] = value * increment;
							update = true;
							if (joint == 0 || target[info.getJoints().begin() + joint - 1] < target[j])
								joint  = ctrl + 1;
						}
					});
					
					//inputCtrlPtr->testJoystick( );

					if (update)
						sendNext(target);
				}
			}
			catch (const Message& msg) {
				context.write(msg);
			}
		}));
		task->start();
	}
	catch (const Message& msg) {
		context.notice("JointCtrl::create(): Joystick input not available (%s)\n", msg.msg());
	}
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_JOINTCTRL_SFML

	setActive(desc.active);
}

//------------------------------------------------------------------------------

golem::RBDist JointCtrl::getJointIncrement() const {
	const Real increment = Math::pow(REAL_TWO, -Real(incrementStep));
	return golem::RBDist(increment*this->increment.lin, increment*this->increment.ang);
}

bool JointCtrl::isRevolute(golem::Configspace::Index index) const {
	return info.getJoints().contains(index) && controller.getJoints()[index]->getTrn().twist.w.magnitudeSqr() > REAL_EPS;
}

golem::Real JointCtrl::getJointIncrement(golem::Configspace::Index index) const {
	const golem::RBDist jointIncrement = getJointIncrement();
	return isRevolute(index) ? jointIncrement.ang : jointIncrement.lin;
}

golem::Real JointCtrl::getJointTolerance(golem::Configspace::Index index) const {
	return isRevolute(index) ? tolerance.ang : tolerance.lin;
}

void JointCtrl::sendNext(golem::ConfigspaceCoord& target, const ConfigspaceBool* positionMode) {
	// profile function
	auto profile = [=] (const SecTmReal duration, const GenCoord& limit, const GenCoord& current, const GenCoord& next) -> Real {
		GenCoordTrj trj;
		Real tmin, tmax, min, max, vel, acc, fvel, facc;

		trj.set(SEC_TM_REAL_ZERO, duration, current, next);
		trj.getVelocityExtrema(tmin, tmax, min, max);
		vel = std::max(Math::abs(min), Math::abs(max));
		trj.getAccelerationExtrema(tmin, tmax, min, max);
		acc = std::max(Math::abs(min), Math::abs(max));

		fvel = Math::abs(vel / (velocity*limit.vel));
		facc = Math::abs(acc / (acceleration*limit.acc));

		return std::max(fvel, Math::sqrt(facc));
	};

	// profiling
	const SecTmReal t = context.getTimer().elapsed() + controller.getCommandLatency() + controller.getCycleDuration();
	Controller::State current = controller.createState();
	controller.lookupCommand(t, current);
	Controller::State next = current;
	next.cvel.setToDefault(info.getJoints());
	next.cacc.setToDefault(info.getJoints());

	SecTmReal duration = this->duration;

	// time adaptation given fixed position
	for (Configspace::Index index = info.getJoints().begin(); index < info.getJoints().end(); ++index) {
		const Real tolerance = getJointTolerance(index);
		next.cpos[index] = Math::clamp(positionMode && (*positionMode)[index] ? target[index] : next.cpos[index] + target[index], cmin.cpos[index] + tolerance, cmax.cpos[index] - tolerance);

		const Real scale = profile(duration, GenCoord(cmax.cpos[index], cmax.cvel[index], cmax.cacc[index]), GenCoord(current.cpos[index], current.cvel[index], current.cacc[index]), GenCoord(next.cpos[index], next.cvel[index], next.cacc[index]));
		if (scale > REAL_ONE) duration *= scale;
	}

	next.t += duration;

	// sending
	try {
		controller.send(&next, &next + 1, true);
	}
	catch (const std::exception& ex) {
		context.write("%s\n", ex.what());
	}
	timeStamp = next.t;
}

//------------------------------------------------------------------------------

void JointCtrl::render() const {
	renderer.reset();
	if (joint > 0) {
		Controller::State state = controller.createState();
		controller.lookupState(SEC_TM_REAL_MAX, state);
		golem::WorkspaceJointCoord wjc;
		controller.jointForwardTransform(state.cpos, wjc);
		const Mat34 frame = wjc[info.getJoints().begin() + joint - 1];
		const Configspace::Index index = info.getJoints().begin() + joint - 1;
		const Twist twist = controller.getJoints()[index]->getTrn().twist;
		Vec3 anchor, axis;
		twist.getAxis(anchor, axis); // Z-axis

		// full frame
		Vec3 x = Vec3::axisX(), y = Vec3::axisY(), z = axis;
		z.normalise();
		if (!Math::equals(REAL_ONE, Math::abs(z.dot(x)), numeric_const<Real>::EPS)) {
			y.cross(z, x);
			x.cross(y, z);
		}
		else {
			x.cross(y, z);
			y.cross(z, x);
		}
		renderer.addAxes3D(frame * Mat34(Mat33(x, y, z), Vec3::zero()), timeStamp > context.getTimer().elapsed() ? frameSizeActive : frameSize);

		// Z-axis only
		//Vec3 p1 = frame.p, p2 = frame.R * axis;
		//p2.multiplyAdd(timeStamp > context.getTimer().elapsed() ? frameSizeActive.z : frameSize.z, p2, p1);
		//renderer.addAxis3D(p1, p2, RGBA(RGBA::BLUE._rgba.r, RGBA::BLUE._rgba.g, RGBA::BLUE._rgba.b, 127));
	}
	renderer.render();
}

void JointCtrl::keyboardHandler(int key, int x, int y) {
	switch (key) {
	case 32: // <space>
		if (simpleMode)
			return;
		if (joint == 0)
			return;
		joint = 0;
		break;
	case (GLUT_KEY_F7 | UIKeyboardMouseCallback::KEY_SPECIAL) : // F7
		if (simpleMode)
			return;
		joint = joint > 0 ? joint - 1 : (U32)info.getJoints().size();
		break;
	case (GLUT_KEY_F8 | UIKeyboardMouseCallback::KEY_SPECIAL) : // F8
		if (simpleMode)
			return;
		joint = joint < (U32)info.getJoints().size() ? joint + 1 : 0;
		break;
	default:
		if (simpleMode && (key == '+' || key == '-' || key == '9' || key == '0')) {
			incrementStep = (key == '+' || key == '9') ? 0 : incrementLow;
			const golem::RBDist jointIncrement = getJointIncrement();
			context.write("Joint control increment: position = %f [m], orientation = %f [deg]\n", jointIncrement.lin, Math::radToDeg(jointIncrement.ang));
		}
		else if (!(key & UIKeyboardMouseCallback::KEY_SPECIAL)) {
			bool update = false;
			golem::ConfigspaceCoord target;
			target.setToDefault(info.getJoints());
			for (KeyMap::const_iterator k = incrementKeyMap.begin(); k != incrementKeyMap.end(); ++k)
				if (k->first == (char)(key & UIKeyboardMouseCallback::KEY_MASK)) {
					const Configspace::Index index = info.getJoints().begin() + k->second.first - 1;
					target[index] = getJointIncrement(index) * k->second.second;
					update = true;
				}
			for (KeyMap::const_iterator k = targetKeyMap.begin(); k != targetKeyMap.end(); ++k)
				if (k->first == (char)(key & UIKeyboardMouseCallback::KEY_MASK)) {
					const Configspace::Index index = info.getJoints().begin() + k->second.first - 1;
					target[index] = k->second.second;
					update = true;
				}
			if (update)
				sendNext(target);
		}
		return;
	}

	if (joint > 0) {
		const Configspace::Index index = info.getJoints().begin() + joint - 1;
		context.write("%s - %s - %s - Joint #%u (%s): Joint control: Enabled\n", controller.getName().c_str(), controller.getJoints()[index]->getChain().getName().c_str(), controller.getJoints()[index]->getName().c_str(), joint, isRevolute(index) ? "revolute" : "prismatic");
	}
	else
		context.write("Joint control: Disabled\n");
};

void JointCtrl::mouseHandler(int button, int state, int x, int y) {
	if (simpleMode)
		return;

	const int mask = button & ~UIKeyboardMouseCallback::KEY_MASK;
	button &= UIKeyboardMouseCallback::KEY_MASK;

	if (joint == 0 || state != 0) {
	}
	else if ((mask & UIKeyboardMouseCallback::KEY_CTRL) && (button == 3 || button == 4)) {
		if (button == 3 && incrementStep > 0)
			--incrementStep;
		if (button == 4 && incrementStep < U32(Math::abs(golem::numeric_const<Real>::MIN_EXP)) - 1)
			++incrementStep;
		const golem::RBDist jointIncrement = getJointIncrement();
		context.write("Joint control increment: position = %f [m], orientation = %f [deg]\n", jointIncrement.lin, Math::radToDeg(jointIncrement.ang));
	}
	else if (button == GLUT_MIDDLE_BUTTON || button == 3 || button == 4) {
		const Configspace::Index index = info.getJoints().begin() + joint - 1;
		golem::ConfigspaceCoord target;
		target.setToDefault(info.getJoints());
		target[index] = button == 3 ? +getJointIncrement(index) : button == 4 ? -getJointIncrement(index) : REAL_ZERO;
		if (button == GLUT_MIDDLE_BUTTON) {
			ConfigspaceBool positionMode;
			positionMode.fill(info.getJoints(), false);
			positionMode[index] = true;
			sendNext(target, &positionMode);
		}
		else
			sendNext(target);
	}
}

void JointCtrl::motionHandler(int x, int y) {
}

//------------------------------------------------------------------------------
