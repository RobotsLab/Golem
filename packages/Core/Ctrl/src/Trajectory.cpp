/** @file Trajectory.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/Trajectory.h>
#include <Golem/Sys/Message.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Trajectory::Trajectory() {
}

void Trajectory::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgTrajectoryInvalidDesc(Message::LEVEL_CRIT, "Trajectory::create(): Invalid description");

	position.pTrajectory = this;
	velocity.pTrajectory = this;
	acceleration.pTrajectory = this;

	begin = desc.begin;
	end = desc.end;
	
	root = desc.root;
	extremum = desc.extremum;
	derivative = desc.derivative;
}

//------------------------------------------------------------------------------

Real Trajectory::getLength() const {
	return getPosition(end) - getPosition(begin);
}

bool Trajectory::getPositionInverse(Real &t, Real position) const {
	return root->findSingle(t, position, this->position, begin, end);
}

void Trajectory::getPositionExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const {
	extremum->findGlobal(tmin, min, position, begin, end, true);
	extremum->findGlobal(tmax, max, position, begin, end, false);
}

Real Trajectory::getVelocity(Real t) const {
	return derivative->findFirst(t, position);
}

bool Trajectory::getVelocityInverse(Seq &t, Real velocity) const {
	return root->findMultiple(t, velocity, this->velocity, begin, end);
}

void Trajectory::getVelocityExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const {
	extremum->findGlobal(tmin, min, velocity, begin, end, true);
	extremum->findGlobal(tmax, max, velocity, begin, end, false);
}

Real Trajectory::getAcceleration(Real t) const {
	return derivative->findSecond(t, position);
}

bool Trajectory::getAccelerationInverse(Seq &t, Real acceleration) const {
	return root->findMultiple(t, acceleration, this->acceleration, begin, end);
}

void Trajectory::getAccelerationExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const {
	extremum->findGlobal(tmin, min, acceleration, begin, end, true);
	extremum->findGlobal(tmax, max, acceleration, begin, end, false);
}

//------------------------------------------------------------------------------

void Polynomial1::create(const Desc& desc) {
	Trajectory::create(desc); // throws

	length = desc.length;
}

//------------------------------------------------------------------------------

void Polynomial3::create(const Desc& desc) {
	Trajectory::create(desc); // throws

	a[0] = desc.a[0];
	a[1] = desc.a[1];
	a[2] = desc.a[2];
}

void Polynomial3::scaleDuration(Real duration) {
	const Real t0 = getBegin();
	const Real t1 = getEnd();
	set2pvp(
		t0,
		t0 + duration,
		Polynomial3::getPosition(t0), Polynomial3::getVelocity(t0),
		Polynomial3::getPosition(t1)
		);
}

void Polynomial3::scaleLength(Real length) {
	const Real t0 = getBegin();
	const Real t1 = getEnd();
	const Real p0 = Polynomial3::getPosition(t0);
	set2pvp(
		t0,
		t1,
		p0, Polynomial3::getVelocity(t0),
		p0 + length
		);
}

Real Polynomial3::getPosition(Real t) const {
	t -= getBegin();
	return a[0] + a[1] * t + a[2] * t*t;
}

void Polynomial3::getPositionExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const {
	tmin = getBegin();
	tmax = getEnd();
	min = Polynomial3::getPosition(tmin);
	max = Polynomial3::getPosition(tmax);
	if (min > max) {
		std::swap(tmin, tmax);
		std::swap(min, max);
	}

	// TODO
}

Real Polynomial3::getVelocity(Real t) const {
	t -= getBegin();
	return a[1] + Real(2.0)*a[2]*t;
}

void Polynomial3::getVelocityExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const {
	tmin = getBegin();
	tmax = getEnd();
	min = Polynomial3::getVelocity(tmin);
	max = Polynomial3::getVelocity(tmax);
	if (min > max) {
		std::swap(tmin, tmax);
		std::swap(min, max);
	}
}

Real Polynomial3::getAcceleration(Real t) const {
	return Real(2.0)*a[2];
}

void Polynomial3::getAccelerationExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const {
	tmin = getBegin();
	tmax = getEnd();
	min = Polynomial3::getAcceleration(tmin);
	max = Polynomial3::getAcceleration(tmax);
	if (min > max) {
		std::swap(tmin, tmax);
		std::swap(min, max);
	}
}

void Polynomial3::set2pvp(Real t0, Real t1, Real p0, Real v0, Real p1) {
	setBegin(t0);
	setEnd(t1);
	t0 -= getBegin();
	t1 -= getBegin();

	Real t0d, t1d;

	Real m1 = p1 - p0 + v0*(t0 - t1);

	t0d = t0*t0; t1d = t1*t1;
	Real m2 = t1d + t0d - Real(2.0)*t0*t1;

	t1d *= t1;
	Real m3 = t1d + t0d*(Real(2.0)*t0 - Real(3.0)*t1);

	a[2] = m1 / m2;
	a[1] = v0 - Real(2.0)*a[2] * t0;
	a[0] = p0 - a[1] * t0 - a[2] * t0*t0;
}

void Polynomial3::set2ppv(Real t0, Real t1, Real p0, Real p1, Real v1) {
	// TODO

	//setBegin(t1);
	//setEnd(t2);
	//t0 -= getBegin();
	//t1 -= getBegin();
	//t2 -= getBegin();

	//Real t0d, t1d, t2d;

	//Real m1 = p0 - p2 + v2*(t2 - t0), n1 = p1 - p2 + v2*(t2 - t1);

	//t0d = t0*t0; t1d = t1*t1; t2d = t2*t2;
	//Real m2 = t0d + t2d - Real(2.0)*t2*t0, n2 = t1d + t2d - Real(2.0)*t2*t1;

	//t0d *= t0; t1d *= t1;
	//Real m3 = t0d + t2d*(Real(2.0)*t2 - Real(3.0)*t0), n3 = t1d + t2d*(Real(2.0)*t2 - Real(3.0)*t1);

	//a[3] = (n1*m2 - m1*n2) / (n3*m2 - m3*n2);
	//a[2] = (m1 - a[3] * m3) / m2;
	//a[1] = v2 - Real(2.0)*a[2] * t2 - Real(3.0)*a[3] * t2*t2;
	//a[0] = p0 - a[1] * t0 - a[2] * t0*t0 - a[3] * t0*t0*t0;
}

void Polynomial3::set3ppp(Real t0, Real t1, Real t2, Real p0, Real p1, Real p2) {
	setBegin(t0);
	setEnd(t2);
	t0 -= getBegin();
	t1 -= getBegin();
	t2 -= getBegin();

	Real t0d, t1d, t2d;
	Real t10 = t1 - t0, t20 = t2 - t0;

	Real mn1 = (p1 - p0) / t10;
	Real m1 = (p2 - p0) / t20 - mn1;

	t0d = t0*t0; t1d = t1*t1; t2d = t2*t2;
	Real mn2 = (t1d - t0d) / t10;
	Real m2 = (t2d - t0d) / t20 - mn2;

	t0d *= t0; t1d *= t1; t2d *= t2;

	a[2] = m1 / m2;
	a[1] = mn1 - a[2] * mn2;
	a[0] = p0 - a[1] * t0 - a[2] * t0*t0;
}

//------------------------------------------------------------------------------

void Polynomial4::create(const Desc& desc) {
	Trajectory::create(desc); // throws
	
	a[0] = desc.a[0];
	a[1] = desc.a[1];
	a[2] = desc.a[2];
	a[3] = desc.a[3];
}

void Polynomial4::scaleDuration(Real duration) {
	const Real t0 = getBegin();
	const Real t1 = getEnd();
	set2pvpv(
		t0,
		t0 + duration,
		Polynomial4::getPosition(t0), Polynomial4::getVelocity(t0),
		Polynomial4::getPosition(t1), Polynomial4::getVelocity(t1)
	);
}

void Polynomial4::scaleLength(Real length) {
	const Real t0 = getBegin();
	const Real t1 = getEnd();
	const Real p0 = Polynomial4::getPosition(t0);
	set2pvpv(
		t0,
		t1,
		p0, Polynomial4::getVelocity(t0),
		p0 + length, Polynomial4::getVelocity(t1)
	);
}

Real Polynomial4::getPosition(Real t) const {
	t -= getBegin();
	return a[0] + a[1]*t + a[2]*t*t + a[3]*t*t*t;
}

void Polynomial4::getPositionExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const {
	tmin = getBegin();
	tmax = getEnd();
	min = Polynomial4::getPosition(tmin);
	max = Polynomial4::getPosition(tmax);
	if (min > max) {
		std::swap(tmin, tmax);
		std::swap(min, max);
	}
	
	// TODO
}

Real Polynomial4::getVelocity(Real t) const {
	t -= getBegin();
	return a[1] + Real(2.0)*a[2]*t + Real(3.0)*a[3]*t*t;
}

void Polynomial4::getVelocityExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const {
	tmin = getBegin();
	tmax = getEnd();
	min = Polynomial4::getVelocity(tmin);
	max = Polynomial4::getVelocity(tmax);
	if (min > max) {
		std::swap(tmin, tmax);
		std::swap(min, max);
	}
	
	if (Math::equals(a[3], REAL_ZERO, REAL_EPS))
		return;
	Real t = - a[2]/(Real(3.0)*a[3]) + getBegin();
	if (t < getBegin() || t > getEnd())
		return;

	if (a[3] > Real(0.0)) {
		tmin = t;
		min = Polynomial4::getVelocity(tmin);
	}
	else {
		tmax = t;
		max = Polynomial4::getVelocity(tmax);
	}
}

Real Polynomial4::getAcceleration(Real t) const {
	t -= getBegin();
	return Real(2.0)*a[2] + Real(6.0)*a[3]*t;
}

void Polynomial4::getAccelerationExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const {
	tmin = getBegin();
	tmax = getEnd();
	min = Polynomial4::getAcceleration(tmin);
	max = Polynomial4::getAcceleration(tmax);
	if (min > max) {
		std::swap(tmin, tmax);
		std::swap(min, max);
	}
}

void Polynomial4::set2pvpv(Real t0, Real t1, Real p0, Real v0, Real p1, Real v1) {
	setBegin(t0);
	setEnd(t1);
	t0 -= getBegin();
	t1 -= getBegin();
	
	Real t10 = t1 - t0, t0d = t0*t0, t1d = t1*t1;

	Real mn1 = (p1 - p0)/t10;
	Real m1 = v0 - mn1, n1 = v1 - mn1;

	Real mn2 = (t1d - t0d)/t10;
	Real m2 = Real(2.0)*t0 - mn2, n2 = Real(2.0)*t1 - mn2;
	
	Real mn3 = (t1d*t1 - t0d*t0)/t10;
	Real m3 = Real(3.0)*t0d - mn3, n3 = Real(3.0)*t1d - mn3;

	a[3] = (n1*m2 - m1*n2)/(n3*m2 - m3*n2);
	a[2] = (m1 - a[3]*m3)/m2;
	a[1] = v0 - Real(2.0)*a[2]*t0 - Real(3.0)*a[3]*t0*t0;
	a[0] = p0 - a[1]*t0 - a[2]*t0*t0 - a[3]*t0*t0*t0;
}

void Polynomial4::set2pvav(Real t0, Real t1, Real p0, Real v0, Real a0, Real v1) {
	setBegin(t0);
	setEnd(t1);
	t0 -= getBegin();
	t1 -= getBegin();
	
	Real t10 = t1 - t0;
	Real t0d = t0*t0, t1d = t1*t1;

	a[3] = ((v1 - v0) - a0*t10)/(Real(3.0)*(t1d - t0d) - Real(6.0)*t0*t10);
	a[2] = Real(0.5)*a0 - Real(3.0)*a[3]*t0;
	a[1] = v0 - Real(2.0)*a[2]*t0 - Real(3.0)*a[3]*t0d;
	a[0] = p0 - a[1]*t0 - a[2]*t0d - a[3]*t0d*t0;
}

void Polynomial4::set3pvpp(Real t0, Real t1, Real t2, Real p0, Real v0, Real p1, Real p2) {
	setBegin(t0);
	setEnd(t1);
	t0 -= getBegin();
	t1 -= getBegin();
	t2 -= getBegin();
	
	Real t0d, t1d, t2d;

	Real m1 = p1 - p0 + v0*(t0 - t1), n1 = p2 - p0 + v0*(t0 - t2);
	
	t0d = t0*t0; t1d = t1*t1; t2d = t2*t2;
	Real m2 = t1d + t0d - Real(2.0)*t0*t1, n2 = t2d + t0d - Real(2.0)*t0*t2;
	
	t1d *= t1; t2d *= t2;
	Real m3 = t1d + t0d*(Real(2.0)*t0 - Real(3.0)*t1), n3 = t2d + t0d*(Real(2.0)*t0 - Real(3.0)*t2);

	a[3] = (n1*m2 - m1*n2)/(n3*m2 - m3*n2);
	a[2] = (m1 - a[3]*m3)/m2;
	a[1] = v0 - Real(2.0)*a[2]*t0 - Real(3.0)*a[3]*t0*t0;
	a[0] = p0 - a[1]*t0 - a[2]*t0*t0 - a[3]*t0*t0*t0;
}

void Polynomial4::set3pppv(Real t0, Real t1, Real t2, Real p0, Real p1, Real p2, Real v2) {
	setBegin(t1);
	setEnd(t2);
	t0 -= getBegin();
	t1 -= getBegin();
	t2 -= getBegin();
	
	Real t0d, t1d, t2d;

	Real m1 = p0 - p2 + v2*(t2 - t0), n1 = p1 - p2 + v2*(t2 - t1);
	
	t0d = t0*t0; t1d = t1*t1; t2d = t2*t2;
	Real m2 = t0d + t2d - Real(2.0)*t2*t0, n2 = t1d + t2d - Real(2.0)*t2*t1;
	
	t0d *= t0; t1d *= t1;
	Real m3 = t0d + t2d*(Real(2.0)*t2 - Real(3.0)*t0), n3 = t1d + t2d*(Real(2.0)*t2 - Real(3.0)*t1);

	a[3] = (n1*m2 - m1*n2)/(n3*m2 - m3*n2);
	a[2] = (m1 - a[3]*m3)/m2;
	a[1] = v2 - Real(2.0)*a[2]*t2 - Real(3.0)*a[3]*t2*t2;
	a[0] = p0 - a[1]*t0 - a[2]*t0*t0 - a[3]*t0*t0*t0;
}

void Polynomial4::set4pppp(Real t0, Real t1, Real t2, Real t3, Real p0, Real p1, Real p2, Real p3) {
	setBegin(t1);
	setEnd(t2);
	t0 -= getBegin();
	t1 -= getBegin();
	t2 -= getBegin();
	t3 -= getBegin();
	
	Real t0d, t1d, t2d, t3p;
	Real t10 = t1 - t0, t20 = t2 - t0, t30 = t3 - t0;
	
	Real mn1 = (p1 - p0)/t10;
	Real m1 = (p2 - p0)/t20 - mn1, n1 = (p3 - p0)/t30 - mn1;
	
	t0d = t0*t0; t1d = t1*t1; t2d = t2*t2; t3p = t3*t3;
	Real mn2 = (t1d - t0d)/t10;
	Real m2 = (t2d - t0d)/t20 - mn2, n2 = (t3p - t0d)/t30 - mn2;
	
	t0d *= t0; t1d *= t1; t2d *= t2; t3p *= t3;
	Real mn3 = (t1d - t0d)/t10;
	Real m3 = (t2d - t0d)/t20 - mn3, n3 = (t3p - t0d)/t30 - mn3;

	a[3] = (n1*m2 - m1*n2)/(n3*m2 - m3*n2);
	a[2] = (m1 - a[3]*m3)/m2;
	a[1] = mn1 - a[2]*mn2 - a[3]*mn3;
	a[0] = p0 - a[1]*t0 - a[2]*t0*t0 - a[3]*t0*t0*t0;
}

//------------------------------------------------------------------------------

