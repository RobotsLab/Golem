/** @file Types.h
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
#ifndef _GOLEM_TINYICE_TYPES_H_
#define _GOLEM_TINYICE_TYPES_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Bounds.h>
#include <Golem/App/UIPlanner.h>
#include <Golem/Ctrl/Katana/Katana.h>
#include <Golem/TinyIce/TinyIceI.h>

//------------------------------------------------------------------------------

namespace golem {
namespace tinyice {

//------------------------------------------------------------------------------

inline Bounds::Type make(tinyice::ShapeType type) {
	return static_cast<Bounds::Type>(type);
}

inline tinyice::ShapeType make(Bounds::Type type) {
	return static_cast<tinyice::ShapeType>(type);
}

inline golem::RGBA make(const tinyice::RGBA& inp) {
	golem::RGBA out;
	out._rgba.r = (U8)Math::round(float(255.0)*inp.r);
	out._rgba.g = (U8)Math::round(float(255.0)*inp.g);
	out._rgba.b = (U8)Math::round(float(255.0)*inp.b);
	out._rgba.a = (U8)Math::round(float(255.0)*inp.a);
	return out;
}

inline tinyice::RGBA make(const golem::RGBA& inp) {
	tinyice::RGBA out;
	out.r = float(inp._rgba.r)/float(255.0);
	out.g = float(inp._rgba.g)/float(255.0);
	out.b = float(inp._rgba.b)/float(255.0);
	out.a = float(inp._rgba.a)/float(255.0);
	return out;
}

//------------------------------------------------------------------------------

inline tinyice::Vec3 make(const golem::Vec3& v) {
	tinyice::Vec3 iv;
	v.getColumn3(&iv.v1);
	return iv;
}

inline golem::Vec3 make(const tinyice::Vec3& iv) {
	return golem::Vec3(iv.v1, iv.v2, iv.v3);
}

inline tinyice::Mat33 make(const golem::Mat33& m) {
	tinyice::Mat33 im;
	m.getRow33(&im.m11);
	return im;
}

inline golem::Mat33 make(const tinyice::Mat33& im) {
	golem::Mat33 m;
	m.setRow33(&im.m11);
	return m;
}

inline tinyice::Mat34 make(const golem::Mat34& m) {
	tinyice::Mat34 im;
	im.R = make(m.R);
	im.p = make(m.p);
	return im;
}

inline golem::Mat34 make(const tinyice::Mat34& im) {
	return golem::Mat34(make(im.R), make(im.p));
}

inline std::vector<golem::Mat34> make(const tinyice::Mat34Seq& inp)  {
	std::vector<golem::Mat34> out;
	for (tinyice::Mat34Seq::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

inline tinyice::Mat34Seq make(const std::vector<golem::Mat34>& inp)  {
	tinyice::Mat34Seq out;
	for (std::vector<golem::Mat34>::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

inline tinyice::Twist make(const golem::Twist& t) {
	tinyice::Twist it;
	it.v = make(t.v);
	it.w = make(t.w);
	return it;
}

inline golem::Twist make(const tinyice::Twist& it) {
	return golem::Twist(make(it.v), make(it.w));
}

inline tinyice::Jacobian make(const Configspace::Range& range, const golem::Jacobian& inp) {
	tinyice::Jacobian out;
	out.j.resize(range.size());
	for (idx_t i = 0; i < range.size(); ++i)
		out.j[i] = make(inp[range.begin() + i]);
	return out;
}

inline golem::Jacobian make(const Configspace::Range& range, const tinyice::Jacobian& inp) {
	golem::Jacobian out;
	for (idx_t i = 0; i < range.size() && i < (idx_t)inp.j.size(); ++i)
		out[range.begin() + i] = make(inp.j[i]);
	return out;
}

//------------------------------------------------------------------------------

inline golem::ChainspaceCoord make(const Chainspace::Range& range, const tinyice::ChainspaceCoord& inp)  {
	golem::ChainspaceCoord out;
	out.set(inp.c.data(), inp.c.data() + std::min(inp.c.size(), (size_t)range.size()), *range.begin());
	return out;
}

inline tinyice::ChainspaceCoord make(const Chainspace::Range& range, const golem::ChainspaceCoord& inp) {
	tinyice::ChainspaceCoord out;
	out.c.insert(out.c.end(), inp.data() + *range.begin(), inp.data() + *range.end());
	return out;
}

inline golem::ConfigspaceCoord make(const Configspace::Range& range, const tinyice::ConfigspaceCoord& inp)  {
	golem::ConfigspaceCoord out;
	out.set(inp.c.data(), inp.c.data() + std::min(inp.c.size(), (size_t)range.size()), *range.begin());
	return out;
}

inline tinyice::ConfigspaceCoord make(const Configspace::Range& range, const golem::ConfigspaceCoord& inp) {
	tinyice::ConfigspaceCoord out;
	out.c.insert(out.c.end(), inp.data() + *range.begin(), inp.data() + *range.end());
	return out;
}

inline golem::GenConfigspaceState make(const Configspace::Range& range, const tinyice::GenConfigspaceState& inp)  {
	golem::GenConfigspaceState out;
	out.cpos = make(range, inp.pos);
	out.cvel = make(range, inp.vel);
	out.cacc.fill(range.begin(), range.end(), REAL_ZERO);
	out.t = (SecTmReal)inp.t;
	return out;
}

inline tinyice::GenConfigspaceState make(const Configspace::Range& range, const golem::GenConfigspaceState& inp) {
	tinyice::GenConfigspaceState out;
	out.pos = make(range, inp.cpos);
	out.vel = make(range, inp.cvel);
	out.t = (Ice::Double)inp.t;
	return out;
}

inline golem::Controller::State make(const golem::Controller& controller, const tinyice::GenConfigspaceState& inp)  {
	golem::Controller::State out = controller.createState();
	controller.setToDefault(out);
	out.cpos = make(controller.getStateInfo().getJoints(), inp.pos);
	out.cvel = make(controller.getStateInfo().getJoints(), inp.vel);
	out.cacc.fill(controller.getStateInfo().getJoints().begin(), controller.getStateInfo().getJoints().end(), REAL_ZERO);
	out.t = (SecTmReal)inp.t;
	return out;
}

inline tinyice::GenConfigspaceState make(const golem::Controller::State& inp)  {
	tinyice::GenConfigspaceState out;
	out.pos = make(inp.getInfo().getJoints(), inp.cpos);
	out.vel = make(inp.getInfo().getJoints(), inp.cvel);
	out.t = (double)inp.t;
	return out;
}

//------------------------------------------------------------------------------

inline golem::WorkspaceChainCoord make(const Chainspace::Range& range, const tinyice::WorkspaceCoord& inp) {
	golem::WorkspaceChainCoord out;
	for (idx_t i = 0; i < range.size() && i < (idx_t)inp.c.size(); ++i)
		out[range.begin() + i] = make(inp.c[i]);
	return out;
}

inline tinyice::WorkspaceCoord make(const Chainspace::Range& range, const golem::WorkspaceChainCoord& inp) {
	tinyice::WorkspaceCoord out;
	out.c.resize(range.size());
	for (idx_t i = 0; i < range.size(); ++i)
		out.c[i] = make(inp[range.begin() + i]);
	return out;
}

inline golem::WorkspaceJointCoord make(const Configspace::Range& range, const tinyice::WorkspaceCoord& inp) {
	golem::WorkspaceJointCoord out;
	for (idx_t i = 0; i < range.size() && i < (idx_t)inp.c.size(); ++i)
		out[range.begin() + i] = make(inp.c[i]);
	return out;
}

inline tinyice::WorkspaceCoord make(const Configspace::Range& range, const golem::WorkspaceJointCoord& inp) {
	tinyice::WorkspaceCoord out;
	out.c.resize(range.size());
	for (idx_t i = 0; i < range.size(); ++i)
		out.c[i] = make(inp[range.begin() + i]);
	return out;
}

inline golem::GenWorkspaceChainState make(const Chainspace::Range& range, const tinyice::GenWorkspaceState& inp) {
	golem::GenWorkspaceChainState out;
	out.wpos = make(range, inp.pos);
	out.t = (SecTmReal)inp.t;
	return out;
}

inline tinyice::GenWorkspaceState make(const Chainspace::Range& range, const golem::GenWorkspaceChainState& inp) {
	tinyice::GenWorkspaceState out;
	out.pos = make(range, inp.wpos);
	out.t = (Ice::Double)inp.t;
	return out;
}

//------------------------------------------------------------------------------

template <typename _Out, typename _Inp> _Out makeSeq(const _Inp& inp) {
	_Out out;
	for (typename _Inp::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

template <typename _Out, typename _Inp, typename _Range> _Out makeSeq(const _Range& range, const _Inp& inp)  {
	_Out out;
	for (typename _Inp::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(range, *i));
	return out;
}

template <typename _Out, typename _Inp> _Out makeSeq(const golem::Controller& controller, const _Inp& inp)  {
	_Out out;
	for (typename _Inp::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(controller, *i));
	return out;
}

//------------------------------------------------------------------------------

inline golem::KatanaGripper::SensorData make(const tinyice::KatanaSensorData& inp) {
	golem::KatanaGripper::SensorData out;
	out.index = inp.index;
	out.value = inp.value;
	return out;
}

inline tinyice::KatanaSensorData make(const golem::KatanaGripper::SensorData& inp) {
	tinyice::KatanaSensorData out;
	out.index = inp.index;
	out.value = inp.value;
	return out;
}

inline golem::KatanaGripper::GripperEncoderData make(const tinyice::KatanaGripperEncoderData& inp) {
	golem::KatanaGripper::GripperEncoderData out;
	out.open = inp.open;
	out.closed = inp.closed;
	out.current = inp.current;
	return out;
}

inline tinyice::KatanaGripperEncoderData make(const golem::KatanaGripper::GripperEncoderData& inp) {
	tinyice::KatanaGripperEncoderData out;
	out.open = inp.open;
	out.closed = inp.closed;
	out.current = inp.current;
	return out;
}

inline golem::KatanaGripper::SensorDataSet make(const tinyice::KatanaSensorDataSet& inp) {
	golem::KatanaGripper::SensorDataSet out;
	for (tinyice::KatanaSensorDataSet::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

inline tinyice::KatanaSensorDataSet make(const golem::KatanaGripper::SensorDataSet& inp) {
	tinyice::KatanaSensorDataSet out;
	for (golem::KatanaGripper::SensorDataSet::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

#endif /*_GOLEM_TINYICE_TYPES_H_*/
