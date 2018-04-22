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
#ifndef _GOLEM_TINY_TYPES_H
#define _GOLEM_TINY_TYPES_H

//------------------------------------------------------------------------------

#include <Golem/Tiny/Tiny.h>
#include <Golem/Math/Bounds.h>
#include <Golem/Ctrl/Katana/Katana.h>

//------------------------------------------------------------------------------

namespace golem {
namespace tiny {

//------------------------------------------------------------------------------

inline Bounds::Type make(tiny::ShapeType type) {
	return static_cast<Bounds::Type>(type);
}

inline tiny::ShapeType make(Bounds::Type type) {
	return static_cast<tiny::ShapeType>(type);
}

inline golem::RGBA make(const tiny::RGBA& inp) {
	golem::RGBA out;
	out._rgba.r = (U8)Math::round(float(255.0)*inp.r);
	out._rgba.g = (U8)Math::round(float(255.0)*inp.g);
	out._rgba.b = (U8)Math::round(float(255.0)*inp.b);
	out._rgba.a = (U8)Math::round(float(255.0)*inp.a);
	return out;
}

inline tiny::RGBA make(const golem::RGBA& inp) {
	tiny::RGBA out;
	out.r = float(inp._rgba.r)/float(255.0);
	out.g = float(inp._rgba.g)/float(255.0);
	out.b = float(inp._rgba.b)/float(255.0);
	out.a = float(inp._rgba.a)/float(255.0);
	return out;
}

//------------------------------------------------------------------------------

inline golem::Jacobian make(const Configspace::Range& range, const tiny::Jacobian& inp) {
	golem::Jacobian out;
	out.set(inp.c, inp.c + range.size(), *range.begin());
	return out;
}

inline tiny::Jacobian make(const Configspace::Range& range, const golem::Jacobian& inp) {
	tiny::Jacobian out;
	inp.get(out.c, out.c + range.size(), *range.begin());
	return out;
}

//------------------------------------------------------------------------------

inline golem::ChainspaceCoord make(const Chainspace::Range& range, const tiny::ChainspaceCoord& inp)  {
	golem::ChainspaceCoord out;
	out.set(inp.c, inp.c + range.size(), *range.begin());
	return out;
}

inline tiny::ChainspaceCoord make(const Chainspace::Range& range, const golem::ChainspaceCoord& inp)  {
	tiny::ChainspaceCoord out;
	inp.get(out.c, out.c + range.size(), *range.begin());
	return out;
}

inline golem::ConfigspaceCoord make(const Configspace::Range& range, const tiny::ConfigspaceCoord& inp)  {
	golem::ConfigspaceCoord out;
	out.set(inp.c, inp.c + range.size(), *range.begin());
	return out;
}

inline tiny::ConfigspaceCoord make(const Configspace::Range& range, const golem::ConfigspaceCoord& inp)  {
	tiny::ConfigspaceCoord out;
	inp.get(out.c, out.c + range.size(), *range.begin());
	return out;
}

inline golem::GenConfigspaceState make(const Configspace::Range& range, const tiny::GenConfigspaceState& inp)  {
	golem::GenConfigspaceState out;
	out.cpos = make(range, inp.pos);
	out.cvel = make(range, inp.vel);
	out.cacc.fill(range.begin(), range.end(), REAL_ZERO);
	out.t = (SecTmReal)inp.t;
	return out;
}

inline tiny::GenConfigspaceState make(const Configspace::Range& range, const golem::GenConfigspaceState& inp)  {
	tiny::GenConfigspaceState out;
	out.pos = make(range, inp.cpos);
	out.vel = make(range, inp.cvel);
	out.t = (double)inp.t;
	return out;
}

inline golem::Controller::State make(const golem::Controller& controller, const tiny::GenConfigspaceState& inp)  {
	golem::Controller::State out = controller.createState();
	controller.setToDefault(out);
	out.cpos = make(controller.getStateInfo().getJoints(), inp.pos);
	out.cvel = make(controller.getStateInfo().getJoints(), inp.vel);
	out.cacc.fill(controller.getStateInfo().getJoints().begin(), controller.getStateInfo().getJoints().end(), REAL_ZERO);
	out.t = (SecTmReal)inp.t;
	return out;
}

inline tiny::GenConfigspaceState make(const golem::Controller::State& inp)  {
	tiny::GenConfigspaceState out;
	out.pos = make(inp.getInfo().getJoints(), inp.cpos);
	out.vel = make(inp.getInfo().getJoints(), inp.cvel);
	out.t = (double)inp.t;
	return out;
}

//------------------------------------------------------------------------------

inline golem::WorkspaceChainCoord make(const Chainspace::Range& range, const tiny::WorkspaceChainCoord& inp)  {
	golem::WorkspaceChainCoord out;
	out.set(inp.c, inp.c + range.size(), *range.begin());
	return out;
}

inline tiny::WorkspaceChainCoord make(const Chainspace::Range& range, const golem::WorkspaceChainCoord& inp)  {
	tiny::WorkspaceChainCoord out;
	inp.get(out.c, out.c + range.size(), *range.begin());
	return out;
}

inline golem::WorkspaceJointCoord make(const Configspace::Range& range, const tiny::WorkspaceJointCoord& inp)  {
	golem::WorkspaceJointCoord out;
	out.set(inp.c, inp.c + range.size(), *range.begin());
	return out;
}

inline tiny::WorkspaceJointCoord make(const Configspace::Range& range, const golem::WorkspaceJointCoord& inp)  {
	tiny::WorkspaceJointCoord out;
	inp.get(out.c, out.c + range.size(), *range.begin());
	return out;
}

inline golem::GenWorkspaceChainState make(const Chainspace::Range& range, const tiny::GenWorkspaceChainState& inp) {
	golem::GenWorkspaceChainState out;
	out.wpos = make(range, inp.pos);
	out.t = (SecTmReal)inp.t;
	return out;
}

tiny::GenWorkspaceChainState make(const Chainspace::Range& range, const golem::GenWorkspaceChainState& inp) {
	tiny::GenWorkspaceChainState out;
	out.pos = make(range, inp.wpos);
	out.t = (double)inp.t;
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

inline golem::KatanaGripper::SensorData make(const tiny::KatanaSensorData& inp) {
	golem::KatanaGripper::SensorData out;
	out.index = inp.index;
	out.value = inp.value;
	return out;
}

inline tiny::KatanaSensorData make(const golem::KatanaGripper::SensorData& inp) {
	tiny::KatanaSensorData out;
	out.index = inp.index;
	out.value = inp.value;
	return out;
}

inline golem::KatanaGripper::GripperEncoderData make(const tiny::KatanaGripperEncoderData& inp) {
	golem::KatanaGripper::GripperEncoderData out;
	out.open = inp.open;
	out.closed = inp.closed;
	out.current = inp.current;
	return out;
}

inline tiny::KatanaGripperEncoderData make(const golem::KatanaGripper::GripperEncoderData& inp) {
	tiny::KatanaGripperEncoderData out;
	out.open = inp.open;
	out.closed = inp.closed;
	out.current = inp.current;
	return out;
}

inline golem::KatanaGripper::SensorDataSet make(const tiny::KatanaSensorDataSet& inp) {
	golem::KatanaGripper::SensorDataSet out;
	for (tiny::KatanaSensorDataSet::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

inline tiny::KatanaSensorDataSet make(const golem::KatanaGripper::SensorDataSet& inp) {
	tiny::KatanaSensorDataSet out;
	for (golem::KatanaGripper::SensorDataSet::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

#endif /*_GOLEM_TINY_TYPES_H*/
