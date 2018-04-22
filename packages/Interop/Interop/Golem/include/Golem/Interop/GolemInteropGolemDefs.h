/** @file GolemInteropGolemDefs.h
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
#ifndef _GOLEM_INTEROP_GOLEM_GOLEM_DEFS_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_GOLEM_GOLEM_DEFS_H_

#include "GolemInteropContactDefs.h"
#include <Golem/Ctrl/Controller.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Math/RB.h>
#include <Golem/Contact/Manipulator.h>
#include <Golem/Contact/Data.h>

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {
	template <typename _SrcType, size_t _SrcSize, typename _DstType, size_t _DstSize> void convert(const golem::_VecN<_SrcType, _SrcSize>& src, interop::Vec<_DstType, _DstSize>& dst, std::uint32_t& size) {
		//Assert::valid(_SrcSize == _DstSize, "interop::convert(): VecN dimensions mismatch");
		copy(src.v, src.v + _DstSize, dst.v);
		size = (std::uint32_t)src.size();
	}
	template <typename _SrcType, size_t _SrcSize, typename _DstType, size_t _DstSize> void convert(const interop::Vec<_SrcType, _SrcSize>& src, const std::uint32_t& size, golem::_VecN<_DstType, _DstSize>& dst) {
		//Assert::valid(_SrcSize == _DstSize, "interop::convert(): VecN dimensions mismatch");
		dst.resize((size_t)size);
		copy(src.v, src.v + _DstSize, dst.v);
	}

	template <> inline void convert(const golem::Vec3& src, interop::Vec3& dst) {
		src.getColumn3(dst.v);
	}
	template <> inline void convert(const interop::Vec3& src, golem::Vec3& dst) {
		dst.setColumn3(src.v);
	}
	template <typename _Src, typename _Dst> inline void convertQuat(const _Src& src, _Dst& dst) {
		convert(src.x, dst.x); convert(src.y, dst.y); convert(src.z, dst.z); convert(src.w, dst.w);
	}
	template <> inline void convert(const golem::Quat& src, interop::Quat& dst) {
		convertQuat(src, dst);
	}
	template <> inline void convert(const interop::Quat& src, golem::Quat& dst) {
		convertQuat(src, dst);
	}
	template <typename _Src, typename _Dst> inline void convertMat33(const _Src& src, _Dst& dst) {
		convert(src.m11, dst.m11); convert(src.m12, dst.m12); convert(src.m13, dst.m13);
		convert(src.m21, dst.m21); convert(src.m22, dst.m22); convert(src.m23, dst.m23);
		convert(src.m31, dst.m31); convert(src.m32, dst.m32); convert(src.m33, dst.m33);
	}
	template <> inline void convert(const golem::Mat34& src, interop::Mat34& dst) {
		convert(src.p, dst.p);
		convertMat33(src.R, dst.R);
	}
	template <> inline void convert(const interop::Mat34& src, golem::Mat34& dst) {
		convert(src.p, dst.p);
		convertMat33(src.R, dst.R);
	}
	template <typename _Src, typename _Dst> inline void convertDist(const _Src& src, _Dst& dst) {
		convert(src.lin, dst.lin); convert(src.ang, dst.ang);
	}
	template <> inline void convert(const golem::RBDist& src, interop::Dist& dst) {
		convertDist(src, dst);
	}
	template <> inline void convert(const interop::Dist& src, golem::RBDist& dst) {
		convertDist(src, dst);
	}
	template <typename _Src, typename _Dst> inline void convertFrame3D(const _Src& src, _Dst& dst) {
		convert(src.p, dst.p);
		convert(src.q, dst.q);
	}
	template <> inline void convert(const golem::RBCoord& src, interop::Frame3D& dst) {
		convertFrame3D(src, dst);
	}
	template <> inline void convert(const interop::Frame3D& src, golem::RBCoord& dst) {
		convertFrame3D(src, dst);
	}

	template <typename _Src, typename _Dst> inline void convertTwist(const _Src& src, _Dst& dst) {
		convert(src.v, dst.v);
		convert(src.w, dst.w);
	}
	template <> inline void convert(const golem::Twist& src, interop::Twist& dst) {
		convertTwist(src, dst);
	}
	template <> inline void convert(const interop::Twist& src, golem::Twist& dst) {
		convertTwist(src, dst);
	}

	inline void convert(const golem::ConfigspaceCoord& src, interop::ConfigspaceCoord& dst) {
		//Assert::valid((size_t)golem::Configspace::DIM == (size_t)interop::ConfigspaceCoord::SIZE, "interop::convert(): Configspace dimensions mismatch");
		copy(src.data(), src.data() + golem::Configspace::DIM, dst.v);
	}
	inline void convert(const interop::ConfigspaceCoord& src, golem::ConfigspaceCoord& dst) {
		//Assert::valid((size_t)golem::Configspace::DIM == (size_t)interop::ConfigspaceCoord::SIZE, "interop::convert(): Configspace dimensions mismatch");
		copy(src.v, src.v + interop::ConfigspaceCoord::SIZE, dst.data());
	}

	inline void convert(const golem::WorkspaceChainCoord& src, interop::WorkspaceCoord& dst) {
		//Assert::valid((size_t)golem::Chainspace::DIM == (size_t)interop::WorkspaceCoord::SIZE, "interop::convert(): Chainspace dimensions mismatch");
		copy(src.data(), src.data() + golem::Chainspace::DIM, dst.v);
	}
	inline void convert(const interop::WorkspaceCoord& src, golem::WorkspaceChainCoord& dst) {
		//Assert::valid((size_t)golem::Chainspace::DIM == (size_t)interop::WorkspaceCoord::SIZE, "interop::convert(): Chainspace dimensions mismatch");
		copy(src.v, src.v + interop::WorkspaceCoord::SIZE, dst.data());
	}

	inline void convert(const golem::Controller::State& src, interop::Config& dst) {
		convert(src.t, dst.t);
		convert(src.cpos, dst.cpos);
		convert(src.cvel, dst.cvel);
	}
	inline void convert(const interop::Config& src, golem::Controller::State& dst) {
		convert(src.t, dst.t);
		convert(src.cpos, dst.cpos);
		convert(src.cvel, dst.cvel);
	}

	template <typename _Real> inline void convert(const golem::Sample<_Real>& src, interop::Sample& dst) {
		convert(src.weight, dst.weight);
	}
	template <typename _Real> inline void convert(const interop::Sample& src, golem::Sample<_Real>& dst) {
		dst.setToDefault();
		convert(src.weight, dst.weight);
	}

	template <> inline void convert(const golem::RGBA& src, interop::RGBA& dst) {
		dst.r = static_cast<std::uint8_t>(src._rgba.r);
		dst.g = static_cast<std::uint8_t>(src._rgba.g);
		dst.b = static_cast<std::uint8_t>(src._rgba.b);
		dst.a = static_cast<std::uint8_t>(src._rgba.a);
	}
	template <> inline void convert(const interop::RGBA& src, golem::RGBA& dst) {
		dst._rgba.r = static_cast<golem::U8>(src.r);
		dst._rgba.g = static_cast<golem::U8>(src.g);
		dst._rgba.b = static_cast<golem::U8>(src.b);
		dst._rgba.a = static_cast<golem::U8>(src.a);
	}


	inline void convert(const golem::Controller::State::Seq& src, Config::Seq& dst) {
		dst.resize(src.size());
		golem::interop::copy(src.begin(), src.end(), dst.begin());
	}
	void convert(const golem::Controller& controller, const Config::Seq& src, golem::Controller::State::Seq& dst);

	/** Import trajectory from file */
	void loadTrajectory(const golem::Controller& controller, const std::string& path, Config::Seq& trajectory);

	inline void convert(const golem::ManifoldCtrl& src, Manifold& dst) {
		convert(src.frame, dst.frame);
		convert(src.frameDev, dst.frameDev);
	}
	inline void convert(const Manifold& src, golem::ManifoldCtrl& dst) {
		convert(src.frame, dst.frame);
		convert(src.frameDev, dst.frameDev);
	}

	inline void convert(const golem::Configuration::Kernel& src, ConfigModel& dst) {
		convert((const golem::Sample<golem::Real>&)src, (Sample&)dst);
		convert(src.config, dst.config);
		convert(src.frame, dst.frame);
	}
	inline void convert(const ConfigModel& src, golem::Configuration::Kernel& dst) {
		convert((const Sample&)src, (golem::Sample<golem::Real>&)dst);
		convert(src.config, dst.config);
		convert(src.frame, dst.frame);
	}

	inline void convert(const golem::Configuration::Path& src, PathModel& dst) {
		convert((const golem::Sample<golem::Real>&)src, (Sample&)dst);
		dst.resize(src.size());
		golem::interop::copy(src.begin(), src.end(), dst.begin());
	}
	inline void convert(const PathModel& src, golem::Configuration::Path& dst) {
		convert((const Sample&)src, (golem::Sample<golem::Real>&)dst);
		dst.resize(src.size());
		golem::interop::copy(src.begin(), src.end(), dst.begin());
	}

	void convert(const golem::Manipulator::Waypoint& src, ConfigModel& dst);
	void convert(const ConfigModel& src, golem::Manipulator::Waypoint& dst);

	void convert(const golem::Contact3D& src, ContactModel3D& dst);
	void convert(const ContactModel3D& src, golem::Contact3D& dst);

	void convert(const golem::Contact3D::Data& src, ContactModel3D::Data& dst);
	void convert(const ContactModel3D::Data& src, golem::Contact3D::Data& dst);

	void convert(const golem::Contact::Config& src, Path& dst);
	void convert(const Path& src, golem::Contact::Config& dst);

	inline void convert(const golem::Contact::Config::Ptr& src, Path& dst) {
		convert(*src, dst);
	}
	inline void convert(const Path& src, golem::Contact::Config::Ptr& dst) {
		dst.reset(new golem::Contact::Config());
		convert(src, *dst);
	}

	template <> void convert(const golem::Configuration::Space& src, ConfigSpace& dst);
	template <> void convert(const ConfigSpace& src, golem::Configuration::Space& dst);

	void convert(const golem::Contact::View& src, ContactView3D& dst);
	void convert(const ContactView3D& src, golem::Contact::View& dst);

	void convert(const golem::data::ContactModel::Data& src, Model3D& dst);
	void convert(const Model3D& src, golem::data::ContactModel::Data& dst);

	void convert(const golem::data::ContactQuery::Data& src, Query& dst);
	void convert(const Query& src, golem::data::ContactQuery::Data& dst);
}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_GOLEM_GOLEM_DEFS_H_