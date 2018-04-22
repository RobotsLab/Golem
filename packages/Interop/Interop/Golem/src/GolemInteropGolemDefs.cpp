/** @file GolemInteropGolemDefs.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

//------------------------------------------------------------------------------

#include "GolemInteropGolemDefs.h"

using namespace golem;
using namespace golem::interop;

//------------------------------------------------------------------------------

void golem::interop::convert(const golem::Controller& controller, const Config::Seq& src, golem::Controller::State::Seq& dst) {
	dst.resize(src.size(), controller.createState());
	Config::Seq::const_iterator j = src.begin();
	for (golem::Controller::State::Seq::iterator i = dst.begin(); i != dst.end(); ++i, ++j) {
		controller.setToDefault(*i);
		convert(*j, *i);
	}
}

void golem::interop::loadTrajectory(const golem::Controller& controller, const std::string& path, Config::Seq& trajectory) {
	// load
	golem::Controller::State::Seq waypoints;
	golem::FileReadStream(path.c_str()).read(waypoints, waypoints.end(), controller.createState());
	// convert
	convert(waypoints, trajectory);
}

void golem::interop::convert(const golem::Manipulator::Waypoint& src, ConfigModel& dst) {
	convert((const golem::Sample<golem::Real>&)src, (Sample&)dst);
	convert(src.config, dst.config);
	convert(src.frame, dst.frame);
	// TODO control
}

void golem::interop::convert(const ConfigModel& src, golem::Manipulator::Waypoint& dst) {
	convert((const Sample&)src, (golem::Sample<golem::Real>&)dst);
	convert(src.config, dst.config);
	convert(src.frame, dst.frame);
	// TODO control
}

void golem::interop::convert(const golem::Contact3D& src, ContactModel3D& dst) {
	convert((const golem::Sample<golem::Real>&)src, (Sample&)dst);
	convert(src.global, dst.global);
	convert(src.local, dst.local);
	convert(src.feature, dst.feature, dst.featureSize);
	convert(src.model, dst.model);
	// reserved area
	reinterpret_cast<golem::Real&>(dst.reserved[0]) = src.distance;
	reinterpret_cast<golem::Vec3&>(dst.reserved[sizeof(golem::Real)]) = src.projection;
	reinterpret_cast<golem::Contact3D::Relation&>(dst.reserved[sizeof(golem::Real) + sizeof(golem::Vec3)]) = src.relation;
}

void golem::interop::convert(const ContactModel3D& src, golem::Contact3D& dst) {
	convert((const Sample&)src, (golem::Sample<golem::Real>&)dst);
	convert(src.global, dst.global);
	convert(src.local, dst.local);
	convert(src.feature, src.featureSize, dst.feature);
	convert(src.model, dst.model);
	// reserved area
	dst.distance = reinterpret_cast<const golem::Real&>(src.reserved[0]);
	dst.projection = reinterpret_cast<const golem::Vec3&>(src.reserved[sizeof(golem::Real)]);
	dst.relation = reinterpret_cast<const golem::Contact3D::Relation&>(src.reserved[sizeof(golem::Real) + sizeof(golem::Vec3)]);
}

void golem::interop::convert(const golem::Contact3D::Data& src, ContactModel3D::Data& dst) {
	convert(src.type, dst.type);
	dst.model.resize(src.model.size());
	golem::interop::copy(src.model.begin(), src.model.end(), dst.model.begin());
	// reserved area
}

void golem::interop::convert(const ContactModel3D::Data& src, golem::Contact3D::Data& dst) {
	convert(src.type, dst.type);
	dst.model.resize(src.model.size());
	golem::interop::copy(src.model.begin(), src.model.end(), dst.model.begin());
	// reserved area
}

void golem::interop::convert(const golem::Contact::Config& src, Path& dst) {
	dst.type = src.type;
	convert(src.view, dst.view);
	convert(src.space, dst.space);
	convert(src.manifold, dst.manifold);
	convert(src.likelihood.value, dst.weight); // likelihood-weight mapping
	dst.path.resize(src.path.size());
	golem::interop::copy(src.path.begin(), src.path.end(), dst.path.begin());
	// reserved area
	reinterpret_cast<golem::Contact::Likelihood&>(dst.reserved[0]) = src.likelihood;
}
void golem::interop::convert(const Path& src, golem::Contact::Config& dst) {
	dst.type = src.type;
	convert(src.view, dst.view);
	convert(src.space, dst.space);
	convert(src.manifold, dst.manifold);
	dst.path.resize(src.path.size());
	golem::interop::copy(src.path.begin(), src.path.end(), dst.path.begin());
	// reserved area
	dst.likelihood = reinterpret_cast<const golem::Contact::Likelihood&>(src.reserved[0]);
	// overwrite likelihood value
	convert(src.weight, dst.likelihood.value); // likelihood-weight mapping
}

template <> void golem::interop::convert(const golem::Configuration::Space& src, ConfigSpace& dst) {
	dst.name = src.name;
	golem::interop::convert(src.paths, dst.paths);
	golem::interop::convert(src.configs, dst.configs);
	// reserved area
	(void)bufferCopy(src.desc, dst.reserved, dst.reserved.begin());
}
template <> void golem::interop::convert(const ConfigSpace& src, golem::Configuration::Space& dst) {
	dst.name = src.name;
	golem::interop::convert(src.paths, dst.paths);
	golem::interop::convert(src.configs, dst.configs);
	// reserved area
	(void)bufferCopy(src.reserved.begin(), dst.desc);
}

void golem::interop::convert(const golem::Contact::View& src, ContactView3D& dst) {
	convert((const golem::Sample<golem::Real>&)src, (golem::interop::Sample&)dst);
	convert(src.space, dst.space);

	dst.models.clear();
	for (auto &i : src.models) {
		ContactView3D::ModelPtr::Map::value_type value;
		const_cast<std::uint32_t&>(value.first) = i.first.getIndex(); // ignore offset
		convert(i.second.index, value.second.index);
		convert(i.second.weight, value.second.weight);
		convert(i.second.frame, value.second.frame);
		dst.models.insert(value);
	}

	// reserved area
	golem::interop::Buffer::iterator ptr = bufferCopy(src.object, dst.reserved, dst.reserved.begin());
	(void)bufferCopySeq(src.points, dst.reserved, ptr);
}
void golem::interop::convert(const ContactView3D& src, golem::Contact::View& dst) {
	convert((const golem::interop::Sample&)src, (golem::Sample<golem::Real>&)dst);
	convert(src.space, dst.space);

	dst.models.clear();
	for (auto &i : src.models) {
		golem::Contact::View::ModelPtr::Map::value_type value;
		const_cast<golem::Manipulator::Link&>(value.first).setIndexOffset(static_cast<golem::U32>(i.first)); // ignore offset
		convert(i.second.index, value.second.index);
		convert(i.second.weight, value.second.weight);
		convert(i.second.frame, value.second.frame);
		dst.models.insert(value);
	}

	// reserved area
	golem::interop::Buffer::const_iterator ptr = bufferCopy(src.reserved.begin(), dst.object);
	(void)bufferCopySeq(ptr, dst.points);
}

void golem::interop::convert(const golem::data::ContactModel::Data& src, Model3D& dst) {
	dst.contacts.clear();
	for (auto &i : src.contacts)
		golem::interop::convert(i.second, dst.contacts[i.first]);
	golem::interop::convert(src.views, dst.views);
	golem::interop::convert(src.spaces, dst.spaces);

	// reserved area
	golem::interop::Buffer::iterator ptr = bufferCopy(src.type, dst.reserved, dst.reserved.begin());
	//(void)bufferCopySeq(src.selectorMap, dst.reserved, ptr); // TODO - requires overloading bufferCopy()
}
void golem::interop::convert(const Model3D& src, golem::data::ContactModel::Data& dst) {
	dst.contacts.clear();
	for (auto &i : src.contacts)
		golem::interop::convert(i.second, dst.contacts[i.first]);
	golem::interop::convert(src.views, dst.views);
	golem::interop::convert(src.spaces, dst.spaces);

	// reserved area
	golem::interop::Buffer::const_iterator ptr = bufferCopy(src.reserved.begin(), dst.type);
	//(void)bufferCopySeq(ptr, dst.selectorMap); // TODO - requires overloading bufferCopy()
}

void golem::interop::convert(const golem::data::ContactQuery::Data& src, Query& dst) {
	golem::interop::convert(src.configs, dst.paths);

	// reserved area
	golem::interop::Buffer::iterator ptr = bufferCopySeq(src.points, dst.reserved, dst.reserved.begin());
	ptr = bufferCopySeq(src.pointsIndices, dst.reserved, ptr);
	ptr = bufferCopySeq(src.pointsClusters, dst.reserved, ptr);
	//ptr = bufferCopySeq(src.pointsSelection, dst.reserved, ptr); // TODO - requires overloading bufferCopy()
}
void golem::interop::convert(const Query& src, golem::data::ContactQuery::Data& dst) {
	golem::interop::convert(src.paths, dst.configs);
	// one cluster
	dst.clusters.clear();
	dst.clusters.push_back(golem::Contact::Config::Cluster(0, (golem::U32)dst.configs.size()));

	// reserved area
	golem::interop::Buffer::const_iterator ptr = bufferCopySeq(src.reserved.begin(), dst.points);
	ptr = bufferCopySeq(ptr, dst.pointsIndices);
	ptr = bufferCopySeq(ptr, dst.pointsClusters);
	//ptr = bufferCopySeq(ptr, dst.pointsSelection); // TODO - requires overloading bufferCopy()
}

//------------------------------------------------------------------------------
