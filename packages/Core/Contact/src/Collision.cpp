/** @file Collision.cpp
 *
 * Contact collision model
 *
 * @author Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

//------------------------------------------------------------------------------

#include <Golem/Contact/Collision.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::Collision::Waypoint::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("depth_offset", depthOffset, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("depth_stddev", depthStdDev, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("path_dist", pathDist, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("weight", weight, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("points", points, const_cast<golem::XMLContext*>(xmlcontext), false);
}

void golem::Collision::Desc::load(const golem::XMLContext* xmlcontext) {
	waypoints.clear();
	golem::XMLData(waypoints, waypoints.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "waypoint", false);
}

//------------------------------------------------------------------------------

Collision::Collision(const Manipulator& manipulator, const Desc& desc) : manipulator(manipulator), desc(desc) {
	desc.assertValid(Assert::Context("Collision()."));

	// joints - hand only
	for (golem::Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i)
		jointBounds[i].create(manipulator.getJointBounds(i), desc.offset);
	
	// base
	baseBounds.create(manipulator.getBaseBounds(), desc.offset);
}

void golem::Collision::create(golem::Rand& rand, const Bounds::Vec3Seq& points) {
	this->points = points;
	std::random_shuffle(this->points.begin(), this->points.end(), rand);
}

void Collision::create(golem::Rand& rand, const data::Point3D& points) {
	// points
	this->points.clear();
	this->points.reserve(points.getSize());
	for (size_t i = 0, size = points.getSize(); i < size; ++i)
		this->points.push_back(Bounds::Vec3((const golem::data::Point3D::Vec3&)points.getPoint(i)));
	std::random_shuffle(this->points.begin(), this->points.end(), rand);
}

golem::Real Collision::evaluate(const Waypoint& waypoint, const Manipulator::Config& config, bool debug) {
	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord joints;
	manipulator.getJointFrames(config.config, base, joints);

	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;

	Bounds::Real eval = golem::numeric_const<Bounds::Real>::ZERO, depth = golem::numeric_const<Bounds::Real>::ZERO;
	size_t collisions = 0;

	// joints - hand only
	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds& bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(joints[i]));
		bounds.evaluate(points.data(), points.data() + size, (Bounds::Real)waypoint.depthOffset, (Bounds::RealEval)waypoint.depthStdDev, eval, depth, collisions);
	}

	// base
	if (!baseBounds.empty()) {
		baseBounds.setPose(Bounds::Mat34(base));
		baseBounds.evaluate(points.data(), points.data() + size, (Bounds::Real)waypoint.depthOffset, (Bounds::RealEval)waypoint.depthStdDev, eval, depth, collisions);
	}

	const Real likelihood = -waypoint.weight*eval;

	if (debug)
		manipulator.getContext().debug("Collision::evaluate(): points=%u, collisions=%u, depth=%f, eval=%f, likelihhod=%f, offset=%f, std_dev=%f, path=%f, weight=%f\n", size, collisions, depth / size, eval, likelihood, waypoint.depthOffset, waypoint.depthStdDev, waypoint.pathDist, waypoint.weight);

	return likelihood;
}

golem::Real Collision::evaluate(const Manipulator::Waypoint::Seq& path, bool debug) {
	golem::Real eval = REAL_ZERO;
	for (Waypoint::Seq::const_iterator i = desc.waypoints.begin(); i != desc.waypoints.end(); ++i)
		eval += evaluate(*i, manipulator.interpolate(path, i->pathDist), debug);
	return eval;
}

//------------------------------------------------------------------------------

void golem::XMLData(Collision::Waypoint& val, golem::XMLContext* xmlcontext, bool create) {
	val.load(xmlcontext);
}

//------------------------------------------------------------------------------
