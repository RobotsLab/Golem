/** @file Hypothesis.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/HBPlanner/Hypothesis.h>

#ifdef WIN32
#pragma warning (push)
#pragma warning (disable : 4291 4244 4996 4305)
#endif
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <flann/flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#ifdef WIN32
#pragma warning (pop)
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void Hypothesis::BoundsAppearance::draw(const golem::Bounds::Seq& bounds, golem::DebugRenderer& renderer) const {
	if (showSolid) {
		renderer.setColour(solidColour);
		renderer.addSolid(bounds.begin(), bounds.end());
	}
	if (showWire) {
		renderer.setColour(wireColour);
		renderer.setLineWidth(wireWidth);
		renderer.addWire(bounds.begin(), bounds.end());
	}
}

Bounds::Seq Hypothesis::bounds() {
	Bounds::Seq bounds;
	bounds.push_back(boundsDesc.create());
	return bounds;
}

//------------------------------------------------------------------------------

Hypothesis::Hypothesis(const Manipulator& manipulator, const Desc& desc) : manipulator(manipulator), desc(desc) {
	if (!desc.isValid())
		throw Message(Message::LEVEL_CRIT, "Hypothesis(): invalid description");
	collisionPtr = desc.collisionDescPtr->create(manipulator);
}

//------------------------------------------------------------------------------

void Hypothesis::create(const golem::U32 idx, const golem::Mat34& trn, const RBPose::Sample& s, golem::Rand& rand, const Cloud::PointSeq& points) {
	collisionPtr->create(rand, points);
	index = idx;
	modelFrame = trn;
	sample = s;
	Real x = REAL_ZERO, y = REAL_ZERO, z = REAL_ZERO;
	Vec3 sampleFrame = toRBPoseSampleGF().p;
	this->points.reserve(points.size());
	for (Cloud::PointSeq::const_iterator i = points.begin(); i != points.end(); ++i) {
		this->points.push_back(*i);
		// calculate max x, y, z values for bounding box
		if (x < Math::abs(sampleFrame.x - (float)i->x))
			x = Math::abs(sampleFrame.x - (float)i->x);
		if (y < Math::abs(sampleFrame.y - (float)i->y))
			y = Math::abs(sampleFrame.y - (float)i->y);
		if (z < Math::abs(sampleFrame.z - (float)i->z))
			z = Math::abs(sampleFrame.z - (float)i->z);
	}
//	printf("Bounding box dim x=%f, y=%f, z=%f\n", x, y, z);
	// set dimension of the bounding box
	boundsDesc.dimensions = Vec3(x, y, z);
	boundsDesc.pose.p = toRBPoseSampleGF().p;
//	printf("Bounding box dim x=%f, y=%f, z=%f, pose=<%f, %f, %f>\n", boundsDesc.dimensions.x, boundsDesc.dimensions.y, boundsDesc.dimensions.z, boundsDesc.pose.p.x, boundsDesc.pose.p.y, boundsDesc.pose.p.z);
}

void Hypothesis::draw(DebugRenderer &renderer) const {
	printf("Belief::Hypothesis::draw(showFrame=%s, showPoints=%s)\n", appearance.showFrames ? "ON" : "OFF", appearance.showPoints ? "ON" : "OFF");
	if (appearance.showFrames || true)
		renderer.addAxes(sample.toMat34() * modelFrame, appearance.frameSize);

	size_t t = 0;
	if (appearance.showPoints || true) {
		for (Cloud::PointSeq::const_iterator i = points.begin(); i != points.end(); ++i) {
			Cloud::Point point = *i;
			//if (++t < 10) printf("Belief::Hypothesis point %d <%.4f %.4f %.4f>\n", t, point.x, point.y, point.z);
			Cloud::setColour(/*appearance.colour*/RGBA::RED, point);
			renderer.addPoint(Cloud::getPoint<Real>(point));
		}
	}
}

//------------------------------------------------------------------------------

std::string Hypothesis::str() const {
	std::stringstream ss; 
	const RBPose::Sample pose = toRBPoseSampleGF();
	ss << pose.p.x << "\t" << pose.p.y << "\t" << pose.p.z << "\t" << pose.q.w << "\t" << pose.q.x << "\t" << pose.q.y << "\t" << pose.q.z;
	return ss.str();
}

//------------------------------------------------------------------------------

void Hypothesis::draw(const HBCollision::Waypoint &waypoint, const Manipulator::Config& config, golem::DebugRenderer& renderer) const {
	collisionPtr->draw(waypoint, config, renderer);
}

void Hypothesis::draw(golem::DebugRenderer& renderer, const golem::Rand& rand, const Manipulator::Config& config) const {
	collisionPtr->draw(renderer, rand, config, desc.collisionDescPtr->flannDesc);
}

//------------------------------------------------------------------------------

void golem::XMLData(Hypothesis::Desc& val, golem::XMLContext* xmlcontext, bool create) {
	XMLData(*val.collisionDescPtr, xmlcontext->getContextFirst("collision"), create);
}

//------------------------------------------------------------------------------