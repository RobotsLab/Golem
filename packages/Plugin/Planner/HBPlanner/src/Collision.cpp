/** @file Collision.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */
#include <Golem/HBPlanner/Collision.h>

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

#ifdef _HBCOLLISION_PERFMON
U32 HBCollision::perfEvalPoints;
U32 HBCollision::perfCheckNN;
U32 HBCollision::perfEstimate;
U32 HBCollision::perfSimulate;
SecTmReal HBCollision::tperfEvalPoints;
SecTmReal HBCollision::tperfCheckNN;
SecTmReal HBCollision::tperfEstimate;
SecTmReal HBCollision::tperfSimulate;
//U32 HBCollision::tperfEvalPoints;
//U32 HBCollision::tperfEvalBelief;
//U32 HBCollision::tperfEvalRobot;

void HBCollision::resetLog() {
	perfEvalPoints = 0;
	perfCheckNN = 0;
	perfEstimate = 0;
	perfSimulate = 0;
	tperfEvalPoints = SEC_TM_REAL_ZERO;
	tperfCheckNN = SEC_TM_REAL_ZERO;
	tperfEstimate = SEC_TM_REAL_ZERO;
	tperfSimulate = SEC_TM_REAL_ZERO;
}

void HBCollision::writeLog(Context &context, const char *str) {
	context.write(
		"%s: HBCollision::collision_{point cloud, [sec], checkNN, [sec], estimate, [sec], simulate, [sec]} = {%u, %f, %u, %f, %u, %f, %u, %f}\n", str, perfEvalPoints, tperfEvalPoints / (perfEvalPoints > 0 ? perfEvalPoints : 1), 
		perfCheckNN, tperfCheckNN / (perfCheckNN > 0 ? perfCheckNN : 1), 
		perfEstimate, tperfEstimate / (perfEstimate > 0 ? perfEstimate : 1), 
		perfSimulate, tperfSimulate / (perfSimulate > 0 ? perfSimulate : 1)
		);
}

#endif

//------------------------------------------------------------------------------

static Vec3 getRelativeFrame(const Mat34 &reference, const Vec3 &point) {
	Vec3 v;
	Mat34 inverse;
	inverse.setInverse(reference);
	inverse.multiply(v, point);
	return v;// .normalise();
}

const char* HBCollision::FaceName[] = {
	"UNKNOWN",
	"FRONT",
	"RIGHT",
	"LEFT",
	"BACK",
	"TIP",
	"TOP",
};

//------------------------------------------------------------------------------

void HBCollision::Feature::draw(const Appearance& appearance, DebugRenderer& renderer) const {
	if (appearance.frameShow)
		renderer.addAxes(frame, appearance.frameSize);

	if (appearance.normalShow) {
		Vec3 n;
		n.multiply(appearance.normalSize, normal);
		frame.R.multiply(n, n);
		n.add(point, n);
		renderer.addLine(point, n, appearance.normalColour);
	}
}

//------------------------------------------------------------------------------

HBCollision::HBCollision(const Manipulator& manipulator, const Desc& desc) : manipulator(manipulator), desc(desc) {
	if (!desc.isValid())
		throw Message(Message::LEVEL_CRIT, "Collision(): invalid description");

	nnSearch.reset();

	// joints - hand only
	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i)
		jointBounds[i].create(manipulator.getJointBounds(i));

	// ft only
//	faces = { Face::FRONT, Face::FRONT, Face::TOP, Face::TOP, Face::RIGHT, Face::RIGHT, Face::LEFT, Face::LEFT, Face::TIP, Face::TIP, Face::BACK, Face::BACK };

	//faces = { Face::RIGHT, Face::RIGHT, Face::RIGHT, Face::RIGHT, Face::RIGHT, Face::LEFT, Face::LEFT, Face::LEFT, Face::LEFT, Face::LEFT, Face::TOP, Face::TOP, Face::BACK, Face::BACK,
	//	Face::TIP, Face::TIP, Face::TIP, Face::TIP, Face::TIP, Face::FRONT, Face::FRONT, Face::FRONT, Face::FRONT};

	// ft has two bounds
	faces = { Face::FRONT, Face::FRONT, Face::BACK, Face::BACK, Face::RIGHT, Face::RIGHT, Face::TOP, Face::TOP, Face::LEFT, Face::LEFT, Face::TIP, Face::TIP};
	// no fts
	//faces = { Face::FRONT, Face::FRONT, Face::TOP, Face::TOP, Face::RIGHT, Face::RIGHT, Face::LEFT, Face::LEFT, Face::TIP, Face::TIP, Face::BACK, Face::BACK };
	for (Chainspace::Index i = manipulator.getHandInfo().getChains().begin(); i != manipulator.getHandInfo().getChains().end(); ++i) {
		const Configspace::Index j = manipulator.getHandInfo().getJoints(i).end() - 1;
		ftBounds[j].create(manipulator.getJointBounds(j));
		for (Bounds::Triangle::SeqSeq::iterator t0 = ftBounds[j].getTriangles().begin(); t0 != ftBounds[j].getTriangles().end(); ++t0) {
			size_t k = 0;
			for (auto t1 = t0->begin(); t1 != t0->end(); ++t1) {
				if (k < faces.size())
					t1->face = faces[k++];
				else
					t1->face = Face::UNKNOWN;
			}
		}
		ftJoints.push_back(j);
	}
	// base
	baseBounds.create(manipulator.getBaseBounds());


	manipulatorAppearance.setToDefault();
	manipulatorAppearance.showSolid = false;
	manipulatorAppearance.showWire = true;
	manipulatorAppearance.wireWidth = 0.01;
	manipulatorAppearance.wireColour = RGBA::BLACK;
	// bounds
	/*bounds.resize(Manipulator::JOINTS + 1);
	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints() + 1; ++i)
		bounds[i].create(i < manipulator.getJoints() ? manipulator.getJointBounds(i) : manipulator.getBaseBounds());*/
}

void HBCollision::create(Rand& rand, const Cloud::PointSeq& points) {
	// points
	this->points.clear();
	this->points.reserve(points.size());

	for (Cloud::PointSeq::const_iterator i = points.begin(); i != points.end(); ++i)
		this->points.push_back(Feature(Cloud::getPoint<Real>(*i), Cloud::getNormal<Real>(*i)));

	std::random_shuffle(this->points.begin(), this->points.end(), rand);

	flann::SearchParams search;
	flann::KDTreeSingleIndexParams index;
	search.checks = 32;
	search.max_neighbors = 10;
	desc.nnSearchDesc.getKDTreeSingleIndex(search, index);
		
	typedef KDTree<Real, Feature::FlannDist, flann::SearchParams> KDTree;
	nnSearch.reset(new KDTree(search, index, this->points, Feature::N, Feature::FlannDist()));

	//if (desc.kdtree) {
	//	flann::SearchParams search;
	//	flann::KDTreeSingleIndexParams index;
	//	search.checks = 32;
	//	search.max_neighbors = 10;
	//	desc.nnSearchDesc.getKDTreeSingleIndex(search, index);
	//	typedef KDTree<Real, Feature::FlannDist, flann::SearchParams> KDTree;
	//	nnSearch.reset(new KDTree(search, index, this->points, Feature::N, Feature::FlannDist()));
	//}
	//else {
	//	std::random_shuffle(this->points.begin(), this->points.end(), rand);
	//}
}

//------------------------------------------------------------------------------

size_t HBCollision::simulate(const FlannDesc& desc, const Rand& rand, const Manipulator::Config& config, std::vector<Configspace::Index> &joints, RealSeq &forces, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("HBCollision::Check(): No KD-Tree\n");
		return REAL_ZERO;
	}
#ifdef _HBCOLLISION_PERFMON
	PerfTimer t;
	++perfSimulate;
#endif
	joints.clear();
	forces.clear();

	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord wjoints;
	manipulator.getJointFrames(config.config, base, wjoints);
	/*const Mat34 pose(rbpose.toMat34());
	Mat34 poses[Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);*/

	NNSearch::IndexSeq indices;
	NNSearch::DistanceF64Seq distances;

	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;

	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty()) {
			//			manipulator.getContext().write("HBCollision::checkNN(): No bounds.\n");
			continue;
		}

		bounds.setPose(Bounds::Mat34(wjoints[i]));
		//		manipulator.getContext().write("-----------------------------\nBound pose <%f, %f, %f> (<%f, %f, %f>)\n", poses[i].p.x, poses[i].p.y, poses[i].p.z, poses[i].p.v[0], poses[i].p.v[1], poses[i].p.v[2]);

		Feature query(wjoints[i].p);
		//nnSearch->knnSearch(/*(const Real*)&*/poses[i].p/*.x*//*.v*/, neighbours, indices, distances);
		nnSearch->knnSearch(query, desc.neighbours, indices, distances);

		Real force = REAL_ZERO;
		Vec3 frame(.0, .0, .0);
		size_t collisions = 0;
		for (size_t j = 0; j < size; ++j) {
			//			const Real depth = Math::abs(bounds.getDepth(points[indices[j]]/*.getPoint()*/));
			const Feature f = desc.points < desc.neighbours ? points[indices[size_t(rand.next()) % indices.size()]] : points[indices[j]];
			const Real depth = bounds.getDepth(f);

			if (depth > REAL_ZERO) {
#ifdef _HBCOLLISION_PERFMON
				SecTmReal t_end = t.elapsed();
				tperfSimulate += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
				if (debug /*&& eval < REAL_ONE*/)
					manipulator.getContext().debug(
					"HBCollision::check(kd-tree): time_elapsed = %f [sec], collision=yes, neighbours=%u, points=%u\n",
					t_end, desc.neighbours, desc.points
					);
#endif
				force += depth;
				frame.add(frame, getRelativeFrame(wjoints[i], f.point));
				//Vec3 w; poses[i].multiply(w, frame);
				//const Vec3 p[2] = { poses[i].p, w };
				//renderer.addLine(p[0], p[1], RGBA::MAGENTA);
				++collisions;
			}
			//if (j % 10 == 0) manipulator.getContext().write("point=%d <%f, %f, %f> index=%d distance=%f depth=%f collision=%s\n", j, f.getPoint().x, f.getPoint().y, f.getPoint().z,
			//	indices[j], distances[j], bounds.getDepth(points[j]), bounds.getDepth(f) > REAL_ZERO ? "YES" : "NO");

		}
		if (collisions > 0) {
			joints.push_back(Configspace::Index(i));
			force /= collisions;
			frame /= collisions;
			forces.push_back(Math::sign(REAL_ONE, -frame.z)*force * 1000);
		}
	}

#ifdef _HBCOLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfSimulate += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().debug("HBCollision::evaluate(): neighbours=%u, , points=%u, collision=no\n", desc.neighbours, desc.points);
#endif

	return joints.size();
}

size_t HBCollision::simulate(const FlannDesc& desc, const Rand& rand, const Manipulator::Config& config, FTGuard::Seq &joints, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("HBCollision::Check(): No KD-Tree\n");
		return REAL_ZERO;
	}
#ifdef _HBCOLLISION_PERFMON
	PerfTimer t;
	++perfSimulate;
#endif
	joints.clear();

	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	NNSearch::IndexSeq indices;
	NNSearch::DistanceF64Seq distances;

	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;

	Real maxDepth = REAL_ZERO;
	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty()) {
			//			manipulator.getContext().write("HBCollision::checkNN(): No bounds.\n");
			continue;
		}

		bounds.setPose(Bounds::Mat34(poses[i]));
		//		manipulator.getContext().write("-----------------------------\nBound pose <%f, %f, %f> (<%f, %f, %f>)\n", poses[i].p.x, poses[i].p.y, poses[i].p.z, poses[i].p.v[0], poses[i].p.v[1], poses[i].p.v[2]);

		Feature query(poses[i].p);
		//nnSearch->knnSearch(/*(const Real*)&*/poses[i].p/*.x*//*.v*/, neighbours, indices, distances);
		nnSearch->knnSearch(query, desc.neighbours, indices, distances);

		Real force = REAL_ZERO;
		Vec3 frame(.0, .0, .0);
		size_t collisions = 0;
		for (size_t j = 0; j < size; ++j) {
			//			const Real depth = Math::abs(bounds.getDepth(points[indices[j]]/*.getPoint()*/));
			const Feature f = desc.points < desc.neighbours ? points[indices[size_t(rand.next()) % indices.size()]] : points[indices[j]];
			const Real depth = bounds.getDepth(f);

			if (depth > REAL_ZERO) {
#ifdef _HBCOLLISION_PERFMON
				SecTmReal t_end = t.elapsed();
				tperfSimulate += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
				if (debug /*&& eval < REAL_ONE*/)
					manipulator.getContext().write(
					"HBCollision::check(kd-tree): time_elapsed = %f [sec], collision=yes, depth=%f, neighbours=%u, points=%u\n",
					t_end, depth, desc.neighbours, desc.points
					);
#endif
				if (depth > maxDepth) maxDepth = depth;
				force += depth;
				frame.add(frame, getRelativeFrame(poses[i], f.point));
				//Vec3 w; poses[i].multiply(w, frame);
				//const Vec3 p[2] = { poses[i].p, w };
				//renderer.addLine(p[0], p[1], RGBA::MAGENTA);
				++collisions;
			}
			//if (j % 10 == 0) manipulator.getContext().write("point=%d <%f, %f, %f> index=%d distance=%f depth=%f collision=%s\n", j, f.getPoint().x, f.getPoint().y, f.getPoint().z,
			//	indices[j], distances[j], bounds.getDepth(points[j]), bounds.getDepth(f) > REAL_ZERO ? "YES" : "NO");
		}
		if (collisions > 0) {
			//const size_t k = i - manipulator.getHandInfo().getJoints().begin();
			//FTGuard guard(manipulator);
			//guard.create(Configspace::Index(i));
			//force /= collisions;
			//frame /= collisions;
			//guard.force = Math::sign(REAL_ONE, -frame.z) * (this->desc.ftContact.ftMedian[k] + (2 * rand.nextUniform<Real>()*this->desc.ftContact.ftStd[k] - this->desc.ftContact.ftStd[k]));//* force * 0.1/* * 1000*/;
			//joints.push_back(guard);
//			forces.push_back(Math::sign(REAL_ONE, -frame.z)*force * 1000);
		}
	}

#ifdef _HBCOLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfSimulate += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().write("HBCollision::evaluate(): neighbours=%u, , points=%u, collision=no\n", desc.neighbours, desc.points);
#endif
	if (maxDepth > REAL_ZERO) manipulator.getContext().write("Simulate contact max depth = %f\n", maxDepth);
	return joints.size();
}

size_t HBCollision::simulate(const FlannDesc& desc, const Rand& rand, const Manipulator::Config& config, RealSeq &forces, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("HBCollision::Check(): No KD-Tree\n");
		return REAL_ZERO;
	}
#ifdef _HBCOLLISION_PERFMON
	PerfTimer t;
	++perfSimulate;
#endif

	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	NNSearch::IndexSeq indices;
	NNSearch::DistanceF64Seq distances;

	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;

	Real maxDepth = REAL_ZERO;
	size_t collided = 0;
	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
		//		manipulator.getContext().write("-----------------------------\nBound pose <%f, %f, %f> (<%f, %f, %f>)\n", poses[i].p.x, poses[i].p.y, poses[i].p.z, poses[i].p.v[0], poses[i].p.v[1], poses[i].p.v[2]);

		Feature query(poses[i].p);
		//nnSearch->knnSearch(/*(const Real*)&*/poses[i].p/*.x*//*.v*/, neighbours, indices, distances);
		nnSearch->knnSearch(query, desc.neighbours, indices, distances);

		Real force = REAL_ZERO;
		Vec3 frame(.0, .0, .0);
		size_t collisions = 0;
		for (size_t j = 0; j < size; ++j) {
			//			const Real depth = Math::abs(bounds.getDepth(points[indices[j]]/*.getPoint()*/));
			const Feature f = desc.points < desc.neighbours ? points[indices[size_t(rand.next()) % indices.size()]] : points[indices[j]];
			const Real depth = bounds.getDepth(f);

			if (depth > REAL_ZERO) {
#ifdef _HBCOLLISION_PERFMON
				SecTmReal t_end = t.elapsed();
				tperfSimulate += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
				if (debug && false/*&& eval < REAL_ONE*/)
					manipulator.getContext().write(
					"HBCollision::check(kd-tree): time_elapsed = %f [sec], collision=yes, depth=%f, neighbours=%u, points=%u\n",
					t_end, depth, desc.neighbours, desc.points
					);
#endif
				if (depth > maxDepth) maxDepth = depth;
				force += depth;
				frame.add(frame, getRelativeFrame(poses[i], f.point));
				//Vec3 w; poses[i].multiply(w, frame);
				//const Vec3 p[2] = { poses[i].p, w };
				//renderer.addLine(p[0], p[1], RGBA::MAGENTA);
				++collisions;
			}
			//if (j % 10 == 0) manipulator.getContext().write("point=%d <%f, %f, %f> index=%d distance=%f depth=%f collision=%s\n", j, f.getPoint().x, f.getPoint().y, f.getPoint().z,
			//	indices[j], distances[j], bounds.getDepth(points[j]), bounds.getDepth(f) > REAL_ZERO ? "YES" : "NO");
		}
		if (collisions > 0) {
//			FTGuard guard(manipulator);
//			guard.create(Configspace::Index(i));
//			force /= collisions;
//			frame /= collisions;
			const size_t k = i - manipulator.getHandInfo().getJoints().begin();
			forces[k] = debug ? (Math::sign(REAL_ONE, -frame.z) * force / collisions)*10000 : Math::sign(REAL_ONE, -frame.z) * (this->desc.ftContact.ftMedian[k] + (2 * rand.nextUniform<Real>()*this->desc.ftContact.ftStd[k] - this->desc.ftContact.ftStd[k]));//* force * 0.1/* * 1000*/;
			//if (debug) manipulator.getContext().write("Collision joint=%d finger=%d link=%d force=%3.3f\n", i, U32(k/4)+1, U32(k%4), forces[k]);
			++collided;
//			joints.push_back(guard);
			//			forces.push_back(Math::sign(REAL_ONE, -frame.z)*force * 1000);
		}
	}

#ifdef _HBCOLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfSimulate += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug && false/*&& eval < REAL_ONE*/)
		manipulator.getContext().write("HBCollision::evaluate(): neighbours=%u, , points=%u, collision=no\n", desc.neighbours, desc.points);
#endif
//	if (maxDepth > REAL_ZERO) manipulator.getContext().write("Simulate contact max depth = %f\n", maxDepth);
	return collided;// joints.size();
}

size_t HBCollision::simulateFT(DebugRenderer& renderer, const FlannDesc& desc, const Rand& rand, const Manipulator::Config& config, RealSeq &forces, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("HBCollision::Check(): No KD-Tree\n");
		return REAL_ZERO;
	}
#ifdef _HBCOLLISION_PERFMON
	PerfTimer t;
	++perfSimulate;
#endif

	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	NNSearch::IndexSeq indices;
	NNSearch::DistanceF64Seq distances;

	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;

	Real maxDepth = REAL_ZERO;
	size_t collided = 0;
	static size_t idx = 0;
	for (Chainspace::Index i = manipulator.getHandInfo().getChains().begin(); i != manipulator.getHandInfo().getChains().end(); ++i) {
		const size_t k = i - manipulator.getHandInfo().getChains().begin();
		if (k > 2)
			break;
		for (Configspace::Index j = manipulator.getHandInfo().getJoints(i).begin(); j < manipulator.getHandInfo().getJoints(i).end(); ++j) {
			Bounds bounds = ftBounds[j];
			if (bounds.empty())
				continue;

			bounds.setPose(Bounds::Mat34(poses[j]));
			if (debug) {
				renderer.reset();
				renderer.addAxes(poses[j], Vec3(.05, .05, .05));
				draw(renderer, this->desc.boundsAppearence, poses[j], bounds);
				//size_t tt = 0;
				//manipulator.getContext().write("Triangles %d\n", bounds.getTriangles().size());
				//for (auto t = bounds.getTriangles().begin(); t != bounds.getTriangles().end(); ++t) {
				//	manipulator.getContext().write("T%d size %d\n", tt++, t->size());
				//	for (auto t1 = t->begin(); t1 != t->end(); ++t1) {
				//		Mat34 m(Mat33::identity(), (Vec3)t1->point);
				//		renderer.addAxes(m, Vec3(.005, .005, .005));
				//	}
				//}
			}

			U32 collisions = 0;

			Real depth = REAL_ZERO;
			Vec3 median;
			for (auto t = bounds.getTriangles().begin(); t != bounds.getTriangles().end(); ++t) {
				for (auto t1 = t->begin(); t1 != t->end(); ++t1) {
					Feature query(t1->point);
					Feature::Seq seq;
					seq.reserve(indices.size());

					nnSearch->knnSearch(query, desc.neighbours, indices, distances);
					for (size_t l = 0; l < indices.size(); ++l)
						seq.push_back(points[indices[l]]);

					//Mat34 inverse; inverse.setInverse(Bounds::Mat34(i < manipulator.getJoints() ? poses[i] : pose)); // compute the inverse of the joint frame
					U32 boundsCollisions = 0;
					median.setZero();
					depth = bounds.distance(seq.data(), seq.data() + seq.size(), median, boundsCollisions);
					//edepth = bounds.evaluate(seq.data(), seq.data() + seq.size(), (Bounds::RealEval)desc.depthStdDev, boundsCollisions);

					if (debug) {
						manipulator.getContext().write("Simulate depth=%f (col=%d) seq=%d\n", depth, boundsCollisions, seq.size());
						draw(seq.begin(), seq.end(), this->desc.featureAppearence, renderer);
						Mat34 m(Mat33::identity(), median);
						renderer.addAxes(m, Vec3(0.005, 0.005, 0.005));
					}

					if (boundsCollisions > 0) {
						const Real value = (this->desc.ftContact.ftMedian[k] + (2 * rand.nextUniform<Real>()*this->desc.ftContact.ftStd[k] - this->desc.ftContact.ftStd[k]));//* force * 0.1/* * 1000*/;
						if (debug) manipulator.getContext().write("Simulate depth=%f (col=%d) seq=%d\n", depth, boundsCollisions, seq.size());
						if (debug) manipulator.getContext().write("Collision joint=%d finger=%d link=%d force=%3.3f\n", i, U32(k / 4) + 1, U32(k % 4), value);
						if (t1->face == Face::TIP)
							forces[k * 6] = -value;
						else if (t1->face == Face::FRONT)
							forces[k * 6 + 1] = -value;
						else if (t1->face == Face::BACK)
							forces[k * 6 + 1] = value;
						else if (t1->face == Face::LEFT)
							forces[k * 6 + 2] = -value;
						else if (t1->face == Face::RIGHT)
							forces[k * 6 + 2] = value;
						collisions += boundsCollisions;
						collided++;
						break;
					}
				}
			}

			//if (debug && idx++ % 100 == 0) {
			//	manipulator.getContext().write("Simulated depth = %.4f, seq size = %d\n", depth, seq.size());
			//	draw(seq.begin(), seq.end(), this->desc.featureAppearence, renderer);
			//	Mat34 m(Mat33::identity(), median);
			//	renderer.addAxes(m, Vec3(1, 1, 1));
			//}
			////if (depth > REAL_ZERO) {
			////	force += depth;
			////	frame.add(frame, getRelativeFrame(poses[j], median));
			////}
			//if (collisions > 0) {
			//	Vec3 frameInv = getRelativeFrame(poses[j], median); Vec3 normalised(frameInv);
			//	if (debug && false) manipulator.getContext().write("Normalised [%.3f %.3f %.3f]\n", frameInv.x, frameInv.y, frameInv.z);
			//	normalised.normalise();
			//	if (debug && false) {
			//		manipulator.getContext().write("Median [%.3f %.3f %.3f] Normalised [%.3f %.3f %.3f]\n", median.x, median.y, median.z, normalised.x, normalised.y, normalised.z);
			//		manipulator.getContext().write("force[%d] = %.2f * (%.3f * %.3f * c) = %.3f\n", k * 6, Math::sign(REAL_ONE, -normalised.x), normalised.x, depth, Math::sign(REAL_ONE, -normalised.x) * (normalised.x * depth * 100000));
			//		manipulator.getContext().write("force[%d] = %.2f * (%.3f * %.3f * c) = %.3f\n", k * 6 + 1, Math::sign(REAL_ONE, -normalised.y), normalised.y, depth, Math::sign(REAL_ONE, -normalised.y) * (normalised.y * depth * 100000));
			//		manipulator.getContext().write("force[%d] = %.2f * (%.3f * %.3f * c) = %.3f\n", k * 6 + 2, Math::sign(REAL_ONE, -normalised.z), normalised.z, depth, Math::sign(REAL_ONE, -normalised.z) * (normalised.z * depth * 100000));
			//	}
			//	//force /= collisions;
			//	forces[k * 6] = Math::sign(REAL_ONE, frameInv.x) * (Math::abs(normalised.x) * depth * 100000);
			//	forces[k * 6 + 1] = Math::sign(REAL_ONE, frameInv.y) * (Math::abs(normalised.y) * depth * 100000);
			//	forces[k * 6 + 2] = Math::sign(REAL_ONE, frameInv.z) * (Math::abs(normalised.z) * depth * 100000);
			//	++collided;
			//}
		}
	}

#ifdef _HBCOLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfSimulate += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug/* && false*/)
		manipulator.getContext().write("HBCollision::simulate(kd-tree): neighbours=%u, , points=%u, collision=no\nforces=[%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f ]\n", 
		desc.neighbours, desc.points, forces[0], forces[1], forces[2], forces[3], forces[4], forces[5], forces[6], forces[7], forces[8], forces[9], 
		forces[10], forces[11], forces[12], forces[13], forces[14], forces[15], forces[16], forces[17]);
#endif
	//	if (maxDepth > REAL_ZERO) manipulator.getContext().write("Simulate contact max depth = %f\n", maxDepth);
	return collided;// joints.size();
}

//------------------------------------------------------------------------------

//bool HBCollision::check(const HBCollision::Waypoint& waypoint, const Manipulator::Config& config, bool debug) const {
//#ifdef _HBCOLLISION_PERFMON
//	PerfTimer t;
////	++perfCheckPoints;
//#endif
//	const Mat34 base(config.frame.toMat34());
//	WorkspaceJointCoord poses;
//	manipulator.getJointFrames(config.config, base, poses);
//
//	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;
//
//	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
//		Bounds bounds = jointBounds[i];
//		if (bounds.empty())
//			continue;
//
//		bounds.setPose(Bounds::Mat34(poses[i]));
//
//		for (size_t j = 0; j < size; ++j) {
//			const Real depth = bounds.getDepth(points[j]/*.getPoint()*/);
//			if (depth > REAL_ZERO) {
//#ifdef _HBCOLLISION_PERFMON
//				SecTmReal t_end = t.elapsed();
////				tperfCheckPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
//				if (debug /*&& eval < REAL_ONE*/)
//					manipulator.getContext().debug(
//					"HBCollision::check(Waypoint): time_elapsed = %f [sec], points=%u, collision=yes\n",
//					t_end, size
//					);
//#endif
//				return true;
//			}
//		}
//	}
//
//#ifdef _HBCOLLISION_PERFMON
//	SecTmReal t_end = t.elapsed();
////	tperfCheckPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
//	if (debug /*&& eval < REAL_ONE*/)
//		manipulator.getContext().debug("HBCollision::evaluate(): points=%u, collision=no\n", size);
//#endif
//
//	return false;
//}

bool HBCollision::checkNN(const FlannDesc& desc, const Manipulator::Config& config, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("HBCollision::Check(): No KD-Tree\n");
		return false;
	}
#ifdef _HBCOLLISION_PERFMON
	PerfTimer t;
	++perfCheckNN;
#endif
	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	NNSearch::IndexSeq indices;
	NNSearch::DistanceF64Seq distances;

	static size_t count = 0;
	Real maxDist = -numeric_const<Real>::MAX;
	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;
	for (Chainspace::Index i = manipulator.getHandInfo().getChains().begin(); i != manipulator.getHandInfo().getChains().end(); ++i) {
		for (Configspace::Index j = manipulator.getHandInfo().getJoints(i).begin(); j < manipulator.getHandInfo().getJoints(i).end(); ++j) {
			Bounds bounds = jointBounds[j];
			if (bounds.empty())
				continue;

			bounds.setPose(Bounds::Mat34(poses[j]));

			Feature query(poses[j].p);
			nnSearch->knnSearch(query, desc.neighbours, indices, distances);

			U32 collisions = 0;

			Feature::Seq seq;
			seq.reserve(indices.size());
			for (size_t l = 0; l < indices.size(); ++l)
				seq.push_back(points[indices[l]]);

			Vec3 median;
			median.setZero();
			const Real depth = bounds.distance(seq.data(), seq.data() + seq.size(), median, collisions);
//			const Real depth = bounds.evaluate(seq.data(), seq.data() + seq.size(), (Bounds::RealEval)desc.depthStdDev, collisions);
			if (depth > maxDist) maxDist = depth;
			if (depth > -desc.radius) {
#ifdef _HBCOLLISION_PERFMON
				SecTmReal t_end = t.elapsed();
				tperfCheckNN += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
				if (debug /*&& eval < REAL_ONE*/)
					manipulator.getContext().write(
					"HBCollision::check(kd-tree): time_elapsed = %f [sec], collision=yes, depth=%.7f, neighbours=%u, points=%u\n",
					t_end, depth, desc.neighbours, desc.points
					);
#endif
				return true;
			}
		}
	}

#ifdef _HBCOLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfCheckNN += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug && count++ % 100 == 0)
		manipulator.getContext().write("HBCollision::evaluate(): elapse_time=%.2f, neighbours=%u, dist=%.3f, collision=no\n", t_end, desc.neighbours, maxDist);
#endif

	return false;
}

//------------------------------------------------------------------------------

Real HBCollision::estimate(const FlannDesc& desc, const Manipulator::Config& config, Real maxDist, bool debug) const {
#ifdef _HBCOLLISION_PERFMON
	PerfTimer t;
	++perfEstimate;
#endif
	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	const size_t size = points.size() < desc.points ? points.size() : desc.points;

	Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	// iterates only for finger tips
	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
		eval += bounds.estimate(poses[i], points.data(), points.data() + size, (Bounds::RealEval)desc.depthStdDev, collisions, maxDist);
		checks += size;
	}

	const Real likelihood = desc.likelihood*((eval + (checks - collisions)) / checks - REAL_ONE);

#ifdef _HBCOLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfEstimate += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().debug("HBCollision::estimate(): points=%u, checks=%u, collisions=%u, eval=%f, likelihhod=%f\n", size, checks, collisions, eval, likelihood);
#endif

	return likelihood;
}

//------------------------------------------------------------------------------

Real HBCollision::evaluate(const Waypoint& waypoint, const Manipulator::Config& config, bool debug) const {
#ifdef _HBCOLLISION_PERFMON
	PerfTimer t;
	++perfEvalPoints;
#endif
	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;

	Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
		eval += bounds.evaluate(points.data(), points.data() + size, (Bounds::RealEval)waypoint.depthStdDev, collisions);
		checks += size;
	}

	const Real likelihood = waypoint.likelihood*((eval + (checks - collisions)) / checks - REAL_ONE);

#ifdef _HBCOLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfEvalPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().debug("HBCollision::evaluate(): points=%u, checks=%u, collisions=%u, likelihhod=%f\n", size, checks, collisions, likelihood);
#endif

	return likelihood;
}

Real HBCollision::evaluate(const Waypoint& waypoint, const Manipulator::Config& config, const FTGuard::Seq &triggeredGuards, bool debug) const {
#ifdef _HBCOLLISION_PERFMON
	PerfTimer t;
	++perfEvalPoints;
#endif
	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;

	Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;
//	manipulator.getContext().write("HBCollision::evaluate():\n");
//	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
//		Bounds bounds = jointBounds[i];
//		if (bounds.empty())
//			continue;
//
//		bounds.setPose(Bounds::Mat34(poses[i]));
//
//		Real force = REAL_ZERO;
//		const bool triggered = [&]() -> const bool {
//			for (auto guard = triggeredGuards.begin(); guard != triggeredGuards.end(); ++guard) {
//				if (i == guard->jointIdx) {
//					force = guard->force;
//					return true;
//				}
//			}
//			return false;
//		}();
//		const Real val = bounds.evaluate(points.data(), points.data() + size, (Bounds::RealEval)waypoint.depthStdDev, (Bounds::RealEval)waypoint.depthStdDev, triggered, poses[i], force, collisions);
////		manipulator.getContext().write("hand link %u, triggered = %u, eval = %f\n", i, triggered, val);
//		eval += val;
//		checks += size;
//	}

	const Real likelihood = waypoint.likelihood*eval/*((eval + (checks - collisions)) / checks - REAL_ONE)*/;

#ifdef _HBCOLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfEvalPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().debug("HBCollision::evaluate(): points=%u, checks=%u, collisions=%u, likelihhod=%f\n", size, checks, collisions, likelihood);
#endif

	return likelihood;
}

//Real HBCollision::evaluate(const FlannDesc& desc, const Manipulator::Pose& rbpose, FTGuard::Seq &triggeredGuards, bool debug) const {
//	if (!this->desc.kdtree) {
//		manipulator.getContext().info("HBCollision::Check(): No KD-Tree\n");
//		// todo: check this is a valid value to return in case of faliure
//		return REAL_ZERO;
//	}
//#ifdef _HBCOLLISION_PERFMON
//	PerfTimer t;
//	++perfEvalPoints;
//#endif
//	const Mat34 pose(rbpose.toMat34());
//	Mat34 poses[Manipulator::JOINTS];
//	manipulator.getPoses(rbpose, pose, poses);
//
//	NNSearch::IndexSeq indices;
//	NNSearch::DistanceF64Seq distances;
//
//	Real eval = REAL_ZERO;
//	size_t collisions = 0, checks = 0;
//
//	for (U32 i = manipulator.getArmJoints()+3; i < manipulator.getJoints();) {
//		Bounds bounds = this->bounds[i];
//		if (bounds.empty())
//			continue;
//
//		bounds.setPose(Bounds::Mat34(i < manipulator.getJoints() ? poses[i] : pose));
//		Real force = REAL_ZERO;
//		const bool triggered = [&]() -> const bool {
//			for (auto guard = triggeredGuards.begin(); guard != triggeredGuards.end(); ++guard) {
//				if (/*i == guard->jointIdx*/guard->jointIdx > i - 3 && guard->jointIdx < i) {
//					force = guard->force;
//					return true;
//				}
//			}
//			return false;
//		}();
//
//		Feature query(poses[i].p);
//		nnSearch->knnSearch(query, desc.neighbours, indices, distances);
//
//		Feature::Seq seq;
//		seq.reserve(indices.size());
//		for (size_t j = 0; j < indices.size(); ++j)
//			seq.push_back(points[indices[j]]);
//
//		eval += bounds.evaluate(seq.data(), seq.data() + seq.size(), (Bounds::RealEval)0.0008, (Bounds::RealEval)desc.depthStdDev * 100, triggered, i < manipulator.getJoints() ? poses[i] : pose, force, collisions);
//		checks += triggered ? indices.size() : 0;
//		i += 4;
//	}
//
//	const Real likelihood = desc.likelihood*eval/collisions/*((eval + (checks - collisions)) / checks - REAL_ONE)*/;
//
//#ifdef _HBCOLLISION_PERFMON
//	SecTmReal t_end = t.elapsed();
//	tperfEvalPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
//	if (debug /*&& eval < REAL_ONE*/)
//		manipulator.getContext().write("HBCollision::evaluate(): points=%u, checks=%u, collisions=%u, likelihhod=%f, eval=%f\n", indices.size(), checks, collisions, likelihood, eval/collisions);
//#endif
//
//	return likelihood;
//}

Real HBCollision::evaluate(const FlannDesc& desc, const Manipulator::Config& config, FTGuard::Seq &triggeredGuards, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("HBCollision::Check(): No KD-Tree\n");
		// todo: check this is a valid value to return in case of faliure
		return REAL_ZERO;
	}
#ifdef _HBCOLLISION_PERFMON
	PerfTimer t;
	++perfEvalPoints;
#endif
	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	NNSearch::IndexSeq indices;
	NNSearch::DistanceF64Seq distances;

	Real eval = REAL_ZERO, c = REAL_ZERO;
	const Real norm = numeric_const<Real>::ONE / ((desc.depthStdDev * 100) * Math::sqrt(2 * numeric_const<Real>::PI));
	size_t collisions = 0, checks = 0;
	static size_t debugjj = 0;

	const U32 jointPerFinger = manipulator.getController().getChains()[manipulator.getHandInfo().getChains().begin()]->getJoints().size();
	const U32 fingers = manipulator.getHandInfo().getChains().size();
	enum jointIndex {
		abductor = 0,
		joint1,
		joint2
	};
	// Utilities
	std::vector<bool> triggeredFingers; triggeredFingers.assign(fingers, false);
	std::vector<RealSeq> fingerGuardSeq;
	fingerGuardSeq.resize(fingers);
	for (std::vector<RealSeq>::iterator i = fingerGuardSeq.begin(); i != fingerGuardSeq.end(); ++i)
		i->assign(jointPerFinger, REAL_ZERO);
	//typedef std::map<U32, FTGuard> GuardMap; GuardMap guardMap;
	typedef std::map<U32, RealSeq> GuardMap; GuardMap guardMap;
	// retrive trigguered guards
	for (FTGuard::Seq::iterator guard = triggeredGuards.begin(); guard != triggeredGuards.end(); ++guard) {
//		triggeredFingers[guard->getHandChain() - 1] = true;
		const U32 k = U32(0 - guard->jointIdx);
//		fingerGuardSeq[guard->getHandChain() - 1][guard->getHandJoint()] = guard->force;
		//manipulator.getContext().write("guard map key %d\n", -k);
//		guardMap.insert(guardMap.begin(), GuardMap::value_type(-k, *guard));
	}
//	if (debugjj % 100 == 0) manipulator.getContext().write("triggered finger [%s,%s,%s,%s,%s], guardMap size %d\n", triggeredFingers[0] ? "T" : "F", triggeredFingers[1] ? "T" : "F", triggeredFingers[2] ? "T" : "F", triggeredFingers[3] ? "T" : "F", triggeredFingers[4] ? "T" : "F", guardMap.size());

	// loops over fingers: thumb = 0,..., pinky = 4
	for (Chainspace::Index j = manipulator.getHandInfo().getChains().begin(); j < manipulator.getHandInfo().getChains().end(); ++j) {
		// thumb: begin=7, end=10
		const Configspace::Index begin = manipulator.getInfo().getJoints(j).begin(), end = manipulator.getInfo().getJoints(j).end();
		const U32 finger = j - manipulator.getHandInfo().getChains().begin();
		// finger frame
		Mat34 fingerFrameInv; fingerFrameInv.setInverse(poses[end - 1]);
		// abductor has no bounds. Retrive contact there at the beginning
		const bool triggeredAbductor = Math::abs(fingerGuardSeq[finger][jointIndex::abductor]) > REAL_ZERO;//false;
		const bool triggeredFlex = Math::abs(fingerGuardSeq[finger][jointIndex::joint1]) > REAL_ZERO || Math::abs(fingerGuardSeq[finger][jointIndex::joint2]) > REAL_ZERO;
		//GuardMap::const_iterator abductor = guardMap.find(begin);
		//if (abductor != guardMap.end())
		//	triggeredAbductor = true;
//		if (debugjj % 100 == 0) manipulator.getContext().write("abductor %d\n", triggeredAbductor);

		for (Configspace::Index i = begin; i < end; ++i) {
			Bounds bounds = jointBounds[i];
			if (bounds.empty())
				continue;

			bounds.setPose(Bounds::Mat34(poses[i]));
			
			// extract the closest point to the joint's bounds as feature
			Feature query(poses[i].p);
			nnSearch->knnSearch(query, desc.neighbours, indices, distances);

			Feature::Seq seq;
			seq.reserve(indices.size());
			for (size_t j = 0; j < indices.size(); ++j)
				seq.push_back(points[indices[j]]);
			
			Real maxDepth = -numeric_const<Real>::MAX;
			U32 boundCollisions = numeric_const<U32>::ZERO;
//			Real force = REAL_ZERO;
//			GuardMap::const_iterator jointGuard = guardMap.find(i);
//			if (jointGuard != guardMap.end()) force = jointGuard->second.force;
////			if (debugjj % 100 == 0) manipulator.getContext().write("joint %d, triggered %s, torque %d\n", i, jointGuard != guardMap.end() ? "T" : "F", force);

			//Mat34 inverse; inverse.setInverse(Bounds::Mat34(i < manipulator.getJoints() ? poses[i] : pose)); // compute the inverse of the joint frame
			Vec3 median;
			median.setZero();
			//for (size_t j = 0; j < seq.size(); ++j) {
			//	const Feature f = seq[j];
			//	const Real depth = bounds.getSurfaceDistance(f);// bounds.getDepth(f, true);
			//	median += f.getPoint();
			//	if (depth > REAL_ZERO) ++boundCollisions; // number of collisions with this bound
			//	if (depth > maxDepth) {
			//		maxDepth = depth; // record max depth
			//	}
			//}
			maxDepth = bounds.distance(seq.data(), seq.data() + seq.size(), median, boundCollisions);

			if (!triggeredFingers[finger] && maxDepth > REAL_ZERO) // the hypothesis intersects a finger that has no contact retrieved
				return REAL_ZERO;
			// if no contact is retrieve and there is no intersection, don't change eval

			// if contact is retrieved
			Real pointEval = REAL_ZERO;
			if (triggeredFingers[finger]) {
				//median /= seq.size();
				Vec3 patchPose; fingerFrameInv.multiply(patchPose, median);
				// abd direction is true if:
				//   1. patchpose is the y-axis side to produce the observed direction of force. (e.g. y>0 && force<0)
				//   2. there is no observed direction (so the patch could be anywhere)
				const bool abdDirection = triggeredAbductor ? !((patchPose.x > REAL_ZERO && fingerGuardSeq[finger][jointIndex::abductor] > REAL_ZERO) || (patchPose.x < REAL_ZERO && fingerGuardSeq[finger][jointIndex::abductor] < REAL_ZERO)) : true;
				const bool flexDirection = triggeredFlex ? !((patchPose.z > REAL_ZERO && (fingerGuardSeq[finger][jointIndex::joint1] > REAL_ZERO || fingerGuardSeq[finger][jointIndex::joint2] > REAL_ZERO)) || (patchPose.z < REAL_ZERO && (fingerGuardSeq[finger][jointIndex::joint1] < REAL_ZERO || fingerGuardSeq[finger][jointIndex::joint2] < REAL_ZERO))) : true;
				const Real scalingFac = abdDirection && flexDirection ? 1 : 0.01;
				//// computes the direction of contact
				//Vec3 v; inverse.multiply(v, median); //v.normalise(); // compute the point in the joint's reference frame
				//const bool adbDirection = triggeredAbductor ? !((v.y > 0 && abductor->second.force >= 0) || (v.y < 0 && abductor->second.force <= 0)) : true;
				//const bool flexDirection = Math::abs(force) > REAL_ZERO ? !((v.z > 0 && force > 0) || (v.z < 0 && force < 0)) : true;
				//const Real coef = Math::abs(maxDepth) >= 1 ? 1 / Math::abs(maxDepth) : Math::abs(maxDepth);
				////if (debugjj % 100 == 0) manipulator.getContext().write("coef %f\n", coef);
				//const Real direction = adbDirection && flexDirection ? 1 /*Math::exp(Real(maxDepth))*/ : Math::exp(Real(maxDepth - 0.01));
				////if (debugjj % 100 == 0) manipulator.getContext().write("direction %f\n", direction);

				//// there is penetration
				//if (maxDepth > REAL_ZERO)
				//	pointEval = Math::exp(-Real(desc.depthStdDev/* * 100*/)*(Real(maxDepth)));
				//else // maxDepth here has to be <= 0. no penetration.
				pointEval = Math::abs(maxDepth) > desc.depthStdDev*3 ? REAL_ZERO : scalingFac*norm*Math::exp(-.5*Math::sqr(Real(maxDepth) / Real(desc.depthStdDev/* * 100*/))); // gaussian 
				//if (debugjj++ % 100 == 0) manipulator.getContext().write("PointEval %f adbDirection %s, flexDirection %s, direction %f\n", pointEval, adbDirection ? "T" : "F", flexDirection ? "T" : "F", direction);
				kahanSum(eval, c, pointEval);
			}
			//if (debugjj++ % 100 == 0)  
			//	manipulator.getContext().write("joint %d [%s, %d] depth %f, eval %f, Lx %f, x %f, collisions %d\n", i, jointGuard != guardMap.end() ? "T" : "F", force, maxDepth, pointEval, 
			//	maxDepth > REAL_ZERO ? -Real(desc.depthStdDev * 100)*(maxDepth) : -.5*Math::sqr(Real(maxDepth) / Real(desc.depthStdDev * 100)), Real(maxDepth), boundCollisions);
			collisions += boundCollisions;
		}
	}

	const Real likelihood = desc.likelihood*eval;// / collisions/*((eval + (checks - collisions)) / checks - REAL_ONE)*/;

#ifdef _HBCOLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfEvalPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	//if (debug /*&& eval < REAL_ONE*/)
//	if (debugjj++ % 100 == 0)	manipulator.getContext().write("HBCollision::evaluate(): points=%u, checks=%u, collisions=%u, likelihhod=%f, eval=%f\n", indices.size(), checks, collisions, likelihood, eval / collisions);
#endif

	return likelihood;
}

Real HBCollision::evaluateFT(DebugRenderer& renderer, const FlannDesc& desc, const Manipulator::Config& config, FTGuard::SeqPtr& triggeredGuards, /*Vec3Seq& handForcesVec,*/ bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("HBCollision::Check(): No KD-Tree\n");
		// todo: check this is a valid value to return in case of faliure
		return REAL_ZERO;
	}
#ifdef _HBCOLLISION_PERFMON
	PerfTimer t;
	++perfEvalPoints;
#endif
	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	NNSearch::IndexSeq indices;
	NNSearch::DistanceF64Seq distances;

	Real eval = REAL_ZERO, c = REAL_ZERO;
	const Real norm = numeric_const<Real>::ONE / ((desc.depthStdDev * 100) * Math::sqrt(2 * numeric_const<Real>::PI));
	U32 collisions = 0, checks = 0;

	const U32 jointPerFinger = 1;// manipulator.getController().getChains()[manipulator.getHandInfo().getChains().begin()]->getJoints().size();
	const U32 fingers = manipulator.getHandInfo().getChains().size();
	enum Direction {
		x = 0,
		y,
		z
	};
	// Utilities
	std::vector<bool> triggeredFingers; triggeredFingers.assign(fingers, false);
	std::vector<Vec3> fingerGuardSeq;
	fingerGuardSeq.assign(fingers, Vec3::zero());
	std::vector<Face> facesInCollision;
	facesInCollision.assign(fingers, Face::UNKNOWN);
	// retrive trigguered guards
	for (FTGuard::SeqPtr::iterator guard = triggeredGuards.begin(); guard != triggeredGuards.end(); ++guard) {
		triggeredFingers[(*guard)->getHandChain()] = (*guard)->isInContact();
		const U32 k = U32(0 - (*guard)->jointIdx);
		fingerGuardSeq[(*guard)->getHandChain()][Direction::x] = Math::abs((*guard)->wrench.getV().x) > (*guard)->limits[Direction::x] ? (*guard)->wrench.getV().x : REAL_ZERO;
		fingerGuardSeq[(*guard)->getHandChain()][Direction::y] = Math::abs((*guard)->wrench.getV().y) > (*guard)->limits[Direction::y] ? (*guard)->wrench.getV().y : REAL_ZERO;
		fingerGuardSeq[(*guard)->getHandChain()][Direction::z] = Math::abs((*guard)->wrench.getV().z) > (*guard)->limits[Direction::z] ? (*guard)->wrench.getV().z : REAL_ZERO;
		facesInCollision[(*guard)->getHandChain()] = (*guard)->faces.empty() ? Face::UNKNOWN : (*guard)->faces[0];

		if (debug) manipulator.getContext().write("Guard[%d] = %s -> face=%s\n", (*guard)->getHandChain(), triggeredFingers[(*guard)->getHandChain()] ? "TRUE" : "FALSE", FaceName[facesInCollision[(*guard)->getHandChain()]]);
	}

	// loops over fingers: thumb = 0,..., pinky = 4
	for (Chainspace::Index i = manipulator.getHandInfo().getChains().begin(); i < manipulator.getHandInfo().getChains().end(); ++i) {
		const U32 finger = i - manipulator.getHandInfo().getChains().begin();
		if (finger > 2)
			break;
		if (debug/* && triggeredFingers[finger]*/) manipulator.getContext().write("evaluateFT(): finger=%d bounds face=%s\n", finger, FaceName[facesInCollision[finger]]);
		// thumb: begin=7, end=10
		const Configspace::Index begin = manipulator.getInfo().getJoints(i).end() - 1, end = manipulator.getInfo().getJoints(i).end();
		// finger frame
		Mat34 fingerFrameInv; fingerFrameInv.setInverse(poses[end - 1]);

		for (Configspace::Index j = begin; j < end; ++j) {
			Bounds bounds = ftBounds[j];
			if (bounds.empty()) {
				if (debug) manipulator.getContext().write("evaluateFT(): joint %d -> empty bounds\n", U32(j - manipulator.getInfo().getJoints().begin()));
				continue;
			}
			if (debug/* && triggeredFingers[finger]*/) manipulator.getContext().write("evaluateFT(): joint %d\n", U32(j - manipulator.getInfo().getJoints().begin()));

			bounds.setPose(Bounds::Mat34(poses[j]));
			if (debug/* && triggeredFingers[finger]*/) {
//				draw(renderer, this->desc.boundsAppearence, poses[j], bounds);
				draw(renderer, this->desc.boundsAppearence, poses[j], bounds, facesInCollision[finger]);
//				renderer.addAxes(poses[j], Vec3(.05, .05, .05));
//				Real normalSize = Real(0.01);
//				Vec3 normalx(1, 0, 0), normaly(0, 1, 0), normalz(0, 0, 1);
//				Vec3 nx, ny, nz;
//				nx.multiply(normalSize, normalx);
//				ny.multiply(normalSize, normaly);
//				nz.multiply(normalSize, normalz);
//				Real vx1 = poses[j].R.m11 * nx.v1 + poses[j].R.m12 * nx.v2 + poses[j].R.m13 * nx.v3;
//				Real vx2 = poses[j].R.m21 * nx.v1 + poses[j].R.m22 * nx.v2 + poses[j].R.m23 * nx.v3;
//				Real vx3 = poses[j].R.m31 * nx.v1 + poses[j].R.m32 * nx.v2 + poses[j].R.m33 * nx.v3;
//				Real vy1 = poses[j].R.m11 * ny.v1 + poses[j].R.m12 * ny.v2 + poses[j].R.m13 * ny.v3;
//				Real vy2 = poses[j].R.m21 * ny.v1 + poses[j].R.m22 * ny.v2 + poses[j].R.m23 * ny.v3;
//				Real vy3 = poses[j].R.m31 * ny.v1 + poses[j].R.m32 * ny.v2 + poses[j].R.m33 * ny.v3;
//				Real vz1 = poses[j].R.m11 * nz.v1 + poses[j].R.m12 * nz.v2 + poses[j].R.m13 * nz.v3;
//				Real vz2 = poses[j].R.m21 * nz.v1 + poses[j].R.m22 * nz.v2 + poses[j].R.m23 * nz.v3;
//				Real vz3 = poses[j].R.m31 * nz.v1 + poses[j].R.m32 * nz.v2 + poses[j].R.m33 * nz.v3;
//				nx.v1 = vx1;
//				nx.v2 = vx2;
//				nx.v3 = vx3;
////				nx.add(poses[j].p, nx);
//				ny.v1 = vy1;
//				ny.v2 = vy2;
//				ny.v3 = vy3;
////				ny.add(poses[j].p, ny);
//				nz.v1 = vz1;
//				nz.v2 = vz2;
//				nz.v3 = vz3;
////				nz.add(poses[j].p, nz);
//				Mat34 origin; origin.setId();
//				renderer.addLine(origin.p/*poses[j].p*/, nx, RGBA::RED);
//				renderer.addLine(origin.p/*poses[j].p*/, ny, RGBA::GREEN);
//				renderer.addLine(origin.p/*poses[j].p*/, nz, RGBA::BLUE);
//				/*Mat34 poseInv; poseInv.setInverse(poses[j]);
//				poseInv.multiply(nx, nx);*/
//				//nx.subtract(nx, poses[j].p);
//				//ny.subtract(ny, poses[j].p);
//				//nz.subtract(nz, poses[j].p);
//				for (auto t = bounds.getTriangles().begin(); t != bounds.getTriangles().end(); ++t) {
//					for (auto t1 = t->begin(); t1 != t->end(); ++t1) {
//						Mat34 m(Mat33::identity(), (Vec3)t1->point);
//						Vec3 n;
//						n.multiply(normalSize, t1->normal);
//						//m.R.multiply(n, n);
//						//Real v1 = poses[j].R.m11 * n.v1 + poses[j].R.m12 * n.v2 + poses[j].R.m13 * n.v3;
//						//Real v2 = poses[j].R.m21 * n.v1 + poses[j].R.m22 * n.v2 + poses[j].R.m23 * n.v3;
//						//Real v3 = poses[j].R.m31 * n.v1 + poses[j].R.m32 * n.v2 + poses[j].R.m33 * n.v3;
//						Real v1 = m.R.m11 * n.v1 + m.R.m12 * n.v2 + m.R.m13 * n.v3;
//						Real v2 = m.R.m21 * n.v1 + m.R.m22 * n.v2 + m.R.m23 * n.v3;
//						Real v3 = m.R.m31 * n.v1 + m.R.m32 * n.v2 + m.R.m33 * n.v3;
//						n.v1 = v1;
//						n.v2 = v2;
//						n.v3 = v3;
//						//n.add(t1->point, n);
//						Vec3 nn;
//						nn.add(t1->point, n);
//						//manipulator.getContext().write("nx [%f %f %f] nn [%f %f %f] normal [%f %f %f]\nnn.dot(nx) = %f nn.dot(nz) = %f normal.dot(nx) = %f normal.dot(nz) = %f\n", nx.x, nx.y, nx.z, n.x, n.y, n.z, t1->normal.x, t1->normal.y, t1->normal.z, 
//						//	n.dot(nx), n.dot(ny), n.dot(nz));
//						manipulator.getContext().write("T=%s nn.dot(nx) = %f nn.dot(ny) = %f nn.dot(nz) = %f\n", FaceName[t1->face], n.dot(nx), n.dot(ny), n.dot(nz));
//						//if (n.dot(ny) < numeric_const<Real>::EPS/*t1->normal.dot(nx) > numeric_const<Real>::EPS && t1->normal.dot(nz) > numeric_const<Real>::EPS*/) {
//						//	manipulator.getContext().write("draw %s!\n", FaceName[t1->face]);
//						//	renderer.addLine(origin.p/*poses[j].p*/, n, RGBA::CYAN);
//						//renderer.addLine(t1->point, nn, RGBA::CYAN);
//						//	//renderer.addAxes(m, Vec3(.005, .005, .005));
//							renderer.addLine(t1->t1, t1->t2, RGBA::RED);
//							renderer.addLine(t1->t2, t1->t3, RGBA::RED);
//							renderer.addLine(t1->t3, t1->t1, RGBA::RED);
//							::Sleep(5000);
//							//	break;
//						//}
//					}
//				}
			}
			// extract the closest point to the joint's bounds as feature
			Feature::Seq seq;
			seq.reserve(bounds.getTriangles().size() * indices.size());
//			size_t tringles = 0;
			for (auto t = bounds.getTriangles().begin(); t != bounds.getTriangles().end(); ++t) {
				for (auto t1 = t->begin(); t1 != t->end(); ++t1) {
//					manipulator.getContext().write("t%d %s\n", tringles++, FaceName[t1->face]);
					if (facesInCollision[finger] != Face::UNKNOWN && t1->face != facesInCollision[finger]) continue;
					Feature query(t1->point);
					nnSearch->knnSearch(query, desc.neighbours, indices, distances);

					for (size_t l = 0; l < indices.size(); ++l)
						seq.push_back(points[indices[l]]);
				}
			}

			Vec3 median;
			median.setZero();
			const Real depth = bounds.distance(seq.data(), seq.data() + seq.size(), median, collisions, facesInCollision[finger]/*, Face::TIP*/);
			if (debug && triggeredFingers[finger]) {
				manipulator.getContext().write("Finger %d: Simulated depth = %.4f seq=%d\n", finger, depth, seq.size());
				draw(seq.begin(), seq.end(), this->desc.featureAppearence, renderer);
				Mat34 m(Mat33::identity(), median);
				renderer.addAxes(m, Vec3(0.005, 0.005, 0.005));
			}

			if (!triggeredFingers[finger] && depth > REAL_ZERO) {// the hypothesis intersects a finger that has no contact retrieved
				if (debug) manipulator.getContext().write("Finger %d: Simulated depth = %.4f. Finger not in contact => return zero!\n", finger, depth);
				return REAL_ZERO;
			}
			// if no contact is retrieve and there is no intersection, don't change eval

			// if contact is retrieved
			Real pointEval = REAL_ZERO;
			if (triggeredFingers[finger]) {
				//median /= seq.size();
				Vec3 patchPose; fingerFrameInv.multiply(patchPose, median);
				if (debug) manipulator.getContext().write("patchPose [%.3f %.3f %.3f]\n", patchPose.x, patchPose.y, patchPose.z);

				// abd direction is true if:
				//   1. patchpose is the y-axis side to produce the observed direction of force. (e.g. y>0 && force<0)
				//   2. there is no observed direction (so the patch could be anywhere)
				//const bool ftX = !((patchPose.x > REAL_ZERO && fingerGuardSeq[finger][Direction::x] < REAL_ZERO) || (patchPose.x < REAL_ZERO && fingerGuardSeq[finger][Direction::x] > REAL_ZERO));
				//const bool ftY = !((patchPose.y > REAL_ZERO && fingerGuardSeq[finger][Direction::y] < REAL_ZERO) || (patchPose.y < REAL_ZERO && fingerGuardSeq[finger][Direction::y] > REAL_ZERO));
				//const bool ftZ = !((patchPose.z > REAL_ZERO && fingerGuardSeq[finger][Direction::z] < REAL_ZERO) || (patchPose.z < REAL_ZERO && fingerGuardSeq[finger][Direction::z] > REAL_ZERO));
				const Real scalingFac = 1.0; // ftX && ftY && ftZ ? 1 : 0.01;
				const Real d = Math::abs(depth);
				pointEval = Math::exp(-.5*Math::sqr(Real(d) / Real(desc.depthStdDev)));
				//pointEval = Math::abs(depth) > desc.depthStdDev * 3 ? REAL_ZERO : scalingFac*norm*Math::exp(-.5*Math::sqr(Real(depth) / Real(desc.depthStdDev/* * 100*/))); // gaussian 
				//if (debugjj++ % 100 == 0) manipulator.getContext().write("PointEval %f adbDirection %s, flexDirection %s, direction %f\n", pointEval, adbDirection ? "T" : "F", flexDirection ? "T" : "F", direction);
				kahanSum(eval, c, pointEval);
				if (debug) manipulator.getContext().write("eval = %.3f * %.3f * e^[-.5*(%.3f / %.3f)^2] = %f\n", scalingFac, norm, Real(depth), Real(desc.depthStdDev),
					pointEval);
			}
		}
	}

	const Real likelihood = desc.likelihood*eval;// / collisions/*((eval + (checks - collisions)) / checks - REAL_ONE)*/;

#ifdef _HBCOLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfEvalPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug)
		manipulator.getContext().write("HBCollision::evaluateFT(): points=%u, checks=%u, collisions=%u, likelihhod=%f\n", indices.size(), checks, collisions, likelihood);
#endif

	return likelihood;
}

Real HBCollision::evaluate(const FlannDesc& desc, const Manipulator::Config& config, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("HBCollision::Check(): No KD-Tree\n");
		// todo: check this is a valid value to return in case of faliure
		return REAL_ZERO;
	}
#ifdef _HBCOLLISION_PERFMON
	PerfTimer t;
	++perfEvalPoints;
#endif
	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	NNSearch::IndexSeq indices;
	NNSearch::DistanceF64Seq distances;

	Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));

		Feature query(poses[i].p);
		nnSearch->knnSearch(query, desc.neighbours, indices, distances);

		Feature::Seq seq;
		seq.reserve(indices.size());
		for (size_t j = 0; j < indices.size(); ++j)
			seq.push_back(points[indices[j]]);

		eval += bounds.evaluate(seq.data(), seq.data() + seq.size(), (Bounds::RealEval)desc.depthStdDev, collisions);
		checks += indices.size();
	}

	const Real likelihood = desc.likelihood*((eval + (checks - collisions)) / checks - REAL_ONE);

#ifdef _HBCOLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfEvalPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().debug("HBCollision::evaluate(): points=%u, checks=%u, collisions=%u, likelihhod=%f\n", indices.size(), checks, collisions, likelihood);
#endif

	return likelihood;
}

Real HBCollision::evaluate(const Manipulator::Waypoint::Seq& path, bool debug) const {
	Real eval = REAL_ZERO;
	for (Waypoint::Seq::const_iterator i = desc.waypoints.begin(); i != desc.waypoints.end(); ++i)
		eval += evaluate(*i, manipulator.interpolate(path, i->pathDist), debug);
	return eval;
}

//------------------------------------------------------------------------------

void HBCollision::draw(const Waypoint& waypoint, const Manipulator::Config& config, DebugRenderer& renderer) const {
	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;

	renderer.setColour(RGBA::BLACK);

	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
		renderer.addAxes(poses[i], Vec3(.05, .05, .05));
		manipulator.getContext().write("-----------------------------\nBound pose <%f, %f, %f> (<%f, %f, %f>)\n", poses[i].p.x, poses[i].p.y, poses[i].p.z, poses[i].p.v[0], poses[i].p.v[1], poses[i].p.v[2]);

		for (size_t j = 0; j < size; ++j) {
			renderer.addPoint(points[j].getPoint());
			if (j % 10 == 0) manipulator.getContext().write("point=%d <%f, %f, %f> distance=%f collision=%s\n", j, points[j].getPoint().x, points[j].getPoint().y, points[j].getPoint().z, bounds.getDepth(points[j]), bounds.getDepth(points[j]) > REAL_ZERO ? "YES" : "NO"
				/*indices[j], distances[j], poses[i].p.distance(points[indices[j]].getPoint())*/);
		}
		// debug: run only for one joint.
		break;
	}

}

void HBCollision::draw(DebugRenderer& renderer, const Rand& rand, const Manipulator::Config& config, const HBCollision::FlannDesc& desc) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("HBCollision::Check(): No KD-Tree\n");
		return;
	}
	PerfTimer t;

	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	NNSearch::IndexSeq indices;
	NNSearch::DistanceF64Seq distances;

	renderer.setColour(RGBA::BLACK);
	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;

	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
		renderer.addAxes(poses[i], Vec3(.05, .05, .05));
		manipulator.getContext().write("-----------------------------\nBound pose <%f, %f, %f> (<%f, %f, %f>)\n", poses[i].p.x, poses[i].p.y, poses[i].p.z, poses[i].p.v[0], poses[i].p.v[1], poses[i].p.v[2]);
//		manipulator.getContext().write("neighbours=%d, points=%d\n", desc.neighbours, desc.points);
		Feature query(poses[i].p);
		nnSearch->knnSearch(query, desc.neighbours, indices, distances);

//		nnSearch->knnSearch((const Real*)&poses[i].p, neighbours, indices, distances);

//		for (size_t j = 0; j < indices.size(); ++j) {
		for (size_t j = 0; j < size; ++j) {
			const Feature f = desc.points < desc.neighbours ? points[indices[size_t(rand.next()) % indices.size()]] : points[indices[j]];
			const Real depth = bounds.getDepth(f);
			if (depth > REAL_ZERO) renderer.addPoint(f.getPoint());
			manipulator.getContext().debug("depth=%f\n", depth);
			if (j % 10 == 0) manipulator.getContext().write("point=%d <%f, %f, %f> index=%d distance=%f depth=%f collision=%s\n", j, f.getPoint().x, f.getPoint().y, f.getPoint().z,
				indices[j], distances[j], bounds.getDepth(points[j]), bounds.getDepth(points[j]) > REAL_ZERO ? "YES" : "NO");
		}

		// debug: run only for one joint.
//		if (i == manipulator.getArmJoints() + 3)
			break;
	}
	SecTmReal t_end = t.elapsed();
	manipulator.getContext().write("HBCollision::draw(): elapse_time=%f neighbours=%u points=%u\n", t_end, desc.neighbours, desc.points);
}

void HBCollision::draw(DebugRenderer& renderer, const Manipulator::Config& config, const HBCollision::FlannDesc& desc) const {
	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	const size_t size = points.size() < desc.points ? points.size() : desc.points;

	renderer.setColour(RGBA::BLACK);

	Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	// iterates only for finger tips
	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
		renderer.addAxes(poses[i], Vec3(.05, .05, .05));
		eval += bounds.estimate(poses[i], points.data(), points.data() + size, (Bounds::RealEval)desc.depthStdDev, collisions, 0.5);
		checks += size;
		
		for (auto k = 0; k < size; ++k)
			renderer.addPoint(points[k].getPoint());
		break;
	}

	const Real likelihood = desc.likelihood*((eval + (checks - collisions)) / checks - REAL_ONE);
	manipulator.getContext().write("HBCollision::draw(): neighbours=%u points=%u\n", desc.neighbours, desc.points);
}

void HBCollision::draw(DebugRenderer& renderer, const Manipulator::Config& config, std::vector<Configspace::Index> &joints, RealSeq &forces, const HBCollision::FlannDesc& desc) const {
	joints.clear();
	forces.clear();

	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	NNSearch::IndexSeq indices;
	NNSearch::DistanceF64Seq distances;

	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;
	renderer.setColour(RGBA::BLACK);

	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
		renderer.addAxes(poses[i], Vec3(.05, .05, .05));

		//		manipulator.getContext().write("-----------------------------\nBound pose <%f, %f, %f> (<%f, %f, %f>)\n", poses[i].p.x, poses[i].p.y, poses[i].p.z, poses[i].p.v[0], poses[i].p.v[1], poses[i].p.v[2]);

		Feature query(poses[i].p);
		//nnSearch->knnSearch(/*(const Real*)&*/poses[i].p/*.x*//*.v*/, neighbours, indices, distances);
		nnSearch->knnSearch(query, desc.neighbours, indices, distances);

		Real force = REAL_ZERO;
		Vec3 frame(.0, .0, .0);
		size_t collisions = 0;
		for (size_t j = 0; j < size; ++j) {
			//			const Real depth = Math::abs(bounds.getDepth(points[indices[j]]/*.getPoint()*/));
			const Feature f = points[indices[j]];
			const Real depth = bounds.getDepth(f);
			manipulator.getContext().debug("depth=%f\n", depth);
			if (depth > REAL_ZERO) {
				renderer.addPoint(f.getPoint());
				force += depth;
				frame.add(frame, getRelativeFrame(poses[i], f.point));
				//Vec3 w; poses[i].multiply(w, frame);
				//const Vec3 p[2] = { poses[i].p, w };
				//renderer.addLine(p[0], p[1], RGBA::MAGENTA);
				++collisions;
			}
			//if (j % 10 == 0) manipulator.getContext().write("point=%d <%f, %f, %f> index=%d distance=%f depth=%f collision=%s\n", j, f.getPoint().x, f.getPoint().y, f.getPoint().z,
			//	indices[j], distances[j], bounds.getDepth(points[j]), bounds.getDepth(f) > REAL_ZERO ? "YES" : "NO");

		}
		if (collisions > 0) {
			joints.push_back(Configspace::Index(i));
			force /= collisions;
			frame /= collisions;
			forces.push_back(Math::sign(REAL_ONE, -frame.z)*force*1000);
			Vec3 w; poses[i].multiply(w, frame);
			const Vec3 p[2] = { poses[i].p, w };
			renderer.addLine(p[0], p[1], RGBA::MAGENTA);
		}
		manipulator.getContext().debug("HBCollision::evaluate(): neighbours=%u, points=%u, collision=%u, force=%f (%f), dir=<%f, %f, %f>\n", desc.neighbours, desc.points, collisions,
			Math::sign(REAL_ONE, -frame.z)*force*100, force, frame.x, frame.y, frame.z);
		break;
	}


}

void HBCollision::draw(DebugRenderer& renderer, const Waypoint& waypoint, const Manipulator::Config& config, const FTGuard::Seq &triggeredGuards, bool debug) const {
	const Mat34 base(config.frame.toMat34());
	WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;

	Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	//for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
	//	Bounds bounds = jointBounds[i];
	//	if (bounds.empty())
	//		continue;

	//	bounds.setPose(Bounds::Mat34(poses[i]));
	//	renderer.addAxes(poses[i], Vec3(.05, .05, .05));

	//	Real force = REAL_ZERO;
	//	const bool triggered = [&]() -> const bool {
	//		for (auto guard = triggeredGuards.begin(); guard != triggeredGuards.end(); ++guard)
	//			if (i == (0 - guard->jointIdx)) {
	//				force = guard->force;
	//				return true;
	//			}
	//		return false;
	//	}();
	//	const Real val = bounds.evaluate(points.data(), points.data() + size, (Bounds::RealEval)waypoint.depthStdDev, (Bounds::RealEval)waypoint.depthStdDev, triggered, poses[i], force, collisions);
	//	eval += val;
	//	checks += size;
	//}

	const Real likelihood = waypoint.likelihood*((eval + (checks - collisions)) / checks - REAL_ONE);

	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().debug("HBCollision::evaluate(): points=%u, checks=%u, collisions=%u, likelihhod=%f\n", size, checks, collisions, likelihood);

}

void HBCollision::draw(Feature::Seq::const_iterator begin, Feature::Seq::const_iterator end, const Feature::Appearance appearence, DebugRenderer& renderer) const {
	for (Feature::Seq::const_iterator p = begin; p != end; ++p)
		p->draw(appearence, renderer);
}

void HBCollision::draw(DebugRenderer& renderer, const Bounds::Appearance appearance, const Mat34& pose, const Bounds& bounds, Face face) const {
	if (appearance.frameShow)
		renderer.addAxes(pose, appearance.frameSize);

	for (auto t0 = bounds.getTriangles().begin(); t0 != bounds.getTriangles().end(); ++t0) {
		for (auto t = t0->begin(); t != t0->end(); ++t) {
			if (face != Face::UNKNOWN && t->face != face)
				continue;
			Mat34 frame(Mat33::identity(), (Vec3)t->point);

			if (appearance.normalShow) {
				Vec3 n;
				n.multiply(appearance.normalSize, t->normal);
				//frame.R.multiply(n, n);
				Real v1 = frame.R.m11 * n.v1 + frame.R.m12 * n.v2 + frame.R.m13 * n.v3;
				Real v2 = frame.R.m21 * n.v1 + frame.R.m22 * n.v2 + frame.R.m23 * n.v3;
				Real v3 = frame.R.m31 * n.v1 + frame.R.m32 * n.v2 + frame.R.m33 * n.v3;
				n.v1 = v1;
				n.v2 = v2;
				n.v3 = v3;
				n.add(t->point, n);
				renderer.addLine(t->point, n, appearance.normalColour);
			}
			if (appearance.boundsShow) {
				//if (face != Face::UNKNOWN) {
				//	renderer.addLine(t->t1, t->t2, face != Face::UNKNOWN ? appearance.boundsColour : RGBA::BLACK);
				//	renderer.addLine(t->t2, t->t3, face != Face::UNKNOWN ? appearance.boundsColour : RGBA::BLACK);
				//	renderer.addLine(t->t3, t->t1, face != Face::UNKNOWN ? appearance.boundsColour : RGBA::BLACK);
				//}
				//renderer.addTriangle(t->t1, t->t2, t->t3);
				renderer.addLine(t->t1, t->t2, appearance.boundsColour);
				renderer.addLine(t->t2, t->t3, appearance.boundsColour);
				renderer.addLine(t->t3, t->t1, appearance.boundsColour);
				::Sleep(5000);
			}
		}
	}
}

//------------------------------------------------------------------------------

void golem::XMLData(HBCollision::Waypoint& val, XMLContext* xmlcontext, bool create) {
	XMLData("path_dist", val.pathDist, xmlcontext, create);
	XMLData("points", val.points, xmlcontext, create);
	XMLData("depth_stddev", val.depthStdDev, xmlcontext, create);
	XMLData("likelihood", val.likelihood, xmlcontext, create);
}

void golem::XMLData(HBCollision::FlannDesc& val, XMLContext* xmlcontext, bool create) {
	XMLData("neighbours", val.neighbours, xmlcontext, create);
	XMLData("points", val.points, xmlcontext, create);
	XMLData("depth_stddev", val.depthStdDev, xmlcontext, create);
	XMLData("likelihood", val.likelihood, xmlcontext, create);
	
	try {
		XMLData("radius", val.radius, xmlcontext, create);
	}
	catch (const Message&) {}
}

void golem::XMLData(HBCollision::FTSensorDesc& val, XMLContext* xmlcontext, bool create) {
	XMLData(val.ftMedian, xmlcontext->getContextFirst("ft_median"), create);
	XMLData(val.ftStd, xmlcontext->getContextFirst("ft_std"), create);
}

void golem::XMLData(HBCollision::Desc& val, XMLContext* xmlcontext, bool create) {
	XMLData(val.waypoints, val.waypoints.max_size(), xmlcontext, "waypoint", create);
	XMLData(val.flannDesc, xmlcontext->getContextFirst("kdtree"), create);
	try {
		XMLData(val.ftBase, xmlcontext->getContextFirst("ft_base"), create);
		XMLData(val.ftContact, xmlcontext->getContextFirst("ft_contact"), create);
	}
	catch (const MsgXMLParser&) { }
}