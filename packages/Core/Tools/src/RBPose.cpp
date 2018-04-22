/** @file RBPose.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sys/XMLData.h>
#include <Golem/Tools/RBPose.h>

#ifdef WIN32
	#pragma warning (push)
	#pragma warning (disable : 4291 4244 4996 4305 4334)
#endif
#include <Golem/Tools/Cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <flann/flann.hpp>
#ifdef WIN32
	#pragma warning (pop)
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void RBAdjust::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData(increment, xmlcontext->getContextFirst("increment"));

	golem::XMLData("lin_keys", linKeys, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("ang_keys", angKeys, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("inc_keys", incKeys, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData(frameSize, xmlcontext->getContextFirst("frame_size"));
	golem::XMLData(colourSolid, xmlcontext->getContextFirst("colour solid"));
	golem::XMLData(colourWire, xmlcontext->getContextFirst("colour wire"));
}

bool RBAdjust::adjustIncrement(int key) const {
	if (incKeys[1] == key && incrementStep > 0) { --incrementStep; return true;	}
	if (incKeys[0] == key && incrementStep < U32(Math::abs(golem::numeric_const<Real>::MIN_EXP)) - 1) { ++incrementStep; return true; }

	return incKeys[0] == key || incKeys[1] == key;
}

bool RBAdjust::adjustFrame(int key, golem::Mat34& frame) const {
	const RBDist increment = getIncrement();

	if (linKeys[0] == key) { frame.multiply(frame, Mat34(Mat33::identity(), Vec3(+increment.lin, REAL_ZERO, REAL_ZERO))); return true; } else
	if (linKeys[1] == key) { frame.multiply(frame, Mat34(Mat33::identity(), Vec3(-increment.lin, REAL_ZERO, REAL_ZERO))); return true; } else
	if (linKeys[2] == key) { frame.multiply(frame, Mat34(Mat33::identity(), Vec3(REAL_ZERO, +increment.lin, REAL_ZERO))); return true; } else
	if (linKeys[3] == key) { frame.multiply(frame, Mat34(Mat33::identity(), Vec3(REAL_ZERO, -increment.lin, REAL_ZERO))); return true; } else
	if (linKeys[4] == key) { frame.multiply(frame, Mat34(Mat33::identity(), Vec3(REAL_ZERO, REAL_ZERO, +increment.lin))); return true; } else
	if (linKeys[5] == key) { frame.multiply(frame, Mat34(Mat33::identity(), Vec3(REAL_ZERO, REAL_ZERO, -increment.lin))); return true; } else
	if (angKeys[0] == key) { frame.R.multiply(frame.R, Mat33(+increment.ang, REAL_ZERO, REAL_ZERO)); return true; } else
	if (angKeys[1] == key) { frame.R.multiply(frame.R, Mat33(-increment.ang, REAL_ZERO, REAL_ZERO)); return true; } else
	if (angKeys[2] == key) { frame.R.multiply(frame.R, Mat33(REAL_ZERO, +increment.ang, REAL_ZERO)); return true; } else
	if (angKeys[3] == key) { frame.R.multiply(frame.R, Mat33(REAL_ZERO, -increment.ang, REAL_ZERO)); return true; } else
	if (angKeys[4] == key) { frame.R.multiply(frame.R, Mat33(REAL_ZERO, REAL_ZERO, +increment.ang)); return true; } else
	if (angKeys[5] == key) { frame.R.multiply(frame.R, Mat33(REAL_ZERO, REAL_ZERO, -increment.ang)); return true; }
	
	return false;
}

RBDist RBAdjust::getIncrement() const {
	const Real fac = Math::pow(REAL_TWO, -Real(incrementStep));
	return RBDist(increment.lin*fac, increment.ang*fac);
}

//------------------------------------------------------------------------------

void golem::XMLData(RBPose::Euler& val, golem::XMLContext* context, bool create) {
	golem::XMLData("x", val.lin.x, context, create);
	golem::XMLData("y", val.lin.y, context, create);
	golem::XMLData("z", val.lin.z, context, create);
	golem::XMLDataEuler(val.ang.x, val.ang.y, val.ang.z, context, create);
}

void golem::XMLData(RBPose::Generator& val, golem::XMLContext* context, bool create) {
	golem::XMLData("weight", val.weight, context, create);
	golem::XMLData(val.mean, context->getContextFirst("mean"), create);
	golem::XMLData(val.variance, context->getContextFirst("variance"), create);
}

void golem::XMLData(RBPose::Desc& val, golem::XMLContext* context, bool create) {
	golem::XMLData("points", val.points, context, create);
	golem::XMLData("features", val.features, context, create);
	golem::XMLData("attempts", val.attempts, context, create);
	golem::XMLData("kernels", val.kernels, context, create);
	golem::XMLData("neighbours", val.neighbours, context, create);
	golem::XMLData("distance_range", val.distanceRange, context, create);
	golem::XMLData("feature_norm_eps", val.featNormEps, context, create);

	golem::XMLData(val.nnSearchDesc, context->getContextFirst("nn_search"), create);

	golem::XMLData(val.dist, context->getContextFirst("dist"), create);
	golem::XMLData("prod", val.distProd, context->getContextFirst("dist"), create);
	golem::XMLData("feature", val.distFeature, context->getContextFirst("dist"), create);

	golem::XMLData(val.poseStdDev, context->getContextFirst("pose_stddev"), create);

	golem::XMLData(&val.covariance[0], &val.covariance[RBCoord::N], "c", context->getContextFirst("covariance"), create);

	golem::XMLData("population_size", val.populationSize, context->getContextFirst("mean_shift"), create);
	golem::XMLData("generations_min", val.generationsMin, context->getContextFirst("mean_shift"), create);
	golem::XMLData("generations_max", val.generationsMax, context->getContextFirst("mean_shift"), create);
	golem::XMLData("distance_diff", val.distanceDiff, context->getContextFirst("mean_shift"), create);

	try {
		val.localEnabled = true;
		val.optimisationDesc.load(context->getContextFirst("optimisation"));
	}
	catch (const golem::Message&) {
		val.localEnabled = false;
	}
	golem::XMLData("distance_max", val.distanceMax, context, create);

	val.frameAdjust.load(context->getContextFirst("align"));
}

void golem::XMLData(RBPose::Feature::Appearance& val, golem::XMLContext* context, bool create) {
	golem::XMLData(val.lineColour, context->getContextFirst("line"), create);
	golem::XMLData("show", val.normalShow, context->getContextFirst("normal"), create);
	golem::XMLData(val.normalColour, context->getContextFirst("normal"), create);
	golem::XMLData("size", val.normalSize, context->getContextFirst("normal"), create);
	golem::XMLData("show", val.frameShow, context->getContextFirst("frame"), create);
	golem::XMLData(val.frameSize, context->getContextFirst("frame"), create);
}

template <> void golem::Stream::read(golem::RBPose& rbPose) const {
	rbPose.getSamples().clear();
	read(rbPose.getSamples(), rbPose.getSamples().begin());
}

template <> void golem::Stream::write(const golem::RBPose& rbPose) {
	write(rbPose.getSamples().begin(), rbPose.getSamples().end());
}

//------------------------------------------------------------------------------

void RBPose::Feature::draw(const Appearance& appearance, const golem::Mat34& frame, golem::DebugRenderer& renderer) const {
	Vec3 p1, p2, l(length, REAL_ZERO, REAL_ZERO);
	frame.R.multiply(l, l);
	
	// frame origin p = p1
	//p1 = frame.p;
	//p2.add(l, frame.p);
	
	// frame origin p = 0.5*(p1 + p2)
	p1.multiplyAdd(-REAL_HALF, l, frame.p);
	p2.multiplyAdd(+REAL_HALF, l, frame.p);

	// line between points
	renderer.addLine(p1, p2, appearance.lineColour);
	// local frame
	if (appearance.frameShow)
		renderer.addAxes(frame, appearance.frameSize);
	// normals
	if (appearance.normalShow) {
		Vec3 n;
		n.multiply(appearance.normalSize, normal[0]);
		frame.R.multiply(n, n);
		n.add(p1, n);
		renderer.addLine(p1, n, appearance.normalColour);
		n.multiply(appearance.normalSize, normal[1]);
		frame.R.multiply(n, n);
		n.add(p2, n);
		renderer.addLine(p2, n, appearance.normalColour);
	}
}

void RBPose::Feature::draw(const Appearance& appearance, golem::DebugRenderer& renderer) const {
	draw(appearance, this->frame, renderer);
}

//------------------------------------------------------------------------------

golem::RBPose::Ptr golem::RBPose::Desc::create(golem::Context &context) const {
	RBPose::Ptr rbpose(new RBPose(context));
	rbpose->create(*this);
	return rbpose;
}

//------------------------------------------------------------------------------

RBPose::RBPose(golem::Context& context) : context(context), rand(context.getRandSeed()) {
}

RBPose::~RBPose() {
}

void RBPose::create(const Desc& desc) {
	desc.assertValid(Assert::Context("RBPose::Desc."));

	this->desc = desc;

	modelFeatures.resize(this->desc.features);
	queryFeatures.resize(this->desc.kernels);
	poses.resize(this->desc.kernels);


	this->desc.poseStdDev.ang = Math::sqrt(REAL_ONE / this->desc.poseStdDev.ang);	// stdDev ~ 1/cov
	poseCovInv.lin = REAL_ONE / (poseCov.lin = Math::sqr(this->desc.poseStdDev.lin));
	poseCovInv.ang = REAL_ONE / (poseCov.ang = Math::sqr(this->desc.poseStdDev.ang));
}

//------------------------------------------------------------------------------

namespace {
	template <typename _Seq> golem::Mat34 createFrame(const _Seq& points) {
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(points, centroid);
		Eigen::Matrix3f covariance;
		pcl::computeCovarianceMatrix(points, centroid, covariance);
		Eigen::Matrix3f evecs;
		Eigen::Vector3f evals;
		pcl::eigen33(covariance, evecs, evals);
	
		Vec3 x(evecs(0, 0), evecs(1, 0), evecs(2, 0)), y(evecs(0, 1), evecs(1, 1), evecs(2, 1)), z(evecs(0, 2), evecs(1, 2), evecs(2, 2));
		x.normalise();
		y.normalise();
		z.normalise();

		return Mat34(Mat33(x, y, z), Vec3(centroid.x(), centroid.y(), centroid.z()));
	}
};

golem::Mat34 RBPose::createFrame(const Vec3Seq& points) {
	typedef pcl::PointCloud<pcl::PointXYZ> PCLPoints;
	PCLPoints pclpoints;
	Cloud::resize(pclpoints, points.size());
	for (size_t i = 0; i < pclpoints.size(); ++i)
		Cloud::convertPointXYZ(points[i], pclpoints[i]);
	return ::createFrame(pclpoints);
}

void RBPose::createModel(const Feature::Seq& modelFeatures) {
	context.debug("RBPose::createModel(): creating model search index\n");
	flann::SearchParams search;
	flann::KDTreeIndexParams index;
	desc.nnSearchDesc.getKDTreeIndex(search, index);
	typedef golem::KDTree<golem::Real, Feature::FlannDist, flann::SearchParams> KDTree;
	nnSearch.reset(new KDTree(search, index, modelFeatures, Feature::N, Feature::FlannDist(desc.distProd, desc.dist, desc.distFeature)));
}

void RBPose::createQuery(const Feature::Seq& queryFeatures) {
	if (nnSearch == nullptr)
		throw Message(Message::LEVEL_ERROR, "RBPose::createQuery(): model has not been created");

	// knn search
	context.debug("RBPose::createQuery(): searching for model features\n");
	nnSearch->knnSearch(queryFeatures, 1, indices, distances);

	//NNSearch::allocate(queryFeatures.size(), 1, indices, distances);
	//size_t threadIdx = 0, threadCount = (size_t)context.getParallels()->getNumOfThreads();
	//CriticalSection cs;
	//ParallelsTask(context.getParallels(), [&] (ParallelsTask*) {
	//	size_t begin, end;
	//	{
	//		CriticalSectionWrapper csw(cs);
	//		begin = threadIdx*queryFeatures.size()/threadCount;
	//		++threadIdx;
	//		end = threadIdx*queryFeatures.size()/threadCount;
	//	}
	//	nnSearch->knnSearch(reinterpret_cast<const Real*>(queryFeatures.data() + begin), end - begin, sizeof(Feature), 1, indices.data() + begin, distances.data() + begin);
	//});

	//NNSearch::allocate(queryFeatures.size(), 1, indices, distances);
	//size_t i = 0;
	//CriticalSection cs;
	//ParallelsTask(context.getParallels(), [&] (ParallelsTask*) {
	//	for (size_t j;;) {
	//		{
	//			CriticalSectionWrapper csw(cs);
	//			j = ++i;
	//		}
	//		if (j >= queryFeatures.size()) break;
	//		nnSearch->knnSearch(reinterpret_cast<const Real*>(queryFeatures.data() + j), 1, sizeof(Feature), 1, indices.data() + j, distances.data() + j);
	//	}
	//});

	// construct kernels
	context.debug("RBPose::createQuery(): generating distribution kernels\n");
	for (size_t i = 0; i < desc.kernels; ++i) {
		// the nearest neighbour model feature
		const Feature& model = modelFeatures[this->indices[i]];
		// query feature
		const Feature& query = queryFeatures[i];
		// transformation model -> query
		Mat34 m;
		m.setInverse(model.frame);
		m.multiply(query.frame, m);
		// update kernels (all weights equal 1, cdf = i + 1)
		poses[i] = Sample(RBCoord(m), REAL_ONE, i*REAL_ONE);
	}

	// mean and covariance
	if (!pose.create<golem::Ref1, RBPose::Sample::Ref>(golem::RBCoord::N, desc.covariance, poses))
		throw Message(Message::LEVEL_ERROR, "RBPose::createQuery(): Unable to create mean and covariance");

	context.debug("RBPose::createQuery(): covariance = {(%f, %f, %f), (%f, %f, %f, %f)}\n", pose.covariance[0], pose.covariance[1], pose.covariance[2], pose.covariance[3], pose.covariance[4], pose.covariance[5], pose.covariance[6]);
}

//------------------------------------------------------------------------------

bool RBPose::createFeature(const pcl::PointXYZRGBNormal& pp1, const pcl::PointXYZRGBNormal& pp2, Feature& feature) const {
	if (Cloud::isNanXYZ(pp1) || Cloud::isNanXYZ(pp2)) {
		//context.verbose("RBPose::createFeature(): NaN points are not allowed\n");
		return false;
	}
	if (Cloud::isNanNormal(pp1) || Cloud::isNanNormal(pp2)) {
		//context.verbose("RBPose::createFeature(): NaN normals are not allowed\n");
		return false;
	}
	if (!Cloud::isNormalised(pp1, (float)golem::Math::sqrt(desc.featNormEps)) || !Cloud::isNormalised(pp2, (float)golem::Math::sqrt(desc.featNormEps))) {
		context.error("RBPose::createFeature(): Invalid normals (%f, %f)\n", Cloud::getNormal<golem::Real>(pp1).magnitude(), Cloud::getNormal<golem::Real>(pp2).magnitude());
		return false;
	}
	
	const Vec3 p1 = Cloud::getPoint<golem::Real>(pp1);
	const Vec3 n1 = Cloud::getNormal<golem::Real>(pp1);
	const RGBA c1 = Cloud::getColour(pp1);
	const Vec3 p2 = Cloud::getPoint<golem::Real>(pp2);
	const Vec3 n2 = Cloud::getNormal<golem::Real>(pp2);
	const RGBA c2 = Cloud::getColour(pp2);

	feature.feature[0].set(Real(c1._rgba.r)/numeric_const<U8>::MAX, Real(c1._rgba.g)/numeric_const<U8>::MAX, Real(c1._rgba.b)/numeric_const<U8>::MAX);
	feature.feature[1].set(Real(c2._rgba.r)/numeric_const<U8>::MAX, Real(c2._rgba.g)/numeric_const<U8>::MAX, Real(c2._rgba.b)/numeric_const<U8>::MAX);
	//feature.feature[0] = Cloud::getCurvature<golem::Real>(pp1);
	//feature.feature[1] = Cloud::getCurvature<golem::Real>(pp2);

	Vec3 x, y, z;

	//// Version #1: x = |p1 - p2|, y = n1 X x
	//x.subtract(p2, p1);
	//feature.frame.p = p1;
	////feature.frame.p.multiplyAdd(REAL_HALF, x, p1);
	//feature.length = x.normalise();
	//if (Math::equals(Math::abs(x.dot(n1)), REAL_ONE, desc.featNormEps))
	//	return false;
	//y.cross(n1, x);
	//y.normalise();
	//z.cross(x, y);
	//z.normalise();

	// Version #2: x = |p1 - p2|, y = 0.5*(n1 + n2) X x
	Vec3 v(REAL_HALF*(n1.x + n2.x), REAL_HALF*(n1.y + n2.y), REAL_HALF*(n1.z + n2.z));
	if (Math::equals(v.normalise(), REAL_ZERO, desc.featNormEps))
		return false;
	x.subtract(p2, p1);
	//feature.frame.p = p1.point;
	feature.frame.p.multiplyAdd(REAL_HALF, x, p1);
	feature.length = x.normalise();
	if (Math::equals(Math::abs(v.dot(x)), REAL_ONE, desc.featNormEps))
		return false;
	y.cross(v, x);
	y.normalise();
	z.cross(x, y);
	z.normalise();

	// All versions: compute R from x, y, z
	feature.frame.R.fromAxes(x, y, z);
	if (!feature.frame.R.isValid()) {
		context.error("RBPose::createFeature(): Invalid rotation matrix\n");
		return false;
	}
	// All versions: transform normals to the new local frame
	feature.frame.R.multiplyByTranspose(feature.normal[0], n1);
	feature.frame.R.multiplyByTranspose(feature.normal[1], n2);

	return true;
}

//------------------------------------------------------------------------------

golem::RBCoord RBPose::sample() const {
	Sample::Seq::const_iterator ptr = Sample::sample<golem::Ref1, Sample::Seq::const_iterator>(poses, rand);
	if (ptr == poses.end())
		throw Message(Message::LEVEL_ERROR, "RBPose::sample(): Sampling error");
	
	RBCoord c;
	//Vec3 v;
	//v.next(rand); // |v|==1
	//v.multiply(Math::abs(rand.nextGaussian<Real>(REAL_ZERO, desc.poseStdDev.lin)), v);
	//c.p.add(ptr->p, v);
	//Quat q;
	//q.next(rand, poseCovInv.ang);
	//c.q.multiply(ptr->q, q);
	this->rand.nextGaussianArray<golem::Real>(&c[0], &c[0] + RBCoord::N, &(*ptr)[0], &pose.covarianceSqrt[0]); // normalised multivariate Gaussian
	
	return c;
}

golem::Real RBPose::distance(const RBCoord& a, const RBCoord& b) const {
	//return poseCovInv.dot(RBDist(a, b));

	const Real d0 = pose.covarianceInv[0]*golem::Math::sqr(a[0] - b[0]) + pose.covarianceInv[1]*golem::Math::sqr(a[1] - b[1]) + pose.covarianceInv[2]*golem::Math::sqr(a[2] - b[2]);
	const Real d1 = pose.covarianceInv[3]*golem::Math::sqr(a[3] - b[3]) + pose.covarianceInv[4]*golem::Math::sqr(a[4] - b[4]) + pose.covarianceInv[5]*golem::Math::sqr(a[5] - b[5]) + pose.covarianceInv[6]*golem::Math::sqr(a[6] - b[6]);
	const Real d2 = pose.covarianceInv[3]*golem::Math::sqr(a[3] + b[3]) + pose.covarianceInv[4]*golem::Math::sqr(a[4] + b[4]) + pose.covarianceInv[5]*golem::Math::sqr(a[5] + b[5]) + pose.covarianceInv[6]*golem::Math::sqr(a[6] + b[6]);
	return golem::REAL_HALF*(d0 + std::min(d1, d2));
}

golem::Real RBPose::kernel(golem::Real distance) const {
	return golem::Math::exp(-distance); // exponential
}

golem::Real RBPose::density(const RBCoord &c) const {
	Real sum = REAL_ZERO;
	Real buf = REAL_ZERO;
	for (Sample::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i) {
		const Real dist = distance(c, *i);
		if (dist < desc.distanceRange)
			golem::kahanSum(sum, buf, i->weight*kernel(dist));
	}

	return sum;// up to scaling factor
}

void RBPose::alignGlobal(Sample& solution) {
	context.debug("RBPose::alignGlobal(): Global alignment...\n");
	
	size_t k = 0;
	Real solutionEval = REAL_MIN;
	CriticalSection cs;
	ParallelsTask(context.getParallels(), [&] (ParallelsTask*) {
		RBCoord test;
		Real testEval = REAL_MIN;
		for (;;) {
			{
				CriticalSectionWrapper csw(cs);
				if (solutionEval < testEval) {
					solutionEval = testEval;
					solution = test;
				}
				if (++k > desc.populationSize)
					break;
			}

			test = sample();
			for (size_t j = 0; j < desc.generationsMax; ++j) {
				// approximate affine combination of quaternions: renormalisation
				RBCoord mean(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO)), meanBuf(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO));
				Real norm = REAL_ZERO, normBuf = REAL_ZERO;
				for (size_t i = 0; i < desc.neighbours; ++i) {
					const golem::RBPose::Sample s = poses[rand.next()%poses.size()];
					const Real d = distance(test, s);
					if (d < desc.distanceRange) {
						const Real w = s.weight*kernel(d);
						golem::kahanSum(norm, normBuf, w);
						for (size_t l = 0; l < RBCoord::N; ++l)
							golem::kahanSum(mean[l], meanBuf[l], w*s[l]);
					}
				}
				if (norm <= REAL_ZERO)
					break;
				const Real normInv = REAL_ONE/norm;
				mean.p *= normInv;
				mean.q *= normInv;
				mean.q.normalise();
				const Real d = distance(test, mean);
				test = mean;
				if (j > desc.generationsMin && d < desc.distanceDiff)
					break;
			}

			testEval = density(test);
		}
	});

	// print debug message
	context.debug("RBPose::alignGlobal(): Objective value: %f\n", solutionEval);
}

void RBPose::alignLocal(Sample& solution) {
	if (!desc.localEnabled)
		return;

	if (points.empty())
		throw Message(Message::LEVEL_ERROR, "RBPose::alignLocal(): No model points provided");
	if (triangles.empty() || vertices.empty())
		throw Message(Message::LEVEL_ERROR, "RBPose::alignLocal(): No model triangle mesh defined");

	context.debug("RBPose::alignLocal(): Local alignment...\n");

	const Mat34 global(solution.toMat34());
	
	Heuristic heuristic(context);

	// sampling TODO: use more efficient (different than rejection) sampling of quaternions around given direction
	golem::U32 initId = 0;
	heuristic.pSample = [&] (golem::Rand& rand, Heuristic::Vec& vec, golem::Real& value) -> bool {
		if (initId == 0) {
			initId = golem::Thread::getCurrentThreadId();
			vec.getRB(0).p.setZero();
			vec.getRB(0).q.setId();
		}
		else {
			vec.getRB(0).p.next(rand); // |v|==1
			vec.getRB(0).p.multiply(Math::abs(rand.nextGaussian<Real>(REAL_ZERO, desc.poseStdDev.lin)), vec.getRB(0).p);
			vec.getRB(0).q.next(rand, poseCovInv.ang);
		}
		return true;
	};

	// preserve parameter range
	heuristic.pProcess = [=] (Heuristic::Vec& vec, Heuristic::ThreadData&) {
		vec.getRB(0).q.normalise();
	};

	// objective function
	Triangle::Seq cputriangles;
	cputriangles.clear();
	cputriangles.reserve(triangles.size());
	for (TriangleSeq::const_iterator i = triangles.begin(); i != triangles.end(); ++i)
		cputriangles.push_back(Triangle(vertices[i->t1], vertices[i->t2], vertices[i->t3]));
	Real initVal = REAL_ZERO;
	heuristic.pValue = [&] (const Heuristic::Vec& vec, Heuristic::ThreadData&) -> Real {
		Mat34 trn;
		trn.setInverse(global * vec.getRB(0).toMat34());

		Real value = REAL_ZERO;
		size_t n = 0;
		for (VectorSeq::const_iterator i = points.begin(); i != points.end(); ++i) {
			const Vector p(trn * Vec3(i->x, i->y, i->z));
			Float dist = REAL_MAX;
			Vector proj;
			for (Triangle::Seq::const_iterator j = cputriangles.begin(); j != cputriangles.end(); ++j)
				j->distance(p, proj, dist);
			if ((Real)dist < desc.distanceMax) {
				value += Math::sqr((Real)dist);
				++n;
			}
		}

		value = n > 0 ? value*points.size()/n : REAL_MAX;
		
		if (initVal <= REAL_ZERO && initId == golem::Thread::getCurrentThreadId()) {
			initVal  = value;
		}

		return value;
	};

	// distance function
	heuristic.pDistance = [=] (const Heuristic::Vec& a, const Heuristic::Vec& b) -> Real {
		return distance(a.getRB(0), b.getRB(0));
	};
	
	// run computation
	Optimisation optimisation(heuristic);
	optimisation.start(desc.optimisationDesc); // throws

	// get results
	const size_t index = optimisation.stop();
	const Optimisation::Vec local = optimisation.getVectors()[index];
	const Real val = optimisation.getValues()[index]; // objective function value

	context.debug("RBPose::alignLocal(): Objective value %f --> %f...\n", initVal, val);

	// the final solution is
	solution.multiply(solution, local.getRB(0));
}

RBPose::Sample RBPose::maximum() {
	RBPose::Sample solution;
	alignGlobal(solution);
	if (!triangles.empty() || !vertices.empty())
		alignLocal(solution);
	return solution;
}

//------------------------------------------------------------------------------

bool RBPose::align(Sample& solution, DebugRendererCallback rendererCallback, golem::UIKeyboardMouseCallback* callback) {
	bool accept = false;
	Vec3Seq vertices;

	golem::DebugRenderer renderer;
	ScopeGuard guard([&] () {
		renderer.reset();
		rendererCallback(renderer);
	});

	for (bool global = false; !global;) {
		alignLocal(solution);
		if (!callback) return true;

		context.write("Press a key to: accept <Enter>, to apply (G)lobal/(L)ocal alignment, (%s/%s) to adjust position/orientation, (%s) to adjust increment...",
			desc.frameAdjust.linKeys.c_str(), desc.frameAdjust.angKeys.c_str(), desc.frameAdjust.incKeys.c_str());

		Mat34 frame = solution.toMat34();
		for (bool local = false;;) {
			vertices = getVertices();
			for (Vec3Seq::iterator j = vertices.begin(); j != vertices.end(); ++j)
				frame.multiply(*j, *j);

			renderer.reset();
			renderer.setColour(desc.frameAdjust.colourSolid);
			renderer.addSolid(vertices.data(), (U32)vertices.size(), getTriangles().data(), (U32)getTriangles().size());
			renderer.setColour(desc.frameAdjust.colourWire);
			renderer.addWire(vertices.data(), (U32)vertices.size(), getTriangles().data(), (U32)getTriangles().size());
			renderer.addAxes3D(frame, desc.frameAdjust.frameSize);
			rendererCallback(renderer);

			if (local) break;

			const int key = callback->waitKey();
			switch (key) {
			case 27: throw Cancel("\nCancelled");
			case '\x0D': accept = true;
			case 'G': global = true;
			case 'L': local = true;  context.write(" )%c(\n", (char)key);  break;
			}
			if (desc.frameAdjust.adjustIncrement(key))
				context.write("\nInrement: position = %f [m], orientation = %f [deg]", desc.frameAdjust.getIncrement().lin, Math::radToDeg(desc.frameAdjust.getIncrement().ang));
			(void)desc.frameAdjust.adjustFrame(key, frame);
		}
		solution.fromMat34(frame);
	}

	return accept;
}

//------------------------------------------------------------------------------
