/** @file Manifold.cpp
 *
 * Contact manifold
 *
 * @author Marek Kopicki
 *
 * @copyright  Copyright (C) 2017 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

//------------------------------------------------------------------------------

#include <Golem/Contact/Manifold.h>
#include <Golem/Sys/XMLData.h>
#ifdef WIN32
#pragma warning (push)
#pragma warning (disable : 4291 4244 4996 4305 4267 4334)
#endif
#include <flann/flann.hpp>
#ifdef WIN32
#pragma warning (pop)
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(Manifold::DimDesc::Seq::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("weight", val.weight, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("axis_type_lin", val.axisTypeLin, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("axis_index", val.axisIndex, const_cast<golem::XMLContext*>(xmlcontext));
	
	golem::XMLDataSeq(val.steps, "x", xmlcontext->getContextFirst("steps"), false, golem::REAL_ZERO);
}

//------------------------------------------------------------------------------

void golem::Manifold::Desc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("compressed", manifoldCompressed, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("space_enable", manifoldSpaceEnable, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("view_enable", manifoldViewEnable, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("enable", densityDistEnable, xmlcontext->getContextFirst("feature_3d density_dist", false), false);
	golem::XMLData("bandwidth", densityDistBandwidth, xmlcontext->getContextFirst("feature_3d density_dist", false), false);
	golem::XMLData(densityDist, xmlcontext->getContextFirst("feature_3d density_dist", false), false);

	golem::XMLData("neighbours", nnNeighbours, xmlcontext->getContextFirst("nn_search", false), false);
	golem::XMLData(nnSearchDesc, xmlcontext->getContextFirst("nn_search", false), false);

	golem::XMLData(downsampleDesc, xmlcontext->getContextFirst("downsample", false), false);

	golem::XMLData("waypoint", directionWaypoint, xmlcontext->getContextFirst("view direction", false), false);
	golem::XMLData("var", directionVar, xmlcontext->getContextFirst("view direction", false), false);
	golem::XMLData(positionRange, xmlcontext->getContextFirst("view position range", false), false);
	golem::XMLData(frameDist, xmlcontext->getContextFirst("view frame dist", false), false);
	
	dimDescSeq.clear();
	golem::XMLData(dimDescSeq, dimDescSeq.max_size(), xmlcontext->getContextFirst("view dim"), "desc");
	
	golem::XMLData(dimDist, xmlcontext->getContextFirst("view dim dist", false), false);
	
	optimisationDesc.load(xmlcontext->getContextFirst("view optimisation"));
}

//------------------------------------------------------------------------------

Manifold::Manifold(const Desc& desc, const Configuration& configuration) : configuration(configuration), manipulator(configuration.getManipulator()), context(const_cast<golem::Context&>(manipulator.getContext())) {
	desc.assertValid(Assert::Context("Manifold()."));

	manifoldSpaceEnable = desc.manifoldSpaceEnable;
	manifoldViewEnable = desc.manifoldViewEnable;

	densityDistEnable = desc.densityDistEnable;
	densityDist = desc.densityDist;
	densityDistBandwidth = desc.densityDistBandwidth;

	nnSearchDesc = desc.nnSearchDesc;
	nnNeighbours = desc.nnNeighbours;

	downsampleDesc = desc.downsampleDesc;

	directionWaypoint = desc.directionWaypoint;
	directionVar = desc.directionVar;
	positionRange = desc.positionRange;
	frameDist = desc.frameDist;
	dimDescSeq = desc.dimDescSeq;
	dimDist = desc.dimDist;
	optimisationDesc = desc.optimisationDesc;
}

//------------------------------------------------------------------------------

void golem::Manifold::process(golem::data::ContactModel::Data& data, UICallback* pUICallback) const {
	// manifold estimation
	Contact::View::MapSeq spaceViewMap;
	//typedef std::pair<Real, Cloud::Point> Point;
	typedef Cloud::Point Point;
	//typedef std::vector<Point> PointSeq;
	typedef Cloud::PointSeq PointSeq;
	typedef std::map<U32, PointSeq> SpacePointMap;
	SpacePointMap spacePointMap;

	// points
	auto cloud = [=](const Contact::View& view, PointSeq& points) {
		for (Contact::View::ModelPtr::Map::const_iterator model = view.models.begin(); model != view.models.end(); ++model) {
			Contact3D::Data::Map::const_iterator contact = data.contacts.find(model->second.index);
			if (contact == data.contacts.end())
				throw Message(Message::LEVEL_ERROR, "Manifold::process(): unknown contact index %u", model->second.index);
			//context.write("space=%u, contact=%u\n", view->space, model->second.index);
			for (Contact3D::Seq::const_iterator j = contact->second.model.begin(); j != contact->second.model.end(); ++j) {
				Cloud::Point point;
				Cloud::convertPointXYZ(j->global.p, point);
				Cloud::convertNormal(j->global.toMat34() * Vec3::axisZ(), point);
				Cloud::convertRGBA(RGBA::BLACK, point);
				//points.push_back(std::make_pair(model->second.weight * j->weight, point));
				points.push_back(point);
			}
		}
	};
	// centroid
	auto centroid = [=](const Cloud::PointSeq& points, const Mat34& trn, ManifoldCtrl& manifold) {
		// compute frame estimate
		Mat34 frame = Mat34::identity();
		for (Cloud::PointSeq::const_iterator j = points.begin(); j != points.end(); ++j)
			frame.p += Cloud::getPoint<Real>(*j);
		frame.p /= points.size();

		// set manifold frame estimate
		frame.R = trn.R;
		manifold.frame = ~trn * frame;
		manifold.frameDev.setZero();
		//manifold.frameDev.v.set(1.0, 0.9, 0.8);
		//manifold.frameDev.w.set(1.0, 0.9, 0.8);
	};

	// Space manifolds: Create a map: space -> point cloud
	for (Contact::View::Seq::const_iterator view = data.views.begin(); view != data.views.end(); ++view) {
		const U32 viewIndex = U32(view - data.views.begin());
		spaceViewMap[view->space].push_back(viewIndex);

		context.debug("Manifold::process(): (space -> view) #%u -> %u\n", view->space, viewIndex);

		PointSeq points;
		cloud(*view, points);

		PointSeq& pointsMap = spacePointMap[view->space];
		pointsMap.insert(pointsMap.end(), points.begin(), points.end());
	}

	// Space manifolds: estimate manifold
	for (SpacePointMap::iterator i = spacePointMap.begin(); i != spacePointMap.end(); ++i) {
		context.debug("Manifold::process(): space #%u/%u\n", i->first + 1, (U32)spacePointMap.size());

		// downsampling
		if (downsampleDesc.enabled)
			Cloud::downsampleWithNormals(context, downsampleDesc, i->second, i->second);

		// set manifold frame estimate from centroid, orientation same as gripper one
		if (data.spaces.size() <= (size_t)i->first)
			throw Message(Message::LEVEL_ERROR, "Manifold::process(): invalid space index %u", i->first);
		Configuration::Space& space = data.spaces[i->first];
		const Mat34 trn = space.paths.front().getGrip().frame.toMat34();// take first path

		centroid(i->second, trn, space.manifold);

		// refine
		if (manifoldSpaceEnable)
			processSpaceManifold(i->first, spaceViewMap[i->first], data);
	}

	// view manifolds: estimate manifold
	for (Contact::View::Seq::iterator view = data.views.begin(); view != data.views.end(); ++view) {
		const U32 viewIndex = U32(view - data.views.begin());

		context.debug("Manifold::process(): view #%u/%u\n", viewIndex + 1, (U32)data.views.size());

		// set manifold frame estimate from centroid, orientation same as gripper one
		if (data.spaces.size() <= (size_t)view->space)
			throw Message(Message::LEVEL_ERROR, "Manifold::process(): invalid space index %u", view->space);
		Configuration::Space& space = data.spaces[view->space];
		const Mat34 trn = space.paths.front().getGrip().frame.toMat34();// take first path

		PointSeq points;
		cloud(*view, points);
		centroid(points, trn, view->manifold);

		// refine
		if (manifoldViewEnable)
			processViewManifold(viewIndex, data);
	}
}

//------------------------------------------------------------------------------

void Manifold::processSpaceManifold(U32 spaceIndex, const U32Seq& views, golem::data::ContactModel::Data& data) const {
	// create kernel density
	Configuration::Space& space = data.spaces[spaceIndex];

	// TODO Space manifold
}

void Manifold::processViewManifold(U32 viewIndex, golem::data::ContactModel::Data& data) const {
	Contact::View& view = data.views[viewIndex];

	// Parameters
	//const size_t directionWaypoint = 2;
	//const Real directionVar = Real(100.0);
	//const Vec3 range(Real(0.05), Real(0.05), Real(0.05));
	//const RealSeq lin = { +Real(0.005), +Real(0.01), -Real(0.005), -Real(0.01) }; // [m]
	//const RealSeq ang = { +Math::degToRad(Real(5.0)), +Math::degToRad(Real(10.0)), -Math::degToRad(Real(5.0)), -Math::degToRad(Real(10.0)) }; // [rad]
	//const RBDist dist(Real(10000.0), Real(100.0));
	//Optimisation::Desc optimisationDesc;
	//optimisationDesc.minimum = true;
	//optimisationDesc.crossProb = 0.05;
	//optimisationDesc.generationsNum = 100000;
	//optimisationDesc.testValue = true;
	//optimisationDesc.testVariance = Real(1e-7);

	// View local data
	struct ViewData {
		typedef std::vector<ViewData> Seq;

		U32 modelIndex;
		Point3DKernel::Dist modelDist;
		Point3DKernel::Seq modelPoints;
	};

	ViewData::Seq viewDataSeq;
	viewDataSeq.reserve(view.models.size());

	for (const auto &model : view.models) {
		ViewData viewData;

		// model index
		viewData.modelIndex = model.second.index;

		// distances
		viewData.modelDist = Point3DKernel::Dist(densityDist);
		const Contact3D::Data::Map::const_iterator pcontact = data.contacts.find(viewData.modelIndex);
		if (pcontact == data.contacts.end())
			throw Message(Message::LEVEL_ERROR, "Manifold::processViewManifold(): Unknown model index %u", viewData.modelIndex);
		const Contact3D::Data& contact = pcontact->second;
		context.verbose("model_{index=%u, size=%u, cov_inv=%f}\n", pcontact->first, (U32)contact.model.size(), contact.feature3DProperty.covarianceInv[0]);
		viewData.modelDist.setFeatureCovInv(Point3DKernel::Feature(contact.feature3DProperty.covarianceInv.begin(), contact.feature3DProperty.covarianceInv.end()));

		// model kernels
		viewData.modelPoints.resize(contact.model.size());
		for (size_t i = 0; i < viewData.modelPoints.size(); ++i)
			viewData.modelPoints[i].set(contact.model[i], i, false, false);
		if (!golem::Sample<Point3DKernel::RealEval>::normalise<golem::Ref1>(viewData.modelPoints))
			throw Message(Message::LEVEL_ERROR, "Manifold::processViewManifold(): Unable to normalise model distribution for model index %u", viewData.modelIndex);

		viewDataSeq.push_back(viewData);
	}

	// frame and mean
	if (data.spaces.size() <= (size_t)view.space)
		throw Message(Message::LEVEL_ERROR, "Manifold::processViewManifold(): Invalid space index %u", view.space);
	Configuration::Space& space = data.spaces[view.space];
	const Configuration::Path& path = space.paths.front();
	const Mat34 frame = path.back().frame.toMat34();
	const Mat34 mean = frame * view.manifold.frame;

	// Motion direction of the hand (default Z-axis)
	Mat33 directionRot = frame.R;
	if (path.size() > 1)
		for (Configuration::Path::const_reverse_iterator i = path.rbegin() + std::min((size_t)directionWaypoint, path.size() - 1); i < path.rend(); ++i) {
			const Real eps = Math::sqrt(numeric_const<Real>::EPS);
			// Z-axis is determined by motion direction
			Vec3 z = space.paths.front().rbegin()->frame.p - i->frame.p;
			if (z.magnitude() < eps)
				continue;
			z.normalise();
			// check if direction does not coincide with Z-axis of the inertial frame
			if (Math::equals(z.dot(Vec3::axisZ()), REAL_ONE, eps)) {
				directionRot.setId();
				break;
			}
			// construct rotation matrix from axes
			Vec3 x = z.cross(Vec3::axisZ()), y = x.cross(z);
			x.normalise();
			y.normalise();
			directionRot.fromAxes(x, y, z);
			// done!
			break;
		}
	Vec3 direction = directionRot * Vec3::axisZ();

	// object interface
	const data::Feature3D* features = is<const data::Feature3D>(view.point3D);
	if (!features)
		throw Message(Message::LEVEL_ERROR, "Manifold::processViewManifold(): data::Feature3D interface not provided");

	// clusters
	data::Cluster3D::IndexSet clusterPoints;
	data::Cluster3D::getIndices(data::Cluster3D::CLUSTER_PROCESSING, features, clusterPoints);

	// points
	Point3DKernel::Seq object(clusterPoints.empty() ? features->getSize() : clusterPoints.size());
	context.verbose("%s size %u\n", clusterPoints.empty() ? "Cloud" : "Cluster", (U32)object.size());

	data::Cluster3D::IndexSet::const_iterator clusterPointPtr = clusterPoints.begin();
	for (size_t i = 0; i < object.size(); ++i)
		object[i].set(*features, clusterPoints.empty() ? i : (size_t)*clusterPointPtr++);
	if (!golem::Sample<Point3DKernel::RealEval>::normalise<golem::Ref1>(object))
		throw Message(Message::LEVEL_ERROR, "Manifold::processViewManifold(): Unable to normalise object distribution");

	// initialise nn-search
	typedef golem::KDTree<Point3DKernel::Real, Point3DKernel::NNDist, flann::SearchParams> KDTree;
	flann::SearchParams search;
	flann::KDTreeSingleIndexParams index;
	nnSearchDesc.getKDTreeSingleIndex(search, index);
	//flann::KDTreeIndexParams index;
	//nnSearchDesc.getKDTreeIndex(search, index);
	NNSearch::Ptr nnSearch(new KDTree(search, index, object.front().data(), object.size(), sizeof(Point3DKernel), Point3DKernel::NNDist::DIM, Point3DKernel::NNDist()));

	// initialise
	Heuristic heuristic(context, Vec::N);
	heuristic.pInit = [&](Point3DKernel::NNData& nnData, Rand& rand) {
		nnData.resize(nnNeighbours);
	};
	heuristic.pSample = [&](Rand& rand, Vec& vector, golem::Real& value) -> bool {
		const Real eps = Math::sqrt(numeric_const<Real>::EPS);
		// position
		for (size_t i = 0; i < 3; ++i)
			vector.p[i] = view.manifold.frame.p[i] + rand.nextUniform<Real>(-positionRange[i], positionRange[i]);
		// orientation
		Mat33 rotZ;
		rotZ.rotZ(rand.nextUniform<Real>(REAL_ZERO, REAL_2_PI));
		Vec3 z;
		z.next(rand, directionVar);
		const Vec3 x = Math::equals(z.dot(Vec3::axisY()), REAL_ONE, eps) ? z.cross(Vec3::axisX()) : z.cross(Vec3::axisY()), y = z.cross(x);
		vector.q.fromMat33(Mat33(x, y, z) * rotZ);
		//{
		//	UI::CriticalSectionWrapper csw(getUICallback());
		//	auto& renderer = const_cast<DebugRenderer&>(renderer);
		//	renderer.addAxes(frame * vector.toMat34(), Vec3(0.01));
		//}
		return true;
	};
	heuristic.pProcess = [&](Vec& vector, Point3DKernel::NNData& nnData) {
		vector.q.normalise();
	};

	// evaluation
	auto eval = [=](const Mat34& trn, Point3DKernel::NNData& nnData)->golem::Real {
		const Point3DKernel::RBCoord coord = RBCoord(trn);
		golem::Real value = REAL_ZERO;
		for (const auto &viewData : viewDataSeq) {
			//const golem::Real dist = (golem::Real)Point3DKernel::distance(viewData.modelDist, Point3DKernel::RBCoord(trn), viewData.modelPoints, object);
			const golem::Real dist = (golem::Real)Point3DKernel::distance(viewData.modelDist, *nnSearch, coord, viewData.modelPoints, object, nnData);
			value += dist;// Contact::Likelihood::getLogValue(dist);
		}
		return value;
	};
	typedef std::function<Mat34(const Vec3&)> FuncTrn;
	auto evalAxis = [=](const size_t axis, FuncTrn funcTrn, const RealSeq& seq, const Mat34& frame, Point3DKernel::NNData& nnData)->golem::Real {
		golem::Real value = REAL_ZERO;
		for (const auto &val : seq) {
			Vec3 local = Vec3::zero();
			local[axis] += val;
			const Mat34 trn = frame * funcTrn(local) * ~frame;
			value += eval(trn, nnData);
		}
		return value;
	};
	const FuncTrn funcTrnLin = [](const Vec3& local) -> Mat34 { return Mat34(Mat33::identity(), local); };
	const FuncTrn funcTrnAng = [](const Vec3& local) -> Mat34 { return Mat34(Mat33(local.x, local.y, local.z), Vec3::zero()); };

	heuristic.pValue = [&](const Vec& vector, Point3DKernel::NNData& nnData) -> golem::Real {
		const Mat34 test = frame * vector.toMat34();

		golem::Real dist = REAL_ZERO;
		for (auto& dim : dimDescSeq)
			dist += dim.weight * (dim.axisTypeLin ? evalAxis(dim.axisIndex, funcTrnLin, dim.steps, test, nnData) : evalAxis(dim.axisIndex, funcTrnAng, dim.steps, test, nnData));

		const golem::Real distLinMean = dimDist.lin * frameDist.lin * mean.p.distanceSqr(test.p);

		const golem::Real distAngMean = dimDist.ang * frameDist.ang * (REAL_ONE - direction.dot(test.R * Vec3::axisZ()));

		return dist + distLinMean + distAngMean;
	};

	heuristic.pDistance = [&](const Vec& a, const Vec& b) -> golem::Real {
		return frameDist.dot(RBDist(a, b));
	};

	// run
	Optimisation optimisation(heuristic);
	optimisation.start(optimisationDesc);

	// fetch results
	const size_t solution = optimisation.stop();
	view.manifold.frame = optimisation.getVectors()[solution].toMat34();
	const Real val = optimisation.getValues()[solution]; // objective function value
	const Real var = optimisation.getVariance(); // population distance variance
	const size_t gen = optimisation.getGenerations(); // num of generations processed

	context.debug("Manifold::processViewManifold(): Solution=#%u, val=%.9f, var=%e, gen=%u\n", (U32)solution + 1, val, var, (U32)gen);
}

//------------------------------------------------------------------------------

