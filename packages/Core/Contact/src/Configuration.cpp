/** @file Configuration.cpp
 *
 * Robot configuration model
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

#include <Golem/Contact/Configuration.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <Golem/Plugin/Defs.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const std::string golem::Configuration::Space::Header::ID = "golem::Configuration::Space";
const golem::Header::Version golem::Configuration::Space::Header::VERSION = golem::Header::Version({ (1 << 0) /* major */ | (0 << 16) /* minor */ });

//------------------------------------------------------------------------------

bool golem::equals(const Configuration::Path& left, const Configuration::Path& right, const golem::Configspace::Range& range, golem::Real eps) {
	// length
	const size_t size = left.size();
	if (size != right.size())
		return false;

	for (size_t i = 0; i < size; ++i) {
		// config
		if (!left[i].config.equals(right[i].config, range, eps))
			return false;
		
		// frame
		// TODO
		
		// control
		if (left[i].control.size() != right[i].control.size() || !left[i].control.equals(right[i].control, eps))
			return false;
	}

	return true;
}

void Configuration::Path::draw(const Manipulator& manipulator, const Appearance& appearance, golem::DebugRenderer& renderer) const {
	Real first = front().getDistance();
	Real last = back().getDistance();
	const Real zero = Math::clamp(REAL_ZERO, std::min(first, last), std::max(first, last)); // zero always in diam(first, last)
	Math::expand(zero + appearance.pathDelta, first, last); // expand range
	Math::expand(zero - appearance.pathDelta, first, last); // expand range
	const Real delta = (last - first)/appearance.pathSegments; // can be negative!
	const I32 lo = (I32)Math::floor((first - zero)/delta); // always <= 0
	const I32 hi = (I32)Math::ceil((last - zero)/delta); // always >= 0

	Manipulator::Config config[2];
	WorkspaceJointCoord joints[2];

	// limit
	appearance.subspaceDist = Math::clamp(appearance.subspaceDist, lo, hi);
	appearance.subspaceDistLo = first;
	appearance.subspaceDistHi = last;
	appearance.subspaceDistVal = delta*appearance.subspaceDist;

	// path and bounds
	for (I32 i = lo; i <= hi; ++i) {
		const Real t = zero + delta*i;
		const bool showBounds = i == appearance.subspaceDist;
		const bool showVertices = appearance.showVertices && (i == lo || i == hi);
		const bool showEdges = appearance.showEdges && (i > lo);

		config[1] = manipulator.interpolate(*this, t);
		manipulator.getJointFrames(config[1].config, config[1].frame.toMat34(), joints[1]);

		if (showBounds) {
			appearance.subspaceConfig = config[1].config;
			appearance.subspaceFrame = config[1].frame;
			appearance.manipulator.draw(manipulator, appearance.subspaceConfig, renderer);
		}

		for (Chainspace::Index j = manipulator.getHandInfo().getChains().begin(); j < manipulator.getHandInfo().getChains().end(); ++j) {
			const Configspace::Index k = manipulator.getHandInfo().getJoints(j).end() - 1;
			if (showEdges)
				renderer.addLine(joints[0][k].p, joints[1][k].p, appearance.pathColour);
			if (showVertices)
				renderer.addAxes(joints[1][k], appearance.manipulator.jointsFrameSize);
		}

		config[0] = config[1];
		joints[0] = joints[1];
	}
}

//------------------------------------------------------------------------------

void golem::Configuration::Space::Desc::create(const Configuration& configuration) {
	this->poseStdDev.ang = Math::sqrt(REAL_ONE / this->poseStdDev.ang);	// stdDev ~ 1/cov

	poseCovInv.lin = REAL_ONE / (poseCov.lin = Math::sqr(this->poseStdDev.lin));
	poseCovInv.ang = REAL_ONE / (poseCov.ang = Math::sqr(this->poseStdDev.ang));

	for (golem::Configspace::Index i = configuration.getManipulator().getConfigRange().begin(); i < configuration.getManipulator().getConfigRange().end(); ++i)
		configCovInv[i] = REAL_ONE / (configCov[i] = Math::sqr(configStdDev[i]));

	distanceMax = Math::sqr(this->distanceStdDevMax);
}

void golem::Configuration::Space::Desc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData(poseStdDev, xmlcontext->getContextFirst("pose_stddev"), false);

	RealSeq configStdDevSeq;
	golem::XMLDataSeq(configStdDevSeq, "c", xmlcontext->getContextFirst("config_stddev"), false, golem::REAL_ZERO);
	configStdDev.set(configStdDevSeq.begin(), configStdDevSeq.end());

	golem::XMLData("distance_scale", distanceScale, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("distance_stddev", distanceStdDev, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("distance_stddev_max", distanceStdDevMax, const_cast<golem::XMLContext*>(xmlcontext), false);

	golem::XMLData("transform_grad_dist", transformGradDist, const_cast<golem::XMLContext*>(xmlcontext), false);
}

bool golem::equals(const Configuration::Space& left, const Configuration::Space& right, const golem::Configspace::Range& range, golem::Real eps) {
	// name/id
	if (left.name != right.name)
		return false;

	// paths
	if (left.paths.size() != right.paths.size())
		return false;
	for (size_t i = 0; i < left.paths.size(); ++i)
		if (!equals(left.paths[i], right.paths[i], range, eps))
			return false;

	// configs
	//if (left.configs.size() != right.configs.size())
	//	return false;

	return true;
}

//------------------------------------------------------------------------------

Configuration::Configuration(const Desc& desc, Manipulator& manipulator) : manipulator(manipulator), context(manipulator.getContext()) {
	desc.assertValid(Assert::Context("Configuration()."));

	this->desc = desc;
	this->desc.spaceDesc.create(*this);

	clear();
}

Configuration::~Configuration() {
}

//------------------------------------------------------------------------------

Configuration::Path Configuration::create(const golem::WaypointCtrl::Seq& waypoints) const {
	if (waypoints.empty())
		throw Message(Message::LEVEL_ERROR, "Configuration::create(): Empty trajectory");

	Path path(manipulator.create(waypoints, [=](const Manipulator::Config& l, const Manipulator::Config& r) -> Real { return distancePath(desc.spaceDesc, l, r); }));

	// grip waypoint alignment
	//RBCoord inv;
	//inv.setInverse(path.getGrip());
	//for (Path::iterator i = path.begin(); i != path.end(); ++i)
	//	i->multiply(inv, *i);
	Manipulator::Waypoint::alignDistance(path.begin(), path.end(), -path.getGrip().getDistance());

	return path;
}

void Configuration::create(const Path& path, Configuration::Kernel::Seq& kernels) const {
	if (path.weight <= REAL_EPS)
		throw Message(Message::LEVEL_ERROR, "Configuration::create(): Invalid path weight");
	// create kernels
	kernels.reserve(kernels.size() + desc.kernels);
	for (size_t i = 0; i < desc.kernels; ++i) {
		const Real t = desc.spaceDesc.distanceStdDev*(REAL_TWO*i / desc.kernels - REAL_ONE);
		kernels.push_back(Kernel(manipulator.interpolate(path, t), path.weight*Math::exp(-Math::abs(t)))); // throws if empty
	}
}

void Configuration::create(const Path& path, Space& space) const {
	space.paths.push_back(path);
	create(path, space.configs);
	space.desc = desc.spaceDesc;
}

void Configuration::create(Path::PtrSeq& paths, Space::Seq& spaces) const {
	// eps pruning
	for (size_t i = 1; i < paths.size(); ++i) {
		for (size_t j = 0; j < i; ++j)
			if (equals(*paths[j], *paths[i], manipulator.getInfo().getJoints(), desc.pathEps)) {
				paths.erase(paths.begin() + i);
				context.debug("Configuration::create(): pruning path...\n");
				continue;
			}
	}

	// TODO clustering
	// assume one cluster per path
	for (Path::PtrSeq::const_iterator i = paths.begin(); i != paths.end(); ++i) {
		Space space;
		create(**i, space);
		spaces.push_back(space);
	}
}

void Configuration::add(const Space::Seq& spaces) {
	if (spaces.empty())
		throw Message(Message::LEVEL_ERROR, "Configuration::add(): empty sub-space sequence");
	//for (Space::Seq::const_iterator i = spaces.begin(); i != spaces.end(); ++i)
	//	if (i->paths.empty() || i->configs.empty())
	//		throw Message(Message::LEVEL_ERROR, "Configuration::add(): invalid sub-space");

	// add spaces
	this->spaces.insert(this->spaces.end(), spaces.begin(), spaces.end());
	
	// normalise data
	normalise();
}

void Configuration::clear() {
	spaces.clear();
}

void Configuration::normalise() {
	for (Space::Seq::iterator i = spaces.begin(); i != spaces.end(); ++i) {
		if (!golem::Sample<Real>::normalise<golem::Ref1>(i->paths))
			throw Message(Message::LEVEL_ERROR, "Configuration::normalise(): Unable to normalise path distribution");
		if (!golem::Sample<Real>::normalise<golem::Ref1>(i->configs))
			throw Message(Message::LEVEL_ERROR, "Configuration::normalise(): Unable to normalise config distribution");
	}
}

//------------------------------------------------------------------------------

void Configuration::sample(golem::Rand& rand, const Space& space, const ManifoldCtrl& manifold, Manipulator::Config& config) const {
	// sample path
	const Path::Seq::const_iterator ptr = golem::Sample<golem::Real>::sample<golem::Ref1, Path::Seq::const_iterator>(space.paths, rand);
	if (ptr == space.paths.end())
		throw Message(Message::LEVEL_ERROR, "Configuration::sample(): Sampling error");

	Real t;
	do
		t = rand.nextGaussian<Real>(); // mean = 0, stddev = 1
	while (Math::abs(t) > space.desc.distanceStdDev); // rejection sampling

	// sample config given generated mean
	sample(rand, space, manifold, manipulator.interpolate(*ptr, t), RBDist(REAL_ONE, REAL_ONE), config);
}

void Configuration::sample(golem::Rand& rand, const Space& space, const Manipulator::Config& config, Path& next) const {
	// sample path
	const Path::Seq::const_iterator ptr = golem::Sample<golem::Real>::sample<golem::Ref1, Path::Seq::const_iterator>(space.paths, rand);
	if (ptr == space.paths.end())
		throw Message(Message::LEVEL_ERROR, "Configuration::sample(): Sampling error");

	next = *ptr;

	transform(space.desc, config, next);
}

void Configuration::sample(golem::Rand& rand, const Space& space, const ManifoldCtrl& manifold, const Manipulator::Config& mean, const golem::RBDist& magnitude, Manipulator::Config& config) const {
	//// Linear component
	//Vec3 v;
	//v.next(rand); // |v|==1
	//v.multiply(Math::abs(rand.nextGaussian<Real>(REAL_ZERO, magnitude.lin*space.desc.poseStdDev.lin)), v);
	//config.frame.p.add(mean.frame.p, v);
	//// Angular component
	//const Real poseCovInvAng = space.desc.poseCovInv.ang/Math::sqr(magnitude.ang);
	//Quat q;
	//q.next(rand, poseCovInvAng);
	//config.frame.q.multiply(mean.frame.q, q);

	Mat34 trn;
	// Linear component
	if (!manifold.frameDev.v.isZero()) {
		// manifold deviation
	}
	else {
		trn.p.next(rand); // |v|==1
		trn.p.multiply(Math::abs(rand.nextGaussian<Real>(REAL_ZERO, magnitude.lin*space.desc.poseStdDev.lin)), trn.p);
	}
	// Angular component
	if (!manifold.frameDev.w.isZero()) {
		// manifold deviation
	}
	else {
		const Real poseCovInvAng = space.desc.poseCovInv.ang / Math::sqr(magnitude.ang);
		Quat q;
		q.next(rand, poseCovInvAng);
		trn.R.fromQuat(q);
	}
	// sampling in the manifold frame
	config.frame.fromMat34(mean.frame.toMat34() * manifold.frame * trn * ~manifold.frame);

	// Joint configuration
	const Configspace::Range range = manipulator.getConfigRange();
	golem::ConfigspaceCoord configStdDev = space.desc.configStdDev;
	for (Configspace::Index j = range.begin(); j < range.end(); ++j)
		configStdDev[j] *= magnitude.lin;
	rand.nextGaussianArray<Real>(&config.config[range.begin()], &config.config[range.end()], &mean.config[range.begin()], (const Real*)&configStdDev[range.begin()]);
	manipulator.clamp(config);
}

void Configuration::generate(golem::Rand& rand, const Space& space, const Manipulator::Config& config, const Path& prev, Path& next) const {
	// sample path
	//const Path::Seq::const_iterator ptr = golem::Sample<golem::Real>::sample<golem::Ref1, Path::Seq::const_iterator>(space.paths, rand);
	//if (ptr == space.paths.end())
	//	throw Message(Message::LEVEL_ERROR, "Configuration::generate(): Sampling error");

	next = prev;

	transform(space.desc, config, next);
}

golem::Real Configuration::evaluate(const Space& space, const Manipulator::Config& config) const {
	golem::Real likelihood = REAL_ZERO, c = REAL_ZERO;
	for (Kernel::Seq::const_iterator i = space.configs.begin(); i != space.configs.end(); ++i) {
		const Real distance = this->distance(space.desc, *i, config);
		if (distance < space.desc.distanceMax) {
			const Real sampleLikelihood = i->weight*Math::exp(-distance);
			golem::kahanSum(likelihood, c, sampleLikelihood);
		}
	}
	return likelihood / space.configs.back().cdf;
}

//------------------------------------------------------------------------------

golem::Real Configuration::distance(const Space::Desc& desc, const Manipulator::Config& left, const Manipulator::Config& right) const {
	Real d1 = REAL_ZERO;
	const Configspace::Range range = manipulator.getArmInfo().getJoints(); // hand
	for (Configspace::Index i = range.begin(); i < range.end(); ++i)
		d1 += desc.configCovInv[i] * Math::sqr(left.config[i] - right.config[i]);
	return d1;
}

golem::Real Configuration::distancePath(const Space::Desc& desc, const Manipulator::Config& left, const Manipulator::Config& right) const {
	// Config space
	Real d1 = REAL_ZERO;
	const Configspace::Range range = manipulator.getConfigRange(); // arm + hand
	for (Configspace::Index i = range.begin(); i < range.end(); ++i)
		d1 += desc.configCovInv[i] * Math::sqr(left.config[i] - right.config[i]);

	// Cartesian space
	const Real d2 = desc.poseCovInv.dot(RBDist(left.frame, right.frame));

	// linear distance
	return desc.distanceScale*(Math::sqrt(d1) + Math::sqrt(d2));
}

void Configuration::transform(const Space::Desc& desc, const Manipulator::Config& config, Path& path) const {
	if (path.empty())
		throw Message(Message::LEVEL_ERROR, "Configuration::transform(): empty path");

	// transformation to the new frame
	RBCoord inv;
	inv.setInverse(path.getGrip().frame);

	// transform configurations using gradient
	ConfigspaceCoord gradient;
	const Configspace::Range range = manipulator.getInfo().getJoints();
	for (Configspace::Index j = range.begin(); j < range.end(); ++j)
		gradient[j] = config.config[j] - path.getGrip().config[j];

	// transform frames and configurations along the path
	for (Manipulator::Waypoint::Seq::iterator i = path.begin(); i != path.end(); ++i) {
		// frame
		i->frame.multiply(inv, i->frame);
		i->frame.multiply(config.frame, i->frame);
		i->frame.q.normalise(); // must be performed after every step!

		// configuration
		// s==1 at grip, s==1/2 at distance transformGradDist from grip
		const Real s = desc.transformGradDist / (desc.transformGradDist + Math::abs(path.getGrip().weight - i->weight));
		for (Configspace::Index j = range.begin(); j < range.end(); ++j)
			i->config[j] += s*gradient[j];

		// make sure waypoint is feasible
		manipulator.clamp(*i);

		// TODO control variable update
	}
}

//------------------------------------------------------------------------------

void golem::Configuration::Path::Appearance::load(const golem::XMLContext* xmlcontext) {
	manipulator.load(xmlcontext->getContextFirst("manipulator"));

	golem::XMLContext* pxmlcontext = xmlcontext->getContextFirst("path");
	golem::XMLData("show_vertices", showVertices, pxmlcontext, false);
	golem::XMLData("show_edges", showEdges, pxmlcontext, false);
	golem::XMLData("path_segments", pathSegments, pxmlcontext, false);
	golem::XMLData("path_delta", pathDelta, pxmlcontext, false);
	golem::XMLData(pathColour, pxmlcontext->getContextFirst("path_colour"), false);
}

void golem::Configuration::Desc::load(const golem::XMLContext* xmlcontext) {
	spaceDesc.load(xmlcontext);
	
	golem::XMLData("desc_override", spaceDescOverride, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("path_eps", pathEps, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("kernels", kernels, const_cast<golem::XMLContext*>(xmlcontext), false);
}

//------------------------------------------------------------------------------

template <> void golem::Stream::read(golem::Configuration::Path& value) const {
	read(value, value.begin());
}

template <> void golem::Stream::write(const golem::Configuration::Path& value) {
	write(value.begin(), value.end());
}

template <> void golem::Stream::read(golem::Configuration::Space::Seq::value_type& value) const {
	value.header.load(*this);

	read(value.name);
	read(value.paths, value.paths.begin());
	read(value.configs, value.configs.begin());

	if (value.header.getVersionCurrent().version != golem::Header::VERSION_UNDEF) {
		read(value.manifold);
	}

	read(value.desc);
}
template <> void golem::Stream::write(const golem::Configuration::Space::Seq::value_type& value) {
	value.header.store(*this);

	write(value.name);
	write(value.paths.begin(), value.paths.end());
	write(value.configs.begin(), value.configs.end());

	write(value.manifold);

	write(value.desc);
}

//------------------------------------------------------------------------------
