/** @file Query.cpp
 *
 * Query density
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

#include <Golem/Contact/Query.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Query::Query(const Desc& desc, Context& context, const std::string& name) : Sample<golem::Real>(REAL_ZERO), context(context), parallels(context.getParallels()), name(name), contact3DDesc(nullptr) {
	desc.assertValid(Assert::Context("Query()."));

	// check parallels
	if (!parallels || parallels->getNumOfThreads() < 1)
		throw golem::Message(golem::Message::LEVEL_CRIT, "Query(): Parallels required");

	this->desc = desc;
}

void Query::create(const Contact3D::Data& contacts, const data::Point3D& points) {
	// all available contact interfaces, no order or preference
	const Contact3D::TypePtr typePtr(&points);
	// preferred contact interface and description
	const Query::Contact3DDesc::Seq::const_iterator contact3DTypeDesc = Contact3D::TypePtr::select(typePtr, desc.contactDescSeq);

	// update variables
	contact3DDesc = &*contact3DTypeDesc;

	// copy model density
	this->contacts = contacts;

	// run procedure
	if (contact3DTypeDesc->first == Contact3D::TYPE_POINT)
		create(contact3DTypeDesc->second, contacts, *typePtr.points);
	else if (contact3DTypeDesc->first == Contact3D::TYPE_NORMAL)
		create(contact3DTypeDesc->second, contacts, *typePtr.normals);
	else if (contact3DTypeDesc->first == Contact3D::TYPE_FEATURE)
		create(contact3DTypeDesc->second, contacts, *typePtr.features);
	else if (contact3DTypeDesc->first == Contact3D::TYPE_PART)
		create(contact3DTypeDesc->second, contacts, *typePtr.parts);
}

void Query::clear() {
	poses.clear();
}

const Query::Contact3DDesc::TypeDesc& Query::getContact3DDesc() const {
	if (!contact3DDesc)
		throw Message(Message::LEVEL_ERROR, "Query::getContact3DDesc(): unknown contact type");
	return *contact3DDesc;
}

const Query::Contact3DDesc::TypeDesc& Query::getContact3DDesc(const data::Point3D& points) const {
	return *Contact3D::TypePtr::select(Contact3D::TypePtr(&points), desc.contactDescSeq);
}

//------------------------------------------------------------------------------

void Query::create(const Contact3DDesc& desc, const Contact3D::Data& contacts, const data::Point3D& points) {
	// TODO
	throw Message(Message::LEVEL_ERROR, "Query::create(): data::Point3D not implemented");
}

void Query::create(const Contact3DDesc& desc, const Contact3D::Data& contacts, const data::Normal3D& normals) {
	// TODO
	throw Message(Message::LEVEL_ERROR, "Query::create(): data::Normal3D not implemented");
}

void Query::create(const Contact3DDesc& desc, const Contact3D::Data& contacts, const data::Feature3D& features) {
	if (object == nullptr || object->empty())
		throw Message(Message::LEVEL_ERROR, "Query::create(): %s: empty object density", getName().c_str());
	if (contacts.model.empty())
		throw Message(Message::LEVEL_ERROR, "Query::create(): %s: empty model density", getName().c_str());
	if (contacts.model.back().cdf < desc.epsilon)
		throw Message(Message::LEVEL_ERROR, "Query::create(): %s: model density not normalised (%.6e)", getName().c_str(), contacts.model.back().cdf);
	if (features.getSize() <= 0)
		throw Message(Message::LEVEL_ERROR, "Query::create(): %s: no feature points available", getName().c_str());

	// debugging
	const DebugProcess* debugProcess = is<DebugProcess>(&features);
	if (debugProcess) debugProcess->debugReset();

	// Maximum distance between features in squared standard deviations
	const Real featureDistanceMax = Math::sqr(desc.featureStdDevMax);
	// Maximum distance between frames in squared standard deviations
	const Real poseDistanceMax = Math::sqr(desc.poseStdDevMax);

	// frame density properties
	const RBDist frameCov = features.getFrameCovariance();
	const RBCoordDist frameDist(frameCov, frameCov * RBDist(poseDistanceMax, poseDistanceMax));

	// query density kernels
	poses.resize(desc.kernels);
	Pose::Seq::iterator l = poses.begin();
	U32 trialsMax = 0, trialsBandMax = 0;

	// kernels
	Point3DKernel::Dist modelDist(desc.densityDist);
	modelDist.setFeatureCovInv(Point3DKernel::Feature(contacts.feature3DProperty.covarianceInv.begin(), contacts.feature3DProperty.covarianceInv.end()));
	const Real normalisation = desc.weight * Math::exp(-contacts.feature3DProperty.penalty);
	const Real weightEps = desc.epsilon / contacts.model.size() + golem::numeric_const<Real>::EPS;

	// model
	Point3DKernel::Seq model(contacts.model.size());
	for (size_t i = 0; i < model.size(); ++i)
		model[i].set(contacts.model[i], i, true);
	if (!golem::Sample<Point3DKernel::RealEval>::normalise<golem::Ref1>(model))
		throw Message(Message::LEVEL_ERROR, "Query::create(): %s: Unable to normalise model distribution", getName().c_str());

	// create query density
	CriticalSection cs;
	ParallelsTask(parallels, [&] (ParallelsTask*) {
		const U32 jobId = parallels->getCurrentJob()->getJobId();
		Rand rand(RandSeed(this->context.getRandSeed()._U32[0] + jobId, (U32)0));
		Point3DKernel::NNData nnData;
		nnData.resize(desc.nnNeighbours);

		// M(u) without weights
		Pose::Seq neighbours(contacts.model.size());
		std::transform(contacts.model.begin(), contacts.model.end(), neighbours.begin(), [=](const Contact3D& contact) -> Kernel { return Kernel(contact.local, frameDist); });
		U32 trials = 0, trialsBand = 0;

		for (Query::Pose::Seq::iterator i = poses.end();;) {
			{
				CriticalSectionWrapper csw(cs);
				// next query density pose kernel
				if (l == poses.end())
					break;
				i = l++;
				// test 
				if (trialsMax < trials)
					trialsMax = trials;
				if (trialsBandMax < trialsBand)
					trialsBandMax = trialsBand;
				if (trialsBandMax > desc.trialsBand)
					break;
			}

			for (; trialsBand <= desc.trialsBand;) {
				// update bandwidth
				const Real bandwidth = REAL_ONE/Math::pow(desc.trialsBandFac, Real(trialsBand));

				for (trials = 0; trials <= desc.trials;) {
					// sample point from O(v, r)
					const Point3DKernel::Seq::const_iterator featurePtr = golem::Sample<Point3DKernel::RealEval>::sample<golem::Ref1, Point3DKernel::Seq::const_iterator>(*object, rand, desc.epsilon);
					if (featurePtr == object->end())
						throw Message(Message::LEVEL_ERROR, "Query::create(): %s: Unable to sample from object distribution", getName().c_str());

					// point
					const data::Point3D::Point point = features.getPoint(featurePtr->index);
					// Select feature and orientation
					data::Feature3D::Feature feature;
					data::Point3D::Mat33 orientation;
					features.getFeature(featurePtr->index, feature, orientation);

					// global frame (v)
					const golem::Mat34 globalFrame(orientation, point);

					// compute M(u|r) and M(r)
					golem::Real likelihood = REAL_ZERO, c = REAL_ZERO;
					Query::Pose::Seq::iterator k = neighbours.begin();
					for (Contact3D::Seq::const_iterator j = contacts.model.begin(); j != contacts.model.end(); ++j, ++k) {
						const Real distance = bandwidth*feature.distanceWeightSqr(contacts.feature3DProperty.covarianceInv, j->feature);
						if (distance < featureDistanceMax) {
							k->weight = j->weight*Math::exp(-distance);
							golem::kahanSum(likelihood, c, k->weight);
						}
						else
							k->weight = REAL_ZERO;
					}

					// normalise M(u|r)
					if (!golem::Sample<golem::Real>::normalise<golem::Ref1>(neighbours, desc.epsilon)) {
						// this can happen if M(r) ~ 0 (cdf ~ 0), i.e. when the current feature is very different than from model M
						if (trials == desc.trials) {
							this->context.debug("Query::create(): %s: Neighbours normalisation error: cdf=%e < eps=%e, bandwidth=%f\n", getName().c_str(), neighbours.back().cdf, desc.epsilon, REAL_ONE/bandwidth);
						}
						++trials;
						continue;
					}
					// sample M(u|r)
					const Query::Pose::Seq::const_iterator ptr = golem::Sample<golem::Real>::sample<golem::Ref1, Query::Pose::Seq::const_iterator>(neighbours, rand, desc.epsilon);
					if (ptr == neighbours.end()) {
						// this can happen if M(r) ~ 0 (cdf ~ 0), i.e. when the current feature is very different than from model M
						if (trials == desc.trials) {
							this->context.debug("Query::create(): %s: Neighbours distribution sampling error, eps=%e\n", getName().c_str(), desc.epsilon);
						}
						++trials;
						continue;
					}

					// link_frame = contact_frame*sensor_model_frame*model_frame

					// apply sensor model
					features.sampleSensorModel(rand, featurePtr->index, orientation);
					const Mat34 sampleSensorModelFrame(orientation, Vec3::zero());

					// local frame (u)
					RBCoord localFrame;
					localFrame.multiply(RBCoord(sampleSensorModelFrame), *ptr);
					localFrame.q.normalise();

					// generate query density kernel frame
					i->multiply(globalFrame, localFrame);
					i->q.normalise();

					// density distance
					if (desc.densityDistEnable) {
						const golem::Mat34 frame = globalFrame * localFrame.toMat34();//sampleSensorModelFrame;
						//const golem::Real dist = (golem::Real)Point3DKernel::distance(modelDist, Point3DKernel::RBCoord(RBCoord(frame)), model, *object);
						const golem::Real dist = (golem::Real)Point3DKernel::distance(modelDist, *nnSearch, Point3DKernel::RBCoord(RBCoord(frame)), model, *object, nnData);
						i->weight = normalisation * golem::Math::exp(-dist*desc.densityDistBandwidth/modelDist.distanceMax());
						//context.write("%s: wgh=%f, dist=%f, dist_max_{all=%f, lin=%f, ang=%f, feat=%f}\n", getName().c_str(), i->weight, dist, modelDist.distanceMax(), modelDist.distMax.lin, modelDist.distMax.ang, modelDist.featureDistMax);
					}
					else {
						i->weight = normalisation * likelihood;
					}

					i->weight = std::max(i->weight, weightEps);

					// kernel parameters
					i->setCov(frameDist.cov);
					i->setDistMax(frameDist.distMax);

					// done
					goto DONE;
				}

				// update bandwidth
				++trialsBand;
			}

			DONE:;
		}
	});

	// unable to create query density
	if (trialsBandMax > desc.trialsBand) {
		std::string debugStr = "";
		for (size_t i = 0; contacts.feature3DProperty.covarianceInv.size(); ++i)
			debugStr += std::to_string(contacts.feature3DProperty.covarianceInv[i]) + ", ";
		throw Message(Message::LEVEL_ERROR, "Query::create(): %s: Maximum number of trials to create query distribution exceeded (%u > %u), band (%u > %u), feature_cov_inv={%s}", getName().c_str(), trialsMax, desc.trials, trialsBandMax, desc.trialsBand, debugStr.c_str());
	}

	// pose distribution normalisation
	if (!golem::Sample<Real>::normalise<golem::Ref1>(poses))
		throw Message(Message::LEVEL_ERROR, "Query::create(): %s: Unable to normalise query distribution", getName().c_str());
	// query density weight
	weight = poses.back().cdf;// *desc.weight * Math::exp(-contacts.feature3DProperty.penalty);

	// debug string
	std::string debugStr;
	if (debugProcess) debugProcess->debugString(debugStr);

	context.debug("Query::create(): %s: model_{size=%u, cdf=%.5e, feature_{dim=%u, penalty=%.5e, std_dev=(%s), std_dev_scale=(%s)}}, query_{size=%u, cdf=%.5e, trials=%u, trials_band=%u, weight=%.5e, pose_std_dev_{lin=%f, ang=%f}}, debug={%s}\n",
		getName().c_str(),
		contacts.model.size(), contacts.model.back().cdf,
		contacts.feature3DProperty.covarianceSqrt.size(), contacts.feature3DProperty.penalty, data::Feature3D::to_string(contacts.feature3DProperty.covarianceSqrt, "%.3e ").c_str(), data::Feature3D::to_string(contacts.feature3DProperty.scale, "%.3e ").c_str(),
		poses.size(), poses.back().cdf, trialsMax, trialsBandMax, weight, frameDist.covSqrt.lin, frameDist.covSqrt.ang,
		debugStr.c_str()
	);
}

void Query::create(const Contact3DDesc& desc, const Contact3D::Data& contacts, const data::Part3D& parts) {
	if (contacts.model.empty())
		throw Message(Message::LEVEL_ERROR, "Query::create(): %s: empty model density", getName().c_str());
	if (contacts.model.back().cdf < desc.epsilon)
		throw Message(Message::LEVEL_ERROR, "Query::create(): %s: model density not normalised (%.6e)", getName().c_str(), contacts.model.back().cdf);
	
	// All parts
	data::Part3D::Part::Map partMap;
	parts.getPart(partMap);
	if (partMap.empty())
		throw Message(Message::LEVEL_ERROR, "Query::create(): %s: no available parts", getName().c_str());

	// debugging
	const DebugProcess* debugProcess = is<DebugProcess>(&parts);
	if (debugProcess) debugProcess->debugReset();

	// stats
	typedef std::map<data::Part3D::Index, Real> IndexMap;
	IndexMap model, query;
	auto updateIndexMap = [](IndexMap& map, data::Part3D::Index index, Real value) {
		IndexMap::iterator ptr = map.find(index);
		if (ptr == map.end())
			map[index] = value;
		else
			ptr->second += value;
	};

	// Maximum distance between frames in squared standard deviations
	const Real poseDistanceMax = Math::sqr(desc.poseStdDevMax);

	// query density kernels
	poses.clear();
	poses.reserve(partMap.size() * contacts.model.size());

	// model index -> contact map
	typedef std::multimap<data::Part3D::Index, Contact3D> Contact3DMap;
	Contact3DMap contact3DMap;
	for (Contact3D::Seq::const_iterator i = contacts.model.begin(); i != contacts.model.end(); ++i)
		contact3DMap.insert(std::make_pair(i->model, *i));

	// iterate over parts
	data::Part3D::Part::Map::const_iterator i = partMap.begin();

	// create query density
	CriticalSection cs;
	ParallelsTask(parallels, [&](ParallelsTask*) {
		const U32 jobId = parallels->getCurrentJob()->getJobId();
		Rand rand(RandSeed(this->context.getRandSeed()._U32[0] + jobId, (U32)0));

		Pose::Seq poseSeq;

		for (data::Part3D::Part::Map::const_iterator j = partMap.end();;) {
			{
				CriticalSectionWrapper csw(cs);
				poses.insert(poses.end(), poseSeq.begin(), poseSeq.end());
				if (i == partMap.end())
					break;
				j = i++;
				updateIndexMap(model, j->second.model, j->second.weight);
			}

			// clear poses
			poseSeq.clear();

			// select model corresponding contacts
			const std::pair<Contact3DMap::const_iterator, Contact3DMap::const_iterator> range = contact3DMap.equal_range(j->second.model);
			if (range.first == range.second)
				continue;

			// frame density properties
			data::Point3D::RBDist frameCov;
			try {
				frameCov = parts.getFrameCovariance(j->first); // per realisation, not only model, may throw
			}
			catch (const Message& msg) {
				this->context.write(msg);
				continue;
			}
			const RBCoordDist frameDist(frameCov, frameCov * RBDist(poseDistanceMax, poseDistanceMax));

			// generate kernels
			for (Contact3DMap::const_iterator k = range.first; k != range.second; ++k) {
				Pose pose;

				// generate query density kernel frame
				pose.multiply(j->second.frame, k->second.local);
				pose.weight = j->second.weight * k->second.weight;

				// kernel parameters
				pose.setCov(frameDist.cov);
				pose.setDistMax(frameDist.distMax);

				// model
				pose.model = j->second.model;

				poseSeq.push_back(pose);
			}
		}
	});

	// pose distribution normalisation
	if (poses.empty())
		throw Message(Message::LEVEL_ERROR, "Query::create(): %s: No common parts models", getName().c_str());

	// retain only given number of kernels
	const size_t size = poses.size();
	if (size > desc.kernels) {
		std::partial_sort(poses.begin(), poses.begin() + desc.kernels, poses.end(), [] (const Pose& l, const Pose& r) -> bool { return l.weight > r.weight; });
		poses.resize(desc.kernels);
		for (Pose::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i)
			updateIndexMap(query, i->model, i->weight);
	}

	// pose distribution normalisation
	if (!golem::Sample<Real>::normalise<golem::Ref1>(poses))
		throw Message(Message::LEVEL_ERROR, "Query::create(): %s: Unable to normalise query distribution", getName().c_str());
	// query density weight
	weight = desc.weight;// / poses.back().cdf;

	// part list
	std::string queryStr;
	for (IndexMap::const_iterator i = query.begin(); i != query.end(); ++i)
		queryStr += "(" + std::to_string(i->first) + ", " + std::to_string(i->second) + ")";

	// debug string
	std::string debugStr;
	if (debugProcess) debugProcess->debugString(debugStr);

	context.debug("Query::create(): %s: model_{size=%u, cdf=%f, parts={%u}}, query_{size=(%u -> %u), cdf=%f, parts={%u, %s}}, debug={%s}\n",
		getName().c_str(),
		(U32)contacts.model.size(), contacts.model.back().cdf, (U32)model.size(),
		(U32)size, (U32)poses.size(), poses.back().cdf, (U32)query.size(), queryStr.c_str(),
		debugStr.c_str()
	);
}

//------------------------------------------------------------------------------

RBCoord Query::sample(golem::Rand& rand, const Kernel& kernel) const {
	RBCoord pose;

	Vec3 v;
	// Linear component
	v.next(rand); // |v|==1
	v.multiply(Math::abs(rand.nextGaussian<Real>(REAL_ZERO, kernel.covSqrt.lin)), v);
	pose.p.add(kernel.p, v);
	// Angular component
	Quat q;
	q.next(rand, kernel.covInv.ang);
	pose.q.multiply(kernel.q, q);

	return pose;
}

RBCoord Query::sample(golem::Rand& rand) const {
	// sample feature curvature and (local) contact frame
	const Pose::Seq::const_iterator ptr = golem::Sample<golem::Real>::sample<golem::Ref1, Pose::Seq::const_iterator>(poses, rand, getContact3DDesc().second.epsilon);
	if (ptr == poses.end())
		throw Message(Message::LEVEL_ERROR, "Query::sample(): %s: Sampling error", getName().c_str());

	return sample(rand, *ptr);
}

golem::Real Query::evaluate(const RBCoord& coord) const {
	// compute likelihood
	golem::Real likelihood = REAL_ZERO, c = REAL_ZERO;
	for (Pose::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i) {
		const golem::Real dlin = i->p.distanceSqr(coord.p);
		if (dlin < i->distMax.lin) {
			const golem::Real dang = i->q.distance(coord.q);
			if (dang < i->distMax.ang) {
				const golem::Real distance = i->covInv.lin*dlin + i->covInv.ang*dang;
				const golem::Real sampleLikelihood = i->weight*golem::Math::exp(-distance);
				golem::kahanSum(likelihood, c, sampleLikelihood);
			}
		}
	}
	// scale by weight
	return this->weight*likelihood;
}

//------------------------------------------------------------------------------

void golem::Query::Contact3DDesc::load(Contact3D::Type type, const golem::XMLContext* xmlcontext) {
	golem::XMLData("weight", weight, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("kernels", kernels, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("epsilon", epsilon, const_cast<golem::XMLContext*>(xmlcontext), false);
	
	golem::XMLData("trials", trials, const_cast<golem::XMLContext*>(xmlcontext), false);
	try {
		golem::XMLData("trials_band", trialsBand, const_cast<golem::XMLContext*>(xmlcontext), false);
		golem::XMLData("trials_band_fac", trialsBandFac, const_cast<golem::XMLContext*>(xmlcontext), false);
	}
	catch (const golem::MsgXMLParser&) {}
	
	golem::XMLData("feature_std_dev_max", featureStdDevMax, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("pose_std_dev_max", poseStdDevMax, const_cast<golem::XMLContext*>(xmlcontext), false);

	try {
		golem::XMLContext* xmldensitydist = xmlcontext->getContextFirst("density_dist", false);
		golem::XMLData("enable", densityDistEnable, xmldensitydist, false);
		golem::XMLData("bandwidth", densityDistBandwidth, xmldensitydist, false);
		golem::XMLData(densityDist, xmldensitydist, false);
	}
	catch (const golem::MsgXMLParser&) {
		densityDistEnable = false;
	}

	golem::XMLContext* xmlnnsearch = xmlcontext->getContextFirst("nn_search", false);
	golem::XMLData("neighbours", nnNeighbours, xmlnnsearch, false);
	golem::XMLData(nnSearchDesc, xmlnnsearch, false);
}

void golem::Query::Desc::load(const golem::XMLContext* xmlcontext) {
	try {
		golem::XMLData("debug_level", debugLevel, const_cast<golem::XMLContext*>(xmlcontext), false);
	}
	catch (const MsgXMLParser&) {}

	golem::XMLData(contactDescSeq, contactDescSeq.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "contact_3d");
}

//------------------------------------------------------------------------------

void golem::XMLData(golem::Query::Contact3DDesc::Seq::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("type", const_cast<golem::Contact3D::Type&>(val.first), xmlcontext, create);
	val.second.load(val.first, xmlcontext);
}

void golem::XMLData(Query::Desc::Map::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("id", const_cast<std::string&>(val.first), xmlcontext, create);
	val.second.reset(new Query::Desc);
	val.second->load(xmlcontext);
}

//------------------------------------------------------------------------------
