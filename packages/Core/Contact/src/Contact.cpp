/** @file Contact.cpp
 *
 * Contact estimator
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

#include <Golem/Contact/Contact.h>
#include <Golem/Contact/OptimisationSA.h>
#include <Golem/Tools/Import.h>
#include <Golem/Math/Clustering.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const std::string golem::Contact::SelectorAny = "Any";

//------------------------------------------------------------------------------

const std::string golem::Contact::Likelihood::Header::ID = "golem::Contact::Likelihood";
const golem::Header::Version golem::Contact::Likelihood::Header::VERSION = golem::Header::Version({ (1 << 0) /* major */ | (1 << 16) /* minor */ });

//------------------------------------------------------------------------------

const std::string golem::Contact::Config::Header::ID = "golem::Contact::Config";
const golem::Header::Version golem::Contact::Config::Header::VERSION = golem::Header::Version({ (1 << 0) /* major */ | (1 << 16) /* minor */ });

//------------------------------------------------------------------------------

const std::string golem::Contact::View::Header::ID = "golem::Contact::View";
const golem::Header::Version golem::Contact::View::Header::VERSION = golem::Header::Version({ (1 << 0) /* major */ | (0 << 16) /* minor */ });

//------------------------------------------------------------------------------

const std::string golem::Contact::Config::Cluster::typeName[golem::Contact::Config::Cluster::TYPE_SIZE] = { "disabled", "type", "likelihood", "configuration" };

void golem::Contact::Config::Cluster::Desc::load(const golem::XMLContext* xmlcontext) {
	ClusteringDesc::load(xmlcontext);
	
	golem::XMLData(distance, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("debug_level", debugLevel, const_cast<golem::XMLContext*>(xmlcontext), false);
}

void golem::Contact::Config::Cluster::find(Context& context, Type type, const Desc& desc, Config::Seq& configs, Cluster::Seq& clusters) {
	switch (type) {
	case TYPE_DISABLED:
		clusters.clear();
		clusters.push_back(Cluster((golem::U32)0, (golem::U32)configs.size()));
		break;
	case TYPE_TYPE:
		findType(context, desc, configs, clusters);
		break;
	case TYPE_LIKELIHOOD:
		findLikelihood(context, desc, configs, clusters);
		break;
	case TYPE_CONFIGURATION:
		findConfiguration(context, desc, configs, clusters);
		break;
	default:
		throw Message(Message::LEVEL_ERROR, "Contact::Config::Cluster::find(): unknown clustering type");
	};
}

void golem::Contact::Config::Cluster::findType(Context& context, const Desc& desc, Contact::Config::Seq& configs, Cluster::Seq& clusters) {
	if (configs.size() < 1)
		throw Message(Message::LEVEL_ERROR, "Contact::Config::Cluster::findType(): no data to cluster");

	std::sort(configs.begin(), configs.end(), [](const Contact::Config::Ptr& l, const Contact::Config::Ptr& r) -> bool {
		return l->type < r->type || l->type == r->type && l->space < r->space || l->type == r->type && l->space == r->space && l->likelihood.value > r->likelihood.value;
	});

	clusters.clear();
	golem::U32 i = 0, j = 0;
	while (++j < configs.size())
		if (configs[j - 1]->type != configs[j]->type || configs[j - 1]->type == configs[j]->type && configs[j - 1]->space != configs[j]->space) {
			clusters.push_back(Cluster(i, j));
			i = j;
		}
	clusters.push_back(Cluster(i, (golem::U32)configs.size()));
}

void golem::Contact::Config::Cluster::findLikelihood(Context& context, const Desc& desc, Contact::Config::Seq& configs, Cluster::Seq& clusters) {
	if (configs.size() < 1)
		throw Message(Message::LEVEL_ERROR, "Contact::Config::Cluster::findLikelihood(): no data to cluster");

	std::sort(configs.begin(), configs.end(), [](const Contact::Config::Ptr& l, const Contact::Config::Ptr& r) -> bool {return l->likelihood.value > r->likelihood.value; });
	// one cluster
	clusters.clear();
	clusters.push_back(Cluster(0, (golem::U32)configs.size()));
}

void golem::Contact::Config::Cluster::findConfiguration(Context& context, const Desc& desc, Contact::Config::Seq& configs, Cluster::Seq& clusters) {
	desc.assertValid(Assert::Context("Contact::Config::Cluster::findConfiguration().desc."));

	if (configs.size() < 1)
		throw Message(Message::LEVEL_ERROR, "Contact::Config::Cluster::findConfiguration(): no data to cluster");

	if (desc.affinityEnabled)
		findConfigurationAffinity(context, desc, configs, clusters);
	else //if (desc.opticsEnabled)
		findConfigurationOPTICS(context, desc, configs, clusters);
}

void golem::Contact::Config::Cluster::findConfigurationAffinity(Context& context, const Desc& desc, Contact::Config::Seq& configs, Cluster::Seq& clusters) {
	/** Similarity */
	typedef std::pair<golem::Real, golem::RBCoord> Similarity;
	/** Similarity sequence */
	typedef std::vector<Similarity> SimilaritySeq;

	/** Clustering */
	typedef golem::Clustering<golem::I32, golem::Real> Clustering;

	const Clustering::Size Size = static_cast<Clustering::Size>(configs.size());

	// frames
	std::vector<RBCoord> frames(Size);
	for (Clustering::Size i = 0; i < static_cast<Clustering::Size>(configs.size()); ++i)
		frames[i] = RBCoord(configs[i]->path.getGrip().frame.toMat34() * configs[i]->manifold.frame);

	// similarity array
	Clustering::ValueSeqSeq similarity;
	Clustering::initialise(Size, similarity, REAL_ZERO, [=](Clustering::Size i, Clustering::Size j) -> golem::Real {
		return -desc.distance.dot(RBDist(frames[i], frames[j]));
	});

	// preferences
	Real median, deviation;
	Clustering::mad(similarity, median, deviation);
	for (Clustering::Size i = 0; i < Size; ++i)
		similarity[i][i] = median + deviation*desc.affinityPreferenceGain + desc.affinityPreferenceOffset;

	// assignments[i] is the exemplar for data point i
	Clustering::SizeSeq assignments;

	// affinity convergence
	auto compare = [](const Clustering::SizeSeq& a, const Clustering::SizeSeq& b) -> bool {
		if (a.size() != b.size())
			return false;
		else for (size_t i = 0; i < a.size(); ++i)
			if (a[i] != b[i])
				return false;
		return true;
	};
	Clustering::Size step = 0;
	Clustering::SizeSeqSeq exemplarsTest(desc.affinityConvergenceCycles);

	// affinity clustering
	Clustering::affinity(similarity, desc.affinityLambda, desc.affinitySteps, assignments, [&](Clustering::Size size, Clustering::Size s, const Clustering::SizeSeq& exemplars) -> bool {
		// new exemplars?
		if (!compare(exemplars, exemplarsTest[0])) {
			// debug
			if (desc.debugLevel >= 2) {
				std::string clusters("");
				for (size_t i = 0; i < exemplars.size(); ++i) {
					clusters += ", (";
					clusters += std::to_string(i + 1);
					clusters += ",";
					clusters += std::to_string(exemplars[i] + 1);
					clusters += ")";
				}
				context.debug("Contact::Config::Cluster::findConfigurationAffinity(): Affinity_{step=%u/%u, test=%u/%u, clusters={%u/%u%s}\n", (U32)s, (U32)desc.affinitySteps, (U32)(step + 1), (U32)desc.affinityConvergenceSteps, (U32)exemplars.size(), (U32)size, clusters.c_str());
			}

			// cycle?
			for (size_t c = 0; ++c < exemplarsTest.size();)
				if (compare(exemplars, exemplarsTest[c]))
					return true;

			// copy
			for (size_t i = exemplarsTest.size(); --i > 0;)
				exemplarsTest[i] = exemplarsTest[i - 1];
			exemplarsTest[0] = exemplars;

			// reset
			step = 0;
		}

		// finish?
		return ++step >= static_cast<Clustering::Size>(desc.affinityConvergenceSteps);
	});

	// extract clusters indices
	typedef std::vector<Clustering::Size> IndexSet;
	typedef std::map<Clustering::Size, IndexSet> ClusterMap;
	ClusterMap clusterMap;
	for (Clustering::Size i = 0; i < (Clustering::Size)assignments.size(); ++i)
		clusterMap[assignments[i]].push_back(i);

	// sort within cluster
	typedef std::map<Real, ClusterMap::const_iterator, std::greater<Real> > ClusterMapRank;
	ClusterMapRank clusterMapRank;
	for (ClusterMap::iterator cluster = clusterMap.begin(); cluster != clusterMap.end(); ++cluster) {
		std::sort(cluster->second.begin(), cluster->second.end(), [&](const Clustering::Size& l, const Clustering::Size& r) -> bool { return configs[l]->likelihood.value > configs[r]->likelihood.value; });
		clusterMapRank.insert(std::make_pair(configs[cluster->second.front()]->likelihood.value, cluster));
	}

	// extract clusters
	Contact::Config::Seq configsTmp;
	Cluster::Seq clustersTmp;
	Cluster clusterDef(0, 0);
	for (ClusterMapRank::iterator cluster = clusterMapRank.begin(); cluster != clusterMapRank.end(); ++cluster) {
		clusterDef.end += (Clustering::Size)cluster->second->second.size();
		
		for (IndexSet::const_iterator index = cluster->second->second.begin(); index != cluster->second->second.end(); ++index)
			configsTmp.push_back(configs[*index]);

		clustersTmp.push_back(clusterDef);
		clusterDef.begin = clusterDef.end;
	}
	configs = configsTmp;
	clusters = clustersTmp;

	//debug
	context.debug("Contact::Config::Cluster::findConfigurationAffinity(): clusters=%u, median=%f, deviation=%e, preference_(gain=%f, offset=%f), dist_{lin=%f, ang=%f}\n", (U32)clusters.size(), median, deviation, desc.affinityPreferenceGain, desc.affinityPreferenceOffset, desc.distance.lin, desc.distance.ang);
}

void golem::Contact::Config::Cluster::findConfigurationOPTICS(Context& context, const Desc& desc, Contact::Config::Seq& configs, Cluster::Seq& clusters) {
	// frames
	std::vector<RBCoord> frames(configs.size());
	for (size_t i = 0; i < configs.size(); ++i)
		frames[i] = RBCoord(configs[i]->path.getGrip().frame.toMat34() * configs[i]->manifold.frame);

	// find clusters
	typedef golem::Clustering<size_t, Real> Clustering;
	Clustering::SizeSeq indices; indices.reserve(configs.size()); // data indices
	Clustering::SizeSeq indexClusters; // indices to data indices
	Clustering::optics(numeric_const<Real>::MAX, configs.size(), desc.opticsDensity, desc.opticsRadius, indices, indexClusters, [=](size_t index, Clustering::SizeSeq& neighbours, Clustering::ValueSeq& distances) {
		// TODO use proper nn-search here!
		typedef std::multimap<Real, size_t> Rank;
		Rank rank;
		for (size_t i = 0; i < configs.size(); ++i)
			if (i != index) {
				const Real dist = desc.distance.dot(RBDist(frames[i], frames[index]));
				if (dist < desc.opticsRadius)
					rank.insert(Rank::value_type(dist, i));
			}

		neighbours.clear();
		distances.clear();
		for (Rank::const_iterator i = rank.begin(); i != rank.end(); ++i) {
			neighbours.push_back(i->second);
			distances.push_back(i->first);
		}
	});
	if (indices.empty() || indexClusters.empty())
		throw Message(Message::LEVEL_ERROR, "Contact::Config::Cluster::findConfigurationOPTICS(): unable to find clusters");

	// extract outliers (as the last cluster), if there are any
	if (indices.size() < configs.size()) {
		indexClusters.push_back(indices.size());// outliers cluster
		Clustering::SizeSeq outliers(configs.size()); // outliers indices
		for (size_t i = 0; i < outliers.size(); ++i)
			outliers[i] = i;
		for (Clustering::SizeSeq::const_iterator i = indices.begin(); i != indices.end(); ++i)
			outliers[*i] = golem::numeric_const<size_t>::MAX;
		for (Clustering::SizeSeq::const_iterator i = outliers.begin(); i != outliers.end(); ++i)
			if (*i != golem::numeric_const<size_t>::MAX) indices.push_back(*i);
	}

	// extract clusters, sort in descending likelihood order within each cluster
	Contact::Config::Seq tmpConfigs;
	Cluster::Seq tmpClusters;
	typedef std::map<Real, golem::U32, std::greater<Real> > Rank; // largest/most likely first
	Rank rank;
	for (Clustering::SizeSeq::const_iterator i = indexClusters.begin(); i != indexClusters.end(); ++i) {
		Cluster cluster;
		cluster.begin = (golem::U32)tmpConfigs.size();
		for (Clustering::SizeSeq::const_iterator j = indices.begin() + *i; j != ((i + 1) != indexClusters.end() ? indices.begin() + *(i + 1) : indices.end()); ++j)
			tmpConfigs.push_back(configs[*j]);
		cluster.end = (golem::U32)tmpConfigs.size();
		std::sort(tmpConfigs.data() + cluster.begin, tmpConfigs.data() + cluster.end, [](const Contact::Config::Ptr& l, const Contact::Config::Ptr& r) -> bool {return l->likelihood.value > r->likelihood.value; });
		rank[(i + 1) != indexClusters.end() ? tmpConfigs[cluster.begin]->likelihood.value : golem::numeric_const<golem::Real>::NEG_INF] = (golem::U32)tmpClusters.size(); // best solution in the current cluster
		tmpClusters.push_back(cluster);
	}

	// copy clusters in order of decreasing likelihood of the best solusion in the cluster
	configs.clear();
	clusters.clear();
	for (Rank::const_iterator i = rank.begin(); i != rank.end(); ++i) {
		Cluster cluster;
		cluster.begin = (golem::U32)configs.size();
		for (size_t j = tmpClusters[i->second].begin; j < tmpClusters[i->second].end; ++j)
			configs.push_back(tmpConfigs[j]);
		cluster.end = (golem::U32)configs.size();
		clusters.push_back(cluster);
	}
}

golem::Contact::Config::Cluster::Type golem::Contact::Config::Cluster::fromString(const std::string& str) {
	for (U32 i = 0; i < (U32)golem::Contact::Config::Cluster::TYPE_SIZE; ++i)
		if (str == typeName[i])
			return (golem::Contact::Config::Cluster::Type)i;

	throw Message(Message::LEVEL_ERROR, "Contact::Config::Cluster::fromString(): unknown clustering type: %s", str.c_str());
}

//------------------------------------------------------------------------------

void golem::Contact::Config::Appearance::load(const golem::XMLContext* context) {
	golem::XMLContext* const xmlconfig = context->getContextFirst("config");
	golem::XMLData("show", showConfig, xmlconfig, false);
	configPath.load(xmlconfig);
	manifold.load(xmlconfig->getContextFirst("manifold"));

	golem::XMLContext* const xmlmodel = context->getContextFirst("model");
	golem::XMLData("config_show", showModelConfig, xmlmodel, false);
	golem::XMLData("contact_show", showModelContact, xmlmodel, false);
	golem::XMLData("frames_show", showModelFrames, xmlmodel, false);

	modelManipulator.load(xmlmodel->getContextFirst("manipulator"));

	golem::XMLData("distrib_samples", modelDistribSamples, xmlmodel, false);
	golem::XMLData("distrib_bounds", modelDistribBounds, xmlmodel, false);

	golem::XMLData(modelSamplesColour, xmlmodel->getContextFirst("sample_colour"), false);
	golem::XMLData("sample_point_size", modelSamplesPointSize, xmlmodel, false);
	golem::XMLData(modelSamplesFrameSize, xmlmodel->getContextFirst("sample_frame_size"), false);
}

//------------------------------------------------------------------------------

void Contact::Config::draw(const Manipulator& manipulator, const Appearance& appearance, golem::Rand& rand, golem::DebugRenderer& renderer) const {
	if (appearance.showConfig) {
		golem::DebugRenderer buffer;
		
		path.draw(manipulator, appearance.configPath, buffer);
		appearance.manifold.draw(manifold, appearance.configPath.subspaceFrame.toMat34(), buffer);

		renderer.add(buffer);
	}

	if (appearance.showModelConfig && contact != nullptr && space < contact->getConfiguration()->getSpaces().size()) {
		renderer.setPointSize(appearance.modelSamplesPointSize);

		for (size_t i = 0; i < appearance.modelDistribSamples; ++i) {
			Manipulator::Config config;
			// TODO View manifold
			contact->getConfiguration()->sample(rand, contact->getConfiguration()->getSpaces()[space], contact->getViews()[view].manifold, config);

			if (i < appearance.modelDistribBounds) {
				appearance.modelManipulator.draw(manipulator, config, renderer);
			}

			WorkspaceJointCoord joints;
			const golem::Mat34 base = config.frame.toMat34();
			manipulator.getJointFrames(config.config, base, joints);
			for (Chainspace::Index j = manipulator.getHandInfo().getChains().begin(); j < manipulator.getHandInfo().getChains().end(); ++j) {
				const Configspace::Index k = manipulator.getHandInfo().getJoints(j).end() - 1;
				appearance.showModelFrames ? renderer.addAxes(joints[k], appearance.modelSamplesFrameSize) : renderer.addPoint(joints[k].p, appearance.modelSamplesColour);
			}
		}
	}

	if (appearance.showModelContact && contact != nullptr && !contact->getQuery().empty()) {
		const Query* query = contact->getQuery()[Math::clamp(appearance.modelSelectionIndex, (U32)0, (U32)contact->getQuery().size() - 1)];
		if (query) {
			renderer.setPointSize(appearance.modelSamplesPointSize);

			for (size_t i = 0; i < appearance.modelDistribSamples; ++i) {
				const RBCoord rbframe = query->sample(rand);
				const Mat34 frame = rbframe.toMat34();

				//if (i < appearance.modelDistribBounds) {
				//	Manipulator::Pose pose(contact->getConfiguration()->sample());
				//	// joints
				//	if (contactIndex < manipulator->getJoints()) {
				//		Mat34 trn[Manipulator::JOINTS], delta;
				//		manipulator->getPoses(pose, trn);
				//		// trn*delta = pose ==> delta = trn^-1*pose
				//		delta.setInverseRT(trn[contactIndex]);
				//		delta.multiply(delta, pose);
				//		pose.multiply(frame, delta);
				//	}
				//	// base
				//	else {
				//		pose = frame;
				//	}
				//	addBounds(pose);
				//}

				appearance.showModelFrames ? renderer.addAxes(frame, appearance.modelSamplesFrameSize) : renderer.addPoint(frame.p, appearance.modelSamplesColour);
			}
		}
	}
}

//------------------------------------------------------------------------------

const std::string golem::Contact::ManifoldSelector::TypeAny = "Any";

void golem::Contact::ManifoldSelector::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData(*this, const_cast<golem::XMLContext*>(xmlcontext), false);
}

void golem::XMLData(golem::Contact::ManifoldSelector& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("type", val.type, xmlcontext, create);
	golem::XMLData("space", val.space, xmlcontext, create);
	try {
		golem::XMLData("view", val.view, xmlcontext, create);
	}
	catch (const golem::MsgXMLParserAttributeNotFound&) {}
}

void golem::XMLData(golem::Contact::ManifoldSelector::Map::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData(const_cast<golem::Contact::ManifoldSelector&>(val.first), const_cast<golem::XMLContext*>(xmlcontext), create);
	golem::XMLData(val.second, const_cast<golem::XMLContext*>(xmlcontext), create);
}

void golem::Contact::ManifoldDesc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData(norm, xmlcontext->getContextFirst("norm"));
}

void golem::Contact::ManifoldDesc::create(const Manipulator& manipulator, const Contact::Config::SetConstPtr& configSet, Configuration::Path& path, ManifoldCtrl& manifold) const {
	if (configSet.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Contact::ManifoldDesc::create(): at least two configurations required");

	// mean contact frame - affine combination of local frames
	RBCoord mean(Vec3::zero(), Quat::zero());
	for (Contact::Config::SetConstPtr::const_iterator i = configSet.begin(); i != configSet.end(); ++i)
		mean += RBCoord((*i)->get()->path.getGrip().frame.toMat34() * (*i)->get()->manifold.frame);
	mean *= REAL_ONE / configSet.size();
	mean.q.normalise();

	// nearest
	const RBDist err = manipulator.getDesc().trajectoryErr;
	Contact::Config::SetConstPtr::const_iterator contact = configSet.begin();
	for (Contact::Config::SetConstPtr::const_iterator i = contact; ++i != configSet.end();)
		if (err.dot(RBDist(mean, RBCoord((*contact)->get()->path.getGrip().frame.toMat34() * (*i)->get()->manifold.frame))) > err.dot(RBDist(mean, RBCoord((*i)->get()->path.getGrip().frame.toMat34() * (*i)->get()->manifold.frame))))
			contact = i;

	manifold.frame = (*contact)->get()->manifold.frame;

	// trn * old = mean |==> trn = mean * old^-1
	path = (*contact)->get()->path;
	const RBCoord trn(mean.toMat34() * ~manifold.frame * ~path.getGrip().frame.toMat34());
	for (auto& i : path)
		i.frame = trn * i.frame;

	// manifold
	manifold.frameDev = golem::Twist::zero();
	const Mat34 meanInv(~mean.toMat34());
	for (Contact::Config::SetConstPtr::const_iterator i = configSet.begin(); i != configSet.end(); ++i) {
		const Mat34 dev = meanInv * (*i)->get()->path.getGrip().frame.toMat34() * (*i)->get()->manifold.frame;
		Vec3 euler;
		dev.R.toEuler(euler.x, euler.y, euler.z);
		manifold.frameDev.v += Vec3(this->norm.v.x*Math::abs(dev.p.x), this->norm.v.y*Math::abs(dev.p.y), this->norm.v.z*Math::abs(dev.p.z));
		manifold.frameDev.w += Vec3(this->norm.w.x*Math::abs(euler.x), this->norm.w.y*Math::abs(euler.y), this->norm.w.z*Math::abs(euler.z));
	}

	// normalise
	const Real mag = manifold.frameDev.max();
	if (mag < REAL_EPS)
		throw Message(Message::LEVEL_ERROR, "Contact::ManifoldDesc::create(): unable to normalise contact manifold");
	manifold.frameDev.multiply(REAL_ONE / mag, manifold.frameDev);
}

//------------------------------------------------------------------------------

Contact::Contact(const Desc& desc, Manipulator& manipulator) : manipulator(manipulator), context(manipulator.getContext()) {
	desc.assertValid(Assert::Context("Contact()."));

	this->desc = desc;
	configuration = desc.configurationDesc->create(manipulator); // throws
	optimisation = desc.optimisationDesc->create(*this);
}

//------------------------------------------------------------------------------

void Contact::add(const golem::Contact3D::Data::Map& contacts, const golem::Contact::View::Seq& views, const golem::Configuration::Space::Seq& spaces, const Contact::SelectorMap& selectorMap) {
	// add configuration sub-space
	configuration->add(spaces); // throws if empty
	
	// selecor map
	this->selectorMap.insert(selectorMap.begin(), selectorMap.end());

	// add contacts
	if (contacts.empty())
		throw Message(Message::LEVEL_ERROR, "Contact::add(): %s: no contacts", getName().c_str());
	for (golem::Contact3D::Data::Map::const_iterator i = contacts.begin(); i != contacts.end(); ++i) {
		golem::Contact3D::Data::Map::iterator j = this->contacts.find(i->first);
		// check if indices do not overlap
		if (j != this->contacts.end())
			throw Message(Message::LEVEL_ERROR, "Contact::add(): %s: overlapping contacts %u >= %u", getName().c_str(), this->contacts.rbegin()->first, contacts.begin()->first);
		// add contact model
		this->contacts[i->first] = i->second;
	}

	// add views
	if (views.empty())
		throw Message(Message::LEVEL_ERROR, "Contact::add(): %s: no views", getName().c_str());
	this->views.insert(this->views.end(), views.begin(), views.end());
}

void Contact::clear() {
	selectorMap.clear();
	configuration->clear();
	contacts.clear();
	views.clear();
}

bool Contact::empty() const {
	return configuration->getSpaces().empty() || contacts.empty() || views.empty();
}

//------------------------------------------------------------------------------

void Contact::create(const data::Point3D& points, Point3DKernel::SeqPtr object, NNSearch::Ptr nnSearch, golem::data::ContactQueryCallback::ContactEval* contactEval) {
	if (empty())
		throw Message(Message::LEVEL_ERROR, "Contact::create(): %s: No training data", getName().c_str());

	// process contacts
	Contact3D::process(desc.contact3DDesc, points, contacts);

	// normalise views
	if (!golem::Sample<Real>::normalise<golem::Ref1>(views))
		throw Message(Message::LEVEL_ERROR, "Contact::create(): %s: Unable to normalise view distribution", getName().c_str());

	// initialise query densities
	const Query::Desc::Map::const_iterator queryDescAny = desc.queryDescMap.find(SelectorAny);
	if (queryDescAny == desc.queryDescMap.end() || queryDescAny->second == nullptr) // should not happen - see Contact::Desc::assertValid() 
		throw Message(Message::LEVEL_ERROR, "Contact::create(): %s: missing %s query density", getName().c_str(), SelectorAny.c_str());

	U32 queryIndexMax = 0;
	queryMap.clear();
	normalisation = REAL_ZERO;
	for (golem::Contact::View::Seq::iterator i = views.begin(); i != views.end(); ++i) {
		// skip views with zero weights
		const bool noQuery = i->weight <= REAL_ZERO;

		// clear query pointers
		i->queryPtrSeq.clear();

		for (golem::Contact::View::ModelPtr::Map::const_iterator j = i->models.begin(), k = i->models.end(); j != i->models.end(); k = j++) {
			const golem::U32 queryIndex = j->second.index;

			// max density index
			if (queryIndexMax < queryIndex)
				queryIndexMax = queryIndex;

			// skip views with zero weights
			if (noQuery)
				continue;

			// check if query density is already created
			QueryMap::const_iterator queryPtr = queryMap.find(queryIndex);
			if (queryPtr == queryMap.end()) {
				// find query density description using query selector
				const golem::Contact::SelectorMap::const_iterator selector = selectorMap.find(queryIndex);
				Query::Desc::Map::const_iterator queryDesc = queryDescAny;
				if (selector != selectorMap.end()) {
					Query::Desc::Map::const_iterator selectorQueryDesc = desc.queryDescMap.find(selector->second);
					if (selectorQueryDesc != desc.queryDescMap.end() && selectorQueryDesc->second != nullptr)
						queryDesc = selectorQueryDesc;
				}

				// find model density
				const Contact3D::Data::Map::iterator contacts = this->contacts.find(queryIndex);
				if (contacts == this->contacts.end())
					throw Message(Message::LEVEL_ERROR, "Contact::create(): %s: missing #%u model density", getName().c_str(), queryIndex + 1);

				// create query density container
				const std::string queryName = queryDesc->first + "-" + std::to_string(queryIndex + 1);
				queryPtr = queryMap.insert(queryMap.end(), std::make_pair(queryIndex, queryDesc->second->create(context, queryName))); // throws

				// object model and nn-search
				queryPtr->second->setObject(object);
				queryPtr->second->setNNSearch(nnSearch);

				// create query density
				queryPtr->second->create(contacts->second, points); // throws

				// highest weight
				if (normalisation < queryPtr->second->weight)
					normalisation = queryPtr->second->weight;
			}

			// create query set for sampling, use only first models - test for different links (it is a multimap, multiple models per the same link are possible)
			if (j == i->models.begin() || k->first < j->first)
				i->queryPtrSeq.push_back(View::QueryPtr(j->second.index, j->first, j->second.frame, queryPtr->second.get(), j->second.weight));
		}

		// skip views with zero weights
		if (noQuery)
			continue;

		// assure that space indexis valid
		if (i->space >= configuration->getSpaces().size())
			throw Message(Message::LEVEL_ERROR, "Contact::create(): %s: sub-space index %u out of range <1, %u>", getName().c_str(), i->space + 1, configuration->getSpaces().size());

		// normalise query set for a given view
		if (!golem::Sample<Real>::normalise<golem::Ref1>(i->queryPtrSeq))
			throw Message(Message::LEVEL_ERROR, "Contact::create(): %s: Unable to normalise view query set distribution", getName().c_str());
	}
	// normalisation
	if (normalisation < REAL_EPS)
		throw Message(Message::LEVEL_ERROR, "Contact::create(): %s: Unable to normalise query set distribution", getName().c_str());
	normalisation = desc.weight / normalisation;
	context.verbose("Contact::create(): %s: query_densities=%u, normalisation=%e\n", getName().c_str(), (U32)queryMap.size(), normalisation);

	// make sure there are query density created
	if (queryMap.empty())
		throw Message(Message::LEVEL_ERROR, "Contact::create(): %s: no query density created", getName().c_str());

	// assure that query density indices are consecutive and begins from 0
	//if (queryMap.rbegin()->first + 1 != queryMap.size())
	//	throw Message(Message::LEVEL_ERROR, "Contact::process(): %s: non-consecutive query densities %u/%u", getName().c_str(), queryMap.rbegin()->first + 1, (U32)queryMap.size());

	// assure that query density indices are consecutive, despite not all off them have to be initialised
	if (queryIndexMax >= (U32)this->contacts.size())
		throw Message(Message::LEVEL_ERROR, "Contact::create(): %s: query density index #%u out of range <1, %u>", getName().c_str(), queryIndexMax + 1, (U32)this->contacts.size());

	// create linear access container
	querySeq.clear();
	querySeq.resize(queryIndexMax + 1, nullptr);
	for (QueryMap::const_iterator i = queryMap.begin(); i != queryMap.end(); ++i)
		querySeq[i->first] = i->second.get();

	// optimisation
	optimisation->create(points, contactEval);
}

Contact::Config::Seq::iterator Contact::find(Config::Seq& configs, Config::Seq::iterator ptr) {
	return optimisation->find(configs, ptr);
}

void Contact::find(Config::Seq::const_iterator begin, Config::Seq::const_iterator end, golem::U32 selectionStep, golem::Real beginStep, golem::Real endStep) {
	optimisation->find(begin, end, selectionStep, beginStep, endStep);
}

void Contact::sample(golem::Rand& rand, Config& config) const {
	// sample view
	const View::Seq::const_iterator viewPtr = golem::Sample<golem::Real>::sample<golem::Ref1, View::Seq::const_iterator>(views, rand);
	if (viewPtr == views.end())
		throw Message(Message::LEVEL_ERROR, "Contact::sample(): %s: View sampling error", getName().c_str());
	config.view = (U32)std::distance(views.begin(), viewPtr);

	// extract space
	config.space = viewPtr->space;
	Configuration::Space::Seq::const_iterator spacePtr = configuration->getSpaces().begin() + config.space;

	// copy manifold
	// TODO View manifold
	config.manifold = viewPtr->manifold;// spacePtr->manifold;

	// sample contact (query density) for a given view
	const View::QueryPtr::Seq::const_iterator queryPtr = golem::Sample<golem::Real>::sample<golem::Ref1, View::QueryPtr::Seq::const_iterator>(viewPtr->getQueryPtrSeq(), rand);
	if (queryPtr == viewPtr->getQueryPtrSeq().end())
		throw Message(Message::LEVEL_ERROR, "Contact::sample(): %s: Query sampling error", getName().c_str());

	// sample contact frame
	const RBCoord frame = queryPtr->query->sample(rand);

	// sample manipulator pose
	Manipulator::Config mconfig;
	configuration->sample(rand, *spacePtr, config.manifold, mconfig);
	// joint
	if (queryPtr->link.getType() == Manipulator::Link::TYPE_JOINT) {
		// TODO more efficient here!
		const Mat34 base(mconfig.frame.toMat34());
		WorkspaceJointCoord joints;
		Mat34 delta;
		manipulator.getJointFrames(mconfig.config, base, joints);
		// trn*delta = base ==> delta = trn^-1*base
		delta.setInverseRT(joints[queryPtr->link.getJoint()]);
		delta.multiply(delta, base);
		mconfig.frame.multiply(frame, RBCoord(delta));
	}
	// base
	else {
		mconfig.frame = frame;
	}

	// sample path
	configuration->sample(rand, *spacePtr, mconfig, config.path);
}

//------------------------------------------------------------------------------

void golem::Contact::Desc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("name", name, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("weight", weight, const_cast<golem::XMLContext*>(xmlcontext), false);

	contact3DDesc.load(xmlcontext->getContextFirst("contact_3d"));

	queryDescMap.clear();
	golem::XMLData(queryDescMap, queryDescMap.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "query", false);

	golem::XMLData("penalty_exp", penaltyExp, const_cast<golem::XMLContext*>(xmlcontext), false);

	if (configurationDesc != nullptr)
		configurationDesc->load(xmlcontext->getContextFirst("configuration"));
	if (collisionDesc != nullptr)
		collisionDesc->load(xmlcontext->getContextFirst("collision"));
	if (optimisationDesc != nullptr)
		optimisationDesc->load(xmlcontext->getContextFirst("optimisation"));
}

void golem::XMLData(Contact::Desc::Map::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("name", const_cast<std::string&>(val.first), xmlcontext, create);
	val.second.reset(new Contact::Desc);
	val.second->optimisationDesc.reset(new OptimisationSA::Desc);
	val.second->load(xmlcontext);
}

void golem::XMLData(const std::string &attr, Contact::SelectorMap& val, golem::XMLContext* xmlcontext, bool create) {
	std::string selectorMap;

	if (create) {
		for (Contact::SelectorMap::const_iterator i = val.begin(); i != val.end(); ++i) {
			selectorMap.append("(");
			selectorMap.append(std::to_string(i->first));
			selectorMap.append(",");
			selectorMap.append(i->second);
			selectorMap.append(")");
		}
	}

	golem::XMLData(attr, selectorMap, xmlcontext, create);

	if (!create) {
		val.clear();
		std::stringstream sstream(selectorMap + " "); // HACK std::stringstream::read(&c, 1) reads one character too much without reporting eof
		IStream istream(sstream, "\t,; (){}");
		Contact::SelectorMap::value_type selector;
		for (bool index = true; !istream.eos(); index = !index) {
			const std::string str(istream.next<const char*>());
			if (index) {
				std::istringstream iss(str, std::istringstream::in);
				iss >> const_cast<U32&>(selector.first);
				if (iss.fail())
					throw MsgXMLParser(Message::LEVEL_ERROR, "golem::XMLData(Contact::SelectorMap): Value parse error at: %s", str.c_str());
			}
			else {
				selector.second = str;
				val.insert(selector);
			}
		}
	}
}

void golem::XMLData(Contact::SelectorTypeMap::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("type", const_cast<std::string&>(val.first), xmlcontext, create);
	golem::XMLData("selectors", val.second, xmlcontext, create);
}

//------------------------------------------------------------------------------

void golem::XMLData(const std::string &attr, golem::Contact::Config::Cluster::Type& val, XMLContext* context, bool create) {
	std::string type = golem::Contact::Config::Cluster::typeName[val];
	XMLData(attr, type, context, create);
	if (!create) {
		transform(type.begin(), type.end(), type.begin(), tolower);
		for (U32 i = 0; i < (U32)golem::Contact::Config::Cluster::TYPE_SIZE; ++i)
			if (type == golem::Contact::Config::Cluster::typeName[i]) {
				val = (golem::Contact::Config::Cluster::Type)i;
				return;
			}

		throw golem::MsgXMLParser(Message::LEVEL_ERROR, "XMLData(): unknown clustering type: %s", type.c_str());
	}
}

//------------------------------------------------------------------------------

template <> void golem::Stream::read(golem::Contact::SelectorMap::value_type& value) const {
	read(const_cast<golem::U32&>(value.first));
	read(value.second);
}

template <> void golem::Stream::write(const golem::Contact::SelectorMap::value_type& value) {
	write(value.first);
	write(value.second);
}

template <> void golem::Stream::read(golem::Contact::Likelihood& value) const {
	value.setToDefault();

	value.header.load(*this);

	read(value.contacts);
	read(value.config);
	read(value.collision);

	if (value.header.getVersionCurrent().version != golem::Header::VERSION_UNDEF)
		read(value.user);

	read(value.value);
	read(value.valueActive);
	read(value.valueInactive);
}

template <> void golem::Stream::write(const golem::Contact::Likelihood& value) {
	value.header.store(*this);

	write(value.contacts);
	write(value.config);
	write(value.collision);
	write(value.user);
	write(value.value);
	write(value.valueActive);
	write(value.valueInactive);
}

template <> void golem::Stream::read(golem::Contact::Config::Ptr& value) const {
	if (value == nullptr)
		value.reset(new Contact::Config);

	read(*value);
}

template <> void golem::Stream::write(const golem::Contact::Config::Ptr& value) {
	if (value == nullptr)
		throw Message(Message::LEVEL_ERROR, "golem::Stream::write(const golem::Contact::Config::Ptr&): null pointer!");

	write(*value);
}

template <> void golem::Stream::read(golem::Contact::Config& value) const {
	value.setToDefault();
	
	value.header.load(*this);

	read(value.type);
	read(value.view);
	read(value.space);

	if (value.header.getVersionCurrent().major >= 1 && value.header.getVersionCurrent().minor >= 1) {
		read(value.manifold);
	}

	read(value.path);
	read(value.likelihood);
}

template <> void golem::Stream::write(const golem::Contact::Config& value) {
	value.header.store(*this);

	write(value.type);
	write(value.view);
	write(value.space);

	write(value.manifold);

	write(value.path);
	write(value.likelihood);
}

template <> void golem::Stream::read(golem::Contact::Config::Map::value_type& value) const {
	read(const_cast<std::string&>(value.first));
	read(value.second);
}

template <> void golem::Stream::write(const golem::Contact::Config::Map::value_type& value) {
	write(value.first);
	write(value.second);
}

template <> void golem::Stream::read(golem::Contact::View::ModelPtr& value) const {
	read(value.index);
	read(value.weight);
	read(value.frame);
}
template <> void golem::Stream::write(const golem::Contact::View::ModelPtr& value) {
	write(value.index);
	write(value.weight);
	write(value.frame);
}

template <> void golem::Stream::read(golem::Contact::View::ModelPtr::Map::value_type& value) const {
	read(const_cast<Manipulator::Link&>(value.first));
	read(value.second);
}
template <> void golem::Stream::write(const golem::Contact::View::ModelPtr::Map::value_type& value) {
	write(value.first);
	write(value.second);
}

template <> void golem::Stream::read(golem::Contact::View::Seq::value_type& value) const {
	value.header.load(*this);

	read(value.weight);
	read(value.cdf);
	read(value.object);
	read(value.space);
	read(value.models, value.models.begin());
	
	if (value.header.getVersionCurrent().version != golem::Header::VERSION_UNDEF) {
		read(value.manifold);
	}

	read(value.points, value.points.begin());
}
template <> void golem::Stream::write(const golem::Contact::View::Seq::value_type& value) {
	value.header.store(*this);

	write(value.weight);
	write(value.cdf);
	write(value.object);
	write(value.space);
	write(value.models.begin(), value.models.end());

	write(value.manifold);

	write(value.points.begin(), value.points.end());
}

//------------------------------------------------------------------------------
