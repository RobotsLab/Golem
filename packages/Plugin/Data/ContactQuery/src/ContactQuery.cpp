/** @file ContactQuery.cpp
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
#include <Golem/Math/Data.h>
#include <Golem/Tools/Import.h>
#include <Golem/Tools/Menu.h>
#include <Golem/Tools/CollisionBounds.h>
#include <Golem/Plugin/Ctrl.h>
#include <Golem/Data/ContactQuery/ContactQuery.h>
#include <Golem/Contact/OptimisationSA.h>
#include <boost/algorithm/string.hpp>

using namespace golem;
using namespace golem::data;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new HandlerContactQuery::Desc();
}

//------------------------------------------------------------------------------

Contact::Config::Seq::const_iterator golem::data::ItemContactQuery::getConfig() const {
	if (data.configs.empty())
		return data.configs.end();

	const golem::U32 index = Contact::Config::Cluster::getIndex(data.clusters, (I32&)clusterIndex, (I32&)configIndex);
	return data.configs.begin() + index;
}

void golem::data::ItemContactQuery::setConfig(Contact::Config::Seq::const_iterator config) {
	// TODO
}

Contact::Config::SetConstPtr golem::data::ItemContactQuery::getConfigCluster() const {
	Contact::Config::SetConstPtr configSet;
	getCurrentCluster(configSet);
	return configSet;
}

Contact::Config::SetConstPtr golem::data::ItemContactQuery::getConfigSelection() const {
	return configSet;
}

//------------------------------------------------------------------------------

golem::data::ItemContactQuery::ItemContactQuery(HandlerContactQuery& handler) :
	Item(handler), handler(handler), dataFile(handler.file), clusterIndex(0), configIndex(0), modelIndex(0)
{}

Item::Ptr golem::data::ItemContactQuery::clone() const {
	throw Message(Message::LEVEL_ERROR, "ItemContactQuery::clone(): not implemented");
}

void golem::data::ItemContactQuery::createRender() {
	handler.createRender(*this);
}

void golem::data::ItemContactQuery::load(const std::string& prefix, const golem::XMLContext* xmlcontext) {
	// data index
	golem::XMLData("cluster_index", clusterIndex, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("config_index", configIndex, const_cast<golem::XMLContext*>(xmlcontext), false);

	// query
	std::string querySuffix;
	golem::XMLData("query", querySuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (querySuffix.length() > 0) {
		dataFile.load(prefix + querySuffix, [&](const std::string& path) {
			data.clear();
			FileReadStream(path.c_str()).read(data);
		});
	}

	// manifold
	try {
		golem::XMLData(manifoldMap, manifoldMap.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "manifold", false);
	}
	catch (const golem::MsgXMLParser&) {}
	for (golem::Contact::ManifoldSelector::Map::const_iterator i = manifoldMap.begin(); i != manifoldMap.end(); ++i) {
		i->second.assertValid(Assert::Context("ItemContactQuery::manifoldMap[]->"));

		// apply
		auto apply = [](const golem::Contact::ManifoldSelector::Map::value_type& manifold, ContactQuery::Data& data) {
			for (auto& configItem : data.configs)
				if (manifold.first.type == configItem->type || manifold.first.type == golem::Contact::ManifoldSelector::TypeAny) {
					if (manifold.first.space != golem::Contact::ManifoldSelector::SpaceNone) {
						if (manifold.first.space == golem::Contact::ManifoldSelector::SpaceAny || manifold.first.space == configItem->space)
							configItem->manifold = manifold.second;
					}
					if (manifold.first.view != golem::Contact::ManifoldSelector::ViewNone) {
						if (manifold.first.view == golem::Contact::ManifoldSelector::ViewAny || manifold.first.view == configItem->view)
							configItem->manifold = manifold.second;
					}
				}
		};
		apply(*i, data);
	}
}

void golem::data::ItemContactQuery::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	// data index
	golem::XMLData("cluster_index", const_cast<golem::U32&>(clusterIndex), xmlcontext, true);
	golem::XMLData("config_index", const_cast<golem::U32&>(configIndex), xmlcontext, true);

	// query xml
	std::string querySuffix = !data.configs.empty() ? handler.querySuffix : "";
	golem::XMLData("query", querySuffix, xmlcontext, true);

	// query binary
	if (querySuffix.length() > 0) {
		dataFile.save(prefix + querySuffix, [=](const std::string& path) {
			FileWriteStream(path.c_str()).write(data);
		});
	}
	else
		dataFile.remove();

	// manifold
	golem::XMLData(const_cast<Contact::ManifoldSelector::Map&>(manifoldMap), manifoldMap.size(), const_cast<golem::XMLContext*>(xmlcontext), "manifold", true);
}

//------------------------------------------------------------------------------

void golem::data::ItemContactQuery::setData(const ContactQuery::Data& data) {
	this->data = data;
	dataFile.setModified(true);
}

const golem::data::ContactQuery::Data& golem::data::ItemContactQuery::getData() const {
	return data;
}

//------------------------------------------------------------------------------

Item::Ptr golem::data::ItemContactQuery::convert(const Handler& handler) {
	return this->handler.convert(*this, handler);
}

const StringSeq& golem::data::ItemContactQuery::getConvertInterfaces() const {
	return this->handler.convertInterfaces;
}

bool golem::data::ItemContactQuery::isConvertSupported(const Handler& handler) const {
	return this->handler.isConvertSupported(handler);
}

void golem::data::ItemContactQuery::getCurrentCluster(Contact::Config::SetConstPtr& configSet) const {
	configSet.clear();
	if ((size_t)clusterIndex < data.clusters.size()) {
		for (U32 i = data.clusters[clusterIndex].begin; i < data.clusters[clusterIndex].end; ++i)
			configSet.insert(configSet.end(), (data.configs.begin() + i));
	}
}

//------------------------------------------------------------------------------

void golem::data::HandlerContactQuery::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	golem::data::Handler::Desc::load(context, xmlcontext);

	golem::XMLData("planner_index", plannerIndex, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("mode_cluster", modeCluster, const_cast<golem::XMLContext*>(xmlcontext));

	manipulatorDesc->load(xmlcontext->getContextFirst("manipulator"));
	solverDesc->load(xmlcontext->getContextFirst("planner"));

	try {
		golem::XMLData("collision_points", collisionPoints, const_cast<golem::XMLContext*>(xmlcontext));
	}
	catch (MsgXMLParserAttributeNotFound&) {}

	clusterDesc.load(xmlcontext->getContextFirst("clustering"));
	golem::XMLData("type", clusterType, xmlcontext->getContextFirst("clustering"), false);
	
	contactManifoldDesc.load(xmlcontext->getContextFirst("manifold"));

	appearance.load(xmlcontext->getContextFirst("appearance"));

	appearanceCluster.load(xmlcontext->getContextFirst("appearance cluster"));
	appearanceClusterMean.load(xmlcontext->getContextFirst("appearance cluster_mean"));

	golem::XMLData("query_suffix", querySuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
}

golem::data::Handler::Ptr golem::data::HandlerContactQuery::Desc::create(golem::Context &context) const {
	Handler::Ptr handler(new HandlerContactQuery(context));
	to<HandlerContactQuery>(handler.get())->create(*this);
	return handler;
}

//------------------------------------------------------------------------------

golem::data::HandlerContactQuery::HandlerContactQuery(golem::Context &context) :
	Handler(context), rand(context.getRandSeed()),
	clusterIndexRequest(0), configIndexRequest(0), modelIndexRequest(0),
	likelihoodRequest(false), typeRequest(false), poseRequest(false), subspaceDistRequest(false),
	selectRequest(false), clusterRequest(false),
	contactManifoldShowCluster(false),
	contactEval(nullptr)
{}

void golem::data::HandlerContactQuery::create(const Desc& desc) {
	golem::data::Handler::create(desc);

	plannerIndex = desc.plannerIndex;

	modeCluster = desc.modeCluster;

	// shallow copy, TODO clone member function
	manipulatorDesc = desc.manipulatorDesc;
	solverDesc = desc.solverDesc;

	collisionPoints = desc.collisionPoints;

	clusterDesc = desc.clusterDesc;
	appearance = desc.appearance;

	appearanceCluster = desc.appearanceCluster;
	appearanceClusterMean = desc.appearanceClusterMean;

	querySuffix = desc.querySuffix;

	clusterType = desc.clusterType;

	contactManifoldDesc = desc.contactManifoldDesc;

	transformInterfaces = {
		"Point3D",
		"ContactModel",
	};
	convertInterfaces = {
		"Trajectory",
	};
}

//------------------------------------------------------------------------------

std::string golem::data::HandlerContactQuery::getFileExtContactQuery() {
	return std::string(".query");
}

golem::data::Item::Ptr golem::data::HandlerContactQuery::create() const {
	return Item::Ptr(new ItemContactQuery(*const_cast<HandlerContactQuery*>(this))); // safe: no modification here - only passing reference
}

//------------------------------------------------------------------------------

golem::U32 golem::data::HandlerContactQuery::getPlannerIndex() const {
	return plannerIndex;
}

void golem::data::HandlerContactQuery::set(const golem::Planner& planner, const ControllerId::Seq& controllerIDSeq) {
	manipulator = manipulatorDesc->create(planner, controllerIDSeq);
	this->solver = solverDesc->create(*manipulator);
}

//------------------------------------------------------------------------------

void golem::data::HandlerContactQuery::printConfigInfo(const ItemContactQuery& item) const {
	if (item.data.configs.empty()) {
		manipulator->getContext().write("No grasp configs available\n");
		return;
	}

	const Contact::Config::Seq::const_iterator config = item.getConfig();
	if (config == item.getData().configs.end() || !config->get()) {
		manipulator->getContext().warning("Invalid config\n");
		return;
	}

	//std::stringstream str;
	//for (size_t i = 0; i < manipulator.getJoints(); ++i)
	//	str << " c" << (i + 1) << "=\"" << pose.jc[i] << "\"";
	//manipulator.getContext().write("<pose v1=\"%.7f\" v2=\"%.7f\" v3=\"%.7f\" q0=\"%.7f\" q1=\"%.7f\" q2=\"%.7f\" q3=\"%.7f\" dim=\"%d\"%s/>\n", frame.p.x, frame.p.y, frame.p.z, frame.q.q0, frame.q.q1, frame.q.q2, frame.q.q3, manipulator.getController().getStateInfo().getJoints().size(), str.what());

	manipulator->getContext().write("Contact config: type=%s, view=#%u, space=#%u, cluster=%d/%d, config=%d/%d, likelihood_{value=%f, active=%f, inactive=%f, contacts=%f, config=%f, collision=%f, user=%f}, valid=%s\n",
		(*config)->type.c_str(), (*config)->view + 1, (*config)->space + 1,
		item.clusterIndex + 1, (golem::U32)item.data.clusters.size(), item.configIndex + 1, (item.data.clusters[item.clusterIndex].end - item.data.clusters[item.clusterIndex].begin),
		(*config)->likelihood.value, (*config)->likelihood.valueActive, (*config)->likelihood.valueInactive, (*config)->likelihood.getActiveContactsAverageValue(), (*config)->likelihood.config, (*config)->likelihood.collision, (*config)->likelihood.user,
		(*config)->getContact() ? (*config)->likelihood.isValid(REAL_EPS) ? "yes" : "no" : "unavailable");
}

//------------------------------------------------------------------------------

Item::Ptr golem::data::HandlerContactQuery::transform(const Item::List& input) {
	Item::Ptr item(create());
	ItemContactQuery* itemContactQuery = to<ItemContactQuery>(item.get());

	// block item rendering (hasRenderBlock()=true)
	RenderBlock renderBlock(*this);

	// training data
	typedef std::vector<const ContactModel::Data::Map*> TrainingData;
	TrainingData trainingData;
	// test features
	const Point3D* points = nullptr;

	// collect data
	for (Item::List::const_iterator i = input.begin(); i != input.end(); ++i) {
		const data::ContactModel* model = is<const data::ContactModel>(*i);
		if (model)
			trainingData.push_back(&model->getData());
		else if (!points) {
			points = is<const Point3D>(*i);
		}
	}
	if (trainingData.empty())
		throw Message(Message::LEVEL_ERROR, "HandlerContactQuery::transform(): no training data specified");
	if (!points)
		throw Message(Message::LEVEL_ERROR, "HandlerContactQuery::transform(): no test features specified");

	// update planner training data
	StringSet availableTypes;
	solver->clear();
	for (TrainingData::const_iterator i = trainingData.begin(); i != trainingData.end(); ++i)
		for (data::ContactModel::Data::Map::const_iterator j = (*i)->begin(); j != (*i)->end(); ++j)
			if (j->second.type == data::ContactModel::Data::TYPE_DEFAULT) {
				const Contact::Map::const_iterator ptr = solver->add(j->first, j->second); // throws if contact type is unknown or data is invalid/empty
				if (ptr->first != j->first)
					context.info("HandlerContactQuery::transform(): contact type re-map: %s -> %s\n", j->first.c_str(), ptr->first.c_str());
				availableTypes.insert(ptr->first);
			}
	if (availableTypes.empty())
		throw Message(Message::LEVEL_ERROR, "HandlerContactQuery::transform(): unable to match contact types");

	// select types
	StringSet selectedTypes;
	if (getUICallback() && getUICallback()->hasInputEnabled()) {
		Menu menu(context, *getUICallback());
		StringSet::const_iterator ptr = availableTypes.end();
		try {
			menu.select(ptr, availableTypes.begin(), availableTypes.end(), "Select contact type:\n  [0]  all\n", [] (StringSet::const_iterator ptr) -> const std::string& { return *ptr; });
			selectedTypes.insert(*ptr);
		}
		catch (const Message&) {}
	}

	// data name/contact type: trajectory name
	if (getUICallback() && getUICallback()->hasInputEnabled()) {
		Menu menu(context, *getUICallback());

		// take as reference first contact model, first query type and query density
		Contact& referenceContact = *solver->getContactMap().begin()->second;
		Query::Desc& referenceQuery = *referenceContact.getDesc().queryDescMap.begin()->second;

		// query densities
		const Contact3D::TypePtr typePtr(points);
		const Query::Contact3DDesc::Seq::const_iterator contact3DTypeDesc = Contact3D::TypePtr::select(typePtr, referenceQuery.contactDescSeq);

		// optimisation steps
		const OptimisationSA* pOptimisationSA = is<OptimisationSA>(referenceContact.getOptimisation());
		if (pOptimisationSA) {
			menu.readNumber("Enter number of trajectory hypotheses: ", const_cast<OptimisationSA*>(pOptimisationSA)->getDesc().runs);
		}

		// number of kernels
		U32 queryKernels = contact3DTypeDesc->second.kernels, queryKernelsRef = queryKernels;
		menu.readNumber("Enter query density kernels: ", queryKernelsRef);

		// number of neighbours in NN-search
		U32 queryNeighbours = contact3DTypeDesc->second.nnNeighbours, queryNeighboursRef = queryNeighbours;
		menu.readNumber("Enter neighbours in nn-search (0-disabled): ", queryNeighboursRef);

		// penalty exponent
		Real penaltyExp = referenceContact.getDesc().penaltyExp;
		menu.readNumber("Enter penalty exponent: ", penaltyExp);

		// weights
		Real contactWeight = referenceContact.getDesc().weight;
		menu.readNumber("Enter contact weight: ", contactWeight);

		// feature bandwidth: take as reference first feature descriptor
		Real featureBandwidth = referenceContact.getDesc().contact3DDesc.feature3DDesc.stdDevFac[0], featureBandwidthScale = featureBandwidth;
		menu.readNumber("Enter feature bandwidth: ", featureBandwidthScale);
		featureBandwidthScale /= featureBandwidth;

		// density distance enable
		bool densityDistEnable = menu.option(contact3DTypeDesc->second.densityDistEnable ? 1 : 0, "Enable density distance: ", { "NO", "YES" }) > 0;
		// density distance bandwidth
		Real densityDistBandwidth = contact3DTypeDesc->second.densityDistBandwidth, densityDistBandwidthScale = densityDistBandwidth;
		if (densityDistEnable) menu.readNumber("Enter density distance bandwidth: ", densityDistBandwidthScale);
		densityDistBandwidthScale /= densityDistBandwidth;

		for (auto& contact : solver->getContactMap()) {
			// penalty exponent
			contact.second->getDesc().penaltyExp = penaltyExp;
			// weights
			contact.second->getDesc().weight = contactWeight;
			// feature bandwidth
			contact.second->getDesc().contact3DDesc.feature3DDesc.stdDevFac.multiply(featureBandwidthScale, contact.second->getDesc().contact3DDesc.feature3DDesc.stdDevFac);
			// query densities
			for (auto& query : contact.second->getDesc().queryDescMap) {
				const Query::Contact3DDesc::Seq::const_iterator contact3DTypeDesc = Contact3D::TypePtr::select(typePtr, query.second->contactDescSeq);
				const_cast<Query::Contact3DDesc&>(contact3DTypeDesc->second).kernels = contact3DTypeDesc->second.kernels <= 0 || queryKernels <= 0 ? queryKernelsRef  : (U32)(Real(queryKernelsRef * contact3DTypeDesc->second.kernels)/ queryKernels);
				const_cast<Query::Contact3DDesc&>(contact3DTypeDesc->second).nnNeighbours = contact3DTypeDesc->second.nnNeighbours <= 0 || queryNeighbours <= 0 ? queryNeighboursRef : (U32)(Real(queryNeighboursRef * contact3DTypeDesc->second.nnNeighbours) / queryNeighbours);
				const_cast<Query::Contact3DDesc&>(contact3DTypeDesc->second).densityDistEnable = densityDistEnable;
				if (densityDistEnable) const_cast<Query::Contact3DDesc&>(contact3DTypeDesc->second).densityDistBandwidth *= densityDistBandwidthScale;
			}
		}
	};

	// configs
	solver->find(*points, itemContactQuery->data.configs, &selectedTypes, contactEval);

	// clusters
	Contact::Config::Cluster::find(context, clusterType, clusterDesc, itemContactQuery->data.configs, itemContactQuery->data.clusters);
	if (clusterType == Contact::Config::Cluster::TYPE_TYPE || clusterType == Contact::Config::Cluster::TYPE_CONFIGURATION) {
		itemContactQuery->getCurrentCluster(itemContactQuery->configSet);
		contactManifoldShowCluster = true;
		appearance.config.showConfig = false;
	}

	// points
	itemContactQuery->data.points.resize(points->getSize());
	for (size_t i = 0; i < itemContactQuery->data.points.size(); ++i)
		itemContactQuery->data.points[i] = points->getPoint(i);
	// clusters
	const Cluster3D* cluster = is<Cluster3D>(points);
	if (cluster) {
		cluster->getClusters(itemContactQuery->data.pointsIndices, itemContactQuery->data.pointsClusters);
		cluster->getClustersSelection(itemContactQuery->data.pointsSelection);
	}

	// DEBUG contact log-likelihood:
	//context.write("Contact likelihoods\n");
	//for (size_t i = 0; i < itemContactQuery->data.configs.size(); ++i) {
	//	const golem::Contact::Config* config = itemContactQuery->data.configs[i].get();
	//	const golem::Contact* contact = config->getContact();
	//	if (!contact)
	//		throw Message(Message::LEVEL_ERROR, "Invalid contact for config #%u", i + 1);
	//	golem::Contact::Likelihood wl, pl, wpl;
	//	// grip
	//	contact->getOptimisation()->evaluate(config->path.getGrip(), wl);
	//	// grip perturbation
	//	golem::Manipulator::Waypoint grip = config->path.getGrip();
	//	Quat q;
	//	q.next(rand, 10000.0);
	//	grip.frame.q.multiply(grip.frame.q, q);
	//	contact->getOptimisation()->evaluate(grip, wpl);
	//	// path
	//	contact->getOptimisation()->evaluate(config->path, pl, true);
	//	context.write("Contact #%u likelihood: grip=%f, grip_perturbed=%f, path=%f\n", i + 1, wl.value, wpl.value, pl.value);
	//}

	itemContactQuery->dataFile.setModified(true);

	return item;
}

const StringSeq& golem::data::HandlerContactQuery::getTransformInterfaces() const {
	return transformInterfaces;
}

bool golem::data::HandlerContactQuery::isTransformSupported(const Item& item) const {
	return is<const data::ContactModel>(&item) || is<const data::Point3D>(&item);
}

//------------------------------------------------------------------------------

Item::Ptr golem::data::HandlerContactQuery::convert(ItemContactQuery& item, const Handler& handler) {
	if (item.data.configs.empty() || item.data.clusters.empty())
		throw Message(Message::LEVEL_ERROR, "HandlerContactQuery::convert(): no data to convert");

	// only one type of output
	Item::Ptr pItem(handler.create());
	data::Trajectory* trajectory = is<data::Trajectory>(pItem.get());
	if (!trajectory)
		throw Message(Message::LEVEL_ERROR, "HandlerContactQuery::convert(): Item %s does not support Trajectory interface", handler.getType().c_str());

	bool modeCluster = this->modeCluster;
	bool collisionPoints = this->collisionPoints;

	U32 end = manipulator->getDesc().trajectoryClusterSize;

	if (getUICallback() && getUICallback()->hasInputEnabled()) {
		Menu menu(context, *getUICallback());

		modeCluster = menu.option(modeCluster ? 0 : 1, "Process cluster: ", { "YES", "NO" }) == 0;
		collisionPoints = menu.option(0, "Collision points: ", { "YES", "NO" }) == 0;

		menu.readNumber(makeString("Number of %s to test: ", modeCluster ? "clusters" : "trajectories").c_str(), end);
		manipulator->getDesc().trajectoryThrow = menu.option(manipulator->getDesc().trajectoryThrow ? 1 : 0, "Always generate trajectory: ", { "YES", "NO" }) == 1;
	}

	// collision bounds
	CollisionBounds::Ptr collisionBounds;
	if (collisionPoints) {
		collisionBounds.reset(new CollisionBounds(
			const_cast<golem::Planner&>(manipulator->getPlanner()),
			CollisionBounds::getBounds([=](size_t i, Vec3& p) -> bool { if (i < item.data.points.size()) p = item.data.points[i]; return i < item.data.points.size(); }, item.data.pointsIndices, item.data.pointsClusters, item.data.pointsSelection),
			getUICallback() ? &rendererBounds : nullptr,
			getUICallback() ? &getUICallback()->getCS() : nullptr
		));
		collisionBounds->setLocal();
	}

	(void)Contact::Config::Cluster::getIndex(item.data.clusters, (I32&)item.clusterIndex, (I32&)item.configIndex);

	if (modeCluster) {
		end = item.clusterIndex + std::max(end, (U32)1) - 1;
		item.configIndex = 0;
		(void)Contact::Config::Cluster::getIndex(item.data.clusters, (I32&)end, (I32&)item.configIndex);
		end += 1;

		Configuration::Path path;
		ManifoldCtrl manifold;

		// search
		const U32 begin = item.clusterIndex;
		const std::pair<U32, RBDistEx> val = manipulator->find<U32>(begin, end, [&](U32 index) -> RBDistEx {
			item.clusterIndex = index;
			item.getCurrentCluster(item.configSet);

			// create manifold
			contactManifoldDesc.create(*manipulator, item.configSet, path, manifold);

			// find trajectory
			const RBDist dist(manipulator->find(path));
			const golem::ConfigspaceCoord approach = path.getApproach().config;
			const bool collides = collisionBounds != nullptr && collisionBounds->collides(approach, manipulator->getArmInfo().getJoints().end() - 1); // TODO test approach config using points
			createRender(item);
			const RBDistEx distex(dist, manipulator->getDesc().trajectoryErr.collision && collides);
			context.verbose("#%03u/%u: Trajectory error: lin=%.9f, ang=%.9f, collision=%s\n", index - begin + 1, end - begin, distex.lin, distex.ang, distex.collision ? "yes" : "no");
			return distex;
		});
		//if (manipulator->getDesc().trajectoryErr.dot(distex) > REAL_ONE && manipulator->getDesc().trajectoryThrow)
		//	throw golem::Message(golem::Message::LEVEL_ERROR, "HandlerContactQuery::convert(): No trajectory found");

		// update trajectory
		golem::WaypointCtrl::Seq waypoints;
		manipulator->create(path, waypoints);
		trajectory->setWaypoints(waypoints);
		trajectory->setManifold(manifold);
	}
	else {
		end = item.configIndex + std::max(end, (U32)1) - 1;
		(void)Contact::Config::Cluster::getIndex(item.data.clusters, (I32&)item.clusterIndex, (I32&)end);
		end += 1;

		// search
		const U32 begin = item.configIndex;
		const std::pair<U32, RBDistEx> val = manipulator->find<U32>(begin, end, [&](U32 index) -> RBDistEx {
			item.configIndex = index;
			//createRender(item);
			Contact::Config::Seq::const_iterator config = item.getConfig();
			const RBDist dist(manipulator->find((*config)->path));
			const golem::ConfigspaceCoord approach = (*config)->path.getApproach().config;
			const bool collides = collisionBounds != nullptr && collisionBounds->collides(approach, manipulator->getArmInfo().getJoints().end() - 1); // TODO test approach config using points
			createRender(item);
			const RBDistEx distex(dist, manipulator->getDesc().trajectoryErr.collision && collides);
			context.verbose("#%03u/%u: Trajectory error: lin=%.9f, ang=%.9f, collision=%s\n", index - begin + 1, end - begin, distex.lin, distex.ang, distex.collision ? "yes" : "no");
			//if (getUICallback() && getUICallback()->hasInputEnabled()) Menu(context, *getUICallback()).option("\x0D", "Press <Enter> to continue...");
			return distex;
		});

		item.configIndex = val.first;

		context.debug("#%03u/%u (%u): Trajectory error: lin=%.9f, ang=%.9f, collision=%s\n", item.configIndex - begin + 1, end - begin, item.configIndex + 1, val.second.lin, val.second.ang, val.second.collision ? "yes" : "no");

		createRender(item);

		// update trajectory
		golem::WaypointCtrl::Seq waypoints;
		manipulator->create((*item.getConfig())->path, waypoints);
		trajectory->setWaypoints(waypoints);
	}

	return pItem;
}

bool golem::data::HandlerContactQuery::isConvertSupported(const Handler& handler) const {
	return handler.isItem<const data::Trajectory>();
}

void golem::data::HandlerContactQuery::setContactEval(ContactEval* contactEval) {
	this->contactEval = contactEval;
}

//------------------------------------------------------------------------------

void golem::data::HandlerContactQuery::createRender(const ItemContactQuery& item) {
	if (likelihoodRequest) {
		context.write("Likelihood clustering\n");
		Contact::Config::Cluster::findLikelihood(context, clusterDesc, const_cast<ItemContactQuery&>(item).data.configs, const_cast<ItemContactQuery&>(item).data.clusters);
	}
	if (typeRequest) {
		context.write("Type clustering\n");
		Contact::Config::Cluster::findType(context, clusterDesc, const_cast<ItemContactQuery&>(item).data.configs, const_cast<ItemContactQuery&>(item).data.clusters);
	}
	if (poseRequest) {
		context.write("Pose clustering\n");
		Contact::Config::Cluster::findConfiguration(context, clusterDesc, const_cast<ItemContactQuery&>(item).data.configs, const_cast<ItemContactQuery&>(item).data.clusters);
	}

	UI::CriticalSectionWrapper cs(getUICallback());

	renderer.reset();
	rendererPoints.reset();
	rendererCluster.reset();
	rendererManifold.reset();

	if (likelihoodRequest) {
		item.dataFile.setModified(true);
		item.clusterIndex = 0;
		item.configIndex = 0;
		const_cast<ItemContactQuery&>(item).configSet.clear();
		likelihoodRequest = false;
	}
	if (typeRequest) {
		item.dataFile.setModified(true);
		item.clusterIndex = 0;
		item.configIndex = 0;
		if (clusterType == Contact::Config::Cluster::TYPE_TYPE)
			item.getCurrentCluster(const_cast<ItemContactQuery&>(item).configSet);
		else
			const_cast<ItemContactQuery&>(item).configSet.clear();
		typeRequest = false;
	}
	if (poseRequest) {
		item.dataFile.setModified(true);
		item.clusterIndex = 0;
		item.configIndex = 0;
		if (clusterType == Contact::Config::Cluster::TYPE_CONFIGURATION)
			item.getCurrentCluster(const_cast<ItemContactQuery&>(item).configSet);
		else
			const_cast<ItemContactQuery&>(item).configSet.clear();
		poseRequest = false;
	}
	if (clusterIndexRequest != 0) {
		item.clusterIndex += clusterIndexRequest;
		item.configIndex = 0;
		(void)Contact::Config::Cluster::getIndex(item.data.clusters, (I32&)item.clusterIndex, (I32&)item.configIndex);
		clusterIndexRequest = 0;
		if (clusterType == Contact::Config::Cluster::TYPE_TYPE || clusterType == Contact::Config::Cluster::TYPE_CONFIGURATION)
			item.getCurrentCluster(const_cast<ItemContactQuery&>(item).configSet);
		printConfigInfo(item);
	}
	if (configIndexRequest != 0) {
		item.configIndex += configIndexRequest;
		(void)Contact::Config::Cluster::getIndex(item.data.clusters, (I32&)item.clusterIndex, (I32&)item.configIndex);
		configIndexRequest = 0;
		printConfigInfo(item);
	}
	if (modelIndexRequest != 0) {
		if (!item.data.configs.empty()) {
			const Contact::Config::Seq::const_iterator config = item.getConfig();
			if (config != item.getData().configs.end() && (*config)->getContact() && !(*config)->getContact()->getQuery().empty() || subspaceDistRequest) {
				item.modelIndex = Math::clamp(I32(item.modelIndex) + modelIndexRequest, (I32)0, (I32)(*config)->getContact()->getQuery().size() - 1);
				context.write("Query density %u/%u: id=%s\n", item.modelIndex + 1, (*config)->getContact()->getQuery().size(), (*config)->getContact()->getQuery()[item.modelIndex]->getName().c_str());
			}
		}
		modelIndexRequest = 0;
	}
	if (selectRequest) {
		if (!item.data.configs.empty()) {
			const Contact::Config::Seq::const_iterator config = item.getConfig();
			if (item.configSet.find(config) != item.configSet.end()) const_cast<ItemContactQuery&>(item).configSet.erase(config); else const_cast<ItemContactQuery&>(item).configSet.insert(config);
		}
		selectRequest = false;
	}
	if (clusterRequest) {
		if (!item.data.configs.empty() && !item.configSet.empty()) {
			const U32 size = (U32)item.configSet.size();
			Contact::Config::Seq configs;
			configs.reserve(item.data.configs.size());
			for (auto& i : item.configSet)
				configs.push_back(*i);
			for (Contact::Config::Seq::const_iterator i = item.data.configs.begin(); i != item.data.configs.end(); ++i)
				if (item.configSet.find(i) == item.configSet.end())
					configs.push_back(*i);

			const_cast<ItemContactQuery&>(item).data.configs = configs;
			const_cast<ItemContactQuery&>(item).data.clusters.clear();
			const_cast<ItemContactQuery&>(item).data.clusters.push_back(golem::Contact::Config::Cluster(0, size));

			const_cast<ItemContactQuery&>(item).configSet.clear();
			for (size_t i = 0; i < size; ++i)
				const_cast<ItemContactQuery&>(item).configSet.insert(item.data.configs.begin() + i);

			item.dataFile.setModified(true);
		}
		clusterRequest = false;
	}

	if (subspaceDistRequest)
		context.write("Trajectory stddev_dist=%f\n", appearance.config.configPath.subspaceDistVal);
	subspaceDistRequest = false;

	appearance.config.modelSelectionIndex = item.modelIndex;

	// config
	if (!item.data.configs.empty()) {
		const Contact::Config::Seq::const_iterator config = item.getConfig();
		if (config != item.getData().configs.end())
			(*config)->draw(*manipulator, appearance.config, rand, renderer);
	}

	// points
	if (appearance.pointShow)
		appearance.point.draw(item.data.points, rendererPoints);

	// mean and manifold
	const bool showManifold = item.configSet.size() > 1;
	if (showManifold) {
		Configuration::Path path;
		ManifoldCtrl manifold;
		contactManifoldDesc.create(*manipulator, item.configSet, path, manifold);
		// manifold
		appearance.config.manifold.draw(manifold, path.getGrip().frame.toMat34(), rendererManifold);
		// mean
		if (appearance.config.manifold.show)
			appearanceClusterMean.draw(manipulator->getBounds(path.getGrip().config, path.getGrip().frame.toMat34()), rendererCluster);
	}
	// clusters
	if (contactManifoldShowCluster)
		for (Contact::Config::SetConstPtr::const_iterator i = item.configSet.begin(); i != item.configSet.end(); ++i)
			appearanceCluster.draw(manipulator->getBounds((***i).path.getGrip().config, (***i).path.getGrip().frame.toMat34()), rendererCluster);
}

void golem::data::HandlerContactQuery::render() const {
	if (hasRenderBlock()) return;

	renderer.render();
	rendererManifold.render();
	rendererPoints.render();
	rendererBounds.render();
	rendererCluster.render();
}

void golem::data::HandlerContactQuery::customRender() const {
}

//------------------------------------------------------------------------------

void golem::data::HandlerContactQuery::mouseHandler(int button, int state, int x, int y) {
	if (hasRenderBlock()) return;

	if (appearance.config.showConfig && state == 0 && (button == 3 || button == 4 || button == 1)) {
		appearance.config.configPath.subspaceDist = button == 3 ? appearance.config.configPath.subspaceDist + 1 : button == 4 ? appearance.config.configPath.subspaceDist - 1 : 0;
		subspaceDistRequest = true;
		requestRender();
	}
}

void golem::data::HandlerContactQuery::motionHandler(int x, int y) {
}

void golem::data::HandlerContactQuery::keyboardHandler(int key, int x, int y) {
	if (hasRenderBlock()) return;

	switch (key) {
	case '4':
		appearance.pointShow = !appearance.pointShow;
		context.write("Points: %s\n", appearance.pointShow ? "ON" : "OFF");
		requestRender();
		break;
	case '5':
		appearance.config.showModelContact = !appearance.config.showModelContact;
		context.write("Contact model: %s\n", appearance.config.showModelContact ? "ON" : "OFF");
		requestRender();
		break;
	case '6':
		appearance.config.showModelConfig = !appearance.config.showModelConfig;
		context.write("Config model: %s\n", appearance.config.showModelConfig ? "ON" : "OFF");
		requestRender();
		break;
	case '7':
		appearance.config.showConfig = !appearance.config.showConfig;
		context.write("Path: %s\n", appearance.config.showConfig ? "ON" : "OFF");
		requestRender();
		break;
	case '8':
		appearance.config.manifold.show = !appearance.config.manifold.show;
		context.write("Contact manifold: %s\n", appearance.config.manifold.show ? "ON" : "OFF");
		requestRender();
		break;
	case '9':
		contactManifoldShowCluster = !contactManifoldShowCluster;
		context.write("Contact cluster: %s\n", contactManifoldShowCluster ? "ON" : "OFF");
		requestRender();
		break;
	case '0':
		likelihoodRequest = true;
		contactManifoldShowCluster = false;
		appearance.config.showConfig = true;
		modeCluster = false;
		clusterType = Contact::Config::Cluster::TYPE_LIKELIHOOD;
		requestRender();
		break;
	case '-':
		typeRequest = true;
		contactManifoldShowCluster = true;
		appearance.config.showConfig = true;
		modeCluster = true;
		clusterType = Contact::Config::Cluster::TYPE_TYPE;
		requestRender();
		break;
	case '=':
		poseRequest = true;
		contactManifoldShowCluster = true;
		modeCluster = true;
		appearance.config.showConfig = true;
		clusterType = Contact::Config::Cluster::TYPE_CONFIGURATION;
		requestRender();
		break;
	case '(':
		(modeCluster ? clusterIndexRequest  : configIndexRequest) = -1;
		requestRender();
		break;
	case ')':
		(modeCluster ? clusterIndexRequest  : configIndexRequest) = +1;
		requestRender();
		break;
	case '_':
		modeCluster = !modeCluster;
		context.write("Cluster mode: %s\n", modeCluster ? "ON" : "OFF");
		break;
	case UIKeyboardMouseCallback::KEY_ALT | int(32) :
		selectRequest = true;
		contactManifoldShowCluster = true;
		requestRender();
		break;
	case UIKeyboardMouseCallback::KEY_ALT | int(13) :
		clusterRequest = true;
		requestRender();
		break;
	case '\'':
		modelIndexRequest = -1;
		requestRender();
		break;
	case '\\':
		modelIndexRequest = +1;
		requestRender();
		break;
	};
}

//------------------------------------------------------------------------------
