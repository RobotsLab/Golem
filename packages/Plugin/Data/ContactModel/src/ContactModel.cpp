/** @file ContactModel.cpp
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
#include <Golem/Plugin/Ctrl.h>
#include <Golem/Tools/Menu.h>
#include <Golem/Tools/Cloud.h>
#include <Golem/Data/ContactModel/ContactModel.h>
#include <boost/algorithm/string.hpp>
#ifdef WIN32
#pragma warning (push)
#pragma warning (disable : 4291 4244 4996 4305 4267 4334)
#endif
#include <flann/flann.hpp>
#ifdef WIN32
#pragma warning (pop)
#endif

using namespace golem;
using namespace golem::data;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new data::HandlerContactModel::Desc();
}

//------------------------------------------------------------------------------

golem::data::ItemContactModel::ItemContactModel(HandlerContactModel& handler) :
	Item(handler), handler(handler), dataFile(handler.file), aspectDataFile(handler.file),
	aspectConfigIndex(0), aspectPruningIndexI(0), aspectPruningIndexJ(0), aspectClusterIndexI(0), aspectClusterIndexJ(0), dataIndex(0), viewIndex(0), pathIndex(0)
{}

data::Item::Ptr golem::data::ItemContactModel::clone() const {
	throw Message(Message::LEVEL_ERROR, "ItemContactModel::clone(): not implemented");
}

void golem::data::ItemContactModel::createRender() {
	handler.createRender(*this);
}

void golem::data::ItemContactModel::load(const std::string& prefix, const golem::XMLContext* xmlcontext) {
	// data index
	golem::XMLData("data_index", dataIndex, const_cast<golem::XMLContext*>(xmlcontext), false);

	// model
	std::string modelSuffix;
	golem::XMLData("model", modelSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (modelSuffix.length() > 0) {
		dataFile.load(prefix + modelSuffix, [&](const std::string& path) {
			dataMap.clear();
			dataBackupMap.clear();
			FileReadStream frs(path.c_str());
			frs.read(dataMap, dataMap.end()); // includes query selector
			try {
				frs.read(dataBackupMap, dataBackupMap.end()); // includes query selector
			}
			catch (const golem::Message&) {}
			
		});
	}

	// manifold
	try {
		golem::XMLData(manifoldMap, manifoldMap.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "manifold", false);
	}
	catch (const golem::MsgXMLParser&) {}
	for (golem::Contact::ManifoldSelector::Map::const_iterator i = manifoldMap.begin(); i != manifoldMap.end(); ++i) {
		i->second.assertValid(Assert::Context("ItemContactModel::manifoldMap[]->"));

		// apply
		auto apply = [] (const golem::Contact::ManifoldSelector::Map::value_type& manifold, ContactModel::Data::Map& dataMap) {
			for (auto& dataItem: dataMap)
				if (manifold.first.type == dataItem.first || manifold.first.type == golem::Contact::ManifoldSelector::TypeAny) {
					if (manifold.first.space != golem::Contact::ManifoldSelector::SpaceNone) {
						for (U32 spaceItem = 0; spaceItem < (U32)dataItem.second.spaces.size(); ++spaceItem)
							if (manifold.first.space == spaceItem || manifold.first.space == golem::Contact::ManifoldSelector::SpaceAny)
								dataItem.second.spaces[spaceItem].manifold = manifold.second;
					}
					if (manifold.first.view != golem::Contact::ManifoldSelector::ViewNone) {
						for (U32 viewItem = 0; viewItem < (U32)dataItem.second.views.size(); ++viewItem)
							if (manifold.first.view == viewItem || manifold.first.view == golem::Contact::ManifoldSelector::ViewAny)
								dataItem.second.views[viewItem].manifold = manifold.second;
					}
				}
		};
		apply(*i, dataMap);
		apply(*i, dataBackupMap);
	}

	// aspect
	try {
		std::string aspectSuffix;
		golem::XMLData("aspect", aspectSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
		if (aspectSuffix.length() > 0) {
			aspectDataFile.load(prefix + aspectSuffix, [&](const std::string& path) {
				aspectDataMap.clear();
				FileReadStream(path.c_str()).read(aspectDataMap, aspectDataMap.end()); // includes query selector
			});
		}
	}
	catch (const golem::Message&) {}

	// ready query selector from xml, if successful also overwrite binary version
	try {
		selectorTypeMap.clear();
		std::pair<XMLContext::XMLContextMap::const_iterator, XMLContext::XMLContextMap::const_iterator> range = xmlcontext->getContextMap().equal_range("query");
		for (XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second; ++i) {
			Contact::SelectorTypeMap::value_type selector;
			XMLData(selector, const_cast<XMLContext*>(&i->second), false);
			selectorTypeMap.insert(selector);

			// find corresponding type in dataMap
			ContactModel::Data::Map::iterator dataMapPtr = dataMap.find(selector.first);
			if (dataMapPtr != dataMap.end()) {
				dataMapPtr->second.selectorMap = selector.second;
				//dataFile.setModified(true);
			}
		}
	}
	catch (const golem::Message&) {}
}

void golem::data::ItemContactModel::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	// data index
	golem::XMLData("data_index", const_cast<golem::U32&>(dataIndex), xmlcontext, true);

	// model xml
	std::string modelSuffix = !dataMap.empty() ? handler.modelSuffix : "";
	golem::XMLData("model", modelSuffix, xmlcontext, true);
	// model binary
	if (modelSuffix.length() > 0) {
		//dataFile.setModified(true);
		dataFile.save(prefix + modelSuffix, [=](const std::string& path) {
			FileWriteStream fws(path.c_str());
			fws.write(dataMap.begin(), dataMap.end()); // includes query selector
			fws.write(dataBackupMap.begin(), dataBackupMap.end()); // includes query selector
		});
	}
	else
		dataFile.remove();

	// manifold
	//golem::XMLData(manifoldMap.begin(), manifoldMap.end(), "manifold", xmlcontext, true);
	golem::XMLData(const_cast<golem::Contact::ManifoldSelector::Map&>(manifoldMap), manifoldMap.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "manifold", true);

	// aspect xml
	std::string aspectSuffix = !aspectDataMap.empty() ? handler.aspectSuffix : "";
	golem::XMLData("aspect", aspectSuffix, xmlcontext, true);
	// aspect binary
	if (aspectSuffix.length() > 0) {
		//aspectDataFile.setModified(true);
		aspectDataFile.save(prefix + aspectSuffix, [=](const std::string& path) {
			FileWriteStream(path.c_str()).write(aspectDataMap.begin(), aspectDataMap.end()); // includes query selector
		});
	}
	else
		aspectDataFile.remove();

	// write query selector to xml
	for (Contact::SelectorTypeMap::const_iterator i = selectorTypeMap.begin(); i != selectorTypeMap.end(); ++i)
		XMLData(const_cast<Contact::SelectorTypeMap::value_type&>(*i), xmlcontext->createContext("query"), true);
}

//------------------------------------------------------------------------------

void golem::data::ItemContactModel::setData(const ContactModel::Data::Map& dataMap) {
	this->dataMap = dataMap;
	dataFile.setModified(true);
}

const golem::data::ContactModel::Data::Map& golem::data::ItemContactModel::getData() const {
	return dataMap;
}

//------------------------------------------------------------------------------

void golem::data::HandlerContactModel::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	golem::data::Handler::Desc::load(context, xmlcontext);

	golem::XMLData("planner_index", plannerIndex, const_cast<golem::XMLContext*>(xmlcontext));

	manipulatorDesc->load(xmlcontext->getContextFirst("manipulator"));
	configurationDesc->load(xmlcontext->getContextFirst("configuration"));
	modelDescMap.clear();
	golem::XMLData(modelDescMap, modelDescMap.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "model", false);

	dataDesc.load(xmlcontext);
	aspectDesc->load(xmlcontext->getContextFirst("aspect"));
	manifoldDesc->load(xmlcontext->getContextFirst("manifold"));

	golem::XMLData("default_type", defaultType, const_cast<golem::XMLContext*>(xmlcontext), false);

	appearance.load(xmlcontext->getContextFirst("appearance"));

	golem::XMLData("model_suffix", modelSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("aspect_suffix", aspectSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
}

golem::data::Handler::Ptr golem::data::HandlerContactModel::Desc::create(golem::Context &context) const {
	Handler::Ptr handler(new HandlerContactModel(context));
	to<HandlerContactModel>(handler.get())->create(*this);
	return handler;
}

//------------------------------------------------------------------------------

golem::data::HandlerContactModel::HandlerContactModel(golem::Context &context) : Handler(context), mode(MODE_DATA), indexIRequest(0), indexJRequest(0), modelInfoRequest(false), subspaceDistRequest(false) {
}

void golem::data::HandlerContactModel::create(const Desc& desc) {
	golem::data::Handler::create(desc);

	plannerIndex = desc.plannerIndex;

	// shallow copy, TODO clone member function
	manipulatorDesc = desc.manipulatorDesc;
	configurationDesc = desc.configurationDesc;
	modelDescMap = desc.modelDescMap;
	dataDesc = desc.dataDesc;
	
	aspectDesc = desc.aspectDesc; // shallow copy
	manifoldDesc = desc.manifoldDesc; // shallow copy

	defaultType = desc.defaultType;

	appearance = desc.appearance;

	modelSuffix = desc.modelSuffix;
	aspectSuffix = desc.aspectSuffix;

	importTypes = {
		getFileExtContactModel(),
	};

	transformInterfaces = {
		"Point3D",
		"Trajectory",
	};
}

//------------------------------------------------------------------------------

const char* golem::data::HandlerContactModel::modeName[MODE_SIZE] = {
	"Data", "View", "Path", "Aspect clusters", "Aspect pruning", "Aspect configs"
};

std::string golem::data::HandlerContactModel::getFileExtContactModel() {
	return std::string(".model");
}

std::string golem::data::HandlerContactModel::getFileExtContactAspect() {
	return std::string(".aspect");
}

golem::data::Item::Ptr golem::data::HandlerContactModel::create() const {
	return Item::Ptr(new ItemContactModel(*const_cast<HandlerContactModel*>(this))); // safe: no modification here - only passing reference
}

//------------------------------------------------------------------------------

golem::U32 golem::data::HandlerContactModel::getPlannerIndex() const {
	return plannerIndex;
}

void golem::data::HandlerContactModel::set(const golem::Planner& planner, const ControllerId::Seq& controllerIDSeq) {
	manipulator = manipulatorDesc->create(planner, controllerIDSeq);
	configuration = configurationDesc->create(*manipulator);
	
	// models
	modelMap.clear();
	for (golem::Model::Desc::Map::const_iterator i = modelDescMap.begin(); i != modelDescMap.end(); ++i)
		modelMap.insert(std::make_pair(i->first, i->second->create(context, i->first)));

	aspect = aspectDesc->create(*configuration);
	manifold = manifoldDesc->create(*configuration);
}

//------------------------------------------------------------------------------

std::string golem::data::HandlerContactModel::toString(const golem::data::ContactModel::Data& data) const {
	std::stringstream strContacts;
	for (Contact3D::Data::Map::const_iterator i = data.contacts.begin(); i != data.contacts.end(); ++i) {
		strContacts << "(" << i->first + 1 << ", " << i->second.model.size() << ")";
	}

	std::stringstream strViews;
	for (golem::Contact::View::Seq::const_iterator i = data.views.begin(); i != data.views.end(); ++i) {
		strViews << "(" << i->object + 1 << ", " << i->space + 1 << ", ";
		for (golem::Contact::View::ModelPtr::Map::const_iterator j = i->models.begin(); j != i->models.end(); ++j)
			strViews << "(" << j->first.toString() << ", " << j->second.index + 1 << ", " << j->second.weight << ")";
		strViews << ")";
	}

	std::stringstream strSpaces;
	for (golem::Configuration::Space::Seq::const_iterator i = data.spaces.begin(); i != data.spaces.end(); ++i) {
		strSpaces << "(" << i->configs.size() << ", " << i->paths.size() << ")";
	}

	std::stringstream str;
	str << "contacts_{size=" << data.contacts.size() << ", (contact, feat): " << strContacts.str()
		<< "}, views_{size=" << data.views.size() << ", (obj, type, (link, contact, weight)): " << strViews.str()
		<< "}, spaces_{size=" << data.spaces.size() << ", (configs, paths): " << strSpaces.str() << "}";

	return str.str();
}

std::string golem::data::HandlerContactModel::toString(golem::U32 viewIndex, golem::U32 pathIndex, const golem::data::ContactModel::Data& data) const {
	std::stringstream str;

	// view
	const golem::Contact::View* view = data.views.empty() ? nullptr : data.views.data() + std::min(viewIndex, (U32)data.views.size() - 1);
	str << "view_{";
	if (view) {
		str << "index=" << viewIndex + 1 << ", contacts_{size=" << view->models.size() << ", (link, contact, weight, feat): ";
		for (auto &i : view->models) {
			Contact3D::Data::Map::const_iterator j = data.contacts.find(i.second.index);
			str << "(" << i.first.toString() << ", " << i.second.index + 1 << ", " << i.second.weight << ", " << (j != data.contacts.end() ? std::to_string(j->second.model.size()) : "not found") << ")";
		}
		str << "}";
	}
	else
		str << "not available";
	str << "}, ";

	// type
	const golem::Configuration::Space* space = data.spaces.empty() ? nullptr : data.spaces.data() + std::min(view ? view->space : 0, (U32)data.spaces.size() - 1);
	str << "space_{";
	if (space) {
		str << "index=" << view->space + 1 << "}";
	}
	else
		str << "not available";
	str << "}, ";

	// path
	const Configuration::Path* path = !space || space->paths.empty() ? nullptr : space->paths.data() + std::min(pathIndex, (U32)space->paths.size() - 1);
	str << "path_{";
	if (space) {
		str << "size=" << space->paths.size() << ", size[" << pathIndex + 1 << "]=" << space->paths[pathIndex].size() << "}";
	}
	else
		str << "not available";
	str << "}";

	return str.str();
}

bool golem::data::HandlerContactModel::create(const std::string& name, golem::I32 objectIndex, const Point3D& points, const golem::WaypointCtrl::Seq& waypoints, golem::data::ContactModel::Data& data) const {
	// manipulator
	const Manipulator& manipulator = configuration->getManipulator();

	// space index
	const golem::U32 spaceIndex = golem::U32(data.spaces.size());

	// new sub-space
	golem::Configuration::Space space(name);
	configuration->create(configuration->create(waypoints), space); // throws

	// joint and base frames
	const golem::Mat34 gripFrame = space.paths.back().getGrip().frame.toMat34();
	golem::WorkspaceJointCoord joints;
	manipulator.getJointFrames(space.paths.back().getGrip().config, gripFrame, joints);

	// next contact model index
	const golem::U32 modelIndexBegin = data.contacts.empty() ? golem::U32(0) : data.contacts.rbegin()->first + 1;
	golem::U32 modelIndex = modelIndexBegin;
	
	// new view
	golem::Contact::View view(objectIndex, spaceIndex);

	// joint links
	for (golem::Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		const Manipulator::Link jointLink(i);
		// find model
		const golem::Model::Map::const_iterator jointModel = jointLink.get<golem::Model::Map::const_iterator>(modelMap);
		if (jointModel != modelMap.end()) {
			// bounds
			golem::Bounds::Seq bounds;
			manipulator.getJointBounds(i, joints[i], bounds);
			// bounds -> triangles
			golem::Contact3D::Triangle::Seq triangles;
			Contact3D::convert(bounds, triangles);
			// find contacts
			Contact3D::Data contacts;
			if (jointModel->second->create(points, joints[i], triangles, contacts)) {
				// insert contact model
				data.contacts.insert(std::make_pair(modelIndex, contacts));
				// update view with default weight
				view.models.insert(std::make_pair(jointLink, Contact::View::ModelPtr(modelIndex, REAL_ONE)));
				// increment model index
				++modelIndex;
			}
		}
	}

	// base link
	const Manipulator::Link baseLink(Manipulator::Link::TYPE_BASE);
	// find model
	const golem::Model::Map::const_iterator baseModel = baseLink.get<golem::Model::Map::const_iterator>(modelMap);
	if (baseModel != modelMap.end()) {
		// bounds
		golem::Bounds::Seq bounds;
		manipulator.getBaseBounds(gripFrame, bounds);
		// bounds -> triangles
		golem::Contact3D::Triangle::Seq triangles;
		Contact3D::convert(bounds, triangles);
		// find contacts
		Contact3D::Data contacts;
		if (baseModel->second->create(points, gripFrame, triangles, contacts)) {
			// insert contact model
			data.contacts.insert(std::make_pair(modelIndex, contacts));
			// update view with default weight
			view.models.insert(std::make_pair(baseLink, Contact::View::ModelPtr(modelIndex, REAL_ONE)));
			// increment model index
			++modelIndex;
		}
	}

	if (modelIndex <= modelIndexBegin) {
		context.debug("HandlerContactModel::create(): no contacts detected\n");
		return false;
	}

	// update points
	view.point3D = &points;
	view.points.resize(points.getSize());
	for (size_t i = 0; i < points.getSize(); ++i)
		view.points[i] = points.getPoint(i);

	// update views
	data.views.push_back(view);
	// update types
	data.spaces.push_back(space);

	return true;
}

//------------------------------------------------------------------------------

Item::Ptr golem::data::HandlerContactModel::import(const std::string& path) {
	// check extension
	if (std::find(importTypes.begin(), importTypes.end(), getExt(path)) == importTypes.end())
		throw Message(Message::LEVEL_ERROR, "HandlerContactModel::import(): unknown file type %s", getExt(path).c_str());

	Item::Ptr item(create());
	ItemContactModel* itemContactModel = to<ItemContactModel>(item.get());

	// block item rendering (hasRenderBlock()=true)
	RenderBlock renderBlock(*this);

	FileReadStream frs(path.c_str());
	frs.read(itemContactModel->dataMap, itemContactModel->dataMap.end());
	itemContactModel->dataMap.clear(); // will be re-processed from backup
	frs.read(itemContactModel->dataBackupMap, itemContactModel->dataBackupMap.end());
	
	// try to load aspect data as well
	if (aspect->getDesc().enabled) {
		try {
			std::string file(path);
			file.replace(file.begin() + file.length() - modelSuffix.size(), file.end(), aspectSuffix);
			FileReadStream(file.c_str()).read(itemContactModel->aspectDataMap, itemContactModel->aspectDataMap.end());
		}
		catch (const Message& msg) {
			context.write(msg);
		}
	}

	// process multiple type entries, insert to item
	for (ContactModel::Data::Map::const_iterator ptr = itemContactModel->dataBackupMap.begin(); ptr != itemContactModel->dataBackupMap.end();) {
		const ContactModel::Data::Map::const_iterator end = itemContactModel->dataBackupMap.upper_bound(ptr->first);

		// contact data
		golem::data::ContactModel::Data data;
		data.add(dataDesc, *configuration, ptr, end);

		// try to load aspect data
		golem::Aspect::Data aspectData;
		if (aspect->getDesc().enabled) {
			Aspect::Data::Map::iterator aspectDataPtr = itemContactModel->aspectDataMap.find(ptr->first);
			if (aspectDataPtr != itemContactModel->aspectDataMap.end()) {
				aspectData = aspectDataPtr->second;
				itemContactModel->aspectDataMap.erase(ptr->first); // multimap - erase entire range, not only single item
			}
			else
				context.notice("HandlerContactModel::import(): Full processing mode: unable to find aspect data for contact type: %s\n", ptr->first.c_str());
			// processing - single type, one or more objects and trajectories
			aspect->process(data, aspectData, getUICallback());
		}

		// duplicate contact selector
		itemContactModel->selectorTypeMap.insert(std::make_pair(ptr->first, data.selectorMap));

		itemContactModel->dataMap.insert(std::make_pair(ptr->first, data));
		if (aspect->getDesc().enabled)
			itemContactModel->aspectDataMap.insert(std::make_pair(ptr->first, aspectData));

		// update pointer
		ptr = end;

		// draw
		UI::CriticalSectionWrapper cs(getUICallback());
		renderer.reset();
		rendererPoints.reset();
		data.draw(*configuration, appearance, 0, 0, renderer, &rendererPoints);
		if (aspect->getDesc().enabled)
			aspectData.draw(aspectDesc->dataAppearance, data, renderer);
	}

	itemContactModel->dataFile.setModified(true);
	itemContactModel->aspectDataFile.setModified(true);

	return item;
}

const StringSeq& golem::data::HandlerContactModel::getImportFileTypes() const {
	return importTypes;
}

//------------------------------------------------------------------------------

data::Item::Ptr golem::data::HandlerContactModel::transform(const Item::List& input) {
	Item::Ptr item(create());
	ItemContactModel* itemContactModel = to<ItemContactModel>(item.get());
	
	// block item rendering (hasRenderBlock()=true)
	RenderBlock renderBlock(*this);

	if (input.empty())
		throw Message(Message::LEVEL_ERROR, "HandlerContactModel::transform(): expected item order: <image_1_1> ... <image_1_n1> <complete_image_1> <trajectory_1> ... <image_m_1> ... <image_m_nm> <complete_image_m> <trajectory_m>");

	// contact compression
	bool contactCompression = aspect->getDesc().enabled;
	if (getUICallback() && getUICallback()->hasInputEnabled())
		contactCompression = Menu(context, *getUICallback()).option(contactCompression ? 1 : 0, "Contact compression: ", { "NO", "YES" }) == 1;

	// collect data
	typedef std::multimap< std::string, std::pair<data::Point3D::ConstSeq, const Trajectory*> > DataMap;
	DataMap dataMap;
	data::Point3D::ConstSeq pointsParam;
	const Trajectory* trajectoryParam = nullptr;
	bool useDefaultType = contactCompression;
	std::string trajectoryType;
	for (Item::List::const_iterator i = input.begin(), j = i; i != input.end(); ++i) {
		++j;
		const Point3D* points = is<const Point3D>(*i);
		if (points) {
			pointsParam.push_back(points);
		}
		const Trajectory* trajectory = is<const Trajectory>(*i);
		if (trajectory) {
			trajectoryParam = trajectory;
			trajectoryType = (*i)->first;
		}

		// done? - 3 cases:
		// 1) points && trajectory now, 2) points && trajectory before && no more data, 3) points && trajectory before && trajectory next
		if (!pointsParam.empty() && (trajectory || trajectoryParam && (j == input.end() || is<const Trajectory>(*j)))) {
			// type
			std::string type = useDefaultType ? defaultType : trajectoryType;

			// data name/contact type: trajectory name
			if (getUICallback() && getUICallback()->hasInputEnabled())
				Menu(context, *getUICallback()).readString("Enter contact type: ", type, [&] (std::string& str) {
					useDefaultType = !useDefaultType;
					str = useDefaultType ? defaultType : trajectoryType;
				});

			// add to sorted collection
			dataMap.insert(std::make_pair(type, std::make_pair(pointsParam, trajectoryParam)));

			// clear local variables
			pointsParam.clear();
			trajectoryParam = nullptr;
		}
	}

	// partial views only
	bool partialViewsOnly = contactCompression;
	if (getUICallback() && getUICallback()->hasInputEnabled())
		partialViewsOnly = Menu(context, *getUICallback()).option(partialViewsOnly ? 1 : 0, "Partial views only: ", { "NO", "YES" }) == 1;

	// create contact data in type order
	for (DataMap::const_iterator ptr = dataMap.begin(); ptr != dataMap.end();) {
		const DataMap::const_iterator end = dataMap.upper_bound(ptr->first);
		golem::I32 objectIndex = 1;

		// single type, multiple objects
		for (; ptr != end; ++ptr, ++objectIndex) {
			// single object and trajectory, multiple views
			for (data::Point3D::ConstSeq::const_iterator view = ptr->second.first.begin(); view != ptr->second.first.end(); ++view) {
				// complete view last
				const bool completeView = partialViewsOnly || view + 1 == ptr->second.first.end();
				
				// create - single object, trajectory, view
				ContactModel::Data data;
				if (!create(ptr->first, completeView ? -objectIndex : +objectIndex, **view, ptr->second.second->getWaypoints(), data))
					continue;

				// update
				itemContactModel->dataBackupMap.insert(std::make_pair(ptr->first, data));

				// draw
				UI::CriticalSectionWrapper cs(getUICallback());
				renderer.reset();
				rendererPoints.reset();
				data.draw(*configuration, appearance, 0, 0, renderer, &rendererPoints);
			}
		}
	}

	// process multiple type entries, insert to item
	for (ContactModel::Data::Map::const_iterator ptr = itemContactModel->dataBackupMap.begin(); ptr != itemContactModel->dataBackupMap.end();) {
		const ContactModel::Data::Map::const_iterator end = itemContactModel->dataBackupMap.upper_bound(ptr->first);

		// contact data
		golem::data::ContactModel::Data data;
		data.add(dataDesc, *configuration, ptr, end);

		// compute manifolds (uncompressed)
		if (!manifoldDesc->manifoldCompressed)
			manifold->process(data, getUICallback());

		// try to load aspect data
		golem::Aspect::Data aspectData;
		// processing - single type, one or more objects and trajectories
		if (contactCompression)
			aspect->process(data, aspectData, getUICallback());

		// compute manifolds (compressed)
		if ( manifoldDesc->manifoldCompressed)
			manifold->process(data, getUICallback());

		// duplicate contact selector
		itemContactModel->selectorTypeMap.insert(std::make_pair(ptr->first, data.selectorMap));

		itemContactModel->dataMap.insert(std::make_pair(ptr->first, data));
		if (contactCompression)
			itemContactModel->aspectDataMap.insert(std::make_pair(ptr->first, aspectData));

		// update pointer
		ptr = end;

		// draw
		UI::CriticalSectionWrapper cs(getUICallback());
		renderer.reset();
		rendererPoints.reset();
		data.draw(*configuration, appearance, 0, 0, renderer, &rendererPoints);
		if (contactCompression)
			aspectData.draw(aspectDesc->dataAppearance, data, renderer);
	}

	if (itemContactModel->dataMap.empty())
		throw Message(Message::LEVEL_ERROR, "HandlerContactModel::transform(): no training data has been created");

	// do not keep backup contact model data if aspect processing is disabled
	if (!contactCompression)
		itemContactModel->dataBackupMap.clear();

	itemContactModel->dataFile.setModified(true);
	itemContactModel->aspectDataFile.setModified(true);

	return item;
}

const StringSeq& golem::data::HandlerContactModel::getTransformInterfaces() const {
	return transformInterfaces;
}

bool golem::data::HandlerContactModel::isTransformSupported(const Item& item) const {
	return is<const data::Trajectory>(&item) || is<const data::Normal3D>(&item) || is<const data::Feature3D>(&item) || is<const data::Part3D>(&item);
}

//------------------------------------------------------------------------------

void golem::data::HandlerContactModel::createRender(const ItemContactModel& item) {
	UI::CriticalSectionWrapper cs(getUICallback());

	renderer.reset();
	rendererPoints.reset();
	if (!item.dataMap.empty()) {
		if (mode == MODE_DATA)
			item.dataIndex = (golem::U32)golem::Math::clamp(golem::I32(item.dataIndex) + indexIRequest, 0, golem::I32(item.dataMap.size()) - 1);
		else if (mode == MODE_VIEW)
			item.viewIndex = (golem::U32)golem::Math::clamp(golem::I32(item.viewIndex) + indexIRequest, 0, golem::numeric_const<I32>::MAX - 1);
		else if (mode == MODE_PATH)
			item.pathIndex = (golem::U32)golem::Math::clamp(golem::I32(item.pathIndex) + indexIRequest, 0, golem::numeric_const<I32>::MAX - 1);
		
		ItemContactModel::Data::Map::const_iterator ptr = item.dataMap.begin();
		std::advance(ptr, item.dataIndex);
		ptr->second.clamp(item.viewIndex, item.pathIndex);

		if (mode == MODE_DATA || mode == MODE_VIEW || mode == MODE_PATH) {
			ptr->second.draw(*configuration, appearance, item.viewIndex, item.pathIndex, renderer, &rendererPoints);

			if (indexIRequest != 0)
				context.write("Training data #%u/%u: name=%s, %s\n", item.dataIndex + 1, (U32)item.dataMap.size(), ptr->first.c_str(), toString(item.viewIndex, item.pathIndex, ptr->second).c_str());
		}
		else {
			const Aspect::Data::Map::const_iterator ptrAspect = item.aspectDataMap.find(ptr->first);

			// aspect data is optional
			if (ptrAspect != item.aspectDataMap.end()) {
				const Aspect::Data::Appearance aspectAppearance = aspectDesc->dataAppearance;

				// aspect data
				if (mode == MODE_ASPECT_CLUSTERS) {
					ptrAspect->second.draw(aspectAppearance, ptr->second, renderer);

					if (!ptrAspect->second.contactComplete.empty() && !ptrAspect->second.simMat.empty()) {
						item.aspectClusterIndexI = (golem::U32)golem::Math::clamp(golem::I32(item.aspectClusterIndexI) + indexIRequest, 0, golem::I32(ptrAspect->second.contactComplete.size()) - 1);
						item.aspectClusterIndexJ = (golem::U32)golem::Math::clamp(golem::I32(item.aspectClusterIndexJ) + indexJRequest, 0, golem::I32(ptrAspect->second.contactComplete.size()) - 1);

						Contact3D::Data::Map::const_iterator i = ptrAspect->second.contactComplete.begin();
						std::advance(i, item.aspectClusterIndexI);
						Contact3D::Data::Map::const_iterator j = ptrAspect->second.contactComplete.begin();
						std::advance(j, item.aspectClusterIndexJ);

						renderer.addAxes3D(aspectAppearance.referenceFramePair, aspectAppearance.frameSize);
						Contact3D::drawPoints(i->second.model, aspectAppearance.pointColour, aspectAppearance.normalLen, aspectAppearance.referenceFramePair, renderer);

						// Draw cloud j in the frame of cloud i
						// Frame_j = Frame_i * simMat[i,j] => Frame_i = Frame_j * (simMat[i,j])^-1 = Frame_j * simMat[j, i]
						const Mat34 localFrame = aspectAppearance.referenceFramePair * ptrAspect->second.simMat[item.aspectClusterIndexJ][item.aspectClusterIndexI].second;
						renderer.addAxes3D(localFrame, aspectAppearance.frameSize);
						Contact3D::drawPoints(j->second.model, aspectAppearance.pointSimColour, aspectAppearance.normalLen, localFrame, renderer);

						if (indexIRequest != 0 || indexJRequest != 0)
							context.write("Aspect cluster=(%u/%u, %u/%u), size=%u, similarity=%e\n", i->first + 1, item.aspectClusterIndexI + 1, j->first + 1, item.aspectClusterIndexJ + 1, (U32)ptrAspect->second.contactComplete.size(), ptrAspect->second.simMat[item.aspectClusterIndexI][item.aspectClusterIndexJ].first);
					}
				}
				else {
					const Contact::View::Seq& views = ptrAspect->second.procViewsPruning;

					if (!views.empty()) {
						if (mode == MODE_ASPECT_PRUNING) {
							item.aspectPruningIndexI = (golem::U32)golem::Math::clamp(golem::I32(item.aspectPruningIndexI) + indexIRequest, 0, golem::I32(views.size()) - 1);
							item.aspectPruningIndexJ = (golem::U32)golem::Math::clamp(golem::I32(item.aspectPruningIndexJ) + indexJRequest, 0, golem::I32(views.size()) - 1);
						}

						const Contact::View::Seq::const_iterator i = views.begin() + item.aspectPruningIndexI;
						const Contact::View::Seq::const_iterator j = views.begin() + item.aspectPruningIndexJ;

						// point cloud
						std::string strObject = "index=" + std::to_string(j->getObjectIndex()) + ", points=";
						const Aspect::Data::PointsMap::const_iterator points = ptrAspect->second.pointsMap.find(j->getObjectIndex());
						if (points != ptrAspect->second.pointsMap.end()) {
							const golem::_Mat34<golem::F32> frame(aspectAppearance.referenceFrameView);
							for (F32Vec3Seq::const_iterator i = points->second.begin(); i != points->second.end(); ++i)
								renderer.addPoint(frame * *i, aspectAppearance.pointColour);
							strObject += std::to_string(points->second.size());
						}
						else
							strObject += "unavailable";

						// configs
						std::string strIndex = std::to_string(item.aspectPruningIndexI + 1);
						std::string strEval = "unavailable";
						std::string strConfig = "unavailable";
						if (item.aspectPruningIndexI < (U32)ptrAspect->second.viewDataMap.size() && item.aspectPruningIndexJ < (U32)ptrAspect->second.viewDataMap[item.aspectPruningIndexI].evalSeq.size() && !ptrAspect->second.viewDataMap[item.aspectPruningIndexI][item.aspectPruningIndexJ].configs.empty()) {
							const Aspect::ViewData& viewData = ptrAspect->second.viewDataMap[item.aspectPruningIndexI];
							strIndex += "->" + (viewData.index < (U32)ptrAspect->second.viewDataMap.size() ? std::to_string(viewData.index + 1) : std::string("removed"));
							strEval = "total=" + std::to_string(viewData.eval);
							
							const Aspect::ViewData::Eval& viewEval = viewData[item.aspectPruningIndexJ];

							if (!viewEval.configs.empty()) {
								if (mode == MODE_ASPECT_CONFIGS) {
									item.aspectConfigIndex = (golem::U32)golem::Math::clamp(golem::I32(item.aspectConfigIndex) + indexIRequest, 0, golem::I32(viewEval.configs.size()) - 1);
								}
								if (viewEval.configs[item.aspectConfigIndex] != nullptr && !viewEval.configs[item.aspectConfigIndex]->path.empty()) {
									Manipulator::Config c = viewEval.configs[item.aspectConfigIndex]->path.back();
									c.frame.multiply(RBCoord(aspectAppearance.referenceFrameView), c.frame);
									this->appearance.path.manipulator.draw(*manipulator, c, renderer);
									
									strConfig = "index=" + std::to_string(item.aspectConfigIndex + 1) + ", eval=";

									if (item.aspectConfigIndex < (U32)viewEval.evals.size())
										strConfig += std::to_string(viewEval.evals[item.aspectConfigIndex]);
									else
										strConfig += "unavailable";
								}

								strEval += ", pair=" + std::to_string(viewEval.eval);
							}
						}

						if (indexIRequest != 0 || indexJRequest != 0)
							context.write("Aspect view=(%u/%u, %u/%u), index=%s, eval_{%s}, object_{%s}, config_{%s}\n", item.aspectPruningIndexI + 1, (U32)views.size(), item.aspectPruningIndexJ + 1, (U32)views.size(), strIndex.c_str(), strEval.c_str(), strObject.c_str(), strConfig.c_str());
					}
					else
						context.debug("Aspect views not available\n");
				}
			}
			else
				context.debug("Aspect data not available\n");
		}

		indexIRequest = 0;
		indexJRequest = 0;

		if (subspaceDistRequest) {
			context.write("Trajectory stddev_dist=%f\n", appearance.path.subspaceDistVal);
			subspaceDistRequest = false;
		}
		if (modelInfoRequest) {
			for (ContactModel::Data::Map::const_iterator cmd = item.dataMap.begin(); cmd != item.dataMap.end(); ++cmd) {
				// contacts
				for (Contact3D::Data::Map::const_iterator cm = cmd->second.contacts.begin(); cm != cmd->second.contacts.end(); ++cm)
					for (Contact3D::Seq::const_iterator cs = cm->second.model.begin(); cs != cm->second.model.end(); ++cs)
						printf("Data=%s, contact=%u, model=%s, weight=%f, frame={(%f, %f, %f), (%f, %f, %f, %f)}\n", cmd->first.c_str(), cm->first, cs->model == Contact3D::INDEX_DEFAULT ? "unavailable" : std::to_string(cs->model).c_str(), cs->weight, cs->local.p.x, cs->local.p.y, cs->local.p.z, cs->local.q.x, cs->local.q.y, cs->local.q.z, cs->local.q.w);
			}
			modelInfoRequest = false;
		}
	}
}

void golem::data::HandlerContactModel::render() const {
	if (hasRenderBlock()) return;

	rendererPoints.render();
	renderer.render();
}

void golem::data::HandlerContactModel::customRender() const {
}

//------------------------------------------------------------------------------

void golem::data::HandlerContactModel::mouseHandler(int button, int state, int x, int y) {
	if (hasRenderBlock()) return;

	if (mode == HandlerContactModel::MODE_PATH && state == 0 && (button == 3 || button == 4 || button == 1)) {
		appearance.path.subspaceDist = button == 3 ? appearance.path.subspaceDist + 1 : button == 4 ? appearance.path.subspaceDist - 1 : 0;
		subspaceDistRequest = true;
		requestRender();
	}
}

void golem::data::HandlerContactModel::motionHandler(int x, int y) {
}

void golem::data::HandlerContactModel::keyboardHandler(int key, int x, int y) {
	if (hasRenderBlock()) return;

	switch (key) {
	case '4':
		mode = mode == HandlerContactModel::MODE_DATA ? HandlerContactModel::MODE_SIZE - 1 : (HandlerContactModel::Mode)(mode - 1);
		context.write("Contact data mode: %s\n", HandlerContactModel::modeName[(size_t)mode]);
		requestRender();
		break;
	case '5':
		mode = mode == HandlerContactModel::MODE_SIZE - 1 ? HandlerContactModel::MODE_DATA : (HandlerContactModel::Mode)(mode + 1);
		context.write("Contact data mode: %s\n", HandlerContactModel::modeName[(size_t)mode]);
		requestRender();
		break;
	case '6':
		appearance.pointShow = !appearance.pointShow;
		context.write("Points: %s\n", appearance.pointShow ? "ON" : "OFF");
		requestRender();
		break;
	case '7':
		appearance.pathShow = !appearance.pathShow;
		context.write("Path: %s\n", appearance.pathShow ? "ON" : "OFF");
		requestRender();
		break;
	case '8':
		appearance.relation = appearance.relation == Contact3D::RELATION_SIZE - 1 ? Contact3D::RELATION_NONE : (Contact3D::Relation)(appearance.relation + 1);
		context.write("Contact relation: %s\n", Contact3D::relationName[(size_t)appearance.relation]);
		requestRender();
		break;
	case '9':
		indexJRequest = -1;
		requestRender();
		break;
	case '0':
		indexJRequest = +1;
		requestRender();
		break;
	case '(':
		indexIRequest = -1;
		requestRender();
		break;
	case ')':
		indexIRequest = +1;
		requestRender();
		break;
	case '=':
		modelInfoRequest = true;
		requestRender();
		break;
	};
}

//------------------------------------------------------------------------------