/** @file Data.cpp
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Contact/Data.h>
#include <Golem/Sys/XMLData.h>

using namespace golem;
using namespace golem::data;

//------------------------------------------------------------------------------

const std::string golem::data::ContactModel::Data::Header::ID = "golem::data::ContactModel::Data";
const golem::Header::Version golem::data::ContactModel::Data::Header::VERSION = golem::Header::Version({ (1 << 0) /* major */ | (0 << 16) /* minor */ });

//------------------------------------------------------------------------------

void golem::data::ContactModel::Data::Desc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("copy_points", copyPoints, const_cast<golem::XMLContext*>(xmlcontext), false);
	contact3DDesc.load(xmlcontext->getContextFirst("contact_3d"));
}

void golem::data::ContactModel::Data::Appearance::load(const golem::XMLContext* xmlcontext) {
	path.load(xmlcontext);
	contacts.clear();
	golem::XMLData(contacts, contacts.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "contact", false);
	point.load(xmlcontext->getContextFirst("point"));
	try {
		manifold.load(xmlcontext->getContextFirst("manifold"));
	}
	catch (const golem::MsgXMLParser&) {}
}

//------------------------------------------------------------------------------

void golem::data::ContactModel::Data::add(const Desc& desc, const golem::Configuration& configuration, const Contact3D::Data::Map& contacts, const Contact::View::Seq& views, const Configuration::Space::Seq& spaces, const Contact::SelectorMap& selectorMap) {
	// old -> new, old indices are unique only within ContactModel::Data
	ContactReMap contactReMap;

	// split into separate clusters
	U32 contactIndex = this->contacts.empty() ? 0 : this->contacts.rbegin()->first + 1;

	// iterate over views
	for (golem::Contact::View::Seq::const_iterator v = views.begin(); v != views.end(); ++v) {
		golem::Contact::View view;

		// contact models
		for (golem::Contact::View::ModelPtr::Map::const_iterator c = v->models.begin(); c != v->models.end(); ++c) {
			golem::Contact::View::ModelPtr::Map::value_type contactPtr = *c;
			golem::Contact3D::Data::Map::const_iterator contact = contacts.find(contactPtr.second.index);
			if (contact == contacts.end())
				throw Message(Message::LEVEL_ERROR, "ContactModel::Data::add(): unable to find contact model: view=#%u, link=%s, model=%u", U32(v - views.begin()) + 1, contactPtr.first.toString().c_str(), contactPtr.second.index + 1);
			// re-map contact index
			ContactReMap::const_iterator contactReMapPtr = contactReMap.find(contactPtr.second.index);
			if (contactReMapPtr == contactReMap.end())
				contactReMapPtr = contactReMap.insert(contactReMap.end(), std::make_pair(contactPtr.second.index, contactIndex++));
			const_cast<U32&>(contactPtr.second.index) = contactReMapPtr->second;
			// copy
			view.models.insert(contactPtr);
			if (this->contacts.find(contactPtr.second.index) == this->contacts.end()) // false only if the input data share contact models
				this->contacts.insert(std::make_pair(contactPtr.second.index, contact->second)); // data
		}

		// update spaces
		if (v->space >= spaces.size())
			throw Message(Message::LEVEL_ERROR, "ContactModel::Data::add(): invalid space index: view=#%u, index=%u/%u", U32(v - views.begin()) + 1, v->space + 1, U32(spaces.size()));
		view.space = 0;
		// search if it is already on the list
		for (; view.space < U32(this->spaces.size()) && !equals(spaces[v->space], this->spaces[view.space], configuration.getManipulator().getInfo().getJoints()); ++view.space);
		// if not found - make a copy
		if (view.space >= U32(this->spaces.size()))
			this->spaces.push_back(spaces[v->space]);

		// others just copy
		view.object = v->object;
		view.weight = v->weight;
		view.point3D = v->point3D;
		
		// optional
		if (desc.copyPoints)
			view.points = v->points;

		// done
		this->views.push_back(view); // data
	}

	// normalise
	if (!golem::Sample<Real>::normalise<golem::Ref1>(this->views))
		throw Message(Message::LEVEL_ERROR, "ContactModel::Data::add(): Unable to normalise view distribution");

	// contact selector
	for (ContactReMap::const_iterator i = contactReMap.begin(); i != contactReMap.end(); ++i) {
		Contact::SelectorMap::const_iterator j = selectorMap.find(i->first);
		if (j != selectorMap.end())
			this->selectorMap.insert(std::make_pair(i->second, j->second)); // insert selector with new index
	}
}

void golem::data::ContactModel::Data::add(const Desc& desc, const golem::Configuration& configuration, const Data& data) {
	add(desc, configuration, data.contacts, data.views, data.spaces, data.selectorMap);
}

void golem::data::ContactModel::Data::add(const Desc& desc, const golem::Configuration& configuration, golem::data::ContactModel::Data::Map::const_iterator begin, golem::data::ContactModel::Data::Map::const_iterator end) {
	// merge
	for (golem::data::ContactModel::Data::Map::const_iterator ptr = begin; ptr != end; ++ptr)
		add(desc, configuration, ptr->second);

	// compute contact properties
	Contact3D::process(desc.contact3DDesc, contacts);
}

void golem::data::ContactModel::Data::clear() {
	contacts.clear();
	views.clear();
	spaces.clear();
	selectorMap.clear();
}

bool golem::data::ContactModel::Data::empty() {
	return contacts.empty();
}

void golem::data::ContactModel::Data::clamp(golem::U32& viewIndex, golem::U32& pathIndex) const {
	// view
	const Contact::View* view = this->views.empty() ? nullptr : this->views.data() + (viewIndex = std::min(viewIndex, (U32)this->views.size() - 1));
	if (!view)
		viewIndex = 0;
	
	// type
	const Configuration::Space* space = this->spaces.empty() ? nullptr : this->spaces.data() + std::min(view ? view->space : 0, (U32)this->spaces.size() - 1);
	
	// path
	const Configuration::Path* path = !space || space->paths.empty() ? nullptr : space->paths.data() + (pathIndex = std::min(pathIndex, (U32)space->paths.size() - 1));
	if (!view)
		pathIndex = 0;
}

void golem::data::ContactModel::Data::draw(const Configuration& configuration, const Appearance& appearance, golem::U32 viewIndex, golem::U32 pathIndex, golem::DebugRenderer& renderer, golem::DebugRenderer* rendererPoint) const {
	// view
	const Contact::View* view = this->views.empty() ? nullptr : this->views.data() + std::min(viewIndex, (U32)this->views.size() - 1);

	// type
	const Configuration::Space* space = this->spaces.empty() ? nullptr : this->spaces.data() + std::min(view ? view->space : 0, (U32)this->spaces.size() - 1);
	if (!space)
		return;

	// path
	const Configuration::Path* path = space->paths.empty() ? nullptr : space->paths.data() + std::min(pathIndex, (U32)space->paths.size() - 1);
	if (!path)
		return;

	// manipulator
	const Manipulator& manipulator = configuration.getManipulator();

	// path
	if (appearance.pathShow) {
		path->draw(manipulator, appearance.path, renderer);
		// TODO View manifold
		appearance.manifold.draw(view->manifold, appearance.path.subspaceFrame.toMat34(), renderer);
	}

	// joint and base frames
	const golem::Mat34 gripFrame = path->getGrip().frame.toMat34();
	golem::WorkspaceJointCoord joints;
	manipulator.getJointFrames(path->getGrip().config, gripFrame, joints);

	// joint links
	for (golem::Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		const Manipulator::Link jointLink(Manipulator::Link::TYPE_JOINT, (golem::U32)*i);
		// find (first) model
		const Contact::View::ModelPtr::Map::const_iterator jointModel = view->models.find(jointLink);
		// TODO select model per link
		if (jointModel != view->models.end()) {
			// find contacts
			const Contact3D::Data::Map::const_iterator jointContacts = this->contacts.find(jointModel->second.index);
			// find appearance
			const Contact3D::Appearance::Map::const_iterator jointAppearance = jointLink.get<Contact3D::Appearance::Map::const_iterator>(appearance.contacts);
			if (jointContacts != this->contacts.end() && jointAppearance != appearance.contacts.end()) {
				jointAppearance->second.relation = appearance.relation; // overwrite
				Contact3D::draw(jointAppearance->second, jointContacts->second.model, joints[i] * jointModel->second.frame, renderer);
			}
		}
	}

	// base link
	const Manipulator::Link baseLink(Manipulator::Link::TYPE_BASE);
	// find (first) model
	// TODO select model per link
	const Contact::View::ModelPtr::Map::const_iterator baseModel = view->models.find(baseLink);
	if (baseModel != view->models.end()) {
		// find contacts
		const Contact3D::Data::Map::const_iterator baseContacts = this->contacts.find(baseModel->second.index);
		// find appearance
		const Contact3D::Appearance::Map::const_iterator baseAppearance = baseLink.get<Contact3D::Appearance::Map::const_iterator>(appearance.contacts);
		if (baseContacts != this->contacts.end() && baseAppearance != appearance.contacts.end()) {
			baseAppearance->second.relation = appearance.relation; // overwrite
			Contact3D::draw(baseAppearance->second, baseContacts->second.model, gripFrame * baseModel->second.frame, renderer);
		}
	}

	// points
	if (appearance.pointShow && view)
		appearance.point.draw(view->points, rendererPoint ? *rendererPoint : renderer);
}

//------------------------------------------------------------------------------

const std::string golem::data::ContactQuery::Data::Header::ID = "golem::data::ContactQuery::Data";
const golem::Header::Version golem::data::ContactQuery::Data::Header::VERSION = golem::Header::Version({ (1 << 0) /* major */ | (0 << 16) /* minor */ });

//------------------------------------------------------------------------------

void golem::data::ContactQuery::Data::Appearance::load(const golem::XMLContext* xmlcontext) {
	config.load(xmlcontext);
	point.load(xmlcontext->getContextFirst("point"));
}

//------------------------------------------------------------------------------

void golem::data::ContactQuery::Data::clear() {
	configs.clear();
	clusters.clear();

	points.clear();

	pointsIndices.clear();
	pointsClusters.clear();
	pointsSelection.clear();
}

//------------------------------------------------------------------------------

template <> void golem::Stream::read(golem::data::ContactModel::Data::Map::value_type& value) const {
	read(const_cast<std::string&>(value.first));

	value.second.header.load(*this);
	
	read(value.second.contacts, value.second.contacts.begin());
	read(value.second.views, value.second.views.begin());
	read(value.second.spaces, value.second.spaces.begin());
	read(value.second.selectorMap, value.second.selectorMap.begin());
	read(value.second.type);
}
template <> void golem::Stream::write(const golem::data::ContactModel::Data::Map::value_type& value) {
	write(value.first);

	value.second.header.store(*this);

	write(value.second.contacts.begin(), value.second.contacts.end());
	write(value.second.views.begin(), value.second.views.end());
	write(value.second.spaces.begin(), value.second.spaces.end());
	write(value.second.selectorMap.begin(), value.second.selectorMap.end());
	write(value.second.type);
}

template <> void golem::Stream::read(golem::data::ContactQuery::Data& value) const {
	value.header.load(*this);

	read(value.configs, value.configs.end());
	read(value.clusters, value.clusters.end());
	
	read(value.points, value.points.end());
	
	read(value.pointsIndices, value.pointsIndices.end());
	read(value.pointsClusters, value.pointsClusters.end());
	read(value.pointsSelection, value.pointsSelection.end());
}
template <> void golem::Stream::write(const golem::data::ContactQuery::Data& value) {
	value.header.store(*this);

	write(value.configs.begin(), value.configs.end());
	write(value.clusters.begin(), value.clusters.end());

	write(value.points.begin(), value.points.end());
	
	write(value.pointsIndices.begin(), value.pointsIndices.end());
	write(value.pointsClusters.begin(), value.pointsClusters.end());
	write(value.pointsSelection.begin(), value.pointsSelection.end());
}

//------------------------------------------------------------------------------