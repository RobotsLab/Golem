/** @file Contact.cpp
*
* @author	Marek Kopicki
*
*/

#include "GolemInteropContactInterface.h"
#include "GolemInteropGolemDefs.h"
#include <Golem/Interop/GolemContact/GolemContact.h>
#include <Golem/Data/Image/Image.h>
#include <Golem/Data/Feature3D/Feature3D.h>
#include <Golem/App/Data.h>
#include <exception>
#include <memory>

using namespace golem;
using namespace golem::interop;

//------------------------------------------------------------------------------

std::shared_ptr<Interface> pInterface;

GOLEM_INTEROP_LIBRARY_DECLDIR void* InterfaceLoader(const char* param) {
	if (!pInterface)
		pInterface.reset(new GolemContact::Desc(std::string(param)));
	return pInterface.get();
}

//-----------------------------------------------------------------------------

golem::interop::GolemContact::Desc::Desc(const std::string& param) : pInterface(nullptr) {
	setToDefault();
	
	// Creates GUI and calls run()

	std::stringstream sstream(param + " "); // HACK std::stringstream::read(&c, 1) reads one character too much without reporting eof
	IStream istream(sstream, " ");
	StringSeq paramSeq;
	std::vector<char*> paramPtrSeq;
	paramPtrSeq.push_back(const_cast<char*>("GolemContact"));
	while (!istream.eos()) {
		paramSeq.push_back(std::string(istream.next<const char*>()));
		paramPtrSeq.push_back(const_cast<char*>(paramSeq.back().data()));
	}
	if (main((int)paramPtrSeq.size(), paramPtrSeq.data()) != 0)
		throw golem::Message(golem::Message::LEVEL_CRIT, "GolemContact::Desc(): Unable to create GolemContact");
}

void golem::interop::GolemContact::Desc::run(int argc, char *argv[]) {
	// Setup application
	load(*context(), xmlcontext());
	// create Tiny interface
	pInterface = golem::is<GolemContact>(scene()->createObject(*this)); // throws
	golem::Assert::valid(pInterface != nullptr, "GolemContact::run(): Unable to cast to GolemContact");
	// Random number generator seed
	context()->info("Random number generator seed %d\n", context()->getRandSeed()._U32[0]);
	// load definitions only, do not block
	pInterface->main(false);
}

void golem::interop::GolemContact::Desc::release() {
	//if (pInterface)
	//	scene()->releaseObject(*pInterface);
	universe()->exit();
}

//-----------------------------------------------------------------------------

void golem::interop::GolemContact::create(const Desc& desc) {
	Player::create(desc);

	contactDesc = desc.contactDesc;

	golem::data::Handler::Map::const_iterator imageHandlerPtr = handlerMap.find(contactDesc.imageHandler);
	imageHandler = imageHandlerPtr != handlerMap.end() ? imageHandlerPtr->second.get() : nullptr;
	golem::Assert::valid(imageHandler != nullptr, "GolemContact::create(): unknown image handler: %s", contactDesc.imageHandler.c_str());

	golem::data::Handler::Map::const_iterator processHandlerPtr = handlerMap.find(contactDesc.processHandler);
	processHandler = processHandlerPtr != handlerMap.end() ? processHandlerPtr->second.get() : nullptr;
	golem::Assert::valid(processHandler != nullptr, "GolemContact::create(): unknown process handler: %s", contactDesc.processHandler.c_str());

	golem::data::Handler::Map::const_iterator modelHandlerPtr = handlerMap.find(contactDesc.modelHandler);
	modelHandler = modelHandlerPtr != handlerMap.end() ? modelHandlerPtr->second.get() : nullptr;
	golem::Assert::valid(modelHandler != nullptr, "GolemContact::create(): unknown contact model handler: %s", contactDesc.modelHandler.c_str());

	golem::data::Handler::Map::const_iterator queryHandlerPtr = handlerMap.find(contactDesc.queryHandler);
	queryHandler = queryHandlerPtr != handlerMap.end() ? queryHandlerPtr->second.get() : nullptr;
	golem::Assert::valid(queryHandler != nullptr, "GolemContact::create(): unknown contact query handler: %s", contactDesc.queryHandler.c_str());

	golem::data::Handler::Map::const_iterator trjHandlerPtr = handlerMap.find(contactDesc.trjHandler);
	trjHandler = trjHandlerPtr != handlerMap.end() ? trjHandlerPtr->second.get() : nullptr;
	golem::Assert::valid(trjHandler != nullptr, "GolemContact::create(): unknown trajectory handler: %s", contactDesc.trjHandler.c_str());
}

golem::interop::GolemContact::GolemContact(golem::Scene &scene) : Player(scene), imageHandler(nullptr), processHandler(nullptr), modelHandler(nullptr), trjHandler(nullptr) {
}

golem::data::Item::Map::iterator golem::interop::GolemContact::addItem(const std::string& name, golem::data::Item::Ptr item, bool erase) {
	RenderBlock renderBlock(*this);
	golem::CriticalSectionWrapper cswData(scene.getCS());
	if (erase) golem::to<Data>(dataCurrentPtr)->itemMap.erase(name);
	golem::data::Item::Map::iterator ptr = golem::to<Data>(dataCurrentPtr)->itemMap.insert(golem::to<Data>(dataCurrentPtr)->itemMap.end(), golem::data::Item::Map::value_type(name, item));
	Data::View::setItem(golem::to<Data>(dataCurrentPtr)->itemMap, ptr, golem::to<Data>(dataCurrentPtr)->getView());
	return ptr;
}

//-----------------------------------------------------------------------------

// SensorCloud
void golem::interop::GolemContact::capture(Point3DCloud& cloud) {
	golem::CriticalSectionWrapper csw(cs);

	golem::Sensor::Map::const_iterator cameraPtr = sensorMap.find(contactDesc.camera);
	golem::CameraDepth* camera = cameraPtr != sensorMap.end() ? golem::is<golem::CameraDepth>(cameraPtr->second.get()) : nullptr;
	golem::Assert::valid(camera != nullptr, "GolemContact::capture(): unknown depth camera: %s", contactDesc.camera.c_str());

	// capture image
	golem::data::Capture* capture = golem::is<golem::data::Capture>(imageHandler);
	golem::Assert::valid(capture != nullptr, "GolemContact::capture(): Handler %s does not support Capture interface", imageHandler->getID().c_str());

	// capture image
	golem::data::Item::Ptr item = capture->capture(*camera, [&](const golem::TimeStamp*) -> bool { return true; });
	// add item
	(void)addItem(contactDesc.imageItem, item, true);
	// convert
	const golem::data::ItemImage* image = golem::is<const golem::data::ItemImage>(item.get());
	golem::Assert::valid(image != nullptr, "GolemContact::capture(): Unknown data item type");
	convert(*image->cloud, cloud);
}

// Controller
void golem::interop::GolemContact::lookupState(double t, Config& state) const {
	golem::Controller::State s = controller->createState();
	controller->lookupState(t, s);
	convert(s, state);
}
// Controller
void golem::interop::GolemContact::sendCommand(const Config* command, std::uintptr_t size)  {
	golem::Controller::State::Seq trajectory(size, controller->createState());
	for (golem::Controller::State::Seq::iterator i = trajectory.begin(); i != trajectory.end(); ++i)
		controller->setToDefault(*i);
	copy(command, command + size, trajectory.data());
	(void)controller->send(trajectory.data(), trajectory.data() + trajectory.size(), true);
}
// Controller
bool golem::interop::GolemContact::waitForTrajectoryEnd(double timewait) {
	return controller->waitForEnd(golem::SecToMSec(timewait));
}
// Controller
bool golem::interop::GolemContact::waitForCycleBegin(double timewait) {
	return controller->waitForBegin(golem::SecToMSec(timewait));
}
// Controller
double golem::interop::GolemContact::cycleDuration() const {
	return controller->getCycleDuration();
}
// Controller
double golem::interop::GolemContact::time() const {
	return context.getTimer().elapsed();
}


// Planner
void golem::interop::GolemContact::findTarget(const ConfigspaceCoord &cbegin, const WorkspaceCoord& wend, ConfigspaceCoord &cend, WorkspaceDist& werr) {
	golem::CriticalSectionWrapper csw(cs);
	// Setup workspace target
	golem::GenWorkspaceChainState gwend;
	gwend.setToDefault(info.getChains().begin(), info.getChains().end()); // all used chains
	convert(wend, gwend.wpos);
	// find target
	golem::GenConfigspaceState gbegin;
	gbegin.setToDefault(info.getJoints().begin(), info.getJoints().end()); // all used joints
	convert(cbegin, gbegin.cpos);
	golem::GenConfigspaceState gcend;
	if (!getPlanner().planner->findTarget(gbegin, gwend, gcend))
		throw golem::Message(golem::Message::LEVEL_ERROR, "GolemContact::findTarget(): unable to find target");
	// convert
	convert(gcend.cpos, cend);
	// compute error
	golem::WorkspaceChainCoord wcc;
	controller->chainForwardTransform(gcend.cpos, wcc);
	wcc[getPlanner().armInfo.getChains().begin()].multiply(wcc[getPlanner().armInfo.getChains().begin()], controller->getChains()[getPlanner().armInfo.getChains().begin()]->getReferencePose());
	const golem::RBDist dist(golem::RBCoord(gwend.wpos[getPlanner().armInfo.getChains().begin()]), golem::RBCoord(wcc[getPlanner().armInfo.getChains().begin()]));
	werr.clear();
	convert(dist, werr[*getPlanner().armInfo.getChains().begin()]);
}
// Planner
void golem::interop::GolemContact::findTrajectory(const ConfigspaceCoord &cbegin, const ConfigspaceCoord &cend, Config::Seq &ctrajectory) {
	//golem::ConfigspaceCoord cc;
	//convert(cend, cc);
	//golem::WorkspaceChainCoord wcc;
	//controller->chainForwardTransform(cc, wcc);
	//const golem::Mat34 frame = wcc[armInfo.getChains().begin()] * controller->getChains()[armInfo.getChains().begin()]->getReferencePose();
	//printf("{(%f, %f, %f)}, {(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n",
	//	frame.p.x, frame.p.y, frame.p.z,
	//	frame.R.m11, frame.R.m12, frame.R.m13, frame.R.m21, frame.R.m22, frame.R.m23, frame.R.m31, frame.R.m32, frame.R.m33
	//);
	golem::CriticalSectionWrapper csw(cs);
	// All bounds are treated as obstacles
	uiPlanner->setCollisionBoundsGroup(golem::Bounds::GROUP_ALL);
	// Find collision-free trajectory
	for (golem::U32 i = 0; i < getPlanner().trajectoryTrials; ++i) {
		// begin
		golem::Controller::State begin = controller->createState();
		controller->setToDefault(begin);
		convert(cbegin, begin.cpos);
		// end
		golem::Controller::State end = controller->createState();
		controller->setToDefault(end);
		convert(cend, end.cpos);
		// try to find
		end.t = begin.t + getPlanner().trajectoryDuration;
		golem::Controller::State::Seq trajectory;
		if (getPlanner().planner->findGlobalTrajectory(begin, end, trajectory, trajectory.begin())) {
			ctrajectory.resize(trajectory.size());
			golem::interop::copy(trajectory.begin(), trajectory.end(), ctrajectory.begin());
			return;
		}
	}
	throw golem::Message(golem::Message::LEVEL_ERROR, "GolemContact::findTrajectory(): unable to find trajectory");
}

// Contact
void golem::interop::GolemContact::findFeatures(const Point3DCloud::Seq& inp, Feature3D::Seq& out) {
	golem::CriticalSectionWrapper csw(cs);

	golem::data::Item::List list;
	for (Point3DCloud::Seq::const_iterator cloud = inp.begin(); cloud != inp.end(); ++cloud) {
		// create empty image
		golem::data::Item::Ptr item = imageHandler->create();
		const golem::data::ItemImage* image = golem::is<const golem::data::ItemImage>(item.get());
		golem::Assert::valid(image != nullptr, "GolemContact::findFeatures(): Unknown data item type");
		// convert
		convert(*cloud, *image->cloud);
		image->cloudFile.setModified(true);
		// add item and insert pointer to the processing list
		list.push_back(addItem(contactDesc.imageItem, item, cloud == inp.begin()));
	}

	// process images
	golem::data::Transform* processTransform = golem::is<golem::data::Transform>(processHandler);
	golem::Assert::valid(processTransform != nullptr, "GolemContact::findFeatures(): Handler %s does not support Transform interface", processHandler->getID().c_str());

	golem::data::Item::Ptr processItem = processTransform->transform(list);
	// add item
	(void)addItem(contactDesc.processItem, processItem, true);
	// convert
	const golem::data::ItemFeature3D* feature3D = golem::is<const golem::data::ItemFeature3D>(processItem.get());
	golem::Assert::valid(feature3D != nullptr, "GolemContact::findFeatures(): Unknown data process type");
	convert(*feature3D->cloud, out);
}

// Contact
void golem::interop::GolemContact::findModel(const Training3D::Map& training, Model3D::Map& models) {
	golem::CriticalSectionWrapper csw(cs);

	// convert training data
	golem::data::Item::List list;
	for (Training3D::Map::const_iterator contact = training.begin(); contact != training.end(); ++contact) {
		// type
		const std::string type = contact->first;
		// create empty cloud
		golem::data::Item::Ptr processItem = processHandler->create();
		golem::data::ItemFeature3D* feature3D = golem::is<golem::data::ItemFeature3D>(processItem.get());
		golem::Assert::valid(feature3D != nullptr, "Contact::findModel(): Unknown data process type");
		// convert
		convert(contact->second.features, *feature3D->cloud);
		feature3D->cloudFile.setModified(true);
		feature3D->process();
		// add item and insert pointer to the processing list
		list.push_back(addItem(type, processItem, true));
		// create empty trajectory
		golem::data::Item::Ptr itemTrajectory = trjHandler->create();
		golem::data::Trajectory* trajectory = golem::is<golem::data::Trajectory>(itemTrajectory.get());
		golem::Assert::valid(trajectory != nullptr, "Contact::findModel(): Trajectory item %s does not support Trajectory interface", trjHandler->getID().c_str());
		// convert
		golem::Controller::State::Seq waypoints;
		convert(*controller, contact->second.trajectory, waypoints);
		trajectory->setWaypoints(golem::WaypointCtrl::make(waypoints, waypoints));
		// add item and insert pointer to the processing list
		list.push_back(addItem(type, itemTrajectory, false));
	}

	// process training data
	golem::data::Transform* modelTransform = golem::is<golem::data::Transform>(modelHandler);
	golem::Assert::valid(modelTransform != nullptr, "Contact::findModel(): Handler %s does not support Transform interface", modelHandler->getID().c_str());

	golem::data::Item::Ptr modelItem = modelTransform->transform(list);
	// convert
	const golem::data::ContactModel* contactModel = golem::is<const golem::data::ContactModel>(modelItem.get());
	golem::Assert::valid(contactModel != nullptr, "Contact::findModel(): ContactModel item %s does not support ContactModel interafce", modelHandler->getID().c_str());
	models.clear();
	for (golem::data::ContactModel::Data::Map::const_iterator model = contactModel->getData().begin(); model != contactModel->getData().end(); ++model) {
		Model3D model3D;
		convert(model->second, model3D);
		models.insert(std::make_pair(model->first, model3D));
	}
	// add item 
	(void)addItem(contactDesc.modelItem, modelItem, true);
}
// Contact
void golem::interop::GolemContact::findQuery(const Model3D::Map& models, const Feature3D::Seq& features, Query& query) {
	golem::CriticalSectionWrapper csw(cs);
	golem::data::Item::List list;

	// contact model
	golem::data::Item::Ptr modelItem = modelHandler->create();
	golem::data::ContactModel* contactModel = golem::is<golem::data::ContactModel>(modelItem.get());
	golem::Assert::valid(contactModel != nullptr, "Contact::findQuery(): ContactModel item %s does not support ContactModel interafce", modelHandler->getID().c_str());
	// convert
	golem::data::ContactModel::Data::Map dataMap;
	for (Model3D::Map::const_iterator model = models.begin(); model != models.end(); ++model) {
		golem::data::ContactModel::Data data;
		convert(model->second, data);
		dataMap.insert(std::make_pair(model->first, data));
	}
	contactModel->setData(dataMap);
	// add item 
	list.push_back(addItem(contactDesc.modelItem, modelItem, true));

	// process features
	golem::data::Item::Ptr processItem = processHandler->create();
	golem::data::ItemFeature3D* feature3D = golem::is<golem::data::ItemFeature3D>(processItem.get());
	golem::Assert::valid(feature3D != nullptr, "Contact::findQuery(): Unknown data process type");
	// convert
	convert(features, *feature3D->cloud);
	feature3D->cloudFile.setModified(true);
	feature3D->process();
	// add item and insert pointer to the processing list
	list.push_back(addItem(contactDesc.processItem, processItem, true));

	// query
	golem::data::Transform* queryTransform = golem::is<golem::data::Transform>(queryHandler);
	golem::Assert::valid(queryTransform != nullptr, "Contact::findQuery(): Handler %s does not support Transform interface", queryHandler->getID().c_str());
	golem::data::Item::Ptr queryItem = queryTransform->transform(list);
	// add item 
	(void)addItem(contactDesc.queryItem, queryItem, true);
	// convert
	golem::data::ContactQuery* contactQuery = golem::is<golem::data::ContactQuery>(queryItem.get());
	golem::Assert::valid(contactQuery != nullptr, "Contact::findQuery(): ContactQuery item %s does not support ContactQuery interafce", queryHandler->getID().c_str());
	convert(contactQuery->getData(), query);
}
// Contact
void golem::interop::GolemContact::findQuery(const Feature3D::Seq& features, Query& query) {
	golem::CriticalSectionWrapper csw(cs);
	golem::data::Item::List list;

	// contact model
	list.push_back(golem::to<golem::data::Data>(dataCurrentPtr)->itemMap.find(contactDesc.modelGraspItem));
	golem::Assert::valid(list.back() != golem::to<golem::data::Data>(dataCurrentPtr)->itemMap.end(), "GolemContact::findQuery(): unable to find grasp model item: %s", contactDesc.modelGraspItem.c_str());

	// process features
	golem::data::Item::Ptr processItem = processHandler->create();
	golem::data::ItemFeature3D* feature3D = golem::is<golem::data::ItemFeature3D>(processItem.get());
	golem::Assert::valid(feature3D != nullptr, "Contact::findQuery(): Unknown data process type");
	// convert
	convert(features, *feature3D->cloud);
	feature3D->cloudFile.setModified(true);
	feature3D->process();
	// add item and insert pointer to the processing list
	list.push_back(addItem(contactDesc.processItem, processItem, true));

	// query
	golem::data::Transform* queryTransform = golem::is<golem::data::Transform>(queryHandler);
	golem::Assert::valid(queryTransform != nullptr, "Contact::findQuery(): Handler %s does not support Transform interface", queryHandler->getID().c_str());
	golem::data::Item::Ptr queryItem = queryTransform->transform(list);
	// add item 
	(void)addItem(contactDesc.queryItem, queryItem, true);
	// convert
	golem::data::ContactQuery* contactQuery = golem::is<golem::data::ContactQuery>(queryItem.get());
	golem::Assert::valid(contactQuery != nullptr, "Contact::findQuery(): ContactQuery item %s does not support ContactQuery interafce", queryHandler->getID().c_str());
	convert(contactQuery->getData(), query);
}
// Contact
void golem::interop::GolemContact::selectTrajectory(const Query& query, Trajectory& trajectory) {
	golem::CriticalSectionWrapper csw(cs);

	// query
	golem::data::Item::Ptr queryItem = queryHandler->create();
	golem::data::ContactQuery* contactQuery = golem::is<golem::data::ContactQuery>(queryItem.get());
	golem::Assert::valid(contactQuery != nullptr, "Contact::selectTrajectory(): ContactQuery item %s does not support ContactQuery interafce", queryHandler->getID().c_str());
	// convert
	golem::data::ContactQuery::Data data;
	convert(query, data);
	contactQuery->setData(data);
	// add item 
	(void)addItem(contactDesc.queryItem, queryItem, true);

	// select
	golem::data::Convert* queryConvert = golem::is<golem::data::Convert>(queryItem.get());
	golem::Assert::valid(queryConvert != nullptr, "Contact::selectTrajectory(): Query item does not support Convert interface");
	InputBlock inputBlock(*this);
	golem::data::Item::Ptr trjSelectItem = queryConvert->convert(*trjHandler);
	// add item 
	(void)addItem(contactDesc.trjSelectionItem, trjSelectItem, true);

	// convert
	golem::data::Trajectory* selectTrajectory = golem::is<golem::data::Trajectory>(trjSelectItem.get());
	golem::Assert::valid(selectTrajectory != nullptr, "Contact::selectTrajectory(): Trajectory item %s does not support Trajectory interface", trjHandler->getID().c_str());
	trajectory.type; // TODO
	golem::Controller::State::Seq tmp;
	selectTrajectory->createTrajectory(tmp);
	convert(tmp, trajectory.trajectory);
	trajectory.error.clear(); // TODO
}

// GraspCloud
void golem::interop::GolemContact::findTrajectories(const Point3DCloud::Seq& clouds, Trajectory::Seq& trajectories) {
	golem::CriticalSectionWrapper csw(cs);

	golem::data::Item::List processList, queryList;

	for (Point3DCloud::Seq::const_iterator cloud = clouds.begin(); cloud != clouds.end(); ++cloud) {
		// create empty image
		golem::data::Item::Ptr item = imageHandler->create();
		const golem::data::ItemImage* image = golem::is<const golem::data::ItemImage>(item.get());
		golem::Assert::valid(image != nullptr, "GolemContact::findTrajectories(): Unknown data item type");
		// convert
		convert(*cloud, *image->cloud);
		image->cloudFile.setModified(true);
		// add item and insert pointer to the processing list
		processList.push_back(addItem(contactDesc.imageItem, item, cloud == clouds.begin()));
	}

	// process images
	golem::data::Transform* processTransform = golem::is<golem::data::Transform>(processHandler);
	golem::Assert::valid(processTransform != nullptr, "GolemContact::findTrajectories(): Handler %s does not support Transform interface", processHandler->getID().c_str());
	queryList.push_back(addItem(contactDesc.processItem, processTransform->transform(processList), true));

	// find model
	queryList.push_back(golem::to<golem::data::Data>(dataCurrentPtr)->itemMap.find(contactDesc.modelGraspItem));
	golem::Assert::valid(queryList.back() != golem::to<golem::data::Data>(dataCurrentPtr)->itemMap.end(), "GolemContact::findTrajectories(): unable to find grasp model item: %s", contactDesc.modelGraspItem.c_str());

	// compute grasps
	golem::data::Transform* queryTransform = golem::is<golem::data::Transform>(queryHandler);
	golem::Assert::valid(queryTransform != nullptr, "GolemContact::findTrajectories(): Handler %s does not support Transform interface", queryHandler->getID().c_str());
	golem::data::Item::Ptr queryItem = queryTransform->transform(queryList);
	// add item 
	(void)addItem(contactDesc.queryItem, queryItem, true);

	// TODO export multiple trajectories
	Trajectory trajectory;

	// select single trajectory
	golem::data::Convert* queryConvert = golem::is<golem::data::Convert>(queryItem.get());
	golem::Assert::valid(queryConvert != nullptr, "Contact::findTrajectories(): Query item does not support Convert interface");
	InputBlock inputBlock(*this);
	golem::data::Item::Ptr trjSelectItem = queryConvert->convert(*trjHandler);
	// add item 
	(void)addItem(contactDesc.trjSelectionItem, trjSelectItem, true);

	// convert single trajectory
	golem::data::Trajectory* selectTrajectory = golem::is<golem::data::Trajectory>(trjSelectItem.get());
	golem::Assert::valid(selectTrajectory != nullptr, "Contact::findTrajectories(): Trajectory item %s does not support Trajectory interface", trjHandler->getID().c_str());
	trajectory.type; // TODO
	golem::Controller::State::Seq tmp;
	selectTrajectory->createTrajectory(tmp);
	convert(tmp, trajectory.trajectory);
	trajectory.error.clear(); // TODO
	
	// export
	trajectories.clear();
	trajectories.push_back(trajectory);
}

// Application
bool golem::interop::GolemContact::dispatch() {
	try {
		golem::Player::menu();
		return true;
	}
	catch (const golem::Exit&) {
		return false;
	}
}

void golem::interop::GolemContact::setCallbackTerminate(Application::CallbackTerminate callback) {
	this->getScene().getUniverse().setHandlerTerminate(callback);
}

//-----------------------------------------------------------------------------