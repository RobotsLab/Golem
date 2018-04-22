/** @file Tracking.cpp
 *
 * Demo tracking
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/App/Tracking/Tracking.h>
#include <Golem/Contact/Data.h>
#include <Golem/Planner/GraphPlanner/Data.h>
#include <Golem/Tools/RBPose.h>
#include <Golem/App/Tracking/posetracker.h>
#include <Golem/Plugin/Text.h>

using namespace golem;

//-----------------------------------------------------------------------------

void AppTracking::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Player::Desc::load(context, xmlcontext);

	xmlcontext = xmlcontext->getContextFirst("demo");

	golem::XMLData("bundle", bundle, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("bundle_log", bundleLog, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("camera", camera, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("camera_tracking", cameraTracking, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("handler", imageHandler, xmlcontext->getContextFirst("image"));
	golem::XMLData("item", imageItem, xmlcontext->getContextFirst("image"));

	golem::XMLData("handler", processHandler, xmlcontext->getContextFirst("process"));
	golem::XMLData("item", processItem, xmlcontext->getContextFirst("process"));
	golem::XMLData("break", processBreak, xmlcontext->getContextFirst("process"));

	golem::XMLData("handler", modelHandler, xmlcontext->getContextFirst("model"));
	golem::XMLData("item", modelItem, xmlcontext->getContextFirst("model"));

	golem::XMLData("handler", queryHandler, xmlcontext->getContextFirst("query"));
	golem::XMLData("item", queryItem, xmlcontext->getContextFirst("query"));

	golem::XMLData("handler", logHandler, xmlcontext->getContextFirst("log"));
	golem::XMLData("item", logItem, xmlcontext->getContextFirst("log"));

	golem::XMLData("handler", videoHandler, xmlcontext->getContextFirst("video"));
	golem::XMLData("item", videoItem, xmlcontext->getContextFirst("video"));

	poseScanSeq.clear();
	XMLData(poseScanSeq, poseScanSeq.max_size(), xmlcontext->getContextFirst("scan"), "pose");
	XMLData(scanRegionDesc, scanRegionDesc.max_size(), xmlcontext->getContextFirst("scan region"), "bounds");

	golem::XMLData("threads", trackingThreads, xmlcontext->getContextFirst("tracking"));

	manipulatorDesc->load(xmlcontext->getContextFirst("manipulator"));

	golem::XMLData(profileDesc, xmlcontext->getContextFirst("manipulator profile"));
	golem::XMLDataSeq(distance, "c", xmlcontext->getContextFirst("manipulator profile distance"), false, golem::REAL_ZERO);
	golem::XMLDataSeq(grip, "c", xmlcontext->getContextFirst("manipulator profile grip"), false, golem::REAL_ZERO);

	kinematicsDesc.reset(new golem::DEKinematics::Desc);
	golem::XMLData(*kinematicsDesc, xmlcontext->getContextFirst("manipulator kinematics"));
	golem::XMLData("thread_parallels", kinematicsThreadParallels, xmlcontext->getContextFirst("manipulator kinematics"));
	golem::XMLData("thread_timeout", kinematicsThreadTimeOut, xmlcontext->getContextFirst("manipulator kinematics"));

	golem::XMLData("dist_thr", ctrlDistThr, xmlcontext->getContextFirst("manipulator controller"));

	golem::XMLData("ikin_dt", ctrlIKin, xmlcontext->getContextFirst("manipulator controller"));
	golem::XMLData("reac_dt", ctrlReac, xmlcontext->getContextFirst("manipulator controller"));
	golem::XMLData("pred_dt", ctrlPred, xmlcontext->getContextFirst("manipulator controller"));
	golem::XMLData("exec_dt", ctrlExec, xmlcontext->getContextFirst("manipulator controller"));

	appearanceBounds.load(xmlcontext->getContextFirst("manipulator appearance_bounds"));
	appearanceSolution.load(xmlcontext->getContextFirst("manipulator appearance_solution"));
	appearanceSelection.load(xmlcontext->getContextFirst("manipulator appearance_selection"));

	golem::XMLData("point_size", objectPointSize, xmlcontext->getContextFirst("object"));
	golem::XMLData(objectPointColour, xmlcontext->getContextFirst("object point_colour"));
	golem::XMLData(objectSolidColour, xmlcontext->getContextFirst("object solid_colour"));
	golem::XMLData(objectWireColour, xmlcontext->getContextFirst("object wire_colour"));
	golem::XMLData(objectTrackingColour, xmlcontext->getContextFirst("object tracking_colour"));
}

//------------------------------------------------------------------------------

AppTracking::AppTracking(Scene &scene) : Player(scene), camera(nullptr), cameraTracking(nullptr), imageHandler(nullptr), processHandler(nullptr), modelHandler(nullptr), queryHandler(nullptr), logHandler(nullptr), videoHandler(nullptr) {
}

AppTracking::~AppTracking() {
}

void AppTracking::create(const Desc& desc) {
	desc.assertValid(Assert::Context("AppTracking::Desc."));

	// create object
	Player::create(desc); // throws

	// scanning camera
	golem::Sensor::Map::const_iterator cameraPtr = sensorMap.find(desc.camera);
	camera = cameraPtr != sensorMap.end() ? is<CameraDepth>(cameraPtr->second.get()) : nullptr;
	//Assert::valid(camera != nullptr, "golem::AppTracking::create(): unknown depth camera: %s", desc.camera.c_str());
	
	// tracking camera
	golem::Sensor::Map::const_iterator cameraTrackingPtr = sensorMap.find(desc.cameraTracking);
	cameraTracking = cameraTrackingPtr != sensorMap.end() ? is<CameraDepth>(cameraTrackingPtr->second.get()) : nullptr;
	//Assert::valid(cameraTracking != nullptr, "golem::AppTracking::create(): unknown depth tracking camera: %s", desc.cameraTracking.c_str());

	Assert::valid(camera != nullptr || cameraTracking != nullptr, "golem::AppTracking::create(): no depth cameras available");
	if (!camera)
		camera = cameraTracking;
	if (!cameraTracking)
		context.info("golem::AppTracking::create(): tracking not available");

	bundle = desc.bundle;
	bundleLog = desc.bundleLog;

	golem::data::Handler::Map::const_iterator imageHandlerPtr = handlerMap.find(desc.imageHandler);
	imageHandler = imageHandlerPtr != handlerMap.end() ? imageHandlerPtr->second.get() : nullptr;
	Assert::valid(imageHandler != nullptr, "golem::AppTracking::create(): unknown image handler: %s", desc.imageHandler.c_str());
	imageItem = desc.imageItem;

	golem::data::Handler::Map::const_iterator processHandlerPtr = handlerMap.find(desc.processHandler);
	processHandler = processHandlerPtr != handlerMap.end() ? processHandlerPtr->second.get() : nullptr;
	Assert::valid(processHandler != nullptr, "golem::AppTracking::create(): unknown process handler: %s", desc.processHandler.c_str());
	processItem = desc.processItem;
	processBreak = desc.processBreak;

	golem::data::Handler::Map::const_iterator modelHandlerPtr = handlerMap.find(desc.modelHandler);
	modelHandler = modelHandlerPtr != handlerMap.end() ? modelHandlerPtr->second.get() : nullptr;
	Assert::valid(modelHandler != nullptr, "golem::AppTracking::create(): unknown contact model handler: %s", desc.modelHandler.c_str());
	modelItem = desc.modelItem;

	golem::data::Handler::Map::const_iterator queryHandlerPtr = handlerMap.find(desc.queryHandler);
	queryHandler = queryHandlerPtr != handlerMap.end() ? queryHandlerPtr->second.get() : nullptr;
	Assert::valid(queryHandler != nullptr, "golem::AppTracking::create(): unknown contact query handler: %s", desc.queryHandler.c_str());
	queryItem = desc.queryItem;

	golem::data::Handler::Map::const_iterator trjHandlerPtr = handlerMap.find(desc.logHandler);
	logHandler = trjHandlerPtr != handlerMap.end() ? trjHandlerPtr->second.get() : nullptr;
	Assert::valid(logHandler != nullptr, "golem::AppTracking::create(): unknown trajectory handler: %s", desc.logHandler.c_str());
	logItem = desc.logItem;

	golem::Sensor::Map::const_iterator videoHandlerPtr = sensorMap.find(desc.videoHandler);
	videoHandler = videoHandlerPtr != sensorMap.end() ? is<Camera>(videoHandlerPtr->second.get()) : nullptr;
	Assert::valid(videoHandler != nullptr, "golem::AppTracking::create(): unknown video handler: %s", desc.videoHandler.c_str());
	videoItem = desc.videoItem;

	poseScanSeq = desc.poseScanSeq;
	scanRegion.clear();
	for (golem::Bounds::Desc::Seq::const_iterator i = desc.scanRegionDesc.begin(); i != desc.scanRegionDesc.end(); ++i)
		scanRegion.push_back((*i)->create());

	trackingThreads = desc.trackingThreads;

	// default planner
	manipulator = desc.manipulatorDesc->create(*getPlanner().planner, getPlanner().controllerIDSeq);

	kinematics = golem::dynamic_pointer_cast<DEKinematics::Ptr>(desc.kinematicsDesc->create(getPlanner().planner->getHeuristic()));
	Assert::valid(kinematics != nullptr, "golem::AppTracking::create(): unable to cast to DEKinematics::Ptr");
	parallels.reset(new golem::Parallels(desc.kinematicsThreadParallels, kinematicsThreadTimeOut));
	Assert::valid(parallels != nullptr, "golem::AppTracking::create(): unable to create Parallels");
	kinematicsThreadTimeOut = desc.kinematicsThreadTimeOut;

	profileDesc = desc.profileDesc;
	distance.fill(golem::REAL_ONE);
	distance.set(desc.distance.data(), desc.distance.data() + std::min(desc.distance.size(), (size_t)Configspace::DIM));
	grip.fill(golem::REAL_ONE);
	grip.set(desc.grip.data(), desc.grip.data() + std::min(desc.grip.size(), (size_t)Configspace::DIM));

	ctrlDistThr = desc.ctrlDistThr;

	ctrlIKin = desc.ctrlIKin;
	ctrlReac = desc.ctrlReac;
	ctrlPred = desc.ctrlPred;
	ctrlExec = desc.ctrlExec;

	appearanceBounds = desc.appearanceBounds;
	appearanceSolution = desc.appearanceSolution;
	appearanceSelection = desc.appearanceSelection;

	objectPointSize = desc.objectPointSize;
	objectPointColour = desc.objectPointColour;
	objectSolidColour = desc.objectSolidColour;
	objectWireColour = desc.objectWireColour;
	objectTrackingColour = desc.objectTrackingColour;

	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0F51", "  R                                       menu run\n"));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("R", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: run (T)racking demo...";
	}));
	menuCmdMap.insert(std::make_pair("RT", [=]() {
		// select data bundle
		data::Data::Map::iterator dataPtr = dataMap.find(bundle);
		if (dataPtr == dataMap.end())
			throw Message(Message::LEVEL_ERROR, "unknown data bundle: %s", bundle.c_str());

		// debug mode
		const bool stopAtBreakPoint = option(0, "Debug mode: ", { "YES", "NO" }) == 0;
		const auto breakPoint = [=] (const char* str) {
			if (!stopAtBreakPoint && str)
				context.write("%s....\n", str);
			if ( stopAtBreakPoint && str ? option("YN", makeString("%s: Continue (Y/N)...", str).c_str()) != 'Y' : waitKey(5) == 27)
				throw Cancel("Demo cancelled");
		};

		// clean up demo
		ScopeGuard cleanup([&]() {
			camera->set(Camera::CMD_STOP);
			camera->setProperty(camera->getImageProperty());
			if (cameraTracking) {
				cameraTracking->set(Camera::CMD_STOP);
				cameraTracking->setProperty(cameraTracking->getImageProperty());
			}
			UI::addCallback(*this, getCurrentHandler());
		});

		for (U32 trial = 0;; ++trial) {
			// run!
			(void)option("\x0D", "<Enter> to start new trial...\n", true);

			// clean up demo trial cycle
			ScopeGuard cleanupCycle([&]() {
				golem::CriticalSectionWrapper csw(getCS());
				demoRenderer.reset();
				demoRendererConfigs.reset();
				demoRendererObject.reset();
			});

			// create new log data bundle
			{
				std::string bundleLog = this->bundleLog + std::to_string(trial + 1) + "/data";
				if (!isExt(bundleLog, dataExt))
					bundleLog += dataExt;
				Data::Ptr data = createData();
				RenderBlock renderBlock(*this);
				scene.getOpenGL(to<Data>(dataCurrentPtr)->getView().openGL); // set current view
				scene.getOpenGL(to<Data>(data)->getView().openGL); // set view of the new data
				{
					golem::CriticalSectionWrapper cswData(getCS());
					dataMap.erase(bundleLog);
					dataCurrentPtr = dataMap.insert(dataMap.begin(), Data::Map::value_type(bundleLog, data));
				}
			}

			// capture depth images
			data::Item::List list;
			for (ConfigMat34::Seq::const_iterator j = poseScanSeq.begin(); j != poseScanSeq.end(); ++j) {
				// go to object scan pose
				breakPoint("Going to scan pose");
				gotoPose(*j);

				// capture image
				data::Capture* capture = is<data::Capture>(imageHandler);
				Assert::valid(capture != nullptr, "Handler %s does not support Capture interface", imageHandler->getID().c_str());

				// capture image
				data::Item::Ptr item = capture->capture(*camera, [&](const golem::TimeStamp*) -> bool { return true; });
				{
					RenderBlock renderBlock(*this);
					golem::CriticalSectionWrapper csw(getCS());
					demoRenderer.reset();
					data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(imageItem, item));
					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
					// insert image to the processing list
					list.push_back(ptr);
				}
			}

			// process images
			data::Transform* processTransform = is<data::Transform>(processHandler);
			Assert::valid(processTransform != nullptr, "Handler %s does not support Transform interface", processHandler->getID().c_str());

			data::Item::Ptr processItem = processTransform->transform(list);
			data::Item::Map::iterator processPtr;
			{
				RenderBlock renderBlock(*this);
				golem::CriticalSectionWrapper csw(getCS());
				processPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(this->processItem, processItem));
				Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, processPtr, to<Data>(dataCurrentPtr)->getView());
			}
			if (processBreak) {
				EnableKeyboardMouse enableKeyboardMouse(*this);
				(void)option("\x0D", "Press <7><8> to select cluster, <Space> to add/remove cluster, <Enter> to continue...\n", true);
			}

			// compute grasps
			data::Item::Map::iterator modelPtr = to<Data>(dataPtr)->itemMap.find(modelItem);
			if (modelPtr == to<Data>(dataPtr)->itemMap.end())
				throw Message(Message::LEVEL_ERROR, "unable to find model item: %s", modelItem.c_str());
			list.clear();
			list.push_back(modelPtr);
			list.push_back(processPtr);
			data::Transform* queryTransform = is<data::Transform>(queryHandler);
			Assert::valid(queryTransform != nullptr, "Handler %s does not support Transform interface", queryHandler->getID().c_str());

			data::Item::Ptr queryItem = queryTransform->transform(list);
			data::Item::Map::iterator queryPtr;
			{
				RenderBlock renderBlock(*this);
				golem::CriticalSectionWrapper csw(getCS());
				queryPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(this->queryItem, queryItem));
				Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, queryPtr, to<Data>(dataCurrentPtr)->getView());
			}

			// select planner
			const PlannerInfo& plannerInfo = getPlanner(); // use current planner
			golem::GraphPlanner* const planner = is<golem::GraphPlanner>(plannerInfo.planner);
			Assert::valid(planner != nullptr, "Graph planner required");
			const Configspace::Range armJoints = plannerInfo.armInfo.getJoints();
			const Configspace::Range handJoints = plannerInfo.handInfo.getJoints();
			const Chainspace::Range armChains = plannerInfo.armInfo.getChains();

			// target selection
			const auto getTarget = [](const Configuration::Path& path) -> const Manipulator::Waypoint&{ return path.getApproach(); };
			// workspace distance
			const auto getDist = [](const RBCoord& a, const RBCoord& b) -> RBDist { return RBDist(a, b); };
			const golem::RBDist distCoeffs = manipulator->getDesc().trajectoryErr;

			// select grasps with SE(3) approach trajectory
			data::ContactQuery* contactQuery = is<data::ContactQuery>(queryItem.get());
			Assert::valid(contactQuery != nullptr, "ContactQuery item %s does not support ContactQuery interafce", queryHandler->getID().c_str());
			Grasp::Map graspMap;
			{
				EnableKeyboardMouse enableKeyboardMouse(*this);
				ScopeGuard configSetCleanup([&]() {
					golem::CriticalSectionWrapper csw(getCS());
					demoRendererConfigs.reset();
				});
				const auto printConfigSet = [&]() {
					std::stringstream str;
					for (Grasp::Map::const_iterator i = graspMap.begin(); i != graspMap.end(); ++i)
						str << "(" << std::distance(contactQuery->getData().configs.begin(), i->first) + 1 << ")";
					context.write("Grasp selection (%u): {%s}\n", (U32)graspMap.size(), str.str().c_str());
				};
				const auto renderConfigSet = [&](const Contact::Config* config) {
					Grasp::Map::const_iterator wneighbour = graspMap.end(), cneighbour = graspMap.end();
					Real cdist = REAL_MAX, wdist = REAL_MAX;
					for (Grasp::Map::const_iterator i = graspMap.begin(); config != nullptr && i != graspMap.end(); ++i)
						if (config != i->first->get()) {
							const Real wdistTest = distCoeffs.dot(getDist(config->path.getGrip().frame, (*i->first)->path.getGrip().frame));
							if (wdist > wdistTest) {
								wdist = wdistTest;
								wneighbour = i;
							}
							Real cdistTest = REAL_ZERO;
							for (Configspace::Index j = armJoints.begin(); j < armJoints.end(); ++j)
								cdistTest += Math::sqr(config->path.getGrip().config[j] - (*i->first)->path.getGrip().config[j]);
							if (cdist > cdistTest) {
								cdist = cdistTest;
								cneighbour = i;
							}
						}
						else {
							wneighbour = cneighbour = graspMap.end();
							break;
						}

						if (wneighbour != graspMap.end() && cneighbour != graspMap.end())
							context.write("Distance_{lin=%f, ang=%f}\n", wdist, cdist);

						golem::CriticalSectionWrapper csw(getCS());
						demoRendererConfigs.reset();
						for (Grasp::Map::const_iterator i = graspMap.begin(); i != graspMap.end(); ++i)
							if (wneighbour != i && cneighbour != i)
								appearanceBounds.draw(manipulator->getBounds((*i->first)->path.getGrip().config, (*i->first)->path.getGrip().frame.toMat34()), demoRendererConfigs);
						if (wneighbour != graspMap.end())
							appearanceSelection.draw(manipulator->getBounds((*wneighbour->first)->path.getGrip().config, (*wneighbour->first)->path.getGrip().frame.toMat34()), demoRendererConfigs);
						if (cneighbour != graspMap.end())
							appearanceSolution.draw(manipulator->getBounds((*cneighbour->first)->path.getGrip().config, (*cneighbour->first)->path.getGrip().frame.toMat34()), demoRendererConfigs);
				};

				context.write("Press <(><)> to select, <Space> to add/remove, <BkSpace> to clear, <Enter> to continue...\n");

				for (bool done = false; !done;)
					switch (waitKey()) {
					case 32: { // <Space>
						const Contact::Config::Seq::const_iterator config = contactQuery->getConfig();
						if (graspMap.find(config) == graspMap.end()) graspMap.insert(std::make_pair(config, Grasp())); else graspMap.erase(config);
						printConfigSet();
						renderConfigSet(nullptr);
						break;
					}
					case 8: // <BkSpace>
						graspMap.clear();
						printConfigSet();
						renderConfigSet(nullptr);
						break;
					case 13: // <Enter>
						if (graspMap.empty()) context.write("Select at least one grasp\n"); else done = true;
						break;
					case '(':case ')':
						renderConfigSet(contactQuery->getConfig()->get());
						break;
					case 27: // Cancel
						throw Cancel("Cancelled");
				}
			}

			// Object representations
			const data::Point3D* processPoint3D = is<data::Point3D>(processPtr);
			Assert::valid(processPoint3D != nullptr, "Item %s does not support Point3D interface", processPtr->first.c_str());

			// Object bounding box
			Vec3Seq objectPoints; // cluster points
			const golem::Bounds::Seq objectBounds = CollisionBounds::getBounds([&](size_t i, Vec3& p) -> bool { if (i < processPoint3D->getSize()) objectPoints.push_back(p = processPoint3D->getPoint(i)); return i < processPoint3D->getSize(); }, is<const data::Cluster3D>(processPoint3D), golem::data::Cluster3D::CLUSTER_PROCESSING);
			golem::BoundingBox* const objectBoundingBox = is<golem::BoundingBox>(objectBounds.empty() ? nullptr : objectBounds.front().get()); // cluster bounding box
			Assert::valid(objectBoundingBox != nullptr, "Point cloud bounding box required");
			golem::Mat34 objectBoundingBoxPose = objectBoundingBox->getPose(), objectBoundingBoxPoseInv; // inertial frame
			objectBoundingBoxPoseInv.setInverse(objectBoundingBoxPose);

			// initialise camera (if available)
			Mat34 cameraFrame(Mat34::identity());
			if (cameraTracking) {
				cameraTracking->setProperty(cameraTracking->getDepthProperty());
				if (!cameraTracking->set(Camera::CMD_VIDEO))
					throw Message(Message::LEVEL_ERROR, "unable to start tracking camera");
				cameraFrame = cameraTracking->getFrame();
			}
			// initialise tracker with segmented point cloud (if available)
			::posetracker tracker;
			::posetracker::CloudPtr trackerCloudTarget(new ::posetracker::Cloud), trackerCloud(new ::posetracker::Cloud);
			if (cameraTracking) {
				tracker.initTracking(trackingThreads);
				// point cloud
				for (Vec3Seq::const_iterator i = objectPoints.begin(); i != objectPoints.end(); ++i) {
					::posetracker::Point point;
					Cloud::convertPointXYZ(*i, point);
					trackerCloudTarget->push_back(point);
				}
				tracker.setTargetCloud(trackerCloudTarget, Eigen::Affine3f(Cloud::toEigen(objectBoundingBoxPose)));
			}
			// object tracking variables
			Cloud::PointSeq cloud;
			Mat34 objectFrame = Mat34::identity(), bodyFrame = Mat34::identity();
			RBAdjust adjust;

			// planner initialisation
			const golem::Waypoint::Seq::const_iterator graph = planner->getGlobalPathFinder()->getGraph().begin() + planner->getGlobalPathFinder()->getGraphPartition();
			const size_t graphSize = planner->getGlobalPathFinder()->getOfflineGraphSize();
			for (Grasp::Map::iterator i = graspMap.begin(); i != graspMap.end(); ++i) {
				i->second.targetPath = i->first->get()->path;
				i->second.solutions.resize(graphSize);
			}
			const size_t populationSize = std::min(graphSize, kinematics->getDesc().populationSize);
			ConfigspaceCoord::Seq population;
			population.reserve(populationSize);

			// Begin tracking
			(void)option("\x0D", "<Enter> to begin tracking, <Space> to (un)follow, <BkSpace> to grasp...\n", true);
			bool commandFollow = false, commandGrasp = false;

			// log stream special characters
			const char logSep = '\t', logEnd = '\n';

			// insert SO(3) transform
			auto logTrn = [=](const golem::Mat34& trn, std::iostream& ios) {
				// Euler
				Real roll, pitch, yaw;
				trn.R.toEuler(roll, pitch, yaw);
				ios << trn.p.x << logSep << trn.p.y << logSep << trn.p.z << logSep << roll << logSep << pitch << logSep << yaw << logSep;
			};
			auto logTrnHead = [=](const std::string& name, std::iostream& ios) {
				// Euler
				ios << name << "_x" << logSep << name << "_y" << logSep << name << "_z" << logSep << name << "_roll" << logSep << name << "_pitch" << logSep << name << "_yaw" << logSep;
			};

			// create log item
			data::Text* log = nullptr;
			{
				RenderBlock renderBlock(*this);
				golem::CriticalSectionWrapper csw(getCS());
				data::Item::Map::iterator logPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(this->logItem, logHandler->create()));
				log = is<data::Text>(logPtr);
				Assert::valid(log != nullptr, "Handler %s does create items with Text interface", logHandler->getID().c_str());
				Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, logPtr, to<Data>(dataCurrentPtr)->getView());
			}
			// create header
			log->getTextStream() << "trial" << logSep << "step" << logSep << "time" << logSep << "type" << logSep << "curr_index" << logSep << "curr_dist" << logSep << "alt_index" << logSep << "alt_dist" << logSep;
			logTrnHead("object", log->getTextStream());
			logTrnHead("effector", log->getTextStream());
			log->getTextStream() << "n_index" << logSep << "n_dist" << logSep << "n_dist_config" << logSep << "n_dist_lin" << logSep << "n_dist_ang" << logEnd;

			// start recording
			if (setCamera(videoHandler, true)) {
				recordingStart(dataCurrentPtr->first, videoItem, true);
				recordingWaitToStart();
			}
			ScopeGuard recordingGuard([&]() {
				// stop recording
				recordingStop(getPlanner().trajectoryIdlePerf);
				recordingWaitToStop();
				setCamera(videoHandler, false);
			});

			// continuously update
			Grasp::Map::iterator current = graspMap.end();
			Real currentDist = REAL_ZERO;
			Controller::Trajectory trj;
			GenWorkspaceChainState::Seq wtrj;
			const Mat34 referencePose = controller->getChains()[armChains.begin()]->getReferencePose();
			CriticalSection parallelsCS;
			for (U32 step = 0; !commandGrasp;) {
				{
					// Update dependent variables
					golem::CriticalSectionWrapper csw(getCS());
					// points
					demoRendererObject.reset();
					demoRendererObject.setPointSize(objectPointSize);
					for (Vec3Seq::const_iterator i = objectPoints.begin(); i != objectPoints.end(); ++i)
						demoRendererObject.addPoint(objectFrame * *i, objectPointColour);
					// bounding box
					objectBoundingBox->setPose(objectFrame * objectBoundingBoxPose);
					demoRendererObject.setColour(objectSolidColour);
					demoRendererObject.addSolid(*objectBoundingBox);
					demoRendererObject.setColour(objectWireColour);
					demoRendererObject.addWire(*objectBoundingBox);
					// display cloud
					for (Cloud::PointSeq::const_iterator i = cloud.begin(); i != cloud.end(); ++i)
						if (!Cloud::isNanXYZ(*i))
							demoRendererObject.addPoint(Cloud::getPoint<golem::Real>(*i), objectTrackingColour);
				}

				// tracking (if available)
				if (cameraTracking) {
					// retrieve latest frame
					bool update = false;
					cameraTracking->waitAndPeek([&](const Image::List& images, bool result) {
						if (result && !images.empty()) {
							cloud = *images.back()->cloud;
							update = true;
						}
					});
					if (update) {
						// transform points
						Cloud::transform(cameraFrame, cloud, cloud);
						// region clipping
						if (!scanRegion.empty())
							Cloud::regionClip(scanRegion, cloud, Cloud::isNanXYZ<Cloud::Point>);
						// convert point cloud
						trackerCloud->clear();
						for (Cloud::PointSeq::const_iterator i = cloud.begin(); i != cloud.end(); ++i)
							if (!Cloud::isNanXYZ(*i)) {
								::posetracker::Point point;
								Cloud::copyPointXYZ(*i, point);
								Cloud::copyRGBA(*i, point);
								trackerCloud->push_back(point);
							}
						// Estimate current frame
						const Mat34 trackerFrame = Cloud::fromEigen(Eigen::Matrix4f(tracker.runtracker(trackerCloud).matrix()));
						// update frame
						objectFrame = trackerFrame * objectBoundingBoxPoseInv;
					}
				}

				// wait for key press
				const int key = waitKey(1);
				switch (key) {
				case 32: commandFollow = !commandFollow; context.write("Follow object: %s\n", commandFollow ? "YES" : "NO"); break;
				case 27: throw Cancel("Demo cancelled");
				case 8: commandGrasp = true; context.write("Grasping object\n"); break;
				default:
					// adjust increment: "-+"
					(void)adjust.adjustIncrement(key);
					// adjust objectFrame: linKeys="wsedrf", angKeys="WSEDRF"
					//adjust.adjustFrame(key, objectFrame); // inertial frame
					if (adjust.adjustFrame(key, bodyFrame)) // body frame
						objectFrame = objectBoundingBoxPose * bodyFrame * objectBoundingBoxPoseInv;
				}

				// convert latest object frame
				const RBCoord frame(objectFrame);

				// current time and state
				const SecTmReal t = context.getTimer().elapsed();
				golem::Controller::State cbegin = controller->createState();
				controller->lookupCommand(t + ctrlReac, cbegin); // in future

				// update waypoints frames of approach trajectories of all grasps
				for (Grasp::Map::iterator i = graspMap.begin(); i != graspMap.end(); ++i) {
					const size_t size = std::min(i->first->get()->path.size(), i->second.targetPath.size());
					for (size_t j = 0; j < size; ++j)
						i->second.targetPath[j].frame = frame * i->first->get()->path[j].frame;
					i->second.targetFrame = getTarget(i->second.targetPath).frame;
				}

				// compute approximate inverse kinematics solutions for approach frames
				Grasp::Map::iterator graspPtr = graspMap.begin();
				ParallelsTask(context.getParallels(), [&](ParallelsTask*) {
					for (Grasp::Map::iterator grasp;;) {
						{
							CriticalSectionWrapper csw(parallelsCS);
							if (graspPtr == graspMap.end()) {
								return;
							}
							grasp = graspPtr++;
						}
						// compute distance for all graph waypoints
						for (size_t i = 0; i < graphSize; ++i) {
							const golem::Waypoint::Seq::const_iterator waypoint = graph + i;
							Solution& solution = grasp->second.solutions[i];
							// set waypoint
							solution.waypoint = waypoint;
							// workspace distance
							solution.wdist = getDist(grasp->second.targetFrame, RBCoord(waypoint->wpos[armChains.begin()].p, waypoint->qrot[armChains.begin()]));
							// weighted distance
							solution.dist = distCoeffs.dot(solution.wdist);
						}
						// sort waypoints from the lowest to the highest cost
						std::partial_sort(grasp->second.solutions.begin(), grasp->second.solutions.begin() + populationSize, grasp->second.solutions.end());
						// best solution
						const Solution::Seq::iterator solution = grasp->second.solutions.begin();
						// frame and coordinates
						grasp->second.frame = solution->waypoint->wpos[armChains.begin()];
						grasp->second.coord = solution->waypoint->cpos;
						// workspace distance
						grasp->second.wdist = solution->wdist;
						// configspace
						Real cdist = REAL_ZERO;
						for (Configspace::Index i = armJoints.begin(); i < armJoints.end(); ++i)
							cdist += Math::sqr(solution->waypoint->cpos[i] - cbegin.cpos[i]);
						grasp->second.cdist = cdist;
						// weighted
						grasp->second.dist = solution->dist + grasp->second.cdist;
					}
				});

				// select a solution that minimise distance to the current robot configuration
				Grasp::Map::iterator selection = graspMap.begin();
				for (Grasp::Map::iterator i = ++graspMap.begin(); i != graspMap.end(); ++i)
					if (selection->second.dist > i->second.dist)
						selection = i;

				// improve
				population.clear();
				for (size_t i = 0; i < populationSize; ++i)
					population.push_back((selection->second.solutions.begin() + i)->waypoint->cpos);
				kinematics->setPopulation(&population);
				// set global root distance factor
				kinematics->setDistRootFac(kinematics->getDesc().distRootGlobalFac);
				GenConfigspaceState root;
				root.setToDefault(info.getJoints().begin(), info.getJoints().end());
				root.cpos = cbegin.cpos;
				// find the goal state
				GenWorkspaceChainState wend;
				wend.setToDefault(info.getChains().begin(), info.getChains().end()); // all used chains
				wend.wpos[armChains.begin()].multiply(selection->second.targetFrame.toMat34(), referencePose);
				golem::Controller::State cend = cbegin;
				if (!kinematics->findGoal(root, wend, cend, SecToMSec(ctrlIKin)))
					throw Message(Message::LEVEL_ERROR, "Unable to find inverse kinematic solution");
				selection->second.coord = cend.cpos;
				// update frame and error
				WorkspaceChainCoord wcc;
				controller->chainForwardTransform(cend.cpos, wcc);
				selection->second.frame = wcc[armChains.begin()];
				wcc[armChains.begin()].multiply(wcc[armChains.begin()], referencePose);
				const Real selectionDist = distCoeffs.dot(getDist(selection->second.targetFrame, RBCoord(selection->second.frame)));

				if (commandFollow) {
					// global vs local planner
					const bool global = current == graspMap.end() || /*current != selection && */currentDist > selectionDist + ctrlDistThr;
					// clear trajectory
					trj.clear();
					const SecTmReal tCtrl = context.getTimer().elapsed(), dtCtrl = tCtrl - t;

					// logging
					log->getTextStream()
						<< (trial + 1) << logSep << (step + 1) << logSep << t << logSep << (global ? "GLOBAL" : "LOCAL") << logSep
						<< (current == graspMap.end() ? std::string("-") : std::to_string(U32(std::distance(graspMap.begin(), current) + 1))) << logSep << currentDist << logSep
						<< U32(std::distance(graspMap.begin(), selection) + 1) << logSep << selectionDist << logSep;
					logTrn(objectFrame, log->getTextStream());
					logTrn(wcc[armChains.begin()], log->getTextStream());
					size_t j = 0;
					for (Grasp::Map::iterator i = graspMap.begin(); i != graspMap.end(); ++i)
						log->getTextStream() << ++j << logSep << i->second.dist << logSep << i->second.cdist << logSep << i->second.wdist.lin << logSep << i->second.wdist.ang << logSep;
					log->getTextStream() << logEnd;

					// print only for re-planning
					if (global)
						context.write("trial=%u, step=%u, time=%f, type=%s, curr_index=%s, curr_dist=%f, alt_index=%u, alt_dist=%f\n",
							trial + 1, step + 1, t, global ? "GLOBAL" : "LOCAL",
							current == graspMap.end() ? "-" : std::to_string(U32(std::distance(graspMap.begin(), current) + 1)).c_str(), currentDist,
							U32(std::distance(graspMap.begin(), selection) + 1), selectionDist
						);

					// global planning
					if (global) {
						cend.t = cbegin.t + dtCtrl + ctrlPred;
						// set new hand pose
						cend.cpos.set(handJoints, getTarget(selection->second.targetPath).config);
						// add collision bounds
						CollisionBounds collisionBounds(*planner, objectBounds);
						// find global trajectory
						(void)planner->findGlobalTrajectory(cbegin, cend, trj, trj.begin());
						// update state variables
						current = selection;
						currentDist = selectionDist;
					}
					// local planning
					else {
						// Setup target trajectory
						if (commandGrasp) {
							// skip the first waypoint (robot is already there)
							Assert::valid(current->second.targetPath.size() > 1, "Invalid target path length");
							wtrj.resize(current->second.targetPath.size() - 1);
							for (size_t i = 0; i < wtrj.size(); ++i) {
								const Manipulator::Waypoint& waypoint = current->second.targetPath[i + 1];
								wtrj[i].setToDefault(info.getChains().begin(), info.getChains().end()); // all used chains
								wtrj[i].wpos[armChains.begin()].multiply(waypoint.frame.toMat34(), referencePose);
								wtrj[i].t = cbegin.t + dtCtrl + ctrlReac + i * ctrlExec / wtrj.size();
							}
						}
						else {
							wtrj.resize(1);
							wtrj[0].setToDefault(info.getChains().begin(), info.getChains().end()); // all used chains
							wtrj[0].wpos[armChains.begin()].multiply(current->second.targetFrame.toMat34(), referencePose);
							wtrj[0].t = cbegin.t + dtCtrl + ctrlReac + ctrlPred;
						}
						{
							// disable waypoint prunning
							const bool prunning = planner->getProfile()->hasPruning();
							planner->getProfile()->setPruning(false);
							golem::ScopeGuard guard([=]() { planner->getProfile()->setPruning(prunning); });
							// find local trajectory
							(void)planner->findLocalTrajectory(cbegin, wtrj.begin(), wtrj.end(), trj, trj.begin(), SecToMSec(commandGrasp ? ctrlReac : ctrlExec));
							// remove cbegin from trj
							trj.erase(trj.begin());
						}
						// update trajectory
						if (wtrj.size() != trj.size())
							throw Message(Message::LEVEL_ERROR, "Invalid trajectory size %u != %u", wtrj.size(), trj.size());
						// profile
						if (commandGrasp) {
							// overwrite hand trajectory
							for (size_t i = 0; i < trj.size(); ++i) {
								const Manipulator::Waypoint& waypoint = current->second.targetPath[i + 1];
								trj[i].cpos.set(handJoints, waypoint.config);
							}
							// add extrapolation waypoint to enable actual grasping
							Controller::State extrapol(trj.back(), trj.back().t + (ctrlExec / trj.size()));
							for (Configspace::Index i = handJoints.begin(); i < handJoints.end(); ++i)
								extrapol.cpos[i] += grip[i];
							controller->clampConfig(extrapol);
							trj.push_back(extrapol);
							// profile
							Profile(plannerInfo, profileDesc, distance).profile(trj);
						}
						// update frame and error
						controller->chainForwardTransform(trj.back().cpos, wcc);
						current->second.frame = wcc[armChains.begin()];
						currentDist = distCoeffs.dot(getDist(current->second.targetFrame, RBCoord(wcc[armChains.begin()])));
						// make sure that the first waypoint is not too close in time
						const SecTmReal dtTrj = std::max(std::min(context.getTimer().elapsed() + controller->getCommandLatency() + ctrlPred - trj.front().t, ctrlPred), SEC_TM_REAL_ZERO);
						for (size_t i = 0; i < trj.size(); ++i)
							trj[i].t += dtTrj;
					}

					// move to stationary waypoint with velocity = 0 and acceleration = 0
					trj.back().cvel.setToDefault(info.getJoints()); 
					trj.back().cacc.setToDefault(info.getJoints());
					// move the robot
					try {
						(void)controller->send(&trj.front(), &trj.back() + 1, true);
					}
					catch (const std::exception& ex) {
						context.write("%s\n", ex.what());
					}
					// block until the last waypoint is executed
					if (global || commandGrasp)
						controller->waitForEnd();

					// update step
					++step;
				}
				// sleep to preserve constant cycle duration
				const SecTmReal dtSleep = ctrlIKin + ctrlReac - (context.getTimer().elapsed() - t);
				if (dtSleep > SEC_TM_REAL_ZERO)
					Sleep::msleep(SecToMSec(dtSleep));
				{
					// Update rendering buffers
					golem::CriticalSectionWrapper csw(getCS());
					demoRendererConfigs.reset();
					// target paths
					for (Grasp::Map::const_iterator i = graspMap.begin(); i != graspMap.end(); ++i)
						if (i != selection)
							appearanceBounds.draw(manipulator->getBounds(getTarget(i->second.targetPath).config, getTarget(i->second.targetPath).frame.toMat34()), demoRendererConfigs);
					// alternative target selection
					appearanceSelection.draw(manipulator->getBounds(getTarget(selection->second.targetPath).config, getTarget(selection->second.targetPath).frame.toMat34()), demoRendererConfigs);
					// solution to the current target path
					const Grasp::Map::const_iterator solution = current == graspMap.end() ? selection : current;
					appearanceSolution.draw(manipulator->getBounds(getTarget(solution->second.targetPath).config, solution->second.frame), demoRendererConfigs);
				}
			}
		}
	}));
}

//------------------------------------------------------------------------------

void golem::AppTracking::render() const {
	Player::render();
	demoRendererConfigs.render();
	demoRenderer.render();
	demoRendererObject.render();
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return AppTracking::Desc().main(argc, argv);
}
