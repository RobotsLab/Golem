/** @file Grasp.cpp
 *
 * Demo Grasp
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/App/Grasp/Grasp.h>
#include "GolemInteropStreamSocket.h"
#include "GolemInteropContactStreamDefs.h"
#include "GolemInteropGolemDefs.h"
#include "GolemInteropPCLDefs.h"

using namespace golem;

//-----------------------------------------------------------------------------

void AppGrasp::InteropDesc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("enabled", enabled, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("input_cloud", enabledInputCloud, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("training", enabledTraining, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("inference", enabledInference, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("override_break", overrideBreak, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("host", host, xmlcontext->getContextFirst("server", false), false);
	golem::XMLData("port", port, xmlcontext->getContextFirst("server", false), false);
}

void AppGrasp::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Player::Desc::load(context, xmlcontext);

	xmlcontext = xmlcontext->getContextFirst("demo");

	golem::XMLData("bundle", bundle, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("camera", camera, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("handler", imageHandler, xmlcontext->getContextFirst("image"));
	golem::XMLData("item", imageItem, xmlcontext->getContextFirst("image"));

	golem::XMLData("handler", processHandler, xmlcontext->getContextFirst("process"));
	golem::XMLData("item", processItem, xmlcontext->getContextFirst("process"));
	golem::XMLData("break", processBreak, xmlcontext->getContextFirst("process"));

	golem::XMLData("handler", modelHandler, xmlcontext->getContextFirst("model"));
	golem::XMLData("item", modelItem, xmlcontext->getContextFirst("model"));

	golem::XMLData("handler", queryHandler, xmlcontext->getContextFirst("query"));
	golem::XMLData("item", queryItem, xmlcontext->getContextFirst("query"));
	golem::XMLData("break", queryBreak, xmlcontext->getContextFirst("query"));

	golem::XMLData("handler", trjHandler, xmlcontext->getContextFirst("trajectory"));
	golem::XMLData("item", trjItem, xmlcontext->getContextFirst("trajectory"));
	golem::XMLData("perform_auto", trjAutoPerf, xmlcontext->getContextFirst("trajectory"));
	golem::XMLData("merge", trjMerge, xmlcontext->getContextFirst("trajectory"));
	golem::XMLData("break", trjBreak, xmlcontext->getContextFirst("trajectory"));

	golem::XMLData(detectFilterDesc, xmlcontext->getContextFirst("detection"));
	golem::XMLData("thread_chunk_size", detectThreadChunkSize, xmlcontext->getContextFirst("detection"));
	try {
		XMLData(detectRegionDesc, detectRegionDesc.max_size(), xmlcontext->getContextFirst("detection"), "bounds");
	}
	catch (const MsgXMLParserNameNotFound&) {
	}
	golem::XMLData("min_size", detectMinSize, xmlcontext->getContextFirst("detection"));
	golem::XMLData("delta_size", detectDeltaSize, xmlcontext->getContextFirst("detection"));
	golem::XMLData("delta_depth", detectDeltaDepth, xmlcontext->getContextFirst("detection"));

	poseScanSeq.clear();
	XMLData(poseScanSeq, poseScanSeq.max_size(), xmlcontext->getContextFirst("scan"), "pose");

	try {
		poseActionSeq.clear();
		XMLData(poseActionSeq, poseActionSeq.max_size(), xmlcontext->getContextFirst("action"), "pose");
	}
	catch (const MsgXMLParserNameNotFound&) {
	}

	golem::XMLData("show", pointShow, xmlcontext->getContextFirst("appearance point"));
	golem::XMLData(pointColour, xmlcontext->getContextFirst("appearance point"));

	try {
		bundleSeq.clear();
		XMLData(bundleSeq, bundleSeq.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "bundle");
	}
	catch (const MsgXMLParserNameNotFound&) {
	}

#ifdef _GOLEM_APP_GRASP_INTEROP_
	interopDesc.load(xmlcontext->getContextFirst("interop"));
#endif // _GOLEM_APP_GRASP_INTEROP_
}

//------------------------------------------------------------------------------

AppGrasp::AppGrasp(Scene &scene) : Player(scene), camera(nullptr), imageHandler(nullptr), processHandler(nullptr), modelHandler(nullptr), queryHandler(nullptr), trjHandler(nullptr) {
}

void golem::AppGrasp::release() {
	Player::release();
}

void AppGrasp::create(const Desc& desc) {
	desc.assertValid(Assert::Context("AppGrasp::Desc."));

	// create object
	Player::create(desc); // throws

	golem::Sensor::Map::const_iterator cameraPtr = sensorMap.find(desc.camera);
	camera = cameraPtr != sensorMap.end() ? is<CameraDepth>(cameraPtr->second.get()) : nullptr;
	//Assert::valid(camera != nullptr, "golem::AppGrasp::create(): unknown depth camera: %s", desc.camera.c_str());

	bundle = desc.bundle;

	golem::data::Handler::Map::const_iterator imageHandlerPtr = handlerMap.find(desc.imageHandler);
	imageHandler = imageHandlerPtr != handlerMap.end() ? imageHandlerPtr->second.get() : nullptr;
	Assert::valid(imageHandler != nullptr, "golem::AppGrasp::create(): unknown image handler: %s", desc.imageHandler.c_str());
	imageItem = desc.imageItem;

	golem::data::Handler::Map::const_iterator processHandlerPtr = handlerMap.find(desc.processHandler);
	processHandler = processHandlerPtr != handlerMap.end() ? processHandlerPtr->second.get() : nullptr;
	Assert::valid(processHandler != nullptr, "golem::AppGrasp::create(): unknown process handler: %s", desc.processHandler.c_str());
	processItem = desc.processItem;
	processBreak = desc.processBreak;

	golem::data::Handler::Map::const_iterator modelHandlerPtr = handlerMap.find(desc.modelHandler);
	modelHandler = modelHandlerPtr != handlerMap.end() ? modelHandlerPtr->second.get() : nullptr;
	Assert::valid(modelHandler != nullptr, "golem::AppGrasp::create(): unknown contact model handler: %s", desc.modelHandler.c_str());
	modelItem = desc.modelItem;

	golem::data::Handler::Map::const_iterator queryHandlerPtr = handlerMap.find(desc.queryHandler);
	queryHandler = queryHandlerPtr != handlerMap.end() ? queryHandlerPtr->second.get() : nullptr;
	Assert::valid(queryHandler != nullptr, "golem::AppGrasp::create(): unknown contact query handler: %s", desc.queryHandler.c_str());
	queryItem = desc.queryItem;
	queryBreak = desc.queryBreak;

	golem::data::Handler::Map::const_iterator trjHandlerPtr = handlerMap.find(desc.trjHandler);
	trjHandler = trjHandlerPtr != handlerMap.end() ? trjHandlerPtr->second.get() : nullptr;
	Assert::valid(trjHandler != nullptr, "golem::AppGrasp::create(): unknown trajectory handler: %s", desc.trjHandler.c_str());
	trjItem = desc.trjItem;
	trjAutoPerf = desc.trjAutoPerf;
	trjMerge = desc.trjMerge;
	trjBreak = desc.trjBreak;

	detectFilterDesc = desc.detectFilterDesc;
	detectThreadChunkSize = desc.detectThreadChunkSize;
	detectRegionDesc = desc.detectRegionDesc;
	detectMinSize = desc.detectMinSize;
	detectDeltaSize = desc.detectDeltaSize;
	detectDeltaDepth = desc.detectDeltaDepth;

	poseScanSeq = desc.poseScanSeq;
	poseActionSeq = desc.poseActionSeq;

	// default planner
	pointShow = desc.pointShow;
	pointColour = desc.pointColour;

	bundleSeq = desc.bundleSeq;

	interopDesc = desc.interopDesc;

	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0F51", "  R                                       menu run\n"));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("R", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: run (G)rasp/(T)rajectory demo...";
	}));
	menuCmdMap.insert(std::make_pair("RT", [=]() {
		// check for camera
		if (bundleSeq.empty())
			throw Message(Message::LEVEL_ERROR, "No trajectory bundles");

		for (Bundle::Seq::const_iterator bundle = bundleSeq.begin(); bundle != bundleSeq.end(); ++bundle) {
			ScopeGuard cleanup([&]() {
				UI::addCallback(*this, getCurrentHandler());
				golem::CriticalSectionWrapper csw(scene.getCS());
				demoRenderer.reset();
			});

			// select data bundle
			data::Data::Map::iterator dataPtr = dataMap.find(bundle->path);
			if (dataPtr == dataMap.end())
				throw Message(Message::LEVEL_ERROR, "Unknown data bundle: %s", bundle->path.c_str());
			setCurrentDataPtr(dataPtr);

			RenderBlock renderBlock(*this);

			// image
			const std::string imageName = bundle->image.empty() ? this->processItem : bundle->image;
			data::Item::Map::iterator imagePtr = to<Data>(dataCurrentPtr)->itemMap.find(imageName);
			if (imagePtr == to<Data>(dataCurrentPtr)->itemMap.end())
				throw Message(Message::LEVEL_ERROR, "Unable to find image item: \"%s\"", imageName.c_str());

			const data::Point3D* point = is<const data::Point3D>(imagePtr);
			if (pointShow && point) {
				golem::CriticalSectionWrapper csw(scene.getCS());
				
				for (size_t i = 0; i < point->getSize(); ++i) {
					const Vec3 p((const Vec3&)point->getPoint(i));
					//const RGBA c(point->getColour(i));
					if (!Cloud::isNanXYZ(p))
						demoRenderer.addPoint(p, pointColour);
						//demoRenderer.addPoint(p, RGBA(c._rgba.r, c._rgba.g, c._rgba.b, 255));
				}
			}

			// start
			if (option("\x0D", std::string("Press <Enter> to run " + bundle->path + "\n").c_str(), true) != 13)
				break;

			// trajectory
			const std::string trjName = bundle->trajectory.empty() ? this->trjItem : bundle->trajectory;
			data::Item::Map::iterator trjPtr = to<Data>(dataCurrentPtr)->itemMap.find(trjName);
			if (trjPtr == to<Data>(dataCurrentPtr)->itemMap.end())
				throw Message(Message::LEVEL_ERROR, "Unable to find trajectory item: \"%s\"", trjName.c_str());

			data::Trajectory* trajectory = is<data::Trajectory>(trjPtr);
			Assert::valid(trajectory != nullptr, "Trajectory item %s does not support Trajectory interafce", trjPtr->second->getHandler().getID().c_str());

			// export trajectory
			trajectory->setProfile(trajectoryProfileSemiAuto);

			Controller::State::Seq seq;
			trajectory->createTrajectory(seq);

			// go to pose
			const ConfigspaceCoord& coord = seq.front().cpos;
			gotoPose(Pose(RealSeq(coord.data(), coord.data() + getPlanner().armInfo.getJoints().size())));

			// find global trajectory & perform
			Controller::State::Seq manipulationTrajectory; // empty
			performSemiAuto(seq, manipulationTrajectory, trajectory->getManifold(), this->trjMerge, dataCurrentPtr->first, this->trjItem);

			// finish
			if (option("\x0D", "Press <Enter> to continue...\n", true) != 13)
				break;
		}

		// done!
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("RG", [=]() {
#ifdef _GOLEM_APP_GRASP_INTEROP_
		// open connection to Grasp server
		interop::Client client;
		interop::Stream::Ptr stream;
		if (interopDesc.enabled)
			stream = client.connect<interop::StreamSocket>(interopDesc.host, interopDesc.port);
#endif // _GOLEM_APP_GRASP_INTEROP_

		// check for camera
		if (!interopDesc.enabledInputCloud && !camera)
			throw Message(Message::LEVEL_ERROR, "Unknown depth camera: %s", desc.camera.c_str());

		// select data bundle
		data::Data::Map::iterator dataPtr = dataMap.find(bundle);
		if (dataPtr == dataMap.end())
			throw Message(Message::LEVEL_ERROR, "Unknown data bundle: %s", bundle.c_str());
		setCurrentDataPtr(dataPtr);

		// find model
		const data::Item::Map::iterator modelPtr = to<Data>(dataCurrentPtr)->itemMap.find(modelItem);
		if (modelPtr == to<Data>(dataCurrentPtr)->itemMap.end())
			throw Message(Message::LEVEL_ERROR, "Unable to find model item: %s", modelItem.c_str());

		// debug mode
		const bool debugMode = option(0, "Debug mode: ", { "YES", "NO" }) == 0;
		const auto breakPoint = [&] (const char* str) {
			if (!debugMode && str)
				context.write("%s....\n", str);
			if ( debugMode && str ? option("\x0D", makeString("%s: Press <Enter> to continue...\n", str).c_str()) != 13 : waitKey(5) == 27)
				throw Cancel("Demo cancelled");
		};

		// only if debug mode is disabled
		const auto overrideBreak = [&] (bool hasBreak) -> bool {
			if (interopDesc.enabled && interopDesc.overrideBreak && !debugMode)
				if (waitKey(5) == 27)
					throw Cancel("Demo cancelled");
				else
					return false;
			return hasBreak;
		};

		Cloud::PointSeq prev, next;
		Cloud::PointSeqQueue pointSeqQueue(detectFilterDesc.window);

		ScopeGuard cleanup([&]() {
			if (camera) {
				camera->set(Camera::CMD_STOP);
				camera->setProperty(camera->getImageProperty());
			}
			UI::addCallback(*this, getCurrentHandler());
		});

		for (;;) {
			// create new data
			if (debugMode)
				menuCmdMap["DN"]();

			// remove temp items
			{
				RenderBlock renderBlock(*this);
				golem::CriticalSectionWrapper csw(scene.getCS());
				demoRenderer.reset();
				to<Data>(dataCurrentPtr)->itemMap.erase(imageItem);
				to<Data>(dataCurrentPtr)->itemMap.erase(processItem);
				to<Data>(dataCurrentPtr)->itemMap.erase(queryItem);
				to<Data>(dataCurrentPtr)->itemMap.erase(trjItem);
			}

			// depth images list
			data::Item::List imageList;

			// add image item
			auto addImage = [&] (data::Item::Ptr item) {
				RenderBlock renderBlock(*this);
				golem::CriticalSectionWrapper csw(scene.getCS());
				demoRenderer.reset();
				data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(imageItem, item));
				Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
				// insert image to the processing list
				imageList.push_back(ptr);
			};

#ifdef _GOLEM_APP_GRASP_INTEROP_
			if (interopDesc.enabled && interopDesc.enabledInputCloud) {
				// request input clouds
				interop::Point3DCloud::Seq cloudSeq;
				interop::StreamWrite(*stream, interop::GRASP_INTEROP_INPUT_CLOUD);
				interop::StreamRead(*stream, cloudSeq);
				// convert clouds
				for (auto& cloud : cloudSeq) {
					data::Item::Ptr item = imageHandler->create();
					Image* image = is<Image>(item.get()); // image == nullptr should never happen
					interop::convert(cloud, *image->cloud);
					image->timeStamp = context.getTimer().elapsed();
					addImage(item);
				}
			}
			else
#endif // _GOLEM_APP_GRASP_INTEROP_
			{
				// capture from poses
				for (Pose::Seq::const_iterator j = poseScanSeq.begin(); j != poseScanSeq.end(); ++j) {
					// go to object detection pose
					breakPoint("Going to detection pose");
					gotoPose(*j);

					// detect changes only at the first pose
					const bool detection = j == poseScanSeq.begin();

					// detect changes
					if (detection) {
						const Mat34 cameraFrame = camera->getFrame();

						// Detection region in global coordinates
						golem::Bounds::Seq detectRegion;
						for (golem::Bounds::Desc::Seq::const_iterator i = detectRegionDesc.begin(); i != detectRegionDesc.end(); ++i)
							detectRegion.push_back((*i)->create());
						// Move bounds to camera frame
						Mat34 cameraFrameInv;
						cameraFrameInv.setInverse(cameraFrame);
						for (golem::Bounds::Seq::const_iterator i = detectRegion.begin(); i != detectRegion.end(); ++i)
							(*i)->multiplyPose(cameraFrameInv, (*i)->getPose());

						// start camera
						camera->setProperty(camera->getDepthProperty());
						if (!camera->set(Camera::CMD_VIDEO))
							throw Message(Message::LEVEL_ERROR, "Unable to start camera");

						pointSeqQueue.clear();
						next.clear();
						prev.clear();
						U32 prevSize = 0;

						for (bool accept = false; !accept;) {
							breakPoint(nullptr);

							// clear image queue
							golem::SecTmReal timeStamp = context.getTimer().elapsed();
							// retrieve a next frame
							for (bool stop = false; !stop;)
								camera->waitAndPeek([&](const Image::List& images, bool result) {
								if (!result || images.empty())
									throw Message(Message::LEVEL_ERROR, "unable to capture image");
								// search from the back, i.e. the latest frames
								for (Image::List::const_reverse_iterator i = images.rbegin(); i != images.rend(); ++i)
									if (timeStamp < (*i)->timeStamp || next.empty()) {
										timeStamp = (*i)->timeStamp;
										Image::assertData((*i)->cloud);
										next = *(*i)->cloud;
										stop = true;
									}
							});

							// clip region
							try {
								if (!detectRegion.empty())
									Cloud::regionClip(context, detectRegion, detectThreadChunkSize, next, true);
								// transform points
								Cloud::transform(cameraFrame, next, next);
							}
							catch (const Message&) {}

							// filter
							pointSeqQueue.push_back(next);
							if (!pointSeqQueue.full())
								continue;
							Cloud::filter(context, detectFilterDesc, detectThreadChunkSize, pointSeqQueue, next);

							// debug accept? <Enter>
							const bool debugAccept = waitKey(0) == '\x0D';

							// count and render points
							U32 nextSize = 0;
							{
								golem::CriticalSectionWrapper csw(scene.getCS());
								demoRenderer.reset();
								for (Cloud::PointSeq::const_iterator i = next.begin(); i != next.end(); ++i)
									if (!Cloud::isNanXYZ(*i)) {
										demoRenderer.addPoint(Vec3(i->x, i->y, i->z), RGBA(i->r, i->g, i->b, 255));
										++nextSize;
									}
							}
							if (!debugAccept && nextSize < detectMinSize)
								continue;

							if (prev.size() == next.size()) {
								size_t sharedSize = 0;
								Real deltaDepth = REAL_ZERO;
								for (size_t i = 0; i < next.size(); ++i) {
									const Cloud::Point p = prev[i], n = next[i];
									if (Cloud::isNanXYZ(p) || Cloud::isNanXYZ(n))
										continue;
									++sharedSize;
									deltaDepth += Math::sqrt(Math::sqr(p.x - n.x) + Math::sqr(p.y - n.y) + Math::sqr(p.z - n.z));
								}
								if (debugAccept || (nextSize - sharedSize < detectDeltaSize && deltaDepth / sharedSize < detectDeltaDepth)) {
									context.write("Object detected (size=%u, delta_size=%u, delta_depth=%f\n", nextSize, nextSize - sharedSize, deltaDepth / sharedSize);
									if (debugMode && option("\x08\x0D", "Press <Bkspace> to repeat, <Enter> to continue...\n", true) == 8)
										continue;
									accept = true;
								}
							}
							if (!accept) {
								prev = next;
								prevSize = nextSize;
							}
						}
					}
					else
						breakPoint(nullptr);

					// capture image
					data::Capture* capture = is<data::Capture>(imageHandler);
					Assert::valid(capture != nullptr, "Handler %s does not support Capture interface", imageHandler->getID().c_str());

					// capture image
					addImage(capture->capture(*camera, [&](const golem::TimeStamp*) -> bool { return true; }));
				}
			}

			// process images
			data::Transform* processTransform = is<data::Transform>(processHandler);
			Assert::valid(processTransform != nullptr, "Handler %s does not support Transform interface", processHandler->getID().c_str());

			data::Item::Ptr processItem = processTransform->transform(imageList);
			data::Item::Map::iterator processPtr;
			{
				RenderBlock renderBlock(*this);
				golem::CriticalSectionWrapper csw(scene.getCS());
				processPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(this->processItem, processItem));
				Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, processPtr, to<Data>(dataCurrentPtr)->getView());
			}
			if (overrideBreak(processBreak)) {
				EnableKeyboardMouse enableKeyboardMouse(*this);
				(void)option("\x0D", "Press <7><8> to select cluster, <Space> to add/remove cluster, <Enter> to continue...\n", true);
			}

			// compute grasps
			data::Transform* queryTransform = is<data::Transform>(queryHandler);
			Assert::valid(queryTransform != nullptr, "Handler %s does not support Transform interface", queryHandler->getID().c_str());
			
			data::Item::List queryList;
			queryList.push_back(modelPtr);
			queryList.push_back(processPtr);
			data::Item::Ptr queryItem = queryTransform->transform(queryList);
			data::Item::Map::iterator queryPtr;
			{
				RenderBlock renderBlock(*this);
				golem::CriticalSectionWrapper csw(scene.getCS());
				queryPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(this->queryItem, queryItem));
				Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, queryPtr, to<Data>(dataCurrentPtr)->getView());
			}

			data::Convert* queryConvert = is<data::Convert>(&*queryItem);
			Assert::valid(queryConvert != nullptr, "Item %s does not support Convert interface", this->queryItem.c_str());

			data::ContactQuery* contactQuery = is<data::ContactQuery>(queryItem.get());
			Assert::valid(contactQuery != nullptr, "ContactQuery item %s does not support ContactQuery interafce", queryHandler->getID().c_str());

#ifdef _GOLEM_APP_GRASP_INTEROP_
			if (interopDesc.enabled && (interopDesc.enabledTraining || interopDesc.enabledInference)) {
				// request input clouds
				interop::StreamWrite(*stream, interopDesc.enabledTraining ? interop::GRASP_INTEROP_TRAINING : interop::GRASP_INTEROP_INFERENCE);
				// send clouds
				interop::Point3DCloud::Seq cloudSeq;
				for (auto& item : imageList) {
					Image* image = is<Image>(item->second.get()); // image == nullptr should never happen
					interop::Point3DCloud cloud;
					interop::convert(*image->cloud, cloud);
					cloudSeq.push_back(cloud);
				}
				interop::StreamWrite(*stream, cloudSeq);
				// read query data
				data::ContactQuery::Data contactQueryData = contactQuery->getData();
				// send query paths
				interop::Query query;
				convert(contactQueryData, query);
				interop::StreamWrite(*stream, query.paths);
				// receive updated query paths
				interop::StreamRead(*stream, query.paths);
				convert(query, contactQueryData);
				// write query data
				contactQuery->setData(contactQueryData);
				// done?
				if (interopDesc.enabledTraining)
					goto DONE;
			}
#endif // _GOLEM_APP_GRASP_INTEROP_
			
			// grasp execution section
			{
				ScopeGuard cleanupPoints([&]() {
					golem::CriticalSectionWrapper csw(scene.getCS());
					demoRenderer.reset();
				});

				// select grasp with approach trajectory
				data::Item::Ptr trjItem;
				for (;;) {
					EnableKeyboardMouse enableKeyboardMouse(*this);

					try {
						if (overrideBreak(queryBreak))
							trjItem = queryConvert->convert(*trjHandler);
						else {
							InputBlock inputBlock(*this);
							trjItem = queryConvert->convert(*trjHandler);
						}
					}
					catch (const Message& msg) {
						context.write(msg);
						trjItem.reset();
					}

					if (/*!overrideBreak(queryBreak) || */option("\x0D() ", "Press <(><)> to select grasp, <Space> to insert/remove grasp to cluster, <Enter> to continue...\n", true) == 13) {
						break;
					}
				}
				if (trjItem == nullptr) {
					context.write("No feasible trajectory found\n");
					continue;
				}

				data::Trajectory* trajectory = is<data::Trajectory>(&*trjItem);
				Assert::valid(trajectory != nullptr, "Trajectory item %s does not support Trajectory interafce", trjItem->getHandler().getID().c_str());

				const data::Point3D* point = is<const data::Point3D>(&*processItem);
				if (pointShow && point) {
					golem::CriticalSectionWrapper csw(scene.getCS());

					for (size_t i = 0; i < point->getSize(); ++i) {
						const Vec3 p((const Vec3&)point->getPoint(i));
						//const RGBA c(point->getColour(i));
						if (!Cloud::isNanXYZ(p))
							demoRenderer.addPoint(p, pointColour);
						//demoRenderer.addPoint(p, RGBA(c._rgba.r, c._rgba.g, c._rgba.b, 255));
					}
				}

				// select collision object
				CollisionBounds::Ptr collisionBounds;
				if (trjAutoPerf && point) {
					collisionBounds.reset(new CollisionBounds(
						*getPlanner().planner,
						CollisionBounds::getBounds([=](size_t i, Vec3& p) -> bool { if (i < point->getSize()) p = point->getPoint(i); return i < point->getSize(); }, is<const data::Cluster3D>(point)),
						&demoRenderer,
						&scene.getCS()
					));

					golem::CriticalSectionWrapper csw(scene.getCS());
					for (size_t i = point->getSize(); i > 0;)
						demoRenderer.addPoint(Vec3((const golem::data::Point3D::Vec3&)point->getPoint(--i)), collisionPointColour);
				}

				// find global trajectory & perform
				// export trajectory
				Controller::State::Seq seq;
				if (overrideBreak(trjBreak))
					UI::addCallback(*this, &trjItem->getHandler());
				perform(dataCurrentPtr->first, this->trjItem, trajectory, seq, debugMode, trjMerge, trjAutoPerf);

				// prepare remaining poses
				Pose::Seq poseSeq;
				for (Pose::Seq::const_iterator i = poseActionSeq.begin(); i != poseActionSeq.end(); ++i) {
					Pose pose = *i;
					// use hand configuration
					const Real* c = pose.flags == "open" ? seq.front().cpos.data() : pose.flags == "close" ? seq.back().cpos.data() : nullptr;
					if (c) std::copy(c + *getPlanner().handInfo.getJoints().begin(), c + *getPlanner().handInfo.getJoints().end(), pose.c.data() + *getPlanner().handInfo.getJoints().begin());
					// go to pose
					poseSeq.push_back(pose);
				}

				// goto remaining poses
				for (Pose::Seq::const_iterator i = poseSeq.begin(); i != poseSeq.end(); ++i) {
					// go to pose
					//breakPoint("Going to action pose");
					gotoPose(*i);
				}

				// compute complete trajectory
				data::Item::Map::iterator trjPtr;
				{
					RenderBlock renderBlock(*this);
					golem::CriticalSectionWrapper csw(scene.getCS());
					trjPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(this->trjItem, trjItem));
					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, trjPtr, to<Data>(dataCurrentPtr)->getView());
				}
			}

			// Finish
			DONE:

			// save data
			if (debugMode) {
				//menuCmdMap["DS"]();
				std::string path = dataCurrentPtr->first;
				readPath("Enter data path to save: ", path, dataExt.c_str());
				if (getExt(path).empty())
					path += dataExt;
				to<Data>(dataCurrentPtr)->save(path);
			}

			if (overrideBreak(true) && option("\x0D", "Done! Press <Enter> to continue...\n", true) != 13)
				break;
		}
	}));
}

//------------------------------------------------------------------------------

void golem::AppGrasp::render() const {
	Player::render();
	demoRenderer.render();
}

//------------------------------------------------------------------------------

void golem::AppGrasp::findTrajectory(const golem::Controller::State& begin, const Pose& pose, golem::Controller::State::Seq& trajectory) {
	//context.debug("STATE[1]: t=%f, (%f, %f, %f, %f, %f, %f, %f)\n", begin.t, begin.cpos.data()[0], begin.cpos.data()[1], begin.cpos.data()[2], begin.cpos.data()[3], begin.cpos.data()[4], begin.cpos.data()[5], begin.cpos.data()[6]);

	// target
	golem::Controller::State end = begin;
	if (pose.configspace)
		end.cpos.set(pose.c.data(), pose.c.data() + std::min(pose.c.size(), (size_t)info.getJoints().size()));
	else
		end.cpos.set(pose.c.data() + *getPlanner().handInfo.getJoints().begin(), pose.c.data() + *getPlanner().handInfo.getJoints().end(), *getPlanner().handInfo.getJoints().begin());

	Mat34 wpose;
	if (pose.position || pose.orientation) {
		WorkspaceChainCoord wcc;
		controller->chainForwardTransform(begin.cpos, wcc);
		wpose.multiply(wcc[getPlanner().armInfo.getChains().begin()], controller->getChains()[getPlanner().armInfo.getChains().begin()]->getReferencePose());
		if (pose.position) wpose.p = pose.w.p;
		if (pose.orientation) wpose.R = pose.w.R;
	}

	// find trajectory
	trajectory.clear();
	golem::Player::findTrajectory(begin, &end, pose.position || pose.orientation ? &wpose : nullptr, pose.dt, trajectory);
}

void golem::AppGrasp::gotoPose(const Pose& pose) {
	// find trajectory
	golem::Controller::State::Seq trajectory;
	findTrajectory(WaypointCtrl::lookup(*controller).command, pose, trajectory);
	// send
	sendTrajectory(trajectory);
	// wait for end
	//controller->waitForEnd();
	for (; !controller->waitForEnd(1);) {
		const int key = universe.waitKey(0);
		if (universe.interrupted()) {
			controller->stop();
			throw Exit();
		}
		if (key == 27) {
			controller->stop();
			throw Cancel("Movement interrupted");
		}
	}
	// sleep
	Sleep::msleep(SecToMSec(getPlanner().trajectoryIdleEnd));
}

//------------------------------------------------------------------------------

void golem::XMLData(golem::AppGrasp::Pose &val, golem::XMLContext* xmlcontext, bool create) {
	if (create) {
		golem::XMLData((ConfigMat34&)val, xmlcontext, true);
		golem::XMLData("dt", val.dt, xmlcontext, true);
		golem::XMLData("flags", val.flags, xmlcontext, true);
	}
	else {
		val.configspace = false;
		val.position = false;
		val.orientation = false;

		try {
			//golem::XMLData((ConfigMat34&)val, xmlcontext, false);
			golem::XMLDataSeq(val.c, "c", xmlcontext, create, golem::REAL_ZERO);
			val.configspace = true;
		}
		catch (golem::MsgXMLParser&) {
			val.c.assign(golem::Configspace::DIM, golem::REAL_ZERO);
		}
		try {
			golem::XMLData(val.w.p, xmlcontext);
			val.position = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::Quat quat;
			golem::XMLData(quat, xmlcontext);
			val.w.R.fromQuat(quat);
			val.orientation = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::Real angle;
			golem::Vec3 axis;
			golem::XMLDataAngleAxis(angle, axis, xmlcontext);
			val.w.R.fromAngleAxis(angle, axis);
			val.orientation = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::Real roll, pitch, yaw;
			golem::XMLDataEuler(roll, pitch, yaw, xmlcontext);
			val.w.R.fromEuler(roll, pitch, yaw);
			val.orientation = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::XMLData(val.w.R, xmlcontext);
			val.orientation = true;
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::Twist twist;
			golem::Real theta;
			golem::XMLData(twist, theta, xmlcontext);
			val.w.fromTwist(twist, theta);
			val.orientation = true;
			val.position = true;
		}
		catch (golem::MsgXMLParser&) {}

		//if (!val.configspace && !val.position && !val.orientation)
		//	throw Message(Message::LEVEL_ERROR, "XMLData(): %s: invalid pose description", xmlcontext->getName().c_str());

		try {
			golem::XMLData("dt", val.dt, xmlcontext);
		}
		catch (golem::MsgXMLParser&) {}
		try {
			golem::XMLData("flags", val.flags, xmlcontext);
		}
		catch (golem::MsgXMLParser&) {}
	}
}

void golem::XMLData(AppGrasp::Bundle & val, golem::XMLContext * xmlcontext, bool create) {
	golem::XMLData("path", val.path, xmlcontext, create);
	golem::XMLData("image", val.image, xmlcontext, create);
	golem::XMLData("trajectory", val.trajectory, xmlcontext, create);
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return AppGrasp::Desc().main(argc, argv);
}
