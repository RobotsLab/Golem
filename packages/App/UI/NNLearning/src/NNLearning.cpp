/** @file NNLearning.cpp
*
* Demo NNLearning
*
* @author	Marek Kopicki
*
* @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
*
* @license  This file copy is licensed to you under the terms described in
*           the License.txt file included in this distribution.
*
*/

#include <Golem/App/NNLearning/NNLearning.h>
#include <Golem/Tools/Image.h>
#include <Golem/Data/Image/Image.h>
#include <Golem/Tools/Import.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Contact/Data.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <Connector.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

using namespace golem;

//-----------------------------------------------------------------------------

void AppNNLearning::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
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
	poseActionSeq.clear();
	XMLData(poseActionSeq, poseActionSeq.max_size(), xmlcontext->getContextFirst("action"), "pose");
}

//------------------------------------------------------------------------------

AppNNLearning::AppNNLearning(Scene &scene) : Player(scene), camera(nullptr), imageHandler(nullptr), processHandler(nullptr), modelHandler(nullptr), queryHandler(nullptr), trjHandler(nullptr) {
}

AppNNLearning::~AppNNLearning() {
}

void getSimulatedData(Image::Ptr& pImage){
	// initialize buffer
	if (pImage == nullptr)
		pImage.reset(new Image());
	pImage->cloud.reset(new PointSeq((uint32_t)640, (uint32_t)480));

	std::cout << "Output size" << pImage->cloud->size() << std::endl;

}

void AppNNLearning::create(const Desc& desc) {
	desc.assertValid(Assert::Context("AppNNLearning::Desc."));

	// create object
	Player::create(desc); // throws
	bundle = desc.bundle;

	golem::data::Handler::Map::const_iterator imageHandlerPtr = handlerMap.find(desc.imageHandler);
	imageHandler = imageHandlerPtr != handlerMap.end() ? imageHandlerPtr->second.get() : nullptr;
	Assert::valid(imageHandler != nullptr, "golem::AppNNLearning::create(): unknown image handler: %s", desc.imageHandler.c_str());
	imageItem = desc.imageItem;

	golem::data::Handler::Map::const_iterator processHandlerPtr = handlerMap.find(desc.processHandler);
	processHandler = processHandlerPtr != handlerMap.end() ? processHandlerPtr->second.get() : nullptr;
	Assert::valid(processHandler != nullptr, "golem::AppNNLearning::create(): unknown process handler: %s", desc.processHandler.c_str());
	processItem = desc.processItem;
	processBreak = desc.processBreak;

	golem::data::Handler::Map::const_iterator modelHandlerPtr = handlerMap.find(desc.modelHandler);
	modelHandler = modelHandlerPtr != handlerMap.end() ? modelHandlerPtr->second.get() : nullptr;
	Assert::valid(modelHandler != nullptr, "golem::AppNNLearning::create(): unknown contact model handler: %s", desc.modelHandler.c_str());
	modelItem = desc.modelItem;

	golem::data::Handler::Map::const_iterator queryHandlerPtr = handlerMap.find(desc.queryHandler);
	queryHandler = queryHandlerPtr != handlerMap.end() ? queryHandlerPtr->second.get() : nullptr;
	Assert::valid(queryHandler != nullptr, "golem::AppNNLearning::create(): unknown contact query handler: %s", desc.queryHandler.c_str());
	queryItem = desc.queryItem;
	queryBreak = desc.queryBreak;

	golem::data::Handler::Map::const_iterator trjHandlerPtr = handlerMap.find(desc.trjHandler);
	trjHandler = trjHandlerPtr != handlerMap.end() ? trjHandlerPtr->second.get() : nullptr;
	Assert::valid(trjHandler != nullptr, "golem::AppNNLearning::create(): unknown trajectory handler: %s", desc.trjHandler.c_str());
	trjItem = desc.trjItem;
	trjBreak = desc.trjBreak;

	detectFilterDesc = desc.detectFilterDesc;
	detectThreadChunkSize = desc.detectThreadChunkSize;
	detectRegionDesc = desc.detectRegionDesc;
	detectMinSize = desc.detectMinSize;
	detectDeltaSize = desc.detectDeltaSize;
	detectDeltaDepth = desc.detectDeltaDepth;

	poseScanSeq = desc.poseScanSeq;
	poseActionSeq = desc.poseActionSeq;

	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0F51", "  R                                       menu run\n"));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("R", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press (S) to start Golem server...";
		const_cast<DebugRenderer&>(objectRenderer).addAxes3D(Mat34::identity(),
			Vec3(1.0));

	}));

	menuCmdMap.insert(std::make_pair("RS", [=]() {
		std::cout << "Entering server loop" << std::endl;
		std::cout.flush();

		// select data bundle
		data::Data::Map::iterator dataPtr = dataMap.find(bundle);
		if (dataPtr == dataMap.end())
			throw Message(Message::LEVEL_ERROR, "unknown data bundle: %s", bundle.c_str());
		setCurrentDataPtr(dataPtr);

		// find model
		const data::Item::Map::iterator modelPtr = to<Data>(dataCurrentPtr)->itemMap.find(modelItem);
		if (modelPtr == to<Data>(dataCurrentPtr)->itemMap.end())
			throw Message(Message::LEVEL_ERROR, "unable to find model item: %s", modelItem.c_str());

		// debug mode
		const bool debugMode = false;

		Cloud::PointSeq prev, next;
		Cloud::PointSeqQueue pointSeqQueue(detectFilterDesc.window);

		ScopeGuard cleanup([&]() {
			UI::addCallback(*this, getCurrentHandler());
		});

		// Read server id
		int serverID = 0;
		try{
			FILE * fp = fopen("serverID.txt", "r");
			fscanf(fp, "%d", &serverID);
			fclose(fp);
		}
		catch (std::exception &e)
		{
			std::cout << "ERROR: Server ID NOT KNOWN. USING 0." << std::endl;
		}

		std::cout << "Entering for loop" << std::endl;

		for (;;) {
			// Create data structures for point cloud processing.
			data::Item::List list;
			data::Item::Ptr item(imageHandler->create());
			data::ItemImage* itemImage = to<data::ItemImage>(item.get());

			// Download the image
			char tmpFileName[100];
			tmpFileName[0] = 0;

			std::cout << "Waiting for the cloud data....." << std::endl;
			// Get the point cloud from the cloud (hehe).
			try{
				while (true)
				{
					bool flag = Grasp::Connector::DownloadFile(tmpFileName, serverID);
					if (flag) {
						std::cout << "File has arrived:" << tmpFileName << std::endl;

						// File has arrived. Read it and delete it.
						pcl::PCDReader().read(std::string(tmpFileName), *itemImage->cloud);
						boost::filesystem::remove(tmpFileName);

						// Break
						break;
					}

					// Wait for a lil bit more
					std::this_thread::sleep_for(std::chrono::seconds(1));
				}
			}
			catch (const std::exception& e) { 
				// There was an issue with the file. Perhaps another server process is dealing with it. Continue;
				std::cout << "There was an issue with the file, another process has likely deleted it. Doing nothing." << std::endl;
				try{
					boost::filesystem::remove(tmpFileName);
				}
				catch (const std::exception& e) {
					std::cout << "Could not delete " << tmpFileName << std::endl;
				}

				continue;
			}

			// Process the point cloud.
			try{
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

				// Insert image to processing list
				{
					RenderBlock renderBlock(*this);
					golem::CriticalSectionWrapper csw(scene.getCS());
					demoRenderer.reset();
					data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(imageItem, item));
					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
					list.push_back(ptr);
				}

				// process images
				data::Transform* processTransform = is<data::Transform>(processHandler);
				Assert::valid(processTransform != nullptr, "Handler %s does not support Transform interface", processHandler->getID().c_str());

				data::Item::Ptr processItem = processTransform->transform(list);
				data::Item::Map::iterator processPtr;
				{
					RenderBlock renderBlock(*this);
					golem::CriticalSectionWrapper csw(scene.getCS());
					processPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(this->processItem, processItem));
					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, processPtr, to<Data>(dataCurrentPtr)->getView());
				}

				// compute grasps
				list.clear();
				list.push_back(modelPtr);
				list.push_back(processPtr);
				data::Transform* queryTransform = is<data::Transform>(queryHandler);
				Assert::valid(queryTransform != nullptr, "Handler %s does not support Transform interface", queryHandler->getID().c_str());

				data::Item::Ptr queryItem = queryTransform->transform(list);
				data::Item::Map::iterator queryPtr;
				{
					RenderBlock renderBlock(*this);
					golem::CriticalSectionWrapper csw(scene.getCS());
					queryPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(this->queryItem, queryItem));
					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, queryPtr, to<Data>(dataCurrentPtr)->getView());
				}

				// select grasp with approach trajectory
				data::Convert* queryConvert = is<data::Convert>(&*queryItem);
				Assert::valid(queryConvert != nullptr, "Item %s does not support Convert interface", this->queryItem.c_str());

				// select grasp with approach trajectory
				data::ContactQuery* contactQuery = is<data::ContactQuery>(&*queryItem);
				Assert::valid(contactQuery != nullptr, "Item %s does not support ContactQuery interface", this->queryItem.c_str());
				const golem::data::ContactQuery::Data& queryData = contactQuery->getData();
				
				// We need to get top grasps for every grasp type. 
				// We select 20 grasps per each grasp, and there are 8 different types of grasps currently. 
				int numberOfGrasps[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
				int graspCountLimit = 10; // Set to a high value so we can eliminate colliding grasps on the other side.
				int totalCounter = 0;

				// Go through each and every grasp and print grasp parameters.
				FILE * fp = fopen("temp.trj", "wb");
				fwrite(&totalCounter, 4, 1, fp);
				float tmp[7];
				float fingers[20];
				typedef std::chrono::high_resolution_clock Time;
				bool flag = true;

				int graspTypeCounters[10];
				for (int itr = 0; itr < 10; itr++)
				{
					graspTypeCounters[itr] = 0;
				}

				// Going through the list.
				for (const auto& config : queryData.configs) {

					// Get grasp.
					unsigned int graspType = config->space;
					float likelihoodVal = (float)(config->likelihood.value);

					if (graspType < 0 || graspType > 9)
						graspType = 0;
					graspTypeCounters[graspType]++;
		//			std::cout << "New grasp! Grasp type: " << graspType << " and likelihood: " << likelihoodVal << std::endl;
				}

				std::cout << "Grasp type counters:" << std::endl;
				for (int itr = 0; itr < 10; itr++)
				{
					std::cout << graspTypeCounters[itr] << " ";
				}
				std::cout << std::endl;

				// Going through the list.
				int i = -1;
				for (const auto& config : queryData.configs) {
					// Increase counter i
					i++;

					// Get grasp.
					golem::Manipulator::Waypoint::Seq path = config->path;
					unsigned int graspType = config->space;
					float likelihoodVal = (float)(config->likelihood.value);

					// Perform a very simple collision detection (of the approach, with respect to the table)
					Mat34 fr = path[0].frame.toMat34();
					Vec3 p = fr.p;

					if (p[2] < -0.3)
						continue;

					// Get grasp type, see if we've collected enough of this type. If not, add.
					if (numberOfGrasps[graspType] > graspCountLimit-1)
					{
						continue;
					}

					// This grasp has been selected! Go ahead and write it in a file.
					int pathSize = path.size();
					fwrite(&likelihoodVal, 4, 1, fp);
					fwrite(&graspType, 4, 1, fp);
					fwrite(&pathSize, 4, 1, fp);

					// Print joint angles.
					for (int j = 0; j < pathSize; j++) {
						Mat34 fr = path[j].frame.toMat34();
						Mat33 R = fr.R;
						Vec3 p = fr.p;

						RBCoord::Quat q;
						fr.R.toQuat(q);

						tmp[0] = (float)p[0], tmp[1] = (float)p[1], tmp[2] = (float)p[2];
						tmp[3] = (float)q.x, tmp[4] = (float)q.y, tmp[5] = (float)q.z, tmp[6] = (float)q.w;
						fwrite(tmp, 4, 7, fp);

						Real * data = path[j].config.data();
						for (int k = 7; k < 27; k++)
							fingers[k - 7] = (float)data[k];
						fwrite(fingers, 4, 20, fp);
					}


					std::cout << "Selected number " << numberOfGrasps[graspType] << " of grasp type " << graspType << std::endl;
					numberOfGrasps[graspType]++;
					totalCounter++;

					if (totalCounter >= graspCountLimit * 10)
						break;
				}
				
				// Update the number of trajectories.
				fseek(fp, 0, SEEK_SET);
				fwrite(&totalCounter, 4, 1, fp);
				fclose(fp);
				std::cout << "Number of Trajectories:" << totalCounter << std::endl;

				std::cout << tmpFileName << std::endl;
				// PROCESSING FINISHED. SEND FILE, WAIT FOR NEXT ROUND.
				char * fn = (char *)strrchr(tmpFileName, '\\');
				char * fn2 = (char *)strrchr(tmpFileName, '.');
				int l = fn2 - fn;
				char newFileName[100];
				strcpy(newFileName, DROPBOX_OUT_PATH);
				strncat(newFileName, fn + 1, l - 1);
				std::cout << " CREATING DIR: " << newFileName << std::endl;
				boost::filesystem::create_directory(newFileName);
				strcat(newFileName, "\\");
				strncat(newFileName, fn + 1, l);
				strcat(newFileName, "trj");
				std::cout << "Written file:" << newFileName << std::endl;
				boost::filesystem::rename("temp.trj", newFileName);
			}
			catch (const std::exception& e) { // reference to the base of a polymorphic object
				// Copy empty point cloud.
				std::cout << tmpFileName << std::endl;

				char * fn = (char *)strrchr(tmpFileName, '\\');
				char * fn2 = (char *)strrchr(tmpFileName, '.');
				int l = fn2 - fn;
				char newFileName[100];
				strcpy(newFileName, DROPBOX_OUT_PATH);
				strncat(newFileName, fn + 1, l - 1);
				std::cout << " CREATING DIR: " << newFileName << std::endl;
				boost::filesystem::create_directory(newFileName);
				strcat(newFileName, "\\");
				strncat(newFileName, fn + 1, l);
				strcat(newFileName, "trj");
				std::cout << "Written empty file:" << newFileName << std::endl;
				FILE * emptyFP = fopen(newFileName, "w");
				fclose(emptyFP);
			}
		}
	}
	));
}

//------------------------------------------------------------------------------

void golem::AppNNLearning::render() const {
	Player::render();
	demoRenderer.render();
}

//------------------------------------------------------------------------------

void golem::AppNNLearning::gotoPose(const Pose& pose) {
	// current state
	golem::Controller::State begin = WaypointCtrl::lookup(*controller).command;
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
	golem::Controller::State::Seq trajectory;
	findTrajectory(begin, &end, pose.position || pose.orientation ? &wpose : nullptr, pose.dt, trajectory);
	sendTrajectory(trajectory);
	// wait for end
	controller->waitForEnd();
	// sleep
	//begin = lookupState();
	//context.debug("STATE[2]: t=%f, (%f, %f, %f, %f, %f, %f, %f)\n", begin.t, begin.cpos.data()[0], begin.cpos.data()[1], begin.cpos.data()[2], begin.cpos.data()[3], begin.cpos.data()[4], begin.cpos.data()[5], begin.cpos.data()[6]);
	Sleep::msleep(SecToMSec(getPlanner().trajectoryIdleEnd));
	//begin = lookupState();
	//context.debug("STATE[3]: t=%f, (%f, %f, %f, %f, %f, %f, %f)\n", begin.t, begin.cpos.data()[0], begin.cpos.data()[1], begin.cpos.data()[2], begin.cpos.data()[3], begin.cpos.data()[4], begin.cpos.data()[5], begin.cpos.data()[6]);
}

//------------------------------------------------------------------------------

void golem::XMLData(golem::AppNNLearning::Pose &val, golem::XMLContext* xmlcontext, bool create) {
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

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return AppNNLearning::Desc().main(argc, argv);
}
