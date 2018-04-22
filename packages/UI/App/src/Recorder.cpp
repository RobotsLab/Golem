/** @file Recorder.cpp
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
#include <Golem/App/Recorder.h>
#include <Golem/Plugin/Point.h>
#include <opencv2/highgui/highgui.hpp>

using namespace golem;

//------------------------------------------------------------------------------

void golem::Recorder::Task::Desc::load(const golem::XMLContext* context) {
	try {
		golem::XMLData("snapshot", snapshot, const_cast<golem::XMLContext*>(context));
	}
	catch (const golem::MsgXMLParser&) {}
	try {
		golem::XMLData("sequence", sequence, const_cast<golem::XMLContext*>(context));
	}
	catch (const golem::MsgXMLParser&) {}
}

void golem::XMLData(golem::Recorder::Task::Desc::Map::value_type& val, golem::XMLContext* context, bool create) {
	golem::XMLData("id", const_cast<std::string&>(val.first), context);
	val.second.load(context);
}

golem::Recorder::Task::Task(const Desc& desc, Recorder& recorder, Sensor& sensor) : captureSnapshot(nullptr), captureSequence(nullptr) {
	// Snapshot handler
	data::Handler::Map::const_iterator snapshotPtr = recorder.handlerMap.find(sensor.getSnapshotHandler());
	if (snapshotPtr != recorder.handlerMap.end() && desc.snapshot) captureSnapshot = is<data::Capture>(snapshotPtr);
	// Sequence handler
	data::Handler::Map::const_iterator sequencePtr = recorder.handlerMap.find(sensor.getSequenceHandler());
	if (sequencePtr != recorder.handlerMap.end() && desc.sequence) captureSequence = is<data::Capture>(sequencePtr);
	// task
	function = [&] () {
		data::Capture* capture = recorder.recorderSequence ? captureSequence : captureSnapshot;
		// nothing to do
		if (!capture)
			return;
		try {
			// capture
			data::Item::Ptr item = capture->capture(sensor, [&] (const golem::TimeStamp* timeStamp) -> bool {
				// action (sequences only)
				if (timeStamp)
					return recorder.recorderStop == golem::SEC_TM_REAL_MAX || timeStamp->timeStamp < recorder.recorderStart + recorder.recorderStop;
				// sync start (sequences and snapshots)
				{
					golem::CriticalSectionWrapper csw(recorder.csRecorder);
					if (recorder.recorderCountStart  > 0)
						--recorder.recorderCountStart;
					if (recorder.recorderCountStart == 0) {// last in the queue signals all threads
						recorder.evRecorderStart.set(true);
						recorder.recorderStart = recorder.getContext().getTimer().elapsed();
					}
				}
				const bool start = recorder.evRecorderStart.wait();
				//recorder.getContext().verbose("Sensor %s: start\n", sensor.getName().c_str());
				return start;
			});
			// save data
			recorder.recordingSave(sensor, item);
		}
		catch (const Message& msg) {
			recorder.getContext().write(msg);
		}
		catch (const std::exception& ex) {
			recorder.getContext().error("Recorder::Task(): %s (Sensor %s)\n", ex.what(), sensor.getID().c_str());
		}
		catch (...) {
			recorder.getContext().error("Recorder::Task(): unknown exception (Sensor %s)\n", sensor.getID().c_str());
		}

		// sync stop
		{
			golem::CriticalSectionWrapper csw(recorder.csRecorder);
			if (recorder.recorderCountStop  > 0)
				--recorder.recorderCountStop;
			if (recorder.recorderCountStop == 0)
				recorder.evRecorderStop.set(true);
		}
		recorder.evRecorderStop.wait();
		//recorder.getContext().verbose("Sensor %s: stop\n", sensor.getName().c_str());
	};
}

size_t golem::Recorder::recordingSize(bool sequence) const {
	size_t count = 0;
	for (Task::Map::const_iterator i = recorderMap.begin(); i != recorderMap.end(); ++i)
		if (sequence ? i->second->captureSequence : i->second->captureSnapshot)
			++count;
	return count;
}

bool golem::Recorder::recordingStart(const std::string& data, const std::string& item, bool sequence) {
	// number of tasks
	recorderCount = recordingSize(sequence);
	if (recorderCount == 0)
		return false;

	// check if recorder is already running
	if (recordingActive())
		return false;

	{
		golem::CriticalSectionWrapper csw(csRecorder);
		// setup data
		recorderSequence = sequence;
		recorderData = data;
		recorderItem = item;
		// initialise start and stop
		recorderCountStop = recorderCountStart = recorderCount;
		recorderStop = recorderSequence ? golem::SEC_TM_REAL_MAX : golem::SEC_TM_REAL_ZERO;
		evRecorderStart.set(false);
		evRecorderStop.set(false);
	}

	// run tasks
	for (Task::Map::iterator i = recorderMap.begin(); i != recorderMap.end(); ++i)
		i->second->start();

	return true;
}

void golem::Recorder::recordingStop(golem::SecTmReal stop) {
	recorderStop = stop; // atomic
}

bool golem::Recorder::recordingActive() {
	golem::CriticalSectionWrapper csw(csRecorder);
	return recorderCountStart > 0 || recorderCountStop > 0;
}

bool golem::Recorder::recordingWaitToStart(golem::MSecTmU32 timeOut) {
	return evRecorderStart.wait(timeOut);
}

bool golem::Recorder::recordingWaitToStop(golem::MSecTmU32 timeOut) {
	return evRecorderStop.wait(timeOut);
}

void golem::Recorder::recordingSave(const Sensor& sensor, const data::Item::Ptr& item) {
	golem::CriticalSectionWrapper csw(scene.getCS());
	data::Data::Map::iterator data = dataMap.find(recorderData);
	if (data == dataMap.end())
		throw Message(Message::LEVEL_ERROR, "Recorder::recordingSave(): unable to find Data %s", recorderData.c_str());
	data->second->itemMap.insert(std::make_pair(recorderItem + makeString("%s%.3f%s%s", dataDesc->sepName.c_str(), recorderStart, dataDesc->sepName.c_str(), sensor.getID().c_str()), item));
}

//------------------------------------------------------------------------------

void golem::Recorder::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Manager::Desc::load(context, xmlcontext);

	golem::XMLContext* pxmlcontext = xmlcontext->getContextFirst("recorder");

	sensors.clear();
	try {
		golem::XMLData(sensors, sensors.max_size(), pxmlcontext, "sensor");
	}
	catch (const golem::MsgXMLParser&) {}

	try {
		golem::XMLData(recordingDescMap, recordingDescMap.max_size(), pxmlcontext->getContextFirst("recording"), "sensor");
	}
	catch (const golem::MsgXMLParser&) {}
	golem::XMLData("label", recordingLabel, pxmlcontext->getContextFirst("recording"));

	cloudAdjust.load(pxmlcontext->getContextFirst("cloud adjust"));
}

//------------------------------------------------------------------------------

golem::Recorder::Recorder(Scene &scene) : Manager(scene), currentCamera(nullptr), currentImageId(0) {
}

void golem::Recorder::create(const Desc& desc) {
	desc.assertValid(Assert::Context("Recorder::Desc."));

	// create object
	Manager::create(desc); // throws

	// sensors
	sensorMap.clear();
	for (golem::Library::Path::Seq::const_iterator i = desc.sensors.begin(); i != desc.sensors.end(); ++i) {
		Sensor::Desc::Ptr sensorDesc = golem::Library::Desc::loadLibrary<Sensor::Desc>(context, *i);
		
		sensorDesc->configQuery = [&] (U32 joint, Sensor::Config& config) { getPose(joint, config); };

		// Camera
		Camera::Desc* cameraDesc = is<Camera::Desc>(sensorDesc.get());
		if (cameraDesc) {
			for (CameraCalibration::Desc::Map::const_iterator j = cameraDesc->calibrationDescMap.begin(); j != cameraDesc->calibrationDescMap.end(); ++j) {
				j->second->drawImage = [&] (const IplImage* image, Camera* camera) { setImage(image, camera); };
			}
		}

		// finish
		try {
			// create sensor
			Sensor::Ptr sensor = sensorDesc->create(context);
			sensorMap.insert(std::make_pair(sensor->getID(), sensor));
		}
		catch (const Message& msg) {
			context.write(msg);
		}
		catch (const std::exception& ex) {
			context.write("%s\n", ex.what());
		}
	}
	
	// batch initialisation
	for (Sensor::Map::const_iterator i = sensorMap.begin(); i != sensorMap.end(); ++i)
		i->second->init(sensorMap);
	
	sensorCurrentPtr = sensorMap.begin();

	// recorder
	recordingDescMap = desc.recordingDescMap;
	recordingLabel = desc.recordingLabel;
	recorderSequence = true;
	recorderCountStop = recorderCountStart = 0;
	recorderStart = recorderStop = golem::SEC_TM_REAL_ZERO;
	for (Sensor::Map::const_iterator i = sensorMap.begin(); i != sensorMap.end(); ++i) {
		const Task::Desc::Map::const_iterator j = recordingDescMap.find(i->first);
		if (j != recordingDescMap.end()) {
			// setup recorder
			context.debug("Recorder::create(): Sensor %s\n", i->first.c_str());
			recorderMap.insert(std::make_pair(i->second.get(), Task::Ptr(new Task(j->second, *this, *i->second))));
			// register golem::UICapture
			golem::UICapture* capture = is<golem::UICapture>(i->second.get());
			if (capture) {
				context.debug("Recorder::create(): Sensor %s: registering golem::UICapture\n", i->first.c_str());
				universe.setCapture(capture);
			}
		}
	}

	cloudAdjust = desc.cloudAdjust;

	// top menu help accessible using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0310", "  <ctrl+t>                                current sensor video stream ON/OFF\n"));
	scene.getHelp().insert(Scene::StrMapVal("0311", "  <ctrl+s>                                capture snapshot\n"));
	scene.getHelp().insert(Scene::StrMapVal("0312", "  <ctrl+q>                                capture sequence start/stop\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F2", "  S                                       menu sensor\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F3", "  C                                       menu cloud\n"));

	// top menu control and commands
	menuCtrlMap.insert(std::make_pair("", [=] (MenuCmdMap& menuCmdMap, std::string& desc) {
		//desc = "Press a key to: access menu (D)ata/(I)tem/(S)ensor, change index of data({})/item([]), change view mode shared(`)/item(1)/label(2)...";
	}));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("S", [&] (MenuCmdMap& menuCmdMap, std::string& desc) {
		if (sensorMap.empty())
			throw Cancel("No available sensors");

		select(sensorCurrentPtr, sensorMap.begin(), sensorMap.end(), "Select Sensor:\n", [] (Sensor::Map::const_iterator ptr) -> const std::string& {
			return ptr->second->getID();
		});

		if (is<CameraDepthModel>(sensorCurrentPtr) || is<CameraImage>(sensorCurrentPtr)) {
			desc = "Press a key to: c(A)pture/(S)can, vie(W), (I)nfo...";
			menuCmdMap.erase("CV");
			menuCmdMap.erase("CC");
		}
		else if (is<Camera>(sensorCurrentPtr)) {
			// Camera
			desc = "Press a key to: c(A)pture/(S)can, vie(W), (V)ideo on/off, (C)alibrate, (I)nfo...";
		}
		else if (is<FT>(sensorCurrentPtr)) {
			// F/T sensor
			desc = "Press a key to: c(A)pture/(S)can, vie(W), (C)alibrate, (I)nfo...";
			menuCmdMap.erase("CV");
		}
		else
			throw Cancel("Unsupported sensor type");
	}));
	menuCmdMap.insert(std::make_pair("SI", [=]() {
		const golem::Mat34 m = sensorCurrentPtr->second->getFrame();
		context.write("Sensor: id=%s, type=%s, p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}, variable_mount=%s\n",
			sensorCurrentPtr->second->getID().c_str(),
			is<Camera>(sensorCurrentPtr) ? is<CameraDepth>(sensorCurrentPtr) ? "Depth Camera" : "RGB Camera" : "F/T",
			m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33,
			sensorCurrentPtr->second->hasVariableMounting() ? "YES" : "NO"
		);
	}));
	menuCmdMap.insert(std::make_pair("SA", [=]() {
		scanPose();
	}));
	menuCmdMap.insert(std::make_pair("SS", [=] () {
		ConfigMat34::Range range = selectPoseRange(poseMap);
		ConfigMat34::Map::const_iterator pose = range.first;
		size_t index = 0, size = std::distance(range.first, range.second);
		scanPose([&] () -> bool {
			context.write("Going to scan pose #%d/%d\n", index + 1, size);
			this->gotoPose(pose++->second);
			return ++index < size;
		});
	}));
	menuCmdMap.insert(std::make_pair("SW", [=] () {
		context.write("Sensor %s view point %s\n", sensorCurrentPtr->first.c_str(), setCamera(is<Camera>(sensorCurrentPtr), false) ? "ON" : "OFF");
	}));
	menuCmdMap.insert(std::make_pair("SV", [=] () {
		context.write("Sensor %s video stream %s\n", sensorCurrentPtr->first.c_str(), setCamera(is<Camera>(sensorCurrentPtr), true) ? "ON" : "OFF");
	}));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("SC", [&] (MenuCmdMap& menuCmdMap, std::string& desc) {
		if (is<Camera>(sensorCurrentPtr)) {
			// Camera
			const CameraCalibration::Map& calibrationMap = to<Camera>(sensorCurrentPtr)->getCalibrationMap();
			CameraCalibration::Map::const_iterator calibrationCurrentPtr = calibrationMap.find(to<Camera>(sensorCurrentPtr)->getCurrentCalibrationFile());

			select(calibrationCurrentPtr, calibrationMap.begin(), calibrationMap.end(), "Select calibration:\n", [](CameraCalibration::Map::const_iterator ptr) -> const std::string&{
				return ptr->first;
			});

			to<Camera>(sensorCurrentPtr)->setCurrentCalibrationFile(calibrationCurrentPtr->first);

			if (is<CameraDepth>(sensorCurrentPtr) != nullptr) {
				desc = "Press a key to: (I)ntrinsic/(E)xtrinsic calibration, (D)eformation estimation...";
				menuCmdMap.erase("SCM");
			}
			else {
				desc = "Press a key to: (I)ntrinsic/(E)xtrinsic calibration...";
				menuCmdMap.erase("SCD");
				menuCmdMap.erase("SCM");
			}
		}
		else {
			// F/T sensor
			desc = "Press a key to: (M)ass estimation...";
			menuCmdMap.erase("SCI");
			menuCmdMap.erase("SCE");
			menuCmdMap.erase("SCD");
		}
	}));
	menuCmdMap.insert(std::make_pair("SCI", [=] () {
		calibrateCamera(to<Camera>(sensorCurrentPtr), true);
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("SCE", [=] () {
		Camera* camera = to<Camera>(sensorCurrentPtr);
		data::Capture* capture = nullptr;
		ScopeGuard guard([&]() {
			if (capture) {
				UI::removeCallback(*this, is<data::Handler>(capture));
				UI::addCallback(*this, getCurrentHandler());
			}
		});
		
		if (option(1, "Use data::Capture interface: ", {"YES", "NO"}) == 0) {
			data::Handler::Map::const_iterator handlerSnapshotPtr = handlerMap.find(camera->getSnapshotHandler());
			if (handlerSnapshotPtr == handlerMap.end())
				throw Message(Message::LEVEL_ERROR, "Unknown snapshot handler %s", camera->getSnapshotHandler().c_str());
			capture = is<data::Capture>(handlerSnapshotPtr);
			if (!capture)
				throw Message(Message::LEVEL_ERROR, "Handler %s does not support Capture interface", camera->getSnapshotHandler().c_str());
			UI::removeCallback(*this, getCurrentHandler());
			UI::addCallback(*this, to<data::Handler>(handlerSnapshotPtr));
		}

		if (camera->hasVariableMounting()) {
			// for variable mounting (ConfigQuery != null), request config change - i.e. go through available calibration poses
			ConfigMat34::Range range = selectPoseRange(poseMap);
			ConfigMat34::Map::const_iterator pose = range.first;
			size_t index = 0, size = std::distance(range.first, range.second);
			auto configCommand = [&] () -> bool {
				if (index < size) {
					context.write("Going to calibration pose #%d/%d\n", index + 1, size);
					this->gotoPose(pose++->second);
				}
				return index++ < size;
			};
			capture ? camera->getCurrentCalibration()->calibrateExtrinsic(capture, this, configCommand) : calibrateCamera(camera, false, configCommand);
		}
		else
			// for fixed mounting ConfigQuery == null, i.e. the camera pose is fixed
			capture ? camera->getCurrentCalibration()->calibrateExtrinsic(capture, this, nullptr) : calibrateCamera(camera, false);

		// optionally update model poses of other sensors which has fixed mounting
		if (sensorMap.size() > 1 && camera->getCurrentCalibration()->getExtrinsicModel().estimatePose && option(0, "Reset model pose of other cameras: ", { "YES", "NO" }) == 0) {
			for (Sensor::Map::iterator i = sensorMap.begin(); i != sensorMap.end(); ++i)
				if (i != sensorCurrentPtr && !to<Camera>(i)->hasVariableMounting()) {
					context.write("Updating pose of %s...\n", to<Camera>(i)->getID().c_str());
					CameraCalibration::ExtrinsicModel model = to<Camera>(i)->getCurrentCalibration()->getExtrinsicModel();
					model.pose = camera->getCurrentCalibration()->getExtrinsicModel().pose; // update pose of the model, do not touch other parameters
					to<Camera>(i)->getCurrentCalibration()->setExtrinsicModel(model);
					to<Camera>(i)->getCurrentCalibration()->save();
				}
		}
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("SCD", [=] () {
		// model
		CameraCalibration::EquationSeq deformationMap;
		Mat34 referenceFrame;
		processItems([&] (const Data::Selection::List& list) {
			if (list.size() < 2)
				throw Cancel("At least two items have to be selected");
			for (Data::Selection::List::const_iterator i = list.begin(); i != list.end(); ++i) {
				{
					golem::CriticalSectionWrapper cswData(scene.getCS());
					Data::View::setItem(to<const Data>(dataCurrentPtr)->itemMap, i->getPtr(), to<Data>(dataCurrentPtr)->getView());
				}
				context.write("Deformation estimation: %s...\n", Data::toString(dataCurrentPtr).c_str());
				
				CameraCalibration::Equation equation;
				data::Model* model = is<data::Model>(i->getPtr()->second.get());
				model->model(equation.first, equation.second);

				if (i == list.begin()) {
					referenceFrame = equation.second;
					equation.second.setId();
				}
				else {
					equation.second.setInverse(equation.second);
					equation.second.multiply(referenceFrame, equation.second);
				}
				deformationMap.push_back(equation);
			}
		}, [=](const data::Item& item) -> bool { return is<data::Model>(&item) != nullptr; });
		// update
		to<Camera>(sensorCurrentPtr)->getCurrentCalibration()->enableDeformationMap(false);
		to<Camera>(sensorCurrentPtr)->getCurrentCalibration()->setDeformationMap(deformationMap);
		// done!
		createRender();
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("SCM", [=] () {
		// select calibration poses
		ConfigMat34::Range range = selectPoseRange(poseMap);
		ConfigMat34::Map::const_iterator pose = range.first;
		size_t index = 0, size = std::distance(range.first, range.second);
		// select sensors
		typedef std::set<FT*> FTSet;
		FTSet ftSet;
		// current one
		ftSet.insert(to<FT>(sensorCurrentPtr));
		// optionally - all the others
		if (option(0, "Calibrate all available F/T sensors: ", { "YES", "NO" }) == 0) {
			for (Sensor::Map::const_iterator i = sensorMap.begin(); i != sensorMap.end(); ++i) {
				FT* ft = is<FT>(i);
				if (ft) ftSet.insert(ft);
			}
		}
		// clear current calibration
		for (FTSet::iterator i = ftSet.begin(); i != ftSet.end(); ++i)
			(*i)->getCurrentCalibration()->clear();
		// go through selected poses and add calibration points
		for (; index < size; ++index) {
			context.write("Going to calibration pose #%d/%d\n", index + 1, size);
			this->gotoPose(pose++->second);
			// add calibration point
			for (FTSet::iterator i = ftSet.begin(); i != ftSet.end(); ++i)
				(*i)->getCurrentCalibration()->add();
		}
		// finish calibration
		for (FTSet::iterator i = ftSet.begin(); i != ftSet.end(); ++i) {
			(*i)->getCurrentCalibration()->calibrate();
			(*i)->getCurrentCalibration()->save();
		}
		context.write("Done!\n");
	}));

	// Point cloud manipulation
	menuCtrlMap.insert(std::make_pair("C", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		if (to<Data>(dataCurrentPtr)->itemMap.empty() || !is<data::Point3D>(to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true)))
			throw Message(Message::LEVEL_ERROR, "Item %s does not support Point3D interface", Data::toString(dataCurrentPtr).c_str());
		desc = "Press a key to: (T)ransform cloud...";
	}));
	menuCmdMap.insert(std::make_pair("CT", [&]() {
		data::Point3D* point3D = is<data::Point3D>(to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true)->second.get());
		if (!point3D)
			throw Message(Message::LEVEL_ERROR, "Item %s does not support Point3D interface", Data::toString(dataCurrentPtr).c_str());

		ScopeGuard cleanup([&]() {
			CriticalSectionWrapper csw(getCS());
			cloudRenderer.reset();
		});

		context.write("Press a key to: <Space> to attach frame, <Enter> to confirm transform, (%s/%s) to adjust position/orientation, (%s) to adjust increment...", cloudAdjust.linKeys.c_str(), cloudAdjust.angKeys.c_str(), cloudAdjust.incKeys.c_str());

		Vec3Seq points;
		Mat34 frame = Mat34::identity(), referenceFrameInv = Mat34::identity();
		for (bool draw = true, transform = false, done = false; !done;) {
			points.clear();

			// wait for key press
			const int key = waitKey(10);
			switch (key) {
			case 13: point3D->transform(frame * referenceFrameInv); done = true; break;
			case 32: referenceFrameInv.setInverse(frame); draw = true; transform = true; break;
			case 27: throw Cancel("Operation cancelled");
			default:
				if (cloudAdjust.adjustIncrement(key))
					context.write("\nInrement: position = %f [m], orientation = %f [deg]", cloudAdjust.getIncrement().lin, Math::radToDeg(cloudAdjust.getIncrement().ang));
				else if (cloudAdjust.adjustFrame(key, frame))
					draw = true;
			}

			if (!draw)
				continue;
			draw = false;

			if (transform) {
				const Mat34 trn = frame * referenceFrameInv;
				for (size_t i = 0; i < point3D->getSize(); ++i)
					points.push_back(trn * point3D->getPoint(i));
			}

			CriticalSectionWrapper csw(getCS());
			cloudRenderer.reset();
			cloudRenderer.addAxes3D(frame, cloudAdjust.frameSize);
			for (Vec3Seq::const_iterator i = points.begin(); i != points.end(); ++i)
				cloudRenderer.addPoint(*i, cloudAdjust.colourSolid);
		}

		// done!
		context.write("Done!\n");
		// refresh
		RenderBlock renderBlock(*this);
	}));
}

//------------------------------------------------------------------------------

void golem::Recorder::release() {
	universe.setCapture(nullptr); // reset capture
}

//------------------------------------------------------------------------------

void golem::Recorder::scanPose(ScanPoseCommand scanPoseCommand) {
	data::Handler::Map::const_iterator handlerSnapshotPtr = handlerMap.find(to<Sensor>(sensorCurrentPtr)->getSnapshotHandler());
	if (handlerSnapshotPtr == handlerMap.end())
		throw Message(Message::LEVEL_ERROR, "Unknown snapshot handler %s", to<Sensor>(sensorCurrentPtr)->getSnapshotHandler().c_str());
	data::Capture* capture = is<data::Capture>(handlerSnapshotPtr);
	if (!capture)
		throw Message(Message::LEVEL_ERROR, "Handler %s does not support Capture interface", to<Sensor>(sensorCurrentPtr)->getSnapshotHandler().c_str());

	readString("Enter label: ", dataItemLabel);

	Camera* camera = is<Camera>(sensorCurrentPtr);
	const bool isEnabledDeformationMap = camera && camera->getCurrentCalibration()->isEnabledDeformationMap();
	ScopeGuard guard([&] () { if (camera) camera->getCurrentCalibration()->enableDeformationMap(isEnabledDeformationMap); });
	if (camera && camera->getCurrentCalibration()->hasDeformationMap())
		camera->getCurrentCalibration()->enableDeformationMap(option(0, "Use deformation map: ", { "YES", "NO" }) == 0);

	for (bool stop = false; !stop; ) {
		stop = scanPoseCommand == nullptr || !scanPoseCommand();
		RenderBlock renderBlock(*this);
		{
			golem::CriticalSectionWrapper cswData(scene.getCS());
			const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(dataItemLabel, capture->capture(*to<Camera>(sensorCurrentPtr), [&] (const golem::TimeStamp*) -> bool { return true; })));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
	}

	context.write("Done!\n");
}

//------------------------------------------------------------------------------

void golem::Recorder::setImage(const IplImage* pImage, Camera* pCamera) const {
	scene.setGLMat(pCamera != nullptr);

	if (pCamera) {
		golem::OpenGL openGL;
		scene.getOpenGL(openGL);
		pCamera->getCurrentCalibration()->getGLMatrices(0.1f, 10.0f, openGL.glMatIntrinsic, openGL.glMatExtrinsic);
		scene.setOpenGL(openGL);
	}

	if (pImage) {
		golem::CriticalSectionWrapper csw(csSensor);
		currentImage.reserve(pImage);
		cvConvertImage(pImage, currentImage.image, CV_CVTIMG_SWAP_RB);
	}
	else {
		golem::CriticalSectionWrapper csw(csSensor);
		currentImage.release();
		currentImageId = 0;
		return;
	}
}

bool golem::Recorder::setCamera(Camera* camera, bool video) const {
	if (currentCamera)
		(void)currentCamera->set(Camera::CMD_STOP);

	{
		golem::CriticalSectionWrapper csw(csSensor);
		currentCamera = camera != nullptr && camera != currentCamera ? camera : nullptr;
	}

	setImage(nullptr, currentCamera);

	if (currentCamera && video)
		(void)currentCamera->set(Camera::CMD_VIDEO);

	return currentCamera != nullptr;
}

bool golem::Recorder::isCamera(const Camera* camera) const {
	return currentCamera == camera;
}

void golem::Recorder::resetCamera() const {
	if (this->currentCamera) {
		context.verbose("Camera %s video stream reset\n", currentCamera->getType().c_str());
		Camera* camera = this->currentCamera;
		setCamera(nullptr);
		setCamera(camera);
	}
}

void golem::Recorder::calibrateCamera(Camera* camera, bool bIntrinsic, CameraCalibration::ConfigCommand configCommand) {
	Camera* const thisCamera = currentCamera;
	setCamera(nullptr);
	const int width = scene.getUniverse().getWindowWidth();
	const int height = scene.getUniverse().getWindowHeight();
	if (camera->getCurrentCalibration()->isSetCameraRes())
		scene.getUniverse().setSize(camera->getProperty().width, camera->getProperty().height);
	ScopeGuard guard([&]() { if (camera->getCurrentCalibration()->isSetCameraRes()) scene.getUniverse().setSize(width, height); setCamera(thisCamera); });
	bIntrinsic ? camera->getCurrentCalibration()->calibrateIntrinsic(this) : camera->getCurrentCalibration()->calibrateExtrinsic(this, configCommand);
}

//------------------------------------------------------------------------------

void golem::Recorder::postprocess(golem::SecTmReal elapsedTime) {
	sensorRenderer.reset();
	for (Sensor::Map::const_iterator i = sensorMap.begin(); i != sensorMap.end(); ++i)
		i->second->draw(i->second->getAppearance(), sensorRenderer);
}

void golem::Recorder::render() const {
	Manager::render();
	sensorRenderer.render();
	cloudRenderer.render();
}

void golem::Recorder::customRender() const {
	Manager::customRender();

	golem::CriticalSectionWrapper csw(csSensor);

	if (currentCamera != nullptr && currentCamera->get() == Camera::CMD_VIDEO)
		(void)currentCamera->peek([&] (const Image::List& images) {
			if (images.empty()) return;
			if (currentImage.image != nullptr && currentImage.timeStamp >= images.back()->timeStamp) return;
			setImage(images.back()->image, currentCamera);
			currentImage.timeStamp = images.back()->timeStamp;
		});

	if (currentImage.image != nullptr)
		currentImage.draw(&currentImageId);
}

//------------------------------------------------------------------------------

void golem::Recorder::keyboardHandler(int key, int x, int y) {
	Manager::keyboardHandler(key, x, y);

	try {
		// video
		const bool videoOnOff = key == 20; // ctrl+t
		if (videoOnOff)
			if (is<Camera>(sensorCurrentPtr) && !is<CameraImage>(sensorCurrentPtr))
				context.write("Sensor %s video stream %s\n", sensorCurrentPtr->first.c_str(), setCamera(is<Camera>(sensorCurrentPtr)) ? "ON" : "OFF");
			else
				context.write("Sensor %s does not support video stream\n", sensorCurrentPtr->first.c_str());
		// recording
		const bool recSnapshot = key == 19; // ctrl+s
		const bool recSequence = key == 17; // ctrl+q
		if (recSnapshot || recSequence) {
			if (recorderMap.empty())
				context.info("Capture unavailable\n");
			else {
				std::string data;
				data = dataCurrentPtr->first;
				if (recSequence) {
					if (recordingActive()) {
						recordingStop(golem::SEC_TM_REAL_ZERO);
						context.info("Capture sequence stop\n");
					}
					else {
						if (recordingStart(data, recordingLabel, true))
							context.info("Capture sequence start\n");
					}
				}
				else {
					if (recordingStart(data, recordingLabel, false))
						context.info("Capture snapshot\n");
				}
			}
		}
	}
	catch (std::exception& ex) {
		context.write("%s\n", ex.what());
	}
}

//------------------------------------------------------------------------------
