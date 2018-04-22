/** @file AppPlacement.cpp
*
* Object placement demo
*
* @author	Marek Kopicki
* @author	Sebastian Zurek
*
* @copyright  Copyright (C) 2015 Marek Kopicki and Sebastian Zurek, University of Birmingham, UK
*
* @license  This file copy is licensed to you under the terms described in
*           the License.txt file included in this distribution.
*
*/

#include <Golem/App/Placement/Placement.h>
#include <Golem/Tools/Image.h>
#include <Golem/Data/Image/Image.h>
#include <Golem/Tools/Import.h>
#include <Golem/Plugin/PlannerI.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Math/Rand.h>

using namespace golem;

//-----------------------------------------------------------------------------

namespace {
std::string toXMLString(const golem::Mat34& m)
{
	char buf[BUFSIZ], *begin = buf, *const end = buf + sizeof(buf) - 1;
	golem::snprintf(begin, end,
			"m11=\"%f\" m12=\"%f\" m13=\"%f\" m21=\"%f\" m22=\"%f\" m23=\"%f\" m31=\"%f\" m32=\"%f\" m33=\"%f\" v1=\"%f\" v2=\"%f\" v3=\"%f\"",
			m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33, m.p.x, m.p.y, m.p.z);
	return std::string(buf);
}

std::string toXMLString(const golem::ConfigMat34& cfg, const bool shortFormat = false)
{
	std::ostringstream os;
	os.precision(6);
	const size_t n = shortFormat ? 7 : cfg.c.size();
	for (size_t i = 0; i < n; ++i)
	{
		os << (i == 0 ? "c" : " c") << i + 1 << "=\"" << cfg.c[i] << "\"";
	}

	return os.str();
}

class ForceEvent
{
public:
	ForceEvent(golem::FT* pFTSensor_, const golem::Twist& threshold_) :
		threshold(threshold_),
		bias(Vec3::zero(), Vec3::zero()),
		maxExcursion(Vec3::zero(), Vec3::zero()),
		//logFile("FT-debug.log", std::ios::out | std::ios::app),
		pFTSensor(pFTSensor_)
	{
		if (pFTSensor == nullptr)
			throw Message(Message::LEVEL_CRIT, "ForceEvent(): pFTSensor is nullptr");
	}

	void setBias()
	{
		golem::FT::Data ftdata;
		pFTSensor->read(ftdata, true);
		bias = ftdata.wrench;
	}

	bool detected(golem::Context* pContext = nullptr)
	{
		bool tripped = false;
		golem::FT::Data ftdata;
		pFTSensor->read(ftdata, true);
		golem::Twist current = ftdata.wrench;
		double currentFT[6], biasFT[6], thresholdFT[6], maxExcursionFT[6];
		current.get(currentFT);
		bias.get(biasFT);
		threshold.get(thresholdFT);
		maxExcursion.get(maxExcursionFT);
		for (size_t i = 0; i < 6; ++i)
		{
			const double excursion = abs(currentFT[i] - biasFT[i]);

			if (excursion > maxExcursionFT[i])
				maxExcursionFT[i] = excursion;

			if (excursion > thresholdFT[i])
			{
				tripped = true;
				if (pContext != nullptr)
					pContext->debug("Force event detected on axis %d: |%f - %f| > %f\n", i + 1, currentFT[i], biasFT[i], thresholdFT[i]);
				//break; // test on all axes
			}
		}
		maxExcursion.set(maxExcursionFT);
		return tripped;
	}

	void showMaxExcursion(golem::Context* pContext)
	{
		const golem::Twist& m = maxExcursion;
		pContext->debug("ForceEvent: max excursion: %g %g %g;  %g %g %g\n", m.v.x, m.v.y, m.v.z, m.w.x, m.w.y, m.w.z);
	}

	void showFT(golem::Context* pContext)
	{
		golem::FT::Data ft1, ft2;
		pFTSensor->read(ft1, false);
		pFTSensor->read(ft2, true);

		SecTmReal t1 = ft1.timeStamp;
		golem::Twist raw = ft1.wrench;
		SecTmReal t2 = ft2.timeStamp;
		golem::Twist in = ft2.wrench;

		pContext->debug("FT raw[%8.3f]: %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f; FT inertia[%8.3f]: %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n",
			t1, raw.v.x, raw.v.y, raw.v.z, raw.w.x, raw.w.y, raw.w.z,
			t2, in.v.x, in.v.y, in.v.z, in.w.x, in.w.y, in.w.z);
	}

public:
	golem::Twist threshold;
	golem::Twist bias;
	golem::Twist maxExcursion;

	//golem::FileStream logFile;

private:
	golem::FT* pFTSensor;

	ForceEvent();
};

}

//-----------------------------------------------------------------------------

const std::string AppPlacement::ID_ANY = "Any";

const std::string AppPlacement::Data::ModeName[MODE_LAST + 1] = {
	"Model",
	"Query",
	"Solution",
};

data::Data::Ptr AppPlacement::Data::Desc::create(golem::Context &context) const {
	golem::data::Data::Ptr data(new AppPlacement::Data(context));
	static_cast<AppPlacement::Data*>(data.get())->create(*this);
	return data;
}

AppPlacement::Data::Data(golem::Context &context) : golem::Player::Data(context), owner(nullptr) {
}

void AppPlacement::Data::create(const Desc& desc) {
	Player::Data::create(desc);

	modelVertices.clear();
	modelTriangles.clear();
	modelFrame.setId();

	queryVertices.clear();
	queryTriangles.clear();
	queryFrame.setId();

	indexType = 0;
	indexItem = 0;
	contactRelation = golem::Contact3D::RELATION_DFLT;

	indexDensity = 0;
	indexSolution = 0;

	mode = MODE_MODEL;
	queryShowDensities = false;
}

void AppPlacement::Data::setOwner(Manager* owner) {
	golem::Player::Data::setOwner(owner);
	this->owner = is<AppPlacement>(owner);
	if (!this->owner)
		throw Message(Message::LEVEL_CRIT, "golem::AppPlacement::Data::setOwner(): unknown data owner");
	
	// initialise owner-dependent data members
	dataName = this->owner->dataName;
	modelState.reset(new golem::Controller::State(this->owner->controller->createState()));
	this->owner->controller->setToDefault(*modelState);
}

AppPlacement::Data::Training::Map::iterator AppPlacement::Data::getTrainingItem() {
	Training::Map::iterator ptr = training.begin();
	U32 indexType = 0;
	for (; ptr != training.end() && ptr == --training.end() && indexType < this->indexType; ++indexType, ptr = training.upper_bound(ptr->first));
	this->indexType = indexType;
	U32 indexItem = 0;
	for (; ptr != training.end() && ptr->first == (--training.end())->first && indexItem < this->indexItem; ++indexItem, ++ptr);
	this->indexItem = indexItem;
	return ptr;
}

void AppPlacement::Data::setTrainingItem(Training::Map::const_iterator ptr) {
	U32 indexType = 0;
	for (Training::Map::const_iterator i = training.begin(); i != training.end(); ++indexType, i = training.upper_bound(i->first))
		if (i->first == ptr->first) {
			U32 indexItem = 0;
			for (Training::Map::const_iterator j = i; j != training.end(); ++indexItem, ++j)
				if (j == ptr) {
					this->indexType = indexType;
					this->indexItem = indexItem;
					return;
				}
		}
}

void AppPlacement::Data::createRender() {
	Player::Data::createRender();
	{
		golem::CriticalSectionWrapper csw(owner->scene.getCS());
		owner->modelRenderer.reset();
		// model/query
		const golem::Vec3Seq& vertices = mode == MODE_MODEL ? modelVertices : queryVertices;
		const golem::TriangleSeq& triangles = mode == MODE_MODEL ? modelTriangles : queryTriangles;
		const golem::Mat34& frame = mode == MODE_MODEL ? modelFrame : queryFrame;
		owner->modelRenderer.setColour(owner->modelColourSolid);
		owner->modelRenderer.addSolid(vertices.data(), (U32)vertices.size(), triangles.data(), (U32)triangles.size());
		owner->modelRenderer.setColour(owner->modelColourWire);
		owner->modelRenderer.addWire(vertices.data(), (U32)vertices.size(), triangles.data(), (U32)triangles.size());
		if (!vertices.empty() && !triangles.empty())
			owner->modelRenderer.addAxes3D(frame, Vec3(0.2));
		// training data
		if (mode == MODE_MODEL) {
			owner->contactAppearance.relation = contactRelation;
			Training::Map::iterator ptr = getTrainingItem();
			if (ptr != training.end()) {
				golem::Contact3D::draw(owner->contactAppearance, ptr->second.contacts.model, modelFrame, owner->modelRenderer);
				for (auto &i : ptr->second.points)
					owner->modelRenderer.addPoint(i, RGBA::BLACK);
			}
		}
		// query data
		else if (mode == MODE_QUERY) {
			if (!densities.empty()) {
				if (queryShowDensities) {
					const Density::Seq::iterator ptr = densities.begin() + indexDensity;
					for (golem::Query::Pose::Seq::const_iterator i = ptr->object.begin(); i != ptr->object.end(); ++i)
						owner->modelRenderer.addAxes(i->toMat34(), Vec3(0.005));
					for (golem::Query::Pose::Seq::const_iterator i = ptr->pose.begin(); i != ptr->pose.end(); ++i)
						owner->modelRenderer.addAxes(i->toMat34(), Vec3(0.02));
					
					//owner->modelRenderer.setPointSize(3);
					//const Real scaleObj = Real(1.0), scalePose = Real(0.1);
					//const RBDist deltaObj(owner->optimisation.saDelta.lin*scaleObj, owner->optimisation.saDelta.ang*scaleObj);
					//const RBDist deltaPose(owner->optimisation.saDelta.lin*scaleObj, owner->optimisation.saDelta.ang*scaleObj);

					//Rand rand(owner->context.getRandSeed());
					//const size_t samples = 100000;
					//for (size_t i = 0; i < samples; ++i) {
					//	RBCoord sample;
					//	Vec3 v;
					//	Quat q;

					//	const golem::Query::Pose::Seq::const_iterator obj = golem::Sample<golem::Real>::sample<golem::Ref1, golem::Query::Pose::Seq::const_iterator>(ptr->object, rand);
					//	v.next(rand); // |v|==1
					//	v.multiply(Math::abs(rand.nextGaussian<Real>(REAL_ZERO, deltaObj.lin*obj->stdDev.lin)), v);
					//	sample.p.add(obj->p, v);
					//	q.next(rand, obj->covInv.ang / Math::sqr(deltaObj.ang));
					//	sample.q.multiply(obj->q, q);
					//	owner->modelRenderer.addPoint(sample.p, RGBA(255, 0, 0, 50));

					//	const golem::Query::Pose::Seq::const_iterator pose = golem::Sample<golem::Real>::sample<golem::Ref1, golem::Query::Pose::Seq::const_iterator>(ptr->pose, rand);
					//	v.next(rand); // |v|==1
					//	v.multiply(Math::abs(rand.nextGaussian<Real>(REAL_ZERO, deltaPose.lin*pose->stdDev.lin)), v);
					//	sample.p.add(pose->p, v);
					//	q.next(rand, pose->covInv.ang / Math::sqr(deltaPose.ang));
					//	sample.q.multiply(pose->q, q);
					//	owner->modelRenderer.addPoint(sample.p, RGBA(0, 255, 0, 50));

					//	ptr->pose;
					//}
				}
			}
		}
		else if (mode == MODE_SOLUTION) {
			if (!solutions.empty()) {
				const Solution::Seq::iterator ptr = solutions.begin() + indexSolution;
				owner->modelRenderer.addAxes3D(ptr->pose.toMat34(), Vec3(0.1));
				if (ptr->path.size() > 0) owner->manipulatorAppearance.draw(*owner->manipulator, ptr->path[0], owner->modelRenderer);
				//if (ptr->path.size() > 1) owner->manipulatorAppearance.draw(*owner->manipulator, ptr->path[1], owner->modelRenderer);
				if (ptr->queryIndex < densities.size()) {
					// F = P * r
					Mat34 trn = ptr->path[0].frame.toMat34() * owner->manipulator->getReferenceFrame();
					for (auto &i : densities[ptr->queryIndex].points) {
						Vec3 p;
						trn.multiply(p, i);
						owner->modelRenderer.addPoint(p, RGBA::BLACK);
					}
				}
			}
		}
	}
}

void AppPlacement::Data::load(const std::string& prefix, const golem::XMLContext* xmlcontext, const data::Handler::Map& handlerMap) {
	data::Data::load(prefix, xmlcontext, handlerMap);

	try {
		dataName.clear();
		golem::XMLData("data_name", dataName, const_cast<golem::XMLContext*>(xmlcontext), false);
		if (dataName.length() > 0) {
			FileReadStream frs((prefix + sepName + dataName).c_str());
			
			modelVertices.clear();
			frs.read(modelVertices, modelVertices.end());
			modelTriangles.clear();
			frs.read(modelTriangles, modelTriangles.end());
			frs.read(modelFrame);
			frs.read(modelFrameOffset);

			queryVertices.clear();
			frs.read(queryVertices, queryVertices.end());
			queryTriangles.clear();
			frs.read(queryTriangles, queryTriangles.end());
			frs.read(queryFrame);

			modelState.reset(new golem::Controller::State(owner->controller->createState()));
			frs.read(*modelState);
			training.clear();
			frs.read(training, training.end(), std::make_pair(std::string(), Training(owner->controller->createState())));

			densities.clear();
			frs.read(densities, densities.end());
			solutions.clear();
			frs.read(solutions, solutions.end());
		}
	}
	catch (const std::exception&) {
	}

	Data::Cluster::setToDefault(this->owner->clusterMap, clusterCounter, training);
}

void AppPlacement::Data::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	data::Data::save(prefix, xmlcontext);

	if (dataName.length() > 0) {
		golem::XMLData("data_name", const_cast<std::string&>(dataName), xmlcontext, true);
		FileWriteStream fws((prefix + sepName + dataName).c_str());

		fws.write(modelVertices.begin(), modelVertices.end());
		fws.write(modelTriangles.begin(), modelTriangles.end());
		fws.write(modelFrame);
		fws.write(modelFrameOffset);

		fws.write(queryVertices.begin(), queryVertices.end());
		fws.write(queryTriangles.begin(), queryTriangles.end());
		fws.write(queryFrame);

		fws.write(*modelState);
		fws.write(training.begin(),training.end());

		fws.write(densities.begin(), densities.end());
		fws.write(solutions.begin(), solutions.end());
	}
}

//------------------------------------------------------------------------------

void AppPlacement::PoseDensity::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData(stdDev, xmlcontext->getContextFirst("std_dev"));
	stdDev.ang = Math::sqrt(REAL_ONE / stdDev.ang);	// stdDev ~ 1/cov

	golem::XMLData("kernels", kernels, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData(pathDist, xmlcontext->getContextFirst("path_dist"));
	pathDist.ang = Math::sqrt(REAL_ONE / pathDist.ang);	// stdDev ~ 1/cov

	golem::XMLData("std_dev", pathDistStdDev, xmlcontext->getContextFirst("path_dist"));
}

//------------------------------------------------------------------------------

void AppPlacement::Data::Cluster::setToDefault(const Map& map, Counter& counter, const Training::Map& training, bool ordered) {
	counter.clear();

	if (ordered) {
		for (Training::Map::const_iterator ptr = training.begin(); ptr != training.end(); ptr = training.upper_bound(ptr->first)) {
			const Map::const_iterator cluster = map.find(ptr->first);
			if (cluster == map.end())
				continue;
			const Data::Training::Range range = training.equal_range(ptr->first);
			const U32 size = std::distance(range.first, range.second);

			for (U32 index = 1; index < size; ++index)
				counter[cluster->second.slot].insert(index);
		}
	}
}

void AppPlacement::Data::Cluster::setOccupied(const Map& map, Counter& counter, const std::string& type, golem::U32 index, bool ordered) {
	// find cluster
	Data::Cluster::Map::const_iterator cluster = map.find(type);
	if (cluster == map.end())
		throw Message(Message::LEVEL_NOTICE, "AppPlacement::Data::Cluster::setOccupied(): %s type does not belong to any cluster", type.c_str());
	// add index
	counter[cluster->second.slot].insert(index);
	if (ordered)
		counter[cluster->second.slot].erase(index + 1);
}

bool AppPlacement::Data::Cluster::isOccupied(const Map& map, const Counter& counter, const std::string& type, golem::U32 index) {
	// find cluster
	Data::Cluster::Map::const_iterator cluster = map.find(type);
	if (cluster == map.end())
		throw Message(Message::LEVEL_NOTICE, "AppPlacement::Data::Cluster::isOccupied(): %s type does not belong to any cluster", type.c_str());
	// find slot
	Data::Cluster::Counter::const_iterator slot = counter.find(cluster->second.slot);
	// check if slot is available
	return slot != counter.end() && slot->second.find(index) != slot->second.end();
}

//------------------------------------------------------------------------------

void AppPlacement::Optimisation::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("runs", runs, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("steps", steps, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("sa_temp", saTemp, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("sa_delta_lin", saDelta.lin, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("sa_delta_ang", saDelta.ang, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("sa_energy", saEnergy, const_cast<golem::XMLContext*>(xmlcontext));
}

//------------------------------------------------------------------------------

void AppPlacement::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Player::Desc::load(context, xmlcontext);

	xmlcontext = xmlcontext->getContextFirst("demo");

	golem::XMLData("data_name", dataName, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("camera", modelCamera, xmlcontext->getContextFirst("model"));
	golem::XMLData("handler", modelHandler, xmlcontext->getContextFirst("model"));
	golem::XMLData("item", modelItem, xmlcontext->getContextFirst("model"));
	golem::XMLData("item_obj", modelItemObj, xmlcontext->getContextFirst("model"));

	modelScanPose.xmlData(xmlcontext->getContextFirst("model scan_pose"));
	golem::XMLData(modelColourSolid, xmlcontext->getContextFirst("model colour solid"));
	golem::XMLData(modelColourWire, xmlcontext->getContextFirst("model colour wire"));

	golem::XMLData("handler_trj", modelHandlerTrj, xmlcontext->getContextFirst("model"));
	golem::XMLData("item_trj", modelItemTrj, xmlcontext->getContextFirst("model"));

	golem::XMLData("camera", queryCamera, xmlcontext->getContextFirst("query"));
	golem::XMLData("handler", queryHandler, xmlcontext->getContextFirst("query"));
	golem::XMLData("item", queryItem, xmlcontext->getContextFirst("query"));
	golem::XMLData("item_obj", queryItemObj, xmlcontext->getContextFirst("query"));

	golem::XMLData("handler_trj", queryHandlerTrj, xmlcontext->getContextFirst("query"));
	golem::XMLData("item_trj", queryItemTrj, xmlcontext->getContextFirst("query"));

	golem::XMLData("sensor", graspSensorForce, xmlcontext->getContextFirst("grasp"));
	golem::XMLData(graspThresholdForce, xmlcontext->getContextFirst("grasp threshold"));
	golem::XMLData("event_time_wait", graspEventTimeWait, xmlcontext->getContextFirst("grasp"));
	golem::XMLData("close_duration", graspCloseDuration, xmlcontext->getContextFirst("grasp"));
	graspPoseOpen.xmlData(xmlcontext->getContextFirst("grasp pose_open"));
	graspPoseClosed.xmlData(xmlcontext->getContextFirst("grasp pose_closed"));

	golem::XMLData("camera", objectCamera, xmlcontext->getContextFirst("object"));
	golem::XMLData("handler_scan", objectHandlerScan, xmlcontext->getContextFirst("object"));
	golem::XMLData("handler", objectHandler, xmlcontext->getContextFirst("object"));
	golem::XMLData("item_scan", objectItemScan, xmlcontext->getContextFirst("object"));
	golem::XMLData("item", objectItem, xmlcontext->getContextFirst("object"));
	objectScanPoseSeq.clear();
	XMLData(objectScanPoseSeq, objectScanPoseSeq.max_size(), xmlcontext->getContextFirst("object"), "scan_pose");
	objectFrameAdjustment.load(xmlcontext->getContextFirst("object frame_adjustment"));

	modelDescMap.clear();
	golem::XMLData(modelDescMap, modelDescMap.max_size(), xmlcontext->getContextFirst("model"), "model", false);
	contactAppearance.load(xmlcontext->getContextFirst("model appearance"));

	queryDescMap.clear();
	golem::XMLData(queryDescMap, queryDescMap.max_size(), xmlcontext->getContextFirst("query"), "query", false);
	poseMap.clear();
	golem::XMLData(poseMap, poseMap.max_size(), xmlcontext->getContextFirst("query"), "query", false);

	optimisation.load(xmlcontext->getContextFirst("query optimisation"));

	manipulatorDesc->load(xmlcontext->getContextFirst("manipulator"));
	manipulatorAppearance.load(xmlcontext->getContextFirst("manipulator appearance"));
	golem::XMLData("item_trj", manipulatorItemTrj, xmlcontext->getContextFirst("manipulator"));
	golem::XMLData(manipulatorPoseStdDev, xmlcontext->getContextFirst("manipulator pose_stddev"), false);

	golem::XMLData("duration", manipulatorTrajectoryDuration, xmlcontext->getContextFirst("manipulator trajectory"));
	golem::XMLData(trajectoryThresholdForce, xmlcontext->getContextFirst("manipulator threshold"));

	golem::XMLData("release_fraction", withdrawReleaseFraction, xmlcontext->getContextFirst("manipulator withdraw_action"));
	golem::XMLData("lift_distance", withdrawLiftDistance, xmlcontext->getContextFirst("manipulator withdraw_action"));

	try {
		clusterMap.clear();
		golem::XMLData(clusterMap, clusterMap.max_size(), xmlcontext->getContextFirst("query"), "cluster_map", false);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}
}

//------------------------------------------------------------------------------

golem::AppPlacement::AppPlacement(Scene &scene) :
	Player(scene),
	modelCamera(nullptr), queryCamera(nullptr), modelHandler(nullptr), modelHandlerTrj(nullptr), queryHandler(nullptr), queryHandlerTrj(nullptr), graspSensorForce(nullptr), objectCamera(nullptr), objectHandlerScan(nullptr), objectHandler(nullptr)
{}

golem::AppPlacement::~AppPlacement() {
}

//------------------------------------------------------------------------------

golem::Camera* AppPlacement::getWristCamera(const bool dontThrow) const
{
	const std::string id("OpenNI+OpenNI");
	golem::Sensor::Map::const_iterator i = sensorMap.find(id);
	if (i == sensorMap.end())
	{
		if (dontThrow) return nullptr;
		context.write("%s was not found\n", id.c_str());
		throw Cancel("getWristCamera: wrist-mounted camera is not available");
	}

	golem::Camera* camera = golem::is<golem::Camera>(i);

	// want the wrist-mounted camera
	if (!camera->hasVariableMounting())
	{
		if (dontThrow) return nullptr;
		context.write("%s is a static camera\n", id.c_str());
		throw Cancel("getWristCamera: wrist-mounted camera is not available");
	}

	return camera;
}

golem::Mat34 AppPlacement::getWristPose() const
{
	const golem::U32 wristJoint = 6; // @@@
	golem::ConfigMat34 pose;
	getPose(wristJoint, pose);
	return pose.w;
}

golem::Controller::State::Seq AppPlacement::getTrajectoryFromPose(const golem::Mat34& w, const SecTmReal duration)
{
	const golem::Mat34 R = controller->getChains()[getPlanner().armInfo.getChains().begin()]->getReferencePose();
	const golem::Mat34 wR = w * R;

	golem::Controller::State begin = controller->createState();
	controller->lookupState(SEC_TM_REAL_MAX, begin);

	golem::Controller::State::Seq trajectory;
	const golem::RBDist err = findTrajectory(begin, nullptr, &wR, duration, trajectory);

	return trajectory;
}

golem::ConfigMat34 AppPlacement::getConfigFromPose(const golem::Mat34& w)
{
	golem::Controller::State::Seq trajectory = getTrajectoryFromPose(w, SEC_TM_REAL_ZERO);
	const golem::Controller::State& last = trajectory.back();
	ConfigMat34 cfg(RealSeq(61,0.0)); // !!! TODO use proper indices
	for (size_t i = 0; i < 7; ++i)
	{
		cfg.c[i] = last.cpos.data()[i];
	}
	return cfg;
}

golem::Controller::State AppPlacement::lookupStateArmCommandHand() const
{
	golem::Controller::State state = WaypointCtrl::lookup(*controller).state;	// current state
	golem::Controller::State cmdHand = WaypointCtrl::lookup(*controller).command;	// commanded state (wanted just for hand)
	state.cpos.set(getPlanner().handInfo.getJoints(), cmdHand.cpos); // only set cpos ???
	return state;
}

void AppPlacement::setHandConfig(Controller::State::Seq& trajectory, const golem::ConfigMat34& handPose)
{
	ConfigspaceCoord cposHand;
	cposHand.set(handPose.c.data(), handPose.c.data() + std::min(handPose.c.size(), (size_t)info.getJoints().size()));

	for (Controller::State::Seq::iterator i = trajectory.begin(); i != trajectory.end(); ++i)
	{
		Controller::State& state = *i;
		state.setToDefault(getPlanner().handInfo.getJoints().begin(), getPlanner().handInfo.getJoints().end());
		state.cpos.set(getPlanner().handInfo.getJoints(), cposHand);
	}
}

void AppPlacement::gotoWristPose(const golem::Mat34& w, const SecTmReal duration)
{
	golem::Controller::State::Seq trajectory = getTrajectoryFromPose(w, duration);
	sendTrajectory(trajectory);
	controller->waitForEnd();
	Sleep::msleep(SecToMSec(getPlanner().trajectoryIdleEnd));
}

void AppPlacement::gotoPose2(const ConfigMat34& pose, const SecTmReal duration)
{
	context.debug("AppPlacement::gotoPose2: %s\n", toXMLString(pose).c_str());

	// always start with hand in commanded config, not actual
	golem::Controller::State begin = lookupStateArmCommandHand();	// current state but commanded state for hand
	golem::Controller::State end = begin;
	end.cpos.set(pose.c.data(), pose.c.data() + std::min(pose.c.size(), (size_t)info.getJoints().size()));
	golem::Controller::State::Seq trajectory;
	findTrajectory(begin, &end, nullptr, duration, trajectory);
	sendTrajectory(trajectory);
	controller->waitForEnd();
	Sleep::msleep(SecToMSec(getPlanner().trajectoryIdleEnd));
}

void AppPlacement::releaseHand(const double openFraction, const SecTmReal duration)
{
	double f = 1.0 - openFraction;
	f = std::max(0.0, std::min(1.0, f));

	golem::Controller::State currentState = WaypointCtrl::lookup(*controller).state;
	ConfigMat34 openPose(RealSeq(61, 0.0)); // !!! TODO use proper indices
	for (size_t i = 0; i < openPose.c.size(); ++i)
		openPose.c[i] = currentState.cpos.data()[i];

	// TODO use proper indices - handInfo.getJoints()
	const size_t handIndexBegin = 7;
	const size_t handIndexEnd   = handIndexBegin + 5*4;
	for (size_t i = handIndexBegin; i < handIndexEnd; ++i)
		openPose.c[i] *= f;

	gotoPose2(openPose, duration);
}

void AppPlacement::closeHand(const double closeFraction, const SecTmReal duration)
{
	// @@@ HACK @@@

	const double f = std::max(0.0, std::min(1.0, closeFraction));

	// !!! TODO use proper indices
	ConfigMat34 pose(RealSeq(61, 0.0)), finalPose(RealSeq(61, 0.0));
	golem::Controller::State currentState = lookupStateArmCommandHand();
	for (size_t i = 0; i < pose.c.size(); ++i)
		pose.c[i] = currentState.cpos.data()[i];

	// TODO use proper indices - handInfo.getJoints()
	const size_t handIndexBegin = 7;
	const size_t handIndexEnd = handIndexBegin + 5 * 4;
	for (size_t i = handIndexBegin; i < handIndexEnd; i += 4)
	{
		finalPose.c[i + 1] = 0.85;
		finalPose.c[i + 2] = 0.2;
		finalPose.c[i + 3] = 0.2;
	}
	// ease off the thumb
	finalPose.c[handIndexBegin + 0] = 0.22;
	finalPose.c[handIndexBegin + 1] = 0.6;
	finalPose.c[handIndexBegin + 2] = 0.1;
	finalPose.c[handIndexBegin + 3] = 0.1;

	//context.debug("AppPlacement::closeHand: finalPose: %s\n", toXMLString(finalPose).c_str());

	for (size_t i = handIndexBegin; i < handIndexEnd; ++i)
		pose.c[i] += f * (finalPose.c[i] - pose.c[i]);

	gotoPose2(pose, duration);
}

void AppPlacement::liftWrist(const double verticalDistance, const SecTmReal duration)
{
	// vertically by verticalDistance; to hand zero config
	Mat34 pose = getWristPose();
	pose.p.z += std::max(0.0, verticalDistance);
	gotoWristPose(pose, duration);

	// TODO open hand while lifting
}

void AppPlacement::haltRobot()
{
	context.debug("STOPPING ROBOT!");

	Controller::State::Seq trj;

	Controller::State command = controller->createState();
	const Real t = controller->getCommandTime();
	controller->lookupCommand(t, command);
	command.cvel.setToDefault(info.getJoints());
	command.cacc.setToDefault(info.getJoints());
	command.t = t;
	trj.push_back(command);

	Controller::State command2 = command;
	command2.t += controller->getCycleDuration();
	trj.push_back(command2);

	controller->send(&trj.front(), &trj.back() + 1, true, false);
}

//------------------------------------------------------------------------------

void AppPlacement::nudgeWrist()
{
	auto showPose = [&](const std::string& description, const golem::Mat34& m) {
		context.write("%s: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", description.c_str(), m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
	};

	double s = 5; // 5cm step
	int dirCode;
	for (;;)
	{
		std::ostringstream os;
		os << "up:7 down:1 left:4 right:6 back:8 front:2 null:0 STEP(" << s << " cm):+/-";
		dirCode = option("7146820+-", os.str().c_str());
		if (dirCode == '+')
		{
			s *= 2;
			if (s > 20.0) s = 20.0;
			continue;
		}
		if (dirCode == '-')
		{
			s /= 2;
			continue;
		}
		break;
	}
	s /= 100.0; // cm -> m

	golem::Vec3 nudge(golem::Vec3::zero());
	switch (dirCode)
	{
	case '7': // up
		nudge.z = s;
		break;
	case '1': // down
		nudge.z = -s;
		break;
	case '4': // left
		nudge.y = -s;
		break;
	case '6': // right
		nudge.y = s;
		break;
	case '8': // back
		nudge.x = -s;
		break;
	case '2': // front
		nudge.x = s;
		break;
	};

	golem::Mat34 pose;

	golem::Camera* camera = getWristCamera(true);
	if (camera != nullptr)
	{
		pose = camera->getFrame();
		showPose("camera before", pose);
	}

	pose = getWristPose();
	showPose("before", pose);

	pose.p = pose.p + nudge;
	showPose("commanded", pose);

	gotoWristPose(pose);

	showPose("final wrist", getWristPose());
	if (camera != nullptr)
	{
		pose = camera->getFrame();
		showPose("final camera", pose);
	}

	golem::ConfigMat34 cp;
	getPose(0, cp);
	context.debug("%s\n", toXMLString(cp, true).c_str());
}

void AppPlacement::rotateObjectInHand()
{
	auto showPose = [&](const std::string& description, const golem::Mat34& m) {
		context.write("%s: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", description.c_str(), m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
	};

	const double maxstep = 45.0;
	double angstep = 30.0;
	int rotType;
	for (;;)
	{
		std::ostringstream os;
		os << "rotation: Clockwise or Anticlockwise screw, Up or Down, Left or Right; STEP(" << angstep << " deg):+/-";
		rotType = option("CAUDLR+-", os.str().c_str());
		if (rotType == '+')
		{
			angstep *= 2;
			if (angstep > maxstep) angstep = maxstep;
			continue;
		}
		if (rotType == '-')
		{
			angstep /= 2;
			continue;
		}
		break;
	}

	golem::Vec3 rotAxis(golem::Vec3::zero());

	switch (rotType)
	{
	case 'C':
		rotAxis.z = 1.0;
		break;
	case 'A':
		rotAxis.z = 1.0;
		angstep = -angstep;
		break;
	case 'U':
		rotAxis.y = 1.0;
		break;
	case 'D':
		rotAxis.y = 1.0;
		angstep = -angstep;
		break;
	case 'L':
		rotAxis.x = 1.0;
		break;
	case 'R':
		rotAxis.x = 1.0;
		angstep = -angstep;
		break;
	}

	golem::Mat33 rot(golem::Mat33(angstep / 180.0 * golem::REAL_PI, rotAxis));

	golem::Mat34 pose = getWristPose();
	showPose("before", pose);

	pose.R = pose.R * rot;
	showPose("commanded", pose);

	gotoWristPose(pose);
	showPose("final wrist", getWristPose());

	golem::ConfigMat34 cp;
	getPose(0, cp);
	context.debug("%s\n", toXMLString(cp, true).c_str());

	//recordingStart(dataCurrentPtr->first, recordingLabel, false);
	//context.write("taken snapshot\n");
}

//------------------------------------------------------------------------------

void golem::AppPlacement::create(const Desc& desc) {
	desc.assertValid(Assert::Context("golem::AppPlacement::Desc."));

	// create object
	Player::create(desc); // throws

	dataName = desc.dataName;

	golem::Sensor::Map::const_iterator modelCameraPtr = sensorMap.find(desc.modelCamera);
	modelCamera = modelCameraPtr != sensorMap.end() ? is<Camera>(modelCameraPtr->second.get()) : nullptr;
	if (!modelCamera)
		throw Message(Message::LEVEL_CRIT, "golem::AppPlacement::create(): unknown model pose estimation camera: %s", desc.modelCamera.c_str());
	golem::data::Handler::Map::const_iterator modelHandlerPtr = handlerMap.find(desc.modelHandler);
	modelHandler = modelHandlerPtr != handlerMap.end() ? modelHandlerPtr->second.get() : nullptr;
	if (!modelHandler)
		throw Message(Message::LEVEL_CRIT, "golem::AppPlacement::create(): unknown model data handler: %s", desc.modelHandler.c_str());
	modelItem = desc.modelItem;
	modelItemObj = desc.modelItemObj;

	modelScanPose = desc.modelScanPose;
	modelColourSolid = desc.modelColourSolid;
	modelColourWire = desc.modelColourWire;

	golem::data::Handler::Map::const_iterator modelHandlerTrjPtr = handlerMap.find(desc.modelHandlerTrj);
	modelHandlerTrj = modelHandlerTrjPtr != handlerMap.end() ? modelHandlerTrjPtr->second.get() : nullptr;
	if (!modelHandlerTrj)
		throw Message(Message::LEVEL_CRIT, "golem::AppPlacement::create(): unknown model trajectory handler: %s", desc.modelHandlerTrj.c_str());
	modelItemTrj = desc.modelItemTrj;

	golem::Sensor::Map::const_iterator queryCameraPtr = sensorMap.find(desc.queryCamera);
	queryCamera = queryCameraPtr != sensorMap.end() ? is<Camera>(queryCameraPtr->second.get()) : nullptr;
	if (!queryCamera)
		throw Message(Message::LEVEL_CRIT, "golem::AppPlacement::create(): unknown query pose estimation camera: %s", desc.queryCamera.c_str());
	golem::data::Handler::Map::const_iterator queryHandlerPtr = handlerMap.find(desc.queryHandler);
	queryHandler = queryHandlerPtr != handlerMap.end() ? queryHandlerPtr->second.get() : nullptr;
	if (!queryHandler)
		throw Message(Message::LEVEL_CRIT, "golem::AppPlacement::create(): unknown query data handler: %s", desc.queryHandler.c_str());
	queryItem = desc.queryItem;
	queryItemObj = desc.queryItemObj;

	golem::data::Handler::Map::const_iterator queryHandlerTrjPtr = handlerMap.find(desc.queryHandlerTrj);
	queryHandlerTrj = queryHandlerTrjPtr != handlerMap.end() ? queryHandlerTrjPtr->second.get() : nullptr;
	if (!queryHandlerTrj)
		throw Message(Message::LEVEL_CRIT, "golem::AppPlacement::create(): unknown query trajectory handler: %s", desc.queryHandlerTrj.c_str());
	queryItemTrj = desc.queryItemTrj;

	golem::Sensor::Map::const_iterator graspSensorForcePtr = sensorMap.find(desc.graspSensorForce);
	graspSensorForce = graspSensorForcePtr != sensorMap.end() ? is<FT>(graspSensorForcePtr->second.get()) : nullptr;
	if (!graspSensorForce)
		throw Message(Message::LEVEL_CRIT, "golem::AppPlacement::create(): unknown grasp F/T sensor: %s", desc.graspSensorForce.c_str());
	graspThresholdForce = desc.graspThresholdForce;
	graspEventTimeWait = desc.graspEventTimeWait;
	graspCloseDuration = desc.graspCloseDuration;
	graspPoseOpen = desc.graspPoseOpen;
	graspPoseClosed = desc.graspPoseClosed;

	golem::Sensor::Map::const_iterator objectCameraPtr = sensorMap.find(desc.objectCamera);
	objectCamera = objectCameraPtr != sensorMap.end() ? is<Camera>(objectCameraPtr->second.get()) : nullptr;
	if (!objectCamera)
		throw Message(Message::LEVEL_CRIT, "golem::AppPlacement::create(): unknown object capture camera: %s", desc.objectCamera.c_str());
	golem::data::Handler::Map::const_iterator objectHandlerScanPtr = handlerMap.find(desc.objectHandlerScan);
	objectHandlerScan = objectHandlerScanPtr != handlerMap.end() ? objectHandlerScanPtr->second.get() : nullptr;
	if (!objectHandlerScan)
		throw Message(Message::LEVEL_CRIT, "golem::AppPlacement::create(): unknown object (scan) data handler: %s", desc.objectHandlerScan.c_str());
	golem::data::Handler::Map::const_iterator objectHandlerPtr = handlerMap.find(desc.objectHandler);
	objectHandler = objectHandlerPtr != handlerMap.end() ? objectHandlerPtr->second.get() : nullptr;
	if (!objectHandler)
		throw Message(Message::LEVEL_CRIT, "golem::AppPlacement::create(): unknown object (process) data handler: %s", desc.objectHandler.c_str());
	objectItemScan = desc.objectItemScan;
	objectItem = desc.objectItem;
	objectScanPoseSeq = desc.objectScanPoseSeq;
	objectFrameAdjustment = desc.objectFrameAdjustment;

	// models
	modelMap.clear();
	for (Model::Desc::Map::const_iterator i = desc.modelDescMap.begin(); i != desc.modelDescMap.end(); ++i)
		modelMap.insert(std::make_pair(i->first, i->second->create(context, i->first)));
	contactAppearance = desc.contactAppearance;

	// query densities
	queryMap.clear();
	for (Query::Desc::Map::const_iterator i = desc.queryDescMap.begin(); i != desc.queryDescMap.end(); ++i)
		queryMap.insert(std::make_pair(i->first, i->second->create(context, i->first)));
	poseMap = desc.poseMap;

	optimisation = desc.optimisation;

	// manipulator
	manipulator = desc.manipulatorDesc->create(*getPlanner().planner, getPlanner().controllerIDSeq);
	manipulatorAppearance = desc.manipulatorAppearance;
	manipulatorItemTrj = desc.manipulatorItemTrj;

	poseCovInv.lin = REAL_ONE / (poseCov.lin = Math::sqr(desc.manipulatorPoseStdDev.lin));
	poseCovInv.ang = REAL_ONE / (poseCov.ang = Math::sqr(desc.manipulatorPoseStdDev.ang));
	poseDistanceMax = Math::sqr(desc.manipulatorPoseStdDevMax);

	manipulatorTrajectoryDuration = desc.manipulatorTrajectoryDuration;
	trajectoryThresholdForce = desc.trajectoryThresholdForce;

	withdrawReleaseFraction = desc.withdrawReleaseFraction;
	withdrawLiftDistance = desc.withdrawLiftDistance;

	clusterMap = desc.clusterMap;

	////////////////////////////////////////////////////////////////////////////
	//                            START OF MENUS                              // 
	////////////////////////////////////////////////////////////////////////////


	// top menu help using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("0F51", "  R                                       menu run\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F52", "  Z                                       menu SZ Tests\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F53", "  <Tab>                                   Query/Model mode switch\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F54", "  ()                                      Training item selection\n"));

	menuCmdMap.insert(std::make_pair("\t", [=]() {
		// set mode
		to<Data>(dataCurrentPtr)->mode = Data::Mode(to<Data>(dataCurrentPtr)->mode >= Data::MODE_LAST ? (U32)Data::MODE_FIRST : U32(to<Data>(dataCurrentPtr)->mode) + 1);
		context.write("%s mode\n", Data::ModeName[to<Data>(dataCurrentPtr)->mode].c_str());
		createRender();
	}));
	itemSelect = [&](ItemSelectFunc itemSelectFunc) {
		if (to<Data>(dataCurrentPtr)->training.empty())
			throw Cancel("No training data");
		Data::Training::Map::iterator ptr = to<Data>(dataCurrentPtr)->getTrainingItem();
		itemSelectFunc(to<Data>(dataCurrentPtr)->training, ptr);
		to<Data>(dataCurrentPtr)->setTrainingItem(ptr);
		context.write("Model type %s/%u, Item #%u\n", ptr->first.c_str(), to<Data>(dataCurrentPtr)->indexType + 1, to<Data>(dataCurrentPtr)->indexItem + 1);
	};
	menuCmdMap.insert(std::make_pair("(", [&]() {
		if (to<Data>(dataCurrentPtr)->mode == Data::MODE_MODEL)
			itemSelect([] (Data::Training::Map& map, Data::Training::Map::iterator& ptr) {
				if (ptr == map.begin()) ptr = --map.end(); else --ptr;
			});
		else if (to<Data>(dataCurrentPtr)->mode == Data::MODE_QUERY) {
			if (to<Data>(dataCurrentPtr)->densities.empty())
				throw Cancel("No query densities created");
			to<Data>(dataCurrentPtr)->indexDensity = to<Data>(dataCurrentPtr)->indexDensity <= 0 ? to<Data>(dataCurrentPtr)->densities.size() - 1 : to<Data>(dataCurrentPtr)->indexDensity - 1;
			context.write("Query type %s, Item #%u\n", to<Data>(dataCurrentPtr)->densities[to<Data>(dataCurrentPtr)->indexDensity].type.c_str(), to<Data>(dataCurrentPtr)->indexDensity + 1);
		}
		else if (to<Data>(dataCurrentPtr)->mode == Data::MODE_SOLUTION) {
			if (to<Data>(dataCurrentPtr)->solutions.empty())
				throw Cancel("No solutions created");
			to<Data>(dataCurrentPtr)->indexSolution = to<Data>(dataCurrentPtr)->indexSolution <= 0 ? to<Data>(dataCurrentPtr)->solutions.size() - 1 : to<Data>(dataCurrentPtr)->indexSolution - 1;
			const Data::Solution& solution = to<Data>(dataCurrentPtr)->solutions[to<Data>(dataCurrentPtr)->indexSolution];
			context.write("Solution type %s (%u/%u), Item #%u, Likelihood_{total, contact, pose, collision}={%.6e, %.6e, %.6e, %.6e}\n", solution.type.c_str(), solution.queryIndex + 1, to<Data>(dataCurrentPtr)->densities.size(), to<Data>(dataCurrentPtr)->indexSolution + 1, solution.likelihood.likelihood, solution.likelihood.contact, solution.likelihood.pose, solution.likelihood.collision);
		}
		createRender();
	}));
	menuCmdMap.insert(std::make_pair(")", [&]() {
		if (to<Data>(dataCurrentPtr)->mode == Data::MODE_MODEL)
			itemSelect([](Data::Training::Map& map, Data::Training::Map::iterator& ptr) {
				if (ptr == --map.end()) ptr = map.begin(); else ++ptr;
			});
		else if (to<Data>(dataCurrentPtr)->mode == Data::MODE_QUERY) {
			if (to<Data>(dataCurrentPtr)->densities.empty())
				throw Cancel("No query densities created");
			to<Data>(dataCurrentPtr)->indexDensity = to<Data>(dataCurrentPtr)->indexDensity < to<Data>(dataCurrentPtr)->densities.size() - 1 ? to<Data>(dataCurrentPtr)->indexDensity + 1 : 0;
			context.write("Query type %s, Item #%u\n", to<Data>(dataCurrentPtr)->densities[to<Data>(dataCurrentPtr)->indexDensity].type.c_str(), to<Data>(dataCurrentPtr)->indexDensity + 1);
		}
		else if (to<Data>(dataCurrentPtr)->mode == Data::MODE_SOLUTION) {
			if (to<Data>(dataCurrentPtr)->solutions.empty())
				throw Cancel("No solutions created");
			to<Data>(dataCurrentPtr)->indexSolution = to<Data>(dataCurrentPtr)->indexSolution < to<Data>(dataCurrentPtr)->solutions.size() - 1 ? to<Data>(dataCurrentPtr)->indexSolution + 1 : 0;
			const Data::Solution& solution = to<Data>(dataCurrentPtr)->solutions[to<Data>(dataCurrentPtr)->indexSolution];
			context.write("Solution type %s (%u/%u), Item #%u, Likelihood_{total, contact, pose, collision}={%.6e, %.6e, %.6e, %.6e}\n", solution.type.c_str(), solution.queryIndex + 1, to<Data>(dataCurrentPtr)->densities.size(), to<Data>(dataCurrentPtr)->indexSolution + 1, solution.likelihood.likelihood, solution.likelihood.contact, solution.likelihood.pose, solution.likelihood.collision);
		}
		createRender();
	}));
	menuCmdMap.insert(std::make_pair("0", [=]() {
		// set mode
		to<Data>(dataCurrentPtr)->queryShowDensities = !to<Data>(dataCurrentPtr)->queryShowDensities;
		context.write("query density %s\n", to<Data>(dataCurrentPtr)->queryShowDensities ? "ON" : "OFF");
		createRender();
	}));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("R", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: (E)stimate pose, (C)apture object, (M)odel, (Q)uery, (T)rajectory, run (D)emo ...";
	}));

	// model pose estimation
	menuCtrlMap.insert(std::make_pair("RE", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: (M)odel/(Q)ery estimation...";
	}));
	menuCmdMap.insert(std::make_pair("REM", [=]() {
		// estimate
		(void)estimatePose(Data::MODE_MODEL);
		// finish
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("REQ", [=]() {
		// estimate
		(void)estimatePose(Data::MODE_QUERY);
		// finish
		context.write("Done!\n");
	}));

	// model attachement
	menuCtrlMap.insert(std::make_pair("RC", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: (C)amera/(L)oad/(I)mport...";
	}));
	menuCmdMap.insert(std::make_pair("RCC", [=]() {
		// grasp and scan object
		golem::data::Item::Map::iterator ptr = objectGraspAndCapture();
		// compute features and add to data bundle
		(void)objectProcess(ptr);
		// finish
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("RCL", [=]() {
		// cast to data::Import
		data::Import* import = is<data::Import>(objectHandlerScan);
		if (!import)
			throw Cancel("Object handler does not implement data::Import");

		// replace current handlers
		UI::removeCallback(*this, getCurrentHandler());
		UI::addCallback(*this, import);

		// load/import item
		readPath("Enter file path: ", dataImportPath, import->getImportFileTypes());
		data::Item::Ptr item = import->import(dataImportPath);

		ScopeGuard guard([&]() { golem::CriticalSectionWrapper csw(scene.getCS()); objectRenderer.reset(); });
		RenderBlock renderBlock(*this);

		// compute reference frame and adjust object frame
		data::Point3D* point = is<data::Point3D>(item.get());
		if (!point)
			throw Cancel("Object handler does not implement data::Point3D");
		Vec3Seq points, pointsTrn;
		for (size_t i = 0; i < point->getSize(); ++i)
			points.push_back(point->getPoint(i));
		pointsTrn.resize(points.size());
		Mat34 frame = RBPose::createFrame(points), frameInv;
		frameInv.setInverse(frame);
		context.write("Press a key to: (%s/%s) to adjust position/orientation, (%s) to adjust increment, finish <Enter>...\n",
			objectFrameAdjustment.linKeys.c_str(), objectFrameAdjustment.angKeys.c_str(), objectFrameAdjustment.incKeys.c_str());
		for (bool finish = false; !finish;) {
			const Mat34 trn = frame * frameInv; // frame = trn * frameInit, trn = frame * frameInit^-1
			for (size_t i = 0; i < points.size(); ++i)
				trn.multiply(pointsTrn[i], points[i]);
			{
				golem::CriticalSectionWrapper csw(scene.getCS());
				objectRenderer.reset();
				for (size_t i = 0; i < pointsTrn.size(); ++i)
					objectRenderer.addPoint(pointsTrn[i], objectFrameAdjustment.colourSolid);
				objectRenderer.addAxes3D(frame, objectFrameAdjustment.frameSize);
			}
			const int key = waitKey();
			switch (key) {
			case 27: throw Cancel("Cancelled");
			case 13: finish = true; break;
			default:
				if (objectFrameAdjustment.adjustIncrement(key))
					context.write("Inrement: position = %f [m], orientation = %f [deg]\n", objectFrameAdjustment.getIncrement().lin, Math::radToDeg(objectFrameAdjustment.getIncrement().ang));
				(void)objectFrameAdjustment.adjustFrame(key, frame);
			}
		}
		// transform
		point->transform(frame * frameInv);

		// add item to data bundle
		data::Item::Map::iterator ptr;
		{
			golem::CriticalSectionWrapper cswData(scene.getCS());
			to<Data>(dataCurrentPtr)->itemMap.erase(objectItemScan);
			ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(objectItemScan, item));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}

		// compute features and add to data bundle
		(void)objectProcess(ptr);

		// finish
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("RCI", [=]() {
		// target data
		const Data::Map::iterator targetDataPtr = dataCurrentPtr;

		// data to import
		data::Item::Ptr ptrObject;
		golem::Controller::State::Ptr ptrState;

		// create menu
		bool stop = false;
		MenuCtrlMap menuCtrlMap;
		menuCtrlMap.insert(std::make_pair("", [&](MenuCmdMap& menuCmdMap, std::string& desc) {}));
		MenuCmdMap menuCmdMap;
		menuCmdMap.insert(std::make_pair("\x0D", [&]() {
			if (targetDataPtr == dataCurrentPtr)
				context.write("Select another data bundle\n");
			else {
				data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(objectItem);
				if (ptr == to<Data>(dataCurrentPtr)->itemMap.end()) {
					context.write("Object item is not set\n");
					return;
				}
				if (to<Data>(dataCurrentPtr)->modelState == nullptr) {
					context.write("Model state is not set\n");
					return;
				}
				ptrObject = ptr->second->clone();
				ptrState = to<Data>(dataCurrentPtr)->modelState;
				stop = true;
			}
		}));
		menuCmdMap.insert(std::make_pair("{", [&]() { getMenuCmdMap()["{"](); }));
		menuCmdMap.insert(std::make_pair("}", [&]() { getMenuCmdMap()["}"](); }));

		// select
		context.write("Press a key to: accept(<Enter>), select index of data({}) bundle...\n");
		while (!stop) {
			U32 menuLevel = 0;
			golem::Menu::menu(menuCtrlMap, menuCmdMap, menuLevel);
		}

		setCurrentDataPtr(targetDataPtr);
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(scene.getCS());
			to<Data>(targetDataPtr)->itemMap.erase(objectItem);
			data::Item::Map::iterator ptr = to<Data>(targetDataPtr)->itemMap.insert(to<Data>(targetDataPtr)->itemMap.end(), data::Item::Map::value_type(objectItem, ptrObject));
			Data::View::setItem(to<Data>(targetDataPtr)->itemMap, ptr, to<Data>(targetDataPtr)->getView());
		}

		// go to model robot pose
		gotoConfig(*ptrState);

		// finish
		context.write("Done!\n");
	}));

	// model operations
	menuCtrlMap.insert(std::make_pair("RM", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		// set mode
		to<Data>(dataCurrentPtr)->mode = Data::MODE_MODEL;
		createRender();

		desc = "Press a key to: (S)et/(G)oto initial object model pose, (I)mport/(A)dd/(R)emove training data...";
	}));
	menuCmdMap.insert(std::make_pair("RMS", [=]() {
		// load object item
		data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(objectItem);
		if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
			throw Cancel("Object item has not been created");
		// clone object item
		data::Item::Ptr item = ptr->second->clone();
		// insert as model object item
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(scene.getCS());
			to<Data>(dataCurrentPtr)->itemMap.erase(modelItemObj);
			ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(modelItemObj, item));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// set model robot pose
		to<Data>(dataCurrentPtr)->modelState.reset(new golem::Controller::State(WaypointCtrl::lookup(*controller).state));

		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("RMG", [=]() {
		// load model object item
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(scene.getCS());
			data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(modelItemObj);
			if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
				throw Message(Message::LEVEL_ERROR, "Model object item has not been created");
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// go to model robot pose
		gotoConfig(*to<Data>(dataCurrentPtr)->modelState);

		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("RMI", [=]() {
		// target data
		const Data::Map::iterator targetDataPtr = dataCurrentPtr;

		// data to import
		data::Item::Ptr ptrObject;
		golem::Controller::State::Ptr ptrState;
		Data::Training::Map training;

		// create menu
		bool stop = false;
		MenuCtrlMap menuCtrlMap;
		menuCtrlMap.insert(std::make_pair("", [&](MenuCmdMap& menuCmdMap, std::string& desc) {}));
		MenuCmdMap menuCmdMap;
		menuCmdMap.insert(std::make_pair("\x0D", [&]() {
			if (targetDataPtr == dataCurrentPtr)
				context.write("Select another data bundle\n");
			else {
				data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(modelItemObj);
				if (ptr == to<Data>(dataCurrentPtr)->itemMap.end()) {
					context.write("Model item is not set\n");
					return;
				}
				if (to<Data>(dataCurrentPtr)->modelState == nullptr) {
					context.write("Model state is not set\n");
					return;
				}
				if (to<Data>(dataCurrentPtr)->training.empty()) {
					context.write("No trainig data\n");
					return;
				}
				ptrObject = ptr->second->clone();
				ptrState.reset(new golem::Controller::State(*to<Data>(dataCurrentPtr)->modelState));
				training = to<Data>(dataCurrentPtr)->training;
				stop = true;
			}
		}));
		menuCmdMap.insert(std::make_pair("{", [&]() { getMenuCmdMap()["{"](); }));
		menuCmdMap.insert(std::make_pair("}", [&]() { getMenuCmdMap()["}"](); }));

		// select
		context.write("Press a key to: accept(<Enter>), select index of data({}) bundle...\n");
		while (!stop) {
			U32 menuLevel = 0;
			golem::Menu::menu(menuCtrlMap, menuCmdMap, menuLevel);
		}

		setCurrentDataPtr(targetDataPtr);
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(scene.getCS());
			to<Data>(targetDataPtr)->itemMap.erase(modelItemObj);
			data::Item::Map::iterator ptr = to<Data>(targetDataPtr)->itemMap.insert(to<Data>(targetDataPtr)->itemMap.end(), data::Item::Map::value_type(modelItemObj, ptrObject));
			to<Data>(dataCurrentPtr)->modelState = ptrState;
			to<Data>(dataCurrentPtr)->training.insert(training.begin(), training.end());
			Data::View::setItem(to<Data>(targetDataPtr)->itemMap, ptr, to<Data>(targetDataPtr)->getView());
		}

		// finish
		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("RMA", [=]() {
		// load model object item
		const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(modelItemObj);
		if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
			throw Message(Message::LEVEL_ERROR, "Model object item has not been created");

		// select model any
		const golem::Model::Map::const_iterator modelAny = modelMap.find(ID_ANY);
		if (modelAny == modelMap.end())
			throw Message(Message::LEVEL_ERROR, "Unable to find Model %s", ID_ANY.c_str());

		// clear
		ScopeGuard guard([&]() { golem::CriticalSectionWrapper csw(scene.getCS()); objectRenderer.reset(); });
		RenderBlock renderBlock(*this);

		// select model type
		std::string modelType;
		try {
			golem::StringSeq modelTypeSeq;
			for (Data::Training::Map::const_iterator i = to<Data>(dataCurrentPtr)->training.begin(); i != to<Data>(dataCurrentPtr)->training.end(); i = to<Data>(dataCurrentPtr)->training.upper_bound(i->first))
				modelTypeSeq.push_back(i->first);
			golem::StringSeq::iterator modelTypePtr = modelTypeSeq.end();
			select(modelTypePtr, modelTypeSeq.begin(), modelTypeSeq.end(), "Select type:\n  [0]  Create new type\n", [] (golem::StringSeq::const_iterator ptr) -> const std::string& { return *ptr; });
			modelType = *modelTypePtr;
		}
		catch (const golem::Message&) {
			readString("Enter type: ", modelType);
		}

		// compute reference frame and adjust object frame
		const data::Point3D* point = is<data::Point3D>(ptr);
		if (!point)
			throw Message(Message::LEVEL_ERROR, "Object handler does not implement data::Point3D");
		Vec3Seq points;
		for (size_t i = 0; i < point->getSize(); ++i)
			points.push_back(point->getPoint(i));
		data::Point3D::Point::Seq pointsTrn;
		pointsTrn.resize(points.size());
		Mat34 frame = forwardTransformArm(WaypointCtrl::lookup(*controller).state), frameInv;
		frameInv.setInverse(frame);

		// run model tools
		const std::string options("Press a key to: add (C)ontact/(T)rajectory data, (G)oto contact, finish <Enter>...");
		context.write("%s\n", options.c_str());
		for (bool finish = false; !finish;) {
			// attach object to the robot's end-effector
			frame = forwardTransformArm(WaypointCtrl::lookup(*controller).state);
			const Mat34 trn = frame * frameInv; // frame = trn * frameInit, trn = frame * frameInit^-1
			for (size_t i = 0; i < points.size(); ++i)
				trn.multiply(pointsTrn[i], points[i]);
			{
				golem::CriticalSectionWrapper csw(scene.getCS());
				objectRenderer.reset();
				for (size_t i = 0; i < pointsTrn.size(); ++i)
					objectRenderer.addPoint(pointsTrn[i], objectFrameAdjustment.colourSolid);
			}

			// model options
			const int key = waitKey(20);
			switch (key) {
			case 'G': {
				context.write("%s )G(\n", options.c_str());
				// clone model object item
				if (to<Data>(dataCurrentPtr)->training.empty())
					throw Message(Message::LEVEL_ERROR, "No contacts");
				Data::Training::Map::const_iterator contactPtr = to<Data>(dataCurrentPtr)->training.begin();
				size_t id = 0;
				select(contactPtr, to<Data>(dataCurrentPtr)->training.begin(), to<Data>(dataCurrentPtr)->training.end(), "Contact:\n", [&](Data::Training::Map::const_iterator ptr) -> std::string {
					return makeString("Contact id: %u", ++id);
				});
				// go to contact pose
				gotoConfig(contactPtr->second.state);
				// done here
				context.write("Done!\n");
				break;
			}
			case 'C': {
				context.write("%s )C(\n", options.c_str());
				// clone model object item
				data::Item::Ptr item = ptr->second->clone();
				data::Feature3D* features = is<data::Feature3D>(item.get());
				if (!features)
					throw Message(Message::LEVEL_ERROR, "Object handler does not implement data::Feature3D");
				// transform
				features->transform(frame * frameInv);
				// select model
				golem::Model::Map::const_iterator model = modelMap.find(modelType);
				if (model == modelMap.end()) model = modelAny;
				context.write("Using model: %s\n", model->first.c_str());
				// create contact training data
				const golem::Vec3Seq& vertices = to<Data>(dataCurrentPtr)->modelVertices;
				const golem::TriangleSeq& triangles = to<Data>(dataCurrentPtr)->modelTriangles;
				golem::Contact3D::Triangle::Seq modelMesh;
				for (golem::TriangleSeq::const_iterator j = triangles.begin(); j != triangles.end(); ++j)
					modelMesh.push_back(Contact3D::Triangle(vertices[j->t1], vertices[j->t2], vertices[j->t3]));
				Contact3D::Data contacts;
				if (model->second->create(*features, to<Data>(dataCurrentPtr)->modelFrame, modelMesh, contacts)) {
					golem::CriticalSectionWrapper cswData(scene.getCS());
					Data::Training training(WaypointCtrl::lookup(*controller).state);
					training.contacts = contacts;
					training.frame = to<Data>(dataCurrentPtr)->modelFrame;
					training.points = pointsTrn;
					to<Data>(dataCurrentPtr)->setTrainingItem(to<Data>(dataCurrentPtr)->training.insert(to<Data>(dataCurrentPtr)->training.end(), std::make_pair(modelType, training)));
					createRender();
				}
				// done here
				context.write("Done!\n");
				break;
			}
			case 'T': {
				context.write("%s )T(\n", options.c_str());
				// add trajectory waypoint
				const std::string trjName = getTrajectoryName(modelItemTrj, modelType);
				golem::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(trjName);
				if (ptr == to<Data>(dataCurrentPtr)->itemMap.end()) {
					golem::CriticalSectionWrapper cswData(scene.getCS());
					ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(trjName, modelHandlerTrj->create()));
				}
				data::Trajectory* trajectory = is<data::Trajectory>(ptr);
				if (!trajectory)
					throw Message(Message::LEVEL_ERROR, "Trajectory handler does not implement data::Trajectory");
				// add current state
				WaypointCtrl::Seq seq = trajectory->getWaypoints();
				seq.push_back(WaypointCtrl::lookup(*controller));
				trajectory->setWaypoints(seq);
				// done here
				context.write("Done!\n");
				break;
			}
			case 13: finish = true; break;
			//case 27: throw Cancel("Cancelled");
			}
		}

		// update occupancy
		Data::Cluster::setToDefault(clusterMap, to<Data>(dataCurrentPtr)->clusterCounter, to<Data>(dataCurrentPtr)->training);

		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("RMR", [=]() {
		if (to<Data>(dataCurrentPtr)->training.empty())
			throw Message(Message::LEVEL_ERROR, "Empty training data");

		const bool bModelType = option("TC", "Remove (T)ype/(C)ontact... ") == 'T';
		// load model object item
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(scene.getCS());
			AppPlacement::Data::Training::Map::iterator ptr = to<Data>(dataCurrentPtr)->getTrainingItem();
			const std::string modelType = ptr->first; // cache
			if (bModelType)
				to<Data>(dataCurrentPtr)->training.erase(ptr->first);
			else
				to<Data>(dataCurrentPtr)->training.erase(ptr);
			// remove trajectory if no items of a given type are present
			//if (to<Data>(dataCurrentPtr)->training.end() == to<Data>(dataCurrentPtr)->training.find(modelType))
			//	to<Data>(dataCurrentPtr)->itemMap.erase(getTrajectoryName(modelItemTrj, modelType));
			createRender();
		}

		context.write("Done!\n");
	}));

	// query operations
	menuCtrlMap.insert(std::make_pair("RQ", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		// set mode
		to<Data>(dataCurrentPtr)->mode = Data::MODE_QUERY;
		createRender();

		desc = "Press a key to: create (D)ensities, generate (S)olutions ...";
	}));
	menuCmdMap.insert(std::make_pair("RQD", [=]() {
		// load object item
		data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(objectItem);
		if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
			throw Message(Message::LEVEL_ERROR, "Object item has not been created");
		// wrist frame
		const Mat34 frame = forwardTransformArm(WaypointCtrl::lookup(*controller).state);
		// create query densities
		createQuery(ptr->second, frame, &to<Data>(dataCurrentPtr)->clusterCounter);
		createRender();

		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("RQS", [=]() {
		// generate wrist pose solutions
		generateSolutions();

		to<Data>(dataCurrentPtr)->mode = Data::MODE_SOLUTION;
		createRender();

		context.write("Done!\n");
	}));

	// trajectory operations
	menuCtrlMap.insert(std::make_pair("RT", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		// set mode
		to<Data>(dataCurrentPtr)->mode = Data::MODE_SOLUTION;
		createRender();

		desc = "Press a key to: trajectory (S)elect/(P)erform/(C)lear occupied slots...";
	}));
	menuCmdMap.insert(std::make_pair("RTS", [=]() {
		// select best trajectory
		selectTrajectory();
		createRender();

		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("RTP", [=]() {
		// perform trajectory
		performTrajectory(true);
		createRender();

		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("RTC", [=]() {
		// reset occupied slots
		Data::Cluster::setToDefault(clusterMap, to<Data>(dataCurrentPtr)->clusterCounter, to<Data>(dataCurrentPtr)->training);
		context.write("Done!\n");
	}));

	////////////////////////////////////////////////////////////////////////////
	//                            MAIN DEMO                                   // 
	////////////////////////////////////////////////////////////////////////////

	menuCmdMap.insert(std::make_pair("RD", [=]() {
		// debug mode
		const bool stopAtBreakPoint = option(0, "Debug mode: ", { "YES", "NO" }) == 0;
		const auto breakPoint = [=] (const char* str) {
			if (stopAtBreakPoint) {
				if (option("YN", makeString("%s: Continue (Y/N)...", str).c_str()) != 'Y')
					throw Cancel("Demo cancelled");
			}
			else {
				context.write("%s\n", str);
				(void)waitKey(0);
			}
		};

		// estimate pose
		if (to<Data>(dataCurrentPtr)->queryVertices.empty() || to<Data>(dataCurrentPtr)->queryVertices.empty()) {
			breakPoint("Dishwasher pose estimation");
			estimatePose(Data::MODE_QUERY);
		}

		// run demo
		for (;;) {
			// grasp and scan object
			breakPoint("Object grasp and point cloud capture");
			golem::data::Item::Map::iterator ptr = objectGraspAndCapture(stopAtBreakPoint);

			//breakPoint("Action planning");

			// compute features and add to data bundle
			ptr = objectProcess(ptr);
			// wrist frame
			const Mat34 frame = forwardTransformArm(WaypointCtrl::lookup(*controller).state);
			// create query densities
			createQuery(ptr->second, frame, &to<Data>(dataCurrentPtr)->clusterCounter);
			// generate wrist pose solutions
			generateSolutions();
			// select best trajectory
			selectTrajectory();

			breakPoint("Action execution");
			// execute trajectory
			performTrajectory(stopAtBreakPoint);
		}
		context.write("Done!\n");

	}));

	////////////////////////////////////////////////////////////////////////////
	//                               UTILITIES                                // 
	////////////////////////////////////////////////////////////////////////////

	menuCtrlMap.insert(std::make_pair("Z", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc =
			"(R)otate, (N)udge, (H)and control, (O)bjectGraspAndCapture, rgb-to-ir (T)ranform\n"
			"(B)atch transform, (D)epth camera adjust, create (P)oses, (L)ocate object, trajectory d(U)ration...";
	}));

	menuCmdMap.insert(std::make_pair("ZU", [=]() {
		context.write("*** Change trajectory duration BEWARE ***\n");
		
		SecTmReal trajectoryDuration = getPlanner().trajectoryDuration;
		do
			readNumber("trajectoryDuration ", trajectoryDuration);
		while (trajectoryDuration < 1.0);
		
		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("ZO", [=]() {
		context.write("Testing objectGraspAndCapture()\n");
		objectGraspAndCapture(true);
		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("ZR", [=]() {
		context.write("rotate object in hand\n");
		rotateObjectInHand();
		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("ZN", [=]() {
		context.write("nudge wrist\n");
		nudgeWrist();
		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("ZH", [=]() {
		context.write("Hand Control\n");
		for (;;)
		{
			const int k = option("+-01 ", "increase grasp:+  relax grasp:-  open:0  close:1  <SPACE> to end");
			if (k == 32) break;
			switch (k)
			{
			case '+':
				closeHand(0.1, 1.0);
				break;
			case '1':
				closeHand(1.0, 4.0);
				break;
			case '-':
				releaseHand(0.1, 1.0);
				break;
			case '0':
				releaseHand(1.0, 2.0);
				break;
			}
		}
		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("ZB", [=]() {
		context.write("Batch transform of all Image items in data bundle\n");

		// find handlers supporting data::Transform
		typedef std::vector<std::pair<data::Handler*, data::Transform*>> TransformMap;
		TransformMap transformMap;
		for (data::Handler::Map::const_iterator i = handlerMap.begin(); i != handlerMap.end(); ++i) {
			data::Transform* transform = is<data::Transform>(i);
			if (transform) transformMap.push_back(std::make_pair(i->second.get(), transform));
		}
		if (transformMap.empty())
			throw Cancel("No handlers support Transform interface");
		// pick up handler
		TransformMap::const_iterator transformPtr = transformMap.begin();
		select(transformPtr, transformMap.begin(), transformMap.end(), "Transform:\n", [] (TransformMap::const_iterator ptr) -> std::string {
			std::stringstream str;
			for (StringSeq::const_iterator i = ptr->second->getTransformInterfaces().begin(); i != ptr->second->getTransformInterfaces().end(); ++i) str << *i << " ";
			return std::string("Handler: ") + ptr->first->getID() + std::string(", Item interfaces: ") + str.str();
		});

		readString("Enter label: ", dataItemLabel);

		// TODO need removeCallback() on ScopeGuard ???
		//UI::addCallback(*this, transformPtr->first);

		// iterate over all items in current bundle - USE A COPY SINCE IT WILL BE MODIFIED
		data::Item::Map& itemMap = to<Data>(dataCurrentPtr)->itemMap;
		data::Item::Map itemMap0(itemMap);
		//const data::Item::Map::iterator ptr = itemMap.begin();
		for (auto ptr0 = itemMap0.begin(); ptr0 != itemMap0.end(); ++ptr0)
		{
			data::ItemImage* itemImage = is<data::ItemImage>(ptr0->second.get());
			if (itemImage==nullptr)
				context.debug("Skipping %s\n", ptr0->first.c_str());
			else
			{
				try
				{
					context.debug("Transforming %s\n", ptr0->first.c_str());
					data::Item::List itemList; // just to contain a single item for transform API
					itemList.push_back(ptr0);

					RenderBlock renderBlock(*this);
					data::Item::Ptr item = transformPtr->second->transform(itemList);
					{
						golem::CriticalSectionWrapper cswData(scene.getCS());
						const data::Item::Map::iterator ptr = itemMap.insert(itemMap.end(), data::Item::Map::value_type(dataItemLabel, item));
						Data::View::setItem(itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
					}
				}
				catch (const std::exception& e)
				{
					context.debug("%s\nFailed to compute transform for %s\n", e.what(), ptr0->first.c_str());
				}
			}
		}

		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("ZT", [=]() {
		context.write("compute rgb-to-ir tranform\n");

		try
		{
			std::string fileRGB, fileIR;
			readString("File[.cal] with extrinsic for rgb: ", fileRGB);
			readString("File[.cal] with extrinsic for ir: ", fileIR);

			XMLParser::Ptr pParserRGB = XMLParser::load(fileRGB + ".cal");
			XMLParser::Ptr pParserIR = XMLParser::load(fileIR + ".cal");

			Mat34 cameraFrame, invCameraFrame, depthCameraFrame, colourToIRFrame;
			XMLData(cameraFrame, pParserRGB->getContextRoot()->getContextFirst("grasp sensor extrinsic"));
			XMLData(depthCameraFrame, pParserIR->getContextRoot()->getContextFirst("grasp sensor extrinsic"));

			invCameraFrame.setInverse(cameraFrame);
			colourToIRFrame.multiply(invCameraFrame, depthCameraFrame);

			XMLParser::Ptr pParser = XMLParser::Desc().create();
			XMLData(colourToIRFrame, pParser->getContextRoot()->getContextFirst("grasp sensor colourToIRFrame", true), true);
			XMLData(cameraFrame, pParser->getContextRoot()->getContextFirst("grasp sensor cameraFrame", true), true);
			XMLData(depthCameraFrame, pParser->getContextRoot()->getContextFirst("grasp sensor depthCameraFrame", true), true);
			FileWriteStream fws("colourToIRFrame.xml");
			pParser->store(fws);

			context.write("Saved transform in colourToIRFrame.xml\n");
		}
		catch (const Message& msg)
		{
			context.debug("%s\nFailed to compute transform!\n", msg.what());
		}
	}));

	menuCmdMap.insert(std::make_pair("ZD", [=]() {
		RBAdjust rba;
		rba.increment.set(golem::Real(0.0005), golem::REAL_PI*golem::Real(0.001)); // small increments

		select(sensorCurrentPtr, sensorMap.begin(), sensorMap.end(), "Select Sensor:\n",
			[](Sensor::Map::const_iterator ptr) -> const std::string& {return ptr->second->getID();} );

		if (!is<CameraDepth>(sensorCurrentPtr))
			throw Cancel("Only works for depth cameras");

		CameraDepth* camera = is<CameraDepth>(sensorCurrentPtr);
		const Mat34 trn0 = camera->getColourToIRFrame();

		context.write(
			"Use adjustment keys %s %s %s or 0 for identity. To finish press <SPACE> or <ESC>\n",
			rba.linKeys.c_str(), rba.angKeys.c_str(), rba.incKeys.c_str());
		for (;;)
		{
			const int k = waitKey(golem::MSEC_TM_U32_INF);
			if (k == 27) // <Esc>
			{
				camera->setColourToIRFrame(trn0);
				throw Cancel("Cancelled");
			}
			if (k == 32) // <Space>
				break;
			if (k == '0')
			{
				camera->setColourToIRFrame(Mat34::identity());
				continue;
			}
			if (rba.adjustIncrement(k))
			{
				const RBDist& incr = rba.getIncrement();
				context.write("increment lin: %f, ang: %f\n", incr.lin, incr.ang);
				continue;
			}
			
			Mat34 trn = camera->getColourToIRFrame();
			rba.adjustFrame(k, trn);
			camera->setColourToIRFrame(trn);
		}

		const Mat34 trn = camera->getColourToIRFrame();
		const Mat34 cameraFrame = camera->getFrame();
		Mat34 depthCameraFrame;
		depthCameraFrame.multiply(cameraFrame, trn);
		context.write("<colourToIRFrame %s></colourToIRFrame>\n", toXMLString(trn).c_str());
		context.write("<extrinsic %s></extrinsic>\n", toXMLString(depthCameraFrame).c_str());
		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("ZP", [=]() {
		context.write("create arm configs for a set of camera frames\n");
		double theta(50), phi1(-180), phi2(160), phiStep(20), R(0.45), cx(0.45), cy(-0.45), cz(-0.30);
		readNumber("theta ", theta);
		readNumber("phi1 ", phi1);
		readNumber("phi2 ", phi2);
		readNumber("phiStep ", phiStep);
		readNumber("R ", R);
		readNumber("cx ", cx);
		readNumber("cy ", cy);
		readNumber("cz ", cz);

		// create a set of poses at co-latitude theta, from phi1 to phi2 (step phiStep)
		// looking at (cx,cy,cz) at a distance R
		std::vector<Mat34> cameraPoses;
		golem::ConfigMat34::Seq configs;
		const Vec3 centre(cx, cy, cz), Zaxis(0.0,0.0,1.0);
		Vec3 cameraCentre, cameraXdir, cameraYdir, cameraZdir;
		Mat34 cameraFrame, wristPose;
		const double degToRad = golem::REAL_PI / 180.0;
		const double sintheta = sin(theta * degToRad);
		const double costheta = cos(theta * degToRad);
		for (double phi = phi1; phi <= phi2; phi += phiStep)
		{
			const double sinphi = sin(phi * degToRad);
			const double cosphi = cos(phi * degToRad);
			const Vec3 ray(cosphi*sintheta, sinphi*sintheta, costheta);
			cameraCentre.multiply(R, ray);
			cameraCentre += centre;
			cameraZdir.multiply(-1.0, ray);
			cameraXdir.cross(cameraZdir, Zaxis);
			cameraXdir.normalise();
			cameraYdir.cross(cameraZdir, cameraXdir);
			cameraFrame.R = Mat33(cameraXdir, cameraYdir, cameraZdir);
			cameraFrame.p = cameraCentre;
			cameraPoses.push_back(cameraFrame);
			objectRenderer.addAxes3D(cameraFrame, golem::Vec3(0.05));

			//Mat33 m;
			//m.setTransposed(cameraFrame.R);
			//m.multiply(m, cameraFrame.R);
			//context.debug("%s\n", toXMLString(Mat34(m,Vec3::zero())).c_str());

			// transform from camera to wrist frame
			// default calibration for wrist extrinsics
			Mat34 trn(
				Mat33(-0.015082, -0.0709979, 0.997362, 0.999767, 0.0143117, 0.0161371, -0.0154197, 0.997374, 0.0707655),
				Vec3(0.0920632, -0.0388034, 0.161098));
			golem::Camera* camera = getWristCamera(true);
			if (camera != nullptr)
				trn = getWristCamera()->getCurrentCalibration()->getParameters().pose;

			Mat34 invTrn;
			invTrn.setInverse(trn);
			wristPose = cameraFrame * invTrn;
			// get config from wrist pose, by executing trajectory from current pose!!!
			gotoWristPose(wristPose);
			golem::ConfigMat34 cfg;
			getPose(0, cfg);
			configs.push_back(cfg);
		}

		// write out configs in xml format to file
		std::string filePoses("scan_poses.xml");
		readString("Filename for scan poses: ", filePoses);
		FileWriteStream fws(filePoses.c_str());

		std::ostringstream os;
		bool first(true);
		for (auto i = configs.begin(); i != configs.end(); ++i, first=false)
		{
			if (!first) os << "\n";
			os << "  <pose name=\"scan_poses\" dim=\"61\" " << toXMLString(*i) << "/>";
		}
		os << std::ends;
		fws.write(os.str().c_str(), os.str().size()-1);

		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("ZL", [=]() {
		context.write("locate object\n");

		std::string itemName("transformed_image_for_locate");
		
		data::Item::Map& itemMap = to<Data>(dataCurrentPtr)->itemMap;

		if (itemMap.empty())
			throw Message(Message::LEVEL_ERROR, "No items");

		// get current item as a data::Item::Map::iterator ptr
		data::Item::Map::iterator ptr = itemMap.begin(); // @@@ should use current item @@@

		data::ItemImage* itemImage = to<data::ItemImage>(ptr->second.get());

		// generate features
		data::Transform* transform = is<data::Transform>(&ptr->second->getHandler());
		if (!transform)
			throw Message(Message::LEVEL_ERROR, "Current item does not support Transform interface");
		data::Item::List list;
		list.insert(list.end(), ptr);
		data::Item::Ptr item = transform->transform(list);
		{
			golem::CriticalSectionWrapper cswData(scene.getCS());
			itemMap.erase(itemName);
			ptr = itemMap.insert(itemMap.end(), data::Item::Map::value_type(itemName, item));
			Data::View::setItem(itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}

		// estimate pose
		data::Model* model = is<data::Model>(ptr);
		if (!model)
			throw Message(Message::LEVEL_ERROR, "Transformed item does not support Model interface");
		golem::ConfigMat34 robotPose;
		golem::Mat34 modelFrame;
		model->model(robotPose, modelFrame);

		context.write("Model pose is: %s\n", toXMLString(modelFrame).c_str());

		const int k = option("MC", "Update (M)odel pose or (C)amera extrinsics for some .cal file, or (Q)uit");
		if (k == 'C')
		{
			Mat34 cameraFrame, newCameraFrame, invModelFrame, trueModelFrame;
			invModelFrame.setInverse(modelFrame);

			// get camera frame of image F (should be extrinsics of depth camera)
			cameraFrame = itemImage->parameters.pose;

			std::string fileCal("GolemCameraOpenNIChest.cal");
			readString("Calibration filename: ", fileCal);

			// assume true model pose is in .cal file
			XMLParser::Ptr pParser = XMLParser::load(fileCal);
			XMLData(trueModelFrame, pParser->getContextRoot()->getContextFirst("grasp sensor extrinsic model pose"));

			context.write("Camera frame for image was:  %s\n", toXMLString(cameraFrame).c_str());
			context.write("Assuming true model pose is: %s\n", toXMLString(trueModelFrame).c_str());

			// new extrinsics = trueModelFrame * modelFrame^-1 * cameraFrame
			newCameraFrame.multiply(invModelFrame, cameraFrame);
			context.write("transform from model to camera wrt model frame: %s\n", toXMLString(newCameraFrame).c_str());
			newCameraFrame.multiply(trueModelFrame, newCameraFrame);

			context.write("Updating %s with\n<extrinsic %s></extrinsic>\n", fileCal.c_str(), toXMLString(newCameraFrame).c_str());
			
			// current static camera on Boris:
			// m11 = "-0.689817" m12 = "-0.351147" m13 = "0.633124" m21 = "-0.723807" m22 = "0.353741" m23 = "-0.592427" m31 = "-0.015934" m32 = "-0.866928" m33 = "-0.498180"
			// v1 = "0.216904" v2 = "-0.058540" v3 = "0.281856"

			// write new extrinsics into .cal xml
			FileWriteStream fws(fileCal.c_str());
			XMLData(newCameraFrame, pParser->getContextRoot()->getContextFirst("grasp sensor extrinsic", true), true);
			pParser->store(fws); // @@@ save does not work
		}
		else if (k == 'M')
		{
			std::string fileCal("GolemCameraOpenNIChest.cal");
			readString("Calibration filename: ", fileCal);

			XMLParser::Ptr pParser = XMLParser::load(fileCal);
			XMLData(modelFrame, pParser->getContextRoot()->getContextFirst("grasp sensor extrinsic model pose", true), true);
			FileWriteStream fws(fileCal.c_str());
			pParser->store(fws); // @@@ save does not work
		}

		context.write("Done!\n");
	}));

	// END OF MENUS

	}

//------------------------------------------------------------------------------

golem::data::Item::Map::iterator golem::AppPlacement::estimatePose(Data::Mode mode) {
	if (mode != Data::MODE_MODEL && (to<Data>(dataCurrentPtr)->modelVertices.empty() || to<Data>(dataCurrentPtr)->modelTriangles.empty()))
		throw Cancel("Model has not been estimated");

	golem::Vec3Seq& vertices = mode != Data::MODE_MODEL ? to<Data>(dataCurrentPtr)->queryVertices : to<Data>(dataCurrentPtr)->modelVertices;
	golem::TriangleSeq& triangles = mode != Data::MODE_MODEL ? to<Data>(dataCurrentPtr)->queryTriangles : to<Data>(dataCurrentPtr)->modelTriangles;
	golem::Mat34& frame = mode != Data::MODE_MODEL ? to<Data>(dataCurrentPtr)->queryFrame : to<Data>(dataCurrentPtr)->modelFrame;
	const std::string itemName = mode != Data::MODE_MODEL ? queryItem : modelItem;
	golem::data::Handler* handler = mode != Data::MODE_MODEL ? queryHandler : modelHandler;
	golem::Camera* camera = mode != Data::MODE_MODEL ? queryCamera : modelCamera;

	// set mode
	to<Data>(dataCurrentPtr)->mode = mode;

	// run robot
	gotoPose(modelScanPose);

	// block keyboard and mouse interaction
	InputBlock inputBlock(*this);
	{
		RenderBlock renderBlock(*this);
		golem::CriticalSectionWrapper cswData(scene.getCS());
		to<Data>(dataCurrentPtr)->itemMap.erase(itemName);
		vertices.clear();
		triangles.clear();
	}
	// capture and insert data
	data::Capture* capture = is<data::Capture>(handler);
	if (!capture)
		throw Message(Message::LEVEL_ERROR, "Handler %s does not support Capture interface", handler->getID().c_str());
	data::Item::Map::iterator ptr;
	data::Item::Ptr item = capture->capture(*camera, [&](const golem::TimeStamp*) -> bool { return true; });
	{
		RenderBlock renderBlock(*this);
		golem::CriticalSectionWrapper cswData(scene.getCS());
		to<Data>(dataCurrentPtr)->itemMap.erase(itemName);
		ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(itemName, item));
		Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
	}
	// generate features
	data::Transform* transform = is<data::Transform>(handler);
	if (!transform)
		throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", handler->getID().c_str());
	data::Item::List list;
	list.insert(list.end(), ptr);
	item = transform->transform(list);
	{
		RenderBlock renderBlock(*this);
		golem::CriticalSectionWrapper cswData(scene.getCS());
		to<Data>(dataCurrentPtr)->itemMap.erase(itemName);
		ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(itemName, item));
		Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
	}
	// estimate pose
	data::Model* model = is<data::Model>(ptr);
	if (!model)
		throw Message(Message::LEVEL_ERROR, "Item %s does not support Model interface", ptr->first.c_str());
	golem::ConfigMat34 robotPose;
	golem::Mat34 modelFrame, modelNewFrame;
	Vec3Seq modelVertices;
	TriangleSeq modelTriangles;
	model->model(robotPose, modelFrame, &modelVertices, &modelTriangles);

	// compute a new frame
	if (mode == Data::MODE_MODEL) {
		Vec3Seq modelPoints;
		Rand rand(context.getRandSeed());
		Import().generate(rand, modelVertices, modelTriangles, [&](const golem::Vec3& p, const golem::Vec3&) { modelPoints.push_back(p); });
		modelNewFrame = RBPose::createFrame(modelPoints);
	}

	// create triangle mesh
	{
		RenderBlock renderBlock(*this);
		golem::CriticalSectionWrapper cswData(scene.getCS());
		vertices = modelVertices;
		triangles = modelTriangles;
		if (mode == Data::MODE_MODEL) {
			// newFrame = modelFrame * offset ==> offset = modelFrame^-1 * newFrame
			frame = modelNewFrame;
			to<Data>(dataCurrentPtr)->modelFrameOffset.setInverse(modelFrame);
			to<Data>(dataCurrentPtr)->modelFrameOffset.multiply(to<Data>(dataCurrentPtr)->modelFrameOffset, modelNewFrame);
		}
		else
			frame = modelFrame * to<Data>(dataCurrentPtr)->modelFrameOffset;
	}

	return ptr;
}

//------------------------------------------------------------------------------

// move robot to grasp open pose, wait for force event, grasp object (closed pose), move through scan poses and capture object, add as objectScan
// 1. move robot to grasp open pose
// 2. wait for force event; reset bias and wait for change > threshold (set in xml)
// 3. grasp object (closed pose - generate in virtual env; strong enough for both plate and cup; only fingers should move)
// full hand grasp, fingers spread out; thumb in centre
// 4. move through scan poses and capture object, add as objectScan
// return ptr to Item
golem::data::Item::Map::iterator golem::AppPlacement::objectGraspAndCapture(const bool stopAtBreakPoint)
{
	const auto breakPoint = [=](const char* str) {
		if (stopAtBreakPoint) {
			if (option("YN", makeString("%s: Continue (Y/N)...", str).c_str()) != 'Y')
				throw Cancel("Demo cancelled");
		}
		else {
			context.write("%s\n", str);
			(void)waitKey(0);
		}
	};

	gotoPose2(graspPoseOpen, getPlanner().trajectoryDuration);

	context.write("Waiting for force event, simulate (F)orce event or <ESC> to cancel\n");
	ForceEvent forceEvent(graspSensorForce, graspThresholdForce);
	forceEvent.setBias();
	for (;;)
	{
		const int k = waitKey(10); // poll FT every 10ms
		if (k == 27) // <Esc>
			throw Cancel("Cancelled");
		if (k == 'F')
		{
			context.debug("Simulated force event\n");
			break;
		}

		if (forceEvent.detected(&context))
			break;
	}

	context.debug("Closing hand!\n");
	gotoPose2(graspPoseClosed, graspCloseDuration);

	Sleep::msleep(SecToMSec(graspEventTimeWait));

	breakPoint("Go to scan pose");

	context.debug("Proceeding to first scan pose!\n");
	gotoPose2(objectScanPoseSeq.front(), getPlanner().trajectoryDuration);

	data::Capture* capture = is<data::Capture>(objectHandlerScan);
	if (!capture)
		throw Message(Message::LEVEL_ERROR, "Handler %s does not support Capture interface", objectHandlerScan->getID().c_str());

	RenderBlock renderBlock(*this);
	data::Item::Map::iterator ptr;
	{
		golem::CriticalSectionWrapper cswData(scene.getCS());
		data::Item::Ptr item = capture->capture(*objectCamera, [&](const golem::TimeStamp*) -> bool { return true; });

		// Finally: insert object scan, remove old one
		data::Item::Map& itemMap = to<Data>(dataCurrentPtr)->itemMap;
		itemMap.erase(objectItemScan);
		ptr = itemMap.insert(itemMap.end(), data::Item::Map::value_type(objectItemScan, item));
		Data::View::setItem(itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
	}

	return ptr;
}

//------------------------------------------------------------------------------

// Process object image and add to data bundle
golem::data::Item::Map::iterator golem::AppPlacement::objectProcess(golem::data::Item::Map::iterator ptr) {
	// generate features
	data::Transform* transform = is<data::Transform>(objectHandler);
	if (!transform)
		throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", objectHandler->getID().c_str());

	data::Item::List list;
	list.insert(list.end(), ptr);
	data::Item::Ptr item = transform->transform(list);

	// insert processed object, remove old one
	RenderBlock renderBlock(*this);
	golem::CriticalSectionWrapper cswData(scene.getCS());
	to<Data>(dataCurrentPtr)->itemMap.erase(objectItem);
	ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(objectItem, item));
	Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
	return ptr;
}

//------------------------------------------------------------------------------

std::string golem::AppPlacement::getTrajectoryName(const std::string& prefix, const std::string& type) const {
	return prefix + dataDesc->sepName + type;
}

//------------------------------------------------------------------------------

void golem::AppPlacement::createQuery(golem::data::Item::Ptr item, const golem::Mat34& frame, const Data::Cluster::Counter* clusterCounter) {
	// Features
	const data::Feature3D* features = is<data::Feature3D>(item.get());
	if (!features)
		throw Message(Message::LEVEL_ERROR, "AppPlacement::createQuery(): No query features");
	// training data are required
	if (to<Data>(dataCurrentPtr)->training.empty())
		throw Message(Message::LEVEL_ERROR, "AppPlacement::createQuery(): No model densities");
	// model object pose
	if (to<Data>(dataCurrentPtr)->modelVertices.empty() || to<Data>(dataCurrentPtr)->modelVertices.empty())
		throw Message(Message::LEVEL_ERROR, "AppPlacement::createQuery(): No model pose");
	// query object pose
	if (to<Data>(dataCurrentPtr)->queryVertices.empty() || to<Data>(dataCurrentPtr)->queryVertices.empty())
		throw Message(Message::LEVEL_ERROR, "AppPlacement::createQuery(): No query pose");

	// select query any
	const golem::Query::Map::const_iterator queryAny = queryMap.find(ID_ANY);
	if (queryAny == queryMap.end())
		throw Message(Message::LEVEL_ERROR, "AppPlacement::createQuery(): Unable to find Query %s", ID_ANY.c_str());
	// select pose std dev any
	const PoseDensity::Map::const_iterator poseAny = poseMap.find(ID_ANY);
	if (poseAny == poseMap.end())
		throw Message(Message::LEVEL_ERROR, "AppPlacement::createQuery(): Unable to find pose density %s", ID_ANY.c_str());

	to<Data>(dataCurrentPtr)->densities.clear();
	for (Data::Training::Map::const_iterator i = to<Data>(dataCurrentPtr)->training.begin(); i != to<Data>(dataCurrentPtr)->training.end(); ++i) {
		Data::Density density;
		density.type = i->first;

		try {
			// range
			const Data::Training::Range range = to<Data>(dataCurrentPtr)->training.equal_range(i->first);
			// index
			const U32 index = (U32)std::distance(range.first, i);
			// check if slot is occupied
			if (clusterCounter && Data::Cluster::isOccupied(clusterMap, *clusterCounter, i->first, index)) {
				density.weight = REAL_ZERO;
				to<Data>(dataCurrentPtr)->densities.push_back(density);
				context.debug("AppPlacement::createQuery(): slot %s index %u is occupied\n", i->first.c_str(), index);
				continue;
			}
			else
				context.debug("AppPlacement::createQuery(): slot %s index %u is empty\n", i->first.c_str(), index);
		}
		catch (const Message& msg) {
			context.write(msg);
		}

		// select query density
		golem::Query::Map::const_iterator query = queryMap.find(i->first);
		if (query == queryMap.end()) query = queryAny;

		try {
			query->second->clear();
			//if (!golem::Sample<golem::Real>::normalise<golem::Ref1>(const_cast<golem::Contact3D::Seq&>(i->second.contacts)))
			//	throw Message(Message::LEVEL_ERROR, "AppPlacement::createQuery(): Unable to normalise model distribution");
			query->second->create(i->second.contacts, *features);
		}
		catch (const std::exception& ex) {
			context.write("%s\n", ex.what());
			continue;
		}

		// object density
		density.object = query->second->getPoses();
		for (golem::Query::Pose::Seq::iterator j = density.object.begin(); j != density.object.end(); ++j) {
			Mat34 trn;

			// frame transform: eff_curr = frame, model = modelFrame
			// model = trn * query |==> trn = model * query^-1
			// eff_pred = trn * eff_curr |==> eff_pred = model * query^-1 * eff_curr
			trn.setInverse(j->toMat34());
			trn.multiply(trn, frame);
			trn.multiply(to<Data>(dataCurrentPtr)->queryFrame, trn);
			j->fromMat34(trn);
		}

		// select pose any
		PoseDensity::Map::const_iterator pose = poseMap.find(i->first);
		if (pose == poseMap.end()) pose = poseAny;

		// trajectory
		const std::string trjName = getTrajectoryName(modelItemTrj, i->first);
		golem::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(trjName);
		if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
			throw Message(Message::LEVEL_ERROR, "AppPlacement::createQuery(): Empty trajectory");
		data::Trajectory* trajectory = is<data::Trajectory>(ptr);
		if (!trajectory)
			throw Message(Message::LEVEL_ERROR, "AppPlacement::createQuery(): Trajectory handler does not implement data::Trajectory");
		// waypoints
		const WaypointCtrl::Seq waypoints = trajectory->getWaypoints();
		if (waypoints.size() < 2)
			throw Message(Message::LEVEL_ERROR, "AppPlacement::createQuery(): Trajectory must have at least two waypoints");

		// Desired trajectory frame at contact pose
		const golem::Mat34 trajectoryFrame = forwardTransformArm(i->second.state);
		// Contact and approach poses as recorded
		const golem::Mat34 contactPose(forwardTransformArm(waypoints[0].state)), approachPose(forwardTransformArm(waypoints[1].state));
		// trajectoryFrame = trn * contactPose |==> trn = trajectoryFrame * contactPose^-1
		golem::Mat34 trnTrj;
		trnTrj.setInverse(contactPose);
		trnTrj.multiply(trajectoryFrame, trnTrj);

		// query = trn * model |==> trn = query * model^-1
		golem::Mat34 trn;
		trn.setInverse(to<Data>(dataCurrentPtr)->modelFrame);
		trn.multiply(to<Data>(dataCurrentPtr)->queryFrame, trn);

		// Contact and approach poses in the desired frame
		const golem::RBCoord contactFrame(trn * trajectoryFrame), approachFrame(trn * trnTrj * approachPose);

		// distance
		const RBDist frameDist(contactFrame.p.distance(approachFrame.p), contactFrame.q.distance(approachFrame.q));
		if (frameDist.lin < REAL_EPS/* || frameDist.ang < REAL_EPS*/)
			throw Message(Message::LEVEL_ERROR, "AppPlacement::createQuery(): Invalid distance between waypoints");

		// create pose distribution
		const I32 range = pose->second.kernels / 2 + 1;
		density.pose.reserve(2 * range + 1);
		for (I32 j = -range; j <= range; ++j) {
			const Real dist = Real(j)/range;

			// kernel
			golem::Query::Pose qp;
			
			// interpolation factor
			const golem::RBDist interpol(dist * pose->second.pathDist.lin / frameDist.lin, REAL_ZERO/*dist * pose->second.pathDist.ang / frameDist.ang*/);

			// linear interpolation/extrapolation
			qp.p.interpolate(contactFrame.p, approachFrame.p, interpol.lin);
			// angular interpolation/extrapolation, TODO use angular distance scaling
			qp.q.slerp(contactFrame.q, approachFrame.q, interpol.lin);

			// kernel parameters
			qp.covSqrt = pose->second.stdDev;
			qp.cov.set(Math::sqr(qp.covSqrt.lin), Math::sqr(qp.covSqrt.ang));
			qp.covInv.set(REAL_ONE / qp.cov.lin, REAL_ONE / qp.cov.ang);
			qp.distMax.set(poseDistanceMax * qp.cov.lin, poseDistanceMax * qp.cov.ang);
			// kernel weight
			qp.weight = Math::exp( - Math::abs(dist)*pose->second.pathDistStdDev); // set this to 1.0 to ignore kernel weights

			// add to pose distribution
			density.pose.push_back(qp);
		}

		// normalise pose distribution
		if (!golem::Sample<Real>::normalise<golem::Ref1>(density.pose))
			throw Message(Message::LEVEL_ERROR, "AppPlacement::createQuery(): Unable to normalise pose distribution");

		// create path
		density.path = manipulator->create(waypoints, [=](const Manipulator::Config& l, const Manipulator::Config& r) -> Real { return poseCovInv.dot(RBDist(l.frame, r.frame)); });
		
		// points
		Mat34 trnObj;
		trnObj.setInverse(frame);
		density.points.clear();
		density.points.reserve(features->getSize());
		for (size_t i = 0; i < features->getSize(); ++i) {
			golem::data::Point3D::Point p = features->getPoint(i);
			trnObj.multiply(p, p);
			density.points.push_back(p);
		}

		// end-effector frame
		density.frame = trajectoryFrame;
		
		// likelihood
		density.weight = query->second->weight;

		// done
		to<Data>(dataCurrentPtr)->densities.push_back(density);
	}

	if (to<Data>(dataCurrentPtr)->densities.empty())
		throw Message(Message::LEVEL_ERROR, "AppPlacement::createQuery(): No query created");

	if (!golem::Sample<Real>::normalise<golem::Ref1>(to<Data>(dataCurrentPtr)->densities))
		throw Message(Message::LEVEL_ERROR, "AppPlacement::createQuery(): Unable to normalise query distributions");
}

void golem::AppPlacement::generateSolutions() {
	// training data are required
	if (to<Data>(dataCurrentPtr)->densities.empty())
		throw Message(Message::LEVEL_ERROR, "AppPlacement::generateSolutions(): No query densities");

	const SecTmReal t = context.getTimer().elapsed();

	Mat34 referenceFrame;
	referenceFrame.setInverse(manipulator->getReferenceFrame());

	size_t acceptGreedy = 0, acceptSA = 0;

	to<Data>(dataCurrentPtr)->solutions.resize(optimisation.runs);
	Data::Solution::Seq::iterator ptr = to<Data>(dataCurrentPtr)->solutions.begin();
	CriticalSection cs;
	ParallelsTask(context.getParallels(), [&](ParallelsTask*) {
		const U32 jobId = context.getParallels()->getCurrentJob()->getJobId();
		Rand rand(RandSeed(this->context.getRandSeed()._U32[0] + jobId, (U32)0));

		Data::Solution *solution = nullptr, test;

		for (;;) {
			// select next pointer
			{
				CriticalSectionWrapper csw(cs);
				if (ptr == to<Data>(dataCurrentPtr)->solutions.end())
					break;
				solution = &*ptr++;
			}

			// sample query density
			Data::Density::Seq::const_iterator query;
			for (;;) {
				query = golem::Sample<golem::Real>::sample<golem::Ref1, Data::Density::Seq::const_iterator>(to<Data>(dataCurrentPtr)->densities, rand);
				if (query == to<Data>(dataCurrentPtr)->densities.end()) {
					context.error("AppPlacement::generateSolutions(): Query density sampling error\n");
					return;
				}
				break;
			}
			// sample pose density
			golem::Query::Pose::Seq::const_iterator pose;
			for (;;) {
				pose = golem::Sample<golem::Real>::sample<golem::Ref1, golem::Query::Pose::Seq::const_iterator>(query->pose, rand);
				if (pose == query->pose.end()) {
					context.error("AppPlacement::generateSolutions(): Pose density sampling error\n");
					return;
				}
				break;
			}
			// set
			test.type = query->type;
			test.queryIndex = (U32)(query - to<Data>(dataCurrentPtr)->densities.begin());

			// local search: try to find better solution using simulated annealing
			for (size_t s = 0; s <= optimisation.steps; ++s) {
				const bool init = s == 0;

				// linear schedule
				const Real Scale = Real(optimisation.steps - s)/optimisation.steps; // 0..1
				const Real Temp = (REAL_ONE - Scale)*optimisation.saTemp + Scale;
				const RBDist Delta(optimisation.saDelta.lin*Temp, optimisation.saDelta.ang*Temp);
				const Real Energy = optimisation.saEnergy*Temp;

				// generate test solution
				for (;;) {
					// Linear component
					Vec3 v;
					v.next(rand); // |v|==1
					v.multiply(Math::abs(rand.nextGaussian<Real>(REAL_ZERO, Delta.lin*pose->covSqrt.lin)), v);
					test.pose.p.add(init ? pose->p : solution->pose.p, v);
					// Angular component
					const Real poseCovInvAng = pose->covInv.ang / Math::sqr(Delta.ang);
					Quat q;
					q.next(rand, poseCovInvAng);
					test.pose.q.multiply(init ? pose->q : solution->pose.q, q);

					// create path
					test.path = query->path;

					// transform to the new frame
					RBCoord inv;
					inv.setInverse(test.path[0].frame);
					for (Manipulator::Waypoint::Seq::iterator i = test.path.begin(); i != test.path.end(); ++i) {
						i->frame.multiply(inv, i->frame);
						i->frame.multiply(test.pose * referenceFrame, i->frame);
					}

					// evaluate
					test.likelihood.setToDefault();
					// comment out to turn off expert
					if (!Data::Solution::Likelihood::isValid(test.likelihood.contact = evaluateSample(query->object.begin(), query->object.end(), test.pose)))
						continue;
					if (!Data::Solution::Likelihood::isValid(test.likelihood.pose = evaluateSample(query->pose.begin(), query->pose.end(), test.pose)))
						continue;
					if (!Data::Solution::Likelihood::isValid(test.likelihood.collision = golem::numeric_const<golem::Real>::ONE))
						continue;

					test.likelihood.make();
					//test.likelihood.makeLog();

					break;
				}

				// first run sampling only
				if (init) {
					*solution = test;
					continue;
				}

				// accept if better
				if (test.likelihood.likelihood > solution->likelihood.likelihood || Math::exp((test.likelihood.likelihood - solution->likelihood.likelihood) / Energy) > rand.nextUniform<Real>()) {
					// debug
					test.likelihood.likelihood > solution->likelihood.likelihood ? ++acceptGreedy : ++acceptSA;
					// update
					solution->pose = test.pose;
					solution->path = test.path;
					solution->likelihood = test.likelihood;
				}
			}
		}
	});

	// sort
	sortSolutions(to<Data>(dataCurrentPtr)->solutions);

	// print debug information
	context.debug("AppPlacement::generateSolutions(): time=%.6f, solutions=%u, steps=%u, energy=%f, greedy_accept=%d, SA_accept=%d\n", context.getTimer().elapsed() - t, optimisation.runs, optimisation.steps, optimisation.saEnergy, acceptGreedy, acceptSA);
}

void golem::AppPlacement::sortSolutions(Data::Solution::Seq& seq) const {
	if (seq.empty())
		throw Message(Message::LEVEL_ERROR, "AppPlacement::sortSolutions(): No solutions");

	// create pointers
	typedef std::vector<const Data::Solution*> PtrSeq;
	PtrSeq ptrSeq;
	ptrSeq.reserve(seq.size());
	for (Data::Solution::Seq::const_iterator i = seq.begin(); i != seq.end(); ++i)
		ptrSeq.push_back(&*i);

	// sort with clustering
	std::sort(ptrSeq.begin(), ptrSeq.end(), [](const Data::Solution* l, const Data::Solution* r) -> bool {return l->queryIndex < r->queryIndex || l->queryIndex == r->queryIndex && l->likelihood.likelihood > r->likelihood.likelihood; });

	// copy
	//Data::Solution::Seq seqSorted;
	//seqSorted.reserve(ptrSeq.size());
	//for (PtrSeq::const_iterator i = ptrSeq.begin(); i != ptrSeq.end(); ++i)
	//	seqSorted.push_back(**i);
	Data::Solution::Seq seqSorted;
	seqSorted.reserve(ptrSeq.size());
	U32 clusterSize = manipulator->getDesc().trajectoryClusterSize, clusterId = ptrSeq.front()->queryIndex, clusterIndex = 0;
	for (PtrSeq::const_iterator i = ptrSeq.begin(); i != ptrSeq.end();) {
		seqSorted.push_back(**i++);
		if (++clusterIndex >= clusterSize) {
			for (; i != ptrSeq.end(); ++i)
				if (clusterId != (*i)->queryIndex) {
					clusterId = (*i)->queryIndex;
					clusterIndex = 0;
					break;
				}
		}
		else if (clusterId != (*i)->queryIndex) {
			clusterId = (*i)->queryIndex;
			clusterIndex = 0;
			break;
		}
	}

	// replace
	seq = seqSorted;
}

void golem::AppPlacement::selectTrajectory() {
	if (to<Data>(dataCurrentPtr)->solutions.empty())
		throw Message(Message::LEVEL_ERROR, "AppPlacement::selectTrajectory(): No solutions");

	const U32 testTrajectories = (U32)to<Data>(dataCurrentPtr)->solutions.size();

	// TODO collision detection
	// collision bounds
	//CollisionBounds collisionBounds(const_cast<golem::Planner&>(manipulator->getPlanner()), [=] (size_t i, Vec3& p)->bool {
	//	return false;
	//}, &objectRenderer, &csRenderer);
	//collisionBounds.setLocal();

	// search
	const std::pair<U32, RBDistEx> val = manipulator->find<U32>(0, testTrajectories, [&](U32 index) -> RBDistEx {
		to<Data>(dataCurrentPtr)->indexSolution = index;
		createRender();
		Manipulator::Waypoint::Seq& path = to<Data>(dataCurrentPtr)->solutions[index].path;
		const RBDist dist(manipulator->find(path));
		const golem::ConfigspaceCoord approach = path.back().config;
		const bool collides = false;// collisionBounds.collides(approach, manipulator->getArm()->getStateInfo().getJoints().end() - 1); // TODO test approach config using points
		const RBDistEx distex(dist, manipulator->getDesc().trajectoryErr.collision && collides);
		context.write("#%03u/%u: Trajectory error: lin=%.9f, ang=%.9f, collision=%s\n", index + 1, testTrajectories, distex.lin, distex.ang, distex.collision ? "yes" : "no");
		//if (getUICallback() && getUICallback()->hasInputEnabled()) Menu(context, *getUICallback()).option("\x0D", "Press <Enter> to continue...");
		return distex;
	});

	context.write("#%03u: Best trajectory\n", val.first + 1);
	to<Data>(dataCurrentPtr)->indexSolution = val.first;
	const Data::Solution& solution = to<Data>(dataCurrentPtr)->solutions[val.first];
	createRender();
	WaypointCtrl::Seq seq;
	manipulator->create(solution.path, seq);

	// add trajectory waypoint
	{
		RenderBlock renderBlock(*this);
		golem::CriticalSectionWrapper cswData(scene.getCS());
		to<Data>(dataCurrentPtr)->itemMap.erase(queryItemTrj);
		golem::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(queryItemTrj, modelHandlerTrj->create()));
		data::Trajectory* trajectory = is<data::Trajectory>(ptr);
		if (!trajectory)
			throw Message(Message::LEVEL_ERROR, "AppPlacement::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
		// add current state
		trajectory->setWaypoints(seq);
		Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
	}

	// set slot occupancy
	try {
		// restore 
		Data::Training::Map::const_iterator i = to<Data>(dataCurrentPtr)->training.begin();
		std::advance(i, solution.queryIndex); // 1:1 map model-query
		// range
		const Data::Training::Range range = to<Data>(dataCurrentPtr)->training.equal_range(i->first);
		// index
		const U32 index = (U32)std::distance(range.first, i);
		// set slot occupied
		context.debug("AppPlacement::selectTrajectory(): slot %s index %u set occupied\n", solution.type.c_str(), index);
		Data::Cluster::setOccupied(clusterMap, to<Data>(dataCurrentPtr)->clusterCounter, solution.type, index);
	}
	catch (const Message& msg) {
		context.write(msg);
	}
}

void golem::AppPlacement::performTrajectory(bool testTrajectory) {
	golem::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(queryItemTrj);
	if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
		throw Message(Message::LEVEL_ERROR, "AppPlacement::performTrajectory(): Unable to find query trajectory");
	data::Trajectory* trajectory = is<data::Trajectory>(ptr);
	if (!trajectory)
		throw Message(Message::LEVEL_ERROR, "AppPlacement::performTrajectory(): Trajectory handler does not implement data::Trajectory");

	Controller::State::Seq seqInv = WaypointCtrl::make(trajectory->getWaypoints(), true);
	if (seqInv.size() < 2)
		throw Message(Message::LEVEL_ERROR, "AppPlacement::performTrajectory(): At least two waypoints required");
	// reverse
	Controller::State::Seq seq;
	for (Controller::State::Seq::const_reverse_iterator i = seqInv.rbegin(); i != seqInv.rend(); ++i) seq.push_back(*i);
	
	// profile
	struct ProfileCallback : Profile::CallbackDist {
		const AppPlacement* demo;
		ProfileCallback(const AppPlacement* demo) : demo(demo) {}
		Real distConfigspaceCoord(const ConfigspaceCoord& prev, const ConfigspaceCoord& next) const {
			RBCoord cprev(demo->forwardTransformArm(prev)), cnext(demo->forwardTransformArm(next));
			return Math::sqrt(demo->poseCovInv.dot(RBDist(cprev, cnext)));
		}
		Real distCoord(Real prev, Real next) const {
			return Math::abs(prev - next);
		}
		bool distCoordPlanning(const Configspace::Index& index) const {
			return true;
		}
	} profileCallback(this);
	Profile::Desc desc;
	desc.pCallbackDist = &profileCallback;
	//auto pDesc = new Polynomial4::Desc; // 4-th deg polynomial - quadratic velocity
	auto pDesc = new Polynomial1::Desc; // 1-st deg polynomial - constant velocity
	desc.pTrajectoryDesc.reset(pDesc);
	Profile::Ptr profile = desc.create(*controller);
	if (profile == nullptr)
		throw Message(Message::LEVEL_ERROR, "AppPlacement::performTrajectory(): unable to create profile");
	seq.back().t = seq.front().t + manipulatorTrajectoryDuration;
	profile->profile(seq);

	// overwrite hand config in final approach trajectory
	setHandConfig(seq, objectScanPoseSeq.back());

	golem::Controller::State::Seq initTrajectory;
	// problem is that we need to know intended state of hand, i.e. if grasping
	// when grasping with low stiffness, commanded and actual can be quite different
	// so we will maintain continuity with commanded state for the hand
	findTrajectory(lookupStateArmCommandHand(), &seq.front(), nullptr, SEC_TM_REAL_ZERO, initTrajectory);
	// overwriting hand config may cause sudden hand closing - exceed finger velocity limits - if not already grasping
	//setHandConfig(initTrajectory, objectScanPoseSeq.back());

	golem::Controller::State::Seq completeTrajectory = initTrajectory;
	completeTrajectory.insert(completeTrajectory.end(), seq.begin(), seq.end());

	// create trajectory item
	data::Item::Ptr itemTrajectory;
	data::Handler::Map::const_iterator handlerPtr = handlerMap.find(getPlanner().trajectoryHandler);
	if (handlerPtr == handlerMap.end())
		throw Message(Message::LEVEL_ERROR, "AppPlacement::performTrajectory(): unknown default trajectory handler %s", getPlanner().trajectoryHandler.c_str());
	data::Handler* handler = is<data::Handler>(handlerPtr);
	if (!handler)
		throw Message(Message::LEVEL_ERROR, "AppPlacement::performTrajectory(): invalid default trajectory handler %s", getPlanner().trajectoryHandler.c_str());
	itemTrajectory = handler->create();
	data::Trajectory* trajectoryIf = is<data::Trajectory>(itemTrajectory.get());
	if (!trajectoryIf)
		throw Message(Message::LEVEL_ERROR, "AppPlacement::performTrajectory(): unable to create trajectory using handler %s", getPlanner().trajectoryHandler.c_str());
	trajectoryIf->setWaypoints(WaypointCtrl::make(completeTrajectory, completeTrajectory));
	
	// remove if failed
	bool finished = false;
	const Data::View view = to<Data>(dataCurrentPtr)->getView();
	ScopeGuard removeItem([&]() {
		if (!finished) {
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper csw(scene.getCS());
			to<Data>(dataCurrentPtr)->itemMap.erase(manipulatorItemTrj);
			to<Data>(dataCurrentPtr)->getView() = view;
		}
	});
	// add trajectory item
	{
		RenderBlock renderBlock(*this);
		golem::CriticalSectionWrapper csw(scene.getCS());
		const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(manipulatorItemTrj, itemTrajectory));
		Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
	}

	// adjust thresholds
	bool adjustedFT = false;
	if (testTrajectory)
	{
		if (option(0, "Adjust F/T thresholds: ", { "YES", "NO" }) == 0)
		{
			adjustedFT = true;
			do {
				readNumber("X ", trajectoryThresholdForce.v.x);
				readNumber("Y ", trajectoryThresholdForce.v.y);
				readNumber("Z ", trajectoryThresholdForce.v.z);
				readNumber("Tx ", trajectoryThresholdForce.w.x);
				readNumber("Ty ", trajectoryThresholdForce.w.y);
				readNumber("Tz ", trajectoryThresholdForce.w.z);
			} while (option("YN", "OK? (Y/N)") != 'Y');
		}
	}

	// test trajectory
	if (testTrajectory) {
		// prompt user
		EnableKeyboardMouse enableKeyboardMouse(*this);
		option("\x0D", "Press <Enter> to accept trajectory...");
	}

	// block displaying the current item
	RenderBlock renderBlock(*this);

	// go to initial state
	sendTrajectory(initTrajectory);
	// wait until the last trajectory segment is sent
	controller->waitForEnd();

	Sleep::msleep(SecToMSec(0.5)); // wait before taking bias for F/T
	ForceEvent forceEvent(graspSensorForce, trajectoryThresholdForce);
	forceEvent.setBias();

	// send trajectory
	sendTrajectory(seq);

	// repeat every send waypoint until trajectory end
	for (U32 i = 0; controller->waitForBegin(); ++i)
	{
		// ~20ms loop
		if (universe.interrupted())
			throw Exit();
		if (controller->waitForEnd(0))
			break;

		bool spoofedForceEvent = false;
		if (waitKey(0) == 'F')
		{
			context.debug("Simulated force event\n");
			spoofedForceEvent = true;
		}

		if (forceEvent.detected(&context) || spoofedForceEvent)
		{
			context.debug("Force event detected. Halting robot.\n");
			haltRobot();
			controller->waitForBegin();
			break;
			// jiggle, if not docked => when bottom of object is near base of rack
		}

		// print every 10th robot state
		if (i % 10 == 0)
			context.write("State #%d\r", i);
		
		if (adjustedFT)
		{
			if (i % 2 == 0)
				forceEvent.showFT(&context);
			if (i % 50 == 0)
				forceEvent.showMaxExcursion(&context);
		}
	}
	context.debug("\n");

	// open hand and perform withdraw action to a safe pose
	// translate wrist vertically upwards, keeping orientation constant

	context.debug("Releasing hand by %g%% in %gs...\n", withdrawReleaseFraction*100.0, getPlanner().trajectoryDuration);
	releaseHand(withdrawReleaseFraction, getPlanner().trajectoryDuration);

	context.debug("Waiting 1s before withdrawing upwards...\n");
	Sleep::msleep(SecToMSec(1.0));

	context.debug("Lifting hand by %gm in %gs...\n", withdrawLiftDistance, getPlanner().trajectoryDuration);
	liftWrist(withdrawLiftDistance, getPlanner().trajectoryDuration);

	// done
	finished = true;
	context.write("Performance finished!\n");
}

//------------------------------------------------------------------------------

void golem::AppPlacement::render() const {
	Player::render();
	
	modelRenderer.render();
	objectRenderer.render();
}

//------------------------------------------------------------------------------

void golem::XMLData(golem::AppPlacement::Data::Cluster::Map::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("type", const_cast<std::string&>(val.first), xmlcontext, create);
	golem::XMLData("slot", val.second.slot, xmlcontext, create);
}

void golem::XMLData(golem::AppPlacement::PoseDensity::Map::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("id", const_cast<std::string&>(val.first), xmlcontext, create);
	val.second.load(xmlcontext->getContextFirst("pose", create));
}

//------------------------------------------------------------------------------

template <> void golem::Stream::read(golem::AppPlacement::Data::Training::Map::value_type& value) const {
	read(const_cast<std::string&>(value.first));
	read(value.second.state);
	read(value.second.frame);
	value.second.contacts.model.clear();
	read(value.second.contacts.model, value.second.contacts.model.begin());
	value.second.points.clear();
	read(value.second.points, value.second.points.begin());
}

template <> void golem::Stream::write(const golem::AppPlacement::Data::Training::Map::value_type& value) {
	write(value.first);
	write(value.second.state);
	write(value.second.frame);
	write(value.second.contacts.model.begin(), value.second.contacts.model.end());
	write(value.second.points.begin(), value.second.points.end());
}

namespace golem {
	// Legacy conversion
	template <> void Stream::read(golem::Query::Pose::Seq::value_type& value) const {
		read((RBCoord&)value);
		read(value.cov);
		read(value.covSqrt);
		read(value.covInv);
		read(value.distMax);
		//read(value.model);//ignore
		read(value.weight);
		read(value.cdf);
		//fprintf(stderr, "%f, %f\n", value.weight, value.cdf);
	}
	template <> void Stream::write(const golem::Query::Pose::Seq::value_type& value) {
		write((RBCoord&)value);
		write(value.cov);
		write(value.covSqrt);
		write(value.covInv);
		write(value.distMax);
		//write(value.model);//ignore
		write(value.weight);
		write(value.cdf);
	}
};	// namespace

template <> void golem::Stream::read(golem::AppPlacement::Data::Density::Seq::value_type& value) const {
	read(value.type);
	value.object.clear();
	read(value.object, value.object.begin());
	value.pose.clear();
	read(value.pose, value.pose.begin());
	value.path.clear();
	read(value.path, value.path.begin());
	value.points.clear();
	read(value.points, value.points.begin());
	read(value.frame);
	read(value.weight);
	read(value.cdf);
}

template <> void golem::Stream::write(const golem::AppPlacement::Data::Density::Seq::value_type& value) {
	write(value.type);
	write(value.object.begin(), value.object.end());
	write(value.pose.begin(), value.pose.end());
	write(value.path.begin(), value.path.end());
	write(value.points.begin(), value.points.end());
	write(value.frame);
	write(value.weight);
	write(value.cdf);
}

template <> void golem::Stream::read(golem::AppPlacement::Data::Solution::Seq::value_type& value) const {
	read(value.type);
	read(value.pose);
	value.path.clear();
	read(value.path, value.path.begin());
	read(value.likelihood);
	read(value.queryIndex);
}

template <> void golem::Stream::write(const golem::AppPlacement::Data::Solution::Seq::value_type& value) {
	write(value.type);
	write(value.pose);
	write(value.path.begin(), value.path.end());
	write(value.likelihood);
	write(value.queryIndex);
}

template <> void golem::Stream::read(golem::Manipulator::Waypoint& value) const {
	read((golem::Manipulator::Config&)value);
	read((golem::Sample<golem::Real>&)value);
	read(value.control);
	value.control.resize(std::min(value.control.size(), golem::Manipulator::ControlCoord::N()));
}

template <> void golem::Stream::write(const golem::Manipulator::Waypoint& value) {
	write((golem::Manipulator::Config&)value);
	write((golem::Sample<golem::Real>&)value);
	write(value.control);
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return golem::AppPlacement::Desc().main(argc, argv);
}
