/** @file RAGPlanner.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */


#include <Golem/App/R2GPlanner/R2GPlanner.h>
#include <Golem/Contact/OptimisationSA.h>
#include <Golem/Tools/Image.h>
#include <Golem/Data/Image/Image.h>
#include <Golem/Tools/Import.h>
#include <Golem/Plugin/PlannerI.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Data/Feature3D/Feature3D.h>
#include <Golem/Math/Rand.h>
#include <algorithm>
#include <boost/tokenizer.hpp>


#ifdef WIN32
	#pragma warning (push)
	#pragma warning (disable : 4244)
	#pragma warning (disable : 4996)
#endif
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#ifdef WIN32
	#pragma warning (pop)
#endif
//#define  GLUT_KEY_INSERT                    0x006C
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

golem::data::Data::Ptr golem::R2GPlanner::Data::Desc::create(golem::Context &context) const {
	golem::data::Data::Ptr data(new R2GPlanner::Data(context));
	static_cast<R2GPlanner::Data*>(data.get())->create(*this);
	return data;
}

golem::R2GPlanner::Data::Data(golem::Context &context) : PosePlanner::Data(context), owner(nullptr) {
}

void golem::R2GPlanner::Data::create(const Desc& desc) {
	PosePlanner::Data::create(desc);

	triggered = 0;
	replanning = false;
	release = false;
}

golem::R2GPlanner::Data* golem::R2GPlanner::Data::clone() const {
	return new Data(*this);
}


void golem::R2GPlanner::Data::setOwner(golem::Manager* owner) {
	PosePlanner::Data::setOwner(owner);
	this->owner = golem::is<R2GPlanner>(owner);
	if (!this->owner)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::Data::setOwner(): unknown data owner");
}

void golem::R2GPlanner::Data::createRender() {
	PosePlanner::Data::createRender();
	{
		golem::CriticalSectionWrapper csw(owner->getCS());
		owner->debugRenderer.render();
		owner->sampleRenderer.render();
	}
}

void golem::R2GPlanner::Data::load(const std::string& prefix, const golem::XMLContext* xmlcontext, const golem::data::Handler::Map& handlerMap) {
	PosePlanner::Data::load(prefix, xmlcontext, handlerMap);
}

void golem::R2GPlanner::Data::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	PosePlanner::Data::save(prefix, xmlcontext);
}

//------------------------------------------------------------------------------

void R2GPlanner::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	PosePlanner::Desc::load(context, xmlcontext);
	
	xmlcontext = xmlcontext->getContextFirst("rag_planner");

	golem::XMLData("planning_uncertainty", uncEnable, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("single_grasp_attempt", singleGrasp, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("withdraw_to_home_pose", withdrawToHomePose, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData(fLimit, xmlcontext->getContextFirst("limit"), false);

	//sensorBundleDesc->load(context, xmlcontext);
	try {
		golem::XMLData("active_ctrl", activeCtrlStr, const_cast<golem::XMLContext*>(xmlcontext));
	}
	catch (const Message& msg) {
		context.info("Active control non selected. Used default %s", activeCtrlStr.c_str());
	}
	golem::XMLData(*objCollisionDescPtr, xmlcontext->getContextFirst("collision"));
}

//------------------------------------------------------------------------------

R2GPlanner::R2GPlanner(Scene &scene) : PosePlanner(scene) {
}

golem::R2GPlanner::~R2GPlanner() {
}

bool R2GPlanner::create(const Desc& desc) {
	PosePlanner::create(desc); // throws

	enableSimContact = true;
	enableForceReading = false;
	forcereadersilent = true;
	objectPointCloudPtr.reset(new golem::Cloud::PointSeq());
	printing = false;
	contactOccured = false;

	// ACTIVE CONTROLLER
	const golem::ActiveCtrl::Map::const_iterator armHandCtrlPtr = activectrlMap.find(desc.activeCtrlStr);
	if (armHandCtrlPtr == activectrlMap.end())
		throw Message(Message::LEVEL_ERROR, "R2GPlanner::create(): active ctrl %s not found", desc.activeCtrlStr.c_str());
	//	armHandForce = dynamic_cast<ArmHandForce*>(&*armHandCtrlPtr->second);
	armHandForce = dynamic_cast<ActiveTouchCtrl*>(&*armHandCtrlPtr->second);
	if (!armHandForce)
		throw Message(Message::LEVEL_ERROR, "R2GPlanner::create(): active ctrl %s is invalid", desc.activeCtrlStr.c_str());
	armHandForce->getArmCtrl()->setMode(ActiveCtrlForce::Mode::MODE_DISABLED);
	armHandForce->getHandCtrl()->setMode(ActiveCtrlForce::Mode::MODE_ENABLED);
	armMode = armHandForce->getArmCtrl()->getMode();
	handMode = armHandForce->getHandCtrl()->getMode();
	context.write("Active ctrl %s mode [arm/hand]: %s/%s\n", desc.activeCtrlStr.c_str(), ActiveCtrlForce::ModeName[armMode], ActiveCtrlForce::ModeName[handMode]);

	// SET FT SENSORS AND GUARDS
	fLimit = desc.fLimit;
	context.write("Thumb limits [%.3f %.3f %.3f %.3f %.3f %.3f]\n",
		fLimit[0], fLimit[1], fLimit[2], fLimit[3], fLimit[4], fLimit[5]);
	context.write("Index limits [%.3f %.3f %.3f %.3f %.3f %.3f]\n",
		fLimit[6], fLimit[7], fLimit[8], fLimit[9], fLimit[10], fLimit[11]);
	context.write("Middle limits [%.3f %.3f %.3f %.3f %.3f %.3f]\n",
		fLimit[12], fLimit[13], fLimit[14], fLimit[15], fLimit[16], fLimit[17]);
	
	collisionPtr.reset();
	collisionPtr = desc.objCollisionDescPtr->create(*this->manipulator.get());

	Sensor::Seq sensorSeq = armHandForce->getSensorSeq();
	// NOTE: skips the sensor at the wrist
	for (auto i = sensorSeq.begin(); i != sensorSeq.end(); ++i) {
		//		if (i == sensorSeq.begin()) continue;
		FT* sensor = golem::is<FT>(*i);
		if (sensor)
		if (i == sensorSeq.begin())
			wristFTSensor = sensor;
		else
			ftSensorSeq.push_back(sensor);
		else
			context.write("FT sensor is not available\n");
	}
	// set FT guard for the thumb
	Chainspace::Index chain = getPlanner().handInfo.getChains().begin();
	FTGuard::Desc thumbFTDesc = FTGuard::Desc();
	thumbFTDesc.chain = HandChain::THUMB;
	thumbFTDesc.jointIdx = getPlanner().handInfo.getJoints(chain++).end() - 1;
	thumbFTDesc.setLimits(&fLimit[0]);
	ftGuards.push_back(thumbFTDesc.create());
	// set FT guard for the index
	FTGuard::Desc indexFTDesc = FTGuard::Desc();
	indexFTDesc.chain = HandChain::INDEX;
	indexFTDesc.jointIdx = getPlanner().handInfo.getJoints(chain++).end() - 1;
	indexFTDesc.setLimits(&fLimit[6]);
	ftGuards.push_back(indexFTDesc.create());
	// set FT guard for the thumb
	FTGuard::Desc middleFTDesc = FTGuard::Desc();
	middleFTDesc.chain = HandChain::MIDDLE;
	middleFTDesc.jointIdx = getPlanner().handInfo.getJoints(chain++).end() - 1;
	middleFTDesc.setLimits(&fLimit[12]);
	ftGuards.push_back(middleFTDesc.create());

	// set the call back to ahndle ground truth
	simulateHandlerCallback = [&](const golem::Cloud::PointSeq& points) {
		collisionPtr->create(rand, points);
		objectPointCloudPtr.reset(new golem::Cloud::PointSeq(points));
	};

	bool ftsensors = !ftSensorSeq.empty();
	const golem::Sensor::Map::const_iterator openNIPtr = sensorMap.find("OpenNI+OpenNI");
	enableSimContact = !ftsensors && (openNIPtr == sensorMap.end());
	armHandForce->setSensorForceReader([&](const golem::Controller::State& state, golem::RealSeq& force) { // throws
		if (!enableForceReading)
			return;

		if (enableSimContact && !collisionPtr->getPoints().empty()) {
			for (auto i = 0; i < force.size(); ++i)
				force[i] = collisionPtr->getFTBaseSensor().ftMedian[i] + (2 * rand.nextUniform<Real>()*collisionPtr->getFTBaseSensor().ftStd[i] - collisionPtr->getFTBaseSensor().ftStd[i]);

			golem::Controller::State dflt = lookupState();
			(void)collisionPtr->simulateFT(debugRenderer, desc.objCollisionDescPtr->flannDesc, rand, manipulator->getConfig(dflt), force, false);
		}
		// read from the state variable (if supported)
		else {
			if (ftsensors) {
				size_t k = 0;
				for (auto i = ftSensorSeq.begin(); i < ftSensorSeq.end(); ++i) {
					golem::FT::Data data;
					(*i)->read(data);
					data.wrench.v.getColumn3(&force[k]);
					data.wrench.w.getColumn3(&force[k + 3]);
					k += 6;
				}
			}
			else {
				const ptrdiff_t forceOffset = armHandForce->getHand()->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
				if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
					for (Configspace::Index j = state.getInfo().getJoints().begin(); j < state.getInfo().getJoints().end(); ++j) {
						const size_t k = j - state.getInfo().getJoints().begin();
						force[k] = state.get<ConfigspaceCoord>(forceOffset)[j];
					}
				}
			}
		}
	}); // end robot->setSimHandForceReader

	armHandForce->setHandForceReader([&](const golem::Controller::State& state, golem::RealSeq& force) { // throws
		if (!enableForceReading)
			return;

		size_t k = 0;
		U32 contacts = golem::numeric_const<U32>::ZERO;
		handFilteredForce = force;

		for (auto i = ftGuards.begin(); i < ftGuards.end(); ++i) {
			(*i)->setColumn6(&handFilteredForce[k]);
			if ((*i)->checkContacts())
				++contacts;
			k += 6;
		}

		if (contacts > 0) {
			for (auto i = ftGuards.begin(); i < ftGuards.end(); ++i)
				(*i)->str(context);
			throw Message(Message::LEVEL_NOTICE, "handForceReader(): Triggered guard(s) %d.\n%s", contacts);
		}
	}); // end robot->setHandForceReader

	armHandForce->setEmergencyModeHandler([=]() {
		enableForceReading = false;
		contactOccured = true;
		armHandForce->getArmCtrl()->setMode(armMode);
		armHandForce->getHandCtrl()->setMode(handMode);
	}); // end robot->setEmergencyModeHandler

	uncEnable = desc.uncEnable;
	singleGrasp = desc.singleGrasp;
	withdrawToHomePose = desc.withdrawToHomePose;
	posterior = true;

	// initialise data handlers for R2GTrajectories
	for (golem::data::Handler::Map::const_iterator i = handlerMap.begin(); i != handlerMap.end(); ++i) {
		data::HandlerR2GPlan* handlerPlan = is<data::HandlerR2GPlan>(i);
		if (handlerPlan) {
			if (handlerPlan->getPlannerIndex() >= plannerInfoSeq.size())
				throw Message(Message::LEVEL_CRIT, "Player::create(): %s: Invalid planner index %u", i->second->getID().c_str(), handlerPlan->getPlannerIndex());
			PlannerInfo& plannerInfo = plannerInfoSeq[handlerPlan->getPlannerIndex()];
			handlerPlan->set(*plannerInfo.planner, plannerInfo.controllerIDSeq);
		}
	}

	ragDesc = desc;

	handBounds.clear();

	auto executeCmd = [&](const std::string command) {
		MenuCmdMap::const_iterator cmd = menuCmdMap.find(command);
		if (cmd == menuCmdMap.end())
			throw Cancel(makeString("Error: impossible to execute command %s.", command.c_str()).c_str());
		cmd->second();
	};

	menuCmdMap.insert(std::make_pair("L", [=]() {
		auto select = [&](Strategy &strat) {
			switch (option("NEMI", "Press a key to select (N)one/(E)lementary/(M)ycroft/(I)r3ne...")) {
			case 'N':
			{
				strat = Strategy::NONE_STRATEGY;
				return;
			}
			case 'E':
			{
				strat = Strategy::ELEMENTARY;
				return;
			}
			case 'M':
			{
				strat = Strategy::MYCROFT;
				return;
			}
			case 'I':
			{
				strat = Strategy::IR3NE;
				return;
			}
			}
		};
		to<Data>(dataCurrentPtr)->createRender();
		// set the simulated object
		if (!to<Data>(dataCurrentPtr)->simulateObjectPose.empty()) {
			//sensorBundlePtr->getCollisionPtr()->create(rand, golem::to<Data>(dataCurrentPtr)->simulateObjectPose);
			collisionPtr->create(rand, golem::to<Data>(dataCurrentPtr)->simulateObjectPose);
			objectPointCloudPtr.reset(new golem::Cloud::PointSeq(golem::to<Data>(dataCurrentPtr)->simulateObjectPose));
		}

		golem::data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<golem::data::Item::Map::const_iterator>(true);
		golem::data::Trajectory* trajectory = is<golem::data::Trajectory>(item->second.get());
		if (!trajectory)
			throw Cancel("Error: no trajectory selected.");
		// play
		//Controller::State::Seq inp = trajectory->getWaypoints();
		golem::WaypointCtrl::Seq inp = trajectory->getWaypoints();
		if (inp.size() < 3)
			throw Cancel("Error: the selected trajectory have not at least 3 waypoints.");
		
		to<Data>(dataCurrentPtr)->actionType = action::NONE_ACTION;
		select(to<Data>(dataCurrentPtr)->stratType);
		for (;;) {
			if (!execute(dataCurrentPtr, inp))
				return;
			if (contactOccured) {
				//golem::to<Data>(cdata->second)->replanning = false;
				contactOccured = false;
				updateAndResample(dataCurrentPtr);
				enableForceReading = false;
				continue;
			}
			// grasp
			enableForceReading = false;
			//golem::to<Data>(dataPtr)->actionType = action::GRASP;
			context.write("execute trajectory (%s)\n", actionToString(golem::to<Data>(dataCurrentPtr)->actionType).c_str());
			if (!execute(dataCurrentPtr, inp))
				return;
					
			// lifting
			enableForceReading = false;
			//golem::to<Data>(dataPtr)->actionType = action::IG_PLAN_LIFT;
			context.write("execute trajectory (%s)\n", actionToString(golem::to<Data>(dataCurrentPtr)->actionType).c_str());
			if (execute(dataCurrentPtr, inp))
				return;
		
			break;
		}

	//finish
	context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("N", [=]() {
		context.write("Test collision detection\n");
		golem::data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<golem::data::Item::Map::const_iterator>(true);
		golem::data::Trajectory* trajectory = is<golem::data::Trajectory>(item->second.get());
		if (!trajectory)
			throw Cancel("Error: no trajectory selected.");
		// play
		golem::WaypointCtrl::Seq inp = trajectory->getWaypoints();
		if (inp.size() < 3)
			throw Cancel("Error: the selected trajectory have not at least 3 waypoints.");
		pHeuristic->enableUnc = false;
		golem::to<Data>(dataCurrentPtr)->actionType = action::NONE_ACTION;

		Controller::State cend = lookupState();
		// transform w.r.t. query frame
		findTarget(golem::to<Data>(dataCurrentPtr)->queryTransform, golem::to<Data>(dataCurrentPtr)->modelFrame, inp[2].state, cend);

		golem::data::Handler* handler = modelHandler;
		golem::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(modelItem);
		if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
			throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not find %s.", modelItem.c_str());

		// retrive point cloud with curvature
		golem::data::ItemFeature3D *Feature3D = is<golem::data::ItemFeature3D>(ptr->second.get());
		if (!Feature3D)
			throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not support PointdCurv interface.");
		golem::Cloud::PointFeatureSeq curvPoints = *Feature3D->cloud;

		// copy as a vector of points in 3D
		Vec3Seq mPoints;
		mPoints.resize(curvPoints.size());
		I32 idx = 0;
		std::for_each(curvPoints.begin(), curvPoints.end(), [&](const Cloud::Point& i){
			mPoints[idx++] = Cloud::getPoint<Real>(i);
		});

		// copy as a generic point+normal point cloud (Cloud::pointSeq)
		Cloud::PointSeq points;
		points.resize(curvPoints.size());
		Cloud::copy(curvPoints, points, Cloud::copyPointXYZ<Cloud::PointFeature, Cloud::Point>);

		//sensorBundlePtr->getCollisionPtr()->create(rand, points);
		collisionPtr->create(rand, points);
		objectPointCloudPtr.reset(new golem::Cloud::PointSeq(points));

		HBCollision::FlannDesc waypointDesc;
		HBCollision::Desc::Ptr cloudDesc;
		cloudDesc.reset(new HBCollision::Desc());
		HBCollision::Ptr cloud = cloudDesc->create(*manipulator);
		waypointDesc.depthStdDev = 0.01/*0.0005*/; waypointDesc.likelihood = 1000.0; waypointDesc.points = 10000; waypointDesc.neighbours = 100;
		waypointDesc.radius = REAL_ZERO;

		golem::Manipulator::Config config(inp[2].state.cpos, manipulator->getBaseFrame(inp[2].state.cpos));
		debugRenderer.reset();
//		debugAppearance.draw(points, debugRenderer);
		//RealSeq ff = {0., 0., -0.1, 0., 0., 0.};
		//ftGuards[1]->setColumn6(&ff[0]);
		//ftGuards[1]->checkContacts();
		cloud->create(rand, points);
		(void)cloud->evaluateFT(debugRenderer, waypointDesc, config, ftGuards, true);

		if (option("YN", "Resample Next? (Y/N)") == 'Y')
			return;


		Controller::State::Seq approach;
		(void)findTrajectory(lookupState(), &cend, nullptr, 0, approach);
		context.write("path dist delta %f\n", pHeuristic->getDesc().collisionDesc.pathDistDelta);
		Real delta = 0.0025;//pHeuristic->getDesc().collisionDesc.pathDistDelta;
		for (auto j = approach.begin(); j != approach.end(); ++j) {
//			const bool res = (*pBelief->getHypotheses().begin())->check(pHeuristic->ftDrivenDesc.checkDesc, rand, manipulator->getConfig(*i), true);
			if (j != approach.end() - 1) {
				golem::Waypoint w0, w1;
				w1.cpos = j->cpos; w0.cpos = (j + 1)->cpos;
				const Real dist = pHeuristic->getDist(w0, w1);
				const U32 size = (U32)Math::round(dist / delta) + 1;
				Real p[2];
				golem::Waypoint w;

				// test for collisions in the range (w0, w1) - excluding w0 and w1
				for (U32 i = 1; i < size;) {
					p[0] = Real(i) / Real(size);
					p[1] = REAL_ONE - p[0];

					// lineary interpolate coordinates
					for (Configspace::Index l = getPlanner().armInfo.getJoints().begin(); l < getPlanner().armInfo.getJoints().end(); ++l)
						w.cpos[l] = p[0] * w0.cpos[l] + p[1] * w1.cpos[l];
					for (Configspace::Index l = getPlanner().handInfo.getJoints().begin(); l < getPlanner().handInfo.getJoints().end(); ++l)
						w.cpos[l] = p[0] * w0.cpos[l] + p[1] * w1.cpos[l];

					// skip reference pose computation
					w.setup(*controller, false, true);
					const golem::Manipulator::Config config(w.cpos, manipulator->getBaseFrame(w.cpos));
					Bounds::Seq bounds = manipulator->getBounds(config.config, config.frame.toMat34());
					renderHand(*j, bounds, true);
					const bool res = (*pBelief->getHypotheses().begin())->checkNN(pHeuristic->ftDrivenDesc.checkDesc, config.config, true);
					if (option("PN", "(N)ext, (P)revious") == 'N') {
						if (i < size) ++i;
						else {
							if (option("YN", "Exit? (Y/N)") == 'Y')
								i = size;
						}
					}
					else { // previous
						if (i > 0) --i;
						else {
							if (option("YN", "Exit? (Y/N)") == 'Y')
								i = size;
						}

					}
				}
			}
		}
		context.write("done.\n");
		return;
	}));

	ragDesc = desc;

	return true;
}

//------------------------------------------------------------------------------

void R2GPlanner::render() const {
	PosePlanner::render();
//	handRenderer.reset();
	{
		golem::CriticalSectionWrapper csw(getCS());
		debugRenderer.render();
		sampleRenderer.render();
		//sensorBundlePtr->render();
	}
}

void R2GPlanner::renderHand(const golem::Controller::State &state, const Bounds::Seq &bounds, bool clear) {
	{
		golem::CriticalSectionWrapper csw(getCS());
		debugRenderer.reset();
		if (clear)
			handBounds.clear();
		if (!bounds.empty())
			handBounds.insert(handBounds.end(), bounds.begin(), bounds.end());
		debugRenderer.setColour(RGBA::BLACK);
		debugRenderer.setLineWidth(Real(2.0));
		debugRenderer.addWire(handBounds.begin(), handBounds.end());
	}
}

//------------------------------------------------------------------------------

void R2GPlanner::findTarget(const golem::Mat34 &trn, const golem::Controller::State &target, golem::Controller::State &cend, const bool lifting) {
	// arm chain and joints pointers
	const golem::Chainspace::Index armChain = getPlanner().armInfo.getChains().begin();
	const golem::Configspace::Range armJoints = getPlanner().armInfo.getJoints();
	// Compute a sequence of targets corresponding to the transformed arm end-effector
	GenWorkspaceChainState gwcs;
	controller->chainForwardTransform(target.cpos, gwcs.wpos);
	gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1
	gwcs.wpos[armChain].multiply(trn, gwcs.wpos[armChain]); // new waypoint frame
	gwcs.t = target.t;
	//	gwcs.wpos[armChain].p.x += 0.045;
	if (lifting) gwcs.wpos[armChain].p.z += 0.07;

	cend = target;
	{
		// Find initial target position
		if (!getPlanner().planner->findTarget(target, gwcs, cend))
			throw Message(Message::LEVEL_ERROR, "golem::Robot::findTarget(): Unable to find initial target configuration");
	}
	//	context.write(">\n"); //std::cout << ">\n"; //context.write(">\n");
	// set fingers pose
	for (auto i = getPlanner().handInfo.getJoints().begin(); i != getPlanner().handInfo.getJoints().end(); ++i)
		cend.cpos[i] = target.cpos[i];

	// update arm configurations and compute average error
	golem::RBDist err;
	WorkspaceChainCoord wcc;
	controller->chainForwardTransform(cend.cpos, wcc);
	wcc[armChain].multiply(wcc[armChain], controller->getChains()[armChain]->getReferencePose());
	err.add(err, golem::RBDist(golem::RBCoord(wcc[armChain]), golem::RBCoord(gwcs.wpos[armChain])));
	context.write("Robot::findTarget(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
}

void R2GPlanner::findTarget(const golem::Mat34& queryTrn, const golem::Mat34& modelFrame, const golem::Controller::State& target, golem::Controller::State& cend, const bool lifting) {
	// arm chain and joints pointers
	const golem::Chainspace::Index armChain = getPlanner().armInfo.getChains().begin();
	const golem::Configspace::Range armJoints = getPlanner().armInfo.getJoints();
	// Compute a sequence of targets corresponding to the transformed arm end-effector
	GenWorkspaceChainState gwcs;
	controller->chainForwardTransform(target.cpos, gwcs.wpos);
	gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1

	Mat34 poseFrameInv, graspFrame, graspFrameInv;
	poseFrameInv.setInverse(gwcs.wpos[armChain]);
	graspFrame.multiply(poseFrameInv, modelFrame);
	graspFrameInv.setInverse(graspFrame);
	//gwcs.wpos[armChain].multiply(queryTrn, graspFrameInv);
	gwcs.wpos[armChain].multiply(modelFrame, graspFrameInv);
	gwcs.wpos[armChain].multiply(queryTrn, gwcs.wpos[armChain]);
	gwcs.t = target.t;
//	gwcs.wpos[armChain].p.z -= 0.007;
	if (lifting) gwcs.wpos[armChain].p.z += 0.07;

	cend = target;
	{
		// Find initial target position
		if (!getPlanner().planner->findTarget(target, gwcs, cend))
			throw Message(Message::LEVEL_ERROR, "golem::Robot::findTarget(): Unable to find initial target configuration");
	}
	//	context.write(">\n"); //std::cout << ">\n"; //context.write(">\n");
	// set fingers pose
	for (auto i = getPlanner().handInfo.getJoints().begin(); i != getPlanner().handInfo.getJoints().end(); ++i)
		cend.cpos[i] = target.cpos[i];

	// update arm configurations and compute average error
	golem::RBDist err;
	WorkspaceChainCoord wcc;
	controller->chainForwardTransform(cend.cpos, wcc);
	wcc[armChain].multiply(wcc[armChain], controller->getChains()[armChain]->getReferencePose());
	err.add(err, golem::RBDist(golem::RBCoord(wcc[armChain]), golem::RBCoord(gwcs.wpos[armChain])));
	context.write("Robot::findTarget(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
}

//------------------------------------------------------------------------------

void R2GPlanner::createTrajectory(const golem::Controller::State& begin, const golem::Controller::State* pcend, const golem::Mat34* pwend, golem::SecTmReal t, const golem::Controller::State::Seq& waypoints, golem::Controller::State::Seq& trajectory) {
	if (!pcend && !pwend)
		throw Message(Message::LEVEL_ERROR, "Robot::findTrajectory(): no target specified");

	// Trajectory from initial position to end position
	for (golem::U32 i = 0; i < getPlanner().trajectoryTrials; ++i) {
		if (universe.interrupted())
			throw Exit();
		context.debug("Player::findTrajectory(): Planning movement...\n");
		// All bounds are treated as obstacles
		uiPlanner->setCollisionBoundsGroup(Bounds::GROUP_ALL);

		// Setup configspace target
		Controller::State cend = pcend ? *pcend : begin;
		// Workspace target
		if (pwend) {
			// Setup workspace target
			GenWorkspaceChainState wend;
			wend.setToDefault(info.getChains().begin(), info.getChains().end()); // all used chains
			wend.wpos[getPlanner().armInfo.getChains().begin()] = *pwend;
			// planner debug
			//context.verbose("%s\n", plannerWorkspaceDebug(*planner, &wend.wpos).c_str());
			if (!getPlanner().planner->findTarget(begin, wend, cend))
				continue;
			// update configspace coords of the hand
			if (pcend) cend.cpos.set(getPlanner().handInfo.getJoints(), pcend->cpos);
			// error
			WorkspaceChainCoord wcc;
			controller->chainForwardTransform(cend.cpos, wcc);
			wcc[getPlanner().armInfo.getChains().begin()].multiply(wcc[getPlanner().armInfo.getChains().begin()], controller->getChains()[getPlanner().armInfo.getChains().begin()]->getReferencePose());
			golem::RBDist err;
			err.set(golem::RBCoord(*pwend), golem::RBCoord(wcc[getPlanner().armInfo.getChains().begin()]));
			context.debug("Robot::findTrajectory(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
		}

		// planner debug
		//context.verbose("%s\n", plannerConfigspaceDebug(*planner, &cend.cpos).c_str());
		// Find collision-free trajectory and wait until the device is ready for new commands
		cend.t = begin.t + (t > SEC_TM_REAL_ZERO ? t : getPlanner().trajectoryDuration);
		if (getPlanner().planner->findGlobalTrajectory(begin, cend, trajectory, trajectory.begin()))
			return;// success
	}

	throw Message(Message::LEVEL_ERROR, "Robot::findTrajectory(): unable to find trajectory");
}

golem::RBDist R2GPlanner::trnTrajectory(const golem::Mat34& actionFrame, const golem::Mat34& modelFrame, const golem::Mat34& trn, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory) {
	if (begin == end)
		throw Message(Message::LEVEL_ERROR, "Robot::createTrajectory(2): Empty input trajectory");
	// arm chain and joints pointers
	const golem::Chainspace::Index armChain = getPlanner().armInfo.getChains().begin();
	const golem::Configspace::Range armJoints = getPlanner().armInfo.getJoints();
	// Compute a sequence of targets corresponding to the transformed arm end-effector
	GenWorkspaceChainState::Seq seq;

	for (Controller::State::Seq::const_iterator i = begin; i != end; ++i) {
		GenWorkspaceChainState gwcs;
		controller->chainForwardTransform(i->cpos, gwcs.wpos);
		gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1
		// define the grasp frame
		Mat34 poseFrameInv, graspFrame, graspFrameInv;
		poseFrameInv.setInverse(gwcs.wpos[armChain]);
		graspFrame.multiply(poseFrameInv, actionFrame * modelFrame);
		graspFrameInv.setInverse(graspFrame);
		gwcs.wpos[armChain].multiply(modelFrame, graspFrameInv);
		//		context.write("trnTrajectory(): grasp frame at model <%f %f %f>\n", gwcs.wpos[armChain].p.x, gwcs.wpos[armChain].p.y, gwcs.wpos[armChain].p.z);
		gwcs.wpos[armChain].multiply(trn, gwcs.wpos[armChain]); // new waypoint frame
		//		context.write("trnTrajectory(): grasp frame at new query <%f %f %f>\n", gwcs.wpos[armChain].p.x, gwcs.wpos[armChain].p.y, gwcs.wpos[armChain].p.z);	
		gwcs.t = i->t;
		seq.push_back(gwcs);
	}
	// planner debug
	//context.verbose("%s\n", plannerDebug(*planner).c_str());
	Controller::State::Seq ctrajectory;
	{
		// Find initial target position
		Controller::State cend = *begin;
		// planner debug
		//context.verbose("%s\n", plannerWorkspaceDebug(*planner, &seq[0].wpos).c_str());
		if (!getPlanner().planner->findTarget(*begin, seq[0], cend))
			throw Message(Message::LEVEL_ERROR, "Robot::createTrajectory(2): Unable to find initial target configuration");
		// Find remaining position sequence
		if (seq.size() == 1)
			ctrajectory.push_back(cend);
		else if (seq.size() > 1 && !getPlanner().planner->findLocalTrajectory(cend, ++seq.begin(), seq.end(), ctrajectory, ctrajectory.end()))
			throw Message(Message::LEVEL_ERROR, "Robot::createTrajectory(2): Unable to find trajectory");
	}
	// update arm configurations and compute average error
	golem::RBDist err;
	Controller::State::Seq::const_iterator j = begin;
	for (size_t i = 0; i < ctrajectory.size(); ++i, ++j) {
		// copy config
		trajectory.push_back(*j);
		trajectory.back().set(getPlanner().armInfo.getJoints().begin(), getPlanner().armInfo.getJoints().end(), ctrajectory[i]);
		// error
		WorkspaceChainCoord wcc;
		controller->chainForwardTransform(trajectory.back().cpos, wcc);
		wcc[armChain].multiply(wcc[armChain], controller->getChains()[armChain]->getReferencePose());
		err.add(err, golem::RBDist(golem::RBCoord(wcc[armChain]), golem::RBCoord(seq[i].wpos[armChain])));
	}
	context.debug("Robot::createTrajectory(2): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);

	return err;
}

golem::RBDist R2GPlanner::findTrnTrajectory(const golem::Mat34& trn, const golem::Controller::State& startPose, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory) {
	if (begin == end)
		throw Message(Message::LEVEL_ERROR, "Robot::transformTrajectory(): Empty input trajectory");
	// arm chain and joints pointers
	const golem::Chainspace::Index armChain = getPlanner().armInfo.getChains().begin();
	const golem::Configspace::Range armJoints = getPlanner().armInfo.getJoints();
	// Compute a sequence of targets corresponding to the transformed arm end-effector
	GenWorkspaceChainState::Seq seq;
	//// the starting pose of the robot does not need a tranformation
	//GenWorkspaceChainState gStart;
	//controller->chainForwardTransform(startPose.cpos, gStart.wpos);
	//gStart.wpos[armChain].multiply(gStart.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1
	//gStart.t = startPose.t;
	//seq.push_back(gStart);
	for (Controller::State::Seq::const_iterator i = begin; i != end; ++i) {
		GenWorkspaceChainState gwcs;
		controller->chainForwardTransform(i->cpos, gwcs.wpos);
		gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1
		gwcs.wpos[armChain].multiply(trn, gwcs.wpos[armChain]); // new waypoint frame
		gwcs.t = i->t;
		seq.push_back(gwcs);
	}
	// planner debug
	//context.verbose("%s\n", plannerDebug(*planner).c_str());
	Controller::State::Seq initTrajectory, ctrajectory;
	{
		// Find initial target position
		Controller::State cend = *begin;
		// planner debug
		//context.debug("Seq[0]: %s\n", golem::plannerWorkspaceDebug(*planner, &seq[0].wpos).c_str());
		//context.debug("Seq[2]: %s\n", golem::plannerWorkspaceDebug(*planner, &seq[2].wpos).c_str());
		if (!getPlanner().planner->findTarget(*begin, seq[0], cend))
			throw Message(Message::LEVEL_ERROR, "Robot::transformTrajectory(): Unable to find initial target configuration");
		// compute the initial trajectory to move thee robot from current pose to the beginning of the approach trajectory
		if (!getPlanner().planner->findGlobalTrajectory(startPose, cend, initTrajectory, initTrajectory.end()))
			throw Message(Message::LEVEL_ERROR, "Robot::transformTrajectory(): Unable to find initial trajectory");
		// Find remaining position sequence
		if (seq.size() == 1)
			ctrajectory.push_back(cend);
		else if (seq.size() > 1 && !getPlanner().planner->findLocalTrajectory(cend, ++seq.begin(), seq.end(), ctrajectory, ctrajectory.end()))
			throw Message(Message::LEVEL_ERROR, "Robot::transformTrajectory(): Unable to find trajectory");
	}
	// update arm configurations and compute average error
	trajectory.insert(trajectory.end(), initTrajectory.begin(), initTrajectory.end());
	golem::RBDist err;
	Controller::State::Seq::const_iterator j = begin;
	for (size_t i = 0; i < ctrajectory.size(); ++i, ++j) {
		// copy config
		trajectory.push_back(*j);
		trajectory.back().set(getPlanner().armInfo.getJoints().begin(), getPlanner().armInfo.getJoints().end(), ctrajectory[i]);
		// error
		WorkspaceChainCoord wcc;
		controller->chainForwardTransform(trajectory.back().cpos, wcc);
		wcc[armChain].multiply(wcc[armChain], controller->getChains()[armChain]->getReferencePose());
		err.add(err, golem::RBDist(golem::RBCoord(wcc[armChain]), golem::RBCoord(seq[i].wpos[armChain])));
	}
	context.debug("Robot::transformTrajectory(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
	return err;
}

//------------------------------------------------------------------------------

void R2GPlanner::perform(const std::string& data, const std::string& item, const golem::Controller::State::Seq& trajectory, bool testTrajectory) {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): At least two waypoints required");

	golem::Controller::State::Seq initTrajectory;
	findTrajectory(golem::WaypointCtrl::lookup(*controller).state, &trajectory.front(), nullptr, SEC_TM_REAL_ZERO, initTrajectory);

	golem::Controller::State::Seq completeTrajectory = initTrajectory;
	completeTrajectory.insert(completeTrajectory.end(), trajectory.begin(), trajectory.end());

	// create trajectory item
	golem::data::Item::Ptr itemTrajectory;
	golem::data::Handler::Map::const_iterator handlerPtr = handlerMap.find(getPlanner().trajectoryHandler);
	if (handlerPtr == handlerMap.end())
		throw Message(Message::LEVEL_ERROR, "Player::perform(): unknown default trajectory handler %s", getPlanner().trajectoryHandler.c_str());
	golem::data::Handler* handler = is<golem::data::Handler>(handlerPtr);
	if (!handler)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): invalid default trajectory handler %s", getPlanner().trajectoryHandler.c_str());
	itemTrajectory = handler->create();
	golem::data::Trajectory* trajectoryIf = is<golem::data::Trajectory>(itemTrajectory.get());
	if (!trajectoryIf)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): unable to create trajectory using handler %s", getPlanner().trajectoryHandler.c_str());
	trajectoryIf->setWaypoints(golem::WaypointCtrl::make(completeTrajectory, completeTrajectory)/*golem::WaypointCtrl::make(trajectory, trajectory)*/);

	// block displaying the current item
	RenderBlock renderBlock(*this);

	// test trajectory
	if (testTrajectory) {
		// insert trajectory to data with temporary name
		const std::string itemLabelTmp = item + dataDesc->sepName + makeString("%f", context.getTimer().elapsed());
		ScopeGuard removeItem([&]() {
			UI::removeCallback(*this, getCurrentHandler());
			{
				golem::CriticalSectionWrapper csw(getCS());
				to<Data>(dataCurrentPtr)->itemMap.erase(itemLabelTmp);
			}
			to<Data>(dataCurrentPtr)->createRender();
		});
		{
			golem::CriticalSectionWrapper csw(getCS());
			const golem::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), golem::data::Item::Map::value_type(itemLabelTmp, itemTrajectory));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// enable GUI interaction and refresh
		UI::addCallback(*this, getCurrentHandler());
		to<Data>(dataCurrentPtr)->createRender();
		// prompt user
		EnableKeyboardMouse enableKeyboardMouse(*this);
		option("\x0D", "Press <Enter> to accept trajectory...");
	}

	bool emergencyMode = armHandForce->getHandCtrl()->getMode();

	// go to initial state
	sendTrajectory(initTrajectory);
	// wait until the last trajectory segment is sent
	controller->waitForEnd();

	{
		// start recording
		recordingStart(data, item, true);
		recordingWaitToStart();
		ScopeGuard recordingGuard([&]() {
			// stop recording
			recordingStop(getPlanner().trajectoryIdlePerf);
			recordingWaitToStop();
		});

		// send trajectory
		sendTrajectory(trajectory);

		// repeat every send waypoint until trajectory end
		for (U32 i = 0; controller->waitForBegin(); ++i) {
			if (universe.interrupted())
				throw Exit();
			if (controller->waitForEnd(0))
				break;

			Controller::State state = lookupState();

			// print every 10th robot state
			if (i % 10 == 0) {
				context.write("State #%d (%s)\r", i, enableForceReading ? "Y" : "N");

				if (emergencyMode && i > 50) {
					armHandForce->getArmCtrl()->setMode(armMode);
					armHandForce->getHandCtrl()->setMode(handMode);
					emergencyMode = false;
				}

				if (golem::to<Data>(dataCurrentPtr)->actionType != action::GRASP && i > 500) {
					enableForceReading = expectedCollisions(state);
				}
			}
		}
	}
	enableForceReading = false;

	// insert trajectory
	{
		golem::CriticalSectionWrapper csw(getCS());
		golem::data::Data::Map::iterator data = dataMap.find(recorderData);
		if (data == dataMap.end())
			throw Message(Message::LEVEL_ERROR, "Player::perform(): unable to find Data %s", recorderData.c_str());
		data->second->itemMap.insert(std::make_pair(recorderItem + makeString("%s%.3f", dataDesc->sepName.c_str(), recorderStart), itemTrajectory));
	}
	context.write("Performance finished!\n");
}

//------------------------------------------------------------------------------

bool R2GPlanner::execute(golem::data::Data::Map::iterator dataPtr, golem::WaypointCtrl::Seq& trajectory) {
	if (trajectory.size() < 3)
		throw Cancel("Error: the selected trajectory have not at least 3 waypoints.");

	const golem::Chainspace::Index armChain = getPlanner().armInfo.getChains().begin();
	bool silent = to<Data>(dataPtr)->actionType != action::NONE_ACTION;
//	context.debug("execute(): silen=%s, actionType=%s\n", silent ? "TRUE" : "FALSE", actionToString(golem::to<Data>(dataPtr)->actionType));
	const int key = !silent ? option("MQTGU", "Press to (M)odel based or (Q)uery based grasp, (T)rajectory based planner, (G)rasp, (U)p lifting") :
		golem::to<Data>(dataPtr)->actionType == action::IG_PLAN_M2Q ? 'M' :
		golem::to<Data>(dataPtr)->actionType == action::IG_PLAN_ON_QUERY ? 'Q' :
		golem::to<Data>(dataPtr)->actionType == action::IG_TRAJ_OPT ? 'T' :
		golem::to<Data>(dataPtr)->actionType == action::GRASP ? 'G' : 'U';

	// lamda function to reset flags for planning
	auto resetPlanning = [&]() {
		pHeuristic->testCollision = false; // debug collissions with point clouds
		pHeuristic->enableUnc = false; // plans trajectories with IG
		pHeuristic->setPointCloudCollision(false); // collision with mean pose hypothesis' point cloud
		enableForceReading = false; // enables/disable force reader
	};
	const U32 initIdx = 0, pregraspIdx = 1, graspIdx = 2;
	// init has the wrist pose of the 2nd element in the trajectory
	Controller::State init = trajectory[pregraspIdx].state;
	// and the fingers opened as the 1st element in the trajectory
	for (auto j = getPlanner().handInfo.getJoints().begin(); j != getPlanner().handInfo.getJoints().end(); ++j)
		init.cpos[j] = trajectory[initIdx].state.cpos[j];
	// pregrasp has the wrist pose of the 2nd element in the trajectory
	Controller::State pregrasp = trajectory[graspIdx].state;
	// and the fingers opened as the 1st element in the trajectory
	for (auto j = getPlanner().handInfo.getJoints().begin(); j != getPlanner().handInfo.getJoints().end(); ++j)
		pregrasp.cpos[j] = trajectory[initIdx].state.cpos[j];
	// grasp pose
	Controller::State grasp = trajectory[graspIdx].state;

	switch (key){
	case 'M':
	{
		context.debug("Plan from Model to Query\n");
		std::string trjItemName("modelR2GTrj");
		resetPlanning();
		pHeuristic->enableUnc = golem::to<Data>(dataPtr)->stratType == Strategy::IR3NE ? true : false;
		pHeuristic->setPointCloudCollision(true);

		Controller::State cend = lookupState();
		// transform w.r.t. query frame
		try {
			findTarget(golem::to<Data>(dataPtr)->queryTransform, golem::to<Data>(dataPtr)->modelFrame, trajectory.front().command, cend);
		}
		catch (const Message& msg) {
			context.getMessageStream()->write(msg);
			context.write("\n");
			resetPlanning();
			return false;
		}
		//golem::Manipulator::Config config(cend.cpos, manipulator->getBaseFrame(cend.cpos));
		//Bounds::Seq bounds = manipulator->getBounds(config.config, config.frame.toMat34());
		//renderHand(cend, bounds, true);

		Controller::State::Seq approach;
		try {
			(void)findTrajectory(golem::WaypointCtrl::lookup(*controller).command, &cend, nullptr, 0, approach);
		}
		catch (const Message& msg) {
			context.getMessageStream()->write(msg);
			context.write("\n");
			resetPlanning();
			return false;
		}

		Controller::State::Seq out = approach;

		// add trajectory waypoint
		golem::data::Item::Map::iterator ptr;
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(getCS());
			to<Data>(dataPtr)->itemMap.erase(trjItemName);
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), golem::data::Item::Map::value_type(trjItemName, modelHandlerTrj->create()));
			golem::data::Trajectory* trajectory = is<golem::data::Trajectory>(ptr);
			if (!trajectory)
				throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
			// add current states
			trajectory->setWaypoints(golem::WaypointCtrl::make(out, out));
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}

		//handBounds.clear();
		//debugRenderer.reset();
		//brecord = true;
		// perform
		try {
			perform(dataPtr->first, ptr->first, out, !silent);
		}
		catch (const Message& msg) {
			context.getMessageStream()->write(msg);
			context.write("\n");
			resetPlanning();
			return false;
		}
//		printf("Finished perform\n");
		resetPlanning();
		return true;
	}
	case 'Q':
	{
		context.debug("Plan to Query\n");
		std::string trjItemName("queryR2GTrj");
		resetPlanning();
		pHeuristic->enableUnc = golem::to<Data>(dataPtr)->stratType == Strategy::IR3NE ? true : false;
		// pre-grasp pose w.r.t. query frame
		Controller::State cend = trajectory[1].state;
		pHeuristic->setPointCloudCollision(true);
		Controller::State::Seq approach;
		findTrajectory(lookupState(), &cend, nullptr, 0, approach);
		Controller::State::Seq out = approach;

		// add trajectory waypoint
		golem::data::Item::Map::iterator ptr;
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(getCS());
			to<Data>(dataPtr)->itemMap.erase(trjItemName);
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), golem::data::Item::Map::value_type(trjItemName, queryHandlerTrj->create()));
			golem::data::Trajectory* trajectory = is<golem::data::Trajectory>(ptr);
			if (!trajectory)
				throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
			// add current state
			trajectory->setWaypoints(golem::WaypointCtrl::make(out, out));
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}

		// perform
		try {
			perform(dataPtr->first, ptr->first, out, !silent);
		}
		catch (const Message& msg) {
			context.getMessageStream()->write(msg);
			context.write("\n");
			resetPlanning();
			return false;
		}
		resetPlanning();

		return true;
	}
	case 'T':
	{
		context.debug("Plan trajectory optimisation\n");
		std::string trjItemName("queryTrj");
		pHeuristic->enableUnc = golem::to<Data>(dataPtr)->stratType == Strategy::IR3NE ? true : false;
		pHeuristic->setPointCloudCollision(true);

		Controller::State cstart = init, cend = pregrasp, cInit = lookupState();

		// transform w.r.t. query frame
		findTarget(golem::to<Data>(dataPtr)->queryTransform, init, cstart);
		findTarget(golem::to<Data>(dataPtr)->queryTransform, pregrasp, cend);

		//Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(cend), manipulator->getConfig(cend).toMat34());
		//renderHand(cend, bounds, true);

		Controller::State::Seq approach, seq;
		findTrajectory(cstart, &cend, nullptr, 0, approach);
		findTrajectory(cInit, &approach.front(), nullptr, 0, approach);
//		approach.insert(approach.end(), seq.begin(), seq.end());

		Controller::State::Seq out = approach;
		pHeuristic->setPointCloudCollision(true);
		
		// add trajectory waypoint
		golem::data::Item::Map::iterator ptr;
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(getCS());
			to<Data>(dataPtr)->itemMap.erase(trjItemName);
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), golem::data::Item::Map::value_type(trjItemName, modelHandlerTrj->create()));
			golem::data::Trajectory* trajectory = is<golem::data::Trajectory>(ptr);
			if (!trajectory)
				throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
			// add current state
			trajectory->setWaypoints(golem::WaypointCtrl::make(out, out));
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}

		// perform
		try {
			perform(dataPtr->first, ptr->first, out, !silent);
		}
		catch (const Message& msg) {
			context.getMessageStream()->write(msg);
			context.write("\n");
			resetPlanning();
			return false;
		}
		resetPlanning();

		return true;
	}
	case 'G':
	{
		context.debug("Plan grasping\n");
		std::string trjItemName("graspingTrj");
		resetPlanning();

		// current configuration (fingers opened)
		Controller::State cstart = lookupState();

		// grasp configuration (fingers closed)
		Controller::State cend = lookupState();
		for (auto i = getPlanner().handInfo.getJoints().begin(); i != getPlanner().handInfo.getJoints().end(); ++i)
			cend.cpos[i] = grasp.cpos[i];

		Controller::State::Seq approach;
		approach.push_back(cstart);
		approach.push_back(cend);
		//findTrajectory(lookupState(), &cend, nullptr, 0, approach);

		Controller::State::Seq out;

		// add trajectory waypoint
		golem::data::Item::Map::iterator ptr;
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(getCS());
			to<Data>(dataPtr)->itemMap.erase(trjItemName);
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), golem::data::Item::Map::value_type(trjItemName, queryHandlerTrj->create()));
			golem::data::Trajectory* trajectory = is<golem::data::Trajectory>(ptr);
			if (!trajectory)
				throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
			// add current state
			trajectory->setWaypoints(golem::WaypointCtrl::make(out, out));
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}


		// disable active controller to apply enough force to the object to lift it.
		if (armHandForce->getArmCtrl()->getMode() != ActiveCtrlForce::MODE_DISABLED || armHandForce->getHandCtrl()->getMode() != ActiveCtrlForce::MODE_DISABLED) {
			armHandForce->getArmCtrl()->setMode(ActiveCtrlForce::MODE_DISABLED);
			armHandForce->getHandCtrl()->setMode(ActiveCtrlForce::MODE_DISABLED);
		}
		// perform
		try {
			perform(dataPtr->first, ptr->first, out, !silent);
		}
		catch (const Message& msg) {
			context.getMessageStream()->write(msg);
			context.write("\n");
			resetPlanning();
			return false;
		}

		// print distance to the targt configuration
		Controller::State cfg = lookupState();
		std::stringstream ss;
		for (auto i = getPlanner().handInfo.getJoints().begin(); i != getPlanner().handInfo.getJoints().end(); ++i) {
			const size_t k = i - getPlanner().handInfo.getJoints().begin();
			ss << "c=" << k << " [" << cend.cpos[i] - cfg.cpos[i] << "]/t";
		}
		context.write("Hand joints error:\n%s\n", ss.str().c_str());

		resetPlanning();
		return true;
	}
	case 'U':
	{
		context.debug("Plan lifting up\n");
		std::string trjItemName("liftingTrj");
		resetPlanning();

		Controller::State cstart = lookupState(), cend = lookupState();

		// transform w.r.t. query frame 
		try {
			findTarget(Mat34::identity(), cend, cend, true);
		}
		catch (const Message& msg) {
			context.getMessageStream()->write(msg);
			context.write("\n");
			resetPlanning();
			return false;
		}

		Controller::State::Seq approach;
		try {
			findTrajectory(cstart, &cend, nullptr, 0, approach);
		}
		catch (const Message& msg) {
			context.getMessageStream()->write(msg);
			context.write("\n");
			resetPlanning();
			return false;
		}

		Controller::State::Seq out;

		// add trajectory waypoint
		golem::data::Item::Map::iterator ptr;
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(getCS());
			to<Data>(dataPtr)->itemMap.erase(trjItemName);
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), golem::data::Item::Map::value_type(trjItemName, modelHandlerTrj->create()));
			golem::data::Trajectory* trajectory = is<golem::data::Trajectory>(ptr);
			if (!trajectory)
				throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
			// add current state
			trajectory->setWaypoints(golem::WaypointCtrl::make(out, out));
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}

		// disable active controller to apply enough force to the object to lift it.
		if (armHandForce->getArmCtrl()->getMode() != ActiveCtrlForce::MODE_DISABLED || armHandForce->getHandCtrl()->getMode() != ActiveCtrlForce::MODE_DISABLED) {
			armHandForce->getArmCtrl()->setMode(ActiveCtrlForce::MODE_DISABLED);
			armHandForce->getHandCtrl()->setMode(ActiveCtrlForce::MODE_DISABLED);
		}
		// perform
		try {
			perform(dataPtr->first, ptr->first, out, !silent);
		}
		catch (const Message& msg) {
			context.getMessageStream()->write(msg);
			context.write("\n");
			resetPlanning();
			return false;
		}

		// open hand and release the object
		// pre-grasp pose w.r.t. query frame
		//::sleep(1000);
		Controller::State openfingers = lookupState();
		Controller::State cnow = lookupState();
		for (auto i = getPlanner().handInfo.getChains().begin(); i != getPlanner().handInfo.getChains().end(); ++i) {
			for (auto j = getPlanner().handInfo.getJoints(i).begin(); j != getPlanner().handInfo.getJoints(i).end(); ++j)
				openfingers.cpos[j] = init.cpos[j]; // pre-grasp fingers' pose
		}
		// transform w.r.t. query frame 
		findTarget(Mat34::identity(), openfingers, openfingers);
		
		//Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(openfingers), manipulator->getConfig(openfingers).toMat34());
		//renderHand(openfingers, bounds, true);
		//Bounds::Seq bounds2 = manipulator->getBounds(manipulator->getConfig(cnow), manipulator->getConfig(cnow).toMat34());
		//renderHand(openfingers, bounds2, false);

		Controller::State::Seq openTrj;
		openTrj.push_back(cnow);
		openTrj.push_back(openfingers);

		Controller::State::Seq out2;
		std::string releaseTrjName = "releaseObjTrj";

		// add trajectory waypoint
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(getCS());
			to<Data>(dataPtr)->itemMap.erase(releaseTrjName);
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), golem::data::Item::Map::value_type(releaseTrjName, modelHandlerTrj->create()));
			golem::data::Trajectory* trajectory = is<golem::data::Trajectory>(ptr);
			if (!trajectory)
				throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
			// add current state
			trajectory->setWaypoints(golem::WaypointCtrl::make(out, out));
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}

		// perform trajectory
		try {
			perform(dataPtr->first, ptr->first, out2, !silent);
		}
		catch (const Message& msg) {
			context.getMessageStream()->write(msg);
			context.write("\n");
			resetPlanning();
			return false;
		}
		// disable active controller to apply enough force to the object to grasp.
		if (armHandForce->getArmCtrl()->getMode() != ActiveCtrlForce::MODE_ENABLED || armHandForce->getHandCtrl()->getMode() != ActiveCtrlForce::MODE_ENABLED) {
			armHandForce->getArmCtrl()->setMode(armMode/*ActiveCtrlForce::MODE_ENABLED*/);
			armHandForce->getHandCtrl()->setMode(handMode/*ActiveCtrlForce::MODE_ENABLED*/);
		}


		resetPlanning();
		return true;
	}
	}
	return false;
}

//------------------------------------------------------------------------------

void R2GPlanner::updateAndResample(Data::Map::iterator dataPtr) {
	if (!pBelief.get() || golem::to<Data>(dataPtr)->queryPoints.empty())
		return;
	context.debug("R2GPlanner::updateAndResample(): %d triggered guards:\n", /*golem::to<Data>(dataPtr)->*/ftGuards.size());
	
	golem::Waypoint w(*controller, lookupState().cpos/*golem::to<Data>(dataPtr)->triggeredStates.begin()->cpos*/);
	context.write("update weights, triggered guards size = %u\n", ftGuards.size());

	//for (auto g = ftGuards.begin(); g != ftGuards.end(); ++g)
	//	(*g)->str(context);
	// render uncertainty before belief update
	resetDataPointers();
	to<Data>(dataCurrentPtr)->createRender();

	const std::string updateItem = makeString("belief-update-%.3f", beliefItem.c_str(), context.getTimer().elapsed());
	recordingStart(dataPtr->first, updateItem, true);
	recordingWaitToStart();

	//::sleep(1000);
	//HBCollision::FlannDesc waypointDesc;
	//HBCollision::Desc::Ptr cloudDesc;
	//cloudDesc.reset(new HBCollision::Desc());
	//HBCollision::Ptr cloud = cloudDesc->create(*manipulator);
	//waypointDesc.depthStdDev = 0.01/*0.0005*/; waypointDesc.likelihood = 1000.0; waypointDesc.points = 10000; waypointDesc.neighbours = 100;
	//waypointDesc.radius = REAL_ZERO;

	//golem::Real norm = golem::REAL_ZERO, c = golem::REAL_ZERO, cdf = golem::REAL_ZERO;
	//pBelief->normaliseFac = REAL_ZERO;
	//golem::Manipulator::Config config(w.cpos, manipulator->getBaseFrame(w.cpos));
	//for (golem::RBPose::Sample::Seq::iterator sampledPose = pBelief->getSamples().begin(); sampledPose != pBelief->getSamples().end();) {
	//	golem::Cloud::PointSeq points;
	//	golem::Cloud::transform(sampledPose->toMat34(), modelPoints, points);
	//	debugRenderer.reset();
	//	debugAppearance.draw(points, debugRenderer);
	//	cloud->create(rand, points);
	//	sampledPose->weight = cloud->evaluateFT(debugRenderer, waypointDesc, config, ftGuards, true);
	//	golem::kahanSum(pBelief->normaliseFac, c, sampledPose->weight);
	//	context.write("sample.weight = %f,\n", sampledPose->weight);
	//	if (option("YN", "Next? (Y/N)") == 'Y')
	//		++sampledPose;
	//}
	//c = golem::REAL_ZERO;
	//for (golem::RBPose::Sample::Seq::iterator sampledPose = pBelief->getSamples().begin(); sampledPose != pBelief->getSamples().end(); ++sampledPose) {
	//	golem::kahanSum(cdf, c, sampledPose->weight);
	//	sampledPose->cdf = cdf;
	//}

	pBelief->createUpdate(debugRenderer, w, ftGuards);
	
	// render the mismatch between estimate and ground truth before resampling
	to<Data>(dataCurrentPtr)->createRender();


//	context.write("resample (wheel algorithm)...\n");
	// resampling (wheel algorithm)
//	pBelief->createResample(/*manipulator->getConfig(getStateFrom(w))*/);
//	size_t N = pBelief->getSamples().size(), index = rand.nextUniform<size_t>(0, N);
//	Real beta = golem::REAL_ZERO;
//	golem::RBPose::Sample::Seq newPoses;
//	newPoses.reserve(N);
//	for (size_t i = 0; i < N; ++i) {
//		beta += rand.nextUniform<golem::Real>() * 2 * pBelief->maxWeight();
//		//		context.write("golem::RBPose::createResampling(): beta=%4.6f\n", beta);
//		while (beta > pBelief->getSamples()[index].weight) {
//			beta -= pBelief->getSamples()[index].weight;
//			index = (index + 1) % N;
//		}
//		context.write("Resample[%d] = Sample[%d].weight=%f\n", i, index, pBelief->getSamples()[index].weight);
//		newPoses.push_back(pBelief->getSamples().at(index));
//	}
//
//	// add noise to the resampled elements and overwrite poses
//	pBelief->getSamples().clear();
//	pBelief->getSamples().reserve(N);
//
//	// generate new (noisy) samples out of selected subset of poses 
//	for (size_t i = 0; i < N;) {
//		//mfsePoses.push_back(Sample(newPoses[i], REAL_ONE, i*REAL_ONE));
//		//continue;
//		golem::RBCoord c = newPoses[i];
////		rand.nextGaussianArray<golem::Real>(&c[0], &c[0] + golem::RBCoord::N, &(newPoses[i])[0], &pBelief->pose.covarianceSqrt[0]); // normalised multivariate Gaussian
//		golem::Cloud::PointSeq points;
//		golem::Cloud::transform(newPoses[i].toMat34(), modelPoints, points);
//		debugRenderer.reset();
//		debugAppearance.draw(points, debugRenderer);
//		golem::Cloud::PointSeq resample;
//		golem::Cloud::transform(c.toMat34(), modelPoints, resample);
//		resampleAppeareance.draw(resample, debugRenderer);
//
//		pBelief->getSamples().push_back(golem::RBPose::Sample(c, REAL_ONE, i*REAL_ONE));
//		if (option("YN", "Resample Next? (Y/N)") == 'Y')
//			++i;
//	}
//	pBelief->normaliseFac = REAL_ZERO;
//
//	// compute mean and covariance
//	if (!pBelief->pose.create<golem::Ref1, golem::RBPose::Sample::Ref>(golem::RBCoord::N, pBelief->desc.covariance, pBelief->getSamples()))
//		throw Message(Message::LEVEL_ERROR, "golem::RBPose::createResample(): Unable to create mean and covariance for the high dimensional representation");
//
//	context.write("golem::Belief::createResample(): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", 
//		pBelief->pose.covariance[0], pBelief->pose.covariance[1], pBelief->pose.covariance[2], 
//		pBelief->pose.covariance[3], pBelief->pose.covariance[4], pBelief->pose.covariance[5], pBelief->pose.covariance[6]);

	// update the query frame
	context.debug("create hypotheses and update query frame...\n");
	golem::RBPose::Sample mlFrame = pBelief->createHypotheses(modelPoints, modelFrame);
	Mat33 qq; mlFrame.q.toMat33(qq);
	golem::to<Data>(dataPtr)->queryTransform/*actionFrame = golem::to<Data>(queryDataPtr)->actionFrame*/ = Mat34(qq, mlFrame.p);

	// update query settings
	golem::to<Data>(dataPtr)->queryFrame.multiply(golem::to<Data>(dataPtr)->queryTransform, modelFrame);

	to<Data>(dataCurrentPtr)->createRender();

	pHeuristic->setHypothesisBounds();
	for (auto g = ftGuards.begin(); g != ftGuards.end(); ++g)
		(*g)->unlock();

	// stop recording
	recordingStop(getPlanner().trajectoryIdlePerf);
	recordingWaitToStop();

	// save belief
	currentBeliefItem = makeString("%s-%.3f", beliefItem.c_str(), recorderStart);
	// clone item
	golem::data::Item::Ptr item = currentBeliefPtr->second->clone();
	RenderBlock renderBlock(*this);
	{
		golem::CriticalSectionWrapper cswData(getCS());
		currentBeliefPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), golem::data::Item::Map::value_type(currentBeliefItem, item));
		golem::data::BeliefState* beliefState = is<golem::data::BeliefState>(currentBeliefPtr->second.get());
		beliefState = currentBeliefPtr != to<Data>(dataCurrentPtr)->itemMap.end() ? beliefState : nullptr;
		if (!beliefState)
			throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): beliefState handler does not implement data::beliefState");
		// add current states
		beliefState->set(pBelief.get());
		beliefState->set(golem::to<Data>(dataPtr)->queryTransform, pBelief->getSamples(), pBelief->getHypothesesToSample());
		beliefState->showMeanPoseOnly(true);
		beliefState->showGroundTruth(true);
		Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, currentBeliefPtr, to<Data>(dataCurrentPtr)->getView());
		context.write("Save: handler %s, inputs %s, %s...\n", beliefHandler->getID().c_str(), currentBeliefPtr->first.c_str(), beliefItem.c_str());
	}
}

//------------------------------------------------------------------------------
