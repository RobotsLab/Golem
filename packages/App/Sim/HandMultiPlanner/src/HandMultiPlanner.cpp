/** @file HandMultiPlanner.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/App/Application.h>
#include <Golem/App/UIPlanner.h>
#include <Golem/App/Data.h>
#include <Golem/App/Common/Tools.h>
#include <Golem/Ctrl/MultiCtrl/MultiCtrl.h>
//#include <Golem/Phys/PhysUniverse.h>
#include <iostream>

using namespace golem;

//------------------------------------------------------------------------------

/** Grasp point */
class GraspPoint {
public:
	typedef std::vector<GraspPoint> Seq;

	/** Finger-object contact point */
	Vec3 contact;
	/** Surface normal at the contact point */
	Vec3 normal;
	
	GraspPoint() {
		contact.setZero();
		normal.set(REAL_ZERO, -REAL_ONE, REAL_ZERO); // Y-axis
	}
	GraspPoint(const Mat34& pose, const GraspPoint& gp) {
		multiply(pose, gp);
	}
	void multiply(const Mat34& pose, const GraspPoint& gp) {
		pose.multiply(contact, gp.contact);
		pose.R.multiply(normal, gp.normal);
	}
	static void multiply(const Mat34& pose, Seq& seq) {
		for (Seq::iterator i = seq.begin(); i != seq.end(); ++i)
			i->multiply(pose, *i);
	}
};

/** Graspable object */
class MyObject : public Actor {
public:
	/** MyObject description */
	class Desc : public Actor::Desc {
	public:
		/** Grasp points */
		GraspPoint::Seq graspPoints;
		/** Hand frame */
		Mat34 handFrame;
		/** Normals' displayed length */
		Real length;
		/** Normals' colour */
		RGBA colour;
		
		/** Constructs description object */
		Desc() {
			setToDefault();
		}
		/** Sets the objet parameters to the default values */
		void setToDefault() {
			Actor::Desc::setToDefault();
			graspPoints.clear();
			handFrame.setId();
			length = Real(0.05);
			colour = RGBA::WHITE;
		}
		/** Sets object pose */
		void setPose(const Mat34& pose) {
			for (Bounds::Desc::Seq::iterator i = boundsDescSeq.begin(); i != boundsDescSeq.end(); ++i)
				if ((*i) != NULL) (*i)->pose = pose;
			GraspPoint::multiply(pose, graspPoints);
			handFrame.multiply(pose, handFrame);
		}
		/** Checks if the object description is valid. */
		bool isValid() const {
			if (!Actor::Desc::isValid())
				return false;
			if (graspPoints.size() < 2 || !handFrame.isValid() || length <= REAL_ZERO)
				return false;
			return true;
		}

	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(MyObject, Object::Ptr, Scene&)
	};

	/** Hand frame */
	Mat34 getHandFrame() const {
		const Mat34 pose = getPose();
		Mat34 hf;
		hf.multiply(pose, handFrame);
		return hf;
	}
	/** Grasp points */
	void getGraspPoints(GraspPoint::Seq& gps) const {
		gps.clear();
		const Mat34 pose = getPose();
		for (GraspPoint::Seq::const_iterator i = graspPoints.begin(); i != graspPoints.end(); ++i)
			gps.push_back(GraspPoint(pose, *i));
	}

protected:
	Actor* pActor;
	GraspPoint::Seq graspPoints;
	Mat34 handFrame;
	Real length;
	RGBA colour;
	mutable DebugRenderer renderer;
	
	void render() const {
		if (pActor) {
			renderer.reset();
			const Mat34 pose = pActor->getPoseSync();
			for (GraspPoint::Seq::const_iterator i = graspPoints.begin(); i != graspPoints.end(); ++i) {
				const GraspPoint gp(pose, *i);
				Vec3 p;
				p.multiply(length, gp.normal);
				p.add(gp.contact, p);
				renderer.addLine(gp.contact, p, colour);
			}
			Mat34 hf;
			hf.multiply(pose, handFrame);
			renderer.addAxes(hf, Vec3(length));
			renderer.render();
		}
	}
	void create(const Desc& desc) {
		Desc* const pDesc = const_cast<Desc*>(&desc);

		// create dynamic Actor
		Actor::Desc::Ptr pActorDesc = scene.createActorDesc();
		pActorDesc->pose = pDesc->pose;
		pActorDesc->boundsDescSeq = pDesc->boundsDescSeq;
		pActorDesc->appearance = pDesc->appearance;
		pActorDesc->kinematic = false;
		pActor = dynamic_cast<Actor*>(scene.createObject(*pActorDesc)); // throws
		if (pActor == NULL)
			throw Message(Message::LEVEL_CRIT, "MyObject::create(): Unable to create dynamic Actor");

		// create Actor
		pDesc->boundsDescSeq.clear(); // clear bounds
		Actor::create(desc); // throws
		graspPoints = desc.graspPoints;
		handFrame = desc.handFrame;
		length = desc.length;
		colour = desc.colour;
		pDesc->boundsDescSeq = pActorDesc->boundsDescSeq; // restore bounds
	}
	MyObject(Scene &scene) : Actor(scene), pActor(NULL) {
	}
	~MyObject() {
		if (pActor) scene.releaseObject(*pActor);
	}
};

//------------------------------------------------------------------------------

/** Creates cylinder and its grasp parameters */
void makeCylinder(MyObject::Desc& desc, Real radius, Real length, U32 fingers = 5, Real fingerDist = Real(0.15), Real fingerTopDist = Real(0.4), Real handFrameHeight = Real(0.25)) {
	ASSERT(fingers > 1)

	// object body
	BoundingCylinder::Desc* pDesc = new BoundingCylinder::Desc;
	desc.boundsDescSeq.push_back(Bounds::Desc::Ptr(pDesc));
	pDesc->radius = radius;
	pDesc->length = length;

	// height
	const Real height = length*(Real(0.5) - fingerTopDist);
	// thumb
	GraspPoint thumb;
	thumb.normal.set(REAL_ZERO, -REAL_ONE, REAL_ZERO); // Y-axis
	thumb.contact.set(REAL_ZERO, -radius, height);
	desc.graspPoints.push_back(thumb);
	// fingers' spread
	const Real angleDelta = REAL_2_PI*std::min(REAL_ONE/fingers, fingerDist);
	const Real angleStart = -angleDelta*(fingers - 2)/2.0;
	// generate contact points for the rest of fingers starting from index finger
	for (U32 i = 0; i < fingers - 1; ++i) {
		const Real angle = angleStart + i*angleDelta;
		GraspPoint finger;
		finger.normal.set(Math::sin(angle), Math::cos(angle), REAL_ZERO);
		finger.contact.set(radius*finger.normal.x, radius*finger.normal.y, height);
		desc.graspPoints.push_back(finger);
	}
	// hand frame
	desc.handFrame.p.set(Real(0.), Real(0.), handFrameHeight - length*fingerTopDist);
}

//------------------------------------------------------------------------------

/** MyApplication */
class MyApplication : public Application {
protected:
	/** Runs MyApplication */
	virtual void run(int argc, char *argv[]) {
		// help message
		scene()->printHelp();
		
		// Random number generator seed
		context()->info("Random number generator seed %d\n", context()->getRandSeed()._U32[0]);

		// Setup planner
		UIPlanner::Desc physPlannerDesc;
		XMLData(physPlannerDesc, context(), xmlcontext());

		// Create planner
		context()->info("Initialising planner...\n");
		UIPlanner *pUIPlanner = dynamic_cast<UIPlanner*>(scene()->createObject(physPlannerDesc));
		if (pUIPlanner == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create planner");
		
		// Useful variables
		Planner &planner = *pUIPlanner->getPlannerSeq()[0];
		Heuristic &heuristic = planner.getHeuristic();
		Controller &controller = planner.getController();

		// Display information
		controllerInfo(controller);

		// We assume that multi controller is used
		MultiCtrl *pMultiCtrl = dynamic_cast<MultiCtrl*>(&controller);
		if (pMultiCtrl == NULL)
			throw Message(Message::LEVEL_CRIT, "Unknown controller");
		// which has two other controllers: arm + hand
		if (pMultiCtrl->getControllers().size() != 2)
			throw Message(Message::LEVEL_CRIT, "Arm and hand controllers expected");
		// State info
		const Controller::State::Info stateInfo = controller.getStateInfo();
		// Arm controller chain
		const Chainspace::Index chainArm = stateInfo.getChains().begin();
		// Arm controller number of chains
		const idx_t chainArmSize = pMultiCtrl->getControllers()[0]->getStateInfo().getChains().size();
		// Hand controller chain
		const Chainspace::Index chainHand = stateInfo.getChains().begin() + chainArmSize;
		// Hand controller number of chains
		const idx_t chainHandSize = pMultiCtrl->getControllers()[1]->getStateInfo().getChains().size(); //stateInfo.getChains().end() - chainHand;

		// create ground plane
		Actor::Desc::Ptr groundPlaneDesc = scene()->createActorDesc();
		groundPlaneDesc->boundsDescSeq.push_back(Bounds::Desc::Ptr(new BoundingPlane::Desc));
		scene()->createObject(*groundPlaneDesc);

		// TODO find object shape and grasp parameters from real data (e.g. using Kinect)
		// create object description with grasp parameters
		MyObject::Desc cylinderDesc;
		makeCylinder(cylinderDesc, 0.04, 0.1, (U32)chainHandSize);
		// object pose
		Mat34 cylinderPose;
		cylinderPose.p.set(0.0, 0.5, 0.1/2.0);
		cylinderPose.R.setId();
		cylinderDesc.setPose(cylinderPose);
		
		// create object
		MyObject *pCylinder = dynamic_cast<MyObject*>(scene()->createObject(cylinderDesc));
		if (pCylinder == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create object");

		// All bounds are treated as obstacles
		pUIPlanner->setCollisionBoundsGroup(Bounds::GROUP_ALL);
		// Velocity and acceleration limits
		planner.getProfile()->getVelocity().fill(Real(0.5));
		planner.getProfile()->getAcceleration().fill(Real(0.5));
		// Main ON/OFF collision detection switch
		//heuristic.setCollisionDetection(false);

		// memorise initial position
		Controller::State init = controller.createState();
		controller.lookupState(SEC_TM_REAL_MAX, init);

		// set target for the arm-hand system
		GenWorkspaceChainState cylinderGraspState;
		cylinderGraspState.setToDefault(stateInfo.getChains().begin(), stateInfo.getChains().end()); // all used chains
		// read current hand frame
		cylinderGraspState.wpos[chainArm] = pCylinder->getHandFrame();
		// and grasp points
		GraspPoint::Seq cylinderGraspPoints;
		pCylinder->getGraspPoints(cylinderGraspPoints);
		for (Chainspace::Index i = chainHand; i < stateInfo.getChains().end(); ++i)
			if (i - chainHand < (idx_t)cylinderGraspPoints.size())
				cylinderGraspState.wpos[i].p = cylinderGraspPoints[i - chainHand].contact;

		// Setup planner: enable linear distance for arm tool frame (chain #0) and fingers tool's frames
		Heuristic::Desc desc = heuristic.getDesc();
		for (Chainspace::Index i = chainArm; i < stateInfo.getChains().end(); ++i) {
			// desc.chains is a standard container with indices counted from 0
			desc.chains[i - chainArm].enabledLin = i - chainHand < (idx_t)cylinderGraspPoints.size(); // ignore fingers which have no grasp points
			desc.chains[i - chainArm].enabledAng = false; // ignore angular distance
		}
		heuristic.setDesc(desc);

		// find target with closed hand
		context()->info("Computing grasp pose...\n");
		Controller::State closed = controller.createState();
		controller.setToDefault(closed);
		if (!planner.findTarget(init, cylinderGraspState, closed))
			return;

		// create target with open hand (assumed here: joint values equal zero)
		Controller::State open = closed;
		for (Configspace::Index i = stateInfo.getJoints(chainHand).begin(); i < stateInfo.getJoints().end(); ++i)
			open.cpos[i] = REAL_ZERO;
		
		// pre-shape hand to open position without changing arm position
		Controller::State initOpen = open;
		for (Configspace::Index i = stateInfo.getJoints(chainArm).begin(); i < stateInfo.getJoints(chainArm).end(); ++i)
			initOpen.cpos[i] = init.cpos[i];
		initOpen.t = context()->getTimer().elapsed() + controller.getCycleDuration() + controller.getCommandLatency() + SecTmReal(1.0); // 1 sec movement
		context()->info("Pre-shaping hand...\n");
		if (controller.send(&initOpen, &initOpen + 1) != &initOpen + 1)
			return;
		// wait to finish
		while (!controller.waitForEnd()) { if (universe()->interrupted()) return; }

		// Setup planner: enable only arm planning
		desc = heuristic.getDesc();
		for (Chainspace::Index i = chainArm; i < stateInfo.getChains().end(); ++i) {
			desc.chains[i - chainArm].enabledLin = i <= chainArm; // enable arm only
			desc.chains[i - chainArm].enabledAng = i <= chainArm; // enable arm only
			for (Configspace::Index j = stateInfo.getJoints(i).begin(); j < stateInfo.getJoints(i).end(); ++j)
				desc.joints[j - stateInfo.getJoints(chainArm).begin()].enabled = i <= chainArm; // enable arm only
		}
		heuristic.setDesc(desc);

		// plan collision-free trajectory to open target
		Controller::Trajectory trajectory;
		while (!universe()->interrupted()) {
			context()->info("Planning movement to object...\n");
			// find trajectory and wait until the device is ready for new commands
			open.t = context()->getTimer().elapsed() + controller.getCycleDuration() + controller.getCommandLatency() + SecTmReal(10.0); // 10 sec movement
			if (planner.findGlobalTrajectory(initOpen, open, trajectory, trajectory.begin()))
				break;
		}

		// move the arm
		context()->info("Moving to object...\n");
		if (controller.send(&trajectory.front(), &trajectory.back() + 1) != &trajectory.back() + 1)
			return;
		// wait to finish
		while (!controller.waitForEnd()) if (universe()->interrupted()) return;

		// close hand
		closed.t = context()->getTimer().elapsed() + controller.getCycleDuration() + controller.getCommandLatency() + SecTmReal(1.0); // 1 sec movement
		context()->info("Closing hand...\n");
		if (controller.send(&closed, &closed + 1) != &closed + 1)
			return;

		context()->info("Press space to finish...\n");
		while (controller.waitForBegin() && universe()->waitKey(0) != ' ') if (universe()->interrupted()) return;

		init.t = context()->getTimer().elapsed() + controller.getCycleDuration() + controller.getCommandLatency() + SecTmReal(10.0); // 10 sec movement
		if (controller.send(&init, &init + 1) != &init + 1)
			return;
		// wait to finish
		while (!controller.waitForEnd()) if (universe()->interrupted()) return;

		context()->info("Good bye!\n");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
	//return MyApplication().main(argc, argv, PhysUniverse::Desc());
}
