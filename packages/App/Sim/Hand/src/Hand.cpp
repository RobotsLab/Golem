/** @file Hand.cpp
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
#include <Golem/App/UIController.h>
#include <Golem/App/Data.h>
#include <Golem/App/Common/Tools.h>
#include <iostream>

using namespace golem;

//------------------------------------------------------------------------------

class MyUIController : public UIController {
public:
	enum Mode {
		THUMB, FINGERS, HAND
	};

	class Desc : public UIController::Desc {
	protected:
		CREATE_FROM_OBJECT_DESC_1(MyUIController, Object::Ptr, Scene&)
	};

	MyUIController(Scene &scene) : UIController(scene) {
	}
	
	void create(const Desc& desc) {
		UIController::create(desc); // throws
		
		for (Chainspace::Index i = getController().getStateInfo().getChains().begin(); i < getController().getStateInfo().getChains().end(); ++i) {
			pose[i] = getController().getChains()[i]->getLocalPose();

			Bounds::Desc::SeqPtr pChainSeq = getController().getChains()[i]->getBoundsDescSeq();
			bounds.insert(bounds.end(), pChainSeq->begin(), pChainSeq->end());
			for (Configspace::Index j = getController().getStateInfo().getJoints(i).begin(); j < getController().getStateInfo().getJoints(i).end(); ++j) {
				Bounds::Desc::SeqPtr pJointSeq = getController().getJoints()[j]->getBoundsDescSeq();
				bounds.insert(bounds.end(), pJointSeq->begin(), pJointSeq->end());
			}
		}
		
		Mat34 id;
		id.setId();
		renderer.addAxes(id, Vec3(0.1));
		
		mode = HAND;
	}

	void render() const {
		UIController::render();
		renderer.render();
	}
	
	void keyboardHandler(int key, int x, int y) {
		Mat33 R;

		switch (key) {
		case 'B':
			for (Bounds::Desc::Seq::const_iterator i = bounds.begin(); i != bounds.end(); ++i) {
				if ((*i)->getType() != Bounds::TYPE_CONVEX_MESH)
					continue;
				BoundingConvexMesh::Desc* pConvexMeshDesc = dynamic_cast<BoundingConvexMesh::Desc*>(i->get());
				if (pConvexMeshDesc == NULL)
					continue;
				for (std::vector<Vec3>::const_iterator j = pConvexMeshDesc->vertices.begin(); j != pConvexMeshDesc->vertices.end(); ++j)
					context.write("<vertex v1=\"%f\" v2=\"%f\" v3=\"%f\"/>\n", j->v1, j->v2, j->v3);
				for (std::vector<Triangle>::const_iterator j = pConvexMeshDesc->triangles.begin(); j != pConvexMeshDesc->triangles.end(); ++j)
					context.write("<triangle t1=\"%d\" t2=\"%d\" t3=\"%d\"/>\n", j->t1, j->t2, j->t3);
			}
			return;
		case 'T':
			mode = THUMB;
			return;
		case 'F':
			mode = FINGERS;
			return;
		case 'H':
			mode = HAND;
			return;
		case '1':
			R.rotX(REAL_PI_2);
			break;
		case '2':
			R.rotX(-REAL_PI_2);
			break;
		case '3':
			R.rotY(REAL_PI_2);
			break;
		case '4':
			R.rotY(-REAL_PI_2);
			break;
		case '5':
			R.rotZ(REAL_PI_2);
			break;
		case '6':
			R.rotZ(-REAL_PI_2);
			break;
		case 'P':
			break;
		default:
			return;
		}

		for (Chainspace::Index i = getController().getStateInfo().getChains().begin(); i < getController().getStateInfo().getChains().end(); ++i) {
			Mat34 m;
			
			if (key == 'P') {
				m = getController().getChains()[i]->getLocalPose();
				context.write("v1=\"%.8f\" v2=\"%.8f\" v3=\"%.8f\" m11=\"%.8f\" m12=\"%.8f\" m13=\"%.8f\" m21=\"%.8f\" m22=\"%.8f\" m23=\"%.8f\" m31=\"%.8f\" m32=\"%.8f\" m33=\"%.8f\"\n", m.p.v1, m.p.v2, m.p.v3, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
				continue;
			}
			else
			if (mode == THUMB && i != 0 || mode == FINGERS && i == 0)
				continue;

			if (key == '4') {
				m = pose[i];
			}
			else
			if (mode == HAND) {
				m.p.setZero();
				m.R = R;
				m.multiply(m, getController().getChains()[i]->getLocalPose());
			}
			else {
				m.p = pose[i].p;
				m.R.multiply(R, getController().getChains()[i]->getLocalPose().R);
			}
			
			getController().getChains()[i]->setLocalPose(m);
		}
	}

	Chainspace::Coord<Mat34> pose;
	Bounds::Desc::Seq bounds;
	DebugRenderer renderer;
	Mode mode;
};

//------------------------------------------------------------------------------

/** MyApplication */
class MyApplication : public Application {
protected:
	/** Runs MyApplication */
	virtual void run(int argc, char *argv[]) {
		// help message
		scene()->printHelp();
		
		// Setup controller
		MyUIController::Desc physControllerDesc;
		XMLData(physControllerDesc, context(), xmlcontext());

		// Create controller
		context()->info("Initialising controller...\n");
		UIController *pUIController = dynamic_cast<UIController*>(scene()->createObject(physControllerDesc));
		if (pUIController == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create controller");
		Controller &controller = pUIController->getController();
		
		// Display information
		controllerInfo(controller);

		// (Maximum) mumber of Joints
		const U32 NUM_JOINTS = 20;
		// State info
		const Controller::State::Info stateInfo = controller.getStateInfo();
		// Assert number of joints
		if (stateInfo.getJoints().size() > NUM_JOINTS)
			throw Message(Message::LEVEL_CRIT, "Invalid number of joints %d > %d", stateInfo.getJoints().size(), NUM_JOINTS);

		// Gestures for 5-finger DLR Hit Hand II (in degrees, with corrections from DLR Hand demo)
		const Real GESTURE[][NUM_JOINTS] = {
			{0.,0.,6.,6.,						0.,0.,6.,6.,						0.,0.,6.,6.,						0.,0.,6.,6.,						0.,0.,6.,6.,						},
			{-1.10688,17.7628,23.3638,23.3638,	0.00216961,22.7019,30.092,30.092,	0.39448,7.29597,5.37546,5.37546,	-0.523871,7.5994,6.76667,6.76667,	-0.774781,6.97447,6.07735,6.07735,	},
			{-1.10688,16.9308,29.1958,29.1958,	5.8742,24.6219,20.66,20.66,			3.94648,32.608,25.5355,25.5355,		-0.587871,7.7914,6.76667,6.76667,	-0.710781,7.10247,6.43735,6.43735,	},
			{2.2871,35.2759,17.7981,17.7981,	3.12653,20.8392,31.5946,31.5946,	1.09591,24.6194,32.128,32.128,		0.404752,29.7219,27.4692,27.4692,	-0.344846,6.84051,7.64844,7.64844,	},
			{5.1672,32.7799,17.4381,17.4381,	3.15853,20.8712,31.5946,31.5946,	0.871905,24.3314,32.2,32.2,			0.404752,29.7219,27.4692,27.4692,	0.135155,33.4325,24.7844,24.7844,	},
			//{0.,},
			//{-15.,0.,0.,0.,-15.,},
			//{+15.,0.,0.,0.,+15.,},
			//{0.,},
			//{0.,+15.,0.,0.,0.,+15.,},
			//{0.,0.,},
			//{0.,0.,+15.,0.,0.,0.,+15.,},
		};

		// Number of gestures
		const U32 NUM_GESTURES = sizeof(GESTURE)/(NUM_JOINTS*sizeof(Real));
		// Trajectory duration
		const SecTmReal duration = controller.getCycleDuration() + controller.getCommandLatency() + SecTmReal(0.5);
		// Idle time duration
		const SecTmReal sleep = SecTmReal(2.0);

		Controller::State target = controller.createState();
		controller.setToDefault(target);
		target.t = context()->getTimer().elapsed() + duration;
		for (U32 i = 0, j = 0; controller.waitForBegin() && !universe()->interrupted(); ++i) {
			// print (every 100th) device state
			if (i%100 == 0) {
				Controller::State state = controller.createState();
				controller.lookupState(SEC_TM_REAL_MAX, state);
				//controllerPosition(controller, state);
			}
			// sequentially present gestures, then return to the initial pose
			if (j <= NUM_GESTURES && target.t + sleep < context()->getTimer().elapsed() && controller.waitForEnd(0)) {
				context()->info("Gesture %d\n", j);
				std::transform(GESTURE[j%NUM_GESTURES], GESTURE[j%NUM_GESTURES] + stateInfo.getJoints().size(), &target.cpos[stateInfo.getJoints().begin()], Math::degToRad<Real>);
				target.t = context()->getTimer().elapsed() + duration;
				controller.send(&target, &target + 1);
				++j;
			}
			// if space button is pressed, restart gesture sequence (skip the initial pose)
			if (j > NUM_GESTURES && universe()->waitKey(0) == ' ')
				j = 1;
		}

		context()->info("Good bye!\n");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}
