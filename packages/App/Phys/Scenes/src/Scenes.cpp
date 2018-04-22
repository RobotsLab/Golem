/** @file Scenes.cpp
 * 
 * Demonstration program which creates multiple controllers in multiple scenes.
 * PhysX simulator controls movement of objects created by setupObjects().
 * 
 * Program can be run in two modes:
 * - the first uses real robot (it's dangerous - no collision detection!)
 * - the second simply runs a specified number of simulators
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
#include <Golem/Phys/PhysUniverse.h>
#include <Golem/App/Common/Creator.h>
#include <Golem/App/Common/Tools.h>
#include <Golem/Sys/XMLData.h>
#include <iostream>
#include <sstream>

using namespace golem;

//------------------------------------------------------------------------------

void setupObjects(Scene &scene) {
	static Rand rand(scene.getContext().getRandSeed());

	// Creator
	Creator creator(scene);
	Actor::Desc::Ptr pActorDesc;
	
	// Create ground plane.
	pActorDesc = creator.createGroundPlaneDesc();
	scene.createObject(*pActorDesc);
	
	// add some objects!
	const U32 numOfObject = 10;
	for (U32 i = 0; i < numOfObject; ++i) {
		pActorDesc = creator.createTreeDesc(rand.nextUniform(Real(0.07), Real(0.10)));
		pActorDesc->pose.p.set(
			rand.nextUniform(Real(-0.3), Real(0.3)),
			rand.nextUniform(Real(-0.3), Real(0.3)),
			rand.nextUniform(Real(+0.3), Real(0.9))
		);
		scene.createObject(*pActorDesc);
	}
}

//------------------------------------------------------------------------------

/** MyApplication */
class MyApplication : public Application {
protected:
	/** Runs MyApplication */
	virtual void run(int argc, char *argv[]) {
		// help message
		scene()->printHelp();

		U32 numScenes = 2;
		XMLData("num_scenes", numScenes, xmlcontext()->getContextFirst("universe"));
		// Number of controllers
		const U32 numOfControllers = numScenes*(numScenes + 1)/2;
		// Controllers
		std::vector<UIController*> controllers(numOfControllers, (UIController*)NULL);
	
		// Create simulations
		for (U32 i = 0; i < numScenes; ++i) {
			Scene *pScene = NULL;
			if (i == 0)
				pScene = scene(); // use already created scene
			else {
				// Create scene
				Scene::Desc::Ptr sceneDesc = universe()->createSceneDesc();
				sceneDesc->load(xmlcontext()->getContextFirst("scene"));
				std::stringstream name;
				name << "Scene #" << i + 1;
				sceneDesc->name = name.str();
				pScene = universe()->createScene(*sceneDesc);
			}
			// Scene objects setup
			setupObjects(*pScene);

			for (U32 j = 0; j <= i ; ++j) {
				U32 n = i*(i + 1)/2 + j;

				// Create controller description
				UIController::Desc controllerDesc;
				controllerDesc.pControllerDesc = n % 2 ? Controller::Desc::load(context(), "GolemCtrlSixAxisSim", "GolemCtrlSixAxisSim") : Controller::Desc::load(context(), "GolemCtrlKatana300Sim", "GolemCtrlKatana300");
				controllerDesc.pControllerDesc->globalPose.p.set(Real(-1.0 * j), Real(-1.0 * j), Real(0.0));

				// Create PhysController controller
				context()->info("Initialising controller #%d...\n", n + 1);
				controllers[n] = static_cast<UIController*>(pScene->createObject(controllerDesc));
				
				// Display information
				controllerInfo(controllers[n]->getController());
			}
		}

		// Repeatedly move joints of all controllers to random positions
		Rand rand(context()->getRandSeed());
		while (!universe()->interrupted()) {
			for (U32 i = 0; i < numOfControllers; ++i) {
				Controller& controller = controllers[i]->getController();
				
				// wait until a new waypoint is sent
				if (!controller.waitForBegin())
					continue;
				
				// then the next nearest waypoint to be sent is:
				const SecTmReal t = context()->getTimer().elapsed() + controller.getCycleDuration() + controller.getCommandLatency();
				Controller::State state = controller.createState();
				controller.lookupCommand(t, state);
				state.t = t; // overwrite since there may be no waypoints in the command queue what will clamp time stamp 'state.t'
				
				// prepare a new target
				Controller::State next = controller.createState();
				controller.setToDefault(next); // zero
				next.t = t + Math::ceil(SecTmReal(5.0), controller.getCycleDuration()); // 5 sec in future, multiplicity of cycle duration (optional)

				REPEAT:
				// generate random target in the neighbourhood of the next nearest waypoint
				const Real delta = Real(1.0)*REAL_PI;
				for (Configspace::Index j = controller.getStateInfo().getJoints().begin(); j < controller.getStateInfo().getJoints().end(); ++j)
					next.cpos[j] = Math::clamp(state.cpos[j] + rand.nextUniform<Real>(-delta, +delta), controller.getMin().cpos[j], controller.getMax().cpos[j]);
				
				try {
					// send new target and clear the command queue
					controller.send(&next, &next + 1, true);
				}
				catch (const MsgControllerLimits&) {
					// try again if out of limits
					goto REPEAT;
				}
			}

			// Wait for some time to let the device move a bit
			Sleep::msleep(SecToMSec(2.0));
		}

		context()->info("Good bye!\n");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv, PhysUniverse::Desc());
}
