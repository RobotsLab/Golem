/** @file SimJoint.cpp
 * 
 * Demonstration program which moves joints to the zero pose.
 * 
 * Program can be run in two modes:
 * - the first uses real robot
 * - the second runs the robot simulator
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

/** MyApplication */
class MyApplication : public Application {
protected:
	/** Runs MyApplication */
	virtual void run(int argc, char *argv[]) {
		// help message
		scene()->printHelp();
		
		// Setup controller
		UIController::Desc controllerDesc;
		XMLData(controllerDesc, context(), xmlcontext());

		// Create controller
		context()->info("Initialising controller...\n");
		UIController *pUIController = dynamic_cast<UIController*>(scene()->createObject(controllerDesc));
		if (pUIController == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create controller");
		Controller &controller = pUIController->getController();
		
		// Display information
		controllerInfo(controller);

		context()->info("Press the spacebar to move to the ZERO position\n");
		while (universe()->waitKey() != ' ') if (universe()->interrupted()) return;

		// Get the initial position
		Controller::State begin = controller.createState();
		controller.lookupState(context()->getTimer().elapsed(), begin);

		// Go to zero pose
		Controller::State end = controller.createState();
		controller.setToDefault(end);
		// KukaKR5Sixx preset
		//end.cpos[controller.getStateInfo().getJoints().begin() + 1] = -REAL_PI_2;
		//end.cpos[controller.getStateInfo().getJoints().begin() + 2] = REAL_PI_2;
		end.t = context()->getTimer().elapsed() + controller.getCycleDuration() + controller.getCommandLatency() + SecTmReal(10.0);
		controller.send(&end, &end + 1);

		// text output constants
		const SecTmReal cyclePrint = 0.1; // in seconds
		const U32 cycleLatency = 10;
		const U32 cycleDrop = std::max((U32)1, (U32)Math::round(cyclePrint/controller.getCycleDuration()));

		// print the device state along sent trajectory, iterate a few more times to account for commands' delay
		for (U32 i = 0, j = 0; controller.waitForBegin() && (!controller.waitForEnd(0) || j++ < cycleLatency); ++i)
			if (i%cycleDrop == 0) {
				Controller::State state = controller.createState();
				controller.lookupState(SEC_TM_REAL_MAX, state);
				controllerPosition(controller, state);
			}

		context()->info("Press the spacebar to move to the INITIAL position\n");
		while (universe()->waitKey() != ' ') if (universe()->interrupted()) return;

		// and back to the initial position
		begin.t = context()->getTimer().elapsed() + controller.getCycleDuration() + controller.getCommandLatency() + SecTmReal(10.0);
		controller.send(&begin, &begin + 1);
		
		// print the device state along sent trajectory, iterate a few more times to account for commands' delay
		for (U32 i = 0, j = 0; controller.waitForBegin() && (!controller.waitForEnd(0) || j++ < cycleLatency); ++i)
			if (i%cycleDrop == 0) {
				Controller::State state = controller.createState();
				controller.lookupState(SEC_TM_REAL_MAX, state);
				controllerPosition(controller, state);
			}

		context()->info("Press the spacebar to finish\n");
		while (universe()->waitKey() != ' ') if (universe()->interrupted()) return;
		
		context()->info("Good bye!\n");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}
