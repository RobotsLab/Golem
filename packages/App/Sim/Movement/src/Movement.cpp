/** @file Movement.cpp
 * 
 * Demonstration program which moves the arm along a straight line
 * using reactive trajectory planner and collision detection
 * (also see DemoReacPlanner and DemoFinger).
 * 
 * Program can be run in two modes:
 * - the first uses real robot
 * - the second runs the arm simulators
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
#include <Golem/App/Application.h>
#include <Golem/App/UIPlanner.h>
#include <Golem/App/Data.h>
#include <Golem/App/Common/Tools.h>

using namespace golem;

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
		UIPlanner::Desc uiPlannerDesc;
		XMLData(uiPlannerDesc, context(), xmlcontext());

		// Create UIPlanner
		context()->info("Initialising planner...\n");
		UIPlanner *pUIPlanner = dynamic_cast<UIPlanner*>(scene()->createObject(uiPlannerDesc));
		if (pUIPlanner == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create ReacPlanner");
		
		// some useful pointers
		Controller &controller = pUIPlanner->getController();
		Planner &planner = *pUIPlanner->getPlannerSeq()[0];
		
		// Display information
		controllerInfo(controller);

		// TODO

		context()->info("Good bye!\n");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}
