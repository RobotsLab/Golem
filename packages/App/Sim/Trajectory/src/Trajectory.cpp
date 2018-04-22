/** @file Trajectory.cpp
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
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <Golem/Ctrl/Data.h>

using namespace golem;

//------------------------------------------------------------------------------

/** MyApplication */
class MyApplication : public Application {
protected:
	/** Get trajectory by name */
	TrajectoryConfigspaceMap::const_iterator getTrajectory(const Controller& controller, const TrajectoryConfigspaceMap& map) {
		const std::string chars = "0123456789";
		std::string name;

		for (;;) {
			context()->write("\rEnter trajectory name: %s ", name.c_str());
		
			const int key = universe()->waitKey();

			if (universe()->interrupted() || key == 27) {
				return map.end();
			}
			else if (key == '*') {
				Controller::State state = controller.createState();
				controller.lookupState(SEC_TM_REAL_MAX, state);
				context()->write("\n<waypoint dt=\"1\"");
				for (golem::Configspace::Index i = state.getInfo().getJoints().begin(); i < state.getInfo().getJoints().end(); ++i)
					context()->write(" c%d=%0.5f", (*i - *state.getInfo().getJoints().begin() + 1), state.cpos[i]);
				context()->write(">\n");
			}
			else if (key == 13) { // enter
				context()->write("\n");
				break;
			}
			else if (key == 8) {  // <Bkspace>
				if (name.length() > 0 )
					name.erase(name.length() - 1);
			}
			else if (chars.find((char)key) != std::string::npos) { // numbers
				name += (char)key;
			}
			else if (key == '?' || key == ' ') {
				const std::string blank(name.length(), ' ');
				context()->write("\rAvailable trajectories:%s\n", blank.c_str());
				for (TrajectoryConfigspaceMap::const_iterator i = map.begin(); i != map.end(); ++i)
					context()->write(" %s\n", i->first.c_str());
			}
		}

		return map.find(name);
	}

	/** Runs MyApplication */
	virtual void run(int argc, char *argv[]) {
		// help message
		scene()->printHelp();
		
		// Setup controller
		UIController::Desc uiControllerDesc;
		XMLData(uiControllerDesc, context(), xmlcontext());

		// Create controller
		context()->info("Initialising controller...\n");
		UIController *pUIController = dynamic_cast<UIController*>(scene()->createObject(uiControllerDesc));
		if (pUIController == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create controller");
		Controller &controller = pUIController->getController();

		TrajectoryConfigspaceMap map;
		Controller::State dflt = controller.createState();
		controller.setToDefault(dflt);
		loadTrajectoryMap(controller, xmlcontext(), map, dflt);
		
		// Display information
		controllerInfo(controller);

		while (!universe()->interrupted()) {
			TrajectoryConfigspaceMap::const_iterator trajectory = getTrajectory(controller, map);
			
			if (trajectory != map.end()) {
				const SecTmReal t = context()->getTimer().elapsed();
				Controller::State::Seq seq = trajectory->second;
				for (Controller::State::Seq::iterator i = seq.begin(); i != seq.end(); ++i) i->t += t;
				controller.send(seq.data(), seq.data() + seq.size(), true);
			}
		}
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}
