/** @file UIPlanner.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/App/UIPlanner.h>
#include <Golem/Planner/GraphPlanner/GraphPlanner.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void UIPlannerVis::BoundsData::create(const golem::Bounds::Desc::Seq& boundsDesc) {
	clear();

	for (golem::Bounds::Desc::Seq::const_iterator j = boundsDesc.begin(); j != boundsDesc.end(); ++j) {
		Bounds::Ptr pBounds = (*j)->create();
		if (pBounds == NULL)
			throw Message(Message::LEVEL_ERROR, "UIPlannerVis::BoundsData::create(): unable to create bounds: %s", (*j)->getName());

		bounds.push_back(pBounds);
		poses.push_back(pBounds->getPose());
	}
}

void UIPlannerVis::BoundsData::setPose(const Mat34& pose) {
	const size_t size = std::min(bounds.size(), poses.size());
	for (size_t j = 0; j < size; ++j)
		bounds[j]->multiplyPose(pose, poses[j]);
}

void UIPlannerVis::BoundsData::render(const Appearance& appearance, DebugRenderer& renderer) const {
	if (appearance.showSolid) {
		renderer.setColour(appearance.solidColour);
		renderer.addSolid(bounds.begin(), bounds.end());
	}
	if (appearance.showWire) {
		renderer.setColour(appearance.wireColour);
		renderer.setLineWidth(appearance.wireWidth);
		renderer.addWire(bounds.begin(), bounds.end());
	}
}

//------------------------------------------------------------------------------

UIPlannerVis::UIPlannerVis(Scene &scene, UIControllerVis& uiControllerVis, Planner& planner) : Object(scene), uiControllerVis(uiControllerVis), controller(planner.getController()), planner(planner) {
}

void UIPlannerVis::create(const Desc& desc) {
	if (!desc.isValid())
		throw Message(Message::LEVEL_CRIT, "UIPlannerVis::create(): invalid description");

	graphRenderData = desc.graphRenderData;
	showDuration = desc.showDuration;

	// Controller: install callback interface
	controller.setCallbackDataSync(this);
	// Planner: install callback interface
	planner.setCallbackDataSync(this);

	chainCollisionAppearance = desc.chainCollisionAppearance;
	jointCollisionAppearance = desc.jointCollisionAppearance;

	syncChainBoundsDesc(NULL);
	syncJointBoundsDesc(NULL);

	scene.getHelp().insert(Scene::StrMapVal("0311", "  $                                       planner global graph show\n"));
	scene.getHelp().insert(Scene::StrMapVal("0312", "  %                                       planner local graph show\n"));
	scene.getHelp().insert(Scene::StrMapVal("0313", "  ^                                       planner optimised path show\n"));
	scene.getHelp().insert(Scene::StrMapVal("0314", "  &                                       planner collision bounds show\n"));
}

void UIPlannerVis::release() {
	// Planner: reset callback interface
	if (planner.getCallbackDataSync() == this)
		planner.setCallbackDataSync(NULL);
	// Planner: reset callback interface
	if (controller.getCallbackDataSync() == this)
		controller.setCallbackDataSync(NULL);
}

//------------------------------------------------------------------------------

// Controller::Callback
void UIPlannerVis::syncChainBoundsDesc(Chain* pChain) {
	planner.getHeuristic().syncChainBounds();

	CriticalSectionWrapper csw(scene.getCS());

	const Chainspace::Range range = planner.getController().getStateInfo().getChains();
	const Heuristic::ChainBoundsSet& chainBoundsSet = planner.getHeuristic().getChainBounds();
	for (Chainspace::Index i = range.begin(); i != range.end(); ++i) {
		chainCollisionBounds[i].clear();

		const Heuristic::BoundsSet& boundsSet = chainBoundsSet[i];
		if (boundsSet.empty())
			continue;

		golem::Bounds::Desc::Seq bounds;
		for (Heuristic::BoundsSet::const_iterator j = boundsSet.begin(); j != boundsSet.end(); ++j)
			bounds.push_back(*j);

		chainCollisionBounds[i].create(bounds);
		//context.debug("UIPlannerVis::syncChainBoundsDesc(): index=%u, size=%u\n", (U32)(*i + 1), (U32)chainCollisionBounds[i].bounds.size());
	}
}

// Controller::Callback
void UIPlannerVis::syncJointBoundsDesc(Joint* pJoint) {
	planner.getHeuristic().syncJointBounds();

	CriticalSectionWrapper csw(scene.getCS());

	const Configspace::Range range = planner.getController().getStateInfo().getJoints();
	const Heuristic::JointBoundsSet& jointBoundsSet = planner.getHeuristic().getJointBounds();
	for (Configspace::Index i = range.begin(); i != range.end(); ++i) {
		jointCollisionBounds[i].clear();

		const Heuristic::BoundsSet& boundsSet = jointBoundsSet[i];
		if (boundsSet.empty())
			continue;

		golem::Bounds::Desc::Seq bounds;
		for (Heuristic::BoundsSet::const_iterator j = boundsSet.begin(); j != boundsSet.end(); ++j)
			bounds.push_back(*j);

		jointCollisionBounds[i].create(bounds);
		//context.debug("UIPlannerVis::syncJointBoundsDesc(): index=%u, size=%u\n", (U32)(*i + 1), (U32)jointCollisionBounds[i].bounds.size());
	}
}

// Controller::Callback
void UIPlannerVis::syncSend(const Controller::State* begin) {
	GraphRenderData::Pair pair;
	{
		CriticalSectionWrapper csw(csPlanned);
		GraphRenderData::Map::iterator i = dataPlanned.lower_bound( - begin->t);
		if (i == dataPlanned.end())
			return; // no renderables

		pair = *i;
		// erase the sent renderable and all renderebles which begin earlier the sent one
		dataPlanned.erase(i, dataPlanned.end());
	}
	{
		CriticalSectionWrapper csw(csSent);
		dataSent.insert(pair);
	}
}
	
//------------------------------------------------------------------------------

// Planner::Callback
void UIPlannerVis::syncFindTrajectory(Controller::Trajectory::const_iterator begin, Controller::Trajectory::const_iterator end, const GenWorkspaceChainState* wend) {
	// prepare renderable data
	GraphRenderData::Ptr data(new GraphRenderData(graphRenderData, planner.getHeuristic()));

	// last trajectory element
	const Controller::State* last = &*(end - 1);

	// goal
	if (wend != NULL)
		data->goalRenderer.fromWorkspace(controller, wend, wend + 1);
	else
		data->goalRenderer.fromConfigspace(controller, last, last + 1);

	const GraphPlanner* pGraphPlanner = dynamic_cast<const GraphPlanner*>(&planner);
	if (pGraphPlanner != NULL) {
		//// goal population
		////data->goalPopulationRenderer.fromWorkspace(controller, pGraphPlanner->getKinematics().getPopulation().begin(), pGraphPlanner->getKinematics().getPopulation().end());

		const PathFinder* pGlobalPathFinder = pGraphPlanner->getGlobalPathFinder().get();
		if (pGlobalPathFinder != NULL) {
			// global waypoint graph
			data->globalGraphRenderer.fromWorkspace(controller, pGlobalPathFinder->getGraph().begin(), pGlobalPathFinder->getGraph().end());
			// global waypoint path
			data->globalPathRenderer.fromWorkspace(controller, pGraphPlanner->getGlobalPath().begin(), pGraphPlanner->getGlobalPath().end());
		}

		const PathFinder* pLocalPathFinder = pGraphPlanner->getLocalPathFinder().get();
		if (pLocalPathFinder != NULL) {
			// global waypoint graph
			data->localGraphRenderer.fromWorkspace(controller, pLocalPathFinder->getGraph().begin(), pLocalPathFinder->getGraph().end());
			// global waypoint path
			data->localPathRenderer.fromWorkspace(controller, pGraphPlanner->getLocalPath().begin(), pGraphPlanner->getLocalPath().end());
		}

		//// optimised waypoint path
		data->optimisedPathRendererEx.fromWorkspace(controller, pGraphPlanner->getOptimisedPath().begin(), pGraphPlanner->getOptimisedPath().end());
	}

	data->tmEnd = last->t + showDuration;

	CriticalSectionWrapper csw(csPlanned);
	dataPlanned.insert(GraphRenderData::Pair( - begin->t, data));
}

// Planner::Callback
void UIPlannerVis::syncCollisionBounds() {
	if (planner.getHeuristic().hasCollisionDetection()) {
		planner.getHeuristic().clearCollisionBounds();
		planner.getHeuristic().addCollisionBounds(*uiControllerVis.getCollisionBounds());
	}
}

//------------------------------------------------------------------------------

void UIPlannerVis::keyboardHandler(int key, int x, int y) {
	switch (key) {
	case '#':
		graphRenderData.goalRenderer.show = !graphRenderData.goalRenderer.show;
		graphRenderData.goalPopulationRenderer.show = !graphRenderData.goalPopulationRenderer.show;
		break;
	case '$':
		graphRenderData.globalGraphRenderer.show = !graphRenderData.globalGraphRenderer.show;
		graphRenderData.globalPathRenderer.show = !graphRenderData.globalPathRenderer.show;
		break;
	case '%':
		graphRenderData.localGraphRenderer.show = !graphRenderData.localGraphRenderer.show;
		graphRenderData.localPathRenderer.show = !graphRenderData.localPathRenderer.show;
		break;
	case '^':
		graphRenderData.optimisedPathRendererEx.show = !graphRenderData.optimisedPathRendererEx.show;
		break;
	case '&':
		chainCollisionAppearance.show = !chainCollisionAppearance.show;
		jointCollisionAppearance.show = !jointCollisionAppearance.show;
		break;
	}
}

void UIPlannerVis::render() const {
	// chain and joint bounds
	Controller::State state = controller.createState();
	if (chainCollisionAppearance.show || jointCollisionAppearance.show)
		controller.lookupState(SEC_TM_REAL_MAX, state);

	collisionRenderer.reset();
	if (chainCollisionAppearance.show) {
		WorkspaceChainCoord coords;
		controller.chainForwardTransform(state.cpos, coords);

		const Chainspace::Range range = planner.getController().getStateInfo().getChains();
		for (Chainspace::Index i = range.begin(); i != range.end(); ++i)
			if (!chainCollisionBounds[i].empty()) {
				chainCollisionBounds[i].setPose(coords[i]);
				chainCollisionBounds[i].render(chainCollisionAppearance, collisionRenderer);
			}
	}
	if (jointCollisionAppearance.show) {
		WorkspaceJointCoord coords;
		controller.jointForwardTransform(state.cpos, coords);

		const Configspace::Range range = planner.getController().getStateInfo().getJoints();
		for (Configspace::Index i = range.begin(); i != range.end(); ++i)
			if (!jointCollisionBounds[i].empty()) {
				jointCollisionBounds[i].setPose(coords[i]);
				jointCollisionBounds[i].render(jointCollisionAppearance, collisionRenderer);
			}
	}
	collisionRenderer.render();

	// planner
	GraphRenderData::Ptr data;
	{
		CriticalSectionWrapper csw(csSent);
		
		const SecTmReal tmCurrent = context.getTimer().elapsed();
		
		GraphRenderData::Map::iterator i = dataSent.lower_bound( - tmCurrent);
		if (i == dataSent.end())
			return; // no renderables

		if (tmCurrent < i->second->tmEnd) {
			data = i->second;
			// erase all renderebles beginning earlier the sent one
			dataSent.erase(++i, dataSent.end());
		}
		else {
			// erase all renderebles beginning from the sent one on
			dataSent.erase(i, dataSent.end());
			return;
		}
	}

	data->goalRenderer.show = graphRenderData.goalRenderer.show;
	data->goalRenderer.render();
	data->goalPopulationRenderer.show = graphRenderData.goalPopulationRenderer.show;
	data->goalPopulationRenderer.render();
	data->globalGraphRenderer.show = graphRenderData.globalGraphRenderer.show;
	data->globalGraphRenderer.render();
	data->globalPathRenderer.show = graphRenderData.globalPathRenderer.show;
	data->globalPathRenderer.render();
	data->localGraphRenderer.show = graphRenderData.localGraphRenderer.show;
	data->localGraphRenderer.render();
	data->localPathRenderer.show = graphRenderData.localPathRenderer.show;
	data->localPathRenderer.render();
	data->optimisedPathRendererEx.show = graphRenderData.optimisedPathRendererEx.show;
	data->optimisedPathRendererEx.render();
}

//------------------------------------------------------------------------------

UIPlanner::UIPlanner(Scene &scene) : Object(scene) {
}

void UIPlanner::create(const Desc& desc) {
	Object::create(desc); // throws

	// controller
	pUIController = dynamic_cast<UIController*>(scene.createObject(*desc.pUIControllerDesc));
	if (pUIController == NULL)
		throw MsgUIPlannerUnknownController(Message::LEVEL_CRIT, "UIPlanner::create(): unable to cast to UIController");

	// planners
	plannerSeq.clear();
	plannerPtrSeq.clear();
	for (Planner::Desc::Seq::const_iterator i = desc.plannerDescSeq.begin(); i != desc.plannerDescSeq.end(); ++i) {
		// set initial collision bounds
		(*i)->pHeuristicDesc->collisionDesc.collisionBounds = *pUIController->getCollisionBounds();
		// create
		plannerSeq.push_back((*i)->create(pUIController->getController())); // throws
		plannerPtrSeq.push_back(plannerSeq.back().get()); // copy pointer
	}
	
	// visualisations
	uiPlannerVisPtrSeq.clear();
	UIPlannerVis::Desc::Seq::const_iterator j = desc.uiPlannerVisDescSeq.begin();
	for (Planner::PtrSeq::const_iterator i = plannerPtrSeq.begin(); i != plannerPtrSeq.end(); ++i) {
		uiPlannerVisPtrSeq.push_back(dynamic_cast<UIPlannerVis*>(scene.insertObject(j->create(scene, pUIController->getUIControllerVis(), **i)))); // throws
		if (!uiPlannerVisPtrSeq.back())
			throw Message(Message::LEVEL_CRIT, "UIPlanner::create(): unable to create planner visualisation");
		if (j != --desc.uiPlannerVisDescSeq.end())
			++j;
	}

	// Controller: install callback interface
	pUIController->getController().setCallbackDataSync(this);
}

void UIPlanner::release() {
	// Controller: install callback interface
	pUIController->getController().setCallbackDataSync(NULL);

	// visualisations
	for (UIPlannerVis::PtrSeq::iterator i = uiPlannerVisPtrSeq.begin(); i != uiPlannerVisPtrSeq.end(); ++i)
		scene.releaseObject(**i);

	// planners
	plannerSeq.clear();
	plannerPtrSeq.clear();

	// controller
	scene.releaseObject(*pUIController);
}

//------------------------------------------------------------------------------

// Controller::Callback
void UIPlanner::syncChainBoundsDesc(Chain* pChain) {
	for (UIPlannerVis::PtrSeq::const_iterator i = uiPlannerVisPtrSeq.begin(); i != uiPlannerVisPtrSeq.end(); ++i)
		(*i)->syncChainBoundsDesc(pChain);
}

// Controller::Callback
void UIPlanner::syncJointBoundsDesc(Joint* pJoint) {
	for (UIPlannerVis::PtrSeq::const_iterator i = uiPlannerVisPtrSeq.begin(); i != uiPlannerVisPtrSeq.end(); ++i)
		(*i)->syncJointBoundsDesc(pJoint);
}

// Controller::Callback
void UIPlanner::syncSend(const Controller::State* begin) {
	for (UIPlannerVis::PtrSeq::const_iterator i = uiPlannerVisPtrSeq.begin(); i != uiPlannerVisPtrSeq.end(); ++i)
		(*i)->syncSend(begin);
}

//------------------------------------------------------------------------------

