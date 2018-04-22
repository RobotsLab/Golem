/** @file Data.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/Data.h>
#include <Golem/Planner/Data.h>
#include <Golem/Plugin/Data.h>
#include <Golem/App/Data.h>
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(UIPlannerVis::BoundsData::Appearance &val, XMLContext* context, bool create) {
	golem::XMLData("show_solid", val.showSolid, const_cast<golem::XMLContext*>(context), false);
	golem::XMLData("show_wire", val.showWire, const_cast<golem::XMLContext*>(context), false);

	golem::XMLData(val.solidColour, context->getContextFirst("solid_colour"), false);
	golem::XMLData(val.wireColour, context->getContextFirst("wire_colour"), false);
	golem::XMLData("wire_width", val.wireWidth, const_cast<golem::XMLContext*>(context), false);
}

void golem::XMLData(GraphRenderer &val, XMLContext* context, bool create) {
	XMLData(val.vertexPositionColour, context->getContextFirst("vertex_position_colour"), create);
	XMLData("vertex_position_show", val.vertexPositionShow, context, create);
	XMLData(val.vertexFrameSize, context->getContextFirst("vertex_frame_size"), create);
	XMLData("vertex_frame_show", val.vertexFrameShow, context, create);
	XMLData(val.edgeColour, context->getContextFirst("edge_colour"), create);
	XMLData("edge_show", val.edgeShow, context, create);
	XMLData("show", val.show, context, create);
}

void golem::XMLData(UIPlannerVis::GraphRenderData &val, XMLContext* context, bool create) {
	XMLData(val.goalRenderer, context->getContextFirst("goal_renderer"), create);
	XMLData(val.goalPopulationRenderer, context->getContextFirst("goal_population_renderer"), create);
	XMLData(val.globalGraphRenderer, context->getContextFirst("global_graph_renderer"), create);
	XMLData(val.globalPathRenderer, context->getContextFirst("global_path_renderer"), create);
	XMLData(val.localGraphRenderer, context->getContextFirst("local_graph_renderer"), create);
	XMLData(val.localPathRenderer, context->getContextFirst("local_path_renderer"), create);
	XMLData(val.optimisedPathRendererEx, context->getContextFirst("optimised_path_rendererex"), create);
}

//------------------------------------------------------------------------------

void golem::XMLData(UIControllerVis::Desc &val, XMLContext* xmlcontext, bool create) {
	xmlcontext = xmlcontext->getContextFirst("renderer");
	// controller visualisation
	XMLData(val.jointAppearance, xmlcontext->getContextFirst("joint_appearance"), create);
	XMLData("path_time_past", val.pathTimePast, xmlcontext->getContextFirst("path_renderer"), create);
	XMLData("path_time_future", val.pathTimeFuture, xmlcontext->getContextFirst("path_renderer"), create);
	XMLData("path_segments", val.pathSegments, xmlcontext->getContextFirst("path_renderer"), create);
	XMLData("path_segment_duration", val.pathSegmentDuration, xmlcontext->getContextFirst("path_renderer"), create);
	XMLData(val.pathRenderer, xmlcontext->getContextFirst("path_renderer"), create);
	XMLData("state_joint_frames_show", val.stateJointFramesShow, xmlcontext->getContextFirst("state_renderer"), create);
	XMLData(val.stateJointFramesSize, xmlcontext->getContextFirst("state_renderer state_joint_frames_size"), create);
	XMLData("state_use_commands", val.stateUseCommands, xmlcontext->getContextFirst("state_renderer"), create);
	XMLData(val.stateRenderer, xmlcontext->getContextFirst("state_renderer"), create);
}

void golem::XMLData(UIController::Desc &val, XMLContext* xmlcontext, bool create) {
	// planner visualisation base
	val.load(xmlcontext);
	// controller visualisation
	golem::XMLData(val.uiControllerVisDesc, xmlcontext->getContextFirst("controller"));
}

void golem::XMLData(UIController::Desc &val, Context* context, XMLContext* xmlcontext, bool create) {
	// controller description
	XMLData(val, xmlcontext, create);

	// load controller description
	val.pControllerDesc = Controller::Desc::load(context, xmlcontext->getContextFirst("controller"));
}

//------------------------------------------------------------------------------

void golem::XMLData(UIPlannerVis::Desc &val, XMLContext* xmlcontext, bool create) {
	xmlcontext = xmlcontext->getContextFirst("renderer");
	// planner visualisation
	XMLData(val.graphRenderData, xmlcontext, create);
	XMLData("show_duration", val.showDuration, xmlcontext, create);
	XMLData(val.chainCollisionAppearance, xmlcontext->getContextFirst("collision_bounds chain"), create);
	XMLData(val.jointCollisionAppearance, xmlcontext->getContextFirst("collision_bounds joint"), create);
}

void golem::XMLData(UIPlanner::Desc &val, XMLContext* xmlcontext, bool create) {
	// planner visualisation base
	val.load(xmlcontext);

	// controller visualisation
	XMLData(*val.pUIControllerDesc, xmlcontext, create);
	
	// planner visualisation
	val.uiPlannerVisDescSeq.clear();
	golem::XMLData(val.uiPlannerVisDescSeq, val.uiPlannerVisDescSeq.max_size(), xmlcontext, "planner");
}

void golem::XMLData(UIPlanner::Desc &val, Context* context, XMLContext* xmlcontext, bool create) {
	// planner description
	XMLData(val, xmlcontext, create);
	
	// load controller description
	val.pUIControllerDesc->pControllerDesc = Controller::Desc::load(context, xmlcontext->getContextFirst("controller"));
	
	// load planners descriptions
	val.plannerDescSeq.clear();
	std::pair<XMLContext::XMLContextMap::const_iterator, XMLContext::XMLContextMap::const_iterator> range = xmlcontext->getContextMap().equal_range("planner");
	for (XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second; ++i) {
		val.plannerDescSeq.push_back(Planner::Desc::load(context, const_cast<XMLContext*>(&i->second)));
	}
}

//------------------------------------------------------------------------------
