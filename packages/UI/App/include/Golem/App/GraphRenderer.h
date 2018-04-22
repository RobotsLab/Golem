/** @file GraphRenderer.h
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_APP_GRAPHRENDERER_H_
#define _GOLEM_APP_GRAPHRENDERER_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Controller.h>
#include <Golem/Plugin/Renderer.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Graph renderer */
class GraphRenderer : public DebugRenderer {
public:
	/** Vertex position colour */
	RGBA vertexPositionColour;
	/** Display vertex position */
	bool vertexPositionShow;
	/** Vertex frame size */
	Vec3 vertexFrameSize;
	/** Display vertex frame */
	bool vertexFrameShow;
	/** Edge colour */
	RGBA edgeColour;
	/** Display edge */
	bool edgeShow;
	/** Display frame */
	Chainspace::Coord<bool> showFrame;
	/** Display */
	bool show;

	/** Constructs the description object. */
	GraphRenderer() {
		setToDefault();
	}
	/** Sets the parameters to the default values */
	void setToDefault() {
		vertexPositionColour = RGBA::YELLOW;
		vertexPositionShow = false;
		vertexFrameSize.set(Real(0.05));
		vertexFrameShow = false;
		edgeColour = RGBA::WHITE;
		edgeShow = false;
		showFrame.fill(true);
		show = false;
	}
	/** Checks if the description is valid. */
	bool isValid() const {
		if (!vertexFrameSize.isPositive())
			return false;
		return true;
	}
	/** create render data */
	template <typename _Ptr> void fromFrame(_Ptr begin, _Ptr end) {
		for (_Ptr i = begin, j = begin; i != end; j = i, ++i)
			make(*j, *i, i != begin);
	}
	/** create render data */
	template <typename _Ptr> void fromWorkspace(const Controller& controller, _Ptr begin, _Ptr end) {
		for (_Ptr i = begin, j = begin; i != end; j = i, ++i)
			make(controller, j->wpos, i->wpos, i != begin);
	}
	/** create render data */
	template <typename _Ptr> void fromConfigspace(const Controller& controller, _Ptr begin, _Ptr end) {
		WorkspaceChainCoord prev, next;
		for (_Ptr i = begin; i != end; ++i)
			make(controller, i->cpos, prev, next, i != begin);
	}
	/** render data */
	void render() const {
		if (show) DebugRenderer::render();
	}

protected:
	void make(const Mat34& prev, const Mat34& next, bool edge) {
		if (vertexFrameShow)
			addAxes(next, vertexFrameSize);
		if (vertexPositionShow)
			addPoint(next.p, vertexPositionColour);
		if (edge && edgeShow)
			addLine(prev.p, next.p, edgeColour);
	}
	void make(const Controller& controller, const WorkspaceChainCoord& wprev, const WorkspaceChainCoord& wnext, bool edge) {
		for (Chainspace::Index i = controller.getStateInfo().getChains().begin(); i < controller.getStateInfo().getChains().end(); ++i)
			if (showFrame[i]) {
				make(wprev[i], wnext[i], edge);
			}
	}
	void make(const Controller& controller, const ConfigspaceCoord& cnext, WorkspaceChainCoord& wprev, WorkspaceChainCoord& wnext, bool edge) {
		controller.chainForwardTransform(cnext, wnext);
		for (Chainspace::Index i = controller.getStateInfo().getChains().begin(); i < controller.getStateInfo().getChains().end(); ++i)
			if (showFrame[i]) {
				wnext[i].multiply(wnext[i], controller.getChains()[i]->getReferencePose()); // reference pose
				make(wprev[i], wnext[i], edge);
			}
		wprev = wnext;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_APP_GRAPHRENDERER_H_*/
