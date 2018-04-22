/** @file UIPlanner.h
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
#ifndef _GOLEM_APP_UIPLANNER_H_
#define _GOLEM_APP_UIPLANNER_H_

//------------------------------------------------------------------------------

#include <Golem/Planner/Planner.h>
#include <Golem/App/GraphRenderer.h>
#include <Golem/App/UIController.h>
#include <map>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgUIPlanner, MsgUIController)
MESSAGE_DEF(MsgUIPlannerUnknownController, MsgUIPlanner)
MESSAGE_DEF(MsgUIPlannerUnknownPlanner, MsgUIPlanner)

//------------------------------------------------------------------------------

class UIPlannerVis : public Object, public Planner::CallbackDataSync, public Controller::CallbackDataSync {
public:
	typedef std::vector<UIPlannerVis*> PtrSeq;

	typedef std::vector<golem::Mat34> Mat34Seq;

	/** Bounds data */
	class BoundsData {
	public:
		/** Chain bounds */
		typedef Chainspace::Coord<BoundsData> ChainCoords;
		/** Joint bounds */
		typedef Configspace::Coord<BoundsData> JointCoords;

		/** Appearance */
		class Appearance {
		public:
			/** Show bounds */
			bool show;
			/** Show bounds solid */
			bool showSolid;
			/** Show bounds wire frames */
			bool showWire;
			/** Bounds solid colour */
			golem::RGBA solidColour;
			/** Bounds wire colour */
			golem::RGBA wireColour;
			/** Bounds wireframe thickness */
			golem::Real wireWidth;

			/** Constructs from description object */
			Appearance() {
				setToDefault();
			}
			/** Sets the parameters to the default values */
			void setToDefault() {
				show = false;
				showSolid = false;
				showWire = true;
				solidColour = golem::RGBA(golem::U8(0), golem::U8(0), golem::U8(255), golem::U8(127));
				wireColour = golem::RGBA(golem::U8(0), golem::U8(0), golem::U8(255), golem::U8(255));
				wireWidth = golem::Real(1.0);
			}
			/** Checks if the appearance is valid. */
			bool isValid() const {
				if (wireWidth < REAL_EPS)
					return false;
				return true;
			}
		};

		/** Bounds */
		golem::Bounds::Seq bounds;
		/** Poses */
		Mat34Seq poses;

		/** Create */
		void create(const golem::Bounds::Desc::Seq& boundsDesc);
		/** Pose */
		void setPose(const Mat34& pose);
		/** Render */
		void render(const Appearance& appearance, DebugRenderer& renderer) const;

		/** Clear */
		void clear() {
			bounds.clear();
			poses.clear();
		}
		/** Empty */
		bool empty() const {
			return bounds.empty() || poses.empty();
		}
	};

	/** Graph render data */
	class GraphRenderData {
		friend class UIPlannerVis;

	protected:
		SecTmReal tmEnd;

	public:
		typedef shared_ptr<GraphRenderData> Ptr;
		typedef std::map<SecTmReal, Ptr> Map;
		typedef std::pair<SecTmReal, Ptr> Pair;

		/** Goal renderer */
		GraphRenderer goalRenderer;
		/** Goal population renderer */
		GraphRenderer goalPopulationRenderer;
		/** Global waypoint graph renderer */
		GraphRenderer globalGraphRenderer;
		/** Global waypoint path renderer */
		GraphRenderer globalPathRenderer;
		/** Local waypoint graph renderer */
		GraphRenderer localGraphRenderer;
		/** Local waypoint path renderer */
		GraphRenderer localPathRenderer;
		/** Optimised waypoint path renderer */
		GraphRenderer optimisedPathRendererEx;
		
		/** Constructs description object */
		GraphRenderData() {
			setToDefault();
		}
		/** Constructs description object */
		GraphRenderData(const GraphRenderData& graphRenderData, const Heuristic& heuristic) {
			*this = graphRenderData;

			Chainspace::Coord<bool> showFrame;
			for (Chainspace::Index i = heuristic.getController().getStateInfo().getChains().begin(); i < heuristic.getController().getStateInfo().getChains().end(); ++i)
				showFrame[i] = heuristic.getChainDesc()[i]->enabledLin || heuristic.getChainDesc()[i]->enabledAng;
			
			goalRenderer.showFrame = showFrame;
			goalPopulationRenderer.showFrame = showFrame;
			globalGraphRenderer.showFrame = showFrame;
			globalPathRenderer.showFrame = showFrame;
			localGraphRenderer.showFrame = showFrame;
			localPathRenderer.showFrame = showFrame;
			optimisedPathRendererEx.showFrame = showFrame;
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			goalRenderer.setToDefault();
			goalRenderer.vertexFrameShow = true;
			goalRenderer.show = true;
			goalPopulationRenderer.setToDefault();
			goalPopulationRenderer.vertexPositionShow = true;
			goalPopulationRenderer.show = true;

			globalGraphRenderer.setToDefault();
			globalGraphRenderer.vertexPositionShow = true;
			globalPathRenderer.setToDefault();
			globalPathRenderer.edgeColour = RGBA::CYAN;
			globalPathRenderer.edgeShow = true;

			localGraphRenderer.setToDefault();
			localGraphRenderer.vertexPositionShow = true;
			localPathRenderer.setToDefault();
			localPathRenderer.edgeColour = RGBA::MAGENTA;
			localPathRenderer.edgeShow = true;

			optimisedPathRendererEx.setToDefault();
			optimisedPathRendererEx.edgeColour = RGBA::BLACK;
			optimisedPathRendererEx.edgeShow = true;
			optimisedPathRendererEx.vertexFrameSize.set(Real(0.02));
			optimisedPathRendererEx.vertexFrameShow = true;
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (!goalRenderer.isValid() || !goalPopulationRenderer.isValid() || !globalGraphRenderer.isValid() || !globalPathRenderer.isValid() || !localGraphRenderer.isValid() || !localPathRenderer.isValid() || !optimisedPathRendererEx.isValid())
				return false;
			return true;
		}
	};
	
	/** Object description */
	class Desc {
	public:
		typedef std::vector<Desc> Seq;

		/** Graph render data */
		GraphRenderData graphRenderData;
		/** Maximum show time */
		SecTmReal showDuration;

		/** Chain collision appearance */
		BoundsData::Appearance chainCollisionAppearance;
		/** Joint collision appearance */
		BoundsData::Appearance jointCollisionAppearance;

		/** Constructs UIController description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Constructs object */
		virtual Object::Ptr create(Scene& scene, UIControllerVis& uiControllerVis, Planner& planner) const {
			Object::Ptr object(new UIPlannerVis(scene, uiControllerVis, planner));
			static_cast<UIPlannerVis&>(*object).create(*this);
			return object;
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			graphRenderData.setToDefault();
			showDuration = SecTmReal(60.0); // 60 sec
			chainCollisionAppearance.setToDefault();
			jointCollisionAppearance.setToDefault();
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!graphRenderData.isValid() || showDuration < SEC_TM_REAL_ZERO)
				return false;
			if (!chainCollisionAppearance.isValid() || !jointCollisionAppearance.isValid())
				return false;
			return true;
		}
	};

protected:
	/** Controller interface */
	Controller& controller;
	/** Planner interface */
	Planner& planner;

	/** UIControllerVis */
	UIControllerVis& uiControllerVis;

	/** Graph render data */
	GraphRenderData graphRenderData;
	/** Maximum show time */
	SecTmReal showDuration;

	mutable GraphRenderData::Map dataPlanned, dataSent;
	mutable CriticalSection csPlanned, csSent;

	mutable DebugRenderer collisionRenderer;

	/** Chain collision appearance */
	BoundsData::Appearance chainCollisionAppearance;
	/** Chain collision bounds */
	mutable BoundsData::ChainCoords chainCollisionBounds;

	/** Joint collision appearance */
	BoundsData::Appearance jointCollisionAppearance;
	/** Joint bounds */
	mutable BoundsData::JointCoords jointCollisionBounds;

	/** Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);
	/** Renders the UIPlannerVis. */
	virtual void render() const;

	/** Creates object. */
	void create(const Desc& desc);
	/** Releases resources */
	virtual void release();
	
	/** Objects can be constructed only in the Scene context. */
	UIPlannerVis(Scene& scene, UIControllerVis& uiControllerVis, Planner& planner);

public:
	/** Controller::Callback */
	virtual void syncChainBoundsDesc(Chain* pChain);
	/** Controller::Callback */
	virtual void syncJointBoundsDesc(Joint* pJoint);
	/** Controller::Callback */
	virtual void syncSend(const Controller::State* begin);
	
	/** Planner::Callback */
	virtual void syncCollisionBounds();
	/** Planner::Callback */
	virtual void syncFindTrajectory(Controller::Trajectory::const_iterator begin, Controller::Trajectory::const_iterator end, const GenWorkspaceChainState* wend = NULL);

	/** Returns the controller */
	const Controller &getController() const {
		return controller;
	}
	Controller &getController() {
		return controller;
	}

	/** Returns the Planner */
	const Planner &getPlanner() const {
		return planner;
	}
	Planner &getPlanner() {
		return planner;
	}
};

//------------------------------------------------------------------------------

class UIPlanner : public Object, public Controller::CallbackDataSync {
public:
	typedef shared_ptr<UIPlanner> Ptr;
	typedef std::vector<Ptr> PtrSeq;

	/** Object description */
	class Desc : public Object::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(UIPlanner, Object::Ptr, Scene&)

	public:
		typedef shared_ptr<Desc> Ptr;
		typedef std::vector<Ptr> PtrSeq;

		/** Single UIController description */
		UIController::Desc::Ptr pUIControllerDesc;

		/** Multiple Planner description */
		Planner::Desc::Seq plannerDescSeq;
		/** Multiple Planner visualisation description */
		UIPlannerVis::Desc::Seq uiPlannerVisDescSeq;

		/** Constructs UIController description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			pUIControllerDesc.reset(new UIController::Desc());
			plannerDescSeq.clear();
			uiPlannerVisDescSeq.clear();
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (pUIControllerDesc == NULL || !pUIControllerDesc->isValid())
				return false;
			if (plannerDescSeq.empty() || uiPlannerVisDescSeq.empty())
				return false;
			for (Planner::Desc::Seq::const_iterator i = plannerDescSeq.begin(); i != plannerDescSeq.end(); ++i)
				if (*i == NULL || !(*i)->isValid())
					return false;
			for (UIPlannerVis::Desc::Seq::const_iterator i = uiPlannerVisDescSeq.begin(); i != uiPlannerVisDescSeq.end(); ++i)
				if (!i->isValid())
					return false;
			return true;
		}
	};

protected:
	/** UIController */
	UIController* pUIController;

	/** Multiple Planner */
	Planner::Seq plannerSeq;
	/** Multiple Planner */
	Planner::PtrSeq plannerPtrSeq;

	/** Multiple Planner visualisation */
	UIPlannerVis::PtrSeq uiPlannerVisPtrSeq;

	/** Creates UIPlanner. */
	void create(const Desc& desc);
	/** Releases resources */
	virtual void release();

	/** Objects can be constructed only in the Scene context. */
	UIPlanner(Scene &scene);

public:
	/** Controller::Callback */
	virtual void syncChainBoundsDesc(Chain* pChain);
	/** Controller::Callback */
	virtual void syncJointBoundsDesc(Joint* pJoint);
	/** Controller::Callback */
	virtual void syncSend(const Controller::State* begin);
	
	/** Returns the controller */
	const UIController &getUIController() const {
		return *pUIController;
	}
	UIController &getUIController() {
		return *pUIController;
	}

	/** Returns collection of all scene bounds different than the controller bounds.
	* @return				pointer to the collection of bounds
	*/
	Bounds::SeqPtr getCollisionBounds() const {
		return pUIController->getCollisionBounds();
	}

	/** Returns controller joints' bounds group
	* @return				bounds group
	*/
	U32 getControllerBoundsGroup() const {
		return pUIController->getControllerBoundsGroup();
	}

	/** Returns bounds group of the Actors which can collide with device
	* @return				bounds group
	*/
	U32 getCollisionBoundsGroup() const {
		return pUIController->getCollisionBoundsGroup();
	}

	/** Sets bounds group of the Actors which can collide with device
	* @param	collisionGroup	bounds group
	*/
	void setCollisionBoundsGroup(U32 collisionGroup) {
		pUIController->setCollisionBoundsGroup(collisionGroup);
	}


	/** Returns the controller */
	const Controller &getController() const {
		return pUIController->getController();
	}
	Controller &getController() {
		return pUIController->getController();
	}

	/** Multiple Planner description */
	const Planner::PtrSeq& getPlannerSeq() const {
		return plannerPtrSeq;
	}
	/** Multiple Planner visualisation description */
	const UIPlannerVis::PtrSeq& getUIPlannerVisSeq() const {
		return uiPlannerVisPtrSeq;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_APP_UIPLANNER_H_*/
