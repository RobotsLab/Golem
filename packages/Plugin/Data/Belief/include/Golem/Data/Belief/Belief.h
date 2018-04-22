//------------------------------------------------------------------------------
//               Simultaneous Perception And Manipulation (SPAM)
//------------------------------------------------------------------------------
//
// Copyright (c) 2010 University of Birmingham.
// All rights reserved.
//
// Intelligence Robot Lab.
// School of Computer Science
// University of Birmingham
// Edgbaston, Brimingham. UK.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy 
// of this software and associated documentation files (the "Software"), to deal 
// with the Software without restriction, including without limitation the 
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
// sell copies of the Software, and to permit persons to whom the Software is 
// furnished to do so, subject to the following conditions:
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimers.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimers in 
//       the documentation and/or other materials provided with the 
//       distribution.
//     * Neither the names of the Simultaneous Perception And Manipulation 
//		 (SPAM), University of Birmingham, nor the names of its contributors  
//       may be used to endorse or promote products derived from this Software 
//       without specific prior written permission.
//
// The software is provided "as is", without warranty of any kind, express or 
// implied, including but not limited to the warranties of merchantability, 
// fitness for a particular purpose and noninfringement.  In no event shall the 
// contributors or copyright holders be liable for any claim, damages or other 
// liability, whether in an action of contract, tort or otherwise, arising 
// from, out of or in connection with the software or the use of other dealings 
// with the software.
//
//------------------------------------------------------------------------------
//! @Author:   Claudio Zito
//! @Date:     25/03/2014
//------------------------------------------------------------------------------
#pragma once
#ifndef _GOLEM_DATA_BELIEF_H_
#define _GOLEM_DATA_BELIEF_H_

//------------------------------------------------------------------------------

#include <Golem/HBPlanner/Data.h>
#include <Golem/Plugin/Renderer.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Tools/Cloud.h>
#include <Golem/Plugin/DataI.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

class ItemBelief;
class HandlerBelief;

/** Data item representing belief state data.
*/
class GOLEM_LIBRARY_DECLDIR ItemBelief : public golem::data::Item, public golem::data::BeliefState {
public:
	friend class HandlerBelief;

	/** Query file */
	mutable File dataFile;

	/** Return the Belief description file to create a belief pointer */
	golem::Belief::Desc::Ptr getBeliefDesc() const;

	/** Set belief state */
	virtual void set(Belief* belief) {
		//this->belief = belief;
		this->belief.reset(belief);
	}
	/** Set the new belief state */
	void set(const Mat34 queryTransform, const RBPose::Sample::Seq& poses, const RBPose::Sample::Seq& hypotheses);
	/** Set model points to draw the belief state */
	virtual void setModelPoints(const std::string modelItem, const Mat34 modelFrame, const Cloud::PointSeq& points);
	/** Set query points to draw the belief state */
	virtual void setQueryPoints(const std::string queryItem, const Cloud::PointSeq& points);

	/** Set simulated query points to draw the belief state */
	virtual void setSimObject(const std::string queryItem, const Mat34& queryTransform, const Cloud::PointSeq& points) {
		this->queryItemSim = queryItem;
		this->queryTransformSim = queryTransform;
		this->queryPointsSim = points;
		dataFile.setModified(true);
	}

	virtual RBPose::Sample::Seq getPoses() const {
		return poses;
	}
	virtual RBPose::Sample::Seq getHypotheses() const {
		return hypotheses;
	}

	virtual const Mat34& getModelFrame() const {
		return modelFrame;
	}
	virtual const Mat34& getQueryTransform() const {
		return queryTransform;
	}
	virtual const std::string& getModelItem() const {
		return modelItem;
	}
	virtual const std::string& getQueryItem() const {
		return queryItem;
	}

	virtual const std::string& getQueryItemSim() const {
		return this->queryItemSim;
	}
	virtual const Mat34& getQueryTransformSim() const {
		return this->queryTransformSim;
	}

	virtual void showMeanPoseOnly(const bool show) {
		this->meanPoseOnly = show;
	}
	virtual void showQuery(const bool show) {
		this->showQueryDistribution = show;
	}
	virtual void showGroundTruth(const bool show) {
		this->showSimulated = show;
	}

	/** Clones item. */
	virtual Item::Ptr clone() const;

	/** Creates render buffer, the buffer can be shared and allocated on Handler */
	virtual void createRender();
	/** UIRenderer: Render on output device. */
	virtual void customRender();

protected:
	/** Data handler */
	data::HandlerBelief& handler;
	/** Pointer to the Belief state */
	//const golem::Belief* belief;
	golem::Belief::Ptr belief;

	/** Show only the mean pose */
	bool meanPoseOnly;
	/** Show poses distribution */
	bool showQueryDistribution;
	/** Show ground truth */
	bool showSimulated;

	/** Query frame */
	Mat34 modelFrame;
	/** Query transformation */
	Mat34 queryTransform;

	/** Model item: keep track of the model in the data bundle */
	std::string modelItem;
	/** Model points */
	Cloud::PointSeq modelPoints;
	/** Query item: keep track of the query in the data bundle */
	std::string queryItem;
	/** Query points */
	Cloud::PointSeq queryPoints;

	/** Query item: keep track of the query in the data bundle */
	std::string queryItemSim;
	/** Simulated query transformation */
	Mat34 queryTransformSim;
	/** Simulated query points */
	Cloud::PointSeq queryPointsSim;


	/** Transformation samples */
	RBPose::Sample::Seq poses;
	/** Transformation sub-samples */
	RBPose::Sample::Seq hypotheses;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, XMLContext* xmlcontext) const;

	/** Initialise data item */
	ItemBelief(HandlerBelief& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerBelief : public golem::data::Handler, public golem::UI {
public:
	friend class ItemBelief;

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Appearance {
	public:
		/** Show frame */
		bool showFrame;
		/** Show point cloud */
		bool showPoints;
		/** Number of displayed samples */
		U32 samples;
		/** Model feature appearance */
		Cloud::Appearance appearance;

		Appearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			showFrame = true;
			showPoints = false;
			samples = U32(0);
			appearance.setToDefault();
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			appearance.assertValid(ac);
		}
		/** Load descritpion from xml context. */
		virtual void load(Context& context, const XMLContext* xmlcontext);
	};

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public golem::data::Handler::Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Pointer to belief desc file */
		golem::Belief::Desc::Ptr pBeliefDescPtr;

		/** Model feature appearance */
		Appearance posesAppearance;
		/** Mean hypothesis feature appearance */
		Appearance meanPoseAppearance;
		/** Hypotheses feature appearance */
		Appearance hypothesisAppearance;
		/** Simulated object feature appearance */
		Appearance simulatedAppearance;

		/** Belief suffix */
		std::string beliefSuffix;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			pBeliefDescPtr.reset(new Belief::Desc());
			posesAppearance.setToDefault();
			meanPoseAppearance.setToDefault();
			hypothesisAppearance.setToDefault();
			simulatedAppearance.setToDefault();
			beliefSuffix = getFileExtBelief();
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(pBeliefDescPtr != nullptr, ac, "Belief desc: null pointer.");	
			posesAppearance.assertValid(ac);
			meanPoseAppearance.assertValid(ac);
			hypothesisAppearance.assertValid(ac);
			simulatedAppearance.assertValid(ac);
			Assert::valid(beliefSuffix.length() > 0, ac, "beliefSuffix: empty");
		}

		/** Load descritpion from xml context. */
		virtual void load(Context& context, const XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual Handler::Ptr create(Context &context) const;
	};

	/** File extension: hypothesis-based state (.hbs) */
	static std::string getFileExtBelief();

protected:
	/** Belief suffix */
	std::string beliefSuffix;
	/** Planner index */
	U32 plannerIndex;

	/** Pointer to the descriptor file */
	data::HandlerBelief::Desc desc;

	/** Debug renderer */
	DebugRenderer renderer;

	/** Creates render buffer */
	void createRender(const ItemBelief& item);
	/** UIRenderer: Render on output device. */
	virtual void render() const;
	/** UIRenderer: Render on output device. */
	virtual void customRender();
	/** UIRenderer: Render on output device. */
	virtual void customRender() const;

	/** UIRenderer: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	/** UIRenderer: Mouse motion handler. */
	virtual void motionHandler(int x, int y);
	/** UIRenderer: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);

	/** Construct empty item, accessible only by Data. */
	virtual Item::Ptr create() const;

	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerBelief(Context &context);
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

#endif /*_GOLEM_DATA_BELIEF_H_*/
