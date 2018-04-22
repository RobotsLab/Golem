/** @file ContactQuery.h
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
#ifndef _GOLEM_DATA_CONTACTQUERY_CONTACTQUERY_H_
#define _GOLEM_DATA_CONTACTQUERY_CONTACTQUERY_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Library.h>
#include <Golem/Plugin/Renderer.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Contact/Solver.h>
#include <Golem/Plugin/DataI.h>
#include <Golem/Plugin/PlannerI.h>
#include <Golem/Contact/Data.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

class ItemContactQuery;
class HandlerContactQuery;

/** Data item representing a collection of contact query densities.
*/
class GOLEM_LIBRARY_DECLDIR ItemContactQuery : public Item, public Convert, public ContactQuery {
public:
	friend class HandlerContactQuery;

	/** Query data */
	ContactQuery::Data data;
	/** Manifold map override */
	Contact::ManifoldSelector::Map manifoldMap;

	/** Query file */
	mutable File dataFile;

	/** Cluster index */
	mutable golem::U32 clusterIndex;
	/** Config index */
	mutable golem::U32 configIndex;
	/** Model index */
	mutable golem::U32 modelIndex;

	/** ContactQuery: sets contact query data. */
	virtual void setData(const ContactQuery::Data& data);
	/** ContactQuery: returns contact query data. */
	virtual const ContactQuery::Data& getData() const;

	/** ContactQuery: current config */
	virtual Contact::Config::Seq::const_iterator getConfig() const;
	/** ContactQuery: set current config */
	virtual void setConfig(Contact::Config::Seq::const_iterator config);

	/** ContactQuery: get config cluster */
	virtual Contact::Config::SetConstPtr getConfigCluster() const;
	/** ContactQuery: get config selection */
	virtual Contact::Config::SetConstPtr getConfigSelection() const;

	/** Clones item. */
	virtual Item::Ptr clone() const;

	/** Creates render buffer, the buffer can be shared and allocated on Handler */
	virtual void createRender();

protected:
	/** Data handler */
	HandlerContactQuery& handler;

	/** Config set */
	Contact::Config::SetConstPtr configSet;

	/** Convert: Convert current item */
	virtual Item::Ptr convert(const Handler& handler);
	/** Convert: return available interfaces */
	virtual const StringSeq& getConvertInterfaces() const;
	/** Convert: is supported by the interface */
	virtual bool isConvertSupported(const Handler& handler) const;

	/** Convert: is supported by the interface */
	virtual void getCurrentCluster(Contact::Config::SetConstPtr& configSet) const;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	/** Initialise data item */
	ItemContactQuery(HandlerContactQuery& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerContactQuery : public Handler, public HandlerPlanner, public UI, public Transform, public ContactQueryCallback {
public:
	friend class ItemContactQuery;

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Handler::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Planner index */
		golem::U32 plannerIndex;

		/** Cluster mode */
		bool modeCluster;

		/** Manipulator description */
		Manipulator::Desc::Ptr manipulatorDesc;
		/** Solver description */
		Solver::Desc::Ptr solverDesc;

		/** Collision detection with point cloud */
		bool collisionPoints;

		/** Clustering algorithm */
		Contact::Config::Cluster::Desc clusterDesc;

		/** Query appearance */
		ContactQuery::Data::Appearance appearance;

		/** Manipulator appearance - cluster */
		Manipulator::BoundsAppearance appearanceCluster;
		/** Manipulator appearance - cluster mean */
		Manipulator::BoundsAppearance appearanceClusterMean;

		/** Query suffix */
		std::string querySuffix;

		/** Transform clustering procedure */
		golem::Contact::Config::Cluster::Type clusterType;

		/** Manifold description */
		golem::Contact::ManifoldDesc contactManifoldDesc;

		/** Set to default */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			golem::data::Handler::Desc::setToDefault();

			plannerIndex = 0;

			manipulatorDesc.reset(new Manipulator::Desc);
			solverDesc.reset(new Solver::Desc);
			collisionPoints = true;
			clusterDesc.setToDefault();
			appearance.setToDefault();

			appearanceCluster.setToDefault();
			appearanceClusterMean.setToDefault();

			querySuffix = getFileExtContactQuery();

			clusterType = golem::Contact::Config::Cluster::TYPE_LIKELIHOOD;
			modeCluster = true;

			contactManifoldDesc.setToDefault();
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			golem::data::Handler::Desc::assertValid(ac);

			Assert::valid(manipulatorDesc != nullptr, ac, "manipulatorDesc: null");
			manipulatorDesc->assertValid(Assert::Context(ac, "manipulatorDesc->"));
			Assert::valid(solverDesc != nullptr, ac, "solverDesc: null");
			solverDesc->assertValid(Assert::Context(ac, "solverDesc->"));

			clusterDesc.assertValid(Assert::Context(ac, "clusterDesc."));

			appearance.assertValid(Assert::Context(ac, "appearance."));

			appearanceCluster.assertValid(Assert::Context(ac, "appearanceCluster."));
			appearanceClusterMean.assertValid(Assert::Context(ac, "appearanceClusterMean."));

			Assert::valid(querySuffix.length() > 0, ac, "querySuffix: empty");

			contactManifoldDesc.assertValid(Assert::Context(ac, "contactManifoldDesc."));
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual Handler::Ptr create(golem::Context &context) const;
	};

	/** File extension: query */
	static std::string getFileExtContactQuery();

protected:
	/** Solver index */
	golem::U32 plannerIndex;

	/** Manipulator description */
	Manipulator::Desc::Ptr manipulatorDesc;
	/** Solver description */
	Solver::Desc::Ptr solverDesc;
	/** Collision detection with point cloud */
	bool collisionPoints;
	/** Clustering algorithm */
	Contact::Config::Cluster::Desc clusterDesc;
	/** Query appearance */
	ContactQuery::Data::Appearance appearance;
	/** Manipulator appearance - cluster */
	Manipulator::BoundsAppearance appearanceCluster;
	/** Manipulator appearance - cluster mean */
	Manipulator::BoundsAppearance appearanceClusterMean;

	/** Manipulator */
	Manipulator::Ptr manipulator;
	/** Solver */
	Solver::Ptr solver;

	/** Transform clustering procedure */
	golem::Contact::Config::Cluster::Type clusterType;
	/** Convert cluster instead of a single solution */
	bool modeCluster;

	/** Rendering */
	golem::DebugRenderer renderer, rendererPoints, rendererCluster, rendererManifold, rendererBounds;

	/** Pseudorandom numer generator */
	mutable golem::Rand rand;

	/** Cluster index request */
	golem::I32 clusterIndexRequest;
	/** Config index request */
	golem::I32 configIndexRequest;
	/** Model index request */
	golem::I32 modelIndexRequest;
	/** Clustering likelihood */
	bool likelihoodRequest;
	/** Clustering type */
	bool typeRequest;
	/** Clustering pose */
	bool poseRequest;
	/** Subspace dist request */
	bool subspaceDistRequest;
	/** Selection */
	bool selectRequest;
	/** Clustering */
	bool clusterRequest;

	/** Manifold description */
	bool contactManifoldShowCluster;

	/** Query suffix */
	std::string querySuffix;

	/** Transform interfaces */
	StringSeq transformInterfaces;
	/** Convert interfaces */
	StringSeq convertInterfaces;

	/** ContactQueryCallback: Set contact evaluation expert */
	ContactEval* contactEval;

	/** Manifold description */
	golem::Contact::ManifoldDesc contactManifoldDesc;

	/** HandlerPlanner: Planner index. */
	virtual golem::U32 getPlannerIndex() const;
	/** HandlerPlanner: Sets planner and controllers. */
	virtual void set(const golem::Planner& planner, const ControllerId::Seq& controllerIDSeq);

	/** Print current config information */
	void printConfigInfo(const ItemContactQuery& item) const;

	/** Creates render buffer */
	void createRender(const ItemContactQuery& item);
	/** golem::UIRenderer: Render on output device. */
	virtual void render() const;
	/** golem::UIRenderer: Render on output device. */
	virtual void customRender() const;

	/** golem::UIRenderer: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	/** golem::UIRenderer: Mouse motion handler. */
	virtual void motionHandler(int x, int y);
	/** golem::UIRenderer: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);

	/** Construct empty item, accessible only by Data. */
	virtual Item::Ptr create() const;

	/** Transform: Transform input items */
	virtual Item::Ptr transform(const Item::List& input);
	/** Transform: return available interfaces */
	virtual const StringSeq& getTransformInterfaces() const;
	/** Transform: is supported by the interface */
	virtual bool isTransformSupported(const Item& item) const;

	/** Convert: Convert current item */
	virtual Item::Ptr convert(ItemContactQuery& item, const Handler& handler);
	/** Convert: is supported by the interface */
	virtual bool isConvertSupported(const Handler& handler) const;

	/** ContactQueryCallback: Set contact evaluation expert */
	virtual void setContactEval(ContactEval* contactEval = nullptr);

	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerContactQuery(golem::Context &context);
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_DATA_CONTACTQUERY_CONTACTQUERY_H_*/
