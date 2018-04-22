/** @file ContactModel.h
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
#ifndef _GOLEM_DATA_CONTACTMODEL_CONTACTMODEL_H_
#define _GOLEM_DATA_CONTACTMODEL_CONTACTMODEL_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Library.h>
#include <Golem/Plugin/Renderer.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Contact/Model.h>
#include <Golem/Contact/Aspect.h>
#include <Golem/Contact/Manifold.h>
#include <Golem/Contact/Configuration.h>
#include <Golem/Plugin/DataI.h>
#include <Golem/Plugin/PlannerI.h>
#include <Golem/Contact/Data.h>
#include <Golem/Math/Optimisation.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

class ItemContactModel;
class HandlerContactModel;

/** Data item representing contact training data.
*/
class GOLEM_LIBRARY_DECLDIR ItemContactModel : public Item , public ContactModel {
public:
	friend class HandlerContactModel;

	/** Training data */
	ContactModel::Data::Map dataMap, dataBackupMap;
	/** Query density selector type map */
	Contact::SelectorTypeMap selectorTypeMap;
	/** Manifold map override */
	Contact::ManifoldSelector::Map manifoldMap;

	/** Aspect data */
	Aspect::Data::Map aspectDataMap;

	/** Training data file */
	mutable File dataFile;

	/** Aspect data file */
	mutable File aspectDataFile;

	/** Data index */
	mutable golem::U32 dataIndex;
	/** View index */
	mutable golem::U32 viewIndex;
	/** Path index */
	mutable golem::U32 pathIndex;

	/** Aspect contact cluster index */
	mutable golem::U32 aspectClusterIndexI;
	/** Aspect contact cluster index */
	mutable golem::U32 aspectClusterIndexJ;
	/** Aspect view pruning index */
	mutable golem::U32 aspectPruningIndexI;
	/** Aspect view pruning index */
	mutable golem::U32 aspectPruningIndexJ;
	/** Aspect view config index */
	mutable golem::U32 aspectConfigIndex;

	/** data::Model: Sets contact model data. */
	virtual void setData(const ContactModel::Data::Map& dataMap);
	/** data::Model: Returns contact model data. */
	virtual const ContactModel::Data::Map& getData() const;

	/** Clones item. */
	virtual Item::Ptr clone() const;

	/** Creates render buffer, the buffer can be shared and allocated on Handler */
	virtual void createRender();

protected:
	/** Data handler */
	HandlerContactModel& handler;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	/** Initialise data item */
	ItemContactModel(HandlerContactModel& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerContactModel : public Handler, public HandlerPlanner, public UI, public Transform, public Import {
public:
	friend class ItemContactModel;

	/** Mode */
	enum Mode {
		/** Data */
		MODE_DATA,
		/** View */
		MODE_VIEW,
		/** Path */
		MODE_PATH,
		/** Aspect contact clusters */
		MODE_ASPECT_CLUSTERS,
		/** Aspect view pruning */
		MODE_ASPECT_PRUNING,
		/** Aspect view configs */
		MODE_ASPECT_CONFIGS,
		/** Size */
		MODE_SIZE,
	};

	/** Mode name string */
	static const char* modeName[MODE_SIZE];

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Handler::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Planner index */
		golem::U32 plannerIndex;

		/** Manipulator description */
		Manipulator::Desc::Ptr manipulatorDesc;
		/** Configuration description */
		Configuration::Desc::Ptr configurationDesc;

		/** Model descriptions */
		golem::Model::Desc::Map modelDescMap;

		/** Data operation description */
		data::ContactModel::Data::Desc dataDesc;
		/** Contact view aspect */
		Aspect::Desc::Ptr aspectDesc;
		/** Contact manifold */
		Manifold::Desc::Ptr manifoldDesc;

		/** Default contact type */
		std::string defaultType;

		/** Appearance */
		ContactModel::Data::Appearance appearance;

		/** Model suffix */
		std::string modelSuffix;
		/** Aspect data suffix */
		std::string aspectSuffix;

		/** Set to default */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			golem::data::Handler::Desc::setToDefault();

			plannerIndex = 0;

			manipulatorDesc.reset(new Manipulator::Desc);
			configurationDesc.reset(new Configuration::Desc);
			
			modelDescMap.insert(std::make_pair(Manipulator::Link::getName(Manipulator::Link::TYPE_ANY), golem::Model::Desc::Ptr(new golem::Model::Desc)));

			dataDesc.setToDefault();
			aspectDesc.reset(new Aspect::Desc);
			manifoldDesc.reset(new Manifold::Desc);

			defaultType = "Any";

			appearance.setToDefault();

			modelSuffix = getFileExtContactModel();
			aspectSuffix = getFileExtContactAspect();
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			golem::data::Handler::Desc::assertValid(ac);

			Assert::valid(manipulatorDesc != nullptr, ac, "manipulatorDesc: null");
			manipulatorDesc->assertValid(Assert::Context(ac, "manipulatorDesc->"));
			Assert::valid(configurationDesc != nullptr, ac, "configurationDesc: null");
			configurationDesc->assertValid(Assert::Context(ac, "configurationDesc->"));

			Assert::valid(!modelDescMap.empty(), ac, "modelDescMap: empty");
			for (golem::Model::Desc::Map::const_iterator i = modelDescMap.begin(); i != modelDescMap.end(); ++i) {
				Assert::valid(i->second != nullptr, ac, "modelDescMap[]: null");
				i->second->assertValid(Assert::Context(ac, "modelDescMap[]->"));
			}

			dataDesc.assertValid(Assert::Context(ac, "dataDesc."));
			Assert::valid(aspectDesc != nullptr, ac, "aspectDesc: null");
			aspectDesc->assertValid(Assert::Context(ac, "aspectDesc->"));
			Assert::valid(manifoldDesc != nullptr, ac, "manifoldDesc: null");
			manifoldDesc->assertValid(Assert::Context(ac, "manifoldDesc->"));

			appearance.assertValid(Assert::Context(ac, "appearance."));

			Assert::valid(modelSuffix.length() > 0, ac, "modelSuffix: empty");
			Assert::valid(aspectSuffix.length() > 0, ac, "aspectSuffix: empty");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual Handler::Ptr create(golem::Context &context) const;
	};

	/** File extension: model */
	static std::string getFileExtContactModel();
	/** File extension: aspect */
	static std::string getFileExtContactAspect();

protected:
	/** Planner index */
	golem::U32 plannerIndex;

	/** Manipulator description */
	Manipulator::Desc::Ptr manipulatorDesc;
	/** Configuration description */
	Configuration::Desc::Ptr configurationDesc;
	/** Model descriptions */
	golem::Model::Desc::Map modelDescMap;
	/** Data operation description */
	data::ContactModel::Data::Desc dataDesc;
	/** Contact view aspect */
	Aspect::Desc::Ptr aspectDesc;
	/** Contact manifold */
	Manifold::Desc::Ptr manifoldDesc;
	/** Appearance */
	ContactModel::Data::Appearance appearance;

	/** Manipulator */
	Manipulator::Ptr manipulator;
	/** Configuration */
	Configuration::Ptr configuration;

	/** Models */
	golem::Model::Map modelMap;

	/** Contact view aspect */
	Aspect::Ptr aspect;

	/** Contact manifold */
	Manifold::Ptr manifold;

	/** Default contact type */
	std::string defaultType;

	/** Rendering */
	golem::DebugRenderer renderer, rendererPoints;

	/** Mode */
	golem::U32 mode;
	/** Index i request */
	golem::I32 indexIRequest;
	/** Index j request */
	golem::I32 indexJRequest;

	/** Model info request */
	bool modelInfoRequest;
	/** Subspace dist request */
	bool subspaceDistRequest;

	/** Model suffix */
	std::string modelSuffix;
	/** Aspect data suffix */
	std::string aspectSuffix;

	/** Transform interfaces */
	StringSeq transformInterfaces;

	/** Import types */
	StringSeq importTypes;

	/** Create training data */
	virtual bool create(const std::string& name, golem::I32 objectIndex, const Point3D& points, const golem::WaypointCtrl::Seq& waypoints, golem::data::ContactModel::Data& data) const;
	
	/** HandlerPlanner: Planner index. */
	virtual golem::U32 getPlannerIndex() const;
	/** HandlerPlanner: Sets planner and controllers. */
	virtual void set(const golem::Planner& planner, const ControllerId::Seq& controllerIDSeq);

	/** Info */
	std::string toString(const golem::data::ContactModel::Data& data) const;
	/** Info */
	std::string toString(golem::U32 viewIndex, golem::U32 pathIndex, const golem::data::ContactModel::Data& data) const;

	/** Creates render buffer */
	void createRender(const ItemContactModel& item);
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

	/** Import: Import from file */
	virtual Item::Ptr import(const std::string& path);
	/** Import: Available file types */
	virtual const StringSeq& getImportFileTypes() const;

	/** Transform: Transform input items */
	virtual Item::Ptr transform(const Item::List& input);
	/** Transform: return available interfaces */
	virtual const StringSeq& getTransformInterfaces() const;
	/** Transform: is supported by the interface */
	virtual bool isTransformSupported(const Item& item) const;

	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerContactModel(golem::Context &context);
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_DATA_CONTACTMODEL_CONTACTMODEL_H_*/
