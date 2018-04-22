/** @file Sensor.h
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
#ifndef _GOLEM_PLUGIN_SENSOR_H_
#define _GOLEM_PLUGIN_SENSOR_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Defs.h>
#include <Golem/Plugin/Library.h>
#include <Golem/Plugin/Renderer.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Sensor interface */
class Sensor : public Library {
public:
	typedef golem::shared_ptr<Sensor> Ptr;

	/** Map */
	typedef std::map<std::string, Ptr> Map;
	/** Seq */
	typedef std::vector<golem::Sensor*> Seq;
	/** SeqSeq */
	typedef std::vector<Seq> SeqSeq;

	typedef golem::ConfigMat34 Config;

	/** Configuration query */
	typedef std::function<void(golem::U32 joint, Config& config)> ConfigQuery;

	/** Sensor appearance */
	class Appearance {
	public:
		/** Frame size */
		golem::Vec3 frameSize;
		/** Show camera frame */
		bool frameShow;

		/** Shape colour */
		golem::RGBA shapeColour;
		/** Show sensor */
		bool shapeShow;

		/** Constructs description. */
		Appearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			frameSize.set(golem::Real(0.03), golem::Real(0.03), golem::Real(0.05));
			frameShow = true;
			shapeColour = golem::RGBA(127, 127, 127, 255);
			shapeShow = true;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(frameSize.isPositive(), ac, "frameSize: <= 0");
		}
		/** Load from xml context */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Sensor description */
	class Desc : public Library::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Snapshot capture handler */
		std::string snapshotHandler;
		/** Sequence capture handler */
		std::string sequenceHandler;

		/** Sensor appearance */
		Appearance appearance;
		/** Sensor shape */
		golem::Bounds::Desc::Seq shapeDesc;

		/** Configuration joint */
		golem::U32 configJoint;
		/** Configuration handler */
		ConfigQuery configQuery;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		/** Destroys description. */
		virtual ~Desc() {}
		
		/** Sets the parameters to the default values. */
		void setToDefault() {
			Library::Desc::setToDefault();

			libraryPrefix = "Golem";
			libraryName = "golem sensor";

			snapshotHandler.clear();
			sequenceHandler.clear();

			appearance.setToDefault();
			shapeDesc.clear();

			configJoint = 0;
			configQuery = nullptr;
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Library::Desc::assertValid(ac);

			//Assert::valid(!snapshotHandler.empty(), ac, "snapshotHandler: empty");
			//Assert::valid(!sequenceHandler.empty(), ac, "sequenceHandler: empty");

			appearance.assertValid(Assert::Context(ac, "appearance."));
			for (golem::Bounds::Desc::Seq::const_iterator i = shapeDesc.begin(); i != shapeDesc.end(); ++i)
				Assert::valid((*i)->isValid(), ac, "shapeDesc[]: invalid");
		}

		/** Creates the object from the description. */
		virtual Sensor::Ptr create(golem::Context &context) const = 0;

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
	};

	/** Batch initialisation */
	virtual void init(const Map& map) {}

	/** Snapshot capture handler */
	const std::string& getSnapshotHandler() const {
		return snapshotHandler;
	}
	/** Sequence capture handler */
	const std::string& getSequenceHandler() const {
		return sequenceHandler;
	}

	/** Variable mounting */
	bool hasVariableMounting() const {
		return configJoint > 0;
	}

	/** Configuration joint */
	golem::U32 getConfigJoint() const {
		return configJoint;
	}

	/** Current config */
	void getConfig(Config& config) const;

	/** Curent sensor frame */
	virtual golem::Mat34 getFrame() const = 0;

	/** Sensor appearance */
	const Appearance& getAppearance() const {
		return appearance;
	}

	/** Draw sensor at the current pose */
	virtual void draw(const Appearance& appearance, golem::DebugRenderer& renderer) const;

protected:
	/** Snapshot capture handler */
	std::string snapshotHandler;
	/** Sequence capture handler */
	std::string sequenceHandler;

	/** Configuration joint */
	golem::U32 configJoint;
	/** Configuration handler */
	ConfigQuery configQuery;

	/** Camera appearance */
	Appearance appearance;
	/** Camera shape */
	mutable golem::Bounds::Seq shape;
	/** Camera shape frame */
	Mat34Seq shapeFrame;

	/** Creates/initialises the Sensor */
	void create(const Desc& desc);
	
	/** Constructs the Sensor */
	Sensor(golem::Context& context);
};

//------------------------------------------------------------------------------

/** Real and/or virtual sensor identifier finds mapping:
*	1) findSensor(): string id (library+config) -> golem::Sensor*
*/
class SensorId {
public:
	/** Seq */
	typedef std::vector<SensorId> Seq;

	/** String id */
	std::string id;
	/** Sensor index */
	golem::U32 index;

	/** Default init */
	SensorId() {
		clear();
	}
	/** Default init */
	void clear() {
		id.clear();
		index = -1;
	}

	/** Available id */
	bool hasId() const {
		return id.length() > 0;
	}
	/** Available range */
	bool hasIndex() const {
		return index != -1;
	}

	/** String */
	std::string toString() const;

	/** Assert that the description is valid. */
	bool isValid() const {
		return hasId() || hasIndex();
	}
	/** Assert that the description is valid. */
	void assertValid(const Assert::Context& ac) const {
		Assert::valid(isValid(), ac, "invalid index for empty id");
	}

	/** Find sensor */
	golem::Sensor* findSensor(const Sensor::Seq& sensorSeq) const;
};

//------------------------------------------------------------------------------

/** Sequentially find sensors from given ids */
void findSensor(const Sensor::Map& sensors, const StringSeq& idSeq, Sensor::Seq& sensorSeq);

/** Reads/writes object from/to a given XML context */
void XMLData(golem::SensorId& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_PLUGIN_SENSOR_H_*/
