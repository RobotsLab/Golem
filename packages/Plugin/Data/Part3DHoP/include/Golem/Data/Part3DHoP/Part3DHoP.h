/** @file Part3DHoP.h
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
#ifndef _GOLEM_DATA_PART_3D_HOP_PART_3D_HOP_H_
#define _GOLEM_DATA_PART_3D_HOP_PART_3D_HOP_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Library.h>
#include <Golem/Plugin/Renderer.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Tools/Cloud.h>
#include <Golem/Tools/Import.h>
#include <Golem/Tools/Image.h>
#include <Golem/Plugin/DataI.h>
#include <Golem/Plugin/SensorI.h>
#include <Golem/Contact/Data.h>
#include <Golem/Data/Part3DHoP/Part3DHoPInterop.h>
#include "GolemInteropStreamSocket.h"

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

/** Part property */
class Part3DProperty {
public:
	/** Map */
	typedef std::map<data::Part3D::Index, Part3DProperty> Map;

	/** Set */
	class Set {
	public:
		/** Level properties */
		Part3DProperty::Map level;
		/** Model properties */
		Part3DProperty::Map model;
		/** Realisation properties */
		Part3DProperty::Map realisation;

		/** Constructs description. */
		Set() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			level.clear();
			model.clear();
			realisation.clear();
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(!level.empty(), ac, "level: empty");
			for (Part3DProperty::Map::const_iterator i = level.begin(); i != level.end(); ++i)
				i->second.assertValid(Assert::Context(ac, "level[]->"));
			for (Part3DProperty::Map::const_iterator i = model.begin(); i != model.end(); ++i)
				i->second.assertValid(Assert::Context(ac, "model[]->"));
			for (Part3DProperty::Map::const_iterator i = realisation.begin(); i != realisation.end(); ++i)
				i->second.assertValid(Assert::Context(ac, "realisation[]->"));
		}
	};

	/** Part weight multiplier */
	golem::Real weight;
	/** Part frame standard deviation */
	RBDist frameStdDev;

	/** Constructs description. */
	Part3DProperty() {
		setToDefault();
	}
	/** Sets the parameters to the default values. */
	void setToDefault() {
		weight = golem::REAL_ONE;
		frameStdDev.set(golem::Real(0.02), golem::Real(100.0));
	}
	/** Assert that the description is valid. */
	void assertValid(const Assert::Context& ac) const {
		Assert::valid(weight > golem::REAL_EPS, ac, "weight: < eps");
		Assert::valid(frameStdDev.isValid(), ac, "frameStdDev: invalid");
	}
};

void XMLData(golem::data::Part3DProperty::Map::value_type& val, golem::XMLContext* context, bool create = false);
void XMLData(golem::data::Part3DProperty::Set& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

class ItemPart3DHoP;
class HandlerPart3DHoP;

/** Data item representing hierarchies of parts 3D.
*/
class GOLEM_LIBRARY_DECLDIR ItemPart3DHoP : public Item, public CloudData, public Normal3D, public Part3D {
public:
	friend class HandlerPart3DHoP;

	/** Point cloud */
	using CloudData::cloud;

	/** Models */
	data::Part3D::IndexSetMap models;

	/** Realisations */
	data::Part3D::IndexSetMap realisations;
	/** Parts */
	data::Part3D::Part::Map parts;
	/** Part-cloud point indices */
	data::Part3D::IndexSetMap indices;

	/** Cloud file */
	mutable File cloudFile;
	/** Part3DHoP file */
	mutable File part3DFile;

	/** Current cloud index */
	golem::U32 cloudIndex;
	/** Current level index */
	golem::U32 levelIndex;
	/** Current part index */
	golem::U32 partIndex;

	/** Part properties */
	Part3DProperty::Set partProperties;

	/** Normal standard deviation */
	golem::Real normalStdDev;
	
	/** Clones item. */
	virtual Item::Ptr clone() const;

	/** Creates render buffer, the buffer can be shared and allocated on Handler */
	virtual void createRender();

	/** Point3D: Total number of available points */
	virtual size_t getSize() const;
	/** Point3D: Dimensions */
	virtual void getDimensions(size_t& width, size_t& height) const;
	/** Point3D: Sample point. */
	virtual size_t samplePoint(golem::Rand& rand) const;
	/** Point3D: Conditional point */
	virtual Point3D::Point getPoint(size_t index) const;
	/** Point3D: Colour */
	virtual golem::RGBA getColour(size_t index) const;

	/** Point3D: Point transform */
	virtual void transform(const golem::Mat34& trn);
	/** Point3D: Sensor frame. */
	virtual void getSensorFrame(golem::Mat34& frame) const;

	/** Normal3D: Query density normal covariance */
	virtual Point3D::Real getNormalCovariance() const;
	/** Normal3D: Select normal given point index. */
	virtual Point3D::Vec3 getNormal(size_t index) const;

	/** Part3D: Return complete realisation map. */
	virtual void getPart(Part::Map& partMap) const;
	/** Part3D: Return realisation map correlated with a given point index. */
	virtual void getPart(size_t index, Part3D::Part::Map& partMap) const;
	/** Part3D: Query density frame covariance for a given realisation */
	virtual Point3D::RBDist getFrameCovariance(Part3D::Index realisation) const;

protected:
	/** Data handler */
	HandlerPart3DHoP& handler;

	/** Model part indices and level map */
	data::Part3D::IndexMap modelToLevelMap;
	/** Realisation part indices and level map */
	data::Part3D::IndexMap partToLevelMap;
	/** Realisation part indices and level map */
	data::Part3D::IndexSetMap levelToPartMap;
	/** Part-cloud point and part indices - top-down mapping */
	data::Part3D::IndexSetMap realisationsRev;
	/** Cloud point to part indices mapping */
	data::Part3D::IndexSetMap indexToPartMap;

	/** Request cloud from server */
	void load(interop::Stream& stream, bool frameTrn = false);
	/** Process data */
	void process();

	/** Print parts info */
	void printPart() const;

	/** Part property from realisation */
	Part3DProperty::Map::const_iterator getPart3DProperty(Part3D::Index realisation) const;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	/** Initialise data item */
	ItemPart3DHoP(HandlerPart3DHoP& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerPart3DHoP : public Handler, public UI, public Generate, public Transform {
public:
	friend class ItemPart3DHoP;

	/** File extension: part3D */
	static const std::string FILE_EXT_CONTACT_PART3D;
	/** File extension: log */
	static const std::string FILE_EXT_LOG;

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Handler::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Hop3D server host name */
		std::string host;
		/** Hop3D server host port */
		unsigned short port;

		/** Cloud frame transformation */
		bool cloudFrameTrn;

		/** Cloud appearance */
		Cloud::Appearance cloudAppearance;
		/** Cloud suffix */
		std::string cloudSuffix;

		/** Part colour */
		golem::RGBA partColour;
		/** Bottom-up part activation colour */
		golem::RGBA partColourActive;
		/** Indirect/high-level level part colour */
		golem::RGBA partColourIndirect;
		/** Part frame size */
		bool partFrameShow;
		/** Part frame size */
		golem::Vec3 partFrameSize;
		/** Selected part frame */
		golem::Vec3 partFrameSizeSelect;
		/** Part cloud point size */
		golem::Real partCloudPointSize;
		/** part3D suffix */
		std::string part3DSuffix;

		/** Part properties */
		Part3DProperty::Set partProperties;

		/** Normal standard deviation */
		golem::Real normalStdDev;
		/** Indirect point to realisation associations */
		bool partIndirect;

		/** Debugging */
		golem::U32 debug;

		/** Set to default */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			golem::data::Handler::Desc::setToDefault();

			host = "localhost";
			port = 26783;

			cloudFrameTrn = false;

			cloudAppearance.setToDefault();
			cloudSuffix = golem::Import::FILE_EXT_CLOUD_PCD;

			partColour = golem::RGBA(255, 127, 0, 255);
			partColourActive = golem::RGBA(127, 255, 0, 255);
			partColourIndirect = golem::RGBA(0, 127, 255, 255);
			partFrameShow = false;
			partFrameSize.set(golem::Real(0.025));
			partFrameSizeSelect.set(golem::Real(0.1));
			partCloudPointSize = golem::Real(3.0);
			part3DSuffix = FILE_EXT_CONTACT_PART3D;

			partProperties.setToDefault();
			normalStdDev = golem::Real(100);
			partIndirect = false;

			debug = 0;
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			golem::data::Handler::Desc::assertValid(ac);

			Assert::valid(host.length() > 0, ac, "host: empty");
			Assert::valid(port > 0, ac, "port: invalid");

			cloudAppearance.assertValid(Assert::Context(ac, "cloudAppearance."));
			Assert::valid(cloudSuffix.length() > 0, ac, "cloudSuffix: empty");

			Assert::valid(partFrameSize.isPositive(), ac, "partFrameSize: invalid");
			Assert::valid(partFrameSizeSelect.isPositive(), ac, "partFrameSizeSelect: invalid");
			Assert::valid(partCloudPointSize > golem::REAL_ZERO, ac, "partCloudPointSize: invalid");
			Assert::valid(part3DSuffix.length() > 0, ac, "part3DSuffix: invalid");

			partProperties.assertValid(Assert::Context(ac, "partProperties."));
			Assert::valid(normalStdDev > golem::REAL_EPS, ac, "normalStdDev: invalid");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual Handler::Ptr create(golem::Context &context) const;
	};

protected:
	/** Rendering */
	golem::DebugRenderer rendererCloud;
	/** Rendering */
	golem::DebugRenderer rendererParts;
	/** Debugging */
	golem::U32 debug;

	/** Hop3D server host name */
	std::string host;
	/** Hop3D server host port */
	unsigned short port;
	/** Hop3D client */
	interop::Client client;

	/** Cloud frame transformation */
	bool cloudFrameTrn;

	/** Part3D models */
	data::Part3D::IndexSetMap models;
	/** Part3D available cloud names */
	StringSeq names;
	/** Model part indices and level map */
	data::Part3D::IndexMap modelToLevelMap;
	/** Model part indices and level map */
	data::Part3D::IndexSetMap levelToModelMap;

	/** Transform interfaces */
	StringSeq transformInterfaces;

	/** Cloud appearance */
	Cloud::Appearance cloudAppearance;
	/** Cloud suffix */
	std::string cloudSuffix;

	/** Part colour */
	golem::RGBA partColour;
	/** Bottom-up part activation colour */
	golem::RGBA partColourActive;
	/** Indirect/high-level  level part colour */
	golem::RGBA partColourIndirect;
	/** Part frame size */
	bool partFrameShow;
	/** Part frame size */
	golem::Vec3 partFrameSize;
	/** Selected part frame */
	golem::Vec3 partFrameSizeSelect;
	/** Part cloud point size */
	golem::Real partCloudPointSize;
	/** part3D suffix */
	std::string part3DSuffix;

	/** Part properties */
	Part3DProperty::Set partProperties;
	/** Normal standard deviation */
	golem::Real normalStdDev;
	/** Indirect point to realisation associations */
	bool partIndirect;

	/** Cloud info request */
	bool cloudInfoRequest;
	/** Cloud load request */
	bool cloudLoadRequest;
	/** Cloud request */
	golem::I32 cloudRequest;
	/** Level request */
	golem::I32 levelRequest;
	/** Part request */
	golem::I32 partRequest;

	/** Print models info */
	void printModel() const;

	/** Process data */
	void process();

	/** Hierarchy level finder */
	void findLevel(const data::Part3D::IndexSetMap& map, data::Part3D::IndexMap& level) const;
	/** Graph index finder */
	void findIndices(const Part3D::IndexSetMap& map, data::Part3D::Index index, data::Part3D::IndexSet& incl, const data::Part3D::IndexSet* excl = nullptr) const;

	/** Creates render buffer */
	void createRender(const ItemPart3DHoP& item);
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

	/** Generate: map of items */
	virtual void generate(Item::Map& ouput);

	/** Transform: Transform input items */
	virtual Item::Ptr transform(const Item::List& input);
	/** Transform: return available interfaces */
	virtual const StringSeq& getTransformInterfaces() const;
	/** Transform: is supported by the interface */
	virtual bool isTransformSupported(const Item& item) const;

	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerPart3DHoP(golem::Context &context);
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

namespace golem {
	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::data::Part3D::IndexSetMap::value_type& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::data::Part3D::IndexSetMap::value_type& value);
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_DATA_PART_3D_HOP_PART_3D_HOP_H_*/
