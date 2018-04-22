/** @file Feature3D.h
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
#ifndef _GOLEM_DATA_FEATURE3D_FEATURE3D_H_
#define _GOLEM_DATA_FEATURE3D_FEATURE3D_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Library.h>
#include <Golem/Plugin/Renderer.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Tools/Cloud.h>
#include <Golem/Plugin/DataI.h>
#include <Golem/Contact/Data.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

class ItemFeature3D;
class HandlerFeature3D;

/** Data item representing RGB image.
*/
class GOLEM_LIBRARY_DECLDIR ItemFeature3D : public Item, public TimeStamp, public Normal3D, public Feature3D, public Cluster3D, public Region3D, public DebugProcess {
public:
	friend class HandlerFeature3D;

	/** Feature type */
	Cloud::Feature::Type type;

	/** Point features sequence pointer */
	Cloud::PointFeatureSeqPtr cloud;

	/** Robot configuration (optional) */
	golem::ConfigMat34 config;

	/** Cloud file */
	mutable File cloudFile;
	/** Cluster file */
	mutable File clusterFile;
	/** Cluster file */
	mutable File regionFile;


	/** Magnitude of random orientation on surface = randGauss(0, PI*exp(-factor*curvature)) */
	golem::Real normalRandAng;
	/** Query distribution standard deviation */
	golem::RBDist frameStdDev;

	/** Cluster3D indices */
	Cluster3D::IndexSeq indices;
	/** Cluster3D clusters */
	Cluster3D::IndexSeq clusters;
	/** Cluster3D selection */
	Cluster3D::IndexMap selection;

	/** Region3D indices */
	Region3D::IndexSeq regIndices;
	/** Region3D clusters */
	Region3D::IndexSeq regClusters;

	/** Feature index */
	mutable golem::U32 featureIndex;

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
	/** Point3D: Point */
	virtual Point getPoint(size_t index) const;
	/** Point3D: Colour */
	virtual golem::RGBA getColour(size_t index) const;

	/** Point3D: transform */
	virtual void transform(const golem::Mat34& trn);

	/** Point3D: Sensor frame. */
	virtual void getSensorFrame(golem::Mat34& frame) const;

	/** Normal3D: Query density normal covariance */
	virtual Point3D::Real getNormalCovariance() const;
	/** Normal3D: Select normal given point index. */
	virtual Point3D::Vec3 getNormal(size_t index) const;

	/** Feature3D: Select feature given point index. Do not sample orientation - return only mean value to use later with sensor model.  */
	virtual void getFeature(size_t index, Feature& feature, Mat33& orientation) const;
	/** Feature3D: Sample local frame orientations from sensor model given mean value. */
	virtual void sampleSensorModel(golem::Rand& rand, size_t index, Mat33& orientation) const;

	/** Feature3D: Query density frame covariance */
	virtual RBDist getFrameCovariance() const;

	/** Cluster3D: Indices sorted along clusters, clusters indices pointers. */
	virtual void getClusters(Cluster3D::IndexSeq& indices, Cluster3D::IndexSeq& clusters) const;
	/** Cluster3D: Cluster selection. */
	virtual void getClustersSelection(Cluster3D::IndexMap& selection) const;

	/** Regions: Indices sorted along clusters, clusters indices pointers. */
	virtual void getRegions(Region3D::IndexSeq& indices, Region3D::IndexSeq& clusters) const;

	/** DebugProcess: Debug reset */
	virtual void debugReset() const;
	/** DebugProcess: Debug string */
	virtual void debugString(std::string& str) const;

	/** Process data */
	void process();

protected:
	/** Data handler */
	HandlerFeature3D& handler;

	/** Samples */
	mutable golem::U32 samplesCount;
	/** Rotation angle */
	mutable golem::Real curvRandMag;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	/** Initialise data item */
	ItemFeature3D(HandlerFeature3D& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerFeature3D : public Handler, public UI, public Transform {
public:
	friend class ItemFeature3D;

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Handler::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Debug level */
		golem::U32 debugLevel;

		/** Feature type */
		Cloud::Feature::Type type;

		/** Cloud appearance */
		Cloud::Appearance cloudAppearance;
		/** Cloud suffix */
		std::string cloudSuffix;

		/** Cloud description */
		Cloud::Desc cloudDesc;

		/** Cloud transformation */
		Mat34Map cloudTrnMap;

		/** Magnitude of random orientation on surface = randGauss(0, PI*exp(-factor*curvature)) */
		golem::Real normalRandAng;
		/** Frame standard deviation */
		golem::RBDist frameStdDev;

		/** Cluster suffix */
		std::string clusterSuffix;
		/** Cluster names */
		StringSet clusterNames;

		/** Region suffix */
		std::string regionSuffix;

		/** Set to default */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			debugLevel = 0;

			golem::data::Handler::Desc::setToDefault();

			type = Cloud::Feature::TYPE_DEFAULT;

			cloudAppearance.setToDefault();
			cloudSuffix = golem::Import::FILE_EXT_CLOUD_PCD;
			cloudDesc.setToDefault();

			cloudTrnMap.clear();

			normalRandAng = golem::Real(100.0);
			frameStdDev.set(golem::Real(0.005), golem::Real(200.0));

			clusterSuffix = golem::data::Cluster3D::FILE_EXT_CLUSTER;
			clusterNames.clear();
			clusterNames.insert(Cluster3D::CLUSTER_PROCESSING);

			regionSuffix = golem::data::Region3D::FILE_EXT_REGION;
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			golem::data::Handler::Desc::assertValid(ac);

			cloudAppearance.assertValid(Assert::Context(ac, "cloudAppearance."));
			Assert::valid(cloudSuffix.length() > 0, ac, "cloudSuffix: empty");

			cloudDesc.assertValid(Assert::Context(ac, "cloudDesc."));

			for (Mat34Map::const_iterator i = cloudTrnMap.begin(); i != cloudTrnMap.end(); ++i)
				Assert::valid(!i->first.empty() && i->second.isFinite(), ac, "cloudTrnMap[]: invalid");

			Assert::valid(normalRandAng >= golem::REAL_ZERO, ac, "normalRandAng: < 0");
			Assert::valid(frameStdDev.isValid(), ac, "frameStdDev: invalid");

			Assert::valid(clusterSuffix.length() > 0, ac, "clusterSuffix: empty");
			Assert::valid(regionSuffix.length() > 0, ac, "regionSuffix: empty");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual Handler::Ptr create(golem::Context &context) const;
	};

	/** Feature type */
	Cloud::Feature::Type getType() const {
		return type;
	}

	/** Cloud appearance */
	const Cloud::Appearance& getCloudAppearance() const {
		return cloudAppearance;
	}

protected:
	/** Debug level */
	golem::U32 debugLevel;

	/** Point cloud rendering */
	golem::DebugRenderer cloudRenderer;
	/** Point cloud rendering */
	golem::DebugRenderer clusterRenderer;

	/** Cloud appearance */
	Cloud::Appearance cloudAppearance;
	/** Cloud suffix */
	std::string cloudSuffix;

	/** Cloud description */
	Cloud::Desc cloudDesc;
	/** Region */
	golem::Bounds::Seq region;

	/** Cloud transformation */
	Mat34Map cloudTrnMap;

	/** Feature type */
	Cloud::Feature::Type type;

	/** Mode request */
	bool modeRequest;
	/** Feature index request */
	golem::I32 featureIndexRequest;

	/** Magnitude of random orientation on surface = randGauss(0, PI*exp(-factor*curvature)) */
	golem::Real normalRandAng;
	/** Query distribution standard deviation */
	golem::RBDist frameStdDev;

	/** Cluster suffix */
	std::string clusterSuffix;
	/** Cluster names */
	StringSet clusterNames;

	/** Region suffix */
	std::string regionSuffix;

	/** Transform interfaces */
	StringSeq transformInterfaces;

	/** Cluster set selection set */
	golem::U32 clusterSetSelection;
	/** Cluster selection index */
	golem::U32 clusterIdxSelection;
	/** Cluster selection index */
	bool clusterSelection, clusterSelectionAddRemove;
	/** Cluster selection coords */
	int clusterX, clusterY;

	/** Enables/disables drawing regions */
	bool showRegions;

	/** Creates render buffer */
	void createRender(const ItemFeature3D& image);
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

	template <typename _Idx, typename _Ptr, typename _Seq> static void selectIndex(_Idx& idx, _Ptr& ptr, _Seq& seq) {
		_Idx i = 0;
		for (; !seq.empty() && i < std::min(idx, (_Idx)seq.size() - 1); ++i, ++ptr);
		idx = i;
	}

	/** Mode string */
	static std::string toString(golem::U32 mode, golem::U32 type, golem::U32 index);

	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerFeature3D(golem::Context &context);
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_DATA_FEATURE3D_FEATURE3D_H_*/
