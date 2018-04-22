/** @file Image.h
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
#ifndef _GOLEM_DATA_IMAGE_IMAGE_H_
#define _GOLEM_DATA_IMAGE_IMAGE_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Library.h>
#include <Golem/Plugin/Renderer.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Tools/CameraCalibration.h>
#include <Golem/Tools/Cloud.h>
#include <Golem/Tools/RBPose.h>
#include <Golem/Tools/Import.h>
#include <Golem/Plugin/Point.h>
#include <Golem/Plugin/DataI.h>
#include <Golem/Plugin/SensorI.h>
#include <Golem/Plugin/PlannerI.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

class ItemImage;
class HandlerImage;

/** Data item representing RGB image.
*/
class GOLEM_LIBRARY_DECLDIR ItemImage : public Item, public golem::Image, public Model, public Normal3D {
public:
	friend class HandlerImage;

	/** Camera parameters */
	golem::CameraCalibration::Parameters parameters;
	/** Robot configuration (optional) */
	golem::ConfigMat34 config;

	/** Image file */
	mutable File imageFile;
	/** Cloud file */
	mutable File cloudFile;

	/** Normal standard deviation */
	golem::Real normalStdDev;

	/** Clones item. */
	virtual Item::Ptr clone() const;

	/** Creates render buffer, the buffer can be shared and allocated on Handler */
	virtual void createRender();

	/** Model: Estimates model pose. */
	virtual void model(golem::ConfigMat34& config, golem::Mat34& frame, Vec3Seq* vertices = nullptr, TriangleSeq* triangles = nullptr);

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

protected:
	/** Data handler */
	HandlerImage& handler;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	/** Initialise data item */
	ItemImage(HandlerImage& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerImage : public Handler, public UI, public Import, public Transform, public Capture {
public:
	friend class ItemImage;

	/** Image appearance */
	class GOLEM_LIBRARY_DECLDIR ImageAppearance {
	public:
		/** Show image */
		bool show;
		/** Show image with point cloud */
		bool showWithCloud;
		/** Show image camera frame */
		bool showCameraFrame;

		/** Constructs description. */
		ImageAppearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			show = true;
			showWithCloud = false;
			showCameraFrame = true;
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
		}
		/** Reads/writes object from/to a given XML context */
		void xmlData(golem::XMLContext* context, bool create = false) const;
	};

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Handler::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Cloud appearance */
		Cloud::Appearance cloudAppearance;
		/** Import settings */
		Cloud::Import cloudImport;
		/** Import cloud generic extension */
		std::string cloudImportExt;
		/** Cloud suffix */
		std::string cloudSuffix;

		/** Cloud description */
		Cloud::Desc cloudDesc;

		/** Capture region in global coordinates */
		golem::Bounds::Desc::Seq regionCaptureDesc;

		/** Image appearance */
		ImageAppearance imageAppearance;
		/** Image suffix */
		std::string imageSuffix;
		/** Image format */
		std::string imageFormat;

		/** Model estimator description */
		RBPose::Desc::Ptr rbPoseDesc;
		/** Model file */
		std::string modelPath;

		/** Normal standard deviation */
		golem::Real normalStdDev;

		/** Set to default */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			golem::data::Handler::Desc::setToDefault();

			cloudAppearance.setToDefault();
			cloudImport.setToDefault();
			cloudImportExt = ".txt";
			cloudSuffix = golem::Import::FILE_EXT_CLOUD_PCD;

			cloudDesc.setToDefault();

			regionCaptureDesc.clear();

			imageAppearance.setToDefault();
			imageSuffix = ".png";
			imageFormat = "16 3"; // CV_IMWRITE_PNG_COMPRESSION=16 0...9

			rbPoseDesc.reset(new RBPose::Desc);
			modelPath.clear();

			normalStdDev = golem::Real(100);
		}
		
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			golem::data::Handler::Desc::assertValid(ac);

			cloudAppearance.assertValid(Assert::Context(ac, "cloudAppearance."));
			cloudImport.assertValid(Assert::Context(ac, "import."));
			Assert::valid(cloudImportExt.length() > 0, ac, "cloudImportExt: empty");
			Assert::valid(cloudSuffix.length() > 0, ac, "cloudSuffix: empty");

			cloudDesc.assertValid(Assert::Context(ac, "cloudDesc."));

			imageAppearance.assertValid(Assert::Context(ac, "imageAppearance."));
			Assert::valid(imageSuffix.length() > 0, ac, "imageSuffix: empty");
			Assert::valid(imageFormat.length() > 0, ac, "imageFormat: empty");

			for (golem::Bounds::Desc::Seq::const_iterator i = regionCaptureDesc.begin(); i != regionCaptureDesc.end(); ++i)
				Assert::valid((*i)->isValid(), ac, "regionCaptureDesc[]: invalid");

			Assert::valid(rbPoseDesc != nullptr, ac, "rbPoseDesc: null");
			rbPoseDesc->assertValid(Assert::Context(ac, "rbPoseDesc->"));

			Assert::valid(normalStdDev > golem::REAL_EPS, ac, "normalStdDev: invalid");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual Handler::Ptr create(golem::Context &context) const;
	};

	/** Cloud appearance */
	const Cloud::Appearance& getCloudAppearance() const {
		return cloudAppearance;
	}

protected:
	/** Image cache */
	golem::ImageData image;
	/** Image texture id */
	mutable unsigned imageID;
	/** Point cloud rendering */
	golem::DebugRenderer cloudRenderer;
	/** Model rendering */
	golem::DebugRenderer modelRenderer;

	/** Cloud appearance */
	Cloud::Appearance cloudAppearance;
	/** Import settings */
	Cloud::Import cloudImport;
	/** Import cloud generic extension */
	std::string cloudImportExt;
	/** Cloud suffix */
	std::string cloudSuffix;

	/** Cloud description */
	Cloud::Desc cloudDesc;
	/** Region */
	golem::Bounds::Seq region;
	/** Region capture */
	golem::Bounds::Seq regionCapture;

	/** Image appearance */
	ImageAppearance imageAppearance;
	/** Image suffix */
	std::string imageSuffix;
	/** Image format */
	IntSeq imageFormat;

	/** Import types */
	StringSeq importTypes;
	/** Transform interfaces */
	StringSeq transformInterfaces;

	/** Model estimator */
	RBPose::Ptr rbPose;
	/** Model file */
	std::string modelPath;
	/** Model cloud */
	PointSeq modelCloud;

	/** Normal standard deviation */
	golem::Real normalStdDev;

	/** Creates render buffer */
	void createRender(const ItemImage& image);
	/** golem::UIRenderer: Render on output device. */
	virtual void render() const;
	/** golem::UIRenderer: Render on output device. */
	virtual void customRender() const;

	/** golem::UIKeyboardMouse: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	/** golem::UIKeyboardMouse: Mouse motion handler. */
	virtual void motionHandler(int x, int y);
	/** golem::UIKeyboardMouse: Keyboard handler. */
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

	/** Capture: Captures snapshot or sequence from sensor. */
	virtual Item::Ptr capture(golem::Sensor& sensor, Control control);

	/** Model: Estimates model pose. */
	virtual void model(const ItemImage& image, golem::ConfigMat34& config, golem::Mat34& frame, Vec3Seq* vertices = nullptr, TriangleSeq* triangles = nullptr);

	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerImage(golem::Context &context);
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_DATA_IMAGE_IMAGE_H_*/
