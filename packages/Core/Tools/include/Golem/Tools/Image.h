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
#ifndef _GOLEM_TOOLS_IMAGE_H_
#define _GOLEM_TOOLS_IMAGE_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Defs.h>
#include <opencv2/core/types_c.h>
#include <boost/shared_ptr.hpp>
#include <list>

//------------------------------------------------------------------------------

/** PCL format of point clouds */
namespace pcl {
	/** Depth camera point/voxel */
	struct PointXYZRGBNormal;
	/** PCL Point cloud */
	template <typename PointT> class PointCloud;
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** 3D Point (compatible with Cloud::Point) */
typedef pcl::PointXYZRGBNormal Point;
/** 3D Point cloud (compatible with Cloud::PointSeq) */
typedef pcl::PointCloud<Point> PointSeq;
/** 3D Point cloud pointer (compatible with Cloud::PointSeqPtr) */
typedef boost::shared_ptr<PointSeq> PointSeqPtr;

//------------------------------------------------------------------------------

/** 2D image. TODO replace IplImage with OpenCV Mat */
class ImageData {
public:
	/** 2D image */
	IplImage* image;

	/** No data allocation */
	ImageData();
	/** Copies image */
	ImageData(const ImageData& image, int code = -1);
	/** Copies image */
	ImageData(const IplImage* image, int code = -1);
	/** Reserves image data */
	ImageData(int width, int height, int depth = IPL_DEPTH_8U, int channels = 3);

	/** Releases data */
	~ImageData();

	/** Sets image */
	void set(const ImageData& image, int code = -1);
	/** Sets image */
	void set(const IplImage* image, int code = -1);

	/** Reserves data */
	void reserve(const IplImage* image);
	/** Reserves data */
	void reserve(int width, int height, int depth = IPL_DEPTH_8U, int channels = 3);

	/** Release data */
	void release();

	/** OpenGL draw Image */
	void draw(unsigned* imageID) const;
};

//------------------------------------------------------------------------------

/** 3D point cloud */
class CloudData {
public:
	/** 3D point cloud */
	PointSeqPtr cloud;

	/** Default allocation */
	CloudData();
	/** Resize */
	void resize(int width, int height);
};

//------------------------------------------------------------------------------

/** 2D image + 3D point cloud + time stamp. */
class Image : public ImageData, public CloudData, public TimeStamp {
public:
	typedef golem::shared_ptr<Image> Ptr;
	typedef std::list<Ptr> List;

	/** Capture index */
	golem::U32 captureIndex;

	/** No data allocation */
	Image(golem::SecTmReal timeStamp = golem::SEC_TM_REAL_ZERO, golem::U32 captureIndex = -1);
	/** Copies image */
	Image(const Image& image, int code = -1);
	/** Copies image */
	Image(const IplImage* image, int code = -1);
	/** Reserves image data */
	Image(int width, int height, int depth = IPL_DEPTH_8U, int channels = 3);

	/** Sets image */
	void set(const Image& image, int code = -1);

	/** Assert image data */
	template <typename _Ptr> static void assertData(const _Ptr& ptr) {
		if (ptr == nullptr)
			throw golem::Message(golem::Message::LEVEL_ERROR, "Image::assertData(): image data not available");
	}
};

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_TOOLS_IMAGE_H_*/
