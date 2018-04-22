/** @file ImageData.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Tools/Image.h>
#include <opencv2/opencv.hpp>
#include <GL/gl.h>
#include <GL/glut.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

ImageData::ImageData() : image(nullptr) {
}

ImageData::~ImageData() {
	release();
}

ImageData::ImageData(const ImageData& image, int code) : image(nullptr) {
	set(image, code);
}

ImageData::ImageData(const IplImage* data, int code) : image(nullptr) {
	set(data, code);
}

ImageData::ImageData(int width, int height, int depth, int channels) : image(nullptr) {
	reserve(width, height, depth, channels);
}

void ImageData::set(const ImageData& image, int code) {
	set(image.image, code);
}

void ImageData::set(const IplImage* image, int code) {
	reserve(image);
	if (image != nullptr)
		code < 0 ? cvCopy(image, this->image) : cvCvtColor(image, this->image, code);
}

void ImageData::reserve(const IplImage* image) {
	if (image != nullptr)
		reserve(image->width, image->height, image->depth, image->nChannels);
}

void ImageData::reserve(int width, int height, int depth, int channels) {
	if (image == nullptr || image->width != width || image->height != height || image->depth != depth || image->nChannels != channels) {
		release();
		image = cvCreateImage(cvSize(width, height), depth, channels);
	}
}

void ImageData::release() {
	if (image != nullptr) {
		cvReleaseImage(&image);
		image = nullptr;
	}
}

void ImageData::draw(unsigned* imageID) const {
	if (image == nullptr || imageID == nullptr)
		return;

	// Set Projection Matrix
	::glMatrixMode(GL_PROJECTION);
	::glLoadIdentity();
	::gluOrtho2D(0, image->width, image->height, 0);
	// Switch to Model View Matrix
	::glMatrixMode(GL_MODELVIEW);
	::glLoadIdentity();

	// Create OpenGL texture
	if (*imageID == 0) {
		::glGenTextures(1, imageID);
		ASSERT(*imageID != 0)

		::glBindTexture(GL_TEXTURE_2D, *imageID);
		::glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image->width, image->height, 0, GL_RGB, GL_UNSIGNED_BYTE, image->imageData);
	}
	else {
		::glBindTexture(GL_TEXTURE_2D, *imageID);
		::glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image->width, image->height, GL_RGB, GL_UNSIGNED_BYTE, image->imageData);
	}

	const GLboolean glDepthTest = ::glIsEnabled(GL_DEPTH_TEST); ::glDisable(GL_DEPTH_TEST);
	const GLboolean glCullFace = ::glIsEnabled(GL_CULL_FACE); ::glDisable(GL_CULL_FACE);
	const GLboolean glLighting = ::glIsEnabled(GL_LIGHTING); ::glDisable(GL_LIGHTING);
	const GLboolean glBlend = ::glIsEnabled(GL_BLEND); ::glDisable(GL_BLEND);
	const GLboolean glTexture2D = ::glIsEnabled(GL_TEXTURE_2D); ::glEnable(GL_TEXTURE_2D);

	::glColor4ub(255, 255, 255, 255);
	::glBegin(GL_QUADS);
	::glTexCoord2f(0.0f, 0.0f);
	::glVertex2f(0.0f, 0.0f);
	::glTexCoord2f(1.0f, 0.0f);
	::glVertex2f((float)image->width, 0.0f);
	::glTexCoord2f(1.0f, 1.0f);
	::glVertex2f((float)image->width, (float)image->height);
	::glTexCoord2f(0.0f, 1.0f);
	::glVertex2f(0.0f, (float)image->height);
	::glEnd();
	::glFlush();

	if (glDepthTest) ::glEnable(GL_DEPTH_TEST);
	if (glCullFace) ::glEnable(GL_CULL_FACE);
	if (glLighting) ::glEnable(GL_LIGHTING);
	if (glBlend) ::glEnable(GL_BLEND);
	if (!glTexture2D) ::glDisable(GL_TEXTURE_2D);
}


//------------------------------------------------------------------------------

CloudData::CloudData() {
	cloud.reset(new PointSeq());
}

void CloudData::resize(int width, int height) {
	//cloud.reset(new PointSeq((uint32_t)property.width, (uint32_t)property.height));
	cloud->resize(width * height);
	cloud->width = (uint32_t)width;
	cloud->height = (uint32_t)height;
	cloud->is_dense = true;
	cloud->sensor_origin_ = Eigen::Vector4f::Zero();
	cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
}

//------------------------------------------------------------------------------

Image::Image(golem::SecTmReal timeStamp, golem::U32 captureIndex) : TimeStamp(timeStamp), captureIndex(captureIndex) {
}

Image::Image(const Image& image, int code) {
	set(image, code);
}

Image::Image(const IplImage* image, int code) : ImageData(image, code) {
}

Image::Image(int width, int height, int depth, int channels) : ImageData(width, height, depth, channels) {
}

void Image::set(const Image& image, int code) {
	ImageData::set(image, code);
	*cloud = *image.cloud;
	timeStamp = image.timeStamp;
	captureIndex = image.captureIndex;
}

//------------------------------------------------------------------------------
