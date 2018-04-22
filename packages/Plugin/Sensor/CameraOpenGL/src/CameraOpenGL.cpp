/** @file OpenGL.cpp
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sensor/CameraOpenGL/CameraOpenGL.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <opencv2/opencv.hpp>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new CameraOpenGL::Desc();
}

//------------------------------------------------------------------------------

namespace golem {
	class CameraOpenGLDevice : public Camera::Device {
	public:
		CameraOpenGLDevice(Camera* camera, const Camera::Property& property) : Camera::Device(camera, property) {
			this->property = property;
		}
		void capture(Image::Ptr& pImage) {
			CameraOpenGL *camera = static_cast<CameraOpenGL*>(this->camera);
			ScopeGuard guard([&] () { camera->imageQuery = false; });
			camera->imageQuery = true;

			camera->evImage.wait();
			camera->evImage.set(false);
			if (!camera->image.image)
				return;

			if (pImage == nullptr)
				pImage.reset(new Image());

			{
				CriticalSectionWrapper csw(camera->csImage);
				// copy and flip
				pImage->reserve(camera->image.image);
				::cvFlip(camera->image.image, pImage->image);
				//const size_t height = (size_t)pImage->image->height;
				//const size_t width = (size_t)pImage->image->width;
				//const U8* srcBegin = (const U8*)camera->image.image->imageData;
				//U8* dstBegin = (U8*)pImage->image->imageData;
				//for (size_t y = 0; y < height; ++y)
				//	for (size_t x = 0; x < width; ++x) {
				//		const U8* src = srcBegin + 3*width*y + 3*x;
				//		U8* dst = dstBegin + 3*width*y + 3*x;
				//		dst[0] = src[0];
				//		dst[1] = src[1];
				//		dst[2] = src[2];
				//	}
				
				// time stamp
				pImage->timeStamp = camera->timeStamp.timeStamp;
			}
		}
	};
};

CameraOpenGL::CameraOpenGL(golem::Context& context) : Camera(context), imageQuery(false) {
}

CameraOpenGL::~CameraOpenGL() {
	CriticalSectionWrapper csw(csImage);
	image.release();
	evImage.set(true);
}

void CameraOpenGL::create(const Desc& desc) {
	desc.assertValid(Assert::Context("CameraOpenGL::Desc."));

	device.reset(new CameraOpenGLDevice(this, desc.properties[0])); // throws
	property = device->getProperty();

	device.reset();

	Camera::create(desc); // throws
}

void CameraOpenGL::start() {
	device.reset(new CameraOpenGLDevice(this, property)); // throws
}

void CameraOpenGL::stop() {
	device.reset();
}

//------------------------------------------------------------------------------

void CameraOpenGL::capture(int x, int y, int width, int height) {
	if (!imageQuery)
		return;

	CriticalSectionWrapper csw(csImage);
	image.reserve(width, height);
	
	::glFlush();
	::glFinish();
	::glReadPixels(x, y, width, height, GL_BGR_EXT, GL_UNSIGNED_BYTE, image.image->imageData);

	timeStamp.timeStamp = context.getTimer().elapsed();
	evImage.set(true);
}

//------------------------------------------------------------------------------
