/** @file DepthSim.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sensor/CameraDepthSim/CameraDepthSim.h>
#include <Golem/Math/Triangle.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new CameraDepthSim::Desc();
}

//------------------------------------------------------------------------------


void golem::CameraDepthSim::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	CameraDepth::Desc::load(context, xmlcontext);
	XMLData(sensorSize, xmlcontext->getContextFirst("sensor"));
	XMLData("focal_length", focalLength, xmlcontext->getContextFirst("sensor"));
	XMLData("near", clipNear, xmlcontext->getContextFirst("clip"));
	XMLData("far", clipFar, xmlcontext->getContextFirst("clip"));
	XMLData("inclination", surfaceInclination, xmlcontext->getContextFirst("clip"));
	XMLData("epsilon", epsilon, xmlcontext->getContextFirst("clip"));
}

namespace golem {
	class CameraDepthSimDevice : public Camera::Device {
	public:
		typedef golem::F32 Float;
		typedef golem::_Vec3<Float> Vector;
		typedef golem::_Triangle<Float> Triangle;

		/** Triangles */
		Triangle::Seq triangles;

		CameraDepthSimDevice(Camera* camera, const Camera::Property& property, golem::I32 index) : Camera::Device(camera, property) {
		}
		void capture(Image::Ptr& pImage) {
			CameraDepthSim *camera = static_cast<CameraDepthSim*>(this->camera);

			// initialize buffer
			if (pImage == nullptr)
				pImage.reset(new Image());
			if ((pImage->cloud == nullptr || pImage->cloud->width != (uint32_t)property.width || pImage->cloud->height != (uint32_t)property.height))
				pImage->cloud.reset(new PointSeq((uint32_t)property.width, (uint32_t)property.height));
			pImage->timeStamp = camera->getContext().getTimer().elapsed();

			{
				golem::CriticalSectionWrapper csw(camera->csBounds);

				golem::Mat34 invFrame;
				invFrame.setInverse(camera->getFrame());

				triangles.clear();
				for (CameraDepthSim::TriangleVertices::Seq::const_iterator i = camera->triangleVerticesSeq.begin(); i != camera->triangleVerticesSeq.end(); ++i) {
					const Triangle triangle(invFrame * i->t1, invFrame * i->t2, invFrame * i->t3, camera->epsilon);
					if (triangle.isValid())
						triangles.push_back(triangle);
					else
						camera->context.debug("CameraDepthSimDevice(): invalid triangle\n");
				}
			}

			const Float focalLength = (Float)camera->focalLength, clipNear = (Float)camera->clipNear, clipFar = (Float)camera->clipFar, surfaceInclination = (Float)camera->surfaceInclination;
			const Float sensorSizeX = (Float)camera->sensorSize.x, sensorSizeY = (Float)camera->sensorSize.y;
			const Vector r1(numeric_const<Float>::ZERO, numeric_const<Float>::ZERO, numeric_const<Float>::ZERO);
			Vector r2(numeric_const<Float>::ZERO, numeric_const<Float>::ZERO, focalLength);

			CriticalSection cs;
			U32 i = 0;
			ParallelsTask(camera->context.getParallels(), [&] (ParallelsTask*) {
				for (U32 j = 0;;) {
					{
						CriticalSectionWrapper csw(cs);
						if (i >= U32(pImage->cloud->size())) break; else j = i++;
					}
					r2.x = sensorSizeX*(j/property.height - numeric_const<Float>::HALF*(property.width - 1))/property.width;
					r2.y = sensorSizeY*(j%property.height - numeric_const<Float>::HALF*(property.height - 1))/property.height;
					const Vector rn = r2 - r1;
					
					Vector p(numeric_const<Float>::ZERO, numeric_const<Float>::ZERO, clipFar), pp;
					Triangle::Seq::const_iterator t;
					
					for (Triangle::Seq::const_iterator tt = triangles.begin(); tt != triangles.end(); ++tt)
						if (tt->intersect(r1, rn, pp) && pp.z < p.z && pp.z > clipNear) {
							p = pp;
							t = tt;
						}

					Point& point = (*pImage->cloud)[j];
					if (p.z < clipFar && (t->tn | rn) > surfaceInclination*t->tnl*rn.magnitude()) {
						point.x = (float)p.x;
						point.y = (float)p.y;
						point.z = (float)p.z;
						point.normal_x = -(float)(t->tn.x/t->tnl); // TODO golem normals are inverse!
						point.normal_y = -(float)(t->tn.y/t->tnl);
						point.normal_z = -(float)(t->tn.z/t->tnl);
						point.r = (uint8_t)camera->colour._rgba.r;
						point.g = (uint8_t)camera->colour._rgba.g;
						point.b = (uint8_t)camera->colour._rgba.b;
						point.a = (uint8_t)camera->colour._rgba.a;
					}
					else {
						point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
						point.normal_x = point.normal_y = point.normal_z = golem::numeric_const<float>::ZERO;
						point.rgba = 0;
					}
				}
			});
		}
	};
};

CameraDepthSim::CameraDepthSim(golem::Context& context) : CameraDepth(context) {
}

void CameraDepthSim::create(const Desc& desc) {
	CameraDepth::create(desc); // throws

	colour = desc.colour;
	sensorSize = desc.sensorSize;
	focalLength = desc.focalLength;
	clipNear = desc.clipNear;
	clipFar = desc.clipFar;
	surfaceInclination = desc.surfaceInclination;
	epsilon = desc.epsilon;
}

void CameraDepthSim::addModel(const golem::Bounds& bounds) {
	const golem::BoundingConvexMesh* convexMesh = dynamic_cast<const golem::BoundingConvexMesh*>(&bounds);
	if (convexMesh) {
		const golem::TriangleMesh& mesh = convexMesh->getTriangleMesh();
		for (std::vector<golem::Triangle>::const_iterator j = mesh.triangles.begin(); j != mesh.triangles.end(); ++j)
			triangleVerticesSeq.push_back(TriangleVertices(mesh.vertices[j->t1], mesh.vertices[j->t2], mesh.vertices[j->t3]));
		return;
	}
	const golem::BoundingBox* box = dynamic_cast<const golem::BoundingBox*>(&bounds);
	if (box) {
		for (const golem::Triangle* j = box->getTriangles(); j < box->getTriangles() + 12; ++j)
			triangleVerticesSeq.push_back(TriangleVertices(box->getVertices()[j->t1], box->getVertices()[j->t2], box->getVertices()[j->t3]));
		return;
	}
}

void CameraDepthSim::setModel(const golem::Bounds::Seq& bounds) {
	golem::CriticalSectionWrapper csw(csBounds);
	triangleVerticesSeq.clear();
	for (golem::Bounds::Seq::const_iterator i = bounds.begin(); i != bounds.end(); ++i)
		addModel(**i);
}

void CameraDepthSim::start() {
	device.reset(new CameraDepthSimDevice(this, property, index)); // throws
}

void CameraDepthSim::stop() {
	device.reset();
}

//------------------------------------------------------------------------------
