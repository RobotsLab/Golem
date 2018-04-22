/** @file DepthSim.h
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
#ifndef _GOLEM_CAMERA_DEPTHSIM_DEPTHSIM_H_
#define _GOLEM_CAMERA_DEPTHSIM_DEPTHSIM_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Camera.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class CameraDepthSimDevice;

/** Depth camera simulator */
class GOLEM_LIBRARY_DECLDIR CameraDepthSim : public CameraDepth, public CameraDepthModel {
public:
	friend class golem::CameraDepthSimDevice;

	/** Camera description */
	class GOLEM_LIBRARY_DECLDIR Desc : public CameraDepth::Desc {
	public:
		/** Sensor element size */
		golem::Vec2 sensorSize;
		/** Focal length */
		golem::Real focalLength;
		/** Clipping near distance */
		golem::Real clipNear;
		/** Clipping far distance */
		golem::Real clipFar;
		/** Max surface normal inclination */
		golem::Real surfaceInclination;
		/** Epsilon */
		golem::Real epsilon;
	
		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			CameraDepth::Desc::setToDefault();
			
			sensorSize.set(golem::Real(0.03), golem::Real(0.04));
			focalLength = golem::Real(0.1);
			clipNear = golem::Real(0.2);
			clipFar = golem::Real(10.0);
			surfaceInclination = golem::Real(0.5); // cos(60deg)
			epsilon = golem::Real(1e-10);
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			CameraDepth::Desc::assertValid(ac);

			Assert::valid(sensorSize.isPositive(), ac, "sensorSize <= 0");
			Assert::valid(golem::Math::isPositiveEps(focalLength), ac, "focalLength > eps");
			Assert::valid(focalLength < clipNear, ac, "focalLength >= clipNear");
			Assert::valid(clipNear < clipFar, ac, "clipNear >= clipFar");
			Assert::valid(golem::Math::abs(surfaceInclination) <= golem::REAL_ONE, ac, "ABS(surfaceInclination) > 1");
			Assert::valid(epsilon >= golem::REAL_ZERO, ac, "epsilon < 0");
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
		/** Creates the object from the description. */
		GOLEM_CREATE_FROM_OBJECT_DESC1(CameraDepthSim, Sensor::Ptr, golem::Context&)
	};

	/** CameraDepthModel: Sets model */
	virtual void setModel(const golem::Bounds::Seq& bounds);
	
protected:
	/** Triangle vertices */
	struct TriangleVertices {
		typedef std::vector<TriangleVertices> Seq;
		golem::Vec3 t1, t2, t3;
		TriangleVertices(const golem::Vec3& t1, const golem::Vec3& t2, const golem::Vec3& t3) : t1(t1), t2(t2), t3(t3) {}
	};

	/** Sensor element size */
	golem::Vec2 sensorSize;
	/** Focal length */
	golem::Real focalLength;
	/** Clipping near distance */
	golem::Real clipNear;
	/** Clipping far distance */
	golem::Real clipFar;
	/** Max surface normal inclination */
	golem::Real surfaceInclination;
	/** Epsilon */
	golem::Real epsilon;

	mutable golem::CriticalSection csBounds;
	/** Triangle vertices */
	TriangleVertices::Seq triangleVerticesSeq;

	/** Adds model */
	void addModel(const golem::Bounds& bounds);

	/** Capture sequence start */
	virtual void start();
	/** Capture sequence stop */
	virtual void stop();

	/** Creates/initialises the Camera */
	void create(const Desc& desc);
	/** Constructs the Camera */
	CameraDepthSim(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_CAMERA_DEPTHSIM_DEPTHSIM_H_*/
