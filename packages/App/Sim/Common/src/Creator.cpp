/** @file Creator.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/App/Common/Creator.h>
#include <iomanip>
#include <sstream>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Creator::Creator(Scene &scene) :
	scene(scene),
	context(scene.getContext()),
	rand(context.getRandSeed())
{
	Creator::setToDefault();
}

Creator::~Creator() {
}

//------------------------------------------------------------------------------

Actor::Desc::Ptr Creator::createGroundPlaneDesc() {
	// Create a ground plane.
	Actor::Desc::Ptr pActorDesc = scene.createActorDesc();
	pActorDesc->boundsDescSeq.push_back(Bounds::Desc::Ptr(new BoundingPlane::Desc()));
	return pActorDesc;
}

Actor::Desc::Ptr Creator::createBoxDesc(Real x, Real y, Real z) {
	Actor::Desc::Ptr pActorDesc = scene.createActorDesc();
	
	pActorDesc->boundsDescSeq.push_back(Bounds::Desc::Ptr(new BoundingBox::Desc()));
	BoundingBox::Desc& boxDesc = static_cast<BoundingBox::Desc&>(*pActorDesc->boundsDescSeq.back());
	boxDesc.dimensions.set(x, y, z);
	if (mode == MODE_GROUNDPLANE) {
		//boxDesc.pose.p.v3 = z;
	}
	if (mode == MODE_GROUNDPLANE) {
		pActorDesc->pose.p.z = z;
	}
	
	return pActorDesc;
}

Actor::Desc::Ptr Creator::createTreeDesc(Real radius, Real thickness) {
	Actor::Desc::Ptr pActorDesc = scene.createActorDesc();
	
	pActorDesc->boundsDescSeq.push_back(Bounds::Desc::Ptr(new BoundingSphere::Desc()));
	BoundingSphere::Desc& sphereDesc = static_cast<BoundingSphere::Desc&>(*pActorDesc->boundsDescSeq.back());
	sphereDesc.radius = radius;
		
	pActorDesc->boundsDescSeq.push_back(Bounds::Desc::Ptr(new BoundingBox::Desc()));
	BoundingBox::Desc& boxDesc = static_cast<BoundingBox::Desc&>(*pActorDesc->boundsDescSeq.back());
	const Real size = thickness*radius;
	boxDesc.dimensions.set(size, size, size);
	boxDesc.pose.p.set((Real)0.0, (Real)0.0, - radius - size);

	if (mode == MODE_GROUNDPLANE) {
		pActorDesc->pose.p.z += radius + REAL_TWO*size;
	}

	return pActorDesc;
}

Actor::Desc::Ptr Creator::createSimple2FlapDesc(Real width, Real height, Real length, Real thickness, Real angle) {
	return create2FlapDesc(width, width, Real(0.0), Real(0.0), height, length, thickness, angle, false);
}

Actor::Desc::Ptr Creator::create2FlapDesc(Real width1, Real width2, Real shift1, Real shift2, Real height, Real length, Real thickness, Real angle, bool moveFrame) {
	Actor::Desc::Ptr pActorDesc = scene.createActorDesc();

	pActorDesc->boundsDescSeq.push_back(Bounds::Desc::Ptr(new BoundingBox::Desc()));
	BoundingBox::Desc& boxYDesc = static_cast<BoundingBox::Desc&>(*pActorDesc->boundsDescSeq.back());
	// Y-up flap
	boxYDesc.dimensions.set(width1, length, thickness);
	if (moveFrame) {
		boxYDesc.pose.p.set(shift1, length + shift2, thickness);
		boxYDesc.pose.R.setId();
//		boxYDesc.pose.R.rotX(angle);
	}
	else {
		boxYDesc.pose.p.set(REAL_ZERO, length, thickness);
		boxYDesc.pose.R.setId();
	}
	
	pActorDesc->boundsDescSeq.push_back(Bounds::Desc::Ptr(new BoundingBox::Desc()));
	BoundingBox::Desc& boxZDesc = static_cast<BoundingBox::Desc&>(*pActorDesc->boundsDescSeq.back());
	// Z-up flap
	Real sin, cos;
	Math::sinCos(angle, sin, cos);
	boxZDesc.dimensions.set(width2, height, thickness);
	if (moveFrame) {
		boxZDesc.pose.p.set(REAL_ZERO, cos*height, sin*height + thickness);
//		boxZDesc.pose.R.setId();
		boxZDesc.pose.R.rotX(angle);
	}
	else {
		boxZDesc.pose.p.set(shift1, cos*height + shift2, sin*height + thickness);
		boxZDesc.pose.R.rotX(angle);
	}

	return pActorDesc;
}

//------------------------------------------------------------------------------
