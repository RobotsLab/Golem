/** @file Desc.h
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
#ifndef _GOLEM_TINYICE_DESC_H_
#define _GOLEM_TINYICE_DESC_H_

//------------------------------------------------------------------------------

#include <Golem/TinyIce/TinyIce.hh>

//------------------------------------------------------------------------------

namespace golem {
namespace tinyice {

//------------------------------------------------------------------------------

#define CLASS_INIT_IMPL(NAME) class NAME##I : public NAME {\
public:\
	NAME##I() {\
		::golem::tinyice::setToDefault(*this);\
	}\
	void setToDefault() {\
		::golem::tinyice::setToDefault(*this);\
	}\
};

#define CLASS_INIT_SIZE_IMPL(NAME) class NAME##I : public NAME {\
public:\
	NAME##I(size_t size) {\
		::golem::tinyice::setToDefault(*this, size);\
	}\
	void setToDefault(size_t size) {\
		::golem::tinyice::setToDefault(*this, size);\
	}\
};

//------------------------------------------------------------------------------

inline void setToDefault(Vec3& v) {
	v.v1 = (::Ice::Double)0.;
	v.v2 = (::Ice::Double)0.;
	v.v3 = (::Ice::Double)0.;
}
CLASS_INIT_IMPL(Vec3)

inline void setToDefault(Mat33& m) {
	m.m11 = (::Ice::Double)1.;	m.m12 = (::Ice::Double)0.;	m.m13 = (::Ice::Double)0.;
	m.m21 = (::Ice::Double)0.;	m.m22 = (::Ice::Double)1.;	m.m23 = (::Ice::Double)0.;
	m.m31 = (::Ice::Double)0.;	m.m32 = (::Ice::Double)0.;	m.m33 = (::Ice::Double)1.;
}
CLASS_INIT_IMPL(Mat33)

inline void setToDefault(Mat34& m) {
	setToDefault(m.R);
	setToDefault(m.p);
}
CLASS_INIT_IMPL(Mat34)

inline void setToDefault(Twist& t) {
	setToDefault(t.v);
	setToDefault(t.w);
}
CLASS_INIT_IMPL(Twist)

inline void setToDefault(WorkspaceCoord& c, size_t size) {
	c.c.resize(size);
	for (size_t i = 0; i < size; ++i)
		setToDefault(c.c[i]);
}
CLASS_INIT_SIZE_IMPL(WorkspaceCoord)

inline void setToDefault(GenWorkspaceState& s, size_t size) {
	setToDefault(s.pos, size);
	s.t = (::Ice::Double)0.;
}
CLASS_INIT_SIZE_IMPL(GenWorkspaceState)

inline void setToDefault(ConfigspaceCoord& c, size_t size) {
	c.c.resize(size);
	for (size_t i = 0; i < size; ++i)
		c.c[i] = (::Ice::Double)0.;
}
CLASS_INIT_SIZE_IMPL(ConfigspaceCoord)

inline void setToDefault(GenConfigspaceState& s, size_t size) {
	setToDefault(s.pos, size);
	setToDefault(s.vel, size);
	s.t = (::Ice::Double)0.;
}
CLASS_INIT_SIZE_IMPL(GenConfigspaceState)

inline void setToDefault(RGBA& c) {
	c.r = (::Ice::Double)1.;
	c.g = (::Ice::Double)1.;
	c.b = (::Ice::Double)1.;
	c.a = (::Ice::Double)1.;
}
CLASS_INIT_IMPL(RGBA)

//------------------------------------------------------------------------------

inline void setToDefault(ShapeDesc& desc) {
	setToDefault(desc.localPose);
	desc.density = (::Ice::Double)1.;
	desc.group = 1<<0;
	setToDefault(desc.color);
}
CLASS_INIT_IMPL(ShapeDesc)

inline void setToDefault(PlaneShapeDesc& desc) {
	setToDefault((ShapeDesc&)desc);
	desc.normal.v1 = (::Ice::Double)0.;
	desc.normal.v2 = (::Ice::Double)0.;
	desc.normal.v3 = (::Ice::Double)1.;
	desc.distance = (::Ice::Double)0.;
}
CLASS_INIT_IMPL(PlaneShapeDesc)

inline void setToDefault(SphereShapeDesc& desc) {
	setToDefault((ShapeDesc&)desc);
	desc.radius = (::Ice::Double)0.1;
}
CLASS_INIT_IMPL(SphereShapeDesc)

inline void setToDefault(CylinderShapeDesc& desc) {
	setToDefault((ShapeDesc&)desc);
	desc.radius = (::Ice::Double)0.1;
	desc.length = (::Ice::Double)0.1;
}
CLASS_INIT_IMPL(CylinderShapeDesc)

inline void setToDefault(BoxShapeDesc& desc) {
	setToDefault((ShapeDesc&)desc);
	desc.dimensions.v1 = (::Ice::Double)0.1;
	desc.dimensions.v2 = (::Ice::Double)0.1;
	desc.dimensions.v3 = (::Ice::Double)0.1;
}
CLASS_INIT_IMPL(BoxShapeDesc)

inline void setToDefault(ConvexMeshShapeDesc& desc) {
	setToDefault((ShapeDesc&)desc);
	desc.vertices.clear();
}
CLASS_INIT_IMPL(ConvexMeshShapeDesc)

inline void setToDefault(ActorDesc& desc) {
	setToDefault(desc.globalPose);
}
CLASS_INIT_IMPL(ActorDesc)

inline void setToDefault(RigidBodyDesc& desc) {
	setToDefault((ActorDesc&)desc);
	desc.shapes.clear();
	desc.kinematic = false;
}
CLASS_INIT_IMPL(RigidBodyDesc)

inline void setToDefault(JointDesc& desc) {
	setToDefault((RigidBodyDesc&)desc);
}
CLASS_INIT_IMPL(JointDesc)

inline void setToDefault(ControllerDesc& desc) {
	setToDefault((ActorDesc&)desc);
}
CLASS_INIT_IMPL(ControllerDesc)

//------------------------------------------------------------------------------

inline void setToDefault(KatanaSensorData& data) {
	data.index = 0;
	data.value = 0;
}
CLASS_INIT_IMPL(KatanaSensorData)

inline void setToDefault(KatanaGripperEncoderData& data) {
	data.open = 0;
	data.closed = 0;
	data.current = 0;
}
CLASS_INIT_IMPL(KatanaGripperEncoderData)

inline void setToDefault(KatanaDesc& desc) {
	setToDefault((ControllerDesc&)desc);
	desc.bGripper = false;
	desc.sensorIndexSet.clear();
	// Katana Finger Type S03.02, force sensors
	desc.sensorIndexSet.push_back(7);	// Right finger, Front
	desc.sensorIndexSet.push_back(15);	// Left finger, Front
	desc.sensorIndexSet.push_back(6);	// Right finger, Rear
	desc.sensorIndexSet.push_back(14);	// Left finger, Rear
}
CLASS_INIT_IMPL(KatanaDesc)

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

#endif /*_GOLEM_TINYICE_DESC_H_*/
