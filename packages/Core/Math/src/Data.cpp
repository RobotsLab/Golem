/** @file Data.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(Vec2 &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData("v1", val.v1, context, create);
	XMLData("v2", val.v2, context, create);
}

void golem::XMLData(Vec3 &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData("v1", val.v1, context, create);
	XMLData("v2", val.v2, context, create);
	XMLData("v3", val.v3, context, create);
}

void golem::XMLData(Quat &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData("q0", val.q0, context, create);
	XMLData("q1", val.q1, context, create);
	XMLData("q2", val.q2, context, create);
	XMLData("q3", val.q3, context, create);
}

void golem::XMLData(Twist &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData("v1", val.v.v1, context, create);
	XMLData("v2", val.v.v2, context, create);
	XMLData("v3", val.v.v3, context, create);
	XMLData("w1", val.w.v1, context, create);
	XMLData("w2", val.w.v2, context, create);
	XMLData("w3", val.w.v3, context, create);
}

void golem::XMLData(Twist &val, Real& theta, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData(val, context, create);
	XMLData("th", theta, context, create);
}

void golem::XMLData(Mat22 &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData("m11", val.m11, context, create);
	XMLData("m12", val.m12, context, create);
	XMLData("m21", val.m21, context, create);
	XMLData("m22", val.m22, context, create);
}

void golem::XMLData(Mat33 &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData("m11", val.m11, context, create);
	XMLData("m12", val.m12, context, create);
	XMLData("m13", val.m13, context, create);
	XMLData("m21", val.m21, context, create);
	XMLData("m22", val.m22, context, create);
	XMLData("m23", val.m23, context, create);
	XMLData("m31", val.m31, context, create);
	XMLData("m32", val.m32, context, create);
	XMLData("m33", val.m33, context, create);
}

void golem::XMLDataAngleAxis(Real &angle, Vec3 &axis, XMLContext* context, bool create) {
	XMLData("angle", angle, context, create);
	XMLData("a1", axis.v1, context, create);
	XMLData("a2", axis.v2, context, create);
	XMLData("a3", axis.v3, context, create);
}

void golem::XMLDataEuler(Real& roll, Real& pitch, Real& yaw, XMLContext* context, bool create) {
	XMLData("roll", roll, context, create);
	XMLData("pitch", pitch, context, create);
	XMLData("yaw", yaw, context, create);
}

void golem::XMLData(Mat23 &val, XMLContext* context, bool create) {
	ASSERT(context)
	
	try {
		// Mat22
		XMLData(val.R, context, create);
		goto END;
	}
	catch (const MsgXMLParserAttributeNotFound& ex) {
		if (create)
			throw ex;
	}
	try {
		// angle
		Real angle;
		XMLData("a", angle, context, create);
		val.R.fromAngle(angle);
		goto END;
	}
	catch (const MsgXMLParserAttributeNotFound&) {}

	throw MsgXMLParserIncompleteData(Message::LEVEL_ERROR, "XMLData(): Unable to load Mat22: %s", context->getName().c_str());

	END:
	XMLData(val.p, context, create);
}

void golem::XMLData(Mat34 &val, XMLContext* context, bool create) {
	ASSERT(context)
	
	try {
		// Mat33
		XMLData(val.R, context, create);
		goto END;
	}
	catch (const MsgXMLParserAttributeNotFound& ex) {
		if (create)
			throw ex;
	}
	try {
		// Quat
		Quat quat;
		XMLData(quat, context, create);
		val.R.fromQuat(quat);
		goto END;
	}
	catch (const MsgXMLParserAttributeNotFound&) {}
	try {
		// Twist
		Twist twist;
		Real theta;
		XMLData(twist, theta, context, create);
		val.fromTwist(twist, theta);
		return;
	}
	catch (const MsgXMLParserAttributeNotFound&) {}
	try {
		// Angle-axis
		Real angle;
		Vec3 axis;
		XMLDataAngleAxis(angle, axis, context, create);
		val.R.fromAngleAxis(angle, axis);
		goto END;
	}
	catch (const MsgXMLParserAttributeNotFound&) {}
	try {
		// Euler
		Real roll, pitch, yaw;
		XMLDataEuler(roll, pitch, yaw, context, create);
		val.R.fromEuler(roll, pitch, yaw);
		goto END;
	}
	catch (const MsgXMLParserAttributeNotFound&) {}

	throw MsgXMLParserIncompleteData(Message::LEVEL_ERROR, "XMLData(): Unable to load Mat33: %s", context->getName().c_str());

	END:
	XMLData(val.p, context, create);
}

//------------------------------------------------------------------------------

void golem::XMLData(RBCoord& val, golem::XMLContext* context, bool create) {
	golem::XMLData(val.p, context, create);
	golem::XMLData(val.q, context, create);
}

void golem::XMLData(RBDist& val, golem::XMLContext* context, bool create) {
	golem::XMLData("lin", val.lin, context, create);
	golem::XMLData("ang", val.ang, context, create);
}

void golem::XMLData(RBDistEx& val, golem::XMLContext* context, bool create) {
	golem::XMLData((RBDist&)val, context, create);
	golem::XMLData("collision", val.collision, context, create);
}

//------------------------------------------------------------------------------

void golem::XMLData(Triangle &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData("t1", val.t1, context, create);
	XMLData("t2", val.t2, context, create);
	XMLData("t3", val.t3, context, create);
}

//------------------------------------------------------------------------------

namespace golem 
{

template <> void Stream::read(Bounds::Desc& desc) const {
	*this >> desc.pose >> desc.group;
}

template <> void Stream::write(const Bounds::Desc& desc) {
	*this << desc.pose << desc.group;
}

template <> void Stream::read(BoundingPlane::Desc& desc) const {
	*this >> (Bounds::Desc&)desc >> desc.normal >> desc.distance >> desc.gridSize >> desc.gridDelta;
}

template <> void Stream::write(const BoundingPlane::Desc& desc) {
	*this << (const Bounds::Desc&)desc << desc.normal << desc.distance << desc.gridSize << desc.gridDelta;
}

template <> void Stream::read(BoundingSphere::Desc& desc) const {
	*this >> (Bounds::Desc&)desc >> desc.radius >> desc.numOfSlices >> desc.numOfStacks;
}

template <> void Stream::write(const BoundingSphere::Desc& desc) {
	*this << (const Bounds::Desc&)desc << desc.radius << desc.numOfSlices << desc.numOfStacks;
}

template <> void Stream::read(BoundingCylinder::Desc& desc) const {
	*this >> (Bounds::Desc&)desc >> desc.radius >> desc.length >> desc.numOfSlices;
}

template <> void Stream::write(const BoundingCylinder::Desc& desc) {
	*this << (Bounds::Desc&)desc << desc.radius << desc.length << desc.numOfSlices;
}

template <> void Stream::read(BoundingBox::Desc& desc) const {
	*this >> (Bounds::Desc&)desc >> desc.dimensions;
}

template <> void Stream::write(const BoundingBox::Desc& desc) {
	*this << (Bounds::Desc&)desc << desc.dimensions;
}

template <> void Stream::read(BoundingConvexMesh::Desc& desc) const {
	*this >> (Bounds::Desc&)desc;
	*this >> desc.bCook;
	desc.vertices.clear();
	read(desc.vertices, desc.vertices.begin());
	desc.triangles.clear();
	read(desc.triangles, desc.triangles.begin());
}

template <> void Stream::write(const BoundingConvexMesh::Desc& desc) {
	*this << (Bounds::Desc&)desc;
	*this << (bool)false;//desc.bCook;
	write(desc.vertices.begin(), desc.vertices.end());
	write(desc.triangles.begin(), desc.triangles.end());
}

//------------------------------------------------------------------------------

template <> void Stream::read(Bounds::Desc::Ptr& desc) const {
	Bounds::Type type;
	*this >> type;

	switch (type) {
	case Bounds::TYPE_PLANE:
	{
		BoundingPlane::Desc* pDesc = new BoundingPlane::Desc;
		desc.reset(pDesc);
		*this >> *pDesc;
		break;
	}
	case Bounds::TYPE_SPHERE:
	{
		BoundingSphere::Desc* pDesc = new BoundingSphere::Desc;
		desc.reset(pDesc);
		*this >> *pDesc;
		break;
	}
	case Bounds::TYPE_CYLINDER:
	{
		BoundingCylinder::Desc* pDesc = new BoundingCylinder::Desc;
		desc.reset(pDesc);
		*this >> *pDesc;
		break;
	}
	case Bounds::TYPE_BOX:
	{
		BoundingBox::Desc* pDesc = new BoundingBox::Desc;
		desc.reset(pDesc);
		*this >> *pDesc;
		break;
	}
	case Bounds::TYPE_CONVEX_MESH:
	{
		BoundingConvexMesh::Desc* pDesc = new BoundingConvexMesh::Desc;
		desc.reset(pDesc);
		*this >> *pDesc;
		break;
	}
	}
}

template <> void Stream::read(Bounds::Ptr& bounds) const {
	Bounds::Desc::Ptr desc;
	*this >> desc;
	ASSERT(desc != NULL)
	bounds = desc->create();
	ASSERT(bounds != NULL)
}

template <> void Stream::write(const Bounds::Desc::Ptr& desc) {
	*this << desc->getType();
	
	switch (desc->getType()) {
	case Bounds::TYPE_PLANE:
		*this << *static_cast<const BoundingPlane::Desc*>(desc.get());
		break;
	case Bounds::TYPE_SPHERE:
		*this << *static_cast<const BoundingSphere::Desc*>(desc.get());
		break;
	case Bounds::TYPE_CYLINDER:
		*this << *static_cast<const BoundingCylinder::Desc*>(desc.get());
		break;
	case Bounds::TYPE_BOX:
		*this << *static_cast<const BoundingBox::Desc*>(desc.get());
		break;
	case Bounds::TYPE_CONVEX_MESH:
		*this << *static_cast<const BoundingConvexMesh::Desc*>(desc.get());
		break;
	}
}

template <> void Stream::write(const Bounds::Ptr& bounds) {
	Bounds::Desc::Ptr desc = bounds->toDesc();
	ASSERT(desc != NULL)
	*this << desc;
}

} // namespace

//------------------------------------------------------------------------------

void golem::XMLData(Bounds::Desc::Ptr &val, XMLContext* context, bool create) {
	ASSERT(context)
	
	if (create) {
		switch (val->getType()) {
		case Bounds::TYPE_PLANE:
			XMLData(*dynamic_cast<BoundingPlane::Desc*>(val.get()), context, create);
			break;
		case Bounds::TYPE_SPHERE:
			XMLData(*dynamic_cast<BoundingSphere::Desc*>(val.get()), context, create);
			break;
		case Bounds::TYPE_CYLINDER:
			XMLData(*dynamic_cast<BoundingCylinder::Desc*>(val.get()), context, create);
			break;
		case Bounds::TYPE_BOX:
			XMLData(*dynamic_cast<BoundingBox::Desc*>(val.get()), context, create);
			break;
		case Bounds::TYPE_CONVEX_MESH:
			XMLData(*dynamic_cast<BoundingConvexMesh::Desc*>(val.get()), context, create);
			break;
		}
	}
	else {
		std::string type = "undef";
		try {
			XMLData("type", type, context, create);
		}
		catch (MsgXMLParserAttributeNotFound&) {
		}
		if (!type.compare("plane")) {
			BoundingPlane::Desc* pDesc = new BoundingPlane::Desc;
			val.reset(pDesc);
			XMLData(*pDesc, context, create);
		}
		else if (!type.compare("sphere")) {
			BoundingSphere::Desc* pDesc = new BoundingSphere::Desc;
			val.reset(pDesc);
			XMLData(*pDesc, context, create);
		}
		else if (!type.compare("cylinder")) {
			BoundingCylinder::Desc* pDesc = new BoundingCylinder::Desc;
			val.reset(pDesc);
			XMLData(*pDesc, context, create);
		}
		else if (!type.compare("box")) {
			BoundingBox::Desc* pDesc = new BoundingBox::Desc;
			val.reset(pDesc);
			XMLData(*pDesc, context, create);
		}
		else if (!type.compare("convex_mesh")) {
			BoundingConvexMesh::Desc* pDesc = new BoundingConvexMesh::Desc;
			val.reset(pDesc);
			XMLData(*pDesc, context, create);
		}
		else
			throw MsgXMLParser(Message::LEVEL_ERROR, "XMLData(): unknown bounds type: %s", type.c_str());;
	}
}

void golem::XMLData(Bounds::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData("group", val.group, context, create);
	XMLData(val.pose, context->getContextFirst("pose"), create);
}

void golem::XMLData(BoundingPlane::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData((Bounds::Desc&)val, context, create);
	XMLData(val.normal, context->getContextFirst("normal"), create);
	XMLData("distance", val.distance, context, create);
	try {
		XMLData("grid_size", val.gridSize, context, create);
	}
	catch (MsgXMLParserAttributeNotFound&) {
	}
	try {
		XMLData("grid_delta", val.gridDelta, context, create);
	}
	catch (MsgXMLParserAttributeNotFound&) {
	}
}

void golem::XMLData(BoundingSphere::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData((Bounds::Desc&)val, context, create);
	XMLData("radius", val.radius, context, create);
	try {
		XMLData("slices", val.numOfSlices, context, create);
	}
	catch (MsgXMLParserAttributeNotFound&) {
	}
	try {
		XMLData("stacks", val.numOfStacks, context, create);
	}
	catch (MsgXMLParserAttributeNotFound&) {
	}
}

void golem::XMLData(BoundingCylinder::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData((Bounds::Desc&)val, context, create);
	XMLData("radius", val.radius, context, create);
	XMLData("length", val.length, context, create);
	try {
		XMLData("slices", val.numOfSlices, context, create);
	}
	catch (MsgXMLParserAttributeNotFound&) {
	}
}

void golem::XMLData(BoundingBox::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData((Bounds::Desc&)val, context, create);
	XMLData(val.dimensions, context->getContextFirst("dimensions"), create);
}

void golem::XMLData(BoundingConvexMesh::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData((Bounds::Desc&)val, context, create);
	XMLData(val.vertices, val.vertices.max_size(), context, "vertex");
	val.bCook = true;
	try {
		XMLData(val.triangles, val.triangles.max_size(), context, "triangle");
		val.bCook = false;
	}
	catch (golem::MsgXMLParserNameNotFound&) {}
	bool triangleInv = false;
	try {
		XMLData("triangle_inv", triangleInv, context, create);
		if (triangleInv && !create)
			for (std::vector<Triangle>::iterator i = val.triangles.begin(); i != val.triangles.end(); ++i)
				std::swap(i->t1, i->t2);
	}
	catch (MsgXMLParserAttributeNotFound&) {}
}

//------------------------------------------------------------------------------

void golem::XMLData(const std::string &attr, Message::Level& val, XMLContext* context, bool create) {
	std::string level;
	XMLData(attr, level, context, create);
	transform(level.begin(), level.end(), level.begin(), tolower);
	val =
		level=="verbose"	?	Message::LEVEL_VERBOSE	:
		level=="debug"		?	Message::LEVEL_DEBUG	:
		level=="info"		?	Message::LEVEL_INFO		:
		level=="warning"	?	Message::LEVEL_WARNING	:
		level=="error"		?	Message::LEVEL_ERROR	:
		level=="crit"		?	Message::LEVEL_CRIT		:
								Message::LEVEL_UNDEF;
}

void golem::XMLData(const std::string &attr, Thread::Priority& val, XMLContext* context, bool create) {
	std::string level;
	XMLData(attr, level, context, create);
	transform(level.begin(), level.end(), level.begin(), tolower);
	val =
		level=="critical"		?	Thread::TIME_CRITICAL	:
		level=="highest"		?	Thread::HIGHEST			:
		level=="above_normal"	?	Thread::ABOVE_NORMAL	:
		level=="normal"			?	Thread::NORMAL			:
		level=="below_normal"	?	Thread::BELOW_NORMAL	:
		level=="lowest"			?	Thread::LOWEST			:
		level=="idle"			?	Thread::IDLE			:
									Thread::ERROR_RETURN;
}

//------------------------------------------------------------------------------

void golem::XMLData(MessageStream::Desc::Ptr& desc, XMLContext* context, bool create) {
	desc.reset(new BufferedStream::Desc);
	Message::Level level;
	XMLData("level", level, context, create);
	desc->messageFilter.reset(level == Message::LEVEL_UNDEF ? NULL : new LevelMessageFilter(level));
}

void golem::XMLData(golem::Context::Desc& desc, XMLContext* context, bool create) {
	ASSERT(context)
	
	try {
		desc.randSeed._U32[0] = 0;
		XMLData("seed", desc.randSeed._U32[0], context->getContextFirst("rand"), create);
	}
	catch (golem::MsgXMLParser&) {}
	desc.randSeed._U32[1] = 0;
	if (desc.randSeed._U32[0] == 0)
		desc.randSeed.generate();
	
	XMLData(desc.messageStreamDesc, context->getContextFirst("messages"), create);
	try {
		desc.threadParallels = 0;
		XMLData("threads", desc.threadParallels, context->getContextFirst("parallels"), create);
	}
	catch (golem::MsgXMLParser&) {}
}

//------------------------------------------------------------------------------
