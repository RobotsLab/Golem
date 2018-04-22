/** @file Point.cpp
 *
 * Point interface
 *
 * @author Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

//------------------------------------------------------------------------------

#include <Golem/Plugin/Point.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::data::Point3D::Appearance::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData(colour, xmlcontext->getContextFirst("colour"), false);
	try {
		golem::XMLData("size", size, const_cast<golem::XMLContext*>(xmlcontext), false);
	}
	catch (const golem::MsgXMLParserAttributeNotFound&) {}
}

template <> void golem::data::Point3D::Appearance::draw(const data::Point3D::Point::Seq& points, golem::DebugRenderer& renderer) const {
	if (!points.empty()) {
		renderer.setPointSize(size);
		data::Point3D::Real weight = golem::numeric_const<data::Point3D::Real>::ZERO;
		for (data::Point3D::Point::Seq::const_iterator i = points.begin(); i != points.end(); ++i)
			if (weight < i->weight)
				weight = i->weight;
		if (weight < golem::numeric_const<data::Point3D::Real>::EPS)
			weight = golem::numeric_const<data::Point3D::Real>::ONE;
		weight /= golem::numeric_const<data::Point3D::Real>::ONE;
		for (data::Point3D::Point::Seq::const_iterator i = points.begin(); i != points.end(); ++i)
			renderer.addPoint(golem::Vec3(static_cast<const golem::data::Point3D::Vec3&>(*i)), golem::RGBA(colour._rgba.r, colour._rgba.g, colour._rgba.b, golem::U8(i->weight*weight*colour._rgba.a)));
	}
}

//------------------------------------------------------------------------------

template <> void golem::Stream::read(golem::data::Feature3D::Feature& value) const {
	golem::U64 size = 0;
	read(size);
	if (size > (golem::U64)data::Feature3D::Feature::N())
		throw Message(Message::LEVEL_ERROR, "golem::Stream::read(golem::data::Feature3D::Feature): feature size too large %u > %u", (U32)size, (U32)data::Feature3D::Feature::N());
	value.setZero();
	value.resize((size_t)size);
	for (golem::U64 i = 0; i < size; ++i)
		read(value[(size_t)i]);
}

template <> void golem::Stream::write(const golem::data::Feature3D::Feature& value) {
	const golem::U64 size = (golem::U64)value.size();
	write(size);
	for (golem::U64 i = 0; i < size; ++i)
		write(value[(size_t)i]);
}

//------------------------------------------------------------------------------
