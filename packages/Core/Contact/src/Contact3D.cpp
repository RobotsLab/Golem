/** @file Contact3D.cpp
 *
 * Contact3D
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

#include <Golem/Contact/Contact3D.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const char* Contact3D::typeName[TYPE_SIZE] = {
	"Point3D", "Normal3D", "Feature3D", "Part3D"
};
const char* Contact3D::relationName[RELATION_SIZE] = {
	"None", "All", "Volume", "Surface", "Edge", "Vertex", "Point"
};
golem::RGBA Contact3D::relationColour[RELATION_SIZE] = {
	golem::RGBA::BLACK, golem::RGBA::BLACK, golem::RGBA::BLACK, golem::RGBA::WHITE, golem::RGBA::YELLOW, golem::RGBA::CYAN, golem::RGBA::MAGENTA
};

//------------------------------------------------------------------------------

void golem::Contact3D::Feature3DProperty::setStdDev(size_t j, data::Feature3D::Real stdDev) {
	covarianceSqrt[j] = stdDev;
	covariance[j] = golem::Math::sqr(covarianceSqrt[j]);
	covarianceInv[j] = golem::numeric_const<data::Feature3D::Real>::ONE / (covariance[j]);
	covarianceInvSqrt[j] = golem::numeric_const<data::Feature3D::Real>::ONE / (covarianceSqrt[j]);
}

void golem::Contact3D::Feature3DProperty::setStdDev(const data::Feature3D::Feature& stdDev) {
	for (size_t j = 0; j < stdDev.size(); ++j)
		setStdDev(j, stdDev[j]);
}

void golem::Contact3D::process(const Feature3DProperty::Desc& desc, Contact3D::Data::Map& dataMap) {
	// feature dimensions
	const size_t dimensions = Data::getDimensions(dataMap);

	data::Feature3D::Feature stdDevMin(dimensions, golem::numeric_const<data::Feature3D::Real>::MAX);

	// model density properties
	for (Contact3D::Data::Map::iterator k = dataMap.begin(); k != dataMap.end(); ++k) {
		if (k->second.type != TYPE_FEATURE)
			throw Message(Message::LEVEL_ERROR, "Contact3D::process(): invalid contact type %s", typeName[size_t(k->second.type)]);

		Contact3D::Seq& model = k->second.model;
		Feature3DProperty& property = k->second.feature3DProperty;

		// normalise model distribution
		if (!golem::Sample<Real>::normalise<golem::Ref1>(model))
			throw Message(Message::LEVEL_ERROR, "Contact3D::process(): unable to normalise model density");

		// properties
		data::Feature3D::getProperty(model.size(), dimensions,
			[=] (size_t i, size_t j) -> data::Feature3D::Real {
				return static_cast<data::Feature3D::Real>(powerScale(static_cast<golem::Real>(model[i].feature[j]), desc.powerScaling));
			},
			[=] (size_t j, data::Feature3D::Real val) -> data::Feature3D::Real {
				return static_cast<data::Feature3D::Real>(golem::Math::sqr(desc.stdDevFac[j]) * val);
			},
			property);

		// min covariance
		for (size_t j = 0; j < dimensions; ++j)
			if (stdDevMin[j] > property.covarianceSqrt[j] && desc.adaptStdDevMin < property.covarianceSqrt[j])
				stdDevMin[j] = property.covarianceSqrt[j];
	}

	// min covariance for all descriptors
	data::Feature3D::Real stdDevMinAll = golem::numeric_const<data::Feature3D::Real>::MAX;
	for (size_t j = 0; j < dimensions; ++j)
		if (stdDevMinAll > stdDevMin[j])
			stdDevMinAll = stdDevMin[j];
	if (stdDevMinAll > desc.adaptStdDevMax)
		throw Message(Message::LEVEL_ERROR, "Contact3D::process(): unable to compute covariance");

	// update min covariance if invalid
	for (size_t j = 0; j < dimensions; ++j)
		if (stdDevMin[j] > desc.adaptStdDevMax)
			stdDevMin[j] = stdDevMinAll;

	// covariance adaptation
	for (Contact3D::Data::Map::iterator k = dataMap.begin(); k != dataMap.end(); ++k) {
		Feature3DProperty& property = k->second.feature3DProperty;

		property.scale.assign(dimensions, golem::numeric_const<data::Feature3D::Real>::ONE);
		property.penalty = golem::numeric_const<data::Feature3D::Real>::ZERO;

		for (size_t j = 0; j < dimensions; ++j) {
			bool adapt = false;
			data::Feature3D::Real stdDev = property.covarianceSqrt[j];

			// zero check
			if (desc.adaptStdDevMinEnable && stdDev < golem::Math::sqr(stdDevMin[j])) {
				stdDev = golem::Math::sqr(stdDevMin[j]);
				adapt = true;
			}

			// update covariance
			if (adapt)
				property.setStdDev(j, stdDev);
		}
	}
}

void golem::Contact3D::process(const Data::Desc& desc, Contact3D::Data::Map& dataMap) {
	// check data
	Data::assertValid(dataMap);

	if (Data::getType(dataMap) == TYPE_FEATURE)
		process(desc.feature3DDesc, dataMap);
}

void golem::Contact3D::process(const Feature3DProperty::Desc& desc, const data::Feature3D& features, Contact3D::Data::Map& dataMap) {
	// compute covariance
	process(desc, dataMap);

	// feature dimensions
	const size_t dimensions = Data::getDimensions(dataMap);

	// object density properties
	data::Feature3D::FeatureSeq objectFeatureSeq(features.getSize());
	for (size_t i = 0; i < objectFeatureSeq.size(); ++i) {
		data::Point3D::Mat33 orientation;
		features.getFeature(i, objectFeatureSeq[i], orientation);
	}
	data::Feature3D::Property objectProperty;
	data::Feature3D::getProperty(objectFeatureSeq.size(), dimensions,
		[=] (size_t i, size_t j) -> data::Feature3D::Real {
			return static_cast<data::Feature3D::Real>(powerScale(static_cast<golem::Real>(objectFeatureSeq[i][j]), desc.powerScaling));
		},
		[=] (size_t j, data::Feature3D::Real val) -> data::Feature3D::Real {
			return static_cast<data::Feature3D::Real>(golem::Math::sqr(desc.stdDevFac[j]) * val);
		},
		objectProperty);

	// covariance adaptation
	for (Contact3D::Data::Map::iterator k = dataMap.begin(); k != dataMap.end(); ++k) {
		Feature3DProperty& property = k->second.feature3DProperty;

		property.scale.assign(dimensions, golem::numeric_const<data::Feature3D::Real>::ONE);
		property.penalty = golem::numeric_const<data::Feature3D::Real>::ZERO;

		for (size_t j = 0; j < dimensions; ++j) {
			bool adapt = false;
			data::Feature3D::Real stdDev = property.covarianceSqrt[j];

			// standard deviation overlap
			if (desc.adaptStdDevOverlapEnable && stdDev < objectProperty.covarianceSqrt[j] / desc.adaptStdDevOverlap) {
				stdDev = objectProperty.covarianceSqrt[j] / desc.adaptStdDevOverlap;
				adapt = true;
			}
			// mean overlap
			if (desc.adaptStdDevMeanEnable && stdDev < Math::abs(property.mean[j] - objectProperty.mean[j]) - objectProperty.covarianceSqrt[j] + objectProperty.covarianceSqrt[j] / desc.adaptStdDevOverlap) {
				stdDev = Math::abs(property.mean[j] - objectProperty.mean[j]) - objectProperty.covarianceSqrt[j] + objectProperty.covarianceSqrt[j] / desc.adaptStdDevOverlap;
				adapt = true;
			}

			property.scale[j] = stdDev / property.covarianceSqrt[j];
			property.penalty += property.scale[j] - golem::numeric_const<data::Feature3D::Real>::ONE;

			// update covariance
			if (adapt)
				property.setStdDev(j, stdDev);
		}

		property.penalty *= desc.adaptStdDevPenalty;
	}
}

void golem::Contact3D::process(const Data::Desc& desc, const data::Point3D& points, Contact3D::Data::Map& dataMap) {
	// check data
	Data::assertValid(dataMap);

	// all available contact interfaces, no order or preference
	const Contact3D::TypePtr typePtr(&points);

	if (typePtr.features)
		process(desc.feature3DDesc, *typePtr.features, dataMap);
}

//------------------------------------------------------------------------------

void golem::Contact3D::Feature3DProperty::Desc::load(const golem::XMLContext * xmlcontext) {
	stdDevFac.resize(data::Feature3D::FEATURE_SIZE); // maximum size
	golem::XMLData(stdDevFac, "v", xmlcontext->getContextFirst("std_dev"), false);

	golem::XMLData("power_scaling", powerScaling, xmlcontext->getContextFirst("std_dev"), false);

	golem::XMLData("min", adaptStdDevMin, xmlcontext->getContextFirst("std_dev_adapt"), false);
	golem::XMLData("max", adaptStdDevMax, xmlcontext->getContextFirst("std_dev_adapt"), false);
	golem::XMLData("overlap", adaptStdDevOverlap, xmlcontext->getContextFirst("std_dev_adapt"), false);
	golem::XMLData("penalty", adaptStdDevPenalty, xmlcontext->getContextFirst("std_dev_adapt"), false);

	golem::XMLData("enable_min", adaptStdDevMinEnable, xmlcontext->getContextFirst("std_dev_adapt"), false);
	golem::XMLData("enable_overlap", adaptStdDevOverlapEnable, xmlcontext->getContextFirst("std_dev_adapt"), false);
	golem::XMLData("enable_mean", adaptStdDevMeanEnable, xmlcontext->getContextFirst("std_dev_adapt"), false);
}

void golem::Contact3D::Data::Desc::load(const golem::XMLContext* xmlcontext) {
	feature3DDesc.load(xmlcontext->getContextFirst("feature_3d"));
}

//------------------------------------------------------------------------------

void Contact3D::convert(const golem::Bounds::Seq& bounds, Triangle::Seq& triangles) {
	for (golem::Bounds::Seq::const_iterator i = bounds.begin(); i != bounds.end(); ++i) {
		const golem::BoundingConvexMesh* mesh = dynamic_cast<const golem::BoundingConvexMesh*>(i->get());
		if (mesh != nullptr)
			for (golem::TriangleSeq::const_iterator j = mesh->getTriangles().begin(); j != mesh->getTriangles().end(); ++j)
				triangles.push_back(Contact3D::Triangle(mesh->getVertices()[j->t1], mesh->getVertices()[j->t3], mesh->getVertices()[j->t2]));
	}
}

void Contact3D::find(const Triangle::Seq& triangles, const golem::Vec3& point, golem::Real& distance, golem::Vec3& projection, Relation& relation) {
	Vec3 p(point);
	Vec3 proj;
	Real dist((Real)std::min(golem::numeric_const<Real>::MAX, Real(distance))), d;

	// verices
	d = dist;
	for (Triangle::Seq::const_iterator i = triangles.begin(); i != triangles.end(); ++i)
		i->distanceVertex(p, proj, dist);
	if (d > dist)
		relation = RELATION_VERTEX;

	// edges
	d = dist;
	for (Triangle::Seq::const_iterator i = triangles.begin(); i != triangles.end(); ++i)
		i->distanceEdge(p, proj, dist);
	if (d > dist)
		relation = RELATION_EDGE;

	// triangles
	d = dist;
	for (Triangle::Seq::const_iterator i = triangles.begin(); i != triangles.end(); ++i)
		i->distanceTriangle(p, proj, dist);
	if (d > golem::Math::abs(dist))
		relation = RELATION_SURFACE;

	// volume
	if (dist < golem::numeric_const<Real>::ZERO) {
		dist = -dist;
		relation = RELATION_VOLUME;
	}

	distance = (golem::Real)dist;
	projection.set(proj);
}

//------------------------------------------------------------------------------

void Contact3D::draw(const Contact3D::Appearance& appearance, const golem::Mat34& pose, golem::DebugRenderer& renderer, golem::U8 a) const {
	if (appearance.relation == Contact3D::RELATION_DFLT || appearance.relation == Contact3D::RELATION_POINT || appearance.relation == relation && appearance.relation != Contact3D::RELATION_NONE) {
		golem::Mat34 trn;
		trn.setInverse(local.toMat34());
		trn.multiply(pose, trn);

		if (appearance.pointSize > REAL_ZERO) {
			const RGBA colour = appearance.colour[(size_t)Contact3D::RELATION_POINT];
			renderer.addPoint(trn.p, RGBA(colour._rgba.r, colour._rgba.g, colour._rgba.b, a));
		}
		if (appearance.relation == Contact3D::RELATION_POINT)
			return;

		golem::Vec3 p;
		if (model == INDEX_DEFAULT) {
			((const golem::Mat33&)trn.R).multiply(p, projection);
			p.multiply(distance, p);
		}
		else
			p.multiply(distance, projection);

		p.add(trn.p, p);

		if (appearance.raysShow)
			renderer.addLine(trn.p, p, appearance.colour[(size_t)relation]);
		if (appearance.framesShow)
			renderer.addAxes(trn, appearance.frameSize);
	}
}

void Contact3D::draw(const Contact3D::Appearance& appearance, const Seq& contacts, const golem::Mat34& pose, golem::DebugRenderer& renderer) {
	if (appearance.pointsShow) {
		Real norm = Real(golem::numeric_const<U8>::MAX);
		if (appearance.pointSize > REAL_ZERO) {
			renderer.setPointSize(appearance.pointSize);
			norm *= golem::Sample<Real>::getNormWeight<golem::Ref1>(contacts);
		}

		for (Contact3D::Seq::const_iterator i = contacts.begin(); i != contacts.end(); ++i)
			i->draw(appearance, pose, renderer, U8(norm*i->weight));
	}
	if (appearance.boundsFrameShow)
		renderer.addAxes(pose, appearance.boundsFrameSize);
}

void golem::Contact3D::drawPoints(const Seq& contacts, const golem::RGBA& colour, golem::Real normalLen, const golem::Mat34& pose, golem::DebugRenderer& renderer) {
	const RBCoord rbpose(pose);
	const Real norm = Real(golem::numeric_const<U8>::MAX)*golem::Sample<Real>::getNormWeight<golem::Ref1>(contacts);
	for (Contact3D::Seq::const_iterator j = contacts.begin(); j != contacts.end(); ++j) {
		RBCoord frame;
		frame.setInverse(j->local);
		frame.multiply(rbpose, frame);
		
		const RGBA colour(colour._rgba.r, colour._rgba.g, colour._rgba.b, U8(norm*j->weight));
		renderer.addPoint(frame.p, colour);
		
		if (normalLen > REAL_EPS) {
			Vec3 z;
			frame.q.multiply(z, Vec3::axisZ());
			Vec3 p;
			p.multiplyAdd(normalLen, z, frame.p);
			renderer.addLine(frame.p, p, colour);
		}
	}
}

//------------------------------------------------------------------------------

void golem::Contact3D::Appearance::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("points_show", pointsShow, const_cast<golem::XMLContext*>(xmlcontext), false);
	try {
		golem::XMLData("rays_show", raysShow, const_cast<golem::XMLContext*>(xmlcontext), false);
	}
	catch (const golem::MsgXMLParser&) {}
	golem::XMLData("frames_show", framesShow, const_cast<golem::XMLContext*>(xmlcontext), false);
	try {
		golem::XMLData(colour, colour + Contact3D::RELATION_SIZE, const_cast<golem::XMLContext*>(xmlcontext), "colour", false);
	}
	catch (const golem::MsgXMLParserIncompleteData&) {}
	try {
		golem::XMLData("point_size", pointSize, const_cast<golem::XMLContext*>(xmlcontext), false);
	}
	catch (const golem::MsgXMLParser&) {}
	golem::XMLData(frameSize, xmlcontext->getContextFirst("frame_size"), false);
	golem::XMLData("bounds_frame_show", boundsFrameShow, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData(boundsFrameSize, xmlcontext->getContextFirst("bounds_frame_size"), false);
}

void golem::XMLData(const std::string &attr, Contact3D::Type& val, golem::XMLContext* context, bool create) {
	if (create) {
		std::string type =
			val == Contact3D::Type::TYPE_NORMAL ? "normal" :
			val == Contact3D::Type::TYPE_FEATURE ? "feature" :
			val == Contact3D::Type::TYPE_PART ? "part" : "point";
		XMLData(attr, type, context, true);
	}
	else {
		std::string type;
		XMLData(attr, type, context, false);
		if (!type.compare("point"))
			val = Contact3D::Type::TYPE_POINT;
		else if (!type.compare("normal"))
			val = Contact3D::Type::TYPE_NORMAL;
		else if (!type.compare("feature"))
			val = Contact3D::Type::TYPE_FEATURE;
		else if (!type.compare("part"))
			val = Contact3D::Type::TYPE_PART;
		else
			throw MsgXMLParser(Message::LEVEL_ERROR, "XMLData(Contact3D::Type&): unknown type: %s", type.c_str());;
	}
}

void golem::XMLData(Contact3D::Appearance::Map::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("id", const_cast<std::string&>(val.first), xmlcontext, create);
	val.second.load(xmlcontext);
}

void golem::XMLData(Point3DKernelDist& val, golem::XMLContext* context, bool create) {
	if (create) {
		// TODO
		return;
	}

	RBDist rbdist;
	Real dist;

	golem::XMLData("correspondences", val.correspondencesMin, context, create);
	golem::XMLData("penalty", val.distPenalty, context, create);

	golem::XMLData(rbdist, context->getContextFirst("dist", false), false);
	val.setCovInv(rbdist); // 1:1
	golem::XMLData(rbdist, context->getContextFirst("dist_max", false), false);
	val.distMax = rbdist.getSqr(); // sqr()

	golem::XMLData("feature", dist, context->getContextFirst("dist", false), false);
	val.featureDist = Math::sqr(dist); // sqr()
	golem::XMLData("feature", dist, context->getContextFirst("dist_max", false), false);
	val.featureDistMax = Math::sqr(dist); // sqr()
}

//------------------------------------------------------------------------------

template <> void golem::Stream::read(golem::Contact3D::Seq::value_type& value) const {
	//read(&value, 1);

	read(value.weight);
	read(value.cdf);
	read(value.global);
	read(value.local);

	read(value.feature);
	//read(&value.feature, 1); // Legacy
	
	read(value.model);
	read(value.distance);
	read(value.projection);
	read(value.relation);
	
	// Legacy: Contact3D memory alignment
	//U32 padding;
	//read(padding);
}

template <> void golem::Stream::write(const golem::Contact3D::Seq::value_type& value) {
	//write(&value, 1);

	write(value.weight);
	write(value.cdf);
	write(value.global);
	write(value.local);
	write(value.feature);
	write(value.model);
	write(value.distance);
	write(value.projection);
	write(value.relation);

	// Legacy: Contact3D memory alignment
	//U32 padding = 0;
	//write(padding);
}

template <> void golem::Stream::read(golem::Contact3D::Feature3DProperty& value) const {
	read(value.mean);
	read(value.covariance);
	read(value.covarianceSqrt);
	read(value.covarianceInv);
	read(value.covarianceInvSqrt);
	read(value.scale);
	read(value.penalty);
}

template <> void golem::Stream::write(const golem::Contact3D::Feature3DProperty& value) {
	write(value.mean);
	write(value.covariance);
	write(value.covarianceSqrt);
	write(value.covarianceInv);
	write(value.covarianceInvSqrt);
	write(value.scale);
	write(value.penalty);
}

template <> void golem::Stream::read(golem::Contact3D::Data& value) const {
	read(value.type);
	read(value.model, value.model.begin());
	read(value.feature3DProperty);

	//value.clear();
	//value.type = Contact3D::TYPE_FEATURE;
	//read(value.model, value.model.begin());
}

template <> void golem::Stream::write(const golem::Contact3D::Data& value) {
	write(value.type);
	write(value.model.begin(), value.model.end());
	write(value.feature3DProperty);
}

template <> void golem::Stream::read(golem::Contact3D::Data::Map::value_type& value) const {
	read(const_cast<golem::U32&>(value.first));
	read(value.second);
}

template <> void golem::Stream::write(const golem::Contact3D::Data::Map::value_type& value) {
	write(value.first);
	write(value.second);
}

//------------------------------------------------------------------------------
