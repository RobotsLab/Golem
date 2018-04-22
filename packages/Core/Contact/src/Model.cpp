/** @file Model.cpp
 *
 * Model density
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

#include <Golem/Contact/Model.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Model::Model(const Desc& desc, golem::Context& context, const std::string& name) : context(context), parallels(context.getParallels()), name(name) {
	desc.assertValid(Assert::Context("Model()."));

	// check parallels
	if (!parallels || parallels->getNumOfThreads() < 1)
		throw golem::Message(golem::Message::LEVEL_CRIT, "Model(): Parallels required");

	this->desc = desc;
}

//------------------------------------------------------------------------------

bool Model::create(const data::Point3D& points, const golem::Mat34& pose, const golem::Contact3D::Triangle::Seq& triangles, Contact3D::Data& contacts) const {
	// all available contact interfaces, no order or preference
	const Contact3D::TypePtr typePtr(&points);
	// preferred contact interface and description
	const Model::Contact3DDesc::Seq::const_iterator contact3DTypeDesc = Contact3D::TypePtr::select(typePtr, desc.contactDescSeq);

	// determine main type
	contacts.type =
		contact3DTypeDesc->first == Contact3D::TYPE_PART ? Contact3D::TYPE_PART :
		contact3DTypeDesc->first == Contact3D::TYPE_FEATURE ? Contact3D::TYPE_FEATURE :
		contact3DTypeDesc->first == Contact3D::TYPE_NORMAL ? Contact3D::TYPE_NORMAL : Contact3D::TYPE_POINT;

	// message prefix
	const std::string prefix = "Model::create(): " + getName() + "-" + Contact3D::typeName[contact3DTypeDesc->first];

	if (triangles.empty()) {
		context.debug("%s: no bounds\n", prefix.c_str());
		return false;
	}

	const size_t numOfPoints = points.getSize();
	if (numOfPoints <= 0)
		throw golem::Message(golem::Message::LEVEL_ERROR, "%s: no available points", prefix.c_str());

	const size_t base = contacts.model.size();
	const size_t size = contact3DTypeDesc->second.subsampleSize <= 0 || (size_t)contact3DTypeDesc->second.subsampleSize >= numOfPoints ? numOfPoints : (size_t)contact3DTypeDesc->second.subsampleSize;
	const bool sampling = size < numOfPoints;

	typedef std::map<data::Part3D::Index, Contact3D> Contact3DMap;
	Contact3DMap contact3DMap;

	contacts.model.reserve(base + size);
	golem::CriticalSection cs;
	size_t j = 0;
	ParallelsTask(parallels, [&] (ParallelsTask*) {
		const golem::U32 jobId = parallels->getCurrentJob()->getJobId();
		golem::Rand rand(golem::RandSeed(this->context.getRandSeed()._U32[0] + jobId, (golem::U32)0));
		
		// contact_frame(A)*model_frame(d) = link_frame(B)
		Contact3D contact;
		Contact3DMap pointContact3DMap;
		bool update = false;

		for (size_t i = 0;;) {
			{
				golem::CriticalSectionWrapper csw(cs);
				if (update) {
					if (contact3DTypeDesc->first == Contact3D::TYPE_PART) {
						for (Contact3DMap::const_iterator p = pointContact3DMap.begin(); p != pointContact3DMap.end(); ++p) {
							Contact3DMap::iterator r = contact3DMap.find(p->first);
							if (r == contact3DMap.end())
								contact3DMap.insert(*p); // insert if realisation is not there yet
							else
								r->second.weight += p->second.weight; // update weight given new point
						}
					}
					else
						contacts.model.push_back(contact);
				}
				if (j >= size)
					break;
				i = j++;
			}

			// no update by default
			update = false;

			// sample or select feature point
			const size_t index = sampling ? points.samplePoint(rand) : i;
			const data::Point3D::Point point = points.getPoint(index);
			contact.global.p.set((const golem::data::Point3D::Vec3&)point); // duplicate, point and frame can be different for parts only

			// determine contact type, continue only if contact point is within receptive field
			contact.distance = contact3DTypeDesc->second.distance;
			contact.relation = Contact3D::RELATION_NONE;
			Contact3D::find(triangles, contact.global.p, contact.distance, contact.projection, contact.relation);
			if (contact.relation == Contact3D::RELATION_NONE)
				continue;

			// projection
			const Vec3 projection = contact.projection; // backup
			contact.projection.subtract(contact.projection, contact.global.p);
			if (contact.projection.normalise() < golem::REAL_EPS)
				continue;

			// No preferred interface - for all interfaces if they support normals: check if normal is within the range from projection
			if (typePtr.normals) {
				if (
					contact.relation != Contact3D::RELATION_VOLUME &&
					golem::Math::acos(contact.projection.dot(golem::Vec3(typePtr.normals->getNormal(index)))) > contact3DTypeDesc->second.normalSlope
					)
					continue;
			}

			// likelihood
			const golem::Real likelihood = contact.distance > golem::REAL_ZERO ? golem::Math::exp(-contact3DTypeDesc->second.lambda*golem::Math::sqr(contact.distance)) : golem::REAL_ONE;

			// Preference - choose interface according to contact3DTypeDesc:

			// compute frame
			if (contact3DTypeDesc->first == Contact3D::TYPE_PART) {
				// find all parts that involve cloud point index
				data::Part3D::Part::Map partMap;
				typePtr.parts->getPart(index, partMap);
				// nothing to do if point is not associated with any parts
				if (partMap.empty())
					continue;

				pointContact3DMap.clear();
				for (data::Part3D::Part::Map::iterator p = partMap.begin(); p != partMap.end(); ++p) {
					Contact3D partContact = contact;

					// contact point and orientation is different than the current point
					//partContact.global.p = p->second.frame.p;
					partContact.global.q.fromMat33(p->second.frame.R); // dummy frame - there are no point frames

					// model
					partContact.model = p->second.model;

					// local frame
					golem::Mat34 frameInv;
					frameInv.setInverse(p->second.frame); // A^-1
					partContact.local.multiply(RBCoord(frameInv), RBCoord(pose)); // A*d = B |=> d = A^-1*B

					// projection
					//partContact.projection.subtract(projection, p->second.frame.p);

					// weight takes into account normalisation factor of the particular part realisation
					partContact.weight = point.weight*likelihood*p->second.weight;

					// update by realisation
					pointContact3DMap[p->first] = partContact;
				}

				update = true;
				continue;
			}
			else if (contact3DTypeDesc->first == Contact3D::TYPE_FEATURE) {
				// select feature, get mean orientation
				data::Point3D::Mat33 orientation;
				typePtr.features->getFeature(index, contact.feature, orientation);
				contact.global.q.fromMat33(orientation);

				const golem::Mat34 frame(orientation, contact.global.p);
				golem::Mat34 frameInv;
				frameInv.setInverse(frame); // A^-1
				contact.local.multiply(RBCoord(frameInv), pose); // A*d = B |=> d = A^-1*B

				// transform projection to the local feature frame
				((const golem::Mat33&)frameInv.R).multiply(contact.projection, contact.projection);
			}
			else if (contact3DTypeDesc->first == Contact3D::TYPE_NORMAL) {

			}
			else {
				contact.global.q.setId();
				contact.local.p.subtract(pose.p, contact.global.p); // cf + cp = pp
				contact.local.q.setId();
			}

			contact.weight = point.weight*likelihood;
					
			update = true;
		}
	});

	// single realisations -> models with multiple kernels
	if (contact3DTypeDesc->first == Contact3D::TYPE_PART) {
		// realisations
		for (Contact3DMap::const_iterator p = contact3DMap.begin(); p != contact3DMap.end(); ++p)
			contacts.model.push_back(p->second);
		// models
		std::sort(contacts.model.begin(), contacts.model.end(), [] (const Contact3D& l, const Contact3D& r) -> bool { return l.model < r.model; });
	}

	if (contacts.model.size() <= base) {
		context.debug("%s: no contact\n", prefix.c_str());
		return false;
	}
	else if (contacts.model.size() - base < contact3DTypeDesc->second.minNum) {
		context.debug("%s: number of contacts too small %u < %u\n", prefix.c_str(), contacts.model.size(), contact3DTypeDesc->second.minNum);
		if (base == 0)
			contacts.model.clear();
		else
			contacts.model.resize(base);
		return false;
	}

	// normalisation
	if (!golem::Sample<golem::Real>::normalise<golem::Ref1>(contacts.model))
		throw Message(Message::LEVEL_ERROR, "%s: Unable to normalise model distribution", prefix.c_str());

	context.debug("%s: type=%s, contacts=%u, cdf=%0.6e\n", prefix.c_str(), Contact3D::typeName[size_t(contacts.type)], contacts.model.size() - base, contacts.model.back().cdf);
	return true;
}

//------------------------------------------------------------------------------

void golem::Model::Contact3DDesc::load(Contact3D::Type type, const golem::XMLContext* xmlcontext) {
	golem::XMLData("subsample_size", subsampleSize, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("distance", distance, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("lambda", lambda, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("normal_slope", normalSlope, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("min_num", minNum, const_cast<golem::XMLContext*>(xmlcontext), false);
}

void golem::Model::Desc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData(contactDescSeq, contactDescSeq.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "contact_3d");
}

//------------------------------------------------------------------------------

void golem::XMLData(golem::Model::Contact3DDesc::Seq::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("type", const_cast<golem::Contact3D::Type&>(val.first), xmlcontext, create);
	val.second.load(val.first, xmlcontext);
}

void golem::XMLData(Model::Desc::Map::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("id", const_cast<std::string&>(val.first), xmlcontext, create);
	val.second.reset(new Model::Desc);
	val.second->load(xmlcontext);
}

//------------------------------------------------------------------------------
