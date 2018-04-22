/** @file Model.h
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

#pragma once
#ifndef _GOLEM_CONTACT_MODEL_H_
#define _GOLEM_CONTACT_MODEL_H_

//------------------------------------------------------------------------------

#include <Golem/Contact/Contact3D.h>
#include <Golem/Plugin/Point.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Model density */
class Model {
public:
	typedef golem::shared_ptr<Model> Ptr;
	/** Collection */
	typedef std::map<std::string, Ptr> Map;

	/** Contact creator description */
	class Contact3DDesc {
	public:
		/** Type-desc pair */
		typedef std::pair<Contact3D::Type, Contact3DDesc> TypeDesc;
		/** Collection */
		typedef std::vector<TypeDesc> Seq;

		/** Point cloud subsampling */
		golem::U32 subsampleSize;
		/** Maximum distance */
		golem::Real distance;
		/** Likelihood ~ exp(-lambda*distance) */
		golem::Real lambda;
		/** Maximum normal slope */
		golem::Real normalSlope;
		/** Min number of contacts */
		golem::U32 minNum;

		/** Constructs description. */
		Contact3DDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			subsampleSize = 0;
			distance = golem::Real(0.01);
			lambda = golem::Real(100.0);
			normalSlope = golem::REAL_PI;
			minNum = 100;
		}
		/** Assert that the description is valid. */
		void assertValid(Contact3D::Type type, const Assert::Context& ac) const {
			Assert::valid(distance > golem::REAL_ZERO, ac, "distance: <= 0");
			Assert::valid(lambda > golem::REAL_ZERO, ac, "lambda: <= 0");
			Assert::valid(normalSlope > golem::REAL_ZERO, ac, "normalSlope: <= 0");
			Assert::valid(minNum > 0, ac, "minNum: <= 0");
		}
		/** Load descritpion from xml context. */
		void load(Contact3D::Type type, const golem::XMLContext* xmlcontext);
	};

	/** Description */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		/** Collection */
		typedef std::map<std::string, Ptr> Map;

		/** Contact descriptions */
		Contact3DDesc::Seq contactDescSeq;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		virtual ~Desc() {}
		/** Creates the object from the description. */
		virtual Model::Ptr create(golem::Context& context, const std::string& name) const {
			return Model::Ptr(new Model(*this, context, name));
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			contactDescSeq.clear();
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Assert::valid(!contactDescSeq.empty(), ac, "contactDescSeq: empty");
			for (Contact3DDesc::Seq::const_iterator i = contactDescSeq.begin(); i != contactDescSeq.end(); ++i)
				i->second.assertValid(i->first, Assert::Context(ac, "contactDescSeq[]."));
		}
		/** Load descritpion from xml context. */
		virtual void load(const golem::XMLContext* xmlcontext);
	};

	/** Create 3D contact model density */
	virtual bool create(const data::Point3D& points, const golem::Mat34& pose, const golem::Contact3D::Triangle::Seq& triangles, Contact3D::Data& contacts) const;

	/** Model density name */
	const std::string& getName() const {
		return name;
	}

	/** Model density description */
	Desc& getDesc() {
		return desc;
	}
	const Desc& getDesc() const {
		return desc;
	}

	/** Bye bye */
	virtual ~Model() {}

protected:
	/** Context object */
	golem::Context &context;
	/** Parallels */
	golem::Parallels* parallels;

	/** Model density name */
	const std::string name;
	/** Model density description */
	Desc desc;

	/** Creates Model */
	Model(const Desc& desc, golem::Context& context, const std::string& name);
};

void XMLData(golem::Model::Contact3DDesc::Seq::value_type& val, golem::XMLContext* context, bool create = false);
void XMLData(golem::Model::Desc::Map::value_type& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CONTACT_MODEL_H_*/
