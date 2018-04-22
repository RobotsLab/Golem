/** @file Point3D.h
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
#ifndef _GOLEM_CONTACT_POINT_H_
#define _GOLEM_CONTACT_POINT_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Data.h>
#include <Golem/Math/RB.h>
#include <Golem/Math/Sample.h>
#include <Golem/Math/Triangle.h>
#include <Golem/Math/VecN.h>
#include <Golem/Math/Rand.h>
#include <Golem/Plugin/UI.h>
#include <vector>
#include <map>
#include <set>

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

/** Position and normal in 3D.
*	(Optionally) Implemented by Item.
*/
class Point3D {
public:
	/** Point3D interface sequence */
	typedef std::vector<Point3D*> Seq;
	/** Point3D interface sequence */
	typedef std::vector<const Point3D*> ConstSeq;

	/** Default floating point */
	typedef golem::F64 Real;
	/** Default 3D vector */
	typedef golem::_Vec3<Real> Vec3;
	/** Default 3D rotation matrix */
	typedef golem::_Mat33<Real> Mat33;
	/** Default 3D rigid body transformation */
	typedef golem::_Mat34<Real> Mat34;
	/** Default SE(3) distance metric */
	typedef golem::_RBDist<Real> RBDist;

	/** Weighted point */
	class Point : public Vec3, public golem::Sample<Real> {
	public:
		/** Sequence */
		typedef std::vector<Point> Seq;

		/** Construction */
		Point(const Vec3& vec = Vec3(), Real weight = golem::numeric_const<Real>::ONE, Real cdf = -golem::numeric_const<Real>::ONE) :
			Vec3(vec), golem::Sample<Real>(weight, cdf)
		{}
	};

	/** Appearance */
	class Appearance {
	public:
		/** Point colour */
		golem::RGBA colour;
		/** Point size */
		Real size;

		/** Constructs description. */
		Appearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			colour = golem::RGBA::BLACK;
			size = Real(1.0);
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(size > REAL_ZERO, ac, "size: < 0");
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
		
		/** Draw */
		template <typename _Points> void draw(const _Points& points, golem::DebugRenderer& renderer) const {
			renderer.setPointSize(size);
			typedef typename _Points::value_type::Real _Real;
			for (typename _Points::const_iterator i = points.begin(); i != points.end(); ++i)
				renderer.addPoint(golem::Vec3(static_cast<const golem::_Vec3<_Real>&>(*i)), colour);
		}
	};

	/** Total number of available points (width x height) */
	virtual size_t getSize() const = 0;
	/** Dimensions */
	virtual void getDimensions(size_t& width, size_t& height) const = 0;

	/** Sample point. */
	virtual size_t samplePoint(golem::Rand& rand) const = 0;
	/** Conditional point coordinates */
	virtual Point getPoint(size_t index) const = 0;
	/** Conditional point colour */
	virtual golem::RGBA getColour(size_t index) const = 0;

	/** Point transform */
	virtual void transform(const golem::Mat34& trn) = 0;

	/** Sensor frame. */
	virtual void getSensorFrame(golem::Mat34& frame) const = 0;
};

/** Draw */
template <> void Point3D::Appearance::draw(const Point::Seq& points, golem::DebugRenderer& renderer) const;

/** 3D feature consists of a frame and a feature vector.
*	(Optionally) Implemented by Item.
*/
class Normal3D : virtual public Point3D {
public:
	/** Select normal given point index. */
	virtual Vec3 getNormal(size_t index) const = 0;

	/** Query density normal covariance */
	virtual Real getNormalCovariance() const = 0;
};

/** 3D feature consists of a frame and a feature vector.
*	(Optionally) Implemented by Item.
*/
class Feature3D : virtual public Point3D {
public:
	/** Feature size */
	static const size_t FEATURE_SIZE = 36;
	/** Feature vector */
	typedef golem::_VecN<Real, FEATURE_SIZE> Feature;
	/** Feature vector sequence */
	typedef std::vector<Feature> FeatureSeq;
	/** Feature property */
	typedef golem::SampleProperty<Real, Feature> Property;

	/** Select feature given point index. Do not sample orientation - return only mean value to use later with sensor model.  */
	virtual void getFeature(size_t index, Feature& feature, Mat33& orientation) const = 0;

	/** Sample local frame orientations from sensor model. */
	virtual void sampleSensorModel(golem::Rand& rand, size_t index, Mat33& orientation) const = 0;

	/** Query density frame covariance */
	virtual RBDist getFrameCovariance() const = 0;

	/** Feature3D property */
	template <typename _Val, typename _Cov> static void getProperty(size_t size, size_t dim, _Val val, _Cov cov, Property& property) {
		if (size <= 0 || dim <= 0)
			throw golem::Message(golem::Message::LEVEL_ERROR, "Feature3D::getCovariance(): no data provided");

		// estimate bandwidth - scott04multi-dimensional
		const Real bandwidthSqr = golem::Math::pow(golem::numeric_const<Real>::ONE/size, golem::numeric_const<Real>::TWO/(dim + 4));
		
		// compute covariance
		property.mean.resize(dim);
		property.covariance.resize(dim);
		property.covarianceSqrt.resize(dim);
		property.covarianceInv.resize(dim);
		property.covarianceInvSqrt.resize(dim);
		if (!property.create(size, dim, val, [=] (size_t j, Real val) -> Real { return cov(j, bandwidthSqr*val); }))
			throw golem::Message(golem::Message::LEVEL_ERROR, "Feature3D::getCovariance(): Unable to create covariance property");
	}

	/** Feature to string */
	static std::string to_string(const Feature& feature, const char* format, bool index = false) {
		char buff[BUFSIZ], *ptr = buff;
		*ptr = '\0';
		for (size_t i = 0; i < feature.size(); ++i)
			ptr = index ? golem::snprintf(ptr, buff + sizeof(buff), format, i + 1, feature[i]) : golem::snprintf(ptr, buff + sizeof(buff), format, feature[i]);
		return std::string(buff);
	}
};

/** Parts 3D consists of a unique identifier, a frame and has some spatial extent.
*	(Optionally) Implemented by Item.
*/
class Part3D : virtual public Point3D {
public:
	/** Index */
	typedef golem::U64 Index;
	/** Index set */
	typedef std::set<Index> IndexSet;
	/** Index map */
	typedef std::map<Index, Index> IndexMap;
	/** Index sequence map */
	typedef std::map<Index, IndexSet> IndexSetMap;
	/** Model SE(3) distance metric */
	typedef std::map<Index, RBDist> RBDistMap;

	/** Default/uninitialised index */
	static const Index INDEX_DEFAULT = -1;

	/** Part */
	class Part : public golem::Sample<Real> {
	public:
		/** Realisation -> model mapping */
		typedef std::map<Index, Part> Map;

		/** Model */
		Index model;
		/** Frame */
		Mat34 frame;

		Part(Index model = INDEX_DEFAULT, Real weight = golem::numeric_const<Real>::ONE, Real cdf = -golem::numeric_const<Real>::ONE) :
			model(model), golem::Sample<Real>(weight, cdf)
		{}
		Part(Mat34& frame, Index model = INDEX_DEFAULT, Real weight = golem::numeric_const<Real>::ONE, Real cdf = -golem::numeric_const<Real>::ONE) :
			frame(frame), model(model), golem::Sample<Real>(weight, cdf)
		{}
	};

	/** Return complete realisation map. */
	virtual void getPart(Part::Map& partMap) const = 0;
	/** Return realisation map correlated with a given point index. */
	virtual void getPart(size_t index, Part::Map& partMap) const = 0;

	/** Query density frame covariance for a given realisation */
	virtual RBDist getFrameCovariance(Index realisation) const = 0;
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

namespace golem {
	template <> void Stream::read(golem::data::Feature3D::Feature& value) const;
	template <> void Stream::write(const golem::data::Feature3D::Feature& value);
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CONTACT_POINT_H_*/
