/** @file Contact3D.h
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

#pragma once
#ifndef _GOLEM_CONTACT_CONTACT3D_H_
#define _GOLEM_CONTACT_CONTACT3D_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Point.h>
#include <Golem/Tools/Search.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Contact represents average point "seen" in the local frame of a given shape (bounds) */
class Contact3D : public golem::Sample<golem::Real> {
public:
	/** Collection */
	typedef std::vector<Contact3D> Seq;
	/** Collection */
	//typedef std::map<::golem::U32, Seq> Map;
	/** Collection */
	//typedef std::vector<Map::iterator> SeqMap;

	/** Default floating point */
	typedef golem::data::Point3D::Real Real;
	/** Default 3D vector */
	typedef golem::data::Point3D::Vec3 Vec3;
	/** Feature */
	typedef golem::data::Feature3D::Feature Feature;
	/** Default part index */
	typedef golem::data::Part3D::Index Index;

	/** Triangle */
	typedef golem::_Triangle<Real> Triangle;

	/** Contact type */
	enum Type {
		/** Point */
		TYPE_POINT = 0,
		/** Normal */
		TYPE_NORMAL,
		/** Feature */
		TYPE_FEATURE,
		/** Part */
		TYPE_PART,
		/** Size */
		TYPE_SIZE,
	};

	/** Contact type pointer */
	class TypePtr {
	public:
		/** Pointer collection */
		union {
			struct {
				/** Point */
				const data::Point3D* const points;
				/** Normal */
				const data::Normal3D* const normals;
				/** Feature */
				const data::Feature3D* const features;
				/** Part */
				const data::Part3D* const parts;
			};
			/** Part */
			data::Part3D* const ptr [TYPE_SIZE];
		};

		/** Create pointers */
		TypePtr(const data::Point3D* points) : points(points), normals(is<data::Normal3D>(points)), features(is<data::Feature3D>(points)), parts(is<data::Part3D>(points)) {}

		/** Access the data as an array. */
		inline data::Part3D* operator [] (Type type) {
			return ptr[size_t(type)];
		}
		/** Access the data as an array. */
		inline const data::Part3D* operator [] (Type type) const {
			return ptr[size_t(type)];
		}

		/** Select type-indexed record */
		template <typename _Seq> static typename _Seq::const_iterator select(const TypePtr& typePtr, const _Seq& seq) {
			for (typename _Seq::const_iterator i = seq.begin(); i != seq.end(); ++i)
				if (typePtr[i->first])
					return i;

			throw golem::Message(golem::Message::LEVEL_ERROR, "Contact3D::TypePtr::select(): unable to find required type");
		}
	};

	/** Contact type name string */
	static const char* typeName[TYPE_SIZE];

	/** Default/uninitialised index */
	static const Index INDEX_DEFAULT = golem::data::Part3D::INDEX_DEFAULT;

	/** Contact relation */
	enum Relation {
		/** None/Undefined */
		RELATION_NONE,
		/** Default/all types */
		RELATION_DFLT,
		/** Volume */
		RELATION_VOLUME,
		/** Surface */
		RELATION_SURFACE,
		/** Edge/line */
		RELATION_EDGE,
		/** Vertex/point */
		RELATION_VERTEX,
		/** Point */
		RELATION_POINT,
		/** Size */
		RELATION_SIZE,
	};

	/** Contact relation name string */
	static const char* relationName[RELATION_SIZE];
	/** Contact relation colour */
	static golem::RGBA relationColour[RELATION_SIZE];

	/** Feature3D Property */
	class Feature3DProperty : public data::Feature3D::Property {
	public:
		/** Description */
		class Desc {
		public:
			/** Std deviation multiplier */
			data::Feature3D::Feature stdDevFac;
			/** Power scaling */
			golem::Real powerScaling;

			/** Std deviation adaptation */
			bool adaptStdDevMinEnable;
			/** Std deviation adaptation */
			bool adaptStdDevOverlapEnable;
			/** Std deviation adaptation */
			bool adaptStdDevMeanEnable;

			/** Std deviation minimum value */
			golem::Real adaptStdDevMin;
			/** Std deviation maximum value */
			golem::Real adaptStdDevMax;
			/** Std deviation overlap */
			golem::Real adaptStdDevOverlap;
			/** Std deviation penalty */
			golem::Real adaptStdDevPenalty;

			/** Constructs description object */
			Desc() {
				setToDefault();
			}
			/** Sets the parameters to the default values. */
			void setToDefault() {
				stdDevFac.resize(data::Feature3D::FEATURE_SIZE);
				stdDevFac.fill(golem::Real(10.0));
				powerScaling = golem::REAL_ONE;

				adaptStdDevMinEnable = true;
				adaptStdDevOverlapEnable = true;
				adaptStdDevMeanEnable = true;

				adaptStdDevMin = golem::Real(1e-4);
				adaptStdDevMax = golem::Real(1e10);
				adaptStdDevOverlap = golem::Real(10.0);
				adaptStdDevPenalty = golem::Real(1.0);
			}
			/** Assert that the description is valid. */
			void assertValid(const Assert::Context& ac) const {
				for (size_t i = 0; i < stdDevFac.size(); ++i)
					Assert::valid(stdDevFac[i] > golem::REAL_EPS, ac, "stdDevFac[i]: < eps");
				Assert::valid(powerScaling > golem::REAL_EPS, ac, "powerScaling: < eps");

				Assert::valid(adaptStdDevMin > golem::REAL_EPS, ac, "adaptStdDevMin: < eps");
				Assert::valid(adaptStdDevMax > adaptStdDevMin, ac, "adaptStdDevMax: < adaptStdDevMin");
				Assert::valid(adaptStdDevOverlap > golem::REAL_EPS, ac, "adaptStdDevOverlap: < eps");
				Assert::valid(adaptStdDevPenalty >= golem::REAL_ZERO, ac, "adaptStdDevPenalty: < 0");
			}
			/** Load descritpion from xml context. */
			void load(const golem::XMLContext* xmlcontext);
		};

		/** Scale */
		data::Feature3D::Feature scale;
		/** Penalty */
		data::Feature3D::Real penalty;

		/** From standard deviation */
		void setStdDev(size_t j, data::Feature3D::Real stdDev);
		/** From standard deviation */
		void setStdDev(const data::Feature3D::Feature& stdDev);

		/** Clear */
		void clear() {
			mean.resize(0);
			covariance.resize(0);
			covarianceSqrt.resize(0);
			covarianceInv.resize(0);
			covarianceInvSqrt.resize(0);
			scale.resize(0);
			penalty = golem::numeric_const<data::Feature3D::Real>::ZERO;
		}
	};

	/** Data */
	class Data {
	public:
		/** Collection */
		typedef std::map<::golem::U32, Data> Map;
		/** Collection */
		typedef std::vector<Map::iterator> SeqMap;

		/** Description */
		class Desc {
		public:
			/** Feature3D properties description */
			Feature3DProperty::Desc feature3DDesc;

			/** Constructs description object */
			Desc() {
				setToDefault();
			}
			/** Sets the parameters to the default values. */
			void setToDefault() {
				feature3DDesc.setToDefault();
			}
			/** Assert that the description is valid. */
			void assertValid(const Assert::Context& ac) const {
				feature3DDesc.assertValid(Assert::Context(ac, "feature3DDesc."));
			}
			/** Load descritpion from xml context. */
			void load(const golem::XMLContext* xmlcontext);
		};

		/** Model type */
		golem::U32 type;
		/** Model data */
		Seq model;

		/** Model Feature3D properties */
		Feature3DProperty feature3DProperty;

		/** Assert that the data collection is valid. */
		static void assertValid(const Map& map) {
			// check data
			if (map.empty())
				throw golem::Message(golem::Message::LEVEL_ERROR, "Contact3D::Data::assertValid(): empty contact model data");
			for (Map::const_iterator i = map.begin(); i != map.end(); ++i)
				if (i->second.model.empty())
					throw golem::Message(golem::Message::LEVEL_ERROR, "Contact3D::Data::assertValid(): no model density provided");
		}

		/** Data type. */
		static Type getType(const Map& map) {
			return static_cast<Type>(map.begin()->second.type);
		}
		/** Feature dimensions. */
		static size_t getDimensions(const Map& map) {
			const size_t dimensions = map.begin()->second.model.front().feature.size();
			if (dimensions <= 0 || dimensions > data::Feature3D::FEATURE_SIZE)
				throw golem::Message(golem::Message::LEVEL_ERROR, "Contact3D::Data::getDimensions(): invalid model feature dimensions %u", golem::U32(dimensions));
			return dimensions;
		}

		/** Set to default */
		void clear() {
			type = TYPE_POINT;
			model.clear();
			feature3DProperty.clear();
		}

		/** Set to default */
		Data(golem::U32 type = TYPE_POINT) : type(type) {}
	};

	/** Appearance */
	class Appearance {
	public:
		/** Collection */
		typedef std::map<std::string, Appearance> Map;

		/** Show points */
		bool pointsShow;
		/** Show rays */
		bool raysShow;
		/** Show frames */
		bool framesShow;
		/** Contact colour */
		golem::RGBA colour[Contact3D::RELATION_SIZE];
		/** Contact point size */
		golem::Real pointSize;
		/** Contact frame axes size */
		golem::Vec3 frameSize;
		/** Show frame */
		bool boundsFrameShow;
		/** Frame axes size */
		golem::Vec3 boundsFrameSize;

		/** Contact relation */
		mutable Contact3D::Relation relation;

		/** Constructs description. */
		Appearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			pointsShow = true;
			raysShow = true;
			framesShow = true;
			std::copy(Contact3D::relationColour, Contact3D::relationColour + sizeof(Contact3D::relationColour) / sizeof(golem::RGBA), colour);
			pointSize = golem::Real(0.0);
			frameSize.set(golem::Real(0.002));
			boundsFrameShow = true;
			boundsFrameSize.set(golem::Real(0.01));

			relation = Contact3D::RELATION_NONE;
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(frameSize.isPositive(), ac, "frameSize: <= 0");
			Assert::valid(boundsFrameSize.isPositive(), ac, "boundsFrameSize: <= 0");
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Point/feature/part global frame: with feature/part orientation (quaternion x, y, z, w) or normal (quaternion x, y, z) */
	golem::RBCoord global;
	/** Point/feature/part local (inverse) frame: transformation frame - shape frame (in the feature/part body frame) */
	golem::RBCoord local;

	/** Feature vector (optional) */
	Feature feature;
	/** Part model (optional) */
	Index model;

	/** Surface distance, zero if intersects with shape (visualisation data) */
	golem::Real distance;
	/** Projection onto the nearest surface point in the local frame (visualisation data) */
	golem::Vec3 projection;
	/** Contact3D contact relation (visualisation data) */
	Relation relation;

	/** Converts bounds to triangles */
	static void convert(const golem::Bounds::Seq& bounds, Triangle::Seq& triangles);
	/** Find relation */
	static void find(const Triangle::Seq& triangles, const golem::Vec3& point, golem::Real& distance, golem::Vec3& projection, Relation& relation);

	/** Contact data processing */
	static void process(const Feature3DProperty::Desc& desc, Contact3D::Data::Map& dataMap);
	/** Contact data processing */
	static void process(const Data::Desc& desc, Contact3D::Data::Map& dataMap);

	/** Contact data processing */
	static void process(const Feature3DProperty::Desc& desc, const data::Feature3D& features, Contact3D::Data::Map& dataMap);
	/** Contact data processing */
	static void process(const Data::Desc& desc, const data::Point3D& points, Contact3D::Data::Map& dataMap);

	/** Draw contact at given pose */
	void draw(const Appearance& appearance, const golem::Mat34& pose, golem::DebugRenderer& renderer, golem::U8 a = golem::numeric_const<golem::U8>::MAX) const;
	/** Draw contacts at given pose */
	static void draw(const Appearance& appearance, const Seq& contacts, const golem::Mat34& pose, golem::DebugRenderer& renderer);
	/** Draw contact points at given pose */
	static void drawPoints(const Seq& contacts, const golem::RGBA& colour, golem::Real normalLen, const golem::Mat34& pose, golem::DebugRenderer& renderer);

	/** Reset weights */
	Contact3D(golem::Real weight = golem::REAL_ONE, golem::Real cdf = -golem::REAL_ONE) : golem::Sample<golem::Real>(weight, cdf), model(INDEX_DEFAULT) {}
};

/** Reads/writes object from/to a given XML context */
void XMLData(const std::string &attr, Contact3D::Type& val, golem::XMLContext* context, bool create = false);
void XMLData(Contact3D::Appearance::Map::value_type& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

/** Feature3D Kernel distance metric parameters */
template <typename _Real> class _Point3DKernelDist: public _RBCoordDist<_Real> {
public:
	using _RBCoordDist<_Real>::cov;
	using _RBCoordDist<_Real>::covSqrt;
	using _RBCoordDist<_Real>::covInv;
	using _RBCoordDist<_Real>::covSqrtInv;
	using _RBCoordDist<_Real>::distMax;
	using _RBCoordDist<_Real>::setCovInv;
	using _RBCoordDist<_Real>::setDistMax;

	/** floating point */
	typedef _Real Real;
	/** RBCoord */
	typedef golem::_RBCoord<Real> RBCoord;
	/** Feature */
	typedef golem::_VecN<Real, data::Feature3D::FEATURE_SIZE> Feature;

	/** Covariance inverse/distance (evaluation) */
	Feature featureCovInv;

	/** Distance maximum, covariance units (evaluation) */
	Real featureDist;
	/** Distance maximum, covariance units (evaluation) */
	Real featureDistMax;

	/** Penalty */
	Real distPenalty;
	/** Minimum number of cerrespondences (evaluation) */
	golem::U32 correspondencesMin;

	/** No initialisation */
	_Point3DKernelDist() {}
	/** Copying */
	_Point3DKernelDist(const _Point3DKernelDist<golem::F32>& dist) :
		_RBCoordDist<Real>(dist), featureCovInv(dist.featureCovInv.begin(), dist.featureCovInv.end()), featureDist((Real)dist.featureDist), featureDistMax((Real)dist.featureDistMax), distPenalty((Real)dist.distPenalty), correspondencesMin(dist.correspondencesMin) {}
	/** Copying */
	_Point3DKernelDist(const _Point3DKernelDist<golem::F64>& dist) :
		_RBCoordDist<Real>(dist), featureCovInv(dist.featureCovInv.begin(), dist.featureCovInv.end()), featureDist((Real)dist.featureDist), featureDistMax((Real)dist.featureDistMax), distPenalty((Real)dist.distPenalty), correspondencesMin(dist.correspondencesMin) {}

	/** From covariance inverse */
	inline void setFeatureCovInv(const Feature& featureCovInv) {
		this->featureCovInv = featureCovInv;
	}
	/** From distance */
	inline void setFeatureDist(Real featureDist, Real featureDistMax) {
		this->featureDist = featureDist;
		this->featureDistMax = featureDistMax;
	}

	/** Feature to feature maximum distance */
	inline Real distanceMax() const {
		return distPenalty*(covInv.dot(distMax) + featureDist*featureDistMax);
	}

	/** Sets the parameters to the default values. */
	void setToDefault() {
		setCovInv(RBDist(Real(1.0), Real(0.01)));
		setDistMax(RBDist(Real(0.005), Real(0.5)));

		featureDist = golem::Real(0.01);
		featureDistMax = golem::Real(5.0);
		
		distPenalty = golem::Real(1.0);
		correspondencesMin = 10;
	}
	/** Assert that the description is valid. */
	void assertValid(const Assert::Context& ac) const {
		Assert::valid(cov.isPositive(), ac, "cov: <= 0");
		Assert::valid(covInv.isPositive(), ac, "covInv: <= 0");
		Assert::valid(covSqrt.isPositive(), ac, "covSqrt: <= 0");
		Assert::valid(covSqrtInv.isPositive(), ac, "covSqrtInv: <= 0");
		Assert::valid(distMax.isPositive(), ac, "distMax: <= 0");

		Assert::valid(featureDist >= golem::REAL_ZERO, ac, "featureDist: < 0");
		Assert::valid(featureDistMax >= golem::REAL_ZERO, ac, "featureDistMax: < 0");

		Assert::valid(distPenalty >= golem::REAL_ZERO, ac, "distPenalty: < 0");
		Assert::valid(correspondencesMin > 0, ac, "correspondencesMin: 0");
	}
};

/** Feature3D Kernel distance metric parameters */
typedef _Point3DKernelDist<golem::Real> Point3DKernelDist;

void XMLData(Point3DKernelDist& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

/** Point3D Kernel */
template <typename _Real, typename _RealEval> class _Point3DKernel : public golem::Sample<_RealEval> {
public:
	/** floating point */
	typedef _Real Real;
	/** floating point */
	typedef _RealEval RealEval;
	/** Vec3 */
	typedef golem::_Vec3<Real> Vec3;
	/** Quat */
	typedef golem::_Quat<Real> Quat;
	/** Mat33 */
	typedef golem::_Mat33<Real> Mat33;
	/** Mat34 */
	typedef golem::_Mat34<Real> Mat34;
	/** RBCoord */
	typedef golem::_RBCoord<Real> RBCoord;
	/** RBDist */
	typedef golem::_RBDist<Real> RBDist;
	/** Feature */
	typedef golem::_VecN<Real, data::Feature3D::FEATURE_SIZE> Feature;
	/** Feature distance */
	typedef golem::_Point3DKernelDist<Real> Dist;

	/** Kernel collection */
	typedef std::vector<_Point3DKernel> Seq;
	/** Kernel collection pointer */
	typedef golem::shared_ptr<Seq> SeqPtr;
	/** Kernel collection */
	typedef std::vector<Seq> SeqSeq;
	/** Kernel collection pointer */
	typedef golem::shared_ptr<SeqSeq> SeqSeqPtr;

	/** NNSearch distance */
	struct NNDist {
		typedef bool is_kdtree_distance;
		typedef Real ElementType;
		typedef Real ResultType;
		static const size_t DIM = 3;

		template <typename _Iter1, typename _Iter2> Real operator() (_Iter1 a, _Iter2 b, size_t size = DIM, Real worst_dist = -golem::numeric_const<Real>::ONE) const {
			return Real((golem::Math::sqr(a[0] - b[0]) + golem::Math::sqr(a[1] - b[1]) + golem::Math::sqr(a[2] - b[2])));
		}
		inline Real accum_dist(const Real& a, const Real& b, int) const {
			return Real(golem::Math::sqr(a - b));
		}
	};
	/** NNSearch shared data */
	struct NNData {
		typedef std::vector<NNData> Seq;
		
		void resize(size_t size) {
			indices.resize(size);
			distances.resize(size);
		}
		size_t size() const {
			return std::min(indices.size(), distances.size());
		}

		NNSearch::IndexSeq indices;
		std::vector<Real> distances;
	};

	/** Point */
	Vec3 point;
	/** Normal */
	Vec3 normal;
	/** Feature */
	Feature feature;

	/** Index */
	size_t index;

	/** No initialisation */
	_Point3DKernel() {}
	/** Set contact from Contact3D */
	_Point3DKernel(const golem::Contact3D& contact, size_t index = -1, bool inverse = true) {
		set(contact, index, inverse);
	}
	/** Set contact from Point3D */
	_Point3DKernel(const data::Point3D& points, size_t index) {
		set(points, index);
	}

	/** Access to data. */
	inline Real* data() {
		return (Real*)&point.x;
	}
	/** Access to data. */
	inline const Real* data() const {
		return (const Real*)&point.x;
	}

	/** Set contact from Contact3D */
	void set(const golem::Contact3D& contact, size_t index = -1, bool inverse = true, bool local = true) {
		// index
		this->index = index;
		// weight
		this->weight = (RealEval)contact.weight;
		// it is a local feature frame, not inertial frame, get back to normal
		golem::RBCoord frame(local ? contact.local : contact.global);
		if (inverse) frame.setInverse(frame);
		// point
		this->point.set(frame.p);
		// normal
		golem::Vec3 normal;
		frame.q.multiply(normal, golem::Vec3(golem::REAL_ZERO, golem::REAL_ZERO, golem::REAL_ONE));
		this->normal.set(normal);
		// feature
		this->feature.assign(contact.feature.begin(), contact.feature.end());
	}
	/** Set contact from Point3D */
	void set(const data::Point3D& points, size_t index) {
		// index
		this->index = index;
		// point
		const data::Point3D::Point point = points.getPoint(index);
		// weight
		this->weight = (RealEval)point.weight;
		// point
		this->point.set((const data::Point3D::Vec3&)point);

		// features
		const data::Feature3D* features = is<const data::Feature3D>(&points);
		if (features) {
			data::Feature3D::Feature feature;
			data::Point3D::Mat33 orientation;
			features->getFeature(index, feature, orientation);
			// normal
			this->normal.set(golem::Mat33(orientation) * golem::Vec3(golem::REAL_ZERO, golem::REAL_ZERO, golem::REAL_ONE));
			// feature
			this->feature.assign(feature.begin(), feature.end());
		}
	}

	/** Transform */
	inline void transform(const RBCoord& trn) {
		trn.multiply(point, point);
		trn.q.multiply(normal, normal);
	}

	/** Distance computation: brute force */
	static RealEval distance(const Dist& dist, const RBCoord& trn, const Seq& a, const Seq& b, bool cutOff = false) {
		const Real distMax = dist.distanceMax();

		//golem::U32 m[3] = { 0 };
		golem::U32 correspondences = 0;
		RealEval distEval = golem::numeric_const<RealEval>::ZERO;
		for (typename Seq::const_iterator i = a.begin(); i != a.end(); ++i) {
			_Point3DKernel ka = *i;
			ka.transform(trn);

			Real distMin = distMax;
			
			for (typename Seq::const_iterator j = b.begin(); j != b.end(); ++j) {
				const _Point3DKernel kb = *j;

				const Real distLin = ka.point.distanceSqr(kb.point);
				if (distLin > dist.distMax.lin) {
					//++m[0];
					continue;
				}

				const Real distAng = golem::numeric_const<Real>::ONE - ka.normal.dot(kb.normal);
				if (distAng > dist.distMax.ang) {
					//++m[1];
					continue;
				}

				const Real distFeat = ka.feature.distanceWeightSqr(dist.featureCovInv, kb.feature);
				if (distFeat > dist.featureDistMax) {
					//++m[2];
					continue;
				}

				const Real distTest = dist.covInv.lin*distLin + dist.covInv.ang*distAng + dist.featureDist*distFeat;
				if (distMin > distTest)
					distMin = distTest;

				++correspondences;
			}
			
			distEval += RealEval(distMin);
		}

		//printf("distance(): p=%f/%u, n=%f/%u, f=%f/%u\n", golem::Real(m[0])/(a.size()*b.size()), m[0], m[0] > 0 ? golem::Real(m[1])/m[0] : golem::REAL_ZERO, m[1], m[0] > 0 ? golem::Real(m[2])/m[0] : golem::REAL_ZERO, m[2]);

		return correspondences < dist.correspondencesMin && cutOff ? golem::numeric_const<RealEval>::MAX : distEval / a.size();
	}

	/** Distance computation: nn-search (created for data set "b") */
	static RealEval distance(const Dist& dist, const NNSearch& nnSearch, const RBCoord& trn, const Seq& a, const Seq& b, NNData& data, bool cutOff = false) {
		const Real distMax = dist.distanceMax();

		//golem::U32 m[3] = { 0 };
		golem::U32 correspondences = 0;
		RealEval distEval = golem::numeric_const<RealEval>::ZERO;
		for (typename Seq::const_iterator i = a.begin(); i != a.end(); ++i) {
			_Point3DKernel ka = *i;
			ka.transform(trn);

			nnSearch.radiusSearch(ka.data(), 1, sizeof(_Point3DKernel), data.indices.size(), dist.distMax.lin / dist.covInv.lin, data.indices.data(), data.distances.data());

			Real distMin = distMax;
			for (size_t i = 0; i < data.indices.size(); ++i) {
				const int index = data.indices[i];
				if (index < 0)
					break;
				const _Point3DKernel kb = b[index];

				//++m[0];

				const Real distAng = golem::numeric_const<Real>::ONE - ka.normal.dot(kb.normal);
				if (distAng > dist.distMax.ang) {
					//++m[1];
					continue;
				}

				const Real distFeat = dist.featureDist > golem::numeric_const<Real>::ZERO ? ka.feature.distanceWeightSqr(dist.featureCovInv, kb.feature) : golem::numeric_const<Real>::ZERO;
				if (distFeat > dist.featureDistMax) {
					//++m[2];
					continue;
				}

				const Real distTest = dist.covInv.lin*data.distances[i] +dist.covInv.ang*distAng + dist.featureDist*distFeat;
				if (distMin > distTest)
					distMin = distTest;

				++correspondences;
			}

			distEval += RealEval(distMin);
		}

		//printf("distance(): p=%f/%u, n=%f/%u, f=%f/%u\n", golem::Real(m[0])/(a.size()*b.size()), m[0], m[0] > 0 ? golem::Real(m[1])/m[0] : golem::REAL_ZERO, m[1], m[0] > 0 ? golem::Real(m[2])/m[0] : golem::REAL_ZERO, m[2]);

		return correspondences < dist.correspondencesMin && cutOff ? golem::numeric_const<RealEval>::MAX : distEval / a.size();
	}
};

//------------------------------------------------------------------------------

/** Point3D Kernels */
typedef _Point3DKernel<golem::F32, golem::F64> Point3DKernel;

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

namespace golem {
	template <> void Stream::read(golem::Contact3D::Seq::value_type& value) const;
	template <> void Stream::write(const golem::Contact3D::Seq::value_type& value);

	template <> void Stream::read(golem::Contact3D::Feature3DProperty& value) const;
	template <> void Stream::write(const golem::Contact3D::Feature3DProperty& value);

	template <> void Stream::read(golem::Contact3D::Data& value) const;
	template <> void Stream::write(const golem::Contact3D::Data& value);

	template <> void Stream::read(golem::Contact3D::Data::Map::value_type& value) const;
	template <> void Stream::write(const golem::Contact3D::Data::Map::value_type& value);
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CONTACT_CONTACT3D_H_*/
