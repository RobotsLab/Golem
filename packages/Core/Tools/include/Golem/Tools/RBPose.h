/** @file RBPose.h
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
#ifndef _GOLEM_TOOLS_RBPOSE_H_
#define _GOLEM_TOOLS_RBPOSE_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Rand.h>
#include <Golem/Plugin/Renderer.h>
#include <Golem/Math/Triangle.h>
#include <Golem/Math/RBOptimisation.h>
#include <Golem/Tools/Defs.h>
#include <Golem/Tools/Search.h>
#include <Golem/Tools/Import.h>

//------------------------------------------------------------------------------

namespace pcl {
	struct PointXYZRGBNormal;
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Rigid body frame adjustment */
class RBAdjust {
public:
	/** Alignment increment */
	RBDist increment;

	/** Alignment linear transform keys */
	std::string linKeys;
	/** Alignment angular transform keys */
	std::string angKeys;
	/** Alignment increment keys */
	std::string incKeys;

	/** Alignment frame size */
	golem::Vec3 frameSize;
	/** Alignment colour solid */
	golem::RGBA colourSolid;
	/** Alignment colour wireframe */
	golem::RGBA colourWire;

	/** Set to default */
	RBAdjust() {
		setToDefault();
	}
	/** Sets the parameters to the default values */
	void setToDefault() {
		incrementStep = 0;
		increment.set(golem::Real(0.02), golem::REAL_PI*golem::Real(0.05));

		linKeys = "wsedrf";
		angKeys = "WSEDRF";
		incKeys = "-+";

		frameSize.set(golem::Real(0.1));
		colourSolid.set(golem::RGBA::GREEN._U8[0], golem::RGBA::GREEN._U8[1], golem::RGBA::GREEN._U8[2], golem::numeric_const<golem::U8>::MAX / 8);
		colourWire.set(golem::RGBA::GREEN);
	}
	/** Assert valid */
	void assertValid(const Assert::Context& ac) const {
		Assert::valid(increment.isValid(), ac, "increment: invalid");

		Assert::valid(frameSize.isPositive(), ac, "frameSize: <= 0");

		Assert::valid(linKeys.length() == 6, ac, "linKeys: length != 6");
		Assert::valid(angKeys.length() == 6, ac, "angKeys: length != 6");
		Assert::valid(incKeys.length() == 2, ac, "incKeys: length != 2");
	}
	/** Load descritpion from xml context. */
	void load(const golem::XMLContext* xmlcontext);

	/** Adjust frame */
	bool adjustIncrement(int key) const;

	/** Adjust frame */
	bool adjustFrame(int key, golem::Mat34& frame) const;

	/** Increment */
	RBDist getIncrement() const;

private:
	/** Increment step */
	mutable golem::U32 incrementStep;
};

//------------------------------------------------------------------------------

/** Pose distribution and estimation */
class RBPose {
public:
	/** Pointer */
	typedef golem::shared_ptr<RBPose> Ptr;

	typedef golem::Real Float;
	typedef golem::_Vec3<Float> Vector;
	typedef std::vector<Vector> VectorSeq;
	typedef golem::_Triangle<Float> Triangle;

	/** Points' pair feature (class members' alignment/packing is of sizeof(golem::Real) bytes) */
	class Feature {
	public:
		typedef std::vector<Feature> Seq;

		/** As real vector: two surface normals, distance between points, rgb colour */
		static const size_t N = 3 + 3 + 1 + 3 + 3;

		/** Feature distance metric */
		class FlannDist {
		public:
			typedef bool is_kdtree_distance;
			typedef golem::Real ElementType;
			typedef golem::Real ResultType;

			const bool prod;
			const golem::RBDist dist;
			const golem::Real distFeature;

			FlannDist(bool prod, const golem::RBDist& dist, const golem::Real& distFeature) : prod(prod), dist(dist), distFeature(distFeature) {
			}
			template <typename _Iter1, typename _Iter2> golem::Real operator() (_Iter1 a, _Iter2 b, size_t size = N, golem::Real worst_dist = -golem::REAL_ONE) const {
				const golem::Real d0 = golem::Math::sqr(a[6] - b[6]);
				const golem::Real d1 = golem::REAL_ONE - (a[0]*b[0] + a[1]*b[1] + a[2]*b[2]);
				const golem::Real d2 = golem::REAL_ONE - (a[3]*b[3] + a[4]*b[4] + a[5]*b[5]);
				const golem::Real d3 = golem::Math::sqr(a[7] - b[7]) + golem::Math::sqr(a[8] - b[8]) + golem::Math::sqr(a[9] - b[9]);
				const golem::Real d4 = golem::Math::sqr(a[10] - b[10]) + golem::Math::sqr(a[11] - b[11]) + golem::Math::sqr(a[12] - b[12]);
				
				return prod ? d0*d1*d2 : dist.lin*d0 + dist.ang*(d1 + d2) + distFeature*(d3 + d4);
			}
			inline golem::Real accum_dist(const golem::Real& a, const golem::Real& b, int) const {
				return golem::Math::sqr(a - b);
			}
		};

		/** Appearance */
		class Appearance {
		public:
			/** Line colour */
			golem::RGBA lineColour;

			/** Show normals */
			bool normalShow;
			/** Points' normals size */
			golem::Real normalSize;
			/** Normal colour */
			golem::RGBA normalColour;
			
			/** Show frame */
			bool frameShow;
			/** Frame axes size */
			golem::Vec3 frameSize;

			/** Constructs description. */
			Appearance() {
				setToDefault();
			}
			/** Sets the parameters to the default values. */
			void setToDefault() {
				lineColour = golem::RGBA::YELLOW;
				normalShow = true;
				normalSize = golem::Real(0.02);
				normalColour = golem::RGBA::CYAN;
				frameShow = true;
				frameSize.set(golem::Real(0.01));
			}
			/** Checks if the description is valid. */
			bool isValid() const {
				if (normalSize <= golem::REAL_ZERO || !frameSize.isPositive())
					return false;
				return true;
			}
		};

		/** Feature normals: points' normals in a local frame  */
		golem::Vec3 normal[2];
		/** Feature length: points' mutual distance */
		golem::Real length;
		/** Feature vector x, y, z in <0..1> */
		golem::Vec3 feature[2];

		/** Local frame */
		golem::Mat34 frame;
		/** Points' indices */
		size_t index[2];

		/** No member init by default */
		Feature() {}
		/** Access to data as a single vector */
		inline golem::Real* data() {
			return (golem::Real*)&normal[0];
		}
		/** Access to data as a single vector */
		inline const golem::Real* data() const {
			return (const golem::Real*)&normal[0];
		}
		/** Draw feature */
		void draw(const Appearance& appearance, const golem::Mat34& frame, golem::DebugRenderer& renderer) const;
		/** Draw feature */
		void draw(const Appearance& appearance, golem::DebugRenderer& renderer) const;
	};

	/** Rigid body sample */
	class Sample : public RBCoord, public golem::Sample<golem::Real> {
	public:
		typedef std::vector<Sample> Seq;

		/** Dereferencing template */
		struct Ref {
			template <typename _Ptr> static inline const golem::RBCoord& get(_Ptr& ptr) {
				return ptr; // nothing to do
			}
		};

		Sample() {}
		Sample(const RBCoord& c, golem::Real weight = golem::REAL_ONE, golem::Real cdf = -golem::REAL_ONE) : RBCoord(c), golem::Sample<golem::Real>(weight, cdf) {}
	};

	typedef golem::RBHeuristic<RBCoord, 1> Heuristic;
	typedef golem::DEOptimisation<Heuristic> Optimisation;

	/** Euler parametrisation */
	class Euler {
	public:
		/** Linear */
		golem::Vec3 lin;
		/** Angular */
		golem::Vec3 ang;

		/** Constructs object. */
		Euler() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			lin.set(golem::Real(0.0), golem::Real(0.0), golem::Real(0.0));
			ang.set(golem::Real(0.0), golem::Real(0.0), golem::Real(0.0));
		}
		/** Checks if the parameters are valid. */
		bool isValid() const {
			if (lin.isNegative() || ang.isNegative())
				return false;
			return true;
		}
	};

	/** Generator */
	class Generator : public golem::Sample<golem::Real> {
	public:
		typedef std::vector<Generator> Seq;

		/** Mean */
		Euler mean;
		/** Variance */
		Euler variance;

		/** Constructs generator. */
		Generator() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			golem::Sample<golem::Real>::setToDefault();
			mean.setToDefault();
			variance.lin.set(golem::Real(0.01));
			variance.ang.set(golem::Real(0.01));
		}
		/** Checks if the parameters are valid. */
		bool isValid() const {
			if (!golem::Math::isPositive(weight) || !mean.isValid() || !variance.isValid())
				return false;
			return true;
		}
	};

	/** Pose description */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** number of points */
		size_t points;
		/** Number of model features */
		size_t features;
		/** number of attempts */
		size_t attempts;
		/** Number of distribution kernels */
		size_t kernels;
		/** number of neighbours */
		size_t neighbours;
		/** Maximum distance between vector and kernel */
		golem::Real distanceRange;
		
		/** Kernel diagonal covariance scale metaparameter */
		RBCoord covariance;
		/** Standard deviation */
		RBDist poseStdDev;
		/** Feature normal epsilon */
		golem::Real featNormEps;
		/** Feature product */
		bool distProd;
		/** Rigid body distance */
		RBDist dist;
		/** Feature distance */
		golem::Real distFeature;

		/** KDTree decription*/
		KDTreeDesc nnSearchDesc;

		/** local alignment enabled */
		bool localEnabled;
		/** population size */
		size_t populationSize;
		/** number of generations min */
		size_t generationsMin;
		/** number of generations max */
		size_t generationsMax;
		/** distance difference threshold */
		golem::Real distanceDiff;

		/** Optimisation */
		Optimisation::Desc optimisationDesc;
		/** Max point model distance */
		golem::Real distanceMax;

		/** Rigid body frame adjustment */
		RBAdjust frameAdjust;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		
		/** Sets the parameters to the default values. */
		void setToDefault() {
			points = 100000;
			features = 500000;
			attempts = 10;
			kernels = 500000;
			neighbours = 10000;
			distanceRange = golem::Real(10.0);
			
			poseStdDev.set(golem::Real(0.01), golem::Real(100.0));
			std::fill(&covariance[0], &covariance[3], golem::Real(0.01)); // Vec3
			std::fill(&covariance[3], &covariance[7], golem::Real(0.01)); // Quat
			featNormEps = golem::Real(1e-7);
			distProd = true;
			dist.set(golem::Real(1.0), golem::Real(1.0));
			distFeature = golem::Real(0.0);
			
			nnSearchDesc.setToDefault();

			localEnabled = true;
			populationSize = 100;
			generationsMin = 10;
			generationsMax = 100;
			distanceDiff = golem::Real(1e-5);

			optimisationDesc.setToDefault();
			distanceMax = golem::Real(0.01);

			frameAdjust.setToDefault();
		}
		/** Assert that the object is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(points > 0, ac, "points: < 1");
			Assert::valid(features > 0, ac, "features: < 1");
			Assert::valid(attempts > 0, ac, "attempts: < 1");
			Assert::valid(kernels > 0, ac, "kernels: < 1");
			Assert::valid(neighbours > 0, ac, "neighbours: < 1");
			Assert::valid(neighbours <= kernels, ac, "neighbours: > kernels");
			Assert::valid(distanceRange > golem::REAL_ZERO, ac, "distanceRange: <= 0");

			for (size_t i = 0; i < RBCoord::N; ++i)
				Assert::valid(covariance[i] > golem::REAL_ZERO, ac, "covariance[]: <= 0");
			Assert::valid(poseStdDev.isValid(), ac, "poseStdDev: invalid");
			Assert::valid(dist.isValid(), ac, "dist: invalid");
			Assert::valid(distFeature >= golem::REAL_ZERO, ac, "distFeature: < 0");
			Assert::valid(!golem::Math::equals(featNormEps, golem::REAL_ZERO, golem::REAL_EPS), ac, "featNormEps: ~ 0");

			nnSearchDesc.assertValid(Assert::Context(ac, "nnSearchDesc."));
			
			Assert::valid(populationSize > 0, ac, "populationSize: < 1");
			Assert::valid(generationsMin > 0, ac, "generationsMin: < 1");
			Assert::valid(generationsMin <= generationsMax, ac, "generationsMin: > generationsMax");
			Assert::valid(distanceDiff >= golem::REAL_ZERO, ac, "distanceDiff: < 0");

			Assert::valid(optimisationDesc.isValid(), ac, "optimisationDesc: invalid");
			Assert::valid(!golem::Math::equals(distanceMax, golem::REAL_ZERO, golem::REAL_EPS), ac, "distanceMax: ~ 0");

			frameAdjust.assertValid(Assert::Context(ac, "frameAdjust."));
		}

		/** Creates the object from the description. */
		virtual RBPose::Ptr create(golem::Context &context) const;
	};

	/** Creates object frame */
	static golem::Mat34 createFrame(const Vec3Seq& points);
	
	/** Creates model features and search index from triangle mesh */
	template <typename _Seq> void createModel(const Vec3Seq& vertices, const TriangleSeq& triangles, _Seq& points) {
		Import import;
		import.size = (golem::U32)desc.points;
		import.generate(rand, vertices, triangles, [&] (const golem::Vec3& p, const golem::Vec3& n) {
			typename _Seq::PointType point;
			p.getColumn3(&point.x);
			n.getColumn3(&point.normal_x);
			points.push_back(point);
		});

		createModel(points);
		this->vertices = vertices;
		this->triangles = triangles;
	}
	/** Creates model features and search index fro point cloud */
	template <typename _Seq> void createModel(const _Seq& points) {
		context.debug("RBPose::createModel(): creating model features\n");
		createFeature(points, modelFeatures);
		createModel(modelFeatures);
		vertices.clear();
		triangles.clear();
	}
	
	/** Creates query object pose distribution */
	template <typename _Seq> void createQuery(const _Seq& points) {
		context.debug("RBPose::createQuery(): creating query features\n");
		createFeature(points, queryFeatures);
		createQuery(queryFeatures);
		this->points.clear();
		this->points.reserve(points.size());
		for (typename _Seq::const_iterator i = points.begin(); i != points.end(); ++i)
			this->points.push_back(Vector(i->x, i->y, i->z));
	}
	
	/** Sample pose from distribution */
	RBCoord sample() const;
	
	/** Distance between a and b */
	golem::Real distance(const RBCoord& a, const RBCoord& b) const;
	
	/** Kernel function */
	golem::Real kernel(golem::Real distance) const;
	
	/** Probability density value=p(s) for c */
	golem::Real density(const RBCoord &c) const;
	
	/** Global alignment */
	void alignGlobal(Sample& solution);
	/** Local alignment */
	void alignLocal(Sample& solution);

	/** Interactive alignment */
	template <typename _Seq> Sample align(const _Seq& points, DebugRendererCallback rendererCallback, golem::UIKeyboardMouseCallback* callback = nullptr) {
		Sample solution;
		do {
			createQuery(points);
			alignGlobal(solution);
		} while (!align(solution, rendererCallback, callback));
		return solution;
	}
	/** Interactive alignment */
	bool align(Sample& solution, DebugRendererCallback rendererCallback, golem::UIKeyboardMouseCallback* callback = nullptr);

	/** Maximum likelihood estimation */
	Sample maximum();

	/** Model features */
	const Feature::Seq& getModelFeatures() const {
		return modelFeatures;
	}
	/** Query features */
	const Feature::Seq& getQueryFeatures() const {
		return queryFeatures;
	}
	/** Nearest neighbour features indices */
	const I32Seq& getIndices() const {
		return indices;
	}
	/** Nearest neighbour features distances */
	const RealSeq& getDistances() const {
		return distances;
	}

	/** Transformation samples */
	const Sample::Seq& getSamples() const {
		return poses;
	}
	/** Transformation samples */
	Sample::Seq& getSamples() {
		return poses;
	}

	/** Points */
	const VectorSeq& getPoints() const {
		return points;
	}
	/** Vertices */
	const Vec3Seq& getVertices() const {
		return vertices;
	}
	/** Triangles */
	const TriangleSeq& getTriangles() const {
		return triangles;
	}

	/** Destruction of incomplete pointers */
	~RBPose();

protected:
	/** Context object */
	golem::Context &context;
	/** Generator of pseudo random numbers */
	golem::Rand rand;
	/** Pose description */
	Desc desc;

	/** Model features */
	Feature::Seq modelFeatures;
	/** Query features */
	Feature::Seq queryFeatures;
	/** Transformation samples */
	RBPose::Sample::Seq poses;
	/** Transformation samples properties */
	golem::SampleProperty<golem::Real, golem::RBCoord> pose;

	/** Feature nn search */
	NNSearch::Ptr nnSearch;
	/** Feature nearest neighbour features indices */
	NNSearch::IndexSeq indices;
	/** Feature nearest neighbour features distances */
	NNSearch::DistanceF64Seq distances;

	/** Pose distribution covariance */
	golem::RBDist poseCov, poseCovInv;

	/** Points */
	VectorSeq points;
	/** Vertices */
	Vec3Seq vertices;
	/** Triangles */
	TriangleSeq triangles;

	/** Creates model features and search index */
	void createModel(const Feature::Seq& modelFeatures);
	/** Creates query object pose distribution */
	void createQuery(const Feature::Seq& queryFeatures);

	/** Creates a sequence features */
	template <typename _Seq> void createFeature(const _Seq& points, Feature::Seq& features) const {
		if (points.size() < 2)
			throw golem::Message(golem::Message::LEVEL_ERROR, "RBPose::createFeature(): Insufficient number of points");
	
		size_t attempt = 0;
		for (Feature::Seq::iterator i = features.begin(); i != features.end();) {
			if (++attempt > desc.attempts)
				throw golem::Message(golem::Message::LEVEL_ERROR, "RBPose::createFeature(): Unable to generate features");
			
			const size_t pi1 = rand.next()%points.size();
			const size_t pi2 = (pi1 + 1 + rand.next()%(points.size() - 1))%points.size();
			if (!createFeature(points[pi1], points[pi2], *i))
				continue;

			i->index[0] = pi1;
			i->index[1] = pi2;
			++i;
			attempt = 0;
		}
	}

	/** Creates a single feature from pair of points */
	bool createFeature(const pcl::PointXYZRGBNormal& pp1, const pcl::PointXYZRGBNormal& pp2, Feature& feature) const;

	/** Returns the max weight associated to the corrent samples */
	golem::Real maxWeight() const;

	/** Creates/initialises the object */
	void create(const Desc& desc);
	/** Creates the object */
	RBPose(golem::Context& context);
};

void XMLData(RBPose::Feature::Appearance& val, golem::XMLContext* context, bool create = false);
void XMLData(RBPose::Euler& val, golem::XMLContext* context, bool create = false);
void XMLData(RBPose::Generator& val, golem::XMLContext* context, bool create = false);
void XMLData(RBPose::Desc& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace golem

namespace golem {
	template <> void Stream::read(golem::RBPose& rbPose) const;
	template <> void Stream::write(const golem::RBPose& rbPose);
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_TOOLS_RBPOSE_H_*/
