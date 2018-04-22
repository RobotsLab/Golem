/** @file Configuration.h
 *
 * Robot configuration model
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
#ifndef _GOLEM_CONTACT_CONFIGURATION_H_
#define _GOLEM_CONTACT_CONFIGURATION_H_

//------------------------------------------------------------------------------

#include <Golem/Contact/Manipulator.h>
#include <Golem/Sys/Stream.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Configuration model */
class Configuration {
public:
	typedef golem::shared_ptr<Configuration> Ptr;

	/** Trajectory parameter vector coordinates */
	typedef golem::_VecN<golem::Real, (size_t)golem::Configspace::DIM> ParamCoord;

	/** Weighted configuration */
	class Kernel : public Manipulator::Config, public golem::Sample<golem::Real> {
	public:
		/** Kernel density estimator */
		typedef std::vector<Kernel> Seq;
		/** Kernel density estimator */
		typedef std::vector<const Seq*> SeqPtrSeq;

		/** Default constructor */
		Kernel(golem::Real weight = golem::REAL_ONE, golem::Real cdf = -golem::REAL_ONE) : golem::Sample<golem::Real>(weight, cdf) {}
		/** Copy constructor */
		Kernel(const Manipulator::Config& config, golem::Real weight = golem::REAL_ONE, golem::Real cdf = -golem::REAL_ONE) : Manipulator::Config(config), golem::Sample<golem::Real>(weight, cdf) {}
		/** Copy constructor */
		Kernel(const golem::ConfigspaceCoord& config, const RBCoord& frame, golem::Real weight = golem::REAL_ONE, golem::Real cdf = -golem::REAL_ONE) : Manipulator::Config(config, frame), golem::Sample<golem::Real>(weight, cdf) {}
	};

	/** Sequence of waypoints */
	class Path : public Manipulator::Waypoint::Seq, public golem::Sample<golem::Real> {
	public:
		/** Collection of paths */
		typedef std::vector<Path> Seq;
		/** Collection of paths */
		typedef std::vector<const Path*> PtrSeq;

		/** Path appearance */
		class Appearance {
		public:
			/** Manipulator */
			Manipulator::Appearance manipulator;

			/** Show edges */
			bool showEdges;
			/** Show vertices */
			bool showVertices;

			/** Path segments */
			golem::U32 pathSegments;
			/** Path extrapolation distance delta */
			golem::Real pathDelta;
			/** Path colour */
			golem::RGBA pathColour;

			/** Current subspace distance */
			mutable golem::I32 subspaceDist;
			/** Current subspace Config */
			mutable golem::ConfigspaceCoord subspaceConfig;
			/** Current subspace frame */
			mutable golem::RBCoord subspaceFrame;
			/** Current subspace distance */
			mutable golem::Real subspaceDistLo, subspaceDistHi, subspaceDistVal;

			/** Constructs from description object */
			Appearance() {
				setToDefault();
			}
			/** Sets the parameters to the default values */
			void setToDefault() {
				manipulator.setToDefault();

				showEdges = true;
				showVertices = true;

				pathSegments = 20;
				pathDelta = golem::Real(0.0);
				pathColour = golem::RGBA::WHITE;

				subspaceDist = 0;
				subspaceConfig.fill(golem::REAL_ZERO);
				subspaceFrame.setId();
				subspaceDistLo = subspaceDistHi = subspaceDistVal = golem::REAL_ZERO;
			}
			/** Assert that the description is valid. */
			void assertValid(const Assert::Context& ac) const {
				manipulator.assertValid(Assert::Context(ac, "manipulator."));
				Assert::valid(pathDelta >= golem::REAL_ZERO, ac, "pathDelta: < 0");
				Assert::valid(pathSegments > 1, ac, "pathSegments: < 2");
			}
			/** Load descritpion from xml context. */
			void load(const golem::XMLContext* xmlcontext);
		};

		/** Kinematic parameter: vector parameterising trajectory "shape" */
		//ParamCoord param;

		/** Default constructor */
		Path(golem::Real weight = golem::REAL_ONE, golem::Real cdf = -golem::REAL_ONE) : golem::Sample<golem::Real>(weight, cdf) {}
		/** Copy constructor */
		Path(const Manipulator::Waypoint::Seq& path, golem::Real weight = golem::REAL_ONE, golem::Real cdf = -golem::REAL_ONE) : Manipulator::Waypoint::Seq(path), golem::Sample<golem::Real>(weight, cdf) {}

		/** Sets the parameters to the default values */
		inline void setToDefault() {
			clear();
		}
		/** Valid */
		inline void assertValid() {
			if (empty())
				throw golem::Message(golem::Message::LEVEL_ERROR, "Configuration::Path::assertValid(): empty path");
		}

		/** Grip */
		inline Manipulator::Waypoint& getGrip() {
			return back();
		}
		/** Grip */
		inline const Manipulator::Waypoint& getGrip() const {
			return back();
		}

		/** Approach */
		inline Manipulator::Waypoint& getApproach() {
			return front();
		}
		/** Approach */
		inline const Manipulator::Waypoint& getApproach() const {
			return front();
		}

		/** Comparison */
		friend bool equals(const Path& left, const Path& right, const golem::Configspace::Range& range, golem::Real eps = golem::REAL_EPS);

		/** Draw path */
		void draw(const Manipulator& manipulator, const Appearance& appearance, golem::DebugRenderer& renderer) const;
	};

	/** Configuration and trajectory sub-space */
	class Space {
	public:
		/** Collection */
		typedef std::vector<Space> Seq;

		/** Parameters */
		class Desc {
		public:
			/** Collection */
			typedef std::vector<Desc> Seq;

			/** Manipulator frame distribution standard deviation */
			golem::RBDist poseStdDev;
			/** Manipulator configuration distribution standard deviation */
			golem::ConfigspaceCoord configStdDev;

			/** Distance scale multiplier */
			golem::Real distanceScale;
			/** Distance standard deviation */
			golem::Real distanceStdDev;
			/** Maximum distance between query point and kernel */
			golem::Real distanceStdDevMax;

			/** Transformation gradient interpolation distance */
			golem::Real transformGradDist;

			/** Manipulator pose distribution covariance */
			golem::RBDist poseCov, poseCovInv;
			/** Manipulator configuration distribution covariance */
			golem::ConfigspaceCoord configCov, configCovInv;
			
			/** Maximum distance between query point and kernel */
			golem::Real distanceMax;

			/** Constructs object description */
			Desc() {
				Desc::setToDefault();
			}
			/** Create */
			void create(const Configuration& configuration);
			/** Sets the parameters to the default values */
			void setToDefault() {
				poseStdDev.set(golem::Real(0.002), golem::Real(1000.0));
				configStdDev.fill(golem::REAL_PI*golem::Real(0.01));
				
				distanceScale = golem::Real(0.02);
				distanceStdDev = golem::Real(2.0);
				distanceStdDevMax = golem::Real(5.0);
				
				transformGradDist = golem::Real(2.0);

				poseCov.set(Real(1.0), Real(1.0));
				poseCovInv.set(Real(1.0), Real(1.0));
				configCov.fill(Real(1.0));
				configCovInv.fill(Real(1.0));

				distanceMax = Real(1.0);
			}
			/** Assert that the description is valid. */
			void assertValid(const Assert::Context& ac) const {
				Assert::valid(poseStdDev.isValid(), ac, "poseStdDev: invalid");
				for (size_t i = 0; i < (size_t)golem::Configspace::DIM; ++i)
					Assert::valid(configStdDev.data()[i] > golem::REAL_EPS, ac, "configStdDev[i]: < eps");

				Assert::valid(distanceScale > golem::REAL_ZERO, ac, "distanceScale: <= 0");
				Assert::valid(distanceStdDev > golem::REAL_ZERO, ac, "distanceStdDev: <= 0");
				Assert::valid(distanceStdDevMax > golem::REAL_ZERO, ac, "distanceStdDevMax: <= 0");

				Assert::valid(transformGradDist > golem::REAL_ZERO, ac, "transformGradDist: <= 0");

				Assert::valid(poseCov.isPositive(), ac, "poseCov: <= 0");
				Assert::valid(poseCovInv.isPositive(), ac, "poseCovInv: <= 0");
			}
			/** Load descritpion from xml context. */
			void load(const golem::XMLContext* xmlcontext);
		};

		/** Header */
		class Header : public golem::Header {
		public:
			/** Version */
			static const Version VERSION;
			/** Version */
			virtual Version getVersion() const { return VERSION; }
			/** Id */
			static const std::string ID;
			/** Id */
			virtual const std::string& getId() const { return ID; }
		} header;

		/** Space name */
		std::string name;
		/** Paths */
		Configuration::Path::Seq paths;
		/** Configuration model (post-processing) */
		Configuration::Kernel::Seq configs;

		/** Manifold */
		ManifoldCtrl manifold;

		/** Parameters */
		Desc desc;

		/** Default initialisation */
		Space() {}
		/** Set name */
		Space(const std::string& name) : name(name) {}

		/** Comparison */
		friend bool equals(const Space& left, const Space& right, const golem::Configspace::Range& range, golem::Real eps = golem::REAL_EPS);
	};

	/** Configuration model description */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Sub-space description */
		Space::Desc spaceDesc;
		/** Sub-space description local override */
		bool spaceDescOverride;

		/** Path coords comparison eps */
		golem::Real pathEps;
		/** Number of kernels per path */
		golem::U32 kernels;

		/** Constructs object description */
		Desc() {
			Desc::setToDefault();
		}
		virtual ~Desc() {
		}
		/** Creates the object from the description. */
		virtual Configuration::Ptr create(Manipulator& manipulator) const {
			return Configuration::Ptr(new Configuration(*this, manipulator));
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			spaceDesc.setToDefault();
			spaceDescOverride = true;
			pathEps = golem::REAL_EPS;
			kernels = 1000;
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			spaceDesc.assertValid(Assert::Context(ac, "spaceDesc."));
			Assert::valid(pathEps >= golem::REAL_ZERO, ac, "pathEps: < 0");
			Assert::valid(kernels > 0, ac, "kernels: <= 0");
		}
		/** Load descritpion from xml context. */
		virtual void load(const golem::XMLContext* xmlcontext);
	};

	/** Create path from trajectory */
	virtual Path create(const golem::WaypointCtrl::Seq& waypoints) const;
	/** Create sub-space */
	virtual void create(const Path& path, Space& space) const;
	/** Create sub-spaces */
	virtual void create(Path::PtrSeq& paths, Space::Seq& spaces) const;
	/** Add sub-spaces */
	virtual void add(const Space::Seq& spaces);
	/** Clear all paths and clusters */
	virtual void clear();

	/** Configuration sampling: p(config) */
	virtual void sample(golem::Rand& rand, const Space& space, const ManifoldCtrl& manifold, Manipulator::Config& config) const;
	/** Path sampling: p(config|path) */
	virtual void sample(golem::Rand& rand, const Space& space, const Manipulator::Config& config, Path& path) const;
	/** Configuration sampling: p(mean, magnitude|config) */
	virtual void sample(golem::Rand& rand, const Space& space, const ManifoldCtrl& manifold, const Manipulator::Config& mean, const golem::RBDist& magnitude, Manipulator::Config& config) const;
	/** Conditional path generation: p(config, prev|next)	*/
	virtual void generate(golem::Rand& rand, const Space& space, const Manipulator::Config& config, const Path& prev, Path& next) const;
	/** Configuration evaluation */
	virtual golem::Real evaluate(const Space& space, const Manipulator::Config& config) const;

	/** Sub-spaces */
	const Space::Seq& getSpaces() const {
		return spaces;
	}
	/** Configuration description */
	const Desc& getDesc() const {
		return desc;
	}
	/** Manipulator */
	const Manipulator& getManipulator() const {
		return manipulator;
	}

	/** Object deletion */
	virtual ~Configuration();

protected:
	/** Context object */
	golem::Context &context;
	/** Manipulator */
	Manipulator& manipulator;

	/** Contact description */
	Desc desc;
	/** Sub-space collection */
	Space::Seq spaces;

	/** Configuration distance */
	virtual golem::Real distance(const Space::Desc& desc, const Manipulator::Config& left, const Manipulator::Config& right) const;
	/** Linear distance between given waypoints on a path */
	virtual golem::Real distancePath(const Space::Desc& desc, const Manipulator::Config& left, const Manipulator::Config& right) const;
	/** Transform path to the new pose */
	virtual void transform(const Space::Desc& desc, const Manipulator::Config& config, Path& path) const;

	/** Create kernels from path */
	virtual void create(const Path& path, Kernel::Seq& kernels) const;
	/** Normalise data */
	virtual void normalise();

	/** Creates Contact */
	Configuration(const Desc& desc, Manipulator& manipulator);
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

namespace golem {
	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::Configuration::Path& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::Configuration::Path& value);

	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::Configuration::Space::Seq::value_type& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::Configuration::Space::Seq::value_type& value);
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CONTACT_CONFIGURATION_H_*/
