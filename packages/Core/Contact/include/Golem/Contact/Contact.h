/** @file Contact.h
 * 
 * Contact estimator
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
#ifndef _GOLEM_CONTACT_CONTACT_H_
#define _GOLEM_CONTACT_CONTACT_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Search.h>
#include <Golem/Tools/Cluster.h>
#include <Golem/Contact/Configuration.h>
#include <Golem/Contact/Query.h>
#include <Golem/Contact/Collision.h>
#include <Golem/Contact/User.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Contact estimator for a single contact type */
class Contact {
public:
	typedef golem::shared_ptr<Contact> Ptr;
	/** Contact estimator collection: contact estimator index -> contact estimator */
	typedef std::map<std::string, Ptr> Map;
	/** Query density container */
	typedef std::map<golem::U32, Query::Ptr> QueryMap;
	/** Sequence of Query densities */
	typedef std::vector<const Query*> QuerySeq;

	/** Query density selector map: contact index -> query density type */
	typedef std::map<golem::U32, std::string> SelectorMap;
	/** Query density selector type map: type -> query density selector */
	typedef std::map<std::string, SelectorMap> SelectorTypeMap;
	/** Default query density selector */
	static const std::string SelectorAny;

	/** Contact view */
	class View : public golem::Sample<golem::Real> {
	public:
		friend class Contact;
		/** Collection of views */
		typedef std::vector<View> Seq;
		/** Collection of views */
		typedef std::vector<View*> PtrSeq;
		/** Map space -> view */
		typedef std::map<U32, U32Seq> MapSeq;

		/** Contact model pointer */
		class ModelPtr {
		public:
			/** Contact model collection */
			typedef std::vector<ModelPtr> Seq;
			/** Contact model multimap with one or more models per link */
			typedef std::multimap<Manipulator::Link, ModelPtr> Map;

			/** Index */
			golem::U32 index;
			/** Weight */
			golem::Real weight;
			/** Local frame (see Aspect::processContacts(), Contact::create()) */
			golem::Mat34 frame;

			/** Default initialisation */
			ModelPtr(golem::U32 index = 0, golem::Real weight = golem::REAL_ONE, const golem::Mat34& frame = ::golem::Mat34::identity()) : index(index), weight(weight), frame(frame) {}
		};

		/** Query density pointer */
		class QueryPtr : public golem::Sample<golem::Real> {
		public:
			/** Query density collection */
			typedef std::vector<QueryPtr> Seq;

			/** Index */
			golem::U32 index;
			/** Link */
			Manipulator::Link link;
			/** Local frame (see OptimisationSA::evaluate()) */
			golem::Mat34 frame;
			/** Query index */
			const Query *query;

			/** Default initialisation */
			QueryPtr(golem::U32 index, const Manipulator::Link& link, const golem::Mat34& frame, const Query *query = nullptr, golem::Real weight = golem::REAL_ONE, golem::Real cdf = -golem::REAL_ONE) : golem::Sample<golem::Real>(weight, cdf), index(index), link(link), frame(frame), query(query) {}
		};

		/** view comparison operator, complete view first */
		struct compare_first {
			inline bool operator () (const View* left, const View* right) const {
				return left->getObjectIndex() < right->getObjectIndex() || left->getObjectIndex() == right->getObjectIndex() && left->hasCompleteView();
			}
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

		/** Object index (pre-processing) */
		::golem::I32 object;
		/** Space index */
		::golem::U32 space;

		/** Contact model map */
		ModelPtr::Map models;

		/** Manifold */
		ManifoldCtrl manifold;

		/** Point interface (processing) */
		const data::Point3D* point3D;

		/** Points (visualisation) */
		F32Vec3Seq points;

		/** Default initialisation */
		View(::golem::I32 object = 0, ::golem::U32 space = 0, golem::Real weight = golem::REAL_ONE, golem::Real cdf = -golem::REAL_ONE) : golem::Sample<golem::Real>(weight, cdf), object(object), space(space), point3D(nullptr) {}

		/** Query pointers */
		const QueryPtr::Seq& getQueryPtrSeq() const {
			return queryPtrSeq;
		}

		/** Object view index */
		golem::U32 getObjectIndex() const {
			return golem::Math::abs(object);
		}
		/** Complete view */
		bool hasCompleteView() const {
			return object < 0;
		}

	private:
		/** Query pointers (local) */
		QueryPtr::Seq queryPtrSeq;
	};

	/** Contact likelihood */
	class Likelihood {
	public:
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

		/** Link contacts likelihood */
		Manipulator::Link::RealCoord contacts;

		/** Configuration likelihood */
		golem::Real config;
		/** Collision component */
		golem::Real collision;
		/** User component (optional) */
		golem::Real user;

		/** Log-Likelihood value */
		golem::Real value;
		/** Log-Likelihood value active contacts */
		golem::Real valueActive;
		/** Log-Likelihood value inactive contacts */
		golem::Real valueInactive;

		/** Basic initialisation */
		Likelihood(golem::Real value = golem::REAL_ZERO) : value(value) {}

		/** Sets the parameters to the default values */
		inline void setToDefault(golem::Real likelihood = -golem::REAL_ONE, golem::Real value = golem::REAL_ZERO) {
			contacts.fill(likelihood);
			config = likelihood;
			collision = likelihood;
			user = golem::REAL_ZERO; // optional, no influence
			
			this->value = value;
			valueActive = valueInactive = value;
		}

		/** Is data valid */
		inline static bool isValid(const golem::Real& component, golem::Real eps) {			
			return component <= -golem::REAL_ONE || component - eps > golem::REAL_ZERO;
		}
		/** Is data valid */
		inline bool isValid(golem::Real eps) const {			
			for (size_t i = 0; i < Manipulator::Link::RealCoord::SIZE; ++i)
				if (!isValid(contacts[i], eps))
					return false;
			return isValid(config, eps);
		}

		/** Number of active contacts */
		inline size_t getActiveContacts() const {
			size_t activeContacts = 0;
			for (size_t i = 0; i < Manipulator::Link::RealCoord::SIZE; ++i) {
				if (contacts[i] > golem::REAL_ZERO)
					++activeContacts;
			}
			return activeContacts;
		}
		/** Average value of active contacts */
		inline golem::Real getActiveContactsAverageValue() const {
			golem::Real value = golem::REAL_ZERO;
			size_t activeContacts = 0;
			for (size_t i = 0; i < Manipulator::Link::RealCoord::SIZE; ++i) {
				const golem::Real contact = contacts[i];
				if (contact > golem::REAL_ZERO) {
					++activeContacts;
					value += contact;
				}
			}
			return activeContacts > 0 ? value/activeContacts : golem::REAL_ZERO;
		}

		/** Log value */
		inline static golem::Real getLogValue(golem::Real value) {
			return value < golem::REAL_ZERO ? golem::REAL_ZERO : golem::Math::ln(value > golem::REAL_ZERO ? value : golem::numeric_const<golem::Real>::MIN);
		}

		/** Log-likelihood product update */
		inline void makeLogProduct() {
			valueActive = golem::REAL_ZERO;
			for (size_t i = 0; i < Manipulator::Link::RealCoord::SIZE; ++i)
				valueActive += getLogValue(contacts[i]);
			
			value = golem::REAL_ZERO;
			value += valueActive;
			value += getLogValue(config);
			value += collision;
			value += user;
		}

		/** Log-likelihood normalised product update */
		inline void makeLogNormProduct(size_t activeContacts, golem::Real penaltyExp) {

			valueActive = golem::REAL_ZERO;
			valueInactive = golem::REAL_ZERO;
			size_t n = 0;

			for (size_t i = 0; i < Manipulator::Link::RealCoord::SIZE; ++i) {
				const golem::Real contact = contacts[i];
				valueActive += getLogValue(contact);
				if (contact > golem::REAL_ZERO) {
					valueInactive += contact;
					++n;
				}
			}

			valueInactive = n > 0 && activeContacts > n ? getLogValue(valueInactive/n)*(activeContacts - n)*penaltyExp : golem::REAL_ZERO;

			value = golem::REAL_ZERO;
			value += valueActive;
			value += valueInactive;
			value += getLogValue(config);
			value += collision;
			value += user;
		}
	};

	/** Contact configuration */
	class Config {
	public:
		/** Unique pointer for performance reasons */
		typedef std::shared_ptr<Config> Ptr;
		/** Collection */
		typedef std::vector<Ptr> Seq;
		/** Collection of pointers */
		typedef std::vector<Seq::const_iterator> SeqConstPtr;
		/** Config set */
		typedef std::set<Contact::Config::Seq::const_iterator> SetConstPtr;

		/** Collection range */
		typedef std::pair<Seq::iterator, Seq::iterator> Range;
		/** Collection range sequence */
		typedef std::vector<Range> RangeSeq;

		/** Contact estimator config collection: contact type name -> contact config */
		typedef std::multimap<std::string, Config> Map;
		/** Contact estimator config sequence collection: contact type name -> contact estimator config sequence */
		typedef std::map<std::string, Seq> SeqMap;

		/** Configuration cluster */
		class Cluster {
		public:
			/** Cluster collection */
			typedef std::vector<Cluster> Seq;

			/** Clustering type */
			enum Type {
				/** Disabled/no clustering */
				TYPE_DISABLED = 0,
				/** Type */
				TYPE_TYPE,
				/** likelihood */
				TYPE_LIKELIHOOD,
				/** configuration */
				TYPE_CONFIGURATION,
				/** size */
				TYPE_SIZE = TYPE_CONFIGURATION + 1,
			};

			/** Clustering name */
			static const std::string typeName[TYPE_SIZE];

			/** Clustering algorithm description */
			class Desc : public ClusteringDesc {
			public:
				/** Clustering search distance */
				RBDist distance;

				/** Debug level */
				golem::U32 debugLevel;

				/** Constructs description object */
				Desc() {
					Desc::setToDefault();
				}
				/** Sets the parameters to the default values */
				void setToDefault() {
					ClusteringDesc::setToDefault();

					distance.set(golem::Real(20.0), golem::Real(200.0));

					debugLevel = 0;
				}
				/** Assert that the description is valid. */
				void assertValid(const Assert::Context& ac) const {
					ClusteringDesc::assertValid(ac);
					
					Assert::valid(distance.isValid(), ac, "distance: invalid");
				}
				/** Load descritpion from xml context. */
				void load(const golem::XMLContext* xmlcontext);
			};

			/** Begin */
			golem::U32 begin;
			/** End */
			golem::U32 end;

			/** Construction */
			Cluster() {}
			/** Construction */
			Cluster(golem::U32 begin, golem::U32 end) : begin(begin), end(end) {}

			/** Cluster size */
			golem::U32 size() const {
				return end - begin;
			}
			/** Is empty */
			bool empty() const {
				return begin == 0 && end == 0;
			}
			/** Is valid */
			template <typename _Seq> bool valid(const _Seq& seq) const {
				return begin <= end && end <= (golem::U32)seq.size();
			}

			/** Get index */
			static golem::U32 getIndex(const Cluster::Seq& clusters, golem::I32& cluster, golem::I32& index) {
				if (clusters.empty())
					return golem::U32(cluster = index = 0);
				cluster = golem::Math::clamp(cluster, (golem::I32)0, (golem::I32)clusters.size() - 1);
				index = golem::Math::clamp(index, (golem::I32)0, (golem::I32)clusters[cluster].size() - 1);
				return clusters[cluster].begin + index;
			}

			/** Find contact clusters */
			static void find(Context& context, Type type, const Desc& desc, Config::Seq& configs, Cluster::Seq& clusters);
			/** Find contact type clusters */
			static void findType(Context& context, const Desc& desc, Config::Seq& configs, Cluster::Seq& clusters);
			/** Find contact likelihood clusters */
			static void findLikelihood(Context& context, const Desc& desc, Config::Seq& configs, Cluster::Seq& clusters);
			/** Find contact configuration clusters */
			static void findConfiguration(Context& context, const Desc& desc, Config::Seq& configs, Cluster::Seq& clusters);

			/** Type */
			static Type fromString(const std::string& str);

		protected:
			/** Find contact configuration clusters - OPTICS clustering */
			static void findConfigurationOPTICS(Context& context, const Desc& desc, Config::Seq& configs, Cluster::Seq& clusters);
			/** Find contact configuration clusters - affinity clustering */
			static void findConfigurationAffinity(Context& context, const Desc& desc, Config::Seq& configs, Cluster::Seq& clusters);
		};

		/** Config appearance */
		class Appearance {
		public:
			/** Path */
			bool showConfig;
			/** Config model */
			bool showModelConfig;
			/** Contact model */
			bool showModelContact;

			/** Config path appearance */
			Configuration::Path::Appearance configPath;
			/** Model manipulator */
			Manipulator::Appearance modelManipulator;
			/** Manifold appearance */
			ManifoldCtrl::Appearance manifold;

			/** Show frames, not points */
			bool showModelFrames;
			/** Model distribution num of samples */
			golem::U32 modelDistribSamples;
			/** Model distribution num of bounds */
			golem::U32 modelDistribBounds;
			/** Model selection index */
			golem::U32 modelSelectionIndex;
			/** Samples colour */
			golem::RGBA modelSamplesColour;
			/** Samples point size */
			golem::Real modelSamplesPointSize;
			/** Samples frame size */
			golem::Vec3 modelSamplesFrameSize;

			/** Constructs from description object */
			Appearance() {
				setToDefault();
			}
			/** Sets the parameters to the default values */
			void setToDefault() {
				showConfig = true;
				showModelConfig = false;
				showModelContact = false;
				
				configPath.setToDefault();
				manifold.setToDefault();
				modelManipulator.setToDefault();

				showModelFrames = false;
				modelDistribSamples = 1000;
				modelDistribBounds = 1;
				modelSelectionIndex = -1;
				modelSamplesColour = golem::RGBA(golem::RGBA::RED._rgba.r, golem::RGBA::RED._rgba.g, golem::RGBA::RED._rgba.b, 50);
				modelSamplesPointSize = golem::Real(3.0);
				modelSamplesFrameSize.set(golem::Real(0.01));
			}
			/** Assert that the description is valid. */
			void assertValid(const Assert::Context& ac) const {
				configPath.assertValid(Assert::Context(ac, "configPath."));
				manifold.assertValid(Assert::Context(ac, "manifold->"));
				modelManipulator.assertValid(Assert::Context(ac, "modelManipulator."));

				Assert::valid(modelSamplesPointSize > golem::REAL_ZERO, ac, "modelSamplesPointSize: < 0");
				Assert::valid(modelSamplesFrameSize.isPositive(), ac, "modelSamplesFrameSize: <= 0");
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

		/** Type */
		std::string type;
		/** View */
		golem::U32 view;
		/** Space */
		golem::U32 space;

		/** Manifold */
		ManifoldCtrl manifold;

		/** Path */
		Configuration::Path path;
		
		/** Likelihood */
		Likelihood likelihood;
		
		/** Basic initialisation */
		Config(golem::Real value = golem::REAL_ZERO) : view(0), space(0), likelihood(value), contact(nullptr) {}

		/** Allocates and copies parameters */
		static void alloc(Ptr& ptr) {
			if (!ptr) ptr.reset(new Config);
		}
		/** Allocates and copies parameters */
		inline void set(const Contact& contact, const Config& config) {
			this->contact = &contact;
			this->type = contact.getType();
			this->view = config.view;
			this->space = config.space;
			this->manifold = config.manifold;
			this->path = config.path;
			this->likelihood = config.likelihood;
		}

		/** Sets the parameters to the default values */
		inline void setToDefault() {
			contact = nullptr;
			type.clear();
			view = 0;
			space = 0;
			path.setToDefault();
			likelihood.setToDefault();
		}
		/** Valid */
		template <typename _Ptr> static void assertValid(const _Ptr& ptr) {
			if (!ptr || ptr->path.empty() || ptr->type.length() <= 0)
				throw golem::Message(golem::Message::LEVEL_ERROR, "Contact::Config::assertValid(): invalid configuration");
		}

		/** Size */
		template <typename _RangePtr> static inline size_t getSize(_RangePtr begin, _RangePtr end) {
			size_t size = 0;
			for (; begin != end; ++begin)
				size += (size_t)std::distance(begin->first, begin->second);
			return size;
		}
		/** Number of active contacts */
		template <typename _Range> static inline size_t getActiveContacts(const _Range& range) {
			size_t activeContacts = 0;
			for (typename _Range::first_type i = range.first; i != range.second; ++i) {
				const size_t ac = (*i)->likelihood.getActiveContacts();
				if (activeContacts < ac)
					activeContacts = ac;
			}
			return activeContacts;
		}
		/** Number of active contacts */
		template <typename _RangePtr> static inline size_t getActiveContacts(_RangePtr begin, _RangePtr end) {
			size_t activeContacts = 0;
			for (; begin != end; ++begin) {
				const size_t ac = getActiveContacts(*begin);
				if (activeContacts < ac)
					activeContacts = ac;
			}
			return activeContacts;
		}
		/** Log-likelihood normalised product update */
		template <typename _Range> static inline void makeLogNormProduct(const _Range& range, size_t activeContacts) {
			for (typename _Range::first_type i = range.first; i != range.second; ++i)
				(*i)->likelihood.makeLogNormProduct(activeContacts, (*i)->getContact() ? (*i)->getContact()->getDesc().penaltyExp : golem::REAL_ONE);
		}
		/** Log-likelihood normalised product update */
		template <typename _RangePtr> static inline void makeLogNormProduct(_RangePtr begin, _RangePtr end, size_t activeContacts) {
			for (; begin != end; ++begin)
				makeLogNormProduct(*begin, activeContacts);
		}

		/** Contact pointer */
		const Contact* getContact() const {
			return contact;
		}

		/** Draw contact config */
		void draw(const Manipulator& manipulator, const Appearance& appearance, golem::Rand& rand, golem::DebugRenderer& renderer) const;

	private:
		/** Contact pointer */
		const Contact* contact;
	};

	/** Manifold */
	class ManifoldSelector {
	public:
		/** Collection */
		typedef std::map<ManifoldSelector, ManifoldCtrl> Map;

		/** Type */
		std::string type;
		/** Space */
		golem::U32 space;
		/** View */
		golem::U32 view;

		/** Default type selector */
		static const std::string TypeAny;
		/** Default type selector */
		static const U32 SpaceAny = -1;
		/** Default type selector */
		static const U32 SpaceNone = -2;
		/** Default type selector */
		static const U32 ViewAny = -1;
		/** Default type selector */
		static const U32 ViewNone = -2;

		ManifoldSelector(const std::string& type = TypeAny, const U32 space = SpaceAny, const U32 view = ViewAny) : type(type), space(space), view(view) {}

		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);

		/** Comparator */
		inline friend bool operator < (const ManifoldSelector &left, const ManifoldSelector &right) {
			return left.type < right.type || left.type == right.type && left.space < right.space || left.type == right.type && left.space == right.space && left.view < right.view;
		}
	};

	/** Manifold description */
	class ManifoldDesc {
	public:
		/** Manifold distance norm/multiplier */
		golem::Twist norm;

		/** Constructs object description */
		ManifoldDesc() {
			ManifoldDesc::setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			norm.set(Vec3(golem::Real(10.0)), Vec3(golem::Real(1.0)));
		}
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			golem::Assert::valid(norm.isPositive(), ac, "norm: < eps");
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);

		/** Create */
		void create(const Manipulator& manipulator, const Contact::Config::SetConstPtr& configSet, Configuration::Path& path, ManifoldCtrl& manifold) const;
	};

	/** Contact configuration optimisation */
	class Optimisation {
	public:
		typedef golem::shared_ptr<Optimisation> Ptr;
		
		/** Optimisation description */
		class Desc {
		public:
			typedef golem::shared_ptr<Desc> Ptr;

			/** Nothing to do here */
			virtual ~Desc() {}
			/** Creates the object from the description. */
			virtual Optimisation::Ptr create(Contact& contact) const = 0;
			/** Sets the parameters to the default values */
			virtual void setToDefault() = 0;
			/** Assert that the description is valid. */
			virtual void assertValid(const Assert::Context& ac) const = 0;
			/** Load descritpion from xml context. */
			virtual void load(const golem::XMLContext* xmlcontext) = 0;
		};

		/** Optimisation: initialisation */
		virtual void create(const data::Point3D& points, golem::data::ContactQueryCallback::ContactEval* contactEval = nullptr) = 0;

		/** Optimisation: find initial solutions */
		virtual Config::Seq::iterator find(Config::Seq& configs, Config::Seq::iterator ptr) = 0;
		/** Optimisation: improve specified solutions [begin, end), using steps [beginStep, endStep], where: 0 <= beginStep < endStep <= 1 */
		virtual void find(Config::Seq::const_iterator begin, Config::Seq::const_iterator end, golem::U32 selectionStep, golem::Real beginStep, golem::Real endStep) = 0;
		
		/** Optimisation: evaluate config with optional collision expert */
		virtual void evaluate(const golem::Contact::Config& config, golem::Contact::Likelihood& likelihood, bool collisions = false) const = 0;

	protected:
		Contact& contact;
		golem::Context& context;
		
		Optimisation(Contact& contact) : contact(contact), context(contact.getContext()) {}
	};

	/** Contact description */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		/** Contact estimator description collection: contact estimator name -> contact estimator description */
		typedef std::map<std::string, Ptr> Map;
		
		/** Name */
		std::string name;
		/** Type */
		mutable std::string type;

		/** Weight */
		Real weight;

		/** Contact3D properties description */
		Contact3D::Data::Desc contact3DDesc;

		/** Query descriptions */
		Query::Desc::Map queryDescMap;

		/** Inactive contact penalty exponent */
		golem::Real penaltyExp;

		/** Configuration description */
		Configuration::Desc::Ptr configurationDesc;

		/** Optimisation description */
		Optimisation::Desc::Ptr optimisationDesc;
		/** Collision description */
		Collision::Desc::Ptr collisionDesc;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		virtual ~Desc() {
		}
		/** Creates the object from the description. */
		virtual Contact::Ptr create(Manipulator& manipulator) const {
			return Contact::Ptr(new Contact(*this, manipulator));
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			name = "Default";
			type = "Generic";
			weight = REAL_ONE;
			contact3DDesc.setToDefault();
			queryDescMap.insert(std::make_pair(Manipulator::Link::getName(Manipulator::Link::TYPE_ANY), Query::Desc::Ptr(new Query::Desc)));
			penaltyExp = golem::Real(1.0);
			configurationDesc.reset(new Configuration::Desc);
			optimisationDesc.reset();
			collisionDesc.reset(new Collision::Desc);
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Assert::valid(name.length() > 0, ac, "name: empty");
			Assert::valid(type.length() > 0, ac, "type: empty");
			Assert::valid(weight > REAL_EPS, ac, "weight: < eps");

			contact3DDesc.assertValid(Assert::Context(ac, "contact3DDesc."));

			for (Query::Desc::Map::const_iterator i = queryDescMap.begin(); i != queryDescMap.end(); ++i) {
				Assert::valid(i->second != nullptr, ac, "queryDescMap[]: null");
				i->second->assertValid(Assert::Context(ac, "queryDescMap[]->"));
			}
			Assert::valid(queryDescMap.find(SelectorAny) != queryDescMap.end(), ac, std::string("queryDescMap: missing " + SelectorAny + "query density").c_str());

			Assert::valid(golem::Math::isFinite(penaltyExp), ac, "penaltyExp: invalid");

			Assert::valid(configurationDesc != nullptr, ac, "configurationDesc: null");
			configurationDesc->assertValid(Assert::Context(ac, "configurationDesc->"));
			Assert::valid(optimisationDesc != nullptr, ac, "optimisationDesc: null");
			optimisationDesc->assertValid(Assert::Context(ac, "optimisationDesc->"));
			Assert::valid(collisionDesc != nullptr, ac, "collisionDesc: null");
			collisionDesc->assertValid(Assert::Context(ac, "collisionDesc->"));
		}
		/** Load descritpion from xml context. */
		virtual void load(const golem::XMLContext* xmlcontext);
	};

	/** Add training data */
	virtual void add(const golem::Contact3D::Data::Map& contacts, const golem::Contact::View::Seq& views, const golem::Configuration::Space::Seq& spaces, const SelectorMap& selectorMap);
	/** Clear training data */
	virtual void clear();
	/** Clear training data */
	virtual bool empty() const;

	/** Create query densities, initialise search */
	virtual void create(const data::Point3D& points, Point3DKernel::SeqPtr object = Point3DKernel::SeqPtr(), NNSearch::Ptr nnSearch = NNSearch::Ptr(), golem::data::ContactQueryCallback::ContactEval* contactEval = nullptr);

	/** Find initial solutions */
	virtual Config::Seq::iterator find(Config::Seq& configs, Config::Seq::iterator ptr);
	/** Improve specified solutions [begin, end), using steps [beginStep, endStep], where: 0 <= beginStep < endStep <= 1 */
	virtual void find(Config::Seq::const_iterator begin, Config::Seq::const_iterator end, golem::U32 selectionStep, golem::Real beginStep, golem::Real endStep);

	/** Sampling */
	virtual void sample(golem::Rand& rand, Config& config) const;

	/** Contact estimator name */
	const std::string& getName() const {
		return desc.name;
	}
	/** Contact estimator type */
	const std::string& getType() const {
		return desc.type;
	}

	/** Manipulator */
	const Manipulator& getManipulator() const {
		return manipulator;
	}

	/** Configuration model */
	const Configuration* getConfiguration() const {
		return configuration.get();
	}
	/** Views */
	const Contact::View::Seq& getViews() const {
		return views;
	}
	/** Query densities */
	const QuerySeq& getQuery() const {
		return querySeq;
	}
	/** Collision model */
	const Collision::Desc* getCollision() const {
		return desc.collisionDesc.get();
	}

	/** Optimisation */
	const Optimisation* getOptimisation() const {
		return optimisation.get();
	}

	/** Contact description */
	Desc& getDesc() {
		return desc;
	}
	const Desc& getDesc() const {
		return desc;
	}

	/** Normalisation constant */
	Real getNormalisation() const {
		return normalisation;
	}

	/** Query density selector map */
	const SelectorMap& getSelectorMap() const {
		return selectorMap;
	}

	golem::Context& getContext() {
		return context;
	}
	const golem::Context& getContext() const {
		return context;
	}

protected:
	/** Manipulator */
	Manipulator& manipulator;
	/** Context object */
	golem::Context &context;
	/** Contact description */
	Desc desc;

	/** Query density selector map */
	SelectorMap selectorMap;

	/** Views */
	Contact::View::Seq views;
	/** Contacts */
	Contact3D::Data::Map contacts;
	/** Query densities */
	QueryMap queryMap;
	/** Query densities */
	QuerySeq querySeq;

	/** Configuration model */
	Configuration::Ptr configuration;

	/** Optimisation */
	Optimisation::Ptr optimisation;

	/** Normalisation constant */
	Real normalisation;

	/** Creates Contact */
	Contact(const Desc& desc, Manipulator& manipulator);
};

void XMLData(const std::string &attr, golem::Contact::SelectorMap& val, golem::XMLContext* xmlcontext, bool create = false);
void XMLData(const std::string &attr, golem::Contact::Config::Cluster::Type& val, XMLContext* context, bool create = false);
void XMLData(golem::Contact::SelectorTypeMap::value_type& val, golem::XMLContext* xmlcontext, bool create = false);
void XMLData(golem::Contact::Desc::Map::value_type& val, golem::XMLContext* xmlcontext, bool create = false);
void XMLData(golem::Contact::ManifoldSelector& val, golem::XMLContext* xmlcontext, bool create = false);
void XMLData(golem::Contact::ManifoldSelector::Map::value_type& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

namespace golem {
	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::Contact::SelectorMap::value_type& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::Contact::SelectorMap::value_type& value);

	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::Contact::Likelihood& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::Contact::Likelihood& value);

	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::Contact::Config::Ptr& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::Contact::Config::Ptr& value);

	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::Contact::Config& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::Contact::Config& value);

	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::Contact::Config::Map::value_type& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::Contact::Config::Map::value_type& value);

	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::Contact::View::ModelPtr& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::Contact::View::ModelPtr& value);

	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::Contact::View::ModelPtr::Map::value_type& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::Contact::View::ModelPtr::Map::value_type& value);

	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::Contact::View::Seq::value_type& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::Contact::View::Seq::value_type& value);
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CONTACT_CONTACT_H_*/
